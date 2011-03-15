/*
*  inode.c
*  Currently implements all core mramfs functionality; at
*    ~2050 lines it is getting unwieldy and probably should
*    be refactored -- MM code was in separate file, but due
*    to GCC inlining complaints, it was merged.
*
* Current version 12 March 2004
* First release version 7 Dec 2003
*
* Nate Edel (nate@cse.ucsc.edu)
* Computer System Research Group
* University of California, Santa Cruz
*
* This file is released under the GPL.
*
* Includes some code from "ramfs.c" in the 2.4.22 Linux Kernel:
*    Copyright (C) 2000 Linus Torvalds / Transmeta Corp.
* and from "localfs.c" in LUFS, a free userspace filesystem implementation.
*    Copyright (C) 2002 Florin Malita <mali@go.ro>
*
*/

#include "mramfs.h"

/***********************************************************************
 *
 * Mem.c
 *
 * Routines for efficiently allocating and freeing small chunks of memory.
 *
 *
 ***********************************************************************
 */



/***********************************************************************
 *
 * ChunkSize
 *
 * Returns the size of a single chunk given the list number.
 *
 ***********************************************************************
 */
static
int
mfs_ChunkSize ( int sz ) {
  return ( 1 << ( sz + MEM_MINSIZE ) );
}

/***********************************************************************
 *
 * ChunkList
 *
 * Returns the list from which chunks of a desired size can be allocated.
 * Chunks on the list may be larger than the desired size, but are
 * guaranteed never to be smaller.
 *
 ***********************************************************************
 */
static
inline
int
mfs_ChunkList ( int desiredSize ) {
  int i;

  for ( i = 0; i < MEM_NLISTS; i++ ) {
    if ( desiredSize <= mfs_ChunkSize( i ) ) {
      break;
    }
  }
  if ( i >= MEM_NLISTS ) {
    return ( -1 );
  } else {
    return ( i );
  }
}

/***********************************************************************
 *
 * MemInit
 *
 * Externally visible routine to initialize the Mem routines.
 *
 ***********************************************************************
 */
void mfs_MemInit ( mfs_super *msb ) {
  int i;
  spin_lock(&msb->freelist_lock );

  for ( i = 0; i < MEM_NLISTS; i++ ) {
    msb->freeList[ i ].hdr = NULL;
    msb->freeList[ i ].nAllocated = msb->freeList[ i ].nFreed = msb->freeList[ i ].nCreated = 0;
  }
  spin_unlock(&msb->freelist_lock );
}

/***********************************************************************
 *
 * MemDumpStats
 *
 * Externally visible routine to print all memory allocation statistics.
 *
 ***********************************************************************
 */
void
mfs_MemDumpStats ( mfs_super * msb ) {
  int i;

  printk ( KERN_NOTICE "%6s %7s %7s %7s\n", "size", "created", "alloced", "freed" );
  for ( i = 0; i < MEM_NLISTS; i++ ) {
    printk ( KERN_NOTICE "%6d %7d %7d %7d\n", mfs_ChunkSize ( i ), msb->freeList[ i ].nCreated,
             msb->freeList[ i ].nAllocated, msb->freeList[ i ].nFreed );
  }
}

/***********************************************************************
 *
 * MemAllocate
 *
 * This function allocates a chunk of size at least that requested by
 * the call.  By using powers of two, we can handle many different chunk
 * sizes relatively efficiently.  Similar code can be used to handle
 * allocation of a large number of a particularly-sized structure.
 *
 ***********************************************************************
 */
handle mfs_MemAllocate ( mfs_super *msb, int chunkSize ) {
  handle p;
  int i, sz;
  spin_lock(&msb->freelist_lock );
  
  sz = mfs_ChunkList ( chunkSize );
  if ( sz < 0 ) {
    spin_unlock(&msb->freelist_lock );
    return ( MEM_NULL );
  }
  if ( msb->freeList[ sz ].hdr == MEM_NULL ) {
    p = mfs_saAllocate( msb );
    if ( p == NULL ) {
      printk ( KERN_ERR "FATAL: out of memory!\n" );
      spin_unlock(&msb->freelist_lock );
      return NULL;
    }
    msb->freeList[ sz ].hdr = p;
    for ( i = 1; i < ( ( 1 << SEGMENTBITS ) / mfs_ChunkSize( sz ) ); i++ ) {
      mfs_putHandle( msb, ( iHandle ) p, p + mfs_ChunkSize ( sz ) );
      p += mfs_ChunkSize ( sz );
    }
    mfs_putHandle( msb, ( iHandle ) p, NULL );
    msb->freeList[ sz ].nCreated += ( 1 << SEGMENTBITS ) / mfs_ChunkSize ( sz );
  }
  TRACE( "sz=%6d cr=%7d all=%7d fr=%7d\n", mfs_ChunkSize ( sz ), msb->freeList[ sz ].nCreated,
         msb->freeList[ sz ].nAllocated, msb->freeList[ sz ].nFreed );
  p = msb->freeList[ sz ].hdr;
  msb->freeList[ sz ].hdr = mfs_getHandle( msb, ( iHandle ) p );
  msb->freeList[ sz ].nAllocated++;
  spin_unlock(&msb->freelist_lock );
  mfs_MemSet( msb, p, 0, mfs_ChunkSize( sz ) );
  return ( p );
}

/***********************************************************************
 *
 * MemFree
 *
 * Externally visible routine to free a chunk.  Unlike malloc/free,
 * the caller must provide the size of the chunk.  This need not be
 * the "true" size, but can instead be the amount of memory requested
 * from the MemAllocate() call.
 *
 ***********************************************************************
 */
int
mfs_MemFree ( mfs_super *msb, int chunkSize, handle chunk ) {
  int sz = mfs_ChunkList ( chunkSize );
  spin_lock(&msb->freelist_lock );
  if ( sz < 0 ) {
    printk ( KERN_ERR "FATAL: trying to free a big chunk (size=%d,list=%d,!\n", chunkSize, sz );
    spin_unlock(&msb->freelist_lock );
    return -1;
  }
  mfs_putHandle( msb, ( iHandle ) chunk, msb->freeList[ sz ].hdr );
  msb->freeList[ sz ].hdr = chunk;
  msb->freeList[ sz ].nFreed++;
  spin_unlock(&msb->freelist_lock );
  return 0;
}


/***********************************************************************
 *
 * MemGet
 * MemPut
 *
 * These are largely dummy routines that transfer information given
 * an address and a size.  They're here so that the use of a non-standard
 * technology (MRAM, flash, etc.) can be handled directly or simulated
 * using standard main memory-based technologies.
 * MemGet: "size" bytes pointed to by "region" are copied to "buffer".
 * MemPut: "size" bytes pointed to by "buffer" are copied to "region".
 *
 ***********************************************************************
 */
inline int
mfs_MemGet ( mfs_super *msb, const handle region, void *buffer, int size ) {
  // todo: delay
  memcpy ( buffer, ( msb->sa_region + ( int ) region ), size );
  return ( 1 );
}

inline int
mfs_MemMove ( mfs_super *msb, handle dest, const handle src, int size ) {
#ifdef DEBUG
  if ( size < 1 ) {
    printk( KERN_ERR "Danger! mfs_MemMove called with size %d < 1!\n", size );
    return 0;
  }
#endif
  TRACE( " mfs_MemMove called : moving %d bytes from %08x to %08x\n", size, ( unsigned ) src, ( unsigned ) dest );
  memmove( msb->sa_region + ( int ) dest, msb->sa_region + ( int ) src, size );
  return 1;
}

inline int mfs_MemSet( mfs_super *msb, const handle region, unsigned char c, int count ) {
  // todo: delay
  memset ( ( msb->sa_region + ( int ) region ), c, count );
  return ( 1 );

}

inline int
mfs_MemPut ( mfs_super *msb, handle region, const void *buffer, int size ) {
  // todo: delay
  memcpy ( ( msb->sa_region + ( int ) region ), buffer, size );
  return ( 1 );
}

inline unsigned char mfs_getByte( mfs_super *msb, handle offset ) {
  unsigned char c = 0;
  mfs_MemGet( msb, offset, &c, sizeof( unsigned char ) );
  return c;
}

inline short mfs_getShort( mfs_super *msb, handle offset ) {
  short s = 0;
  mfs_MemGet( msb, offset, &s, sizeof( short ) );
  return s;
}

inline unsigned short mfs_getUShort( mfs_super *msb, handle offset ) {
  unsigned short s = 0;
  mfs_MemGet( msb, offset, &s, sizeof( unsigned short ) );
  return s;
}

inline int mfs_getInt( mfs_super *msb, handle offset ) {
  int i = 0;
  mfs_MemGet( msb, offset, &i, sizeof( int ) );
  return i;
}

inline unsigned int mfs_getUInt( mfs_super *msb, handle offset ) {
  unsigned int i = 0;
  mfs_MemGet( msb, offset, &i, sizeof( unsigned int ) );
  return i;
}

inline long mfs_getLong( mfs_super *msb, handle offset ) {
  long l = 0;
  mfs_MemGet( msb, offset, &l, sizeof( long ) );
  return l;
}

inline unsigned long mfs_getULong( mfs_super *msb, handle offset ) {
  unsigned long l = 0;
  mfs_MemGet( msb, offset, &l, sizeof( unsigned long ) );
  return l;
}

inline handle mfs_getHandle( mfs_super *msb, iHandle offset ) {
  handle h = NULL;
  mfs_MemGet( msb, ( handle ) offset, &h, sizeof( handle ) );
  return h;
}

inline handle mfs_getHandleIndexed( mfs_super *msb, iHandle offset, int index ) {
  handle h = NULL;
  mfs_MemGet( msb, ( handle ) ( ( handle ) offset + ( index * sizeof( handle ) ) ), &h, sizeof( handle ) );
  return h;
}

inline int mfs_putHandleIndexed( mfs_super *msb, iHandle offset, int index, handle h ) {
  mfs_MemPut( msb, ( handle ) ( ( handle ) offset + ( index * sizeof( handle ) ) ), &h, sizeof( handle ) );
  return 1;
}

inline int mfs_putHandle( mfs_super *msb, iHandle offset, handle h ) {
  mfs_MemPut( msb, ( handle ) offset, &h, sizeof( handle ) );
  return 1;
}

inline int mfs_putByte( mfs_super *msb, handle offset, unsigned char c ) {
  mfs_MemPut( msb, offset, &c, sizeof( unsigned char ) );
  return 1;
}

inline int mfs_putShort( mfs_super *msb, handle offset, short s ) {
  mfs_MemPut( msb, offset, &s, sizeof( short ) );
  return 1;
}

inline int mfs_putUShort( mfs_super *msb, handle offset, unsigned short s ) {
  mfs_MemPut( msb, offset, &s, sizeof( unsigned short ) );
  return 1;
}

inline int mfs_putInt( mfs_super *msb, handle offset, int i ) {
  mfs_MemPut( msb, offset, &i, sizeof( int ) );
  return 1;
}

inline int mfs_putUInt( mfs_super *msb, handle offset, unsigned int i ) {
  mfs_MemPut( msb, offset, &i, sizeof( unsigned int ) );
  return 1;
}

inline int mfs_putLong( mfs_super *msb, handle offset, long l ) {
  mfs_MemPut( msb, offset, &l, sizeof( long ) );
  return 1;
}

inline int mfs_putULong( mfs_super *msb, handle offset, unsigned long l ) {
  mfs_MemPut( msb, offset, &l, sizeof( unsigned long ) );
  return 1;
}



//
// segment allocator, allocates 64k blocks
// ("segments" after classic 8086/8
//

void * mfs_saAllocateRegion( int mb ) {
  void * region;
  long size = mb * MEG;
  if ( mb < 4 ) {
    TRACE( "mfs_saAllocateRegion called without a size or with size < 4mb.\n" );
    return NULL;
  }
  if ( mb > 1024 ) {
    TRACE( "mfs_saAllocateRegion called with a size > 1 gb\n" );
    return NULL;
  }
  region = __vmalloc( size, GFP_NOIO, PAGE_KERNEL );
  if ( region == NULL ) {
    TRACE( "mfs_saAllocateRegion failed to allocate memory\n" );
    return NULL;
  }
  TRACE( "mfs_saAllocateRegion allocated %d mb @ %08x\n", mb, ( unsigned int ) region );
  memset( region, 0, 65536 );
  return region;
}

inline int mfs_saMarkUsed( mfs_super *msb, int block ) {
  if ( block < 0 || block > 32767 ) {
    TRACE( "saMarkUsed called w/ illegal block\n" );
    return -1;
  }
  mfs_putByte( msb, ( handle ) ( block + 32768 ), 1 );
  return 0;
}

inline int mfs_saMarkUnused( mfs_super *msb, int block ) {
  if ( block < 0 || block > 32767 ) {
    TRACE( "saMarkUnused called w/ illegal block\n" );
    return -1;
  }
  mfs_putByte( msb, ( handle ) ( block + 32768 ), 0 );
  return 0;
}


inline int mfs_saIsUsed( mfs_super *msb, int block ) {
  if ( block < 0 || block > 32767 ) {
    TRACE( "saMarkUnused called w/ illegal block\n" );
    return -1;
  }
  return mfs_getByte( msb, ( handle ) ( block + 32768 ) );
}

int mfs_saInit( mfs_super *msb ) {
  //  unsigned char * bytes = ( ( unsigned char * ) msb->sa_region + 32768 );
  TRACE( "Region = %08x, bytes=%08x\n", ( unsigned int ) msb->sa_region, ( unsigned int ) bytes );
  msb->sa_size = msb->sa_size_mb * MEG;          // size in bytes
  msb->sa_blocks = msb->sa_size_mb * 16;      // blocks
  msb->sa_freeblocks = msb->sa_blocks - 1; // free blocks
  mfs_saMarkUsed( msb, 0 );
  return 0;
}

inline handle mfs_saAllocate( mfs_super *msb ) {
  handle chkpointer = ( handle ) 32768;
  int i = 1;

  if ( msb->sa_freeblocks < 1 )
    return NULL;

  spin_lock(&msb->segment_lock);
  while ( i < msb->sa_blocks ) {
    if ( mfs_getByte( msb, chkpointer + i ) == 0 )
      break;
    i++;
  }
  if ( i >= msb->sa_blocks ) {
    spin_unlock(&msb->segment_lock);
    return NULL;
  } else {
    mfs_putByte( msb, chkpointer + i, 1 );
    msb->sa_freeblocks--;
    spin_unlock(&msb->segment_lock );
    return ( handle ) ( i << 16 );
  }
}

inline int mfs_saFree( mfs_super *msb, handle to_free ) {
  int i;
  long tofree = ( long ) to_free;
  int block_num = ( tofree >> 16 );

  if ( ( tofree & 0xFFFF ) != 0 ) {
    TRACE( "saFree not called on a segment.\n" );
    return -1;
  }

  if ( !mfs_saIsUsed( msb, block_num ) ) {
    TRACE( "saFree called on an unallocated segment.\n" );
    return -1;
  }
  
  spin_lock(&msb->segment_lock );
  i = mfs_saMarkUnused( msb, block_num );
  spin_unlock(&msb->segment_lock );
  return i;
}

