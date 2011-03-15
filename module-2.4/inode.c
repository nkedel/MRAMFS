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

#define  REAL
#include "mramfs.h"

#ifdef MODULE
static unsigned long region_mb = 128;
static unsigned long do_compress_inodes = 1;
static unsigned long do_compress_blocks = 0;
MODULE_PARM( region_mb, "l" );
MODULE_PARM_DESC( region_mb, "Buffer size, in MB" );
MODULE_PARM( do_compress_inodes, "l" );
MODULE_PARM_DESC( do_compress_inodes, "Compress Inodes (<=0 - no, >=1 yes" );
MODULE_PARM( do_compress_blocks, "l" );
MODULE_PARM_DESC( do_compress_blocks, "Compress Blocks (<=0 - no, 1-9 - yes, will use for gzip compression level" );
#define MRAMFS_REGION_MB (region_mb)
#define MRAMFS_COMPRESS_INODES (do_compress_inodes)
#define MRAMFS_COMPRESS_BLOCKS (do_compress_blocks)
#else
#define MRAMFS_REGION_MB (128)
#define MRAMFS_COMPRESS_INODES (1)
#define MRAMFS_COMPRESS_BLOCKS (0)
#endif


/* some random number */
#define MRAMFS_MAGIC    0x1024EDE7

static void *global_region = NULL;

static struct super_operations mramfs_ops;
static struct address_space_operations mramfs_aops;
static struct file_operations mramfs_file_operations;
static struct file_operations mramfs_dir_file_operations;
static struct inode_operations mramfs_file_inode_operations;
static struct inode_operations mramfs_dir_inode_operations;
static struct inode_operations mramfs_symlink_operations;

static compressor_list compressors;

// simple helper to return our context
inline mfs_super *mramfs_getSB( struct super_block *sb ) {
  //       TRACE("mramfs_getSB: sb@ %08x / ctx@ %08x\n", (unsigned int) sb,(unsigned int) sb->u.generic_sbp);
  if ( sb == NULL ) {
    printk( KERN_ERR "mramfs_getSB called on NULL superblock.\n" );
    return NULL;
  }
  return sb->u.generic_sbp;
}

int find_free_in_block( mfs_super *con, handle blk ) {
  unsigned char lens[ 17 ];
  mfs_MemGet( con, blk, lens, 16 );
  int i = 0;
  while ( i < 16 ) {
    if ( lens[ i ] == 0 )
      return i;
    i++;
  }
  return -1;
}

/*
 * Finds an unllocated inode
 * TODO: Find one with i_nlink = 0;
 * TODO: We really should use a bitmap.  This isn't that bad, though.
 */
unsigned long find_free_inode( struct super_block *sb, unsigned long starting_from ) {
  mfs_super *con = mramfs_getSB( sb );
  //  int blk_offset = starting_from & 15;            // hardcoded may be bad here
  int blk_nr = ( starting_from >> 4 ) & 1023;
  int tab_nr = starting_from >> 14;
  // assert (tab_nr < 1024);
  iHandle blk_table;
  handle blk;
  TRACE( "find_free_inode, starting from == %lu (%d/%d/%d)\n", starting_from, tab_nr, blk_nr, blk_offset );

  while ( tab_nr < con->inode_table_count ) {
    blk_table = ( iHandle ) mfs_getHandleIndexed( con, con->inodes, tab_nr );
    //    TRACE("Testing table #%d\n", tab_nr);
    if ( blk_table == NULL )
      return ( tab_nr * con->inode_table_size * 16 );

    while ( blk_nr < con->inode_table_size ) {
      blk = mfs_getHandleIndexed( con, blk_table, blk_nr );
      if ( blk == NULL ) {
        //        TRACE("Testing table #%d / block #%d == NULL", tab_nr, blk_nr);
        return ( tab_nr * con->inode_table_size * 16 ) + ( blk_nr * 16 );
      }
      if ( ( ( ( unsigned long ) blk ) & 0xF ) > 0 ) {
        int offset = find_free_in_block( con, ( handle ) ( ( unsigned long ) blk & BLOCK_ADDR_MASK ) );
        if ( offset >= 0 )
          return ( tab_nr * con->inode_table_size * 16 ) + ( blk_nr * 16 ) + offset;
        // we might want to mark that we were wrong about free inodes.
      }
      blk_nr++;
    }
    blk_nr = 0;
    tab_nr++;
  }
  return 0;
}

handle mramfs_insert_inode( mfs_super *msb, handle blk, int offset, unsigned char *buffer, int len ) {
  handle new_blk;
  int new_len = 0;
  int live_count = 0;
  int bytes_before = 0;
  int bytes_after = 0;
  int scan_count = 0;
  int old_len = 0;
  spin_lock( msb->cbuf_inode_lock );

  if ( blk )
    mfs_MemGet( msb, blk, msb->cbuf_inode, sizeof( i_block ) );
  else
    memset( msb->cbuf_inode, 0, sizeof( i_block ) );
  old_len = msb->cbuf_inode->length;

  while ( scan_count < offset ) {
    if ( msb->cbuf_inode->i_len[ scan_count ] != 0 )
      live_count++;
    bytes_before += msb->cbuf_inode->i_len[ scan_count++ ];
  }
  if ( len )
    live_count++;
  scan_count++;
  while ( scan_count < 16 ) {
    if ( msb->cbuf_inode->i_len[ scan_count ] != 0 )
      live_count++;
    bytes_after += msb->cbuf_inode->i_len[ scan_count++ ];
  }
  if ( live_count == 0 )
    return NULL;

  if ( blk && bytes_before > 0 )
    mfs_MemGet( msb, blk + sizeof( i_block ), &msb->cbuf_inode->data[ 0 ], bytes_before );

  if ( len > 0 )
    memcpy( &msb->cbuf_inode->data[ bytes_before ], buffer, len );
  msb->cbuf_inode->i_len[ offset ] = len;

  if ( blk && bytes_after > 0 )
    mfs_MemGet( msb, blk + sizeof( i_block ) + bytes_before + len, &msb->cbuf_inode->data[ bytes_before + len ], bytes_after );

  new_len = sizeof( i_block ) + bytes_before + len + bytes_after;

  if ( new_len <= MRAMFS_INODE_BLOCK_SIZE )
    msb->cbuf_inode->length = ( MRAMFS_INODE_BLOCK_SIZE >> 2 );
  else
    msb->cbuf_inode->length = ( ( new_len & 0x7C0 ) + 32 ) >> 2;
  TRACE( "Old len = %d/%d / New len = %d/%d\n", old_len, old_len << 2, msb->cbuf_inode->length, new_len );
  // round up to the next 32-byte boundary
  if ( blk == NULL || old_len != msb->cbuf_inode->length )
    new_blk = mfs_MemAllocate( msb, msb->cbuf_inode->length * 4 );
  else
    new_blk = blk;
  if ( new_blk )
    mfs_MemPut( msb, new_blk, msb->cbuf_inode, new_len );
  spin_unlock( msb->cbuf_inode_lock );
  ( unsigned long ) new_blk |= ( 16 - live_count );
  return new_blk;
}

int mramfs_put_inode_data( mfs_super *con, unsigned long inode_num, unsigned char *buffer, int len ) {
  int block_offset = inode_num & 15;
  int block_number = ( inode_num >> 4 ) & 1023;
  int block_table = inode_num >> 14;
  TRACE( "mramfs_put_inode_data: #%lu (%d/%d/%d) len=%02d\n",
         inode_num, block_table, block_number, block_offset, len );

  iHandle blk_table = ( iHandle ) mfs_getHandleIndexed( con, con->inodes, block_table );
  if ( blk_table == NULL ) {
    TRACE( "new inode block table, allocating memory.\n" );
    blk_table = ( iHandle ) mfs_MemAllocate( con, con->inode_table_size * sizeof( iHandle ) );
    mfs_putHandleIndexed( con, con->inodes, block_table, ( handle ) blk_table );
    /*    for ( c = 0;c < ( con->inode_table_size );c++ )
          ( mramfs_inode_block * ) blk_table[ c ] = NULL;
     */
  }
  handle blk = mfs_getHandleIndexed( con, blk_table, block_number );
  blk = ( handle ) ( ( unsigned long ) blk & BLOCK_ADDR_MASK ); // strip mask
  handle newblk = NULL;

  newblk = mramfs_insert_inode( con, blk, block_offset, buffer, len );

  if ( newblk == NULL ) {
    if ( len == 0 )
      return 0;
    else
      return -1;
  }

  mfs_putHandleIndexed( con, blk_table, block_number, newblk );

  newblk = ( handle ) ( ( unsigned long ) newblk & BLOCK_ADDR_MASK ); // strip mask
  if ( newblk != blk && blk != NULL ) {
    int len = 4 * mfs_getByte( con, blk );
    mfs_MemFree( con, len, blk );
  }
  return 0;
}

/*
 * Puts an inode into the inode table.
 * This will allocate memory for the inode if a put is attempted to one which
 * doesn't exist.
 *
 * This (I hope) allows for either uncompressed or compressed implementations
 * with otherwise unchanged code.
 *
 * TODO: We may want to MemFree unused inode blocks here.
 *
 * right now, we never shrink compressed inodes, only grow them
 */
int mramfs_compress_inode( struct inode *inode, int live ) {
  //  struct inode check;
  if ( inode == NULL ) {
    TRACE( "mramfs_compress_inode called on NULL inode\n" );
    return -1;
  }
  mfs_super *con = mramfs_getSB( inode->i_sb );
  //  check.i_sb = inode->i_sb;
  if ( inode->i_ino < 1 ) {
    TRACE( "mramfs_compress_inode called on inode # < 1\n" );
    return -1;
  }
  unsigned long inode_num = ( inode->i_ino ) - 1;
  //  int block_offset = inode_num & 15;
  int block_number = ( inode_num >> 4 ) & 1023;
  int block_table = inode_num >> 14;
  TRACE( "mramfs_compress_inode # %lu (%d/%d/%d) live=%01d mode=%d, filedata=%08x\n", inode_num,
         block_table, block_number, block_offset, live, inode->i_mode, ( unsigned int ) inode->u.generic_ip );

  if ( block_table > con->inode_table_count ) {
    printk( KERN_WARNING "Inode block #%d larger than table size\n", block_table );
    return -1;
  }
  if ( block_number > con->inode_table_size ) {
    printk( KERN_WARNING "Inode block #%d larger than table size\n", block_number );
    return -1;
  }
  unsigned int buffer[ 16 ];
  //    if (inode->i_nlink < 0) {
  //        inode->i_nlink = 0;
  //        printk(KERN_WARNING "Inode #%d had links_count < 0 -- correcting\n",inode_num);
  //    }
  int c = 0;
  for ( c = 0;c < 16;c++ )
    buffer[ c ] = 0;
  int len = 0;
  if ( live )
    len = comp_inode_gamma( inode, buffer, 0 );
  else
    len = 0;
  /*  decomp_inode_gamma(&check, buffer, 0);
    if (check.u.generic_ip != inode->u.generic_ip) {
      TRACE("Compression check failed for inode #%lu (%08x != %08x)\n", inode_num, (unsigned int) check.u.generic_ip, (unsigned int) inode->u.generic_ip);
    } else TRACE("Compression check passed for inode #%lu (%08x == %08x)\n", inode_num, (unsigned int) check.u.generic_ip, (unsigned int) inode->u.generic_ip);
  */  // TRACE("compressed length %d bits\n",len);
  if ( len & 7 )
    len += 8;
  len = len >> 3;
  // assert(len < 128);
  TRACE( "compressed length %d bytes\n", len );
  return mramfs_put_inode_data( con, inode_num, ( unsigned char * ) buffer, len );
}


inline int mramfs_get_inode_data( mfs_super *con, unsigned long inode_num, unsigned char *buffer ) {
  int block_offset = inode_num & 15;
  int block_number = ( inode_num >> 4 ) & 1023;
  int block_table = inode_num >> 14;
  iHandle blk_table = ( iHandle ) mfs_getHandleIndexed( con, con->inodes, block_table );
  TRACE( "mramfs_get_inode_data: #%lu (%u/%u/%u)\n", inode_num, block_table, block_number, block_offset );
  if ( blk_table == NULL ) {
    TRACE( "Inode table #%d is NULL on a get\n", block_table );
    return -1;
  }
  handle blk = mfs_getHandleIndexed( con, blk_table, block_number );
  if ( blk == NULL ) {
    TRACE( "Inode block #%d in table %d is NULL on a get\n", block_number, block_table );
    return -1;
  }
  blk = ( handle ) ( ( unsigned long ) blk & BLOCK_ADDR_MASK ); // strip mask

  unsigned char lens[ 16 ];
  int bytes_before = 0;
  int scan_count = 0;
  mfs_MemGet( con, blk, &lens, 16 );
  if ( lens[ block_offset ] == 0 )
    return 0;
  while ( scan_count < block_offset ) {
    bytes_before += lens[ scan_count++ ];
  }
  if ( lens[ block_offset ] > 0 )
    mfs_MemGet( con, blk + sizeof( i_block ) + bytes_before, buffer, lens[ block_offset ] );
  return lens[ block_offset ];
}

/*
 * Gets an inode from the inode table.
 * This (I hope) allows for either uncompressed or compressed implementations
 * with otherwise unchanged code.
 *
 * Returns -1 if attempting to get a non-allocated inode or one past the end of
 * the table.  It will happily return one without a link count, though!
 */
inline int mramfs_decomp_inode( struct inode *inode ) {
  mfs_super *con = mramfs_getSB( inode->i_sb );
  if ( inode == NULL || inode->i_ino < 1 ) {
    printk( KERN_ERR "mramfs_decomp_inode called with inode < 1 or NULL inode\n" );
    return -1;
  }
  unsigned long inode_num = ( inode->i_ino ) - 1;
  //  int block_offset = inode_num & 15;
  int block_number = ( inode_num >> 4 ) & 1023;
  int block_table = inode_num >> 14;
  TRACE( "mramfs_decomp_inode # %lu (%d/%d/%d)\n", inode_num, block_table, block_number, block_offset );
  if ( block_table > con->inode_table_count ) {
    printk( KERN_WARNING "Inode table #%d larger than table count\n", block_table );
    return -1;
  }
  if ( block_number > con->inode_table_size ) {
    printk( KERN_WARNING "Inode block #%d larger than table size\n", block_number );
    return -1;
  }
  unsigned int buffer[ 16 ];
  if ( mramfs_get_inode_data( con, inode_num, ( unsigned char * ) buffer ) < 0 ) {
    //    printk( KERN_WARNING "mramfs_decomp_inode: mramfs_get_inode_data failed\n");
    return -1;
  }
  decomp_inode_gamma( inode, buffer, 0 );
  inode->i_blksize = PAGE_CACHE_SIZE;
  inode->i_blocks = ( inode->i_size >> PAGE_CACHE_SHIFT );
  inode->i_rdev = NODEV;
  inode->i_mapping->a_ops = &mramfs_aops;
  switch ( inode->i_mode & S_IFMT ) {
  default:
    //TODO: last should be dev
    init_special_inode( inode, inode->i_mode, 0 ); // ,dev);
    break;
  case S_IFREG:
    inode->i_op = &mramfs_file_inode_operations;
    inode->i_fop = &mramfs_file_operations;
    break;
  case S_IFDIR:
    inode->i_op = &mramfs_dir_inode_operations;
    inode->i_fop = &mramfs_dir_file_operations;
    break;
  case S_IFLNK:
    inode->i_op = &mramfs_symlink_operations;
    //                inode->i_fop = &mramfs_symlink_file_operations;
    break;
  }
  return 0;
}

void mramfs_write_inode( struct inode *inode, int sync ) {
  //  TRACE( "mramfs_write_inode #%lu\n", inode->i_ino );
  mramfs_compress_inode( inode, 1 );
}

void mramfs_read_inode( struct inode *inode ) {
  /*  if (  == 0)
      TRACE( "mramfs_read_inode #%lu, data @ %08x\n", inode->i_ino, (unsigned int) inode->u.generic_ip);
    else
      TRACE( "mramfs_read_inode FAILED #%lu, data @ %08x\n", inode->i_ino, (unsigned int) inode->u.generic_ip);
  */
  mramfs_decomp_inode( inode );
}

void mramfs_put_inode( struct inode *inode ) {
  TRACE( "mramfs_put_inode #%lu\n", inode->i_ino );
  if ( inode->i_nlink > 0 )
    mramfs_compress_inode( inode, 1 );
  else
    mramfs_compress_inode( inode, 0 );
}

void mramfs_delete_inode( struct inode *inode ) {
  TRACE( "mramfs_delete_inode #%lu\n", inode->i_ino );
  mramfs_compress_inode( inode, 0 );
  remove_inode_hash( inode );
}

static int mramfs_statfs( struct super_block *sb, struct statfs *buf ) {
  TRACE( "mramfs_statfs\n" );
  mfs_super *con = mramfs_getSB( sb );
  buf->f_type = MRAMFS_MAGIC;
  buf->f_bsize = PAGE_CACHE_SIZE;
  buf->f_namelen = MRAMFS_MAX_NAME_LEN;
  buf->f_blocks = con->sa_size_mb << 8;
  buf->f_bfree = buf->f_bavail = ( buf->f_blocks - con->block_count - 16 );
  buf->f_files = con->file_count;
  buf->f_ffree = ( ( 1 << 24 ) - con->file_count );
  return 0;
}


/*
 * Lookup the file, filling the dentry if found.
 */
static struct dentry * mramfs_lookup( struct inode *dir, struct dentry *dentry ) {
  //        TRACE("mramfs_lookup: file='%s'\n",dentry->d_name.name);
  const unsigned char *file = dentry->d_name.name;
  mfs_super *msb = mramfs_getSB( dir->i_sb );
  int nl = strlen( file );
  mfs_directory *dir_obj = ( mfs_directory * ) dir->u.generic_ip;
  if ( !S_ISDIR( dir->i_mode ) ) {
    printk( KERN_WARNING "Called mramfs_find_file_in_dir on a non-directory inode #.\n" );
    return NULL;
  }
  if ( dir_obj == NULL ) {
    printk( KERN_WARNING "Called mramfs_find_file_in_dir on an unitialized directory.\n" );
    return NULL;
  }

  //TODO: hardcoded sizes are bad
  unsigned int fullhash = hash( ( unsigned char * ) file, nl, 0 );
  int key = fullhash & 0xff;

  iHandle dtable = ( iHandle ) mfs_getHandle( msb, ( iHandle ) &dir_obj->table );
  mfs_dentry * dentry_obj = ( mfs_dentry * ) mfs_getHandleIndexed( msb, dtable, key );
  TRACE( "mramfs_lookup: dtable @ %08x, dentry_obj @ %08x, hash=%08x\n", ( unsigned int ) dtable, ( unsigned int ) dentry_obj, fullhash );
  while ( dentry_obj != NULL ) {
    int denl = mfs_getShort( msb, ( handle ) & dentry_obj->namelen );
    //    TRACE("mramfs_lookup: comparing dentry nl('%d') == file nl('%d')\n", denl, nl);
    if ( nl == ( unsigned int ) denl ) {
      char tname[ MRAMFS_MAX_NAME_LEN + 2 ];
      mfs_MemGet( msb, ( handle ) & dentry_obj->name, tname, nl + 1 );
      //      TRACE("mramfs_lookup: comparing tname('%s') == file('%s')\n", tname, file);
      if ( strcmp( tname, file ) == 0 ) {
        struct inode * inode = NULL;
        inode = iget( dir->i_sb, mfs_getULong( msb, ( handle ) & dentry_obj->inode_num ) + 1 );
        mramfs_decomp_inode( inode );
        //        TRACE( "mramfs_lookup: file='%s' found as inode #%lu\n", dentry->d_name.name, inode->i_ino );
        d_add( dentry, inode );
        return NULL;
      }
    }
    dentry_obj = ( mfs_dentry * ) mfs_getHandle( msb, ( iHandle ) & dentry_obj->next );
    // didn't find it, keep looking
  }
  TRACE( "mramfs_lookup: file='%s' not found\n", dentry->d_name.name );
  d_add( dentry, NULL );
  return NULL;
}

inline int mramfs_do_readpage( struct inode *inode, unsigned long b_offset, unsigned long count,
                               unsigned char *buf ) {

  mfs_super *msb = mramfs_getSB( inode->i_sb );
  if ( inode == NULL ) {
    printk( KERN_ERR "mramfs_read with NULL inode\n" );
  }
  TRACE( "read %lu, offset=%lu/%lu, count=%lu\n", inode->i_ino, offset, offset << PAGE_CACHE_SHIFT, count );

  if ( !S_ISREG( inode->i_mode ) ) {
    printk( KERN_WARNING "Can't read on something other than a regular file (MODE: %d)\n", inode->i_mode );
    return -1;
  }

  if ( b_offset > inode->i_blocks ) {
    TRACE( "Offset %lu is past the last block in the file at %d bytes\n", offset, inode->i_blocks );
    return -1;
  }

  mfs_file_data *block = ( mfs_file_data * ) inode->u.generic_ip;

  int bcount = 0;
  // TODO: hardcoded block sizes are bad -- and there are a LOT of them in read/write/truncate
  while ( bcount < b_offset ) {
    if ( block == NULL ) {
      TRACE( "Attempting to read past end of file.\n" );
      return -1;
    }
    block = ( mfs_file_data * ) mfs_getHandle( msb, ( iHandle ) & block->next );
    bcount++;
  }

  // TODO: hardcoded block sizes are bad -- and there are a LOT of them in read/write/truncate
  TRACE( "Reading block %d from inode %lu.\n", b_offset, inode->i_ino );
  // read as many whole blocks as we can
  handle data = mfs_getHandle( msb, ( iHandle ) &block->data );
  if ( msb->compress_blocks ) {
    spin_lock( msb->cbuf_data_lock );
    unsigned short clen = mfs_getUShort( msb, ( handle ) & block->clength );
    unsigned short dlen = mfs_getUShort( msb, ( handle ) & block->dlength );
    if ( clen < dlen ) {
      mfs_MemGet( msb, data, msb->cbuf_data, clen );
      mramfs_uncompress_block( buf, dlen, msb->cbuf_data, clen );
    } else {
      mfs_MemGet( msb, data, buf, count );
    }
    spin_unlock( msb->cbuf_data_lock );
  } else
    mfs_MemGet( msb, data, buf, count );

  // TODO: We could update atimes here if we care.

  return count;
}

/*
 * Read a page. Again trivial. If it didn't already exist
 * in the page cache, it is zero-filled.
 */
static int mramfs_readpage( struct file *file, struct page * page ) {
  /*        if (!Page_Uptodate(page)) {
                  memset(kmap(page), 0, PAGE_CACHE_SIZE);
                  kunmap(page);
                  flush_dcache_page(page);
          SetPageUptodate(page);
          }
  */
  struct address_space *as = page->mapping;
  if ( as == NULL ) {
    printk( "NULL address_space in readpage\n" );
    return -EINVAL;
  }
  struct inode *inode = as->host;
  if ( inode == NULL ) {
    printk( "NULL inode in readpage\n" );
    return -EINVAL;
  }
  unsigned long len = PAGE_CACHE_SIZE;
  unsigned char *addr = page_address( page );

  int res = mramfs_do_readpage( inode, page->index, len, addr );
  TRACE( "mramfs_readpage: file='%s', offset=%lu, size=%lu, len=%d of %lu, @ %08x\n", file->f_dentry->d_name.name, offset, ( unsigned long ) inode->i_size, res, len, ( unsigned int ) addr );
  SetPageUptodate( page );
  UnlockPage( page );
  if ( res > 0 )
    return 0;
  else
    return res;
}

/* Returns length of decompressed data.
 * int mramfs_uncompress_block(void *dst, int dstlen, void *src, int srclen);
 * int mramfs_compress_block(void *dst, int dstlen, void *src, int srclen);
 */

/* Write to a file
 */
int
mramfs_do_writepage( struct inode *inode, unsigned long b_offset, unsigned long count, unsigned char *buf ) {
  if ( inode == NULL ) {
    printk( KERN_ERR "mramfs_read with NULL inode\n" );
  }
  //    long inode_num = inode->i_ino;
  TRACE( "write %lu: offset=%lu, count=%lu Mode: %04X\n", inode->i_ino, offset, count, inode->i_mode );
  if ( !S_ISREG( inode->i_mode ) ) {
    TRACE( "Can't write on something other than a regular file (MODE: %d)\n", inode->i_mode );
    return -1;
  }
  mfs_super *msb = mramfs_getSB( inode->i_sb );
  // first, find our place.
  unsigned long bcount = 0;
  mfs_file_data *block = ( mfs_file_data * ) inode->u.generic_ip,
                         *lastblock = NULL;

  while ( bcount <= b_offset ) {
    if ( block == NULL ) {
      //      TRACE( "Allocating new block at %lu\n", bcount);
      block = ( mfs_file_data * ) mfs_MemAllocate( msb, sizeof( mfs_file_data ) );
      if ( block == NULL ) {
        //        TRACE( "Memory allocation failed on write.\n" );
        return -ENOMEM;
      }
      //            block->inode_num = inode_num;
      mfs_putHandle( msb, ( iHandle ) & block->next, NULL );
      mfs_putHandle( msb, ( iHandle ) & block->data, NULL );
      mfs_putUShort( msb, ( handle ) & block->clength, 0 );
      mfs_putUShort( msb, ( handle ) & block->dlength, 0 );
      mfs_putULong ( msb, ( handle ) & block->offset, bcount );
      //TODO: we may want to zero-out our data.
      msb->block_count++;
      if ( lastblock == NULL ) {            // case: creating a new first block to a previously zero-length file
        // TRACE("New file data!");
        inode->u.generic_ip = block;
      } else {
        mfs_putHandle( msb, ( iHandle ) & lastblock->next, ( handle ) block );
      }
    }
    if ( bcount == b_offset )
      break;
    lastblock = block;
    block = ( mfs_file_data * ) mfs_getHandle( msb, ( iHandle ) & block->next );
    // could optimize this to = NULL for newly created blocks
    // TODO: hardcoded block sizes are bad -- and there are a LOT of them in read/write/truncate
    bcount ++;
  }

  handle data = mfs_getHandle( msb, ( iHandle ) &block->data );
  // assert(b_offset == mfs_getULong(msb, (handle) &block->offset)
  TRACE( "Writing block.\n" );
  if ( msb->compress_blocks ) {
    spin_lock( msb->cbuf_data_lock );
    unsigned short clen = mramfs_compress_block( msb->cbuf_data, clen, buf, count );
    unsigned short old_clen = mfs_getUShort( msb, ( handle ) & block->clength );
    printk( KERN_NOTICE "Writing compressed: %lu bytes in, %d bytes out (%d before)\n", count, clen, old_clen );

    if ( clen >= count )
      clen = count;  // skip saving compressed data if we produce longer data!
    mfs_putUShort( msb, ( handle ) & block->dlength, count );
    mfs_putUShort( msb, ( handle ) & block->clength, clen );

    if ( data == NULL ) {
      data = mfs_MemAllocate( msb, clen );
      if ( data == NULL ) {
        printk( KERN_ERR "Couldn't allocate data for new block" );
        return ( -ENOMEM );
      } else
        mfs_putHandle( msb, ( iHandle ) & block->data, data );
    }
    if ( clen < count )
      mfs_MemPut( msb, data, msb->cbuf_data, clen ); // if we actually compressed anything
    else
      mfs_MemPut( msb, data, buf, count );           // otherwise, skip it
    spin_unlock( msb->cbuf_data_lock );
  } else {
    if ( data == NULL ) {
      data = mfs_MemAllocate( msb, PAGE_CACHE_SIZE );
      if ( data == NULL ) {
        printk( KERN_ERR "Couldn't allocate data for new block" );
        return ( -ENOMEM );
      } else
        mfs_putHandle( msb, ( iHandle ) & block->data, data );
    }
    mfs_MemPut( msb, data, buf, count );
  }
  /* inode->i_atime = inode->i_mtime = CURRENT_TIME;
  mark_inode_dirty( inode );
  if ( mramfs_compress_inode( inode, 1 ) == -1 ) {
      TRACE( "Couldn't write inode.\n" );
      return -1;
    }
  */
  return count;
}


static int mramfs_writepage( struct page * page ) {
  TRACE( "mramfs_writepage: page=%08x\n", ( int ) page );
  struct address_space *as = page->mapping;
  if ( as == NULL ) {
    printk( "NULL address_space in writepage\n" );
    return -EINVAL;
  }
  struct inode *inode = as->host;
  if ( inode == NULL ) {
    printk( "NULL inode in writepage\n" );
    return -EINVAL;
  }
  unsigned long len = PAGE_CACHE_SIZE;
  unsigned char *addr = page_address( page );
  int res = mramfs_do_writepage( inode, page->index, len, addr );
  if ( res > 0 )
    return 0;
  else
    return res;
}

static int mramfs_prepare_write( struct file *file, struct page *page, unsigned offset, unsigned to ) {
  TRACE( "mramfs_prepare_write: file='%s', bytes=%08x-%08x\n", file->f_dentry->d_name.name, offset, to );
  void *addr = kmap( page );
  if ( !Page_Uptodate( page ) ) {
    TRACE( "Page not up to date. Allocating?\n" );
    memset( addr, 0, PAGE_CACHE_SIZE );
    flush_dcache_page( page );
    SetPageUptodate( page );
  }
  SetPageDirty( page );
  return 0;
}

static int mramfs_commit_write( struct file *file, struct page *page, unsigned offset, unsigned to ) {
  //  TRACE( "mramfs_commit_write: file='%s', bytes=%08x-=%08x\n", file->f_dentry->d_name.name, offset, to );
  struct inode *inode = page->mapping->host;
  loff_t pos = ( ( loff_t ) page->index << PAGE_CACHE_SHIFT ) + to;
  kunmap( page );
  if ( pos > inode->i_size )
    inode->i_size = pos;
  SetPageDirty( page );
  mramfs_writepage( page );
  return 0;
}


/*
 * Initializes an inode, allocating one if a number isn't given.
 *
 * Assumes a regular file, the current A/M/C times, and RWX for user only.
 * ... and links_count = 0!  Must set in create/mkdir
 *
 * Normally called with inode_num set to -1 allocates us a brand-new inode,
 * we can call it withj inode_num == 0 to create the root_dir inode, and we
 * might also have some other special case where we wanted to (re?)initialize
 * an inode where we already know the number.
 */
unsigned long mramfs_initialize_inode( struct super_block *sb, unsigned long number, int mode ) {
  unsigned long i_number;
  if ( number < 1 )
    i_number = find_free_inode( sb, 0 ) + 1;
  else
    i_number = number;
  if ( i_number < 1 )
    return 0;
  TRACE( "mramfs_initialize_inode: sb@%08x, inode #%lu, mode=%d\n", ( int ) sb, i_number, mode );
  struct inode *inode = iget( sb, i_number );
  if ( inode == NULL )
    return 0;
  inode->i_mode = mode;
  inode->i_uid = 0;
  inode->i_gid = 0;
  inode->i_atime = inode->i_mtime = inode->i_ctime = CURRENT_TIME;
  inode->i_flags = 0;
  inode->i_size = 0;
  inode->i_nlink = 0;
  inode->i_generation = 0;
  inode->u.generic_ip = NULL;
  if ( mramfs_compress_inode( inode, 1 ) >= 0 )
    return i_number;
  return 0;
}

/*
 * Creates a new directory with an empty hash table, and no files.
 * After calling, still needs a backlink to the owning inode (for GC) which
 *     may or may not yet be actually handled properly.
 */
handle mramfs_create_new_dir( mfs_super *msb ) {
  mfs_directory * dir = ( mfs_directory * ) mfs_MemAllocate( msb, sizeof( mfs_directory ) );
  if ( dir == NULL ) {
    TRACE( "Can't allocate new directory structure.\n" );
    return NULL;
  }

  // TODO: hardcoded table sizes are bad
  handle dir_table = mfs_MemAllocate( msb, sizeof( handle ) * MRAMFS_DIR_TABLE_SIZE );
  if ( dir_table == NULL ) {
    TRACE( "Can't allocate new directory table.\n" );
    mfs_MemFree( msb, sizeof( mfs_directory ), ( handle ) dir );
    return NULL;
  }
  mfs_putHandle( msb, ( iHandle ) & dir->table, dir_table );
  TRACE( "Allocated directory table at (%08x)\n", ( int ) dir_table );

  // clear the table: we have this cleared already
  // mfs_MemSet(msb, dir_table, 0, MRAMFS_DIR_TABLE_SIZE )
  mfs_putLong( msb, ( handle ) & dir->file_count, 0 );
  mfs_putLong( msb, ( handle ) & dir->inode_num, 0 );
  TRACE( "Created dir @ %08x\n", ( int ) dir );
  return ( handle ) dir;
}


/*
 * Creates a new (unpacked, in the current implementation) directory entry.
 * Find a spot in the hash table, and follows the chain of dentries to the end.
 */
mfs_dentry *mramfs_create_new_dentry( struct inode *dir_inode, struct inode *inode, struct dentry *vfs_dentry ) {
  int nl = 0;
  if ( dir_inode == NULL || inode == NULL || vfs_dentry == NULL ) {
    printk( KERN_WARNING "create_new_dentry being called on NULL dir/inode/dentry.\n" );
    return NULL;
  }
  const char *name = vfs_dentry->d_name.name;
  if ( name == NULL || strlen( name ) < 1 ) {
    printk( KERN_ERR "create_new_dentry being called with blank name.\n" );
    return NULL;
  }
  nl = strlen( name );
  if ( nl > MRAMFS_MAX_NAME_LEN ) {
    printk( KERN_WARNING "Name is too long!\n" );
    return NULL;
  }
  mfs_super *msb = mramfs_getSB( inode->i_sb );
  mfs_directory *dir = dir_inode->u.generic_ip;
  unsigned long inode_num = inode->i_ino - 1;

  // TODO: hardcoded table sized are bad
  int fullhash = hash( ( unsigned char * ) name, nl, 0 );
  int key = fullhash & 0xff;

  TRACE( "Creating dentry in %08x for inode %lu with name %s (key = %d, nl = %d)\n",
         ( unsigned int ) dir_inode, inode->i_ino, name, key, nl );
  iHandle dtable = ( iHandle ) mfs_getHandle( msb, ( iHandle ) &dir->table );
  mfs_dentry *m_dentry = ( mfs_dentry * ) mfs_getHandleIndexed( msb, dtable, key );
  if ( m_dentry == NULL ) {    // case: directory hash bucket empty.
    m_dentry = ( mfs_dentry * ) mfs_MemAllocate( msb, sizeof( mfs_dentry ) );
    if ( m_dentry == NULL ) {
      printk( KERN_ERR "Couldn't allocate memory for new Dentry\n" );
      return NULL;
    }
    mfs_putLong ( msb, ( handle ) & m_dentry->inode_num, inode_num );
    mfs_putHandle( msb, ( iHandle ) & m_dentry->next, NULL );
    mfs_putShort ( msb, ( handle ) & m_dentry->namelen, nl );
    mfs_MemPut ( msb, ( handle ) & m_dentry->name, name, nl + 1 );
    mfs_putHandleIndexed( msb, dtable, key, ( handle ) m_dentry );
  } else {                          // case: directory hash bucket not empty
    mfs_dentry *scan = m_dentry;
    mfs_dentry *next = ( mfs_dentry * ) mfs_getHandle( msb, ( iHandle ) & scan->next );
    while ( next != NULL ) {
      scan = next; // find the end of our chain
      next = ( mfs_dentry * ) mfs_getHandle( msb, ( iHandle ) & scan->next );
    }
    m_dentry = ( mfs_dentry * ) mfs_MemAllocate( msb, sizeof( mfs_dentry ) );
    if ( m_dentry == NULL ) {
      TRACE( "Couldn't allocate memory for new Dentry\n" );
      return NULL;
    }
    mfs_putLong ( msb, ( handle ) & m_dentry->inode_num, inode_num );
    mfs_putHandle( msb, ( iHandle ) & m_dentry->next, NULL );
    mfs_putShort ( msb, ( handle ) & m_dentry->namelen, nl );
    mfs_MemPut ( msb, ( handle ) & m_dentry->name, name, nl + 1 );
    mfs_putHandle( msb, ( iHandle ) & scan->next, ( handle ) m_dentry );
  }
  long fc = mfs_getLong( msb, ( handle ) &dir->file_count );
  mfs_putLong( msb, ( handle ) &dir->file_count, fc + 1 );
  TRACE( "Dentry created at %08x\n", ( unsigned int ) m_dentry );
  mark_inode_dirty( dir_inode );
  return m_dentry;
}


/*
 * File creation. Allocate an inode, and we're done..
 */
static int mramfs_do_mknod( struct inode *dir, struct dentry *dentry, int mode, int dev, handle payload ) {
  int error = -ENOSPC;
  mfs_super *msb = mramfs_getSB( dir->i_sb );

  unsigned long i_num = mramfs_initialize_inode( dir->i_sb, 0, mode );
  if ( i_num < 1 )
    return error;

  struct inode * inode = iget( dir->i_sb, i_num );
  if ( inode ) {
    mramfs_decomp_inode( inode );
    inode->u.generic_ip = ( void * ) payload;
    if ( mramfs_create_new_dentry( dir, inode, dentry ) == NULL )
      return error;
    d_instantiate( dentry, inode );
    //    dget( dentry );                /* Extra count - pin the dentry in core */
    error = 0;
    mark_inode_dirty( inode );
    msb->file_count++;
  }
  return error;
}

static int mramfs_mknod( struct inode *dir, struct dentry *dentry, int mode, int dev ) {
  TRACE( "mramfs_mknod: file='%s', mode=%d, dev=%d\n", dentry->d_name.name, mode, dev );
  return mramfs_do_mknod( dir, dentry, mode, 0, NULL );
}

static int mramfs_mkdir( struct inode * dir, struct dentry * dentry, int mode ) {
  mfs_directory *dir_obj = ( mfs_directory * ) mramfs_create_new_dir( mramfs_getSB( dir->i_sb ) );
  TRACE( "mramfs_mkdir: dentry='%s', mode=%d, dir_obj @ %08x\n", dentry->d_name.name, mode, ( unsigned int ) dir_obj );
  if ( dir_obj == NULL )
    return -ENOMEM;
  return mramfs_do_mknod( dir, dentry, mode | S_IFDIR, 0, ( handle ) dir_obj );
}

static int mramfs_create( struct inode *dir, struct dentry *dentry, int mode ) {
  TRACE( "mramfs_create: file='%s', mode=%d\n", dentry->d_name.name, mode );
  return mramfs_do_mknod( dir, dentry, mode | S_IFREG, 0, NULL ); //TODO: New file data.
}

/*
 * Link a file..
 */
static int mramfs_link( struct dentry *old_dentry, struct inode * dir, struct dentry * dentry ) {
  TRACE( "mramfs_link: old='%s', new='%s'\n", old_dentry->d_name.name, dentry->d_name.name );
  struct inode *inode = old_dentry->d_inode;
  if ( S_ISDIR( inode->i_mode ) )
    return -EPERM;
  inode->i_nlink++;
  if ( mramfs_create_new_dentry( dir, inode, dentry ) == NULL )
    return -ENOENT;
  atomic_inc( &inode->i_count );        /* New dentry reference */
  dget( dentry );                /* Extra pinning count for the created dentry */
  d_instantiate( dentry, inode );
  return 0;
}

static inline int mramfs_positive( struct dentry *dentry ) {
  TRACE( "mramfs_positive: file='%s'\n", dentry->d_name.name );
  return dentry->d_inode && !d_unhashed( dentry );
}

/*
 * Check that a directory is empty (this works
 * for regular files too, they'll just always be
 * considered empty..).
 *
 * Note that an empty directory can still have
 * children, they just all have to be negative..
 */
static int mramfs_empty( struct dentry *dentry ) {
  TRACE( "mramfs_empty: file='%s'\n", dentry->d_name.name );
  struct list_head *list;

  if ( dentry && dentry->d_inode && S_ISDIR( dentry->d_inode->i_mode ) &&
       dentry->d_inode->u.generic_ip &&
       ( ( mfs_directory* ) dentry->d_inode->u.generic_ip ) ->file_count > 0 )
    return 1;

  spin_lock( &dcache_lock );
  list = dentry->d_subdirs.next;

  while ( list != &dentry->d_subdirs ) {
    struct dentry * de = list_entry( list, struct dentry, d_child );
    if ( mramfs_positive( de ) ) {
      spin_unlock( &dcache_lock );
      return 0;
    }
    list = list->next;
  }
  spin_unlock( &dcache_lock );
  return 1;
}

int long mramfs_do_unlink( struct inode *dir_inode, struct dentry *dentry ) {
  const unsigned char *file = dentry->d_name.name;
  int nl = 0;
  nl = strlen( file );
  mfs_super * msb = mramfs_getSB( dir_inode->i_sb );
  mfs_directory *dir_obj = ( mfs_directory * ) dir_inode->u.generic_ip;
  if ( !S_ISDIR( dir_inode->i_mode ) ) {
    printk( KERN_WARNING "Called mramfs_do_unlink on a non-directory inode #.\n" );
    return -1;
  }
  if ( dir_obj == NULL ) {
    printk( KERN_WARNING "Called mramfs_find_file_in_dir on an unitialized directory.\n" );
    return -1;
  }
  int fullhash = hash( ( unsigned char * ) file, nl, 0 ); // not really using this yet
  int key = fullhash & 0xff;                                   // TODO: fixed table sizes are bad.
  iHandle dtable = ( iHandle ) mfs_getHandle( msb, ( iHandle ) &dir_obj->table );
  mfs_dentry *curr = ( mfs_dentry * ) mfs_getHandleIndexed( msb, dtable, key ); // a traversal pointer
  mfs_dentry *last = NULL;

  TRACE( "Removing dentry from dir_inode %lu with name %s (key = %d, nl = %d)\n", dir_inode->i_ino, file, key, nl );
  if ( curr == NULL ) {
    printk( KERN_ERR "Called mramfs_do_unlink on a missing file\n" );
    return -1;
  }
  while ( curr != NULL ) {
    int cnl = mfs_getShort( msb, ( handle ) & curr->namelen );
    if ( nl == cnl ) {
      char tname[ MRAMFS_MAX_NAME_LEN + 2 ];
      mfs_MemGet( msb, ( handle ) & curr->name, tname, cnl + 1 );
      TRACE( "Do_unlink: NL(%d) == CNL(%d) -- fn:\"%s\" ?= dentry_name=\"%s\"\n", nl, cnl, file, tname );
      if ( strcmp( tname, file ) == 0 )
        break; // found it!
    }
    TRACE( "Do_unlink: NL(%d) != CNL(%d) -- fn:\"%s\"", nl, cnl, file );
    last = curr;                                     // otherwise, save prior entry and keep looking
    curr = ( mfs_dentry * ) mfs_getHandle( msb, ( iHandle ) & curr->next );  // pointer traversals are a bear!
    if ( curr == NULL ) {
      printk( KERN_ERR "Called mramfs_do_unlink on a missing file\n" );
      return -1;
    }
  }
  handle next = mfs_getHandle( msb, ( iHandle ) &curr->next ); // this can be NULL, we don't mind
  if ( last != NULL )
    mfs_putHandle( msb, ( iHandle ) &last->next, next );
  else
    mfs_putHandleIndexed( msb, dtable, key, next );

  long fc = mfs_getLong( msb, ( handle ) &dir_obj->file_count );
  mfs_putLong( msb, ( handle ) &dir_obj->file_count, fc - 1 );

  msb->file_count--;
  mfs_MemFree( msb, sizeof( mfs_dentry ), ( handle ) curr );
  return 0;
}


/*
 * This works for both directories and regular files.
 * (non-directories will always have empty subdirs)
 */
static int mramfs_unlink( struct inode * dir, struct dentry *dentry ) {
  TRACE( "mramfs_unlink: file='%s'\n", dentry->d_name.name );
  int retval = -ENOTEMPTY;
  struct inode *inode = dentry->d_inode;
  //  inode->i_nlink--;
  mfs_super *msb = mramfs_getSB( inode->i_sb );
  dput( dentry );                        /* Undo the count from "create" - this does all the work */
  retval = mramfs_do_unlink( dir, dentry );
  if ( inode->i_nlink == 0 && S_ISREG( inode->i_mode ) ) {
    inode->i_size = 0;
    mfs_file_data *curr, *next;
    curr = inode->u.generic_ip;
    int freed = 0;
    while ( curr != NULL ) {
      next = ( mfs_file_data * ) mfs_getHandle( msb, ( iHandle ) & curr->next );
      //free_pages((unsigned long) curr->data, 0);
      mfs_MemFree( msb, PAGE_CACHE_SIZE, mfs_getHandle( msb, ( iHandle ) & curr->data ) );
      mfs_MemFree( msb, sizeof( mfs_file_data ), ( handle ) curr );
      curr = next;
      freed++;
    }
    mfs_super * con = mramfs_getSB( dir->i_sb );
    con->block_count -= freed;
    inode->u.generic_ip = NULL;
  }
  if ( inode->i_nlink == 0 )
    mramfs_delete_inode( inode );
  return retval;
}

/*
 * This works for both directories and regular files.
 * (non-directories will always have empty subdirs)
 */
static int mramfs_rmdir( struct inode * dir, struct dentry *dentry ) {
  TRACE( "mramfs_rmdir: file='%s'\n", dentry->d_name.name );
  int retval = -ENOTEMPTY;
  struct inode *child = dentry->d_inode;
  mfs_super *msb = mramfs_getSB( child->i_sb );
  if ( !S_ISDIR( child->i_mode ) ) {
    printk( KERN_ERR "Called rmdir on a non-directory inode\n" );
    return retval;
  }
  mfs_directory *dir_to_die = ( mfs_directory * ) child->u.generic_ip;
  if ( mfs_getLong( msb, ( handle ) &dir_to_die->file_count ) == 0 &&
       mramfs_empty( dentry ) ) {
    //    child->i_nlink--;
    dput( dentry );                        /* Undo the count from "create" - this does all the work */
    mfs_MemFree( msb, sizeof( mfs_dentry * ) * 256, mfs_getHandle( msb, ( iHandle ) & dir_to_die->table ) );
    mfs_MemFree( msb, sizeof( mfs_directory ), ( handle ) dir_to_die );
    child->u.generic_ip = NULL;
    retval = mramfs_do_unlink( dir, dentry );
  }
  return retval;

}


/*
 * The VFS layer already does all the dentry stuff for rename,
 * we just have to decrement the usage count for the target if
 * it exists so that the VFS layer correctly free's it when it
 * gets overwritten.
 */
//TODO: Still not native
static int mramfs_rename( struct inode * old_dir, struct dentry *old_dentry, struct inode * new_dir, struct dentry *new_dentry ) {
  TRACE( "mramfs_rename: old_name='%s', name='%s'\n", old_dentry->d_name.name, new_dentry->d_name.name );
  int error = -ENOTEMPTY;

  if ( mramfs_empty( new_dentry ) ) {
    struct inode * inode = new_dentry->d_inode;
    if ( inode ) {
      //      inode->i_nlink--;
      dput( new_dentry );
    }
    error = 0;
  }
  return error;
}

static int mramfs_readlink( struct dentry* entry,
                            char* out,
                            int length ) {
  char tname[ MRAMFS_MAX_NAME_LEN + 2 ];
  int nl = 0;
  if ( entry == NULL || entry->d_inode == NULL ) {
    printk( KERN_ERR "Called mramfs_readlink with NULL dentry or inode\n" );
    return -EINVAL;
  }
  struct inode *inode = entry->d_inode;
  mfs_super *msb = mramfs_getSB( inode->i_sb );
  if ( ( inode->i_mode & S_IFMT ) != S_IFLNK ) {
    printk( KERN_ERR "Called mramfs_readlink called on non-symlink\n" );
    return -EINVAL;
  }
  handle syml_obj = inode->u.generic_ip;
  if ( syml_obj == NULL ) {
    printk( KERN_ERR "Called mramfs_readlink called on a symlink with NULL string\n" );
    return -EINVAL;
  }
  nl = ( int ) mfs_getByte( msb, syml_obj );
  mfs_MemGet( msb, syml_obj + 1, tname, nl + 1 );
  TRACE( "mramfs_readlink: file='%s', symname='%s'\n", entry->d_name.name, tname );
  int rv = vfs_readlink( entry, out, length, tname );
  return rv;
}

static int mramfs_follow_link( struct dentry *entry, struct nameidata *nd ) {
  char tname[ MRAMFS_MAX_NAME_LEN + 2 ];
  int nl = 0;
  if ( entry == NULL || entry->d_inode == NULL ) {
    printk( KERN_ERR "Called mramfs_follow_link with NULL dentry or inode\n" );
    return -EINVAL;
  }
  struct inode *inode = entry->d_inode;
  mfs_super *msb = mramfs_getSB( inode->i_sb );
  if ( ( inode->i_mode & S_IFMT ) != S_IFLNK ) {
    printk( KERN_ERR "Called mramfs_follow_link called on non-symlink\n" );
    return -EINVAL;
  }
  handle syml_obj = inode->u.generic_ip;
  if ( syml_obj == NULL ) {
    printk( KERN_ERR "Called mramfs_follow_link called on a symlink with NULL string\n" );
    return -EINVAL;
  }
  nl = ( int ) mfs_getByte( msb, syml_obj );
  mfs_MemGet( msb, syml_obj + 1, tname, nl + 1 );
  TRACE( "mramfs_follow_link: file='%s', symname='%s'\n", entry->d_name.name, tname );
  int rv = vfs_follow_link( nd, tname );
  return rv;
}


static int mramfs_symlink( struct inode * dir, struct dentry *dentry, const char * symname ) {
  int error;
  TRACE( "mramfs_symlink: file='%s', symname='%s'\n", dentry->d_name.name, symname );
  int l = strlen( symname ) + 1;
  mfs_super *msb = mramfs_getSB( dir->i_sb );
  handle syml_obj = mfs_MemAllocate( msb, l + 2 );
  mfs_putByte( msb, syml_obj, l );
  mfs_MemPut( msb, syml_obj + 1, symname, l + 1 );
  mfs_putByte( msb, syml_obj + l + 1, 0 );
  error = mramfs_do_mknod( dir, dentry, S_IFLNK | S_IRWXUGO, 0, syml_obj );
  return error;
}



//TODO: Not native, but probably not needed
static int mramfs_sync_file( struct file * file, struct dentry *dentry, int datasync ) {
  TRACE( "mramfs_sync_file: file='%s', dentry='%s', datasync=%d\n", file->f_dentry->d_name.name, dentry->d_name.name, datasync );
  return 0; // generic_sync_file(file, dentry, datasync);
}

void *mount_fs( struct super_block *vfs_sb, unsigned char *region ) {
  TRACE( "loading global context\n" );
  mfs_super *super;
  if ( ( super = kmalloc( sizeof( mfs_super ), GFP_ATOMIC ) ) == NULL )
    return NULL;
  memcpy( super, region, sizeof( mfs_super ) );
  dump_super( super );
  super->sa_region = region;
  vfs_sb->u.generic_sbp = super;
  super->vfs_sb = vfs_sb;
  spin_lock_init( super->inode_lock );
  spin_lock_init( super->directory_lock );
  spin_lock_init( super->cbuf_inode_lock );
  spin_lock_init( super->cbuf_data_lock );
  super->cbuf_data = kmalloc( 8192, GFP_KERNEL );
  super->cbuf_inode = kmalloc( 1024, GFP_KERNEL );

  TRACE( "Mount FS -> rootdir = %08x, rootdir->table=%08x\n", ( int ) super->rootdir, ( int ) mfs_getInt( super, ( handle ) &( ( mfs_directory * ) ( super->rootdir ) ) ->table ) );
  TRACE( " BCnt = %ld, FCnt = %ld\nAcTSz = %d, AcHSz = %d, AcCnt = %d, AcTH = %08x, AcHH = %08x\n", super->block_count, super->file_count, super->acl_table_size, super->acl_hash_size, super->acl_count, ( unsigned int ) super->acl_table, ( unsigned int ) super->acl_hash );
  if ( super->acl_table == NULL ) {
    printk( KERN_WARNING "No ACL reference table handle." );
    return NULL;
  }
  if ( super->acl_hash == NULL ) {
    printk( KERN_WARNING "No ACL Hash Table handle." );
    return NULL;
  }
  TRACE( "TSp=%ld, ICnt=%ld, ITSz=%d ITCnt=%d\n", super->total_space, super->inode_count, super->inode_table_size, super->inode_table_count );

  if ( super->inodes == NULL ) {
    TRACE( "No inode table pointers.\n" );
    return NULL;
  };
  struct inode *rootdir_inode = iget( vfs_sb, 1 );
  if ( rootdir_inode == NULL ) {
    TRACE( "Couldn't create root directory inode.\n" );
    return NULL;
  };
  // TODO: Check for error
  mramfs_decomp_inode( rootdir_inode );
  rootdir_inode->u.generic_ip = ( mfs_file_data * ) super->rootdir;
  // NOTE: I'm not sure whether ++ or = 1 is clearer here.
  return super;
}


void *create_fs( struct super_block *vfs_sb, unsigned char *region ) {
  TRACE( "initializing global context\n" );
  mfs_super *super;
  if ( ( super = kmalloc( sizeof( mfs_super ), GFP_ATOMIC ) ) == NULL )
    return NULL;
  super->sa_size_mb = MRAMFS_REGION_MB;
  super->sa_region = region;
  spin_lock_init( super->inode_lock );
  spin_lock_init( super->directory_lock );
  spin_lock_init( super->cbuf_inode_lock );
  spin_lock_init( super->cbuf_data_lock );
  super->cbuf_data = kmalloc( 8192, GFP_KERNEL );
  super->cbuf_inode = kmalloc( 1024, GFP_KERNEL );
  super->compress_inodes = MRAMFS_COMPRESS_INODES;
  super->compress_blocks = MRAMFS_COMPRESS_BLOCKS;
  mfs_saInit( super );
  mfs_MemInit( super );
  super->count = 1;
  vfs_sb->u.generic_sbp = super;
  super->vfs_sb = vfs_sb;

  if ( !( super->rootdir = mramfs_create_new_dir( super ) ) ) {
    TRACE( "couldn't create root directory\n" );
    return NULL;
  };
  mfs_directory *rootdir = ( mfs_directory * ) super->rootdir;
  TRACE( "Create FS -> rootdir = %08x, rootdir->table=%08x\n", ( int ) super->rootdir, ( int ) mfs_getInt( super, ( handle ) &( ( mfs_directory * ) ( super->rootdir ) ) ->table ) );
  // HASH SIZE _MUST_ BE A POWER OF TWO -- YOU HAVE BEEN WARNED
  super->block_count = 0;
  super->file_count = 0;
  super->acl_table_size = 4000;       // We should not hardcode this!
  super->acl_hash_size = 1024;        // We should not hardcode this!
  super->acl_count = 0;
  super->acl_table = ( iHandle ) 16384; // We should not hardcode this!
  //  ( iHandle ) mfs_MemAllocate( super, sizeof( handle ) * super->acl_table_size );
  super->acl_hash = ( iHandle ) mfs_MemAllocate( super, sizeof( handle ) * super->acl_hash_size );
  if ( super->acl_table == NULL ) {
    printk( KERN_WARNING "Couldn't allocate ACL reference table." );
    return NULL;
  }
  if ( super->acl_hash == NULL ) {
    printk( KERN_WARNING "Couldn't allocate ACL Hash Table." );
    return NULL;
  }
  mfs_MemSet( super, ( handle ) super->acl_hash, 0, super->acl_hash_size * sizeof( handle ) );
  mfs_MemSet( super, ( handle ) super->acl_table, 0, super->acl_table_size * sizeof( handle ) );
  super->total_space = 0;
  super->file_count = 0;
  super->inode_count = 0;
  super->inode_table_size = 1024;   // We should not hard code this!
  super->inode_table_count = 1024;  // We should not hard code this!
  super->inodes = ( iHandle ) mfs_MemAllocate( super, super->inode_table_count * sizeof ( iHandle ) );

  if ( super->inodes == NULL ) {
    TRACE( "Couldn't allocate inode table pointers.\n" );
    return NULL;
  };
  mfs_MemSet( super, ( handle ) super->inodes, 0, 1024 * sizeof( handle ) );
  if ( mramfs_initialize_inode( vfs_sb, 1, S_IFDIR | 0755 ) != 1 )
    return NULL;
  struct inode *rootdir_inode = iget( vfs_sb, 1 );
  if ( rootdir_inode == NULL ) {
    TRACE( "Couldn't create root directory inode.\n" );
    return NULL;
  };
  // TODO: Check for error
  mramfs_decomp_inode( rootdir_inode );
  rootdir_inode->u.generic_ip = ( mfs_file_data * ) super->rootdir;
  // NOTE: I'm not sure whether ++ or = 1 is clearer here.
  rootdir_inode->i_nlink++; // = 1;
  rootdir_inode->i_generation++;  // = 1;
  mfs_putUInt( super, ( handle ) &rootdir->inode_num, 1 );
  mfs_putUInt( super, ( handle ) &rootdir->parent_inode, 1 );
  if ( mramfs_compress_inode( rootdir_inode, 1 ) < 0 )
    return NULL;
  return super;
}

unsigned int mramfs_find_acl( mfs_super *con, unsigned short uid, unsigned short gid, unsigned short mode ) {
  TRACE( "find_acl -- uid: %d, gid: %d, mode: %d\n", uid, gid, mode );
  long hashvalue = hash_ugp( uid, gid, mode );
  int hashindex = hashvalue & ( con->acl_hash_size - 1 );        // Hence the power of two -- I cheat!
  //  TRACE("hashvalue: %lu, index: %d, acl_count: %d\n", hashvalue, hashindex, con->acl_count);
  mfs_acl *last = ( mfs_acl * ) mfs_getHandleIndexed( con, con->acl_hash, hashindex );
  //  TRACE("Last @ %08x\n",(unsigned int) last);
  while ( last != NULL ) {
    if ( mfs_getInt( con, ( handle ) & last->hashvalue ) == hashvalue && mfs_getUShort( con, ( handle ) & last->mode ) == mode && mfs_getUShort( con, ( handle ) & last->uid ) == uid && mfs_getUShort( con, ( handle ) & last->gid ) == gid ) {
      //      TRACE( "Matched ACL # %ld\n", mfs_getLong(con, (handle)&last->table_number));
      return mfs_getInt( con, ( handle ) & last->table_number );
    }
    last = ( mfs_acl * ) mfs_getHandle( con, ( iHandle ) & last->next );
    if ( last == NULL )
      break;
  }

  mfs_acl *acl = ( mfs_acl * ) mfs_MemAllocate( con, sizeof( mfs_acl ) );
  //    TRACE("Allocated %d bytes for new ACL at %08x\n",sizeof(mfs_acl),(unsigned int) acl);
  if ( acl == NULL ) {
    printk( KERN_WARNING "Couldn't allocate memory for ACL object.\n" );
    return -1;
  }
  mfs_putInt ( con, ( handle ) & acl->hashvalue, hashvalue );
  mfs_putHandle( con, ( iHandle ) & acl->next, NULL );
  mfs_putUShort( con, ( handle ) & acl->uid, uid );
  mfs_putUShort( con, ( handle ) & acl->gid, gid );
  mfs_putUShort( con, ( handle ) & acl->mode, mode );
  mfs_putInt ( con, ( handle ) & acl->table_number, con->acl_count );
  mfs_putHandleIndexed( con, con->acl_table, con->acl_count, ( handle ) acl );
  if ( last == NULL )
    mfs_putHandleIndexed( con, con->acl_hash, hashindex, ( handle ) acl );
  else
    mfs_putHandle( con, ( iHandle ) & last->next, ( handle ) acl );
  //    assert(con->acl_count <= con->acl_table_size);
  return con->acl_count++;
}

int comp_inode_gamma( struct inode *inode, unsigned int *dest, int bits ) {
  mfs_super *con = mramfs_getSB( inode->i_sb );
  // assert(dest != NULL);
  unsigned int acl = mramfs_find_acl( con, inode->i_uid, inode->i_gid, inode->i_mode );
  if ( acl < 0 ) {
    printk( KERN_WARNING "Danger! Danger! mramfs_find_acl failed!\n" );
    acl = 0;
  }
  //  TRACE("Compressing inode @ %08x to buffer @ %08x bit offset %d, data @ %08x, ACL=%u\n", (unsigned int) inode, (unsigned int) dest, bits, (unsigned int) inode->u.generic_ip, acl);
  int curbits = bits;
  long ctd = ( inode->i_ctime ) - millenium;
  long mtd = ( inode->i_mtime ) - ( inode->i_ctime );
  long atd = ( inode->i_atime ) - ( inode->i_mtime );
  gamma_PutOneBit( 1, dest, &curbits );
  gamma_PutBits( ( unsigned int ) inode->u.generic_ip, 32, dest, &curbits );
  gamma_CompressOnce( &compressors.i_mode_g, acl, dest, &curbits, 0 );
  //    gamma_CompressOnce(&compressors.i_mode_g, inode->i_mode,   dest, &curbits, 0);
  gamma_CompressOnce( &compressors.i_flags_g, inode->i_flags, dest, &curbits, 0 );
  //    gamma_CompressOnce(&compressors.i_uid_g,   inode->i_uid,   dest, &curbits, 0);
  //    gamma_CompressOnce(&compressors.i_gid_g,   inode->i_gid,   dest, &curbits, 0);
  gamma_CompressOnce( &compressors.i_size_g, inode->i_size, dest, &curbits, 0 );
  gamma_CompressOnce( &compressors.i_ctime_g, ctd, dest, &curbits, 1 );
  gamma_CompressOnce( &compressors.i_mtime_g, mtd, dest, &curbits, 1 );
  gamma_CompressOnce( &compressors.i_atime_g, atd, dest, &curbits, 1 );
  gamma_CompressOnce( &compressors.i_nlink_g, inode->i_nlink, dest, &curbits, 0 );
  gamma_CompressOnce( &compressors.i_generation_g, inode->i_generation, dest, &curbits, 0 );
  //    TRACE("Filedata @ %08x\n", inode->u.generic_ip);

  if ( curbits > compressors.max_inode_bits )
    compressors.max_inode_bits = curbits;
  if ( curbits < compressors.min_inode_bits )
    compressors.min_inode_bits = curbits;
  compressors.inode_count++;
  compressors.total_bits += curbits;

  return curbits;
}

int decomp_inode_gamma( struct inode *inode, unsigned int *src, int bits ) {
  mfs_super *con = mramfs_getSB( inode->i_sb );
  //    assert(src != NULL);
  int curbits = bits;
  gamma_GetOneBit( &curbits, src );
  inode->u.generic_ip = ( void * ) gamma_GetBits( &curbits, src, 32 );
  unsigned int acl = gamma_UncompressOnce( &compressors.i_mode_g, src, &curbits, 0 );
  handle acl_handle = NULL;
  if ( ( acl_handle = mfs_getHandleIndexed( con, con->acl_table, acl ) ) != NULL ) {
    mfs_acl s_acl;
    mfs_MemGet( con, acl_handle, &s_acl, sizeof( mfs_acl ) );
    inode->i_mode = s_acl.mode;
    inode->i_gid = s_acl.gid;
    inode->i_uid = s_acl.uid;
    ;
  } else {
    printk( KERN_WARNING "Trying to access a nonexistent ACL table entry!\n" );
    inode->i_mode = 0;
    inode->i_gid = 0;
    inode->i_uid = 0;
  }
  //    inode->i_mode  = gamma_UncompressOnce(&compressors.i_mode_g,  src, &curbits, 0);
  inode->i_flags = gamma_UncompressOnce( &compressors.i_flags_g, src, &curbits, 0 );
  //    inode->i_uid   = gamma_UncompressOnce(&compressors.i_uid_g,   src, &curbits, 0);
  //    inode->i_gid   = gamma_UncompressOnce(&compressors.i_gid_g,   src, &curbits, 0);
  inode->i_size = gamma_UncompressOnce( &compressors.i_size_g, src, &curbits, 0 );
  inode->i_ctime = millenium +
                   ( long ) gamma_UncompressOnce( &compressors.i_ctime_g, src, &curbits, 1 );
  inode->i_mtime = inode->i_ctime +
                   ( long ) gamma_UncompressOnce( &compressors.i_mtime_g, src, &curbits, 1 );
  inode->i_atime = inode->i_mtime +
                   ( long ) gamma_UncompressOnce( &compressors.i_atime_g, src, &curbits, 1 );
  inode->i_nlink = gamma_UncompressOnce( &compressors.i_nlink_g, src, &curbits, 0 );
  inode->i_generation = gamma_UncompressOnce( &compressors.i_generation_g, src, &curbits, 0 );
  //  TRACE("Decmpressing inode to @ %08x from buffer @ %08x bit offset %d, data @ %08x, acl=%u\n", (unsigned int)inode, (unsigned int)src, bits, (unsigned int)inode->u.generic_ip, acl);
  /*
      TRACE("DATA  @ %08x\n", inode->u.generic_ip);
      TRACE("Mode  @ %08x\n", inode->i_mode);
      TRACE("Flags @ %08x\n", inode->i_flags);
      TRACE("Uid   @ %08x\n", inode->i_uid);
      TRACE("Gid   @ %08x\n", inode->i_gid);
      TRACE("Size  @ %08x\n", inode->i_size);
      TRACE("CTime @ %08x\n", inode->i_ctime);
      TRACE("MTime @ %08x\n", inode->i_mtime);
      TRACE("ATime @ %08x\n", inode->i_atime);
      TRACE("Links @ %08x\n", inode->i_nlink);
      TRACE("Versn @ %08x\n", inode->i_generation);
  */
  return curbits;
}


void initialize_compressors( void ) {
  TRACE( "mramfs: initialize_compressors\n" );
  const int b_mode[] = {
                         4, 8, 16, 31, -1, -1, -1
                       };
  //    const int b_uid[] =         { 1, 16, 31, -1, -1, -1 };
  //    const int b_gid[] =         { 1, 16, 31, -1, -1, -1 };
  const int b_size[] = {
                         1, 16, 31, -1, -1, -1
                       };
  const int b_ctime[] = {
                          16, 28, 31, -1, -1, -1
                        };
  const int b_mtime[] = {
                          16, 28, 31, -1, -1, -1
                        };
  const int b_atime[] = {
                          16, 28, 31, -1, -1, -1
                        };
  const int b_links_count[] = {
                                1, 4, 16, 31, -1, -1, -1
                              };
  const int b_flags[] = {
                          1, 16, 31, -1, -1, -1
                        };
  const int b_generation[] = {
                               1, 16, 31, -1, -1, -1
                             };

  gamma_InitCompressor( &compressors.i_mode_g, b_mode );  /* File mode */
  //    gamma_InitCompressor(&compressors.i_uid_g, b_uid);  /* Low 16 bits of Owner Uid */
  //    gamma_InitCompressor(&compressors.i_gid_g, b_gid);    /* Low 16 bits of Group Id */
  gamma_InitCompressor( &compressors.i_size_g, b_size );  /* Size in bytes */
  gamma_InitCompressor( &compressors.i_ctime_g, b_ctime );    /* Creation time */
  gamma_InitCompressor( &compressors.i_mtime_g, b_mtime );    /* Modification time */
  gamma_InitCompressor( &compressors.i_atime_g, b_atime );  /* Access time */
  gamma_InitCompressor( &compressors.i_nlink_g, b_links_count );  /* Links count */
  gamma_InitCompressor( &compressors.i_flags_g, b_flags ); /* File flags */
  gamma_InitCompressor( &compressors.i_generation_g, b_generation ); /* File version (for NFS) */

  compressors.max_inode_bits = 0;
  compressors.min_inode_bits = 512;
  compressors.inode_count = 0;
  compressors.total_bits = 0;
}

inline int calcbits( unsigned int i ) {
  if ( i == 0 )
    return 0;
  if ( i >= bitsize[ 31 ] )
    return 32;
  int j = 1;
  if ( i >= bitsize[ 8 ] ) {
    if ( i >= bitsize[ 16 ] ) {
      if ( i >= bitsize[ 24 ] ) {
        j = 25;
      } else {
        j = 17;
      }
    } else {
      j = 9;
    }
  }
  while ( bitsize[ j ] <= i )
    j++;
  return j;
}

/* static int mramfs_setattr( struct dentry* entry, struct iattr* attr ) {
  TRACE( "mramfs_setattr\n" );
  int error = 0;
  struct inode* inode = entry->d_inode;
  if ( inode == NULL ) {
    return -EINVAL;
  }
  if ( ( error = inode_change_ok( inode, attr ) ) < 0 ) {
    printk( "mramfs_setattr: inode_change_ok = %d\n", error );
    return error;
  }
  inode_setattr( inode, attr );
  mramfs_write_inode( inode, 1 );
  return 0;
}
*/

static void mramfs_truncate( struct inode *inode ) {
  mfs_super *msb = mramfs_getSB( inode->i_sb );
  TRACE( "mramfs_truncate on inode %lu to size %ld\n", ( unsigned long ) inode->i_ino,
         ( unsigned long ) inode->i_size );
  if ( !S_ISREG( inode->i_mode ) ) {
    TRACE( "Can't truncate on something other than a regular file (MODE: %d)\n", inode->i_mode );
    return ;
  }
  mfs_file_data *ptr = inode->u.generic_ip;
  if ( ptr == NULL ) {
    TRACE( "Truncate on empty file." );
    return ;
  }
  mfs_file_data *next = ( mfs_file_data * ) mfs_getHandle( msb, ( iHandle ) &ptr->next );
  unsigned long count = 0;
  while ( next != NULL && count < inode->i_size ) {
    count += PAGE_CACHE_SIZE;
    ptr = next;
    next = ( mfs_file_data * ) mfs_getHandle( msb, ( iHandle ) & ptr->next );
  }
  if ( next == NULL ) {
    if ( inode->i_size > count + PAGE_CACHE_SIZE )
      TRACE( "Hmmm.... size is greater than allocated space." );
    else
      TRACE( "No change in block allocation, just changed byte count." );
    return ;
  }
  mfs_putHandle( msb, ( iHandle ) &ptr->next, NULL );
  int freed = 0;
  while ( next != NULL ) {
    ptr = next;
    next = ( mfs_file_data * ) mfs_getHandle( msb, ( iHandle ) & ptr->next );
    mfs_MemFree( msb, PAGE_CACHE_SIZE, mfs_getHandle( msb, ( iHandle ) & ptr->data ) );
    mfs_MemFree( msb, sizeof( mfs_file_data ), ( handle ) ptr );
    freed++;
  }
  msb->block_count -= freed;
  return ;
}

int mramfs_readdir( struct file* filp, void *dirent, filldir_t filldir ) {
  if ( filp == NULL ) {
    printk( KERN_ERR "READDIR CALLED WITH FILLP == NULL\n" );
    return 0;
  }
  struct inode* dir_inode;
  dir_inode = filp->f_dentry->d_inode;
  mfs_super *msb = mramfs_getSB( dir_inode->i_sb );
  TRACE( "mramfs_readdir on dir_inode #%lu\n", dir_inode->i_ino );
  mfs_directory *dir = ( mfs_directory * ) dir_inode->u.generic_ip;  // ACTUALLY A HANDLE
  long fc = mfs_getLong( msb, ( handle ) &dir->file_count );
  if ( fc < 0 ) {
    printk( KERN_ERR "Notice me please! dir->file_count = %ld < 0\n", fc );
    // lie through our teeth, for now!
    fc = 1;
  }
  if ( filp->f_pos > fc + 2 )
    return 0;
  int i = 0;
  int fn = 2;
  unsigned long parent = mfs_getLong( msb, ( handle ) &dir->parent_inode );
  iHandle dtable = ( iHandle ) mfs_getHandle( msb, ( iHandle ) & dir->table );
  TRACE( "mramfs_readdir: dir_obj @ %08x, dtable @ %08x, inode#%lu, parent inode#%lu, pos=%d\n", ( unsigned ) dir, ( unsigned int ) dtable, dir_inode->i_ino, parent, ( int ) filp->f_pos == 0 );
  if ( filp->f_pos == 0 )
    filldir( dirent, ".", 1, filp->f_pos++, dir_inode->i_ino, DT_UNKNOWN );
  if ( filp->f_pos == 1 )
    filldir( dirent, "..", 2, filp->f_pos++, parent + 1, DT_UNKNOWN );
  while ( i < MRAMFS_DIR_TABLE_SIZE ) {
    mfs_dentry * dentry_obj = ( mfs_dentry * ) mfs_getHandleIndexed( msb, dtable, i );
    while ( dentry_obj != NULL ) {
      char tname[ MRAMFS_MAX_NAME_LEN + 2 ];
      long inode_num = mfs_getLong( msb, ( handle ) & dentry_obj->inode_num );
      int nl = mfs_getShort( msb, ( handle ) & dentry_obj->namelen );
      mfs_MemGet( msb, ( handle ) & dentry_obj->name, tname, nl + 1 );
      TRACE( "mramfs_readdir file#%d/%d, table #%d dentry @ %08x, name='%s', nl=%d, ino=%ld\n", ( int ) filp->f_pos, fn, i, ( unsigned ) dentry_obj, tname, nl, inode_num );
      if ( fn >= filp->f_pos )
        if ( filldir( dirent, tname, nl, filp->f_pos++, inode_num + 1, DT_UNKNOWN ) )
          break;
      fn++;
      dentry_obj = ( mfs_dentry * ) mfs_getHandle( msb, ( iHandle ) & dentry_obj->next );
    }
    i++;
  }
  return 0;
}


static struct address_space_operations mramfs_aops = {
    readpage:
      mramfs_readpage,
    writepage:
      mramfs_writepage,
    prepare_write:
      mramfs_prepare_write,
    commit_write:
      mramfs_commit_write
    };

static struct file_operations mramfs_file_operations = {
    read:
      generic_file_read,
    write:
      generic_file_write,
    mmap:
      generic_file_mmap,
    fsync:
      mramfs_sync_file,
    };

static struct file_operations mramfs_dir_file_operations = {
      //    read: generic_read_dir,
    readdir:
      mramfs_readdir
    };

static struct inode_operations mramfs_file_inode_operations = {
      /*    setattr:
            mramfs_setattr,
    */    truncate:
      mramfs_truncate,
    };

static struct inode_operations mramfs_symlink_operations = {
    follow_link:
      mramfs_follow_link,
    readlink:
      mramfs_readlink,
      /*    setattr:
            mramfs_setattr,
      */
    };

static struct inode_operations mramfs_dir_inode_operations = {
    create:
      mramfs_create,
    lookup:
      mramfs_lookup,
    link:
      mramfs_link,
    unlink:
      mramfs_unlink,
    symlink:
      mramfs_symlink,
    mkdir:
      mramfs_mkdir,
    rmdir:
      mramfs_rmdir,
    mknod:
      mramfs_mknod,
    rename:
      mramfs_rename,
      /*    setattr:
            mramfs_setattr,
      */
    };

static struct super_operations mramfs_ops = {
    put_super:
      mramfs_put_super,
    read_inode:
      mramfs_read_inode,
    write_inode:
      mramfs_write_inode,
    statfs:
      mramfs_statfs,
    put_inode:
      mramfs_put_inode,
    delete_inode:
      mramfs_delete_inode,
    };

static void mramfs_put_super( struct super_block *s ) {
  struct buffer_head *bh = NULL;
  mfs_super *msb = mramfs_getSB( s );
  void *base = msb->sa_region;
  memcpy( base, msb, sizeof( mfs_super ) );
  dump_super( msb );
  int total = msb->sa_blocks * 16;
  int count = 0;
  strcpy( base, "MRAMFS" );
  while ( count < total ) {
    bh = sb_bread( s, count );
    if ( bh == NULL )
      break;
    // printk(KERN_NOTICE "Buffer_head data at %08x\n", bh->b_data);
    memcpy( bh->b_data, base + ( count * 4096 ), 4096 );
    mark_buffer_dirty( bh );
    brelse( bh );
    count++;
  }
  if ( count == total )
    printk( KERN_NOTICE "Image successfully written in %d blocks.\n", total );
  else
    printk( KERN_NOTICE "Couldn't write complete image: %d of %d written.\n", count, total );
  vfree( base );
  kfree( msb );
}

static struct super_block *mramfs_read_super( struct super_block * sb, void * data, int silent ) {
  TRACE( "mramfs_read_super: sb @ %08x, data @ %08x, silent=%d\n", ( int ) sb, ( int ) data, silent );
  if ( global_region != NULL )
    return NULL;
  struct inode * inode = NULL;
  struct dentry * root = NULL;
  void *region = NULL;
  sb->s_blocksize = PAGE_CACHE_SIZE;
  sb->s_blocksize_bits = PAGE_CACHE_SHIFT;
  set_blocksize( sb->s_dev, PAGE_CACHE_SIZE );
  sb->s_magic = MRAMFS_MAGIC;
  sb->s_op = &mramfs_ops;
  if ( ( region = mfs_saAllocateRegion( MRAMFS_REGION_MB ) ) == NULL ) {
    printk( KERN_ERR "FATAL: Couldn't allocate memory region.\n" );
    return NULL;
  }
  global_region = region;
  struct buffer_head *bh = NULL;
  int total = 256 * MRAMFS_REGION_MB;
  int count = 1;
  bh = sb_bread( sb, 0 );
  if ( bh != NULL ) {
    TRACE( "Reading first block.\n" );
    memcpy( region, bh->b_data, 4096 );
    brelse( bh );
    if ( strncmp( region, "MRAMFS", 8 ) == 0 ) {
      TRACE( "Reading remaining blocks." );
      while ( count < total ) {
        bh = sb_bread( sb, count );
        if ( bh == NULL )
          break;
        // printk(KERN_NOTICE "Buffer_head data at %08x\n", bh->b_data);
        memcpy( region + ( count * 4096 ), bh->b_data, 4096 );
        brelse( bh );
        count++;
      }
    } else
      TRACE( "Signature not found.\n" );
  }
  if ( count == total ) {
    printk( KERN_NOTICE "Mounting existing mramfs image.\n" );
    if ( ( sb->u.generic_sbp = mount_fs( sb, region ) ) == NULL )
      return NULL;
  } else {
    printk( KERN_NOTICE "No signature found or read error (%d of %d read)\nCreating mramfs image.\n", count, total );
    if ( ( sb->u.generic_sbp = create_fs( sb, region ) ) == NULL )
      return NULL;
  }
  //If signed, read remainder, mount fs:
  inode = iget( sb, 1 );
  mramfs_decomp_inode( inode );
  TRACE( "Inode #%lu, Mode=%d\n", inode->i_ino, inode->i_mode );
  root = d_alloc_root( inode );
  if ( !root ) {
    iput( inode );
    return NULL;
  }
  sb->s_root = root;
  return sb;
}

void output_compressor_stats( void ) {
  if ( compressors.inode_count == 0 ) {
    printk( KERN_NOTICE "We have never compressed any inodes." );
    return ;
  }
  printk( KERN_NOTICE "OVERALL   GAvg      GMax      GMin\n" );
  printk( KERN_NOTICE "%8d %8ld %8d %8d\n",
          compressors.inode_count,
          compressors.total_bits / compressors.inode_count,
          compressors.max_inode_bits,
          compressors.min_inode_bits );
  printk( KERN_NOTICE "Field GAvg     GMax     GMin\n" );
  printk( KERN_NOTICE "ACL   %8ld %8d %8d\n",
          compressors.i_mode_g.total_bits / compressors.inode_count,
          compressors.i_mode_g.max_bits,
          compressors.i_mode_g.min_bits );
  printk( KERN_NOTICE "Size %8ld %8d %8d\n",
          compressors.i_size_g.total_bits / compressors.inode_count,
          compressors.i_size_g.max_bits,
          compressors.i_size_g.min_bits );

  printk( KERN_NOTICE "Ctime %8ld %8d %8d\n",
          compressors.i_ctime_g.total_bits / compressors.inode_count,
          compressors.i_ctime_g.max_bits,
          compressors.i_ctime_g.min_bits );

  printk( KERN_NOTICE "Mtime %8ld %8d %8d\n",
          compressors.i_mtime_g.total_bits / compressors.inode_count,
          compressors.i_mtime_g.max_bits,
          compressors.i_mtime_g.min_bits );

  printk( KERN_NOTICE "Atime %8ld %8d %8d\n",
          compressors.i_atime_g.total_bits / compressors.inode_count,
          compressors.i_atime_g.max_bits,
          compressors.i_atime_g.min_bits );

  printk( KERN_NOTICE "Links %8ld %8d %8d\n",
          compressors.i_nlink_g.total_bits / compressors.inode_count,
          compressors.i_nlink_g.max_bits,
          compressors.i_nlink_g.min_bits );

  printk( KERN_NOTICE "Vers. %8ld %8d %8d\n\n",
          compressors.i_generation_g.total_bits / compressors.inode_count,
          compressors.i_generation_g.max_bits,
          compressors.i_generation_g.min_bits );
}


static DECLARE_FSTYPE_DEV( mramfs_fs_type, "mramfs", mramfs_read_super );

static int __init init_mramfs_fs( void ) {
  TRACE( "init_mramfs_fs\n" );
  initialize_compressors();
  mramfs_zlib_init( MRAMFS_COMPRESS_BLOCKS ); // replace w/ level
  return register_filesystem( &mramfs_fs_type );
}

static void __exit exit_mramfs_fs( void ) {
  output_compressor_stats();
  if ( global_region != NULL )
    vfree( global_region );
  mramfs_zlib_exit();
  TRACE( "exit_mramfs_fs\n" );
  unregister_filesystem( &mramfs_fs_type );
}

module_init( init_mramfs_fs )
module_exit( exit_mramfs_fs )

MODULE_LICENSE( "GPL" );

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
  set_bit( 0, &msb->freelist_lock );

  for ( i = 0; i < MEM_NLISTS; i++ ) {
    msb->freeList[ i ].hdr = NULL;
    msb->freeList[ i ].nAllocated = msb->freeList[ i ].nFreed = msb->freeList[ i ].nCreated = 0;
  }
  clear_bit( 0, &msb->freelist_lock );
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
  if ( test_and_set_bit( 0, &msb->freelist_lock ) ) {
    printk( KERN_ERR "TRYING TO ALLOCATE WITH MEM_LOCK HELD" );
    return NULL;
  }

  sz = mfs_ChunkList ( chunkSize );
  if ( sz < 0 ) {
    clear_bit( 0, &msb->freelist_lock );
    return ( MEM_NULL );
  }
  if ( msb->freeList[ sz ].hdr == MEM_NULL ) {
    p = mfs_saAllocate( msb );
    if ( p == NULL ) {
      printk ( KERN_ERR "FATAL: out of memory!\n" );
      clear_bit( 0, &msb->freelist_lock );
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
  clear_bit( 0, &msb->freelist_lock );
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
  if ( test_and_set_bit( 0, &msb->freelist_lock ) ) {
    printk( KERN_ERR "TRYING TO FREE WITH MEM_LOCK HELD" );
    return -1;
  }

  int sz = mfs_ChunkList ( chunkSize );
  if ( sz < 0 ) {
    printk ( KERN_ERR "FATAL: trying to free a big chunk (size=%d,list=%d,!\n", chunkSize, sz );
    clear_bit( 0, &msb->freelist_lock );
    return -1;
  }
  mfs_putHandle( msb, ( iHandle ) chunk, msb->freeList[ sz ].hdr );
  msb->freeList[ sz ].hdr = chunk;
  msb->freeList[ sz ].nFreed++;
  clear_bit( 0, &msb->freelist_lock );
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
  if ( msb->sa_freeblocks < 1 )
    return NULL;

  if ( test_and_set_bit( 0, &msb->segment_lock ) ) {
    printk( KERN_ERR "TRYING TO ALLOCATE WITH SEGMENT_LOCK HELD" );
    return NULL;
  }
  int i = 1;
  handle chkpointer = ( handle ) 32768;
  while ( i < msb->sa_blocks ) {
    if ( mfs_getByte( msb, chkpointer + i ) == 0 )
      break;
    i++;
  }
  if ( i >= msb->sa_blocks ) {
    clear_bit( 0, &msb->segment_lock );
    return NULL;
  } else {
    mfs_putByte( msb, chkpointer + i, 1 );
    msb->sa_freeblocks--;
    clear_bit( 0, &msb->segment_lock );
    return ( handle ) ( i << 16 );
  }
}

inline int mfs_saFree( mfs_super *msb, handle to_free ) {
  int i;
  long tofree = ( long ) to_free;
  if ( ( tofree & 0xFFFF ) != 0 ) {
    TRACE( "saFree not called on a segment.\n" );
    return -1;
  }
  int block_num = ( tofree >> 16 );
  if ( !mfs_saIsUsed( msb, block_num ) ) {
    TRACE( "saFree called on an unallocated segment.\n" );
    return -1;
  }
  if ( test_and_set_bit( 0, &msb->segment_lock ) ) {
    printk( KERN_ERR "TRYING TO FREE WITH SEGMENT_LOCK HELD" );
    return -1;
  }
  i = mfs_saMarkUnused( msb, block_num );
  clear_bit( 0, &msb->segment_lock );
  return i;
}

void dump_super( mfs_super *msb ) {
  printk( KERN_NOTICE "---- MSB Dump ----\nCount = %u, RootdirHandle = %08x, ITableHandle = %08x\nInode_Count = %ld, Inode_table_count=%d, Inode_table_size=%d\n",
          msb->count,
          ( unsigned int ) msb->rootdir,
          ( unsigned int ) msb->inodes,
          msb->inode_count,
          msb->inode_table_count,
          msb->inode_table_size
        );
  printk( KERN_NOTICE "acl_table = %08x, acl_hash = %08x\nacl_count = %d, acl_table_size = %d, acl_hash_size = %d\ntotal_space = %ld, file_count = %ld, block_count = %ld\n",
          ( unsigned int ) msb->acl_table,
          ( unsigned int ) msb->acl_hash,
          msb->acl_count,
          msb->acl_table_size,
          msb->acl_hash_size,
          msb->total_space,
          msb->file_count,
          msb->block_count
        );
  printk( KERN_NOTICE "sa_size = %ld, sa_size_mb = %d, sa_blocks = %d, sa_freeblocks = %d\n------\n",
          msb->sa_size,
          msb->sa_size_mb,
          msb->sa_blocks,
          msb->sa_freeblocks
        );
}
