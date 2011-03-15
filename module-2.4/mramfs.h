/*
*  mramfs.h
*
* Current version 12 March 2004
* First release version 7 Dec 2003
*
* Nate Edel (nate@cse.ucsc.edu)
* Computer System Research Group
* University of California, Santa Cruz
*
* This file is released under the GPL.
*/

#ifndef _inode_h_
#define _inode_h_

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/locks.h>
#include <linux/dcache.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

// #define MRAMFS_DEBUG 1
// #ifdef MRAMFS_DEBUG
// #define TRACE(x...)   printk(KERN_NOTICE x)
// #else
#define TRACE(x...)
// #endif

// const unsigned int bitMask[33];
#define MRAMFS_MAGIC            0x1024EDE7
#define GAMMA_MAX_BUCKETS       16
#define MRAMFS_INODE_BLOCK_SIZE 256
#define MRAMFS_DIR_TABLE_SIZE   256
#define MRAMFS_MAX_NAME_LEN     110
#define BLOCK_ADDR_MASK         0xFFFFFFF0

#define MEG (1024 * 1024);

typedef unsigned char* handle;
typedef handle* iHandle;

typedef struct mramfs_inode_block {
  unsigned char i_len[ 16 ];
  unsigned short length;
  unsigned char data[ 0 ];
}
i_block;

/* Allocate things in 16 KB chunks  ---   2^14 = 16KB */
#define SEGMENTBITS 16
#define MEM_NULL    ((void *)0)
#define MEM_NLISTS  12
#define MEM_MINSIZE 4

/*
 * This structure for each list header keeps track of the number of
 * chunks allocated, the number of chunks freed, and the total number of
 * chunks created.  nAllocated - nFreed = number in use.
 */
typedef struct {
  handle hdr;
  int nAllocated;
  int nFreed;
  int nCreated;
}
freeListHdr;

typedef struct gammacompressor {
  unsigned int bucketStart[ GAMMA_MAX_BUCKETS + 1 ];
  unsigned char bits[ GAMMA_MAX_BUCKETS ];
  long total_bits;
  int min_bits;
  int max_bits;
}
gammacompressor;

#define hashsize(n) ((ub4)1<<(n))
#define hashmask(n) (hashsize(n)-1)

typedef unsigned long int ub4;   /* unsigned 4-byte quantities */
typedef unsigned char ub1;

typedef struct {
  int max_inode_bits;
  int min_inode_bits;
  int inode_count;
  long total_bits;
  gammacompressor i_mode_g;         /* File mode */
  gammacompressor i_uid_g;          /* Low 16 bits of Owner Uid */
  gammacompressor i_gid_g;          /* Low 16 bits of Group Id */
  gammacompressor i_size_g;         /* Size in bytes */
  gammacompressor i_atime_g;        /* Access time */
  gammacompressor i_ctime_g;        /* Creation time */
  gammacompressor i_mtime_g;        /* Modification time */
  gammacompressor i_nlink_g;  /* Links \count */
  gammacompressor i_pointer_g;
  gammacompressor i_flags_g;        /* File flags */
  gammacompressor i_generation_g;   /* File version (for NFS) */
  gammacompressor i_file_acl_g;     /* File ACL */
  gammacompressor i_dir_acl_g;      /* Directory ACL */
}
compressor_list;

typedef struct mramfs_acl_record {
  handle next;
  long hashvalue;
  long table_number;
  unsigned short uid;
  unsigned short gid;
  unsigned short mode;
}
mfs_acl;


/*
 * A rather space-inefficient data block implementation.
 */
typedef struct mfd_struct {
  handle next;
  handle data;
  unsigned short clength;
  unsigned short dlength;
  unsigned long offset;  // do we really need this?
}
mfs_file_data;


/*
 * Directory entry data structure.  Currently an unpacked linked list.
 */
typedef struct mfsdentry {
  unsigned long inode_num;
  handle next;
  short namelen;
  char name[ MRAMFS_MAX_NAME_LEN + 2 ];
}
mfs_dentry;

/*
 * Directory structure: Basically a hash
 */
typedef struct {
  long file_count;
  long inode_num;
  long parent_inode;
  iHandle table;
}
mfs_directory;


/*
 *  Example global data structure --
 *  I _think_ it's kosher that everything is in the context structure.
 */
typedef struct mramfs_superblock {
  struct super_block *vfs_sb;        // not a handle; VFS structure
  unsigned char *sa_region;        // NOT a handle; this is a real pointer
  unsigned count;
  handle rootdir;
  iHandle inodes;
  long inode_count;
  int inode_table_count;
  int inode_table_size;
  iHandle acl_table;
  iHandle acl_hash;
  int acl_count;
  int acl_table_size;
  int acl_hash_size;        // MUST BE A POWER OF TWO
  long total_space;
  long file_count;
  long block_count;
  long sa_size;                // size in bytes
  int sa_size_mb;                // size in megabytes
  int sa_blocks;                // size in segments (64k blocks)
  int sa_freeblocks;        // free segments (64k blocks)
  atomic_t freelist_lock;
  atomic_t segment_lock;
  spinlock_t inode_lock;
  spinlock_t directory_lock;
  spinlock_t cbuf_inode_lock;
  spinlock_t cbuf_data_lock;
  i_block *cbuf_inode;
  unsigned char *cbuf_data;
  int compress_inodes;
  int compress_blocks;
  freeListHdr freeList[ MEM_NLISTS ];
}
mfs_super;

inline int compress_gamma( gammacompressor *gc, unsigned int value, unsigned
                           int *data, int offset, int _signed );

inline int uncompress_gamma( gammacompressor *cdata, unsigned int *val,
                             unsigned int *compressed, int nbits, int _signed );

int mramfs_split_path( char *to_split, char *dir, char *file, int len );
inline int find_separator( char *path, int i );
inline int mramfs_is_absolute( char *path );
inline int calcbits( unsigned int i );

int comp_inode_gamma( struct inode *inode, unsigned int *dest, int bits );
int decomp_inode_gamma( struct inode *inode, unsigned int *dest, int bits );
void initialize_compressors( void );

ub4 hash( ub1 *k, ub4 length, ub4 initval );
ub4 hash_ugp( unsigned short uid, unsigned short gid, unsigned short mode );

void mfs_MemInit ( mfs_super * );
handle mfs_MemAllocate ( mfs_super *, int );
int mfs_MemFree ( mfs_super *, int, handle );

inline int mfs_MemGet ( mfs_super *, handle offset, void *buffer, int size );
// inline int mfs_MemGetIndirect ( mfs_super *, iHandle offset, void *buffer, int size );
inline int mfs_MemPut ( mfs_super *, handle offset, const void *buffer, int size );
inline int mfs_MemSet( mfs_super *msb, const handle region, unsigned char c, int count );
inline int mfs_MemMove ( mfs_super *msb, handle dest, const handle src, int size );

inline unsigned char mfs_getByte( mfs_super *, handle offset );
inline short mfs_getShort( mfs_super *msb, handle offset );
inline unsigned short mfs_getUShort( mfs_super *msb, handle offset );
inline int mfs_getInt( mfs_super *msb, handle offset );
inline unsigned int mfs_getUInt( mfs_super *msb, handle offset );
inline long mfs_getLong( mfs_super *msb, handle offset );
inline unsigned long mfs_getULong( mfs_super *msb, handle offset );

inline int mfs_putByte( mfs_super *, handle offset, unsigned char c );
inline int mfs_putShort( mfs_super *msb, handle offset, short s );
inline int mfs_putUShort( mfs_super *msb, handle offset, unsigned short s );
inline int mfs_putInt( mfs_super *msb, handle offset, int i );
inline int mfs_putUInt( mfs_super *msb, handle offset, unsigned int i );
inline int mfs_putULong( mfs_super *msb, handle offset, unsigned long l );
inline int mfs_putLong( mfs_super *msb, handle offset, long l );

inline int mfs_putHandle( mfs_super *msb, iHandle offset, handle h );
inline handle mfs_getHandle( mfs_super *msb, iHandle offset );
inline handle mfs_getHandleIndexed( mfs_super *msb, iHandle offset, int index );
inline int mfs_putHandleIndexed( mfs_super *msb, iHandle offset, int index, handle h );

void *create_fs( struct super_block *vfs_sb, unsigned char *region );

void * mfs_saAllocateRegion( int mb );
inline int mfs_saMarkUsed( mfs_super *msb, int block );
inline int mfs_saMarkUnused( mfs_super *msb, int block );
inline int mfs_saIsUsed( mfs_super *msb, int block );
inline int mfs_saFree( mfs_super *msb, handle to_free );
inline handle mfs_saAllocate( mfs_super *msb );
int mfs_saInit( mfs_super *msb );

void mfs_MemDumpStats ( mfs_super * );

inline void gamma_PutOneBit ( unsigned int bit, unsigned int *s, int *curbit );
inline void gamma_PutBits ( unsigned int val, int nbits, unsigned int *s, int *curbit );

static void mramfs_put_super( struct super_block *s );

inline
unsigned int
gamma_GetOneBit ( int *curbit, const unsigned int *s );

inline
unsigned int
gamma_GetBits ( int *curbit, const unsigned int *s, int nbits );

void gamma_InitCompressor ( gammacompressor *, const int * );
void gamma_Compress ( unsigned int *, int *, const int,
                      gammacompressor *,
                      const unsigned int *, const int );
void gamma_CompressOne ( unsigned int *, int *, const int,
                         gammacompressor *,
                         const unsigned int );
int gamma_Uncompress ( const unsigned int *, int *,
                       const int, const gammacompressor *,
                       unsigned int *, const int );
int gamma_UncompressCount ( const unsigned int *, int *,
                            const int, const gammacompressor *,
                            unsigned int *, const int );
unsigned int gamma_UncompressOne ( const unsigned int *, int *,
                                   const int, const gammacompressor * );
inline unsigned int
gamma_UncompressOnce ( const gammacompressor *gc,
                       const unsigned int *compressed, int *curbit, const int sgn );

inline void
gamma_CompressOnce ( gammacompressor *gc, const unsigned int val,
                     unsigned int *compressed, int *curbit, const int sgn );

void output_compressor_stats( void );

handle mramfs_create_new_dir( mfs_super *msb );

void dump_super( mfs_super *msb );

/* Returns length of decompressed data. */
int mramfs_uncompress_block( void *dst, int dstlen, void *src, int srclen );

/* Returns length of decompressed data. */
int mramfs_compress_block( void *dst, int dstlen, void *src, int srclen );
int mramfs_zlib_init( int level );
int mramfs_zlib_exit( void );

#ifdef REAL
const unsigned int millenium = 946684800;
const unsigned int bitMask[ 33 ] = {
                                     0,
                                     0x1, 0x3, 0x7, 0xf,
                                     0x1f, 0x3f, 0x7f, 0xff,
                                     0x1ff, 0x3ff, 0x7ff, 0xfff,
                                     0x1fff, 0x3fff, 0x7fff, 0xffff,
                                     0x1ffff, 0x3ffff, 0x7ffff, 0xfffff,
                                     0x1fffff, 0x3fffff, 0x7fffff, 0xffffff,
                                     0x1ffffff, 0x3ffffff, 0x7ffffff, 0xfffffff,
                                     0x1fffffff, 0x3fffffff, 0x7fffffff, 0xffffffff
                                   };

const unsigned int blankMask[ 33 ] = {
                                       0xFFFFFFFF,
                                       0xFFFFFFFE, 0xFFFFFFFC, 0xFFFFFFF8, 0xFFFFFFF0,
                                       0xFFFFFFE0, 0xFFFFFFC0, 0xFFFFFF80, 0xFFFFFF00,
                                       0xFFFFFE00, 0xFFFFFC00, 0xFFFFF800, 0xFFFFF000,
                                       0xFFFFE000, 0xFFFFC000, 0xFFFF8000, 0xFFFF0000,
                                       0xFFFE0000, 0xFFFC0000, 0xFFF80000, 0xFFF00000,
                                       0xFFE00000, 0xFFC00000, 0xFF800000, 0xFF000000,
                                       0xFE000000, 0xFC000000, 0xF8000000, 0xF0000000,
                                       0xE0000000, 0xC0000000, 0x80000000, 0x0
                                     };

const unsigned short setmasks[] = {
                                    0x0001,     // 1
                                    0x0002,     // 2
                                    0x0004,     // 3
                                    0x0008,     // 4
                                    0x0010,     // 5
                                    0x0020,     // 6
                                    0x0040,     // 7
                                    0x0080,     // 8
                                    0x0100,     // 9
                                    0x0200,     // 10
                                    0x0400,     // 11
                                    0x0800,     // 12
                                    0x1000,     // 13
                                    0x2000,     // 14
                                    0x4000,     // 15
                                    0x8000 // 16
                                  };

const unsigned short resetmasks[] = {
                                      0xfffe,
                                      0xfffd,
                                      0xfffb,
                                      0xfff7,
                                      0xffef,
                                      0xffdf,
                                      0xffbf,
                                      0xff7f,
                                      0xfeff,
                                      0xfdff,
                                      0xfbff,
                                      0xf7ff,
                                      0xefff,
                                      0xdfff,
                                      0xbfff,
                                      0x7fff
                                    };

const unsigned int bitsize[] = {
                                 1,
                                 2,
                                 4,
                                 8,
                                 16,
                                 32,
                                 64,
                                 128,
                                 256,
                                 512,
                                 1024,
                                 2048,
                                 4096,
                                 8192,
                                 16384,
                                 32768,
                                 65536,
                                 131072,
                                 262144,
                                 524288,
                                 1048576,
                                 2097152,
                                 4194304,
                                 8388608,
                                 16777216,
                                 33554432,
                                 67108864,
                                 134217728,
                                 268435456,
                                 536870912,
                                 0x40000000,
                                 0x80000000
                               };
#endif
#ifndef REAL
extern const unsigned int bitMask[];
extern const unsigned int millenium;
extern const unsigned int blankMask[];
extern const unsigned short setmasks[];
extern const unsigned short resetmasks[];
extern const unsigned int bitsize[];
#endif
// !REAL

#endif
// _inode_h_
