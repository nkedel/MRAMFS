/*
* filecompression.c
*
* (C) Copyright 1999 Linus Torvalds
*
* modified from cramfs interfaces to the uncompression library. There's
* really just three entrypoints:
*
*  - mramfs_zlib_init() - called to initialize the thing.
*  - mramfs_uncompress_exit() - tell me when you're done
*  - mramfs_uncompress_block() - uncompress a block.
*  - mramfs_compress_block() - compress a block.
*
* NOTE NOTE NOTE! The (un)compression is entirely single-threaded. We only
* have one stream, and we'll initialize it only once even if it then is
* used by multiple filesystems. (Of course, we're limited to one mramfs
* mount at a time anyway.)
*
* In practice, this will normally be protected by the cbuf_data spin lock
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/vmalloc.h>
#include <linux/zlib.h>

static z_stream c_stream;
static z_stream d_stream;
static int initialized = 0;
static int local_level;

/* Returns length of decompressed data. */
int mramfs_uncompress_block( void *dst, int dstlen, void *src, int srclen ) {
  int err;

  d_stream.next_in = src;
  d_stream.avail_in = srclen;

  d_stream.next_out = dst;
  d_stream.avail_out = dstlen;

  err = zlib_inflateReset( &d_stream );
  if ( err != Z_OK ) {
    printk( "zlib_inflateReset error %d\n", err );
    zlib_inflateEnd( &d_stream );
    zlib_inflateInit( &d_stream );
  }

  err = zlib_inflate( &d_stream, Z_FINISH );
  if ( err != Z_STREAM_END )
    goto err;
  return d_stream.total_out;

err:
  printk( "Error %d while decompressing!\n", err );
  printk( "%p(%d)->%p(%d)\n", src, srclen, dst, dstlen );
  return 0;
}

/* Returns length of decompressed data. */
int mramfs_compress_block( void *dst, int dstlen, void *src, int srclen ) {
  int err;

  c_stream.next_in = src;
  c_stream.avail_in = srclen;

  c_stream.next_out = dst;
  c_stream.avail_out = dstlen;

  err = zlib_deflateReset( &c_stream );
  if ( err != Z_OK ) {
    printk( "zlib_deflateReset error %d\n", err );
    zlib_deflateEnd( &c_stream );
    zlib_deflateInit( &c_stream, local_level );
  }

  err = zlib_deflate( &c_stream, Z_FINISH );
  if ( err != Z_STREAM_END )
    goto err;
  return c_stream.total_out;

err:
  printk( "Error %d while decompressing!\n", err );
  printk( "%p(%d)->%p(%d)\n", src, srclen, dst, dstlen );
  return 0;
}

int mramfs_zlib_init( int level ) {
  local_level = level;
  if ( !initialized++ ) {
    d_stream.workspace = vmalloc( zlib_inflate_workspacesize() );
    if ( !d_stream.workspace ) {
      initialized = 0;
      return -ENOMEM;
    }
    d_stream.next_in = NULL;
    d_stream.avail_in = 0;
    zlib_inflateInit( &d_stream );
    c_stream.workspace = vmalloc( zlib_deflate_workspacesize() );
    if ( !c_stream.workspace ) {
      initialized = 0;
      return -ENOMEM;
    }
    c_stream.next_in = NULL;
    c_stream.avail_in = 0;
    zlib_deflateInit( &c_stream, level );
  }
  return 0;
}

int mramfs_zlib_exit( void ) {
  if ( !--initialized ) {
    zlib_inflateEnd( &d_stream );
    vfree( d_stream.workspace );
    zlib_deflateEnd( &c_stream );
    vfree( c_stream.workspace );
  }
  return 0;
}

