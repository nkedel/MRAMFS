//
// gammacompress.c
//
//	Algorithms for compressing and decompressing data using various
//	mechanisms.
//
// Copyright 1998, 2003 by Ethan Miller (elm@cs.ucsc.edu)
// University of Maryland Baltimore County
// / University of California, Santa Cruz
//
// Check with Ethan before distributing this version
//
// Modified for mramfs by Nate Edel (nate@cse.ucsc.edu)
//

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/dcache.h>
#include <asm/uaccess.h>
#include "mramfs.h"

extern compressor_list compressors;



//----------------------------------------------------------------------
//
//	gamma_InitCompressor
//
//	Create a compressor from the bucket lengths passed.  The sequence
//	of bucket widths is terminated by a negative number.
//
//----------------------------------------------------------------------
void
gamma_InitCompressor ( gammacompressor *g, const int* bucketWidths ) {
  int	i;
  int	curMin = 0;

  for ( i = 0; bucketWidths[ i ] >= 0; i++ ) {
    g->bits[ i ] = bucketWidths[ i ];
    g->bucketStart[ i ] = curMin;
    curMin += 1 << g->bits[ i ];
  }
  g->bucketStart[ i ] = curMin;
  g->bits[ i ] = 32;
  g->bucketStart[ i + 1 ] = 0xffffffff;
  g->total_bits = 0;
  g->max_bits = 0;
  g->min_bits = 99;
}


/*----------------------------------------------------------------------
 *
 * gammagamma_PutBits
 * gammagamma_PutOneBit
 *
 * Gamma encode the value and stick it onto the end of the bit string.
 * There's an optimized version for a single bit.
 *
 *----------------------------------------------------------------------
 */

inline
void
gamma_PutOneBit ( unsigned int bit, unsigned int *s, int *curbit ) {
  if ( bit & 1 ) {
    s[ ( *curbit ) >> 5 ] |= 1 << ( ( 31 - ( ( *curbit ) & 31 ) ) );
  }
  *curbit += 1;
}


inline
void
gamma_PutBits ( unsigned int val, int nbits, unsigned int *s, int *curbit ) {
  int	bitsInWord;

  val &= bitMask[ nbits ];
  while ( nbits > 0 ) {
    bitsInWord = 32 - ( ( *curbit ) & 31 );
    if ( bitsInWord == 32 ) {
      s[ ( *curbit ) >> 5 ] = val << ( 32 - nbits );
      ( *curbit ) += nbits;
      nbits = 0;
    } else if ( nbits >= bitsInWord ) {
      // More bits left than we have space for in this word
      s[ ( *curbit ) >> 5 ] |= val >> ( nbits - bitsInWord );
      nbits -= bitsInWord;
      ( *curbit ) += bitsInWord;
    } else {
      s[ ( *curbit ) >> 5 ] |= val << ( bitsInWord - nbits );
      ( *curbit ) += nbits;
      nbits = 0;
    }
  }
}



inline
unsigned int
gamma_GetOneBit ( int *curbit, const unsigned int *s ) {
  unsigned int v;

  v = ( s[ ( *curbit ) >> 5 ] >> ( 31 - ( ( *curbit ) & 31 ) ) ) & 1;
  *curbit += 1;
  return ( v );
}

inline
unsigned int
gamma_GetBits ( int *curbit, const unsigned int *s, int nbits ) {
  unsigned int	v = 0;
  int	bitsInWord;

  while ( nbits > 0 ) {
    bitsInWord = 32 - ( ( *curbit ) & 31 );
    if ( nbits < bitsInWord ) {
      // All of the bits can come from here.
      v <<= nbits;
      v |= ( s[ ( *curbit ) >> 5 ] >> ( bitsInWord - nbits ) ) & bitMask[ nbits ];
      ( *curbit ) += nbits;
      nbits = 0;
    } else {
      // gamma_Get the rest of the bits in this word
      v <<= bitsInWord;
      v |= s[ ( *curbit ) >> 5 ] & bitMask[ bitsInWord ];
      nbits -= bitsInWord;
      ( *curbit ) += bitsInWord;
    }
  }
  return ( v );
}

inline
int
gammaFindZero ( int *curbit, const unsigned int *s ) {
  int	totalBits;

  for ( totalBits = 1; ( s[ ( *curbit ) >> 5 ] & ( 1 << ( 31 - ( ( *curbit ) & 31 ) ) ) ) != 0;
        ( *curbit ) += 1 ) {
    totalBits += 1;
  }
  ( *curbit ) += 1;
  return ( totalBits );
}



/*----------------------------------------------------------------------
 *
 * gammaCompressOnce
 *
 * Compress a single value into the bitstring.  This is called by several
 * other routines to do multi-value compression, which is why it's inlined.
 *
 *----------------------------------------------------------------------
 */
inline
void
gamma_CompressOnce ( gammacompressor *gc, const unsigned int val,
                     unsigned int *compressed, int *curbit, const int sgn ) {
  int	j, cnt, full = 0;
  unsigned int	v = val;
  unsigned int	signVal = 0;
  unsigned int	fullVal = 0;
  int startbit = *curbit;

  // Correct for negative numbers if necessary
  if ( sgn && ( v & 0x80000000 ) ) {
    signVal = 0xf;
    v = ( ~v ) + 1;
  }
  for ( j = 0; gc->bits[ j ] < 32; j++ ) {
    if ( v < gc->bucketStart[ j + 1 ] ) {
      break;
    }
  }
  cnt = gc->bits[ j ];
  if ( cnt == 32 ) {
    cnt = 31;
    if ( v & 0x80000000 )
      fullVal = 0xf;
    v &= 0x7fffffff;
    full = 1;
    if ( v & 0x80000000 )
      fullVal = 0xf;
  }
  //  if (j > 12) printf("v = %u, bucketStart[%d]=%u, bucketStart[%d]=%u, bits[%d]=%d\n\n", v, j, gc->bucketStart[j], j+1, gc->bucketStart[j+1], j, gc->bits[j]);

  // Output a unary string corresponding to the bucket number
  gamma_PutBits ( 0xfffffe, j + 1, compressed, curbit );
  // Compute the value to store as a binary, and output it.
  gamma_PutBits ( v - gc->bucketStart[ j ], cnt, compressed, curbit );
  // Output the sign value if requested
  if ( sgn ) {
    gamma_PutOneBit ( signVal, compressed, curbit );
  }
  if ( full ) {
    gamma_PutOneBit ( fullVal, compressed, curbit );
  }
  if ( gc->max_bits < ( *curbit - startbit ) )
    gc->max_bits = ( *curbit - startbit );
  if ( gc->min_bits > ( *curbit - startbit ) )
    gc->min_bits = ( *curbit - startbit );
  gc->total_bits += ( *curbit - startbit );
}

//----------------------------------------------------------------------
//
//	gamma_Compress
//
//	Compress a sequence of integers using gamma compression.  The
//	array "compressed" must be large enough to hold the result.  Bits
//	are stored left (MSB) to right (LSB).  Current bitstring position
//	is updated.
//
//----------------------------------------------------------------------
void
gamma_Compress ( unsigned int *compressed, int *curbit, const int sgn,
                 gammacompressor *gc,
                 const unsigned int *vals, const int nvals ) {
  int	i;

  for ( i = 0; i < nvals; i++ ) {
    gamma_CompressOnce ( gc, vals[ i ], compressed, curbit, sgn );
  }
}

void
gamma_CompressOne ( unsigned int *compressed, int *curbit, const int sgn,
                    gammacompressor *gc, const unsigned int val ) {
  gamma_CompressOnce ( gc, val, compressed, curbit, sgn );
}


/***********************************************************************
 *
 * gammaUncompressOnce
 *
 * This inline routine uncompresses a single value and returns it
 * directly.  It's called repeatedly by various routines to uncompress a
 * value
 ***********************************************************************
 */
inline unsigned int
gamma_UncompressOnce ( const gammacompressor *gc,
                       const unsigned int *compressed, int *curbit, const int sgn ) {
  int full = 0;
  int	b, v, cnt;
  int	signbit, fullbit;
  // gamma_Get the unary prefix
  b = gammaFindZero ( curbit, compressed ) - 1;
  cnt = gc->bits[ b ];
  if ( cnt == 32 ) {
    cnt = 31;
    full = 1;
  }
  v = gamma_GetBits ( curbit, compressed, cnt ) + gc->bucketStart[ b ];
  if ( sgn ) {
    // gamma_Get the sign bit
    signbit = gamma_GetOneBit ( curbit, compressed );
    if ( signbit != 0 ) {
      v = ( ~v ) + 1;
    }
  }
  if ( full ) {
    // gamma_Get the sign bit
    fullbit = gamma_GetOneBit ( curbit, compressed );
    if ( fullbit != 0 ) {
      v |= 0x80000000;
    }
  }
  return ( v );
}

//----------------------------------------------------------------------
//
//	GammaUncompress
//	GammaUncompressCount
//	GammaUncompressOne
//
//	Uncompress a bit string into integers.  Return the number of
//	values placed into the values array.  Note that the vals array
//	must have enough space to accommodate all of the values.
//
//----------------------------------------------------------------------
int
gamma_Uncompress ( const unsigned int *compressed, int *curbit,
                   const int sgn, const gammacompressor *gc,
                   unsigned int *vals, const int nbits ) {
  int	i = 0;
  int	maxbits = nbits + *curbit;

  while ( *curbit < maxbits ) {
    vals[ i++ ] = gamma_UncompressOnce ( gc, compressed, curbit, sgn );
  }

  return ( i );
}

int
gamma_UncompressCount ( const unsigned int *compressed, int *curbit,
                        const int sgn, const gammacompressor *gc,
                        unsigned int *vals, const int nvals ) {
  int	i;

  for ( i = 0; i < nvals; i++ ) {
    vals[ i ] = gamma_UncompressOnce ( gc, compressed, curbit, sgn );
  }
  return ( nvals );
}

unsigned int
gamma_UncompressOne ( const unsigned int *compressed, int *curbit,
                      const int sgn, const gammacompressor *gc ) {
  return ( gamma_UncompressOnce ( gc, compressed, curbit, sgn ) );
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
