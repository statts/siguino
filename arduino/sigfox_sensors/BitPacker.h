#ifndef BITPACKER_H_INCLUDED
#define BITPACKER_H_INCLUDED

#define BIT(n)                  ( 1ULL<<(n) ) //UL = unsigned long, forces chip to use 32bit int not 16

#define BIT_SET(y, mask)        ( y |=  (mask) )
#define BIT_CLEAR(y, mask)      ( y &= ~(mask) )
#define BIT_FLIP(y, mask)       ( y ^=  (mask) )

/*
        Set bits        Clear bits      Flip bits
y        0x0011          0x0011          0x0011
mask     0x0101 |        0x0101 &~       0x0101 ^
        ---------       ----------      ---------
result   0x0111          0x0010          0x0110
*/

//! Create a bitmask of length \a len.
#define BIT_MASK(len)           ( BIT(len)-1 )

//! Create a bitfield mask of length \a starting at bit \a start.
#define BF_MASK(start, len)     ( BIT_MASK(len)<<(start) )

//! Prepare a bitmask for insertion or combining.
#define BF_PREP(x, start, len)  ( ((x)&BIT_MASK(len)) << (start) )


//! Extract a bitfield of length \a len starting at bit \a start from \a y.
#define BF_GET(y, start, len)   ( ((y)>>(start)) & BIT_MASK(len) )

//! Insert a new bitfield value \a x into \a y.
#define BF_SET(y, x, start, len)    \
    ( y= ((y) &~ BF_MASK(start, len)) | BF_PREP(x, start, len) )

namespace BitPacker {
    static uint32_t get_packed_message_32(unsigned int values[], unsigned int bits_used[], int num_vals){
        uint32_t retval = 0x0;
        int j = 0;
        for (int i=0;i<num_vals;i++){
            BF_SET(retval, values[i], j, j + bits_used[i]);
            j += bits_used[i];
        }
        return retval;
    }

    static uint64_t get_packed_message_64(unsigned int values[], unsigned int bits_used[], int num_vals){
        uint64_t retval = 0x0;
        int j = 0;
        for (int i=0;i<num_vals;i++){
            BF_SET(retval, values[i], j, j + bits_used[i]);
            j += bits_used[i];
        }
        return retval;
    }

}
#endif // BITPACKER_H_INCLUDED
