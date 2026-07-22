#ifndef TYPES_H
#define TYPES_H

// All of our current platforms support the standard int header, so we can just
// use that to remove some of the pain of standard bit width determination
#include <stdint.h>

// On C28x, stdint.h doesn't provide uint8_t/int8_t (no 8-bit types in hardware)
// Only define them if hw_types.h hasn't already been included (via device.h)
#if (defined(__TMS320C28XX__) || defined(__TMS320C28XX_CLA__)) && !defined(HW_TYPES_H)
    typedef uint16_t uint8_t;  // C28x: 8-bit operations use 16-bit registers
    typedef int16_t  int8_t;
#endif

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <WTypes.h>
#else
typedef unsigned long BOOL;
#endif

#ifndef TRUE
# define TRUE  (BOOL)1
#endif // TRUE

#ifndef FALSE
# define FALSE (BOOL)0
#endif // FALSE

#ifndef NULL
# define NULL        0
#endif // NULL

// Fallback for platforms that don't define uint8_t (but not C28x - handled above)
#if !defined(UINT8_MAX) && !defined(__TMS320C28XX__) && !defined(__TMS320C28XX_CLA__)
typedef   signed char  int8_t;
typedef unsigned char uint8_t;
#endif

#ifndef xdc_std__include

// These types conflict with the XDC Tools std.h, so don't include them when building with XDC tools
typedef uint8_t  UInt8;
typedef uint16_t UInt16;
typedef uint32_t UInt32;

    #ifdef UINT64_MAX
    typedef uint64_t UInt64;
    #endif // UINT64_MAX

#endif // xdc_std__include

typedef int8_t   SInt8;
typedef int16_t  SInt16;
typedef uint32_t UInt24;
typedef int32_t  SInt24;
typedef int32_t  SInt32;

#ifdef UINT64_MAX
typedef int64_t  SInt64;
#endif // UINT64_MAX

#endif // TYPES_H
