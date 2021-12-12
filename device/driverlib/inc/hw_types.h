//###########################################################################
//
// FILE:   hw_types.h
//
// TITLE:  Type definitions used in driverlib functions.
//
//###########################################################################
// $Copyright:  $
//###########################################################################

#ifndef HW_TYPES_H
#define HW_TYPES_H

//*****************************************************************************
//
// Macros for hardware access
//
//*****************************************************************************
#define HWREG(x)                                                              \
        (*((volatile uint32_t *)((uintptr_t)(x))))
#define HWREG_BP(x)                                                           \
        __byte_peripheral_32((uint32_t *)(x))
#define HWREGH(x)                                                             \
        (*((volatile uint16_t *)((uintptr_t)(x))))
#define HWREGB(x)                                                             \
        __byte((int16_t *)(x),0)

//*****************************************************************************
//
// SUCCESS and FAILURE for API return value
//
//*****************************************************************************
#define STATUS_S_SUCCESS    (0)
#define STATUS_E_FAILURE    (-1)

//*****************************************************************************
//
// Definition of 8 bit types for USB Driver code to maintain portability
// between byte and word addressable cores of C2000 Devices.
//
//*****************************************************************************
typedef uint16_t uint8_t;
typedef int16_t int8_t;

//****************************************************************************
//
// For checking NULL pointers
//
//****************************************************************************
#ifndef NULL
#define NULL ((void *)0x0)
#endif

//*****************************************************************************
//
// 32-bit & 64-bit float type
//
//*****************************************************************************
#ifndef C2000_IEEE754_TYPES
#define C2000_IEEE754_TYPES
#ifdef __TI_EABI__
typedef float         float32_t;
typedef double        float64_t;
#else // TI COFF
typedef float         float32_t;
typedef long double   float64_t;
#endif // __TI_EABI__
#endif // C2000_IEEE754_TYPES


//*****************************************************************************
//
// Emulated Bitbanded write
//
//*****************************************************************************
#define HWREGBITW(address, mask, value)                                       \
        (*(volatile uint32_t *)(address)) =                                   \
       ((*(volatile uint32_t *)(address)) & ~((uint32_t)1 << mask))           \
       | ((uint32_t)value << mask)

#define HWREGBITHW(address, mask, value)                                       \
        (*(volatile uint16_t *)(address)) =                                    \
       ((*(volatile uint16_t *)(address)) & ~((uint16_t)1 << mask))            \
       | ((uint16_t)value << mask)

//*****************************************************************************
//
// Emulated Bitbanded read
//
//*****************************************************************************
#define HWREGBITR(address, mask)                                              \
        (((*(volatile uint32_t *)(address)) & ((uint32_t)1 << mask)) >> mask)

#define HWREGBITHR(address, mask)                                              \
        (((*(volatile uint16_t *)(address)) & ((uint16_t)1 << mask)) >> mask)

//*****************************************************************************
//
// Extern compiler intrinsic prototypes. See compiler User's Guide for details.
// These are provided to satisfy static analysis tools. The #ifndef is required
// because the '&' is for a C++-style reference, and although it is the correct
// prototype, it will not build in C code.
//
//*****************************************************************************
#if(defined(__TMS320C28XX__) || defined(__TMS320C28XX_CLA__))
#else
extern int16_t &__byte(int16_t *array, uint16_t byte_index);
extern uint32_t &__byte_peripheral_32(uint32_t *x);
#endif

#endif // HW_TYPES_H
