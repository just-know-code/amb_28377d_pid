//###########################################################################
//
// FILE:   pin_map.h
//
// TITLE:  Legacy definitions of pin mux info for gpio.c.
//
//###########################################################################
// $Copyright:  $
//###########################################################################

#ifndef __PIN_MAP_LEGACY_H__
#define __PIN_MAP_LEGACY_H__


#include "pin_map.h"

//*****************************************************************************
// Legacy pinmuxing MACROS - Retained for portability across devices ONLY
// Not recommended for new users
//*****************************************************************************
#define GPIO_16_SD_D1                   GPIO_16_SD1_D1

#define GPIO_17_SD_C1                   GPIO_17_SD1_C1

#define GPIO_18_SD_D2                   GPIO_18_SD1_D2

#define GPIO_19_SD_C2                   GPIO_19_SD1_C2

#define GPIO_20_SD_D3                   GPIO_20_SD1_D3

#define GPIO_21_SD_C3                   GPIO_21_SD1_C3

#define GPIO_22_SD_D4                   GPIO_22_SD1_D4

#define GPIO_23_SD_C4                   GPIO_23_SD1_C4

#define GPIO_24_SD_D5                   GPIO_24_SD2_D1

#define GPIO_25_SD_C5                   GPIO_25_SD2_C1

#define GPIO_26_SD_D6                   GPIO_26_SD2_D2

#define GPIO_27_SD_C6                   GPIO_27_SD2_C2

#define GPIO_28_SD_D7                   GPIO_28_SD2_D3

#define GPIO_29_SD_C7                   GPIO_29_SD2_C3

#define GPIO_30_SD_D8                   GPIO_30_SD2_D4

#define GPIO_31_SD_C8                   GPIO_31_SD2_C4

#define GPIO_36_EM1WAIT1                GPIO_36_EM1WAIT

#define GPIO_110_EM2WAIT1               GPIO_110_EM2WAIT

#define GPIO_115_EM2CS0                 GPIO_115_EM2CS0N

#define GPIO_116_EM2CS2                 GPIO_116_EM2CS2N

#define GPIO_132_ODMSE0                 GPIO_132_ODMSE1

#endif // __PIN_MAP_LEGACY_H__
