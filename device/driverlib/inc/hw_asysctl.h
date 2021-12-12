//###########################################################################
//
// FILE:    hw_asysctl.h
//
// TITLE:   Definitions for the ASYSCTL registers.
//
//###########################################################################
// $Copyright:  $
//###########################################################################

#ifndef HW_ASYSCTL_H
#define HW_ASYSCTL_H

//*************************************************************************************************
//
// The following are defines for the ASYSCTL register offsets
//
//*************************************************************************************************
#define ASYSCTL_O_INTOSC1TRIM   0x20U   // Internal Oscillator 1 Trim Register
#define ASYSCTL_O_INTOSC2TRIM   0x22U   // Internal Oscillator 2 Trim Register
#define ASYSCTL_O_TSNSCTL       0x26U   // Temperature Sensor Control Register
#define ASYSCTL_O_LOCK          0x2EU   // Lock Register
#define ASYSCTL_O_ANAREFTRIMA   0x36U   // Analog Reference Trim A Register
#define ASYSCTL_O_ANAREFTRIMB   0x38U   // Analog Reference Trim B Register
#define ASYSCTL_O_ANAREFTRIMC   0x3AU   // Analog Reference Trim C Register
#define ASYSCTL_O_ANAREFTRIMD   0x3CU   // Analog Reference Trim D Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the INTOSC1TRIM register
//
//*************************************************************************************************
#define ASYSCTL_INTOSC1TRIM_VALFINETRIM_S   0U
#define ASYSCTL_INTOSC1TRIM_VALFINETRIM_M   0xFFFU   // Oscillator Value Fine Trim Bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the INTOSC2TRIM register
//
//*************************************************************************************************
#define ASYSCTL_INTOSC2TRIM_VALFINETRIM_S   0U
#define ASYSCTL_INTOSC2TRIM_VALFINETRIM_M   0xFFFU   // Oscillator Value Fine Trim Bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the TSNSCTL register
//
//*************************************************************************************************
#define ASYSCTL_TSNSCTL_ENABLE   0x1U   // Temperature Sensor Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the LOCK register
//
//*************************************************************************************************
#define ASYSCTL_LOCK_TSNSCTL       0x8U         // Temperature Sensor Control Register Lock
#define ASYSCTL_LOCK_ANAREFTRIMA   0x800000U    // Analog Reference A Trim Register Lock
#define ASYSCTL_LOCK_ANAREFTRIMB   0x1000000U   // Analog Reference B Trim Register Lock
#define ASYSCTL_LOCK_ANAREFTRIMC   0x2000000U   // Analog Reference C Trim Register Lock
#define ASYSCTL_LOCK_ANAREFTRIMD   0x4000000U   // Analog Reference D Trim Register Lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the ANAREFTRIMA register
//
//*************************************************************************************************
#define ASYSCTL_ANAREFTRIMA_BGVALTRIM_S     0U
#define ASYSCTL_ANAREFTRIMA_BGVALTRIM_M     0x3FU     // Bandgap Value Trim
#define ASYSCTL_ANAREFTRIMA_BGSLOPETRIM_S   6U
#define ASYSCTL_ANAREFTRIMA_BGSLOPETRIM_M   0x7C0U    // Bandgap Slope Trim
#define ASYSCTL_ANAREFTRIMA_IREFTRIM_S      11U
#define ASYSCTL_ANAREFTRIMA_IREFTRIM_M      0xF800U   // Reference Current Trim

//*************************************************************************************************
//
// The following are defines for the bit fields in the ANAREFTRIMB register
//
//*************************************************************************************************
#define ASYSCTL_ANAREFTRIMB_BGVALTRIM_S     0U
#define ASYSCTL_ANAREFTRIMB_BGVALTRIM_M     0x3FU     // Bandgap Value Trim
#define ASYSCTL_ANAREFTRIMB_BGSLOPETRIM_S   6U
#define ASYSCTL_ANAREFTRIMB_BGSLOPETRIM_M   0x7C0U    // Bandgap Slope Trim
#define ASYSCTL_ANAREFTRIMB_IREFTRIM_S      11U
#define ASYSCTL_ANAREFTRIMB_IREFTRIM_M      0xF800U   // Reference Current Trim

//*************************************************************************************************
//
// The following are defines for the bit fields in the ANAREFTRIMC register
//
//*************************************************************************************************
#define ASYSCTL_ANAREFTRIMC_BGVALTRIM_S     0U
#define ASYSCTL_ANAREFTRIMC_BGVALTRIM_M     0x3FU     // Bandgap Value Trim
#define ASYSCTL_ANAREFTRIMC_BGSLOPETRIM_S   6U
#define ASYSCTL_ANAREFTRIMC_BGSLOPETRIM_M   0x7C0U    // Bandgap Slope Trim
#define ASYSCTL_ANAREFTRIMC_IREFTRIM_S      11U
#define ASYSCTL_ANAREFTRIMC_IREFTRIM_M      0xF800U   // Reference Current Trim

//*************************************************************************************************
//
// The following are defines for the bit fields in the ANAREFTRIMD register
//
//*************************************************************************************************
#define ASYSCTL_ANAREFTRIMD_BGVALTRIM_S     0U
#define ASYSCTL_ANAREFTRIMD_BGVALTRIM_M     0x3FU     // Bandgap Value Trim
#define ASYSCTL_ANAREFTRIMD_BGSLOPETRIM_S   6U
#define ASYSCTL_ANAREFTRIMD_BGSLOPETRIM_M   0x7C0U    // Bandgap Slope Trim
#define ASYSCTL_ANAREFTRIMD_IREFTRIM_S      11U
#define ASYSCTL_ANAREFTRIMD_IREFTRIM_M      0xF800U   // Reference Current Trim



#endif
