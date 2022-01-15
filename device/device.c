//#############################################################################
//
// FILE:   device.c
//
// TITLE:  Device setup for examples.
//
//#############################################################################
//
// $Release Date: $
// $Copyright:
// Copyright (C) 2013-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "device.h"
#include "driverlib.h"
#include "inc/hw_ipc.h"
#include <string.h>

#ifdef __cplusplus
using std::memcpy;
#endif

#define OVERSAMPLING_TIMES 4
uint16_t sampling_times;
volatile uint16_t loop_sel;
/*
* 0b0000 two loops are open loop, 0b11 two loops are closed loop
* 0b01 only current is closed loop, 0b10 only position loop is closed loop
*/
volatile uint16_t pos_pid_sel;
/*
* 0b00000 all PID operations do not work
* 0b00001 only the first PID works
* 0b00110 front radial directions PID work, 6
* 0b11000 trailing radial directions PID work, 24
* 0b11110 all radial directions PID work, 30(2 4 8 16)
*/
struct pi_t currentLoopPI;
struct pid_t pid_tArray[5];
float posIntegralArray[5];
float currIntegralArray[10];
uint16_t rotorPosition[5];
uint16_t coilCurrent[10];
uint16_t pwmDuty[10];
uint16_t forwardFirstPos[5];
uint16_t refCurrent[10];
uint16_t refPosition[5];
uint16_t coilBiasCurrent[5];
uint32_t rawPosData[5];
uint32_t rawCurrData[10];



//*****************************************************************************
//
// Function to initialize the device. Primarily initializes system control to a
// known state by disabling the watchdog, setting up the SYSCLKOUT frequency,
// and enabling the clocks to the peripherals.
//
//*****************************************************************************
void Device_init(void)
{
	//
	// Disable the watchdog
	//
	SysCtl_disableWatchdog();

#ifdef _FLASH
	//
	// Copy time critical code and flash setup code to RAM. This includes the
	// following functions: InitFlash();
	//
	// The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
	// are created by the linker. Refer to the device .cmd file.
	//
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

	//
	// Call Flash Initialization to setup flash waitstates. This function must
	// reside in RAM.
	//
	Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif
#ifdef CPU1

	//
	// Configure Analog Trim in case of untrimmed or TMX sample
	//
	if((SysCtl_getDeviceParametric(SYSCTL_DEVICE_QUAL) == 0x0U) &&
			(HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMA) == 0x0U))
	{
		Device_configureTMXAnalogTrim();
	}

	//
	// Set up PLL control and clock dividers
	//
	SysCtl_setClock(DEVICE_SETCLOCK_CFG);

	//
	// Make sure the LSPCLK divider is set to the default (divide by 4)
	//
	SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4);

	//
	// These asserts will check that the #defines for the clock rates in
	// device.h match the actual rates that have been configured. If they do
	// not match, check that the calculations of DEVICE_SYSCLK_FREQ and
	// DEVICE_LSPCLK_FREQ are accurate. Some examples will not perform as
	// expected if these are not correct.
	//
	ASSERT(SysCtl_getClock(DEVICE_OSCSRC_FREQ) == DEVICE_SYSCLK_FREQ);
	ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == DEVICE_LSPCLK_FREQ);

#ifndef _FLASH
	//
	// Call Device_cal function when run using debugger
	// This function is called as part of the Boot code. The function is called
	// in the Device_init function since during debug time resets, the boot code
	// will not be executed and the gel script will reinitialize all the
	// registers and the calibrated values will be lost.
	// Sysctl_deviceCal is a wrapper function for Device_Cal
	//
	SysCtl_deviceCal();
#endif

#endif
	//
	// Turn on all peripherals
	//
	Device_enableAllPeripherals();


}

//*****************************************************************************
//
// Function to turn on all peripherals, enabling reads and writes to the
// peripherals' registers.
//
// Note that to reduce power, unused peripherals should be disabled.
//
//*****************************************************************************
void Device_enableAllPeripherals(void)
{
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
#ifdef CPU1
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRPWM);
#endif
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

#ifdef CPU1
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EMIF1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EMIF2);
#endif

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM9);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM10);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM11);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM12);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP4);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP5);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP6);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP3);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD1);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD2);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIC);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCID);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIC);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CB);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANB);

	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCBSPA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCBSPB);

#ifdef CPU1
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_USBA);

SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_UPPA);
#endif

SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCD);

SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS5);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS6);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS8);

SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACB);
SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACC);
}

//*****************************************************************************
//
// Function to disable pin locks on GPIOs.
//
//*****************************************************************************
void Device_initGPIO(void)
{
	//
	// Disable pin locks.
	//
	GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFF);
	GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFF);
	GPIO_unlockPortConfig(GPIO_PORT_C, 0xFFFFFFFF);
	GPIO_unlockPortConfig(GPIO_PORT_D, 0xFFFFFFFF);
	GPIO_unlockPortConfig(GPIO_PORT_E, 0xFFFFFFFF);
	GPIO_unlockPortConfig(GPIO_PORT_F, 0xFFFFFFFF);

	//
	// Enable GPIO Pullups
	//
	Device_enableUnbondedGPIOPullups();
}

//*****************************************************************************
//
// Function to enable pullups for the unbonded GPIOs on the 176PTP package:
// GPIOs     Grp Bits
// 95-132    C   31
//           D   31:0
//           E   4:0
// 134-168   E   31:6
//           F   8:0
//
//*****************************************************************************

void Device_enableUnbondedGPIOPullupsFor176Pin(void)
{
	EALLOW;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPCPUD) = ~0x80000000U;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPDPUD) = ~0xFFFFFFF7U;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPEPUD) = ~0xFFFFFFDFU;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPFPUD) = ~0x000001FFU;
	EDIS;
}

//*****************************************************************************
//
// Function to enable pullups for the unbonded GPIOs on the 100PZ package:
// GPIOs     Grp Bits
// 0-1       A   1:0
// 5-9       A   9:5
// 22-40     A   31:22
//           B   8:0
// 44-57     B   25:12
// 67-68     C   4:3
// 74-77     C   13:10
// 79-83     C   19:15
// 93-168    C   31:29
//           D   31:0
//           E   31:0
//           F   8:0
//
//*****************************************************************************
void Device_enableUnbondedGPIOPullupsFor100Pin(void)
{
	EALLOW;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) = ~0xFFC003E3U;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) = ~0x03FFF1FFU;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPCPUD) = ~0xE10FBC18U;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPDPUD) = ~0xFFFFFFF7U;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPEPUD) = ~0xFFFFFFFFU;
	HWREG(GPIOCTRL_BASE + GPIO_O_GPFPUD) = ~0x000001FFU;
	EDIS;
}

//*****************************************************************************
//
// Function to enable pullups for the unbonded GPIOs on the 100PZ or
// 176PTP package.
//
//*****************************************************************************
void Device_enableUnbondedGPIOPullups(void)
{
	//
	// bits 8-10 have pin count
	//
	uint16_t pinCount = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
			(uint32_t)SYSCTL_PARTIDL_PIN_COUNT_M) >>
			SYSCTL_PARTIDL_PIN_COUNT_S);

	/*
	 * 5 = 100 pin
	 * 6 = 176 pin
	 * 7 = 337 pin
	 */
	if(pinCount == 5)
	{
		Device_enableUnbondedGPIOPullupsFor100Pin();
	}
	else if(pinCount == 6)
	{
		Device_enableUnbondedGPIOPullupsFor176Pin();
	}
	else
	{
		//
		// Do nothing - this is 337 pin package
		//
	}
}

#ifdef CPU1
//*****************************************************************************
//
// Function to implement Analog trim of TMX devices
//
//*****************************************************************************
void Device_configureTMXAnalogTrim(void)
{
	//
	// Enable ADC clock
	//
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
	SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCD);

	//
	// Configure ADC reference trim for TMX devices
	//
	EALLOW;
	HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMA) = 0x7BDDU;
	HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMB) = 0x7BDDU;
	HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMC) = 0x7BDDU;
	HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMD) = 0x7BDDU;

	//
	// Configure ADC offset trim. The user should generate the trim values
	// by following the instructions in the "ADC Zero Offset Calibration"
	// section in device TRM. The below lines needs to be uncommented and
	// updated with the correct trim values.
	//
	//    HWREGH(ADCA_BASE + ADC_O_OFFTRIM) = 0x0U;
	//    HWREGH(ADCB_BASE + ADC_O_OFFTRIM) = 0x0U;
	//    HWREGH(ADCC_BASE + ADC_O_OFFTRIM) = 0x0U;
	//    HWREGH(ADCD_BASE + ADC_O_OFFTRIM) = 0x0U;

	//
	// Configure internal oscillator trim. If the internal oscillator trim
	// contains all zeros, the user can adjust the lowest 10 bits of the
	// oscillator trim register between 1 (minimum) and 1023 (maximum)
	// while observing the system clock on the XCLOCKOUT pin. The below
	// lines needs to be uncommented and updated with the correct trim values.
	//
	//    if(HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC1TRIM) == 0x0U)
	//    {
	//        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC1TRIM) = 0x0U;
	//    }
	//    if( HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC2TRIM) = 0x0U)
	//    {
	//        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC2TRIM) = 0x0U;
	//    }

	EDIS;

	//
	// Disable ADC clock
	//
	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
	SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCD);
}

//*****************************************************************************
//! Executes a CPU02 control system bootloader.
//!
//! \param bootMode specifies which CPU02 control system boot mode to execute.
//!
//! This function will allow the CPU01 master system to boot the CPU02 control
//! system via the following modes: Boot to RAM, Boot to Flash, Boot via SPI,
//! SCI, I2C, or parallel I/O. This function blocks and waits until the
//! control system boot ROM is configured and ready to receive CPU01 to CPU02
//! IPC INT0 interrupts. It then blocks and waits until IPC INT0 and
//! IPC FLAG31 are available in the CPU02 boot ROM prior to sending the
//! command to execute the selected bootloader.
//!
//! The \e bootMode parameter accepts one of the following values:
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_PARALLEL
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_SCI
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_SPI
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_I2C
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_CAN
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_RAM
//!  - \b C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH
//!
//! \return 0 (success) if command is sent, or 1 (failure) if boot mode is
//! invalid and command was not sent.
//
//*****************************************************************************
uint16_t
Device_bootCPU2(uint32_t bootMode)
{
	uint32_t bootStatus;
	uint16_t pin;
	uint16_t returnStatus = STATUS_PASS;

	//
	// If CPU2 has already booted, return a fail to let the application
	// know that something is out of the ordinary.
	//
	bootStatus = HWREG(IPC_BASE + IPC_O_BOOTSTS) & 0x0000000FU;

	if(bootStatus == C2_BOOTROM_BOOTSTS_C2TOC1_BOOT_CMD_ACK)
	{
		//
		// Check if MSB is set as well
		//
		bootStatus = ((uint32_t)(HWREG(IPC_BASE + IPC_O_BOOTSTS) &
				0x80000000U)) >> 31U;

		if(bootStatus != 0)
		{
			returnStatus = STATUS_FAIL;

			return returnStatus;
		}
	}

	//
	// Wait until CPU02 control system boot ROM is ready to receive
	// CPU01 to CPU02 INT1 interrupts.
	//
	do
	{
		bootStatus = HWREG(IPC_BASE + IPC_O_BOOTSTS) &
				C2_BOOTROM_BOOTSTS_SYSTEM_READY;
	} while ((bootStatus != C2_BOOTROM_BOOTSTS_SYSTEM_READY));

	//
	// Loop until CPU02 control system IPC flags 1 and 32 are available
	//
	while (((HWREG(IPC_BASE + IPC_O_FLG) & IPC_FLG_IPC0)  != 0U) ||
			((HWREG(IPC_BASE + IPC_O_FLG) & IPC_FLG_IPC31) != 0U))
	{

	}

	if (bootMode >= C1C2_BROM_BOOTMODE_BOOT_COMMAND_MAX_SUPPORT_VALUE)
	{
		returnStatus = STATUS_FAIL;
	}
	else
	{
		//
		// Based on boot mode, enable pull-ups on peripheral pins and
		// give GPIO pin control to CPU02 control system.
		//
		switch (bootMode)
		{
		case C1C2_BROM_BOOTMODE_BOOT_FROM_SCI:

			//
			//SCIA connected to CPU02
			//
			SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI, 1,
					SYSCTL_CPUSEL_CPU2);

			//
			//Allows CPU02 bootrom to take control of clock
			//configuration registers
			//
			EALLOW;
			HWREG(CLKCFG_BASE + SYSCTL_O_CLKSEM) = 0xA5A50000U;
			HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) = 0x0002U;
			EDIS;

			GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
			GPIO_setQualificationMode(29, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_29_SCITXDA);
			GPIO_setMasterCore(29, GPIO_CORE_CPU2);

			GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(28, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_28_SCIRXDA);
			GPIO_setMasterCore(28, GPIO_CORE_CPU2);

			break;

		case C1C2_BROM_BOOTMODE_BOOT_FROM_SPI:

			//
			//SPI-A connected to CPU02
			//
			SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL6_SPI, 1,
					SYSCTL_CPUSEL_CPU2);

			//
			//Allows CPU02 bootrom to take control of clock configuration
			// registers
			//
			EALLOW;
			HWREG(CLKCFG_BASE + SYSCTL_O_CLKSEM) = 0xA5A50000U;
			EDIS;

			GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(16, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_16_SPISIMOA);
			GPIO_setMasterCore(16, GPIO_CORE_CPU2);

			GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(17, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_17_SPISOMIA);
			GPIO_setMasterCore(17, GPIO_CORE_CPU2);

			GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(18, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_18_SPICLKA);
			GPIO_setMasterCore(18, GPIO_CORE_CPU2);

			GPIO_setDirectionMode(19, GPIO_DIR_MODE_OUT);
			GPIO_setQualificationMode(19, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_19_GPIO19);
			GPIO_setMasterCore(19, GPIO_CORE_CPU2);

			break;

		case C1C2_BROM_BOOTMODE_BOOT_FROM_I2C:

			//
			//I2CA connected to CPU02
			//
			SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL7_I2C, 1,
					SYSCTL_CPUSEL_CPU2);

			//
			//Allows CPU2 bootrom to take control of clock
			//configuration registers
			//
			EALLOW;
			HWREG(CLKCFG_BASE + SYSCTL_O_CLKSEM) = 0xA5A50000U;
			HWREG(CLKCFG_BASE + SYSCTL_O_LOSPCP) = 0x0002U;
			EDIS;

			GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(32, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_32_SDAA);
			GPIO_setMasterCore(32, GPIO_CORE_CPU2);

			GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(33, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_33_SCLA);
			GPIO_setMasterCore(33, GPIO_CORE_CPU2);

			break;

		case C1C2_BROM_BOOTMODE_BOOT_FROM_PARALLEL:

			for(pin=58;pin<=65;pin++)
			{
				GPIO_setDirectionMode(pin, GPIO_DIR_MODE_IN);
				GPIO_setQualificationMode(pin, GPIO_QUAL_ASYNC);
				GPIO_setMasterCore(pin, GPIO_CORE_CPU2);
			}

			GPIO_setDirectionMode(69, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(69, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_69_GPIO69);
			GPIO_setMasterCore(69, GPIO_CORE_CPU2);

			GPIO_setDirectionMode(70, GPIO_DIR_MODE_IN);
			GPIO_setQualificationMode(70, GPIO_QUAL_ASYNC);
			GPIO_setPinConfig(GPIO_70_GPIO70);
			GPIO_setMasterCore(70, GPIO_CORE_CPU2);

			break;


		case C1C2_BROM_BOOTMODE_BOOT_FROM_CAN:
			//
			//Set up the GPIO mux to bring out CANATX on GPIO71
			//and CANARX on GPIO70
			//
			GPIO_unlockPortConfig(GPIO_PORT_C, 0xFFFFFFFFU);

			GPIO_setMasterCore(71, GPIO_CORE_CPU2);
			GPIO_setPinConfig(GPIO_71_CANTXA);
			GPIO_setQualificationMode(71, GPIO_QUAL_ASYNC);

			GPIO_setMasterCore(70, GPIO_CORE_CPU2);
			GPIO_setPinConfig(GPIO_70_CANRXA);
			GPIO_setQualificationMode(70, GPIO_QUAL_ASYNC);


			GPIO_lockPortConfig(GPIO_PORT_C, 0xFFFFFFFFU);

			//
			// Set CANA Bit-Clock Source Select = SYSCLK and enable CAN
			//
			EALLOW;
			HWREG(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) &=
					SYSCTL_CLKSRCCTL2_CANABCLKSEL_M;
			EDIS;
			SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);

			break;

		}

		//
		//CPU01 to CPU02 IPC Boot Mode Register
		//
		HWREG(IPC_BASE + IPC_O_BOOTMODE) = bootMode;

		//
		// CPU01 To CPU02 IPC Command Register
		//
		HWREG(IPC_BASE + IPC_O_SENDCOM) = BROM_IPC_EXECUTE_BOOTMODE_CMD;

		//
		// CPU01 to CPU02 IPC flag register
		//
		HWREG(IPC_BASE + IPC_O_SET) = 0x80000001U;

	}
	return returnStatus;
}
#endif // #ifdef CPU1
//*****************************************************************************
//
// Error handling function to be called when an ASSERT is violated
//
//*****************************************************************************
void __error__(char *filename, uint32_t line)
{
	//
	// An ASSERT condition was evaluated as false. You can use the filename and
	// line parameters to determine what went wrong.
	//
	ESTOP0;
}


static void UpdatePWMDuty(){

	EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, pwmDuty[0]);
	EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, pwmDuty[1]);

	EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, pwmDuty[2]);
	EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, pwmDuty[3]);

	EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, pwmDuty[4]);
	EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, pwmDuty[5]);

	EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, pwmDuty[6]);
	EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_B, pwmDuty[7]);

	EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, pwmDuty[8]);
	EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, pwmDuty[9]);
}

#define POSITION_CLOSED_LOOP
#define CURRENT_CLOSED_LOOP

__interrupt void INT_curADCD_1_ISR(void){

	if (sampling_times == 0){
		//
		// Get rotor position
		//
		//GET ADC A
		rawPosData[0] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
		rawPosData[1] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
		rawPosData[2] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);

		//GET ADC B
		rawPosData[3] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
		rawPosData[4] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);

		//
		// Get coil current
		//
		//GET ADC C
		rawCurrData[2] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
		rawCurrData[3] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
		rawCurrData[4] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);

		//GET ADC D
		rawCurrData[5] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
		rawCurrData[6] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
		rawCurrData[7] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);
		rawCurrData[8] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER3);
		rawCurrData[9] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER4);
		rawCurrData[0] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER5);
		rawCurrData[1] = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER6);
	} else {
		rawPosData[0] += ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
		rawPosData[1] += ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
		rawPosData[2] += ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2);

		rawPosData[3] += ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
		rawPosData[4] += ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);

		rawCurrData[2] += ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
		rawCurrData[3] += ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
		rawCurrData[4] += ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2);

		rawCurrData[5] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
		rawCurrData[6] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
		rawCurrData[7] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER2);
		rawCurrData[8] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER3);
		rawCurrData[9] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER4);
		rawCurrData[0] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER5);
		rawCurrData[1] += ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER6);
	}
	//
	// Clear the interrupt flag
	//
	ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
	//
	// Check if overflow has occurred
	//
	//	if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1)){
	//		ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
	//		ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
	//	}
	sampling_times++;
	if (sampling_times == OVERSAMPLING_TIMES){
	
	
		uint16_t index;
		for(index = 0; index < 5; index++)
			rotorPosition[index] = rawPosData[index] / OVERSAMPLING_TIMES;

		for(index = 0; index < 10; index++)
			coilCurrent[index] = rawCurrData[index] / OVERSAMPLING_TIMES;

		//compute displacement loop pid
#ifdef POSITION_CLOSED_LOOP
		if (loop_sel & 0b10){
			if (pos_pid_sel & 0b00001)
				CalculPID(0);
			if (pos_pid_sel & 0b00010)
				CalculPID(1);
			if (pos_pid_sel & 0b00100)
				CalculPID(2);
			if (pos_pid_sel & 0b01000)
				CalculPID(3);
			if (pos_pid_sel & 0b10000)
				CalculPID(4);
		}
#endif
		//compute current loop pi
#ifdef CURRENT_CLOSED_LOOP
		if (loop_sel & 0b01){

			uint16_t i;
			for(i = 0; i < 10; i++){
				CalculPI(i);
			}
		}
#endif
		// update pwm duty
		UpdatePWMDuty();
		
		sampling_times = 0;
	} else {
		ADC_forceMultipleSOC(ADCA_BASE, ADC_FORCE_SOC0 |
										ADC_FORCE_SOC1 |
										ADC_FORCE_SOC2);
		ADC_forceMultipleSOC(ADCB_BASE, ADC_FORCE_SOC0 |
										ADC_FORCE_SOC1);
		ADC_forceMultipleSOC(ADCC_BASE, ADC_FORCE_SOC0 |
										ADC_FORCE_SOC1 |
										ADC_FORCE_SOC2);
		ADC_forceMultipleSOC(ADCD_BASE, ADC_FORCE_SOC0 |
										ADC_FORCE_SOC1 |
										ADC_FORCE_SOC2 |
										ADC_FORCE_SOC3 |
										ADC_FORCE_SOC4 |
										ADC_FORCE_SOC5 |
										ADC_FORCE_SOC6);
	}
	//
	// Acknowledge the interrupt
	//
	Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
	
}


#define CONTROL_PERIOD 0.00005f
#define MAX_POS_INTEGRAL 500.0f
#define MIN_POS_INTEGRAL -500.0f
#define MAX_CONTROL_CURRENT 1000
#define MIN_CONTROL_CURRENT -1000

void CalculPID(uint16_t index){

	float propotion, differential;
	int16_t pos_loop_outcome, error, firstOrderDiff;
	error = (int16_t)(refPosition[index] - rotorPosition[index]); /* 平衡位置与设定点的差值 */
	firstOrderDiff = (int16_t)(rotorPosition[index] - forwardFirstPos[index]); /* 相邻两点之间的差值 */
	posIntegralArray[index] += error * CONTROL_PERIOD;

	forwardFirstPos[index] = rotorPosition[index];
	propotion = pid_tArray[index].P * error;
	differential = pid_tArray[index].D * firstOrderDiff / CONTROL_PERIOD;

	if (posIntegralArray[index] > MAX_POS_INTEGRAL)
		posIntegralArray[index] = MAX_POS_INTEGRAL;
	if (posIntegralArray[index] < MIN_POS_INTEGRAL)
		posIntegralArray[index] = MIN_POS_INTEGRAL;
	// outcome * 1200 / 2000
	pos_loop_outcome = (int16_t)(propotion + pid_tArray[index].I * posIntegralArray[index] + differential);
	if (pos_loop_outcome > MAX_CONTROL_CURRENT)
		pos_loop_outcome = MAX_CONTROL_CURRENT;
	if (pos_loop_outcome < MIN_CONTROL_CURRENT)
		pos_loop_outcome = MIN_CONTROL_CURRENT;
	pos_loop_outcome = (int16_t)(pos_loop_outcome * 0.6);

	refCurrent[index * 2] = coilBiasCurrent[index] - pos_loop_outcome;
	refCurrent[index * 2 + 1] = coilBiasCurrent[index] + pos_loop_outcome;

	/*
	 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 *轴向：
	 *		【传感器0】0 号线圈			1号线圈
	 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 *径向A：			       【传感器1】
	 * 					2 号线圈
	 *
	 * 	      【传感器2】4号线圈			5号线圈
	 *
	 * 					3 号线圈
	 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 *径向B:
	 *				        【传感器3】
	 * 					6 号线圈
	 *
	 * 	       【传感器4】8 号线圈			9号线圈
	 *
	 * 					7 号线圈
	 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 * */
}

#define MAX_PWM_DUTY 4000
#define MIN_PWM_DUTY 1000
#define MAX_CURR_INTEGRAL 1000.0f
#define MIN_CURR_INTEGRAL -1000.0f
void CalculPI(uint16_t index){

    int16_t propotion, integral, outcome, error;
	error = (refCurrent[index] - coilCurrent[index]); 	// 平衡位置与设定点的差值
	currIntegralArray[index] +=  error * CONTROL_PERIOD;
	propotion = (int16_t)(currentLoopPI.P * error);

	if (currIntegralArray[index] > MAX_CURR_INTEGRAL)
		currIntegralArray[index] = MAX_CURR_INTEGRAL;
	if (currIntegralArray[index] < MIN_CURR_INTEGRAL)
		currIntegralArray[index] = MIN_CURR_INTEGRAL;
	integral = (int16_t)(currentLoopPI.I * currIntegralArray[index]);
	outcome = (propotion + integral) + EPWM_TIMER_TBPRD / 2;
	if (outcome > MAX_PWM_DUTY)
		outcome = MAX_PWM_DUTY;
	if (outcome < MIN_PWM_DUTY)
		outcome = MIN_PWM_DUTY;
	pwmDuty[index] = outcome;
}

//*****************************************************************************
//
// initialize all variable and array
//
//*****************************************************************************
void Variable_init(){

	uint16_t i;

	for(i = 0; i < 5; i++){
		posIntegralArray[i] = 0;
		forwardFirstPos[i] = 0;
		refPosition[i] = 49385;
	}
	for(i = 0; i < 10; i++){
		currIntegralArray[i] = 0;
		pwmDuty[i] = 2500;
		refCurrent[i] = 2048;
	}

	currentLoopPI.P = 10;
	currentLoopPI.I = 0.038;

	coilBiasCurrent[0] = 2648;
	coilBiasCurrent[1] = 2648;
	coilBiasCurrent[2] = 2648;
	coilBiasCurrent[3] = 2648;
	coilBiasCurrent[4] = 2648;

	refPosition[0] = 1765;
	refPosition[1] = 1765;
	refPosition[2] = 1765;
	refPosition[3] = 1765;
	refPosition[4] = 2121;

	pid_tArray[0].P = 2;
	pid_tArray[0].I = 10;
	pid_tArray[0].D = 0.0001;

	pid_tArray[1].P = 2;
	pid_tArray[1].I = 10;
	pid_tArray[1].D = 0.0001;

	pid_tArray[2].P = 0.5;
	pid_tArray[2].I = 10;
	pid_tArray[2].D = 0.0001;

	pid_tArray[3].P = 1.1;
	pid_tArray[3].I = 10;
	pid_tArray[3].D = 0.0001;

	pid_tArray[4].P = 0.9;
	pid_tArray[4].I = 10;
	pid_tArray[4].D = 0.0001;

	sampling_times = 0;
	loop_sel = 0b11;
	pos_pid_sel = 0b00000;

#ifndef POSITION_CLOSED_LOOP
	for(i = 0; i < 10; i++){
		refCurrent[i] = 2048;
	}
#endif
	//compute current loop pi
#ifndef CURRENT_CLOSED_LOOP
	for(i = 0; i < 10; i++){

		pwmDuty[i] = 2500;
	}
#endif

}



