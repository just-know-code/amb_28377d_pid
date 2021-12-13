/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "board.h"


void Board_init()
{
	EALLOW;

	PinMux_init();
	ADC_init();
	CAN_init();
	EMIF1_init();
	EPWM_init();
	GPIO_init();
	I2C_init();
	SCI_init();
//	SPI_init();
	USB_init();

	EDIS;
}

void PinMux_init()
{
	//
	// CANA -> myCAN0 Pinmux
	//
	GPIO_setPinConfig(GPIO_62_CANRXA);
	GPIO_setPinConfig(GPIO_63_CANTXA);
	//
	// EMIF1 -> myEMIF10 Pinmux
	//
	GPIO_setPinConfig(GPIO_34_EM1CS2N);
	GPIO_setPinConfig(GPIO_35_EM1CS3N);
	GPIO_setPinConfig(GPIO_29_EM1SDCKE);
	GPIO_setPinConfig(GPIO_30_EM1CLK);
	GPIO_setPinConfig(GPIO_31_EM1WEN);
	GPIO_setPinConfig(GPIO_36_EM1WAIT);
	GPIO_setPinConfig(GPIO_37_EM1OEN);
	GPIO_setPinConfig(GPIO_93_EM1BA0);
	GPIO_setPinConfig(GPIO_92_EM1BA1);
	GPIO_setPinConfig(GPIO_38_EM1A0);
	GPIO_setPinConfig(GPIO_39_EM1A1);
	GPIO_setPinConfig(GPIO_40_EM1A2);
	GPIO_setPinConfig(GPIO_41_EM1A3);
	GPIO_setPinConfig(GPIO_44_EM1A4);
	GPIO_setPinConfig(GPIO_45_EM1A5);
	GPIO_setPinConfig(GPIO_46_EM1A6);
	GPIO_setPinConfig(GPIO_47_EM1A7);
	GPIO_setPinConfig(GPIO_48_EM1A8);
	GPIO_setPinConfig(GPIO_49_EM1A9);
	GPIO_setPinConfig(GPIO_50_EM1A10);
	GPIO_setPinConfig(GPIO_51_EM1A11);
	GPIO_setPinConfig(GPIO_52_EM1A12);
	GPIO_setPinConfig(GPIO_86_EM1A13);
	GPIO_setPinConfig(GPIO_87_EM1A14);
	GPIO_setPinConfig(GPIO_88_EM1A15);
	GPIO_setPinConfig(GPIO_89_EM1A16);
	GPIO_setPinConfig(GPIO_90_EM1A17);
	GPIO_setPinConfig(GPIO_91_EM1A18);
	GPIO_setPinConfig(GPIO_85_EM1D0);
	GPIO_setPinConfig(GPIO_83_EM1D1);
	GPIO_setPinConfig(GPIO_82_EM1D2);
	GPIO_setPinConfig(GPIO_81_EM1D3);
	GPIO_setPinConfig(GPIO_80_EM1D4);
	GPIO_setPinConfig(GPIO_79_EM1D5);
	GPIO_setPinConfig(GPIO_78_EM1D6);
	GPIO_setPinConfig(GPIO_77_EM1D7);
	GPIO_setPinConfig(GPIO_76_EM1D8);
	GPIO_setPinConfig(GPIO_75_EM1D9);
	GPIO_setPinConfig(GPIO_74_EM1D10);
	GPIO_setPinConfig(GPIO_73_EM1D11);
	GPIO_setPinConfig(GPIO_72_EM1D12);
	GPIO_setPinConfig(GPIO_71_EM1D13);
	GPIO_setPinConfig(GPIO_70_EM1D14);
	GPIO_setPinConfig(GPIO_69_EM1D15);
	//
	// EPWM1 -> myEPWM0 Pinmux
	//
	GPIO_setPinConfig(GPIO_0_EPWM1A);
	GPIO_setPinConfig(GPIO_1_EPWM1B);
	//
	// EPWM2 -> myEPWM1 Pinmux
	//
	GPIO_setPinConfig(GPIO_2_EPWM2A);
	GPIO_setPinConfig(GPIO_3_EPWM2B);
	//
	// EPWM3 -> myEPWM2 Pinmux
	//
	GPIO_setPinConfig(GPIO_4_EPWM3A);
	GPIO_setPinConfig(GPIO_5_EPWM3B);
	//
	// EPWM4 -> myEPWM3 Pinmux
	//
	GPIO_setPinConfig(GPIO_6_EPWM4A);
	GPIO_setPinConfig(GPIO_7_EPWM4B);
	//
	// EPWM5 -> myEPWM4 Pinmux
	//
	GPIO_setPinConfig(GPIO_8_EPWM5A);
	GPIO_setPinConfig(GPIO_9_EPWM5B);
	//
	// EPWM6 -> myEPWM5 Pinmux
	//
	GPIO_setPinConfig(GPIO_10_EPWM6A);
	GPIO_setPinConfig(GPIO_11_EPWM6B);
	//
	// EPWM7 -> myEPWM6 Pinmux
	//
	GPIO_setPinConfig(GPIO_12_EPWM7A);
	GPIO_setPinConfig(GPIO_13_EPWM7B);
	//
	// EPWM8 -> myEPWM7 Pinmux
	//
	GPIO_setPinConfig(GPIO_14_EPWM8A);
	GPIO_setPinConfig(GPIO_15_EPWM8B);
	// GPIO53 -> myGPIO0 Pinmux
	GPIO_setPinConfig(GPIO_53_GPIO53);
	// GPIO66 -> myGPIO1 Pinmux
	GPIO_setPinConfig(GPIO_66_GPIO66);
	// GPIO67 -> myGPIO2 Pinmux
	GPIO_setPinConfig(GPIO_67_GPIO67);
	// GPIO68 -> myGPIO3 Pinmux
	GPIO_setPinConfig(GPIO_68_GPIO68);
	// GPIO84 -> myGPIO4 Pinmux
	GPIO_setPinConfig(GPIO_84_GPIO84);
	// GPIO94 -> myGPIO5 Pinmux
	GPIO_setPinConfig(GPIO_94_GPIO94);
	// GPIO99 -> myGPIO6 Pinmux
	GPIO_setPinConfig(GPIO_99_GPIO99);
	// GPIO133 -> myGPIO7 Pinmux
	GPIO_setPinConfig(GPIO_133_GPIO133);
	// GPIO20 -> myGPIO8 Pinmux
	GPIO_setPinConfig(GPIO_20_GPIO20);
	// GPIO21 -> myGPIO9 Pinmux
	GPIO_setPinConfig(GPIO_21_GPIO21);
	// GPIO22 -> myGPIO10 Pinmux
	GPIO_setPinConfig(GPIO_22_GPIO22);
	// GPIO23 -> myGPIO11 Pinmux
	GPIO_setPinConfig(GPIO_23_GPIO23);
	// GPIO24 -> myGPIO12 Pinmux
	GPIO_setPinConfig(GPIO_24_GPIO24);
	// GPIO25 -> myGPIO13 Pinmux
	GPIO_setPinConfig(GPIO_25_GPIO25);
	// GPIO26 -> myGPIO14 Pinmux
	GPIO_setPinConfig(GPIO_26_GPIO26);
	// GPIO27 -> myGPIO15 Pinmux
	GPIO_setPinConfig(GPIO_27_GPIO27);
	//
	// I2CA -> myI2C0 Pinmux
	//
	GPIO_setPinConfig(GPIO_32_SDAA);
	GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(32, GPIO_QUAL_ASYNC);
	GPIO_setPinConfig(GPIO_33_SCLA);
	GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
	GPIO_setQualificationMode(33, GPIO_QUAL_ASYNC);
	//
	// SCIA -> mySCI0 Pinmux
	//
	GPIO_setPinConfig(GPIO_28_SCIRXDA);
	GPIO_setPinConfig(GPIO_65_SCITXDA);
	//
	// SCIB -> mySCI1 Pinmux
	//
	GPIO_setPinConfig(GPIO_55_SCIRXDB);
	GPIO_setPinConfig(GPIO_54_SCITXDB);
	//
	// SCIC -> mySCI2 Pinmux
	//
	GPIO_setPinConfig(GPIO_57_SCIRXDC);
	GPIO_setPinConfig(GPIO_56_SCITXDC);
	//
	// SPIA -> mySPI0 Pinmux
	//
	GPIO_setPinConfig(GPIO_16_SPISIMOA);
	GPIO_setPinConfig(GPIO_17_SPISOMIA);
	GPIO_setPinConfig(GPIO_18_SPICLKA);
	GPIO_setPinConfig(GPIO_19_SPISTEA);
	//
	// SPIB -> mySPI1 Pinmux
	//
	GPIO_setPinConfig(GPIO_60_SPISIMOB);
	GPIO_setPinConfig(GPIO_61_SPISOMIB);
	GPIO_setPinConfig(GPIO_58_SPICLKB);
	GPIO_setPinConfig(GPIO_59_SPISTEB);
	// USB pinmux
	GPIO_setAnalogMode(42, GPIO_ANALOG_ENABLED);
	GPIO_setAnalogMode(43, GPIO_ANALOG_ENABLED);

}


void ADC_init(){
	//myADC0 initialization

	// ADC Initialization: Write ADC configurations and power up the ADC
	// Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(myADC0_BASE, ADC_CLK_DIV_1_0);
	// Configures the analog-to-digital converter resolution and signal mode.
	ADC_setMode(myADC0_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(myADC0_BASE, ADC_PULSE_END_OF_CONV);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(myADC0_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(500);

	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	// Disables SOC burst mode.
	ADC_disableBurstMode(myADC0_BASE);
	// Sets the priority mode of the SOCs.
	ADC_setSOCPriority(myADC0_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	// Start of Conversion 0 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 1 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 2 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN4_ADCIN5
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN4_ADCIN5, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 3 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN6_ADCIN7
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN6_ADCIN7, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 4 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_ADCINT1
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_ADCINT1);
	// Start of Conversion 5 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 6 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 6
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN4_ADCIN5
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN4_ADCIN5, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 7 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 7
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN6_ADCIN7
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC0_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN6_ADCIN7, 20U);
	ADC_setInterruptSOCTrigger(myADC0_BASE, ADC_SOC_NUMBER7, ADC_INT_SOC_TRIGGER_NONE);

	//myADC1 initialization

	// ADC Initialization: Write ADC configurations and power up the ADC
	// Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(myADC1_BASE, ADC_CLK_DIV_1_0);
	// Configures the analog-to-digital converter resolution and signal mode.
	ADC_setMode(myADC1_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(myADC1_BASE, ADC_PULSE_END_OF_CONV);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(myADC1_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(500);

	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	// Disables SOC burst mode.
	ADC_disableBurstMode(myADC1_BASE);
	// Sets the priority mode of the SOCs.
	ADC_setSOCPriority(myADC1_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	// Start of Conversion 0 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC1_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC1_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 1 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC1_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC1_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 2 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC1_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC1_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 3 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC1_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC1_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);

	//myADC2 initialization

	// ADC Initialization: Write ADC configurations and power up the ADC
	// Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(myADC2_BASE, ADC_CLK_DIV_1_0);
	// Configures the analog-to-digital converter resolution and signal mode.
	ADC_setMode(myADC2_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(myADC2_BASE, ADC_PULSE_END_OF_CONV);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(myADC2_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(500);

	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	// Disables SOC burst mode.
	ADC_disableBurstMode(myADC2_BASE);
	// Sets the priority mode of the SOCs.
	ADC_setSOCPriority(myADC2_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	// Start of Conversion 0 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC2_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADC2_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 1 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC2_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC2_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 2 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC2_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(myADC2_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 3 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC2_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADC2_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 4 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC2_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC2_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 5 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC2_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(myADC2_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

	//myADC3 initialization

	// ADC Initialization: Write ADC configurations and power up the ADC
	// Configures the analog-to-digital converter module prescaler.
	ADC_setPrescaler(myADC3_BASE, ADC_CLK_DIV_1_0);
	// Configures the analog-to-digital converter resolution and signal mode.
	ADC_setMode(myADC3_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	// Sets the timing of the end-of-conversion pulse
	ADC_setInterruptPulseMode(myADC3_BASE, ADC_PULSE_END_OF_CONV);
	// Powers up the analog-to-digital converter core.
	ADC_enableConverter(myADC3_BASE);
	// Delay for 1ms to allow ADC time to power up
	DEVICE_DELAY_US(500);

	// SOC Configuration: Setup ADC EPWM channel and trigger settings
	// Disables SOC burst mode.
	ADC_disableBurstMode(myADC3_BASE);
	// Sets the priority mode of the SOCs.
	ADC_setSOCPriority(myADC3_BASE, ADC_PRI_ALL_ROUND_ROBIN);
	// Start of Conversion 0 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 0
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 1 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 1
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 2 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 2
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER2, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 3 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 3
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER3, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 4 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 4
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN4, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER4, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 5 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 5
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN14
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN14, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 6 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 6
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN15
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN15, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 7 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 7
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN0
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER7, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 8 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 8
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN1
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER8, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN1, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER8, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 9 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 9
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN2
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER9, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER9, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 10 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 10
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN3
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER10, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN3, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER10, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 11 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 11
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN4
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER11, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN4, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER11, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 12 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 12
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN14
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER12, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN14, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER12, ADC_INT_SOC_TRIGGER_NONE);
	// Start of Conversion 13 Configuration
	// Configures a start-of-conversion (SOC) in the ADC and its interrupt SOC trigger.
	// 	  	SOC number		: 13
	//	  	Trigger			: ADC_TRIGGER_EPWM6_SOCA
	//	  	Channel			: ADC_CH_ADCIN15
	//	 	Sample Window	: 20 SYSCLK cycles
	//		Interrupt Trigger: ADC_INT_SOC_TRIGGER_NONE
	ADC_setupSOC(myADC3_BASE, ADC_SOC_NUMBER13, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN15, 20U);
	ADC_setInterruptSOCTrigger(myADC3_BASE, ADC_SOC_NUMBER13, ADC_INT_SOC_TRIGGER_NONE);
	// ADC Interrupt 1 Configuration
	// 		SOC/EOC number	: 13
	// 		Interrupt Source: enabled
	// 		Continuous Mode	: disabled
	ADC_setInterruptSource(myADC3_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER13);
	ADC_enableInterrupt(myADC3_BASE, ADC_INT_NUMBER1);
	ADC_clearInterruptStatus(myADC3_BASE, ADC_INT_NUMBER1);
	ADC_disableContinuousMode(myADC3_BASE, ADC_INT_NUMBER1);

}

void CAN_init(){

	//myCAN0 initialization
	CAN_initModule(myCAN0_BASE);

	// Refer to the Driver Library User Guide for information on how to set
	// tighter timing control. Additionally, consult the device data sheet
	// for more information about the CAN module clocking.
	//
	CAN_setBitTiming(myCAN0_BASE, 15, 0, 15, 7, 3);

	//
	// Start CAN module operations
	//
	CAN_startModule(myCAN0_BASE);
}
void EMIF1_init(){
	
	//myEMIF10 initialization 
	EMIF_AsyncTimingParams tparam;
    SysCtl_setEMIF1ClockDivider(SYSCTL_EMIF1CLK_DIV_1);
    //
    // Grab EMIF1 For CPU1.
    //
    EMIF_selectMaster(EMIF1CONFIG_BASE, EMIF_MASTER_CPU1_G);

    //
    // Disable Access Protection. (CPU_FETCH/CPU_WR/DMA_WR)
    //
    EMIF_setAccessProtection(EMIF1CONFIG_BASE, 0x0);

    //
    // Commit the configuration related to protection. Till this bit remains
    // set, contents of EMIF1ACCPROT0 register can't be changed.
    //
    EMIF_commitAccessConfig(EMIF1CONFIG_BASE);

    //
    // Lock the configuration so that EMIF1COMMIT register can't be changed
    // any more.
    //
    EMIF_lockAccessConfig(EMIF1CONFIG_BASE);


    //
    // Configure Normal Asynchronous Mode of Operation.
    //
    EMIF_setAsyncMode(EMIF1_BASE, EMIF_ASYNC_CS2_OFFSET,
                      EMIF_ASYNC_NORMAL_MODE);
    EMIF_setAsyncMode(EMIF1_BASE, EMIF_ASYNC_CS3_OFFSET,
                      EMIF_ASYNC_NORMAL_MODE);

    //
    // Disable Extended Wait Mode.
    //
    EMIF_disableAsyncExtendedWait(EMIF1_BASE, EMIF_ASYNC_CS2_OFFSET);
    EMIF_disableAsyncExtendedWait(EMIF1_BASE, EMIF_ASYNC_CS3_OFFSET);

    //
    // Configure EMIF1 Data Bus Width.
    //
    EMIF_setAsyncDataBusWidth(EMIF1_BASE, EMIF_ASYNC_CS2_OFFSET,
                              EMIF_ASYNC_DATA_WIDTH_16);
    EMIF_setAsyncDataBusWidth(EMIF1_BASE, EMIF_ASYNC_CS3_OFFSET,
                              EMIF_ASYNC_DATA_WIDTH_16);

    //
    // Configure the access timing for CS2/CS3 space.
    //
    tparam.rSetup = 0;
    tparam.rStrobe = 3;
    tparam.rHold = 0;
    tparam.turnArnd = 0;
    tparam.wSetup = 0;
    tparam.wStrobe = 1;
    tparam.wHold = 0;
    EMIF_setAsyncTimingParams(EMIF1_BASE, EMIF_ASYNC_CS2_OFFSET, &tparam);
    EMIF_setAsyncTimingParams(EMIF1_BASE, EMIF_ASYNC_CS3_OFFSET, &tparam);

		
}


static void initEPWM(uint32_t base)
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, EPWM_TIMER_TBPRD);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(base,
                                EPWM_COUNTER_COMPARE_A,
                                EPWM_TIMER_TBPRD/2);
    EPWM_setCounterCompareValue(base,
                                EPWM_COUNTER_COMPARE_B,
                                EPWM_TIMER_TBPRD/2);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setClockPrescaler(base,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(base,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
}

void EPWM_init(){

    // Disable sync(Freeze clock to PWM as well). GTBCLKSYNC is applicable
    // only for multiple core devices. Uncomment the below statement if
    // applicable.
    //
    // SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Initialize PWM1 without phase shift as master
    //
    initEPWM(myEPWM0_BASE);

    //
    // Initialize PWM2
    //
    initEPWM(myEPWM1_BASE);
    EPWM_selectPeriodLoadEvent(myEPWM1_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);


    //
    // Initialize PWM3
    //
    initEPWM(myEPWM2_BASE);
    EPWM_selectPeriodLoadEvent(myEPWM3_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    //
    // Initialize PWM4
    //
    initEPWM(myEPWM3_BASE);
    EPWM_selectPeriodLoadEvent(myEPWM4_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    //
    // Initialize PWM5
    //
    initEPWM(myEPWM4_BASE);
    EPWM_selectPeriodLoadEvent(myEPWM4_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    //
    // Initialize PWM6
    //
    initEPWM(myEPWM5_BASE);
    EPWM_selectPeriodLoadEvent(myEPWM5_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    //
    // Disable SOCA
    //
    EPWM_disableADCTrigger(EPWM6_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the first up-count event
    //
    EPWM_setADCTriggerSource(EPWM6_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM6_BASE, EPWM_SOC_A, 1);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);


}
void GPIO_init(){
		
	//myGPIO0 initialization
	GPIO_setDirectionMode(myGPIO0, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO0, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO0, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO0, GPIO_QUAL_SYNC);
		
	//myGPIO1 initialization
	GPIO_setDirectionMode(myGPIO1, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO1, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO1, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO1, GPIO_QUAL_SYNC);
		
	//myGPIO2 initialization
	GPIO_setDirectionMode(myGPIO2, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO2, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO2, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO2, GPIO_QUAL_SYNC);
		
	//myGPIO3 initialization
	GPIO_setDirectionMode(myGPIO3, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO3, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO3, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO3, GPIO_QUAL_SYNC);
		
	//myGPIO4 initialization
	GPIO_setDirectionMode(myGPIO4, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO4, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO4, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO4, GPIO_QUAL_SYNC);
		
	//myGPIO5 initialization
	GPIO_setDirectionMode(myGPIO5, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO5, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO5, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO5, GPIO_QUAL_SYNC);
		
	//myGPIO6 initialization
	GPIO_setDirectionMode(myGPIO6, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO6, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO6, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO6, GPIO_QUAL_SYNC);
		
	//myGPIO7 initialization
	GPIO_setDirectionMode(myGPIO7, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO7, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO7, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO7, GPIO_QUAL_SYNC);
		
	//myGPIO8 initialization
	GPIO_setDirectionMode(myGPIO8, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO8, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO8, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO8, GPIO_QUAL_SYNC);
		
	//myGPIO9 initialization
	GPIO_setDirectionMode(myGPIO9, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO9, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO9, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO9, GPIO_QUAL_SYNC);
		
	//myGPIO10 initialization
	GPIO_setDirectionMode(myGPIO10, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO10, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO10, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO10, GPIO_QUAL_SYNC);
		
	//myGPIO11 initialization
	GPIO_setDirectionMode(myGPIO11, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO11, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO11, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO11, GPIO_QUAL_SYNC);
		
	//myGPIO12 initialization
	GPIO_setDirectionMode(myGPIO12, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO12, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO12, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO12, GPIO_QUAL_SYNC);
		
	//myGPIO13 initialization
	GPIO_setDirectionMode(myGPIO13, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO13, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO13, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO13, GPIO_QUAL_SYNC);
		
	//myGPIO14 initialization
	GPIO_setDirectionMode(myGPIO14, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO14, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO14, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO14, GPIO_QUAL_SYNC);
		
	//myGPIO15 initialization
	GPIO_setDirectionMode(myGPIO15, GPIO_DIR_MODE_IN);
	GPIO_setPadConfig(myGPIO15, GPIO_PIN_TYPE_STD);
	GPIO_setMasterCore(myGPIO15, GPIO_CORE_CPU1);
	GPIO_setQualificationMode(myGPIO15, GPIO_QUAL_SYNC);
}
void I2C_init(){
	//myI2C0 initialization
	
	I2C_disableModule(myI2C0_BASE);
	I2C_initMaster(myI2C0_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_33);
	I2C_setConfig(myI2C0_BASE, I2C_MASTER_SEND_MODE);
	I2C_setSlaveAddress(myI2C0_BASE, 0);
	I2C_disableLoopback(myI2C0_BASE);
	I2C_setBitCount(myI2C0_BASE, I2C_BITCOUNT_1);
	I2C_setDataCount(myI2C0_BASE, 1);
	I2C_setAddressMode(myI2C0_BASE, I2C_ADDR_MODE_7BITS);
	I2C_enableFIFO(myI2C0_BASE);
	I2C_setEmulationMode(myI2C0_BASE, I2C_EMULATION_STOP_SCL_LOW);
	I2C_enableModule(myI2C0_BASE);

}
void SCI_init(){
	
	//mySCI0 initialization
	SCI_clearInterruptStatus(mySCI0_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(mySCI0_BASE);

	SCI_resetTxFIFO(mySCI0_BASE);
	SCI_resetRxFIFO(mySCI0_BASE);
	SCI_resetChannels(mySCI0_BASE);

	SCI_setConfig(mySCI0_BASE, DEVICE_LSPCLK_FREQ, mySCI0_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(mySCI0_BASE);
	SCI_performSoftwareReset(mySCI0_BASE);
	SCI_setFIFOInterruptLevel(mySCI0_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
	SCI_enableFIFO(mySCI0_BASE);
	SCI_enableModule(mySCI0_BASE);
	
	//mySCI1 initialization
	SCI_clearInterruptStatus(mySCI1_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(mySCI1_BASE);

	SCI_resetTxFIFO(mySCI1_BASE);
	SCI_resetRxFIFO(mySCI1_BASE);
	SCI_resetChannels(mySCI1_BASE);

	SCI_setConfig(mySCI1_BASE, DEVICE_LSPCLK_FREQ, mySCI1_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(mySCI1_BASE);
	SCI_performSoftwareReset(mySCI1_BASE);
	SCI_setFIFOInterruptLevel(mySCI1_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
	SCI_enableFIFO(mySCI1_BASE);
	SCI_enableModule(mySCI1_BASE);
	
	//mySCI2 initialization
	SCI_clearInterruptStatus(mySCI2_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
	SCI_clearOverflowStatus(mySCI2_BASE);

	SCI_resetTxFIFO(mySCI2_BASE);
	SCI_resetRxFIFO(mySCI2_BASE);
	SCI_resetChannels(mySCI2_BASE);

	SCI_setConfig(mySCI2_BASE, DEVICE_LSPCLK_FREQ, mySCI2_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
	SCI_disableLoopback(mySCI2_BASE);
	SCI_performSoftwareReset(mySCI2_BASE);
	SCI_setFIFOInterruptLevel(mySCI2_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
	SCI_enableFIFO(mySCI2_BASE);
	SCI_enableModule(mySCI2_BASE);
}
void SPI_init()
{
	
	//mySPI0 initialization
	SPI_disableModule(mySPI0_BASE);
	SPI_setConfig(mySPI0_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
				  SPI_MODE_MASTER, 25000, 	16);
	SPI_disableFIFO(mySPI0_BASE);
	SPI_disableLoopback(mySPI0_BASE);
	SPI_setEmulationMode(mySPI0_BASE, SPI_EMULATION_STOP_MIDWAY);
	SPI_enableModule(mySPI0_BASE);
	
	//mySPI1 initialization
	SPI_disableModule(mySPI1_BASE);
	SPI_setConfig(mySPI1_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
				  SPI_MODE_MASTER, 25000, 	16);
	SPI_disableFIFO(mySPI1_BASE);
	SPI_disableLoopback(mySPI1_BASE);
	SPI_setEmulationMode(mySPI1_BASE, SPI_EMULATION_STOP_MIDWAY);
	SPI_enableModule(mySPI1_BASE);
}
void USB_init(){
	
	//myUSB0 initialization 
}

