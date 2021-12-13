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

#ifndef BOARD_H
#define BOARD_H

#define EPWM_TIMER_TBPRD 5000
//
// Included Files
//

#include "driverlib.h"
#include "device.h"

#define GPIO_PIN_CANRXA 62
#define GPIO_PIN_CANTXA 63
#define GPIO_PIN_EM1CS2N 34
#define GPIO_PIN_EM1CS3N 35
#define GPIO_PIN_EM1SDCKE 29
#define GPIO_PIN_EM1CLK 30
#define GPIO_PIN_EM1WEN 31
#define GPIO_PIN_EM1WAIT 36
#define GPIO_PIN_EM1OEN 37
#define GPIO_PIN_EM1BA0 93
#define GPIO_PIN_EM1BA1 92
#define GPIO_PIN_EM1A0 38
#define GPIO_PIN_EM1A1 39
#define GPIO_PIN_EM1A2 40
#define GPIO_PIN_EM1A3 41
#define GPIO_PIN_EM1A4 44
#define GPIO_PIN_EM1A5 45
#define GPIO_PIN_EM1A6 46
#define GPIO_PIN_EM1A7 47
#define GPIO_PIN_EM1A8 48
#define GPIO_PIN_EM1A9 49
#define GPIO_PIN_EM1A10 50
#define GPIO_PIN_EM1A11 51
#define GPIO_PIN_EM1A12 52
#define GPIO_PIN_EM1A13 86
#define GPIO_PIN_EM1A14 87
#define GPIO_PIN_EM1A15 88
#define GPIO_PIN_EM1A16 89
#define GPIO_PIN_EM1A17 90
#define GPIO_PIN_EM1A18 91
#define GPIO_PIN_EM1D0 85
#define GPIO_PIN_EM1D1 83
#define GPIO_PIN_EM1D2 82
#define GPIO_PIN_EM1D3 81
#define GPIO_PIN_EM1D4 80
#define GPIO_PIN_EM1D5 79
#define GPIO_PIN_EM1D6 78
#define GPIO_PIN_EM1D7 77
#define GPIO_PIN_EM1D8 76
#define GPIO_PIN_EM1D9 75
#define GPIO_PIN_EM1D10 74
#define GPIO_PIN_EM1D11 73
#define GPIO_PIN_EM1D12 72
#define GPIO_PIN_EM1D13 71
#define GPIO_PIN_EM1D14 70
#define GPIO_PIN_EM1D15 69
#define GPIO_PIN_EPWM1A 0
#define GPIO_PIN_EPWM1B 1
#define GPIO_PIN_EPWM2A 2
#define GPIO_PIN_EPWM2B 3
#define GPIO_PIN_EPWM3A 4
#define GPIO_PIN_EPWM3B 5
#define GPIO_PIN_EPWM4A 6
#define GPIO_PIN_EPWM4B 7
#define GPIO_PIN_EPWM5A 8
#define GPIO_PIN_EPWM5B 9
#define GPIO_PIN_EPWM6A 10
#define GPIO_PIN_EPWM6B 11
#define GPIO_PIN_EPWM7A 12
#define GPIO_PIN_EPWM7B 13
#define GPIO_PIN_EPWM8A 14
#define GPIO_PIN_EPWM8B 15
#define GPIO_PIN_SDAA 32
#define GPIO_PIN_SCLA 33
#define GPIO_PIN_SCIRXDA 64
#define GPIO_PIN_SCITXDA 65
#define GPIO_PIN_SCIRXDB 55
#define GPIO_PIN_SCITXDB 54
#define GPIO_PIN_SCIRXDC 57
#define GPIO_PIN_SCITXDC 56
#define GPIO_PIN_SPISIMOA 16
#define GPIO_PIN_SPISOMIA 17
#define GPIO_PIN_SPICLKA 18
#define GPIO_PIN_SPISTEA 19
#define GPIO_PIN_SPISIMOB 60
#define GPIO_PIN_SPISOMIB 61
#define GPIO_PIN_SPICLKB 58
#define GPIO_PIN_SPISTEB 59

#define myADC0_BASE ADCA_BASE
#define myADC0_RESULT_BASE ADCARESULT_BASE
#define myADC1_BASE ADCB_BASE
#define myADC1_RESULT_BASE ADCBRESULT_BASE
#define myADC2_BASE ADCC_BASE
#define myADC2_RESULT_BASE ADCCRESULT_BASE
#define myADC3_BASE ADCD_BASE
#define myADC3_RESULT_BASE ADCDRESULT_BASE

#define myCAN0_BASE CANA_BASE



#define myEMIF10_BASE EMIF1_BASE

#define myEPWM0_BASE EPWM1_BASE
#define myEPWM1_BASE EPWM2_BASE
#define myEPWM2_BASE EPWM3_BASE
#define myEPWM3_BASE EPWM4_BASE
#define myEPWM4_BASE EPWM5_BASE
#define myEPWM5_BASE EPWM6_BASE
#define myEPWM6_BASE EPWM7_BASE
#define myEPWM7_BASE EPWM8_BASE

#define myGPIO0 53
#define myGPIO1 66
#define myGPIO2 67
#define myGPIO3 68
#define myGPIO4 84
#define myGPIO5 94
#define myGPIO6 99
#define myGPIO7 133
#define myGPIO8 20
#define myGPIO9 21
#define myGPIO10 22
#define myGPIO11 23
#define myGPIO12 24
#define myGPIO13 25
#define myGPIO14 26
#define myGPIO15 27

#define myI2C0_BASE I2CA_BASE
#define myI2C0_BITRATE 400000
#define myI2C0_SLAVE_ADDRESS 0
#define myI2C0_OWN_SLAVE_ADDRESS 0

#define mySCI0_BASE SCIA_BASE
#define mySCI0_BAUDRATE 115200
#define mySCI0_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define mySCI0_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define mySCI0_CONFIG_PAR SCI_CONFIG_PAR_NONE
#define mySCI1_BASE SCIB_BASE
#define mySCI1_BAUDRATE 115200
#define mySCI1_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define mySCI1_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define mySCI1_CONFIG_PAR SCI_CONFIG_PAR_NONE
#define mySCI2_BASE SCIC_BASE
#define mySCI2_BAUDRATE 115200
#define mySCI2_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define mySCI2_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define mySCI2_CONFIG_PAR SCI_CONFIG_PAR_NONE

#define mySPI0_BASE SPIA_BASE
#define mySPI0_BITRATE 25000
#define mySPI1_BASE SPIB_BASE
#define mySPI1_BITRATE 25000

#define myUSB0_BASE USB0_BASE

void	Board_init();
void	ADC_init();
void	CAN_init();
void	EMIF1_init();
void	EPWM_init();
void	GPIO_init();
void	I2C_init();
void	SCI_init();
void	SPI_init();
void	USB_init();
void	PinMux_init();

#endif  // end of BOARD_H definition
