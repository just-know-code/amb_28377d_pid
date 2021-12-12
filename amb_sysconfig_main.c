//#############################################################################
//
// FILE:   empty_sysconfig_main.c
//
// TITLE:  Empty Pinmux Example
//
// Empty SysCfg & Driverlib Example
//
// This example is an empty project setup for SysConfig and Driverlib 
// development.
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
#include "driverlib.h"
#include "device.h"
#include "board.h"

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals.
    //
    Device_init();

	//
	// Setup GPIO by disabling pin locks and enabling pullups.
	//
	Device_initGPIO();

	//
	// Initialize PIE and clear PIE registers. Disables CPU interrupts.
	//
	Interrupt_initModule();

	//
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	//
	Interrupt_initVectorTable();

	Board_init();

    Interrupt_register(INT_ADCD1, &adcD1ISR);

    //
    // Enable ADC interrupt
    //
    Interrupt_enable(INT_ADCD1);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

}


//
// ADC D Interrupt1 ISR
//
__interrupt void adcD1ISR(void)
{
    //
    // Get rotor position
    //
	uint16_t index = 0;
	for(; index < 3; index++)
		rotorPosition[index] = (ADC_readResult(ADCARESULT_BASE, index) +
								ADC_readResult(ADCARESULT_BASE, index+3))/2;
	for(; index < 5; index++)
		rotorPosition[index] = (ADC_readResult(ADCBRESULT_BASE, index) +
							  ADC_readResult(ADCBRESULT_BASE, index+2))/2;
    //
    // Get coil current
    //
	index = 0;
	for(; index < 3; index++)
		coilCurrent[index] = (ADC_readResult(ADCCRESULT_BASE, index) +
								ADC_readResult(ADCCRESULT_BASE, index+3))/2;
	for(; index < 10; index++)
		coilCurrent[index] = (ADC_readResult(ADCDRESULT_BASE, index) +
							  ADC_readResult(ADCDRESULT_BASE, index+7))/2;
    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCD_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCD_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCD_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


//
// End of File
//
