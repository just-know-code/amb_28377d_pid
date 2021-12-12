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


__interrupt void adcD1ISR(void);
void ComputeUpdateDuty();
void CalculPID(uint16_t index);
void CalculPD(uint16_t index);
void UpdatePWMDuty();
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

	currentLoopPI.P = 10;
	currentLoopPI.I = 100;


	for (;;){

	}

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

void ComputeUpdateDuty(){

	//compute displacement loop pid
	uint16_t i;
	for(i = 0; i < 5; i++){

		CalculPID(i);
	}
	//compute current loop pd
	for(i = 0; i < 10; i++){

			CalculPD(i);
		}
	// update pwm duty
	UpdatePWMDuty();
}

#define CONTROL_PERIOD 0.00005
#define MAX_POS_INTEGRAL 500
#define MIN_POS_INTEGRAL -500
#define MAX_CONTROL_CURRENT 1000
#define MIN_CONTROL_CURRENT -1000

void CalculPID(uint16_t index){

	double error, firstOrderDiff, propotion, differential;
	uint16_t outcome;
	error = refPosition[index] - rotorPosition[index]; /* ƽ��λ�����趨��Ĳ�ֵ */
	firstOrderDiff = rotorPosition[index] - forwardFirstPos[index]; /* ��������֮��Ĳ�ֵ */
	forwardFirstPos[index] = rotorPosition[index];

	propotion = pid_tArray[index].P * error;
	posIntegralArray[index] += pid_tArray[index].I * firstOrderDiff * CONTROL_PERIOD;
	differential = pid_tArray[index].D * firstOrderDiff / CONTROL_PERIOD;

	if (posIntegralArray[index] > MAX_POS_INTEGRAL)
		posIntegralArray[index] = MAX_POS_INTEGRAL;
	if (posIntegralArray[index] < MIN_POS_INTEGRAL)
		posIntegralArray[index] = MIN_POS_INTEGRAL;

	outcome = propotion + posIntegralArray[index] + differential;
	if (outcome > MAX_CONTROL_CURRENT)
		outcome = MAX_CONTROL_CURRENT;
	if (outcome < MIN_CONTROL_CURRENT)
		outcome = MIN_CONTROL_CURRENT;
	refCurrent[index * 2] = coilBiasCurrent[index] - outcome;
	refCurrent[index * 2 + 1] = coilBiasCurrent[index] + outcome;
	/*				       ����������
	 * 					0 ����Ȧ
	 *
	 * 	����������2 ����Ȧ			3����Ȧ
	 *
	 * 					1 ����Ȧ
	 *
	 *
	 *				        ����������
	 * 					4 ����Ȧ
	 *
	 * 	����������6 ����Ȧ			7����Ȧ
	 *
	 * 					5 ����Ȧ
	 *
	 *
	 *
	 * */
}


#define MAX_PWM_DUTY 1000
#define MIN_PWM_DUTY 1000
#define MAX_CURR_INTEGRAL 500
#define MIN_CURR_INTEGRAL -500
void CalculPD(uint16_t index){

	double error, firstOrderDiff, propotion;
	uint16_t outcome;
	error = refCurrent[index] - coilCurrent[index]; /* ƽ��λ�����趨��Ĳ�ֵ */
	posIntegralArray[index] += currentLoopPI.I * firstOrderDiff * CONTROL_PERIOD;
	currIntegralArray[index] = coilCurrent[index];
	propotion = currentLoopPI.P * error;

	if (posIntegralArray[index] > MAX_CURR_INTEGRAL)
		posIntegralArray[index] = MAX_CURR_INTEGRAL;
	if (posIntegralArray[index] < MIN_CURR_INTEGRAL)
		posIntegralArray[index] = MIN_CURR_INTEGRAL;

	outcome = propotion + posIntegralArray[index];
	if (outcome > MAX_PWM_DUTY)
		outcome = MAX_PWM_DUTY;
	if (outcome < MIN_PWM_DUTY)
		outcome = MIN_PWM_DUTY;
	pwmDuty[index] = outcome;
}


void UpdatePWMDuty(){

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

//
// End of File
//