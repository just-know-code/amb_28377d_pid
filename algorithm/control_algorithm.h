/*
 * control_algorithm.h
 *
 *  Created on: 2022��2��28��
 *      Author: 17699
 */

#ifndef DEVICE_CONTROL_ALGORITHM_H_
#define DEVICE_CONTROL_ALGORITHM_H_

#include "stdint.h"

extern int16_t rotorPosition[5];

extern float coilCurrent[10];

extern float refCurrent[10];
extern float currIntegralArray[10];
extern struct pi_t currentLoopPI;

extern uint16_t shift_phase;
extern uint32_t rawPosData[5];
extern uint32_t rawCurrData[10];
extern uint16_t pwmDuty[10];
extern struct PID s_PID[5];
extern uint16_t sampling_times;
extern uint16_t loop_sel;
extern uint16_t pos_pid_sel;
extern uint16_t cur_pid_sel;
void PIDCalc(int16_t channel, int16_t NextPoint);
extern void CalculPI(uint16_t index);
extern void CalculPI_I1();

extern void Variable_init();
extern void AutoMeasurCenterPos();
#define EPWM_TIMER_TBPRD 2500U
#endif /* DEVICE_CONTROL_ALGORITHM_H_ */
