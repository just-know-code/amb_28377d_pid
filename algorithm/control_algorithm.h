/*
 * control_algorithm.h
 *
 *  Created on: 2022Äê2ÔÂ28ÈÕ
 *      Author: 17699
 */

#ifndef DEVICE_CONTROL_ALGORITHM_H_
#define DEVICE_CONTROL_ALGORITHM_H_

#include "stdint.h"

extern struct pid_t pid_tArray[5];
extern float posIntegralArray[5];
extern float rotorPosition[5];
extern float forwardFirstPos[5];
extern float refPosition[5];
extern float propotion[5];
extern float differential[5];

extern float coilCurrent[10];
extern float coilBiasCurrent[5];
extern float refCurrent[10];
extern float currIntegralArray[10];
extern struct pi_t currentLoopPI;

extern uint32_t rawPosData[5];
extern uint32_t rawCurrData[10];
extern uint16_t pwmDuty[10];

extern uint16_t sampling_times;
extern uint16_t loop_sel;
extern uint16_t pos_pid_sel;
extern void CalculPID(uint16_t index);
extern void CalculPI(uint16_t index);
extern void Variable_init();
extern void AutoMeasurCenterPos();
#define EPWM_TIMER_TBPRD 5000U
#endif /* DEVICE_CONTROL_ALGORITHM_H_ */
