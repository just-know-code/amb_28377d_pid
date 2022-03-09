#include "control_algorithm.h"

struct pid_t {
	float P;
	float I;
	float D;
};

struct pi_t {
	float P;
	float I;
};

struct pid_t pid_tArray[5];
float posIntegralArray[5];
float rotorPosition[5];
float forwardFirstPos[5];
float refPosition[5];
float propotion[5];
float differential[5];

float coilCurrent[10];
float coilBiasCurrent[5];
float refCurrent[10];
float currIntegralArray[10];
struct pi_t currentLoopPI;

uint32_t rawPosData[5];
uint32_t rawCurrData[10];
uint16_t pwmDuty[10];

uint16_t sampling_times;

uint16_t loop_sel;
/*
 * 0b0000 two loops are open loop, 0b11 two loops are closed loop
 * 0b01 only current is closed loop, 0b10 only position loop is closed loop
 */

uint16_t pos_pid_sel;
/*
 * 0b00000 all PID operations do not work
 * 0b00001 only the first PID works
 * 0b00110 front radial directions PID work, 6
 * 0b11000 trailing radial directions PID work, 24
 * 0b11110 all radial directions PID work, 30(2 4 8 16)
 */

#define CONTROL_PERIOD 0.00005f
#define MAX_POS_INTEGRAL 4.0f
#define MIN_POS_INTEGRAL -4.0f
#define MAX_CONTROL_CURRENT 6.0f
#define MIN_CONTROL_CURRENT -6.0f

void CalculPID(uint16_t index) {

//	float propotion, differential;
	float pos_loop_outcome, error, firstOrderDiff;
	error = refPosition[index] - rotorPosition[index]; /* 平衡位置与设定点的差值 */
	firstOrderDiff = rotorPosition[index] - forwardFirstPos[index]; /* 相邻两点之间的差值 */
	forwardFirstPos[index] = rotorPosition[index];

	propotion[index] = pid_tArray[index].P * error;
	differential[index] = pid_tArray[index].D * firstOrderDiff / CONTROL_PERIOD;

	posIntegralArray[index] += error * CONTROL_PERIOD;
	if (posIntegralArray[index] > MAX_POS_INTEGRAL)
		posIntegralArray[index] = MAX_POS_INTEGRAL;
	if (posIntegralArray[index] < MIN_POS_INTEGRAL)
		posIntegralArray[index] = MIN_POS_INTEGRAL;

	pos_loop_outcome = propotion[index]
					   + pid_tArray[index].I * posIntegralArray[index]
					   + differential[index];

	if (pos_loop_outcome > MAX_CONTROL_CURRENT)
		pos_loop_outcome = MAX_CONTROL_CURRENT;
	if (pos_loop_outcome < MIN_CONTROL_CURRENT)
		pos_loop_outcome = MIN_CONTROL_CURRENT;

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

#define MAX_PWM_DUTY 0.45f
#define MIN_PWM_DUTY -0.45f
#define MAX_CURR_INTEGRAL 1.0f
#define MIN_CURR_INTEGRAL -1.0f
void CalculPI(uint16_t index) {

	float propotion, integral, error, outcome;
	error = refCurrent[index] - coilCurrent[index];
	currIntegralArray[index] += error * CONTROL_PERIOD;
	propotion = currentLoopPI.P * error;
	if (currIntegralArray[index] > MAX_CURR_INTEGRAL)
		currIntegralArray[index] = MAX_CURR_INTEGRAL;
	if (currIntegralArray[index] < MIN_CURR_INTEGRAL)
		currIntegralArray[index] = MIN_CURR_INTEGRAL;

	integral = currentLoopPI.I * currIntegralArray[index];
	outcome = propotion + integral;
	if (outcome > MAX_PWM_DUTY)
		outcome = MAX_PWM_DUTY;
	if (outcome < MIN_PWM_DUTY)
		outcome = MIN_PWM_DUTY;
	outcome += 0.5;
	pwmDuty[index] = (uint16_t) (outcome * EPWM_TIMER_TBPRD);
}


//*****************************************************************************
//
// initialize all variable and array
//
//*****************************************************************************
void Variable_init() {

	uint16_t i;
	for (i = 0; i < 5; i++) {
		posIntegralArray[i] = 0;
		forwardFirstPos[i] = 0;
		refPosition[i] = 250;
	}
	for (i = 0; i < 10; i++) {
		currIntegralArray[i] = 0;
		pwmDuty[i] = 2500;
		refCurrent[i] = 2.0f;

	}

	currentLoopPI.P = 0.32;
	currentLoopPI.I = 0.1;

	coilBiasCurrent[0] = 2.0f;
	coilBiasCurrent[1] = 2.0f;
	coilBiasCurrent[2] = 2.0f;
	coilBiasCurrent[3] = 2.0f;
	coilBiasCurrent[4] = 2.0f;

	refPosition[0] = 100;
	refPosition[1] = 100;
	refPosition[2] = 200;
	refPosition[3] = 200;
	refPosition[4] = 300;

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
	loop_sel = 0;
	pos_pid_sel = 0;
}

#define MeasureCurrent 4.0f
void AutoMeasurCenterPos(){
	static uint16_t count = 0;
	uint16_t index;
	if (count < 30000){
		if (count == 0){
			for (index = 0; index < 5; index++){
				refCurrent[index * 2] = MeasureCurrent;
				refCurrent[index * 2 + 1] = 0.0f;
			}
			for (index = 0; index < 5; index++){
				refPosition[index] = 0;
			}
		}
		if (count >= 29995){
			for (index = 0; index < 5; index++){
				refPosition[index] += rotorPosition[index];
			}
		}
	} else if (count < 60000){
		if (count == 30000){
			for (index = 0; index < 5; index++){
				refCurrent[index * 2] = 0.0f;
				refCurrent[index * 2 + 1] = MeasureCurrent;
			}
		}
		if (count >= 59995){
			for (index = 0; index < 5; index++){
				refPosition[index] += rotorPosition[index];
			}
		}
	} else {
		for (index = 0; index < 5; index++){
			refPosition[index] /= 10;
		}
		for (index = 0; index < 5; index++){
			refCurrent[index * 2] = 0.0f;
			refCurrent[index * 2 + 1] = 0.0f;
		}
		for (index = 0; index < 10; index++){
			refCurrent[index] = 0.0f;
			pwmDuty[index] = 2500;
		}
		loop_sel = 0;
		count = 0;
	}
	count++;
}
