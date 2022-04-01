#include "control_algorithm.h"

struct pi_t {
	float P;
	float I;
};

float coilCurrent[10];

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
uint16_t cur_pid_sel;
/*
 * 0b0000000000   NO PI RUN
 * 0b0000000001	  the first PI run
 * 0b0000000010	  the second PI run
 * -----------
 * 0b1111111111	  all PI run
 */
int16_t rotorPosition[5];
float f_v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float f_pv[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float f_iv[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float f_dv[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int16_t PID_OUT[5] = { 0.0, 0.0, 0.0, 0.0, 0.0};
float current_bias[10] = {550.0f, 550.0f, 550.0f, 550.0f, 550.0f, 550.0f, 550.0f, 550.0f, 550.0f, 550.0f};

struct PID                         /* 定义PID结构体函数 */
{
	int16_t      SetPoint;
	//	 float    Proportion0;                 /* 变参数的PID */
	//	 float    Derivative0;
	float    Proportion;
	float    Integral;
	float    Derivative;
	int16_t      LastV;
};

struct PID  s_PID[5];

#define maxi  500.0f     //积分饱和
#define mini -500.0f       //积分饱和
#define maxv  1000.0f       //电压输出上限   PID饱和
#define minv -1000.0f       //电压输出下限   PID饱和
#define DT 0.00005f

void PIDCalc(int16_t channel, int16_t NextPoint) {

	float i_dError, i_Error;
	i_Error = NextPoint - s_PID[channel].SetPoint; /* 平衡位置与设定点的差值 */
	i_dError = NextPoint - s_PID[channel].LastV; /* 相邻两点之间的差值 */
	s_PID[channel].LastV = NextPoint;

	f_pv[channel] = s_PID[channel].Proportion * i_Error;
	f_iv[channel] += s_PID[channel].Integral * i_Error * DT;
	f_dv[channel] = s_PID[channel].Derivative * i_dError / DT;

	if (f_iv[channel] > maxi)
		f_iv[channel] = maxi;
	if (f_iv[channel] < mini)
		f_iv[channel] = mini;

//	if (f_dv[channel] > 300)
//		f_dv[channel] = 300;
//	if (f_dv[channel] < -300)
//		f_dv[channel] = -300;

	f_v[channel] = f_pv[channel] + f_iv[channel] + f_dv[channel];
	if (f_v[channel] > maxv)
		f_v[channel] = maxv;
	if (f_v[channel] < minv)
		f_v[channel] = minv;

	int pid_out = (f_v[channel]*current_bias[channel]/900.0F);

	refCurrent[channel * 2] = (float)(current_bias[channel] + pid_out) / 235.5f;
	refCurrent[channel * 2] = (float)(current_bias[channel] - pid_out) / 235.5f;
}
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


#define MAX_PWM_DUTY 0.45f
#define MIN_PWM_DUTY -0.45f
#define MAX_CURR_INTEGRAL 1.0f
#define MIN_CURR_INTEGRAL -1.0f
void CalculPI(uint16_t index) {

	float propotion, integral, error, outcome;
	error = refCurrent[index] - coilCurrent[index];
	currIntegralArray[index] += error * DT;
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

	for (i = 0; i < 10; i++) {
		currIntegralArray[i] = 0;
		pwmDuty[i] = 2500;
		refCurrent[i] = 2.0f;

	}

	currentLoopPI.P = 0.32;
	currentLoopPI.I = 0.1;



	sampling_times = 0;
	loop_sel = 0;
	pos_pid_sel = 0;
	cur_pid_sel = 0;
}

#define MeasureCurrent 4.0f
void AutoMeasurCenterPos(){
	static uint32_t count = 0;
	uint16_t index;
	if (count < 30000){				//Magnetic attraction upward
		if (count == 1){
			for (index = 0; index < 10; index++){
				refCurrent[index] = 0.0f;
			}
			refCurrent[2] = MeasureCurrent;
			refCurrent[3] = 0.0f;
			refCurrent[6] = MeasureCurrent;
			refCurrent[7] = 0.0f;
			for (index = 0; index < 5; index++){

			}
		}
		if (count >= 29995){

		}
	} else if (count < 60000){		//Magnetic attraction downward
		if (count == 30000){
			for (index = 0; index < 10; index++){
				refCurrent[index] = 0.0f;
			}
			refCurrent[2] = 0.0f;
			refCurrent[3] = MeasureCurrent;
			refCurrent[6] = 0.0f;
			refCurrent[7] = MeasureCurrent;
		}
		if (count >= 59995){

		}
	} else if (count < 90000){		//Magnetic attraction to the left
		if (count == 60000){
			for (index = 0; index < 10; index++){
				refCurrent[index] = 0.0f;
			}
			refCurrent[4] = MeasureCurrent;
			refCurrent[5] = 0.0f;
			refCurrent[8] = MeasureCurrent;
			refCurrent[9] = 0.0f;
		}
		if (count >= 89995){

		}
	} else if (count < 120000){		//Magnetic attraction to the right
		if (count == 90000){
			for (index = 0; index < 10; index++){
				refCurrent[index] = 0.0f;
			}
			refCurrent[4] = 0.0f;
			refCurrent[5] = MeasureCurrent;
			refCurrent[8] = 0.0f;
			refCurrent[9] = MeasureCurrent;
		}
		if (count >= 119995){

		}
	} else {
		for (index = 0; index < 5; index++){

		}
		for (index = 0; index < 10; index++){
			pwmDuty[index] = 2500;
		}
		loop_sel = 0;
		count = 0;
	}
	count++;
}
