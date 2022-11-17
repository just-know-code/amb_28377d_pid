#include "control_algorithm.h"

struct pi_t {
	float P;
	float I;
};

#pragma DATA_SECTION(point, "mydata_sec")
int point[2000];

float coilCurrent[10];
float refCurrent[10];
float currIntegral[10];
float currIntegralArray[10];
struct pi_t currentLoopPI;

uint32_t rawPosData[5];
uint32_t rawCurrData[10];
uint16_t pwmDuty[10];

uint16_t sampling_times;
uint16_t shift_phase;

/*
 * 0b0000 two loops are open loop, 0b11 two loops are closed loop
 * 0b01 only current is closed loop, 0b10 only position loop is closed loop
 */
uint16_t loop_sel;

/*
 * 0b00000 all PID operations do not work
 * 0b00001 only the first PID works
 * 0b00110 front radial directions PID work, 6
 * 0b11000 trailing radial directions PID work, 24
 * 0b11110 all radial directions PID work, 30(2 4 8 16)
 */
uint16_t pos_pid_sel;


/*
 * 0b0000000000   NO PI RUN
 * 0b0000000001	  the first PI run
 * 0b0000000010	  the second PI run
 * -----------
 * 0b1111111111	  all PI run(1020)
 * 0b0000111100   60
 * 0b1111000000   960
 */
uint16_t cur_pid_sel;
uint16_t epwm_tbprd;
const float u_dc = 50.0;
int16_t rotorPosition[5];
float f_v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float f_pv[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float f_iv[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float f_dv[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int16_t PID_OUT[5] = { 0.0, 0.0, 0.0, 0.0, 0.0};

#define BIAS_CURRENT 525.0f
#define BIAS_CURRENT_FREE_END 525.0f

float current_bias[10] = {BIAS_CURRENT, BIAS_CURRENT, BIAS_CURRENT, BIAS_CURRENT, BIAS_CURRENT, BIAS_CURRENT, \
					BIAS_CURRENT_FREE_END, BIAS_CURRENT_FREE_END, BIAS_CURRENT_FREE_END, BIAS_CURRENT_FREE_END};

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

#define maxi  175.0f     //积分饱和
#define mini -175.0f       //积分饱和

#define DT 0.00005f
float tick_pre = 0.000000005 * 5000;
float tick = 0.000000005 * 5000;
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
	if (f_v[channel] > current_bias[channel])
		f_v[channel] = current_bias[channel];
	if (f_v[channel] < -current_bias[channel])
		f_v[channel] = -current_bias[channel];

//	int pid_out = (f_v[channel]*current_bias[channel]/900.0f);
	int pid_out = f_v[channel];
	refCurrent[channel * 2] = (float)(current_bias[channel] + pid_out) / 235.5f;//9 路 后上 sensor
	refCurrent[channel * 2 + 1] = (float)(current_bias[channel] - pid_out) / 235.5f;//10  路 后下
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
	pwmDuty[index] = (uint16_t) (outcome * epwm_tbprd);
}

static float CalculPI_(uint16_t index) {

	float propotion, integral, error, outcome;
	error = refCurrent[index] - coilCurrent[index];
	currIntegral[index] += error * tick;
	propotion = currentLoopPI.P * error;
	if (currIntegral[0] > u_dc)
		currIntegral[0] = u_dc;
	if (currIntegral[0] < -u_dc)
		currIntegral[0] = -u_dc;
	integral = currentLoopPI.I * currIntegral[index];
	outcome = propotion + integral;
	if (outcome > u_dc)
		outcome = u_dc;
	if (outcome < -u_dc)
		outcome = -u_dc;
	return outcome;
}


#define sqrt_2 1.4142


#define calculations_and_checks() do {	\
	t0 = ts - t1 - t2; 			\
	ta = t0 / 4;				\
	tb = ta + t1 / 2;			\
	tc = ta + t1 / 2  + t2 / 2; \
	if (ta >= max_duty) {		\
		ta = max_duty;			\
	} else if (ta < min_duty) {	\
		ta = min_duty;			\
	}							\
	if (tb >= max_duty) {		\
		tb = max_duty;			\
	} else if (tb < min_duty) {	\
		tb = min_duty;			\
	}							\
	if (tc >= max_duty) {		\
		tc = max_duty;			\
	} else if (tc < min_duty) {	\
		tc = min_duty;			\
	}							\
} while(0)


static void modulate(float ux, float uy) {

	float ts = epwm_tbprd * 2u;
	uint16_t max_duty = epwm_tbprd * 0.9;
	uint16_t min_duty = epwm_tbprd * 0.1;
	uint16_t t0, t1, t2;
	uint16_t ta, tb, tc;

if (ux >= 0.0 && uy >= 0.0 && ux >= uy) {           	//区域1
		t1 = ts * (ux - uy) / u_dc;
		t2 = sqrt_2 * ts * uy / u_dc;
		calculations_and_checks();
		pwmDuty[0] = ta;
		pwmDuty[1] = tc;
		pwmDuty[2] = tb;

	} else if (ux >= 0.0 && uy >= 0.0 && ux < uy) {    	//区域2
	    t1 = ts * (uy - ux) / u_dc;
	    t2 = sqrt_2 * ts * ux / u_dc;
	    calculations_and_checks();
		pwmDuty[0] = tb;
		pwmDuty[1] = tc;
		pwmDuty[2] = ta;

	} else if (ux <= 0.0 && uy > 0.0) {                	//区域3
	    t1 = ts * uy / u_dc;
	    t2 = - ts * ux / u_dc;
	    calculations_and_checks();
		pwmDuty[0] = tc;
		pwmDuty[1] = tb;
		pwmDuty[2] = ta;

	} else if (ux <= 0.0 && uy <= 0.0 && ux <= uy) {	//区域4
	    t1 = - sqrt_2 * ts * uy / u_dc;
	    t2 = ts * (uy - ux) / u_dc;
	    calculations_and_checks();
		pwmDuty[0] = tc;
		pwmDuty[1] = ta;
		pwmDuty[2] = tb;

	} else if (ux < 0.0 && uy <= 0.0 && ux > uy) {		//区域5
	    t1 = - sqrt_2 * ts * ux / u_dc;
	    t2 = ts * (ux - uy) / u_dc;
	    calculations_and_checks();
		pwmDuty[0] = tb;
		pwmDuty[1] = ta;
		pwmDuty[2] = tc;

	} else if (ux > 0.0 && uy < 0.0) {					//区域6
	    t1 = ts * ux / u_dc;
	    t2 = - ts * uy / u_dc;
	    calculations_and_checks();
		pwmDuty[0] = ta;
		pwmDuty[1] = tb;
		pwmDuty[2] = tc;

	}
}


void svpwm() {
	tick = 1e-8f * epwm_tbprd * 2U;
	float ux = CalculPI_(0);
	float uy = CalculPI_(1);
	modulate(ux, uy);
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
		pwmDuty[i] = epwm_tbprd;
		refCurrent[i] = 0.0f;
		currIntegral[i] = 0.0f;
	}
	currentLoopPI.P = 35;
	currentLoopPI.I = 1.8;
	s_PID[0].Proportion = 0.2;    // 0.4-0.5
	s_PID[0].Integral = 5;
	s_PID[0].Derivative = 0.00005;
	s_PID[0].SetPoint = 1672;
	s_PID[0].LastV = 0.0;


	s_PID[1].Proportion = 0.4;    // 0.4-0.5
	s_PID[1].Integral = 5;
	s_PID[1].Derivative = 0.0001;
	s_PID[1].SetPoint = 2009;
	s_PID[1].LastV = 0.0;


	s_PID[2].Proportion = 0.4;
	s_PID[2].Integral = 5;
	s_PID[2].Derivative = 0.0001;
	s_PID[2].SetPoint = 2205;
	s_PID[2].LastV = 0.0;


	s_PID[3].Proportion = 0.45;
	s_PID[3].Integral = 10;
	s_PID[3].Derivative = 0.00065;
	s_PID[3].SetPoint = 2205;
	s_PID[3].LastV = 0.0;


	s_PID[4].Proportion = 0.45;
	s_PID[4].Integral = 10;
	s_PID[4].Derivative = 0.00065;
	s_PID[4].SetPoint = 1292;
	s_PID[4].LastV = 0.0;

	sampling_times = 0;
	loop_sel = 0;
	pos_pid_sel = 0;
	cur_pid_sel = 0;
	shift_phase = 2500;
	epwm_tbprd = 2500;
}

