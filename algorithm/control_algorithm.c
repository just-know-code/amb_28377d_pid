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

struct PID                         /* ����PID�ṹ�庯�� */
{
	int16_t      SetPoint;
	//	 float    Proportion0;                 /* �������PID */
	//	 float    Derivative0;
	float    Proportion;
	float    Integral;
	float    Derivative;
	int16_t      LastV;
};

struct PID  s_PID[5];

#define maxi  175.0f     //���ֱ���
#define mini -175.0f       //���ֱ���

#define DT 0.00005f

void PIDCalc(int16_t channel, int16_t NextPoint) {

	float i_dError, i_Error;
	i_Error = NextPoint - s_PID[channel].SetPoint; /* ƽ��λ�����趨��Ĳ�ֵ */
	i_dError = NextPoint - s_PID[channel].LastV; /* ��������֮��Ĳ�ֵ */
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
	refCurrent[channel * 2] = (float)(current_bias[channel] + pid_out) / 235.5f;//9 · ���� sensor
	refCurrent[channel * 2 + 1] = (float)(current_bias[channel] - pid_out) / 235.5f;//10  · ����
}
	/*
	 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 *����
	 *		��������0��0 ����Ȧ			1����Ȧ
	 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 *����A��			       ��������1��
	 * 					2 ����Ȧ
	 *
	 * 	      ��������2��4����Ȧ			5����Ȧ
	 *
	 * 					3 ����Ȧ
	 *%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 *����B:
	 *				        ��������3��
	 * 					6 ����Ȧ
	 *
	 * 	       ��������4��8 ����Ȧ			9����Ȧ
	 *
	 * 					7 ����Ȧ
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

void CalculPI_I1() {

	float propotion, integral, error, outcome;
	error = refCurrent[1] - coilCurrent[1];
	currIntegralArray[1] += error * DT;
	propotion = currentLoopPI.P * error;
	if (currIntegralArray[1] > MAX_CURR_INTEGRAL)
		currIntegralArray[1] = MAX_CURR_INTEGRAL;
	if (currIntegralArray[1] < MIN_CURR_INTEGRAL)
		currIntegralArray[1] = MIN_CURR_INTEGRAL;

	integral = currentLoopPI.I * currIntegralArray[1];
	outcome = propotion + integral;
	if (outcome > MAX_PWM_DUTY)
		outcome = MAX_PWM_DUTY;
	if (outcome < MIN_PWM_DUTY)
		outcome = MIN_PWM_DUTY;
	outcome += 0.5;
	uint16_t pwm_duty = outcome * EPWM_TIMER_TBPRD;
	pwmDuty[1] = pwm_duty;
	pwmDuty[2] = pwm_duty;
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
		refCurrent[i] = 0.0f;
	}

	currentLoopPI.P = 0.32;
	currentLoopPI.I = 0.1;

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
	s_PID[2].Derivative = 0.0001;  // 50.0
	s_PID[2].SetPoint = 2205;        //501
	s_PID[2].LastV = 0.0;


	s_PID[3].Proportion = 0.45;
	s_PID[3].Integral = 10;
	s_PID[3].Derivative = 0.00065;  // 50.0
	s_PID[3].SetPoint = 2205;        //273
	s_PID[3].LastV = 0.0;


	s_PID[4].Proportion = 0.45;
	s_PID[4].Integral = 10;
	s_PID[4].Derivative = 0.00065;  // 50.0
	s_PID[4].SetPoint = 1292;
	s_PID[4].LastV = 0.0;

	sampling_times = 0;
	loop_sel = 0;
	pos_pid_sel = 0;
	cur_pid_sel = 0;
	shift_phase = 2500;
}

