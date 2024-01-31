
#include "PI_Cale.h"
#include "BLDC.h"
#include "Analog.h"
#include "SysConfig.h"
#include "Timer_ISR.h"
#include "User_Config.h"
#include "Parameter.h"

uint8_t flag_SpeedTime = 0;
uint16_t SpeedTimeCnt = 0;
uint16_t SpeedTime = 0;
uint16_t SpeedTimeTemp = 0;
uint32_t SpeedTimeSum = 0;
uint16_t Last_Speed = 0;

#ifdef CLOSED_SPEEDLOOP_Halless // �ջ�����

PID PID_Speed;

void PIDParament_Init(void)
{
	PID_Speed.pwm_out = 1;
	PID_Speed.pwm_max = 800; // 1500
	PID_Speed.pwm_min = 150;
	PID_Speed.P = 0.001;
	PID_Speed.I = 0.001;
	PID_Speed.D = 0.001;
	PID_Speed.Ek = 0;
	PID_Speed.Ek_1 = 0;
	PID_Speed.Ek_2 = 0;
}

#endif

/*****************************************************************************
 �� �� ��  : CalcAverageSpeedTime
 ��������  : ��ת��ʱ����ƽ��
 �������  : ��
 �������  : void
*****************************************************************************/
void CalcAvgSpeedTime()
{
	if (!flag_SpeedTime)
	{
		SpeedTime = SpeedTimeTemp;
	}
	SpeedTimeSum += SpeedTimeTemp;

	SpeedTimeCnt++;
	if (SpeedTimeCnt >= 6)
	{
		if (!flag_SpeedTime)
		{
			flag_SpeedTime = 1;
		}
		SpeedTime = (uint16_t)(SpeedTimeSum / 6);
		SpeedTimeSum = 0;
		SpeedTimeCnt = 0;
	}
}
/*****************************************************************************
 �� �� ��  : CalcSpeedTime
 ��������  : ����ת��ʱ��[60��]
 �������  : ��
 �������  : void
*****************************************************************************/
void CalcSpeedTime()
{
	TIM_Cmd(TIM3, DISABLE);
	SpeedTimeTemp = TIM3->CNT;
	SpeedTime = SpeedTimeTemp;
	TIM3->CNT = 0;

	TIM_Cmd(TIM3, ENABLE);
	TIM14->ARR = (SpeedTimeTemp >> 1);
	TIM14->CNT = 0;

	TIM_Cmd(TIM14, ENABLE);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	Sysflags.ChangePhase = 1;
}

float Get_KRM_Actual_RPM(float InputValue)
{
	static float RPM_Kx_Out = 0; // ���Ƶ�״ֵ̬
	static float RPM_KP = 1;	 // ��������Э����
	// �������˲�����Ԥ�ⲽ��
	// ���¹�������Э����
	RPM_KP = RPM_KP + KQ;
	// ���㿨��������
	float RPM_KK = RPM_KP / (RPM_KP + KR);
	// ##########################################
	// ����״̬����
	float rpm_value = InputValue - RPM_Kx_Out;
	RPM_Kx_Out = RPM_Kx_Out + RPM_KK * rpm_value;
	// ���¹�������Э����
	RPM_KP = (1 - RPM_KK) * RPM_KP;
	// ##########################################
	return RPM_Kx_Out;
}

/*****************************************************************************
 �� �� ��  : SpeedController
 ��������  : ռ�ձȿ���
 �������  : ��
 �������  : void
*****************************************************************************/
void SpeedController() // 500us
{
	static uint8_t RPM_Cnt = 0; // AD�������ֲ�����
	static float Actual_RPM = 0;
#ifdef CLOSED_SPEEDLOOP_Halless // �ջ�����
	// if (Motor.ControlMode == CLOSED_SPEEDLOOP_Halless) // �ջ�����
	{
		RPM_Cnt++;
		if (RPM_Cnt < (SPEEDLOOPCNT / 10)) // 500us * 40 = 20 000us = 20ms
		{
			RPM_Cnt = 0;
			Last_Speed = SPEEDFACTOR / SpeedTime;
			FirstOrder_LPF_Cacl(Last_Speed, Actual_RPM, 0.005);
			Motor.ActualSpeed = Get_KRM_Actual_RPM(Actual_RPM);
		}

		if (TuneDutyRatioCnt >= SPEEDLOOPCNT) // �ٶȻ�
		{									  // 500us * 300 = 150 000us = 150ms
			TuneDutyRatioCnt = 0;

			PID_bldcSpeedControl(UserRequireSpeed, Motor.ActualSpeed);
			if (Debug_cnt == 0) // 500us*500 = 250ms
			{
				Debug_cnt = 500;
				printf("UserRequireSpeed: %f->Motor.ActualSpeed: %f->Motor.Duty: %d->\r\n", UserRequireSpeed, Motor.ActualSpeed, Motor.Duty);
			}
		}
	}
#endif

// else // ��������
#ifdef OPEN_LOOP_Halless // ��������
	{
		// RPM_Cnt++;
		// if (RPM_Cnt < (SPEEDLOOPCNT / 10)) // 500us * 40 = 20 000us = 20ms
		// {
		// 	RPM_Cnt = 0;
		// 	Last_Speed = SPEEDFACTOR / SpeedTime;
		// 	FirstOrder_LPF_Cacl(Last_Speed, Actual_RPM, 0.005);
		// 	Motor.ActualSpeed = Get_KRM_Actual_RPM(Actual_RPM);
		// }

		RPM_Cnt++;
		if (RPM_Cnt < (SPEEDLOOPCNT / 10)) // 500us * 40 = 20 000us = 20ms
		{
			RPM_Cnt = 0;

			FirstOrder_LPF_Cacl(UserRequireSpeed, Actual_RPM, 0.005);
			Motor.ActualSpeed = Get_KRM_Actual_RPM(Actual_RPM);
		}

		if (TuneDutyRatioCnt >= (SPEEDLOOPCNT / 5)) // �ٶȻ� SPEEDLOOPCNT
		{
			TuneDutyRatioCnt = 0;
			if (Motor.Duty > Motor.ActualSpeed)
				Motor.Duty--;

			if (Motor.Duty < Motor.ActualSpeed)
				Motor.Duty++;

			// Motor.Duty = Motor.ActualSpeed;
			if (Motor.Duty < MIN_DUTY) // �������ռ�ձȵ������Сֵ
			{
				Motor.Duty = MIN_DUTY;
			}

			if (Motor.Duty >= 800)//MAX_DUTY
			{
				Motor.Duty = 800;
			}

			// Motor.StepNum++;

			// if (Motor.Duty < PWM_MIN_DUTY)
			// {
			// 	if (Motor.StepNum > LOW_DUTY_COUNT)
			// 	{
			// 		Motor.StepNum = 0;
			// 		Motor.Duty += ADD_DUTY1;
			// 	}
			// }
			// else
			// {
			// 	if (Motor.ActualSpeed <= UserRequireSpeed)
			// 	{
			// 		if (Motor.Duty <= DUTYTHRESHOLD1)
			// 		{
			// 			if (Motor.StepNum > LOW_DUTY_COUNT)
			// 			{
			// 				Motor.StepNum = 0;
			// 				Motor.Duty += ADD_DUTY1;
			// 			}
			// 		}
			// 		else if (Motor.Duty < DUTYTHRESHOLD2)
			// 		{
			// 			if (Motor.StepNum > HIGH_DUTY_COUNT)
			// 			{
			// 				Motor.StepNum = 0;
			// 				Motor.Duty += ADD_DUTY2;
			// 			}
			// 		}
			// 		else
			// 		{
			// 			if (Motor.StepNum > HIGH_DUTY_COUNT)
			// 			{
			// 				Motor.StepNum = 0;
			// 				Motor.Duty += ADD_DUTY3;
			// 			}
			// 		}

			// 		if (Motor.Duty >= MAX_DUTY)
			// 		{
			// 			Motor.Duty = MAX_DUTY;
			// 			Motor.StepNum = 0;
			// 		}
			// 	}
			// 	else if (Motor.ActualSpeed > UserRequireSpeed)
			// 	{
			// 		Motor.StepNum = 0;
			// 		Motor.Duty -= ADD_DUTY2;
			// 		if (Motor.Duty < MIN_DUTY) // �������ռ�ձȵ������Сֵ
			// 		{
			// 			Motor.Duty = MIN_DUTY;
			// 		}
			// 	}
			// }

			if (Debug_cnt == 0) // 500us*500 = 250ms
			{
				Debug_cnt = 500;
				printf("UserRequireSpeed: %f->Motor.ActualSpeed: %f->Motor.Duty: %d->\r\n", UserRequireSpeed, Motor.ActualSpeed, Motor.Duty);
			}
		}
	}
#endif
}

#ifdef CLOSED_SPEEDLOOP_Halless // �ջ�����

/*********************************************************************************************************
** Function name:           BLDC_Speed_Control_Time
** Descriptions:            �����pid�������ڵ������ʱ����
** input parameters:        Idle_SpeedValue   ��������ٶ�
							Actual_SpeedValue ���ʵ���ٶ�
							Time         �ϴε��øú��������ڵ��øú�����ʱ����
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
void PID_bldcSpeedControl(float Idle_SpeedValue, float Actual_SpeedValue)
{
	float pid_out;
	// ���㵱ǰ���
	PID_Speed.Ek = Idle_SpeedValue - Actual_SpeedValue;
	// PID���
	pid_out = (PID_Speed.P * PID_Speed.Ek) + (PID_Speed.I * PID_Speed.Ek) + (PID_Speed.D * (PID_Speed.Ek - PID_Speed.Ek_1));

	PID_Speed.pwm_out += pid_out;
	// ����PWM�ı仯��ʹ���ƽ��
	if (PID_Speed.pwm_out > PID_Speed.pwm_max)
	{ /* ռ�ձ�����������  1500 */
		PID_Speed.pwm_out = PID_Speed.pwm_max;
	}

	if (PID_Speed.pwm_out < PID_Speed.pwm_min)
	{ /* ռ�ձ�����������  150 */
		PID_Speed.pwm_out = PID_Speed.pwm_min;
	}

	PID_Speed.Ek_2 = PID_Speed.Ek_1; // ���¹�ȥ���
	PID_Speed.Ek_1 = PID_Speed.Ek;

	// // static unsigned char PID_Cnt = 0;
	// // PID_Cnt++;
	// // if (PID_Cnt >= 4) // 150ms * 7 = 1050ms
	// {
	// 	// PID_Cnt = 0;
	// 	if (Motor.Duty > PID_Speed.pwm_out)
	// 		Motor.Duty--;

	// 	if (Motor.Duty < PID_Speed.pwm_out)
	// 		Motor.Duty++;
	// }

	Motor.Duty = PID_Speed.pwm_out;
}

#endif
