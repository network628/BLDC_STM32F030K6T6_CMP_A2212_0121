
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

#ifdef CLOSED_SPEEDLOOP_Halless // 闭环运行

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
 函 数 名  : CalcAverageSpeedTime
 功能描述  : 对转速时间求平均
 输入参数  : 无
 输出参数  : void
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
 函 数 名  : CalcSpeedTime
 功能描述  : 计算转速时间[60度]
 输入参数  : 无
 输出参数  : void
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
	static float RPM_Kx_Out = 0; // 估计的状态值
	static float RPM_KP = 1;	 // 估计误差的协方差
	// 卡尔曼滤波器的预测步骤
	// 更新估计误差的协方差
	RPM_KP = RPM_KP + KQ;
	// 计算卡尔曼增益
	float RPM_KK = RPM_KP / (RPM_KP + KR);
	// ##########################################
	// 更新状态估计
	float rpm_value = InputValue - RPM_Kx_Out;
	RPM_Kx_Out = RPM_Kx_Out + RPM_KK * rpm_value;
	// 更新估计误差的协方差
	RPM_KP = (1 - RPM_KK) * RPM_KP;
	// ##########################################
	return RPM_Kx_Out;
}

/*****************************************************************************
 函 数 名  : SpeedController
 功能描述  : 占空比控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void SpeedController() // 500us
{
	static uint8_t RPM_Cnt = 0; // AD计数器局部变量
	static float Actual_RPM = 0;
#ifdef CLOSED_SPEEDLOOP_Halless // 闭环运行
	// if (Motor.ControlMode == CLOSED_SPEEDLOOP_Halless) // 闭环运行
	{
		RPM_Cnt++;
		if (RPM_Cnt < (SPEEDLOOPCNT / 10)) // 500us * 40 = 20 000us = 20ms
		{
			RPM_Cnt = 0;
			Last_Speed = SPEEDFACTOR / SpeedTime;
			FirstOrder_LPF_Cacl(Last_Speed, Actual_RPM, 0.005);
			Motor.ActualSpeed = Get_KRM_Actual_RPM(Actual_RPM);
		}

		if (TuneDutyRatioCnt >= SPEEDLOOPCNT) // 速度环
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

// else // 开环运行
#ifdef OPEN_LOOP_Halless // 开环运行
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

		if (TuneDutyRatioCnt >= (SPEEDLOOPCNT / 5)) // 速度环 SPEEDLOOPCNT
		{
			TuneDutyRatioCnt = 0;
			if (Motor.Duty > Motor.ActualSpeed)
				Motor.Duty--;

			if (Motor.Duty < Motor.ActualSpeed)
				Motor.Duty++;

			// Motor.Duty = Motor.ActualSpeed;
			if (Motor.Duty < MIN_DUTY) // 限制输出占空比的最大最小值
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
			// 		if (Motor.Duty < MIN_DUTY) // 限制输出占空比的最大最小值
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

#ifdef CLOSED_SPEEDLOOP_Halless // 闭环运行

/*********************************************************************************************************
** Function name:           BLDC_Speed_Control_Time
** Descriptions:            变积分pid函数，在电机换向时调用
** input parameters:        Idle_SpeedValue   电机理论速度
							Actual_SpeedValue 电机实际速度
							Time         上次调用该函数至现在调用该函数的时间间隔
** output parameters:       none
** Returned value:          none
*********************************************************************************************************/
void PID_bldcSpeedControl(float Idle_SpeedValue, float Actual_SpeedValue)
{
	float pid_out;
	// 计算当前误差
	PID_Speed.Ek = Idle_SpeedValue - Actual_SpeedValue;
	// PID输出
	pid_out = (PID_Speed.P * PID_Speed.Ek) + (PID_Speed.I * PID_Speed.Ek) + (PID_Speed.D * (PID_Speed.Ek - PID_Speed.Ek_1));

	PID_Speed.pwm_out += pid_out;
	// 限制PWM的变化，使其更平滑
	if (PID_Speed.pwm_out > PID_Speed.pwm_max)
	{ /* 占空比上下限限制  1500 */
		PID_Speed.pwm_out = PID_Speed.pwm_max;
	}

	if (PID_Speed.pwm_out < PID_Speed.pwm_min)
	{ /* 占空比上下限限制  150 */
		PID_Speed.pwm_out = PID_Speed.pwm_min;
	}

	PID_Speed.Ek_2 = PID_Speed.Ek_1; // 更新过去误差
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
