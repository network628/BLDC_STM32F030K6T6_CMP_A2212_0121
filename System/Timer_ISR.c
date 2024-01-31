#include "Timer_ISR.h"
#include "Analog.h"
#include "BLDC.h"
#include "PI_Cale.h"
#include "Timer.h"
#include "User_Config.h"

uint16_t ADCIntProtectCnt = 0;
uint16_t TuneDutyRatioCnt = 0;
uint8_t Charger_Time = 0;
uint16_t Debug_cnt = 0;//500us

/*****************************************************************
 * @file     TIM3_IRQHandler
 * @brief     定时器3 中断--计算转速 1s
 * @param    无
 * @retval   无
 ***************************************************************/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) // 是否发生中断
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}
/*****************************************************************
 * @file     TIM16_IRQHandler
 * @brief    定时器16 中断--电机状态处理 500us
 * @param    无
 * @retval   无
 ***************************************************************/
void TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET) // 是否发生中断
	{
		UserSpeedControl(); // 接口函数
		MotorControl();		// 电机状态流程
		Fault_Detection();
		// 计数变量
		TuneDutyRatioCnt++;
		ADCIntProtectCnt++;
		if (Debug_cnt > 0)
			Debug_cnt--;

		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
	}
}
/*****************************************************************
 * @file     TIM14_IRQHandler
 * @brief     定时器14 中断--换相 1ms
 * @param    无
 * @retval   无
 ***************************************************************/
void TIM14_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET) // 是否发生中断
	{
		if (Sysflags.Angle_Mask == 1)
		{
			Sysflags.Angle_Mask = 0;
			TIM_ITConfig(TIM14, TIM_IT_Update, DISABLE);
		}
		if (Sysflags.ChangePhase == 1)
		{
			EXTI->IMR &= ~(EXTI_Line4); // 禁止中断
			EXTI->IMR &= ~(EXTI_Line5); // 禁止中断
			EXTI->IMR &= ~(EXTI_Line6); // 禁止中断
			Startup_Turn();
			TIM14->CNT = 0;
			TIM14->ARR = Mask_TIME;
			Sysflags.Angle_Mask = 1;
			Sysflags.ChangePhase = 0;
		}
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
	}
}
/*****************************************************************
 * @file    TIM1_BRK_UP_TRG_COM_IRQHandler
 * @brief    pwm周期中断-过零点判断
 * @param    无
 * @retval   无
 ***************************************************************/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // 是否发生中断
	{
		switch (mcState)
		{
		case mcAlignment:
			if (Motor.StartMode == IPD)
			{
				Align_pos_check_proc(); // 初始位置检测
				if ((Flag_adc == 1) || (Flag_Charger == 1))
				{
					Charger_Time++;
				}
				if (Flag_adc == 1)
				{
					if (pos_idx <= 5)
					{
						ADC_check_buf[pos_idx] = RegularConvData_Tab[3];
					}
				}
				if (Flag_Charger == 1)
				{
					if (Charger_Time >= LongPulse_Time)
					{
						Charger_Time = 0;
						All_Discharg();
						Flag_Charger = 0;
					}
				}
				else if (Flag_adc == 1)
				{
					if (Charger_Time >= ShortPulse_Time)
					{
						All_Discharg();
						Charger_Time = 0;
						Flag_adc = 0;
					}
				}
			}
			break;
		case mcDrag:
			StartupDrag();
			break;
		default:
			break;
		}

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
