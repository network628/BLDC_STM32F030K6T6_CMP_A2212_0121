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
 * @brief     ��ʱ��3 �ж�--����ת�� 1s
 * @param    ��
 * @retval   ��
 ***************************************************************/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) // �Ƿ����ж�
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}
/*****************************************************************
 * @file     TIM16_IRQHandler
 * @brief    ��ʱ��16 �ж�--���״̬���� 500us
 * @param    ��
 * @retval   ��
 ***************************************************************/
void TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET) // �Ƿ����ж�
	{
		UserSpeedControl(); // �ӿں���
		MotorControl();		// ���״̬����
		Fault_Detection();
		// ��������
		TuneDutyRatioCnt++;
		ADCIntProtectCnt++;
		if (Debug_cnt > 0)
			Debug_cnt--;

		TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
	}
}
/*****************************************************************
 * @file     TIM14_IRQHandler
 * @brief     ��ʱ��14 �ж�--���� 1ms
 * @param    ��
 * @retval   ��
 ***************************************************************/
void TIM14_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET) // �Ƿ����ж�
	{
		if (Sysflags.Angle_Mask == 1)
		{
			Sysflags.Angle_Mask = 0;
			TIM_ITConfig(TIM14, TIM_IT_Update, DISABLE);
		}
		if (Sysflags.ChangePhase == 1)
		{
			EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
			EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
			EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
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
 * @brief    pwm�����ж�-������ж�
 * @param    ��
 * @retval   ��
 ***************************************************************/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) // �Ƿ����ж�
	{
		switch (mcState)
		{
		case mcAlignment:
			if (Motor.StartMode == IPD)
			{
				Align_pos_check_proc(); // ��ʼλ�ü��
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
