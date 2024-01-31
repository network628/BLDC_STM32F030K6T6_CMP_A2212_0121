#ifndef _GPIO_FILE
#define _GPIO_FILE

#include "GPIO_Init.h"
#include "Timer.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "User_Config.h"
#include "Analog.h"
void BLDC_GPIO_Config(void)
{
	// ����һ��GPIO_InitTypeDef ���͵Ľṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	// ʹ��GPIOF������ʱ��
	RCC_AHBPeriphClockCmd(Key_BLDCEnable_RCC | Current_Status_RCC, ENABLE); // ʹ��GPIO������ʱ��

	GPIO_InitStructure.GPIO_Pin = HALLLESS_U_GPIO_PIN | HALLLESS_V_GPIO_PIN | HALLLESS_W_GPIO_PIN; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;												   // ��������Ϊ����
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //��������Ϊ�������--ֻ����������Ϊ��� ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	  // ��������Ϊ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  // ���������ٶ�Ϊ50MHZ
	GPIO_Init(HALLLESS_U_GPIO_PORT, &GPIO_InitStructure); // ���ÿ⺯������ʼ��GPIO
	/////INPUT
	GPIO_InitStructure.GPIO_Pin = Current_Status_Pin; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	  // ��������Ϊ����
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //��������Ϊ�������--ֻ����������Ϊ��� ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		 // ��������Ϊ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // ���������ٶ�Ϊ50MHZ
	GPIO_Init(Current_Status_Port, &GPIO_InitStructure); // ���ÿ⺯������ʼ��GPIO

	GPIO_InitStructure.GPIO_Pin = Key_BLDCEnable_Pin; // ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	  // ��������Ϊ����
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //��������Ϊ�������--ֻ����������Ϊ��� ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	 // ��������Ϊ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // ���������ٶ�Ϊ50MHZ
	GPIO_Init(Key_BLDCEnable_Port, &GPIO_InitStructure); // ���ÿ⺯������ʼ��GPIO

#if 0
	RCC_AHBPeriphClockCmd(Power_Delay_RCC,ENABLE);       //ʹ��GPIO������ʱ��
	GPIO_InitStructure.GPIO_Pin =Power_Delay_PIN;	        //ѡ��Ҫ�õ�GPIO����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	        //��������Ϊ����				
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //��������Ϊ�������--ֻ����������Ϊ��� ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;          //��������Ϊ����		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //���������ٶ�Ϊ50MHZ	
	GPIO_Init(Power_Delay_Port, &GPIO_InitStructure);  //���ÿ⺯������ʼ��GPIO

#endif
}
void Exti_Config(void)
{
	/**************Ӳ�������ж�**********/
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	/********�Ƚ��������ж�********************/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4 | EXTI_PinSource5 | EXTI_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line4 | EXTI_Line5 | EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; // ������ռ���ȼ������ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // ʹ���ж�����
	NVIC_Init(&NVIC_InitStructure);					// ��ʼ��NVIC

	EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
	EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
	EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
}
void EXTI4_15_IRQHandler(void)
{
	/*���ָ����EXTI��·�������������*/
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		/*���EXTI��·����λ*/
		EXTI_ClearITPendingBit(EXTI_Line15);
		if ((Flag_HardOverCurr == 1) && (mcState != mcStop))
		{

			mcFault = HardWare_OverCurrent;
			mcState = mcStop;
			MotorStop();
			printf("-----Flag_HardOverCurr==1\r\n");
		}
	}
	if ((EXTI_GetITStatus(EXTI_Line4) != RESET) || (EXTI_GetITStatus(EXTI_Line5) != RESET) || (EXTI_GetITStatus(EXTI_Line6) != RESET))
	{
		// ����λ�ñ仯
		EXTI_ClearITPendingBit(EXTI_Line4);
		EXTI_ClearITPendingBit(EXTI_Line5);
		EXTI_ClearITPendingBit(EXTI_Line6);
		// ��ȡ״̬
		if ((mcState == mcRun) && (Sysflags.Angle_Mask == 0) && (Sysflags.ChangePhase == 0))
		{
			BemfCheck();
		}
	}
}
#endif
