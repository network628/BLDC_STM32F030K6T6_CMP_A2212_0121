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
	// 定义一个GPIO_InitTypeDef 类型的结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	// 使能GPIOF的外设时钟
	RCC_AHBPeriphClockCmd(Key_BLDCEnable_RCC | Current_Status_RCC, ENABLE); // 使能GPIO的外设时钟

	GPIO_InitStructure.GPIO_Pin = HALLLESS_U_GPIO_PIN | HALLLESS_V_GPIO_PIN | HALLLESS_W_GPIO_PIN; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;												   // 设置引脚为输入
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //设置引脚为推挽输出--只在引脚配置为输出 才用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	  // 设置引脚为上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	  // 设置引脚速度为50MHZ
	GPIO_Init(HALLLESS_U_GPIO_PORT, &GPIO_InitStructure); // 调用库函数，初始化GPIO
	/////INPUT
	GPIO_InitStructure.GPIO_Pin = Current_Status_Pin; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	  // 设置引脚为输入
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //设置引脚为推挽输出--只在引脚配置为输出 才用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		 // 设置引脚为上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // 设置引脚速度为50MHZ
	GPIO_Init(Current_Status_Port, &GPIO_InitStructure); // 调用库函数，初始化GPIO

	GPIO_InitStructure.GPIO_Pin = Key_BLDCEnable_Pin; // 选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	  // 设置引脚为输入
	// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //设置引脚为推挽输出--只在引脚配置为输出 才用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	 // 设置引脚为上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 // 设置引脚速度为50MHZ
	GPIO_Init(Key_BLDCEnable_Port, &GPIO_InitStructure); // 调用库函数，初始化GPIO

#if 0
	RCC_AHBPeriphClockCmd(Power_Delay_RCC,ENABLE);       //使能GPIO的外设时钟
	GPIO_InitStructure.GPIO_Pin =Power_Delay_PIN;	        //选择要用的GPIO引脚  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	        //设置引脚为输入				
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	      //设置引脚为推挽输出--只在引脚配置为输出 才用
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;          //设置引脚为上拉		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //设置引脚速度为50MHZ	
	GPIO_Init(Power_Delay_Port, &GPIO_InitStructure);  //调用库函数，初始化GPIO

#endif
}
void Exti_Config(void)
{
	/**************硬件过流中断**********/
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	/********比较器输入中断********************/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4 | EXTI_PinSource5 | EXTI_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line4 | EXTI_Line5 | EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; // 配置抢占优先级：优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能中断向量
	NVIC_Init(&NVIC_InitStructure);					// 初始化NVIC

	EXTI->IMR &= ~(EXTI_Line4); // 禁止中断
	EXTI->IMR &= ~(EXTI_Line5); // 禁止中断
	EXTI->IMR &= ~(EXTI_Line6); // 禁止中断
}
void EXTI4_15_IRQHandler(void)
{
	/*检查指定的EXTI线路出发请求发生与否*/
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		/*清除EXTI线路挂起位*/
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
		// 发生位置变化
		EXTI_ClearITPendingBit(EXTI_Line4);
		EXTI_ClearITPendingBit(EXTI_Line5);
		EXTI_ClearITPendingBit(EXTI_Line6);
		// 读取状态
		if ((mcState == mcRun) && (Sysflags.Angle_Mask == 0) && (Sysflags.ChangePhase == 0))
		{
			BemfCheck();
		}
	}
}
#endif
