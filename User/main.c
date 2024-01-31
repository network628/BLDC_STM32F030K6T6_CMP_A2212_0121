/****************************************************
 * 文件名  ：main.c
 **********************************************************************************/

// 头文件
#include "stm32f0xx.h"
#include "SysConfig.h"
#include "GPIO_Init.h"
#include "Analog.h"
#include "BLDC.h"
#include "Timer.h"
#include "Timer_ISR.h"
#include "PI_Cale.h"
#include "User_Config.h"
#include "Parameter.h"

/****************************************************
 * @file   main
 * @brief  Main program.
 * @param  None
 * @retval None
 **************************************************/
int main(void)
{
	/**********初始化函数*******/
	// Motor.ControlMode = Control_Mode;
	Motor.StartMode = Start_Mode;
	/*******系统函数初始化******/
	USART1_Config();
	BLDC_GPIO_Config(); // BLDC GPIO初始化
	Exti_Config();
	Delay_ms(20);
	ADC_Configuration(); // ADC初始化
	Delay_ms(20);
	TIM1_Config();	// 定时器 PWM初始化 62.5us
	TIM3_Config();	// 定时器初始化 中断--计算转速 1s
	TIM14_Config(); // 中断--换相 1ms
	TIM16_Config(); // 中断--电机状态处理 500us
	Delay_ms(20);
	mcState = mcStop; 
	printf("Printf_OK\r\n");
	while (1)
	{
		if (ADCIntProtectCnt >= 10) // 1ms*100=100ms
		{
			ADCIntProtectCnt = 0;
			Cal_AverCurrent();
		}

		// if (Debug_cnt == 0) // 500us*1000 = 500ms
		// {
		// 	Debug_cnt = 1000;
		// 	// USART_SendString(USART1, "hello_seng_string\r\n");
		// 	printf("Duty: %d->\r\n", Motor.Duty);
		// }
	}
}

/*********************************************************************************************************
	  END FILE
*********************************************************************************************************/
