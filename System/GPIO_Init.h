#ifndef _GPIO_Init_H
#define _GPIO_Init_H

#ifndef _GPIO_FILE
#define GLOBAL_GPIO_ extern
#else
#define GLOBAL_GPIO_
#endif

#include "SysConfig.h"

// 开关使能  PF1
#define Key_BLDCEnable_RCC RCC_AHBPeriph_GPIOF
#define Key_BLDCEnable_Port GPIOF
#define Key_BLDCEnable_Pin GPIO_Pin_1
#define RUN_STATUS GPIO_ReadInputDataBit(Key_BLDCEnable_Port, Key_BLDCEnable_Pin)

// 电流状态  PA3
#define Current_Status_RCC RCC_AHBPeriph_GPIOA
#define Current_Status_Port GPIOA
#define Current_Status_Pin GPIO_Pin_15

// 比较器输入  PA6 PA5  PA4
#define HALLLESS_U_GPIO_PORT GPIOA
#define HALLLESS_U_GPIO_PIN GPIO_Pin_6

#define HALLLESS_V_GPIO_PORT GPIOA
#define HALLLESS_V_GPIO_PIN GPIO_Pin_5

#define HALLLESS_W_GPIO_PORT GPIOA
#define HALLLESS_W_GPIO_PIN GPIO_Pin_4

#define NULL_U GPIO_ReadInputDataBit(HALLLESS_U_GPIO_PORT, HALLLESS_U_GPIO_PIN)
#define NULL_V GPIO_ReadInputDataBit(HALLLESS_V_GPIO_PORT, HALLLESS_V_GPIO_PIN)
#define NULL_W GPIO_ReadInputDataBit(HALLLESS_W_GPIO_PORT, HALLLESS_W_GPIO_PIN)

#define Power_Delay_RCC RCC_AHBPeriph_GPIOB
#define Power_Delay_Port GPIOB
#define Power_Delay_PIN GPIO_Pin_5

#define POWER_ON GPIO_SetBits(Power_Delay_Port, Power_Delay_PIN)
#define POWER_OFF GPIO_ResetBits(Power_Delay_Port, Power_Delay_PIN)
#define POWER_DELAY 1000

void BLDC_GPIO_Config(void);
void Exti_Config(void);
#endif
