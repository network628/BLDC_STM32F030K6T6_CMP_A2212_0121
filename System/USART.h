#ifndef _USART_H
#define _USART_H
 
#include <stdio.h>
#include "SysConfig.h"
#define BaudRate 9600

#define USART USART1
#define USART_RCC RCC_APB2Periph_USART1

#define USART_GPIO_RCC RCC_AHBPeriph_GPIOB
#define USART_TX_GPIO_PinSource GPIO_PinSource6
#define USART_RX_GPIO_PinSource GPIO_PinSource7
#define USART_TX GPIO_Pin_6 // out
#define USART_RX GPIO_Pin_7 // in
#define USART_GPIO_PORT GPIOB

void USART1_Config(void);
// void USART_SendString(USART_TypeDef *USARTx, const char *str);

#endif
