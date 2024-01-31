#include <string.h>
#include "USART.h"
 
// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
// 标准库需要的支持函数
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* FILE is typedef’ d in stdio.h. */
FILE __stdout;
 
// 重定向fputc函数
// printf的输出，指向fputc，由fputc输出到串口
// 这里使用串口1(USART1)输出printf信息
int fputc(int ch, FILE *f)
{ 
  while (!(USART1->ISR & USART_FLAG_TXE))
      ;
  USART_SendData(USART1, (uint16_t)ch);
  return ch;
}
#endif

#define BUFFER_SIZE 16
uint8_t dma_buffer[BUFFER_SIZE];

void USART1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; // 定义串口初始化结构体
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(USART_GPIO_RCC, ENABLE); // 使能GPIOB的时钟
  RCC_APB2PeriphClockCmd(USART_RCC, ENABLE);     // 使能USART的时钟
  RCC_APB2PeriphResetCmd(USART_RCC, ENABLE);

  GPIO_PinAFConfig(USART_GPIO_PORT, USART_TX_GPIO_PinSource, GPIO_AF_0);
  GPIO_PinAFConfig(USART_GPIO_PORT, USART_RX_GPIO_PinSource, GPIO_AF_0);

  /*USART1_TX ->PB6  USART1_RX ->PB7*/
  GPIO_InitStructure.GPIO_Pin = USART_TX | USART_RX; // 选中串口默认输出管脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // 定义输出最大速率
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure); // 调用函数，把结构体参数输入进行初始化
  RCC_USARTCLKConfig(RCC_USART1CLK_SYSCLK);
 
  /*串口通讯参数设置*/
  USART_DeInit(USART);
  USART_InitStructure.USART_BaudRate = 9600;                                      // 波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 数据位8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 停止位1位
  USART_InitStructure.USART_Parity = USART_Parity_No;                             // 校验位 无
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 使能接收和发送引脚

  USART_Init(USART, &USART_InitStructure);

  USART_ClearFlag(USART, USART_FLAG_TC);
  USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
  // 使能USART1
  USART_Cmd(USART, ENABLE);

  // 使能DMA1时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // 配置USART1的DMA传输
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->TDR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dma_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // 外设作为数据目标
  DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // 或者选择DMA_Mode_Circular，根据需求选择
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  // 使能相关中断
  USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

  // 使能DMA1通道2中断
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn; // DMA1通道2中断
  NVIC_InitStructure.NVIC_IRQChannelPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // 启动DMA传输
  DMA_Cmd(DMA1_Channel2, ENABLE);
}

// void USART_SendString(USART_TypeDef *USARTx, const char *str)
// {
//   int len = strlen(str);
//   for (int i = 0; i < len; i++)
//   {
//     while (!(USARTx->ISR & USART_FLAG_TXE))
//       ;
//     USART_SendData(USARTx, (uint8_t)str[i]);
//   }
// }

// void USART1_IRQHandler(void)
// {
//   if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//   {
//     USART_SendData(USART1, USART_ReceiveData(USART1));
//     while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
//       ;
//   }
// }
