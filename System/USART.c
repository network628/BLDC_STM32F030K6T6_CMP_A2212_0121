#include <string.h>
#include "USART.h"
 
// �������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
// ��׼����Ҫ��֧�ֺ���
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* FILE is typedef�� d in stdio.h. */
FILE __stdout;
 
// �ض���fputc����
// printf�������ָ��fputc����fputc���������
// ����ʹ�ô���1(USART1)���printf��Ϣ
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
  USART_InitTypeDef USART_InitStructure; // ���崮�ڳ�ʼ���ṹ��
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(USART_GPIO_RCC, ENABLE); // ʹ��GPIOB��ʱ��
  RCC_APB2PeriphClockCmd(USART_RCC, ENABLE);     // ʹ��USART��ʱ��
  RCC_APB2PeriphResetCmd(USART_RCC, ENABLE);

  GPIO_PinAFConfig(USART_GPIO_PORT, USART_TX_GPIO_PinSource, GPIO_AF_0);
  GPIO_PinAFConfig(USART_GPIO_PORT, USART_RX_GPIO_PinSource, GPIO_AF_0);

  /*USART1_TX ->PB6  USART1_RX ->PB7*/
  GPIO_InitStructure.GPIO_Pin = USART_TX | USART_RX; // ѡ�д���Ĭ������ܽ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       // ��������������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART_GPIO_PORT, &GPIO_InitStructure); // ���ú������ѽṹ�����������г�ʼ��
  RCC_USARTCLKConfig(RCC_USART1CLK_SYSCLK);
 
  /*����ͨѶ��������*/
  USART_DeInit(USART);
  USART_InitStructure.USART_BaudRate = 9600;                                      // ������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // ����λ8λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // ֹͣλ1λ
  USART_InitStructure.USART_Parity = USART_Parity_No;                             // У��λ ��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // ʹ�ܽ��պͷ�������

  USART_Init(USART, &USART_InitStructure);

  USART_ClearFlag(USART, USART_FLAG_TC);
  USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
  // ʹ��USART1
  USART_Cmd(USART, ENABLE);

  // ʹ��DMA1ʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // ����USART1��DMA����
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->TDR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dma_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // ������Ϊ����Ŀ��
  DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // ����ѡ��DMA_Mode_Circular����������ѡ��
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  // ʹ������ж�
  USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

  // ʹ��DMA1ͨ��2�ж�
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn; // DMA1ͨ��2�ж�
  NVIC_InitStructure.NVIC_IRQChannelPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // ����DMA����
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
