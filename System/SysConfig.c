#ifndef _SYS_FILE
#define _SYS_FILE

#include "SysConfig.h"

static __IO uint32_t TimingDelay;
 void TimingDelay_Decrement(void);
 /**
  * @file   SYSTICK_Init
  * @brief  初始化SYSTICK，1Ms中断1次
  * @param  无
  * @retval 无
  */
void SYSTICK_Init(void)
{		/*SystemCoreClock/1000000：1us中断1次；SystemCoreClock/ 1000：1ms中断一次*/
	if(SysTick_Config(SystemCoreClock/1000))
	{
		while(1);
	}
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void Delay_ms( uint16_t time_ms )  
{
  uint16_t i,j;
  for( i=0;i<time_ms;i++ )
  {
		for( j=0;j<2065;j++ );
  }
}
void Delay_us( uint16_t time_us )  
{
  uint16_t i,j;
  for( i=0;i<time_us;i++ )
  {
		for( j=0;j<1;j++ );
  }
}

#endif
