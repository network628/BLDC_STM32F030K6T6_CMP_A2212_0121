#ifndef  _TIM_FILE
#define  _TIM_FILE

#include "SysConfig.h" 

#include "Timer.h"
#include "GPIO_Init.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "User_Config.h"
/*****************************************************************
  * @file     TIM3_Config
  * @brief    定时器3初始化   定时 中断--计算转速
  * @param    无
  * @retval   无
  ***************************************************************/
 void TIM3_Config(void)
 {
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);  //
  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)--得到的是   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
	TIM_DeInit(TIM3);
  TIM_TimeBaseStructure.TIM_Prescaler =48-1 ;      //设置预分频 12.5us  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //计数模式  向下
  TIM_TimeBaseStructure.TIM_Period =60000-1;       //设置自动重装载值  定时周期1s 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //时钟分频系数
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);   
  TIM_Cmd(TIM3, DISABLE);    //失能定时器
            
  TIM_ClearFlag(TIM3, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);                            //清中断标志位
  TIM_ITConfig(TIM3,TIM_IT_Update ,DISABLE);                              //打开中断 
		
	NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;   
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
 }
 /*****************************************************************
  * @file     TIM3_Config
  * @brief    定时器3初始化   定时 中断--换相
  * @param    无
  * @retval   无
  ***************************************************************/
 void TIM14_Config(void)
 {
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);  //
  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)--得到的是   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
	TIM_DeInit(TIM14);
  TIM_TimeBaseStructure.TIM_Prescaler =48-1 ;      //设置预分频   12.5us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //计数模式  向下
  TIM_TimeBaseStructure.TIM_Period =20-1;       //设置自动重装载值  定时周期1ms 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //时钟分频系数
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);  
	TIM_ARRPreloadConfig(TIM14,ENABLE);//使能APR预装载缓冲器
	 
  TIM_Cmd(TIM14, DISABLE);    //失能定时器	 
            
  TIM_ClearFlag(TIM14, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM14, TIM_IT_Update);                            //清中断标志位
  TIM_ITConfig(TIM14,TIM_IT_Update ,ENABLE);                              //打开中断 
		
	NVIC_InitStructure.NVIC_IRQChannel =TIM14_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;     //优先级低于TIM1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
 }
 /*****************************************************************
  * @file     TIM16_Config
  * @brief    定时器16初始化   定时 中断--电机状态处理
  * @param    无
  * @retval   无
  ***************************************************************/
 void TIM16_Config(void)
 {
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , ENABLE);  //
  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)--得到的是   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
	TIM_DeInit(TIM16);
  TIM_TimeBaseStructure.TIM_Prescaler =48-1 ;      //设置预分频 1us     
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //计数模式  向下
  TIM_TimeBaseStructure.TIM_Period =1000-1;       //设置自动重装载值  定时周期500us定时
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //时钟分频系数
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);   
  TIM_Cmd(TIM16, ENABLE);    
            

  TIM_ClearFlag(TIM16, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM16, TIM_IT_Update);                            //清中断标志位
  TIM_ITConfig(TIM16,TIM_IT_Update ,ENABLE);                              //打开中断 
		
	NVIC_InitStructure.NVIC_IRQChannel =TIM16_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;   
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
 }
/*****************************************************************
  * @file     TIM1_Config
  * @brief    定时器1PWM初始化  用于产生PWM
  * @param    无
  * @retval   无
  ***************************************************************/
 void TIM1_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_BDTRInitTypeDef     TIM_BDTRInitStructure;

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;  //
  /********************************
	U:H -PA10-CH3     L -PB1-CH3N
  V:H-PA9-CH2       L-PB0-CH2N
  W:H-PA8-CH1       L-PA7-CH1N
	*************************************/
	/* GPIO Configuration ---------------------------------------------------
    GPIOA, Clocks enable */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB , ENABLE);
  
  /* GPIOA Configuration: Channel 1, 2, 3, 4 and Channel 1N as alternate function push-pull */
	///上管
  GPIO_InitStructure.GPIO_Pin = W_Mos_H_Pin | V_Mos_H_Pin | U_Mos_H_Pin ;  //PA10 PA9  PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = W_Mos_L_Pin  ;  //PA10 PA9  PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	
	GPIO_InitStructure.GPIO_Pin = U_Mos_L_Pin | V_Mos_L_Pin  ;  //PA10 PA9  PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
	/*端口映射*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* TIM1 Configuration ---------------------------------------------------*/
  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  /*****
	计数器的时钟=Fclk/(Prescaler+1)
	PWM周期T=  (PWM_Arr+1 )*(Prescaler+1)/Fclk (HSI配置  Fclk=48MHZ)---单位是s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration 初始化定时器*/
	TIM_DeInit(TIM1);
  TIM_TimeBaseStructure.TIM_Prescaler =0 ;      //设置预分频   sys/48000
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1 ;  // TIM_CounterMode_CenterAligned1 向上计数时候被设置
  TIM_TimeBaseStructure.TIM_Period =(SystemCoreClock/PWM_FREQ)-1;    //设置自动重装载值 16k  1500  --16k 对应62.5us 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //时钟分频系数
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;   //计数溢出多少次才进中断  0表示溢出1次 进入1次  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   //TIM1决定PWM周期
	TIM_PrescalerConfig(TIM1,0,TIM_PSCReloadMode_Immediate);  //预分频值即时装入
	TIM_ARRPreloadConfig(TIM1,ENABLE);//使能APR预装载缓冲器


  /* Channel 1, 2,3  Configuration in PWM mode */
   /* Channel 1, 2,3  Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //PWM模式2:CNT>CCR 输出有效
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //输出使能
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//互补输出使能
  TIM_OCInitStructure.TIM_OCPolarity =   TIM_OCPolarity_High;     //设置初始极性  -高电平 
  TIM_OCInitStructure.TIM_OCNPolarity =  TIM_OCNPolarity_Low;  //互补输出极性--高电平   下桥臂
  TIM_OCInitStructure.TIM_OCIdleState =  TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //使能通道1  初始化输出比较参数

 //死区设置
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 100; //>1US >50   
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);	
		

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //使能通道2

  TIM_OCInitStructure.TIM_Pulse = 0;      //使能通道3
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 0;//使能通道4
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);  //初始状态 上管输出0-截止
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);//初始状态  下管输出1-截至
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_ClearFlag(TIM1, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);                             //清中断标志位
  TIM_ITConfig(TIM1,TIM_IT_Update ,ENABLE);                              //禁止中断 

	NVIC_InitStructure.NVIC_IRQChannel =TIM1_BRK_UP_TRG_COM_IRQn;// TIM1_BRK_UP_TRG_COM_IRQn
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2; //
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM1, ENABLE);  //使能定时器
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update); 
	TIM_CtrlPWMOutputs(TIM1,ENABLE);


}

/*****************************************************************
  * @file     Start_Motor
  * @brief    开启电机
  * @param    无
  * @retval   无
  ***************************************************************/
void Start_Motor(void)
{
	//关闭
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);  //初始状态 上管输出0-截止
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);	
	
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);//初始状态  下管输出1-截至
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q15PWM(void)      
{   

	TIM1->CCR1 = 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	  //PA10输出pwm	 -U上	
	//U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //上管输出pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); //下管同步输出 -带死区时间
	//V
  TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   //上管截止  输出 低电平
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); //下管恒导通 输出低电平
  //W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);  //上管输出 0 
  TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);//  下管输出1-截至

}
 /*****************************************************************
  * @file     MOS_Q16PWM
  * @brief    U相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void  MOS_Q16PWM(void)
{    

  TIM1->CCR1= 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;   //PA10输出pwm	 -U上		
	//U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //上管输出pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); //下管同步输出 -带死区时间
	//V
 	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);  //上下管 截止
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);// 
  //W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   //上管截止  输出 低电平
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); //下管恒导通 输出低电平
	
}
 /*****************************************************************
  * @file     MOS_Q26PWM
  * @brief    V相上管通和W相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q26PWM(void)
{    
	
	TIM1->CCR1=0; 
	TIM1->CCR3=0;
	TIM1->CCR2 = Motor.Duty;	//PA9输出pwm -V上		
	//U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);   //上下管截止
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable); 
	//V
 	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);    //上管输出 PWM
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);  //  下管同步
  //W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   //上管截止  输出 低电平
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); //下管恒导通 输出低电平
}
 /*****************************************************************
  * @file     MOS_Q24PWM
  * @brief    V相上管通和U相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q24PWM(void) 
{    

	TIM1->CCR2 = Motor.Duty;	 //PA9输出pwm -V上
	TIM1->CCR1 = 0;  
	TIM1->CCR3 = 0;		
	//U
   TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //上管截止  输出 低电平
   TIM_OC3PolarityConfig(TIM1, TIM_OCPolarity_High); 
	 TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); //下管恒导通 输出低电平
	 TIM_OC3NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	//W
	 TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);   //上下管截止
	 TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	//V
 	 TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);    //上管输出 PWM
   TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);  //下管输出同步

}

/*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W相上管通和U相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q34PWM(void)
{

	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		    //PA8输出pwm	-W上	
	//U
	TIM_OC3PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //上管截止  输出 低电平
	TIM_OC3NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);  //下管恒导通 输出低电平
	//V
 	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);   //上下管截止
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
  //W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);    //上管输出pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);  //下管同步输出 -带死区时间


	
}
 /*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W相上管通和V相的下管通  其他关闭
  * @param    无
  * @retval   无
  ***************************************************************/
void MOS_Q35PWM(void)
{  

 	TIM1->CCR2 = 0; 
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		        //W		  
  //U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);  //上管输出 0截止 
  TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);//  下管输出1-截至
	//V
 
	TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   //上管截止  输出 低电平
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); //下管恒导通 输出低电平
  //W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   //上管输出pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); //下管同步输出 -带死区时间

}

#endif
