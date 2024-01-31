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
  * @brief    ��ʱ��3��ʼ��   ��ʱ �ж�--����ת��
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM3_Config(void)
 {
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);  //
  /*****
	��������ʱ��=Fclk/(Prescaler+1)
	PWM����T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI����  Fclk=48MHZ)--�õ�����   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration ��ʼ����ʱ��*/
	TIM_DeInit(TIM3);
  TIM_TimeBaseStructure.TIM_Prescaler =48-1 ;      //����Ԥ��Ƶ 12.5us  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //����ģʽ  ����
  TIM_TimeBaseStructure.TIM_Period =60000-1;       //�����Զ���װ��ֵ  ��ʱ����1s 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);   
  TIM_Cmd(TIM3, DISABLE);    //ʧ�ܶ�ʱ��
            
  TIM_ClearFlag(TIM3, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);                            //���жϱ�־λ
  TIM_ITConfig(TIM3,TIM_IT_Update ,DISABLE);                              //���ж� 
		
	NVIC_InitStructure.NVIC_IRQChannel =TIM3_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;   
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
 }
 /*****************************************************************
  * @file     TIM3_Config
  * @brief    ��ʱ��3��ʼ��   ��ʱ �ж�--����
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM14_Config(void)
 {
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);  //
  /*****
	��������ʱ��=Fclk/(Prescaler+1)
	PWM����T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI����  Fclk=48MHZ)--�õ�����   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration ��ʼ����ʱ��*/
	TIM_DeInit(TIM14);
  TIM_TimeBaseStructure.TIM_Prescaler =48-1 ;      //����Ԥ��Ƶ   12.5us
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //����ģʽ  ����
  TIM_TimeBaseStructure.TIM_Period =20-1;       //�����Զ���װ��ֵ  ��ʱ����1ms 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);  
	TIM_ARRPreloadConfig(TIM14,ENABLE);//ʹ��APRԤװ�ػ�����
	 
  TIM_Cmd(TIM14, DISABLE);    //ʧ�ܶ�ʱ��	 
            
  TIM_ClearFlag(TIM14, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM14, TIM_IT_Update);                            //���жϱ�־λ
  TIM_ITConfig(TIM14,TIM_IT_Update ,ENABLE);                              //���ж� 
		
	NVIC_InitStructure.NVIC_IRQChannel =TIM14_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;     //���ȼ�����TIM1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
 }
 /*****************************************************************
  * @file     TIM16_Config
  * @brief    ��ʱ��16��ʼ��   ��ʱ �ж�--���״̬����
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM16_Config(void)
 {
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 , ENABLE);  //
  /*****
	��������ʱ��=Fclk/(Prescaler+1)
	PWM����T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI����  Fclk=48MHZ)--�õ�����   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration ��ʼ����ʱ��*/
	TIM_DeInit(TIM16);
  TIM_TimeBaseStructure.TIM_Prescaler =48-1 ;      //����Ԥ��Ƶ 1us     
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;  //����ģʽ  ����
  TIM_TimeBaseStructure.TIM_Period =1000-1;       //�����Զ���װ��ֵ  ��ʱ����500us��ʱ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);   
  TIM_Cmd(TIM16, ENABLE);    
            

  TIM_ClearFlag(TIM16, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM16, TIM_IT_Update);                            //���жϱ�־λ
  TIM_ITConfig(TIM16,TIM_IT_Update ,ENABLE);                              //���ж� 
		
	NVIC_InitStructure.NVIC_IRQChannel =TIM16_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;   
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	 
 }
/*****************************************************************
  * @file     TIM1_Config
  * @brief    ��ʱ��1PWM��ʼ��  ���ڲ���PWM
  * @param    ��
  * @retval   ��
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
	///�Ϲ�
  GPIO_InitStructure.GPIO_Pin = W_Mos_H_Pin | V_Mos_H_Pin | U_Mos_H_Pin ;  //PA10 PA9  PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	
	
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = W_Mos_L_Pin  ;  //PA10 PA9  PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_2);
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	
	GPIO_InitStructure.GPIO_Pin = U_Mos_L_Pin | V_Mos_L_Pin  ;  //PA10 PA9  PA8
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     //�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	
	/*�˿�ӳ��*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* TIM1 Configuration ---------------------------------------------------*/
  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  /*****
	��������ʱ��=Fclk/(Prescaler+1)
	PWM����T=  (PWM_Arr+1 )*(Prescaler+1)/Fclk (HSI����  Fclk=48MHZ)---��λ��s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration ��ʼ����ʱ��*/
	TIM_DeInit(TIM1);
  TIM_TimeBaseStructure.TIM_Prescaler =0 ;      //����Ԥ��Ƶ   sys/48000
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1 ;  // TIM_CounterMode_CenterAligned1 ���ϼ���ʱ������
  TIM_TimeBaseStructure.TIM_Period =(SystemCoreClock/PWM_FREQ)-1;    //�����Զ���װ��ֵ 16k  1500  --16k ��Ӧ62.5us 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;   //����������ٴβŽ��ж�  0��ʾ���1�� ����1��  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   //TIM1����PWM����
	TIM_PrescalerConfig(TIM1,0,TIM_PSCReloadMode_Immediate);  //Ԥ��Ƶֵ��ʱװ��
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ʹ��APRԤװ�ػ�����


  /* Channel 1, 2,3  Configuration in PWM mode */
   /* Channel 1, 2,3  Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //PWMģʽ2:CNT>CCR �����Ч
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //���ʹ��
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//�������ʹ��
  TIM_OCInitStructure.TIM_OCPolarity =   TIM_OCPolarity_High;     //���ó�ʼ����  -�ߵ�ƽ 
  TIM_OCInitStructure.TIM_OCNPolarity =  TIM_OCNPolarity_Low;  //�����������--�ߵ�ƽ   ���ű�
  TIM_OCInitStructure.TIM_OCIdleState =  TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��1  ��ʼ������Ƚϲ���

 //��������
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 100; //>1US >50   
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);	
		

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��2

  TIM_OCInitStructure.TIM_Pulse = 0;      //ʹ��ͨ��3
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 0;//ʹ��ͨ��4
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
  TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);  //��ʼ״̬ �Ϲ����0-��ֹ
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);//��ʼ״̬  �¹����1-����
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_ClearFlag(TIM1, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);                             //���жϱ�־λ
  TIM_ITConfig(TIM1,TIM_IT_Update ,ENABLE);                              //��ֹ�ж� 

	NVIC_InitStructure.NVIC_IRQChannel =TIM1_BRK_UP_TRG_COM_IRQn;// TIM1_BRK_UP_TRG_COM_IRQn
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2; //
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ�ܶ�ʱ��
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update); 
	TIM_CtrlPWMOutputs(TIM1,ENABLE);


}

/*****************************************************************
  * @file     Start_Motor
  * @brief    �������
  * @param    ��
  * @retval   ��
  ***************************************************************/
void Start_Motor(void)
{
	//�ر�
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);  //��ʼ״̬ �Ϲ����0-��ֹ
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);	
	
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);//��ʼ״̬  �¹����1-����
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U���Ϲ�ͨ��V����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q15PWM(void)      
{   

	TIM1->CCR1 = 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	  //PA10���pwm	 -U��	
	//U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //�Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); //�¹�ͬ����� -������ʱ��
	//V
  TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   //�Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); //�¹ܺ㵼ͨ ����͵�ƽ
  //W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);  //�Ϲ���� 0 
  TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);//  �¹����1-����

}
 /*****************************************************************
  * @file     MOS_Q16PWM
  * @brief    U���Ϲ�ͨ��W����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void  MOS_Q16PWM(void)
{    

  TIM1->CCR1= 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;   //PA10���pwm	 -U��		
	//U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //�Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); //�¹�ͬ����� -������ʱ��
	//V
 	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);  //���¹� ��ֹ
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);// 
  //W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   //�Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); //�¹ܺ㵼ͨ ����͵�ƽ
	
}
 /*****************************************************************
  * @file     MOS_Q26PWM
  * @brief    V���Ϲ�ͨ��W����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q26PWM(void)
{    
	
	TIM1->CCR1=0; 
	TIM1->CCR3=0;
	TIM1->CCR2 = Motor.Duty;	//PA9���pwm -V��		
	//U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);   //���¹ܽ�ֹ
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable); 
	//V
 	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);    //�Ϲ���� PWM
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);  //  �¹�ͬ��
  //W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   //�Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); //�¹ܺ㵼ͨ ����͵�ƽ
}
 /*****************************************************************
  * @file     MOS_Q24PWM
  * @brief    V���Ϲ�ͨ��U����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q24PWM(void) 
{    

	TIM1->CCR2 = Motor.Duty;	 //PA9���pwm -V��
	TIM1->CCR1 = 0;  
	TIM1->CCR3 = 0;		
	//U
   TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //�Ϲܽ�ֹ  ��� �͵�ƽ
   TIM_OC3PolarityConfig(TIM1, TIM_OCPolarity_High); 
	 TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); //�¹ܺ㵼ͨ ����͵�ƽ
	 TIM_OC3NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	//W
	 TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);   //���¹ܽ�ֹ
	 TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	//V
 	 TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);    //�Ϲ���� PWM
   TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);  //�¹����ͬ��

}

/*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W���Ϲ�ͨ��U����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q34PWM(void)
{

	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		    //PA8���pwm	-W��	
	//U
	TIM_OC3PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   //�Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC3NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);  //�¹ܺ㵼ͨ ����͵�ƽ
	//V
 	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);   //���¹ܽ�ֹ
  TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
  //W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);    //�Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);  //�¹�ͬ����� -������ʱ��


	
}
 /*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W���Ϲ�ͨ��V����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q35PWM(void)
{  

 	TIM1->CCR2 = 0; 
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		        //W		  
  //U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);  //�Ϲ���� 0��ֹ 
  TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);//  �¹����1-����
	//V
 
	TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High); 
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   //�Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); //�¹ܺ㵼ͨ ����͵�ƽ
  //W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   //�Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); //�¹�ͬ����� -������ʱ��

}

#endif
