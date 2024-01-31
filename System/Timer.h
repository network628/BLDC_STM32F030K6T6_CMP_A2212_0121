#ifndef _Timer_H
#define _Timer_H

#ifndef _TIM_FILE
#define GLOBAL_TIM_ extern
#else
#define GLOBAL_TIM_
#endif
#include "SysConfig.h"

/*-----------------------------------Ƶ��ѡ��-------------------------------------*/
// PWM����-���ϼ���
#define DEF_PWMFRE_8K 8000   // 48000000/8000=6000
#define DEF_PWMFRE_16K 16000 // 48000000/16000=3000
#define DEF_PWMFRE_20K 20000 // 48000000/20000=2400
#define DEF_PWMFRE_25K 25000 // 48000000/25000=1920
#define DEF_PWMFRE_30K 30000 // 48000000/30000=1600

// PWM����-�м����
#define Cent_PWMFRE_4K 8000   // 48000000/8000=6000
#define Cent_PWMFRE_8K 16000  // 48000000/16000=3000
#define Cent_PWMFRE_10K 20000 // 48000000/20000=2400
#define Cent_PWMFRE_15K 30000 // 48000000/30000=1600
#define Cent_PWMFRE_16K 32000 // 48000000/32000=1500

#define U_Mos_H_Pin GPIO_Pin_10
#define V_Mos_H_Pin GPIO_Pin_9
#define W_Mos_H_Pin GPIO_Pin_8

#define U_Mos_L_Pin GPIO_Pin_1
#define V_Mos_L_Pin GPIO_Pin_0
#define W_Mos_L_Pin GPIO_Pin_7

void TIM1_Config(void);
void TIM3_Config(void);
void TIM14_Config(void);
void TIM16_Config(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
GLOBAL_TIM_ void Stop_Motor(void);
GLOBAL_TIM_ void Start_Motor(void);
GLOBAL_TIM_ void MOS_Q15PWM(void);
GLOBAL_TIM_ void MOS_Q16PWM(void);
GLOBAL_TIM_ void MOS_Q26PWM(void);
GLOBAL_TIM_ void MOS_Q24PWM(void);
GLOBAL_TIM_ void MOS_Q34PWM(void);
GLOBAL_TIM_ void MOS_Q35PWM(void);
#endif
