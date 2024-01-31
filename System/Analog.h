
#ifndef _Analog_H
#define _Analog_H

#ifndef _ADC_FILE
#define GLOBAL_ADC_ extern
#else
#define GLOBAL_ADC_
#endif
#include "SysConfig.h"
#define ADCSamp_DEFAULTS                    \
	{                                       \
		0, 0, 0, 0, 0, 0, 0, 0, 1000, 24, 0 \
	} // 初始化参数
#define mcCurOffset_DEFAULTS      \
	{                             \
		0, 0, 0, 0, 0, 0, 0, 0, 0 \
	}
#define Current_Collect_Pin GPIO_Pin_0 // 三相总电流采集引脚
#define Bat_Collect_Pin GPIO_Pin_1	   // 电池电压采集引脚
#define Adjust_R_Pin GPIO_Pin_2		   // 可调电阻电压采集引脚
#define Inst_Curr_Pin GPIO_Pin_3	   // 瞬时电流采集引脚

// #define  U_Collect_Pin        GPIO_Pin_6       //U相电压采集引脚
// #define  V_Collect_Pin        GPIO_Pin_5       //V相电压采集引脚
// #define  W_Collect_Pin        GPIO_Pin_4       //W相电压采集引脚

#define Current_Channel ADC_Channel_0
#define BatVol_Channel ADC_Channel_1
#define Adjust_R_Channel ADC_Channel_2
#define Inst_Curr_Channel ADC_Channel_3

// #define  U_Vol_Channel     ADC_Channel_6
// #define  V_Vol_Channel     ADC_Channel_5
// #define  W_Vol_Channel     ADC_Channel_4

GLOBAL_ADC_ uint8_t Filter_Count;
GLOBAL_ADC_ float Aver_Current;
GLOBAL_ADC_ float DC_TotalCurrent;
GLOBAL_ADC_ uint16_t EndTime;

#define BUFFER_SIZE 16
#define Filter_Const1 16
typedef struct
{
	uint16_t array[BUFFER_SIZE]; /*!< Buffer array to storage noisy signal to be filtered */
	uint16_t bufferIndex;		 /*!< Buffer array storage index */
} BUFFER_TYPE;

typedef struct
{
	unsigned Motor_Stop : 1;
	unsigned Angle_Mask : 1;
	unsigned ChangePhase : 1;
	unsigned SpeedTime : 1;
	unsigned I_Limt : 1;
	unsigned Systerm_On : 1;
	unsigned Run_Steady : 1;
	unsigned Run_One : 1;

} Sysflag;
GLOBAL_ADC_ Sysflag Sysflags;

typedef struct
{
	float PCB_Temp;			 // PCB温度
	float BUS_Curr;			 // 母线电流
	float PhaseW_Voltage;	 // W相电压
	float PhaseU_Voltage;	 // U相电压
	float PhaseV_Voltage;	 // V相电压
	float BUS_Voltage;		 // 母线电压DC Bus  Voltage
	float RP_speed_Voltage;	 // 电位器电压 RP1_Voltage
	int32_t Offset_BUS_Curr; // 电流偏移量
	BUFFER_TYPE busCur;		 /*!< Buffer to storage bus current in bldc control */
	int32_t Coeff_filterK1;	 // 一阶低通滤波器系数1
	int32_t Coeff_filterK2;	 // 一阶低通滤波器系数2
} ADCSamp;
GLOBAL_ADC_ ADCSamp ADCSampPare;
GLOBAL_ADC_ volatile uint16_t RegularConvData_Tab[4];
GLOBAL_ADC_ float Current;
// GLOBAL_ADC_ float Current_Filter(void);
GLOBAL_ADC_ float UserRequireSpeed;
GLOBAL_ADC_ uint16_t rr;
GLOBAL_ADC_ uint8_t rr1;
GLOBAL_ADC_ uint16_t Run_Times;
GLOBAL_ADC_ uint8_t Falg_Waiting;
GLOBAL_ADC_ uint8_t DELTADUTYCYCLE;
GLOBAL_ADC_ uint16_t Global_Current;
GLOBAL_ADC_ uint16_t Filter_Coff;
GLOBAL_ADC_ uint16_t U_Bemf, V_Bemf, W_Bemf;

GLOBAL_ADC_ uint8_t FILTER_LONG;
void ADC_Configuration(void);

GLOBAL_ADC_ void BemfCheck(void);
GLOBAL_ADC_ void JudgeErrorCommutation(void);
// GLOBAL_ADC_ void Motor_Limit_Current(void);
GLOBAL_ADC_ void Fault_OverUnderVoltage(void);
GLOBAL_ADC_ void FaultMotor_Stall(void);
GLOBAL_ADC_ void Cal_AverCurrent(void);
GLOBAL_ADC_ void Instant_Current_Cale(void);
GLOBAL_ADC_ void Fault_Detection(void);
GLOBAL_ADC_ void Fault_Soft_Overcurrent(void);

#endif
