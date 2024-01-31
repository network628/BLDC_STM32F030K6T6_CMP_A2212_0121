#ifndef _BLDC_H
#define _BLDC_H

#ifndef _BLDC_FILE
#define GLOBAL_BLDC_ extern
#else
#define GLOBAL_BLDC_
#endif

#include "SysConfig.h"

#define NUM_AVR 3
#define BLDCSTOP 1	   /* 电机停止     */
#define BLDCSTART 2	   /* 电机启动     */
#define BLDCRUN 3	   /* 电机运行     */
#define BLDCSTARTERR 4 /*电机启动错误  */
#define BLDCTIMEERR 5
#define BLDCRUNTIMEERR 6 /*电机运行错误  */

#define Flag_DEFAULTS \
	{                 \
		0, 0, 0       \
	}

#define PWM_DUTYCYCLE_05 75
#define PWM_DUTYCYCLE_10 150
#define PWM_DUTYCYCLE_15 225
#define PWM_DUTYCYCLE_20 300
#define PWM_DUTYCYCLE_30 450
#define PWM_DUTYCYCLE_50 750
#define PWM_DUTYCYCLE_60 900
#define PWM_DUTYCYCLE_80 1200
#define PWM_DUTYCYCLE_90 1350
#define PWM_DUTYCYCLE_100 1500

#define PWM_MIN_DUTY PWM_DUTYCYCLE_10  // 最小占空比
#define PWM_MAX_DUTY PWM_DUTYCYCLE_100 // 最大占空比

#define MINIMUMSPEED 5000  // 最小转速
#define MAXIMUMSPEED 22000 // 电机最大转   DELTADUTYCYCLE		  30

// 电位器和给定速度的系数
#define Coff_1 3.6
#define TEST_MANUELL 0
#define RHEOSTATMIN 300
#define RHEOSTATMAX 3800

#define MOTOR_POLES 4
#define Motor_DelayTime 6000

typedef enum
{
	Startup = 0,
	Operation = 1,
	MotorOFF = 2,
} MasterState_T;

GLOBAL_BLDC_ MasterState_T MasterState;

typedef struct
{
	uint16_t Stall_Cnt1;
	uint16_t Stall_Cnt2;
	uint16_t Stall_Cnt3;
	uint16_t Stall_Cnt4;
	uint16_t Stall_Cnt5;
	uint16_t Stall_Cnt6;

	uint16_t OverCurr_Cnt;
	uint16_t OverUnderVoltage_Cnt;
	uint16_t OverTemp_Cnt;

} Protect;
GLOBAL_BLDC_ Protect MProtect;

typedef enum
{
	mcReady = 0,
	mcInit = 1,
	mcCharge = 2,
	mcAlignment = 3,
	mcDrag = 4,
	mcRun = 5,
	mcStop = 6,
	mcBrake = 7,
} MotorState_T;
GLOBAL_BLDC_ MotorState_T mcState;

typedef enum
{
	RunNormal = 0,
	OverUnderVoltage = 1,
	OverCurrent = 2,
	OverTemperature = 3,
	Motor_Stall_1 = 4,
	HardWare_OverCurrent = 5,
	Overtime_PhaseChange = 6,

} Fault;

GLOBAL_BLDC_ Fault mcFault;
typedef struct
{
	// uint8_t ControlMode;  /* 电机控制方式    */
	uint8_t State;		  /* 电机当前状态   */
	uint16_t TheorySpeed; /* 电机理论速度   */
	float ActualSpeed; /* 电机实际速度   */
	uint16_t ActAvgSpeed; /* 电机平均速度   */
	uint16_t OpenSpeed;
	uint16_t Duty; /* 电机PWM占空比  */
	uint16_t Order;
	uint16_t Current;
	uint16_t StepNum;
	uint8_t StartMode;
	uint8_t Current_Phase;
	uint8_t Last_Phase;
} BLDC;

GLOBAL_BLDC_ BLDC Motor;
GLOBAL_BLDC_ uint16_t ADCTimeCnt;
GLOBAL_BLDC_ uint16_t SamePhaseCnt;
// GLOBAL_BLDC_ uint16_t UserSpeedSample;
GLOBAL_BLDC_ uint8_t pos_check_stage;
GLOBAL_BLDC_ uint8_t Flag_adc;
GLOBAL_BLDC_ uint8_t Flag_Charger;
GLOBAL_BLDC_ uint16_t ADC_check_buf[6];
GLOBAL_BLDC_ uint8_t pos_idx;
GLOBAL_BLDC_ uint8_t Flag_HardOverCurr;
GLOBAL_BLDC_ uint8_t IPD_Times;
GLOBAL_BLDC_ void MotorInit(void);
GLOBAL_BLDC_ void Startup_Turn(void);
GLOBAL_BLDC_ void MotorStop(void);
GLOBAL_BLDC_ void MotorAlignment(void);
GLOBAL_BLDC_ void EnterDragInit(void);
GLOBAL_BLDC_ void EnterRunInit(void);
GLOBAL_BLDC_ void StartupDrag(void);
GLOBAL_BLDC_ void MotorControl(void);
GLOBAL_BLDC_ void UserSpeedControlInit(void);
GLOBAL_BLDC_ void UserSpeedControl(void);
GLOBAL_BLDC_ void ManulStartup_Turn(void);
GLOBAL_BLDC_ void Align_pos_check_proc(void);
GLOBAL_BLDC_ void All_Discharg(void);
GLOBAL_BLDC_ void Init_EXT(void);
GLOBAL_BLDC_ uint8_t GetComparatorState(void);

#endif
