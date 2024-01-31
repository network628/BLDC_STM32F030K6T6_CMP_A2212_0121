
#ifndef _SysConfig_H
#define _SysConfig_H

#ifndef _SYSTEM_FILE
#define GLOBAL_SYS_ extern
#else
#define GLOBAL_SYS_
#endif

#include "stm32f0xx.h"
#define State_DEFAULTS                              \
	{                                               \
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 \
	} // 初始化参数
#define Debug 1
enum
{
	Normal = 0,
	InNormal
};

typedef struct
{
	// uint8_t Control_Mode;	// 控制模式
	uint8_t TripFlagDMC;	// 过流 保护标志
	uint8_t drive_car;		// 开始驱动电机状态
	uint8_t olddrive_car;	// 历史开始驱动电机状态
	uint8_t clear_PWMtripz; // 清除保护标志
	uint8_t Run_mode;		// 正反转运行状态
	uint16_t Qiehuan_count; // 切换状态的计数值
	uint8_t Start_order;	// 启动PWM，启动电机
	uint16_t Duty;			// 切换状态的计数值
	uint16_t Speed_Count;	// 速度环计时
	uint16_t Current_Count; // 电流环计时
	uint16_t Aim_Speed;
	uint32_t Aim_Duty;
	uint32_t INVERSION;
	uint32_t Temp;
} State;

GLOBAL_SYS_ uint16_t nTime;
extern uint8_t Sys_Status;

// 测试变量
extern uint32_t TTEMP1;
void SYSTICK_Init(void);
GLOBAL_SYS_ void Delay_ms(uint16_t time_ms);
GLOBAL_SYS_ void Delay(__IO uint32_t nTime);
GLOBAL_SYS_ void Delay_us(uint16_t time_us);

#endif
