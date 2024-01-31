// ############################################################
#ifndef PI_Cale_H
#define PI_Cale_H
// ############################################################
#include "SysConfig.h"
// 电转速60度时间
extern uint8_t flag_SpeedTime;
extern uint16_t SpeedTimeCnt;
extern uint16_t SpeedTime;
extern uint16_t SpeedTimeTemp;
extern uint32_t SpeedTimeSum;
extern uint16_t Last_Speed;
// 速度PID
// #define INIT_PURPOSE 0
// #define RUN_PURPOSE 1
#define SPEEDLOOPCNT 100 // 3 速度占空比调整周期

extern float KQ; // 过程噪声方差
extern float KR; // 测量噪声方差
#define FirstOrder_LPF_Cacl(Xn, Yn_1, a) \
	Yn_1 = (1 - a) * Yn_1 + a * Xn; // Xn:in;Yn:out;a:系数

// #define _IQmpy(A, B) ((A) * (B))

// #define UP16LIMIT(var, max, min)               \
// 	{                                          \
// 		(var) = (var) > (max) ? (max) : (var); \
// 		(var) = (var) < (min) ? (min) : (var); \
// 	}
// #define PID_CALC(v)                       \
// 	v.Err = v.Ref - v.Fdb;                \
// 	v.Up = _IQmpy(v.Kp, v.Err);           \
// 	v.Ui = v.Ui + _IQmpy(v.Ki, v.Up);     \
// 	UP16LIMIT(v.Ui, v.OutMax, v.OutMin);  \
// 	v.Ud = v.Kd * (v.Up - v.Up1);         \
// 	v.Out = v.Up + v.Ui + v.Ud;           \
// 	UP16LIMIT(v.Out, v.OutMax, v.OutMin); \
// 	v.Up1 = v.Up;

/*********************************************************************************************************
  电机PID控制结构体
*********************************************************************************************************/
typedef struct
{
	// float now_T;		 // 当前温度
	// unsigned char set_T; // 设定温度
	// unsigned char calc_cycle;
	float pwm_out;			// 当前的pwm宽度
	unsigned short pwm_max; // 最大pwm的周期
	unsigned short pwm_min; // 最小pwm
	float P;				// P
	float I;				// I
	float D;				// D
	float Ek;				// 当前误差
	float Ek_1;				// 上一次误差
	float Ek_2;				// 上上次误差
} PID;
extern PID PID_Speed;

extern float Limit_Coff;
extern float Speed;
extern void CalcSpeedTime(void);   // 计算转速时间[60度]
extern void SpeedController(void); // 占空比控制
extern void PIDParament_Init(void);
extern void PID_bldcSpeedControl(float Idle_SpeedValue, float Actual_SpeedValue);
#endif
