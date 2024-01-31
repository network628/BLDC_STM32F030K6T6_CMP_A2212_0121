#ifndef _User_Config_H
#define _User_Config_H
#include "Parameter.h"
#include "SysConfig.h"
#include "USART.h"
/*----------------------------------系统参数设置------------------------------------*/
// 系统频率
#define SYS_CLK (48000000)
#define PWM_FREQ (Cent_PWMFRE_16K)
#define PWM_ADJ (16)
#define TIM3_Prescaler (48)
#define Enable (1)
#define Disable (0)
/*----------------------------------启动运行相关设置----------------------------------*/
// 打草机  0 100  220  28  8  100  10 (带载) -100  160  25  8  100  8
// 外转子  0  100  160  13   5  100  10
// 拓邦    0   100  160  15 7   100  10
// 启动参数
// #define ALIGNMENTNMS (0)    // 定位时间
// #define ALIGNMENTDUTY (100) // 100        //定位力矩0-0.3
// #define RAMP_TIM_STA (100)  // 160         //爬坡开始步进时间
// #define RAMP_TIM_END (8)    // 12          //爬坡结束步进时间 -----------主要参数
// #define RAMP_TIM_STEP (6)   // 5           //爬坡步进时间递减增量--------主要参数
// #define RAMP_DUTY_STA (50)  // 爬坡开始力矩
// #define RAMP_DUTY_END (150) // 爬坡结束力矩
// #define RAMP_DUTY_INC (10)  // 爬坡步进力矩增量
// #define Start_Duty (150)
/**********A2212/6T****2200KV**********/
#define ALIGNMENTNMS (0)    // 定位时间
#define ALIGNMENTDUTY (100) // 100 100 //定位力矩0-0.3
#define RAMP_TIM_STA (250)   // 100 160 //爬坡开始步进时间
#define RAMP_TIM_END (8)    // 8 12  //爬坡结束步进时间 -----------主要参数
#define RAMP_TIM_STEP (3)   // 6 5   //爬坡步进时间递减增量--------主要参数
#define RAMP_DUTY_STA (30)  // 爬坡开始力矩
#define RAMP_DUTY_END (150) // 爬坡结束力矩
#define RAMP_DUTY_INC (20)  // 爬坡步进力矩增量
#define Start_Duty (150)

// 初始位置检测
#define Lock_Duty 1500    // 短脉冲幅值
#define LongPulse_Time 12 // 充电时间
#define ShortPulse_Time 3 // 短脉冲时间

// 续流屏蔽和换相时间补偿
#define Mask_TIME (5) // 5 续流屏蔽时间  无故硬件过流 调整该值(20)
// #define Delay_TIME (0) // 滤波延迟系数

// 速度阈值以及加速度
#define ADD_DUTY1 (1)                            // 开环加速幅度
#define ADD_DUTY2 (1)                            // 开环加速幅度
#define ADD_DUTY3 (1)                            // 开环加速幅度
#define LOW_DUTY_COUNT (2)                       // 低占空比开环加速加速度
#define HIGH_DUTY_COUNT (3)                      // 高占空比开环加速加速度
#define DUTYTHRESHOLD1 (0.5 * PWM_DUTYCYCLE_100) // 阈值1
#define DUTYTHRESHOLD2 (0.7 * PWM_DUTYCYCLE_100) // 阈值2
#define DUTYTHRESHOLD3 (1 * PWM_DUTYCYCLE_100)   // 阈值3

// 输出限幅
#define MIN_DUTY (PWM_MIN_DUTY)      // 输出最小值
#define MAX_DUTY (PWM_DUTYCYCLE_100) // 输出最大值

// 启动方式
#define Alig_Drag (1) // 不带位置检测
#define IPD (2)       // 带初始位置检测
#define Start_Mode (Alig_Drag)

// 停机方式
#define FREEDOWN (0)      // 自由停车
#define BRAKEDOWN (1)     // 强刹
#define BRAKEFORCE (5600) // 暂时没用
#define STOPMODE (FREEDOWN)

// 电流计算方式
#define Filter_Function (0) // 函数滤波
#define Filter_Average (1)  // 平均值滤波
#define Filter_Mode (Filter_Function)

// 调速方式
#define DIRECT_GIVE (0)    // 直接给定
#define STEPLESS_SPEED (1) // 无极变速
#define ADJ_MODE (DIRECT_GIVE)
/*--------------------------------------------PID控制区--------------------------------------*/
// 1:开环运行   2：速度闭环
// #define OPEN_LOOP_Halless         // 速度开环
#define CLOSED_SPEEDLOOP_Halless // 速度闭环

// #define OPEN_LOOP_Halless (1)        // 速度开环
// #define CLOSED_SPEEDLOOP_Halless (2) // 速度闭环
// #define Control_Mode (CLOSED_SPEEDLOOP_Halless)

#define Motor_MinSpeed (6000)   // 闭环电机转速最小值
#define Motor_MaxSpeed (22000)  // 闭环电机转速最大值
#define Motor_UserSpeed (12345) // 17000 设定转速

/*--------------------------------------保护参数设置区--------------------------------------*/
// 过欠压保护
#define VoltageProtectEnable (Enable)   // 电压保护：1--使能  0--禁止
#define Over_Protect_Voltage Volt_k(42) // (V) 直流电压过压保护值
#define Under_Protect_Voltage Volt_k(8) // (V) 直流电压欠压保护值
#define Under_OverProtectTime (1000)

// 软件过流保护
#define OverSoftCurrentProtectEnable (Disable) // --使能 0--禁止
#define OverSoftCurrent (25000.0)              // (mA) 软件过流值
#define OverSoftCurrentTime (1000)             // (ms)软件过流检测时间

// 限流
#define Limit_Current (13000) // 单位mA

// 堵转
#define StallProtectEnable (Enable)
#define Motor_Stall_Speed (30000) // 电机最大转速的1.2-2倍
#define Motor_Stall_Count (1000)

/*-----------------------------------------控制器参数-----------------------------------------*/
// 母线电压采样
#define RV_BUS_1 (10.0) // 单位KΩ 分压电阻    Y
#define RV_BUS_2 (0.68) // 单位KΩ 采样电阻    Y

// 电流采样
#define Rs (0.001)      // 单位Ω   采样电阻    Y
#define RI_BUS_1 (10.0) // 单位KΩ  运算放大器电阻 Y
#define RI_BUS_2 (2.0)  // 单位KΩ  运算放大器电阻 Y

// 电机参数设定
#define POLE_PAIR (6)      // 5 单位* 电机极对数         Y
#define BASE_VOLTAGE (36)  // 单位V 电机额定电压       Y
#define BASE_SPEED (20000) // 单位rpm 电机额定转速*1.5 Y

#define SPEEDFACTOR Speed_k(POLE_PAIR)        //(60 * SYS_CLK / 6* POLE_PAIR * TIM3_Prescaler) )// 计算转速的系数
#define CURRENTFACTOR (Current_k(0.3) * 1000) // 只需要修改0.8修正系数
#endif
