#ifndef   _Parameter_H
#define   _Parameter_H
#include "User_Config.h"
#include "Parameter.h"

#define   Speed_k(x)                (uint32_t)(10*SYS_CLK/TIM3_Prescaler/x)  //转速因子

#define   AMP_GAIN                  (uint32_t)(RI_BUS_1/RI_BUS_2+1)

#define   Current_k(x)              (float)(3.3*x/(4095*AMP_GAIN*Rs))         //电流计算系数  0.13

#define   Volt_k(x)                 (uint16_t)(4095*RV_BUS_2*x/((RV_BUS_2+RV_BUS_1)*3.3))

#define   Tempera_Value(NTC_Value)  (uint16_t)(4095*NTC_Value/(10.0+NTC_Value))

#endif


