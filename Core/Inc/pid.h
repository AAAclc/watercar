#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

// 增量式PID结构体（电机速度环用）
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float target;      // 目标值
    float current;     // 当前值
    float err[3];      // 误差：e(k), e(k-1), e(k-2)
    float output;      // 输出值
    float output_max;  // 输出限幅最大值
    float output_min;  // 输出限幅最小值
}IncPID_Typedef;

// 位置式PID结构体（位置/航向外环用）
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float target;      // 目标值
    float current;     // 当前值
    float err;         // 当前误差
    float err_last;    // 上一次误差
    float integral;    // 积分项
    float integral_max;// 积分限幅
    float output;      // 输出值
    float output_max;  // 输出限幅最大值
    float output_min;  // 输出限幅最小值
}PosPID_Typedef;

// 对外接口声明
void IncPID_Init(IncPID_Typedef *pid, float Kp, float Ki, float Kd, float output_max, float output_min);
void IncPID_Calc(IncPID_Typedef *pid);
void PosPID_Init(PosPID_Typedef *pid, float Kp, float Ki, float Kd, float integral_max, float output_max, float output_min);
void PosPID_Calc(PosPID_Typedef *pid);
void PID_Reset(IncPID_Typedef *inc_pid, PosPID_Typedef *pos_pid);

#endif