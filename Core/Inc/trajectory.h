#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include "stm32f4xx_hal.h"
#include "odom.h"
#include "pid.h"
#include "mecanum.h"

// 系统状态定义
typedef enum
{
    STATE_IDLE = 0,  // 空闲状态
    STATE_RUNNING,    // 运行中（前往目标点）
    STATE_ARRIVED     // 已到达目标点
}SysState_Typedef;

// 控制参数宏定义
#define MAX_VEL         0.5f    // 最大线速度，单位m/s
#define MAX_OMEGA       3.0f    // 最大角速度，单位rad/s
#define ARRIVE_DIS      0.005f  // 到达阈值，单位m（5mm）
#define ACC_MAX         1.0f    // 最大加速度，单位m/s²
#define DECEL_MAX       1.0f    // 最大减速度，单位m/s²

// 对外接口声明
void Trajectory_Init(void);
void Trajectory_SetTarget(float target_x, float target_y);
void Trajectory_Stop(void);
void Trajectory_ResetOrigin(void);
void Trajectory_ControlLoop(void);
SysState_Typedef Trajectory_GetState(void);

#endif