#ifndef __MECANUM_H
#define __MECANUM_H

#include "stm32f4xx_hal.h"
#include "math.h"

// ==================== 你的底盘实测参数，必须修改！====================
#define MECANUM_R       0.04f   // 麦轮半径，单位m（80mm麦轮=0.04）
#define MECANUM_L       0.1f    // 半轴距：前后轮轴间距的一半，单位m
#define MECANUM_W       0.08f   // 半轮距：左右轮中心间距的一半，单位m
#define PPR             2200.0f // 编码器单圈总脉冲数=线数*减速比*4倍频
#define CONTROL_DT      0.01f   // 控制周期，单位s（固定10ms，和主循环一致）
// =====================================================================

// 轮子编号定义
#define WHEEL_LF    0   // 左前轮
#define WHEEL_RF    1   // 右前轮
#define WHEEL_LB    2   // 左后轮
#define WHEEL_RB    3   // 右后轮

// 底盘速度结构体
typedef struct
{
    float vx;    // 前向速度，单位m/s（向前为正）
    float vy;    // 左向速度，单位m/s（向左为正）
    float omega; // 自转角速度，单位rad/s（逆时针为正）
}ChassisSpeed_Typedef;

// 对外接口声明
void Mecanum_InverseCalc(ChassisSpeed_Typedef *speed, float *wheel_rpm);
void Mecanum_PositiveCalc(float *wheel_rad_s, ChassisSpeed_Typedef *speed);
float Theta_Normalize(float theta);

#endif