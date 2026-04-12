#ifndef __ODOM_H
#define __ODOM_H

#include "stm32f4xx_hal.h"
#include "mecanum.h"

// 里程计位姿结构体
typedef struct
{
    float x;         // 世界坐标系x坐标，单位m
    float y;         // 世界坐标系y坐标，单位m
    float theta;     // 航向角，单位rad
    ChassisSpeed_Typedef speed; // 当前底盘速度
}Odom_Typedef;

// 对外接口声明
void Odom_Init(void);
void Odom_Update(void);
void Odom_Reset(void);
Odom_Typedef* Odom_GetInfo(void);

#endif