#include "mecanum.h"
#ifndef PI
#define PI 3.1415926f
#endif
/**
 * @brief  麦轮逆运动学解算（输入底盘速度，输出轮子目标转速rpm）
 * @param  speed: 底盘速度结构体指针
 * @param  wheel_rpm: 输出4个轮子的转速数组，单位rpm（转/分钟）
 */
void Mecanum_InverseCalc(ChassisSpeed_Typedef *speed, float *wheel_rpm)
{
    // 先计算轮子角速度 rad/s
    float w_lf = (speed->vx - speed->vy - (MECANUM_L + MECANUM_W)*speed->omega) / MECANUM_R;
    float w_rf = (speed->vx + speed->vy + (MECANUM_L + MECANUM_W)*speed->omega) / MECANUM_R;
    float w_lb = (speed->vx + speed->vy - (MECANUM_L + MECANUM_W)*speed->omega) / MECANUM_R;
    float w_rb = (speed->vx - speed->vy + (MECANUM_L + MECANUM_W)*speed->omega) / MECANUM_R;
    
    // rad/s 转 rpm
    wheel_rpm[WHEEL_LF] = w_lf * 60.0f / (2*PI);
    wheel_rpm[WHEEL_RF] = w_rf * 60.0f / (2*PI);
    wheel_rpm[WHEEL_LB] = w_lb * 60.0f / (2*PI);
    wheel_rpm[WHEEL_RB] = w_rb * 60.0f / (2*PI);
}

/**
 * @brief  麦轮正运动学解算（输入轮子角速度，输出底盘当前速度）
 * @param  wheel_rad_s: 4个轮子的角速度数组，单位rad/s
 * @param  speed: 输出底盘速度结构体指针
 */
void Mecanum_PositiveCalc(float *wheel_rad_s, ChassisSpeed_Typedef *speed)
{
    speed->vx = MECANUM_R * (wheel_rad_s[WHEEL_LF] + wheel_rad_s[WHEEL_RF] + wheel_rad_s[WHEEL_LB] + wheel_rad_s[WHEEL_RB]) / 4.0f;
    speed->vy = MECANUM_R * (-wheel_rad_s[WHEEL_LF] + wheel_rad_s[WHEEL_RF] + wheel_rad_s[WHEEL_LB] - wheel_rad_s[WHEEL_RB]) / 4.0f;
    speed->omega = MECANUM_R * (-wheel_rad_s[WHEEL_LF] + wheel_rad_s[WHEEL_RF] - wheel_rad_s[WHEEL_LB] + wheel_rad_s[WHEEL_RB]) / (4.0f * (MECANUM_L + MECANUM_W));
}

/**
 * @brief  航向角归一化，限制在[-π, π]
 * @param  theta: 原始航向角，单位rad
 * @retval 归一化后的航向角
 */
float Theta_Normalize(float theta)
{
    while(theta > PI) theta -= 2*PI;
    while(theta < -PI) theta += 2*PI;
    return theta;
}