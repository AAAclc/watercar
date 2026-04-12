#include "odom.h"
#include "bsp_encoder.h"
#include "math.h"

// 里程计全局结构体
static Odom_Typedef odom;

/**
 * @brief  里程计初始化
 */
void Odom_Init(void)
{
    Odom_Reset();
}

/**
 * @brief  里程计复位（重置原点为当前位置）
 */
void Odom_Reset(void)
{
    odom.x = 0.0f;
    odom.y = 0.0f;
    odom.theta = 0.0f;
    odom.speed.vx = 0.0f;
    odom.speed.vy = 0.0f;
    odom.speed.omega = 0.0f;
}

/**
 * @brief  里程计更新（必须10ms固定周期调用）
 */
void Odom_Update(void)
{
    float wheel_rad_s[4];
    // 1. 获取轮子角速度
    Encoder_GetRadS(wheel_rad_s);
    // 2. 正运动学解算本体速度
    Mecanum_PositiveCalc(wheel_rad_s, &odom.speed);
    // 3. 旋转矩阵：本体速度转世界坐标系速度
    float vx_world = odom.speed.vx * cosf(odom.theta) - odom.speed.vy * sinf(odom.theta);
    float vy_world = odom.speed.vx * sinf(odom.theta) + odom.speed.vy * cosf(odom.theta);
    // 4. 积分更新位姿
    odom.x += vx_world * CONTROL_DT;
    odom.y += vy_world * CONTROL_DT;
    odom.theta += odom.speed.omega * CONTROL_DT;
    // 5. 航向角归一化
    odom.theta = Theta_Normalize(odom.theta);
}

/**
 * @brief  获取里程计信息
 * @retval 里程计结构体指针
 */
Odom_Typedef* Odom_GetInfo(void)
{
    return &odom;
}
