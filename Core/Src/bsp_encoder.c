#include "bsp_encoder.h"
// ==================== 新增：补充PI定义，解决报错 ====================
#ifndef PI
#define PI 3.1415926f
#endif
// 复用CubeMX生成的定时器句柄（必须和CubeMX配置一致）
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// 编码器32位总计数（处理16位定时器溢出）
static int32_t encoder_total[4] = {0};
// 上一次的编码器计数（用于测速）
static int32_t encoder_last[4] = {0};

/**
 * @brief  编码器初始化（启动编码器模式+更新中断）
 */
void Encoder_Init(void)
{
    // 启动编码器定时器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    // 启动定时器更新中断（处理溢出）
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    // 初始化计数
    for(uint8_t i=0; i<4; i++) encoder_last[i] = Encoder_GetCnt(i);
}

/**
 * @brief  编码器溢出处理（在定时器更新中断回调中调用）
 * @param  htim: 定时器句柄
 */
void Encoder_OverflowHandler(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) // 左前
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) encoder_total[WHEEL_LF] -= 65536;
        else encoder_total[WHEEL_LF] += 65536;
    }
    else if(htim->Instance == TIM3) // 右前
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) encoder_total[WHEEL_RF] -= 65536;
        else encoder_total[WHEEL_RF] += 65536;
    }
    else if(htim->Instance == TIM4) // 左后
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) encoder_total[WHEEL_LB] -= 65536;
        else encoder_total[WHEEL_LB] += 65536;
    }
    else if(htim->Instance == TIM5) // 右后
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) encoder_total[WHEEL_RB] -= 65536;
        else encoder_total[WHEEL_RB] += 65536;
    }
}

/**
 * @brief  获取编码器总计数
 * @param  wheel_id: 轮子编号
 * @retval 32位总计数
 */
// int32_t Encoder_GetCnt(uint8_t wheel_id)
// {
//     switch(wheel_id)
//     {
//         case WHEEL_LF: return encoder_total[WHEEL_LF] + __HAL_TIM_GET_COUNTER(&htim2);
//         case WHEEL_RF: return encoder_total[WHEEL_RF] + __HAL_TIM_GET_COUNTER(&htim3);
//         case WHEEL_LB: return encoder_total[WHEEL_LB] + __HAL_TIM_GET_COUNTER(&htim4);
//         case WHEEL_RB: return encoder_total[WHEEL_RB] + __HAL_TIM_GET_COUNTER(&htim5);
//         default: return 0;
//     }
// }
int32_t Encoder_GetCnt(uint8_t wheel_id)
{
    int32_t cnt = 0;
    switch(wheel_id)
    {
        case WHEEL_LF: 
            cnt = encoder_total[WHEEL_LF] + __HAL_TIM_GET_COUNTER(&htim2);
            return cnt; 
        case WHEEL_RF: 
            cnt = encoder_total[WHEEL_RF] + __HAL_TIM_GET_COUNTER(&htim3);
            return cnt; 
        case WHEEL_LB: 
            return encoder_total[WHEEL_LB] + __HAL_TIM_GET_COUNTER(&htim4);
        case WHEEL_RB: 
            return encoder_total[WHEEL_RB] + __HAL_TIM_GET_COUNTER(&htim5);
        default: return 0;
    }
}
/**
 * @brief  获取轮子转速rpm
 * @param  wheel_id: 轮子编号
 * @retval 转速，单位rpm
 */
float Encoder_GetRPM(uint8_t wheel_id)
{
    int32_t cnt_now = Encoder_GetCnt(wheel_id);
    int32_t cnt_diff = cnt_now - encoder_last[wheel_id];
    encoder_last[wheel_id] = cnt_now;
    // 计算rpm：(脉冲差/单圈总脉冲数) * 60s / 控制周期
    return (cnt_diff * 60.0f) / (PPR * CONTROL_DT);
}

/**
 * @brief  获取轮子角速度rad/s
 * @param  wheel_rad_s: 输出4个轮子的角速度数组
 */
void Encoder_GetRadS(float *wheel_rad_s)
{
    int32_t cnt_now[4], cnt_diff[4];
    for(uint8_t i=0; i<4; i++)
    {
        cnt_now[i] = Encoder_GetCnt(i);
        cnt_diff[i] = cnt_now[i] - encoder_last[i];
        encoder_last[i] = cnt_now[i];
        // 计算rad/s：(脉冲差/单圈总脉冲数) * 2π / 控制周期
        wheel_rad_s[i] = (cnt_diff[i] * 2 * PI) / (PPR * CONTROL_DT);
    }
}