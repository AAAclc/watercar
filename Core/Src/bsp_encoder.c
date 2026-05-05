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
// 缓存本周期采集到的转速（只由 Encoder_UpdateAll 写入）
static float encoder_rpm_cache[4] = {0};
static float encoder_rads_cache[4] = {0};

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
 * @brief  采集所有编码器（每个控制周期调用一次）
 */
void Encoder_UpdateAll(void)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        int32_t cnt_now = Encoder_GetCnt(i);
        int32_t cnt_diff = cnt_now - encoder_last[i];
        encoder_last[i] = cnt_now;
        encoder_rpm_cache[i] = (cnt_diff * 60.0f) / (PPR * CONTROL_DT);
        encoder_rads_cache[i] = (cnt_diff * 2 * PI) / (PPR * CONTROL_DT);
    }
}

/**
 * @brief  获取轮子转速rpm（返回 Encoder_UpdateAll 缓存的值）
 * @param  wheel_id: 轮子编号
 * @retval 转速，单位rpm
 */
float Encoder_GetRPM(uint8_t wheel_id)
{
    return encoder_rpm_cache[wheel_id];
}

/**
 * @brief  获取轮子角速度rad/s（返回 Encoder_UpdateAll 缓存的值）
 * @param  wheel_rad_s: 输出4个轮子的角速度数组
 */
void Encoder_GetRadS(float *wheel_rad_s)
{
    for(uint8_t i = 0; i < 4; i++)
        wheel_rad_s[i] = encoder_rads_cache[i];
}