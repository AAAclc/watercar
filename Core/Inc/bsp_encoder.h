#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "stm32f4xx_hal.h"
#include "mecanum.h"

// 对外接口声明
void Encoder_Init(void);
void Encoder_OverflowHandler(TIM_HandleTypeDef *htim);
void Encoder_UpdateAll(void);          // 每周期调用一次，采集所有编码器
int32_t Encoder_GetCnt(uint8_t wheel_id);
float Encoder_GetRPM(uint8_t wheel_id);  // 返回 Encoder_UpdateAll 缓存的值
void Encoder_GetRadS(float *wheel_rad_s); // 返回 Encoder_UpdateAll 缓存的值

#endif