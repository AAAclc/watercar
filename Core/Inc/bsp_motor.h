#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "stm32f4xx_hal.h"

// PWM输出限幅（对应ARR=4199，0-4199对应0-100%占空比）
#define PWM_MAX 4199
#define PWM_MIN -4199

// 对外接口声明
void Motor_Init(void);
void Motor_SetPWM(uint8_t wheel_id, int32_t pwm);
void Motor_StopAll(void);

#endif