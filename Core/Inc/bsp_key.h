#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "stm32f4xx_hal.h"

#define KEY_START   1   // 启动按键
#define KEY_RESET   2   // 原点复位按键
#define KEY_STOP    3   // 停止按键
#define KEY_NONE    0   // 无按键

void Key_Init(void);
uint8_t Key_Scan(void);

#endif