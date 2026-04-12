#include "bsp_key.h"

void Key_Init(void)
{
    // 按键GPIO已由CubeMX初始化，这里无需重复操作
}

uint8_t Key_Scan(void)
{
    // 按键按下为低电平，消抖处理
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET);
            return KEY_START;
        }
    }
    else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET)
        {
            while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);
            return KEY_RESET;
        }
    }
    return KEY_NONE;
}