#include "bsp_motor.h"
#include "mecanum.h"
// 复用CubeMX生成的定时器句柄
extern TIM_HandleTypeDef htim1;

/**
 * @brief  电机初始化（启动PWM输出）
 */
void Motor_Init(void)
{
    // 启动TIM1的4路PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    // 初始PWM=0，电机停止
    Motor_StopAll();
}

/**
 * @brief  设置单个电机的PWM（正反转+调速）
 * @param  wheel_id: 轮子编号
 * @param  pwm: PWM值，正=正转，负=反转，范围PWM_MIN~PWM_MAX
 */
void Motor_SetPWM(uint8_t wheel_id, int32_t pwm)
{
    // PWM限幅
    if(pwm > PWM_MAX) pwm = PWM_MAX;
    if(pwm < PWM_MIN) pwm = PWM_MIN;

    switch(wheel_id)
    {
        case WHEEL_LF: // 左前电机
            if(pwm >= 0)
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);   // IN1=1
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // IN2=0
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // IN1=0
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);   // IN2=1
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm);
            }
            break;

        case WHEEL_RF: // 右前电机
            if(pwm >= 0)
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);   // IN1=1
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // IN2=0
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); // IN1=0
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // IN2=1
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pwm);
            }
            break;

        case WHEEL_LB: // 左后电机
            if(pwm >= 0)
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN1=1
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); // IN2=0
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); // IN1=0
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);   // IN2=1
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pwm);
            }
            break;

        case WHEEL_RB: // 右后电机
            if(pwm >= 0)
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN1=1
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); // IN2=0
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); // IN1=0
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);   // IN2=1
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -pwm);
            }
            break;
    }
}

/**
 * @brief  停止所有电机
 */
void Motor_StopAll(void)
{
    for(uint8_t i=0; i<4; i++) Motor_SetPWM(i, 0);
}