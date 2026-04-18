/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_key.h"
#include "pid.h"
#include "mecanum.h"
#include "odom.h"
#include "trajectory.h"
#include <stdio.h>
#include <string.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void UART_SendSpeed(void)
{
    char tx_buf[128];  // 发送缓冲区，大小根据实际数据调整
    int len;            // 格式化后的字符串长度

// // 【临时测试】直接打印4个定时器的CNT寄存器值（不经过Encoder_GetCnt）
//     uint16_t tim2_cnt = __HAL_TIM_GET_COUNTER(&htim2); // LF
//     uint16_t tim3_cnt = __HAL_TIM_GET_COUNTER(&htim3); // RF
//     uint16_t tim4_cnt = __HAL_TIM_GET_COUNTER(&htim4); // LB
//     uint16_t tim5_cnt = __HAL_TIM_GET_COUNTER(&htim5); // RB

//     len = sprintf(tx_buf, "TIM_CNT[2:%d, 3:%d, 4:%d, 5:%d]\r\n",
//                   tim2_cnt, tim3_cnt, tim4_cnt, tim5_cnt);
//     HAL_UART_Transmit(&huart6, (uint8_t *)tx_buf, len, 10);
    // 1. 获取4个轮子的转速（RPM）
    float wheel_rpm[4];
    wheel_rpm[WHEEL_LF] = Encoder_GetRPM(WHEEL_LF);
    wheel_rpm[WHEEL_RF] = Encoder_GetRPM(WHEEL_RF);
    wheel_rpm[WHEEL_LB] = Encoder_GetRPM(WHEEL_LB);
    wheel_rpm[WHEEL_RB] = Encoder_GetRPM(WHEEL_RB);

    // 2. 获取底盘当前速度（vx, vy, omega）
    float wheel_rad_s[4];
    ChassisSpeed_Typedef chassis_speed;
    Encoder_GetRadS(wheel_rad_s);
    Mecanum_PositiveCalc(wheel_rad_s, &chassis_speed);

    // 3. 用sprintf将数据格式化为字符串（不使用printf）
    // 格式说明：%.1f保留1位小数，%.2f保留2位小数
    len = sprintf(tx_buf, 
                  "WHEEL[LF:%.1f, RF:%.1f, LB:%.1f, RB:%.1f] | CHASSIS[vx:%.2f, vy:%.2f, omega:%.2f]\r\n",
                  wheel_rpm[WHEEL_LF], wheel_rpm[WHEEL_RF], wheel_rpm[WHEEL_LB], wheel_rpm[WHEEL_RB],
                  chassis_speed.vx, chassis_speed.vy, chassis_speed.omega);

    // 4. 直接调用HAL库底层函数发送字符串
    // 参数说明：串口句柄、数据指针、数据长度、超时时间(ms)
    HAL_UART_Transmit(&huart6, (uint8_t *)tx_buf, len, 10);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUF_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t last_tick = 0; // 10ms周期计时
Odom_Typedef *odom_info; // 里程计信息
uint8_t uart_buf[UART_BUF_SIZE]; // 新增：串口发送缓冲区
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    Encoder_OverflowHandler(htim);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
 // ==================== 模块初始化 ====================
  Motor_Init();         // 电机初始化
  Encoder_Init();       // 编码器初始化
  Key_Init();           // 按键初始化
 
  Odom_Init();          // 里程计初始化
  Trajectory_Init();    // 轨迹控制初始化

  // 获取里程计信息指针
  odom_info = Odom_GetInfo();


  HAL_UART_Transmit(&huart6, (uint8_t *)"Init Success!\r\n", 16, 0xFFFF);


  // 开机显示
  // Oled_ShowString(0, 0, "Mecanum Chassis");
  // Oled_ShowString(0, 2, "Init Success!");
  // HAL_Delay(1000);
  // Oled_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {// ==================== 10ms固定周期控制（核心）====================
    if(HAL_GetTick() - last_tick >= 10)
    {
        last_tick = HAL_GetTick();

   
        UART_SendSpeed();
        // 轨迹控制主循环（核心，实现定点直达）
        Trajectory_ControlLoop(); 
           // ==================== 串口打印调试信息 ====================
  //  sprintf((char*)uart_buf, "X:%.3f Y:%.3f Theta:%.2f State:%d\r\n",
  //          odom_info->x, odom_info->y, odom_info->theta, Trajectory_GetState());
  //   HAL_UART_Transmit(&huart6, uart_buf, strlen((char*)uart_buf), 100);
    }

    // ==================== 按键处理 ====================
    uint8_t key = Key_Scan();
    switch(key)
    {
        case KEY_START:
            // 启动：设置目标坐标(2m, 0m)，可修改为任意坐标
            Trajectory_SetTarget(1.0f, 0.0f);
            sprintf((char*)uart_buf, "Start! Target: (%.2fm, %.2fm)\r\n", 0.5f, 0.0f);  
            HAL_UART_Transmit(&huart6, uart_buf, strlen((char*)uart_buf), 100);
            break;

        case KEY_RESET:
            // 原点复位
            Trajectory_ResetOrigin();
            HAL_UART_Transmit(&huart6, (uint8_t*)"Origin Reset!\r\n", 14, 100);
            break;

        case KEY_STOP:
            // 紧急停止
            Trajectory_Stop();
            HAL_UART_Transmit(&huart6, (uint8_t*)"Stop!\r\n", 6, 100);
            break;
    }

    

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
