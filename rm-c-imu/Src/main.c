/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "bsp_delay.h"
#include "ist8310driver.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// BMI088传感器数据变量
float gyro[3] = {0};      // 陀螺仪数据 [x, y, z] (rad/s)
float accel[3] = {0};     // 加速度计数据 [x, y, z] (m/s²)
float temperature = 0;    // 温度数据 (°C)

// IST8310磁力计数据变量
float mag[3] = {0};       // 磁力计数据 [x, y, z] (uT)

uint32_t last_print_time = 0;  // 上次打印时间
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  // 初始化DWT延时系统
  DWT_Delay_Init();
  
  // 初始化SEGGER RTT
  SEGGER_RTT_Init();
  SEGGER_RTT_printf(0, "\n=== BMI088 + IST8310传感器数据读取系统启动 ===\n");
  
  // 初始化BMI088传感器
  SEGGER_RTT_printf(0, "正在初始化BMI088传感器...\n");
  uint8_t bmi088_init_result = BMI088_init();
  if (bmi088_init_result == BMI088_NO_ERROR) {
    SEGGER_RTT_printf(0, "BMI088初始化成功!\n");
    SEGGER_RTT_printf(0, "- 加速度计量程: ±3g\n");
    SEGGER_RTT_printf(0, "- 陀螺仪量程: ±2000°/s\n");
  } else {
    SEGGER_RTT_printf(0, "BMI088初始化失败! 错误代码: 0x%02X\n", bmi088_init_result);
  }
  
  // 初始化IST8310磁力计
  SEGGER_RTT_printf(0, "正在初始化IST8310磁力计...\n");
  uint8_t ist8310_init_result = ist8310_init();
  if (ist8310_init_result == IST8310_NO_ERROR) {
    SEGGER_RTT_printf(0, "IST8310初始化成功!\n");
    SEGGER_RTT_printf(0, "- 磁力计量程: ±1600uT\n");
    SEGGER_RTT_printf(0, "- 输出频率: 200Hz\n");
  } else {
    SEGGER_RTT_printf(0, "IST8310初始化失败! 错误代码: 0x%02X\n", ist8310_init_result);
  }
  
  // 等待传感器稳定
  HAL_Delay(100);
  
  last_print_time = HAL_GetTick();
  
  SEGGER_RTT_printf(0, "\n=== 开始传感器数据读取 ===\n");
  SEGGER_RTT_printf(0, "数据格式: 加速度(m/s²) | 陀螺仪(°/s) | 磁力计(uT) | 温度(°C)\n\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SEGGER_RTT_printf(0, "Start");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 读取BMI088传感器数据
    BMI088_read(gyro, accel, &temperature);
    
    // 读取IST8310磁力计数据
    ist8310_read_mag(mag);
    
    // 通过SEGGER RTT输出9轴传感器数据
    // SEGGER_RTT_printf(0, "IMU Data - Accel: X:%.3f Y:%.3f Z:%.3f, Gyro: X:%.3f Y:%.3f Z:%.3f, Mag: X:%.3f Y:%.3f Z:%.3f, Temp:%.1f\r\n",
    //                   accel[0], accel[1], accel[2], 
    //                   gyro[0], gyro[1], gyro[2],
    //                   mag[0], mag[1], mag[2], temperature);
    
    // 小延时，避免CPU占用过高
    HAL_Delay(10);
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
