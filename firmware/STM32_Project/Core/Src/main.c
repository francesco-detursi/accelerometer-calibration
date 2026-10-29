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
#include "memorymap.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scl3300.h"
#include "tinyml_online.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT_MS 100U
#define SENSOR_STARTUP_DELAY_MS 100U
#define MEASUREMENT_PERIOD_MS 20U
#define UART_BUFFER_SIZE 256U
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static SCL3300_Handle_t scl3300_sensor;
static SCL3300_Diagnostics_t scl3300_diagnostics;
static SCL3300_Measurement_t scl3300_measurement;
static TinyML_OnlineResult_t tinyml_result;
static volatile stai_return_code tinyml_status;
static char uart_buffer[UART_BUFFER_SIZE];
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void UART_SendString(const char *text);
static void UART_PrintRawMeasurement(const SCL3300_Measurement_t *measurement);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_SendString(const char *text)
{
  size_t text_length;
  if (text == NULL)
  {
    return;
  }
  text_length = strlen(text);
  if (text_length == 0U)
  {
    return;
  }
  if (text_length > UINT16_MAX)
  {
    text_length = UINT16_MAX;
  }
  (void)HAL_UART_Transmit(
      &huart3,
      (uint8_t *)text,
      (uint16_t)text_length,
      UART_TIMEOUT_MS);
}
static void UART_PrintRawMeasurement(const SCL3300_Measurement_t *measurement)
{
  int length;
  if (measurement == NULL)
  {
    return;
  }
  length = snprintf(
      uart_buffer,
      sizeof(uart_buffer),
      "%d,%d,%d,"
      "%.9f,%.9f,%.9f,"
      "%.9f,%.9f,%.9f,"
      "%.9f,%.9f\r\n",
      (int)measurement->acc_x_raw,
      (int)measurement->acc_y_raw,
      (int)measurement->acc_z_raw,
      tinyml_result.reference[0],
      tinyml_result.reference[1],
      tinyml_result.reference[2],
      tinyml_result.neural[0],
      tinyml_result.neural[1],
      tinyml_result.neural[2],
      tinyml_result.max_abs_error,
      tinyml_result.mean_abs_error);
  if (length <= 0)
  {
    return;
  }
  if ((size_t)length >= sizeof(uart_buffer))
  {
    length = (int)(sizeof(uart_buffer) - 1U);
  }
  (void)HAL_UART_Transmit(
      &huart3,
      (uint8_t *)uart_buffer,
      (uint16_t)length,
      UART_TIMEOUT_MS);
}
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  SCL3300_Status_t driver_status;
  /* USER CODE END 1 */
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(
      SCL3300_CS_GPIO_Port,
      SCL3300_CS_Pin,
      GPIO_PIN_SET);
  HAL_Delay(SENSOR_STARTUP_DELAY_MS);
  driver_status = SCL3300_InitHandle(
      &scl3300_sensor,
      &hspi1,
      SCL3300_CS_GPIO_Port,
      SCL3300_CS_Pin);
  if (driver_status != SCL3300_OK)
  {
    UART_SendString("SCL3300_INIT_ERROR\r\n");
    while (1)
    {
      HAL_Delay(1000U);
    }
  }
  driver_status = SCL3300_Start(
      &scl3300_sensor,
      SCL3300_MODE_1,
      &scl3300_diagnostics);
  if (driver_status != SCL3300_OK)
  {
    UART_SendString("SCL3300_START_ERROR\r\n");
    while (1)
    {
      HAL_Delay(1000U);
    }
  }
  tinyml_status = TinyML_Online_Init();
  if (tinyml_status != STAI_SUCCESS)
  {
    UART_SendString("TINYML_INIT_ERROR\r\n");
    while (1)
    {
      HAL_Delay(1000U);
    }
  }
  UART_SendString(
      "raw_x,raw_y,raw_z,"
      "ref_x,ref_y,ref_z,"
      "nn_x,nn_y,nn_z,"
      "max_abs_error,mean_abs_error\r\n");
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    driver_status = SCL3300_ReadMeasurement(
        &scl3300_sensor,
        &scl3300_measurement);
    if (driver_status == SCL3300_OK)
    {
      tinyml_status = TinyML_Online_Run(
          scl3300_measurement.acc_x_raw,
          scl3300_measurement.acc_y_raw,
          scl3300_measurement.acc_z_raw,
          &tinyml_result);
      if (tinyml_status == STAI_SUCCESS)
      {
        UART_PrintRawMeasurement(&scl3300_measurement);
      }
    }
    HAL_Delay(MEASUREMENT_PERIOD_MS);
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
  /* Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /* Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }
  /* Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2 |
      RCC_CLOCKTYPE_D3PCLK1 |
      RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  /* Initializes the peripherals clock */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
  * @brief MPU Configuration
  * @retval None
  */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  /* Disables the MPU */
  HAL_MPU_Disable();
  /* Initializes and configures the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
