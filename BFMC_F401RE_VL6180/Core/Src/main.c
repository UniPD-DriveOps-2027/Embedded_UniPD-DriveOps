/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VL6180X_ADDR 0x52
#define VL6180X_ID 0xB4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL6180X_IDENTIFICATION_MODEL_ID   0x0000
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET 0x0016
#define VL6180X_SYSRANGE_START            0x0018
#define VL6180X_RESULT_RANGE_STATUS       0x004D
#define VL6180X_RESULT_RANGE_VAL          0x0062
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t distance = 0;
char uart_buf[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t VL6180X_WriteReg(uint16_t reg, uint8_t value);
uint8_t VL6180X_ReadReg(uint16_t reg);
uint8_t VL6180X_Init(void);
uint8_t VL6180X_ReadDistance(uint8_t *distance);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* VL6180X Register Access Functions */
uint8_t VL6180X_WriteReg(uint16_t reg, uint8_t value) {
	uint8_t data[3] = {(reg >> 8) & 0xFF, reg & 0xFF, value};
	return HAL_I2C_Master_Transmit(&hi2c1, VL6180X_ADDR, data, 3, 100);
}

uint8_t VL6180X_ReadReg(uint16_t reg) {
	uint8_t data[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
	HAL_I2C_Master_Transmit(&hi2c1, VL6180X_ADDR, data, 2, 100);
	HAL_I2C_Master_Receive(&hi2c1, VL6180X_ADDR, data, 1, 100);
	return data[0];
}

/* Sensor Initialization */
uint8_t VL6180X_Init(void) {
	// Check device ID
	if(VL6180X_ReadReg(VL6180X_IDENTIFICATION_MODEL_ID) != VL6180X_ID) {
		return 1; // Device not found
	}

	// Mandatory settings from ST datasheet
	VL6180X_WriteReg(0x0207, 0x01);
	VL6180X_WriteReg(0x0208, 0x01);
	VL6180X_WriteReg(0x0096, 0x00);
	VL6180X_WriteReg(0x0097, 0xFD);
	VL6180X_WriteReg(0x00E3, 0x00);
	VL6180X_WriteReg(0x00E4, 0x04);
	VL6180X_WriteReg(0x00E5, 0x02);
	VL6180X_WriteReg(0x00E6, 0x01);
	VL6180X_WriteReg(0x00E7, 0x03);
	VL6180X_WriteReg(0x00F5, 0x02);
	VL6180X_WriteReg(0x00D9, 0x05);
	VL6180X_WriteReg(0x00DB, 0xCE);
	VL6180X_WriteReg(0x00DC, 0x03);
	VL6180X_WriteReg(0x00DD, 0xF8);
	VL6180X_WriteReg(0x009F, 0x00);
	VL6180X_WriteReg(0x00A3, 0x3C);
	VL6180X_WriteReg(0x00B7, 0x00);
	VL6180X_WriteReg(0x00BB, 0x3C);
	VL6180X_WriteReg(0x00B2, 0x09);
	VL6180X_WriteReg(0x00CA, 0x09);
	VL6180X_WriteReg(0x0198, 0x01);
	VL6180X_WriteReg(0x01B0, 0x17);
	VL6180X_WriteReg(0x01AD, 0x00);
	VL6180X_WriteReg(0x00FF, 0x05);
	VL6180X_WriteReg(0x0100, 0x05);
	VL6180X_WriteReg(0x0199, 0x05);
	VL6180X_WriteReg(0x01A6, 0x1B);
	VL6180X_WriteReg(0x01AC, 0x3E);
	VL6180X_WriteReg(0x01A7, 0x1F);
	VL6180X_WriteReg(0x0030, 0x00);

	// Set scaling to 1x
	VL6180X_WriteReg(0x0011, 0x10);

	// Prepare for 2.8V operation
	VL6180X_WriteReg(0x012A, 0x02);

	// Set averaging period
	VL6180X_WriteReg(0x003A, 0x03);

	// Set light/dark gain
	VL6180X_WriteReg(0x0031, 0xFF);

	// Set interrupt config
	VL6180X_WriteReg(0x0014, 0x24);

	return 0;
}

/* Distance Measurement */
uint8_t VL6180X_ReadDistance(uint8_t *distance) {
	VL6180X_WriteReg(VL6180X_SYSRANGE_START, 0x01);
	while((VL6180X_ReadReg(VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0);
	*distance = VL6180X_ReadReg(VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	uint8_t sensor_status = 0;
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
	MX_USART2_UART_Init();
	MX_TIM4_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	// Sensor initialization
	sensor_status = VL6180X_Init();
	if(sensor_status != 0) {
		printf("VL6180X Init Failed!\r\n");
		Error_Handler();
	}

	printf("VL6180X Ready!\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if(VL6180X_ReadDistance(&distance) == 0) {
			printf("Distance: %d mm\r\n", distance);
		}
		HAL_Delay(100);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
