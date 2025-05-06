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
#include <errno.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VL6180X_ADDR 0x52
// Default address (7-bit)
#define VL6180X_SENSOR1_ADDR 0x29
#define VL6180X_SENSOR2_ADDR 0x30
#define VL6180X_ID 0xB4

/* Parameter Linear Regression */
#define LINEAR_CORRECTION_A_1 1.00590103  //1.01787916
#define LINEAR_CORRECTION_B_1 15.12839818 //15.85141800
#define LINEAR_CORRECTION_A_2 1.020000
#define LINEAR_CORRECTION_B_2 0.300000

// Register definitions (essential ones for address change)
#define VL6180X_REG_I2C_SLAVE_DEV_ADDR 0x0212
//#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET 0x0016

// Define your XSHUT pin (replace with your actual pin definition)
#define TOF_XSHUT_PIN GPIO_PIN_9
#define TOF_XSHUT_PORT GPIOC
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
uint8_t VL6180X_WriteReg(uint8_t sensor_addr, uint16_t reg, uint8_t value);
uint8_t VL6180X_ReadReg(uint8_t sensor_addr, uint16_t reg);
uint8_t VL6180X_Init(uint8_t sensor_addr);
uint8_t VL6180X_ReadDistance(uint8_t sensor_addr, uint8_t *distance);
uint8_t i2c_Initialization();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* VL6180X Register Access Functions */

uint8_t VL6180X_WriteReg(uint8_t sensor_addr, uint16_t reg, uint8_t value) {
	uint8_t data[3] = {(reg >> 8) & 0xFF, reg & 0xFF, value};
	return HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, data, 3, 100);
}

uint8_t VL6180X_ReadReg(uint8_t sensor_addr, uint16_t reg) {
	uint8_t data[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
	HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, data, 2, 100);
	HAL_I2C_Master_Receive(&hi2c1, (sensor_addr << 1) | 0x01, data, 1, 100);
	return data[0];
}

uint8_t VL6180X_Init(uint8_t sensor_addr) {
	// Check device ID
	if(VL6180X_ReadReg(sensor_addr, VL6180X_IDENTIFICATION_MODEL_ID) != VL6180X_ID) {
		return 1; // Device not found
	}

	// Mandatory settings from ST datasheet
	VL6180X_WriteReg(sensor_addr, 0x0207, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x0208, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x0096, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x0097, 0xFD);
	VL6180X_WriteReg(sensor_addr, 0x00E3, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00E4, 0x04);
	VL6180X_WriteReg(sensor_addr, 0x00E5, 0x02);
	VL6180X_WriteReg(sensor_addr, 0x00E6, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x00E7, 0x03);
	VL6180X_WriteReg(sensor_addr, 0x00F5, 0x02);
	VL6180X_WriteReg(sensor_addr, 0x00D9, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x00DB, 0xCE);
	VL6180X_WriteReg(sensor_addr, 0x00DC, 0x03);
	VL6180X_WriteReg(sensor_addr, 0x00DD, 0xF8);
	VL6180X_WriteReg(sensor_addr, 0x009F, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00A3, 0x3C);
	VL6180X_WriteReg(sensor_addr, 0x00B7, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00BB, 0x3C);
	VL6180X_WriteReg(sensor_addr, 0x00B2, 0x09);
	VL6180X_WriteReg(sensor_addr, 0x00CA, 0x09);
	VL6180X_WriteReg(sensor_addr, 0x0198, 0x01);
	VL6180X_WriteReg(sensor_addr, 0x01B0, 0x17);
	VL6180X_WriteReg(sensor_addr, 0x01AD, 0x00);
	VL6180X_WriteReg(sensor_addr, 0x00FF, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x0100, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x0199, 0x05);
	VL6180X_WriteReg(sensor_addr, 0x01A6, 0x1B);
	VL6180X_WriteReg(sensor_addr, 0x01AC, 0x3E);
	VL6180X_WriteReg(sensor_addr, 0x01A7, 0x1F);
	VL6180X_WriteReg(sensor_addr, 0x0030, 0x00);

	// Set scaling to 1x
	VL6180X_WriteReg(sensor_addr, 0x0011, 0x10);

	// Prepare for 2.8V operation
	VL6180X_WriteReg(sensor_addr, 0x012A, 0x02);

	// Set averaging period
	VL6180X_WriteReg(sensor_addr, 0x003A, 0x03);

	// Set light/dark gain
	VL6180X_WriteReg(sensor_addr, 0x0031, 0xFF);

	// Set interrupt config
	VL6180X_WriteReg(sensor_addr, 0x0014, 0x24);

	return 0;
}

/*
uint8_t VL6180X_ReadDistance(uint8_t sensor_addr, uint8_t *distance) {
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSRANGE_START, 0x01);
	while ((VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0);
 *distance = VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}

uint8_t VL6180X_ReadDistance(uint8_t sensor_addr, uint8_t *distance) {
	uint32_t timeout = 10000;
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSRANGE_START, 0x01);

	// Wait for measurement to complete or timeout
	while (((VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0) && (--timeout > 0));

	if (timeout == 0) {
		printf("[ReadDiscance] Timeout occurred, sensor didn't respond\r\n");
		return 1;
	}

 *distance = VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}
 */
uint8_t VL6180X_ReadDistance(uint8_t sensor_addr, uint8_t *distance) {
	uint32_t start_time = HAL_GetTick();  // Get start time in milliseconds
	uint32_t timeout_ms = 70;             // Timeout after x milliseconds

	VL6180X_WriteReg(sensor_addr, VL6180X_SYSRANGE_START, 0x01);

	// Wait for measurement to complete or timeout
	while ((VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0) {
		if ((HAL_GetTick() - start_time) > timeout_ms) {
			printf("[ReadDistance] Timeout occurred, sensor didn't respond\r\n");
			*distance = -1;
			return 1;
		}
	}

	*distance = VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}


uint8_t i2c_Initialization(){
	/* Sensor 1 has the shutdown signal and is off */

	/* Initialization sensor 2 */
	uint8_t found = 0;
	for (uint8_t addr = 1; addr < 127; addr++) {
		HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 10);

		/* Here it find only one device with address 0x29
		 *
		 * May happen is found one device with address 0x30
		 * in that case the sensor is not resetted
		 *
		 * Take care in case of sensor swapping one may
		 * keep the other address for a bit
		 */

		if (res == HAL_OK) {
			//printf("Device found at: 0x%02X\r\n", addr);
			found++;

			if (addr == VL6180X_SENSOR1_ADDR) {
				// Initialize sensor 2 using default address 0x29
				if(VL6180X_Init(VL6180X_SENSOR1_ADDR) != 0) {
					return 2;
				}

				// Change address of sensor 2 to 0x30
				HAL_Delay(50);
				VL6180X_WriteReg(VL6180X_SENSOR1_ADDR, VL6180X_REG_I2C_SLAVE_DEV_ADDR, VL6180X_SENSOR2_ADDR);
				HAL_Delay(50);
				break;
			}

			if (addr == VL6180X_SENSOR2_ADDR) {
				/* Here sensor 2 is already initialized
				 * just brake the for and keep going
				 */
				break;
			}
		}
	}
	if (found == 0) {
		return 3;
	}

	/* Initialization sensor 1 */

	// Turn on sensor 1
	HAL_Delay(50);
	HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_SET);
	HAL_Delay(50);

	// Initialize sensor 1 using default address 0x29
	if(VL6180X_Init(VL6180X_SENSOR1_ADDR) != 0) {
		return 1;
	}
	return 0;
}


uint8_t correct_sensor1(uint8_t raw_value) {
	uint8_t tmp = LINEAR_CORRECTION_A_1 * raw_value + LINEAR_CORRECTION_B_1;
    return tmp;
}
float correct_sensor2(uint8_t raw_value) {
    return LINEAR_CORRECTION_A_2 * raw_value + LINEAR_CORRECTION_B_2;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t tof_sensors = 1;	// flag to use TOF sensors
	uint8_t line_sensors5 = 0;	// flag to use 5 line sensors
	uint8_t line_sensor1 = 0;	// flag to use 1 line sensor

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

	/* TOF code */
	uint8_t distance1 = 0, distance2 = 0;
	if (tof_sensors == 1) {
		printf("Start initialization TOF sensors \r\n");

		switch(i2c_Initialization()) {
		case 0:
			printf("TOF sensors ready!\r\n");
			break;

		case 1:
			printf("Sensor 1 initialization failed!\r\n");
			break;

		case 2:
			printf("Sensor 2 initialization failed!\r\n");
			break;

		case 3:
			printf("Sensor 2 not connected!\r\n");
			break;

		default:
			printf("Unknown initialization error!\r\n");
			break;
		}
	}

	/* 5 line sensors code */
	uint8_t sensor5_1 = 0; // OUT1
	uint8_t sensor5_2 = 0; // OUT2
	uint8_t sensor5_3 = 0; // OUT3
	uint8_t sensor5_4 = 0; // OUT4
	uint8_t sensor5_5 = 0; // OUT5
	if (line_sensors5 == 1) {
		printf("Testing 5 sensors IR array module...\r\n");

	}

	/* 1 line sensor code */
	GPIO_PinState sensor1;
	if (line_sensor1 == 1) {
		printf("Testing 1 sensor IR...\r\n");
	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float i = 3;
	HAL_Delay(5000);
	while (1) {

		if (tof_sensors == 1) {
			if (VL6180X_ReadDistance(VL6180X_SENSOR1_ADDR, &distance1) == 0) {

				//printf("Sensor 1 measured distance: %d mm for real distance: %d\r\n", distance1, i*10);
				printf("Sensor 1 measured: %d mm, corrected: %d mm, real distance: %.1f mm\r\n", distance1, correct_sensor1(distance1), i*10);
				i = i + 0.5;
			}
			if (i == 20.5) {
				printf("Got to i = 20");
				Error_Handler();
			}
			/*
			if (VL6180X_ReadDistance(VL6180X_SENSOR2_ADDR, &distance2) == 0) {
				printf("Sensor 2 measured distance: %d mm\r\n", distance2);
			}
			*/
		}

		if (line_sensors5 == 1) {
			sensor5_1 = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2); // OUT1
			sensor5_2 = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1); // OUT2
			sensor5_3 = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15); // OUT3
			sensor5_4 = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); // OUT4
			sensor5_5 = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); // OUT5

			// print low when detect
			printf("Sensors: %d %d %d %d %d\r\n", sensor5_1, sensor5_2, sensor5_3, sensor5_4, sensor5_5);
		}

		if (line_sensor1 == 1) {
			sensor1 = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

			// print low when detect
			printf("Sensor: %d \r\n", sensor1);
		}


		HAL_Delay(200);
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
