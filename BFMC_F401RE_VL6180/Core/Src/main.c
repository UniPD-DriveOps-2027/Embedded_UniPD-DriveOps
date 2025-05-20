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

#include "ToF/vl6180x_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VL6180X_ADDR 0x52
// Default address (7-bit)
#define VL6180X_SENSOR1_ADDR 0x29
#define VL6180X_SENSOR2_ADDR 0x30
#define VL6180X_ID 0xB4

/* Parameter Linear Regression */
#define LINEAR_CORRECTION_A_1 0.97053299 //0.95185037 //1.00590103  //1.01787916
#define LINEAR_CORRECTION_B_1 19.55846396 //18.39083335 //15.12839818 //15.85141800
#define LINEAR_CORRECTION_A_2 0.96133608 //0.83103655 //1.12594560
#define LINEAR_CORRECTION_B_2 0.36225924 //11.71308245 //-18.18327961

// Register definitions (essential ones for address change)
#define VL6180X_REG_I2C_SLAVE_DEV_ADDR 0x0212
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
	uint32_t timeout_ms = 60;             // Timeout after x milliseconds

	VL6180X_WriteReg(sensor_addr, VL6180X_SYSRANGE_START, 0x01);

	// Wait for measurement to complete or timeout
	while ((VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0) {
		if ((HAL_GetTick() - start_time) > timeout_ms) {
			//printf("[ReadDistance] Timeout occurred, sensor didn't respond\r\n");
			return 1;
		}
	}

	*distance = VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}


uint8_t i2c_Initialization(){
	// Turn off sensor 1
	HAL_Delay(50);
	HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_Delay(50);

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
					printf("Sensor 2 initialization failed! \r\n");
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
				printf("TOF sensor 2 already initialized \r\n");
				break;
			}
		}
	}
	if (found == 0) {
		printf("Sensor 2 not connected! \r\n");
		return 3;
	}

	/* Initialization sensor 1 */

	// Turn on sensor 1
	HAL_Delay(50);
	HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_SET);
	HAL_Delay(50);

	// Initialize sensor 1 using default address 0x29
	if(VL6180X_Init(VL6180X_SENSOR1_ADDR) != 0) {
		printf("Sensor 1 initialization failed! \r\n");
		return 1;
	}
	printf("TOF sensors ready! \r\n");
	return 0;
}


// Calibration function for VL6180X
int linear_correction_1(uint8_t raw_value) {
	int tmp = LINEAR_CORRECTION_A_1 * raw_value + LINEAR_CORRECTION_B_1;
	return tmp;
}
int linear_correction_2(uint8_t raw_value) {
	int tmp = LINEAR_CORRECTION_A_2 * raw_value + LINEAR_CORRECTION_B_2;
	return tmp;
}

/* LINE SENSOR */
uint32_t readADC(uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	uint32_t adc_value = 0;

	// Configure the ADC channel
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	// Start conversion
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		return -1;  // Error starting ADC
	}

	// Wait for conversion
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		adc_value = HAL_ADC_GetValue(&hadc1);
	} else {
		adc_value = -1;  // Conversion timeout
	}

	HAL_ADC_Stop(&hadc1);

	return adc_value;
}

void readAllSensors(uint32_t channels[], uint32_t results[], uint8_t count)
{
	for(uint8_t i = 0; i < count; i++) {
		results[i] = readADC(channels[i]);
		if(results[i] == (uint32_t)-1) {
			printf("Error reading sensor %d\r\n", i+1);
		}
	}
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
		i2c_Initialization();
	}

	/* 5 line sensors code */
	// Define which ADC channels correspond to each sensor
	uint32_t sensorChannels[5] = {
			ADC_CHANNEL_11,  // OUT1 on PC1 (LINE_5_1)
			ADC_CHANNEL_12,  // OUT2 on PC2 (LINE_5_2)
			ADC_CHANNEL_13,  // OUT3 on PC3 (LINE_5_3)
			ADC_CHANNEL_14,  // OUT4 on PC4 (LINE_5_4)
			ADC_CHANNEL_15   // OUT5 on PC5 (LINE_5_5)
	};
	uint32_t sensorValues[5];
	if (line_sensors5 == 1) {
		printf("Testing 5 sensors IR array module... \r\n");

	}

	/* 1 line sensor code */
	uint32_t sensor1Channel = ADC_CHANNEL_10; // OUT on PC0 (LINE_1)
	uint32_t sensor1Value;
	if (line_sensor1 == 1) {
		printf("Testing 1 sensor IR... \r\n");
	}


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint8_t TOFcounterInitialize = 0;
	uint8_t TOFflag = 0;

	//HAL_Delay(5000);
	while (1) {

		if (tof_sensors == 1) {

			/* If one of the 2 sensors is not working TOFflag goes to 1
			 * and after 20 cycles (TOFcounterInitialize == 20)
			 * attempt to reconnect
			 * If one sensor stop working automatically also the other
			 * sensor stop working
			 */

			if (TOFflag == 0){

				if (VL6180X_ReadDistance(VL6180X_SENSOR1_ADDR, &distance1) == 0) {
					//printf("Sensor 1 measured distance: %d mm for real distance: %d\r\n", distance1, i*10);
					printf("Sensor 1 measured: %d mm, corrected: %d mm \r\n", distance1, linear_correction_1(distance1));
				} else {
					TOFflag = 1;
				}
				if (VL6180X_ReadDistance(VL6180X_SENSOR2_ADDR, &distance2) == 0) {
					//printf("Sensor 2 measured distance: %d mm\r\n", distance2);
					/* SENSOR 2 HAS THE SCRATCH */
					printf("Sensor 2 measured: %d mm, corrected: %d mm \r\n", distance2, linear_correction_2(distance2));
				} else {
					TOFflag = 1;
				}

			} else {
				TOFcounterInitialize++;
				printf("TOFcounterInitialize = %d \r\n", TOFcounterInitialize);
			}

			if (TOFcounterInitialize == 20) {
				printf("Attempt TOF sensors initialization...");
				if (i2c_Initialization() == 0) {
					TOFflag = 0;
				}
				TOFcounterInitialize = 0;
			}

		}

		if (line_sensors5 == 1) {

			// Read all sensors
			//for(int i = 0; i < 5; i++) {
			//	sensorValues[i] = readADC(sensorChannels[i]);
			//}
			readAllSensors(sensorChannels, sensorValues, 5);

			// Print formatted results
			printf("S1: %4lu | S2: %4lu | S3: %4lu | S4: %4lu | S5: %4lu\r\n",
					sensorValues[0], sensorValues[1], sensorValues[2],
					sensorValues[3], sensorValues[4]);

			// Visual representation
			printf("[");
			for(int i = 0; i < 5; i++) {
				printf(sensorValues[i] > 2000 ? "#" : " "); // '#' for detected
				if(i < 4) printf("       ");
			}
			printf("]\r\n\r\n");
		}

		if (line_sensor1 == 1) {
			sensor1Value = readADC(sensor1Channel);

			// print low when detect
			printf("Sensor: %d \r\n", sensor1Value);
			printf("[");
			printf(sensor1Value > 2000 ? " # " : "   "); // '#' for detected
			printf("]\r\n\r\n");
		}


		HAL_Delay(100);
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
