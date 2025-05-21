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

#define NOTE_G3  196
#define NOTE_F3  175
#define NOTE_A3  220
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void playTone(uint32_t frequency_hz, uint32_t duration_ms);
void playMelody(const uint16_t melody[][2], uint16_t notes);
void policeSiren(uint32_t cycles);
void playClick(uint32_t duration_ms);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
	if(frequency_hz == 0) {
		htim1.Instance->CCR1 = 0;  // Silence
		HAL_Delay(duration_ms);
		return;
	}

	uint32_t period = (1000000 / frequency_hz) - 1;  // 1MHz clock

	htim1.Instance->ARR = period;          // Set frequency
	htim1.Instance->CCR1 = period / 2;    // 50% duty cycle
	HAL_Delay(duration_ms);
	htim1.Instance->CCR1 = 0;             // Silence
}

void playMelody(const uint16_t melody[][2], uint16_t notes) {
	for (uint16_t i = 0; i < notes; i++) {
		playTone(melody[i][0], melody[i][1]);
		HAL_Delay(50);  // Short pause between notes
	}
}

void policeSiren(uint32_t cycles) {
	for (uint32_t i = 0; i < cycles; i++) {
		// Rising pitch
		for (uint32_t freq = 800; freq <= 1500; freq += 10) {
			playTone(freq, 10);
		}
		// Falling pitch
		for (uint32_t freq = 1500; freq >= 800; freq -= 10) {
			playTone(freq, 10);
		}
	}
}

void playClick(uint32_t duration_ms) {
	htim1.Instance->CCR1 = 10;  // Very short pulse
	HAL_Delay(duration_ms);
	htim1.Instance->CCR1 = 0;
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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/* USER CODE BEGIN 2 */

	// Melody definitions
	const uint16_t testMelody[][2] = {
			{NOTE_C4, 200}, {NOTE_D4, 200}, {NOTE_E4, 200}, {NOTE_F4, 400},
			{NOTE_G4, 400}, {NOTE_A4, 200}, {NOTE_B4, 200}, {NOTE_C5, 800}
	};

	const uint16_t marioMelody[][2] = {
			{NOTE_E4, 150}, {NOTE_E4, 150}, {NOTE_E4, 150}, {NOTE_C4, 300},
			{NOTE_E4, 300}, {NOTE_G4, 600}, {NOTE_G3, 600}
	};

	// Start PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	while (1) {

		/* USER CODE BEGIN 2 */
		playTone(1000, 500);  // Play 1kHz tone for 500ms
		playTone(2000, 500);  // Play 2kHz tone for 500ms

		playMelody(testMelody, 8);  // Play test melody
		HAL_Delay(1000);

		policeSiren(2);  // Play police siren twice
		HAL_Delay(1000);

		playMelody(marioMelody, 7);  // Play Mario theme snippet
		HAL_Delay(2000);

		// Play 5 quick clicks
		for(int i=0; i<5; i++) {
			playClick(50);
			HAL_Delay(100);
		}
		HAL_Delay(1000);
		/* USER CODE END 2 */

		/* USER CODE BEGIN WHILE */
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
