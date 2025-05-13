/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PUB_COUNT_MAX 9999
#define SERVO_PWM_COUNT_MAX 39999
#define SERVO_PRESCALER 41
#define ENC_COUNT_MAX 999
#define MOT_PRESCALER 41
#define ENC_PRESCALER 83
#define RASPI_UART_BAUD 1000000
#define PUB_PRESCALER 83
#define MOT_PWM_COUNT_MAX 39999
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LINE_1_Pin GPIO_PIN_0
#define LINE_1_GPIO_Port GPIOC
#define LINE_5_1_Pin GPIO_PIN_1
#define LINE_5_1_GPIO_Port GPIOC
#define LINE_5_2_Pin GPIO_PIN_2
#define LINE_5_2_GPIO_Port GPIOC
#define LINE_5_3_Pin GPIO_PIN_3
#define LINE_5_3_GPIO_Port GPIOC
#define SERVO_PWM_Pin GPIO_PIN_0
#define SERVO_PWM_GPIO_Port GPIOA
#define LINE_SENSOR_RIGHT_Pin GPIO_PIN_1
#define LINE_SENSOR_RIGHT_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LINE_5_4_Pin GPIO_PIN_4
#define LINE_5_4_GPIO_Port GPIOC
#define LINE_5_5_Pin GPIO_PIN_5
#define LINE_5_5_GPIO_Port GPIOC
#define MOTOR_PWM_Pin GPIO_PIN_6
#define MOTOR_PWM_GPIO_Port GPIOC
#define TOF_XSHUT_Pin GPIO_PIN_9
#define TOF_XSHUT_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENCODER_CHA_Pin GPIO_PIN_6
#define ENCODER_CHA_GPIO_Port GPIOB
#define ENCODER_CHB_Pin GPIO_PIN_7
#define ENCODER_CHB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
