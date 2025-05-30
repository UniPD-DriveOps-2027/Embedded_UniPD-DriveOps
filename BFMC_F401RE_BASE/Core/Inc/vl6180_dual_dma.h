/*
 * vl6180_dual_dma.h
 *
 *  Created on: May 20, 2025
 *      Author: eugen
 */

#ifndef INC_VL6180_DUAL_DMA_H_
#define INC_VL6180_DUAL_DMA_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"

#define VL6180_DEFAULT_ADDR   0x29
#define VL6180_ADDR_1         0x31
#define VL6180_ADDR_2         0x33

// XSHUT pins
#define VL6180_SHUTDOWN_1_PIN  GPIO_PIN_9
#define VL6180_SHUTDOWN_1_PORT GPIOC
#define VL6180_SHUTDOWN_2_PIN  GPIO_PIN_8
#define VL6180_SHUTDOWN_2_PORT GPIOC

// maximum consecutive bad reads before hardware reset
#define MAX_CONSECUTIVE_ERRORS 5

// public interface
void    VL6180_InitSensors(I2C_HandleTypeDef *hi2c);
void    VL6180_StartMeasurement(uint8_t sensorIndex);
uint8_t VL6180_IsDataReady(uint8_t sensorIndex);
uint8_t VL6180_GetDistance(uint8_t sensorIndex);

// Must be called from your HAL_I2C callback overrides:
void VL6180_DMA_TxCpltCallback(I2C_HandleTypeDef *hi2c);
void VL6180_DMA_RxCpltCallback(I2C_HandleTypeDef *hi2c);


#ifdef __cplusplus
}
#endif

#endif /* INC_VL6180_DUAL_DMA_H_ */
