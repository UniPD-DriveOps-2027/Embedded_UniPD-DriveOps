/*
 * vl6180_dual_dma.c
 *
 *  Created on: May 20, 2025
 *      Author: eugen
 */

#include "vl6180_dual_dma.h"
#include <stdio.h>

static I2C_HandleTypeDef *vl6180_i2c;
static const uint8_t sensor_addr[2] = { VL6180_DEFAULT_ADDR, VL6180_DEFAULT_ADDR };
static uint8_t distance_data[2];
static uint8_t data_ready[2];
static uint8_t error_count[2];

// this tells us which sensor index we last started
static volatile uint8_t last_start_idx = 0;

// VL6180 registers
static const uint16_t REG_SYSRANGE_START   = 0x0018;
static const uint16_t REG_RESULT_RANGE_VAL = 0x0062;
static const uint16_t REG_I2C_SLAVE_DEV    = 0x0212;

// helper: blocking write for initialization only
static void writeRegBlocking(uint8_t addr7, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = { (reg>>8)&0xFF, (reg&0xFF), val };
    if (HAL_I2C_Master_Transmit(vl6180_i2c, addr7<<1, buf, 3, HAL_MAX_DELAY) != HAL_OK) {
        printf("ERR init write to 0x%02X reg 0x%04X\n", addr7, reg);
    }
}

// Call at startup
void VL6180_InitSensors(I2C_HandleTypeDef *hi2c) {
    vl6180_i2c = hi2c;
    for (int i=0;i<2;i++) {
        data_ready[i]=0; error_count[i]=0;
    }

    // Power down both sensors initially
    HAL_GPIO_WritePin(VL6180_SHUTDOWN_1_PORT, VL6180_SHUTDOWN_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL6180_SHUTDOWN_2_PORT, VL6180_SHUTDOWN_2_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Power up only sensor 0 (default address 0x29)
    HAL_GPIO_WritePin(VL6180_SHUTDOWN_1_PORT, VL6180_SHUTDOWN_1_PIN, GPIO_PIN_SET);
    HAL_Delay(50); // Allow boot time

    printf("VL6180 initialized (single sensor at 0x29)\n");
}

// start a measurement on one sensor
void VL6180_StartMeasurement(uint8_t idx) {
    last_start_idx = idx;
    uint8_t cmd = 0x01;
    printf("VL6180 start idx=%u\n", idx);
    HAL_I2C_Mem_Write_DMA(vl6180_i2c,
        (sensor_addr[idx]<<1),
        REG_SYSRANGE_START,
        I2C_MEMADD_SIZE_16BIT,
        &cmd, 1
    );
}

// HAL callback: TX complete for that write
void VL6180_DMA_TxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != vl6180_i2c) return;
    uint8_t idx = last_start_idx;
    printf("VL6180 TXCplt idx=%u → queue RX\n", idx);
    HAL_I2C_Mem_Read_DMA(vl6180_i2c,
        (sensor_addr[idx]<<1),
        REG_RESULT_RANGE_VAL,
        I2C_MEMADD_SIZE_16BIT,
        &distance_data[idx], 1
    );
}

// HAL callback: RX complete
void VL6180_DMA_RxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != vl6180_i2c) return;
    uint8_t idx = last_start_idx;
    uint8_t d = distance_data[idx];
    printf("VL6180 RXCplt idx=%u: raw=%u\n", idx, d);
    if (d < 200) {
        data_ready[idx] = 1;
        error_count[idx] = 0;
    } else {
        if (++error_count[idx]>=MAX_CONSECUTIVE_ERRORS) {
            // hardware reset
            printf("VL6180 idx=%u RESET\n", idx);
            if (idx==0) {
                HAL_GPIO_WritePin(VL6180_SHUTDOWN_1_PORT, VL6180_SHUTDOWN_1_PIN, GPIO_PIN_RESET);
                HAL_Delay(10);
                HAL_GPIO_WritePin(VL6180_SHUTDOWN_1_PORT, VL6180_SHUTDOWN_1_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(VL6180_SHUTDOWN_2_PORT, VL6180_SHUTDOWN_2_PIN, GPIO_PIN_RESET);
                HAL_Delay(10);
                HAL_GPIO_WritePin(VL6180_SHUTDOWN_2_PORT, VL6180_SHUTDOWN_2_PIN, GPIO_PIN_SET);
            }
            error_count[idx]=0;
        }
    }
}

// called by your timer or loop
uint8_t VL6180_IsDataReady(uint8_t idx) {
    return data_ready[idx];
}

// get & clear
uint8_t VL6180_GetDistance(uint8_t idx) {
    data_ready[idx]=0;
    return distance_data[idx];
}

// HAL error callback override, for debug
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c==vl6180_i2c) {
        printf("I2C ERR: 0x%08X\n", hi2c->ErrorCode);
    }
}
