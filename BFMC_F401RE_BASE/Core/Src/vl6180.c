/*
 * vl6180.c
 *
 *  Created on: May 14, 2025
 *      Author: eugen
 */

#include "vl6180.h"
#include "main.h"
#include <stdio.h>


/* VL6180X Register Access Functions */

uint8_t VL6180X_WriteReg(uint8_t sensor_addr, uint16_t reg, uint8_t value) {
	uint8_t data[3] = { (reg >> 8) & 0xFF, reg & 0xFF, value };
	return HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, data, 3, 100);
}

uint8_t VL6180X_ReadReg(uint8_t sensor_addr, uint16_t reg) {
	uint8_t tx_data[2] = { (reg >> 8) & 0xFF, reg & 0xFF };
	uint8_t rx_data = 0;
	HAL_I2C_Master_Transmit(&hi2c1, sensor_addr << 1, tx_data, 2, 100);
	HAL_I2C_Master_Receive(&hi2c1, (sensor_addr << 1) | 0x01, &rx_data, 1, 100);
	return rx_data;
}

void VL6180X_Setup(VL6180X_Sensor *sensors) {
	// Sensor 0
	sensors[0].addr = VL6180X_1_ADDR;
	sensors[0].xshut_pin  = VL6180X_1_SHUT_PIN;
	sensors[0].xshut_port = VL6180X_1_SHUT_PORT;
	sensors[0].initialized = false;
	sensors[0].m = VL6180X_1_M;
	sensors[0].q = VL6180X_1_Q;


	// Sensor 1
	sensors[1].addr = VL6180X_2_ADDR;
	sensors[1].xshut_pin  = VL6180X_2_SHUT_PIN;
	sensors[1].xshut_port = VL6180X_2_SHUT_PORT;
	sensors[1].initialized = false;
	sensors[1].m = VL6180X_2_M;
	sensors[1].q = VL6180X_2_Q;
}

uint8_t VL6180X_Init(uint8_t sensor_addr) {
	if (VL6180X_ReadReg(sensor_addr, VL6180X_IDENTIFICATION_MODEL_ID) != VL6180X_ID) {
		return 1;
	}

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
	VL6180X_WriteReg(sensor_addr, 0x0011, 0x10);
	VL6180X_WriteReg(sensor_addr, 0x012A, 0x02);
	VL6180X_WriteReg(sensor_addr, 0x003A, 0x03);
	VL6180X_WriteReg(sensor_addr, 0x0031, 0xFF);
	VL6180X_WriteReg(sensor_addr, 0x0014, 0x24);

	return 0;
}

void VL6180X_i2c_Init(VL6180X_Sensor *sensors) {

	/* IMPORTANT!!!
	 * VL6180X_2 HAS THE SCRATCH
	 * swapping sensor may lead missreading
	 * due to calibration of each VL6180X
	 * with separate linear regression
	 */

	for (uint8_t i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		if (!sensors[i].initialized) {
			HAL_Delay(50);
			HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_SET);
			HAL_Delay(50);
			VL6180X_WriteReg(VL6180X_DEFAULT_ADDR, VL6180X_REG_I2C_SLAVE_DEV_ADDR, sensors[i].addr);
			HAL_Delay(50);
			if (VL6180X_Init(sensors[i].addr) == 0) {
				sensors[i].initialized = true;
			}
		}
	}
}

uint8_t VL6180X_ReadDistance(uint8_t sensor_addr, uint8_t *distance) {
	uint32_t start_time = HAL_GetTick();
	const uint32_t timeout_ms = 60;

	VL6180X_WriteReg(sensor_addr, VL6180X_SYSRANGE_START, 0x01);

	while ((VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_STATUS) & 0x01) == 0) {
		if ((HAL_GetTick() - start_time) > timeout_ms) {
			return 1;
		}
	}

	*distance = VL6180X_ReadReg(sensor_addr, VL6180X_RESULT_RANGE_VAL);
	VL6180X_WriteReg(sensor_addr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return 0;
}

void VL6180X_ReadAllDistances(VL6180X_Sensor *sensors, uint8_t *counter) {
	for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		if (sensors[i].initialized) {
			if (VL6180X_ReadDistance(sensors[i].addr, &sensors[i].distance) != 0) {
				/* Error reading ith VL6180X */
				sensors[i].initialized = false;

				/* Shutdown that VL6180X */
				HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
				HAL_Delay(50);
			}
		} else {
			/* ith VL6180X not initialized */
			(*counter)++;
		}
	}
}

void VL6180X_PrintAllDistances(VL6180X_Sensor *sensors){
	for (int i = 0; i < NUMBER_OF_VL6180X_ID; i++) {
		printf("Sensor %d  ",i+1);
		if (sensors[i].initialized) {
			if (VL6180X_ReadDistance(sensors[i].addr, &sensors[i].distance) != 0) {
				/* Error reading ith VL6180X */
				sensors[i].initialized = false;

				/* Shutdown that VL6180X */
				HAL_GPIO_WritePin(sensors[i].xshut_port, sensors[i].xshut_pin, GPIO_PIN_RESET);
				HAL_Delay(50);
			} else {
				printf("measured %d, corrected %d ",
						sensors[i].distance,
						(int)(sensors[i].m * sensors[i].distance + sensors[i].q));
			}
		} else {
			printf("--- ");
		}
		printf("     |     ");
	}
	printf("\r\n");
}

