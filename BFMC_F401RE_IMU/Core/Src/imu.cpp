/*
 * imu.cpp
 *
 *  Created on: Feb 12, 2025
 *      Author: eugen
 */

#include "imu.h"
#include "main.h"
#include "bno055_stm32.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;

bno055_calibration_data_t calData;

volatile bool calibrationComplete = false;


void performCalibrationProcedure(void);
bool isFullyCalibrated(bno055_calibration_state_t calState);

void setupIMU(void) {
	// Initialize sensor
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	// Calibration
	performCalibrationProcedure();
}

void performCalibrationProcedure(void) {
	printf("\n=== CALIBRATION PROCEDURE ===\n");
	printf("Follow these steps:\n");
	printf("1. Place sensor on flat surface\n");
	printf("2. Slowly rotate sensor through various orientations\n");
	printf("3. Make figure-8 motions for magnetometer\n\n");

	calibrationComplete = false;

	// start timer with interrupt
	HAL_TIM_Base_Start_IT(&htim1);

	while (!calibrationComplete) {
		// wait for calibration to complete (is handled in the interrupt)
	}

	// stop timer after calibration
	HAL_TIM_Base_Stop_IT(&htim1);

	calData = bno055_getCalibrationData();
	bno055_setCalibrationData(calData);
	printf("\nCalibration successful!\n");
}

bool isFullyCalibrated(bno055_calibration_state_t calState) {
	return (calState.sys == 3 && calState.gyro == 3 && calState.mag == 3 && calState.accel == 3);
}

void imuTimerCallback(void) {
	static uint32_t lastPrintTime = 0;
	static uint32_t timerCounter = 0;

	timerCounter++;

	bno055_calibration_state_t calState = bno055_getCalibrationState();

	// Print every 2 seconds
	if (timerCounter - lastPrintTime >= 200) {
		lastPrintTime = timerCounter;

		if (calState.accel < 3) printf(" -> Accelerometer: %d\n", calState.accel);
		if (calState.gyro < 3)  printf(" -> Gyroscope:     %d\n", calState.gyro);
		if (calState.mag < 3)   printf(" -> Magnetometer:  %d\n", calState.mag);
		if (calState.sys < 3)   printf(" -> System:        %d\n", calState.sys);
		printf("----------------------------\n");
	}

	if (isFullyCalibrated(calState)) {
		calibrationComplete = true;
	}
}

