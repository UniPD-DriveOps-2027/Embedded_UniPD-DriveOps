/*
 * imu.h
 *
 *  Created on: Feb 12, 2025
 *      Author: eugen
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

void setupIMU(void);
void imuTimerCallback(void);

#ifdef __cplusplus
}
#endif



#endif /* INC_IMU_H_ */
