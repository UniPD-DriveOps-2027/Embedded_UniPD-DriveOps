/*
 * pid_controller.h
 *
 *  Created on: Mar 30, 2025
 *      Author: eugen
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

class PID {
public:
    PID(float kp, float ki, float kd, float out_min, float out_max);

    void SetTunings(float kp, float ki, float kd);
    void SetOutputLimits(float min, float max);
    float Compute(float setpoint, float input, float dt);
    void Reset(float initial_input = 0.0f);

private:
    float _kp;
    float _ki;
    float _kd;
    float _integral;
    float _prev_input;
    float _out_min;
    float _out_max;

    float clamp(float value, float min, float max);
};

#endif /* INC_PID_CONTROLLER_H_ */
