/*
 * pid_controller.cpp
 *
 *  Created on: Mar 30, 2025
 *      Author: eugen
 */
#include "pid_controller.h"

PID::PID(float kp, float ki, float kd, float out_min, float out_max)
    : _kp(kp), _ki(ki), _kd(kd), _out_min(out_min), _out_max(out_max)
{
    Reset();
}

float PID::clamp(float value, float min, float max) {
    if(value > max) return max;
    if(value < min) return min;
    return value;
}

void PID::SetTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::SetOutputLimits(float min, float max) {
    _out_min = min;
    _out_max = max;
}

float PID::Compute(float setpoint, float input, float dt) {
    float error = setpoint - input;

    // Proportional term
    float proportional = _kp * error;

    // Integral term
    _integral += _ki * error * dt;

    // Derivative term (using input to avoid derivative kick)
    float derivative = 0.0f;
    if(dt > 0.0f) {
        derivative = -_kd * (input - _prev_input) / dt;
    }

    // Calculate total output
    float output = proportional + _integral + derivative;

    // Clamp output to specified range using custom clamp
    float output_clamped = clamp(output, _out_min, _out_max);

    // Anti-windup: adjust integral if output is clamped
    if(output_clamped != output) {
        _integral = output_clamped - proportional - derivative;
        _integral = clamp(_integral, _out_min, _out_max);
    }

    // Save current input for next calculation
    _prev_input = input;

    return output_clamped;
}

void PID::Reset(float initial_input) {
    _integral = 0.0f;
    _prev_input = initial_input;
}
