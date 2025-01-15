//
// Created by pepe on 14/7/21.
//

#include "robot.h"
#include <stdio.h>
#include <thread>
#include <chrono>

Robot::Robot(
        int m_isConnectionOk,
        float kp,
        float kd,
        float ki,
        uint sample_time_ms,
        RobotPins pins
        )
:
_kp(kp), _kd(kd), _ki(ki), _sample_time_ms(sample_time_ms),
_l_input(0.0f), _l_output(0.0f), _l_setpoint(0.0f),
_r_input(0.0f), _r_output(0.0f), _r_setpoint(0.0f),
_l_motor(pins.left.en, pins.left.in1, pins.left.in2, m_isConnectionOk, pins.left.pwm),
_r_motor(pins.right.en, pins.right.in1, pins.right.in2, m_isConnectionOk, pins.right.pwm),
_l_pid(&_l_input, &_l_output, &_l_setpoint, kp, ki, kd, sample_time_ms),
_r_pid(&_r_input, &_r_output, &_r_setpoint, kp, ki, kd, sample_time_ms)
{
    _l_motor.write(0.0f);
    _r_motor.write(0.0f);
    _l_pid.set_output_limits(-1.0f, 1.0f);
    _r_pid.set_output_limits(-1.0f, 1.0f);
    _l_setpoint = 0;
    _r_setpoint = 0;
    _left_last_error = 0.0f;
    _right_last_error = 0.0f;
    _left_integral = 0.0f;
    _right_integral = 0.0f;
    _left_speed = 0.0f;
    _right_speed = 0.0f;
    _pid_rate = float(sample_time_ms) / 1000.0f;
}

void Robot::updatePid(uint l_encoder_ticks, uint r_encoder_ticks)
{
    // printf("LTicks: %d - RTicks: %d\n", l_encoder_ticks, r_encoder_ticks);

    int32_t l_ticks = l_encoder_ticks;
    int32_t r_ticks = r_encoder_ticks;

    _state.l_position = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * l_ticks / ROBOT_MOTOR_PPR;
    _state.r_position = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * r_ticks / ROBOT_MOTOR_PPR;

    int32_t dl_ticks = l_ticks - _state.l_ticks;
    int32_t dr_ticks = r_ticks - _state.r_ticks;

    // update odometry
    updateOdometry(dl_ticks, dr_ticks);

    _state.l_ref_speed = _l_setpoint;
    _state.r_ref_speed = _r_setpoint;

    if (_state.l_ref_speed <= 0.003 && _state.r_ref_speed <= 0.003 &&
        l_encoder_ticks > 0 && r_encoder_ticks > 0) {
        _l_motor.write(0.0f); // Fully stop the left motor
        _r_motor.write(0.0f); // Fully stop the right motor
    }
    else {
        _state.l_speed = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * dl_ticks / (ROBOT_MOTOR_PPR * _pid_rate);
        _state.r_speed = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * dr_ticks / (ROBOT_MOTOR_PPR * _pid_rate);

        _odom.v = (ROBOT_WHEEL_RADIUS / 2.0f) * (_state.l_speed + _state.r_speed);
        _odom.w = (ROBOT_WHEEL_RADIUS / ROBOT_WHEEL_SEPARATION) * (_state.r_speed - _state.l_speed);

        _l_input = _state.l_speed;
        _r_input = _state.r_speed;

        // _l_output = _pid.calculate(_l_setpoint, _l_input);
        // _r_output = _pid.calculate(_r_setpoint, _r_input);
        _l_pid.compute();
        _r_pid.compute();

        // float left_error = _state.l_ref_speed - _state.l_speed;
        // _left_integral = _left_integral + left_error;
        // _left_speed = _left_speed + left_error * _kp + _left_integral * _ki + ((left_error - _left_last_error) * _kd);
        // _left_last_error = left_error;

        // float right_error = _state.r_ref_speed - _state.r_speed;
        // _right_integral = _right_integral + right_error;
        // _right_speed = _right_speed + right_error * _kp + _right_integral * _ki + ((right_error - _right_last_error) * _kd);
        // _right_last_error = right_error;

        _state.l_effort = _l_output;
        _state.r_effort = _r_output;
        // _state.l_effort = _left_speed * 100.0f;
        // _state.r_effort = _right_speed * 100.0f;

        // if (_state.l_ref_speed > 0 && _state.l_effort < 100.0f) _state.l_effort = 40.0f;
        // if (_state.r_ref_speed > 0 && _state.r_effort < 100.0f) _state.r_effort = 40.0f;
        // if (_state.l_effort > 100.0f) _state.l_effort = 60.0f;
        // if (_state.r_effort > 100.0f) _state.r_effort = 60.0f;

        _l_motor.write(_state.l_effort);
        _r_motor.write(_state.r_effort);

        printf("LEncoder ticks: %d, REncoder ticks: %d\nDl ticks: %d, Dr ticks: %d\nDesired speed: %f\nActual left speed: %f, Actual right speed: %f\nLeft PWM: %f, Right PWM: %f\n------------\n", 
                l_encoder_ticks, r_encoder_ticks, dl_ticks, dr_ticks, _state.l_ref_speed, _state.l_speed, _state.r_speed, _state.l_effort, _state.r_effort);

        // printf("LTick: %d - RTick: %d \nDl: %d - Dr: %d \nLRefSpeed: %f - RRefSpeed: %f \nLSpeed: %f - RSpeed: %f \nLOutput: %f - ROutput: %f\n--------------------\n",
        // l_ticks, r_ticks, dl_ticks, dr_ticks, _l_setpoint, _r_setpoint, _l_input, _r_input, _l_output, _r_output);   
    }

    _state.l_ticks = l_ticks;
    _state.r_ticks = r_ticks;
}

void Robot::updateOdometry(int32_t dl_ticks, int32_t dr_ticks) {
    float delta_l = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR;
    float delta_r = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR;
    float delta_center = (delta_l + delta_r) / 2;

    _odom.x_pos += delta_center * cosf(_odom.theta);
    _odom.y_pos += delta_center * sinf(_odom.theta);
    _odom.theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION;
    _odom.v = _linear;
    _odom.w = _angular;
}

void Robot::setWheels(float left_speed, float right_speed)
{    
    _l_setpoint = left_speed;
    _r_setpoint = right_speed;
}

void Robot::setUnicycle(float v, float w)
{
    // limit values
    if(v > ROBOT_MAX_LINEAR_M_S) v = ROBOT_MAX_LINEAR_M_S;
    if(v < ROBOT_MIN_LINEAR_M_S) v = ROBOT_MIN_LINEAR_M_S;
    if(w > ROBOT_MAX_ANGULAR_R_S) w = ROBOT_MAX_ANGULAR_R_S;
    if(w < ROBOT_MIN_ANGULAR_R_S) w = ROBOT_MIN_ANGULAR_R_S;

    float v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / 2;
    float v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / 2;

    _linear = v;
    _angular = w;
    setWheels(v_l, v_r);
}

void Robot::updatePidParams(float kp, float kd, float ki) {
    // Update internal PID parameters
    _l_pid.set_gains(kp, ki, kd);
    _r_pid.set_gains(kp, ki, kd);
}


RobotState Robot::getState() {
    return _state;
}

RobotOdometry Robot::getOdometry() {
    return _odom;
}