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
_fl_motor(pins.front_left.en, pins.front_left.in1, pins.front_left.in2, m_isConnectionOk, pins.front_left.pwm),
_fr_motor(pins.front_right.en, pins.front_right.in1, pins.front_right.in2, m_isConnectionOk, pins.front_right.pwm),
_rl_motor(pins.rear_left.en, pins.rear_left.in1, pins.rear_left.in2, m_isConnectionOk, pins.rear_left.pwm),
_rr_motor(pins.rear_right.en, pins.rear_right.in1, pins.rear_right.in2, m_isConnectionOk, pins.rear_right.pwm),
_l_pid(&_l_input, &_l_output, &_l_setpoint, kp, ki, kd, sample_time_ms),
_r_pid(&_r_input, &_r_output, &_r_setpoint, kp, ki, kd, sample_time_ms)
{
    _fl_motor.write(0.0f);
    _fr_motor.write(0.0f);
    _rl_motor.write(0.0f);
    _rr_motor.write(0.0f);
   
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

void Robot::updatePid(uint fl_encoder_ticks, uint fr_encoder_ticks, 
    uint rl_encoder_ticks, uint rr_encoder_ticks)
{
    int32_t l_ticks = (fl_encoder_ticks + rl_encoder_ticks) / 2;
    int32_t r_ticks = (fr_encoder_ticks + rr_encoder_ticks) / 2;

    _state.l_position = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * l_ticks / ROBOT_MOTOR_PPR;
    _state.r_position = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * r_ticks / ROBOT_MOTOR_PPR;

    int32_t dl_ticks = l_ticks - _state.l_ticks;
    int32_t dr_ticks = r_ticks - _state.r_ticks;

    // update odometry
    updateOdometry(dl_ticks, dr_ticks);

    _state.l_ref_speed = _l_setpoint;
    _state.r_ref_speed = _r_setpoint;

    if (_state.l_ref_speed <= 0.003 && _state.r_ref_speed <= 0.003 &&
        fl_encoder_ticks > 0 && fr_encoder_ticks > 0 &&
        rl_encoder_ticks > 0 && rr_encoder_ticks > 0) {
        _fl_motor.write(0.0f);
        _rl_motor.write(0.0f);
        _fr_motor.write(0.0f);
        _rr_motor.write(0.0f);
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else {
        _state.l_speed = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * dl_ticks / (ROBOT_MOTOR_PPR * _pid_rate);
        _state.r_speed = (2.0 * M_PI * ROBOT_WHEEL_RADIUS) * dr_ticks / (ROBOT_MOTOR_PPR * _pid_rate);

        // _odom.v = (ROBOT_WHEEL_RADIUS / 2.0f) * (_state.l_speed + _state.r_speed);
        // _odom.w = (ROBOT_WHEEL_RADIUS / ROBOT_WHEEL_SEPARATION) * (_state.r_speed - _state.l_speed);
        _odom.v = (_state.l_speed + _state.r_speed) / 2.0f;
        _odom.w = (_state.r_speed - _state.l_speed) / ROBOT_WHEEL_SEPARATION;

        _l_input = _state.l_speed;
        _r_input = _state.r_speed;

        _l_pid.compute();
        _r_pid.compute();

        _state.l_effort = _l_output;
        _state.r_effort = _r_output;

        _fl_motor.write(_state.l_effort);
        _rl_motor.write(_state.l_effort);

        _fr_motor.write(_state.r_effort);
        _rr_motor.write(_state.r_effort);

        printf("Front Left Encoder ticks: %d, Rear Left Encoder ticks: %d\nFront Right Encoder ticks: %d, Rear Right Encoder ticks: %d\nDl ticks: %d, Dr ticks: %d\nDesired speed: %f\nActual left speed: %f, Actual right speed: %f\nLeft PWM: %f, Right PWM: %f\n------------\n", 
                fl_encoder_ticks, rl_encoder_ticks, fr_encoder_ticks, rr_encoder_ticks, dl_ticks, dr_ticks, _state.l_ref_speed, _state.l_speed, _state.r_speed, _state.l_effort, _state.r_effort);
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