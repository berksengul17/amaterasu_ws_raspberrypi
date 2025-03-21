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
   
    _r_pid.set_output_limits(-1.0f, 1.0f);
    _l_pid.set_output_limits(-1.0f, 1.0f);
   
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

Robot::~Robot() {
    _fl_motor.write(0.0f);
    _rl_motor.write(0.0f);
    _fr_motor.write(0.0f);
    _rr_motor.write(0.0f);
    printf("Robot shutting down. Motors stopped.\n");
}

void Robot::updatePid(int32_t fl_encoder_ticks, int32_t fr_encoder_ticks, 
                        int32_t rl_encoder_ticks, int32_t rr_encoder_ticks)
{
    // Compute actual encoder ticks per cycle
    float l_ticks = (fl_encoder_ticks + rl_encoder_ticks) / 2.0f;
    float r_ticks = (fr_encoder_ticks + rr_encoder_ticks) / 2.0f;

    float dl_ticks = l_ticks - _state.l_ticks;
    float dr_ticks = r_ticks - _state.r_ticks;

    bool left_reverse = (_l_setpoint < 0);
    bool right_reverse = (_r_setpoint < 0);

    float l_speed = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / (_pid_rate * ROBOT_MOTOR_PPR);
    float r_speed = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / (_pid_rate * ROBOT_MOTOR_PPR);

    l_speed = left_reverse ? -l_speed : l_speed;
    r_speed = right_reverse ? -r_speed : r_speed;

    updateOdometry(l_speed, r_speed);

    // Compute target encoder ticks per cycle based on desired speed
    // float target_l_ticks = (_l_setpoint * _pid_rate / (2.0 * M_PI * ROBOT_WHEEL_RADIUS)) * ROBOT_MOTOR_PPR;
    // float target_r_ticks = (_r_setpoint * _pid_rate / (2.0 * M_PI * ROBOT_WHEEL_RADIUS)) * ROBOT_MOTOR_PPR;

    _state.l_speed = l_speed;
    _state.r_speed = r_speed;
    
    _odom.v = (_state.l_speed + _state.r_speed) / 2.0f;
    _odom.w = (_state.r_speed - _state.l_speed) / ROBOT_WHEEL_SEPARATION;
    
    // Set PID input as tick error instead of speed error
    _l_input = l_speed;
    _r_input = r_speed;

    // Run PID controller
    _l_pid.compute();
    _r_pid.compute();
    
    float l_pwm;
    float r_pwm;

    if (left_reverse) {
        l_pwm = std::min(_state.l_effort + _l_output, 0.0f);
    } else {
        l_pwm = std::max(_state.l_effort + _l_output, 0.0f);
    }

    if (right_reverse) {
        r_pwm = std::min(_state.r_effort + _r_output, 0.0f);
    } else {
        r_pwm = std::max(_state.r_effort + _r_output, 0.0f);

    }

    // STOP
    if (_l_setpoint == 0 &&_r_setpoint == 0) {
        l_pwm = 0.0f;
        r_pwm = 0.0f;
    }

    // Apply PID outputs as motor efforts
    // Adjust motor effort based on directions  
    _fl_motor.write(l_pwm);
    _rl_motor.write(l_pwm);
    _fr_motor.write(r_pwm);
    _rr_motor.write(r_pwm);

    _state.l_effort = l_pwm;
    _state.r_effort = r_pwm;

    _state.l_ticks = l_ticks;
    _state.r_ticks = r_ticks;

    printf("Left encoder ticks: %.2f, Right encoder ticks: %.2f\n"
           "State L Encoder: %.2f, State R Encoder: %.2f\n"
           "Dl: %.2f, Dr: %.2f\n"
           "Target Left Speed: %.2f, Actual Left Speed: %.2f\n"
           "Target Right Speed: %.2f, Actual Right Speed: %.2f\n"
           "Left PWM: %f, Right PWM: %f\n------------\n", 
           l_ticks, r_ticks, _state.l_ticks, _state.r_ticks, dl_ticks, dr_ticks, 
           _l_setpoint, l_speed, _r_setpoint, r_speed, l_pwm, r_pwm);
}

void Robot::updateOdometry(float l_speed, float r_speed) {
    float delta_l = l_speed * _pid_rate;
    float delta_r = r_speed * _pid_rate;
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