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
_l_ticks(0.0f), _r_ticks(0.0f), _dead_stop_counter(0), new_command(false), l_sum(0.0f), r_sum(0.0f), l_count(0), r_count(0),
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
    _l_pid.reset();
    _r_pid.reset();

    _fl_motor.brake();
    _rl_motor.brake();
    _fr_motor.brake();
    _rr_motor.brake();
    printf("Robot shutting down. Motors stopped.\n");
}

void Robot::updatePid(int32_t fl_encoder_ticks, int32_t fr_encoder_ticks, 
                      int32_t rl_encoder_ticks, int32_t rr_encoder_ticks)
{
    _l_ticks = (fl_encoder_ticks + rl_encoder_ticks) / 2.0f;
    _r_ticks = (fr_encoder_ticks + rr_encoder_ticks) / 2.0f;

    float dl_ticks = _l_ticks - _state.l_ticks;
    float dr_ticks = _r_ticks - _state.r_ticks;

    bool left_reverse = (_l_setpoint < 0);
    bool right_reverse = (_r_setpoint < 0);

    float l_speed = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / (_pid_rate * ROBOT_MOTOR_PPR);
    float r_speed = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / (_pid_rate * ROBOT_MOTOR_PPR);

    l_speed = left_reverse ? -l_speed : l_speed;
    r_speed = right_reverse ? -r_speed : r_speed;

    _state.l_speed = l_speed;
    _state.r_speed = r_speed;
    
    _state.l_ticks = _l_ticks;
    _state.r_ticks = _r_ticks;

    if (!new_command && l_count < 1 && r_count < 1) {
        l_count++;
        r_count++;

        l_sum += l_speed;
        r_sum += r_speed;

    } else {
        l_sum = l_speed;
        r_sum = r_speed;

        l_count = 1;
        r_count = 1;
    }

    float l_avg = l_sum / l_count;
    float r_avg = r_sum / r_count;

    // // when turning ignore slippage
    // if (left_reverse || right_reverse) {
    //     updateOdometry(0.0f, 0.0f);
    //     _odom.v = 0.0f;
    // } else {
    //     updateOdometry(l_speed, r_speed);
    // }
    
    updateOdometry(l_speed, r_speed);
    
    _odom.v = (l_avg + r_avg) / 2.0f;
    _odom.w = (r_avg - l_avg) / ROBOT_WHEEL_SEPARATION;

    constexpr float SPEED_ZERO_THRESHOLD = 0.1f;
    constexpr int DEAD_STOP_CYCLES = 10;

    bool left_stopped = fabs(_state.l_speed) < SPEED_ZERO_THRESHOLD;
    bool right_stopped = fabs(_state.r_speed) < SPEED_ZERO_THRESHOLD;

    if (left_stopped && right_stopped) {
        _dead_stop_counter++;
    } else {
        _dead_stop_counter = 0;
    }

    bool confirmed_dead_stop = (_dead_stop_counter >= DEAD_STOP_CYCLES);

    // if moving and wish to stop
    if (!confirmed_dead_stop && _l_setpoint == 0 && _r_setpoint == 0) {
        _l_pid.reset();
        _r_pid.reset();

        _fl_motor.brake();
        _rl_motor.brake();
        _fr_motor.brake();
        _rr_motor.brake();

        _state.l_effort = 0.0f;
        _state.r_effort = 0.0f;

        return;
    }

    _state.prev_l_setpoint = _l_setpoint;
    _state.prev_r_setpoint = _r_setpoint;

    _l_setpoint = fabs(_l_setpoint);
    _r_setpoint = fabs(_r_setpoint);

    // Proceed with PID control
    _l_input = fabs(l_avg);
    _r_input = fabs(r_avg);

    _l_pid.compute();
    _r_pid.compute();

    float l_pwm = _state.l_effort + _l_output;
    float r_pwm = _state.r_effort + _r_output;

    printf("PID results: %.2f | %.2f\n", l_pwm, r_pwm);
   
    _l_setpoint = left_reverse ? -_l_setpoint : _l_setpoint;
    _r_setpoint = right_reverse ? -_r_setpoint : _r_setpoint;

    // if the robot will move from deadstop
    if (confirmed_dead_stop && (_l_setpoint != 0 || _r_setpoint != 0)) {
        if ((left_reverse && !right_reverse) || (!left_reverse && right_reverse)) {
            l_pwm = left_reverse ? std::min(-l_pwm, -0.5f) : std::max(l_pwm, 0.5f);
            r_pwm = right_reverse ? std::min(-r_pwm, -0.5f) : std::max(r_pwm, 0.5f);
        } else if (!left_reverse && !right_reverse) {
            l_pwm = std::max(l_pwm, 0.3f);
            r_pwm = std::max(r_pwm, 0.3f);
        }
    }

    if (left_reverse && l_pwm > 0) {
        l_pwm = -l_pwm;
    }

    if (right_reverse && r_pwm > 0) {
        r_pwm = -r_pwm;
    }

    l_pwm = std::max(std::min(l_pwm, 1.0f), -1.0f);
    r_pwm = std::max(std::min(r_pwm, 1.0f), -1.0f);

    if (!left_reverse && !right_reverse && _l_setpoint != 0 && _r_setpoint != 0) {
        l_pwm = std::min(l_pwm, 0.3f);
        r_pwm = std::min(r_pwm, 0.3f);
    }

    _fl_motor.write(l_pwm);
    _rl_motor.write(l_pwm);
    _fr_motor.write(r_pwm);
    _rr_motor.write(r_pwm);

    _state.l_effort = fabs(l_pwm);
    _state.r_effort = fabs(r_pwm);

    printf("Dead stop: %d\n"
           "Left reverse: %d | Right reverse: %d\n"
           "Front Left encoder ticks: %.d, Front Right encoder ticks: %d\n"
           "Rear Left encoder ticks: %d, Rear Right encoder ticks: %d\n"
           "Dl: %.2f, Dr: %.2f\n"
           "Target Left Speed: %.2f, Actual Left Speed: %.2f\n"
           "Target Right Speed: %.2f, Actual Right Speed: %.2f\n"
           "Left PWM: %f, Right PWM: %f\n------------\n", 
           confirmed_dead_stop, left_reverse, right_reverse, fl_encoder_ticks, fr_encoder_ticks, 
           rl_encoder_ticks, rr_encoder_ticks, dl_ticks, dr_ticks, _l_setpoint, l_avg, 
           _r_setpoint, r_avg, l_pwm, r_pwm);
}

void Robot::updateOdometry(float l_speed, float r_speed) {
    float delta_l = l_speed * _pid_rate;
    float delta_r = r_speed * _pid_rate;
    float delta_center = (delta_l + delta_r) / 2;

    _odom.x_pos += delta_center * cosf(_odom.theta);
    _odom.y_pos += delta_center * sinf(_odom.theta);
    _odom.theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION;
}

void Robot::setWheels(float left_speed, float right_speed)
{    
    _l_setpoint = left_speed;
    _r_setpoint = right_speed;

    if (_l_setpoint != _state.prev_l_setpoint && 
        _r_setpoint != _state.prev_r_setpoint) {
        new_command = true;
    } else {
        new_command = false;
    }

    printf("Setpoints are set: %.2f | %.2f\n", _l_setpoint, _r_setpoint);
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

    if ((w != 0 && fabs(_angular - w) <= 0.1) || (v != 0 && fabs(_linear - v) <= 0.1)) {
        new_command = false;
    } else {
        new_command = true;
    }

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