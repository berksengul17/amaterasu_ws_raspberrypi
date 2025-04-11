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
_l_ticks(0.0f), _r_ticks(0.0f), _sample_counter(0), _dead_stop_counter(0),
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

    _tick_buffer_left[TICK_ACCUM_WINDOW] = {0}; 
    _tick_buffer_right[TICK_ACCUM_WINDOW] = {0};
   
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
    _l_ticks = (fl_encoder_ticks + rl_encoder_ticks) / 2.0f;
    _r_ticks = (fr_encoder_ticks + rr_encoder_ticks) / 2.0f;
    _sample_counter++;

    if (_sample_counter >= TICK_ACCUM_WINDOW) {
        float sum_left = 0;
        float sum_right = 0;
        for (int i = 0; i < TICK_ACCUM_WINDOW; ++i) {
            sum_left += _tick_buffer_left[i];
            sum_right += _tick_buffer_right[i];
        }
        // Rolling average is commented out but can be enabled here
        // _l_ticks = sum_left / TICK_ACCUM_WINDOW;
        // _r_ticks = sum_right / TICK_ACCUM_WINDOW;
    }

    float dl_ticks = _l_ticks - _state.l_ticks;
    float dr_ticks = _r_ticks - _state.r_ticks;

    bool left_reverse = (_l_setpoint < 0);
    bool right_reverse = (_r_setpoint < 0);

    float l_speed = (2 * M_PI * ROBOT_WHEEL_RADIUS * dl_ticks) / (_pid_rate * ROBOT_MOTOR_PPR);
    float r_speed = (2 * M_PI * ROBOT_WHEEL_RADIUS * dr_ticks) / (_pid_rate * ROBOT_MOTOR_PPR);

    l_speed = left_reverse ? -l_speed : l_speed;
    r_speed = right_reverse ? -r_speed : r_speed;

    updateOdometry(l_speed, r_speed);

    _state.l_speed = l_speed;
    _state.r_speed = r_speed;

    _state.l_ticks = _l_ticks;
    _state.r_ticks = _r_ticks;

    _odom.v = (_state.l_speed + _state.r_speed) / 2.0f;
    _odom.w = (_state.r_speed - _state.l_speed) / ROBOT_WHEEL_SEPARATION;

    constexpr float SPEED_ZERO_THRESHOLD = 0.07f;
    constexpr int DEAD_STOP_CYCLES = 3;

    bool left_stopped = fabs(_state.l_speed) < SPEED_ZERO_THRESHOLD;
    bool right_stopped = fabs(_state.r_speed) < SPEED_ZERO_THRESHOLD;

    if (left_stopped && right_stopped) {
        _dead_stop_counter++;
    } else {
        _dead_stop_counter = 0;
    }

    bool confirmed_dead_stop = (_dead_stop_counter >= DEAD_STOP_CYCLES);

    if (!confirmed_dead_stop && _l_setpoint == 0 && _r_setpoint == 0) {
        _l_pid.reset();
        _r_pid.reset();

        _fl_motor.brake();
        _rl_motor.brake();
        _fr_motor.brake();
        _rr_motor.brake();

        return;
    }

    // if (_l_setpoint == 0 && _r_setpoint == 0) {
    //     if (!_breaking) {
    //         _breaking = true; // ← mark as braked
    
    //         _l_pid.reset();
    //         _r_pid.reset();
    
    //         constexpr float SPEED_BRAKE_THRESHOLD = 0.15f;
    
    //         bool is_moving = fabs(l_speed) > SPEED_BRAKE_THRESHOLD || fabs(r_speed) > SPEED_BRAKE_THRESHOLD;
    
    //         if (is_moving) {
                    
    //             float brake_force = -copysignf(1.0f, _state.l_effort + _state.r_effort);
    //             float l_brake_pwm = brake_force * fabs(_state.l_effort);
    //             float r_brake_pwm = brake_force * fabs(_state.r_effort);
                
    //             printf("[BRAKING ONCE]\n"
    //                    "Left Speed: %.2f, Right Speed: %.2f\n"
    //                    "Left Effort: %.2f, Right Effort: %.2f\n"
    //                    "Brake PWM Left: %.2f, Brake PWM Right: %.2f\n"
    //                    "-------------------------\n",
    //                    l_speed, r_speed,
    //                    _state.l_effort, _state.r_effort,
    //                    l_brake_pwm, r_brake_pwm);
    
    //             _fl_motor.write(l_brake_pwm);
    //             _rl_motor.write(l_brake_pwm);
    //             _fr_motor.write(r_brake_pwm);
    //             _rr_motor.write(r_brake_pwm);
    
    //             std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //         }
            
    //         // Stop all motors after brake pulse
    //         _fl_motor.write(0.0f);
    //         _rl_motor.write(0.0f);
    //         _fr_motor.write(0.0f);
    //         _rr_motor.write(0.0f);
    
    //         _state.l_ticks = _l_ticks;
    //         _state.r_ticks = _r_ticks;
    
    //         return;
    //     } else {
    //         // Already braked, just keep motors stopped
    //         _fl_motor.write(0.0f);
    //         _rl_motor.write(0.0f);
    //         _fr_motor.write(0.0f);
    //         _rr_motor.write(0.0f);
    //         return;
    //     }
    // } else {
    //     _breaking = false; // ← reset flag when movement command comes
    // }    

    // Proceed with PID control
    _l_input = l_speed;
    _r_input = r_speed;

    _l_pid.compute();
    _r_pid.compute();

    float l_pwm = _state.l_effort + _l_output;
    float r_pwm = _state.r_effort + _r_output;

    // if the robot will move from deadstop
    if (confirmed_dead_stop && (_l_setpoint != 0 || _r_setpoint != 0)) {
        if ((left_reverse && !right_reverse) || (!left_reverse && right_reverse)) {
            l_pwm = left_reverse ? std::min(l_pwm, -0.4f) : std::max(l_pwm, 0.4f);
            r_pwm = right_reverse ? std::min(r_pwm, -0.4f) : std::max(r_pwm, 0.4f);
        } else if (!left_reverse && !right_reverse) {
            l_pwm = std::max(l_pwm, 0.3f);
            r_pwm = std::max(r_pwm, 0.3f);
        }
    }

    _fl_motor.write(l_pwm);
    _rl_motor.write(l_pwm);
    _fr_motor.write(r_pwm);
    _rr_motor.write(r_pwm);

    _state.l_effort = l_pwm;
    _state.r_effort = r_pwm;

    printf("Dead stop: %d\n"
           "Left encoder ticks: %.2f, Right encoder ticks: %.2f\n"
           "Dl: %.2f, Dr: %.2f\n"
           "Target Left Speed: %.2f, Actual Left Speed: %.2f\n"
           "Target Right Speed: %.2f, Actual Right Speed: %.2f\n"
           "Left PWM: %f, Right PWM: %f\n------------\n", 
           confirmed_dead_stop, _l_ticks, _r_ticks, dl_ticks, dr_ticks, 
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