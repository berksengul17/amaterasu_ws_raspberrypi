//
// Created by pepe on 14/7/21.
//

#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include "math.h"
#include "pid_controller.h"
#include "dc_motor.h"

#define L_MOTOR_MIN_SPEED -1.0f
#define L_MOTOR_MAX_SPEED 1.0f
#define R_MOTOR_MIN_SPEED -1.0f
#define R_MOTOR_MAX_SPEED 1.0f
#define ROBOT_MOTOR_PPR 40.0f // 20.0f
#define ROBOT_WHEEL_RADIUS 0.034f // m
#define ROBOT_WHEEL_SEPARATION 0.296f // m
#define ROBOT_MAX_LINEAR_M_S 1.0
#define ROBOT_MIN_LINEAR_M_S (-1.0)
#define ROBOT_MAX_ANGULAR_R_S 2.0
#define ROBOT_MIN_ANGULAR_R_S (-2.0)

struct MotorPins
{
    uint pwm;
    uint en;
    uint in1;
    uint in2;
};

struct RobotPins
{
    MotorPins front_left;
    MotorPins front_right;
    MotorPins rear_left;
    MotorPins rear_right;
};

struct RobotState
{
    float l_ticks;
    float r_ticks;
    float l_position;
    float r_position;
    float l_speed;
    float r_speed;
    float l_effort;
    float r_effort;
    float prev_l_setpoint;
    float prev_r_setpoint;
};

struct RobotOdometry
{
    float x_pos;
    float y_pos;
    float theta;
    float v;
    float w;
};

class Robot{
public:
    Robot(
            int m_isConnectionOk,
            float kp,
            float kd,
            float ki,
            uint sample_time_ms,
            RobotPins pins
            );
    void start();
    void setWheels(float left_speed, float right_speed);
    void setUnicycle(float v, float w);
    RobotState getState();
    RobotOdometry getOdometry();
    void updatePid(int32_t fl_encoder_ticks, int32_t fr_encoder_ticks, 
        int32_t rl_encoder_ticks, int32_t rr_encoder_ticks);
    void updatePidParams(float kp, float kd, float ki);
    
    ~Robot();

private:
    float _kp;
    float _kd;
    float _ki;
    float _pid_rate;
    uint _sample_time_ms;
    float _l_input;
    float _l_output;
    float _l_setpoint;
    float _r_input;
    float _r_output;
    float _r_setpoint;
    float _l_ticks;
    float _r_ticks;
    int _dead_stop_counter;
    bool new_command;
    float l_sum;
    float r_sum;
    int l_count;
    int r_count;
    float _linear;
    float _angular;
    float _left_last_error;
    float _right_last_error;
    float _left_integral;
    float _right_integral;
    float _left_speed;
    float _right_speed;

    DCMotor _fl_motor;
    DCMotor _fr_motor;
    DCMotor _rl_motor;
    DCMotor _rr_motor;
    PID _l_pid;
    PID _r_pid;
    RobotState _state;
    RobotOdometry _odom;

    void controlLoop();
    void updateOdometry(float l_speed, float r_speed);
};

#endif //DIFF_DRIVE_ROBOT_H
