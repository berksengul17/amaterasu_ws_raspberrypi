// //
// // Created by pepe on 21/6/21.
// //

#include "pid_controller.h"
#include <cstdio>
#include <fstream>

PID::PID(float *input, float *output, float *setpoint, float kp, float ki, float kd, uint sample_time_ms)
: _sample_time_ms(sample_time_ms), _my_input(input), _my_output(output), _my_setpoint(setpoint), _out_sum(0), _last_input(0)
{
    set_output_limits(-1.0f, 1.0f);
    set_gains(kp, ki, kd);
}

void PID::set_sample_time(uint new_sample_time_ms)
{
    float ratio = (float)new_sample_time_ms / _sample_time_ms;

    _ki *= ratio;
    _kd /= ratio;
    _sample_time_ms = new_sample_time_ms;
}

void PID::set_gains(float kp, float ki, float kd)
{
    if(kp < 0 || ki < 0 || kd < 0) return;

    float sample_time_s = (float)_sample_time_ms / 1000.0f;
    printf("pid sample time: %f [s]", sample_time_s);

    _kp = kp;
    _ki = ki * sample_time_s;
    _kd = kd / sample_time_s;
}

void PID::set_output_limits(float min, float max)
{
    if(min >= max) return;
    _out_min = min;
    _out_max = max;

    if(*_my_output > _out_max) *_my_output = _out_max;
    else if(*_my_output < _out_min) *_my_output = _out_min;

    if(_out_sum > _out_max) _out_sum = _out_max;
    else if(_out_sum < _out_min) _out_sum = _out_min;
}

void PID::compute(void)
{
    // Calculate the error
    float error = *_my_setpoint - *_my_input;

    // Proportional term
    float p_input = error;

    // Integral term with anti-windup
    _out_sum += _ki * error;
    _out_sum = constrain(_out_sum, -50.0f, 50.0f);

    // Derivative term
    float d_input = error - _last_input;

    // Compute the PID output
    float output = (_kp * p_input) + _out_sum + (_kd * d_input);

    // Constrain the output to the specified limits
    output = constrain(output, _out_min, _out_max);

    // Apply the constrained output
    *_my_output = output;

    // Save the current error for the next derivative calculation
    _last_input = error;
    
    FILE *file = fopen("pid_logs.txt", "a");
    if (file) {
        // Write PID data to the file
        fprintf(file, "Setpoint: %f, Input: %f, Output: %f, P: %f, I: %f, D: %f\n",
                *_my_setpoint, *_my_input, *_my_output,
                _kp * p_input, _out_sum, _kd * d_input);

        // Close the file
        fclose(file);
    } else {
        printf("Error: Unable to open file for logging.\n");
    }


    // float input = *_my_input;
    // float error = *_my_setpoint - input;

    // float delta_input = input - _last_input;
    // _out_sum += _ki * error;

    // if(_out_sum > _out_max) _out_sum = _out_max;
    // else if(_out_sum < _out_min) _out_sum = _out_min;

    // float output = _kp * error + _out_sum - _kd * delta_input;

    // if(output > _out_max) output = _out_max;
    // else if(output < _out_min) output = _out_min;

    // *_my_output = output;
    // _last_input = input;
}

float PID::constrain(float x, float lower, float upper) {
    return (x < lower) ? lower : (x > upper) ? upper : x;
}


// #ifndef _PID_SOURCE_
// #define _PID_SOURCE_

// #include <iostream>
// #include <cmath>
// #include "pid_controller.h"

// using namespace std;

// class PIDImpl
// {
//     public:
//         PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
//         ~PIDImpl();
//         double calculate( double setpoint, double pv );

//     private:
//         double _dt;
//         double _max;
//         double _min;
//         double _Kp;
//         double _Kd;
//         double _Ki;
//         double _pre_error;
//         double _integral;
// };


// PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
// {
//     pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
// }
// double PID::calculate( double setpoint, double pv )
// {
//     return pimpl->calculate(setpoint,pv);
// }
// PID::~PID() 
// {
//     delete pimpl;
// }


// /**
//  * Implementation
//  */
// PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
//     _dt(dt),
//     _max(max),
//     _min(min),
//     _Kp(Kp),
//     _Kd(Kd),
//     _Ki(Ki),
//     _pre_error(0),
//     _integral(0)
// {
// }

// double PIDImpl::calculate( double setpoint, double pv )
// {
    
//     // Calculate error
//     double error = setpoint - pv;

//     // Proportional term
//     double Pout = _Kp * error;

//     // Integral term
//     _integral += error * _dt;
//     double Iout = _Ki * _integral;

//     // Derivative term
//     double derivative = (error - _pre_error) / _dt;
//     double Dout = _Kd * derivative;

//     // Calculate total output
//     double output = Pout + Iout + Dout;

//     // Restrict to max/min
//     if( output > _max )
//         output = _max;
//     else if( output < _min )
//         output = _min;

//     // Save error to previous error
//     _pre_error = error;

//     return output;
// }

// PIDImpl::~PIDImpl()
// {
// }

// #endif

