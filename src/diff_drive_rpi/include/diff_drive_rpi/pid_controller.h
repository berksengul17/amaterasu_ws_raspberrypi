// //
// // Created by pepe on 21/6/21.
// //

#ifndef DIFF_DRIVE_PID_CONTROLLER_H
#define DIFF_DRIVE_PID_CONTROLLER_H

#include "cstdio"
#include <string>
#include <stdexcept>

class PID {
public:
    PID(float* input, float* output, float* setpoint, float kp, float ki, float kd, uint sample_time_ms);
    void compute(void);
    void set_output_limits(float min, float max);
    void set_gains(float kp, float ki, float kd);
    void set_sample_time(uint new_sample_time_ms);

private:
    uint _sample_time_ms;
    uint _last_time;
    float _kp;
    float _kd;
    float _ki;

    float* _my_input;
    float* _my_output;
    float* _my_setpoint;
    float _out_min;
    float _out_max;
    float _out_sum;
    float _err_sum;
    float _last_err;
    float _i_term;
    float _last_input;

};


#endif //DIFF_DRIVE_PID_CONTROLLER_H

// #ifndef _PID_H_
// #define _PID_H_

// class PIDImpl;
// class PID
// {
//     public:
//         // Kp -  proportional gain
//         // Ki -  Integral gain
//         // Kd -  derivative gain
//         // dt -  loop interval time
//         // max - maximum value of manipulated variable
//         // min - minimum value of manipulated variable
//         PID( double dt, double max, double min, double Kp, double Kd, double Ki );

//         // Returns the manipulated variable given a setpoint and current process value
//         double calculate( double setpoint, double pv );
//         ~PID();

//     private:
//         PIDImpl *pimpl;
// };

// #endif