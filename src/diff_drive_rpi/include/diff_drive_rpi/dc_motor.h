//
// Created by pepe on 19/6/21.
//

#ifndef DIFF_DRIVE_DC_MOTOR_H
#define DIFF_DRIVE_DC_MOTOR_H

#include <cmath>

#define TOP 1024

class DCMotor {
public:
    DCMotor(uint en_pin, int in1_pin, int in2_pin, 
    int m_isConnectionOk, int pwmFrequency);
    void write_int(int pwm);
    void write(float duty_cycle);

private:
    uint _en_pin;
    uint _in1_pin;
    uint _in2_pin;

    int _m_isConnectionOk;
    unsigned int _pwmFrequency;
};


#endif //DIFF_DRIVE_DC_MOTOR_H
