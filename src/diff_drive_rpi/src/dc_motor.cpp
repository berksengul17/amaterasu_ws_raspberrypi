//
// Created by pepe on 19/6/21.
//

#include "dc_motor.h"
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <wiringPi.h>

DCMotor::DCMotor(uint en_pin, int in1_pin, int in2_pin, int m_isConnectionOk, int pwmFrequency)
:_en_pin(en_pin), _in1_pin(in1_pin), _in2_pin(in2_pin), _m_isConnectionOk(m_isConnectionOk), _pwmFrequency(pwmFrequency)
{
    int wiringpi_handle_ = wiringPiSetupPinType(WPI_PIN_BCM);
    if (wiringpi_handle_ < 0) {
        throw std::runtime_error("Failed to initialize wiringpi.");
    }
    else 
    {
        pinMode(this->_in1_pin, OUTPUT);
        pinMode(this->_in2_pin, OUTPUT);
        pinMode(this->_en_pin, PWM_OUTPUT);

        // Nasıl kullanıldığını yüzde yüz anlamadım
        pwmSetRange(TOP);
        pwmSetClock(192);

        pwmWrite(this->_en_pin, 0);
    }
}

void DCMotor::write_int(int pwm)
{
    // backwards
    if (pwm < 0)
    {
        digitalWrite(this->_in1_pin, 0);
        digitalWrite(this->_in2_pin, 1);
        pwmWrite(this->_en_pin, abs(pwm));
    }
    // forwards
    else
    {
        digitalWrite(this->_in1_pin, 1);
        digitalWrite(this->_in2_pin, 0);
        pwmWrite(this->_en_pin, pwm);
    }
}

void DCMotor::write(float duty_cycle)
{
    if (duty_cycle > 1.0f)
        duty_cycle = 1.0f;
    if (duty_cycle < 0.0f)
        duty_cycle = 0.15f;

    // minimum negative and positive speed
    if (duty_cycle < 0.15f && duty_cycle > 0) 
        duty_cycle = 0.15f;
    // if (duty_cycle > -0.15f && duty_cycle < 0) 
    //     duty_cycle = -0.15f;

    write_int(static_cast<int>(duty_cycle * TOP));
}



