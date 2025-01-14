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

#include <pigpiod_if2.h> 

DCMotor::DCMotor(uint en_pin, int in1_pin, int in2_pin, int m_isConnectionOk, int pwmFrequency)
:_en_pin(en_pin), _in1_pin(in1_pin), _in2_pin(in2_pin), _m_isConnectionOk(m_isConnectionOk), _pwmFrequency(pwmFrequency)
{
    if (m_isConnectionOk < 0)
    {
        throw std::runtime_error("Failed to initialize Raspberry Pi using Pigpio");
    } 
    else 
    {
        set_mode(_m_isConnectionOk, this->_in1_pin, PI_OUTPUT);
        set_mode(_m_isConnectionOk, this->_in2_pin, PI_OUTPUT);
        set_mode(_m_isConnectionOk, this->_en_pin, PI_OUTPUT);

        set_PWM_frequency(_m_isConnectionOk, this->_en_pin, _pwmFrequency);
        set_PWM_dutycycle(_m_isConnectionOk, this->_en_pin, 0);
    }
}

void DCMotor::write_int16(int16_t pwm)
{
    // backwards
    if (pwm < 0)
    {
        gpio_write(_m_isConnectionOk, this->_in1_pin, 0);
        gpio_write(_m_isConnectionOk, this->_in2_pin, 1);
        set_PWM_dutycycle(_m_isConnectionOk, this->_en_pin, abs(pwm));
    }
    // forwards
    else
    {
        gpio_write(_m_isConnectionOk, this->_in1_pin, 1);
        gpio_write(_m_isConnectionOk, this->_in2_pin, 0);
        set_PWM_dutycycle(_m_isConnectionOk, this->_en_pin, pwm);
    }
}

void DCMotor::write(float duty_cycle)
{
    // if (duty_cycle > 1.0f)
    //     duty_cycle = 1.0f;
    // if (duty_cycle < -1.0f)
    //     duty_cycle = -1.0f;

    write_int16(static_cast<int16_t>(duty_cycle));
}



