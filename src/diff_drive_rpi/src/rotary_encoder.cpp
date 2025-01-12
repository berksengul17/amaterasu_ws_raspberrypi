#include "rotary_encoder.hpp"
#include <pigpio.h>

void single_pin_encoder::_pulse(int gpio, int level, [[maybe_unused]] uint32_t tick)
{
    if (gpio == mygpio) // Ensure this is the correct GPIO
    {
        if (level == 1 && lastLevel == 0) // Detect rising edge
        {
            mycallback(1); // Call the callback with positive direction
        }

        lastLevel = level; // Update the last level
    }
}

void single_pin_encoder::_pulseEx(int gpio, int level, [[maybe_unused]] uint32_t tick, void *user)
{
    single_pin_encoder *instance = (single_pin_encoder *)user;
    instance->_pulse(gpio, level, tick); // Call the instance's pulse handler
}

single_pin_encoder::single_pin_encoder(int gpio, encoder_callback_t callback)
{
    mygpio = gpio;
    mycallback = callback;
    lastLevel = 0;

    gpioSetMode(mygpio, PI_INPUT);
    gpioSetPullUpDown(mygpio, PI_PUD_UP); // Enable pull-up resistor

    gpioSetAlertFuncEx(mygpio, _pulseEx, this); // Attach the callback function
}

void single_pin_encoder::cancel(void)
{
    gpioSetAlertFuncEx(mygpio, nullptr, nullptr); // Remove the callback
}

// Implementation for dual_encoder

dual_encoder::dual_encoder(int gpio1, encoder_callback_t callback1, int gpio2, encoder_callback_t callback2)
    : encoder1(gpio1, callback1), encoder2(gpio2, callback2) {}

void dual_encoder::cancel(void)
{
    encoder1.cancel();
    encoder2.cancel();
}
