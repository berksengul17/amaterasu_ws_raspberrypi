#include "encoder.h"
#include <iostream>
#include <unordered_map>
#include <stdexcept>
#include <chrono>

std::unordered_map<int, Encoder*> Encoder::encoders;

#define DEBOUNCE_TIME_MS 1  // Ignore signals faster than 5ms

Encoder::Encoder(int out) : 
_out(out), _pulses(0), _last_state(digitalRead(out)), _last_time(0) {

    if (wiringPiSetupGpio() == -1) {
        throw std::runtime_error("Failed to initialize WiringPi.");
    }

    pinMode(_out, INPUT);
    pullUpDnControl(_out, PUD_UP);  // Enable pull-up resistor

    encoders[_out] = this;

    wiringPiISR(_out, INT_EDGE_BOTH, &Encoder::handleInterruptStatic);
}

void Encoder::handleInterruptStatic() {
    for (auto& [pin, encoder] : encoders) {
        int prev_state = encoder->_last_state;
        int current_state = digitalRead(pin);
        uint32_t now = millis();

        if ((now - encoder->_last_time) > DEBOUNCE_TIME_MS) {
            if (current_state != prev_state) {
                encoder->_last_state = current_state;
                encoder->_last_time = now;

                if (prev_state != current_state) {  // Any state change
                    encoder->incrementTicks();
                }                
            }
        }
    }
}

// Increment tick count
void Encoder::incrementTicks() {
    _pulses++;
}

// Set pulses manually (for resetting)
void Encoder::set_pulses(int new_pulses) {
    _pulses = new_pulses;
}

// Get pulse count
int32_t Encoder::get_pulses() const {
    return _pulses;
}
