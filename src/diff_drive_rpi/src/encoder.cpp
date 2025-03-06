#include "encoder.h"
#include <iostream>
#include <unordered_map>

std::unordered_map<uint, Encoder*> Encoder::encoders;

Encoder::Encoder(uint out, int m_isConnectionOk) : _out(out), _pulses(0), _last_level(0), _m_isConnectionOk(m_isConnectionOk) {
    if (_m_isConnectionOk < 0) {
        throw std::runtime_error("WiringPi initialization failed.");
    }

    pinMode(_out, INPUT);
    encoders[_out] = this;

    wiringPiISR(_out, INT_EDGE_BOTH, &Encoder::handleInterruptStatic);
}

void Encoder::handleInterruptStatic() {
    for (const auto& [pin, encoder] : encoders) {
        if (digitalRead(pin) == HIGH) {
            encoder->incrementTicks();
        }
    }
}

void Encoder::incrementTicks() {
    _pulses++;
}

void Encoder::set_pulses(int new_pulses) {
    _pulses = new_pulses;
}

int32_t Encoder::get_pulses() const {
    return _pulses;
}
