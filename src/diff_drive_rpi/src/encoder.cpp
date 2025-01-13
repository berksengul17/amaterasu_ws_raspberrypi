#include "encoder.h"
#include <iostream>

// Static function to access the map instance
std::unordered_map<unsigned, Encoder*>& Encoder::getInstances() {
    static std::unordered_map<unsigned, Encoder*> instances;
    return instances;
}

Encoder::Encoder(uint out, int m_isConnectionOk)
    : _out(out), _pulses(0), _last_level(0), _m_isConnectionOk(m_isConnectionOk) {
    if (_m_isConnectionOk < 0) {
        throw std::runtime_error("Pigpio initialization failed.");
    }

    set_mode(_m_isConnectionOk, _out, PI_INPUT);
    set_pull_up_down(_m_isConnectionOk, _out, PI_PUD_UP);
    set_glitch_filter(_m_isConnectionOk, _out, 1000); // 1000 µs debounce

    // Use the static function to access the map and register the instance
    auto& instances = getInstances();
    if (instances.find(_out) != instances.end()) {
        std::cerr << "Warning: Overwriting existing encoder instance for GPIO " << _out << std::endl;
    }
    instances[_out] = this;

    callback(_m_isConnectionOk, _out, EITHER_EDGE, Encoder::gpioCallback);
}

// Static callback function
void Encoder::gpioCallback([[maybe_unused]] int pi, unsigned user_gpio, unsigned level, [[maybe_unused]] uint32_t tick) {
    auto& instances = getInstances();
    auto it = instances.find(user_gpio);
    if (it != instances.end()) {
        Encoder* instance = it->second;
        if (level == 1 && instance->_last_level == 0) { // Rising or falling edge
            instance->incrementTicks();
        }
        // Update the last recorded level
        instance->_last_level = level;
    } else {
        std::cerr << "Error: Encoder instance not found for GPIO " << user_gpio << std::endl;
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
