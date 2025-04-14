#include "encoder.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <wiringPi.h>
#include <unistd.h>  // For usleep()
#include <robot.h>

#define DEBOUNCE_TIME_MS 500  // Ignore signals faster than 1ms
#define MAX_TIME_DIFF 0.05f  // seconds

Encoder::Encoder(int pin) : _pin(pin), _pulses(0), _running(false), _speed(0) {
    if (wiringPiSetupGpio() == -1) {
        throw std::runtime_error("Failed to initialize WiringPi.");
    }

    pinMode(_pin, INPUT);
    pullUpDnControl(_pin, PUD_UP);  // Enable pull-up resistor
    _last_pulse_time = std::chrono::steady_clock::now();
}

// **Thread Function for Counting Encoder Pulses**
void Encoder::countTicks() {
    bool last_state = digitalRead(_pin);

    while (_running) {
        bool current_state = digitalRead(_pin);

        if (current_state != last_state) {  // Detect rising or falling edge
            _pulses++;
        }

        last_state = current_state;
        usleep(DEBOUNCE_TIME_MS);  // Small delay (1ms) to balance accuracy and CPU usage
    }
}

// **Start the encoder counting thread**
void Encoder::start() {
    _running = true;
    _thread = std::thread(&Encoder::countTicks, this);
}

// **Stop the encoder counting thread**
void Encoder::stop() {
    _running = false;
    if (_thread.joinable()) {
        _thread.join();
    }
}

// **Get current pulse count**
int32_t Encoder::get_pulses() const {
    return _pulses.load();
}

float Encoder::get_speed() {
    return _speed.load();
}

void Encoder::set_speed(float new_speed) {
    _speed.store(new_speed);
}

// **Manually set pulses (for reset)**
void Encoder::set_pulses(int new_pulses) {
    _pulses = new_pulses;
}

// **Destructor to ensure proper cleanup**
Encoder::~Encoder() {
    stop();
}
