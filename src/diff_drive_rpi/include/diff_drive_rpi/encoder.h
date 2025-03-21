#ifndef ENCODER_H
#define ENCODER_H

#include <atomic>
#include <thread>
#include <deque>
#include <numeric>

class Encoder {
public:
    Encoder(int pin);
    ~Encoder();

    void start();
    void stop();
    int32_t get_pulses() const;
    void set_pulses(int new_pulses);
    float get_speed();
    void set_speed(float new_speed);

private:
    int _pin;
    std::atomic<int32_t> _pulses;
    std::atomic<bool> _running;
    std::thread _thread;
    std::atomic<float> _speed;
    std::chrono::steady_clock::time_point _last_pulse_time;

    std::deque<float> _speed_buffer;
    static const size_t SPEED_AVG_WINDOW = 3;       

    void countTicks();
};

#endif // ENCODER_H
