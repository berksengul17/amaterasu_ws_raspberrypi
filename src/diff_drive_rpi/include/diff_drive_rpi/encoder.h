#ifndef ENCODER_H
#define ENCODER_H

#include <atomic>
#include <thread>

class Encoder {
public:
    Encoder(int pin);
    ~Encoder();

    void start();
    void stop();
    int32_t get_pulses() const;
    void set_pulses(int new_pulses);

private:
    int _pin;
    std::atomic<int32_t> _pulses;
    std::atomic<bool> _running;
    std::thread _thread;

    void countTicks();
};

#endif // ENCODER_H
