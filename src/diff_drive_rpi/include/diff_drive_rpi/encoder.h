#ifndef ENCODER_H
#define ENCODER_H

#include <wiringPi.h>
#include <stdint.h>
#include <unordered_map>

class Encoder {
public:
    Encoder(int out);
    void set_pulses(int new_pulses);
    int32_t get_pulses() const;

private:
    int _out;
    int _last_state;
    uint32_t _last_time;
    int32_t _pulses;

    static std::unordered_map<int, Encoder*> encoders;
    static void handleInterruptStatic();
    void incrementTicks();
};

#endif  // ENCODER_H
