#ifndef ENCODER_H
#define ENCODER_H

#include <wiringPi.h>
#include <stdint.h>
#include <unordered_map>

class Encoder {
public:
    Encoder(uint out, int m_isConnectionOk);
    void set_pulses(int new_pulses);
    int32_t get_pulses() const;

private:
    uint _out;
    int _m_isConnectionOk;
    int32_t _pulses;
    int _last_level;

    static std::unordered_map<uint, Encoder*> encoders;
    static void handleInterruptStatic();
    void incrementTicks();
};

#endif  // ENCODER_H
