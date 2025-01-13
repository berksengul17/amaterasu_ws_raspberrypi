//
// Created by pepe on 19/6/21.
//

#ifndef DIFF_DRIVE_ENCODER_H
#define DIFF_DRIVE_ENCODER_H

#include <pigpiod_if2.h>
#include <string>
#include <stdexcept>
#include <unordered_map>

class Encoder {
public:
    Encoder(uint out, int m_isConnectionOk);
    void set_pulses(int32_t new_pulses);
    int32_t get_pulses() const;
    static void gpioCallback(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

private:
    uint _out;
    int32_t _pulses;
    unsigned _last_level;
    int _m_isConnectionOk;

    void incrementTicks();

    // Static map to associate GPIO pins with Encoder instances
    static std::unordered_map<unsigned, Encoder*>& getInstances();
};

#endif // DIFF_DRIVE_ENCODER_H
