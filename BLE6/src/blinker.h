#ifndef BLINKER_H
#define BLINKER_H

#include <cstdint>

class Blinker
{
public:
    Blinker();
    ~Blinker();

    // number <= 0 means run indefinitely
    void blink(int number, uint32_t onTimeMs, uint32_t offTimeMs);

    // non-copyable
    Blinker(const Blinker &) = delete;
    Blinker &operator=(const Blinker &) = delete;
};
#endif
