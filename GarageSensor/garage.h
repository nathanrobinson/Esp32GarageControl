#pragma once

#include <Arduino.h>
#include "display.h"

class Garage
{
public:
    enum State
    {
        Closed = 0,
        Opening = 1,
        Open = 2,
        Closing = 3,
    };

    Garage();
    State getState() const;
    void setState(State s);
    void buttonPressed();
    void control(State targetState);

private:
    State state;
    void updateDisplay();
};

extern Garage garage;
