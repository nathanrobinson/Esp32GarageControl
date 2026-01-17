#include "garage.h"
#include <Arduino.h>
#include "wifi_manager.h"
#include "sound.h"

Garage garage;

Garage::Garage()
    : state(Closed)
{
    // ensure display shows initial state when constructed
    updateDisplay();
}

Garage::State Garage::getState() const
{
    return state;
}

void Garage::setState(State s)
{
    if (s == state)
        return;
    state = s;
    Serial.printf("Garage state changed to %d\n", (int)state);
    updateDisplay();
    sound.cowbell();
}

void Garage::buttonPressed()
{
    State nextState = state == Garage::Open ? Garage::Closed : Garage::Open;
    wifiManager.sendGarageState(nextState);
}

void Garage::updateDisplay()
{
    uint8_t idx = (uint8_t)state;
    display.showImage(idx);
}
