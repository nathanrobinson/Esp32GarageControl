#include "garage.h"
#include <Arduino.h>
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "display.h"

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
    wifiManager.sendGarageState(s);
}

void Garage::control(State targetState)
{
    if (targetState == Open || targetState == Closed)
    {
        Serial.printf("Controlling garage target state: %d\n", targetState);
        mqttManager.publishGarageControl(static_cast<MqttManager::TargetState>(targetState));
    }
}

void Garage::buttonPressed()
{
    State nextState = state == Garage::Open ? Garage::Closed : Garage::Open;
    control(nextState);
}

void Garage::updateDisplay()
{
    uint8_t idx = (uint8_t)state;
    display.showImage(idx);
}
