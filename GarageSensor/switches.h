// Switches.h - Wrapper class to read two normally-open switches on DI0 and DI1
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "pin_config.h"

class Switches
{
public:
    // activeLow: true if switch closes to GND (common for NO switches)
    explicit Switches(bool activeLow = true);

    // Initialize the IO expander object. Returns true on success.
    bool init();

    // checks the value of the switches
    void loop();

private:
    // Configure DI0 and DI1 as inputs. Returns true on success.
    bool begin();

    // Return raw level mask for DI0/DI1 (bits IO_EXPANDER_PIN_NUM_0/1)
    uint32_t readRaw();

    // High-level helpers: true when switch is pressed (closed)
    bool isDoorClosedActive(uint32_t levels);
    bool isDoorOpenActive(uint32_t levels);

    // explicit Wire access; no expander object
    bool _activeLow;
    bool _ready;
};

extern Switches switches;
