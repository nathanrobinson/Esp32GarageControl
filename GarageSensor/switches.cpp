// Implementation of Switches class to read normally-open switches on DI0 and DI1
#include "switches.h"
#include "pin_config.h" // provide pin masks (DI0_mask, DI1_mask) and register addresses
#include "garage.h"
#include <Wire.h>

Switches switches;

Switches::Switches(bool activeLow)
    : _activeLow(activeLow), _ready(false) {}

// Initialize Wire/I2C. Returns true when Wire appears initialized.
bool Switches::init()
{
    // Ensure Wire is initialized (safe if already started in setup)
    Serial.println("Switches::init: starting");
    Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    Wire.setClock(400000);

    _ready = begin();
    // Serial.printf("Switches::init: begin() returned %s\n", _ready ? "true" : "false");
    return _ready;
}

// Probe expected I2C devices: external expander and the digital-pins device
bool Switches::begin()
{
    // Serial.printf("Switches::begin: probing expander 0x%02x\n", EXPANDER_I2C_EXTERNAL_ADDRESS);
    Wire.beginTransmission(EXPANDER_I2C_EXTERNAL_ADDRESS);
    int r = Wire.endTransmission();
    if (r != 0)
    {
        // Serial.printf("Switches::begin: no ACK from 0x%02x (err %d)\n", EXPANDER_I2C_EXTERNAL_ADDRESS, r);
        return false;
    }

    // Serial.printf("Switches::begin: probing digital pins device 0x%02x\n", I2C_DIGITAL_PINS_ADDRESS);
    Wire.beginTransmission(I2C_DIGITAL_PINS_ADDRESS);
    r = Wire.endTransmission();
    if (r != 0)
    {
        // Serial.printf("Switches::begin: no ACK from 0x%02x (err %d)\n", I2C_DIGITAL_PINS_ADDRESS, r);
        return false;
    }

    Serial.println("Switches::begin: probes OK");
    return true;
}

void Switches::loop()
{
    if (!_ready)
    {
        // initialization failed or not run; skip reading to avoid errors
        return;
    }
    uint32_t levels = readRaw();
    bool closedActive = switches.isDoorClosedActive(levels);
    bool openActive = switches.isDoorOpenActive(levels);
    Garage::State before = garage.getState();
    // Serial.printf("Switches::loop: levels=0x%02x closed=%d open=%d garageBefore=%d\n", (unsigned)levels, closedActive, openActive, (int)before);

    if (closedActive)
    {
        garage.publishState(Garage::State::Closed);
    }
    else if (openActive)
    {
        garage.publishState(Garage::State::Open);
    }
    else if (before == Garage::State::Closed)
    {
        garage.publishState(Garage::State::Opening);
    }
    else if (before == Garage::State::Open)
    {
        garage.publishState(Garage::State::Closing);
    }

    // Garage::State after = garage.getState();
    // if (after != before)
    // Serial.printf("Switches::loop: garageState changed %d -> %d\n", (int)before, (int)after);
}

// Read raw levels from the digital-pins I2C address. Returns the raw byte
// read from the expander so callers can mask the correct bits (DI0=bit0, DI1=bit5).
uint32_t Switches::readRaw()
{
    uint8_t val = 0;

    // Serial.printf("Switches::readRaw: request from 0x%02x\n", I2C_DIGITAL_PINS_ADDRESS);
    Wire.requestFrom((int)I2C_DIGITAL_PINS_ADDRESS, 1);

    // Do not block waiting for data; return 0 immediately if none available.
    if (!Wire.available())
    {
        Serial.println("Switches::readRaw: no data available, returning 0");
        return 0;
    }

    val = Wire.read();
    // Serial.printf("Switches::readRaw: got 0x%02x\n", val);

    return static_cast<uint32_t>(val);
}

bool Switches::isDoorClosedActive(uint32_t levels)
{
    bool high = (levels & DI0_mask) != 0;
    return _activeLow ? !high : high;
}

bool Switches::isDoorOpenActive(uint32_t levels)
{
    // 0xdf indicates D1 is pressed. 0xff indicates D1 is open.
    bool high = (levels & DI1_mask) != 0;
    return _activeLow ? !high : high;
}
