#pragma once

#include <Arduino.h>
#include <memory>
#include <functional>
#include <Adafruit_XCA9554.h>
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"

struct TouchPoint
{
    int32_t x;
    int32_t y;
};

class Touch
{
public:
    using TouchCallback = std::function<void(const TouchPoint &)>;

    Touch();
    void init(TouchCallback cb);

private:
    static Touch *instance;
    static void IIC_Touch_Interrupt(void);
    static void touchTaskStatic(void *pv);

    void touchTask();

    Adafruit_XCA9554 expander;
    std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus;
    std::unique_ptr<Arduino_IIC> FT3168;

    // synchronization and callback
    void *touchSem = nullptr; // SemaphoreHandle_t (opaque here)
    void *touchTaskHandle = nullptr; // TaskHandle_t (opaque here)
    TouchCallback callback;
};

extern Touch touch;