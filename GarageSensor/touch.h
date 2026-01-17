#pragma once

#include <Arduino.h>
#include <memory>
#include <functional>
#include "pin_config.h"
#include <TAMC_GT911.h>

// GT911
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

    std::unique_ptr<TAMC_GT911> touch;

    // synchronization and callback
    void *touchSem = nullptr;        // SemaphoreHandle_t (opaque here)
    void *touchTaskHandle = nullptr; // TaskHandle_t (opaque here)
    TouchCallback callback;
};

extern Touch touch;