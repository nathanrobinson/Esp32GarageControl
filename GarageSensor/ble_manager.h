#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SCAN_LOOPS 10

class BleManager
{
public:
    BleManager();
    void init();
    bool running() const { return _running; }

private:
    static void taskEntry(void *param);
    void taskLoop();

    bool _running = false;
    bool isWarned = false;
    bool isTimeout = false;
};

extern BleManager bleManager;
