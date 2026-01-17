#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

class TaskLoop
{
public:
    TaskLoop() : q(nullptr), t(nullptr) {}
    ~TaskLoop()
    {
        if (t)
        {
            vTaskDelete(t);
            t = nullptr;
        }
        if (q)
        {
            vQueueDelete(q);
            q = nullptr;
        }
    }

    // Create a queue for items of size itemSize and length queueLen
    bool createQueue(size_t itemSize, UBaseType_t queueLen)
    {
        q = xQueueCreate(queueLen, itemSize);
        return q != nullptr;
    }

    // Create a pinned task. func should match FreeRTOS TaskFunction_t signature.
    bool createTaskPinnedToCore(TaskFunction_t func, const char *name, uint32_t stackSize, void *param, UBaseType_t priority, BaseType_t core)
    {
        if (xTaskCreatePinnedToCore(func, name, stackSize, param, priority, &t, core) != pdPASS)
        {
            t = nullptr;
            return false;
        }
        return true;
    }

    QueueHandle_t queue() const { return q; }
    TaskHandle_t task() const { return t; }

    // Queue send helpers
    bool sendFromISR(const void *item, BaseType_t *pxHigher)
    {
        if (!q)
            return false;
        return xQueueSendFromISR(q, item, pxHigher) == pdTRUE;
    }

    bool send(const void *item, TickType_t ticksToWait = 0)
    {
        if (!q)
            return false;
        return xQueueSend(q, item, ticksToWait) == pdTRUE;
    }

private:
    QueueHandle_t q;
    TaskHandle_t t;
};
