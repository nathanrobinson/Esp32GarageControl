#include <Arduino.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include "pin_config.h"
#include "touch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
// GT911

//  Define static instance pointer
Touch *Touch::instance = nullptr;

Touch touch; // global instance

Touch::Touch()
    : touchSem(nullptr), touchTaskHandle(nullptr), callback(nullptr)
{
    instance = this;
}

/// @brief Touch has to be initialized after display
/// @param cb
void Touch::init(TouchCallback cb)
{
    // 1. Reset the touch controller via CH422G
    Wire.beginTransmission(0x38); // CH422G output register
    Wire.write(0x0C);             // Pull Pin 1 (Touch RST) LOW, keep 2 & 3 (LCD) HIGH
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(0x38);
    Wire.write(0x0E); // Pull Pin 1 (Touch RST) HIGH to wake up
    Wire.endTransmission();
    delay(100);

    touch = std::make_unique<TAMC_GT911>(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, ESP_PANEL_BOARD_TOUCH_INT_IO, TP_RST, LCD_WIDTH, LCD_HEIGHT);

    pinMode(ESP_PANEL_BOARD_TOUCH_INT_IO, 1);

    // Attach interrupt to GPIO 4 on the Falling edge
    attachInterrupt(digitalPinToInterrupt(ESP_PANEL_BOARD_TOUCH_INT_IO), IIC_Touch_Interrupt, FALLING);

    // Create a binary semaphore for ISR -> task signaling
    touchSem = (void *)xSemaphoreCreateBinary();
    if (!touchSem)
    {
        Serial.println("Failed to create touch semaphore");
        return;
    }

    // Create the touch handling task (runs separately from main loop)
    BaseType_t ok = xTaskCreatePinnedToCore(touchTaskStatic, "TouchTask", 4096, this, 1, (TaskHandle_t *)&touchTaskHandle, 1);
    if (ok != pdPASS)
    {
        Serial.println("Failed to create touch task");
    }

    callback = cb;

    if (touch)
    {
        touch->begin();
        touch->setRotation(TOUCH_ROTATION_INVERTED);
    }
}

void Touch::IIC_Touch_Interrupt(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (instance && instance->touchSem)
    {
        xSemaphoreGiveFromISR((SemaphoreHandle_t)instance->touchSem, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
    }
}

void Touch::touchTaskStatic(void *pv)
{
    Touch *self = static_cast<Touch *>(pv);
    if (self)
        self->touchTask();
    vTaskDelete(NULL);
}

void Touch::touchTask()
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)touchSem;
    for (;;)
    {
        if (xSemaphoreTake(sem, portMAX_DELAY) == pdTRUE)
        {
            if (!touch || !callback)
                continue;

            touch->read();
            if (touch->isTouched && touch->touches)
            {
                TouchPoint tp{touch->points[0].x, touch->points[0].y};
                callback(tp);
            }
        }
    }
}