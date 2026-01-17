#include <Arduino.h>
#include <Adafruit_XCA9554.h>
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"
#include "touch.h"
#include "HWCDC.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Define static instance pointer
Touch *Touch::instance = nullptr;

Touch touch; // global instance

Touch::Touch()
    : expander(), IIC_Bus(nullptr), FT3168(nullptr), touchSem(nullptr), touchTaskHandle(nullptr), callback(nullptr)
{
    instance = this;
}

void Touch::init(TouchCallback cb)
{
    if (!expander.begin(0x20))
    { // Replace with actual I2C address if different
        Serial.println("Failed to find XCA9554 chip");
        while (1)
            ;
    }
    expander.pinMode(0, OUTPUT);
    expander.pinMode(1, OUTPUT);
    expander.pinMode(2, OUTPUT);
    expander.digitalWrite(0, LOW);
    expander.digitalWrite(1, LOW);
    expander.digitalWrite(2, LOW);
    delay(20);
    expander.digitalWrite(0, HIGH);
    expander.digitalWrite(1, HIGH);
    expander.digitalWrite(2, HIGH);

    IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
    FT3168.reset(new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS,
                                    DRIVEBUS_DEFAULT_VALUE, TP_INT, Touch::IIC_Touch_Interrupt));

    while (FT3168->begin() == false)
    {
        Serial.println("FT3168 initialization fail");
        delay(2000);
    }
    Serial.println("FT3168 initialized");

    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

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
            if (!FT3168 || !callback)
                continue;

            int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
            int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

            TouchPoint tp{touchX, touchY};
            callback(tp);
        }
    }
}