#include <main.h>
#include <Arduino.h>
#include <Wire.h>
#include "settings.h"
#include "display.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "touch.h"
#include "pin_config.h"

#define TOUCH_COOLDOWN_TIME_MS 750

void setup()
{
    Serial.begin(115200);

    Serial.println("Booting...");

    Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    delay(100);

    settings.init();
    Serial.println("Settings ready...");

    display.init();
    Serial.println("Display ready...");

    touch.init([](const TouchPoint &tp)
               {
        if (tp.x >= 390 && tp.x <= 790
                && tp.y >= 60 && tp.y <= 460)
        {
            static unsigned long lastCall = 0;
            unsigned long now = millis();
            if ((now - lastCall) >= TOUCH_COOLDOWN_TIME_MS)
            {
                lastCall = now;
                Serial.printf("Touch X:%d Y:%d\n", tp.x, tp.y);
                garage.buttonPressed();
            }
        } });
    Serial.println("Touch ready...");

    wifiManager.init();
    Serial.println("WiFi ready...");

    mqttManager.init();
    Serial.println("MQTT ready...");
}

void loop()
{
    wifiManager.loop();
    delay(1000);
}
