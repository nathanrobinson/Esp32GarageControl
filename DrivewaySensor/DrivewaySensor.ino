#include <main.h>
#include <Arduino.h>
#include <Wire.h>
#include "settings.h"
#include "display.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "pin_config.h"

#define REBOOT_TIME (60 * 60 * 24) // every hour

static long loops = 0;

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

  wifiManager.init();
  Serial.println("WiFi ready...");

  mqttManager.init();
  Serial.println("MQTT ready...");
}

void loop()
{
  wifiManager.loop();
  if (loops++ > REBOOT_TIME)
  {
    // reboot to clear out and reset everything
    Serial.println("Rebooting...");
    ESP.restart();
  }
  delay(1000);
}
