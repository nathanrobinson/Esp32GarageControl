#include <main.h>
#include <Arduino.h>
#include <Wire.h>
#include "settings.h"
#include "display.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"
#include "pin_config.h"

void setup()
{
  Serial.begin(115200);

  Serial.println("Booting...");

  Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
  delay(100);

  settings.init();
  Serial.println("Settings ready...");

  mqttManager.init();
  Serial.println("MQTT ready...");

  wifiManager.init();
  Serial.println("WiFi ready...");

  display.init();
  Serial.println("Display ready...");
}

void loop()
{
  wifiManager.loop();
  delay(1000);
}
