#include <Arduino.h>
#include "touch.h"
#include "display.h"
#include "HWCDC.h"
#include "settings.h"
#include "wifi_manager.h"
#include "garage.h"
#include "sound.h"

HWCDC USBSerial;

#define TOUCH_COOLDOWN_TIME_MS 750UL

void setup()
{
  USBSerial.begin(115200);

  while (!USBSerial.availableForWrite())
  {
    delay(10);
  }
  Serial.println("Booting...");

  Wire.begin(IIC_SDA, IIC_SCL);

  settings.init();
  Serial.println("Settings imported...");

  touch.init([](const TouchPoint &tp)
             {
              display.enableDisplay(true);
    if (tp.x >= 0 && tp.y >= 0)
    {
      static unsigned long lastCall = 0;
      unsigned long now = millis();
      if ((now - lastCall) >= TOUCH_COOLDOWN_TIME_MS
          && tp.x >= 34 && tp.x <= 334
          && tp.y >= 114 && tp.y <= 414)
      {
        lastCall = now;
        Serial.printf("Touch X:%d Y:%d\n", tp.x, tp.y);
        sound.click();
        garage.buttonPressed();
      }
    } });
  Serial.println("Touch initialized...");

  sound.init();
  Serial.println("ES8311 initialized...");

  display.init();
  Serial.println("Display initialized...");

  wifiManager.init();
  Serial.println("ESP_NOW initialized...");
}

void loop()
{
  wifiManager.loop();
  display.loop();
  delay(500);
}
