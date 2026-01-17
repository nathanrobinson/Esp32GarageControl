#pragma once

#include <Arduino.h>
#include <LittleFS.h>

class Settings
{
public:
    Settings();

    // Load settings from LittleFS. Returns true on success.
    void init();

    String SSID();
    String WifiPassword();
    String MqttServer();
    String MqttUser();
    String MqttPassword();
    String EspNowPassword();
    String GarageMAC();
    String DrivewayMAC();
    String Car1MAC();
    String Car2MAC();
    int DisplayBrightness();
    int Volume();

private:
    String ssid;
    String wifi_password;
    String esp_now_passwd;
    String garage_mac;
    String driveway_mac;
    String car_1_mac;
    String car_2_mac;
    String mqtt_server;
    String mqtt_user;
    String mqtt_password;
    int display_brightness; // 0-100
    int volume;             // 0-100

    // Path on LittleFS where settings are stored
    static constexpr const char *SETTINGS_PATH = "/settings.txt";
};

extern Settings settings;