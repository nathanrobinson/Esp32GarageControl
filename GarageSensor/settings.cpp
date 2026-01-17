#include "settings.h"

// Define the global Settings instance declared as `extern` in settings.h
Settings settings;

Settings::Settings()
{
    // defaults
    ssid = "";
    wifi_password = "";
    esp_now_passwd = "";
    garage_mac = "";
    driveway_mac = "";
    car_1_mac = "";
    car_2_mac = "";
    mqtt_server = "";
    mqtt_user = "";
    mqtt_password = "";
    display_brightness = 80;
    volume = 80;
}

static String trim(const String &s)
{
    String t = s;
    // remove leading/trailing whitespace
    while (t.length() && isspace((unsigned char)t.charAt(0)))
        t.remove(0, 1);
    while (t.length() && isspace((unsigned char)t.charAt(t.length() - 1)))
        t.remove(t.length() - 1, 1);
    return t;
}

void Settings::init()
{
    if (!LittleFS.begin())
    {
        Serial.println("LittleFS mount failed");
        while (1)
            ;
    }

    File f = LittleFS.open(Settings::SETTINGS_PATH, "r");
    if (!f)
        return;

    while (f.available())
    {
        String line = f.readStringUntil('\n');
        line = trim(line);
        if (line.length() == 0)
            continue;
        if (line.charAt(0) == '#')
            continue; // comment
        int eq = line.indexOf('=');
        if (eq <= 0)
            continue;
        String key = trim(line.substring(0, eq));
        String val = trim(line.substring(eq + 1));

        if (key == "esp_now_passwd")
            esp_now_passwd = val;
        else if (key == "mqtt_server")
            mqtt_server = val;
        else if (key == "mqtt_user")
            mqtt_user = val;
        else if (key == "mqtt_password")
            mqtt_password = val;
        else if (key == "display_brightness")
            display_brightness = val.toInt();
        else if (key == "garage_mac")
            garage_mac = val;
        else if (key == "driveway_mac")
            driveway_mac = val;
        else if (key == "car_1_mac")
            car_1_mac = val;
        else if (key == "car_2_mac")
            car_2_mac = val;
        else if (key == "volume")
            volume = val.toInt();
        else if (key == "ssid")
            ssid = val;
        else if (key == "wifi_password")
            wifi_password = val;
    }

    f.close();
}

String Settings::SSID()
{
    return ssid;
}

String Settings::WifiPassword()
{
    return wifi_password;
}

String Settings::EspNowPassword()
{
    return esp_now_passwd;
}

String Settings::MqttServer()
{
    return mqtt_server;
}

String Settings::MqttUser()
{
    return mqtt_user;
}

String Settings::MqttPassword()
{
    return mqtt_password;
}

String Settings::GarageMAC()
{
    return garage_mac;
}

String Settings::DrivewayMAC()
{
    return driveway_mac;
}

String Settings::Car1MAC()
{
    return car_1_mac;
}

String Settings::Car2MAC()
{
    return car_2_mac;
}

int Settings::DisplayBrightness()
{
    return display_brightness > 10 ? display_brightness : 50;
}

int Settings::Volume()
{
    return volume > 10 ? volume : 50;
}
