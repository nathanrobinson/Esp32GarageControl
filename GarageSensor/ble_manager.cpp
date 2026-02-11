#include "ble_manager.h"
#include <string.h>
#include <vector>
#include <algorithm>
#include <NimBLEDevice.h>
#include "display.h"
#include "mqtt_manager.h"

static long lastAdvertisementTime = 0;
// Timeouts (ms) for display indication when no beacon seen
static const long BLE_WARN_TIMEOUT_MS = 10000; // show poor after 10s
static const long BLE_TIMEOUT_MS = 50000;      // show off after 50s

class AdvertisedCallbacks : public NimBLEScanCallbacks
{
private:
    bool resetDistance = false;

public:
    AdvertisedCallbacks() {}
    void onResult(const NimBLEAdvertisedDevice *adv) override
    {
        std::string name = adv->getName();
        if (name != "GarageRanger")
        {
            return;
        }

        lastAdvertisementTime = millis();

        // Read raw service data (first byte: car index, next bytes: float distance)
        std::string svc = adv->getServiceData(0);

        // Serial.printf("GarageRanger advertised: %s\n", svc.c_str());
        if (svc.size() < 4)
        {
            return;
        }

        uint8_t carIdx = svc[0] == '1' ? 1 : svc[0] == '2' ? 2
                                                           : 0;

        if (carIdx == 0 && !resetDistance)
        {
            mqttManager.publishCarDistance(MqttManager::GARAGE, MqttManager::CAR1, 666);
            mqttManager.publishCarDistance(MqttManager::GARAGE, MqttManager::CAR2, 666);
            display.updateDistance(Display::CAR1, 666);
            display.updateDistance(Display::CAR2, 666);
            resetDistance = true;
            return;
        }
        resetDistance = false;

        size_t colon = svc.find(':');
        if (colon == std::string::npos)
            return;
        size_t mpos = svc.find('m', colon + 1);
        std::string distStr;
        if (mpos == std::string::npos)
            distStr = svc.substr(colon + 1);
        else
            distStr = svc.substr(colon + 1, mpos - (colon + 1));

        float distanceMeters = static_cast<float>(atof(distStr.c_str()));

        // Serial.printf("GarageRanger sent %u -> %.2fm\n", carIdx, distanceMeters);

        // Map numeric car index to MqttManager::CarIndex enum when possible
        MqttManager::CarIndex cindex;
        if (carIdx == 1)
            cindex = MqttManager::CAR1;
        else if (carIdx == 2)
            cindex = MqttManager::CAR2;
        else
            return;

        mqttManager.publishCarDistance(MqttManager::GARAGE, cindex, distanceMeters);

        // Map numeric car index to MqttManager::CarIndex enum when possible
        Display::SCAN_LOCATION location;
        if (carIdx == 1)
            location = Display::SCAN_LOCATION::CAR1;
        else if (carIdx == 2)
            location = Display::SCAN_LOCATION::CAR2;
        else
            return;

        display.updateDistance(location, distanceMeters);

        // Read RSSI and update BLE signal strength on the display
        int rssi = adv->getRSSI();
        Display::SIGNAL_STRENGTH strength = Display::OFF;
        if (rssi >= -50)
            strength = Display::GOOD;
        else if (rssi >= -65)
            strength = Display::OK;
        else if (rssi >= -75)
            strength = Display::WEAK;
        else if (rssi >= -90)
            strength = Display::POOR;

        display.updateConnection(Display::BLE, strength);
    }
};

BleManager bleManager;

BleManager::BleManager() {}

void BleManager::taskEntry(void *param)
{
    BleManager *self = reinterpret_cast<BleManager *>(param);
    if (self)
    {
        self->taskLoop();
    }
    vTaskDelete(NULL);
}

void BleManager::init()
{
    if (_running)
        return;

    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    xTaskCreatePinnedToCore(BleManager::taskEntry, "ble_manager", 4096, this, 1, NULL, 1);
    _running = true;
}

static uint8_t scanLoopCount = 0;
void BleManager::taskLoop()
{
    NimBLEScan *pScan = NimBLEDevice::getScan();
    // Use passive scans and short bursts so WiFi/ESP-NOW can share the radio.
    pScan->setScanCallbacks(new AdvertisedCallbacks(), false);
    pScan->setActiveScan(true); // passive: do not send scan requests
    // Keep scanning interval/window small to reduce BLE radio duty cycle
    pScan->setInterval(160);
    pScan->setWindow(48);

    while (true)
    {
        long diff = millis() - lastAdvertisementTime;
        if (!isTimeout && diff > BLE_TIMEOUT_MS)
        {
            Serial.println("No response from GarageRanger");
            display.updateConnection(Display::SCAN_LOCATION::BLE, Display::SIGNAL_STRENGTH::OFF);
            isTimeout = true;
        }
        else if (!isWarned && diff > BLE_WARN_TIMEOUT_MS)
        {
            Serial.println("Still no response from GarageRanger");
            display.updateConnection(Display::SCAN_LOCATION::BLE, Display::SIGNAL_STRENGTH::POOR);
            isWarned = true;
        }
        else if (diff < BLE_WARN_TIMEOUT_MS)
        {
            isTimeout = false;
            isWarned = false;
        }
        // Run a short passive scan burst (100ms), then yield the radio to WiFi/ESP-NOW.
        // The duration parameter is in ms not s.
        if (scanLoopCount++ > SCAN_LOOPS)
        {
            scanLoopCount = 0;
            pScan->clearResults();
            pScan->start(100, false, true);
        }
        else
        {
            pScan->start(100, true, false);
        }
        // Allow at least ~.25s of free time for WiFi/ESP-NOW activity
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
