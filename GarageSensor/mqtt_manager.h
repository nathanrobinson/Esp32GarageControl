#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "TaskLoop.h"

class MqttManager
{
public:
    MqttManager();
    void init();
    void taskLoop();

    // Publish a control message to /garage/control with value OPEN(2) or CLOSED(0)
    enum TargetState
    {
        CLOSED = 0,
        OPEN = 2
    };
    void publishGarageControl(TargetState state);

    // Publish an RSSI value to /{area}/car/{index}/rssi (area: "garage" or "driveway")
    enum Area
    {
        GARAGE = 0,
        DRIVEWAY = 1
    };
    enum CarIndex
    {
        CAR1 = 1,
        CAR2 = 2
    };
    void publishCarRssi(Area area, CarIndex carIndex, int rssi);
    // Publish a distance value (meters) to /{area}/car/{index}/distance
    void publishCarDistance(Area area, CarIndex carIndex, float distance);
    void publishWifiSsid(Area area, String ssid);
    void publishWifiChannel(Area area, int channel);
    void publishWifiRssi(Area area, int rssi);

private:
    void connectIfNeeded();
    void onMessage(char *topic, byte *payload, unsigned int length);
    static void staticOnMessage(char *topic, byte *payload, unsigned int length);

    unsigned long lastReconnectAttempt = 0;

    // MQTT task + queues for thread-safe operation
    // Topic and payload sizes sized to actual usage in this project.
    // Longest topic currently: "home/driveway/car/2/rssi" (~24 chars).
    // Longest payload currently: numeric strings like "-666".
    struct TopicMessage
    {
        char topic[32];
        char payload[16];
        uint16_t len;
    };

    TaskLoop outLoop;
    TaskLoop inLoop;
    // buffer messages published before the MQTT out queue is created
    static const int EARLY_BUFFER_CAP = 16;
    TopicMessage earlyBuffer[EARLY_BUFFER_CAP];
    int earlyHead = 0;
    int earlyTail = 0;
    int earlyCount = 0;
    bool enqueueEarly(const TopicMessage &m);
    void flushEarlyBufferToOutLoop();
};

extern MqttManager mqttManager;
