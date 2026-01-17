#include <main.h>
#include "mqtt_manager.h"
#include "settings.h"
#include "garage.h"
#include <WiFi.h>
#include "PubSubClient.h"
#include "IPAddress.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// single global instance
MqttManager mqttManager;

static WiFiClient wifiClient;
static PubSubClient client(wifiClient);

MqttManager::MqttManager()
{
}

bool MqttManager::enqueueEarly(const TopicMessage &m)
{
    if (earlyCount >= EARLY_BUFFER_CAP)
    {
        // buffer full
        return false;
    }
    earlyBuffer[earlyTail] = m;
    earlyTail = (earlyTail + 1) % EARLY_BUFFER_CAP;
    ++earlyCount;
    return true;
}

void MqttManager::flushEarlyBufferToOutLoop()
{
    // move buffered messages into outLoop queue when available
    while (earlyCount > 0 && outLoop.queue())
    {
        TopicMessage msg = earlyBuffer[earlyHead];
        if (xQueueSend(outLoop.queue(), &msg, 0) != pdTRUE)
            break; // stop if queue is full/unavailable
        earlyHead = (earlyHead + 1) % EARLY_BUFFER_CAP;
        --earlyCount;
    }
}

// MQTT background task wrapper
static void mqttTaskStatic(void *pv)
{
    auto self = static_cast<MqttManager *>(pv);
    if (self)
        self->taskLoop();
    vTaskDelete(NULL);
}

///////////////////////////////////////////////
//              Public methods               //
///////////////////////////////////////////////

void MqttManager::init()
{
    const String server = settings.MqttServer();
    if (server.length() == 0)
    {
        Serial.println("MQTT server not configured");
        return;
    }

    client.setCallback(MqttManager::staticOnMessage);

    // configure server from settings; accept IP string or hostname
    IPAddress ip;
    if (ip.fromString(server.c_str()))
    {
        Serial.println("MQTT using IP address");
        client.setServer(ip, 1883);
    }
    else
    {
        client.setServer(server.c_str(), 1883);
    }
    // try to connect immediately if Wi-Fi already up
    connectIfNeeded();

    // create queues
    outLoop.createQueue(sizeof(TopicMessage), 16);
    inLoop.createQueue(sizeof(TopicMessage), 8);

    // start mqtt task (use the outLoop to create to save memory)
    if (outLoop.queue() && inLoop.queue())
    {
        outLoop.createTaskPinnedToCore(mqttTaskStatic, "MqttTask", 4096, this, 1, 1);
    }

    // flush any early buffered publishes
    flushEarlyBufferToOutLoop();
}

void MqttManager::publishGarageControl(TargetState state)
{
    // validate state
    if (!(state == CLOSED || state == OPEN))
        return;

    TopicMessage m;
    memset(&m, 0, sizeof(m));
    strncpy(m.topic, "home/garage/control", sizeof(m.topic) - 1);
    int n = snprintf(m.payload, sizeof(m.payload), "%d", static_cast<int>(state));
    m.len = (uint16_t)max(0, n);

    // enqueue outgoing publish; mqtt task will send
    if (!outLoop.queue())
    {
        // fallback: direct publish
        connectIfNeeded();
        if (!client.connected())
        {
            Serial.println("MQTT client not connected.");
            return;
        }
        client.publish(m.topic, m.payload, m.len);
        return;
    }
    xQueueSend(outLoop.queue(), &m, 0);
}

void MqttManager::publishCarRssi(Area area, CarIndex carIndex, int rssi)
{
    // validate enums
    if (!(area == GARAGE || area == DRIVEWAY))
        return;
    if (!(carIndex == CAR1 || carIndex == CAR2))
        return;
    // validate rssi in expected range: 0 down to -666 (inclusive)
    if (rssi > 0)
        rssi = 0;

    if (rssi < -666)
        rssi = -666;

    TopicMessage m;
    memset(&m, 0, sizeof(m));
    const char *areaStr = (area == GARAGE) ? "garage" : "driveway";
    snprintf(m.topic, sizeof(m.topic), "home/%s/car/%d/rssi", areaStr, static_cast<int>(carIndex));
    int n = snprintf(m.payload, sizeof(m.payload), "%d", rssi);
    m.len = (uint16_t)max(0, n);

    if (!outLoop.queue())
    {
        // queue isn't created yet; buffer for later flush
        if (!enqueueEarly(m))
            Serial.println("MQTT early buffer full, dropping publish.");
        return;
    }
    xQueueSend(outLoop.queue(), &m, 0);
}

void MqttManager::publishWifiSsid(Area area, String ssid)
{
    // validate enums
    if (!(area == GARAGE || area == DRIVEWAY))
        return;

    TopicMessage m;
    memset(&m, 0, sizeof(m));
    const char *areaStr = (area == GARAGE) ? "garage" : "driveway";
    snprintf(m.topic, sizeof(m.topic), "home/%s/wifi/ssid", areaStr);
    // copy up to the payload buffer size (16 chars) from ssid
    size_t cap = min((size_t)ssid.length(), sizeof(m.payload));
    if (cap)
        memcpy(m.payload, ssid.c_str(), cap);
    m.len = (uint16_t)cap;

    if (!outLoop.queue())
    {
        if (!enqueueEarly(m))
            Serial.println("MQTT early buffer full, dropping publish.");
        return;
    }
    xQueueSend(outLoop.queue(), &m, 0);
}

void MqttManager::publishWifiChannel(Area area, int channel)
{
    // validate enums
    if (!(area == GARAGE || area == DRIVEWAY))
        return;
    // validate channel in range 0..13
    if (channel < 0 || channel > 13)
        channel = 0;

    TopicMessage m;
    memset(&m, 0, sizeof(m));
    const char *areaStr = (area == GARAGE) ? "garage" : "driveway";
    snprintf(m.topic, sizeof(m.topic), "home/%s/wifi/channel", areaStr);
    int n = snprintf(m.payload, sizeof(m.payload), "%d", channel);
    m.len = (uint16_t)max(0, n);

    if (!outLoop.queue())
    {
        if (!enqueueEarly(m))
            Serial.println("MQTT early buffer full, dropping publish.");
        return;
    }
    xQueueSend(outLoop.queue(), &m, 0);
}

void MqttManager::publishWifiRssi(Area area, int rssi)
{
    // validate enums
    if (!(area == GARAGE || area == DRIVEWAY))
        return;
    // validate rssi in expected range: 0 down to -666 (inclusive)
    if (rssi > 0)
        rssi = 0;

    if (rssi < -666)
        rssi = -666;

    TopicMessage m;
    memset(&m, 0, sizeof(m));
    const char *areaStr = (area == GARAGE) ? "garage" : "driveway";
    snprintf(m.topic, sizeof(m.topic), "home/%s/wifi/rssi", areaStr);
    int n = snprintf(m.payload, sizeof(m.payload), "%d", rssi);
    m.len = (uint16_t)max(0, n);

    if (!outLoop.queue())
    {
        if (!enqueueEarly(m))
            Serial.println("MQTT early buffer full, dropping publish.");
        return;
    }
    xQueueSend(outLoop.queue(), &m, 0);
}

///////////////////////////////////////////////
//             Private methods               //
///////////////////////////////////////////////

void MqttManager::connectIfNeeded()
{
    if (settings.MqttServer().length() == 0)
    {
        Serial.println("MQTT server not configured");
        return;
    }

    if (client.connected())
        return;

    unsigned long now = millis();
    if (now - lastReconnectAttempt < 5000)
        return; // retry interval

    lastReconnectAttempt = now;

    const String user = settings.MqttUser();
    const String pass = settings.MqttPassword();

    // client id derived from compile-time area and MAC address
    String id;
#if defined(IS_GARAGE)
    id = "garage-";
#elif defined(IS_DRIVEWAY)
    id = "driveway-";
#else
    id = "esp32-";
#endif
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    mac.toUpperCase();
    id += mac;

    Serial.printf("Connecting to MQTT %s as %s\n", settings.MqttServer().c_str(), id.c_str());

    bool ok;
    if (user.length())
        ok = client.connect(id.c_str(), user.c_str(), pass.c_str());
    else
        ok = client.connect(id.c_str());

    if (!ok)
    {
        Serial.printf("MQTT connect failed, rc=%d\n", client.state());
        return;
    }

    Serial.println("MQTT connected");

    // subscribe to /garage/state
    if (!client.subscribe("home/garage/state"))
        Serial.println("MQTT subscribe home/garage/state failed");
}

void MqttManager::staticOnMessage(char *topic, byte *payload, unsigned int length)
{
    // enqueue incoming message for the mqtt task to process
    if (!mqttManager.inLoop.queue())
    {
        // fallback: process directly
        mqttManager.onMessage(topic, payload, length);
        return;
    }

    MqttManager::TopicMessage msg;
    memset(&msg, 0, sizeof(msg));
    strncpy(msg.topic, topic ? topic : "", sizeof(msg.topic) - 1);
    size_t cap = min((size_t)length, sizeof(msg.payload) - 1);
    if (cap)
        memcpy(msg.payload, payload, cap);
    msg.len = (uint16_t)cap;
    // enqueue without blocking; drop if full
    xQueueSend(mqttManager.inLoop.queue(), &msg, 0);
}

void MqttManager::onMessage(char *topic, byte *payload, unsigned int length)
{
    // make a null-terminated copy
    String t(topic);
    String msg;
    msg.reserve(length + 1);
    for (unsigned int i = 0; i < length; ++i)
        msg += (char)payload[i];

    Serial.printf("MQTT recv topic=%s payload=%s\n", t.c_str(), msg.c_str());

    if (t.equals("home/garage/state"))
    {
        // payload expected to be an integer matching Garage::State
        int v = atoi(msg.c_str());
        if (v < 0)
            return;
        if (v > 3)
            return;
        garage.setState(static_cast<Garage::State>(v));
    }
}

void MqttManager::taskLoop()
{
    // MQTT processing task: maintain connection, run client.loop(),
    // process outgoing publishes and incoming messages from callback queue.
    TopicMessage message;
    for (;;)
    {
        connectIfNeeded();

        if (client.connected())
        {
            // ensure any early buffered messages are moved into the out queue
            flushEarlyBufferToOutLoop();

            client.loop();

            // drain outgoing queue (non-blocking)
            while (outLoop.queue() && xQueueReceive(outLoop.queue(), &message, 0) == pdTRUE)
            {
                client.publish(message.topic, (const uint8_t *)message.payload, message.len);
            }

            // process at most one incoming message per loop to avoid locking
            if (inLoop.queue() && xQueueReceive(inLoop.queue(), &message, 0) == pdTRUE)
            {
                // construct args and call onMessage
                onMessage(message.topic, (byte *)message.payload, message.len);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
