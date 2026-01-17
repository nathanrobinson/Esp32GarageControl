#include <main.h>
#include "wifi_manager.h"
#include "garage.h"
#include "display.h"
#include <esp_wifi.h>
#include "mqtt_manager.h"

#define WIFI_RECONNECT_INTERVAL 10000 // ms between reconnect attempts when disconnected

#ifdef IS_GARAGE
#define PUBLISHER MqttManager::GARAGE
#else
#define PUBLISHER MqttManager::DRIVEWAY
#endif

WifiManager wifiManager;

static WifiManager *s_instance = nullptr;

static void macFromString(const String &s, uint8_t out[6])
{
    // accept formats like AA:BB:CC:DD:EE:FF or AABBCCDDEEFF
    String t = s;
    t.replace(":", "");
    t.toUpperCase();
    if (t.length() < 12)
    {
        memset(out, 0, 6);
        return;
    }
    for (int i = 0; i < 6; ++i)
    {
        String sub = t.substring(i * 2, i * 2 + 2);
        out[i] = (uint8_t)strtol(sub.c_str(), nullptr, 16);
    }
}

WifiManager::WifiManager()
{
    memset(car1Mac, 0, 6);
    memset(car2Mac, 0, 6);
    peers[0] = car1Mac;
    peers[1] = car2Mac;
    peerAdded[0] = peerAdded[1] = false;
    lastRssi[0] = lastRssi[1] = lastRssi[2] = Display::OFF;
    lastSeen[0] = lastSeen[1] = 0;
    lastPublishedRssi = -10000;
    espNowStarted = false;
    wifiConnected = false;
    wifiChannel = 0;
    lastWifiAttemptMs = 0;
    s_instance = this;
}

WifiManager::~WifiManager()
{
    // TaskLoop members clean up their own queue and task resources
    if (espNowStarted)
    {
        esp_now_deinit();
        espNowStarted = false;
    }
    esp_wifi_set_promiscuous(false);
}

void WifiManager::init()
{
    // derive macs from settings
    macFromString(settings.Car1MAC(), car1Mac);
    macFromString(settings.Car2MAC(), car2Mac);

    // derive 16-byte key from settings password
    espNowPassword = settings.EspNowPassword();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    beginEspNow();

    // create queues for safe callback-to-task communication
    peerLoop.createQueue(sizeof(PeerEvent), 8);
    messageLoop.createQueue(sizeof(MessageEvent), 8);

    // single comms task to handle both peer and message queues (use the peerLoop to create to save memory)
    peerLoop.createTaskPinnedToCore(WifiManager::commsTaskStatic, "CommsTask", 4096, this, 1, 1);

    // attempt to connect to configured Wi-Fi (WPA2 if password provided)
    String ssid = settings.SSID();
    String pass = settings.WifiPassword();
    wifiConnected = false;
    wifiChannel = 0;
    if (ssid.length())
    {
        Serial.printf("Connecting to WiFi SSID '%s'...\n", ssid.c_str());
        WiFi.begin(ssid.c_str(), pass.length() ? pass.c_str() : nullptr);
        unsigned long start = millis();
        // wait up to 5 seconds for connection (non-blocking overall flow)
        while (millis() - start < 5000 && WiFi.status() != WL_CONNECTED)
            delay(100);

        if (WiFi.status() == WL_CONNECTED)
        {
            updateWifi();
        }
        else
        {
            Serial.println("WiFi connection failed or timed out");
        }
    }

    // enable promiscuous for reliable RSSI capture
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuousCb);
}

void WifiManager::beginEspNow()
{
    if (espNowStarted)
        return;

    if (esp_now_init() != ESP_OK)
    {
        // try to reinit after small delay
        delay(100);
        esp_now_deinit();
        if (esp_now_init() != ESP_OK)
            return;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
    espNowStarted = true;
}

// send callback (avoid NULL callback which triggers SDK error)
void WifiManager::onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
}

void WifiManager::commsTaskStatic(void *pv)
{
    auto self = static_cast<WifiManager *>(pv);
    for (;;)
    {
        // Prefer processing message events quickly; wait up to 200ms
        if (self->messageLoop.queue())
        {
            MessageEvent me;
            if (xQueueReceive(self->messageLoop.queue(), &me, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                self->processMessagePayload(me);
            }
        }

        // Drain any pending peer events without blocking
        if (self->peerLoop.queue())
        {
            PeerEvent pe;
            while (xQueueReceive(self->peerLoop.queue(), &pe, 0) == pdTRUE)
            {
                self->peerSeen(pe);
            }
        }
    }
}

void WifiManager::loop()
{
    // dead peer detection
    for (int i = 0; i < 2; ++i)
    {
        // if we've never seen a ping, or last ping older than 5s -> timeout
        if ((lastSeen[i] == 0 || (millis() - lastSeen[i]) > 5000UL) && lastRssi[i + 1] != Display::SIGNAL_STRENGTH::OFF)
        {
            Display::SCAN_LOCATION location = i == 0 ? Display::SCAN_LOCATION::CAR1 : Display::SCAN_LOCATION::CAR2;
            Serial.printf("Removing expired peer: %d\n", i);
            updateRssi(location, -666, false);
        }
    }

    // Monitor Wi-Fi connection and channel; if it changes, sync ESP-NOW channel
    bool nowConnected = (WiFi.status() == WL_CONNECTED);
    uint8_t nowChannel = nowConnected ? static_cast<uint8_t>(WiFi.channel()) : 0;
    if (nowConnected && (!wifiConnected || nowChannel != wifiChannel))
    {
        updateWifi();
    }
    else if (!nowConnected && wifiConnected)
    {
        // lost Wi-Fi
        wifiConnected = false;
        wifiChannel = 0;
        Serial.println("WiFi disconnected");
        updateRssi(Display::WIFI, -666, false);
    }
    else if (!nowConnected && !wifiConnected)
    {
        // periodically attempt to reconnect to configured Wi-Fi
        String ssid = settings.SSID();
        if (ssid.length())
        {
            unsigned long now = millis();
            if (now - lastWifiAttemptMs >= WIFI_RECONNECT_INTERVAL)
            {
                String pass = settings.WifiPassword();
                Serial.printf("Attempting to reconnect to WiFi SSID '%s'...\n", ssid.c_str());
                WiFi.begin(ssid.c_str(), pass.length() ? pass.c_str() : nullptr);
                lastWifiAttemptMs = now;
            }
        }
    }
}

void WifiManager::updateWifi()
{
    wifiConnected = true;
    wifiChannel = static_cast<uint8_t>(WiFi.channel());
    String mac = WiFi.macAddress();
    String ip = WiFi.localIP().toString();
    String ssid = WiFi.SSID();
    Serial.printf("WiFi %s channel %d IP %s RSSI %d\n", mac.c_str(), wifiChannel, ip.c_str(), WiFi.RSSI());

    esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);

    // Update display with SSID and channel
    display.updateWifi(String(ssid) + " (" + String(wifiChannel) + "):");
    updateRssi(Display::WIFI, WiFi.RSSI(), false);
    mqttManager.publishWifiSsid(PUBLISHER, ssid);
    mqttManager.publishWifiChannel(PUBLISHER, wifiChannel);

    // Update the channel on all connected peers.
    for (int i = 0; i < 2; ++i)
    {
        // remove old peer entry then re-add with new channel
        if (peerAdded[i])
            esp_now_del_peer((uint8_t *)peers[i]);
        addPeer(peers[i]);
    }
}

int8_t WifiManager::getPeerIndex(const uint8_t mac[6])
{
    if (memcmp(mac, peers[0], 6) == 0)
        return 0;
    if (memcmp(mac, peers[1], 6) == 0)
        return 1;

    return -1;
}

void WifiManager::updateRssi(Display::SCAN_LOCATION peer, int rssi, bool anonymous)
{
    if (!anonymous && (peer == Display::CAR1 || peer == Display::CAR2))
    {
        MqttManager::CarIndex car = peer == Display::CAR1 ? MqttManager::CarIndex::CAR1 : MqttManager::CarIndex::CAR2;
        mqttManager.publishCarRssi(PUBLISHER, car, rssi);
    }
    else
    {
        int delta = rssi - lastPublishedRssi;
        if (delta < -5 || delta >= 5)
        {
            lastPublishedRssi = rssi;
            mqttManager.publishWifiRssi(PUBLISHER, rssi);
        }
    }
    Display::SIGNAL_STRENGTH strength = Display::OFF;

    if (rssi > -30)
    {
        strength = Display::GOOD;
    }
    else if (rssi > -55)
    {
        strength = Display::OK;
    }
    else if (rssi > -80)
    {
        strength = Display::WEAK;
    }
    else if (rssi > -100)
    {
        strength = Display::POOR;
    }

    uint8_t index = static_cast<uint8_t>(peer);
    if (lastRssi[index] != strength)
    {
        lastRssi[index] = strength;
        display.updateConnection(peer, strength);
    }
}

bool WifiManager::addPeer(const uint8_t mac[6])
{
    uint8_t channel = (wifiChannel >= 1 || wifiChannel <= 13) ? wifiChannel : 1;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = true;

    uint8_t key[16];
    memset(key, 0, sizeof(key));
    if (espNowPassword.length())
    {
        int n = min((int)espNowPassword.length(), 16);
        memcpy(key, espNowPassword.c_str(), n);
    }
    memcpy(peerInfo.lmk, key, 16);

    const int8_t i = getPeerIndex(mac);
    if (i >= 0)
    {
        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            peerAdded[i] = false;
            lastSeen[i] = 0;
            return false;
        }

        peerAdded[i] = true;
        lastSeen[i] = millis();

        sendCurrentGarageStateToPeer(mac);

        return true;
    }
    return false;
}

void WifiManager::sendCurrentGarageStateToPeer(const uint8_t mac[6])
{
    uint8_t s = static_cast<uint8_t>(garage.getState());
    char msg[8];
    int n = snprintf(msg, sizeof(msg), "gs:%u", (unsigned)s);
    if (n <= 0)
        return;

    esp_now_send(mac, (const uint8_t *)msg, n + 1);
}

void WifiManager::sendGarageState(Garage::State state)
{
    uint8_t s = static_cast<uint8_t>(state);
    char msg[8];
    int n = snprintf(msg, sizeof(msg), "gs:%u", (unsigned)s);
    if (n <= 0)
        return;

    sendToPeers(msg, n);
}

void WifiManager::sendToPeers(const char *msg, int n)
{
    // send to all peers if added
    for (int i = 0; i < 2; ++i)
    {
        if (peerAdded[i])
            esp_now_send(peers[i], (const uint8_t *)msg, n + 1);
    }
}

// static callbacks
void WifiManager::onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    if (!s_instance || !recv_info)
        return;

    uint8_t *mac = recv_info->src_addr;
    if (s_instance && recv_info->rx_ctrl)
    {
        s_instance->queuePeerSeen(mac, recv_info->rx_ctrl->rssi, false);
    }

    // parse payload commands if any
    if (s_instance && data && len > 0)
    {
        s_instance->queueProcessMessagePayload(mac, data, len);
    }
}

void WifiManager::promiscuousCb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (!s_instance)
        return;
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA && type != WIFI_PKT_CTRL && type != WIFI_PKT_MISC)
        return;

    wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
    if (!p)
        return;

    uint8_t *payload = p->payload;

    // Basic IEEE802.11 header: addr2 = source MAC (offset 10 in header struct)
    // wifi_ieee80211_hdr_t layout isn't guaranteed here; use offsets: addr2 starts at payload+10
    if (!payload)
        return;

    // source MAC is at payload + 10
    uint8_t *src = payload + 10;

    if (!src)
        return;

    s_instance->queuePeerSeen(src, p->rx_ctrl.rssi, true);
}
// */

void WifiManager::queuePeerSeen(const uint8_t mac[6], int rssi, bool anonymous)
{
    const int8_t i = getPeerIndex(mac);
    if (i >= 0)
    {
        if (peerLoop.queue())
        {
            BaseType_t xHigher = pdFALSE;
            PeerEvent event{static_cast<uint8_t>(i), static_cast<int16_t>(rssi), anonymous};
            peerLoop.sendFromISR(&event, &xHigher);
            if (xHigher)
                portYIELD_FROM_ISR();
        }
    }
}
void WifiManager::peerSeen(PeerEvent event)
{
    if (event.idx >= 0 && event.idx < 2)
    {
        if (!peerAdded[event.idx])
        {
            addPeer(peers[event.idx]);
        }

        if (peerAdded[event.idx])
        {
            Display::SCAN_LOCATION location = event.idx == 0 ? Display::CAR1 : Display::CAR2;
            updateRssi(location, event.rssi, event.anonymous);
            lastSeen[event.idx] = millis();
        }
    }
}

void WifiManager::queueProcessMessagePayload(const uint8_t mac[6], const uint8_t *data, int len)
{
    if (!data || len <= 0)
        return;

    const int8_t i = getPeerIndex(mac);
    if (i >= 0)
    {
        if (messageLoop.queue())
        {
            BaseType_t xHigher = pdFALSE;
            MessageEvent event;
            event.idx = static_cast<uint8_t>(i);

            // copy to null-terminated buffer
            int cap = min(len, 4);
            memset(event.data, 0, sizeof(event.data));
            memcpy(event.data, data, cap);

            messageLoop.sendFromISR(&event, &xHigher);

            if (xHigher)
                portYIELD_FROM_ISR();
        }
    }
}

void WifiManager::processMessagePayload(MessageEvent event)
{
    if (event.idx >= 0 && event.idx < 2)
    {
        if (!peerAdded[event.idx])
        {
            addPeer(peers[event.idx]);
        }

        if (strncmp(event.data, "cg:", 3) == 0)
        {
            processGarageStateMessage(peers[event.idx], event.data);
        }
        else if (event.data[0] == 'p')
        {
            sendCurrentGarageStateToPeer(peers[event.idx]);
        }
    }
}

void WifiManager::processGarageStateMessage(const uint8_t mac[6], const char *buf)
{
    int gs = atoi(buf + 3);
    if (gs < 0 || gs > 3)
        return;

    Serial.printf("Received control garage command: %d\n", gs);
    garage.control(static_cast<Garage::State>(gs));
}
