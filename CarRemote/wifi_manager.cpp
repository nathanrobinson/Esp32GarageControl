#include <stdint.h>
#include "HardwareSerial.h"
#include "wifi_manager.h"
#include "garage.h"
#include "display.h"
#include <esp_wifi.h>

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
    memset(garageMac, 0, 6);
    memset(drivewayMac, 0, 6);
    peerAdded[0] = peerAdded[1] = false;
    lastRssi[0] = lastRssi[1] = Display::SIGNAL_STRENGTH::OFF;
    lastSeen[0] = lastSeen[1] = 0;
    peerChannel[0] = peerChannel[1] = 0;
    lastSendMs = 0;
    espNowStarted = false;
    s_instance = this;

    // point peers[] at our known MAC buffers to avoid uninitialized pointers
    peers[0] = garageMac;
    peers[1] = drivewayMac;
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
    macFromString(settings.GarageMAC(), garageMac);
    macFromString(settings.DrivewayMAC(), drivewayMac);

    Serial.printf("Garage MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  garageMac[0], garageMac[1], garageMac[2], garageMac[3], garageMac[4], garageMac[5]);
    Serial.printf("Driveway MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  drivewayMac[0], drivewayMac[1], drivewayMac[2], drivewayMac[3], drivewayMac[4], drivewayMac[5]);

    // derive 16-byte key from settings password
    espNowPassword = settings.EspNowPassword();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    beginEspNow();

    Serial.println("My MAC: " + WiFi.macAddress());

    // create queues for safe callback-to-task communication
    peerLoop.createQueue(sizeof(PeerEvent), 8);
    messageLoop.createQueue(sizeof(MessageEvent), 8);

    // single comms task to handle both peer and message queues (use the peerLoop to create to save memory)
    peerLoop.createTaskPinnedToCore(WifiManager::commsTaskStatic, "CommsTask", 4096, this, 1, 1);

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
        Serial.printf("esp_now_init failed, retrying\n");
        // try to reinit after small delay
        delay(100);
        esp_now_deinit();
        if (esp_now_init() != ESP_OK)
        {
            Serial.printf("esp_now_init retry failed\n");
            return;
        }
    }

    Serial.printf("esp_now initialized\n");
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(nullptr);
    espNowStarted = true;
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
    // automatic reconnection: if peer was added earlier but we see no recent RSSI, try re-add
    for (int i = 0; i < PEER_COUNT; ++i)
    {
        // if we've never seen a ping, or last ping older than 5s -> timeout
        if (peerAdded[i])
        {
            unsigned long now = millis();
            if (lastSeen[i] == 0 || (now - lastSeen[i]) > 5000UL)
            {
                // Serial.printf("Removing expired peer: %d\n", i);
                updateRssi(i == 0 ? Display::SCAN_LOCATION::GARAGE : Display::SCAN_LOCATION::DRIVEWAY, -666);
            }
        }
    }

    // ensure peers are added (and scanned if necessary)
    ensurePeers();

    // heartbeat twice a second
    unsigned long now = millis();
    if (now - lastSendMs >= 500)
    {
        sendHeartbeat();
        lastSendMs = now;
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

void WifiManager::updateRssi(Display::SCAN_LOCATION peer, int rssi)
{
    Display::SIGNAL_STRENGTH strength = Display::OFF;

    if (rssi >= -50)
    {
        strength = Display::GOOD;
    }
    else if (rssi >= -65)
    {
        strength = Display::OK;
    }
    else if (rssi >= -85)
    {
        strength = Display::WEAK;
    }
    else if (rssi >= -100)
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

void WifiManager::ensurePeers()
{
    // attempt to add peers if missing by scanning channels
    bool anyMissing = false;
    for (int p = 0; p < PEER_COUNT; ++p)
        if (!peerAdded[p])
            anyMissing = true;
    if (!anyMissing)
        return;

    // scan channels until we find any peer; when found, add ALL peers on that channel
    for (uint8_t ch = 1; ch <= 13; ++ch)
    {
        // Serial.printf("ensurePeers: scanning channel %d\n", ch);
        for (int peer = 0; peer < PEER_COUNT; ++peer)
        {
            if (peerAdded[peer])
                continue;

            // Serial.printf("ensurePeers: trying peer %d on channel %d\n", peer, ch);
            if (findAndAddPeer(peers[peer], ch))
            {
                Serial.printf("ensurePeers: found peer %d on channel %d -> adding all peers to that channel\n", peer, ch);
                for (int j = 0; j < PEER_COUNT; ++j)
                {
                    // if peer already added on different channel, move it
                    if (peerAdded[j])
                    {
                        if (peerChannel[j] != ch)
                        {
                            Serial.printf("ensurePeers: moving peer %d from channel %d to %d\n", j, peerChannel[j], ch);
                            esp_now_del_peer(peers[j]);
                            addPeer(peers[j], ch);
                        }
                    }
                    else
                    {
                        addPeer(peers[j], ch);
                    }
                }

                // stop scanning after assigning all peers to this channel
                return;
            }
        }
    }

    bool channelSet = false;
    for (int peer = 0; peer < PEER_COUNT; peer++)
        if (peerAdded[peer])
        {
            if (!channelSet)
            {
                esp_err_t e = esp_wifi_set_channel(peerChannel[peer], WIFI_SECOND_CHAN_NONE);
            }
            Serial.printf("ensurePeers: Added peer %d on channel %d\n", peer, peerChannel[peer]);
        }
}

bool WifiManager::findAndAddPeer(const uint8_t mac[6], uint8_t channel)
{
    uint8_t i = getPeerIndex(mac);
    if (i < 0 || i >= PEER_COUNT)
    {
        return false;
    }

    lastSeen[i] = 0;
    lastRssi[i] = Display::SIGNAL_STRENGTH::OFF;

    // set radio to channel to try
    // Serial.printf("findAndAddPeer: set channel -> %d for peer %d\n", channel, i);
    esp_err_t e = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (e != ESP_OK)
    {
        Serial.printf("findAndAddPeer: esp_wifi_set_channel failed for channel %d err=%d\n", channel, e);
        return false;
    }

    // temporarily try to add peer on this channel (encrypted with derived key)
    // Serial.printf("findAndAddPeer: attempting addPeer on channel %d for peer %d\n", channel, i);
    bool ok = addPeer(mac, channel);
    if (!ok)
    {
        // cleanup if add failed
        esp_now_del_peer((uint8_t *)mac);
        uint8_t i = getPeerIndex(mac);
        peerChannel[i] = 0;
        return false;
    }

    // send a quick heartbeat and wait short time for response (promiscuous cb will populate RSSI)
    const char *msg = "p";
    esp_now_send(mac, (const uint8_t *)msg, strlen(msg) + 1);

    unsigned long start = millis();
    while (millis() - start < 500)
    {
        esp_now_send(mac, (const uint8_t *)msg, strlen(msg) + 1);
        // check if we've captured RSSI for this mac
        // promiscuous callback updates lastRssi
        // promiscuous/peerSeen will set lastSeen[i] when we observe activity
        if (lastRssi[i] != Display::SIGNAL_STRENGTH::OFF)
        {
            Serial.printf("Found peer %d on channel %d\n", i, channel);
            return true;
        }

        delay(50);
    }

    // didn't see response in this channel; remove peer and return false
    esp_now_del_peer((uint8_t *)mac);
    peerChannel[i] = 0;
    peerAdded[i] = false;
    return false;
}

bool WifiManager::addPeer(const uint8_t mac[6], uint8_t channel)
{
    if (channel < 1 || channel > 13)
    {
        return false;
    }

    // ensure the radio is on the same channel as the peer before adding
    esp_err_t setch = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (setch != ESP_OK)
    {
        Serial.printf("esp_wifi_set_channel failed for channel %d, err=%d\n", channel, setch);
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = channel;
    // only enable encryption if a password/key was configured
    peerInfo.encrypt = (espNowPassword.length() > 0) ? true : false;

    // derive 16-byte key from settings password
    uint8_t key[16];
    memset(key, 0, sizeof(key));
    if (espNowPassword.length())
    {
        int n = min((int)espNowPassword.length(), 16);
        memcpy(key, espNowPassword.c_str(), n);
    }
    memcpy(peerInfo.lmk, key, 16);

    const int8_t i = getPeerIndex(mac);
    if (i >= 0 && i < PEER_COUNT)
    {
        esp_err_t res = esp_now_add_peer(&peerInfo);
        if (res != ESP_OK)
        {
            Serial.printf("esp_now_add_peer failed for index %d, err=%d\n", i, res);
            peerAdded[i] = false;
            lastSeen[i] = 0;
            peerChannel[i] = 0;
            return false;
        }
        // record the channel we added this peer on
        peerChannel[i] = channel;
        peerAdded[i] = true;
        lastSeen[i] = millis();
        return true;
    }
    return false;
}

void WifiManager::sendHeartbeat()
{
    const char *msg = "p";
    sendToPeers(msg, 1);
}

void WifiManager::sendGarageState(Garage::State state)
{
    uint8_t s = static_cast<uint8_t>(state);
    char msg[8];
    int n = snprintf(msg, sizeof(msg), "cg:%u", (unsigned)s);
    if (n <= 0)
        return;

    Serial.printf("Sending %s\n", msg);
    // send to both peers if added
    sendToPeers(msg, n);
}

void WifiManager::sendToPeers(const char *msg, int n)
{
    // send to all peers if added
    for (int i = 0; i < PEER_COUNT; ++i)
    {
        if (peerAdded[i] && peerChannel[i] > 0)
        {
            esp_now_send(peers[i], (const uint8_t *)msg, n + 1);
        }
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
        s_instance->queuePeerSeen(mac, recv_info->rx_ctrl->rssi, static_cast<uint8_t>(recv_info->rx_ctrl->channel));
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

    s_instance->queuePeerSeen(src, p->rx_ctrl.rssi, static_cast<uint8_t>(p->rx_ctrl.channel));
}

void WifiManager::queuePeerSeen(const uint8_t mac[6], int rssi, uint8_t channel)
{
    const int8_t i = getPeerIndex(mac);
    if (i >= 0 && i < PEER_COUNT)
    {
        if (peerLoop.queue())
        {
            BaseType_t xHigher = pdFALSE;
            PeerEvent event{static_cast<uint8_t>(i), static_cast<int16_t>(rssi), channel};
            peerLoop.sendFromISR(&event, &xHigher);
            if (xHigher)
                portYIELD_FROM_ISR();
        }
    }
}

void WifiManager::peerSeen(PeerEvent event)
{
    if (event.idx >= 0 && event.idx < PEER_COUNT)
    {
        // If we see any peer on a new channel, move ALL peers to that channel
        if (peerChannel[event.idx] != event.channel)
        {
            Serial.printf("peerSeen: peer %d seen on new channel %d (was %d) - moving all peers\n", event.idx, event.channel, peerChannel[event.idx]);
            for (int j = 0; j < PEER_COUNT; ++j)
            {
                if (peerAdded[j])
                {
                    esp_now_del_peer(peers[j]);
                }
                addPeer(peers[j], event.channel);
            }
        }

        if (!peerAdded[event.idx])
        {
            // ensure it's added
            addPeer(peers[event.idx], event.channel);
        }

        if (peerAdded[event.idx])
        {
            updateRssi(event.idx == 0 ? Display::SCAN_LOCATION::GARAGE : Display::SCAN_LOCATION::DRIVEWAY, event.rssi);
            lastSeen[event.idx] = millis();
        }
    }
}

void WifiManager::queueProcessMessagePayload(const uint8_t mac[6], const uint8_t *data, int len)
{
    if (!data || len <= 0)
        return;

    const int8_t i = getPeerIndex(mac);

    // Serial.printf("Got payload from peer %d : %s\n", i, data);

    if (i >= 0 && i < PEER_COUNT)
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
    if (event.idx < PEER_COUNT)
    {
        if (strncmp(event.data, "gs:", 3) == 0)
        {
            processGarageStateMessage(peers[event.idx], event.data);
        }
    }
}

void WifiManager::processGarageStateMessage(const uint8_t mac[6], const char *buf)
{
    int gs = atoi(buf + 3);
    if (gs < 0 || gs > 3)
        return;

    // Serial.printf("Received garage state command: %d\n", gs);
    garage.setState(static_cast<Garage::State>(gs));
}
