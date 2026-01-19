#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "settings.h"
#include "garage.h"
#include "TaskLoop.h"

#define WIFI_UPDATE_INTERVAL_SECONDS 90

class WifiManager
{
public:
    WifiManager();
    ~WifiManager();
    void init();                               // initialize WiFi and ESP-NOW
    void loop();                               // call from main loop to handle periodic tasks
    void sendGarageState(Garage::State state); // send gs:# message to peers

private:
    struct PeerEvent
    {
        uint8_t idx;
        int16_t rssi;
        bool anonymous;
    };
    struct MessageEvent
    {
        uint8_t idx;
        char data[16]; // rs:1:-666
    };
    void beginEspNow();
    bool addPeer(const uint8_t mac[6]);
    void sendToPeers(const char *msg, int n);
    void queueProcessMessagePayload(const uint8_t mac[6], const uint8_t *data, int len);
    void processMessagePayload(MessageEvent event);
    void processGarageStateMessage(const uint8_t mac[6], const char *buf);
    void processPingMessage(const uint8_t mac[6]);
    void updateRssi(Display::SCAN_LOCATION peer, int rssi, bool anonymous);
    void queuePeerSeen(const uint8_t mac[6], int rssi, bool anonymous);
    void peerSeen(PeerEvent event);
    void sendCurrentGarageStateToPeer(const uint8_t mac[6]);

    static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    static void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
    static void promiscuousCb(void *buf, wifi_promiscuous_pkt_type_t type);

    int8_t getPeerIndex(const uint8_t mac[6]);
    void updateWifi();

    static void commsTaskStatic(void *pv);

    uint8_t car1Mac[6];
    uint8_t car2Mac[6];
    uint8_t *peers[2];

    bool peerAdded[2];
    Display::SIGNAL_STRENGTH lastRssi[3];
    unsigned long lastSeen[2];
    unsigned long lastWifiAttemptMs;
    int lastPublishedRssi;
    TaskLoop peerLoop;
    TaskLoop messageLoop;
    bool espNowStarted;
    bool wifiConnected;
    uint8_t wifiChannel;
    String espNowPassword;
    unsigned long lastWifiUpdate;
};

extern WifiManager wifiManager;
