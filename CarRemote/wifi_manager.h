#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "settings.h"
#include "garage.h"
#include "TaskLoop.h"

#define PEER_COUNT 2

class WifiManager
{
public:
    WifiManager();
    ~WifiManager();

    void init();                               // initialize WiFi and ESP-NOW
    void loop();                               // call from main loop to handle periodic tasks
    void sendGarageState(Garage::State state); // send cg:# message to peers

private:
    struct PeerEvent
    {
        uint8_t idx;
        int16_t rssi;
        uint8_t channel;
    };
    struct MessageEvent
    {
        uint8_t idx;
        char data[5];
    };
    void beginEspNow();
    void ensurePeers();
    bool findAndAddPeer(const uint8_t mac[6], uint8_t channelIndex);
    bool addPeer(const uint8_t mac[6], uint8_t channel);
    void sendHeartbeat();
    void sendToPeers(const char *msg, int n);
    void queueProcessMessagePayload(const uint8_t mac[6], const uint8_t *data, int len);
    void processMessagePayload(MessageEvent event);
    void processGarageStateMessage(const uint8_t mac[6], const char *buf);
    void updateRssi(Display::SCAN_LOCATION peer, int rssi);
    void queuePeerSeen(const uint8_t mac[6], int rssi, uint8_t channel);
    void peerSeen(PeerEvent event);

    static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
    static void promiscuousCb(void *buf, wifi_promiscuous_pkt_type_t type);

    int8_t getPeerIndex(const uint8_t mac[6]);

    static void commsTaskStatic(void *pv);

    uint8_t garageMac[6];
    uint8_t drivewayMac[6];

    // current esp_now channel for each peer (1-13), 0 = unknown/not added
    uint8_t peerChannel[PEER_COUNT];
    uint8_t *peers[PEER_COUNT];

    bool peerAdded[PEER_COUNT];
    Display::SIGNAL_STRENGTH lastRssi[PEER_COUNT];
    unsigned long lastSeen[PEER_COUNT];
    unsigned long lastSendMs;
    TaskLoop peerLoop;
    TaskLoop messageLoop;
    bool espNowStarted;
    String espNowPassword;
};

extern WifiManager wifiManager;
