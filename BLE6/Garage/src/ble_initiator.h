// Lightweight C++ wrappers for Zephyr BLE Channel Sounding (initiator & reflector)
#ifndef BLE_INITIATOR_H
#define BLE_INITIATOR_H

#include <zephyr/bluetooth/conn.h>
/* Forward declarations for work and DM types used by handlers */
struct k_work;

class BleInitiator
{
public:
    // Initialize the initiator with an active connection for a given index (0 or 1)
    // Returns true on success
    bool init();
    bool connected();

private:
    // Enable and configure radio hardware and Bluetooth stack.
    // Returns true on success.
    bool enableRadio();
    // Initialize and start BLE scanning. Returns true on success.
    bool startScan();
    // configure the RAS and ChannelSounding
    bool configureRasCs();
    bool configureRealtimeRas();
    bool configureChannelSounding();
    bool beginCsProcedure();
};

#endif