// Lightweight C++ wrappers for Zephyr BLE Channel Sounding (initiator & reflector)
#ifndef BLE_REFLECTOR_H
#define BLE_REFLECTOR_H

#include <zephyr/bluetooth/conn.h>

class BleReflector
{
public:
    bool init();
    bool connected();

private:
};

#endif // BLE_REFLECTOR_H
