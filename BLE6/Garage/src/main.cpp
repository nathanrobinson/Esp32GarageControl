#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "../../src/blinker.h"
#include "ble_initiator.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 2000
#define ON_TIME_MS 150
#define OFF_TIME_MS 100

int main(void)
{
    printk("[main] Starting application\n");

    printk("[main] Initializing Blinker...\n");
    Blinker bl;
    bl.blink(2, ON_TIME_MS, OFF_TIME_MS);
    printk("[main] Blinker initialized\n");

    printk("[main] Initializing Bluetooth...\n");
    BleInitiator bleInitiator;
    bleInitiator.init();
    printk("[main] Bluetooth initialized\n");

    printk("[main] Requesting blink patterns\n");
    while (1)
    {
        if (bleInitiator.connected())
        {
            bl.blink(1, ON_TIME_MS * 2, OFF_TIME_MS);
        }
        else
        {
            bl.blink(2, ON_TIME_MS, OFF_TIME_MS);
        }
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
