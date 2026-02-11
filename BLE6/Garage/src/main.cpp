#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include "../../src/blinker.h"
#include "ble_initiator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 2000
#define ON_TIME_MS 150
#define OFF_TIME_MS 100
#define REBOOT_TIME (60 * 60 * 12) // * SLEEP_TIME_MS = 24 hours.

static uint8_t loopCount = 0;

int main(void)
{
    LOG_INF("[main] Starting application\n");

    LOG_INF("[main] Initializing Blinker...\n");
    Blinker bl;
    bl.blink(2, ON_TIME_MS, OFF_TIME_MS);
    LOG_INF("[main] Blinker initialized\n");

    LOG_INF("[main] Initializing Bluetooth...\n");
    BleInitiator bleInitiator;
    bleInitiator.init();
    LOG_INF("[main] Bluetooth initialized\n");

    LOG_INF("[main] Requesting blink patterns\n");
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

        if (loopCount++ > REBOOT_TIME)
        {
            LOG_INF("[main] Rebooting (cold) after %d loops", loopCount);
            sys_reboot(SYS_REBOOT_COLD);
        }
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
