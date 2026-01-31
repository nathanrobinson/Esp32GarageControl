#include "blinker.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <atomic>
#include <limits.h>

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});

/* Thread + stack for blinking */
static K_THREAD_STACK_DEFINE(blinker_stack, 512);
static struct k_thread blinker_thread;
static struct k_sem blinker_sem;

static std::atomic<bool> blinker_cancel{false};
static std::atomic<bool> blinker_ready{false};
static int blinker_number = 0;
static uint32_t blinker_on = 0;
static uint32_t blinker_off = 0;

static void blinker_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1)
    {
        k_sem_take(&blinker_sem, K_FOREVER);
        blinker_cancel.store(false);

        int number = blinker_number;
        uint32_t on = blinker_on;
        uint32_t off = blinker_off;

        int loops = (number <= 0) ? INT_MAX : number;

        for (int i = 0; i < loops; ++i)
        {
            if (blinker_cancel.load())
                break;

            gpio_pin_set_dt(&led, 0);
            if (on)
                k_msleep(on);

            if (blinker_cancel.load())
                break;

            gpio_pin_set_dt(&led, 1);
            if (off)
                k_msleep(off);
        }
    }
}

static void ensure_initialized()
{
    bool expected = false;
    if (blinker_ready.compare_exchange_strong(expected, true))
    {
        if (!gpio_is_ready_dt(&led))
        {
            return;
        }

        gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

        k_sem_init(&blinker_sem, 0, 1);

        k_thread_create(&blinker_thread, blinker_stack, K_THREAD_STACK_SIZEOF(blinker_stack),
                        blinker_thread_entry, NULL, NULL, NULL,
                        K_PRIO_PREEMPT(7), 0, K_NO_WAIT);
    }
}

Blinker::Blinker()
{
    ensure_initialized();
}

Blinker::~Blinker()
{
    /* request cancel and wake thread so it stops its current loop */
    blinker_cancel.store(true);
    k_sem_give(&blinker_sem);
}

void Blinker::blink(int number, uint32_t onTimeMs, uint32_t offTimeMs)
{
    ensure_initialized();

    if (!blinker_ready.load())
        return;

    /* request cancel of any running pattern */
    blinker_cancel.store(true);

    /* update parameters for the thread */
    blinker_number = number;
    blinker_on = onTimeMs;
    blinker_off = offTimeMs;

    /* wake the thread to run the new pattern */
    k_sem_give(&blinker_sem);
}
