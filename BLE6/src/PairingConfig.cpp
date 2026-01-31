/* PairingConfig: centralized pairing logic implemented in C++ with C callback wrappers */
#include "PairingConfig.h"
#include "../include/pairing_pin.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>

PairingConfig::PairingCompleteCb PairingConfig::s_callback = nullptr;

/* Forwarders called from C callback wrappers */
void PairingConfig::on_passkey_entry(struct bt_conn *conn)
{
    printk("[INF]: Passkey entry requested, entering shared PIN\n");
    bt_conn_auth_passkey_entry(conn, PAIRING_PIN);
}

void PairingConfig::on_pairing_confirm(struct bt_conn *conn)
{
    printk("[INF]: Auto-confirming pairing\n");
    bt_conn_auth_pairing_confirm(conn);
}

void PairingConfig::on_cancel(struct bt_conn *conn)
{
    printk("[WARN]: Pairing cancelled\n");
}

void PairingConfig::on_pairing_complete(struct bt_conn *conn, bool bonded)
{
    ARG_UNUSED(bonded);
    printk("[INF]: Pairing complete callback received\n");
    if (s_callback)
    {
        s_callback(conn);
    }
}

void PairingConfig::on_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    ARG_UNUSED(conn);
    printk("[WARN]: Pairing failed (%d)\n", reason);
}

/* C wrappers required by Zephyr's C APIs. They forward to the C++ class methods. */
extern "C" void pc_auth_passkey_entry(struct bt_conn *conn)
{
    PairingConfig::on_passkey_entry(conn);
}

extern "C" void pc_auth_pairing_confirm(struct bt_conn *conn)
{
    PairingConfig::on_pairing_confirm(conn);
}

extern "C" void pc_auth_cancel(struct bt_conn *conn)
{
    PairingConfig::on_cancel(conn);
}

extern "C" void pc_auth_info_pairing_complete(struct bt_conn *conn, bool bonded)
{
    PairingConfig::on_pairing_complete(conn, bonded);
}

extern "C" void pc_auth_info_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    PairingConfig::on_pairing_failed(conn, reason);
}

static struct bt_conn_auth_cb s_auth_cb = {};
static struct bt_conn_auth_info_cb s_info_cb = {};

void PairingConfig::init(PairingCompleteCb cb)
{
    s_callback = cb;

    /* Assign function pointers at runtime to avoid designated-initializer
     * order mismatches between Zephyr headers and C++ code.
     */
    s_auth_cb.passkey_entry = pc_auth_passkey_entry;
    s_auth_cb.pairing_confirm = pc_auth_pairing_confirm;
    s_auth_cb.cancel = pc_auth_cancel;

    s_info_cb.pairing_complete = pc_auth_info_pairing_complete;
    s_info_cb.pairing_failed = pc_auth_info_pairing_failed;

    bt_conn_auth_cb_register(&s_auth_cb);
    bt_conn_auth_info_cb_register(&s_info_cb);
}
