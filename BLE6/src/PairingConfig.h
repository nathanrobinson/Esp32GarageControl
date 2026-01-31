#pragma once

#include <zephyr/bluetooth/conn.h>

class PairingConfig
{
public:
    using PairingCompleteCb = void (*)(struct bt_conn *conn);

    static void init(PairingCompleteCb cb);

    /* Callback handlers exposed for C wrappers to call */
    static void on_passkey_entry(struct bt_conn *conn);
    static void on_pairing_confirm(struct bt_conn *conn);
    static void on_cancel(struct bt_conn *conn);
    static void on_pairing_complete(struct bt_conn *conn, bool bonded);
    static void on_pairing_failed(struct bt_conn *conn, enum bt_security_err reason);

private:
    static PairingCompleteCb s_callback;
    /* C wrappers are defined in PairingConfig.cpp with C linkage; no friend
     * declarations are required here to avoid linkage conflicts.
     */
};
