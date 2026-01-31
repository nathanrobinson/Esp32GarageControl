#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>
#include <zephyr/settings/settings.h>

#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#include "ble_reflector.h"
#include "../../src/PairingConfig.h"

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_config, 0, 1);

static struct bt_conn *connection;

/* Thread to configure default Channel Sounding settings after connection */
K_THREAD_STACK_DEFINE(cs_stack, 1024);

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    (void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected to %s (err 0x%02X)", addr, err);

    if (err)
    {
        bt_conn_unref(conn);
        connection = NULL;
    }
    else
    {
        connection = bt_conn_ref(conn);

        k_sem_give(&sem_connected);
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02X)", reason);

    bt_conn_unref(conn);
    connection = NULL;

    sys_reboot(SYS_REBOOT_COLD);
}

static void remote_capabilities_cb(struct bt_conn *conn,
                                   uint8_t status,
                                   struct bt_conn_le_cs_capabilities *params)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(params);

    if (status == BT_HCI_ERR_SUCCESS)
    {
        LOG_INF("CS capability exchange completed.");
    }
    else
    {
        LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
    }
}

static void config_create_cb(struct bt_conn *conn, uint8_t status, struct bt_conn_le_cs_config *config)
{
    ARG_UNUSED(conn);

    if (status == BT_HCI_ERR_SUCCESS)
    {
        const char *mode_str[5] = {"Unused", "1 (RTT)", "2 (PBR)", "3 (RTT + PBR)", "Invalid"};
        const char *role_str[3] = {"Initiator", "Reflector", "Invalid"};
        const char *rtt_type_str[8] = {
            "AA only", "32-bit sounding", "96-bit sounding", "32-bit random",
            "64-bit random", "96-bit random", "128-bit random", "Invalid"};
        const char *phy_str[4] = {"Invalid", "LE 1M PHY", "LE 2M PHY", "LE 2M 2BT PHY"};
        const char *chsel_type_str[3] = {"Algorithm #3b", "Algorithm #3c", "Invalid"};
        const char *ch3c_shape_str[3] = {"Hat shape", "X shape", "Invalid"};

        uint8_t mode_idx = config->mode > 0 && config->mode < 4 ? config->mode : 4;
        uint8_t role_idx = MIN(config->role, 2);
        uint8_t rtt_type_idx = MIN(config->rtt_type, 7);
        uint8_t phy_idx = config->cs_sync_phy > 0 && config->cs_sync_phy < 4
                              ? config->cs_sync_phy
                              : 0;
        uint8_t chsel_type_idx = MIN(config->channel_selection_type, 2);
        uint8_t ch3c_shape_idx = MIN(config->ch3c_shape, 2);

        LOG_INF("CS config creation complete.\r\n"
                " - id: %u\r\n"
                " - mode: %s\r\n"
                " - min_main_mode_steps: %u\r\n"
                " - max_main_mode_steps: %u\r\n"
                " - main_mode_repetition: %u\r\n"
                " - mode_0_steps: %u\r\n"
                " - role: %s\r\n"
                " - rtt_type: %s\r\n"
                " - cs_sync_phy: %s\r\n"
                " - channel_map_repetition: %u\r\n"
                " - channel_selection_type: %s\r\n"
                " - ch3c_shape: %s\r\n"
                " - ch3c_jump: %u\r\n"
                " - t_ip1_time_us: %u\r\n"
                " - t_ip2_time_us: %u\r\n"
                " - t_fcs_time_us: %u\r\n"
                " - t_pm_time_us: %u\r\n"
                " - channel_map: 0x%08X%08X%04X\r\n",
                config->id, mode_str[mode_idx],
                config->min_main_mode_steps, config->max_main_mode_steps,
                config->main_mode_repetition, config->mode_0_steps, role_str[role_idx],
                rtt_type_str[rtt_type_idx], phy_str[phy_idx],
                config->channel_map_repetition, chsel_type_str[chsel_type_idx],
                ch3c_shape_str[ch3c_shape_idx], config->ch3c_jump, config->t_ip1_time_us,
                config->t_ip2_time_us, config->t_fcs_time_us, config->t_pm_time_us,
                sys_get_le32(&config->channel_map[6]),
                sys_get_le32(&config->channel_map[2]),
                sys_get_le16(&config->channel_map[0]));

        k_sem_give(&sem_config);
    }
    else
    {
        LOG_WRN("CS config creation failed. (HCI status 0x%02x)", status);
    }
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
    ARG_UNUSED(conn);

    if (status == BT_HCI_ERR_SUCCESS)
    {
        LOG_INF("CS security enabled.");
    }
    else
    {
        LOG_WRN("CS security enable failed. (HCI status 0x%02x)", status);
    }
}

static void procedure_enable_cb(struct bt_conn *conn,
                                uint8_t status,
                                struct bt_conn_le_cs_procedure_enable_complete *params)
{
    ARG_UNUSED(conn);

    if (status == BT_HCI_ERR_SUCCESS)
    {
        if (params->state == 1)
        {
            LOG_INF("CS procedures enabled:\r\n"
                    " - config ID: %u\r\n"
                    " - antenna configuration index: %u\r\n"
                    " - TX power: %d dbm\r\n"
                    " - subevent length: %u us\r\n"
                    " - subevents per event: %u\r\n"
                    " - subevent interval: %u\r\n"
                    " - event interval: %u\r\n"
                    " - procedure interval: %u\r\n"
                    " - procedure count: %u\r\n"
                    " - maximum procedure length: %u",
                    params->config_id, params->tone_antenna_config_selection,
                    params->selected_tx_power, params->subevent_len,
                    params->subevents_per_event, params->subevent_interval,
                    params->event_interval, params->procedure_interval,
                    params->procedure_count, params->max_procedure_len);
        }
        else
        {
            LOG_INF("CS procedures disabled.");
        }
    }
    else
    {
        LOG_WRN("CS procedures enable failed. (HCI status 0x%02x)", status);
    }
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
    .le_cs_config_complete = config_create_cb,
    .le_cs_security_enable_complete = security_enable_cb,
    .le_cs_procedure_enable_complete = procedure_enable_cb,
};

static struct k_thread cs_thread_data;
static void cs_thread(void *p1, void *p2, void *p3)
{
    int err;
    while (true)
    {
        k_sem_take(&sem_connected, K_FOREVER);

        const struct bt_le_cs_set_default_settings_param default_settings = {
            .enable_initiator_role = false,
            .enable_reflector_role = true,
            .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
            .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
        };

        err = bt_le_cs_set_default_settings(connection, &default_settings);
        if (err)
        {
            LOG_ERR("Failed to configure default CS settings (err %d)", err);
        }

        k_sem_take(&sem_config, K_FOREVER);

        const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
            .config_id = 0,
            .max_procedure_len = 1000,
            .min_procedure_interval = 1,
            .max_procedure_interval = 100,
            .max_procedure_count = 0,
            .min_subevent_len = 10000,
            .max_subevent_len = 75000,
            .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
            .phy = BT_LE_CS_PROCEDURE_PHY_2M,
            .tx_power_delta = (int8_t)0x80,
            .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
            .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
            .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
        };

        err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
        if (err)
        {
            LOG_ERR("Failed to set procedure parameters (err %d)", err);
            return;
        }
    }
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0xA455)),
};

bool BleReflector::init()
{
    int err;

    LOG_INF("Starting Channel Sounding Reflector Sample");

    /* Configure RF switch - EXPERIMENTAL: Change these values to find correct antenna */
    /* rfsw-pwr (GPIO2.3): HIGH=enable switch, LOW=disable */
    /* rfsw-ctl (GPIO2.5): Controls antenna selection (try HIGH and LOW) */
    const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));
    if (device_is_ready(gpio2))
    {
        gpio_pin_configure(gpio2, 3, GPIO_OUTPUT_ACTIVE); /* rfsw-pwr = HIGH */
        gpio_pin_configure(gpio2, 5, GPIO_OUTPUT_ACTIVE); /* rfsw-ctl = LOW (try changing to ACTIVE) */
        LOG_INF("RF switch: pwr=HIGH, ctl=LOW");
    }

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return false;
    }

    if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
        settings_load();
    }

    PairingConfig::init(NULL);

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return false;
    }

    /* Spawn background thread to configure default CS settings when connected */
    k_thread_create(&cs_thread_data,
                    cs_stack,
                    K_THREAD_STACK_SIZEOF(cs_stack),
                    cs_thread,
                    NULL,
                    NULL,
                    NULL,
                    K_PRIO_PREEMPT(7),
                    0,
                    K_NO_WAIT);

    return true;
}

bool BleReflector::connected()
{
    return connection != NULL;
}