/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Channel Sounding Initiator with Ranging Requestor sample
 */

#include <math.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
extern "C"
{
#include <bluetooth/cs_de.h>
}

#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#include "ble_initiator.h"
#include "../../src/PairingConfig.h"

/* GAAT UUIDS
Garage_Ranger service: 7e949dd6-9969-484f-8b39-070d0fed42e1
Connected car: 01cb165c-b836-4c6c-91c1-a820d7ac043a
IFFT: 2151813e-b745-4a24-8812-b856c0d6ea89
Phase Slope: c9688b0d-19e1-4955-ac67-6747be7c50a5
RTT: c21672a2-6200-40f2-84a4-e3518a4e1f72
Best: 1d05d611-1f24-492a-bc7d-d2e8dba3f7eb
Weighted: d8686a9b-102b-45f4-a6cf-b987a66d4260
*/

/* Calibration factors derived from measurements at 0.5m, 1m, 2m, 3m, and 4m
 * Formula: Actual_distance = (Raw_measurement - OFFSET) / SLOPE
 *
 * Weighting strategy based on stability testing:
 * - Phase Slope: Most consistent across range (Weight: 50% - 2:1 ratio)
 * - RTT: Stable but noisier (Weight: 25%)
 * - IFFT: Phase ambiguity issues (Weight: 25%)
 */

/* Phase Slope calibration: Actual = (Phase_raw - 1.307) / 1.786 */
/* Most stable and consistent method - primary distance estimator */
#define PHASE_SLOPE_SLOPE (1.786f)
#define PHASE_SLOPE_OFFSET_M (1.307f)
#define PHASE_WEIGHT (0.4f) /* Highest weight - 2:1 ratio vs others */

/* RTT calibration: Actual = (RTT_raw - 3.200) / 1.800 */
#define RTT_SLOPE (1.800f)
#define RTT_OFFSET_M (3.200f)
#define RTT_WEIGHT (0.1f)

/* IFFT calibration: Complex non-linear, use best-fit approximation */
/* At short range (< 2m): ~1.75m offset, at long range more linear */
#define IFFT_SLOPE (1.600f)    /* Approximate slope for calibration */
#define IFFT_OFFSET_M (1.400f) /* Approximate offset */
#define IFFT_WEIGHT (0.5f)

#define CS_CONFIG_ID 0
#define NUM_MODE_0_STEPS 3
#define PROCEDURE_COUNTER_NONE (-1)
#define DE_SLIDING_WINDOW_SIZE (9)
#define MAX_AP (CONFIG_BT_RAS_MAX_ANTENNA_PATHS)

#define LOCAL_PROCEDURE_MEM                                                     \
    ((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) + \
     (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovery_done, 0, 1);
static K_SEM_DEFINE(sem_mtu_exchange_done, 0, 1);
static K_SEM_DEFINE(sem_security, 0, 1);
static K_SEM_DEFINE(sem_ras_features, 0, 1);
static K_SEM_DEFINE(sem_local_steps, 1, 1);
static K_SEM_DEFINE(sem_distance_estimate_updated, 0, 1);
static K_SEM_DEFINE(sem_cs_pairing_successful, 0, 1);

static K_MUTEX_DEFINE(distance_estimate_buffer_mutex);

static struct bt_conn *connection;
static char connected_name[64] = {0};

NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
static int32_t most_recent_local_ranging_counter = PROCEDURE_COUNTER_NONE;
static int32_t dropped_ranging_counter = PROCEDURE_COUNTER_NONE;
static uint32_t ras_feature_bits;

static uint8_t buffer_index;
static uint8_t buffer_num_valid;
static cs_de_dist_estimates_t distance_estimate_buffer[MAX_AP][DE_SLIDING_WINDOW_SIZE];
static struct bt_conn_le_cs_config cs_config;

/* Thread to configure default Channel Sounding settings after connection */
K_THREAD_STACK_DEFINE(cs_stack, 1024);

/* C-linkage callback to pass into PairingConfig */
static void pairing_complete_enable_cs_c(struct bt_conn *conn)
{
    LOG_INF("[Garage]: Pairing successful\n");
    k_sem_give(&sem_cs_pairing_successful);
}

static void store_distance_estimates(cs_de_report_t *p_report)
{
    int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

    __ASSERT_NO_MSG(lock_state == 0);

    for (uint8_t ap = 0; ap < p_report->n_ap; ap++)
    {
        memcpy(&distance_estimate_buffer[ap][buffer_index],
               &p_report->distance_estimates[ap], sizeof(cs_de_dist_estimates_t));
    }

    buffer_index = (buffer_index + 1) % DE_SLIDING_WINDOW_SIZE;

    if (buffer_num_valid < DE_SLIDING_WINDOW_SIZE)
    {
        buffer_num_valid++;
    }

    k_mutex_unlock(&distance_estimate_buffer_mutex);
}

static int float_cmp(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;

    return (fa > fb) - (fa < fb);
}

static float median_inplace(int count, float *values)
{
    if (count == 0)
    {
        return NAN;
    }

    qsort(values, count, sizeof(float), float_cmp);

    if (count % 2 == 0)
    {
        return (values[count / 2] + values[count / 2 - 1]) / 2;
    }
    else
    {
        return values[count / 2];
    }
}

static cs_de_dist_estimates_t get_distance(uint8_t ap)
{
    cs_de_dist_estimates_t averaged_result = {};
    uint8_t num_ifft = 0;
    uint8_t num_phase_slope = 0;
    uint8_t num_rtt = 0;

    static float temp_ifft[DE_SLIDING_WINDOW_SIZE];
    static float temp_phase_slope[DE_SLIDING_WINDOW_SIZE];
    static float temp_rtt[DE_SLIDING_WINDOW_SIZE];

    int lock_state = k_mutex_lock(&distance_estimate_buffer_mutex, K_FOREVER);

    __ASSERT_NO_MSG(lock_state == 0);

    for (uint8_t i = 0; i < buffer_num_valid; i++)
    {
        if (isfinite(distance_estimate_buffer[ap][i].ifft))
        {
            temp_ifft[num_ifft] = distance_estimate_buffer[ap][i].ifft;
            num_ifft++;
        }
        if (isfinite(distance_estimate_buffer[ap][i].phase_slope))
        {
            temp_phase_slope[num_phase_slope] =
                distance_estimate_buffer[ap][i].phase_slope;
            num_phase_slope++;
        }
        if (isfinite(distance_estimate_buffer[ap][i].rtt))
        {
            temp_rtt[num_rtt] = distance_estimate_buffer[ap][i].rtt;
            num_rtt++;
        }
    }

    k_mutex_unlock(&distance_estimate_buffer_mutex);

    averaged_result.ifft = median_inplace(num_ifft, temp_ifft);
    averaged_result.phase_slope = median_inplace(num_phase_slope, temp_phase_slope);
    averaged_result.rtt = median_inplace(num_rtt, temp_rtt);

    /* Apply calibration: Actual = (Raw - Offset) / Slope
     * Store calibrated values back into the result structure
     */
    float ifft_calibrated = 0.0f;
    float phase_calibrated = 0.0f;
    float rtt_calibrated = 0.0f;

    bool ifft_valid = false;
    bool phase_valid = false;
    bool rtt_valid = false;

    /* Calibrate IFFT - ignore zeros and invalid values */
    if (isfinite(averaged_result.ifft) && averaged_result.ifft > 0.01f)
    {
        ifft_calibrated = (averaged_result.ifft - IFFT_OFFSET_M) / IFFT_SLOPE;
        if (ifft_calibrated < 0.0f)
            ifft_calibrated = 0.0f;
        ifft_valid = true;
        averaged_result.ifft = ifft_calibrated;
    }

    /* Calibrate Phase Slope - ignore zeros and invalid values */
    if (isfinite(averaged_result.phase_slope) && averaged_result.phase_slope > 0.01f)
    {
        phase_calibrated = (averaged_result.phase_slope - PHASE_SLOPE_OFFSET_M) / PHASE_SLOPE_SLOPE;
        if (phase_calibrated < 0.0f)
            phase_calibrated = 0.0f;
        phase_valid = true;
        averaged_result.phase_slope = phase_calibrated;
    }

    /* Calibrate RTT - ignore zeros and invalid values */
    if (isfinite(averaged_result.rtt) && averaged_result.rtt > 0.01f)
    {
        rtt_calibrated = (averaged_result.rtt - RTT_OFFSET_M) / RTT_SLOPE;
        if (rtt_calibrated < 0.0f)
            rtt_calibrated = 0.0f;
        rtt_valid = true;
        averaged_result.rtt = rtt_calibrated;
    }

    return averaged_result;
}

static void ranging_data_cb(struct bt_conn *conn, uint16_t ranging_counter, int err)
{
    ARG_UNUSED(conn);

    if (err)
    {
        // LOG_ERR("Error when receiving ranging data with ranging counter %d (err %d)", ranging_counter, err);
        return;
    }

    if (ranging_counter != most_recent_local_ranging_counter)
    {
        // LOG_INF("Ranging data dropped as peer ranging counter doesn't match local ranging data counter. (peer: %u, local: %u)", ranging_counter, most_recent_local_ranging_counter);
        net_buf_simple_reset(&latest_local_steps);
        k_sem_give(&sem_local_steps);
        return;
    }

    // LOG_DBG("Ranging data received for ranging counter %d", ranging_counter);

    if (latest_local_steps.len == 0)
    {
        // LOG_WRN("All subevents in ranging counter %u were aborted", most_recent_local_ranging_counter);
        net_buf_simple_reset(&latest_local_steps);
        k_sem_give(&sem_local_steps);

        if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD))
        {
            net_buf_simple_reset(&latest_peer_steps);
        }
        return;
    }

    /* This struct is static to avoid putting it on the stack (it's very large) */
    static cs_de_report_t cs_de_report;

    cs_de_populate_report(&latest_local_steps, &latest_peer_steps, &cs_config, &cs_de_report);

    net_buf_simple_reset(&latest_local_steps);

    if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD))
    {
        net_buf_simple_reset(&latest_peer_steps);
    }

    k_sem_give(&sem_local_steps);

    cs_de_quality_t quality = cs_de_calc(&cs_de_report);

    if (quality == CS_DE_QUALITY_OK)
    {
        for (uint8_t ap = 0; ap < cs_de_report.n_ap; ap++)
        {
            if (cs_de_report.tone_quality[ap] == CS_DE_TONE_QUALITY_OK)
            {
                store_distance_estimates(&cs_de_report);
            }
        }
        k_sem_give(&sem_distance_estimate_updated);
    }
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
    if (dropped_ranging_counter == result->header.procedure_counter)
    {
        return;
    }

    if (most_recent_local_ranging_counter != bt_ras_rreq_get_ranging_counter(result->header.procedure_counter))
    {
        int sem_state = k_sem_take(&sem_local_steps, K_NO_WAIT);

        if (sem_state < 0)
        {
            dropped_ranging_counter = result->header.procedure_counter;
            // LOG_INF("Dropped subevent results. Waiting for ranging data from peer.");
            return;
        }

        most_recent_local_ranging_counter =
            bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
    }

    if (result->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED)
    {
        /* The steps from this subevent will not be used. */
    }
    else if (result->step_data_buf)
    {
        if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps))
        {
            uint16_t len = result->step_data_buf->len;
            uint8_t *step_data = (uint8_t *)net_buf_simple_pull_mem(result->step_data_buf, len);

            net_buf_simple_add_mem(&latest_local_steps, step_data, len);
        }
        else
        {
            LOG_ERR("Not enough memory to store step data. (%d > %d)",
                    latest_local_steps.len + result->step_data_buf->len,
                    latest_local_steps.size);
            net_buf_simple_reset(&latest_local_steps);
            dropped_ranging_counter = result->header.procedure_counter;
            return;
        }
    }

    dropped_ranging_counter = PROCEDURE_COUNTER_NONE;

    if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE)
    {
        most_recent_local_ranging_counter =
            bt_ras_rreq_get_ranging_counter(result->header.procedure_counter);
    }
    else if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED)
    {
        LOG_WRN("Procedure %u aborted", result->header.procedure_counter);
        net_buf_simple_reset(&latest_local_steps);
        k_sem_give(&sem_local_steps);
    }
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
    LOG_DBG("Ranging data ready %i", ranging_counter);

    if (ranging_counter == most_recent_local_ranging_counter)
    {
        int err = bt_ras_rreq_cp_get_ranging_data(connection, &latest_peer_steps,
                                                  ranging_counter,
                                                  ranging_data_cb);
        if (err)
        {
            LOG_ERR("Get ranging data failed (err %d)", err);
            net_buf_simple_reset(&latest_local_steps);
            net_buf_simple_reset(&latest_peer_steps);
            k_sem_give(&sem_local_steps);
        }
    }
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
    LOG_INF("Ranging data overwritten %i", ranging_counter);
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    if (err)
    {
        LOG_ERR("MTU exchange failed (err %d)", err);
        return;
    }

    LOG_INF("MTU exchange success (%u)", bt_gatt_get_mtu(conn));
    k_sem_give(&sem_mtu_exchange_done);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
    int err;

    LOG_INF("The discovery procedure succeeded");

    struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

    bt_gatt_dm_data_print(dm);

    err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
    if (err)
    {
        LOG_ERR("RAS RREQ alloc init failed (err %d)", err);
    }

    err = bt_gatt_dm_data_release(dm);
    if (err)
    {
        LOG_ERR("Could not release the discovery data (err %d)", err);
    }

    k_sem_give(&sem_discovery_done);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
    LOG_INF("The service could not be found during the discovery, disconnecting");
    bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
    LOG_INF("The discovery procedure failed (err %d)", err);
    bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_service_not_found_cb,
    .error_found = discovery_error_found_cb,
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        LOG_ERR("Security failed: %s level %u err %d %s", addr, level, err,
                bt_security_err_to_str(err));
        return;
    }

    LOG_INF("Security changed: %s level %u", addr, level);
    k_sem_give(&sem_security);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    /* Ignore peer parameter preferences. */
    return false;
}

static void check_bond_cb(const struct bt_bond_info *info, void *user_data)
{
    bt_addr_le_t *dst = (bt_addr_le_t *)user_data;

    if (bt_addr_le_cmp(&info->addr, dst) == 0)
    {
        LOG_INF("[Garage]: Already bonded\n");
        k_sem_give(&sem_cs_pairing_successful);
    }
}

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

        const bt_addr_le_t *addr = bt_conn_get_dst(conn);

        /* Iterate through all bonds to see if this address exists */
        bt_foreach_bond(BT_ID_DEFAULT, check_bond_cb, (void *)addr);
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02X)", reason);

    bt_conn_unref(conn);
    connection = NULL;
    connected_name[0] = 0;

    k_sleep(K_MSEC(500));
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
        k_sem_give(&sem_remote_capabilities_obtained);
    }
    else
    {
        LOG_WRN("CS capability exchange failed. (HCI status 0x%02x)", status);
    }
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
                             struct bt_conn_le_cs_config *config)
{
    ARG_UNUSED(conn);

    if (status == BT_HCI_ERR_SUCCESS)
    {
        cs_config = *config;

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

        k_sem_give(&sem_config_created);
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
        k_sem_give(&sem_cs_security_enabled);
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
                    " - subevents per event: %uv\r\n"
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

void ras_features_read_cb(struct bt_conn *conn, uint32_t feature_bits, int err)
{
    if (err)
    {
        LOG_WRN("Error while reading RAS feature bits (err %d)", err);
    }
    else
    {
        LOG_INF("Read RAS feature bits: 0x%x", feature_bits);
        ras_feature_bits = feature_bits;
    }

    k_sem_give(&sem_ras_features);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match, bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    int err;

    LOG_INF("Connecting failed, restarting scanning");

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err)
    {
        LOG_ERR("Failed to restart scanning (err %i)", err);
        return;
    }
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
    connected_name[0] = 0;

    /* Try to parse advertised name from advertisement data (AD structures).
     * The adv data is a TLV: [len][type][data...]
     */
    if (device_info && device_info->adv_data)
    {
        const uint8_t *adv = device_info->adv_data->data;
        size_t adv_len = device_info->adv_data->len;

        size_t i = 0;
        while (i < adv_len)
        {
            uint8_t field_len = adv[i];
            if (field_len == 0 || (i + field_len) >= adv_len)
            {
                break;
            }
            uint8_t field_type = adv[i + 1];

            if (field_type == BT_DATA_NAME_COMPLETE || field_type == BT_DATA_NAME_SHORTENED)
            {
                size_t nlen = field_len - 1;
                if (nlen >= sizeof(connected_name))
                {
                    nlen = sizeof(connected_name) - 1;
                }
                memcpy(connected_name, &adv[i + 2], nlen);
                connected_name[nlen] = '\0';
                break;
            }

            i += (size_t)field_len + 1U;
        }
    }

    if (connected_name[0] == 'C' && connected_name[1] == 'a' && connected_name[2] == 'r' && connected_name[3] == '_' && (connected_name[4] == '1' || connected_name[4] == '2'))
    {
        LOG_INF("Connecting to advertised name: %s", connected_name);
    }
    else
    {
        LOG_INF("Rejecting name");
        bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static int scan_init(void)
{
    int err;

    static struct bt_le_conn_param scan_conn_param =
        BT_LE_CONN_PARAM_INIT(0x10, 0x10, 0, BT_GAP_MS_TO_CONN_TIMEOUT(4000));

    struct bt_scan_init_param param;
    memset(&param, 0, sizeof(param));
    param.scan_param = NULL;
    param.conn_param = &scan_conn_param;
    param.connect_if_match = 1;

    bt_scan_init(&param);
    bt_scan_cb_register(&scan_cb);

    // err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_DECLARE_16(0xA455));
    if (err)
    {
        LOG_ERR("Scanning filters cannot be set (err %d)", err);
        return err;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err)
    {
        LOG_ERR("Filters cannot be turned on (err %d)", err);
        return err;
    }

    return 0;
}

static struct bt_conn_cb conn_cb;

static struct k_thread cs_thread_data;
static void cs_thread(void *p1, void *p2, void *p3)
{
    while (true)
    {
        k_sem_take(&sem_distance_estimate_updated, K_FOREVER);
        if (buffer_num_valid != 0)
        {
            for (uint8_t ap = 0; ap < MAX_AP; ap++)
            {
                cs_de_dist_estimates_t distance_on_ap = get_distance(ap);
                uint32_t timestamp = k_uptime_get_32();

                /* Calculate weighted average, ignoring zero/invalid values */
                float weighted_sum = 0.0f;
                float weight_sum = 0.0f;

                if (distance_on_ap.ifft > 0.01f)
                {
                    weighted_sum += distance_on_ap.ifft * IFFT_WEIGHT;
                    weight_sum += IFFT_WEIGHT;
                }
                if (distance_on_ap.phase_slope > 0.01f)
                {
                    weighted_sum += distance_on_ap.phase_slope * PHASE_WEIGHT;
                    weight_sum += PHASE_WEIGHT;
                }
                if (distance_on_ap.rtt > 0.01f)
                {
                    weighted_sum += distance_on_ap.rtt * RTT_WEIGHT;
                    weight_sum += RTT_WEIGHT;
                }

                float best_estimate = 0.0f;
                if (weight_sum > 0.01f)
                {
                    best_estimate = weighted_sum / weight_sum;
                }

                /* Log individual calibrated values and weighted average */
                printk("[RANGE] %s: [AP%u] IFFT:%.2fm Phase:%.2fm RTT:%.2fm Best:%.2fm → Weighted:%.2fm\r",
                       connected_name,
                       ap,
                       (double)distance_on_ap.ifft,
                       (double)distance_on_ap.phase_slope,
                       (double)distance_on_ap.rtt,
                       (double)distance_on_ap.best,
                       (double)best_estimate);
            }
        }
        /* Pause to rate-limit logging */
        k_sleep(K_MSEC(250));
    }
}

bool BleInitiator::init()
{
    int err;

    LOG_INF("Starting Channel Sounding Initiator Sample");
    /* Enable radio and initialize Bluetooth stack */
    if (!enableRadio())
    {
        return 0;
    }

    PairingConfig::init(pairing_complete_enable_cs_c);

    if (!startScan())
    {
        return 0;
    }

    k_sem_take(&sem_connected, K_FOREVER);

    err = bt_conn_set_security(connection, BT_SECURITY_L2);
    if (err)
    {
        LOG_ERR("Failed to encrypt connection (err %d)", err);
        /* Short delay to allow logs/events to flush before disconnecting */
        k_sleep(K_MSEC(100));

        /* Disconnect from the remote if encryption failed */
        if (connection)
        {
            bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
        return 0;
    }

    k_sem_take(&sem_security, K_FOREVER);

    k_sem_take(&sem_cs_pairing_successful, K_FOREVER);

    static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

    bt_gatt_exchange_mtu(connection, &mtu_exchange_params);

    k_sem_take(&sem_mtu_exchange_done, K_FOREVER);

    err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
    if (err)
    {
        LOG_ERR("Discovery failed (err %d)", err);
        return 0;
    }

    k_sem_take(&sem_discovery_done, K_FOREVER);

    if(!configureRasCs())
    {
        return 0;
    }

    k_sem_take(&sem_ras_features, K_FOREVER);

    if (!configureRealtimeRas())
    {
        return 0;
    }

    k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

    if (!configureChannelSounding())
    {
        return 0;
    }

    k_sem_take(&sem_config_created, K_FOREVER);

    err = bt_le_cs_security_enable(connection);
    if (err)
    {
        LOG_ERR("Failed to start CS Security (err %d)", err);
        return 0;
    }

    k_sem_take(&sem_cs_security_enabled, K_FOREVER);

    if (!beginCsProcedure())
    {
        return 0;
    }

    LOG_INF("=== Channel Sounding with Weighted Average Fusion ===");
    LOG_INF("Calibrated distance shown for each method + weighted average");
    LOG_INF("Weights: Phase=%.2f, RTT=%.2f, IFFT=%.2f (%.0f:%.0f:%.0f ratio)", PHASE_WEIGHT, RTT_WEIGHT, IFFT_WEIGHT, PHASE_WEIGHT * 10, RTT_WEIGHT * 10, IFFT_WEIGHT * 10);
    LOG_INF("========================================================\r\n");

    /* Spawn background thread to perform periodic distance logging */
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

    return 0;
}

bool BleInitiator::connected()
{
    return connection != NULL;
}

// Configure RF switch and initialize the Bluetooth stack.
// Returns true on success.
bool BleInitiator::enableRadio()
{
    int err;

    /* Configure RF switch BEFORE Bluetooth initialization */
    const struct device *gpio2 = DEVICE_DT_GET(DT_NODELABEL(gpio2));
    if (device_is_ready(gpio2))
    {
        gpio_pin_configure(gpio2, 3, GPIO_OUTPUT_ACTIVE); /* rfsw-pwr = HIGH */
        gpio_pin_configure(gpio2, 5, GPIO_OUTPUT_ACTIVE); /* rfsw-ctl = LOW */
        LOG_INF("RF switch configured for onboard antenna");
    }
    else
    {
        LOG_WRN("GPIO2 not ready, RF switch not configured");
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

    return true;
}

// Wrapper to initialize and start BLE scanning. Returns true on success.
bool BleInitiator::startScan()
{
    int err;

    /* Register connection callbacks at runtime to avoid C-designated
     * initializer ordering issues when building as C++.
     */
    conn_cb.connected = connected_cb;
    conn_cb.disconnected = disconnected_cb;
    conn_cb.le_param_req = le_param_req;
    conn_cb.security_changed = security_changed;
    conn_cb.le_cs_read_remote_capabilities_complete = remote_capabilities_cb;
    conn_cb.le_cs_config_complete = config_create_cb;
    conn_cb.le_cs_security_enable_complete = security_enable_cb;
    conn_cb.le_cs_procedure_enable_complete = procedure_enable_cb;
    conn_cb.le_cs_subevent_data_available = subevent_result_cb;

    bt_conn_cb_register(&conn_cb);

    err = scan_init();
    if (err)
    {
        LOG_ERR("Scan init failed (err %d)", err);
        return false;
    }

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err)
    {
        LOG_ERR("Scanning failed to start (err %i)", err);
        return false;
    }

    return true;
}

bool BleInitiator::configureRasCs()
{
    const struct bt_le_cs_set_default_settings_param default_settings = {
        .enable_initiator_role = true,
        .enable_reflector_role = false,
        .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
        .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
    };

    int err = bt_le_cs_set_default_settings(connection, &default_settings);
    if (err)
    {
        LOG_ERR("Failed to configure default CS settings (err %d)", err);
        return false;
    }

    err = bt_ras_rreq_read_features(connection, ras_features_read_cb);
    if (err)
    {
        LOG_ERR("Could not get RAS features from peer (err %d)", err);
        return false;
    }

    return true;
}

bool BleInitiator::configureRealtimeRas()
{
    int err;

    if (ras_feature_bits & RAS_FEAT_REALTIME_RD)
    {
        err = bt_ras_rreq_realtime_rd_subscribe(connection,
                                                &latest_peer_steps,
                                                ranging_data_cb);
        if (err)
        {
            LOG_ERR("RAS RREQ Real-time ranging data subscribe failed (err %d)", err);
            return false;
        }
    }
    else
    {
        err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
        if (err)
        {
            LOG_ERR("RAS RREQ ranging data overwritten subscribe failed (err %d)", err);
            return false;
        }

        err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
        if (err)
        {
            LOG_ERR("RAS RREQ ranging data ready subscribe failed (err %d)", err);
            return false;
        }

        err = bt_ras_rreq_on_demand_rd_subscribe(connection);
        if (err)
        {
            LOG_ERR("RAS RREQ On-demand ranging data subscribe failed (err %d)", err);
            return false;
        }

        err = bt_ras_rreq_cp_subscribe(connection);
        if (err)
        {
            LOG_ERR("RAS RREQ CP subscribe failed (err %d)", err);
            return false;
        }
    }

    err = bt_le_cs_read_remote_supported_capabilities(connection);
    if (err)
    {
        LOG_ERR("Failed to exchange CS capabilities (err %d)", err);
        return false;
    }

    return true;
}

bool BleInitiator::configureChannelSounding()
{
    struct bt_le_cs_create_config_params config_params = {
        .id = CS_CONFIG_ID,
        .mode = BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1,
        .min_main_mode_steps = 2,
        .max_main_mode_steps = 5,
        .main_mode_repetition = 0,
        .mode_0_steps = NUM_MODE_0_STEPS,
        .role = BT_CONN_LE_CS_ROLE_INITIATOR,
        .rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
        .cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
        .channel_map_repetition = 1,
        .channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
        .ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
        .ch3c_jump = 2,
    };

    bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

    int err = bt_le_cs_create_config(connection, &config_params,
                                 BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
    if (err)
    {
        LOG_ERR("Failed to create CS config (err %d)", err);
        return false;
    }

    return true;
}

bool BleInitiator::beginCsProcedure()
{
    const uint16_t procedure_interval = (uint16_t)((ras_feature_bits & RAS_FEAT_REALTIME_RD) ? 5 : 10);
    
    const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
        .config_id = CS_CONFIG_ID,
        .max_procedure_len = 1000,
        .min_procedure_interval = procedure_interval,
        .max_procedure_interval = procedure_interval,
        .max_procedure_count = 0,
        .min_subevent_len = 16000,
        .max_subevent_len = 16000,
        .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
        .phy = BT_LE_CS_PROCEDURE_PHY_2M,
        .tx_power_delta = (int8_t)0x80,
        .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
        .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
        .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
    };

    int err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
    if (err)
    {
        LOG_ERR("Failed to set procedure parameters (err %d)", err);
        return false;
    }

    struct bt_le_cs_procedure_enable_param params = {
        .config_id = CS_CONFIG_ID,
        .enable = BT_CONN_LE_CS_PROCEDURES_ENABLED,
    };

    err = bt_le_cs_procedure_enable(connection, &params);
    if (err)
    {
        LOG_ERR("Failed to enable CS procedures (err %d)", err);
        return false;
    }

    return true;
}