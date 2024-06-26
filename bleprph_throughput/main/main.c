/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "gatts_sens.h"
#include "../src/ble_hs_hci_priv.h"

#define NOTIFY_THROUGHPUT_PAYLOAD 500
#define MIN_REQUIRED_MBUF         2 /* Assuming payload of 500Bytes and each mbuf can take 292Bytes.  */
#define PREFERRED_MTU_VALUE       512
#define LL_PACKET_TIME            2120
#define LL_PACKET_LENGTH          251
#define MTU_DEF                   512

static const char *tag = "bleprph_speedtest";
static const char *device_name = "nimble_prph";
// static SemaphoreHandle_t notify_sem;
static uint16_t conn_handle;
/* Dummy variable */
// static uint8_t dummy;
static uint8_t gatts_addr_type;

static int gatts_gap_event(struct ble_gap_event *event, void *arg);

/** 
 * Utility function to log an array of bytes.
 */

void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        ESP_LOGI(tag, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    ESP_LOGI(tag, "%02x:%02x:%02x:%02x:%02x:%02x",
             u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    ESP_LOGI(tag, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    ESP_LOGI(tag, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    ESP_LOGI(tag, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    ESP_LOGI(tag, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    ESP_LOGI(tag, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void
gatts_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    /**
     * Could advertise services via UUID, but not really necessary since name is 
     * adverstised. Would be an option if multiple peripherals have the same name.
    */

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(tag, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(gatts_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gatts_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "Error enabling advertisement; rc=%d", rc);
        return;
    }
}

static int
gatts_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        ESP_LOGI(tag, "connection %s; status = %d ",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        rc = ble_att_set_preferred_mtu(PREFERRED_MTU_VALUE);
        if (rc != 0) {
            ESP_LOGE(tag, "Failed to set preferred MTU; rc = %d", rc);
        }

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            gatts_advertise();
        }

        rc = ble_hs_hci_util_set_data_len(event->connect.conn_handle,
                                          LL_PACKET_LENGTH,
                                          LL_PACKET_TIME);
        if (rc != 0) {
            ESP_LOGE(tag, "Set packet length failed");
        }

        conn_handle = event->connect.conn_handle;
        gatts_advertise();
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(tag, "disconnect; reason = %d", event->disconnect.reason);
        
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(tag, "connection updated; status=%d ",
                 event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(tag, "adv complete ");
        gatts_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(tag, "subscribe event; conn_handle=%d attr_handle=%d "
            "reason=%d prevn=%d curn=%d previ=%d curi=%d",
            event->subscribe.conn_handle,
            event->subscribe.attr_handle,
            event->subscribe.reason,
            event->subscribe.prev_notify,
            event->subscribe.cur_notify,
            event->subscribe.prev_indicate,
            event->subscribe.cur_indicate);
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        ESP_LOGI(tag, "BLE_GAP_EVENT_NOTIFY_TX success !!");
        if ((event->notify_tx.status == 0) ||
                (event->notify_tx.status == BLE_HS_EDONE)) {
        } else {
            ESP_LOGE(tag, "BLE_GAP_EVENT_NOTIFY_TX notify tx status = %d", event->notify_tx.status);
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(tag, "mtu update event; conn_handle = %d mtu = %d ",
                 event->mtu.conn_handle,
                 event->mtu.value);
        break;
    }
    return 0;
}

static void
gatts_on_sync(void)
{
    int rc;
    uint8_t addr_val[6] = {0};

    rc = ble_hs_id_infer_auto(0, &gatts_addr_type);
    assert(rc == 0);
    rc = ble_hs_id_copy_addr(gatts_addr_type, addr_val, NULL);
    assert(rc == 0);
    ESP_LOGI(tag, "Device Address: ");
    print_addr(addr_val);
    /* Begin advertising */
    gatts_advertise();
}

static void
gatts_on_reset(int reason)
{
    ESP_LOGE(tag, "Resetting state; reason=%d", reason);
}

void gatts_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");

    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void app_main(void)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize BT controller and nimble stack */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }

    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = gatts_on_sync;
    ble_hs_cfg.reset_cb = gatts_on_reset;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb,
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(gatts_host_task);
} // End of app_main()
