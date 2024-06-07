/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "gatts_sens.h"
#include "esp_log.h"

/* 0000xxxx-8c26-476f-89a7-a108033a69c7 */
#define CONN_TEST_UUID_DECLARE(uuid16)                                \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0xc7, 0x69, 0x3a, 0x03, 0x08, 0xa1, 0xa7, 0x89,             \
    0x6f, 0x47, 0x26, 0x8c, uuid16, uuid16 >> 8, 0x00, 0x00     \
    )))

/* 0000xxxx- */
#define CMD_UUID_DECLARE(uuid16)                            \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0x38, 0xf2, 0x5f, 0xe3, 0x9e, 0x3a, 0x49, 0xbd,             \
    0x64, 0x4a, 0x8d, 0x89, uuid16, uuid16 >> 8, 0x00, 0x00     \
    )))

/* Simpler UUID initialization */
// static const ble_uuid128_t my_gatt_svr_svc_uuid =
// BLE_UUID128_INIT(0x38, 0xf2, 0x5f, 0xe3, 0x9e, 0x3a, 0x49, 0xbd,
//                  0x64, 0x4a, 0x8d, 0x89, 0x29, 0xb9, 0xd2, 0x60);

/* Local 16bit IDs for 128bit UUIDs */
#define  CMD_SVC    0x00A0  // service
#define  CMD_INPUT  0x00A1  // characteristic
#define  CMD_RESULT   0x00A2  // characteristic

#define  CONN_TEST_SVC                  0x00B0 // service
#define  CONN_TEST_CHR_READ_WRITE       0x00B1 // characteristic
#define  CONN_TEST_CHR_NOTIFY           0x00B2 // characteristic
#define  CONN_TEST_LONG_CHR_READ_WRITE  0x00B3 // characteristic

#define READ_THROUGHPUT_PAYLOAD            500
#define WRITE_THROUGHPUT_PAYLOAD           500

// New
#define BUFF_SIZE 500

static const char *tag = "bleprph_speedtest";
static uint16_t num_cmd_input_writes = 0;
static uint16_t num_cmd_input_reads = 0;

static uint8_t gatt_svr_thrpt_static_long_val[READ_THROUGHPUT_PAYLOAD];
static uint8_t gatt_svr_thrpt_static_short_val[WRITE_THROUGHPUT_PAYLOAD];

uint16_t cmd_input_notify_handle;
uint16_t cmd_result_notify_handle;

uint16_t conn_test_notify_handle;

// New
static char cmd_input_buffer[2]; // Byte 1: test type. Byte 2: Delay in seconds
static char cmd_result_buffer[BUFF_SIZE];
static char conn_test_buffer[BUFF_SIZE];

// Service Characteristic Callback Functions
static int
gatt_svr_read_write_long_test(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg);

static int
conn_test_svc_read_write(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg);                         

static int
cmd_svc_input(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg);

static int
cmd_svc_result(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg);                                                          

static const struct ble_gatt_svc_def gatts_test_svcs[] = {
    {
        /*** My new command service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = CMD_UUID_DECLARE(CMD_SVC),
        .characteristics = (struct ble_gatt_chr_def[])
        {   {
                .uuid = CMD_UUID_DECLARE(CMD_INPUT),
                .access_cb = cmd_svc_input,
                .val_handle = &cmd_input_notify_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                    BLE_GATT_CHR_F_NOTIFY, // read and notify for other esp, write for phone
            }, { 
                .uuid = CMD_UUID_DECLARE(CMD_RESULT),
                .access_cb = cmd_svc_result,
                .val_handle = &cmd_result_notify_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, // contains measurements results, only server writes to it. may need to let other esp write to it
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },
    
    {
        /*** Service: CONN_TEST test. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = CONN_TEST_UUID_DECLARE(CONN_TEST_SVC),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                .uuid = CONN_TEST_UUID_DECLARE(CONN_TEST_CHR_READ_WRITE),
                .access_cb = gatt_svr_read_write_long_test,
                .flags = BLE_GATT_CHR_F_READ |
                BLE_GATT_CHR_F_WRITE,
            }, {
                .uuid = CONN_TEST_UUID_DECLARE(CONN_TEST_CHR_NOTIFY),
                .access_cb = gatt_svr_read_write_long_test,
                .val_handle = &conn_test_notify_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                .uuid = CONN_TEST_UUID_DECLARE(CONN_TEST_LONG_CHR_READ_WRITE),
                .access_cb = gatt_svr_read_write_long_test,
                .flags = BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};

static uint16_t
univ_uuid128_to_local_uuid16(const ble_uuid_t *uuid)
{
    const uint8_t *u8ptr;
    uint16_t uuid16;

    u8ptr = BLE_UUID128(uuid)->value;
    uuid16 = u8ptr[12];
    uuid16 |= (uint16_t)u8ptr[13] << 8;
    return uuid16;
}

static int
gatt_svr_chr_write(uint16_t conn_handle, uint16_t attr_handle,
                   struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int
gatt_svr_read_write_long_test(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg)
{
    uint16_t uuid16;
    int rc;

    ESP_LOGI(tag, "gatt_svr_read_write_long_test");

    uuid16 = univ_uuid128_to_local_uuid16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
    case CONN_TEST_LONG_CHR_READ_WRITE:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                    ctxt->om, 0,
                                    sizeof gatt_svr_thrpt_static_long_val,
                                    &gatt_svr_thrpt_static_long_val, NULL);
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            gatt_svr_thrpt_static_long_val[0] = rand();
            rc = os_mbuf_append(ctxt->om, &gatt_svr_thrpt_static_long_val,
                                sizeof gatt_svr_thrpt_static_long_val);
            ESP_LOGI(tag, "long read rc=%d", rc);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;

    case CONN_TEST_CHR_READ_WRITE:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                    ctxt->om, 0,
                                    sizeof gatt_svr_thrpt_static_short_val,
                                    gatt_svr_thrpt_static_short_val, NULL);
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            rc = os_mbuf_append(ctxt->om, gatt_svr_thrpt_static_short_val,
                                sizeof gatt_svr_thrpt_static_short_val);
            ESP_LOGI(tag, "short read rc=%d", rc);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int
conn_test_svc_read_write(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg)
{
    uint16_t uuid16;
    int rc;

    ESP_LOGI(tag, "conn_test_svc_read_write");

    uuid16 = univ_uuid128_to_local_uuid16(ctxt->chr->uuid);
    assert(uuid16 == CONN_TEST_CHR_READ_WRITE); // Should only reach this cb via one characteristic

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                ctxt->om, 0,
                                sizeof conn_test_buffer,
                                &conn_test_buffer, NULL);
        return rc;
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) { 
        gatt_svr_thrpt_static_long_val[0] = rand();
        rc = os_mbuf_append(ctxt->om, &conn_test_buffer,
                            sizeof conn_test_buffer);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

static int
cmd_svc_input(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg)
{
    uint16_t uuid16;
    int rc;

    uuid16 = univ_uuid128_to_local_uuid16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        num_cmd_input_reads += 1;
        rc = os_mbuf_append(ctxt->om, &cmd_input_buffer,
                    sizeof cmd_input_buffer);
        rc = rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        num_cmd_input_writes += 1;
        rc = gatt_svr_chr_write(conn_handle, attr_handle,
                        ctxt->om, 0,
                        sizeof cmd_input_buffer,
                        &cmd_input_buffer, NULL);
        ble_gatts_chr_updated(cmd_input_notify_handle);
    } 
    else { 
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }

    ble_gatts_chr_updated(cmd_result_notify_handle); // tell phone about update
    return rc;
}

static int
cmd_svc_result(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg)
{
    uint16_t uuid16;
    int status;

    uuid16 = univ_uuid128_to_local_uuid16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }

    snprintf(cmd_result_buffer, BUFF_SIZE-1,
            "Num cmd input writes = %d\nNum cmd input reads = %d", num_cmd_input_writes, num_cmd_input_reads);

    status = os_mbuf_append(ctxt->om, cmd_result_buffer, sizeof cmd_result_buffer);

    return status == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Not really necessary. Just for logs
void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(tag, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(tag, "registering characteristic %s with "
                 "def_handle=%d val_handle=%d\n",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(tag, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatts_test_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatts_test_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
