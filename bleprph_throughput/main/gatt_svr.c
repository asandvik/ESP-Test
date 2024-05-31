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
#define THRPT_UUID_DECLARE(uuid16)                                \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0xc7, 0x69, 0x3a, 0x03, 0x08, 0xa1, 0xa7, 0x89,             \
    0x6f, 0x47, 0x26, 0x8c, uuid16, uuid16 >> 8, 0x00, 0x00     \
    )))

/* 0000xxxx-8c26-476f-89a7-a108033a69c6 */
#define MY_UUID_DECLARE(uuid16)                            \
    ((const ble_uuid_t *) (&(ble_uuid128_t) BLE_UUID128_INIT(   \
    0x38, 0xf2, 0x5f, 0xe3, 0x9e, 0x3a, 0x49, 0xbd,             \
    0x64, 0x4a, 0x8d, 0x89, uuid16, uuid16 >> 8, 0x00, 0x00     \
    )))

/* Simpler UUID initialization */
// static const ble_uuid128_t my_gatt_svr_svc_uuid =
// BLE_UUID128_INIT(0x38, 0xf2, 0x5f, 0xe3, 0x9e, 0x3a, 0x49, 0xbd,
//                  0x64, 0x4a, 0x8d, 0x89, 0x29, 0xb9, 0xd2, 0x60);

#define  THRPT_SVC                           0x0001
#define  THRPT_CHR_READ_WRITE                0x0006
#define  THRPT_CHR_NOTIFY                    0x000a
#define  THRPT_LONG_CHR_READ_WRITE           0x000b

// New
#define  CMD_SVC_ID 0x00A0
#define  CMD_WRITE_ID 0x00A1
#define  CMD_READ_ID 0x00A2

#define READ_THROUGHPUT_PAYLOAD            500
#define WRITE_THROUGHPUT_PAYLOAD           500

// New
#define BUFF_SIZE 500

static const char *tag = "bleprph_throughput";
static uint16_t num_cmd_writes = 0;
static uint16_t num_cmd_reads = 0;

static uint8_t gatt_svr_thrpt_static_long_val[READ_THROUGHPUT_PAYLOAD];
static uint8_t gatt_svr_thrpt_static_short_val[WRITE_THROUGHPUT_PAYLOAD];
uint16_t notify_handle;

// New
static char cmd_output_buffer[BUFF_SIZE];
uint16_t cmd_notify_handle;

static int
gatt_svr_read_write_long_test(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg);

static int
cmd_svc_write(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg);

static int
cmd_svc_read(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg);                                                            

static const struct ble_gatt_svc_def gatts_test_svcs[] = {
    {
        /*** Service: THRPT test. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = THRPT_UUID_DECLARE(THRPT_SVC),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                .uuid = THRPT_UUID_DECLARE(THRPT_CHR_READ_WRITE),
                .access_cb = gatt_svr_read_write_long_test,
                .flags = BLE_GATT_CHR_F_READ |
                BLE_GATT_CHR_F_WRITE,
            }, {
                .uuid = THRPT_UUID_DECLARE(THRPT_CHR_NOTIFY),
                .access_cb = gatt_svr_read_write_long_test,
                .val_handle = &notify_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                .uuid = THRPT_UUID_DECLARE(THRPT_LONG_CHR_READ_WRITE),
                .access_cb = gatt_svr_read_write_long_test,
                .flags = BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        /*** My new command service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = MY_UUID_DECLARE(CMD_SVC_ID),
        .characteristics = (struct ble_gatt_chr_def[])
        {   {
                .uuid = MY_UUID_DECLARE(CMD_WRITE_ID),
                .access_cb = cmd_svc_write,
                .flags = BLE_GATT_CHR_F_WRITE,
            }, {
                .uuid = MY_UUID_DECLARE(CMD_READ_ID),
                .access_cb = cmd_svc_read,
                .val_handle = &cmd_notify_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
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

    uuid16 = univ_uuid128_to_local_uuid16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
    case THRPT_LONG_CHR_READ_WRITE:
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
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;

    case THRPT_CHR_READ_WRITE:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = gatt_svr_chr_write(conn_handle, attr_handle,
                                    ctxt->om, 0,
                                    sizeof gatt_svr_thrpt_static_short_val,
                                    gatt_svr_thrpt_static_short_val, NULL);
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            rc = os_mbuf_append(ctxt->om, gatt_svr_thrpt_static_short_val,
                                sizeof gatt_svr_thrpt_static_short_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int
cmd_svc_write(uint16_t conn_handle, uint16_t attr_handle,
              struct ble_gatt_access_ctxt *ctxt,
              void *arg)
{
    uint16_t uuid16;
    int status = 0;

    uuid16 = univ_uuid128_to_local_uuid16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }

    num_cmd_writes += 1;

    ble_gatts_chr_updated(cmd_notify_handle);

    // status = gatt_svr_chr_write(conn_handle, attr_handle,
    //                             ctxt->om, 0,
    //                             sizeof cmd_input_buffer,
    //                             &cmd_input_buffer, NULL);

    return status;
}

static int
cmd_svc_read(uint16_t conn_handle, uint16_t attr_handle,
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

    num_cmd_reads += 1;

    snprintf(cmd_output_buffer, BUFF_SIZE-1,
            "Num writes = %d\nNum reads = %d", num_cmd_writes, num_cmd_reads);

    status = os_mbuf_append(ctxt->om, cmd_output_buffer, sizeof cmd_output_buffer);

    return status == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

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
