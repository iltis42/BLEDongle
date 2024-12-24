/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"
#include "driver/uart.h"
#include "heart_rate.h"
#include "led.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "nimble/ble.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"


/* Private variables */

extern bool connected;

uint16_t attr_handle = 0;
uint16_t tx_handle = 0;
uint16_t rx_handle = 0;

static const ble_uuid128_t service_uuid = BLE_UUID128_INIT( 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E ) ;
static const ble_uuid128_t rx_uuid      = BLE_UUID128_INIT( 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E );
static const ble_uuid128_t tx_uuid      = BLE_UUID128_INIT( 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E );

// static uint8_t tx_value[256];
// static uint8_t rx_value[256];

static int gatt_rx_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        const uint8_t *data = ctxt->om->om_data;
        uint16_t len = ctxt->om->om_len;

        // Send the received BLE data to UART
        uart_write_bytes(UART_NUM_1, (const char *)data, len);

        printf("Received %d bytes from BLE and sent to UART\n", len);
        return 0; // Success
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_tx_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
	ESP_LOGI(TAG, "tx cb handle=%d attr:%d op:%d", conn_handle, attr_handle, ctxt->op );
    return BLE_ATT_ERR_UNLIKELY;
}

// GATT service definition
static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &tx_uuid.u,
                .access_cb = gatt_tx_write_cb, // TX is read-only or notify-only
//                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,   // CHARACTERISTIC_UUID_TX,	BLECharacteristic::PROPERTY_NOTIFY
				.flags = BLE_GATT_CHR_F_NOTIFY,
				.val_handle = &tx_handle, // Capture the TX handle
            },
            {
                .uuid = &rx_uuid.u,
                .access_cb = gatt_rx_write_cb,  // Callback to handle RX writes
                .flags = BLE_GATT_CHR_F_WRITE,  // CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE
				.val_handle = &rx_handle,
            },
            {0}, // End of characteristics
        },
    },
    {0}, // End of services
};


/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

// Subscription state tracking
static struct {
    uint16_t conn_handle;      // Connection handle of the subscribed client
    bool tx_notify_enabled;    // Whether TX notifications are enabled
} uart_subscription_state = { 0 };

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
	 ESP_LOGI( TAG, "gatt_svc_init()");
    /* Local variables */
    int rc = 0;
    // Initialize NimBLE stack
    ble_hs_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_services);
    if (rc != 0) {
    	ESP_LOGI( TAG, "ble_gatts_count_cfg() error");
        return rc;
    }

    /* 1. GATT service initialization */
    ble_svc_gatt_init();
    ESP_LOGI( TAG, "ble_svc_gatt_init()");

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0) {
    	ESP_LOGI( TAG, "ble_gatts_add_svcs() error");
        return rc;
    }

    ESP_LOGI( TAG, "TX Characteristic handle: %d\n", tx_handle);

    return 0;
}
