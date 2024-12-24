


/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "common.h"
#include "gap.h"
#include "gatt_svc.h"
#include "heart_rate.h"
#include "led.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os.h"
#include "esp_efuse.h"
#include "miniz.h"
#include "esp_mac.h"

/* Library function declarations */
void ble_store_config_init(void);

/* Private function declarations */
static void on_stack_reset(int reason);
static void on_stack_sync(void);
static void nimble_host_config_init(void);
static void nimble_host_task(void *param);

// UART configuration
#define UART_NUM         UART_NUM_1
#define TX_PIN           20
#define RX_PIN           3
#define UART_BUF_SIZE    1024
static const int baud[] = { 0, 4800, 9600, 19200, 38400, 57600, 115200 };
static int baudrate=3;
static unsigned int timeout=600;
#define UART_RX_BUFFER_SIZE 256

// Extern Variables
extern uint16_t conn_handle;
extern uint16_t tx_handle;
extern bool connected;
int holddown = 30;  // 3 Seconds

// Local Variables
char device_id[32] = { 0 };

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}

static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    /* Store host configuration */
    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

// Funktion zur Berechnung der Prüfziffer
uint8_t calculate_checksum(const char *message) {
    uint8_t checksum = 0;
    while (*message) {
        if (*message == '$') {
            message++;  // Ignoriere das Startzeichen
            continue;
        }
        if (*message == '*') {
            break;  // Stoppe, wenn das Endzeichen erreicht wird
        }
        checksum ^= *message;  // XOR-Verknüpfung der Zeichen
        message++;
    }
    return checksum;
}

// Funktion zur Überprüfung der NMEA-Nachricht
bool check_nmea_message(const char *message) {
    // Überprüfe, ob die Nachricht mit '$' beginnt und mit '*' endet
    if (message[0] != '$' || strchr(message, '*') == NULL) {
        return false;
    }
    // Finde die Position des '*' (Prüfziffer)
    char *checksum_pos = strchr(message, '*');
    if (checksum_pos == NULL) {
        return false;
    }
    // Extrahiere die Prüfziffer aus der Nachricht (nach dem '*' kommen zwei Hex-Zeichen)
    uint8_t received_checksum;
    if (sscanf(checksum_pos + 1, "%2hhx", &received_checksum) != 1) {
        return false;
    }
    // Berechne die Prüfziffer aus der Nachricht
    uint8_t calculated_checksum = calculate_checksum(message);
    // Vergleiche die berechnete und empfangene Prüfziffer
    if (calculated_checksum != received_checksum) {
        return false;
    }
    // Wenn alle Prüfungen bestanden sind, ist die Nachricht konsistent
    return true;
}

void next_baud(){
	baudrate++;
	if( baudrate > 6 ){
		baudrate = 1;
	}
	ESP_LOGI(TAG, "next baudrate: %d", baud[baudrate] );
	uart_set_baudrate(UART_NUM, baud[baudrate]);
}

bool autobaud( int len, uint8_t *msg ){
	bool okay = false;
	for( int i=0; i< len; i++ ){
		char c = msg[i];
		if( c =='$' ){
			if( check_nmea_message( (char *)&(msg[i]) ) ){
				okay = true;
			    break;
			}
		}
	}
	// ESP_LOGI(TAG, "autobaud:%d baud, okay:%d", baud[baudrate], okay );
	if( !okay ){
		timeout--;
		if( timeout == 0 ){
			next_baud();
			timeout = 30;
		}
	}else{
		// ESP_LOGI(TAG, "autobaud OKAY baud=%d", baud[baudrate] );
		timeout = 3000;  // 300 seconds -> 5 minutes
	}
	return okay;
}

extern void update_params();

void keep_alive(void *arg) {
	while(1){
		vTaskDelay(pdMS_TO_TICKS(5000));
		update_params();
	}
}

void uart_to_ble_task(void *arg) {
    uint8_t uart_buffer[UART_RX_BUFFER_SIZE];
    int len;
    while (1) {
    	if( connected && holddown ){
    		holddown--;
    		vTaskDelay(pdMS_TO_TICKS(100));
    	}else{
    		// Read from UART
    		len = uart_read_bytes(UART_NUM, uart_buffer, sizeof(uart_buffer), pdMS_TO_TICKS(50));
    		uart_buffer[len] = 0;
    		bool okay = autobaud( len, uart_buffer );
    		if (len > 0 && connected && okay ) {
    			// ESP_LOGI(TAG, "uart bytes read: %d\n data:%s\n", len, uart_buffer );
    			// Send data as BLE notification
    			struct os_mbuf *om = ble_hs_mbuf_from_flat(uart_buffer, len);
    			if (om != NULL) {
    				int rc = ble_gattc_notify_custom(conn_handle, tx_handle, om);
    				if (rc == 0) {
    					ESP_LOGI(TAG, "Sent %d bytes from UART to BLE conn %d\n", len, conn_handle);
    				} else {
    					ESP_LOGI(TAG, "Failed to send notification: %d\n", rc);
    				}
    			}
    		}
    	}
    }
}

// UART setup
void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, RX_PIN, TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Invert UART signals (TX/RX)
    uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);  // RS232 TTL need invert
    uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
}

void app_main(void) {
    /* Local variables */
    int rc;
    esp_err_t ret;

    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations
     */
    uint8_t mac[6];
    unsigned int crc = 0;
    if( esp_read_mac(mac, ESP_MAC_BT) == ESP_OK ) {
    	crc = mz_crc32(0L, mac, 6) % 10000;
    }
    sprintf( device_id, "XCBLE-%04d", crc );

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }
    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }
    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }
    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }
    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    // Start the NimBLE stack   tbc...
    // nimble_port_freertos_init(ble_host_task);

    uart_init(); // initialize serial interface

    /* Start NimBLE host task thread and return */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 15, NULL);
    xTaskCreate(uart_to_ble_task, "UART2BLE Task", 4*1024, NULL, 10, NULL);
    xTaskCreate(keep_alive, "KeepAlive", 4*1024, NULL, 9, NULL);
    return;
}

