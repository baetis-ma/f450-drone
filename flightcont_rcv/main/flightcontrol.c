#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"

//globals
int throttle, yaw, pitch, roll;

//requirements for espnow
#define ESPNNOW_SEND_COUNT  1000
#define ESPNOW_SEND_DELAY   100
#define ESPNOW_SEND_LEN     200
static uint8_t s_tx_mac[ESP_NOW_ETH_ALEN] = { 0xde, 0x4f, 0x22, 0x17, 0xf5, 0x6a };
#include "./espnow.h"

//requirements for sbus uart
#define TXD            23
#define RXD            22
#define SBUS_UART      UART_NUM_1
#include "driver/uart.h"
#include "./sbus.h"

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    espnow_init();  //starts task after init

    sbus_init();
    xTaskCreate (sbus_tx, "sbus_tx_task", 4096, NULL, 5, NULL);
}
