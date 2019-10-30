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
#include <math.h>

//globals
int throttle, yaw, pitch, roll;
double accel_mag, accel_theta, accel_phi;
float acclx_cal, accly_cal, acclz_cal;

//i2c and periferal requirements
#include "driver/i2c.h"
static uint32_t i2c_frequency = 400000;
static i2c_port_t i2c_port = I2C_NUM_0;
static gpio_num_t i2c_gpio_sda = 18;
static gpio_num_t i2c_gpio_scl = 19;
#include "../components/i2c.h"

//mac for attached transmitter
static uint8_t s_tx_mac[ESP_NOW_ETH_ALEN] = { 0xde, 0x4f, 0x22, 0x17, 0xf5, 0x6a };
//requirements for espnow
#define ESPNNOW_SEND_COUNT  1000
#define ESPNOW_SEND_DELAY   50
#define ESPNOW_SEND_LEN     200
#include "rom/crc.h"
#include "../components/espnow.h"

//requirements for sbus uart
#define TXD            23
#define RXD            22
#define SBUS_UART      UART_NUM_1
#include "driver/uart.h"
#include "../components/sbus.h"

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();
    vTaskDelay(10);
    espnow_init();  //starts task after init

    //printf("get here 0\n");
    //sbus_init();
    //printf("get here 1\n");
    //vTaskDelay(10);
    //printf("get here 2\n");
    //xTaskCreate (sbus_tx, "sbus_tx_task", 4096, NULL, 5, NULL);

    while (1) {
       //if((( 0.001 * esp_log_timestamp()) - timerold) > 0.5){
       //     throttle = 1000; yaw = 1000; pitch = 1500; roll = 1500;
       //     printf("watchdog disarming\n");
       //}
       vTaskDelay(100);
    }
}
