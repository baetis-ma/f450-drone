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
#define ESPNOW_SEND_DELAY   100
#define ESPNOW_SEND_LEN     200
#include "rom/crc.h"
#include "../components/espnow.h"

//requirements for sbus uart
#define TXD            23
#define RXD            22
#define SBUS_UART      UART_NUM_1
#include "driver/uart.h"
#include "../components/sbus.h"

//4 pole butterworth high pass filter >1Hz
float gyrx[5], gyry[5], gyrz[5];
float tgyrx[5], tgyry[5], tgyrz[5];
float coeff_lp4[5] = {2.072820954e+02, -0.1873794924, 1.0546654059, -2.3139884144, 2.3695130072 };
float coeff_lp1[5] = {7.522343863e+04, -0.7199103273, 3.1159669252, -5.0679983867, 3.6717290892 };
static void accel_state_lp() 
{
    uint8_t regdata[6]; 
    float acclx, accly, acclz;

    i2c_read(0x68, 0x3b, regdata, 6);

    acclx = 256*regdata[0] + regdata[1];
    if (acclx >= 0x8000) { acclx = -1.0*(0xffff - acclx); } //2s comp
    acclx = acclx / 0x4000; //full scale +/- 2g
    acclx = acclx + acclx_cal;

    accly = 256*regdata[2] + regdata[3];
    if (accly >= 0x8000) { accly = -1.0*(0xffff - accly); }
    accly = accly / 0x4000;
    accly = accly + accly_cal;

    acclz = 256*regdata[4] + regdata[5];
    if (acclz >= 0x8000) { acclz = -1.0*(0xffff - acclz); }
    acclz = acclz / 0x4000;
    acclz = acclz + acclz_cal;

    gyrx[0]  = gyrx[1]; gyrx[1] = gyrx[2]; gyrx[2] = gyrx[3]; gyrx[3] = gyrx[4];
    gyrx[4]  = acclx/coeff_lp4[0];
    tgyrx[0] = tgyrx[1]; tgyrx[1] = tgyrx[2]; tgyrx[2] = tgyrx[3]; tgyrx[3] = tgyrx[4];
    tgyrx[4]= (gyrx[0] + gyrx[4]) + 4 * (gyrx[1] + gyrx[3]) + 6 * gyrx[2] + 
               ( coeff_lp4[1] * tgyrx[0]) + ( coeff_lp4[2] * tgyrx[1]) + 
               ( coeff_lp4[3] * tgyrx[2]) + ( coeff_lp4[4] * tgyrx[3]);

    gyry[0]  = gyry[1]; gyry[1] = gyry[2]; gyry[2] = gyry[3]; gyry[3] = gyry[4];
    gyry[4]  = accly/coeff_lp4[0];
    tgyry[0] = tgyry[1]; tgyry[1] = tgyry[2]; tgyry[2] = tgyry[3]; tgyry[3] = tgyry[4];
    tgyry[4]= (gyry[0] + gyry[4]) + 4 * (gyry[1] + gyry[3]) + 6 * gyry[2] + 
               ( coeff_lp4[1] * tgyry[0]) + ( coeff_lp4[2] * tgyry[1]) + 
               ( coeff_lp4[3] * tgyry[2]) + ( coeff_lp4[4] * tgyry[3]);

    gyrz[0]  = gyrz[1]; gyrz[1] = gyrz[2]; gyrz[2] = gyrz[3]; gyrz[3] = gyrz[4];
    gyrz[4]  = acclz/coeff_lp4[0];
    tgyrz[0] = tgyrz[1]; tgyrz[1] = tgyrz[2]; tgyrz[2] = tgyrz[3]; tgyrz[3] = tgyrz[4];
    tgyrz[4]= (gyrz[0] + gyrz[4]) + 4 * (gyrz[1] + gyrz[3]) + 6 * gyrz[2] + 
               ( coeff_lp4[1] * tgyrz[0]) + ( coeff_lp4[2] * tgyrz[1]) + 
               ( coeff_lp4[3] * tgyrz[2]) + ( coeff_lp4[4] * tgyrz[3]);

    accel_mag   = sqrt( tgyrx[4]*tgyrx[4] + tgyry[4]*tgyry[4] + tgyrz[4]*tgyrz[4] ); //throttle
    accel_theta = asin (tgyrx[4]/accel_mag);  //roll
    accel_phi   = asin (tgyry[4]/accel_mag);  //pitch

    printf("@%8.3fms  g = %6.5f  roll = %5.1fdeg  pitch = %5.1fdeg  zaxis = %5.3fg\n",
            0.001 * esp_log_timestamp(), accel_mag,57.3*accel_theta,57.3*accel_phi, acclz);
}

static void accel_cal() 
{
    uint8_t regdata[6]; 
    i2c_read(0x68, 0x3b, regdata, 6);
    acclx_cal =  (256*regdata[0] + regdata[1]);
    if (acclx_cal >= 0x8000) { acclx_cal = -1.0*(0xffff - acclx_cal); } //2s comp
    acclx_cal = -1.0 * acclx_cal / 0x4000; //full scale +/- 2g

    accly_cal =  (256*regdata[2] + regdata[3]);
    if (accly_cal >= 0x8000) { accly_cal = -1.0*(0xffff - accly_cal); }
    accly_cal = -1.0 * accly_cal / 0x4000;

    acclz_cal =  (256*regdata[4] + regdata[5]);
    if (acclz_cal >= 0x8000) { acclz_cal = -1.0*(0xffff - acclz_cal); }
    acclz_cal = 1.0 - acclz_cal / 0x4000;

    printf(" calibrate accl  %f   %f   %f\n",acclx_cal,accly_cal, acclz_cal);
}

static void accel_state() 
{
    uint8_t regdata[6]; 
    float acclx, accly, acclz;
    i2c_read(0x68, 0x3b, regdata, 6);
    acclx = 256*regdata[0] + regdata[1];
    if (acclx >= 0x8000) { acclx = -1.0*(0xffff - acclx); } //2s comp
    acclx = acclx / 0x4000; //full scale +/- 2g

    accly = 256*regdata[2] + regdata[3];
    if (accly >= 0x8000) { accly = -1.0*(0xffff - accly); }
    accly = accly / 0x4000;

    acclz = 256*regdata[4] + regdata[5];
    if (acclz >= 0x8000) { acclz = -1.0*(0xffff - acclz); }
    acclz = acclz / 0x4000;
    acclx = acclx + acclx_cal;
    accly = accly + accly_cal;
    acclz = acclz + acclz_cal;

    accel_mag   = sqrt( acclx*acclx + accly*accly + acclz*acclz ); //throttle
    accel_theta = asin (acclx/accel_mag);  //roll
    accel_phi   = asin (accly/accel_mag);  //pitch

    printf("@%8.3fms  g = %6.5f  roll = %5.1fdeg  pitch = %5.1fdeg  zaxis = %5.3fg\n",
            0.001 * esp_log_timestamp(), accel_mag,57.3*accel_theta,57.3*accel_phi, acclz);
}

static void mpu6050_init() 
{
    i2c_write( 0x68 , 0x19, 0x00); //sample rate 1khz/(reg+1) 0x00 = 1000samp/sec
    i2c_write( 0x68 , 0x1a, 0x01); //sync and filtering (minimal filtering) - 1KHz rate
    i2c_write( 0x68 , 0x1b, 0x00); //gyro range +/- 250deg/sec
    i2c_write( 0x68 , 0x1c, 0x00); //accl range +/- 2g
    i2c_write( 0x68 , 0x23, 0x70); //fifoen gyro 3 axes enable = 6 bytes/sample
    i2c_write( 0x68 , 0x68, 0x06); //reset
    vTaskDelay(50);
    i2c_write( 0x68 , 0x6a, 0x40); //fifoen
    i2c_write( 0x68 , 0x6b, 0x00); //not sleep
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    mpu6050_init();

    wifi_init();
    vTaskDelay(100);
    espnow_init();  //starts task after init

    sbus_init();
    xTaskCreate (sbus_tx, "sbus_tx_task", 4096, NULL, 5, NULL);
    printf("porttick = %d\n",portTICK_RATE_MS);

    vTaskDelay(10);
    accel_cal(); 
    accel_cal(); 
    uint8_t fifocnth, fifocntl;
    uint8_t regdata[1024];
    while(1){
        accel_state_lp(); 
        
        i2c_read(0x68, 0x72, &fifocnth, 1);
        i2c_read(0x68, 0x73, &fifocntl, 1);
        //printf("@%8.3fms   fifo cnt = %4d\n", 0.001 * esp_log_timestamp(), 256*fifocnth+fifocntl);
        i2c_read(0x68, 0x74, regdata, 256*fifocnth + fifocntl);

        vTaskDelay(45/portTICK_RATE_MS);
    }
}
