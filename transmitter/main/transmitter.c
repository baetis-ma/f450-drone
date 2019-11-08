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
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "crc.h"
#include "esp_wifi.h"

//globals 
int throttle, yaw, pitch, roll, throttle_off;
float fthrottle_off=1000;
float fthrottle_last=1500, fyaw_last=1500, fpitch_last=1500, froll_last=1500;
int calset = 0, astate = 0, calib = 0;

//required for espnow
#include "../components/espnow.h"
#define ESPNOW_SEND_DELAY   60
#define ESPNOW_SEND_LEN     200
//static uint8_t fc_mac[ESP_NOW_ETH_ALEN] =        { 0x4c, 0x11, 0xae, 0x75, 0x59, 0x58 };
static uint8_t fc_mac[ESP_NOW_ETH_ALEN] =        { 0x24, 0x6f, 0x28, 0x17, 0xff, 0xc8 };
static xQueueHandle espnow_queue;
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = { 0, 0 };
static void espnow_deinit(espnow_send_param_t *send_param);
#include "./espnow.h"

//required fir i2c
#include "driver/i2c.h"
#include "../components/i2c.h"

static void adc_collect_data () 
{
    //noise from wifi transmission requires some pretty aggresive filtering
    float fthrottle, fyaw, fpitch, froll;
    float rate = .025; //40 msec loop
    float tau = .1;    //10 hz one pole LP filter
    float filter_T;
    uint8_t tmp_str[2];
    int temp;
    int tempa[3];
    filter_T = (tau/rate)/(1+(tau/rate));
    tmp_str[0] = 0x40; 
    tmp_str[1] = 0xc0; 
    i2c_write_block( 0x4a, 0x01, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    tempa[0] = 256*(0x7f & tmp_str[0])+tmp_str[1];
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    tempa[1] = 256*(0x7f & tmp_str[0])+tmp_str[1];
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    tempa[2] = 256*(0x7f & tmp_str[0])+tmp_str[1];
    if(tempa[0] > tempa[1]){temp = tempa[0];tempa[0] = tempa[1]; tempa[1] = temp; }
    if(tempa[1] > tempa[2]){temp = tempa[1];tempa[1] = tempa[2]; tempa[2] = temp; }  //tempa[2] has largest
    if(tempa[0] > tempa[1]){temp = tempa[0];tempa[0] = tempa[1]; tempa[1] = temp; }  //tempa[0] has lowest
    
    if(tempa[1] >= 0x3000) fthrottle = 1500 - 500*(tempa[1]-0x3000)/0x3000;
                  else fthrottle = 1500 + 500*(0x3000-tempa[1])/0x3000;
    //fthrottle = filter_T * fthrottle_last + (1 - filter_T) * fthrottle;
    if(fthrottle > 1450 && fthrottle < 1550) fthrottle = 1500;
    //if((abs(fthrottle - fthrottle_last))>100 ) fthrottle = 1500;
    fthrottle_last = fthrottle;
    if(fthrottle > 1800 && fthrottle <= 2000) fthrottle_off = fthrottle_off +  5.0; //+ (fthrottle-1800)/50;
    if(fthrottle > 1500 && fthrottle <= 1800) fthrottle_off = fthrottle_off +  0.5; //+ (fthrottle-1500)/100;
    if(fthrottle > 1200 && fthrottle <= 1499) fthrottle_off = fthrottle_off -  0.5; //+ (fthrottle-1500)/150;
    if(fthrottle > 1000 && fthrottle <= 1200) fthrottle_off = fthrottle_off - 10.0; //+ (fthrottle-1200)/50;
    if(fthrottle_off < 1000) fthrottle_off = 1000;
    if(fthrottle_off > 1500) fthrottle_off = 1500; //limit command throttle
    throttle_off = (int) fthrottle_off;
    throttle = (int) fthrottle;

    tmp_str[0] = 0x50; 
    tmp_str[1] = 0xc0; 
    i2c_write_block( 0x4a, 0x01, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    temp = 256*tmp_str[0]+tmp_str[1];
    //printf("a1 = 0x%04x  ", temp);
    if(temp >= 0x3000) fyaw = 1500 + 500*(temp-0x3000)/0x3000;
                  else fyaw = 1500 - 500*(0x3000-temp)/0x3000;
    if(fyaw < 1000 || fyaw > 2000) fyaw = fyaw_last;
    fyaw = filter_T * fyaw_last + (1 - filter_T) * fyaw;
    if(fyaw > 1450 && fyaw < 1550) fyaw = 1500;
    fyaw_last = fyaw;
    yaw = (int) fyaw;

    tmp_str[0] = 0x60; 
    tmp_str[1] = 0xc0; 
    i2c_write_block( 0x4a, 0x01, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    temp = 256*tmp_str[0]+tmp_str[1];
    //printf("a2 = 0x%04x  ", temp);
    if(temp >= 0x3000) fpitch = 1500 - 500*(temp-0x3000)/0x3000;
                  else fpitch = 1500 + 500*(0x3000-temp)/0x3000;
    if(fpitch < 1000 || fpitch > 2000) fpitch = fpitch_last;
    fpitch = filter_T * fpitch_last + (1 - filter_T) * fpitch;
    if(fpitch > 1470 && fpitch < 1530) fpitch = 1500;
    fpitch_last = fpitch;
    pitch = (int) fpitch;

    tmp_str[0] = 0x70; 
    tmp_str[1] = 0xc0; 
    i2c_write_block( 0x4a, 0x01, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    i2c_read( 0x4a, 0x00, tmp_str, 2);
    temp = 256*tmp_str[0]+tmp_str[1];
    //printf("a3 = 0x%04x\n", temp);
    if(temp >= 0x3000) froll = 1500 - 500*(temp-0x3000)/0x3000;
                  else froll = 1500 + 500*(0x3000-temp)/0x3000;
    if(froll < 1000 || froll > 2000) froll = froll_last;
    froll = filter_T * froll_last + (1 - filter_T) * froll;
    if(froll > 1470 && froll < 1530) froll = 1500;
    froll_last = froll;
    roll = (int) froll;
    //ESP_LOGI(""," adc data read"); 

    if (fthrottle > 1250 && yaw > 1250 && calset == 1) calset = 0; 
    if (calib == 1) calib = 0;
    if (fthrottle < 1200 && yaw < 1200 && calset == 0) {calib = 1; astate = 0;
                                         calset = 1; fthrottle_off = 1000; }
    if (fthrottle < 1200 && yaw > 1800) {astate = 1; fthrottle_off = 1000; }
}

void espnow_data_prepare(espnow_send_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    int i = 0;
    assert(send_param->len >= sizeof(espnow_data_t));
    buf->type =  ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->magic = send_param->magic;
    sprintf( (char *) buf->payload, "astate=%d;calib=%d;throttle=%d;yaw=%d;pitch=%d;roll=%d;\n\n", 
          astate, calib, throttle_off, yaw, pitch, roll);
    //printf("%s", (char *)buf->payload);
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len - sizeof(espnow_data_t));
}

void app_main()
{
    // Initialize NVS
    ESP_ERROR_CHECK( nvs_flash_init() );

    wifi_init();
    vTaskDelay(10);

    //only receiving now
    espnow_init(); //sets up espnow_task 

    i2c_init();
    i2cdetect();

    espnow_send_param_t *send_param;
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->count = 1;
    send_param->delay = ESPNOW_SEND_DELAY;
    send_param->len   = ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    memcpy(send_param->dest_mac, fc_mac, ESP_NOW_ETH_ALEN);

    int cnt = 0;
    while(1) {
       adc_collect_data();
       //vTaskDelay(10/portTICK_RATE_MS); 
       espnow_data_prepare(send_param);
       esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len);
       //printf("throttle = %4d   yaw = %4d   pitch = %4d  roll = %4d\n", throttle_off, yaw, pitch, roll);
       //printf("throttle = %4d   yaw = %4d   pitch = %4d  roll = %4d\n", throttle, yaw, pitch, roll);
       //if(throttle != 1500)printf("%4d throttle = %d   throttle_off = %d\n", cnt++, throttle, throttle_off);
       //if(yaw != 1500)     printf("%4d yaw      = %d\n", cnt++, yaw);
       //if(pitch != 1500)   printf("%4d pitch    = %d\n", cnt++, pitch);
       //if(roll != 1500)    printf("%4d roll     = %d\n", cnt++, roll);
       vTaskDelay(60/portTICK_RATE_MS); 
    }
}
