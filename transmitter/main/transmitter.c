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

#include "espnow.h"

#include "driver/i2c.h"
#include "../components/i2c.h"
#include "../components/ssd1306.h"

#define ESPNNOW_SEND_COUNT 1000
#define ESPNOW_SEND_DELAY   (100-10)
#define ESPNOW_SEND_LEN     200

int throttle, yaw, pitch, roll;

static uint8_t fc_mac[ESP_NOW_ETH_ALEN] =        { 0x24, 0x6F, 0x28, 0x17, 0xff, 0xc8 };
static xQueueHandle espnow_queue;
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = { 0, 0 };
static void espnow_deinit(espnow_send_param_t *send_param);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI("", "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
    if (mac_addr == NULL) { ESP_LOGE("", "Send cb arg error"); return; }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW("", "Send send queue fail");
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE("", "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE("", "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW("", "Send receive queue fail");
        free(recv_cb->data);
    }
}

int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    for (int a=10 ; a<data_len; a++){
        printf("%c", data[a]);
        if(data[a] == ';')break;
    }
    printf("\n");
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE("", "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }
    return -1;
}

/* Prepare ESPNOW data to be sent. */
void espnow_data_prepare(espnow_send_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    int i = 0;
    assert(send_param->len >= sizeof(espnow_data_t));
    buf->type =  ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    char temp_str[200];
    sprintf(temp_str, "throttle=%d;yaw=%d;pitch=%d;roll=%d;\n\n",
                        throttle, yaw, pitch, roll);
    for (i = 0; i < send_param->len - sizeof(espnow_data_t); i++) {
        buf->payload[i] = (uint8_t)temp_str[i];
        //buf->payload[i] = (uint8_t)esp_random();
    }
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void espnow_task(void *pvParameter)
{
    espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    int ret;

    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE("", "Send error");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                //ESP_LOGD("", "Send data to "MACSTR", status1: %d", 
                                       //MAC2STR(send_cb->mac_addr), send_cb->status);
                send_param->count--;

                if (send_param->delay > 0) { vTaskDelay(send_param->delay/portTICK_RATE_MS); }

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                espnow_data_prepare(send_param);
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE("", "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                ESP_LOGI("", "control packet sent");
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                break;
            }
            default:
                ESP_LOGE("", "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t espnow_init(void)
{
    espnow_send_param_t *send_param;
    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL) { ESP_LOGE("", "Create mutex fail"); return ESP_FAIL; }

    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE("", "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, fc_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE("", "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = ESPNNOW_SEND_COUNT;
    send_param->delay = ESPNOW_SEND_DELAY;
    send_param->len   = ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE("", "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, fc_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

static void collect_controls () 
{
    uint8_t tmp_str[2];
    int throt_off = 0;
    while(1){
       tmp_str[0] = 0x40; 
       tmp_str[1] = 0xc0; 
       i2c_write_block( 0x4a, 0x01, tmp_str, 2);
       i2c_read( 0x4a, 0x00, tmp_str, 2);
       throt_off = throt_off + ((((256*tmp_str[0]+tmp_str[1])/16)-543)/10);
       if(throt_off > 600){ throt_off= 600; }
       if(throt_off < 0 ){ throt_off= 0; }
       throttle = 1000 + throt_off;

       tmp_str[0] = 0x50; 
       tmp_str[1] = 0xc0; 
       i2c_write_block( 0x4a, 0x01, tmp_str, 2);
       i2c_read( 0x4a, 0x00, tmp_str, 2);
       yaw = 1500 - (((256*tmp_str[0]+tmp_str[1])/16)-566)/10;

       tmp_str[0] = 0x60; 
       tmp_str[1] = 0xc0; 
       i2c_write_block( 0x4a, 0x01, tmp_str, 2);
       i2c_read( 0x4a, 0x00, tmp_str, 2);
       pitch = 1500 - (((256*tmp_str[0]+tmp_str[1])/16)-545)/10;

       tmp_str[0] = 0x70; 
       tmp_str[1] = 0xc0; 
       i2c_write_block( 0x4a, 0x01, tmp_str, 2);
       i2c_read( 0x4a, 0x00, tmp_str, 2);
       roll = 1500 + (((256*tmp_str[0]+tmp_str[1])/16)-560)/10;

       vTaskDelay((100-10)/portTICK_RATE_MS); 
       ESP_LOGI("","collected all data");
    }

}

void app_main()
{
    // Initialize NVS
    ESP_ERROR_CHECK( nvs_flash_init() );

    wifi_init();
    vTaskDelay(10);
    espnow_init(); //sets up espnow_task

    i2c_init();
    i2cdetect();
    ssd1305_init();

    vTaskDelay(10);

    xTaskCreate(collect_controls, "collect_controls_task", 1048, NULL, 5, NULL);
     
    char disp_str[128];
    while(1) {   //actuall using ssd1305 (128x32) so size 4 only
       sprintf(disp_str, "4 Throt %4d|| Yaw   %4d|| Pitch %4d|| Roll  %4d",
             throttle, yaw, pitch, roll);
       printf("%s\n",disp_str);
       ssd1305_text(disp_str);
       vTaskDelay((500)/portTICK_RATE_MS); 
    }

}
