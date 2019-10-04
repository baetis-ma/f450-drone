#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#define ESPNOW_QUEUE_SIZE           6

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};


typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;

static xQueueHandle s_espnow_queue;
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = { 0, 0 };
static void espnow_deinit(espnow_send_param_t *send_param);
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI( "", "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

static esp_err_t espnow_init(void);

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

    if (mac_addr == NULL) {
        ESP_LOGE( "", "Send cb arg error");
        return;
    }

    evt.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW( "", "Send send queue fail");
    }
    //printf("@%8.3fms espnow packet sent\n", 0.001 * esp_log_timestamp());
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_event_t evt;
    espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE( "", "Receive cb arg error");
        return;
    }

    evt.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE( "", "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW( "", "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    sscanf((char*)data+10,"throttle=%d;yaw=%d;pitch=%d;roll=%d;", &throttle, &yaw, &pitch, &roll);
    printf("%4d@%8.3fms   thr=%4d yaw=%4d pitch=%4d roll=%4d\n",
                  buf->seq_num, 0.001 * esp_log_timestamp(), throttle, yaw, pitch, roll);
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE( "", "Receive ESPNOW data too short, len:%d", data_len);
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

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = 0;
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    char temp_str[200];
    sprintf(temp_str, "Blackbox Data - MCP Command 0x55;\n\n");
    for (int i = 0; i < send_param->len - sizeof(espnow_data_t); i++) {
        buf->payload[i] = (uint8_t)temp_str[i];
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

    vTaskDelay(5000 / portTICK_RATE_MS);

    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE( "", "Send error");
        espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                //ESP_LOGI( "", "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));
                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                espnow_data_prepare(send_param);

                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE( "", "Send error");
                    espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
                ret = espnow_data_parse(recv_cb->data, recv_cb->data_len,  
                                       &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                break;
            }
            default:
                ESP_LOGE( "", "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t espnow_init(void)
{
    espnow_send_param_t *send_param;

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE( "", "Create mutex fail");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE( "", "Malloc peer information fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_tx_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_send_param_t));
    memset(send_param, 0, sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE( "", "Malloc send parameter fail");
        vSemaphoreDelete(s_espnow_queue);
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
        ESP_LOGE( "", "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_tx_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    xTaskCreate(espnow_task, "espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_espnow_queue);
    esp_now_deinit();
}

