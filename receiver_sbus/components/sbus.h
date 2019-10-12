//requirements for sbus uart
#include "driver/uart.h"
#define RTS            (UART_PIN_NO_CHANGE)
#define CTS            (UART_PIN_NO_CHANGE)
#define UART_BUF_SIZE  (1024)

//sbus protocol defines
/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

char  sbus_byte[25];
uint16_t channel_dat[16];
uint16_t channel_init[16] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                              1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

static void sbus_init()
{
    uart_config_t uart_config = {
        .baud_rate = 102000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(SBUS_UART, &uart_config);
    uart_set_pin(SBUS_UART, TXD, RXD, RTS, CTS);
    uart_driver_install(SBUS_UART, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void sbus_monitor()
{
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(SBUS_UART, data, UART_BUF_SIZE, 20 / portTICK_RATE_MS);
        //if(len>0){printf("sbus monitor rcvd %3d bytes ",len); 
        //    for(int a = 0; a < len; a++){ 
        //        printf("0x%02x ", data[a]);
        //    } 
        //    printf("\n"); 
        //}
    }
}

void channel_sbus(){
    // chan0 roll 1 pitch 2 throttle 4 yaw
    channel_init[0] = roll;
    channel_init[1] = pitch;
    channel_init[2] = throttle;
    channel_init[3] = yaw;
    //adjust raw channel data for sbus so they get put pack again correctly
    for (int x= 0; x <16; x++){
       channel_dat[x] = 0x07ff & (int) ((((channel_init[x]-SBUS_TARGET_MIN)/SBUS_SCALE_FACTOR)+1.0f) + (int)(SBUS_RANGE_MIN-8));
    }
    sbus_byte[0]  = 0x0f;
    sbus_byte[1]  = 0xff & (                       ((channel_dat[0]%(1<<9))<<0));   // 0.7 0.6 0.5 0.4 0.3 0.2 0.1 0.0
    sbus_byte[2]  = 0xff & ((channel_dat[0]>>8)  + ((channel_dat[1]%(1<<6))<<3));   // 1.4 1.3 1.2 1.1 1.0 0.a 0.9 0.8
    sbus_byte[3]  = 0xff & ((channel_dat[1]>>5)  + ((channel_dat[2]%(1<<2))<<6));   // 2.1 2.0 1.a 1.9 1.8 1.7 1.6 1.5
    sbus_byte[4]  = 0xff & (                     + ((channel_dat[2]%(1<<10))>>2));  // 2.9 2.8 2.7 2.6 2.5 2.4 2.3 2.2
    sbus_byte[5]  = 0xff & ((channel_dat[2]>>10) + ((channel_dat[3]%(1<<7))<<1));   // 3.6 3.5 3.4 3.3 3.2 3.1 3.0 2.a
    sbus_byte[6]  = 0xff & ((channel_dat[3]>>7)  + ((channel_dat[4]%(1<<4))<<4));   // 4.3 4.2 4.1 4.0 3.a 3.9 3.8 3.7
    sbus_byte[7]  = 0xff & ((channel_dat[4]>>4)  + ((channel_dat[5]%(1<<1))<<7));   // 5.0 4.a 4.9 4.8 4.7 4.6 4.5 4.4
    sbus_byte[8]  = 0xff & ((channel_dat[5]>>1)  + ((channel_dat[5]%(1<<9))>>1));   // 5.8 5.7 5.6 5.5 5.4 5.3 5.2 5.1
    sbus_byte[9]  = 0xff & ((channel_dat[5]>>9)  + ((channel_dat[6]%(1<<6))<<2));   // 6.5 6.4 6.3 6.2 6.1 6.0 5.a 5.9
    sbus_byte[10] = 0xff & ((channel_dat[6]>>6)  + ((channel_dat[7]%(1<<3))<<5));   // 7.2 7.1 7.0 6.a 6.9 6.8 6.7 6.6
    sbus_byte[11] = 0xff & ((channel_dat[7]>>3)                                );   // 7.a 7.9 7.8 7.7 7.6 7.5 7.4 7.3
    sbus_byte[12] = 0xff & (                       ((channel_dat[8]%(1<<9))<<0));   // same pattern channel_dat[x+8]
    sbus_byte[13] = 0xff & ((channel_dat[8]>>8)  + ((channel_dat[9]%(1<<6))<<3));
    sbus_byte[14] = 0xff & ((channel_dat[9]>>5)  + ((channel_dat[10]%(1<<2))<<6));
    sbus_byte[15] = 0xff & (                     + ((channel_dat[10]%(1<<10))>>2));
    sbus_byte[16] = 0xff & ((channel_dat[10]>>10)+ ((channel_dat[11]%(1<<7))<<1));
    sbus_byte[17] = 0xff & ((channel_dat[11]>>7) + ((channel_dat[12]%(1<<4))<<4));
    sbus_byte[18] = 0xff & ((channel_dat[12]>>4) + ((channel_dat[13]%(1<<1))<<7));
    sbus_byte[19] = 0xff & ((channel_dat[13]>>1) + ((channel_dat[13]%(1<<9))>>1));
    sbus_byte[20] = 0xff & ((channel_dat[13]>>9) + ((channel_dat[14]%(1<<6))<<2));
    sbus_byte[21] = 0xff & ((channel_dat[14]>>6) + ((channel_dat[15]%(1<<3))<<5));
    sbus_byte[22] = 0xff & ((channel_dat[15]>>3)                                );
    sbus_byte[23] = 0x00; //can contain bit information
    sbus_byte[24] = 0x00;
}

void sbus_tx () {
    int cnt = 0;
    while(1){
        channel_sbus();
        uart_write_bytes(SBUS_UART, (const char*) sbus_byte, 25);
        //printf("@%8.3fms  sbus packet sent cnt = %d   ", 0.001 * esp_log_timestamp(), ++cnt);
        //for(int x=0;x<10;x++)printf("0x%02x ",sbus_byte[x]);
        //printf("\n");
        vTaskDelay(50/portTICK_RATE_MS);
    }
}

