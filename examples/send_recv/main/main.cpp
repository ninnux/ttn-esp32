/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program showing how to send and receive messages.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "TheThingsNetwork.h"
#include "../src/lmic/oslmic.h"
#include "../src/lmic/lmic.h"
#include "../src/hal/hal_esp32.h"


// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = "0000000000000015";

// Copy the below two lines from bottom of the same page
const char *appEui = "0000000000000005";
const char *appKey = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

// Pins and other resources
#define TTN_SPI_HOST      HSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  5
#define TTN_PIN_SPI_MOSI  27
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       18
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       14
#define TTN_PIN_DIO0      26
#define TTN_PIN_DIO1      33

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
static uint8_t msgData[] = "Hello, world";

static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR u4_t RTCnetid;
RTC_DATA_ATTR u4_t RTCdevaddr;
RTC_DATA_ATTR u1_t RTCnwkKey[16];
RTC_DATA_ATTR u1_t RTCartKey[16];
RTC_DATA_ATTR int RTCseqnoUp;
RTC_DATA_ATTR int RTCseqnoDn;
RTC_DATA_ATTR int deepsleep=0;

void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
	    //deepsleep=1;
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
	    //deepsleep=0;
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    const int ext_wakeup_pin_1 = 25;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = 26;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    //rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);
    deepsleep=1;
    esp_deep_sleep_start();
}
void sendMessages(void* pvParameter)
{
    while (1) {
        printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

	RTCseqnoUp=LMIC.seqnoUp;	
	RTCseqnoDn=LMIC.seqnoDn;	
	sleeppa(20);
        //vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}

void messageReceived(const uint8_t* message, size_t length, port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

extern "C" void app_main(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
    
    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);
    ttn.onMessage(messageReceived);

    // The below line can be commented after the first run as the data is saved in NVS
    if(!deepsleep){
    	ttn.provision(devEui, appEui, appKey);


    	printf("Joining...\n");
    	if (ttn.join())
    	{
    	    printf("Joined.\n");
	    LMIC_getSessionKeys (&RTCnetid, &RTCdevaddr,RTCnwkKey,RTCartKey);	    
            printf("netid:%x\n",RTCnetid);
            printf("devaddr:%x\n",RTCdevaddr);
            int i=0;
            int k=0;
            for(i=0;i<16;i++){
            	printf("%.2x",RTCnwkKey[i]);
            }
            printf("\n");
            for(k=0;k<16;k++){
            	printf("%.2x",RTCartKey[k]);
            }
            printf("\n");
    	    xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
    	}
    	else
    	{
    	    printf("Join failed. Goodbye\n");
    	    sleeppa(600);
    	}
    }else{
    	LMIC_reset();
    	//hal_enterCriticalSection();
    	LMIC_setSession (RTCnetid, RTCdevaddr, RTCnwkKey, RTCartKey);
    	//hal_leaveCriticalSection();
	LMIC.seqnoUp=RTCseqnoUp;
	LMIC.seqnoDn=RTCseqnoDn;
	printf("mando il messaggio in ABP con numeri di sequenza Up:%d Dn:%d\n",LMIC.seqnoUp,LMIC.seqnoDn);
    	xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, NULL);

    }
}
