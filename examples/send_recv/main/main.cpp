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
#include "../src/lmic/radio.c"
#include "../src/lmic/lmic_eu_like.h"


#include "esp32/ulp.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"



#include <stdio.h>
#include "esp_types.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"


#define TIMESLOT 0 



#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (5) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;





uint32_t userUTCTime; // Seconds since the UTC epoch


// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = "0000000000000033";

// Copy the below two lines from bottom of the same page
const char *appEui = "0000000000000005";
const char *appKey = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

// Pins and other resources //lolin32
#define TTN_SPI_HOST      HSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  18 
#define TTN_PIN_SPI_MOSI  23 
#define TTN_PIN_SPI_MISO  19 
#define TTN_PIN_NSS       5 
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       25 
#define TTN_PIN_DIO0      27 
#define TTN_PIN_DIO1      26 
// RTC_GPIO FOR NCC E RST
//#define TTN_SPI_HOST      HSPI_HOST
//#define TTN_SPI_DMA_CHAN  1
//#define TTN_PIN_SPI_SCLK  18 
//#define TTN_PIN_SPI_MOSI  23 
//#define TTN_PIN_SPI_MISO  19 
//#define TTN_PIN_NSS       27 
//#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
//#define TTN_PIN_RST       25 
//#define TTN_PIN_DIO0      14 
//#define TTN_PIN_DIO1      26 
// ttgo
//#define TTN_SPI_HOST      HSPI_HOST
//#define TTN_SPI_DMA_CHAN  1
//#define TTN_PIN_SPI_SCLK  5
//#define TTN_PIN_SPI_MOSI  27
//#define TTN_PIN_SPI_MISO  19
//#define TTN_PIN_NSS       18
//#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
//#define TTN_PIN_RST       14
//#define TTN_PIN_DIO0      26
//#define TTN_PIN_DIO1      33

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 300;
static uint8_t msgData[] = "Hello, world";

static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR u4_t RTCnetid;
RTC_DATA_ATTR u4_t RTCdevaddr;
RTC_DATA_ATTR u1_t RTCnwkKey[16];
RTC_DATA_ATTR u1_t RTCartKey[16];
RTC_DATA_ATTR int RTCseqnoUp;
RTC_DATA_ATTR int RTCseqnoDn;
RTC_DATA_ATTR int deepsleep=0;
RTC_DATA_ATTR int counter=0;
RTC_DATA_ATTR int RTCtxend;



void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;
    lmic_time_reference_t lmicTimeReference;
    if (flagSuccess != 1) {
        printf("USER CALLBACK: Not a success");
        return;
    }
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        printf("USER CALLBACK: LMIC_getNetworkTimeReference didn't succeed");
        return;
    }
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;
    printf("ticsNow:%d time:%d tNetwork:%d tLocal:%d\n",ticksNow,*pUserUTCTime, lmicTimeReference.tNetwork, lmicTimeReference.tLocal);
}


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
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    //const int ext_wakeup_pin_1 = 25;
    //const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    //const int ext_wakeup_pin_2 = 26;
    //const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    //printf("Enabling EXT1 wakeup on on
    //esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    //rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);
    deepsleep=1;
    //mycounter=0;
    esp_deep_sleep_start();
}


/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0,(timer_idx_t) timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0,(timer_idx_t) timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, (timer_idx_t) timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, (timer_idx_t) timer_idx);
    timer_isr_register(TIMER_GROUP_0, (timer_idx_t) timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, (timer_idx_t) timer_idx);
}

/*
 * The main task of this example program
 */
static void timer_example_evt_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        if (evt.type == TEST_WITHOUT_RELOAD) {
            printf("\n    Example timer without reload\n");
        } else if (evt.type == TEST_WITH_RELOAD) {
            printf("\n    Example timer with auto reload\n");
        } else {
            printf("\n    UNKNOWN EVENT TYPE\n");
        }
        //printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);

        ///* Print the timer values passed by event */
        //printf("------- EVENT TIME --------\n");
        //print_timer_counter(evt.timer_counter_value);

        ///* Print the timer values as visible by this task */
        //printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value((timer_group_t) evt.timer_group,(timer_idx_t) evt.timer_idx, &task_counter_value);
        print_timer_counter(task_counter_value);
    }
}






void sendMessages2()
{
	printf("Sending message 1...\n");
	TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
	printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
	LMIC.bands[BAND_MILLI].avail =
        LMIC.bands[BAND_CENTI].avail =
        LMIC.bands[BAND_DECI ].avail = os_getTime();

	printf("Sending message 2...\n");
	TTNResponseCode res2 = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
	printf(res2 == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
	LMIC.bands[BAND_MILLI].avail =
        LMIC.bands[BAND_CENTI].avail =
        LMIC.bands[BAND_DECI ].avail = os_getTime();

	RTCseqnoUp=LMIC.seqnoUp;	
	RTCseqnoDn=LMIC.seqnoDn;	
	printf("Sendmessage2 now:%d\n",hal_ticks());
	sleeppa(10);
}





void sendMessages(void* pvParameter)
{
    while (1) {
	printf("counter=%d\n",counter);
	if (counter==TIMESLOT){
        	printf("Sending message 1...\n");
        	TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        	printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

		RTCseqnoUp=LMIC.seqnoUp;	
		RTCseqnoDn=LMIC.seqnoDn;	

        	printf("aspetto un po...\n");
    		//vTaskDelay(5000 / portTICK_PERIOD_MS);
		vTaskDelay( 1000 / portTICK_PERIOD_MS);
        	printf("Sending message 2...\n");
        	res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        	printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

		RTCseqnoUp=LMIC.seqnoUp;	
		RTCseqnoDn=LMIC.seqnoDn;	
		//rtc_gpio_set_direction(GPIO_NUM_27,RTC_GPIO_MODE_INPUT_OUTPUT);
		//rtc_gpio_pulldown_dis(GPIO_NUM_27);
		//rtc_gpio_pullup_en(GPIO_NUM_27);                     // set the GPIO_NUM_27 as pull-up
		//rtc_gpio_set_direction(GPIO_NUM_25,RTC_GPIO_MODE_INPUT_OUTPUT);
		//rtc_gpio_pulldown_dis(GPIO_NUM_25);
		//rtc_gpio_pullup_en(GPIO_NUM_25);                     // set the GPIO_NUM_25 as pull-up
		//esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON); 
		//////rtc_gpio_hold_en(GPIO_NUM_27);
		////hal_sleep ();
		//opmode(OPMODE_SLEEP);
		counter=0;
	}else{
	  counter+=1;
	}
	sleeppa(10);
        //vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}

void messageReceived(const uint8_t* message, size_t length, port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        //printf(" %02x", message[i]);
        printf("%c", message[i]);
    printf("\n");
}

extern "C" void app_main(void)
{

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    //example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
    
    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

   if(counter==TIMESLOT or !deepsleep){
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
   }
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

	    sendMessages2();
    	    //xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
    	}
    	else
    	{
    	    printf("Join failed. Goodbye\n");
    	    sleeppa(300);
    	}
    }else{
        if (counter==TIMESLOT){
    	  ttn_hal.initCriticalSection();
	  printf("LMIC_reset\n");
    	  LMIC_reset();
	  printf("ttn_hal.wakeUp\n");
          ttn_hal.wakeUp();
	  printf("LMIC_setSession\n");
    	  LMIC_setSession (RTCnetid, RTCdevaddr, RTCnwkKey, RTCartKey);
          LMIC.seqnoUp=RTCseqnoUp;
          LMIC.seqnoDn=RTCseqnoDn;
	  //LMIC.txend=RTCtxend;
          ttn_hal.leaveCriticalSection();
          printf("mando il messaggio in ABP con numeri di sequenza Up:%d Dn:%d\n",LMIC.seqnoUp,LMIC.seqnoDn);
          //LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
	  LMIC.bands[BAND_MILLI].avail = os_getTime();
	  LMIC.bands[BAND_CENTI].avail = os_getTime();
	  LMIC.bands[BAND_DECI].avail = os_getTime();

	  sendMessages2();
    	  //xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
        }

    }
}
