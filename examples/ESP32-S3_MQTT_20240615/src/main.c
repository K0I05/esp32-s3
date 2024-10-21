/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file main.c
 * @brief Sample application that connects to an MQTT broker and publishes items.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

/* 

    OpenSSL Example: https://stackoverflow.com/questions/59340198/how-do-i-save-a-https-certificate-and-put-it-in-a-pem-file-on-openssl

    it is the second base-64 certificate that seems to work 

    openssl s_client -connect mqtt.eclipseprojects.io:8883 -showcerts > eclipseprojects.txt

    openssl x509 -in eclipseprojects_io.pem -inform PEM -text
    openssl x509 -in lets_encrypt.pem -inform PEM -text


    LwIP SNTP example: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html#overview

    Time-Zone: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
    Time-Zone Abbreviations: https://www.timeanddate.com/time/zones/

    https://github.com/nopnop2002/esp-idf-mqtt-client/blob/main/main/mqtt.c
*/


#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h>
#include <sdkconfig.h>

#include <esp_system.h>
#include <esp_event.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_types.h>
#include <esp_wifi.h>
#include <esp_netif_sntp.h>
#include <esp_sntp.h>
#include <esp_tls.h>

#include <lwip/ip_addr.h>

#include <nvs_flash.h>

#include <mqtt_client.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>


#define CONFIG_WIFI_SSID                      "SSID"
#define CONFIG_WIFI_PASSWORD                  "PWD"


#define INET4_IP_FORMAT_BUFFER_SIZE             (15) // (255.255.255.255)

#define WIFI_CONNECT_MAXIMUM_RETRY              (10)

#define SNTP_TIME_FORMAT_BUFFER_SIZE            (64)
#define SNTP_TIME_SYNC_TIMEOUT_MS               (2000)
#define SNTP_TIME_SYNC_MAXIMUM_RETRY            (10)

#define MQTT_SMP_FORMAT_BUFFER_SIZE             (12)
#define MQTT_PUB_QOS0_QUEUE_SIZE                (10)


/**
 * @brief Event group definitions
 */
//
/* The wifi event group allows multiple bits for each event, but we only care about 2 events:
 *
 * 0 - we are connected to the AP with an IP over wifi
 * 1 - we failed to connect to AP over wifi after the maximum amount of retries 
 */
#define WIFI_EVTGRP_CONNECTED_BIT               (BIT0)
#define WIFI_EVTGRP_DISCONNECTED_BIT            (BIT1)
//
/* The mqtt event group allows multiple bits for each event, but we only care about 3 events:
 *
 * 0 - MQTT client connected to broker
 * 1 - MQTT client discconnected from broker
 * 2 - MQTT client connection error
 */
#define MQTT_EVTGRP_CONNECTED_BIT               (BIT0)
#define MQTT_EVTGRP_DISCONNECTED_BIT            (BIT1)
#define MQTT_EVTGRP_ERROR_BIT                   (BIT2)

/**
 * @brief macro definitions
 */
#define SEC_TO_USEC(sec)    { return (1000000U * sec); }
#define SEC_TO_MSEC(sec)    { return (1000U * sec); }
#define SEC_TO_TICKS(sec)   { return (SEC_TO_MSEC(sec) / portTICK_PERIOD_MS); }
#define MSEC_TO_TICKS(msec) { return (msec / portTICK_PERIOD_MS); }

/* SNTP default config for single or multiple time servers */
#define SNTP_DEFAULT_CONFIG                     ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org")
//#define SNTP_DEFAULT_CONFIG                     ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2, ESP_SNTP_SERVER_LIST("ca.pool.ntp.org", "pool.ntp.org") ) // set lwip->sntp config CONFIG_LWIP_SNTP_MAX_SERVERS

/**
 * @brief struct and enum definitions
 */

/**
 * @brief Environmental samples structure.  A basic data model to 
 * transmit and receive environmental samples as a queued item.
 */
typedef struct environmental_samples_tag {
    float temperature;  /*!< Temperature in degrees celsius */
    float humidity;     /*!< Relative humidity in percent */
    float pressure;     /*!< Barometric pressure in hecto-pascal */
} environmental_samples_t;

/**
 * @brief external constant definitions
 */

/* eclipse ca certificate - binary (embedded file as text) */
extern const uint8_t mqtt_eclipse_server_pem_start[] asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipse_server_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");

/**
 * @brief static constant and global definitions
 */

/* app tag for esp logging */
static const char              *TAG                         = "MQTT TLS [APP]";

/* global variables */
static esp_netif_t             *l_sta_netif                 = NULL;
static TaskHandle_t             l_sample_sensor_task_hdl    = NULL;
static TaskHandle_t             l_publish_sensor_task_hdl   = NULL;
static int                      l_wifi_retry_count          = 0;
static EventGroupHandle_t       l_wifi_evtgrp_hdl           = NULL;
static EventGroupHandle_t       l_mqtt_evtgrp_hdl           = NULL;
static QueueHandle_t            l_mqtt_pub_qos0_queue_hdl   = NULL;
static esp_mqtt_client_handle_t l_mqtt_client_hdl           = NULL;
static bool                     l_mqtt_connected            = false;

/* global variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and maintains its value 
 * when ESP32 wakes from deep sleep.
 */
RTC_DATA_ATTR static int        l_boot_count                = 0;

/**
 * @brief Prints the system date-time as esp information log.
 */
static inline void print_system_time(void) {
    time_t now; struct tm timeinfo; static char strftime_buf[SNTP_TIME_FORMAT_BUFFER_SIZE];
    time(&now); localtime_r(&now, &timeinfo); strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Halifax is: %s", strftime_buf);
}

/**
 * @brief Prints NTP time server(s) as an esp information log.
 */
static inline void print_sntp_time_servers(void) {
    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i) {
        if (esp_sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            // we have either IPv4 address, let's print it
            static char buff[INET4_IP_FORMAT_BUFFER_SIZE];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET4_IP_FORMAT_BUFFER_SIZE) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
        }
    }
}

/**
 * @brief Prints the free heap size in bytes with consumed bytes stats as an
 * esp information log.  This is used to monitor consumed bytes for possible 
 * memory leak(s) in the application.
 * 
 * @param free_heap_size_last Last free heap size in bytes.
 * @return uint32_t Adjusted last free heap size in bytes.
 */
static inline uint32_t print_free_heap_size(const uint32_t free_heap_size_last) {
    uint32_t free_heap_size_start = free_heap_size_last; /* set last heap size */
    uint32_t free_heap_size = esp_get_free_heap_size(); /* set free heap size */
    if(free_heap_size_start == 0) free_heap_size_start = free_heap_size; 
    int32_t free_heap_size_delta = free_heap_size_start - free_heap_size;
    if(free_heap_size_delta < 0) { free_heap_size_start = free_heap_size; free_heap_size_delta = 0; }
    ESP_LOGW(TAG, "Free Memory: %lu bytes (%li bytes Consumed)", free_heap_size, free_heap_size_delta);
    return free_heap_size_start;
}

/**
 * @brief Prints environmental sample values as an ESP information log.
 * 
 * @param samples Environmental samples structure to print.
 */
static inline void print_samples(environmental_samples_t const* samples) {
    ESP_LOGI(TAG, "Temperature(degC): %.2f", samples->temperature);
    ESP_LOGI(TAG, "Humidity(%%):      %.2f", samples->humidity);
    ESP_LOGI(TAG, "Pressure(hPa):     %.2f", samples->pressure);
}

/**
 * @brief Clears environmental sample values.
 * 
 * @param samples Environmental samples structure to clear.
 */
static inline void clear_samples(environmental_samples_t* const samples) {
    samples->temperature = NAN;
    samples->humidity    = NAN;
    samples->pressure    = NAN;
}

/**
 * @brief Delays the task by the specified delay in seconds.
 * 
 * @param previous_wake_ticks Ticks when the task was previous woken.
 * @param delay_sec Amount of time in seconds to delay the task.
 */
static inline void task_delay_until_sec(TickType_t *previous_wake_ticks, const uint delay_sec) {
    const TickType_t frequency_ticks = ((delay_sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previous_wake_ticks, frequency_ticks );  
}

/**
 * @brief An event handler registered to receive WIFI events.  This subroutine is called by the WIFI event loop.
 *
 * @param handler_args The user data registered to the event.
 * @param event_base Event base for the handler (i.e. WIFI and IP events).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_wifi_event_handle_t.
 */
static inline void wifi_event_handler(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGD(TAG, "WIFI event dispatched from event loop base=%s, event_id=%" PRIi32, event_base, event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /* init wifi event group state bits */
        xEventGroupClearBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_CONNECTED_BIT);
        xEventGroupClearBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_DISCONNECTED_BIT);
        /* initialize wifi retry counter to 0 */
        l_wifi_retry_count = 0;
        /* connect to wifi sta */
        esp_wifi_connect();
        ESP_LOGI(TAG, "Starting wifi access point connection.");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /* init wifi event group state bits */
        xEventGroupClearBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_CONNECTED_BIT);
        xEventGroupClearBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_DISCONNECTED_BIT);
        /* wifi connection retry handler */
        if (l_wifi_retry_count <= WIFI_CONNECT_MAXIMUM_RETRY) {
            /* connect to wifi sta */
            esp_wifi_connect();
            ESP_LOGW(TAG,"Connect to wifi access point failed, attempting retry (%d/%d)", l_wifi_retry_count, WIFI_CONNECT_MAXIMUM_RETRY);
            /* increment wifi retry counter */
            ++l_wifi_retry_count;
        } else {
            /* init wifi event group state bits */
            xEventGroupSetBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_DISCONNECTED_BIT);
            xEventGroupClearBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_CONNECTED_BIT);
            ESP_LOGE(TAG,"Unable to connect to wifi access point.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /* init wifi event group state bits */
        xEventGroupSetBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_CONNECTED_BIT);
        xEventGroupClearBits(l_wifi_evtgrp_hdl, WIFI_EVTGRP_DISCONNECTED_BIT);
        /* reset wifi retry counter to 0 */
        l_wifi_retry_count = 0;
        /* set got ip event data */
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got ip from wifi access point:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/**
 * @brief An event handler registered to receive SNTP events.  This subroutine is called by the SNTP event loop.
 * 
 * @param tv Time value from event handler.
 */
static inline void sntp_time_sync_event_handler(struct timeval *tv) {
    sntp_sync_status_t status = sntp_get_sync_status();

    ESP_LOGD(TAG, "SNTP notification of a time synchronization event");

    if(status == SNTP_SYNC_STATUS_COMPLETED) {
        ESP_LOGI(TAG, "Time synchronization completed..");
    } else if (status == SNTP_SYNC_STATUS_IN_PROGRESS) {
        ESP_LOGI(TAG, "Time synchronization in progress...");
    } else if (status == SNTP_SYNC_STATUS_RESET) {
        ESP_LOGI(TAG, "Time synchronization was reset...");
    } else {
        ESP_LOGI(TAG, "Time synchronization status is unknown...");
    }
}

/**
 * @brief An event handler registered to receive MQTT events.  This subroutine is called by the MQTT event loop.
 *
 * @param handler_args The user data registered to the event.
 * @param event_base Event base for the handler (MQTT events).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static inline void mqtt_event_handler(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    int                      msg_id;
    esp_mqtt_event_handle_t  event  = event_data;
    esp_mqtt_client_handle_t client = event->client;

    ESP_LOGD(TAG, "MQTT event dispatched from event loop base=%s, event_id=%" PRIi32, event_base, event_id);
    
    switch ((esp_mqtt_event_id_t)event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

			xEventGroupSetBits(l_mqtt_evtgrp_hdl, MQTT_EVTGRP_CONNECTED_BIT);
			xEventGroupClearBits(l_mqtt_evtgrp_hdl, MQTT_EVTGRP_DISCONNECTED_BIT);

			msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

			xEventGroupSetBits(l_mqtt_evtgrp_hdl, MQTT_EVTGRP_DISCONNECTED_BIT);
			xEventGroupClearBits(l_mqtt_evtgrp_hdl, MQTT_EVTGRP_CONNECTED_BIT);
			break;
		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);

			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
	 
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
	  
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");

			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);

			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(TAG, "MQTT_EVENT_ERROR");

			if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
				ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
				ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
				ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
						 strerror(event->error_handle->esp_transport_sock_errno));
			} else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
				ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
			} else {
				ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
			}

			break;
		default:
			ESP_LOGI(TAG, "Other event id:%d", event->event_id);
			break;
    }
}

/**
 * @brief Starts WIFI services.  This is a blocking function that waits for event bits to be
 * initialized based on WIFI event results or it returns an error when the timeout period 
 * has elapsed.
 * 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t wifi_start( void ) {
    esp_err_t                    ret = ESP_OK;
    wifi_init_config_t           cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    /* attempt to create default event loop and initialize netif */
    ESP_RETURN_ON_ERROR( esp_event_loop_create_default(), TAG, "Unable to create default event loop, wifi start failed" );
    ESP_RETURN_ON_ERROR( esp_netif_init(), TAG, "Unable to initialize netif, wifi start failed" );

    /* attempt to create default netif sta instance */
    l_sta_netif             = esp_netif_create_default_wifi_sta();
    ESP_RETURN_ON_FALSE( l_sta_netif, ESP_ERR_ESP_NETIF_INIT_FAILED, TAG, "Unable to create default netif sta instance, wifi start failed");

    /* attempt to instantiate wifi event group handle */
    l_wifi_evtgrp_hdl       = xEventGroupCreate();
    ESP_RETURN_ON_FALSE( l_wifi_evtgrp_hdl, ESP_ERR_INVALID_STATE, TAG, "Unable to create wifi event group handle, wifi start failed");
    
    /* attempt to initialize wifi */
    ESP_RETURN_ON_ERROR( esp_wifi_init(&cfg), TAG, "Unable to initialize wifi, wifi start failed" );

    /* attempt to register wifi event handlers */
    ESP_RETURN_ON_ERROR( esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id), TAG, 
                                                        "esp_event_handler_instance_register event any id, wifi start failed" );
    ESP_RETURN_ON_ERROR( esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip), TAG, 
                                                        "esp_event_handler_instance_register event sta got ip, wifi start failed" );

    /* set wifi configuration */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid               = CONFIG_WIFI_SSID,
            .password           = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    /* attempt to set wifi mode, configuration, storage and start wifi */
    ESP_RETURN_ON_ERROR( esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode, wifi start failed" );
    ESP_RETURN_ON_ERROR( esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "esp_wifi_set_config, wifi start failed" );
    ESP_RETURN_ON_ERROR( esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "esp_wifi_set_storage, wifi start failed" );
    ESP_RETURN_ON_ERROR( esp_wifi_start(), TAG, "esp_wifi_start, wifi start failed" );

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t wifi_link_bits = xEventGroupWaitBits(l_wifi_evtgrp_hdl,
            WIFI_EVTGRP_CONNECTED_BIT | WIFI_EVTGRP_DISCONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (wifi_link_bits & WIFI_EVTGRP_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to wifi access point SSID:%s password:%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);

        ret = ESP_OK;
    } else if (wifi_link_bits & WIFI_EVTGRP_DISCONNECTED_BIT) {
        ESP_LOGE(TAG, "Failed to connect to wifi access point SSID:%s, password:%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);

        ret = ESP_ERR_WIFI_NOT_CONNECT;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");

        ret = ESP_ERR_WIFI_STATE;
    }

    return ret;
}

/**
 * @brief Stops WIFI services.  Do not use this function within the WIFI event handler.
 * 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t wifi_stop(void) {
    /* attempt to stop wifi */
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR( err, TAG, "esp_wifi_stop, wifi stop failed" );
    
    /* de-initialize wifi and clear driver and event handlers */
    ESP_RETURN_ON_ERROR( esp_wifi_deinit(), TAG, "esp_wifi_deinit, wifi stop failed" );
    ESP_RETURN_ON_ERROR( esp_wifi_clear_default_wifi_driver_and_handlers(l_sta_netif), TAG, "esp_wifi_clear_default_wifi_driver_and_handlers, wifi stop failed" );

    /* clean-up */
    esp_netif_destroy(l_sta_netif);
    l_sta_netif = NULL;

    return ESP_OK;
}

/**
 * @brief Synchronizes system date-time with time server(s) over SNTP once 
 * connected to an IP network.  This is a blocking function that returns
 * once the system date-time is initialized or it returns an error when 
 * the timeout period has elapsed.
 * 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sntp_synch_time(void) {
    /* set sntp configuration */
    esp_sntp_config_t config = SNTP_DEFAULT_CONFIG;
    config.start             = false;                        // start SNTP service explicitly (after connecting)
    config.sync_cb           = sntp_time_sync_event_handler; // only if we need the notification function

    /* attempt to initialize and start sntp services */
    ESP_RETURN_ON_ERROR( esp_netif_sntp_init(&config), TAG, "Unable to initialize sntp, sntp time synchronization failed" );
    ESP_RETURN_ON_ERROR( esp_netif_sntp_start(), TAG, "Unable to start sntp, sntp time synchronization failed" );

    // attempt to synchronize system time with time server(s)
    esp_err_t ret        = ESP_OK;
    int sntp_retry_count = 1;
    //
    do {
        ret = esp_netif_sntp_sync_wait(SNTP_TIME_SYNC_TIMEOUT_MS / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Waiting for system date-time to be set... (%d/%d)", sntp_retry_count, SNTP_TIME_SYNC_MAXIMUM_RETRY);
    } while (ret == ESP_ERR_TIMEOUT && ++sntp_retry_count <= SNTP_TIME_SYNC_MAXIMUM_RETRY);

    /* de-initialize sntp */
    esp_netif_sntp_deinit();

    /* validate ntp sync results */
    ESP_RETURN_ON_ERROR( ret, TAG, "Unable to synchronize system date-time with time server(s), sntp time synchronization failed" );

    return ESP_OK;
}

/**
 * @brief Gets date-time from network time server(s) and synchronizes system date-time 
 * and time-zone.  This function should only be called once connected to an IP network.
 * This is a blocking function that returns once the system date-time is initialized or 
 * it returns an error when the timeout period has elapsed.
 * 
 * See time-zones list: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
 * 
 * @param[in] timezone system timezone or set this parameter empty to default to UTC.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sntp_get_time(const char* timezone) {
    time_t now; struct tm timeinfo;

    /* set current system time and time information */
    time(&now); localtime_r(&now, &timeinfo);

    // validate system time, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        /* attempt to synchronize system date-time with time server */
        ESP_RETURN_ON_ERROR( sntp_synch_time(), TAG, "Unable to get sntp time from time server(s), get sntp time failed" );
    }

    /* set timezone or leave it in UTC when empty */
    if(timezone == NULL || strlen(timezone) == 0) {
        // attempt to set timezone to UTC
        ESP_RETURN_ON_FALSE( setenv("TZ", "GMT0", 1) == 0, ESP_ERR_INVALID_RESPONSE, TAG, "Unable to set UTC environment time-zone, get sntp time failed");
        tzset();
    } else {
        // attempt set user-defined timezone
        ESP_RETURN_ON_FALSE( setenv("TZ", timezone, 1) == 0, ESP_ERR_INVALID_RESPONSE, TAG, "Unable to set user-defined environment time-zone, get sntp time failed");
        tzset();
    }

    return ESP_OK;
}

/**
 * @brief Starts the MQTT application.  This function should only be called once connected 
 * to an IP network.  This is a blocking function that waits for event bits to be initialized 
 * based on MQTT event results or it returns an error when the timeout period has elapsed. 
 * 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mqtt_app_start(void) {
    esp_err_t     ret = ESP_OK;

    /* attempt to instantiate mqtt event group handle */
    l_mqtt_evtgrp_hdl = xEventGroupCreate();
    ESP_RETURN_ON_FALSE( l_mqtt_evtgrp_hdl, ESP_ERR_INVALID_STATE, TAG, "Unable to create MQTT event group handle, MQTT app start failed");

    /* set mqtt client configuration */
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            //.address.uri = "mqtt://mqtt.eclipseprojects.io:1883",
            .address.uri = "mqtts://mqtt.eclipseprojects.io:8883",
            .verification.certificate = (const char *)mqtt_eclipse_server_pem_start,
        },
    };

    /* attempt to initialize mqtt client handle */
    l_mqtt_client_hdl = esp_mqtt_client_init(&mqtt_cfg);
    ESP_RETURN_ON_FALSE( l_mqtt_client_hdl, ESP_ERR_INVALID_STATE, TAG, "Unable to initialize MQTT client, MQTT app start failed");

    /* attempt to register mqtt client event, the last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    ESP_RETURN_ON_ERROR( esp_mqtt_client_register_event(l_mqtt_client_hdl, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL), TAG, "Unable to register MQTT client event, MQTT app start failed" );
    
    /* attempt to start mqtt client services */
    ESP_RETURN_ON_ERROR( esp_mqtt_client_start(l_mqtt_client_hdl), TAG, "Unable to start MQTT client, MQTT app start failed" );

    /* wait for either an mqtt connected, disconnected, or error event bit to be set */
    EventBits_t mqtt_link_bits = xEventGroupWaitBits(l_mqtt_evtgrp_hdl,
        MQTT_EVTGRP_CONNECTED_BIT | MQTT_EVTGRP_DISCONNECTED_BIT | MQTT_EVTGRP_ERROR_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);
        
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we 
        can test which event actually happened with mqtt link bits. */
    if (mqtt_link_bits & MQTT_EVTGRP_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to MQTT broker");
        l_mqtt_connected = true;
        ret = ESP_OK;
    } else if (mqtt_link_bits & MQTT_EVTGRP_DISCONNECTED_BIT) {
        ESP_LOGE(TAG, "Disconnected from MQTT broker");
        l_mqtt_connected = false;
        ret = MQTT_ERROR_TYPE_CONNECTION_REFUSED;
    } else if (mqtt_link_bits & MQTT_EVTGRP_ERROR_BIT) {
        ESP_LOGE(TAG, "MQTT client error");
        l_mqtt_connected = false;
        ret = MQTT_ERROR_TYPE_NONE;
    } else {
        ESP_LOGE(TAG, "Unexpected MQTT client event");
        l_mqtt_connected = false;
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

/**
 * @brief Stops the MQTT application.  Do not use this function within the MQTT event handler.
 * 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mqtt_app_stop(void) {
    /* attempt to disconnect mqtt client */
    ESP_RETURN_ON_ERROR( esp_mqtt_client_disconnect(l_mqtt_client_hdl), TAG, "Unable to disconnect MQTT client, MQTT app stop failed" );

    /* attempt to stop mqtt client services */
    ESP_RETURN_ON_ERROR( esp_mqtt_client_stop(l_mqtt_client_hdl), TAG, "Unable to stop MQTT client, MQTT app stop failed" );

    /* attempt to unregister mqtt client event */
    ESP_RETURN_ON_ERROR(esp_mqtt_client_unregister_event(l_mqtt_client_hdl, ESP_EVENT_ANY_ID, mqtt_event_handler), TAG, "Unable to unregister MQTT client event, MQTT app stop failed" );

    /* clean-up */
    esp_mqtt_client_destroy(l_mqtt_client_hdl);
    l_mqtt_client_hdl = NULL;
    free(l_mqtt_evtgrp_hdl);
    l_mqtt_evtgrp_hdl = NULL;

    return ESP_OK;
}

/**
 * @brief Task that sends a sensor sample item to the MQTT sensor 
 * sampling queue every 30-seconds once MQTT client is connected.
 * 
 * @param pvParameters Parameters for task.
 */
static void sample_sensor_task( void *pvParameters ) {
    static const float smp_offset_seed = 0.02135;                                   /* to show change in sample values */
    float t_lst_smp = 23.321; float h_lst_smp = 43.454; float p_lst_smp = 1002.33;  /* init last sample values */

    /* set last wake ticks from tick count */
    TickType_t last_wake_ticks = xTaskGetTickCount ();

    /* attempt to create a queue for environmental samples */
    l_mqtt_pub_qos0_queue_hdl = xQueueCreate(MQTT_PUB_QOS0_QUEUE_SIZE, sizeof(environmental_samples_t*));
    if(l_mqtt_pub_qos0_queue_hdl == pdFALSE) {
        ESP_LOGE(TAG, "Unable to create queue for publishing environmental samples");
        esp_restart();
    }

    /* attempt to instantiate samples pointer */
    environmental_samples_t* samples = (environmental_samples_t*)calloc(1, sizeof(environmental_samples_t));
    if(samples == NULL) {
        ESP_LOGE(TAG, "Unable to allocate memory for environmental samples pointer");
        //esp_restart();
    }

    /* enter task loop */
    for ( ;; ) {
        // pause the task per defined wait period
        task_delay_until_sec( &last_wake_ticks, 30 );

        /* validate mqtt link status */
        if(l_mqtt_connected == false) continue;

        /* clear sample values */
        clear_samples(samples);

        /* set sample values */
        samples->temperature = t_lst_smp;
        samples->humidity    = h_lst_smp;
        samples->pressure    = p_lst_smp;

        /* set last sample values with offset seed */
        t_lst_smp += smp_offset_seed;
        h_lst_smp += smp_offset_seed;
        p_lst_smp += smp_offset_seed;

        // queue item and send
        xQueueSend(l_mqtt_pub_qos0_queue_hdl, (void *)&samples, (TickType_t)20);

        ESP_LOGI(TAG, "Queue Sent..");
    }
    vTaskDelete( NULL );
}

/**
 * @brief Task that publishes incoming sensor sampling item queue to
 * an MQTT broker when a queued item is received.  This task waits for 
 * a queued item that is sent from the sample sensor task before 
 * publishing to the MQTT broker.
 * 
 * @param pvParameters Parameters for task.
 */
static void publish_sensor_task( void *pvParameters ) {
    static char fmt_buffer_ta[MQTT_SMP_FORMAT_BUFFER_SIZE];
    static char fmt_buffer_hr[MQTT_SMP_FORMAT_BUFFER_SIZE];
    static char fmt_buffer_pa[MQTT_SMP_FORMAT_BUFFER_SIZE];
    environmental_samples_t* samples = NULL;
    uint32_t free_heap_size_last     = 0;

    /* enter task loop */
    for ( ;; ) {
        /* validate receive queue and handle queued item */
        if(xQueueReceive(l_mqtt_pub_qos0_queue_hdl, &(samples), (TickType_t)20) && samples != NULL) {
            /* validate mqtt link status */
            if(l_mqtt_connected == false) esp_restart();

            ESP_LOGI(TAG, "Queue Received..");
            
            /* format samples for publishing */
            sprintf(fmt_buffer_ta, "%.2f degC", samples->temperature);
            sprintf(fmt_buffer_hr, "%.2f %%",   samples->humidity);
            sprintf(fmt_buffer_pa, "%.2f hPa",  samples->pressure);

            /* publish samples */
            esp_mqtt_client_publish(l_mqtt_client_hdl, "/topic/qos0", fmt_buffer_ta, 0, 0, 0);
            esp_mqtt_client_publish(l_mqtt_client_hdl, "/topic/qos0", fmt_buffer_hr, 0, 0, 0);
            esp_mqtt_client_publish(l_mqtt_client_hdl, "/topic/qos0", fmt_buffer_pa, 0, 0, 0);

            /* monitor consumed bytes for possible memory leak */
            free_heap_size_last = print_free_heap_size(free_heap_size_last);
        }
    }
    vTaskDelete( NULL );
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    /* increment boot counter */
    ++l_boot_count;

    /* print general startup information */
    ESP_LOGI(TAG, "Startup..");
    ESP_LOGI(TAG, "Boot Count: %d", l_boot_count);
    ESP_LOGI(TAG, "Free Memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF Version: %s", esp_get_idf_version());

    /* set log levels */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    /* attempt to initialize nvs flash */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    /* attempt to start wifi services */
    ESP_ERROR_CHECK( wifi_start() );

    // attempt to set system date-time and timezone
    ESP_ERROR_CHECK( sntp_get_time("AST4ADT,M3.2.0,M11.1.0") ); // AST4ADT,M3.2.0,M11.1.0 Atlantic-Standard-Time (Alantic)
    print_sntp_time_servers(); print_system_time(); // print ntp time server(s) and system time
    
    /* attempt to start mqtt app */
    ESP_ERROR_CHECK( mqtt_app_start() );

    /* attempt to start sensor sampling task */
    xTaskCreatePinnedToCore( 
        sample_sensor_task, 
        "smp_snr_tsk", 
        (configMINIMAL_STACK_SIZE * 4), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        &l_sample_sensor_task_hdl,
        APP_CPU_NUM );
    
    /* attempt to start sensor mqtt publishing task */
    xTaskCreatePinnedToCore( 
        publish_sensor_task, 
        "pub_snr_tsk", 
        (configMINIMAL_STACK_SIZE * 4), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        &l_publish_sensor_task_hdl, 
        APP_CPU_NUM );
}

