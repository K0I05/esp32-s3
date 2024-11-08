/**
 * @file main.c
 * @brief ESP Data-Logger Component
 *
 * 
 * CTRL + SHIFT + P
 * pio run -t menufconfig
 * k & l keys for up or down
 * OR
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * 
 * `pio system prune` to save disk space
 * 
 * 
 * There is a new version 6.1.16 of PlatformIO available.
 * Please upgrade it via `platformio upgrade` or `python -m pip install -U platformio` command.
 * Changes: https://docs.platformio.org/en/latest/history.html
 * 
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_wifi.h>
#include <esp_sntp.h>
#include <esp_netif_sntp.h>

#include <datatable.h>


#define WIFI_CONNECT_MAX_RETRY          (10)
#define SNTP_TIME_SYNC_TIMEOUT_MS       (2000)
#define SNTP_TIME_SYNC_MAX_RETRY        (10)
#define INET4_ADDRSTRLEN                (15) // (255.255.255.255)

#define CONFIG_WIFI_SSID                "YOUR SSID"
#define CONFIG_WIFI_PASSWORD            "SSID KEY"


#define CONFIG_I2C_0_TASK_NAME          "dt_1min_smp_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 5)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define APP_TAG                         "DATA-LOGGER [APP]"

/* function and subroutine definitions */
static inline void set_time(void);
static inline void get_time(void);
static inline void stnp_print_servers(void);
static inline void print_time_into_interval_event(time_into_interval_handle_t time_into_interval_handle);
static inline void datatable_event_handler(void *handle, datatable_event_t event);
static inline void stnp_time_sync_notification(struct timeval *tv);
static inline void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static inline esp_err_t wifi_start(void);
static inline esp_err_t wifi_stop(void);
static inline void sntp_synch_time(void);
static inline void sntp_time_sync_start(void);
static inline void nvs(void);

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t   ntp_event_group_hdl = NULL;
static EventGroupHandle_t   net_event_group_hdl = NULL;
static esp_netif_t         *sta_netif           = NULL;
static int                  wifi_retry          = 0;

/* The network event group allows multiple bits for each event, but we only care about 2 events:
 *
 * 1 - connected to the AP with an IP
 * 2 - failed to connect to AP after the maximum amount of retries 
 */
static const int NET_EVENT_GROUP_WIFI_CONNECTED_BIT    = BIT0;
static const int NET_EVENT_GROUP_WIFI_FAIL_BIT         = BIT1;

/* The ntp event group allows multiple bits for each event, but we only care about 2 events:
 *
 * 1 - NTP time synchronized
 * 2 - NTP time synchronization failed
 */
static const int NTP_EVENT_GROUP_NTP_SYNCHRONIZED_BIT  = BIT0;
static const int NTP_EVENT_GROUP_NTP_SYNCH_FAIL_BIT    = BIT1;

// data-table variables for the example - independent of data-logger
static datatable_handle_t       dt_1min_hdl;          /* example data-table handle */
static datatable_config_t       dt_1min_cfg = {       /* example data-table configuration */
    .name                       = "1min_tbl",
    .data_storage_type          = DATATABLE_DATA_STORAGE_MEMORY_RING,
    .columns_size               = 7,
    .rows_size                  = 10,
    .sampling_config            = { /* 10-sec sampling */
        .interval_type          = TIME_INTO_INTERVAL_SEC,
        .interval_period        = 10,
        .interval_offset        = 0
    },
    .processing_config          = { /* 1-min processing */
        .interval_type          = TIME_INTO_INTERVAL_MIN,
        .interval_period        = 1,
        .interval_offset        = 0
    },
    .event_handler              = datatable_event_handler
};
static uint8_t                  dt_1min_pa_avg_col_index;     /* data-table average atmospheric pressure (pa-avg) column index reference */
static uint8_t                  dt_1min_ta_avg_col_index;     /* data-table average air temperature (ta-avg) column index reference */
static uint8_t                  dt_1min_ta_max_col_index;     /* data-table minimum air temperature (ta-min) column index reference */
static uint8_t                  dt_1min_ta_min_col_index;     /* data-table maximum air temperature (ta-max) column index reference */
static uint8_t                  dt_1min_hr_avg_col_index;     /* data-table average relative humidity (hr-avg) column index reference */
static uint8_t                  dt_1min_td_avg_col_index;     /* data-table average dew-point temperature (td-avg) column index reference */
static uint8_t                  dt_1min_wsd_avg_col_index;    /* data-table average (vector) wind speed and direction (wsd-avg) column index reference */

int32_t restart_counter = 0;
/* 

 simulation samples for pressure and temperature

 10-sec sampling = 6 samples per min = 10-min 

*/
static uint8_t samples_index = 0;

static const uint8_t samples_size  = 6 * 10;

static const float pa_samples[] = { 1001.34, 1001.35, 1001.35, 1001.34, 1001.35, 1001.36,
                                    1001.35, 1001.34, 1001.36, 1001.36, 1001.35, 1001.35,
                                    1001.36, 1001.36, 1001.35, 1001.36, 1001.37, 1001.36,
                                    1001.35, 1001.35, 1001.34, 1001.35, 1001.34, 1001.34,
                                    1001.33, 1001.34, 1001.33, 1001.33, 1001.32, 1001.33,
                                    1001.32, 1001.32, 1001.31, 1001.32, 1001.31, 1001.30,
                                    1001.31, 1001.30, 1001.29, 1001.30, 1001.29, 1001.28,
                                    1001.29, 1001.28, 1001.28, 1001.27, 1001.28, 1001.27,
                                    1001.26, 1001.27, 1001.26, 1001.26, 1001.25, 1001.26,
                                    1001.25, 1001.25, 1001.24, 1001.24, 1001.25, 1001.24 };

static const float ta_samples[] = { 22.34, 22.35, 22.35, 22.34, 22.35, 22.36,
                                    22.35, 22.34, 22.36, 22.36, 22.35, 22.35,
                                    22.36, 22.36, 22.35, 22.36, 22.37, 22.36,
                                    22.35, 22.35, 22.34, 22.35, 22.34, 22.34,
                                    22.33, 22.34, 22.33, 22.33, 22.32, 22.33,
                                    22.32, 22.32, 22.31, 22.32, 22.31, 22.30,
                                    22.31, 22.30, 22.29, 22.30, 22.29, 22.28,
                                    22.29, 22.28, 22.28, 22.27, 22.28, 22.27,
                                    22.26, 22.27, 22.26, 22.26, 22.25, 22.26,
                                    22.25, 22.25, 22.24, 22.24, 22.25, 22.24 };

static const float td_samples[] = { 20.34, 20.35, 20.35, 20.34, 20.35, 20.36,
                                    20.35, 20.34, 20.36, 20.36, 20.35, 20.35,
                                    20.36, 20.36, 20.35, 20.36, 20.37, 20.36,
                                    20.35, 20.35, 20.34, 20.35, 20.34, 20.34,
                                    20.33, 20.34, 20.33, 20.33, 20.32, 20.33,
                                    20.32, 20.32, 20.31, 20.32, 20.31, 20.30,
                                    20.31, 20.30, 20.29, 20.30, 20.29, 20.28,
                                    20.29, 20.28, 20.28, 20.27, 20.28, 20.27,
                                    20.26, 20.27, 20.26, 20.26, 20.25, 20.26,
                                    20.25, 20.25, 20.24, 20.24, 20.25, 20.24 };

static const int16_t hr_samples[]={ 50, 50, 51, 51, 51, 51,
                                    52, 52, 53, 53, 53, 53,
                                    54, 54, 55, 55, 55, 56,
                                    56, 56, 57, 57, 57, 58,
                                    58, 58, 58, 57, 57, 57,
                                    56, 56, 56, 56, 55, 55,
                                    55, 55, 54, 54, 54, 53,
                                    53, 53, 53, 53, 52, 52,
                                    52, 51, 51, 51, 50, 50,
                                    50, 49, 49, 49, 48, 48 };


static inline void set_time( void ) {
    // Prepare the broken-down time.
    // See https://en.cppreference.com/w/c/chrono/tm
    struct tm initial_time = {
        .tm_year    = 2024,
        .tm_mon     = 8,
        .tm_mday    = 13,
        .tm_hour    = 14,
        .tm_min     = 00,
        .tm_sec     = 50
    };
    // Convert to Unix time.
    // See https://en.cppreference.com/w/c/chrono/mktime
    time_t initial_unix_time = mktime(&initial_time);
    // Convert to 'struct timeval'
    struct timeval initial_timeval = {
        .tv_sec     = initial_unix_time,
        .tv_usec    = 0
    };
    // Set system time.
    // See https://linux.die.net/man/2/settimeofday
    int err = settimeofday(&initial_timeval, NULL);
    assert(err == 0);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &initial_time);
    ESP_LOGW(APP_TAG, "Time set to:          %s", strftime_buf);
}

static inline void get_time( void ) {
    // Get current time as 'struct timeval'.
    // See https://linux.die.net/man/2/gettimeofday
    struct timeval new_timeval;
    int err = gettimeofday(&new_timeval, NULL);
    assert(err == 0);
    // Extract Unix time
    time_t new_unix_time = new_timeval.tv_sec;
    // Convert to broken-down time
    // See https://en.cppreference.com/w/c/chrono/localtime
    struct tm new_time;
    localtime_r(&new_unix_time, &new_time);
    char strftime_buf[64];
    // 'new_time' now contains the current time components
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &new_time);
    ESP_LOGW(APP_TAG, "Current Time:         %s", strftime_buf);
}

static inline void stnp_print_servers(void) {
    ESP_LOGI(APP_TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < SNTP_MAX_SERVERS; ++i) {
        if (esp_sntp_getservername(i)){
            ESP_LOGI(APP_TAG, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            // we have either IPv4 address, let's print it
            char buff[INET4_ADDRSTRLEN];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET4_ADDRSTRLEN) != NULL)
                ESP_LOGI(APP_TAG, "server %d: %s", i, buff);
        }
    }
}

static inline void print_time_into_interval_event(time_into_interval_handle_t time_into_interval_handle) {
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    time(&now);

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%A %c", &timeinfo);
    ESP_LOGW(APP_TAG, "%s System Time:          %s", time_into_interval_handle->name, strftime_buf);

    time_t next_unix_time = time_into_interval_handle->epoch_timestamp / 1000U;
    localtime_r(&next_unix_time, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%A %c", &timeinfo);
    ESP_LOGW(APP_TAG, "%s Next Event Time:      %s", time_into_interval_handle->name, strftime_buf);
}

static inline void datatable_event_handler(void *handle, datatable_event_t event) {
    if(handle == NULL) return;
    datatable_handle_t datatable_handle = (datatable_handle_t)handle;
    if(datatable_handle != NULL) {
        ESP_LOGW(APP_TAG, "data-table %s event: %s", datatable_handle->name, event.message);
    }
}

static inline void stnp_time_sync_notification(struct timeval *tv) {
    ESP_LOGW(APP_TAG, "Notification of a time synchronization event");
}

static inline void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI(APP_TAG, "wifi_event_handler: %s:%lu", event_base, event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        wifi_retry = 0;
        esp_wifi_connect();
        ESP_LOGI(APP_TAG, "Starting wifi access point connection.");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry < WIFI_CONNECT_MAX_RETRY) {
            esp_wifi_connect();
            wifi_retry++;
        } else {
            xEventGroupSetBits(net_event_group_hdl, NET_EVENT_GROUP_WIFI_FAIL_BIT);
        }
        ESP_LOGI(APP_TAG,"Connect to wifi access point failed... (%d/%d)", wifi_retry, WIFI_CONNECT_MAX_RETRY);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(APP_TAG, "Got ip from wifi access point:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry = 0;
        xEventGroupSetBits(net_event_group_hdl, NET_EVENT_GROUP_WIFI_CONNECTED_BIT);
    }
}

static inline esp_err_t wifi_start( void ) {
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    esp_err_t ret           = ESP_OK;
    net_event_group_hdl     = xEventGroupCreate();
    sta_netif               = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg  = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK( esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id) );
    ESP_ERROR_CHECK( esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid               = CONFIG_WIFI_SSID,
            .password           = CONFIG_WIFI_PASSWORD,
        },
    };

    ESP_LOGI(APP_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_LOGI(APP_TAG, "wifi_start finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or 
        connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The 
        bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(net_event_group_hdl,
            NET_EVENT_GROUP_WIFI_CONNECTED_BIT | NET_EVENT_GROUP_WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can 
        test which event actually happened. */
    if (bits & NET_EVENT_GROUP_WIFI_CONNECTED_BIT) {
        ESP_LOGI(APP_TAG, "Connected to wifi access point SSID:%s password:%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);

        ret = ESP_OK;
    } else if (bits & NET_EVENT_GROUP_WIFI_FAIL_BIT) {
        ESP_LOGE(APP_TAG, "Failed to connect to wifi access point SSID:%s, password:%s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);

        ret = ESP_ERR_WIFI_NOT_CONNECT;
    } else {
        ESP_LOGE(APP_TAG, "UNEXPECTED EVENT");

        ret = ESP_ERR_WIFI_STATE;
    }

    return ret;
}

static inline esp_err_t wifi_stop(void) {
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return ESP_OK;
    }
    ESP_ERROR_CHECK( err );
    ESP_ERROR_CHECK( esp_wifi_deinit() );
    ESP_ERROR_CHECK( esp_wifi_clear_default_wifi_driver_and_handlers(sta_netif) );
    esp_netif_destroy(sta_netif);
    sta_netif = NULL;

    return ESP_OK;
}

static inline void sntp_synch_time(void) {
    ESP_LOGI(APP_TAG, "Initializing SNTP");

    ntp_event_group_hdl         = xEventGroupCreate();
    esp_sntp_config_t config    = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.start                = false;                        // start SNTP service explicitly (after connecting)
    config.sync_cb              = stnp_time_sync_notification;  // only if we need the notification function

    ESP_ERROR_CHECK( esp_netif_sntp_init(&config) );

    ESP_LOGI(APP_TAG, "Starting SNTP");

    ESP_ERROR_CHECK( esp_netif_sntp_start() );

    stnp_print_servers();

    // wait for time to be set
    time_t now          = 0;
    struct tm timeinfo  = { 0 };
    int ntp_retry       = 1;
    esp_err_t ret       = ESP_OK;

    do {
        ret = esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS);

        ESP_LOGI(APP_TAG, "Waiting for system time to be set... (%d/%d)", ntp_retry, SNTP_TIME_SYNC_MAX_RETRY);
    } while (ret == ESP_ERR_TIMEOUT && ++ntp_retry < SNTP_TIME_SYNC_MAX_RETRY);
    
    if(ret != ESP_OK) {
        xEventGroupSetBits(ntp_event_group_hdl, NTP_EVENT_GROUP_NTP_SYNCH_FAIL_BIT);

        ESP_LOGE(APP_TAG, "Unable to synchronize system date-time");
    } else {
        xEventGroupSetBits(ntp_event_group_hdl, NTP_EVENT_GROUP_NTP_SYNCHRONIZED_BIT);
    }

    time(&now);
    localtime_r(&now, &timeinfo);

    esp_netif_sntp_deinit();
}

static inline void sntp_time_sync_start(void) {
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(APP_TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");

        sntp_synch_time();

        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(APP_TAG, "The current date/time in New York is: %s", strftime_buf);

    // Set timezone to China Standard Time
    setenv("TZ", "CST-8", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(APP_TAG, "The current date/time in Shanghai is: %s", strftime_buf);

    // Set timezone to Eastern Standard Time (Atlantic) and print local time
    setenv("TZ", "EST4EDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(APP_TAG, "The current date/time in Halifax is: %s", strftime_buf);

}

static void dt_1min_smp_task( void *pvParameters ) {
    time_into_interval_handle_t dt_1min_tii_5min_hdl;
    time_into_interval_config_t dt_1min_tii_5min_cfg = {
        .name               = "tii_5min",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 5 * 60,
        .interval_offset    = 10
    };

    uint32_t free_heap_size_start = 0;

    // create a new time-into-interval handle - task system clock synchronization
    time_into_interval_init(&dt_1min_tii_5min_cfg, &dt_1min_tii_5min_hdl);
    if (dt_1min_tii_5min_hdl == NULL) ESP_LOGE(APP_TAG, "time_into_interval_new, new time-into-interval handle failed"); 

    // task loop entry point
    for ( ;; ) {
        // set start timer
        uint64_t start_time = esp_timer_get_time(); 
        
        /* delay data-table sampling task until sampling interval has elapsed */
        datatable_sampling_task_delay(dt_1min_hdl);
        
        // print start of sampling process
        ESP_LOGI(APP_TAG, "######################## DATA-LOGGER - START #########################");
        
        // print free memory available and boot count
        uint32_t heap_size = esp_get_free_heap_size();
        if(free_heap_size_start == 0) free_heap_size_start = heap_size;
        //ESP_LOGI(APP_TAG, "Free Memory Start:   %lu bytes", free_heap_size_start);
        ESP_LOGI(APP_TAG, "Free Memory:         %lu bytes (%lu bytes Consumed)", heap_size, free_heap_size_start - heap_size);
        ESP_LOGI(APP_TAG, "Reboot Count:        %li", restart_counter);
        ESP_LOGI(APP_TAG, "Records Count:       %u", dt_1min_hdl->rows_count);

        // print next data-table sampling time-into-interval event
        ESP_LOGI(APP_TAG, "Data-Table Sampling....");
        print_time_into_interval_event(dt_1min_hdl->sampling_tii_handle);

        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_1min_hdl, dt_1min_pa_avg_col_index, pa_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_avg_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_min_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_max_col_index, ta_samples[samples_index]);
        datatable_push_int16_sample(dt_1min_hdl, dt_1min_hr_avg_col_index, hr_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_td_avg_col_index, td_samples[samples_index]);
        datatable_push_vector_sample(dt_1min_hdl, dt_1min_wsd_avg_col_index, 210, 1.45);  // static values for now (TODO: make them random)

        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_1min_hdl);

        // print next data-table processing time-into-interval event
        ESP_LOGI(APP_TAG, "Data-Table Processing....");
        print_time_into_interval_event(dt_1min_hdl->processing_tii_handle);

        // increment samples index
        samples_index += 1;

        // check samples index against samples size for reset if applicable
        if(samples_index >= samples_size) samples_index = 0; // reset index
        
        /* serialize data-table and output in json string format every 
        5-minutes (i.e. 12:00:00, 12:05:00, 12:10:00, etc.) */
        // omit for now - memory leak hunting...
        if(time_into_interval(dt_1min_tii_5min_hdl)) {
            
            // create root object for data-table
            cJSON *dt_1min_json = cJSON_CreateObject();

            // convert the data-table to json object
            datatable_to_json(dt_1min_hdl, &dt_1min_json);

            // render json data-table object to text and print
            char *dt_1min_json_str = cJSON_Print(dt_1min_json);
            ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_1min_json_str);

            // free-up json resources
            cJSON_Delete(dt_1min_json);
            cJSON_free(dt_1min_json_str);
            
           ESP_LOGI(APP_TAG, "JSON Data-Table Output....");
        }
        

        // print next data-table output time-into-interval event
        ESP_LOGI(APP_TAG, "Data-Table Output....");
        print_time_into_interval_event(dt_1min_tii_5min_hdl);
    
        // set end timer
        uint64_t end_time = esp_timer_get_time();
        
        // compute task duration (delta time) in micro-seconds
        int64_t delta_time = end_time - start_time;
        
        // print task duration in micro-seconds and milli-seconds.
        ESP_LOGI(APP_TAG, "Task Duration: %llu us / %llu ms", delta_time, delta_time / 1000);

        // print end of sampling process
        ESP_LOGI(APP_TAG, "######################## DATA-LOGGER - END ###########################");
    }
    //
    // free up task resources
    time_into_interval_del( dt_1min_tii_5min_hdl ); //delete time-into-interval handle
    vTaskDelete( NULL );
}

static inline void nvs( void ) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(APP_TAG, "NVS error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        /* read */
        ESP_LOGI(APP_TAG, "NVS reading restart counter...");
        //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
        restart_counter = 0;
        err = nvs_get_i32(nvs_handle, "restart_counter", &restart_counter);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(APP_TAG, "NVS restart counter = %" PRIu32, restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGI(APP_TAG, "NVS value is not initialized!");
                break;
            default :
                ESP_LOGI(APP_TAG, "NVS error (%s) reading!", esp_err_to_name(err));
        }

        /* write */
        ESP_LOGI(APP_TAG, "NVS updating restart counter..");
        restart_counter++;
        err = nvs_set_i32(nvs_handle, "restart_counter", restart_counter);
        if(err != ESP_OK) {
            ESP_LOGI(APP_TAG, "Failed!!");
        } else {
            ESP_LOGI(APP_TAG, "Done");
        }

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(APP_TAG, "NVS committing updates..");
        err = nvs_commit(nvs_handle);
        if(err != ESP_OK) {
            ESP_LOGI(APP_TAG, "Failed!!");
        } else {
            ESP_LOGI(APP_TAG, "Done");
        }

        // Close
        nvs_close(nvs_handle);
    }
}

void app_main( void ) {
    ESP_LOGI(APP_TAG, "Startup..");
    ESP_LOGI(APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(APP_TAG, "IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(APP_TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    nvs();

    // attempt to start wifi and synchronize system clock via SNTP
    ESP_ERROR_CHECK( wifi_start() );
    sntp_time_sync_start();



    // initialize system time - manually populated time structure
    //set_time();
    //get_time();


    
    // wait for an event group bit to be set.
    EventBits_t bits = xEventGroupWaitBits(net_event_group_hdl,
            NTP_EVENT_GROUP_NTP_SYNCHRONIZED_BIT | NTP_EVENT_GROUP_NTP_SYNCH_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // determine which event group bit was set
    if (bits & NTP_EVENT_GROUP_NTP_SYNCHRONIZED_BIT) {
        ESP_LOGI(APP_TAG, "ntp event group - ntp synchronized");
    } else if (bits & NTP_EVENT_GROUP_NTP_SYNCH_FAIL_BIT) {
        ESP_LOGE(APP_TAG, "ntp event group - ntp synchronization failed");
        esp_restart();
    }
    

    // data-table testing
    //
    // initialize a data-table handle for the example
    datatable_init(&dt_1min_cfg, &dt_1min_hdl);   
    if (dt_1min_hdl == NULL) {
        ESP_LOGE(APP_TAG, "datatable_init, data-table handle initialization failed");
        esp_restart();
    }

    // configure data-table columns
    //
    // add float average column to data-table
    datatable_add_float_avg_column(dt_1min_hdl, "Pa_1-Min", &dt_1min_pa_avg_col_index);                 // column index 2 exptected
    // add float average column to data-table
    datatable_add_float_avg_column(dt_1min_hdl, "Ta_1-Min", &dt_1min_ta_avg_col_index);                 // column index 3 exptected
    // add float minimum column to data-table
    datatable_add_float_min_column(dt_1min_hdl, "Ta_1-Min", &dt_1min_ta_min_col_index);                 // column index 4 exptected
    // add float maximum column to data-table
    datatable_add_float_max_column(dt_1min_hdl, "Ta_1-Min", &dt_1min_ta_max_col_index);                 // column index 5 exptected
    // add int16 average column to data-table
    datatable_add_int16_avg_column(dt_1min_hdl, "Hr_1-Min", &dt_1min_hr_avg_col_index);                 // column index 6 exptected
    // add float average column to data-table
    datatable_add_float_avg_column(dt_1min_hdl, "Td_1-Min", &dt_1min_td_avg_col_index);                 // column index 7 exptected
    // add vector average column to data-table
    datatable_add_vector_avg_column(dt_1min_hdl, "Wd_1-Min", "Ws_1-Min", &dt_1min_wsd_avg_col_index);   // column index 8 exptected
    
    // print data-table column indexes
    ESP_LOGW(APP_TAG, "data-table id column index:        %d", 0);
    ESP_LOGW(APP_TAG, "data-table ts column index:        %d", 1);
    ESP_LOGW(APP_TAG, "data-table pa column index:        %d", dt_1min_pa_avg_col_index);
    ESP_LOGW(APP_TAG, "data-table ta column index:        %d", dt_1min_ta_avg_col_index);
    ESP_LOGW(APP_TAG, "data-table ta-min column index:    %d", dt_1min_ta_min_col_index);
    ESP_LOGW(APP_TAG, "data-table ta-max column index:    %d", dt_1min_ta_max_col_index);
    ESP_LOGW(APP_TAG, "data-table hr column index:        %d", dt_1min_hr_avg_col_index);
    ESP_LOGW(APP_TAG, "data-table td column index:        %d", dt_1min_td_avg_col_index);
    ESP_LOGW(APP_TAG, "data-table ws-wd column index:     %d", dt_1min_wsd_avg_col_index);
    
    /*
    // print data-table as a string in json format
    //
    // create root object for data-table
    cJSON *dt_1min_json = cJSON_CreateObject();
    // convert the data-table to json object
    datatable_to_json(dt_1min_hdl, &dt_1min_json);
    // render json data-table object to text and print
    char *dt_1min_json_str = cJSON_Print(dt_1min_json);
    ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_1min_json_str);
    // free-up json resources
    cJSON_Delete(dt_1min_json);
    cJSON_free(dt_1min_json_str);
    */
    


    // create a task that is pinned to core 1 for data-table sampling and processing
    xTaskCreatePinnedToCore( 
        dt_1min_smp_task, 
        CONFIG_I2C_0_TASK_NAME, 
        CONFIG_I2C_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_I2C_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
}