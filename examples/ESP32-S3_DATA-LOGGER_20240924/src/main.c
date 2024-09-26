/**
 * @file main.c
 * @brief ESP Data-Logger Component
 *
 * i2c sensors: AHT20 + BMP280 
 * 
 * This example takes the parameters 
 *
 *  Sensor Board AHT20 + BMP280 Wiring
 *  VIN -> MCU VDD 3.3V Pin
 *  GNG -> MCU GND Pin
 *  SCL -> MCU SCL Pin
 *  SDA -> MCU SDA Pin
 *  
 *  BMP280 I2C Address: 0x77
 *  AHT20  I2C Address: 0x38
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
#include <driver/i2c_master.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_wifi.h>
#include <esp_sntp.h>
#include <esp_netif_sntp.h>

#include <datalogger.h>
#include <bmp280.h>
#include <ahtxx.h>

#define WIFI_CONNECT_MAX_RETRY          (10)
#define SNTP_TIME_SYNC_TIMEOUT_MS       (2000)
#define SNTP_TIME_SYNC_MAX_RETRY        (10)
#define INET4_ADDRSTRLEN                (15) // (255.255.255.255)

//#define CONFIG_WIFI_SSID                "NOKIA-8764"
//#define CONFIG_WIFI_PASSWORD            "qpLQaC.pbk"
#define CONFIG_WIFI_SSID                "APOLLO"
#define CONFIG_WIFI_PASSWORD            "41F43DA524D6"

#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow

#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 5)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define APP_TAG                         "AHT20+BMP280 [APP]"

// macros
#define CONFIG_I2C_0_MASTER_DEFAULT {                               \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = CONFIG_I2C_0_PORT,        \
        .scl_io_num                     = CONFIG_I2C_0_SCL_IO,      \
        .sda_io_num                     = CONFIG_I2C_0_SDA_IO,      \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

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

// data-table variables
static task_schedule_handle_t sampling_task_sch_hdl;
static datatable_handle_t dt_sample_hdl;
static uint8_t dt_pa_avg_column_index;
static uint8_t dt_ta_avg_column_index;
static uint8_t dt_ta_max_column_index;
static uint8_t dt_ta_min_column_index;
static uint8_t dt_td_avg_column_index;
static uint8_t dt_rh_avg_column_index;
static uint8_t dt_rh_max_ts_column_index;
static uint8_t dt_rh_min_ts_column_index;
static uint8_t dt_wsd_avg_column_index;


static inline void datatable_print_columns(void) {
    if(dt_sample_hdl == NULL || dt_sample_hdl->columns_size == 0) {
        ESP_LOGW(APP_TAG, "data-table columns error: intialize data-table");
        return;
    } else {
        ESP_LOGW(APP_TAG, "data-table (%s) columns:", dt_sample_hdl->name);
    }
    
    for(uint8_t i = 0; i <= dt_sample_hdl->columns_index; i++) {
        datatable_column_t col = dt_sample_hdl->columns[i];
        ESP_LOGW(APP_TAG, "->name (%d): %s", i, col.names[0].name);
    }

    /*
    // print recorded records
    //
    // data-table columns - header
    
    ESP_LOGI(CONFIG_APP_TAG, "ID   %s %s %s %s %s %s",
            dt_sample_hdl->columns[dt_pa_column_index].name, 
            dt_sample_hdl->columns[dt_ta_column_index].name,
            dt_sample_hdl->columns[dt_ta_min_column_index].name,
            dt_sample_hdl->columns[dt_ta_max_column_index].name,
            dt_sample_hdl->columns[dt_td_column_index].name,
            dt_sample_hdl->columns[dt_rh_column_index].name);

    for(uint16_t row_index = 0; row_index < dt_sample_hdl->rows_count; row_index++) {
        float dt_pa     = dt_sample_hdl->rows[row_index].data_columns[dt_pa_column_index].data.float_data.value;
        float dt_ta     = dt_sample_hdl->rows[row_index].data_columns[dt_ta_column_index].data.float_data.value;
        float dt_ta_min = dt_sample_hdl->rows[row_index].data_columns[dt_ta_min_column_index].data.float_data.value;
        float dt_ta_max = dt_sample_hdl->rows[row_index].data_columns[dt_ta_max_column_index].data.float_data.value;
        float dt_td     = dt_sample_hdl->rows[row_index].data_columns[dt_td_column_index].data.float_data.value;
        float dt_rh     = dt_sample_hdl->rows[row_index].data_columns[dt_rh_column_index].data.float_data.value;
        //
        ESP_LOGI(CONFIG_APP_TAG, "[%d]    %.2f      %.2f      %.2f      %.2f      %.2f      %.2f",
            dt_sample_hdl->rows[row_index].data_columns[0].data.id_data.value,
            dt_pa, dt_ta, dt_ta_min, dt_ta_max, dt_td, dt_rh);
    }
    */
}

/*
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    uint64_t time_ms = (uint64_t)tv_now.tv_sec * 1000U + (uint64_t)tv_now.tv_usec / 1000U;
    uint64_t time_us = (uint64_t)tv_now.tv_sec * 1000000U + (uint64_t)tv_now.tv_usec;

    ESP_LOGI(TAG, "UNIX time in mseconds: %lld", time_ms);
    ESP_LOGI(TAG, "UNIX time in useconds: %lld", time_us);
*/

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
    printf("Time set to:          %s", asctime(&initial_time));
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
    // 'new_time' now contains the current time components
    printf("Current time:         %s", asctime(&new_time)); 
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

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(net_event_group_hdl,
            NET_EVENT_GROUP_WIFI_CONNECTED_BIT | NET_EVENT_GROUP_WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
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

static inline void sntp_obtain_time(void) {
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

        sntp_obtain_time();

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

static void i2c_0_task( void *pvParameters ) {
    time_into_interval_handle_t time_into_interval_hdl;
    float                       ta; 
    float                       td; 
    float                       rh;
    float                       pa;
    char                        deg_char        = 176;
    uint64_t                    start_time      = 0;
    uint64_t                    end_time        = 0;
    int64_t                     delta_time      = 0;

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
        ESP_LOGI(APP_TAG, "ntp event group - ntp synchronization failed");

        // abort
    }
    
    // instantiate a new time into interval handle - task system clock synchronization
    time_into_interval_new(DATALOGGER_TIME_INTERVAL_MIN, 5, 0, &time_into_interval_hdl);
    if (time_into_interval_hdl == NULL) ESP_LOGE(APP_TAG, "time_into_new_interval, new time into interval handle failed");
    
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize bmp280 i2c device configuration
    i2c_bmp280_config_t         bmp280_dev_cfg = I2C_BMP280_CONFIG_DEFAULT;
    i2c_bmp280_handle_t         bmp280_dev_hdl;
    //
    // initialize aht20 i2c device configuration
    i2c_ahtxx_config_t          aht20_dev_cfg = I2C_AHT2X_CONFIG_DEFAULT;
    i2c_ahtxx_handle_t          aht20_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(APP_TAG, "i2c0 i2c_bus_create new master bus handle failed");
    //
    // init i2c devices
    //
    // bmp280 init device
    i2c_bmp280_init(i2c0_bus_hdl, &bmp280_dev_cfg, &bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) ESP_LOGE(APP_TAG, "i2c0 i2c_bus_device_create bmp280 handle init failed");
    //
    // aht20 init device
    i2c_ahtxx_init(i2c0_bus_hdl, &aht20_dev_cfg, &aht20_dev_hdl);
    if (aht20_dev_hdl == NULL) ESP_LOGE(APP_TAG, "i2c0 i2c_bus_device_create aht2x handle init failed");
    //
    // 
    //
    // task loop entry point
    for ( ;; ) {
        // set start timer
        start_time = esp_timer_get_time(); 
        //
        // delay task until task schedule condition is valid
        task_schedule_delay(sampling_task_sch_hdl);
        //
        //
        ESP_LOGI(APP_TAG, "######################## AHT20+BMP280 - START #########################");
        //
        ESP_LOGI(APP_TAG, "free memory:  %lu bytes", esp_get_free_heap_size());
        //
        // handle bmp280 sensor
        if(i2c_bmp280_get_pressure(bmp280_dev_hdl, &pa) != 0) {
            ESP_LOGE(APP_TAG, "i2c_bmp280_get_pressure failed");
        } else {
            // convert pressure to hPa
            pa = pa / 100;
            //
            ESP_LOGI(APP_TAG, "pressure:     %.1f hPa", pa);
        }
        //
        // handle aht2x sensor
        if(i2c_ahtxx_get_measurements(aht20_dev_hdl, &ta, &rh, &td) != 0) {
            ESP_LOGI(APP_TAG, "i2c_ahtxx_get_measurements failed");
        } else {
            ESP_LOGI(APP_TAG, "air:          %.2f%cC", ta, deg_char);
            ESP_LOGI(APP_TAG, "dew-point:    %.2f%cC", td, deg_char);
            ESP_LOGI(APP_TAG, "humidity:     %.2f %%", rh);
        }
        //
        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_sample_hdl, dt_pa_avg_column_index, pa);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_avg_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_min_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_max_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_td_avg_column_index, td);
        datatable_push_float_sample(dt_sample_hdl, dt_rh_avg_column_index, rh);
        datatable_push_float_sample(dt_sample_hdl, dt_rh_min_ts_column_index, rh);
        datatable_push_float_sample(dt_sample_hdl, dt_rh_max_ts_column_index, rh);
        datatable_push_vector_sample(dt_sample_hdl, dt_wsd_avg_column_index, 210, 1.45);
        //
        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_sample_hdl);
        //
        // print data-table in json format at specified interval
        
        if(time_into_interval(time_into_interval_hdl)) {
            char *dt_json = "";
            datatable_to_json(dt_sample_hdl, dt_json);
	        ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_json);
        }
    
        //
        //
        ESP_LOGI(APP_TAG, "######################## AHT20+BMP280 - END ###########################");
        //
        // set end timer
        end_time = esp_timer_get_time();
        //
        // compute delta time 
        delta_time = end_time - start_time;
        //
        ESP_LOGI(APP_TAG, "Task Duration: %llu us / %llu ms", delta_time, delta_time / 1000);
    }
    //
    // free up task resources and remove task from stack
    i2c_bmp280_rm( bmp280_dev_hdl );    // remove bmp280 device from master i2c bus
    i2c_ahtxx_rm( aht20_dev_hdl );      // remove aht20 device from master i2c bus
    i2c_del_master_bus( i2c0_bus_hdl ); // delete master i2c bus
    time_into_interval_del( time_into_interval_hdl ); //delete time into interval
    vTaskDelete( NULL );
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

    ESP_ERROR_CHECK( wifi_start() );

    sntp_time_sync_start();

    // initialize system time - manually populated time structure
    //set_time();
    //get_time();

    // data-table testing
    //
    // create a new task schedule handle - task system clock synchronization
    task_schedule_new(DATALOGGER_TIME_INTERVAL_SEC, 10, 0, &sampling_task_sch_hdl);
    if (sampling_task_sch_hdl == NULL) ESP_LOGE(APP_TAG, "task_schedule_new, new task schedule handle failed");
    //
    // create a new data-table handle
    datatable_new("Tbl_1-Min", 9, 10, DATALOGGER_TIME_INTERVAL_MIN, 1, 0, sampling_task_sch_hdl, DATATABLE_DATA_STORAGE_MEMORY_RING, &dt_sample_hdl);
    if (dt_sample_hdl == NULL) ESP_LOGE(APP_TAG, "datatable_new, new data-table handle failed");
    //
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Pa_1-Min", &dt_pa_avg_column_index);                 // column index 2
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_avg_column_index);                 // column index 3
    // add float minimum column to data-table
    datatable_add_float_min_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_min_column_index);                 // column index 4
    // add float maximum column to data-table
    datatable_add_float_max_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_max_column_index);                 // column index 5
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Td_1-Min", &dt_td_avg_column_index);                 // column index 6
    // add float average column to data-table
    datatable_add_float_avg_column(dt_sample_hdl, "Rh_1-Min", &dt_rh_avg_column_index);                 // column index 7
    // add float minimum timestamp column to data-table
    datatable_add_float_min_ts_column(dt_sample_hdl, "Rh_1-Min", &dt_rh_min_ts_column_index);           // column index 8
    // add float maximum timestamp column to data-table
    datatable_add_float_max_ts_column(dt_sample_hdl, "Rh_1-Min", &dt_rh_max_ts_column_index);           // column index 9
    // add vector average column to data-table
    datatable_add_vector_avg_column(dt_sample_hdl, "Wd_1-Min", "Ws_1-Min", &dt_wsd_avg_column_index);   // column index 10
    //
    ESP_LOGW(APP_TAG, "data-table id column index:        %d", 0);
    ESP_LOGW(APP_TAG, "data-table ts column index:        %d", 1);
    ESP_LOGW(APP_TAG, "data-table pa column index:        %d", dt_pa_avg_column_index);
    ESP_LOGW(APP_TAG, "data-table ta column index:        %d", dt_ta_avg_column_index);
    ESP_LOGW(APP_TAG, "data-table ta-min column index:    %d", dt_ta_min_column_index);
    ESP_LOGW(APP_TAG, "data-table ta-max column index:    %d", dt_ta_max_column_index);
    ESP_LOGW(APP_TAG, "data-table td column index:        %d", dt_td_avg_column_index);
    ESP_LOGW(APP_TAG, "data-table rh column index:        %d", dt_rh_avg_column_index);
    ESP_LOGW(APP_TAG, "data-table rh-min-ts column index: %d", dt_rh_min_ts_column_index);
    ESP_LOGW(APP_TAG, "data-table rh-max-ts column index: %d", dt_rh_max_ts_column_index);
    ESP_LOGW(APP_TAG, "data-table ws-wd column index:     %d", dt_wsd_avg_column_index);
    //
    // print data-table columns
    //datatable_print_columns();

    char *dt_json = "";
    datatable_to_json(dt_sample_hdl, dt_json);
	ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_json);

    
    xTaskCreatePinnedToCore( 
        i2c_0_task, 
        CONFIG_I2C_0_TASK_NAME, 
        CONFIG_I2C_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_I2C_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    
}