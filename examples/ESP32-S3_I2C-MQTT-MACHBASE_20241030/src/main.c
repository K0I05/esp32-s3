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
 * @brief Sample application that connects to a machbase broker and publishes environmental 
 * samples (atmospheric pressure, air temperature, dewpoint temperature, and relative humidity).
 * Measurements are taken directly from a BMP280 sensor and an SHT40 sensor over the I2C master 
 * bus.  The machbase server is configured with three tables (env_temperature, env_humidity, and
 * env_pressure) and samples are stored by environmental parameter category and type.  The 
 * sampling and publishing rate is once a minute.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 * 
 * C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * 
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
#include <mqtt_client.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>

#include <network_connect.h>

/* components */
#include <time_into_interval.h>
#include <bmp280.h>
#include <sht4x.h>
#include <nvs_ext.h>


/**
 * @brief MQTT definitions
 */

#define MQTT_PUB_ENV_QUEUE_SIZE                 (30)                        /*!< environmental queue size for MQTT publshing */
#define MQTT_PUB_ENV                            "db/append/ENVIRONMENTAL" /*!< environmental for MQTT publshing topic */
#define MQTT_NET_DEVICE_ID                      "CA.NB.AWS.01-1000"         /*!< unique network device identifier (max 50-chars) */
#define MQTT_PUB_MSG_BUFFER_SIZE                (200)                       /*!< buffer size for publishing MQTT messages */   

/**
 * @brief FreeRTOS definitions
 */

#define MINIMAL_STACK_SIZE                      (1024)

/**
 * @brief I2C 0 master bus definitions
 */
#define I2C_0_MASTER_PORT                       I2C_NUM_0
#define I2C_0_MASTER_SDA_IO                     (gpio_num_t)(45) // blue
#define I2C_0_MASTER_SCL_IO                     (gpio_num_t)(48) // yellow

/**
 * @brief macro definitions
 */
#define I2C_0_MASTER_DEFAULT_CONFIG {                               \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = I2C_0_MASTER_PORT,        \
        .scl_io_num                     = I2C_0_MASTER_SCL_IO,      \
        .sda_io_num                     = I2C_0_MASTER_SDA_IO,      \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

/**
 * @brief struct and enum definitions
 */

/**
 * @brief Sample parameter types enumerator.
 */
typedef enum sample_parameters_tag {
    SAMPLE_AIR_TEMPERATURE,       /*!< Air temperature in degrees celsius */
    SAMPLE_DEWPOINT_TEMPERATURE,  /*!< Dewpoint temperature in degrees celsius */
    SAMPLE_RELATIVE_HUMIDITY,     /*!< Relative humidity in percent */
    SAMPLE_ATMOSPHERIC_PRESSURE   /*!< Atmospheric pressure in hecto-pascal */
} sample_parameters_t;

/**
 * @brief Environmental sample structure.  A basic data model to 
 * transmit and receive environmental sample as a queued item.
 */
typedef struct environmental_sample_tag {
    const char*             device_id;      /*!< unique network device identifier */
    uint64_t                timestamp;      /*!< sample time-stamp in nano-seconds */
    sample_parameters_t     parameter;      /*!< sample parameter */
    float                   value;          /*!< sample value */
} environmental_sample_t;


typedef struct system_state_tag {
    uint16_t    reboot_counter;         /*!< number of times system has restarted */
    uint64_t    reboot_timestamp;       /*!< system restart unix epoch timestamp (UTC) in seconds */
    uint64_t    system_uptime;          /*!< up-time in seconds since system restart */
} system_state_t;

/**
 * @brief static constant and global definitions
 */

/* app tag for esp logging */
static const char              *TAG                         = "I2C-MQTT MACHBASE [APP]";

/* global variables */
static TaskHandle_t             s_sample_sensor_task_hdl    = NULL;
static TaskHandle_t             s_publish_sensor_task_hdl   = NULL;
static QueueHandle_t            s_mqtt_pub_env_queue_hdl    = NULL;
static system_state_t          *s_system_state              = NULL;

/**
 * @brief static inline function and subroutine definitions
 */


static inline uint32_t print_free_heap_size(const uint32_t free_heap_size_last);
static inline const char* sample_parameter_to_string(const sample_parameters_t type);
static inline environmental_sample_t* create_sample(const char* device_id, sample_parameters_t type);

/**
 * @brief static function and subroutine definitions
 */

static void sample_sensor_task(void *pvParameters);
static void publish_sensor_task(void *pvParameters);




/**
 * @brief Prints the free heap size in bytes with consumed bytes stats as an
 * esp information log.  This is used to monitor consumed bytes for possible 
 * memory leak(s) in the application.
 * 
 * @param free_heap_size_last Last free heap size in bytes.
 * @return uint32_t Adjusted last free heap size in bytes.
 */
static inline uint32_t print_free_heap_size(const uint32_t free_heap_size_last) {
    uint32_t free_heap_size_start = free_heap_size_last; /* set free heap size start from last */
    uint32_t free_heap_size = esp_get_free_heap_size();  /* set free heap size */
    if(free_heap_size_start == 0) free_heap_size_start = free_heap_size;
    int32_t free_heap_size_delta = free_heap_size_start - free_heap_size;
    if(free_heap_size_delta < 0) { free_heap_size_start = free_heap_size; free_heap_size_delta = 0; }
    ESP_LOGW(TAG, "Free Memory: %lu bytes (%li bytes Consumed)", free_heap_size, free_heap_size_delta);
    return free_heap_size_start;
}

/**
 * @brief Converts `sample_parameters_t` enumerator to a string.
 * 
 * @param type Sample parameter type.
 * @return const char* Sample parameter type as a string.
 */
static inline const char* sample_parameter_to_string(const sample_parameters_t type) {
    switch(type) {
        case SAMPLE_AIR_TEMPERATURE:
            return "Air-Temperature";
        case SAMPLE_DEWPOINT_TEMPERATURE:
            return "Dewpoint-Temperature";
        case SAMPLE_RELATIVE_HUMIDITY:
            return "Relative-Humidity";
        case SAMPLE_ATMOSPHERIC_PRESSURE:
            return "Atmospheric-Pressure";
        default:
            return "-";
    }
}

/**
 * @brief Creates a sample instance by device identifier, parameter type, statistical type, and statistical interval.
 * 
 * @param device_id Unique device identifier.
 * @param type Sample parameter type.
 * @param stat_type Statistical processing type.
 * @param stat_interval Statistical processing interval.
 * @return environmental_sample_t* Sample pointer instance.
 */
static inline environmental_sample_t* create_sample(const char* device_id, sample_parameters_t parameter) {
    environmental_sample_t* sample = (environmental_sample_t*)calloc(1, sizeof(environmental_sample_t));
    if(!sample) return NULL;
    sample->device_id     = device_id;
    sample->parameter     = parameter;
    sample->timestamp     = 0;
    sample->value         = NAN;
    return sample;
}


static inline esp_err_t nvs_write_system_state(system_state_t *system_state) {
    esp_err_t ret = nvs_write_struct("system_state", system_state, sizeof(system_state_t));
    return ret;
}

static inline esp_err_t nvs_read_system_state(system_state_t **system_state) {
    esp_err_t ret = nvs_read_struct("system_state", (void **)system_state, sizeof(system_state_t));
    return ret;
}

static inline void init_system_state(void) {
    s_system_state = (system_state_t*)calloc(1, sizeof(system_state_t));

    /* attempt to read system state structure from nvs */
    if(nvs_read_system_state(&s_system_state) != ESP_OK) {
        /* assume system state structure doesn't exist - initialize attributes */
        s_system_state->reboot_counter  += 1;
        s_system_state->system_uptime    = time_into_interval_get_epoch_timestamp();
        s_system_state->reboot_timestamp = time_into_interval_get_epoch_timestamp();
    } else {
        /* assume system state structure exists - update attributes */
        s_system_state->reboot_counter  += 1;
        s_system_state->system_uptime    = time_into_interval_get_epoch_timestamp() - s_system_state->reboot_timestamp;
        s_system_state->reboot_timestamp = time_into_interval_get_epoch_timestamp();
    }

    /* attempt to write system state structure to nvs */
    nvs_write_system_state(s_system_state);

    /* print system state structure */
    ESP_LOGW(TAG, "Boot Count: %u", s_system_state->reboot_counter);
    ESP_LOGW(TAG, "Up-Time:    %llu", s_system_state->system_uptime);
    ESP_LOGW(TAG, "Timestamp:  %llu", s_system_state->reboot_timestamp);

}

/**
 * @brief Task that sends a sensor sample item to the MQTT sensor 
 * sampling queue every 60-seconds once MQTT client is connected.
 * 
 * @param pvParameters Parameters for task.
 */
static void sample_sensor_task( void *pvParameters ) {
    uint64_t                    epoch_timestamp;
    esp_err_t                   result;
    /* time-into-interval handle and configuration - */
    time_into_interval_handle_t tii_3sec_hdl;
    const time_into_interval_config_t tii_3sec_cfg = {
        .name               = "tii_3sec",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 3,
        .interval_offset    = 0
    };
    /* master i2c 0 bus handle and configuration*/
    const i2c_master_bus_config_t i2c0_master_cfg = I2C_0_MASTER_DEFAULT_CONFIG;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    /* bmp280 i2c device handle and configuration */
    const i2c_bmp280_config_t   bmp280_dev_cfg = I2C_BMP280_CONFIG_DEFAULT;
    i2c_bmp280_handle_t         bmp280_dev_hdl;
    /* sht40 i2c device handle and configuration */
    const i2c_sht4x_config_t    sht4x_dev_cfg = I2C_SHT4X_CONFIG_DEFAULT;
    i2c_sht4x_handle_t          sht4x_dev_hdl;

    /* attempt to create a queue for environmental samples */
    s_mqtt_pub_env_queue_hdl = xQueueCreate(MQTT_PUB_ENV_QUEUE_SIZE, sizeof(environmental_sample_t*));
    if(s_mqtt_pub_env_queue_hdl == pdFALSE) {
        ESP_LOGE(TAG, "Unable to create queue for publishing environmental samples");
        esp_restart();
    }

    /* attempt to initialize a time-into-interval handle - task system clock synchronization */
    time_into_interval_init(&tii_3sec_cfg, &tii_3sec_hdl);
    if (tii_3sec_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize time-into-interval handle");
        esp_restart(); 
    }

    /* attempt to initialize a new i2c 0 master bus handle */
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize i2c 0 master bus handle");
        esp_restart(); 
    }

    /* attempt to initialize a bmp280 device handle */
    i2c_bmp280_init(i2c0_bus_hdl, &bmp280_dev_cfg, &bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize bmp280 device handle");
        esp_restart(); 
    }

    /* attempt to initialize a sht40 device handle */
    i2c_sht4x_init(i2c0_bus_hdl, &sht4x_dev_cfg, &sht4x_dev_hdl);
    if (sht4x_dev_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize sht40 device handle");
        esp_restart(); 
    }

    /* attempt to create air temperature sample pointer */
    environmental_sample_t* ta_sample = create_sample(MQTT_NET_DEVICE_ID, SAMPLE_AIR_TEMPERATURE);
    if(ta_sample == NULL) {
        ESP_LOGE(TAG, "Unable to allocate memory for environmental air temperature sample pointer");
        esp_restart();
    }

    /* attempt to create dewpoint temperature sample pointer */
    environmental_sample_t* td_sample = create_sample(MQTT_NET_DEVICE_ID, SAMPLE_DEWPOINT_TEMPERATURE);
    if(td_sample == NULL) {
        ESP_LOGE(TAG, "Unable to allocate memory for environmental dewpoint temperature sample pointer");
        esp_restart();
    }

    /* attempt to create relative humidity sample pointer */
    environmental_sample_t* hr_sample = create_sample(MQTT_NET_DEVICE_ID, SAMPLE_RELATIVE_HUMIDITY);
    if(hr_sample == NULL) {
        ESP_LOGE(TAG, "Unable to allocate memory for environmental relative humidity sample pointer");
        esp_restart();
    }

    /* attempt to create atmospheric pressure sample pointer */
    environmental_sample_t* pa_sample = create_sample(MQTT_NET_DEVICE_ID, SAMPLE_ATMOSPHERIC_PRESSURE);
    if(pa_sample == NULL) {
        ESP_LOGE(TAG, "Unable to allocate memory for environmental atmospheric pressure sample pointer");
        esp_restart();
    }

    /* enter task loop */
    for ( ;; ) {
        /* time-into-interval task delay (1-min sampling interval) */
        time_into_interval_delay(tii_3sec_hdl);

        /* validate mqtt link status */
        if(mqtt_connected == false) continue;

        /* get timestamp value from last time-into-interval event */
        time_into_interval_get_last_event(tii_3sec_hdl, &epoch_timestamp); // msec
        epoch_timestamp = 1000000U * epoch_timestamp; // convert msec to nsec

        /* set timestamp in nano-seconds for each sample */
        ta_sample->timestamp = epoch_timestamp;
        td_sample->timestamp = epoch_timestamp;
        hr_sample->timestamp = epoch_timestamp;
        pa_sample->timestamp = epoch_timestamp;

        /* handle sht40 device sampling */
        result = i2c_sht4x_get_measurements(sht4x_dev_hdl, &ta_sample->value, &hr_sample->value, &td_sample->value);
        if(result != ESP_OK) {
            ta_sample->value = NAN, hr_sample->value = NAN, td_sample->value = NAN;
            ESP_LOGE(TAG, "SHT40 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "SHT40 Air Temperature:       %.2f C", ta_sample->value);
            ESP_LOGI(TAG, "SHT40 Relative Humidity:     %.2f %%", hr_sample->value);
            ESP_LOGI(TAG, "SHT40 Dewpoint Temperature:  %.2f C", td_sample->value);
        }

        /* settling delay between i2c device transactions on the same i2c master bus */
        vTaskDelay(pdMS_TO_TICKS(50));

        /* handle bmp280 device sampling */
        result = i2c_bmp280_get_pressure(bmp280_dev_hdl, &pa_sample->value);
        if(result != ESP_OK) {
            pa_sample->value = NAN;
            ESP_LOGE(TAG, "BMP280 device read failed (%s)", esp_err_to_name(result));
        } else {
            pa_sample->value = pa_sample->value / 100;
            ESP_LOGI(TAG, "BMP280 Atmospheric Pressure: %.2f hPa", pa_sample->value);
        }

        // attempt to queue ta sample item and send
        if(xQueueSend(s_mqtt_pub_env_queue_hdl, (void *)&ta_sample, (TickType_t)0) != pdTRUE) {
            ESP_LOGE(TAG, "Unable to Send Publish Environmental %s Sample Queue", sample_parameter_to_string(ta_sample->parameter));
        }

        // attempt to queue hr sample item and send
        if(xQueueSend(s_mqtt_pub_env_queue_hdl, (void *)&hr_sample, (TickType_t)0) != pdTRUE) {
            ESP_LOGE(TAG, "Unable to Send Publish Environmental %s Sample Queue", sample_parameter_to_string(hr_sample->parameter));
        }

        // attempt to queue td sample item and send
        if(xQueueSend(s_mqtt_pub_env_queue_hdl, (void *)&td_sample, (TickType_t)0) != pdTRUE) {
            ESP_LOGE(TAG, "Unable to Send Publish Environmental %s Sample Queue", sample_parameter_to_string(td_sample->parameter));
        }

        // attempt to queue pa sample item and send
        if(xQueueSend(s_mqtt_pub_env_queue_hdl, (void *)&pa_sample, (TickType_t)0) != pdTRUE) {
            ESP_LOGE(TAG, "Unable to Send Publish Environmental %s Sample Queue", sample_parameter_to_string(pa_sample->parameter));
        }
    }
    /* free resources */
    i2c_bmp280_rm( bmp280_dev_hdl );
    i2c_sht4x_rm( sht4x_dev_hdl ); 
    i2c_del_master_bus( i2c0_bus_hdl );
    vTaskDelete( NULL );
}

/**
 * @brief Task that publishes incoming sensor sampling item queue to
 * an MQTT broker when a queued item is received.  This task waits for 
 * a queued item that is sent from the sample sensor task before 
 * publishing to the MQTT broker.
 * 
 * @note This task will restart the system if the MQTT client disconnects.
 * 
 * @param pvParameters Parameters for task.
 */
static void publish_sensor_task( void *pvParameters ) {
    environmental_sample_t* sample = NULL;
    //uint32_t free_heap_size_last   = 0;

    /* enter task loop */
    for ( ;; ) {
        /* validate receive queue and handle queued item */
        if(xQueueReceive(s_mqtt_pub_env_queue_hdl, &(sample), (TickType_t)10) == pdTRUE && sample != NULL) {
            /* validate mqtt link status */
            if(mqtt_connected == false) esp_restart();

            //ESP_LOGI(TAG, "Publish Environmental %s Sample Queue Received....", sample_param_type_to_string(sample->type));

            /* 
                construct mqtt message from sample item received from the queue
                sample: ["ca-nb-aws-01-1000.Air-Temperature",1729957661187888000,1002.928162,"Air-Temperature", "ca-nb-aws-01-1000"] 
            */
            char* msg = malloc(MQTT_PUB_MSG_BUFFER_SIZE);
            snprintf(msg, MQTT_PUB_MSG_BUFFER_SIZE, "[\"%s.%s\",%llu,%f,\"%s\",\"%s\"]", 
                    sample->device_id, sample_parameter_to_string(sample->parameter),
                    sample->timestamp, sample->value, 
                    sample_parameter_to_string(sample->parameter), sample->device_id);

            /* publish sample message */
            esp_mqtt_client_publish(mqtt_client_hdl, MQTT_PUB_ENV, (const char *)msg, 0, 0, 0);

            /* free rousource */
            free(msg);

            /* monitor consumed bytes for possible memory leak */
            //free_heap_size_last = print_free_heap_size(free_heap_size_last);
        }
    }
    vTaskDelete( NULL );
}

/**
 * @brief Task that prints memory usage.
 * 
 * @param pvParameters 
 */
static void heap_size_task( void *pvParameters ) {
    uint32_t free_heap_size_last = 0;
    /* time-into-interval handle and configuration - */
    time_into_interval_handle_t       tii_1min_hdl;
    const time_into_interval_config_t tii_1min_cfg = {
        .name               = "tii_1min",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 60,
        .interval_offset    = 10
    };

    /* attempt to initialize a time-into-interval handle - task system clock synchronization */
    time_into_interval_init(&tii_1min_cfg, &tii_1min_hdl);
    if (tii_1min_hdl == NULL) {
        ESP_LOGE(TAG, "Unable to initialize time-into-interval handle");
        esp_restart(); 
    }

    /* enter task loop */
    for ( ;; ) {
        /* time-into-interval task delay (1-min interval with 10-second offset into the interval) */
        time_into_interval_delay(tii_1min_hdl);

        /* monitor consumed bytes for possible memory leak */
        free_heap_size_last = print_free_heap_size(free_heap_size_last);

        ESP_LOGW(TAG, "Free Stack Memory: %lu bytes (heap_size_task)", uxTaskGetStackHighWaterMark2(NULL));

        if(s_sample_sensor_task_hdl != NULL) 
            ESP_LOGW(TAG, "Free Stack Memory: %lu bytes (sample_sensor_task)", uxTaskGetStackHighWaterMark2(s_sample_sensor_task_hdl));

        if(s_publish_sensor_task_hdl != NULL)  
            ESP_LOGW(TAG, "Free Stack Memory: %lu bytes (publish_sensor_task)", uxTaskGetStackHighWaterMark2(s_publish_sensor_task_hdl));
    }
    vTaskDelete( NULL );
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    /* print general startup information */
    ESP_LOGI(TAG, "Startup..");
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
    ESP_ERROR_CHECK( nvs_init() );

    /* attempt to start wifi services */
    ESP_ERROR_CHECK( wifi_start() );

    // attempt to set system date-time and timezone
    ESP_ERROR_CHECK( sntp_start("AST4ADT,M3.2.0,M11.1.0") ); // AST4ADT,M3.2.0,M11.1.0 Atlantic-Standard-Time (Alantic)
    print_sntp_time_servers(); print_system_time(); // print ntp time server(s) and system time

    /* system clock dependent */
    init_system_state();
    
    /* attempt to start mqtt app */
    ESP_ERROR_CHECK( mqtt_app_start() );

    /* attempt to start sensor sampling task */
    xTaskCreatePinnedToCore( 
        sample_sensor_task, 
        "smp_snr_tsk", 
        (MINIMAL_STACK_SIZE * 4), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        &s_sample_sensor_task_hdl,
        APP_CPU_NUM );
    
    /* attempt to start sensor mqtt publishing task */
    xTaskCreatePinnedToCore( 
        publish_sensor_task, 
        "pub_snr_tsk", 
        (MINIMAL_STACK_SIZE * 4), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        &s_publish_sensor_task_hdl, 
        APP_CPU_NUM );

    /* attempt to start memory usage task */
    xTaskCreatePinnedToCore( 
        heap_size_task, 
        "heap_size_tsk", 
        (MINIMAL_STACK_SIZE * 3), 
        NULL, 
        (tskIDLE_PRIORITY + 2), 
        NULL, 
        APP_CPU_NUM );
}

