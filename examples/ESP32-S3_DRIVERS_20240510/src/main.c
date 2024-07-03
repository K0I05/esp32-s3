/**
 * @file main.c
 * @brief explain
 *
 * This example performs various operations against device drivers developed for the ESP32-S3 chipset. 
 * Runs adc, i2c tasks and a status task with monitoring task.  Basic global data device model sharing
 * using a mutex.
 *
 * board: (1) ESP32-S3-­WROOM­-1-N16R8
 * 
 * CTRL + SHIFT + P
 * pio run -t menufconfig
 * k & l keys for up or down
 * OR
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
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
#include <esp_vfs_fat.h>

#include <driver/i2c_master.h>
#include <driver/sdmmc_host.h>
#include <driver/temperature_sensor.h>

#include <nvs.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <sht4x.h>
#include <bh1750.h>
#include <bmp280.h>
#include <mlx90614.h>
#include <s12sd.h>

#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow

#define CONFIG_I2C_0_BH1750_ENABLED     (1)
#define CONFIG_I2C_0_SHT4X_ENABLED      (1)

#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 3)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_ADC_0_TASK_NAME          "adc_0_tsk"
#define CONFIG_ADC_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 3)
#define CONFIG_ADC_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_STATUS_TASK_NAME        "status_tsk"
#define CONFIG_STATUS_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE * 5)
#define CONFIG_STATUS_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

#define CONFIG_MONITOR_TASK_NAME        "monitor_tsk"
#define CONFIG_MONITOR_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE * 5)
#define CONFIG_MONITOR_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

#define CONFIG_SEM_WAIT_TIME            (50)  // ticks
#define CONFIG_APP_TAG                  "ECG_DRIVER_TEST"

/*
 * macro definitions
*/
#define ARRAY_SIZE(arr)	(sizeof(arr) / sizeof((arr)[0]))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CONFIG_I2C_0_MASTER_DEFAULT {                               \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = CONFIG_I2C_0_PORT,        \
        .scl_io_num                     = CONFIG_I2C_0_SCL_IO,      \
        .sda_io_num                     = CONFIG_I2C_0_SDA_IO,      \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }
#define ENSURE_TRUE(ACTION)           \
    do                                \
    {                                 \
        BaseType_t __res = (ACTION);  \
        assert(__res == pdTRUE);      \
        (void)__res;                  \
    } while (0)


/*
 * Device Data Model
*/
typedef struct {
    struct {
        float       temperature;
    } system;
    struct {
        float       latitude;
        float       longitude;
        int16_t     elevation;
        uint16_t    heading;
    } position;
    struct {
        float       illuminance;
        uint8_t     uv_index;
        struct {
            float   ambient_temperature;
            float   object1_temperature;
            float   object2_temperature;
        } temperature;
    } sky;
    struct {
        float       air_temperature;
        float       relative_humdity;
        float       dewpoint_temperature;
        float       wetbulb_temperature;
        float       barometric_pressure;
        struct {
            float compensation_temperature;
        } pressure;
    } environment;
} device_data_model_t;

/*
* main prototypes (excludes inline - see inlines)
*/
static void i2c_0_task( void *pvParameters );
static void adc_0_task( void *pvParameters );
static void monitor_task( void *pvParameters );
static void status_task( void *pvParameters );

/*
 * local declerations
*/
static TaskHandle_t                 l_main_task_hdl;
static TaskHandle_t                 l_status_task_hdl;
static TaskHandle_t                 l_monitor_task_hdl;
static TaskHandle_t                 l_i2c_0_task_hdl;
static TaskHandle_t                 l_adc_0_task_hdl;
static SemaphoreHandle_t            l_dev_data_model_mutex;
static volatile device_data_model_t l_dev_data_model; // mutex on write

/**
 * @brief Pauses the task in milliseconds.
 *
 * @param[in] ms number of milliseconds the task will pause for.
 */
static inline void vTaskDelayMs(const uint ms) {
    const TickType_t xDelay = (ms / portTICK_PERIOD_MS);
    vTaskDelay( xDelay );
}

/**
 * @brief Pauses the task in milliseconds from previous wake time.
 *
 * @param[in] previousWakeTime previous task wake time.
 * @param[in] ms number of milliseconds the task will pause for.
 */
static inline void vTaskDelayMsUntil(TickType_t *previousWakeTime, const uint ms) {
    const TickType_t xFrequency = (ms / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

/**
 * @brief Pauses the task in seconds from previous wake time.
 *
 * @param[in] previousWakeTime previous task wake time.
 * @param[in] sec number of seconds the task will pause for.
 */
static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

static inline void print_data_model(const device_data_model_t data) {
    /* prints the device data model
     Position
     - Latitude in decimal degrees
     - Longitude in decimal degrees
     - Elevation in centimeters
     - Heading in degrees (magnetic)

     Sky
     - Illuminance in lux
     - Ultraviolet light as an index from 1 to 11

     Environment
     - Air Temperature in degree celcius
     - Relative Humidity in percentage
     - Barometric Pressure in hectopascal
    */
   ESP_LOGI(CONFIG_APP_TAG, "[APP] ################################################");
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 board temperature:               %.2f C", data.system.temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 bh1750 ambient light:            %.2f Lux", data.sky.illuminance);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] adc0 s12sd uv index:                  %u", data.sky.uv_index);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 sht4x air temperature:           %.2f C", data.environment.air_temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 sht4x dewpoint temperature:      %.2f C", data.environment.dewpoint_temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 sht4x relative humidity:         %.2f %%", data.environment.relative_humdity);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 bmp280 barometric pressure:      %.2f hPa", data.environment.barometric_pressure / 100);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 bmp280 compensation temperature: %.2f C", data.environment.pressure.compensation_temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 mlx90614 ambient temperature:    %.2f C", data.sky.temperature.ambient_temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 mlx90614 object1 temp:           %.2f C", data.sky.temperature.object1_temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] i2c0 mlx90614 object2 temp:           %.2f C", data.sky.temperature.object2_temperature);
   ESP_LOGI(CONFIG_APP_TAG, "[APP] ################################################");
}

static inline esp_err_t wx_td(const float ta, const float rh, float *td) {
    ESP_ARG_CHECK(ta && rh);

    // validate parameters
    if(ta > 80 || ta < -40) return ESP_ERR_INVALID_ARG;
    if(rh > 100 || rh < 0) return ESP_ERR_INVALID_ARG;
    
    // calculate dew-point temperature
    double H = (log10(rh)-2)/0.4343 + (17.62*ta)/(243.12+ta);
    *td = 243.12*H/(17.62-H);
    
    return ESP_OK;
}

static void i2c_0_task( void *pvParameters ) {
    TickType_t                      xLastWakeTime;
    //
    /*
    * internal temperature sensor (ESP32-S3)
    */
    temperature_sensor_handle_t     ti_sensor_hdl;
    temperature_sensor_config_t     ti_sensor_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT (20, 50);
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime                   = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t         i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t         i2c0_bus_hdl;
    //
    // initialize sht4x i2c device configuration
    i2c_sht4x_config_t              sht4x_dev_cfg = I2C_SHT4X_CONFIG_DEFAULT;
    i2c_sht4x_handle_t              sht4x_dev_hdl;
    //
    // initialize bh1750 i2c device configuration
    i2c_bh1750_config_t             bh1750_dev_cfg = I2C_BH1750_CONFIG_DEFAULT;
    i2c_bh1750_handle_t             bh1750_dev_hdl;
    //
    // initialize bmp280 i2c device configuration
    i2c_bmp280_config_t             bmp280_dev_cfg = I2C_BMP280_CONFIG_DEFAULT;
    i2c_bmp280_handle_t             bmp280_dev_hdl;
    //
    // initialize mlx90614 i2c device configuration
    i2c_mlx90614_config_t           mlx90614_dev_cfg = I2C_MLX90614_CONFIG_DEFAULT;
    i2c_mlx90614_handle_t           mlx90614_dev_hdl;
    //
    //
    // install and enable temperature sensor
    ESP_ERROR_CHECK( temperature_sensor_install(&ti_sensor_cfg, &ti_sensor_hdl) );
    ESP_ERROR_CHECK( temperature_sensor_enable(ti_sensor_hdl) );
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // sht4x init device
    i2c_sht4x_init(i2c0_bus_hdl, &sht4x_dev_cfg, &sht4x_dev_hdl);
    if (sht4x_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_device_create sht4x handle init failed");
    //
    // bh1750 init device
    i2c_bh1750_init(i2c0_bus_hdl, &bh1750_dev_cfg, &bh1750_dev_hdl);
    if (bh1750_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_device_create bh1750 handle init failed");
    //
    // bmp280 init device
    i2c_bmp280_init(i2c0_bus_hdl, &bmp280_dev_cfg, &bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_device_create bmp280 handle init failed");
    //
    // mlx90614 init device
    i2c_mlx90614_init(i2c0_bus_hdl, &mlx90614_dev_cfg, &mlx90614_dev_hdl);
    if (mlx90614_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_device_create mlx90614 handle init failed");
    //
    // task loop entry point
    for ( ;; ) {
        //
        // handle sht4x sensor
        float ta, rh, td;
        if(i2c_sht4x_get_measurement(sht4x_dev_hdl, &ta, &rh) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] sht4x device read failed");
        }
        wx_td(ta, rh, &td); // compute dew-point temperature
        //
        // handle bh1750 sensor
        float lu;
        if(i2c_bh1750_get_measurement(bh1750_dev_hdl, &lu) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] bh1750 device read failed");
        }
        //
        // handle bmp280 sensor
        float cmp_t, bp;
        if(i2c_bmp280_get_measurement(bmp280_dev_hdl, &cmp_t, &bp) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] bmp280 device read failed");
        }
        //
        // handle mlx90614 sensor
        float amb_t, obj1_t, obj2_t;
        if(i2c_mlx90614_get_temperatures(mlx90614_dev_hdl, &amb_t, &obj1_t, &obj2_t) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] mlx90614 device read ambient temperature failed");
        }
        //
        // handle internal temperature sensor
        //
        // get converted sensor data
        float ts;
        if(temperature_sensor_get_celsius(ti_sensor_hdl, &ts) != 0) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] internal temperature read failed");
        }
        //
        // copy measurements to local device data model
        ENSURE_TRUE( xSemaphoreTake(l_dev_data_model_mutex, CONFIG_SEM_WAIT_TIME) );
        l_dev_data_model.environment.air_temperature = ta;
        l_dev_data_model.environment.relative_humdity = rh;
        l_dev_data_model.environment.dewpoint_temperature = td;
        l_dev_data_model.environment.barometric_pressure = bp;
        l_dev_data_model.environment.pressure.compensation_temperature = cmp_t;
        l_dev_data_model.sky.illuminance = lu;
        l_dev_data_model.sky.temperature.ambient_temperature = amb_t;
        l_dev_data_model.sky.temperature.object1_temperature = obj1_t;
        l_dev_data_model.sky.temperature.object2_temperature = obj2_t;
        l_dev_data_model.system.temperature = ts;
        ENSURE_TRUE( xSemaphoreGive(l_dev_data_model_mutex) );
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 9 );
    }
    //
    // free up task resources and remove task from stack
    i2c_sht4x_rm( sht4x_dev_hdl );       //remove sht4x device from master i2c bus
    i2c_bh1750_rm( bh1750_dev_hdl );     //remove bh1750 device from master i2c bus
    i2c_bmp280_rm( bmp280_dev_hdl );     //remove bmp280 device from master i2c bus
    i2c_mlx90614_rm( mlx90614_dev_hdl ); //remove mlx90614 device from master i2c bus
    i2c_del_master_bus( i2c0_bus_hdl );  //delete master i2c bus
    vTaskDelete( NULL );
}

static void adc_0_task( void *pvParameters ) {
    TickType_t                         xLastWakeTime;
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime                      = xTaskGetTickCount ();
    //
    // initialize s12sd device configuration
    adc_s12sd_config_t                s12sd_dev_cfg = ADC_UV_S12SD_CONFIG_DEFAULT;
    adc_s12sd_handle_t                s12sd_dev_hdl;
    //
    // s12sd init device
    adc_s12sd_init(&s12sd_dev_cfg, &s12sd_dev_hdl);
    if (s12sd_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] adc0 continuous s12sd handle init failed");
    //
    // task loop entry point
    for ( ;; ) {
        // read s12sd uv sensor
        uint8_t uv_index;
        if(adc_s12sd_measure(s12sd_dev_hdl, &uv_index) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] adc0 one-shot s12sd measurement failed");
        }
        //
        // copy measurement to local device data model
        ENSURE_TRUE( xSemaphoreTake(l_dev_data_model_mutex, CONFIG_SEM_WAIT_TIME) );
        l_dev_data_model.sky.uv_index = uv_index;
        ENSURE_TRUE( xSemaphoreGive(l_dev_data_model_mutex) );
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 9 );
    }
    //
    // free up task resources and remove task from stack
    //i2c_del_master_bus( i2c0_bus_hdl ); //delete master i2c bus
    vTaskDelete( NULL );
}

static void status_task( void *pvParameters ) {
    TickType_t                      xLastWakeTime;
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime                       = xTaskGetTickCount ();
    //
    // task loop entry point
    for ( ;; ) {
        // 
        print_data_model( l_dev_data_model );
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 15 );
    }
    //
    // free up task resources and remove task from stack
    //i2c_del_master_bus( i2c0_bus_hdl ); //delete master i2c bus
    vTaskDelete( NULL );
}

static void monitor_task( void *pvParameters ) {
    TickType_t                      xLastWakeTime;
    TaskStatus_t                    *pxTaskStatusArray;
    volatile UBaseType_t            uxArraySize, x;
    configRUN_TIME_COUNTER_TYPE     ulTotalRunTime, ulStatsAsPercentage;
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime                       = xTaskGetTickCount ();
    //
    // task loop entry point
    for ( ;; ) {
        char task_stats_buffer[50];

        //vTaskDelay(50000 / portTICK_PERIOD_MS); // 1000ms = 1sec
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 60 );

        ESP_LOGI(CONFIG_APP_TAG, "[APP] ************* TASK MONITOR *****************");
        ESP_LOGI(CONFIG_APP_TAG, "[APP]   Free Heap:  %u bytes", xPortGetFreeHeapSize());
        ESP_LOGI(CONFIG_APP_TAG, "[APP]   Min Heap:   %u bytes", xPortGetMinimumEverFreeHeapSize());

        // Take a snapshot of the number of tasks in case it changes while this
        // function is executing.
        uxArraySize = uxTaskGetNumberOfTasks();

        // Allocate a TaskStatus_t structure for each task.  An array could be
        // allocated statically at compile time.
        pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

        if( pxTaskStatusArray != NULL ) {
            ESP_LOGI(CONFIG_APP_TAG, "[APP] --------------------------------------------");
            ESP_LOGI(CONFIG_APP_TAG, "[APP]   TASK                ABS             Util");
            // Generate raw status information about each task.
            uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

            // For percentage calculations.
            ulTotalRunTime /= 100UL;

            // Avoid divide by zero errors.
            if( ulTotalRunTime > 0 ) {
                // For each populated position in the pxTaskStatusArray array,
                // format the raw data as human readable ASCII data
                for( x = 0; x < uxArraySize; x++ ) {
                    // What percentage of the total run time has the task used?
                    // This will always be rounded down to the nearest integer.
                    // ulTotalRunTimeDiv100 has already been divided by 100.
                    ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

                    if( ulStatsAsPercentage > 0UL ) {
                        sprintf( task_stats_buffer, "%s\t\t%llu\t\t%llu", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter, ulStatsAsPercentage );
                        ESP_LOGI(CONFIG_APP_TAG, "[APP]   %s", task_stats_buffer);
                    } else {
                        // If the percentage is zero here then the task has
                        // consumed less than 1% of the total run time.
                        sprintf( task_stats_buffer, "%s\t\t%llu\t\t<1", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter );
                        ESP_LOGI(CONFIG_APP_TAG, "[APP]   %s", task_stats_buffer);
                    }
                }
            }
            // The array is no longer needed, free the memory it consumes.
            vPortFree( pxTaskStatusArray );
            ESP_LOGI(CONFIG_APP_TAG, "[APP] --------------------------------------------");
        }
        ESP_LOGI(CONFIG_APP_TAG, "[APP] ************* TASK MONITOR *****************");
    }
    vTaskDelete( NULL );
}

void app_main( void ) {
    ESP_LOGI(CONFIG_APP_TAG, "[APP] Startup..");
    ESP_LOGI(CONFIG_APP_TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(CONFIG_APP_TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(CONFIG_APP_TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    l_main_task_hdl = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(l_main_task_hdl == NULL);

    l_dev_data_model_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(l_dev_data_model_mutex == NULL);
    
    xTaskCreatePinnedToCore( 
        i2c_0_task, 
        CONFIG_I2C_0_TASK_NAME, 
        CONFIG_I2C_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_I2C_0_TASK_PRIORITY, 
        &l_i2c_0_task_hdl, 
        APP_CPU_NUM );

    xTaskCreatePinnedToCore( 
        adc_0_task, 
        CONFIG_ADC_0_TASK_NAME, 
        CONFIG_ADC_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_ADC_0_TASK_PRIORITY, 
        &l_adc_0_task_hdl, 
        APP_CPU_NUM );

    xTaskCreatePinnedToCore( 
        status_task, 
        CONFIG_STATUS_TASK_NAME, 
        CONFIG_STATUS_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_STATUS_TASK_PRIORITY, 
        &l_status_task_hdl, 
        APP_CPU_NUM );

    xTaskCreatePinnedToCore( 
        monitor_task, 
        CONFIG_MONITOR_TASK_NAME, 
        CONFIG_MONITOR_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_MONITOR_TASK_PRIORITY, 
        &l_monitor_task_hdl, 
        APP_CPU_NUM );

}