/**
 * @file main.c
 * @brief explain
 *
 * This example takes the parameters 
 *
 * board: (1) ESP32-S3-­WROOM­-1-N16R8 (esp32s3box)
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
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <sht4x.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "SHT4X [APP]"

// macros
#define CONFIG_I2C_0_MASTER_DEFAULT {                               \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = CONFIG_I2C_0_PORT,        \
        .scl_io_num                     = CONFIG_I2C_0_SCL_IO,      \
        .sda_io_num                     = CONFIG_I2C_0_SDA_IO,      \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

static inline void vTaskDelayMs(const uint ms) {
    const TickType_t xDelay = (ms / portTICK_PERIOD_MS);
    vTaskDelay( xDelay );
}

static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

static void i2c_0_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime  = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize sht4x i2c device configuration
    i2c_sht4x_config_t          sht4x_dev_cfg = I2C_SHT4X_CONFIG_DEFAULT;
    i2c_sht4x_handle_t          sht4x_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // sht4x init device
    i2c_sht4x_init(i2c0_bus_hdl, &sht4x_dev_cfg, &sht4x_dev_hdl);
    if (sht4x_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create sht4x handle init failed");
    //
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## SHT4X - START #########################");
        //
        //i2c_master_bus_detect_devices(i2c0_bus_hdl);
        //
        // handle sht4x sensor
        //
        
        float air_temperature, dewpoint_temperature, relative_humidity;
        //
        if(i2c_sht4x_get_measurements(sht4x_dev_hdl, &air_temperature, &relative_humidity, &dewpoint_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "sht4x device air temperature and relative humidity measurement failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "sht4x air temperature:       %.2f C", air_temperature);
            ESP_LOGI(CONFIG_APP_TAG, "sht4x relative humidity:     %.2f %%", relative_humidity);
            ESP_LOGI(CONFIG_APP_TAG, "sht4x dewpoint temperature:  %.2f C", dewpoint_temperature);
        }
        
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## SHT4X - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free resources
    i2c_sht4x_del( sht4x_dev_hdl );     // delete sht4x device from master i2c bus and free handle
    i2c_del_master_bus( i2c0_bus_hdl ); // delete master i2c bus and free handle
    vTaskDelete( NULL );                // delete task
}


void app_main( void ) {
    ESP_LOGI(CONFIG_APP_TAG, "Startup..");
    ESP_LOGI(CONFIG_APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(CONFIG_APP_TAG, "IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(CONFIG_APP_TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    xTaskCreatePinnedToCore( 
        i2c_0_task, 
        CONFIG_I2C_0_TASK_NAME, 
        CONFIG_I2C_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_I2C_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
}