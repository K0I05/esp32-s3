/**
 * @file main.c
 * @brief explain
 *
 * This example takes the parameters 
 *
 * board: (1) ESP32-S3-­WROOM­-1-N16R8 | (2) ESP32-WROOM-32UE (ESP32 DevKit V4)
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
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <bmp280.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "BMP280 [APP]"

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
    TickType_t                  xLastWakeTime;
    //
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime               = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize bmp280 i2c device configuration
    i2c_bmp280_config_t         bmp280_dev_cfg = I2C_BMP280_CONFIG_DEFAULT;
    i2c_bmp280_handle_t         bmp280_dev_hdl;
    //
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // bmp280 init device
    i2c_bmp280_init(i2c0_bus_hdl, &bmp280_dev_cfg, &bmp280_dev_hdl);
    if (bmp280_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c_bus_device_create bmp280 handle init failed");
    //
    //
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## BMP280 - START #########################");
        //
        // handle bmp280 sensor
        float temperature, pressure;
        esp_err_t result = i2c_bmp280_get_measurements(bmp280_dev_hdl, &temperature, &pressure);
        if(result != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "bmp280 device read failed (%s)", esp_err_to_name(result));
        } else {
            pressure = pressure / 100;
            ESP_LOGI(CONFIG_APP_TAG, "air temperature:     %.2f C", temperature);
            ESP_LOGI(CONFIG_APP_TAG, "barometric pressure: %.2f hPa", pressure);
        }
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## BMP280 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    i2c_bmp280_rm( bmp280_dev_hdl );     //remove bmp280 device from master i2c bus
    i2c_del_master_bus( i2c0_bus_hdl );  //delete master i2c bus
    vTaskDelete( NULL );
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