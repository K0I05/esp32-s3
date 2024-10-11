/**
 * @file main.c
 * @brief explain
 *
 * Reference examples
 *      https://github.com/kgrozdanovski/veml7700-esp-idf/blob/main/src/veml7700.c
 *      https://github.com/adafruit/Adafruit_VEML7700/blob/master/Adafruit_VEML7700.cpp
 * 
 *
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

#include <veml7700.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "VEML7700 [APP]"

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
    TickType_t                          xLastWakeTime;
    //
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime               = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize veml7700 i2c device configuration  (I2C_VEML7700_CONFIG_DEFAULT)
    i2c_veml7700_config_t       veml7700_dev_cfg = I2C_VEML7700_CONFIG_DEFAULT;
    i2c_veml7700_handle_t       veml7700_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // veml7700 init device
    i2c_veml7700_init(i2c0_bus_hdl, &veml7700_dev_cfg, &veml7700_dev_hdl);
    if (veml7700_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create veml7700 handle init failed");
    //
    // veml7700 registers
    ESP_LOGI(CONFIG_APP_TAG, "Configuration Register in Binary:     %s", uint16_to_binary(veml7700_dev_hdl->config_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Power Saving Mode Register in Binary: %s", uint16_to_binary(veml7700_dev_hdl->power_saving_mode_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Interrupt Status Register in Binary:  %s", uint16_to_binary(veml7700_dev_hdl->interrupt_status_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Identifier Register in Binary:        %s", uint16_to_binary(veml7700_dev_hdl->identifier_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Hi Threshold: %d", veml7700_dev_hdl->hi_threshold_reg);
    ESP_LOGI(CONFIG_APP_TAG, "Lo Threshold: %d", veml7700_dev_hdl->lo_threshold_reg);
    ESP_LOGI(CONFIG_APP_TAG, "Resolution:   %0.4f", veml7700_dev_hdl->resolution);
    ESP_LOGI(CONFIG_APP_TAG, "Maximum Lux:  %lu", veml7700_dev_hdl->maximum_lux);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## VEML7700 - START #########################");
        //
        // handle veml7700 sensor
        //
        float illumination;
        //
        if(i2c_veml7700_get_ambient_light(veml7700_dev_hdl, &illumination) != 0) {
            ESP_LOGI(CONFIG_APP_TAG, "i2c_veml7700_get_ambient_light failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "Ambient Light:  %0.2f", illumination);
        }
        //
        /*
        if(i2c_veml7700_get_ambient_light_auto(veml7700_dev_hdl, &illumination) != 0) {
            ESP_LOGI(CONFIG_APP_TAG, "i2c_veml7700_get_ambient_light_auto failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "Ambient Light (Auto):  %0.2f", illumination);
        }
        */
        //
        if(i2c_veml7700_get_white_channel(veml7700_dev_hdl, &illumination) != 0) {
            ESP_LOGI(CONFIG_APP_TAG, "i2c_veml7700_get_white_channel failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "White Channel:  %0.2f", illumination);
        }
        //
        /*
        if(i2c_veml7700_get_white_channel_auto(veml7700_dev_hdl, &illumination) != 0) {
            ESP_LOGI(CONFIG_APP_TAG, "i2c_veml7700_get_white_channel_auto failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "White Channel (Auto):  %0.2f", illumination);
        }
        */
        //
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## VEML7700 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    i2c_veml7700_rm( veml7700_dev_hdl );  //remove veml7700 device from master i2c bus
    i2c_del_master_bus( i2c0_bus_hdl ); //delete master i2c bus
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