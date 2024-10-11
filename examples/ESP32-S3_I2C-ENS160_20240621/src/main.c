/**
 * @file main.c
 * @brief explain
 * 
 * reference examples
 *      https://github.com/sciosense/ens16x-arduino/tree/main/src
 *      https://github.com/sciosense/ENS160_driver/blob/master/src/ScioSense_ENS160.cpp
 *      https://github.com/adafruit/ENS160_driver/blob/master/src/ScioSense_ENS160.h
 *
 * 
 *
 * Sensor Board ENS160 + AHT21 Wiring (Pay Attention 2024-06-26)
 *  VIN -> VDD 3.3V
 *  GNG -> GND
 *  SCL -> MCU SCL Pin
 *  SDA -> MCU SDA Ping
 *  
 *  ENS160 I2C Address: 0x53
 *  AHT21  I2C Address: 0x38
 *  
 *  NOTE: Attempting to set CS or ADD to GND ro VDD causes a short. ENS160 
 *        I2C address does not appear to be changeable.
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

#include <ens160.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "ENS160 [APP]"

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
    // initialize ens160 i2c device configuration
    i2c_ens160_config_t         ens160_dev_cfg = I2C_ENS160_CONFIG_DEFAULT;
    i2c_ens160_handle_t         ens160_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // ens160 init device
    i2c_ens160_init(i2c0_bus_hdl, &ens160_dev_cfg, &ens160_dev_hdl);
    if (ens160_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create ens160 handle init failed");
    //
    ESP_LOGW(CONFIG_APP_TAG, "Part ID:             0x%04x", ens160_dev_hdl->part_id);

    ESP_LOGW(CONFIG_APP_TAG, "IRQ Config Register: 0x%04x (0b%s)", ens160_dev_hdl->irq_config_reg.reg, uint16_to_binary(ens160_dev_hdl->irq_config_reg.reg));
    //
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## ENS160 START #########################");
        //
        //i2c_detect(i2c0_bus_hdl);
        //
        //bool data_ready;
        //i2c_ens160_get_data_status(ens160_dev_hdl, &data_ready);
        //ESP_LOGW(CONFIG_APP_TAG, "data   %s", data_ready ? "true" : "false");
        
        i2c_ens160_air_quality_data_t aq_data;
        if(i2c_ens160_get_measurement(ens160_dev_hdl, &aq_data) != 0) {
            ESP_LOGE(CONFIG_APP_TAG, "i2c_ens160_get_measurement failed");
        } else {
            i2c_ens160_aqi_uba_row_t uba_aqi = i2c_ens160_aqi_index_to_definition(aq_data.uba_aqi);

            ESP_LOGW(CONFIG_APP_TAG, "index    %1x (%s)", aq_data.uba_aqi, uba_aqi.rating);
            ESP_LOGW(CONFIG_APP_TAG, "tvco     %d (0x%04x)", aq_data.tvoc, aq_data.tvoc);
            ESP_LOGW(CONFIG_APP_TAG, "eco2     %d (0x%04x)", aq_data.eco2, aq_data.eco2);
        }

        i2c_ens160_air_quality_raw_data_t aq_raw_data;
        if(i2c_ens160_get_raw_measurement(ens160_dev_hdl, &aq_raw_data) != 0) {
            ESP_LOGE(CONFIG_APP_TAG, "i2c_ens160_get_raw_measurement failed");
        } else {
            ESP_LOGW(CONFIG_APP_TAG, "ri-res 0 %lu", aq_raw_data.hp0_rs);
            ESP_LOGW(CONFIG_APP_TAG, "ri-res 1 %lu", aq_raw_data.hp1_rs);
            ESP_LOGW(CONFIG_APP_TAG, "ri-res 2 %lu", aq_raw_data.hp2_rs);
            ESP_LOGW(CONFIG_APP_TAG, "ri-res 3 %lu", aq_raw_data.hp3_rs);

            ESP_LOGW(CONFIG_APP_TAG, "bl-res 0 %lu", aq_raw_data.hp0_bl);
            ESP_LOGW(CONFIG_APP_TAG, "bl-res 1 %lu", aq_raw_data.hp1_bl);
            ESP_LOGW(CONFIG_APP_TAG, "bl-res 2 %lu", aq_raw_data.hp2_bl);
            ESP_LOGW(CONFIG_APP_TAG, "bl-res 3 %lu", aq_raw_data.hp3_bl);
        }
        
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## ENS160 END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 30 );
    }
    //
    // free up task resources and remove task from stack
    i2c_ens160_rm( ens160_dev_hdl );      //remove ens160 device from master i2c bus
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