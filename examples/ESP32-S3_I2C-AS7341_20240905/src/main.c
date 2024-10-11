/**
 * @file main.c
 * @brief explain
 *
 * This example takes the parameters 
 *
 *  https://github.com/adafruit/Adafruit_AS7341/blob/master/Adafruit_AS7341.cpp
 *  https://github.com/sparkfun/SparkFun_AS7341X_Arduino_Library/blob/main/src/SparkFun_AS7341X_Arduino_Library.cpp
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

#include <as7341.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "AS7341 [APP]"

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
    // initialize the xLastWakeTime variable with the current time
    xLastWakeTime               = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize as7341 i2c device configuration
    i2c_as7341_config_t          as7341_dev_cfg = I2C_AS7341_CONFIG_DEFAULT;
    i2c_as7341_handle_t          as7341_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // as7341 init device
    i2c_as7341_init(i2c0_bus_hdl, &as7341_dev_cfg, &as7341_dev_hdl);
    if (as7341_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create as7341 handle init failed");
    //
    i2c_as7341_set_atime(as7341_dev_hdl, 29);
    i2c_as7341_set_astep(as7341_dev_hdl, 599);
    i2c_as7341_set_spectral_gain(as7341_dev_hdl, I2C_AS7341_SPECTRAL_GAIN_256X);
    //
    ESP_LOGI(CONFIG_APP_TAG, "Enable Register: 0x%02x (0b%s)", as7341_dev_hdl->enable_reg.reg, uint8_to_binary(as7341_dev_hdl->enable_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Aux ID Register: 0x%02x (0b%s)", as7341_dev_hdl->aux_id_reg.reg, uint8_to_binary(as7341_dev_hdl->aux_id_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Rev ID Register: 0x%02x (0b%s)", as7341_dev_hdl->revision_id_reg.reg, uint8_to_binary(as7341_dev_hdl->revision_id_reg.reg));
    ESP_LOGI(CONFIG_APP_TAG, "Part ID Register: 0x%02x (0b%s)", as7341_dev_hdl->part_id_reg.reg, uint8_to_binary(as7341_dev_hdl->part_id_reg.reg));

    ESP_LOGW(CONFIG_APP_TAG, "Part ID  0x%02x", as7341_dev_hdl->part_id_reg.bits.identifier);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## AS7341 - START #########################");
        //
        //
        //i2c_detect(i2c0_bus_hdl);
        //
        // handle as7341 sensor
        //
        //i2c_as7341_enable_led(as7341_dev_hdl);
        //
        //vTaskDelay(pdMS_TO_TICKS(500));
        //
        //
        i2c_as7341_channels_adc_data_t adc_data;
        //i2c_as7341_channels_data_t data;

        esp_err_t result = i2c_as7341_get_adc_measurements(as7341_dev_hdl, &adc_data);
  
        if(result != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "measurement failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGW(CONFIG_APP_TAG, "F1    %d", adc_data.f1_415nm);
            ESP_LOGW(CONFIG_APP_TAG, "F2    %d", adc_data.f2_445nm);
            ESP_LOGW(CONFIG_APP_TAG, "F3    %d", adc_data.f3_480nm);
            ESP_LOGW(CONFIG_APP_TAG, "F4    %d", adc_data.f4_515nm);
            ESP_LOGW(CONFIG_APP_TAG, "F5    %d", adc_data.f5_555nm);
            ESP_LOGW(CONFIG_APP_TAG, "F6    %d", adc_data.f6_590nm);
            ESP_LOGW(CONFIG_APP_TAG, "F7    %d", adc_data.f7_630nm);
            ESP_LOGW(CONFIG_APP_TAG, "F8    %d", adc_data.f8_680nm);
            ESP_LOGW(CONFIG_APP_TAG, "NIR   %d", adc_data.nir);
            ESP_LOGW(CONFIG_APP_TAG, "CLEAR %d", adc_data.clear);

            /*
            i2c_as7341_get_basic_counts(as7341_dev_hdl, adc_data, &data);
            
            ESP_LOGW(CONFIG_APP_TAG, "F1    %f", data.f1_415nm);
            ESP_LOGW(CONFIG_APP_TAG, "F2    %f", data.f2_445nm);
            ESP_LOGW(CONFIG_APP_TAG, "F3    %f", data.f3_480nm);
            ESP_LOGW(CONFIG_APP_TAG, "F4    %f", data.f4_515nm);
            ESP_LOGW(CONFIG_APP_TAG, "F5    %f", data.f5_555nm);
            ESP_LOGW(CONFIG_APP_TAG, "F6    %f", data.f6_590nm);
            ESP_LOGW(CONFIG_APP_TAG, "F7    %f", data.f7_630nm);
            ESP_LOGW(CONFIG_APP_TAG, "F8    %f", data.f8_680nm);
            ESP_LOGW(CONFIG_APP_TAG, "NIR   %f", data.nir);
            ESP_LOGW(CONFIG_APP_TAG, "CLEAR %f", data.clear);
            */
        }
        //
        //i2c_as7341_disable_led(as7341_dev_hdl);
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## AS7341 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    i2c_as7341_rm( as7341_dev_hdl );      //remove as7341 device from master i2c bus
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