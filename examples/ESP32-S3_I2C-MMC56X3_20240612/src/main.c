/**
 * @file main.c
 * @brief explain
 *
 * examples
 *      https://github.com/adafruit/Adafruit_MMC56x3/blob/main/Adafruit_MMC56x3.cpp
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
#include <driver/i2c_master.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <mmc56x3.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "MMC56X3 [APP]"

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
    // initialize mmc56x3 i2c device configuration
    i2c_mmc56x3_config_t          mmc56x3_dev_cfg = I2C_MMC56X3_CONFIG_DEFAULT;
    i2c_mmc56x3_handle_t          mmc56x3_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // mmc56x3 init device
    i2c_mmc56x3_init(i2c0_bus_hdl, &mmc56x3_dev_cfg, &mmc56x3_dev_hdl);
    if (mmc56x3_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create mmc56x3 handle init failed");
    //
    //
    // task loop entry point
    for ( ;; ) {
        //
        //i2c_master_bus_detect_devices(i2c0_bus_hdl);
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## MMC56X3 - START #########################");
        //
        // handle mmc56x3 sensor
        esp_err_t mmc56x3_result;
        //
        i2c_mmc56x3_magnetic_axes_data_t magnetic_axes_data; float magnetic_heading;
        mmc56x3_result = i2c_mmc56x3_get_magnetic_axes(mmc56x3_dev_hdl, &magnetic_axes_data);
        //
        if(mmc56x3_result == ESP_OK) {
            magnetic_heading = i2c_mmc56x3_convert_to_heading(magnetic_axes_data);
            ESP_LOGI(CONFIG_APP_TAG, "X-Axis:       %.2f", magnetic_axes_data.x_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Y-Axis:       %.2f", magnetic_axes_data.y_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Z-Axis:       %.2f", magnetic_axes_data.z_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Heading:      %.2f", magnetic_heading);
        } else {
            ESP_LOGE(CONFIG_APP_TAG, "Get Magnetic Failed: %s", esp_err_to_name(mmc56x3_result));
        }
        //
        /*
        float temperature;
        mmc56x3_result = i2c_mmc56x3_get_temperature(mmc56x3_dev_hdl, &temperature);
        //
        if(mmc56x3_result == ESP_OK) {
            ESP_LOGI(CONFIG_APP_TAG, "Temperature: %.2f C", temperature);
        } else {
            ESP_LOGE(CONFIG_APP_TAG, "Get Temperature Failed: %s", esp_err_to_name(mmc56x3_result));
        }
        */
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## MMC56X3 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 1 );
    }
    //
    // free up task resources and remove task from stack
    i2c_mmc56x3_rm( mmc56x3_dev_hdl );      //remove mmc56x3 device from master i2c bus
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