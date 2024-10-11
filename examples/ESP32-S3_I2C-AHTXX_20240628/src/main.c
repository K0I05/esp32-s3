/**
 * @file main.c
 * @brief explain
 *
 * This example takes the parameters 
 *
 *  Sensor Board ENS160 + AHT21 Wiring (Pay Attention 2024-06-26)
 *  VIN -> VDD 3.3V
 *  GNG -> GND
 *  SCL -> MCU SCL Pin
 *  SDA -> MCU SDA Ping
 *  
 *  ENS160 I2C Address: 0x53
 *  AHT21  I2C Address: 0x38
 *  
 *  NOTE: Attempting to set CS or ADD to GND ro VDD causes a short. ENS160 
 *        I2C address does appear to be changeable.
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

#include <ahtxx.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "AHTXX [APP]"

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
    float                       ta_max = NAN, ta_min = NAN;
    char                        deg_char = 176;
    //
    //
    // initialize the xLastWakeTime variable with the current time
    xLastWakeTime               = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize ahtxx i2c device configuration
    i2c_ahtxx_config_t          ahtxx_dev_cfg = I2C_AHT2X_CONFIG_DEFAULT;
    i2c_ahtxx_handle_t          ahtxx_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // ahtxx init device
    i2c_ahtxx_init(i2c0_bus_hdl, &ahtxx_dev_cfg, &ahtxx_dev_hdl);
    if (ahtxx_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create ahtxx handle init failed");
    //
    ESP_LOGI(CONFIG_APP_TAG, "Status Register: 0x%02x (0b%s)", ahtxx_dev_hdl->status_reg.reg, uint8_to_binary(ahtxx_dev_hdl->status_reg.reg));
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## AHTXX - START #########################");
        //
        //
        //i2c_master_bus_detect_devices(i2c0_bus_hdl);
        //
        // handle aht2x sensor
        //
        
        float ta; float td; float rh;
        if(i2c_ahtxx_get_measurements(ahtxx_dev_hdl, &ta, &rh, &td) != 0) {
            ESP_LOGI(CONFIG_APP_TAG, "i2c_ahtxx_get_measurements failed");
        } else {
            // process min and max ta since reboot
            if(isnan(ta_max) && isnan(ta_min)) {
                // initialize
                ta_max = ta;
                ta_min = ta;
            }
            
            // set min and max values
            if(ta > ta_max) ta_max = ta;
            if(ta < ta_min) ta_min = ta;

            ESP_LOGI(CONFIG_APP_TAG, "air:         %.2f%cC", ta, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "air-min:     %.2f%cC", ta_min, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "air-max:     %.2f%cC", ta_max, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "dew-point:   %.2f%cC", td, deg_char);
            ESP_LOGI(CONFIG_APP_TAG, "humidity:    %.2f %%", rh);
        }
        
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## AHTXX - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    i2c_ahtxx_rm( ahtxx_dev_hdl );      //remove aht2x device from master i2c bus
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