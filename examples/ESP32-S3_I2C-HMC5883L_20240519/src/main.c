/**
 * @file main.c
 * @brief explain
 *
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
#include <esp_vfs_fat.h>

#include <driver/sdmmc_host.h>

#include <nvs.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <hmc5883l.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "ECG_HMC5883L_TEST"


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
    xLastWakeTime                       = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t             i2c0_master_cfg = {
        .clk_source                     = I2C_CLK_SRC_DEFAULT,
        .i2c_port                       = CONFIG_I2C_0_PORT,
        .scl_io_num                     = CONFIG_I2C_0_SCL_IO,
        .sda_io_num                     = CONFIG_I2C_0_SDA_IO,
        .glitch_ignore_cnt              = 7,
        .flags.enable_internal_pullup   = true,
    }; // i2c_bus configurations
    i2c_master_bus_handle_t             i2c0_bus_hdl;
    //
    // initialize hmc5883l i2c device configuration
    i2c_hmc5883l_config_t               hmc5883l_dev_cfg = {
        .dev_config.device_address      = I2C_HMC5883L_ADDR,
        .mode                           = I2C_HMC5883L_MODE_CONTINUOUS,
        .sample                         = I2C_HMC5883L_SAMPLE_4,
        .rate                           = I2C_HMC5883L_DATA_RATE_00_75,
        .gain                           = I2C_HMC5883L_GAIN_1090,
        .bias                           = I2C_HMC5883L_BIAS_NORMAL,
    };
    i2c_hmc5883l_handle_t               hmc5883l_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // hmc5883l init device
    i2c_hmc5883l_init(i2c0_bus_hdl, &hmc5883l_dev_cfg, &hmc5883l_dev_hdl);
    if (hmc5883l_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_device_create hmc5883l handle init failed");
    //
    //hmc5883l_dev_hdl->mode = I2C_HMC5883L_MODE_SINGLE;
    //if(i2c_hmc5883l_write_mode(hmc5883l_dev_hdl) != ESP_OK) {
    //if(i2c_hmc5883l_write_mode(hmc5883l_dev_hdl, hmc5883l_dev_hdl->mode) != ESP_OK) {
    //    ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_hmc5883l_write_mode (%02x) hmc5883l failed", hmc5883l_dev_hdl->mode);
    //}
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## HMC5883L - START #########################");
        //
        // handle hmc5883l sensor
        //
        //
        /*
        i2c_hmc5883l_raw_data_t raw_data;
        if(i2c_hmc5883l_read_raw_data(hmc5883l_dev_hdl, &raw_data) != 0) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] hmc5883l device read raw data (X-Y-Z axis) failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "hmc5883l X(%d) | Y(%d) | Z(%d)", raw_data.x, raw_data.y, raw_data.z);
        }
        */
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## HMC5883L - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    //i2c_hmc5883l_rm( hmc5883l_dev_hdl ); //remove hmc5883l device from master i2c bus
    i2c_del_master_bus( i2c0_bus_hdl ); //delete master i2c bus
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
    
    xTaskCreatePinnedToCore( 
        i2c_0_task, 
        CONFIG_I2C_0_TASK_NAME, 
        CONFIG_I2C_0_TASK_STACK_SIZE, 
        NULL, 
        CONFIG_I2C_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
}