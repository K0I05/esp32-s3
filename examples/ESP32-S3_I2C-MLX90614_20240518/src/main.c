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
#include <driver/i2c_master.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <mlx90614.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "ECG_MLX90614_TEST"

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
    // initialize mlx90614 i2c device configuration
    i2c_mlx90614_config_t       mlx90614_dev_cfg = I2C_MLX90614_CONFIG_DEFAULT;
    i2c_mlx90614_handle_t       mlx90614_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // mlx90614 init device
    i2c_mlx90614_init(i2c0_bus_hdl, &mlx90614_dev_cfg, &mlx90614_dev_hdl);
    if (mlx90614_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] i2c0 i2c_bus_device_create mlx90614 handle init failed");
    //
    //
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## MLX90614 - START #########################");
        //
        // handle mlx90614 sensor
        //
        float ambient_temperature;
        float obj1_temperature; float obj2_temperature;
        //
        //
        ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device configuration settings:");
        ESP_LOGI(CONFIG_APP_TAG, "mlx90614 identification number (init),         Hi: %lu, Lo: %lu", mlx90614_dev_hdl->dev_params->ident_number_hi, mlx90614_dev_hdl->dev_params->ident_number_lo);
        ESP_LOGI(CONFIG_APP_TAG, "mlx90614 object temperature range (init), maximum: %.2f C, minimum: %.2f C", mlx90614_dev_hdl->dev_params->obj_max_temperature, mlx90614_dev_hdl->dev_params->obj_min_temperature);
        ESP_LOGI(CONFIG_APP_TAG, "mlx90614 emissivity (0.1 to 1.0 - init):           %.2f", mlx90614_dev_hdl->dev_params->emissivity);
        //
        ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device measurements:");
        if(i2c_mlx90614_get_ambient_temperature(mlx90614_dev_hdl, &ambient_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] mlx90614 device read ambient temperature failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "mlx90614 ambient temperature:   %.2f C", ambient_temperature);
        }
        //
        if(i2c_mlx90614_get_object1_temperature(mlx90614_dev_hdl, &obj1_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] mlx90614 device read object(1) temperature failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "mlx90614 object(1) temperature: %.2f C", obj1_temperature);
        }
        //
        if(i2c_mlx90614_get_object2_temperature(mlx90614_dev_hdl, &obj2_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "[APP] mlx90614 device read object(2) temperature failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "mlx90614 object(2) temperature: %.2f C", obj2_temperature);
        }
        //
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## MLX90614 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    i2c_mlx90614_rm( mlx90614_dev_hdl ); //remove mlx90614 device from master i2c bus
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