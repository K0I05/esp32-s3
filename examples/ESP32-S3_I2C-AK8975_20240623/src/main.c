/**
 * @file main.c
 * @brief explain
 *
 * This example takes the parameters 
 *
 * board: (1) SEEED ESP32-S3 8MB XIAO (seeed_xiao_esp32s3)
 * 
 * CTRL + SHIFT + P
 * pio run -t menufconfig
 * k & l keys for up or down
 * OR
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 *  pio system prune - purge temporary files
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

#include <ak8975.h>


#define CONFIG_I2C_0_PORT               I2C_NUM_0
#define CONFIG_I2C_0_SDA_IO             (gpio_num_t)(45) // blue
#define CONFIG_I2C_0_SCL_IO             (gpio_num_t)(48) // yellow
//
#define CONFIG_I2C_0_TASK_NAME          "i2c_0_tsk"
#define CONFIG_I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define CONFIG_I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define CONFIG_APP_TAG                  "AK8975 [APP]"

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

static inline int i2c_detect(i2c_master_bus_handle_t master_bus_handle) {
    const uint16_t probe_timeout_ms = 50; // timeout in milliseconds
    uint8_t address;

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");

    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(master_bus_handle, address, probe_timeout_ms);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    return 0;  
}

static void i2c_0_task( void *pvParameters ) {
    TickType_t                   xLastWakeTime;
    //
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime               = xTaskGetTickCount ();
    //
    // initialize master i2c 0 bus configuration
    i2c_master_bus_config_t     i2c0_master_cfg = CONFIG_I2C_0_MASTER_DEFAULT;
    i2c_master_bus_handle_t     i2c0_bus_hdl;
    //
    // initialize ak8975 i2c device configuration
    i2c_ak8975_config_t         ak8975_dev_cfg = I2C_AK8975_CONFIG_DEFAULT;
    i2c_ak8975_handle_t         ak8975_dev_hdl;
    //
    //
    // instantiate i2c 0 master bus
    i2c_new_master_bus(&i2c0_master_cfg, &i2c0_bus_hdl);
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // ak8975 init device
    i2c_ak8975_init(i2c0_bus_hdl, &ak8975_dev_cfg, &ak8975_dev_hdl);
    if (ak8975_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create ak8975 handle init failed");
    //
    ESP_LOGW(CONFIG_APP_TAG, "ak8975 device id   %02x", ak8975_dev_hdl->dev_params->device_id);
    ESP_LOGW(CONFIG_APP_TAG, "ak8975 device info %02x", ak8975_dev_hdl->dev_params->device_info);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(CONFIG_APP_TAG, "######################## AK8975 START #########################");
        //
        //i2c_detect(i2c0_bus_hdl);
        //
        i2c_ak8975_compass_axes_data_t compass_axes;
        //
        //
        
        if(i2c_ak8975_get_compass_axes(ak8975_dev_hdl, &compass_axes) == ESP_OK) {
            ESP_LOGI(CONFIG_APP_TAG, "X-Axis:  %d", compass_axes.x_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Y-Axis:  %d", compass_axes.y_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Z-Axis:  %d", compass_axes.z_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Heading: %f", i2c_ak8975_process_heading(compass_axes));
        } else {
            ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_ak8975_get_heading failed");
        }
        
        //
        //
        /*
        if(i2c_ak8975_self_test(ak8975_dev_hdl, &compass_axes) == ESP_OK) {
            ESP_LOGI(CONFIG_APP_TAG, "X:     %d", compass_axes.x_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Y:     %d", compass_axes.y_axis);
            ESP_LOGI(CONFIG_APP_TAG, "Z:     %d", compass_axes.z_axis);
        } else {
            ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_ak8975_self_test failed");
        }
        */
        //
        /*
        i2c_bytes_to_int16_t test_1 = { .value = 0x0FFF }; // 0x0FFF = 4095
        i2c_bytes_to_int16_t test_2 = { .value = 0x0001 }; // 0x0001 = 1
        i2c_bytes_to_int16_t test_3 = { .value = 0xFFFF }; // 0xFFFF = -1
        i2c_bytes_to_int16_t test_4 = { .value = 0xF000 }; // 0xF000 = -4096
        //
        ESP_LOGW(CONFIG_APP_TAG, "test data 1 (%d): byte[0] %02x | byte[1] %02x", test_1.value, test_1.bytes[0], test_1.bytes[1]);
        ESP_LOGW(CONFIG_APP_TAG, "test data 2 (%d): byte[0] %02x | byte[1] %02x", test_2.value, test_2.bytes[0], test_2.bytes[1]);
        ESP_LOGW(CONFIG_APP_TAG, "test data 3 (%d): byte[0] %02x | byte[1] %02x", test_3.value, test_3.bytes[0], test_3.bytes[1]);
        ESP_LOGW(CONFIG_APP_TAG, "test data 4 (%d): byte[0] %02x | byte[1] %02x", test_4.value, test_4.bytes[0], test_4.bytes[1]);
        */
        //
        ESP_LOGI(CONFIG_APP_TAG, "######################## AK8975 END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    i2c_ak8975_rm( ak8975_dev_hdl );    //remove ak8975 device from master i2c bus
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