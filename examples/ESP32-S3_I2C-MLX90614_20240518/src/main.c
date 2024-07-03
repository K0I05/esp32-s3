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

#define CONFIG_APP_TAG                  "MLX90614 [APP]"

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


static inline void LOGI_config_register_binary(uint16_t config_reg) {
    char buffer[17]; // 16 bits + 1 for null terminator
    buffer[16] = '\0';

    for (int i = 15; i >= 0; --i) {
        buffer[i] = '0' + (config_reg & 1); // '0' or '1'
        config_reg >>= 1; // Shift to the next bit
    }

    ESP_LOGI(CONFIG_APP_TAG, "16-bit Register in Binary: %s", buffer);
}

static inline void LOGI_config_register(i2c_mlx90614_handle_t mlx90614_handle) {

    LOGI_config_register_binary(mlx90614_handle->dev_params->config_reg.reg);

    if(mlx90614_handle->dev_params->config_reg.bit.test_state == I2C_MLX90614_SENSOR_TEST_ENABLED) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-15 sensor test enabled");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-15 sensor test disabled");
    }

    if(mlx90614_handle->dev_params->config_reg.bit.kt2_sign == I2C_MLX90614_KT2_SIGN_POSITIVE) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-14 KT2 sign positive");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-14 KT2 sign negative");
    }

    switch(mlx90614_handle->dev_params->config_reg.bit.gain) {
        case I2C_MLX90614_GAIN_1:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 1 - amplifier is bypassed");
            break;
        case I2C_MLX90614_GAIN_3:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 3");
            break;
        case I2C_MLX90614_GAIN_6:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 6");
            break;
        case I2C_MLX90614_GAIN_12_5:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 12.5");
            break;
        case I2C_MLX90614_GAIN_25:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 25");
            break;
        case I2C_MLX90614_GAIN_50:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 50");
            break;
        case I2C_MLX90614_GAIN_100A:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 100");
            break;
        case I2C_MLX90614_GAIN_100B:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-13-11 gain = 100");
            break;
    }

    switch(mlx90614_handle->dev_params->config_reg.bit.fir) {
        case I2C_MLX90614_FIR_128:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-10-8 FIR = 128");
            break;
        case I2C_MLX90614_FIR_256:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-10-8 FIR = 256");
            break;
        case I2C_MLX90614_FIR_512:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-10-8 FIR = 512");
            break;
        case I2C_MLX90614_FIR_1024:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-10-8 FIR = 1024");
            break;
    }

    if(mlx90614_handle->dev_params->config_reg.bit.k_sign == I2C_MLX90614_K_SIGN_POSITIVE) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-7 K sign positive");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-7 K sign negative");
    }

    switch(mlx90614_handle->dev_params->config_reg.bit.t_sensors) {
        case I2C_MLX90614_GAIN_1:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-5-4 Ta, Tobj1");
            break;
        case I2C_MLX90614_GAIN_3:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-5-4 Ta, Tobj2");
            break;
        case I2C_MLX90614_GAIN_6:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-5-4 Tobj2");
            break;
        case I2C_MLX90614_GAIN_12_5:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-5-4 Tobj1, Tobj2");
            break;
    }

    if(mlx90614_handle->dev_params->config_reg.bit.test_repeat_state == I2C_MLX90614_SENSOR_TEST_REPEAT_ON) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-3 sensor repeat test on");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-3 sensor repeat test off");
    }

    switch(mlx90614_handle->dev_params->config_reg.bit.iir) {
        case I2C_MLX90614_SENSOR_IIR_100:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (100%%) a1=1, b1=0");
            break;
        case I2C_MLX90614_SENSOR_IIR_80:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (80%%) a1=0.8, b1=0.2");
            break;
        case I2C_MLX90614_SENSOR_IIR_67:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (67%%) a1=0.666, b1=0.333");
            break;
        case I2C_MLX90614_SENSOR_IIR_57:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (57%%) a1=0.571, b1=0.428");
            break;
        case I2C_MLX90614_SENSOR_IIR_50:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (50%%) a1=0.5, b1=0.5");
            break;
        case I2C_MLX90614_SENSOR_IIR_25:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (25%%) a1=0.25, b1=0.75");
            break;
        case I2C_MLX90614_SENSOR_IIR_17:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (17%%) a1=0.166(6), b1=0.83(3)");
            break;
        case I2C_MLX90614_SENSOR_IIR_13:
            ESP_LOGI(CONFIG_APP_TAG, "BIT-2-0 IIR (13%%) a1=0.125, b1=0.875");
            break;
    }
}

static inline void LOGI_pwmctrl_register(i2c_mlx90614_handle_t mlx90614_handle) {

    LOGI_config_register_binary(mlx90614_handle->dev_params->pwmctrl_reg.reg);

    ESP_LOGI(CONFIG_APP_TAG, "BIT-15-9 PWM period %.3f ms", mlx90614_handle->dev_params->pwmctrl_reg.pwm_period);

    ESP_LOGI(CONFIG_APP_TAG, "BIT-8-4 PWM repetition number is %u (0...62 step 2)", mlx90614_handle->dev_params->pwmctrl_reg.bit.pwm_repetition);

    if(mlx90614_handle->dev_params->pwmctrl_reg.bit.thermal_mode == I2C_MLX90614_THERMAL_MODE_PWM) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-3 PWM mode selected");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-3 Thermal relay mode selected");
    }

    if(mlx90614_handle->dev_params->pwmctrl_reg.bit.sda_pin_mode == I2C_MLX90614_SDA_PIN_MODE_OPEN_DRAIN) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-2 SDA pin configured as Open-Drain");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-2 SDA pin configured as Push-Pull");
    }

    if(mlx90614_handle->dev_params->pwmctrl_reg.bit.pwm_mode_state == I2C_MLX90614_PWM_MODE_STATE_DISABLED) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-1 PWM mode disabled");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-1 PWM mode enabled");
    }

    if(mlx90614_handle->dev_params->pwmctrl_reg.bit.pwm_mode == I2C_MLX90614_PWM_MODE_EXTENDED) {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-0 PWM extended mode");
    } else {
        ESP_LOGI(CONFIG_APP_TAG, "BIT-0 PWM single mode");
    }
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
    if (i2c0_bus_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_create handle init failed");
    //
    // init i2c devices
    //
    // mlx90614 init device
    i2c_mlx90614_init(i2c0_bus_hdl, &mlx90614_dev_cfg, &mlx90614_dev_hdl);
    if (mlx90614_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "i2c0 i2c_bus_device_create mlx90614 handle init failed");
    //
    ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device configuration register:");
    LOGI_config_register(mlx90614_dev_hdl);
    //
    ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device pwmctrl register:");
    LOGI_pwmctrl_register(mlx90614_dev_hdl);
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
        //ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device configuration register:");
        //LOGI_config_register(mlx90614_dev_hdl);
        //
        //ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device pwmctrl register:");
        //LOGI_pwmctrl_register(mlx90614_dev_hdl);
        //
        ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device configuration settings:");
        ESP_LOGI(CONFIG_APP_TAG, "mlx90614 identification number (init), Hi(32-bit): %lu, Lo(32-bit): %lu", mlx90614_dev_hdl->dev_params->ident_number_hi, mlx90614_dev_hdl->dev_params->ident_number_lo);
        ESP_LOGI(CONFIG_APP_TAG, "mlx90614 object temperature range (init), maximum: %.2f C, minimum: %.2f C", mlx90614_dev_hdl->dev_params->obj_max_temperature, mlx90614_dev_hdl->dev_params->obj_min_temperature);
        ESP_LOGI(CONFIG_APP_TAG, "mlx90614 emissivity (0.1 to 1.0 - init):           %.2f", mlx90614_dev_hdl->dev_params->emissivity);
        //
        ESP_LOGW(CONFIG_APP_TAG, "mlx90614 device measurements:");
        if(i2c_mlx90614_get_ambient_temperature(mlx90614_dev_hdl, &ambient_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "mlx90614 device read ambient temperature failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "mlx90614 ambient temperature:   %.2f C", ambient_temperature);
        }
        //
        if(i2c_mlx90614_get_object1_temperature(mlx90614_dev_hdl, &obj1_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "mlx90614 device read object(1) temperature failed");
        } else {
            ESP_LOGI(CONFIG_APP_TAG, "mlx90614 object(1) temperature: %.2f C", obj1_temperature);
        }
        //
        if(i2c_mlx90614_get_object2_temperature(mlx90614_dev_hdl, &obj2_temperature) != ESP_OK) {
            ESP_LOGE(CONFIG_APP_TAG, "mlx90614 device read object(2) temperature failed");
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