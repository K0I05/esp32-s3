/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file mpu6050_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <mpu6050_task.h>
#include <mpu6050.h>


void i2c0_mpu6050_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_mpu6050_config_t dev_cfg          = I2C_MPU6050_CONFIG_DEFAULT;
    i2c_mpu6050_handle_t dev_hdl;
    //
    // init device
    i2c_mpu6050_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "mpu6050 handle init failed");
        assert(dev_hdl);
    }

    // show registers
    ESP_LOGI(APP_TAG, "Sample Rate Divider Register:         0x%02x (%s)", dev_hdl->sample_rate_divider_reg, uint8_to_binary(dev_hdl->sample_rate_divider_reg));
    ESP_LOGI(APP_TAG, "Configuration Register:               0x%02x (%s)", dev_hdl->config_reg.reg, uint8_to_binary(dev_hdl->config_reg.reg));
    ESP_LOGI(APP_TAG, "Gyroscope Configuration Register:     0x%02x (%s)", dev_hdl->gyro_config_reg.reg, uint8_to_binary(dev_hdl->gyro_config_reg.reg));
    ESP_LOGI(APP_TAG, "Accelorometer Configuration Register: 0x%02x (%s)", dev_hdl->accel_config_reg.reg, uint8_to_binary(dev_hdl->accel_config_reg.reg));
    ESP_LOGI(APP_TAG, "Interrupt Enable Register:            0x%02x (%s)", dev_hdl->irq_enable_reg.reg, uint8_to_binary(dev_hdl->irq_enable_reg.reg));
    ESP_LOGI(APP_TAG, "Power Management 1 Register:          0x%02x (%s)", dev_hdl->power_management1_reg.reg, uint8_to_binary(dev_hdl->power_management1_reg.reg));
    ESP_LOGI(APP_TAG, "Power Management 2 Register:          0x%02x (%s)", dev_hdl->power_management2_reg.reg, uint8_to_binary(dev_hdl->power_management2_reg.reg));
    ESP_LOGI(APP_TAG, "Who am I Register:                    0x%02x (%s)", dev_hdl->who_am_i_reg.reg, uint8_to_binary(dev_hdl->who_am_i_reg.reg));
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## MPU6050 - START #########################");
        //
        // handle sensor
        float temperature;
        i2c_mpu6050_gyro_data_axes_t gyro_data;
        i2c_mpu6050_accel_data_axes_t accel_data;
        esp_err_t result = i2c_mpu6050_get_motion(dev_hdl, &gyro_data, &accel_data, &temperature);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "mpu6050 device read failed (%s)", esp_err_to_name(result));
        } else {
            /* pitch and roll */
            float pitch = atanf(accel_data.x_axis / sqrtf(powf(accel_data.y_axis, 2.0f) + powf(accel_data.z_axis, 2.0f)));
            float roll  = atanf(accel_data.y_axis / sqrtf(powf(accel_data.x_axis, 2.0f) + powf(accel_data.z_axis, 2.0f)));

            ESP_LOGI(APP_TAG, "Accelorometer X-Axis: %fg", accel_data.x_axis);
            ESP_LOGI(APP_TAG, "Accelorometer Y-Axis: %fg", accel_data.y_axis);
            ESP_LOGI(APP_TAG, "Accelorometer Z-Axis: %fg", accel_data.z_axis);
            ESP_LOGI(APP_TAG, "Gyroscope X-Axis:     %f°/sec", gyro_data.x_axis);
            ESP_LOGI(APP_TAG, "Gyroscope Y-Axis:     %f°/sec", gyro_data.y_axis);
            ESP_LOGI(APP_TAG, "Gyroscope Z-Axis:     %f°/sec", gyro_data.z_axis);
            ESP_LOGI(APP_TAG, "Temperature:          %f°C", temperature);
            ESP_LOGI(APP_TAG, "Pitch Angle:          %f°", pitch);
            ESP_LOGI(APP_TAG, "Roll Angle:           %f°", roll);
        }
        //
        ESP_LOGI(APP_TAG, "######################## MPU6050 - END ###########################");
        //
        //
        // pause the task per defined wait period
        //vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE / 2 );
        vTaskDelaySecUntil( &last_wake_time, 1 );
    }
    //
    // free resources
    i2c_mpu6050_delete( dev_hdl );
    vTaskDelete( NULL );
}