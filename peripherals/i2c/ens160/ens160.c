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
 * @file ens160.c
 *
 * ESP-IDF driver for ENS160 Air Quality sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ens160.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/i2c_master.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "ens160";


/**
 * @brief ENS160 air quality index of the UBA definition table structure.
 */
static const struct I2C_ENS160_AQI_UBA_ROW_TAG i2c_ens160_aqi_uba_definition_table[I2C_ENS160_ERROR_MSG_TABLE_SIZE] = {
  {I2C_ENS160_AQI_UBA_INDEX_UNKNOWN,    "-", "-", "-", "-"},
  {I2C_ENS160_AQI_UBA_INDEX_1,          "Excellent",    "No objections",            "Target", "no limit"},
  {I2C_ENS160_AQI_UBA_INDEX_2,          "Good",         "No relevant objections",   "Sufficient ventilation recommended", "no limit"},
  {I2C_ENS160_AQI_UBA_INDEX_3,          "Moderate",     "Some objections",          "Increased ventilation recommended, search for sources", "<12 months"},
  {I2C_ENS160_AQI_UBA_INDEX_4,          "Poor",         "Major objections",         "Intensified ventilation recommended, search for sources", "<1 month"},
  {I2C_ENS160_AQI_UBA_INDEX_5,          "Unhealthy",    "Situation not acceptable", "Use only if unavoidable, intensified ventilation recommended", "hours"}
};

/*
* functions and subrountines
*/

/**
 * @brief Get air quality (uba) index.
 */
static inline i2c_ens160_aqi_uba_indexes_t i2c_ens160_get_aqi_uba_index(i2c_ens160_caqi_data_register_t caqi_data_reg) {
    switch(caqi_data_reg.bits.aqi_uba) {
        case 1: return I2C_ENS160_AQI_UBA_INDEX_1;
        case 2: return I2C_ENS160_AQI_UBA_INDEX_2;
        case 3: return I2C_ENS160_AQI_UBA_INDEX_3;
        case 4: return I2C_ENS160_AQI_UBA_INDEX_4;
        case 5: return I2C_ENS160_AQI_UBA_INDEX_5;
        default:
            return I2C_ENS160_AQI_UBA_INDEX_UNKNOWN;
    }
}

/**
 * @brief Decodes `uint16_t` temperature format to degrees Celsius.
 * 
 * @param[in] t_uin16 compensation temperature from register.
 * @return `float` temperature compensation in degrees Celsius.
 */
static inline float i2c_ens160_decode_temperature(uint16_t t_uint16) {
    return (float)((t_uint16 / 64) - 273.15);
}

/**
 * @brief Encodes temperature in degrees Celsius to `uint16_t` format.
 * 
 * @param[in] t compensation temperature in degrees Celsius.
 * @return `uint16_t` temperature compensation.
 */
static inline uint16_t i2c_ens160_encode_temperature(float t) {
    return (uint16_t)((t + 273.15) * 64);
}

/**
 * @brief Decodes `uint16_t` humidity format.
 * 
 * @param[in] t_uin16 compensation humidity from register.
 * @return `float` humidity compensation.
 */
static inline float i2c_ens160_decode_humidity(uint16_t h_uint16) {
    return (float)(h_uint16 / 512);
}

/**
 * @brief Encodes humidity to `uint16_t` format.
 * 
 * @param[in] h compensation humidity.
 * @return `uint16_t` humidity compensation.
 */
static inline uint16_t i2c_ens160_encode_humidity(float h) {
    return (uint16_t)(h * 512);
}

static inline esp_err_t i2c_ens160_get_mode(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_OPMODE_RW, (uint8_t*)ens160_handle->dev_params->mode), TAG, "read operating mode register failed" );

    return ESP_OK;
}

static inline esp_err_t i2c_ens160_set_mode(i2c_ens160_handle_t ens160_handle, const i2c_ens160_operating_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_OPMODE_RW, mode), TAG, "write operating mode register failed" );

    /* set handle operating mode */
    ens160_handle->dev_params->mode = mode;

    return ESP_OK;
}

static inline esp_err_t i2c_ens160_get_command(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_COMMAND_RW, (uint8_t*)ens160_handle->dev_params->command), TAG, "read command register failed" );

    return ESP_OK;
}

static inline esp_err_t i2c_ens160_set_command(i2c_ens160_handle_t ens160_handle, const i2c_ens160_commands_t command) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_COMMAND_RW, command), TAG, "write command register failed" );

    /* set handle command */
    ens160_handle->dev_params->command = command;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_interrupt_config_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_INT_CONFIG_RW, &ens160_handle->dev_params->int_config_reg.reg), TAG, "read interrupt configuration register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ens160_set_interrupt_config_register(i2c_ens160_handle_t ens160_handle, i2c_ens160_interrupt_config_register_t int_config_reg) {
   /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* set interrupt configuration register reserved fields to 0 */
    int_config_reg.bits.reserved1 = 0;
    int_config_reg.bits.reserved2 = 0;
    int_config_reg.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_INT_CONFIG_RW, int_config_reg.reg), TAG, "write interrupt configuration register failed" );

    /* set handle interrupt configuration register */
    ens160_handle->dev_params->int_config_reg = int_config_reg;

    return ESP_OK; 
}

esp_err_t i2c_ens160_get_status_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DEVICE_STATUS_R, &ens160_handle->dev_params->status_reg.reg), TAG, "read device status register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ens160_clear_command_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set normal operation command */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_command(ens160_handle, I2C_ENS160_COMMAND_NORMAL), TAG, "write normal operation command failed" );
    
    /* attempt to set clear general purpose registers command */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_command(ens160_handle, I2C_ENS160_COMMAND_CLEAR_GPR), TAG, "write clear general purpose registers command failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CLEAR_GPR_DELAY_MS));

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CLEAR_GPR_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ens160_get_compensation_registers(i2c_ens160_handle_t ens160_handle) {
    uint16_t t; uint16_t h;

    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c temperature & humidity compensation read transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_TEMP_IN_RW, &t), TAG, "read temperature compensation register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_RH_IN_RW, &h), TAG, "read humidity compensation register failed" );

    /* decode temperature & humidity compensation and set handle parameters */
    ens160_handle->dev_params->temperature_comp = i2c_ens160_decode_temperature(t);
    ens160_handle->dev_params->humidity_comp = i2c_ens160_decode_humidity(h);

    return ESP_OK;
}

esp_err_t i2c_ens160_set_compensation_registers(i2c_ens160_handle_t ens160_handle, float temperature, float humidity) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* encode temperature & humidity compensation */
    uint16_t t = i2c_ens160_encode_temperature(temperature); 
    uint16_t h = i2c_ens160_encode_humidity(humidity);

    /* attempt i2c temperature & humidity compensation write transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_TEMP_IN_RW, t), TAG, "write temperature compensation register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_RH_IN_RW, h), TAG, "write humidity compensation register failed" );

    /* set handle parameters */
    ens160_handle->dev_params->temperature_comp = temperature;
    ens160_handle->dev_params->humidity_comp = humidity;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_part_id(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_PART_ID_R, &ens160_handle->dev_params->part_id), TAG, "read part identifier register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ens160_init(i2c_master_bus_handle_t bus_handle, const i2c_ens160_config_t *ens160_config, i2c_ens160_handle_t *ens160_handle) {
    esp_err_t           ret = ESP_OK;
    i2c_ens160_handle_t  out_handle;

    ESP_ARG_CHECK( bus_handle && ens160_config );

    out_handle = (i2c_ens160_handle_t)calloc(1, sizeof(i2c_ens160_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ens160 device");

    out_handle->dev_params = (i2c_ens160_params_t*)calloc(1, sizeof(i2c_ens160_params_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_params, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ens160 device configuration parameters");

    i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ens160_config->dev_config.device_address,
        .scl_speed_hz       = I2C_ENS160_DATA_RATE_HZ,
    };

    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err, TAG, "i2c new bus failed");
    }

    /* copy configuration */


    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR( i2c_ens160_reset(out_handle), err, TAG, "soft-reset failed" );

    /* attempt to read part identifier */
    ESP_GOTO_ON_ERROR( i2c_ens160_get_part_id(out_handle), err, TAG, "read part identifier failed" );

    /* attempt to read firmware version */

    /* attempt to read interrupt configuration register */
    ESP_GOTO_ON_ERROR( i2c_ens160_get_interrupt_config_register(out_handle), err, TAG, "read interrupt configuration register failed" );

    /* attempt to read status register */
    ESP_GOTO_ON_ERROR( i2c_ens160_get_status_register(out_handle), err, TAG, "read status register failed" );

    /* set device handle */
    *ens160_handle = out_handle;

    return ESP_OK;

    err:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
        return ret;
}

esp_err_t i2c_ens160_get_measurement(i2c_ens160_handle_t ens160_handle, i2c_ens160_air_quality_data_t *data) {
    esp_err_t                       ret = ESP_OK;
    bool                            data_is_ready = false;
    bool                            gpr_data_is_ready = false;
    i2c_ens160_caqi_data_register_t caqi_reg;
    uint16_t                        tvoc_data;
    uint16_t                        eco2_data;
    uint16_t                        ri_raw_0;
    uint16_t                        ri_raw_1;
    uint16_t                        ri_raw_2;
    uint16_t                        ri_raw_3;

    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set operating mode to start  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode(ens160_handle, I2C_ENS160_OPMODE_STANDARD), TAG, "standard mode write failed" );

    /* air quality */

    /* attempt to wait until data is available */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_ens160_get_data_status(ens160_handle, &data_is_ready), err, TAG, "data ready read failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_DATA_READY_DELAY_MS));
    } while (data_is_ready == false);

    /* attempt i2c data read transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_AQI_R, &caqi_reg.value), TAG, "read calculated air quality index data register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_TVOC_R, &tvoc_data), TAG, "read tvoc data register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_ECO2_R, &eco2_data), TAG, "read eco2 data register failed" );

    /* set air quality fields */
    data->uba_aqi = i2c_ens160_get_aqi_uba_index(caqi_reg);
    data->tvoc = tvoc_data;
    data->eco2 = eco2_data;

    /* resistance signals */

    /* attempt to wait until gpr data is available */
    do {
        /* attempt to check if gpr data is ready */
        ESP_GOTO_ON_ERROR( i2c_ens160_get_gpr_data_status(ens160_handle, &gpr_data_is_ready), err, TAG, "gpr data ready read failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_DATA_READY_DELAY_MS));
    } while (gpr_data_is_ready == false);

    /* attempt i2c gpr data read transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_GPR_READ0_R, &ri_raw_0), TAG, "read resitance signal gpr data register (0:1) failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_GPR_READ2_R, &ri_raw_1), TAG, "read resitance signal gpr data register (2:3) failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_GPR_READ4_R, &ri_raw_2), TAG, "read resitance signal gpr data register (4:5) failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_GPR_READ6_R, &ri_raw_3), TAG, "read resitance signal gpr data register (6:7) failed" );

    /* convert raw resitance and set resitance signals */
    data->ri_res_0 = I2C_ENS160_CONVERT_RS_RAW2OHMS_F(ri_raw_0);
    data->ri_res_1 = I2C_ENS160_CONVERT_RS_RAW2OHMS_F(ri_raw_1);
    data->ri_res_2 = I2C_ENS160_CONVERT_RS_RAW2OHMS_F(ri_raw_2);
    data->ri_res_3 = I2C_ENS160_CONVERT_RS_RAW2OHMS_F(ri_raw_3);

    /* attempt to set operating mode to idle  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode(ens160_handle, I2C_ENS160_OPMODE_IDLE), TAG, "idle mode write failed" );

    /* attempt to clear command register */
    ESP_RETURN_ON_ERROR( i2c_ens160_clear_command_register(ens160_handle), TAG, "clear command failed" );

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ens160_get_data_status(i2c_ens160_handle_t ens160_handle, bool *ready) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register failed" );

    /* set ready state */
    *ready = ens160_handle->dev_params->status_reg.bits.new_data;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_gpr_data_status(i2c_ens160_handle_t ens160_handle, bool *ready) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register failed" );

    /* set ready state */
    *ready = ens160_handle->dev_params->status_reg.bits.new_gpr_data;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_error_status(i2c_ens160_handle_t ens160_handle, bool *error) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register failed" );

    /* set error state */
    *error = ens160_handle->dev_params->status_reg.bits.error;

    return ESP_OK;
}

esp_err_t i2c_ens160_deep_sleep(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set operating mode to deep sleep  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode(ens160_handle, I2C_ENS160_OPMODE_DEEP_SLEEP), TAG, "deep sleep mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_reset(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set operating mode to reset  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode(ens160_handle, I2C_ENS160_OPMODE_RESET), TAG, "soft-reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_RESET_DELAY_MS));

    /* attempt to set operating mode to reset  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode(ens160_handle, I2C_ENS160_OPMODE_IDLE), TAG, "idle mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_rm(i2c_ens160_handle_t ens160_handle) {
    ESP_ARG_CHECK( ens160_handle );

    return i2c_master_bus_rm_device(ens160_handle->i2c_dev_handle);
}

const i2c_ens160_aqi_uba_row_t *i2c_ens160_aqi_index_to_definition(const i2c_ens160_aqi_uba_indexes_t index) {
    /* attempt aqi-uba index lookup */
    for (size_t i = 0; i < sizeof(i2c_ens160_aqi_uba_definition_table) / sizeof(i2c_ens160_aqi_uba_definition_table[0]); ++i) {
        if (i2c_ens160_aqi_uba_definition_table[i].index == index) {
            //return i2c_ens160_aqi_uba_definition_table[i];
            return NULL;
        }
    }

    return NULL;
}