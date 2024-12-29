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
#include <esp_timer.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define I2C_ENS160_REG_PART_ID_R            UINT8_C(0x00) //!< ens160 I2C part identifier (default id: 0x01, 0x60)
#define I2C_ENS160_REG_OPMODE_RW            UINT8_C(0x10) //!< ens160 I2C operating mode
#define I2C_ENS160_REG_INT_CONFIG_RW        UINT8_C(0x11) //!< ens160 I2C interrupt pin configuration
#define I2C_ENS160_REG_COMMAND_RW           UINT8_C(0x12) //!< ens160 I2C additional system commands
#define I2C_ENS160_REG_TEMP_IN_RW           UINT8_C(0x13) //!< ens160 I2C host ambient temperature information
#define I2C_ENS160_REG_RH_IN_RW             UINT8_C(0x15) //!< ens160 I2C host relative humidity information
#define I2C_ENS160_REG_DEVICE_STATUS_R      UINT8_C(0x20) //!< ens160 I2C operating status
#define I2C_ENS160_REG_DATA_AQI_R           UINT8_C(0x21) //!< ens160 I2C air quality index
#define I2C_ENS160_REG_DATA_TVOC_R          UINT8_C(0x22) //!< ens160 I2C TVOC concentration (ppm) / ETOH gas signals
#define I2C_ENS160_REG_DATA_ECO2_R          UINT8_C(0x24) //!< ens160 I2C equivalent CO2 concentration (ppm)
#define I2C_ENS160_REG_DATA_BL_R            UINT8_C(0x28) //!< ens160 I2C baseline information
#define I2C_ENS160_REG_DATA_TEMP_R          UINT8_C(0x30) //!< ens160 I2C temperature used in calculations
#define I2C_ENS160_REG_DATA_RH_R            UINT8_C(0x32) //!< ens160 I2C relative humidity used in calculations
#define I2C_ENS160_REG_DATA_MISR_R          UINT8_C(0x38) //!< ens160 I2C data integrity field
#define I2C_ENS160_REG_GPR_WRITE0_RW        UINT8_C(0x40) //!< ens160 I2C general purpose write0 register
#define I2C_ENS160_REG_GPR_WRITE1_RW        UINT8_C(0x41) //!< ens160 I2C general purpose write1 register
#define I2C_ENS160_REG_GPR_WRITE2_RW        UINT8_C(0x42) //!< ens160 I2C general purpose write2 register
#define I2C_ENS160_REG_GPR_WRITE3_RW        UINT8_C(0x43) //!< ens160 I2C general purpose write3 register
#define I2C_ENS160_REG_GPR_WRITE4_RW        UINT8_C(0x44) //!< ens160 I2C general purpose write4 register
#define I2C_ENS160_REG_GPR_WRITE5_RW        UINT8_C(0x45) //!< ens160 I2C general purpose write5 register
#define I2C_ENS160_REG_GPR_WRITE6_RW        UINT8_C(0x46) //!< ens160 I2C general purpose write6 register
#define I2C_ENS160_REG_GPR_WRITE7_RW        UINT8_C(0x47) //!< ens160 I2C general purpose write7 register
#define I2C_ENS160_REG_GPR_READ0_R          UINT8_C(0x48) //!< ens160 I2C general purpose read0 register
#define I2C_ENS160_REG_GPR_READ1_R          UINT8_C(0x49) //!< ens160 I2C general purpose read1 register
#define I2C_ENS160_REG_GPR_READ2_R          UINT8_C(0x4a) //!< ens160 I2C general purpose read2 register
#define I2C_ENS160_REG_GPR_READ3_R          UINT8_C(0x4b) //!< ens160 I2C general purpose read3 register
#define I2C_ENS160_REG_GPR_READ4_R          UINT8_C(0x4c) //!< ens160 I2C general purpose read4 register
#define I2C_ENS160_REG_GPR_READ5_R          UINT8_C(0x4d) //!< ens160 I2C general purpose read5 register
#define I2C_ENS160_REG_GPR_READ6_R          UINT8_C(0x4e) //!< ens160 I2C general purpose read6 register
#define I2C_ENS160_REG_GPR_READ7_R          UINT8_C(0x4f) //!< ens160 I2C general purpose read7 register

#define I2C_ENS160_TEMPERATURE_MAX         (float)(125.0)  //!< ens160 maximum temperature range
#define I2C_ENS160_TEMPERATURE_MIN         (float)(-40.0)  //!< ens160 minimum temperature range
#define I2C_ENS160_HUMIDITY_MAX            (float)(100.0)  //!< ens160 maximum humidity range
#define I2C_ENS160_HUMIDITY_MIN            (float)(0.0)    //!< ens160 minimum humidity range

#define I2C_ENS160_POWERUP_DELAY_MS         UINT16_C(15)            //!< ens160 50ms delay before making i2c transactions
#define I2C_ENS160_APPSTART_DELAY_MS        UINT16_C(25)            //!< ens160 25ms delay before making a measurement
#define I2C_ENS160_CMD_DELAY_MS             UINT16_C(5)             //!< ens160 5ms delay before making the next i2c transaction
#define I2C_ENS160_MODE_DELAY_MS            UINT16_C(10) 
#define I2C_ENS160_RESET_DELAY_MS           UINT16_C(50)            //!< ens160 25ms delay when resetting the device
#define I2C_ENS160_CLEAR_GPR_DELAY_MS       UINT16_C(10)             //!< ens160 2ms delay when clearing general purpose registers
#define I2C_ENS160_DATA_READY_DELAY_MS      UINT16_C(1)             //!< ens160 1ms delay when checking data ready in a loop
#define I2C_ENS160_DATA_POLL_TIMEOUT_MS     UINT16_C(1500)          //!< ens160 1.5s timeout when making a measurement

/*
 * macro definitions
*/
#define I2C_ENS160_CONVERT_RS_RAW2OHMS_F(x) 	(pow (2, (float)(x) / 2048))
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
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
static inline i2c_ens160_aqi_uba_indexes_t i2c_ens160_get_aqi_uba_index(const i2c_ens160_caqi_data_register_t caqi_data_reg) {
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
static inline float i2c_ens160_decode_temperature(const uint16_t t_uint16) {
    return (float)((t_uint16 / 64) - 273.15);
}

/**
 * @brief Encodes temperature in degrees Celsius to `uint16_t` format.
 * 
 * @param[in] t compensation temperature in degrees Celsius.
 * @return `uint16_t` temperature compensation.
 */
static inline uint16_t i2c_ens160_encode_temperature(const float t) {
    return (uint16_t)((t + 273.15) * 64);
}

/**
 * @brief Decodes `uint16_t` humidity format.
 * 
 * @param[in] t_uin16 compensation humidity from register.
 * @return `float` humidity compensation.
 */
static inline float i2c_ens160_decode_humidity(const uint16_t h_uint16) {
    return (float)(h_uint16 / 512);
}

/**
 * @brief Encodes humidity to `uint16_t` format.
 * 
 * @param[in] h compensation humidity.
 * @return `uint16_t` humidity compensation.
 */
static inline uint16_t i2c_ens160_encode_humidity(const float h) {
    return (uint16_t)(h * 512);
}

/**
 * @brief Reads command from ENS160 command register.
 * 
 * @param ens160_handle ENS160 device handle.
 * @param command Command returned from ENS160 command register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ens160_get_command(i2c_ens160_handle_t ens160_handle, i2c_ens160_commands_t *const command) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_COMMAND_RW, (uint8_t*)command), TAG, "read command register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes command to ENS160 command register.
 * 
 * @param ens160_handle ENS160 device handle.
 * @param command ENS160 command for command register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ens160_set_command(i2c_ens160_handle_t ens160_handle, const i2c_ens160_commands_t command) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_COMMAND_RW, command), TAG, "write command register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads operating mode register from ENS160.
 * 
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ens160_get_mode_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_OPMODE_RW, (uint8_t*)&ens160_handle->mode), TAG, "read operating mode register for get mode failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_MODE_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes operating mode register to ENS160.
 * 
 * @param[in] ens160_handle ENS160 device handle.
 * @param[in] mode Operating mode register setting.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ens160_set_mode_register(i2c_ens160_handle_t ens160_handle, const i2c_ens160_operating_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_OPMODE_RW, mode), TAG, "write operating mode register for set mode failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_MODE_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_mode_register(ens160_handle), TAG, "read operating mode register for get mode failed" );

    return ESP_OK;
}

/**
 * @brief Reads configuration registers from ENS160 to initialize the device handle after a reset.
 * 
 * @param ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t i2c_ens160_get_registers(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read interrupt configuration register */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_interrupt_config_register(ens160_handle), TAG, "read interrupt configuration register failed" );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register failed" );

    /* attempt to read compensation registers */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_compensation_registers(ens160_handle), TAG, "read compensation registers failed" );

    /* attempt to read part identifier */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_part_id_register(ens160_handle), TAG, "read part identifier register failed" );

    /* attempt to read firmware version */

    return ESP_OK;
}

esp_err_t i2c_ens160_get_interrupt_config_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_INT_CONFIG_RW, &ens160_handle->irq_config_reg.reg), TAG, "read interrupt configuration register failed" );
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ens160_set_interrupt_config_register(i2c_ens160_handle_t ens160_handle, const i2c_ens160_interrupt_config_register_t irq_config_reg) {
   /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* set interrupt configuration register reserved fields to 0 */
    i2c_ens160_interrupt_config_register_t irq_config = { .reg = irq_config_reg.reg };
    irq_config.bits.reserved1 = 0;
    irq_config.bits.reserved2 = 0;
    irq_config.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_INT_CONFIG_RW, irq_config.reg), TAG, "write interrupt configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_interrupt_config_register(ens160_handle), TAG, "read interrupt configuration register failed" );
    
    return ESP_OK; 
}

esp_err_t i2c_ens160_get_status_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DEVICE_STATUS_R, &ens160_handle->status_reg.reg), TAG, "read device status register failed" );
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ens160_clear_general_purpose_registers(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set normal operation command */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_command(ens160_handle, I2C_ENS160_CMD_NORMAL), TAG, "write normal operation command failed" );
    
    /* attempt to set clear general purpose registers command */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_command(ens160_handle, I2C_ENS160_CMD_CLEAR_GPR), TAG, "write clear general purpose registers command failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CLEAR_GPR_DELAY_MS));

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CLEAR_GPR_DELAY_MS));

    /* attempt to set normal operation command */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_command(ens160_handle, I2C_ENS160_CMD_NORMAL), TAG, "write normal operation command failed" );

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
    ens160_handle->temperature_comp = i2c_ens160_decode_temperature(t);
    ens160_handle->humidity_comp    = i2c_ens160_decode_humidity(h);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ens160_set_compensation_registers(i2c_ens160_handle_t ens160_handle, const float temperature, const float humidity) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* validate temperature argument */
    if(temperature > I2C_ENS160_TEMPERATURE_MAX || temperature < I2C_ENS160_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, write compensation registers failed");
    }

    /* validate humidity argument */
    if(humidity > I2C_ENS160_HUMIDITY_MAX || humidity < I2C_ENS160_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, write compensation registers failed");
    }

    /* encode temperature & humidity compensation */
    uint16_t t = i2c_ens160_encode_temperature(temperature); 
    uint16_t h = i2c_ens160_encode_humidity(humidity);

    /* attempt i2c temperature & humidity compensation write transactions */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_TEMP_IN_RW, t), TAG, "write temperature compensation register failed" );
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_RH_IN_RW, h), TAG, "write humidity compensation register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_compensation_registers(ens160_handle), TAG, "read compensation registers failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_get_part_id_register(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_PART_ID_R, &ens160_handle->part_id), TAG, "read part identifier register failed" );
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ens160_init(i2c_master_bus_handle_t bus_handle, const i2c_ens160_config_t *ens160_config, i2c_ens160_handle_t *ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && ens160_config );

    /* power-up task delay */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, ens160_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ens160 device handle initialization failed", ens160_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_ens160_handle_t out_handle = (i2c_ens160_handle_t)calloc(1, sizeof(i2c_ens160_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ens160 device, init failed");

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ens160_config->dev_config.device_address,
        .scl_speed_hz       = ens160_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    /* copy configuration */
    out_handle->irq_enabled         = ens160_config->irq_enabled;
    out_handle->irq_data_enabled    = ens160_config->irq_data_enabled;
    out_handle->irq_gpr_enabled     = ens160_config->irq_gpr_enabled;
    out_handle->irq_pin_driver      = ens160_config->irq_pin_driver;
    out_handle->irq_pin_polarity    = ens160_config->irq_pin_polarity;

    /* attempt to reset device and initialize device configuration and handle */
    ESP_GOTO_ON_ERROR( i2c_ens160_reset(out_handle), err_handle, TAG, "soft-reset for init failed" );

    /* set device handle */
    *ens160_handle = out_handle;

    /* app-start task delay  */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_ens160_get_measurement(i2c_ens160_handle_t ens160_handle, i2c_ens160_air_quality_data_t *const data) {
    esp_err_t                       ret             = ESP_OK;
    uint64_t                        start_time      = 0;
    bool                            data_is_ready   = false;
    i2c_ens160_caqi_data_register_t caqi_reg;
    uint16_t                        tvoc_data;
    uint16_t                        eco2_data;

    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to poll if data is ready or timeout */
        ESP_GOTO_ON_ERROR( i2c_ens160_get_data_status(ens160_handle, &data_is_ready), err, TAG, "data ready read for measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (I2C_ENS160_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c data read transactions */
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_uint8(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_AQI_R, &caqi_reg.value), err, TAG, "read calculated air quality index data register for measurement failed" );
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_TVOC_R, &tvoc_data), err, TAG, "read tvoc data register for measurement failed" );
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_uint16(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_ECO2_R, &eco2_data), err, TAG, "read eco2 data register for measurement failed" );

    /* set air quality fields */
    data->uba_aqi = i2c_ens160_get_aqi_uba_index(caqi_reg);
    data->tvoc    = tvoc_data;
    data->eco2    = eco2_data;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ens160_get_raw_measurement(i2c_ens160_handle_t ens160_handle, i2c_ens160_air_quality_raw_data_t *const data) {
    esp_err_t       ret                 = ESP_OK;
    uint64_t        start_time          = 0;
    bool            gpr_data_is_ready   = false;
    i2c_uint64_t    rx                  = { };

    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until gpr data is available or timeout */
    do {
        /* attempt to check if gpr data is ready */
        ESP_GOTO_ON_ERROR( i2c_ens160_get_gpr_data_status(ens160_handle, &gpr_data_is_ready), err, TAG, "gpr data ready read for raw measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (I2C_ENS160_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (gpr_data_is_ready == false);

    /* attempt i2c gpr data read transactions */
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte64(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_GPR_READ0_R, &rx), err, TAG, "read resitance signal gpr data registers for raw measurement failed" );

    /* convert gpr raw resitance and set resitance signals */
    data->hp0_rs = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[0] | ((uint16_t)rx[1] << 8)));
    data->hp1_rs = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[2] | ((uint16_t)rx[3] << 8)));
    data->hp2_rs = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[4] | ((uint16_t)rx[5] << 8)));
    data->hp3_rs = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[6] | ((uint16_t)rx[7] << 8)));

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    /* attempt i2c baseline data read transactions */
    ESP_GOTO_ON_ERROR( i2c_master_bus_read_byte64(ens160_handle->i2c_dev_handle, I2C_ENS160_REG_DATA_BL_R, &rx), err, TAG, "read baseline data registers for raw measurement failed" );

    /* convert baseline raw resitance and set resitance signals */
    data->hp0_bl = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[0] | ((uint16_t)rx[1] << 8)));
    data->hp1_bl = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[2] | ((uint16_t)rx[3] << 8)));
    data->hp2_bl = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[4] | ((uint16_t)rx[5] << 8)));
    data->hp3_bl = I2C_ENS160_CONVERT_RS_RAW2OHMS_F((uint32_t)(rx[6] | ((uint16_t)rx[7] << 8)));

    /* attempt to clear general purpose registers */
    //ESP_GOTO_ON_ERROR( i2c_ens160_clear_general_purpose_registers(ens160_handle), err, TAG, "clear general purpose registers failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ens160_get_data_status(i2c_ens160_handle_t ens160_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register for get data status failed" );

    /* set ready state */
    *ready = ens160_handle->status_reg.bits.new_data;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_gpr_data_status(i2c_ens160_handle_t ens160_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register for get general purpose registers data status failed" );

    /* set ready state */
    *ready = ens160_handle->status_reg.bits.new_gpr_data;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_validity_status(i2c_ens160_handle_t ens160_handle, i2c_ens160_validity_flags_t *const state) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register for get validity flag status failed" );

    /* set validity flag state */
    *state = ens160_handle->status_reg.bits.state;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_error_status(i2c_ens160_handle_t ens160_handle, bool *const error) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register for get error status failed" );

    /* set error state */
    *error = ens160_handle->status_reg.bits.error;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_mode_status(i2c_ens160_handle_t ens160_handle, bool *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register for get operating mode status failed" );

    /* set error state */
    *mode = ens160_handle->status_reg.bits.mode;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_status(i2c_ens160_handle_t ens160_handle, bool *const data_ready, bool *const gpr_data_ready, i2c_ens160_validity_flags_t *const state, bool *const error, bool *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_status_register(ens160_handle), TAG, "read status register for get status failed" );

    /* set states */
    *data_ready     = ens160_handle->status_reg.bits.new_data;
    *gpr_data_ready = ens160_handle->status_reg.bits.new_gpr_data;
    *state          = ens160_handle->status_reg.bits.state;
    *error          = ens160_handle->status_reg.bits.error;
    *mode           = ens160_handle->status_reg.bits.mode;

    return ESP_OK;
}

esp_err_t i2c_ens160_get_compensation_factors(i2c_ens160_handle_t ens160_handle, float *const temperature, float *const humidity) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to read compensation registers */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_compensation_registers(ens160_handle), TAG, "read compensation registers failed" );

    /* set output parameters */
    *temperature = ens160_handle->temperature_comp;
    *humidity    = ens160_handle->humidity_comp;

    return ESP_OK;
}

esp_err_t i2c_ens160_set_compensation_factors(i2c_ens160_handle_t ens160_handle, const float temperature, const float humidity) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to write compensation registers */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_compensation_registers(ens160_handle, temperature, humidity), TAG, "write compensation registers failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_enable_standard_mode(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set operating mode to standard  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode_register(ens160_handle, I2C_ENS160_OPMODE_STANDARD), TAG, "write mode for standard operating mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_enable_idle_mode(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set operating mode to idle  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode_register(ens160_handle, I2C_ENS160_OPMODE_IDLE), TAG, "write mode for idle operating mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_enable_deep_sleep_mode(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to set operating mode to deep sleep  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode_register(ens160_handle, I2C_ENS160_OPMODE_DEEP_SLEEP), TAG, "write mode for deep sleep operating mode failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_reset(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* attempt to write operating mode to reset  */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_mode_register(ens160_handle, I2C_ENS160_OPMODE_RESET), TAG, "write mode for soft-reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_ENS160_RESET_DELAY_MS));

    /* attempt read device configuration registers */
    ESP_RETURN_ON_ERROR( i2c_ens160_get_registers(ens160_handle), TAG, "read device configuration registers for reset failed" );

    /* attempt to enable idle operating mode before writing to configuration registers */
    ESP_RETURN_ON_ERROR( i2c_ens160_enable_idle_mode(ens160_handle), TAG, "enable idle operating mode for reset failed" );

    /* attempt to clear general purpose registers */
    ESP_RETURN_ON_ERROR( i2c_ens160_clear_general_purpose_registers(ens160_handle), TAG, "clear general purpose registers for reset failed" );

    /* copy irq configuration from device handle */
    i2c_ens160_interrupt_config_register_t irq_config_reg;
    irq_config_reg.bits.irq_enabled         = ens160_handle->irq_enabled;
    irq_config_reg.bits.irq_data_enabled    = ens160_handle->irq_data_enabled;
    irq_config_reg.bits.irq_gpr_enabled     = ens160_handle->irq_gpr_enabled;
    irq_config_reg.bits.irq_pin_driver      = ens160_handle->irq_pin_driver;
    irq_config_reg.bits.irq_pin_polarity    = ens160_handle->irq_pin_polarity;

    /* attempt to write interrupt configuration register */
    ESP_RETURN_ON_ERROR( i2c_ens160_set_interrupt_config_register(ens160_handle, irq_config_reg), TAG, "write interrupt configuration register for reset failed" );

    /* attempt to enable standard operating mode to start making measurements (idle by default)  */
    ESP_RETURN_ON_ERROR( i2c_ens160_enable_standard_mode(ens160_handle), TAG, "enable standard operating mode for reset failed" );

    return ESP_OK;
}

esp_err_t i2c_ens160_remove(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(ens160_handle->i2c_dev_handle);
}

esp_err_t i2c_ens160_delete(i2c_ens160_handle_t ens160_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ens160_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_ens160_remove(ens160_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(ens160_handle->i2c_dev_handle) {
        free(ens160_handle->i2c_dev_handle);
        free(ens160_handle);
    }

    return ESP_OK;
}

i2c_ens160_aqi_uba_row_t i2c_ens160_aqi_index_to_definition(const i2c_ens160_aqi_uba_indexes_t index) {

    /* attempt aqi-uba index lookup */
    for (size_t i = 0; i < sizeof(i2c_ens160_aqi_uba_definition_table) / sizeof(i2c_ens160_aqi_uba_definition_table[0]); ++i) {
        if (i2c_ens160_aqi_uba_definition_table[i].index == index) {
            return i2c_ens160_aqi_uba_definition_table[i];
        }
    }

    return i2c_ens160_aqi_uba_definition_table[0];
}