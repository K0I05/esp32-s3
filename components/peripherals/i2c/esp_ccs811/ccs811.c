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
 * @file ccs811.c
 *
 * ESP-IDF driver for CCS811 Air Quality sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ccs811.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
//#include <driver/gpio.h>
//#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * CCS811 definitions
*/

#define I2C_CCS811_HW_ID                    UINT8_C(0x81)   //!< ccs811 I2C hardware identification (0x81)

#define I2C_CCS811_REG_STATUS_R             UINT8_C(0x00)   //!< ccs811 I2C status register (1-byte)
#define I2C_CCS811_REG_MEAS_MODE_RW         UINT8_C(0x01)   //!< ccs811 I2C measurement mode and conditions register (1-byte)
#define I2C_CCS811_REG_ALG_RESULT_DATA_R    UINT8_C(0x02)   //!< ccs811 I2C algorithm results (up to 8-bytes)
#define I2C_CCS811_REG_RAW_DATA_R           UINT8_C(0x03)   //!< ccs811 I2C raw ADC data values (2-bytes)
#define I2C_CCS811_REG_ENV_DATA_W           UINT8_C(0x05)   //!< ccs811 I2C temperature and humidity compensation (4-bytes)
#define I2C_CCS811_REG_NTC_R                UINT8_C(0x06)   //!< ccs811 I2C temperature and humidity compensation (4-bytes)
#define I2C_CCS811_REG_THRESHOLDS_W         UINT8_C(0x10)   //!< ccs811 I2C interrupt threshold when in operation (4-bytes)
#define I2C_CCS811_REG_BASELINE_RW          UINT8_C(0x11)   //!< ccs811 I2C encoded current baseline (2-bytes)
#define I2C_CCS811_REG_HW_ID_R              UINT8_C(0x20)   //!< ccs811 I2C hardware identification register (1-byte), value is 0x81
#define I2C_CCS811_REG_HW_VERSION_R         UINT8_C(0x21)   //!< ccs811 I2C hardware version register (1-byte), value is 0x1x
#define I2C_CCS811_REG_FW_BOOT_VERSION_R    UINT8_C(0x23)   //!< ccs811 I2C firmware boot version register (2-bytes)
#define I2C_CCS811_REG_FW_APP_VERSION_R     UINT8_C(0x24)   //!< ccs811 I2C firmware application version register (2-bytes)
#define I2C_CCS811_REG_INTERNAL_STATE_R     UINT8_C(0xa0)   //!< ccs811 I2C internal status register (1-byte)
#define I2C_CCS811_REG_ERROR_ID_R           UINT8_C(0xe0)   //!< ccs811 I2C error source register from internal status register (1-byte)
#define I2C_CCS811_REG_APP_START_W          UINT8_C(0xf4)   //!< ccs811 I2C application start (1-byte)
#define I2C_CCS811_REG_SW_RESET_W           UINT8_C(0xff)   //!< ccs811 I2C software reset when correct 4-bytes written (0x11 0xe5 0x72 0x8a)

#define I2C_CCS811_ECO2_RANGE_MIN           (400)           //!< ccs811 eCO2 minimum in ppm
#define I2C_CCS811_ECO2_RANGE_MAX           (32768)         //!< ccs811 eCO2 maximum in ppm
#define I2C_CCS811_ETVOC_RANGE_MIN          (0)             //!< ccs811 eTVOC minimum in ppb
#define I2C_CCS811_ETVOC_RANGE_MAX          (29206)         //!< ccs811 eTVOC maximum in ppb
#define I2C_CCS811_TEMPERATURE_RANGE_MIN    (-25)           //!< ccs811 temperature minimum in degrees Celsius
#define I2C_CCS811_TEMPERATURE_RANGE_MAX    (50)            //!< ccs811 temperature maximum in degrees Celsius
#define I2C_CCS811_HUMIDITY_RANGE_MIN       (0)             //!< ccs811 relative humidity minimum in percent
#define I2C_CCS811_HUMIDITY_RANGE_MAX       (100)           //!< ccs811 relative humidity maximum in percent

#define I2C_CCS811_POWERUP_DELAY_MS         UINT16_C(25)    //!< ccs811 I2C start-up delay before device accepts transactions
#define I2C_CCS811_APPSTART_DELAY_MS        UINT16_C(25)            
#define I2C_CCS811_RESET_DELAY_MS           UINT16_C(100)   //!< ccs811 I2C software reset delay before device accepts transactions
#define I2C_CCS811_WAKE_DELAY_MS            UINT16_C(5)     //!< ccs811 I2C wake-up delay from sleep before device accepts transactions
#define I2C_CCS811_DATA_READY_DELAY_MS      UINT16_C(10)
#define I2C_CCS811_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define I2C_CCS811_ERASE_DELAY_MS           UINT16_C(500)   //!< ccs811 I2C erase delay before device accepts transactions
#define I2C_CCS811_VERIFY_DELAY_MS          UINT16_C(70)    //!< ccs811 I2C verification delay before device accepts transactions

/*
 * macro definitions
*/
#define I2C_CCS811_SW_RESET_DATA    { 0x11, 0xe5, 0x72, 0x8a }
#define I2C_CCS811_ERASE_DATA       { 0xe7, 0xa7, 0xe6, 0x09 }

#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declerations
*/
static const char *TAG = "ccs811";

/**
 * @brief CCS811 I2C unknown error message.
 */
static const char *i2c_ccs811_unknown_msg = "Unknown error.";

/**
 * @brief CCS811 I2C unknown error code.
 */
static const char *i2c_ccs811_unknown_code = "UNKNOWN.";


/**
 * @brief CCS811 I2C error definition table structure.
 */
static const struct I2C_CCS811_ERROR_ROW_TAG i2c_ccs811_error_definition_table[I2C_CCS811_ERROR_TABLE_SIZE] = {
  {"WRITE_REG_INVALID",  "The CCS811 received an I²C write request addressed to this station but with invalid register address ID"},
  {"READ_REG_INVALID",   "The CCS811 received an I²C read request to a mailbox ID that is invalid"},
  {"DRIVERMODE_INVALID", "The CCS811 received an I²C request to write an unsupported mode to driver mode"},
  {"MAX_RESISTANCE",     "The sensor resistance measurement has reached or exceeded the maximum range"},
  {"HEATER_FAULT",       "The Heater current in the CCS811 is not in range"},
  {"HEATER_SUPPLY",      "The Heater voltage is not being applied correctly"}
};

/**
 * @brief CCS811 I2C unknown measure mode.
 */
static const char* i2c_ccs811_unknown_measure_mode = "UNKNOWN.";

/**
 * @brief CCS811 I2C measure mode definition table structure.
 */
static const struct I2C_CCS811_MEASURE_MODE_ROW_TAG i2c_ccs811_measure_mode_definition_table[I2C_CCS811_MEASURE_MODE_TABLE_SIZE] = {
    {I2C_CCS811_DRIVE_MODE_IDLE,                    "Idle - measurements are disabled"},
    {I2C_CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ,      "Constant Power IAQ - iaq measurement every second"},
    {I2C_CCS811_DRIVE_MODE_PULSE_HEATING_IAQ,       "Pulse Heating IAQ - iaq measurement every 10-seconds"},
    {I2C_CCS811_DRIVE_MODE_LP_PULSE_HEATING_IAQ,    "Low-Power Pulse Heating IAQ - iaq measurement every 60-seconds"},
    {I2C_CCS811_DRIVE_MODE_CONSTANT_POWER,          "Constant Power - measurement every 250ms"}
};



/*
* functions and subrountines
*/

/**
 * @brief Gets CCS811 microsecond duration from device handle.  See datasheet for details.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return duration in microseconds.
 */
static inline uint64_t i2c_ccs811_get_duration_us(i2c_ccs811_handle_t ccs811_handle) {
    switch (ccs811_handle->measure_mode_reg.bits.drive_mode) {
        case I2C_CCS811_DRIVE_MODE_IDLE:
            return 0;   // stand-by 
        case I2C_CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ:
            return 1500000; // 1-second (1000-ms)
        case I2C_CCS811_DRIVE_MODE_PULSE_HEATING_IAQ:
            return 15000000; // 10-seconds (10000-ms)
        case I2C_CCS811_DRIVE_MODE_LP_PULSE_HEATING_IAQ:
            return 65000000; // 60-seconds (60000-ms)
        case I2C_CCS811_DRIVE_MODE_CONSTANT_POWER:
            return 300000; // 250-ms
        default:
            return 1500000;
    }
}

esp_err_t i2c_ccs811_get_status_register(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_STATUS_R, &ccs811_handle->status_reg.reg), TAG, "read status register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ccs811_get_measure_mode_register(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_MEAS_MODE_RW, &ccs811_handle->measure_mode_reg.reg), TAG, "read measure mode register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ccs811_set_measure_mode_register(i2c_ccs811_handle_t ccs811_handle, const i2c_ccs811_measure_mode_register_t measure_mode_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* set reserved bits */
    i2c_ccs811_measure_mode_register_t measure_mode = { .reg = measure_mode_reg.reg };
    measure_mode.bits.reserved1 = 0;
    measure_mode.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint8(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_MEAS_MODE_RW, measure_mode.reg), TAG, "write measure mode register failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_ccs811_get_measure_mode_register(ccs811_handle), TAG, "read measure mode register failed" );


    return ESP_OK;
}

esp_err_t i2c_ccs811_get_error_register(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_ERROR_ID_R, &ccs811_handle->error_reg.reg), TAG, "read error identifier register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ccs811_set_environmental_data_register(i2c_ccs811_handle_t ccs811_handle, const float temperature, const float humidity) {
    i2c_uint40_t tx = { I2C_CCS811_REG_ENV_DATA_W, 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* validate temperature range (-25 to 50 C) */
    ESP_RETURN_ON_FALSE(!(temperature < I2C_CCS811_TEMPERATURE_RANGE_MIN), ESP_ERR_INVALID_ARG, TAG, "Temperature must within a range of -25 to 50 degrees Celsius");
    ESP_RETURN_ON_FALSE(!(temperature > I2C_CCS811_TEMPERATURE_RANGE_MAX), ESP_ERR_INVALID_ARG, TAG, "Temperature must within a range of -25 to 50 degrees Celsius");

	/* validate humidity range (0 to 100 %) */
    ESP_RETURN_ON_FALSE(!(humidity < I2C_CCS811_HUMIDITY_RANGE_MIN), ESP_ERR_INVALID_ARG, TAG, "Relative humidity must within a range of 0 to 100 percent");
    ESP_RETURN_ON_FALSE(!(humidity > I2C_CCS811_HUMIDITY_RANGE_MAX), ESP_ERR_INVALID_ARG, TAG, "Relative humidity must within a range of 0 to 100 percent");

    /* note: application note appears to be incorrect with value to register conversion */

    /* temperature with offset and humidity multipliers */
    uint32_t h_uint = humidity * 1000;              // 42.348 becomes 42348
	uint32_t t_uint = (temperature * 1000) + 25000;	// 23.2 becomes 23200 with 25 C offset

    /* set frame data */

    // correct rounding, see issue 8: https://github.com/sparkfun/Qwiic_BME280_CCS811_Combo/issues/8
	tx[1] = (h_uint + 250) / 500;
	tx[2] = 0; // CCS811 only supports increments of 0.5 so bits 7-0 will always be zero

    // correct rounding
	tx[3] = (t_uint + 250) / 500;
	tx[4] = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ccs811_handle->i2c_dev_handle, tx, I2C_UINT40_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write environmental data failed" );

    /* set handle environmental data parameters */
    ccs811_handle->enviromental_data_reg.humidity    = humidity;
    ccs811_handle->enviromental_data_reg.temperature = temperature;

    return ESP_OK;
}

esp_err_t i2c_ccs811_set_thresholds_register(i2c_ccs811_handle_t ccs811_handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis) {
    i2c_uint48_t                    tx                 = { I2C_CCS811_REG_THRESHOLDS_W, 0, 0, 0, 0, 0 };
    i2c_ccs811_threshold_value_t    low_to_med_value   = { .value = low_to_med };
    i2c_ccs811_threshold_value_t    med_to_high_value  = { .value = med_to_high };

    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* validate eCO2 threshold ranges (400 to 32768) */
    ESP_RETURN_ON_FALSE(!(low_to_med >= med_to_high),               ESP_ERR_INVALID_ARG, TAG, "Low to medium threshold must be less than medium to high threshold");
    ESP_RETURN_ON_FALSE(!(med_to_high <= low_to_med),               ESP_ERR_INVALID_ARG, TAG, "Medium to high threshold must be greater than low to medium threshold");
    ESP_RETURN_ON_FALSE(!(low_to_med < I2C_CCS811_ECO2_RANGE_MIN),  ESP_ERR_INVALID_ARG, TAG, "Low to medium threshold must within a range of 400 to 32768 ppm");
    ESP_RETURN_ON_FALSE(!(low_to_med > I2C_CCS811_ECO2_RANGE_MAX),  ESP_ERR_INVALID_ARG, TAG, "Low to medium threshold must within a range of 400 to 32768 ppm");
    ESP_RETURN_ON_FALSE(!(med_to_high < I2C_CCS811_ECO2_RANGE_MIN), ESP_ERR_INVALID_ARG, TAG, "Medium to high threshold must within a range of 400 to 32768 ppm");
    ESP_RETURN_ON_FALSE(!(med_to_high > I2C_CCS811_ECO2_RANGE_MAX), ESP_ERR_INVALID_ARG, TAG, "Medium to high threshold must within a range of 400 to 32768 ppm");

    /* set frame data */
    tx[1] = low_to_med_value.bits.hi_byte;
    tx[2] = low_to_med_value.bits.lo_byte;
    tx[3] = med_to_high_value.bits.hi_byte;
    tx[4] = med_to_high_value.bits.lo_byte;
    tx[5] = hysteresis;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ccs811_handle->i2c_dev_handle, tx, I2C_UINT48_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write thresholds failed" );

    /* set handle threshold parameters */
    ccs811_handle->thresholds_reg.low_to_med  = low_to_med_value;
    ccs811_handle->thresholds_reg.med_to_high = med_to_high_value;
    ccs811_handle->thresholds_reg.hysteresis  = hysteresis;

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_baseline_register(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

     /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint16(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_BASELINE_RW, &ccs811_handle->baseline_reg), TAG, "read baseline register failed" );

    return ESP_OK;
}

esp_err_t i2c_ccs811_set_baseline_register(i2c_ccs811_handle_t ccs811_handle, const uint16_t baseline) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_uint16(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_BASELINE_RW, baseline), TAG, "write baseline register failed" );

    /* set handle baseline parameters */
    ccs811_handle->baseline_reg = baseline;

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_hardware_identifier_register(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_HW_ID_R, &ccs811_handle->hardware_id), TAG, "read hardware identifier register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ccs811_get_hardware_version_register(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_uint8(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_HW_VERSION_R, &ccs811_handle->hardware_version), TAG, "read hardware version register failed" );
    
    return ESP_OK;
}

esp_err_t i2c_ccs811_start_application(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_write_cmd(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_APP_START_W), TAG, "write application start register failed" );

    return ESP_OK;
}

esp_err_t i2c_ccs811_init(i2c_master_bus_handle_t bus_handle, const i2c_ccs811_config_t *ccs811_config, i2c_ccs811_handle_t *ccs811_handle) {
    gpio_config_t       io_conf         = {};
    uint64_t            gpio_pin_sel;

    /* validate arguments */
    ESP_ARG_CHECK( bus_handle && ccs811_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, ccs811_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ccs811 device handle initialization failed", ccs811_config->dev_config.device_address);

    /* validate memory availability for handle */
    i2c_ccs811_handle_t out_handle = (i2c_ccs811_handle_t)calloc(1, sizeof(i2c_ccs811_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ccs811 device");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ccs811_config->dev_config.device_address,
        .scl_speed_hz       = ccs811_config->dev_config.scl_speed_hz,
    };

    /* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* copy device configuration to handle parameters */
    out_handle->reset_io_enabled    = ccs811_config->reset_io_enabled;
    out_handle->reset_io_num        = ccs811_config->reset_io_num;
    out_handle->wake_io_enabled     = ccs811_config->wake_io_enabled;
    out_handle->wake_io_num         = ccs811_config->wake_io_num;

    /* validate and init gpio for reset and/or wake pins */
    if(ccs811_config->reset_io_enabled == true && ccs811_config->wake_io_enabled == true) {
        // validate reset io num
        ESP_GOTO_ON_FALSE(GPIO_IS_VALID_GPIO(ccs811_config->reset_io_num), ESP_ERR_INVALID_ARG, err_handle, TAG, "reset gpio number is invalid, ccs811 device handle initialization failed");
        // validate wake io num
        ESP_GOTO_ON_FALSE(GPIO_IS_VALID_GPIO(ccs811_config->wake_io_num), ESP_ERR_INVALID_ARG, err_handle, TAG, "wake gpio number is invalid, ccs811 device handle initialization failed");
        // set gpio pin bit mask
        gpio_pin_sel = ((1ULL<<ccs811_config->reset_io_num) | (1ULL<<ccs811_config->wake_io_num));
        // interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE; 
        // bit mask of the pins
        io_conf.pin_bit_mask = gpio_pin_sel;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // enable pull-up mode
        io_conf.pull_up_en = 1; 
        // configure gpio with the given settings
        ESP_GOTO_ON_ERROR( gpio_config(&io_conf), err_handle, TAG, "set gpio configuration for reset and wake failed" );
    } else if(ccs811_config->reset_io_enabled == true && ccs811_config->wake_io_enabled == false) {
        // validate reset io num
        ESP_GOTO_ON_FALSE(GPIO_IS_VALID_GPIO(ccs811_config->reset_io_num), ESP_ERR_INVALID_ARG, err_handle, TAG, "reset gpio number is invalid, ccs811 device handle initialization failed");
        // set gpio pin bit mask
        gpio_pin_sel = (1ULL<<ccs811_config->reset_io_num);
        // interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE; 
        // bit mask of the pins
        io_conf.pin_bit_mask = gpio_pin_sel;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // enable pull-up mode
        io_conf.pull_up_en = 1; 
        // configure gpio with the given settings
        ESP_GOTO_ON_ERROR( gpio_config(&io_conf), err_handle, TAG, "set gpio configuration for reset failed" );
    } else if(ccs811_config->reset_io_enabled == false && ccs811_config->wake_io_enabled == true) {
        // validate wake io num
        ESP_GOTO_ON_FALSE(GPIO_IS_VALID_GPIO(ccs811_config->wake_io_num), ESP_ERR_INVALID_ARG, err_handle, TAG, "wake gpio number is invalid, ccs811 device handle initialization failed");
        // set gpio pin bit mask
        gpio_pin_sel = (1ULL<<ccs811_config->wake_io_num);
        // interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // bit mask of the pins
        io_conf.pin_bit_mask = gpio_pin_sel;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // enable pull-up mode 
        io_conf.pull_up_en = 1;
        // configure gpio with the given settings
        ESP_GOTO_ON_ERROR( gpio_config(&io_conf), err_handle, TAG, "set gpio configuration for wake failed" );
    }

    /* validate reset gpio to set io state */
    if(ccs811_config->reset_io_enabled == true) {
        /* active low for reset gpio */
        ESP_GOTO_ON_ERROR( gpio_set_level(ccs811_config->reset_io_num, 1), err_handle, TAG, "set reset gpio level high failed (gpio: %i)", ccs811_config->reset_io_num );
    }

    /* validate wake gpio to wake the device for i2c transactions */
    if(ccs811_config->wake_io_enabled == true) {
        /* active low for wake gpio */
        ESP_GOTO_ON_ERROR( gpio_set_level(ccs811_config->wake_io_num, 0), err_handle, TAG, "set wake gpio level low failed (gpio: %i)", ccs811_config->wake_io_num );
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* attempt to soft-reset */
    ESP_GOTO_ON_ERROR(i2c_ccs811_reset(out_handle), err_handle, TAG, "soft-reset failed");

    vTaskDelay(pdMS_TO_TICKS(100));

    /* attempt to read status register */
    ESP_GOTO_ON_ERROR(i2c_ccs811_get_status_register(out_handle), err_handle, TAG, "read status register failed");

    /* validate application firmware mode */
    if(out_handle->status_reg.bits.firmware_mode != I2C_CCS811_FW_MODE_APP) {
        /* validate bootloader mode */
        ESP_GOTO_ON_FALSE(out_handle->status_reg.bits.app_valid, ESP_ERR_INVALID_STATE, err_handle, TAG, "no valid application for i2c ccs811 device");

        /* attempt tp switch to application mode - start application */
        ESP_GOTO_ON_ERROR(i2c_ccs811_start_application(out_handle), err_handle, TAG, "application start failed");

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_RESET_DELAY_MS));

        /* attempt to read status register */
        ESP_GOTO_ON_ERROR(i2c_ccs811_get_status_register(out_handle), err_handle, TAG, "read status register failed");

        /* validate applicaton firmware mode switch */
        ESP_GOTO_ON_FALSE(out_handle->status_reg.bits.firmware_mode == I2C_CCS811_FW_MODE_APP, ESP_ERR_INVALID_STATE, err_handle, TAG, "unable to start application for i2c ccs811 device");
    }

    /* attempt to read baseline register */
    ESP_GOTO_ON_ERROR(i2c_ccs811_get_baseline_register(out_handle), err_handle, TAG, "read baseline register failed");

    /* attempt to read measure mode register */
    ESP_GOTO_ON_ERROR(i2c_ccs811_get_measure_mode_register(out_handle), err_handle, TAG, "read measure mode register failed");

    /* attempt to read hardware identifier */
    ESP_GOTO_ON_ERROR(i2c_ccs811_get_hardware_identifier_register(out_handle), err_handle, TAG, "read hardware identifier failed");

    /* attempt to read hardware version */
    ESP_GOTO_ON_ERROR(i2c_ccs811_get_hardware_version_register(out_handle), err_handle, TAG, "read hardware version failed");

    /* configuration */

    /* init measure mode register and set register bits from config */
    i2c_ccs811_measure_mode_register_t measure_mode_reg = { .reg = out_handle->measure_mode_reg.reg };
    measure_mode_reg.bits.drive_mode             = ccs811_config->drive_mode;
    measure_mode_reg.bits.irq_data_ready_enabled = ccs811_config->irq_data_ready_enabled;
    measure_mode_reg.bits.irq_threshold_enabled  = ccs811_config->irq_threshold_enabled;

    /* attempt to write measure mode register */
    ESP_GOTO_ON_ERROR(i2c_ccs811_set_measure_mode_register(out_handle, measure_mode_reg), err_handle, TAG, "write measure mode register failed");

    /* set device handle */
    *ccs811_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_ccs811_get_measurement(i2c_ccs811_handle_t ccs811_handle, uint16_t *eco2, uint16_t *etvoc) {
    esp_err_t       ret             = ESP_OK;
    uint64_t        start_time      = 0;
    bool            data_is_ready   = false;
    bool            has_error       = false;
    i2c_uint8_t     tx              = { I2C_CCS811_REG_ALG_RESULT_DATA_R };
    i2c_uint64_t    rx              = { };

    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to wait until data is available */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( i2c_ccs811_get_data_status(ccs811_handle, &data_is_ready), err, TAG, "data ready read failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, i2c_ccs811_get_duration_us(ccs811_handle)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read error state */
    ESP_GOTO_ON_ERROR( i2c_ccs811_get_error_status(ccs811_handle, &has_error), err, TAG, "error read failed." );

    /* validate error state */
    if(has_error == true) {
        /* attempt to read error register */
        ESP_GOTO_ON_ERROR( i2c_ccs811_get_error_register(ccs811_handle), err, TAG, "read error register failed." );

        /* validate error state */
        ESP_GOTO_ON_FALSE(!has_error, ESP_ERR_INVALID_STATE, err, TAG, "error for i2c ccs811 device (%s)", i2c_ccs811_err_to_code(ccs811_handle->error_reg));
    }

    /* attempt i2c write and then read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(ccs811_handle->i2c_dev_handle, tx, I2C_UINT8_SIZE, rx, I2C_UINT64_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "read alg result data failed" );

    /* set eco2 and etvoc values */
    *eco2  = rx[1] | (rx[0] << 8);  // big endian order (msb | lsb)
    *etvoc = rx[3] | (rx[2] << 8);  // big endian order (msb | lsb)

    // eco2_data = ((uint16_t)i2c_buf[0] << 8) | ((uint32_t)i2c_buf[1] << 0));

    //ESP_LOGW(TAG, "eco2  hi-byte 0x%02x | lo-byte 0x%02x (value: %d)", alg_result_data[0], alg_result_data[1], eco2_val);
    //ESP_LOGW(TAG, "etvoc hi-byte 0x%02x | lo-byte 0x%02x (value: %d)", alg_result_data[2], alg_result_data[3], etvoc_val);  

    return ESP_OK;

    err:
        return ret;
}

esp_err_t i2c_ccs811_set_environmental_data(i2c_ccs811_handle_t ccs811_handle, const float temperature, const float humidity) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ccs811_set_environmental_data_register(ccs811_handle, temperature, humidity), TAG, "write environmental data failed" );

    return ESP_OK;
}

esp_err_t i2c_ccs811_set_thresholds(i2c_ccs811_handle_t ccs811_handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_ccs811_set_thresholds_register(ccs811_handle, low_to_med, med_to_high, hysteresis), TAG, "write environmental data failed" );

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_drive_mode(i2c_ccs811_handle_t ccs811_handle, i2c_ccs811_drive_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt to read measure mode register */
    ESP_RETURN_ON_ERROR( i2c_ccs811_get_measure_mode_register(ccs811_handle), TAG, "read measure mode register failed" );

    /* set drive mode */
    *mode = ccs811_handle->measure_mode_reg.bits.drive_mode;

    return ESP_OK;
}

esp_err_t i2c_ccs811_set_drive_mode(i2c_ccs811_handle_t ccs811_handle, const i2c_ccs811_drive_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* copy measure mode register */
    i2c_ccs811_measure_mode_register_t measure_mode_reg = { .reg = ccs811_handle->measure_mode_reg.reg };

    /* set drive mode */
    measure_mode_reg.bits.drive_mode = mode;

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ccs811_set_measure_mode_register(ccs811_handle, measure_mode_reg), TAG, "write measure mode register failed" );

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_firmware_mode(i2c_ccs811_handle_t ccs811_handle, i2c_ccs811_firmware_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ccs811_get_status_register(ccs811_handle), TAG, "read status register (firmware mode state) failed" );

    /* set mode state */
    *mode = ccs811_handle->status_reg.bits.firmware_mode;

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_ntc_resistance(i2c_ccs811_handle_t ccs811_handle, const uint32_t r_ref, uint32_t *const resistance) {
    i2c_uint32_t rx = { 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_bus_read_byte32(ccs811_handle->i2c_dev_handle, I2C_CCS811_REG_NTC_R, &rx), TAG, "read ntc register failed" );

    uint16_t v_ref = (uint16_t)(rx[0] << 8) | rx[1];
    uint16_t v_ntc = (uint16_t)(rx[2] << 8) | rx[3];

    *resistance = v_ntc * r_ref / v_ref;

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_data_status(i2c_ccs811_handle_t ccs811_handle, bool *const ready) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ccs811_get_status_register(ccs811_handle), TAG, "read status register (data ready state) failed" );

    /* set ready state */
    *ready = ccs811_handle->status_reg.bits.data_ready;

    return ESP_OK;
}

esp_err_t i2c_ccs811_get_error_status(i2c_ccs811_handle_t ccs811_handle, bool *const error) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( i2c_ccs811_get_status_register(ccs811_handle), TAG, "read status (error state) register failed" );

    /* set error state */
    *error = ccs811_handle->status_reg.bits.error;

    return ESP_OK;
}

esp_err_t i2c_ccs811_reset(i2c_ccs811_handle_t ccs811_handle) {
    i2c_uint40_t tx  = { };

    const static uint8_t sw_reset[4] = I2C_CCS811_SW_RESET_DATA;

    /* set frame data */
    tx[0] = I2C_CCS811_REG_SW_RESET_W;
    tx[1] = sw_reset[0];
    tx[2] = sw_reset[1];
    tx[3] = sw_reset[2];
    tx[4] = sw_reset[3];

    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(ccs811_handle->i2c_dev_handle, tx, I2C_UINT40_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "write soft-reset data failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ccs811_io_wake(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* validate wake io state */
    ESP_RETURN_ON_FALSE(ccs811_handle->wake_io_enabled, ESP_ERR_INVALID_ARG, TAG, "wake gpio must be enabled");

    /* active low for wake - set wake gpio low */
    ESP_RETURN_ON_ERROR( gpio_set_level(ccs811_handle->wake_io_num, 0), TAG, "set wake gpio level low failed (gpio: %i)", ccs811_handle->wake_io_num );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_WAKE_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_ccs811_io_sleep(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* validate wake io state */
    ESP_RETURN_ON_FALSE(ccs811_handle->wake_io_enabled, ESP_ERR_INVALID_ARG, TAG, "wake gpio must be enabled");

    /* active high for sleep - set wake gpio high */
    ESP_RETURN_ON_ERROR( gpio_set_level(ccs811_handle->wake_io_num, 1), TAG, "set wake gpio level high failed (gpio: %i)", ccs811_handle->wake_io_num );

    return ESP_OK;
}

esp_err_t i2c_ccs811_io_reset(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* validate reset io state */
    ESP_RETURN_ON_FALSE(ccs811_handle->reset_io_enabled, ESP_ERR_INVALID_ARG, TAG, "reset gpio must be enabled");

    /* active low for reset - set reset gpio low */
    ESP_RETURN_ON_ERROR( gpio_set_level(ccs811_handle->reset_io_num, 0), TAG, "set reset gpio level low failed (gpio: %i)", ccs811_handle->reset_io_num );

    /* delay reset pgio in low state - for reset to take effect */
    vTaskDelay(pdMS_TO_TICKS(I2C_CCS811_RESET_DELAY_MS));

    /* set reset gpio high - normal state */
    ESP_RETURN_ON_ERROR( gpio_set_level(ccs811_handle->reset_io_num, 1), TAG, "set reset gpio level high failed (gpio: %i)", ccs811_handle->reset_io_num );

    return ESP_OK;
}

esp_err_t i2c_ccs811_remove(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(ccs811_handle->i2c_dev_handle);
}

esp_err_t i2c_ccs811_delete(i2c_ccs811_handle_t ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( ccs811_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_ccs811_remove(ccs811_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(ccs811_handle->i2c_dev_handle) {
        free(ccs811_handle->i2c_dev_handle);
        free(ccs811_handle);
    }

    return ESP_OK;
}

const char *i2c_ccs811_err_to_message(i2c_ccs811_error_code_register_t error_reg) {
    /* attempt error message lookup */
    if(error_reg.bits.write_register_invalid == true) return i2c_ccs811_error_definition_table[0].msg;
    if(error_reg.bits.read_register_invalid == true) return i2c_ccs811_error_definition_table[1].msg;
    if(error_reg.bits.drive_mode_invalid == true) return i2c_ccs811_error_definition_table[2].msg;
    if(error_reg.bits.max_resistance_exceeded == true) return i2c_ccs811_error_definition_table[3].msg;
    if(error_reg.bits.heater_current_fault == true) return i2c_ccs811_error_definition_table[4].msg;
    if(error_reg.bits.heater_voltage_fault == true) return i2c_ccs811_error_definition_table[5].msg;

    return i2c_ccs811_unknown_msg;
}

const char *i2c_ccs811_err_to_code(const i2c_ccs811_error_code_register_t error_reg) {
    /* attempt error code lookup */
    if(error_reg.bits.write_register_invalid == true) return i2c_ccs811_error_definition_table[0].code;
    if(error_reg.bits.read_register_invalid == true) return i2c_ccs811_error_definition_table[1].code;
    if(error_reg.bits.drive_mode_invalid == true) return i2c_ccs811_error_definition_table[2].code;
    if(error_reg.bits.max_resistance_exceeded == true) return i2c_ccs811_error_definition_table[3].code;
    if(error_reg.bits.heater_current_fault == true) return i2c_ccs811_error_definition_table[4].code;
    if(error_reg.bits.heater_voltage_fault == true) return i2c_ccs811_error_definition_table[5].code;

    return i2c_ccs811_unknown_code;
}

const char *i2c_ccs811_measure_mode_description(const i2c_ccs811_drive_modes_t mode) {
    /* attempt measure mode lookup */
    for(int i = 0; i< I2C_CCS811_MEASURE_MODE_TABLE_SIZE; i++) {
        if(i2c_ccs811_measure_mode_definition_table[i].mode == mode) {
            return i2c_ccs811_measure_mode_definition_table[i].desc;
        }
    }

    return i2c_ccs811_unknown_measure_mode;
}