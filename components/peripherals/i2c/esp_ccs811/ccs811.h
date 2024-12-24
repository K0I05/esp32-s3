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
 * @file ccs811.h
 * @defgroup drivers ccs811
 * @{
 *
 * ESP-IDF driver for ccs811 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CCS811_H__
#define __CCS811_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_event.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CCS811 definitions
*/
#define I2C_CCS811_SCL_SPEED_HZ                 UINT32_C(100000) //!< ccs811 I2C default clock frequency (100KHz)

#define I2C_CCS811_DEV_ADDR_LO                  UINT8_C(0x5a)   //!< ccs811 I2C address when ADDR pin floating/low
#define I2C_CCS811_DEV_ADDR_HI                  UINT8_C(0x5b)   //!< ccs811 I2C address when ADDR pin high


#define I2C_CCS811_ERROR_MSG_SIZE               (150)           //!< ccs811 I2C error message size
#define I2C_CCS811_ERROR_CODE_SIZE              (20)            //!< ccs811 I2C error code size
#define I2C_CCS811_ERROR_TABLE_SIZE             (6)             //!< ccs811 I2C error table size

#define I2C_CCS811_MEASURE_MODE_DESC_SIZE       (100)           //!< ccs811 I2C measure mode description size
#define I2C_CCS811_MEASURE_MODE_TABLE_SIZE      (5)             //!< ccs811 I2C measure mode table size


/**
 * @brief declare of CCS811 monitor event base.
 */
ESP_EVENT_DECLARE_BASE(ESP_CCS811_EVENT);

/*
 * CCS811 macro definitions
*/

#define I2C_CCS811_CONFIG_DEFAULT   {                                           \
        .dev_config.device_address  = I2C_CCS811_DEV_ADDR_LO,                   \
        .dev_config.scl_speed_hz    = I2C_CCS811_SCL_SPEED_HZ,                  \
        .wake_io_enabled            = false,                                    \
        .reset_io_enabled           = false,                                    \
        .threshold_irq_enabled      = false,                                    \
        .data_ready_irq_enabled     = false,                                    \
        .drive_mode                 = I2C_CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ, \
        .set_environmental_data     = false }


/*
 * CCS811 enumerator and sructure declerations
*/

/**
 * CCS811 firmware modes enumerator.
 */
typedef enum {
    I2C_CCS811_FW_MODE_BOOT = 0,   //!< firmware is in boot mode, this allows new firmware to be loaded
    I2C_CCS811_FW_MODE_APP  = 1    //!< formware is in application mode, deivce is ready to take ADC measurements
} i2c_ccs811_firmware_modes_t;


/**
 * @brief CCS811 status register structure.
 */
typedef union __attribute__((packed)) {
    struct STS_REG_BITS_TAG {
        bool                            error:1;                /*!< error when true                        (bit:0)   */
        uint8_t                         reserved:2;             /*!< reserved and set to 0                  (bit:2-1) */
        bool                            data_ready:1;           /*!< data ready when true                   (bit:3)   */
        bool                            app_valid:1;            /*!< application firwmare pased when true   (bit:4)   */
        bool                            app_verify_completed:1; /*!< application verify completed when true (bit:5)   */
        bool                            app_erase_completed:1;  /*!< application erase completed when true  (bit:6)   */
        i2c_ccs811_firmware_modes_t     firmware_mode:1;        /*!< firmware mode                          (bit:7)   */
    } bits;          /*!< represents the 8-bit register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit register as `uint8_t` */
} i2c_ccs811_status_register_t;

/**
 * CCS811 drive modes enumerator.
 */
typedef enum {
    I2C_CCS811_DRIVE_MODE_IDLE                  = (0b000),   //!< idle (measurements are disabled in this mode)
    I2C_CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ    = (0b001),   //!< constant power mode, IAQ measurement every second
    I2C_CCS811_DRIVE_MODE_PULSE_HEATING_IAQ     = (0b010),   //!< pulse heating mode IAQ measurement every 10 seconds
    I2C_CCS811_DRIVE_MODE_LP_PULSE_HEATING_IAQ  = (0b011),   //!< low power pulse heating mode IAQ measurement every 60 seconds
    I2C_CCS811_DRIVE_MODE_CONSTANT_POWER        = (0b100)    //!< constant power mode, sensor measurement every 250ms
} i2c_ccs811_drive_modes_t;

/**
 * @brief CCS811 measure mode and condition register structure.
 */
typedef union __attribute__((packed)) {
    struct MODE_REG_BITS_TAG {
        uint8_t                                  reserved1:2;            /*!< reserved and set to 0                     (bit:1-0) */
        bool                                     threshold_irq:1;        /*!< interrupt threshold asserted when true    (bit:2)   */
        bool                                     data_ready_irq:1;       /*!< interrupt data ready when true            (bit:3)   */
        i2c_ccs811_drive_modes_t                 drive_mode:3;           /*!< drive mode                                (bit:6-4) */
        uint8_t                                  reserved2:1;            /*!< reserved and set to 0                     (bit:7)   */
    } bits;         /*!< represents the 8-bit register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit register as `uint8_t`   */
} i2c_ccs811_measure_mode_register_t;

/**
 * @brief CCS811 raw data register structure (review - not correct).
 */
typedef union __attribute__((packed)) {
    struct RDAT_REG_BITS_TAG {
        uint16_t    raw_adc:10;    /*!< raw adc        (bit:) */
        uint16_t    current:6;     /*!< current        (bit:7)   */
    } bits;         /*!< represents the 16-bit register parts in bits. */
    uint16_t reg;   /*!< represents the 16-bit register as `uint16_t`  */
} i2c_ccs811_raw_data_register_t;

/**
 * @brief CCS811 threshold value structure (big endien).
 */
typedef union __attribute__((packed)) {
    struct THRVAL_REG_BITS_TAG {
        uint8_t    hi_byte:8;  /*!< low to medium threshold (1500ppm = 0x05DC default)  (bit:7-0)    */
        uint8_t    lo_byte:8;  /*!< low to medium threshold (1500ppm = 0x05DC default)  (bit:8-15)   */
    } bits;              /*!< represents the 16-bit register parts in bits. */
    //uint8_t  bytes[2];  /*!< represents the 16-bit register 2-byte array.  */
    uint16_t value;     /*!< represents the 16-bit threshold value register as `uint16_t`  */
} i2c_ccs811_threshold_value_t;

/**
 * @brief CCS811 thresholds register structure.
 */
typedef struct {
    i2c_ccs811_threshold_value_t    low_to_med;  /*!< low to medium threshold (1500ppm = 0x05DC default)   */
    i2c_ccs811_threshold_value_t    med_to_high; /*!< medium to high threshold (2500ppm = 0x09C4 default)  */
    uint8_t                         hysteresis;  /*!< threshold hysteresis value (50ppm = 0x default)*/
} i2c_ccs811_thresholds_register_t;

/**
 * @brief CCS811 environmental data register structure.
 */
typedef struct {
    float temperature;  /*!< temperature (25 C = 0x64, 0x00 default)   */
    float humidity;     /*!< humidity (50% = 0x64, 0x00 default)  */
} i2c_ccs811_environmental_data_register_t;

/**
 * @brief CCS811 firmware version format (bootloader 0x23 and application 0x24) structure (review).
 */
typedef union __attribute__((packed)) {
    struct FWVF_PARTS_TAG {
        uint16_t    major:4;      /*!< major         (bit:3-0)  */
        uint16_t    minor:4;      /*!< minor         (bit:4-7)  */
        uint16_t    trivial:8;    /*!< trivial       (bit:15-8) */
    } parts;            /*!< represents the 16-bit version parts in bits. */
    uint16_t version;   /*!< represents the 16-bit version as `uint16_t`  */
} i2c_ccs811_firmware_version_format_t;

/**
 * @brief CCS811 error codes `ERROR_ID` register structure.
 */
typedef union __attribute__((packed)) {
    struct ERRC_REG_BITS_TAG {
        bool            write_register_invalid:1;   /*!< write register invalid when tru            (bit:0) */
        bool            read_register_invalid:1;    /*!< read register invalid when true            (bit:1) */
        bool            drive_mode_invalid:1;       /*!< measurement drive mode invalid when tru    (bit:2) */
        bool            max_resistance_exceeded:1;  /*!< maximum resistance exceeded when true      (bit:3) */
        bool            heater_current_fault:1;     /*!< heater current supply fault when true      (bit:4) */
        bool            heater_voltage_fault:1;     /*!< heater voltage supply fault when true      (bit:5) */
        uint8_t         reserved1:1;                /*!< reserved and set to 0                      (bit:6) */
        uint8_t         reserved2:1;                /*!< reserved and set to 0                      (bit:7) */
    } bits;          /*!< represents the 8-bit register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit register as `uint8_t`   */
} i2c_ccs811_error_code_register_t;

/**
 * @brief CCS811 I2C error row definition structure.
 */
typedef struct I2C_CCS811_ERROR_ROW_TAG {
    char        code[I2C_CCS811_ERROR_CODE_SIZE];   /*!< error code */
    char        msg[I2C_CCS811_ERROR_MSG_SIZE];     /*!< error meassage */
} i2c_ccs811_error_row_t;

/**
 * @brief CCS811 I2C measure mode row definition structure.
 */
typedef struct I2C_CCS811_MEASURE_MODE_ROW_TAG {
    i2c_ccs811_drive_modes_t    mode;                                           /*!< drive mode */
    char                        desc[I2C_CCS811_MEASURE_MODE_DESC_SIZE]; /*!< drive mode description */
} i2c_ccs811_measre_mode_row_t;

/**
 * @brief CCS811 device event object structure.
 */
typedef struct CCS811_DEVICE_TAG {
    uint16_t eco2;
    uint16_t etvoc;
} ccs811_device_t;

/**
 * @brief CCS811 monitor handle type definition
 *
 */
typedef void *ccs811_monitor_handle_t;

/**
 * @brief CCS811 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t      dev_config;                /*!< configuration for ccs811 device */
    //bool                     irq_io_enabled;            /*!< ccs811 flag to enable hardware interrupt */
    //uint32_t                 irq_io_num;                /*!< mcu interrupt pin number for ccs811 device */
    bool                     wake_io_enabled;           /*!< ccs811 flag to enable hardware wake */
    uint32_t                 wake_io_num;               /*!< mcu wake pin number for ccs811 device */
    bool                     reset_io_enabled;          /*!< ccs811 flag to enable hardware reset */
    uint32_t                 reset_io_num;              /*!< mcu reset pin number for ccs811 device */
    bool                     threshold_irq_enabled;     /*!< interrupt threshold asserted when enabled */
    bool                     data_ready_irq_enabled;    /*!< interrupt data ready asserted when enabled  */
    i2c_ccs811_drive_modes_t drive_mode;                /*!< drive mode */
    bool                     set_environmental_data;    /*!< flag to set user-defined environmental data */
    i2c_ccs811_environmental_data_register_t environmental_data; /*!< user-defined environmental data */
} i2c_ccs811_config_t;


/**
 * @brief CCS811 I2C device handle structure.
 */
struct i2c_ccs811_t {
    i2c_master_dev_handle_t                 i2c_dev_handle;  /*!< I2C device handle */
    uint8_t                                 hardware_id;            /*!< ccs811 hardware identifier (static 0x81) */
    uint8_t                                 hardware_version;       /*!< ccs811 hardware version (0x1X) */
    i2c_ccs811_firmware_version_format_t    bootloader_version;     /*!< ccs811 firmware bootloader version */
    i2c_ccs811_firmware_version_format_t    application_version;    /*!< ccs811 firmware application version */
    i2c_ccs811_status_register_t            status_reg;             /*!< ccs811 status register */
    i2c_ccs811_measure_mode_register_t      measure_mode_reg;       /*!< ccs811 measure mode register */
    i2c_ccs811_error_code_register_t        error_reg;              /*!< ccs811 error identifier regiser */
    i2c_ccs811_environmental_data_register_t enviromental_data_reg; /*!< ccs811 environmental data register */
    i2c_ccs811_thresholds_register_t        thresholds_reg;         /*!< ccs811 thresholds register */
    uint16_t                                baseline_reg;           /*!< ccs811 baseline register */
    bool                                    wake_io_enabled;        /*!< ccs811 flag to enable hardware wake */
    uint32_t                                wake_io_num;            /*!< mcu wake pin number for ccs811 device */
    bool                                    reset_io_enabled;       /*!< ccs811 flag to enable hardware reset */
    uint32_t                                reset_io_num;           /*!< mcu reset pin number for ccs811 device */
};

/**
 * @brief CCS811 I2C device handle type definitions
*/
typedef struct i2c_ccs811_t i2c_ccs811_t;
typedef struct i2c_ccs811_t *i2c_ccs811_handle_t;


/**
 * @brief Initialize CCS811 monitor instance.
 * 
 * @param[in] bus_handle i2c master bus handle.
 * @param[in] ccs811_config CCS811 configuration.
 * @return ccs811_monitor_handle_t CCS811 monitor handle.
 */
ccs811_monitor_handle_t ccs811_monitor_init(i2c_master_bus_handle_t bus_handle, const i2c_ccs811_config_t *ccs811_config);

/**
 * @brief De-initialize CCS811 monitor instance.
 * 
 * @param[in] monitor_handle CCS811 monitor handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_monitor_deinit(ccs811_monitor_handle_t monitor_handle);

/**
 * @brief Adds user defined event handler for CCS811 monitor.
 * 
 * @param[in] monitor_handle CCS811 monitor handle.
 * @param[in] event_handler user defined event handler.
 * @param[out] handler_args handler specific arguments.
 * @return esp_err_t
 *  - ESP_OK on success.
 *  - ESP_ERR_NO_MEM when unable to allocate memory for the handler.
 *  - ESP_ERR_INVALIG_ARG when invalid combination of event base and event id is provided.
 */
esp_err_t ccs811_monitor_add_handler(ccs811_monitor_handle_t monitor_handle, esp_event_handler_t event_handler, void *handler_args);

/**
 * @brief Removes user defined event handler for CCS811 monitor.
 * 
 * @param[in] monitor_handle CCS811 monitor handle.
 * @param[in] event_handler user defined event handler.
 * @return esp_err_t
 *  - ESP_OK on success.
 *  - ESP_ERR_INVALIG_ARG when invalid combination of event base and event id is provided.
 */
esp_err_t ccs811_monitor_remove_handler(ccs811_monitor_handle_t monitor_handle, esp_event_handler_t event_handler);

/**
 * @brief Reads status register from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_status_register(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Reads measure mode register from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_measure_mode_register(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Writes measure mode register to CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[in] mode_reg measure mode register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_set_measure_mode_register(i2c_ccs811_handle_t ccs811_handle, const i2c_ccs811_measure_mode_register_t mode_reg);

/**
 * @brief Reads error register from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_error_register(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Writes environmental compensation data to CCS811 register.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[in] temperature temperature compensation in degrees Celsius (default: 25 C).
 * @param[in] humidity relative humidity compensation in percentage (default: 50 %).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_set_environmental_data_register(i2c_ccs811_handle_t ccs811_handle, const float temperature, const float humidity);

/**
 * @brief Writes eCO2 thresholds to CCS811 register.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[in] low_to_med Low to medium threshold  within a range of 400 to 32768 ppm (1500 ppm).
 * @param[in] med_to_high Medium to high threshold within a range of 400 to 32768 ppm (2500 ppm).
 * @param[in] hysteresis Variance for thresholds in ppm (default: 50 ppm).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_set_thresholds_register(i2c_ccs811_handle_t ccs811_handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis);

/**
 * @brief Reads encoded version of the current baseline register from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_baseline_register(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Writes encoded version to the CCS811 baseline register.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[in] baseline encoded version of the baseline.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_set_baseline_register(i2c_ccs811_handle_t ccs811_handle, const uint16_t baseline);

/**
 * @brief Reads hardware identifier from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_hardware_identifier(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Reads hardware version from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_hardware_version(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Starts the CCS811 application.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_start_application(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Initializes a CCS811 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ccs811_config Configuration of CCS811 device.
 * @param[out] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_init(i2c_master_bus_handle_t bus_handle, const i2c_ccs811_config_t *ccs811_config, i2c_ccs811_handle_t *ccs811_handle);

/**
 * @brief Reads air quality measurement from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[out] eco2 Equivalent CO2 in ppm (range is from 400 to 32768 ppm).
 * @param[out] etvoc Equivalent Total Volatile Oragnic Compound (TVOC) in ppb (range is from 0 to 29206 ppb).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_measurement(i2c_ccs811_handle_t ccs811_handle, uint16_t *eco2, uint16_t *etvoc);

/**
 * @brief Reads drive mode from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[out] mode Measurement drive mode.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_mode(i2c_ccs811_handle_t ccs811_handle, i2c_ccs811_drive_modes_t *const mode);

/**
 * @brief Writes drive mode to CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[in] mode Measurement drive mode.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_set_mode(i2c_ccs811_handle_t ccs811_handle, const i2c_ccs811_drive_modes_t mode);

/**
 * @brief Reads firmware mode from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[out] mode Firmware mode.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_firmware_mode(i2c_ccs811_handle_t ccs811_handle, i2c_ccs811_firmware_modes_t *const mode);

/**
 * @brief Reads data ready status from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[out] ready CCS811 data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_data_status(i2c_ccs811_handle_t ccs811_handle, bool *const ready);

/**
 * @brief Reads error status from CCS811.
 * 
 * @param[in] ccs811_handle CCS811 device handle.
 * @param[out] error CCS811 error is present when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_get_error_status(i2c_ccs811_handle_t ccs811_handle, bool *const error);

/**
 * @brief Issues soft-reset to CCS811.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_reset(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Wakes the CCS811 through the configured wake GPIO pin number.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_io_wake(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Puts the CCS811 a sleep through the configured wake GPIO pin number.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_io_sleep(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Resets the CCS811 through the configured reset GPIO pin number.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_io_reset(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Removes a CCS811 device from master bus.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_remove(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Removes an CCS811 device from master I2C bus and delete the handle.
 * 
 * @param ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ccs811_delete(i2c_ccs811_handle_t ccs811_handle);

/**
 * @brief Decodes CCS811 device error to a textual message representation.
 * 
 * @param[in] code CCS811 I2C register error.
 * @return char textual representation of error message on success.
 */
const char *i2c_ccs811_err_to_message(const i2c_ccs811_error_code_register_t error_reg);

/**
 * @brief Decodes CCS811 device error to a textual code representation.
 * 
 * @param[in] code CCS811 I2C register error.
 * @return char textual representation of error code on success.
 */
const char *i2c_ccs811_err_to_code(const i2c_ccs811_error_code_register_t error_reg);

/**
 * @brief Decodes enumeratored drive mode to a textual descritive representation.
 * 
 * @param[in] mode measurement drive mode.
 * @return char textual representation of measure mode on success.
 */
const char *i2c_ccs811_measure_mode_description(const i2c_ccs811_drive_modes_t mode);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CCS811_H__
