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
 * @file bme680.h
 * @defgroup drivers bme680
 * @{
 * 
 * https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c
 *
 * ESP-IDF driver for bme680 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BME680_H__
#define __BME680_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BME680 definitions
*/
#define I2C_BME680_SCL_SPEED_HZ     UINT32_C(100000)   //!< bme680 I2C default clock frequency (100KHz)

/*
 * supported device addresses
*/
#define I2C_BME680_DEV_ADDR_LO      UINT8_C(0x76) //!< bme680 I2C address when ADDR pin floating/low
#define I2C_BME680_DEV_ADDR_HI      UINT8_C(0x77) //!< bme680 I2C address when ADDR pin high

/*
 * BME680 macros
*/
#define I2C_BME680_CONFIG_DEFAULT {                                              \
        .dev_config.device_address  = I2C_BME680_DEV_ADDR_HI,                    \
        .dev_config.scl_speed_hz    = I2C_BME680_SCL_SPEED_HZ,                   \
        .power_mode                 = I2C_BME680_POWER_MODE_NORMAL,              \
        .iir_filter                 = I2C_BME680_IIR_FILTER_OFF,                 \
        .pressure_oversampling      = I2C_BME680_PRESSURE_OVERSAMPLING_4X,       \
        .temperature_oversampling   = I2C_BME680_TEMPERATURE_OVERSAMPLING_4X,    \
        .humidity_oversampling      = I2C_BME680_HUMIDITY_OVERSAMPLING_4X }

/*
 * BME680 enumerator and sructure declerations
*/


/**
 * @brief BME680 I2C heater set-points enumerator.
 */
typedef enum {
    I2C_BME680_HEATER_SETPOINT_0  = (0b0000),
    I2C_BME680_HEATER_SETPOINT_1  = (0b0001),
    I2C_BME680_HEATER_SETPOINT_2  = (0b0010),
    I2C_BME680_HEATER_SETPOINT_3  = (0b0011),
    I2C_BME680_HEATER_SETPOINT_4  = (0b0100),
    I2C_BME680_HEATER_SETPOINT_5  = (0b0101),
    I2C_BME680_HEATER_SETPOINT_6  = (0b0110),
    I2C_BME680_HEATER_SETPOINT_7  = (0b0111),
    I2C_BME680_HEATER_SETPOINT_8  = (0b1000),
    I2C_BME680_HEATER_SETPOINT_9  = (0b1001)
} i2c_bme680_heater_setpoints_t;


/**
 * @brief BME680 I2C gas wait multipliers enumerator.
 */
typedef enum {
    I2C_BME680_GAS_WAIT_MULT_1  = (0b00),
    I2C_BME680_GAS_WAIT_MULT_4  = (0b01),
    I2C_BME680_GAS_WAIT_MULT_16 = (0b10),
    I2C_BME680_GAS_WAIT_MULT_64 = (0b11)
} i2c_bme680_gas_wait_multipliers_t;

/**
 * @brief BME680 I2C IIR filters coefficient enumerator.
 */
typedef enum {
    I2C_BME680_IIR_FILTER_OFF = (0b000),
    I2C_BME680_IIR_FILTER_1   = (0b001),
    I2C_BME680_IIR_FILTER_3   = (0b010),
    I2C_BME680_IIR_FILTER_7   = (0b011),
    I2C_BME680_IIR_FILTER_15  = (0b100),
    I2C_BME680_IIR_FILTER_31  = (0b101),
    I2C_BME680_IIR_FILTER_63  = (0b110),
    I2C_BME680_IIR_FILTER_127 = (0b111),
} i2c_bme680_iir_filters_t;

/**
 * @brief BME680 I2C power modes enumerator.
 */
typedef enum {
    I2C_BME680_POWER_MODE_SLEEP   = (0b00), //!< sleep mode, default after power-up
    I2C_BME680_POWER_MODE_FORCED  = (0b01), //!< measurement is initiated by user
} i2c_bme680_power_modes_t;

/**
 * @brief BME680 I2C pressure oversampling enumerator.
 */
typedef enum {
    I2C_BME680_PRESSURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BME680_PRESSURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    I2C_BME680_PRESSURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    I2C_BME680_PRESSURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    I2C_BME680_PRESSURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    I2C_BME680_PRESSURE_OVERSAMPLING_16X        = (0b101)   //!< ultra high resolution
} i2c_bme680_pressure_oversampling_t;

/**
 * @brief BME680 I2C temperature oversampling enumerator.
 */
typedef enum {
    I2C_BME680_TEMPERATURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BME680_TEMPERATURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    I2C_BME680_TEMPERATURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    I2C_BME680_TEMPERATURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    I2C_BME680_TEMPERATURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    I2C_BME680_TEMPERATURE_OVERSAMPLING_16X        = (0b101),  //!< ultra high resolution
} i2c_bme680_temperature_oversampling_t;

/**
 * @brief BME680 I2C humidity oversampling enumerator.
 */
typedef enum {
    I2C_BME680_HUMIDITY_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BME680_HUMIDITY_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    I2C_BME680_HUMIDITY_OVERSAMPLING_2X         = (0b010),  //!< low power
    I2C_BME680_HUMIDITY_OVERSAMPLING_4X         = (0b011),  //!< standard
    I2C_BME680_HUMIDITY_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    I2C_BME680_HUMIDITY_OVERSAMPLING_16X        = (0b101),  //!< ultra high resolution
} i2c_bme680_humidity_oversampling_t;

/**
 * @brief BME680 I2C status 0 register (0x1d) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t gas_measurement_index:4; /*!< bme680 user can program sequence of up to 10 conversions by setting nb_conv<3:0> (bit:0-3) */
        uint8_t reserved:1;              /*!< reserved (bit:4) */
        bool    measuring:1;             /*!< bmp680 automatically set to 1 whenever a conversion is running and back to 0 when results transferred to data registers (bit:5) */
        bool    gas_measuring:1;         /*!< bme680 automatically set to 1 during gas measurement and back to 0 when results transferred to data registers (bit:6) */
        bool    new_data:1;              /*!< bme680 measured data are stored into the data registers (bit:7) */
    } bits;
    uint8_t reg;
} i2c_bme680_status0_register_t;

/**
 * @brief BME680 I2C control measurement register (0x74) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_bme680_power_modes_t                power_mode:2;               /*!< bme680 power mode of the device            (bit:0-1)  */
        i2c_bme680_pressure_oversampling_t      pressure_oversampling:3;    /*!< bme680 oversampling of pressure data       (bit:2-4) */
        i2c_bme680_temperature_oversampling_t   temperature_oversampling:3; /*!< bme680 oversampling of temperature data    (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_bme680_control_measurement_register_t;

/**
 * @brief BME680 I2C control measurement register (0x72) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_bme680_humidity_oversampling_t      humidity_oversampling:3;    /*!< bme680 oversampling of humidity data          (bit:0-2)  */
        uint8_t                                 reserved1:3;                /*!< bme680 reserved                               (bit:3-5) */
        bool                                    spi_irq_enabled:1;          /*!< bme680 3-wire SPI interrupt enabled when true (bit:6)  */
        uint8_t                                 reserved2:1;                /*!< bme680 reserved                               (bit:7) */
    } bits;
    uint8_t reg;
} i2c_bme680_control_humidity_register_t;

/**
 * @brief BME680 I2C control gas 0 register (0x71) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_bme680_heater_setpoints_t           heater_setpoint:4;   /*!< bme680           (bit:0-3)  */
        bool                                    run_gas:1;           /*!< bme680 gas conversions are started only appropriate mode when true (bit:4)  */
        uint8_t                                 reserved:3;          /*!< bme680 reserved                               (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_bme680_control_gas0_register_t;

/**
 * @brief BME680 I2C control gas 1 register (0x70) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t                                 reserved1:3;    /*!< bme680           (bit:0-2)  */
        bool                                    heater_off:1;   /*!< bme680 heater is off when true (bit:3)  */
        uint8_t                                 reserved2:4;    /*!< bme680 reserved   (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_bme680_control_gas1_register_t;

/**
 * @brief BME680 I2C configuration register (0x75) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        bool                        spi_enabled:1;  /*!< bme680 3-wire SPI interface enabled when true  (bit:0)  */
        uint8_t                     reserved1:1;    /*!< bme680 reserved                                (bit:1) */
        i2c_bme680_iir_filters_t    iir_filter:3;   /*!< bme680 time constant of the IIR filter         (bit:2-4) */
        uint8_t                     reserved2:3;    /*!< bme680 reserved                                (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_bme680_configuration_register_t;



/**
 * @brief BME680 calibration factors structure.
 */
typedef struct {
    /* temperature compensation */
    uint16_t                par_T1;
    int16_t                 par_T2;
    int8_t                  par_T3;
    float                   temperature_fine;
    /* humidity compensation */
    uint16_t                par_H1;
    uint16_t                par_H2;
    int8_t                  par_H3;
    int8_t                  par_H4;
    int8_t                  par_H5;
    uint8_t                 par_H6;
    int8_t                  par_H7;
    /* pressure compensation */
    uint16_t                par_P1;
    int16_t                 par_P2;
    int8_t                  par_P3;
    int16_t                 par_P4;
    int16_t                 par_P5;
    int8_t                  par_P6;
    int8_t                  par_P7;
    int16_t                 par_P8;
    int16_t                 par_P9;
    uint8_t                 par_P10;
    /* resistance heat compensation */
    int8_t                  par_G1;
    int16_t                 par_G2;
    int8_t                  par_G3;
    uint8_t                 res_heat_range;
    int8_t                  res_heat_val;
    /* gas resistance compensation */
    int8_t                  gas_range;
    int8_t                  range_switching_error;
} i2c_bme680_cal_factors_t;

/**
 * @brief BME680 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t                         dev_config;                 /*!< I2C configuration for bmp280 device */
    i2c_bme680_power_modes_t                    power_mode;                 /*!< bme680 power mode */
    i2c_bme680_iir_filters_t                    iir_filter;
    i2c_bme680_pressure_oversampling_t          pressure_oversampling;
    i2c_bme680_temperature_oversampling_t       temperature_oversampling;
    i2c_bme680_humidity_oversampling_t          humidity_oversampling;
    bool                                        gas_enabled;                /*!< bme680 enable gas measurement */
    uint16_t                                    heater_temperature;         /*!< bme680 heater temperature for forced mode in degrees celsius */
    uint16_t                                    heater_duration;            /*!< bme680 heating duration for forced mode in milli-seconds */
    uint16_t                                   *heater_temperature_profile; /*!< bme680 heater temperature profile in degrees celsius */
    uint16_t                                   *heater_duration_profile;    /*!< bme680 heating duration profile in milli-seconds */
    uint8_t                                     heater_profile_size;        /*!< bme680 size of the heating profile */
    uint16_t                                    heater_shared_duration;     /*!< bme680 heating duration for parallel mode in milli-seconds */
} i2c_bme680_config_t;

/**
 * @brief BME680 I2C device structure.
 */
struct i2c_bme680_t {
    i2c_master_dev_handle_t                     i2c_dev_handle;     /*!< I2C device handle */
    i2c_bme680_cal_factors_t                   *dev_cal_factors;    /*!< bme680 device calibration factors */
    i2c_bme680_status0_register_t               status0_reg;        /*!< bme680 status 0 register */
    i2c_bme680_control_measurement_register_t   ctrl_meas_reg;      /*!< bme680 control measurement register */
    i2c_bme680_control_humidity_register_t      ctrl_hum_reg;       /*!< bme680 control humidity register */
    i2c_bme680_control_gas0_register_t          ctrl_gas0_reg;      /*!< bme680 control gas 0 register */
    i2c_bme680_control_gas1_register_t          ctrl_gas1_reg;      /*!< bme680 control gas 1 register */
    i2c_bme680_configuration_register_t         config_reg;         /*!< bme680 configuration register */
    uint8_t                                     chip_id;            /*!< bme680 chip identification register */
    uint16_t                                    ambient_temperature;
};

/**
 * @brief BME680 I2C device definition.
 */
typedef struct i2c_bme680_t i2c_bme680_t;

/**
 * @brief BME680 I2C device handle definition.
 */
typedef struct i2c_bme680_t *i2c_bme680_handle_t;



/**
 * @brief Reads chip indentification register from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_chip_id_register(i2c_bme680_handle_t bme680_handle);

/**
 * @brief Reads status register from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_status0_register(i2c_bme680_handle_t bme680_handle);

/**
 * @brief Reads control measurement register from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_control_measurement_register(i2c_bme680_handle_t bme680_handle);

/**
 * @brief Writes control measurement register to BME680. 
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] ctrl_meas_reg Control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_control_measurement_register(i2c_bme680_handle_t bme680_handle, const i2c_bme680_control_measurement_register_t ctrl_meas_reg);

/**
 * @brief Reads configuration register from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_configuration_register(i2c_bme680_handle_t bme680_handle);

/**
 * @brief Writes configuration register to BME680. 
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] config_reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_configuration_register(i2c_bme680_handle_t bme680_handle, const i2c_bme680_configuration_register_t config_reg);

/**
 * @brief Initializes an BME680 device onto the master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] bme680_config Configuration of BME680 device.
 * @param[out] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_init(i2c_master_bus_handle_t bus_handle, const i2c_bme680_config_t *bme680_config, i2c_bme680_handle_t *bme680_handle);

/**
 * @brief Reads temperature and pressure measurements from BME680
 *
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] pressure Pressure in pascal.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_measurements(i2c_bme680_handle_t bme680_handle, float *const temperature, float *const pressure);

/**
 * @brief Reads data status of the BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] ready Data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_data_status(i2c_bme680_handle_t bme680_handle, bool *const ready);

/**
 * @brief Reads power mode setting from the BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] power_mode BME680 power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_power_mode(i2c_bme680_handle_t bme680_handle, i2c_bme680_power_modes_t *const power_mode);

/**
 * @brief Writes power mode setting to BME680.  See datasheet, section 3.6, table 10.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] power_mode BME680 power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_power_mode(i2c_bme680_handle_t bme680_handle, const i2c_bme680_power_modes_t power_mode);

/**
 * @brief Reads pressure oversampling setting from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] oversampling BME680 pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_pressure_oversampling(i2c_bme680_handle_t bme680_handle, i2c_bme680_pressure_oversampling_t *const oversampling);

/**
 * @brief Writes pressure oversampling setting to BME680.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] oversampling BME680 pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_pressure_oversampling(i2c_bme680_handle_t bme680_handle, const i2c_bme680_pressure_oversampling_t oversampling);

/**
 * @brief Reads temperature oversampling setting from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] oversampling BME680 temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_temperature_oversampling(i2c_bme680_handle_t bme680_handle, i2c_bme680_temperature_oversampling_t *const oversampling);

/**
 * @brief Writes temperature oversampling setting to BME680.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] oversampling BME680 temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_temperature_oversampling(i2c_bme680_handle_t bme680_handle, const i2c_bme680_temperature_oversampling_t oversampling);

/**
 * @brief Reads humidity oversampling setting from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] oversampling BME680 humidity oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_humidity_oversampling(i2c_bme680_handle_t bme680_handle, i2c_bme680_humidity_oversampling_t *const oversampling);

/**
 * @brief Writes humidity oversampling setting to BME680.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] oversampling BME680 humidity oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_humidity_oversampling(i2c_bme680_handle_t bme680_handle, const i2c_bme680_humidity_oversampling_t oversampling);

/**
 * @brief Reads IIR filter setting from BME680.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[out] iir_filter BME680 IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_get_iir_filter(i2c_bme680_handle_t bme680_handle, i2c_bme680_iir_filters_t *const iir_filter);

/**
 * @brief Writes IIR filter setting to BME680.  See datasheet, section 3.4, table 7.
 * 
 * @param[in] bme680_handle BME680 device handle.
 * @param[in] iir_filter BME680 IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_set_iir_filter(i2c_bme680_handle_t bme680_handle, const i2c_bme680_iir_filters_t iir_filter);

/**
 * @brief Issues soft-reset sensor and initializes registers for BME680.
 *
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_reset(i2c_bme680_handle_t bme680_handle);

/**
 * @brief Removes an BME680 device from master bus.
 *
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_remove(i2c_bme680_handle_t bme680_handle);

/**
 * @brief Removes an BME680 device from master bus and frees handle.
 *
 * @param[in] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bme680_delete(i2c_bme680_handle_t bme680_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BME680_H__
