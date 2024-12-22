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
 * @file bmp390.h
 * @defgroup drivers bmp390
 * @{
 *
 * ESP-IDF driver for bmp390 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP390_H__
#define __BMP390_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMP390 definitions
*/
#define I2C_BMP390_SCL_SPEED_HZ     UINT32_C(100000)          //!< bmp390 I2C default clock frequency (100KHz)

/*
 * supported device addresses
*/
#define I2C_BMP390_DEV_ADDR_LO      0x76 //!< bmp390 I2C address when ADDR pin low
#define I2C_BMP390_DEV_ADDR_HI      0x77 //!< bmp390 I2C address when ADDR pin high

/*
 * BMP390 macros
*/
#define I2C_BMP390_CONFIG_DEFAULT {                                              \
        .dev_config.device_address  = I2C_BMP390_DEV_ADDR_HI,                    \
        .dev_config.scl_speed_hz    = I2C_BMP390_SCL_SPEED_HZ,                   \
        .power_mode                 = I2C_BMP390_POWER_MODE_NORMAL,              \
        .iir_filter                 = I2C_BMP390_IIR_FILTER_OFF,                 \
        .pressure_oversampling      = I2C_BMP390_PRESSURE_OVERSAMPLING_8X,       \
        .temperature_oversampling   = I2C_BMP390_TEMPERATURE_OVERSAMPLING_8X,    \
        .output_data_rate           = I2C_BMP390_ODR_40MS }

/*
 * BMP390 enumerator and sructure declerations
*/

/**
 * @brief BMP390 I2C IIR filters coefficient enumerator.
 */
typedef enum {
    I2C_BMP390_IIR_FILTER_OFF = (0b000),
    I2C_BMP390_IIR_FILTER_1   = (0b001),
    I2C_BMP390_IIR_FILTER_3   = (0b010),
    I2C_BMP390_IIR_FILTER_7   = (0b011),
    I2C_BMP390_IIR_FILTER_15  = (0b100),
	I2C_BMP390_IIR_FILTER_31  = (0b101),
	I2C_BMP390_IIR_FILTER_63  = (0b110),
	I2C_BMP390_IIR_FILTER_127 = (0b111)
} i2c_bmp390_iir_filters_t;

/**
 * @brief BMP390 I2C output data rates enumerator.
 */
typedef enum {
    I2C_BMP390_ODR_5MS    = (0x00),  //!< sampling period 5ms
    I2C_BMP390_ODR_10MS   = (0x01),  //!< sampling period 10ms
    I2C_BMP390_ODR_20MS   = (0x02),  //!< sampling period 20ms
    I2C_BMP390_ODR_40MS   = (0x03),  //!< sampling period 40ms
    I2C_BMP390_ODR_80MS   = (0x04),  //!< sampling period 80ms
    I2C_BMP390_ODR_160MS  = (0x05),  //!< sampling period 160ms
    I2C_BMP390_ODR_320MS  = (0x06),  //!< sampling period 320ms
    I2C_BMP390_ODR_640MS  = (0x07)   //!< sampling period 640ms
    // TODO: ODR 1.280s to 655.36s
} i2c_bmp390_output_data_rates_t;

/**
 * @brief BMP390 I2C power modes enumerator.
 */
typedef enum {
    I2C_BMP390_POWER_MODE_SLEEP   = (0b00), //!< sleep mode, default after power-up
    I2C_BMP390_POWER_MODE_FORCED  = (0b01), //!< measurement is initiated by user
    I2C_BMP390_POWER_MODE_FORCED1 = (0b10), //!< measurement is initiated by user
    I2C_BMP390_POWER_MODE_NORMAL  = (0b11)  //!< continuosly cycles between active measurement and inactive (standby-time) periods
} i2c_bmp390_power_modes_t;

/**
 * @brief BMP390 I2C pressure oversampling enumerator.
 */
typedef enum {
    I2C_BMP390_PRESSURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BMP390_PRESSURE_OVERSAMPLING_2X         = (0b001),  //!< ultra low power
    I2C_BMP390_PRESSURE_OVERSAMPLING_4X         = (0b010),  //!< low power
    I2C_BMP390_PRESSURE_OVERSAMPLING_8X         = (0b011),  //!< standard
    I2C_BMP390_PRESSURE_OVERSAMPLING_16X        = (0b100),  //!< high resolution
    I2C_BMP390_PRESSURE_OVERSAMPLING_32X        = (0b101)   //!< ultra high resolution
} i2c_bmp390_pressure_oversampling_t;

/**
 * @brief BMP390 I2C temperature oversampling enumerator.
 */
typedef enum {
    I2C_BMP390_TEMPERATURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    I2C_BMP390_TEMPERATURE_OVERSAMPLING_2X         = (0b001),  //!< ultra low power
    I2C_BMP390_TEMPERATURE_OVERSAMPLING_4X         = (0b010),  //!< low power
    I2C_BMP390_TEMPERATURE_OVERSAMPLING_8X         = (0b011),  //!< standard
    I2C_BMP390_TEMPERATURE_OVERSAMPLING_16X        = (0b100),  //!< high resolution
    I2C_BMP390_TEMPERATURE_OVERSAMPLING_32X        = (0b101),  //!< ultra high resolution
} i2c_bmp390_temperature_oversampling_t;

/**
 * @brief BMP390 I2C status register (0x03) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:4;    			/*!< bmp390 reserved (bit:0-3) */
        bool    command_ready:1;			/*!< bmp390 command decoder is ready to accept a new command when true                      (bit:4) */
		bool    pressure_data_ready:1;		/*!< bmp390 pressure data ready when true and it is reset when data register is read out    (bit:5) */
		bool    temperature_data_ready:1;	/*!< bmp390 temperature data ready when true and it is reset when data register is read out (bit:6) */
        uint8_t reserved2:1;    			/*!< bmp390 reserved (bit:7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_status_register_t;

/**
 * @brief BMP390 I2C interrupt status register (0x11) structure.  The reset state is ? for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        bool    fifo_watermark_int:1;  /*!< bmp390 FIFO watermark interrupt, cleared after reading       (bit:0) */
		bool    fifo_full_int:1;	   /*!< bmp390 FIFO full interrupt, cleared after reading            (bit:1) */
        uint8_t reserved1:1;    	   /*!< bmp390 reserved (bit:2) */
		bool    data_ready_int:1;	   /*!< bmp390 data ready interrupt, cleared after reading           (bit:3) */
        uint8_t reserved2:4;    	   /*!< bmp390 reserved (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_interrupt_status_register_t;

/**
 * @brief BMP390 I2C interrupt control register (0x19) structure.  The reset state is 0x02 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        bool    int_output:1;       /*!< bmp390 open-drain (true) or push-pull (false)            (bit:0) */
		bool    int_level:1;	     /*!< bmp390 active-high (true) or active-low (false)         (bit:1) */
        bool    int_latch_enabled:1;         /*!< bmp390 latching of interrupt pin is enabled when true   (bit:2) */
        bool    int_fifo_watermark_enabled:1; /*!< bmp390         (bit:3) */
        bool    int_fifo_full_enabled:1;	/*!< bmp390     (bit:4) */
        bool    int_ds:1;	            /*!< bmp390 high (true) or low (false)    (bit:5) */
        bool    int_data_ready_enabled:1;	/*!<     (bit:6) */
        uint8_t reserved:1;    	   /*!< bmp390 reserved (bit:7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_interrupt_control_register_t;

/**
 * @brief BMP390 I2C power control register (0x1b) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
		bool						pressure_enabled:1;		/*!< bmp390 pressure sensor enabled when true     (bit:0) */
		bool						temperature_enabled:1;	/*!< bmp390 temperature sensor enabled when true  (bit:1) */
		uint8_t						reserved1:2;			/*!< bmp380 reserved                              (bit:2-3) */
        i2c_bmp390_power_modes_t	power_mode:2;           /*!< bmp390 power mode of the device              (bit:4-5)  */
        uint8_t						reserved2:2;    		/*!< bmp390 reserved                              (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_power_control_register_t;

/**
 * @brief BMP390 I2C OSR register (0x1c) structure.  The reset state is 0x02 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_bmp390_pressure_oversampling_t      pressure_oversampling:3;    /*!< bmp390 oversampling of pressure data       (bit:0-2) */
        i2c_bmp390_temperature_oversampling_t   temperature_oversampling:3; /*!< bmp390 oversampling of temperature data    (bit:3-5) */
        uint8_t						            reserved:2;    		        /*!< bmp390 reserved                            (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_oversampling_register_t;

/**
 * @brief BMP390 I2C ODR register (0x1d) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_bmp390_output_data_rates_t output_data_rate:5; /*!< bmp390 output data rate       (bit:0-4) */
        uint8_t						   reserved:3;    	   /*!< bmp390 reserved               (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_output_data_rate_register_t;

/**
 * @brief BMP390 I2C configuration register (0x1f) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t                     reserved1:1;    /*!< bmp390 reserved                                (bit:0) */
        i2c_bmp390_iir_filters_t    iir_filter:3;   /*!< bmp390 time constant of the IIR filter         (bit:1-3) */
        uint8_t                     reserved2:4;    /*!< bmp390 reserved                                (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_bmp390_configuration_register_t;


/**
 * @brief BMP390 temperature and pressure calibration factors structure.
 */
typedef struct {
    /* temperature and pressure compensation */
    uint16_t                dig_T1;
    uint16_t                dig_T2;
    int8_t                  dig_T3;
    int16_t                 dig_P1;
    int16_t                 dig_P2;
    int8_t                  dig_P3;
    int8_t                  dig_P4;
    uint16_t                dig_P5;
    uint16_t                dig_P6;
    int8_t                  dig_P7;
    int8_t                  dig_P8;
    int16_t                 dig_P9;
	int8_t                  dig_P10;
	int8_t                  dig_P11;
} i2c_bmp390_cal_factors_t;

/**
 * @brief BMP390 temperature and pressure converted calibration factors structure.
 */
typedef struct {
    /* temperature and pressure compensation */
    double                PAR_T1;
    double                PAR_T2;
    double                PAR_T3;
    double                PAR_P1;
    double                PAR_P2;
    double                PAR_P3;
    double                PAR_P4;
    double                PAR_P5;
    double                PAR_P6;
    double                PAR_P7;
    double                PAR_P8;
    double                PAR_P9;
	double                PAR_P10;
	double                PAR_P11;
    double                t_lin;
} i2c_bmp390_conv_cal_factors_t;

typedef struct {
    i2c_device_config_t                         dev_config;         /*!< I2C configuration for bmp390 device */
    i2c_bmp390_iir_filters_t                    iir_filter;
    i2c_bmp390_pressure_oversampling_t          pressure_oversampling;
    i2c_bmp390_temperature_oversampling_t       temperature_oversampling;
    i2c_bmp390_output_data_rates_t              output_data_rate;
    i2c_bmp390_power_modes_t	                power_mode;
} i2c_bmp390_config_t;

struct i2c_bmp390_t {
    i2c_master_dev_handle_t                     i2c_dev_handle;       /*!< I2C device handle */
    i2c_bmp390_cal_factors_t                   *dev_cal_factors;      /*!< bmp390 device calibration factors */
    i2c_bmp390_conv_cal_factors_t              *dev_conv_cal_factors; /*!< bmp390 device calibration factors converted to floating point numbers (section 8.4)*/
    uint8_t                                     dev_type;             /*!< device type, should be bmp390 */
    i2c_bmp390_status_register_t                status_reg;           /*!< bmp390 status register */
    i2c_bmp390_oversampling_register_t          oversampling_reg;     /*!< bmp390 oversampling register */
    i2c_bmp390_configuration_register_t         config_reg;           /*!< bmp390 configuration register */
    i2c_bmp390_output_data_rate_register_t      output_data_rate_reg; /*!< bmp390 output data rate register */
    i2c_bmp390_power_control_register_t         power_ctrl_reg;
    i2c_bmp390_interrupt_status_register_t      interrupt_status_reg;
    i2c_bmp390_interrupt_control_register_t     interrupt_ctrl_reg;
};

typedef struct i2c_bmp390_t i2c_bmp390_t;
typedef struct i2c_bmp390_t *i2c_bmp390_handle_t;



/**
 * @brief reads chip indentification register from bmp390.
 * 
 * @param bmp390_handle bmp390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_chip_id_register(i2c_bmp390_handle_t bmp390_handle);

/**
 * @brief reads status register from bmp390.
 * 
 * @param bmp390_handle bmp390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_status_register(i2c_bmp390_handle_t bmp390_handle);


esp_err_t i2c_bmp390_get_interrupt_status_register(i2c_bmp390_handle_t bmp390_handle);

esp_err_t i2c_bmp390_get_interrupt_control_register(i2c_bmp390_handle_t bmp390_handle);
esp_err_t i2c_bmp390_set_interrupt_control_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_interrupt_control_register_t interrupt_control_reg);


esp_err_t i2c_bmp390_get_power_control_register(i2c_bmp390_handle_t bmp390_handle);
esp_err_t i2c_bmp390_set_power_control_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_power_control_register_t power_control_reg);


esp_err_t i2c_bmp390_get_output_data_rate_register(i2c_bmp390_handle_t bmp390_handle);
esp_err_t i2c_bmp390_set_output_data_rate_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_output_data_rate_register_t output_data_rate_reg);

/**
 * @brief reads oversampling register from bmp390.
 * 
 * @param bmp390_handle bmp390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_oversampling_register(i2c_bmp390_handle_t bmp390_handle);

/**
 * @brief writes oversampling register to bmp390. 
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[in] oversampling_reg oversampling register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_oversampling_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_oversampling_register_t oversampling_reg);

/**
 * @brief reads configuration register from bmp390.
 * 
 * @param bmp390_handle bmp390 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_configuration_register(i2c_bmp390_handle_t bmp390_handle);

/**
 * @brief writes configuration register to bmp390. 
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[in] config_reg configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_configuration_register(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_configuration_register_t config_reg);

/**
 * @brief initializes an bmp390 device onto the master bus.
 *
 * @param[in] bus_handle I2C master bus handle
 * @param[in] bmp390_config configuration of bmp390 device
 * @param[out] bmp390_handle bmp390 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_init(i2c_master_bus_handle_t bus_handle, const i2c_bmp390_config_t *bmp390_config, i2c_bmp390_handle_t *bmp280_handle);

/**
 * @brief high-level measurement (temperature & pressure) function for bmp390
 *
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] temperature temperature in degree Celsius
 * @param[out] pressure pressure in pascal
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_measurements(i2c_bmp390_handle_t bmp390_handle, float *const temperature, float *const pressure);

/**
 * @brief reads status of the bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] temperature_ready temperature data is ready when asserted to true.
 * @param[out] pressure_ready pressure data is ready when asserted to true.
 * @param[out] command_ready command is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_status(i2c_bmp390_handle_t bmp390_handle, bool *const temperature_ready, bool *const pressure_ready, bool *const command_ready);

/**
 * @brief reads data status of the bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] temperature_ready temperature data is ready when asserted to true.
 * @param[out] pressure_ready pressure data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_data_status(i2c_bmp390_handle_t bmp390_handle, bool *const temperature_ready, bool *const pressure_ready);

/**
 * @brief reads power mode setting from the bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] power_mode power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_power_mode(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_power_modes_t *const power_mode);

/**
 * @brief writes power mode setting to the bmp390.  See datasheet, section 3.6, table 10.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[in] power_mode power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_power_mode(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_power_modes_t power_mode);

/**
 * @brief reads pressure oversampling setting from bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] oversampling pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_pressure_oversampling(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_pressure_oversampling_t *const oversampling);

/**
 * @brief writes pressure oversampling setting to bmp390.  See datasheet, section 3.3.1, table 4.
 * 
 * @param bmp390_handle[in] bmp390 device handle.
 * @param oversampling[in] pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_pressure_oversampling(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_pressure_oversampling_t oversampling);

/**
 * @brief reads temperature oversampling setting from bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] oversampling temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_temperature_oversampling(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_temperature_oversampling_t *const oversampling);

/**
 * @brief writes temperature oversampling setting to bmp390.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[in] oversampling temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_temperature_oversampling(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_temperature_oversampling_t oversampling);

/**
 * @brief reads output data rate setting from bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @brief reads output data rate setting from bmp390.
 * @param[out] standby_time  setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_output_data_rate(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_output_data_rates_t *const output_data_rate);

/**
 * @brief writes output data rate setting to bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[in] standby_time output data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_output_data_rate(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_output_data_rates_t output_data_rate);

/**
 * @brief reads IIR filter setting to bmp390.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[out] iir_filter IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_get_iir_filter(i2c_bmp390_handle_t bmp390_handle, i2c_bmp390_iir_filters_t *const iir_filter);

/**
 * @brief writes IIR filter setting from bmp390.  See datasheet, section 3.4, table 7.
 * 
 * @param[in] bmp390_handle bmp390 device handle.
 * @param[in] iir_filter IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_set_iir_filter(i2c_bmp390_handle_t bmp390_handle, const i2c_bmp390_iir_filters_t iir_filter);

/**
 * @brief issues soft-reset sensor and initializes registers for bmp390.
 *
 * @param[in] bmp390_handle bmp390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_reset(i2c_bmp390_handle_t bmp280_handle);

/**
 * @brief removes an bmp390 device from master bus.
 *
 * @param[in] bmp390_handle bmp390 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_rm(i2c_bmp390_handle_t bmp390_handle);

/**
 * @brief Removes an bmp390 device from master I2C bus and delete the handle.
 * 
 * @param bmp390_handle bmp390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_bmp390_del(i2c_bmp390_handle_t bmp390_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP390_H__
