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
 * @file tlv493d.h
 * @defgroup drivers tlv493d
 * @{
 *
 * ESP-IDF driver for tlv493d sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TLV493D_H__
#define __TLV493D_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * TLV493D definitions
 */
#define I2C_TLV493D_SCL_SPEED_HZ          UINT32_C(100000) //!< tlv493d I2C default clock frequency (100KHz)

#define I2C_TLV493D_DEV_ADDR_LO           UINT8_C(0x1F) //!< tlv493d I2C address
#define I2C_TLV493D_DEV_ADDR_HI           UINT8_C(0x5E) //!< tlv493d I2C address


/*
 * TLV493D macro definitions
 */
#define I2C_TLV493D_CONFIG_DEFAULT {                                    \
    .dev_config.device_address  = I2C_TLV493D_DEV_ADDR_LO,              \
    .dev_config.scl_speed_hz    = I2C_TLV493D_SCL_SPEED_HZ,             \
    .parity_test_enabled        = true,                                 \
    .power_mode                 = I2C_TLV493D_LOW_POWER_MODE,   \
    .irq_pin_enabled            = true }


/**
 * @brief TLV493D I2C power modes enumerator.
 * 
 * Fast, Low Power, Low Power Period, Measurement Time
 */
typedef enum i2c_tlv_493d_power_modes_tag {
    I2C_TLV493D_POWER_DOWN_MODE,        /*!< 0, 0, 0, 1000 */
    I2C_TLV493D_FAST_MODE,              /*!< 1, 0, 0, 0 */
    I2C_TLV493D_LOW_POWER_MODE,         /*!< 0, 1, 1, 10 */
    I2C_TLV493D_ULTRA_LOW_POWER_MODE,   /*!< 0, 1, 0, 100 */
    I2C_TLV493D_MASTER_CONTROLLED_MODE  /*!< 1, 1, 1, 10 */
} i2c_tlv_493d_power_modes_t;

/**
 * @brief TLV493D I2C channel conversations enumerator.
 */
typedef enum i2c_tlv493d_channel_conversions_tag {
    I2C_TLV493D_CHANNEL_CONV_COMPLETED    = (0b00),
    I2C_TLV493D_CHANNEL_Y_CONV_ONGOING    = (0b01),
    I2C_TLV493D_CHANNEL_Z_CONV_ONGOING    = (0b10),
    I2C_TLV493D_CHANNEL_TEMP_CONV_ONGOING = (0b11),
} i2c_tlv493d_channel_conversions_t;


/**
 * @brief TLV493D I2C addresses for slaves enumerator.
 */
typedef enum i2c_tlv493d_i2c_addresses_tag {
    I2C_TLV493D_I2C_ADDRESS_00 = (0b00),
    I2C_TLV493D_I2C_ADDRESS_01 = (0b01),
    I2C_TLV493D_I2C_ADDRESS_10 = (0b10),
    I2C_TLV493D_I2C_ADDRESS_11 = (0b11),
} i2c_tlv493d_i2c_addresses_t;

/**
 * @brief TLV493D I2C low power periods enumarator.
 */
typedef enum i2c_tlv493d_low_power_periods_tag {
    I2C_TLV493D_LOW_POWER_PERIOD_100MS = (0b0),
    I2C_TLV493D_LOW_POWER_PERIOD_12MS  = (0b1),
} i2c_tlv493d_low_power_periods_t;


typedef union __attribute__((packed)) {
    struct {
        i2c_tlv493d_channel_conversions_t channel:2;          /*!< channel conversion status           (bit:0-1)  */
        uint8_t frame_counter:2;    /*!< frame counter     (bit:2-3) */
        uint8_t temperature_msb:4;  /*!< temperature msb   (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_temperature_msb_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t by_lsb:4;  /*!< by lsb         (bit:0-3)  */
        uint8_t bx_lsb:4;  /*!< bx lsb         (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_bx_by_lsb_register_t;

typedef union __attribute__((packed)) {
    struct {
        uint8_t bz_lsb:4;          /*!< bz lsb           (bit:0-3)  */
        bool power_down_flag:1;    /*!< power-down flag, bx, by, bz and temperature conversion completed when true     (bit:4) */
        bool parity_fuse_flag:1; /*!< parity fuse flag, fuse setup is okay with true (PT bit must be enabled MOD2)     (bit:5) */
        bool test_mode_flag:1;  /*!< test-mode flag, when false the data is valid   (bit:6) */
        uint8_t reserved:1;     /*!< reserved, do not modify   (bit:7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_bz_lsb_register_t;

/**
 * @brief read only 0x07
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:3;          /*!< reserved           (bit:0-2)  */
        uint8_t factory_setting:2; /*!< factory setting - device spefici (bit:3-4)  */
        uint8_t reserved2:3;  /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_factorysetting1_register_t;

/**
 * @brief read only 0x08
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:8; /*!< factory setting - device spefici (bit:0-7)  */
    } bits;
    uint8_t reg;
} i2c_tlv493d_factorysetting2_register_t;

/**
 * @brief read only 0x09
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:5; /*!< factory setting - device spefici (bit:0-4)  */
        uint8_t reserved:3;  /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_factorysetting3_register_t;


/**
 * @brief write only 0x00
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:8;    /*!< factory setting - device specific  0x00 (reset)         (bit:0-7)  */
    } bits;
    uint8_t reg;
} i2c_tlv493d_reserved1_register_t;

/**
 * @brief write only 0x01
 */
typedef union __attribute__((packed)) {
    struct {
        bool low_power_mode_enabled:1;/*!< low power mode enabled when true           (bit:0)  */
        bool fast_mode_enabled:1;    /*!< fast mode enabled when true     (bit:1) */
        bool irq_pin_enabled:1; /*!<  interrupt pin assertion when true   (bit:2) */
        uint8_t factory_setting:2;  /*!< factory setting - device specific 0x07  (bit:3-4) */
        i2c_tlv493d_i2c_addresses_t i2c_slave_address:2;  /*!< defines slave address in bus configuration   (bit:5-6) */
        uint8_t parity:1;  /*!< parity of configuration map   (bit:7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_mode1_register_t;

/**
 * @brief write only 0x02
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:8;    /*!< factory setting - device specific 0x08          (bit:0-7)  */
    } bits;
    uint8_t reg;
} i2c_tlv493d_reserved2_register_t;

/**
 * @brief write only 0x03
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t factory_setting:5;    /*!< factory setting - device specific 0x09           (bit:0-4)  */
        bool parity_test_enabled:1;    /*!< parity test enabled when true     (bit:5) */
        i2c_tlv493d_low_power_periods_t low_power_period:1; /*!<  low power period, 12ms or 100ms   (bit:6) */
        bool temperature_disabled:1;  /*!<  temperature measurement is disabled when true  (bit:7) */
    } bits;
    uint8_t reg;
} i2c_tlv493d_mode2_register_t;

typedef struct {
    int16_t x_axis; /*!< 12-bit resolution x-axis read out */
    int16_t y_axis; /*!< 12-bit resolution y-axis read out */
    int16_t z_axis; /*!< 12-bit resolution z-axis read out */
    int16_t temperature; /*!< 12-bit resolution temperature read out */
    bool temperature_enabled;
} i2c_tlv493d_raw_data_t;

typedef struct {
    float x_axis;       /*!< x-axis magnetic in mT (+/-130 mT) */
    float y_axis;       /*!< y-axis magnetic in mT (+/-130 mT) */
    float z_axis;       /*!< z-axis magnetic in mT (+/-130 mT) */
    float temperature;  /*!< temperature in degrees celsius */
    bool temperature_enabled;
} i2c_tlv493d_data_t;

 
/**
 * @brief TLV493D I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t dev_config; /*!< I2C configuration for tlv493d device */
    bool parity_test_enabled;       /*!< tlv493d parity test enabled when true */
    bool temperature_disabled;      /*!< tlv493d temperature sensor disabled when true */
    i2c_tlv_493d_power_modes_t power_mode;  /*!< tlv493d power mode */
    bool irq_pin_enabled;           /*!< tlv493d interrupt pin enabled when true */
} i2c_tlv493d_config_t;

/**
 * @brief TLV493D I2C device structure.
 */
struct i2c_tlv493d_t {
    i2c_master_dev_handle_t     i2c_dev_handle; /*!< I2C device handle */
    i2c_tlv493d_factorysetting1_register_t factorysetting1_reg;
    i2c_tlv493d_factorysetting2_register_t factorysetting2_reg;
    i2c_tlv493d_factorysetting3_register_t factorysetting3_reg;
    bool parity_test_enabled;       /*!< tlv493d parity test enabled when true */
    bool temperature_disabled;      /*!< tlv493d temperature sensor disabled when true */
    i2c_tlv_493d_power_modes_t power_mode;  /*!< tlv493d power mode */
    bool irq_pin_enabled;           /*!< tlv493d interrupt pin enabled when true */
};

/**
 * @brief TLV493D I2C device definition.
 */
typedef struct i2c_tlv493d_t i2c_tlv493d_t;

/**
 * @brief TLV493D I2C device handle definition.
 */
typedef struct i2c_tlv493d_t *i2c_tlv493d_handle_t;


/**
 * @brief Initializes an TLV493D device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] tlv493d_config configuration of TLV493D device.
 * @param[out] tlv493d_handle TLV493D device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_tlv493d_init(i2c_master_bus_handle_t bus_handle, const i2c_tlv493d_config_t *tlv493d_config, i2c_tlv493d_handle_t *tlv493d_handle);

esp_err_t i2c_tlv493d_get_data(i2c_tlv493d_handle_t tlv493d_handle, i2c_tlv493d_data_t *const data);

esp_err_t i2c_tlv493d_get_data_status(i2c_tlv493d_handle_t tlv493d_handle, bool *const ready);

esp_err_t i2c_tlv493d_power_down(i2c_tlv493d_handle_t tlv493d_handle);

esp_err_t i2c_tlv493d_power_up(i2c_tlv493d_handle_t tlv493d_handle);


/**
 * @brief Issues soft-reset and initializes TLV493D.  See datasheet for details.
 *
 * @param tlv493d_handle TLV493D device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_tlv493d_reset(i2c_tlv493d_handle_t tlv493d_handle);

/**
 * @brief Removes an TLV493D device from master bus.
 *
 * @param[in] tlv493d_handle TLV493D device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_tlv493d_remove(i2c_tlv493d_handle_t tlv493d_handle);

/**
 * @brief Removes an TLV493D device from master bus and frees handle.
 * 
 * @param tlv493d_handle TLV493D device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_tlv493d_delete(i2c_tlv493d_handle_t tlv493d_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __TLD493D_H__
