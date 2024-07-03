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
 * @file ens160.h
 * @defgroup drivers ens160
 * @{
 *
 * ESP-IDF driver for ens160 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __ENS160_H__
#define __ENS160_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ENS160 definitions
*/
#define I2C_ENS160_DATA_RATE_HZ             (100000)        //!< ens160 I2C default clock frequency (100KHz)

#define I2C_ENS160_RESET_DELAY_MS           (10)            //!< ens160 
#define I2C_ENS160_STD_MEASURE_DELAY_MS     (100)           //!< ens160 
#define I2C_ENS160_CLEAR_GPR_DELAY_MS       (2)             //!< ens160 
#define I2C_ENS160_DATA_READY_DELAY_MS      (1)             //!< ens160 1ms when checking data ready in a loop

#define I2C_ENS160_ADDR_LO                  UINT8_C(0x52)   //!< ens160 I2C address ADDR pin low
#define I2C_ENS160_ADDR_HI                  UINT8_C(0x53)   //!< ens160 I2C address ADDR pin high

#define I2C_ENS160_TVOC_MIN                 (0)         /*!< ens160 tvoc minimum in ppb (section 5.1) */
#define I2C_ENS160_TVOC_MAX                 (65000)     /*!< ens160 tvoc maximum in ppb (section 5.1) */
#define I2C_ENS160_ECO2_MIN                 (400)       /*!< ens160 equivalent co2 minimum in ppm (table 5) */
#define I2C_ENS160_ECO2_MAX                 (65000)     /*!< ens160 equivalent co2 maximum in ppm (table 5) */
#define I2C_ENS160_AQI_MIN                  (1)         /*!< ens160 air quality index UBA minimum (table 6) */
#define I2C_ENS160_AQI_MAX                  (5)         /*!< ens160 air quality index UBA maximum (table 6) */

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

#define I2C_ENS160_ERROR_MSG_SIZE          (80)   //!< ens160 I2C error message size
#define I2C_ENS160_ERROR_MSG_TABLE_SIZE    (7)    //!< ens160 I2C error message table size

/**
 * Parameter Range
 * TVOC     0..65,000 ppb (Total Volatile Organic Compounds)
 * eCO2    400..65,00 ppm (Equivalent Carbon Dioxide)
 * AQI-UBA   1..5         (UBA Air Quality Index)
 */

/*
 * ENS160 macro definitions
*/
#define I2C_ENS160_CONVERT_RS_RAW2OHMS_F(x) 	(pow (2, (float)(x) / 2048))


/**
 * @brief Macro that initializes `i2c_ens160_config_t` to default configuration settings.
 */
#define I2C_ENS160_CONFIG_DEFAULT {                                             \
        .dev_config.device_address  = I2C_ENS160_ADDR_HI,                       \
        .irq_enabled                = false,                                    \
        .irq_data_enabled           = false,                                    \
        .irq_gpr_enabled            = false,                                    \
        .irq_pin_driver             = I2C_ENS160_INT_PIN_DRIVE_OPEN_DRAIN,      \
        .irq_pin_polarity           = I2C_ENS160_INT_PIN_POLARITY_ACTIVE_LO }

/*
 * ENS160 enumerator and sructure declerations
*/

/**
 * @brief ENS160 air quality index of the uba enumerator.
 */
typedef enum {
    I2C_ENS160_AQI_UBA_INDEX_UNKNOWN    = 0, /*!< uba air quality index is unknown */
    I2C_ENS160_AQI_UBA_INDEX_1          = 1, /*!< uba air quality index of 1 is excellent */
    I2C_ENS160_AQI_UBA_INDEX_2          = 2, /*!< uba air quality index of 2 is good */
    I2C_ENS160_AQI_UBA_INDEX_3          = 3, /*!< uba air quality index of 3 is moderate */
    I2C_ENS160_AQI_UBA_INDEX_4          = 4, /*!< uba air quality index of 4 is poor */
    I2C_ENS160_AQI_UBA_INDEX_5          = 5  /*!< uba air quality index of 5 is unhealthy */
} i2c_ens160_aqi_uba_indexes_t;

/**
 * @brief ENS160 interrupt pin polarities enumerator.
 */
typedef enum {
    I2C_ENS160_INT_PIN_POLARITY_ACTIVE_LO  = 0, /*!< ens160 interrupt pin polarity active low (default) */
    I2C_ENS160_INT_PIN_POLARITY_ACTIVE_HI  = 1  /*!< ens160 interrupt pin polarity active high  */
} i2c_ens160_interrupt_pin_polarities_t;

/**
 * @brief ENS160 interrupt pin polarities enumerator.
 */
typedef enum {
    I2C_ENS160_INT_PIN_DRIVE_OPEN_DRAIN  = 0, /*!< ens160 interrupt pin drive open drain */
    I2C_ENS160_INT_PIN_DRIVE_PUSH_PULL   = 1, /*!< ens160 interrupt pin drive push/pull  */
} i2c_ens160_interrupt_pin_drivers_t;

/**
 * @brief ENS160 operating modes enumerator.
 */
typedef enum {
    I2C_ENS160_OPMODE_DEEP_SLEEP    = 0x00, /*!< ens160 deep sleep mode (low-power standby) */
    I2C_ENS160_OPMODE_IDLE          = 0x01, /*!< ens160 idle mode (low-power) (default) */
    I2C_ENS160_OPMODE_STANDARD      = 0x02, /*!< ens160 standard gas sensing mode */
    I2C_ENS160_OPMODE_RESET         = 0xf0  /*!< ens160 reset mode */
} i2c_ens160_operating_modes_t;

/**
 * @brief ENS160 commands enumerator.
 */
typedef enum {
    I2C_ENS160_COMMAND_NORMAL         = 0x00, /*!< ens160 normal operation command (default) */
    I2C_ENS160_COMMAND_GET_FW_APPVER  = 0x0e, /*!< ens160 get firmware version command */
    I2C_ENS160_COMMAND_CLEAR_GPR      = 0xcc  /*!< ens160 clear general purpose read registers command */
} i2c_ens160_commands_t;

/**
 * @brief ENS160 validity flags enumerator.
 */
typedef enum {
    I2C_ENS160_VALFLAG_NORMAL           = 0x00, /*!< ens160 normal operation validity flag */
    I2C_ENS160_VALFLAG_WARMUP           = 0x01, /*!< ens160 warm-up phase validity flag */
    I2C_ENS160_VALFLAG_INITIAL_STARTUP  = 0x02, /*!< ens160 initial start-up phase validity flag */
    I2C_ENS160_VALFLAG_INVALID_OUTPUT   = 0x03  /*!< ens160 invalid output validity flag */
} i2c_ens160_validity_flags_t;

/**
 * @brief ENS160 status register structure.
 */
typedef union __attribute__((packed)) {
    struct STS_REG_BIT_TAG {
        bool                        new_gpr_data:1;   /*!< true indicates new data is available in `GPR_READ` registers (bit:0)   */
        bool                        new_data:1;       /*!< true indicates new data is available in `DATA_x` registers   (bit:1)   */
        i2c_ens160_validity_flags_t state:2;          /*!< device status                                                (bit:2-3) */
        uint8_t                     reserved:2;       /*!< reserved and set 0                                           (bit:4-5) */
        bool                        error:1;          /*!< true indicates an error is detected                          (bit:6)   */
        bool                        mode:1;           /*!< true indicates an operating mode is running                  (bit:7)   */
    } bits;            /*!< represents the 8-bit status register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status register as `uint8_t`.   */
} i2c_ens160_status_register_t;

/**
 * @brief ENS160 interrupt configuration register structure.
 */
typedef union __attribute__((packed)) {
    struct CFG_REG_BIT_TAG {
        bool                        irq_enabled:1;       /*!< true indicates interrupt pin is enabled                       (bit:0)   */
        bool                        irq_data_enabled:1;  /*!< true indicates new data is available in `DATA_XXX` registers  (bit:1)   */
        uint8_t                     reserved1:1;         /*!< reserved and set to 0                                         (bit:2)   */
        bool                        irq_gpr_enabled:1;   /*!< true indicates new data is available in general purpose registers (bit:3) */
        uint8_t                     reserved2:1;         /*!< reserved and set to 0                                         (bit:4)   */
        i2c_ens160_interrupt_pin_drivers_t irq_pin_driver:1; /*!< interrupt pin driver configuration                        (bit:5)   */
        i2c_ens160_interrupt_pin_polarities_t irq_pin_polarity:1; /*!< interrupt pin polarity                               (bit:6)   */
        uint8_t                     reserved3:1;         /*!< reserved and set to 0                                         (bit:7)   */
    } bits;            /*!< represents the 8-bit interrupt configuration register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit interrupt configuration register as `uint8_t`.   */
} i2c_ens160_interrupt_config_register_t;

/**
 * @brief ENS160 application version register structure.
 */
typedef union {
    uint8_t major;      /*!< ens160 major firmware version */
    uint8_t minor;      /*!< ens160 minor firmware version */
    uint8_t release;    /*!< ens160 firmware version release */
    uint8_t bytes[3];   /*!< represents app version as a byte array. */
} i2c_ens160_app_version_t;

/**
 * @brief ENS160 calculated air quality index (aqi) data register structure.  See datasheet for AQI-UBA details.
 */
typedef union __attribute__((packed)) {
    struct CAL_AQI_REG_BITS_TAG {
        uint8_t          aqi_uba:3;     /*!< air quality index per uba[1..5] (default: 0x01)  (bit:0-2)  */
        uint8_t          reserved:5;    /*!< reserved and set to 0                            (bit:3-7)  */
    } bits;        /*!< represents the 8-bit calculated air quality index data register parts in bits.  */
    uint8_t value; /*!< represents the 8-bit calculated air quality index data register as `uint8_t`.   */
} i2c_ens160_caqi_data_register_t;

typedef struct {
    i2c_ens160_aqi_uba_indexes_t    uba_aqi;
    uint16_t                        tvoc;
    uint16_t                        eco2;
    float                           ri_res_0;
    float                           ri_res_1;
    float                           ri_res_2;
    float                           ri_res_3;
} i2c_ens160_air_quality_data_t;

/**
 * @brief ENS160 air quality index of the UBA row definition structure.
 */
typedef struct I2C_ENS160_AQI_UBA_ROW_TAG {
    i2c_ens160_aqi_uba_indexes_t index;           /*!< AQI-UBA index rating */
    char                    rating[20];           /*!< AQI-UBA rating category */
    char                    hygienic_rating[50];  /*!< AQI-UBA hygienic rating guidance */
    char                    recommendation[100];  /*!< AQI-UBA recommendation */
    char                    exposure_limit[20];   /*!< AQI-UBA exposure limit guidance */
} i2c_ens160_aqi_uba_row_t;

/**
 * @brief ENS160 I2C device structure definition.
 */
typedef struct i2c_ens160_t i2c_ens160_t;
/**
 * @brief ENS160 I2C device handle structure.
 */
typedef struct i2c_ens160_t *i2c_ens160_handle_t;

/**
 * @brief ENS160 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t                     dev_config;       /*!< I2C configuration for ens160 device */
    bool                                    irq_enabled;      /*!< true indicates interrupt pin is enabled  */
    bool                                    irq_data_enabled; /*!< true indicates new data is available in `DATA_XXX` registers   */
    bool                                    irq_gpr_enabled;  /*!< true indicates new data is available in general purpose registers  */
    i2c_ens160_interrupt_pin_drivers_t      irq_pin_driver;   /*!< interrupt pin driver configuration   */
    i2c_ens160_interrupt_pin_polarities_t   irq_pin_polarity; /*!< interrupt pin polarity   */
} i2c_ens160_config_t;

/**
 * @brief ENS160 I2C device parameters structure.
 */
typedef struct {
    bool                                    sleeping;           /*!< ens160 flag when true the device is in sleep mode */
    i2c_ens160_interrupt_config_register_t  int_config_reg;     /*!< ens160 interrupt configuration registyer */
    i2c_ens160_status_register_t            status_reg;         /*!< ens160 status register */
    uint16_t                                part_id;            /*!< ens160 part identifier */
    i2c_ens160_operating_modes_t            mode;               /*!< ens160 operating mode */
    i2c_ens160_commands_t                   command;            /*!< ens160 command */
    float                                   temperature_comp;   /*!< ens160 temperature compensation in degrees Celsius */
    float                                   humidity_comp;      /*!< ens160 humidity compensation in percentage */
} i2c_ens160_params_t;

/**
 * @brief ENS160 I2C device structure.
 */
struct i2c_ens160_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
    i2c_ens160_params_t       *dev_params;    /*!< ens160 device configuration parameters */
};

/**
 * @brief Reads interrupt configuration register from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_interrupt_config_register(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Writes interrupt configuration register to ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @param[in] int_config_reg ens100 interrupt configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_set_interrupt_config_register(i2c_ens160_handle_t ens160_handle, i2c_ens160_interrupt_config_register_t int_config_reg);

/**
 * @brief Reads status register from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_status_register(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Resets command to operate normal and clears general purpose registers on ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_clear_command_register(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Reads temperature and humidity compensation registers from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_compensation_registers(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Writes temperature and humidity compensation registers to ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @param[in] temperature temperature compensation in degree Celsius.
 * @param[in] humidity humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_set_compensation_registers(i2c_ens160_handle_t ens160_handle, float temperature, float humidity);

/**
 * @brief Reads part identifier from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_part_id(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Initializes an ENS160 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ens160_config configuration of ENS160 device.
 * @param[out] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_init(i2c_master_bus_handle_t bus_handle, const i2c_ens160_config_t *ens160_config, i2c_ens160_handle_t *ens160_handle);

/**
 * @brief Reads caculated air quality measurements from ENS160.
 *
 * 
 * @param[in] ens160_handle ENS160 device handle.
 * @param[out] index ens100 air quality index according to the UBA.
 * @param[out] tvoc ens160 calculated tvco in ppb.
 * @param[out] eco2 ens160 calculated equivalent co2 concentration in ppm.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_measurement(i2c_ens160_handle_t ens160_handle, i2c_ens160_air_quality_data_t *data);

/**
 * @brief Reads data ready status from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @param[out] ready ens100 data ready status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_data_status(i2c_ens160_handle_t ens160_handle, bool *ready);

/**
 * @brief Reads general purpose registers data ready status from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @param[out] ready ens100 general purpose registers data ready status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_gpr_data_status(i2c_ens160_handle_t ens160_handle, bool *ready);

/**
 * @brief Read error status from ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @param[out] error ens100 error status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_get_error_status(i2c_ens160_handle_t ens160_handle, bool *error);

/**
 * @brief Issues deep sleep mode to ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_deep_sleep(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Issues soft-reset mode to ENS160.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_reset(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Removes an ENS160 device from master I2C bus.
 *
 * @param[in] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ens160_rm(i2c_ens160_handle_t ens160_handle);

/**
 * @brief Decodes ENS160 air quality index to a uba definition row.
 * 
 * @param[in] index ENS160 air quality index of the uba.
 * @return i2c_ens160_aqi_uba_row_t air quality index of the uba definition row on success.
 */
const i2c_ens160_aqi_uba_row_t *i2c_ens160_aqi_index_to_definition(const i2c_ens160_aqi_uba_indexes_t index);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __ENS160_H__
