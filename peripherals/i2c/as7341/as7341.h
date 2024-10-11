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
 * @file as7341.h
 * @defgroup drivers as7341
 * @{
 *
 * ESP-IDF driver for as7341 11-channel spectrometer
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AS7341_H__
#define __AS7341_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AS7341 definitions
*/
#define I2C_AS7341_DATA_RATE_HZ          (100000)        //!< as7341 I2C default clock frequency (100KHz)

#define I2C_AS7341_ADDR                  UINT8_C(0x39)   //!< as7341 I2C address
#define I2C_AS7341_PART_ID               UINT8_C(0x09)   //!< as7341 I2C device part identifier

//#define I2C_AS7341_ASTATUS              (0x60)  //!< as7341 (see i2c_as7341_astatus_register_t)
//#define I2C_AS7341_CH0_ADC_DATA_L       (0x61)  //!< as7341
//#define I2C_AS7341_CH0_ADC_DATA_H       (0x62)  //!< as7341
#define I2C_AS7341_ITIME_L              (0x63)  //!< as7341
#define I2C_AS7341_ITIME_M              (0x64)  //!< as7341
#define I2C_AS7341_ITIME_H              (0x65)  //!< as7341
//#define I2C_AS7341_CH1_ADC_DATA_L       (0x66)  //!< as7341
//#define I2C_AS7341_CH1_ADC_DATA_H       (0x67)  //!< as7341
//#define I2C_AS7341_CH2_ADC_DATA_L       (0x68)  //!< as7341
//#define I2C_AS7341_CH2_ADC_DATA_H       (0x69)  //!< as7341
//#define I2C_AS7341_CH3_ADC_DATA_L       (0x6a)  //!< as7341
//#define I2C_AS7341_CH3_ADC_DATA_H       (0x6b)  //!< as7341
//#define I2C_AS7341_CH4_ADC_DATA_L       (0x6c)  //!< as7341
//#define I2C_AS7341_CH4_ADC_DATA_H       (0x6d)  //!< as7341
//#define I2C_AS7341_CH5_ADC_DATA_L       (0x6e)  //!< as7341
//#define I2C_AS7341_CH5_ADC_DATA_H       (0x6f)  //!< as7341

#define I2C_AS7341_CONFIG               (0x70)  //!< as7341 (see i2c_as7341_config_register_t)
#define I2C_AS7341_DEV_STATUS           (0x71)  //!< as7341 (see i2c_as7341_device_status_register_t)
#define I2C_AS7341_EDGE                 (0x72)  //!< as7341
#define I2C_AS7341_GPIO1                (0x73)  //!< as7341 (see i2c_as7341_gpio1_register_t)
#define I2C_AS7341_LED                  (0x74)  //!< as7341 (see i2c_as7341_led_register_t)
#define I2C_AS7341_ENABLE               (0x80)  //!< as7341 (see i2c_as7341_enable_register_t)
#define I2C_AS7341_ATIME                (0x81)  //!< as7341
#define I2C_AS7341_WTIME                (0x83)  //!< as7341
#define I2C_AS7341_SP_TH_L_LSB          (0x84)  //!< as7341
#define I2C_AS7341_SP_TH_L_MSB          (0x85)  //!< as7341
#define I2C_AS7341_SP_TH_H_LSB          (0x86)  //!< as7341
#define I2C_AS7341_SP_TH_H_MSB          (0x87)  //!< as7341
#define I2C_AS7341_AUXID                (0x90)  //!< as7341
#define I2C_AS7341_REVID                (0x91)  //!< as7341
#define I2C_AS7341_ID                   (0x92)  //!< as7341

#define I2C_AS7341_INT_STATUS           (0x93)  //!< as7341 (see i2c_as7341_interrupt_status_register_t)

#define I2C_AS7341_ASTATUS              (0x94)  //!< as7341 (see i2c_as7341_astatus_register_t)
#define I2C_AS7341_CH0_ADC_DATA_L       (0x95)  //!< as7341
#define I2C_AS7341_CH0_ADC_DATA_H       (0x96)  //!< as7341
#define I2C_AS7341_CH1_ADC_DATA_L       (0x97)  //!< as7341
#define I2C_AS7341_CH1_ADC_DATA_H       (0x98)  //!< as7341
#define I2C_AS7341_CH2_ADC_DATA_L       (0x99)  //!< as7341
#define I2C_AS7341_CH2_ADC_DATA_H       (0x9a)  //!< as7341
#define I2C_AS7341_CH3_ADC_DATA_L       (0x9b)  //!< as7341
#define I2C_AS7341_CH3_ADC_DATA_H       (0x9c)  //!< as7341
#define I2C_AS7341_CH4_ADC_DATA_L       (0x9d)  //!< as7341
#define I2C_AS7341_CH4_ADC_DATA_H       (0x9e)  //!< as7341
#define I2C_AS7341_CH5_ADC_DATA_L       (0x9f)  //!< as7341
#define I2C_AS7341_CH5_ADC_DATA_H       (0xa0)  //!< as7341

#define I2C_AS7341_STATUS2              (0xa3)  //!< as7341 (see i2c_as7341_status2_register_t)
#define I2C_AS7341_STATUS3              (0xa4)  //!< as7341 (see i2c_as7341_status3_register_t)
#define I2C_AS7341_STATUS5              (0xa6)  //!< as7341 (see i2c_as7341_status5_register_t)
#define I2C_AS7341_STATUS6              (0xa7)  //!< as7341 (see i2c_as7341_status6_register_t)

#define I2C_AS7341_CONFIG0              (0xa9)  //!< as7341 (see i2c_as7341_config0_register_t)
#define I2C_AS7341_CONFIG1              (0xaa)  //!< as7341 (see i2c_as7341_config1_register_t)
#define I2C_AS7341_CONFIG3              (0xac)  //!< as7341 (see i2c_as7341_config3_register_t)
#define I2C_AS7341_CONFIG6              (0xaf)  //!< as7341 (see i2c_as7341_config6_register_t)
#define I2C_AS7341_CONFIG8              (0xb1)  //!< as7341 (see i2c_as7341_config8_register_t)
#define I2C_AS7341_CONFIG9              (0xb2)  //!< as7341 (see i2c_as7341_config9_register_t)
#define I2C_AS7341_CONFIG10             (0xb3)  //!< as7341 (see i2c_as7341_config10_register_t)
#define I2C_AS7341_CONFIG12             (0xb5)  //!< as7341 (see i2c_as7341_config12_register_t)

#define I2C_AS7341_PERS                 (0xbd)  //!< as7341 (see i2c_as7341_pers_register_t)
#define I2C_AS7341_GPIO2                (0xbe)  //!< as7341 (see i2c_as7341_gpio2_register_t)
#define I2C_AS7341_ASTEP_L              (0xca)  //!< as7341
#define I2C_AS7341_ASTEP_H              (0xcb)  //!< as7341
#define I2C_AS7341_AGC_GAIN_MAX         (0xcf)  //!< as7341 (see i2c_as7341_agc_gain_register_t)
#define I2C_AS7341_AZ_CONFIG            (0xd6)  //!< as7341
#define I2C_AS7341_FD_TIME1             (0xd8)  //!< as7341
#define I2C_AS7341_FD_TIME2             (0xda)  //!< as7341 (see i2c_as7341_fd_time2_register_t)
#define I2C_AS7341_FD_CONFIG0           (0xd7)  //!< as7341 (see i2c_as7341_fd_config0_register_t)
#define I2C_AS7341_FD_STATUS            (0xdb)  //!< as7341 (see i2c_as7341_fd_status_register_t)
#define I2C_AS7341_INTENAB              (0xf9)  //!< as7341 (see i2c_as7341_interrupt_enable_register_t)
#define I2C_AS7341_CONTROL              (0xfa)  //!< as7341 (see i2c_as7341_control_register_t)


#define I2C_AS7341_DATA_POLL_TIMEOUT_MS  (1000)
#define I2C_AS7341_DATA_READY_DELAY_MS   (1)
#define I2C_AS7341_POWERUP_DELAY_MS      (200)
#define I2C_AS7341_APPSTART_DELAY_MS     (200)
#define I2C_AS7341_RESET_DELAY_MS        (25)
#define I2C_AS7341_SETUP_DELAY_MS        (15)

/*
 * AS7341 macro definitions
*/
#define I2C_AS7341_CONFIG_DEFAULT { .dev_config.device_address = I2C_AS7341_ADDR }

/*
 * AS7341 enumerator and sructure declerations
*/

/**
 * @brief AS7341 I2C ambient light sensing mode enumerator.
 * 
 */
typedef enum {
    I2C_AS7341_ALS_SPM_MODE         = (0), /*!< as7341 spectral measurement, normal mode */
    I2C_AS7341_ALS_SYNS_MODE        = (1), /*!< as7341 SYNS mode */
    I2C_AS7341_ALS_RESERVED_MODE    = (2), /*!< as7341 reserved, do not use */
    I2C_AS7341_ALS_SYND_MODE        = (3)  /*!< as7341 SYND mode, use spectra data registers 0x60 to 0x6F in this mode */
} i2c_as7341_als_modes_t;

/**
 * @brief AS7341 I2C led driving strengths enumerator.
 * 
 */
typedef enum {
    I2C_AS7341_LED_DRIVE_STRENGTH_4MA         = (0b0000000), /*!< as7341  */
    I2C_AS7341_LED_DRIVE_STRENGTH_6MA         = (0b0000001), /*!< as7341  */
    I2C_AS7341_LED_DRIVE_STRENGTH_8MA         = (0b0000010), /*!< as7341  */
    I2C_AS7341_LED_DRIVE_STRENGTH_10MA        = (0b0000011), /*!< as7341  */
    I2C_AS7341_LED_DRIVE_STRENGTH_12MA        = (0b0000100), /*!< as7341 (default) */

    I2C_AS7341_LED_DRIVE_STRENGTH_256MA       = (0b1111110), /*!< as7341  */
    I2C_AS7341_LED_DRIVE_STRENGTH_258MA       = (0b1111111), /*!< as7341  */
} i2c_as7341_led_drive_strengths_t;

/**
 * @brief AS7341 I2C allowable gain multipliers enumerator.
 *
 */
typedef enum {
    I2C_AS7341_SPECTRAL_GAIN_0_5X,
    I2C_AS7341_SPECTRAL_GAIN_1X,
    I2C_AS7341_SPECTRAL_GAIN_2X,
    I2C_AS7341_SPECTRAL_GAIN_4X,
    I2C_AS7341_SPECTRAL_GAIN_8X,
    I2C_AS7341_SPECTRAL_GAIN_16X,
    I2C_AS7341_SPECTRAL_GAIN_32X,
    I2C_AS7341_SPECTRAL_GAIN_64X,
    I2C_AS7341_SPECTRAL_GAIN_128X,
    I2C_AS7341_SPECTRAL_GAIN_256X,
    I2C_AS7341_SPECTRAL_GAIN_512X,
} i2c_as7341_spectral_gains_t;

/**
 * @brief AS7341 I2C available SMUX commands enumerator.
 *
 */
typedef enum {
    I2C_AS7341_SMUX_CMD_ROM_RESET,  ///< ROM code initialization of SMUX
    I2C_AS7341_SMUX_CMD_READ,       ///< Read SMUX configuration to RAM from SMUX chain
    I2C_AS7341_SMUX_CMD_WRITE,      ///< Write SMUX configuration from RAM to SMUX chain
} i2c_as7341_smux_commands_t;

/**
 * @brief AS7341 I2C ADC channel specifiers enumerator.
 *
 */
typedef enum {
    I2C_AS7341_ADC_CHANNEL_0,
    I2C_AS7341_ADC_CHANNEL_1,
    I2C_AS7341_ADC_CHANNEL_2,
    I2C_AS7341_ADC_CHANNEL_3,
    I2C_AS7341_ADC_CHANNEL_4,
    I2C_AS7341_ADC_CHANNEL_5,
} i2c_as7341_adc_channels_t;

/**
 * @brief AS7341 I2C spectral channel specifiers enumerator.
 *
 */
typedef enum {
    I2C_AS7341_COLOR_CHANNEL_415NM_F1,
    I2C_AS7341_COLOR_CHANNEL_445NM_F2,
    I2C_AS7341_COLOR_CHANNEL_480NM_F3,
    I2C_AS7341_COLOR_CHANNEL_515NM_F4,
    I2C_AS7341_COLOR_CHANNEL_555NM_F5,
    I2C_AS7341_COLOR_CHANNEL_590NM_F6,
    I2C_AS7341_COLOR_CHANNEL_630NM_F7,
    I2C_AS7341_COLOR_CHANNEL_680NM_F8,
    I2C_AS7341_COLOR_CHANNEL_CLEAR,
    I2C_AS7341_COLOR_CHANNEL_NIR,
} i2c_as7341_color_channels_t;


/**
 * violet -> F1
 * indigo -> F2
 * blue   -> F3
 * cyan   -> F4
 * green  -> F5
 * yellow -> F6
 * orange -> F7
 * red    -> F8
 */


/**
 * @brief AS7341 I2C enable register (0x80) structure. 
 * 
 * @note To operate the device enable power and interrupts before enabling spectral measurements.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool    power_enabled:1;                /*!< as7341 power enabled when true                                                             (bit:0)  */
        bool    spectral_measurement_enabled:1; /*!< as7341 spectral measurement enabled when true                                              (bit:1)  */
        uint8_t reserved1:1;                    /*!< reserved                                                                                   (bit:2) */
        bool    wait_time_enabled:1;            /*!< as7341 wait time between two consecutive spectral measurements enabled when true           (bit:3) */
        bool    smux_enabled:1;                 /*!< as7341 starts SMUX command when true and bit is cleared when SMUX operation is finished    (bit:4) */
        uint8_t reserved2:1;                    /*!< reserved                                                                                   (bit:5) */
        bool    flicker_detection_enabled:1;    /*!< as7341 flicker detection enabled when true                                                 (bit:6)  */
        uint8_t reserved3:1;                    /*!< reserved                                                                                   (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_enable_register_t;

/**
 * @brief AS7341 I2C configuration register (0x70) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_as7341_als_modes_t  irq_mode:2;                 /*!< as7341 ambient light sensing mode                              (bit:0-1)  */
        bool                    irq_sync_signal_enabled:1;  /*!< as7341 sync signal applied on output pin interrupt when true   (bit:2) */
        bool                    led_ldr_control_enabled:1;  /*!< as7341 register LED controls LED connected to pin LDR when true (register 0x74), otherwise, exernal LED is not controlled by as7341 (bit:3) */
        uint8_t                 reserved:4;                 /*!< reserved                                                       (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_config_register_t;

/**
 * @brief AS7341 I2C gpio1 register (0x73) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool                    photo_diode_irq_enabled:1;  /*!< as7341 photo diode connected to interrupt pin when true (bit:0) */
        bool                    photo_diode_gpio_enabled:1; /*!< as7341 photo diode connected to GPIO pin when true      (bit:1) */
        uint8_t                 reserved:6;                 /*!< reserved                                                (bit:2-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_gpio1_register_t;

/**
 * @brief AS7341 I2C gpio2 register (0xbe) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool                    gpio_input_state:1;     /*!< as7341 GPIO pin is input when true if gpio input is enabled (bit:0) */
        bool                    gpio_output_state:1;    /*!< as7341 output state of GPIO pin is active when true         (bit:1) */
        bool                    gpio_input_enabled:1;   /*!< as7341 GPIO pin accepts non-floating input when true        (bit:2) */
        bool                    gpio_output_inverted:1; /*!< as7341 output state of GPIO pin is inverted when true       (bit:3) */
        uint8_t                 reserved:4;             /*!< reserved                                                    (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_gpio2_register_t;

/**
 * @brief AS7341 I2C led register (0x74) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_as7341_led_drive_strengths_t    led_drive_strength:7;   /*!< as7341 LED driving strength (bit:0-6) */
        bool                                led_ldr_enabled:1;      /*!< as7341 external LED connected to LDR pin is enabled when true (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_led_register_t;

/**
 * @brief AS7341 I2C interrupt enable register (0xf9) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool                    irq_system_enabled:1;           /*!< as7341 interrupt asserted when system interrupts occur when enabled (bit:0) */
        uint8_t                 reserved1:1;                    /*!< reserved         (bit:1) */
        bool                    irq_fifo_enabled:1;             /*!< as7341 interrupt asserted when FIFO LVL exceeds FIFO threshold condition when enabled (bit:2) */
        bool                    irq_spectral_enabled:1;         /*!< as7341 interrupt asserted subject to spectral threholds and persistence filter when enabled (bit:3) */
        uint8_t                 reserved2:3;                    /*!< reserved         (bit:4-6) */
        bool                    irq_spectral_flicker_enabled:1; /*!< as7341 spectral and flicker detection interrupt asserted when enabled (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_interrupt_enable_register_t;


/**
 * @brief AS7341 I2C interrupt status register (0x93) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool                    irq_system:1;           /*!< as7341 system interrupt asserted when true (bit:0) */
        bool                    irq_calibration:1;      /*!< as7341 calibration interrupt asserted when true         (bit:1) */
        bool                    irq_fifo:1;             /*!< as7341 interrupt asserted when FIFO LVL exceeds FIFO threshold condition when true (bit:2) */
        bool                    irq_spectral:1;         /*!< as7341 interrupt asserted subject to spectral threholds and persistence filter when true (bit:3) */
        uint8_t                 reserved:3;             /*!< reserved         (bit:4-6) */
        bool                    irq_spectral_flicker:1; /*!< as7341 spectral and flicker detection interrupt asserted when true (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_interrupt_status_register_t;


/**
 * @brief AS7341 I2C device status register (0x71) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool    data_ready:1;       /*!< as7341 spectral measurement status is ready when true (bit:0) */
        bool    gpio_wait_sync:1;   /*!< as7341 device waits for sync pulse on GPIO to start integration when true (bit:1) */
        uint8_t reserved:6;         /*!< reserved                       (bit:2-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_device_status_register_t;

/**
 * @brief AS7341 I2C astatus register (0x94) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t again_status:4; /*!< as7341 gain applied for the spectral data latched to this ASTATUS read (bit:3-0) */
        uint8_t reserved:3;     /*!< reserved                                                               (bit:4-6) */
        bool    asat_status:1;  /*!< as7341 latched data is affected by analog or digital saturation        (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_astatus_register_t;

/**
 * @brief AS7341 I2C status 2 register (0xa3) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool    flicker_detect_digital_saturation:1;    /*!< as7341 maximum counter value has been reached during flicker detection when true (bit:0) */
        bool    flicker_detect_analog_saturation:1;     /*!< as7341 intensity of ambient light ha exceeded the maximum integration level for analg circuit for flicker detection when true (bit:1) */
        uint8_t reserved1:1;                            /*!< reserved         (bit:2) */
        bool    analog_saturation:1;                    /*!< as7341 intensity of ambient light ha exceeded the maximum integration level for spectral analg circuit when true (bit:3) */
        bool    digital_saturation:1;                   /*!< as7341 maximum counter value has been reached when true, dependent of ATIME register (bit:4) */
        uint8_t reserved2:1;                            /*!< reserved         (bit:5) */
        bool    spectral_valid:1;                       /*!< as7341 spectral measurement has been completed when true (bit:6) */
        uint8_t reserved3:1;                            /*!< reserved         (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_status2_register_t;

/**
 * @brief AS7341 I2C status 3 register (0xa4) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:4;                    /*!< reserved                                                                   (bit:3-0) */
        bool    irq_spectral_threshold_lo:1;    /*!< as7341 spectral interrupt asserted when data is below the low threshold    (bit:4) */
        bool    irq_spectral_threshold_hi:1;    /*!< as7341 spectral interrupt asserted when data is above the high threshold   (bit:5) */
        uint8_t reserved2:2;                    /*!< reserved                                                                   (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_status3_register_t;

/**
 * @brief AS7341 I2C status 5 register (0xa6) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:2;                /*!< reserved                                                                           (bit:1-0) */
        bool    irq_smux_operation:1;       /*!< as7341 SMUX command execution has finished when true                               (bit:2) */
        bool    irq_flicker_detection:1;    /*!< as7341 FD_STATUS register status has changed when true and if SIEN_FD is enabled   (bit:3) */
        uint8_t reserved2:4;                /*!< reserved                                                                           (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_status5_register_t;


/**
 * @brief AS7341 I2C status 6 register (0xa7) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool    initalizing:1;                  /*!< as7341 device is initializing when true, wait until cleared                (bit:0) */
        bool    sleep_after_irq:1;              /*!< as7341 device is in sleep due to an interrupt when true                    (bit:1) */
        bool    spectral_trigger_error:1;       /*!< as7341 timing error when true, WTIME to short for ATIME                    (bit:2) */
        uint8_t reserved1:1;                    /*!< reserved                                                                   (bit:3) */
        bool    flicker_detect_trigger_error:1; /*!< as7341 timing error that prevents flicker detect from working correctly    (bit:4) */
        bool    over_temperature_detected:1;    /*!< as7341 device temperature is to high when true                             (bit:5) */
        uint8_t reserved2:1;                    /*!< reserved                                                                   (bit:6) */
        bool    fifo_buffer_overflow:1;         /*!< as7341 FIFO buffer overflowed and information is lost when true            (bit:7) */
    } bits;
    uint8_t reg;
} i2c_as7341_status6_register_t;

/**
 * @brief AS7341 I2C flicker detection status register (0xdb) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        bool    fd_100hz_flicker:1;         /*!< as7341 ambient light source is flickering at 100Hz when true                       (bit:0) */
        bool    fd_120hz_flicker:1;         /*!< as7341 ambient light source is flickering at 120Hz when true                       (bit:1) */
        bool    fd_100hz_flicker_valid:1;   /*!< as7341 100Hz flicker detection calculation is valid when true                      (bit:2) */
        bool    fd_120hz_flicker_valid:1;   /*!< as7341 120Hz flicker detection calculation is valid when true                      (bit:3) */
        bool    fd_saturation_detected:1;   /*!< as7341 saturation ocurred during the last flicker detection measurement when true  (bit:4) */
        bool    fd_measurement_value:1;     /*!< as7341 flicker detection measurement is complete when true                         (bit:5) */
        uint8_t reserved:2;                 /*!< reserved                                                                           (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_flicker_detection_status_register_t;




/**
 * @brief AS7341 I2C auxiliary identifier register (0x90) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t identifier:4;   /*!< as7341 auxiliary identification    (bit:0-3) */
        uint8_t reserved:4;     /*!< reserved                           (bit:4-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_auxiliary_id_register_t;

/**
 * @brief AS7341 I2C revision number identifier register (0x91) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t identifier:3;   /*!< as7341 revision number identification  (bit:0-2) */
        uint8_t reserved:5;     /*!< reserved                               (bit:3-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_revision_id_register_t;

/**
 * @brief AS7341 I2C part number identifier register (0x92) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved:2;     /*!< reserved                                   (bit:0-1) */
        uint8_t identifier:6;   /*!< as7341 device part number identification   (bit:2-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_part_id_register_t;


/**
 * @brief AS7341 I2C configuration 0 register (0xa9) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved1:2;        /*!< reserved                                                   (bit:1-0) */
        bool    trigger_long:1;     /*!< increase te WTIME setting by a factor of 16 when asserted  (bit:2) */
        uint8_t reserved2:1;        /*!< reserved                                                   (bit:3) */
        bool    reg_bank_access:1;  /*!< bit needs to be set to access registers 0x60 to 0x70       (bit:4) */
        bool    low_power:1;        /*!< device will run in a low power modem when asserted         (bit:5) */
        uint8_t reserved3:2;        /*!< reserved                                                   (bit:6-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_config0_register_t;

/**
 * @brief AS7341 I2C configuration 1 register (0xaa) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        i2c_as7341_spectral_gains_t spectral_gain:5;    /*!< spectral sensitivity  (bit:4-0) */
        uint8_t                     reserved:3;         /*!< reserved              (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_config1_register_t;

/**
 * @brief AS7341 I2C configuration 6 register (0xaf) structure.
 * 
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t                     reserved1:3;    /*!< reserved                                                   (bit:2-0) */
        i2c_as7341_smux_commands_t  smux_command:2; /*!< as7341 SMUX command to execute when smux enabled gets set  (bit:3-4) */
        uint8_t                     reserved2:3;    /*!< reserved                                                   (bit:5-7) */
    } bits;
    uint8_t reg;
} i2c_as7341_config6_register_t;



typedef struct {
    float f1_415nm;
    float f2_445nm;
    float f3_480nm;
    float f4_515nm;
    float f5_555nm;
    float f6_590nm;
    float f7_630nm;
    float f8_680nm;
    float clear;
    float nir;
} i2c_as7341_channels_data_t;

typedef struct {
    uint16_t f1_415nm;
    uint16_t f2_445nm;
    uint16_t f3_480nm;
    uint16_t f4_515nm;
    uint16_t f5_555nm;
    uint16_t f6_590nm;
    uint16_t f7_630nm;
    uint16_t f8_680nm;
    uint16_t clear;
    uint16_t nir;
} i2c_as7341_channels_adc_data_t;

/**
 * @brief AS7341 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t                 dev_config;               /*!< I2C configuration for as7341 device */
    uint16_t                            atime;
    uint16_t                            astep;
    uint8_t                             gain;
    i2c_as7341_led_drive_strengths_t    led_drive_strength;
} i2c_as7341_config_t;


/**
 * @brief AS7341 I2C device structure.
 */
struct i2c_as7341_t {
    i2c_master_dev_handle_t i2c_dev_handle;  /*!< I2C device handle */
    uint8_t                             atime_reg;
    uint16_t                            astep_reg;
    i2c_as7341_led_register_t           led_reg;
    i2c_as7341_enable_register_t        enable_reg;
    i2c_as7341_device_status_register_t dev_status_reg;          /*!< status register */
    i2c_as7341_astatus_register_t       astatus_reg;
    i2c_as7341_status2_register_t       status2_reg;
    i2c_as7341_auxiliary_id_register_t  aux_id_reg;
    i2c_as7341_revision_id_register_t   revision_id_reg;
    i2c_as7341_part_id_register_t       part_id_reg;
    i2c_as7341_config_register_t        config_reg;
    i2c_as7341_config0_register_t       config0_reg;
    i2c_as7341_config1_register_t       config1_reg;
    i2c_as7341_config6_register_t       config6_reg;
};

/**
 * @brief AS7341 I2C device structure definitions.
 * 
 */
typedef struct i2c_as7341_t i2c_as7341_t;
/**
 * @brief AS7341 I2C device handle structure.
 * 
 */
typedef struct i2c_as7341_t *i2c_as7341_handle_t;




esp_err_t i2c_as7341_get_led_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_led_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_led_register_t led_reg);

/**
 * @brief Read device status register from AS7341.
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success. 
 */
esp_err_t i2c_as7341_get_device_status_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_get_astatus_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_get_status2_register(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_get_enable_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_enable_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_enable_register_t enable_reg);

esp_err_t i2c_as7341_get_auxiliary_id_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_get_revision_id_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_get_part_id_register(i2c_as7341_handle_t as7341_handle);


esp_err_t i2c_as7341_get_config_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_config_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config_register_t config_reg);

esp_err_t i2c_as7341_get_config0_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_config0_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config0_register_t config0_reg);


esp_err_t i2c_as7341_get_config1_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_config1_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config1_register_t config1_reg);


esp_err_t i2c_as7341_get_config6_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_config6_register(i2c_as7341_handle_t as7341_handle, i2c_as7341_config6_register_t config6_reg);


esp_err_t i2c_as7341_get_atime_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_atime_register(i2c_as7341_handle_t as7341_handle, uint8_t atime_reg);

esp_err_t i2c_as7341_get_astep_register(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_set_astep_register(i2c_as7341_handle_t as7341_handle, uint16_t astep_reg);


esp_err_t i2c_as7341_set_smux_command(i2c_as7341_handle_t as7341_handle, i2c_as7341_smux_commands_t command);

esp_err_t i2c_as7341_set_atime(i2c_as7341_handle_t as7341_handle, uint8_t atime);

esp_err_t i2c_as7341_set_astep(i2c_as7341_handle_t as7341_handle, uint16_t astep);

esp_err_t i2c_as7341_set_spectral_gain(i2c_as7341_handle_t as7341_handle, i2c_as7341_spectral_gains_t gain);

esp_err_t i2c_as7341_enable_flicker_detection(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_disable_flicker_detection(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_enable_smux(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_enable_wait_time(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_disable_wait_time(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_enable_spectral_measurement(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_disable_spectral_measurement(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_enable_power(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_disable_power(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_enable_hi_register_bank(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_enable_lo_register_bank(i2c_as7341_handle_t as7341_handle);

esp_err_t i2c_as7341_enable_led(i2c_as7341_handle_t as7341_handle);
esp_err_t i2c_as7341_disable_led(i2c_as7341_handle_t as7341_handle);

/**
 * @brief Initializes an AS7341 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] as7341_config configuration of AS7341 device.
 * @param[out] as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as7341_init(i2c_master_bus_handle_t bus_handle, const i2c_as7341_config_t *as7341_config, i2c_as7341_handle_t *as7341_handle);

esp_err_t i2c_as7341_get_adc_measurements(i2c_as7341_handle_t as7341_handle, i2c_as7341_channels_adc_data_t *adc_spectral_data);

esp_err_t i2c_as7341_get_data_status(i2c_as7341_handle_t as7341_handle, bool *ready);

esp_err_t i2c_as7341_get_basic_counts(i2c_as7341_handle_t as7341_handle, i2c_as7341_channels_adc_data_t adc_data, i2c_as7341_channels_data_t *data);

/**
 * @brief removes an AS7341 device from master bus.
 *
 * @param[in] as7341_handle AS7341 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_as7341_rm(i2c_as7341_handle_t as7341_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __AS7341_H__
