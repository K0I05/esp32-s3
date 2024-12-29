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
 * @file max31865.c
 *
 * ESP-IDF driver for MAX31865 RTD temperature sensor
 * 
 * https://github.com/UncleRus/esp-idf-lib/blob/master/components/max31865/max31865.c
 * 
 * https://github.com/espressif/esp-idf/blob/v5.3.2/examples/peripherals/spi_master/hd_eeprom/components/eeprom/spi_eeprom.h
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "max31865.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * MAX31865 definitions
 */

#define SPI_MAX31865_REG_CONFIG             UINT8_C(0x00)
#define SPI_MAX31865_REG_RTD_MSB            UINT8_C(0x01)
#define SPI_MAX31865_REG_HIGH_FAULT_MSB     UINT8_C(0x03)
#define SPI_MAX31865_REG_LOW_FAULT_MSB      UINT8_C(0x05)
#define SPI_MAX31865_REG_FAULT_STATUS       UINT8_C(0x07)

#define SPI_MAX31865_DATA_POLL_TIMEOUT_MS  UINT16_C(100)
#define SPI_MAX31865_DATA_READY_DELAY_MS   UINT16_C(2)
#define SPI_MAX31865_POWERUP_DELAY_MS      UINT16_C(120)
#define SPI_MAX31865_RESET_DELAY_MS        UINT16_C(25)
#define SPI_MAX31865_SETUP_DELAY_MS        UINT16_C(15)
#define SPI_MAX31865_APPSTART_DELAY_MS     UINT16_C(10)    /*!< max31865 delay after initialization before application start-up */
#define SPI_MAX31865_CMD_DELAY_MS          UINT16_C(5)     /*!< max31865 delay before attempting SPI transactions after a command is issued */
#define SPI_MAX31865_TX_RX_DELAY_MS        UINT16_C(10)    /*!< max31865 delay after attempting an SPI transmit transaction and attempting an SPI receive transaction */


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "max31865";

typedef struct {
    float a, b;
} spi_rtd_coeff_t;

/*
static const spi_rtd_coeff_t spi_rtd_coeff[] = {
     [SPI_MAX31865_ITS90]         = { .a = 3.9083e-3f, .b = -5.775e-7f },
     [SPI_MAX31865_DIN43760]      = { .a = 3.9848e-3f, .b = -5.8019e-7f },
     [SPI_MAX31865_US_INDUSTRIAL] = { .a = 3.9692e-3f, .b = -5.8495e-7f },
};
*/


static inline void spi_max31865_cs_high(spi_transaction_t* t) {
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((spi_max31865_handle_t)t->user)->spi_dev_config.cs_io_num);
    gpio_set_level(((spi_max31865_handle_t)t->user)->spi_dev_config.cs_io_num, 1);
}

static inline void spi_max31865_cs_low(spi_transaction_t* t) {
    gpio_set_level(((spi_max31865_handle_t)t->user)->spi_dev_config.cs_io_num, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((spi_max31865_handle_t)t->user)->spi_dev_config.cs_io_num);
}

void spi_max31865_ready_rising_isr(void* arg) {
    spi_max31865_handle_t handle = (spi_max31865_handle_t)arg;
    xSemaphoreGive(handle->spi_ready_sem);
    ESP_EARLY_LOGV(TAG, "ready detected.");
}


esp_err_t spi_max31865_init(const spi_max31865_config_t *max31865_config, spi_max31865_handle_t *max31865_handle) {
    esp_err_t ret = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( max31865_config );

    /* validate host and interrupt */
    if (max31865_config->irq_enabled == true && max31865_config->host == SPI1_HOST) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "interrupt cannot be used on SPI1 host, init failed");
    }

    /* validate memory availability for handle */
    spi_max31865_handle_t out_handle = (spi_max31865_handle_t)calloc(1, sizeof(spi_max31865_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for spi max31865 device, init failed");

    /* copy configuration to device handle */
    out_handle->spi_dev_config = *max31865_config;

    /* set device configuration */
    spi_device_interface_config_t spi_dev_conf = {
        .command_bits = 10,
        .clock_speed_hz = out_handle->spi_dev_config.clock_speed_hz,
        .mode = 0,          //SPI mode 0
        /*
         * The timing requirements to read the busy signal from the EEPROM cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
        .pre_cb = spi_max31865_cs_high,
        .post_cb = spi_max31865_cs_low,
        .input_delay_ns = out_handle->spi_dev_config.input_delay_ns,  //the EEPROM output the data half a SPI clock behind.
    };

    /* validate device handle */
    if (out_handle->spi_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(spi_bus_add_device(out_handle->spi_dev_config.host, &spi_dev_conf, &out_handle->spi_dev_handle), err_handle, TAG, "spi new bus for init failed");
    }

    /* configure cs io */
    gpio_config_t cs_gpio_cfg = {
        .pin_bit_mask = BIT64(out_handle->spi_dev_config.cs_io_num),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_set_level(out_handle->spi_dev_config.cs_io_num, 0);
    gpio_config(&cs_gpio_cfg);

    /* configure isr handler */
    if (out_handle->spi_dev_config.irq_enabled == true) {
        out_handle->spi_ready_sem = xSemaphoreCreateBinary();
        ESP_GOTO_ON_FALSE(out_handle->spi_ready_sem, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for spi ready semaphore, init failed");

        gpio_set_intr_type(out_handle->spi_dev_config.miso_io_num, GPIO_INTR_POSEDGE);
        ESP_GOTO_ON_ERROR(gpio_isr_handler_add(out_handle->spi_dev_config.miso_io_num, spi_max31865_ready_rising_isr, out_handle), err_handle, TAG, "add gpio isr handler for init failed");
        gpio_intr_disable(out_handle->spi_dev_config.miso_io_num);
    }

    /* set device handle */
    *max31865_handle = out_handle;

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->spi_dev_handle) {
            spi_bus_remove_device(out_handle->spi_dev_handle);
        }
        if (out_handle && out_handle->spi_ready_sem) {
            vSemaphoreDelete(out_handle->spi_ready_sem);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t spi_max31865_remove(spi_max31865_handle_t max31865_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max31865_handle );

    return ESP_OK;
}

esp_err_t spi_max31865_delete(spi_max31865_handle_t max31865_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( max31865_handle );

    return ESP_OK;
}