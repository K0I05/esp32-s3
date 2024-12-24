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
 * @file ssd1306.h
 * @defgroup drivers ssd1306
 * @{
 *
 * ESP-IDF driver for ssd1306 display panel
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <i2c_master_ext.h>

#ifdef __cplusplus
extern "C" {
#endif



#define I2C_SSD1306_SCL_SPEED_HZ            	UINT32_C(100000)        //!< ssd1306 I2C default clock frequency (100KHz)

#define I2C_SSD1306_DEV_ADDR               		UINT8_C(0x3c)   //!< ssd1306 I2C address


#define I2C_SSD1306_128x32_CONFIG_DEFAULT 	{				\
    .dev_config.device_address  = I2C_SSD1306_DEV_ADDR,		\
	.dev_config.scl_speed_hz    = I2C_SSD1306_SCL_SPEED_HZ, \
    .panel_size                 = I2C_SSD1306_PANEL_128x32,	\
    .offset_x                   = 0,						\
    .flip_enabled               = false }		

#define I2C_SSD1306_128x64_CONFIG_DEFAULT 	{				\
    .dev_config.device_address  = I2C_SSD1306_DEV_ADDR,		\
	.dev_config.scl_speed_hz    = I2C_SSD1306_SCL_SPEED_HZ, \
    .panel_size                 = I2C_SSD1306_PANEL_128x64,	\
    .offset_x                   = 0,						\
    .flip_enabled               = false }


/**
 * @brief SSD1306 I2C scroll step in terms of frame frequency enumerator.
 * 
 */
typedef enum {
	I2C_SSD1306_SCROLL_5_FRAMES 	= 0b000,
	I2C_SSD1306_SCROLL_64_FRAMES 	= 0b001,
	I2C_SSD1306_SCROLL_128_FRAMES 	= 0b010,
	I2C_SSD1306_SCROLL_256_FRAMES	= 0b011,
	I2C_SSD1306_SCROLL_3_FRAMES 	= 0b100,
	I2C_SSD1306_SCROLL_4_FRAMES 	= 0b101,
	I2C_SSD1306_SCROLL_25_FRAMES 	= 0b110,
	I2C_SSD1306_SCROLL_2_FRAMES 	= 0b111
} i2c_ssd1306_scroll_frames_t;

/**
 * @brief SSD1306 I2C scroll types enumerator.
 * 
 */
typedef enum {
	I2C_SSD1306_SCROLL_RIGHT	= 1,
	I2C_SSD1306_SCROLL_LEFT 	= 2,
	I2C_SSD1306_SCROLL_DOWN 	= 3,
	I2C_SSD1306_SCROLL_UP 		= 4,
	I2C_SSD1306_SCROLL_STOP 	= 5
} i2c_ssd1306_scroll_types_t;

/**
 * @brief SSD1306 I2C panel sizes enumerator.
 * 
 */
typedef enum {
	I2C_SSD1306_PANEL_128x32 = 1,
	I2C_SSD1306_PANEL_128x64 = 2
} i2c_ssd1306_panel_sizes_t;

/**
 * @brief SSD1306 I2C page structure.
 */
typedef struct {
	uint8_t segment[128];		/*!< page segment data to display */
} i2c_ssd1306_page_t;

/**
 * @brief SSD1306 I2C device configuration structure.
 */
typedef struct {
    i2c_device_config_t			dev_config;     /*!< configuration for ssd1306 device */
	i2c_ssd1306_panel_sizes_t	panel_size;		/*!< panel size */
	uint8_t						offset_x;	    /*!< x-axis offset */
	bool						flip_enabled;   /*!< displayed information is flipped when true */
} i2c_ssd1306_config_t;

/**
 * @brief SSD1306 I2C device structure.
 */
struct i2c_ssd1306_t {
    i2c_master_dev_handle_t  i2c_dev_handle;  /*!< I2C device handle */
	uint8_t				width;				/*!< width of display panel */
	uint8_t 			height;				/*!< height display panel */
	uint8_t				offset_x;			/*!< x-axis offset */
	bool				display_enabled;	/*!< display is on when true otherwise it is sleeping */
	bool				flip_enabled;		/*!< flips information being displayed when true */
	bool				scroll_enabled;		/*!< scroll enabled when true */
	uint8_t				scroll_start;		/*!< start page of scroll */
	uint8_t				scroll_end;			/*!< end page of scroll */
	int8_t			    scroll_direction;   /*!< scroll direction */
	uint8_t				pages;				/*!< number of pages supported by display panel */
	i2c_ssd1306_page_t	page[8];			/*!< pages of segment data to display */
};

/**
 * @brief SSD1306 I2C device structure definition.
 */
typedef struct i2c_ssd1306_t i2c_ssd1306_t;
/**
 * @brief SSD1306 I2C device handle definition.
 */
typedef struct i2c_ssd1306_t *i2c_ssd1306_handle_t;


/**
 * @brief Turns display panel on.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_on(i2c_ssd1306_handle_t ssd1306_handle);

/**
 * @brief Turns display panel off.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_off(i2c_ssd1306_handle_t ssd1306_handle);

/**
 * @brief Displays segment data for each page supported by the display panel.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_pages(i2c_ssd1306_handle_t ssd1306_handle);

/**
 * @brief Sets segment data for each page supported by the display panel.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param buffer Segement data in 128-byte blocks by page.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_pages(i2c_ssd1306_handle_t ssd1306_handle, uint8_t *buffer);

/**
 * @brief Gets segment data for each page supported by the display panel.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param buffer Segement data in 128-byte blocks by page.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_get_pages(i2c_ssd1306_handle_t ssd1306_handle, uint8_t *buffer);

/**
 * @brief Sets page segment data for a pixel.
 * 
 * @note Call `i2c_ssd1306_display_pages` to display the pixel.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param xpos X-axis position of the pixel.
 * @param ypos Y-axis position of the pixel.
 * @param invert Pixel is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_pixel(i2c_ssd1306_handle_t ssd1306_handle, int16_t xpos, int16_t ypos, bool invert);

/**
 * @brief Sets pages and segments data for a line.
 * 
 * @note Call `i2c_ssd1306_display_pages` to display the line.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param x1 X-axis start position of the line.
 * @param y1 Y-axis start position of the line.
 * @param x2 X-axis end position of the line.
 * @param y2 Y-axis end position of the line.
 * @param invert Line is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_line(i2c_ssd1306_handle_t ssd1306_handle, int16_t x1, int16_t y1, int16_t x2, int16_t y2,  bool invert);

/**
 * @brief Sets pages and segments data for a circle.
 * 
 * @note Call `i2c_ssd1306_display_pages` to display the circle.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param x0 X-axis start position of the circle.
 * @param y0 Y-axis start position of the circle.
 * @param r Radius of the circle.
 * @param invert Circle is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_circle(i2c_ssd1306_handle_t ssd1306_handle, int16_t x0, int16_t y0, int16_t r, bool invert);

/**
 * @brief Sets contrast of the display panel.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param contrast Contrast of information being displayed (0 to 255).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_display_contrast(i2c_ssd1306_handle_t ssd1306_handle, uint8_t contrast);

/**
 * @brief Displays a bitmap.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param xpos X-axis position of the bitmap.
 * @param ypos Y-axis position of the bitmap.
 * @param bitmap Bitmap data.
 * @param width Width of the bitmap.
 * @param height Height of the bitmap
 * @param invert Bitmap is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_bitmap(i2c_ssd1306_handle_t ssd1306_handle, int16_t xpos, int16_t ypos, uint8_t *bitmap, uint8_t width, uint8_t height, bool invert);

/**
 * @brief Displays an image by page and segment.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param page Index of page.
 * @param segment Index of segment data.
 * @param image Image data.
 * @param width Width of the image.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_image(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, uint8_t segment, uint8_t *image, uint8_t width);

/**
 * @brief Displays text by page with a maximum of 16-characters.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param page Index of page.
 * @param text Text characters.
 * @param text_len Length of text (16-characters maximum).
 * @param invert Text is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_text(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, char *text, uint8_t text_len, bool invert);

/**
 * @brief Displays text x2 larger by page.
 * 
 * @note Text displayed uses 2-pages with a maximum of 8-characters.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param page Index of page.
 * @param text Text characters.
 * @param text_len Length of text (8-characters maximum).
 * @param invert Text is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_text_x2(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, char *text, uint8_t text_len, bool invert);

/**
 * @brief Displays text x3 larger by page.
 * 
 * @note Text displayed uses 3-pages with a maximum of 5 characters.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param page Index of page.
 * @param text Text characters.
 * @param text_len Length of text (5-characters maximum).
 * @param invert Text is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_text_x3(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, char *text, uint8_t text_len, bool invert);

/**
 * @brief Clears a page from the display.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param page Index of page to clear from the display.
 * @param invert Background is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_clear_display_page(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, bool invert);

/**
 * @brief Clears the entire display.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param invert Background is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_clear_display(i2c_ssd1306_handle_t ssd1306_handle, bool invert);

/**
 * @brief Sets scroll orientation and frame frequency for hardware based scrolling text.
 * 
 * @note Call `i2c_ssd1306_display_text` to display hardware based scrolling text.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param scroll Scrolling orientation.
 * @param frame_frequency Frame rate of scrolling text.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_hardware_scroll(i2c_ssd1306_handle_t ssd1306_handle, i2c_ssd1306_scroll_types_t scroll, i2c_ssd1306_scroll_frames_t frame_frequency);

/**
 * @brief Sets start and end page for software based scrolling text.
 * 
 * @note Call `i2c_ssd1306_display_scroll_text` to display software based scrolling text.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param start Index of start page.
 * @param end Index of end page.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_software_scroll(i2c_ssd1306_handle_t ssd1306_handle, uint8_t start, uint8_t end);

/**
 * @brief Displays software based scrolling text.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param text Text characters.
 * @param text_len Length of text.
 * @param invert Text is inverted when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_display_scroll_text(i2c_ssd1306_handle_t ssd1306_handle, char *text, uint8_t text_len, bool invert);

/**
 * @brief Clears software based scrolling text from display.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_clear_scroll_display(i2c_ssd1306_handle_t ssd1306_handle);

/**
 * @brief Sets scroll orientation, start and end pages to wrap around the display.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @param scroll Scrolling orientation.
 * @param start Index of page for left or right scroll, othewise, height position for up or down scroll.
 * @param end Index of page for left or right scroll, othewise, height position for up or down scroll.
 * @param delay Delay before information is display, a value 0 there is no wait, and nothing is displayed with a value of -1.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_set_display_wrap_arround(i2c_ssd1306_handle_t ssd1306_handle, i2c_ssd1306_scroll_types_t scroll, uint8_t start, uint8_t end, int8_t delay);

/**
 * @brief Copies bit from source to destination.
 * 
 * @param src 
 * @param src_bits 
 * @param dst 
 * @param dst_bits 
 * @return uint8_t 
 */
uint8_t i2c_ssd1306_copy_bit(uint8_t src, uint8_t src_bits, uint8_t dst, uint8_t dst_bits);

/**
 * @brief Inverts the buffer data.
 * 
 * @param buf Buffer data.
 * @param blen Length of buffer data.
 */
void i2c_ssd1306_invert_buffer(uint8_t *buf, size_t blen);

/**
 * @brief Flips the buffer data (upsidedown).
 * 
 * @param buf Buffer data.
 * @param blen Length of buffer data.
 */
void i2c_ssd1306_flip_buffer(uint8_t *buf, size_t blen);

/**
 * @brief Rotates 8-bits, as an example, 0x12 becomes 0x48.
 * 
 * @param ch1 8-bit value to rotate.
 * @return uint8_t rotated 8-bit value.
 */
uint8_t i2c_ssd1306_rotate_byte(uint8_t ch1);

/**
 * @brief Display is faded out and cleared.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_fadeout_display(i2c_ssd1306_handle_t ssd1306_handle);

/**
 * @brief Initializes an SSD1306 device onto the I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle.
 * @param[in] ssd1306_config configuration of SSD1306 device.
 * @param[out] ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_init(i2c_master_bus_handle_t bus_handle, const i2c_ssd1306_config_t *ssd1306_config, i2c_ssd1306_handle_t *ssd1306_handle);

/**
 * @brief Removes an SSD1306 device from master bus.
 *
 * @param[in] ssd1306_handle SSD1306 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_remove(i2c_ssd1306_handle_t ssd1306_handle);

/**
 * @brief Removes an SSD1306 device from master bus and frees handle.
 * 
 * @param ssd1306_handle SSD1306 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t i2c_ssd1306_delete(i2c_ssd1306_handle_t ssd1306_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SSD1306_H__ */