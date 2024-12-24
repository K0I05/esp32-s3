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
 * @file ssd1306.c
 *
 * ESP-IDF driver for SSD1306 display panel
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "ssd1306.h"
#include "font8x8_latin.h"
#include <string.h>
#include <esp_log.h>
#include <esp_check.h>
//#include <driver/gpio.h>
#include <i2c_master_ext.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Following definitions are bollowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

/* Control byte for i2c
Co : bit 8 : Continuation Bit 
 * 1 = no-continuation (only one byte to follow) 
 * 0 = the controller should expect a stream of bytes. 
D/C# : bit 7 : Data/Command Select bit 
 * 1 = the next byte or byte stream will be Data. 
 * 0 = a Command byte or byte stream will be coming up next. 
 Bits 6-0 will be all zeros. 
Usage: 
0x80 : Single Command byte 
0x00 : Command Stream 
0xC0 : Single Data byte 
0x40 : Data Stream
*/
#define I2C_SSD1306_CONTROL_BYTE_CMD_SINGLE    0x80
#define I2C_SSD1306_CONTROL_BYTE_CMD_STREAM    0x00
#define I2C_SSD1306_CONTROL_BYTE_DATA_SINGLE   0xC0
#define I2C_SSD1306_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define I2C_SSD1306_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define I2C_SSD1306_CMD_DISPLAY_RAM            0xA4
#define I2C_SSD1306_CMD_DISPLAY_ALLON          0xA5
#define I2C_SSD1306_CMD_DISPLAY_NORMAL         0xA6
#define I2C_SSD1306_CMD_DISPLAY_INVERTED       0xA7
#define I2C_SSD1306_CMD_DISPLAY_OFF            0xAE
#define I2C_SSD1306_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define I2C_SSD1306_CMD_SET_MEMORY_ADDR_MODE   0x20
#define I2C_SSD1306_CMD_SET_HORI_ADDR_MODE     0x00    // Horizontal Addressing Mode
#define I2C_SSD1306_CMD_SET_VERT_ADDR_MODE     0x01    // Vertical Addressing Mode
#define I2C_SSD1306_CMD_SET_PAGE_ADDR_MODE     0x02    // Page Addressing Mode
#define I2C_SSD1306_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define I2C_SSD1306_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define I2C_SSD1306_CMD_SET_DISPLAY_START_LINE 0x40
#define I2C_SSD1306_CMD_SET_SEGMENT_REMAP_0    0xA0    
#define I2C_SSD1306_CMD_SET_SEGMENT_REMAP_1    0xA1    
#define I2C_SSD1306_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define I2C_SSD1306_CMD_SET_COM_SCAN_MODE      0xC8    
#define I2C_SSD1306_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define I2C_SSD1306_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define I2C_SSD1306_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define I2C_SSD1306_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define I2C_SSD1306_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define I2C_SSD1306_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define I2C_SSD1306_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

// Scrolling Command
#define I2C_SSD1306_CMD_HORIZONTAL_RIGHT       0x26
#define I2C_SSD1306_CMD_HORIZONTAL_LEFT        0x27
#define I2C_SSD1306_CMD_CONTINUOUS_SCROLL      0x29
#define I2C_SSD1306_CMD_DEACTIVE_SCROLL        0x2E
#define I2C_SSD1306_CMD_ACTIVE_SCROLL          0x2F
#define I2C_SSD1306_CMD_VERTICAL               0xA3


/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define PACK8 __attribute__((aligned( __alignof__( uint8_t ) ), packed ))

/*
* static constant declerations
*/
static const char *TAG = "ssd1306";

typedef union i2c_ssd1306_out_column_t {
	uint32_t u32;
	uint8_t  u8[4];
} PACK8 i2c_ssd1306_out_column_t;


// Set pixel to internal buffer. Not show it.
esp_err_t i2c_ssd1306_set_pixel(i2c_ssd1306_handle_t ssd1306_handle, int16_t xpos, int16_t ypos, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	uint8_t _page = (ypos / 8);
	uint8_t _bits = (ypos % 8);
	uint8_t _seg = xpos;
	uint8_t wk0 = ssd1306_handle->page[_page].segment[_seg];
	uint8_t wk1 = 1 << _bits;

	ESP_LOGD(TAG, "ypos=%d _page=%d _bits=%d wk0=0x%02x wk1=0x%02x", ypos, _page, _bits, wk0, wk1);

	if (invert) {
		wk0 = wk0 & ~wk1;
	} else {
		wk0 = wk0 | wk1;
	}

	if (ssd1306_handle->flip_enabled) wk0 = i2c_ssd1306_rotate_byte(wk0);

	ESP_LOGD(TAG, "wk0=0x%02x wk1=0x%02x", wk0, wk1);

	ssd1306_handle->page[_page].segment[_seg] = wk0;

	return ESP_OK;
}

// Set line to internal buffer. Not show it.
esp_err_t i2c_ssd1306_set_line(i2c_ssd1306_handle_t ssd1306_handle, int16_t x1, int16_t y1, int16_t x2, int16_t y2,  bool invert) {
	int16_t i;
	int16_t dx,dy;
	int16_t sx,sy;
	int16_t E;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	/* distance between two points */
	dx = ( x2 > x1 ) ? x2 - x1 : x1 - x2;
	dy = ( y2 > y1 ) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = ( x2 > x1 ) ? 1 : -1;
	sy = ( y2 > y1 ) ? 1 : -1;

	/* inclination < 1 */
	if ( dx > dy ) {
		E = -dx;
		for ( i = 0 ; i <= dx ; i++ ) {
			i2c_ssd1306_set_pixel(ssd1306_handle, x1, y1, invert);
			x1 += sx;
			E += 2 * dy;
			if ( E >= 0 ) {
				y1 += sy;
				E -= 2 * dx;
			}
		}
	/* inclination >= 1 */
	} else {
		E = -dy;
		for ( i = 0 ; i <= dy ; i++ ) {
			i2c_ssd1306_set_pixel(ssd1306_handle, x1, y1, invert);
			y1 += sy;
			E += 2 * dx;
			if ( E >= 0 ) {
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}

	return ESP_OK;
}

// Draw circle
esp_err_t i2c_ssd1306_set_circle(i2c_ssd1306_handle_t ssd1306_handle, int16_t x0, int16_t y0, int16_t r, bool invert) {
	int16_t x;
	int16_t y;
	int16_t err;
	int16_t old_err;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	x=0;
	y=-r;
	err=2-2*r;

	do {
		i2c_ssd1306_set_pixel(ssd1306_handle, x0-x, y0+y, invert); 
		i2c_ssd1306_set_pixel(ssd1306_handle, x0-y, y0-x, invert); 
		i2c_ssd1306_set_pixel(ssd1306_handle, x0+x, y0-y, invert); 
		i2c_ssd1306_set_pixel(ssd1306_handle, x0+y, y0+x, invert); 

		if ((old_err=err)<=x)	err+=++x*2+1;
		if (old_err>y || err>x) err+=++y*2+1;	 
	} while(y < 0);

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_on(i2c_ssd1306_handle_t ssd1306_handle) {
	uint8_t out_buf[2];
	uint8_t out_index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	out_buf[out_index++] = I2C_SSD1306_CONTROL_BYTE_CMD_STREAM; // 00
	out_buf[out_index++] = I2C_SSD1306_CMD_DISPLAY_ON; // AF

	ESP_RETURN_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, out_index, I2C_XFR_TIMEOUT_MS), TAG, "write contrast configuration failed");

	/* set handle parameter */
	ssd1306_handle->display_enabled = true;

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_off(i2c_ssd1306_handle_t ssd1306_handle) {
	uint8_t out_buf[2];
	uint8_t out_index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	out_buf[out_index++] = I2C_SSD1306_CONTROL_BYTE_CMD_STREAM; // 00
	out_buf[out_index++] = I2C_SSD1306_CMD_DISPLAY_OFF; // AE

	ESP_RETURN_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, out_index, I2C_XFR_TIMEOUT_MS), TAG, "write contrast configuration failed");

	/* set handle parameter */
	ssd1306_handle->display_enabled = false;

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_pages(i2c_ssd1306_handle_t ssd1306_handle) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	for (uint8_t page = 0; page < ssd1306_handle->pages; page++) {
		ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, page, 0, ssd1306_handle->page[page].segment, ssd1306_handle->width), TAG, "show buffer failed (page %d)", page);
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_set_pages(i2c_ssd1306_handle_t ssd1306_handle, uint8_t *buffer) {
	uint8_t index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	for (uint8_t page = 0; page < ssd1306_handle->pages; page++) {
		memcpy(&ssd1306_handle->page[page].segment, &buffer[index], 128);
		index = index + 128;
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_get_pages(i2c_ssd1306_handle_t ssd1306_handle, uint8_t *buffer) {
	uint8_t index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	for (uint8_t page = 0; page < ssd1306_handle->pages; page++) {
		memcpy(&buffer[index], &ssd1306_handle->page[page].segment, 128);
		index = index + 128;
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_bitmap(i2c_ssd1306_handle_t ssd1306_handle, int16_t xpos, int16_t ypos, uint8_t *bitmap, uint8_t width, uint8_t height, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	if ( (width % 8) != 0) {
		ESP_LOGE(TAG, "width must be a multiple of 8");
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t _width = width / 8;
	uint8_t wk0;
	uint8_t wk1;
	uint8_t wk2;
	uint8_t page = (ypos / 8);
	uint8_t _seg = xpos;
	uint8_t dstBits = (ypos % 8);

	ESP_LOGD(TAG, "ypos=%d page=%d dstBits=%d", ypos, page, dstBits);

	uint8_t offset = 0;

	for(uint16_t _height = 0; _height < height; _height++) {
		for (uint16_t index = 0; index < _width; index++) {
			for (int8_t srcBits=7; srcBits>=0; srcBits--) {
				wk0 = ssd1306_handle->page[page].segment[_seg];
				if (ssd1306_handle->flip_enabled) wk0 = i2c_ssd1306_rotate_byte(wk0);

				wk1 = bitmap[index+offset];
				if (invert) wk1 = ~wk1;

				wk2 = i2c_ssd1306_copy_bit(wk1, srcBits, wk0, dstBits);
				if (ssd1306_handle->flip_enabled) wk2 = i2c_ssd1306_rotate_byte(wk2);

				ESP_LOGD(TAG, "index=%d offset=%d page=%d _seg=%d, wk2=%02x", index, offset, page, _seg, wk2);
				ssd1306_handle->page[page].segment[_seg] = wk2;
				_seg++;
			}
		}
		vTaskDelay(1);
		offset = offset + _width;
		dstBits++;
		_seg = xpos;
		if (dstBits == 8) {
			page++;
			dstBits=0;
		}
	}

	ESP_RETURN_ON_ERROR(i2c_ssd1306_display_pages(ssd1306_handle), TAG, "display pages for bitmap failed");

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_image(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, uint8_t segment, uint8_t *image, uint8_t width) {
	esp_err_t ret = ESP_OK;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	if (page >= ssd1306_handle->pages) return ESP_ERR_INVALID_SIZE;
	if (segment >= ssd1306_handle->width) return ESP_ERR_INVALID_SIZE;

	uint8_t _seg = segment + ssd1306_handle->offset_x;
	uint8_t columLow = _seg & 0x0F;
	uint8_t columHigh = (_seg >> 4) & 0x0F;

	uint8_t _page = page;
	if (ssd1306_handle->flip_enabled) {
		_page = (ssd1306_handle->pages - page) - 1;
	}

	uint8_t *out_buf;
	out_buf = malloc(width + 1);
	if (out_buf == NULL) {
		ESP_LOGE(TAG, "malloc for image display failed");
		return ESP_ERR_NO_MEM;
	}

	uint8_t out_index = 0;
	out_buf[out_index++] = I2C_SSD1306_CONTROL_BYTE_CMD_STREAM;
	// Set Lower Column Start Address for Page Addressing Mode
	out_buf[out_index++] = (0x00 + columLow);
	// Set Higher Column Start Address for Page Addressing Mode
	out_buf[out_index++] = (0x10 + columHigh);
	// Set Page Start Address for Page Addressing Mode
	out_buf[out_index++] = 0xB0 | _page;

	ESP_GOTO_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, out_index, I2C_XFR_TIMEOUT_MS), err, TAG, "write page addressing mode for image display failed");

	out_buf[0] = I2C_SSD1306_CONTROL_BYTE_DATA_STREAM;

	memcpy(&out_buf[1], image, width);

	ESP_GOTO_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, width + 1, I2C_XFR_TIMEOUT_MS), err, TAG, "write image for image display failed");

	free(out_buf);

	// Set to internal buffer
	memcpy(&ssd1306_handle->page[page].segment[segment], image, width);

	return ESP_OK;

	err:
	free(out_buf);
	return ret;
}

esp_err_t i2c_ssd1306_display_text(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, char *text, uint8_t text_len, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	if (page >= ssd1306_handle->pages) return ESP_ERR_INVALID_SIZE;

	if (text_len > 16) text_len = 16;

	uint8_t seg = 0;
	uint8_t image[8];

	for (uint8_t i = 0; i < text_len; i++) {
		memcpy(image, font8x8_latin_tr[(uint8_t)text[i]], 8);
		if (invert) i2c_ssd1306_invert_buffer(image, 8);
		if (ssd1306_handle->flip_enabled) i2c_ssd1306_flip_buffer(image, 8);
		ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, page, seg, image, 8), TAG, "display image for display text failed");
		seg = seg + 8;
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_text_x2(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, char *text, uint8_t text_len, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	if (page >= ssd1306_handle->pages) return ESP_ERR_INVALID_SIZE;

	if (text_len > 8) text_len = 8;

	uint8_t seg = 0;

	for (uint8_t nn = 0; nn < text_len; nn++) {
		uint8_t const * const in_columns = font8x8_latin_tr[(uint8_t)text[nn]];

		// make the character 2x as high
		i2c_ssd1306_out_column_t out_columns[8];
		memset(out_columns, 0, sizeof(out_columns));

		for (uint8_t xx = 0; xx < 8; xx++) { // for each column (x-direction)
			uint32_t in_bitmask = 0b1;
			uint32_t out_bitmask = 0b11;

			for (uint8_t yy = 0; yy < 8; yy++) { // for pixel (y-direction)
				if (in_columns[xx] & in_bitmask) {
					out_columns[xx].u32 |= out_bitmask;
				}
				in_bitmask <<= 1;
				out_bitmask <<= 2;
			}
		}

		// render character in 8 column high pieces, making them 2x as wide
		for (uint8_t yy = 0; yy < 2; yy++)	{ // for each group of 8 pixels high (y-direction)
			uint8_t image[16];
			for (uint8_t xx = 0; xx < 8; xx++) { // for each column (x-direction)
				image[xx*2+0] = 
				image[xx*2+1] = out_columns[xx].u8[yy];
			}
			if (invert) i2c_ssd1306_invert_buffer(image, 16);
			if (ssd1306_handle->flip_enabled) i2c_ssd1306_flip_buffer(image, 16);
			
			ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, page+yy, seg, image, 16), TAG, "display image for display text x2 failed");

			memcpy(&ssd1306_handle->page[page+yy].segment[seg], image, 16);
		}
		seg = seg + 16;
	}

	return ESP_OK;
}

// by Coert Vonk
esp_err_t i2c_ssd1306_display_text_x3(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, char *text, uint8_t text_len, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	if (page >= ssd1306_handle->pages) return ESP_ERR_INVALID_SIZE;

	if (text_len > 5) text_len = 5;

	uint8_t seg = 0;

	for (uint8_t nn = 0; nn < text_len; nn++) {
		uint8_t const * const in_columns = font8x8_latin_tr[(uint8_t)text[nn]];

		// make the character 3x as high
		i2c_ssd1306_out_column_t out_columns[8];
		memset(out_columns, 0, sizeof(out_columns));

		for (uint8_t xx = 0; xx < 8; xx++) { // for each column (x-direction)
			uint32_t in_bitmask = 0b1;
			uint32_t out_bitmask = 0b111;

			for (uint8_t yy = 0; yy < 8; yy++) { // for pixel (y-direction)
				if (in_columns[xx] & in_bitmask) {
					out_columns[xx].u32 |= out_bitmask;
				}
				in_bitmask <<= 1;
				out_bitmask <<= 3;
			}
		}

		// render character in 8 column high pieces, making them 3x as wide
		for (uint8_t yy = 0; yy < 3; yy++)	{ // for each group of 8 pixels high (y-direction)
			uint8_t image[24];
			for (uint8_t xx = 0; xx < 8; xx++) { // for each column (x-direction)
				image[xx*3+0] = 
				image[xx*3+1] = 
				image[xx*3+2] = out_columns[xx].u8[yy];
			}
			if (invert) i2c_ssd1306_invert_buffer(image, 24);
			if (ssd1306_handle->flip_enabled) i2c_ssd1306_flip_buffer(image, 24);
			
			ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, page+yy, seg, image, 24), TAG, "display image for display text x3 failed");

			memcpy(&ssd1306_handle->page[page+yy].segment[seg], image, 24);
		}
		seg = seg + 24;
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_clear_display_page(i2c_ssd1306_handle_t ssd1306_handle, uint8_t page, bool invert) {
	char space[16];

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	memset(space, 0x00, sizeof(space));

	ESP_RETURN_ON_ERROR(i2c_ssd1306_display_text(ssd1306_handle, page, space, sizeof(space), invert), TAG, "display text for clear line failed");

	return ESP_OK;
}

esp_err_t i2c_ssd1306_clear_display(i2c_ssd1306_handle_t ssd1306_handle, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	for (uint8_t page = 0; page < ssd1306_handle->pages; page++) {
		ESP_RETURN_ON_ERROR(i2c_ssd1306_clear_display_page(ssd1306_handle, page, invert), TAG, "clear line for clear screen failed");
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_set_display_contrast(i2c_ssd1306_handle_t ssd1306_handle, uint8_t contrast) {
	uint8_t out_buf[3];
	uint8_t out_index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	out_buf[out_index++] = I2C_SSD1306_CONTROL_BYTE_CMD_STREAM; // 00
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_CONTRAST; // 81
	out_buf[out_index++] = contrast;

	ESP_RETURN_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, out_index, I2C_XFR_TIMEOUT_MS), TAG, "write contrast configuration failed");

	return ESP_OK;
}

esp_err_t i2c_ssd1306_set_software_scroll(i2c_ssd1306_handle_t ssd1306_handle, uint8_t start, uint8_t end) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	ESP_LOGD(TAG, "software_scroll start=%d end=%d _pages=%d", start, end, ssd1306_handle->pages);
	
	if (start >= ssd1306_handle->pages || end >= ssd1306_handle->pages) {
		ssd1306_handle->scroll_enabled = false;
	} else {
		ssd1306_handle->scroll_enabled = true;
		ssd1306_handle->scroll_start = start;
		ssd1306_handle->scroll_end = end;
		ssd1306_handle->scroll_direction = 1;
		if (start > end ) ssd1306_handle->scroll_direction = -1;
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_display_scroll_text(i2c_ssd1306_handle_t ssd1306_handle, char *text, uint8_t text_len, bool invert) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	ESP_LOGD(TAG, "ssd1306_handle->dev_params->scroll_enabled=%d", ssd1306_handle->scroll_enabled);
	if (ssd1306_handle->scroll_enabled == false) return ESP_ERR_INVALID_ARG;

	uint16_t srcIndex = ssd1306_handle->scroll_end - ssd1306_handle->scroll_direction;
	while(1) {
		uint16_t dstIndex = srcIndex + ssd1306_handle->scroll_direction;
		ESP_LOGD(TAG, "srcIndex=%d dstIndex=%d", srcIndex,dstIndex);
		for(uint16_t seg = 0; seg < ssd1306_handle->width; seg++) {
			ssd1306_handle->page[dstIndex].segment[seg] = ssd1306_handle->page[srcIndex].segment[seg];
		}
		ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, dstIndex, 0, ssd1306_handle->page[dstIndex].segment, sizeof(ssd1306_handle->page[dstIndex].segment)), TAG, "display image for scroll text failed");
		if (srcIndex == ssd1306_handle->scroll_start) break;
		srcIndex = srcIndex - ssd1306_handle->scroll_direction;
	}
	
	uint8_t _text_len = text_len;
	if (_text_len > 16) _text_len = 16;
	
	ESP_RETURN_ON_ERROR(i2c_ssd1306_display_text(ssd1306_handle, srcIndex, text, text_len, invert), TAG, "display text for scroll text failed");

	return ESP_OK;
}

esp_err_t i2c_ssd1306_clear_scroll_display(i2c_ssd1306_handle_t ssd1306_handle) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	ESP_LOGD(TAG, "ssd1306_handle->dev_params->scroll_enabled=%d", ssd1306_handle->scroll_enabled);
	if (ssd1306_handle->scroll_enabled == false) return ESP_ERR_INVALID_ARG;

	uint16_t srcIndex = ssd1306_handle->scroll_end - ssd1306_handle->scroll_direction;
	while(1) {
		uint16_t dstIndex = srcIndex + ssd1306_handle->scroll_direction;
		ESP_LOGD(TAG, "srcIndex=%d dstIndex=%d", srcIndex,dstIndex);
		ESP_RETURN_ON_ERROR(i2c_ssd1306_clear_display_page(ssd1306_handle, dstIndex, false), TAG, "clear display page for scroll clear failed");
		if (dstIndex == ssd1306_handle->scroll_start) break;
		srcIndex = srcIndex - ssd1306_handle->scroll_direction;
	}

	return ESP_OK;
}

esp_err_t i2c_ssd1306_set_hardware_scroll(i2c_ssd1306_handle_t ssd1306_handle, i2c_ssd1306_scroll_types_t scroll, i2c_ssd1306_scroll_frames_t frame_frequency) {
    uint8_t out_buf[11];
    uint8_t out_index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	out_buf[out_index++] = I2C_SSD1306_CONTROL_BYTE_CMD_STREAM; // 00

	if (scroll == I2C_SSD1306_SCROLL_RIGHT) {
		out_buf[out_index++] = I2C_SSD1306_CMD_HORIZONTAL_RIGHT; // 26
		out_buf[out_index++] = 0x00; // Dummy byte
		out_buf[out_index++] = 0x00; // Define start page address
		out_buf[out_index++] = frame_frequency; // Frame frequency
		out_buf[out_index++] = 0x07; // Define end page address
		out_buf[out_index++] = 0x00; // Dummy byte 0x00
		out_buf[out_index++] = 0xFF; // Dummy byte 0xFF
		out_buf[out_index++] = I2C_SSD1306_CMD_ACTIVE_SCROLL; // 2F
	} 

	if (scroll == I2C_SSD1306_SCROLL_LEFT) {
		out_buf[out_index++] = I2C_SSD1306_CMD_HORIZONTAL_LEFT; // 27
		out_buf[out_index++] = 0x00; // Dummy byte
		out_buf[out_index++] = 0x00; // Define start page address
		out_buf[out_index++] = frame_frequency; // Frame frequency
		out_buf[out_index++] = 0x07; // Define end page address
		out_buf[out_index++] = 0x00; //
		out_buf[out_index++] = 0xFF; //
		out_buf[out_index++] = I2C_SSD1306_CMD_ACTIVE_SCROLL; // 2F
	} 

	if (scroll == I2C_SSD1306_SCROLL_DOWN) {
		out_buf[out_index++] = I2C_SSD1306_CMD_CONTINUOUS_SCROLL; // 29
		out_buf[out_index++] = 0x00; // Dummy byte
		out_buf[out_index++] = 0x00; // Define start page address
		out_buf[out_index++] = frame_frequency; // Frame frequency
		out_buf[out_index++] = 0x00; // Define end page address
		out_buf[out_index++] = 0x3F; // Vertical scrolling offset

		out_buf[out_index++] = I2C_SSD1306_CMD_VERTICAL; // A3
		out_buf[out_index++] = 0x00;
		if (ssd1306_handle->height == 64)
			out_buf[out_index++] = 0x40;
		if (ssd1306_handle->height == 32)
			out_buf[out_index++] = 0x20;
		out_buf[out_index++] = I2C_SSD1306_CMD_ACTIVE_SCROLL; // 2F
	}

	if (scroll == I2C_SSD1306_SCROLL_UP) {
		out_buf[out_index++] = I2C_SSD1306_CMD_CONTINUOUS_SCROLL; // 29
		out_buf[out_index++] = 0x00; // Dummy byte
		out_buf[out_index++] = 0x00; // Define start page address
		out_buf[out_index++] = frame_frequency; // Frame frequency
		out_buf[out_index++] = 0x00; // Define end page address
		out_buf[out_index++] = 0x01; // Vertical scrolling offset

		out_buf[out_index++] = I2C_SSD1306_CMD_VERTICAL; // A3
		out_buf[out_index++] = 0x00;
		if (ssd1306_handle->height == 64)
			out_buf[out_index++] = 0x40;
		if (ssd1306_handle->height == 32)
			out_buf[out_index++] = 0x20;
		out_buf[out_index++] = I2C_SSD1306_CMD_ACTIVE_SCROLL; // 2F
	}

	if (scroll == I2C_SSD1306_SCROLL_STOP) {
		out_buf[out_index++] = I2C_SSD1306_CMD_DEACTIVE_SCROLL; // 2E
	}

	ESP_RETURN_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, out_index, I2C_XFR_TIMEOUT_MS), TAG, "write hardware scroll configuration failed");

	return ESP_OK;
}

// delay = 0 : display with no wait
// delay > 0 : display with wait
// delay < 0 : no display
esp_err_t i2c_ssd1306_set_display_wrap_arround(i2c_ssd1306_handle_t ssd1306_handle, i2c_ssd1306_scroll_types_t scroll, uint8_t start, uint8_t end, int8_t delay) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	if (scroll == I2C_SSD1306_SCROLL_RIGHT) {
		uint8_t _start = start; // 0 to 7
		uint8_t _end = end; // 0 to 7
		if (_end >= ssd1306_handle->pages) _end = ssd1306_handle->pages - 1;
		uint8_t wk;
		for (uint8_t page = _start; page <= _end;page++) {
			wk = ssd1306_handle->page[page].segment[127];
			for (uint8_t seg = 127; seg > 0; seg--) {
				ssd1306_handle->page[page].segment[seg] = ssd1306_handle->page[page].segment[seg-1];
			}
			ssd1306_handle->page[page].segment[0] = wk;
		}
	} else if (scroll == I2C_SSD1306_SCROLL_LEFT) {
		uint8_t _start = start; // 0 to 7
		uint8_t _end = end; // 0 to 7
		if (_end >= ssd1306_handle->pages) _end = ssd1306_handle->pages - 1;
		uint8_t wk;
		for (uint8_t page=_start;page<=_end;page++) {
			wk = ssd1306_handle->page[page].segment[0];
			for (uint8_t seg = 0; seg < 127; seg++) {
				ssd1306_handle->page[page].segment[seg] = ssd1306_handle->page[page].segment[seg+1];
			}
			ssd1306_handle->page[page].segment[127] = wk;
		}

	} else if (scroll == I2C_SSD1306_SCROLL_UP) {
		uint8_t _start = start; // 0 to {width-1}
		uint8_t _end = end; // 0 to {width-1}
		if (_end >= ssd1306_handle->width) _end = ssd1306_handle->width - 1;
		uint8_t wk0;
		uint8_t wk1;
		uint8_t wk2;
		uint8_t save[128];
		// Save pages 0
		for (uint8_t seg = 0; seg < 128; seg++) {
			save[seg] = ssd1306_handle->page[0].segment[seg];
		}
		// Page0 to Page6
		for (uint8_t page=0; page < ssd1306_handle->pages-1; page++) {
			for (uint8_t seg = _start; seg <= _end; seg++) {
				wk0 = ssd1306_handle->page[page].segment[seg];
				wk1 = ssd1306_handle->page[page+1].segment[seg];
				if (ssd1306_handle->flip_enabled) wk0 = i2c_ssd1306_rotate_byte(wk0);
				if (ssd1306_handle->flip_enabled) wk1 = i2c_ssd1306_rotate_byte(wk1);
				if (seg == 0) {
					ESP_LOGD(TAG, "b page=%d wk0=%02x wk1=%02x", page, wk0, wk1);
				}
				wk0 = wk0 >> 1;
				wk1 = wk1 & 0x01;
				wk1 = wk1 << 7;
				wk2 = wk0 | wk1;
				if (seg == 0) {
					ESP_LOGD(TAG, "a page=%d wk0=%02x wk1=%02x wk2=%02x", page, wk0, wk1, wk2);
				}
				if (ssd1306_handle->flip_enabled) wk2 = i2c_ssd1306_rotate_byte(wk2);
				ssd1306_handle->page[page].segment[seg] = wk2;
			}
		}
		// Page7
		uint8_t pages = ssd1306_handle->pages-1;
		for (uint8_t seg = _start; seg <= _end; seg++) {
			wk0 = ssd1306_handle->page[pages].segment[seg];
			wk1 = save[seg];
			if (ssd1306_handle->flip_enabled) wk0 = i2c_ssd1306_rotate_byte(wk0);
			if (ssd1306_handle->flip_enabled) wk1 = i2c_ssd1306_rotate_byte(wk1);
			wk0 = wk0 >> 1;
			wk1 = wk1 & 0x01;
			wk1 = wk1 << 7;
			wk2 = wk0 | wk1;
			if (ssd1306_handle->flip_enabled) wk2 = i2c_ssd1306_rotate_byte(wk2);
			ssd1306_handle->page[pages].segment[seg] = wk2;
		}

	} else if (scroll == I2C_SSD1306_SCROLL_DOWN) {
		uint8_t _start = start; // 0 to {width-1}
		uint8_t _end = end; // 0 to {width-1}
		if (_end >= ssd1306_handle->width) _end = ssd1306_handle->width - 1;
		uint8_t wk0;
		uint8_t wk1;
		uint8_t wk2;
		uint8_t save[128];
		// Save pages 7
		uint8_t pages = ssd1306_handle->pages-1;
		for (uint8_t seg = 0; seg < 128; seg++) {
			save[seg] = ssd1306_handle->page[pages].segment[seg];
		}
		// Page7 to Page1
		for (uint8_t page = pages; page > 0; page--) {
			for (uint8_t seg = _start; seg <= _end; seg++) {
				wk0 = ssd1306_handle->page[page].segment[seg];
				wk1 = ssd1306_handle->page[page-1].segment[seg];
				if (ssd1306_handle->flip_enabled) wk0 = i2c_ssd1306_rotate_byte(wk0);
				if (ssd1306_handle->flip_enabled) wk1 = i2c_ssd1306_rotate_byte(wk1);
				if (seg == 0) {
					ESP_LOGD(TAG, "b page=%d wk0=%02x wk1=%02x", page, wk0, wk1);
				}
				wk0 = wk0 << 1;
				wk1 = wk1 & 0x80;
				wk1 = wk1 >> 7;
				wk2 = wk0 | wk1;
				if (seg == 0) {
					ESP_LOGD(TAG, "a page=%d wk0=%02x wk1=%02x wk2=%02x", page, wk0, wk1, wk2);
				}
				if (ssd1306_handle->flip_enabled) wk2 = i2c_ssd1306_rotate_byte(wk2);
				ssd1306_handle->page[page].segment[seg] = wk2;
			}
		}
		// Page0
		for (uint8_t seg = _start; seg <= _end; seg++) {
			wk0 = ssd1306_handle->page[0].segment[seg];
			wk1 = save[seg];
			if (ssd1306_handle->flip_enabled) wk0 = i2c_ssd1306_rotate_byte(wk0);
			if (ssd1306_handle->flip_enabled) wk1 = i2c_ssd1306_rotate_byte(wk1);
			wk0 = wk0 << 1;
			wk1 = wk1 & 0x80;
			wk1 = wk1 >> 7;
			wk2 = wk0 | wk1;
			if (ssd1306_handle->flip_enabled) wk2 = i2c_ssd1306_rotate_byte(wk2);
			ssd1306_handle->page[0].segment[seg] = wk2;
		}

	}

	if(delay >= 0) {
		for (uint8_t page = 0; page < ssd1306_handle->pages; page++) {
			ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, page, 0, ssd1306_handle->page[page].segment, 128), TAG, "display image for wrap around failed");
			if (delay) vTaskDelay(delay);
		}
	}

	return ESP_OK;
}

void i2c_ssd1306_invert_buffer(uint8_t *buf, size_t blen) {
	uint8_t wk;
	for(uint16_t i = 0; i < blen; i++) {
		wk = buf[i];
		buf[i] = ~wk;
	}
}

uint8_t i2c_ssd1306_copy_bit(uint8_t src, uint8_t src_bits, uint8_t dst, uint8_t dst_bits) {
	ESP_LOGD(TAG, "src=%02x src_bits=%d dst=%02x dst_bits=%d", src, src_bits, dst, dst_bits);

	uint8_t smask = 0x01 << src_bits;
	uint8_t dmask = 0x01 << dst_bits;
	uint8_t _src = src & smask;
	uint8_t _dst;

	if (_src != 0) {
		_dst = dst | dmask; // set bit
	} else {
		_dst = dst & ~(dmask); // clear bit
	}

	return _dst;
}

// Flip upside down
void i2c_ssd1306_flip_buffer(uint8_t *buf, size_t blen) {
	for(uint16_t i = 0; i < blen; i++) {
		buf[i] = i2c_ssd1306_rotate_byte(buf[i]);
	}
}


uint8_t i2c_ssd1306_rotate_byte(uint8_t ch1) {
	uint8_t ch2 = 0;

	for (int8_t j = 0; j < 8; j++) {
		ch2 = (ch2 << 1) + (ch1 & 0x01);
		ch1 = ch1 >> 1;
	}

	return ch2;
}

esp_err_t i2c_ssd1306_fadeout_display(i2c_ssd1306_handle_t ssd1306_handle) {
	uint8_t image[1];

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	for(uint8_t page = 0; page < ssd1306_handle->pages; page++) {
		image[0] = 0xFF;
		for(uint8_t line=0; line<8; line++) {
			if (ssd1306_handle->flip_enabled) {
				image[0] = image[0] >> 1;
			} else {
				image[0] = image[0] << 1;
			}
			for(uint8_t seg = 0; seg < 128; seg++) {
				ESP_RETURN_ON_ERROR(i2c_ssd1306_display_image(ssd1306_handle, page, seg, image, 1), TAG, "display image for fadeout failed");
				ssd1306_handle->page[page].segment[seg] = image[0];
			}
		}
	}

	return ESP_OK;
}

static inline esp_err_t i2c_ssd1306_setup(i2c_ssd1306_handle_t ssd1306_handle) {
	uint8_t	out_buf[27];
	uint8_t	out_index = 0;

	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

	out_buf[out_index++] = I2C_SSD1306_CONTROL_BYTE_CMD_STREAM;
	out_buf[out_index++] = I2C_SSD1306_CMD_DISPLAY_OFF;	         // AE
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_MUX_RATIO;           // A8
	if (ssd1306_handle->height == 64) out_buf[out_index++] = 0x3F;
	if (ssd1306_handle->height == 32) out_buf[out_index++] = 0x1F;
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_DISPLAY_OFFSET;      // D3
	out_buf[out_index++] = 0x00;
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_DISPLAY_START_LINE;	 // 40
	if (ssd1306_handle->flip_enabled) {
		out_buf[out_index++] = I2C_SSD1306_CMD_SET_SEGMENT_REMAP_0; // A0
	} else {
		out_buf[out_index++] = I2C_SSD1306_CMD_SET_SEGMENT_REMAP_1;  // A1
	}
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_COM_SCAN_MODE;		// C8
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_DISPLAY_CLK_DIV;		// D5
	out_buf[out_index++] = 0x80;
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_COM_PIN_MAP;			// DA
	if (ssd1306_handle->height == 64) out_buf[out_index++] = 0x12;
	if (ssd1306_handle->height == 32) out_buf[out_index++] = 0x02;
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_CONTRAST;			// 81
	out_buf[out_index++] = 0xFF;
	out_buf[out_index++] = I2C_SSD1306_CMD_DISPLAY_RAM;				// A4
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_VCOMH_DESELCT;		// DB
	out_buf[out_index++] = 0x40;
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_MEMORY_ADDR_MODE;	// 20
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_PAGE_ADDR_MODE;		// 02
	// Set Lower Column Start Address for Page Addressing Mode
	out_buf[out_index++] = 0x00;
	// Set Higher Column Start Address for Page Addressing Mode
	out_buf[out_index++] = 0x10;
	out_buf[out_index++] = I2C_SSD1306_CMD_SET_CHARGE_PUMP;			// 8D
	out_buf[out_index++] = 0x14;
	out_buf[out_index++] = I2C_SSD1306_CMD_DEACTIVE_SCROLL;			// 2E
	out_buf[out_index++] = I2C_SSD1306_CMD_DISPLAY_NORMAL;			// A6
	out_buf[out_index++] = I2C_SSD1306_CMD_DISPLAY_ON;				// AF

	ESP_RETURN_ON_ERROR(i2c_master_transmit(ssd1306_handle->i2c_dev_handle, out_buf, out_index, I2C_XFR_TIMEOUT_MS), TAG, "write setup configuration failed");

	ssd1306_handle->display_enabled = true;

	return ESP_OK;
}

esp_err_t i2c_ssd1306_init(i2c_master_bus_handle_t bus_handle, const i2c_ssd1306_config_t *ssd1306_config, i2c_ssd1306_handle_t *ssd1306_handle) {
	/* validate arguments */
	ESP_ARG_CHECK( bus_handle && ssd1306_config );

	/* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(bus_handle, ssd1306_config->dev_config.device_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ssd1306 device handle initialization failed", ssd1306_config->dev_config.device_address);

	/* validate memory availability for handle */
    i2c_ssd1306_handle_t out_handle = (i2c_ssd1306_handle_t)calloc(1, sizeof(i2c_ssd1306_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ssd1306 device, init failed");

	/* set device configuration */
	const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = ssd1306_config->dev_config.device_address,
        .scl_speed_hz       = ssd1306_config->dev_config.scl_speed_hz,
    };

	/* validate device handle */
    if (out_handle->i2c_dev_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &out_handle->i2c_dev_handle), err_handle, TAG, "i2c new bus for init failed");
    }

	/* copy configuration */
	if(ssd1306_config->panel_size == I2C_SSD1306_PANEL_128x32) {
		out_handle->width 		= 128;
		out_handle->height		= 32;
		out_handle->pages 		= 4; 
	} else {
		out_handle->width 		= 128;
		out_handle->height		= 64;
		out_handle->pages		= 8;
	}
	out_handle->offset_x 		= ssd1306_config->offset_x;
	out_handle->flip_enabled	= ssd1306_config->flip_enabled;

	for (uint8_t i = 0; i < out_handle->pages; i++) {
		memset(out_handle->page[i].segment, 0, 128);
	}

	/* attempt to setup display */
	ESP_GOTO_ON_ERROR(i2c_ssd1306_setup(out_handle), err_handle, TAG, "panel setup for init failed");

	/* set device handle */
    *ssd1306_handle = out_handle;

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_dev_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_dev_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t i2c_ssd1306_remove(i2c_ssd1306_handle_t ssd1306_handle) {
	/* validate parameters */
	ESP_ARG_CHECK( ssd1306_handle );

    return i2c_master_bus_rm_device(ssd1306_handle->i2c_dev_handle);
}

esp_err_t i2c_ssd1306_delete(i2c_ssd1306_handle_t ssd1306_handle) {
	/* validate arguments */
    ESP_ARG_CHECK( ssd1306_handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( i2c_ssd1306_remove(ssd1306_handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(ssd1306_handle->i2c_dev_handle) {
        free(ssd1306_handle->i2c_dev_handle);
        free(ssd1306_handle);
    }

    return ESP_OK;
}




