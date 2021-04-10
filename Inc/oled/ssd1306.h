#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "hdr.h"
#include "main.h"
#include "fontik.h"

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)

//------------------------------------------------------------------------

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7
#define OLED_CMD_SHIFT_STOP             0x2e    // deactivate shift
#define OLED_CMD_SHIFT_START            0x2f    // activate shift
#define OLED_RIGHT_HORIZONTAL_SCROLL    0x26
#define OLED_LEFT_HORIZONTAL_SCROLL     0x27

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

//
#define OLED_TIME_INTERVAL              0x00    // 0 -   5 frame
												// 1 -  64 frame
                                                // 2 - 128 frame
                                                // 3 - 256 frame
                                                // 4 -   3 frame
                                                // 5 -   4 frame
                                                // 6 -  25 frame
                                                // 7 -   2 frame
#define BUF_LINE_SIZE   128
#define OLED_DUMMY_BYTE 0x00

#define OLED_WIDTH 128
#define FONT_WIDTH 8
#define MAX_FONT_CHAR (OLED_WIDTH / FONT_WIDTH)

#define OLED_DMA DMA1_Channel3

//------------------------------------------------------------------------

#ifdef SET_OLED_I2C
	#define OLED_I2C_ADDRESS                0x3C << 1

	void i2c_ssd1306_on(bool flag);
	void i2c_ssd1306_init();
	void i2c_ssd1306_invert();
	void i2c_ssd1306_clear();
	void i2c_ssd1306_pattern();
	void i2c_ssd1306_contrast(uint8_t value);
	void i2c_ssd1306_clear_line(uint8_t cy);
	void i2c_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy);
	void i2c_ssd1306_text(const char *stroka);
#endif

#ifdef SET_OLED_SPI
	uint8_t withDMA;

	void spi_ssd1306_Reset();
	void spi_ssd1306_WriteCmds(uint8_t *cmds, size_t sz);
	void spi_ssd1306_WriteData(const char *buf, size_t sz, uint8_t with);
	bool spi_ssd1306_shift(uint8_t cy, uint8_t on_off);
	void spi_ssd1306_on(uint8_t flag);
	void spi_ssd1306_init();
	void spi_ssd1306_invert();
	void spi_ssd1306_clear();
	void spi_ssd1306_clear_from_to(uint8_t from, uint8_t to);
	void spi_ssd1306_pattern();
	void spi_ssd1306_contrast(uint8_t value);
	void spi_ssd1306_clear_line(uint8_t cy, bool inv);
	void spi_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy, bool inv);
	void spi_ssd1306_text(const char *stroka);
	uint8_t spi_ssd1306_calcx(int len);
	char *mkLineCenter(char *str, uint16_t width);
	char *mkLineWidth(char *str1, char *str2, uint16_t width);
#endif

//------------------------------------------------------------------------

#endif

#endif /* __SSD1306_H__ */

