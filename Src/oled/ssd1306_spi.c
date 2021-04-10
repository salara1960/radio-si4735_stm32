

#include "hdr.h"
#include "main.h"
#include "ssd1306.h"

#ifdef SET_OLED_SPI

//------------------------------------------------------------------------

uint8_t spi_invert = OLED_CMD_DISPLAY_NORMAL;
uint8_t withDMA = 0;
const uint32_t waits = 100;
bool invText = false;


void CS_OLED_SELECT() {}
void CS_OLED_DESELECT() {}

//******************************************************************************************

void spi_ssd1306_Reset()
{
	CS_OLED_DESELECT();
	HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_WriteCmds(uint8_t *cmds, size_t sz)
{
	CS_OLED_SELECT();
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
	if (withDMA && (sz > 1)) {
		spiRdy = 0;
		uint16_t cnt = 128;
		HAL_SPI_Transmit_DMA(portOLED, (uint8_t *)cmds, sz);
		while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY) {
			cnt--;
			if (!cnt) break;
		}
	} else {
		HAL_SPI_Transmit(portOLED, cmds, sz, waits);//HAL_MAX_DELAY);
	}
	spiRdy = 1;
	CS_OLED_DESELECT();

}
//-----------------------------------------------------------------------------------------
// Send data
void spi_ssd1306_WriteData(const char *buf, size_t sz, uint8_t with)
{

	CS_OLED_SELECT();
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
	if (with) {
		spiRdy = 0;
		uint16_t cnt = 128;
		HAL_SPI_Transmit_DMA(portOLED, (uint8_t *)buf, sz);
		while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY) {
			cnt--;
			if (!cnt) break;
		}
	} else {
		HAL_SPI_Transmit(portOLED, (uint8_t *)buf, sz, waits);//HAL_MAX_DELAY);
	}
	spiRdy = 1;
	CS_OLED_DESELECT();
}
//-----------------------------------------------------------------------------------------
bool spi_ssd1306_shift(uint8_t cy, uint8_t on_off)//0x2e - deactivate, 0x2f - activate
{
bool ret = true;

	uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM,
						OLED_LEFT_HORIZONTAL_SCROLL,//to left shift
						OLED_DUMMY_BYTE,// Dummy
						cy - 1,//Start page
						// 0 -   5 frame, 1 -  64 frame, 2 - 128 frame, 3 - 256 frame
						// 4 -   3 frame, 5 -   4 frame, 6 -  25 frame, 7 -   2 frame
						OLED_TIME_INTERVAL,//Time Interval as 2 frames// 5 - !!!
						cy - 1,//Stop page
						OLED_DUMMY_BYTE,//Dummy
						0xff,
						OLED_CMD_SHIFT_START};
	uint8_t len = sizeof(dat);
	if (on_off == OLED_CMD_SHIFT_STOP) {
		dat[0] = OLED_CONTROL_BYTE_CMD_SINGLE;
		dat[1] = OLED_CMD_SHIFT_STOP;
		len = 2;
		ret = false;
	}

	spi_ssd1306_WriteCmds(dat, len);

	return ret;
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_on(unsigned char flag)
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};

    if (flag) dat[1] = OLED_CMD_DISPLAY_ON;
    	 else dat[1] = OLED_CMD_DISPLAY_OFF;

    spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_init()
{
uint8_t dat[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,//0x00
	OLED_CMD_SET_CHARGE_PUMP,    //0x8D
	0x14,
	OLED_CMD_SET_SEGMENT_REMAP,  //0xA1
	OLED_CMD_SET_COM_SCAN_MODE,  //0xC8
	OLED_CMD_SET_COLUMN_RANGE,   //0x21
	0x00,
	0x7F,
	OLED_CMD_SET_PAGE_RANGE,     //0x22
	0x00,
	0x07,
	OLED_CMD_SHIFT_STOP,
	OLED_CMD_DISPLAY_ON,         //0xAF
	spi_invert
};
/*
	uint8_t dat[] = {
		OLED_CONTROL_BYTE_CMD_STREAM,//0x00
		0xAE, // Display off
	    0x20, // Set Memory Addressing Mode
	    0x10, // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	    0xB0, // Set Page Start Address for Page Addressing Mode,0-7
	    0xC8, // Set COM Output Scan Direction
	    0x00, // Set low column address
	    0x10, // Set high column address
	    0x40, // Set start line address
	    0x81, // set contrast control register
	    0xFF,
		0xA1, // Set segment re-map 0 to 127
	    0xA6, // Set normal display
	    0xA8, // Set multiplex ratio(1 to 64)
	    0x3F,
	    0xA4, // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	    0xD3, // Set display offset
	    0x00, // No offset
	    0xD5, // Set display clock divide ratio/oscillator frequency
	    0xF0, // Set divide ratio
	    0xD9, // Set pre-charge period
	    0x22,
	    0xDA, // Set com pins hardware configuration
	    0x12,
	    0xDB, // Set vcomh
	    0x20, // 0x20,0.77xVcc
	    0x8D, // Set DC-DC enable
	    0x14, //
	    0xAF};// Turn on SSD1306 panel
*/
	spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_invert()
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};

    if (spi_invert == OLED_CMD_DISPLAY_INVERTED) spi_invert = OLED_CMD_DISPLAY_NORMAL;
											else spi_invert = OLED_CMD_DISPLAY_INVERTED;
    dat[1] = spi_invert;

    spi_ssd1306_WriteCmds(dat, sizeof(dat));

}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_clear_line(uint8_t cy, bool inv)
{
uint8_t cif_zero[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
size_t len = sizeof(cif_zero);
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	0,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	cy - 1,
	7
};

	if (inv) for (uint8_t j = 1; j < len; j++) cif_zero[j] = ~cif_zero[j];

	spi_ssd1306_WriteCmds(first, sizeof(first));

	for (uint8_t i = 0; i < 16; i++) {
		spi_ssd1306_WriteCmds(cif_zero, 1);
		spi_ssd1306_WriteData((const char *)&cif_zero[1], len - 1, withDMA);
	}
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_clear_from_to(uint8_t from, uint8_t to)
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
uint8_t zero[BUF_LINE_SIZE + 1] = {0};
size_t len = sizeof(dat);

    zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (uint8_t i = from - 1; i < to; i++) {
    	dat[1] = 0xB0 | i;
    	spi_ssd1306_WriteCmds(dat, len);
    	spi_ssd1306_WriteData((const char *)zero, sizeof(zero), withDMA);
    }
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_clear()
{
    spi_ssd1306_clear_from_to(1, 8);
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_pattern()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
uint8_t buf[129];
size_t len = sizeof(dat);

    buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;
    for (i = 1; i < BUF_LINE_SIZE; i++) buf[i] = 0xFF >> (i % 8);
    for (i = 0; i < 8; i++) {
    	dat[1] = 0xB0 | i;
    	spi_ssd1306_WriteCmds(dat, len);
    	spi_ssd1306_WriteData((const char *)buf, sizeof(buf), withDMA);
    }
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_contrast(uint8_t value)//0xff or 0x00
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, value};

	spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy, bool inv)
{
uint8_t i, lin = cy - 1, col = cx - 1;
int len = strlen(stroka);
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, 0, 0x10, 0};
size_t dl = sizeof(dat);
uint8_t cif[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	col << 3,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	lin,
	7
};

	spi_ssd1306_WriteCmds(first, sizeof(first));
	for (i = 0; i < len; i++) {
		if (stroka[i] == '\n') {
			dat[3] = 0xB0 | ++lin;
			spi_ssd1306_WriteCmds(dat, dl);
		} else {
			memcpy(&cif[1], &font8x8[(uint8_t)stroka[i]][0], FONT_WIDTH);
			if (inv) for (uint8_t j = 1; j < sizeof(cif); j++) cif[j] = ~cif[j];
			spi_ssd1306_WriteCmds(cif, 1);
			spi_ssd1306_WriteData((const char *)&cif[1], sizeof(cif) - 1, withDMA);
		}
	}
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_text(const char *stroka)
{
	if (stroka) spi_ssd1306_text_xy(stroka, 1, 1, false);
}
//-----------------------------------------------------------------------------------------
uint8_t spi_ssd1306_calcx(int len)
{
uint8_t ret = 0;

    if ( (len > 0) && (len < 16) ) ret = ((16 - len) >> 1) + 1;

    return ret;
}
//******************************************************************************************
//-----------------------------------------------------------------------------------------
char *mkLineCenter(char *str, uint16_t width)
{
char st[32] = {0};

	memset(st, 0x20, 32);
	uint8_t slen = OLED_WIDTH / width;
	uint8_t k = strlen(str);
	if (k < slen) {
		uint8_t n = (slen - k) >> 1;
		memcpy((char *)&st[n], (char *)str, k);
		st[slen] = '\0';
		strcpy(str, st);
	}

	return str;
}
//-----------------------------------------------------------------------------------------
char *mkLineWidth(char *str1, char *str2, uint16_t width)
{
char st[64] = {0};

	uint8_t slen = OLED_WIDTH / width;
	uint8_t k1 = strlen(str1);
	uint8_t k2 = strlen(str2);
	if ((k1 + k2) <= slen) {
		uint8_t k = slen - (k1 + k2);
		strcpy(st, str1);
		for (int8_t i = 0; i < k; i++) st[k1 + i] = 0x20;
		strcat(st, str2);
		strcpy(str1, st);
	}

	return str1;
}


#endif

