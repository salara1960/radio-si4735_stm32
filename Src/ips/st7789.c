#include "hdr.h"
#include "main.h"
#include "st7789.h"

#ifdef SET_ST_IPS

//-----------------------------------------------------------------------------------------


const uint32_t waits = 150;
uint8_t *frm_buf = NULL;
uint8_t *pbar_buf = NULL;



uint16_t arrScr[ST7789_WIDTH] = {0};


static const lcd_init_cmd_t st_init_cmds[] = {
    // Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0
    {0x36, {(1 << 5) | (1 << 6)}, 1},
    // Interface Pixel Format, 16bits/pixel for RGB/MCU interface
    {0x3A, {0x55}, 1},
    // Porch Setting
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    // Gate Control, Vgh=13.65V, Vgl=-10.43V
    {0xB7, {0x45}, 1},
    // VCOM Setting, VCOM=1.175V
    {0xBB, {0x2B}, 1},
    // LCM Control, XOR: BGR, MX, MH
    {0xC0, {0x2C}, 1},
    // VDV and VRH Command Enable, enable=1
    {0xC2, {0x01, 0xff}, 2},
    // VRH Set, Vap=4.4+...
    {0xC3, {0x11}, 1},
    // VDV Set, VDV=0
    {0xC4, {0x20}, 1},
    // Frame Rate Control, 60Hz, inversion=0
    {0xC6, {0x0f}, 1},
    // Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V
    {0xD0, {0xA4, 0xA1}, 1},
    // Positive Voltage Gamma Control
    {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
    // Negative Voltage Gamma Control
    {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
    // Sleep Out
    {0x11, {0}, 0x80},
    // Display On
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}
};

//-----------------------------------------------------------------------------------------

inline void ST7789_Select()
{
#ifdef SET_WITH_CS
	ST7789_SelON();
#endif
}
inline void ST7789_UnSelect()
{
#ifdef SET_WITH_CS
	ST7789_SelOFF();
#endif
}
void st7789_WaitForDMA(void) {
	while(ST7789_DMA->CNDTR);
}
//-------------------------------------------------------------------------------------------
inline uint16_t invColor(uint16_t x)
{
    return ~x;
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
static void ST7789_WriteCommand(uint8_t cmd)
{
	ST7789_Select();
	ST7789_DC_Clr();
	if (HAL_SPI_Transmit(portOLED, &cmd, sizeof(cmd), waits) != HAL_OK) {
		devError |= devSPI;
		cnt_err++;
		putMsg(msg_errCounter);
	} else {
		devError &= ~devSPI;
	}
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
static void ST7789_WriteData(uint8_t *buff, size_t buff_size)
{
HAL_StatusTypeDef rt = HAL_OK;


	ST7789_Select();
	ST7789_DC_Set();

	// split data in small chunks because HAL can't send more than 64K at once
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;

		while(ST7789_DMA->CNDTR);//while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY);
		rt |= HAL_SPI_Transmit_DMA(portOLED, buff, chunk_size);
		//rt |= HAL_SPI_Transmit(portOLED, buff, chunk_size, waits);

		buff += chunk_size;
		buff_size -= chunk_size;

	}

	ST7789_UnSelect();

	if (rt != HAL_OK) {
		devError |= devSPI;
		cnt_err++;
		putMsg(msg_errCounter);
	} else {
		devError &= ~devSPI;
	}
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Write data to ST7789 controller, simplify for 8bit data.
 * data -> data to write
 * @return none
 */
static void ST7789_WriteSmallData(uint8_t data)
{
	ST7789_Select();
	ST7789_DC_Set();
	if (HAL_SPI_Transmit(portOLED, &data, sizeof(data), waits) != HAL_OK) {
		devError |= devSPI;
		cnt_err++;
		putMsg(msg_errCounter);
	} else {
		devError &= ~devSPI;
	}
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
void ipsOn(uint8_t act)
{
	if (act) ST7789_WriteCommand(ST7789_DISPON);
	    else ST7789_WriteCommand(ST7789_DISPOFF);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in st7789.h)
 * @return none
 */
void ST7789_SetRotation(uint8_t m)
{
	ST7789_WriteCommand(ST7789_MADCTL);	// MADCTL
	switch (m) {
	case 0:
		ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		break;
	case 1:
		ST7789_WriteSmallData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	case 2:
		ST7789_WriteSmallData(ST7789_MADCTL_RGB);
		break;
	case 3:
		ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	default:
		break;
	}
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	ST7789_Select();

	uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
	uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;

	one_list_t list[3] = {
		{ST7789_CASET, {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF}, 4},
		{ST7789_RASET, {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF}, 4},
		{ST7789_RAMWR, {0}, 0},
	};

	HAL_StatusTypeDef rt = HAL_OK;
	int8_t i = -1;
	while (++i < 3) {
		while(ST7789_DMA->CNDTR);//while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY);
		ST7789_DC_Clr();
		rt |= HAL_SPI_Transmit_DMA(portOLED, &list[i].cmd , 1);
		if (list[i].len) {
			while(ST7789_DMA->CNDTR);//while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY);
			ST7789_DC_Set();
			rt |= HAL_SPI_Transmit_DMA(portOLED, &list[i].data[0] , list[i].len);
		}
	}

	if (rt != HAL_OK) {
		devError |= devSPI;
		cnt_err++;
		putMsg(msg_errCounter);
	} else {
		devError &= ~devSPI;
	}

	/* Write to RAM */

	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
void ST7789_Reset()
{
    ST7789_RST_Clr();
    HAL_Delay(2);//25
    ST7789_RST_Set();
    HAL_Delay(10);//50
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Initialize ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Init()
{
#ifdef SET_NEW_INIT

	int cmd = 0;
	while (st_init_cmds[cmd].databytes != 0xff) {
		ST7789_WriteCommand(st_init_cmds[cmd].cmd);
		ST7789_WriteData((uint8_t *)st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
		if (st_init_cmds[cmd].databytes & 0x80) {
			HAL_Delay(10);
	    }
		cmd++;
	}

	ST7789_SetRotation(ST7789_ROTATION);

	///Enable backlight
	//ST7789_BlkSet();//ST7789_BlkClr(); //

#else
		
    ST7789_WriteCommand(ST7789_COLMOD);		//	Set color mode
    ST7789_WriteSmallData(ST7789_COLOR_MODE_16bit);//
  	ST7789_WriteCommand(0xB2);				//	Porch control
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		ST7789_WriteData(data, sizeof(data));
	}
	ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Display Rotation)
	
	/* Internal LCD Voltage generator settings */
    ST7789_WriteCommand(0xB7);				//	Gate Control
    ST7789_WriteSmallData(0x35);			//	Default value
    ST7789_WriteCommand(0xBB);				//	VCOM setting
    ST7789_WriteSmallData(0x19);			//	0.725v (default 0.75v for 0x20)
    ST7789_WriteCommand(0xC0);				//	LCMCTRL	
    ST7789_WriteSmallData(0x2C);			//	Default value
    ST7789_WriteCommand(0xC2);				//	VDV and VRH command Enable
    ST7789_WriteSmallData(0x01);			//	Default value
    ST7789_WriteSmallData(0xff);            //	Default value
    ST7789_WriteCommand(0xC3);				//	VRH set
    ST7789_WriteSmallData(0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
    ST7789_WriteCommand(0xC4);				//	VDV set
    ST7789_WriteSmallData(0x20);			//	Default value
    ST7789_WriteCommand(0xC6);				//	Frame rate control in normal mode
    ST7789_WriteSmallData(0x0F);			//	Default value (60HZ)
    ST7789_WriteCommand(0xD0);				//	Power control
    ST7789_WriteSmallData(0xA4);			//	Default value
    ST7789_WriteSmallData(0xA1);			//	Default value
	/**************** Division line ****************/

	ST7789_WriteCommand(0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}

    ST7789_WriteCommand(0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		ST7789_WriteData(data, sizeof(data));
	}

  	uint8_t cmds[] = {ST7789_INVON, ST7789_SLPOUT, ST7789_NORON, ST7789_DISPON};
  	ST7789_WriteCommands(cmds, sizeof(cmds));


	ST7789_Fill_Color(invColor(BLUE));				//	Fill with Black.

#endif

}
//-----------------------------------------------------------------------------------------
/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(uint16_t color)
{

	ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
//	ST7789_Select();

	uint16_t cm = 16;//for stm32f103 :  57600 / 16 = 3600 ; for stm32f411 : 57600 / 2 = 28800
	int len = ((ST7789_WIDTH * ST7789_HEIGHT) / cm) << 1;
	if (!frm_buf) frm_buf = (uint8_t *)calloc(1, len);
	if (!frm_buf) {
		devError |= devMem;
		return;
	}

	uint16_t j = 0;
	while (j < len) {
		frm_buf[j++] = color >> 8;
		frm_buf[j++] = color & 0xff;
	}
	HAL_StatusTypeDef rt = HAL_OK;
	ST7789_DC_Set();
	for (uint16_t j = 0; j < cm; j++) {
		while(ST7789_DMA->CNDTR);//while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY);
		rt |= HAL_SPI_Transmit_DMA(portOLED, frm_buf, len);// * dma_spi2_cnt);
	}
	if (rt != HAL_OK) {
		devError |= devSPI;
		cnt_err++;
		putMsg(msg_errCounter);
	} else {
		devError &= ~devSPI;
	}
	//while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY);

	if (frm_buf) {
		free(frm_buf);
		frm_buf =  NULL;
	}

	//	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;
	
	ST7789_SetAddressWindow(x, y, x, y);
	uint8_t data[] = {color >> 8, color & 0xFF};
	ST7789_Select();
	ST7789_WriteData(data, sizeof(data));
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Fill an Area with single color
 * @param xSta&ySta -> coordinate of the start point
 * @param xEnd&yEnd -> coordinate of the end point
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
	if ((xEnd >= ST7789_WIDTH) || (yEnd >= ST7789_HEIGHT))	return;

	ST7789_Select();
	uint16_t i, j;
	uint8_t data[] = {color >> 8, color & 0xFF};
	ST7789_SetAddressWindow(xSta, ySta, xEnd, yEnd);
	for (i = ySta; i <= yEnd; i++) {
		for (j = xSta; j <= xEnd; j++) ST7789_WriteData(data, sizeof(data));
	}
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Draw a big Pixel at a point
 * @param x&y -> coordinate of the point
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel_4px(uint16_t x, uint16_t y, uint16_t color)
{
	if ((x > ST7789_WIDTH) || (y > ST7789_HEIGHT))	return;

	ST7789_Select();
	ST7789_Fill(x - 1, y - 1, x + 1, y + 1, color);
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void ST7789_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
	uint16_t swap;
    uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);

    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = ABS(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            ST7789_DrawPixel(y0, x0, color);
        } else {
            ST7789_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Draw a Rectangle with single color
 * @param xi&yi -> 2 coordinates of 2 top points.
 * @param color -> color of the Rectangle line
 * @return none
 */
void ST7789_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	ST7789_Select();
	ST7789_DrawLine(x1, y1, x2, y1, color);
	ST7789_DrawLine(x1, y1, x1, y2, color);
	ST7789_DrawLine(x1, y2, x2, y2, color);
	ST7789_DrawLine(x2, y1, x2, y2, color);
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/** 
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_Select();
	ST7789_DrawPixel(x0, y0 + r, color);
	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	ST7789_DrawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawPixel(x0 + x, y0 + y, color);
		ST7789_DrawPixel(x0 - x, y0 + y, color);
		ST7789_DrawPixel(x0 + x, y0 - y, color);
		ST7789_DrawPixel(x0 - x, y0 - y, color);

		ST7789_DrawPixel(x0 + y, y0 + x, color);
		ST7789_DrawPixel(x0 - y, y0 + x, color);
		ST7789_DrawPixel(x0 + y, y0 - x, color);
		ST7789_DrawPixel(x0 - y, y0 - x, color);
	}
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;
	if ((x + w - 1) >= ST7789_WIDTH) return;
	if ((y + h - 1) >= ST7789_HEIGHT) return;

	ST7789_Select();
	ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);
	ST7789_WriteData((uint8_t *)data, sizeof(uint16_t) * w * h);
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Invert Fullscreen color
 * @param invert -> Whether to invert
 * @return none
 */
void ST7789_InvertColors(uint8_t invert)
{
	ST7789_Select();
	ST7789_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/** 
 * @brief Write a char
 * @param  x&y -> cursor of the start point.
 * @param ch -> char to write
 * @param font -> fontstyle of the string
 * @param color -> color of the char
 * @param bgcolor -> background color of the char
 * @return  none
 */
void ST7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{

	ST7789_Select();

	ST7789_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);
	uint8_t cdata[] = {color >> 8, color & 0xFF};
	uint8_t bdata[] = {bgcolor >> 8, bgcolor & 0xFF};
	uint8_t *uk = NULL;

	uint32_t i, b, j;
	for (i = 0; i < font.height; i++) {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++) {
			if ((b << j) & 0x8000) {
				uk = cdata;
			} else {
				uk = bdata;
			}
			ST7789_WriteData(uk, sizeof(cdata));
		}
	}

	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------

/** 
 * @brief Write a string 
 * @param  x&y -> cursor of the start point.
 * @param str -> string to write
 * @param font -> fontstyle of the string
 * @param color -> color of the string
 * @param bgcolor -> background color of the string
 * @return  none
 */
void ST7789_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	if (!str) return;

	ST7789_Select();
	while (*str) {
		if (x + font.width >= ST7789_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= ST7789_HEIGHT) break;

			if (*str == ' ') {// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		if (*str != '\n') {
			ST7789_WriteChar(x, y, *str, font, color, bgcolor);
			x += font.width;
		} else {
			x = 0;
			y += font.height;
		}
		str++;
	}
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/** 
 * @brief Draw a filled Rectangle with single color
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param color -> color of the Rectangle
 * @return  none
 */
void ST7789_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	ST7789_Select();
	uint8_t i;

	/* Check input parameters */
	if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;

	/* Check width and height */
	if ((x + w) >= ST7789_WIDTH) w = ST7789_WIDTH - x;
	if ((y + h) >= ST7789_HEIGHT) h = ST7789_HEIGHT - y;

	/* Draw lines */
	for (i = 0; i <= h; i++) ST7789_DrawLine(x, y + i, x + w, y + i, color);

	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/** 
 * @brief Draw a Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the lines
 * @return  none
 */
void ST7789_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	ST7789_Select();
	// Draw lines
	ST7789_DrawLine(x1, y1, x2, y2, color);
	ST7789_DrawLine(x2, y2, x3, y3, color);
	ST7789_DrawLine(x3, y3, x1, y1, color);
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/** 
 * @brief Draw a filled Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the triangle
 * @return  none
 */
void ST7789_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	ST7789_Select();
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
			yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
			curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay) {
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ST7789_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/** 
 * @brief Draw a Filled circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle
 * @return  none
 */
void ST7789_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	ST7789_Select();

	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(x0, y0 + r, color);
	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	ST7789_DrawPixel(x0 - r, y0, color);
	ST7789_DrawLine(x0 - r, y0, x0 + r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
		ST7789_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

		ST7789_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
		ST7789_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
	}

	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Open/Close tearing effect line
 * @param tear -> Whether to tear
 * @return none
 */
void ST7789_TearEffect(uint8_t tear)
{
	ST7789_Select();
	ST7789_WriteCommand(tear ? 0x35 /* TEON */ : 0x34 /* TEOFF */);
	ST7789_UnSelect();
}
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
char *mkLineCenter(char *str, uint16_t width)
{
char st[128] = {0};

	memset(st, 0x20, 127);
	uint8_t slen = ST7789_WIDTH / width;
	uint8_t k = strlen(str);
	if (k < slen) {
		uint8_t n = (slen - k)/2;
		memcpy((char *)&st[n], (char *)str, k);
		st[k + (n << 1)] = '\0';
		strcpy(str, st);
	}

	return str;
}
//-------------------------------------------------------------------------------------
void ST7789_DrawDuga(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color, void *adr)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;
	uint16_t *ar = (uint16_t *)adr;

	ST7789_Select();

	ST7789_DrawPixel(x0, y0 - r, color);
	ST7789_DrawPixel(x0 + r, y0, color);
	if (ar) {
		memset((uint8_t *)ar, ST7789_HEIGHT - r - 10, sizeof(uint16_t) * ST7789_WIDTH);
		if (y0 < ST7789_HEIGHT) {
			ar[x0] = y0 - r;
			if (((x0 + r) >= 0) && ((x0 + r) < ST7789_WIDTH)) ar[x0 + r] = y0;
			ar[0] = y0;
			ar[ST7789_WIDTH - 1] = y0;
		}
	}

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawPixel(x0 + x, y0 - y, color);
		ST7789_DrawPixel(x0 - x, y0 - y, color);
		if (ar) {
			if ((y0 - y) < ST7789_HEIGHT) {
				if (((x0 + x) >= 0) && ((x0 + x) < ST7789_WIDTH)) ar[x0 + x] = y0 - y;
				if (((x0 - x) >= 0) && ((x0 - x) < ST7789_WIDTH)) ar[x0 - x] = y0 - y;
			}
		}

		ST7789_DrawPixel(x0 + y, y0 - x, color);
		ST7789_DrawPixel(x0 - y, y0 - x, color);
		if (ar) {
			if ((y0 - x) < ST7789_HEIGHT) {
				if (((x0 + y) >= 0) && ((x0 + y) < ST7789_WIDTH)) ar[x0 + y] = y0 - x;
				if (((x0 - y) >= 0) && ((x0 - y) < ST7789_WIDTH)) ar[x0 - y] = y0 - x;
			}
		}
	}
	ST7789_UnSelect();
}
//-------------------------------------------------------------------------------------
void ST7789_DrawEllipse(uint16_t xc, uint16_t yc, uint16_t a, uint16_t b, float alpha, uint16_t color, void *adr)
{
	float t = 3.1415 / 180;
    alpha = 360 - alpha;
    int theta;
    uint16_t cx, cy;
    uint16_t *ar = (uint16_t *)adr;
    memset((uint8_t *)ar, 0, sizeof(uint16_t) * ST7789_WIDTH);

    // Заполняем каждый соответствующий пиксель на каждый угол от 0 до 360

    for (int i = 0; i < 360; i += 1) {
        theta = i;
        int x = a * cos(t * theta) * cos(t * alpha) + b * sin(t * theta) * sin(t * alpha);
        int y = b * sin(t * theta) * cos(t * alpha) - a * cos(t * theta) * sin(t * alpha);

        cx = xc + x;
        cy = yc - y;
        if ((cx >= 10) && (cx <= ST7789_WIDTH - 10) && (cy <= yc)) {
        	ST7789_DrawPixel(cx, cy, color);
        	if (ar) {
        		if (cy < ST7789_HEIGHT) ar[cx] = cy;
        	}
        }
    }

}
//--------------------------------------------------------------------------------------
void mkFace(area_t *rc)
{
    ST7789_DrawRectangle(rc->x1, rc->y1, rc->x2, rc->y2, rc->bcolor);
    //
    //ST7789_DrawDuga((ST7789_WIDTH >> 1) - 1, rc->y2, (ST7789_WIDTH >> 1) - 1 + 0, invColor(WHITE), (void *)rc->duga);

    //ST7789_DrawEllipse((ST7789_WIDTH >> 1) - 1, ST7789_WIDTH - 1 - 40, 100, 70, 0, rc->bcolor, (void *)rc->duga);

    /*uint16_t xx =  0, ugl = -10;
    while (xx < ST7789_WIDTH) {
        ST7789_DrawLine(xx, rc->duga[xx] - 2, xx - ugl, rc->duga[xx] + 2, invColor(WHITE));
        xx += STEP;
        ugl += 2;
    }*/
}
//-------------------------------------------------------------------------------------
uint16_t updateFace(area_t *rc, uint16_t x, uint16_t cf)
{
uint16_t ret = 0;

	x *= (ST7789_WIDTH / cf);
    if (x < ST7789_WIDTH) {
        if (lcorX != x) ST7789_DrawLine(ST7789_WIDTH >> 1, rc->y2, lcorX, rc->duga[lcorX] + 15, invColor(BLUE));

        uint16_t color = GREEN;
        uint16_t pers = (x  * 100) / ST7789_WIDTH;
        if (pers < 20) color = RED;
        else
        if (pers < 50) color = YELLOW;

        ST7789_DrawLine(ST7789_WIDTH >> 1, rc->y2, x, rc->duga[x] + 15, invColor(color));

        lcorX = x;

        ret = x / cf;

    }

    return ret;
}
//-------------------------------------------------------------------------------------
void updateBar(area_t *rc, uint16_t val)
{
uint16_t color = invColor(GREEN);
uint16_t bc = invColor(BLUE);
uint16_t h = rc->y2 - rc->y1 - 2;
uint16_t sx = rc->x1 + 2;

    uint16_t w = ST7789_WIDTH - rc->x1 - 6;
    uint16_t wa = w;
    if (val) {
    	if (val < 63) w = val * 3;
    } else {
    	color = bc;
    }
    int len = w << 1;

    if (!pbar_buf) {
    	pbar_buf = (uint8_t *)calloc(1, wa + 2);
    	if (!pbar_buf) {
    		devError |= devMem;
    		return;
    	}
    }

    uint32_t j = 0;
    while (j < (wa << 1)) {
    	if (j > len) color = bc;
    	pbar_buf[j++] = color >> 8;
    	pbar_buf[j++] = color & 0xff;
    }

    //ST7789_Select();
    HAL_StatusTypeDef rt = HAL_OK;
    for (j = rc->y1 + 1; j <= rc->y1 + 1 + h; j++) {
    	while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY);
    	ST7789_SetAddressWindow(sx, j, sx + wa, j);
    	ST7789_DC_Set();
    	rt |= HAL_SPI_Transmit_DMA(portOLED, pbar_buf, wa << 1);
    }
    if (rt != HAL_OK) {
    	devError |= devSPI;
    	cnt_err++;
    	putMsg(msg_errCounter);
    } else {
    	devError &= ~devSPI;
    }

    //ST7789_UnSelect();

    /*if (pbar_buf) {
    	free(pbar_buf);
    	pbar_buf = NULL;
    }*/



}
//--------------------------------------------------------------------------------------
void ST7789_setScrollArea(uint16_t tfa, uint16_t bfa)
{
uint16_t vsa = ST7789_WIDTH - tfa - bfa; // ST7789 240x240 VRAM

	ST7789_WriteCommand(ST7789_VSCRDEF);   // SETSCROLLAREA = 0x33

	uint8_t data[] = {tfa & 0xff, vsa & 0xff, bfa & 0xff};

	ST7789_WriteData(data, sizeof(data));
}
//-------------------------------------------------------------------------------------
void ST7789_setScroll(uint16_t vsp)
{
	ST7789_WriteCommand(ST7789_VSCRSADD); // VSCRSADD = 0x37

	ST7789_WriteData((uint8_t *)&vsp, 1);
}
//-------------------------------------------------------------------------------------
void ST7789_setPartArea(uint16_t sr, uint16_t er)
{

	ST7789_WriteCommand(ST7789_PTLAR);  // SETPARTAREA = 0x30

	uint8_t data[] = {sr & 0xff, er & 0xff};
	ST7789_WriteData(data, sizeof(data));
}
//-------------------------------------------------------------------------------------
// doesn't work ?
/*void ST7789_setBrightness(uint8_t br)
{
uint8_t val = 0x04;

	ST7789_WriteCommand(ST7789_WRCTRLD);
	ST7789_WriteData(&val, 1);
	ST7789_WriteCommand(ST7789_WRDISBV);
	ST7789_WriteData(&br, 1);
}*/
//-------------------------------------------------------------------------------------


#endif
