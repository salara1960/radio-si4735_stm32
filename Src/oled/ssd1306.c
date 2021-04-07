
#include "hdr.h"

#ifdef SET_OLED_I2C

#include "main.h"
#include "ssd1306.h"


//------------------------------------------------------------------------

uint8_t invert = OLED_CMD_DISPLAY_NORMAL;
const uint32_t min_wait_ms = 350;
const uint32_t max_wait_ms = 1000;
//const uint32_t sem_wait = portMAX_DELAY;

//******************************************************************************************
/*
osStatus waitEvent()
{
#ifdef SET_MUTEX_LCD
    return osMutexWait(mutexLCD, sem_wait);
#else
	#ifdef SET_SEM_LCD
    	return osSemaphoreWait(semLCD, sem_wait);
	#else
    	return osOK;
	#endif
#endif
}
//-----------------------------------------------------------------------------------------
void doneEvent()
{
#ifdef SET_MUTEX_LCD
	osMutexRelease(mutexLCD);
#else
	#ifdef SET_SEM_LCD
		osSemaphoreRelease(semLCD);
	#endif
#endif
}
*/
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
uint8_t i2c_ssd1306_calcx(int len)
{
uint8_t ret = 0;

    if ( (len > 0) && (len < 16) ) ret = ((16 - len) >> 1) + 1;

    return ret;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_on(bool flag)
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
HAL_StatusTypeDef rt = HAL_OK;

    if (flag) dat[1] = OLED_CMD_DISPLAY_ON;
    	 else dat[1] = OLED_CMD_DISPLAY_OFF;

    //if (waitEvent() == osOK) {
    	rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
    //	doneEvent();
    //}

    if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_init()
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
	OLED_CMD_DISPLAY_ON,         //0xAF
	invert
};
HAL_StatusTypeDef rt = HAL_OK;

	//if (waitEvent() == osOK) {
		rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
	//	doneEvent();
	//}


	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
#ifdef SET_SSD1306_INVERT
void i2c_ssd1306_invert()
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
HAL_StatusTypeDef rt = HAL_OK;


    if (invert == OLED_CMD_DISPLAY_INVERTED) invert = OLED_CMD_DISPLAY_NORMAL;
										else invert = OLED_CMD_DISPLAY_INVERTED;
    dat[1] = invert;

    //if (waitEvent() == osOK) {
    	rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
    //	doneEvent();
    //}

    if (rt != HAL_OK) devError |= devI2C;
}
#endif
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_clear()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
HAL_StatusTypeDef rt = HAL_OK;
uint8_t zero[129] = {0};

	zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;
	//if ( waitEvent()== osOK) {
		for (i = 0; i < 8; i++) {
			dat[1] = 0xB0 | i;
			rt  = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat,    2, min_wait_ms);
			rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, zero, 129, max_wait_ms);
		}
	//	doneEvent();
	//}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_clear_line(uint8_t cy)
{
cy--;
HAL_StatusTypeDef rt = HAL_OK;
uint8_t cif_zero[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	0,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	cy,
	7
};
uint8_t next[] = {
	OLED_CONTROL_BYTE_CMD_SINGLE,
	0xB0 | cy
};

	//if ( waitEvent()== osOK) {
		if (HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, first, sizeof(first), max_wait_ms) == HAL_OK) {
			if (HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, next, sizeof(next), min_wait_ms) == HAL_OK) {
				for (uint8_t i = 0; i < 16; i++) {
					rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, cif_zero, sizeof(cif_zero), max_wait_ms);
				}
			}
		}
	//	doneEvent();
	//}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_pattern()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
uint8_t buf[129] = {0};
HAL_StatusTypeDef rt = HAL_OK;

	buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;
	for (i = 1; i < 129; i++) buf[i] = 0xFF >> (i % 8);
	//if (waitEvent() == osOK) {
		for (i = 0; i < 8; i++) {
			dat[1] = 0xB0 | i;
			rt  = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat,   2, min_wait_ms);
			rt |= HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, buf, 129, max_wait_ms);
		}
	//	doneEvent();
	//}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_contrast(uint8_t value)//0xff or 0x00
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, value};
HAL_StatusTypeDef rt = HAL_OK;

	//if (waitEvent() == osOK) {
		rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
	//	doneEvent();
	//}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy)
{
HAL_StatusTypeDef rt = HAL_OK;
uint8_t i, lin = cy - 1, col = cx - 1;
int len = strlen(stroka);
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, 0, 0x10, 0};
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

	//if (waitEvent() == osOK) {
		rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, first, sizeof(first), min_wait_ms);
		if (rt == HAL_OK) {
			/*dat[0] = OLED_CONTROL_BYTE_CMD_STREAM;
			dat[1] = 0;
			dat[2] = 0x10;
			dat[3] = 0;*/
			for (i = 0; i < len; i++) {
				if (stroka[i] == '\n') {
					dat[3] = 0xB0 | ++lin;
					rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
				} else {
					memcpy(&cif[1], &font8x8[(uint8_t)stroka[i]][0], 8);
					rt = HAL_I2C_Master_Transmit(portOLED, OLED_I2C_ADDRESS, cif, sizeof(cif), max_wait_ms);
				}
			}
		}
	//	doneEvent();
	//}

	if (rt != HAL_OK) devError |= devI2C;
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_text(const char *stroka)
{
	if (stroka) ssd1306_text_xy(stroka, 1, 1);
}
//******************************************************************************************

#endif

