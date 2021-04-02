/*
 * hdr.h
 *
 *  Created on: Mar 6, 2021
 *      Author: alarm
 */

//#define SET_DBG_INFO

//#define SET_STATIC_MEM

#define SET_ST_IPS
#ifdef SET_ST_IPS
	//#define SET_WITH_DMA
//	#define SET_WITH_CS
	#ifdef SET_WITH_CS
		#define SET_BLK_PIN
	#endif
#endif

//#define SET_OLED_SPI
//#define SET_OLED_I2C

#define SET_SI4735
#ifdef SET_SI4735
//	#define SET_SI4735_INT
#endif

//#define SET_KBD



