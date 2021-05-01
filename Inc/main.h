/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <malloc.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>
#include "hdr.h"

#if defined(SET_OLED_SPI)
	#include "ssd1306.h"
#elif defined(SET_ST_IPS)
	#include "st7789.h"
#endif
#ifdef SET_IRED
	#include "IRremote.h"
#endif

#ifdef SET_SI4735
	#include "si4735.h"
#endif

#ifdef SET_KBD
	#include "mpr121.h"
#endif

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum {
	devOK = 0,
	devSPI = 1,
	devUART = 2,
	devADC = 4,
	devI2C = 8,
	devFifo = 0x10,
	devMem = 0x20,
	devKBD = 0x40
};

typedef enum {
	msg_empty = 0,
	msg_rst,
	msg_500ms,
	msg_sec,
	msg_showLibs,
//	msg_encCounter,
//	msg_encPressed,
	msg_encReleased,
//	msg_keyEvent,
	msg_kbd,
	msg_incFrec,
	msg_decFrec,
	msg_updateScr,
	msg_showAll,
	msg_setEpoch,
	msg_none
} evt_t;

enum {
	NONE = 0,
	KEY1,
	KEY2,
	KEY3,
	EKEY
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MAX_ENC_VALUE 32767
#define MIN_ENC_VALUE 0
#define ENC_PERIOD 65535
#define LED_ERROR_Pin GPIO_PIN_1
#define LED_ERROR_GPIO_Port GPIOA
#define ENC_LED_Pin GPIO_PIN_2
#define ENC_LED_GPIO_Port GPIOA
#define KBD_INT_Pin GPIO_PIN_3
#define KBD_INT_GPIO_Port GPIOA
#define KBD_INT_EXTI_IRQn EXTI3_IRQn
#define IRED_Pin GPIO_PIN_4
#define IRED_GPIO_Port GPIOA
#define ENC_KEY_Pin GPIO_PIN_6
#define ENC_KEY_GPIO_Port GPIOA
#define ENC_KEY_EXTI_IRQn EXTI9_5_IRQn
#define OLED_DC_Pin GPIO_PIN_7
#define OLED_DC_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define SI_RST_Pin GPIO_PIN_1
#define SI_RST_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_4
#define OLED_RST_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_5
#define OLED_MOSI_GPIO_Port GPIOB
#define TIM4_CH1_S2_Pin GPIO_PIN_6
#define TIM4_CH1_S2_GPIO_Port GPIOB
#define TIM4_CH2_S1_Pin GPIO_PIN_7
#define TIM4_CH2_S1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/*
#define htons(x) \
    ((uint16_t)((x >> 8) | ((x << 8) & 0xff00)))
#define htonl(x) \
    ((uint32_t)((x >> 24) | ((x >> 8) & 0xff00) | ((x << 8) & 0xff0000) | ((x << 24) & 0xff000000)))
*/

#define _10ms 1
#define _20ms (_10ms * 2)
#define _30ms (_10ms * 3)
#define _40ms (_10ms * 4)
#define _50ms (_10ms * 5)
#define _60ms (_10ms * 6)
#define _70ms (_10ms * 7)
#define _80ms (_10ms * 8)
#define _90ms (_10ms * 9)
#define _100ms (_10ms * 10)
#define _150ms (_10ms * 15)
#define _200ms (_10ms * 20)
#define _250ms (_10ms * 25)
#define _300ms (_10ms * 30)
#define _350ms (_10ms * 35)
#define _400ms (_10ms * 40)
#define _450ms (_10ms * 45)
#define _500ms (_10ms * 50)
#define _600ms (_10ms * 60)
#define _700ms (_10ms * 70)
#define _800ms (_10ms * 80)
#define _900ms (_10ms * 90)
#define _1s (_100ms * 10)
#define _1s5 (_100ms * 15)
#define _2s (_1s * 2)//2000
#define _3s (_1s * 3)//3000
#define _4s (_1s * 4)//4000
#define _5s (_1s * 5)//5000
#define _10s (_1s * 10)//10000
//#define _15s (_1s * 15)
//#define _20s (_1s * 20)
//#define _25s (_1s * 25)
//#define _30s (_1s * 30)

#define MAX_FIFO_SIZE  32
#define MAX_UART_BUF  768

#define TIME_encKeyPressed 75
#define TIME_btKeyPressed 80

#define min_wait_ms 150
#define max_wait_ms 1000



#ifdef SET_OLED_SPI
	#ifdef WITH_CS
		#define CS_OLED_SELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
		#define CS_OLED_DESELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)
	#endif
#endif


typedef struct {
	uint16_t cel;
	uint16_t dro;
} s_float_t;


DMA_HandleTypeDef hdma_spi1_tx;

uint8_t devError;
uint32_t spiRdy;
volatile uint32_t cnt_err;


#ifdef SET_KBD
//	#define min_wait_ms 150
//	#define max_wait_ms 1000

	I2C_HandleTypeDef *portKBD;
	int16_t kbdAddr;
#endif

#if defined(SET_ST_IPS) || defined(SET_OLED_SPI)
	SPI_HandleTypeDef *portOLED;
	uint8_t screenON;
#endif
#if defined(SET_ST_IPS)
	const FontDef *fntKey;
	const FontDef *tFont;
	const FontDef *lFont;
#endif


#ifdef SET_IRED

	#define MAX_IRED_KEY 21

	enum {
		key_ch_minus = 0,
		key_ch,
		key_ch_plus,
		key_left,
		key_right,
		key_sp,
		key_minus,
		key_plus,
		key_eq,
		key_100,
		key_200,
		key_0,
		key_1,
		key_2,
		key_3,
		key_4,
		key_5,
		key_6,
		key_7,
		key_8,
		key_9
	};

	typedef struct {
		char name[8];
		uint32_t code;
	} one_key_t;

	TIM_HandleTypeDef *portIRED;//htim3; // таймер для приёма

#endif

#ifdef SET_SI4735
	#define FM_BAND_TYPE 0
	#define MW_BAND_TYPE 1
	#define SW_BAND_TYPE 2
	#define LW_BAND_TYPE 3
	#define FMm 0
	#define LSBm 1
	#define USBm 2
	#define AMm 3
	#define LW 4
	#define SSB 1
	//#define MIN_ELAPSED_TIME 100
	//#define MIN_ELAPSED_RSSI_TIME 150
	//
	//
	//
	#define max_allStep 5
    #define max_rModes 4

    #define minFrecFM 6400  //MHz
    #define maxFrecFM 10800 //MHz
	#define curFrecFM 10390 //MHz
	//
	#define minFrecAM 520  //KHZ
    #define maxFrecAM 1710 //KHz
	#define curFrecAM 810  //KHz
	//
	#define minFrecSW 1800  //KHZ
	#define maxFrecSW 7500  //KHz
	#define curFrecSW 3640  //KHz
	//
	#define minFrecLW 153 //KHz
	#define maxFrecLW 279 //KHz
	#define curFrecLW 216 //KHz


	#define MIN_aVol   0
	#define MAX_aVol  63
    #define STEP_aVol  8

	typedef struct {
		uint8_t bandType;     // Band type (FM, MW or SW)
		uint16_t minimumFreq; // Minimum frequency of the band
		uint16_t maximumFreq; // maximum frequency of the band
		uint16_t currentFreq; // Default frequency or current frequency
		uint16_t currentStep; // Defeult step (increment and decrement)
	} Band;

	#pragma pack(push,1)
	typedef struct {
		uint16_t freq;
		char name[16];
		//uint8_t zero;
	} fm_station_t;
	#pragma pack(pop)

	I2C_HandleTypeDef *portRADIO;

	uint16_t lcorX;

	void SI4735_RST_Clr();
	void SI4735_RST_Set();
	uint32_t _millis();
	void _delay(uint32_t x);
	void i2cWrite(uint8_t *data, size_t len, uint16_t dev_addr);
	void i2cWriteTo(uint8_t *data, size_t len, uint16_t dev_addr, uint16_t to);
	void i2cRead(uint8_t *data, size_t len, uint16_t dev_addr);
	void i2cReadFrom(uint8_t *data, size_t len, uint16_t dev_addr, uint16_t from);

	extern void SI4735_init(uint8_t defaultFunction);
#endif


	void putMsg(evt_t evt);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
