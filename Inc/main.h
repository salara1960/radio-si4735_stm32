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

#if defined(SET_OLED_SPI) || defined(SET_OLED_I2C)
	#include "ssd1306.h"
#elif defined(SET_ST_IPS)
	#include "st7789.h"
#endif

#ifdef SET_SI4735
	#include "si4735.h"
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
	devMem = 0x20
};

typedef enum {
	msg_empty = 0,
	msg_rst,
	msg_print,
	msg_100ms,
//	msg_500ms,
	msg_sec,
	msg_encCounter,
	msg_encPressed,
	msg_encReleased,
	msg_errCounter,
	msg_keyEvent,
	msg_incFrec,
	msg_decFrec,
	msg_radioStatus,
	msg_none
} evt_t;

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
#define BT1_Pin GPIO_PIN_1
#define BT1_GPIO_Port GPIOA
#define BT1_EXTI_IRQn EXTI1_IRQn
#define BT2_Pin GPIO_PIN_2
#define BT2_GPIO_Port GPIOA
#define BT2_EXTI_IRQn EXTI2_IRQn
#define BT3_Pin GPIO_PIN_3
#define BT3_GPIO_Port GPIOA
#define BT3_EXTI_IRQn EXTI3_IRQn
#define LED_ERROR_Pin GPIO_PIN_4
#define LED_ERROR_GPIO_Port GPIOA
#define OLED_CS_Pin GPIO_PIN_6
#define OLED_CS_GPIO_Port GPIOA
#define OLED_DC_Pin GPIO_PIN_7
#define OLED_DC_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define SI_RST_Pin GPIO_PIN_12
#define SI_RST_GPIO_Port GPIOB
#define SI_INT_Pin GPIO_PIN_14
#define SI_INT_GPIO_Port GPIOB
#define SI_INT_EXTI_IRQn EXTI15_10_IRQn
#define ENC_KEY_Pin GPIO_PIN_15
#define ENC_KEY_GPIO_Port GPIOA
#define ENC_KEY_EXTI_IRQn EXTI15_10_IRQn
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
#define ENC_LED_Pin GPIO_PIN_8
#define ENC_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

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
#define _15s (_1s * 15)
#define _20s (_1s * 20)
#define _25s (_1s * 25)
#define _30s (_1s * 30)


#define SET_FLOAT_PART

#define MAX_FIFO_SIZE  32
#define MAX_UART_BUF  768

#define TIME_encKeyPressed 50
#define TIME_btKeyPressed 75

#define MAX_ENC_VALUE 255

#define KEY1 1
#define KEY2 2
#define KEY3 3
#define NONE 0

//#define bMIN 0
//#define bMID 127
//#define bMAX 255


#ifdef SET_OLED_SPI
	#define CS_OLED_SELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
	#define CS_OLED_DESELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)
#endif

#ifdef SET_FLOAT_PART
	typedef struct {
		uint16_t cel;
		uint16_t dro;
	} s_float_t;
#endif

DMA_HandleTypeDef hdma_spi1_tx;

uint8_t devError;
uint32_t spiRdy;
volatile uint32_t cnt_err;

void putMsg(evt_t evt);


#ifdef SET_OLED_I2C
	I2C_HandleTypeDef *portOLED;
#else
	SPI_HandleTypeDef *portOLED;
	const FontDef *fntKey;
	const FontDef *tFont;
	const FontDef *lFont;
#endif

#ifdef SET_SI4735
	#define max_allStep 4
    #define max_rModes 3
	//
    #define minFrecFM 8400
    #define maxFrecFM 10800
	#define curFrecFM 10390
	//
	#define minFrecAM 550
    #define maxFrecAM 1750
	#define curFrecAM 810

	#define MIN_aVol   0
	#define MAX_aVol  63
    #define STEP_aVol  8

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


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
