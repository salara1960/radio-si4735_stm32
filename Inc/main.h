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
	msg_sec,
	msg_encCounter,
	msg_encPressed,
	msg_encReleased,
	msg_keyEvent,
	msg_incFrec,
	msg_decFrec,
	msg_radioStatus,
	msg_kbd,
	msg_none
} evt_t;

enum {
	NONE = 0,
	KEY1,
	KEY2,
	KEY3
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
#define BT1_Pin GPIO_PIN_1
#define BT1_GPIO_Port GPIOA
#define BT1_EXTI_IRQn EXTI1_IRQn
#define BT2_Pin GPIO_PIN_2
#define BT2_GPIO_Port GPIOA
#define BT2_EXTI_IRQn EXTI2_IRQn
#define BT3_Pin GPIO_PIN_3
#define BT3_GPIO_Port GPIOA
#define BT3_EXTI_IRQn EXTI3_IRQn
#define KBD_INT_Pin GPIO_PIN_4
#define KBD_INT_GPIO_Port GPIOA
#define KBD_INT_EXTI_IRQn EXTI4_IRQn
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
#define LED_ERROR_Pin GPIO_PIN_9
#define LED_ERROR_GPIO_Port GPIOB
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

#define TIME_encKeyPressed 50
#define TIME_btKeyPressed 75


#ifdef SET_OLED_SPI
	#define CS_OLED_SELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
	#define CS_OLED_DESELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)
#endif


typedef struct {
	uint16_t cel;
	uint16_t dro;
} s_float_t;


DMA_HandleTypeDef hdma_spi1_tx;

uint8_t devError;
uint32_t spiRdy;
volatile uint32_t cnt_err;


#ifdef SET_OLED_I2C
	I2C_HandleTypeDef *portOLED;
#else
	SPI_HandleTypeDef *portOLED;
	const FontDef *fntKey;
	const FontDef *tFont;
	const FontDef *lFont;
#endif

#ifdef SET_KBD
	#define KBD_ADDR1 0x5A
	#define KBD_ADDR2 0x5B
	#define KBD_ADDR3 0x5C
	#define KBD_ADDR4 0x5D

	#define NUM_OF_ELECTRODES	13
	//soft reset register
	#define SRST		0x80
	// touch and OOR statuses
	#define TS1		    0x00
	#define TS2 		0x01
	#define OORS1 		0x02
	#define OORS2 		0x03
	// filtered data
	#define E0FDL 		0x04
	#define E0FDH 		0x05
	#define E1FDL 		0x06
	#define E1FDH 		0x07
	#define E2FDL 		0x08
	#define E2FDH 		0x09
	#define E3FDL 		0x0A
	#define E3FDH 		0x0B
	#define E4FDL 		0x0C
	#define E4FDH 		0x0D
	#define E5FDL 		0x0E
	#define E5FDH 		0x0F
	#define E6FDL 		0x10
	#define E6FDH 		0x11
	#define E7FDL 		0x12
	#define E7FDH 		0x13
	#define E8FDL 		0x14
	#define E8FDH 		0x15
	#define E9FDL 		0x16
	#define E9FDH 		0x17
	#define E10FDL 		0x18
	#define E10FDH 		0x19
	#define E11FDL 		0x1A
	#define E11FDH 		0x1B
	#define E12FDL 		0x1C
	#define E12FDH 		0x1D
	// baseline values
	#define E0BV 		0x1E
	#define E1BV 		0x1F
	#define E2BV 		0x20
	#define E3BV 		0x21
	#define E4BV 		0x22
	#define E5BV 		0x23
	#define E6BV 		0x24
	#define E7BV 		0x25
	#define E8BV 		0x26
	#define E9BV 		0x27
	#define E10BV 		0x28
	#define E11BV 		0x29
	#define E12BV 		0x2A
	// general electrode touch sense baseline filters
	// rising filter
	#define MHDR 		0x2B
	#define NHDR 		0x2C
	#define NCLR 		0x2D
	#define FDLR 		0x2E
	// falling filter
	#define MHDF 		0x2F
	#define NHDF 		0x30
	#define NCLF 		0x31
	#define FDLF 		0x32
	// touched filter
	#define NHDT 		0x33
	#define NCLT 		0x34
	#define FDLT 		0x35
	// proximity electrode touch sense baseline filters
	// rising filter
	#define MHDPROXR 	0x36
	#define NHDPROXR 	0x37
	#define NCLPROXR 	0x38
	#define FDLPROXR 	0x39
	// falling filter
	#define MHDPROXF 	0x3A
	#define NHDPROXF 	0x3B
	#define NCLPROXF 	0x3C
	#define FDLPROXF 	0x3D
	// touched filter
	#define NHDPROXT 	0x3E
	#define NCLPROXT 	0x3F
	#define FDLPROXT 	0x40
	// electrode touch and release thresholds
	#define E0TTH 		0x41
	#define E0RTH 		0x42
	#define E1TTH 		0x43
	#define E1RTH 		0x44
	#define E2TTH 		0x45
	#define E2RTH 		0x46
	#define E3TTH 		0x47
	#define E3RTH 		0x48
	#define E4TTH 		0x49
	#define E4RTH 		0x4A
	#define E5TTH 		0x4B
	#define E5RTH 		0x4C
	#define E6TTH 		0x4D
	#define E6RTH 		0x4E
	#define E7TTH 		0x4F
	#define E7RTH 		0x50
	#define E8TTH 		0x51
	#define E8RTH 		0x52
	#define E9TTH 		0x53
	#define E9RTH 		0x54
	#define E10TTH 		0x55
	#define E10RTH 		0x56
	#define E11TTH 		0x57
	#define E11RTH 		0x58
	#define E12TTH 		0x59
	#define E12RTH 		0x5A
	//debounce settings
	#define DTR 		0x5B
	// configuration registers
	#define AFE1 		0x5C
	#define AFE2 		0x5D
	#define ECR 		0x5E
	// electrode currents
	#define CDC0		0x5F
	#define CDC1 		0x60
	#define CDC2 		0x61
	#define CDC3 		0x62
	#define CDC4 		0x63
	#define CDC5 		0x64
	#define CDC6 		0x65
	#define CDC7 		0x66
	#define CDC8 		0x67
	#define CDC9 		0x68
	#define CDC10 		0x69
	#define CDC11 		0x6A
	#define CDC12 		0x6B
	// electrode charge times
	#define CDT01 		0x6C
	#define CDT23 		0x6D
	#define CDT45 		0x6E
	#define CDT67 		0x6F
	#define CDT89 		0x70
	#define CDT1011 	0x71
	#define CDT11 		0x72
	// GPIO
	#define CTL0		0x73
	#define CTL1 		0x74
	#define DAT 		0x75
	#define DIR 		0x76
	#define EN 		    0x77
	#define SET 		0x78
	#define CLR 		0x79
	#define TOG 		0x7A
	// auto-config
	#define ACCR0		0x7B
	#define ACCR1		0x7C
	#define USL		0x7D
	#define LSL		0x7E
	#define TL		0x7F
	// soft reset
	#define SRST		0x80
	// PWM
	#define PWM0		0x81
	#define PWM1		0x82
	#define PWM2		0x83
	#define PWM3 		0x84

	#define STAR  0
	#define SEVEN 1
	#define FOUR  2
	#define ONE   3
	#define ZERO  4
	#define EIGHT 5
	#define FIVE  6
	#define TWO   7
	#define POUND 8
	#define NINE  9
	#define SIX   10
	#define THREE 11

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
	#define MIN_ELAPSED_TIME 100
	#define MIN_ELAPSED_RSSI_TIME 150
	//
	//
	//
	#define max_allStep 4
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
