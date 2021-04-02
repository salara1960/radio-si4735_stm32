/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//const char *version = "Version 0.1 (05.03.2021)";
//const char *version = "Version 0.2 (06.03.2021)";
//const char *version = "Version 0.3 (08.03.2021)";
//const char *version = "Version 0.4 (09.03.2021)";
//const char *version = "Version 0.5 (13.03.2021)";
//const char *version = "Version 0.6 (15.03.2021)";
//const char *version = "Version 0.7 (16.03.2021)";
//const char *version = "Version 0.8 (17.03.2021)";
//const char *version = "Version 0.8 (22.03.2021)";
//const char *version = "Version 0.9 (23.03.2021)";
//const char *version = "Version 1.0 (26.03.2021)";
//const char *version = "Version 1.1 (27.03.2021)";
//const char *version = "Version 1.2 (28.03.2021)";
//const char *version = "Version 1.3 (29.03.2021)";
//const char *version = "Version 1.3.1 (29.03.2021)";//fixed bug in SI4735_sendProperty(uint16_t propertyNumber, uint16_t parameter)
//const char *version = "Version 1.4 (30.03.2021)";
//const char *version = "Version 1.4.1 (30.03.2021)";
//const char *version = "Version 1.5.1 (31.03.2021)";
//const char *version = "Version 1.5.2 (31.03.2021)";// add mpr121
//const char *version = "Version 1.5.3 (01.04.2021)";
const char *version = "Version 1.6 (02.04.2021)";//temporarily remove the module KBD mpr121


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

static evt_t evt_fifo[MAX_FIFO_SIZE] = {msg_empty};
uint8_t rd_evt_adr = 0;
uint8_t wr_evt_adr = 0;
uint8_t wr_evt_err = 0;
uint8_t cnt_evt = 0;
uint8_t max_evt = 0;

//1616962770;//1615977250;//1615885520;//1615814070;//1615655630;//1615298580;//1615039137;
volatile time_t epoch = 1617362170;//1617305710;//1617097990;//1617036280;//1617015492;
uint8_t tZone = 2;
volatile uint32_t cnt_err = 0;
volatile uint8_t restart_flag = 0;
volatile static uint32_t secCounter = 0;
volatile static uint32_t HalfSecCounter = 0;
volatile static float dataADC = 0.0;
volatile static bool setDate = false;
static const char *_extDate = "epoch=";
static const char *_restart = "rst";
volatile uint32_t extDate = 0;
static char RxBuf[MAX_UART_BUF];
volatile uint8_t rx_uk;
volatile uint8_t uRxByte = 0;
uint8_t uartRdy = 1;
uint8_t devError = 0;
uint32_t tikStart = 0;

#if defined(SET_ST_IPS) || defined(SET_OLED_SPI) || defined(SET_OLED_I2C)

	#ifdef SET_OLED_I2C
		I2C_HandleTypeDef *portOLED = &hi2c2;
	#else
		SPI_HandleTypeDef *portOLED = &hspi1;
	#endif

	uint32_t spiRdy = 1;
	uint32_t spi_cnt = 0;
	char sline[128] = {0};
	uint16_t lcorX = 0;

	#ifdef SET_ST_IPS
		const FontDef *fntKey = &Font_16x26;
		const FontDef *tFont = &Font_11x18;
		const FontDef *lFont = &Font_7x10;
	#endif

#endif

bool encKeyPressed = false;
uint32_t Encoder = MIN_ENC_VALUE;
uint32_t lastEncoder = MIN_ENC_VALUE;
uint32_t encKeyCnt = 0;
uint8_t encKeyCntTmp = 0;

#ifdef SET_DBG_INFO
	bool outDebug = false;
	struct mallinfo mem_info;
#endif

#ifdef SET_STATIC_MEM
	char PrnBuf[MAX_UART_BUF] = {0};
#endif

GPIO_PinState bt1State = GPIO_PIN_SET;
GPIO_PinState bt2State = GPIO_PIN_SET;
GPIO_PinState bt3State = GPIO_PIN_SET;
GPIO_PinState encState = GPIO_PIN_SET;
uint32_t bt1tik = 0;
uint32_t bt2tik = 0;
uint32_t bt3tik = 0;
uint32_t enctik = 0;
bool keyFlag = false;
uint8_t keyNumber = NONE;

uint16_t kbdCode = 0;
bool kbdPresent = false;
bool kbdInitOk = false;
int16_t kbdAddr = 0;
bool kbdEnable = false;
volatile uint32_t kbdCnt = 0;
#ifdef SET_KBD
	I2C_HandleTypeDef *portKBD = &hi2c2;
	uint32_t kbd1tik = 0;
#endif

#ifdef SET_SI4735
	I2C_HandleTypeDef *portRADIO = &hi2c2;
	uint32_t min_wait_ms = 100;
	uint32_t max_wait_ms = 250;
	bool radioPresent = false;
	uint16_t minFrec, maxFrec;
	uint16_t curFrec = curFrecFM; // KHz
	uint16_t lastFrecFM = 0, lastFrecAM = 0, lastFrecSW = 0, lastFrecLW = 0;
	uint16_t stepFrec = 1; //KHz
	uint8_t radioMode = FMm;
	uint8_t radioModeNew = FMm;//POWER_UP_FM;
	//uint16_t minFrec = minFrecFM;
	//uint16_t maxFrec = maxFrecFM;
	const uint16_t allStep[] = {1, 10, 100, 1000};
	uint8_t stepFrecInd = 1;
	uint8_t radioSNR = 0, radioRSSI = 0, lradioRSSI = 1;
	volatile uint8_t aVol = MAX_aVol;
	volatile uint32_t radioCntInt = 0;
	//
	//
	//
	const uint8_t rcModes[] = {FMm, LSBm, USBm, AMm};
	const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM "};
	uint8_t bwIdxSSB = 2;
	const char *bandwitdthSSB[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
	uint8_t bwIdxAM = 1;
	const char *bandwitdthAM[] = {"6", "4", "3", "2", "1", "1.8", "2.5"};

	int currentBFO = 0;
	uint8_t currentBFOStep = 25;
	int curBFO = 0;
	bool bfoOn = false;
	bool ssbLoaded = false;
	//bool fmStereo = true;
	// AGC and attenuation control
	uint8_t agcIdx = 0;
	uint8_t disableAgc = 0;
	uint8_t agcNdx = 0;

	Band band[] = {
	  {FM_BAND_TYPE, 6400, 10800, 10390, 10},
	  {LW_BAND_TYPE, 100, 510, 300, 1},
	  {MW_BAND_TYPE, 520, 1720, 810, 10},
	  {SW_BAND_TYPE, 1800, 3500, 1900, 1}, // 160 meters
	  {SW_BAND_TYPE, 3500, 4500, 3700, 1}, // 80 meters
	  {SW_BAND_TYPE, 4500, 5500, 4850, 5},
	  {SW_BAND_TYPE, 5600, 6300, 6000, 5},
	  {SW_BAND_TYPE, 6800, 7800, 7200, 5}, // 40 meters
	  {SW_BAND_TYPE, 9200, 10000, 9600, 5},
	  {SW_BAND_TYPE, 10000, 11000, 10100, 1}, // 30 meters
	  {SW_BAND_TYPE, 11200, 12500, 11940, 5},
	  {SW_BAND_TYPE, 13400, 13900, 13600, 5},
	  {SW_BAND_TYPE, 14000, 14500, 14200, 1}, // 20 meters
	  {SW_BAND_TYPE, 15000, 15900, 15300, 5},
	  {SW_BAND_TYPE, 17200, 17900, 17600, 5},
	  {SW_BAND_TYPE, 18000, 18300, 18100, 1},  // 17 meters
	  {SW_BAND_TYPE, 21000, 21900, 21200, 1},  // 15 mters
	  {SW_BAND_TYPE, 24890, 26200, 24940, 1},  // 12 meters
	  {SW_BAND_TYPE, 26200, 27900, 27500, 1},  // CB band (11 meters)
	  {SW_BAND_TYPE, 28000, 30000, 28400, 1}
	}; // 10 meters

	const int lastBand = (sizeof band / sizeof(Band)) - 1;
	int bandIdx = 0;

#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
void floatPart(float val, s_float_t *part);
void errLedOn(const char *from);
void set_Date(time_t epoch);
int sec_to_str_time(uint32_t sec, char *stx);
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-------------------------------------------------------------------------------------------
void putMsg(evt_t evt)
{

	if (cnt_evt > (MAX_FIFO_SIZE - 5)) return;

	/*HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);*/

	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(SPI1_IRQn);


	if (cnt_evt >= MAX_FIFO_SIZE) {
		wr_evt_err++;
	} else {
		evt_fifo[wr_evt_adr] = evt;
		cnt_evt++;
		if (wr_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			wr_evt_adr++;
		} else  {
			wr_evt_adr = 0;
		}
		wr_evt_err = 0;
		if (cnt_evt > max_evt) max_evt = cnt_evt;
	}

	if (wr_evt_err) devError |= devFifo;
		       else devError &= ~devFifo;

	HAL_NVIC_EnableIRQ(SPI1_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	/*HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);*/
}
//-------------------------------------------------------------------------------------------
evt_t getMsg()
{
evt_t ret = msg_empty;

	/*HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);*/

	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(SPI1_IRQn);

	if (cnt_evt) {
		ret = evt_fifo[rd_evt_adr];
		if (cnt_evt) cnt_evt--;
		if (rd_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			rd_evt_adr++;
		} else {
			rd_evt_adr = 0;
		}
	}

	HAL_NVIC_EnableIRQ(SPI1_IRQn);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	/*HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);*/

	return ret;
}
//-------------------------------------------------------------------------------------------
#ifdef SET_KBD
/*
void kbdReadBuf(uint8_t from, uint8_t *buf, size_t len)
{
	HAL_I2C_Master_Transmit(portKBD, kbdAddr << 1, &from, 1, min_wait_ms);

	HAL_I2C_Master_Receive(portKBD, kbdAddr << 1, buf, len, max_wait_ms);
}
*/
//-------------------------------------------------------------------------------------------
void kbdWriteRegs(uint8_t reg, uint8_t *data, size_t len)
{
	if (HAL_I2C_Mem_Write(portKBD, kbdAddr << 1, reg, sizeof(reg), data, len, min_wait_ms) != HAL_OK) {
		devError |= devI2C;
		cnt_err++;
	} else {
		devError &= ~devI2C;
	}
}
//-------------------------------------------------------------------------------------------
/*
uint8_t kbdReadReg(uint8_t reg)
{
	uint8_t ret = 0;

	if (HAL_I2C_Mem_Read(portKBD, kbdAddr << 1, reg, 1, &ret, 1, min_wait_ms) != HAL_OK) {
		devError |= devI2C;
		cnt_err++;
	} else {
		devError &= ~devI2C;
	}

	return ret;
}
*/
//-------------------------------------------------------------------------------------------
void kbdReadRegs(uint8_t reg, uint8_t *data, size_t len)
{
	if (HAL_I2C_Mem_Read(portKBD, kbdAddr << 1, reg, 1, data, len, max_wait_ms) != HAL_OK) {
		devError |= devI2C;
		cnt_err++;
	} else {
		devError &= ~devI2C;
	}
}
//-------------------------------------------------------------------------------------------
bool KBD_getAddr(int16_t *addr)
{
	int16_t i = 0, adr = KBD_ADDR1;
	uint8_t byte = 0x63;
	for (i = 0; i < 4; i++) {
		adr += i;
		if (HAL_I2C_Mem_Write(portKBD, adr << 1, SRST, 1, &byte, 1, max_wait_ms) == HAL_OK) {
			*addr = adr;
			return true;
		}
	}
    return false;
}
//-------------------------------------------------------------------------------------------
bool kbdInit()
{
	bool success = true;
	uint8_t ec;
  	uint8_t reg_value = 0;
  	uint8_t byte;
/*
void mpr121QuickConfig(void)
{

	 mpr121_irqInit();//interrupt set
  // Section A
  // This group controls filtering when data is > baseline.
  mpr121Write(MHD_R, 0x01);//0x2B
  mpr121Write(NHD_R, 0x01);//
  mpr121Write(NCL_R, 0x00);//
  mpr121Write(FDL_R, 0x00);//

  // Section B
  // This group controls filtering when data is < baseline.
  mpr121Write(MHD_F, 0x01);
  mpr121Write(NHD_F, 0x01);
  mpr121Write(NCL_F, 0xFF);
  mpr121Write(FDL_F, 0x02);//0x32

  // Section C
  // This group sets touch and release thresholds for each electrode
  mpr121Write(ELE0_T, TOU_THRESH);
  mpr121Write(ELE0_R, REL_THRESH);
  mpr121Write(ELE1_T, TOU_THRESH);
  mpr121Write(ELE1_R, REL_THRESH);
  mpr121Write(ELE2_T, TOU_THRESH);
  mpr121Write(ELE2_R, REL_THRESH);
  mpr121Write(ELE3_T, TOU_THRESH);
  mpr121Write(ELE3_R, REL_THRESH);
  mpr121Write(ELE4_T, TOU_THRESH);
  mpr121Write(ELE4_R, REL_THRESH);
  mpr121Write(ELE5_T, TOU_THRESH);
  mpr121Write(ELE5_R, REL_THRESH);
  mpr121Write(ELE6_T, TOU_THRESH);
  mpr121Write(ELE6_R, REL_THRESH);
  mpr121Write(ELE7_T, TOU_THRESH);
  mpr121Write(ELE7_R, REL_THRESH);
  mpr121Write(ELE8_T, TOU_THRESH);
  mpr121Write(ELE8_R, REL_THRESH);
  mpr121Write(ELE9_T, TOU_THRESH);
  mpr121Write(ELE9_R, REL_THRESH);
  mpr121Write(ELE10_T, TOU_THRESH);
  mpr121Write(ELE10_R, REL_THRESH);
  mpr121Write(ELE11_T, TOU_THRESH);
  mpr121Write(ELE11_R, REL_THRESH);

  // Section D
  // Set the Filter Configuration
  // Set ESI2
  mpr121Write(FIL_CFG, 0x04);//0x5D

  // Section E
  // Electrode Configuration
  // Enable 6 Electrodes and set to run mode
  // Set ELE_CFG to 0x00 to return to standby mode
  mpr121Write(ELE_CFG, 0x0C);//0x5E	// Enables all 12 Electrodes
  //mpr121Write(ELE_CFG, 0x06);//0x5E		// Enable first 6 electrodes

  // Section F
  // Enable Auto Config and auto Reconfig
  ////mpr121Write(ATO_CFG0, 0x0B);
  ////mpr121Write(ATO_CFGU, 0xC9);	// USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   mpr121Write(ATO_CFGL, 0x82);	// LSL = 0.65*USL = 0x82 @3.3V
  ////mpr121Write(ATO_CFGT, 0xB5);	// Target = 0.9*USL = 0xB5 @3.3V

}
*/

	// soft reset
	//write_register(SRST, 0x63);
	//byte = 0x63;
	//kbdWriteRegs(SRST, &byte, 1);
	//HAL_Delay(1);
/*
	// read AFE Configuration 2
	//read_register(AFE2, &reg_value);
	kbdReadRegs(AFE2, &reg_value, 1);
	// check default value
	if (reg_value != 0x24) {
		//reg_value = 0x24;
		//kbdWriteRegs(AFE2, &reg_value, 1);
		return false;
	}
*/
	// read Touch Status register
	//read_register(TS2, &reg_value);
	kbdReadRegs(TS2, &reg_value, 1);
	if (reg_value & 0x80) {//clear OVCF
		reg_value = 0x80;
		kbdWriteRegs(TS2, &reg_value, 1);

		kbdReadRegs(TS2, &reg_value, 1);
		if (reg_value & 0x80) {
			return false;
		}
	}

	// if no previous error
	//if (success) {
		byte = 0;
		kbdWriteRegs(ECR, &byte, 1);// turn off all electrodes to stop

		uint8_t data[] = {
			1, 1, 0x10, 0x20, 1, 1, 0x10, 0x20,
			1, 0x10, 0xFF, 0x0F, 0x0F, 0, 0, 1,
			1, 0xFF, 0xFF, 0, 0, 0
		};
		kbdWriteRegs(MHDR, data, sizeof(data));//0x2B ... 0x40

		data[0] = 0x11;
		data[1] = 0xD0;//0x5C - 16uA
		data[2] = 0x14;//0x5D - Period set to 16 ms (Default)
		kbdWriteRegs(DTR, data, 3);//0x5B ... 0x5D

		memset(data, 0, 5);
		kbdWriteRegs(ACCR0, data, 5);//0x7B ... 0x7F

		byte = 0xCC;
		kbdWriteRegs(ECR, &byte, 1);

		// apply next setting for all electrodes
		data[0] = 40;//5..0x30
		data[1] = 20;
		byte = E0TTH;
		for (ec = 0; ec < NUM_OF_ELECTRODES; ec++) {
			kbdWriteRegs(byte, data, 2);
			byte += 2;
		}

		byte = 0x10;
		kbdWriteRegs(ECR, &byte, 1);// enable electrodes and set the current to 16uA
	//}

	return success;
}
//-------------------------------------------------------------------------------------------
uint16_t kbd_get_touch()// get touch status
{
	uint8_t data[2] = {0};
	kbdReadRegs(TS1, data, 2);
	/*if (data[1] & 0x80) {
		uint8_t byte = 0x80;
		kbdWriteRegs(TS2, &byte, 1);
	}*/
	uint16_t ret = data[1];
	ret <<= 8;
	ret |= data[0];

	return ret;

/*
	//int tn = 0;
	//for (int j = 0; j < NUM_OF_ELECTRODES - 1; j++) if ((ret & (1 << j))) tn++;

	uint16_t key = 0;
	//if (tn == 1) {
		     if (ret & (1 << STAR))  key = 0x2a;//'*';
		else if (ret & (1 << SEVEN)) key = 0x37;//'7';
		else if (ret & (1 << FOUR))  key = 0x34;//'4';
		else if (ret & (1 << ONE))   key = 0x31;//'1';
		else if (ret & (1 << ZERO))  key = 0x30;//'0';
		else if (ret & (1 << EIGHT)) key = 0x38;//'8';
		else if (ret & (1 << FIVE))  key = 0x35;//'5';
		else if (ret & (1 << TWO))   key = 0x32;//'2';
		else if (ret & (1 << POUND)) key = 0x23;//'#';
		else if (ret & (1 << NINE))  key = 0x39;//'9';
		else if (ret & (1 << SIX))   key = 0x36;//'6';
		else if (ret & (1 << THREE)) key = 0x33;//'3';
	//}

	return key;
*/
}

/*
char getPhoneNumber()
{
  int touchNumber;
	int j;
  uint16_t touchstatus;
	char key=-1;
  //Serial.println("Please Enter a phone number...");

    //while(key_pressed);//用while读取会阻塞程序运行
	if(key_pressed==0)//非阻塞方式
	{
	key_pressed=1;
    touchNumber = 0;

    touchstatus = mpr121Read(0x01) << 8;
    touchstatus |= mpr121Read(0x00);

    for (j=0; j<12; j++)  // Check how many electrodes were pressed
    {
      if ((touchstatus & (1<<j)))
        touchNumber++;
    }

    if (touchNumber == 1)
    {
      if (touchstatus & (1<<STAR))
        key = '*';
      else if (touchstatus & (1<<SEVEN))
        key = '7';
      else if (touchstatus & (1<<FOUR))
        key= '4';
      else if (touchstatus & (1<<ONE))
        key = '1';
      else if (touchstatus & (1<<ZERO))
        key= '0';
      else if (touchstatus & (1<<EIGHT))
        key = '8';
      else if (touchstatus & (1<<FIVE))
        key = '5';
      else if (touchstatus & (1<<TWO))
        key = '2';
      else if (touchstatus & (1<<POUND))
        key = '#';
      else if (touchstatus & (1<<NINE))
        key = '9';
      else if (touchstatus & (1<<SIX))
        key = '6';
      else if (touchstatus & (1<<THREE))
        key = '3';

      //Serial.print(key[i]);

    }
    else if (touchNumber == 0);
    else;
      //Serial.println("Only touch ONE button!");
	}
		return key;
}
*/

#endif
//-------------------------------------------------------------------------------------------
#ifdef SET_SI4735
	void SI4735_RST_Clr()
	{
		HAL_GPIO_WritePin(SI_RST_GPIO_Port, SI_RST_Pin, GPIO_PIN_RESET);
	}
	void SI4735_RST_Set()
	{
		HAL_GPIO_WritePin(SI_RST_GPIO_Port, SI_RST_Pin, GPIO_PIN_SET);
	}
	uint32_t _millis()
	{
		return HAL_GetTick();
	}
	void _delay(uint32_t x)
	{
		HAL_Delay(x);
	}
	void i2cWrite(uint8_t *data, size_t len, uint16_t dev_addr)
	{
		if (HAL_I2C_Master_Transmit(portRADIO, dev_addr << 1, data, len, max_wait_ms) != HAL_OK) {
			devError |= devI2C;
			cnt_err++;
			//putMsg(msg_errCounter);
		} else {
			devError &= ~devI2C;
		}
	}
	void i2cWriteTo(uint8_t *data, size_t len, uint16_t dev_addr, uint16_t to)
	{
		if (HAL_I2C_Mem_Write(portRADIO, dev_addr << 1, to, 1, data, len, max_wait_ms) != HAL_OK) {
			devError |= devI2C;
			cnt_err++;
			//putMsg(msg_errCounter);
		} else {
			devError &= ~devI2C;
		}
	}
	void i2cRead(uint8_t *data, size_t len, uint16_t dev_addr)
	{
		if (HAL_I2C_Master_Receive(portRADIO, dev_addr << 1, data, len, max_wait_ms) != HAL_OK) {
			devError |= devI2C;
			cnt_err++;
			//putMsg(msg_errCounter);
		} else {
			devError &= ~devI2C;
		}
	}
	void i2cReadFrom(uint8_t *data, size_t len, uint16_t dev_addr, uint16_t from)
	{
		if (HAL_I2C_Mem_Read(portRADIO, dev_addr << 1, from, 1, data, len, max_wait_ms) != HAL_OK) {
			devError |= devI2C;
			cnt_err++;
			//putMsg(msg_errCounter);
		} else {
			devError &= ~devI2C;
		}
	}
	bool devReady(uint16_t dev_addr)
	{
		if (HAL_I2C_IsDeviceReady(portRADIO, dev_addr << 1, 3, 1000) == HAL_OK)
			return true;
		else
			return false;
	}
	//
	void useBand()
	{
		if (band[bandIdx].bandType == FM_BAND_TYPE) {
			radioMode = FMm;
			SI4735_setTuneFrequencyAntennaCapacitor(0);
			SI4735_setFM1(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
			bfoOn = ssbLoaded = false;
		} else {
			if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE)
				SI4735_setTuneFrequencyAntennaCapacitor(0);
			else
				SI4735_setTuneFrequencyAntennaCapacitor(1);
			if (ssbLoaded) {
				SI4735_setSSB1(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, radioMode);
				SI4735_setSSBAutomaticVolumeControl(1);
			} else {
				radioMode = AMm;
				SI4735_setAM1(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
				bfoOn = false;
			}
			//SI4735_setAmSoftMuteMaxAttenuation(0);
			SI4735_sendProperty(AM_SOFT_MUTE_MAX_ATTENUATION, 0);
			SI4735_setAutomaticGainControl(disableAgc, agcNdx);
		}
		_delay(100);
		curFrec = band[bandIdx].currentFreq;
		stepFrec = band[bandIdx].currentStep;
		minFrec = band[bandIdx].minimumFreq;
		maxFrec = band[bandIdx].maximumFreq;

		//showStatus();
	}
	void bandUp()
	{
		// save the current frequency for the band
		band[bandIdx].currentFreq = curFrec;
		band[bandIdx].currentStep = stepFrec;
		if (bandIdx < lastBand) bandIdx++;
				           else bandIdx = 0;
		useBand();
	}
	void bandDown()
	{
		// save the current frequency for the band
		band[bandIdx].currentFreq = curFrec;
		band[bandIdx].currentStep = stepFrec;
		if (bandIdx > 0) bandIdx--;
					else bandIdx = lastBand;
		useBand();
	}
	void modeSwitchButton()
	{
		if (radioMode != FMm) {
			if (radioMode == AMm) {
				// If you were in AM mode, it is necessary to load SSB patch (avery time)
				SI4735_loadSSB(bwIdxSSB);
				radioMode = LSBm;
			} else if (radioMode == LSBm) {
				radioMode = USBm;
			} else if (radioMode == USBm) {
				radioMode = AMm;
				ssbLoaded = false;
				bfoOn = false;
			}
			// Nothing to do if you are in FM mode
			band[bandIdx].currentFreq = curFrec;
			band[bandIdx].currentStep = stepFrec;
			useBand();
		}
	}
	void bandwitdthButton()
	{
		uint8_t yes = 1;

		if (radioMode == LSBm || radioMode == USBm) {
			bwIdxSSB++;
			if (bwIdxSSB > 5) bwIdxSSB = 0;
			SI4735_setSSBAudioBandwidth(bwIdxSSB);
			// If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
			if (bwIdxSSB == 0 || bwIdxSSB == 4 || bwIdxSSB == 5)
				SI4735_setSBBSidebandCutoffFilter(0);
			else
				SI4735_setSBBSidebandCutoffFilter(1);
		} else if (radioMode == AMm) {
			bwIdxAM++;
			if (bwIdxAM > 6) bwIdxAM = 0;
			SI4735_setBandwidth(bwIdxAM, 1);
		} else yes = 0;
		//resetBuffer();
		//showStatus();
		if (yes) _delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
	}
	void bfoSwitchButton()
	{
		if (radioMode == LSBm || radioMode == USBm) {
			bfoOn = !bfoOn;
			//if (bfoOn) showBFO();
			//showStatus();
			_delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
		}
		//showFrequency();
	}
	void stepButton()
	{
		// This command should work only for SSB mode
		if (bfoOn && (radioMode == LSBm || radioMode == USBm)) {
			currentBFOStep = (currentBFOStep == 25) ? 10 : 25;
			//showBFO();
		} else {
			if (radioMode != FMm) {
				if (stepFrec == 1) currentStep = 5;
				else if (stepFrec == 5) stepFrec = 10;
				else if (stepFrec == 10) stepFrec = 50;
				else stepFrec = 1;
			} else {
				if (stepFrec == 10) stepFrec = 100;
				               else stepFrec = 10;
			}
			SI4735_setFrequencyStep(stepFrec);
			band[bandIdx].currentStep = stepFrec;
			//showStatus();
			_delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
		}

	}
	void attenuationButton()
	{
		switch(agcIdx) {
			case 0:
				disableAgc = 0; // Turns AGC ON
				agcNdx = 0;
				agcIdx = 1;
				break;
			case 1:
				disableAgc = 1; // Turns AGC OFF
				agcNdx = 0;     // Sets minimum attenuation
				agcIdx = 2;
				break;
			case 2:
				disableAgc = 1; // Turns AGC OFF
				agcNdx = 10;    // Increases the attenuation AM/SSB AGC Index  = 10
				agcIdx = 3;
				break;
			case 3:
				disableAgc = 1; // Turns AGC OFF
				agcNdx = 15;    // Increases the attenuation AM/SSB AGC Index  = 30
				agcIdx = 4;
				break;
			case 4:
				disableAgc = 1; // Turns AGC OFF
				agcNdx = 25;    // Increases the attenuation AM/SSB AGC Index  = 30
				agcIdx = 5;
				break;
			case 5:
				disableAgc = 1; // Turns AGC OFF
				agcNdx = 35;    // Increases the attenuation AM/SSB AGC Index  = 30
				agcIdx = 0;
				break;
		}
		// Sets AGC on/off and gain
		SI4735_setAutomaticGainControl(disableAgc, agcNdx);
	    //showStatus();
	}
	void volumeButton(uint8_t d)
	{
		if (d == 1) SI4735_volumeUp();
		       else SI4735_volumeDown();
		aVol = SI4735_getVolume();
		//showVolume();
		_delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
	}

#endif
//-------------------------------------------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


  	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(ENC_LED_GPIO_Port, ENC_LED_Pin, GPIO_PIN_SET);
  	HAL_Delay(250);
  	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  	HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
  	HAL_GPIO_TogglePin(ENC_LED_GPIO_Port, ENC_LED_Pin);


  	HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    //start ADC1 + interrupt
    HAL_ADC_Start_IT(&hadc1);

    //"start" rx_interrupt
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uRxByte, 1);


  	char buf[MAX_UART_BUF] = {0};
  	s_float_t vcc = {0, 0};
  	set_Date((time_t)(++epoch));
    uint32_t pack_num = 0;

#if defined(SET_ST_IPS) || defined(SET_OLED_SPI) || defined(SET_OLED_I2C)
	#ifdef SET_OLED_I2C
		portOLED = &hi2c2;
	#else
		portOLED = &hspi1;
	#endif
#endif

#if defined(SET_ST_IPS)

  	ST7789_Reset();
  	ST7789_Init();

  	ST7789_Fill_Color(invColor(BLUE));

  	area_t recta = {
  		.x1 = 0,
		.y1 = (fntKey->height << 1) + 1,
		.x2 = ST7789_WIDTH - 1,
		.y2 = ST7789_HEIGHT - 1,
		.bcolor = invColor(YELLOW),
		.duga = &arrScr[0],
  	};
  	mkFace(&recta);

  	area_t pbar = {
  		.x1 = recta.x1 + 2,
		.y1 = recta.y1 + 2,
		.x2 = recta.x2 - 2,
		.y2 = recta.y1 + fntKey->height + 5,
		.bcolor = invColor(MAGENTA),
		.duga = NULL,
  	};
  	ST7789_DrawRectangle(pbar.x1, pbar.y1, pbar.x2, pbar.y2, pbar.bcolor);


#elif defined(SET_OLED_SPI)
  	spi_ssd1306_Reset();
  	//spi_ssd1306_on(1);//screen ON
  	spi_ssd1306_init();//screen INIT
  	spi_ssd1306_pattern();//set any params for screen
  	//spi_ssd1306_invert();
  	spi_ssd1306_clear();//clear screen
#elif defined(SET_OLED_I2C)
  	//i2c_ssd1306_on(true);
  	i2c_ssd1306_init();
  	i2c_ssd1306_pattern();
  	//i2c_ssd1306_invert();
    i2c_ssd1306_clear();
#endif


#ifdef SET_KBD
    kbdPresent = KBD_getAddr(&kbdAddr);
    if (kbdPresent) {
    	kbdInitOk = kbdInit();
    	HAL_Delay(10);
    }
#endif


#ifdef SET_SI4735


    radioPresent = devReady(deviceAddress);
    if (radioPresent) {

    	SI4735_init(radioMode);

    	HAL_Delay(300);
    	// Set up the radio for the current band (see index table variable bandIdx )
    	useBand();
    	curFrec = SI4735_getFrequency();
    	SI4735_setVolume(aVol);
    	//showStatus();

    	SI4735_getFirmware();

	#if defined(SET_ST_IPS)
    	updateBar(&pbar, aVol);
    	//
    	//HAL_Delay(1);
    	//
    	int dl = sprintf(sline, "volume: %u", aVol);
    	ST7789_WriteString(4 + (((ST7789_WIDTH / tFont->width) - dl) >> 1) * tFont->width,
    			pbar.y1 + 6,
				sline,
				*tFont,
				invColor(BLACK),
				invColor(WHITE));
    	//
	#endif
    }

#endif

  	Report(NULL, true, "Version '%s'\nkbdPresent=%d kbdInit=%d kbdAddr=0x%02X\nsi4735:\n\tSTAT=0x%02x:\n\t\tCTS:%u ERR=%u DUMMY2=%u RSQINT=%u RDSINT=%u DUMMY1=%u STCINT=%u\
  			\n\tPN=0x%02x\n\tFW=%c%c\n\tPATCH=0x%02x%02x\n\tCMP=%c%c\n\tCHIP=%c\n",
  			version,
			kbdPresent, kbdInitOk, kbdAddr,
			firmwareInfo.raw[0],
			firmwareInfo.resp.CTS,    firmwareInfo.resp.ERR,    firmwareInfo.resp.DUMMY2,
			firmwareInfo.resp.RSQINT, firmwareInfo.resp.RDSINT, firmwareInfo.resp.DUMMY1, firmwareInfo.resp.STCINT,
			firmwareInfo.raw[1],
			firmwareInfo.raw[2], firmwareInfo.raw[3],
			firmwareInfo.raw[4], firmwareInfo.raw[5],
			firmwareInfo.raw[6], firmwareInfo.raw[7],
			firmwareInfo.raw[8]);

  	// start encoder's channels
  	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  	// start timer2 in interrupt mode
  	HAL_TIM_Base_Start_IT(&htim2);

  	lastEncoder = Encoder = TIM4->CNT;
  	putMsg(msg_encCounter);

    /* Infinite loop */

  	#ifdef SET_KBD
  	if (kbdPresent && kbdInitOk) {
  		kbdEnable = true;
  		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  	}
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	//LOOP_FOREVER();

  	evt_t evt = msg_none;
  	while (1) {
	  //
  		evt = getMsg();
  		switch ((int)evt) {
	  	  	//
  			case msg_kbd:
#ifdef SET_KBD
  				kbdCnt++;
  				if (kbdEnable) {
  					//HAL_GPIO_TogglePin(ENC_LED_GPIO_Port, ENC_LED_Pin);
  					kbdCode = kbd_get_touch();
  				}
#endif
  			break;
  			//
  			case msg_incFrec:
  				if ((curFrec + stepFrec) <= maxFrec) {
  					SI4735_frequencyUp();//+stepFrec
  					curFrec += stepFrec;
  					putMsg(msg_encCounter);
  				}
  			break;
  			case msg_decFrec:
  				if ((curFrec - stepFrec) >= minFrec) {
  					SI4735_frequencyDown();//-stepFrec
  					curFrec -= stepFrec;
  					putMsg(msg_encCounter);
  				}
  			break;
  			case msg_encCounter:
  			case msg_encPressed:
  			case msg_encReleased:
  				/*
  				if (bfoOn) {
  					currentBFO = (Encoder == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
  				    SI4735_setSSBBfo(currentBFO);
  				    //showBFO();
  				} else {
  					//if (Encoder == 1) SI4735_frequencyUp();
  				    //             else SI4735_frequencyDown();
  				    // Show the current frequency only if it has changed
  					//curFrec = SI4735_getFrequency();
  					//showFrequency();
  				}
  				//Encoder = 0;*/

  				//
  				if (radioMode == FMm) {
  					lastFrecFM = curFrec;
  					sprintf(sline, " %u.%02u MHz  %u KHz ", curFrec / 100, curFrec % 100, stepFrec);
  				} else {
  					sprintf(sline, " %u KHz  %u KHz ", curFrec, stepFrec);
  					//if (radioMode == POWER_UP_SW) lastFrecSW = curFrec;
  					//else
  					if (radioMode == LW) lastFrecLW = curFrec;
  					else if (radioMode == AMm) lastFrecAM = curFrec;
  				}
#if defined(SET_ST_IPS)
  				ST7789_WriteString(2, fntKey->height + 1, mkLineCenter(sline, tFont->width), *tFont, invColor(GREEN), invColor(BLUE));
#elif defined(SET_OLED_I2C)
  				i2c_ssd1306_text_xy(sline, 1, 3);
#elif defined(SET_OLED_SPI)
  				spi_ssd1306_text_xy(sline, 1, 3);
#endif
  			break;
  			case msg_sec:
  				sec_to_str_time(get_tmr(0), buf);
#if defined(SET_ST_IPS)
  				ST7789_WriteString(4, 0, buf, *fntKey, invColor(YELLOW), invColor(BLUE));
#elif defined(SET_OLED_I2C)
  				i2c_ssd1306_text_xy(buf, 2, 1);
#elif defined(SET_OLED_SPI)
  				spi_ssd1306_text_xy(buf, 2, 1);
#endif
  				sprintf(buf+strlen(buf), " [%lu] devError(%lu): 0x%02X, Fifo: %d/%d, KBD: cnt=%lu code=%04X, Radio: mode=%s Frec=",
  					                   ++pack_num, cnt_err, devError,
									   (int)cnt_evt, (int)max_evt,
									   kbdCnt, kbdCode,
									   bandModeDesc[radioMode & 3]);
  				if (radioMode == FMm)
  					sprintf(buf+strlen(buf), "%u.%02u (%u.%u-%u.%u) MHz",
  							curFrec / 100, curFrec % 100,
							minFrec / 100, minFrec % 100,
							maxFrec / 100, maxFrec % 100);
  				else
  					sprintf(buf+strlen(buf), "%u (%u-%u) KHz", curFrec, minFrec, maxFrec);
  				floatPart(dataADC, &vcc);
  				sprintf(buf+strlen(buf), " Volume=%u, Encoder: Pressed=%lu Counter=%lu, Volt:%u.%u\n",
										aVol,
										encKeyCnt, Encoder,
										vcc.cel, vcc.dro);
#ifdef SET_DBG_INFO
  				if (outDebug) {
  					mem_info = mallinfo();
  					sprintf(buf+strlen(buf), "(noused=%d #freeChunk=%d #freeBlk=%d #mapReg=%d busy=%d max_busy=%d #freedBlk=%d used=%d free=%d top=%d)\n",
  						                   mem_info.arena,// Non-mmapped space allocated (bytes)
										   mem_info.ordblks,// Number of free chunks
										   mem_info.smblks,// Number of free fastbin blocks
										   mem_info.hblks,// Number of mmapped regions
										   mem_info.hblkhd,// Space allocated in mmapped regions (bytes)
										   mem_info.usmblks,// Maximum total allocated space (bytes)
										   mem_info.fsmblks,// Space in freed fastbin blocks (bytes)
										   mem_info.uordblks,// Total allocated space (bytes)
										   mem_info.fordblks,// Total free space (bytes)
										   mem_info.keepcost);// Top-most, releasable space (bytes)
  				}
#endif
  				Report(NULL, false, "%s", buf);//putMsg(msg_print);

  				if (devError) errLedOn(NULL);

  				putMsg(msg_radioStatus);
  			break;
  			case msg_keyEvent:
  				switch (keyNumber) {
  					case KEY1:
  						radioMode = radioModeNew;
  						modeSwitchButton();
  						bandwitdthButton();
  					break;
  					case KEY2:
  						if (radioMode == FMm) {
  							curFrec = SI4735_seekNextStation();
  						} else {
  							bandUp();
  						}
  					break;
  					case KEY3:
  						if (radioMode == FMm) {
  							curFrec = SI4735_seekPreviousStation();
  						} else {
  							bandDown();
  						}
  					break;
  				}
  				if (keyNumber != NONE) {
  					HAL_Delay(2);
  					putMsg(msg_encCounter);
  				}
  				keyNumber = NONE;
  			break;
  			case msg_radioStatus:
  				//SI4735_getStatus1(0, 1);
  				SI4735_getCurrentReceivedSignalQuality1(0);
  				radioSNR = SI4735_getCurrentSNR();
  				radioRSSI = SI4735_getCurrentRSSI();
				#ifdef SET_ST_IPS
  					sprintf(sline, " %s SNR:", bandModeDesc[radioMode]);
  					if (radioSNR < 10) strcat(sline, " ");
  					sprintf(sline+strlen(sline), "%u RSSI:", radioSNR);
  					if (radioRSSI < 10) strcat(sline, " ");
  					sprintf(sline+strlen(sline), "%u ", radioRSSI);
  					ST7789_WriteString(4,
									(fntKey->height * 4) - (fntKey->height >> 1) + 4,//pbar.y1 + 4,
									mkLineCenter(sline, tFont->width),
									*tFont,
									invColor(GREEN),
									invColor(BLUE));
				#endif
  				//HAL_Delay(1);
  			break;
  			case msg_print:
  				if (Report(NULL, false, "%s", buf)) putMsg(msg_print);
  			break;
  			case msg_rst:
  				HAL_Delay(100);
  				Report(NULL, true, "Restart...\n");
  				HAL_Delay(1000);
  				NVIC_SystemReset();
  			break;
  		}

  		//
  	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = MIN_ENC_VALUE;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = ENC_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|ENC_LED_Pin|LED_ERROR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SI_RST_Pin|OLED_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BT1_Pin BT2_Pin BT3_Pin ENC_KEY_Pin */
  GPIO_InitStruct.Pin = BT1_Pin|BT2_Pin|BT3_Pin|ENC_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KBD_INT_Pin */
  GPIO_InitStruct.Pin = KBD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KBD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin SI_RST_Pin OLED_RST_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|SI_RST_Pin|OLED_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SI_INT_Pin */
  GPIO_InitStruct.Pin = SI_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SI_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_LED_Pin */
  GPIO_InitStruct.Pin = ENC_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ENC_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_ERROR_Pin */
  GPIO_InitStruct.Pin = LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ERROR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//*************************************************************************************************

//-----------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//LED OFF
	HAL_Delay(20);
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);//LED ON

	if (from) Report(NULL, true, "Error in function '%s'\r\n", from);
}
//------------------------------------------------------------------------------------------
void set_Date(time_t epoch)
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;

	gmtime_r(&epoch, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
	else {
		sTime.Hours   = ts.tm_hour + tZone;
		sTime.Minutes = ts.tm_min;
		sTime.Seconds = ts.tm_sec;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
		else {
			setDate = true;
		}
	}
}
//------------------------------------------------------------------------------------------
uint32_t get_Date()
{
	if (!setDate) return get_tmr(0);

	struct tm ts;

	RTC_TimeTypeDef sTime = {0};
	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
	ts.tm_hour = sTime.Hours;
	ts.tm_min  = sTime.Minutes;
	ts.tm_sec  = sTime.Seconds;

	RTC_DateTypeDef sDate = {0};
	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
	ts.tm_wday = sDate.WeekDay;
	ts.tm_mon  = sDate.Month - 1;
	ts.tm_mday = sDate.Date;
	ts.tm_year = sDate.Year;

	return ((uint32_t)mktime(&ts));
}
//-----------------------------------------------------------------------------
int sec_to_str_time(uint32_t sec, char *stx)
{
int ret = 0;

	if (!setDate) {//no valid date in RTC
		uint32_t day = sec / (60 * 60 * 24);
		sec %= (60 * 60 * 24);
		uint32_t hour = sec / (60 * 60);
		sec %= (60 * 60);
		uint32_t min = sec / (60);
		sec %= 60;
		ret = sprintf(stx, "%lu.%02lu:%02lu:%02lu", day, hour, min, sec);
	} else {//in RTC valid date (epoch time)
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
			else {
				ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u",
								   sDate.Date, sDate.Month,
								   sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
	}

	return ret;
}
//-----------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return secCounter;
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------
uint64_t get_hsCounter()
{
	return HalfSecCounter;
}
//-----------------------------------------------------------------------------
void inc_hsCounter()
{
	HalfSecCounter++;
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//------------------------------------------------------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//------------------------------------------------------------------------------------------
uint64_t get_hstmr(uint64_t hs)
{
	return (get_hsCounter() + hs);
}
//------------------------------------------------------------------------------------------
bool check_hstmr(uint64_t hs)
{
	return (get_hsCounter() >= hs ? true : false);
}
//----------------------------------------------------------------------------------------
int sec_to_string(uint32_t sec, char *stx)
{
int ret = 0;

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
	else {
		if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
		else {
			ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u ",
							sDate.Date, sDate.Month,
							sTime.Hours, sTime.Minutes, sTime.Seconds);
		}
	}

    return ret;
}
//------------------------------------------------------------------------------------------
void floatPart(float val, s_float_t *part)
{
	part->cel = (uint16_t)val;
	part->dro = (val - part->cel) * 1000;
}
//------------------------------------------------------------------------------------------
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...)
{
#ifndef SET_STATIC_MEM
	HAL_StatusTypeDef er = HAL_OK;
#endif
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;


	if (!uartRdy) {
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//ON err_led
		return 1;
	} else {
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);//OFF err_led
	}

#ifdef SET_STATIC_MEM
	char *buff = &PrnBuf[0];
	buff[0] = 0;
#else
	char *buff = (char *)calloc(1, len);
	if (buff) {
#endif
		if (addTime) {
			uint32_t ep;
			if (!setDate) ep = get_secCounter();
					 else ep = extDate;
			dl = sec_to_string(ep, buff);
		}
		if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);
		va_start(args, fmt);
		vsnprintf(buff + dl, len - dl, fmt, args);
		uartRdy = 0;
		//er = HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
		if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buff, strlen(buff)) != HAL_OK) devError |= devUART;
		//while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) {
		//	if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_RX) break;
		//	HAL_Delay(1);
		//}
		//
		va_end(args);
#ifndef SET_STATIC_MEM
		free(buff);
	} else er = HAL_ERROR;
#endif

	if (er != HAL_OK) errLedOn(NULL);

	return 0;
}
//------------------------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		dataADC = ((float)HAL_ADC_GetValue(hadc)) * 3.3 / 4096;
	}
}
//------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		uartRdy = 1;
	}
}
//------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		RxBuf[rx_uk & 0xff] = (char)uRxByte;
		if (uRxByte == 0x0a) {//end of line
			char *uk = strstr(RxBuf, _extDate);//const char *_extDate = "epoch=";
			if (uk) {
				uk += strlen(_extDate);
				if (*uk != '?') {
					if (strlen(uk) < 10) setDate = false;
					else {
						char *uke = strchr(uk, ':');
						if (uke) {
							tZone = atoi(uke + 1);
							*uke = '\0';
						} else tZone = 0;
						extDate = atol(uk);
						set_Date((time_t)extDate);
					}
				} else setDate = true;
			} else {
				if (strstr(RxBuf, _restart)) {//const char *_restart = "restart";
					if (!restart_flag) {
						restart_flag = 1;
						putMsg(msg_rst);
					}
				} else
#ifdef SET_DBG_INFO
					if (strstr(RxBuf, "dbg_on")) {//key_3
					outDebug = 1;
				} else if (strstr(RxBuf, "dbg_off")) {//key_4
					outDebug = 0;
				} else
#endif
					if (strstr(RxBuf, "clr")) {
					devError = 0;
					HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
				} else if (strstr(RxBuf, "CLR")) {
					devError = 0;
					cnt_err = 0;
					HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
					//putMsg(msg_errCounter);
				}
#ifdef SET_KBD
				else if (strstr(RxBuf, "kbd")) {
					kbdEnable = true;
				}
#endif
			}
			rx_uk = 0;
			memset(RxBuf, 0, sizeof(RxBuf));
		} else rx_uk++;

		HAL_UART_Receive_IT(huart, (uint8_t *)&uRxByte, 1);
	}
}
//-------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {//one call in _10ms

		HalfSecCounter++;//+10ms

		if (!(HalfSecCounter % _350ms)) {
			Encoder = (TIM4->CNT) >> 1;
			if (lastEncoder != Encoder) {
				evt_t ev = msg_none;
				if ((lastEncoder== MIN_ENC_VALUE) && (Encoder == MAX_ENC_VALUE)) ev = msg_decFrec;//dec
				else
				if ((lastEncoder == MAX_ENC_VALUE) && (Encoder == MIN_ENC_VALUE)) ev = msg_incFrec;//inc
				else
				if (lastEncoder < Encoder) ev = msg_incFrec;//inc
				else
				if (lastEncoder > Encoder) ev = msg_decFrec;//dec
				lastEncoder = Encoder;
				if (ev != msg_none)	putMsg(ev);
			}
		}

		if (!(HalfSecCounter % _1s)) {//seconda
			secCounter++;
			//HalfSecCounter = 0;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);//set ON/OFF LED1
			putMsg(msg_sec);
		}

		/*uint8_t yes = 0;
		if (bt3tik) {
			if ((HAL_GetTick() - bt3tik) > 1000) {
				bt3tik = 0;//HAL_GetTick();
				if (aVol > STEP_aVol) aVol -= STEP_aVol; else aVol = MIN_aVol + 1;
				keyNumber = KEY3;
				yes = 1;
			}
		}
		if (bt2tik) {
			if ((HAL_GetTick() - bt2tik) > 1000) {
				bt2tik = 0;//HAL_GetTick();
				if (aVol <= (MAX_aVol - STEP_aVol)) aVol += STEP_aVol; else aVol = MAX_aVol - 1;
				keyNumber = KEY2;
				yes = 1;
			}
		}
		if (yes) putMsg(msg_keyEvent);*/
	}
}
//-------------------------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {
#if defined(SET_ST_IPS)
		#ifdef SET_WITH_CS
			ST7789_SelOFF();
		#endif
		spi_cnt++;
		spiRdy = 1;
#elif defined(SET_OLED_SPI)
		spi_cnt++;
		spiRdy = 1;
#endif
	}
}
//-------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	keyFlag = false;

	if (GPIO_Pin == SI_INT_Pin) {//пока не используется : this interrupt disable now
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		HAL_GPIO_TogglePin(ENC_LED_GPIO_Port, ENC_LED_Pin);
		radioCntInt++;
		//
	} else if (GPIO_Pin == ENC_KEY_Pin) {
		encState = HAL_GPIO_ReadPin(ENC_KEY_GPIO_Port, ENC_KEY_Pin);
		if (encState == GPIO_PIN_SET) {//released key
			if (tikStart) {
				if ((HAL_GetTick() - tikStart) > TIME_encKeyPressed) {
					tikStart = 0;
					encKeyPressed = false;
					encKeyCnt++;
					//
					stepFrecInd++;
					if (stepFrecInd >= max_allStep) stepFrecInd = 0;
					stepFrec = allStep[stepFrecInd];
					SI4735_setStep(stepFrec);//currentStep = stepFrec;
					//
					putMsg(msg_encReleased);
				}
			}
		} else {//pressed key
			encKeyPressed = true;
			if (!tikStart) tikStart = HAL_GetTick();
		}
		HAL_GPIO_WritePin(ENC_LED_GPIO_Port, ENC_LED_Pin, encKeyPressed);
	} else if (GPIO_Pin == BT1_Pin) {
		bt1State = HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin);
		if (bt1State == GPIO_PIN_RESET) {//pressed key
			if (!bt1tik) bt1tik = HAL_GetTick();
		} else {//released key
			if (bt1tik) {
				if ((HAL_GetTick() - bt1tik) > TIME_btKeyPressed) {
					bt1tik = 0;
					keyFlag = true;
					keyNumber = KEY1;

					//bandIdx++;
					//if (bandIdx > lastBand) bandIdx = 0;
					//radioModeNew = bandIdx;
					radioMode = (radioMode + 1) & 3;
					radioModeNew = rcModes[radioMode];
				}
			}
		}
		HAL_GPIO_WritePin(ENC_LED_GPIO_Port, ENC_LED_Pin, !bt1State);
	} else if (GPIO_Pin == BT2_Pin) {
		bt2State = HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin);
		if (bt2State == GPIO_PIN_RESET) {//pressed key
			if (!bt2tik) bt2tik = HAL_GetTick();
		} else {//released key
			if (bt2tik) {
				if ((HAL_GetTick() - bt2tik) > TIME_btKeyPressed) {
					bt2tik = 0;
					keyFlag = true;
					keyNumber = KEY2;
				}
			}
		}
		HAL_GPIO_WritePin(ENC_LED_GPIO_Port, ENC_LED_Pin, !bt2State);
	} else if (GPIO_Pin == BT3_Pin) {
		bt3State = HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin);
		if (bt3State == GPIO_PIN_RESET) {//pressed key
			if (!bt3tik) bt3tik = HAL_GetTick();
		} else {//released key
			if (bt3tik) {
				if ((HAL_GetTick() - bt3tik) > TIME_btKeyPressed) {
					bt3tik = 0;
					keyFlag = true;
					keyNumber = KEY3;
				}
			}
		}
		HAL_GPIO_WritePin(ENC_LED_GPIO_Port, ENC_LED_Pin, !bt3State);
	}
#ifdef SET_KBD
	else if (GPIO_Pin == KBD_INT_Pin) {
		if (kbdEnable) {
			//HAL_GPIO_TogglePin(ENC_LED_GPIO_Port, ENC_LED_Pin);
			if ((HAL_GetTick() - kbd1tik) > 50) {
				putMsg(msg_kbd);
				kbd1tik = HAL_GetTick();
			}
		}
	}
#endif

	if (keyFlag) putMsg(msg_keyEvent);

}
//-------------------------------------------------------------------------------------------
/*
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{

}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
*/
//-------------------------------------------------------------------------------------------



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
