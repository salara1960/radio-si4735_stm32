#include "hdr.h"
#include "main.h"
#include "si4735.h"

#ifdef SET_SI4735


/**
 * @brief Construct a new SI4735::SI4735
 *
 * @details This class has a set of functions that can help you to build your receiver based on Si47XX IC family.
 * @details This library uses the I²C communication protocol and implements most of the functions offered by Si47XX (BROADCAST AM / FM / SW / LW RADIO RECEIVER) IC family from Silicon Labs.
 * @details Currently you have more than 120 functions implemented to control the Si47XX devices. These functions are listed and documented here.
 * @details Some methods were implemented using inline resource. Inline methods are implemented in SI4735.h
 *
 * IMPORTANT: According to Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 207, "For write operations, the system controller next
 * sends a data byte on SDIO, which is captured by the device on rising edges of SCLK. The device acknowledges
 * each data byte by driving SDIO low for one cycle on the next falling edge of SCLK.
 * The system controller may write up to 8 data bytes in a single 2-wire transaction.
 * The first byte is a command, and the next seven bytes are arguments. Writing more than 8 bytes results
 * in unpredictable device behavior". So, If you are extending this library, consider that restriction presented earlier.
 */
//---------------------------------------------------------------------------------------------
char rds_buffer2A[65]; //!<  RDS Radio Text buffer - Program Information
char rds_buffer2B[33]; //!<  RDS Radio Text buffer - Station Informaation
char rds_buffer0A[9];  //!<  RDS Basic tuning and switching information (Type 0 groups)
char rds_time[20];     //!<  RDS date time received information

int rdsTextAdress2A; //!<  rds_buffer2A current position
int rdsTextAdress2B; //!<  rds_buffer2B current position
int rdsTextAdress0A; //!<  rds_buffer0A current position

int16_t deviceAddress = SI473X_ADDR_SEN_LOW; //!<  Stores the current I2C bus address.


// Delays
uint16_t maxDelaySetFrequency = MAX_DELAY_AFTER_SET_FREQUENCY; //!< Stores the maximum delay after set frequency command (in ms).
uint16_t maxDelayAfterPouwerUp = MAX_DELAY_AFTER_POWERUP;      //!< Stores the maximum delay you have to setup after a power up command (in ms).
unsigned long maxSeekTime = MAX_SEEK_TIME;                     //!< Stores the maximum time (ms) for a seeking process. Defines the maximum seeking time.

uint8_t lastTextFlagAB;
//uint8_t resetPin;     //!<  pin used on Arduino Board to RESET the Si47XX device
uint8_t interruptPin; //!<  pin used on Arduino Board to control interrupt. If -1, interrupt is no used.

uint8_t currentTune; //!<  tell the current tune (FM, AM or SSB)

uint16_t currentMinimumFrequency; //!<  minimum frequency of the current band
uint16_t currentMaximumFrequency; //!<  maximum frequency of the current band
uint16_t currentWorkFrequency;    //!<  current frequency

uint16_t currentStep; //!<  Stores the current step used to increment or decrement the frequency.

uint8_t lastMode = -1; //!<  Stores the last mode used.

uint8_t currentAvcAmMaxGain = 48;          //!<  Stores the current Automatic Volume Control Gain for AM. Default value is 48.
uint8_t currentClockType = XOSCEN_CRYSTAL; //!< Stores the current clock type used (Crystal or REF CLOCK)
uint8_t currentInterruptEnable = 0;        //!< If you are using interrupt, this variable stores 1.

uint16_t refClock = 31768;     //!< Frequency of Reference Clock in Hz.//было 31768 - почему ???
uint16_t refClockPrescale = 1; //!< Prescaler for Reference Clock (divider).
uint8_t refClockSourcePin = 0; //!< 0 = RCLK pin is clock source; 1 = DCLK pin is clock source.

si47x_frequency currentFrequency; //!<  data structure to get current frequency
si47x_set_frequency currentFrequencyParams;
si47x_rqs_status currentRqsStatus;       //!<  current Radio SIgnal Quality status
si47x_response_status currentStatus;     //!<  current device status
si47x_firmware_information firmwareInfo; //!<  firmware information
si47x_rds_status currentRdsStatus;       //!<  current RDS status
si47x_agc_status currentAgcStatus;       //!<  current AGC status
si47x_ssb_mode currentSSBMode;           //!<  indicates if USB or LSB

si473x_powerup powerUp;

uint8_t Volume = 32; //!< Stores the current vlume setup (0-63).

uint8_t currentAudioMode = SI473X_ANALOG_AUDIO; //!< Current audio mode used (ANALOG or DIGITAL or both)
uint8_t currentSsbStatus = 0;// 1 = LSB and 2 = USB; 0 = AM, FM or WB
int8_t audioMuteMcuPin = -1;


//---------------------------------------------------------------------------------------------
void SI4735_write(uint8_t *data, size_t len)
{
	i2cWrite(data, len, deviceAddress);
}
void SI4735_write_to(uint8_t *data, size_t len, uint16_t to)
{
	i2cWriteTo(data, len, deviceAddress, to);
}
void SI4735_read(uint8_t *data, size_t len)
{
	i2cRead(data, len, deviceAddress);
}
void SI4735_read_from(uint8_t *data, size_t len, uint16_t from)
{
	i2cReadFrom(data, len, deviceAddress, from);
}
void SI4735_setStep(uint16_t s)
{
	currentStep = s;
}
uint16_t SI4735_getStep()
{
	return currentStep;
}
//---------------------------------------------------------------------------------------------
/** @defgroup group05 Deal with Interrupt and I2C bus */

/**
 * @ingroup group05 Interrupt
 * @brief Interrupt handle
 * @details If you setup interrupt, this function will be called whenever the Si4735 changes.
 *
 */
#ifdef SET_SI4735_INT
void SI4735_waitInterrupr(void)
{
    while (!data_from_si4735);
}
#endif
/**
 * @ingroup group05 Interrupt
 *
 * @brief Updates bits 6:0 of the status byte.
 *
 * @details This command should be called after any command that sets the STCINT or RSQINT bits.
 * @details When polling this command should be periodically called to monitor the STATUS byte, and when using interrupts, this command should be called after the interrupt is set to update the STATUS byte.
 * @details The CTS bit (and optional interrupt) is set when it is safe to send the next command.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 135
 * @see si47x_status
 * @see waitToSend
 *
 * @return si47x_status the bit data structure with the status response
 */
#ifdef SET_SI4735_INT
si47x_status SI4735_getInterruptStatus()
{
    si47x_status status;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(GET_INT_STATUS);
    Wire.endTransmission();*/
    uint8_t byte = GET_INT_STATUS;
    SI4735_write(&byte, 1);

    /*Wire.requestFrom(deviceAddress, 1);
    status.raw = Wire.read();*/
    SI4735_read(&status.raw, 1);

    return status;
}
#endif
/**
 * @ingroup group05 Interrupt
 *
 * @brief Enables output for GPO1, 2, and 3.
 *
 * @details GPO1, 2, and 3 can be configured for output (Hi-Z or active drive) by setting the GPO1OEN, GPO2OEN, and GPO3OEN bit.
 * @details The state (high or low) of GPO1, 2, and 3 is set with the GPIO_SET command.
 * @details To avoid excessive current consumption due to oscillation, GPO pins should not be left in a high impedance state.
 *
 * | GPIO Output Enable  | value 0 | value 1 |
 * | ---- ---------------| ------- | ------- |
 * | GPO1OEN             | Output Disabled (Hi-Z) (default) | Output Enabled |
 * | GPO2OEN             | Output Disabled (Hi-Z) (default) | Output Enabled |
 * | GPO3OEN             | Output Disabled (Hi-Z) (default) | Output Enabled |
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 82 and 144
 *
 * @param GPO1OEN
 * @param GPO2OEN
 * @param GPO3OEN
 */
void SI4735_setGpioCtl(uint8_t GPO1OEN, uint8_t GPO2OEN, uint8_t GPO3OEN)
{
    si473x_gpio gpio;

    gpio.arg.GPO1OEN = GPO1OEN;
    gpio.arg.GPO2OEN = GPO2OEN;
    gpio.arg.GPO3OEN = GPO3OEN;
    gpio.arg.DUMMY1 = 0;
    gpio.arg.DUMMY2 = 0;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(GPIO_CTL);
    Wire.write(gpio.raw);
    Wire.endTransmission();*/
    uint8_t dat[] = {GPIO_CTL, gpio.raw};
    SI4735_write(dat, sizeof(dat));
}

/**
 * @ingroup group05 Interrupt
 *
 * @brief Sets the output level (high or low) for GPO1, 2, and 3.
 *
 * @details GPO1, 2, and 3 can be configured for output by setting the GPO1OEN, GPO2OEN, and GPO3OEN bit in the GPIO_CTL command.
 * @details To avoid excessive current consumption due to oscillation, GPO pins should not be left in a high impedance state.
 * @details To avoid excessive current consumption due to oscillation, GPO pins should not be left in a high impedance state.
 *
 * | GPIO Output Enable  | value 0 | value 1 |
 * | ---- ---------------| ------- | ------- |
 * | GPO1LEVEL            |  Output low (default) | Output high |
 * | GPO2LEVEL            |  Output low (default) | Output high |
 * | GPO3LEVEL            |  Output low (default) | Output high |
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 83 and 145
 *
 * @param GPO1LEVEL
 * @param GPO2LEVEL
 * @param GPO3LEVEL
 */
void SI4735_setGpio(uint8_t GPO1LEVEL, uint8_t GPO2LEVEL, uint8_t GPO3LEVEL)
{
    si473x_gpio gpio;

    gpio.arg.GPO1OEN = GPO1LEVEL;
    gpio.arg.GPO2OEN = GPO2LEVEL;
    gpio.arg.GPO3OEN = GPO3LEVEL;
    gpio.arg.DUMMY1 = 0;
    gpio.arg.DUMMY2 = 0;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(GPIO_SET);
    Wire.write(gpio.raw);
    Wire.endTransmission();*/
    uint8_t dat[] = {GPIO_SET, gpio.raw};
    SI4735_write(dat, sizeof(dat));
}

/**
 * @ingroup group05 Interrupt
 *
 * @brief Configures the sources for the GPO2/INT interrupt pin.
 *
 * @details Valid sources are the lower 8 bits of the STATUS byte, including CTS, ERR, RSQINT, and STCINT bits.
 * @details The corresponding bit is set before the interrupt occurs. The CTS bit (and optional interrupt) is set when it is safe to send the next command.
 * @details The CTS interrupt enable (CTSIEN) can be set with this property and the POWER_UP command.
 * @details The state of the CTSIEN bit set during the POWER_UP command can be read by reading this property and modified by writing this property.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 146
 *
 * @param STCIEN Seek/Tune Complete Interrupt Enable (0 or 1).
 * @param RSQIEN RSQ Interrupt Enable (0 or 1).
 * @param ERRIEN ERR Interrupt Enable (0 or 1).
 * @param CTSIEN CTS Interrupt Enable (0 or 1).
 * @param STCREP STC Interrupt Repeat (0 or 1).
 * @param RSQREP RSQ Interrupt Repeat(0 or 1).
 */
void SI4735_setGpioIen(uint8_t STCIEN, uint8_t RSQIEN, uint8_t ERRIEN, uint8_t CTSIEN, uint8_t STCREP, uint8_t RSQREP)
{
    si473x_gpio_ien gpio;

    gpio.arg.DUMMY1 = gpio.arg.DUMMY2 = gpio.arg.DUMMY3 = gpio.arg.DUMMY4 = 0;
    gpio.arg.STCIEN = STCIEN;
    gpio.arg.RSQIEN = RSQIEN;
    gpio.arg.ERRIEN = ERRIEN;
    gpio.arg.CTSIEN = CTSIEN;
    gpio.arg.STCREP = STCREP;
    gpio.arg.RSQREP = RSQREP;

    SI4735_sendProperty(GPO_IEN, gpio.raw);
}

/**
 * @ingroup group05 I2C bus address
 *
 * @brief I2C bus address setup
 *
 * @details Scans for two possible addresses for the Si47XX (0x11 or 0x63).
 * @details This function also sets the system to the found I2C bus address of Si47XX.
 * @details You do not need to use this function if the SEN PIN is configured to ground (GND). The default I2C address is 0x11.
 * @details Use this function if you do not know how the SEN pin is configured.
 *
 * @param uint8_t  resetPin MCU Mater (Arduino) reset pin
 *
 * @return int16_t 0x11   if the SEN pin of the Si47XX is low or 0x63 if the SEN pin of the Si47XX is HIGH or 0x0 if error.
 */
int16_t SI4735_getDeviceI2CAddress()//uint8_t resetPin)
{

	if (!(devError & devI2C)) return deviceAddress; else return 0;

/*
    int16_t error;

    //resetPin = resetPin;

    SI4735_reset();

    Wire.begin();
    // check 0X11 I2C address
    Wire.beginTransmission(SI473X_ADDR_SEN_LOW);
    error = Wire.endTransmission();
    if (!error) {
    	SI4735_setDeviceI2CAddress(0);
        return SI473X_ADDR_SEN_LOW;
    }

    // check 0X63 I2C address
    Wire.beginTransmission(SI473X_ADDR_SEN_HIGH);
    error = Wire.endTransmission();
    if (!error) {
    	SI4735_setDeviceI2CAddress(1);
        return SI473X_ADDR_SEN_HIGH;
    }

    // Did find the device
    return 0;
*/
}

/**
 * @ingroup group05 I2C bus address
 *
 * @brief Sets the I2C Bus Address
 *
 * @details The parameter senPin is not the I2C bus address. It is the SEN pin setup of the schematic (eletronic circuit).
 * @details If it is connected to the ground, call this function with senPin = 0; else senPin = 1.
 * @details You do not need to use this function if the SEN PIN configured to ground (GND).
 * @details The default value is 0x11 (senPin = 0). In this case you have to ground the pin SEN of the SI473X.
 * @details If you want to change this address, call this function with senPin = 1.
 *
 * @param senPin 0 -  when the pin SEN (16 on SSOP version or pin 6 on QFN version) is set to low (GND - 0V);
 *               1 -  when the pin SEN (16 on SSOP version or pin 6 on QFN version) is set to high (+3.3V).
 */
/*
void SI4735_setDeviceI2CAddress(uint8_t senPin)
{
    deviceAddress = (senPin) ? SI473X_ADDR_SEN_HIGH : SI473X_ADDR_SEN_LOW;
};
*/
/**
 * @ingroup group05 I2C bus address
 *
 * @brief Sets the onther I2C Bus Address (for Si470X)
 *
 * @details You can set another I2C address different of 0x11  and 0x63
 *
 * @param uint8_t i2cAddr (example 0x10)
 */
/*
void SI4735_setDeviceOtherI2CAddress(uint8_t i2cAddr)
{
    deviceAddress = i2cAddr;
};
*/
/** @defgroup group06 Host and slave MCU setup */

/**
 * @ingroup group06 RESET
 *
 * @brief Reset the SI473X
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0);
 */
void SI4735_reset()
{
    SI4735_RST_Clr();
    _delay(1);
    SI4735_RST_Set();
    _delay(1);
}

/**
 * @ingroup group06 Wait to send command
 *
 * @brief  Wait for the si473x is ready (Clear to Send (CTS) status bit have to be 1).
 *
 * @details This function should be used before sending any command to a SI47XX device.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 63, 128
 */
void SI4735_waitToSend()
{
	for (int i = 0; i < (MIN_DELAY_WAIT_SEND_LOOP >> 2); i++) {
		asm(" nop");
	}

    /*do
    {
        delayMicroseconds(MIN_DELAY_WAIT_SEND_LOOP); // Need check the minimum value.
        Wire.requestFrom(deviceAddress, 1);
    } while (!(Wire.read() & B10000000));*/
}

/** @defgroup group07 Device Setup and Start up */

/**
 * @ingroup group07 Device Power Up parameters
 *
 * @brief Set the Power Up parameters for si473X.
 *
 * @details Use this method to chenge the defaul behavior of the Si473X. Use it before PowerUp()
 * @details About the parameter XOSCEN:
 * @details     0 = Use external RCLK (crystal oscillator disabled);
 * @details     1 = Use crystal oscillator (RCLK and GPO3/DCLK with external 32.768 kHz crystal and OPMODE = 01010000).
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 65 and 129
 *
 * @param uint8_t CTSIEN sets Interrupt anabled or disabled (1 = anabled and 0 = disabled )
 * @param uint8_t GPO2OEN sets GP02 Si473X pin enabled (1 = anabled and 0 = disabled )
 * @param uint8_t PATCH  Used for firmware patch updates. Use it always 0 here.
 * @param uint8_t XOSCEN sets external Crystal enabled or disabled. 0 = Use external RCLK (crystal oscillator disabled); 1 = Use crystal oscillator
 * @param uint8_t FUNC sets the receiver function have to be used [0 = FM Receive; 1 = AM (LW/MW/SW) and SSB (if SSB patch apllied)]
 * @param uint8_t OPMODE set the kind of audio mode you want to use.
 */
void SI4735_setPowerUp(uint8_t CTSIEN, uint8_t GPO2OEN, uint8_t PATCH, uint8_t XOSCEN, uint8_t FUNC, uint8_t OPMODE)
{
    powerUp.arg.CTSIEN = CTSIEN;   // 1 -> Interrupt anabled;
    powerUp.arg.GPO2OEN = GPO2OEN; // 1 -> GPO2 Output Enable;
    powerUp.arg.PATCH = PATCH;     // 0 -> Boot normally;
    powerUp.arg.XOSCEN = XOSCEN;   // 1 -> Use external crystal oscillator;
    powerUp.arg.FUNC = FUNC;       // 0 = FM Receive; 1 = AM/SSB (LW/MW/SW) Receiver.
    powerUp.arg.OPMODE = OPMODE;   // 0x5 = 00000101 = Analog audio outputs (LOUT/ROUT).

    // Set the current tuning frequancy mode 0X20 = FM and 0x40 = AM (LW/MW/SW)
    // See See Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 55 and 124

    currentClockType = XOSCEN;

    if (!FUNC) {
        currentTune = FM_TUNE_FREQ;
        currentFrequencyParams.arg.FREEZE = 1;
    } else {
        currentTune = AM_TUNE_FREQ;
        currentFrequencyParams.arg.FREEZE = 0;
    }
    currentFrequencyParams.arg.FAST = 1;
    currentFrequencyParams.arg.DUMMY1 = 0;
    currentFrequencyParams.arg.ANTCAPH = 0;
    currentFrequencyParams.arg.ANTCAPL = 1;
}

/**
 * @ingroup group07 Device Power Up
 *
 * @brief Powerup the Si47XX
 *
 * @details Before call this function call the setPowerUp to set up the parameters.
 *
 * @details Parameters you have to set up with setPowerUp
 *
 * | Parameter | Description |
 * | --------- | ----------- |
 * | CTSIEN    | Interrupt anabled or disabled |
 * | GPO2OEN   | GPO2 Output Enable or disabled |
 * | PATCH     | Boot normally or patch |
 * | XOSCEN    | Use external crystal oscillator. 1 = Use crystal oscillator;  (crystal oscillator disabled) |
 * | FUNC      | defaultFunction = 0 = FM Receive; 1 = AM (LW/MW/SW) Receiver |
 * | OPMODE    | SI473X_ANALOG_AUDIO (B00000101) or SI473X_DIGITAL_AUDIO (B00001011) |
 *
 * ATTENTION: The document AN383; "Si47XX ANTENNA, SCHEMATIC, LAYOUT, AND DESIGN GUIDELINES"; rev 0.8; page 6; there is the following note:
 *            Crystal and digital audio mode cannot be used at the same time. Populate R1 and remove C10, C11, and X1 when using digital audio.
 *
 *see setMaxDelaySetFrequency()
 * @see MAX_DELAY_AFTER_POWERUP
 * @see  SI4735::setPowerUp()
 * @see  Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 64, 129
 */
void SI4735_radioPowerUp(void)
{
    // delayMicroseconds(1000);
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(POWER_UP);
    Wire.write(powerUp.raw[0]); // Content of ARG1
    Wire.write(powerUp.raw[1]); // COntent of ARG2
    Wire.endTransmission();*/
    uint8_t dat[] = {POWER_UP, powerUp.raw[0], powerUp.raw[1]};
    SI4735_write(dat, sizeof(dat));

    // Delay at least 500 ms between powerup command and first tune command to wait for
    // the oscillator to stabilize if XOSCEN is set and crystal is used as the RCLK.
    SI4735_waitToSend();
    _delay(maxDelayAfterPouwerUp);

    // Turns the external mute circuit off
    if (audioMuteMcuPin >= 0) SI4735_setHardwareAudioMute(false);

    if (currentClockType == 0) {
    	SI4735_setRefClock(refClock);
    	SI4735_setRefClockPrescaler(refClockPrescale, refClockSourcePin);
    }
}

/**
 * @ingroup group07 Device Power Up
 *
 * @brief You have to call setPowerUp method before.
 * @details This function is still available only for legacy reasons.
 *          If you are using this function, please, replace it by radioPowerup().
 * @deprecated Use radioPowerUp instead.
 * @see  SI4735::setPowerUp()
 * @see  Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 64, 129
 */
void SI4735_analogPowerUp(void)
{
	SI4735_radioPowerUp();
}

/**
 * @ingroup group07 Device Power Down
 *
 * @brief Moves the device from powerup to powerdown mode.
 *
 * @details After Power Down command, only the Power Up command is accepted.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 67, 132
 * @see radioPowerUp()
 */
void SI4735_powerDown(void)
{
    // Turns the external mute circuit on
    if (audioMuteMcuPin >= 0) SI4735_setHardwareAudioMute(true);

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(POWER_DOWN);
    Wire.endTransmission();*/
    uint8_t byte = POWER_DOWN;
    SI4735_write(&byte, 1);
    _delay(3);
}

/**
 * @ingroup group07 Firmware Information
 *
 * @brief Gets firmware information
 * @details The firmware information will be stored in firmwareInfo member variable
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 66, 131
 * @see firmwareInfo
 */
void SI4735_getFirmware(void)
{
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(GET_REV);
    Wire.endTransmission();*/
    uint8_t byte = GET_REV;
    SI4735_write(&byte, 1);
    /*do
    {
    	SI4735_waitToSend();
        // Request for 9 bytes response
        Wire.requestFrom(deviceAddress, 9);
        for (int i = 0; i < 9; i++)
            firmwareInfo.raw[i] = Wire.read();
    } while (firmwareInfo.resp.ERR);*/
    SI4735_waitToSend();
    SI4735_read(firmwareInfo.raw, 9);
}

/**
 * @ingroup group07
 * @brief Sets the frequency of the REFCLK from the output of the prescaler
 * @details The REFCLK range is 31130 to 34406 Hz (32768 ±5% Hz) in 1 Hz steps, or 0 (to disable AFC). For example, an RCLK of 13 MHz would require a prescaler value of 400 to divide it to 32500 Hz REFCLK.
 * @details The reference clock frequency property would then need to be set to 32500 Hz.
 * @details RCLK frequencies between 31130 Hz and 40 MHz are supported, however, there are gaps in frequency coverage for prescaler values ranging from 1 to 10, or frequencies up to 311300 Hz. See table below.
 *
 * Table REFCLK Prescaler
 * | Prescaler  | RCLK Low (Hz) | RCLK High (Hz)   |
 * | ---------- | ------------- | ---------------- |
 * |    1       |   31130       |   34406          |
 * |    2       |   62260       |   68812          |
 * |    3       |   93390       |  103218          |
 * |    4       |  124520       |  137624          |
 * |    5       |  155650       |  172030          |
 * |    6       |  186780       |  206436          |
 * |    7       |  217910       |  240842          |
 * |    8       |  249040       |  275248          |
 * |    9       |  280170       |  309654          |
 * |   10       |  311300       |  344060          |
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 34 and 35
 *
 * @param refclk The allowed REFCLK frequency range is between 31130 and 34406 Hz (32768 ±5%), or 0 (to disable AFC).
 */
void SI4735_setRefClock(uint16_t refclk)
{
	SI4735_sendProperty(REFCLK_FREQ, refclk);
    refClock = refclk;
}

/**
 * @ingroup group07
 * @brief Sets the number used by the prescaler to divide the external RCLK down to the internal REFCLK.
 * @details The range may be between 1 and 4095 in 1 unit steps.
 * @details For example, an RCLK of 13 MHz would require a prescaler value of 400 to divide it to 32500 Hz. The reference clock frequency property would then need to be set to 32500 Hz.
 * @details ATTENTION by default, this function considers you are using the RCLK pin as clock source.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 34 and 35
 *
 * @param prescale  Prescaler for Reference Clock value; Between 1 and 4095 in 1 unit steps. Default is 1.
 * @param rclk_sel  0 = RCLK pin is clock source (default); 1 = DCLK pin is clock source
 */
void SI4735_setRefClockPrescaler(uint16_t prescale, uint8_t rclk_sel)
{
	SI4735_sendProperty(REFCLK_PRESCALE, prescale); //| (rclk_sel << 13)); // Sets the D12 to rclk_sel
    refClockPrescale = prescale;
    refClockSourcePin = rclk_sel;
}

/**
 * @ingroup   group07 Device start up
 *
 * @brief Starts the Si473X device.
 *
 * @details Use this function to start the device up with the parameters shown below.
 * @details If the audio mode parameter is not entered, analog mode will be considered.
 * @details You can use any Arduino digital pin. Be sure you are using less than 3.6V on Si47XX RST pin.
 *
 * ATTENTION: The document AN383; "Si47XX ANTENNA, SCHEMATIC, LAYOUT, AND DESIGN GUIDELINES"; rev 0.8; page 6; there is the following note:
 *            Crystal and digital audio mode cannot be used at the same time. Populate R1 and remove C10, C11, and X1 when using digital audio.
 *
 * @param resetPin Digital Arduino Pin used to RESET de Si47XX device.
 * @param interruptPin interrupt Arduino Pin (see your Arduino pinout). If less than 0, iterrupt disabled.
 * @param defaultFunction is the mode you want the receiver starts.
 * @param audioMode default SI473X_ANALOG_AUDIO (Analog Audio). Use SI473X_ANALOG_AUDIO or SI473X_DIGITAL_AUDIO.
 * @param clockType 0 = Use external RCLK (crystal oscillator disabled); 1 = Use crystal oscillator
 */
void SI4735_setup(int interruptPin, uint8_t defaultFunction, uint8_t audioMode, uint8_t clockType)
{
	//void SI4735_setup1
	//(uint8_t resetPin,
	//int interruptPin,
	//uint8_t defaultFunction,
	//uint8_t audioMode = SI473X_ANALOG_AUDIO,
	//uint8_t clockType = XOSCEN_CRYSTAL);
    currentInterruptEnable = 0;

    //Wire.begin();

#ifdef SET_SI4735_INT
    interruptPin = interruptPin;

    // Arduino interrupt setup (you have to know which Arduino Pins can deal with interrupt).
    if (interruptPin >= 0) {
        pinMode(interruptPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt_hundler, RISING);
        currentInterruptEnable = 1;
    }
#else
    currentInterruptEnable = 0;
#endif

    data_from_si4735 = false;

    currentAudioMode = audioMode;

    // Set the initial SI473X behavior
    // CTSIEN   interruptEnable -> Interrupt anabled or disable;
    // GPO2OEN  1 -> GPO2 Output Enable;
    // PATCH    0 -> Boot normally;
    // XOSCEN   clockType -> Use external crystal oscillator (XOSCEN_CRYSTAL) or reference clock (XOSCEN_RCLK);
    // FUNC     defaultFunction = 0 = FM Receive; 1 = AM (LW/MW/SW) Receiver.
    // OPMODE   SI473X_ANALOG_AUDIO or SI473X_DIGITAL_AUDIO.

    SI4735_setPowerUp(currentInterruptEnable, 0, 0, clockType, defaultFunction, audioMode);

    if (audioMuteMcuPin >= 0) SI4735_setHardwareAudioMute(true); // If you are using external citcuit to mute the audio, it turns the audio mute

    SI4735_reset();

    _delay(10);

    SI4735_radioPowerUp();
    SI4735_setVolume(Volume);//30); // Default volume level.
    SI4735_getFirmware();
}

/**
 * @ingroup   group07 Device start up
 *
 * @brief  Starts the Si473X device.
 *
 * @details Use this setup if you are not using interrupt resource.
 * @details If the audio mode parameter is not entered, analog mode will be considered.
 * @details You can use any Arduino digital pin. Be sure you are using less than 3.6V on Si47XX RST pin.
 *
 * @param uint8_t resetPin Digital Arduino Pin used to RESET command.
 * @param uint8_t defaultFunction. 0 =  FM mode; 1 = AM
 */
void SI4735_init(uint8_t defaultFunction)
{
	SI4735_setup(-1, defaultFunction, SI473X_ANALOG_AUDIO, XOSCEN_CRYSTAL);
    //delay(250);//microsec.
}

/** @defgroup group08 Tune, Device Mode and Filter setup */

/**
 * @ingroup   group08 Internal Antenna Tuning capacitor
 *
 * @brief Selects the tuning capacitor value.
 *
 * @details On FM mode, the Antenna Tuning Capacitor is valid only when using TXO/LPI pin as the antenna input.
 * This selects the value of the antenna tuning capacitor manually, or automatically if set to zero.
 * The valid range is 0 to 191. Automatic capacitor tuning is recommended.
 * For example, if the varactor is set to a value of 5 manually, when read back the value will be 1.
 * @details on AM mode, If the value is set to anything other than 0, the tuning capacitance is manually set as 95 fF x ANTCAP + 7 pF.
 * ANTCAP manual range is 1–6143. Automatic capacitor tuning is recommended. In SW mode, ANTCAPH[15:8] (high byte) needs to be set to 0 and ANTCAPL[7:0] (low byte) needs to be set to 1.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 71 and 136
 *
 * @param capacitor If zero, the tuning capacitor value is selected automatically.
 *                  If the value is set to anything other than 0:
 *                  AM - the tuning capacitance is manually set as 95 fF x ANTCAP + 7 pF.
 *                       ANTCAP manual range is 1–6143;
 *                  FM - the valid range is 0 to 191.
 *                  According to Silicon Labs, automatic capacitor tuning is recommended (value 0).
 */
void SI4735_setTuneFrequencyAntennaCapacitor(uint16_t capacitor)
{
    si47x_antenna_capacitor cap;

    cap.value = capacitor;

    currentFrequencyParams.arg.DUMMY1 = 0;

    if (currentTune != AM_TUNE_FREQ) {// For FM, the capacitor value has just one byte
        currentFrequencyParams.arg.ANTCAPH = (capacitor <= 191) ? cap.raw.ANTCAPL : 0;
    } else {
        if (capacitor <= 6143) {
            currentFrequencyParams.arg.FREEZE = 0; // This parameter is not used for AM
            currentFrequencyParams.arg.ANTCAPH = cap.raw.ANTCAPH;
            currentFrequencyParams.arg.ANTCAPL = cap.raw.ANTCAPL;
        }
    }
    // Tune the device again with the current frequency.
    SI4735_setFrequency(currentWorkFrequency);
}

/**
 * @ingroup   group08 Tune Frequency
 *
 * @brief Set the frequency to the corrent function of the Si4735 (FM, AM or SSB)
 *
 * @details You have to call setup or setPowerUp before call setFrequency.
 *
 * @see maxDelaySetFrequency()
 * @see MAX_DELAY_AFTER_SET_FREQUENCY
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 70, 135
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 13
 *
 * @param uint16_t  freq is the frequency to change. For example, FM => 10390 = 103.9 MHz; AM => 810 = 810 kHz.
 */
void SI4735_setFrequency(uint16_t freq)
{
	SI4735_waitToSend(); // Wait for the si473x is ready.

    currentFrequency.value = freq;
    currentFrequencyParams.arg.FREQH = currentFrequency.raw.FREQH;
    currentFrequencyParams.arg.FREQL = currentFrequency.raw.FREQL;

    if (currentSsbStatus != 0) {
        currentFrequencyParams.arg.DUMMY1 = 0;
        currentFrequencyParams.arg.USBLSB = currentSsbStatus; // Set to LSB or USB
        currentFrequencyParams.arg.FAST = 1;                  // Used just on AM and FM
        currentFrequencyParams.arg.FREEZE = 0;                // Used just on FM
    }

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(currentTune);
    Wire.write(currentFrequencyParams.raw[0]); // Send a byte with FAST and  FREEZE information; if not FM must be 0;
    Wire.write(currentFrequencyParams.arg.FREQH);
    Wire.write(currentFrequencyParams.arg.FREQL);
    Wire.write(currentFrequencyParams.arg.ANTCAPH);
    // If current tune is not FM sent one more byte
    if (currentTune == AM_TUNE_FREQ) // if AM or SSB
        Wire.write(currentFrequencyParams.arg.ANTCAPL);
    Wire.endTransmission();*/
    uint8_t dat[] = {
    		currentTune,
			currentFrequencyParams.raw[0],
			currentFrequencyParams.arg.FREQH,
			currentFrequencyParams.arg.FREQL,
			currentFrequencyParams.arg.ANTCAPH,
			currentFrequencyParams.arg.ANTCAPL
    };
    size_t len = sizeof(dat);
    if (currentTune != AM_TUNE_FREQ) len--;
    SI4735_write(dat, len);

    SI4735_waitToSend();                // Wait for the si473x is ready.
    currentWorkFrequency = freq; // check it
    //delay(maxDelaySetFrequency); // For some reason I need to delay here.
}

/**
 * @ingroup group08 Tune Frequency
 *
 * @brief Increments the current frequency on current band/function by using the current step.
 *
 * @see setFrequencyStep()
 */
void SI4735_frequencyUp()
{
    if (currentWorkFrequency >= currentMaximumFrequency)
        currentWorkFrequency = currentMinimumFrequency;
    else
        currentWorkFrequency += currentStep;

    SI4735_setFrequency(currentWorkFrequency);
}

/**
 * @ingroup group08 Tune Frequency
 *
 * @brief Decrements the current frequency on current band/function by using the current step.
 *
 * @see setFrequencyStep()
 */
void SI4735_frequencyDown()
{

    if (currentWorkFrequency <= currentMinimumFrequency)
        currentWorkFrequency = currentMaximumFrequency;
    else
        currentWorkFrequency -= currentStep;

    SI4735_setFrequency(currentWorkFrequency);
}

/**
 * @ingroup group08 Set mode and Band
 *
 * @todo Adjust the power up parameters
 *
 * @brief Sets the radio to AM function. It means: LW MW and SW.
 *
 * @details Define the band range you want to use for the AM mode.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 129.
 */
void SI4735_setAM0()
{
    // If you’re already using AM mode, it is not necessary to call powerDown and radioPowerUp.
    // The other properties also should have the same value as the previous status.
    if (lastMode != AM_CURRENT_MODE) {
    	SI4735_powerDown();
    	SI4735_setPowerUp(currentInterruptEnable, 0, 0, currentClockType, AM_CURRENT_MODE, currentAudioMode);
    	SI4735_radioPowerUp();
    	SI4735_setAvcAmMaxGain1(currentAvcAmMaxGain); // Set AM Automatic Volume Gain to 48
    	SI4735_setVolume(Volume);                    // Set to previus configured volume
    }
    currentSsbStatus = 0;
    lastMode = AM_CURRENT_MODE;
}

/**
 * @ingroup group08 Set mode and Band
 *
 * @todo Adjust the power up parameters
 *
 * @brief Sets the radio to FM function
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 64.
 */
void SI4735_setFM0()
{
	SI4735_powerDown();
	SI4735_setPowerUp(currentInterruptEnable, 0, 0, currentClockType, FM_CURRENT_MODE, currentAudioMode);
	SI4735_radioPowerUp();
	_delay(1);
	SI4735_setVolume(Volume); // Set to previus configured volume
    currentSsbStatus = 0;
    SI4735_disableFmDebug();
    lastMode = FM_CURRENT_MODE;
}

/**
 * @ingroup group08 Set mode and Band
 *
 * @brief Sets the radio to AM (LW/MW/SW) function.
 *
 * @details The example below sets the band from 550kHz to 1750kHz on AM mode. The band will start on 810kHz and step is 10kHz.
 *
 * @code
 * si4735.setAM(520, 1750, 810, 10);
 * @endcode
 *
 * @see setFM()
 * @see setSSB()
 *
 * @param fromFreq minimum frequency for the band
 * @param toFreq maximum frequency for the band
 * @param initialFreq initial frequency
 * @param step step used to go to the next channel
 */
void SI4735_setAM1(uint16_t fromFreq, uint16_t toFreq, uint16_t initialFreq, uint16_t step)
{

    currentMinimumFrequency = fromFreq;
    currentMaximumFrequency = toFreq;
    currentStep = step;

    if (initialFreq < fromFreq || initialFreq > toFreq)
        initialFreq = fromFreq;

    SI4735_setAM0();
    currentWorkFrequency = initialFreq;
    SI4735_setFrequency(currentWorkFrequency);
}

/**
 * @ingroup group08 Set mode and Band
 *
 * @brief Sets the radio to FM function.
 *
 * @details Defines the band range you want to use for the FM mode.
 *
 * @details The example below sets the band from 64MHz to 108MHzkHz on FM mode. The band will start on 103.9MHz and step is 100kHz.
 * On FM mode, the step 10 means 100kHz. If you want a 1MHz step, use 100.
 *
 * @code
 * si4735.setFM(6400, 10800, 10390, 10);
 * @endcode
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 70
 * @see setFM()
 * @see setFrequencyStep()
 *
 * @param fromFreq minimum frequency for the band
 * @param toFreq maximum frequency for the band
 * @param initialFreq initial frequency (default frequency)
 * @param step step used to go to the next channel
 */
void SI4735_setFM1(uint16_t fromFreq, uint16_t toFreq, uint16_t initialFreq, uint16_t step)
{
    currentMinimumFrequency = fromFreq;
    currentMaximumFrequency = toFreq;
    currentStep = step;

    if (initialFreq < fromFreq || initialFreq > toFreq) initialFreq = fromFreq;

    SI4735_setFM0();

    currentWorkFrequency = initialFreq;
    SI4735_setFrequency(currentWorkFrequency);
}

/** @defgroup group08 Tune */

/**
 * @ingroup group08 Set bandwidth
 *
 * @brief Selects the bandwidth of the channel filter for AM reception.
 *
 * @details The choices are 6, 4, 3, 2, 2.5, 1.8, or 1 (kHz). The default bandwidth is 2 kHz. It works only in AM / SSB (LW/MW/SW)
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 125, 151, 277, 181.
 *
 * @param AMCHFLT the choices are:   0 = 6 kHz Bandwidth
 *                                   1 = 4 kHz Bandwidth
 *                                   2 = 3 kHz Bandwidth
 *                                   3 = 2 kHz Bandwidth
 *                                   4 = 1 kHz Bandwidth
 *                                   5 = 1.8 kHz Bandwidth
 *                                   6 = 2.5 kHz Bandwidth, gradual roll off
 *                                   7–15 = Reserved (Do not use).
 * @param AMPLFLT Enables the AM Power Line Noise Rejection Filter.
 */
void SI4735_setBandwidth(uint8_t AMCHFLT, uint8_t AMPLFLT)
{
    si47x_bandwidth_config filter;
    si47x_property property;

    if (currentTune != AM_TUNE_FREQ) return;// Only for AM/SSB mode

    if (AMCHFLT > 6) return;

    filter.raw[0] = filter.raw[1] = 0;

    property.value = AM_CHANNEL_FILTER;

    filter.param.AMCHFLT = AMCHFLT;
    filter.param.AMPLFLT = AMPLFLT;

    SI4735_waitToSend();

    //Volume = Volume;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);                  // Always 0x00
    Wire.write(property.raw.byteHigh); // High byte first
    Wire.write(property.raw.byteLow);  // Low byte after
    Wire.write(filter.raw[1]);         // Raw data for AMCHFLT and
    Wire.write(filter.raw[0]);         // AMPLFLT
    Wire.endTransmission();*/
    uint8_t dat[] = {SET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow, filter.raw[1], filter.raw[0]};
    SI4735_write(dat, sizeof(dat));

    SI4735_waitToSend();
}

/**
 * @ingroup group08 Frequency
 *
 * @brief Gets the current frequency of the Si4735 (AM or FM)
 *
 * @details The method status do it an more. See getStatus below.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 73 (FM) and 139 (AM)
 */
uint16_t SI4735_getFrequency()
{
    si47x_frequency freq;
    SI4735_getStatus1(0, 1);

    freq.raw.FREQL = currentStatus.resp.READFREQL;
    freq.raw.FREQH = currentStatus.resp.READFREQH;

    currentWorkFrequency = freq.value;
    return freq.value;
}

/**
 * @ingroup group08 Frequency
 *
 * @brief Gets the current status  of the Si4735 (AM or FM)
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 73 (FM) and 139 (AM)
 *
 * @param uint8_t INTACK Seek/Tune Interrupt Clear. If set, clears the seek/tune complete interrupt status indicator;
 * @param uint8_t CANCEL Cancel seek. If set, aborts a seek currently in progress;
 */
void SI4735_getStatus1(uint8_t INTACK, uint8_t CANCEL)
{
    si47x_tune_status status;
    uint8_t cmd = FM_TUNE_STATUS;
    int limitResp = 8;

    if (currentTune == FM_TUNE_FREQ)
        cmd = FM_TUNE_STATUS;
    else if (currentTune == AM_TUNE_FREQ)
        cmd = AM_TUNE_STATUS;
    else if (currentTune == NBFM_TUNE_FREQ) {
        cmd = NBFM_TUNE_STATUS;
        limitResp = 6;
    }

    SI4735_waitToSend();

    status.arg.INTACK = INTACK;
    status.arg.CANCEL = CANCEL;
    status.arg.RESERVED2 = 0;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(cmd);
    Wire.write(status.raw);
    Wire.endTransmission();*/
    uint8_t dat[] = {cmd, status.raw};
    SI4735_write(dat, sizeof(dat));
    // Reads the current status (including current frequency).
    /*do
    {
    	SI4735_waitToSend();
        Wire.requestFrom(deviceAddress, limitResp); // Check it
        // Gets response information
        for (uint8_t i = 0; i < limitResp; i++)
            currentStatus.raw[i] = Wire.read();
    } while (currentStatus.resp.ERR); // If error, try it again*/
    SI4735_read(currentStatus.raw, limitResp);

    SI4735_waitToSend();
}

/**
 * @ingroup group08 AGC
 *
 * @brief Queries Automatic Gain Control STATUS
 *
 * @details After call this method, you can call isAgcEnabled to know the AGC status and getAgcGainIndex to know the gain index value.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); For FM page 80; for AM page 142.
 * @see AN332 REV 0.8 Universal Programming Guide Amendment for SI4735-D60 SSB and NBFM patches; page 18.
 *
 */
void SI4735_getAutomaticGainControl()
{
    uint8_t cmd;

    if (currentTune == FM_TUNE_FREQ) { // FM TUNE
        cmd = FM_AGC_STATUS;
    }
    else if (currentTune == NBFM_TUNE_FREQ) {
        cmd = NBFM_AGC_STATUS;
    }
    else { // AM TUNE - SAME COMMAND used on SSB mode
        cmd = AM_AGC_STATUS;
    }

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(cmd);
    Wire.endTransmission();*/
    SI4735_write(&cmd, 1);
    do
    {
    	SI4735_waitToSend();
        //Wire.requestFrom(deviceAddress, 3);
        //currentAgcStatus.raw[0] = Wire.read(); // STATUS response
        //currentAgcStatus.raw[1] = Wire.read(); // RESP 1
        //currentAgcStatus.raw[2] = Wire.read(); // RESP 2
    	SI4735_read(currentAgcStatus.raw, 3);
    } while (currentAgcStatus.refined.ERR);    // If error, try get AGC status again.

}

/**
 * @ingroup group08 AGC
 *
 * @brief Automatic Gain Control setup
 *
 * @details If FM, overrides AGC setting by disabling the AGC and forcing the LNA to have a certain gain that ranges between 0
 * (minimum attenuation) and 26 (maximum attenuation).
 * @details If AM/SSB, Overrides the AGC setting by disabling the AGC and forcing the gain index that ranges between 0
 * (minimum attenuation) and 37+ATTN_BACKUP (maximum attenuation).
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); For FM page 81; for AM page 143
 *
 * @param uint8_t AGCDIS This param selects whether the AGC is enabled or disabled (0 = AGC enabled; 1 = AGC disabled);
 * @param uint8_t AGCIDX AGC Index (0 = Minimum attenuation (max gain); 1 – 36 = Intermediate attenuation);
 *                if >greater than 36 - Maximum attenuation (min gain) ).
 */
void SI4735_setAutomaticGainControl(uint8_t AGCDIS, uint8_t AGCIDX)
{
    si47x_agc_overrride agc;

    uint8_t cmd;

    // cmd = (currentTune == FM_TUNE_FREQ) ? FM_AGC_OVERRIDE : AM_AGC_OVERRIDE; // AM_AGC_OVERRIDE = SSB_AGC_OVERRIDE = 0x48

    if (currentTune == FM_TUNE_FREQ)
        cmd = FM_AGC_OVERRIDE;
    else if (currentTune == FM_TUNE_FREQ)
        cmd = NBFM_AGC_OVERRIDE;
    else
        cmd = AM_AGC_OVERRIDE;

    agc.arg.AGCDIS = AGCDIS;
    agc.arg.AGCIDX = AGCIDX;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(cmd);
    Wire.write(agc.raw[0]);
    Wire.write(agc.raw[1]);
    Wire.endTransmission();*/
    uint8_t dat[] = {cmd, agc.raw[0], agc.raw[1]};
    SI4735_write(dat, sizeof(dat));

    SI4735_waitToSend();
}

/**
 * @ingroup group08 Automatic Volume Control
 *
 * @brief Sets the maximum gain for automatic volume control.
 *
 * @details If no parameter is sent, it will be consider 48dB.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 152
 * @see setAvcAmMaxGain()
 *
 * @param uint8_t gain  Select a value between 12 and 192.  Defaul value 48dB.
 */
void SI4735_setAvcAmMaxGain1(uint8_t gain)
{
    uint16_t aux;
    aux = (gain > 12 && gain < 193) ? (gain * 340) : (48 * 340);
    currentAvcAmMaxGain = gain;
    SI4735_sendProperty(AM_AUTOMATIC_VOLUME_CONTROL_MAX_GAIN, aux);
}

/**
 * @ingroup group08 Received Signal Quality
 *
 * @brief Queries the status of the Received Signal Quality (RSQ) of the current channel.
 *
 * @details This method sould be called berore call getCurrentRSSI(), getCurrentSNR() etc.
 * Command FM_RSQ_STATUS
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 75 and 141
 *
 * @param INTACK Interrupt Acknowledge.
 *        0 = Interrupt status preserved;
 *        1 = Clears RSQINT, BLENDINT, SNRHINT, SNRLINT, RSSIHINT, RSSILINT, MULTHINT, MULTLINT.
 */
void SI4735_getCurrentReceivedSignalQuality1(uint8_t INTACK)
{
    uint8_t arg = INTACK;
    uint8_t cmd;
    int sizeResponse;

    if (currentTune == FM_TUNE_FREQ) { // FM TUNE
        cmd = FM_RSQ_STATUS;
        sizeResponse = 8;
    } else if (currentTune == NBFM_TUNE_FREQ) {
        cmd = NBFM_RSQ_STATUS;
        sizeResponse = 8; // Check it
    } else { // AM TUNE
        cmd = AM_RSQ_STATUS;
        sizeResponse = 6; // Check it
    }

    SI4735_waitToSend();

    //arg = INTACK;
    /*Wire.beginTransmission(deviceAddress);
    Wire.write(cmd);
    Wire.write(arg); // send B00000001
    Wire.endTransmission();*/
    uint8_t dat[] = {cmd, arg};
    SI4735_write(dat, sizeof(dat));

    // Check it
    do
    {
    	SI4735_waitToSend();

    //Wire.requestFrom(deviceAddress, sizeResponse);
    // Gets response information
    //for (uint8_t i = 0; i < sizeResponse; i++)
    //    currentRqsStatus.raw[i] = Wire.read();
    	SI4735_read(&currentRqsStatus.raw[0], sizeResponse);
    } while (currentRqsStatus.resp.ERR); // Try again if error found

}

/**
 * @ingroup group08 Received Signal Quality
 *
 * @brief Queries the status of the Received Signal Quality (RSQ) of the current channel (FM_RSQ_STATUS)
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 75 and 141
 *
 * @param INTACK Interrupt Acknowledge.
 *        0 = Interrupt status preserved;
 *        1 = Clears RSQINT, BLENDINT, SNRHINT, SNRLINT, RSSIHINT, RSSILINT, MULTHINT, MULTLINT.
 */
void SI4735_getCurrentReceivedSignalQuality(void)
{
	SI4735_getCurrentReceivedSignalQuality1(0);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Look for a station (Automatic tune)
 * @details Starts a seek process for a channel that meets the RSSI and SNR criteria for AM.
 * @details __This function does not work on SSB mode__.
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 55, 72, 125 and 137
 *
 * @param SEEKUP Seek Up/Down. Determines the direction of the search, either UP = 1, or DOWN = 0.
 * @param Wrap/Halt. Determines whether the seek should Wrap = 1, or Halt = 0 when it hits the band limit.
 */
void SI4735_seekStation(uint8_t SEEKUP, uint8_t WRAP)
{
    si47x_seek seek;
    si47x_seek_am_complement seek_am_complement;

    // Check which FUNCTION (AM or FM) is working now
    uint8_t seek_start_cmd = (currentTune == FM_TUNE_FREQ) ? FM_SEEK_START : AM_SEEK_START;

    SI4735_waitToSend();

    seek.arg.SEEKUP = SEEKUP;
    seek.arg.WRAP = WRAP;
    seek.arg.RESERVED1 = 0;
    seek.arg.RESERVED2 = 0;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(seek_start_cmd);
    Wire.write(seek.raw); // ARG1
    if (seek_start_cmd == AM_SEEK_START) {// Sets additional configuration for AM mode
        seek_am_complement.ARG2 = seek_am_complement.ARG3 = 0;
        seek_am_complement.ANTCAPH = 0;
        seek_am_complement.ANTCAPL = (currentWorkFrequency > 1800) ? 1 : 0; // if SW = 1
        Wire.write(seek_am_complement.ARG2);                                // ARG2 - Always 0
        Wire.write(seek_am_complement.ARG3);                                // ARG3 - Always 0
        Wire.write(seek_am_complement.ANTCAPH);                             // ARG4 - Tuning Capacitor: The tuning capacitor value
        Wire.write(seek_am_complement.ANTCAPL);                             // ARG5 - will be selected automatically.
    }
    Wire.endTransmission();*/
    uint8_t dat[] = {
    		seek_start_cmd,
    		seek.raw,
			seek_am_complement.ARG2 = seek_am_complement.ARG3 = 0,
			seek_am_complement.ANTCAPH = 0,
			seek_am_complement.ANTCAPL = (currentWorkFrequency > 1800) ? 1 : 0
    };
    size_t len = 2;
    if (seek_start_cmd == AM_SEEK_START) len = sizeof(dat);
    SI4735_write(dat, len);

    _delay(MAX_DELAY_AFTER_SET_FREQUENCY << 2);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Search for the next station.
 * @details Like seekStationUp this function goes to a next station.
 * @details The main difference is the method used to look for a station.
 *
 * @see seekStation, seekStationUp, seekStationDown, seekPreviousStation, seekStationProgress
 */
void SI4735_seekNextStation()
{
	SI4735_seekStation(1, 1);
    _delay(maxDelaySetFrequency);
    SI4735_getFrequency();
}

/**
 * @ingroup group08 Seek
 *
 * @brief Search the previous station
 * @details Like seekStationDown this function goes to a previous station.
 * @details The main difference is the method used to look for a station.
 * @see seekStation, seekStationUp, seekStationDown, seekPreviousStation, seekStationProgress
 */
void SI4735_seekPreviousStation()
{
	SI4735_seekStation(0, 1);
    _delay(maxDelaySetFrequency);
    SI4735_getFrequency();
}

/**
 * @ingroup group08 Seek
 * @brief Seeks a station up or down.
 * @details Seek up or down a station and call a function defined by the developer to show the frequency.
 * @details The first parameter of this function is a name of your function that you have to implement to show the current frequency.
 * @details If you do not want to show the seeking progress,  you can set NULL instead the name of the function.
 * @details The code below shows an example using ta function the shows the current frequency on he Serial Monitor. You might want to implement a function that shows the frequency on your display device.
 * @details Also, you have to declare the frequency parameter that will be used by the function to show the frequency value.
 * @details __This function does not work on SSB mode__.
 * @code
 * void showFrequency( uint16_t freq ) {
 *    Serial.print(freq);
 *    Serial.println("MHz ");
 * }
 *
 * void loop() {
 *
 *  receiver.seekStationProgress(showFrequency,1); // Seek Up
 *  .
 *  .
 *  .
 *  receiver.seekStationProgress(showFrequency,0); // Seek Down
 *
 * }
 * @endcode
 *
 * @see seekStation, seekStationUp, seekStationDown, getStatus, setMaxSeekTime
 * @param showFunc  function that you have to implement to show the frequency during the seeking process. Set NULL if you do not want to show the progress.
 * @param up_down   set up_down = 1 for seeking station up; set up_down = 0 for seeking station down
 */
void SI4735_seekStationProgress0(void (*showFunc)(uint16_t f), uint8_t up_down)
{
    si47x_frequency freq;
    long elapsed_seek = _millis();

    // seek command does not work for SSB
    if (lastMode == SSB_CURRENT_MODE) return;

    do
    {
    	SI4735_seekStation(up_down, 0);
        _delay(maxDelaySetFrequency);
        SI4735_getStatus1(0, 0);
        _delay(maxDelaySetFrequency);
        freq.raw.FREQH = currentStatus.resp.READFREQH;
        freq.raw.FREQL = currentStatus.resp.READFREQL;
        currentWorkFrequency = freq.value;
        if (showFunc != NULL) showFunc(freq.value);
    } while (!currentStatus.resp.VALID && !currentStatus.resp.BLTF && (_millis() - elapsed_seek) < maxSeekTime);
}

/**
 * @ingroup group08 Seek
 * @brief Seeks a station up or down.
 * @details Seek up or down a station and call a function defined by the developer to show the frequency and stop seeking process by the user.
 * @details The first parameter of this function is a name of your function that you have to implement to show the current frequency.
 * @details The second parameter is the name function that will check stop seeking action. Thus function should return true or false and should read a button, encoder or some status to make decision to stop or keep seeking.
 * @details If you do not want to show the seeking progress,  you can set NULL instead the name of the function.
 * @details If you do not want stop seeking checking, you can set NULL instead the name of a function.
 * @details The code below shows an example using ta function the shows the current frequency on he Serial Monitor. You might want to implement a function that shows the frequency on your display device.
 * @details Also, you have to declare the frequency parameter that will be used by the function to show the frequency value.
 * @details __This function does not work on SSB mode__.
 * @code
 * void showFrequency( uint16_t freq ) {
 *    Serial.print(freq);
 *    Serial.println("MHz ");
 * }
 *
 * void loop() {
 *
 *  receiver.seekStationProgress(showFrequency, checkStopSeeking, 1); // Seek Up
 *  .
 *  .
 *  .
 *  receiver.seekStationProgress(showFrequency, checkStopSeeking, 0); // Seek Down
 *
 * }
 * @endcode
 *
 * @see seekStation, seekStationUp, seekStationDown, getStatus, setMaxSeekTime
 * @param showFunc  function that you have to implement to show the frequency during the seeking process. Set NULL if you do not want to show the progress.
 * @param stopSeeking functionthat you have to implement if you want to control the stop seeking action.
 * @param up_down   set up_down = 1 for seeking station up; set up_down = 0 for seeking station down
 */
void SI4735_seekStationProgress1(void (*showFunc)(uint16_t f), bool (*stopSeking)(), uint8_t up_down)
{
    si47x_frequency freq;
    long elapsed_seek = _millis();

    // seek command does not work for SSB
    if (lastMode == SSB_CURRENT_MODE) return;

    do
    {
    	SI4735_seekStation(up_down, 0);
        _delay(maxDelaySetFrequency);
        SI4735_getStatus1(0, 0);
        _delay(maxDelaySetFrequency);
        freq.raw.FREQH = currentStatus.resp.READFREQH;
        freq.raw.FREQL = currentStatus.resp.READFREQL;
        currentWorkFrequency = freq.value;
        if (showFunc != NULL)
        	showFunc(freq.value);
        if (stopSeking != NULL )
           if ( stopSeking() ) return;

    } while (!currentStatus.resp.VALID && !currentStatus.resp.BLTF && (_millis() - elapsed_seek) < maxSeekTime);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Sets the bottom frequency and top frequency of the AM band for seek. Default is 520 to 1710.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 127, 161, and 162
 *
 * @param uint16_t bottom - the bottom of the AM (MW/SW) mode for seek
 * @param uint16_t    top - the top of the AM (MW/SW) mode for seek
 */
void SI4735_setSeekAmLimits(uint16_t bottom, uint16_t top)
{
	SI4735_sendProperty(AM_SEEK_BAND_BOTTOM, bottom);
	SI4735_sendProperty(AM_SEEK_BAND_TOP, top);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Sets the bottom frequency and top frequency of the FM band for seek. Default is 8750 to 10790.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 100 and  101
 *
 * @param uint16_t bottom - the bottom of the FM(VHF) mode for seek
 * @param uint16_t    top - the top of the FM(VHF) mode for seek
 */
void SI4735_setSeekFmLimits(uint16_t bottom, uint16_t top)
{
	SI4735_sendProperty(FM_SEEK_BAND_BOTTOM, bottom);
	SI4735_sendProperty(FM_SEEK_BAND_TOP, top);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Selects frequency spacingfor AM seek. Default is 10 kHz spacing.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 163, 229 and 283
 *
 * @param uint16_t spacing - step in kHz
 */
void SI4735_setSeekAmSpacing(uint16_t spacing)
{
	SI4735_sendProperty(AM_SEEK_FREQ_SPACING, spacing);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Selects frequency spacingfor FM seek. Default is 100 kHz kHz spacing. There are only 3 valid values: 5, 10, and 20.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 101
 *
 * @param uint16_t spacing - step in kHz
 */
void SI4735_setSeekFmSpacing(uint16_t spacing)
{
	SI4735_sendProperty(FM_SEEK_FREQ_SPACING, spacing);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Sets the SNR threshold for a valid AM Seek/Tune.
 *
 * @details If the value is zero then SNR threshold is not considered when doing a seek. Default value is 5 dB.
 *
 * @see Si47XX PROGRAMMING GUIDE;  (REV 1.0); page 127
 */
void SI4735_setSeekAmSrnThreshold(uint16_t value)
{
	SI4735_sendProperty(AM_SEEK_SNR_THRESHOLD, value);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Sets the SNR threshold for a valid FM Seek/Tune.
 *
 * @details SNR Threshold which determines if a valid channel has been found during Seek/Tune. Specified in units of dB in 1 dB steps (0–127). Default is 3 dB
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 102
 *
 * @param value between 0 and 127.
 */
void SI4735_setSeekFmSrnThreshold(uint16_t value)
{
	SI4735_sendProperty(FM_SEEK_TUNE_SNR_THRESHOLD, value);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Sets the RSSI threshold for a valid AM Seek/Tune.
 *
 * @details If the value is zero then RSSI threshold is not considered when doing a seek. Default value is 25 dBμV.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 127
 */
void SI4735_setSeekAmRssiThreshold(uint16_t value)
{
	SI4735_sendProperty(AM_SEEK_RSSI_THRESHOLD, value);
}

/**
 * @ingroup group08 Seek
 *
 * @brief Sets the RSSI threshold for a valid FM Seek/Tune.
 *
 * @details RSSI threshold which determines if a valid channel has been found during seek/tune. Specified in units of dBμV in 1 dBμV steps (0–127). Default is 20 dBμV.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 102
 */
void SI4735_setSeekFmRssiThreshold(uint16_t value)
{
	SI4735_sendProperty(FM_SEEK_TUNE_RSSI_THRESHOLD, value);
}

/** @defgroup group10 Tools method
 * @details A set of functions used to support other functions
*/

/**
 * @ingroup group10 Generic send property
 *
 * @brief Sends (sets) property to the SI47XX
 *
 * @details This method is used for others to send generic properties and params to SI47XX
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 68, 124 and  133.
 * @see setProperty, sendCommand, getProperty, getCommandResponse
 *
 * @param propertyNumber property number (example: RX_VOLUME)
 * @param parameter   property value that will be seted
 */
void SI4735_sendProperty(uint16_t propertyNumber, uint16_t parameter)
{
    si47x_property property;
    si47x_property param;

    property.value = propertyNumber;
    param.value = parameter;

    SI4735_waitToSend();

    uint8_t dat[] = {SET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow, param.raw.byteHigh, param.raw.byteLow};
    SI4735_write(dat, sizeof(dat));

    _delay(1);//delayMicroseconds(550);
}

/**
 * @ingroup group10 Generic Command and Response
 * @brief Sends a given command to the SI47XX devices.
 * @details This function can be useful when you want to execute a SI47XX device command and it was not implemented by this library.
 * @details In this case you have to check the  AN332-Si47XX PROGRAMMING GUIDE to know how the command works.
 * @details Also, you need to work with bit operators to compose the parameters of the command [ &(and), ˆ(xor), |(or) etc ].
 *
 * @see getCommandResponse, setProperty
 *
 * @param cmd command number (see AN332-Si47XX PROGRAMMING GUIDE)
 * @param parameter_size Parameter size in bytes. Tell the number of argument used by the command.
 * @param parameter unsigned byte array with the arguments of the command
 */
void SI4735_sendCommand(uint8_t cmd, int parameter_size, const uint8_t *parameter)
{
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    // Sends the command to the device
    Wire.write(cmd);
    // Sends the argments (parameters) of the command
    for (byte i = 0; i < parameter_size; i++)
        Wire.write(parameter[i]);
    Wire.endTransmission();*/

    uint8_t *dat = (uint8_t *)calloc(1, parameter_size + 1);
    if (dat) {
    	*dat = cmd;
    	memcpy(dat + 1, parameter, parameter_size);
        SI4735_write(dat, parameter_size + 1);
        free(dat);
    }
}

/**
 * @ingroup group10 Generic Command and Response
 * @brief   Returns with the command response.
 * @details After a command is executed by the device, you can get the result (response) of the command by calling this method.
 *
 * @see sendCommand, setProperty
 *
 * @param response_size  num of bytes returned by the command.
 * @param response  byte array where the response will be stored.
 */
void SI4735_getCommandResponse(int response_size, uint8_t *response)
{
	SI4735_waitToSend();
    // Asks the device to return a given number o bytes response
    /*Wire.requestFrom(deviceAddress, response_size);
    // Gets response information
    for (byte i = 0; i < response_size; i++)
        response[i] = Wire.read();*/
    SI4735_read(response, response_size);
}

/**
 * @ingroup group10 Generic Command and Response
 * @brief Gets the first byte response.
 * @details In this context status is the first response byte for any SI47XX command. See si47x_status structure.
 * @details This function can be useful to check, for example, the success or failure of a command.
 *
 * @see si47x_status
 *
 * @return si47x_status
 */
si47x_status SI4735_getStatusResponse()
{
    si47x_status status;

    /*Wire.requestFrom(deviceAddress, 1);
    status.raw = Wire.read();*/
    SI4735_read(&status.raw, 1);

    return status;
}

/**
 * @ingroup group10 Generic get property
 *
 * @brief Gets a given property from the SI47XX
 *
 * @details This method is used to get a given property from SI47XX
 * @details You might need to extract set of bits information from the returned value to know the real value
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 55, 69, 124 and  134.
 * @see sendProperty, setProperty, sendCommand, getCommandResponse
 *
 * @param propertyNumber property number (example: RX_VOLUME)
 *
 * @return property value  (the content of the property)
 */
int32_t SI4735_getProperty(uint16_t propertyNumber)
{
    si47x_property property;
    si47x_status status;

    property.value = propertyNumber;
    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(GET_PROPERTY);
    Wire.write(0x00);
    Wire.write(property.raw.byteHigh); // Send property - High byte - most significant first
    Wire.write(property.raw.byteLow);  // Send property - Low byte - less significant after
    Wire.endTransmission();*/
    uint8_t dat[] = {GET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow};
    SI4735_write(dat, sizeof(dat));

    SI4735_waitToSend();

    /*Wire.requestFrom(deviceAddress, 4);
    status.raw = Wire.read();*/
    SI4735_read(dat, 4);
    status.raw = dat[0];

    // if error, return 0;
    if (status.refined.ERR == 1) return -1;

    //Wire.read(); // dummy

    // gets the property value
    property.raw.byteHigh = dat[2];//Wire.read();
    property.raw.byteLow = dat[3];//Wire.read();

    return property.value;
}

/** @defgroup group12 FM Mono Stereo audio setup */

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets RSSI threshold for stereo blend (Full stereo above threshold, blend below threshold).
 *
 * @details To force stereo, set this to 0. To force mono, set this to 127.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 90.
 *
 * @param parameter  valid values: 0 to 127
 */
void SI4735_setFmBlendStereoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_STEREO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets RSSI threshold for mono blend (Full mono below threshold, blend above threshold).
 *
 * @details To force stereo set this to 0. To force mono set this to 127. Default value is 30 dBμV.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 56.
 *
 * @param parameter valid values: 0 to 127
 */
void SI4735_setFmBlendMonoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_MONO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets RSSI threshold for stereo blend. (Full stereo above threshold, blend below threshold.)
 *
 * @details To force stereo, set this to 0. To force mono, set this to 127. Default value is 49 dBμV.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 59.
 *
 * @param parameter valid values: 0 to 127
 */
void SI4735_setFmBlendRssiStereoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_RSSI_STEREO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets RSSI threshold for mono blend (Full mono below threshold, blend above threshold).
 *
 * @details To force stereo, set this to 0. To force mono, set this to 127. Default value is 30 dBμV.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 59.
 *
 * @param parameter valid values: 0 to 127
 */
void SI4735_setFmBLendRssiMonoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_RSSI_MONO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets SNR threshold for stereo blend (Full stereo above threshold, blend below threshold).
 *
 * @details To force stereo, set this to 0. To force mono, set this to 127. Default value is 27 dB.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 59.
 *
 * @param parameter valid values: 0 to 127
 */
void SI4735_setFmBlendSnrStereoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_SNR_STEREO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets SNR threshold for mono blend (Full mono below threshold, blend above threshold).
 *
 * @details To force stereo, set this to 0. To force mono, set this to 127. Default value is 14 dB.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 59.
 *
 * @param parameter valid values: 0 to 127
 */
void SI4735_setFmBLendSnrMonoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_SNR_MONO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets multipath threshold for stereo blend (Full stereo below threshold, blend above threshold).
 *
 * @details To force stereo, set this to 100. To force mono, set this to 0. Default value is 20.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 60.
 *
 * @param parameter valid values: 0 to 100
 */
void SI4735_setFmBlendMultiPathStereoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_MULTIPATH_STEREO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief Sets Multipath threshold for mono blend (Full mono above threshold, blend below threshold).
 *
 * @details To force stereo, set to 100. To force mono, set to 0. The default is 60.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 60.
 *
 * @param parameter valid values: 0 to 100
 */
void SI4735_setFmBlendMultiPathMonoThreshold(uint8_t parameter)
{
	SI4735_sendProperty(FM_BLEND_MULTIPATH_MONO_THRESHOLD, parameter);
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 * @todo
 * @brief Turn Off Stereo operation.
 */
void SI4735_setFmStereoOff()
{
    //! TO DO
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 * @todo
 * @brief Turn Off Stereo operation.
 */
void SI4735_setFmStereoOn()
{
    //! TO DO
}

/**
 * @ingroup group12 FM Mono Stereo audio setup
 *
 * @brief There is a debug feature that remains active in Si4704/05/3x-D60 firmware which can create periodic noise in audio.
 *
 * @details Silicon Labs recommends you disable this feature by sending the following bytes (shown here in hexadecimal form):
 * 0x12 0x00 0xFF 0x00 0x00 0x00.
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 299.
 */
void SI4735_disableFmDebug()
{
    /*Wire.beginTransmission(deviceAddress);
    Wire.write(0x12);
    Wire.write(0x00);
    Wire.write(0xFF);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();*/
    uint8_t dat[] = {0x12, 0, 0xff, 0, 0, 0};
    SI4735_write(dat, sizeof(dat));

    _delay(3);//delayMicroseconds(2500);
}

/** @defgroup group13 Audio setup */

/**
 * @ingroup group13 Digital Audio setup
 *
 * @brief Configures the digital audio output format.
 *
 * @details Options: DCLK edge, data format, force mono, and sample precision.
 *
 * ATTENTION: The document AN383; "Si47XX ANTENNA, SCHEMATIC, LAYOUT, AND DESIGN GUIDELINES"; rev 0.8; page 6; there is the following note:
 *            Crystal and digital audio mode cannot be used at the same time. Populate R1 and remove C10, C11, and X1 when using digital audio.
 *
 * @see Si47XX PROGRAMINGGUIDE; AN332 (REV 1.0); page 195.
 * @see Si47XX ANTENNA, SCHEMATIC, LAYOUT, AND DESIGN GUIDELINES"; AN383; rev 0.8; page 6;

 * @param uint8_t OSIZE Dgital Output Audio Sample Precision (0=16 bits, 1=20 bits, 2=24 bits, 3=8bits).
 * @param uint8_t OMONO Digital Output Mono Mode (0=Use mono/stereo blend ).
 * @param uint8_t OMODE Digital Output Mode (0=I2S, 6 = Left-justified, 8 = MSB at second DCLK after DFS pulse, 12 = MSB at first DCLK after DFS pulse).
 * @param uint8_t OFALL Digital Output DCLK Edge (0 = use DCLK rising edge, 1 = use DCLK falling edge)
 */
void SI4735_digitalOutputFormat(uint8_t OSIZE, uint8_t OMONO, uint8_t OMODE, uint8_t OFALL)
{
    si4735_digital_output_format df;
    df.refined.OSIZE = OSIZE;
    df.refined.OMONO = OMONO;
    df.refined.OMODE = OMODE;
    df.refined.OFALL = OFALL;
    df.refined.dummy = 0;
    SI4735_sendProperty(DIGITAL_OUTPUT_FORMAT, df.raw);
}

/**
 * @ingroup group13 Digital Audio setup
 *
 * @brief Enables digital audio output and configures digital audio output sample rate in samples per second (sps).
 *
 * ATTENTION: The document AN383; "Si47XX ANTENNA, SCHEMATIC, LAYOUT, AND DESIGN GUIDELINES"; rev 0.8; page 6; there is the following note:
 *            Crystal and digital audio mode cannot be used at the same time. Populate R1 and remove C10, C11, and X1 when using digital audio.
 *
 * @see Si47XX PROGRAMINGGUIDE; AN332 (REV 1.0); page 196.
 * @see Si47XX ANTENNA, SCHEMATIC, LAYOUT, AND DESIGN GUIDELINES; AN383; rev 0.8; page 6
 *
 * @param uint16_t DOSR Diital Output Sample Rate(32–48 ksps .0 to disable digital audio output).
 */
void SI4735_digitalOutputSampleRate(uint16_t DOSR)
{
	SI4735_sendProperty(DIGITAL_OUTPUT_SAMPLE_RATE, DOSR);
}

/**
 * @ingroup group13 Audio volume
 *
 * @brief Sets volume level (0  to 63)
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 62, 123, 170, 173 and 204
 *
 * @param uint8_t volume (domain: 0 - 63)
 */
void SI4735_setVolume(uint8_t volume)
{
	SI4735_sendProperty(RX_VOLUME, volume);
    Volume = volume;
}

/**
 * @ingroup group13 Audio volume
 * @brief Sets the audio on or off.
 * @details Useful to mute the audio output of the SI47XX device. This function does not work to reduce the pop in the speaker at start the system up.
 * @details If you want to remove the loud click or pop in the speaker at start, power down and power up commands, use setHardwareAudioMute with a external mute circuit.
 *
 * @see See Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 62, 123, 171
 * @see setHardwareAudioMute
 *
 * @param value if true, mute the audio; if false unmute the audio.
 */
void SI4735_setAudioMute(bool off)
{
    uint16_t value = (off) ? 3 : 0; // 3 means mute; 0 means unmute
    SI4735_sendProperty(RX_HARD_MUTE, value);
}



/**
 * @ingroup group13 Aud volume
 *
 * @brief Gets the current volume level.
 *
 * @see setVolume()
 *
 * @return volume (domain: 0 - 63)
 */
uint8_t SI4735_getVolume()
{
    return Volume;
}

/**
 * @ingroup group13 Audio volume
 *
 * @brief Set sound volume level Up
 *
 * @see setVolume()
 */
void SI4735_volumeUp()
{
    if (Volume < 63) Volume++;
    SI4735_setVolume(Volume);
}

/**
 * @ingroup group13 Audio volume
 *
 * @brief Set sound volume level Down
 *
 * @see setVolume()
 */
void SI4735_volumeDown()
{
    if (Volume) Volume--;
    SI4735_setVolume(Volume);
}

/**
 * @defgroup group16 FM RDS/RBDS
 * @todo RDS Dynamic PS or Scrolling PS support
 */

/*******************************************************************************
 * RDS implementation
 ******************************************************************************/

/**
 * @ingroup group16 RDS setup
 *
 * @brief  Starts the control member variables for RDS.
 *
 * @details This method is called by setRdsConfig()
 *
 * @see setRdsConfig()
 */
void SI4735_RdsInit()
{
	SI4735_clearRdsBuffer2A();
	SI4735_clearRdsBuffer2B();
	SI4735_clearRdsBuffer0A();
    rdsTextAdress2A = rdsTextAdress2B = lastTextFlagAB = rdsTextAdress0A = 0;
}

/**
 * @ingroup group16 RDS setup
 *
 * @brief Clear RDS buffer 2A (text)
 *
 */
void SI4735_clearRdsBuffer2A()
{
    for (int8_t i = 0; i < 65; i++) rds_buffer2A[i] = ' '; // Radio Text buffer - Program Information
    rds_buffer2A[64] = '\0';
}

/**
 * @ingroup group16 RDS setup
 *
 * @brief Clear RDS buffer 2B (text)
 *
 */
void SI4735_clearRdsBuffer2B()
{
    for (int8_t i = 0; i < 33; i++) rds_buffer2B[i] = ' '; // Radio Text buffer - Station Informaation
    rds_buffer2B[32] = '\0';
}
/**
 * @ingroup group16 RDS setup
 *
 * @brief Clear RDS buffer 0A (text)
 *
 */
void SI4735_clearRdsBuffer0A()
{
    for (int8_t i = 0; i < 9; i++) rds_buffer0A[i] = ' '; // Station Name buffer
    rds_buffer0A[8] = '\0';
}

/**
 * @ingroup group16 RDS setup
 *
 * @brief Sets RDS property
 *
 * @details Configures RDS settings to enable RDS processing (RDSEN) and set RDS block error thresholds.
 * @details When a RDS Group is received, all block errors must be less than or equal the associated block
 * error threshold for the group to be stored in the RDS FIFO.
 * @details IMPORTANT:
 * All block errors must be less than or equal the associated block error threshold
 * for the group to be stored in the RDS FIFO.
 * |Value | Description |
 * |------| ----------- |
 * | 0    | No errors |
 * | 1    | 1–2 bit errors detected and corrected |
 * | 2    | 3–5 bit errors detected and corrected |
 * | 3    | Uncorrectable |
 *
 * @details Recommended Block Error Threshold options:
 * | Exemples | Description |
 * | -------- | ----------- |
 * | 2,2,2,2  | No group stored if any errors are uncorrected |
 * | 3,3,3,3  | Group stored regardless of errors |
 * | 0,0,0,0  | No group stored containing corrected or uncorrected errors |
 * | 3,2,3,3  | Group stored with corrected errors on B, regardless of errors on A, C, or D |
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 104
 *
 * @param uint8_t RDSEN RDS Processing Enable; 1 = RDS processing enabled.
 * @param uint8_t BLETHA Block Error Threshold BLOCKA.
 * @param uint8_t BLETHB Block Error Threshold BLOCKB.
 * @param uint8_t BLETHC Block Error Threshold BLOCKC.
 * @param uint8_t BLETHD Block Error Threshold BLOCKD.
 */
void SI4735_setRdsConfig(uint8_t RDSEN, uint8_t BLETHA, uint8_t BLETHB, uint8_t BLETHC, uint8_t BLETHD)
{
    si47x_property property;
    si47x_rds_config config;

    SI4735_waitToSend();

    // Set property value
    property.value = FM_RDS_CONFIG;

    // Arguments
    config.arg.RDSEN = RDSEN;
    config.arg.BLETHA = BLETHA;
    config.arg.BLETHB = BLETHB;
    config.arg.BLETHC = BLETHC;
    config.arg.BLETHD = BLETHD;
    config.arg.DUMMY1 = 0;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);                  // Always 0x00 (I need to check it)
    Wire.write(property.raw.byteHigh); // Send property - High byte - most significant first
    Wire.write(property.raw.byteLow);  // Low byte
    Wire.write(config.raw[1]);         // Send the argments. Most significant first
    Wire.write(config.raw[0]);
    Wire.endTransmission();*/
    uint8_t dat[] = {SET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow, config.raw[1], config.raw[0]};
    SI4735_write(dat, sizeof(dat));

    _delay(1);//delayMicroseconds(550);

    SI4735_RdsInit();
}

/**
 * @ingroup group16 RDS setup
 *
 * @brief Configures interrupt related to RDS
 *
 * @details Use this method if want to use interrupt
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); page 103
 *
 * @param RDSRECV If set, generate RDSINT when RDS FIFO has at least FM_RDS_INT_FIFO_COUNT entries.
 * @param RDSSYNCLOST If set, generate RDSINT when RDS loses synchronization.
 * @param RDSSYNCFOUND set, generate RDSINT when RDS gains synchronization.
 * @param RDSNEWBLOCKA If set, generate an interrupt when Block A data is found or subsequently changed
 * @param RDSNEWBLOCKB If set, generate an interrupt when Block B data is found or subsequently changed
 */
void SI4735_setRdsIntSource(uint8_t RDSNEWBLOCKB, uint8_t RDSNEWBLOCKA, uint8_t RDSSYNCFOUND, uint8_t RDSSYNCLOST, uint8_t RDSRECV)
{
    si47x_property property;
    si47x_rds_int_source rds_int_source;

    if (currentTune != FM_TUNE_FREQ) return;

    rds_int_source.refined.RDSNEWBLOCKB = RDSNEWBLOCKB;
    rds_int_source.refined.RDSNEWBLOCKA = RDSNEWBLOCKA;
    rds_int_source.refined.RDSSYNCFOUND = RDSSYNCFOUND;
    rds_int_source.refined.RDSSYNCLOST = RDSSYNCLOST;
    rds_int_source.refined.RDSRECV = RDSRECV;
    rds_int_source.refined.DUMMY1 = 0;
    rds_int_source.refined.DUMMY2 = 0;

    property.value = FM_RDS_INT_SOURCE;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);                  // Always 0x00 (I need to check it)
    Wire.write(property.raw.byteHigh); // Send property - High byte - most significant first
    Wire.write(property.raw.byteLow);  // Low byte
    Wire.write(rds_int_source.raw[1]); // Send the argments. Most significant first
    Wire.write(rds_int_source.raw[0]);
    Wire.endTransmission();*/
    uint8_t dat[] = {SET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow, rds_int_source.raw[1], rds_int_source.raw[0]};
    SI4735_write(dat, sizeof(dat));

    SI4735_waitToSend();
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets the RDS status. Store the status in currentRdsStatus member. RDS COMMAND FM_RDS_STATUS
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 55 and 77
 *
 * @param INTACK Interrupt Acknowledge; 0 = RDSINT status preserved. 1 = Clears RDSINT.
 * @param MTFIFO 0 = If FIFO not empty, read and remove oldest FIFO entry; 1 = Clear RDS Receive FIFO.
 * @param STATUSONLY Determines if data should be removed from the RDS FIFO.
 */
void SI4735_getRdsStatus3(uint8_t INTACK, uint8_t MTFIFO, uint8_t STATUSONLY)
{
    si47x_rds_command rds_cmd;
    static uint16_t lastFreq;
    // checking current FUNC (Am or FM)
    if (currentTune != FM_TUNE_FREQ) return;

    if (lastFreq != currentWorkFrequency) {
        lastFreq = currentWorkFrequency;
        SI4735_clearRdsBuffer2A();
        SI4735_clearRdsBuffer2B();
        SI4735_clearRdsBuffer0A();
    }

    SI4735_waitToSend();

    rds_cmd.arg.INTACK = INTACK;
    rds_cmd.arg.MTFIFO = MTFIFO;
    rds_cmd.arg.STATUSONLY = STATUSONLY;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(FM_RDS_STATUS);
    Wire.write(rds_cmd.raw);
    Wire.endTransmission();*/
    uint8_t dat[] = {FM_RDS_STATUS, rds_cmd.raw};
    SI4735_write(dat, sizeof(dat));

    /*do
    {
    	SI4735_waitToSend();
        // Gets response information
        Wire.requestFrom(deviceAddress, 13);
        for (uint8_t i = 0; i < 13; i++) currentRdsStatus.raw[i] = Wire.read();
    } while (currentRdsStatus.resp.ERR);*/
    SI4735_read(currentRdsStatus.raw, 13);

    _delay(1);//delayMicroseconds(550);
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets RDS Status.
 *
 * @details Same result of calling getRdsStatus(0,0,0).
 * @details Please, call getRdsStatus(uint8_t INTACK, uint8_t MTFIFO, uint8_t STATUSONLY) instead getRdsStatus()
 * if you want other behaviour.
 *
 * @see SI4735::getRdsStatus(uint8_t INTACK, uint8_t MTFIFO, uint8_t STATUSONLY)
 */
void SI4735_getRdsStatus()
{
	SI4735_getRdsStatus3(0, 0, 0);
}

// See inlines methods / functions on SI4735.h

/**
 * @ingroup group16 RDS status
 *
 * @brief Returns the programa type.
 *
 * @details Read the Block A content
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 77 and 78
 *
 * @return BLOCKAL
 */
uint16_t SI4735_getRdsPI(void)
{
    if (SI4735_getRdsReceived() && SI4735_getRdsNewBlockA())
    {
        return currentRdsStatus.resp.BLOCKAL;
    }
    return 0;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Returns the Group Type (extracted from the Block B)
 *
 * @return BLOCKBL
 */
uint8_t SI4735_getRdsGroupType(void)
{
    si47x_rds_blockb blkb;

    blkb.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
    blkb.raw.highValue = currentRdsStatus.resp.BLOCKBH;

    return blkb.refined.groupType;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Returns the current Text Flag A/B
 *
 * @return uint8_t current Text Flag A/B
 */
uint8_t SI4735_getRdsFlagAB(void)
{
    si47x_rds_blockb blkb;

    blkb.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
    blkb.raw.highValue = currentRdsStatus.resp.BLOCKBH;

    return blkb.refined.textABFlag;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Returns the address of the text segment.
 *
 * @details 2A - Each text segment in version 2A groups consists of four characters. A messages of this group can be
 * have up to 64 characters.
 * @details 2B - In version 2B groups, each text segment consists of only two characters. When the current RDS status is
 *      using this version, the maximum message length will be 32 characters.
 *
 * @return uint8_t the address of the text segment.
 */
uint8_t SI4735_getRdsTextSegmentAddress(void)
{
    si47x_rds_blockb blkb;
    blkb.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
    blkb.raw.highValue = currentRdsStatus.resp.BLOCKBH;

    return blkb.refined.content;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets the version code (extracted from the Block B)
 *
 * @returns  0=A or 1=B
 */
uint8_t SI4735_getRdsVersionCode(void)
{
    si47x_rds_blockb blkb;

    blkb.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
    blkb.raw.highValue = currentRdsStatus.resp.BLOCKBH;

    return blkb.refined.versionCode;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Returns the Program Type (extracted from the Block B)
 *
 * @see https://en.wikipedia.org/wiki/Radio_Data_System
 *
 * @return program type (an integer betwenn 0 and 31)
 */
uint8_t SI4735_getRdsProgramType(void)
{
    si47x_rds_blockb blkb;

    blkb.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
    blkb.raw.highValue = currentRdsStatus.resp.BLOCKBH;

    return blkb.refined.programType;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Process data received from group 2B
 *
 * @param c  char array reference to the "group 2B" text
 */
void SI4735_getNext2Block(char *c)
{
    char raw[2];
    int8_t i, j;

    raw[1] = currentRdsStatus.resp.BLOCKDL;
    raw[0] = currentRdsStatus.resp.BLOCKDH;

    for (i = j = 0; i < 2; i++) {
        if (raw[i] == 0xD || raw[i] == 0xA) {
            c[j] = '\0';
            return;
        }
        if (raw[i] >= 32) {
            c[j] = raw[i];
            j++;
        } else {
            c[i] = ' ';
        }
    }
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Process data received from group 2A
 *
 * @param c  char array reference to the "group  2A" text
 */
void SI4735_getNext4Block(char *c)
{
    char raw[4];
    int8_t i, j;

    raw[0] = currentRdsStatus.resp.BLOCKCH;
    raw[1] = currentRdsStatus.resp.BLOCKCL;
    raw[2] = currentRdsStatus.resp.BLOCKDH;
    raw[3] = currentRdsStatus.resp.BLOCKDL;
    for (i = j = 0; i < 4; i++) {
        if (raw[i] == 0xD || raw[i] == 0xA) {
            c[j] = '\0';
            return;
        }
        if (raw[i] >= 32) {
            c[j] = raw[i];
            j++;
        } else {
            c[i] = ' ';
        }
    }
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets the RDS Text when the message is of the Group Type 2 version A
 *
 * @return char*  The string (char array) with the content (Text) received from group 2A
 */
char *SI4735_getRdsText(void)
{

    // Needs to get the "Text segment address code".
    // Each message should be ended by the code 0D (Hex)

    if (rdsTextAdress2A >= 16) rdsTextAdress2A = 0;

    SI4735_getNext4Block(&rds_buffer2A[rdsTextAdress2A * 4]);

    rdsTextAdress2A += 4;

    return rds_buffer2A;
}

/**
 * @ingroup group16 RDS status
 * @todo RDS Dynamic PS or Scrolling PS
 * @brief Gets the station name and other messages.
 *
 * @return char* should return a string with the station name.
 *         However, some stations send other kind of messages
 */
char *SI4735_getRdsText0A(void)
{
    si47x_rds_blockb blkB;

    // getRdsStatus();

    if (SI4735_getRdsReceived()) {
        if (!SI4735_getRdsGroupType()) {
            // Process group type 0
            blkB.raw.highValue = currentRdsStatus.resp.BLOCKBH;
            blkB.raw.lowValue = currentRdsStatus.resp.BLOCKBL;

            rdsTextAdress0A = blkB.group0.address;
            if ((rdsTextAdress0A >= 0) && (rdsTextAdress0A < 4)) {
            	SI4735_getNext2Block(&rds_buffer0A[rdsTextAdress0A << 1]);
                rds_buffer0A[8] = '\0';
                return rds_buffer0A;
            }
        }
    }
    return NULL;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets the Text processed for the 2A group
 *
 * @return char* string with the Text of the group A2
 */
char *SI4735_getRdsText2A(void)
{
    si47x_rds_blockb blkB;

    // getRdsStatus();
    if (SI4735_getRdsReceived()) {
        if (SI4735_getRdsGroupType() == 2 /* && getRdsVersionCode() == 0 */) {
            // Process group 2A
            // Decode B block information
            blkB.raw.highValue = currentRdsStatus.resp.BLOCKBH;
            blkB.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
            rdsTextAdress2A = blkB.group2.address;

            if ((rdsTextAdress2A >= 0) && (rdsTextAdress2A < 16)) {
            	SI4735_getNext4Block(&rds_buffer2A[rdsTextAdress2A << 2]);
                rds_buffer2A[63] = '\0';
                return rds_buffer2A;
            }
        }
    }
    return NULL;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets the Text processed for the 2B group
 *
 * @return char* string with the Text of the group AB
 */
char *SI4735_getRdsText2B(void)
{
    si47x_rds_blockb blkB;

    // getRdsStatus();
    // if (getRdsReceived())
    // {
    // if (getRdsNewBlockB())
    // {
    if (SI4735_getRdsGroupType() == 2 /* && getRdsVersionCode() == 1 */) {
        // Process group 2B
        blkB.raw.highValue = currentRdsStatus.resp.BLOCKBH;
        blkB.raw.lowValue = currentRdsStatus.resp.BLOCKBL;
        rdsTextAdress2B = blkB.group2.address;
        if ((rdsTextAdress2B >= 0) && (rdsTextAdress2B < 16)) {
        	SI4735_getNext2Block(&rds_buffer2B[rdsTextAdress2B << 1]);
            rds_buffer2B[32] = '\0';
            return rds_buffer2B;
        }
    }
    //  }
    // }
    return NULL;
}

/**
 * @ingroup group16 RDS status
 *
 * @brief Gets the RDS time and date when the Group type is 4
 *
 * @return char* a string with hh:mm +/- offset
 */
char *SI4735_getRdsTime()
{
    // Under Test and construction
    // Need to check the Group Type before.
    si47x_rds_date_time dt;

    uint16_t minute;
    uint16_t hour;

    if (SI4735_getRdsGroupType() == 4) {
        char offset_sign;
        int offset_h;
        int offset_m;

        // uint16_t y, m, d;

        dt.raw[4] = currentRdsStatus.resp.BLOCKBL;
        dt.raw[5] = currentRdsStatus.resp.BLOCKBH;
        dt.raw[2] = currentRdsStatus.resp.BLOCKCL;
        dt.raw[3] = currentRdsStatus.resp.BLOCKCH;
        dt.raw[0] = currentRdsStatus.resp.BLOCKDL;
        dt.raw[1] = currentRdsStatus.resp.BLOCKDH;

        // Unfortunately it was necessary to wotk well on the GCC compiler on 32-bit
        // platforms. See si47x_rds_date_time (typedef union) and CGG “Crosses boundary” issue/features.
        // Now it is working on Atmega328, STM32, Arduino DUE, ESP32 and more.
        minute = (dt.refined.minute2 << 2) | dt.refined.minute1;
        hour = (dt.refined.hour2 << 4) | dt.refined.hour1;

        offset_sign = (dt.refined.offset_sense == 1) ? '+' : '-';
        offset_h = (dt.refined.offset * 30) / 60;
        offset_m = (dt.refined.offset * 30) - (offset_h * 60);
        // sprintf(rds_time, "%02u:%02u %c%02u:%02u", dt.refined.hour, dt.refined.minute, offset_sign, offset_h, offset_m);
        sprintf(rds_time, "%02u:%02u %c%02u:%02u", hour, minute, offset_sign, offset_h, offset_m);

        return rds_time;
    }

    return NULL;
}

/**
 * @defgroup group17 Si4735-D60 Single Side Band (SSB) support
 *
 * @brief Single Side Band (SSB) implementation.<br>
 * First of all, it is important to say that the SSB patch content **is not part of this library**.
 * The paches used here were made available by Mr. Vadim Afonkin on his [Dropbox repository](https://www.dropbox.com/sh/xzofrl8rfaaqh59/AAA5au2_CVdi50NBtt0IivyIa?dl=0).
 * It is important to note that the author of this library does not encourage anyone to use the SSB patches content for commercial purposes.
 * In other words, this library only supports SSB patches, the patches themselves are not part of this library.
 *
 * @details This implementation was tested only on Si4735-D60  and SI4732-A10 devices.
 * @details SSB modulation is a refinement of amplitude modulation that one of the side band and the carrier are suppressed.
 *
 * @details What does SSB patch means?
 * In this context, a patch is a piece of software used to change the behavior of the SI4735-D60/SI4732-A10 device.
 * There is little information available about patching the SI4735-D60/SI4732-A10.
 *
 * The following information is the understanding of the author of this project and
 * it is not necessarily correct.
 *
 * A patch is executed internally (run by internal MCU) of the device. Usually,
 * patches are used to fixes bugs or add improvements and new features of the firmware
 * installed in the internal ROM of the device. Patches to the SI4735 are distributed
 * in binary form and have to be transferred to the internal RAM of the device by the
 * host MCU (in this case Arduino boards).
 * Since the RAM is volatile memory, the patch stored into the device gets lost when
 * you turn off the system. Consequently, the content of the patch has to be transferred
 * again to the device each time after turn on the system or reset the device.
 *
 * I would like to thank Mr Vadim Afonkin for making available the SSBRX patches for
 * SI4735-D60/SI4732-A10 on his Dropbox repository. On this repository you have two files,
 * amrx_6_0_1_ssbrx_patch_full_0x9D29.csg and amrx_6_0_1_ssbrx_patch_init_0xA902.csg.
 * It is important to know that the patch content of the original files is constant
 * hexadecimal representation used by the language C/C++. Actally, the original files
 * are in ASCII format (not in binary format).
 * If you are not using C/C++ or if you want to load the files directly to the SI4735,
 * you must convert the values to numeric value of the hexadecimal constants.
 * For example: 0x15 = 21 (00010101); 0x16 = 22 (00010110); 0x01 = 1 (00000001);
 * 0xFF = 255 (11111111);
 *
 * @details ATTENTION: The author of this project does not guarantee that procedures shown
 * here will work in your development environment. Given this, it is at your own risk
 * to continue with the procedures suggested here. This library works with the I²C
 * communication protocol and it is designed to apply a SSB extension PATCH to
 * SI4735-D60 and SI4732-A10 devices. Once again, the author disclaims any liability for any damage this
 * procedure may cause to your SI4735-D60 or SI4732-A10 or other devices that you are using.
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; pages 3 and 5
 */

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets the SSB Beat Frequency Offset (BFO).
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; pages 5 and 23
 *
 * @param offset 16-bit signed value (unit in Hz). The valid range is -16383 to +16383 Hz.
 */
void SI4735_setSSBBfo(int offset)
{

    si47x_property property;
    si47x_frequency bfo_offset;

    if (currentTune == FM_TUNE_FREQ) return;// Only for AM/SSB mode

    SI4735_waitToSend();

    property.value = SSB_BFO;
    bfo_offset.value = offset;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);                  // Always 0x00
    Wire.write(property.raw.byteHigh); // High byte first
    Wire.write(property.raw.byteLow);  // Low byte after
    Wire.write(bfo_offset.raw.FREQH);  // Offset freq. high byte first
    Wire.write(bfo_offset.raw.FREQL);  // Offset freq. low byte first
    Wire.endTransmission();*/
    uint8_t dat[] = {SET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow, bfo_offset.raw.FREQH, bfo_offset.raw.FREQL};
    SI4735_write(dat, sizeof(dat));


    _delay(1);//delayMicroseconds(550);
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets the SSB receiver mode.
 *
 * @details You can use this method for:
 * @details 1) Enable or disable AFC track to carrier function for receiving normal AM signals;
 * @details 2) Set the audio bandwidth;
 * @details 3) Set the side band cutoff filter;
 * @details 4) Set soft-mute based on RSSI or SNR;
 * @details 5) Enable or disbable automatic volume control (AVC) function.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param AUDIOBW SSB Audio bandwidth; 0 = 1.2kHz (default); 1=2.2kHz; 2=3kHz; 3=4kHz; 4=500Hz; 5=1kHz.
 * @param SBCUTFLT SSB side band cutoff filter for band passand low pass filter
 *                 if 0, the band pass filter to cutoff both the unwanted side band and high frequency
 *                  component > 2kHz of the wanted side band (default).
 * @param AVC_DIVIDER set 0 for SSB mode; set 3 for SYNC mode.
 * @param AVCEN SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
 * @param SMUTESEL SSB Soft-mute Based on RSSI or SNR.
 * @param DSP_AFCDIS DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
 */
void SI4735_setSSBConfig(uint8_t AUDIOBW, uint8_t SBCUTFLT, uint8_t AVC_DIVIDER, uint8_t AVCEN, uint8_t SMUTESEL, uint8_t DSP_AFCDIS)
{
    if (currentTune == FM_TUNE_FREQ) return;// Only AM/SSB mode

    currentSSBMode.param.AUDIOBW = AUDIOBW;
    currentSSBMode.param.SBCUTFLT = SBCUTFLT;
    currentSSBMode.param.AVC_DIVIDER = AVC_DIVIDER;
    currentSSBMode.param.AVCEN = AVCEN;
    currentSSBMode.param.SMUTESEL = SMUTESEL;
    currentSSBMode.param.DUMMY1 = 0;
    currentSSBMode.param.DSP_AFCDIS = DSP_AFCDIS;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets DSP AFC disable or enable
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param DSP_AFCDIS 0 = SYNC mode, AFC enable; 1 = SSB mode, AFC disable
 */
void SI4735_setSSBDspAfc(uint8_t DSP_AFCDIS)
{
    currentSSBMode.param.DSP_AFCDIS = DSP_AFCDIS;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets SSB Soft-mute Based on RSSI or SNR Selection:
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param SMUTESEL  0 = Soft-mute based on RSSI (default); 1 = Soft-mute based on SNR.
 */
void SI4735_setSSBSoftMute(uint8_t SMUTESEL)
{
    currentSSBMode.param.SMUTESEL = SMUTESEL;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets SSB Automatic Volume Control (AVC) for SSB mode
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param AVCEN 0 = Disable AVC; 1 = Enable AVC (default).
 */
void SI4735_setSSBAutomaticVolumeControl(uint8_t AVCEN)
{
    currentSSBMode.param.AVCEN = AVCEN;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets AVC Divider
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param AVC_DIVIDER  SSB mode, set divider = 0; SYNC mode, set divider = 3; Other values = not allowed.
 */
void SI4735_setSSBAvcDivider(uint8_t AVC_DIVIDER)
{
    currentSSBMode.param.AVC_DIVIDER = AVC_DIVIDER;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Sets SBB Sideband Cutoff Filter for band pass and low pass filters.
 *
 * @details 0 = Band pass filter to cutoff both the unwanted side band and high frequency components > 2.0 kHz of the wanted side band. (default)
 * @details 1 = Low pass filter to cutoff the unwanted side band.
 * Other values = not allowed.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param SBCUTFLT 0 or 1; see above
 */
void SI4735_setSBBSidebandCutoffFilter(uint8_t SBCUTFLT)
{
    currentSSBMode.param.SBCUTFLT = SBCUTFLT;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief SSB Audio Bandwidth for SSB mode
 *
 * @details 0 = 1.2 kHz low-pass filter  (default).
 * @details 1 = 2.2 kHz low-pass filter.
 * @details 2 = 3.0 kHz low-pass filter.
 * @details 3 = 4.0 kHz low-pass filter.
 * @details 4 = 500 Hz band-pass filter for receiving CW signal, i.e. [250 Hz, 750 Hz] with center
 * frequency at 500 Hz when USB is selected or [-250 Hz, -750 1Hz] with center frequency at -500Hz
 * when LSB is selected* .
 * @details 5 = 1 kHz band-pass filter for receiving CW signal, i.e. [500 Hz, 1500 Hz] with center
 * frequency at 1 kHz when USB is selected or [-500 Hz, -1500 1 Hz] with center frequency
 *     at -1kHz when LSB is selected.
 * @details Other values = reserved.
 *
 * @details If audio bandwidth selected is about 2 kHz or below, it is recommended to set SBCUTFLT[3:0] to 0
 * to enable the band pass filter for better high- cut performance on the wanted side band. Otherwise, set it to 1.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 24
 *
 * @param AUDIOBW the valid values are 0, 1, 2, 3, 4 or 5; see description above
 */
void SI4735_setSSBAudioBandwidth(uint8_t AUDIOBW)
{
    // Sets the audio filter property parameter
    currentSSBMode.param.AUDIOBW = AUDIOBW;

    SI4735_sendSSBModeProperty();
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Set the radio to AM function.
 *
 * @todo Adjust the power up parameters
 *
 * @details Set the radio to SSB (LW/MW/SW) function.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; pages 13 and 14
 * @see setAM()
 * @see setFrequencyStep()
 * @see void SI4735::setFrequency(uint16_t freq)
 *
 * @param usblsb upper or lower side band;  1 = LSB; 2 = USB
 */
void SI4735_setSSB0(uint8_t usblsb)
{
    // Is it needed to load patch when switch to SSB?
    // powerDown();
    // It starts with the same AM parameters.
    // setPowerUp(1, 1, 0, 1, 1, currentAudioMode);
	SI4735_setPowerUp(currentInterruptEnable, 0, 0, currentClockType, 1, currentAudioMode);
	SI4735_radioPowerUp();
    // ssbPowerUp(); // Not used for regular operation
	SI4735_setVolume(Volume); // Set to previus configured volume
    currentSsbStatus = usblsb;
    lastMode = SSB_CURRENT_MODE;
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @details Tunes the SSB (LSB or USB) receiver to a frequency between 520 and 30 MHz in 1 kHz steps.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; pages 13 and 14
 * @see setAM()
 * @see setFrequencyStep()
 * @see void SI4735::setFrequency(uint16_t freq)
 *
 * @param fromFreq minimum frequency for the band
 * @param toFreq maximum frequency for the band
 * @param initialFreq initial frequency
 * @param step step used to go to the next channel
 * @param usblsb SSB Upper Side Band (USB) and Lower Side Band (LSB) Selection;
 *               value 2 (banary 10) = USB;
 *               value 1 (banary 01) = LSB.
 */
void SI4735_setSSB(uint16_t fromFreq, uint16_t toFreq, uint16_t initialFreq, uint16_t step, uint8_t usblsb)
{
    currentMinimumFrequency = fromFreq;
    currentMaximumFrequency = toFreq;
    currentStep = step;

    if (initialFreq < fromFreq || initialFreq > toFreq) initialFreq = fromFreq;

    SI4735_setSSB0(usblsb);

    currentWorkFrequency = initialFreq;

    SI4735_setFrequency(currentWorkFrequency);
    // delayMicroseconds(550);
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Just send the property SSB_MOD to the device.  Internal use (privete method).
 */
void SI4735_sendSSBModeProperty()
{
    si47x_property property;
    property.value = SSB_MODE;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SET_PROPERTY);
    Wire.write(0x00);                  // Always 0x00
    Wire.write(property.raw.byteHigh); // High byte first
    Wire.write(property.raw.byteLow);  // Low byte after
    Wire.write(currentSSBMode.raw[1]); // SSB MODE params; freq. high byte first
    Wire.write(currentSSBMode.raw[0]); // SSB MODE params; freq. low byte after
    Wire.endTransmission();*/

    uint8_t dat[] = {SET_PROPERTY, 0, property.raw.byteHigh, property.raw.byteLow, currentSSBMode.raw[1], currentSSBMode.raw[0]};
    SI4735_write(dat, sizeof(dat));

    _delay(1);//delayMicroseconds(550);
}

/**
 * @ingroup group17 AGC
 *
 * @brief Queries SSB Automatic Gain Control STATUS
 * @details After call this method, you can call isAgcEnabled to know the AGC status and getAgcGainIndex to know the gain index value.
 *
 * @see AN332 REV 0.8 Universal Programming Guide Amendment for SI4735-D60 SSB and NBFM patches; page 18.
 *
 */
void SI4735_getSsbAgcStatus()
{
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SSB_AGC_STATUS);
    Wire.endTransmission();*/
    uint8_t byte = SSB_AGC_STATUS;
    SI4735_write(&byte, 1);

    do
    {
    	SI4735_waitToSend();
        //Wire.requestFrom(deviceAddress, 3);
        //currentAgcStatus.raw[0] = Wire.read(); // STATUS response
        //currentAgcStatus.raw[1] = Wire.read(); // RESP 1
        //currentAgcStatus.raw[2] = Wire.read(); // RESP 2
    	SI4735_read(&currentAgcStatus.raw[0], 3);
    } while (currentAgcStatus.refined.ERR);    // If error, try get AGC status again.

}

/**
 * @ingroup group17
 *
 * @brief Automatic Gain Control setup
 * @details Overrides the SSB AGC setting by disabling the AGC and forcing the gain index that ranges between 0 (minimum attenuation) and 37+ATTN_BACKUP (maximum attenuation).
 *
 * @param uint8_t SSBAGCDIS This param selects whether the AGC is enabled or disabled (0 = AGC enabled; 1 = AGC disabled);
 * @param uint8_t SSBAGCNDX If 1, this byte forces the AGC gain index. if 0,  Minimum attenuation (max gain)
 *
 */
void SI4735_setSsbAgcOverrite(uint8_t SSBAGCDIS, uint8_t SSBAGCNDX)
{
    si47x_agc_overrride agc;

    agc.arg.AGCDIS = SSBAGCDIS;
    agc.arg.AGCIDX = SSBAGCNDX;

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(SSB_AGC_OVERRIDE);
    Wire.write(agc.raw[0]);
    Wire.write(agc.raw[1]);
    Wire.endTransmission();*/
    uint8_t dat[] = {SSB_AGC_OVERRIDE, agc.raw[0], agc.raw[1]};
    SI4735_write(dat, sizeof(dat));

    SI4735_waitToSend();
}



/***************************************************************************************
 * SI47XX PATCH RESOURCES
 **************************************************************************************/

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Query the library information of the Si47XX device
 *
 * @details Used to confirm if the patch is compatible with the internal device library revision.
 *
 * @details You have to call this function if you are applying a patch on SI47XX (SI4735-D60/SI4732-A10).
 * @details The first command that is sent to the device is the POWER_UP command to confirm
 * that the patch is compatible with the internal device library revision.
 * @details The device moves into the powerup mode, returns the reply, and moves into the
 * powerdown mode.
 * @details The POWER_UP command is sent to the device again to configure
 * the mode of the device and additionally is used to start the patching process.
 * @details When applying the patch, the PATCH bit in ARG1 of the POWER_UP command must be
 * set to 1 to begin the patching process. [AN332 (REV 1.0) page 219].
 *
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 64 and 215-220.
 * @see struct si47x_firmware_query_library
 *
 * @return a struct si47x_firmware_query_library (see it in SI4735.h)
*/
si47x_firmware_query_library SI4735_queryLibraryId()
{
    si47x_firmware_query_library libraryID;

    SI4735_powerDown(); // Is it necessary

    // delay(500);

    SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(POWER_UP);
    Wire.write(0b00011111);          // Set to Read Library ID, disable interrupt; disable GPO2OEN; boot normaly; enable External Crystal Oscillator  .
    Wire.write(SI473X_ANALOG_AUDIO); // Set to Analog Line Input.
    Wire.endTransmission();*/
    uint8_t dat[] = {POWER_UP, 0x1f, SI473X_ANALOG_AUDIO};
    SI4735_write(dat, sizeof(dat));

    /*do
    {
    	SI4735_waitToSend();
        Wire.requestFrom(deviceAddress, 8);
        for (int8_t i = 0; i < 8; i++) libraryID.raw[i] = Wire.read();
    } while (libraryID.resp.ERR); // If error found, try it again.*/
    SI4735_read(libraryID.raw, 8);

    _delay(3);//delayMicroseconds(2500);

    return libraryID;
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief This method can be used to prepare the device to apply SSBRX patch
 *
 * @details Call queryLibraryId before call this method. Powerup the device by issuing the POWER_UP
 * command with FUNC = 1 (AM/SW/LW Receive).
 *
 * @see setMaxDelaySetFrequency()
 * @see MAX_DELAY_AFTER_POWERUP
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 64 and 215-220 and
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE AMENDMENT FOR SI4735-D60 SSB AND NBFM PATCHES; page 7.
 */
void SI4735_patchPowerUp()
{
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(POWER_UP);
    Wire.write(0b00110001);          // This is a condition for loading the patch: Set to AM, Enable External Crystal Oscillator; Set patch enable; GPO2 output disabled; CTS interrupt disabled. You can change this calling setSSB.
    Wire.write(SI473X_ANALOG_AUDIO); // This is a condition for loading the patch: Set to Analog Output. You can change this calling setSSB.
    Wire.endTransmission();*/
    uint8_t dat[] = {POWER_UP, 0x31, SI473X_ANALOG_AUDIO};
    SI4735_write(dat, sizeof(dat));

    _delay(maxDelayAfterPouwerUp);
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief This function can be useful for debug and test.
 */
void SI4735_ssbPowerUp()
{
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(POWER_UP);
    Wire.write(0b00010001); // This is a condition for loading the patch: Set to AM, Enable External Crystal Oscillator; Set patch enable; GPO2 output disabled; CTS interrupt disabled. You can change this calling setSSB.
    Wire.write(0b00000101); // This is a condition for loading the patch: Set to Analog Output. You can change this calling setSSB.
    Wire.endTransmission();*/
    uint8_t dat[] = {POWER_UP, 0x11, 5};
    SI4735_write(dat, sizeof(dat));

    _delay(3);//delayMicroseconds(2500);

    powerUp.arg.CTSIEN = currentInterruptEnable; // 1 -> Interrupt anabled;
    powerUp.arg.GPO2OEN = 0;                     // 1 -> GPO2 Output Enable;
    powerUp.arg.PATCH = 0;                       // 0 -> Boot normally;
    powerUp.arg.XOSCEN = currentClockType;       // 1 -> Use external crystal oscillator;
    powerUp.arg.FUNC = 1;                        // 0 = FM Receive; 1 = AM/SSB (LW/MW/SW) Receiver.
    powerUp.arg.OPMODE = 0b00000101;             // 0x5 = 00000101 = Analog audio outputs (LOUT/ROUT).
}

/**
 * @ingroup group17 Patch and SSB support
 *
 * @brief Transfers the content of a patch stored in a array of bytes to the SI4735 device.
 *
 * @details You must mount an array as shown below and know the size of that array as well.
 *
 *  @details It is importante to say  that patches to the SI4735 are distributed in binary form and
 *  have to be transferred to the internal RAM of the device by the host MCU (in this case Arduino).
 *  Since the RAM is volatile memory, the patch stored into the device gets lost when you turn off
 *  the system. Consequently, the content of the patch has to be transferred again to the device
 *  each time after turn on the system or reset the device.
 *
 *  @details The disadvantage of this approach is the amount of memory used by the patch content.
 *  This may limit the use of other radio functions you want implemented in Arduino.
 *
 *  @details Example of content:
 *  @code
 *  const PROGMEM uint8_t ssb_patch_content_full[] =
 *   { // SSB patch for whole SSBRX full download
 *       0x15, 0x00, 0x0F, 0xE0, 0xF2, 0x73, 0x76, 0x2F,
 *       0x16, 0x6F, 0x26, 0x1E, 0x00, 0x4B, 0x2C, 0x58,
 *       0x16, 0xA3, 0x74, 0x0F, 0xE0, 0x4C, 0x36, 0xE4,
 *          .
 *          .
 *          .
 *       0x16, 0x3B, 0x1D, 0x4A, 0xEC, 0x36, 0x28, 0xB7,
 *       0x16, 0x00, 0x3A, 0x47, 0x37, 0x00, 0x00, 0x00,
 *       0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x29};
 *
 *  const int size_content_full = sizeof ssb_patch_content_full;
 *  @endcode
 *
 * @see Si47XX PROGRAMMING GUIDE; ;AN332 (REV 1.0) pages 64 and 215-220.
 *
 *  @param ssb_patch_content point to array of bytes content patch.
 *  @param ssb_patch_content_size array size (number of bytes). The maximum size allowed for a patch is 15856 bytes
 *
 *  @return false if an error is found.
 */
bool SI4735_downloadPatch(const uint8_t *ssb_patch_content, const uint16_t ssb_patch_content_size)
{

/*
    uint8_t content;
    register int i, offset;
    // Send patch to the SI4735 device
    for (offset = 0; offset < (int)ssb_patch_content_size; offset += 8) {

        Wire.beginTransmission(deviceAddress);
        for (i = 0; i < 8; i++) {
            content = pgm_read_byte_near(ssb_patch_content + (i + offset));
            Wire.write(content);
        }
        Wire.endTransmission();


        // Testing download performance
        // approach 1 - Faster - less secure (it might crash in some architectures)
        delayMicroseconds(MIN_DELAY_WAIT_SEND_LOOP); // Need check the minimum value

        // approach 2 - More control. A little more secure than approach 1


        // approach 3 - same approach 2
        // waitToSend();

        // approach 4 - safer

    }

    _delay(1);//delayMicroseconds(250);
*/

    return true;
}

/**
 * @ingroup group17 Patch and SSB support
 * @brief Loads a given SSB patch content
 * @details Configures the Si4735-D60/SI4732-A10 device to work with SSB.
 *
 * @param ssb_patch_content        point to patch content array
 * @param ssb_patch_content_size   size of patch content
 * @param ssb_audiobw              SSB Audio bandwidth; 0 = 1.2kHz (default); 1=2.2kHz; 2=3kHz; 3=4kHz; 4=500Hz; 5=1kHz.
 */
void SI4735_loadPatch(const uint8_t *ssb_patch_content, const uint16_t ssb_patch_content_size, uint8_t ssb_audiobw)//1
{
	SI4735_queryLibraryId();
	SI4735_patchPowerUp();
    _delay(50);
    SI4735_downloadPatch(ssb_patch_content, ssb_patch_content_size);
    // Parameters
    // AUDIOBW - SSB Audio bandwidth; 0 = 1.2kHz (default); 1=2.2kHz; 2=3kHz; 3=4kHz; 4=500Hz; 5=1kHz;
    // SBCUTFLT SSB - side band cutoff filter for band passand low pass filter ( 0 or 1)
    // AVC_DIVIDER  - set 0 for SSB mode; set 3 for SYNC mode.
    // AVCEN - SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
    // SMUTESEL - SSB Soft-mute Based on RSSI or SNR (0 or 1).
    // DSP_AFCDIS - DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
    SI4735_setSSBConfig(ssb_audiobw, 1, 0, 0, 0, 1);
    _delay(25);
}

/**
 * @ingroup group17 Patch and SSB support
 * @brief Transfers the content of a patch stored in an eeprom to the SI4735 device.
 * @details To used this method, you will need an eeprom with the patch content stored into it.
 * @details This content have to be generated by the sketch [SI47XX_09_SAVE_SSB_PATCH_EEPROM](https://github.com/pu2clr/SI4735/tree/master/examples/TOOLS/SI47XX_09_SAVE_SSB_PATCH_EEPROM) on folder TOOLS.
 *
 * @see SI47XX_09_SAVE_SSB_PATCH_EEPROM
 * @see si4735_eeprom_patch_header
 * @ref https://github.com/pu2clr/SI4735/tree/master/examples/TOOLS/SI47XX_09_SAVE_SSB_PATCH_EEPROM
 *
 * @param eeprom_i2c_address
 * @return false if an error is found.
 */
si4735_eeprom_patch_header SI4735_downloadPatchFromEeprom(int eeprom_i2c_address)
{
    si4735_eeprom_patch_header eep;
    const int header_size = sizeof eep;
    uint8_t bufferAux[8];
    int offset, i;

    // Gets the EEPROM patch header information
    /*Wire.beginTransmission(eeprom_i2c_address);
    Wire.write(0x00); // offset Most significant Byte
    Wire.write(0x00); // offset Less significant Byte
    Wire.endTransmission();*/
    uint8_t dat[] = {0,0};
    SI4735_write_to(dat, sizeof(dat), eeprom_i2c_address);

    _delay(5);

    // The first two bytes of the header will be ignored.
    for (int k = 0; k < header_size; k += 8)
    {
        //Wire.requestFrom(eeprom_i2c_address, 8);
        //for (int i = k; i < (k + 8); i++) eep.raw[i] = Wire.read();
        i = k * 8;
        SI4735_read_from(&eep.raw[i], 8, eeprom_i2c_address);
    }


    // Transferring patch from EEPROM to SI4735 device
    offset = header_size;
    for (i = 0; i < (int)eep.refined.patch_size; i += 8) {
        // Reads patch content from EEPROM
        /*Wire.beginTransmission(eeprom_i2c_address);
        Wire.write((int)offset >> 8);   // header_size >> 8 wil be always 0 in this case
        Wire.write((int)offset & 0XFF); // offset Less significant Byte
        Wire.endTransmission();*/
        uint8_t dat[] = {offset >> 8, offset & 0xFF};
        SI4735_write_to(dat, sizeof(dat), eeprom_i2c_address);

        /*Wire.requestFrom(eeprom_i2c_address, 8);
        for (int j = 0; j < 8; j++)
        {
            bufferAux[j] = Wire.read();
        }*/
        SI4735_read_from(bufferAux, 8, eeprom_i2c_address);

        /*Wire.beginTransmission(deviceAddress);
        Wire.write(bufferAux, 8);
        Wire.endTransmission();*/
        SI4735_write(bufferAux, 8);

        SI4735_waitToSend();

        uint8_t cmd_status;
        /*Wire.requestFrom(deviceAddress, 1);
        cmd_status = Wire.read();*/
        // The SI4735 issues a status after each 8 byte transfered.Just the bit 7(CTS)should be seted.if bit 6(ERR)is seted, the system halts.
        SI4735_read(&cmd_status, 1);
        if (cmd_status != 0x80) {
            strcpy((char *)eep.refined.patch_id, "error!");
            return eep;
        }
        offset += 8; // Start processing the next 8 bytes
    }

    _delay(50);

    return eep;
}

/**
 * @defgroup group20 SI4735-D60 / SI4732-A10  NBFM
 *
 * @brief Narrow Band FM (Frequency Modulation) implementation.<br>
 * First of all, it is important to say that the NBFM patch content **is not part of this library**.
 * It is important to note that the author of this library does not encourage anyone to use the NBFM patches content for commercial purposes.
 * In other words, this library only supports NBFM patches, the patches themselves are not part of this library.
 *
 * @details This implementation was tested only on Si4735-D60  and SI4732-A10 devices.
 * @details This implementation is applicable to Si47035-D60 and SI4732-A10 when powering up the part in FM mode with the NBFM patch
 *
 * @details What does NBFM patch means?
 * In this context, a patch is a piece of software used to change the behavior of the SI4735-D60/SI4732-A10 device.
 * There is little information available about patching the SI4735-D60/SI4732-A10.
 *
 * The following information is the understanding of the author of this project and
 * it is not necessarily correct.
 *
 * A patch is executed internally (run by internal MCU) of the device. Usually,
 * patches are used to fixes bugs or add improvements and new features of the firmware
 * installed in the internal ROM of the device. Patches to the SI4735 or SI4732 are distributed
 * in binary form and have to be transferred to the internal RAM of the device by the
 * host MCU (in this case Arduino boards).
 * Since the RAM is volatile memory, the patch stored into the device gets lost when
 * you turn off the system. Consequently, the content of the patch has to be transferred
 * again to the device each time after turn on the system or reset the device.
 *
 * I would like to thank Mr Vadim Afonkin for making available the SSBRX patches for
 * SI4735-D60/SI4732-A10 on his Dropbox repository. On this repository you have two files,
 * amrx_6_0_1_ssbrx_patch_full_0x9D29.csg and amrx_6_0_1_ssbrx_patch_init_0xA902.csg.
 * It is important to know that the patch content of the original files is constant
 * hexadecimal representation used by the language C/C++. Actally, the original files
 * are in ASCII format (not in binary format).
 * If you are not using C/C++ or if you want to load the files directly to the SI4735,
 * you must convert the values to numeric value of the hexadecimal constants.
 * For example: 0x15 = 21 (00010101); 0x16 = 22 (00010110); 0x01 = 1 (00000001);
 * 0xFF = 255 (11111111);
 *
 * @details ATTENTION: The author of this project does not guarantee that procedures shown
 * here will work in your development environment. Given this, it is at your own risk
 * to continue with the procedures suggested here. This library works with the I²C
 * communication protocol and it is designed to apply a SSB extension PATCH to
 * SI4735-D60 and SI4732-A10 devices. Once again, the author disclaims any liability for any damage this
 * procedure may cause to your SI4735-D60 or SI4732-A10 or other devices that you are using.
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; pages 3 and 5
 */

/**
 * @ingroup group20 Patch and NBFM support
 *
 * @brief This method can be used to prepare the device to apply NBFM patch
 *
 * @details Call queryLibraryId before call this method. Powerup the device by issuing the POWER_UP
 * command with FUNC = 0 (FM Receiver).
 *
 * @see setMaxDelaySetFrequency()
 * @see MAX_DELAY_AFTER_POWERUP
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 64 and 215-220 and
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE AMENDMENT FOR SI4735-D60 SSB AND NBFM PATCHES; page 32.
 */
void SI4735_patchPowerUpNBFM()
{
	SI4735_waitToSend();

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(POWER_UP);
    Wire.write(0b00110000);          // This is a condition for loading the patch: Set to AM, Enable External Crystal Oscillator; Set patch enable; GPO2 output disabled; CTS interrupt disabled.
    Wire.write(SI473X_ANALOG_AUDIO); // This is a condition for loading the patch: Set to Analog Output. You can change this calling setNBFM.
    Wire.endTransmission();*/

    uint8_t dat[] = {POWER_UP, 0x30, SI473X_ANALOG_AUDIO};
    SI4735_write(dat, sizeof(dat));

    _delay(maxDelayAfterPouwerUp);
}

/**
 * @ingroup group20 Patch and NBFM support
 * @brief Loads a given NBFM patch content
 * @details Configures the Si4735-D60/SI4732-A10 device to work with NBFM.
 *
 * @param patch_content        point to patch content array
 * @param patch_content_size   size of patch content
 */
void SI4735_loadPatchNBFM(const uint8_t *patch_content, const uint16_t patch_content_size)
{
	SI4735_queryLibraryId();
	SI4735_patchPowerUpNBFM();
    _delay(50);
    SI4735_downloadPatch(patch_content, patch_content_size);
    // TODO
    _delay(25);
}

/**
 * @ingroup group20 Patch and NBFM support
 *
 * @brief Set the radio to FM function.
 *
 * @todo Adjust the power up parameters
 *
 * @details Set the radio to NBFM function.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; pages 32 and 14
 * @see setAM(), setSSB(), setFM()
 * @see setFrequencyStep()
 * @see void SI4735::setFrequency(uint16_t freq)
 */
void SI4735_setNBFM0()
{
    // Is it needed to load patch when switch to SSB?
    // powerDown();
    // It starts with the same AM parameters.
    // setPowerUp(1, 1, 0, 1, 1, currentAudioMode);
	SI4735_setPowerUp(currentInterruptEnable, 0, 0, currentClockType, 0, currentAudioMode);
	SI4735_radioPowerUp();
    currentTune = NBFM_TUNE_FREQ; // Force current tune to NBFM commands
    // ssbPowerUp(); // Not used for regular operation
    SI4735_setVolume(Volume); // Set to previus configured volume
    currentSsbStatus = 0;
    lastMode = NBFM_CURRENT_MODE;
}

/**
 * @ingroup group20 Patch and NBFM support
 *
 * @details Tunes the SSB (LSB or USB) receiver to a frequency between 64 and 108 MHz.
 *
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE;
 * @see setAM(), setFM(), setSSB()
 * @see setFrequencyStep()
 * @see void SI4735::setFrequency(uint16_t freq)
 *
 * @param fromFreq minimum frequency for the band
 * @param toFreq maximum frequency for the band
 * @param initialFreq initial frequency
 * @param step step used to go to the next channel

 */
void SI4735_setNBFM(uint16_t fromFreq, uint16_t toFreq, uint16_t initialFreq, uint16_t step)
{
    currentMinimumFrequency = fromFreq;
    currentMaximumFrequency = toFreq;
    currentStep = step;

    if (initialFreq < fromFreq || initialFreq > toFreq) initialFreq = fromFreq;

    SI4735_setNBFM0();

    currentWorkFrequency = initialFreq;
    SI4735_setFrequency(currentWorkFrequency);
}

/**
 * @ingroup   group20 Tune Frequency
 *
 * @brief Set the frequency to the corrent function of the Si4735 on NBFM mode
 * @details You have to call setup or setPowerUp before call setFrequency.
 *
 * @see maxDelaySetFrequency()
 * @see MAX_DELAY_AFTER_SET_FREQUENCY
 * @see Si47XX PROGRAMMING GUIDE; AN332 (REV 1.0); pages 70, 135
 * @see AN332 REV 0.8 UNIVERSAL PROGRAMMING GUIDE; page 39
 *
 * @param uint16_t  freq is the frequency to change. For example, FM => 10390 = 103.9 MHz; AM => 810 = 810 kHz.
 */
void SI4735_setFrequencyNBFM(uint16_t freq)
{
	SI4735_waitToSend(); // Wait for the si473x is ready.

    currentFrequency.value = freq;
    currentFrequencyParams.arg.FREQH = currentFrequency.raw.FREQH;
    currentFrequencyParams.arg.FREQL = currentFrequency.raw.FREQL;

    /*Wire.beginTransmission(deviceAddress);
    Wire.write(0x50);
    Wire.write(0x00); // Send a byte with FAST and  FREEZE information; if not FM must be 0;
    Wire.write(currentFrequency.raw.FREQH);
    Wire.write(currentFrequency.raw.FREQL);
    Wire.endTransmission();*/

    uint8_t dat[] = {0x50, 0, currentFrequency.raw.FREQH, currentFrequency.raw.FREQL};
    SI4735_write(dat, sizeof(dat));

    SI4735_waitToSend();                // Wait for the si473x is ready.
    currentWorkFrequency = freq; // check it
    _delay(250); // For some reason I need to delay here.
}



#endif

