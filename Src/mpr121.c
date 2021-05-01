
#include "hdr.h"

#ifdef SET_KBD

#include "main.h"
#include "mpr121.h"


//******************************************************************************************
void chkErrKBD(HAL_StatusTypeDef ret)
{
	if (ret != HAL_OK) devError |= devKBD; else devError &= ~devKBD;
}
//-------------------------------------------------------------------------------------------
void kbdWriteRegs(uint8_t reg, uint8_t *data, size_t len)
{
	HAL_StatusTypeDef rt = HAL_I2C_Mem_Write(portKBD, kbdAddr << 1, reg, sizeof(reg), data, len, min_wait_ms);

	chkErrKBD(rt);
}
//-------------------------------------------------------------------------------------------
void kbdReadRegs(uint8_t reg, uint8_t *data, size_t len)
{
	HAL_StatusTypeDef rt = HAL_I2C_Master_Transmit(portKBD, kbdAddr << 1, &reg, 1, min_wait_ms);
	rt |= HAL_I2C_Master_Receive(portKBD, kbdAddr << 1, data, len, max_wait_ms);

	chkErrKBD(rt);
}
//-------------------------------------------------------------------------------------------
bool KBD_getAddr(int16_t *addr)
{
	int16_t i = 0, adr = KBD_ADDR1;

	uint8_t byte = 0x63;
	for (i = 0; i < 4; i++) {
		adr += i;
		HAL_Delay(50);
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
uint8_t byte = 0;

	byte = 0x63;
	kbdWriteRegs(SRST, &byte, 1);
	HAL_Delay(1);


	// read Touch Status register
	kbdReadRegs(TS2, &byte, 1);
	if (byte & 0x80) {//clear OVCF
		byte = 0x80;
		kbdWriteRegs(TS2, &byte, 1);
		//
		kbdReadRegs(TS2, &byte, 1);
	}
	if (byte & 0x80) {
		//Report(__func__, true, "ERROR MPR121: TS2=0x%02X\n", byte);
		return false;
	}

	// Put the MPR into setup mode
	byte = 0;
	kbdWriteRegs(ECR, &byte, 1);// turn off all electrodes to stop

	uint8_t data[] = { 1, 1, 0, 0, 1, 1, 0xFF, 2 };
	kbdWriteRegs(MHDR, data, sizeof(data));//0x2B ...

/**/
	data[0] = 0x11;
	data[1] = 0xD0;//0x5C - 16uA
	data[2] = 4;//0x14;//0x5D - Period set to 16 ms (Default)
	kbdWriteRegs(DTR, data, 3);//0x5B ... 0x5D

	memset(data, 0, 5);
	kbdWriteRegs(ACCR0, data, 5);//0x7B ... 0x7F

	//byte = 0xCC;
	//kbdWriteRegs(ECR, &byte, 1);
/**/

	// apply next setting for all electrodes
	uint8_t dat[] = {
			TOU_THRESH, REL_THRESH, TOU_THRESH, REL_THRESH,
			TOU_THRESH, REL_THRESH, TOU_THRESH, REL_THRESH,
			TOU_THRESH, REL_THRESH, TOU_THRESH, REL_THRESH,
			TOU_THRESH, REL_THRESH, TOU_THRESH, REL_THRESH,
			TOU_THRESH, REL_THRESH, TOU_THRESH, REL_THRESH,
			TOU_THRESH, REL_THRESH, TOU_THRESH, REL_THRESH
	};
	kbdWriteRegs(E0TTH, dat, sizeof(dat));

	// Proximity Settings
	memset(dat + 2, 0, 9);
	dat[0] = 0xff;//,   //MHD_Prox_R
	dat[1] = 0xff;//,   //NHD_Prox_R
	dat[4] = 1;//,   //MHD_Prox_F
	dat[5] = 1;//,   //NHD_Prox_F
	dat[6] = 0xff;//,   //NCL_Prox_F
	dat[7] = 0xff;//,   //FDL_Prox_F
	kbdWriteRegs(MHDPROXR, dat, 11);

	//line 13
	dat[0] = TOU_THRESH;// Touch Threshold
	dat[1] = REL_THRESH;// Release Threshold
	kbdWriteRegs(E12TTH, dat, 2);

	dat[0] = 4;
	dat[1] = 0x0c;
	kbdWriteRegs(AFE2, dat, 2);

	return true;
}
//-------------------------------------------------------------------------------------------
uint16_t kbd_get_touch()// get touch status
{
	uint8_t data[2] = {0};
	kbdReadRegs(TS1, data, 2);
	uint16_t ret = data[0];
	ret |= ((data[1] & 0x1f) << 8);

	//return ret;
	if (!ret) return ret;

	int tn = 0;
	for (int j = 0; j < NUM_OF_ELECTRODES - 1; j++) if ((ret & (1 << j))) tn++;

	uint16_t key = 0;
	if (tn == 1) {
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
	}

	return key;
}
//-----------------------------------------------------------------------------------------

//******************************************************************************************

#endif

