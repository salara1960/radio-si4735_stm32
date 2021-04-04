
#include "IRremote.h"


#ifdef SET_IRED

//-----------------------------------------------------------------------------

volatile irparams_t irparams;
decode_results results;

//-----------------------------------------------------------------------------
int16_t decodeIRED(decode_results *results)
{
	if (irparams.rcvstate != STATE_STOP) return 0;

	results->rawbuf = irparams.rawbuf;
	results->rawlen = irparams.rawlen;
	results->overflow = irparams.overflow;

	if(irparams.rcvstate != STATE_STOP) return false;

	if (decodeHashIRED(results)) return 1;

	enIntIRED();

	return 0;
}
//-----------------------------------------------------------------------------
void enIntIRED()
{
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;

	__HAL_TIM_SET_COUNTER(portIRED, 0x0000);

	HAL_TIM_Base_Start_IT(portIRED);
}
//-----------------------------------------------------------------------------
void resumeIRED() // Restart the ISR state machine
{
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;
}
//-----------------------------------------------------------------------------
int16_t compareIRED(uint16_t oldval, uint16_t newval)
{
	     if (newval < oldval * .8) return 0;
	else if (oldval < newval * .8) return 2;
	else return 1;
}
//-----------------------------------------------------------------------------
int32_t decodeHashIRED(decode_results *results)
{
	int32_t hash = FNV_BASIS_32;

	if (results->rawlen < 6) return 0;

	for (int16_t i = 1; (i + 2) < results->rawlen; i++) {
		int16_t value = compareIRED(results->rawbuf[i], results->rawbuf[i + 2]);
		hash = (hash * FNV_PRIME_32) ^ value;
	}

	results->value = hash;
	results->bits = 32;

	return 1;
}
//-----------------------------------------------------------------------------


#endif







