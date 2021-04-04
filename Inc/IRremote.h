#ifndef IRREMOTE__h
#define IRREMOTE__h


#include "hdr.h"

#ifdef SET_IRED

#include "main.h"

//-----------------------------------------------------------------------------

#define RECIV_PIN (HAL_GPIO_ReadPin(IRED_GPIO_Port, IRED_Pin))

#define RAWBUF 256

#define STATE_IDLE      2
#define STATE_MARK      3
#define STATE_SPACE     4
#define STATE_STOP      5
#define STATE_OVERFLOW  6

#define GAP_TICKS       100

#define MARK   0
#define SPACE  1

#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

//-----------------------------------------------------------------------------
/*
typedef enum
{
	UNUSED = -1,
	UNKNOWN = 0,
	RC5,
	RC6,
	NEC,
	SONY,
	PANASONIC,
	JVC,
	SAMSUNG,
	WHYNTER,
	AIWA_RC_T501,
	LG,
	SANYO,
	MITSUBISHI,
	DISH,
	SHARP,
	DENON,
	PRONTO,
} decode_type_t;
*/
typedef struct {// The fields are ordered to reduce memory over caused by struct-padding
	uint8_t       rcvstate;        // State Machine state
	uint8_t       rawlen;          // counter of entries in rawbuf
	uint16_t      timer;           // State timer, counts 50uS ticks.
	uint16_t      rawbuf[RAWBUF];  // raw data
	uint8_t       overflow;        // Raw buffer overflow occurred
} irparams_t;

// Results returned from the decoder
typedef struct {
	//decode_type_t decode_type; // UNKNOWN, NEC, SONY, RC5, ...
	//uint16_t address; // Used by Panasonic & Sharp [16-bits]
	uint32_t value; // Decoded value [max 32-bits]
	int16_t bits; // Number of bits in decoded value
	volatile uint16_t *rawbuf; // Raw intervals in 50uS ticks
	int16_t rawlen; // Number of records in rawbuf
	int16_t overflow; // true iff IR raw code too long
} decode_results;

//-----------------------------------------------------------------------------

volatile irparams_t irparams;
decode_results results;

//-----------------------------------------------------------------------------

int16_t decodeIRED(decode_results *results);
int32_t decodeHashIRED(decode_results *results);
void enIntIRED();
void resumeIRED();
//uint8_t my_isIdle();

//-----------------------------------------------------------------------------

#endif

#endif


