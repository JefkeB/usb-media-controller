/*
 * Rotary.c
 *
 *  Created on: 12 dec. 2015
 *      Author: Jef-PC
 *
 *
 *      Module is based on : https://github.com/0xPIT/encoder/tree/arduino
 *
 */
#include "Rotary.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

#include "main.h"

// ----------------------------------------------------------------------------
// Acceleration configuration (for 1000Hz scan rate)
//
#define ENC_ACCEL_ENABLED	  1
#define ENC_ACCEL_TOP      3072   // max. acceleration: *12 (val >> 8)
#define ENC_ACCEL_INC        25
#define ENC_ACCEL_DEC         2

#define ROTARY_A_BIT	ROTARY_A_Bit
#define ROTARY_B_BIT	ROTARY_B_Bit
#define ROTARY_BITS_VAL	3

#define ROTARY_A_VAL	(1 << ROTARY_A_BIT)
#define ROTARY_B_VAL	(1 << ROTARY_B_BIT)

#define PULSES_PER_STEP	2

//
//
uint16_t			delta = 0;
uint16_t			last  = 0;
volatile uint16_t 	acceleration;

const uint16_t		steps = PULSES_PER_STEP;



//
//
void RotaryStart()
{
	//uint32_t 		val = LPC_GPIO3->DATA;
	uint32_t		val = GPIOB->IDR;

	if((val & ROTARY_B_VAL) != 0)
	{
		last = 3;
	}

	if((val & ROTARY_A_VAL) != 0)
	{
		last ^= 1;
	}
}


//
//
void RotaryScan()
{
	//uint32_t 		val = LPC_GPIO3->DATA;
	uint32_t		val = GPIOB->IDR;

	static uint8_t	last;
	uint8_t			curr = 0;
	uint8_t			moved = 0;

	#if ENC_ACCEL_ENABLED
		acceleration -= ENC_ACCEL_DEC;
		if (acceleration & 0x8000) { // handle overflow of MSB is set
		  acceleration = 0;
		}
	#endif

	if((val & ROTARY_B_VAL) != 0)
	{
		curr = 3;
	}

	if((val & ROTARY_A_VAL) != 0)
	{
		curr ^= 1;
	}

	int8_t diff = last - curr;

	if(diff & 0x01)
	{
		last = curr;
		delta += (diff & 2) - 1;
		moved = 1;
	}


	#if ENC_ACCEL_ENABLED
	  if (moved) {
	    // increment accelerator if encoder has been moved
	    if (acceleration <= (ENC_ACCEL_TOP - ENC_ACCEL_INC)) {
	      acceleration += ENC_ACCEL_INC;
	    }
	  }

	#endif
}


//
//
int16_t RotaryGet()
{
	int16_t		val;

	val = delta;

	if(steps == 2) delta = val & 1;
	else if (steps == 4) delta = val & 3;
	else delta = 0;

	if(steps == 4) val >>= 2;
	if(steps == 2) val >>= 1;

	int16_t r = 0;
	int16_t accel = 0;

	#if ENC_ACCEL_ENABLED
		accel = acceleration >> 8;
	#endif

	if(val < 0) {
		r -= 1 + accel;
	} else if (val > 0) {
		r += 1 + accel;
	}

	return r;
}
