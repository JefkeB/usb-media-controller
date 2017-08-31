/*
 * WS2812B.h
 *
 *  Created on: 31 aug. 2017
 *      Author: JefPC
 */

#ifndef WS2812B_LEDS_H_
#define WS2812B_LEDS_H_

#include <stdint.h>
#include <stdbool.h>


//
//
//
typedef struct
{
	uint8_t	R;
	uint8_t G;
	uint8_t B;
}TLedColor;

//
//
//
void WS2812B_Leds_Init();
void WS2812B_Leds_AllLedsOff();
bool WS2818B_Leds_Busy();
void WS2812B_Leds_SingleColor(uint8_t R, uint8_t G, uint8_t B);
void WS2812B_Leds_Update();

#endif /* WS2812B_LEDS_H_ */
