/*
 * WS2812B.h
 *
 *  Created on: 31 aug. 2017
 *      Author: JefPC
 */

#ifndef WS2812B_H_
#define WS2812B_H_

#include <stdint.h>
#include <stdbool.h>

//
//
//
extern const uint8_t _400ns;
extern const uint8_t _450ns;
extern const uint8_t _800ns;
extern const uint8_t _850ns;

//
//
//
extern volatile bool WS2812B_DmaIsBusy;
//
//
//
void WS2812B_Start();
void WS2812B_SendData(uint8_t* data, uint16_t length);


#endif /* WS2812B_H_ */
