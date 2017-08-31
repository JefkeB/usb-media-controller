/*
 * WS2812B.c
 *
 *  Created on: 31 aug. 2017
 *      Author: JefPC
 */

#include <WS2812B.h>
#include <WS2812B_leds.h>

#define NoOfLeds  16
#define DmaBuffer_Length (NoOfLeds * 24)


TLedColor Leds[NoOfLeds];
uint8_t DmaBuffer[DmaBuffer_Length];


//
//
//
void WS2812B_Leds_Init()
{
	uint16_t idx;

	for(idx = 0; idx < NoOfLeds; idx++)
	{
		Leds[idx].R = 0x00;
		Leds[idx].G = 0x00;
		Leds[idx].B = 0x00;
	}

	for(idx = 0; idx < DmaBuffer_Length; idx++)
	{
		DmaBuffer[idx] = 0;
	}

	WS2812B_Start();
}


//
//
//
bool WS2818B_Leds_Busy()
{
	return WS2812B_DmaIsBusy;
}


//
//
//
void Leds2DmaBuffer()
{
	uint8_t mask;
	uint16_t dmaBufferIdx = 0;

	for(uint16_t idx = 0; idx < NoOfLeds; idx++)
	{
		// G
		mask = 0x80;
		for(uint16_t b = 0; b < 8; b++)
		{
			if((Leds[idx].G & mask) > 0)
			{
				// 1
				DmaBuffer[dmaBufferIdx++] = _800ns;
				//DmaBuffer[dmaBufferIdx++] = _450ns;
			}
			else
			{
				// 0
				DmaBuffer[dmaBufferIdx++] = _400ns;
				//DmaBuffer[dmaBufferIdx++] = _850ns;
			}

			mask >>= 1;
		}

		// R
		mask = 0x80;
		for(uint16_t b = 0; b < 8; b++)
		{
			if((Leds[idx].R & mask) > 0)
			{
				// 1
				DmaBuffer[dmaBufferIdx++] = _800ns;
				//DmaBuffer[dmaBufferIdx++] = _450ns;
			}
			else
			{
				// 0
				DmaBuffer[dmaBufferIdx++] = _400ns;
				//DmaBuffer[dmaBufferIdx++] = _850ns;
			}

			mask >>= 1;
		}

		// B
		mask = 0x80;
		for(uint16_t b = 0; b < 8; b++)
		{
			if((Leds[idx].B & mask) > 0)
			{
				// 1
				DmaBuffer[dmaBufferIdx++] = _800ns;
				//DmaBuffer[dmaBufferIdx++] = _450ns;
			}
			else
			{
				// 0
				DmaBuffer[dmaBufferIdx++] = _400ns;
				//DmaBuffer[dmaBufferIdx++] = _850ns;
			}

			mask >>= 1;
		}
	}
}


//
//
//
void WS2812B_Leds_SingleColor(uint8_t R, uint8_t G, uint8_t B)
{
	for(uint8_t idx = 0; idx < NoOfLeds; idx++)
	{
		Leds[idx].R = R;
		Leds[idx].G = G;
		Leds[idx].B = B;
	}
}


//
//
//
void WS2812B_Leds_AllLedsOff()
{
	WS2812B_Leds_SingleColor(0x00, 0x00, 0x00);

	WS2812B_Leds_Update();
}


//
//
//
void WS2812B_Leds_Update()
{
	Leds2DmaBuffer();
	WS2812B_SendData(DmaBuffer, DmaBuffer_Length);
}
