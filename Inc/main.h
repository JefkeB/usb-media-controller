/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "ScanPort.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_PC13_Pin GPIO_PIN_13
#define LED_PC13_GPIO_Port GPIOC



/* USER CODE BEGIN Private defines */

#define	USB_PULL_Pin			GPIO_PIN_15
#define USB_PULL_GPIO_Port		GPIOA

// Rotary
#define ROTARY_PB12_Pin 		GPIO_PIN_12
#define ROTARY_PB12_GPIO_Port 	GPIOB

#define ROTARY_PB13_Pin			GPIO_PIN_13
#define ROTARY_PB13_GPIO_Port 	GPIOB

#define ROTARY_PB14_Pin 		GPIO_PIN_14
#define ROTARY_PB14_GPIO_Port 	GPIOB

#define ROTARY_A_Pin			ROTARY_PB12_Pin
#define ROTARY_A_Bit			12
#define ROTARY_A_Port			ROTARY_PB12_GPIO_Port

#define ROTARY_B_Pin			ROTARY_PB13_Pin
#define ROTARY_B_Bit			13
#define ROTARY_B_Port			ROTARY_PB13_GPIO_Port

#define ROTARY_SW_Pin			ROTARY_PB14_Pin
#define ROTARY_SW_Bit			14
#define ROTARY_SW_Port			ROTARY_PB14_GPIO_Port
#define KEYS_ROTARY_SW			0x01


typedef struct TKeys
{
	uint8_t Rotary_SW	: 1;

	uint8_t nx			: 7;
} TKeys ;

TPort Keys;


/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
