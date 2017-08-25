/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "usb_hid_keyboard.h"

#include "Rotary.h"
#include "Uart.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile uint32_t Delayer = 0;
TPort keys;
uint8_t MustClearKey = 0;
uint16_t MustClearKey_Delay = 0;
uint16_t RotaryKeyTimePressed;
uint16_t RotaryMode = 0;
uint16_t RotaryModeOld = 0;
uint16_t RotaryModeTimeOut;
uint16_t RotarySpecialMode = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//
//
//
void KeyScan()
{
	(*(TKeys *)(&Keys.input)).Rotary_SW = !HAL_GPIO_ReadPin(ROTARY_SW_Port, ROTARY_SW_Pin);

	ScanPort( &Keys );
}


//
//
//
void HAL_SYSTICK_Callback()
{
	if(Delayer) { Delayer--; }
	if(MustClearKey_Delay) { MustClearKey_Delay--; }

	if(RotaryKeyTimePressed < 6000)
	{
		RotaryKeyTimePressed++;
	}

	if(RotaryModeTimeOut) { RotaryModeTimeOut--; }

	RotaryScan();
	KeyScan();
}


//
//
//
void SendReport(uint8_t key)
{
	struct mediaHID_t mediaHID;
	mediaHID.id = 2;

	mediaHID.keys = key;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));

	MustClearKey = 1;
	MustClearKey_Delay = 30;
}


// deze werkt leuk
// alleen :
// controle op niet veranderd heeft volgende probleem
// beginnen in mode 1
// 1 -> 2 -> 0 -> 1  is dan niet verandert  -> dus verlaten speciale mode !!
//
void Task()
{
  int16_t rot;

  struct mediaHID_t mediaHID;

  mediaHID.id = 2;
  mediaHID.keys = 0;

  while(1)
  {
	  // TURN => Volume up\down
	  rot = RotaryGet();

	  if(rot > 0)
	  {
		  switch(RotaryMode)
		  {
		  	  case 0:
				  	Uart_Puts("+\r\n");
				    SendReport(USB_HID_VOL_UP);
		  		  break;

		  	  case 1:
					Uart_Puts("Next\r\n");
					SendReport(USB_HID_SCAN_NEXT);
					RotaryModeTimeOut = 2000;
		  		  break;

		  	  case 2:
					Uart_Puts("FFW\r\n");
					SendReport(USB_HID_FFW);
					RotaryModeTimeOut = 2000;
		  		  break;
		  }
	  }

	  if(rot < 0)
	  {
		  switch(RotaryMode)
		  {
		  	  case 0:
				  	Uart_Puts("-\r\n");
				    SendReport(USB_HID_VOL_DEC);
		  		  break;

		  	  case 1:
					Uart_Puts("Prev\r\n");
					SendReport(USB_HID_SCAN_PREV);
					RotaryModeTimeOut = 2000;
		  		  break;

		  	  case 2:
					Uart_Puts("REV\r\n");
					SendReport(USB_HID_REW);
					RotaryModeTimeOut = 2000;
		  		  break;
		  }
	  }


	  //
	  // rotary key pressed
	  if((Keys.down & KEYS_ROTARY_SW) > 0)
	  {
		  Keys.down &= ~KEYS_ROTARY_SW;

		  RotaryKeyTimePressed = (RotaryMode * 750);

		  RotaryModeOld = RotaryMode;
	  }

	  // rotary key still pressed
	  if((Keys.level & KEYS_ROTARY_SW) > 0)
	  {
		  switch(RotaryMode)
		  {
		  	  case 0x00:
		      case 0x80:
		  		  if(RotaryKeyTimePressed > 750)
		  		  {
				  	  Uart_Puts("Special mode 1 <Enter>\r\n");
		  			  RotaryMode = 0x81;
		  			  RotaryModeOld = 0x10;
		  		  }
		  		  break;

		      case 0x01:
		  	  case 0x81:
		  		  if(RotaryKeyTimePressed > 1500)
		  		  {
				  	  Uart_Puts("Special mode 2 <Enter>\r\n");
		  			  RotaryMode = 0x82;
		  			  RotaryModeOld = 0x10;
		  		  }
		  		  break;

		  	  case 0x02:
		  	  case 0x82:
		  		  if(RotaryKeyTimePressed >= 2250)
		  		  {
				  	  Uart_Puts("Special mode <Exit>\r\n");
				  	  RotaryKeyTimePressed = 0;
		  			  RotaryMode = 0x80;
		  			  RotaryModeOld = 0x10;
		  		  }
		  		  break;
		  }
	  }

	  // rotary key released
	  if((Keys.up & KEYS_ROTARY_SW) > 0)
	  {
			Keys.up &= ~KEYS_ROTARY_SW;

			// kort gedrukt in normale mode anders tijd altijd > 750
			if(RotaryKeyTimePressed < 750)
			{
				// in normale mode -> pauze
				if(RotaryMode == 0x00)
				{
					HAL_GPIO_TogglePin(LED_PC13_GPIO_Port, LED_PC13_Pin);

					Uart_Puts("||\r\n");
					SendReport(USB_HID_PAUSE);
				}
			}
			else
			{
				// has RotaryMode changed ?
				if((RotaryModeOld == RotaryMode) || ((RotaryModeOld | 0x80) == RotaryMode))
				{
					// no -> short press -> exit special mode
					RotaryMode = 0x00;
					Uart_Puts("Special mode <Exit> (no change aka short press while in special mode)\r\n");
				}

				// langer gedrukt
				// in speciale mode
				if(RotaryMode >= 0x80)
				{
					// clear select vlag
					RotaryMode &= ~0x80;
					// start timeout
					RotaryModeTimeOut = 2000;
					Uart_Puts("RotaryModeTimeOut gestart\r\n");
				}
			}

			// alway clear flag
			RotaryMode &= ~0x80;
	  }

	  // time out -> alleen in een speciale mode van toepassing
	  if((RotaryMode > 0x00) && (RotaryMode < 0x80) && (RotaryModeTimeOut == 0))
	  {
		  Uart_Puts("Special mode <Exit>  (timeout)\r\n");
		  RotaryMode = 0x00;
	  }


	  // send a usb message key has been released
	  if((MustClearKey) && (MustClearKey_Delay == 0))
	  {
		  MustClearKey  = 0;

		  mediaHID.keys = 0;
		  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mediaHID, sizeof(struct mediaHID_t));
	  }


	  // delay task
	  if(Delayer == 0)
	  {
	  	  Delayer = 100;

	  	  HAL_GPIO_TogglePin(LED_PC13_GPIO_Port, LED_PC13_Pin);
	  }

	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
#if 0
	struct keyboardHID_t keyboardHID;
	keyboardHID.id = 1;
	keyboardHID.modifiers = 0;
	keyboardHID.key1 = 0;
	keyboardHID.key2 = 0;
	keyboardHID.key3 = 0;
#endif

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  Uart_Setup();
  Uart_Puts("Gestart\r\n");

  KeyScan();
  KeyScan();

  RotaryStart();

  // ena usb
  HAL_GPIO_WritePin(USB_PULL_GPIO_Port, USB_PULL_Pin, GPIO_PIN_SET);

  Task();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_PC13_Pin */
  GPIO_InitStruct.Pin = LED_PC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PC13_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(USB_PULL_GPIO_Port, USB_PULL_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = USB_PULL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PULL_GPIO_Port, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = ROTARY_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(ROTARY_A_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROTARY_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROTARY_B_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ROTARY_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROTARY_SW_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
