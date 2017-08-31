/*
 * WS2812B.c
 *
 *  Created on: 31 aug. 2017
 *      Author: JefPC
 */


#include <WS2812B.h>
#include "stm32f1xx_hal.h"

//
//
//
const uint8_t _400ns = 10;
const uint8_t _450ns = 11;
const uint8_t _800ns = 19;
const uint8_t _850ns = 20;

volatile bool WS2812B_DmaIsBusy;

//
//
//
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;


//
//
//
void MX_TIM3_Init();
void MX_DMA_Init();
void DMA_Setup();



//
//
//
void WS2812B_Start()
{
  WS2812B_DmaIsBusy = false;

  MX_DMA_Init();

  __HAL_RCC_TIM3_CLK_ENABLE();
  MX_TIM3_Init();

  // TIM3 GPIO Configuration
  // PA6     ------> TIM3_CH1
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_TIM_Base_Start(&htim3);

  // start pwm with set value
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}


//
//
//
void WS2812B_SendData(uint8_t* data, uint16_t length)
{
  WS2812B_DmaIsBusy = true;
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)data, length);
}


//
//
//
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch1_trig);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */
  WS2812B_DmaIsBusy = false;
  /* USER CODE END DMA1_Channel6_IRQn 1 */
}


// TIM3 init function
//
//
void MX_TIM3_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;

	//htim3.hdma[TIM_DMA_ID_CC1] = &hdma_tim3_ch1_trig;

	htim3.Init.Prescaler = (uint16_t) (72000000 / 24000000) - 1;	// 2.4MHz
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 29;	// 800Khz -> ca 41.66ns/bit
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}

	//HAL_TIM_MspPostInit(&htim3);
}


//Enable DMA controller clock
//
//
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA_Setup();
}


//
//
//
void DMA_Setup()
{
    // TIM3 DMA Init
    // TIM3_CH1_TRIG Init
    hdma_tim3_ch1_trig.Instance = DMA1_Channel6;

    hdma_tim3_ch1_trig.Init.Direction = DMA_MEMORY_TO_PERIPH;

    hdma_tim3_ch1_trig.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch1_trig.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;

    hdma_tim3_ch1_trig.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch1_trig.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;

    hdma_tim3_ch1_trig.Init.Mode = DMA_NORMAL;
    hdma_tim3_ch1_trig.Init.Priority = DMA_PRIORITY_MEDIUM;
    /*if (HAL_DMA_Init(&hdma_tim3_ch1_trig) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }*/

    // link hdma_tim to hdma[TIM_DMA_ID_CC1]
    __HAL_LINKDMA(&htim3,hdma[TIM_DMA_ID_CC1],hdma_tim3_ch1_trig);

    // Initialize timer DMA handle
    HAL_DMA_Init(htim3.hdma[TIM_DMA_ID_CC1]);

    // DMA interrupt init
    // DMA1_Channel6_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}


//
//HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_CC1]);
//HAL_DMA_DeInit(htim_base->hdma[TIM_DMA_ID_TRIGGER]);
//
