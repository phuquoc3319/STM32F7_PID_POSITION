/*
 * encoder.c
 *
 *  Created on: Feb 11, 2020
 *      Author: RICHARD
 */
#include "encoder.h"

void ENCODER2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_Encoder_InitTypeDef ENCODER_InitStructure;

	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_3;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);

	TIM2_Handler.Instance = TIM2;
	TIM2_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM2_Handler.Init.ClockDivision = 0;
	TIM2_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM2_Handler.Init.Prescaler = 0;
	TIM2_Handler.Init.Period = 0xffffffff;
	TIM2_Handler.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TIM2_Handler);

	ENCODER_InitStructure.EncoderMode = TIM_ENCODERMODE_TI12;
	ENCODER_InitStructure.IC1Filter = 0;
	ENCODER_InitStructure.IC1Polarity = TIM_ICPOLARITY_RISING;
	ENCODER_InitStructure.IC1Prescaler = 0;
	ENCODER_InitStructure.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	ENCODER_InitStructure.IC2Filter = 0;
	ENCODER_InitStructure.IC2Polarity = TIM_ICPOLARITY_RISING;
	ENCODER_InitStructure.IC2Prescaler = 0;
	ENCODER_InitStructure.IC2Selection = TIM_ICSELECTION_DIRECTTI;

	HAL_TIM_Encoder_Init(&TIM2_Handler,&ENCODER_InitStructure);

	__HAL_TIM_SET_COUNTER(&TIM2_Handler,200);

	HAL_TIM_Encoder_Start(&TIM2_Handler,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&TIM2_Handler,TIM_CHANNEL_2);
}

void ENCODER5_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_Encoder_InitTypeDef ENCODER_InitStructure;

	__HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM5;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);

	TIM5_Handler.Instance = TIM5;
	TIM5_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM5_Handler.Init.ClockDivision = 0;
	TIM5_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM5_Handler.Init.Prescaler = 0;
	TIM5_Handler.Init.Period = 0xffffffff;
	TIM5_Handler.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TIM5_Handler);

	ENCODER_InitStructure.EncoderMode = TIM_ENCODERMODE_TI12;
	ENCODER_InitStructure.IC1Filter = 0;
	ENCODER_InitStructure.IC1Polarity = TIM_ICPOLARITY_RISING;
	ENCODER_InitStructure.IC1Prescaler = 0;
	ENCODER_InitStructure.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	ENCODER_InitStructure.IC2Filter = 0;
	ENCODER_InitStructure.IC2Polarity = TIM_ICPOLARITY_RISING;
	ENCODER_InitStructure.IC2Prescaler = 0;
	ENCODER_InitStructure.IC2Selection = TIM_ICSELECTION_DIRECTTI;

	HAL_TIM_Encoder_Init(&TIM5_Handler,&ENCODER_InitStructure);

	__HAL_TIM_SET_COUNTER(&TIM5_Handler,200);

	HAL_TIM_Encoder_Start(&TIM5_Handler,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&TIM5_Handler,TIM_CHANNEL_2);
}

