/*
 * timer.c
 *
 *  Created on: Feb 11, 2020
 *      Author: RICHARD
 */
#include "timer.h"

void TIMER7_Init(uint16_t Time)
{
	__HAL_RCC_TIM7_CLK_ENABLE();

	TIM7_Handler.Instance = TIM7;
	TIM7_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM7_Handler.Init.ClockDivision = 0;
	TIM7_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM7_Handler.Init.Prescaler = 108 - 1;
	TIM7_Handler.Init.Period = Time;
	TIM7_Handler.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TIM7_Handler);

	__HAL_TIM_ENABLE_IT(&TIM7_Handler,TIM_IT_UPDATE);
	HAL_NVIC_SetPriority(TIM7_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);

	HAL_TIM_Base_Start_IT(&TIM7_Handler);
}

