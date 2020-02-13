/*
 * pwm.c
 *
 *  Created on: Feb 11, 2020
 *      Author: RICHARD
 */
#include "pwm.h"

void PWM1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OC_InitTypeDef TIM_OCInitStructure;

	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOE,&GPIO_InitStructure);

	TIM1_Handler.Instance = TIM1;
	TIM1_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM1_Handler.Init.ClockDivision = 0;
	TIM1_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
	TIM1_Handler.Init.Prescaler = 10 - 1;
	TIM1_Handler.Init.Period = 1000;
	TIM1_Handler.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&TIM1_Handler);

	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_OCInitStructure.Pulse = 0;

	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM_OCInitStructure,TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM_OCInitStructure,TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM_OCInitStructure,TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM_OCInitStructure,TIM_CHANNEL_4);

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;

	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_4);
}

void PWM_Control(int32_t Duty)
{
	if(Duty > 0)
	{
		TIM1->CCR2 = 0;
		TIM1->CCR3 = Duty;
	}
	else if (Duty < 0)
	{
		TIM1->CCR2 = abs(Duty);
		TIM1->CCR3 = 0;
	}
}
