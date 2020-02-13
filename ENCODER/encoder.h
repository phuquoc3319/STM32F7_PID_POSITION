/*
 * encoder.h
 *
 *  Created on: Feb 11, 2020
 *      Author: RICHARD
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f7xx.h"

TIM_HandleTypeDef TIM2_Handler;
TIM_HandleTypeDef TIM5_Handler;

void ENCODER2_Init(void);
void ENCODER5_Init(void);

#endif /* ENCODER_H_ */
