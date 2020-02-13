/*
 * timer.h
 *
 *  Created on: Feb 11, 2020
 *      Author: RICHARD
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f7xx.h"

TIM_HandleTypeDef TIM7_Handler;

void TIMER7_Init(uint16_t Time);

#endif /* TIMER_H_ */
