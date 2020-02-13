/*
 * pwm.h
 *
 *  Created on: Feb 11, 2020
 *      Author: RICHARD
 */

#ifndef PWM_H_
#define PWM_H_

#include "stm32f7xx.h"
#include "stdlib.h"

TIM_HandleTypeDef TIM1_Handler;

void PWM1_Init(void);
void PWM_Control(int32_t Duty);


#endif /* PWM_H_ */
