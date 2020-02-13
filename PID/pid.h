/*
 * pid.h
 *
 *  Created on: Dec 27, 2019
 *      Author: RICHARD
 */

#ifndef PID_H_
#define PID_H_

#include "stm32f7xx.h"
#include "stdbool.h"


extern uint32_t msTick;

typedef struct
{

#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1


	double dispKp;
	double dispKi;
	double dispKd;

	double kp;
	double ki;
	double kd;

	int controllerDirection;
	int pOn;

	double *myInput;
	double *myOutput;
	double *mySetpoint;

	uint32_t lastTime;
	double outputSum, lastInput;

	uint32_t SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;

}PID_Instance;



void PID_Init(PID_Instance *s, double *Input, double *Output, double *Setpoint,
		double Kp, double Ki, double Kd, int ControllerDirection);

_Bool PID_Compute(PID_Instance *s);
void PID_SetTunings(PID_Instance *s, double Kp, double Ki, double Kd, int POn);
void PID_SetSampleTime(PID_Instance *s, uint32_t NewSampleTime);
void PID_SetOutputLimits(PID_Instance *s, double Min, double Max);
void PID_SetMode(PID_Instance *s, int Mode);
void PID_InitiaLize(PID_Instance *s);
void PID_SetControllerDirection(PID_Instance *s, int Direction);
double PID_Kp(PID_Instance *s);
double PID_Ki(PID_Instance *s);
double PID_Kd(PID_Instance *s);
int PID_GetMode(PID_Instance *s);
int PID_GetDirection(PID_Instance *s);




#endif /* PID_H_ */
