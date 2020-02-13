/*
 * pid.c
 *
 *  Created on: Dec 27, 2019
 *      Author: RICHARD
 */
#include "pid.h"

void PID_Init(PID_Instance *s, double *Input, double *Output, double *Setpoint,
		double Kp, double Ki, double Kd, int ControllerDirection)
{
	s->myOutput = Output;
	s->myInput = Input;
	s->mySetpoint = Setpoint;
	s->inAuto = false;

	// Set output limit
	s->outMin = 0;
	s->outMax = 255;

	s->SampleTime = 100; // Default sample time is 0.1 second

	// Set control direction
	s->controllerDirection = ControllerDirection;

	// Set tuning
	s->kp = Kp;
	s->ki = Ki;
	s->kd = Kd;


	s->lastTime = msTick - s->SampleTime;
}

_Bool PID_Compute(PID_Instance *s)
{
	if(!(s->inAuto))
	{
		return false;
	}
	uint32_t now = msTick;
	uint32_t timeChange = (now - s->lastTime);
	if(timeChange >= s->SampleTime)
	{
		double input = *(s->myInput);
		double error = *(s->mySetpoint) - input;
		double dInput = (input - s->lastInput);
		s->outputSum += (s->ki * error);

		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if(!(s->pOnE))
		{
			s->outputSum -= s->kp * dInput;
		}

		if(s->outputSum > s->outMax)
		{
			s->outputSum = s->outMax;
		}
		else if (s->outputSum < s->outMin)
		{
			s->outputSum = s->outMin;
		}

		/*Add Proportional on Error, if P_ON_E is specified*/
		double output;
		if(s->pOnE)
		{
			output = s->kp * error;
		}
		else
		{
			output = 0;
		}

		/*Compute Rest of PID Output*/
		output += s->outputSum - s->kd * dInput;

		if(output > s->outMax)
		{
			output = s->outMax;
		}
		else if (output < s->outMin)
		{
			output = s->outMin;
		}

		*(s->myOutput) = output;

		/*Remember some variables for next time*/
		s->lastInput = input;
		s->lastTime = now;

		return true;
	}
	else return false;
}

void PID_SetTunings(PID_Instance *s, double Kp, double Ki, double Kd, int POn)
{
	if(Kp < 0 || Ki < 0 || Kd < 0)
	{
		return;
	}
	s->pOn = POn;
	s->pOnE = POn == P_ON_E;

	s->dispKp = Kp;
	s->dispKi = Ki;
	s->dispKd = Kd;

	double SampleTimeInSec = ((double)(s->SampleTime)) / 1000;
	s->kp = Kp;
	s->ki = Ki * SampleTimeInSec;
	s->kd = Kd / SampleTimeInSec;

	if((s->controllerDirection) == REVERSE)
	{
		s->kp = (0 - s->kp);
		s->ki = (0 - s->ki);
		s->kd = (0 - s->kd);
	}
}

void PID_SetSampleTime(PID_Instance *s, uint32_t NewSampleTime)
{
	if(NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime / (double)(s->SampleTime);
		s->ki *= ratio;
		s->kd /= ratio;
		s->SampleTime = (uint32_t)NewSampleTime;
	}
}

void PID_SetOutputLimits(PID_Instance *s, double Min, double Max)
{
	if(Min >= Max)
	{
		return;
	}

	s->outMin = Min;
	s->outMax = Max;

	if(s->inAuto)
	{
		if(*(s->myOutput) > s->outMax)
		{
			*(s->myOutput) = s->outMax;
		}
		else if (*(s->myOutput) < s->outMin)
		{
			*(s->myOutput) = s->outMin;
		}

		if(s->outputSum > s->outMax)
		{
			s->outputSum = s->outMax;
		}
		else if (s->outputSum < s->outMin)
		{
			s->outputSum = s->outMin;
		}
	}
}

void PID_SetMode(PID_Instance *s, int Mode)
{
	_Bool newAuto = (Mode == AUTOMATIC);
	if (newAuto && !(s->inAuto))
	{
		/*we just went from manual to auto*/
		// Calling initialize function
		PID_InitiaLize(s);
	}
	s->inAuto = newAuto;
}

void PID_InitiaLize(PID_Instance *s)
{
	s->outputSum = *(s->myOutput);
	s->lastInput = *(s->myInput);
	if(s->outputSum > s->outMax)
	{
		s->outputSum = s->outMax;
	}
	else if (s->outputSum < s->outMin)
	{
		s->outputSum = s->outMin;
	}
}

void PID_SetControllerDirection(PID_Instance *s, int Direction)
{
	if((s->inAuto) && Direction != (s->controllerDirection))
	{
		s->kp = (0 - s->kp);
		s->ki = (0 - s->ki);
		s->kd = (0 - s->kd);
	}

	s->controllerDirection = Direction;
}

double PID_Kp(PID_Instance *s)
{
	return s->dispKp;
}

double PID_Ki(PID_Instance *s)
{
	return s->dispKi;
}

double PID_Kd(PID_Instance *s)
{
	return s->dispKd;
}

int PID_GetMode(PID_Instance *s)
{
	return (s->inAuto) ? AUTOMATIC : MANUAL;
}

int PID_GetDirection(PID_Instance *s)
{
	return s->controllerDirection;
}
