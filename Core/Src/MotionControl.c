/*
 * MotionControl.c
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */


#include "MotionControl.h"

/* Convert SPEED command from RPM unit to encoder ticks *\
 * stepAngle unit is degree , ticks is "number of ticks per second"
 ********************************************************/
int32_t rpm2ticks(int32_t RPM)
{
	int32_t ticksOut;
	ticksOut = 16384 * RPM * 60;

	return ticksOut;
}

int32_t ticks2rpm(int32_t ticksPerSecond)
{
	int32_t rpmOut;
	rpmOut = ticksPerSecond * 60 / 16384;

	return rpmOut;
}

/* Convert RPM to ARR value
 * *** RPM have to be POSITIVE
 **************************/
uint16_t rpm2ARR(uint32_t fclk, uint32_t prescalerValue, uint32_t stepAngle, int32_t RPM)
{
	uint16_t OutARR;
	if(RPM < 0) RPM = -RPM;
	OutARR = (uint16_t)(fclk / 6 / prescalerValue / RPM) * stepAngle;

	return OutARR;
}
