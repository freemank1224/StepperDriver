/*
 * MotionControl.c
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */


#include "MotionControl.h"

uint32_t rpmCalculate(uint32_t fclk, uint32_t prescalerValue, uint32_t deltaN, uint32_t ARRvalue)
{
	uint32_t rpmValue;
	rpmValue = (uint32_t)((273.67f * fclk / prescalerValue) * (deltaN / ARRvalue));
	return rpmValue;
}

// Speed regulation using RPM as unit for refSpeed and currentSpeed
uint32_t speedControl(PID_TypeDef *PID, int32_t refSpeed, int32_t currentSpeed)
{



}
