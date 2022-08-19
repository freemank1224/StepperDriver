/*
 * MotionControl.h
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */

#ifndef SRC_MOTIONCONTROL_H_
#define SRC_MOTIONCONTROL_H_

#include "main.h"
#include "PID.h"

uint32_t rpmCalculate(uint32_t fclk, uint32_t prescalerValue, uint32_t deltaN, uint32_t ARRvalue);
int32_t rpm2ticks(int32_t RPM);
int32_t ticks2rpm(int32_t ticksPerSecond);
uint16_t rpm2ARR(uint32_t fclk, uint32_t prescalerValue, uint32_t stepAngle, int32_t RPM);

#endif /* SRC_MOTIONCONTROL_H_ */
