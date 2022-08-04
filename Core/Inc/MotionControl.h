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
uint32_t speedControl(PID_TypeDef *PID, int32_t refSpeed, int32_t currentSpeed);

#endif /* SRC_MOTIONCONTROL_H_ */
