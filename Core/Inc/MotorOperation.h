/*
 * MotorOperation.h
 *
 *  Created on: Jul 30, 2022
 *      Author: freeman
 *
 *      Include 4 functions: getMode(), moveMotor(), fullStep()
 */

#ifndef SRC_MOTOROPERATION_H_
#define SRC_MOTOROPERATION_H_

#include "main.h"
#include "Variables.h"
#include "EncoderOperation.h"


int16_t getMod(int32_t xMod,int16_t mMod);
void moveMotor(int32_t theta, uint16_t effort);
void fullStep(void);

#endif /* SRC_MOTOROPERATION_H_ */
