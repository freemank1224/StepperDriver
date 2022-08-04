/*
 * EncoderOperation.h
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */

#ifndef SRC_ENCODEROPERATION_H_
#define SRC_ENCODEROPERATION_H_

#include "main.h"

#include "FlashOperation.h"
#include "MotorOperation.h"

uint16_t readAngle(void);
int32_t encoderFilter(uint8_t filterDepth);
void calibrateEncoder(void);

#endif /* SRC_ENCODEROPERATION_H_ */
