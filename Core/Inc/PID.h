/*
 * PID.h
 *
 *  Created on: Jul 5, 2022
 *      Author: freeman
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include "main.h"

typedef struct{
	int32_t setPoint;
	int32_t error_k, error_k_1, error_k_2;
	int32_t kp, ki, kd;	// Constant between 0 ~ MAX(int32_t)
	uint8_t filterEnableFlag;
	int32_t filterCoef;	// Constant between 0 ~ 100
	int32_t outputMAX, outputMIN;	// Constant between 0 ~ 100
}PID_TypeDef;

void initPID(PID_TypeDef *PID, int32_t Kp, int32_t Ki, int32_t Kd, uint8_t FilterEnable, int32_t FilterCoef, int32_t OutMin, int32_t OutMax);
int32_t controlPID(PID_TypeDef *PID, int32_t setValue, int32_t currentValue);

#endif /* SRC_PID_H_ */
