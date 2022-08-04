/*
 * MotorOperation.c
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */

#include "MotorOperation.h"

int16_t getMod(int32_t xMod,int16_t mMod)
{
	int16_t temp;
	temp=xMod % mMod;
	if(temp<0)
		return (temp+mMod);
	else
		return  temp;
}

/*** moveMotor() defines fundamental operation of Stepper ****\
 * Parameter theta is step size defined using encoder resolution *
 * Parameter effort is corresponding to the PWM duty settings ****
 * for current setting, it is between (0, 100) since the duty is  *
 * set to 100 AND its frequency is set to 360 kHz ************/

void moveMotor(int32_t theta, uint16_t effort){

	int16_t voltPhaseA, voltPhaseB;
	int16_t sinPhaseA, cosPhaseB;

	int16_t angleSin, angleCos;

	float magicCoef = 12.5f;	// 50/4

	angleSin = getMod(magicCoef * theta, 4096);
	angleCos = angleSin + 1024;	// sin(theta + pi/2) = cos(theta)
	if(angleCos > 4096)	angleCos -= 4096;

	sinPhaseA = sinLookupTable[angleSin];
	cosPhaseB = sinLookupTable[angleCos];

	voltPhaseB = effort * sinPhaseA / 1024;
	voltPhaseA = effort * cosPhaseB / 1024;

	// Set Iref
	if(voltPhaseA >= 0){
		LL_TIM_OC_SetCompareCH1(TIM3, voltPhaseA);
		LL_GPIO_SetOutputPin(INA1_GPIO_Port, INA1_Pin);
		LL_GPIO_ResetOutputPin(INA2_GPIO_Port, INA2_Pin);
	}else{
		LL_TIM_OC_SetCompareCH1(TIM3, -voltPhaseA);
		LL_GPIO_ResetOutputPin(INA1_GPIO_Port, INA1_Pin);
		LL_GPIO_SetOutputPin(INA2_GPIO_Port, INA2_Pin);
	}

	if(voltPhaseB >= 0){
		LL_TIM_OC_SetCompareCH2(TIM3, voltPhaseB);
		LL_GPIO_SetOutputPin(INB1_GPIO_Port, INB1_Pin);
		LL_GPIO_ResetOutputPin(INB2_GPIO_Port, INB2_Pin);
	}else{
		LL_TIM_OC_SetCompareCH2(TIM3, -voltPhaseB);
		LL_GPIO_ResetOutputPin(INB1_GPIO_Port, INB1_Pin);
		LL_GPIO_SetOutputPin(INB2_GPIO_Port, INB2_Pin);
	}

}

void fullStep(void){
	if(dirCmd == 1)	stepNumber += 1;
	if(dirCmd == 0)	stepNumber -= 1;

	moveMotor(81.92f * stepNumber, 40);
	LL_mDelay(10);
}

