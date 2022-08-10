/*
 * Variables.h
 *
 *  Created on: Jul 30, 2022
 *      Author: freeman
 */

#ifndef SRC_VARIABLES_H_
#define SRC_VARIABLES_H_

#include "main.h"
#include "PID.h"

// To define a fixed length array, we have to do this !!
#define nBytes 19

extern int16_t dutyLED;
extern int16_t dutyCmd;
extern int16_t stepNumber;
extern int16_t roundNumber;

extern int16_t dirCmd;	// 1:Clockwise 0:Counterclockwise
extern uint16_t testAngle;
extern int32_t filteredAngle;
extern int32_t fullAngle;
extern int32_t fullAngle_k_1;
extern int32_t fullAngle_k_2;
extern int32_t angularSpeed;
extern int32_t angularSpeed_k_1;
extern int32_t angularAcceleration;
extern int32_t commandSpeed;
extern int32_t targetSpeed;
extern int32_t deltaSpeed;
extern int32_t commandAngle;
extern int32_t commandAngle_k_1;
extern int32_t filteredAngle_k_1;
extern int32_t commandOut;
extern int32_t commandOut_k_1;
extern int32_t errorAngleOut;
extern int32_t errorSpeedOut;
extern uint16_t flashAngle;
extern uint16_t amplitude;
extern uint16_t counterTIM1_SLOW;
extern uint16_t counterTIM1_5k;
extern uint16_t counterTIM1_1k;
extern uint16_t counterTIM2_10k;
extern uint16_t counterTIM2_1k;
extern uint16_t counterTIM2_5k;
extern uint16_t counterTIM2_speed;
extern uint16_t testARR;
extern int32_t freeCounter;

// Motor Speed Control Vars
extern int16_t CAL_rotationSpeed;
extern int16_t rotationSpeed_k;
extern int16_t rotationSpeed_k_1;
extern int32_t rotationSpeedEncoder;
extern int32_t rotationSpeedRPM;
extern uint32_t speedCmd;


// PID struct & parameters
extern PID_TypeDef pidPosition;
extern PID_TypeDef pidEffort;
extern PID_TypeDef pidSpeed;

// REAL-TIME Parameters Tuning
extern uint8_t parameterTuningEnable;
extern uint8_t parameterTuningEnable;
extern int32_t recKp;
extern int32_t recKi;
extern int32_t recKd;
extern int32_t recFilterCoef;
extern int32_t recCommandAngle;
extern int32_t recCommandSpeed;
extern uint8_t recFilterEnableFlag;
extern uint8_t recStepSizeIdx;
extern uint32_t recSpeedSetpoint;

// USART Communication
extern uint8_t inData[];
extern uint8_t idx;

extern int32_t controlEffort;
extern uint8_t CAL_closedLoop;
extern uint8_t CAL_releaseMotor;
extern uint8_t CAL_dirCmd;
extern uint8_t CAL_calibrateEncoder;
extern float CAL_stepSize;

// For speed regulation
extern uint32_t setpointARR;
extern int32_t deltaARR;
extern int32_t stepARR;
extern uint32_t outputARR;
extern uint32_t readARR;
extern int32_t deltaARRmax;
extern int32_t deltaARRmin;
extern int32_t deltaPositiveMax;
extern int32_t deltaNegativeMax;
extern uint16_t lowpassFilterCoef;
extern uint32_t ARRmin;
extern uint32_t ARRmax;
extern int32_t speedMax;
extern int32_t speedMin;
extern int32_t effort2freqCoef;
extern int32_t accelerationLimit;

/*********** CONSTANT EXTERN *************/

extern const int16_t ppsConst;
extern const uint32_t frequencyMIN;	// unit: ARR value 4000 for 1kHz
extern const uint32_t frequencyMAX;
extern const uint32_t speedSetpoint;	// RPM
extern const uint32_t speedMAX;			// RPM, NO need to set MIN speed since it can be 0 when stopped!
extern const int16_t sinLookupTable[];


#endif /* SRC_VARIABLES_H_ */
