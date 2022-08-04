/*
 * EncoderOperation.c
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */
#include "EncoderOperation.h"



uint16_t readAngle()
{
  uint16_t data1,data2;
  // NSS LOW
  LL_GPIO_ResetOutputPin(NSS_GPIO_Port, NSS_Pin);
  while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
  LL_SPI_TransmitData16(SPI1,0x8355);
  while(LL_SPI_IsActiveFlag_RXNE(SPI1)==0);
  data1=LL_SPI_ReceiveData16(SPI1)&0x00FF;
  LL_GPIO_SetOutputPin(NSS_GPIO_Port, NSS_Pin);
  LL_GPIO_ResetOutputPin(NSS_GPIO_Port, NSS_Pin);
  while(LL_SPI_IsActiveFlag_TXE(SPI1)==0);
  LL_SPI_TransmitData16(SPI1,0x84A0);
  while(LL_SPI_IsActiveFlag_RXNE(SPI1)==0);
  data2=LL_SPI_ReceiveData16(SPI1)&0x00FF;
  LL_GPIO_SetOutputPin(NSS_GPIO_Port, NSS_Pin);

//  if((data2 & 0b00000010) == 0x02) LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);

  data1=(data1<<6)+(data2>>2);

  return data1;
}


int32_t encoderFilter(uint8_t filterDepth)
{
	int32_t encoderOut = 0;

	if(filterDepth < 1) filterDepth = 1;

	for(uint8_t i=0; i<filterDepth; i++)
	{
		encoderOut += readAngle();
	}

	encoderOut = encoderOut / filterDepth;

	return encoderOut;
}


void calibrateEncoder(void){

	int32_t encoderReadingFiltered = 0;
	int32_t currentEncoderReading = 0;
	int32_t lastEncoderReading = 0;

	int32_t iStart = 0;
	int32_t jStart = 0;
	int32_t stepIdx = 0;

	int32_t fullStepReadingArray[200];
	int32_t ticks = 0;
	uint32_t flashAddressBegin = 0x08008000;	// Flash write initial address

	uint16_t lookupAngle;

	// Temporarily Testing Vars

	// ************* Main Algorithm ***************//

	dirCmd = 1;
	moveMotor(0, 80);

	// Indicate Calibration process begin
	for(uint8_t m = 0; m < 4; m++){
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_mDelay(250);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_mDelay(250);
	}

	// RUN 360 degree and log encoder value at each step
	for(int16_t i=0; i<200; i++){

		encoderReadingFiltered = 0;
		LL_mDelay(20);

		lastEncoderReading = readAngle();
		// Averaging filter : 10 samples
		for(uint8_t reading=0; reading<10; reading++){
			currentEncoderReading = readAngle();
			if(currentEncoderReading - lastEncoderReading < -8192){
				currentEncoderReading += 16384;
			}
			if(currentEncoderReading - lastEncoderReading > 8192){
				currentEncoderReading -= 16384;
			}
			encoderReadingFiltered += currentEncoderReading;
			LL_mDelay(10);
			lastEncoderReading = currentEncoderReading;
		}

		encoderReadingFiltered = encoderReadingFiltered / 10;

		// Saturation
		if(encoderReadingFiltered > 16384) encoderReadingFiltered -= 16384;
		if(encoderReadingFiltered < 0) encoderReadingFiltered += 16384;

		fullStepReadingArray[i] = encoderReadingFiltered;

		// move a SINGLE full step
		fullStep();
		LL_mDelay(50);
	}

	// Switch rotate direction to clockwise and move a full step
	// to align with corresponding positions
	dirCmd = 0;
	fullStep();
	LL_mDelay(1000);

	for(int16_t x=199; x>=0; x--)
	{
		encoderReadingFiltered = 0;
		LL_mDelay(20);
		lastEncoderReading = readAngle();
		for(uint8_t reading = 0; reading < 10; reading++)
		{
			currentEncoderReading = readAngle();
			if(currentEncoderReading - lastEncoderReading < -8192){
				currentEncoderReading += 16384;
			}
			if(currentEncoderReading - lastEncoderReading > 8192){
				currentEncoderReading -= 16384;
			}
			encoderReadingFiltered += currentEncoderReading;
			LL_mDelay(10);
			lastEncoderReading = currentEncoderReading;
		}
		encoderReadingFiltered = encoderReadingFiltered / 10;
		if(encoderReadingFiltered > 16384)
			encoderReadingFiltered -= 16384;
		if(encoderReadingFiltered< 0)
			encoderReadingFiltered += 16384;
		fullStepReadingArray[x] = (fullStepReadingArray[x] + encoderReadingFiltered) / 2;
		fullStep();
		LL_mDelay(50);
	}

	// Turn off motor
	LL_TIM_OC_SetCompareCH1(TIM3, 0);
	LL_TIM_OC_SetCompareCH2(TIM3, 0);

	for(uint8_t i=0; i<200; i++)
	{
		ticks = fullStepReadingArray[(i+1) % 200] - fullStepReadingArray[i % 200];
		if(ticks < -15000)
			ticks += 16384;
		if(ticks > 15000)
			ticks -= 16384;
		for(int32_t j=0; j<ticks; j++)
		{
			stepIdx = (fullStepReadingArray[i]+j) % 16384;
			if(stepIdx == 0)
			{
				iStart = i;
				jStart = j;
			}
		}
	}

	flashUnlock();
	flashErase32K();

	for(int32_t i=iStart; i < (iStart + 200 + 1); i++)
	{
		ticks = fullStepReadingArray[(i+1) % 200] - fullStepReadingArray[i % 200];
		if(ticks < -15000)
			ticks += 16384;
		if(i == iStart)
		{
			for(uint32_t j = jStart; j < ticks; j++)
			{
				lookupAngle = (8192 * i + 8192 * j / ticks) % 16384 / 100;
				flashWriteHalfWord(flashAddressBegin, (uint16_t)lookupAngle);
				flashAddressBegin += 2;
			}
		}
		else if(i == iStart + 200)
		{
			for(uint32_t j = 0; j < jStart; j++)
			{
				lookupAngle = (8192 * i + 8192 * j / ticks) % 16384 / 100;
				flashWriteHalfWord(flashAddressBegin, (uint16_t)lookupAngle);
				flashAddressBegin += 2;
			}
		}
		else
		{
			for(uint32_t j = 0; j < ticks; j++)
			{
				lookupAngle = (8192 * i + 8192 * j / ticks) % 16384 / 100;
				flashWriteHalfWord(flashAddressBegin, (uint16_t)lookupAngle);
				// Verify Flash Writing Operation
//				ReadHalfWord = flashReadHalfWord(flashAddressBegin);
//				if(ReadHalfWord == lookupAngle) LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//				LL_mDelay(100);
				// Verify Complete
				flashAddressBegin += 2;

			}
		}
	}

	flashLock();

	// blinking 4 cycles

	for(uint8_t m = 0; m < 4; m++){
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_mDelay(250);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_mDelay(250);
	}
}


