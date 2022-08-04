/*
 * UsDelay.c
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */


#include "UsDelay.h"

void uDelay(uint32_t us){
	uint32_t temp;
	SysTick -> LOAD = 72 * us;	// Systic freq is equal to HCLK, ie, 72MHz
	SysTick -> VAL = 0x00;
	SysTick -> CTRL = 0x05;		// SysTick -> CTRL bit0: Enable, bit2: 1-HCLK | 0-HCLK/8
	do{
		temp = SysTick -> CTRL;
	}while((temp & 0x01) && (!(temp & (1 << 16))));

//	while(!(SysTick -> CTRL & 0x00010000));

	SysTick -> CTRL = 0x00;
	SysTick -> VAL = 0x00;
}
