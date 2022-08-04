/*
 * FlashOperation.h
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */

#ifndef SRC_FLASHOPERATION_H_
#define SRC_FLASHOPERATION_H_

#include "main.h"
#include "Variables.h"

void flashUnlock(void);
void flashLock(void);
uint8_t flashGetStatus(void);
uint8_t flashWaitDone(uint16_t time);
uint8_t flashErasePage(uint32_t pageAddr);
void flashErase32K(void);
uint8_t flashWriteHalfWord(uint32_t faddr, uint16_t data);
uint16_t flashReadHalfWord(uint32_t faddr);


#endif /* SRC_FLASHOPERATION_H_ */
