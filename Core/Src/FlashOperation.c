/*
 * FlashOperation.c
 *
 *  Created on: Jul 31, 2022
 *      Author: freeman
 */

#include "FlashOperation.h"

void flashUnlock(void)
{
	FLASH -> KEYR = FLASH_KEY1;
	FLASH -> KEYR = FLASH_KEY2;
}

void flashLock(void)
{
	FLASH -> CR |= 1 << 7;
}

uint8_t flashGetStatus(void)
{
	uint32_t res;
	res = FLASH -> SR;
	if(res & (1 << 0))
		return 1;
	else if(res & (1 << 2))
		return 2;
	else if(res & (1 << 4))
		return 3;
	else
		return 0;
}

uint8_t flashWaitDone(uint16_t time)
{
	uint8_t res;
	do
	{
		res = flashGetStatus();
		if(res != 1)
			break;
		LL_mDelay(1);
		time --;
	}while(time);

	if(time == 0)
		res = 0xff;
	return res;
}

uint8_t flashErasePage(uint32_t pageAddr)
{
	uint8_t res = 0;
	res = flashWaitDone(0x5fff);
	if(res == 0)
	{
		FLASH -> CR |= 1<<1;
		FLASH -> AR = pageAddr;
		FLASH -> CR |= 1<<6;
		res = flashWaitDone(0x5fff);
		if(res != 1)
			FLASH -> CR &= ~(1 << 1);
	}
	return res;
}

void flashErase32K(void)
{
	  flashErasePage(0x08008000);
	  flashErasePage(0x08008400);
	  flashErasePage(0x08008800);
	  flashErasePage(0x08008C00);
	  flashErasePage(0x08009000);
	  flashErasePage(0x08009400);
	  flashErasePage(0x08009800);
	  flashErasePage(0x08009C00);
	  flashErasePage(0x0800A000);
	  flashErasePage(0x0800A400);
	  flashErasePage(0x0800A800);
	  flashErasePage(0x0800AC00);
	  flashErasePage(0x0800B000);
	  flashErasePage(0x0800B400);
	  flashErasePage(0x0800B800);
	  flashErasePage(0x0800BC00);
	  flashErasePage(0x0800C000);
	  flashErasePage(0x0800C400);
	  flashErasePage(0x0800C800);
	  flashErasePage(0x0800CC00);
	  flashErasePage(0x0800D000);
	  flashErasePage(0x0800D400);
	  flashErasePage(0x0800D800);
	  flashErasePage(0x0800DC00);
	  flashErasePage(0x0800E000);
	  flashErasePage(0x0800E400);
	  flashErasePage(0x0800E800);
	  flashErasePage(0x0800EC00);
	  flashErasePage(0x0800F000);
	  flashErasePage(0x0800F400);
	  flashErasePage(0x0800F800);
	  flashErasePage(0x0800FC00);
}

uint8_t flashWriteHalfWord(uint32_t faddr, uint16_t data)
{
	uint8_t res;
	res = flashWaitDone(0xff);
	if(res == 0)
	{
		FLASH -> CR |= 1<<0;
		*(volatile uint16_t*)faddr = data;
		res = flashWaitDone(0xff);
		if(res != 1)
		{
			FLASH -> CR &= ~(1<<0);
		}
	}
	return res;
}

uint16_t flashReadHalfWord(uint32_t faddr)
{
	return *(volatile uint16_t*)faddr;
}

