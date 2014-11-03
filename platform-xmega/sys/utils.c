/*
 * utils.c
 *
 * Created: 31.05.2014 16:35:03
 *  Author: manu
 */ 

#include <stddef.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "sys/utils.h"

uint16_t getID16(void)
{
	uint8_t address;
	uint16_t id = 0;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (address=offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0);address<=offsetof(NVM_PROD_SIGNATURES_t, COORDY1);address++)
	{
		id += pgm_read_byte(address);
		__asm__ __volatile__("");
	}
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	return id;
}

