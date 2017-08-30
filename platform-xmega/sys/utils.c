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
	uint8_t address, temp8;
	uint16_t id = 0;
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	CRC.CTRL = CRC_RESET0_bm | CRC_SOURCE_IO_gc;
	
	while (NVM.STATUS&NVM_NVMBUSY_bm);
	
	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	
	for (address=offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0);address<=offsetof(NVM_PROD_SIGNATURES_t, COORDY1);address++)
	{
		temp8 = pgm_read_byte(address);
		CRC.DATAIN = temp8;
		id += temp8;
		__asm__ __volatile__("");
	}
	
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	id = (CRC.CHECKSUM1<<8) + CRC.CHECKSUM0;
	return id;
}

void usb_configure_clock() {
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	
	OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc | OSC_RC2MCREF_RC32K_gc;
	
	DFLLRC32M.COMP1 = 0x12; // 32 MHz / 1.024 kHz should be the default
	DFLLRC32M.COMP2 = 0x7A;
	
	while((OSC.STATUS & (OSC_RC32MRDY_bm | OSC_RC32KRDY_bm | OSC_RC2MRDY_bm))!=(OSC_RC32MRDY_bm | OSC_RC32KRDY_bm | OSC_RC2MRDY_bm)); // wait for oscillator ready

	DFLLRC2M.CTRL = 0;
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;
	
	OSC.PLLCTRL = OSC_PLLSRC_RC32M_gc | 6; // 32MHz/4 * 6 = 48MHz
	
	OSC.CTRL = OSC_RC32MEN_bm | OSC_PLLEN_bm | OSC_RC2MEN_bm; // Enable PLL
	
	while(!(OSC.STATUS & OSC_PLLRDY_bm)); // wait for PLL ready

	CCP = CCP_IOREG_gc; //Security Signature to modify clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // Select 32 MHz
	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = 0x00; // No peripheral clock prescaler
	
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm | OSC_PLLEN_bm; // Disable 2 MHz clock
}
