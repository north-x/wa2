// Minimal USB Stack for ATxmega32a4u and related
// http://nonolithlabs.com
// (C) 2011 Kevin Mehall (Nonolith Labs) <km@kevinmehall.net>
//
// Heavily borrows from LUFA
// Copyright 2011  Dean Camera (dean [at] fourwalledcubicle [dot] com)
//
// Licensed under the terms of the GNU GPLv3+

#include <avr/io.h>

#ifndef F_USB
	#define F_USB 48000000
#endif

#define DEFINE_EVENT_ALIASES
#include "usb.h"
#include <util/delay.h>

uint8_t ep0_buf_in[USB_EP0SIZE];
uint8_t ep0_buf_out[USB_EP0SIZE];
USB_EP_pair_t endpoints[USB_MAXEP+1] GCC_FORCE_ALIGN_2;


volatile uint8_t USB_DeviceState;
volatile uint8_t USB_Device_ConfigurationNumber;

void USB_Init(){
	//uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
	//GlobalInterruptDisable();

	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	USB.CAL0 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL0));
	NVM.CMD  = NVM_CMD_READ_CALIB_ROW_gc;
	USB.CAL1 = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBCAL1));
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	
	//SetGlobalInterruptMask(CurrentGlobalInt);

	USB_ResetInterface();	
}

void USB_ResetInterface(){

	//if (USB_Options & USB_DEVICE_OPT_LOWSPEED)
	//  CLK.USBCTRL = ((((F_USB / 6000000) - 1) << CLK_USBPSDIV_gp) | CLK_USBSRC_RC32M_gc | CLK_USBSEN_bm);
	//else
	CLK.USBCTRL = ((((F_USB / 48000000) - 1) << CLK_USBPSDIV_gp) | CLK_USBSRC_PLL_gc | CLK_USBSEN_bm);
	USB.EPPTR = (unsigned) &endpoints;
	USB.ADDR = 0;
	
	endpoints[0].out.STATUS = 0;
	endpoints[0].out.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_EP0SIZE);
	endpoints[0].out.DATAPTR = (unsigned) &ep0_buf_out;
	endpoints[0].in.STATUS = USB_EP_BUSNACK0_bm;
	endpoints[0].in.CTRL = USB_EP_TYPE_CONTROL_gc | USB_EP_size_to_gc(USB_EP0SIZE);
	endpoints[0].in.DATAPTR = (unsigned) &ep0_buf_in;
	
	USB.CTRLA = USB_ENABLE_bm | USB_SPEED_bm | USB_MAXEP;
	
	USB_Attach();
}

void USB_ep0_send_progmem(const uint8_t* addr, uint16_t size){
	uint8_t *buf = ep0_buf_in;
	uint16_t remaining = size;
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
	while (remaining--){
		*buf++ = pgm_read_byte(addr++);
	}
	USB_ep0_send(size);
}

void USB_ConfigureClock(){
	
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

void USB_Enable_SOF_DFLL(void)
{
	// Disable DFLL
	while (DFLLRC32M.CTRL!=0)
	{
		DFLLRC32M.CTRL = 0;
	}
	
	OSC.CTRL = OSC_RC32MEN_bm | OSC_PLLEN_bm;	// Disable 32 kHz clock
	OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc;
	
	DFLLRC32M.COMP1 = 0x00;	// 32 MHz / 1 kHz
	DFLLRC32M.COMP2 = 0x7D;
	
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;
}
