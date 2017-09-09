/*
 * Copyright (c) 2014, Manuel Vetterli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */ 

#include <avr/pgmspace.h>
#include "sys/process.h"
#include "usb_support.h"
#include "loconet.h"
#include "ln_interface.h"
#include "uart.h"

#define UART_BAUD_RATE 57142
PROCESS(usb_process, "USB Handler");

LnBuf ln_buf;

void usb_process_init(void)
{	
	PORTD.PIN6CTRL = PORT_OPC_PULLUP_gc;
	PORTD.OUTSET = (1<<7);
	PORTD.DIRSET = (1<<7);
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	initLnBuf(&ln_buf);
	process_start(&usb_process, NULL);
}

void sendLocoNetPacketUSB(lnMsg *LnPacket)
{
	uint8_t index, size;
	size = getLnMsgSize(LnPacket);
		
	for (index=0;index<size;index++)
	{
		uart_putc(LnPacket->data[index]);
	}
}

PROCESS_THREAD(usb_process, ev, data)
{
	lnMsg *LnPacket;
	uint16_t rec;
	static uint8_t masterEnabled = 0;
	
	PROCESS_BEGIN();
	
	while (1)
	{
		LnPacket = recvLnMsg(&ln_buf);
		
		if (LnPacket)
		{
			if (!masterEnabled)
			{
				enableLocoNetMaster(1);
				masterEnabled = 1;
			}
			sendLocoNetPacket(LnPacket);
		}
		
		while (1)
		{
			rec = uart_getc();
			if (rec & UART_NO_DATA)
			{
				break;
			}
			addByteLnBuf(&ln_buf, (uint8_t) rec);
		}
		
		PROCESS_PAUSE();
	}
	
	PROCESS_END();
}
