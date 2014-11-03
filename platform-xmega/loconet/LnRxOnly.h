/****************************************************************************
    Copyright (C) 2006 Stefan Bormann

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

 Title :   RX only Loconet Receiver
 Author:   Stefan Bormann <stefan.bormann@gmx.de>
 Date:     17-Mar-2006
 Software: AVR-GCC
 Target:   AtMega8

 DESCRIPTION
       Header of receive only slim LocoNet receiver.
       Some functions are inline, because they are expected to be called
       only once.

*****************************************************************************/
#ifndef _LN_RX_ONLY_H_
#define _LN_RX_ONLY_H_

#include <avr/io.h>
#include "sysdef.h"

static void inline LnRxOnlyInit(void)
{
	// Enable Pull-down on RX line (voltage divider), input sense both edges
	PORTD.PIN6CTRL = PORT_ISC_BOTHEDGES_gc; // | PORT_OPC_PULLDOWN_gc;
	
	// enable UART receiver and transmitter
	LN_HW_UART_CONTROL_REGB = USART_RXEN_bm;
	
	// set baud rate
	//LN_HW_UART_BAUDRATE_REG = ((F_CPU)/(16666l*16l)-1);
	LN_HW_UART_BAUDRATE_REG = (int)((F_CPU)/(16.0/0.00006)-0.5);
}



unsigned char LnRxOnlyOpcode(void);
unsigned char LnRxOnlyData(void);
void LnRxOnlySevereError(void);
void LnRxOnlyChecksum(void);





#endif
