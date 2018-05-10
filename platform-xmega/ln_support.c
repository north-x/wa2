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

#include <avr/io.h>
#include <avr/wdt.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "eeprom.h"
#include "loconet/utils.h"
#include "ln_interface.h"
#include "sv.h"
#include "IdStorage.h"
#include "ln_support.h"
#include "usb_support.h"
#include "config.h"
#include "sys/utils.h"

PROCESS(ln_process, "Loconet Handler");
PROCESS(ln_ack_process, "Loconet Ack Handler");
PROCESS(ln_wdt_process, "Loconet Watchdog Handler");

static struct etimer ln_ack_timer;
static struct etimer ln_wdt_timer;

static process_event_t ln_ack_event;

LnBuf LnBuffer;

uint8_t ln_gpio_tx[LN_GPIO_BW];
uint8_t ln_gpio_tx_ack[LN_GPIO_BW];
uint8_t ln_gpio_status[LN_GPIO_BW];
uint8_t ln_gpio_status_pre[LN_GPIO_BW];
uint8_t ln_gpio_status_ack[LN_GPIO_BW];
uint8_t ln_gpio_status_flag[LN_GPIO_BW];
uint8_t ln_gpio_ack_counter[LN_GPIO_CH_COUNT];
uint8_t ln_wdt_flag;
uint8_t ln_wdt_counter __attribute__ ((section (".noinit")));

extern uint16_t deviceID;

void loconet_init(void)
{
	initLnBuf(&LnBuffer);
	initLocoNet(&LnBuffer);
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	
	if (eeprom.sv_serial_number==0xFFFF)
	{
		eeprom.sv_serial_number = deviceID;
		ln_load_board_config();
		eeprom_sync_storage();
		
		// Issue a software reset
		/*
		CCP = CCP_IOREG_gc;
		RST.CTRL = RST_SWRST_bm;
		*/
	}
	
	if (readSVDestinationId()==0xFFFF)
	{
		writeSVDestinationId(deviceID);
	}
	
	ln_ack_event = process_alloc_event();
	process_start(&ln_ack_process, NULL);
	
	// Start Watchdog process if requested
	if (eeprom.ln_wdt_enable)
	{
		// Enable watchdog if the fuses are not configured
		if (!(WDT.CTRL&WDT_ENABLE_bm))
		{
			wdt_enable(WDT_PER_8KCLK_gc);
		}
		process_start(&ln_wdt_process, NULL);
	}
	else
	{
		ln_wdt_counter = 0;
	}
}

void ln_update_threshold(void)
{
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
}


PROCESS_THREAD(ln_wdt_process, ev, data)
{
	static lnMsg wdt_msg;
	
	PROCESS_BEGIN();
	
	if (RST.STATUS&RST_PORF_bm)
	{
		ln_wdt_counter = 0;
		RST.STATUS = RST_PORF_bm;
	}
	
	if (RST.STATUS&RST_WDRF_bm)
	{
		if (ln_wdt_counter<0xFF)
		{
			ln_wdt_counter++;
		}
		RST.STATUS = RST_WDRF_bm;
	}
	
	wdt_msg.data[0] = 0x80;
	etimer_set(&ln_wdt_timer, deviceID/256 + 2.5*CLOCK_SECOND);
	
	while (1)
	{
		PROCESS_YIELD();

		etimer_reset(&ln_wdt_timer);

		if (!ln_wdt_flag)
		{
			sendLocoNetPacket(&wdt_msg);
		}
		
		ln_wdt_flag = 0;
	}
	
	PROCESS_END();
}

PROCESS_THREAD(ln_ack_process, ev, data)
{
	static uint8_t current_channel = 0;
	
	PROCESS_BEGIN();
	
	etimer_set(&ln_ack_timer, 250E-3*CLOCK_SECOND);
	
	while (1)
	{
		PROCESS_WAIT_EVENT();
		if (ev==PROCESS_EVENT_TIMER)
			etimer_reset(&ln_ack_timer);
		else if (ev==ln_ack_event)
			current_channel = (intptr_t) data;
		
		if (current_channel>=LN_GPIO_CH_COUNT)
			current_channel = 0;

		if (ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))
		{
			if (ln_gpio_ack_counter[current_channel]>0)
			{
				ln_gpio_tx[current_channel/8] |= (1<<(current_channel%8));
				
				if (ln_gpio_ack_counter[current_channel]<255)
				{
					ln_gpio_ack_counter[current_channel]--;
				}
			}
			else
			{
				ln_gpio_status_ack[current_channel/8] &= ~(1<<(current_channel%8));
			}
		}
		
		current_channel++;
	}
	
	PROCESS_END();
}

PROCESS_THREAD(ln_process, ev, data)
{
	lnMsg *LnPacket;
	uint8_t index;
	
	PROCESS_BEGIN();
	
	// Initialization
	loconet_init();
	for (index=0;index<LN_GPIO_BW;index++)
	{
		ln_gpio_status[index] = eeprom_status.ln_gpio_status[index];
		ln_gpio_status_pre[index] = eeprom_status.ln_gpio_status[index];
	}
	
	while (1)
	{
		PROCESS_PAUSE();
		
		for (index=0;index<LN_GPIO_BW;index++)
		{
			eeprom_status.ln_gpio_status[index] = ln_gpio_status[index];
		}
		
		doSVDeferredProcessing();
		
		LnPacket = recvLocoNetPacket();
	
		if (LnPacket)
		{   
			wdt_reset();
			ln_wdt_flag = 1;
			
			sendLocoNetPacketUSB(LnPacket);
			
			ln_gpio_process_rx(LnPacket);
			
			//ln_throttle_process(LnPacket);
			
			#define LN_RX_CALLBACK(fun) fun(LnPacket);
			#include "config.h"
			#undef LN_RX_CALLBACK
			
			if ((LnPacket->sz.command == OPC_GPON))
			{
				// Force transmission of current state
				for (index=0;index<LN_GPIO_BW;index++)
				{
					ln_gpio_tx[index] = 0xFF;
				}
			}
			/*
			else if (BootloaderParseMessage(LnPacket)==1)
			{
				//BootloaderEnter(); // enter bootloader from running application
			}*/
			else
				processSVMessage(LnPacket);
		}
		else
		{
			ln_gpio_process_tx();
		}
		
	}
	
	PROCESS_END();
}

void ln_gpio_process_tx(void)
{
	uint8_t *msg;
	static uint8_t current_channel = 0;
	
	if (current_channel>=LN_GPIO_CH_COUNT)
		current_channel = 0;
		
	if (((ln_gpio_status[current_channel/8]^ln_gpio_status_pre[current_channel/8])|ln_gpio_tx[current_channel/8])&(1<<(current_channel%8)))
	{
		// Check direction
		if (eeprom.ln_gpio_dir[current_channel/8]&(1<<(current_channel%8)))
		{
			// Transmit opcode if output (dir=1)
			if (ln_gpio_status[current_channel/8]&(1<<(current_channel%8)))
			{
				msg = eeprom.ln_gpio_opcode[current_channel*2+1];
				
				if (msg[0]!=0)
				{
					if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
					{
						ln_gpio_status_pre[current_channel/8] |= (1<<(current_channel%8));
						ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
						if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
						{
							ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
							ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
						}
					}
				}
			}
			else
			{
				msg = eeprom.ln_gpio_opcode[current_channel*2];
				
				if (msg[0]!=0)
				{
					if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
					{
						ln_gpio_status_pre[current_channel/8] &= ~(1<<(current_channel%8));
						ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
						if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
						{
							ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
							ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
						}
					}
				}
			}
		}
		else
		{
			// Transmit status if input (dir=0)
			if (ln_gpio_status[current_channel/8]&(1<<(current_channel%8)))
			{
				if (ln_create_message(eeprom.ln_gpio_opcode[current_channel*2+1]))
				{
					ln_gpio_status_pre[current_channel/8] |= (1<<(current_channel%8));
					ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
					if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
					{
						ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
						ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
					}
				}
			}
			else
			{
				if (ln_create_message(eeprom.ln_gpio_opcode[current_channel*2]))
				{
					ln_gpio_status_pre[current_channel/8] &= ~(1<<(current_channel%8));
					ln_gpio_tx[current_channel/8] &= ~(1<<(current_channel%8));
					if ((ln_gpio_status_ack[current_channel/8]&(1<<(current_channel%8)))==0)
					{
						ln_gpio_status_ack[current_channel/8] |= (1<<(current_channel%8));
						ln_gpio_ack_counter[current_channel] = eeprom.ln_gpio_ack_count;
					}
				}
			}
		}
	}
	else if (ln_gpio_tx_ack[current_channel/8]&(1<<(current_channel%8)))
	{
		if (ln_gpio_status[current_channel/8]&(1<<(current_channel%8)))
		{
			if (ln_create_message_ack(eeprom.ln_gpio_opcode[current_channel*2+1]))
			{
				ln_gpio_tx_ack[current_channel/8] &= ~(1<<(current_channel%8));
			}
		}
		else
		{
			if (ln_create_message_ack(eeprom.ln_gpio_opcode[current_channel*2]))
			{
				ln_gpio_tx_ack[current_channel/8] &= ~(1<<(current_channel%8));
			}
		}
	}
	
	current_channel++;
}

uint8_t ln_create_message(uint8_t *msg)
{
	if (msg[0]==0)
		return 1;
		
	switch (msg[0])
	{
		// translate switch request to switch report
		case OPC_SW_REQ:
			//msg0 = OPC_SW_REP;
			//msg2 = (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?OPC_SW_REP_HI:0);
			return (sendLocoNet4BytePacket(OPC_SW_REP, msg[1], (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?OPC_SW_REP_HI:0))==LN_DONE)?1:0;
	}
	
	return (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)?1:0;
}

uint8_t ln_create_message_ack(uint8_t *msg)
{
	if (msg[0]==0)
	{
		return 1;
	}
	
	if (msg[0]==OPC_SW_ACK)
	{
		return (sendLocoNet4BytePacket(OPC_LONG_ACK, msg[0]&0x7F, 0x7F)==LN_DONE)?1:0;
	}
	
	if (eeprom.ln_gpio_ack_count>0)
	{
		return (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2]^(1<<6))==LN_DONE)?1:0;
	}
	else
	{
		return 1;
	}
}

uint8_t ln_is_ack_message(uint8_t *msg)
{
	switch (msg[0])
	{		
		case OPC_SW_REQ:
			return (msg[2]&(1<<6))!=0;
		case OPC_SW_REP:
		case OPC_INPUT_REP:
			return (msg[2]&(1<<6))==0;
		default:
			return 1==0;
	}
}

void ln_gpio_process_rx(lnMsg *LnPacket)
{
	uint8_t index;
	uint8_t ack, cmd;
	
	if (!LnPacket)
		return;
	
	if (getLnMsgSize(LnPacket)>4)
		return;
	
	
	for (index=0;index<(2*LN_GPIO_CH_COUNT);index++)
	{
		// 1. Find a matching opcode
		// - either a direct match, or
		// - by matching a switch report/request pair
		if ((LnPacket->sr.command==eeprom.ln_gpio_opcode[index][0])
		|| ((LnPacket->sr.command==OPC_SW_REP) && (eeprom.ln_gpio_opcode[index][0]==OPC_SW_REQ)))
		{
			
		}
		else
			continue;
		
		// 2. Check first address byte
		if (LnPacket->srq.sw1!=eeprom.ln_gpio_opcode[index][1])
			continue;
			
		ack = 0;
		cmd = 0;
		
		// 3. Handle the opcode
		switch (LnPacket->sr.command)
		{
			case OPC_SW_REQ:
				if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
					continue;
				
				if ((LnPacket->srq.sw2&(1<<6))==0)
				{
					// Ack Command if direction input (==0)
					if ((eeprom.ln_gpio_dir[index/16]&(1<<((index/2)%8)))==0)
					{
						cmd = 1;
						ack = 1;
					}
					else
					{
						cmd = 1;
					}
				}
				else
				{
					// Ack received and direction output (==1)
					if ((eeprom.ln_gpio_dir[index/16]&(1<<((index/2)%8)))!=0)
					{
						// Ack
						ack = 1;
					}
				}
				break;
			case OPC_SW_REP:
				if (eeprom.ln_gpio_opcode[index][0]==OPC_SW_REQ)
				{
					if ((LnPacket->srq.sw2&0xF)!=(eeprom.ln_gpio_opcode[index][2]&0xF))
					{
						continue;
					}
				
					// We received the ACK, so we can clear the flag and stop transmitting
					if ((LnPacket->srq.sw2&(1<<6))==0)
					{
						ack = 1;
					}
				}
				else
				{
					if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
						continue;
					
					if ((LnPacket->srq.sw2&(1<<6))!=0)
					{
						// Ack Command if direction input (==0)
						if ((eeprom.ln_gpio_dir[index/16]&(1<<((index/2)%8)))==0)
						{
							cmd = 1;
							ack = 1;
						}
						else
						{
							cmd = 1;
						}
					}
					else
					{
						// Ack received and direction output (==1)
						if ((eeprom.ln_gpio_dir[index/16]&(1<<((index/2)%8)))!=0)
						{
							// Ack
							ack = 1;
						}
					}
				}	
				break;
			case OPC_INPUT_REP:
				if ((LnPacket->srq.sw2&(~(1<<6)))!=(eeprom.ln_gpio_opcode[index][2]&(~(1<<6))))
					continue;
					
				if ((LnPacket->srq.sw2&(1<<6))!=0)
				{
					// Ack Command if direction input (==0)
					if ((eeprom.ln_gpio_dir[index/16]&(1<<((index/2)%8)))==0)
					{
						cmd = 1;
						ack = 1;
					}
					else
					{
						cmd = 1;
					}
					
				}
				else
				{
					// Ack received and direction output (==1)
					if ((eeprom.ln_gpio_dir[index/16]&(1<<((index/2)%8)))!=0)
					{
						// Ack
						ack = 1;
					}
				}
				break;
			default:
				if (LnPacket->srq.sw2!=eeprom.ln_gpio_opcode[index][2])
					continue;
		}
		
		// We have a match, now set or clear the status
		if (cmd)
		{
			if ((index%2)==0)
			{
				ln_gpio_status[index/16] &= ~(1<<((index/2)%8));
				ln_gpio_status_pre[index/16] &= ~(1<<((index/2)%8));
			}
			else
			{
				ln_gpio_status[index/16] |= (1<<((index/2)%8));
				ln_gpio_status_pre[index/16] |= (1<<((index/2)%8));
			}
			
			ln_gpio_status_flag[index/16] |= (1<<((index/2)%8));
		
			if (ack && (eeprom.ln_gpio_ack_count>0))
			{
				ln_gpio_tx_ack[index/16] |= (1<<((index/2)%8));
				process_post(&ln_ack_process, ln_ack_event, (void *) (index/2));
			}
		}
		else
		{
			if (ack)
			{
				ln_gpio_status_ack[index/16] &= ~(1<<((index/2)%8));
			}
		}

	}
}

void ln_create_opcode(uint8_t *buf, uint8_t opc, uint16_t addr)
{
	if (addr==0)
	{
		buf[0] = 0;
		buf[3] = 0;
		
		return;
	}
	
	switch (opc)
	{
		case OPC_SW_REQ:
			addr--;
			buf[0] = OPC_SW_REQ;
			buf[3] = OPC_SW_REQ;
			buf[1] = addr&0x7F;
			buf[4] = buf[1];
			buf[5] = (addr>>7)&0x7F;
			buf[2] = buf[5]|OPC_SW_REQ_DIR;
			break;
		case OPC_INPUT_REP:
			addr--;
			buf[0] = OPC_INPUT_REP;
			buf[3] = OPC_INPUT_REP;
			buf[1] = (addr>>1)&0x7F;
			buf[4] = buf[1];
			buf[2] = ((addr>>8)&0x7F)|OPC_INPUT_REP_CB|((addr&1)?0x20:0);
			buf[5] = buf[2]|OPC_INPUT_REP_HI;
			break;
	}
}

void ln_load_board_config(void)
{
	uint8_t index;
	
	switch (deviceID)
	{
		case 0x2CFC:
			eeprom.sv_destination_id = deviceID;
			eeprom.sv_serial_number = deviceID;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_INPUT_REP, 251);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_INPUT_REP, 252);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 253);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 254);
			ln_create_opcode(eeprom.ln_gpio_opcode[8], OPC_INPUT_REP, 243);
			ln_create_opcode(eeprom.ln_gpio_opcode[10], OPC_INPUT_REP, 242);
			ln_create_opcode(eeprom.ln_gpio_opcode[12], OPC_INPUT_REP, 241);
			ln_create_opcode(eeprom.ln_gpio_opcode[14], OPC_INPUT_REP, 240);
			break;
		case 0x3924:
			eeprom.sv_destination_id = 492;
			eeprom.sv_serial_number = 492;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_INPUT_REP, 251);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_INPUT_REP, 252);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 253);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 254);
			ln_create_opcode(eeprom.ln_gpio_opcode[8], OPC_INPUT_REP, 243);
			ln_create_opcode(eeprom.ln_gpio_opcode[10], OPC_INPUT_REP, 242);
			ln_create_opcode(eeprom.ln_gpio_opcode[12], OPC_INPUT_REP, 241);
			ln_create_opcode(eeprom.ln_gpio_opcode[14], OPC_INPUT_REP, 240);
			break;
		case 0xDCB:
			eeprom.sv_destination_id = 494;
			eeprom.sv_serial_number = 494;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_INPUT_REP, 255);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_INPUT_REP, 244);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 246);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 245);
			ln_create_opcode(eeprom.ln_gpio_opcode[8], OPC_INPUT_REP, 247);
			ln_create_opcode(eeprom.ln_gpio_opcode[10], OPC_INPUT_REP, 249);
			ln_create_opcode(eeprom.ln_gpio_opcode[12], OPC_INPUT_REP, 250);
			ln_create_opcode(eeprom.ln_gpio_opcode[14], OPC_INPUT_REP, 248);
			break;
		case 0x7B5A:
			eeprom.sv_destination_id = 498;
			eeprom.sv_serial_number = 498;
			break;
		case 0x7C84:
			eeprom.sv_destination_id = 481;
			eeprom.sv_serial_number = 481;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 125);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 124);
			eeprom.servo_min[0] = 0x56FF;
			eeprom.servo_max[0] = 0xB300;
			eeprom.servo_min[1] = 0x58FF;
			eeprom.servo_max[1] = 0x9900;
			break;
		case 0x87E7:
			eeprom.sv_destination_id = 482;
			eeprom.sv_serial_number = 482;
			eeprom.servo_startup_delay = 200;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 123);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 122);
			eeprom.servo_min[0] = 0x89FF;
			eeprom.servo_max[0] = 0x5100;
			eeprom.servo_min[1] = 0x6FFF;
			eeprom.servo_max[1] = 0xA600;
			break;
		case 0xF23E:
			eeprom.sv_destination_id = 483;
			eeprom.sv_serial_number = 483;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 121);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 120);
			eeprom.servo_min[0] = 0x63FF;
			eeprom.servo_max[0] = 0xA200;
			eeprom.servo_min[1] = 0x54FF;
			eeprom.servo_max[1] = 0x8E00;
			break;
		case 0x7401:
			eeprom.sv_destination_id = 484;
			eeprom.sv_serial_number = 484;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 118);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 117);
			eeprom.servo_min[0] = 0x6CFF;
			eeprom.servo_max[0] = 0xB500;
			eeprom.servo_min[1] = 0x7FFF;
			eeprom.servo_max[1] = 0xCB00;
			break;
		case 0x85E:
			eeprom.sv_destination_id = 485;
			eeprom.sv_serial_number = 485;
			eeprom.servo_startup_delay = 100;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 116);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 115);
			eeprom.servo_min[0] = 0x4BFF;
			eeprom.servo_max[0] = 0x9800;
			eeprom.servo_min[1] = 0x68FF;
			eeprom.servo_max[1] = 0xB800;
			break;
		case 0x80EA:
			eeprom.sv_destination_id = 486;
			eeprom.sv_serial_number = 486;
			eeprom.servo_startup_delay = 50;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 113);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 112);
			eeprom.servo_min[0] = 0x75FF;
			eeprom.servo_max[0] = 0xA200;
			eeprom.servo_min[1] = 0xBAFF;
			eeprom.servo_max[1] = 0x7D00;
			break;
		case 0xC71F:
			eeprom.sv_destination_id = 487;
			eeprom.sv_serial_number = 487;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 111);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 110);
			eeprom.servo_min[0] = 0x71FF;
			eeprom.servo_max[0] = 0xB200;
			eeprom.servo_min[1] = 0x8CFF;
			eeprom.servo_max[1] = 0x4A00;
			break;
		case 0x1A28:
			eeprom.sv_destination_id = 488;
			eeprom.sv_serial_number = 488;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 114);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 109);
			eeprom.servo_min[0] = 0x9DFF;
			eeprom.servo_max[0] = 0x5A00;
			eeprom.servo_min[1] = 0xCDFF;
			eeprom.servo_max[1] = 0x9000;
			break;
		case 0x9CA9:
			eeprom.sv_destination_id = 489;
			eeprom.sv_serial_number = 489;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 108);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 107);
			eeprom.servo_min[0] = 0xA5FF;
			eeprom.servo_max[0] = 0x6E00;
			eeprom.servo_min[1] = 0x64FF;
			eeprom.servo_max[1] = 0xA000;
			break;
		case 0x2B9C:
			eeprom.sv_destination_id = 490;
			eeprom.sv_serial_number = 490;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 106);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 105);
			eeprom.servo_min[0] = 0xB4FF;
			eeprom.servo_max[0] = 0x8000;
			eeprom.servo_min[1] = 0x9DFF;
			eeprom.servo_max[1] = 0xDF00;
			break;
		case 0x594A:
			eeprom.sv_destination_id = 881;
			eeprom.sv_serial_number = 881;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*881);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*881+1);
			break;
		case 0x2F16:
			eeprom.sv_destination_id = 882;
			eeprom.sv_serial_number = 882;
			eeprom.servo_startup_delay = 200;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*882);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*882+1);
			break;
		case 0xD390:
			eeprom.sv_destination_id = 883;
			eeprom.sv_serial_number = 883;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*883);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*883+1);
			break;
		case 0x1820:
			eeprom.sv_destination_id = 884;
			eeprom.sv_serial_number = 884;
			eeprom.servo_startup_delay = 100;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*884);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*884+1);
			break;
		case 0x4974:
			eeprom.sv_destination_id = 885;
			eeprom.sv_serial_number = 885;
			eeprom.servo_startup_delay = 50;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*885);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*885+1);
			break;
		case 0x6A93:
			eeprom.sv_destination_id = 886;
			eeprom.sv_serial_number = 886;
			eeprom.servo_startup_delay = 25;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*886);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*886+1);
			break;
		case 0xc7a9:
			eeprom.sv_destination_id = 887;
			eeprom.sv_serial_number = 887;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*887);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*887+1);
			break;
		case 0xe73d:
			eeprom.sv_destination_id = 888;
			eeprom.sv_serial_number = 888;
			eeprom.servo_startup_delay = 250;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*888);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*888+1);
			break;
		case 0x4ee8:
			eeprom.sv_destination_id = 889;
			eeprom.sv_serial_number = 889;
			eeprom.servo_startup_delay = 100;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*889);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*889+1);
			break;
		case 0x2ffe:
			eeprom.sv_destination_id = 890;
			eeprom.sv_serial_number = 890;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*890);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*890+1);
			break;
		case 0x7aad:
			eeprom.sv_destination_id = 891;
			eeprom.sv_serial_number = 891;
			eeprom.servo_startup_delay = 200;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*891);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*891+1);
			break;
		case 0x9566:
			eeprom.sv_destination_id = 892;
			eeprom.sv_serial_number = 892;
			eeprom.servo_startup_delay = 25;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*892);
			//ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*892+1);
			eeprom.ln_gpio_opcode[2][0] =  0;
			break;
		case 0x499c:
			eeprom.sv_destination_id = 479;
			eeprom.sv_serial_number = 479;
			eeprom.servo_startup_delay = 125;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*479);
			//ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*479+1);
			eeprom.ln_gpio_opcode[2][0] =  0;
			break;
		case 0x9189:
			eeprom.sv_destination_id = 480;
			eeprom.sv_serial_number = 480;
			eeprom.servo_startup_delay = 25;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2*480);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2*480+1);
			break;
		case 0x8A41:
			eeprom.sv_destination_id = 100;
			eeprom.sv_serial_number = 100;
			eeprom.servo_startup_delay = 25;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 8);
			eeprom.ln_gpio_opcode[2][0] =  0;
			//ln_create_opcode(eeprom.ln_gpio_opcode[2], 0, 0);
			break;
		case 0x84AF:
			eeprom.sv_destination_id = 101;
			eeprom.sv_serial_number = 101;
			eeprom.servo_startup_delay = 100;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 2);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 3);
			break;
		case 0xBE0:
			eeprom.sv_destination_id = 102;
			eeprom.sv_serial_number = 102;
			eeprom.servo_startup_delay = 150;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 4);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 5);
			break;
		case 0xF65E:
			eeprom.sv_destination_id = 103;
			eeprom.sv_serial_number = 103;
			eeprom.servo_startup_delay = 50;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 9);
			eeprom.ln_gpio_opcode[2][0] =  0;
			//ln_create_opcode(eeprom.ln_gpio_opcode[2], 0, 0);
			break;
		case 0x4C2E:
			eeprom.sv_destination_id = 104;
			eeprom.sv_serial_number = 104;
			eeprom.servo_startup_delay = 75;
			eeprom.servo_timeout = 256;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 6);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 7);
			break;
		case 0xF703:
			eeprom.sv_destination_id = 42;
			eeprom.sv_serial_number = 42;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 1);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 2);
			return;
			break;
		case 0x6CF3:
			eeprom.sv_destination_id = deviceID;
			eeprom.sv_serial_number = deviceID;
			ln_create_opcode(eeprom.ln_gpio_opcode[0], OPC_SW_REQ, 125);
			ln_create_opcode(eeprom.ln_gpio_opcode[2], OPC_SW_REQ, 124);
			ln_create_opcode(eeprom.ln_gpio_opcode[4], OPC_INPUT_REP, 126);
			ln_create_opcode(eeprom.ln_gpio_opcode[6], OPC_INPUT_REP, 127);
			for (index=8;index<16;index++)
			{
				eeprom.ln_gpio_opcode[index][0] = 0;
			}
			return;
			break;
		default:
			return;
	}
	
	for (index=4;index<16;index++)
	{
		eeprom.ln_gpio_opcode[index][0] = 0;
	}
	/*
	eeprom.ln_threshold = 50;
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	*/
}
