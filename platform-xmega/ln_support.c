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
#include "sys/process.h"
#include "sys/etimer.h"
#include "eeprom.h"
#include "loconet/utils.h"
#include "ln_interface.h"
#include "sv.h"
#include "IdStorage.h"
#include "ln_support.h"
#include "usb_support.h"
#include "servo.h"

PROCESS(ln_process, "Loconet Handler");
static LnBuf LnBuffer;
uint8_t ln_gpio_status;
uint8_t ln_gpio_status_pre;
uint8_t ln_gpio_status_tx;
uint8_t ln_gpio_opcode_tx;
uint8_t ln_gpio_opcode_tx2;

extern uint16_t deviceID;
rwSlotDataMsg rSlot;

void loconet_init(void)
{
	initLnBuf(&LnBuffer);
	initLocoNet(&LnBuffer);
	ACA.CTRLB = ((eeprom.ln_threshold/4)-1)&0x3F;
	
	if (readSVDestinationId()==0xFFFF)
	{
		writeSVDestinationId(deviceID);
	}
	
	if (eeprom.sv_serial_number==0xFFFF)
	{
		eeprom.sv_serial_number = deviceID;
		eeprom_sync_storage();
	}
	
	process_start(&ln_process, NULL);
}

PROCESS_THREAD(ln_process, ev, data)
{
	lnMsg *LnPacket;
	
	PROCESS_BEGIN();
	
	// Initialization
	ln_gpio_status = eeprom_status.ln_gpio_status;
	ln_gpio_status_pre = eeprom_status.ln_gpio_status;
	rSlot.slot = 0xFF;
	
	while (1)
	{
		PROCESS_PAUSE();
		
		eeprom_status.ln_gpio_status = ln_gpio_status;
		
		doSVDeferredProcessing();
		
		LnPacket = recvLocoNetPacket();
	
		if (LnPacket)
		{   
			sendLocoNetPacketUSB(LnPacket);
			
			ln_gpio_process_rx(LnPacket);
			
			ln_throttle_process(LnPacket);
			
			if ((LnPacket->sz.command == OPC_GPON))
			{
				// Force transmission of current state
				ln_gpio_status_tx = 0xFF;
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
	static uint8_t current_bit = 0;
	
	if (current_bit>7)
		current_bit = 0;
	
	if (((ln_gpio_status^ln_gpio_status_pre)|ln_gpio_status_tx) & (1<<current_bit))
	{
		if (ln_gpio_status&(1<<current_bit))
		{
			if (ln_create_message(eeprom.ln_gpio_opcode[current_bit*2]))
			{
				ln_gpio_status_pre |= (1<<current_bit);
				ln_gpio_status_tx &= ~(1<<current_bit);				
			}					
		}
		else
		{
			if (ln_create_message(eeprom.ln_gpio_opcode[current_bit*2+1]))
			{
				ln_gpio_status_pre &= ~(1<<current_bit);
				ln_gpio_status_tx &= ~(1<<current_bit);				
			}
		}
	}
	
	if (ln_gpio_opcode_tx&(1<<current_bit))
	{
		msg = eeprom.ln_gpio_opcode[current_bit];
		if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
			ln_gpio_opcode_tx &= ~(1<<current_bit);
	}

	if (ln_gpio_opcode_tx2&(1<<current_bit))
	{
		msg = eeprom.ln_gpio_opcode[current_bit+8];
		if (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)
			ln_gpio_opcode_tx2 &= ~(1<<current_bit);
	}
	
	current_bit++;
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
			return (sendLocoNet4BytePacket(OPC_SW_REP, msg[1], (msg[2]&0xF)|OPC_SW_REP_SW|OPC_SW_REP_INPUTS|((msg[2]&OPC_SW_REQ_DIR)?0:OPC_SW_REP_HI))==LN_DONE)?1:0;
	}
	
	return (sendLocoNet4BytePacket(msg[0]|(1<<7), msg[1], msg[2])==LN_DONE)?1:0;
}

void ln_gpio_process_rx(lnMsg *LnPacket)
{
	uint8_t index;
	
	if (!LnPacket)
	return;
	
	if (getLnMsgSize(LnPacket)>4)
	return;
	
	for (index=0;index<16;index++)
	{		
		if (LnPacket->sr.command!=eeprom.ln_gpio_opcode[index][0])
		continue;
		
		if (LnPacket->srq.sw1!=eeprom.ln_gpio_opcode[index][1])
		continue;
		
		if (LnPacket->srq.sw2!=eeprom.ln_gpio_opcode[index][2])
		continue;
		
		// We have a match, now set or clear the status
		if ((index%2)==0)
		{
			ln_gpio_status &= ~(1<<(index/2));
			ln_gpio_status_pre &= ~(1<<(index/2));
		}
		else
		{
			ln_gpio_status |= (1<<(index/2));
			ln_gpio_status_pre |= (1<<(index/2));
		}
	}
}

void ln_throttle_process(lnMsg *LnPacket)
{
	static lnMsg SendPacket;
	static uint16_t servo_base;
	static uint16_t *servo_act;
	static int8_t servo_mult;
	static uint8_t *servo_speed;
	
	switch (LnPacket->data[0])
	{
		case OPC_LOCO_ADR:
			rSlot.adr = eeprom.sv_destination_id&0x7F;
			rSlot.adr2 = (eeprom.sv_destination_id>>7)&0x7F;
			rSlot.stat = LOCO_IDLE | DEC_MODE_128;
			rSlot.id1 = 0;
			rSlot.id2 = 0;

			if ((LnPacket->la.adr_hi==rSlot.adr2) && (LnPacket->la.adr_lo==rSlot.adr))
			{
				rSlot.slot = (eeprom.sv_destination_id&0x7F)?(eeprom.sv_destination_id&0x7F):1;
				SendPacket.sd.command   = OPC_SL_RD_DATA  ; //opcode
				SendPacket.sd.mesg_size = 14              ; // length
				SendPacket.sd.slot      = rSlot.slot      ; // slot    2
				SendPacket.sd.stat      = rSlot.stat      ; // stat    3
				SendPacket.sd.adr       = rSlot.adr       ; // adr     4
				SendPacket.sd.spd       = rSlot.spd       ; // spd     5
				SendPacket.sd.dirf      = rSlot.dirf      ; // dirf    6
				SendPacket.sd.trk       = rSlot.trk       ; // trk     7
				SendPacket.sd.ss2       = rSlot.ss2       ; // ss2     8
				SendPacket.sd.adr2      = rSlot.adr2      ; // adr2    9
				SendPacket.sd.snd       = rSlot.snd       ; // snd    10
				SendPacket.sd.id1       = rSlot.id1       ; // id1    11
				SendPacket.sd.id2       = rSlot.id2       ; // id2    12
					
				sendLocoNetPacket(&SendPacket);
			}
			else
			{
				rSlot.slot = 0xFF;
			}
			break;
		case OPC_MOVE_SLOTS:
			if (LnPacket->sm.src==LnPacket->sm.dest && LnPacket->sm.src==rSlot.slot)
			{
				if (rSlot.slot==0)
				{
					rSlot.slot = (eeprom.sv_destination_id&0x7F)?(eeprom.sv_destination_id&0x7F):1;
				}
				
				servo_act = &servo[0].min;
				servo_base = *servo_act;
				servo_mult = 1;
				servo_speed = &servo[0].time_ratio;
				rSlot.stat = DEC_MODE_128 | LOCO_IN_USE;
				SendPacket.sd.command   = OPC_SL_RD_DATA  ; //opcode
				SendPacket.sd.mesg_size = 14              ; // length
				SendPacket.sd.slot      = rSlot.slot      ; // slot    2
				SendPacket.sd.stat      = rSlot.stat      ; // stat    3
				SendPacket.sd.adr       = rSlot.adr       ; // adr     4
				SendPacket.sd.spd       = rSlot.spd       ; // spd     5
				SendPacket.sd.dirf      = rSlot.dirf      ; // dirf    6
				SendPacket.sd.trk       = rSlot.trk       ; // trk     7
				SendPacket.sd.ss2       = rSlot.ss2       ; // ss2     8
				SendPacket.sd.adr2      = rSlot.adr2      ; // adr2    9
				SendPacket.sd.snd       = rSlot.snd       ; // snd    10
				SendPacket.sd.id1       = rSlot.id1       ; // id1    11
				SendPacket.sd.id2       = rSlot.id2       ; // id2    12
				
				if (LnPacket->sm.src==0)
				{
					if (sendLocoNetPacketTry(&SendPacket, LN_BACKOFF_INITIAL + (eeprom.sv_destination_id % (uint8_t) 10))!=LN_DONE)
						rSlot.slot = 0;
				}
				else
					sendLocoNetPacket(&SendPacket);
			}
			break;
		case OPC_WR_SL_DATA:
			rSlot.id1 = LnPacket->sd.id1;
			rSlot.id2 = LnPacket->sd.id2;
			sendLocoNet4BytePacket(OPC_LONG_ACK, OPC_WR_SL_DATA&0x7F, 0);
			break;
		case OPC_LOCO_DIRF:
			if (LnPacket->ldf.slot==rSlot.slot)
			{
				uint8_t dirf_changes = rSlot.dirf^LnPacket->ldf.dirf;
					
				// Changes in DIR or F0
				if (dirf_changes&((1<<4)|(1<<5)))
				{
					// Restore "unsaved" position
					*servo_act = servo_base;
						
					// Reset multiplier
					servo_mult = 1;
						
					// Direction 0/forward = right 1/reversed=left
					if ((LnPacket->ldf.dirf&(1<<5))==0)
					{
						// F0: 0 -> Servo 1 / 1 -> Servo 2
						if (LnPacket->ldf.dirf&(1<<4))
						{
							ln_gpio_status |= (1<<1);
							ln_gpio_status_pre |= (1<<1);
							servo_act = &servo[1].max;
							servo_speed = &servo[1].time_ratio;
						}
						else
						{
							ln_gpio_status |= (1<<0);
							ln_gpio_status_pre |= (1<<0);
							servo_act = &servo[0].max;
							servo_speed = &servo[0].time_ratio;
						}
					}
					else
					{
						// F0: 0 -> Servo 1 / 1 -> Servo 2
						if (LnPacket->ldf.dirf&(1<<4))
						{
							ln_gpio_status &= ~(1<<1);
							ln_gpio_status_pre &= ~(1<<1);
							servo_act = &servo[1].min;
							servo_speed = &servo[1].time_ratio;
						}
						else
						{
							ln_gpio_status &= ~(1<<0);
							ln_gpio_status_pre &= ~(1<<0);
							servo_act = &servo[0].min;
							servo_speed = &servo[0].time_ratio;
						}
					}
						
					servo_base = *servo_act;
				}
					
				// F1
				if ((dirf_changes&(1<<0)))
				{
					switch (servo_mult)
					{
						case 1:
							servo_mult = 2;
							break;
						case 2:
							servo_mult = 3;
							break;
						case 3:
						default:
							servo_mult = 1;
							break;
					}
				}
					
				// F2
				if ((dirf_changes&(1<<1)))
				{
					switch (servo_mult)
					{
						case -1:
							servo_mult = -2;
							break;
						case -2:
							servo_mult = -3;
							break;
						case -3:
						default:
							servo_mult = -1;
							break;
					}
				}
					
				if (dirf_changes)
				{
					//sendLocoNet4BytePacket(OPC_LOCO_DIRF, rSlot.slot, LnPacket->ldf.dirf);
					sendLocoNet4BytePacket(OPC_LOCO_SPD, rSlot.slot, 1);
				}
					
				rSlot.dirf = LnPacket->ldf.dirf;
			}
			break;
		case OPC_LOCO_SPD:
			if (LnPacket->lsp.slot==rSlot.slot)
			{
				if (LnPacket->lsp.spd==1)
				{
					servo_base = *servo_act;
				}
				else if (LnPacket->lsp.spd>1)
				{
					int32_t temp32 = 0;
					switch (servo_mult)
					{
						case 1:
							temp32 = (int32_t) servo_base + (LnPacket->lsp.spd - 1)*256;
							break;
						case 2:
							temp32 = (int32_t) servo_base + (LnPacket->lsp.spd - 1)*32;
							break;
						case 3:
							temp32 = (int32_t) servo_base + (LnPacket->lsp.spd - 1);
							break;
						case -1:
							temp32 = (int32_t) servo_base - (LnPacket->lsp.spd - 1)*256;
							break;
						case -2:
							temp32 = (int32_t) servo_base - (LnPacket->lsp.spd - 1)*32;
							break;
						case -3:
							temp32 = (int32_t) servo_base - (LnPacket->lsp.spd - 1);
							break;
					}
						
					if (temp32<0)
						*servo_act = 0;
					else if (temp32>65535)
						*servo_act = 65535;
					else
						*servo_act = temp32;
				}
				else
				{
					*servo_act = servo_base;
				}
			}
			break;
		case OPC_LOCO_SND:
			if (LnPacket->ls.slot==rSlot.slot)
			{
				uint8_t snd_changes = rSlot.snd^LnPacket->ls.snd;
						
				// F5 Increase Speed
				if (snd_changes&(1<<0))
				{
					if (*servo_speed>1)
						(*servo_speed)--;
				}
				// F6 Decrease Speed
				if (snd_changes&(1<<1))
				{
					if (*servo_speed<255)
					(*servo_speed)++;
				}
				// F7 Load EEPROM Settings
				if (snd_changes&(1<<2))
				{
					servo[0].min = eeprom.servo_min[0];
					servo[1].min = eeprom.servo_min[1];
							
					servo[0].max = eeprom.servo_max[0];
					servo[1].max = eeprom.servo_max[1];
							
					servo[0].time_ratio = eeprom.servo_time_ratio[0];
					servo[1].time_ratio = eeprom.servo_time_ratio[1];
				}
				// F8 Save Config
				if (snd_changes&(1<<2))
				{
					eeprom.servo_min[0] = servo[0].min;
					eeprom.servo_min[1] = servo[1].min;
							
					eeprom.servo_max[0] = servo[0].max;
					eeprom.servo_max[1] = servo[1].max;
							
					eeprom.servo_time_ratio[0] = servo[0].time_ratio;
					eeprom.servo_time_ratio[1] = servo[1].time_ratio;
							
					eeprom_sync_storage();
				}
						
				rSlot.snd = LnPacket->ls.snd;
			}
			break;
	}
}
