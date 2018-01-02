/*
 * Copyright (c) 2017, Manuel Vetterli
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
#include "ln_interface.h"
#include "wa2.h"
#include "config.h"
#include "eeprom.h"

PROCESS(wa2_process,"WA2 Application Process");
rwSlotDataMsg rSlot;

PROCESS_THREAD(wa2_process, ev, data)
{
	
	PROCESS_BEGIN();
	
	// Initialization
	rSlot.slot = 0xFF;
	
	while (1)
	{
		PROCESS_PAUSE();
		
		// Servo 1
		// Check if a new command was received
		if (ln_gpio_status_flag[0]&(1<<0))
		{
			// Set new position accordingly
			if (ln_gpio_status[0]&(1<<0))
			{
				servo[0].position_setpoint = 230;
			}
			else
			{
				servo[0].position_setpoint = 25;
			}

			// Turn on relay if position greater than 127
			if (servo[0].position_actual>127)
			{
				relay_request |= (1<<0);
			}
			else
			{
				relay_request &= ~(1<<0);		
			}
			
			// If we reached the final position, transmit a switch report
			if (servo[0].position_actual==servo[0].position_setpoint)
			{
				ln_gpio_tx[0] |= (1<<0);
				ln_gpio_status_flag[0] &= ~(1<<0);
			}
			
		}
		
		// Servo 2
		// Check if a new command was received
		if (ln_gpio_status_flag[0]&(1<<1))
		{
			// Set new position accordingly
			if (ln_gpio_status[0]&(1<<1))
			{
				servo[1].position_setpoint = 230;
			}
			else
			{
				servo[1].position_setpoint = 25;
			}

			// Turn on relay if position greater than 127
			if (servo[1].position_actual>127)
			{
				relay_request |= (1<<1);
			}
			else
			{
				relay_request &= ~(1<<1);
			}
			
			// If we reached the final position, transmit the new status
			if (servo[1].position_actual==servo[1].position_setpoint)
			{
				ln_gpio_tx[0] |= (1<<1);
				ln_gpio_status_flag[0] &= ~(1<<1);
			}
			
		}
	}
	
	PROCESS_END();
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
					rSlot.adr = eeprom.sv_destination_id&0x7F;
					rSlot.adr2 = (eeprom.sv_destination_id>>7)&0x7F;
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
					if (sendLocoNetPacket(&SendPacket)!=LN_DONE)  // , LN_BACKOFF_INITIAL + (eeprom.sv_destination_id % (uint8_t) 10))
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
							ln_gpio_status[0] |= (1<<1);
							ln_gpio_status_pre[0] |= (1<<1);
							servo_act = &servo[1].max;
							servo_speed = &servo[1].time_ratio;
						}
						else
						{
							ln_gpio_status[0] |= (1<<0);
							ln_gpio_status_pre[0] |= (1<<0);
							servo_act = &servo[0].max;
							servo_speed = &servo[0].time_ratio;
						}
					}
					else
					{
						// F0: 0 -> Servo 1 / 1 -> Servo 2
						if (LnPacket->ldf.dirf&(1<<4))
						{
							ln_gpio_status[0] &= ~(1<<1);
							ln_gpio_status_pre[0] &= ~(1<<1);
							servo_act = &servo[1].min;
							servo_speed = &servo[1].time_ratio;
						}
						else
						{
							ln_gpio_status[0] &= ~(1<<0);
							ln_gpio_status_pre[0] &= ~(1<<0);
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
				if (snd_changes&(1<<3))
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
