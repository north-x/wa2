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

/*
 * sv_table.h
 *
 */ 

#ifndef PARAMETER_TABLE_H_
#define PARAMETER_TABLE_H_

#include "config.h"
#include "eeprom.h"
#include "usb/usb.h"
#include <avr/wdt.h>
#include <string.h>
#include "ln_buf.h"

void cmd_exec(void);
void tse_update_page_read(void);
void tse_update_page_write(void);
void tse_update_time_ratio(void);
void dimm_target_parameter_update(void);
void dimm_delta_parameter_update(void);
void ln_update_threshold(void);

extern LnBuf LnBuffer;
extern rwSlotDataMsg rSlot;

uint8_t dimm_target_temp;
uint8_t dimm_delta_temp;
uint8_t dimm_parameter_select;
uint8_t cmd_register = 0;


// Block mapping of TSE scripts starting at SV 112/170/240/304 (0x70/0xB0/0xF0/0x130)
SV_BLOCK_TABLE_BEGIN()
SV_BLOCK_TABLE_END();

SV_TABLE_BEGIN()
SV_CONST(1, "EEPROM Size", 0)
SV_CONST(2, "SW Version", SOFTWARE_VERSION)
SV_LSB(3, "Serial Number L", eeprom.sv_serial_number, 0)
SV_MSB(4, "Serial Number H", eeprom.sv_serial_number, 0)
SV(5, "Command Register", cmd_register, cmd_exec)
SV(16, "PWM Port Select", dimm_parameter_select, 0)
SV(17, "PWM Port Target", dimm_target_temp, dimm_target_parameter_update)
SV(18, "PWM Port Delta", dimm_delta_temp, dimm_delta_parameter_update)
SV(19, "Relay Command", relay_request, 0)

SV(89, "PWM Port 1 Target", pwm_port[0].dimm_target, 0)
SV(90, "PWM Port 2 Target", pwm_port[1].dimm_target, 0)
SV(91, "PWM Port 3 Target", pwm_port[2].dimm_target, 0)
SV(92, "PWM Port 4 Target", pwm_port[3].dimm_target, 0)
SV(93, "PWM Port 5 Target", pwm_port[4].dimm_target, 0)
SV(94, "PWM Port 6 Target", pwm_port[5].dimm_target, 0)
SV(95, "PWM Port 7 Target", pwm_port[6].dimm_target, 0)
SV(96, "PWM Port 1 Delta", pwm_port[0].dimm_delta, 0)
SV(97, "PWM Port 2 Delta", pwm_port[1].dimm_delta, 0)
SV(98, "PWM Port 3 Delta", pwm_port[2].dimm_delta, 0)
SV(99, "PWM Port 4 Delta", pwm_port[3].dimm_delta, 0)
SV(100, "PWM Port 5 Delta", pwm_port[4].dimm_delta, 0)
SV(101, "PWM Port 6 Delta", pwm_port[5].dimm_delta, 0)
SV(102, "PWM Port 7 Delta", pwm_port[6].dimm_delta, 0)
SV(103, "Servo 1 Setpoint", servo[0].position_setpoint, 0)
SV(104, "Servo 1 Delta", servo[0].time_delta, 0)
SV(105, "Servo 2 Setpoint", servo[1].position_setpoint, 0)
SV(106, "Servo 2 Delta", servo[1].time_delta, 0)
SV_CONST(107, "IR Read Parameter Address", 107)
SV_CONST(108, "IR Read Parameter", 108)
SV_CONST(109, "IR Read Parameter Value", 109)

#define SV_CFG
#include "config.h"
#undef SV_CFG

SV_TABLE_END()


void cmd_exec(void)
{
	switch (cmd_register)
	{
		default:
		case 0:
			break;
		case 1:
			eeprom_sync_storage();
			break;
		case 2:
			wdt_enable(WDTO_1S);
			break;
		case 3:
			eeprom_load_defaults();
			break;
		case 4:
			USB_enter_bootloader();
			break;
		case 5:
			rSlot.slot = 0;
			servo_status |= (1<<SERVO_STATUS_PWR_ALWAYS_ON);
			break;
	}
	
	cmd_register = 0;
}

void dimm_target_parameter_update(void)
{
	uint8_t index;
	
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if (dimm_parameter_select&(1<<index))
		{
			pwm_port[index].dimm_target = dimm_target_temp;
		}
	}
}

void dimm_delta_parameter_update(void)
{
	uint8_t index;

	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if (dimm_parameter_select&(1<<index))
		{
			pwm_port[index].dimm_delta = dimm_delta_temp;
		}
	}
}

#endif /* PARAMETER_TABLE_H_ */