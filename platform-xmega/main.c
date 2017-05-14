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
#include <avr/interrupt.h>
#include <stdlib.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "sys/utils.h"
#include "eeprom.h"
#include "wa2.h"
#include "usb_support.h"
#include "ln_support.h"
#include "usb/usb.h"
#include "port.h"
#include "servo.h"

void ubasic_start(void);
void init(void);
PROCESS_NAME(port_process);
uint16_t deviceID;

void init(void)
{
	usb_configure_clock();
	
	eeprom_load_status();
	eeprom_load_storage();

	deviceID = getID16();
	srand(deviceID);
	
	process_init();
	clock_init();
	
	process_start(&etimer_process, NULL);
	
	usb_process_init();
	loconet_init();
	port_init();
	servo_init();
	
	wa2_update_configuration();
	ubasic_start();
	
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();
}

int main(void)
{
	init();
	
    while(1)
    {
        process_run();
    }
}

void wa2_update_configuration(void)
{
	MAP_BITS(eeprom.configA, servo_status, WA2_CONF_SERVO_PWR_ALWAYS_ON, SERVO_STATUS_PWR_ALWAYS_ON);
	MAP_BITS(eeprom.configA, servo_status, WA2_CONF_SERVO_ENABLE_PWM_A, SERVO_STATUS_ENABLE_PWM_A);
	MAP_BITS(eeprom.configA, servo_status, WA2_CONF_SERVO_ENABLE_PWM_B, SERVO_STATUS_ENABLE_PWM_B);
	MAP_BITS(eeprom.configA, port_mode, WA2_CONF_RELAY_MONOSTABLE, PORT_MODE_RELAY_MONOSTABLE);
	MAP_BITS(eeprom.configA, port_mode, WA2_CONF_PWM_OUTPUTS_ENABLE, PORT_MODE_PWM_ENABLE);
	MAP_BITS(eeprom.configA, port_mode, WA2_CONF_PWM_CHANNEL7_ENABLE, PORT_MODE_PWM_CH7_ENABLE);
	MAP_BITS(eeprom.configA, port_mode, WA2_CONF_INPUTS_PULLUP_ENABLE, PORT_MODE_PULLUP_ENABLE);
	
	servo_mode_update();
	port_update_configuration();
}
