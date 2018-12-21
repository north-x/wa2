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

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "sys/process.h"
#include "sys/etimer.h"
#include "port.h"
#include "eeprom.h"
#include "ln_support.h"

uint8_t port[4][8]; // 4 ports A-D, 8 outputs each, 0=off, 63=on
t_pwm_port pwm_port[PWM_PORT_COUNT];
int8_t pwm_target[PWM_PORT_COUNT];
uint16_t pwm_update_trig;
uint16_t pwm_at_setpoint;
uint8_t pwm_target_temp;
uint8_t pwm_delta_temp;
uint16_t pwm_parameter_select;

uint16_t port_do;
uint16_t port_di;
uint16_t port_di_mapped;
uint16_t port_do_mapped;
uint8_t port_user;

// Gamma correction tables
uint8_t pwm_gamma_4b6b[16] = { 0, 1, 2, 5, 8, 11, 15, 19, 24, 28, 34, 39, 44, 50, 56, 63 };
uint8_t pwm_gamma_5b6b[32] = { 0, 1, 2, 3, 4, 5, 7, 8, 9, 11, 13, 14, 16, 18, 20, 22, 24, 27, 29, 32, 34, 37, 39, 42, 45, 48, 51, 54, 58, 61, 63, 63 };
// Binary code modulation tables (4 ports, 6 timeslices)
uint8_t pwm_bitslice[6][4];
uint8_t pwm_bitslice_shadow[6][4];
volatile uint8_t pwm_bitslice_update_needed;

static struct etimer port_timer;

PROCESS(port_process, "Port IO Handling");

PROCESS_THREAD(port_process, ev, data)
{
	static uint16_t ct0, ct1;
	uint16_t i, in;
	
	PROCESS_BEGIN();
	
	port_init();
	etimer_set(&port_timer, 20E-3*CLOCK_SECOND);
	
	while(1)
	{
		PROCESS_YIELD();
		// DI (digital input) handling
		PORT_PIN_STATUS(in);
		i = port_di ^ ~in;			// key changed ?

		ct0 = ~( ct0 & i );			// reset or count ct0
		ct1 = ct0 ^ (ct1 & i);		// reset or count ct1
		i &= ct0 & ct1;				// count until roll over ?
		port_di ^= i;				// then toggle debounced state

		port_do = ln_gpio_status[0] | (ln_gpio_status[1]<<8);
		port_do_mapping();
		relay_process();
		pwm_step();
		port_update_mapping();
		pwm_update_bitslices();


		// Overwrite status bits if dir=1 (LN output)
		ln_gpio_status[0] = (ln_gpio_status[0]&(~eeprom.ln_gpio_dir[0]))|(((uint8_t) port_di_mapped)&eeprom.ln_gpio_dir[0]);
		ln_gpio_status[1] = (ln_gpio_status[1]&(~eeprom.ln_gpio_dir[1]))|(((uint8_t) (port_di_mapped>>8))&eeprom.ln_gpio_dir[1]);
		
		etimer_reset(&port_timer);
	}
	
	PROCESS_END();
}

void port_update_configuration(void)
{	
	port_di_init();
	pwm_init();
	relay_init();
}

void port_init(void)
{	
	// S_Power is configured regardless of mode
	PORTA.PIN5CTRL = PORT_OPC_TOTEM_gc;
	PORTA.OUTCLR = (1<<5);
	PORTA.DIRSET = (1<<5);

	port_di_init();
	pwm_init();
	relay_init();
}

void port_di_init(void)
{
	if (eeprom.port_config&(1<<PORT_MODE_PULLUP_ENABLE))
	{
		PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN1CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN0CTRL = (PORTC.PIN0CTRL&(~PORT_OPC_gm))|PORT_OPC_PULLUP_gc;
		PORTC.PIN1CTRL = (PORTC.PIN1CTRL&(~PORT_OPC_gm))|PORT_OPC_PULLUP_gc;
		
		PORTA.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTA.PIN7CTRL = PORT_OPC_PULLUP_gc;
		PORTB.PIN0CTRL = PORT_OPC_PULLUP_gc;
		PORTB.PIN1CTRL = PORT_OPC_PULLUP_gc;
	}
	else
	{
		PORTC.PIN7CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN0CTRL = (PORTC.PIN0CTRL&(~PORT_OPC_gm))|PORT_OPC_TOTEM_gc;
		PORTC.PIN1CTRL = (PORTC.PIN1CTRL&(~PORT_OPC_gm))|PORT_OPC_TOTEM_gc;

		PORTA.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTA.PIN7CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN1CTRL = PORT_OPC_TOTEM_gc;
	}

	MAP_BITS(eeprom.port_dir, PORTC.DIR, 0, 2);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 1, 3);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 2, 7);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 3, 6);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 4, 5);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 5, 5);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 6, 1);
	MAP_BITS(eeprom.port_dir, PORTD.DIR, 7, 0);
	if (!(eeprom.port_config&(1<<PORT_MODE_RELAY)))
	{
		MAP_BITS(eeprom.port_dir, PORTA.DIR, 8, 6);
		MAP_BITS(eeprom.port_dir, PORTA.DIR, 9, 7);
		MAP_BITS(eeprom.port_dir, PORTB.DIR, 10, 0);
		MAP_BITS(eeprom.port_dir, PORTB.DIR, 11, 1);
	}
	
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 12, 0);
	MAP_BITS(eeprom.port_dir, PORTC.DIR, 13, 1);

	// Initial key state
	PORT_PIN_STATUS(port_di);
}

uint8_t relay_cmd = 0;
uint8_t relay_state = 0;
uint8_t relay_request = 0;

void relay_init(void)
{
	if (!(eeprom.port_config&(1<<PORT_MODE_RELAY)))
	{
		return;	
	}
	
	relay_request = eeprom_status.relay_request;
	relay_state = !relay_request;

	port[0][6] = 0;	// RC4
	port[0][7] = 0;	// RC1
	port[1][0] = 0;	// RC3
	port[1][1] = 0; // RC2
//	PORTA.OUTCLR = (1<<7)|(1<<6); /* RC1 RC4 */
//	PORTB.OUTCLR = (1<<0)|(1<<1); /* RC2 RC3 */
	PORTA.DIRSET = (1<<7)|(1<<6);
	PORTB.DIRSET = (1<<0)|(1<<1);

	if (eeprom.port_config&(1<<PORT_MODE_RELAY_MONOSTABLE))
	{
		//PORTA.OUTSET |= (1<<6);
		port[0][6] = 63;
	}
}

void relay_governor(void)
{
	uint8_t relay_change;
	
	if (relay_cmd!=0)
	return;
	
	relay_change = relay_request^relay_state;
	if (relay_change)
	{
		// Override Relay 1
		if (relay_request&(1<<2))
		{
			if (relay_change&((1<<2)|(1<<4)))
			{
				if (relay_request&(1<<4))
				{
					relay_cmd |= RELAY_CMD_RIGHT1;
				}
				else
				{
					relay_cmd |= RELAY_CMD_LEFT1;
				}
			}
		}
		else
		{
			if (relay_change&(1<<0))
			{
				if (relay_request&(1<<0))
				{
					relay_cmd |= RELAY_CMD_RIGHT1;
				}
				else
				{
					relay_cmd |= RELAY_CMD_LEFT1;
				}
			}
		}
		
		// Override Relay 2
		if (relay_request&(1<<3))
		{
			if (relay_change&((1<<3)|(1<<5)))
			{
				if (relay_request&(1<<5))
				{
					relay_cmd |= RELAY_CMD_RIGHT2;
				}
				else
				{
					relay_cmd |= RELAY_CMD_LEFT2;
				}
			}
		}
		else
		{
			if (relay_change&(1<<1))
			{
				if (relay_request&(1<<1))
				{
					relay_cmd |= RELAY_CMD_RIGHT2;
				}
				else
				{
					relay_cmd |= RELAY_CMD_LEFT2;
				}
			}
		}
		
		relay_cmd |= 0xF0;
		relay_state = relay_request;
		eeprom_status.relay_request = relay_request;
	}
}

#define RELAY_CHECK_DONE(RCMD)	\
if ((relay_cmd&0xF0)==0)	\
{ \
	relay_cmd &= ~(RCMD); \
	relay_cmd |= 0xF0; \
	\
	/* Idle State */ \
	/*PORTA.OUTCLR = (1<<7)|(1<<6); *//* RC1 RC4 */ \
	/*PORTB.OUTCLR = (1<<0)|(1<<1); *//* RC2 RC3 */\
	port[0][6] = 0;	/* RC4 */ \
	port[0][7] = 0;	/* RC1 */ \
	port[1][0] = 0;	/* RC3 */ \
	port[1][1] = 0; /* RC2 */ \
} \

void relay_process(void)
{
	relay_governor();
		
	if (eeprom.port_config&(1<<PORT_MODE_RELAY_MONOSTABLE))
	{
		if (relay_cmd&RELAY_CMD_RIGHT1)
		{
			//PORTA.OUTSET = (1<<7);
			port[0][7] = 63;
		}
		else if (relay_cmd&RELAY_CMD_LEFT1)
		{
			//PORTA.OUTCLR = (1<<7);
			port[0][7] = 0;
		}
			
		if (relay_cmd&RELAY_CMD_RIGHT2)
		{
			//PORTB.OUTSET = (1<<1);
			port[1][1] = 63;
		}
		else if (relay_cmd&RELAY_CMD_LEFT2)
		{
			//PORTB.OUTCLR = (1<<1);
			port[1][1] = 0;
		}
	}
	else
	{
		if ((relay_cmd&0xF)==0)
		{
			// Idle state
			//PORTA.OUTCLR = (1<<7)|(1<<6); // RC1 RC4
			//PORTB.OUTCLR = (1<<0)|(1<<1); // RC2 RC3
			port[0][6] = 0;	// RC4
			port[0][7] = 0;	// RC1
			port[1][0] = 0;	// RC3
			port[1][1] = 0; // RC2
		}
			
		if (relay_cmd&RELAY_CMD_RIGHT1)
		{
			//PORTB.OUTSET = (1<<0)|(1<<1); // RC2 RC3
			port[1][0] = 63;	// RC3
			port[1][1] = 63;	// RC2
			RELAY_CHECK_DONE(RELAY_CMD_RIGHT1);
		}
		else if (relay_cmd&RELAY_CMD_LEFT1)
		{
			//PORTA.OUTSET = (1<<7)|(1<<6); // RC1 RC4
			port[0][6] = 63;	// RC4
			port[0][7] = 63;	// RC1
			RELAY_CHECK_DONE(RELAY_CMD_LEFT1);
		}
		else if (relay_cmd&RELAY_CMD_RIGHT2)
		{
			//PORTA.OUTSET = (1<<6);	// RC4
			//PORTB.OUTSET = (1<<1);	// RC3
			port[0][6] = 63;	// RC4
			port[1][1] = 63;	// RC3
			RELAY_CHECK_DONE(RELAY_CMD_RIGHT2);
		}
		else if (relay_cmd&RELAY_CMD_LEFT2)
		{
			//PORTA.OUTSET = (1<<7);	// RC1
			//PORTB.OUTSET = (1<<0);	// RC2
			port[0][7] = 63; // RC1
			port[1][0] = 63; // RC2
			RELAY_CHECK_DONE(RELAY_CMD_LEFT2);
		}
	}

	if ((relay_cmd&0xF0) == 0)
		relay_cmd = 0;
	else
		relay_cmd -= 0x10;
}

inline void servo_power_enable(void)
{
	//PORTA.OUTCLR = (1<<5);
	port[0][5] = 63;
}

inline void servo_power_disable(void)
{
	PORTA.OUTSET = (1<<5);
	port[0][5] = 0;
}

void pwm_step(void)
{
	uint8_t index;
	
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if ((pwm_update_trig&(1<<index))!=0)
		{
			pwm_port[index].dimm_delta = eeprom.pwm_delta[index];
			pwm_port[index].dimm_target = pwm_target[index];
			if (!(eeprom.port_config&(1<<PORT_MODE_PWM_UPDATE_CONT))) {
				pwm_update_trig &= ~(1<<index);			
			}
		}
				
		if (pwm_port[index].dimm_current==pwm_port[index].dimm_target)
		{
			pwm_at_setpoint |= (1<<index);
			continue;
		}
		else
		{
			pwm_at_setpoint &= ~(1<<index);
		}
		
		if (pwm_port[index].dimm_current>(pwm_port[index].dimm_target+pwm_port[index].dimm_delta))
		{
			pwm_port[index].dimm_current -= pwm_port[index].dimm_delta;
		}
		else if (pwm_port[index].dimm_current<(pwm_port[index].dimm_target-pwm_port[index].dimm_delta))
		{
			pwm_port[index].dimm_current += pwm_port[index].dimm_delta;
		}
		else
		{
			pwm_port[index].dimm_current = pwm_port[index].dimm_target;
		}
		
		if (pwm_port[index].dimm_current<DIMM_RANGE_MIN)
		{
			pwm_port[index].pwm_current = DIMM_RANGE_MIN;
		}
		else if (pwm_port[index].dimm_current>DIMM_RANGE_MAX)
		{
			pwm_port[index].pwm_current = DIMM_RANGE_MAX;
		}
		else
		{
			pwm_port[index].pwm_current = pwm_gamma_5b6b[pwm_port[index].dimm_current];
		}
	}
}

uint8_t port_map_get_bit(uint8_t index)
{
	if (index<16)
	{
		return port_do&(1<<index) ? 1 : 0;
	}
	else if (index<32)
	{
		return port_di&(1<<(index-16)) ? 1 : 0;
	}
	else if (index<48)
	{
		return port_do_mapped&(1<<(index-32)) ? 1 : 0;
	}
	else if (index<64)
	{
		return port_di_mapped&(1<<(index-48)) ? 1 : 0;
	}
	else if ((index>=128) && (index<136))
	{
		return port_user&(1<<(index-128)) ? 1 : 0;
	}
	
	return 0;
}

void port_do_mapping(void)
{
	uint8_t index;
	uint8_t lut_bit;
	uint16_t status;
	uint16_t status_mapped = 0;
		
	// Determine new input or output state
	for (index=0;index<16;index++)
	{
		if (eeprom.port_map_dir&(1<<index))
		{
			status = port_do;
		}
		else
		{
			status = port_di;
		}
				
		lut_bit = status&(1<<index) ? (1<<2) : 0;
		
		if (eeprom.port_map_mux1[index]<16)
		{
			lut_bit |= status&(1<<eeprom.port_map_mux1[index]) ? (1<<1) : 0;
		}
		else
		{
			lut_bit |= port_map_get_bit(eeprom.port_map_mux1[index])? (1<<1) : 0;
		}
		
		if (eeprom.port_map_mux2[index]<16)
		{
			lut_bit |= status&(1<<eeprom.port_map_mux2[index]) ? (1<<0) : 0;
		}
		else
		{
			lut_bit |= port_map_get_bit(eeprom.port_map_mux2[index])? (1<<0) : 0;
		}
				
		status_mapped |= (eeprom.port_map_lut[index]&(1<<lut_bit))? (1<<index) : 0;
	}
	
	// Assign inputs and outputs (brightness)
	for (index=0;index<16;index++)
	{
		if ((eeprom.port_map_dir&(1<<index))==0)
		{
			if (status_mapped&(1<<index))
			{
				port_di_mapped |= (1<<index);
			}
			else
			{
				port_di_mapped &= ~(1<<index);
			}
		}
		else
		{
			if (status_mapped&(1<<index))
			{
				port_do_mapped |= (1<<index);
				if (!(eeprom.port_pwm_enable&(1<<index)))
				{
					pwm_port[index].dimm_target = eeprom.port_brightness_on[index];
				}
			}
			else
			{
				port_do_mapped &= ~(1<<index);
			
				if (!(eeprom.port_pwm_enable&(1<<index)))
				{
					pwm_port[index].dimm_target = eeprom.port_brightness_off[index];
				}
			}
		}
	}	
		
	MAP_BITS(port_do_mapped, relay_request, 14, 0);
	MAP_BITS(port_do_mapped, relay_request, 15, 1);		
}

void pwm_init(void)
{
	uint8_t index;
	
	// Load settings from EEPROM
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		pwm_port[index].dimm_delta = eeprom.pwm_delta[index];
	}

	pwm_update_bitslices();
	
	PORTCFG.VPCTRLA = PORTCFG_VP02MAP_PORTA_gc | PORTCFG_VP13MAP_PORTB_gc;
	PORTCFG.VPCTRLB = PORTCFG_VP02MAP_PORTC_gc | PORTCFG_VP13MAP_PORTD_gc;
	
	EVSYS.CH3MUX = EVSYS_CHMUX_OFF_gc;
	
	// Enable Timer Type 2
	TCD2.CTRLE = TC2_BYTEM_SPLITMODE_gc;
	TCD2.LPER = PWM_STEPS-1;
	TCD2.HPER = PWM_STEPS-1;
	TCD2.CTRLB = 0; // Disable compare channels
	TCD2.CTRLA = TC2_CLKSEL_EVCH3_gc;
	
	TCD2.LCNT = 1;
	TCD2.HCNT = 0;
	
	TCD2.LCMPA = PWM_STEPS-2;
	TCD2.LCMPB = PWM_STEPS-2-4;
	TCD2.LCMPC = PWM_STEPS-2-4-8;
	TCD2.LCMPD = PWM_STEPS-2-4-8-16;

	// Clear all interrupt flags
	TCD2.INTFLAGS = 0xFF;
	
	// Enable interrupt on TCD2
	TCD2.INTCTRLA = TC2_HUNFINTLVL_LO_gc|TC2_LUNFINTLVL_LO_gc;
	TCD2.INTCTRLB = TC2_LCMPAINTLVL_LO_gc|TC2_LCMPBINTLVL_LO_gc|TC2_LCMPCINTLVL_LO_gc|TC2_LCMPDINTLVL_LO_gc;
	
	// Event CH3: 32 MHz / 2048 = 15625 Hz clock = 244 Hz PWM frequency
	EVSYS.CH3MUX = EVSYS_CHMUX_PRESCALER_2048_gc;
}

void pwm_target_parameter_update(void)
{
	uint8_t index;
	
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if (pwm_parameter_select&(1<<index))
		{
			pwm_port[index].dimm_target = pwm_target_temp;
		}
	}
}

void pwm_delta_parameter_update(void)
{
	uint8_t index;

	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if (pwm_parameter_select&(1<<index))
		{
			pwm_port[index].dimm_delta = pwm_delta_temp;
		}
	}
}

void pwm_update_bitslices(void)
{
	pwm_bitslice_update_needed = 0;
	// t <- 5 bit brightness
	// r <- bitslice, that needs to be stored after each for loop
	uint8_t *t; //[8];
	uint8_t r;
	uint8_t *p_bitslice;

	// PortA
	t = &(port[0][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][0]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	// PortB
	t = &(port[1][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][1]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	// PortC
	t = &(port[2][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][2]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	// PortD
	t = &(port[3][0]);
	p_bitslice = &(pwm_bitslice_shadow[0][3]);
	for( uint8_t bitmask = 0b00000001 ; bitmask < 0b01000000 ; bitmask <<= 1 )
	{
		r =  (t[0] & bitmask) ? 0b00000001 : 0 ;
		r |= (t[1] & bitmask) ? 0b00000010 : 0 ;
		r |= (t[2] & bitmask) ? 0b00000100 : 0 ;
		r |= (t[3] & bitmask) ? 0b00001000 : 0 ;
		r |= (t[4] & bitmask) ? 0b00010000 : 0 ;
		r |= (t[5] & bitmask) ? 0b00100000 : 0 ;
		r |= (t[6] & bitmask) ? 0b01000000 : 0 ;
		r |= (t[7] & bitmask) ? 0b10000000 : 0 ;
		*p_bitslice = r ;
		p_bitslice += 4 ;
	}
	
	pwm_bitslice_update_needed = 1;
}

#if 0
ISR(TCD0_CCA_vect)
{
	static uint8_t pwm_bitpos = 0;
	
	pwm_bitpos++;
	if (pwm_bitpos>5)
	{
		pwm_bitpos = 0;
		TCD0.CCA = 1;
	}
	else
	{
		TCD0.CCA <<= 1;
	}
	
	VPORT0.OUT = pwm_bitslice[pwm_bitpos][0];
	VPORT1.OUT = pwm_bitslice[pwm_bitpos][1];
	VPORT2.OUT = pwm_bitslice[pwm_bitpos][2];
	VPORT3.OUT = pwm_bitslice[pwm_bitpos][3];
	
}
#endif

ISR(TCD2_HUNF_vect)
{
#if 1
	//__asm__ __volatile ("push r24" "\n\t");
	VPORT0.OUT = pwm_bitslice[0][0];
	VPORT1.OUT = pwm_bitslice[0][1];
	VPORT2.OUT = pwm_bitslice[0][2];
	VPORT3.OUT = pwm_bitslice[0][3];
	//__asm__ __volatile ("pop r24" "\n\t");
	//__asm__ __volatile ("reti" "\n\t");
#else	
	__asm__ __volatile (
	"push r24" "\n\t"
	"lds r24, %[inreg1]" "\n\t"
	"out %[outreg1], r24" "\n\t"
	"lds r24, %[inreg2]" "\n\t"
	"out %[outreg2], r24" "\n\t"
	"lds r24, %[inreg3]" "\n\t"
	"out %[outreg3], r24" "\n\t"
	"lds r24, %[inreg4]" "\n\t"
	"out %[outreg4], r24" "\n\t"
	"pop r24" "\n\t"
	"reti" "\n\t"
	: /* output operands */
	
	: /* input operands */
	[inreg1]	""	(&pwm_bitslice[0][0]),
	[inreg2]	""	(&pwm_bitslice[0][1]),
	[inreg3]	""	(&pwm_bitslice[0][2]),
	[inreg4]	""	(&pwm_bitslice[0][3]),
	[outreg1]	"M" (_SFR_IO_ADDR (VPORT0.OUT)),
	[outreg2]	"M" (_SFR_IO_ADDR (VPORT1.OUT)),
	[outreg3]	"M" (_SFR_IO_ADDR (VPORT2.OUT)),
	[outreg4]	"M" (_SFR_IO_ADDR (VPORT3.OUT))
	
	/* no clobbers */
	);
#endif
}

ISR(TCD2_LUNF_vect)
{
	VPORT0.OUT = pwm_bitslice[1][0];
	VPORT1.OUT = pwm_bitslice[1][1];
	VPORT2.OUT = pwm_bitslice[1][2];
	VPORT3.OUT = pwm_bitslice[1][3];
}

ISR(TCD2_LCMPA_vect)
{
	VPORT0.OUT = pwm_bitslice[2][0];
	VPORT1.OUT = pwm_bitslice[2][1];
	VPORT2.OUT = pwm_bitslice[2][2];
	VPORT3.OUT = pwm_bitslice[2][3];	
}

ISR(TCD2_LCMPB_vect)
{
	VPORT0.OUT = pwm_bitslice[3][0];
	VPORT1.OUT = pwm_bitslice[3][1];
	VPORT2.OUT = pwm_bitslice[3][2];
	VPORT3.OUT = pwm_bitslice[3][3];
}

ISR(TCD2_LCMPC_vect)
{
	VPORT0.OUT = pwm_bitslice[4][0];
	VPORT1.OUT = pwm_bitslice[4][1];
	VPORT2.OUT = pwm_bitslice[4][2];
	VPORT3.OUT = pwm_bitslice[4][3];
}

ISR(TCD2_LCMPD_vect)
{
	VPORT0.OUT = pwm_bitslice[5][0];
	VPORT1.OUT = pwm_bitslice[5][1];
	VPORT2.OUT = pwm_bitslice[5][2];
	VPORT3.OUT = pwm_bitslice[5][3];
	
	if (pwm_bitslice_update_needed)
	{
		memcpy(pwm_bitslice, pwm_bitslice_shadow, sizeof(pwm_bitslice));
		pwm_bitslice_update_needed = 0;
	}
}

void port_update_mapping(void)
{
		port[2][2] = pwm_port[0].pwm_current;
		port[2][3] = pwm_port[1].pwm_current;
		port[2][7] = pwm_port[2].pwm_current;
		port[2][6] = pwm_port[3].pwm_current;
		port[3][5] = pwm_port[4].pwm_current;
		port[2][5] = pwm_port[5].pwm_current;
		port[3][1] = pwm_port[6].pwm_current;
		port[3][0] = pwm_port[7].pwm_current;
		port[2][0] = pwm_port[12].pwm_current;
		port[2][1] = pwm_port[13].pwm_current;
		
		if (!(eeprom.port_config&(1<<PORT_MODE_RELAY)))
		{
			port[0][6] = pwm_port[8].pwm_current;
			port[0][7] = pwm_port[9].pwm_current;
			port[1][0] = pwm_port[10].pwm_current;
			port[1][1] = pwm_port[11].pwm_current;
		}
}