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
#include "sys/process.h"
#include "sys/etimer.h"
#include "port.h"
#include "eeprom.h"

t_pwm_port pwm_port[PWM_PORT_COUNT];
int8_t pwm_target[PWM_PORT_COUNT];
uint8_t pwm_delta[PWM_PORT_COUNT];
uint16_t pwm_update_trig;
uint16_t pwm_update_cont;
uint16_t pwm_at_setpoint;

uint8_t port_mode;
uint16_t port_do;
uint16_t port_di;

uint8_t cur_dimm;

static struct etimer port_timer;

PROCESS(port_process, "Port IO Handling");

PROCESS_THREAD(port_process, ev, data)
{
	static uint8_t ct0, ct1;
	uint8_t i, in;
	
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

		relay_process();
		pwm_step();
		port_do_mapping();
		etimer_reset(&port_timer);
	}
	
	PROCESS_END();
}

void port_update_configuration(void)
{	
	port_mode = eeprom.port_config;
	port_di_init();
	pwm_init();
	relay_init();
	port_do_mapping_init();
}

void port_init(void)
{	
	port_mode = eeprom.port_config;
	
	// S_Power is configured regardless of mode
	PORTA.PIN5CTRL = PORT_INVEN_bm;
	PORTA.OUTSET = (1<<5);
	PORTA.DIRSET = (1<<5);

	port_di_init();
	pwm_init();
	relay_init();
}

void port_di_init(void)
{
	if (port_mode&(1<<PORT_MODE_PULLUP_ENABLE))
	{
		PORTC.PIN7CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN6CTRL = PORT_OPC_PULLUP_gc;
		PORTC.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN1CTRL = PORT_OPC_PULLUP_gc;
		PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
	}
	else
	{
		PORTC.PIN7CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN5CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTD.PIN0CTRL = PORT_OPC_TOTEM_gc;
	}
	
	// Initial key state
	PORT_PIN_STATUS(port_di);
}

uint8_t relay_cmd = 0;
uint8_t relay_state = 0;
uint8_t relay_request = 0;

void relay_init(void)
{
	if (port_mode&(1<<PORT_MODE_PWM_ENABLE))
	{
		return;	
	}
	
	relay_request = eeprom_status.relay_request;
	relay_state = !relay_request;

	PORTA.OUTCLR = (1<<7)|(1<<6); /* RC1 RC4 */
	PORTB.OUTCLR = (1<<0)|(1<<1); /* RC2 RC3 */
	PORTA.DIRSET = (1<<7)|(1<<6);
	PORTB.DIRSET = (1<<0)|(1<<1);

	if (port_mode&(1<<PORT_MODE_RELAY_MONOSTABLE))
	{
		PORTA.OUTSET |= (1<<6);
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
	PORTA.OUTCLR = (1<<7)|(1<<6); /* RC1 RC4 */ \
	PORTB.OUTCLR = (1<<0)|(1<<1); /* RC2 RC3 */\
} \

void relay_process(void)
{
	relay_governor();
		
	if (port_mode&(1<<PORT_MODE_RELAY_MONOSTABLE))
	{
		if (relay_cmd&RELAY_CMD_RIGHT1)
			PORTA.OUTSET = (1<<7);
		else if (relay_cmd&RELAY_CMD_LEFT1)
			PORTA.OUTCLR = (1<<7);
			
		if (relay_cmd&RELAY_CMD_RIGHT2)
			PORTB.OUTSET = (1<<1);
		else if (relay_cmd&RELAY_CMD_LEFT2)
			PORTB.OUTCLR = (1<<1);
	}
	else
	{
		if ((relay_cmd&0xF)==0)
		{
			// Idle state
			PORTA.OUTCLR = (1<<7)|(1<<6); // RC1 RC4
			PORTB.OUTCLR = (1<<0)|(1<<1); // RC2 RC3
		}
			
		if (relay_cmd&RELAY_CMD_RIGHT1)
		{
			PORTB.OUTSET = (1<<0)|(1<<1); // RC2 RC3
			RELAY_CHECK_DONE(RELAY_CMD_RIGHT1);
		}
		else if (relay_cmd&RELAY_CMD_LEFT1)
		{
			PORTA.OUTSET = (1<<7)|(1<<6); // RC1 RC4
			RELAY_CHECK_DONE(RELAY_CMD_LEFT1);
		}
		else if (relay_cmd&RELAY_CMD_RIGHT2)
		{
			PORTA.OUTSET = (1<<6);	// RC4
			PORTB.OUTSET = (1<<1);	// RC3
			RELAY_CHECK_DONE(RELAY_CMD_RIGHT2);
		}
		else if (relay_cmd&RELAY_CMD_LEFT2)
		{
			PORTA.OUTSET = (1<<7);	// RC1
			PORTB.OUTSET = (1<<0);	// RC2
			RELAY_CHECK_DONE(RELAY_CMD_LEFT2);
		}
	}

	if ((relay_cmd&0xF0) == 0)
		relay_cmd = 0;
	else
		relay_cmd -= 0x10;
}

void servo_power_enable(void)
{
	if (!(port_mode&(1<<PORT_MODE_PWM_CH7_ENABLE)))
	{
		PORTA.OUTCLR = (1<<5);
	}
}

void servo_power_disable(void)
{
	if (!(port_mode&(1<<PORT_MODE_PWM_CH7_ENABLE)))
	{
		PORTA.OUTSET = (1<<5);
	}
}

//==============================================================================
//
// Section 3
//
// Timing Engine
//
// Howto:    Timer1 (with prescaler 8 and 16 bit total count) triggers
//           an interrupt every PWM_PERIOD (=300us @8MHz);
//
//==============================================================================
//
// Section 3
//
// Lichtsteuerung als PWM-Dimmer
//
//             |                        |                          |
// Output:     |XXXXXXXXXX______________|XXXXXXXXXX________________|
//             |<-------->              |<------------------------>|
//             | dimm_val               |        PWM_PERIOD        |
//         ----|------------------------|--------------------------|---> Time
//
//
// 1. Einstellen des aktuellen Helligkeitswertes (do_dimm)
//
//    Es gibt 60 Helligkeitstufen.
//    Alle 300us erfolgt ein Interrupt, dieser schaltet den Dimmer um eine
//    Stufe weiter, nach 60 Stufen wird wieder von vorne begonnen.
//    Stufe MIN:  alle Ports mit einem dimm_val > DIMM_MIN werden eingeschaltet.
//    Stufe x:    Ein Port, dessen dimm_val kleiner x ist, wird abgeschaltet.
//    Stufe MAX:  Restart und Meldung an den DIMMER (C_Dimmstep)
//
//    Folge: Alle Ports mit einem dimm_val kleiner DIMM_RANGE_MIN sind
//    dauerhaft aus, alle Ports mit einem dimm_val gr��er DIMM_RANGE_MAX
//    sind dauerhaft ein.
//
//    Da der PWM 60 Stufen hat und alle 300us ein Int erfolgt, wird dieser
//    Durchlauf alle 18ms durchgef�hrt. Dies entspricht einer Refreshrate von
//    55Hz.
//
//
// 2. Langsame Ver�nderung der Helligkeit
//
//    Nach einem Zyklus des PWM wird vom Hauptprogramm der neue aktuelle
//    dimm_val ausgerechnet. Hierzu wird vom aktuellen Wert mit einem
//    Schritt "delta" nach oben oder unten gerechnet, bis der neue Zielwert
//    erreicht ist.
//
//    Die Zykluszeit der PWM ist 18ms, somit wird je nach "delta" folgende
//    Dimmzeit erreicht:
//
//      delta    |    Dimmzeit
//    -----------------------------
//        1      |    1080ms
//        2      |     540ms
//        3      |     360ms
//        4      |     270ms
//        5      |     216ms
//        6      |     180ms
//      100      |    sofort
//
//    Der neue Zielwert wird in light_val hinterlegt.
//
//    Wenn man von einem Wert kleiner DIMM_RANGE_MIN startet, dann wird
//    der Port erst mit Verz�gerung aufgedimmt, weil zuerst der Bereich
//    bis DIMM_RANGE_MIN "aufgedimmt" wird.
//    Dies wird dazu benutzt, zuerst das alte Signalbild wegzudimmen
//    und dann das neue Signalbild aufzudimmen.
//
// 3. Signalbilder
//
//    Signalbilder werden als Bitfeld hinterlegt. F�r jedes Kommando
//    gibt es ein Bitfeld, in dem das neue Signalbild abgelegt ist und eine
//    G�ltigkeitsmaske, diese bestimmt, auf welche Dimmwerte das Signalbild
//    wirken soll. Mit diesen beiden Pattern wird "set_new_light_val"
//    aufgerufen.
//
// 4. Blinken
//
//    Falls ontime bzw. offtime ungleich 0 sind, wird nach Ablauf der
//    jeweils andere Phasenwert geladen.
//



// This is the PWM Interrupt

void pwm_tick(void)
{
	uint8_t port;
	uint8_t mask;
	uint8_t port_mask = 0;

	//if (!(port_mode&(1<<PORT_MODE_PWM_ENABLE)))
	//	return;
	
	mask = 1;
	cur_dimm++;
	if (cur_dimm == DIMM_RANGE_MAX)
	{
		cur_dimm = DIMM_RANGE_MIN;
		for (port=0; port<PWM_PORT_COUNT; port++)
		{
			if ((uint8_t) (pwm_port[port].dimm_current > DIMM_RANGE_MIN)) port_mask |= mask;   // Einschalten wenn !0
			mask = mask << 1;
		}
		// Assign outputs
		if (port_mode&(1<<PORT_MODE_PWM_ENABLE))
		{
			PORTA.OUTSET = (port_mask&0b00000011)<<6;
			PORTB.OUTSET = (port_mask&0b00001100)>>2;
			PORTC.OUTCLR = (port_mask&0b00110000)>>4;
			if (port_mode&(1<<PORT_MODE_PWM_CH7_ENABLE))
				PORTA.OUTSET = (port_mask&0b01000000)>>1;
		}		
	}
	else
	{
		for (port=0; port<PWM_PORT_COUNT; port++)
		{
			if (cur_dimm >= pwm_port[port].dimm_current) port_mask |= mask;
			mask = mask << 1;
		}
		// Assign outputs
		if (port_mode&(1<<PORT_MODE_PWM_ENABLE))
		{
			PORTA.OUTCLR = (port_mask&0b00000011)<<6;
			PORTB.OUTCLR = (port_mask&0b00001100)>>2;
			PORTC.OUTSET = (port_mask&0b00110000)>>4;
			if (port_mode&(1<<PORT_MODE_PWM_CH7_ENABLE))
				PORTA.OUTCLR = (port_mask&0b01000000)>>1;
		}
	}
}

//---------------------------------------------------------------------------------
// dimmer()
// this routine is called from main(), if C_Dimmstep is activated
//

// Timing Engine
//
// Howto:
// 1. Generelles Timing:
//    Diese Routine wird alle PWM_PERIOD aufgerufen. Es wird folgendes
//    gepr�ft:
//    a) Wenn pwm_port[port].rest gleich 0: dann bleibt dieser Port unver�ndert.
//    b) pwm_port[port].rest wird decrementiert, wenn es dabei 0 wird, dann
//       wird ein Dimmvorgang in die andere Richtung eingeleitet.
//
// 2. Dimm-�berg�nge:
//    Je nach aktueller Richtung des Dimmvorgang (CurrentTarget) wird der aktuelle
//    Dimmwert erh�ht oder erniedrigt (z.Z. linear).
//    Die Dimmrampe ist unabh�ngig von den Zeiten, die bei ontime bzw. offtime
//    vorgegeben werden.
//    Wenn ein Ausgang ohne PWM durchschalten soll, dann mu� sein Delta sehr
//    gro� gew�hlt werden! (>60)



void pwm_step(void)
{
	uint8_t index;
	
	for (index=0;index<PWM_PORT_COUNT;index++)
	{
		if ((pwm_update_trig&(1<<index))!=0 || (pwm_update_cont&(1<<index))!=0)
		{
			pwm_update_trig &= ~(1<<index);
			pwm_port[index].dimm_delta = pwm_delta[index];
			pwm_port[index].dimm_target = pwm_target[index];
		}
		
		if (eeprom.port_do_select&(1<<index))
		{
			continue;
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
	}
}

void pwm_init(void)
{
	if (!(port_mode&(1<<PORT_MODE_PWM_ENABLE)))
	{	
		// Revert pins to inputs
		PORTA.DIRCLR = (1<<6)|(1<<7);
		PORTB.DIRCLR = (1<<1)|(1<<0);
		PORTC.DIRCLR = (1<<1)|(1<<0);
		
		PORTA.OUTCLR = (1<<6)|(1<<7);
		PORTB.OUTCLR = (1<<1)|(1<<0);
		PORTC.OUTSET = (1<<1)|(1<<0);
		
		PORTA.PIN6CTRL = PORT_OPC_TOTEM_gc;
		PORTA.PIN7CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN0CTRL = PORT_OPC_TOTEM_gc;
		PORTB.PIN1CTRL = PORT_OPC_TOTEM_gc;
		PORTC.PIN0CTRL = PORT_INVEN_bm;
		PORTC.PIN1CTRL = PORT_INVEN_bm;
		return;
	}
	
	cur_dimm = DIMM_RANGE_MIN;
	
	PORTA.PIN6CTRL = PORT_OPC_TOTEM_gc;
	PORTA.PIN7CTRL = PORT_OPC_TOTEM_gc;
	PORTB.PIN0CTRL = PORT_OPC_TOTEM_gc;
	PORTB.PIN1CTRL = PORT_OPC_TOTEM_gc;
	PORTC.PIN0CTRL = PORT_INVEN_bm;
	PORTC.PIN1CTRL = PORT_INVEN_bm;
	
	PORTA.OUTCLR = (1<<6)|(1<<7);
	PORTB.OUTCLR = (1<<1)|(1<<0);
	PORTC.OUTSET = (1<<1)|(1<<0);
	
	PORTA.DIRSET = (1<<6)|(1<<7);
	PORTB.DIRSET = (1<<1)|(1<<0);
	PORTC.DIRSET = (1<<1)|(1<<0);
	
	// Enable Timer Type 2
	TCC2.CTRLE = TC2_BYTEM_SPLITMODE_gc;
	TCC2.LPER = PWM_STEPS+1;
	TCC2.HPER = PWM_STEPS+1;
	TCC2.CTRLB = 0xFF; // Enable all compare channels
	TCC2.CTRLA = TC2_CLKSEL_EVCH3_gc;

	// Enable Timer Type 2
	TCD2.CTRLE = TC2_BYTEM_SPLITMODE_gc;
	TCD2.LPER = PWM_STEPS+1;
	TCD2.HPER = PWM_STEPS+1;
	TCD2.CTRLB = 0xFF; // Enable all compare channels
	TCD2.CTRLA = TC2_CLKSEL_EVCH3_gc;
	
	// Event CH3: 32 MHz / 4096 = 7812.5 Hz clock
	EVSYS.CH3MUX = EVSYS_CHMUX_PRESCALER_4096_gc;

	// Enable interrupt on TCC2
	TCD2.INTCTRLA = TC2_HUNFINTLVL_LO_gc;
}

// TCD2 Low-byte Timer Underflow Interrupt
// Frequency: CLKper / 4096 / PWM_STEPS = 120.2 Hz
ISR(TCD2_HUNF_vect)
{
	// Do housekeeping
	TCC2.LCMPA = pwm_port[4].dimm_current>DIMM_RANGE_MIN ? pwm_port[4].dimm_current>DIMM_RANGE_MAX ? PWM_STEPS+1 : pwm_port[4].dimm_current : 0;
	TCC2.LCMPB = pwm_port[6].dimm_current>DIMM_RANGE_MIN ? pwm_port[6].dimm_current - DIMM_RANGE_MIN : 0;
	TCC2.LCMPC = pwm_port[1].dimm_current>DIMM_RANGE_MIN ? pwm_port[1].dimm_current - DIMM_RANGE_MIN : 0;
	TCC2.LCMPD = pwm_port[3].dimm_current>DIMM_RANGE_MIN ? pwm_port[3].dimm_current>DIMM_RANGE_MAX ? PWM_STEPS+1 : pwm_port[3].dimm_current - DIMM_RANGE_MIN : 0;
	TCC2.HCMPA = pwm_port[5].dimm_current>DIMM_RANGE_MIN ? pwm_port[5].dimm_current - DIMM_RANGE_MIN : 0;
	TCC2.HCMPB = pwm_port[7].dimm_current>DIMM_RANGE_MIN ? pwm_port[7].dimm_current - DIMM_RANGE_MIN : 0;
	TCC2.HCMPC = pwm_port[8].dimm_current>DIMM_RANGE_MIN ? pwm_port[8].dimm_current - DIMM_RANGE_MIN : 0;
	TCC2.HCMPD = pwm_port[10].dimm_current>DIMM_RANGE_MIN ? pwm_port[10].dimm_current - DIMM_RANGE_MIN : 0;
	
	TCD2.LCMPA = pwm_port[12].dimm_current>DIMM_RANGE_MIN ? pwm_port[12].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.LCMPB = pwm_port[14].dimm_current>DIMM_RANGE_MIN ? pwm_port[14].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.LCMPC = pwm_port[9].dimm_current>DIMM_RANGE_MIN ? pwm_port[9].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.LCMPD = pwm_port[11].dimm_current>DIMM_RANGE_MIN ? pwm_port[11].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.HCMPA = pwm_port[13].dimm_current>DIMM_RANGE_MIN ? pwm_port[13].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.HCMPB = pwm_port[15].dimm_current>DIMM_RANGE_MIN ? pwm_port[15].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.HCMPC = pwm_port[0].dimm_current>DIMM_RANGE_MIN ? pwm_port[0].dimm_current - DIMM_RANGE_MIN : 0;
	TCD2.HCMPD = pwm_port[2].dimm_current>DIMM_RANGE_MIN ? pwm_port[2].dimm_current>DIMM_RANGE_MAX ? PWM_STEPS+1 : pwm_port[2].dimm_current - DIMM_RANGE_MIN : 0;
	
	// Disable interrupt
	TCD2.INTCTRLA = 0;
}


void port_do_mapping(void)
{
	uint8_t index;
	uint16_t port_state_on = 0;
	uint16_t port_state_off = 0;
	
	// Determine new output state
	for (index=0;index<16;index++)
	{
		if (port_do&(1<<index))
		{
			port_state_on |= eeprom.port_map_select_on[index];
			port_state_off |= eeprom.port_map_select_off[index];
		}
	}
	
	port_state_on &= ~port_state_off;
	
	// Assign brightness
	for (index=0;index<16;index++)
	{
		if ((eeprom.port_do_select&(1<<index))==0)
		{
			continue;
		}
		
		if ((port_state_on&(1<<index))==0)
		{
			pwm_port[index].dimm_current = 0;
			//pwm_port[index].dimm_target = 0;
			continue;
		}
		
		uint8_t idx_reg = 0;
		
		for (uint8_t idx=0;idx<sizeof(eeprom.port_brightness_select)/sizeof(eeprom.port_brightness_select[0]);idx++)
		{
			idx_reg |= (eeprom.port_brightness_select[idx]&(1<<index)) ? (1<<idx) : 0;
		}
		
		pwm_port[index].dimm_current = eeprom.port_brightness[idx_reg];
		//pwm_port[index].dimm_target = port_brightness[idx_reg];
	}
	
	// Trigger update by enabling interrupt of TCD2
	TCD2.INTFLAGS = TC2_HUNFIF_bm;
	TCD2.INTCTRLA = TC2_HUNFINTLVL_LO_gc;
}

void port_do_mapping_init(void)
{
		
}
