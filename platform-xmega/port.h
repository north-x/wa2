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

/************************************************************************/
/* Module Configuration                                                 */
/************************************************************************/
#ifdef CONFIGURATION

/*
 *	Autostart List
 *
 *	List of all processes that are started automatically on startup
 *
 */
#ifdef AUTOSTART_CFG
&port_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(6, "Port Configuration Register", eeprom.port_config, port_update_configuration)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t port_config;
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
uint8_t relay_request;
#endif

/*
 *	EEPROM Confiuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.port_config = (1<<PORT_MODE_PULLUP_ENABLE),
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
.relay_request = 0,
#endif

#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef PORT_H_
#define PORT_H_

#define PORT_PIN0	(PORTC.IN&(1<<7)) // SIG_IS1_RX	PC7
#define PORT_PIN1	(PORTC.IN&(1<<6)) // SIG_IS2_RX	PC6
#define PORT_PIN2	(PORTD.IN&(1<<5)) // SIG_IS1_TX	PD5
#define PORT_PIN3	(PORTC.IN&(1<<5)) // SIG_IS2_TX	PC5
#define PORT_PIN4	(PORTD.IN&(1<<1)) // SIG_IS1_O	PD1
#define PORT_PIN5	(PORTD.IN&(1<<0)) // SIG_IS2_O	PD0
#define PORT_PIN6	// SIG_S12_PWM	PC2
#define PORT_PIN7	// SIG_S22_PWM	PC3

#define PORT_PIN_STATUS(VAR)	do { \
	uint8_t temp8;\
	temp8 = PORT_PIN0?1:0; \
	temp8 |= PORT_PIN1?2:0; \
	temp8 |= PORT_PIN2?4:0; \
	temp8 |= PORT_PIN3?8:0; \
	temp8 |= PORT_PIN4?16:0; \
	temp8 |= PORT_PIN5?32:0; \
	temp8 |= (ACA.STATUS&AC_AC0STATE_bm)?64:0; \
	VAR = temp8; \
} while (0);


#define PORT_FLAGS_PWM_TICK		0

#define PORT_MODE_PWM_ENABLE		0
#define PORT_MODE_PWM_CH7_ENABLE	1
#define PORT_MODE_PULLUP_ENABLE		2
#define PORT_MODE_RELAY_MONOSTABLE	3

#define PWM_PORT_COUNT	7

#define PWM_TIMER_PRESCALER	8
#define PWM_TICK_PERIOD     250L        // 250us tick for PWM Engine
#define PWM_STEPS           60L
#define PWM_PERIOD          (PWM_TICK_PERIOD * PWM_STEPS)    // 15ms = 250 * 60L
#define DIMM_RANGE_MIN		100    // aktiver Bereich 100-160
#define DIMM_RANGE_MAX (DIMM_RANGE_MIN+PWM_STEPS+1)

#define PWM_PERVAL   (F_CPU / 1000000L * PWM_TICK_PERIOD / PWM_TIMER_PRESCALER)

#define RELAY_CMD_LEFT1		1
#define RELAY_CMD_RIGHT1	2
#define RELAY_CMD_LEFT2		4
#define RELAY_CMD_RIGHT2	8
#define RELAY_CMD_LEFT		1
#define RELAY_CMD_RIGHT		2

typedef struct t_pwm_port
{
	uint8_t dimm_current;
	uint8_t dimm_target;
	uint8_t dimm_delta;
} t_pwm_port;

void port_init(void);
void port_update_configuration(void);
void port_di_init(void);

void pwm_init(void);
void pwm_tick(void);
void pwm_step(void);

extern uint8_t port_do_select;
extern uint8_t port_do;
extern uint8_t port_di;

extern uint8_t port_mode;

extern t_pwm_port pwm_port[PWM_PORT_COUNT];
extern uint8_t pwm_target[PWM_PORT_COUNT];
extern uint8_t pwm_delta[PWM_PORT_COUNT];
extern uint16_t pwm_update_trig;
extern uint16_t pwm_update_cont;
extern uint16_t pwm_at_setpoint;

extern uint8_t relay_cmd;
extern uint8_t relay_request;
extern uint8_t relay_state;

void relay_init(void);
void relay_governor(void);
void relay_process(void);

void servo_power_enable(void);
void servo_power_disable(void);

PROCESS_NAME(port_process);

#endif /* PORT_H_ */
#endif