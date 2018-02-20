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
SV_LSB(13, "Port Direction Register L", eeprom.port_dir, port_update_configuration)
SV_MSB(14, "Port Direction Register H", eeprom.port_dir, port_update_configuration)
SV(15, "Port Digital Output Cmd", port_user, 0)
SV(16, "Port Digital Input Status", port_di, 0)
SV_LSB(17, "Port Map Direction Register L", eeprom.port_map_dir, 0)
SV_MSB(18, "Port Map Direction Register L", eeprom.port_map_dir, 0)
SV(19, "Port Brightness 0", eeprom.port_brightness_select[0], 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t port_config;
uint16_t port_dir;
uint16_t port_map_dir;
uint16_t port_map_lut[16];
uint8_t port_map_mux1[16];
uint8_t port_map_mux2[16];
uint16_t port_brightness_select[2];
int8_t port_brightness[4];
uint8_t pwm_dimm_delta[14];
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
.port_config = (1<<PORT_MODE_PULLUP_ENABLE)|(1<<PORT_MODE_RELAY)|(1<<PORT_MODE_PWM_ENABLE)|(1<<PORT_MODE_PWM1_ENABLE),
.port_dir = (1<<0)|(1<<1),
.port_map_dir = ((1<<14)|(1<<15)),
.port_map_lut = {240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 204, 204},
.port_map_mux1 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 129},
.port_map_mux2 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
.port_brightness_select = {0, 0},
.port_brightness = {DIMM_RANGE_MAX, DIMM_RANGE_MIN+PWM_STEPS/4, DIMM_RANGE_MIN+PWM_STEPS/2, DIMM_RANGE_MIN+3*PWM_STEPS/4 },
.pwm_dimm_delta = {PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1},
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

#define MAP_BITS(SRC_REG, DEST_REG, SRC_BIT, DEST_BIT) if (SRC_REG&(1<<(SRC_BIT))) DEST_REG |= (1<<(DEST_BIT)); else DEST_REG &= ~(1<<(DEST_BIT))

#define PORT_PIN0	(PORTC.IN&(1<<2)) // SIG_S12_PWMPC2
#define PORT_PIN1	(PORTC.IN&(1<<3)) // SIG_S22_PWMPC3
#define PORT_PIN2	(PORTC.IN&(1<<7)) // SIG_IS1_RX	PC7
#define PORT_PIN3	(PORTC.IN&(1<<6)) // SIG_IS2_RX	PC6
#define PORT_PIN4	(PORTD.IN&(1<<5)) // SIG_IS1_TX	PD5
#define PORT_PIN5	(PORTC.IN&(1<<5)) // SIG_IS2_TX	PC5
#define PORT_PIN6	(PORTD.IN&(1<<1)) // SIG_IS1_O	PD1
#define PORT_PIN7	(PORTD.IN&(1<<0)) // SIG_IS2_O	PD0
#define PORT_PIN8	(PORTA.IN&(1<<6)) // S1L - PA6
#define PORT_PIN9	(PORTA.IN&(1<<7)) // S1R - PA7
#define PORT_PIN10	(PORTB.IN&(1<<0)) // S1H - PB0
#define PORT_PIN11	(PORTB.IN&(1<<1)) // S2L - PB1
#define PORT_PIN12	(PORTC.IN&(1<<0)) // S2R - PC0
#define PORT_PIN13	(PORTC.IN&(1<<1)) // S2H - PC1

#define PORT_PIN_STATUS(VAR)	do { \
	uint16_t temp16;\
	temp16 = PORT_PIN0?1:0; \
	temp16 |= PORT_PIN1?2:0; \
	temp16 |= PORT_PIN2?4:0; \
	temp16 |= PORT_PIN3?8:0; \
	temp16 |= PORT_PIN4?16:0; \
	temp16 |= PORT_PIN5?32:0; \
	temp16 |= PORT_PIN6?64:0; \
	temp16 |= PORT_PIN7?128:0; \
	temp16 |= PORT_PIN8?256:0; \
	temp16 |= PORT_PIN9?512:0; \
	temp16 |= PORT_PIN10?1024:0; \
	temp16 |= PORT_PIN11?2048:0; \
	temp16 |= PORT_PIN12?4096:0; \
	temp16 |= PORT_PIN13?8192:0; \
	VAR = temp16; \
} while (0);


#define PORT_FLAGS_PWM_TICK		0

#define PORT_MODE_PWM_ENABLE		0
#define PORT_MODE_PWM_CH7_ENABLE	1
#define PORT_MODE_PULLUP_ENABLE		2
#define PORT_MODE_RELAY_MONOSTABLE	3
#define PORT_MODE_RELAY				4
#define PORT_MODE_PWM1_ENABLE		5
#define PORT_MODE_PWM2_ENABLE		6

#define PWM_PORT_COUNT	14

#define PWM_TIMER_PRESCALER	8
#define PWM_TICK_PERIOD     250L        // 250us tick for PWM Engine
#define PWM_STEPS           63L
#define PWM_PERIOD          (PWM_TICK_PERIOD * PWM_STEPS)    // 15ms = 250 * 60L
#define DIMM_RANGE_MIN		0    // aktiver Bereich 0-63
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
	int8_t dimm_current;
	int8_t dimm_target;
	uint8_t dimm_delta;
} t_pwm_port;

typedef struct t_function_mapping
{
	uint16_t select_on;
	uint16_t select_off;
} t_function_mapping;

void port_init(void);
void port_update_configuration(void);
void port_di_init(void);

void port_do_mapping(void);

void pwm_init(void);
void pwm_tick(void);
void pwm_step(void);

extern uint16_t port_do;
extern uint16_t port_di;
extern uint8_t port_user;

extern uint8_t port_mode;

extern t_pwm_port pwm_port[PWM_PORT_COUNT];
extern int8_t pwm_target[PWM_PORT_COUNT];
extern uint8_t pwm_delta[PWM_PORT_COUNT];
extern uint16_t pwm_update_trig;
extern uint16_t pwm_update_cont;
extern uint16_t pwm_at_setpoint;

extern t_function_mapping port_map[];
extern uint16_t port_brightness_select[];
extern int8_t port_brightness[];

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