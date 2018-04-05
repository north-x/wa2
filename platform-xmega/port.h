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
SV(7, "Port Configuration Register", eeprom.port_config, port_update_configuration)
SV_LSB(8, "Port Direction Register L", eeprom.port_dir, port_update_configuration)
SV_MSB(9, "Port Direction Register H", eeprom.port_dir, port_update_configuration)
SV_LSB(10, "Port Map Lut 1 L", eeprom.port_map_lut[0], 0)
SV_MSB(11, "Port Map Lut 1 H", eeprom.port_map_lut[0], 0)
SV_LSB(12, "Port Map Lut 2 L", eeprom.port_map_lut[1], 0)
SV_MSB(13, "Port Map Lut 2 H", eeprom.port_map_lut[1], 0)
SV_LSB(14, "Port Map Lut 3 L", eeprom.port_map_lut[2], 0)
SV_MSB(15, "Port Map Lut 3 H", eeprom.port_map_lut[2], 0)
SV_LSB(16, "Port Map Lut 4 L", eeprom.port_map_lut[3], 0)
SV_MSB(17, "Port Map Lut 4 H", eeprom.port_map_lut[3], 0)
SV_LSB(18, "Port Map Lut 5 L", eeprom.port_map_lut[4], 0)
SV_MSB(19, "Port Map Lut 5 H", eeprom.port_map_lut[4], 0)
SV_LSB(20, "Port Map Lut 6 L", eeprom.port_map_lut[5], 0)
SV_MSB(21, "Port Map Lut 6 H", eeprom.port_map_lut[5], 0)
SV_LSB(22, "Port Map Lut 7 L", eeprom.port_map_lut[6], 0)
SV_MSB(23, "Port Map Lut 7 H", eeprom.port_map_lut[6], 0)
SV_LSB(24, "Port Map Lut 8 L", eeprom.port_map_lut[7], 0)
SV_MSB(25, "Port Map Lut 8 H", eeprom.port_map_lut[7], 0)
SV_LSB(26, "Port Map Lut 9 L", eeprom.port_map_lut[8], 0)
SV_MSB(27, "Port Map Lut 9 H", eeprom.port_map_lut[8], 0)
SV_LSB(28, "Port Map Lut 10 L", eeprom.port_map_lut[9], 0)
SV_MSB(29, "Port Map Lut 10 H", eeprom.port_map_lut[9], 0)
SV_LSB(30, "Port Map Lut 11 L", eeprom.port_map_lut[10], 0)
SV_MSB(31, "Port Map Lut 11 H", eeprom.port_map_lut[10], 0)
SV_LSB(32, "Port Map Lut 12 L", eeprom.port_map_lut[11], 0)
SV_MSB(33, "Port Map Lut 12 H", eeprom.port_map_lut[11], 0)
SV_LSB(34, "Port Map Lut 13 L", eeprom.port_map_lut[12], 0)
SV_MSB(35, "Port Map Lut 13 H", eeprom.port_map_lut[12], 0)
SV_LSB(36, "Port Map Lut 14 L", eeprom.port_map_lut[13], 0)
SV_MSB(37, "Port Map Lut 14 H", eeprom.port_map_lut[13], 0)
SV_LSB(38, "Port Map Lut 15 L", eeprom.port_map_lut[14], 0)
SV_MSB(39, "Port Map Lut 15 H", eeprom.port_map_lut[14], 0)
SV_LSB(40, "Port Map Lut 16 L", eeprom.port_map_lut[15], 0)
SV_MSB(41, "Port Map Lut 16 H", eeprom.port_map_lut[15], 0)
SV(42, "Port Map Mux1 1", eeprom.port_map_mux1[0], 0)
SV(43, "Port Map Mux1 2", eeprom.port_map_mux1[1], 0)
SV(44, "Port Map Mux1 3", eeprom.port_map_mux1[2], 0)
SV(45, "Port Map Mux1 4", eeprom.port_map_mux1[3], 0)
SV(46, "Port Map Mux1 5", eeprom.port_map_mux1[4], 0)
SV(47, "Port Map Mux1 6", eeprom.port_map_mux1[5], 0)
SV(48, "Port Map Mux1 7", eeprom.port_map_mux1[6], 0)
SV(49, "Port Map Mux1 8", eeprom.port_map_mux1[7], 0)
SV(50, "Port Map Mux1 9", eeprom.port_map_mux1[8], 0)
SV(51, "Port Map Mux1 10", eeprom.port_map_mux1[9], 0)
SV(52, "Port Map Mux1 11", eeprom.port_map_mux1[10], 0)
SV(53, "Port Map Mux1 12", eeprom.port_map_mux1[11], 0)
SV(54, "Port Map Mux1 13", eeprom.port_map_mux1[12], 0)
SV(55, "Port Map Mux1 14", eeprom.port_map_mux1[13], 0)
SV(56, "Port Map Mux1 15", eeprom.port_map_mux1[14], 0)
SV(57, "Port Map Mux1 16", eeprom.port_map_mux1[15], 0)
SV(58, "Port Map Mux2 1", eeprom.port_map_mux2[0], 0)
SV(59, "Port Map Mux2 2", eeprom.port_map_mux2[1], 0)
SV(60, "Port Map Mux2 3", eeprom.port_map_mux2[2], 0)
SV(61, "Port Map Mux2 4", eeprom.port_map_mux2[3], 0)
SV(62, "Port Map Mux2 5", eeprom.port_map_mux2[4], 0)
SV(63, "Port Map Mux2 6", eeprom.port_map_mux2[5], 0)
SV(64, "Port Map Mux2 7", eeprom.port_map_mux2[6], 0)
SV(65, "Port Map Mux2 8", eeprom.port_map_mux2[7], 0)
SV(66, "Port Map Mux2 9", eeprom.port_map_mux2[8], 0)
SV(67, "Port Map Mux2 10", eeprom.port_map_mux2[9], 0)
SV(68, "Port Map Mux2 11", eeprom.port_map_mux2[10], 0)
SV(69, "Port Map Mux2 12", eeprom.port_map_mux2[11], 0)
SV(70, "Port Map Mux2 13", eeprom.port_map_mux2[12], 0)
SV(71, "Port Map Mux2 14", eeprom.port_map_mux2[13], 0)
SV(72, "Port Map Mux2 15", eeprom.port_map_mux2[14], 0)
SV(73, "Port Map Mux2 16", eeprom.port_map_mux2[15], 0)
SV_LSB(74, "Port Brightness Select1 L", eeprom.port_brightness_select[0], 0)
SV_MSB(75, "Port Brightness Select1 H", eeprom.port_brightness_select[0], 0)
SV_LSB(76, "Port Brightness Select2 L", eeprom.port_brightness_select[1], 0)
SV_MSB(77, "Port Brightness Select2 H", eeprom.port_brightness_select[1], 0)
SV(78, "Port Brightness 1", eeprom.port_brightness[0], 0)
SV(79, "Port Brightness 2", eeprom.port_brightness[1], 0)
SV(80, "Port Brightness 3", eeprom.port_brightness[2], 0)
SV(81, "Port Brightness 4", eeprom.port_brightness[3], 0)
SV(82, "PWM Dimm Delta 1", eeprom.pwm_delta[0], 0)
SV(83, "PWM Dimm Delta 2", eeprom.pwm_delta[1], 0)
SV(84, "PWM Dimm Delta 3", eeprom.pwm_delta[2], 0)
SV(85, "PWM Dimm Delta 4", eeprom.pwm_delta[3], 0)
SV(86, "PWM Dimm Delta 5", eeprom.pwm_delta[4], 0)
SV(87, "PWM Dimm Delta 6", eeprom.pwm_delta[5], 0)
SV(88, "PWM Dimm Delta 7", eeprom.pwm_delta[6], 0)
SV(89, "PWM Dimm Delta 8", eeprom.pwm_delta[7], 0)
SV(90, "PWM Dimm Delta 9", eeprom.pwm_delta[8], 0)
SV(91, "PWM Dimm Delta 10", eeprom.pwm_delta[9], 0)
SV(92, "PWM Dimm Delta 11", eeprom.pwm_delta[10], 0)
SV(93, "PWM Dimm Delta 12", eeprom.pwm_delta[11], 0)
SV(94, "PWM Dimm Delta 13", eeprom.pwm_delta[12], 0)
SV(95, "PWM Dimm Delta 14", eeprom.pwm_delta[13], 0)
SV(96, "Port User Command", port_user, 0)
SV_LSB(97, "Port Input Status L", port_di, 0)
SV_MSB(98, "Port Input Status H", port_di, 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t port_config;
uint16_t port_dir;
uint16_t port_map_dir;
uint8_t port_map_lut[16];
uint8_t port_map_mux1[16];
uint8_t port_map_mux2[16];
uint16_t port_brightness_select[2];
int8_t port_brightness[4];
uint8_t pwm_delta[14];
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
.pwm_delta = {PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1, PWM_STEPS+1},
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
#define PORT_MODE_PWM_UPDATE_CONT	7

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

void pwm_target_parameter_update(void);
void pwm_delta_parameter_update(void);

extern uint16_t port_do;
extern uint16_t port_di;
extern uint8_t port_user;

extern uint8_t port_mode;

extern t_pwm_port pwm_port[PWM_PORT_COUNT];
extern int8_t pwm_target[PWM_PORT_COUNT];
extern uint8_t pwm_delta[PWM_PORT_COUNT];
extern uint16_t pwm_update_trig;
extern uint16_t pwm_at_setpoint;
extern uint8_t pwm_target_temp;
extern uint8_t pwm_delta_temp;
extern uint16_t pwm_parameter_select;

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