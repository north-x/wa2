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
&servo_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(7, "Servo Configuration Register", eeprom.servo_config, servo_mode_update)
SV_MSB(20, "Servo 1 Minimum H", eeprom.servo_min[0], servo_update_configuration)
SV_MSB(21, "Servo 1 Maximum H", eeprom.servo_max[0], servo_update_configuration)
SV(22, "Servo 1 Speed", eeprom.servo_time_ratio[0], servo_update_configuration)
SV_LSB(23, "Servo 1 Minimum L", eeprom.servo_min[0], servo_update_configuration)
SV_LSB(24, "Servo 1 Maximum L", eeprom.servo_max[0], servo_update_configuration)
SV_MSB(25, "Servo 2 Minimum H", eeprom.servo_min[1], servo_update_configuration)
SV_MSB(26, "Servo 2 Maximum H", eeprom.servo_max[1], servo_update_configuration)
SV(27, "Servo 2 Speed", eeprom.servo_time_ratio[1], servo_update_configuration)
SV_LSB(28, "Servo 2 Minimum L", eeprom.servo_min[1], servo_update_configuration)
SV_LSB(29, "Servo 2 Maximum L", eeprom.servo_max[1], servo_update_configuration)
SV_LSB(30, "Standby Delay L", eeprom.servo_timeout, 0)
SV_MSB(31, "Standby Delay H", eeprom.servo_timeout, 0)
SV_LSB(32, "Startup Delay L", eeprom.servo_startup_delay, 0)
SV_MSB(33, "Startup Delay H", eeprom.servo_startup_delay, 0)
SV(34, "Servo Start Method", eeprom.servo_start_method, servo_mode_update)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t servo_config;
uint16_t servo_startup_delay;
uint16_t servo_timeout;
uint8_t servo_start_method;
uint16_t servo_min[2];
uint16_t servo_max[2];
uint8_t servo_time_ratio[2];
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
uint8_t servo_position[2];
#endif

/*
 *	EEPROM Confiuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.servo_config = (1<<SERVO_STATUS_ENABLE_PWM_A),
.servo_startup_delay = 80,
.servo_timeout = 0,
.servo_start_method = 1,
.servo_min = {32767, 32767},
.servo_max = {32768, 32768},
.servo_time_ratio = {16, 16},
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
.servo_position = {127, 127},
#endif

#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef __SERVO_HP_H
#define __SERVO_HP_H

#define SERVO_COUNT          2         // number of Servos 

// Timing Definition

// TCC0, PORTC
// S1/S2 need to be inverted
// S12/S22 are direct feedthrough
#define S2_PWM				0
#define S1_PWM				1
#define S12_PWM				2
#define S22_PWM				3

// Timing Borders for Servo Pulse
#define SERVO_MINTIME      1000L    // 1ms
#define SERVO_INTERVAL     1000L    // 1ms

// defines for the Timer and the OCR
// note: only valid for "rounded" F_CPU values
#define T1_PRESCALER		16

#define SERVO_MIN          (F_CPU / 1000000L * SERVO_MINTIME / T1_PRESCALER)
#define SERVO_DELTA        (F_CPU / 1000000L * SERVO_INTERVAL / T1_PRESCALER)
#define SERVO_MAX          (F_CPU / 1000000L * (SERVO_MINTIME+SERVO_INTERVAL) / T1_PRESCALER)

// bit field for servo.status:
//#define ST_BIT_POSITION		0		// Last valid position: 0: A/L , 1: B/R
#define ST_BIT_MOVING    7          // 0=stopped, 1=moving
#define ST_BIT_AT_SETPOINT	6
//#define ST_BIT_TRANSMIT_STATUS	5
//#define ST_BIT_MODE_MULTIPOSITION	4

// bit field for servo.command
//#define SC_BIT_POSITION	0
#define SC_BIT_MOVING	7

#define SERVO_STATUS_PWR                7
#define SERVO_STATUS_PWR_ALWAYS_ON      6
#define SERVO_STATUS_PWR_TEST			5
#define SERVO_STATUS_INT_FLAG			4
#define SERVO_STATUS_PWM_DISABLE		3
#define SERVO_STATUS_TICK_FLAG			2
#define SERVO_STATUS_ENABLE_PWM_B		1
#define SERVO_STATUS_ENABLE_PWM_A		0

typedef struct SERVO_T
{ 
	uint16_t pulse_min;         // lowest pulse width [0..65535]
    uint16_t pulse_delta;       // range for pulse width [0..65535]
    uint16_t min;               // lower move limit [0..65535]
    uint16_t max;               // upper move limit [0..65535]
    
    uint8_t status;          // Bit 0: ACTUAL:   0=pre A, 1=pre B movement
                                    // Bit 1: MOVING:   0=at endpoint, 1=currently moving
                                    // Bit 5: OUT_CTRL: 0=no, 1=yes
                                    // Bit 6: REPEAT:   0=single, 1=forever
                                    // Bit 7: TERMINATE: flag: terminate after next B
	uint8_t command;								
      
    uint16_t active_time;       // runtime: relative time to start point
                                    // 0xFFFF   = restart Servos
                                    // 0   		= finished

    uint8_t time_ratio;       	// ratio between runtime and curve time
	
	uint8_t time_delta;
	uint8_t position_setpoint;
	uint8_t position_start;
	uint8_t position_actual;
	uint16_t pulse_value;
} t_servo;

void servo_update_configuration(void);
void servo_mode_update(void);

extern t_servo servo[SERVO_COUNT];
extern volatile uint8_t servo_status;
extern volatile uint16_t servo_timer;
extern volatile uint8_t servo_timer2;

void servo_isr(void);
void servo_init(void);

PROCESS_NAME(servo_process);

#endif

#endif 