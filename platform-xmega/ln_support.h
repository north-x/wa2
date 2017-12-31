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
&ln_process,
#endif

/*
 *	SV Configuration Table
 *
 *	List of all configuration variables defined by this module
 *
 */
#ifdef SV_CFG
SV(8, "WDT Reset Counter", ln_wdt_counter, 0)
SV(9, "Rx Error Counter", LnBuffer.Stats.RxErrors, 0)
SV(10, "LN GPIO Status", ln_gpio_status[0], 0)
SV(11, "LN GPIO Status Transmit", ln_gpio_tx[0], 0)
SV(12, "LN Threshold Voltage x10", eeprom.ln_threshold, ln_update_threshold)

SV(41, "LN GPIO 1 On Opcode 1", eeprom.ln_gpio_opcode[0][0], 0)
SV(42, "LN GPIO 1 On Opcode 2", eeprom.ln_gpio_opcode[0][1], 0)
SV(43, "LN GPIO 1 On Opcode 3", eeprom.ln_gpio_opcode[0][2], 0)
SV(44, "LN GPIO 1 Off Opcode 1", eeprom.ln_gpio_opcode[1][0], 0)
SV(45, "LN GPIO 1 Off Opcode 2", eeprom.ln_gpio_opcode[1][1], 0)
SV(46, "LN GPIO 1 Off Opcode 3", eeprom.ln_gpio_opcode[1][2], 0)
SV(47, "LN GPIO 2 On Opcode 1", eeprom.ln_gpio_opcode[2][0], 0)
SV(48, "LN GPIO 2 On Opcode 2", eeprom.ln_gpio_opcode[2][1], 0)
SV(49, "LN GPIO 2 On Opcode 3", eeprom.ln_gpio_opcode[2][2], 0)
SV(50, "LN GPIO 2 Off Opcode 1", eeprom.ln_gpio_opcode[3][0], 0)
SV(51, "LN GPIO 2 Off Opcode 2", eeprom.ln_gpio_opcode[3][1], 0)
SV(52, "LN GPIO 2 Off Opcode 3", eeprom.ln_gpio_opcode[3][2], 0)
SV(53, "LN GPIO 3 On Opcode 1", eeprom.ln_gpio_opcode[4][0], 0)
SV(54, "LN GPIO 3 On Opcode 2", eeprom.ln_gpio_opcode[4][1], 0)
SV(55, "LN GPIO 3 On Opcode 3", eeprom.ln_gpio_opcode[4][2], 0)
SV(56, "LN GPIO 3 Off Opcode 1", eeprom.ln_gpio_opcode[5][0], 0)
SV(57, "LN GPIO 3 Off Opcode 2", eeprom.ln_gpio_opcode[5][1], 0)
SV(58, "LN GPIO 3 Off Opcode 3", eeprom.ln_gpio_opcode[5][2], 0)
SV(59, "LN GPIO 4 On Opcode 1", eeprom.ln_gpio_opcode[6][0], 0)
SV(60, "LN GPIO 4 On Opcode 2", eeprom.ln_gpio_opcode[6][1], 0)
SV(61, "LN GPIO 4 On Opcode 3", eeprom.ln_gpio_opcode[6][2], 0)
SV(62, "LN GPIO 4 Off Opcode 1", eeprom.ln_gpio_opcode[7][0], 0)
SV(63, "LN GPIO 4 Off Opcode 2", eeprom.ln_gpio_opcode[7][1], 0)
SV(64, "LN GPIO 4 Off Opcode 3", eeprom.ln_gpio_opcode[7][2], 0)
SV(65, "LN GPIO 5 On Opcode 1", eeprom.ln_gpio_opcode[8][0], 0)
SV(66, "LN GPIO 5 On Opcode 2", eeprom.ln_gpio_opcode[8][1], 0)
SV(67, "LN GPIO 5 On Opcode 3", eeprom.ln_gpio_opcode[8][2], 0)
SV(68, "LN GPIO 5 Off Opcode 1", eeprom.ln_gpio_opcode[9][0], 0)
SV(69, "LN GPIO 5 Off Opcode 2", eeprom.ln_gpio_opcode[9][1], 0)
SV(70, "LN GPIO 5 Off Opcode 3", eeprom.ln_gpio_opcode[9][2], 0)
SV(71, "LN GPIO 6 On Opcode 1", eeprom.ln_gpio_opcode[10][0], 0)
SV(72, "LN GPIO 6 On Opcode 2", eeprom.ln_gpio_opcode[10][1], 0)
SV(73, "LN GPIO 6 On Opcode 3", eeprom.ln_gpio_opcode[10][2], 0)
SV(74, "LN GPIO 6 Off Opcode 1", eeprom.ln_gpio_opcode[11][0], 0)
SV(75, "LN GPIO 6 Off Opcode 2", eeprom.ln_gpio_opcode[11][1], 0)
SV(76, "LN GPIO 6 Off Opcode 3", eeprom.ln_gpio_opcode[11][2], 0)
SV(77, "LN GPIO 7 On Opcode 1", eeprom.ln_gpio_opcode[12][0], 0)
SV(78, "LN GPIO 7 On Opcode 2", eeprom.ln_gpio_opcode[12][1], 0)
SV(79, "LN GPIO 7 On Opcode 3", eeprom.ln_gpio_opcode[12][2], 0)
SV(80, "LN GPIO 7 Off Opcode 1", eeprom.ln_gpio_opcode[13][0], 0)
SV(81, "LN GPIO 7 Off Opcode 2", eeprom.ln_gpio_opcode[13][1], 0)
SV(82, "LN GPIO 7 Off Opcode 3", eeprom.ln_gpio_opcode[13][2], 0)
SV(83, "LN GPIO 8 On Opcode 1", eeprom.ln_gpio_opcode[14][0], 0)
SV(84, "LN GPIO 8 On Opcode 2", eeprom.ln_gpio_opcode[14][1], 0)
SV(85, "LN GPIO 8 On Opcode 3", eeprom.ln_gpio_opcode[14][2], 0)
SV(86, "LN GPIO 8 Off Opcode 1", eeprom.ln_gpio_opcode[15][0], 0)
SV(87, "LN GPIO 8 Off Opcode 2", eeprom.ln_gpio_opcode[15][1], 0)
SV(88, "LN GPIO 8 Off Opcode 3", eeprom.ln_gpio_opcode[15][2], 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t ln_threshold;
uint8_t ln_gpio_ack_count;
uint8_t ln_wdt_enable;
uint8_t ln_gpio_opcode[16][3];
#endif

/*
 *	EEPROM Status Variable Definition
 */
#ifdef EEPROM_STATUS_CFG
uint8_t ln_gpio_status[2];
#endif

/*
 *	EEPROM Confiuration Variable Default Configuration
 */
#ifdef EEPROM_DEFAULT
.ln_threshold = 20,
.ln_gpio_ack_count = 0,
.ln_wdt_enable = 0,
.ln_gpio_opcode = {
	{ 0xB0, 0x01, 0x20},
	{ 0xB0, 0x01, 0x00},
	{ 0xB0, 0x02, 0x20},
	{ 0xB0, 0x02, 0x00},
	{ 0xB0, 0x03, 0x20},
	{ 0xB0, 0x03, 0x00},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0xB0, 0x04, 0x20},
	{ 0xB0, 0x04, 0x00},
	{ 0xB0, 0x05, 0x20},
	{ 0xB0, 0x05, 0x00},
	{ 0xB0, 0x06, 0x20},
	{ 0xB0, 0x06, 0x00},
	{ 0, 0, 0},
	{ 0, 0, 0}
},
#endif

/*
 *	EEPROM Status Variable Default Configuration
 */
#ifdef EEPROM_STATUS_DEFAULT
.ln_gpio_status = { 0, 0 },
#endif

#else
/************************************************************************/
/* Module Header File                                                   */
/************************************************************************/
#ifndef LN_SUPPORT_H_
#define LN_SUPPORT_H_

#include "loconet.h"

#define LN_GPIO_CH_COUNT	16
#define LN_GPIO_BW		((LN_GPIO_CH_COUNT-1)/8L)+1

extern uint8_t ln_gpio_dir[LN_GPIO_BW];
extern uint8_t ln_gpio_tx[LN_GPIO_BW];
extern uint8_t ln_gpio_tx_ack[LN_GPIO_BW];
extern uint8_t ln_gpio_status[LN_GPIO_BW];
extern uint8_t ln_gpio_status_pre[LN_GPIO_BW];
extern uint8_t ln_gpio_status_ack[LN_GPIO_BW];
extern uint8_t ln_gpio_ack_count;
extern uint8_t ln_wdt_counter;

void loconet_init(void);
uint8_t ln_create_message(uint8_t *msg);
uint8_t ln_create_message_ack(uint8_t *msg);
void ln_gpio_process_tx(void);
void ln_gpio_process_rx(lnMsg *LnPacket);
void ln_throttle_process(lnMsg *LnPacket);
void ln_load_board_config(void);
void ln_create_opcode(uint8_t *buf, uint8_t opc, uint16_t addr);

PROCESS_NAME(ln_process);

#endif /* LN_SUPPORT_H_ */
#endif