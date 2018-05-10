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
SV(126, "LN Threshold Voltage x10", eeprom.ln_threshold, ln_update_threshold)
SV(127, "LN Ack Count", eeprom.ln_gpio_ack_count, 0)
SV(128, "LN Watchdog Enable", eeprom.ln_wdt_enable, 0)
SV(129, "LN GPIO Direction L", eeprom.ln_gpio_dir[0], 0)
SV(130, "LN GPIO Direction H", eeprom.ln_gpio_dir[1], 0)
SV(131, "LN GPIO Status L", ln_gpio_status[0], 0)
SV(132, "LN GPIO Status H", ln_gpio_status[1], 0)
SV(133, "LN GPIO Tx L", ln_gpio_tx[0], 0)
SV(134, "LN GPIO Tx H", ln_gpio_tx[1], 0)
SV(135, "LN GPIO 1 On Opcode 1", eeprom.ln_gpio_opcode[0][0], 0)
SV(136, "LN GPIO 1 On Opcode 2", eeprom.ln_gpio_opcode[0][1], 0)
SV(137, "LN GPIO 1 On Opcode 3", eeprom.ln_gpio_opcode[0][2], 0)
SV(138, "LN GPIO 1 Off Opcode 1", eeprom.ln_gpio_opcode[1][0], 0)
SV(139, "LN GPIO 1 Off Opcode 2", eeprom.ln_gpio_opcode[1][1], 0)
SV(140, "LN GPIO 1 Off Opcode 3", eeprom.ln_gpio_opcode[1][2], 0)
SV(141, "LN GPIO 2 On Opcode 1", eeprom.ln_gpio_opcode[2][0], 0)
SV(142, "LN GPIO 2 On Opcode 2", eeprom.ln_gpio_opcode[2][1], 0)
SV(143, "LN GPIO 2 On Opcode 3", eeprom.ln_gpio_opcode[2][2], 0)
SV(144, "LN GPIO 2 Off Opcode 1", eeprom.ln_gpio_opcode[3][0], 0)
SV(145, "LN GPIO 2 Off Opcode 2", eeprom.ln_gpio_opcode[3][1], 0)
SV(146, "LN GPIO 2 Off Opcode 3", eeprom.ln_gpio_opcode[3][2], 0)
SV(147, "LN GPIO 3 On Opcode 1", eeprom.ln_gpio_opcode[4][0], 0)
SV(148, "LN GPIO 3 On Opcode 2", eeprom.ln_gpio_opcode[4][1], 0)
SV(149, "LN GPIO 3 On Opcode 3", eeprom.ln_gpio_opcode[4][2], 0)
SV(150, "LN GPIO 3 Off Opcode 1", eeprom.ln_gpio_opcode[5][0], 0)
SV(151, "LN GPIO 3 Off Opcode 2", eeprom.ln_gpio_opcode[5][1], 0)
SV(152, "LN GPIO 3 Off Opcode 3", eeprom.ln_gpio_opcode[5][2], 0)
SV(153, "LN GPIO 4 On Opcode 1", eeprom.ln_gpio_opcode[6][0], 0)
SV(154, "LN GPIO 4 On Opcode 2", eeprom.ln_gpio_opcode[6][1], 0)
SV(155, "LN GPIO 4 On Opcode 3", eeprom.ln_gpio_opcode[6][2], 0)
SV(156, "LN GPIO 4 Off Opcode 1", eeprom.ln_gpio_opcode[7][0], 0)
SV(157, "LN GPIO 4 Off Opcode 2", eeprom.ln_gpio_opcode[7][1], 0)
SV(158, "LN GPIO 4 Off Opcode 3", eeprom.ln_gpio_opcode[7][2], 0)
SV(159, "LN GPIO 5 On Opcode 1", eeprom.ln_gpio_opcode[8][0], 0)
SV(160, "LN GPIO 5 On Opcode 2", eeprom.ln_gpio_opcode[8][1], 0)
SV(161, "LN GPIO 5 On Opcode 3", eeprom.ln_gpio_opcode[8][2], 0)
SV(162, "LN GPIO 5 Off Opcode 1", eeprom.ln_gpio_opcode[9][0], 0)
SV(163, "LN GPIO 5 Off Opcode 2", eeprom.ln_gpio_opcode[9][1], 0)
SV(164, "LN GPIO 5 Off Opcode 3", eeprom.ln_gpio_opcode[9][2], 0)
SV(165, "LN GPIO 6 On Opcode 1", eeprom.ln_gpio_opcode[10][0], 0)
SV(166, "LN GPIO 6 On Opcode 2", eeprom.ln_gpio_opcode[10][1], 0)
SV(167, "LN GPIO 6 On Opcode 3", eeprom.ln_gpio_opcode[10][2], 0)
SV(168, "LN GPIO 6 Off Opcode 1", eeprom.ln_gpio_opcode[11][0], 0)
SV(169, "LN GPIO 6 Off Opcode 2", eeprom.ln_gpio_opcode[11][1], 0)
SV(170, "LN GPIO 6 Off Opcode 3", eeprom.ln_gpio_opcode[11][2], 0)
SV(171, "LN GPIO 7 On Opcode 1", eeprom.ln_gpio_opcode[12][0], 0)
SV(172, "LN GPIO 7 On Opcode 2", eeprom.ln_gpio_opcode[12][1], 0)
SV(173, "LN GPIO 7 On Opcode 3", eeprom.ln_gpio_opcode[12][2], 0)
SV(174, "LN GPIO 7 Off Opcode 1", eeprom.ln_gpio_opcode[13][0], 0)
SV(175, "LN GPIO 7 Off Opcode 2", eeprom.ln_gpio_opcode[13][1], 0)
SV(176, "LN GPIO 7 Off Opcode 3", eeprom.ln_gpio_opcode[13][2], 0)
SV(177, "LN GPIO 8 On Opcode 1", eeprom.ln_gpio_opcode[14][0], 0)
SV(178, "LN GPIO 8 On Opcode 2", eeprom.ln_gpio_opcode[14][1], 0)
SV(179, "LN GPIO 8 On Opcode 3", eeprom.ln_gpio_opcode[14][2], 0)
SV(180, "LN GPIO 8 Off Opcode 1", eeprom.ln_gpio_opcode[15][0], 0)
SV(181, "LN GPIO 8 Off Opcode 2", eeprom.ln_gpio_opcode[15][1], 0)
SV(182, "LN GPIO 8 Off Opcode 3", eeprom.ln_gpio_opcode[15][2], 0)
SV(183, "LN GPIO 9 On Opcode 1", eeprom.ln_gpio_opcode[16][0], 0)
SV(184, "LN GPIO 9 On Opcode 2", eeprom.ln_gpio_opcode[16][1], 0)
SV(185, "LN GPIO 9 On Opcode 3", eeprom.ln_gpio_opcode[16][2], 0)
SV(186, "LN GPIO 9 Off Opcode 1", eeprom.ln_gpio_opcode[17][0], 0)
SV(187, "LN GPIO 9 Off Opcode 2", eeprom.ln_gpio_opcode[17][1], 0)
SV(188, "LN GPIO 9 Off Opcode 3", eeprom.ln_gpio_opcode[17][2], 0)
SV(189, "LN GPIO 10 On Opcode 1", eeprom.ln_gpio_opcode[18][0], 0)
SV(190, "LN GPIO 10 On Opcode 2", eeprom.ln_gpio_opcode[18][1], 0)
SV(191, "LN GPIO 10 On Opcode 3", eeprom.ln_gpio_opcode[18][2], 0)
SV(192, "LN GPIO 10 Off Opcode 1", eeprom.ln_gpio_opcode[19][0], 0)
SV(193, "LN GPIO 10 Off Opcode 2", eeprom.ln_gpio_opcode[19][1], 0)
SV(194, "LN GPIO 10 Off Opcode 3", eeprom.ln_gpio_opcode[19][2], 0)
SV(195, "LN GPIO 11 On Opcode 1", eeprom.ln_gpio_opcode[20][0], 0)
SV(196, "LN GPIO 11 On Opcode 2", eeprom.ln_gpio_opcode[20][1], 0)
SV(197, "LN GPIO 11 On Opcode 3", eeprom.ln_gpio_opcode[20][2], 0)
SV(198, "LN GPIO 11 Off Opcode 1", eeprom.ln_gpio_opcode[21][0], 0)
SV(199, "LN GPIO 11 Off Opcode 2", eeprom.ln_gpio_opcode[21][1], 0)
SV(200, "LN GPIO 11 Off Opcode 3", eeprom.ln_gpio_opcode[21][2], 0)
SV(201, "LN GPIO 12 On Opcode 1", eeprom.ln_gpio_opcode[22][0], 0)
SV(202, "LN GPIO 12 On Opcode 2", eeprom.ln_gpio_opcode[22][1], 0)
SV(203, "LN GPIO 12 On Opcode 3", eeprom.ln_gpio_opcode[22][2], 0)
SV(204, "LN GPIO 12 Off Opcode 1", eeprom.ln_gpio_opcode[23][0], 0)
SV(205, "LN GPIO 12 Off Opcode 2", eeprom.ln_gpio_opcode[23][1], 0)
SV(206, "LN GPIO 12 Off Opcode 3", eeprom.ln_gpio_opcode[23][2], 0)
SV(207, "LN GPIO 13 On Opcode 1", eeprom.ln_gpio_opcode[24][0], 0)
SV(208, "LN GPIO 13 On Opcode 2", eeprom.ln_gpio_opcode[24][1], 0)
SV(209, "LN GPIO 13 On Opcode 3", eeprom.ln_gpio_opcode[24][2], 0)
SV(210, "LN GPIO 13 Off Opcode 1", eeprom.ln_gpio_opcode[25][0], 0)
SV(211, "LN GPIO 13 Off Opcode 2", eeprom.ln_gpio_opcode[25][1], 0)
SV(212, "LN GPIO 13 Off Opcode 3", eeprom.ln_gpio_opcode[25][2], 0)
SV(213, "LN GPIO 14 On Opcode 1", eeprom.ln_gpio_opcode[26][0], 0)
SV(214, "LN GPIO 14 On Opcode 2", eeprom.ln_gpio_opcode[26][1], 0)
SV(215, "LN GPIO 14 On Opcode 3", eeprom.ln_gpio_opcode[26][2], 0)
SV(216, "LN GPIO 14 Off Opcode 1", eeprom.ln_gpio_opcode[27][0], 0)
SV(217, "LN GPIO 14 Off Opcode 2", eeprom.ln_gpio_opcode[27][1], 0)
SV(218, "LN GPIO 14 Off Opcode 3", eeprom.ln_gpio_opcode[27][2], 0)
SV(219, "LN GPIO 15 On Opcode 1", eeprom.ln_gpio_opcode[28][0], 0)
SV(220, "LN GPIO 15 On Opcode 2", eeprom.ln_gpio_opcode[28][1], 0)
SV(221, "LN GPIO 15 On Opcode 3", eeprom.ln_gpio_opcode[28][2], 0)
SV(222, "LN GPIO 15 Off Opcode 1", eeprom.ln_gpio_opcode[29][0], 0)
SV(223, "LN GPIO 15 Off Opcode 2", eeprom.ln_gpio_opcode[29][1], 0)
SV(224, "LN GPIO 15 Off Opcode 3", eeprom.ln_gpio_opcode[29][2], 0)
SV(225, "LN GPIO 16 On Opcode 1", eeprom.ln_gpio_opcode[30][0], 0)
SV(226, "LN GPIO 16 On Opcode 2", eeprom.ln_gpio_opcode[30][1], 0)
SV(227, "LN GPIO 16 On Opcode 3", eeprom.ln_gpio_opcode[30][2], 0)
SV(228, "LN GPIO 16 Off Opcode 1", eeprom.ln_gpio_opcode[31][0], 0)
SV(229, "LN GPIO 16 Off Opcode 2", eeprom.ln_gpio_opcode[31][1], 0)
SV(230, "LN GPIO 16 Off Opcode 3", eeprom.ln_gpio_opcode[31][2], 0)
SV(231, "LN Watchdog Counter", ln_wdt_counter, 0)
SV(232, "LN Rx Error Counter", LnBuffer.Stats.RxErrors, 0)
SV(233, "LN Tx Error Counter", LnBuffer.Stats.TxError, 0)
SV(234, "LN Collision Counter", LnBuffer.Stats.Collisions, 0)
SV_LSB(235, "LN Rx Packets Counter L", LnBuffer.Stats.RxPackets, 0)
SV_MSB(236, "LN Rx Packets Counter H", LnBuffer.Stats.RxPackets, 0)
SV_LSB(237, "LN Tx Packets Counter L", LnBuffer.Stats.TxPackets, 0)
SV_MSB(238, "LN Rx Packets Counter H", LnBuffer.Stats.TxPackets, 0)
#endif

/*
 *	EEPROM Configuration Variable Definition
 */
#ifdef EEPROM_CFG
uint8_t ln_threshold;
uint8_t ln_wdt_enable;
uint8_t ln_gpio_ack_count;
uint8_t ln_gpio_dir[2];
uint8_t ln_gpio_opcode[32][3];
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
.ln_wdt_enable = 0,
.ln_gpio_ack_count = 0,
.ln_gpio_dir = { 0, 0},
.ln_gpio_opcode = {
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
	{ 0, 0, 0},
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
#include "ln_buf.h"

#define LN_GPIO_CH_COUNT	16
#define LN_GPIO_BW		((LN_GPIO_CH_COUNT-1)/8L)+1

extern uint8_t ln_gpio_tx[LN_GPIO_BW];
extern uint8_t ln_gpio_tx_ack[LN_GPIO_BW];
extern uint8_t ln_gpio_status[LN_GPIO_BW];
extern uint8_t ln_gpio_status_pre[LN_GPIO_BW];
extern uint8_t ln_gpio_status_ack[LN_GPIO_BW];
extern uint8_t ln_gpio_status_flag[LN_GPIO_BW];
extern uint8_t ln_wdt_counter;
extern LnBuf LnBuffer;

void loconet_init(void);
uint8_t ln_create_message(uint8_t *msg);
uint8_t ln_create_message_ack(uint8_t *msg);
uint8_t ln_is_ack_message(uint8_t *msg);
void ln_gpio_process_tx(void);
void ln_gpio_process_rx(lnMsg *LnPacket);
void ln_load_board_config(void);
void ln_create_opcode(uint8_t *buf, uint8_t opc, uint16_t addr);
void ln_update_threshold(void);

PROCESS_NAME(ln_process);

#endif /* LN_SUPPORT_H_ */
#endif