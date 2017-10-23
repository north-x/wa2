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

#endif /* LN_SUPPORT_H_ */