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

#ifndef WA2_H_
#define WA2_H_

#define MAP_BITS(SRC_REG, DEST_REG, SRC_BIT, DEST_BIT) if (SRC_REG&(1<<(SRC_BIT))) DEST_REG |= (1<<(DEST_BIT)); else DEST_REG &= ~(1<<(DEST_BIT))

/************************************************************************/
/* Config Flags                                                         */
/************************************************************************/
#define WA2_CONF_SERVO_PWR_ALWAYS_ON	0
#define WA2_CONF_SERVO_ENABLE_PWM_A		1
#define WA2_CONF_SERVO_ENABLE_PWM_B		2
#define WA2_CONF_RELAY_MONOSTABLE		3
#define WA2_CONF_PWM_OUTPUTS_ENABLE		4
#define WA2_CONF_PWM_CHANNEL7_ENABLE	5
#define WA2_CONF_INPUTS_PULLUP_ENABLE	6

/************************************************************************/
/* Function Prototypes                                                  */
/************************************************************************/
void wa2_update_configuration(void);

#endif /* WA2_H_ */