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

#ifdef MEMORY_MAPPING
USER_VARIABLE(1, ln_gpio_status, ln_gpio_status, 1)
USER_VARIABLE(2, ln_gpio_tx, ln_gpio_status_tx, 1)
USER_VARIABLE(3, relay_request, relay_request, 1)
USER_VARIABLE(4, servo1_time_delta, servo[0].time_delta, 1)
USER_VARIABLE(5, servo1_setpoint, servo[0].position_setpoint, 1)
USER_VARIABLE(6, servo1_position, servo[0].position_actual, 1)
USER_VARIABLE(7, servo2_time_delta, servo[1].time_delta, 1)
USER_VARIABLE(8, servo2_setpoint, servo[1].position_setpoint, 1)
USER_VARIABLE(9, servo2_position, servo[1].position_actual, 1)

/*
USER_VARIABLE(0, "var1", var1, 0)
USER_VARIABLE(1, "var2", var2, update_func)
USER_ARRAY(2, "arr1", arr1, 8, update_func)
*/

#endif