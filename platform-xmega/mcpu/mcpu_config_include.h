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
 
#if defined (__AVR_ARCH__)
#define USER_VAR_CAT(a,b,c) a ## _ ## b ## _ ## c

#define MEMORY_MAPPING
#define USER_VARIABLE_1_1(var) (void *) &var,
#define USER_VARIABLE_1_2(var) \
(void *) &var[0], \
(void *) &var[1],
#define USER_VARIABLE_1_3(var) \
(void *) &var[0], \
(void *) &var[1], \
(void *) &var[2],
#define USER_VARIABLE_1_8(var) \
(void *) &var[0], \
(void *) &var[1], \
(void *) &var[2], \
(void *) &var[3], \
(void *) &var[4], \
(void *) &var[5], \
(void *) &var[6], \
(void *) &var[7],

#define USER_VARIABLE(name,text,var,update) USER_VAR_CAT(USER_VARIABLE,1,1)(var)
#define USER_ARRAY(name,text,var,len,update) USER_VAR_CAT(USER_VARIABLE,1,len)(var)
#include "mcpu_config.h"
#undef USER_VARIABLE
#undef USER_VAR_CAT
#undef MEMORY_MAPPING
#endif

#if defined (__CC65__)
#define USER_VAR_CAT(a, type, len, var) a ## _ ## type ## _ ## len ## ( ## type ## , ## var ## )
typedef struct param_struct
{
#define MEMORY_MAPPING
#define USER_VARIABLE_1_1(type, var) unsigned char var;
#define USER_VARIABLE_1_2(type, var) unsigned char var[2];
#define USER_VARIABLE_1_3(type, var) unsigned char var[3];
#define USER_VARIABLE_1_4(type, var) unsigned char var[4];
#define USER_VARIABLE_1_8(type, var) unsigned char var[8];
#define USER_VARIABLE_1_12(type, var) unsigned char var[12];
#define USER_VARIABLE_1_16(type, var) unsigned char var[16];
#define USER_VARIABLE_2_1(type, var) unsigned short var;

#define USER_VARIABLE(name,text,var,update) USER_VAR_CAT(USER_VARIABLE,1,1,text)
#define USER_ARRAY(name,text,var,len,update) USER_VAR_CAT(USER_VARIABLE,1,len,text)

#include "mcpu_config.h"
#undef USER_VARIABLE
#undef USER_VAR_CAT
#undef MEMORY_MAPPING
} PS_t;
#endif