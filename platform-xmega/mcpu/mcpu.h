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


#ifndef MCPU_H_
#define MCPU_H_

#define MCPU_CORE_COUNT 2
#define MCPU_SIZE_PROGRAM_MEMORY	128
#define MCPU_SIZE_DATA_MEMOMRY 32

typedef struct
{
	union {
		uint8_t u08;
		struct {
			unsigned C : 1;
			unsigned Z : 1;
			unsigned I : 1;
			unsigned D : 1;
			unsigned B : 1;
			unsigned R : 1;
			unsigned V : 1;
			unsigned N : 1;
		} flags;
	};
} mcpu_flags_t;

typedef struct
{
	mcpu_flags_t status;
	uint8_t timer;
	uint8_t A, X, Y, S;
	uint16_t PC;
	
	uint8_t prog_mem[MCPU_SIZE_PROGRAM_MEMORY];
	uint8_t data_mem[MCPU_SIZE_DATA_MEMOMRY];
} mcpu_context_t;

typedef struct mcpu_program {
	uint8_t program[MCPU_SIZE_PROGRAM_MEMORY];
} mcpu_program_t;

extern mcpu_context_t *mcpu_ctx;
extern mcpu_context_t mcpu_cores[MCPU_CORE_COUNT];
extern mcpu_flags_t status;
extern uint8_t mcpu_core_status;

void mcpu_start(void);
void mcpu_supervisor(void);
void mcpu_load_scripts(void);
void mcpu_save_scripts(void);
void mcpu_load_default_scripts(void);

#endif /* MCPU_H_ */