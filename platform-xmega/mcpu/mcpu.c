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
#include "sys/process.h"
#include "sys/etimer.h"
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "eeprom.h"

#include "mcpu.h"
#include "instructions.h"
#include "tables.h"
#include "ln_support.h"
#include "port.h"
#include "servo.h"

PROCESS(mcpu_process, "6502");
PROCESS(mcpu_tick_process, "6502 Tick Process");
static struct etimer mcpu_timer;
uint8_t mcpu_core_status;

uint8_t var1, var2, var3;
uint8_t arr1[8];
#include "test.src.c"

mcpu_context_t *mcpu_ctx;
mcpu_context_t mcpu_cores[MCPU_CORE_COUNT];
mcpu_program_t mcpu_program_store[MCPU_CORE_COUNT] EEMEM;

const void * const userpointer [] PROGMEM  = {
	#include "mcpu_config_include.h"
};

void mcpu_start(void)
{
	mcpu_load_scripts();
	process_start(&mcpu_process, NULL);
	process_start(&mcpu_tick_process, NULL);
}

PROCESS_THREAD(mcpu_process, ev, data)
{
	PROCESS_BEGIN();
	
	while (1)
	{
		mcpu_supervisor();
		PROCESS_PAUSE();
	}
	
	PROCESS_END();
}

PROCESS_THREAD(mcpu_tick_process, ev, data)
{
	static uint8_t index;
	PROCESS_BEGIN();
	
	etimer_set(&mcpu_timer, 1E-3*CLOCK_SECOND);
	
	while (1)
	{
		for (index=0;index<MCPU_CORE_COUNT;index++)
		{
			if (mcpu_cores[index].timer)
				mcpu_cores[index].timer--;
		}
		etimer_reset(&mcpu_timer);
		PROCESS_YIELD();
	}
	
	PROCESS_END();
}

uint8_t mcpu_read_mem(uint16_t address)
{
	if (address>255)
	{
		return mcpu_ctx->prog_mem[(uint8_t) (address&0xFF)];
	}
	
	if (address<MCPU_SIZE_DATA_MEMOMRY)
	{
		return mcpu_ctx->data_mem[address];
	}

	uint8_t *temp = (uint8_t *) pgm_read_word(&userpointer[address-MCPU_SIZE_DATA_MEMOMRY]);
	return *temp;
}

void mcpu_write_mem(uint16_t address, uint8_t value)
{	
	if (address>255)
	{
		return;
	}
	
	if (address<MCPU_SIZE_DATA_MEMOMRY)
	{
		mcpu_ctx->data_mem[address] = value;
		return;
	}
	
	uint8_t *temp = (uint8_t *) pgm_read_word(&userpointer[address-32]);

	*temp = value;
}

void mcpu_init(void)
{
	var1 = 1;
	var2 = 2;
	var3 = 3;
	
	mcpu_ctx->PC = 0x100;
	
	for (unsigned char index=0;index<test_length;index++)
	{
		mcpu_ctx->prog_mem[index] = test[index];
	}
}

void mcpu_supervisor(void)
{
	uint8_t index, opcode;
	
	for (index=0;index<MCPU_CORE_COUNT;index++)
	{
		mcpu_ctx = &mcpu_cores[index];
		
		if (!mcpu_ctx->status.flags.R)
		{
			mcpu_init();
			mcpu_ctx->status.flags.R = 1;
		}
		
		// Stop execution if break flag is set
		if (mcpu_ctx->status.flags.B)
		{
			mcpu_core_status &= ~(1<<index);
			continue;
		}
		else
		{
			mcpu_core_status |= (1<<index);
		}
		
		// Restore CPU State
		A = mcpu_ctx->A;
		X = mcpu_ctx->X;
		Y = mcpu_ctx->Y;
		emPC = mcpu_ctx->PC;
		S = mcpu_ctx->S;
		status = mcpu_ctx->status;
		
		// Execute one instruction
		opcode = READ_CODE(emPC); ++emPC;
		EXECUTE(opcode);
		
		// Backup CPU State
		mcpu_ctx->A = A;
		mcpu_ctx->X = X;
		mcpu_ctx->Y = Y;
		mcpu_ctx->PC = emPC;
		mcpu_ctx->S = S;
		mcpu_ctx->status = status;
	}
}

void mcpu_load_scripts(void)
{
	mcpu_context_t *ctx;
	uint8_t index, temp8;
	
	for (index=0;index<MCPU_CORE_COUNT;index++)
	{
		ctx = &mcpu_cores[index];
		
		temp8 = eeprom_read_byte(mcpu_program_store[index].program);
		
		if (temp8!=0xFF)
		{
			eeprom_read_block(ctx->prog_mem, mcpu_program_store[index].program, MCPU_SIZE_PROGRAM_MEMORY);
		}
		else
		{
			// Load default program	
		}
		
		// Clear all flags
		ctx->status.u08 = 0;
		
		// Then set break flag if autostart disabled
		if (!(eeprom.mcpu_autostart&(1<<index)))
		{
			ctx->status.flags.B	= 1;
		}
	}
}

void mcpu_save_scripts(void)
{
	
	
}

void mcpu_load_default_scripts(void)
{
	
}