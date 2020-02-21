/*
 * Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 */

#ifndef BACKTRACE_H_
#define BACKTRACE_H_

#include <stdint.h>

typedef struct backtrace_frame
{
	uint32_t fp;
	uint32_t sp;
	uint32_t lr;
	uint32_t pc;
} backtrace_frame_t;

typedef struct backtrace
{
	void *function;
	void *address;
	const char *name;
} backtrace_t;

typedef struct unwind_control_block
{
	uint32_t vrs[16];
	const uint32_t *current;
	int remaining;
	int byte;
} unwind_control_block_t;

typedef struct unwind_index
{
	uint32_t addr_offset;
	uint32_t insn;
} unwind_index_t;

/* These symbols point to the unwind index and should be provide by the linker script */
extern const unwind_index_t __exidx_start[];
extern const unwind_index_t __exidx_end[];

int _backtrace_unwind(backtrace_t *buffer, int size, backtrace_frame_t *frame);
const char *backtrace_function_name(uint32_t pc);

static inline int __attribute__((always_inline)) backtrace_unwind(backtrace_t *buffer, int size)
{
	/* Get the current pc */
	register uint32_t pc;
	__asm__ volatile("mov %0, pc" : "=r"(pc));

	/* Initialize the stack frame */
	backtrace_frame_t frame;
	frame.sp = (uint32_t)__builtin_frame_address(0);
	frame.fp = (uint32_t)__builtin_frame_address(0);
	frame.lr = (uint32_t)__builtin_return_address(0);
	frame.pc = pc;

	/* Let it rip */
	return _backtrace_unwind(buffer, size, &frame);
}

#endif /* BACKTRACE_H_ */
