/*
 * Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
 * 
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 */

#include <stdio.h>
#include <stdint.h>
#include <backtrace.h>

#define BACKTRACE_SIZE 25

static int ping(int ball, backtrace_t *backtrace, int size);
static int pong(int ball, backtrace_t *backtrace, int size);

static void dump_backtrace(const backtrace_t *backtrace, int count)
{
	for (int i = 0; i < count; ++i)
		printf("%p - %s@%p\n", backtrace[i].address, backtrace[i].name, backtrace[i].address);
}

static void ball_location(void *function, int ball)
{
	printf("%s - %d\n", backtrace_function_name((uint32_t)function), ball);
}

static int pong(int ball, backtrace_t *backtrace, int size)
{
	ball_location(pong, ball);

	if (ball > 0)
		return ping(ball - 1, backtrace, size);
	else
		return backtrace_unwind(backtrace, size);
}

static int ping(int ball, backtrace_t *backtrace, int size)
{
	ball_location(pong, ball);

	if (ball > 0)
		return pong(ball - 1, backtrace, size);
	else
		return backtrace_unwind(backtrace, size);
}

int main(int argc, char **argv)
{
	backtrace_t backtrace[BACKTRACE_SIZE];
	
	/* Play ball */
	int count = ping(10, backtrace, BACKTRACE_SIZE);

	/* Dump the backtrace */
	dump_backtrace(backtrace, count);

	/* All good */
	return 0;
}

