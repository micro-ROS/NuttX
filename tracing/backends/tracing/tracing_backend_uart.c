/*
 * Copyright (c) 2019 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nuttx/config.h>

#include <backend.h>

#include <tracing_core.h>
#include <tracing_buffer.h>
#include <sys/stat.h>
#include <fcntl.h>

#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)
#ifndef CONFIG_CTF_TRACING_BACKEND_UART_NAME
#define CONFIG_CTF_TRACING_BACKEND_UART_NAME "/dev/ttyS0"
#endif //CONFIG_CTF_TRACING_BACKEND_UART_NAME

#define USART3_SR	0x40004800
#define USART3_DR	0x40004804
#define USART6_SR	0x40011400
#define USART6_DR	0x40011404

// #ifdef CONFIG_USART3_SERIAL_CONSOLE
#ifdef CONFIG_CTF_BACKEND_SERIAL_UART6
	#define USART_SR	USART6_SR
	#define USART_DR	USART6_DR
#else
	#define USART_SR	USART3_SR
	#define USART_DR	USART3_DR
#endif

FAR struct file g_backend_uart;
FAR bool is_first = true;
FAR int g_error_open = 0;

static const char starter[] = {0xbe, 0xbe, 0xde, 0xad};

static void raw_writing(u8_t *data, u32_t length)
{
	volatile uint32_t *uart_st = (volatile uint32_t *) USART_SR;
	volatile uint32_t *uart_dt = (volatile uint32_t *) USART_DR;

	for (uint32_t i = 0 ; i < length; i++) {
		while(!(*uart_st & (1 << 7))) {asm("nop");}
		*uart_dt = data[i];
	}
}

static void tracing_backend_uart_output(
		const struct tracing_backend *backend,
		const u8_t *data, u32_t length)
{
	if (is_first) {
		raw_writing(starter, sizeof(starter));
		is_first = false;
	}

	raw_writing(data, length);
}

static void tracing_backend_uart_init(void)
{
  	int ret = file_open(&g_backend_uart, CONFIG_CTF_TRACING_BACKEND_UART_NAME,
			O_RDWR, OPEN_MODE);
	if (ret) {
		g_error_open = 1;
		return;
	}
}

static const struct tracing_backend_api tracing_backend_uart_api = {
	.init = tracing_backend_uart_init,
	.output  = tracing_backend_uart_output
};

TRACING_BACKEND_DEFINE(tracing_backend_uart, tracing_backend_uart_api);
