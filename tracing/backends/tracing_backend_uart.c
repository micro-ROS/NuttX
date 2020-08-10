/*
 * Copyright (c) 2019 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nuttx/config.h>

#include <tracing_core.h>
#include <tracing_buffer.h>
#include <sys/stat.h>
#include <fcntl.h>

#define OPEN_MODE  (S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR)
#define CONFIG_TRACING_BACKEND_UART_NAME "/dev/ttyS0"

#define USART3_SR	0x40004800
#define USART3_DR	0x40004804

FAR struct file g_backend_uart;
FAR bool is_first = true;
FAR int g_error_open = 0;
static const char starter[] = {0xbe, 0xbe, 0xde, 0xad};

#ifdef CONFIG_TRACING_HANDLE_HOST_CMD
// TODO Not for now?
static void uart_isr(struct device *dev)
{
	int rx;
	u8_t byte;
	static u8_t *cmd;
	static u32_t length, cur;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		rx = uart_fifo_read(dev, &byte, 1);
		if (rx < 0) {
			uart_irq_rx_disable(dev);
			return;
		}

		if (!cmd) {
			length = tracing_cmd_buffer_alloc(&cmd);
		}

		if (!isprint(byte)) {
			if (byte == '\r') {
				cmd[cur] = '\0';
				tracing_cmd_handle(cmd, cur);
				cmd = NULL;
				cur = 0U;
			}

			continue;
		}

		if (cur < length - 1) {
			cmd[cur++] = byte;
		}
	}
}
#endif

static void raw_writing(u8_t *data, u32_t length)
{
	volatile uint32_t *uart_st = (volatile uint32_t *) USART3_SR;
	volatile uint32_t *uart_dt = (volatile uint32_t *) USART3_DR;

	for (uint32_t i = 0 ; i < length; i++) {
		while(!(*uart_st & (1 << 7))) {asm("nop");}
		*uart_dt = data[i];
	}
}

static void tracing_backend_uart_output(
		const struct tracing_backend *backend,
		u8_t *data, u32_t length)
{

	if (is_first) {
		raw_writing(starter, sizeof(starter));
		is_first = false;
	}

	raw_writing(data, length);
}

static void tracing_backend_uart_init(void)
{
  	int ret = file_open(&g_backend_uart, CONFIG_TRACING_BACKEND_UART_NAME,
			O_RDWR, OPEN_MODE);
	if (ret) {
		g_error_open = 1;
		return;
	}

#ifdef CONFIG_TRACING_HANDLE_HOST_CMD
	uart_irq_rx_disable(dev);
	uart_irq_tx_disable(dev);

	uart_irq_callback_set(dev, uart_isr);

	while (uart_irq_rx_ready(dev)) {
		u8_t c;

		uart_fifo_read(dev, &c, 1);
	}

	uart_irq_rx_enable(dev);
#endif
}

static const struct tracing_backend_api tracing_backend_uart_api = {
	.init = tracing_backend_uart_init,
	.output  = tracing_backend_uart_output
};

TRACING_BACKEND_DEFINE(tracing_backend_uart, tracing_backend_uart_api);
