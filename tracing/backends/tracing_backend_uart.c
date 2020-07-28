/*
 * Copyright (c) 2019 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
//#include <ctype.h>
//#include <kernel.h>
//#include <device.h>
//#include <drivers/uart.h>
//#include <sys/__assert.h>

#include <tracing_core.h>
#include <tracing_buffer.h>
#include <fcntl.h>

#define CONFIG_TRACING_BACKEND_UART_NAME "/dev/ttyS0"
int g_fd_uart = -1;

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

static void tracing_backend_uart_output(
		const struct tracing_backend *backend,
		u8_t *data, u32_t length)
{
	//for (u32_t i = 0; i < length; i++) {
	// TOOD MAYBE use internal primitive?
	write(g_fd_uart, data, length);
	//}
}

static void tracing_backend_uart_init(void)
{
	g_fd_uart = open(CONFIG_TRACING_BACKEND_UART_NAME, O_WRONLY);
	//__ASSERT(g_fd_uart >= 0, "uart backend not found\n");

#ifdef CONFIG_TRACING_HANDLE_HOST_CMD
// TODO Not for now?
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

// TODO MOVE IT
#if 1
TRACING_BACKEND_DEFINE(tracing_backend_uart, tracing_backend_uart_api);
#endif

#if 0
const struct tracing_backend uart_backend __attribute__((section("._tracing_backend"))) __used = {
		.name = "tracing_backend_uart",
		.api = &tracing_backend_uart_api,
} ;

#endif
