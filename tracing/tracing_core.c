/*
 * Copyright (c) 2019 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <init.h>
#include <string.h>
#include <atomic.h>
#include <tracing_core.h>
#include <tracing_buffer.h>
#include <stdbool.h>
#include <tracing_backend.h>

#include <nuttx/kthread.h>
#include <nuttx/tracing/tracing_common.h>
#include <nuttx/arch.h>

#define TRACING_CMD_ENABLE  "enable"
#define TRACING_CMD_DISABLE "disable"

#ifdef CONFIG_CTF_BACKEND_TRACING_SERIAL
#define TRACING_BACKEND_NAME "tracing_backend_uart"
#elif defined CONFIG_CTF_BACKEND_TRACING_USB
#define TRACING_BACKEND_NAME "tracing_backend_usb"
#elif defined CONFIG_CTF_BACKEND_TRACING_POSIX
#define TRACING_BACKEND_NAME "tracing_backend_posix"
#else
#define TRACING_BACKEND_NAME ""
#endif

enum tracing_state {
	TRACING_DISABLE = 0,
	TRACING_ENABLE
};

static atomic_t tracing_state;
static atomic_t tracing_packet_drop_num;
static struct tracing_backend *working_backend;

#ifdef CONFIG_ASYNC_CTF_TRACING
static pid_t tracing_task_pid;
static struct work_s tracing_trigger_flush;
static sem_t tracing_thread_sem;

static int tracing_thread_func(int argc, char *argv[])
{
	u8_t *transferring_buf;
	u32_t transferring_length, tracing_buffer_max_length;

	//tracing_task_pid = sched_self()->pid;

	tracing_buffer_max_length = tracing_buffer_capacity_get();

	while (true) {
		if (tracing_buffer_is_empty()) {
			nxsem_wait(&tracing_thread_sem);
		} else {
			transferring_length =
				tracing_buffer_get_claim(
						&transferring_buf,
						tracing_buffer_max_length);
			tracing_buffer_handle(transferring_buf,
					      transferring_length);
			tracing_buffer_get_finish(transferring_length);
		}
	}

	return 0;
}

static void tracing_thread_timer_expiry_fn(void *item)
{
	nxsem_post(&tracing_thread_sem);
}
#endif

static void tracing_set_state(enum tracing_state state)
{
	atomic_set(&tracing_state, state);
}

int tracing_init(void)
{
	//ARG_UNUSED(arg);;
	//
	tracing_buffer_init();
	nxsem_init(&tracing_thread_sem, 0, 1);
  	nxsem_setprotocol(&tracing_thread_sem, SEM_PRIO_NONE);

	working_backend = tracing_backend_get(TRACING_BACKEND_NAME);
	if (!working_backend) {
		return -1;
	}
	tracing_backend_init(working_backend);

	atomic_set(&tracing_packet_drop_num, 0);

	

#ifdef CONFIG_ASYNC_CTF_TRACING
	memset(&tracing_trigger_flush, 0, sizeof(struct work_s));
	tracing_task_pid = kthread_create(CONFIG_ASYNC_THREAD_CTF_NAME,
		       	180,
			CONFIG_ASYNC_THREAD_CTF_STACK,
			(main_t) tracing_thread_func, (FAR char * const) NULL);
#endif

	tracing_set_state(TRACING_ENABLE);
	return 0;
}

SYS_INIT(tracing_init, APPLICATION, 0);

#ifdef CONFIG_ASYNC_CTF_TRACING
void tracing_trigger_output(bool before_put_is_empty)
{
	if (before_put_is_empty) {
		work_queue(LPWORK,
			       	&tracing_trigger_flush,
			       	tracing_thread_timer_expiry_fn, 
				NULL, 
				100);

	}
}

bool is_tracing_thread(void)
{
	if (!up_interrupt_context() && (sched_self()->pid == tracing_task_pid)) {
		return true;
	}

	return false;
}
#endif

bool is_tracing_enabled(void)
{
	return atomic_get(&tracing_state) == TRACING_ENABLE;
}

void tracing_cmd_handle(u8_t *buf, u32_t length)
{
	if (strncmp((char *)buf, TRACING_CMD_ENABLE, length) == 0) {
		tracing_set_state(TRACING_ENABLE);
	} else if (strncmp((char *)buf, TRACING_CMD_DISABLE, length) == 0) {
		tracing_set_state(TRACING_DISABLE);
	}
}

void tracing_buffer_handle(u8_t *data, u32_t length)
{
	tracing_backend_output(working_backend, data, length);
}

void tracing_packet_drop_handle(void)
{
	atomic_inc(&tracing_packet_drop_num);
}

struct tracing_backend *tracing_backend_get(char *name)
{
	Z_STRUCT_SECTION_FOREACH(tracing_backend, backend) {
		if (strcmp(backend->name, name) == 0) {
			return backend;
		}
	}

	return NULL;
}

void tracing_backend_output(
		const struct tracing_backend *backend,
		u8_t *data, u32_t length)
{
	if (backend && backend->api) {
		backend->api->output(backend, data, length);
	}
}

void tracing_backend_init(
		const struct tracing_backend *backend)
{
	if (backend && backend->api) {
		backend->api->init();
	}
}
