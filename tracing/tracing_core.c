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

/** Here set up the timer, we rathe ruse 32 bit to avoid to manu interrupt */
#if defined(CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER)
#include <time.h>
#include <stm32_freerun.h>
struct stm32_freerun_s g_freerun;
#endif // CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER

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

/**
 * @brief This function will init and start the timer 
 * */
static int32_t tracing_init_timer(void)
{
#ifdef CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER
	/** 
	 * We are using the stnm32f4xx timer 2 which correspond to the criteria
	 * of 32 bit timer.
	 */
	return stm32_freerun_initialize(&g_freerun, 
			CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER_ID, 
			CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER_RES_US);
	/* Print the timer status */

#endif //CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER
}

uint64_t tracing_get_counter_value(void)
{
#ifdef CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER
	struct timespec time;

	if (!is_tracing_enabled() || is_tracing_thread()) {
		return 0;
	}

	stm32_freerun_counter(&g_freerun, &time);
	return time.tv_nsec + (uint64_t)time.tv_sec * NSEC_PER_SEC;
#elif TRACE_TIMESTAMP_REFERENCE_SYSTICK_TIMER
	return g_system_timer;
#else
#error "No Counter selected"
#endif //TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER
}

int tracing_init(void)
{
	/** Init the internal ring buffer */
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
			CONFIG_ASYNC_THREAD_CTF_PRIO,
			CONFIG_ASYNC_THREAD_CTF_STACK,
			(main_t) tracing_thread_func, (FAR char * const *) NULL);
#endif
	/** start tracing */
	if (tracing_init_timer()) {
		/** Werror while tracing do not init the trace module*/	
		return -1;
	}

	tracing_set_state(TRACING_ENABLE);
	return 0;
}

#ifdef CONFIG_ASYNC_CTF_TRACING
void tracing_trigger_output(bool before_put_is_empty)
{
	if (before_put_is_empty) {
		work_queue(LPWORK,
			   &tracing_trigger_flush,
			   tracing_thread_timer_expiry_fn,
			   NULL,
			   10);
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

int tracing_finish(void)
{
	nxsem_post(&tracing_thread_sem);
	return 0;
}
