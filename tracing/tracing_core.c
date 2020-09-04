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
#include <backend.h>
#include <backend_timer.h>

#include <nuttx/kthread.h>
#include <nuttx/tracing/tracing_common.h>
#include <nuttx/arch.h>

#ifdef CONFIG_ASYNC_CTF_TRACING
#include <nuttx/tracing/ctf_map.h>
#include <nuttx/tracing/ctf_types.h>
#endif

#define TRACING_CMD_ENABLE  "enable"
#define TRACING_CMD_DISABLE "disable"

#ifdef CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER_FREERUNER
#define CONFIG_TRACING_BACKEND_TIMER_NAME "backend_timer_freerunner"
#elif defined(CONFIG_TRACE_TIMESTAMP_REFERENCE_SYSTICK_TIMER)
#define CONFIG_TRACING_BACKEND_TIMER_NAME "backend_timer_systick"
#else 
#define CONFIG_TRACING_BACKEND_TIMER_NAME ""
#endif //CONFIG_CTF_BACKEND_TIMER_STM32_FREERUNNER

#ifdef CONFIG_CTF_BACKEND_TRACING_SERIAL
#define CONFIG_TRACING_BACKEND_NAME "tracing_backend_uart"
#elif defined CONFIG_CTF_BACKEND_TRACING_USB
#define CONFIG_TRACING_BACKEND_NAME "tracing_backend_usb"
#elif defined CONFIG_CTF_BACKEND_TRACING_POSIX
#define CONFIG_TRACING_BACKEND_NAME "tracing_backend_posix"
#else
#define CONFIG_TRACING_BACKEND_NAME ""
#endif //CONFIG_CTF_BACKEND_TRACING_SERIAL

enum tracing_state {
	TRACING_DISABLE = 0,
	TRACING_ENABLE
};

static atomic_t tracing_state;
static atomic_t tracing_packet_drop_num;
static struct tracing_backend *working_backend;
static struct backend_timer *btimer;

#ifdef CONFIG_ASYNC_CTF_TRACING

/*
 * Obtain a field's size at compile-time.
 */
#define CTF_INTERNAL_FIELD_SIZE(x)      + sizeof(x)


#define CTF_INTERNAL_FIELD_APPEND(x)			 \
	{						 \
		memcpy(epacket_cursor, &(x), sizeof(x)); \
		epacket_cursor += sizeof(x);		 \
	}

/*
 * Gather fields to a contiguous event-packet, then atomically emit.
 */
#define CTF_GATHER_FIELDS(...)						    \
{									    \
	u8_t epacket[0 MAP(CTF_INTERNAL_FIELD_SIZE, ##__VA_ARGS__)];	    \
	u8_t *epacket_cursor = &epacket[0];				    \
									    \
	MAP(CTF_INTERNAL_FIELD_APPEND, ##__VA_ARGS__)			    \
	tracing_buffer_handle(epacket, sizeof(epacket));                    \
}

#define CTF_EVENT(...)							    \
	{								    \
		const u64_t tstamp = tracing_get_counter_for_dump();	    \
		CTF_GATHER_FIELDS(tstamp, __VA_ARGS__)			    \
	}

/* Anonymous compound literal with 1 member. Legal since C99.
 * This permits us to take the address of literals, like so:
 *  &CTF_LITERAL(int, 1234)
 *
 * This may be required if a ctf_bottom layer uses memcpy.
 *
 * NOTE string literals already support address-of and sizeof,
 * so string literals should not be wrapped with CTF_LITERAL.
 */
#define CTF_LITERAL(type, value)  ((type) { (type)(value) })


static pid_t tracing_task_pid;
static struct work_s tracing_trigger_flush;
static sem_t tracing_thread_sem;

#ifdef TRACE_CTF_FUNCTIONS_USAGE
volatile uint64_t g_measuredtimestamp = 0;
#endif

uint64_t tracing_get_counter_value(void)
{
	return backend_timer_gettime(btimer);
}

#ifdef TRACE_CTF_FUNCTIONS_USAGE
uint64_t tracing_get_counter_for_dump(void)
{
	return g_measuredtimestamp;
}
#endif //TRACE_CTF_FUNCTIONS_USAGE

static int tracing_thread_func(int argc, char *argv[])
{
	u8_t *transferring_buf;
#ifdef TRACE_CTF_FUNCTIONS_USAGE
	u8_t start_id = START_ID;
	u8_t stop_id = STOP_ID;
#endif
	u32_t transferring_length, tracing_buffer_max_length;

	//tracing_task_pid = sched_self()->pid;

	tracing_buffer_max_length = tracing_buffer_capacity_get();
	while (true) {
		if (tracing_buffer_is_empty()) {
#ifdef TRACE_CTF_FUNCTIONS_USAGE
			g_measuredtimestamp =  tracing_get_counter_value();
			CTF_EVENT(CTF_LITERAL(u8_t, stop_id));
#endif
			nxsem_wait(&tracing_thread_sem);
		} else {
#ifdef TRACE_CTF_FUNCTIONS_USAGE
			g_measuredtimestamp =  tracing_get_counter_value();
#endif
			transferring_length =
				tracing_buffer_get_claim(
						&transferring_buf,
						tracing_buffer_max_length);
			tracing_buffer_handle(transferring_buf,
					      transferring_length);
			tracing_buffer_get_finish(transferring_length);
#ifdef TRACE_CTF_FUNCTIONS_USAGE
			CTF_EVENT(CTF_LITERAL(u8_t, start_id));
#endif
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
	/** Init the internal ring buffer */
	tracing_buffer_init();

#ifdef CONFIG_ASYNC_CTF_TRACING
	nxsem_init(&tracing_thread_sem, 0, 1);
  	nxsem_setprotocol(&tracing_thread_sem, SEM_PRIO_NONE);
#endif

	btimer = backend_timer_get(CONFIG_TRACING_BACKEND_TIMER_NAME);
	if (!btimer) {
		return -1;
	}
	backend_timer_init(btimer);

	working_backend = tracing_backend_get(CONFIG_TRACING_BACKEND_NAME);
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
			   MSEC2TICK(CONFIG_ASYNC_THREAD_CTF_WAIT_THRESHOLD)
		);
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

int tracing_finish(void)
{
	tracing_set_state(TRACING_DISABLE);
}
