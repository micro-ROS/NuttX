/*
* Copyright (c) 2018 Oticon A/S
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <stddef.h>
#include <string.h>
#include <nuttx/tracing/ctf_map.h>
#include <nuttx/tracing/ctf_types.h>
#include <nuttx/tracing/tracing_format.h>
#include <nuttx/tracing/tracing_probes.h>
#include <nuttx/sched.h>

#define CPU_0 0

/* Limit strings to 20 bytes to optimize bandwidth */
#define CTF_MAX_STRING_LEN 20

/*
 * Obtain a field's size at compile-time.
 */
#define CTF_INTERNAL_FIELD_SIZE(x)      + sizeof(x)

/*
 * Append a field to current event-packet.
 */
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
	tracing_format_raw_data(epacket, sizeof(epacket));		    \
}

#if 1
#define CTF_EVENT(...)							    \
	{								    \
		const u32_t tstamp = g_system_timer;			    \
		CTF_GATHER_FIELDS(tstamp, __VA_ARGS__)			    \
	}
#else
#define CTF_EVENT(...)							    \
	{								    \
		CTF_GATHER_FIELDS(__VA_ARGS__)				    \
	}
#endif

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

typedef enum {
	CTF_EVENT_THREAD_SWITCHED_OUT   =  0x10,
	CTF_EVENT_THREAD_SWITCHED_IN    =  1188,
	CTF_EVENT_THREAD_PRIORITY_SET   =  0x12,
	CTF_EVENT_THREAD_CREATE         =  0x13,
	CTF_EVENT_THREAD_ABORT          =  0x14,
	CTF_EVENT_THREAD_SUSPEND        =  0x15,
	CTF_EVENT_THREAD_RESUME         =  0x16,
	CTF_EVENT_THREAD_READY          =  0x17,
	CTF_EVENT_THREAD_PENDING        =  0x18,
	CTF_EVENT_THREAD_INFO           =  0x19,
	CTF_EVENT_THREAD_NAME_SET       =  0x1A,
	CTF_EVENT_ISR_ENTER             =  0x20,
	CTF_EVENT_ISR_EXIT              =  0x21,
	CTF_EVENT_ISR_EXIT_TO_SCHEDULER =  0x22,
	CTF_EVENT_IDLE                  =  0x30,
	CTF_EVENT_ID_START_CALL         =  0x41,
	CTF_EVENT_ID_END_CALL           =  0x42,
} ctf_event_t;


typedef struct {
	char buf[CTF_MAX_STRING_LEN];
} ctf_bounded_string_t;


static  void ctf_top_thread_switched_out(u32_t thread_id)
{
	u8_t cpu = 1;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_SWITCHED_OUT),
		cpu,
		thread_id
		);
}

static  void ctf_top_thread_switched_in(u32_t thread_id)
{
	const char thread_name[20] = "Switching_Thread";
	uint32_t prio = 100;

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_SWITCHED_IN),
		thread_name,
		thread_id,
		prio	
		);
}

static  void ctf_top_thread_priority_set(u32_t thread_id, s8_t prio)
{
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_PRIORITY_SET),
		thread_id,
		prio
		);
}

static  void ctf_top_thread_create(
	u32_t thread_id,
	s8_t prio,
	ctf_bounded_string_t name
	)
{

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_CREATE),
		thread_id,
		name
		);
}

static  void ctf_top_thread_abort(u32_t thread_id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_ABORT),
		thread_id
		);
}

static  void ctf_top_thread_suspend(u32_t thread_id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_SUSPEND),
		thread_id
		);
}

static  void ctf_top_thread_resume(u32_t thread_id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_RESUME),
		thread_id
		);
}

static  void ctf_top_thread_ready(u32_t thread_id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_READY),
		thread_id
		);
}

static  void ctf_top_thread_pend(u32_t thread_id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_PENDING),
		thread_id
		);
}

static  void ctf_top_thread_info(
	u32_t thread_id,
	u32_t stack_base,
	u32_t stack_size
	)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_INFO),
		thread_id,
		stack_base,
		stack_size
		);
}

static  void ctf_top_thread_name_set(
	u32_t thread_id,
	ctf_bounded_string_t name
	)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_NAME_SET),
		thread_id,
		name
		);
}

static  void ctf_top_isr_enter(void)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ISR_ENTER)
		);
}

static  void ctf_top_isr_exit(void)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ISR_EXIT)
		);
}

static  void ctf_top_isr_exit_to_scheduler(void)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ISR_EXIT_TO_SCHEDULER)
		);
}

static  void ctf_top_idle(void)
{
	u8_t cpu = 1;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_IDLE),
		cpu
		);
}

static void ctf_top_void(u32_t id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_START_CALL),
		id
		);
}

static void ctf_top_end_call(u32_t id)
{
	//return ;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_END_CALL),
		id
		);
}

void sys_trace_thread_switched_out(struct tcb_s *thread)
{
	ctf_top_thread_switched_out((u32_t)thread->pid);
}

void sys_trace_thread_switched_in(struct tcb_s *thread)
{
	ctf_top_thread_switched_in((u32_t)thread->pid);
}

void sys_trace_thread_priority_set(struct tcb_s *thread)
{
	ctf_top_thread_priority_set((u32_t)thread->pid,
				    thread->sched_priority);
}

void sys_trace_thread_create(struct tcb_s *thread)
{
	ctf_bounded_string_t name = { "Unnamed thread" };

#if CONFIG_TASK_NAME_SIZE > 0
	const char *tname = thread->name;

	if (tname != NULL) {
		strncpy(name.buf, tname, sizeof(name.buf));
		/* strncpy may not always null-terminate */
		name.buf[sizeof(name.buf) - 1] = 0;
	}
#endif

	ctf_top_thread_create(
		(u32_t)thread->pid,
		thread->init_priority,
		name
		);

#if defined(CONFIG_THREAD_STACK_INFO)
	ctf_top_thread_info(
		(u32_t)thread->pid,
		(u32_t)(uintptr_t)thread->adj_stack_ptr,
		thread->adj_stack_size
		);
#endif
}

void sys_trace_thread_abort(struct tcb_s *thread)
{
	ctf_top_thread_abort((u32_t)thread->pid);
}

void sys_trace_thread_suspend(struct tcb_s *thread)
{
	ctf_top_thread_suspend((u32_t)thread->pid);
}

void sys_trace_thread_resume(struct tcb_s *thread)
{
	ctf_top_thread_resume((u32_t)thread->pid);
}

void sys_trace_thread_ready(struct tcb_s *thread)
{
	ctf_top_thread_ready((u32_t)thread->pid);
}

void sys_trace_thread_pend(struct tcb_s *thread)
{
	ctf_top_thread_pend((u32_t)thread->pid);
}

void sys_trace_thread_info(struct tcb_s *thread)
{
	ctf_top_thread_info(
		(u32_t)thread->pid,
		(u32_t)(uintptr_t)thread->adj_stack_ptr,
		thread->adj_stack_size
		);
}

void sys_trace_thread_name_set(struct tcb_s *thread)
{
#if CONFIG_TASK_NAME_SIZE > 0
	ctf_bounded_string_t name = { "Unnamed thread" };
	const char *tname =  thread->name;

	if (tname != NULL) {
		strncpy(name.buf, tname, sizeof(name.buf));
		/* strncpy may not always null-terminate */
		name.buf[sizeof(name.buf) - 1] = 0;
	}
	ctf_top_thread_name_set(
		(u32_t)thread->pid,
		name
		);
#endif
}

void sys_trace_isr_enter(void)
{
	ctf_top_isr_enter();
}

void sys_trace_isr_exit(void)
{
	ctf_top_isr_exit();
}

void sys_trace_isr_exit_to_scheduler(void)
{
	ctf_top_isr_exit_to_scheduler();
}

void sys_trace_idle(void)
{
	ctf_top_idle();
}

void sys_trace_void(unsigned int id)
{
	ctf_top_void(id);
}

void sys_trace_end_call(unsigned int id)
{
	ctf_top_end_call(id);
}
