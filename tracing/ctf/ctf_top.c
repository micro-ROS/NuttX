/*
 * Copyright (c) 2018 Oticon A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nuttx/tracing/tracing_probes.h>

void sys_trace_thread_switched_out(void)
{
	struct tcb_s *thread = sched_self();
	ctf_top_thread_switched_out((u32_t)thread->pid);
}

void sys_trace_thread_switched_in(void)
{
	struct tcb_s *thread = sched_self();
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
