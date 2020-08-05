/*
 * Copyright (c) 2018 Oticon A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TRRACING_PROBES_H__
#define __TRRACING_PROBES_H__

#include <nuttx/config.h>
#include <nuttx/tracing/ctf_types.h>
#include <nuttx/sched.h>

#if defined(CONFIG_TRACE_USE_NOCTF)
#ifdef __cplusplus
extern "C" {
#endif

struct cpu_stats {
	u64_t idle;
	u64_t non_idle;
	u64_t sched;
};

void sys_trace_thread_switched_in(struct tcb_s *thread);
void sys_trace_thread_switched_out(struct tcb_s *thread);
void sys_trace_isr_enter(void);
void sys_trace_isr_exit(void);
void sys_trace_idle(void);

void cpu_stats_get_ns(struct cpu_stats *cpu_stats_ns);
u32_t cpu_stats_non_idle_and_sched_get_percent(void);
void cpu_stats_reset_counters(void);

#define sys_trace_isr_exit_to_scheduler()

#define sys_trace_thread_priority_set(thread)
#define sys_trace_thread_info(thread)
#define sys_trace_thread_create(thread)
#define sys_trace_thread_abort(thread)
#define sys_trace_thread_suspend(thread)
#define sys_trace_thread_resume(thread)
#define sys_trace_thread_ready(thread)
#define sys_trace_thread_pend(thread)
#define sys_trace_thread_name_set(thread)

#define sys_trace_void(id)
#define sys_trace_end_call(id)

#ifdef __cplusplus
}
#endif

#elif defined(CONFIG_TRACE_USE_CTF)

void sys_trace_thread_abort(struct tcb_s *thread);
void sys_trace_thread_suspend(struct tcb_s *thread);
void sys_trace_thread_switched(struct tcb_s *prev, struct tcb_s *next);
void sys_trace_thread_switched_in(struct tcb_s *thread);
void sys_trace_thread_switched_out(struct tcb_s *thread);
void sys_trace_isr_enter(void);
void sys_trace_isr_exit(void);
void sys_trace_idle(void);
void sys_trace_void(unsigned int id);
void sys_trace_end_call(unsigned int id);

#else

#define sys_trace_thread_switched_out()
#define sys_trace_thread_switched_in()
#define sys_trace_thread_priority_set(x)
#define sys_trace_thread_create(x)
#define sys_trace_thread_abort(x)
#define sys_trace_thread_suspend(x)
#define sys_trace_thread_resume(x)
#define sys_trace_thread_ready(x)
#define sys_trace_thread_pend(x)
#define sys_trace_thread_info(x)
#define sys_trace_thread_name_set(x)
#define sys_trace_isr_enter()
#define sys_trace_isr_exit()
#define sys_trace_isr_exit_to_scheduler()
#define sys_trace_idle()
#define sys_trace_(x)
#define sys_trace_end_call(x)

#endif /* CONFIG_CTF_TRACE_USE_CTF */

#endif /* __TRRACING_PROBES_H__ */
