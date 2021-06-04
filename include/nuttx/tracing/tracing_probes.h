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
#include <nuttx/mm/mm.h>

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
void sys_trace_idle(void);


void cpu_stats_get_ns(struct cpu_stats *cpu_stats_ns);
void cpu_stats_reset_counters(void);
uint32_t cpu_stats_non_idle_and_sched_get_percent(void);
void sys_trace_isr_enter(uint32_t isr);
void sys_trace_isr_exit(uint32_t isr);

#define sys_trace_ctf_meas_pwr()

#define sys_trace_isr_exit_to_scheduler()

#define ctf_top_ctf_meas_stop()
#define ctf_top_ctf_meas_start()

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
#define sys_trace_com_pkt(iface, pkt, pkt_size, is_rx)
#define sys_trace_com_start(iface, is_rx)
#define sys_trace_com_finish(iface, pkt_size, is_rx)
#define  sys_trace_ctf_meas_start()
#define  sys_trace_ctf_meas_stop()

/** Function stack pointer usage */
#define sys_trace_fenter_stack_usage(thread, func, size)

/** Function calls tracing */
#define sys_trace_fenter(thread, fn)

/** Function calls tracing specific */
#define sys_trace_fenter_timer(thread)
#define sys_trace_fexit_timer(thread)


/** Function stack and heap usage */
#define sys_trace_memory_dynamic_free(bt, heap, ptr, real_size, user_size)
#define sys_trace_memory_dynamic_allocate(bt, heap, ptr,real_size, user_size, requested)
#define sys_trace_memory_static_alloc(func, size)

/** Function calls tracing */
#define sys_trace_func_usage_enter(func)
#define sys_trace_func_usage_exit(func)

#ifdef __cplusplus
}
#endif

#elif defined(CONFIG_TRACE_USE_CTF)

/** Send/recv packet (rather used for wireshark later?) network/serial/usb interface */
void sys_trace_packet_sent(const char *interface, const char *buf, uint32_t size) __attribute__((no_instrument_function));
void sys_trace_packet_rcvd(const char *interface, const char *buf, uint32_t size) __attribute__((no_instrument_function));


/** Send/recv latency network/serial/usb interface */
void sys_trace_com_pkt(const char *iface, uint8_t *pkt, uint32_t pkt_size, uint8_t is_rx) __attribute__((no_instrument_function));
void sys_trace_com_start(const char *iface, uint8_t is_rx) __attribute__((no_instrument_function));
void sys_trace_com_finish(const char *iface, uint32_t pkt_size, uint8_t is_rx) __attribute__((no_instrument_function));

/** Thread entering/exiting etc */
void sys_trace_thread_abort(struct tcb_s *thread) __attribute__((no_instrument_function));
void sys_trace_thread_suspend(struct tcb_s *thread) __attribute__((no_instrument_function));
void sys_trace_thread_switched(struct tcb_s *prev, struct tcb_s *next) __attribute__((no_instrument_function));

void sys_trace_thread_switched_in(struct tcb_s *thread) __attribute__((no_instrument_function));

void sys_trace_thread_switched_out(struct tcb_s *thread) __attribute__((no_instrument_function));

/** Scheduler realtimeness */
void sys_trace_thread_preempt_start(struct tcb_s *thread_old) __attribute__((no_instrument_function));
void sys_trace_thread_preempt_stop(struct tcb_s *thread_new) __attribute__((no_instrument_function));

/** Function IRQ entering realtimness */
void sys_trace_isr_enter(uint32_t isr_id) __attribute__((no_instrument_function));
void sys_trace_isr_exit(uint32_t isr_id) __attribute__((no_instrument_function));

/** Maybe useless */
void sys_trace_idle(void) __attribute__((no_instrument_function));

void sys_trace_void(unsigned int id) __attribute__((no_instrument_function));
void sys_trace_end_call(unsigned int id) __attribute__((no_instrument_function));


/** Function stack and heap usage */
void sys_trace_memory_dynamic_free(struct mm_heap_s *heap, void *ptr, uint32_t real_size, uint32_t user_size) __attribute__((no_instrument_function));
void sys_trace_memory_dynamic_allocate(void **bt, struct mm_heap_s *heap, void *ptr, uint32_t real_size, uint32_t user_size, uint32_t requested) __attribute__((no_instrument_function));
void sys_trace_memory_static_alloc(void *func, uint32_t size) __attribute__((no_instrument_function));

/** Function calls tracing */
void sys_trace_func_usage_enter(void *func) __attribute__((no_instrument_function));
void sys_trace_func_usage_exit(void *func) __attribute__((no_instrument_function));

/** CTF timer trace */
void sys_trace_ctf_timer_start(uint32_t tid, const char *func_name,
		uint32_t line, void* func_ptr) __attribute__((no_instrument_function));

void sys_trace_ctf_timer_stop(uint32_t tid, const char *func_name,
		uint32_t line, void* func_ptr) __attribute__((no_instrument_function));

/** Thread create to keep track of the what's going on */
void sys_trace_thread_create(struct tcb_s *thread) __attribute__((no_instrument_function));

void sys_trace_thread_name_set(struct tcb_s *thread)  __attribute__((no_instrument_function));
void sys_trace_thread_resume(struct tcb_s *thread) __attribute__((no_instrument_function));
void sys_trace_thread_priority_set(struct tcb_s *thread)  __attribute__((no_instrument_function));

void sys_trace_ctf_meas_stop(void) __attribute__((no_instrument_function));
void sys_trace_ctf_meas_start(void) __attribute__((no_instrument_function));

void sys_trace_ctf_meas_pwr(void) __attribute__((no_instrument_function));

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
#define sys_trace_isr_enter(x)
#define sys_trace_isr_exit(x)
#define sys_trace_isr_exit_to_scheduler()
#define sys_trace_idle()
#define sys_trace_(x)
#define sys_trace_end_call(x)

#define sys_trace_com_pkt(iface, pkt, pkt_size, is_rx)
#define sys_trace_com_start(iface, is_rx)
#define sys_trace_com_finish(iface, pkt_size, is_rx)
#define sys_trace_ctf_meas_pwr()

/** Function stack pointer usage */
#define sys_trace_fenter_stack_usage(thread, func, size)

/** Function calls tracing */
#define sys_trace_fenter(thread, fn)

/** Function calls tracing specific */
#define sys_trace_fenter_timer(thread)
#define sys_trace_fexit_timer(thread)

/** Function stack and heap usage */
#define sys_trace_memory_dynamic_free(heap, ptr, real_size, user_size)
#define sys_trace_memory_dynamic_allocate(bt, heap, ptr,real_size, user_size, requested)
#define sys_trace_memory_static_alloc(func, size)

/** Function calls tracing */
#define sys_trace_func_usage_enter(func)
#define sys_trace_func_usage_exit(func)

#define  sys_trace_ctf_meas_start()
#define  sys_trace_ctf_meas_stop()

#endif /* CONFIG_CTF_TRACE_USE_CTF */

#endif /* __TRRACING_PROBES_H__ */
