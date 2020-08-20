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
void sys_trace_idle(void);

void cpu_stats_get_ns(struct cpu_stats *cpu_stats_ns);
void cpu_stats_reset_counters(void);

#define sys_trace_isr_exit_to_scheduler()
#define sys_trace_isr_enter(x)
#define sys_trace_isr_exit(x)

#define cpu_stats_non_idle_and_sched_get_percent(x)

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

/** Send/recv packet (rather used for wireshark later?) network/serial/usb interface */
#define sys_trace_packet_sent(interface, buf, size)
#define sys_trace_packet_rcvd(interface, buf, size)

/** Send/recv latency network/serial/usb interface */
#define sys_trace_latency_start(interface, buf, size)
#define sys_trace_latency_end(interface, buf, size)

/** Send/recv bandwitdh used */
#define sys_trace_bw_up(interface, size)
#define sys_trace_bw_down(interface, size)

/** Function stack pointer usage */
#define sys_trace_fenter_stack_usage(thread, func, size)

/** Function calls tracing */
#define sys_trace_fenter(thread, fn)

/** Function calls tracing specific */
#define sys_trace_fenter_timer(thread)
#define sys_trace_fexit_timer(thread)

#ifdef __cplusplus
}
#endif

#elif defined(CONFIG_TRACE_USE_CTF)

/** Send/recv packet (rather used for wireshark later?) network/serial/usb interface */
void sys_trace_packet_sent(const char *interface, const char *buf, uint32_t size) __attribute__((no_instrument_function));
void sys_trace_packet_rcvd(const char *interface, const char *buf, uint32_t size) __attribute__((no_instrument_function));


/** Send/recv latency network/serial/usb interface */
void sys_trace_com_pkt(const char *iface, uint8_t *pkt, uint32_t pkt_size, uint8_t is_rx) __attribute__((no_instrument_function));
void sys_trace_com_start(const char *iface, uint32_t pkt_size, uint8_t is_rx) __attribute__((no_instrument_function));
void sys_trace_com_finish(const char *iface, uint8_t is_rx) __attribute__((no_instrument_function));

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
void sys_trace_memory_dynamic_free(void *ptr) __attribute__((no_instrument_function));
void sys_trace_memory_dynamic_allocate(void *ptr, uint32_t size) __attribute__((no_instrument_function));
void sys_trace_memory_static_alloc(void *func, uint32_t size) __attribute__((no_instrument_function));

/** Function calls tracing */
void sys_trace_func_usage_enter(void *func) __attribute__((no_instrument_function));
void sys_trace_func_usage_exit(void *func) __attribute__((no_instrument_function));


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

/** Send/recv packet (rather used for wireshark later?) network/serial/usb interface */
#define sys_trace_packet_sent(interface, buf, size)
#define sys_trace_packet_rcvd(interface, buf, size)

/** Send/recv latency network/serial/usb interface */
#define sys_trace_latency_start(interface, buf, size)
#define sys_trace_latency_end(interface, buf, size)

/** Send/recv bandwitdh used */
#define sys_trace_bw_up(interface, size)
#define sys_trace_bw_down(interface, size)

/** Function stack pointer usage */
#define sys_trace_fenter_stack_usage(thread, func, size)

/** Function calls tracing */
#define sys_trace_fenter(thread, fn)

/** Function calls tracing specific */
#define sys_trace_fenter_timer(thread)
#define sys_trace_fexit_timer(thread)
#endif /* CONFIG_CTF_TRACE_USE_CTF */

#endif /* __TRRACING_PROBES_H__ */
