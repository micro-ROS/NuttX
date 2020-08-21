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

#define CTF_EVENT(...)							    \
	{								    \
		const u64_t tstamp = tracing_get_counter_value();	    \
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

typedef enum {
	CTF_EVENT_THREAD_SWITCHED_OUT   =  0x10,
	CTF_EVENT_THREAD_SWITCHED_IN    =  0x11,
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
	CTF_EVENT_ID_STACK_USAGE        =  0x50,
	CTF_EVENT_ID_FUNC_USAGE         =  0x51,
	CTF_EVENT_ID_HEAP_ALLOC         =  0x60,
	CTF_EVENT_ID_HEAP_FREE          =  0x61,
	CTF_EVENT_ID_COM_START          =  0x70,
	CTF_EVENT_ID_COM_FINISH         =  0x71,
	CTF_EVENT_ID_COM_RECV_PKT       =  0x72,
	CTF_EVENT_ID_COM_SEND_PKT       =  0x73,
	CTF_EVENT_ID_CTF_TIMER_START    =  0xA0,
	CTF_EVENT_ID_CTF_TIMER_STOP     =  0xA1,
	CTF_EVENT_ID_CTF_MANTIMER_START =  0xA2,
} ctf_event_t;

typedef enum {
	FUNCTION_ENTER = 0U,
	FUNCTION_EXIT = 1U,
} func_usage_t;


typedef enum {
	COM_TX = 0U,
	COM_RX = 1U
} com_start_end_src_t;

typedef struct {
	char buf[CTF_MAX_STRING_LEN];
} ctf_bounded_string_t;


static  void ctf_top_thread_switched_out(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_SWITCHED_OUT),
		thread_id
		);
#else
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_switched_in(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_SWITCHED_IN),
		thread_id
		);
#else
	(void)thread_id;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_priority_set(u32_t thread_id, s8_t prio)
{

#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_PRIORITY_SET),
		thread_id,
		prio
		);
#else
	(void)thread_id;
	(void) prio;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
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
	//	prio
}

static  void ctf_top_thread_abort(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_ABORT),
		thread_id
		);
#else
	(void)thread_id;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_suspend(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_SUSPEND),
		thread_id
		);
#else
	(void)thread_id;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_resume(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_RESUME),
		thread_id
		);
#else
	(void)thread_id;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_ready(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_READY),
		thread_id
		);
#else
	(void)thread_id;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_pend(u32_t thread_id)
{
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_PENDING),
		thread_id
		);
#else
	(void)thread_id;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_thread_info(
	u32_t thread_id,
	u32_t stack_base,
	u32_t stack_size
	)
{
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
#ifdef CONFIG_TRACE_CTF_SCHEDULER_INFO
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_THREAD_NAME_SET),
		thread_id,
		name
		);
#else
	(void)thread_id;
	(void)name;
#endif //CONFIG_TRACE_CTF_SCHEDULER_INFO
}

static  void ctf_top_isr_enter(uint32_t isr_id)
{
#ifdef CONFIG_TRACE_CTF_IRQ
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ISR_ENTER),
		isr_id
		);
#else
	(void)isr_id;
#endif //CONFIG_TRACE_CTF_IRQ
}

static  void ctf_top_isr_exit(uint32_t isr_id)
{
#ifdef CONFIG_TRACE_CTF_IRQ
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ISR_EXIT),
		isr_id
		);
#else
	(void)isr_id;
#endif //CONFIG_TRACE_CTF_IRQ
}

static  void ctf_top_isr_exit_to_scheduler(void)
{
#if 0
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ISR_EXIT_TO_SCHEDULER)
		);
#endif
}

static  void ctf_top_idle(void)
{
#ifdef CONFIG_TRACE_FUNCTIONS_CPU_USAGE
	u8_t cpu = 1;
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_IDLE),
		cpu
		);
#endif // CONFIG_TRACE_FUNCTIONS_CPU_USAGE
}

static void ctf_top_void(u32_t id)
{
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_START_CALL),
		id
		);
}

static void ctf_top_end_call(u32_t id)
{
#if 0
	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_END_CALL),
		id
		);
#endif
}

static void ctf_top_malloc(void *ptr, uint32_t real_size, uint32_t user_size)
{
#ifdef CONFIG_TRACE_CTF_MEMORY_DYNAMIC_INFO
	struct tcb_s *proc = sched_self();
	uint32_t pid = 0;

	if (proc) {
		pid = proc->pid;
	}

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_HEAP_ALLOC),
		pid,
		ptr,
		real_size,
		user_size
		);
#else
	(void)ptr;
	(void)real_size;
	(void)user_size;
#endif // CONFIG_TRACE_CTF_MEMORY_DYNAMIC_INFO
}

static void ctf_top_free(void *ptr, uint32_t real_size, uint32_t user_size)
{
#ifdef CONFIG_TRACE_CTF_MEMORY_DYNAMIC_INFO
	struct tcb_s *proc = sched_self();
	uint32_t pid = 0;

	if (proc) {
		pid = proc->pid;
	}

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_HEAP_FREE),
		pid,
		ptr,
		real_size,
		user_size
		);
#else
	(void)ptr;
	(void)real_size;
	(void)user_size;
#endif //CONFIG_TRACE_CTF_MEMORY_DYNAMIC_INFO
}

static void ctf_top_func_usage(void *func, func_usage_t fe)
{
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	struct tcb_s *proc = sched_self();
	uint32_t pid = 0;

	if (proc) {
		pid = proc->pid;
	}

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_FUNC_USAGE),
		pid, func, fe
		);
#else
	(void)func;
	(void)fe;
#endif
}

static void ctf_top_stack_usage(void *func, uint32_t size)
{
#ifdef CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
	struct tcb_s *proc = sched_self();
	uint32_t pid = 0;

	if (proc) {
		pid = proc->pid;
	}

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_STACK_USAGE),
		pid, func, size 
		);
#else
	(void)func;
	(void)size;
#endif //CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
}

static void ctf_top_com_pkt_tx(const char *iface, const uint8_t *pkt,
	       	uint32_t pkt_size)
{
#ifdef CONFIG_TRACE_CTF_COM_PACKETS
	uint8_t packet[64] = { 0 };
	uint32_t size = pkt_size < sizeof(packet) ? pkt_size : sizeof(packet);
	ctf_bounded_string_t name = { "unk_if" };

	if (pkt != NULL) {
		memcpy(packet, pkt, size);
	}

	if (iface != NULL) {
		strncpy(name.buf, iface, sizeof(name.buf));
		/* strncpy may not always null-terminate */
		name.buf[sizeof(name.buf) - 1] = 0;
	}

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_COM_SEND_PKT),
		 name, pkt, size 
		);
#else
	(void) iface;
	(void) pkt;
	(void) pkt_size;
#endif// CONFIG_TRACE_CTF_COM_PACKETS

}

static void ctf_top_com_pkt_rx(const char *iface, const uint8_t *pkt, 
	       	uint32_t pkt_size)
{
#ifdef CONFIG_TRACE_CTF_COM_PACKETS
	uint8_t packet[64] = { 0 };
	uint32_t size = pkt_size < sizeof(packet) ? pkt_size : sizeof(packet);
	ctf_bounded_string_t name = { "unk_if" };

	if (pkt != NULL) {
		memcpy(packet, pkt, size);
	}


	if (iface != NULL) {
		strncpy(name.buf, iface, sizeof(name.buf));
		/* strncpy may not always null-terminate */
		name.buf[sizeof(name.buf) - 1] = 0;
	}

	CTF_EVENT(
		CTF_LITERAL(u8_t, CTF_EVENT_ID_COM_RECV_PKT),
		 name, pkt, size 
		);
#else
	(void) iface;
	(void) pkt;
	(void) pkt_size;
#endif// CONFIG_TRACE_CTF_COM_PACKETS
}

static void ctf_top_com_usage(const char *iface, uint32_t pkt_size, 
		com_start_end_src_t src, uint8_t is_start)
{
#ifdef CONFIG_TRACE_CTF_COM_USAGE
	ctf_bounded_string_t name = { "unkfac" };

	if (iface != NULL) {
		strncpy(name.buf, iface, sizeof(name.buf));
		/* strncpy may not always null-terminate */
		name.buf[sizeof(name.buf) - 1] = 0;
	}

	if (is_start) { 
		CTF_EVENT(CTF_LITERAL(u8_t, CTF_EVENT_ID_COM_START),
				name, pkt_size, src	
			 );
	} else {
		CTF_EVENT(CTF_LITERAL(u8_t, CTF_EVENT_ID_COM_FINISH),
				name, src	
			 );
	}
#else
	(void) iface;	
	(void) pkt_size;	
	(void) src;	
	(void) is_start;	
#endif //CONFIG_TRACE_CTF_COM_USAGE
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
		strncpy(name.buf, tname, sizeof(name.buf) - 1);
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

void sys_trace_isr_enter(uint32_t isr_id)
{
	ctf_top_isr_enter(isr_id);
}

void sys_trace_isr_exit(uint32_t isr_id)
{
	ctf_top_isr_exit(isr_id);
}

void sys_trace_isr_exit_to_scheduler(void)
{
	ctf_top_isr_exit_to_scheduler();
}

void sys_trace_idle(void)
{
	ctf_top_idle();
}

// Malloc calls
void sys_trace_memory_dynamic_allocate(struct mm_heap_s *heap, void *ptr, uint32_t real_size, uint32_t user_size)
{ 
	ctf_top_malloc(ptr, real_size, user_size);
}

void sys_trace_memory_dynamic_free(struct mm_heap_s *heap, void *ptr, uint32_t real_size, uint32_t user_size)
{ 
	ctf_top_free(ptr, real_size, user_size);
}

void sys_trace_func_usage_enter(void *func)
{ 
	ctf_top_func_usage(func, FUNCTION_ENTER);
}

void sys_trace_func_usage_exit(void *func)
{ 
	ctf_top_func_usage(func, FUNCTION_EXIT);
}

void sys_trace_memory_static_alloc(void *func, uint32_t size)
{ 
	ctf_top_stack_usage(func, size);
}

void sys_trace_com_start(const char *iface, uint32_t pkt_size, uint8_t is_rx)
{
	ctf_top_com_usage(iface, pkt_size, (com_start_end_src_t)is_rx, 1);
}

void sys_trace_com_finish(const char *iface, uint8_t is_rx)
{
	ctf_top_com_usage(iface, 0, (com_start_end_src_t)is_rx, 0);
}

void sys_trace_com_pkt(const char *iface, uint8_t *pkt, uint32_t pkt_size, uint8_t is_rx)
{
	if (is_rx) {
		ctf_top_com_pkt_tx(iface, pkt, pkt_size);
	} else {
		ctf_top_com_pkt_rx(iface, pkt, pkt_size);
	}
}
