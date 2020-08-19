#include <nuttx/config.h>
#include <nuttx/tracing/tracing_probes.h>
#include <tracing_core.h>
#include <atomic.h>
#include <stdint.h>
#include <sched.h>
#include <stdbool.h>

#define STACK_SIZE_DATA

void __cyg_profile_func_enter (void *, void *) __attribute__((no_instrument_function));
void __cyg_profile_func_exit (void *, void *) __attribute__((no_instrument_function));
extern bool is_tracing_enabled (void) __attribute__((no_instrument_function));
extern struct tcb_s * sched_self(void) __attribute__((no_instrument_function));
extern void sys_trace_memory_static_alloc (void*, uint32_t) __attribute__((no_instrument_function));
extern atomic_val_t atomic_get 
	(const atomic_t *target) __attribute__((no_instrument_function));
extern void *memcpy(FAR void *, FAR const void *, size_t) __attribute__((no_instrument_function));

struct tracing_stack_func {
	bool is_tracing;
} tracing_ctx [64];

void __cyg_profile_func_enter(void *this_fn, void *call_site)
{
	uint32_t pid;
	if (!is_tracing_enabled()) {
		return;
	}

#if 0
	asm volatile (
			"   mrs r2, ipsr        \n" /* Check whether we are in interrupt mode */
			"   cmp r2, #0          \n" /* since we don't switch r10 on interrupt entry, we */
			"   bne 1f              \n" /* can't detect overflow of the interrupt stack. */
			"1:                     \n"
			"   bx lr               \n"
		     );
#endif



	struct tcb_s *thread = sched_self();
	if (!thread || thread == 0xffffffff)
		pid = 63;
	else  {
		if (pid > 62) {
			return;
		}
		pid = thread->pid;
	}

	if (tracing_ctx[pid].is_tracing) {
		return;
	}

#ifdef CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
	static uint32_t reached_max = 0;
	static uint32_t call_count = 0;
	register void *sp asm ("sp");
	uint32_t sp_ptr = (uint32_t)(uintptr_t) sp;

	sp_ptr = ((uint32_t) (uintptr_t)thread->adj_stack_ptr) - sp_ptr;

#if 0
	if  (!(++calls_cnt % CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING)) {
		sys_trace_stack_usage(thread, this_fn, sp_ptr);
	} 
#endif
	if (reached_max < sp_ptr) {
		tracing_ctx[pid].is_tracing = true;
		sys_trace_memory_static_alloc(this_fn, sp_ptr);
		reached_max = sp_ptr;
		tracing_ctx[pid].is_tracing = false;
	}
#endif // CONFIG_TRACE_CTF_MEMORY_STATIC_INFO

#if 0



	for (int i = (int) pid_backtrace[pid].count; i > -1 ; i--) {
	 	if (&sys_trace_memory_static_alloc == pid_backtrace[i].address[i]) {
			irq_unlock(key);
			return ;
		}

	 	if (&__cyg_profile_func_exit == pid_backtrace[i].address[i]) {
			irq_unlock(key);
			return ;
		}
	}

	pid_backtrace[pid].address[pid_backtrace[pid].count] = call_site;
	pid_backtrace[pid].count++;
	register void *sp asm ("sp");
	static uint32_t maxusage = 0;
	uint32_t sp_ptr = (uint32_t)(uintptr_t) sp;

	sp_ptr = ((uint32_t) (uintptr_t)thread->adj_stack_ptr) - sp_ptr;
	if (maxusage < sp_ptr) {
		sys_trace_memory_static_alloc(this_fn, sp_ptr);
		maxusage = sp_ptr;
	}
#if 0
	if (max_size < sp_ptr 
	    || !(++calls_cnt % CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING)) {
		sys_trace_stack_usage(thread, this_fn, sp_ptr);
	}
#endif
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	 sys_trace_func_usage_enter(this_fn);
#endif // CONFIG_TRACE_CTF_FUNCTIONS_USAGE

	irq_unlock(key);
#endif
}

void __cyg_profile_func_exit (void *this_fn,
	       	void *call_site)
{
	asm volatile (
			"   mrs r2, ipsr        \n" /* Check whether we are in interrupt mode */
			"   cmp r2, #0          \n" /* since we don't switch r10 on interrupt entry, we */
			"   bne 1f              \n"
			"1:                     \n"
			"   bx lr               \n"
		     );

#ifdef CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
#endif //CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	sys_trace_func_usage_exit(this_fn);
#endif
}
