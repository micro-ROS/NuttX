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
	bool is_tracing_enter;
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	bool is_tracing_exit;
#endif
#ifdef CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
	uint32_t reached_max;
#endif
} tracing_ctx [64];


void __cyg_profile_func_enter(void *this_fn, void *call_site)
{
	uint32_t pid;
	if (!is_tracing_enabled()) {
		return;
	}

	if (is_tracing_thread()) {
		return;
	}

	struct tcb_s *thread = sched_self();
	if (!thread || thread == 0xffffffff)
		return;

	pid = thread->pid;
	if (!pid || pid > 63) {
		return;
	}

	if (thread->task_state != TSTATE_TASK_RUNNING) {
		return;
	}


	if (tracing_ctx[pid].is_tracing_enter) {
		return;
	}

	tracing_ctx[pid].is_tracing_enter = true;
#ifdef CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
#if CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING > 0
	static uint32_t call_count = 0;
#endif

	register void *sp asm ("sp");
	uint32_t sp_ptr = (uint32_t)(uintptr_t) sp;
	sp_ptr = ((uint32_t) (uintptr_t)thread->adj_stack_ptr) - sp_ptr;
	if (sp_ptr > (thread->adj_stack_size << 4)) {
		// Probably the scheduler doing some work withou updating the 
		// pid 
		tracing_ctx[pid].is_tracing_enter = false;
		return;		
	}

#if CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING > 0
	if  (!(++call_count % CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING)) {
		sys_trace_memory_static_alloc(this_fn, sp_ptr);
	} else
#endif //CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING > 0
	if (tracing_ctx[pid].reached_max < sp_ptr) {
		sys_trace_memory_static_alloc(this_fn, sp_ptr);
		tracing_ctx[pid].reached_max = sp_ptr;
	}
#endif // CONFIG_TRACE_CTF_MEMORY_STATIC_INFO
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	sys_trace_func_usage_enter(this_fn);
#endif // CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	tracing_ctx[pid].is_tracing_enter = false;
}

void __cyg_profile_func_exit (void *this_fn,
	       	void *call_site)
{
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	uint32_t pid;
	if (!is_tracing_enabled()) {
		return;
	}

	if (is_tracing_thread()) {
		return;
	}

	struct tcb_s *thread = sched_self();
	if (!thread || thread == 0xffffffff)
		return;

	pid = thread->pid;
	if (!pid || pid > 63) {
		return;
	}

	if (thread->task_state != TSTATE_TASK_RUNNING) {
		return;
	}


	if (tracing_ctx[pid].is_tracing_exit) {
		return;
	}

	tracing_ctx[pid].is_tracing_exit = true;
	sys_trace_func_usage_exit(this_fn);
	tracing_ctx[pid].is_tracing_exit = false;
#endif // CONFIG_TRACE_CTF_FUNCTIONS_USAGE
}
