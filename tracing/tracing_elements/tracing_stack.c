#include <tracing_extensions.h>
#include <stddint.h>
#include <nuttx/tracing_probe.h>
FAR static uint32_t max_size;
FAR static uint32_t calls_cnt;

void __cyg_profile_func_enter (void *, void *) __attribute__((no_instrument_function));
void __cyg_profile_func_exit (void *, void *) __attribute__((no_instrument_function));
extern int printf (const char *__restrict __format, ...) __attribute__((no_instrument_function));

void __cyg_profile_func_enter(void *this_fn,
	       	void *call_site)
{
	struct tcb_s *thread = sched_self();
#ifdef CONFIG_TRACE_STACK_USAGE
	register void *sp asm ("sp");
	uint32_t sp_ptr = (uint32_t)(uintptr_t) sp;

	sp_ptr = thread->adj_stack_ptr - sp_ptr;
	if (max_size < sp_ptr 
	    || !(++calls_cnt % CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING)) {
		sys_trace_stack_usage(thread, this_fn, sp_ptr);
	}
#elif defined(CONFIG_TRACE_FUNCTIONS_CALLS)
	sys_trace_fenter(thread, this_fn);
#elif defined(CONFIG_TRACE_FENTER_TIMER)
	sys_trace_fenter_timer(thread, this_fn);
#endif

}

void __cyg_profile_func_exit (void *this_fn,
	       	void *call_site)
{
	struct tcb_s *thread = sched_self();
#if defined(CONFIG_TRACE_STACK_USAGE)
	register void *sp asm ("sp");
	uint32_t sp_ptr = (uint32_t)(uintptr_t) sp;

	sp_ptr = thread->adj_stack_ptr - sp_ptr;
	if (!(++calls_cnt % CONFIG_TRACE_STACK_USAGE_CALLS_CNT_SAMPLING)) {
		sys_trace_stack_usage(thread, this_fn, sp_ptr);
	}
	sys_trace_stack_usage(thread, this_fn, sp_ptr);
#elif defined(CONFIG_TRACE_FUNCTION_TIMER)
	sys_trace_fexit_timer(thread, this_fn);
#endif
}
