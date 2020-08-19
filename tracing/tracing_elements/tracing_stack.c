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
#ifdef CONFIG_TRACE_STACK_USAGE
	register void *sp asm ("sp");
	static uint32_t maxusage = 0;
	uint32_t sp_ptr = (uint32_t)(uintptr_t) sp;

	sp_ptr = thread->adj_stack_ptr - sp_ptr;
	if (maxusage < sp_ptr) {
		sys_trace_stack_usage(this_fn, sp_ptr);
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
#endif 
#if 0
	sys_trace_fenter_timer(thread, this_fn);
#endif

}

void __cyg_profile_func_exit (void *this_fn,
	       	void *call_site)
{
#ifdef CONFIG_TRACE_CTF_FUNCTIONS_USAGE
	sys_trace_func_usage_exit(this_fn);
#endif
}
