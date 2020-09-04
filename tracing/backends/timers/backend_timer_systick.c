#include <nuttx/clock.h>

#include <backend_timer.h>
#include <utils.h>

#include <time.h>

int32_t backend_timer_systick_init(void)
{
	return 0;
}

uint64_t backend_timer_systick_gettime(const struct backend_timer *btimer)
{
	return (uint64_t) g_system_timer;
}	

static const struct backend_timer_api backend_timer_systick_api = {
	.init = backend_timer_systick_init,
	.gettime = backend_timer_systick_gettime,
};

BACKEND_TIMER_DEFINE(backend_timer_systick, backend_timer_systick_api);
