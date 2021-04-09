#include <backend_timer.h>
#include <utils.h>
#include <stdio.h>

#include <time.h>
#include <stm32_freerun.h>
#include <stdbool.h>

#include <nuttx/clock.h>

#include "stm32_freerun.h"

#ifndef  CONFIG_STM32_FREERUN
#error "config CONFIG_STM32_FREERUN needs to be selected"
#endif

static struct stm32_freerun_s g_freerun;

static bool g_initialised = false;

static int32_t backend_timer_freerunner_init(void)
{
	int32_t rc = stm32_freerun_initialize(&g_freerun, 
			CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER_ID, 
			CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER_RES_US);
	if (!rc) {
		g_initialised = true;
		return 0;
	}
	
	return -1;
}

static uint64_t backend_timer_freerunner_gettime(const struct backend_timer *btimer)
{
	struct timespec time;

	if (!g_initialised) {
		return 0;
	}
	/* Use the final free runing counter for finer timer */
	stm32_freerun_counter(&g_freerun, &time);
	return time.tv_nsec + (uint64_t)time.tv_sec * NSEC_PER_SEC;
}	

static const struct backend_timer_api backend_timer_freerunner_api = {
	.init = backend_timer_freerunner_init,
	.gettime = backend_timer_freerunner_gettime,
};

BACKEND_TIMER_DEFINE(backend_timer_freerunner, backend_timer_freerunner_api);
