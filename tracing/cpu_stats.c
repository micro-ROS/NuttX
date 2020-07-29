/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nuttx/tracing/tracing_cpu_stats.h>
#include <stdio.h>

enum cpu_state {
	CPU_STATE_IDLE,
	CPU_STATE_NON_IDLE,
	CPU_STATE_SCHEDULER
};

static enum cpu_state last_cpu_state = CPU_STATE_SCHEDULER;
static enum cpu_state cpu_state_before_interrupts;

static u32_t last_time;
static struct cpu_stats stats_hw_tick;
static int nested_interrupts;
static struct tcb_s *current_thread;

void update_counter(volatile u64_t *cnt)
{
	u32_t time = g_system_timer;

	if (time >= last_time) {
		(*cnt) += (time - last_time);
	} else {
		(*cnt) += (UINT32_MAX - last_time + 1 + time);
	}
	last_time = time;
}

static void cpu_stats_update_counters(void)
{
	switch (last_cpu_state) {
	case CPU_STATE_IDLE:
		update_counter(&stats_hw_tick.idle);
		break;

	case CPU_STATE_NON_IDLE:
		update_counter(&stats_hw_tick.non_idle);
		break;

	case CPU_STATE_SCHEDULER:
		update_counter(&stats_hw_tick.sched);
		break;

	default:
		/* Invalid CPU state */
#if 0
		__ASSERT_NO_MSG(false);
#endif
		break;
	}
}

void cpu_stats_get_ns(struct cpu_stats *cpu_stats_ns)
{
	int key = irq_lock();

	cpu_stats_update_counters();
	cpu_stats_ns->idle = stats_hw_tick.idle * NSEC_PER_TICK;
	cpu_stats_ns->non_idle = stats_hw_tick.non_idle * NSEC_PER_TICK;
	cpu_stats_ns->sched = stats_hw_tick.sched * NSEC_PER_TICK;
	irq_unlock(key);
}

u32_t cpu_stats_non_idle_and_sched_get_percent(void)
{
	int key = irq_lock();

	cpu_stats_update_counters();
	irq_unlock(key);
	return ((stats_hw_tick.non_idle + stats_hw_tick.sched) * 100) /
		(stats_hw_tick.idle + stats_hw_tick.non_idle +
		 stats_hw_tick.sched);
}

void cpu_stats_reset_counters(void)
{
	int key = irq_lock();

	stats_hw_tick.idle = 0;
	stats_hw_tick.non_idle = 0;
	stats_hw_tick.sched = 0;
	last_time = g_system_timer;
	irq_unlock(key);
}

void sys_trace_thread_switched_in(void)
{
	int key = irq_lock();

#if 0
	__ASSERT_NO_MSG(nested_interrupts == 0);
#endif
	cpu_stats_update_counters();
	current_thread = sched_self();
	if (z_is_idle_thread_object(current_thread)) {
		last_cpu_state = CPU_STATE_IDLE;
	} else {
		last_cpu_state = CPU_STATE_NON_IDLE;
	}
	irq_unlock(key);
}

void sys_trace_thread_switched_out(void)
{
	int key = irq_lock();

#if 0
	__ASSERT_NO_MSG(nested_interrupts == 0);
	__ASSERT_NO_MSG(!current_thread || (current_thread == k_current_get()));
#endif

	cpu_stats_update_counters();
	last_cpu_state = CPU_STATE_SCHEDULER;
	irq_unlock(key);
}

void sys_trace_isr_enter(void)
{
	int key = irq_lock();

	if (nested_interrupts == 0) {
		cpu_stats_update_counters();
		cpu_state_before_interrupts = last_cpu_state;
		last_cpu_state = CPU_STATE_NON_IDLE;
	}
	nested_interrupts++;
	irq_unlock(key);
}

void sys_trace_isr_exit(void)
{
	int key = irq_lock();

	nested_interrupts--;
	if (nested_interrupts == 0) {
		cpu_stats_update_counters();
		last_cpu_state = cpu_state_before_interrupts;
	}
	irq_unlock(key);
}

void sys_trace_idle(void)
{
}

static struct work_s cpu_stats_log;

static void cpu_stats_display(void)
{
	printf("CPU usage: %u\n", cpu_stats_non_idle_and_sched_get_percent());
}

static void cpu_stats_log_fn(void *item)
{
	cpu_stats_display();
	cpu_stats_reset_counters();
	work_queue(LPWORK, &cpu_stats_log, cpu_stats_log_fn, NULL, 500);
}

int cpu_stats_log_init(void)
{
	work_queue(LPWORK, &cpu_stats_log, cpu_stats_log_fn, NULL, 500);

#if 0
struct work_s
{
  struct dq_entry_s dq;  /* Implements a doubly linked list */
  worker_t  worker;      /* Work callback */
  FAR void *arg;         /* Callback argument */
  clock_t qtime;         /* Time work queued */
  clock_t delay;         /* Delay until work performed */
};
	k_delayed_work_init(&cpu_stats_log, cpu_stats_log_fn);
	k_delayed_work_submit(&cpu_stats_log,
			      K_MSEC(CONFIG_TRACING_CPU_STATS_INTERVAL));

#endif
	return 0;
}
