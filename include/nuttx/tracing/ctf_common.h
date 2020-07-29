#ifndef __TRACING_CTF_COMMON_H__
#define __TRACING_CTF_COMMON_H__

#include <nuttx/tracing/tracing_common.h>
#include <nuttx/tracing/ctf_types.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>

#include <utils.h>

#define ARG_UNUSED(x) (void)(x)

int irq_lock(void);

void irq_unlock(int key);

bool z_is_idle_thread_object(struct tcb_s *thread);

#endif //__TRACING_CTF_COMMON_H_
