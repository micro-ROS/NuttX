/*
 * Copyright (c) 2019 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __NUTTX_TRACING_TRACING_COMMON_H__
#define __NUTTX_TRACING_TRACING_COMMON_H__

#include <string.h>
#include <nuttx/tracing/ctf_types.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef CONFIG_TRACE_USE_NOCTF

int cpu_stats_log_init(void);

#else //CONFIG_TRACE_USE_NOCTF

#define cpu_stats_log_init()

#endif //CONFIG_TRACE_USE_NOCTF

#ifdef CONFIG_TRACE_USE_CTF

int tracing_init(void);
int tracing_finish(void);

#else //CONFIG_TRACE_USE_CTF

#define tracing_init()

#endif //CONFIG_TRACE_USE_CTF

#ifdef __cplusplus
}
#endif

#endif //__NUTTX_TRACING_TRACING_COMMON_H__
