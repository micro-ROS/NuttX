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

#define ASSERT(x,y)
#define __ASSERT(x,y)

#define __used		__attribute__((__used__))

#define ___in_section(a, b, c) \
	__attribute__((section("." Z_STRINGIFY(a)			\
				"." Z_STRINGIFY(b)			\
				"." Z_STRINGIFY(c))))
#define __in_section(a, b, c) ___in_section(a, b, c)

#define IS_ENABLED(x) 0
#define SYS_INIT(x, y, z)

#define Z_STRINGIFY(x) #x
#define STRINGIFY(s) Z_STRINGIFY(s)

/* concatenate the values of the arguments into one */
#define _DO_CONCAT(x, y) x ## y
#define _CONCAT(x, y) _DO_CONCAT(x, y)

/*
 * This is meant to be used in conjunction with __in_section() and similar
 * where scattered structure instances are concatened together by the linker
 * and walked by the code at run time just like a contiguous array of such
 * structures.
 *
 * Assemblers and linkers may insert alignment padding by default whose
 * size is larger than the natural alignment for those structures when
 * gathering various section segments together, messing up the array walk.
 * To prevent this, we need to provide an explicit alignment not to rely
 * on the default that might just work by luck.
 *
 * Alignment statements in  linker scripts are not sufficient as
 * the assembler may add padding by itself to each segment when switching
 * between sections within the same file even if it merges many such segments
 * into a single section in the end.
 */
#define Z_DECL_ALIGN(type) __attribute__((aligned(__alignof__(type)))) type

/*
 * Convenience helper combining __in_section() and Z_DECL_ALIGN().
 * The section name is the struct type prepended with an underscore.
 * The subsection is "static" and the subsubsection is the variable name.
 */
#define Z_STRUCT_SECTION_ITERABLE(struct_type, name) \
	Z_DECL_ALIGN(struct struct_type) name \
	__in_section(_##struct_type, static, name) __used

/*
 * Itterator for structure instances gathered by Z_STRUCT_SECTION_ITERABLE().
 * The linker must provide a _<struct_type>_list_start symbol and a
 * _<struct_type>_list_end symbol to mark the start and the end of the
 * list of struct objects to iterate over.
 */
#define Z_STRUCT_SECTION_FOREACH(struct_type, iterator) \
	extern struct struct_type _CONCAT(_##struct_type, _list_start)[]; \
	extern struct struct_type _CONCAT(_##struct_type, _list_end)[]; \
	for (struct struct_type *iterator = \
			_CONCAT(_##struct_type, _list_start); \
	     ({ __ASSERT(iterator <= _CONCAT(_##struct_type, _list_end), \
			 "unexpected list end location"); \
		iterator < _CONCAT(_##struct_type, _list_end); }); \
	     iterator++)


/**
 * @brief Tracing backend
 * @defgroup Tracing_backend Tracing backend
 * @{
 * @}
 */

struct tracing_backend;

/**
 * @brief Tracing backend API.
 */
struct tracing_backend_api {
	void (*init)(void);
	void (*output)(const struct tracing_backend *backend,
		       u8_t *data, u32_t length);
};

/**
 * @brief Tracing backend structure.
 */
struct tracing_backend {
	const char *name;
	const struct tracing_backend_api *api;
};

/**
 * @brief Create tracing_backend instance.
 *
 * @param _name Instance name.
 * @param _api  Tracing backend API.
 */
#define TRACING_BACKEND_DEFINE(_name, _api)                              \
	static const Z_STRUCT_SECTION_ITERABLE(tracing_backend, _name) = \
	{                                                                \
		.name = STRINGIFY(_name),                                \
		.api = &_api                                             \
	}

/**
 * @brief Initialize tracing backend.
 *
 * @param backend Pointer to tracing_backend instance.
 */
void tracing_backend_init(
		const struct tracing_backend *backend);

/**
 * @brief Output tracing packet with tracing backend.
 *
 * @param backend Pointer to tracing_backend instance.
 * @param data    Address of outputing buffer.
 * @param length  Length of outputing buffer.
 */
void tracing_backend_output(
		const struct tracing_backend *backend,
		u8_t *data, u32_t length);

/**
 * @brief Get tracing backend based on the name of
 *        tracing backend in tracing backend section.
 *
 * @param name Name of wanted tracing backend.
 *
 * @return Pointer of the wanted backend or NULL.
 */
struct tracing_backend *tracing_backend_get(char *name);

#ifdef CONFIG_TRACE_USE_NOCTF
int cpu_stats_log_init(void);
#else //CONFIG_TRACE_USE_NOCTF
#define cpu_stats_log_init()
#endif //CONFIG_TRACE_USE_NOCTF

#ifdef CONFIG_TRACE_USE_CTF
int tracing_init(void);
#else //CONFIG_TRACE_USE_CTF
#define tracing_init()
#endif //CONFIG_TRACE_USE_CTF

#ifdef __cplusplus
}
#endif

#endif //__NUTTX_TRACING_TRACING_COMMON_H__
