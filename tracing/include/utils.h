#ifndef __TRACING_UTILS_H__
#define __TRACING_UTILS_H__


#include <stdbool.h>

#define ASSERT_TRACING(x,y)
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

static inline bool is_power_of_two(unsigned int x)
{
	return (x != 0U) && ((x & (x - 1)) == 0U);
}

/** @brief Return larger value of two provided expressions.
 *
 * @note Arguments are evaluated twice. See Z_MAX for GCC only, single
 * evaluation version.
 */
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/** @brief Return smaller value of two provided expressions.
 *
 * @note Arguments are evaluated twice. See Z_MIN for GCC only, single
 * evaluation version.
 */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#endif /* __TRACING_UTILS_H__*/
