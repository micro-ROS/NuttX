#ifndef __TRACING_UTILS_H__
#define __TRACING_UTILS_H__

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
