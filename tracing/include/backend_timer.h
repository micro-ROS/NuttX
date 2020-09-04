#ifndef __BACKEND_TIMER_H__
#define __BACKEND_TIMER_H__

#include <string.h>
#include <stdbool.h>
#include <utils.h>

struct backend_timer;

/**
 * @brief backend timer API.
 */
struct backend_timer_api {
	int32_t (*init)(void);
	uint64_t (*gettime)(const struct backend_timer *btimer);
};

/**
 * @brief backend timer structure.
 */
struct backend_timer {
	const char *name;
	const struct backend_timer_api *api;
};

/**
 * @brief Create a backend_timer instance.
 *
 * @param _name Instance name.
 * @param _api  Tracing backend API.
 */
#define BACKEND_TIMER_DEFINE(_name, _api)                              \
	static const Z_STRUCT_SECTION_ITERABLE(backend_timer, _name) = \
	{                                                                \
		.name = STRINGIFY(_name),                                \
		.api = &_api                                             \
	}

/**
 * @brief Initialize backend timer.
 *
 * @param backend Pointer to timer_backend instance.
 */
static inline void backend_timer_init(
		const struct backend_timer *backend)
{
	if (backend && backend->api) {
		backend->api->init();
	}
}

/**
 * @brief Retrieves the system backend timer timer.
 *
 * @param backend Pointer to backend_timer instance.
 * @return A 64bits time value.
 */
static inline uint64_t backend_timer_gettime(
		const struct backend_timer *backend)
{
	if (backend && backend->api) {
		return backend->api->gettime(backend);
	}

	return 0;
}

/**
 * @brief Get backend timer based on the name of
 *        backend timer in the backend timer section.
 *
 * @param name Name of wanted backend timer.
 *
 * @return Pointer of the wanted backend or NULL.
 */
static inline struct backend_timer *backend_timer_get(char *name)
{
	Z_STRUCT_SECTION_FOREACH(backend_timer, backend) {
		if (strcmp(backend->name, name) == 0) {
			return backend;
		}
	}

	return NULL;
}

#endif //__BACKEND_TIMER_H__
