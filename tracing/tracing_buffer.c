/*
 * Copyright (c) 2019 Intel corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tracing_buffer.h>
#include <string.h>

/**
 * Internal data structure for a buffer header.
 *
 * We want all of this to fit in a single u32_t. Every item stored in the
 * ring buffer will be one of these headers plus any extra data supplied
 */
struct ring_element {
	u32_t  type   :16; /**< Application-specific */
	u32_t  length :8;  /**< length in 32-bit chunks */
	u32_t  value  :8;  /**< Room for small integral values */
};
#define SIZE32_OF(x) (sizeof((x))/sizeof(u32_t)) 
/**
 * @brief A structure to represent a ring buffer
 */
struct ring_buf {
        u32_t head;      /**< Index in buf for the head element */
        u32_t tail;      /**< Index in buf for the tail element */
        union ring_buf_misc {
                struct ring_buf_misc_item_mode {
                        u32_t dropped_put_count; /**< Running tally of the
                                                   * number of failed put
                                                   * attempts.
                                                   */
                } item_mode;
                struct ring_buf_misc_byte_mode {
                        u32_t tmp_tail;
                        u32_t tmp_head;
                } byte_mode;
        } misc;
        u32_t size;   /**< Size of buf in 32-bit chunks */

        union ring_buf_buffer {
                u32_t *buf32;    /**< Memory region for stored entries */
                u8_t *buf8;
        } buf;
        u32_t mask;   /**< Modulo mask if size is a power of 2 */
};

static inline u32_t z_ring_buf_custom_space_get(u32_t size, u32_t head,
		u32_t tail)
{
	if (tail < head) {
		return head - tail - 1;
	}

	/* buf->tail > buf->head */
	return (size - tail) + head - 1;
}

static inline u32_t ring_buf_space_get(struct ring_buf *buf)
{
        return z_ring_buf_custom_space_get(buf->size, buf->head, buf->tail);
}

static inline u32_t ring_buf_capacity_get(struct ring_buf *buf)
{
	/* One element is used to distinguish between empty and full
	 * state. */
	return buf->size - 1;
}

/**
 * @brief Initialize a ring buffer.
 *
 * This routine initializes a ring buffer, prior to its first use. It is only
 * used for ring buffers not defined using RING_BUF_DECLARE,
 * RING_BUF_ITEM_DECLARE_POW2 or RING_BUF_ITEM_DECLARE_SIZE.
 *
 * Setting @a size to a power of 2 establishes a high performance ring buffer
 * that doesn't require the use of modulo arithmetic operations to maintain
 * itself.
 *
 * @param buf Address of ring buffer.
 * @param size Ring buffer size (in 32-bit words or bytes).
 * @param data Ring buffer data area (u32_t data[size] or u8_t data[size] for
 *             bytes mode).
 */
static inline void ring_buf_init(struct ring_buf *buf, u32_t size, void *data)
{
        memset(buf, 0, sizeof(struct ring_buf));
        buf->size = size;
        buf->buf.buf32 = (u32_t *)data;
        if (is_power_of_two(size)) {
                buf->mask = size - 1;
        } else {
                buf->mask = 0U;
        }
}

/**
 * @brief Determine if a ring buffer is empty.
 *
 * @param buf Address of ring buffer.
 *
 * @return 1 if the ring buffer is empty, or 0 if not.
 */
static inline int ring_buf_is_empty(struct ring_buf *buf)
{
        return (buf->head == buf->tail);
}

/**
 * @brief Reset ring buffer state.
 *
 * @param buf Address of ring buffer.
 */
static inline void ring_buf_reset(struct ring_buf *buf)
{
        buf->head = 0;
        buf->tail = 0;
        memset(&buf->misc, 0, sizeof(buf->misc));
}



/**
 * @brief Write a data item to a ring buffer.
 *
 * This routine writes a data item to ring buffer @a buf. The data item
 * is an array of 32-bit words (from zero to 1020 bytes in length),
 * coupled with a 16-bit type identifier and an 8-bit integer value.
 *
 * @warning
 * Use cases involving multiple writers to the ring buffer must prevent
 * concurrent write operations, either by preventing all writers from
 * being preempted or by using a mutex to govern writes to the ring buffer.
 *
 * @param buf Address of ring buffer.
 * @param type Data item's type identifier (application specific).
 * @param value Data item's integer value (application specific).
 * @param data Address of data item.
 * @param size32 Data item size (number of 32-bit words).
 *
 * @retval 0 Data item was written.
 * @retval -EMSGSIZE Ring buffer has insufficient free space.
 */
int ring_buf_item_put(struct ring_buf *buf, u16_t type, u8_t value,
                      u32_t *data, u8_t size32);

/**
 * @brief Read a data item from a ring buffer.
 *
 * This routine reads a data item from ring buffer @a buf. The data item
 * is an array of 32-bit words (up to 1020 bytes in length),
 * coupled with a 16-bit type identifier and an 8-bit integer value.
 *
 * @warning
 * Use cases involving multiple reads of the ring buffer must prevent
 * concurrent read operations, either by preventing all readers from
 * being preempted or by using a mutex to govern reads to the ring buffer.
 *
 * @param buf Address of ring buffer.
 * @param type Area to store the data item's type identifier.
 * @param value Area to store the data item's integer value.
 * @param data Area to store the data item.
 * @param size32 Size of the data item storage area (number of 32-bit chunks).
 *
 * @retval 0 Data item was fetched; @a size32 now contains the number of
 *         32-bit words read into data area @a data.
 * @retval -EAGAIN Ring buffer is empty.
 * @retval -EMSGSIZE Data area @a data is too small; @a size32 now contains
 *         the number of 32-bit words needed.
 */
static int ring_buf_item_get(struct ring_buf *buf, u16_t *type, u8_t *value,
                      u32_t *data, u8_t *size32);

/**
 * @brief Allocate buffer for writing data to a ring buffer.
 *
 * With this routine, memory copying can be reduced since internal ring buffer
 * can be used directly by the user. Once data is written to allocated area
 * number of bytes written can be confirmed (see @ref ring_buf_put_finish).
 *
 * @warning
 * Use cases involving multiple writers to the ring buffer must prevent
 * concurrent write operations, either by preventing all writers from
 * being preempted or by using a mutex to govern writes to the ring buffer.
 *
 * @warning
 * Ring buffer instance should not mix byte access and item access
 * (calls prefixed with ring_buf_item_).
 *
 * @param[in]  buf  Address of ring buffer.
 * @param[out] data Pointer to the address. It is set to a location within
 *                  ring buffer.
 * @param[in]  size Requested allocation size (in bytes).
 *
 * @return Size of allocated buffer which can be smaller than requested if
 *         there is not enough free space or buffer wraps.
 */
static u32_t ring_buf_put_claim(struct ring_buf *buf, u8_t **data, u32_t size);

/**
 * @brief Indicate number of bytes written to allocated buffers.
 *
 * @warning
 * Use cases involving multiple writers to the ring buffer must prevent
 * concurrent write operations, either by preventing all writers from
 * being preempted or by using a mutex to govern writes to the ring buffer.
 *
 * @warning
 * Ring buffer instance should not mix byte access and item access
 * (calls prefixed with ring_buf_item_).
 *
 * @param  buf  Address of ring buffer.
 * @param  size Number of valid bytes in the allocated buffers.
 *
 * @retval 0 Successful operation.
 * @retval -EINVAL Provided @a size exceeds free space in the ring buffer.
 */
static int ring_buf_put_finish(struct ring_buf *buf, u32_t size);

/**
 * @brief Write (copy) data to a ring buffer.
 *
 * This routine writes data to a ring buffer @a buf.
 *
 * @warning
 * Use cases involving multiple writers to the ring buffer must prevent
 * concurrent write operations, either by preventing all writers from
 * being preempted or by using a mutex to govern writes to the ring buffer.
 *
 * @warning
 * Ring buffer instance should not mix byte access and item access
 * (calls prefixed with ring_buf_item_).
 *
 * @param buf Address of ring buffer.
 * @param data Address of data.
 * @param size Data size (in bytes).
 *
 * @retval Number of bytes written.
 */
static u32_t ring_buf_put(struct ring_buf *buf, const u8_t *data, u32_t size);

/**
 * @brief Get address of a valid data in a ring buffer.
 *
 * With this routine, memory copying can be reduced since internal ring buffer
 * can be used directly by the user. Once data is processed it can be freed
 * using @ref ring_buf_get_finish.
 *
 * @warning
 * Use cases involving multiple reads of the ring buffer must prevent
 * concurrent read operations, either by preventing all readers from
 * being preempted or by using a mutex to govern reads to the ring buffer.
 *
 * @warning
 * Ring buffer instance should not mix byte access and item access
 * (calls prefixed with ring_buf_item_).
 *
 * @param[in]  buf  Address of ring buffer.
 * @param[out] data Pointer to the address. It is set to a location within
 *                  ring buffer.
 * @param[in]  size Requested size (in bytes).
 *
 * @return Number of valid bytes in the provided buffer which can be smaller
 *         than requested if there is not enough free space or buffer wraps.
 */
static u32_t ring_buf_get_claim(struct ring_buf *buf, u8_t **data, u32_t size);

/**
 * @brief Indicate number of bytes read from claimed buffer.
 *
 * @warning
 * Use cases involving multiple reads of the ring buffer must prevent
 * concurrent read operations, either by preventing all readers from
 * being preempted or by using a mutex to govern reads to the ring buffer.
 *
 * @warning
 * Ring buffer instance should not mix byte access and  item mode
 * (calls prefixed with ring_buf_item_).
 *
 * @param  buf  Address of ring buffer.
 * @param  size Number of bytes that can be freed.
 *
 * @retval 0 Successful operation.
 * @retval -EINVAL Provided @a size exceeds valid bytes in the ring buffer.
 */
static int ring_buf_get_finish(struct ring_buf *buf, u32_t size);

/**
 * @brief Read data from a ring buffer.
 *
 * This routine reads data from a ring buffer @a buf.
 *
 * @warning
 * Use cases involving multiple reads of the ring buffer must prevent
 * concurrent read operations, either by preventing all readers from
 * being preempted or by using a mutex to govern reads to the ring buffer.
 *
 * @warning
 * Ring buffer instance should not mix byte access and  item mode
 * (calls prefixed with ring_buf_item_).
 *
 * @param buf  Address of ring buffer.
 * @param data Address of the output buffer.
 * @param size Data size (in bytes).
 *
 * @retval Number of bytes written to the output buffer.
 */
static u32_t ring_buf_get(struct ring_buf *buf, u8_t *data, u32_t size);

/**
 * @}
 */



static struct ring_buf tracing_ring_buf;
static u8_t tracing_buffer[CONFIG_CTF_RING_BUFFER_SIZE + 1];
static u8_t tracing_cmd_buffer[CONFIG_CTF_RING_BUFFER_SIZE];

u32_t tracing_cmd_buffer_alloc(u8_t **data)
{
	*data = &tracing_cmd_buffer[0];

	return sizeof(tracing_cmd_buffer);
}

u32_t tracing_buffer_put_claim(u8_t **data, u32_t size)
{
	return ring_buf_put_claim(&tracing_ring_buf, data, size);
}

int tracing_buffer_put_finish(u32_t size)
{
	return ring_buf_put_finish(&tracing_ring_buf, size);
}

u32_t tracing_buffer_put(u8_t *data, u32_t size)
{
	return ring_buf_put(&tracing_ring_buf, data, size);
}

u32_t tracing_buffer_get_claim(u8_t **data, u32_t size)
{
	return ring_buf_get_claim(&tracing_ring_buf, data, size);
}

int tracing_buffer_get_finish(u32_t size)
{
	return ring_buf_get_finish(&tracing_ring_buf, size);
}

u32_t tracing_buffer_get(u8_t *data, u32_t size)
{
	return ring_buf_get(&tracing_ring_buf, data, size);
}

void tracing_buffer_init(void)
{
	ring_buf_init(&tracing_ring_buf,
		      sizeof(tracing_buffer), tracing_buffer);
}

bool tracing_buffer_is_empty(void)
{
	return ring_buf_is_empty(&tracing_ring_buf);
}

u32_t tracing_buffer_capacity_get(void)
{
	return ring_buf_capacity_get(&tracing_ring_buf);
}

u32_t tracing_buffer_space_get(void)
{
	return ring_buf_space_get(&tracing_ring_buf);
}

int ring_buf_item_put(struct ring_buf *buf, u16_t type, u8_t value,
		      u32_t *data, u8_t size32)
{
	u32_t i, space, index, rc;

	space = ring_buf_space_get(buf);
	if (space >= (size32 + 1)) {
		struct ring_element *header =
			(struct ring_element *)&buf->buf.buf32[buf->tail];
		header->type = type;
		header->length = size32;
		header->value = value;

		if (buf->mask) {
			for (i = 0U; i < size32; ++i) {
				index = (i + buf->tail + 1) & buf->mask;
				buf->buf.buf32[index] = data[i];
			}
			buf->tail = (buf->tail + size32 + 1) & buf->mask;
		} else {
			for (i = 0U; i < size32; ++i) {
				index = (i + buf->tail + 1) % buf->size;
				buf->buf.buf32[index] = data[i];
			}
			buf->tail = (buf->tail + size32 + 1) % buf->size;
		}
		rc = 0U;
	} else {
		buf->misc.item_mode.dropped_put_count++;
		rc = -1;
	}

	return rc;
}

int ring_buf_item_get(struct ring_buf *buf, u16_t *type, u8_t *value,
		      u32_t *data, u8_t *size32)
{
	struct ring_element *header;
	u32_t i, index;

	if (ring_buf_is_empty(buf)) {
		return -1;
	}

	header = (struct ring_element *) &buf->buf.buf32[buf->head];

	if (header->length > *size32) {
		*size32 = header->length;
		return -1;
	}

	*size32 = header->length;
	*type = header->type;
	*value = header->value;

	if (buf->mask) {
		for (i = 0U; i < header->length; ++i) {
			index = (i + buf->head + 1) & buf->mask;
			data[i] = buf->buf.buf32[index];
		}
		buf->head = (buf->head + header->length + 1) & buf->mask;
	} else {
		for (i = 0U; i < header->length; ++i) {
			index = (i + buf->head + 1) % buf->size;
			data[i] = buf->buf.buf32[index];
		}
		buf->head = (buf->head + header->length + 1) % buf->size;
	}

	return 0;
}

/** @brief Wraps index if it exceeds the limit.
 *
 * @param val  Value
 * @param max  Max.
 *
 * @return value % max.
 */
static inline u32_t wrap(u32_t val, u32_t max)
{
	return val >= max ? (val - max) : val;
}

u32_t ring_buf_put_claim(struct ring_buf *buf, u8_t **data, u32_t size)
{
	u32_t space, trail_size, allocated;

	space = z_ring_buf_custom_space_get(buf->size, buf->head,
					    buf->misc.byte_mode.tmp_tail);

	/* Limit requested size to available size. */
	size = MIN(size, space);
	trail_size = buf->size - buf->misc.byte_mode.tmp_tail;

	/* Limit allocated size to trail size. */
	allocated = MIN(trail_size, size);

	*data = &buf->buf.buf8[buf->misc.byte_mode.tmp_tail];
	buf->misc.byte_mode.tmp_tail =
		wrap(buf->misc.byte_mode.tmp_tail + allocated, buf->size);

	return allocated;
}

int ring_buf_put_finish(struct ring_buf *buf, u32_t size)
{
	if (size > ring_buf_space_get(buf)) {
		return -1;
	}

	buf->tail = wrap(buf->tail + size, buf->size);
	buf->misc.byte_mode.tmp_tail = buf->tail;

	return 0;
}

u32_t ring_buf_put(struct ring_buf *buf, const u8_t *data, u32_t size)
{
	u8_t *dst;
	u32_t partial_size;
	u32_t total_size = 0U;
	int err;

	do {
		partial_size = ring_buf_put_claim(buf, &dst, size);
		memcpy(dst, data, partial_size);
		total_size += partial_size;
		size -= partial_size;
		data += partial_size;
	} while (size && partial_size);

	err = ring_buf_put_finish(buf, total_size);

	return total_size;
}

u32_t ring_buf_get_claim(struct ring_buf *buf, u8_t **data, u32_t size)
{
	u32_t space, granted_size, trail_size;

	space = (buf->size - 1) -
		z_ring_buf_custom_space_get(buf->size,
					    buf->misc.byte_mode.tmp_head,
					    buf->tail);
	trail_size = buf->size - buf->misc.byte_mode.tmp_head;

	/* Limit requested size to available size. */
	granted_size = MIN(size, space);

	/* Limit allocated size to trail size. */
	granted_size = MIN(trail_size, granted_size);

	*data = &buf->buf.buf8[buf->misc.byte_mode.tmp_head];
	buf->misc.byte_mode.tmp_head =
		wrap(buf->misc.byte_mode.tmp_head + granted_size, buf->size);

	return granted_size;
}

int ring_buf_get_finish(struct ring_buf *buf, u32_t size)
{
	u32_t allocated = (buf->size - 1) - ring_buf_space_get(buf);

	if (size > allocated) {
		return -1;
	}

	buf->head = wrap(buf->head + size, buf->size);
	buf->misc.byte_mode.tmp_head = buf->head;

	return 0;
}

u32_t ring_buf_get(struct ring_buf *buf, u8_t *data, u32_t size)
{
	u8_t *src;
	u32_t partial_size;
	u32_t total_size = 0U;
	int err;

	do {
		partial_size = ring_buf_get_claim(buf, &src, size);
		memcpy(data, src, partial_size);
		total_size += partial_size;
		size -= partial_size;
		data += partial_size;
	} while (size && partial_size);

	err = ring_buf_get_finish(buf, total_size);
	return total_size;
}
