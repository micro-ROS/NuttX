/****************************************************************************
 * mm/mm_heap/mm_malloc.c
 *
 *   Copyright (C) 2007, 2009, 2013-2014, 2017  Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <string.h>

#include <nuttx/mm/mm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#  define NULL ((void *)0)
#endif

/****************************************************************************
 * Debugging Functions
 ****************************************************************************/
#ifdef CONFIG_LIBBACKTRACE

#include <backtrace.h>

#define BACKTRACE_SIZE	8
FAR backtrace_t backtrace_buf[BACKTRACE_SIZE];

/* To satisfy nuttx with its -Wmissing-declarations */
FAR void print_itm(unsigned int data);
FAR void print_str(const char *str, int max_len);
FAR void print_number_dec(unsigned int data);
FAR void print_number_hex(unsigned int data);
FAR void dump_backtrace(backtrace_t *bt, int count, int size_blk, void *blk);

/* TODO: fix this to be usable indepently of the CPU core */
#define STIMULI_BASE 	(0xE0000000)
#define ITM_TCR         ( *(volatile uint32_t *) 0xe0000e80)
#define ITM_TENR             ( *(volatile uint32_t *) 0xe0000e00)

struct stimuli_reg {
	union {
		uint32_t reg_base;
		char reg_char[4];
	};
}__attribute__((packed));

/****************************************************************************
 * Name: print_itm
 *
 * Description: Print the data to the itm. The data will be casted to a char.
 ****************************************************************************/
FAR void print_itm(unsigned int data)
{
	volatile struct stimuli_reg *STMU =
					(volatile struct stimuli_reg *) STIMULI_BASE;
	unsigned int timeout = 100000;

	/* ITM enabled */
	if (((ITM_TCR & 0x1UL) != 0UL) &&
			((ITM_TENR & 1UL) != 0UL))  {
		/* Wait for available reg ot be available */
		while (STMU->reg_base == (unsigned int) 0x0) {
			if (!timeout--) {
				return;
			}
		}
		STMU->reg_char[0] = (char) data;
	}
}

/****************************************************************************
 * Name: print_number_dec
 *
 * Description: Procedure to print to decimal  number to the itm
 ****************************************************************************/
FAR void print_str(const char *str, int max_len)
{
	unsigned int i = 0;

	while ((str[i] != '\0') && (i < max_len)) {
		print_itm((unsigned int) str[i++]);
	}
}

/****************************************************************************
 * Name: print_number_dec
 *
 * Description: Procedure to print to decimal  number to the itm
 ****************************************************************************/
FAR void print_number_dec(unsigned int data)
{
	char data_str[] = "000000000";
	int j = sizeof(data_str) - 2;
	unsigned int unity;

	while (data > 9) {
		unity = data % 10;
		data_str[j--] = unity + '0';
		data = data / 10;
	}

	data_str[j] = data + '0';
	print_str(&data_str[j], sizeof(data_str) - 2 + j);
}

/****************************************************************************
 * Name: print_number_hex
 *
 * Description: Procedure to print to hex number to the itm
 ****************************************************************************/
FAR void print_number_hex(unsigned int data)
{
	const char hex_to_str[] = "0123456789ABCDEF";
	char hex;

	print_itm((unsigned int)'0');
	print_itm((unsigned int)'x');
	for (int i = (sizeof(unsigned int) * 2) - 1; i >= 0; --i) {
		hex = hex_to_str[((data >> i * 4)  & 0xf)];
		print_itm((unsigned int) hex);
	}
}


/****************************************************************************
 * Name: dump_backtrace
 *
 * Description:
 *	Create a backtrace dump from the once called and print it to the ITM
 *	trace. The format shall look as follow:
 *		alloc <size_in_bytes> <blk_pointer>
 *		<address_backtrace> <function_name>  <-- one line per depth bt
 ****************************************************************************/
FAR void dump_backtrace(backtrace_t *bt, int count, int size_blk, void *blk)
{
	print_str("alloc  ", sizeof("alloc ") - 1);
	print_number_dec(size_blk);
	print_itm(' ');
	print_number_hex((unsigned int) blk);
	print_itm('\n');

	for (int i = count - 1; i > -1 ; i--) {
		print_number_hex((unsigned int) bt[i].address);
		print_itm(' ');
		print_str(bt[i].name, 32);
		print_itm((unsigned int)'\n');
	}
}

#endif // CONFIG_LIBBACKTRACE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_malloc
 *
 * Description:
 *  Find the smallest chunk that satisfies the request. Take the memory from
 *  that chunk, save the remaining, smaller chunk (if any).
 *
 *  8-byte alignment of the allocated data is assured.
 *
 ****************************************************************************/

FAR void *mm_malloc(FAR struct mm_heap_s *heap, size_t size)
{
  FAR struct mm_freenode_s *node;
  size_t alignsize;
  void *ret = NULL;
  int ndx;
#ifdef CONFIG_LIBBACKTRACE
  int count;
#endif

  /* Ignore zero-length allocations */

  if (size < 1)
    {
      return NULL;
    }

  /* Adjust the size to account for (1) the size of the allocated node and
   * (2) to make sure that it is an even multiple of our granule size.
   */

  alignsize = MM_ALIGN_UP(size + SIZEOF_MM_ALLOCNODE);
  DEBUGASSERT(alignsize >= size);  /* Check for integer overflow */

  /* We need to hold the MM semaphore while we muck with the nodelist. */

  mm_takesemaphore(heap);

  /* Get the location in the node list to start the search. Special case
   * really big allocations
   */

  if (alignsize >= MM_MAX_CHUNK)
    {
      ndx = MM_NNODES-1;
    }
  else
    {
      /* Convert the request size into a nodelist index */

      ndx = mm_size2ndx(alignsize);
    }

  /* Search for a large enough chunk in the list of nodes. This list is
   * ordered by size, but will have occasional zero sized nodes as we visit
   * other mm_nodelist[] entries.
   */

  for (node = heap->mm_nodelist[ndx].flink;
       node && node->size < alignsize;
       node = node->flink);

  /* If we found a node with non-zero size, then this is one to use. Since
   * the list is ordered, we know that is must be best fitting chunk
   * available.
   */

  if (node)
    {
      FAR struct mm_freenode_s *remainder;
      FAR struct mm_freenode_s *next;
      size_t remaining;

      /* Remove the node.  There must be a predecessor, but there may not be
       * a successor node.
       */

      DEBUGASSERT(node->blink);
      node->blink->flink = node->flink;
      if (node->flink)
        {
          node->flink->blink = node->blink;
        }

      /* Check if we have to split the free node into one of the allocated
       * size and another smaller freenode.  In some cases, the remaining
       * bytes can be smaller (they may be SIZEOF_MM_ALLOCNODE).  In that
       * case, we will just carry the few wasted bytes at the end of the
       * allocation.
       */

      remaining = node->size - alignsize;
      if (remaining >= SIZEOF_MM_FREENODE)
        {
          /* Get a pointer to the next node in physical memory */

          next = (FAR struct mm_freenode_s *)(((FAR char *)node) + node->size);

          /* Create the remainder node */

          remainder = (FAR struct mm_freenode_s *)
            (((FAR char *)node) + alignsize);

          remainder->size      = remaining;
          remainder->preceding = alignsize;

          /* Adjust the size of the node under consideration */

          node->size = alignsize;

          /* Adjust the 'preceding' size of the (old) next node, preserving
           * the allocated flag.
           */

          next->preceding = remaining | (next->preceding & MM_ALLOC_BIT);

          /* Add the remainder back into the nodelist */

          mm_addfreechunk(heap, remainder);
        }

      /* Handle the case of an exact size match */

      node->preceding |= MM_ALLOC_BIT;
      ret = (void *)((FAR char *)node + SIZEOF_MM_ALLOCNODE);
    }

  mm_givesemaphore(heap);

#ifdef CONFIG_MM_FILL_ALLOCATIONS
  if (ret)
    {
       memset(ret, 0xAA, alignsize - SIZEOF_MM_ALLOCNODE);
    }
#endif

  /* If CONFIG_DEBUG_MM is defined, then output the result of the allocation
   * to the SYSLOG.
   */

#ifdef CONFIG_DEBUG_MM
  if (!ret)
    {
      mwarn("WARNING: Allocation failed, size %d\n", alignsize);
    }
  else
    {
      minfo("Allocated %p, size %d\n", ret, alignsize);
    }
#endif

#ifdef CONFIG_LIBBACKTRACE
  count = backtrace_unwind(backtrace_buf, BACKTRACE_SIZE);
  dump_backtrace(backtrace_buf, count, alignsize + SIZEOF_MM_ALLOCNODE, ret);
#endif // CONFIG_LIBBACKTRACE

  return ret;
}
