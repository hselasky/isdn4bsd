/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <bsd_module_all.h>

void
bus_space_read_multi_1(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint8_t *datap, bus_size_t count)
{
	volatile uint8_t *ptr = (h + offset);

	while (count >= 4) {
		datap[0] = *ptr;
		datap[1] = *ptr;
		datap[2] = *ptr;
		datap[3] = *ptr;
		count -= 4;
		datap += 4;
	}

	while (count--) {
		*datap++ = *ptr;
	}
	return;
}

void
bus_space_read_multi_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint16_t *datap, bus_size_t count)
{
	volatile uint16_t *ptr = (void *)(h + offset);

	while (count >= 4) {
		datap[0] = *ptr;
		datap[1] = *ptr;
		datap[2] = *ptr;
		datap[3] = *ptr;
		count -= 4;
		datap += 4;
	}

	while (count--) {
		*datap++ = *ptr;
	}
	return;
}

void
bus_space_read_multi_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint32_t *datap, bus_size_t count)
{
	volatile uint32_t *ptr = (void *)(h + offset);
	uint32_t rem;

#ifdef TARGET_IS_ARM
	rem = count & 7;
	count &= ~7;
	if (count >= 8) {
		do {
			__asm volatile (
			             "ldr r0, [%2]\n"
			             "ldr r1, [%2]\n"
			             "ldr r2, [%2]\n"
			             "ldr r3, [%2]\n"
			             "ldr r4, [%2]\n"
			             "ldr r5, [%2]\n"
			             "ldr r6, [%2]\n"
			             "ldr r7, [%2]\n"
			             "stmia %0!,{r0-r7}\n":
			             "=r" (datap):
			             "r"(datap),
			             "r"(ptr):"memory",
			             "r0", "r1", "r2", "r3",
			             "r4", "r5", "r6", "r7");
		} while ((count -= 8));
	}
#else
	rem = count & 3;
	count &= ~3;
	if (count >= 4) {
		do {
			datap[0] = *ptr;
			datap[1] = *ptr;
			datap[2] = *ptr;
			datap[3] = *ptr;
			datap += 4;
		} while ((count -= 4));
	}
#endif
	while (rem--) {
		*datap++ = *ptr;
	}
	return;
}

void
bus_space_write_multi_1(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint8_t *datap, bus_size_t count)
{
	volatile uint8_t *ptr = (h + offset);

	while (count >= 4) {
		*ptr = datap[0];
		*ptr = datap[1];
		*ptr = datap[2];
		*ptr = datap[3];
		count -= 4;
		datap += 4;
	}

	while (count--) {
		*ptr = *datap++;
	}
	return;
}

void
bus_space_write_multi_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint16_t *datap, bus_size_t count)
{
	volatile uint16_t *ptr = (void *)(h + offset);

	while (count >= 4) {
		*ptr = datap[0];
		*ptr = datap[1];
		*ptr = datap[2];
		*ptr = datap[3];
		count -= 4;
		datap += 4;
	}

	while (count--) {
		*ptr = *datap++;
	}
	return;
}

void
bus_space_write_multi_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint32_t *datap, bus_size_t count)
{
	volatile uint32_t *ptr = (void *)(h + offset);
	uint32_t rem;

#ifdef TARGET_IS_ARM
	rem = count & 7;
	count &= ~7;

	if (count >= 8) {
		do {
			__asm volatile (
			             "ldmia %0!, {r0-r7}\n"
			             "str r0, [%2]\n"
			             "str r1, [%2]\n"
			             "str r2, [%2]\n"
			             "str r3, [%2]\n"
			             "str r4, [%2]\n"
			             "str r5, [%2]\n"
			             "str r6, [%2]\n"
			             "str r7, [%2]\n":
			             "=r" (datap):
			             "r"(datap),
			             "r"(ptr):"memory",
			             "r0", "r1", "r2", "r3",
			             "r4", "r5", "r6", "r7");

		} while ((count -= 8));
	}
#else
	rem = count & 3;
	count &= ~3;
	if (count >= 4) {
		do {
			*ptr = datap[0];
			*ptr = datap[1];
			*ptr = datap[2];
			*ptr = datap[3];
			datap += 4;
		} while ((count -= 4));
	}
#endif

	while (rem--) {
		*ptr = *datap++;
	}
	return;
}

void
bus_space_barrier(bus_space_tag_t space, bus_space_handle_t handle,
    bus_size_t offset, bus_size_t length, int flags)
{
	/* Assume uncached I/O */
	return;
}
