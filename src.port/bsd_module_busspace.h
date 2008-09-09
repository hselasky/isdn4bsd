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

/* Lite implementation for memory mapped I/O */

typedef void *bus_space_tag_t;
typedef uint8_t *bus_space_handle_t;

#define	bus_space_write_1(t,h,off,val)  do { \
    *((volatile uint8_t *)((h) + (off))) = (val); \
} while (0)

#define	bus_space_write_2(t,h,off,val)  do { \
    *((volatile uint16_t *)((h) + (off))) = (val); \
} while (0)

#define	bus_space_write_4(t,h,off,val) do { \
    *((volatile uint32_t *)((h) + (off))) = (val); \
} while (0)

#define	bus_space_read_1(t,h,off) *((volatile uint8_t *)((h) + (off)))
#define	bus_space_read_2(t,h,off) *((volatile uint16_t *)((h) + (off)))
#define	bus_space_read_4(t,h,off) *((volatile uint32_t *)((h) + (off)))

void	bus_space_read_multi_1(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint8_t *datap, bus_size_t count);
void	bus_space_read_multi_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint16_t *datap, bus_size_t count);
void	bus_space_read_multi_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint32_t *datap, bus_size_t count);

void	bus_space_write_multi_1(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint8_t *datap, bus_size_t count);
void	bus_space_write_multi_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint16_t *datap, bus_size_t count);
void	bus_space_write_multi_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint32_t *datap, bus_size_t count);

#define	BUS_SPACE_BARRIER_READ 0x01
#define	BUS_SPACE_BARRIER_WRITE 0x02

void	bus_space_barrier(bus_space_tag_t space, bus_space_handle_t handle, bus_size_t offset, bus_size_t length, int flags);
