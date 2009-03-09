/*-
 * Copyright (c) 2009 Hans Petter Selasky. All rights reserved.
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

#define off_t my_off_t
#define mode_t my_mode_t
#undef __printflike
#undef __aligned

#include </usr/include/sys/cdefs.h>
#include </usr/include/sys/fcntl.h>
#include </usr/include/sys/ttycom.h>
#include </usr/include/sys/termios.h>

static int f;
static struct termios t;

static uint8_t
inb(uint8_t reg)
{
	uint8_t tmp[2];

	tmp[0] = 0x8A;
	tmp[1] = reg & 0x7F;

	if (write(f, tmp, 2) != 2)
		exit(1);

	if (ioctl(f, TIOCDRAIN, NULL) < 0)
		exit(1);

	while (read(f, tmp, 1) == 1) {
		if (tmp[0] == 0x8A) {
			if (read(f, tmp, 1) == 1)
				return (tmp[0]);
		}
	}
	exit(1);
	return (0);
}

static void
outb(uint8_t reg, uint8_t data)
{
	uint8_t tmp[3];

	tmp[0] = 0x85;
	tmp[1] = reg & 0x7F;
	tmp[2] = data;

	if (write(f, tmp, 3) != 3)
		exit(1);

	if (ioctl(f, TIOCDRAIN, NULL) < 0)
		exit(1);

	while (read(f, tmp, 1) == 1) {
		if (tmp[0] == 0x85)
			return;
	}
	exit(1);
}

void
bus_space_write_1(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint8_t data)
{
	outb(h - (uint8_t *)0 + offset, data);
}

void
bus_space_write_2(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint16_t data)
{
	printf("cannot write 16-bit\n");
}

void
bus_space_write_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint32_t data)
{
	printf("cannot write 32-bit\n");
}

uint8_t
bus_space_read_1(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset)
{
	return (inb(h - (uint8_t *)0 + offset));
}
uint16_t
bus_space_read_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset)
{
	printf("cannot read 16-bit\n");
	return (0);
}

uint32_t
bus_space_read_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset)
{
	printf("cannot read 32-bit\n");
}

int
main()
{
	uint8_t tmp[8];
	uint32_t count;

	f = open("/dev/cuaU0", O_RDWR);
	if (f < 0)
		exit(1);

	if (ioctl(f, TIOCGETA, &t) < 0)
		exit(1);

	cfmakeraw(&t);

	t.c_ispeed = 19200;
	t.c_ospeed = 19200;

	if (ioctl(f, TIOCSETA, &t) < 0)
		exit(1);

	tmp[0] = 0x80;
	tmp[1] = 0x80;
	tmp[2] = 0x80;
	tmp[3] = 0x80;
	tmp[4] = 0x80;
	tmp[5] = 0x80;
	tmp[6] = 0x80;
	tmp[7] = 0x80;

	if (write(f, tmp, 8) != 8)
		exit(1);

	printf("0x%02, 0x%02x\n",
	    inb(0xCC), inb(0xCD));

	return (0);
}
