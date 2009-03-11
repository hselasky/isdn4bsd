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

#define	off_t my_off_t
#define	mode_t my_mode_t
#define	gid_t my_gid_t
#define	uid_t my_uid_t
#define	size_t my_size_t

#undef __printflike
#undef __aligned

#include </usr/include/sys/cdefs.h>
#include </usr/include/sys/fcntl.h>
#include </usr/include/sys/ttycom.h>
#include </usr/include/sys/termios.h>

extern int ioctl(int f, int cmd, void *arg);
extern int read(int f, void *arg, int len);
extern int write(int f, void *arg, int len);
extern int usleep(int delay);

int     (*UsbInterruptFilterPtr) (void *);
void    (*UsbInterruptHandlerPtr) (void *);
void   *UsbInterruptHandlerArg;
uint8_t *UsbIoBase = 0;

static int f;
static struct termios t;

static uint8_t
cmpdata(const uint8_t *ptr, uint32_t len, uint8_t val)
{
	while (len--) {
		if (ptr[len] != val)
			return (0);
	}
	return (1);
}

void
bus_space_read_multi_1(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint8_t *datap, bus_size_t count)
{
	uint8_t tmp[3];
	bus_size_t mcount;
	int err;

	while (count > 0) {

		mcount = 4;
		if (mcount > count) {
			mcount = count;
		}
		count -= mcount;

		tmp[0] = 0x8B;
		tmp[1] = h - (uint8_t *)0 + offset;
		tmp[2] = mcount;

		if (write(f, tmp, 3) != 3)
			panic("Failure\n");

		if (ioctl(f, TIOCDRAIN, NULL) < 0)
			panic("Failure\n");

		while (read(f, tmp, 1) == 1) {
			if (tmp[0] == 0x8B)
				break;
		}

		if ((err = read(f, datap, mcount)) != mcount) {
			if (err < 0)
				panic("Failure\n");

			datap += err;
			mcount -= err;
		}
		datap += mcount;
	}
}

void
bus_space_read_multi_2(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint16_t *datap, bus_size_t count)
{
	panic("read_multi_2: Not supported\n");
}

void
bus_space_read_multi_4(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint32_t *datap, bus_size_t count)
{
	panic("read_multi_4: Not supported\n");
}

void
bus_space_write_multi_1(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint8_t *datap, bus_size_t count)
{
	uint8_t tmp[3];
	bus_size_t mcount;
	uint8_t cmd;

	while (count > 0) {

		mcount = 4;
		if (mcount > count) {
			mcount = count;
		}
		count -= mcount;

		if (cmpdata(datap, mcount, 0x00))
			cmd = 0x87;
		else if (cmpdata(datap, mcount, 0xFF))
			cmd = 0x88;
		else
			cmd = 0x86;

		tmp[0] = cmd;
		tmp[1] = h - (uint8_t *)0 + offset;
		tmp[2] = mcount;

		if (write(f, tmp, 3) != 3)
			panic("Failure\n");

		if (cmd == 0x86) {
			if (write(f, datap, mcount) != mcount)
				panic("Failure\n");
		}
		if (ioctl(f, TIOCDRAIN, NULL) < 0)
			panic("Failure\n");

		while (read(f, tmp, 1) == 1) {
			if (tmp[0] == cmd)
				break;
		}

		datap += mcount;
	}
}

void
bus_space_write_multi_2(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint16_t *datap, bus_size_t count)
{
	panic("write_multi_2: Not supported\n");
}

void
bus_space_write_multi_4(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint32_t *datap, bus_size_t count)
{
	panic("write_multi_4: Not supported\n");
}

void
bus_space_write_1(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint8_t data)
{
	uint8_t tmp[3];

	tmp[0] = 0x85;
	tmp[1] = h - (uint8_t *)0 + offset;
	tmp[2] = data;

	if (write(f, tmp, 3) != 3)
		panic("Failure\n");

	if (ioctl(f, TIOCDRAIN, NULL) < 0)
		panic("Failure\n");

	while (read(f, tmp, 1) == 1) {
		if (tmp[0] == 0x85)
			return;
	}
	panic("Failure\n");
}

void
bus_space_write_2(bus_space_tag_t t, bus_space_handle_t h,
    bus_size_t offset, uint16_t data)
{
	panic("write_2: Not supported\n");
}

void
bus_space_write_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset, uint32_t data)
{
	panic("write_4: Not supported\n");
}

uint8_t
bus_space_read_1(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset)
{
	uint8_t tmp[2];

	tmp[0] = 0x8A;
	tmp[1] = h - (uint8_t *)0 + offset;

	if (write(f, tmp, 2) != 2)
		panic("Failure\n");

	if (ioctl(f, TIOCDRAIN, NULL) < 0)
		panic("Failure\n");

	while (read(f, tmp, 1) == 1) {
		if (tmp[0] == 0x8A) {
			if (read(f, tmp, 1) == 1)
				return (tmp[0]);
			else
				panic("Failure\n");
		}
	}
	panic("Failure\n");
	return (0);
}

uint16_t
bus_space_read_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset)
{
	panic("read_2: Not supported\n");
	return (0);
}

uint32_t
bus_space_read_4(bus_space_tag_t t, bus_space_handle_t h, bus_size_t offset)
{
	panic("read_4: Not supported\n");
	return (0);
}

#undef malloc
#undef free

extern void *malloc(int size);
extern void free(void *arg);

void   *
malloc_wrap(int size, struct malloc_type *type, int flags)
{
	void *retval;

	mtx_lock(&Atomic);
	retval = malloc(size);
	mtx_unlock(&Atomic);

	return (retval);
}

void
free_wrap(void *addr, struct malloc_type *type)
{
	mtx_lock(&Atomic);
	free(addr);
	mtx_unlock(&Atomic);
}

struct timezone;

struct timeval {
	uint32_t tv_sec;
	uint32_t tv_usec;
};

extern int gettimeofday(struct timeval *tp, struct timezone *tzp);

uint32_t
fbsd_get_timer_us(void)
{
	struct timeval tv;

	mtx_lock(&Atomic);
	gettimeofday(&tv, NULL);
	mtx_unlock(&Atomic);
	return ((tv.tv_sec * 1000000) + tv.tv_usec);
}

extern int bsd_load_module(void);

int
main()
{
	uint8_t tmp[1];
	uint32_t count;

	f = open("/dev/cuaU0", O_RDWR);
	if (f < 0)
		panic("Failure\n");

	if (ioctl(f, TIOCGETA, &t) < 0)
		panic("Failure\n");

	cfmakeraw(&t);

	t.c_ispeed = 38400;
	t.c_ospeed = 38400;

	if (ioctl(f, TIOCSETA, &t) < 0)
		panic("Failure\n");

	tmp[0] = 0x80;

	for (count = 0; count != 20; count++) {
		if (write(f, tmp, 1) != 1)
			panic("Failure\n");
	}

	bsd_load_module();

	while (1) {

		if (UsbInterruptHandlerPtr != NULL) {
			UsbInterruptHandlerPtr(UsbInterruptHandlerArg);
		}
		usleep(1000);
	}

	return (0);
}
