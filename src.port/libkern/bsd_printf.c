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

#ifdef printf
int
printf(const char *fmt,...)
{
	va_list ap;
	int retval;

	va_start(ap, fmt);
	retval = vprintf(fmt, ap);
	va_end(ap);
	return (retval);
}

#endif

#ifdef vprintf
int
vprintf(const char *fmt, va_list ap)
{
	int val;
	char buf[128];

	val = vsnprintf(buf, sizeof(buf), fmt, ap);

	printf("%s", buf);

	return (val);
}

#endif

#ifdef snprintf
int
snprintf(char *dst, size_t size, const char *fmt,...)
{
	va_list ap;
	int retval;

	va_start(ap, fmt);
	retval = vsnprintf(dst, size, fmt, ap);
	va_end(ap);
	return (retval);
}

#endif

#ifdef vsnrprintf
int
vsnrprintf(char *dst, size_t size, int radix, const char *fmt, va_list ap)
{
	return (vsnprintf(dst, size, fmt, ap));
}

#endif

#ifdef vsnprintf

static void
rotate(char *ptr, char *buf)
{
	char c;

	while (ptr < buf) {
		buf--;
		c = *ptr;
		*ptr = *buf;
		*buf = c;
		ptr++;
	}
	return;
}

int
vsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
{
	static const char *hex = "0123456789abcdef";
	union {
		int    *psint;
		unsigned int *puint;
		char   *pstr;
		char  **ppstr;
		void   *arg;
	}     u;
	uint32_t itemp;
	size_t osize;
	char *ptr;
	char c;
	uint8_t escape;

	if (size == 0) {
		return (0);
	}
	size--;				/* reserve space for NUL */
	escape = 0;
	osize = size;

	while ((c = *fmt++)) {
		if (size == 0) {
			break;
		}
		switch (escape) {
		default:
			switch (c) {
			case '%':
				escape = 2;
				break;
			case '\\':
				escape = 1;
				break;
			default:
				*buf++ = c;
				size--;
				break;
			}
			break;
		case 1:
			switch (c) {
			case 'n':
				*buf++ = '\n';
				break;
			case 'r':
				*buf++ = '\r';
				break;
			case 't':
				*buf++ = '\t';
				break;
			default:
				*buf++ = c;
				break;
			}
			size--;
			escape = 0;
			break;
		case 2:
			escape = 0;
			u.arg = ap;
			ap += sizeof(void *);
			switch (c) {
			case 'c':
				*buf++ = *u.pstr;
				size--;
				break;
			case 's':
				ptr = *u.ppstr;
				do {
					if (*ptr == 0)
						break;
					*buf++ = *ptr++;
				} while (--size);
				break;
			case 'z':
			case 'i':
			case 'd':	/* A lie, always prints unsigned */
			case 'u':
				itemp = *u.puint;
				ptr = buf;
				do {
					*buf++ = '0' + (itemp % 10);
				} while (--size && (itemp /= 10));
				rotate(ptr, buf);
				break;

			case 'p':
				*buf++ = '0';
				if (--size == 0)
					break;
				*buf++ = 'x';
				if (--size == 0)
					break;
			case 'X':
			case 'x':
				itemp = *u.puint;
				ptr = buf;
				do {
					*buf++ = hex[itemp & 0xF];
				} while (--size && (itemp >>= 4));
				rotate(ptr, buf);
				break;

			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case '-':
				ap -= sizeof(void *);
				escape = 2;
				break;

			default:
				*buf++ = c;
				size--;
				break;
			}
			break;
		}
	}
	*buf = '\0';			/* always zero terminate */
	return (osize - size);
}

#endif
