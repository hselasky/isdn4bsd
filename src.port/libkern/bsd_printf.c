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

	va_start(ap, fmt);
	val = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	printf("%s", buf);

	return (val);
}

#endif

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

int
vsnrprintf(char *dst, size_t size, int radix, const char *fmt, va_list ap)
{
	return (vsnprintf(dst, size, fmt, ap));
}
