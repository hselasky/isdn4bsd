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

extern uint32_t __umodsi3(uint32_t rem, uint32_t div);

uint32_t
__umodsi3(uint32_t rem, uint32_t div)
{
	uint8_t c;

	if (((div - 1) & div) == 0) {
		/* power of two */
		return (rem & (div - 1));
	}
	c = 1;
	while (!(div & 0x80000000)) {
		div *= 2;
		c++;
	}

	do {
		if (rem >= div) {
			rem -= div;
		}
		div /= 2;
	} while (--c);

	return (rem);
}

extern uint32_t __udivsi3(uint32_t rem, uint32_t div);

uint32_t
__udivsi3(uint32_t rem, uint32_t div)
{
	uint32_t temp;
	uint8_t c;

	c = 1;
	while (!(div & 0x80000000)) {
		div *= 2;
		c++;
	}

	temp = 0;

	while (c--) {
		if (rem >= div) {
			rem -= div;
			temp |= 1 << c;
		}
		div /= 2;
	}

	return (temp);
}
