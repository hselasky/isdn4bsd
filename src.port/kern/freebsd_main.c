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

#undef SYSINIT
#undef SYSUNINIT

#define	SYSINIT(uniquifier, subsystem, order, func, ident)	\
  extern const struct sysinit bsd_##uniquifier##_sys_init;

#define	SYSUNINIT(uniquifier, subsystem, order, func, ident)	\
  extern const struct sysinit bsd_##uniquifier##_sys_uninit;

#include <bsd_module_sysinit.h>
#include <bsd_module_sysuninit.h>

#undef SYSINIT
#undef SYSUNINIT

#define	SYSINIT(uniquifier, subsystem, order, func, ident)	\
  &bsd_##uniquifier##_sys_init,

#define	SYSUNINIT(uniquifier, subsystem, order, func, ident)	\
  &bsd_##uniquifier##_sys_uninit,

static const struct sysinit *sysinit_load[] = {
#include <bsd_module_sysinit.h>
};

static const struct sysinit *sysinit_unload[] = {
#include <bsd_module_sysuninit.h>
};

static void
do_sysinit(const struct sysinit **ppstart,
    const struct sysinit **ppend,
    uint32_t level_first,
    uint32_t level_last)
{
	const struct sysinit **ppsys;
	const struct sysinit *sys;
	uint32_t level_next;
	uint32_t order_first;
	uint32_t order_next;
	uint32_t temp1;
	uint32_t temp2;

	if (level_last < level_first) {
		level_last = level_first;
	}
	while (1) {
		level_next = level_last;
		order_first = 0;

		while (1) {
			order_next = 0xffffffff;

			for (ppsys = ppstart;
			    ppsys != ppend;
			    ppsys++) {
				sys = ppsys[0];

				temp1 = sys->subsystem;
				temp2 = sys->order;

				if ((temp1 == level_first) &&
				    (temp2 == order_first)) {
#if 1
					printf("calling %p(%p) @ %s:%d:\n",
					    sys->func, sys->udata,
					    sys->file, sys->line);
#endif
					(sys->func) (__DECONST(void *, sys->udata));
				}
				if ((temp1 > level_first) &&
				    (temp1 < level_next)) {
					level_next = temp1;
				}
				if ((temp2 > order_first) &&
				    (temp2 < order_next)) {
					order_next = temp2;
				}
			}

			if (order_first == 0xffffffff) {
				break;
			}
			order_first = order_next;
		}

		if (level_first == level_last) {
			break;
		}
		level_first = level_next;
	}
	return;
}

int
bsd_load_module(void)
{
	mtx_init(&Atomic, "Atomic", NULL, MTX_DEF);

	do_sysinit(sysinit_load, sysinit_load +
	    (sizeof(sysinit_load) / sizeof(sysinit_load[0])),
	    0, 0 - 1);

	printf("BSD Module Load Complete\n");

	return (0);			/* XXX success */
}

int
bsd_unload_module(void)
{
	do_sysinit(sysinit_unload, sysinit_unload +
	    (sizeof(sysinit_unload) / sizeof(sysinit_unload[0])),
	    0, 0 - 1);

	mtx_destroy(&Atomic);

	printf("BSD Module Unload Complete\n");

	return (0);			/* XXX success */
}
