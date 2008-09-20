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

int
copyin(const void *uaddr, void *kaddr, size_t len)
{
	return (EINVAL);
}

int
copyout(const void *kaddr, void *uaddr, size_t len)
{
	return (EINVAL);
}

int
priv_check(struct thread *td, int priv)
{
	return (EPERM);
}

int
pause(const char *wmesg, int timo)
{
	DROP_GIANT();
	delay((OSTIME) timo);
	PICKUP_GIANT();
	return (0);
}

int
m_apply(struct mbuf *mbuf, int off, int len,
    int (*f) (void *arg, void *data, uint32_t len), void *arg)
{
	/* not supported */
	return (EINVAL);
}

void
selrecord(struct thread *td, struct selinfo *sip)
{
	return;
}

void
selwakeup(struct selinfo *sip)
{
	return;
}

void
dev_ref(struct cdev *dev)
{
	return;
}

struct cdev *
make_dev(struct cdevsw *_devsw, int _minor, uid_t _uid, gid_t _gid,
    int _perms, const char *_fmt,...)
{
	return ((void *)1);
}

void
destroy_dev(struct cdev *_dev)
{
	return;
}

extern uint32_t fbsd_get_timer_us(void);

void
DELAY(uint32_t us)
{
	uint32_t start;
	uint32_t delta;

	start = fbsd_get_timer_us();
	while (1) {
		delta = fbsd_get_timer_us() - start;
		if (delta >= us) {
			break;
		}
	}
	return;
}
