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

struct selinfo;

#define	vaccess(...) EPERM
#define	VWRITE 1
#define	VREAD 2

#define	PRIV_ROOT 1
#define	PRIV_DRIVER 2

int	copyin(const void *uaddr, void *kaddr, size_t len);
int	copyout(const void *kaddr, void *uaddr, size_t len);
int	priv_check(struct thread *td, int priv);
int	pause(const char *wmesg, int timo);

int	m_apply(struct mbuf *mbuf, int off, int len, int (*f) (void *arg, void *data, uint32_t len), void *arg);

/* Assume little endian */

#define	htole32(x) ((uint32_t)(x))
#define	le32toh(x) ((uint32_t)(x))

#define	htole16(x) ((uint16_t)(x))
#define	le16toh(x) ((uint16_t)(x))

struct selinfo {
	uint8_t	dummy;
};
