/*-
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/net/if_var.h"
 */
#ifndef __FREEBSD_NET_IF_VAR_H__
#define __FREEBSD_NET_IF_VAR_H__

struct mbuf;
struct ifnet;

#define _IF_QFULL(ifq)          ((ifq)->ifq_len >= (ifq)->ifq_maxlen)
#define _IF_DROP(ifq)           ((ifq)->ifq_drops++)
#define _IF_QLEN(ifq)           ((ifq)->ifq_len)

#define _IF_ENQUEUE(ifq, m) do {                                \
        (m)->m_nextpkt = NULL;                                  \
        if ((ifq)->ifq_tail == NULL)                            \
                (ifq)->ifq_head = m;                            \
        else                                                    \
                (ifq)->ifq_tail->m_nextpkt = m;                 \
        (ifq)->ifq_tail = m;                                    \
        (ifq)->ifq_len++;                                       \
} while (0)

#define _IF_PREPEND(ifq, m) do {                                \
        (m)->m_nextpkt = (ifq)->ifq_head;                       \
        if ((ifq)->ifq_tail == NULL)                            \
                (ifq)->ifq_tail = (m);                          \
        (ifq)->ifq_head = (m);                                  \
        (ifq)->ifq_len++;                                       \
} while (0)

#define _IF_DEQUEUE(ifq, m) do {                                \
        (m) = (ifq)->ifq_head;                                  \
        if (m) {                                                \
                if (((ifq)->ifq_head = (m)->m_nextpkt) == NULL) \
                        (ifq)->ifq_tail = NULL;                 \
                (m)->m_nextpkt = NULL;                          \
                (ifq)->ifq_len--;                               \
        }                                                       \
} while (0)

#define _IF_POLL(ifq, m)        ((m) = (ifq)->ifq_head)

#define _IF_DRAIN(ifq) do {                                     \
        struct mbuf *m;                                         \
        for (;;) {                                              \
                _IF_DEQUEUE(ifq, m);                            \
                if (m == NULL)                                  \
                        break;                                  \
                m_freem(m);                                     \
        }                                                       \
} while (0)

#ifndef BPF_MTAP
#define BPF_MTAP(_ifp,_m) do {			\
        if ((_ifp)->if_bpf) {			\
                bpf_mtap((_ifp)->if_bpf, (_m));	\
        }					\
} while (0)
#endif

# ifndef M_PROTO1
#  define M_PROTO1 0x100000 /* driver specific "m_flags" */
# endif

# ifndef m_length
# define m_length __m_length
extern u_int32_t
__m_length(struct mbuf *m0, struct mbuf **last);
# endif

# ifndef IF_DUNIT_NONE
#  define IF_DUNIT_NONE (-1)
# endif

# ifndef pp_last_recv
#  define pp_last_recv pp_last_activity
# endif

# ifndef pp_last_sent
#  define pp_last_sent pp_last_activity
# endif

extern void if_printf(struct ifnet *, const char *, ...)
    __attribute__((__format__(__printf__,2,3)));

extern void
if_initname(struct ifnet *ifp, const char *name, int unit);

extern int
netisr_queue(u_int32_t num, struct mbuf *m);

#endif
