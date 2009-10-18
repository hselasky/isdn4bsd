/*-
 * Copyright (c) 2005-2009 Hans Petter Selasky. All rights reserved.
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
 *
 *---------------------------------------------------------------------------
 *
 *	i4b_queue.h - I4B queue kernel include file
 *	---------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_QUEUE_H_
#define	_I4B_QUEUE_H_

struct mbuf;

/*---------------------------------------------------------------------------*
 *	Definition of interface queue
 *---------------------------------------------------------------------------*/
#undef STRUCT_IFQUEUE
#define	STRUCT_IFQUEUE				\
        struct  mbuf *ifq_head;			\
        struct  mbuf *ifq_tail;			\
        int     ifq_len;			\
        int     ifq_maxlen;			\
					/**/

struct _ifqueue {
	STRUCT_IFQUEUE;
};

#undef _IF_DRAIN
#define	_IF_DRAIN(ifq) do {			\
        struct mbuf *m;				\
        for (;;) {				\
                _IF_DEQUEUE(ifq, m);		\
                if (m == NULL)			\
                        break;			\
                m_freem(m);			\
        }					\
} while (0)

#undef IF_QUEUE_GET
#define	IF_QUEUE_GET(what) what

#undef _IF_QUEUE_GET
#define	_IF_QUEUE_GET(what) what

#undef _IF_ENQUEUE_HEAD
#define	_IF_ENQUEUE_HEAD _IF_PREPEND

#undef _IF_QEMPTY
#define	_IF_QEMPTY(ifq)	((ifq)->ifq_len == 0)

#undef _IF_HANDOFF
#define	_IF_HANDOFF(ifq,m,ifp) _if_handoff(ifq,m)

#undef _IF_DEQUEUE
#define	_IF_DEQUEUE(ifq, m) do {                                \
        (m) = (ifq)->ifq_head;                                  \
        if (m) {                                                \
                if (((ifq)->ifq_head = (m)->m_nextpkt) == NULL) \
                        (ifq)->ifq_tail = NULL;                 \
                (m)->m_nextpkt = NULL;                          \
                (ifq)->ifq_len--;                               \
        }                                                       \
} while (0)


#undef _IF_PREPEND
#define	_IF_PREPEND(ifq, m) do {		\
        (m)->m_nextpkt = (ifq)->ifq_head;	\
        if ((ifq)->ifq_tail == NULL)		\
                (ifq)->ifq_tail = (m);		\
        (ifq)->ifq_head = (m);			\
        (ifq)->ifq_len++;			\
} while (0)

#undef _IF_ENQUEUE
#define	_IF_ENQUEUE(ifq, m) do {		\
        (m)->m_nextpkt = NULL;			\
        if ((ifq)->ifq_tail == NULL)		\
                (ifq)->ifq_head = m;		\
        else					\
                (ifq)->ifq_tail->m_nextpkt = m;	\
        (ifq)->ifq_tail = m;			\
        (ifq)->ifq_len++;			\
} while (0)

#undef _IF_QFULL
#define	_IF_QFULL(ifq)          ((ifq)->ifq_len >= (ifq)->ifq_maxlen)

#undef _IF_QLEN
#define	_IF_QLEN(ifq)           ((ifq)->ifq_len)

#undef _IF_POLL
#define	_IF_POLL(ifq, m)        ((m) = (ifq)->ifq_head)

static inline int
_if_handoff(struct _ifqueue *ifq, struct mbuf *m)
{
	if (_IF_QFULL(ifq)) {
		m_freem(m);
		return (0);
	}
	_IF_ENQUEUE(ifq, m);
	return (1);
}

#endif					/* _I4B_QUEUE_H_ */
