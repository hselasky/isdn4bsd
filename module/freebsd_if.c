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
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/net/if.c"
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <machine/stdarg.h>

#include <sys/freebsd_compat.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>

#include <net/netisr.h>

#undef __m_length
#undef if_printf
#undef if_initname
#undef netisr_queue

u_int32_t
__m_length(struct mbuf *m0, struct mbuf **last)
{
    struct mbuf *m;
    u_int32_t len = 0;

    for(m = m0; m != NULL; m = m->m_next)
    {
        len += m->m_len;
	if(m->m_next == NULL)
	{
	    break;
	}
    }
    if(last != NULL)
    {
        last[0] = m;
    }
    return (len);
}

#if (__NetBSD_Version__ < 500000000)
void
if_initname(struct ifnet *ifp, const char *name, int unit)
{
#if 0
    ifp->if_dname = name;
    ifp->if_dunit = unit;
    if(unit == IF_DUNIT_NONE)
      strlcpy(ifp->if_xname, name, sizeof(ifp->if_xname));
    else
#endif
      snprintf(ifp->if_xname, sizeof(ifp->if_xname), "%s%d", name, unit);
}
#endif

#if 0
int
if_printf(struct ifnet *ifp, const char * fmt, ...)
{
    va_list ap;

    printf("%s: ", ifp->if_xname);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    /* XXX should return the
     * number of characters
     * printed, but the
     * NetBSD kernel does
     * not support that!
     */
    return 2;
}
#endif

int
netisr_queue(u_int32_t num, struct mbuf *m)
{
    int error;

    if(IF_QFULL(&ipintrq))
    {
        IF_DROP(&ipintrq);
	m_freem(m);
	error = ENXIO;
    }
    else
    {
        IF_ENQUEUE(&ipintrq, m);
	schednetisr(num);
	error = 0;
    }
    return error;
}

