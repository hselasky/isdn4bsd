/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	i4b_mbuf.c - mbuf handling support routines
 *	-------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <net/if.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_global.h>

__FBSDID("$FreeBSD: $");

#define I4B_MBUF_DEBUG

#if 0

#define MT_I4B_D	42
#define MT_I4B_B	43

#else

#define MT_I4B_D	MT_DATA
#define MT_I4B_B	MT_DATA

#endif

/*---------------------------------------------------------------------------*
 *	allocate mbuf space
 *---------------------------------------------------------------------------*/
struct mbuf *
i4b_getmbuf(int len, int how)
{
	struct mbuf *m;

#if ((defined(M_DONTWAIT) ||			\
      defined(M_TRYWAIT)) &&			\
     ((M_DONTWAIT != M_NOWAIT) ||		\
      (M_TRYWAIT != M_WAITOK)))
#error "\"how\" argument needs to be converted!"
#endif

	if(len > MCLBYTES)	/* if length > max extension size */
	{

#ifdef I4B_MBUF_DEBUG
		printf("%s: %s: ERROR: len(%d) > MCLBYTES(%d)\n",
		       __FILE__, __FUNCTION__, len, MCLBYTES);
#endif
		
		return(NULL);
	}

	MGETHDR(m, how, MT_I4B_B);	/* get mbuf with pkthdr */

	/* did we actually get the mbuf ? */

	if(!m)	
	{

#ifdef I4B_MBUF_DEBUG
		printf("%s: %s: ERROR: MGETHDR failed!\n",
		       __FILE__, __FUNCTION__);
#endif

		return(NULL);
	}

	if(len >= MHLEN)
	{
		MCLGET(m, how);

		if(!(m->m_flags & M_EXT))
		{
			m_freem(m);

#ifdef I4B_MBUF_DEBUG
			printf("%s: %s: ERROR: MCLGET failed, len(%d)\n", 
			       __FILE__, __FUNCTION__, len);
#endif
			
			return (NULL);
		}
	}

	m->m_len = len;

	return(m);
}

