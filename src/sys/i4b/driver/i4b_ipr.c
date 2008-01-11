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
 *	i4b_ipr.c - isdn4bsd IP over raw HDLC ISDN network driver
 *	---------------------------------------------------------
 *
 *---------------------------------------------------------------------------*
 *
 *	statistics counter usage (interface lifetime):
 *	----------------------------------------------
 *	sc->sc_ifp->if_ipackets	# of received packets
 *	sc->sc_ifp->if_ierrors	# of error packets not going to upper layers
 *	sc->sc_ifp->if_opackets	# of transmitted packets
 *	sc->sc_ifp->if_oerrors	# of error packets not being transmitted
 *	sc->sc_ifp->if_collisions# of invalid ip packets after VJ decompression
 *	sc->sc_ifp->if_ibytes	# of bytes coming in from the line (before VJ)
 *	sc->sc_ifp->if_obytes	# of bytes going out to the line (after VJ)
 *	sc->sc_ifp->if_imcasts	  (currently unused)
 *	sc->sc_ifp->if_omcasts	# of frames sent out of the fastqueue
 *	sc->sc_ifp->if_iqdrops	# of frames dropped on input because queue full
 *	sc->sc_ifp->if_noproto	# of frames dropped on output because !AF_INET
 *
 *---------------------------------------------------------------------------*/

#include "opt_i4b.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/ioccom.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/time.h>
#include <net/if.h>
#include <net/if_types.h>
#include <net/netisr.h>
#include <net/bpf.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

#ifndef IFF_DRV_RUNNING
#define IFF_DRV_RUNNING IFF_RUNNING
#undef if_drv_flags
#define if_drv_flags if_flags
#endif

#ifdef IPR_VJ
#include <sys/malloc.h>
#include <net/slcompress.h>       
#define IPR_COMPRESS IFF_LINK0  /* compress TCP traffic */
#define IPR_AUTOCOMP IFF_LINK1  /* auto-enable TCP compression */

/*---------------------------------------------------------------------------
 * NOTICE: using NO separate buffer relies on the assumption, that the HSCX
 * IRQ handler _always_ allocates a single, continuous mbuf cluster large
 * enough to hold the maximum MTU size if the ipr interface !
 *
 * CAUTION: i have re-defined IPR_VJ_USEBUFFER because it makes problems
 *          with 2 i4b's back to back running cvs over ssh, cvs simply
 *          aborts because it gets bad data. Everything else (telnet/ftp?etc)
 *          functions fine. 
 *---------------------------------------------------------------------------*/
#define IPR_VJ_USEBUFFER	/* define to use an allocated separate buffer*/
				/* undef to uncompress in the mbuf itself    */
#endif /* IPR_VJ */

#define I4BIPRMTU	1500		/* regular MTU */
#define I4BIPRMAXMTU	2000		/* max MTU */
#define I4BIPRMINMTU	500		/* min MTU */

#define I4BIPRMAXQLEN	50		/* max queue length */

#define I4BIPRADJFRXP	1		/* adjust 1st rxd packet */

/* initialized by L4 */

struct ipr_softc {
#if (__FreeBSD_version < 600031)
	struct ifnet	sc_if_old;
#define __IF_ALLOC(sc, type, ptr) \
{ *(ptr) = &(sc)->sc_if_old; (sc)->sc_if_old.if_type = (type); }
#else
#define __IF_ALLOC(sc, type, ptr) \
{ *(ptr) = if_alloc(type); }
#endif
	struct ifnet *	sc_ifp;		/* network-visible interface	*/
	call_desc_t *	sc_cdp;		/* ptr to call descriptor	*/
	struct _ifqueue sc_fastq;	/* interactive traffic		*/
	struct _ifqueue sc_sendq;	/* send queue			*/

	struct usb_callout sc_callout;

	fifo_translator_t *sc_fifo_translator;	/* fifo translator      */

#if I4B_ACCOUNTING
	struct i4b_accounting sc_accounting;
#endif	

#ifndef IPR_LOG
#define IPR_LOG 0
#endif

#if IPR_LOG
	int		sc_log_first;	/* log first n packets          */
#endif

#ifdef IPR_VJ
	struct slcompress sc_compr;	/* tcp compression data		*/
#ifdef IPR_VJ_USEBUFFER
	u_int8_t	sc_cbuf[I4BIPRMAXMTU+128];  /* tcp decompression buffer */
#endif
#endif
	u_int32_t       sc_unit;

} ipr_softc[NI4BIPR];

enum ipr_states {
	ST_IDLE,			/* initialized, ready, idle	*/
	ST_DIALING,			/* dialling out to remote	*/
	ST_CONNECTED_W,			/* connected to remote		*/
	ST_CONNECTED_A,			/* connected to remote		*/
};

static int i4biprioctl(struct ifnet *ifp, u_long cmd, caddr_t data);

static int i4biproutput(struct ifnet *ifp, struct mbuf *m, struct sockaddr *dst, struct rtentry *rtp);
static void iprclearqueues(struct ipr_softc *sc);

/*===========================================================================*
 *			DEVICE DRIVER ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	interface attach routine at kernel boot time
 *---------------------------------------------------------------------------*/
static void
i4biprattach(void *dummy)
{
	struct ipr_softc *sc = &ipr_softc[0];
	struct ifnet *ifp;
	u_int32_t i;

#ifdef IPR_VJ
	printf("i4bipr: %d IP over raw HDLC ISDN device(s) attached "
	       "(VJ header compression)\n", NI4BIPR);
#else
	printf("i4bipr: %d IP over raw HDLC ISDN device(s) "
	       "attached\n", NI4BIPR);
#endif

	for(i=0; i < NI4BIPR; sc++, i++)
	{
		NDBGL4(L4_DIALST, "setting dial state to ST_IDLE");

		sc->sc_unit = i;

		__IF_ALLOC(sc, IFT_ISDNBASIC, &ifp);
		if(ifp == NULL)
		{
			panic("%s, %s: cannot if_alloc()", 
			      __FILE__, __FUNCTION__);
		}

		sc->sc_ifp = ifp;
		ifp->if_softc = sc;
		if_initname(ifp, "ipr", i);

#ifdef	IPR_VJ
		ifp->if_flags = IFF_POINTOPOINT | IFF_SIMPLEX | IPR_AUTOCOMP;
#else
		ifp->if_flags = IFF_POINTOPOINT | IFF_SIMPLEX;
#endif

		ifp->if_mtu = I4BIPRMTU;
		ifp->if_ioctl = i4biprioctl;
		ifp->if_output = i4biproutput;
		ifp->if_snd.ifq_maxlen = I4BIPRMAXQLEN; /* not used */

		sc->sc_sendq.ifq_maxlen = I4BIPRMAXQLEN;
		sc->sc_fastq.ifq_maxlen = I4BIPRMAXQLEN;

		ifp->if_ipackets = 0;
		ifp->if_ierrors = 0;
		ifp->if_opackets = 0;
		ifp->if_oerrors = 0;
		ifp->if_collisions = 0;
		ifp->if_ibytes = 0;
		ifp->if_obytes = 0;
		ifp->if_imcasts = 0;
		ifp->if_omcasts = 0;
		ifp->if_iqdrops = 0;
		ifp->if_noproto = 0;

#if I4B_ACCOUNTING
		I4B_ACCOUNTING_INIT(&sc->sc_accounting);
#endif

#if IPR_LOG
		sc->sc_log_first = IPR_LOG;
#endif

#ifdef IPR_VJ
		sl_compress_init(&sc->sc_compr, -1);
#endif
		if_attach(ifp);

		bpfattach(ifp, DLT_NULL, sizeof(u_int));
	}
	return;
}
SYSINIT(i4biprattach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4biprattach, NULL);

/*---------------------------------------------------------------------------*
 *	output a packet to the ISDN B-channel
 *---------------------------------------------------------------------------*/
static int
i4biproutput(struct ifnet *ifp, struct mbuf *m, struct sockaddr *dst,
	     struct rtentry *rtp)
{
	struct ipr_softc *sc;
	struct _ifqueue *ifq;
	struct ip *ip;
	int error = 0;
	
	sc = ifp->if_softc;

	SC_LOCK(f,sc->sc_fifo_translator);

	/* check for IP */
	
	if(dst->sa_family != AF_INET)
	{
		if_printf(ifp, "af%d not supported\n", dst->sa_family);
		m_freem(m);
		sc->sc_ifp->if_noproto++;
		sc->sc_ifp->if_oerrors++;

		error = EAFNOSUPPORT;
		goto done;
	}

	/* check interface state = UP */

	if(!(ifp->if_flags & IFF_UP))
	{
		NDBGL4(L4_IPRDBG, "ipr%d: interface is DOWN!", sc->sc_unit);
		m_freem(m);
		sc->sc_ifp->if_oerrors++;
		error = ENETDOWN;
		goto done;
	}

#if IPR_LOG
	if(sc->sc_log_first > 0)
	{
		--(sc->sc_log_first);
		i4b_l4_packet_ind(DRVR_IPR, sc->sc_unit, 1, m );
	}
#endif

	/* update access time */
	microtime(&sc->sc_ifp->if_lastchange);

	/*
	 * check, if type of service indicates interactive, i.e. telnet,
	 * traffic. in case it is interactive, put it into the fast queue,
	 * else (i.e. ftp traffic) put it into the "normal" queue
	 */

	ip = mtod(m, struct ip *);		/* get ptr to ip header */
	 
	if(ip->ip_tos & IPTOS_LOWDELAY)
	  ifq = &sc->sc_fastq;
	else
	  ifq = &sc->sc_sendq;

	/* check for space in choosen send queue */
	
	if(! _IF_HANDOFF(ifq, m, NULL))
	{
		NDBGL4(L4_IPRDBG, "ipr%d: send queue full!", sc->sc_unit);
		sc->sc_ifp->if_oerrors++;
		error = ENOBUFS;
		goto done;
	}
	
	NDBGL4(L4_IPRDBG, "ipr%d: add packet to send queue!", sc->sc_unit);

	if(f)
	{
	    /* connected */
	    L1_FIFO_START(sc->sc_fifo_translator);
	}
	else
	{
	    /* not connected */
	    /* dial if necessary */
	
	    NDBGL4(L4_IPRDBG, "ipr%d: send dial request message!", 
		   sc->sc_unit);

	    NDBGL4(L4_DIALST, "ipr%d: setting dial state to ST_DIALING", 
		   sc->sc_unit);
	    i4b_l4_dialout(DRVR_IPR, sc->sc_unit);
	}

 done:
	SC_UNLOCK(f);

	return error;
}

/*---------------------------------------------------------------------------*
 *	process ioctl
 *---------------------------------------------------------------------------*/
static int
i4biprioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ipr_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *)data;
	struct ifaddr *ifa = (struct ifaddr *)data;

	int error = 0;

	SC_LOCK(f,sc->sc_fifo_translator);

	switch(cmd) {
	case SIOCAIFADDR:	/* add interface address */
	case SIOCSIFADDR:	/* set interface address */
	case SIOCSIFDSTADDR:	/* set interface destination address */
	    if(ifa->ifa_addr->sa_family != AF_INET)
	      error = EAFNOSUPPORT;
	    else
	      sc->sc_ifp->if_flags |= IFF_UP;
	    break;

	case SIOCSIFFLAGS:	/* set interface flags */
	    if(!(ifr->ifr_flags & IFF_UP))
	    {
	        if(sc->sc_ifp->if_drv_flags & IFF_DRV_RUNNING)
		{
		    /* disconnect ISDN line */
		    i4b_l4_drvrdisc(DRVR_IPR, sc->sc_unit);
		    sc->sc_ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
		}

		/* empty queues */

		iprclearqueues(sc);
	    }

	    if(ifr->ifr_flags & IFF_DEBUG)
	    {
	        /* enable debug messages */
	    }
	    break;

	case SIOCSIFMTU:	/* set interface MTU */
	    if((ifr->ifr_mtu > I4BIPRMAXMTU) ||
	       (ifr->ifr_mtu < I4BIPRMINMTU))
	    {
	        error = EINVAL;
	    }
	    else
	    {
	        ifp->if_mtu = ifr->ifr_mtu;
	    }
	    break;

	default:
	    error = EINVAL;
	    break;
	}

	if(!error)
	{
		microtime(&sc->sc_ifp->if_lastchange);
	}

	SC_UNLOCK(f);
	return(error);
}

/*---------------------------------------------------------------------------*
 *	clear the interface's send queues
 *---------------------------------------------------------------------------*/
static void
iprclearqueues(struct ipr_softc *sc)
{
	SC_LOCK(f,sc->sc_fifo_translator);

	_IF_DRAIN(&sc->sc_fastq);
	_IF_DRAIN(&sc->sc_sendq);

	SC_UNLOCK(f);
	return;
}

/*===========================================================================*
 *			ISDN INTERFACE ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	this routine is used to give a feedback from userland daemon
 *	in case of dial problems
 *---------------------------------------------------------------------------*/
void
ipr_response_to_user(msg_response_to_user_t *mrtu)
{
	struct ipr_softc *sc = &ipr_softc[mrtu->driver_unit];

	NDBGL4(L4_IPRDBG, "ipr%d: status=%d",
	       mrtu->driver_unit, mrtu->status);

	if(DSTAT_IS_DIAL_FAILURE(mrtu->status))
	{
		NDBGL4(L4_IPRDBG, "ipr%d: clearing queues", mrtu->driver_unit);
		iprclearqueues(sc);
	}
}
	
/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	each time a packet is received or transmitted. It should
 *	be used to implement an activity timeout mechanism.
 *---------------------------------------------------------------------------*/
static void
ipr_activity(struct ipr_softc *sc)
{
	if (sc->sc_cdp) {
	    sc->sc_cdp->last_active_time = SECOND;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a new frame (mbuf) has been received
 *---------------------------------------------------------------------------*/
static void
ipr_put_mbuf(struct fifo_translator *__f, struct mbuf *m)
{
	register struct ipr_softc *sc = __f->L5_sc;

#ifdef IPR_VJ
#ifdef IPR_VJ_USEBUFFER
	u_int8_t *cp = &sc->sc_cbuf[0];
#endif	
	int len, c;
#endif
	ipr_activity(sc);
	
	m->m_pkthdr.rcvif = sc->sc_ifp;

	m->m_pkthdr.len = m->m_len;

	microtime(&sc->sc_ifp->if_lastchange);

	sc->sc_ifp->if_ipackets++;
	sc->sc_ifp->if_ibytes += m->m_pkthdr.len;

#ifdef	IPR_VJ
	if((c = (*(mtod(m, u_int8_t *)) & 0xf0)) != (IPVERSION << 4))
	{
		/* copy data to buffer */

		len = m->m_len;

#ifdef IPR_VJ_USEBUFFER
/* XXX */	m_copydata(m, 0, len, cp);
#endif
		
		if(c & 0x80)
		{
			c = TYPE_COMPRESSED_TCP;
		}
		else if(c == TYPE_UNCOMPRESSED_TCP)
		{
#ifdef IPR_VJ_USEBUFFER
			*cp &= 0x4f;		/* XXX */
#else
			*(mtod(m, u_int8_t *)) &= 0x4f; 			
#endif			
		}

		/*
		 * We've got something that's not an IP packet.
		 * If compression is enabled, try to decompress it.
		 * Otherwise, if `auto-enable' compression is on and
		 * it's a reasonable packet, decompress it and then
		 * enable compression.  Otherwise, drop it.
		 */
		if(sc->sc_ifp->if_flags & IPR_COMPRESS)
		{
#ifdef IPR_VJ_USEBUFFER
			len = sl_uncompress_tcp(&cp,len,(u_int)c,&sc->sc_compr);
#else
			len = sl_uncompress_tcp((u_int8_t **)&m->m_data, len,
					(u_int)c, &sc->sc_compr);
#endif			

			if(len <= 0)
			{
#ifdef DEBUG_IPR_VJ
				printf("i4b_ipr, ipr_put_mbuf: len <= 0 IPR_COMPRESS!\n");
#endif
				goto error;
			}
		}
		else if((sc->sc_ifp->if_flags & IPR_AUTOCOMP) &&
			(c == TYPE_UNCOMPRESSED_TCP) && (len >= 40))
		{
#ifdef IPR_VJ_USEBUFFER
			len = sl_uncompress_tcp(&cp,len,(u_int)c,&sc->sc_compr);
#else
			len = sl_uncompress_tcp((u_int8_t **)&m->m_data, len,
					(u_int)c, &sc->sc_compr);
#endif

			if(len <= 0)
			{
#ifdef DEBUG_IPR_VJ
				printf("i4b_ipr, ipr_put_mbuf: len <= 0 IPR_AUTOCOMP!\n");
#endif
				goto error;
			}

			sc->sc_ifp->if_flags |= IPR_COMPRESS;
		}
		else
		{
#ifdef DEBUG_IPR_VJ
			printf("i4b_ipr, ipr_input: invalid ip packet!\n");
#endif

error:
			sc->sc_ifp->if_ierrors++;
			sc->sc_ifp->if_collisions++;
			m_freem(m);
			return;
		}
#ifdef IPR_VJ_USEBUFFER
/* XXX */	m_copyback(m, 0, len, cp);
		m->m_pkthdr.len = len;
#else
		m->m_len = m->m_pkthdr.len = len;
#endif
	}
#endif

#if I4B_ACCOUNTING
	/* NB. do the accounting after decompression! */
	sc->sc_accounting.sc_inb += m->m_pkthdr.len;
#endif

#if IPR_LOG
	if(sc->sc_log_first > 0)
	{
		--(sc->sc_log_first);
		i4b_l4_packet_ind(DRVR_IPR, sc->sc_unit, 0, m);
	}
#endif

	if(sc->sc_ifp->if_bpf)
	{
	    struct mbuf *m_prep = i4b_getmbuf(4, M_NOWAIT);

	    if(m_prep)
	    {
	        /* prepend the address family as a four byte field */

	        m_prep->m_next = m;
		((u_int32_t *)(m_prep->m_data))[0] = htole32(AF_INET);
		m_prep->m_pkthdr.rcvif = sc->sc_ifp;
		BPF_MTAP(sc->sc_ifp, m_prep);
	        m_prep->m_next = NULL;
		m_freem(m_prep);
	    }
	}

	if(netisr_queue(NETISR_IP, m)) /* (0) on success */
	{
		NDBGL4(L4_IPRDBG, "ipr%d: ipintrq full!", 
		       sc->sc_unit);
		sc->sc_ifp->if_ierrors++;
		sc->sc_ifp->if_iqdrops++;		
	}
	return;
}

#ifdef I4BIPRADJFRXP
static void
ipr_put_mbuf_first_packet(struct fifo_translator *f, struct mbuf *m)
{
	/*
	 * The very first packet after the B channel is switched thru
	 * has very often several bytes of random data prepended. This
	 * routine looks where the IP header starts and removes the
	 * the bad data.
	 */

	unsigned char *mp = m->m_data;
	int i;
		
	for(i = 0; i < m->m_len; i++, mp++)
	{
		if( ((*mp & 0xf0) == 0x40) &&
		    ((*mp & 0x0f) >= 0x05) )
		{
				m->m_data = mp;
				m->m_len -= i;
				break;
		}
	}

	f->L5_PUT_MBUF = ipr_put_mbuf;
	ipr_put_mbuf(f,m);
}
#endif

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when the last frame has been sent out and there is no
 *	further frame (mbuf)
 *---------------------------------------------------------------------------*/
static struct mbuf *
ipr_get_mbuf_wait(struct fifo_translator *f)
{
	return NULL;
}

static struct mbuf *
ipr_get_mbuf(struct fifo_translator *f)
{
	register struct ipr_softc *sc = f->L5_sc;
	register struct mbuf *m;
#ifdef	IPR_VJ	
	struct ip *ip;	
#endif
	_IF_DEQUEUE(&sc->sc_fastq, m);
	if(m)
	{
		sc->sc_ifp->if_omcasts++;
	}
	else
	{
		_IF_DEQUEUE(&sc->sc_sendq, m);
	}

	if(m)
	{
		ipr_activity(sc);

		microtime(&sc->sc_ifp->if_lastchange);
		
		if(sc->sc_ifp->if_bpf)
		{
		    struct mbuf *m_prep = i4b_getmbuf(4, M_NOWAIT);

		    if(m_prep)
		    {
		        /* prepend the address family as a four byte field */

		        m_prep->m_next = m;
			((u_int32_t *)(m_prep->m_data))[0] = htole32(AF_INET);
			BPF_MTAP(sc->sc_ifp, m_prep);
			m_prep->m_next = NULL;
			m_freem(m_prep);
		    }
		}
	
#if I4B_ACCOUNTING
		sc->sc_accounting.sc_outb += m->m_pkthdr.len;	/* size before compression */
#endif

#ifdef IPR_VJ	
		if((ip = mtod(m, struct ip *))->ip_p == IPPROTO_TCP)
		{
			if(sc->sc_ifp->if_flags & IPR_COMPRESS)
			{
				*mtod(m, u_int8_t *) |= 
				  sl_compress_tcp(m, ip, &sc->sc_compr, 1);
			}
		}
#endif

		sc->sc_ifp->if_obytes += m->m_pkthdr.len;

		sc->sc_ifp->if_opackets++;
	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	start transmitting after connect
 *---------------------------------------------------------------------------*/
static void
i4bipr_connect_startio(struct ipr_softc *sc)
{
	SC_LOCK(f,sc->sc_fifo_translator);

	if(f)
	{
		/* connected */
		f->L5_GET_MBUF = ipr_get_mbuf;
		L1_FIFO_START(sc->sc_fifo_translator);
	}
	else
	{
		/* not connected */
	}

	SC_UNLOCK(f);
	return;
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
ipr_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	     struct i4b_protocol *pp, u_int32_t driver_type, 
	     u_int32_t driver_unit, call_desc_t *cd)
{
	struct ipr_softc *sc = &ipr_softc[driver_unit];

	if(!pp)
	{
	  return (driver_unit < NI4BIPR) ?
	    sc->sc_fifo_translator : FT_INVALID;
	}

	sc->sc_fifo_translator = f;

#if I4B_ACCOUNTING
	I4B_ACCOUNTING_UPDATE(&sc->sc_accounting,
			      sc->sc_fifo_translator,
			      DRVR_IPR,
			      driver_unit);
#endif

	if(pp->protocol_1)
	{
	  /* connected */

	  f->L5_sc = sc;

	  f->L5_PUT_MBUF = 
#ifdef I4BIPRADJFRXP
		ipr_put_mbuf_first_packet;
#else
		ipr_put_mbuf;
#endif
	  f->L5_GET_MBUF = ipr_get_mbuf;

	  sc->sc_cdp = cd;

	  NDBGL4(L4_DIALST, "ipr%d: setting dial state to ST_CONNECTED", driver_unit);

	  sc->sc_ifp->if_drv_flags |= IFF_DRV_RUNNING;

	  usb_callout_init_mtx(&sc->sc_callout, CNTL_GET_LOCK(cntl), 0);

	  /*
	   * Sometimes ISDN B-channels are switched thru asymmetic. This
	   * means that under such circumstances B-channel data (the first
	   * three packets of a TCP connection in my case) may get lost,
	   * causing a large delay until the connection is started.
	   * When the sending of the very first packet of a TCP connection
	   * is delayed for a to be empirically determined delay (close
	   * to a second in my case) those packets go thru and the TCP
	   * connection comes up "almost" immediately (-hm).
	   */

	  if(cd)
	  {
	    if(cd->isdntxdelay > 0)
	    {
		int delay;

		if (hz == 100) {
			delay = cd->isdntxdelay;	/* avoid any rounding */
		} else {
			delay = cd->isdntxdelay*hz;
			delay /= 100;
		}

		f->L5_GET_MBUF = ipr_get_mbuf_wait;

		usb_callout_reset(&sc->sc_callout, delay,
				(void *)(void *)&i4bipr_connect_startio, (void *)sc);
	    }

	    /* we don't need any negotiation - pass event back right now */
	    i4b_l4_negcomplete_ind(cd);
	  }
	}
	else
	{
	  /* not connected */

#if IPR_LOG
	  /* show next IPR_LOG packets again */
	  sc->sc_log_first = IPR_LOG;
#endif
	  sc->sc_cdp = NULL;

	  NDBGL4(L4_DIALST, "setting dial state to ST_IDLE");

	  sc->sc_ifp->if_drv_flags &= ~IFF_DRV_RUNNING;

	  usb_callout_stop(&sc->sc_callout);
	}

	return f;
}
