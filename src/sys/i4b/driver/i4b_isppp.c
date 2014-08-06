/*-
 * Copyright (c) 1997 Joerg Wunsch. All rights reserved.
 *
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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
 *	i4b_isppp.c - isdn4bsd kernel SyncPPP driver
 *	--------------------------------------------
 *
 * 	Uses Serge Vakulenko's sppp backend (originally contributed with
 *	the "cx" driver for Cronyx's HDLC-in-hardware device).  This driver
 *	is only the glue between sppp and i4b.
 *
 *	last edit-date: [Sat Mar  9 14:09:27 2002]
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/ioccom.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_sppp.h>

#include <sys/time.h>
#include <net/bpf.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

#define ISPPP_FMT	"isp%d: "

# ifndef IOCTL_CMD_T
#  define IOCTL_CMD_T u_long
# endif

#define PPP_HDRLEN  4 /* PPP header length in bytes */

struct i4bisppp_softc {
#if (__FreeBSD_version < 600031)
	struct sppp     sc_sppp_old; /* struct sppp starts with
				      * struct ifnet
				      */
#define IFP2SP(ifp) ((struct sppp *)(ifp))
#define SP2IFP(sp)  ((struct ifnet *)(sp))
#define __IF_ALLOC(sc, type, ptr)			\
{ *(ptr) = SP2IFP(&(sc)->sc_sppp_old);			\
  SP2IFP(&(sc)->sc_sppp_old)->if_type = (type); }
#else
#define __IF_ALLOC(sc, type, ptr) \
{ *(ptr) = if_alloc(type); }
#endif
	struct ifnet *sc_ifp;

	call_desc_t *sc_cdp;	/* ptr to call descriptor	*/

	fifo_translator_t *sc_fifo_translator; /* fifo translator */

#ifdef I4B_ACCOUNTING
	struct i4b_accounting sc_accounting;
#endif

	u_int32_t sc_unit;

} i4bisppp_softc[NI4BISPPP];

static int	i4bisppp_ioctl(struct ifnet *ifp, IOCTL_CMD_T cmd, caddr_t data);

#if 0
static void	i4bisppp_send(struct ifnet *ifp);
#endif

static void	i4bisppp_start(struct ifnet *ifp);

static void	i4bisppp_tls(struct sppp *sp);
static void	i4bisppp_tlf(struct sppp *sp);
static void	i4bisppp_state_changed(struct sppp *sp, int new_state);
static void	i4bisppp_negotiation_complete(struct sppp *sp);

/*===========================================================================*
 *			DEVICE DRIVER ROUTINES
 *===========================================================================*/

MODULE_VERSION(i4bisppp, 1);
MODULE_DEPEND(i4bisppp, sppp, 1, 1, 1);

/*---------------------------------------------------------------------------*
 *	interface attach routine at kernel boot time
 *---------------------------------------------------------------------------*/
static void
i4bispppattach(void *dummy)
{
	struct i4bisppp_softc *sc = i4bisppp_softc;
	struct ifnet *ifp;
	u_int32_t i;

#ifdef SPPP_VJ
	printf("i4bisppp: %d ISDN SyncPPP device(s) attached "
	       "(VJ header compression)\n", NI4BISPPP);
#else
	printf("i4bisppp: %d ISDN SyncPPP device(s) attached\n", 
	       NI4BISPPP);
#endif

	for(i = 0; i < NI4BISPPP; sc++, i++) {

		sc->sc_unit = i;

		__IF_ALLOC(sc, IFT_PPP, &ifp);
		if(ifp == NULL)
		{
			panic("%s: %s: cannot if_alloc()",
			      __FILE__, __FUNCTION__);
		}
		sc->sc_ifp = ifp;

		ifp->if_softc = sc;
		if_initname(ifp, "isp", i);
		ifp->if_mtu = PP_MTU;
		ifp->if_flags = IFF_SIMPLEX | IFF_POINTOPOINT;
		/* 
		 * XXX: If this "ifp" is detached, if_free_type() 
		 * must be used. Not sure if the following
		 * type makes sense.
		 */
		ifp->if_type = IFT_ISDNBASIC;

		ifp->if_ioctl = i4bisppp_ioctl;

		/* actually initialized by sppp_attach() */
		/* ifp->if_output = sppp_output; */

		ifp->if_start = i4bisppp_start;

		ifp->if_hdrlen = 0;
		ifp->if_addrlen = 0;
		ifp->if_snd.ifq_maxlen = IFQ_MAXLEN;

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

 		IFP2SP(ifp)->pp_tls = i4bisppp_tls;
		IFP2SP(ifp)->pp_tlf = i4bisppp_tlf;
		IFP2SP(ifp)->pp_con = i4bisppp_negotiation_complete;
		IFP2SP(ifp)->pp_chg = i4bisppp_state_changed;

		sppp_attach(ifp);

		if_attach(ifp);

		bpfattach(ifp, DLT_PPP, PPP_HDRLEN);
	}
	return;
}
SYSINIT(i4bispppattach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4bispppattach, NULL);

/*---------------------------------------------------------------------------*
 *	process ioctl
 *---------------------------------------------------------------------------*/
static int
i4bisppp_ioctl(struct ifnet *ifp, IOCTL_CMD_T cmd, caddr_t data)
{
	struct i4bisppp_softc *sc = ifp->if_softc;
#if 0
	struct sppp *sp = IFP2SP(ifp);
	struct ifaddr *ifa = (struct ifaddr *) data;
	struct ifreq *ifr = (struct ifreq *) data;
#endif
	int error;

	SC_LOCK(f,sc->sc_fifo_translator);
	error = sppp_ioctl(sc->sc_ifp, cmd, data);
	SC_UNLOCK(f);

	return error;
}

/*---------------------------------------------------------------------------*
 *	start output to ISDN B-channel
 *---------------------------------------------------------------------------*/
static void
i4bisppp_start(struct ifnet *ifp)
{
	struct i4bisppp_softc *sc = ifp->if_softc;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
	  if (!sppp_isempty(ifp))
	  {
	    /* connected
	     *
	     * ifp->if_flags |= IFF_OACTIVE; // - need to clear this somewhere
	     */

	    L1_FIFO_START(sc->sc_fifo_translator);
	  }
	},
	{
	  /* not connected */
	});

	return;
}

/*
 *===========================================================================*
 *			SyncPPP layer interface routines
 *===========================================================================*
 */

/*---------------------------------------------------------------------------*
 *	PPP this-layer-started action
 *---------------------------------------------------------------------------*
 */
static void
i4bisppp_tls(struct sppp *sp)
{
	struct i4bisppp_softc *sc = SP2IFP(sp)->if_softc;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
	    /* connected */
	},
	{
	    /* not connected */
	    i4b_l4_dialout(DRVR_ISPPP, sc->sc_unit);
	});
}

/*---------------------------------------------------------------------------*
 *	PPP this-layer-finished action
 *---------------------------------------------------------------------------*
 */
static void
i4bisppp_tlf(struct sppp *sp)
{
	struct i4bisppp_softc *sc = SP2IFP(sp)->if_softc;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
	    /* connected */
	    i4b_l4_drvrdisc(DRVR_ISPPP, sc->sc_unit);
	},
	{
	    /* not connected */
	});
}
/*---------------------------------------------------------------------------*
 *	PPP interface phase change
 *---------------------------------------------------------------------------*
 */
static void
i4bisppp_state_changed(struct sppp *sp, int new_state)
{
	struct i4bisppp_softc *sc = SP2IFP(sp)->if_softc;
	
	SC_LOCK(f,sc->sc_fifo_translator);
	if(sc->sc_cdp)
	{
		i4b_l4_ifstate_changed(sc->sc_cdp, new_state);
	}
	SC_UNLOCK(f);

	return;
}

/*---------------------------------------------------------------------------*
 *	PPP control protocol negotiation complete (run ip-up script now)
 *---------------------------------------------------------------------------*
 */
static void
i4bisppp_negotiation_complete(struct sppp *sp)
{
	struct i4bisppp_softc *sc = SP2IFP(sp)->if_softc;
	
	SC_LOCK(f,sc->sc_fifo_translator);
	if(sc->sc_cdp)
	{
		i4b_l4_negcomplete_ind(sc->sc_cdp);
	}
	SC_UNLOCK(f);

	return;
}

/*===========================================================================*
 *			ISDN INTERFACE ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	this routine is used to give a feedback from userland demon
 *	in case of dial problems
 *---------------------------------------------------------------------------*/
static void
i4bisppp_response_to_user(msg_response_to_user_t *mrtu)
{
	struct i4bisppp_softc *sc = &i4bisppp_softc[mrtu->driver_unit];
	struct sppp *sp = IFP2SP(sc->sc_ifp);

	NDBGL4(L4_ISPDBG, "isp%d: status=%d, cause=%d",
	       mrtu->driver_unit, mrtu->status, mrtu->cause);
	  
	if(DSTAT_IS_DIAL_FAILURE(mrtu->status))
	{
		struct mbuf *m;

		SC_LOCK(f,sc->sc_fifo_translator);

		NDBGL4(L4_ISPDBG, "isp%d: clearing queues", mrtu->driver_unit);

		if(!(sppp_isempty(sc->sc_ifp)))
		{
			while((m = sppp_dequeue(sc->sc_ifp)) != NULL)
			{
				m_freem(m);
			}
		}

		/*
		 * Ahh, sppp doesn't like to get a down event when
		 * dialing fails. So first tell it that we are up
		 * and then go down.
		 */
		sp->pp_up(sp);
		sp->pp_down(sp);

		SC_UNLOCK(f);
	}
	return;
}
	
/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	each time a packet is received or transmitted. It should
 *	be used to implement an activity timeout mechanism.
 *---------------------------------------------------------------------------*/
static void
i4bisppp_activity(struct i4bisppp_softc *sc)
{
	struct sppp *sp = IFP2SP(sc->sc_ifp);

	if (sc->sc_cdp) {
	    /* update cd->last_active_time */
	    sc->sc_cdp->last_active_time =
	      ((sp->pp_last_recv < sp->pp_last_sent) ?
	       sp->pp_last_sent : sp->pp_last_recv);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a new frame (mbuf) has been received
 *---------------------------------------------------------------------------*/
static void
i4bisppp_put_mbuf(struct fifo_translator *__f, struct mbuf *m)
{
	struct i4bisppp_softc *sc = __f->L5_sc;

	m->m_pkthdr.rcvif = sc->sc_ifp;
	m->m_pkthdr.len = m->m_len;

	microtime(&sc->sc_ifp->if_lastchange);

	sc->sc_ifp->if_ipackets++;
#if 0
	sc->sc_ifp->if_ibytes += m->m_pkthdr.len;
#endif

#if I4B_ACCOUNTING
	sc->sc_accounting.sc_inb += m->m_pkthdr.len;
#endif
	
#ifdef I4BISPPPDEBUG
	printf("%s: received packet!\n", __FUNCTION__);
#endif

	BPF_MTAP(sc->sc_ifp, m);

	sppp_input(sc->sc_ifp, m);

	/* must be after sppp_input */
	i4bisppp_activity(sc);

	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when the last frame has been sent out and there is no
 *	further frame (mbuf)
 *---------------------------------------------------------------------------*/
static struct mbuf *
i4bisppp_get_mbuf(struct fifo_translator *__f)
{
	struct i4bisppp_softc *sc = __f->L5_sc;
	struct ifnet *ifp = sc->sc_ifp;
	struct mbuf *m;

	m = sppp_dequeue(ifp);

	if(m)
	{
		BPF_MTAP(ifp, m);

		microtime(&ifp->if_lastchange);
#if 0
		ifp->if_obytes += m->m_pkthdr.len;
#endif

#if I4B_ACCOUNTING
		sc->sc_accounting.sc_outb += m->m_pkthdr.len;
#endif
		ifp->if_opackets++;

		/* must be after sppp_dequeue */
		i4bisppp_activity(sc);
	}

	return m;
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
static fifo_translator_t *
i4bisppp_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
		  struct i4b_protocol *pp, u_int32_t driver_type,
		  u_int32_t driver_unit, call_desc_t *cd)
{
	struct i4bisppp_softc *sc = &i4bisppp_softc[driver_unit];
	struct sppp *sp = IFP2SP(sc->sc_ifp);

	if(!pp)
	{
	  return (driver_unit < NI4BISPPP) ?
	    sc->sc_fifo_translator : FT_INVALID;
	}

	sc->sc_fifo_translator = f;

#if I4B_ACCOUNTING
	I4B_ACCOUNTING_UPDATE(&sc->sc_accounting,
			      sc->sc_fifo_translator,
			      DRVR_ISPPP,
			      driver_unit);
#endif

	if(pp->protocol_1)
	{
	  /* connected */

	  f->L5_sc = sc;

	  f->L5_PUT_MBUF = i4bisppp_put_mbuf;
	  f->L5_GET_MBUF = i4bisppp_get_mbuf;

	  sc->sc_cdp = cd;

	  sp->pp_up(sp);		/* tell PPP we are ready */

	  sp->pp_last_sent = sp->pp_last_recv = SECOND;

	}
	else
	{
	  /* not connected */

	  sc->sc_cdp = NULL;

				/* pp_down calls i4bisppp_tlf */
	  sp->pp_down(sp);	/* tell PPP we have hung up */
	}
	return f;
}
I4B_REGISTER(DRVR_ISPPP, i4bisppp_setup_ft, i4bisppp_response_to_user);
