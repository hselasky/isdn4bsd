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
 *	i4b_rbch.c - device driver for raw B channel data
 *	-------------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/filio.h>
#include <sys/tty.h>
#include <sys/ioccom.h>
#include <sys/poll.h>

#include <net/if.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_rbch_ioctl.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

static struct rbch_softc {
	int sc_unit;		/* unit number */

	int sc_flags;		/* state of driver */
#define ST_IDLE		0x00
#define ST_ISOPEN	0x02
#define ST_RDWAITDATA	0x04
#define ST_WRWAITEMPTY	0x08
#define ST_NOBLOCK	0x10
#define ST_VBR          0x20 /* Variable Bit Rate */

	call_desc_t *sc_cd;	/* Call Descriptor */
	struct termios it_in;

	fifo_translator_t *sc_fifo_translator; /* fifo translator  */

	struct selinfo selp;		/* select / poll	*/

#if I4B_ACCOUNTING
	struct i4b_accounting sc_accounting;
#endif	
} rbch_softc[NI4BRBCH];

static 	d_open_t	i4b_rbch_open;
static 	d_close_t	i4b_rbch_close;
static 	d_read_t	i4b_rbch_read;
static 	d_write_t	i4b_rbch_write;
static 	d_ioctl_t	i4b_rbch_ioctl;
static 	d_poll_t	i4b_rbch_poll;

static cdevsw_t i4b_rbch__cdevsw = {
#ifdef D_VERSION
      .d_version =    D_VERSION,
#endif
      .d_open =       i4b_rbch_open,
      .d_close =      i4b_rbch_close,
      .d_read =       i4b_rbch_read,
      .d_write =      i4b_rbch_write,
      .d_ioctl =      i4b_rbch_ioctl,
      .d_poll =       i4b_rbch_poll,
      .d_name =       "i4brbch",
};

/*===========================================================================*
 *			DEVICE DRIVER ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	interface attach routine
 *---------------------------------------------------------------------------*/
static void
i4b_rbch_attach(void *dummy)
{
	struct cdev *dev;
	int i;

	printf("i4brbch: %d raw B channel access device(s) attached\n", 
	       NI4BRBCH);
	
	for(i=0; i < NI4BRBCH; i++)
	{
		struct rbch_softc *sc;

		sc = &rbch_softc[i];

		dev = make_dev(&i4b_rbch__cdevsw, i,
			       UID_ROOT, GID_WHEEL, 0600, "i4brbch%d", i);

		if(dev)
		{
		  dev->si_drv1 = sc;
		}

#if I4B_ACCOUNTING
		I4B_ACCOUNTING_INIT(&sc->sc_accounting);
#endif
		sc->sc_unit = i;
		sc->sc_flags = ST_IDLE;

		sc->it_in.c_ispeed = sc->it_in.c_ospeed = 64000;
		termioschars(&sc->it_in);
	}
	return;
}
SYSINIT(i4b_rbch_attach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4b_rbch_attach, NULL);

/*---------------------------------------------------------------------------*
 *	open rbch device
 *---------------------------------------------------------------------------*/
static int
i4b_rbch_open(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	struct rbch_softc *sc = dev->si_drv1;
	int error = 0;
	
	SC_LOCK(f,sc->sc_fifo_translator);

	if(sc->sc_flags & ST_ISOPEN)
	{
		error = EBUSY;
	}
	else
	{
		sc->sc_flags |= ST_ISOPEN;
	}

	sc->sc_flags &= ~ST_NOBLOCK; /* block by default */

	SC_UNLOCK(f);

	NDBGL4(L4_RBCHDBG, "unit %d, open", minor(dev));

	return(error);
}

/*---------------------------------------------------------------------------*
 *	close rbch device
 *---------------------------------------------------------------------------*/
static int
i4b_rbch_close(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	struct rbch_softc *sc = dev->si_drv1;
	int unit = minor(dev);

	SC_LOCK(f,sc->sc_fifo_translator);

	if(f)
	{
		/* connected */
		i4b_l4_drvrdisc(DRVR_RBCH, unit);
	}

	sc->sc_flags &= ~ST_ISOPEN;

	SC_UNLOCK(f);
	
	NDBGL4(L4_RBCHDBG, "unit %d, closed", unit);
	
	return(0);
}

/*---------------------------------------------------------------------------*
 *	read from rbch device
 *---------------------------------------------------------------------------*/
static int
i4b_rbch_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	struct rbch_softc *sc = dev->si_drv1;
	int unit = minor(dev);
	struct mbuf *m;
	int error = 0;

	NDBGL4(L4_RBCHDBG, "unit %d, enter read", unit);
	
	m = 0;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
		/* connected */

		while(_IF_QEMPTY(&f->rx_queue))
		{
			if(sc->sc_flags & ST_NOBLOCK)
			{
			  error = EWOULDBLOCK;
			}
			else
			{
			  NDBGL4(L4_RBCHDBG, "unit %d, wait read data", unit);
		
			  sc->sc_flags |= ST_RDWAITDATA;
		
			  FIFO_TRANSLATOR_SLEEP(f,&f->rx_queue,
						(PSOCK|PCATCH),
						"rrbch", 0, error);

			  /* sc->sc_flags &= ~ST_RDWAITDATA; */
			}
		}

		_IF_DEQUEUE(&f->rx_queue, m);
	},
	{
		/* not connected */

		if(sc->sc_flags & ST_NOBLOCK)
		{
		  error = EWOULDBLOCK;
		}
		else
		{
		  NDBGL4(L4_RBCHDBG, "unit %d, wait read init", unit);

		  FIFO_TRANSLATOR_GOTO_CONNECTED_CODE(f, &rbch_softc[unit],
						      (PSOCK|PCATCH),
						      "rrrbch", 0, error);

		  NDBGL4(L4_RBCHDBG, "unit %d, error %d tsleep", unit, error);
		}
	});

	if(m && m->m_len)
	{
		NDBGL4(L4_RBCHDBG, "unit %d, read %d bytes", unit, m->m_len);

		error = uiomove(m->m_data, m->m_len, uio);
	}
	else
	{
		NDBGL4(L4_RBCHDBG, "unit %d, error %d uiomove", unit, error);
		error = EIO;
	}
		
	if(m)
		m_freem(m);

	return(error);
}

/*---------------------------------------------------------------------------*
 *	write to rbch device
 *---------------------------------------------------------------------------*/
static int
i4b_rbch_write(struct cdev *dev, struct uio * uio, int ioflag)
{
	struct rbch_softc *sc = dev->si_drv1;
	int unit = minor(dev);
	struct mbuf *m;
	int error = 0;

	NDBGL4(L4_RBCHDBG, "unit %d, write", unit);	

	m = i4b_getmbuf(BCH_MAX_DATALEN, M_WAITOK);

	if(m)
	{
		m->m_len = min(BCH_MAX_DATALEN, uio->uio_resid);

		NDBGL4(L4_RBCHDBG, "unit %d, write %d bytes", unit, m->m_len);
		
		error = uiomove(m->m_data, m->m_len, uio);

		if(!error)
		{
		  FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
		  {
		    /* connected */

		    while(_IF_QFULL(&f->tx_queue))
		    {
		      if(sc->sc_flags & ST_NOBLOCK)
		      {
			error = EWOULDBLOCK;
			goto connected_done;
		      }
		      else
		      {
			NDBGL4(L4_RBCHDBG, "unit %d, write queue full", unit);
		
			sc->sc_flags |= ST_WRWAITEMPTY;

			FIFO_TRANSLATOR_SLEEP(f, &f->tx_queue,
					      (PSOCK|PCATCH),
					      "wrbch", 0, error);

			/* sc->sc_flags &= ~ST_WRWAITEMPTY; */
		      }
		    }

		    _IF_ENQUEUE(&f->tx_queue, m);
		    m = 0;

		    L1_FIFO_START(f);
		  }
		  connected_done:
		  ,
		  {
		    /* not connected */

		    if(sc->sc_flags & ST_NOBLOCK)
		    {
		      error = EWOULDBLOCK;
		    }
		    else
		    {
		      NDBGL4(L4_RBCHDBG, "unit %d, write wait init", unit);

		      FIFO_TRANSLATOR_GOTO_CONNECTED_CODE(f, &rbch_softc[unit],
							  (PSOCK|PCATCH),
							  "wrrbch", 0, error);

		      NDBGL4(L4_RBCHDBG, "unit %d, error %d tsleep init", unit, error);
		    }
		  });
		}
	}
	else
	{
	  error = ENOMEM;
	}

	if(m)
	{
	  m_freem(m);
	}

	return(error);
}

/*---------------------------------------------------------------------------*
 *	rbch device ioctl handling
 *---------------------------------------------------------------------------*/
static int
i4b_rbch_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	struct rbch_softc *sc = dev->si_drv1;
	int unit = minor(dev);
	int error = 0;

	SC_LOCK(f,sc->sc_fifo_translator);

	switch(cmd)
	{
		case FIOASYNC:	/* Set async mode */
			if (*(int *)data)
			{
				NDBGL4(L4_RBCHDBG, "unit %d, setting async mode", unit);
			}
			else
			{
				NDBGL4(L4_RBCHDBG, "unit %d, clearing async mode", unit);
			}
			break;

		case FIONBIO:
			if (*(int *)data)
			{
				NDBGL4(L4_RBCHDBG, "unit %d, setting non-blocking mode", unit);
				sc->sc_flags |= ST_NOBLOCK;
			}
			else
			{
				NDBGL4(L4_RBCHDBG, "unit %d, clearing non-blocking mode", unit);
				sc->sc_flags &= ~ST_NOBLOCK;
			}
			break;

		case TIOCCDTR:	/* Clear DTR */
			if(f)
			{
				/* connected */
				NDBGL4(L4_RBCHDBG, "unit %d, disconnecting for DTR down", unit);
				i4b_l4_drvrdisc(DRVR_RBCH, unit);
			}
			break;

		case I4B_RBCH_DIALOUT:
                {
			size_t l;

			for (l = 0; l < TELNO_MAX && ((char *)data)[l]; l++)
				;
			if (l)
			{
				NDBGL4(L4_RBCHDBG, "unit %d, attempting dialout to %s", unit, (char *)data);
				i4b_l4_dialoutnumber(DRVR_RBCH, unit, l, (char *)data);
				break;
			}
			/* FALLTHROUGH to SDTR */
		}

		case TIOCSDTR:	/* Set DTR */
			NDBGL4(L4_RBCHDBG, "unit %d, attempting dialout (DTR)", unit);
			i4b_l4_dialout(DRVR_RBCH, unit);
			break;

		case TIOCSETAW: /* drain output, set termios struct? */
		case TIOCSETA:	/* Set termios struct */
			break;

		case TIOCGETA:	/* Get termios struct */
			*(struct termios *)data = sc->it_in;
			break;

		case TIOCMGET:
			*(int *)data = TIOCM_LE|TIOCM_DTR|TIOCM_RTS|TIOCM_CTS|TIOCM_DSR;
			if(f)
			{
			  /* connected */
			  *(int *)data |= TIOCM_CD;
			}
			break;

			/* support the old VR_REQ to not
			 * break "ppp":
			 */
		case _IOR('R', 2, int[3]):
                {
			msg_vr_req_t *mvr = (void *)data;

			mvr->version = I4B_VERSION;
			mvr->release = I4B_REL;
			mvr->step = I4B_STEP;

			NDBGL4(L4_MSG, "unit %d, old ioctl command used: "
			       "please recompile application!", unit);
			break;
		}
		case I4B_RBCH_VR_REQ:
			i4b_version_request((void *)data);
			break;

		default:	/* Unknown stuff */
			NDBGL4(L4_RBCHDBG, "unit %d, ioctl, unknown cmd %lx", unit, (u_long)cmd);
			error = EINVAL;
			break;
	}
	SC_UNLOCK(f);
	return(error);
}

/*---------------------------------------------------------------------------*
 *	device driver poll
 *---------------------------------------------------------------------------*/
static int
i4b_rbch_poll(struct cdev *dev, int events, struct thread *td)
{
	struct rbch_softc *sc = dev->si_drv1;
	int revents = 0;	/* events we found */

	SC_LOCK(f,sc->sc_fifo_translator);

	if(f)
	{
		/* connected */
		if(!_IF_QFULL(&f->tx_queue))
		{
			revents |= (events & (POLLOUT|POLLWRNORM));
		}
		
		if(!_IF_QEMPTY(&f->rx_queue))
		{
			revents |= (events & (POLLIN|POLLRDNORM));
		}
	}

	if(revents == 0)
	{
		selrecord(td, &sc->selp);
	}

	SC_UNLOCK(f);
	return(revents);
}

/*===========================================================================*
 *			ISDN INTERFACE ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	feedback from daemon in case of dial problems
 *---------------------------------------------------------------------------*/
void
rbch_response_to_user(msg_response_to_user_t *mrtu)
{
}
	
/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	each time a packet is received or transmitted
 *---------------------------------------------------------------------------*/
static void
rbch_activity(struct rbch_softc *sc)
{
	if (sc->sc_cd) {
	    sc->sc_cd->last_active_time = SECOND;
	}
	selwakeup(&sc->selp);
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a new frame (mbuf) has been received
 *---------------------------------------------------------------------------*/
static void
rbch_put_mbuf(struct fifo_translator *f, struct mbuf *m)
{
	struct rbch_softc *sc = f->L5_sc;

	if((sc->sc_flags & ST_VBR) || !i4b_l1_bchan_tel_silence(m->m_data, m->m_len))
	{
	  rbch_activity(sc);
	}

	m->m_pkthdr.len = m->m_len;

	if (! _IF_HANDOFF(&f->rx_queue, m, NULL))
	{
	  NDBGL4(L4_RBCHDBG, "unit %d: hdlc rx queue full!", sc->sc_unit);
	}

	if(sc->sc_flags & ST_RDWAITDATA)
	{
		NDBGL4(L4_RBCHDBG, "unit %d, wakeup", sc->sc_unit);
		sc->sc_flags &= ~ST_RDWAITDATA;
		wakeup(&f->rx_queue);
	}
	else
	{
#if 0
		NDBGL4(L4_RBCHDBG, "unit %d, NO wakeup", sc->sc_unit);
#endif
	}
	selwakeup(&sc->selp);
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when the last frame has been sent out and there is no
 *	further frame (mbuf)
 *---------------------------------------------------------------------------*/
static struct mbuf *
rbch_get_mbuf(struct fifo_translator *f)
{
	struct mbuf *m;

	struct rbch_softc *sc = f->L5_sc;

	_IF_DEQUEUE(&f->tx_queue, m);

	if(!m)
	{
	  if(sc->sc_flags & ST_WRWAITEMPTY)
	  {
		NDBGL4(L4_RBCHDBG, "unit %d, wakeup", sc->sc_unit);
		sc->sc_flags &= ~ST_WRWAITEMPTY;
		wakeup(&f->tx_queue);
	  }
	  else
	  {
#if 0
		NDBGL4(L4_RBCHDBG, "unit %d, NO wakeup", sc->sc_unit);
#endif
	  }
	  selwakeup(&sc->selp);
	}

	if(m)
	{
	  if((sc->sc_flags & ST_VBR) || !i4b_l1_bchan_tel_silence(m->m_data, m->m_len))
	  {
	    rbch_activity(sc);
	  }
	}

	return m;
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
rbch_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	      struct i4b_protocol *pp, u_int32_t driver_type, 
	      u_int32_t driver_unit, call_desc_t *cd)
{
	struct rbch_softc *sc = &rbch_softc[driver_unit];

	if(!pp)
	{
	  return (driver_unit < NI4BRBCH) ?
	    sc->sc_fifo_translator : FT_INVALID;
	}

	sc->sc_fifo_translator = f;

	if(PROT_IS_HL_VBR(pp))
	  sc->sc_flags |= ST_VBR;
	else
	  sc->sc_flags &= ~ST_VBR;

#if I4B_ACCOUNTING
	if((sc->sc_flags & ST_VBR) || (!(pp->protocol_1)))
	{
	  I4B_ACCOUNTING_UPDATE(&sc->sc_accounting,
				sc->sc_fifo_translator,
				DRVR_RBCH,
				driver_unit);
	}
#endif

	if(pp->protocol_1)
	{
	  /* connected */

	  f->L5_sc = sc;

	  f->L5_PUT_MBUF = rbch_put_mbuf;
	  f->L5_GET_MBUF = rbch_get_mbuf;

	  NDBGL4(L4_RBCHDBG, "unit %d, connected, wakeup", driver_unit);

	  sc->sc_cd = cd;
	  wakeup(sc);
	}
	else
	{
	  /* not connected */
	  NDBGL4(L4_RBCHDBG, "unit %d, disconnect", driver_unit);

	  /* NOTE: if threads are sleeping in "i4b_rbch_read" or 
	   * "i4b_rbch_write" they will not be woken up !
	   */

	  sc->sc_cd = NULL;
	}

	return f;
}

