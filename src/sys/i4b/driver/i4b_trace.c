/* $FreeBSD$ */
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
 *	i4b_trace.c - device driver for trace data read device
 *	------------------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#ifdef I4B_GLOBAL_INCLUDE_FILE
#include I4B_GLOBAL_INCLUDE_FILE
#else
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioccom.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/filio.h>
#include <sys/ioccom.h>
#include <sys/poll.h>

#include <net/if.h>
#endif

#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_global.h>

struct i4b_trace_softc {
	uint16_t	sc_unit;
	struct _ifqueue sc_queue;
	uint8_t	sc_flags;
#       define          ST_OPEN         0x01
#       define          ST_WAIT_SLP     0x02
#       define          ST_WAIT_WUP     0x04
#	define		ST_WAIT_SELP    0x08
#       define          ST_BLOCKING     0x10
#	define          ST_CLOSING      0x20

	struct cdev *   sc_dev;
	struct mtx *    sc_mtx;
	struct selinfo  sc_selp;
};

static struct i4b_trace_softc i4b_trace_softc[MAX_CONTROLLERS];

static d_open_t	  i4btrc_open;
static d_close_t  i4btrc_close;
static d_read_t   i4btrc_read;
static d_ioctl_t  i4btrc_ioctl;
static d_poll_t   i4btrc_poll;

static cdevsw_t i4btrc_cdevsw = {
      .d_version =    D_VERSION,
      .d_open =       &i4btrc_open,
      .d_close =      &i4btrc_close,
      .d_read =       &i4btrc_read,
      .d_ioctl =      &i4btrc_ioctl,
      .d_poll =       &i4btrc_poll,
      .d_name =       "i4btrc",
      .d_flags =      D_TRACKCLOSE,
};

#define DEV2SC(dev) (dev)->si_drv1
#define DEV2CNTL(dev) (dev)->si_drv2

/*---------------------------------------------------------------------------*
 *	interface attach routine
 *---------------------------------------------------------------------------*/
static void
i4btrcattach(void *dummy)
{
	struct i4b_controller *cntl;
	struct i4b_trace_softc *sc;
	struct cdev *dev;
	uint16_t i;

	for(i=0; i < MAX_CONTROLLERS; i++)
	{
		cntl = CNTL_FIND(i);
		if(cntl == NULL)
		{
		    panic("%s: %s: controller %d == NULL!",
			  __FILE__, __FUNCTION__, i);
		}

		dev = make_dev(&i4btrc_cdevsw, i,
			       UID_ROOT, GID_WHEEL, 0600, "i4btrc%d", i);

		sc = &i4b_trace_softc[i];

		CNTL_LOCK(cntl);

		sc->sc_unit = i;
		sc->sc_mtx = CNTL_GET_LOCK(cntl);
		sc->sc_dev = dev;

		if(dev)
		{
		    DEV2SC(dev) = sc;
		    DEV2CNTL(dev) = cntl;
		}
		sc->sc_queue.ifq_maxlen = IFQ_MAXLEN;
		sc->sc_flags = 0;

		CNTL_UNLOCK(cntl);
	}

	printf("i4btrc: %d ISDN trace device(s) attached\n", 
	       MAX_CONTROLLERS);
	return;
}
SYSINIT(i4btrcattach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4btrcattach, NULL);

/*---------------------------------------------------------------------------*
 *	i4b_l1_trace_ind - queue a trace indication from layer 1
 *---------------------------------------------------------------------------*/
void
i4b_l1_trace_ind(i4b_trace_hdr_t *hdr, struct mbuf *m1)
{
	struct i4b_trace_softc *sc;
	struct mbuf *m2;
	struct mbuf *m3;

	if((hdr->unit < 0) ||
	   (hdr->unit >= MAX_CONTROLLERS))
	{
	    return;
	}

	/* XXX one could use "m_copypacket()" here,
	 * but "m1" might be modified after this 
	 * routine returns, so a writeable copy
	 * is preferred !
	 */
	sc = &i4b_trace_softc[hdr->unit];
	m2 = i4b_getmbuf(m1->m_len, M_NOWAIT);
	m3 = i4b_getmbuf(sizeof(*hdr), M_NOWAIT);

	/* setup header */

	hdr->trunc = 0;
	hdr->length = m_length(m1, NULL) + sizeof(*hdr);

	if(m2 && m3 && (!_IF_QFULL(&sc->sc_queue)) && 
	   (sc->sc_flags & ST_OPEN))
	{
	    mtx_assert(sc->sc_mtx, MA_OWNED);

	    m3->m_next = m2;
	    bcopy(hdr, m3->m_data, sizeof(*hdr));
	    bcopy(m1->m_data, m2->m_data, m1->m_len);

	    _IF_ENQUEUE(&sc->sc_queue, m3);
	    m2 = NULL;
	    m3 = NULL;

	    if(sc->sc_flags & ST_WAIT_SELP)
	    {
	        sc->sc_flags &= ~ST_WAIT_SELP;
		selwakeup(&sc->sc_selp);
	    }

	    if(sc->sc_flags & ST_WAIT_WUP)
	    {
	        sc->sc_flags &= ~ST_WAIT_WUP;
		wakeup(sc);
	    }
	}

	if(m2)
	  m_freem(m2);

	if(m3)
	  m_freem(m3);

	return;
}

/*---------------------------------------------------------------------------*
 *	open trace device
 *---------------------------------------------------------------------------*/
static int
i4btrc_open(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	struct i4b_trace_softc *sc = DEV2SC(dev);
	int error = 0;

	if(sc == NULL)
	{
	    return ENODEV;
	}

	mtx_lock(sc->sc_mtx);

	if(sc->sc_flags & (ST_OPEN|ST_CLOSING))
	{
	    error = EBUSY;
	    goto done;
	}

	sc->sc_flags |= (ST_OPEN|ST_BLOCKING);

	_IF_DRAIN(&sc->sc_queue);

 done:
	mtx_unlock(sc->sc_mtx);

	return(error);
}

/*---------------------------------------------------------------------------*
 *	close trace device
 *---------------------------------------------------------------------------*/
static int
i4btrc_close(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	struct i4b_trace_softc *sc = DEV2SC(dev);

	if(sc == NULL)
	{
	    return 0;
	}

	mtx_lock(sc->sc_mtx);

	if(sc->sc_flags & ST_OPEN)
	{
	    uint32_t temp = TRACE_OFF;
	    sc->sc_flags |= ST_CLOSING;

	    L1_COMMAND_REQ(CNTL_FIND(sc->sc_unit), CMR_SETTRACE, &temp);

	    while(sc->sc_flags & ST_WAIT_SLP)
	    {
	        wakeup(sc);
		(void) msleep(sc, sc->sc_mtx, 
			      (PSOCK|PCATCH), "I4B close wait", 0);
	    }

	    sc->sc_flags &= ~(ST_OPEN|ST_CLOSING);

	    _IF_DRAIN(&sc->sc_queue);
	}

	mtx_unlock(sc->sc_mtx);

	return 0;
}

/*---------------------------------------------------------------------------*
 *	read from trace device
 *---------------------------------------------------------------------------*/
static int
i4btrc_read(struct cdev *dev, struct uio * uio, int ioflag)
{
	struct i4b_trace_softc *sc = DEV2SC(dev);
	struct mbuf *m1 = NULL, *m2;
	int error = 0;

	if(sc == NULL)
	{
	    return ENODEV;
	}

	mtx_lock(sc->sc_mtx);

	if(sc->sc_flags & (ST_WAIT_SLP|ST_CLOSING))
	{
	    error = EBUSY;
	    goto done;
	}

	while(_IF_QEMPTY(&sc->sc_queue))
	{
	    if(!(sc->sc_flags & ST_BLOCKING))
	    {
	        error = EWOULDBLOCK;
		goto done;
	    }

	    sc->sc_flags |= (ST_WAIT_SLP|ST_WAIT_WUP);

	    error = msleep(sc, sc->sc_mtx, (PSOCK|PCATCH), 
			   "I4B trace wait", 0);

	    sc->sc_flags &= ~(ST_WAIT_SLP|ST_WAIT_WUP);

	    if(sc->sc_flags & ST_CLOSING)
	    {
		wakeup(sc);
	        error = EINTR;
		goto done;
	    }

	    if(error)
	    {
	        goto done;
	    }
	}
	_IF_DEQUEUE(&sc->sc_queue, m1);

 done:
	mtx_unlock(sc->sc_mtx);

	m2 = m1;
	while(m2)
	{
	    error = uiomove(m2->m_data, m2->m_len, uio);
	    if(error) break;

	    m2 = m2->m_next;
	}

	if(m1)
	  m_freem(m1);
	else
	  if(!error)
	    error = EIO;

	return(error);
}

/*---------------------------------------------------------------------------*
 *	poll device
 *---------------------------------------------------------------------------*/
static int
i4btrc_poll(struct cdev *dev, int events, struct thread *td)
{
	struct i4b_trace_softc *sc = DEV2SC(dev);
	int revents = 0;

	if(sc == NULL)
	{
	    return POLLNVAL;
	}

	mtx_lock(sc->sc_mtx);

	if(!_IF_QEMPTY(&sc->sc_queue))
	{
	    revents |= (events & (POLLIN|POLLRDNORM));
	}

	if(revents == 0)
	{
	    sc->sc_flags |= ST_WAIT_SELP;
	    selrecord(td, &sc->sc_selp);
	}

	mtx_unlock(sc->sc_mtx);
	return(revents);
}

/*---------------------------------------------------------------------------*
 *	device driver ioctl routine
 *---------------------------------------------------------------------------*/
static int
i4btrc_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	struct i4b_trace_softc * sc = DEV2SC(dev);
	struct i4b_controller *cntl = DEV2CNTL(dev);
	int error = 0;

	if(cntl == NULL)
	{
	    return ENODEV;
	}

	switch(cmd) {
	case I4B_TRC_SET:
	    (void) L1_COMMAND_REQ(cntl, CMR_SETTRACE, data);
	    break;

	case FIONBIO:
	    mtx_lock(sc->sc_mtx);
	    if (*(int *)data)
		sc->sc_flags &= ~ST_BLOCKING;
	    else
	        sc->sc_flags |= ST_BLOCKING;
	    mtx_unlock(sc->sc_mtx);
	    break;

	default:
	    error = ENOTTY;
	    break;
	}
	return (error);
}

