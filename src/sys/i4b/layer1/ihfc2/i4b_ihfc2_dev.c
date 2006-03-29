/*-
 * Copyright (c) 2001-2005 Hans Petter Selasky. All rights reserved.
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
 *	i4b_ihfc2_dev.c - /dev/ihfcX.X interface
 *	----------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <i4b/layer1/ihfc2/i4b_ihfc2.h>
#include <i4b/layer1/ihfc2/i4b_ihfc2_ext.h>
#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

/*---------------------------------------------------------------------------*
 * :prototypes
 *---------------------------------------------------------------------------*/
static d_open_t		ihfc_dopen;
static d_close_t	ihfc_dclose;
static d_read_t		ihfc_dread;
static d_write_t	ihfc_dwrite;
static d_ioctl_t	ihfc_dioctl;

/*---------------------------------------------------------------------------*
 * :ihfc_cdevsw struct
 *---------------------------------------------------------------------------*/
static cdevsw_t
ihfc_cdevsw = {
#ifdef D_VERSION
  .d_version   = D_VERSION,
#endif
  .d_open      = ihfc_dopen,
  .d_close     = ihfc_dclose,
  .d_read      = ihfc_dread,
  .d_write     = ihfc_dwrite,
  .d_ioctl     = ihfc_dioctl,
  .d_name      = "ihfc",
};

/*---------------------------------------------------------------------------*
 * :shortcuts to the ihfc_softc struct
 *---------------------------------------------------------------------------*/
#define i4b_controller(dev)  (*((struct i4b_controller **)&((dev)->si_drv1)))
#define fifo_translator(dev) (*((fifo_translator_t **)&((dev)->si_drv2)))
#define fifo_transmit(ft)    (((ihfc_fifo_t *)((ft)->L5_fifo))+transmit)
#define fifo_receive(ft)     (((ihfc_fifo_t *)((ft)->L5_fifo))+receive)
#define channel(dev)         ((minor(dev) >> 16) % IHFC_DEVICES)

/* maximum frame length */
#define IHFC_MAX_FRAME  (BCH_MAX_DATALEN - 3)
	/* -3: Make room for 2 CRC + 1 STAT */

#define IHFC_DEVICES (IHFC_CHANNELS/2)

/*---------------------------------------------------------------------------*
 * :setup local devices, /dev/ihfcX.Y
 *---------------------------------------------------------------------------*/
u_int8_t
ihfc_setup_ldev(ihfc_sc_t *sc, u_int8_t *error)
{
	struct i4b_controller *cntl;
	struct cdev *dev;
	ihfc_fifo_t *f;
	u_int32_t unit_curr;
	u_int32_t unit_dev;
	u_int32_t channel;
	u_int32_t n;

	for(n = 0; 
	    n < sc->sc_default.d_sub_controllers;
	    n++)
	{
	    cntl = sc->sc_state[n].i4b_controller;
	    unit_curr = cntl->unit;
	    f = cntl->L1_fifo;

	    device_printf(sc->sc_device, "Creating /dev/ihfc%d.X.\n",
			  unit_curr);

	    for(channel = 0; 
		channel < cntl->L1_channel_end;
		channel++, f += 2)
	    {
	        if(sc->sc_test_dev[FIFO_NO(f)/2] == NULL)
		{
		    unit_dev = ((unit_curr*IHFC_DEVICES) + channel) << 16;

		    dev = make_dev(&ihfc_cdevsw, unit_dev,
				   UID_ROOT, GID_OPERATOR, 0600,
				   "ihfc%d.%d", unit_curr, channel);
		    if(!dev)
		    {
		        device_printf(sc->sc_device,
				      "WARNING: make_dev(lunit=0x%x,"
				      "device=ihfc%d.%d) failed!\n",
				      unit_dev, unit_curr, channel);
		    }
		    else
		    {
		        i4b_controller(dev) = cntl;
			fifo_translator(dev) = NULL;
			sc->sc_test_dev[FIFO_NO(f)/2] = dev;
		    }
		}
	    }
	}
	return IHFC_IS_ERR(error);
}

/*---------------------------------------------------------------------------*
 * :unsetup
 *---------------------------------------------------------------------------*/
void
ihfc_unsetup_ldev(ihfc_sc_t *sc)
{
	u_int32_t channel = IHFC_DEVICES;
	ihfc_fifo_t *f;

	FIFO_FOREACH(f,sc)
	{
	    IHFC_WAKEUP(f);
	}

	while(channel--)
	{
	    if(sc->sc_test_dev[channel])
	    {
	        /* in case the device is opened, close it: */
	        ihfc_dclose(sc->sc_test_dev[channel],0,0,0);

		destroy_dev(sc->sc_test_dev[channel]);
		sc->sc_test_dev[channel] = NULL;
	    }
	}
	return;
}

static void
ihfc_dev_put_mbuf(struct fifo_translator *ft, struct mbuf *m)
{
	ihfc_fifo_t *f = fifo_receive(ft);
#if 0
        _IF_HANDOFF(&ft->rx_queue, m, NULL);
#else
        _IF_HANDOFF(&f->ifqueue, m, NULL);
#endif
	IHFC_WAKEUP(f);
	return;
}

static struct mbuf *
ihfc_dev_get_mbuf(struct fifo_translator *ft)
{
	ihfc_fifo_t *f = fifo_transmit(ft);
        struct mbuf *m;
#if 0
        _IF_DEQUEUE(&ft->tx_queue, m);
#else
        _IF_DEQUEUE(&f->ifqueue, m);
#endif
        if(!m)
        {
	  IHFC_WAKEUP(f);
        }

        return m;
}

static void
ihfc_dev_rx_interrupt RXTX_INTERRUPT_T(ft,buf)
{
	ihfc_fifo_t *f = fifo_receive(ft);

	IHFC_WAKEUP(f);

	return;
}

static void
ihfc_dev_tx_interrupt RXTX_INTERRUPT_T(ft,buf)
{
	ihfc_fifo_t *f = fifo_transmit(ft);

	IHFC_WAKEUP(f);

	return;
}

fifo_translator_t *
ihfc_dev_setup_ft(i4b_controller_t *cntl, fifo_translator_t *ft, 
		  struct i4b_protocol *pp, u_int32_t driver_type,
		  u_int32_t driver_unit, call_desc_t *cd)
{
	ihfc_sc_t *sc = cntl->L1_sc;
	ihfc_fifo_t *f = cntl->L1_fifo;
	struct cdev *dev;

	driver_unit += (FIFO_NO(f)/2);
	dev = sc->sc_test_dev[driver_unit];

	IHFC_MSG("\n");

	if(!pp)
	{
	  return (driver_unit < IHFC_DEVICES) ?
	    fifo_translator(dev) : FT_INVALID;
	}

	fifo_translator(dev) = ft;

	if(pp->protocol_1)
	{
	  /* connected */

	  ft->L5_sc = sc;
          ft->L5_fifo = ft->L1_fifo;

          ft->L5_PUT_MBUF = ihfc_dev_put_mbuf;
          ft->L5_GET_MBUF = ihfc_dev_get_mbuf;
	  ft->L5_TX_INTERRUPT = ihfc_dev_tx_interrupt;
	  ft->L5_RX_INTERRUPT = ihfc_dev_rx_interrupt;

	  /* set protocol */
	  pp->protocol_1 = fifo_receive(ft)->default_prot;
	}
	else
	{
	  /* not connected */
	}
	return ft;
}

/*---------------------------------------------------------------------------*
 * :ihfc io control
 *---------------------------------------------------------------------------*/
static int
ihfc_dioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	ihfc_sc_t *sc = i4b_controller(dev)->L1_sc;
	int error = 0;

	IHFC_MSG("cmd=0x%x data %s, flag=0x%x\n",
		 (u_int32_t)cmd, (data) ? "!= 0" : "== 0", flag);

	FIFO_TRANSLATOR_ACCESS(ff_t,fifo_translator(dev),
	{
	  ihfc_fifo_t *fr = fifo_receive(ff_t);
	  ihfc_fifo_t *ft = fifo_transmit(ff_t);

	  /* connected */

	  switch (cmd)
	  {
		case SET_FRAME_LENGTH:

		  if(ft->mbuf_dev)
		  {
		    /* restore m_data and m_len */
		    ft->mbuf_dev->m_data -= (IHFC_MAX_FRAME - ft->mbuf_dev->m_len);
		    ft->mbuf_dev->m_len   = (IHFC_MAX_FRAME - ft->mbuf_dev->m_len);

		    /* m_len now holds the current
		     * number of bytes written
		     */
		    if(ft->mbuf_dev->m_len < *(u_int *)data)
		    {
		        IHFC_ERR("Cannot set this length, %d bytes!\n",
				 *(int *)data);

			/* free the current mbuf */
			I4B_FREEMBUF(ft,ft->mbuf_dev);
		    }
		    else
		    {
		        ft->mbuf_dev->m_len = *(u_int *)data;

			/* enqueue mbuf */
			_IF_ENQUEUE(&ft->ifqueue, ft->mbuf_dev);

			/* start TX fifo */
			ihfc_fifo_call(sc,ft);
		    }

		    /* mark mbuf as unused */
		    ft->mbuf_dev = NULL;
		  }
		  break;

		case GET_FRAME_LENGTH:

		  /*
		   * NOTE: according to stdlib the ANSI standard
		   * does not allow zero reads. Zero writes are
		   * allowed. For example if a program executes
		   * ``read(f, &buffer, 0)''  ihfc_dread  will
		   * [currently] not be called.  Therefore the
		   * first mbuf must be dequeued here and zero
		   * mbufs must be freed here. Also fifo setup
		   * must  be   called   from   here   through
		   * ihfc_check_state(sc,f)  else   the   fifo
		   * will not receive any data.
		   */

		  /* dequeue first mbuf */
		  if(!fr->mbuf_dev)
		  {
		    _IF_DEQUEUE(&fr->ifqueue, fr->mbuf_dev);
		  }

		  if(fr->mbuf_dev)
		  {
		    *(u_int *)data = fr->mbuf_dev->m_len;

		    if(fr->mbuf_dev->m_len == 0)
		    {
		      /* free the zero mbuf */
		      I4B_FREEMBUF(fr,fr->mbuf_dev);

		      /* mark mbuf as unused */
		      fr->mbuf_dev = 0;

		      /* get next mbuf */
		      _IF_DEQUEUE(&fr->ifqueue, fr->mbuf_dev);
		    }
		  }
		  else
		  {
		    *(u_int *)data = 0;
		  }
		  break;

		default:
		  IHFC_MSG("Unknown command: 0x%08x.\n", 
			   (u_int32_t)cmd);
		  error = EINVAL;
		  break;
	  }
	},
	{
	    /* not connected */
	    error = ENXIO;
	});
	return error;
}

/*---------------------------------------------------------------------------*
 * :ihfc open
 *---------------------------------------------------------------------------*/
static int
ihfc_dopen(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct i4b_controller *cntl = i4b_controller(dev);
	struct i4b_protocol p = { /* zero */ };
	u_int32_t channel = channel(dev);
#if DO_I4B_DEBUG
	ihfc_sc_t *sc = cntl->L1_sc;
#endif
	IHFC_MSG("oflags=0x%x\n",oflags);

	p.protocol_1 = P_HDLC; /* overwritten by setup_ft */

	i4b_setup_driver(cntl, channel, &p, 
			 DRVR_IHFC_DEV, channel, NULL);

	/*
	 * Provide a default protocol for ``cat''
	 * and other utilities.
	 *
	 * Allow open and write by different threads.
	 */

	return 0;
}

/*---------------------------------------------------------------------------*
 * :ihfc close
 *---------------------------------------------------------------------------*/
static int
ihfc_dclose(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct i4b_controller *cntl = i4b_controller(dev);
	u_int32_t channel = channel(dev);
#if DO_I4B_DEBUG
	ihfc_sc_t *sc = cntl->L1_sc;
#endif
	IHFC_MSG("fflag = 0x%x\n", (u_int32_t)fflag);

	i4b_setup_driver(cntl, channel, P_DISABLE, 
			 DRVR_IHFC_DEV, channel, NULL);

	return 0;
}

static int
__uiomove(fifo_translator_t *f, void *cp, int n, struct uio *uio)
{
	__typeof(f->refcount)
	  refcount = f->refcount;

	struct mtx *mtx = f->mtx;

	u_int8_t buffer[n];
	int error;

	if(uio->uio_rw == UIO_READ)
	{
		/* data to userland */
		bcopy(cp, &buffer[0], n);
	}

	mtx_unlock(mtx);

	error = uiomove(&buffer[0], n, uio);

	mtx_lock(mtx);

	if(f->refcount != refcount)
	{
		error = EINTR;
	}
	else
	{
		if(uio->uio_rw == UIO_WRITE)
		{
			/* data from userland */
			bcopy(&buffer[0], cp, n);
		}
	}
	return error;
}

/*---------------------------------------------------------------------------*
 * :ihfc read
 *---------------------------------------------------------------------------*/
static int
ihfc_dread(struct cdev *dev, struct uio *uio, int ioflag)
{
#if DO_I4B_DEBUG
	ihfc_sc_t *sc = i4b_controller(dev)->L1_sc;
#endif
	int error = 0;

	IHFC_MSG("\n");

	FIFO_TRANSLATOR_ACCESS(ft,fifo_translator(dev),
	{
	  ihfc_fifo_t *f  = fifo_receive(ft);

	  /* connected */

	  /* check uio->uio_resid in post
	   * so that zero  frames will be
	   * be dequeued if (m_len == 0)
	   * && (uio_resid == 0)
	   */

	  do {
		int len;

		if(f->prot_curr.protocol_1 == P_TRANS_RING)
		{
		  /*
		   * Transparent ringbuffer routines
		   */

		  /* read from buffer */
		  BUF_GET_READLEN(&f->buf,&len);

		  if(!len)
		  {
		    /* sleep */
		    f->state |= ST_TSLEEP;
		    FIFO_TRANSLATOR_SLEEP(ft,f,(PSOCK|PCATCH),
					  "ihfc_dev", /* 1*hz */0, error);
		    continue;
		  }

		  len = min(len, uio->uio_resid);

		  error = __uiomove(ft, f->buf.Dat_start, len, uio);

		  if(error) break;

		  /* release fifo space */
		  f->buf.Dat_start += len;

		} else if((f->prot_curr.protocol_1 == P_HDLC) ||
			  (f->prot_curr.protocol_1 == P_HDLC_EMU) ||
			  (f->prot_curr.protocol_1 == P_HDLC_EMU_D) ||
			  (f->prot_curr.protocol_1 == P_TRANS)) {

		  /*
		   * HDLC(mbuf) routines
		   */

		  if(f->mbuf_dev)
		  {
		    len = min(f->mbuf_dev->m_len,uio->uio_resid);

		    error = __uiomove(ft, f->mbuf_dev->m_data, len, uio);

		    if(error) break;

		    f->mbuf_dev->m_len  -= len;
		    f->mbuf_dev->m_data += len;

		    if(!f->mbuf_dev->m_len)
		    {
		      I4B_FREEMBUF(f,f->mbuf_dev);
		      f->mbuf_dev = 0;
		    }
		  }

		  if(!f->mbuf_dev)
		  {
		    _IF_DEQUEUE(&f->ifqueue, f->mbuf_dev);

		    if(!f->mbuf_dev)
		    {
			/* sleep */
			f->state |= ST_TSLEEP;
			FIFO_TRANSLATOR_SLEEP(ft,f,(PSOCK|PCATCH),
					      "ihfc_dev", /* 1*hz */0, error);
		    }
		  }
		}
		else
		{
		  error = ENXIO;
		  break;
		}

	  } while(uio->uio_resid);
	},
	{
	  /* not connected */
	  error = ENXIO;
	});

	return error;
}

/*---------------------------------------------------------------------------*
 * :ihfc write
 *---------------------------------------------------------------------------*/
static int
ihfc_dwrite(struct cdev *dev, struct uio *uio, int ioflag)
{
	ihfc_sc_t *sc = i4b_controller(dev)->L1_sc;
	int error = 0;

	IHFC_MSG("\n");

	FIFO_TRANSLATOR_ACCESS(ft,fifo_translator(dev),
	{
	  ihfc_fifo_t *f = fifo_transmit(ft);

	  /* connected */

	  /* check uio->uio_resid in post
	   * hence it is usually not zero
	   * when this function is called
	   * ...
	   */
	  do {
		int len;

		if(f->prot_curr.protocol_1 == P_TRANS_RING)
		{
		  /*
		   * Transparent ringbuffer routines
		   */

		  /* write to buffer */
		  BUF_GET_WRITELEN(&f->buf,&len);

		  if(!len)
		  {
		    /* sleep */
		    f->state |= ST_TSLEEP;
		    FIFO_TRANSLATOR_SLEEP(ft,f,(PSOCK|PCATCH),
					  "ihfc_dev", /* 1*hz */0, error);
		    continue;
		  }

		  len = min(len, uio->uio_resid);
		  error = __uiomove(ft, f->buf.Dat_end, len, uio);

		  if(error) break;

		  /* release fifo space */
		  f->buf.Dat_end += len;

		  /* start TX fifo */
		  ihfc_fifo_call(sc,f);

		} else if((f->prot_curr.protocol_1 == P_HDLC) ||
			  (f->prot_curr.protocol_1 == P_HDLC_EMU) ||
			  (f->prot_curr.protocol_1 == P_HDLC_EMU_D) ||
			  (f->prot_curr.protocol_1 == P_TRANS)) {
		  /*
		   * HDLC(mbuf) routines
		   */

		  if(!f->mbuf_dev)
		  {
		    if(_IF_QFULL(&f->ifqueue))
		    {
		      /* avoid using too many mbufs ... */

		      /* sleep */
		      f->state |= ST_TSLEEP;
		      FIFO_TRANSLATOR_SLEEP(ft,f,(PSOCK|PCATCH),
					    "ihfc_dev", /* 1*hz */0, error);
		      continue;
 		    }

		    f->mbuf_dev = i4b_getmbuf(IHFC_MAX_FRAME, M_NOWAIT);

		    if(!f->mbuf_dev)
		    {
		      error = ENOMEM;
		      break;
		    }
		  }

		  if(f->mbuf_dev)
		  {
		    len = min(f->mbuf_dev->m_len,uio->uio_resid);

		    error = __uiomove(ft, f->mbuf_dev->m_data, len, uio);

		    if(error) break;

		    /* update m_data and m_len */
		    f->mbuf_dev->m_len  -= len;
		    f->mbuf_dev->m_data += len;

		    if(!f->mbuf_dev->m_len)
		    {
		      /* restore m_data and m_len */
		      f->mbuf_dev->m_data -= (IHFC_MAX_FRAME - f->mbuf_dev->m_len);
		      f->mbuf_dev->m_len   = (IHFC_MAX_FRAME - f->mbuf_dev->m_len);

		      _IF_ENQUEUE(&f->ifqueue, f->mbuf_dev);
		      f->mbuf_dev = 0;

		      /* start TX fifo */
		      ihfc_fifo_call(sc,f);
		    }
		  }
		}
		else
		{
		  error = ENXIO;
		  break;
		}
	  } while(uio->uio_resid);
	},
	{
	  /* not connected */
	  error = ENXIO;
	});

	return error;
}
