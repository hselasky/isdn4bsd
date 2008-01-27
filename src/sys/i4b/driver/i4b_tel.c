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
 *	i4b_tel.c - device driver for ISDN telephony
 *	--------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioccom.h>
#include <sys/poll.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/lock.h>

#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_tel_ioctl.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

#if 0
#define TEL_USE_SIGNED_8_BIT
#endif
#if 1
#define TEL_NO_SOUND_BRIDGE
#else
#include <dev/sound/pcm/sound.h>
#include <dev/sound/pcm/dsp.h>
#endif

struct tel_parameters {
	u_int8_t	audio_amp;	/* audio amplification */
	u_int8_t	audio_flag;	/* conversion flag 
					 * bit usage:
					 *  0x10: mute
					 *  0x20: large buffer
					 *  0x40: see source code
					 *  0x80: see source code
					 */
 	u_int8_t	audio_input_bsubprot;
 	u_int8_t	audio_output_bsubprot;
	u_int8_t	audio_last_byte;/* last byte sent or received */
	u_int16_t	audio_total;	/* bytes */
	struct cdev *	audio_dev;	/* audio device */
};

typedef struct {

	/* used by ``/dev/i4btel'' device */

	fifo_translator_t *	sc_fifo_translator;	/* fifo translator */
	int 			audiofmt;	/* audio format conversion */
	call_desc_t *		cdp;		/* call descriptor pointer */
	struct tel_parameters   rd;  /* parameters for data read  from user-land */
	struct tel_parameters   wr;  /* parameters for data written to user-land */
	u_int8_t		use_sound_bridge;

	struct mbuf *		rd_mbuf; /* used by sound-bridge */

	struct i4b_tel_tones	tones;
	u_int8_t		tone_index;
	u_int16_t		tone_pos_1;
	u_int16_t		tone_pos_2;
	u_int16_t		tone_freq_1;
	u_int16_t		tone_freq_2;

	u_int16_t		tone_duration;
	struct selinfo		selp1;		/* select / poll */

	/* used by ``/dev/i4bteld'' device */

	u_int8_t		last_status;	/* last status from dialresponse */
	struct selinfo		selp2;		/* select / poll */

	/* shared */
	
	u_int8_t		state;	/* state of this unit */
#define ST_IDLE		 0x00		/* idle */
#define ST_ISOPEN1	 0x02		/* userland opened */
#define ST_ISOPEN2	 0x04		/* userland opened */
#define ST_RDWAIT_DATA	 0x08		/* userland read waiting */
#define ST_WRWAIT_EMPTY	 0x10		/* userland write waiting */
#define ST_TONE		 0x20		/* tone generator */
#define ST_RDWAIT_STATUS 0x40		/* userland read waiting */
#define ST_NO_STATUS     0x80		/* userland can't read result */
	u_int8_t		src_telno[TELNO_MAX];
	u_int8_t		src_display[DISPLAY_MAX];
	u_int8_t		src_user_user[USER_USER_MAX];
} tel_sc_t;

static tel_sc_t tel_sc[NI4BTEL];

static struct mbuf *
i4b_tel_tone(tel_sc_t *sc)
{
	struct mbuf *m;
	u_int32_t len;
	u_int8_t *ptr;

	m = i4b_getmbuf(BCH_MAX_DATALEN, M_NOWAIT);

	if(m == NULL)
	{
		NDBGL4(L4_ERR,"out of mbufs!");
		goto done;
	}

	ptr = m->m_data;
	len = BCH_MAX_DATALEN;

	while(len--)
	{
	check_duration:

	  if(sc->tone_duration == 0)
	  {
		/* this code will allow a frequency of 0Hz
		 * for example to generate pauses
		 */
		if((sc->tone_index >= I4B_TEL_MAXTONES) ||
		   (sc->tones.duration[sc->tone_index] == 0))
		{
			if(sc->state & ST_TONE)
			{
				sc->state &= ~ST_TONE;
				wakeup(&sc->tones);
			}

			sc->tone_pos_1 = 0;
			sc->tone_pos_2 = 0;

			sc->tone_freq_1 = 0;
			sc->tone_freq_2 = 0;
			break;
		}
		else
		{
			sc->tone_freq_1 = sc->tones.frequency_1[sc->tone_index];
			sc->tone_freq_1 %= 8000;

			if(sc->tone_freq_1 == 0)
			{
			  sc->tone_pos_1 = 0;
			}

			sc->tone_freq_2 = sc->tones.frequency_2[sc->tone_index];
			sc->tone_freq_2 %= 8000;

			if(sc->tone_freq_2 == 0)
			{
			  sc->tone_pos_2 = 0;
			}

			sc->tone_duration = sc->tones.duration[sc->tone_index];
			sc->tone_index++;

			goto check_duration;
		}
	  }

	  *ptr++ = i4b_signed_to_ulaw
	    ((i4b_sine_to_signed[sc->tone_pos_1] +
	      i4b_sine_to_signed[sc->tone_pos_2]) / 4);

	  sc->tone_pos_1 += sc->tone_freq_1;
	  if(sc->tone_pos_1 >= 8000)
	  {
	    sc->tone_pos_1 -= 8000;
	  }

	  sc->tone_pos_2 += sc->tone_freq_2;
	  if(sc->tone_pos_2 >= 8000)
	  {
	    sc->tone_pos_2 -= 8000;
	  }

	  sc->tone_duration--;
	}

	/* update length */
	m->m_len = ptr - ((__typeof(ptr))m->m_data);

	/* convert data */
	i4b_convert_bsubprot(m->m_data, m->m_len, 1, 1,
			     BSUBPROT_G711_ULAW, 
			     sc->rd.audio_output_bsubprot);
 done:
	return m;
}

/*===========================================================================*
 *			DEVICE DRIVER ROUTINES
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	open tel device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_open(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;
	int error = 0;
	
	SC_LOCK(f,sc->sc_fifo_translator);

	if(sc->state & ST_ISOPEN1)
	{
		error = EBUSY;
	}
	else
	{
		sc->state |= ST_ISOPEN1;

		/* reset format */
		sc->audiofmt = CVT_NONE;
		sc->rd.audio_input_bsubprot = BSUBPROT_PLAIN_ALAW;
		sc->rd.audio_amp = AUDIOAMP_DP;
		sc->wr.audio_output_bsubprot = BSUBPROT_PLAIN_ALAW;
		sc->wr.audio_amp = AUDIOAMP_DP;
		sc->use_sound_bridge = 0;
	}

	SC_UNLOCK(f);
	return(error);
}

/*---------------------------------------------------------------------------*
 *	open tel device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_dopen(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;
	int error = 0;
	
	SC_LOCK(f,sc->sc_fifo_translator);
	if(sc->state & ST_ISOPEN2)
	{
		error = EBUSY;
	}
	else
	{
		sc->state |= ST_ISOPEN2|ST_NO_STATUS;
	}
	SC_UNLOCK(f);
	return(error);
}

/*---------------------------------------------------------------------------*
 *	close tel device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_close(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;
	int error = 0;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
		/* connected */
		while(!_IF_QEMPTY(&f->tx_queue))
		{
			sc->state |= ST_WRWAIT_EMPTY;

			FIFO_TRANSLATOR_SLEEP(f,&f->tx_queue,
					      (PSOCK|PCATCH), "wtcl", 0, error);
		}

		while(sc->state & ST_TONE)
		{
		  FIFO_TRANSLATOR_SLEEP(f,&sc->tones,
					(PSOCK|PCATCH), "wtcl", 0, error);
		}
	},
	{
		/* not connected */
	});

	SC_LOCK(g,sc->sc_fifo_translator);
	sc->state &= ~ST_ISOPEN1;

	/* XXX */
	sc->use_sound_bridge = 0;
	SC_UNLOCK(g);
	return(error);
}

/*---------------------------------------------------------------------------*
 *	close teld device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_dclose(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;

	SC_LOCK(f,sc->sc_fifo_translator);
	sc->state &= ~ST_ISOPEN2;
	SC_UNLOCK(f);

	return 0;
}

/*---------------------------------------------------------------------------*
 *	i4b_tel_ioctl - device driver ioctl routine
 *---------------------------------------------------------------------------*/
static int
i4b_tel_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;
	int error = 0;

	SC_LOCK(f,sc->sc_fifo_translator);

	switch(cmd)
	{
	  case I4B_TEL_SET_SOUNDBRIDGE:
		sc->use_sound_bridge = 1;
		break;

	  case I4B_TEL_SET_AUDIOAMP:
		sc->wr.audio_amp = *(u_int8_t *)data;
		break;

	  case I4B_TEL_GET_AUDIOAMP:
		*(u_int8_t *)data = sc->wr.audio_amp;
		break;

	  case I4B_TEL_ENABLE_MUTE:
		sc->rd.audio_flag |= 0x10;
		break;

	  case I4B_TEL_DISABLE_MUTE:
		sc->rd.audio_flag &= ~0x10;
		break;

	  case I4B_TEL_ENABLE_LARGE_BUFFER:
		sc->rd.audio_flag |= 0x20;
		sc->wr.audio_flag |= 0x20;
		break;

	  case I4B_TEL_DISABLE_LARGE_BUFFER:
		sc->rd.audio_flag &= ~0x20;
		sc->wr.audio_flag &= ~0x20;
		break;

	  case I4B_TEL_GETAUDIOFMT:
		*(int *)data = sc->audiofmt;
		break;
			
	  case I4B_TEL_SETAUDIOFMT:
		switch (*(int *)data) {
		case CVT_NONE:
		case CVT_ULAW2ALAW:
		    /* ISDN: XXX, user: a-law */
		    sc->rd.audio_input_bsubprot = BSUBPROT_PLAIN_ALAW;
		    sc->wr.audio_output_bsubprot = BSUBPROT_PLAIN_ALAW;
		    break;

		case CVT_ALAW2ULAW:
		    /* ISDN: XXX, user: u-law */ 
		    sc->rd.audio_input_bsubprot = BSUBPROT_PLAIN_ULAW;
		    sc->wr.audio_output_bsubprot = BSUBPROT_PLAIN_ULAW;
		    break;

		case CVT_8BIT_SIGNED:
		    /* ISDN: XXX, user: 8-bit signed */
		    sc->rd.audio_input_bsubprot = BSUBPROT_SIGNED_8BIT;
		    sc->wr.audio_output_bsubprot = BSUBPROT_SIGNED_8BIT;
		    break;

		default:
		    error = ENODEV;
		    break;
		}

		if(error == 0)
		{
		    sc->audiofmt = *(int *)data;
		}
		break;
	
	  case I4B_TEL_EMPTYINPUTQUEUE:
		if(f)
		{
		  /* connected */
		  _IF_DRAIN(&f->rx_queue);
		}
		break;

	  case I4B_TEL_VR_REQ:
		i4b_version_request((void *)data);
		break;

	  case I4B_TEL_TONES:
		if(f)
		{
			struct i4b_tel_tones *tt = (void *)data;
			enum { _connected_code = 1 };
			struct mbuf *m;
			__typeof(f->refcount) 
			  refcount = f->refcount;

			/* connected */

			while((sc->state & ST_TONE) && 
			      (sc->tones.duration[sc->tone_index] != 0))
			{
				FIFO_TRANSLATOR_SLEEP(f,&sc->tones,
						      (PSOCK|PCATCH), "rtone", 0, error);
			}

			sc->tones = *tt;
			sc->tone_index = 0;
			sc->tone_duration = 0;

			sc->state |= ST_TONE;

			m = i4b_tel_tone(sc);

			if(m)
			{
				_IF_ENQUEUE(&f->tx_queue, m);
				L1_FIFO_START(f);
			}
			else
			{
				error = ENOMEM;
			}
		}
		else
		{
			/* not connected */
			error = EIO;
		}
		break;

	case I4B_TEL_GET_TELNO:
		strlcpy(data, &sc->src_telno[0], TELNO_MAX);
		break;

	case I4B_TEL_GET_DISPLAY:
		strlcpy(data, &sc->src_display[0], DISPLAY_MAX);
		break;

	case I4B_TEL_GET_SMS:
		strlcpy(data, &sc->src_user_user[0], USER_USER_MAX);
		break;
	
	  default:
		error = ENOTTY;
		break;
	}
	SC_UNLOCK(f);

	if(error)
	{
		NDBGL4(L4_ERR,"ioctl(0x%08x) error!", (u_int32_t)cmd);
	}
	return(error);
}

/*---------------------------------------------------------------------------*
 *	read from tel device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_read(struct cdev *dev, struct uio *uio, int ioflag)
{
	tel_sc_t *sc = dev->si_drv1;
	struct mbuf *m;
	int error = 0;

	m = 0;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
		  /* connected */

		  while(_IF_QEMPTY(&f->rx_queue))
		  {
			sc->state |= ST_RDWAIT_DATA;

			FIFO_TRANSLATOR_SLEEP(f,&f->rx_queue,(PSOCK|PCATCH),
					      "rtel",0,error);
		  }
	
		  _IF_DEQUEUE(&f->rx_queue, m);
	},
	{
	  /* not connected */
	});
		
	if(m && m->m_len > 0)
	{
			/* convert data */
			i4b_convert_bsubprot(m->m_data, m->m_len, 1, 1, 
					     (sc->wr.audio_flag & 0x10) ? 
					     BSUBPROT_MUTED : sc->wr.audio_input_bsubprot,
					     sc->wr.audio_output_bsubprot);

			error = uiomove(m->m_data, m->m_len, uio);

			NDBGL4(L4_TELDBG, "%s, mbuf (%d bytes), uiomove %d!",
			       devtoname(dev), m->m_len, error);
	}
	else
	{
			NDBGL4(L4_TELDBG, "%s, empty mbuf from queue!",
			       devtoname(dev));
			error = EIO;
	}

	if(m)
	{
	  m_freem(m);
	}
	return(error);
}

/*---------------------------------------------------------------------------*
 *	read from teld device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_dread(struct cdev *dev, struct uio *uio, int ioflag)
{
	tel_sc_t *sc = dev->si_drv1;
	int status = 0;
	int error = 0;

	FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	{
	  /* connected */
	  while(sc->state & ST_NO_STATUS)
	  {
	    sc->state |= ST_RDWAIT_STATUS;

	    FIFO_TRANSLATOR_SLEEP(f,&sc->last_status,
				  (PSOCK|PCATCH),
				  "rtel1",0,error);
	  }

	  sc->state |= ST_NO_STATUS;
	  status = sc->last_status;
	},
	{
	  /* not connected */
	  while(sc->state & ST_NO_STATUS)
	  {
	    sc->state |= ST_RDWAIT_STATUS;

	    FIFO_TRANSLATOR_GOTO_CONNECTED_CODE(f,&sc->last_status,
						(PSOCK|PCATCH),
						"rtel1",0,error);
	  }

	  sc->state |= ST_NO_STATUS;
	  status = sc->last_status;
	});

	if(!error)
	{
	  NDBGL4(L4_TELDBG, "%s, wait for status: 0x%02x!", devtoname(dev), sc->last_status);
	  error = uiomove(&status, 1, uio);
	}
	else
	{
	  NDBGL4(L4_TELDBG, "%s, wait for status error", devtoname(dev));
	  error = EIO;
	}
	return error;
}

/*---------------------------------------------------------------------------*
 *	write to tel device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_write(struct cdev *dev, struct uio * uio, int ioflag)
{
	tel_sc_t *sc = dev->si_drv1;
	struct mbuf *m;
	int error = 0;

	m = i4b_getmbuf(BCH_MAX_DATALEN, M_WAITOK);

	if(m)
	{
	  m->m_len = min(BCH_MAX_DATALEN, uio->uio_resid);
	
	  error = uiomove(m->m_data, m->m_len, uio);

	  if(!error)
	  {
	    /* convert data */
	    i4b_convert_bsubprot(m->m_data, m->m_len, 1, 1, 
				 (sc->rd.audio_flag & 0x10) ? 
				 BSUBPROT_MUTED : sc->rd.audio_input_bsubprot,
				 sc->rd.audio_output_bsubprot);

	    FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
	    {
		/* connected */

		sc->state &= ~ST_TONE;

		while(_IF_QFULL(&f->tx_queue))
		{
		  sc->state |= ST_WRWAIT_EMPTY;

		  FIFO_TRANSLATOR_SLEEP(f,&f->tx_queue,
					(PSOCK|PCATCH),"wtel",0,error);
		}

		_IF_ENQUEUE(&f->tx_queue, m);
		m = 0;

		L1_FIFO_START(f);
	    },
	    {
		/* not connected */
		error = EIO;
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
 *	write to teld device
 *---------------------------------------------------------------------------*/
static int
i4b_tel_dwrite(struct cdev *dev, struct uio * uio, int ioflag)
{
	int unit = minor(dev);
	int error;

#define CMDBUFSIZ 80 
	char cmdbuf[CMDBUFSIZ];
	int len = min(CMDBUFSIZ-1, uio->uio_resid);

	error = uiomove(cmdbuf, len, uio);

	if((len >= 1) && (!error))
	{
	  if(cmdbuf[0] == CMD_DIAL)
	  {
	    i4b_l4_dialoutnumber(DRVR_TEL, unit, len-1, &cmdbuf[1]);
	  }
	  else if(cmdbuf[0] == CMD_HANGUP)
	  {
	    i4b_l4_drvrdisc(DRVR_TEL, unit);
	  }
	  else if(cmdbuf[0] == CMD_KEYP)
	  {
	    i4b_l4_keypad(DRVR_TEL, unit, len-1, &cmdbuf[1]);
	  }
	  else if(cmdbuf[0] == CMD_ANSWER)
	  {
	    i4b_l4_drvranswer(DRVR_TEL, unit);
	  }
	  else if(cmdbuf[0] == CMD_REJECT)
	  {
	    i4b_l4_drvrreject(DRVR_TEL, unit);
	  }
	}

	return error;
}

/*---------------------------------------------------------------------------*
 *	device driver poll
 *---------------------------------------------------------------------------*/
static int
i4b_tel_poll(struct cdev *dev, int events, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;
	int revents = 0;	/* events we found */

	SC_LOCK(f,sc->sc_fifo_translator);

	if(f)
	{
	    /* connected */

	    if(!_IF_QFULL(&f->tx_queue))
	    {
	      NDBGL4(L4_TELDBG, "%s, POLLOUT", devtoname(dev));
	      revents |= (events & (POLLOUT|POLLWRNORM));
	    }
		
	    if(!_IF_QEMPTY(&f->rx_queue))
	    {
	      NDBGL4(L4_TELDBG, "%s, POLLIN", devtoname(dev));
	      revents |= (events & (POLLIN|POLLRDNORM));
	    }
	}

	if(revents == 0)
	{
	    NDBGL4(L4_TELDBG, "%s, selrecord", devtoname(dev));
	    selrecord(td, &sc->selp1);
	}
	SC_UNLOCK(f);
	return(revents);
}

/*---------------------------------------------------------------------------*
 *	device driver poll
 *---------------------------------------------------------------------------*/
static int
i4b_tel_dpoll(struct cdev *dev, int events, struct thread *td)
{
	tel_sc_t *sc = dev->si_drv1;
	int revents = 0;	/* events we found */

	SC_LOCK(f,sc->sc_fifo_translator);
	NDBGL4(L4_TELDBG, "%s, POLLOUT", devtoname(dev));
	revents |= (events & (POLLOUT|POLLWRNORM));

	if(!(sc->state & ST_NO_STATUS))
	{
		NDBGL4(L4_TELDBG, "%s, POLLIN, status = %d",
		       devtoname(dev), sc->last_status);
		revents |= (events & (POLLIN|POLLRDNORM));
	}

	if(revents == 0)
	{
		NDBGL4(L4_TELDBG, "%s, selrecord", devtoname(dev));
		selrecord(td, &sc->selp2);
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
tel_response_to_user(msg_response_to_user_t *mrtu)
{
	tel_sc_t *sc = &tel_sc[mrtu->driver_unit];

	NDBGL4(L4_TELDBG, "i4bteld%d,  status=%d, cause=0x%4x",
	       mrtu->driver_unit, mrtu->status, mrtu->cause);

	SC_LOCK(f,sc->sc_fifo_translator);

	/* overwrite last value, no buffering */
	sc->last_status = mrtu->status;
	sc->state &= ~ST_NO_STATUS;

	if(mrtu->status == DSTAT_INCOMING_CALL)
	{
		strlcpy(&sc->src_telno[0], &mrtu->src_telno[0],
			sizeof(sc->src_telno));

		strlcpy(&sc->src_display[0], &mrtu->src_display[0],
			sizeof(sc->src_display));

		strlcpy(&sc->src_user_user[0], &mrtu->src_user_user[0],
			sizeof(sc->src_user_user));
	}

	if(sc->state & ST_RDWAIT_STATUS)
	{
		sc->state &= ~ST_RDWAIT_STATUS;
		wakeup(&sc->last_status);
	}
	selwakeup(&sc->selp2);
	SC_UNLOCK(f);
	return;
}
	
/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	each time a packet is received or transmitted.
 *---------------------------------------------------------------------------*/
static void
tel_activity(tel_sc_t *sc)
{
	if (sc->cdp) {
	    sc->cdp->last_active_time = SECOND;
	}
	return;
}
	
/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a new frame (mbuf) has been received
 *---------------------------------------------------------------------------*/
static void
tel_put_mbuf(struct fifo_translator *f, struct mbuf *m)
{
	tel_sc_t *sc = f->L5_sc;

	if(!i4b_l1_bchan_tel_silence(m->m_data, m->m_len))
	{
	  tel_activity(sc);
	}

	_IF_HANDOFF(&f->rx_queue, m, NULL);
	
	if(sc->state & ST_RDWAIT_DATA)
	{
		sc->state &= ~ST_RDWAIT_DATA;
		wakeup(&f->rx_queue);
	}

	selwakeup(&sc->selp1);
	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when the last frame has been sent out and there is no
 *	further frame or mbuf
 *---------------------------------------------------------------------------*/
static struct mbuf *
tel_get_mbuf(struct fifo_translator *f)
{
	struct mbuf *m;

	tel_sc_t *sc = f->L5_sc;

	_IF_DEQUEUE(&f->tx_queue, m);

	if(!m)
	{
	  if(sc->state & ST_TONE)
	  {
	    m = i4b_tel_tone(sc);
	  }
	  else
	  {
	    if(sc->state & ST_WRWAIT_EMPTY)
	    {
		sc->state &= ~ST_WRWAIT_EMPTY;
		wakeup(&f->tx_queue);
	    }
	  }
	}

	if(m)
	{
	  if(!i4b_l1_bchan_tel_silence(m->m_data, m->m_len))
	  {
	    tel_activity(sc);
	  }
	}
	else
	{
	  selwakeup(&sc->selp1);
	}

	return m;
}

/*---------------------------------------------------------------------------*
 *	sound-bridge
 *---------------------------------------------------------------------------*/
#ifndef TEL_NO_SOUND_BRIDGE
#include <i4b/layer1/ihfc2/i4b_ihfc2.h> /* struct buffer */

#define TEL_AUDIORATE	8000 /*hz*/
#define TEL_MIN_BUFFER    32 /*bytes*/
#define TEL_MAX_BUFFER   800 /*bytes*/
#define TEL_AUDIODEVICE	"/dev/dsp"

#ifndef devsw
#define devsw(dev) ((dev)->si_devsw)
#endif

#if ((__FreeBSD_version > 0) && (__FreeBSD_version < 600034))
#define dev_lock(arg)
#define dev_unlock(arg)
#define dev_refl(arg)
#define dev_rel(arg)
#endif

static void
snd_close(struct cdev **pdev)
{
	struct cdev *dev = *pdev;

	if(dev)
	{
	  devsw(dev)->d_close(dev,0/*flags*/,0/*mode*/,NULL);
	  dev_rel(dev);
	  *pdev = NULL;
	}
	return;
}

static struct cdev *
getdevbyname(const char *name)
{
  struct cdev *dev = NULL;

#ifdef NDEVFSINO
  struct cdev **pdev;
  int inode;

  if((name[0] == '/') &&
     (name[1] == 'd') &&
     (name[2] == 'e') &&
     (name[3] == 'v') &&
     (name[4] == '/'))
  {
    name += 5;
  }

  dev_lock();

  for(inode = 0; inode < NDEVFSINO; inode++)
  {
	pdev = devfs_itod(inode);

	if(pdev)
	{
	  dev = *pdev;

	  if(dev)
	  {
	    if(dev->si_flags & SI_NAMED)
	    {
	      if(strcmp(dev->si_name,name) == 0)
	      {
		dev_refl(dev);
		goto found;
	      }
	    }
	  }
	}
  }

  dev = NULL;

 found:

  dev_unlock();
#else

  if((name[0] == '/') &&
     (name[1] == 'd') &&
     (name[2] == 'e') &&
     (name[3] == 'v') &&
     (name[4] == '/'))
  {
      name += 5;
  }

  dev_lock();

  LIST_FOREACH(dev, &(dsp_cdevsw.d_devs), si_list)
  {
      if(dev->si_flags & SI_NAMED)
      {
	  if(strcmp(dev->si_name,name) == 0)
	  {
	      dev_refl(dev);
	      goto found;
	  }
      }
  }

  dev = NULL;

 found:

  dev_unlock();
#endif

  return dev;
}

static int
snd_init(char *audiodevice, int flags, struct cdev **pdev)
{
	const char * msg;

	/**/
	struct cdev *dev;
	snd_chan_param pa;
	struct snd_size sz;
	snd_capabilities soundcaps;
	__typeof(soundcaps.formats) play_fmt, rec_fmt;

	dev = getdevbyname(audiodevice);

	if(dev == NULL)
	{
	  dev = NULL;
	  msg = "device not present";
	  goto error;
	}

	if(devsw(dev)->d_open(dev,flags,0/*mode*/,curthread) != 0)
	{
	  dev_rel(dev);
	  dev = NULL;
	  msg = "unable to open";
	  goto error;
	}

	if(devsw(dev)->d_ioctl(dev,AIOGCAP,(caddr_t)&soundcaps,0,0) != 0)
	{
	  msg = "ioctl AIOGCAP";
	  goto error;
	}
#if 0
	if(soundcaps.formats & AFMT_A_LAW)
	{
	    play_fmt = rec_fmt = AFMT_A_LAW;
	}
	else
#endif
#ifdef TEL_USE_SIGNED_8_BIT
	if(soundcaps.formats & AFMT_S8)
	{
	    play_fmt = rec_fmt = AFMT_S8;
	}
#else
	if(soundcaps.formats & AFMT_MU_LAW)
	{
	    play_fmt = rec_fmt = AFMT_MU_LAW;
	}
#endif
	else
	{
	    msg = "A-law or u-law not supported!";
	    goto error;
	}

	pa.play_format = play_fmt;
	pa.rec_format = rec_fmt;
	pa.play_rate = pa.rec_rate = TEL_AUDIORATE;

	if(devsw(dev)->d_ioctl(dev,AIOSFMT,(caddr_t)&pa,0,0) != 0)
	{
	  msg = "ioctl AIOSFMT";
	  goto error;
	}
#if 1
	sz.play_size = BCH_MAX_DATALEN;
	sz.rec_size = BCH_MAX_DATALEN;
#else
	sz.play_size = 800;
	sz.rec_size = 800;
#endif
	if(devsw(dev)->d_ioctl(dev,AIOSSIZE,(caddr_t)&sz,0,0) != 0)
	{
	  msg = "ioctl AIOSSIZE";
	  goto error;
	}

	*pdev = dev;

	return(0);

 error:
	*pdev = dev;

	snd_close(pdev);

	NDBGL4(L4_TELDBG, "%s: %s", audiodevice, msg);
	return(1);
}

static int
snd_open(tel_sc_t *sc)
{
	char *audio_dev_name = TEL_AUDIODEVICE "0.0";

	if(snd_init(audio_dev_name, FREAD|O_NONBLOCK, &sc->rd.audio_dev) == 0)
	{
		/* try full duplex first 
		 * (if "0.1" is used here the sound driver will
		 * forget to check for full duplex capability!)
		 */
		audio_dev_name = TEL_AUDIODEVICE "0.0";

		if(snd_init(audio_dev_name, FWRITE|O_NONBLOCK, 
			    &sc->wr.audio_dev) == 0)
		{
			NDBGL4(L4_TELDBG, "using full duplex");
		}
		else
		{
			/* try simplex */
			audio_dev_name = TEL_AUDIODEVICE "1.0";

			if(snd_init(audio_dev_name, FWRITE|O_NONBLOCK, 
				    &sc->wr.audio_dev) == 0)
			{
				NDBGL4(L4_TELDBG, "using two soundcards");
			}
			else
			{
				/* error */
				NDBGL1(L4_ERR, "cannot open dsp write device");
			}
		}
	}
	else
	{
		/* error */
		NDBGL1(L4_ERR, "cannot open dsp read device");
	}

	if((sc->rd.audio_dev == NULL) ||
	   (sc->wr.audio_dev == NULL))
	{
		snd_close(&sc->rd.audio_dev);
		snd_close(&sc->wr.audio_dev);

		NDBGL4(L4_ERR, "full-duplex not possible (closing)");
		return 0;
	}
	return 1;
}

/* snd_enter creates an overlay
 * buffer for the sound device
 */
static void
snd_enter(struct buffer *buf, struct snd_dbuf *b)
{
	/* read and write */
	buf->Buf_start = b->buf;
	buf->Buf_end = b->buf + b->bufsize;

	buf->Dat_start = b->rp + b->buf;
	buf->Dat_end = b->rp + b->rl + b->buf;

	if((b->rl == b->bufsize) && (b->rp != 0))
	{
	  buf->Dat_end--;
	}

	if(buf->Dat_end > buf->Buf_end)
	{
	  /* wrapped */
	  buf->Dat_end -= b->bufsize;
	}
	return;
}

static void
snd_exit(struct buffer *buf, struct snd_dbuf *b)
{
	/* read and write */
	BUF_GET_WRAPPED_READLEN(buf,&b->rl);
	b->rp = buf->Dat_start - buf->Buf_start;
	return;
}

/*---------------------------------------------------------------------------*
 * filter_BUF
 *
 * duplex data-filter, which is used between
 * the audio-device and the ISDN-device
 *---------------------------------------------------------------------------*/
static void
filter_BUF(struct buffer *in_buf, struct buffer *out_buf,
	   struct tel_parameters *parm)
{
  enum { AUDIO_LIMIT = 400 /* bytes */ };
  int in_max, out_max, len;
  u_int8_t expand, silence, *ptr;

  /* get wrapped remaining input length */
  BUF_GET_WRAPPED_READLEN(in_buf,&in_max);
  /* get wrapped unused output length */
  BUF_GET_APPROXIMATE_WRAPPED_WRITELEN(out_buf,&out_max);

#if 0
  /* limiting the transfer-length will cause
   * glitches in the audio-stream if the
   * interrupt-rate is not constant, and
   * is therefore disabled:
   */
  if((out_max < 256) ||
     (in_max < 256))
  {
    NDBGL4(L4_TELDBG,"too small transfer");
    /* avoid small transfers */
    return;
  }
#endif

  /* get wrapped, used output length */
  BUF_GET_WRAPPED_READLEN(out_buf,&len);

  expand = (len < ((parm->audio_flag & 0x20) ? TEL_MAX_BUFFER : TEL_MIN_BUFFER));
  silence = (len == 0) ? TEL_MIN_BUFFER : 0;

  NDBGL4(L4_TELDBG, "in_max:%d, out_max:%d, rlen=%d silence:%d expand:%d",
	 in_max, out_max, len, silence, expand);

  while(1)
  {
	BUF_GET_WRITELEN(out_buf,&out_max);

	if(out_max == 0)
	{
	  /* dump rest of input data in
	   * software buffer
	   */
	  in_buf->Dat_start =
	    in_buf->Dat_end;
	  break;
	}

	if(silence)
	{
	  ptr = out_buf->Dat_end;
	  len = min(silence, out_max);

	  /* update counters */
	  silence -= len;
	  out_buf->Dat_end += len;
	  out_max -= len;
	  parm->audio_total += len;

	  while(len--)
	  {
	    /* make silence by repeating
	     * last byte
	     */
	    *ptr++ = parm->audio_last_byte;
	  }

	  /* no data to convert */
	  continue;
	}

	if(/* read data */1)
	{
	  BUF_GET_READLEN(in_buf,&in_max);

	  if(in_max == 0)
	  {
		/* no more input data */
		break;
	  }

	  len = min(out_max,in_max);

	  /* copy sound-samples */
	  bcopy(in_buf->Dat_start,
		out_buf->Dat_end,len);

	  /* update counters */
	  in_buf->Dat_start += len;
	  in_max -= len;

	  goto convert;
	}

  convert:
	out_buf->Dat_end += len;
	out_max -= len;
	parm->audio_total += len;

	/* need to check that ptr is the same format !! */
	if(parm->audio_total >= AUDIO_LIMIT)
	{
	  if(len >= 3)
	  {
	    ptr = out_buf->Dat_end - 3;

	    if(expand)
	    {
	      if(out_max >= 1)
	      {
		/* expand 3 -> 4 */
		ptr[3] = ptr[2];

		/* update counters */
		len++;
		out_buf->Dat_end++;
		parm->audio_total -= AUDIO_LIMIT;
	      }
	    }
	    else
	    {
	      /* compress 3 -> 2 */

	      /* update counters */
	      len--;
	      out_buf->Dat_end--;
	      parm->audio_total -= AUDIO_LIMIT;
	    }
	  }
	}

	ptr = out_buf->Dat_end - len;

	if(len)
	{
	    /* convert data */
	    i4b_convert_bsubprot(ptr,len, parm->audio_amp, AUDIOAMP_DP,
				 (parm->audio_flag & 0x10) ? 
				 BSUBPROT_MUTED : parm->audio_input_bsubprot,
				 parm->audio_output_bsubprot);

	    /* update last byte */
	    parm->audio_last_byte = ptr[len-1];
	}
  }
  return;
}

static void
tel_rx_interrupt(struct fifo_translator *f, struct buffer *buf_in)
{
	tel_sc_t *sc = f->L5_sc;
	struct buffer out_buf;

	/* dsp_write() */

        struct pcm_channel *rdch, *wrch;

	tel_activity(sc);

	mtx_lock(&Giant);
	if(1)
	{
	    chn_get(sc->wr.audio_dev, &rdch, &wrch, SD_F_PRIO_WR);

	    if(!(wrch->flags & (CHN_F_MAPPED | CHN_F_DEAD)))
	    {
	        if(!(wrch->flags & CHN_F_RUNNING))
		{
		    wrch->flags |= CHN_F_RUNNING;
		}

		if(!(sc->wr.audio_flag & 0x80))
		{
		    /* get current DMA-counters! */
		    chn_intr(wrch);
		}

		snd_enter(&out_buf,wrch->bufsoft);

		filter_BUF(buf_in,&out_buf,&sc->wr);

		snd_exit(&out_buf,wrch->bufsoft);

		if(!(wrch->flags & CHN_F_TRIGGERED))
		{
		    chn_start(wrch, 0);
		}
#if 1
		if(!(sc->wr.audio_flag & 0x80))
		{
		    /* get current DMA-counters! */
		    chn_intr(wrch);
		}
#endif
	    }

	    chn_rel(sc->wr.audio_dev, rdch, wrch, SD_F_PRIO_WR);
	    mtx_unlock(&Giant);
	}
	return;
}

static void
tel_tx_interrupt(struct fifo_translator *f, struct buffer *out_buf)
{
	tel_sc_t *sc = f->L5_sc;
	struct buffer buf_in;

	/* dsp_read() */

        struct pcm_channel *rdch, *wrch;

	/* fill out_buf with data 
	 * from /dev/i4btelX first
	 */
	if(sc->rd_mbuf == NULL)
	{
		sc->rd_mbuf = tel_get_mbuf(f);
	}

	while(sc->rd_mbuf)
	{
		int len;

		BUF_GET_WRITELEN(out_buf,&len);

		if(len == 0)
		{
			break;
		}

		if(len > sc->rd_mbuf->m_len)
		{
			len = sc->rd_mbuf->m_len;
		}

		/* copy sound-samples */
		bcopy(sc->rd_mbuf->m_data,
		      out_buf->Dat_end,len);

		out_buf->Dat_end += len;
		sc->rd_mbuf->m_len -= len;
		sc->rd_mbuf->m_data += len;

		if(sc->rd_mbuf->m_len == 0)
		{
			/* get next mbuf */
			sc->rd_mbuf = tel_get_mbuf(f);
		}
	}

	/* fill rest of out_buf with
	 * data from sound-card
	 */
	tel_activity(sc);

	/* XXX the sound system seems 
	 * to require Giant !
	 */
	mtx_lock(&Giant);

	if(1)
	{
	    chn_get(sc->rd.audio_dev, &rdch, &wrch, SD_F_PRIO_RD);

	    if(!(rdch->flags & (CHN_F_MAPPED | CHN_F_DEAD)))
	    {
	        if(!(rdch->flags & CHN_F_TRIGGERED))
		{
		    chn_start(rdch, 0);
		}

		if(!(rdch->flags & CHN_F_RUNNING))
		{
		    rdch->flags |= CHN_F_RUNNING;
		}

		if(!(sc->rd.audio_flag & 0x80))
		{
		    /* get current DMA-counters! */
		    chn_intr(rdch);
		}

		snd_enter(&buf_in,rdch->bufsoft);

		filter_BUF(&buf_in,out_buf,&sc->rd);

		snd_exit(&buf_in,rdch->bufsoft);
	    }

	    chn_rel(sc->rd.audio_dev, rdch, wrch, SD_F_PRIO_RD);
	    mtx_unlock(&Giant);
	}
	else
	{
	    /* if we cannot lock, then we
	     * have to send some dummy data
	     * to avoid not beeing called back !
	     */
	    int len;

	    BUF_GET_WRITELEN(out_buf, &len);

	    if(len)
	    {
	        *(u_int8_t *)(out_buf->Dat_end) = sc->rd.audio_last_byte;
		out_buf->Dat_end += 1;
	    }
	}
	return;
}
#endif

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
tel_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	     struct i4b_protocol *pp, u_int32_t driver_type, 
	     u_int32_t driver_unit, call_desc_t *cd)
{
	tel_sc_t *sc = &tel_sc[driver_unit];

	if(!pp)
	{
	  return (driver_unit < NI4BTEL) ?
	    sc->sc_fifo_translator : FT_INVALID;
	}

	NDBGL4(L4_TELDBG, "i4btel%d", driver_unit);

	if(f)
	{
	  /* need to update ``sc_fifo_translator''
	   * last, because it is used to compute
	   * addresses for ``wakeup'' at
	   * disconnect!
	   */
	  sc->sc_fifo_translator = f;
	}
	else
	{
	  f = sc->sc_fifo_translator;
	  sc->sc_fifo_translator = 0;
	}

	if(pp->protocol_1)
	{
	  /* connected */

	  f->L5_sc = sc;

	  f->L5_PUT_MBUF     = tel_put_mbuf;
	  f->L5_GET_MBUF     = tel_get_mbuf;
#ifndef TEL_NO_SOUND_BRIDGE
	  f->L5_TX_INTERRUPT = tel_tx_interrupt;
	  f->L5_RX_INTERRUPT = tel_rx_interrupt;
#endif
	  sc->cdp = cd;

	  sc->rd.audio_output_bsubprot = pp->protocol_4;
	  sc->wr.audio_input_bsubprot = pp->protocol_4;

	  if(sc->use_sound_bridge)  /* driver_type == DRVR_TEL_SB */
	  {
#ifndef TEL_NO_SOUND_BRIDGE
		/* open sound-brigde */
		if(snd_open(sc))
		{
		  /* force ringbuffers! */
		  pp->protocol_1 = P_TRANSPARENT_RING;

#ifdef TEL_USE_SIGNED_8_BIT
		  sc->rd.audio_input_bsubprot = BSUBPROT_SIGNED_8BIT;
		  sc->wr.audio_output_bsubprot = BSUBPROT_SIGNED_8BIT;
#else
		  sc->rd.audio_input_bsubprot = BSUBPROT_PLAIN_ULAW;
		  sc->wr.audio_output_bsubprot = BSUBPROT_PLAIN_ULAW;
#endif
		}
		else
#endif
		{
		  /* error */
		  /* see unsetup under
		   * not connected
		   */
		  pp->protocol_1 = P_DISABLE;
		}
	  }
	}
	else
	{
	  /* not connected */

	  sc->cdp = NULL;
	
#ifndef TEL_NO_SOUND_BRIDGE
	  /* close sound-bridge */
	  snd_close(&sc->rd.audio_dev);
	  snd_close(&sc->wr.audio_dev);
#endif
	
	  if(sc->state & ST_RDWAIT_DATA)
	  {
		sc->state &= ~ST_RDWAIT_DATA;
		wakeup(&f->rx_queue);
	  }

	  if(sc->state & ST_WRWAIT_EMPTY)
	  {
		sc->state &= ~ST_WRWAIT_EMPTY;
		wakeup(&f->tx_queue);
	  }

	  if(sc->state & ST_TONE)
	  {
		sc->state &= ~ST_TONE;
		sc->tone_duration = 0;
		wakeup(&sc->tones);
	  }

	  if(sc->rd_mbuf)
	  {
		m_freem(sc->rd_mbuf);
		sc->rd_mbuf = NULL;
	  }
	}
	return f;
}

static cdevsw_t i4b_tel_cdevsw = {
#ifdef D_VERSION
      .d_version =    D_VERSION,
#endif
      .d_open =       i4b_tel_open,
      .d_close =      i4b_tel_close,
      .d_read =       i4b_tel_read,
      .d_write =      i4b_tel_write,
      .d_ioctl =      i4b_tel_ioctl,
      .d_poll =       i4b_tel_poll,
      .d_name =       "i4btel",
};

static cdevsw_t i4b_tel_d_cdevsw = {
#ifdef D_VERSION
      .d_version =    D_VERSION,
#endif
      .d_open =       i4b_tel_dopen,
      .d_close =      i4b_tel_dclose,
      .d_read =       i4b_tel_dread,
      .d_write =      i4b_tel_dwrite,
      .d_poll =       i4b_tel_dpoll,
      .d_name =       "i4bteld",
};

/*---------------------------------------------------------------------------*
 *	interface attach routine
 *---------------------------------------------------------------------------*/
static void
i4b_tel_attach(void *dummy)
{
	tel_sc_t *sc = &tel_sc[0];
	struct cdev *dev;
	int i;

	printf("i4btel: %d ISDN telephony interface device(s) attached\n", NI4BTEL);
	
	for(i=0; i < NI4BTEL; i++)
	{
		/* normal i4btel device */
		dev = make_dev(&i4b_tel_cdevsw, i,
			       UID_ROOT, GID_WHEEL,
			       0600, "i4btel%d", i);
		if(dev)
		{
		  dev->si_drv1 = sc;
		}

		/* i4bteld dialout device */
		dev = make_dev(&i4b_tel_d_cdevsw, i,
			       UID_ROOT, GID_WHEEL,
			       0600, "i4bteld%d", i);
		if(dev)
		{
		  dev->si_drv1 = sc;
		}

		sc++;
	}

	return;
}
SYSINIT(i4b_tel_attach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4b_tel_attach, NULL);

/*---------------------------------------------------------------------------*
 *	dial and ringing tone generator
 *---------------------------------------------------------------------------*/

static void
tel_dial_put_mbuf(struct fifo_translator *f, struct mbuf *m)
{
	/* just free the mbuf */

	m_freem(m);

	return;
}

static struct mbuf *
tel_dial_get_mbuf(struct fifo_translator *f)
{
	struct call_desc *cd = f->L5_sc;
	struct mbuf *m;
	u_int16_t pos;
	u_int16_t len;
	u_int16_t freq;
	u_int8_t *ptr;

	m = i4b_getmbuf(880, M_NOWAIT);

	if (m) {

	  if ((cd->tone_gen_ptr == NULL) || 
	      (cd->tone_gen_ptr[0] == 0))
	  {
	      if ((cd->tone_gen_state == 0x19) && (cd->dst_telno[0] == 0))
	      {
		  /* continuous 440Hz sine wave */
		  cd->tone_gen_ptr = "a";
	      }
	      else if (cd->tone_gen_state != 0x07)
	      {
	          /* all silent */
	          cd->tone_gen_ptr = " ";
	      }
	      else
	      {
		  /* ringing */
		  cd->tone_gen_ptr = 
		    "aaaaaaaaaa"
		    "          "
		    "          "
		    "          ";
	      }
	  }

	  if (cd->tone_gen_ptr[0] == 'a')
	      freq = 440; /* Hz */
	  else
	      freq = 0; /* Hz */

	  if (freq == 0) {
	    cd->tone_gen_pos = 0;
	  }

	  /* get last position */
	  pos = cd->tone_gen_pos;

	  len = m->m_len;
	  ptr = m->m_data;

	  while (len) {

	    if (cd->channel_bsubprot == BSUBPROT_G711_ULAW) {
	        *ptr = i4b_signed_to_ulaw(i4b_sine_to_signed[pos] / 4);
	    } else {
	        *ptr = i4b_signed_to_alaw(i4b_sine_to_signed[pos] / 4);
	    }

	    ptr++;
	    len--;
	    pos += freq;

	    if (pos >= 8000) {
	        pos -= 8000;
	    }
	  }

	  /* store position */
	  cd->tone_gen_pos = pos;

	  /* increment tone program */
	  cd->tone_gen_ptr++;

	} else {

	  /* out of memory */

	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
tel_dial_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
		  struct i4b_protocol *pp, u_int32_t driver_type,
		  u_int32_t driver_unit, call_desc_t *cd)
{
	if(!cd)
	{
		return FT_INVALID;
	}

	if(!pp)
	{
		return cd->fifo_translator_tone_gen;
	}

	cd->fifo_translator_tone_gen = f; 

	if(pp->protocol_1)
	{
	    /* connected */

	    f->L5_sc = cd;

	    f->L5_PUT_MBUF = tel_dial_put_mbuf;
	    f->L5_GET_MBUF = tel_dial_get_mbuf;

	    /* reset tone generator */

	    cd->tone_gen_ptr = NULL;
	}
	return f;
}
