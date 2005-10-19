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

#define TEL_USE_SIGNED_8_BIT
#define TEL_USE_SOUND_BRIDGE

struct tel_parameters {
	u_int8_t	audio_amp;	/* audio amplification */
	u_int8_t	audio_flag;	/* conversion flag 
					 * bit usage:
					 *  i4b_tel_convert:
					 *   0x01,0x02,0x04,0x08,0x10
					 *  filter_BUF:
					 *   0x20
					 *  other: 
					 *  0x40, 0x80
					 */
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
	u_int8_t		src_sms[SMS_MAX];
} tel_sc_t;

static tel_sc_t tel_sc[NI4BTEL];

/* audio format conversion tables */
static u_int8_t a2u_tab[];
static u_int8_t u2a_tab[];
static int16_t  u2l_tab[];
static u_int8_t bitreverse[];
static u_int8_t sinetab[];

static void
i4b_tel_u2l(u_int8_t *ptr, u_int16_t len)
{
    while(len--)
    {
        *((int8_t *)ptr) = u2l_tab[*ptr] / 256;
	ptr++;
    }
    return;
}

static void
i4b_tel_l2u(u_int8_t *ptr, u_int16_t len)
{
    int32_t temp;
    u_int8_t mask;
    u_int8_t value;

    while(len--)
    {
      temp = ((int8_t)ptr[0]) * 256;

      if(temp <= -32124)
      {
	  temp = -32124;
	  value = 0x00;
	  goto skip;
      }

      if(temp >= 32124)
      {
	  temp = 32124;
	  value = 0x80;
	  goto skip;
      }

      /*
       * search u-law value for temp, to save memory 
       */
      if(temp < 0)
      {
	  value = 0x00;
	  mask = 0x40;

	  do {
	    value |= mask;

	    if(temp < u2l_tab[value])
	    {
	        value ^= mask;
	    }
	  } while(mask >>= 1);
      }
      else
      {
	  value = 0x80;
	  mask = 0x40;

	  do {
	    value |= mask;

	    if(temp > u2l_tab[value])
	    {
	        value ^= mask;
	    }
	  } while(mask >>= 1);
      }

  skip:

      /* write back new value */
      *ptr = value;
      *ptr++;
    }
    return;
}

/*---------------------------------------------------------------------------*
 *	amplify u-law data
 *---------------------------------------------------------------------------*/
static void
i4b_tel_amplify(u_int8_t *ptr, u_int len, u_int8_t loudness)
{
  int32_t temp;
  u_int8_t mask;
  u_int8_t value;

  while(len--)
  {
    temp = u2l_tab[*ptr];

    temp *= loudness;
    temp /= AUDIOAMP_DP;

    if(temp <= -32124)
    {
      temp = -32124;
      value = 0x00;
      goto skip;
    }

    if(temp >= 32124)
    {
      temp = 32124;
      value = 0x80;
      goto skip;
    }

    /*
     * search u-law value for temp, to save memory 
     */
    if(temp < 0)
    {
      value = 0x00;
      mask = 0x40;

      do {
	value |= mask;

	if(temp < u2l_tab[value])
	{
	  value ^= mask;
	}
      } while(mask >>= 1);
    }
    else
    {
      value = 0x80;
      mask = 0x40;

      do {
	value |= mask;

	if(temp > u2l_tab[value])
	{
	  value ^= mask;
	}
      } while(mask >>= 1);
    }

  skip:

    /* write back new value */
    *ptr = value;
    *ptr++;
  }
  return;
}

static void
i4b_tel_convert(u_int8_t *ptr, u_int len, u_int8_t audio_flag)
{
	if(audio_flag & 0x10)
	{
	  /* mute */
	  while(len--)
	  {
	    *ptr++ = 0xFF;
	  }
	}
	else
	{
	  while(len--)
	  {
	    if(audio_flag & 0x01)
	    {
	      /* always reverse bit order from line first */
	      *ptr = bitreverse[*ptr];
	    }
	    if(audio_flag & 0x02)
	    {
	      /* convert if necessary */
	      *ptr = a2u_tab[*ptr];
	    }
	    if(audio_flag & 0x04)
	    {
	      /* convert if necessary */
	      *ptr = u2a_tab[*ptr];
	    }
	    if(audio_flag & 0x08)
	    {
	      /* always reverse bitorder to line last */
	      *ptr = bitreverse[*ptr];
	    }
	    ptr++;
	  }
	}
	return;
}

static struct mbuf *
i4b_tel_tone(tel_sc_t *sc)
{
	struct mbuf *m;
	u_int32_t len;
	int32_t temp;
	u_int8_t *ptr;
	u_int8_t value,mask;

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

	  temp = u2l_tab[sinetab[sc->tone_pos_1]] +
	    u2l_tab[sinetab[sc->tone_pos_2]];

	  if(temp <= -32124)
	  {
		temp = -32124;
		value = 0x00;
		goto skip;
	  }

	  if(temp >= 32124)
	  {
		temp = 32124;
		value = 0x80;
		goto skip;
	  }

	  /*
	   * search u-law value for temp, to save memory 
	   */
	  if(temp < 0)
	  {
		value = 0x00;
		mask = 0x40;

		do {
		  value |= mask;

		  if(temp < u2l_tab[value])
		  {
			value ^= mask;
		  }
		} while(mask >>= 1);
	  }
	  else
	  {
		value = 0x80;
		mask = 0x40;

		do {
		  value |= mask;

		  if(temp > u2l_tab[value])
		  {
			value ^= mask;
		  }
		} while(mask >>= 1);
	  }

	skip:

	  *ptr++ = value;

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

	/* convert data (u-law to a-law and bitreverse) */
	i4b_tel_convert(m->m_data, m->m_len, 0x08|0x04);
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
		sc->rd.audio_flag = 0x08;
		sc->rd.audio_amp = AUDIOAMP_DP;
		sc->wr.audio_flag = 0x01;
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
		switch (*(int *)data)
		{
			case CVT_NONE:
			  sc->wr.audio_flag = 0x01;
			  sc->rd.audio_flag = 0x08;
			  break;
			case CVT_ALAW2ULAW:
			  /* ISDN: a-law */
			  /* user: u-law */ 
			  sc->wr.audio_flag = 0x01|0x02;
			  sc->rd.audio_flag = 0x08|0x04;
			  break;
			case CVT_ULAW2ALAW:
			  /* ISDN: u-law */
			  /* user: a-law */ 
			  sc->wr.audio_flag = 0x01|0x04;
			  sc->rd.audio_flag = 0x08|0x02;
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
	  {
		msg_vr_req_t *mvr = (void *)data;

		mvr->version = I4B_VERSION;
		mvr->release = I4B_REL;
		mvr->step = I4B_STEP;
		break;
	  }

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
		strlcpy(data, &sc->src_sms[0], SMS_MAX);
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
			i4b_tel_convert(m->m_data, m->m_len, sc->wr.audio_flag);

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
	    i4b_tel_convert(m->m_data, m->m_len, sc->rd.audio_flag);

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

		strlcpy(&sc->src_sms[0], &mrtu->src_sms[0],
			sizeof(sc->src_sms));
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
	sc->cdp->last_active_time = SECOND;
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
#include <i4b/layer1/ihfc2/i4b_ihfc2.h> /* struct buffer */

#define TEL_AUDIORATE	8000 /*hz*/
#define TEL_MIN_BUFFER    32 /*bytes*/
#define TEL_MAX_BUFFER   800 /*bytes*/
#define TEL_AUDIODEVICE	"/dev/dsp"

#ifndef devsw
#define devsw(dev) ((dev)->si_devsw)
#endif

static void
snd_close(struct cdev **pdev)
{
	if(*pdev)
	{
	  devsw(*pdev)->d_close(*pdev,0/*flags*/,0/*mode*/,NULL);
	  *pdev = 0;
	}

	return;
}

static struct cdev *
getdevbyname(const char *name)
{
#ifdef NDEVFSINO
  struct cdev *dev, **pdev;
  int inode;

  if((name[0] == '/') &&
     (name[1] == 'd') &&
     (name[2] == 'e') &&
     (name[3] == 'v') &&
     (name[4] == '/'))
  {
    name += 5;
  }

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
		goto found;
	      }
	    }
	  }
	}
  }

  dev = NULL;

 found:

  return dev;
#else
  /* XXX devfs was redesigned ... */
  return NULL;
#endif
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
	  dev = 0;
	  msg = "device not present";
	  goto error;
	}

	if(devsw(dev)->d_open(dev,flags,0/*mode*/,curthread) != 0)
	{
	  dev = 0;
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

	if((sc->rd.audio_dev == 0) ||
	   (sc->wr.audio_dev == 0))
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
	    if(parm->audio_flag & 0x40)
	    {
	        i4b_tel_l2u(ptr,len);
	    }

	    /* convert data */
	    i4b_tel_convert(ptr,len,parm->audio_flag);

	    /* amplify data */
	    if(parm->audio_amp != AUDIOAMP_DP)
	    {
	        i4b_tel_amplify(ptr,len,parm->audio_amp);
	    }

	    if(parm->audio_flag & 0x80)
	    {
	        i4b_tel_u2l(ptr,len);
	    }

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

#ifdef TEL_USE_SIGNED_8_BIT
		sc->wr.audio_flag |= 0x80;
#endif
		filter_BUF(buf_in,&out_buf,&sc->wr);

		sc->wr.audio_flag &= ~0x80;

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

#ifdef TEL_USE_SIGNED_8_BIT
		sc->rd.audio_flag |= 0x40;
#endif
		filter_BUF(&buf_in,out_buf,&sc->rd);

		sc->rd.audio_flag &= ~0x40;

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

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
tel_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, u_int *protocol,
	     u_int driver_type, u_int driver_unit, call_desc_t *cd)
{
	tel_sc_t *sc = &tel_sc[driver_unit];

	if(!protocol)
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

	if(*protocol)
	{
	  /* connected */

	  f->L5_sc = sc;

	  f->L5_PUT_MBUF     = tel_put_mbuf;
	  f->L5_GET_MBUF     = tel_get_mbuf;
	  f->L5_TX_INTERRUPT = tel_tx_interrupt;
	  f->L5_RX_INTERRUPT = tel_rx_interrupt;

	  sc->cdp = cd;

	  if(sc->use_sound_bridge)  /* driver_type == DRVR_TEL_SB */
	  {
		/* open sound-brigde */
		if(snd_open(sc))
		{
		  /* force ringbuffers! */
		  *protocol = P_TRANSPARENT_RING;
		}
		else
		{
		  /* error */
		  /* see unsetup under
		   * not connected
		   */
		  *protocol = P_DISABLE;
		}
	  }
	}
	else
	{
	  /* not connected */
	
	  /* close sound-bridge */
	  snd_close(&sc->rd.audio_dev);
	  snd_close(&sc->wr.audio_dev);
	
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

/*===========================================================================*
 *	AUDIO FORMAT CONVERSION (produced by running g711conv)
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	A-law to u-law conversion
 *---------------------------------------------------------------------------*/
static u_int8_t a2u_tab[256] = {
/* 00 */	0x2a, 0x2b, 0x28, 0x29, 0x2e, 0x2f, 0x2c, 0x2d, 
/* 08 */	0x22, 0x23, 0x20, 0x21, 0x26, 0x27, 0x24, 0x25, 
/* 10 */	0x39, 0x3a, 0x37, 0x38, 0x3d, 0x3e, 0x3b, 0x3c, 
/* 18 */	0x31, 0x32, 0x30, 0x30, 0x35, 0x36, 0x33, 0x34, 
/* 20 */	0x0a, 0x0b, 0x08, 0x09, 0x0e, 0x0f, 0x0c, 0x0d, 
/* 28 */	0x02, 0x03, 0x00, 0x01, 0x06, 0x07, 0x04, 0x05, 
/* 30 */	0x1a, 0x1b, 0x18, 0x19, 0x1e, 0x1f, 0x1c, 0x1d, 
/* 38 */	0x12, 0x13, 0x10, 0x11, 0x16, 0x17, 0x14, 0x15, 
/* 40 */	0x62, 0x63, 0x60, 0x61, 0x66, 0x67, 0x64, 0x65, 
/* 48 */	0x5d, 0x5d, 0x5c, 0x5c, 0x5f, 0x5f, 0x5e, 0x5e, 
/* 50 */	0x74, 0x76, 0x70, 0x72, 0x7c, 0x7e, 0x78, 0x7a, 
/* 58 */	0x6a, 0x6b, 0x68, 0x69, 0x6e, 0x6f, 0x6c, 0x6d, 
/* 60 */	0x48, 0x49, 0x46, 0x47, 0x4c, 0x4d, 0x4a, 0x4b, 
/* 68 */	0x40, 0x41, 0x3f, 0x3f, 0x44, 0x45, 0x42, 0x43, 
/* 70 */	0x56, 0x57, 0x54, 0x55, 0x5a, 0x5b, 0x58, 0x59, 
/* 78 */	0x4f, 0x4f, 0x4e, 0x4e, 0x52, 0x53, 0x50, 0x51, 
/* 80 */	0xaa, 0xab, 0xa8, 0xa9, 0xae, 0xaf, 0xac, 0xad, 
/* 88 */	0xa2, 0xa3, 0xa0, 0xa1, 0xa6, 0xa7, 0xa4, 0xa5, 
/* 90 */	0xb9, 0xba, 0xb7, 0xb8, 0xbd, 0xbe, 0xbb, 0xbc, 
/* 98 */	0xb1, 0xb2, 0xb0, 0xb0, 0xb5, 0xb6, 0xb3, 0xb4, 
/* a0 */	0x8a, 0x8b, 0x88, 0x89, 0x8e, 0x8f, 0x8c, 0x8d, 
/* a8 */	0x82, 0x83, 0x80, 0x81, 0x86, 0x87, 0x84, 0x85, 
/* b0 */	0x9a, 0x9b, 0x98, 0x99, 0x9e, 0x9f, 0x9c, 0x9d, 
/* b8 */	0x92, 0x93, 0x90, 0x91, 0x96, 0x97, 0x94, 0x95, 
/* c0 */	0xe2, 0xe3, 0xe0, 0xe1, 0xe6, 0xe7, 0xe4, 0xe5, 
/* c8 */	0xdd, 0xdd, 0xdc, 0xdc, 0xdf, 0xdf, 0xde, 0xde, 
/* d0 */	0xf4, 0xf6, 0xf0, 0xf2, 0xfc, 0xfe, 0xf8, 0xfa, 
/* d8 */	0xea, 0xeb, 0xe8, 0xe9, 0xee, 0xef, 0xec, 0xed, 
/* e0 */	0xc8, 0xc9, 0xc6, 0xc7, 0xcc, 0xcd, 0xca, 0xcb, 
/* e8 */	0xc0, 0xc1, 0xbf, 0xbf, 0xc4, 0xc5, 0xc2, 0xc3, 
/* f0 */	0xd6, 0xd7, 0xd4, 0xd5, 0xda, 0xdb, 0xd8, 0xd9, 
/* f8 */	0xcf, 0xcf, 0xce, 0xce, 0xd2, 0xd3, 0xd0, 0xd1
};

/*---------------------------------------------------------------------------*
 *	u-law to A-law conversion
 *---------------------------------------------------------------------------*/
static u_int8_t u2a_tab[256] = {
/* 00 */	0x2a, 0x2b, 0x28, 0x29, 0x2e, 0x2f, 0x2c, 0x2d, 
/* 08 */	0x22, 0x23, 0x20, 0x21, 0x26, 0x27, 0x24, 0x25, 
/* 10 */	0x3a, 0x3b, 0x38, 0x39, 0x3e, 0x3f, 0x3c, 0x3d, 
/* 18 */	0x32, 0x33, 0x30, 0x31, 0x36, 0x37, 0x34, 0x35, 
/* 20 */	0x0a, 0x0b, 0x08, 0x09, 0x0e, 0x0f, 0x0c, 0x0d, 
/* 28 */	0x02, 0x03, 0x00, 0x01, 0x06, 0x07, 0x04, 0x05, 
/* 30 */	0x1b, 0x18, 0x19, 0x1e, 0x1f, 0x1c, 0x1d, 0x12, 
/* 38 */	0x13, 0x10, 0x11, 0x16, 0x17, 0x14, 0x15, 0x6a, 
/* 40 */	0x68, 0x69, 0x6e, 0x6f, 0x6c, 0x6d, 0x62, 0x63, 
/* 48 */	0x60, 0x61, 0x66, 0x67, 0x64, 0x65, 0x7a, 0x78, 
/* 50 */	0x7e, 0x7f, 0x7c, 0x7d, 0x72, 0x73, 0x70, 0x71, 
/* 58 */	0x76, 0x77, 0x74, 0x75, 0x4b, 0x49, 0x4f, 0x4d, 
/* 60 */	0x42, 0x43, 0x40, 0x41, 0x46, 0x47, 0x44, 0x45, 
/* 68 */	0x5a, 0x5b, 0x58, 0x59, 0x5e, 0x5f, 0x5c, 0x5d, 
/* 70 */	0x52, 0x52, 0x53, 0x53, 0x50, 0x50, 0x51, 0x51, 
/* 78 */	0x56, 0x56, 0x57, 0x57, 0x54, 0x54, 0x55, 0x55, 
/* 80 */	0xaa, 0xab, 0xa8, 0xa9, 0xae, 0xaf, 0xac, 0xad, 
/* 88 */	0xa2, 0xa3, 0xa0, 0xa1, 0xa6, 0xa7, 0xa4, 0xa5, 
/* 90 */	0xba, 0xbb, 0xb8, 0xb9, 0xbe, 0xbf, 0xbc, 0xbd, 
/* 98 */	0xb2, 0xb3, 0xb0, 0xb1, 0xb6, 0xb7, 0xb4, 0xb5, 
/* a0 */	0x8a, 0x8b, 0x88, 0x89, 0x8e, 0x8f, 0x8c, 0x8d, 
/* a8 */	0x82, 0x83, 0x80, 0x81, 0x86, 0x87, 0x84, 0x85, 
/* b0 */	0x9b, 0x98, 0x99, 0x9e, 0x9f, 0x9c, 0x9d, 0x92, 
/* b8 */	0x93, 0x90, 0x91, 0x96, 0x97, 0x94, 0x95, 0xea, 
/* c0 */	0xe8, 0xe9, 0xee, 0xef, 0xec, 0xed, 0xe2, 0xe3, 
/* c8 */	0xe0, 0xe1, 0xe6, 0xe7, 0xe4, 0xe5, 0xfa, 0xf8, 
/* d0 */	0xfe, 0xff, 0xfc, 0xfd, 0xf2, 0xf3, 0xf0, 0xf1, 
/* d8 */	0xf6, 0xf7, 0xf4, 0xf5, 0xcb, 0xc9, 0xcf, 0xcd, 
/* e0 */	0xc2, 0xc3, 0xc0, 0xc1, 0xc6, 0xc7, 0xc4, 0xc5, 
/* e8 */	0xda, 0xdb, 0xd8, 0xd9, 0xde, 0xdf, 0xdc, 0xdd, 
/* f0 */	0xd2, 0xd2, 0xd3, 0xd3, 0xd0, 0xd0, 0xd1, 0xd1, 
/* f8 */	0xd6, 0xd6, 0xd7, 0xd7, 0xd4, 0xd4, 0xd5, 0xd5
};
  
/*---------------------------------------------------------------------------*
 *	reverse bits in a byte
 *---------------------------------------------------------------------------*/
static u_int8_t bitreverse[256] = {
/* 00 */	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 
/* 08 */	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0, 
/* 10 */	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8, 
/* 18 */	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8, 
/* 20 */	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4, 
/* 28 */	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4, 
/* 30 */	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec, 
/* 38 */	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc, 
/* 40 */	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2, 
/* 48 */	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2, 
/* 50 */	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea, 
/* 58 */	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa, 
/* 60 */	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6, 
/* 68 */	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6, 
/* 70 */	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee, 
/* 78 */	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe, 
/* 80 */	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1, 
/* 88 */	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1, 
/* 90 */	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9, 
/* 98 */	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9, 
/* a0 */	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5, 
/* a8 */	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5, 
/* b0 */	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed, 
/* b8 */	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd, 
/* c0 */	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, 
/* c8 */	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3, 
/* d0 */	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb, 
/* d8 */	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb, 
/* e0 */	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7, 
/* e8 */	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7, 
/* f0 */	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef, 
/* f8 */	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};

/*---------------------------------------------------------------------------*
 *	u-law encoded sine wave (1Hz, amplitude: max/3)
 *---------------------------------------------------------------------------*/
static u_int8_t sinetab[8000] = {
255, 254, 252, 251, 250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 
240, 239, 238, 238, 237, 237, 236, 236, 235, 234, 234, 233, 233, 232, 
232, 231, 231, 230, 230, 229, 229, 228, 228, 227, 226, 226, 225, 225, 
224, 224, 223, 223, 223, 222, 222, 222, 222, 221, 221, 221, 220, 220, 
220, 220, 219, 219, 219, 219, 218, 218, 218, 217, 217, 217, 217, 216, 
216, 216, 216, 215, 215, 215, 215, 214, 214, 214, 213, 213, 213, 213, 
212, 212, 212, 212, 211, 211, 211, 211, 210, 210, 210, 209, 209, 209, 
209, 208, 208, 208, 208, 207, 207, 207, 207, 207, 206, 206, 206, 206, 
206, 206, 206, 206, 205, 205, 205, 205, 205, 205, 205, 204, 204, 204, 
204, 204, 204, 204, 204, 203, 203, 203, 203, 203, 203, 203, 202, 202, 
202, 202, 202, 202, 202, 202, 201, 201, 201, 201, 201, 201, 201, 200, 
200, 200, 200, 200, 200, 200, 200, 199, 199, 199, 199, 199, 199, 199, 
198, 198, 198, 198, 198, 198, 198, 198, 197, 197, 197, 197, 197, 197, 
197, 197, 196, 196, 196, 196, 196, 196, 196, 195, 195, 195, 195, 195, 
195, 195, 195, 194, 194, 194, 194, 194, 194, 194, 193, 193, 193, 193, 
193, 193, 193, 193, 192, 192, 192, 192, 192, 192, 192, 191, 191, 191, 
191, 191, 191, 191, 191, 191, 191, 191, 191, 190, 190, 190, 190, 190, 
190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 189, 189, 189, 189, 
189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 188, 188, 188, 
188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 187, 187, 
187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 
186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 
186, 185, 185, 185, 185, 185, 185, 185, 185, 185, 185, 185, 185, 185, 
185, 185, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184, 
184, 184, 184, 184, 183, 183, 183, 183, 183, 183, 183, 183, 183, 183, 
183, 183, 183, 183, 183, 182, 182, 182, 182, 182, 182, 182, 182, 182, 
182, 182, 182, 182, 182, 182, 182, 181, 181, 181, 181, 181, 181, 181, 
181, 181, 181, 181, 181, 181, 181, 181, 181, 180, 180, 180, 180, 180, 
180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 179, 179, 179, 179, 
179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 178, 178, 
178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 
177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 
177, 177, 176, 176, 176, 176, 176, 176, 176, 176, 176, 176, 176, 176, 
176, 176, 176, 176, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 
175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 
174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 
174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 
174, 174, 174, 174, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 
173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 
173, 173, 173, 173, 173, 173, 173, 173, 173, 172, 172, 172, 172, 172, 
172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 
172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 
171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 
171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 
171, 171, 171, 171, 171, 170, 170, 170, 170, 170, 170, 170, 170, 170, 
170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 
170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 169, 169, 169, 
169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 
169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 
169, 169, 169, 169, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 
168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 
168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 167, 167, 167, 
167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 
167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 
167, 167, 167, 167, 167, 166, 166, 166, 166, 166, 166, 166, 166, 166, 
166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 
166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 165, 
165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 
165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 
165, 165, 165, 165, 165, 165, 165, 165, 164, 164, 164, 164, 164, 164, 
164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 
164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 
164, 164, 164, 164, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 
163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 
163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 
163, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 
162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 
162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 
161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 
161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 
161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 160, 
160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 
160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 
160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 
153, 153, 153, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 154, 
154, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 
155, 155, 155, 155, 155, 155, 155, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 156, 
156, 156, 156, 156, 156, 156, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 157, 
157, 157, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 158, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 
159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 159, 160, 160, 160, 
160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 
160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 
160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 161, 161, 
161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 
161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 
161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 162, 162, 162, 
162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 
162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 
162, 162, 162, 162, 162, 162, 162, 162, 162, 162, 163, 163, 163, 163, 
163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 
163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 
163, 163, 163, 163, 163, 163, 163, 164, 164, 164, 164, 164, 164, 164, 
164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 
164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 164, 
164, 164, 164, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 
165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 
165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 165, 166, 166, 
166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 
166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 
166, 166, 166, 166, 166, 166, 167, 167, 167, 167, 167, 167, 167, 167, 
167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 
167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 167, 
168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 
168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 168, 
168, 168, 168, 168, 168, 168, 168, 169, 169, 169, 169, 169, 169, 169, 
169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 
169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 169, 
170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 
170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 
170, 170, 170, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 171, 
171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 
171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 171, 172, 172, 172, 
172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 
172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 172, 
172, 172, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 
173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 173, 
173, 173, 173, 173, 173, 173, 173, 174, 174, 174, 174, 174, 174, 174, 
174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 
174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 174, 175, 175, 175, 
175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 175, 
175, 175, 175, 175, 175, 175, 175, 176, 176, 176, 176, 176, 176, 176, 
176, 176, 176, 176, 176, 176, 176, 176, 176, 177, 177, 177, 177, 177, 
177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 178, 178, 178, 
178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 179, 
179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179, 
179, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 
180, 180, 181, 181, 181, 181, 181, 181, 181, 181, 181, 181, 181, 181, 
181, 181, 181, 181, 182, 182, 182, 182, 182, 182, 182, 182, 182, 182, 
182, 182, 182, 182, 182, 182, 183, 183, 183, 183, 183, 183, 183, 183, 
183, 183, 183, 183, 183, 183, 183, 184, 184, 184, 184, 184, 184, 184, 
184, 184, 184, 184, 184, 184, 184, 184, 184, 185, 185, 185, 185, 185, 
185, 185, 185, 185, 185, 185, 185, 185, 185, 185, 186, 186, 186, 186, 
186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 186, 187, 187, 187, 
187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 187, 188, 
188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 188, 
189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 189, 
189, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 190, 
190, 190, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 191, 
192, 192, 192, 192, 192, 192, 192, 193, 193, 193, 193, 193, 193, 193, 
193, 194, 194, 194, 194, 194, 194, 194, 195, 195, 195, 195, 195, 195, 
195, 195, 196, 196, 196, 196, 196, 196, 196, 197, 197, 197, 197, 197, 
197, 197, 197, 198, 198, 198, 198, 198, 198, 198, 198, 199, 199, 199, 
199, 199, 199, 199, 200, 200, 200, 200, 200, 200, 200, 200, 201, 201, 
201, 201, 201, 201, 201, 202, 202, 202, 202, 202, 202, 202, 202, 203, 
203, 203, 203, 203, 203, 203, 204, 204, 204, 204, 204, 204, 204, 204, 
205, 205, 205, 205, 205, 205, 205, 206, 206, 206, 206, 206, 206, 206, 
206, 207, 207, 207, 207, 207, 208, 208, 208, 208, 209, 209, 209, 209, 
210, 210, 210, 211, 211, 211, 211, 212, 212, 212, 212, 213, 213, 213, 
213, 214, 214, 214, 215, 215, 215, 215, 216, 216, 216, 216, 217, 217, 
217, 217, 218, 218, 218, 219, 219, 219, 219, 220, 220, 220, 220, 221, 
221, 221, 222, 222, 222, 222, 223, 223, 223, 224, 224, 225, 225, 226, 
226, 227, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, 233, 
234, 234, 235, 236, 236, 237, 237, 238, 238, 239, 240, 241, 242, 243, 
244, 245, 246, 247, 248, 249, 250, 251, 252, 254, 255, 126, 124, 123, 
122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 110, 
109, 109, 108, 108, 107, 106, 106, 105, 105, 104, 104, 103, 103, 102, 
102, 101, 101, 100, 100,  99,  98,  98,  97,  97,  96,  96,  95,  95, 
 95,  94,  94,  94,  94,  93,  93,  93,  92,  92,  92,  92,  91,  91, 
 91,  91,  90,  90,  90,  89,  89,  89,  89,  88,  88,  88,  88,  87, 
 87,  87,  87,  86,  86,  86,  85,  85,  85,  85,  84,  84,  84,  84, 
 83,  83,  83,  83,  82,  82,  82,  81,  81,  81,  81,  80,  80,  80, 
 80,  79,  79,  79,  79,  79,  78,  78,  78,  78,  78,  78,  78,  78, 
 77,  77,  77,  77,  77,  77,  77,  76,  76,  76,  76,  76,  76,  76, 
 76,  75,  75,  75,  75,  75,  75,  75,  74,  74,  74,  74,  74,  74, 
 74,  74,  73,  73,  73,  73,  73,  73,  73,  72,  72,  72,  72,  72, 
 72,  72,  72,  71,  71,  71,  71,  71,  71,  71,  70,  70,  70,  70, 
 70,  70,  70,  70,  69,  69,  69,  69,  69,  69,  69,  69,  68,  68, 
 68,  68,  68,  68,  68,  67,  67,  67,  67,  67,  67,  67,  67,  66, 
 66,  66,  66,  66,  66,  66,  65,  65,  65,  65,  65,  65,  65,  65, 
 64,  64,  64,  64,  64,  64,  64,  63,  63,  63,  63,  63,  63,  63, 
 63,  63,  63,  63,  63,  62,  62,  62,  62,  62,  62,  62,  62,  62, 
 62,  62,  62,  62,  62,  62,  61,  61,  61,  61,  61,  61,  61,  61, 
 61,  61,  61,  61,  61,  61,  61,  60,  60,  60,  60,  60,  60,  60, 
 60,  60,  60,  60,  60,  60,  60,  60,  59,  59,  59,  59,  59,  59, 
 59,  59,  59,  59,  59,  59,  59,  59,  59,  59,  58,  58,  58,  58, 
 58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  58,  57,  57,  57, 
 57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  56,  56, 
 56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56, 
 55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55, 
 55,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54, 
 54,  54,  54,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53, 
 53,  53,  53,  53,  53,  52,  52,  52,  52,  52,  52,  52,  52,  52, 
 52,  52,  52,  52,  52,  52,  51,  51,  51,  51,  51,  51,  51,  51, 
 51,  51,  51,  51,  51,  51,  51,  51,  50,  50,  50,  50,  50,  50, 
 50,  50,  50,  50,  50,  50,  50,  50,  50,  50,  49,  49,  49,  49, 
 49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  49,  48,  48, 
 48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48, 
 47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47, 
 47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  46,  46,  46,  46, 
 46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46, 
 46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46, 
 45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45, 
 45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45, 
 45,  45,  45,  45,  45,  44,  44,  44,  44,  44,  44,  44,  44,  44, 
 44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44, 
 44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  43,  43,  43,  43, 
 43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43, 
 43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43, 
 43,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42, 
 42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42, 
 42,  42,  42,  42,  42,  42,  42,  41,  41,  41,  41,  41,  41,  41, 
 41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41, 
 41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41, 
 40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40, 
 40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40, 
 40,  40,  40,  40,  40,  40,  40,  39,  39,  39,  39,  39,  39,  39, 
 39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39, 
 39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39, 
 39,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38, 
 38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38, 
 38,  38,  38,  38,  38,  38,  38,  38,  38,  37,  37,  37,  37,  37, 
 37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37, 
 37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37, 
 37,  37,  37,  37,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36, 
 36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36, 
 36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36, 
 35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35, 
 35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35, 
 35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  34,  34,  34, 
 34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34, 
 34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34, 
 34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  33,  33,  33,  33, 
 33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33, 
 33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33, 
 33,  33,  33,  33,  33,  33,  33,  33,  33,  32,  32,  32,  32,  32, 
 32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32, 
 32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32, 
 32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25, 
 25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  25,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26, 
 26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  26,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27,  27, 
 27,  27,  27,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28,  28, 
 28,  28,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29, 
 29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  29,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30, 
 30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31,  31, 
 31,  31,  31,  31,  31,  31,  31,  32,  32,  32,  32,  32,  32,  32, 
 32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32, 
 32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32,  32, 
 32,  32,  32,  32,  32,  32,  32,  32,  33,  33,  33,  33,  33,  33, 
 33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33, 
 33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33,  33, 
 33,  33,  33,  33,  33,  33,  33,  34,  34,  34,  34,  34,  34,  34, 
 34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34, 
 34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34,  34, 
 34,  34,  34,  34,  34,  34,  35,  35,  35,  35,  35,  35,  35,  35, 
 35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35, 
 35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35,  35, 
 35,  35,  35,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36, 
 36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36, 
 36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  36,  37, 
 37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37, 
 37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37,  37, 
 37,  37,  37,  37,  37,  37,  37,  37,  38,  38,  38,  38,  38,  38, 
 38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38, 
 38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38,  38, 
 38,  38,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39, 
 39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39, 
 39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  40,  40,  40,  40, 
 40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40, 
 40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40,  40, 
 40,  40,  40,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41, 
 41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  41, 
 41,  41,  41,  41,  41,  41,  41,  41,  41,  41,  42,  42,  42,  42, 
 42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42, 
 42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42,  42, 
 42,  42,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43, 
 43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43,  43, 
 43,  43,  43,  43,  43,  43,  43,  44,  44,  44,  44,  44,  44,  44, 
 44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44, 
 44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  45,  45, 
 45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45, 
 45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45,  45, 
 45,  45,  45,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46, 
 46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46,  46, 
 46,  46,  46,  46,  46,  46,  46,  47,  47,  47,  47,  47,  47,  47, 
 47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47,  47, 
 47,  47,  47,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48, 
 48,  48,  48,  48,  48,  49,  49,  49,  49,  49,  49,  49,  49,  49, 
 49,  49,  49,  49,  49,  49,  49,  50,  50,  50,  50,  50,  50,  50, 
 50,  50,  50,  50,  50,  50,  50,  50,  50,  51,  51,  51,  51,  51, 
 51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  51,  52,  52,  52, 
 52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  52,  53,  53, 
 53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53,  53, 
 54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54,  54, 
 54,  54,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55,  55, 
 55,  55,  55,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  56, 
 56,  56,  56,  56,  56,  57,  57,  57,  57,  57,  57,  57,  57,  57, 
 57,  57,  57,  57,  57,  57,  58,  58,  58,  58,  58,  58,  58,  58, 
 58,  58,  58,  58,  58,  58,  58,  59,  59,  59,  59,  59,  59,  59, 
 59,  59,  59,  59,  59,  59,  59,  59,  59,  60,  60,  60,  60,  60, 
 60,  60,  60,  60,  60,  60,  60,  60,  60,  60,  61,  61,  61,  61, 
 61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  61,  62,  62,  62, 
 62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  62,  63,  63, 
 63,  63,  63,  63,  63,  63,  63,  63,  63,  63,  64,  64,  64,  64, 
 64,  64,  64,  65,  65,  65,  65,  65,  65,  65,  65,  66,  66,  66, 
 66,  66,  66,  66,  67,  67,  67,  67,  67,  67,  67,  67,  68,  68, 
 68,  68,  68,  68,  68,  69,  69,  69,  69,  69,  69,  69,  69,  70, 
 70,  70,  70,  70,  70,  70,  70,  71,  71,  71,  71,  71,  71,  71, 
 72,  72,  72,  72,  72,  72,  72,  72,  73,  73,  73,  73,  73,  73, 
 73,  74,  74,  74,  74,  74,  74,  74,  74,  75,  75,  75,  75,  75, 
 75,  75,  76,  76,  76,  76,  76,  76,  76,  76,  77,  77,  77,  77, 
 77,  77,  77,  78,  78,  78,  78,  78,  78,  78,  78,  79,  79,  79, 
 79,  79,  80,  80,  80,  80,  81,  81,  81,  81,  82,  82,  82,  83, 
 83,  83,  83,  84,  84,  84,  84,  85,  85,  85,  85,  86,  86,  86, 
 87,  87,  87,  87,  88,  88,  88,  88,  89,  89,  89,  89,  90,  90, 
 90,  91,  91,  91,  91,  92,  92,  92,  92,  93,  93,  93,  94,  94, 
 94,  94,  95,  95,  95,  96,  96,  97,  97,  98,  98,  99, 100, 100, 
101, 101, 102, 102, 103, 103, 104, 104, 105, 105, 106, 106, 107, 108, 
108, 109, 109, 110, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
120, 121, 122, 123, 124, 126 };

/*===========================================================================*
 *	AUDIO FORMAT CONVERSION (from the sox package)
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 *	u-law to linear conversion
 *---------------------------------------------------------------------------*/
static int16_t u2l_tab[256] = {
    -32124,  -31100,  -30076,  -29052,  -28028,  -27004,  -25980,
    -24956,  -23932,  -22908,  -21884,  -20860,  -19836,  -18812,
    -17788,  -16764,  -15996,  -15484,  -14972,  -14460,  -13948,
    -13436,  -12924,  -12412,  -11900,  -11388,  -10876,  -10364,
     -9852,   -9340,   -8828,   -8316,   -7932,   -7676,   -7420,
     -7164,   -6908,   -6652,   -6396,   -6140,   -5884,   -5628,
     -5372,   -5116,   -4860,   -4604,   -4348,   -4092,   -3900,
     -3772,   -3644,   -3516,   -3388,   -3260,   -3132,   -3004,
     -2876,   -2748,   -2620,   -2492,   -2364,   -2236,   -2108,
     -1980,   -1884,   -1820,   -1756,   -1692,   -1628,   -1564,
     -1500,   -1436,   -1372,   -1308,   -1244,   -1180,   -1116,
     -1052,    -988,    -924,    -876,    -844,    -812,    -780,
      -748,    -716,    -684,    -652,    -620,    -588,    -556,
      -524,    -492,    -460,    -428,    -396,    -372,    -356,
      -340,    -324,    -308,    -292,    -276,    -260,    -244,
      -228,    -212,    -196,    -180,    -164,    -148,    -132,
      -120,    -112,    -104,     -96,     -88,     -80,     -72,
       -64,     -56,     -48,     -40,     -32,     -24,     -16,
        -8,       0,   32124,   31100,   30076,   29052,   28028,
     27004,   25980,   24956,   23932,   22908,   21884,   20860,
     19836,   18812,   17788,   16764,   15996,   15484,   14972,
     14460,   13948,   13436,   12924,   12412,   11900,   11388,
     10876,   10364,    9852,    9340,    8828,    8316,    7932,
      7676,    7420,    7164,    6908,    6652,    6396,    6140,
      5884,    5628,    5372,    5116,    4860,    4604,    4348,
      4092,    3900,    3772,    3644,    3516,    3388,    3260,
      3132,    3004,    2876,    2748,    2620,    2492,    2364,
      2236,    2108,    1980,    1884,    1820,    1756,    1692,
      1628,    1564,    1500,    1436,    1372,    1308,    1244,
      1180,    1116,    1052,     988,     924,     876,     844,
       812,     780,     748,     716,     684,     652,     620,
       588,     556,     524,     492,     460,     428,     396,
       372,     356,     340,     324,     308,     292,     276,
       260,     244,     228,     212,     196,     180,     164,
       148,     132,     120,     112,     104,      96,      88,
        80,      72,      64,      56,      48,      40,      32,
        24,      16,       8,       0
};
/*===========================================================================*/
