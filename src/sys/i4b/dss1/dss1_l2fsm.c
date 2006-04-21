/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2003, 2004 Hans Petter Selasky. All rights reserved.
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
 *	dss1_l2fsm.c - layer 2 FSM
 *	--------------------------
 *
 * $FreeBSD: $
 *
 *
 * existing-implementations:
 * -------------------------
 *
 * NOTE: some devices does not allow other C/R-bit selections
 *       than the ones used
 *
 * NOTE: some devices does not allow S- nor U-frames to be
 *	 length-extended in the future
 *
 * this implementation:
 * --------------------
 *
 * TODO: no support for duplicate TEI detection in NT-mode
 *	- wrong NR detection ?
 *	- double RR detection ?
 *
 * TODO. no support for call-redirection or moving of calls
 *
 * NOTE: no support for broadcast echo-back
 * NOTE: no support for outgoing-line reservation
 * NOTE: this protocol depends on hardware with collision-detection
 *
 * Upper layers send frames once, lower layers repeat frames, but
 * there is no guarantee that frames will never be lost
 *
 * Frame-number start value could have been 0xFF, because on collision
 * the bits will go towards zero. Frame-numbers does not have a
 * greater probability to be error free than other number-bits, so if
 * one frame-number is invalid, don't let that lead to a complete
 * protocol-reset. Frame-number moral: It is better to have two copies
 * of a frame than none!
 *
 * When a frame is received the answer is sent to the same "pipe"
 * that it was received through. Active services must increase the
 * "pipe's" reference-count, so that the "pipe" is not removed.
 *
 * In TE-mode only "broadcast-frames" can be received before "TEI
 * is assigned"
 *
 * Manual hardware numbering is preferred over automatic numbering. In
 * that regard it is possible to set the serial-number of the
 * protocol
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/syslog.h>
#include <sys/socket.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

#include <i4b/dss1/dss1_l2.h>
#include <i4b/dss1/dss1_l3.h>

#define L1_ACTIVATION_TIME (4*hz)

static void
dss1_pipe_set_state(DSS1_TCP_pipe_t *pipe, u_int8_t newstate);

/*---------------------------------------------------------------------------*
 *	a safe way to read a byte
 *---------------------------------------------------------------------------*/
u_int8_t
dss1_get_1(struct dss1_buffer *src, u_int16_t offset)
{
	offset += src->offset;
	return
	  ((offset < src->offset) || 
	   (offset >= src->len)) ? 0 : src->start[offset];
}

/*---------------------------------------------------------------------------*
 *	check if a offset is valid
 *
 * returns 1 if valid else 0
 *---------------------------------------------------------------------------*/
u_int8_t
dss1_get_valid(struct dss1_buffer *src, u_int16_t offset)
{
	offset += src->offset;
	return ((offset >= src->offset) &&
		(offset < src->len));
}

/*---------------------------------------------------------------------------*
 *	set new length for a buffer
 *
 * returns the old length
 *---------------------------------------------------------------------------*/
u_int16_t
dss1_set_length(struct dss1_buffer *src, u_int16_t new_len)
{
	u_int16_t old_len = src->len;

	if(new_len < old_len)
	{
	    src->len = new_len;
	}
	return old_len;
}

/*---------------------------------------------------------------------------*
 *	initialize a buffer
 *---------------------------------------------------------------------------*/
void
dss1_buf_init(struct dss1_buffer *dst, void *start, u_int16_t len)
{
	bzero(dst, sizeof(*dst));

	dst->start = start;
	dst->len = len;
	dst->offset = 0;
	return;
}

/*---------------------------------------------------------------------------*
 *	PIPE ACTIVATE REQUEST from Layer 3
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_activate_req(DSS1_TCP_pipe_t *pipe)
{
	if(pipe->state == ST_L2_PAUSE)
	{
	  /* need to set refcount before ST_L2_SEND_TEI */
	  pipe->refcount++;
	  dss1_pipe_set_state(pipe,ST_L2_SEND_TEI);
	  pipe->refcount--;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	PIPE DATA REQUEST from Layer 3
 *
 * frames might queue up if persistent L1-deactivation
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_data_req(DSS1_TCP_pipe_t *pipe, struct mbuf *m)
{
	l2softc_t *sc = pipe->L5_sc;

	if(_IF_QFULL(pipe) || !sc->sc_fifo_translator)
	{
	  NDBGL2(L2_ERROR, "unit=%x, pipe=%x, i_queue full or "
		 "no fifo translator!!", sc->sc_unit, PIPE_NO(pipe));
	  m_freem(m);
	}
	else
	{
	  _IF_ENQUEUE(pipe,m);

	  /* call transmit FIFO */
	  L1_FIFO_START(sc->sc_fifo_translator);
	}
	return;
}

static u_int32_t
get_callreference(u_int8_t *ptr);

/*---------------------------------------------------------------------------*
 *	PIPE DATA ACKNOWLEDGE from Layer 3
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_data_acknowledge(DSS1_TCP_pipe_t *pipe, call_desc_t *cd)
{
	struct mbuf *m;
	m = pipe->ifq_head;

	while(m)
	{
	  u_int8_t *ptr;

	  ptr = m->m_data + max(U_FRAME_LEN,I_HEADER_LEN);

	  if(m->m_flags & M_PROTO1)
	  {
	    /* header is adjusted */
	    ptr--;
	  }

	  /* frames are removed by only sending
	   * the header, hence removing a frame from
	   * the queue will break the pipe's tx_nr-
	   * and frame-repetition-system!
	   */
	  if((ptr[0] == PD_Q931) &&
	     (get_callreference(ptr+1) == cd->cr))
	  {
	      /* update length */
	      m->m_len = ptr - ((__typeof(ptr))m->m_data);
	  }

	  m = m->m_nextpkt;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	DATA REQUEST from Layer 2
 *---------------------------------------------------------------------------*/
static void
dss1_l2_data_req(l2softc_t *sc, struct mbuf *m)
{
	if(_IF_QFULL(sc) || 
	   (!(sc->L1_activity)) || 
	   (!(sc->sc_fifo_translator)))
	{
	  NDBGL2(L2_PRIM, "unit=%x, queue full, L1 deactivated or "
		 "no FIFO translator!", sc->sc_unit);
	  m_freem(m);
	}
	else
	{
	  _IF_ENQUEUE(sc,m);

	  /* call transmit FIFO */
	  L1_FIFO_START(sc->sc_fifo_translator);
	}
	return;
}

#include <i4b/dss1/dss1_l3encoder.h>
#include <i4b/dss1/dss1_l3fsm.h>
#include <i4b/dss1/dss1_l3decoder.h>

/*---------------------------------------------------------------------------*
 *	transmit S- or U-frame
 *
 * CR_COMMAND can be used with:
 *	CNTL_RR, CNTL_RNR, CNTL_SABME, CNTL_DISC
 *
 * CR_RESPONSE can be used with:
 *	 CNTL_RR, CNTL_RNR, CNTL_REJ, CNTL_DM, CNTL_UA
 *---------------------------------------------------------------------------*/
static void
dss1_cntl_tx_frame(l2softc_t *sc, DSS1_TCP_pipe_t *pipe, u_int8_t sapi, 
		  u_int8_t cntl)
{
	struct mbuf *m;
	u_int8_t *ptr;

	if(L2_STATE_IS_TEI_ASSIGNED(pipe->state))
	{
	  NDBGL2(L2_S_MSG, "");

	  m = i4b_getmbuf(max(S_FRAME_LEN,U_FRAME_LEN), M_NOWAIT);

	  if(m == NULL)
	  {
	    NDBGL2(L2_S_ERR, "out of mbufs!");
	    goto done;
	  }
	  else
	  {
	    /* S-frame(s) */

	    ptr = m->m_data;

	    ptr[OFF_RX_NR] = (pipe->rx_nr << 1) & 0xFE;

	    if(GET_CR_BIT(sapi) == CR_RESPONSE)
	    {
	      ptr[OFF_RX_NR] |= 1; /* F == 1 */
	    }
	    else /* CR_COMMAND */
	    {
	      /* P == 0, with exception of I-frames */
	    }

	    ptr[OFF_SAPI]   = sapi;
	    ptr[OFF_TEI]    = pipe->tei;
	    ptr[OFF_CNTL]   = cntl;

	    /* NOTE: some implementations does not allow
	     * U-frames to be length-extended:
	     */
	    m->m_len = (!(cntl & 2)) ? S_FRAME_LEN : U_FRAME_LEN;

	    dss1_l2_data_req(sc,m);
	  }
	}
 done:
	return;
}

#define dss1_cntl_tx_frame(sc,pipe,crbit,cntl)			  \
	dss1_cntl_tx_frame(sc,pipe,MAKE_SAPI(SAPI_CCP,crbit),cntl) \
/**/

/*---------------------------------------------------------------------------*
 *	get unused pipe
 *---------------------------------------------------------------------------*/
static DSS1_TCP_pipe_t *
dss1_get_unused_pipe(l2softc_t *sc)
{
	DSS1_TCP_pipe_t *pipe;
	DSS1_TCP_pipe_t *pipe_adapter = &sc->sc_pipe[0];

	PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	{
	    if((pipe->state == ST_L2_PAUSE) &&
	       (pipe != pipe_adapter))
	    {
	        goto found;
	    }
	}
	return 0;

 found:
	/* pipe is initialized when
	 * "dss1_pipe_set_state(pipe,ST_L2_PAUSE)"
	 * is called
	 */
	return pipe;
}

/*---------------------------------------------------------------------------*
 *	count unused pipes
 *---------------------------------------------------------------------------*/
static u_int16_t
dss1_count_unused_pipes(l2softc_t *sc)
{
	DSS1_TCP_pipe_t *pipe;
	DSS1_TCP_pipe_t *pipe_adapter = &sc->sc_pipe[0];
	u_int16_t unused = 0;

	PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	{
	    if((pipe->state == ST_L2_PAUSE) &&
	       (pipe != pipe_adapter))
	    {
	        unused++;
	    }
	}
	return unused;
}

/*---------------------------------------------------------------------------*
 *	get unused, automatic, TEI-value
 * NOTE: this routine is only used in NT-mode
 *---------------------------------------------------------------------------*/
static u_int8_t
dss1_get_unused_TEI(l2softc_t *sc)
{
	DSS1_TCP_pipe_t *pipe;

 repeat:
	M128INC(sc->sc_tei_last);
	        sc->sc_tei_last |= 0x80 DSS1_TEI_IS_ODD(|1);

	PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	{
	    if(pipe->tei == sc->sc_tei_last)
	    {
	        goto repeat;
	    }
	}

	/* NOTE: this routine cannot return TEI_BROADCAST,
	 * because in NT-mode "sc->sc_pipe[0].tei = TEI_BROADCAST"
	 */
	return sc->sc_tei_last;
}

/*---------------------------------------------------------------------------*
 *	encode a TEI management frame
 *
 *	- Ri is a constant serial number loaded from config
 *	- when "type == MT_ID_DENY" Ai must be 0xFF!
 *---------------------------------------------------------------------------*/
static void
dss1_tei_tx_frame(l2softc_t *sc, DSS1_TCP_pipe_t *pipe,
		 u_int8_t type)
{
	struct mbuf *m;
	u_int8_t *ptr;

	if(IS_POINT_TO_POINT(sc))
	{
	    /* not supported */
	    return;
	}

	NDBGL2(L2_TEI_MSG, "type=0x%02x", type);

	if(type == MT_ID_REQUEST)
	{
	  /*
	   * according to Q.921 (03/93) p.25 ID_REQUEST
	   * with Ai in the range 1..127, inclusively, is
	   * ignored. Ai == 0xFF means any TEI value
	   * acceptable
	   */

	  pipe->tei = TEI_BROADCAST;
	}

	m = i4b_getmbuf(TEI_FRAME_LEN, M_NOWAIT);

	if(m)
	{
	  /* build TEI frame */

	  ptr = m->m_data;

          ptr[OFF_SAPI] = MAKE_SAPI(SAPI_L2M,CR_COMMAND);    /* SAPI */
          ptr[OFF_TEI]  = TEI_BROADCAST;	/* TEI = 127, EA = 1 */
          ptr[OFF_CNTL] = CNTL_UI;		/* CNTL = CNTL_UI    */
          ptr[OFF_MEI]  = MEI_TEI_MANAGEMENT;	/* MEI               */
          ptr[OFF_RI+0] = pipe->serial_number;	/* Reference number  */
          ptr[OFF_RI+1] = pipe->serial_number >> 8;
          ptr[OFF_MT]   = type;			/* Message Type      */
	  ptr[OFF_AI]   = pipe->tei;		/* Action indicator  */

	  dss1_l2_data_req(sc,m);
	}
	else
	{
	  NDBGL2(L2_TEI_ERR, "out of mbufs!");
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode a TEI management frame
 *
 *	- with exception of MT_ID_CHECK_RESPONSE,
 *	  EA-bit should be set to 1
 *
 *	- all MT_ID's used in NT-mode should have been grouped!!
 *---------------------------------------------------------------------------*/
static void
ID_REQUEST_timeout(void *sc)
{
	return;
}

static void
dss1_tei_rx_frame(l2softc_t *sc, struct dss1_buffer *buf)
{
	DSS1_TCP_pipe_t *pipe = &sc->sc_pipe[0];
	u_int8_t mt;
	u_int8_t ai;
	u_int8_t ri[2];

	if(IS_POINT_TO_POINT(sc))
	{
	    /* not supported */
	    return;
	}

	mt = dss1_get_1(buf, OFF_MT);
	ai = dss1_get_1(buf, OFF_AI);
	ri[0] = dss1_get_1(buf, OFF_RI+0);
	ri[1] = dss1_get_1(buf, OFF_RI+1);

	NDBGL2(L2_TEI_MSG, "type=0x%02x", mt);

 	switch(mt) {
	case MT_ID_ASSIGN:
		  if(TE_MODE(sc))
		  {
		    if(pipe->tei == ai)
		    {
		      /* TEI-value is assigned to another unit  */
		      dss1_pipe_set_state(pipe,ST_L2_PAUSE);
		    }

		    if(pipe->state == ST_L2_SEND_TEI)
		    {
		      if(pipe->serial_number == ((ri[0]) | 
						 (ri[1] << 8)))
		      {
			pipe->tei = ai;

			i4b_l4_teiasg(sc->sc_unit,pipe->tei);

			log(LOG_INFO,
			    "i4b: unit %d, assigned TEI = %d = 0x%02x\n",
			    sc->sc_unit, pipe->tei, pipe->tei);
  
			NDBGL2(L2_TEI_MSG, "TEI ID Assign - TEI = %d", pipe->tei);
  
			dss1_pipe_set_state(pipe,ST_L2_SEND_SABME);
		      }
		    }
		  }
		  break;

	case MT_ID_DENY: /* TE-mode */
		  /* nothing to do */
		  break;

	case MT_ID_CHECK_REQUEST:
		  /* NOTE: there is no ID_CHECK_REQUEST_ACK */
		  if(L2_STATE_IS_TEI_ASSIGNED(pipe->state)) /* both modes */
		  {
		    if((ai == pipe->tei) ||
		       (ai == TEI_BROADCAST))
		    {
		      dss1_tei_tx_frame(sc,pipe,MT_ID_CHECK_RESPONSE);
		    }
		  }
		  break;

	case MT_ID_REMOVE:
		  if(L2_STATE_IS_TEI_ASSIGNED(pipe->state)) /* both modes */
		  {
		    if((ai == pipe->tei) ||
		       (ai == TEI_BROADCAST))
		    {
		      /* lost TEI-value */
		      dss1_pipe_set_state(pipe,ST_L2_PAUSE);
		    }
		  }
		  break;

	case MT_ID_REQUEST:
		  if(NT_MODE(sc))
		  {
		      if(dss1_count_unused_pipes(sc) <= (PIPE_MAX/2))
		      {
		          /* NOTE: the Network limits the number of
			   * MT_ID_REQUEST-replies over time, allowing
			   * Terminals to finish sending of
			   * MT_ID_REQUEST. If not, all TEI values can be
			   * allocated within eights of a second, to the
			   * same device!
			   */
		          if(__callout_pending(&sc->ID_REQUEST_callout))
			  {
			      break;
			  }
			  __callout_reset(&sc->ID_REQUEST_callout, 1*hz, 
					  ID_REQUEST_timeout, sc);
		      }

		      pipe = dss1_get_unused_pipe(sc);

		      if(pipe)
		      {
			pipe->serial_number = 
			  (ri[0]) | 
			  (ri[1] << 8);
			pipe->tei           = dss1_get_unused_TEI(sc);

			dss1_tei_tx_frame(sc,pipe,MT_ID_REMOVE);
			dss1_tei_tx_frame(sc,pipe,MT_ID_REMOVE);

			dss1_tei_tx_frame(sc,pipe,MT_ID_ASSIGN);

			/* activate the pipe */
			dss1_pipe_activate_req(pipe);
		      }
		  }
		  break;

	case MT_ID_CHECK_RESPONSE: /* both modes */
		  /* this frame may contain
		   * more than one TEI value
		   *
		   * EA == 1 in the final Ai
		   */
		  break;

	case MT_ID_VERIFY: /**/
		  break;

	default:
		  NDBGL2(L2_TEI_ERR, "unknown Message "
			 "Type: 0x%02x", mt);
		  break;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	set timeout pipe state
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_set_state_timeout(DSS1_TCP_pipe_t *pipe)
{
  static const __typeof(pipe->state)
    MAKE_TABLE(L2_STATES,TIMEOUT_STATE,[]);

  l2softc_t *sc = pipe->L5_sc;

  FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
  {
    /* connected */
    dss1_pipe_set_state(pipe, L2_STATES_TIMEOUT_STATE[pipe->state]);
  },
  {
    /* not connected */
  });
  return;
}

/*---------------------------------------------------------------------------*
 *	set pipe state
 *---------------------------------------------------------------------------*/
static void
dss1_pipe_set_state(DSS1_TCP_pipe_t *pipe, u_int8_t newstate)
{
  static const u_int8_t MAKE_TABLE(L2_STATES,TIMEOUT_DELAY,[]);
#if DO_I4B_DEBUG
  static const char * const MAKE_TABLE(L2_STATES,DESC,[]);
#endif

  l2softc_t *sc = pipe->L5_sc;
  DSS1_TCP_pipe_t *pipe_adapter = &sc->sc_pipe[0];

  if((pipe->state <= ST_L2_SEND_TEI) &&
     (newstate >= (ST_L2_SEND_TEI+2)))
  {
    /* cannot set a "TEI assigned state"
     * if the current state is "TEI not assigned"
     */
    return;
  }

   /* update "pipe->state" before calling any functions
    * that might use "pipe->state"!
    */
  pipe->state = newstate;

  /* stop timer */
  __callout_stop(&pipe->set_state_callout);

  if(pipe->state == ST_L2_SINGLE_FRAME)
  {
	if((pipe->refcount == 0) && _IF_QEMPTY(pipe))
	{
	    if(IS_POINT_TO_POINT(sc) ||
	       (NT_MODE(sc) && (pipe_adapter->refcount)))
	    {
	        /* call transmit FIFO, 
		 * to keep the pipe active 
		 */
	        L1_FIFO_START(sc->sc_fifo_translator);
	    }
	    else
	    {
	        /* the pipe is removed when no services
		 * use the pipe, and there is nothing to
		 * send through the pipe
		 */
		if(NT_MODE(sc) && 
		  (!(IS_POINT_TO_POINT(sc))) &&
		  (!(pipe == pipe_adapter)))
		{
		    dss1_cntl_tx_frame(sc,pipe,CR_COMMAND,
				       (CNTL_DISC|CNTL_PF_BIT));
		    dss1_cntl_tx_frame(sc,pipe,CR_COMMAND,
				       (CNTL_DISC|CNTL_PF_BIT));
#if 0
		    if(pipe->tei >= 0x80)
		    {
		        /* remove the TEI value */
		        dss1_tei_tx_frame(sc,pipe,MT_ID_REMOVE);
			dss1_tei_tx_frame(sc,pipe,MT_ID_REMOVE);
		    }
#endif
		}
	        pipe->state = ST_L2_PAUSE;
	    }
	}
  }

  if(pipe->state == ST_L2_SEND_TEI)
  {
      if(pipe->refcount) /* auto activate */
      {
	  if(!IS_POINT_TO_POINT(sc))
	  {
	      if(NT_MODE(sc))
	      {
		  if(pipe == pipe_adapter)
		  {
		      /* pipe-adapter is not assigned 
		       * a TEI-value in NT-mode!
		       */
		      dss1_pipe_resend_ind(pipe);
		      goto done;
		  }
	      }
	      else
	      {
		  if(TEI_IS_AUTOMATIC(pipe))
		  {
		      /* send ID_REQUEST
		       * NOTE: TE-mode has only one pipe
		       */
		      dss1_tei_tx_frame(sc,pipe,MT_ID_REQUEST);
		      goto done;
		  }

		  /* get custom TEI value */
		  pipe->tei = pipe->serial_number;
	      }
	  }

	  /* TEI is already assigned, 
	   * try to get the link up:
	   */
	  pipe->state = ST_L2_SEND_SABME;
      }
      else
      {
	  /* the pipe is removed when no services
	   * use the pipe, and no TEI-value has
	   * been assigned
	   */
	  pipe->state = ST_L2_PAUSE;
      }
  }

  if(pipe->state == ST_L2_SEND_SABME)
  {
	if(TE_MODE(sc) || IS_POINT_TO_POINT(sc))
	{
	    /* Set Asynchronous Balanced Mode Extended 
	     *
	     * NOTE: "pipe->tx_nr" should not be reset here 
	     */
	    dss1_cntl_tx_frame(sc,pipe,CR_COMMAND,(CNTL_SABME|CNTL_PF_BIT));
	}
	goto done;
  }

  if(pipe->state == ST_L2_PAUSE)
  {
	/* pipe is removed through ST_L2_PAUSE */
	dss1_pipe_reset_ind(pipe);

	/* cleanup queue */
	_IF_DRAIN(pipe);

	/* reset variables:
	 *
	 * if frames are outstanding
	 * all frames must be retransmitted
	 */
	pipe->tx_window_size = 0;
	pipe->tx_window_length = 0;

	pipe->rx_nr = 0;
	pipe->tx_nr = 0;

	/* stop re-transmit timeout */
	__callout_stop(&pipe->get_mbuf_callout);

	goto done;
  }

 done:

#if 0
  if(pipe->state >= ST_L2_MULTI_FRAME)
  {
    MDL_Status_Ind(sc->unit,STI_L2STAT,LAYER_ACTIVE);
  }
  else
  {
    MDL_Status_Ind(sc->unit,STI_L2STAT,LAYER_IDLE);
  }
#endif

  /* re-start timeout */
  if(pipe->state != ST_L2_PAUSE)
  {
	__callout_reset(&pipe->set_state_callout,
			(L2_STATES_TIMEOUT_DELAY[pipe->state]*hz),
			(void *)(void *)&dss1_pipe_set_state_timeout, pipe);
  }

  NDBGL2(L2_PRIM,"unit=%x, pipe=%x [%s]",
	 sc->sc_unit, PIPE_NO(pipe), L2_STATES_DESC[pipe->state]);
  return;
}

/*---------------------------------------------------------------------------*
 *	dss1_l2_put_mbuf - process frame from Layer 1
 *---------------------------------------------------------------------------*/
static void
dss1_l2_put_mbuf(fifo_translator_t *f, struct mbuf *m)
{
  DSS1_TCP_pipe_t *pipe;
  __typeof(pipe->tx_nr) __nr;
  l2softc_t *sc = f->L5_sc;
  struct dss1_buffer buf;
  u_int8_t broadcast;
  u_int8_t resp;
  u_int8_t sapi;
  u_int8_t crbit;
  u_int8_t tei;
  u_int8_t cntl;
  u_int8_t mei;

  /* Frame structure:
   * (bits are transmitted from LSB to MSB)
   *
   *  MSB                         LSB
   * +---+---+---+---+---+---+---+---+  OFFSET
   * | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 0 |    -1 HDLC FLAG (not accessible)
   * +---+---+---+---+---+---+---+---+
   * | SAPI                  |C/R| 0 |    +0
   * +---+---+---+---+---+---+---+---+
   * | TEI                       | 1 |    +1
   * +---+---+---+---+---+---+---+---+
   * | CNTL                  |   |   |    +2
   * +---+---+---+---+---+---+---+---+
   * | N(R)                      |P/F|    +3 [some frames only]
   * +---+---+---+---+---+---+---+---+
   * | L3-protocol discriminator     |    +4 [I-frames]
   * +---+---+---+---+---+---+---+---+
   * | L3-call reference length      |    +5 [I-frames]
   * +---+---+---+---+---+---+---+---+
   * | L3-call reference value       |    +6 [I-frames]
   * +---+---+---+---+---+---+---+---+
   * | L3-message type               |    +7 [I-frames]
   * +---+---+---+---+---+---+---+---+
   * |   |   |   |   |   |   |   |   |    +8
   *
   *
   * TEI  : Terminal Endpoint Identifier
   * SAPI : Service Access Point Identifier
   * CNTL : Control byte
   * C    : Command 
   * F    : Final
   * N(R) : 
   * P    : Poll
   * R    : Response
   *  
   *
   * Receiver-Not-Ready should be detected at hardware- and not software-level,
   * and is not supported, because the software is too slow.
   */

  dss1_buf_init(&buf, m->m_data, m->m_len);

  sc->sc_received_frame = 1;

  if(0 /* receiver not ready */)
  {
    resp = CNTL_RNR;
  }
  else
  {
    /* set default response */
    resp = CNTL_RR;
  }

  /* get type bits */
  sapi  = GET_SAPI  (dss1_get_1(&buf,OFF_SAPI));
  crbit = GET_CR_BIT(dss1_get_1(&buf,OFF_SAPI));
  tei   =            dss1_get_1(&buf,OFF_TEI);

  DSS1_CR_IS_INVERTED
  (
     /* TE must invert input C/R-bit */
     if(TE_MODE(sc))
     {
         crbit ^= (CR_COMMAND|CR_RESPONSE);
     }
  )

  /* get default pipe */
  /* lookup pipe */
  /* check TEI value(s) */

  pipe = &sc->sc_pipe[0];

  if(tei == TEI_BROADCAST)
  {
    /**/
    broadcast = 1;
  }
  else
  {
    broadcast = 0;

    if(IS_POINT_TO_POINT(sc))
    {
        if(pipe->tei != tei)
	{
	    goto done;
	}

	/* activate the pipe,
	 * if not already activated:
	 */
	dss1_pipe_activate_req(pipe);
    }
    else
    {
        if(TE_MODE(sc))
	{
	    /* TE-MODE only accepts one TEI value */
	    if((pipe->tei != tei) || 
	       (!L2_STATE_IS_TEI_ASSIGNED(pipe->state)))
	    {
	        goto done;
	    }
	}
	else
	{
	    PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	    {
	        if(pipe->tei == tei)
		{
		    goto TEI_found;
		}
	    }

	    /* create pipe if no match, if possible */
	    pipe = dss1_get_unused_pipe(sc);

	    if(pipe)
	    {
	        pipe->tei = tei;
	    }
	    else
	    {
	        /* let Terminal timeout */
	        goto done;
	    }

	TEI_found:

	    /* activate the pipe,
	     * if not already activated:
	     */
	    dss1_pipe_activate_req(pipe);
	}
    }
  }

  cntl = dss1_get_1(&buf,OFF_CNTL);
  mei = dss1_get_1(&buf,OFF_MEI);

  if(cntl & 1)
  {
	/*
	 * decode S- and U-frame(s)
	 */
	switch(cntl & ~CNTL_PF_BIT) {
	    /*
	     * Unnumbered-command(s)
	     */
	case CNTL_UI:
	    /*
	     * decode SAPI
	     *
	     * It is assumed that "(cntl & 
	     *  ~CNTL_PF_BIT) == CNTL_UI" for all SAPI services
	     */
	    switch(sapi) {
	    case SAPI_L2M:
	        /* layer 2 management (SAPI = 63) */

	        if(mei == MEI_TEI_MANAGEMENT)
		{
		    dss1_tei_rx_frame(sc, &buf);
		}
		break;

	    case SAPI_CCP:
	        /* call control (SAPI = 0) */

	        /* skip header and pass 
		 * unnumbered I-frame to Layer3
		 */
	        buf.offset = U_FRAME_LEN;

		dss1_pipe_data_ind(pipe, &buf, broadcast);

		buf.offset = 0;

		break;

	    default:
	        goto done;
	    }

	    break;

	case CNTL_SABME:
	    /* echo-back CNTL_PF_BIT */
	    dss1_cntl_tx_frame(sc,pipe,CR_RESPONSE,
			       CNTL_UA|(cntl & CNTL_PF_BIT));

	    /* set state after transmit */
	    dss1_pipe_set_state(pipe,ST_L2_SEND_UA);

	    pipe->rx_nr = 0;
	    break;

	case CNTL_DISC:
	    /* echo-back CNTL_PF_BIT */
#if 1
	    dss1_cntl_tx_frame(sc,pipe,CR_RESPONSE,
			       CNTL_UA|(cntl & CNTL_PF_BIT));
#else
	    dss1_cntl_tx_frame(sc,pipe,CR_RESPONSE,
			       CNTL_DM|(cntl & CNTL_PF_BIT));
#endif
	    /* remove pipe */
	    dss1_pipe_set_state(pipe,ST_L2_PAUSE);
	    break;

	default:
	    NDBGL2(L2_PRIM,
		   "unit=0x%x, not decoded cntl:0x%02x, frame: ",
		   sc->sc_unit, cntl);
	    break;
	}

	/* assuming that all S-frames have NR */
	if(!(cntl & 2))
	{
	    goto check_NR;
	}
  }
  else
  {
	/*
	 * decode Information-frame(s)
	 */
 
	/* get remote tx_nr */
	__nr = GET_NR(dss1_get_1(&buf,OFF_TX_NR));

	/* check __nr */
	if(pipe->rx_nr == __nr)
	{
	  /* acknowledge rx_nr */
	  M128INC(pipe->rx_nr);

	  /* skip header and pass 
	   * numbered I-frame to Layer3
	   */
	  buf.offset = I_HEADER_LEN;

	  dss1_pipe_data_ind(pipe, &buf, broadcast);

	  buf.offset = 0;
	}
	else
	{
	  resp = CNTL_REJ;
	}

    check_NR:

	/* store old NR */
	__nr = pipe->tx_nr;

	/* take new NR */
	       pipe->tx_nr = GET_NR(dss1_get_1(&buf,OFF_RX_NR));

	/* get NR difference */
	__nr = pipe->tx_nr - __nr;

	/* assuming that the starting NR is zero: */
	__nr %= NR_MAX;

	if(__nr != 0)
	{
	  /* check for invalid NR increment */

	  if(__nr > pipe->tx_window_length)
	  {
	    /* wrong NR event */
	    dss1_pipe_set_state(pipe,ST_L2_SINGLE_FRAME);

	    /* retransmit all frames */
	    pipe->tx_window_size = pipe->tx_window_length = 0;
	  }
	  else
	  {
	    /* valid NR event
	     * I-FRAMES are cleared and freed elsewhere
	     */
	    dss1_pipe_set_state(pipe,ST_L2_MULTI_FRAME);

	    /* update window_length */
	    pipe->tx_window_length -= __nr;

	    if(pipe->tx_window_length == 0)
	    {
	      /* call transmit FIFO */
	      L1_FIFO_START(sc->sc_fifo_translator);
	    }
	  }
	}

	/* check if response is needed
	 *
	 * (I-frame) and ((P == 1) or NT-mode)
	 *
	 * NOTE: Some buggy PBXs does not set P = 1
	 * when sending I-frames, but still expects
	 * a response.
	 */
	if((((~cntl) & 1) && 
	    ((dss1_get_1(&buf,OFF_RX_NR) & 1) || NT_MODE(sc))) || (resp == CNTL_REJ))
	{
	  dss1_cntl_tx_frame(sc,pipe,CR_COMMAND,resp);
	}

	/* check if response is needed
	 *
	 * (CR_COMMAND) and (P == 1) and (S-frame)
	 */
	if((crbit == CR_COMMAND) && (cntl & dss1_get_1(&buf,OFF_RX_NR) & 1))
	{
	  dss1_cntl_tx_frame(sc,pipe,CR_RESPONSE,CNTL_RR /* or "resp" */); 
	}
  }

 done:

  /* make sure that the mbuf is freed */
  m_freem(m);
  return;
}

/*---------------------------------------------------------------------------*
 *	dss1_l2_get_mbuf_timeout
 *---------------------------------------------------------------------------*/
static void
dss1_l2_get_mbuf_timeout(DSS1_TCP_pipe_t *pipe)
{
  l2softc_t *sc = pipe->L5_sc;

  FIFO_TRANSLATOR_ACCESS(f,sc->sc_fifo_translator,
  {
    /* connected */

    /* retransmit unacked frames */
    pipe->tx_window_size -= pipe->tx_window_length;
    pipe->tx_window_length = 0;

    /* call transmit FIFO */
    L1_FIFO_START(sc->sc_fifo_translator);
  },
  {
    /* not connected */
  });
  return;
}

/*---------------------------------------------------------------------------*
 *	dss1_l2_get_mbuf
 *---------------------------------------------------------------------------*/
static struct mbuf *
dss1_l2_get_mbuf(fifo_translator_t *f)
{
  l2softc_t *sc = f->L5_sc;
  DSS1_TCP_pipe_t *pipe;
  DSS1_TCP_pipe_t *pipe_adapter = &sc->sc_pipe[0];
  DSS1_TCP_pipe_t *pipe_end = &sc->sc_pipe[PIPE_MAX];
  struct mbuf *m;

  _IF_DEQUEUE(sc,m);

  if(m)
  {
    goto done;
  }

  pipe = sc->sc_current_pipe;

  do {
    if(L2_STATE_IS_TEI_ASSIGNED(pipe->state) || 
       (NT_MODE(sc) && (!IS_POINT_TO_POINT(sc)) && (pipe == pipe_adapter)))
    {
      /* send I-frame(s) */

      /* pipe->tx_window_length < 2 */
      if(pipe->tx_window_length == 0)
      {
		__typeof(pipe->tx_window_size) len, nr_length_max;

		len = (pipe->tx_window_size - pipe->tx_window_length);
		       pipe->tx_window_size = pipe->tx_window_length;

		while(len--)
	        {
		  struct mbuf *m;

		  /* remove one frame from I-queue */
		  _IF_DEQUEUE(pipe,m);
		  m_freem(m);
		  m = NULL;
		}

		if(!L2_STATE_IS_TEI_ASSIGNED(pipe->state))
		{
		  /* unnumbered pipe */
		  nr_length_max = 0x7F;
		}
		else
		{
		  /* get maximum unwrapped window_size
		   * and window_length minus one:
		   */

		  nr_length_max = 0x7F - pipe->tx_nr;

#if 0
#define NR_LENGTH_MAX 0x80
		  if(nr_length_max > MAX_I_FRAMES)
		  {
		    nr_length_max = MAX_I_FRAMES;
		    /* ?? */
		  }
#endif

		  /* check if out of NR's */

		  if((pipe->tx_nr == 0x7F) ||
		     (pipe->tx_nr == 0x00) ||
		     (pipe->state <= ST_L2_SINGLE_FRAME))
		  {
		    struct mbuf *m;

		    nr_length_max = 2;

		    /* A zero-length-I-frame is used to guess the remote
		     * rx_nr, when the local tx_nr is invalid.
		     */

		    m = _IF_QUEUE_GET(pipe)->ifq_head;

		    /* check if first frame in IF_QUEUE is a ZIF */
		    if(m && (m->m_len <= I_HEADER_LEN))
		    {
		      goto check_second_ZIF;
		    }

		  insert_second_ZIF:

		    /* enqueue one ZIF */
		    m = i4b_getmbuf(I_HEADER_LEN, M_NOWAIT);
		    if(m)
		    {
		      _IF_ENQUEUE_HEAD(pipe,m);

		    check_second_ZIF:

		      /* m == ifq_head
		       * check if second frame in IF_QUEUE is a ZIF
		       */
		      m = m->m_nextpkt;

		      if(m && (m->m_len <= I_HEADER_LEN))
		      {
			/* done */
		      }
		      else
		      {
			goto insert_second_ZIF;
		      }
		    }
		    else
		    {
		      /* error */
		    }
		  }
		}

		/* reset */

		__callout_stop(&pipe->get_mbuf_callout);

		pipe->tx_window_length = 
		pipe->tx_window_size =
		  min(nr_length_max, _IF_QLEN(pipe));

		sc->sc_current_mbuf = 
		  _IF_QUEUE_GET(pipe)->ifq_head;

		sc->sc_current_length = 
		  pipe->tx_window_size;

		sc->sc_current_tx_nr =
		  pipe->tx_nr;
      }

      if(sc->sc_current_length)
      {
		u_int8_t *ptr;

		/* update header(s), when TEI is valid */

		m = sc->sc_current_mbuf;

		if(!L2_STATE_IS_TEI_ASSIGNED(pipe->state))
		{
		  /* Unnumbered-I-frame(s) (UI) */

		  /* adjust header */
		  if(!(m->m_flags & M_PROTO1))
		  {
		    m->m_flags |= M_PROTO1;
		    m->m_data += 1;
		    m->m_len  -= 1;
		  }

		  ptr = m->m_data;

		  ptr[OFF_SAPI] = MAKE_SAPI(SAPI_CCP, CR_COMMAND);
		  ptr[OFF_TEI ] = TEI_BROADCAST;
		  ptr[OFF_CNTL] = CNTL_UI;

		  /* acknowledge frame */
		  pipe->tx_window_length--;
		}
		else
		{
		  /* I-frame(s) */

		  ptr = m->m_data;
	
		  ptr[OFF_SAPI]  = MAKE_SAPI(SAPI_CCP, CR_COMMAND);
		  ptr[OFF_TEI]   = pipe->tei;
		  ptr[OFF_TX_NR] = (sc->sc_current_tx_nr << 1) & 0xfe; /* bit 0 = 0 (tx_nr) */
		  ptr[OFF_RX_NR] = (pipe->rx_nr << 1) & 0xfe; /* P bit = 0 (rx_nr) */

		  if(sc->sc_current_length == 1)
		  {
		    /* the remote end must transmit a
		     * response on every I-frame
		     * with (P == 1)
		     */

		    /* P bit == 1 */
		    ptr[OFF_RX_NR] |= 1;
		  }
		  else
		  {
		    /* P bit == 0 */
		  }
		}

		sc->sc_current_mbuf = m->m_nextpkt;

		sc->sc_current_length--;

		sc->sc_current_tx_nr++;

		/* check for ZIF, Zero length I-Frame */

		if(m->m_len < ((m->m_flags & M_PROTO1) ? 
			       (I_HEADER_LEN) : 
			       (I_HEADER_LEN+1)))
		{
		    u_int8_t len = m->m_len;
		    ptr = m->m_data;

		    /*
		     * NOTE: some PBXs lack a check for
		     * zero length I-frames, and obviously
		     * starts decoding information that is
		     * not there giving random results ...
		     */

		    m = i4b_getmbuf(len+16, M_NOWAIT);
		    if(m)
		    {
		        bcopy(ptr,m->m_data,len);

			ptr = ((u_int8_t *)(m->m_data)) + len;

			/* send a dummy RELEASE_COMPLETE */

			*ptr++ = PD_Q931;
			 ptr   = make_callreference(pipe_adapter, 0x7f, ptr);
			*ptr++ = RELEASE_COMPLETE;
			*ptr++ = IEI_CAUSE;
			*ptr++ = IEI_CAUSE_LEN;
			*ptr++ = NT_MODE(sc) ? 
			  CAUSE_STD_LOC_PUBLIC : 
			  CAUSE_STD_LOC_OUT;
			*ptr++ = 0x10|EXT_LAST; /* normal call clearing */

			m->m_len = ptr - ((__typeof(ptr))m->m_data);
		    }
		}
		else
		{
		    /* make a freeable copy 
		     * of this I-frame 
		     * for Layer 1
		     */
		    m = m_copypacket(m, M_DONTWAIT);
		}

		if(!m)
		{
		    NDBGL2(L2_ERROR, "out of mbufs!");
		}
      }
    }

    if(m)
    {
        break;
    }
    else
    {
      if(_IF_QLEN(pipe))
      {
	/**/
	if(!__callout_pending(&pipe->get_mbuf_callout))
	{
	  /* re-start timeout */
	  __callout_reset(&pipe->get_mbuf_callout, T200DEF,
			  (void *)(void *)&dss1_l2_get_mbuf_timeout, pipe);
	}
      }
    }

    /* clear current length so that
     * other pipes does not send
     * out the frames [if any]
     */
    sc->sc_current_length = 0;

    pipe++;

    if(pipe >= pipe_end)
    {
        pipe -= PIPE_MAX;
    }

  } while(pipe != sc->sc_current_pipe);

  sc->sc_current_pipe = pipe;

 done:

  if(m)
  {
    DSS1_CR_IS_INVERTED
    (
	  /* NT must invert output C/R-bit */
	  if(NT_MODE(sc))
	  {
	    ((u_int8_t *)m->m_data)[OFF_SAPI] ^= 0x02;
	  }
    )
  }
  return m;
}

static l2softc_t l2_softc[MAX_CONTROLLERS];

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
dss1_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	      struct i4b_protocol *pp, u_int32_t driver_type, 
	      u_int32_t driver_unit, call_desc_t *cd)
{
	DSS1_TCP_pipe_t *pipe;
	l2softc_t *sc = &l2_softc[driver_unit];
	u_int16_t max_channels;

	if(!pp)
	{
	  return (driver_unit < MAX_CONTROLLERS) ? 
	    sc->sc_fifo_translator : FT_INVALID;
	}

	if(pp->protocol_1)
	{
#define BZERO(ptr) bzero(ptr,sizeof(*ptr))

	  /* set all call-descriptors free */
	  BZERO(&cntl->N_call_desc);

	  /* set all channels free */
	  BZERO(&cntl->N_channel_utilization);

	  /**/
	  BZERO(sc);
	}

	cntl->N_fifo_translator =
	  sc->sc_fifo_translator = f;

	if(pp->protocol_1)
	{
	  /* connected */

	  __typeof(pipe->__pn)
	    pipe_number = 0;

	  f->L5_sc       = sc;
	  f->L5_GET_MBUF = &dss1_l2_get_mbuf;
	  f->L5_PUT_MBUF = &dss1_l2_put_mbuf;

	  pp->protocol_1 = P_HDLC;

	  cntl->N_RETRIEVE_REQUEST = n_retrieve_request;
	  cntl->N_HOLD_REQUEST = n_hold_request;
	  cntl->N_DEFLECT_REQUEST = n_deflect_request;
	  cntl->N_MCID_REQUEST = n_mcid_request;
	  cntl->N_CONNECT_REQUEST = n_connect_request;
	  cntl->N_INFORMATION_REQUEST = n_information_request;
	  cntl->N_CONNECT_RESPONSE = n_connect_response;
	  cntl->N_DISCONNECT_REQUEST = n_disconnect_request;
	  cntl->N_ALERT_REQUEST = n_alert_request;        
	  cntl->N_PROGRESS_REQUEST = n_progress_request;        
	  cntl->N_ALLOCATE_CD = n_allocate_cd;
	  cntl->N_FREE_CD = n_free_cd;

	  cntl->N_cdid_end = 0x7F; /* exclusive */
	  /* cntl->N_cdid_count = use last value */

	  cntl->N_lapdstat = NULL;

#define MAX_QUEUE_PER_CHANNEL 16

	  max_channels = cntl->L1_channel_end;

	  if(max_channels < 3)
	     max_channels = 3;

	  if(cntl->L1_type == L1_TYPE_ISDN_PRI)
	    sc->sc_primary_rate = 1;
	  else
	    sc->sc_primary_rate = 0;

	  if((driver_type == DRVR_DSS1_P2P_TE) ||
	     (driver_type == DRVR_DSS1_P2P_NT))
	    sc->sc_point_to_point = 1;
	  else
	    sc->sc_point_to_point = 0;

	  /* set default queue length */

	  _IF_QUEUE_GET(sc)->ifq_maxlen =
	    max_channels * MAX_QUEUE_PER_CHANNEL;

	  if((driver_type == DRVR_DSS1_NT) ||
	     (driver_type == DRVR_DSS1_P2P_NT))
	  {
	    /* set NT-mode */
	    i4b_l1_set_options(cntl, 
			       I4B_OPTION_NT_MODE, 
			       I4B_OPTION_NT_MODE);

	    cntl->N_nt_mode = 1;
	    sc->sc_nt_mode = 1;

	    if(!IS_POINT_TO_POINT(sc))
	    {
	        _IF_QUEUE_GET(sc)->ifq_maxlen =
		  max_channels * (PIPE_MAX * MAX_QUEUE_PER_CHANNEL);
	    }
	  }
	  else
	  {
	    /* set TE-mode */
	    i4b_l1_set_options(cntl, 
			       I4B_OPTION_NT_MODE, 
			       0);

	    cntl->N_nt_mode = 0;
	    sc->sc_nt_mode = 0;
	  }
#if 0
	  _IF_QUEUE_GET(sc)->ifq_maxlen = IFQ_MAXLEN;
#endif

	  /* initialize the callout handles for timeout routines */
	  __callout_init_mtx(&sc->ID_REQUEST_callout, 
			     CNTL_GET_LOCK(cntl), 0);

	  __callout_init_mtx(&sc->L1_activity_callout, 
			     CNTL_GET_LOCK(cntl), 0);

	  sc->sc_cntl = cntl;
	  sc->sc_unit = cntl->unit;

	  /* start L1-activity-callout */
	  dss1_L1_activity_timeout(sc);

	  sc->sc_current_pipe = &sc->sc_pipe[0];

	  L1_COMMAND_REQ(cntl,CMR_SET_L1_AUTO_ACTIVATE_VARIABLE,&sc->L1_auto_activate);
	  L1_COMMAND_REQ(cntl,CMR_SET_L1_ACTIVITY_VARIABLE,&sc->L1_activity);

	  PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	  {
	    pipe->__pn = pipe_number++;

	    pipe->L5_sc = sc;

	    pipe->tei = (IS_POINT_TO_POINT(sc) && 
			 (PIPE_NO(pipe) == 0)) ? 
	      TEI_POINT2POINT :
	      TEI_BROADCAST;

	    pipe->serial_number = 
	      ((cntl->N_serial_number + PIPE_NO(pipe))
	       DSS1_TEI_IS_ODD(*2)) DSS1_TEI_IS_ODD(|1);

	    __callout_init_mtx(&pipe->set_state_callout, 
			       CNTL_GET_LOCK(cntl), 0);

	    __callout_init_mtx(&pipe->get_mbuf_callout, 
			       CNTL_GET_LOCK(cntl), 0);
#if 0
	    _IF_QUEUE_GET(pipe)->ifq_maxlen = IFQ_MAXLEN;
#else
	    _IF_QUEUE_GET(pipe)->ifq_maxlen = max_channels * MAX_QUEUE_PER_CHANNEL;
#endif
	  }

	  /* own receiver is always ready */

	  /* reserve D-channel */
	  SET_CHANNEL_UTILIZATION(cntl,CHAN_D1,1);
	}
	else
	{
	  /* not connected */

	  sc->L1_activity = 0; /* clear activity */
	  sc->L1_auto_activate = 0; /* clear auto-activate */
	  sc->L1_deactivate_count = 0; /* clear deactivate-count */
	  sc->sc_current_length = 0; /* clear current-length */

	  cntl->N_lapdstat = NULL;

	  /* untimeout */
	  __callout_stop(&sc->ID_REQUEST_callout);
	  __callout_stop(&sc->L1_activity_callout);

	  PIPE_FOREACH(pipe,&sc->sc_pipe[0])
	  {
		/**/
		dss1_pipe_set_state(pipe,ST_L2_PAUSE);

		/* _IF_DRAIN(pipe); */
		if(pipe->refcount != 0)
		{
		  /* error */
		}

		/* untimeout */
		__callout_stop(&pipe->set_state_callout);
		__callout_stop(&pipe->get_mbuf_callout);
	  }

	  _IF_DRAIN(sc);

	  L1_COMMAND_REQ(cntl,CMR_SET_L1_AUTO_ACTIVATE_VARIABLE,NULL);
	  L1_COMMAND_REQ(cntl,CMR_SET_L1_ACTIVITY_VARIABLE,NULL);
	}
	return f;
}

