/*-
 * Copyright (c) 2002 Hans Petter Selasky. All rights reserved.
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
 *	i4b_filter.h - data filters/feeders (read and write)
 * 	----------------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_FILTER_H_
#define _I4B_FILTER_H_

#include <i4b/layer1/i4b_hdlc.h>

/* ==================================================
 * EXTENDED TRANSPARENT MODE EXAMPLES
 * ==================================================
 *
 * transmit direction:
 *
 * do {
 *	    FIFO_GET_MEMORY(sc,f,&dst,&dst_end,&len);
 *	    f->buf_ptr = dst;
 *	    f->buf_len = len;
 *
 *	    while(dst != dst_end)
 *	    {
 *		dst++;
 *	    }
 *
 *	    f->buf_len = dst - f->buf_ptr;
 *	    filter_tx(sc,f);
 *
 *	    if(dst < dst_end) 
 *	    {
 *		out of data
 *		break;
 *	    }
 *
 * } while(f->Z_chip);
 *
 * receive direction:
 *
 * do {
 *	    FIFO_GET_MEMORY(sc,f,&src,&src_end,&len);
 *	    f->buf_ptr = src;
 *	    f->buf_len = len;
 *	    filter_rx(sc,f);
 *
 *	    while(src != src_end)
 *	    {
 *		src++;
 *	    }
 *
 *	    if(src < src_end)
 *	    {
 *		cannot receive more data
 *		break;
 *	    }
 *
 * } while(f->Z_chip);
 */

/*
 * ==================================================
 * SOFTWARE RECEIVE FILTER(S)
 * ==================================================
 *
 * NOTE: routine names starting with rx_ or tx_ are
 *       filters. Routine names ending with _rx or
 *       _tx are sub routines used by the filters.
 *
 * NOTE: a receive filter should read the hardware
 *	 buffer till it is empty (f->Z_chip == zero).
 *
 * NOTE: default value after fifo setup, for all
 *	 f->buf_xxx variables is zero.
 *
 * NOTE: (f->Z_chip) is checked after that the
 *	 buffer has been setup, to allow zero frames:
 *	 do {
 *		if(no buffer) setup buffer;
 *		. . .
 *
 *	 } while(f->Z_chip);
 *
 * ==================================================
 * Parameters for filter_rx:
 *
 * f->Z_chip  == largest fifo transfer, in bytes, from
 *               chip's [internal] buffer
 *
 * f->buf_len == current receive buffer length
 * f->buf_ptr == current receive buffer position
 *
 * NOTE: all parameters are updated after the 
 *       fifo transfer
 * ==================================================
 */

static void
filter_rx FIFO_FILTER_T(sc,f)
{
	register int io_len;
        /* buf.curr |---> buf.end */
        (io_len)          = min((f->Z_chip),(f->buf_len));

	/* debugging */
        IHFC_MSG("RX: %d\n", (io_len));
        
        /* RX data */
        FIFO_READ_MULTI_1(sc,f,(f->buf_ptr),(io_len));

	/* echo cancel */
	if((f->prot_curr.protocol_1 == P_TRANSPARENT) &&
	   (f->prot_curr.u.transp.echo_cancel_enable))
	{
	    struct i4b_echo_cancel *ec = &(sc->sc_echo_cancel[FIFO_NO(f)/2]);
 	    i4b_echo_cancel_merge(ec, f->buf_ptr, io_len);
	}

        /* post increment buf_ptr */
        (f->buf_ptr)     += (io_len);

        /* post decrement buf_len and Z_chip */
        (f->buf_len)     -= (io_len);
        (f->Z_chip)      -= (io_len);

	return;
}

static void
get_mbuf_rx FIFO_FILTER_T(sc,f)
{
	/* NOTE:   when f->ifqueue is  full no
	 * further mbufs should be allocated.
	 * This is checked by  ihfc_i4b_putmbuf
	 * and if the  f->ifqueue is  full  the
	 * f->mbuf will be freed,  and probably
	 * the same mbuf will be allocated here
	 * again.
	 */

	f->mbuf = L5_ALLOC_MBUF(FIFO_TRANSLATOR(sc,f), BCH_MAX_DATALEN, f->Z_chip);;

	if(f->mbuf) {
	  /* setup buffer */
	  f->buf_size =
	  f->buf_len  = f->mbuf->m_len; /* variable length ! */
	  f->buf_ptr  = f->mbuf->m_data;
	} else {
	  /* reset buffer */
	  f->buf_size =
	  f->buf_len  = 0;
	  f->buf_ptr  = 0;
	}

	return;
}

static void
rx_transparent FIFO_FILTER_T(sc,f)
{
	do {
	  /* check f->buf_len */
	  if(!f->buf_len)
	  {
		if(f->mbuf)
		{
		  /* assume frame is done */
		  ihfc_i4b_putmbuf(sc, f, f->mbuf);

		  /* mark mbuf as unused */
		  f->mbuf = 0;
		}

		/* f->mbuf,
		 * f->buf_size,
		 * f->buf_len and,
		 * f->buf_ptr will be 
		 * setup by get_mbuf_rx
		 */
		get_mbuf_rx(sc,f);

		if(!f->mbuf)
		{
		  /* no buffer available
		   * (goto done)
		   */
		  goto done;
		}
	  }

	  /* continue receive till mbuf
	   * is full or f->Z_chip is zero.
	   */
	  filter_rx(sc,f);

	} while(f->Z_chip);

 done:
	/* echo cancel */
	if((f->prot_curr.protocol_1 == P_TRANSPARENT) &&
	   (f->prot_curr.u.transp.echo_cancel_enable))
	{
	    struct i4b_echo_cancel *ec = &(sc->sc_echo_cancel[FIFO_NO(f)/2]);
	    i4b_echo_cancel_update_merger(ec, f->Z_read_time - f->Z_chip);
	}
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = receive,
	.protocol[0] = P_TRANS,
	.filter      = &rx_transparent,
};

static void
rx_hdlc FIFO_FILTER_T(sc,f)
{
	if(f->state & ST_FRAME_ERROR)
	{
		/* restore original pointers
		 * instead of freeing and
		 * allocating an mbuf.
		 */
		f->buf_ptr -= f->buf_size - f->buf_len;
		f->buf_len  = f->buf_size;
	}

	do {
	  /* check f->buf_len */
	  if(!f->buf_len)
	  {
		if(f->mbuf) /* RDO */
		{
		  if(f->Z_chip) /* RDO */
		  {
		    /* the  leftover of the frame
		     * will only be dumped if the
		     * filter     returns    when
		     * f->Z_chip is non-zero (see
		     * "framing rules"  in  the
		     * file i4b_program.h)
		     */

		    goto rx_hdlc_error;
		  }
		  else /* wait */
		  {
		    /* if f->buf_len is zero no
		     * more data can be received (break)
		     *
		     * wait for ST_FRAME_END or ST_FRAME_ERROR, or
		     * wait for RDO...
		     */
		    break;
		  }
		}
		else /* new frame */
		{
		  /* f->mbuf,
		   * f->buf_size,
		   * f->buf_len and,
		   * f->buf_ptr will be 
		   * setup by get_mbuf_rx
		   */
		  get_mbuf_rx(sc,f);

		  if(!f->mbuf)
		  {
		    /* no buffer available
		     * (goto done)
		     */
		    goto done;
		  }
		}
	  }

	  /* continue receive till mbuf
	   * is full or f->Z_chip is zero.
	   */
	  filter_rx(sc,f);

	} while(f->Z_chip);

	if(f->state & ST_FRAME_END)
	{
		/* set mbuf length */
		f->mbuf->m_len = f->buf_size - f->buf_len;

		/* NOTE: the FIFO_FRAME_CHECK should
		 * not alter the mbuf unless the
		 * frame is valid!
		 */
		if(!FIFO_FRAME_CHECK(sc,f,f->mbuf))
		{
		    /* send f->mbuf to upper layers */
		    ihfc_i4b_putmbuf(sc, f, f->mbuf);

		    /* reset mbuf and buffer */
		    f->mbuf     = NULL;
		    f->buf_ptr  = 0;
		    f->buf_len  = 0;
		    f->buf_size = 0;
		}
		else
		{

		rx_hdlc_error:

		    /* restore original pointers
		     * instead of freeing and
		     * allocating an mbuf.
		     */

		    f->buf_ptr -= f->buf_size - f->buf_len;
		    f->buf_len  = f->buf_size;
		}
	}
 done:
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = receive,
	.protocol[0] = P_HDLC,
	.filter      = &rx_hdlc,
};

static void
rx_transparent_ring FIFO_FILTER_T(sc,f)
{
	do {
	  /* check f->buf_len */
	  if(!f->buf_len)
	  {
		int len;

		/* write to buffer */
		BUF_GET_WRITELEN(&f->buf,&len);

		if(len) {
		  /* setup buffer */
		  f->buf_size  =
		  f->buf_len   =  len;
		  f->buf_ptr   =  f->buf.Dat_end;
		} else {
		  /*
		   * f->buf_size == 0 &&
		   * f->buf_len  == 0;
		   * (same reasoning as for
		   *  tx_transparent_ring)
		   *
		   * if len is zero no more data
		   * can be received (break)
		   */
		  break;
		}
          }

	  /* continue receive till buffer
	   * is full or f->Z_chip is zero.
	   */
	  filter_rx(sc,f);

	  /* update fifospace counters (f->buf.)
	   *
	   * update write ptr (f->buf.Dat_end)
	   * and shrink buffer block (f->buf_xxx)
	   * size. (also see filter_rx(,))
	   */

          f->buf.Dat_end += (f->buf_size) - (f->buf_len);
	                    (f->buf_size) = (f->buf_len);

	} while(f->Z_chip);

	L5_RX_INTERRUPT(FIFO_TRANSLATOR(sc,f),&f->buf);

	return;
}

I4B_FILTER_EXPORT()     = {
	.direction      = receive,
	.protocol[0]    = P_TRANS_RING,
	.buffersize     = 2048,
	.filter         = &rx_transparent_ring,
	.rxtx_interrupt = NULL, /* must be set by driver */
};

static void
rx_hdlc_emulation FIFO_FILTER_T(sc,f)
{
                 u_int8_t * src_end;
		 u_int8_t * src;
	register u_int8_t   blevel;
	register u_int16_t  crc;
	register u_int16_t  tmp;
	register u_int16_t  ib;
	register u_int8_t * dst;
	         u_int16_t  len;
	register u_int16_t  tmp2;

	do {
	    /* get buffer */
	    FIFO_GET_MEMORY(sc,f,&src,&src_end,&len);
	    f->buf_ptr = src;
	    f->buf_len = len;

	    /* RX data */
	    filter_rx(sc,f);

	    /* restore HDLC variables */
	    blevel  = f->hdlc.blevel;
	    crc     = f->hdlc.crc;
	    tmp     = f->hdlc.tmp;
	    ib      = f->hdlc.ib;

	    /* restore "len" and "dst"
	     * if "f->mbuf" is present
	     */
	    if(f->mbuf)
	    {
		/* resume */
	        len = f->mbuf_rem_length;
		if(len > f->mbuf->m_len)
		{
		    HDLC_ERR("fifo(#%d) invalid remaining length: %d!\n",
			     FIFO_NO(f), len);

		    len = f->mbuf->m_len;
		}
		dst = f->mbuf->m_data + (f->mbuf->m_len - len);
	    }
	    else
	    {
		len = 0;
		dst = NULL;
	    }

	    while(src != src_end)
	    {
		HDLC_DECODE(*dst++, len, tmp, tmp2, blevel, ib, crc, (f->hdlc.flag),
		{/* rdd */
			tmp2 = *src++;
		},
		{/* nfr */
			f->mbuf = L5_ALLOC_MBUF
			  (FIFO_TRANSLATOR(sc,f), BCH_MAX_DATALEN, 0);

			if(!f->mbuf) {
			  HDLC_ERR("fifo(#%d) out of mbufs!\n",
				   FIFO_NO(f));
			  (f->hdlc.flag) = -1; /* reset */
			} else {
			  dst = f->mbuf->m_data;
			  len = f->mbuf->m_len; /* remaining length */
			}
		},
		{/* cfr */
			len = (f->mbuf->m_len - len);

			if ((!len) || (len > BCH_MAX_DATALEN))
			{
				/*
				 * NOTE: frames without any data,
				 * only crc field, should be silently
				 * discarded.
				 */

				I4B_FREEMBUF(f,f->mbuf);
				HDLC_MSG("fifo(#%d) had a bad "
					 "frame(len=%d).\n",
					 FIFO_NO(f), len);
				goto s0;
			}

			if (crc)
			{	I4B_FREEMBUF(f,f->mbuf);
				HDLC_ERR("fifo(#%d) had a "
					 "CRC error(crc=0x%04x, "
					 "len=%d).\n", FIFO_NO(f), crc, len);
				goto s0;
			}

			f->mbuf->m_len = len;

			ihfc_i4b_putmbuf(sc, f, f->mbuf);
		 s0:
			f->mbuf = NULL;
		},
		{/* rab */
			I4B_FREEMBUF(f,f->mbuf);
			f->mbuf = NULL;

			HDLC_MSG("fifo(#%d) had a Read Abort command.\n",
				 FIFO_NO(f));
		},
		{/* rdo */
			I4B_FREEMBUF(f,f->mbuf);
			f->mbuf = NULL;

			HDLC_ERR("fifo(#%d) had Data Overflow "
				 "(len > MAX_FRAME_SIZE).\n", FIFO_NO(f));
		},
 		continue,
		d);
	    }

	    /* keep track of the 
	     * remaining mbuf length
	     */
	    if(f->mbuf)
	    {
		f->mbuf_rem_length = len;
	    }

	    /* suspend HDLC variables */
	    f->hdlc.ib       = ib;
	    f->hdlc.crc      = crc;
	    f->hdlc.tmp      = tmp;
	    f->hdlc.blevel   = blevel;

	} while(f->Z_chip);
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = receive,
	.protocol[0] = P_HDLC_EMU,
	.protocol[1] = P_HDLC_EMU_D,
	.filter      = &rx_hdlc_emulation,
};

/*
 * ==================================================
 * SOFTWARE TRANSMIT FILTER(s)
 * ==================================================
 *
 * NOTE: a transmit filter should write the hardware
 *	 buffer till it is full (f->Z_chip == zero).
 *
 * NOTE: default value after fifo setup, for all
 *	 f->buf_xxx variables is zero.
 *
 * ==================================================
 * Parameters for filter_tx:
 *
 * f->Z_chip  == largest fifo transfer, in bytes, to
 *               chip's [internal] buffer
 *
 * f->buf_len == current transmit buffer length
 * f->buf_ptr == current transmit buffer position
 *
 * NOTE: all parameters are updated after the 
 *       fifo transfer
 * ==================================================
 */
static void
filter_tx FIFO_FILTER_T(sc,f)
{
	register int io_len;
        /* buf.curr |---> buf.end */
        (io_len)          = min((f->Z_chip),(f->buf_len));

	/* debugging */
        IHFC_MSG("TX: %d\n", (io_len));

        /* TX data */
        FIFO_WRITE_MULTI_1(sc,f,(f->buf_ptr),(io_len));

	/* echo cancel */
	if((f->prot_curr.protocol_1 == P_TRANSPARENT) &&
	   (f->prot_curr.u.transp.echo_cancel_enable))
	{
	    struct i4b_echo_cancel *ec = &(sc->sc_echo_cancel[FIFO_NO(f)/2]);
 	    i4b_echo_cancel_feed(ec, f->buf_ptr, io_len);
	}

        /* post increment buf_ptr */
        (f->buf_ptr)     += (io_len);

	/* post decrement buf_len and Z_chip */
        (f->buf_len)     -= (io_len);
        (f->Z_chip)      -= (io_len);

	/* increment number of bytes in FIFO */
	(f->Z_chip_written) += (io_len);
}

static void
get_mbuf_tx FIFO_FILTER_T(sc,f)
{
	/* free last mbuf, if any.
	 * NOTE: f->mbuf may be zero
	 */
	I4B_FREEMBUF(f,f->mbuf);

	f->mbuf = ihfc_i4b_getmbuf(sc, f);

	if(f->mbuf) {
	  /* setup buffer */
	  f->buf_size =
          f->buf_len  =  f->mbuf->m_len;
	  f->buf_ptr  =  f->mbuf->m_data;
	} else {
	  /* reset buffer */
	  f->buf_size =
	  f->buf_len  = 0;
	  f->buf_ptr  = 0;
	}
}

static void
tx_transparent FIFO_FILTER_T(sc,f)
{
	do {
	  /* check f->buf_len */
	  if(!f->buf_len)
	  {
		/* assume frame is done */
		get_mbuf_tx(sc,f);

		if(!f->mbuf)
		{
		  /* no buffer available
		   * (goto done)
		   */
		  goto done;
		}
	  }

	  /* continue transmit till mbuf
	   * is empty or f->Z_chip is zero.
	   */
	  filter_tx(sc,f);

	} while(f->Z_chip);

 done:
	/* echo cancel */
	if((f->prot_curr.protocol_1 == P_TRANSPARENT) &&
	   (f->prot_curr.u.transp.echo_cancel_enable))
	{
	    struct i4b_echo_cancel *ec = &(sc->sc_echo_cancel[FIFO_NO(f)/2]);
	    i4b_echo_cancel_update_feeder(ec, f->Z_read_time + f->Z_chip_written);
	}
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = transmit,
	.protocol[0] = P_TRANS,
	.filter      = &tx_transparent,
};

static void
tx_hdlc FIFO_FILTER_T(sc,f)
{
	if(f->state & ST_FRAME_ERROR)
	{
		/* restore original pointers
		 * and try repeating the current
		 * frame
		 */
		f->buf_ptr -= f->buf_size - f->buf_len;
		f->buf_len  = f->buf_size;
	}

	do {
	  /* check f->buf_len */
	  if(!f->buf_len)
	  {
		if(f->mbuf)
		{ /* currently add a flag after all mbufs and
		   * ignore "datagrams" B+D-channel. Set
		   * ST_FRAME_END according to "framing rules".
		   * (see i4b_program.h)
		   */

		  f->state |= ST_FRAME_END;
		}

		/* f->mbuf,
		 * f->buf_size,
		 * f->buf_len and,
		 * f->buf_ptr will be 
		 * setup by get_mbuf_tx
		 */
		get_mbuf_tx(sc,f);

		if((f->mbuf == NULL) ||
		   (f->state & ST_FRAME_END))
		{
		  /* no buffer available or
		   *
		   * wait till ST_FRAME_END is cleared
		   * wait till flag sequence has been added
		   */

		  goto done;
		}
	  }

	  /* continue transmit till mbuf
	   * is empty or f->Z_chip is zero.
	   */
	  filter_tx(sc,f);

	} while(f->Z_chip);

 done:
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = transmit,
	.protocol[0] = P_HDLC,
	.filter      = &tx_hdlc,
};

static void
tx_transparent_ring FIFO_FILTER_T(sc,f)
{
	L5_TX_INTERRUPT(FIFO_TRANSLATOR(sc,f),&f->buf);

	do {
          /* provide faster data access
           * for phone / audio / multimedia /
           * codecs etc.
           *
           * NOTE: not all hardware will repeat the
           * last byte when the buffer runs out.
	   */

	  /* check f->buf_len */
	  if(!f->buf_len)
          {
		int len;

		/* read from buffer */
		BUF_GET_READLEN(&f->buf,&len);

		if(len) {
		  /* setup buffer */
		  f->buf_size = 
		  f->buf_len  =  len;
		  f->buf_ptr  =  f->buf.Dat_start;
		} else {
		  /* default values after
		   * fifo setup:
		   *
		   * f->buf_size == 0 &&
		   * f->buf_len  == 0
		   *
		   * and below
		   * (f->buf_size) = (f->buf_len);
		   *
		   * which means that if
		   * (f->buf_len) is zero,
		   * (f->buf_size) is also zero
		   * and then (f->buf.Dat_start)
		   * is not changed: (see release 
		   * fifospace below)
		   *
		   * Reset f->buf.Dat_xxx when
		   * it is empty so that a
		   * read doesn't start at
		   * the end of the buffer:
		   */
		  f->buf.Dat_start =
		    f->buf.Dat_end =
		    f->buf.Buf_start;
		  /*
		   * if len is zero no more data
		   * can be transmitted (break)
		   */
		  break;
		}
          }

	  /* continue transmit till mbuf
	   * is empty or f->Z_chip is zero.
	   */
	  filter_tx(sc,f);

	  /* release fifospace:
	   *
	   * update read ptr (f->buf.Dat_start)
	   * and shrink buffer block (f->buf_xxx)
	   * size. (also see filter_tx(,))
	   */
          f->buf.Dat_start += (f->buf_size) - (f->buf_len);
	                      (f->buf_size) = (f->buf_len);

	} while(f->Z_chip);

	return;
}

I4B_FILTER_EXPORT()     = {
	.direction      = transmit,
	.protocol[0]    = P_TRANS_RING,
	.buffersize     = 2048,
	.filter         = &tx_transparent_ring,
	.rxtx_interrupt = NULL, /* must be set by driver! */
};

static void
tx_hdlc_emulation FIFO_FILTER_T(sc,f)
{
                 u_int8_t * dst_end;
	         u_int8_t * dst;
	register u_int16_t  blevel;
	register u_int32_t  tmp;
	register u_int16_t  crc;
	register u_int16_t  ib;
	register u_int8_t * src;
	         u_int16_t  len;
	register u_int32_t  tmp2;

	do {
	  /* get buffer */
	  FIFO_GET_MEMORY(sc,f,&dst,&dst_end,&len);
	  f->buf_ptr = dst;
	  f->buf_len = len;

#if 0
	  if (!f->mbuf && IF_QEMPTY(&f->ifqueue) && (f->hdlc.flag == 2))
	  {
	      /* no data to encode */
	      break;
	  }
#endif

	  /* restore HDLC variables */
	  blevel  = f->hdlc.blevel;
	  tmp     = f->hdlc.tmp;
	  crc     = f->hdlc.crc;
	  ib      = f->hdlc.ib;

	  /* restore "src" and "len"
	   * if "f->mbuf" is present
	   */
	  if(f->mbuf)
	  {
		/* resume */
		src = f->mbuf->m_data;
		len = f->mbuf->m_len;

		/* currently do nothing on XDU hence
		 * it is hard to know which frame
		 * was XDU, due to buffering.
		 * if(f->state & ST_FRAME_ERROR) {
		 *	 * XDU * 
		 *	(f->hdlc.flag) = -2;
		 *	          len  =  0;
		 *
		 *	HDLC_ERR("fifo(#%d) XDU.\n", FIFO_NO(f));
		 * }
		 */
	  }
	  else
	  {
		src = 0;
		len = 0;
	  }

	  while(dst != dst_end)
	  {
		HDLC_ENCODE(*src++, len, tmp, tmp2, blevel, ib, crc, (f->hdlc.flag),
		{/* gfr */
			I4B_FREEMBUF(f,f->mbuf);
			f->mbuf = ihfc_i4b_getmbuf(sc, f);

			if(f->mbuf)
			{
				src = f->mbuf->m_data;
				len = f->mbuf->m_len;
			}
			else
			{
				dst_end = dst + 1;

				/* exit after final FS  *
				 * (flag sequence),     *
				 * else the buffer will *
				 * only be filled with  *
				 * "0x7e"-bytes!        *
				 * Usually the hardware *
				 * will repeat the last *
				 * byte.                */
			}
		},
		{/* nmb */
		},
		{/* wrd */
			*dst++ = (u_int8_t)(tmp);
		},
		dd );
	  }

	  /* suspend "f->mbuf->m_len" and "f->mbuf->m_data"
	   * if "f->mbuf" is present
	   */
	  if(f->mbuf)
	  {
		f->mbuf->m_data = src;
		f->mbuf->m_len  = len;
	  }

	  /* suspend HDLC variables */
          f->hdlc.ib        = ib;
          f->hdlc.blevel    = blevel;
          f->hdlc.tmp       = tmp;
          f->hdlc.crc       = crc;

	  /* need to recompute dst_end */
	  dst_end = f->buf_ptr + f->buf_len;

	  /* TX data */
	  f->buf_len = dst - f->buf_ptr;
	  filter_tx(sc,f);

	  if(dst < dst_end)
	  {
	    /* out of data */
	    break;
	  }

	} while(f->Z_chip);
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = transmit,
	.protocol[0] = P_HDLC_EMU,
	.filter      = &tx_hdlc_emulation,
};

#undef HDLC_ENCODE_TYPE
#define HDLC_ENCODE_TYPE(default,dchan) R dchan

/*
 * DO NOT CHANGE THE ROUTINE BELOW,
 * BECAUSE IT IS A TEXT-COPY OF THE
 * ROUTINE ABOVE
 */
static void
tx_hdlc_emulation_dchan FIFO_FILTER_T(sc,f)
{
                 u_int8_t * dst_end;
	         u_int8_t * dst;
	register u_int16_t  blevel;
	register u_int32_t  tmp;
	register u_int16_t  crc;
	register u_int16_t  ib;
	register u_int8_t * src;
	         u_int16_t  len;
	register u_int32_t  tmp2;

	do {
	  /* get buffer */
	  FIFO_GET_MEMORY(sc,f,&dst,&dst_end,&len);
	  f->buf_ptr = dst;
	  f->buf_len = len;

#if 0
	  if (!f->mbuf && IF_QEMPTY(&f->ifqueue) && (f->hdlc.flag == 2))
	  {
	      /* no data to encode */
	      break;
	  }
#endif

	  /* restore HDLC variables */
	  blevel  = f->hdlc.blevel;
	  tmp     = f->hdlc.tmp;
	  crc     = f->hdlc.crc;
	  ib      = f->hdlc.ib;

	  /* restore "src" and "len"
	   * if "f->mbuf" is present
	   */
	  if(f->mbuf)
	  {
		/* resume */
		src = f->mbuf->m_data;
		len = f->mbuf->m_len;

		/* currently do nothing on XDU hence
		 * it is hard to know which frame
		 * was XDU, due to buffering.
		 * if(f->state & ST_FRAME_ERROR) {
		 *	 * XDU * 
		 *	(f->hdlc.flag) = -2;
		 *	          len  =  0;
		 *
		 *	HDLC_ERR("fifo(#%d) XDU.\n", FIFO_NO(f));
		 * }
		 */
	  }
	  else
	  {
		src = 0;
		len = 0;
	  }

	  while(dst != dst_end)
	  {
		HDLC_ENCODE(*src++, len, tmp, tmp2, blevel, ib, crc, (f->hdlc.flag),
		{/* gfr */
			I4B_FREEMBUF(f,f->mbuf);
			f->mbuf = ihfc_i4b_getmbuf(sc, f);

			if(f->mbuf)
			{
				src = f->mbuf->m_data;
				len = f->mbuf->m_len;
			}
			else
			{
				dst_end = dst + 1;

				/* exit after final FS  *
				 * (flag sequence),     *
				 * else the buffer will *
				 * only be filled with  *
				 * "0x7e"-bytes!        *
				 * Usually the hardware *
				 * will repeat the last *
				 * byte.                */
			}
		},
		{/* nmb */
		},
		{/* wrd */
			*dst++ = (u_int8_t)(tmp);
		},
		dd );
	  }

	  /* suspend "f->mbuf->m_len" and "f->mbuf->m_data"
	   * if "f->mbuf" is present
	   */
	  if(f->mbuf)
	  {
		f->mbuf->m_data = src;
		f->mbuf->m_len  = len;
	  }

	  /* suspend HDLC variables */
          f->hdlc.ib        = ib;
          f->hdlc.blevel    = blevel;
          f->hdlc.tmp       = tmp;
          f->hdlc.crc       = crc;

	  /* need to recompute dst_end */
	  dst_end = f->buf_ptr + f->buf_len;

	  /* TX data */
	  f->buf_len = dst - f->buf_ptr;
	  filter_tx(sc,f);

	  if(dst < dst_end)
	  {
	    /* out of data */
	    break;
	  }

	} while(f->Z_chip);
	return;
}

I4B_FILTER_EXPORT()  = {
	.direction   = transmit,
	.protocol[0] = P_HDLC_EMU_D,
	.filter      = &tx_hdlc_emulation_dchan,
};

#endif /* _I4B_FILTER_H_ */
