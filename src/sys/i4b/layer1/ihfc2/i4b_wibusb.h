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
 *	i4b_wibusb.h - W6694x USB support
 *	---------------------------------
 *
 *      last edit-date: []
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_WIBUSB_H_
#define _I4B_WIBUSB_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>
#include <i4b/layer1/ihfc2/i4b_regdata.h>

/*
 * MAXIMUM_TRANSMIT_DRIFT =
 *
 * WIBUSB_TX_ADJUST * (USB_FRAMES_PER_SECOND
 *                      / WIBUSB_TX_FRAMES)
 * = +- 20 bytes / second
 *
 * USB_FRAMES_PER_SECOND = 1000
 *
 * TX/RX delay for B-channel is ~50ms.
 *
 * Average B-channel data length
 * per frame is 8 bytes
 *
 * Average D-channel data length
 * per frame is 2 bytes
 *
 * NOTE: To write a WINBOND USB register the
 * address must be ored with 0x80!
 *
 * NOTE: To read a WINDBOND USB register a
 * dummy value must first be written to the
 * register, and then the next read will read
 * the register. The read-register-value is ignored!
 *
 * The W6694 uses an internal microprocessor.
 *
 * TODO: stop D-channel output on D-channel collision?
 */
#define WIBUSB_RX_FRAMES	50 /* units (total) */
#define WIBUSB_RX_FRAMESIZE	41 /* bytes */

#define WIBUSB_TX_FRAMES	50 /* units (total) */
#define WIBUSB_TX_ADJUST         1 /* units */
#define WIBUSB_TX_FRAMESIZE	23 /* bytes */

#if 0
# define WIBUSB_DEBUG
# define WIBUSB_DEBUG_INTR
# define WIBUSB_DEBUG_ERR
# define WIBUSB_DEBUG_STAT
#else
# define WIBUSB_DEBUG_ERR
#endif

/* imports */

#define w_po1 aux_data_0
#define wibusb_chip_status_check default_chip_status_check
#define wibusb_chip_write regdata_usb_chip_write
#define wibusb_chip_read regdata_usb_chip_read /* BUFFERED! */
#define wibusb_fsm_table isac_fsm_table

/* driver */

static void
wibusb_callback_chip_write USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t             *sc  = xfer->priv_sc;
	struct regdata_usb    *buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

	regdata_usb_update(sc);
	goto done;

 tr_setup:

	/* setup packet for register access (2 bytes):
	 *
	 * WIBUSB is capable of doing 4 register
	 * accesses in one packet. Currently only
	 * one register is written per packet
	 */

	buf->data[0] = buf->current_register; /* register */
	buf->data[1] = buf->current_data; /* data */

	xfer->length = 2; /* bytes */

 tr_error:
	/* [re-]transfer ``xfer->buffer'' */
	usbd_start_hardware(xfer);
 done:
	return;
}

static void
wibusb_callback_chip_read USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t             *sc  = xfer->priv_sc;
	struct regdata_usb    *buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:
	IHFC_MSG("ReadReg:0x%02x, Val:0x%02x\n",
		 buf->data[0], buf->data[1]);

	switch(buf->data[0])
	{
	default:
	  /* unknown */
	  IHFC_ERR("Unknown reg=0x%02x; data=0x%02x\n",
		   buf->data[0], buf->data[1]);
	  break;
	}

	regdata_usb_update(sc);
	goto done;

 tr_setup:
	/* setup data length */
	xfer->length = 2; /* bytes */

 tr_error:
	/* [re-]transfer ``xfer->buffer'' */
	usbd_start_hardware(xfer);
 done:
	return;
}

static void
wibusb_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	register u_int8_t *fifo_ptr = (f->Z_ptr);

	/* pre increment Z-counter (before `len` is changed) */
	(f->Z_ptr) += (len);

	bcopy(fifo_ptr,ptr,len);
	return;
}

static void
wibusb_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	register u_int8_t *fifo_ptr = (f->Z_ptr);

	if(len)
	{
	  /* store the last byte */
	  (f->last_byte) = ptr[len-1];

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_ptr) += (len);

	  bcopy(ptr,fifo_ptr,len);
	}
	return;
}

/*
 * About the hardware:
 * ===================
 * 
 * The WIBUSB-chip receives and sends all FIFO data through the same
 * two pipes, pipe 4 and 5 respectively. There is no separate pipe for
 * each FIFO. Instead each ``USB-frame'' contains a variable length of
 * data for D-, B1- and B2-channel FIFOs.
 *
 *
 * Host receive direction (``USB-frame'' format; wMaxPacketSize==41 bytes):
 * ========================================================================
 *
 * +-------+
 * | ISOE  | // (currently not checked)
 * +-------+
 * | XFIFO | // D - channel
 * +-------+
 * | data  |
 * |  ..   |
 * +-------+
 * | XFIFO | // B1 - channel
 * +-------+
 * | data  |
 * |  ..   |
 * +-------+
 * | XFIFO | // B2 - channel
 * +-------+
 * | data  |
 * |  ..   |
 * +-------+
 *
 * In host-receive direction the FIFO data length is stored in an
 * 8-bit XFIFO status preceding each data block. This XFIFO also
 * contains information about the data flow, XDU, RDO etc.
 *
 * Hence an ``USB-frame'' contains very little data, 0..15 bytes
 * inclusively for B-channel and 0..7 bytes inclusively for D-channel,
 * the driver will wait till 50 frames are received (== 50ms) and then
 * do processing. A part of the processing is sorting the raw data by
 * channel: D-channel data first then B1-channel data and then
 * B2-channel data. This avoids calling the (f->filter) one time for
 * each ``USB-frame'' and channel. The second half of xfer->buffer is
 * used as temporary storage for this purpose, which is not used by
 * USB transfers.
 *
 *
 * Host transmit direction (``USB-frame'' format; wMaxPacketSize==23 bytes):
 * =========================================================================
 *
 * +-------+
 * | XFIFO | // D - channel
 * +-------+
 * |3 bytes|
 * |of data|
 * |  ..   |
 * +-------+
 * | XFIFO | // B1 - channel
 * +-------+
 * | data  | // B1 - channel data
 * |  ..   |
 * +-------+
 * | data  | // B2 - channel data
 * |  ..   |
 * +-------+
 *
 * Transmit frames have about the same structure as receive frames,
 * with the exception that there is only one B - channel XFIFO field
 * giving the data length for B1 - channel, and there is no ISOE
 * field. The remaining bytes of the packet is B2 - channel data. The
 * manual does not say wether different frame lengths for B1- and B2-
 * channel is allowed. To avoid any trouble, B-channel lengths are
 * assumed to be the same. Total packet size will be adjusted
 * according to this. The first 4 bytes of each transmit packet are
 * reserved for D-channel data. Unused bytes should be set to
 * (unsigned)(-1): all bits binary one.
 */

static void
wibusb_callback_isoc_rx USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t *sc = xfer->priv_sc;

	__typeof(xfer->frlengths)
	  frlengths = xfer->frlengths,
	  frlengths_end  =  frlengths + WIBUSB_RX_FRAMES;

	u_int8_t
	  *fifo_ptr = xfer->buffer,
	  *d1_start, *d1_end, d1_stat = 0,
	  *b1_start, *b1_end, b1_stat = 0,
	  *b2_start, *b2_end, b2_stat = 0,
	  *tmp;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

#if WIBUSB_RX_FRAMESIZE < (8+16+16)
#error "WIBUSB_RX_FRAMESIZE too small"
#endif
	                    tmp = fifo_ptr + (WIBUSB_RX_FRAMES*WIBUSB_RX_FRAMESIZE);
	d1_start = d1_end = tmp;
	b1_start = b1_end = tmp + (WIBUSB_RX_FRAMES*(8));
	b2_start = b2_end = tmp + (WIBUSB_RX_FRAMES*(8 + 16));

	for(;
	    frlengths < frlengths_end;
	    frlengths++)
	{
#define break illegal
#define goto illegal

		/* The code below copies some extra bytes,
		 * but that is due to the design of the
		 * hardware:
		 *
		 * B1_LEN and B2_LEN should have been
		 * limited to 12 bytes inclusive and D_LEN
		 * should have been limited to 4 bytes
		 * inclusive!
		 */

		/* get read data pointer */
		tmp = fifo_ptr;

		/* next frame */
		fifo_ptr += WIBUSB_RX_FRAMESIZE;

#ifdef WIBUSB_DEBUG_STAT
		printf(" frlen=%d ", *frlengths);
#endif
		if(*frlengths < 4) continue;

		/*
		 * tmp[0] == Status
		 * Isochronous OUT error (ignored)
		 */

		tmp++;

		/*
		 * D-channel
		 */

		tmp++;

		/* move 8-bytes */
		((u_int32_t *)d1_end)[0] = ((u_int32_t *)tmp)[0];
		((u_int32_t *)d1_end)[1] = ((u_int32_t *)tmp)[1];

#ifdef WIBUSB_DEBUG_STAT
		printf(" d1:0x%02x ", tmp[-1]);
#endif
		/* update data pointers */
		d1_stat |= tmp[-1];
		d1_end  += tmp[-1] & 7;
		tmp     += tmp[-1] & 7;

		/*
		 * B1-channel
		 */

		tmp++;

		/* move 16-bytes */
		((u_int32_t *)b1_end)[0] = ((u_int32_t *)tmp)[0];
		((u_int32_t *)b1_end)[1] = ((u_int32_t *)tmp)[1];
		((u_int32_t *)b1_end)[2] = ((u_int32_t *)tmp)[2];
		((u_int32_t *)b1_end)[3] = ((u_int32_t *)tmp)[3];

#ifdef WIBUSB_DEBUG_STAT
		printf(" b1:0x%02x ", tmp[-1]);
#endif
		/* update data pointers */
		b1_stat |= tmp[-1];
		b1_end  += tmp[-1] & 15;
		tmp     += tmp[-1] & 15;

		/*
		 * B2-channel
		 */

		tmp++;

		/* move 16-bytes */
		((u_int32_t *)b2_end)[0] = ((u_int32_t *)tmp)[0];
		((u_int32_t *)b2_end)[1] = ((u_int32_t *)tmp)[1];
		((u_int32_t *)b2_end)[2] = ((u_int32_t *)tmp)[2];
		((u_int32_t *)b2_end)[3] = ((u_int32_t *)tmp)[3];

#ifdef WIBUSB_DEBUG_STAT
		printf(" b2:0x%02x ", tmp[-1]);
#endif
		/* update data pointers */
		b2_stat |= tmp[-1];
		b2_end  += tmp[-1] & 15;
		tmp     += tmp[-1] & 15;
#undef break
#undef goto
	}

	/* restore frlengths */
	frlengths -= WIBUSB_RX_FRAMES;

	/* restore fifo_ptr */
	fifo_ptr -= WIBUSB_RX_FRAMESIZE*WIBUSB_RX_FRAMES;

	/*
	 * Check for errors (Overflow/Underflow)
	 * 
	 * On XCOL,XDUN,XDOV and RDOV ``the XFIFO
	 * is  reset  and   disabled   for   that
	 * channel'',   according to manual.   To
	 * clear the XFIFO error bits,CMDR_X must
	 * be written with XEN set.
	 */

	sc->sc_config.fifo_level =
	  ((d1_stat & 0x80) >> d1t) |
	  ((b1_stat & 0x80) >> b1t) |
	  ((b2_stat & 0x80) >> b2t);

	/* b1_stat and b2_stat are not always
	 * the same
	 */
	if((b1_stat | b2_stat) & 0x70)
	{
#ifdef WIBUSB_DEBUG_ERR
		/* B1 - channel */
		if(b1_stat & 0x40) {
		  IHFC_ERR("B1 Transmit Data Overflow.\n");
		}

		if(b1_stat & 0x20) {
		  IHFC_ERR("B1 Transmit Data Underflow.\n");
		}

		if(b1_stat & 0x10) {
		  IHFC_ERR("B1 Receive Data Overflow.\n");
		}

		/* B2 - channel */
		if(b2_stat & 0x40) {
		  IHFC_ERR("B2 Transmit Data Overflow.\n");
		}

		if(b2_stat & 0x20) {
		  IHFC_ERR("B2 Transmit Data Underflow.\n");
		}

 		if(b2_stat & 0x10) {
		  IHFC_ERR("B2 Receive Data Overflow.\n");
		}
#endif
		/* due to an unknown reason
		 * resetting all fifos at the same
		 * time does not work
		 */

		if(b1_stat & 0x60) {
		  sc->sc_config.w_cmdr2 |= 0x80;
		}

		if(b1_stat & 0x10) {
		  sc->sc_config.w_cmdr2 |= 0x40;
		}

		if(b2_stat & 0x60) {
		  sc->sc_config.w_cmdr2 |= 0x08;
		}

 		if(b2_stat & 0x10) {
		  sc->sc_config.w_cmdr2 |= 0x04;
		}

		/* write CMDR2(reg=(0x02|0x80), WIBUSB) */
		wibusb_chip_write(sc,(0x02|0x80),
				 &sc->sc_config.w_cmdr2, 1);

		/* disable reset */
		sc->sc_config.w_cmdr2 &= ~0xcc;
	}
	
	if(d1_stat & 0x78)
	{
#ifdef WIBUSB_DEBUG_ERR
		/* D1 - channel */
		if(d1_stat & 0x40) {
		  IHFC_MSG("D1 Transmit Collision.\n");
		}

		if(d1_stat & 0x20) {
		  IHFC_ERR("D1 Transmit Data Overflow.\n");
		}

		if(d1_stat & 0x10) {
		  IHFC_ERR("D1 Transmit Data Underflow.\n");
		}

		if(d1_stat & 0x08) {
		  IHFC_ERR("D1 Receive Data Overflow.\n");
		}
#endif
		/* due to an unknown reason
		 * resetting all fifos at the same
		 * time does not work
		 */
		if(d1_stat & 0x08) {
		  sc->sc_config.w_cmdr1 |= 0x40;
		}

		if(d1_stat & 0x30) {
		  sc->sc_config.w_cmdr1 |= 0x80;
		}

		/* write CMDR1(reg=(0x01|0x80), WIBUSB) */
		wibusb_chip_write(sc,(0x01|0x80),
				 &sc->sc_config.w_cmdr1, 1);

		/* disable reset */
		sc->sc_config.w_cmdr1 &= ~0xc0;
	}

	/*
	 * Setup Z_ptr and Z_chip
	 * for the respective fifos
	 */

	sc->sc_fifo[d1r].Z_ptr  =          d1_start; /* where data starts */
	sc->sc_fifo[d1r].Z_chip = d1_end - d1_start; /* length of data */

	sc->sc_fifo[b1r].Z_ptr  =          b1_start; /* where data starts */
	sc->sc_fifo[b1r].Z_chip = b1_end - b1_start; /* length of data */

	sc->sc_fifo[b2r].Z_ptr  =          b2_start; /* where data starts */
	sc->sc_fifo[b2r].Z_chip = b2_end - b2_start; /* length of data */

	/*
	 * Call filter(,)
	 */

	(sc->sc_fifo[d1r].filter)(sc, &sc->sc_fifo[d1r]);
	(sc->sc_fifo[b1r].filter)(sc, &sc->sc_fifo[b1r]);
	(sc->sc_fifo[b2r].filter)(sc, &sc->sc_fifo[b2r]);

 tr_setup:
	/* setup framelengths and
	 * start USB hardware;
	 * Reuse xfer->buffer and
	 * xfer->nframes
	 *
	 * xfer->nframes == WIBUSB_RX_FRAMES;
	 */

	for(;
	    frlengths < frlengths_end;
	    frlengths++)
	{
	  *frlengths = WIBUSB_RX_FRAMESIZE;
	}

	/* restore frlengths */
	frlengths -= WIBUSB_RX_FRAMES;

 tr_error:
	/* [re-]transfer ``xfer->buffer'' */
	usbd_start_hardware(xfer);
	return;
}

/*
 * NOTE: The current USB driver does not use a fixed frame size. That
 * means all frames are back to back in xfer->buffer!
 * *(xfer->frlengths +x) gives the size of frame ``x''.
 */

static void
wibusb_callback_isoc_tx USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t *sc = xfer->priv_sc;

	u_int8_t
	  *d1_start, d_average,
	  *b1_start, b_average,
	  *b2_start, p_average, x, *tmp;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
 tr_transferred:

#if WIBUSB_TX_FRAMESIZE < (4+(2*9))
#error "WIBUSB_TX_FRAMESIZE too small!"
#endif
	/* get 2nd buffer */
		   tmp = xfer->buffer;
		   tmp += (WIBUSB_TX_FRAMES*WIBUSB_TX_FRAMESIZE);
	d1_start = tmp;
	           tmp += (WIBUSB_TX_FRAMES*4);
	b1_start = tmp;
	           tmp += (WIBUSB_TX_FRAMES*9);
	b2_start = tmp;
	           tmp += (WIBUSB_TX_FRAMES*9);

	/* get MWBA */
	sc->sc_fifo[d1t].Z_ptr = d1_start; /* where data starts */
	sc->sc_fifo[b1t].Z_ptr = b1_start; /* where data starts */
	sc->sc_fifo[b2t].Z_ptr = b2_start; /* where data starts */

	/* setup D-transfer length  */
	if(sc->sc_config.fifo_level & (0x80 >> d1t))
	{
	  /* send more data */
	  sc->sc_fifo[d1t].Z_chip = (2*WIBUSB_TX_FRAMES) + (1*WIBUSB_TX_ADJUST);
	  d_average = (2+1);
	}
	else
	{
	  /* send less data */
	  sc->sc_fifo[d1t].Z_chip = (2*WIBUSB_TX_FRAMES) - (1*WIBUSB_TX_ADJUST);
	  d_average = (2-1);
	}

	/* setup B-transfer length  */
	if((sc->sc_config.fifo_level & (0x80 >> b1t)) &&
	   (sc->sc_config.fifo_level & (0x80 >> b2t)))
	{
	  /* send more data */
	  sc->sc_fifo[b1t].Z_chip = (8*WIBUSB_TX_FRAMES) + (1*WIBUSB_TX_ADJUST);
	  sc->sc_fifo[b2t].Z_chip = (8*WIBUSB_TX_FRAMES) + (1*WIBUSB_TX_ADJUST);
	  b_average = (8+1);
	}
	else
	{
	  /* send less data */
	  sc->sc_fifo[b1t].Z_chip = (8*WIBUSB_TX_FRAMES) - (1*WIBUSB_TX_ADJUST);
	  sc->sc_fifo[b2t].Z_chip = (8*WIBUSB_TX_FRAMES) - (1*WIBUSB_TX_ADJUST);
	  b_average = (8-1);
	}

#ifdef WIBUSB_DEBUG
	printf("%d%d", d_average, b_average);
#endif

	/*
	 * call FIFOs
	 */

	(sc->sc_fifo[d1t].filter)(sc, &sc->sc_fifo[d1t]);
	(sc->sc_fifo[b1t].filter)(sc, &sc->sc_fifo[b1t]);
	(sc->sc_fifo[b2t].filter)(sc, &sc->sc_fifo[b2t]);

	/*
	 * fill unused FIFO space
	 * with the last byte:
	 */

	memset(sc->sc_fifo[d1t].Z_ptr,
	       sc->sc_fifo[d1t].last_byte,
	       sc->sc_fifo[d1t].Z_chip);

	memset(sc->sc_fifo[b1t].Z_ptr,
	       sc->sc_fifo[b1t].last_byte,
	       sc->sc_fifo[b1t].Z_chip);

	memset(sc->sc_fifo[b2t].Z_ptr,
	       sc->sc_fifo[b2t].last_byte,
	       sc->sc_fifo[b2t].Z_chip);
	/*
	 * clear ``Z_chip'', hence the
	 * FIFO cannot be written out-
	 * side this routine:
	 */

	sc->sc_fifo[d1t].Z_chip = 0;
	sc->sc_fifo[b1t].Z_chip = 0;
	sc->sc_fifo[b2t].Z_chip = 0;

	/* get 1st buffer */
	tmp = xfer->buffer;

#if WIBUSB_TX_ADJUST == 0
#error "WIBUSB_TX_ADJUST == 0"
#endif
	/* setup average packet length */
	p_average = (2+3)+(2*b_average);

	/* build ``USB frames'' */
	for(x = 0;
	    x < WIBUSB_TX_FRAMES;
	    x++)
	{
		if(x == WIBUSB_TX_ADJUST)
		{
		  if(d_average < 2)
		     d_average++;
		  else
		     d_average--;

		  if(b_average < 8)
		     b_average++;
		  else
		     b_average--;

		  /* update average packet length */
		  p_average = (2+3)+(2*b_average);
		}

		/* store frame length */
		*(xfer->frlengths + x)     = p_average;

		/* D-channel: copy 4 bytes */
		*((u_int32_t *)(tmp   +1)) = *((u_int32_t *)(d1_start   ));

		/* B1-channel: copy 9 bytes */
  		*((u_int32_t *)(tmp   +5)) = *((u_int32_t *)(b1_start   ));
		*((u_int32_t *)(tmp   +9)) = *((u_int32_t *)(b1_start +4));
		*((u_int8_t  *)(tmp  +13)) = *((u_int8_t  *)(b1_start +8));

		/* store frame length(s) */
		*((u_int8_t  *)(tmp     )) = d_average; /* D-length (0x02) */
		*((u_int8_t  *)(tmp   +4)) = b_average; /* B-length (0x08) */
		               (tmp     ) += b_average; /* adjust tmp */

		/* B2-channel: copy 9 bytes */
 		*((u_int32_t *)(tmp   +5)) = *((u_int32_t *)(b2_start   ));
 		*((u_int32_t *)(tmp   +9)) = *((u_int32_t *)(b2_start +4));
		*((u_int8_t  *)(tmp  +13)) = *((u_int8_t  *)(b2_start +8));

		/* update pointers (byte granularity) */
		               (d1_start) += d_average;
			       (b1_start) += b_average;
			       (b2_start) += b_average;

		/* update tmp */
			       (tmp     ) -= b_average; /* restore tmp */
			       (tmp     ) += p_average; /* */
   	}

 tr_error:
	/* [re-]transfer ``xfer->buffer''
	 *
	 * Reuse xfer->buffer and
	 * xfer->nframes
	 *
	 * xfer->nframes == WIBUSB_TX_FRAMES;
	 */

	usbd_start_hardware(xfer);
	return;
}

typedef struct {
  u_int8_t w_ista;
  u_int8_t w_cir;
  u_int8_t w_pcir;
  u_int8_t w_pdata;
  u_int8_t w_moir;
} __packed wibusb_status_t;

static void
wibusb_callback_interrupt USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t *sc = xfer->priv_sc;
	wibusb_status_t *stat = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

	/* if(stat->w_ista & 0x80)
	 * check for statemachine change
	 */
	if(sc->sc_config.w_cir != stat->w_cir)
	{
		/* update w_cir */
		sc->sc_config.w_cir = stat->w_cir;

		/* update statemachine */
		fsm_update(sc,0);
	}

	/* clear all status bits */
	stat->w_ista = 0x00;

 tr_setup:
	/* setup data length */
	xfer->length = sizeof(wibusb_status_t); /* == 5 */

 tr_error:
	/* [re-]transfer ``xfer->buffer'' */
	usbd_start_hardware(xfer);
	return;
}

static void
wibusb_chip_reset CHIP_RESET_T(sc,error)
{
	u_int8_t tmp;

	/* setup clear stall */
	sc->sc_resources.usb_xfer[0]->clearstall_xfer =
	  sc->sc_resources.usb_xfer[7];
	sc->sc_resources.usb_xfer[1]->clearstall_xfer =
	  sc->sc_resources.usb_xfer[8];
	sc->sc_resources.usb_xfer[2]->clearstall_xfer =
	  sc->sc_resources.usb_xfer[9];

	/*
	 * Start interrupt-pipe before
	 * reset, hence interrupts will
	 * appear during reset
	 * (e.g. statemachine change)
	 */
	usbd_transfer_start(sc->sc_resources.usb_xfer[2]);

	/*
	 * setup some defaults
	 */
	sc->sc_config.w_cir = 0x0f; /* deactivated */

	sc->sc_config.fifo_level =
	  (0x80 >> d1t) | /* XFR */
	  (0x80 >> b1t) | /* XFR */
	  (0x80 >> b2t) ; /* XFR */

	/* perform reset
	 *
	 * according to manual reset bit
	 * must be set alone
	 */

	tmp = 0x08;

	/* write CMDR1(reg=(0x01|0x80), WIBUSB) */
	wibusb_chip_write(sc,(0x01|0x80), &tmp, 1);

	/* assuming 1ms hardware delay
	 * between register writes
	 *
	 * clear reset
	 */
	sc->sc_config.w_cmdr1 &= ~0x08;

	/* write CMDR1(reg=(0x01|0x80), WIBUSB) */
	wibusb_chip_write(sc,(0x01|0x80), &sc->sc_config.w_cmdr1, 1);

	/* trying to force the wibusb
	 * to generate an interrupt:
	 *
	 * NOTE: ``L1 reset command'' does not allow NT to activate,
	 * unless ``internal clock is enabled''.
	 */

	tmp = 0x01; /* L1 reset */

	/* write CIX(reg=(0x04|0x80), WIBUSB) */
	wibusb_chip_write(sc,(0x04|0x80),&tmp,1);

	tmp = 0x00; /* enable clock */

	/* write CIX(reg=(0x04|0x80), WIBUSB) */
	wibusb_chip_write(sc,(0x04|0x80),&tmp,1);
	return;
}

static void
wibusb_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	if((f == CONFIG_WRITE_UPDATE) ||
	   (f == CONFIG_WRITE_RELOAD))
	{
	  /*
	   * If the device is not connected
	   * or the line is powered down,
	   * the USB pipes are stopped.
	   * Else the data-transfer-pipes
	   * are started.
	   */

	  if(GROUP_ANY(sc) && 
	     sc->sc_statemachine.state.active)
	  {
	    /* start pipes (rx) */

	    usbd_transfer_start(sc->sc_resources.usb_xfer[5]);
	    usbd_transfer_start(sc->sc_resources.usb_xfer[6]);

	    if(GROUP_TX(sc))
	    {
		/* start pipes (tx) */

		usbd_transfer_start(sc->sc_resources.usb_xfer[3]);
		usbd_transfer_start(sc->sc_resources.usb_xfer[4]);
	    }
	  }
	  else
	  {
	    usbd_transfer_stop(sc->sc_resources.usb_xfer[3]);
	    usbd_transfer_stop(sc->sc_resources.usb_xfer[4]);
	    usbd_transfer_stop(sc->sc_resources.usb_xfer[5]);
	    usbd_transfer_stop(sc->sc_resources.usb_xfer[6]);
	  }
	}
	else
	{
	  /* nothing to configure for fifo */
	}

	return;
}

static void
wibusb_fsm_read FSM_READ_T(sc,ptr)
{
	/* NOTE: w_cir cannot be read by a
	 * USB register read,   because it
	 * it takes too much time.  Return
	 * the last known value instead:
	 */
	*ptr  = (sc->sc_config.w_cir & 0x0f);

	return;
}

/*
 * NOTE: the wibusb state machine does not have
 * ``IOM-2 mode'' bits in CIX, like the ISAC.
 *
 * NOTE: shifting ISAC CIX command right by 2,
 * will correct for the differences between the
 * ISAC CIX cmdr and the wibusb CIX cmdr. At
 * least for the commands that are used.
 */

static void
wibusb_fsm_write FSM_WRITE_T(sc,ptr)
{
	u_int8_t tmp   = (*ptr) | sc->sc_config.i_cirq;
		 tmp >>= 2;

	/* write CIX(reg=(0x04|0x80), WIBUSB) */
	wibusb_chip_write(sc,(0x04|0x80),&tmp,1);

	return;
}

static ihfc_fifo_program_t *
wibusb_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if(PROT_IS_TRANSPARENT(f->prot))
	{
	  program = &i4b_unknown_program;
	}

	return program;
}

#if 0
static void
wibusb_chip_status_read(ihfc_sc_t *sc)
{
	u_int8_t tmp = 0;
	IHFC_MSG("\n");

	wibusb_chip_write(sc,0x04,&tmp,1);
	wibusb_chip_read(sc,0x04,NULL,1);
	return;
}
#endif

register_list_t
wibusb_register_list[] =
{
  { REG2OFF(w_imask)    , (0x00|0x80) },
  { REG2OFF(w_cmdr1)    , (0x01|0x80) },
  { REG2OFF(w_cmdr2)    , (0x02|0x80) },
  { REG2OFF(w_ctl)      , (0x03|0x80) },
  { REG2OFF(w_gcr)      , (0x06|0x80) },
  { REG2OFF(w_mocr)     , (0x07|0x80) },
  { REG2OFF(w_pie)      , (0x0a|0x80) },
  { REG2OFF(w_po1)      , (0x0b|0x80) },
  { REG2OFF(w_po2)      , (0x0c|0x80) },
  { REG2OFF(w_l1b1rs)   , (0x0e|0x80) },
  { REG2OFF(w_l1b2rs)   , (0x0f|0x80) },
  { REG2OFF(w_usbb1rs)  , (0x10|0x80) },
  { REG2OFF(w_usbb2rs)  , (0x11|0x80) },
  { REG2OFF(w_pcm1rs)   , (0x12|0x80) },
  { REG2OFF(w_pcm2rs)   , (0x13|0x80) },
  { 0, 0 }
};

#if (REGDATA_XFER_WRITE != 0) || (REGDATA_XFER_READ != 1)
#error "(REGDATA_XFER_WRITE != 0) || (REGDATA_XFER_READ != 1)"
#endif

static const struct usbd_config
wibusb_usb[] =
{
  /*
   * data transfer is done
   * using pipes 3,4,5 and 6. The
   * other pipes are used for
   * configuration read and write.
   *
   * total amount of memory used
   * by this setup is 13Kbyte
   */

  [REGDATA_XFER_WRITE] = {
    .type      = UE_BULK,
    .endpoint  = 0x01, /* Bulk Out */
    .direction = UE_DIR_OUT,
    .bufsize   = REGDATA_BUFFER_WRITE_SIZE, /* wMaxPacketSize == 8 bytes */
    .callback  = &wibusb_callback_chip_write,
  },

  [REGDATA_XFER_READ] = {
    .type      = UE_BULK,
    .endpoint  = 0x02, /* Bulk In  */
    .direction = UE_DIR_IN,
    .bufsize   = REGDATA_BUFFER_READ_SIZE, /* wMaxPacketSize == 8 bytes */
    .callback  = &wibusb_callback_chip_read,
  },

  [2] = {
    .type      = UE_INTERRUPT,
    .endpoint  = 0x03, /* Interrupt In */
    .direction = UE_DIR_IN,
    .flags     = USBD_SHORT_XFER_OK,
    .bufsize   = sizeof(wibusb_status_t), /* bytes */
    .callback  = &wibusb_callback_interrupt,
  },

  [3] = {
    .type      = UE_ISOCHRONOUS,
    .endpoint  = 0x04, /* ISOC Out */
    .direction = UE_DIR_OUT,
    .flags     = USBD_SHORT_XFER_OK,
    .frames    = 1*(WIBUSB_TX_FRAMES),
    .bufsize   = 2*(WIBUSB_TX_FRAMES*WIBUSB_TX_FRAMESIZE), /* bytes */
    .callback  = &wibusb_callback_isoc_tx,
  },

  [4] = {
    .type      = UE_ISOCHRONOUS,
    .endpoint  = 0x04, /* ISOC Out */
    .direction = UE_DIR_OUT,
    .flags     = USBD_SHORT_XFER_OK,
    .frames    = 1*(WIBUSB_TX_FRAMES),
    .bufsize   = 2*(WIBUSB_TX_FRAMES*WIBUSB_TX_FRAMESIZE), /* bytes */
    .callback  = &wibusb_callback_isoc_tx,
  },

  [5] = {
    .type      = UE_ISOCHRONOUS,
    .endpoint  = 0x05, /* ISOC In */
    .direction = UE_DIR_IN,
    .flags     = USBD_SHORT_XFER_OK,
    .frames    = 1*(WIBUSB_RX_FRAMES),
    .bufsize   = 2*(WIBUSB_RX_FRAMES*WIBUSB_RX_FRAMESIZE), /* bytes */
    .callback  = &wibusb_callback_isoc_rx,
  },

  [6] = {
    .type      = UE_ISOCHRONOUS,
    .endpoint  = 0x05, /* ISOC In */
    .direction = UE_DIR_IN,
    .flags     = USBD_SHORT_XFER_OK,
    .frames    = 1*(WIBUSB_RX_FRAMES),
    .bufsize   = 2*(WIBUSB_RX_FRAMES*WIBUSB_RX_FRAMESIZE), /* bytes */
    .callback  = &wibusb_callback_isoc_rx,
  },

  [7] = {
    .type      = UE_CONTROL,
    .endpoint  = 0,
    .direction = -1,
    .timeout   = USBD_DEFAULT_TIMEOUT,
    .flags     = 0,
    .bufsize   = sizeof(usb_device_request_t),
    .callback  = &usbd_clearstall_callback,
  },

  [8] = {
    .type      = UE_CONTROL,
    .endpoint  = 0,
    .direction = -1,
    .timeout   = USBD_DEFAULT_TIMEOUT,
    .flags     = 0,
    .bufsize   = sizeof(usb_device_request_t),
    .callback  = &usbd_clearstall_callback,
  },

  [9] = {
    .type      = UE_CONTROL,
    .endpoint  = 0,
    .direction = -1,
    .timeout   = USBD_DEFAULT_TIMEOUT,
    .flags     = 0,
    .bufsize   = sizeof(usb_device_request_t),
    .callback  = &usbd_clearstall_callback,
  },
};

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(wibusb_dbase_root)
{
  I4B_DBASE_ADD(desc                    , "Winbond 128K USB ISDN adapter");

  I4B_DBASE_ADD(c_chip_read             , &wibusb_chip_read);
  I4B_DBASE_ADD(c_chip_write            , &wibusb_chip_write);
  I4B_DBASE_ADD(c_chip_reset            , &wibusb_chip_reset);

  I4B_DBASE_ADD(c_chip_config_write     , &wibusb_chip_config_write);

#if 0
  /* delay 1/2 second */
  I4B_DBASE_ADD(c_chip_status_read      , &wibusb_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check     , &wibusb_chip_status_check);
  I4B_DBASE_ADD(d_interrupt_delay       , hz / 2);
  I4B_DBASE_ADD(o_POLLED_MODE           , 1);
#endif

#if 0
  I4B_DBASE_ADD(c_chip_unselect         , &wibusb_chip_unselect);
#endif

  I4B_DBASE_ADD(c_fsm_read              , &wibusb_fsm_read);
  I4B_DBASE_ADD(c_fsm_write             , &wibusb_fsm_write);
  I4B_DBASE_ADD(d_fsm_table             , &wibusb_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program      , &wibusb_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read             , &wibusb_fifo_read);
  I4B_DBASE_ADD(c_fifo_write            , &wibusb_fifo_write);

  I4B_DBASE_ADD(d_register_list         , &wibusb_register_list[0]);
  I4B_DBASE_ADD(d_channels              , 6);

  I4B_DBASE_ADD(usb                     , &wibusb_usb[0]);
  I4B_DBASE_ADD(usb_length              , (sizeof(wibusb_usb)/sizeof(wibusb_usb[0])));

  I4B_DBASE_ADD(usb_conf_no             , 1); /* bConfigurationValue */
  I4B_DBASE_ADD(usb_iface_no            , 0); /* bInterfaceNumber */
  I4B_DBASE_ADD(usb_alt_iface_no        , 1); /* bAlternateSetting */

  I4B_DBASE_ADD(o_PORTABLE              , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x00,YES) /* non-existent */		\
  m(d1, 0x30,YES)				\
  m(b1, 0x0c,YES)				\
  m(b2, 0x03,YES)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* Generic W6694 USB */
	       .vid           = 0x10466694,
	       );

I4B_USB_DRIVER(/* Asuscom ISDNlink 128K (USB)
		* model: TA-280-ST-W
		*/
	       .vid           = 0x07356694,
	       );
/*
 * cleanup
 */

#undef WIBUSB_RX_FRAMES
#undef WIBUSB_RX_FRAMESIZE

#undef WIBUSB_TX_FRAMES
#undef WIBUSB_TX_ADJUST
#undef WIBUSB_TX_FRAMESIZE

#undef WIBUSB_MEM_BASE

#undef WIBUSB_DEBUG
#undef WIBUSB_DEBUG_INTR
#undef WIBUSB_DEBUG_ERR
#undef WIBUSB_DEBUG_STAT
#undef LED_SCHEME

#endif /* _I4B_WIBUSB_H_ */
