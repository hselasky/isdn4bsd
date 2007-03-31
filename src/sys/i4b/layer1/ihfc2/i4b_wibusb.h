/*-
 * Copyright (c) 2002-2007 Hans Petter Selasky. All rights reserved.
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
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_WIBUSB_H_
#define _I4B_WIBUSB_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

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
 * NOTE: All functions with name
 * starting like "hfcsusb_cfg" can
 * only be called from the config
 * thread!
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

#define WIBUSB_CONF_XFER_WRITE   0
#define WIBUSB_CONF_XFER_READ    1

#if 0
# define WIBUSB_DEBUG
# define WIBUSB_DEBUG_INTR
# define WIBUSB_DEBUG_ERR
# define WIBUSB_DEBUG_STAT
#endif

/* imports */

#define w_po1 aux_data_0
#define wibusb_chip_status_check default_chip_status_check
#define wibusb_fsm_table isac_fsm_table

/* prototypes */

static void
wibusb_cfg_chip_config_write_r(struct ihfc_sc *sc,
			       struct ihfc_config_copy *cc, uint16_t refcount);

/* driver */

static void
wibusb_cfg_write_1(ihfc_sc_t *sc, uint8_t reg, uint8_t data)
{
	sc->sc_reg_temp.reg = reg;
	sc->sc_reg_temp.data = data;

        if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto done;
        }

	IHFC_MSG("0x%02x->0x%02x\n", data, reg);

	usbd_transfer_start(sc->sc_resources.usb_xfer[WIBUSB_CONF_XFER_WRITE]);

	if (msleep(&(sc->sc_reg_temp), sc->sc_mtx_p, 0, "write reg", 0)) {

	}
 done:
	return;
}

#if 0
static uint8_t
wibusb_cfg_read_1(ihfc_sc_t *sc, uint8_t reg)
{
	sc->sc_reg_temp.reg = reg;
	sc->sc_reg_temp.data = 0xff;

        if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto done;
        }

	usbd_transfer_start(sc->sc_resources.usb_xfer[WIBUSB_CONF_XFER_WRITE]);

	if (msleep(&(sc->sc_reg_temp), sc->sc_mtx_p, 0, "write reg", 0)) {

	}

        if (usbd_config_td_is_gone(&(sc->sc_config_td))) {
	    goto done;
        }

	usbd_transfer_start(sc->sc_resources.usb_xfer[WIBUSB_CONF_XFER_READ]);

	if (msleep(&(sc->sc_reg_temp), sc->sc_mtx_p, 0, "read reg", 0)) {

	}
 done:
	return sc->sc_reg_temp.data;
}
#endif

static void
wibusb_callback_chip_write USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t	*sc  = xfer->priv_sc;
	uint8_t		*buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:
 tr_error:
	wakeup(&(sc->sc_reg_temp));
	return;

 tr_setup:
	/* setup packet for register access (2 bytes):
	 *
	 * WIBUSB is capable of doing 4 register
	 * accesses in one packet. Currently only
	 * one register is written per packet
	 */
	buf[0] = sc->sc_reg_temp.reg; /* register */
	buf[1] = sc->sc_reg_temp.data; /* data */

	xfer->length = 2; /* bytes */

	usbd_start_hardware(xfer);
	return;
}

static void
wibusb_callback_chip_read USBD_CALLBACK_T(xfer)
{
	ihfc_sc_t	*sc  = xfer->priv_sc;
	uint8_t		*buf = xfer->buffer;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:

	IHFC_MSG("ReadReg:0x%02x, Val:0x%02x\n",
		 buf[0], buf[1]);

	sc->sc_reg_temp.data = buf[1];

 tr_error:
	wakeup(&(sc->sc_reg_temp));
	return;

 tr_setup:
	/* setup data length */
	xfer->length = 2; /* bytes */

	usbd_start_hardware(xfer);

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
	  *tmp, status;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
		return;
	}
	goto tr_setup;

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

		status = tmp[0];

		tmp++;

		/* move 8-bytes */
		((u_int32_p_t *)(d1_end+0))->data = ((u_int32_p_t *)(tmp+0))->data;
		((u_int32_p_t *)(d1_end+4))->data = ((u_int32_p_t *)(tmp+4))->data;

#ifdef WIBUSB_DEBUG_STAT
		printf(" d1:0x%02x ", status);
#endif
		/* update data pointers */
		d1_stat |= status;
		d1_end  += status & 7;
		tmp     += status & 7;

		/*
		 * B1-channel
		 */

		status = tmp[0];

		tmp++;

		/* move 16-bytes */
		((u_int32_p_t *)(b1_end +0 ))->data = ((u_int32_p_t *)(tmp +0 ))->data;
		((u_int32_p_t *)(b1_end +4 ))->data = ((u_int32_p_t *)(tmp +4 ))->data;
		((u_int32_p_t *)(b1_end +8 ))->data = ((u_int32_p_t *)(tmp +8 ))->data;
		((u_int32_p_t *)(b1_end +12))->data = ((u_int32_p_t *)(tmp +12))->data;

#ifdef WIBUSB_DEBUG_STAT
		printf(" b1:0x%02x ", status);
#endif
		/* update data pointers */
		b1_stat |= status;
		b1_end  += status & 15;
		tmp     += status & 15;

		/*
		 * B2-channel
		 */

		status = tmp[0];

		tmp++;

		/* move 16-bytes */
		((u_int32_p_t *)(b2_end +0 ))->data = ((u_int32_p_t *)(tmp +0 ))->data;
		((u_int32_p_t *)(b2_end +4 ))->data = ((u_int32_p_t *)(tmp +4 ))->data;
		((u_int32_p_t *)(b2_end +8 ))->data = ((u_int32_p_t *)(tmp +8 ))->data;
		((u_int32_p_t *)(b2_end +12))->data = ((u_int32_p_t *)(tmp +12))->data;

#ifdef WIBUSB_DEBUG_STAT
		printf(" b2:0x%02x ", status);
#endif
		/* update data pointers */
		b2_stat |= status;
		b2_end  += status & 15;
		tmp     += status & 15;
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

		usbd_config_td_queue_command
		  (&(sc->sc_config_td), NULL, 
		   &wibusb_cfg_chip_config_write_r, 0, 0);
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

		usbd_config_td_queue_command
		  (&(sc->sc_config_td), NULL,
		   &wibusb_cfg_chip_config_write_r, 0, 0);
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

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
		return;
	}

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

	memset_1(sc->sc_fifo[d1t].Z_ptr,
		 sc->sc_fifo[d1t].last_byte,
		 sc->sc_fifo[d1t].Z_chip);

	memset_1(sc->sc_fifo[b1t].Z_ptr,
		 sc->sc_fifo[b1t].last_byte,
		 sc->sc_fifo[b1t].Z_chip);

	memset_1(sc->sc_fifo[b2t].Z_ptr,
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
		((u_int32_p_t *)(tmp +1))->data = ((u_int32_p_t *)(d1_start))->data;

		/* B1-channel: copy 9 bytes */
  		((u_int32_p_t *)(tmp +5))->data = ((u_int32_p_t *)(b1_start   ))->data;
		((u_int32_p_t *)(tmp +9))->data = ((u_int32_p_t *)(b1_start +4))->data;
		               *(tmp +13) = *(b1_start +8);

		/* store frame length(s) */
		*(tmp   ) = d_average; /* D-length (0x02) */
		*(tmp +4) = b_average; /* B-length (0x08) */
		  tmp    += b_average; /* adjust tmp */

		/* B2-channel: copy 9 bytes */
		((u_int32_p_t *)(tmp +5))->data = ((u_int32_p_t *)(b2_start   ))->data;
 		((u_int32_p_t *)(tmp +9))->data = ((u_int32_p_t *)(b2_start +4))->data;
		               *(tmp +13) = *(b2_start +8);

		/* update pointers (byte granularity) */
		               (d1_start) += d_average;
			       (b1_start) += b_average;
			       (b2_start) += b_average;

		/* update tmp */
			       (tmp     ) -= b_average; /* restore tmp */
			       (tmp     ) += p_average; /* */
   	}

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

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    return;
	}
	goto tr_setup;

 tr_transferred:

	/* if(stat->w_ista & 0x80)
	 * check for statemachine change
	 */

	/* update w_cir */
	sc->sc_config.w_cir = stat->w_cir;

	/* update statemachine */
	ihfc_fsm_update(sc,&sc->sc_fifo[0],0);

	/* clear all status bits */
	stat->w_ista = 0x00;

 tr_setup:
	/* setup data length */
	xfer->length = sizeof(wibusb_status_t); /* == 5 */

	usbd_start_hardware(xfer);
	return;
}

static void
wibusb_cfg_reset(struct ihfc_sc *sc,
		 struct ihfc_config_copy *cc, uint16_t refcount)
{
	/* perform reset
	 *
	 * according to the datasheet the reset bit
	 * must be set alone
	 */

	/* write CMDR1 */
	wibusb_cfg_write_1(sc, (0x01|0x80), 0x08);

	/* assuming 1ms hardware delay
	 * between register writes
	 *
	 * clear reset
	 */
	sc->sc_config.w_cmdr1 &= ~0x08;

	/* write CMDR1 */
	wibusb_cfg_write_1(sc, (0x01|0x80), sc->sc_config.w_cmdr1);

	/* trying to force the wibusb
	 * to generate an interrupt:
	 *
	 * NOTE: ``L1 reset command'' does not allow NT to activate,
	 * unless ``internal clock is enabled''.
	 */

	/* write CIX - L1 reset */
	wibusb_cfg_write_1(sc,(0x04|0x80), 0x01);

	/* write CIX - enable clock */
	wibusb_cfg_write_1(sc, (0x04|0x80), 0x00);

	return;
}

static void
wibusb_chip_reset CHIP_RESET_T(sc,error)
{
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
	sc->sc_config.w_cir = 0x00; /* deactivate request */

	sc->sc_config.fifo_level =
	  (0x80 >> d1t) | /* XFR */
	  (0x80 >> b1t) | /* XFR */
	  (0x80 >> b2t) ; /* XFR */

	usbd_config_td_queue_command
	  (&(sc->sc_config_td), NULL,
	   &wibusb_cfg_reset, 0, 0);

	return;
}

static void
wibusb_cfg_chip_config_write_r(struct ihfc_sc *sc,
			       struct ihfc_config_copy *cc, uint16_t refcount)
{
	register_list_t *r;

	/*
	 * configure chip,
	 * write new configuration
	 */
	REGISTER_FOREACH(r,sc->sc_default.d_register_list) {
	    uint8_t *data  = &OFF2REG(sc->sc_config, r->offset);
	    uint8_t *data2 = &OFF2REG(sc->sc_config2, r->offset);
	    uint8_t temp;

	    if (refcount || (*data != *data2)) {

	        /* must update the shadow config before writing
		 * the register, hence "wibusb_cfg_write_1()" will
		 * sleep, and then the config value can change:
		 */

	        temp = *data;

		if (r->offset == REG2OFF(w_cmdr1)) {
		    /* clear reset bits */
		    *data &= ~0xC0;
		} else if (r->offset == REG2OFF(w_cmdr2)) {
		    /* clear reset bits */
		    *data &= ~0xCC;
		}

		*data2 = *data;

		/* write the register */

		wibusb_cfg_write_1(sc, r->regval, temp);
	    }
	}
	return;
}

static void
wibusb_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	if ((f == IHFC_CONFIG_WRITE_UPDATE) ||
	    (f == IHFC_CONFIG_WRITE_RELOAD))
	{
	  usbd_config_td_queue_command
	    (&(sc->sc_config_td), NULL, 
	     &wibusb_cfg_chip_config_write_r, 0, 
	     (f == IHFC_CONFIG_WRITE_RELOAD) ? 1 : 0);

	  /*
	   * If the device is not connected
	   * or the line is powered down,
	   * the USB pipes are stopped.
	   * Else the data-transfer-pipes
	   * are started.
	   */

	  if(GROUP_ANY(sc))
	  {
	    /* start pipes (rx) */

	    usbd_transfer_start(sc->sc_resources.usb_xfer[5]);
	    usbd_transfer_start(sc->sc_resources.usb_xfer[6]);

	    if(GROUP_TX(sc) &&
	       sc->sc_state[0].state.active)
	    {
		/* start pipes (tx) */

		usbd_transfer_start(sc->sc_resources.usb_xfer[3]);
		usbd_transfer_start(sc->sc_resources.usb_xfer[4]);
	    }
	    else
	    {
	        usbd_transfer_stop(sc->sc_resources.usb_xfer[3]);
		usbd_transfer_stop(sc->sc_resources.usb_xfer[4]);
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
	  /* nothing to configure for FIFO */
	}

	return;
}

static void
wibusb_fsm_read FSM_READ_T(sc,f,ptr)
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
wibusb_cfg_fsm_write(struct ihfc_sc *sc,
		     struct ihfc_config_copy *cc, uint16_t refcount)
{
	/* write (CIX, WIBUSB) */
	wibusb_cfg_write_1(sc, (0x04|0x80), refcount);

	return;
}

static void
wibusb_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	u_int8_t tmp   = (*ptr) | sc->sc_config.i_cirq;
		 tmp >>= 2;

	usbd_config_td_queue_command
          (&(sc->sc_config_td), NULL, 
	   &wibusb_cfg_fsm_write, 0, tmp);
	return;
}

static ihfc_fifo_program_t *
wibusb_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if(PROT_IS_TRANSPARENT(&(f->prot_curr)))
	{
	  program = &i4b_unknown_program;
	}

	return program;
}

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

  [WIBUSB_CONF_XFER_WRITE] = {
    .type      = UE_BULK,
    .endpoint  = 0x01, /* Bulk Out */
    .direction = UE_DIR_OUT,
    .bufsize   = (4*2), /* bytes */
    .callback  = &wibusb_callback_chip_write,
    .flags     = 0,
  },

  [WIBUSB_CONF_XFER_READ] = {
    .type      = UE_BULK,
    .endpoint  = 0x02, /* Bulk In  */
    .direction = UE_DIR_IN,
    .bufsize   = (4*2), /* bytes */
    .callback  = &wibusb_callback_chip_read,
    .flags     = 0,
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

  I4B_DBASE_ADD(c_chip_reset            , &wibusb_chip_reset);

  I4B_DBASE_ADD(c_chip_config_write     , &wibusb_chip_config_write);

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

  I4B_DBASE_ADD(i4b_option_mask         , (I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value        , 0);

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

#undef WIBUSB_CONF_XFER_WRITE
#undef WIBUSB_CONF_XFER_READ

#undef WIBUSB_MEM_BASE

#undef WIBUSB_DEBUG
#undef WIBUSB_DEBUG_INTR
#undef WIBUSB_DEBUG_ERR
#undef WIBUSB_DEBUG_STAT
#undef LED_SCHEME

#endif /* _I4B_WIBUSB_H_ */
