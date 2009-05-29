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
 *	i4b_hfcsusb.h - HFC-S USB driver module
 * 	---------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFCSUSB_H_
#define _I4B_HFCSUSB_H_

#include <i4b/layer1/ihfc3/i4b_hfc.h>

/*
 * MAXIMUM_TRANSMIT_DRIFT =
 *
 * HFCSUSB_TX_ADJUST * (USB_FRAMES_PER_SECOND
 *                      / HFCSUSB_TX_FRAMES)
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
 * NOTE: D-channel [extended] trans-
 * parent mode does not provide re-
 * peating of frames, and only 1/4 of
 * the buffer can be used, so hardware
 * HDLC is used for D-channel trans-
 * mission, though this might change in
 * the future?
 *
 * NOTE: After a host-processor failure
 * there is a chance that the chip can
 * block some ISDN channels. The chip
 * does not have ``reset on timeout''?
 *
 * NOTE: D-channel only repeat the last
 * two bits in [extended] transparent
 * mode.
 *
 * NOTE: D-channel transmit: Z_chip is
 * also used to store the FIFO-usage!
 *
 * NOTE: All functions with name
 * starting like "hfcsusb2_cfg" can
 * only be called from the config
 * thread!
 *
 * TODO: speedup F_USAGE reads?
 */

#define HFCSUSB_FIFOSIZE	  128 /* bytes */

#define HFCSUSB_RX_FRAMES          25 /* units (total) */
#define HFCSUSB_RX_FRAMESIZE       12 /* bytes */
#define HFCSUSB_RX_BUFSIZE (HFCSUSB_RX_FRAMESIZE * HFCSUSB_RX_FRAMES)

#define HFCSUSB_TX_FRAMES          25 /* units (total) */
#define HFCSUSB_TX_ADJUST           1 /* units */
#define HFCSUSB_TX_FRAMESIZE       12 /* bytes */
#define HFCSUSB_TX_BUFSIZE (HFCSUSB_TX_FRAMESIZE * HFCSUSB_TX_FRAMES)

#define HFCSUSB_CONF_XFER_WRITE     0
#define HFCSUSB_CONF_XFER_READ      1
#define HFCSUSB_FIFO_XFER_START    (2)       /* offset (inclusive) */
#define HFCSUSB_FIFO_XFER_DCHAN    (2)       /* offset (inclusive)
					      * for D-channel transmit
					      */
#define HFCSUSB_FIFO_XFER_END      (2*(6+1)) /* offset (exclusive) */

#if HFCSUSB_TX_FRAMESIZE != HFCSUSB_RX_FRAMESIZE
#error "FRAMESIZES are not equal!"
#endif

#if 0
# define HFCSUSB_DEBUG
# define HFCSUSB_DEBUG_INTR
# define HFCSUSB_DEBUG_STAT
# define HFCSUSB_DEBUG_ERR
#endif

/* imports */

#define s_ldata aux_data_0
#define hfcsusb2_chip_status_check default_chip_status_check
#define hfcsusb2_fsm_table hfc_fsm_table

static void
hfcsusb2_cfg_write_1(ihfc_sc_t *sc, uint8_t reg, uint8_t data)
{
	sc->sc_reg_temp.reg = reg;
	sc->sc_reg_temp.data = data;

        if (usb2_config_td_is_gone(&(sc->sc_config_td))) {
	    goto done;
        }

	IHFC_MSG("0x%02x->0x%02x\n", data, reg);

	usb2_transfer_start(sc->sc_resources.usb_xfer[HFCSUSB_CONF_XFER_WRITE]);

	if (msleep(&(sc->sc_reg_temp), sc->sc_mtx_p, 0, "write reg", 0)) {

	}
 done:
	return;
}

static uint8_t
hfcsusb2_cfg_read_1(ihfc_sc_t *sc, uint8_t reg)
{
	sc->sc_reg_temp.reg = reg;
	sc->sc_reg_temp.data = 0xff;

        if (usb2_config_td_is_gone(&(sc->sc_config_td))) {
	    goto done;
        }

	usb2_transfer_start(sc->sc_resources.usb_xfer[HFCSUSB_CONF_XFER_READ]);

	if (msleep(&(sc->sc_reg_temp), sc->sc_mtx_p, 0, "read reg", 0)) {

	}
 done:
	return sc->sc_reg_temp.data;
}

/* driver */

static void
hfcsusb2_callback_chip_read USB_CALLBACK_T(xfer)
{
	ihfc_sc_t              *sc  = xfer->priv_sc;
	struct usb_device_request   req;
	uint8_t		       buf[1];

  switch (USB_GET_STATE(xfer)) {
  case USB_ST_TRANSFERRED: 

	usb2_copy_out(xfer->frbuffers + 1, 0, buf, sizeof(buf));

	IHFC_MSG("ReadReg:0x%02x, Val:0x%02x\n",
		 sc->sc_reg_temp.reg, buf[0]);

	sc->sc_reg_temp.data = buf[0];

	goto tr_error;
  case USB_ST_SETUP: 

	/* setup request (8-bytes) */
	req.bmRequestType = 0xC0; /* read data */
	req.bRequest      = 0x01; /* register read access HFC_REG_RD */
	req.wValue[0]     = 0x00; /* data (low byte, ignored) */
	req.wValue[1]     = 0x00; /* data (high byte, ignored) */
	req.wIndex[0]     = sc->sc_reg_temp.reg; /* index (low byte) */
	req.wIndex[1]     = 0x00; /* index (high byte, ignored) */
	req.wLength[0]    = 0x01; /* length (0x0001 for read) */
	req.wLength[1]    = 0x00; /* length (0x0001 for read) */

	/* setup data lengths */
	xfer->frlengths[0] = sizeof(req);
	xfer->frlengths[1] = 1;

	usb2_copy_in(xfer->frbuffers + 0, 0, &req, sizeof(req));

	/* start hardware */
	usb2_start_hardware(xfer);

	return;

  default: tr_error:
	wakeup(&(sc->sc_reg_temp));
	return;
  }
}

static void
hfcsusb2_callback_chip_write USB_CALLBACK_T(xfer)
{
	ihfc_sc_t             *sc  = xfer->priv_sc;
	struct usb_device_request  req;

  switch (USB_GET_STATE(xfer)) {
  case USB_ST_TRANSFERRED: 
	goto tr_error;

  case USB_ST_SETUP: 
	/* setup request (8-bytes) */
	req.bmRequestType = 0x40; /* write data */
	req.bRequest      = 0x00; /* register write access HFC_REG_WR */
	req.wValue[0]     = sc->sc_reg_temp.data; /* data (low byte) */
	req.wValue[1]     = 0x00; /* data (high byte, ignored) */
	req.wIndex[0]     = sc->sc_reg_temp.reg; /* index (low byte) */
	req.wIndex[1]     = 0x00; /* index (high byte, ignored) */
	req.wLength[0]    = 0x00; /* length (0x0000 for write) */
	req.wLength[1]    = 0x00; /* length (0x0000 for write) */

	/* setup data length */
	xfer->frlengths[0] = sizeof(req);
	xfer->frlengths[1] = 0;

	usb2_copy_in(xfer->frbuffers + 0, 0, &req, sizeof(req));

	/* start hardware */
	usb2_start_hardware(xfer);

	return;

  default: tr_error:
	wakeup(&(sc->sc_reg_temp));
	return;
  }
}

static void
hfcsusb2_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	register u_int8_t *fifo_ptr = (f->Z_ptr);

	/* pre increment Z-counter (before `len` is changed) */
	(f->Z_ptr) += (len);

	bcopy(fifo_ptr,ptr,len);
	return;
}

static void
hfcsusb2_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
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

static void
hfcsusb2_cfg_reset(struct ihfc_sc *sc,
		  struct ihfc_config_copy *cc, uint16_t refcount)
{
	/* CIRM_USB: AUX latch enable: data out is
	 * valid until next AUX write
	 *
	 * NOTE: setting FIFO threshold higher
	 * than 64 bytes, does not work ?
	 */

	static const struct regdata regdata[] =
	{
	  { .reg = 0x00, .data = 0x18 }, /* CIRM_USB - reset enable */
	  { .reg = 0x00, .data = 0x10 }, /* CIRM_USB - reset disable */

	  { .reg = 0x06, .data = HFCSUSB_RX_FRAMESIZE }, /* USB_SIZE_I */
	  { .reg = 0x07, .data = 0x11 }, /* USB_SIZE (bulk) */
	  { .reg = 0x0B, .data = 0x00 }, /* F_CROSS */
	  { .reg = 0x0C, .data = 0x44 }, /* F_THRES - 32 bytes */
	  { .reg = 0x0D, .data = 0x00 }, /* F_MODE */
	  { .reg = 0x2C, .data = 0xff }, /* send 1's only */
	  { .reg = 0x2D, .data = 0xff }, /* send 1's only */
	  { .reg = 0xff, .data = 0xff }, /* END */
	};

	const struct regdata *ptr;

	/* reset and reload configuration */
	for(ptr = regdata;
	    ptr->reg != 0xff;
	    ptr++)
	{
	    /* assuming 1ms hardware delay
	     * between register writes
	     *
	     * write register
	     */
	    hfcsusb2_cfg_write_1(sc, ptr->reg, ptr->data);
	}
	return;
}

static void
hfcsusb2_chip_reset CHIP_RESET_T(sc,error)
{
	/*
	 * setup fifo pointers
	 */
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x0]->priv_fifo 
	  = &sc->sc_fifo[d1t];
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x1]->priv_fifo 
	  = &sc->sc_fifo[d1t]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x2]->priv_fifo 
	  = &sc->sc_fifo[d1r]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x3]->priv_fifo 
	  = &sc->sc_fifo[d1r]; 

	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x4]->priv_fifo 
	  = &sc->sc_fifo[b1t]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x5]->priv_fifo 
	  = &sc->sc_fifo[b1t]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x6]->priv_fifo 
	  = &sc->sc_fifo[b1r]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x7]->priv_fifo 
	  = &sc->sc_fifo[b1r]; 

	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x8]->priv_fifo 
	  = &sc->sc_fifo[b2t]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0x9]->priv_fifo 
	  = &sc->sc_fifo[b2t]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0xA]->priv_fifo 
	  = &sc->sc_fifo[b2r]; 
	sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START+0xB]->priv_fifo 
	  = &sc->sc_fifo[b2r]; 

	/*
	 * setup some defaults
	 */
	sc->sc_config.s_states = 0x00; /* F0 - reset */

	/*
	 * the bits set below must always
	 * be set for D-channel(!)
	 */
	sc->sc_fifo[d1t].s_con_hdlc |= 1; /* all 1's inter frame fill */
	sc->sc_fifo[d1r].s_con_hdlc |= 1; /* all 1's inter frame fill */

	sc->sc_fifo[d1t].s_par_hdlc = 2; /* 2-bits per millisecond */
	sc->sc_fifo[d1r].s_par_hdlc = 2; /* 2-bits per millisecond */

	usb2_config_td_queue_command
	  (&(sc->sc_config_td), NULL,
	   &hfcsusb2_cfg_reset, 0, 0);
	return;
}

static void
hfcsusb2_chip_config_write CHIP_CONFIG_WRITE_T(sc,f);

static void
hfcsusb2_cfg_read_f_usage(struct ihfc_sc *sc,
			 struct ihfc_config_copy *cc, uint16_t refcount)
{
	ihfc_fifo_t *f;
	uint8_t temp;

	f = sc->sc_fifo + d1t;

	/* check if callback is ready */

	if ((f->Z_chip & 0x81) == 0x81) {

	    /* write FIFO_SEL */
	    hfcsusb2_cfg_write_1(sc, 0x0F, f->s_fifo_sel);

	    /* read F_USAGE */
	    temp = hfcsusb2_cfg_read_1(sc, 0x1A);

	    /* check if callback is still ready */

	    if ((f->Z_chip & 0x81) == 0x81) {

	        /* update Z_chip */
	        f->Z_chip = temp & ~0x81;

		/* re-start any stopped pipes,
		 * hence the D-channel transmit
		 * pipe is allowed to be stopped, to
		 * reduce CPU usage:
		 */
		hfcsusb2_chip_config_write(sc, IHFC_CONFIG_WRITE_UPDATE);
	    }
	}
	return;
}

static void
hfcsusb2_callback_isoc_tx_d_hdlc USB_CALLBACK_T(xfer)
{
	ihfc_sc_t  *sc = xfer->priv_sc;
	ihfc_fifo_t *f = xfer->priv_fifo;
#define FRAME_NUMBER    25 /* frames */
#define FRAME_SIZE    0x08 /*bytes*/

	/* get pointers to store frame lengths */
	uint32_t *frlengths = xfer->frlengths;
	uint32_t *frlengths_end = frlengths + FRAME_NUMBER;

	u_int8_t 
	  *d1_start, *tmp, len, average = FRAME_SIZE,
	  control_byte;

#if (FRAME_NUMBER > HFCSUSB_TX_FRAMES) || (FRAME_SIZE > HFCSUSB_TX_FRAMESIZE)
#error "(FRAME_NUMBER > HFCSUSB_TX_FRAMES) || (FRAME_SIZE > HFCSUSB_TX_FRAMESIZE)"
#endif

  switch (USB_GET_STATE(xfer)) {
  case USB_ST_TRANSFERRED: tr_transferred:
  case USB_ST_SETUP: 

	if(xfer != sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_DCHAN])
	{
		/* wrong xfer */
		goto done;
	}

	/* get start of 2nd buffer */
	d1_start = sc->sc_temp_ptr;
	d1_start += HFCSUSB_TX_BUFSIZE;

	/* need to re-read F_USAGE */
	if(f->Z_chip & 0x80)
	{
	  if(!(f->Z_chip & 0x01)) {
	      f->Z_chip |= 0x01;

	      usb2_config_td_queue_command
		(&(sc->sc_config_td), NULL, 
		 &hfcsusb2_cfg_read_f_usage, 0, 0);
	  }

	  /* waiting for F_USAGE read */
	  goto done;
	}

	if((f->state & ST_FRAME_END) && (f->Z_chip != 0))
	{
	  /* waiting for F-counter increment */
	  control_byte = 0x00; /* default value */
	  len = 0;
	}
	else
	{
	  /* clear ST_FRAME_ERROR and ST_FRAME_END */
	  f->state &= ~(ST_FRAME_ERROR|ST_FRAME_END);

	  /* get start of 2nd buffer */
	  f->Z_ptr = d1_start;

	  if(f->Z_chip == 0)
	  {
	    /* FIFO XDU
	     * assume that FLAG has been sent ?
	     */
	    f->state |= ST_FRAME_ERROR;
	  }

	  /* get maximum FIFO transfer
	   * length minus two
	   */
	  f->Z_chip ^= 0x7E;

	  /*
	   * Call filter(,)
	   */

	  (f->filter)(sc, f);

	  /* set control byte and 
	   * get transfer length
	   */
	  control_byte = 0x01; /* end of frame */
	  len = f->Z_ptr - d1_start;
	}

	/* need to re-read F_USAGE */
	f->Z_chip = 0x80;

	if((len == 0) && !(f->state & ST_FRAME_END))
	{
	  /* out of data:
	   * need a "fifo_call()" to re-start xfer, that
	   * again will call "hfcsusb2_d1t_program()"
	   */
	  f->state &= ~ST_RUNNING;
	  goto done;
	}

	/*
	 * Prepare data for USB
	 *
	 * get start of 1st buffer
	 * (output pointer)
	 */
	tmp = sc->sc_temp_ptr;

	/* build ``USB frames'' */
	for(;
	    frlengths < frlengths_end;
	    frlengths++)
	{
	  if(len > average)
	  {
	    len -= average;
	  }
	  else
	  {
	    average = len;
	    len = 0;
	  }

	  /* FIFO Control Byte */
	  *(tmp) = 0x00; /* default value */

	  /* copy 8-bytes      */
	  ((u_int32_p_t *)(tmp +1))->data = ((u_int32_p_t *)(d1_start   ))->data;
	  ((u_int32_p_t *)(tmp +5))->data = ((u_int32_p_t *)(d1_start +4))->data;

	  /* update d1_start   */
	  d1_start              += average;

	  /* update tmp and store total frame length */
	  tmp                   += (*frlengths = (average +1));
	}

	if(f->state & ST_FRAME_END)
	{
	  /* XXX must subtract first
	   * XXX compiler issue
	   */
	  tmp -= (average +1);

	  /* FIFO Control Byte
	   * increment F_drvr-counter to
	   * avoid irregular CRC-byte
	   * insertion:
	   */
	  *(tmp) = control_byte; /**/

	  /* restore "tmp" pointer */
	  tmp += (average +1);
	}

	usb2_copy_in(xfer->frbuffers + 0, 0, sc->sc_temp_ptr,
		     tmp - ((uint8_t *)(sc->sc_temp_ptr)));

	xfer->nframes = FRAME_NUMBER;

	usb2_start_hardware(xfer);

 done:
	return;

#undef FRAME_SIZE
#undef FRAME_NUMBER

  default: /* Error */
	if (xfer->error == USB_ERR_CANCELLED) {
		return;
	}
	goto tr_transferred;
  }
}

/*
 * About the hardware:
 * ===================
 *
 * FIFO-data is transferred using ``data transfers'' which can have
 * any length.
 *
 * A ``data transfer'' starts when the ``USB frame length'' is non-zero.
 * A ``data transfer'' ends   when the ``USB frame length'' is less than
 * ``wMaxPacketSize'' and includes the data in that frame.
 *
 * One ``data transfer'' may start and end in the same ``USB frame''.
 *
 *
 * Host-transmit direction (``data transfer'' format):
 * ===================================================
 *
 * +-------+
 * | CNTL  | // FIFO Control Byte 
 * +-------+
 * | data  | // FIFO Data
 * |       |
 * |  ..   |
 * +-------+
 *
 *
 * Host-receive direction (``data transfer'' format):
 * ==================================================
 *
 * +-------+
 * | CNTL1 | // FIFO Control Byte1
 * +-------+
 * | CNTL2 | // FIFO Control Byte2
 * +-------+
 * | data  | // FIFO Data
 * |  .    |
 * |  ..   |
 * |  ...  |
 * +-------+
 *
 * D-channel note ([extended] transparent mode only)
 * =================================================
 *
 * The D-channel FIFO has a data-rate of 8Kbyte per second, like the
 * other FIFOs, but only two bits of each FIFO-byte are used. In host-
 * receive direction 2-bits are inserted at the MSB of each byte and
 * shifted down towards LSB:
 *
 * MSB                           LSB
 * +---+---+---+---+---+---+---+---+
 * | X | X | . | . | . | . | . | . | FIFO BYTE +0
 * +---+---+---+---+---+---+---+---+
 * +---+---+---+---+---+---+---+---+
 * | . | . | X | X | . | . | . | . | FIFO BYTE +1
 * +---+---+---+---+---+---+---+---+
 * +---+---+---+---+---+---+---+---+
 * | Z | Z | . | . | X | X | . | . | FIFO BYTE +2
 * +---+---+---+---+---+---+---+---+
 * +---+---+---+---+---+---+---+---+
 * | . | . | Z | Z | . | . | X | X | FIFO BYTE +3
 * +---+---+---+---+---+---+---+---+
 *
 * The driver simply keeps every 4th byte,
 * to get the D-channel data.
 */

static void
hfcsusb2_callback_isoc_tx USB_CALLBACK_T(xfer)
{
	ihfc_sc_t  *sc = xfer->priv_sc;
	ihfc_fifo_t *f = xfer->priv_fifo;

	uint16_t
	  Z_chip_written;

	u_int8_t 
	  *d1_start, *d1_end, *tmp, fifo_level,
	  average = 8; /* default */

	/* get pointers to store frame lengths */
	
	uint32_t *frlengths = xfer->frlengths;
	uint32_t *frlengths_end = frlengths + (HFCSUSB_TX_FRAMES);
	uint32_t *frlengths_adj = frlengths + (HFCSUSB_TX_ADJUST);

	if(PROT_IS_HDLC(&(f->prot_curr)))
	{
	  hfcsusb2_callback_isoc_tx_d_hdlc(xfer);
	  goto done;
	}

  switch (USB_GET_STATE(xfer)) {
  case USB_ST_SETUP: tr_setup:
  case USB_ST_TRANSFERRED: 

	/* get FIFO fill level (cleared elsewhere) */
	fifo_level = sc->sc_config.fifo_level & (1 << (f->s_fifo_sel));

	/* increase frame counter */
	f->F_drvr++;

	/* setup Z_chip */
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* D-channel 2 Kbyte/sec */
	  f->Z_chip = (2*HFCSUSB_TX_FRAMES);    /* length of data */

	  /* only one of four callbacks
	   * should send out +/- 4 bytes
	   */
	  if(f->F_drvr & 3)
	  {
	    frlengths_adj = frlengths-1;
	    goto no_adjust;
	  }
	}
	else
	{
	  /* B-channel 8 Kbyte/sec */
	  f->Z_chip = (8*HFCSUSB_TX_FRAMES);    /* length of data */

	  /* only one of two callbacks
	   * should send out +/- 1 byte
	   */
	  if (f->F_drvr & 1) {
	      frlengths_adj = frlengths-1;
	      goto no_adjust;
	  }
	}

	/*
	 * Adjust average transmit speed
	 * to match ISDN clock, by adding
	 * or removing a byte:
	 */
#ifdef HFCSUSB_DEBUG
	printf("%c", !fifo_level ? '+' : '-');
#endif
	if(!fifo_level)
	{
	  /* send more data */
	  f->Z_chip += (1*HFCSUSB_TX_ADJUST);
	  average   += (1);
	}
	else
	{
	  /* send less data */
	  f->Z_chip -= (1*HFCSUSB_TX_ADJUST);
	  average   -= (1);
	}

 no_adjust:
	Z_chip_written = f->Z_chip;

	/* get start of 2nd buffer */
	d1_start = sc->sc_temp_ptr;
	d1_start += HFCSUSB_TX_BUFSIZE;

	/* get start of 2nd buffer */
	f->Z_ptr = d1_start;

	/*
	 * Call filter(,)
	 */

	(f->filter)(sc, f);

	/* update ``d1_end'' */
	d1_end = f->Z_ptr;

	/*
	 * fill unused FIFO space
	 * with the last byte:
	 */

	memset_1(f->Z_ptr,
		 f->last_byte,
		 f->Z_chip);

	d1_end += f->Z_chip;

	/* it is not possible to write
	 * any data to the fifo outside
	 * of this function: clear ``Z_chip''
	 */
	f->Z_chip = 0;

	/*
	 * Expand D-channel data
	 *
	 * NOTE: In the code below
	 * ``d1_end'' is fixed and
	 * ``d1_start'' is decreased
	 */
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  enum { EXPANSION_BUFFER_SIZE = HFCSUSB_TX_BUFSIZE
		 /* leave room for optimization: */
		 - (HFCSUSB_TX_FRAMESIZE) };

#if (HFCSUSB_TX_BUFSIZE -		\
     (2*HFCSUSB_TX_FRAMES) - (1*HFCSUSB_TX_ADJUST) -	\
     (8*HFCSUSB_TX_FRAMES) - (4*HFCSUSB_TX_ADJUST) -	\
     (HFCSUSB_TX_FRAMESIZE)) < 0
#error "HFCSUSB_TX_FRAMESIZE too small to expand D-channel data!"
#endif
	  /* increase adjustment to +/- 4 bytes
	   * [after expansion].
	   */
	  if (frlengths_adj >= frlengths) {
	      frlengths_adj = frlengths + (4*HFCSUSB_TX_ADJUST);
	  }

	  /* 0                    1                    2
	   * |----------- d1_start|--------------------|
	   * |                    |<- d1_end|          | 
	   * |                    |           <- tmp|  |
	   * |                    |                  XX|
	   * |                    |<- exp. buffer ->|  |
	   *
	   * get end of expansion buffer
	   * into tmp; input pointer is d1_end
	   */
	  tmp = d1_start + EXPANSION_BUFFER_SIZE;

	  while(d1_end != d1_start) 
	  {
	    tmp -= 4;
	    d1_end -= 1;

	    /* expand 1:4 */
	    tmp[0] = d1_end[0] >> 0;
	    tmp[1] = d1_end[0] >> 2;
	    tmp[2] = d1_end[0] >> 4;
	    tmp[3] = d1_end[0] >> 6;
	  }

	  /* update ``d1_start'' and ``d1_end'' */
	  d1_start = tmp;
	  d1_end += EXPANSION_BUFFER_SIZE;
	}

	/*
	 * Prepare data for USB
	 *
	 * get start of 1st buffer
	 * (output pointer)
	 */
	tmp = sc->sc_temp_ptr;

	/* build ``USB frames'' */
	for(;
	    frlengths < frlengths_end;
	    frlengths++)
	{
	  /* NOTE:
	   * ``(u_int32_t *)tmp + 1'' is not the same as
	   * ``(u_int32_t *)(tmp+1)''
	   */

	  /* FIFO Control Byte */
	  *(tmp) = 0;

	  /* copy 9-bytes */
	  ((u_int32_p_t *)(tmp +1))->data = ((u_int32_p_t *)(d1_start   ))->data;
	  ((u_int32_p_t *)(tmp +5))->data = ((u_int32_p_t *)(d1_start +4))->data;
	                 *(tmp +9) = *(d1_start +8);

	  if(frlengths == frlengths_adj)
	  {
	    /* restore average */
	    if(average < 8)
	       average++;
	    else
	       average--;
	  }

	  /* update d1_start */
	  d1_start              += average;

	  /* update tmp and store total frame length */
	  tmp                   += (*frlengths = (average +1));

	  /* check end of data */
	  if(d1_start > d1_end)
	  {
	    /* adjust frame length */
	    *frlengths             -= (d1_start - d1_end);

	    /* send zero-length frame(s)
	     * when out of data
	     */
	    while(++frlengths < frlengths_end)
	    {
	      *frlengths = 0;
	    }

	    break;
	  }
	}

	usb2_copy_in(xfer->frbuffers + 0, 0, sc->sc_temp_ptr,
		     tmp - ((uint8_t *)(sc->sc_temp_ptr)));

	xfer->nframes = HFCSUSB_TX_FRAMES; 

	usb2_start_hardware(xfer);

	/* the completion-time of the last transfer
	 * gives you the start time of the next
	 * transfer!
	 */
	f->Z_read_time = (xfer->isoc_time_complete * 8);
	f->Z_chip_written = 16; /* Approximation on the number
				 * of bytes left in the FIFO
				 * at "Z_read_time". This number
				 * should be less than the FIFO
				 * threshold value.
				 */
 done:
	return;

  default: /* Error */
	if (xfer->error == USB_ERR_CANCELLED) {
		return;
	}
	goto tr_setup;
  }
}

static void
hfcsusb2_callback_isoc_rx USB_CALLBACK_T(xfer)
{
	ihfc_sc_t  *sc = xfer->priv_sc;
	ihfc_fifo_t *f = xfer->priv_fifo;

	u_int8_t
	  *d1_start, *d1_end, *tmp, *tmp_end, len;

	uint32_t *frlengths = xfer->frlengths;
	uint32_t *frlengths_end  =  frlengths + HFCSUSB_RX_FRAMES;

  switch (USB_GET_STATE(xfer)) {
  case USB_ST_TRANSFERRED: 

	/* store "Z_read_time", which is used by echo-cancelling */

	f->Z_read_time = (xfer->isoc_time_complete * 8);

	/* get start of 1st buffer */
	tmp = sc->sc_temp_ptr;

	usb2_copy_out(xfer->frbuffers + 0, 0, 
		      tmp, HFCSUSB_RX_BUFSIZE);

	/* get start of 2nd buffer */
	d1_start = d1_end = tmp + (HFCSUSB_RX_FRAMES*HFCSUSB_RX_FRAMESIZE);

	for(;
	    frlengths < frlengths_end;
	    frlengths++)
	{
#define break illegal
#define goto illegal

		u_int8_t adj;

#define ST_TRANSFER ST_FZ_LOADED

		/* get frame length */

		len = *frlengths;

#ifdef HFCSUSB_DEBUG
		printf("len=%d ", len);
#endif
		adj = 0;

		if(!(f->state & ST_TRANSFER))
		{
		  if(len >= 2)
		  {
		    /*
		     * new data transfer
		     */

		    if(len >= HFCSUSB_RX_FRAMESIZE)
		    {
		      f->state |= ST_TRANSFER;

		      /* check EOF ?? */
		    }

		    /* skip FIFO Control Bytes */
		    len -= 2;
		    tmp += 2;
		    adj = 2;
		  }
		  else
		  {
		    len = 0; /* ? */
		  }
		}
		else
		{
		  if(len < HFCSUSB_RX_FRAMESIZE)
		  {
		    f->state &= ~ST_TRANSFER;

		    /* check EOF ?? */
		  }
		}

#if ( (16*1024) - (3*2*2*HFCSUSB_RX_FRAMESIZE*HFCSUSB_RX_FRAMES) \
      - (4*0x100) ) < HFCSUSB_RX_FRAMESIZE
#error "No room for optimization bytes past end of xfer->buffer!"
#endif

#if HFCSUSB_RX_FRAMESIZE != 12
#error "HFCSUSB_RX_FRAMESIZE != 12: Re-optimize code!"
#endif

		/* copy 12-bytes
		 *
		 * NOTE: This optimization can read a
		 * maximum of 2 bytes past the end of
		 * the buffer
		 */

		(((u_int32_p_t *)d1_end)+0)->data = (((u_int32_p_t *)tmp)+0)->data;
		(((u_int32_p_t *)d1_end)+1)->data = (((u_int32_p_t *)tmp)+1)->data;
		(((u_int32_p_t *)d1_end)+2)->data = (((u_int32_p_t *)tmp)+2)->data;

		/*
		 * update data pointer
		 */
		d1_end             += len;
		tmp -= adj;
		tmp += HFCSUSB_RX_FRAMESIZE;
#undef break
#undef goto
	}

	/* restore frlengths */
	frlengths -= HFCSUSB_RX_FRAMES;

	/*
	 * Compress D-channel data (4:1)
	 */
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* reset output buffer */
	  tmp = tmp_end = d1_start;

	  /* align ``tmp'' to 4 bytes */
	  tmp += (-f->F_drvr) & 3;

	  /* update alignment by adding
	   * total length received:
	   */
	  (f->F_drvr) += (d1_end - d1_start);

	  /* compress data */
	  while(tmp < d1_end)
	  {
	    tmp_end[0]  = tmp[0];
	    tmp_end += 1; tmp += 4;
	  }

	  /* update d1_end */
	  d1_end = tmp_end;
	}

	/*
	 * Setup Z_ptr and Z_chip
	 */

	f->Z_ptr  =          d1_start; /* where data starts */
	f->Z_chip = d1_end - d1_start; /* length of data */

	/*
	 * Call filter(,)
	 */

	(f->filter)(sc, f);

  case USB_ST_SETUP: tr_setup:

	/* setup transfer */

	xfer->nframes = HFCSUSB_RX_FRAMES;

	for(;
	    frlengths < frlengths_end;
	    frlengths++)
	{
	  *frlengths = HFCSUSB_RX_FRAMESIZE;
	}

	usb2_start_hardware(xfer);
	return;

  default: /* Error */
	if (xfer->error == USB_ERR_CANCELLED) {
		return;
	}
	goto tr_setup;

  }
}

static void
hfcsusb2_cfg_chip_config_write_f(struct ihfc_sc *sc,
				struct ihfc_config_copy *cc, uint16_t refcount)
{
	ihfc_fifo_t *f;
	ihfc_fifo_t *f_other;
	uint8_t regtemp[6];

	f = sc->sc_fifo + refcount;

	/*
	 * write FIFO configuration
	 */

	/* bits[5..7] must be the same
	 * in registers ``f_rx->s_con_hdlc''
	 * and ``f_tx->s_con_hdlc''
	 */
	if (FIFO_DIR(f) == transmit) {
	    f_other = f - transmit + receive;
	} else {
	    f_other = f - receive + transmit;
	}

	/* buffer up the registers so that what is
	 * written to the chip is consistent, hence
	 * the "hfcsusb2_cfg_write_1()" function will
	 * sleep:
	 */
	regtemp[0] = f_other->s_fifo_sel;
	regtemp[1] = f_other->s_con_hdlc;
	regtemp[2] = f->s_fifo_sel;
	regtemp[3] = f->s_con_hdlc;
	regtemp[4] = f->s_par_hdlc;
	regtemp[5] = f->last_byte;

	/* write FIFO_SEL */
	hfcsusb2_cfg_write_1(sc, 0x0F, regtemp[0]);

	/* write CON_HDLC */
	hfcsusb2_cfg_write_1(sc, 0xFA, regtemp[1]);

	/* write FIFO_SEL */
	hfcsusb2_cfg_write_1(sc, 0x0F, regtemp[2]);

	/* write CON_HDLC */
	hfcsusb2_cfg_write_1(sc, 0xFA, regtemp[3]);

	/* write PAR_HDLC */
	hfcsusb2_cfg_write_1(sc, 0xFB, regtemp[4]);

	/* write INC_RES_F */
	hfcsusb2_cfg_write_1(sc, 0x0E, 0x02);

	if (FIFO_DIR(f) == transmit) {
	    /* write FIFO_DATA (update repeating byte) */
	    hfcsusb2_cfg_write_1(sc, 0x84, regtemp[5]);
	}
	return;
}

static void
hfcsusb2_cfg_chip_config_write_r(struct ihfc_sc *sc,
				struct ihfc_config_copy *cc, uint16_t refcount)
{
	register_list_t *r;

	/*
	 * configure chip,
	 * write new configuration
	 */
	REGISTER_FOREACH(r,sc->sc_default.d_register_list) {
	    u_int8_t *data  = &OFF2REG(sc->sc_config, r->offset);
	    u_int8_t *data2 = &OFF2REG(sc->sc_config2, r->offset);

	    if (refcount || (*data != *data2)) {

	        /* must update the shadow config before writing
		 * the register, hence "hfcsusb2_cfg_write_1()" will
		 * sleep, and then the config value can change:
		 */

		*data2 = *data;

		/* write the register */

		hfcsusb2_cfg_write_1(sc, r->regval, *data);
	    }
	}
	return;
}

static void
hfcsusb2_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	if ((f == IHFC_CONFIG_WRITE_UPDATE) ||
	    (f == IHFC_CONFIG_WRITE_RELOAD))
	{
	  __typeof(sc->sc_resources.usb_xfer[0])
	    *xfer = &sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_START],
	    *xfer_end = &sc->sc_resources.usb_xfer[HFCSUSB_FIFO_XFER_END];

	  usb2_config_td_queue_command
	    (&(sc->sc_config_td), NULL,
	     &hfcsusb2_cfg_chip_config_write_r, 0, 
	     (f == IHFC_CONFIG_WRITE_RELOAD) ? 1 : 0);

	  /*
	   * Update pipes:
	   * =============
	   *
	   * If the device is not connected or the
	   * line is powered down, the USB pipes
	   * are stopped.  Else the USB pipes are
	   * started.
	   */

	  for(;
	      xfer < xfer_end;
	      xfer++)
	  {
	    ihfc_fifo_t *f = xfer[0]->priv_fifo;

	    if((f->prot_curr.protocol_1 != P_DISABLE) &&
	       ((FIFO_NO(f) == d1r) || 
		sc->sc_state[0].state.active))
	    {
	      /* start pipe */
	      usb2_transfer_start(xfer[0]);
	    }
	    else
	    {
	      /* stop pipe */
	      usb2_transfer_stop(xfer[0]);
	    }
	  }
	}
	else
	{
	  /* reset frame counter */
	  f->F_drvr = -1;

	  /* need to re-read F_USAGE */
	  f->Z_chip = 0x80;

	  /* clear FIFO level */
	  sc->sc_config.fifo_level &= ~(1 << (f->s_fifo_sel));

	  usb2_config_td_queue_command
	    (&(sc->sc_config_td), NULL,
	     &hfcsusb2_cfg_chip_config_write_f, 0, 
	     FIFO_NO(f));
	}
	return;
}

static void
hfcsusb2_fsm_read FSM_READ_T(sc,f,ptr)
{
	/*
	 * Use cached version,
	 * hence an USB-transfer
	 * takes too much time:
	 */
	*ptr = (sc->sc_config.s_states +
		((sc->sc_config.s_sctrl & 0x4) ?
		 HFC_NT_OFFSET : HFC_TE_OFFSET)) & 0xf;
	return;
}

static void
hfcsusb2_cfg_fsm_write(struct ihfc_sc *sc,
		      struct ihfc_config_copy *cc, uint16_t refcount)
{
	uint8_t temp;

	temp = refcount;

	/* write STATES */
	hfcsusb2_cfg_write_1(sc,0x30,temp);

#ifdef HFC_FSM_RESTART
	if (temp & 0x10) {
	    temp ^= 0x10;

	    /* assuming more than 5.21us delay
	     * in "hfcsusb2_cfg_write_1":
	     */
	    hfcsusb2_cfg_write_1(sc,0x30,temp);
	}
#endif
	return;
}

static void
hfcsusb2_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	usb2_config_td_queue_command
          (&(sc->sc_config_td), NULL,
	   &hfcsusb2_cfg_fsm_write, 0, *ptr);
	return;
}

/*-------------------------------------------------+
 | hfcsusb related information                     |
 +-------------------------------------------------*/

register_list_t
hfcsusb2_register_list[] =
{
#if 0
  { REG2OFF(s_int_m1_usb),   0x1a },
  { REG2OFF(s_int_m2_usb),   0x1b },
#endif

  { REG2OFF(s_clkdel),       0x37 }, /* bit 7 unused */
  { REG2OFF(s_sctrl),        0x31 },
  { REG2OFF(s_sctrl_e_usb),  0x32 }, /* ~s_sctrl_e(pci) */
  { REG2OFF(s_sctrl_r),      0x33 },

  { REG2OFF(s_ldata),        0x1F }, /* AUX port data (leds) */

  { REG2OFF(s_b1_ssl),       0x20 },
  { REG2OFF(s_b2_ssl),       0x21 },
  { REG2OFF(s_a1_ssl),       0x22 },
  { REG2OFF(s_a2_ssl),       0x23 },
  { REG2OFF(s_b1_rsl),       0x24 },
  { REG2OFF(s_b2_rsl),       0x25 },
  { REG2OFF(s_a1_rsl),       0x26 },
  { REG2OFF(s_a2_rsl),       0x27 },

  { REG2OFF(s_mst_mode),     0x14 },

  { 0, 0 }
};

static const struct usb_config
hfcsusb2_usb[] =
{
  /*
   * NOTE: 2 xfers are used for configuration
   * and 12 xfers are used for FIFO transfer.
   * The order of the FIFOs is the same as
   * for the HFC-SP.
   */

  [HFCSUSB_CONF_XFER_WRITE] = {
    .type      = UE_CONTROL,
    .endpoint  = 0x00, /* Control pipe */
    .direction = UE_DIR_ANY,
    .bufsize   = sizeof(struct usb_device_request), /* bytes */
    .callback  = &hfcsusb2_callback_chip_write,
    .timeout   = 1000, /* ms */
  },

  [HFCSUSB_CONF_XFER_READ] = {
    .type      = UE_CONTROL,
    .endpoint  = 0x00, /* Control pipe */
    .direction = UE_DIR_ANY,
    .bufsize   = sizeof(struct usb_device_request)+1, /* bytes */
    .callback  = &hfcsusb2_callback_chip_read,
    .timeout   = 1000, /* ms */
  },

#define HFCSUSB_XFER(index, pipe_no, fifo)\
  [index+0] = {\
    .type      = UE_ISOCHRONOUS,\
    .endpoint  = pipe_no, /* ISOC Out */\
    .direction = UE_DIR_OUT,\
    .flags     = { .short_xfer_ok = 1 },\
    .frames    = HFCSUSB_TX_FRAMES,\
    .bufsize   = HFCSUSB_TX_BUFSIZE,\
    .callback  = &hfcsusb2_callback_isoc_tx,\
  },\
  [index+1] = {\
    .type      = UE_ISOCHRONOUS,\
    .endpoint  = pipe_no, /* ISOC Out */\
    .direction = UE_DIR_OUT,\
    .flags     = { .short_xfer_ok = 1 },\
    .frames    = HFCSUSB_TX_FRAMES,\
    .bufsize   = HFCSUSB_TX_BUFSIZE,\
    .callback  = &hfcsusb2_callback_isoc_tx,\
  },\
  [index+2] = {\
    .type      = UE_ISOCHRONOUS,\
    .endpoint  = pipe_no, /* ISOC In */\
    .direction = UE_DIR_IN,\
    .flags     = { .short_xfer_ok = 1 },\
    .frames    = HFCSUSB_RX_FRAMES,\
    .bufsize   = HFCSUSB_RX_BUFSIZE,\
    .callback  = &hfcsusb2_callback_isoc_rx,\
  },\
  [index+3] = {\
    .type      = UE_ISOCHRONOUS,\
    .endpoint  = pipe_no, /* ISOC In */\
    .direction = UE_DIR_IN,\
    .flags     = { .short_xfer_ok = 1 },\
    .frames    = HFCSUSB_RX_FRAMES,\
    .bufsize   = HFCSUSB_RX_BUFSIZE,\
    .callback  = &hfcsusb2_callback_isoc_rx,\
  },\
  /**/

  /* need to update hfcsusb2_chip_reset() when 
   * changing order of transfers:
   */
  HFCSUSB_XFER(0+HFCSUSB_FIFO_XFER_START, 7, d1)    /* D1 - channel */
  HFCSUSB_XFER(4+HFCSUSB_FIFO_XFER_START, 5, b1)    /* B1 - channel */
  HFCSUSB_XFER(8+HFCSUSB_FIFO_XFER_START, 6, b2)    /* B2 - channel */
};

static void
hfcsusb2_cfg_read_status(struct ihfc_sc *sc,
			struct ihfc_config_copy *cc, uint16_t refcount)
{
	ihfc_fifo_t *f;
	uint8_t temp;

	IHFC_MSG("\n");

	f = sc->sc_fifo + d1t;

	/* read STATES */
	temp = hfcsusb2_cfg_read_1(sc,0x30);

	/* check for statemachine change */
	if (sc->sc_config.s_states != temp) {

	    /* update s_states */
	    sc->sc_config.s_states = temp;

	    ihfc_fsm_update(sc,f,0);
	}

	if (!PROT_IS_HDLC(&(f->prot_curr))) {

	    /* check FIFO frame counters */

	    /* write FIFO_SEL */
	    hfcsusb2_cfg_write_1(sc, 0x0F, f->s_fifo_sel);

	    /* read F1 and F2 */
	    if (hfcsusb2_cfg_read_1(sc,0x0C) !=
		hfcsusb2_cfg_read_1(sc,0x0D)) {

	        IHFC_ERR("F1 != F2\n");

		/* reset and re-configure FIFO */
		hfcsusb2_chip_config_write(sc,f);
	    }
	}

	/* read F_FILL(reg=0x1B) */
	temp = hfcsusb2_cfg_read_1(sc,0x1B);

	/* update F_FILL (FIFO-threshold-bits) */
	sc->sc_config.fifo_level = temp;

	return;
}

static void
hfcsusb2_chip_status_read(ihfc_sc_t *sc)
{
	usb2_config_td_queue_command
	  (&(sc->sc_config_td), NULL, 
	   &hfcsusb2_cfg_read_status, 0, 0);
	return;
}

static u_int8_t
hfcsusb2_d1t_program(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	/* re-start any stopped pipes,
	 * hence the D-channel transmit
	 * pipe is allowed to be stopped, to
	 * reduce CPU usage:
	 */
	hfcsusb2_chip_config_write (sc,IHFC_CONFIG_WRITE_UPDATE);
	return PROGRAM_DONE;
}

static ihfc_fifo_program_t *
hfcsusb2_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if(FIFO_NO(f) == d1t)
	{
	  if(PROT_IS_HDLC(&(f->prot_curr)))
	  {
	      program = &hfcsusb2_d1t_program;
	  }
	  if(PROT_IS_TRANSPARENT(&(f->prot_curr)))
	  {
	      program = &i4b_unknown_program;
	  }
	}
	else
	{
	  if(PROT_IS_TRANSPARENT(&(f->prot_curr)))
	  {
	      program = &i4b_unknown_program;
	  }
	}
	return program;
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(hfcsusb2_dbase_root)
{
  I4B_DBASE_ADD(desc                    , "HFC-2BDS0 128K USB ISDN adapter");

  I4B_DBASE_ADD(c_chip_reset            , &hfcsusb2_chip_reset);

  I4B_DBASE_ADD(c_chip_config_write     , &hfcsusb2_chip_config_write);

#if 0
  I4B_DBASE_ADD(c_chip_unselect         , &hfcsusb2_chip_unselect);
#endif

  /* delay 1/2 second */
  I4B_DBASE_ADD(c_chip_status_read      , &hfcsusb2_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check     , &hfcsusb2_chip_status_check);
  I4B_DBASE_ADD(d_interrupt_delay       , hz / 2);

  I4B_DBASE_ADD(c_fsm_read              , &hfcsusb2_fsm_read);
  I4B_DBASE_ADD(c_fsm_write             , &hfcsusb2_fsm_write);
  I4B_DBASE_ADD(d_fsm_table             , &hfcsusb2_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program      , &hfcsusb2_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read             , &hfcsusb2_fifo_read);
  I4B_DBASE_ADD(c_fifo_write            , &hfcsusb2_fifo_write);

  I4B_DBASE_ADD(d_register_list         , &hfcsusb2_register_list[0]);
  I4B_DBASE_ADD(d_channels              , 6);

  I4B_DBASE_ADD(d_temp_size		, (2*MAX(HFCSUSB_RX_BUFSIZE,HFCSUSB_TX_BUFSIZE)));

  I4B_DBASE_ADD(usb                     , &hfcsusb2_usb[0]);
  I4B_DBASE_ADD(usb2_length              , (sizeof(hfcsusb2_usb)/sizeof(hfcsusb2_usb[0])));

  I4B_DBASE_ADD(usb2_conf_no             , 1); /* bConfigurationValue */
  I4B_DBASE_ADD(usb2_iface_no            , 1); /* bInterfaceNumber */
  I4B_DBASE_ADD(usb2_alt_iface_no        , 1); /* bAlternateSetting */

  I4B_DBASE_ADD(o_PORTABLE              , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */
  I4B_DBASE_ADD(o_ECHO_CANCEL_ENABLED, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask         , (I4B_OPTION_NT_MODE|
					   I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value        , (I4B_OPTION_POLLED_MODE));
}

I4B_USB_DRIVER(/* Generic HFC-S USB
                * Colognechip USB evaluation TA (no led support)
                */
	       /* write aux port if no led support ?*/
               .vid      = 0x09592bd0,
               );

/* In the LED schemes YES means "on" 
 * and NO means "off"
 */
#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfcsusb2_dbase_root);

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x01,YES)				\
  m(d1, 0x02,YES)				\
  m(b1, 0x00,YES) /* not present */		\
  m(b2, 0x00,YES) /* not present */		\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* DrayTec ISDN USB
                * DrayTek USB ISDN TA (MiniVigor)
                */
               .vid      = 0x06751688,
               );

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfcsusb2_dbase_root);

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x04,YES)				\
  m(d1, 0x00,YES) /* not present */		\
  m(b1, 0x01,YES)				\
  m(b2, 0x02,YES)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* Stollmann USB TA */
               .vid      = 0x07422008,
               );

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfcsusb2_dbase_root);

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x02,YES)				\
  m(d1, 0x01,YES)				\
  m(b1, 0x04,YES)				\
  m(b2, 0x08,YES)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* ISDN Terminal Adapter
                */
               .vid      = 0x07422009,
               );

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfcsusb2_dbase_root);

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x02,YES)				\
  m(d1, 0x00,YES) /* not present */		\
  m(b1, 0x04,YES)				\
  m(b2, 0x01,YES)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* OliTec ISDN USB
                * OliTec Modem RNIS USB V2
                */
               .vid      = 0x08e30301,
               );

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfcsusb2_dbase_root);

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x80,YES)				\
  m(d1, 0x40, NO)				\
  m(b1, 0x10, NO)				\
  m(b2, 0x20, NO)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* Bewan ISDN USB TA
                * Bewan Modem RNIS USB
		* Twister ISDN TA
                */
               .vid      = 0x07fa0846,
               );

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfcsusb2_dbase_root);

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x80,YES)				\
  m(d1, 0x40, NO)				\
  m(b1, 0x20, NO)				\
  m(b2, 0x10, NO)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_USB_DRIVER(/* ISDN USB TA */
               .vid      = 0x07b00006,
               );

I4B_USB_DRIVER(/* Billion tinyUSB
                * Billion USB TA 2
                */
               .vid      = 0x07b00007,
               );

/*
 * cleanup
 */

#undef HFCSUSB_FIFOSIZE
#undef HFCSUSB_RX_FRAMES
#undef HFCSUSB_RX_FRAMESIZE

#undef HFCSUSB_TX_FRAMES
#undef HFCSUSB_TX_ADJUST
#undef HFCSUSB_TX_FRAMESIZE

#undef HFCSUSB_CONF_XFER_WRITE
#undef HFCSUSB_CONF_XFER_READ
#undef HFCSUSB_FIFO_XFER_START
#undef HFCSUSB_FIFO_XFER_DCHAN

#undef HFCSUSB_FIFO_XFER_END

#undef HFCSUSB_MEM_BASE

#undef HFCSUSB_XFER
#undef HFCSUSB_DEBUG
#undef HFCSUSB_DEBUG_INTR
#undef HFCSUSB_DEBUG_STAT
#undef HFCSUSB_DEBUG_ERR
#undef LED_SCHEME
#undef ST_TRANSFER

#endif /* _I4B_HFCSUSB_H_ */
