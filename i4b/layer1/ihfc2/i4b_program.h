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
 *	i4b_program.h - FIFO processing programs
 * 	----------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_PROGRAM_H_
#define _I4B_PROGRAM_H_

/*
 * What is a FIFO processing program?
 * ==================================
 *
 * A FIFO processing program is ...  
 *
 *  - an attempt to generalize the way software interfaces with
 *  hardware to transfer data and frames.

 *  - emulating the ideal FIFO interface (see further down)
 *
 *  - reset when "f->program_state" is set to "-1". Else "f->program_state"
 *  is entirely under control of the FIFO program. 
 *
 *  - selected by the multiplexed function FIFO_GET_PROGRAM(sc,f)
 *  and is cached in "f->program".
 *
 *  - executed in serial.
 *
 *  - prototyped in the file "i4b_ihfc2_ext.h".
 *
 *  - storing its parameters in "struct sc_fifo" and "struct ihfc_sc".
 *
 * PROGRAM SPECIFICATIONS:
 *
 *   a) A FIFO program must be capable of being reset by
 *      ``ihfc_fifo_program_reset(,)'' anytime after
 *      that the FIFO program has returned.
 *
 *   b) A FIFO program must be capable to recover from [any possible] 
 *      previous program when it is reset.
 *
 * HARDWARE SPECIFICATIONS:
 *
 *   c) The chip must generate at least one interrupt after chip_reset.
 *
 *   d) The chip must be able to return to normal operation after
 *      chip_reset and reloading of register values.
 *
 *
 * The hardware and software configuration that is needed to
 * make a FIFO work, can be found in the following functions:
 *
 *		o chip_reset(,) function (called at reset only)
 *		o chip_config_write(,) function
 *		o fifo_setup(,) function
 *		o fifo_link(,) function
 *
 * The IPAC FIFO programs are used when
 * the hardware is block-buffered.
 *
 * The HFC FIFO programs are used when
 * the hardware is ring-buffered.
 *
 * The FIFO filters are called directly
 * from the driver if using a FIFO program
 * makes the driver more complicated.
 *
 */


/*
 * THE IDEAL FIFO INTERFACE
 * ========================
 *
 * The ideal FIFO interface consists of one counter and two status
 * bits. The ideal FIFO interface is used by all the FIFO filters.
 *
 * In the descriptions below "f->" is a pointer to a structure of
 * type sc_fifo defined in "i4b_ihfc2.h".
 *
 * In the descriptions below "f->Z_chip" is a counter which represents
 * the maximum transfer length. This counter must be decrementer by
 * the filter each time it calls FIFO_READ_MULTI_1(..) or
 * FIFO_WRITE_MULTI_1(..).
 *
 *
 *
 * Flags in transmit direction (as seen by (f->filter)(sc,f))
 +------------+----------------+-------------------------------------+
 |ST_FRAME_END| ST_FRAME_ERROR | meaning (what the filter can*** do) |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |     0 **** |        0       A Can transmit data with length up to |
 |            |                | the value given by f->Z_chip. Data  |
 |            |                | is written to the "current frame".  |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |     0 **** |        1 **    B The "current frame", if any, was    |
 |            |                | aborted by chip.                    |
 |            |                |                                     |
 |            |                A Can transmit data with length up to |
 |            |                | the value given by f->Z_chip. Data  |
 |            |                | is written to the "next frame".     |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |      1 *   |        0       A Can transmit data with length up to |
 |            |                | the value given by f->Z_chip. Data  |
 |  HDLC-mode |                | is written to the "current frame".  |
 |  only.     |                |                                     |
 |            |                C The "current frame" ends after the  |
 |            |                | data, if any, which was or has been |
 |            |                | written.                            |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |      1 *   |        1 **    B The "current frame", if any, was    |
 |            |                | aborted by chip.                    |
 |  HDLC-mode |                |                                     |
 |  only.     |                A Can transmit data with length up to |
 |            |                | the value given by f->Z_chip. Data  |
 |            |                | is written to the "next frame".     |
 |            |                |                                     |
 |            |                C The "next frame" ends after the     |
 |            |                | data, if any, which was or has been |
 |            |                | written.                            |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 * NOTES:
 *   **** if HDLC mode is selected, writing less than f->Z_chip 
 *        bytes totally and not setting ST_FRAME_END, is not allowed.
 *
 *    *** if the filter writes no data, when (f->Z_chip != 0), it
 *        cannot depend on being called back, with the exception
 *        of sending a zero length HDLC-frame.
 *
 *     ** the filter is not allowed to set ST_FRAME_ERROR
 *
 *      * the filter can set ST_FRAME_END to indicate end of frame.
 *
 *        NOTE: ST_FRAME_END should always be cleared before the
 *              filter is called.
 *
 *        NOTE: Zero frames are allowed, but the hardware may
 *              just ignore them (should not).
 *
 *
 * Flags in receive direction (as seen by (f->filter)(sc,f))
 +------------+----------------+-------------------------------------+
 |ST_FRAME_END| ST_FRAME_ERROR | meaning (what the filter must** do) |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |      0     |        0       A Must receive data with length given |
 |            |                | by f->Z_chip. Data is read from the |
 |            |                | "current frame".                    |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |      0     |        1 *     B The "current frame", if any, was    |
 |            |                | aborted by chip.                    |
 |            |                |                                     |
 |            |                A Must receive data with length given |
 |            |                | by f->Z_chip. Data is read from the |
 |            |                | "next frame".                       |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |      1 *   |        0       A Must receive data with length given |
 |            |                | by f->Z_chip. Data is read from the |
 |  HDLC-mode |                | "current frame".                    |
 |  only.     |                |                                     |
 |            |                C After data has been read the        |
 |            |                | "current frame" ends.               |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 |            |                |                                     |
 |      1 *   |        1 *     B The "current frame", if any, was    |
 |            |                | aborted by chip.                    |
 |            |                |                                     |
 |  HDLC-mode |                A Must receive data with length given |
 |  only.     |                | by f->Z_chip. Data is read from the |
 |            |                | "next frame".                       |
 |            |                |                                     |
 |            |                C After data has been read the        |
 |            |                | "next frame" ends.                  |
 |            |                |                                     |
 +------------+----------------+-------------------------------------+
 *
 * NOTES:
 *     ** if the filter returns when f->Z_chip is non-zero the
 *        leftover data of the "current" or in the  two latter
 *        cases "next" frame, will be dumped. The filter  will
 *        not know when the dumping ends,  but if f->Z_chip is
 *        non-zero while dumping,  the  data  is read from the
 *        post-dumped frame. Post means after.
 *
 *      * the filter is not allowed to set ST_FRAME_END or ST_FRAME_ERROR!
 *
 *        NOTE: Zero-length frames are allowed !
 */

/*---------------------------------------------------------------------------*
 * : program check status - routine for converting bits to FIFO pointers
 *---------------------------------------------------------------------------*/
static void
ihfc_program_check_status(ihfc_sc_t *sc)
{
	ihfc_fifo_t    **ptr = &sc->sc_intr_list[0];
	ihfc_fifo_t       *f = &sc->sc_fifo[0];
	__typeof(sc->sc_intr_status[0])
	  *status            = &sc->sc_intr_status[0],
	  *status_end        = &sc->sc_intr_status_end[0];

	/* FIFO processing programs will be called in
	 * order receive first then transmit
	 */
	do {
	    /* check for any interrupts */
	    if(status[0] != 0)
	    {
#if (SC_INTR_BITS != 8)
#error "please update this code to check SC_INTR_BITS at a time!"
#endif
		if(status[0] & (1<<0)) *ptr++ = f+0;
		if(status[0] & (1<<1)) *ptr++ = f+1;
		if(status[0] & (1<<2)) *ptr++ = f+2;
		if(status[0] & (1<<3)) *ptr++ = f+3;
		if(status[0] & (1<<4)) *ptr++ = f+4;
		if(status[0] & (1<<5)) *ptr++ = f+5;
		if(status[0] & (1<<6)) *ptr++ = f+6;
		if(status[0] & (1<<7)) *ptr++ = f+7;

		/* clear status bits */
		status[0] = 0;
	    }

	    /* update "f" */
	    f += SC_INTR_BITS;

	    /* update "status" */
	    status++;
	} while(status != status_end);

	/* update "sc_intr_list_curr" */
	sc->sc_intr_list_curr = ptr;

	return;
}

/* FIFO processing program return values: */

enum {
  PROGRAM_SLEEP,
  PROGRAM_LOOP,
  PROGRAM_DONE,
};

/*---------------------------------------------------------------------------*
 * : fifo processing kernel
 *---------------------------------------------------------------------------*/
static void
ihfc_fifo_program(ihfc_sc_t *sc)
{
	ihfc_fifo_t *f;

	u_int8_t status;
	u_int8_t max;

	/*
	 * overview:
	 * =========
	 *
	 * while(intr)
	 * {
	 *       intr = 0;
	 *
	 *      ... fifo processing ...
	 * }
	 *
	 * return;
	 */

	while(1)
	{
		/* Interrupts that occur during
		 * processing will be reported
		 * at next status check.
		 *
		 * After setup 
		 * sc->sc_intr_list_curr == NULL,
		 * so ``<='' must be used below:
		 */

		if(sc->sc_intr_list_curr <=
		   &sc->sc_intr_list[0])
		{
		    ihfc_program_check_status(sc);

		    if(sc->sc_intr_list_curr <=
		       &sc->sc_intr_list[0])
		    {
		        break;
		    }
		}

		/* get current fifo */
		f = *(sc->sc_intr_list_curr - 1);

		if(f->prot != P_DISABLE)
		{
		    IHFC_MSG("(#%d,prot=%d)\n", FIFO_NO(f), f->prot);
		}

		/* limit amount of processing.
		 * In the worst case 32*125us = 4ms
		 * is used per FIFO
		 */
		max = 32; /* XXX */

	loop:

		status = (f->program)(sc, f);

		/* call fifo processing program */
		switch(status) {
		case PROGRAM_SLEEP:
		    return;

		case PROGRAM_LOOP:
		    /*
		     * check number of loops
		     */
		    if(!max--)
		    {
		        IHFC_ERR("(#%d) FIFO quota "
				 "exceeded!\n", FIFO_NO(f));
			break;
		    }
		    goto loop;

		default:
		case PROGRAM_DONE:
		    break;
		}

		/* get next entry */
		sc->sc_intr_list_curr--;
	}
	return;
}

/*---------------------------------------------------------------------------*
 * : generic IPAC receive program (HDLC & TRANS)
 *---------------------------------------------------------------------------*/
u_int8_t
i4b_ipac_rx_program(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	/* check for program reset */

	if(f->program_state != 0)
	{
	    f->program_state = 0;

	    /* set error to recover from any
	     * previous frame(s)
	     */
	    (f->i_ista) |= (I_ISTA_ERR);

	    /* clear all ST_XX's used
	     */
	    (f->state) &= ~(ST_PROGRAM_MASK);
	}

	/* check for RME, RPF or ERR */

	if(!(f->i_ista & (I_ISTA_RME|I_ISTA_RPF|I_ISTA_ERR)))
	{
	    /* ISAC, HSCX and IPAC should not
	     * be read  unless  an  interrupt
	     * was generated
	     */
	    goto done;
	}

	/* check for ERR */

	if(f->i_ista & I_ISTA_ERR)
	{
	    goto rx_error;
	}

	/* check for ReceiveMessageEnd (RME) */

	if(f->i_ista & I_ISTA_RME)
	{
	    /*
	     * CHIP_STATUS_READ(sc) should have read
	     *	1) RBCL into f->Z_chip and
	     *	2) RSTA into f->F_chip
	     */

	    if(!(f->Z_chip &= (f->fm.i.block_size -1))) {
	         f->Z_chip  = (f->fm.i.block_size);
	    }

	    /* remove RSTA from RFIFO, if any */
	    if(f->fm.i.remove_stat)
	    {
	        f->Z_chip--;
	    }

	    /* check RSTA:
	     * check includes RDO, CRC and RAB
	     */
	    if(((f->F_chip) ^ 0x20) & 0x70)
	    {
	        goto rx_error;
	    }
	    else
	    {
	        f->state |= ST_FRAME_END;
	    }
	}
	else
	{
	    /*
	     * ReceivePoolFull (RPF)
	     */

	    f->Z_chip = (f->fm.i.block_size);
	}

	/* call filter */
	(f->filter)(sc,f);

	/*
	 * If  any  data  is leftover
	 * that  means that the  rest
	 * of  the  frame  should  be
	 * dumped. Else the  IRQ  can
	 * start looping with RPF set:
	 */
	if(f->Z_chip)
	{
 rx_error:
	    /* clear ST_FRAME_END and ST_FRAME_ERROR [if set] */
	    (f->state) &= ~(ST_FRAME_ERROR|ST_FRAME_END);

	    /* clear all interrupts */
	    f->i_ista &= ~(I_ISTA_ERR|I_ISTA_RME|I_ISTA_RPF);

	    /* Issue ``RRES+RMC'' on all errors to ensure
	     * that the transmission does not stall !
	     *
	     * RME + ``some frame error'' -> RMC
	     *    -> receiver stalls
	     */
	    HDLC_ERR("Frame error (Z_chip=%d,#%d)\n",
		     f->Z_chip, FIFO_NO(f));

	    /* set ST_FRAME_ERROR
	     * for next filter call, if any
	     */
	    f->state |= (ST_FRAME_ERROR);

	    /* no data for ``next frame''
	     * f->Z_chip = 0;
	     */

	    /* set CMDR reset bits
	     */
	    f->i_cmdr |= (I_CMDR_RRES|I_CMDR_RMC);
	}
	else
	{
	  /* clear ST_FRAME_END and ST_FRAME_ERROR [if set] */
	  (f->state) &= ~(ST_FRAME_ERROR|ST_FRAME_END);

	  /* clear all interrupts */
	  f->i_ista &= ~(I_ISTA_ERR|I_ISTA_RME|I_ISTA_RPF);

	  /* execute ReceiveMessageComplete command */
	  (f->i_cmdr) |=  (I_CMDR_RMC);
	}

 done:
        /*
	 * CMDR is written to chip
	 * by "CHIP_UNSELECT()"
	 */
	return PROGRAM_DONE;
}

/*---------------------------------------------------------------------------*
 * : generic IPAC transmit program (HDLC & TRANS)
 *---------------------------------------------------------------------------*/
u_int8_t 
i4b_ipac_tx_program(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	/* check for program reset */

	if(f->program_state != 0)
	{
	    f->program_state = 0;

	    /* set error to recover from any
	     * previous frame(s)
	     */
	    (f->i_ista) |= (I_ISTA_ERR);

	    /* clear all ST_XX's used
	     */
	    (f->state) &= ~(ST_PROGRAM_MASK);
	}

	/* check for XPR or ERR */

	if(!(f->i_ista & (I_ISTA_XPR|I_ISTA_ERR)))
	{
	    /* ISAC, HSCX and IPAC should not
	     * be written  unless  an  interrupt
	     * was generated
	     */
	    goto done;
	}

	/* clear any previous ST_FRAME_END or ST_FRAME_ERROR */

	(f->state) &= ~(ST_FRAME_END|ST_FRAME_ERROR);

	/* check for ERR */

	if(f->i_ista & I_ISTA_ERR)
	{
	    /* Issue XRES on all errors to ensure that
	     * the transmission does not stall !
	     *
	     * NOTE: It is assumed that the hardware
	     * will clear XTF and XME command-bits, if
	     * XRES command-bit is present. Else this
	     * must be done in software!
	     */
	    HDLC_MSG("(#%d) Transmit data underflow or "
		     "collision.\n", FIFO_NO(f));

	    /* execute XRES command so that an XPR
	     * interrupt will be generated:
	     */
	    f->i_cmdr |= I_CMDR_XRES;

	    /* set ST_FRAME_ERROR */

	    f->state |= ST_FRAME_ERROR;

	    /* no data for next frame */

	    f->Z_chip = 0;
	}
	else
	{
	    f->Z_chip = (f->fm.i.block_size); /* 0x20 or 0x40 */
	}

	/* clear status bits */

	f->i_ista &= ~(I_ISTA_XPR|I_ISTA_ERR);

	/* call filter */
	(f->filter)(sc,f);

	if(PROT_IS_HDLC(f->prot))
	{
	    if(f->state & ST_FRAME_END)
	    {
	        /* XTF will start transmission of data.
		 * XME will add an unstuffed 0x7e byte
		 * to the end of the frame. If XRES has
		 * been added it is up to the hardware
		 * to send a zero length frame!
		 */
	        (f->i_cmdr) |= (I_CMDR_XME|I_CMDR_XTF);

		goto done;
	    }
	}

	if(f->Z_chip < (f->fm.i.block_size))
	{
	    if(f->Z_chip > 0)
	    {
	        /* should only get here
		 * in extended transparent mode,
		 * but just in case, check the
		 * protocol:
		 */
		if(PROT_IS_HDLC(f->prot) == 0)
		{
		    /* allocate a temporary buffer 
		     * on the stack :
		     */
		    u_int16_t len = (f->Z_chip + 3);
		    u_int8_t buf[len];

		    /* to avoid extremely high
		     * interrupt rates, fill the
		     * remaining buffer space with
		     * 0xFF, which is what these
		     * chips will send when there
		     * is no more data:
		     */
		    IHFC_MSG("TX fill: %d bytes\n", f->Z_chip);

		    len /= 4;
		    while(len--)
		    {
		      ((u_int32_t *)&buf)[len] = 0xffffffff;
		    }

		    FIFO_WRITE_MULTI_1(sc,f,&buf[0],f->Z_chip);

		    f->Z_chip = 0;
		}
	    }

	    /* assume the filter is sending
	     * data when f->Z_chip has
	     * changed
	     */
	    (f->i_cmdr) |= (I_CMDR_XTF);
	}
	else
	{
	    /* assume the filter has finished
	     * sending data
	     *
	     * (see ``framing rules'')
	     *
	     * restore XPR bit, 
	     * which is still valid:
	     */
	    (f->i_ista) |= (I_ISTA_XPR);

	    /* TX FIFO needs a call-back,
	     * because it is not running
	     * any more
	     */
	    (f->state) &= ~(ST_RUNNING);
	}

 done:
        /*
	 * CMDR is written to chip
	 * by "CHIP_UNSELECT()"
	 */
	return PROGRAM_DONE;
}

static void
ihfc_fifo_fz_read(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	u_int16_t max;
	u_int16_t min;

	/* read fz counters */
	FIFO_FZ_READ(sc,f);

	IHFC_MSG("(#%d): "
		 "Z_chip:%04x Z_drvr:%04x, "
		 "F_chip:%02x F_drvr:%02x\n",
		 FIFO_NO(f),
		 (f->Z_chip), (f->Z_drvr),
		 (f->F_chip), (f->F_drvr));

	/* Z-counter range check:
	 * ======================
	 *
	 * NOTE: This code assumes that the Z-counters are
	 * of incremental type, and should be set to the
	 * lowest, inclusive value, if out of range. See
	 * ``INC_COUNTER_TIME_CHECK()'' for more information.
	 *
	 * NOTE: F-counters must be range checked by
	 * ``FIFO_FZ_READ(sc,f)'' !
	 *
	 * NOTE: Z_chip is only valid when F1 == F2
	 */

	max =  (f->fm.h.Zend); /* excluding */
	min = ((f->fm.h.Zend) - (f->fm.h.Zsize)); /* including */

	INC_COUNTER_RANGE_CHECK(f->Z_chip,min,max);
	INC_COUNTER_RANGE_CHECK(f->Z_drvr,min,max);

	/*
	 * FIFO calculation overview:
	 *
	 * len    = f->Z_chip - f->Z_drvr   (subtract 1 if tx)
	 * frames = f->F_chip - f->F_drvr   (subtract 1 if tx)
	 */

	/* calculate number of bytes in FIFO (RX-channel) */
	if(((f->Z_chip) -= (f->Z_drvr)) & Z_MSB) {
	    (f->Z_chip) += (f->fm.h.Zsize);
	}

	/* calculate number of frames in FIFO (TX-channel) */
	if(((f->F_chip)  = (f->F_drvr) - (f->F_chip)) & F_MSB) {
	    (f->F_chip) += (f->fm.h.Fsize);
	}
	return;
}

static void
ihfc_fifo_inc_fx(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	IHFC_MSG("FIFO_INC_FX()\n");

	/* increment F-counter */
	FIFO_INC_FX(sc,f);

	/* clear ST_FRAME_END and ST_FRAME_ERROR */
	f->state &= ~(ST_FRAME_END|ST_FRAME_ERROR);

	/* store the maximum allowed number
	 * of bytes that can be used by the
	 * next frame, into f->Z_chip3:
	 */
	f->Z_chip3 = f->fm.h.Zsize - f->Z_min_free;
	return;
}

/*---------------------------------------------------------------------------*
 * : generic HFC-XXX receive program (HDLC & TRANS)
 *---------------------------------------------------------------------------*/
u_int8_t
i4b_hfc_rx_program(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	u_int8_t *start, *end;
	u_int16_t len;

	enum {
	  /* wait states */
	  ST_0_FIFO_SEL,
	  ST_1_FIFO_FZ_READ,
	  ST_2_FIFO_INC_FX,

	  ST_3_FIFO_SEL_RST,
	  ST_4_FIFO_FZ_READ,
        };

	switch(f->program_state)
	{
	case ST_0_FIFO_SEL:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        return PROGRAM_SLEEP;
	    }

	    /* select FIFO */
	    FIFO_SELECT(sc,f);

	case ST_1_FIFO_FZ_READ:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_1_FIFO_FZ_READ;
	        return PROGRAM_SLEEP;
	    }

	    ihfc_fifo_fz_read(sc,f);

	    /* In HDLC mode the FIFO will append a
	     * status byte to the end of each frame,
	     * but without incrementing the Z-counter
	     * until the start of the next frame:
	     */
	    if(f->F_chip)
	    {
	        /* add the status byte */
	        (f->Z_chip)++;

		/* set ST_FRAME_END */
		(f->state) |= ST_FRAME_END;
	    }

	    /* check ST_FRAME_ERROR */
	    if(f->state & ST_FRAME_ERROR)
	    {
	        /* dump data */
	        goto rx_error;
	    }

	    /* call filter */
	    (f->filter)(sc,f);

	    if(PROT_IS_HDLC(f->prot))
	    {
	        /* In HDLC mode leftover data must be dumped
		 * to advance the FIFO pointer to the 
		 * beginning of the next frame:
		 */
	        if(f->Z_chip)
		{
		    HDLC_ERR("(#%d) Filter read overflow.\n",
			     FIFO_NO(f));
 rx_error:
		    do {
		        /* get buffer */
		        FIFO_GET_MEMORY(sc,f,&start,&end,&len);

			/* RX data */
			FIFO_READ_MULTI_1(sc,f,&start[0],len);

			/* update Z_chip */
			f->Z_chip -= len;

		    } while(f->Z_chip);

		    /* set ST_FRAME_ERROR */
		    f->state |= ST_FRAME_ERROR;
		}

		/* check ST_FRAME_END */
		if(f->state & ST_FRAME_END)
		{
		    /* calling "FIFO_INC_FX_PRE(,)" is not needed
		     * in receive direction !
		     */

	case ST_2_FIFO_INC_FX:

		    /* check T125 */
		    if(SC_T125_WAIT(sc))
		    {
		        f->program_state = ST_2_FIFO_INC_FX;
		        return PROGRAM_SLEEP;
		    }

		    ihfc_fifo_inc_fx(sc,f);

		    f->program_state = ST_0_FIFO_SEL;
		    return PROGRAM_LOOP;
		}
	    }
	    else
	    {
	        /* NON-HDLC mode
		 *
		 * nothing to do
		 */
	    }

	    /* wait for more data or frame end */

	    f->program_state = ST_0_FIFO_SEL;
	    return PROGRAM_DONE;


 /* FIFO RESET */


	default:
	case ST_3_FIFO_SEL_RST:
	    /* clear all ST_'s that are set by the
	     * program; And ST_FZ_LOADED, so that the
	     * driver will reload all F- and Z-
	     * counters
	     */
	    f->state &= ~(ST_PROGRAM_MASK|ST_FZ_LOADED|ST_FRAME_ERROR|ST_FRAME_END);

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_3_FIFO_SEL_RST;
	        return PROGRAM_SLEEP;
	    }

	    /* select FIFO */
	    FIFO_SELECT(sc,f);

	case ST_4_FIFO_FZ_READ:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_4_FIFO_FZ_READ;
	        return PROGRAM_SLEEP;
	    }

	    ihfc_fifo_fz_read(sc,f);

	    /* set ST_FRAME_ERROR to
	     * cancel current frame, if any
	     *
	     * ST_FRAME_END should not be set here!
	     */
	    f->state |= ST_FRAME_ERROR;

	    /* no data transfer for
	     * next frame
	     */
	    f->Z_chip = 0;

	    /* call filter
	     *
	     * TX: a zero frame may be lost
	     * RX: a zero frame may be received if
	     *     ST_FRAME_END is set
	     */
	    (f->filter)(sc,f);

	    f->state &= ~(ST_FRAME_ERROR|ST_FRAME_END);

	    if(f->F_chip)
	    {
	        if(PROT_IS_HDLC(f->prot) == 0)
		{
		    IHFC_ERR("(#%d) F_drvr != F_chip in [extended] "
			     "transparent mode!\n",
			     FIFO_NO(f));

		    /* in [extended] transparent mode one
		     * cannot receive any frames, so the
		     * frame counter difference must be 
		     * forced to zero by a chip reset.
		     *
		     * reset chip, all FIFOs and this program
		     */
		    ihfc_reset(sc, 0);

		    /* suspend queue until next T50, hence
		     * the FIFOs are empty after reset; And
		     * to avoid program loops:
		     */

		    f->program_state = -1;
		    return PROGRAM_SLEEP;
		}
	    }
	}

	f->program_state = ST_0_FIFO_SEL;
	return PROGRAM_LOOP;
}

/*---------------------------------------------------------------------------*
 * : generic HFC-XXX transmit program (HDLC & TRANS)
 *---------------------------------------------------------------------------*/
u_int8_t
i4b_hfc_tx_program(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	enum {
	  /* wait states, must be 
	   * the same as in 
	   * the next program !
	   */

	  ST_0_FIFO_SEL,
	  ST_1_FIFO_FZ_READ,
	  ST_2_FIFO_INC_FX,

	  ST_3_FIFO_SEL_RST,
	  ST_4_FIFO_FZ_READ,
	  ST_5_FIFO_INC_FX,
        };

	switch(f->program_state)
	{
	case ST_0_FIFO_SEL:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        return PROGRAM_SLEEP;
	    }

	    /* select FIFO */
	    FIFO_SELECT(sc,f);

	case ST_1_FIFO_FZ_READ:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_1_FIFO_FZ_READ;
	        return PROGRAM_SLEEP;
	    }

	    ihfc_fifo_fz_read(sc,f);

/*
 * the following definition defines the maximum
 * number of frames that can be buffered at a
 * time. It is not recommended to change this
 * value, hence one might have to change
 * the interrupt intervals in the drivers
 * to avoid transmit data underflow too.
 *
 *
 * maximum interrupt interval in bytes =
 * (Zsize / HFC_MAX_FRAMES)
 *
 * Fsize must be greater than or equal 
 * to HFC_MAX_FRAMES, in all FIFO maps
 *
 */
#define HFC_MAX_FRAMES 2

	    /* limit number of frames in transmit FIFO */
	    if(f->F_chip >= HFC_MAX_FRAMES)
	    {
	        IHFC_MSG("(#%d) FIFO full!\n",
			 FIFO_NO(f));

	        f->program_state = ST_0_FIFO_SEL;
		return PROGRAM_DONE;
	    }

	    if(f->F_chip == 0)
	    {
	        /* first frame */

	        if(f->Z_chip == 0)
		{
		    /* transmit data underflow */
		    f->state |= ST_FRAME_ERROR;
		    f->Z_chip = f->fm.h.Zsize;
		}

		/* 1) reserve FIFO space so that the 
		 * response times are kept low!
		 *
		 * 2) reserve at least one byte 
		 * due to simple ringbuffer logic
		 */
		if((f->Z_chip -= (f->Z_min_free+1)) & Z_MSB)
		{
		    IHFC_ERR("(#%d) Data overflow or "
			     "underflow!\n", FIFO_NO(f));

		    f->program_state = ST_0_FIFO_SEL;
		    return PROGRAM_DONE;
		}
	    }
	    else
	    {
	        /* get free space in FIFO
		 * for ``next frame'' (HDLC mode)
		 */
	        f->Z_chip = f->Z_chip3;
	    }

	    /* call filter */
	    (f->filter)(sc,f);

	    /* update free space in FIFO
	     * for ``next frame'' (HDLC mode)
	     */
	    f->Z_chip3 = f->Z_chip;

	    /* ST_FRAME_ERROR may not be valid any more */
	    f->state &= ~(ST_FRAME_ERROR);

	    if(PROT_IS_HDLC(f->prot))
	    {
	        /* HDLC mode
		 *
		 * check if ST_FRAME_END is set
		 */
	        if(f->state & ST_FRAME_END)
		{
		    /* prepare incrementation of F-counter
		     * (T125 is checked by ihfc_fifo_fz_read)
		     */
		    FIFO_INC_FX_PRE(sc,f);

	case ST_2_FIFO_INC_FX:

		    /* check T125 */
		    if(SC_T125_WAIT(sc))
		    {
		        f->program_state = ST_2_FIFO_INC_FX;
		        return PROGRAM_SLEEP;
		    }

		    ihfc_fifo_inc_fx(sc,f);

		    f->program_state = ST_0_FIFO_SEL;
		    return PROGRAM_LOOP;
		}
	    }
	    else
	    {
		/* NON-HDLC mode */
	        FIFO_WRITE_FILLER(sc,f);
	    }

	    f->program_state = ST_0_FIFO_SEL;
	    return PROGRAM_DONE;


 /* FIFO RESET */


	default:
	case ST_3_FIFO_SEL_RST:
	    /* clear all ST_'s that are set by the
	     * program; And ST_FZ_LOADED, so that the
	     * driver will reload all F- and Z-
	     * counters
	     */
	    f->state &= ~(ST_PROGRAM_MASK|ST_FZ_LOADED|ST_FRAME_ERROR|ST_FRAME_END);

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_3_FIFO_SEL_RST;
	        return PROGRAM_SLEEP;
	    }

	    /* select FIFO */
	    FIFO_SELECT(sc,f);

	case ST_4_FIFO_FZ_READ:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_4_FIFO_FZ_READ;
	        return PROGRAM_SLEEP;
	    }

	    ihfc_fifo_fz_read(sc,f);

	    /* set ST_FRAME_ERROR to
	     * retransmit current frame, if any
	     *
	     * ST_FRAME_END should not be set here!
	     */
	    f->state |= ST_FRAME_ERROR;

	    /* no data transfer for
	     * next frame
	     */
	    f->Z_chip = 0;

	    /* call filter
	     *
	     * TX: a zero frame may be lost
	     * RX: a zero frame may be received if
	     *     ST_FRAME_END is set
	     */
	    (f->filter)(sc,f);

	    f->state &= ~(ST_FRAME_ERROR|ST_FRAME_END);

	    if(f->F_chip)
	    {
	        if(!(PROT_IS_HDLC(f->prot)))
		{
		    IHFC_ERR("(#%d) F_drvr != F_chip in [extended] "
			     "transparent mode!\n",
			     FIFO_NO(f));

		    /* in [extended] transparent mode one
		     * cannot transmit any frames, so the
		     * frame counter difference must be 
		     * forced to zero by a chip reset.
		     *
		     * reset chip, all FIFOs and this program
		     */
		    ihfc_reset(sc, 0);

		    /* suspend queue until next T50, hence
		     * the FIFOs are empty after reset; And
		     * to avoid program loops:
		     */

		    f->program_state = -1;
		    return PROGRAM_SLEEP;
		}
	    }

	    if(PROT_IS_HDLC(f->prot))
	    {
	        /* increment the F-counter so that 
		 * one is sure that an old frame is
		 * not continued and to make sure
		 * that Z_chip3 is initialized
		 */

	case ST_5_FIFO_INC_FX:

		/* check T125 */
		if(SC_T125_WAIT(sc))
		{
		    f->program_state = ST_5_FIFO_INC_FX;
		    return PROGRAM_SLEEP;
		}

		ihfc_fifo_inc_fx(sc,f);
	    }
	    else
	    {
	        static const u_int8_t temp = 0xFF;

		/* in [extended] transparent mode, 
		 * send one 0xFF byte in
		 * case the transmit channel
		 * is shared
		 */
		FIFO_WRITE_MULTI_1(sc,f,&temp,1);
	    }
	}

	f->program_state = ST_0_FIFO_SEL;
	return PROGRAM_LOOP;
}

/*---------------------------------------------------------------------------*
 * : generic HFC-XXX transmit program (HDLC & TRANS)
 *---------------------------------------------------------------------------*/
u_int8_t
i4b_hfc_tx_program_new(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	enum {
	  /* wait states, 
	   * must be the same 
	   * as in the program above !
	   */

	  ST_0_FIFO_SEL,
	  ST_1_FIFO_FZ_READ,
	  ST_2_FIFO_INC_FX,

	  ST_3_FIFO_SEL_RST,
	  ST_4_FIFO_FZ_READ,
	  ST_5_FIFO_INC_FX,
        };

	switch(f->program_state)
	{
	case ST_0_FIFO_SEL:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        return PROGRAM_SLEEP;
	    }

	    /* select FIFO */
	    FIFO_SELECT(sc,f);

	case ST_1_FIFO_FZ_READ:

	    /* check T125 */
	    if(SC_T125_WAIT(sc))
	    {
	        f->program_state = ST_1_FIFO_FZ_READ;
	        return PROGRAM_SLEEP;
	    }

	    ihfc_fifo_fz_read(sc,f);

	    /* limit number of frames in transmit FIFO */
	    if((f->F_chip+1) >= f->fm.h.Fsize)
	    {
	        IHFC_MSG("(#%d) FIFO full!\n",
			 FIFO_NO(f));

	        f->program_state = ST_0_FIFO_SEL;
		return PROGRAM_DONE;
	    }

	    if(f->Z_chip == 0)
	    {
	        /* transmit data underflow */
	        f->state |= ST_FRAME_ERROR;
		f->Z_chip = f->fm.h.Zsize;
	    }

	    /* need to reserve one byte due to simple
	     * ringbuffer logic
	     */
	    if((f->Z_chip -= (f->Z_min_free+1)) & Z_MSB)
	    {
	        IHFC_ERR("(#%d) Data overflow or "
			 "underflow!\n", FIFO_NO(f));

		f->program_state = ST_0_FIFO_SEL;
		return PROGRAM_DONE;
	    }

	    /* call filter */
	    (f->filter)(sc,f);

	    /* ST_FRAME_ERROR may not be valid any more */
	    f->state &= ~(ST_FRAME_ERROR);

	    if(PROT_IS_HDLC(f->prot))
	    {
	        /* HDLC mode
		 *
		 * check if ST_FRAME_END is set
		 */
	        if(f->state & ST_FRAME_END)
		{
		    ihfc_fifo_inc_fx(sc,f);

		    f->program_state = ST_0_FIFO_SEL;
		    return PROGRAM_LOOP;
		}
	    }
	    else
	    {
	        /* NON-HDLC mode */
	        FIFO_WRITE_FILLER(sc,f);
	    }

	    f->program_state = ST_0_FIFO_SEL;
	    return PROGRAM_DONE;

	default:

	    /* reuse code */

	    return i4b_hfc_tx_program(sc, f);
	}

	f->program_state = ST_0_FIFO_SEL;
	return PROGRAM_LOOP;
}

u_int8_t
i4b_unknown_program(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	/* this program is used when the driver calls
	 * f->filter(,) directly
	 */

	if(f->prot != P_DISABLE)
	{
	    IHFC_MSG("(#%d) << unsupported!\n", FIFO_NO(f));
	}
	return PROGRAM_DONE;
}

static void
unsupported FIFO_FILTER_T(sc,f)
{
	if(f->prot != P_DISABLE)
	{
	    IHFC_MSG("(FIFO #%d) << no filter!\n",FIFO_NO(f));
	}
	return;
}

/*---------------------------------------------------------------------------*
 * : fifo program reset
 *---------------------------------------------------------------------------*/
static void
ihfc_fifo_program_reset(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	/* f->filter might be called outside
	 * ihfc_fifo_program(), and must
	 * be initialized here:
	 */

	if(f->filter == NULL)
	{
		/* set default filter
		 * if not initialized
		 */
		f->filter = &unsupported;
	}

	if(f->program == NULL)
	{
		/* set default program,
		 * if not initialized
		 */
		f->program = &i4b_unknown_program;
	}

	/* reset program state */
	f->program_state = -1;

	/* queue FIFO so that reset
	 * program will be run at
	 * next interrupt
	 */
	QFIFO(sc,lo,FIFO_NO(f));

	return;
}

#endif /* _I4B_PROGRAM_H_ */
