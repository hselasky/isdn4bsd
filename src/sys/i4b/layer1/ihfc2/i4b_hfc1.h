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
 *	i4b_hfc1.h - HFC-2B driver module
 * 	---------------------------------
 *
 *	last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFC1_H_
#define _I4B_HFC1_H_

#include <i4b/layer1/ihfc2/i4b_hfc.h>
#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */

#define hfc1_chip_status_check     default_chip_status_check
#define hfc1_fsm_table             isac_fsm_table

/* driver */

#ifndef __i386__
#if 0
#warning "8-bit  I/O access delay must be about 1us."
#warning "16-bit I/O access order must be lower byte first, higher byte last."
#endif

/* XXX can we trust the write order of non i386 CPU's ? */

#undef bus_space_write_2
#define bus_space_write_2 i4b_bus_space_write_2

static __inline void 
i4b_bus_space_write_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t off, u_int16_t val)
{
	bus_space_write_1(t, h, off, (u_int8_t)(val));
	off++;
	bus_space_write_1(t, h, off, (u_int8_t)(val >> 8));
	return;
}

#undef bus_space_read_2
#define bus_space_read_2 i4b_bus_space_read_2

static __inline u_int16_t
i4b_bus_space_read_2(bus_space_tag_t t, bus_space_handle_t h, bus_size_t off)
{
	register u_int16_t temp;
	temp = bus_space_read_1(t, h, off);
	off++;
	temp |= bus_space_read_1(t, h, off) << 8;
	return temp;
}

#endif

/*
 * hfc1 related methods
 *
 * This chip uses two io-addresses.  One to write
 * the register index and one to read/write  data
 * to the selected register. The register IO-port
 * is also used to read DISBUSY,      where bit 0
 * indicates if the HFC-2B  chip  can be accessed
 * or not.     Setting some register indexes have
 * special meaning like card reset or timer reset.
 *
 *
 * A typical HFC-2B card consists of 8 or 32kbyte
 * SRAM, a HFC-2B chip and a ISAC-TE chip.
 *
 * iobase + 1    :
 * (t,h,1,reg)   : set register or read DISBUSY
 *
 * iobase + 0    :
 * (t,h,0,...)   : read/write HFC-2B or ISAC
 *               : ISAC registers at 0x00..0x1f
 *               : HFC-2B fifo registers starts
 *		   at 0x80.
 *
 * BUG NOTE: ISAC's SIN does not generate 125us interrupts
 * properly.   Instead of 125us the delay can be more than
 * 50ms, which breaks the data stream. Reason unknown.
 */

#define HFC1_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

static void
hfc1_t125_sync FIFO_SYNC_T(sc)
{
#if 0
	HFC1_BUS_VAR(sc);
	u_int8_t to = 255; /* ~255us max */

	/* disable interrupts */
	cli(sc);

	/* select dummy register
	 * read STAR (reg=0x21,ISAC)
	 */
	bus_space_write_1(t,h,1, 0x21);

	/* DISBUSY */
	while(!to--)
	{
	  if(!(bus_space_read_1(t,h,1) & 1))
	  {
	    /* WAITBUSY */
	    while(!(bus_space_read_1(t,h,1) & 1))
	    {
	      /* must read data after DISBUSY */
	      bus_space_read_1(t,h,0);
	      DELAY(2);

	      if(!to--) goto error;
	    }
	    goto done;
	  }
	  DELAY(2);
	}

 error:
	IHFC_ERR("T125 timeout!\n");

	/* wait for T50 - chip is not running */
	SC_T125_WAIT_SET(sc);

 done:
	/* enable interrupts */
	sti(sc);
#else
	/* this code assumes that the
	 * F0IO clock is always running ...
	 */

	DELAY(200);
#endif
	return;
}

static void
hfc1_chip_reset CHIP_RESET_T(sc,error)
{
	HFC1_BUS_VAR(sc);

	/* enable internal I/O */
	bus_space_write_2(t,h,0,((sc->sc_resources.iio[0] & 0x3ff) | 0x5400));

	/* perform reset (will also reset ISAC co-chip)
	 *
	 * CIRM write with reset=1(enabled):
	 *
	 * The HFC-2B manual recommends a
	 * 4 clock-cycles delay. When
	 * the clock is 7.68mHz, a 1us
	 * delay should	do.
	 *
	 * CIRM write with reset=0 after
	 * CIRM write with reset=1:
	 *
	 * The ISAC manual recommends a 
	 * 125us delay for the chip
	 * to configure.
	 *
	 * Too short delays cause chip
	 * corruption and IRQ failure.
	 *
	 * A 500us delay should do for both
	 * cases.
	 */
	bus_space_write_1(t,h,1,(sc->sc_config.s_cirm_0 ^ 0x08));
	DELAY(500);

	bus_space_write_1(t,h,1,(sc->sc_config.s_cirm_0));
	DELAY(500);

	/* select SPCR(reg=0x30) */
	bus_space_write_1(t,h,1,0x30);

	/* read DISBUSY */
	if(bus_space_read_1(t,h,1) & 0x01)
	{
	  /* if the HFC-2B chip
	   * was reset during
	   * operation BUSY might
	   * be high longer than
	   * 500us ...
	   */
	  IHFC_MSG("(BUSY != 0)\n");
	}

	/* read SPCR */
	if(bus_space_read_1(t,h,0) & 0xff)
	{
		IHFC_ADD_ERR(error,"(SPCR != 0)");
	}

	/* select MODE(reg=0x22) */
	bus_space_write_1(t,h,1,0x22);

	/* read mode */
	if(bus_space_read_1(t,h,0) & 0xff)
	{
		IHFC_ADD_ERR(error,"(MODE != 0)");
	}

	return;
}

static void
hfc1_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
  HFC1_BUS_VAR(sc);

  /* NOTE: two configuration registers
   * have been put here, because they
   * are not mappable, and will only
   * be updated after ``fifo_setup(..)''
   * or ``ihfc_fsm_update(..)''
   */

  if((f == CONFIG_WRITE_UPDATE) ||
     (f == CONFIG_WRITE_RELOAD))
  {
    bus_space_write_1(t,h,1, sc->sc_config.s_cirm_0);
    bus_space_write_1(t,h,1, sc->sc_config.s_ctmt_0);
  }
  else
  {
    /* nothing to configure for fifo */
  }

  return;
}

static void
hfc1_reload_fz (ihfc_sc_t *sc);

static void
hfc1_reload_fz (ihfc_sc_t *sc)
{
  ihfc_fifo_t *f;

  FIFO_FOREACH(f,sc)
  {
      f->state &= ~ST_FZ_LOADED;
  }

  return;
}

static void
hfc1_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	bus_space_write_1(t,h,1,(reg));

	/* disable interrupts */
	cli(sc);

	tmp = 0x100;

	while(len--) {
		DISBUSY(tmp,tmp,
		{
			IHFC_ERR("Disbusy timeout!\n");
			/*
			 * NOTE: when read fails Z_drvr
			 *       must be reloaded from
			 *       hardware.
			 */
			hfc1_reload_fz(sc);
			bzero(ptr,++len);
			break;
		});

		if(len) {
		  *ptr++ = (tmp = bus_space_read_2(t,h,0));
		} else {
		  *ptr++ = bus_space_read_1(t,h,0);
		}
	}

	/* enable interrupts */
	sti(sc);

	return;
}

static void
hfc1_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	/* pre increment Z-counter
	 * NOTE: ISAC(chan=d1) does not use f->Z_drvr
	 *       although it is updated.
	 */
	if(((f->Z_drvr)   += (len)) >= (f->fm.h.Zend)) {
	    (f->Z_drvr)   -= (f->fm.h.Zsize);
	}

	hfc1_chip_read(sc,(f->fm.h.Zdata),ptr,len);

	return;
}

static void
hfc1_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	bus_space_write_1(t,h,1, (reg));

	/* disable interrupts */
	cli(sc);

	while(len--) {
		if(bus_space_read_1(t,h,1) & 1) {
			DISBUSY(-1,tmp,
			{
			  IHFC_ERR("Disbusy timeout!\n");
			  /*
			   * NOTE: when write fails Z_drvr
			   *       must be reloaded from
			   *       hardware.
			   */
			  hfc1_reload_fz(sc);
			  break;
			});
		}

		bus_space_write_1(t,h,0,(*ptr++));
	}

	/* enable interrupts */
	sti(sc);

	return;
}

static void
hfc1_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	/* pre increment Z-counter
	 * NOTE: ISAC(chan=d1) does not use f->Z_drvr
	 *       although it is updated.
	 */
	if(((f->Z_drvr)   += (len)) >= (f->fm.h.Zend)) {
	    (f->Z_drvr)   -= (f->fm.h.Zsize);
	}

	hfc1_chip_write(sc,(f->fm.h.Zdata),ptr,len);

	return;
}

static void
hfc1_fsm_read FSM_READ_T(sc,f,ptr)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	/* disable interrupts */
	cli(sc);

	/* READ ISAC CI-state (reg=0x31) */
	bus_space_write_1(t,h,1,0x31);
	DISBUSY(-1,tmp,);
	*ptr = (bus_space_read_1(t,h,0) >> 2) & 0xf;

	/* enable interrupts */
	sti(sc);

	return;
}

static void
hfc1_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	/* disable interrupts */
	cli(sc);

	bus_space_write_1(t,h,1,0x31);
	DISBUSY(-1,tmp,);
	bus_space_write_1(t,h,0,
			  (*ptr | sc->sc_config.i_cirq));

	/* enable interrupts */
	sti(sc);

	return;
}

static void
hfc1_fifo_select FIFO_SELECT_T(sc,f)
{
	HFC1_BUS_VAR(sc);

	if(sc->sc_fifo_select_last != f)
	{  sc->sc_fifo_select_last = f;

	  /* select fifo
	   * 
	   * NOTE: fifo is selected
	   * after next busy period,
	   * F0IO, ~125us.
	   */
	  bus_space_write_1(t,h,1, f->fm.h.Zbase);

	  hfc1_t125_sync(sc);
	}

	return;
}

static void
hfc1_fifo_inc_fx_pre FIFO_INC_FX_PRE_T(sc,f)
{
	/* If fx counter is incremented it must
	 * be in a separate busy/non-busy
	 * period.
	 */

	hfc1_t125_sync(sc);

	return;
}

static void
hfc1_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	/* increment drvr's
	 * F-counter by one.
	 * NOTE: fifo must be
	 * selected first.
	 */

	/* update soft counter */
	if(++(f->F_drvr) & 0x20) {
	     (f->F_drvr) -= (f->fm.h.Fsize);
	}

	/* disable interrupts */
	cli(sc);

	bus_space_write_1(t,h,1, f->fm.h.Fibase);
	DISBUSY(-1,tmp,);
	bus_space_read_1(t,h,0);

	/* enable interrupts */
	sti(sc);

	/* NOTE: in some special cases the
	 * Z_drvr counter may be changed
	 * after a F_drvr increment.
	 * Reload FZ:
	 */

	f->state &= ~ST_FZ_LOADED;

	hfc1_t125_sync(sc);

	return;
}

static void
hfc1_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	u_int16_t z0, Z_chip2;
	HFC1_BUS_VAR(sc);

	/* read F- and Z-counters respectively
	 *
	 * NOTE: interrupts are disabled to
	 * run this code at maximum speed.
	 */

	/* disable interrupts */
	cli(sc);

	/* fx */
	bus_space_write_1(t,h,1, f->fm.h.Fbase ^0);
	DISBUSY       (-1,z0,);
	              (z0) = bus_space_read_2(t,h,0);
	(f->F_chip) = (z0);

	/* zx - high (first read) */
	bus_space_write_1(t,h,1, f->fm.h.Zbase ^4);
	DISBUSY        (z0,z0,);
	               (z0) = bus_space_read_2(t,h,0);
	(   Z_chip2) = (z0) << 8;

	/* zx - low */
	bus_space_write_1(t,h,1, f->fm.h.Zbase ^0);
	DISBUSY       (z0,z0,);
	              (z0) = bus_space_read_2(t,h,0);
	(f->Z_chip) = (z0) & 0xff;

	/* NOTE: it is only necessary to
	 * read F_drvr and Z_drvr once
	 * from the hardware:
	 */
	if(!(f->state &  ST_FZ_LOADED)) {
	     f->state |= ST_FZ_LOADED;
		/* fx */
		bus_space_write_1(t,h,1, f->fm.h.Fbase ^4);
		DISBUSY       (z0,z0,);
		              (z0) = bus_space_read_2(t,h,0);
		(f->F_drvr) = (z0);

		/* zx - low */
		bus_space_write_1(t,h,1, f->fm.h.Zbase ^8);
		DISBUSY       (z0,z0,);
		              (z0) = bus_space_read_2(t,h,0);
		(f->Z_drvr) = (z0) & 0xff;

		/* zx - high */
		bus_space_write_1(t,h,1, f->fm.h.Zbase ^12);
		DISBUSY       (z0,z0,);
		              (z0) = bus_space_read_2(t,h,0);
		(f->Z_drvr)|= (z0) << 8;
	}

	/* zx - high (second read)
	 * 
	 * NOTE: to save a disbusy
	 * read when  ST_FZ_LOADED
	 * is zero, this register
	 * read has been put last.
	 */
	bus_space_write_1(t,h,1, f->fm.h.Zbase ^4);
	DISBUSY       (z0,z0,);
	              (z0) = bus_space_read_1(t,h,0);
	(f->Z_chip)|= (z0) << 8;
 
	/* enable interrupts */
	sti(sc);

	/* remove extra [undefined] bits and
	 * add bit 4 to D-channel F_chip and F_drvr
	 */
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* D-channel */
	  (f->F_chip)  &=  0x001f;
	  (f->F_chip)  |=  0x0010; /* exception */
	  (f->F_drvr)  &=  0x001f;
	  (f->F_drvr)  |=  0x0010; /* exception */

	  (f->Z_chip)  &=  0x01ff;
	  (   Z_chip2) &=  0x01ff; /* copy of f->Z_chip */
	  (f->Z_drvr)  &=  0x01ff;
	}
	else
	{
	  /* B-channel */
	  (f->F_chip)  &=  0x001f;
	  (f->F_drvr)  &=  0x001f;

	  (f->Z_chip)  &=  0x1fff;
	  (   Z_chip2) &=  0x1fff; /* copy of f->Z_chip */
	  (f->Z_drvr)  &=  0x1fff;
	}

	/* analyse/correct the counters */
	INC_COUNTER_TIME_CHECK(f->Z_chip,    Z_chip2);

	return;
}

static void
hfc1_chip_unselect CHIP_UNSELECT_T(sc)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	/* disable interrupts */
	cli(sc);

	/* write CMDR (ISAC) */
	if(sc->sc_fifo[d1r].i_cmdr |=
	   sc->sc_fifo[d1t].i_cmdr)
	{
#if 0
		/* disable TIN hence T50 is used instead */
		sc->sc_fifo[d1r].i_cmdr &= ~0x10;
#endif
		/* write CMDR (reg=0x21,ISAC) */
		bus_space_write_1(t,h,1, 0x21);
		DISBUSY(-1,tmp,);
		bus_space_write_1(t,h,0, sc->sc_fifo[d1r].i_cmdr);

		sc->sc_fifo[d1r].i_cmdr =
		sc->sc_fifo[d1t].i_cmdr = 0;
	}

	/* take advantage of delay in resetting
	 * all MASKS, to make sure that
	 *
	 * 1) there is a 5-8us delay after
	 * CMDR write, to prevent too early
	 * ISTA interrupts, like XPR.
	 *
	 * 2) that the output IRQ reaches
	 * a low level, because the chip
	 * ors all input IRQ sources:
	 *
	 * Problem:
	 * ========
	 * external IRQ: ____.------------
	 *      t50 IRQ: ---------._______
	 *
	 *   output IRQ: -----------------
	 *
	 * Solution (disable all IRQ's for a
	 *           short period of time):
	 * =================================
	 * external IRQ: ____.---.____.---
	 *      t50 IRQ: ---------._______
	 *
	 *   output IRQ: ---------.___.---
	 */

	/* disable external IRQ
	 * write MASK (reg=0x20,ISAC)
	 */
	bus_space_write_1(t,h,1,0x20);
	DISBUSY(-1,tmp,);
	bus_space_write_1(t,h,0,0xff);

	/* disable t50 */
	bus_space_write_1(t,h,1, (sc->sc_config.s_ctmt_0 & ~0x04));

	/* enable t50 */
	bus_space_write_1(t,h,1, (sc->sc_config.s_ctmt_0));

	/* enable external IRQ
	 * write MASK (reg=0x20,ISAC)
	 */
	bus_space_write_1(t,h,1,0x20);
	DISBUSY(-1,tmp,);
	bus_space_write_1(t,h,0,sc->sc_config.i_mask);

	/* enable interrupts */
	sti(sc);

	return;
}

static void
hfc1_chip_status_read CHIP_STATUS_READ_T(sc)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	/* disable interrupts */
	cli(sc);

	/* read status/DISBUSY */
	tmp = bus_space_read_1(t,h,1);

	/* check t50 */
	if(tmp & 0x04)
	{
	  /* clear ``sc_t125_wait'' */
	  SC_T125_WAIT_CLEAR(sc);

	  /* emulate HFC-2BDS0 chip */
	  sc->sc_config.s_int_s1 |= 0x80;

	  /* reset t50 interrupt */
	  bus_space_write_1(t,h,1, (sc->sc_config.s_ctmt_0 ^ 0x10));
	}

	/* read ISTA(reg=0x20,ISAC) */
	bus_space_write_1(t,h,1,0x20);
	DISBUSY(-1,tmp,);
	sc->sc_config.i_ista |= bus_space_read_1(t,h,0);

	/* check for RME (D - channel) */
	if(sc->sc_config.i_ista & 0x80)
	{
	  /* read RBCL (reg=0x25, ISAC) */
	  bus_space_write_1(t,h,1, 0x25);
	  DISBUSY                (-1,tmp,);
	                            (tmp) = bus_space_read_2(t,h,0);
	  sc->sc_fifo[d1r].Z_chip = (tmp) & 0xff;

	  /* read RSTA (reg=0x27, ISAC) */
	  bus_space_write_1(t,h,1, 0x27);
	  DISBUSY               (tmp,tmp,);
	                            (tmp) = bus_space_read_1(t,h,0);
	  sc->sc_fifo[d1r].F_chip = (tmp) & 0xff;
	}

	/* check for EXIR */
	if(sc->sc_config.i_ista & 0x01)
	{
		/* read EXIR */
		bus_space_write_1(t,h,1,0x24);
		DISBUSY(-1,tmp,);
		sc->sc_config.i_exir |= bus_space_read_1(t,h,0);
	}

	/*
	 * i_ista and i_exir should be cleared
	 * by the caller of this function
	 * sc->sc_config.i_ista &= ~0x03;
	 */

	/* enable interrupts */
	sti(sc);

	return;
}

static u_int8_t
hfc1_fifo_frame_check FIFO_FRAME_CHECK_T(sc,f,m)
{
  if(FIFO_CMP(f,<,(b1t & b1r)))
  {
	return 0; /* valid frame */
  }
  else
  {
	return hfcs_fifo_frame_check(sc,f,m);
  }
}

static ihfc_fifo_program_t *
hfc1_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if((FIFO_NO(f) == d1t) ||
	   (FIFO_NO(f) == d1r))
	{
	  if(PROT_IS_HDLC(&(f->prot_curr)))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_ipac_tx_program :
			&i4b_ipac_rx_program;
	  }
	}
	else
	{
	  if(HFC_USE_HARDWARE_HDLC(PROT_IS_HDLC(&(f->prot_curr)) ||)
	     PROT_IS_TRANSPARENT(&(f->prot_curr)))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_hfc_tx_program :
			&i4b_hfc_rx_program;
	  }
	}
	return program;
}

static int
hfc1_chip_identify(device_t dev)
{
	struct resource * res;
	u_int32_t rid;
	u_int32_t base;
	u_int8_t irq_list[] = { 3, 4, 5, 7, 10, 11, 0 };
	u_int8_t *irq = &irq_list[0];

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0/* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "iobase!\n", __FUNCTION__);
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	switch(base) {
	case 0x300:
	case 0x330:
	case 0x278:
	case 0x2e8:
		break;

	default:
		device_printf(dev, 
			      "%s: ERROR: invalid iobase 0x%x!\n",
			      __FUNCTION__, base);
		return ENXIO;
	}

	/* setup I/O-ports */

	bus_set_resource(dev, SYS_RES_IOPORT, 0, base, 2);

	rid = 0;

	while(!(res = bus_alloc_resource_any
		(dev, SYS_RES_IRQ, &rid, 0 /* !RF_ACTIVE */)))
	{
		if(*irq)
		{
			bus_set_resource(dev, SYS_RES_IRQ, 0, *irq, 1);
			rid = 0;
			irq++;
		}
		else
		{
			device_printf(dev, "%s: ERROR: could not get "
				      "an IRQ!\n", __FUNCTION__);
			return ENXIO;
		}
	}

	/* get IRQ */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IRQ, rid, res);

	/* check IRQ validity */

	switch(base) {
	case 3:
	case 4:
	case 5:
	case 7:
	case 10:
	case 11:
		break;
			
	default:
		device_printf(dev, "%s: ERROR: invalid IRQ [%d]!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}
	return(0);
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfc1_dbase_root)
{
  I4B_DBASE_ADD(c_chip_identify        , &hfc1_chip_identify);
  I4B_DBASE_ADD(c_chip_read            , &hfc1_chip_read);
  I4B_DBASE_ADD(c_chip_write           , &hfc1_chip_write);
  I4B_DBASE_ADD(c_chip_reset           , &hfc1_chip_reset);
  I4B_DBASE_ADD(c_chip_config_write    , &hfc1_chip_config_write);
  I4B_DBASE_ADD(c_chip_unselect        , &hfc1_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read     , &hfc1_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check    , &hfc1_chip_status_check);

  I4B_DBASE_ADD(c_fsm_read             , &hfc1_fsm_read);
  I4B_DBASE_ADD(c_fsm_write            , &hfc1_fsm_write);
  I4B_DBASE_ADD(d_fsm_table            , &hfc1_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program     , &hfc1_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read            , &hfc1_fifo_read);
  I4B_DBASE_ADD(c_fifo_write           , &hfc1_fifo_write);
  I4B_DBASE_ADD(c_fifo_select          , &hfc1_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx          , &hfc1_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read         , &hfc1_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre      , &hfc1_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_frame_check     , &hfc1_fifo_frame_check);

  I4B_DBASE_ADD(d_register_list        , &hfc1_register_list[0]);
  I4B_DBASE_ADD(d_channels             , 6);

  /* delay 16 milliseconds (due to ISAC,
   * else 50 milliseconds)
   */
  I4B_DBASE_ADD(d_interrupt_delay      , hz / 62);

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2        , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC                 , 0); /* disable */
  I4B_DBASE_ADD(o_RES_IRQ_0            , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0         , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask        , (I4B_OPTION_POLLED_MODE|
					  I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value       , 0);

#if 1   /* if (sc->sc_config.s_cirm_0 & 0x10) */

  I4B_DBASE_ADD(o_8KFIFO               , 1); /* enable */

  /*
   * 8kbyte fifo mode selected
   */
    
  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(isac_fifo_map[ 8]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(isac_fifo_map[ 8]));

  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[ 4]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[ 5]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[ 6]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[ 7]));

#else

  I4B_DBASE_ADD(o_8KFIFO               , 0); /* disable */

  /*
   * 32kbyte fifo mode selected
   */
  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(isac_fifo_map[ 8]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(isac_fifo_map[ 8]));
    
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[ 8]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[ 9]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[10]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[11]));

#endif
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  static const struct internal list_iio[]  =
  {
    { .value = 0x300, .internal = 0x300 },
    { .value = 0x330, .internal = 0x330 },
    { .value = 0x278, .internal = 0x278 },
    { .value = 0x2e8, .internal = 0x2e8 },
    { .value = 0x000, .internal = 0x000 }, /* not possible */
  };

  static const struct internal list_iirq[]  =
  {
    { .value =  3, .internal =  1 },
    { .value =  4, .internal =  2 },
    { .value =  5, .internal =  3 },
    { .value =  7, .internal =  4 },
    { .value = 10, .internal =  5 },
    { .value = 11, .internal =  6 },
    { .value =  0, .internal =  0 }, /* IRQ disabled */
  };

  I4B_DBASE_IMPORT(hfc1_dbase_root);

  I4B_DBASE_ADD(desc          , "HFC-2B ISDN card (TELEINT ISDN SPEED No. 1)");

  I4B_DBASE_ADD(list_iio      , &list_iio[0]);
  I4B_DBASE_ADD(list_iirq     , &list_iirq[0]);
}

I4B_ISA_DRIVER(/* TELEINT ISDN SPEED No. 1 */
	       .vid = 0x0A /* card number */);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  static const struct internal list_iirq[]  =
  {
    { .value = 0, .internal = 0 } /* IRQ disabled */
  };

  I4B_DBASE_IMPORT(hfc1_dbase_root);

  I4B_DBASE_ADD(desc          , "HFC-2B ISDN card");

  I4B_DBASE_ADD(list_iio      , 0);
  I4B_DBASE_ADD(list_iirq     , &list_iirq[0]);
}

I4B_ISA_DRIVER(/* custom HFC1 */
	       .vid = 0x01 /* card number */);

#endif /* _I4B_HFC1_H_ */
