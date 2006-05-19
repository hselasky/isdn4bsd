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
 *	i4b_hfcs.h - HFC-2BDS0 driver module
 * 	------------------------------------
 *
 *	last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFCS_H_
#define _I4B_HFCS_H_

#include <i4b/layer1/ihfc2/i4b_hfc1.h>

/* imports */

#define hfcs_chip_status_check  default_chip_status_check
#define hfcs_chip_unselect      default_chip_unselect
#define hfcs_fifo_fz_read       hfc1_fifo_fz_read
#define hfcs_fsm_table          hfc_fsm_table

/* driver */

/*-------------------------------------------------+
 | hfcs related methods                            |
 |                                                 |
 | (t,h,1,reg)   : set register or read DISBUSY    |
 | (t,h,0,...)   : read/write HFC-2BDS0            |
 +-------------------------------------------------*/

static void
hfcs_t125_sync FIFO_SYNC_T(sc)
{
	HFC1_BUS_VAR(sc);

	/* enable 125us timeout (sync(R)) */

	/* disable interrupts */
	cli(sc);

	/* write int_m2(reg=0x1b) */
	sc->sc_config.s_int_m2 |= 1;
	bus_space_write_1(t,h,1,0x1b);
	bus_space_write_1(t,h,0,(sc->sc_config.s_int_m2));

	/* reset sync(R) by a int_s1 read */
	bus_space_write_1(t,h,1,0x1e);
	sc->sc_config.s_int_s1 |= bus_space_read_1(t,h,0);

	/* enable interrupts */
	sti(sc);

	SC_T125_WAIT_SET(sc);

	return;
}

static void
hfcs_chip_reset CHIP_RESET_T(sc,error)
{
	HFC1_BUS_VAR(sc);

	/* enable internal io */
	bus_space_write_2(t,h,0,((sc->sc_resources.iio[0] & 0x3ff) | 0x5400));

	/* select cirm */
	bus_space_write_1(t,h,1,0x18);

	/* perform reset
	 *
	 * CIRM write with reset=1:
	 *
	 * The HFC-2BDS0 manual recommends
	 * a 4 clock-cycle delay. When
	 * the clock is 12.288mhz a 1us
	 * delay should do.
	 *
	 * CIRM write with reset=0 after
	 * CIRM write with reset=1:
	 *
	 * The chip should not be accessed
	 * until the busy signal goes
	 * low. Experiments show this delay
	 * should be in the range 8-9us to
	 * avoid chip corruption.
	 *
	 * A 100us delay should do for both
	 * cases.
	 *
	 * write cirm
	 */
	bus_space_write_1(t,h,0, (sc->sc_config.s_cirm ^ 0x08));
	DELAY(100);

	bus_space_write_1(t,h,0, (sc->sc_config.s_cirm));
	DELAY(100);

	/* select states(reg=0x30) */
	bus_space_write_1(t,h,1,0x30);

	/* read disbusy */
	if(bus_space_read_1(t,h,1) & 0x01)
	{
		IHFC_ADD_ERR(error,"(BUSY != 0)");
	}

	/* read states */
	if(bus_space_read_1(t,h,0) & 0x0f)
	{
		IHFC_ADD_ERR(error,"(STATES != 0)");
	}

	return;
}

static void
hfcs_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	/* pre increment Z-counter */
	if(((f->Z_drvr)   += (len)) >= (f->fm.h.Zend)) {
	    (f->Z_drvr)   -= (f->fm.h.Zsize);
	}

	hfc1_chip_read(sc,(f->fm.h.Zdata),ptr,len);

	return;
}

static void
hfcs_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	/* check if register needs
	 * disbusy(STATUS bit0 == 0)
	 */
	if(reg & 0x80) {
	  hfc1_chip_read(sc,reg,ptr,len);
	} else {
	  HFC1_BUS_VAR(sc);

	  bus_space_write_1(t,h,1,(reg));
	  bus_space_read_multi_1(t,h,0,(ptr),(len));
	}

	return;
}

static void
hfcs_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	/* pre increment Z-counter */
	if(((f->Z_drvr)   += (len)) >= (f->fm.h.Zend)) {
	    (f->Z_drvr)   -= (f->fm.h.Zsize);
	}

	hfc1_chip_write(sc,(f->fm.h.Zdata),ptr,len);

	return;
}

static void
hfcs_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	/* check if register needs
	 * disbusy(STATUS bit0 == 0)
	 */
	if(reg & 0x80) {
	  hfc1_chip_write(sc,reg,ptr,len);
	} else {
	  HFC1_BUS_VAR(sc);

	  bus_space_write_1(t,h,1, (reg));
	  bus_space_write_multi_1(t,h,0,(ptr),(len));
	}

	return;
}

static void
hfcs_fsm_read FSM_READ_T(sc,f,ptr)
{
	HFC1_BUS_VAR(sc);

	/* read states(reg=0x30) */
	       bus_space_write_1(t,h,1,0x30);
	*ptr = (bus_space_read_1(t,h,0) +
		((sc->sc_config.s_sctrl & 0x4) ?
		 HFC_NT_OFFSET : HFC_TE_OFFSET)) & 0xf;
	return;
}

static void
hfcs_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	HFC1_BUS_VAR(sc);

	/* write STATES(reg=0x30) */
	bus_space_write_1(t,h,1,0x30);
	bus_space_write_1(t,h,0, (*ptr));

#ifdef HFC_FSM_RESTART
	if((*ptr) & 0x10) {
	    /* 5.21us delay */

	    DELAY(10);

	    bus_space_write_1(t,h,0, ((*ptr) ^ 0x10));
	}
#endif
	return;
}

static void
hfcs_fifo_select FIFO_SELECT_T(sc,f)
{
	HFC1_BUS_VAR(sc);

	if(sc->sc_fifo_select_last != f)
	{  sc->sc_fifo_select_last = f;

	  /* select fifo
	   *
	   * NOTE: fifo is selected
	   * after the end of the next
	   * busy period, F0IO, ~125us.
	   */
	  bus_space_write_1(t,h,1, f->fm.h.Zbase);

	  hfcs_t125_sync(sc);
	}

	return;
}

static void
hfcs_fifo_inc_fx_pre FIFO_INC_FX_PRE_T(sc,f)
{
	/* If fx counter is incremented it must
	 * be in a separate busy/non-busy
	 * period.
	 */

	hfcs_t125_sync(sc);

	return;
}

static void
hfcs_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	register u_int16_t tmp;
	HFC1_BUS_VAR(sc);

	/* increment drvr's F-counter by one.
	 * NOTE: fifo must be selected first.
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

	hfcs_t125_sync(sc);

	return;
}

static void
hfcs_chip_status_read CHIP_STATUS_READ_T(sc)
{
	HFC1_BUS_VAR(sc);

	/* disable interrupts */
	cli(sc);

	/* select int_s1 */
	bus_space_write_1(t,h,1,0x1e);
	/* read status */
	sc->sc_config.s_status  = bus_space_read_1(t,h,1);
	/* read int_s1 */
	sc->sc_config.s_int_s1 |= bus_space_read_1(t,h,0);

	/* check if t125 is running */
	if(sc->sc_config.s_int_m2 & 1)
	{
		/* check busy/non-busy transition flag */
		if(sc->sc_config.s_status & 0x04)
		{
			/* disable sync(R)
			 * write s_int_m2(reg=0x1b)
			 */
			bus_space_write_1(t,h,1,0x1b);
			bus_space_write_1(t,h,0,
					  (sc->sc_config.s_int_m2 &= ~1));
			SC_T125_WAIT_CLEAR(sc);
		}
	}

	/* enable interrupts */
	sti(sc);

	return;
}

static ihfc_fifo_program_t *
hfcs_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if((FIFO_NO(f) == d1t) ||
	   (FIFO_NO(f) == d1r))
	{
	  if(PROT_IS_HDLC(&(f->prot_curr)))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_hfc_tx_program :
			&i4b_hfc_rx_program;
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

/*-------------------------------------------------+
 | hfcs related structures                         |
 +-------------------------------------------------*/

register_list_t
hfcs_register_list[] =
{
  { REG2OFF(s_cirm),         0x18 },
  { REG2OFF(s_ctmt),         0x19 },
  { REG2OFF(s_int_m1),       0x1a },
  { REG2OFF(s_int_m2),       0x1b },
  { REG2OFF(s_clkdel),       0x37 },
  { REG2OFF(s_connect),      0x2f }, /* must be loaded before SCTRL */
  { REG2OFF(s_sctrl),        0x31 },
  { REG2OFF(s_test),         0x32 },
  { REG2OFF(s_b1_ssl),       0x20 },
  { REG2OFF(s_b2_ssl),       0x21 },
  { REG2OFF(s_a1_ssl),       0x22 },
  { REG2OFF(s_a2_ssl),       0x23 },
  { REG2OFF(s_b1_rsl),       0x24 },
  { REG2OFF(s_b2_rsl),       0x25 },
  { REG2OFF(s_a1_rsl),       0x26 },
  { REG2OFF(s_a2_rsl),       0x27 },
  { REG2OFF(s_mst_mode),     0x2e },
  { 0, 0 }
};

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfcs_dbase_root)
{
  I4B_DBASE_ADD(c_chip_read         , &hfcs_chip_read);
  I4B_DBASE_ADD(c_chip_write        , &hfcs_chip_write);
  I4B_DBASE_ADD(c_chip_reset        , &hfcs_chip_reset);
  I4B_DBASE_ADD(c_chip_unselect     , &hfcs_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read  , &hfcs_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check , &hfcs_chip_status_check);

  I4B_DBASE_ADD(c_fsm_read          , &hfcs_fsm_read);
  I4B_DBASE_ADD(c_fsm_write         , &hfcs_fsm_write);
  I4B_DBASE_ADD(d_fsm_table         , &hfcs_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program  , &hfcs_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read         , &hfcs_fifo_read);
  I4B_DBASE_ADD(c_fifo_write        , &hfcs_fifo_write);
  I4B_DBASE_ADD(c_fifo_select       , &hfcs_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx       , &hfcs_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read      , &hfcs_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre   , &hfcs_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_frame_check  , &hfcs_fifo_frame_check);

  I4B_DBASE_ADD(d_register_list     , &hfcs_register_list[0]);
  I4B_DBASE_ADD(d_channels          , 6);

  /* delay 50 milliseconds */ 
  I4B_DBASE_ADD(d_interrupt_delay   , hz / 20);

  I4B_DBASE_ADD(o_RES_IRQ_0         , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0      , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */
  I4B_DBASE_ADD(o_ECHO_CANCEL_ENABLED, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask         , (I4B_OPTION_POLLED_MODE|
					   I4B_OPTION_NT_MODE|
					   I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value        , 0);

#if 1 /* (sc->sc_config.s_cirm & 0x10) */

  /*
   * 8kbyte fifo mode selected
   */

  I4B_DBASE_ADD(o_8KFIFO            , 1); /* enable */

  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(hfc_fifo_map[ 2]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(hfc_fifo_map[ 3]));
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[ 4]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[ 5]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[ 6]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[ 7]));

#else

  /*
   * 32kbyte fifo mode selected
   */

  I4B_DBASE_ADD(o_8KFIFO            , 0); /* disable */

  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(hfc_fifo_map[ 2]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(hfc_fifo_map[ 3]));
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[ 8]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[ 9]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[10]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[11]));

#endif
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  static const struct internal list_iio[] =
  {
    { .value = 0x000, .internal = 0x200 }
  };

  static const struct internal list_iirq[] =
  {
    { .value = 0, .internal = 2 }
  };

  I4B_DBASE_IMPORT(hfcs_dbase_root);

  I4B_DBASE_ADD(stdel_nt            , 0x6c);
  I4B_DBASE_ADD(stdel_te            , 0x2d);

  I4B_DBASE_ADD(list_iio            , &list_iio[0]);
  I4B_DBASE_ADD(list_iirq           , &list_iirq[0]);
}

I4B_PNP_DRIVER(/* TELES 16.3c version A */
	       .vid           = 0x10262750);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  static const struct internal list_iio[] =
  {
    { .value = 0x000, .internal = 0x300 }
  };

  static const struct internal list_iirq[] =
  {
    { .value = 0, .internal = 1 }
  };

  I4B_DBASE_IMPORT(hfcs_dbase_root);

  I4B_DBASE_ADD(stdel_nt            , 0x6c);
  I4B_DBASE_ADD(stdel_te            , 0x0e);

  I4B_DBASE_ADD(list_iio            , &list_iio[0]);
  I4B_DBASE_ADD(list_iirq           , &list_iirq[0]);
}

I4B_PNP_DRIVER(/* AcerISDN P10 */
	       .vid           = 0x1411d805);

static int
hfcs_chip_identify(device_t dev)
{
	struct resource * res;
	u_int32_t rid;
	u_int32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "I/O-base for HFC-S 2BDS0!\n",
			      __FUNCTION__);
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	/* setup I/O-port */

	bus_set_resource(dev, SYS_RES_IOPORT, 0, base, 2);
	return(0);
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  static const struct internal list_iirq[] =
  {
    { .value = 0, .internal = 0 } /* IRQ disabled */
  };

  I4B_DBASE_IMPORT(hfcs_dbase_root);

  I4B_DBASE_ADD(c_chip_identify     , &hfcs_chip_identify);
  I4B_DBASE_ADD(desc                , "HFC-2BDS0 ISDN card");

  I4B_DBASE_ADD(list_iio            , 0);
  I4B_DBASE_ADD(list_iirq           , &list_iirq[0]);
}

I4B_ISA_DRIVER(/* custom HFC-S */
	       .vid  = 0x02 /* card number */);

#endif /* _I4B_HFCS_H_ */
