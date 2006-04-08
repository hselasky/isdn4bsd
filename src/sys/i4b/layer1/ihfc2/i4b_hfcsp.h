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
 *	i4b_hfcsp.h - HFC-2BDS0 PnP driver module
 * 	-----------------------------------------
 *
 *	last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFCSP_H_
#define _I4B_HFCSP_H_

#include <i4b/layer1/ihfc2/i4b_hfcs.h>

/* imports */

#define hfcsp_chip_status_read  hfcs_chip_status_read
#define hfcsp_chip_status_check default_chip_status_check
#define hfcsp_fsm_read          hfcs_fsm_read
#define hfcsp_fsm_write         hfcs_fsm_write
#define hfcsp_fsm_table         hfc_fsm_table
#define hfcsp_fifo_frame_check  hfcs_fifo_frame_check

/* driver */

/*-------------------------------------------------+
 | HFC-SP related methods                          |
 |                                                 |
 | (t,h,1,reg)   : set register or read STATUS     |
 | (t,h,0,...)   : read/write HFC-2BDS0            |
 |                                                 |
 +-------------------------------------------------*/

static void
hfcsp_t125_sync FIFO_SYNC_T(sc)
{
	/* 6us delay */
	DELAY(6);
}

static void
hfcsp_chip_reset CHIP_RESET_T(sc,error)
{
	HFC1_BUS_VAR(sc);

	/* enable internal I/O */
	bus_space_write_2(t,h,0,((sc->sc_resources.iio[0] & 0x3ff) | 0x5400));

	/* select CIRM(reg=0x18) */
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

	/* select STATES(reg=0x30) */
	bus_space_write_1(t,h,1,0x30);

	/* read disbusy and states */
	if(bus_space_read_1(t,h,1) & 0x01)
	{
		IHFC_ADD_ERR(error,"(BUSY != 0)");
	}

	if(bus_space_read_1(t,h,0) & 0x0f)
	{
		IHFC_ADD_ERR(error,"(STATES != 0)");
	}

	/*
	 * In case of unknown clones or
	 * newer chips, chip_id is not
	 * checked.
	 * select chip_id
	 * bus_space_write_1(t,h,1,0x16);
	 *
	 * read chip_id
	 * if((bus_space_read_1(t,h,0) & 0xf0) != 0x90)
	 * {
	 *	IHFC_ADD_ERR(error, "(CHIPID != 0x90)");
	 * }
	 */

	return;
}

static void
hfcsp_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	HFC1_BUS_VAR(sc);

	bus_space_write_1(t,h,1,(reg));
	bus_space_read_multi_1(t,h,0,(ptr),(len));

	return;
}

static void
hfcsp_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	/* pre increment Z-counter */
	if(((f->Z_drvr)   += (len)) >= (f->fm.h.Zend)) {
	    (f->Z_drvr)   -= (f->fm.h.Zsize);
	}

	hfcsp_chip_read(sc,(f->fm.h.Zdata),ptr,len);

	return;
}

static void
hfcsp_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	HFC1_BUS_VAR(sc);

	bus_space_write_1(t,h,1, (reg));
	bus_space_write_multi_1(t,h,0,(ptr),(len));

	return;
}

static void
hfcsp_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	/* pre increment Z-counter */
	if(((f->Z_drvr)   += (len)) >= (f->fm.h.Zend)) {
	    (f->Z_drvr)   -= (f->fm.h.Zsize);
	}

	hfcsp_chip_write(sc,(f->fm.h.Zdata),ptr,len);

	return;
}

static void
hfcsp_fifo_select FIFO_SELECT_T(sc,f)
{
	HFC1_BUS_VAR(sc);

	/* write FIFO_SEL(reg=0x10) */
	bus_space_write_1(t,h,1,0x10);
	bus_space_write_1(t,h,0,f->s_fifo_sel);

	hfcsp_t125_sync(sc);

	return;
}

/* NOTE: In POLLED mode the HFC-SP will
 * not work properly if the F-counter
 * is not double incremented after
 * each transmitted frame. This will reduce
 * performance temporarily:
 */

static void
hfcsp_fifo_inc_fx FIFO_INC_FX_T(sc,f);

static void
hfcsp_fifo_inc_fx_pre FIFO_INC_FX_PRE_T(sc,f)
{
	if(FIFO_DIR(f) == transmit)
	{
	  hfcsp_fifo_inc_fx(sc,f);
	}

	return;
}

static void
hfcsp_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	HFC1_BUS_VAR(sc);

 	/* increment currently selected
	 * F_drvr by one:
	 */

	/* update soft counter */
	if(++(f->F_drvr) & 0x20) {
	     (f->F_drvr) -= (f->fm.h.Fsize);
	}

	bus_space_write_1(t,h,1, f->fm.h.Fibase);
	bus_space_read_1(t,h,0);

	/* NOTE: in some special cases the
	 * Z_drvr counter may be changed
	 * after a F_drvr increment.
	 * Reload FZ:
	 */

	f->state &= ~ST_FZ_LOADED;

	hfcsp_t125_sync(sc);

	return;
}

static void
hfcsp_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	u_int16_t Z_chip2;
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
	(f->F_chip) = bus_space_read_1(t,h,0);

	/* zx - high */
	bus_space_write_1(t,h,1, f->fm.h.Zbase ^4);
	(   Z_chip2) = bus_space_read_1(t,h,0) << 8;

	/* zx - low */
	bus_space_write_1(t,h,1, f->fm.h.Zbase ^0);
	(f->Z_chip) = bus_space_read_1(t,h,0);

	/* NOTE: it is only necessary to
	 * read F_drvr and Z_drvr once
	 * from the hardware:
	 */
	if(!(f->state &  ST_FZ_LOADED)) {
	     f->state |= ST_FZ_LOADED;
		/* fx */
		bus_space_write_1(t,h,1, f->fm.h.Fbase ^4);
		(f->F_drvr) = bus_space_read_1(t,h,0);

		/* zx - low */
		bus_space_write_1(t,h,1, f->fm.h.Zbase ^8);
		(f->Z_drvr) = bus_space_read_1(t,h,0);

		/* zx - high */
		bus_space_write_1(t,h,1, f->fm.h.Zbase ^12);
		(f->Z_drvr)|= bus_space_read_1(t,h,0) << 8;
	}

	/* zx - high */
	bus_space_write_1(t,h,1, f->fm.h.Zbase ^4);
	(f->Z_chip)|= bus_space_read_1(t,h,0) << 8;
 
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
hfcsp_chip_unselect CHIP_UNSELECT_T(sc)
{
	/* NOTE:  the HFC-SP chip requires that a
	 * fifo-command,   fifo-select   or    F-
	 * increment,   must be issued after data
	 * has been written to a transmit fifo,to
	 * start transmission of data. If not,the
	 * data transmission  will  start at  the
	 * next fifo-command or fifo-select. This
	 * command may be selecting a different
	 * fifo.
	 *
	 *
	 * Select a fifo only when QUEUE is done;
	 * That means after that all fifo
	 * commands have completed ...
	 */
	if(QDONE(sc))
	{
	  HFC1_BUS_VAR(sc);

	  bus_space_write_1(t,h,1,0x10);
	  bus_space_write_1(t,h,0,0);

	  /* delay some microseconds so that
	   * two fifo selects cannot follow
	   * immediately:
	   */
	  DELAY(6);
	}

	return;
}

static ihfc_fifo_program_t *
hfcsp_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
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
 | hfcsp related structures                        |
 +-------------------------------------------------*/

register_list_t
hfcsp_register_list[] =
{
  { REG2OFF(s_cirm),         0x18 },
  { REG2OFF(s_ctmt_pnp),     0x19 },
  { REG2OFF(s_int_m1),       0x1a },
  { REG2OFF(s_int_m2),       0x1b },
  { REG2OFF(s_mst_emod),     0x2d },
  { REG2OFF(s_clkdel),       0x37 },
  { REG2OFF(s_connect),      0x2f }, /* must be loaded before SCTRL */
  { REG2OFF(s_sctrl),        0x31 },
  { REG2OFF(s_sctrl_e),      0x32 },
  { REG2OFF(s_sctrl_r),      0x33 },
  { REG2OFF(s_trm),          0x12 },
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

I4B_DBASE(hfcsp_dbase_root)
{
  static const struct internal list_iirq[] =
  {
    { .value =  5, .internal =  1 },
    { .value =  9, .internal =  2 },
    { .value = 10, .internal =  3 },
    { .value = 11, .internal =  4 },
    { .value = 12, .internal =  5 },
    { .value = 15, .internal =  6 },
    { .value =  3, .internal =  7 },
    { .value =  0, .internal =  0 } /* irq disabled */
  };

  I4B_DBASE_ADD(c_chip_read         , &hfcsp_chip_read);
  I4B_DBASE_ADD(c_chip_write        , &hfcsp_chip_write);
  I4B_DBASE_ADD(c_chip_reset        , &hfcsp_chip_reset);
  I4B_DBASE_ADD(c_chip_unselect     , &hfcsp_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read  , &hfcsp_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check , &hfcsp_chip_status_check);

  I4B_DBASE_ADD(c_fsm_read          , &hfcsp_fsm_read);
  I4B_DBASE_ADD(c_fsm_write         , &hfcsp_fsm_write);
  I4B_DBASE_ADD(d_fsm_table         , &hfcsp_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program  , &hfcsp_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read         , &hfcsp_fifo_read);
  I4B_DBASE_ADD(c_fifo_write        , &hfcsp_fifo_write);
  I4B_DBASE_ADD(c_fifo_select       , &hfcsp_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx       , &hfcsp_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read      , &hfcsp_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre   , &hfcsp_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_frame_check  , &hfcsp_fifo_frame_check);

  I4B_DBASE_ADD(d_register_list     , &hfcsp_register_list[0]);
  I4B_DBASE_ADD(d_channels          , 6);

  /* delay 50 milliseconds */ 
  I4B_DBASE_ADD(d_interrupt_delay   , hz / 20);

  I4B_DBASE_ADD(list_iio            , 0);
  I4B_DBASE_ADD(list_iirq           , &list_iirq[0]);

  I4B_DBASE_ADD(o_RES_IRQ_0         , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0      , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask      , (I4B_OPTION_POLLED_MODE|
					I4B_OPTION_NT_MODE|
					I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value     , 0);

#if 1  /* (sc->sc_config.s_cirm & 0x10) */

  I4B_DBASE_ADD(o_8KFIFO            , 1); /* enable */

  /*
   * 8kbyte fifo mode selected
   */

  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(hfc_fifo_map[ 0]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(hfc_fifo_map[ 1]));
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[ 4]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[ 5]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[ 4]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[ 5]));

#else

  I4B_DBASE_ADD(o_8KFIFO            , 0); /* disable */

  /*
   * 32kbyte fifo mode selected
   */
  
  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(hfc_fifo_map[ 0]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(hfc_fifo_map[ 1]));
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[ 8]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[ 9]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[ 8]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[ 9]));

#endif
}

I4B_PNP_DRIVER(/* TELES 16.3c version B */
	       .vid           = 0x20262750);

#endif /* _I4B_HFCSP_H_ */
