/*-
 * Copyright (c) 1996 Arne Helme. All rights reserved.
 *
 * Copyright (c) 1996 Gary Jennejohn. All rights reserved. 
 *
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
 *
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
 *	HSCX/ISAC driver
 *	----------------
 *
 * $FreeBSD: $
 *
 *	last edit-date: [ ]
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_CTX_S0P_H_
#define _I4B_CTX_S0P_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

/* imports */
#define ctxs0P_chip_reset_verify  isac_hscx_generic_chip_reset_verify
#define ctxs0P_fifo_get_program   isac_hscx_generic_fifo_get_program
#define ctxs0P_fsm_read           isac_hscx_generic_fsm_read
#define ctxs0P_fsm_write          isac_hscx_generic_fsm_write
#define ctxs0P_chip_unselect      isac_hscx_generic_chip_unselect
#define ctxs0P_chip_status_read   isac_hscx_generic_chip_status_read
#define ctxs0P_chip_status_check  default_chip_status_check

/*
 * (from i4b_ctx_s0P.c ....)
 *
 *	NOTE: this driver works for the Creatix ISDN S0-16 P+P and
 *	      for the Teles S0/16.3 PnP card. Although they are not
 *            the same hardware and don't share the same PnP config
 *            information, once the base addresses are set, the
 *            offsets are the same and therefore they can use the
 *	      same driver.
 */

#define IPAC_BUS_VAR(sc)			\
	bus_space_tag_t    t;			\
	bus_space_handle_t h;

/* this macro is used to convert an IPAC 
 * register into a CREATIX register 
 */
#define IPAC_BUS_SETUP(sc, reg)					\
	if((reg) & 0x80)					\
	{							\
		/* ISAC */					\
		t = (sc)->sc_resources.io_tag[0];		\
		h = (sc)->sc_resources.io_hdl[0];		\
		if((reg) & 0x20)				\
		  (reg) &= ~(0x80|0x40|0x20); /* REGISTER */	\
		else						\
		  (reg) = 0x1e; /* FIFO */			\
	}							\
	else							\
	{							\
		/* HSCX */					\
		t = (sc)->sc_resources.io_tag[1];		\
		h = (sc)->sc_resources.io_hdl[1];		\
		if((reg) & 0x20)				\
		{						\
			/* REGISTER */				\
			if((reg) & 0x40)			\
			  (reg) &= ~(0x80|0x40);		\
			else					\
			  (reg) &= ~(0x80|0x40|0x20);		\
		}						\
		else						\
		{						\
			/* FIFO */				\
			if((reg) & 0x40)			\
			  (reg) = 0x3e;				\
			else					\
			  (reg) = 0x1e;				\
		}						\
	}

static void
ctxs0P_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc, reg);

	/* read ISAC, HSCX A or HSCX B */
	bus_space_read_multi_1(t,h,(reg),(ptr),(len));
	return;
}

static void
ctxs0P_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	ctxs0P_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
ctxs0P_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);

	/* write ISAC, HSCX A or HSCX B */
	bus_space_write_multi_1(t,h,(reg),(ptr),(len));	
	return;
}

static void
ctxs0P_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	ctxs0P_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
ctxs0P_common_chip_reset CHIP_RESET_T(sc,error)
{
	bus_space_tag_t    t;
	bus_space_handle_t h;

	switch(sc->sc_cookie) {
	case 1:
		/* CREATIX reset */
	
		t = sc->sc_resources.io_tag[0];
		h = sc->sc_resources.io_hdl[0];

		bus_space_write_1(t, h, 0x1c, 0);
		DELAY(4000);

		bus_space_write_1(t, h, 0x1c, 1);
		DELAY(4000);

		break;
	case 2:
		/* NOTE: The COMPAQ reset
		 * register is allocated on
		 * the third I/O-port resource
		 */

		t = sc->sc_resources.io_tag[2];
		h = sc->sc_resources.io_hdl[2];

		bus_space_write_1(t, h, 0xff, 0);
		DELAY(4000);

		bus_space_write_1(t, h, 0x00, 1);
		DELAY(4000);

		break;
	default:
		/*
		 * checks should only be
		 * performed if the chip
		 * was reset. Else return.
		 */
		return;
	}

	/*
	 * call reset verify routine
	 */

	ctxs0P_chip_reset_verify(sc,error);

	return;
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(ctxs0P_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &ctxs0P_common_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &ctxs0P_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &ctxs0P_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &ctxs0P_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &ctxs0P_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &ctxs0P_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &ctxs0P_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &ctxs0P_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &ctxs0P_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &ctxs0P_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &ctxs0P_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_isac_hscx_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(i4b_option_mask    , (I4B_OPTION_POLLED_MODE|
				      I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value   , 0);

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));

  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF (isac_fifo_map[1]));
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF (isac_fifo_map[1])); 
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF (isac_fifo_map[2])); 
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF (isac_fifo_map[2]));
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(ctxs0P_dbase_root);

  I4B_DBASE_ADD(desc            , "Teles S0/16.3 PnP");

  I4B_DBASE_ADD(o_RES_IRQ_0     , 1); /* enable */ 
  I4B_DBASE_ADD(o_RES_IOPORT_0  , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_1  , 1); /* enable */
  
  I4B_DBASE_ADD(cookie          , 1);
}

I4B_PNP_DRIVER(/* Teles 16.3 PnP (not c version!) */
               .vid           = 0x10212750);

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(ctxs0P_dbase_root);

  I4B_DBASE_ADD(desc            , "Creatix S0/16 PnP");

  I4B_DBASE_ADD(o_RES_IRQ_0     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0  , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_1  , 1); /* enable */

  I4B_DBASE_ADD(cookie          , 1);
}

I4B_PNP_DRIVER(/* Creatix S0/16 P+P  */
               .vid           = 0x0000980e);

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(ctxs0P_dbase_root);

  I4B_DBASE_ADD(desc            , "Compaq Microcom 610");

  I4B_DBASE_ADD(o_RES_IRQ_0     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0  , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_1  , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_2  , 1); /* enable */

  /* NOTE: I/O-port 0 and
   * I/O-port 1 is switched:
   */
  I4B_DBASE_ADD(io_rid[0]       , 1); 
  I4B_DBASE_ADD(io_rid[1]       , 0);

  I4B_DBASE_ADD(cookie          , 2);
}

I4B_PNP_DRIVER(/* Compaq Microcom 610 */
               .vid           = 0x0210110e);


#undef ctxs0P_chip_reset_verify
#undef ctxs0P_fifo_get_program
#undef ctxs0P_fsm_read
#undef ctxs0P_fsm_write
#undef ctxs0P_chip_unselect
#undef ctxs0P_chip_status_read
#undef ctxs0P_chip_status_check
#undef IPAC_BUS_VAR
#undef IPAC_BUS_SETUP
#endif /* _I4B_CTX_S0P_H_ */
