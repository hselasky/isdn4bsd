/*-
 * Copyright (c) 2001 Hellmuth Michaelis. All rights reserved.
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
 *	Eicon Diehl DIVA 2.0 (ISA PnP) driver
 *	-------------------------------------
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_DIVA_H_
#define _I4B_DIVA_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */
#define diva_chip_reset_verify    isac_hscx_generic_chip_reset_verify
#define diva_fifo_get_program     isac_hscx_generic_fifo_get_program
#define diva_fsm_read             isac_hscx_generic_fsm_read
#define diva_fsm_write            isac_hscx_generic_fsm_write
#define diva_chip_unselect        isac_hscx_generic_chip_unselect
#define diva_chip_status_read     isac_hscx_generic_chip_status_read

/*
 * offsets from base address
 *
 * #define DIVA_IPAC_OFF_ALE	0x00
 * #define DIVA_IPAC_OFF_RW	0x01
 *
 * #define DIVA_ISAC_OFF_RW	0x02
 * #define DIVA_ISAC_OFF_ALE	0x06
 *
 * #define DIVA_HSCX_OFF_RW	0x00
 * #define DIVA_HSCX_OFF_ALE	0x04
 *
 * #define DIVA_CTRL_OFF		0x07
 * #define		DIVA_CTRL_RDIST	0x01
 * #define		DIVA_CTRL_WRRST	0x08
 * #define		DIVA_CTRL_WRLDA	0x20
 * #define		DIVA_CTRL_WRLDB	0x40
 * #define		DIVA_CTRL_WRICL	0x80
 *
 * HSCX channel base offsets
 *
 * #define DIVA_HSCXA		0x00
 * #define DIVA_HSCXB		0x40
 *
 */

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

static void
diva_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* read ISAC */
		bus_space_write_1(t,h,6,(reg & ~0x80));
		bus_space_read_multi_1(t,h,2,(ptr),(len));
	}
	else
	{
		/* read HSCX */
		bus_space_write_1(t,h,4,(reg));
		bus_space_read_multi_1(t,h,0,(ptr),(len));
	}
	return;
}

static void
diva_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	diva_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
diva_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* write ISAC */
		bus_space_write_1(t,h,6,(reg & ~0x80));
		bus_space_write_multi_1(t,h,2,(ptr),(len));
	}
	else
	{
		/* write HSCX */
		bus_space_write_1(t,h,4,(reg));
		bus_space_write_multi_1(t,h,0,(ptr),(len));
	}
	return;
}

static void
diva_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	diva_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
diva_common_chip_reset CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);

	/* reset on */
	bus_space_write_1(t,h,7, 0x00);
	DELAY(500);

	/* reset off */
	bus_space_write_1(t,h,7, 0x08);
	DELAY(500);

	/*
	 * call reset verify routine last 
	 */

	diva_chip_reset_verify(sc, error);
	return;
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(diva_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &diva_common_chip_reset );
  I4B_DBASE_ADD(c_chip_read        , &diva_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &diva_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &diva_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &diva_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &default_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &diva_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &diva_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &diva_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &diva_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &diva_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_isac_hscx_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF (isac_fifo_map[1])); 
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF (isac_fifo_map[1])); 
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF (isac_fifo_map[2])); 
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF (isac_fifo_map[2]));

  I4B_DBASE_ADD(desc               , "Eicon.Diehl DIVA 2.0 ISA PnP");

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
}

I4B_PNP_DRIVER(/* Eicon DIVA 2.0 ISAC/HSCX */
               .vid           = 0x7100891c);

#undef diva_chip_reset_verify
#undef diva_fifo_get_program
#undef diva_fsm_read
#undef diva_fsm_write
#undef diva_chip_unselect
#undef diva_chip_status_read
#undef IPAC_BUS_VAR
#endif /* _I4B_DIVA_H_ */
