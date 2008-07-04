/*-
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
 *      Dr. Neuhaus Niccy GO@ and SAGEM Cybermod Driver
 *	===============================================
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_DRN_NGO_H_
#define _I4B_DRN_NGO_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

/* imports */
#define drn_ngo_fifo_get_program  isac_hscx_generic_fifo_get_program
#define drn_ngo_fsm_read          isac_hscx_generic_fsm_read
#define drn_ngo_fsm_write         isac_hscx_generic_fsm_write
#define drn_ngo_chip_unselect     isac_hscx_generic_chip_unselect
#define drn_ngo_chip_status_read  isac_hscx_generic_chip_status_read

/*
 * Niccy GO@ definitions
 *
 * the card uses 2 I/O-bases each using 2 bytes
 *
 * iobase0 + 0 : ISAC dataregister r/w
 * iobase0 + 1 : HSCX dataregister r/w
 *
 * iobase1 + 0 : ISAC index register write
 * iobase1 + 1 : HSCX index register write
 *
 * to access an ISAC or HSCX register,  you first have to write the
 * register number into  the  respective  address register and then
 * read or write data from or to the respective data register.
 *
 * Thanks to Klaus Muehle of Dr. Neuhaus Telekommunikation for giving
 * out this information!
 *                                                     
 * some more information:
 *
 * #define NICCY_PORT_MAX	0x200
 * #define NICCY_PORT_MAX	0x3e0
 *
 * #define HSCX_ABIT	0x1000		flag, HSCX A is meant
 * #define HSCX_BBIT	0x2000		flag, HSCX B is meant
 *
 * #define HSCX_BOFF	0x40
 *
 * #define ADDR_OFF	2		address register range offset
 *
 * #define ISAC_DATA	0
 * #define HSCX_DATA	1
 *
 * #define ISAC_ADDR	0
 * #define HSCX_ADDR	1
 */

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t  = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h  = sc->sc_resources.io_hdl[0];	\
	bus_space_tag_t    tt = sc->sc_resources.io_tag[1];	\
	bus_space_handle_t hh = sc->sc_resources.io_hdl[1];

static void
drn_ngo_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* ISAC set register */
		bus_space_write_1(tt,hh,0,((reg) & ~0x80));
		/* ISAC read */
		bus_space_read_multi_1(t,h,0,(ptr),(len));
	}
	else
	{
		/* HSCX set register */
		bus_space_write_1(tt,hh,1,(reg));
		/* HSCX read */
		bus_space_read_multi_1(t,h,1,(ptr),(len));
	}
	return;
}

static void
drn_ngo_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	drn_ngo_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
drn_ngo_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* ISAC set register */
		bus_space_write_1(tt,hh,0,((reg) & ~0x80));
		/* ISAC write */
		bus_space_write_multi_1(t,h,0,(ptr),(len));
	}
	else
	{
		/* HSCX set register */
		bus_space_write_1(tt,hh,1,(reg));
		/* HSCX write */
		bus_space_write_multi_1(t,h,1,(ptr),(len));
	}
	return;
}

static void
drn_ngo_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	drn_ngo_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
drn_ngo_chip_reset CHIP_RESET_T(sc,error)
{
	return;
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(drn_ngo_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &drn_ngo_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &drn_ngo_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &drn_ngo_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &drn_ngo_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &drn_ngo_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &default_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &drn_ngo_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &drn_ngo_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &drn_ngo_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &drn_ngo_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &drn_ngo_fifo_get_program);

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

  I4B_DBASE_ADD(desc               , "Dr.Neuhaus Niccy Go@");

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_1     , 1); /* enable */
}

I4B_PNP_DRIVER(/* Dr.Neuhaus Niccy Go@ */
	       .vid           = 0x5001814c);

#undef drn_ngo_fifo_get_program
#undef drn_ngo_fsm_read
#undef drn_ngo_fsm_write
#undef drn_ngo_chip_unselect
#undef drn_ngo_chip_status_read
#undef IPAC_BUS_VAR
#endif /* _I4B_DRN_NGO_H_ */
