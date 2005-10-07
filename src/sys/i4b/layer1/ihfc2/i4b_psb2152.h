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
 *      I4B Siemens PSB 2152 ISDN Chipset Driver
 *      ----------------------------------------
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_PSB2152_H_
#define _I4B_PSB2152_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */
#define psb2152_fifo_get_program  isac_hscx_generic_fifo_get_program
#define psb2152_fsm_read          isac_hscx_generic_fsm_read
#define psb2152_fsm_write         isac_hscx_generic_fsm_write
#define psb2152_chip_unselect     isac_hscx_generic_chip_unselect
#define psb2152_chip_status_read  isac_hscx_generic_chip_status_read
#define psb2152_chip_status_check default_chip_status_check
#define psb2152_fsm_table         isac_fsm_table

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

#define IPAC_READ_1(reg,var)                    \
        /* IPAC read */                         \
        bus_space_write_1(t,h,0,(reg));         \
        var = bus_space_read_1(t,h,1)

#define IPAC_WRITE_1(reg,var)                   \
        /* IPAC write */                        \
        bus_space_write_1(t,h,0,(reg));         \
        bus_space_write_1(t,h,1,(var))

#define IPAC_READ_MULTI_1(reg,ptr,len)                  \
        /* IPAC read */                                 \
        bus_space_write_1(t,h,0,(reg));                 \
        bus_space_read_multi_1(t,h,1,(ptr),(len))

#define IPAC_WRITE_MULTI_1(reg,ptr,len)                 \
        /* IPAC write */                                \
        bus_space_write_1(t,h,0,(reg));                 \
        bus_space_write_multi_1(t,h,1,(ptr),(len))


static void
psb2152_chip_reset CHIP_RESET_T(sc,error)
{
	return;
}

static void
psb2152_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	/* read ISAC, HSCX A or HSCX B */
	IPAC_READ_MULTI_1(reg,ptr,len);
	return;
}

static void
psb2152_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	psb2152_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
psb2152_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	/* write ISAC, HSCX A or HSCX B */
	IPAC_WRITE_MULTI_1(reg,ptr,len);
	return;
}

static void
psb2152_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	psb2152_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(psb2152_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &psb2152_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &psb2152_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &psb2152_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &psb2152_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &psb2152_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &psb2152_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &psb2152_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &psb2152_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &psb2152_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &psb2152_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &psb2152_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &psb2152_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_isac_hscx_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));

  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF (isac_fifo_map[1 +2])); 
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF (isac_fifo_map[1 +2])); 
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF (isac_fifo_map[2 +2])); 
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF (isac_fifo_map[2 +2]));

  I4B_DBASE_ADD(desc               , "PSB 2152 ISDN card");

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
}

I4B_PNP_DRIVER(/* */
               .vid           = -1/* PnP id here */);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(psb2152_dbase_root);

  /*  I4B_DBASE_ADD(io_rid[0]       , must specify here ); */
}

I4B_PCI_DRIVER(/* */
	       .vid           = -1/* PCI id here */);

#undef psb2152_fifo_get_program
#undef psb2152_fsm_read
#undef psb2152_fsm_write
#undef psb2152_chip_unselect
#undef psb2152_chip_status_read
#undef psb2152_chip_status_check
#undef psb2152_fsm_table
#undef IPAC_BUS_VAR
#undef IPAC_READ_1
#undef IPAC_WRITE_1
#undef IPAC_READ_MULTI_1
#undef IPAC_WRITE_MULTI_1
#endif /* _I4B_PSB2152_H_ */
