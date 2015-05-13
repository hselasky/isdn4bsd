/*-
 * Copyright (c) 1998 Martijn Plak. All rights reserved.
 *
 * Copyright (c) 1998, 1999 Martin Husemann. All rights reserved.
 *
 * Copyright (c) 1998, 2000 German Tischler. All rights reserved.
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
 * 	----------------
 *
 *	last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_DYNALINK_H_
#define _I4B_DYNALINK_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

/* imports */
#define dynalink_chip_reset_verify isac_hscx_generic_chip_reset_verify
#define dynalink_fifo_get_program  isac_hscx_generic_fifo_get_program
#define dynalink_fsm_read          isac_hscx_generic_fsm_read
#define dynalink_fsm_write         isac_hscx_generic_fsm_write
#define dynalink_chip_unselect     isac_hscx_generic_chip_unselect
#define dynalink_chip_status_read  isac_hscx_generic_chip_status_read

/*
 * (from i4b_itk_ix1.c ....)
 *
 * The ITK ix1 micro ISDN card is an ISA card with one region
 * of four io ports  mapped  and  a fixed IRQ all jumpered on
 * the card.      Access to the board is straight forward and 
 * [very] similar to the ELSA and DYNALINK cards.
 *
 * To reset the card:
 *   - write 0x01 to ITK_CONFIG
 *   - wait >= 10 ms
 *   - write 0x00 to ITK_CONFIG
 *
 * To read or write data:
 *  - write address to ITK_ALE port
 *  - read data from or write data to ITK_ISAC_DATA port  or
 *    ITK_HSCX_DATA port. The two HSCX channel registers are
 *    offset by HSCXA (0x00) and HSCXB (0x40).
 *
 * The ITK ix1 micro ISDN card  can be identified  by reading
 * 0x00 from HSCX register 0xff,   which do not really exist.
 * Besides from that  it  is  also possible to check the HSCX
 * version registers. But in case clones exist it is better
 * to check registers which are independent of the chip  ver-
 * sion. When the card is PnP it has already been identified,
 * so what is left is to check that the chip ``works''.
 *
 * The ITK uses an area of 4 bytes for io.
 * Card format:
 *
 * iobase + 0 : ISAC read/write
 * iobase + 1 : HSCX read/write ( index 0x00-0x3f HSCX A ,
 *                                index 0x40-0x7f HSCX B )
 * iobase + 2 : ISAC/HSCX register index
 * iobase + 3 : reset
 *
 * Register offsets
 *
 *#define	ITK_ISAC_DATA	0
 *#define	ITK_HSCX_DATA	1
 *#define	ITK_ALE		2
 *#define	ITK_CONFIG	3
 *
 * Size of IO range to allocate for this card
 *
 *#define	ITK_IO_SIZE	4
 *
 * Register offsets for the two HSCX channels
 *
 *#define	HSCXA	0x00
 *#define	HSCXB	0x40
 *
 *
 * (from i4b_dynalink.c ...)
 *
 * Dynalink i/o access  methods
 *
 * (t,h,0,reg..) : read/write ISAC
 * (t,h,1,reg..) : read/write HSCX
 * (t,h,2,reg)   : set register (ISAC/HSCX)
 *
 * (from i4b_sws.c ....)
 *
 * The SWS uses an area of 8 bytes for io.
 * Card format:
 * 
 * iobase + 0 : reset on  (0x03)
 * iobase + 1 : reset off (0x00)
 * iobase + 2 : ISAC read/write
 * iobase + 3 : HSCX read/write ( offset 0-0x3f    hscx0 , 
 *                                offset 0x40-0x7f hscx1 )
 * iobase + 4 : set register offset (ISAC/HSCX)
 * iobase + 5 : this is for faking that we mean hscx1, though
 *              access is done through hscx0
 * iobase + 6 : ??
 * iobase + 7 : ??
 *
 */

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

static void
dynalink_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* read ISAC */
		bus_space_write_1(t,h,2,((reg) & ~0x80));
		bus_space_read_multi_1(t,h,0,(ptr),(len));
	}
	else
	{
		/* read HSCX */
		bus_space_write_1(t,h,2,(reg));
		bus_space_read_multi_1(t,h,1,(ptr),(len));
	}
	return;
}

static void
dynalink_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	dynalink_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
dynalink_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* write ISAC */
		bus_space_write_1(t,h,2,((reg) & ~0x80));
		bus_space_write_multi_1(t,h,0,(ptr),(len));
	}
	else
	{
		/* write HSCX */
		bus_space_write_1(t,h,2,(reg));
		bus_space_write_multi_1(t,h,1,(ptr),(len));
	}
	return;
}

static void
dynalink_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	dynalink_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
dynalink_common_chip_reset CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);
	uint8_t a;

	switch(sc->sc_cookie) {
	case 1:
	  /*
	   * Dynalink:
	   * perform reset
	   */
	  for(a = 2; a--; )
	  {
	    bus_space_write_1(t,h,2,(a ^= 0x80));
	    DELAY(500);
	  }
	  break;
	case 2:
	  /* ITK Ix1 Micro V3:
	   * perform reset
	   */
	  bus_space_write_1(t,h,3,1);
	  DELAY(500);
	  bus_space_write_1(t,h,3,0);
	  DELAY(500);
	  break;
	case 3:
	  /*
	   * Sedlbauer WinSpeed
	   * 
	   * NOTE: sc->sc_resources.io_hdl[1] should not be updated
	   *       after reset, hence it will be used to backup the
	   *       original io_hdl[0].
	   */

	  sc->sc_resources.io_hdl[1] = sc->sc_resources.io_hdl[0];
	  sc->sc_resources.io_tag[1] = sc->sc_resources.io_tag[0];

	  /*
	   * NOTE: The only difference between SWS and the other
	   *	   Dynalink ISDN cards, is that the I/O-port should
	   *	   be shifted two [byte] units higher. Instead of
	   *	   making a separate driver with this shift, the
	   *	   shift is done in software by adding two to the
	   *	   io resource handle. This should only be performed
	   *	   once.
	   *
	   * shift and update:
	   */
	   sc->sc_resources.io_hdl[0] += 2;
	   sc->sc_cookie += 1;

	case 4:
	   t = sc->sc_resources.io_tag[1];
	   h = sc->sc_resources.io_hdl[1];

	   /* perform reset */
	   bus_space_write_1(t,h,0,0x3);
	   DELAY(500);
	   bus_space_write_1(t,h,1,0x0);
	   DELAY(500);
	   break;

	default:
	  /* no reset */
	  return;
	}

	/*
	 * call reset verify routine
	 */

	dynalink_chip_reset_verify(sc,error);

	return;
}

static int
dynalink_chip_identify(device_t dev)
{
	struct resource * res;
	uint32_t rid;
	uint32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "I/O-base for ITK ix1!\n",
			      __FUNCTION__);
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	/* setup I/O-port */

	bus_set_resource(dev, SYS_RES_IOPORT, 0, base, 4);
	return(0);
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(dynalink_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &dynalink_common_chip_reset );
  I4B_DBASE_ADD(c_chip_read        , &dynalink_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &dynalink_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &dynalink_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &dynalink_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &default_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &dynalink_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &dynalink_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &dynalink_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &dynalink_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &dynalink_fifo_get_program);

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


  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(dynalink_dbase_root);
  I4B_DBASE_ADD(desc          , "Dynalink IS64PH");
  I4B_DBASE_ADD(cookie        , 1);
}

I4B_PNP_DRIVER(/* Dynalink IS64PH
		* MultiTech MT128SA
		*/
	       .vid           = 0x88167506);

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(dynalink_dbase_root);
  I4B_DBASE_ADD(c_chip_identify, &dynalink_chip_identify);
  I4B_DBASE_ADD(desc           , "ITK ix1 Micro ISA");
  I4B_DBASE_ADD(cookie         , 2);
}

I4B_ISA_DRIVER(/* ITK Ix1 Micro ISA */
	       .vid = 0x13/* card number */);

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(dynalink_dbase_root);
  I4B_DBASE_ADD(desc          , "ITK ix1 Micro V3.0");
  I4B_DBASE_ADD(cookie        , 2);
}

I4B_PNP_DRIVER(/* ITK Ix1 Micro V3 */
	       .vid           = 0x25008b26);

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(dynalink_dbase_root);
  I4B_DBASE_ADD(desc          , "Sedlbauer WinSpeed");
  I4B_DBASE_ADD(cookie        , 3);
}

I4B_PNP_DRIVER(/* Sedlbauer WinSpeed */
	       .vid           = 0x0100274c);

#undef dynalink_chip_reset_verify
#undef dynalink_fifo_get_program
#undef dynalink_fsm_read
#undef dynalink_fsm_write
#undef dynalink_chip_unselect
#undef dynalink_chip_status_read
#undef IPAC_BUS_VAR
#endif /* _I4B_DYNALINK_H_ */
