/*-
 * Copyright (c) 1996 Arne Helme. All rights reserved.
 *
 * Copyright (c) 1996 Gary Jennejohn. All rights reserved. 
 *
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 4. Altered versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software and/or documentation.
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
 *	Teles S0/16.3 passive card driver
 *	---------------------------------
 *
 *      last edit-date:
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_TEL_S0163_H_
#define _I4B_TEL_S0163_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

/* imports */
#define tel_s0163_chip_reset_verify  isac_hscx_generic_chip_reset_verify
#define tel_s0163_fifo_get_program   isac_hscx_generic_fifo_get_program
#define tel_s0163_fsm_read           isac_hscx_generic_fsm_read
#define tel_s0163_fsm_write          isac_hscx_generic_fsm_write
#define tel_s0163_chip_unselect      isac_hscx_generic_chip_unselect
#define tel_s0163_chip_status_read   isac_hscx_generic_chip_status_read
#define tel_s0163_chip_status_check  default_chip_status_check
#define tel_s0163_fsm_table          isac_fsm_table

#define ISAC_OFFS	0x400
#define	HSCXA_OFFS	0xc00
#define HSCXB_OFFS	0x800

#define IPAC_BUS_VAR(sc)			\
	bus_space_tag_t    t;			\
	bus_space_handle_t h;

#define IPAC_BUS_SETUP(sc,reg)				\
	if((reg) & 0x80)				\
	{						\
		/* ISAC */				\
		t = (sc)->sc_resources.io_tag[1];	\
		h = (sc)->sc_resources.io_hdl[1];	\
	}						\
	else						\
	{						\
	    if((reg) & 0x40)				\
	    {						\
		/* HSCX B */				\
		t = (sc)->sc_resources.io_tag[3];	\
		h = (sc)->sc_resources.io_hdl[3];	\
	    }						\
	    else					\
	    {						\
		/* HSCX A */				\
		t = (sc)->sc_resources.io_tag[2];	\
		h = (sc)->sc_resources.io_hdl[2];	\
	    }						\
	}						\
	if((reg) & 0x20)				\
	  (reg) &= ~(0x80|0x40|0x20); /* REGISTER */	\
	else						\
	  (reg) = 0x1e; /* FIFO */


static void
tel_s0163_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);

	/* read ISAC, HSCX A or HSCX B */
	bus_space_read_multi_1(t,h,reg,(ptr),(len));
	return;
}

static void
tel_s0163_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	tel_s0163_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
tel_s0163_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);

	/* write ISAC, HSCX A or HSCX B */
	bus_space_write_multi_1(t,h,reg,(ptr),(len));
	return;
}

static void
tel_s0163_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	tel_s0163_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
tel_s0163_chip_reset CHIP_RESET_T(sc,error)
{
	static u_int8_t intr_no[] = { 1, 1, 0, 2, 4, 6, 1, 1, 
				      1, 0, 8, 10, 12, 1, 1, 14 };

	bus_space_tag_t    t = sc->sc_resources.io_tag[0];
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

	u_int8_t temp = intr_no[sc->sc_resources.iirq[0]];

	u_int8_t b0, b1, b2;

	b0 = bus_space_read_1(t, h, 0);
	b1 = bus_space_read_1(t, h, 1);
	b2 = bus_space_read_1(t, h, 2);
	
	if((b0 != 0x51) && (b0 != 0x10))
	{
		IHFC_ADD_ERR(error, "signature 1, 0x%x, != "
			     "0x51 and 0x10 for Teles S0/16.3", b0);
		return;
	}
	
	if(b1 != 0x93)
	{
		IHFC_ADD_ERR(error, "signature 2, 0x%x, != "
			     "0x93 for Teles S0/16.3", b1);
		return;
	}
	if((b2 != 0x1c) && (b2 != 0x1f))
	{
		IHFC_ADD_ERR(error, "signature 3, 0x%x, != "
			     "0x1c and 0x1f for Teles S0/16.3", b2);
		return;
	}

	/* configure IRQ */

	DELAY(500);
	bus_space_write_1(t, h, 4, temp);

	DELAY(500);
	bus_space_write_1(t, h, 4, temp | 0x01);

	return;
}

static int
tel_s0163_chip_identify(device_t dev)
{
	struct resource * res;
	u_int32_t rid;
	u_int32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0 /* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "iobase for Teles S0/16.3!\n",
			      __FUNCTION__);
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	switch(base) {
	case 0xd80:
	case 0xe80:
	case 0xf80:
		break;
			
	case 0x180:
	case 0x280:
	case 0x380:
		device_printf(dev, "using 0x%04x as iobase "
			      "instead of 0x%04x!\n", base+0xC00, base);

		/* take a hint */
		base += 0xC00;
		break;

	default:
		device_printf(dev, 
			      "%s: ERROR: invalid iobase [0x%x] "
			      "specified for Teles S0/16.3!\n",
			      __FUNCTION__, base);
		return ENXIO;
	}
	
	/* setup I/O-ports */

	bus_set_resource(dev, SYS_RES_IOPORT, 0, base, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 1, base-ISAC_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 2, base-HSCXA_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 3, base-HSCXB_OFFS, 0x20);

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IRQ, &rid, 0/* !RF_ACTIVE */)))
	{
		device_printf(dev, "ERROR: could not get "
			      "an irq for Teles S0/16.3!\n");
		return ENXIO;
	}

	/* get IRQ */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IRQ, rid, res);

	/* check IRQ validity */

	switch(base) {
	case 2:
	case 9:
	case 5:
	case 10:
	case 12:
	case 15:
		break;

	default:
		device_printf(dev, "%s: ERROR: invalid IRQ [%d] "
			      "specified for Teles S0/16.3!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}
	return (0);
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(tel_s0163_dbase_root)
{
  I4B_DBASE_ADD(c_chip_identify    , &tel_s0163_chip_identify);
  I4B_DBASE_ADD(c_chip_reset       , &tel_s0163_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &tel_s0163_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &tel_s0163_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &tel_s0163_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &tel_s0163_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &tel_s0163_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &tel_s0163_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &tel_s0163_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &tel_s0163_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &tel_s0163_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &tel_s0163_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &tel_s0163_fifo_get_program);

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
  I4B_DBASE_ADD(o_RES_IOPORT_1     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_2     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_3     , 1); /* enable */
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(tel_s0163_dbase_root);
  I4B_DBASE_ADD(desc          , "Teles S0/16.3 ISA");
}

I4B_ISA_DRIVER(.vid = 0x10 /* card number */);

#undef tel_s0163_chip_reset_verify
#undef tel_s0163_fifo_get_program
#undef tel_s0163_fsm_read
#undef tel_s0163_fsm_write
#undef tel_s0163_chip_unselect
#undef tel_s0163_chip_status_read
#undef tel_s0163_chip_status_check
#undef tel_s0163_fsm_table
#undef ISAC_OFFS
#undef HSCXA_OFFS
#undef HSCXB_OFFS
#undef IPAC_BUS_VAR
#undef IPAC_BUS_SETUP
#endif /* _I4B_TEL_S0163_H_ */
