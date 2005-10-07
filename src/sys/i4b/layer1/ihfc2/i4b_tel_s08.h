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
 *	Teles S0/8 and clones passive card driver
 *	-----------------------------------------
 *
 *      last edit-date: 
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_TEL_S08_H_
#define _I4B_TEL_S08_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */
#define tel_s08_chip_reset_verify    isac_hscx_generic_chip_reset_verify
#define tel_s08_fifo_get_program     isac_hscx_generic_fifo_get_program
#define tel_s08_fsm_read             isac_hscx_generic_fsm_read
#define tel_s08_fsm_write            isac_hscx_generic_fsm_write
#define tel_s08_chip_unselect        isac_hscx_generic_chip_unselect
#define tel_s08_chip_status_read     isac_hscx_generic_chip_status_read
#define tel_s08_chip_status_check    default_chip_status_check
#define tel_s08_fsm_table            isac_fsm_table

#define TELES_S08_MEMSIZE 0x1000 /* bytes */

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.mem_hdl[0];

/* chip  : offset  : IPAC offset  : XOR difference
 * ------+---------+--------------+----------------
 * ISAC  : 0x100   : 0x80         : 0x180
 * HSCXA : 0x180   : 0x00         : 0x180
 * HSCXB : 0x1c0   : 0x40         : 0x180
 */

#define IPAC_BUS_SETUP(sc,reg)					\
	if((reg) & 0x20)					\
	{							\
	  /* REGISTER */					\
	  (reg) = ((reg) ^ (((reg) & 1) ? 0x380 : 0x180));	\
	}							\
	else							\
	{							\
	  /* FIFO */						\
	  (reg) = (((reg) ^ 0x180) & 0x1C0);			\
	}

static void
tel_s08_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);
#if 1
	if(len == 0)
	{
		return;
	}
#endif
	/* read ISAC, HSCX A or HSCX B */
	if(reg & 0x20)
	  bus_space_read_multi_1(t, h, reg, ptr, len);
	else
	  bus_space_read_region_1(t, h, reg, ptr, len);
	return;
}

static void
tel_s08_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	tel_s08_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
tel_s08_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);
#if 1
	if(len == 0)
	{
		return;
	}
#endif
	/* write ISAC, HSCX A or HSCX B */
	if(reg & 0x20)
	  bus_space_write_multi_1(t, h, reg, ptr, len);
	else
	  bus_space_write_region_1(t, h, reg, ptr, len);
	return;
}

static void
tel_s08_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	tel_s08_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
tel_s08_chip_reset CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);

	/* set card off */

	bus_space_write_1(t, h, 0x80, 0);

	DELAY(500);

	/* set card on */

	bus_space_write_1(t, h, 0x80, 1);

#if 0
	/*
	 * call reset verify routine
	 */

	tel_s08_chip_reset_verify(sc,error);
#endif
	return;
}

static int
tel_s08_chip_identify(device_t dev)
{
#if 0
	/* see if an io base was supplied */

	if((sc->sc_resources.io_base[0] =
			bus_alloc_resource_any(dev, SYS_RES_IOPORT,
					       &sc->sc_resources.io_rid[0],
					       RF_ACTIVE)))
	{
		/* the S0/8 is completely memory mapped ! */
		
	 	bus_release_resource(dev,SYS_RES_IOPORT,
				     sc->sc_resources.io_rid[0],
				     sc->sc_resources.io_base[0]);
		printf("isic%d: Error, iobase specified for Teles S0/8!\n", unit);
		return(ENXIO);
	}
#endif
	struct resource * res;
	u_int32_t rid;
	u_int32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_MEMORY, &rid, 0 /* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "memory base for Teles S0/8!\n",
			      __FUNCTION__);
		return(ENXIO);
	}

	/* get memory base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_MEMORY, rid, res);

	/* check if inside memory range of 0xA0000 .. 0xDF000 */

	if((base < 0xa0000) ||
	   (base > 0xdf000))
	{
		device_printf(dev,
			      "%s: ERROR: memory base (0x%08x) "
			      "outside 0xA0000-0xDF000 "
			      "for Teles S0/8!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}

	/* setup memory */

	bus_set_resource(dev, SYS_RES_MEMORY, 0, base, TELES_S08_MEMSIZE);

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IRQ, &rid, 0/* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "an irq for Teles S0/8!\n", 
			      __FUNCTION__);
		return ENXIO;
	}

	/* get IRQ */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IRQ, rid, res);

	/* check IRQ validity */

	switch(base) {
	case 2:
	case 9:		/* XXX */
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		break;
			
	default:
		device_printf(dev, "%s: ERROR: invalid IRQ [%d] "
			      "specified for Teles S0/8!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}
	return (0);
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(tel_s08_dbase_root)
{
  I4B_DBASE_ADD(c_chip_identify    , &tel_s08_chip_identify);
  I4B_DBASE_ADD(c_chip_reset       , &tel_s08_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &tel_s08_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &tel_s08_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &tel_s08_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &tel_s08_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &tel_s08_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &tel_s08_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &tel_s08_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &tel_s08_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &tel_s08_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &tel_s08_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &tel_s08_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_isac_hscx_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 0); /* disable, IOM-mode 1 */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));

  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF (isac_fifo_map[1]));
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF (isac_fifo_map[1])); 
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF (isac_fifo_map[2])); 
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF (isac_fifo_map[2]));

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0     , 1); /* enable */
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(tel_s08_dbase_root);

  I4B_DBASE_ADD(desc               , "Teles S0/8 ISA");
}

I4B_ISA_DRIVER(.vid = 0x0F /* card number */);

#undef tel_s08_chip_reset_verify
#undef tel_s08_fifo_get_program
#undef tel_s08_fsm_read
#undef tel_s08_fsm_write
#undef tel_s08_chip_unselect
#undef tel_s08_chip_status_read
#undef tel_s08_chip_status_check
#undef tel_s08_fsm_table
#undef TELES_S08_MEMSIZE
#undef IPAC_BUS_VAR
#undef IPAC_BUS_SETUP
#endif /* _I4B_TEL_S08_H_ */
