/*-
 * Copyright (c) 1999, 2001 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *	ELSA MicroLink ISDN/PCC-16, ELSA PCFpro and
 *	ELSA Quickstep 1000pro ISA passive card driver
 *	----------------------------------------------
 *
 *      last edit-date: 
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_ELSA_PCC16_H_
#define _I4B_ELSA_PCC16_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */
#define elsa_pcc16_chip_reset_verify isac_hscx_generic_chip_reset_verify
#define elsa_pcc16_fifo_get_program  isac_hscx_generic_fifo_get_program
#define elsa_pcc16_fsm_read          isac_hscx_generic_fsm_read
#define elsa_pcc16_fsm_write         isac_hscx_generic_fsm_write
#define elsa_pcc16_chip_unselect     isac_hscx_generic_chip_unselect
#define elsa_pcc16_chip_status_read  isac_hscx_generic_chip_status_read
#define elsa_pcc16_chip_status_check default_chip_status_check
#define elsa_pcc16_fsm_table         isac_fsm_table

/* masks for register encoded in base addr */

#define ELSA_BASE_MASK		0x0ffff
#define ELSA_OFF_MASK		0xf0000

/* register id's to be encoded in base addr */

#define ELSA_IDISAC		0x00000
#define ELSA_IDHSCXA		0x10000
#define ELSA_IDHSCXB		0x20000

/* offsets from base address */

#define ELSA_OFF_ISAC		0x00
#define ELSA_OFF_HSCX		0x02
#define ELSA_OFF_REG		0x03
#define ELSA_OFF_CTRL		0x04
#define ELSA_OFF_CFG		0x05
#define ELSA_OFF_TIMR		0x06
#define ELSA_OFF_IRQ		0x07

/* control register (write access) */

#define ELSA_CTRL_LED_YELLOW	0x02
#define ELSA_CTRL_LED_GREEN	0x08
#define ELSA_CTRL_RESET		0x20
#define ELSA_CTRL_TIMEREN	0x80
#define ELSA_CTRL_SECRET	0x50

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

static void
elsa_pcc16_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
	    /* read ISAC */
  	    bus_space_write_1(t, h, ELSA_OFF_REG, (reg) & ~0x80);
	    bus_space_read_multi_1(t, h, ELSA_OFF_ISAC,(ptr),(len));
	}
	else
	{
	    /* read HSCX */
	    bus_space_write_1(t, h, ELSA_OFF_REG, reg);
	    bus_space_read_multi_1(t, h, ELSA_OFF_HSCX,(ptr),(len));
	}
	return;
}

static void
elsa_pcc16_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	elsa_pcc16_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
elsa_pcc16_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
		/* write ISAC */
		bus_space_write_1(t, h, ELSA_OFF_REG, (reg) & ~0x80);
		bus_space_write_multi_1(t, h, ELSA_OFF_ISAC,(ptr),(len));
	}
	else
	{
		/* write HSCX */
		bus_space_write_1(t, h, ELSA_OFF_REG, reg);
		bus_space_write_multi_1(t, h, ELSA_OFF_HSCX,(ptr),(len));
	}
	return;
}

static void
elsa_pcc16_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	elsa_pcc16_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
elsa_pcc16_clear_irq(ihfc_sc_t *sc)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp;

	isac_hscx_generic_chip_unselect(sc);

#if 1
#define IPAC_WRITE_MULTI_1(reg, ptr, len) CHIP_WRITE_MULTI_1(sc,reg,ptr,len)

	/* XXX
	 * XXX if this is not necessary just comment out !
	 * XXX
	 */

        temp = 0xFF;

        /* write MASK (ISAC) */
        IPAC_WRITE_MULTI_1(REG_isac_mask, &temp, 1);

        /* write MASK (HSCX A) */
        IPAC_WRITE_MULTI_1(REG_hscxA_mask, &temp, 1);

        /* write MASK (HSCX B) */
        IPAC_WRITE_MULTI_1(REG_hscxB_mask, &temp, 1);


        /* clear IRQ */

	bus_space_write_1(t, h, ELSA_OFF_IRQ, 0);


        temp = sc->sc_config.i_mask;

        /* write MASK (ISAC) */
        IPAC_WRITE_MULTI_1(REG_isac_mask, &temp, 1);

        temp = sc->sc_config.h_mask[0];

        /* write MASK (HSCX A) */
        IPAC_WRITE_MULTI_1(REG_hscxA_mask, &temp, 1);

        temp = sc->sc_config.h_mask[1];

        /* write MASK (HSCX B) */
        IPAC_WRITE_MULTI_1(REG_hscxB_mask, &temp, 1);

#undef IPAC_WRITE_MULTI_1
#endif
	return;
}

static void
elsa_pcc16_chip_reset CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp = (ELSA_CTRL_SECRET & (~ELSA_CTRL_RESET));

        bus_space_write_1(t, h, ELSA_OFF_CTRL, temp);

        DELAY(500);

	temp |= ELSA_CTRL_RESET;
        bus_space_write_1(t, h, ELSA_OFF_CTRL, temp);

        DELAY(500);
        bus_space_write_1(t, h, ELSA_OFF_IRQ, 0xff);

	/*
	 * call reset verify routine
	 */

	elsa_pcc16_chip_reset_verify(sc,error);

	return;
}

static int
elsa_pcc16_chip_identify(device_t dev)
{
	struct resource * res;
	u_int32_t rid;
	u_int32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0 /* !RF_ACTIVE */)))
	{
		device_printf(dev, "ERROR: could not get "
			      "iobase for ELSA PCC-16!\n");
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	switch(base) {
	case 0x160:
	case 0x170:
	case 0x260:
	case 0x360:
		break;

	default:
		device_printf(dev, 
			      "%s: ERROR: invalid iobase [0x%x]!\n",
			      __FUNCTION__, base);
		return ENXIO;
	}

	/* setup I/O-port */

	bus_set_resource(dev, SYS_RES_IOPORT, 0, base, 8);


	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IRQ, &rid, 0/* !RF_ACTIVE */)))
	{
		device_printf(dev, "ERROR: could not get "
			      "an irq for ELSA PCC-16!\n");
		return ENXIO;
	}

	/* get IRQ */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IRQ, rid, res);

	/* check IRQ validity */

	switch(base) {
	case 2:
	case 9:		
	case 3:		
	case 5:
	case 10:
	case 11:
	case 15:		
		break;
			
	default:
		device_printf(dev, "%s: ERROR: invalid IRQ [%d]!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}
	return (0);
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(elsa_pcc16_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &elsa_pcc16_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &elsa_pcc16_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &elsa_pcc16_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &elsa_pcc16_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &elsa_pcc16_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &elsa_pcc16_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &elsa_pcc16_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &elsa_pcc16_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &elsa_pcc16_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &elsa_pcc16_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &elsa_pcc16_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &elsa_pcc16_fifo_get_program);

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

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(elsa_pcc16_dbase_root);
  I4B_DBASE_ADD(c_chip_identify    , &elsa_pcc16_chip_identify);
  I4B_DBASE_ADD(desc               , "ELSA MicroLink ISDN/PCC-16");
}

I4B_ISA_DRIVER(.vid = 0x0E /* card number */);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  /* ELSA QuickStep 1000pro */

  I4B_DBASE_IMPORT(elsa_pcc16_dbase_root);

  I4B_DBASE_ADD(c_chip_unselect, &elsa_pcc16_clear_irq);

}

I4B_PNP_DRIVER(.vid = 0x33019315);

/* cleanup */

#undef elsa_pcc16_chip_reset_verify
#undef elsa_pcc16_fifo_get_program
#undef elsa_pcc16_fsm_read
#undef elsa_pcc16_fsm_write
#undef elsa_pcc16_chip_unselect
#undef elsa_pcc16_chip_status_read
#undef elsa_pcc16_chip_status_check
#undef elsa_pcc16_fsm_table
#undef ELSA_BASE_MASK
#undef ELSA_OFF_MASK
#undef ELSA_IDISAC
#undef ELSA_IDHSCXA
#undef ELSA_IDHSCXB
#undef ELSA_OFF_ISAC
#undef ELSA_OFF_HSCX
#undef ELSA_OFF_REG
#undef ELSA_OFF_CTRL
#undef ELSA_OFF_CFG
#undef ELSA_OFF_TIMR
#undef ELSA_OFF_IRQ
#undef ELSA_CTRL_LED_YELLOW
#undef ELSA_CTRL_LED_GREEN
#undef ELSA_CTRL_RESET
#undef ELSA_CTRL_TIMEREN
#undef ELSA_CTRL_SECRET
#undef IPAC_BUS_VAR
#endif /* _I4B_ELSA_PCC16_H_ */
