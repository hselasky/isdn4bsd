/*-
 * Copyright (c) 1996 Andrew Gordon. All rights reserved.
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
 *	AVM A1/Fritz passive card driver
 *	--------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_AVM_A1_H_
#define _I4B_AVM_A1_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

/* imports */

#define avm_a1_chip_reset_verify  isac_hscx_generic_chip_reset_verify
#define avm_a1_fifo_get_program   isac_hscx_generic_fifo_get_program
#define avm_a1_fsm_read           isac_hscx_generic_fsm_read
#define avm_a1_fsm_write          isac_hscx_generic_fsm_write
#define avm_a1_chip_unselect      isac_hscx_generic_chip_unselect
#define avm_a1_chip_status_read   isac_hscx_generic_chip_status_read
#define avm_a1_chip_status_check  default_chip_status_check
#define avm_a1_fsm_table          isac_fsm_table

/*---------------------------------------------------------------------------*
 *	AVM A1 and AVM Fritz! Card special registers
 *---------------------------------------------------------------------------*/

#define	AVM_CONF_REG	0x1800		/* base offset for config register */
#define	AVM_CONF_IRQ	0x1801		/* base offset for IRQ register    */
					/* config register write           */
#define	 AVM_CONF_WR_RESET	0x01	/* 1 = RESET ISAC and HSCX         */
#define	 AVM_CONF_WR_CCL	0x02	/* 1 = clear counter low nibble    */
#define	 AVM_CONF_WR_CCH	0x04	/* 1 = clear counter high nibble   */
#define	 AVM_CONF_WR_IRQEN	0x08	/* 1 = enable IRQ                  */
#define	 AVM_CONF_WR_TEST	0x10	/* test bit                        */
					/* config register read            */
#define	 AVM_CONF_RD_IIRQ	0x01	/* 0 = ISAC IRQ active             */
#define	 AVM_CONF_RD_HIRQ	0x02	/* 0 = HSCX IRQ active             */
#define	 AVM_CONF_RD_CIRQ	0x04    /* 0 = counter IRQ active          */
#define	 AVM_CONF_RD_ZER1	0x08	/* unused, always read 0           */
#define	 AVM_CONF_RD_TEST	0x10	/* test bit read back              */
#define	 AVM_CONF_RD_ZER2	0x20	/* unused, always read 0           */

#define AVM_ISAC_R_OFFS		(0x1400)
#define AVM_HSCXA_R_OFFS	(0x400)
#define AVM_HSCXB_R_OFFS	(0xc00)
#define AVM_ISAC_F_OFFS		(0x1400-0x20-0x3e0)
#define AVM_HSCXA_F_OFFS	(0x400-0x20-0x3e0)
#define AVM_HSCXB_F_OFFS	(0xc00-0x20-0x3e0)

#define IPAC_BUS_VAR(sc)			\
	bus_space_tag_t    t;			\
	bus_space_handle_t h;

#define IPAC_BUS_SETUP(reg)				\
	if((reg) & 0x80)				\
	{						\
	    if((reg) & 0x20)				\
	    {						\
		/* ISAC REGISTER */			\
	        t = sc->sc_resources.io_tag[1];		\
	        h = sc->sc_resources.io_hdl[1];		\
	    }						\
	    else					\
	    {						\
	        /* ISAC FIFO */				\
	        t = sc->sc_resources.io_tag[4];		\
	        h = sc->sc_resources.io_hdl[4];		\
	    }						\
	}						\
	else						\
	{						\
	    if((reg) & 0x40)				\
	    {						\
	        if((reg) & 0x20)			\
		{					\
		    /* HSCX B REGISTER */		\
		    t = sc->sc_resources.io_tag[3];	\
		    h = sc->sc_resources.io_hdl[3];	\
		}					\
		else					\
		{					\
		    /* HSCX B FIFO */			\
		    t = sc->sc_resources.io_tag[6];	\
		    h = sc->sc_resources.io_hdl[6];	\
		}					\
	    }						\
	    else					\
	    {						\
	        if((reg) & 0x20)			\
		{					\
		    /* HSCX A REGISTER */		\
		    t = sc->sc_resources.io_tag[2];	\
		    h = sc->sc_resources.io_hdl[2];	\
		}					\
		else					\
		{					\
		    /* HSCX A FIFO */			\
		    t = sc->sc_resources.io_tag[5];	\
		    h = sc->sc_resources.io_hdl[5];	\
		}					\
	    }						\
	}

static void
avm_a1_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(reg);

	/* read ISAC, HSCX A or HSCX B */
	bus_space_read_multi_1
	  (t,h,(reg) & ~(0x80|0x40|0x20),(ptr),(len));
	return;
}

static void
avm_a1_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	avm_a1_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
avm_a1_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(reg);

	/* write ISAC, HSCX A or HSCX B */
	bus_space_write_multi_1
	  (t,h,(reg) & ~(0x80|0x40|0x20),(ptr),(len));
	return;
}

static void
avm_a1_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	avm_a1_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
avm_a1_chip_reset CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);
	uint8_t temp1, temp2;

	t = sc->sc_resources.io_tag[0];
	h = sc->sc_resources.io_hdl[0];

  	/* AVM A1 or Fritz! control register bits:
	 *        read                write
	 * 0x01  hscx irq*           RESET
	 * 0x02  isac irq*           clear counter1
	 * 0x04  counter irq*        clear counter2
	 * 0x08  always 0            irq enable
	 * 0x10  read test bit       set test bit
	 * 0x20  always 0            unused
	 */

	temp1 = bus_space_read_1(t, h, 0);
	
	/* write low to test bit */

	bus_space_write_1(t, h, 0, 0x00);

	DELAY(500);
	
	/* test bit and next higher and lower bit should be 0 */

	if((temp2 = bus_space_read_1(t, h, 0) & 0x38) != 0x00)
	{
		IHFC_ADD_ERR(error, "probe-1 failed, "
			     "0x%02x should be 0x00 "
			     "for AVM A1/Fritz!", temp2);

		/* restore register value */
		bus_space_write_1(t, h, 0, temp1);
		return;
	}

	/* write high to test bit */

	bus_space_write_1(t, h, 0, 0x10);
	
	/* test bit must be high, next higher and lower bit should be 0 */

	if((temp2 = bus_space_read_1(t, h, 0) & 0x38) != 0x10)
	{
		IHFC_ADD_ERR(error, "probe-2 failed, "
			     "0x%02x should be 0x10 "
			     "for AVM A1/Fritz!", temp2);

		/* restore register value */
		bus_space_write_1(t, h, 0, temp1);
		return;
	}

	/* reset ISAC/HSCX */

	bus_space_write_1(t, h, 0, 0x00);
	DELAY(500);

	bus_space_write_1(t, h, 0, AVM_CONF_WR_RESET);
	DELAY(500);

	bus_space_write_1(t, h, 0, 0x00);
	DELAY(500);

	/* setup IRQ */

	bus_space_write_1(t, h, 1, sc->sc_resources.iirq[0]);

	DELAY(500);

	/* enable IRQ, disable counter IRQ */

	bus_space_write_1(t, h, 0, AVM_CONF_WR_IRQEN |
				AVM_CONF_WR_CCH | AVM_CONF_WR_CCL);
	DELAY(500);

	/*
	 * call reset verify routine
	 */

	avm_a1_chip_reset_verify(sc,error);

	return;
}

static int
avm_a1_chip_identify(device_t dev)
{
	struct resource * res;
	uint32_t rid;
	uint32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0 /* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "iobase for AVM A1/Fritz!\n",
			      __FUNCTION__);
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	switch(base) {
	case 0x200:
	case 0x240:
	case 0x300:
	case 0x340:		
		break;

	default:
		device_printf(dev, 
			      "%s: ERROR: invalid iobase [0x%x]!\n",
			      __FUNCTION__, base);
		return ENXIO;
	}

	/* setup I/O-ports */

	bus_set_resource(dev, SYS_RES_IOPORT, 0, base+AVM_CONF_REG, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 1, base+AVM_ISAC_R_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 2, base+AVM_HSCXA_R_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 3, base+AVM_HSCXB_R_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 4, base+AVM_ISAC_F_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 5, base+AVM_HSCXA_F_OFFS, 0x20);
	bus_set_resource(dev, SYS_RES_IOPORT, 6, base+AVM_HSCXB_F_OFFS, 0x20);

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IRQ, &rid, 0/* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "an irq for AVM A1/Fritz!\n",
			      __FUNCTION__);
		return ENXIO;
	}

	/* get IRQ */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IRQ, rid, res);

	/* check IRQ validity */

	switch(base) {
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		break;
			
	default:
		device_printf(dev, "%s: ERROR: invalid IRQ [%d]"
			      "specified for AVM A1/Fritz!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}
	return(0);
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(avm_a1_dbase_root)
{
  I4B_DBASE_ADD(c_chip_identify    , &avm_a1_chip_identify);
  I4B_DBASE_ADD(c_chip_reset       , &avm_a1_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &avm_a1_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &avm_a1_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &avm_a1_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &avm_a1_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &avm_a1_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &avm_a1_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &avm_a1_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &avm_a1_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &avm_a1_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &avm_a1_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &avm_a1_fifo_get_program);

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
  I4B_DBASE_ADD(o_RES_IOPORT_4     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_5     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_6     , 1); /* enable */
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(avm_a1_dbase_root);
  I4B_DBASE_ADD(desc          , "AVM A1 or Fritz!Card Classic");
}

I4B_ISA_DRIVER(.vid = 0x0D /* card number */);

/* cleanup */

#undef avm_a1_chip_reset_verify
#undef avm_a1_fifo_get_program
#undef avm_a1_fsm_read
#undef avm_a1_fsm_write
#undef avm_a1_chip_unselect
#undef avm_a1_chip_status_read
#undef avm_a1_chip_status_check
#undef avm_a1_fsm_table

#undef AVM_CONF_REG
#undef AVM_CONF_IRQ

#undef  AVM_CONF_WR_RESET
#undef  AVM_CONF_WR_CCL
#undef  AVM_CONF_WR_CCH
#undef  AVM_CONF_WR_IRQEN
#undef  AVM_CONF_WR_TEST

#undef  AVM_CONF_RD_IIRQ
#undef  AVM_CONF_RD_HIRQ
#undef  AVM_CONF_RD_CIRQ
#undef  AVM_CONF_RD_ZER1
#undef  AVM_CONF_RD_TEST
#undef  AVM_CONF_RD_ZER2

#undef AVM_ISAC_R_OFFS
#undef AVM_HSCXA_R_OFFS
#undef AVM_HSCXB_R_OFFS
#undef AVM_ISAC_F_OFFS
#undef AVM_HSCXA_F_OFFS
#undef AVM_HSCXB_F_OFFS

#undef IPAC_BUS_VAR
#undef IPAC_BUS_SETUP
#endif /* _I4B_AVM_A1_H_ */
