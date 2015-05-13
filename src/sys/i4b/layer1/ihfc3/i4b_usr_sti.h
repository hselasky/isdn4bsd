/*-
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
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
 *	USRobotics Sportster ISDN TA intern (Tina-pp) card driver
 *	---------------------------------------------------------
 *
 *      last edit-date: []
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_USR_STI_H_
#define _I4B_USR_STI_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

/* imports */

#define usrtai_chip_reset_verify  isac_hscx_generic_chip_reset_verify
#define usrtai_fifo_get_program   isac_hscx_generic_fifo_get_program
#define usrtai_fsm_read           isac_hscx_generic_fsm_read
#define usrtai_fsm_write          isac_hscx_generic_fsm_write
#define usrtai_chip_unselect      isac_hscx_generic_chip_unselect
#define usrtai_chip_status_read   isac_hscx_generic_chip_status_read
#define usrtai_chip_status_check  default_chip_status_check
#define usrtai_fsm_table          isac_fsm_table


/*---------------------------------------------------------------------------*
 *	USR Sportster TA intern special registers
 *---------------------------------------------------------------------------*/
#define USR_HSCXA_OFF	0x0000
#define USR_HSCXB_OFF	0x4000
#define USR_INTL_OFF	0x8000
#define USR_ISAC_OFF	0xc000

#define USR_RES_BIT	0x80	/* 0 = normal, 1 = reset ISAC/HSCX	*/
#define USR_INTE_BIT	0x40	/* 0 = IRQ disabled, 1 = IRQ's enabled	*/
#define USR_IL_MASK	0x07	/* IRQ level config			*/

#define ADDR(reg) ((((reg) / 4) * 1024) + (((reg) % 4) * 2))

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[1];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[1];


#define IPAC_BUS_SETUP(sc,reg)			\
	uint16_t base;				\
	if((reg) & 0x80)			\
	{					\
		base = USR_ISAC_OFF;		\
	}					\
	else					\
	{					\
		if((reg) & 0x40)		\
		{				\
			base = USR_HSCXB_OFF;	\
		}				\
		else				\
		{				\
			base = USR_HSCXA_OFF;	\
		}				\
	}					\
	(reg) &= ~(0x80|0x40);


static void
usrtai_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);

	if(reg & 0x20)
	{
	  /* REGISTER */
	  bus_space_read_multi_1(t,h,base + ADDR(reg),ptr,len);
	}
	else
	{
	  /* XXX
	   * XXX it should be possible to use 
	   * XXX "bus_space_read_multi_1" here,
	   * XXX which is much faster !
	   * XXX
	   */
	  while(len--)
	  {
	    *ptr = bus_space_read_1(t,h,base + ADDR(reg));
	    ptr++;
	    reg++;
	  }
	}
	return;
}

static void
usrtai_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	usrtai_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
usrtai_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);
	IPAC_BUS_SETUP(sc,reg);

	if(reg & 0x20)
	{
	  /* REGISTER */
	  bus_space_write_multi_1(t,h,base + ADDR(reg),ptr,len);
	}
	else
	{
	  /* XXX
	   * XXX it should be possible to use 
	   * XXX "bus_space_write_multi_1" here,
	   * XXX which is much faster !
	   * XXX
	   */

	  /* FIFO */
	  while(len--)
	  {
	    bus_space_write_1(t,h,base + ADDR(reg), *ptr);
	    ptr++;
	    reg++;
	  }
	}
	return;
}

static void
usrtai_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	usrtai_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
usrtai_chip_reset CHIP_RESET_T(sc,error)
{
	static uint8_t intr_no[] = { 0, 0, 0, 0, 0, 1, 0, 2, 0, 0, 3, 4, 5, 0, 6, 7 };
	uint8_t temp;
	IPAC_BUS_VAR(sc);

	/* reset the HSCX and ISAC chips */
	
	bus_space_write_1(t,h,USR_INTL_OFF, USR_RES_BIT);
	DELAY(500);

	bus_space_write_1(t,h,USR_INTL_OFF, 0x00);
	DELAY(500);

	/* setup IRQ */

	temp = intr_no[sc->sc_resources.iirq[0]];

	/* configure and enable irq */

	bus_space_write_1(t,h,USR_INTL_OFF, temp | USR_INTE_BIT);
	DELAY(500);

	/*
	 * call reset verify routine
	 */

	usrtai_chip_reset_verify(sc,error);

	return;
}

static void
usrtai_setup_io_ports(device_t dev, uint16_t base)
{ 
	int i, num = 0;

	/* 49 I/O mappings: 1 config and 48x8 registers */

	/* config at offset 0x8000 */
	bus_set_resource(dev, SYS_RES_IOPORT, num, 
			 base + 0x8000, 1);
	num++;

	/* HSCX A at offset 0x0000 */
	for(i = 0; i < 16; i++)
	{
		bus_set_resource(dev, SYS_RES_IOPORT, num, 
				 base + 0x0000 + (i*1024), 8);
		num++;
	}
	/* HSCX B at offset 0x4000 */
	for(i = 0; i < 16; i++)
	{
		bus_set_resource(dev, SYS_RES_IOPORT, num, 
				 base + 0x4000 + (i*1024), 8);
		num++;
	}
	/* ISAC at offset 0xC000 */
	for (i = 0; i < 16; i++)
	{
		bus_set_resource(dev, SYS_RES_IOPORT, num, 
				 base + 0xC000 + (i*1024), 8);
		num++;
	}
	return;
}

static int
usrtai_allocate_io_ports(device_t dev)
{
	uint32_t rid[49];
	struct resource * res[49];
	int i;

	for(i = 0; i < 49; i++)
	{
		rid[i] = i;
		res[i] = bus_alloc_resource_any
		  (dev, SYS_RES_IOPORT, &rid[i], RF_ACTIVE);

		if(res[i] == NULL) break;
	}

	if(i < 49)
	{
		while(i--)
		{
			bus_release_resource
			  (dev, SYS_RES_IOPORT, rid[i], res[i]);
		}

		device_printf(dev, "%s: could not allocate all "
			      "I/O ports for USR Sportster TA!\n",
			      __FUNCTION__);

		return ENXIO;
	}

#if 1
	/* release all I/O ports, hence it is currently
	 * too many to handle
	 *
	 * NOTE: some resources will be allocated
	 * later and must be released !
	 */
	while(i--)
	{
		bus_release_resource
		  (dev, SYS_RES_IOPORT, rid[i], res[i]);
	}
#endif
	return 0;
}

static int
usrtai_chip_identify(device_t dev)
{
	struct resource * res;
	uint32_t rid;
	uint32_t base;

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IOPORT, &rid, 0 /* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "iobase for USR Sportster TA!\n",
			      __FUNCTION__);
		return(ENXIO);
	}

	/* get I/O base */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IOPORT, rid, res);

	switch(base) {
	case 0x200:
	case 0x208:
	case 0x210:
	case 0x218:
	case 0x220:
	case 0x228:
	case 0x230:
	case 0x238:
	case 0x240:
	case 0x248:
	case 0x250:
	case 0x258:
	case 0x260:
	case 0x268:
	case 0x270:
	case 0x278:
		break;
			
	default:
		device_printf(dev, 
			      "%s: ERROR: invalid iobase [0x%x] "
			      "specified for USR Sportster TA!\n",
			      __FUNCTION__, base);
		return ENXIO;
	}

#if ((0x278 + 0xC000 + (1024*15) + 7) >= 0x10000)
#error "I/O port is too high!"
#endif

	usrtai_setup_io_ports(dev, base);

	rid = 0;

	if(!(res = bus_alloc_resource_any
	     (dev, SYS_RES_IRQ, &rid, 0/* !RF_ACTIVE */)))
	{
		device_printf(dev, "%s: ERROR: could not get "
			      "an irq for USR Sportster TA!\n",
			      __FUNCTION__);
		return ENXIO;
	}

	/* get IRQ */

	base = rman_get_start(res);

	bus_release_resource(dev, SYS_RES_IRQ, rid, res);

	/* check IRQ validity */

	switch(base) {
	case 5:
	case 7:
	case 10:
	case 11:
	case 12:
	case 14:
	case 15:
		break;

	default:
		device_printf(dev, "%s: ERROR: invalid IRQ [%d]"
			      "specified for USR Sportster TA!\n",
			      __FUNCTION__, base);
		return(ENXIO);
	}

	/* allocate I/O-ports last, so that one doesn't 
	 * have to worry about freeing the I/O-ports
	 * if something goes wrong
	 */
	if(usrtai_allocate_io_ports(dev))
	{
		return ENXIO;
	}
	return(0);
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(usrtai_dbase_root)
{
  I4B_DBASE_ADD(c_chip_identify    , &usrtai_chip_identify);
  I4B_DBASE_ADD(c_chip_reset       , &usrtai_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &usrtai_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &usrtai_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &usrtai_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &usrtai_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &usrtai_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &usrtai_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &usrtai_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &usrtai_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &usrtai_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &usrtai_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &usrtai_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_isac_hscx_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(i4b_option_mask      , (I4B_OPTION_POLLED_MODE|
					I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value     , 0);

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));

  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF (isac_fifo_map[1]));
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF (isac_fifo_map[1])); 
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF (isac_fifo_map[2])); 
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF (isac_fifo_map[2]));

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */

  /* XXX should be able to allocate more I/O-ports XXX */

  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_1     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_2     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_3     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_4     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_5     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_6     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_7     , 1); /* enable */
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(usrtai_dbase_root);
  I4B_DBASE_ADD(desc          , "USR Sportster TA");
}

I4B_ISA_DRIVER(.vid = 0x12 /* card number */);

/* cleanup */
#undef usrtai_chip_reset_verify
#undef usrtai_fifo_get_program
#undef usrtai_fsm_read
#undef usrtai_fsm_write
#undef usrtai_chip_unselect
#undef usrtai_chip_status_read
#undef usrtai_chip_status_check
#undef usrtai_fsm_table
#undef USR_HSCXA_OFF
#undef USR_HSCXB_OFF
#undef USR_INTL_OFF
#undef USR_ISAC_OFF
#undef USR_RES_BIT
#undef USR_INTE_BIT
#undef USR_IL_MASK
#undef ADDR
#undef IPAC_BUS_VAR
#undef IPAC_BUS_SETUP
#endif /* _I4B_USR_STI_H_ */
