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
 *	i4b_hfcspci.h - HFC-2BDS0 PCI driver module
 * 	-------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFCSPCI_H_
#define _I4B_HFCSPCI_H_

#include <i4b/layer1/ihfc3/i4b_hfc.h>

/* imports */

#define hfcspci_chip_status_check   default_chip_status_check
#define hfcspci_chip_unselect       default_chip_unselect
#define hfcspci_fifo_select         default_fifo_select
#define hfcspci_fifo_inc_fx_pre     default_fifo_inc_fx_pre
#define hfcspci_fifo_frame_check    hfcs_fifo_frame_check
#define hfcspci_fsm_table           hfc_fsm_table

/*
 * NOTE: when the D-channel and B-channels are not in use, 
 * output bits must be forced to "1" by writing to SCTRL and
 * SCTRL_E registers, and not just by disabling the FIFO !
 */

#define HFCSPCI_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.mem_hdl[0];

#define HFCSPCI_READ_1(reg,var)			\
	(var) = bus_space_read_1(t,h,(reg))

#define HFCSPCI_WRITE_1(reg,var)		\
	bus_space_write_1(t,h,(reg),(var))

#define HFCSPCI_READ_MULTI_1(reg,ptr,len)		\
	bus_space_read_multi_1(t,h,(reg),(ptr),(len))

#define HFCSPCI_WRITE_MULTI_1(reg,ptr,len)		\
	bus_space_write_multi_1(t,h,(reg),(ptr),(len))

static void
hfcspci_t125_sync FIFO_SYNC_T(sc)
{
#if 0
	HFCSPCI_BUS_VAR(sc);
	u_int8_t temp;

	/* NOTE: t125_sync is used to prevent
	 * infinity loops if memory becomes corrupted.
	 */

	/* disable interrupts */
	cli(sc);

	/* enable sync(R) */
	sc->sc_config.s_int_m2_pci |= 1;
	HFCSPCI_WRITE_1(0x6C, sc->sc_config.s_int_m2_pci);

	/* reset sync(R) */
	HFCSPCI_READ_1(0x78, temp);
	sc->sc_config.s_int_s1 |= temp;

	/* enable interrupts */
	sti(sc);

	SC_T125_WAIT_SET(sc);
#else
	/* ihfc_fifo_program(,) checks number of
	 * loops
	 */
#endif
	return;
}

static void
hfcspci_chip_reset CHIP_RESET_T(sc,error)
{
	HFCSPCI_BUS_VAR(sc);
	u_int8_t temp;

	/*
	 * Setup MWBA by a PCI config
	 * write:
	 */
 
	pci_write_config(sc->sc_device, 0x80,
			 sc->sc_resources.mwba_phys_start[0], 4);

	/*
	 * Manual says that a dword write to
	 * the MWBA PCI register will enable
	 * busmaster ...
	 *
	 * Enable busmaster so that the
	 * chip can write to MWBA (just in case).
	 */

	pci_enable_busmaster(sc->sc_device);

	/* perform reset
	 *
	 * NOTE: When the chip is reset it will
	 *	 write default F- and Z-values
	 *	 for all channels to MWBA.
	 *
	 * write cirm(reg=0x60)
	 */
	HFCSPCI_WRITE_1(0x60, (sc->sc_config.s_cirm_pci ^ 0x08));
	DELAY(100);

	HFCSPCI_WRITE_1(0x60, sc->sc_config.s_cirm_pci);
	DELAY(100);

	/* read status(reg=0x70) */
	HFCSPCI_READ_1(0x70, temp);
	if((temp & 0x01) != 0)
	{
		IHFC_ADD_ERR(error,"(BUSY != 0)");
	}

	/* read states(reg=0xC0) */
	HFCSPCI_READ_1(0xC0, temp);
	if((temp & 0x0f) != 0)
	{
		IHFC_ADD_ERR(error,"(STATES != 0)");
	}

	/* The CHIP-ID register, 0x58, is not
	 * checked in case of unknown clones
	 */
	return;
}

static void
hfcspci_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	if ((f == IHFC_CONFIG_WRITE_UPDATE) ||
	    (f == IHFC_CONFIG_WRITE_RELOAD)) {

	  ihfc_config_write_sub(sc,f);

	} else {

	  /* reset FIFO */
	  u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
			       (f->fm.h.Zdata)+(f->fm.h.Zend - f->fm.h.Zsize);
	  u_int16_t fifo_len = (f->fm.h.Zsize);

	  /* fill FIFO with 0xFF bytes */
	  memset_1(fifo_ptr, 0xFF, fifo_len);
	}
	return;
}

static void
hfcspci_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	/* HFC-SPCI 
	 * external variables: len (destination), ptr (destination)
	 */
	register u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
	                              (f->fm.h.Zdata)+(f->Z_drvr);
	register u_int16_t fifo_len = (f->fm.h.Zend)-(f->Z_drvr);

	/* pre increment Z-counter (before `len` is changed) */
	(f->Z_drvr) += (len);

	if(len >= fifo_len)
	{
	  /* Wrapped read:
	   * fifo.curr |---> fifo.end
	   */
	  bcopy(fifo_ptr,ptr,fifo_len);

	       len -= fifo_len;
	       ptr += fifo_len;
	  fifo_ptr += fifo_len;

	  /* get ``min'' value */
	  (f->Z_drvr) -=   (f->fm.h.Zsize);
	  (fifo_ptr)  -= 1*(f->fm.h.Zsize);
	}

	/* Continuous read
	 * fifo.curr |---> fifo.Z_drvr(after pre increment)
	 */
	bcopy(fifo_ptr,ptr,len);

	/* write Z_drvr (driver incremented) to MWBA */
	*(__volatile__ u_int16_t *)(f->Z_ptr) = f->Z_drvr;

	return;
}

static void
hfcspci_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	HFCSPCI_BUS_VAR(sc);
#if 1
	/* XXX bug in /sys/i386/include/bus.h XXX */
	if(len == 0)
	{
		return;
	}
#endif

	HFCSPCI_READ_MULTI_1((reg & 0xFF), ptr, len);
	return;
}

static void
hfcspci_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	/* HFC-SPCI
	 * external variables: len (source), ptr (source)
	 */
	register u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
				      (f->fm.h.Zdata)+(f->Z_drvr);
	register u_int16_t fifo_len = (f->fm.h.Zend)-(f->Z_drvr);

	/* pre increment Z-counter (before `len` is changed) */
	(f->Z_drvr) += (len);

	if(len >= fifo_len)
	{
	  /* Wrapped write:
	   * fifo.curr |---> fifo.end
	   */
	  bcopy(ptr,fifo_ptr,fifo_len);

	       len -= fifo_len;
	       ptr += fifo_len;
	  fifo_ptr += fifo_len;

	  /* get ``min'' value */
	  (f->Z_drvr)   -=   (f->fm.h.Zsize);
	  (fifo_ptr)    -= 1*(f->fm.h.Zsize);
	}

	/* Continuous write 
	 * fifo.curr |---> fifo.Z_drvr(after pre increment)
	 */
	bcopy(ptr,fifo_ptr,len);

	/* write Z_drvr (driver incremented) to MWBA */
	*(__volatile__ u_int16_t *)(f->Z_ptr) = f->Z_drvr;

	return;
}

static void
hfcspci_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	HFCSPCI_BUS_VAR(sc);
#if 1
	/* XXX bug in /sys/i386/include/bus.h XXX */
	if(len == 0)
	{
	    return;
	}
#endif
	HFCSPCI_WRITE_MULTI_1((reg & 0xFF), ptr, len);
	return;
}

static void
hfcspci_fsm_read FSM_READ_T(sc,f,ptr)
{
	HFCSPCI_BUS_VAR(sc);
	u_int8_t temp;

	/* read STATES(reg=0xC0) */
	HFCSPCI_READ_1(0xC0, temp);
	*ptr = (temp +
		((sc->sc_config.s_sctrl & 0x4) ?
		 HFC_NT_OFFSET : HFC_TE_OFFSET)) & 0xf;
	return;
}

static void
hfcspci_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	HFCSPCI_BUS_VAR(sc);

	/* write STATES(reg=0xC0) */
	HFCSPCI_WRITE_1(0xC0, *ptr);

#ifdef HFC_FSM_RESTART
	if((*ptr) & 0x10) {
	    /* 5.21us delay */

 	    DELAY(10);

	    HFCSPCI_WRITE_1(0xC0, ((*ptr) ^ 0x10));
	}
#endif
	return;
}

static void
hfcspci_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	/* next F-entry */
	if(++(f->F_drvr) & 0x20) {
	     (f->F_drvr) -= (f->fm.h.Fsize);
	     (f->Z_ptr ) -= 4*(f->fm.h.Fsize);
	}

	/* next Z-entry
	 * NOTE: Z_ptr has byte granularity
	 */
	(f->Z_ptr) += 4;

	/* write Z_drvr counter to MWBA */
	*(__volatile__ u_int16_t *)(f->Z_ptr) = (f->Z_drvr);

	/* write F_drvr counter to MWBA */
	*(__volatile__ u_int8_t  *)(f->F_ptr) = (f->F_drvr);

#if 0
	/* NOTE: in some special cases the
	 * Z_drvr counter may be changed
	 * after a F_drvr increment.
	 * Reload FZ:
	 */

	f->state &= ~ST_FZ_LOADED;
#endif
	hfcspci_t125_sync(sc);

	return;
}

static void
hfcspci_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	u_int16_t Z_chip2;

	/* read F- and Z-counters respectively
	 *
	 * disable interrupts to
	 * run code at maximum speed
	 */
	cli(sc);

	/* get current time */

	f->Z_read_time = ihfc_get_f0_counter(sc);

	/*
	 * F_drvr is needed to compute the MWBA
	 * offset for Z_chip and Z_drvr, and is
	 * read every time this procedure is
	 * called.
	 *
	 * F_drvr is not buffered, hence the
	 * counter is already in host memory
	 * (MWBA)
	 *
	 * F_drvr and Z_drvr counters should not
	 * change during read and are only read
	 * once. The Z_chip counter is read twice.
	 *
	 * mwba_start[0] is aligned to 0x8000 or
	 * 32Kbyte in memory.
	 *
	 * f->Z_drvr and f->Z_chip is within the
	 * MWBA for any 8-bit value of f->F_drvr
	 */
	   
	/* get F_chip pointer */
	(f->F_ptr) = (u_int8_t *)(sc->sc_resources.mwba_start[0]
				  +f->fm.h.Fbase);
	/* read F_chip from MWBA */
	(f->F_chip) = *(__volatile__ u_int8_t  *)(f->F_ptr);

	/* get F_drvr pointer */
	(f->F_ptr) = (u_int8_t *)(sc->sc_resources.mwba_start[0]
				  +(f->fm.h.Fbase ^ 1));

	/* read F_drvr from MWBA */
	(f->F_drvr) = *(__volatile__ u_int8_t  *)(f->F_ptr);

	/* mask F_drvr and F_chip */
	(f->F_drvr)  &=  0x001f;
	(f->F_chip)  &=  0x001f;

	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  (f->F_drvr)  |=  0x0010; /* exception */
	  (f->F_chip)  |=  0x0010; /* exception */
	}

	/* get Z_chip pointer (using F_drvr) */
	(f->Z_ptr) = (sc->sc_resources.mwba_start[0]
		      +f->fm.h.Zbase +(4*(f->F_drvr)));

	/* read Z_chip from MWBA */
#ifdef __i386__

	/*
	 * NOTE: the chip updates all counters with a 32-bit PCI
	 * host memory write.  If the host processor cannot read
	 * 16 bits in one instruction, Z_chip must be read _two_
	 * times. See macro ``INC_COUNTER_TIME_CHECK()'' for more
	 * information.
	 */

	(   Z_chip2) =
	(f->Z_chip ) = *(__volatile__ u_int16_t *)(f->Z_ptr);
#else
	(   Z_chip2) = ((__volatile__ u_int8_t  *)(f->Z_ptr))[1] << 8; /* zx - high  */
	(f->Z_chip ) = ((__volatile__ u_int8_t  *)(f->Z_ptr))[0];      /* zx - low   */
	(f->Z_chip )|= ((__volatile__ u_int8_t  *)(f->Z_ptr))[1] << 8; /* zx - high  */
#endif

	/* get Z_drvr pointer */
	(f->Z_ptr) = (sc->sc_resources.mwba_start[0]
		      +(f->fm.h.Zbase^2) +(4*(f->F_drvr)));

	/* read Z_drvr from MWBA */
	(f->Z_drvr) = *(__volatile__ u_int16_t *)(f->Z_ptr);

#if 0
	if(!(f->state &  ST_FZ_LOADED)) {
	     f->state |= ST_FZ_LOADED;
	}
#endif

	/* NOTE:  HFC-PCI Z-counters are 
	 * invalid  in  some cases,  but
	 * those cases should be ignored!
	 */

	/* mask Z_chip, Z_chip2 and Z_drvr */
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* D-channel */

	  (f->Z_chip)  &=  0x01ff;
	  (   Z_chip2) &=  0x01ff; /* copy of f->Z_chip */
	  (f->Z_drvr)  &=  0x01ff;
	}
	else
	{
	  /* B-channel */

	  (f->Z_chip)  &=  0x1fff;
	  (   Z_chip2) &=  0x1fff; /* copy of f->Z_chip */
	  (f->Z_drvr)  &=  0x1fff;
	}

	/* analyse/correct the counters */
	INC_COUNTER_TIME_CHECK(f->Z_chip,    Z_chip2);

	/* Z_ptr and F_ptr are used by fifo_read,
	 * fifo_write and fifo_inc_fx
	 *
	 * enable interrupts
	 */
	sti(sc);

	return;
}

static void
hfcspci_chip_status_read CHIP_STATUS_READ_T(sc)
{
	HFCSPCI_BUS_VAR(sc);
	u_int8_t temp;

	/* disable interrupts */
	cli(sc);

	/* read status(reg=0x70) and s_int_s1(reg=0x78)  */
	HFCSPCI_READ_1(0x70, temp);
	sc->sc_config.s_status = temp;

	HFCSPCI_READ_1(0x78, temp);
	sc->sc_config.s_int_s1 |= temp;

	/* check if t125 is running */
	if(sc->sc_config.s_int_m2_pci & 1)
	{
		/* check busy/non-busy transition flag */
		if(sc->sc_config.s_status & 0x04)
		{
			/* disable sync(R)
			 * write s_int_m2_pci(reg=0x6c)
			 */

			sc->sc_config.s_int_m2_pci &= ~1;
			HFCSPCI_WRITE_1(0x6C, sc->sc_config.s_int_m2_pci);

			SC_T125_WAIT_CLEAR(sc);
		}
	}

	/* enable interrupts */
	sti(sc);

	return;
}

static ihfc_fifo_program_t *
hfcspci_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if((FIFO_NO(f) == d1t) ||
	   (FIFO_NO(f) == d1r))
	{
	  if(PROT_IS_HDLC(&(f->prot_curr)))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_hfc_tx_program :
			&i4b_hfc_rx_program;
	  }
	}
	else
	{
	  if(HFC_USE_HARDWARE_HDLC(PROT_IS_HDLC(&(f->prot_curr)) ||)
	     PROT_IS_TRANSPARENT(&(f->prot_curr)))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_hfc_tx_program :
			&i4b_hfc_rx_program;
	  }
	}
	return program;
}

/*-------------------------------------------------+
 | hfcspci related information                     |
 +-------------------------------------------------*/

register_list_t
hfcspci_register_list [] =
{
  { REG2OFF(s_cirm_pci),   0x60 },
  { REG2OFF(s_ctmt_pci),   0x64 },
  { REG2OFF(s_int_m1),     0x68 },
  { REG2OFF(s_int_m2_pci), 0x6C }, /* ~s_int_m2(isa) */
  { REG2OFF(s_mst_emod),   0xB4 },
  { REG2OFF(s_clkdel),     0xDC },
  { REG2OFF(s_connect),    0xBC }, /* must be loaded before SCTRL */
  { REG2OFF(s_sctrl),      0xC4 },
  { REG2OFF(s_sctrl_e_pci),0xC8 },
  { REG2OFF(s_sctrl_r),    0xCC },
  { REG2OFF(s_trm),        0x48 },
  { REG2OFF(s_fifo_en),    0x44 },
  { REG2OFF(s_b1_ssl),     0x80 },
  { REG2OFF(s_b2_ssl),     0x84 },
  { REG2OFF(s_a1_ssl),     0x88 },
  { REG2OFF(s_a2_ssl),     0x8C },
  { REG2OFF(s_b1_rsl),     0x90 },
  { REG2OFF(s_b2_rsl),     0x94 },
  { REG2OFF(s_a1_rsl),     0x98 },
  { REG2OFF(s_a2_rsl),     0x9C },
  { REG2OFF(s_mst_mode),   0xb8 },
  { 0, 0 }
};

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(hfcspci_dbase_root)
{
  I4B_DBASE_ADD(c_chip_read          , &hfcspci_chip_read);
  I4B_DBASE_ADD(c_chip_write         , &hfcspci_chip_write);
  I4B_DBASE_ADD(c_chip_reset         , &hfcspci_chip_reset);
  I4B_DBASE_ADD(c_chip_unselect      , &hfcspci_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read   , &hfcspci_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check  , &hfcspci_chip_status_check);
  I4B_DBASE_ADD(c_chip_config_write  , &hfcspci_chip_config_write);

  I4B_DBASE_ADD(c_fsm_read           , &hfcspci_fsm_read);
  I4B_DBASE_ADD(c_fsm_write          , &hfcspci_fsm_write);
  I4B_DBASE_ADD(d_fsm_table          , &hfcspci_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program   , &hfcspci_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read          , &hfcspci_fifo_read);
  I4B_DBASE_ADD(c_fifo_write         , &hfcspci_fifo_write);
  I4B_DBASE_ADD(c_fifo_select        , &hfcspci_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx        , &hfcspci_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read       , &hfcspci_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre    , &hfcspci_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_frame_check   , &hfcspci_fifo_frame_check);

  I4B_DBASE_ADD(d_register_list      , &hfcspci_register_list[0]);
  I4B_DBASE_ADD(d_channels           , 6);

  /* delay 50 milliseconds */ 
  I4B_DBASE_ADD(d_interrupt_delay    , hz / 20);
 
  I4B_DBASE_ADD(desc                 , "HFC-2BDS0 128K PCI ISDN adapter");

  I4B_DBASE_ADD(o_RES_IRQ_0          , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0       , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0       , 1); /* enable */
  I4B_DBASE_ADD(o_HFC_MWBA           , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */
  I4B_DBASE_ADD(o_ECHO_CANCEL_ENABLED, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask      , (I4B_OPTION_POLLED_MODE|
					I4B_OPTION_NT_MODE|
					I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value     , 0);

  I4B_DBASE_ADD(io_rid[0]            , PCIR_BAR(0));
  I4B_DBASE_ADD(mem_rid[0]           , PCIR_BAR(1));

  /*
   * 32kbyte fifo mode selected
   */

  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(hfc_fifo_map[12]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(hfc_fifo_map[13]));
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(hfc_fifo_map[14]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(hfc_fifo_map[15]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(hfc_fifo_map[16]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(hfc_fifo_map[17]));
}

I4B_PCI_DRIVER(/* CCD HFC PCI A - generic
		* Xplorer 500
		*/
	       .vid = 0x2bd01397);

I4B_PCI_DRIVER(/* ASUSCOM HFC-S PCI A */
	       .vid = 0x06751043,
	       .sub = 0x17040675);

I4B_PCI_DRIVER(/* ASUSCOM HFC-S PCI A */
	       .vid = 0x06751704);

I4B_PCI_DRIVER(/* ASUSCOM HFC-S PCI A */
	       .vid = 0x147A1100);

I4B_PCI_DRIVER(/* Motorola MC145575 */
	       .vid = 0x01001057);

I4B_PCI_DRIVER(/* ISDN PCI DC-105V2 */
	       .vid = 0x3069182d,
	       .sub = 0x3069182d);

I4B_PCI_DRIVER(/* PrimuX S0 */
		.vid = 0xb7001397);

#undef HFCSPCI_BUS_VAR
#undef HFCSPCI_READ_1
#undef HFCSPCI_WRITE_1
#undef HFCSPCI_READ_MULTI_1
#undef HFCSPCI_WRITE_MULTI_1

#endif /* _I4B_HFCSPCI_H_ */
