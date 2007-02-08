/*-
 * Copyright (c) 1997, 2000 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 1999, 2000 Udo Schweigert. All rights reserved.
 *
 * Copyright (c) 1999-2001 Gary Jennejohn. All rights reserved.
 *
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
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
 *	i4b_avm_pci.h - AVM Fritz!Card v.2 PCI hardware driver
 *	------------------------------------------------------
 *
 * NOTE: This driver uses HDLC emulation, hence the chip requires data
 * transfer lengths to be pre-written, which is very impractical, for 
 * example when one is dealing with chained "mbufs".
 *---------------------------------------------------------------------------*/

#ifndef _I4B_AVM_PCI_H_
#define _I4B_AVM_PCI_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */
#define avm_pci_chip_status_check  default_chip_status_check
#define avm_pci_fsm_table          isac_fsm_table

/* register offsets from I/O-base */
#define REG_STAT0_OFFSET   	0x02
#define REG_ISACSX_INDEX   	0x04
#define REG_ISACSX_DATA    	0x08
#define REG_BCHAN_1             0x10
#define REG_BCHAN_2             0x18

/* not really a HSCX, but sort of */
#define HSCX_FIFO		0x00
#define HSCX_STAT		0x04
#define HSCX_LEN                0x05
#define HSCX_PROT		0x06

/* AVM PCI Status Latch 0 read only bits */
#define ASL_IRQ_ISAC            0x01    /* ISAC  interrupt, active low */
#define ASL_IRQ_HSCX            0x02    /* HSCX  interrupt, active low */
#define ASL_IRQ_TIMER           0x04    /* Timer interrupt, active low */

/* NOTE: high is active */
#define ASL_IRQ_PENDING         (ASL_IRQ_ISAC | ASL_IRQ_HSCX | ASL_IRQ_TIMER)

/* AVM PCI Status Latch 0 write only bits */
#define ASL_RESET_ALL           0x01  /* reset ISAC-SX, active 1 */
#define ASL_TIMERDISABLE        0x02  /* active high */
#define ASL_TIMERRESET          0x04  /* active high */
#define ASL_ENABLE_INT          0x08  /* active high */

/* HSCX mode bits */
#define  HSCX_MODE_ITF_FLG	0x01 /* HDLC mode */
#define  HSCX_MODE_TRANS	0x02 /* extended transparent mode */

/* HSCX status bits */
#define  HSCX_STAT_RME		0x01
#define  HSCX_STAT_RDO		0x10
#define  HSCX_STAT_CRCVFRRAB	0x0E
#define  HSCX_STAT_CRCVFR	0x06
#define  HSCX_STAT_RML_MASK	0x3f00

/* HSCX interrupt bits */
#define  HSCX_INT_XPR		0x80
#define  HSCX_INT_XDU		0x40
#define  HSCX_INT_RPR		0x20
#define  HSCX_INT_MASK		0xE0

/* HSCX command bits */
#define  HSCX_CMD_XRS		0x80
#define  HSCX_CMD_XME		0x01
#define  HSCX_CMD_RRS		0x20

#define IPAC_BUS_VAR(sc)				\
  bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
  bus_space_handle_t h = sc->sc_resources.mem_hdl[0];

static void
avm_pci_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
	    /* ISAC-SX REGISTER */
	    bus_space_write_4(t, h, REG_ISACSX_INDEX, (reg & 0x7F));
	    bus_space_read_multi_1(t, h, REG_ISACSX_DATA, ptr, len);
	}
	else
	{
	    bzero(ptr,len);
	}
	return;
}

static void
avm_pci_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	if(FIFO_NO(f) == d1r)
	{
	    avm_pci_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	}
	else
	{
	    bcopy(f->Z_ptr,ptr,len);

	    /* update data pointer */
	    f->Z_ptr += len;
	}
	return;
}

static void
avm_pci_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	if(reg & 0x80)
	{
	    /* ISAC-SX REGISTER */
	    bus_space_write_4(t, h, REG_ISACSX_INDEX, (reg & 0x7F));
	    bus_space_write_multi_1(t, h, REG_ISACSX_DATA, ptr, len);
	}
	else
	{
	    /* ignore */
	}
	return;
}

static void
avm_pci_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	if(FIFO_NO(f) == d1t)
	{
	    avm_pci_chip_write(sc,(f->fm.h.Zdata),ptr,len);
	}
	else
	{
	    if(len)
	    {
	        bcopy(ptr, f->Z_ptr, len);

		/* store the last byte */
		f->last_byte = ptr[len-1];

		/* update data pointer */
		f->Z_ptr += len;
	    }
	}
	return;
}

static void
avm_pci_fsm_read FSM_READ_T(sc,f,ptr)
{
	u_int8_t temp;

	/* read CIR0 */
	avm_pci_chip_read(sc, REG_isacsx_cir0, &temp, 1);

	*ptr = (temp >> 4) & 0xf;

	return;
}

static void
avm_pci_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	u_int8_t temp = ((*ptr) << 2) | 0x0E;

	if(IS_DLOWPRI(sc,0))
        {
	    temp |=  0x10;
	}

	/* write CIR0 */
	avm_pci_chip_write(sc, REG_isacsx_cir0, &temp, 1);
	return;
}

static void
avm_pci_b_status_read(ihfc_sc_t *sc, ihfc_fifo_t *f, u_int8_t offset)
{
	IPAC_BUS_VAR(sc);

	u_int8_t buffer[0x40 + 0x10]; /* allocate a buffer on the stack */
	u_int8_t temp;

	/* read status */
	temp = bus_space_read_1(t, h, offset + HSCX_STAT);

	IHFC_MSG("b_status=0x%02x\n", temp);

	if(temp)
	{
	  if(temp & HSCX_INT_RPR)
	  {
	      (f+receive)->i_ista |= I_ISTA_RPF;
	  }

	  if(temp & HSCX_STAT_RDO)
	  {
	      (f+receive)->i_ista |= I_ISTA_ERR;
	  }

	  if(temp & HSCX_INT_XPR)
	  {
	      (f+transmit)->i_ista |= I_ISTA_XPR;
	  }

	  if(temp & HSCX_INT_XDU)
	  {
	      (f+transmit)->i_ista |= I_ISTA_ERR;
	  }
	}

	/* RME or RPF - B channel receive */
	if(((f+receive)->i_ista & (I_ISTA_RPF|I_ISTA_ERR)) &&
	   ((f+receive)->prot_curr.protocol_1 != P_DISABLE))
	{
	    (f+receive)->i_ista &= ~(I_ISTA_RPF|I_ISTA_ERR);

	    temp = bus_space_read_1(t, h, offset + HSCX_LEN) & 0x3F;
	    if(temp == 0) temp = 32;

	    /* read FIFO */
	    bus_space_read_multi_4(t, h, offset + HSCX_FIFO, 
				   (void *)&buffer[0], (temp+3)/4);

	    (f+receive)->Z_ptr = &buffer[0];
	    (f+receive)->Z_chip = temp;

	    /* call filter */

	    ((f+receive)->filter)(sc, f+receive);
	}

	/* XPR - B channel transmit */
	if(((f+transmit)->i_ista & I_ISTA_XPR) &&
	   ((f+transmit)->prot_curr.protocol_1 != P_DISABLE))
	{
	    if((f+transmit)->i_ista & I_ISTA_ERR)
	    {
	        (f+transmit)->state |= ST_FRAME_ERROR;
	    }

	    temp = 32;

	    (f+transmit)->i_ista &= ~(I_ISTA_ERR|I_ISTA_XPR);
	    (f+transmit)->Z_ptr = &buffer[0];
	    (f+transmit)->Z_chip = temp;

	    /* call filter */

	    ((f+transmit)->filter)(sc, f+transmit);

	    /* fill unused FIFO space with 0xFF, which
	     * is what these chips send when there is
	     * no more data
	     */
	    memset_1((f+transmit)->Z_ptr, 0xFF,
		     (f+transmit)->Z_chip);

	    /* update Z_chip */
	    (f+transmit)->Z_chip = 0;

	    /* update state */
	    (f+transmit)->state &= ~(ST_FRAME_ERROR|ST_FRAME_END);

	    /* write FIFO */
	    bus_space_write_multi_4(t, h, offset + HSCX_FIFO, 
				    (void *)&buffer[0], (temp+3)/4);
	}
	return;
}

static void
avm_pci_chip_status_read CHIP_STATUS_READ_T(sc)
{
	IPAC_BUS_VAR(sc);

	u_int8_t status;
	u_int8_t ista_d;
	u_int8_t ista;
	u_int8_t temp;

	status = bus_space_read_1(t, h, REG_STAT0_OFFSET);

	IHFC_MSG("status=0x%02x\n", status);

	if(status & ASL_IRQ_ISAC)
	{
	    /*
	     * generic ISAC-SX code:
	     */

	    /* read ISTA (ISAC) */
	    avm_pci_chip_read(sc, REG_isacsx_ista, &ista, 1);

	    IHFC_MSG("ista=0x%02x\n", ista);

	    if(ista & 0x01 /* ICD */)
	    {
	        /* read ISTAD (ISAC) */
	        avm_pci_chip_read(sc, REG_isacsx_istad, &ista_d, 1);

		IHFC_MSG("ista_d=0x%02x\n", ista_d);

		if(ista_d & 0x80 /* RME */)
		{
		    /* read RBCL (ISAC) */
		    avm_pci_chip_read(sc, REG_isacsx_rbcld, &temp, 1);
		    sc->sc_fifo[d1r].Z_chip = temp;

		    /* read RSTA (ISAC) */
		    avm_pci_chip_read(sc, REG_isacsx_rstad, &temp, 1);
		    sc->sc_fifo[d1r].F_chip = temp;
		}

		/* RME or RPF - D channel receive */
		if(ista_d & 0xC0)
		{
		    QFIFO(sc,hi,d1r);
		    sc->sc_fifo[d1r].i_ista |=
		      ((ista_d & 0x80) ? I_ISTA_RME : I_ISTA_RPF);
		}

		/* XPR - D channel transmit */
		if(ista_d & 0x10)
		{
		    QFIFO(sc,hi,d1t);
		    sc->sc_fifo[d1t].i_ista |= I_ISTA_XPR;
		}

		/* RFO - D channel receive */
		if(ista_d & 0x20)
		{
		    QFIFO(sc,hi,d1r);
		    sc->sc_fifo[d1r].i_ista |= I_ISTA_ERR;
		}

		/* XDU or XCOL/XMR - D channel transmit */
		if(ista_d & (0x04 /* XDU */ | 0x08 /* XMR */))
		{
		    QFIFO(sc,hi,d1t);
		    sc->sc_fifo[d1t].i_ista |= I_ISTA_ERR;
		}
	    }

	    if(ista & 0x10 /* CIC */)
	    {
		ihfc_fsm_update(sc,&sc->sc_fifo[0],0);
	    }
	}

	/* ASL_IRQ_HSCX (always check B-channel status) */

	avm_pci_b_status_read(sc, &sc->sc_fifo[b1t & b1r], REG_BCHAN_1);

	avm_pci_b_status_read(sc, &sc->sc_fifo[b2t & b2r], REG_BCHAN_2);

	if(status & ASL_IRQ_TIMER)
	{
	    IHFC_MSG("timer interrupt\n");
	}
	return;
}

static void
avm_pci_chip_unselect CHIP_UNSELECT_T(sc)
{
	u_int8_t temp;

  	temp = 0xFF;

	/* write MASK and MASKD (ISAC-SX) */
	avm_pci_chip_write(sc, REG_isacsx_mask, &temp, 1);
	avm_pci_chip_write(sc, REG_isacsx_maskd, &temp, 1);

	/*
	 * write CMDR (ISAC/HSCX)
	 * NOTE: A 5-8us delay must be after the
	 * CMDR write to prevent too early
	 * ISTA interrupts, like XPR.
	 */

	temp = (sc->sc_fifo[d1r].i_cmdr |
		sc->sc_fifo[d1t].i_cmdr);

	/* write CMDR (ISAC-SX) */
	if(temp)
	{
		avm_pci_chip_write(sc, REG_isacsx_cmdrd, &temp, 1);
		sc->sc_fifo[d1r].i_cmdr = 0;
		sc->sc_fifo[d1t].i_cmdr = 0;

		DELAY(12);
	}

	/* NOTE: the interrupt write MASK is inverted */

	temp = 0xFF ^ (0x01 /* ICD */ | 0x10 /* CIC */);

	/* write MASK (ISAC-SX) */
	avm_pci_chip_write(sc, REG_isacsx_mask, &temp, 1);

	temp = 0xFF ^ (0xFC); /* enable all interrupts */

	/* write MASKD (ISAC-SX) */
	avm_pci_chip_write(sc, REG_isacsx_maskd, &temp, 1);

	return;
}

static void
avm_pci_fifo_reset(ihfc_sc_t *sc, ihfc_fifo_t *f, u_int8_t offset)
{
	IPAC_BUS_VAR(sc);

      	/* first step, reset the FIFO, 
	 * which will clear the selected protocol
	 */

	bus_space_write_1(t, h, offset + HSCX_STAT, 
			  (HSCX_CMD_XRS|HSCX_CMD_RRS));

	DELAY(150); /* wait at least 125 us */

	/* second step, select extended transparent mode */

	bus_space_write_1(t, h, offset + HSCX_PROT, HSCX_MODE_TRANS);

	DELAY(150); /* wait at least 125 us */

	/* update FIFO state */

	(f+receive)->i_ista = 0;
	(f+transmit)->i_ista = (I_ISTA_XPR|I_ISTA_ERR);

	return;
}

static void
avm_pci_chip_reset CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp;

 	bus_space_write_1(t, h, REG_STAT0_OFFSET, 0);

	DELAY(4000); /* 4 ms */

	bus_space_write_1(t, h, REG_STAT0_OFFSET, ASL_RESET_ALL);

	DELAY(4000); /* 4 ms */

	bus_space_write_1(t, h, REG_STAT0_OFFSET, 0);

	DELAY(4000); /* 4 ms */

 	bus_space_write_1(t, h, REG_STAT0_OFFSET, ASL_TIMERRESET);

	DELAY(4000); /* 4 ms */

	bus_space_write_1(t, h, REG_STAT0_OFFSET, ASL_ENABLE_INT);

	DELAY(4000); /* 4 ms */

 	avm_pci_fifo_reset(sc, &(sc->sc_fifo[b1t & b1r]), REG_BCHAN_1);
	avm_pci_fifo_reset(sc, &(sc->sc_fifo[b2t & b2r]), REG_BCHAN_2);

	/* TR_CONF0: Transceiver Configuration Register 0:
	 *	DIS_TR - transceiver enabled
	 *	EN_ICV - normal operation
	 *	EXLP - no external loop
	 *	LDD - automatic clock generation
	 */
	temp = 0;
	avm_pci_chip_write(sc, REG_isacsx_wtr_conf0, &temp, 1);

	/* TR_CONF2: Transceiver Configuration Register 1:
	 *	DIS_TX - transmitter enabled
	 *	PDS - phase deviation 2 S-bits
	 *	RLP - remote line loop open
	 */
	temp = 0;
	avm_pci_chip_write(sc, REG_isacsx_wtr_conf2, &temp, 1);

	/* MODED: Mode Register:
	 *	MDSx - transparent mode 0
	 *	TMD  - timer mode = external
	 *	RAC  - Receiver enabled
	 *	DIMx - digital i/f mode
	 */
	temp = 
	  0x80 /* MDS2 */|
	  0x40 /* MDS1 */|
	  0x08 /* RAC  */|
	  0x01 /* DIM0 */;
	avm_pci_chip_write(sc, REG_isacsx_wmoded, &temp, 1);

	/* TR_MODE:
	 *     Select TE-mode
	 */
	temp = 0;
	avm_pci_chip_write(sc, REG_isacsx_tr_mode, &temp, 1);

	return;
}

static ihfc_fifo_program_t *
avm_pci_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if((FIFO_NO(f) == d1t) ||
	   (FIFO_NO(f) == d1r))
	{
		if(PROT_IS_HDLC(&(f->prot_curr)))
		{
		  program = (FIFO_DIR(f) == transmit) ?
		    &i4b_ipac_tx_program :
		    &i4b_ipac_rx_program;
		}
	}
	else
	{
		if(PROT_IS_TRANSPARENT(&(f->prot_curr)))
		{
		    program = &i4b_unknown_program;
		}
	}
	return program;
}

register_list_t
avm_pci_register_list [] =
{
	{ 0, 0 }
};

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(avm_pci_dbase_root)
{
  I4B_DBASE_ADD(desc               , "AVM Fritz!Card version 2 PCI");

  I4B_DBASE_ADD(c_chip_reset       , &avm_pci_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &avm_pci_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &avm_pci_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &avm_pci_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &avm_pci_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &avm_pci_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &avm_pci_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &avm_pci_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &avm_pci_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &avm_pci_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &avm_pci_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &avm_pci_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &avm_pci_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(i4b_option_mask    , (I4B_OPTION_POLLED_MODE|
				      I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value   , 0);

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0     , 1); /* enable */

  I4B_DBASE_ADD(mem_rid[0]         , PCIR_BAR(0));
  I4B_DBASE_ADD(io_rid[0]          , PCIR_BAR(1));
}

I4B_PCI_DRIVER(.vid = 0x0e001244);

/* cleanup */

#undef avm_pci_chip_status_check
#undef avm_pci_fsm_table
#undef REG_STAT0_OFFSET
#undef REG_ISACSX_INDEX
#undef REG_ISACSX_DATA
#undef REG_BCHAN_1
#undef REG_BCHAN_2
#undef HSCX_FIFO
#undef HSCX_STAT
#undef HSCX_LEN
#undef HSCX_PROT
#undef ASL_IRQ_ISAC
#undef ASL_IRQ_HSCX
#undef ASL_IRQ_TIMER
#undef ASL_IRQ_PENDING
#undef ASL_RESET_ALL
#undef ASL_TIMERDISABLE
#undef ASL_TIMERRESET
#undef ASL_ENABLE_INT
#undef HSCX_MODE_ITF_FLG
#undef HSCX_MODE_TRANS
#undef HSCX_STAT_RME
#undef HSCX_STAT_RDO
#undef HSCX_STAT_CRCVFRRAB
#undef HSCX_STAT_CRCVFR
#undef HSCX_STAT_RML_MASK
#undef HSCX_INT_XPR
#undef HSCX_INT_XDU
#undef HSCX_INT_RPR
#undef HSCX_INT_MASK
#undef HSCX_CMD_XRS
#undef HSCX_CMD_XME
#undef HSCX_CMD_RRS
#undef IPAC_BUS_VAR

#endif /* _I4B_AVM_PCI_H_ */
