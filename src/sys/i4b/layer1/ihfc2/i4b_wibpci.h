/*-
 * Copyright (c) 1999, 2000 Dave Boyce. All rights reserved.
 *
 * Copyright (c) 2000, 2001 Hellmuth Michaelis. All rights reserved.
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
 *	W6692x PCI support
 *	------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_WIBPCI_H_
#define _I4B_WIBPCI_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */

#define wibpci_fsm_table isac_fsm_table


/* BUG NOTE: The W6629x appears to have a bug.    When ``XME+XTF'' is
 * issued two times in a row with frame length == zero,    no further
 * ``XPR'' interrupts are generated.     Also the chip starts sending
 * HDLC data without stop.
 *
 * BUG NOTE:     The W6629x will sometimes generate ``XPR'' too early
 * when sending large frames over D - channel.   Fortunately this can
 * be detected by polling the XDOW bit before writing DCMDR.  If XDOW
 * is set, XRES will be issued and the frame will be tried repeated.
 *
 * NOTE: The W6692x does not have any RSTA added to B-channel receive
 * fifo in HDLC mode, so ``remove_stat == 0'' !
 *
 * NOTE: writing to timr2 register will set the ``count value'' to the
 * selected value and reset timer2. No interrupt is generated until
 * the ``count value'' has reached zero.
 *
 * NOTE: on-chip timers can not be used when
 * ``sc->sc_statemachine.state.active == 0'' (ISDN line deactivated)
 *
 */

#define WIBPCI_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.mem_hdl[0];

#define WIBPCI_READ_1(reg,var)			\
	var = bus_space_read_1(t,h,(reg))

#define WIBPCI_WRITE_1(reg,var)			\
	bus_space_write_1(t,h,(reg),(var))

#define WIBPCI_READ_MULTI_1(reg,ptr,len)		\
	bus_space_read_multi_1(t,h,(reg),(ptr),(len))

#define WIBPCI_WRITE_MULTI_1(reg,ptr,len)		\
	bus_space_write_multi_1(t,h,(reg),(ptr),(len))


I4B_FIFO_MAP_DECLARE(wibpci_fifo_map[]) =
{
  [d1t].i=
  {
    .Zdata        = 0x04,
    .block_size   = 0x40, /* = 64 bytes */
  },
  [d1r].i=
  {
    .Zdata        = 0x00,
    .block_size   = 0x40, /* = 64 bytes */
  },
  [b1t].i=
  {
    .Zdata        = 0x84,
    .block_size   = 0x40, /* = 64 bytes */
  },
  [b1r].i=
  {
    .Zdata        = 0x80,
    .block_size   = 0x40, /* = 64 bytes */
  },
  [b2t].i=
  {
    .Zdata        = 0xc4,
    .block_size   = 0x40, /* = 64 bytes */
  },
  [b2r].i=
  {
    .Zdata        = 0xc0,
    .block_size   = 0x40, /* = 64 bytes */
  },
};

static void
wibpci_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	WIBPCI_BUS_VAR(sc);
#if 1
	/* XXX bug in /sys/i386/include/bus.h XXX */
	if(len == 0)
	{
	    return;
	}
#endif
	/* read WIBPCI */
	WIBPCI_READ_MULTI_1(reg,ptr,len);
	return;
}

static void
wibpci_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	wibpci_chip_read(sc,(f->fm.h.Zdata),ptr,len);
	return;
}

static void
wibpci_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	WIBPCI_BUS_VAR(sc);
#if 1
	/* XXX bug in /sys/i386/include/bus.h XXX */
	if(len == 0)
	{
	    return;
	}
#endif
	/* write WIBPCI */
	WIBPCI_WRITE_MULTI_1(reg,ptr,len);
	return;
}

static void
wibpci_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	if(len != 0)
	{
		/* set write in progress bit */
		f->i_ista |= I_ISTA_WIP;

		wibpci_chip_write (sc,(f->fm.h.Zdata),ptr,len);
	}
	return;
}

static void
wibpci_chip_reset CHIP_RESET_T(sc,error)
{
	WIBPCI_BUS_VAR(sc);

	/* write CTL(reg=0x54, WIBPCI) */
	WIBPCI_WRITE_1(0x54, sc->sc_config.w_ctl_pci ^ 0x20);
	DELAY(4000);

	/* write CTL(reg=0x54, WIBPCI) */
	WIBPCI_WRITE_1(0x54, sc->sc_config.w_ctl_pci);
	DELAY(4000);
	return;
}

static void
wibpci_fsm_read FSM_READ_T(sc,f,ptr)
{
	register u_int8_t tmp;
	WIBPCI_BUS_VAR(sc);

	/* read CIR(reg=0x58, WIBPCI) */
	WIBPCI_READ_1(0x58, tmp);

	*ptr = (tmp & 0x0f);
	return;
}

static void
wibpci_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	WIBPCI_BUS_VAR(sc);

	u_int8_t tmp   = (*ptr) | sc->sc_config.i_cirq;
		 tmp >>= 2;

	/* write CIX(reg=0x5c, WIBPCI) */
	WIBPCI_WRITE_1(0x5c, tmp);
	return;
}

static void
wibpci_chip_status_read CHIP_STATUS_READ_T(sc) 
{
	register u_int8_t tmp, w_tmp;
	WIBPCI_BUS_VAR(sc);

	/* read ISTA (reg=0x14, WIBPCI) */
	WIBPCI_READ_1(0x14,     w_tmp);
	sc->sc_config.i_ista |= w_tmp;

	/* check D_RME; */
	if(w_tmp & 0x40 /* RME */)
	{
		/* read RBCL (reg=0x48, WIBPCI) */
		WIBPCI_READ_1(0x48,       tmp);
		sc->sc_fifo[d1r].Z_chip = tmp;

		/* read RSTA (reg=0x28, WIBPCI) */
		WIBPCI_READ_1(0x28,       tmp);
		sc->sc_fifo[d1r].F_chip = tmp ^ 0x20;
	}

	/* check D_EXIR */
	if(w_tmp & 0x04)
	{
		/* read D_EXIR (reg=0x1c, WIBPCI) */
		WIBPCI_READ_1(0x1c,     tmp);
		sc->sc_config.i_exir |= tmp;
	}

	/* check B1_EXIR */
	if(w_tmp & 0x02)
	{
		/* read B1_EXIR (reg=0x90, WIBPCI) */
		WIBPCI_READ_1(0x90,     tmp);
		sc->sc_config.h_exir |= tmp;

		/* check B1_RME */
		if(tmp & 0x20 /* RME */)
		{
			/* read RBCL (reg=0xAC, WIBPCI B1 - channel) */
			WIBPCI_READ_1(0xAC,       tmp);
			sc->sc_fifo[b1r].Z_chip = tmp;

			/* read STAR (reg=0x98, WIBPCI B1 - channel) */
			WIBPCI_READ_1(0x98,       tmp);
			sc->sc_fifo[b1r].F_chip = tmp ^ 0x20;
		}
	}

	/* check B2_EXIR */
	if(w_tmp & 0x01)
	{
		/* read B2_EXIR (reg=0xD0, WIBPCI) */
		WIBPCI_READ_1(0xD0,      tmp);
		sc->sc_config.h_exir |= (tmp << 8);

		/* check B2_RME */
		if(tmp & 0x20 /* RME */)
		{
			/* read RBCL (reg=0xEC, WIBPCI B2 - channel) */
			WIBPCI_READ_1(0xEC,       tmp);
			sc->sc_fifo[b2r].Z_chip = tmp;

			/* read STAR (reg=0xD8, WIBPCI B2 - channel) */
			WIBPCI_READ_1(0xD8,       tmp);
			sc->sc_fifo[b2r].F_chip = tmp ^ 0x20;
		}
	}
	return;
}

#define WIBPCI_CMDS (I_CMDR_XRES|I_CMDR_XME|I_CMDR_XTF|	\
		     I_CMDR_RRES|I_CMDR_RMC)
/*
 * D1 CMDR:
 *
 * (CMDR & 0x01) == 1 : XRST
 * (CMDR & 0x02) == 1 : XME
 * (CMDR & 0x04) == 1 : **   
 * (CMDR & 0x08) == 1 : XTF/XMS
 * (CMDR & 0x10) == 1 : start timer **
 * (CMDR & 0x20) == 1 : **
 * (CMDR & 0x40) == 1 : RRST
 * (CMDR & 0x80) == 1 : RMC/RACK
 *
 * B1/B2 CMDR:
 *
 * (CMDR & 0x01) == 1 : XRST
 * (CMDR & 0x02) == 1 : XME
 * (CMDR & 0x04) == 1 : XTF/XMS **
 * (CMDR & 0x08) == 1 : 128K mux mode **
 * (CMDR & 0x10) == 0 : transmitter enabled **
 * (CMDR & 0x20) == 1 : receiver enabled **
 * (CMDR & 0x40) == 1 : RRST
 * (CMDR & 0x80) == 1 : RMC/RACK
 *
 * **these bits are not the same as with IPAC chips
 */

static void                                                  
wibpci_chip_unselect CHIP_UNSELECT_T(sc)                              
{ 
	u_int8_t d1_cmdr = (sc->sc_fifo[d1t].i_cmdr|
			    sc->sc_fifo[d1r].i_cmdr) & WIBPCI_CMDS;

	u_int8_t b1_cmdr = (sc->sc_fifo[b1t].i_cmdr|
			    sc->sc_fifo[b1r].i_cmdr) & WIBPCI_CMDS;

	u_int8_t b2_cmdr = (sc->sc_fifo[b2t].i_cmdr|
			    sc->sc_fifo[b2r].i_cmdr) & WIBPCI_CMDS;

	WIBPCI_BUS_VAR(sc);

	/* check for D - channel commands */
	if(d1_cmdr)
	{ 
		/* check XME or XTF first */
		if(d1_cmdr & (I_CMDR_XME|I_CMDR_XTF))
		{
			u_int8_t tmp;

			/* read D_XSTAR (reg=0x24, WIBPCI) */
			WIBPCI_READ_1(0x24, tmp);

			/* check XDOW */
			if(tmp & 0x80)
			{
			  /* set XDU/XCOL bit */
			  sc->sc_fifo[d1t].i_ista |= I_ISTA_ERR;

			  /* add XRES */
			  d1_cmdr |= I_CMDR_XRES;
			}
		}

		/* check XME or XRES */
		if(d1_cmdr & (I_CMDR_XME|I_CMDR_XRES))
		{
		  /* check if write is not in progress */
		  if(!(sc->sc_fifo[d1t].i_ista & I_ISTA_WIP))
		  {
		       d1_cmdr |= I_CMDR_XRES; /* add XRES */
		  }

		  sc->sc_fifo[d1t].i_ista &= ~I_ISTA_WIP; /* clear I_ISTA_WIP */
		}

		/* write CMDR (reg=0x08,WIBPCI) */
		WIBPCI_WRITE_1(0x08,(d1_cmdr));
	}

	/* check for B1 - channel commands */
	if(b1_cmdr)
	{
		/* check XME or XRES */
		if(b1_cmdr & (I_CMDR_XME|I_CMDR_XRES))
		{
		  /* check if write is not in progress */
		  if(!(sc->sc_fifo[b1t].i_ista & I_ISTA_WIP))
		  {
		    b1_cmdr |= I_CMDR_XRES; /* add XRES */
		  }

		  /* clear I_ISTA_WIP */
		  sc->sc_fifo[b1t].i_ista &= ~I_ISTA_WIP;
		}

		/* move XTF command */
		if(b1_cmdr &  I_CMDR_XTF)
		{
		   b1_cmdr ^= I_CMDR_XTF|0x04;
		}

		/* receiver enable */
		b1_cmdr |= 0x20;

		/* write CMDR (reg=0x88,WIBPCI B1 - channel) */
		WIBPCI_WRITE_1(0x88,(b1_cmdr));
	} 

	/* check for B2 - channel commands */
	if(b2_cmdr)
	{
		/* check XME or XRES */
		if(b2_cmdr & (I_CMDR_XME|I_CMDR_XRES))
		{
		  /* check if write is not in progress */
		  if(!(sc->sc_fifo[b2t].i_ista & I_ISTA_WIP))
		  {
		    b2_cmdr |= I_CMDR_XRES; /* add XRES */
		  }

		  /* clear I_ISTA_WIP */
		  sc->sc_fifo[b2t].i_ista &= ~I_ISTA_WIP;
		}

   		/* move XTF command */
		if(b2_cmdr &  I_CMDR_XTF)
		{
		   b2_cmdr ^= I_CMDR_XTF|0x04;
		}

		/* receiver enable */
		b2_cmdr |= 0x20;

		/* write CMDR (reg=0xC8,WIBPCI B2 - channel) */
		WIBPCI_WRITE_1(0xC8,(b2_cmdr));
	}

	/* clear all CMDR */
	sc->sc_fifo[d1t].i_cmdr = 0; 
	sc->sc_fifo[d1r].i_cmdr = 0;
	sc->sc_fifo[b1t].i_cmdr = 0; 
	sc->sc_fifo[b1r].i_cmdr = 0;
	sc->sc_fifo[b2t].i_cmdr = 0; 
	sc->sc_fifo[b2r].i_cmdr = 0;
	return;
}

static void
wibpci_chip_status_check CHIP_STATUS_CHECK_T(sc)
{
	/* IHFC_MSG(" ... "); */

	/*
	 * D - channel interrupts (ISTA and D_EXIR)
	 */
	if(sc->sc_config.i_ista & 0xE4)
	{
 	  /* ISC - check statemachine change first */
	  if(sc->sc_config.i_exir & 0x04)
	  {
	      ihfc_fsm_update(sc,&sc->sc_fifo[0],0);
	  }

	  /* RME or RPF - D channel receive */
	  if(sc->sc_config.i_ista & 0xc0)
	  {      
		QFIFO(sc,hi,d1r);
                sc->sc_fifo[d1r].i_ista |=
                      (sc->sc_config.i_ista & 0x40) ?
                        I_ISTA_RME :
                        I_ISTA_RPF ;
	  }

	  /* RFO or RDO - D channel receive */
	  if(sc->sc_config.i_exir & 0x80)
	  {
                QFIFO(sc,hi,d1r);
                sc->sc_fifo[d1r].i_ista |= I_ISTA_ERR;
	  }

	  /* XPR - D channel transmit */
	  if(sc->sc_config.i_ista & 0x20)
	  {
		QFIFO(sc,hi,d1t);
		sc->sc_fifo[d1t].i_ista |= I_ISTA_XPR;
	  }

	  /* XDU or XCOL - D channel transmit */
	  if(sc->sc_config.i_exir & (0x40 /* XDU */ | 0x20 /* XCOL */))
	  {
                QFIFO(sc,hi,d1t);
                sc->sc_fifo[d1t].i_ista |= I_ISTA_ERR;
	  }

	  /* TIMER1 - 100ms timeout */
	  if(sc->sc_config.i_exir & 0x02)
	  {
                QFIFO(sc,lo,d1t);
                QFIFO(sc,lo,b1t);
                QFIFO(sc,lo,b2t);
	  }

	  /* clear all interrupts */
	  sc->sc_config.i_exir = 0;
	}

	/*
	 * B1 - channel interrupts
	 */
        if(sc->sc_config.i_ista & 0x02)
        {
          /* RME or RPF - B1 channel receive */
          if(sc->sc_config.h_exir & 0x0060)
	  {
                QFIFO(sc,hi,b1r);
                sc->sc_fifo[b1r].i_ista |=
                      (sc->sc_config.h_exir & 0x0020) ?
                        I_ISTA_RME :
                        I_ISTA_RPF ;
          }

	  /* RFO - B1 channel receive */
	  if(sc->sc_config.h_exir & 0x0010)
	  {
                QFIFO(sc,hi,b1r);
                sc->sc_fifo[b1r].i_ista |= I_ISTA_ERR;
	  }

          /* XPR - B1 channel transmit */
          if(sc->sc_config.h_exir & 0x0002)
	  {
                QFIFO(sc,hi,b1t);
                sc->sc_fifo[b1t].i_ista |= I_ISTA_XPR;
          }

	  /* XDU - B1 channel transmit */
	  if(sc->sc_config.h_exir & 0x0001)
	  {
                QFIFO(sc,hi,b1t);
                sc->sc_fifo[b1t].i_ista |= I_ISTA_ERR;
	  }
	}

	/*
	 * B2 - channel interrupts
	 */
        if(sc->sc_config.i_ista & 0x01)
	{
          /* RME or RPF - B2 channel receive */
          if(sc->sc_config.h_exir & 0x6000)
	  {
                QFIFO(sc,hi,b2r);
                sc->sc_fifo[b2r].i_ista |=
                      (sc->sc_config.h_exir & 0x2000) ?
                        I_ISTA_RME :
                        I_ISTA_RPF ;
          }

	  /* RFO - B2 channel receive */
	  if(sc->sc_config.h_exir & 0x1000)
	  {
		QFIFO(sc,hi,b2r);
                sc->sc_fifo[b2r].i_ista |= I_ISTA_ERR;
	  }

          /* XPR - B2 channel transmit */
          if(sc->sc_config.h_exir & 0x0200)
	  {
                QFIFO(sc,hi,b2t);
                sc->sc_fifo[b2t].i_ista |= I_ISTA_XPR;
          }

	  /* XDU - B2 channel transmit */
	  if(sc->sc_config.h_exir & 0x0100)
	  {
		QFIFO(sc,hi,b2t);
                sc->sc_fifo[b2t].i_ista |= I_ISTA_ERR;
	  }
        }

	/* clear all interrupts */
	sc->sc_config.h_exir = 0;

	/* clear all interrupts */
	sc->sc_config.i_ista = 0;
	return;
}

static ihfc_fifo_program_t *
wibpci_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
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
	  if(PROT_IS_HDLC(&(f->prot_curr)) ||
	     PROT_IS_TRANSPARENT(&(f->prot_curr)))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_ipac_tx_program :
			&i4b_ipac_rx_program;
	  }
	}
	return program;
}

/*-------------------------------------------------+
 | wibpci related structures                       |
 +-------------------------------------------------*/

register_list_t wibpci_register_list[] =
{
  { REG2OFF(w_b1_adm1)  , 0x9c },
  { REG2OFF(w_b2_adm1)  , 0xdc },
  { REG2OFF(w_b1_adm2)  , 0xa0 },
  { REG2OFF(w_b2_adm2)  , 0xe0 },
  { REG2OFF(w_d_mode)   , 0x0c },
  { REG2OFF(w_b1_mode)  , 0x8c },
  { REG2OFF(w_b2_mode)  , 0xcc },
  { REG2OFF(w_timr1)    , 0x10 },
  { REG2OFF(w_timr2)    , 0x4c },
  { REG2OFF(w_ctl_pci)  , 0x54 },
  { REG2OFF(w_pctl)     , 0x68 },
  { REG2OFF(w_gcr_pci)  , 0x7c },
  { REG2OFF(w_sam)      , 0x2c },
  { REG2OFF(w_tam)      , 0x38 },
  { REG2OFF(w_sqx)      , 0x64 },
  { REG2OFF(w_d_exim)   , 0x20 },
  { REG2OFF(w_b1_exim)  , 0x94 },
  { REG2OFF(w_b2_exim)  , 0xd4 },
  { REG2OFF(w_imask_pci), 0x18 },
  { 0, 0 }
};

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(wibpci_dbase_root)
{
  I4B_DBASE_ADD(desc, "Winbond 128K PCI ISDN adapter");

  I4B_DBASE_ADD(c_chip_reset         , &wibpci_chip_reset);
  I4B_DBASE_ADD(c_chip_read          , &wibpci_chip_read);
  I4B_DBASE_ADD(c_chip_write         , &wibpci_chip_write); 
  I4B_DBASE_ADD(c_chip_unselect      , &wibpci_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read   , &wibpci_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check  , &wibpci_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read           , &wibpci_fsm_read); 
  I4B_DBASE_ADD(c_fsm_write          , &wibpci_fsm_write);
  I4B_DBASE_ADD(d_fsm_table          , &wibpci_fsm_table);
  I4B_DBASE_ADD(c_fifo_get_program   , &wibpci_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read          , &wibpci_fifo_read);
  I4B_DBASE_ADD(c_fifo_write         , &wibpci_fifo_write); 

  I4B_DBASE_ADD(d_register_list      , &wibpci_register_list[0]);
  I4B_DBASE_ADD(d_channels           , 6);

  /* delay 7.14 milliseconds */
  I4B_DBASE_ADD(d_interrupt_delay    , hz / 140);

  I4B_DBASE_ADD(d_fifo_map[d1t], FM2OFF(wibpci_fifo_map[d1t]));
  I4B_DBASE_ADD(d_fifo_map[d1r], FM2OFF(wibpci_fifo_map[d1r]));
  I4B_DBASE_ADD(d_fifo_map[b1t], FM2OFF(wibpci_fifo_map[b1t]));
  I4B_DBASE_ADD(d_fifo_map[b1r], FM2OFF(wibpci_fifo_map[b1r]));
  I4B_DBASE_ADD(d_fifo_map[b2t], FM2OFF(wibpci_fifo_map[b2t]));
  I4B_DBASE_ADD(d_fifo_map[b2r], FM2OFF(wibpci_fifo_map[b2r]));

  I4B_DBASE_ADD(o_RES_IRQ_0          , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0       , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0       , 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask      , (I4B_OPTION_POLLED_MODE|
					I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value     , 0);

  I4B_DBASE_ADD(mem_rid[0]           , PCIR_BAR(0));
  I4B_DBASE_ADD(io_rid[0]            , PCIR_BAR(1));
}

I4B_PCI_DRIVER(/* Generic Winbond W6692 ISDN PCI (0x66921050)
		* Subvendor: -1, Subdevice: -1
		*
		* Planet PCI ISDN Adapter (Model IA128P-STDV)
		* Subvendor: 0x144F, Subdevice: 0x1707
		*/
	       .vid= 0x66921050,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x17020675)
		* Subvendor: -1, Subdevice: -1
		*/
	       .vid= 0x17020675,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x17030675)
		* Subvendor: -1, Subdevice: -1
		*/
	       .vid= 0x17030675,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x17070675)
		* Subvendor: -1, Subdevice: -1
		*/
	       .vid= 0x17070675,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x105e10cf)
		* Subvendor: -1, Subdevice: -1
		*/
	       .vid= 0x105e10cf,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x06751043) */
	       .vid= 0x06751043,
	       .sub= 0x1707144F,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x06751043) */
	       .vid= 0x06751043,
	       .sub= 0x2000144F,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x06751043) */
	       .vid= 0x06751043,
	       .sub= 0x17021443,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x06751043) */
	       .vid= 0x06751043,
	       .sub= 0x17071443,
	 );

I4B_PCI_DRIVER(/* ASUSCOM P-IN100-ST-D (Winbond W6692, 0x06751043) */
	       .vid= 0x06751043,
	       .sub= 0x2000144F,
	 );

#undef wibpci_fsm_table
#undef WIBPCI_CMDS
#undef WIBPCI_BUS_VAR
#undef WIBPCI_READ_1
#undef WIBPCI_WRITE_1
#undef WIBPCI_READ_MULTI_1
#undef WIBPCI_WRITE_MULTI_1

#endif /* _I4B_WIBPCI_H_ */
