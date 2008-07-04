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
 *	I4B Siemens PSB 3186 ISDN Chipset Driver
 *	----------------------------------------
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_PSB3186_H_
#define _I4B_PSB3186_H_

#include <i4b/layer1/ihfc3/i4b_isac.h>

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

/* ``reg'' value definition: [0x00 - 0x6f] (PSB3186) */

#define IPAC_READ_1(reg,var)			\
	/* IPAC read */				\
	bus_space_write_1(t,h,0,(reg));		\
	var = bus_space_read_1(t,h,1)

#define IPAC_WRITE_1(reg,var)			\
	/* IPAC write */			\
	bus_space_write_1(t,h,0,(reg));		\
	bus_space_write_1(t,h,1,(var))

#define IPAC_READ_MULTI_1(reg,ptr,len)			\
	/* IPAC read */					\
	bus_space_write_1(t,h,0,(reg));			\
	bus_space_read_multi_1(t,h,1,(ptr),(len))

#define IPAC_WRITE_MULTI_1(reg,ptr,len)			\
	/* IPAC write */				\
	bus_space_write_1(t,h,0,(reg));			\
	bus_space_write_multi_1(t,h,1,(ptr),(len))

static void
psb3186_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
  IPAC_BUS_VAR(sc);

  /* read ISAC, HSCX A or HSCX B */
  IPAC_READ_MULTI_1(reg,ptr,len);
  return;
}

static void
psb3186_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
  psb3186_chip_read(sc,(f->fm.h.Zdata),ptr,len);
  return;
}

static void
psb3186_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
  IPAC_BUS_VAR(sc);

  /* write ISAC, HSCX A or HSCX B */
  IPAC_WRITE_MULTI_1(reg,ptr,len);
  return;
}

static void
psb3186_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
  psb3186_chip_write(sc,(f->fm.h.Zdata),ptr,len);
  return;
}

static ihfc_fifo_program_t *
psb3186_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
  ihfc_fifo_program_t *program = NULL;

  /* NOTE: currently only support for D-channel */

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
  return program;
}

static void
psb3186_chip_unselect CHIP_UNSELECT_T(sc)
{
  IPAC_BUS_VAR(sc);
  u_int8_t temp;

  /*
   * write CMDR (ISAC/HSCX)
   * NOTE: A 5-8us delay must be after the
   * CMDR write to prevent too early
   * ISTA interrupts, like XPR.
   */

  temp = (sc->sc_fifo[d1r].i_cmdr | sc->sc_fifo[d1t].i_cmdr);

  /* write CMDR (ISAC) */
  if(temp)
  {
	IPAC_WRITE_1(REG_psb3186_cmdr, temp);

	sc->sc_fifo[d1r].i_cmdr = 0;
	sc->sc_fifo[d1t].i_cmdr = 0;
  }

  /* if interrupts are still active,
   * it is necessary to toggle the
   * interrupt output by enabling and
   * disabling all interrupts (if the
   * interrupt is edge-triggered):
   */

  /* write MASK (ISAC) */
  IPAC_WRITE_1(REG_psb3186_mask, 0xff);

  /* delay a little after CMDR write */

  DELAY(12);

  /* write MASK (ISAC) */
  IPAC_WRITE_1(REG_psb3186_mask,
                 sc->sc_config.i_mask);
  return;
}

static void
psb3186_chip_reset CHIP_RESET_T(sc,error)
{
  IPAC_BUS_VAR(sc);

  IPAC_WRITE_1(REG_psb3186_sres, 0xff);

  DELAY(500);

  IPAC_WRITE_1(REG_psb3186_sres, 0x00);

  DELAY(500);

  return;
}

#if 0
static void
psb3186_t125_sync FIFO_SYNC_T(sc)
{
  IPAC_BUS_VAR(sc);

  /* this code is untested!
   * please verify that no
   * STI is generated if
   * ASTI is written within
   * a 125us delay!
   */

  /* acknowledge STI */
  IPAC_WRITE_1(REG_psb3186_sti, 1);

  SC_T125_WAIT_SET(sc);

  return;
}
#endif

static void
psb3186_chip_status_read CHIP_STATUS_READ_T(sc) 
{
  IPAC_BUS_VAR(sc); 
   
  /*
   * Generic ISAC/HSCX code:             
   */             
  register u_int8_t tmp, p_tmp;

  /* read ISTA (reg=0x60, PSB) */
  IPAC_READ_1(REG_psb3186_ista, p_tmp);

  /* check statemachine first (CIC) */
  if(p_tmp & 0x10)
  {
    /* CISQ */
    if(/*tmp & 0x04 assume it is CIR0 */ 1)
    {
      ihfc_fsm_update(sc,&sc->sc_fifo[0],0);
    }
  }

  /* check ICD */
  if(p_tmp & 0x01)
  {
    /* read ISTA (reg=0x20, ISAC) */
    IPAC_READ_1(REG_psb3186_istad, tmp);

    /* RME or RPF - D channel receive */
    if(tmp & 0xc0) {
      QFIFO(sc,hi,d1r);
      sc->sc_fifo[d1r].i_ista |=
	(tmp & 0x80) ?
	I_ISTA_RME :
	I_ISTA_RPF ;
    }

    /* RFO - D channel receive */
    if(tmp & 0x20)
    {
      QFIFO(sc,hi,d1r);
      sc->sc_fifo[d1r].i_ista |= I_ISTA_ERR;
    }

    /* XPR - D channel transmit */
    if(tmp & 0x10)
    {
      QFIFO(sc,hi,d1t);
      sc->sc_fifo[d1t].i_ista |= I_ISTA_XPR;
    }

    /* XDU or XCOL - D channel transmit */
    if(tmp & (0x04 /* XDU */ | 0x08 /* XCOL */))
    {
      QFIFO(sc,hi,d1t);
      sc->sc_fifo[d1t].i_ista |= I_ISTA_ERR;
    }
	  
    /* this test must be last, hence
     * it will change tmp!
     */
    if(tmp & 0x80 /* RME */)
    {
      /* read RBCL (reg=0x25, ISAC) */
      IPAC_READ_1(REG_psb3186_rbcl, tmp);
      sc->sc_fifo[d1r].Z_chip =     tmp;
                        
      /* read RSTA (reg=0x27, ISAC) */
      IPAC_READ_1(REG_psb3186_rsta, tmp);
      sc->sc_fifo[d1r].F_chip =     tmp;
    }

    /* clear all interrupts */
    tmp = 0;
  }
       
  /* check AUX (assuming TIN2 is disabled) */
  if(p_tmp & 0x08)
  {
    /* TIN */
    /* if(tmp & 0x08)  assume it is TIN2 { */
    QFIFO(sc,lo,d1t);
    QFIFO(sc,lo,b1t);
    QFIFO(sc,lo,b2t);
    /* } */
  }

  /* check for SIN (1500us timeout) */
  /* check STI */
  if(p_tmp & 0x20)
  {
    if(0 /* currently done elsewhere */) {
      /* acknowledge STI */
      IPAC_WRITE_1(REG_psb3186_sti, 1);
    }

    SC_T125_WAIT_CLEAR(sc);
  }

  return;
}

static void
psb3186_chip_status_check CHIP_STATUS_CHECK_T(sc)
{
  /* chip_status_read checks status */
  return;
}

static void    
psb3186_fsm_read FSM_READ_T(sc,f,ptr)   
{
  IPAC_BUS_VAR(sc);
  register u_int8_t tmp;

  /* CIRQ read (reg=0x2e, PSB3186) */
  IPAC_READ_1(REG_psb3186_cir0, tmp);

  *ptr = (tmp >> 4);

  return;
}

static void    
psb3186_fsm_write FSM_WRITE_T(sc,f,ptr) 
{
  IPAC_BUS_VAR(sc);

  /* CIRQ write (reg=0x2e, PSB3186) */
  IPAC_WRITE_1(REG_psb3186_cir0,
	       (((*ptr) << 2) | sc->sc_config.b_cir0));
  return;
}

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(psb1_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &psb3186_chip_reset);
  I4B_DBASE_ADD(c_chip_read        , &psb3186_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &psb3186_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &psb3186_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &psb3186_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &psb3186_chip_status_check );
  I4B_DBASE_ADD(c_fsm_read         , &psb3186_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &psb3186_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &psb3186_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &psb3186_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &psb3186_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_psb3186_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 *2));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 0); /* disable */

  I4B_DBASE_ADD(i4b_option_mask    , (I4B_OPTION_POLLED_MODE|
				      I4B_OPTION_DLOWPRI));
  I4B_DBASE_ADD(i4b_option_value   , 0);

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (psb3186_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (psb3186_fifo_map[0]));

 
  I4B_DBASE_ADD(desc               , "PSB 3186 ISDN card");

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
}

I4B_PNP_DRIVER(/* */
	       .vid = -1/* PnP id here */);

#include <i4b/layer1/ihfc3/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(psb1_dbase_root);

  /* I4B_DBASE_ADD(io_rid[0]  , PCIR_MAPS+0x00 must specify here ); */
}

I4B_PCI_DRIVER(/* */
	       .vid = -1/* PCI id here */);

#undef IPAC_BUS_VAR
#undef IPAC_READ_1
#undef IPAC_WRITE_1
#undef IPAC_READ_MULTI_1
#undef IPAC_WRITE_MULTI_1
#endif /* _I4B_PSB3186_H_ */
