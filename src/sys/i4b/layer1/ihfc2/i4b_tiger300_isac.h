/*-
 * Copyright (c) 2000, 2001 Sergio Prallon. All rights reserved.
 *
 * Copyright (c) 2002-2004 Hans Petter Selasky. All rights reserved.
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
 *	TigerJet 300/320 driver module
 * 	------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_TIGER300_H_
#define _I4B_TIGER300_H_

#include <i4b/layer1/ihfc2/i4b_hfc.h>
#include <i4b/layer1/ihfc2/i4b_isac.h>

#ifndef __i386__
/* depend on compatible bus_space_xxxx from
 * the file below:
 */
#include <i4b/layer1/ihfc2/i4b_hfc1.h>
#endif

/* imports */

#define t_aux_data aux_data_0 /* default value: 0x00;
			       * latch enabled, leds disabled
			       */

/* AMD and ISAC */

#define tiger300_fifo_select default_fifo_select
#define tiger300_fifo_inc_fx default_fifo_inc_fx
#define tiger300_fifo_inc_fx_pre default_fifo_inc_fx_pre
#define tiger300_chip_status_check default_chip_status_check

/* AMD only */

#define tiger300_amd_fsm_table  hfc_fsm_table

#define CHIP_IS_AMD(sc)				\
	(sc->sc_default.d_register_list ==	\
	    &tiger300_amd_register_list[0])	\
     /**/

/*
 * Tiger320 BUG: according to the manual the read INT0 value should be
 * written back, but if one write back a bit that is not enabled in the
 * T_INT0_MASK, the chip corrupts!
 *
 * About the hardware:
 * ===================
 *
 * The Tiger300/320 ASIC does nothing more than transfer via DMA the
 * first 32 bits of every IOM-2 frame to the serial interface to the
 * ISAC. There are no framing, deframing and flow control facilities
 * like provided by the ISAC chip.    In transmit direction the last
 * ring-buffer is repeated,   and in receive direction incoming data
 * will overwrite existing data if the ring-buffer is full.      The
 * occurrence of over/underruns can  be  limited by specifying large
 * ring-buffers**:
 *
 *
 * The first 4 bytes of the of IOM-2* frame:
 * =========================================
 *
 *     +0      +1      +2     +3
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 * | B1-CH | B2-CH |  MON  |  C/I &| 0xff  |  0xff | 0xff  | . . . |
 * |       |       |       |  D-CH |       |       |       |       |
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 *
 *  *see IOM-2 Interface Reference Guide (AMD) for more information.
 * **a safety-timer could have been implemented to stop data output.
 *
 * B-channels : 1 byte need 1 IOM-2-frames
 * D-channels : 1 byte need 4 IOM-2-frames
 *
 * NOTE: Hardware HDLC is used to transmit D-channel data, hence
 * neither the ISAC- or AMD-chip support repetition of transparent
 * data, on collision.
 *
 * NOTE: Transparent D-channel-data reception and transmission, is
 * supported. Transparent D-channel-data transmission might not work
 * unless the IOM-2 line is pulled to low for AM79C3XX and high for
 * ISAC. Also the TIC-bus must be enabled.
 *
 * NOTE: Tiger300/320 interrupt-masks will only disable interrupt
 * output, and not interrupt generation.
 *
 * NOTE: The AM79C3XX HDLC-transmitter interrupts when there are
 * ``threshold'' free bytes in the FIFO and not when there are
 * ``threshold'' bytes left in the FIFO like [most?] other chips do!
 *
 * I/O map overview:
 * =================
 *
 * 0x00
 *  .
 *  .  Internal
 *  .  register
 *  .  address
 *  .   space
 *  .
 * 0xC0
 *  .    PIB ***
 *  .  address
 *  .   space
 * 0xFF
 *
 * *** A Peripheral Interface Bus, PIB, is mapped 1:4 at offset 0xC0
 *     from the Tiger300/320 I/O-base-address. This bus can be used to
 *     access external devices like the ISAC-S TE. The PIB supply
 *     4 addressing lines, 8 bi-directional AUX lines and two
 *     Motorola compatible control signals: ~(DS) and Rd/~(Wr)
 *
 *
 * Tiger320 w/ISAC-S TE - AUX port:
 * ================================
 * A0 - ISAC's A4
 * A1 - ISAC's A5
 * A2 - ALE
 * A3 - ~(Chip Select)
 * A4 - ~(Interrupt)
 * A5 - LED1
 * A6 - LED2
 * A7 - LED3
 *
 * NOTE: The ISAC-S TE microcontroller interface have three modes:
 *
 * 1) ``Motorola type''
 * 2) ``Siemens/Intel non-multiplexed''
 * 3) ``Siemens/Intel multiplexed''
 *
 * The selection is performed via pin ALE as follows:
 *
 * ALE tied to Vdd -> 1
 * ALE tied to Vss -> 2
 * Edge on ALE     -> 3
 *
 * The Tiger300/320 use Motorola interface mode, 1), so ALE must be low
 * during chip reset! According to the ISAC-S TE manual, any edge on
 * ALE will immediately select multiplexed mode, 3).
 *
 *
 * Tiger300 w/AMD79C32A - AUX port:
 * ================================
 * A0 - NC
 * A1 - NC
 * A2 - NC
 * A3 - NC
 * A4 - ~(Interrupt)
 * A5 - LED1
 * A6 - LED2
 * A7 - LED3
 *
 */

#define TIGER300_TX_SIZE 0x1000 /* bytes of IOM-2 frames */
#define TIGER300_RX_SIZE 0x3000 /* bytes of IOM-2 frames */
#define TIGER300_TX_END   (TIGER300_TX_START+TIGER300_TX_SIZE) /* exclusive */
#define TIGER300_RX_END   (TIGER300_RX_START+TIGER300_RX_SIZE) /* exclusive */

/* transmit buffer first, then
 * receive buffer:
 */
#define TIGER300_TX_START (0)
#define TIGER300_RX_START (TIGER300_TX_END)

#if (TIGER300_RX_SIZE+TIGER300_TX_SIZE) > 0x10000
#error "Please limit buffer, so that 16-bit offsets can be used!"
#endif
#if (TIGER300_TX_END) > 0x4000
#error "TIGER_MWBA too small!"
#endif
#if (TIGER300_RX_SIZE % (4*4)) != 0
#error "TIGER300_RX_SIZE is not a factor of (4*4)"
#endif
#if (TIGER300_TX_SIZE % (4*4)) != 0
#error "TIGER300_TX_SIZE is not a factor of (4*4)"
#endif

I4B_FIFO_MAP_DECLARE(tiger300_fifo_map[]) =
{
  [d1t].h={ .Zbase = 0x14, .Zdata = 3, .Zsize = TIGER300_TX_SIZE/(4*4), .Zend = TIGER300_TX_END/(4*4), .block_size = 0x20 /* ISAC */ },
  [d1r].h={ .Zbase = 0x24, .Zdata = 3, .Zsize = TIGER300_RX_SIZE/(4*4), .Zend = TIGER300_RX_END/(4*4), .block_size = 0x20 /* ISAC */ },

  [b1t].h={ .Zbase = 0x14, .Zdata = 0, .Zsize = TIGER300_TX_SIZE/(4)  , .Zend = TIGER300_TX_END/(4)   },
  [b1r].h={ .Zbase = 0x24, .Zdata = 0, .Zsize = TIGER300_RX_SIZE/(4)  , .Zend = TIGER300_RX_END/(4)   },

  [b2t].h={ .Zbase = 0x14, .Zdata = 1, .Zsize = TIGER300_TX_SIZE/(4)  , .Zend = TIGER300_TX_END/(4)   },
  [b2r].h={ .Zbase = 0x24, .Zdata = 1, .Zsize = TIGER300_RX_SIZE/(4)  , .Zend = TIGER300_RX_END/(4)   },
};

#define ITJC_PIB(reg)		(0xC0 + ((reg & 0x0F) << 2))

/* AMD driver */

#define AMD_BUS_VAR(sc)						\
	bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.mem_hdl[0];	\
/**/

#define AMD_READ_1(reg,var)					\
	if(reg & 0xC0)						\
	{							\
		/* AMD READ - indirect register */		\
		bus_space_write_1(t,h,(ITJC_PIB(0x00)),(reg));	\
		var = bus_space_read_1(t,h,(ITJC_PIB(0x01)));	\
	}							\
	else							\
	{							\
		/* ITJC READ */					\
		var = bus_space_read_1(t,h,(reg));		\
	}

#define AMD_WRITE_1(reg,var)					\
	if(reg & 0xC0)						\
	{							\
		/* AMD WRITE - indirect register */		\
		bus_space_write_1(t,h,(ITJC_PIB(0x00)),(reg));	\
		bus_space_write_1(t,h,(ITJC_PIB(0x01)),(var));	\
	}							\
	else							\
	{							\
		/* ITJC WRITE */				\
		bus_space_write_1(t,h,(reg),(var));		\
	}

static void
tiger300_amd_fsm_read FSM_READ_T(sc,f,ptr)
{
	AMD_BUS_VAR(sc);
	u_int8_t tmp;

	/* read LSR(indirect reg=0xA1) */
	AMD_READ_1(0xA1,tmp);

	/* convert and store state */
	*ptr = (tmp & 7) + 2;

	return;
}

static void
tiger300_amd_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	AMD_BUS_VAR(sc);

	/* check activate */
	if((*ptr & 0x60) == 0x60)
	{
	  sc->sc_config.a_lmr1 |= 0x10;
	}
	else /* deactivate or T3 timeout */
	{
	  sc->sc_config.a_lmr1 &=~0x10;
	}

	/* write LMR1(indirect reg=0xA3) */
	AMD_WRITE_1(0xA3, sc->sc_config.a_lmr1);

	return;
}

static void
tiger300_amd_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	AMD_BUS_VAR(sc);
#if 1
	/* XXX bug in /sys/i386/include/bus.h XXX */
	if(len == 0)
	{
	        return;
	}
#endif
	/* read data */
	if(reg & 0xC0)						
	{
	  /* AMD READ - indirect register */
	  bus_space_write_1(t,h,(ITJC_PIB(0x00)),(reg));
	  bus_space_read_multi_1(t,h,(ITJC_PIB(0x01)),(ptr),(len));
	}
	else
	{
	  /* ITJC READ */
	  bus_space_read_multi_1(t,h,(reg),(ptr),(len));
	}

	return;
}

static void
tiger300_amd_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	AMD_BUS_VAR(sc);
#if 1
	/* XXX bug in /sys/i386/include/bus.h XXX */
	if(len == 0)
	{
	    return;
	}
#endif
	/* write data */
	if(reg & 0xC0)
 	{
	  /* AMD WRITE - indirect register */
	  bus_space_write_1(t,h,(ITJC_PIB(0x00)),(reg));
	  bus_space_write_multi_1(t,h,(ITJC_PIB(0x01)),(ptr),(len));
	}
	else
	{
	  /* ITJC WRITE */
	  bus_space_write_multi_1(t,h,(reg),(ptr),(len));
	}

	return;
}

#define A_INIT (0x02) /* active data mode; A_INIT cannot be zero! */

static void                                                  
tiger300_amd_chip_unselect CHIP_UNSELECT_T(sc)                              
{ 
	AMD_BUS_VAR(sc);

	u_int8_t d1_cmdr = (sc->sc_fifo[d1t].i_cmdr|
			    sc->sc_fifo[d1r].i_cmdr);

	/* check for D - channel commands */
	if(d1_cmdr)
	{ 
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

		/* The AM79C3XX chip only
		 * supports FIFO reset
		 */

		/* check XRES */
		if(d1_cmdr & (I_CMDR_XRES))
		{
		  /* send a zero length frame */
		  bus_space_write_1(t,h,(ITJC_PIB(0x00)),0x85);
		  bus_space_write_1(t,h,(ITJC_PIB(0x01)),0x00);
		  bus_space_write_1(t,h,(ITJC_PIB(0x01)),0x00);

		  /* a_init |= 0x80; doesn't work properly */
		}

		/* check RRES */
		if(d1_cmdr & (I_CMDR_RRES))
		{
		  /* write INIT(indirect reg=0x21) */
		  bus_space_write_1(t,h,(ITJC_PIB(0x00)),0x21);
		  bus_space_write_1(t,h,(ITJC_PIB(0x01)),(A_INIT|0x40));
		  bus_space_write_1(t,h,(ITJC_PIB(0x01)),(A_INIT));
		}
	}

	/* clear all CMDR */
	sc->sc_fifo[d1t].i_cmdr = 0; 
	sc->sc_fifo[d1r].i_cmdr = 0;
	return;
}

/* ISAC driver */

#define IPAC_LATCH_REG(reg)				\
{  u_int8_t tmp = sc->sc_config.t_aux_data;		\
	    tmp |= ((reg) >> 4) & 3;			\
   bus_space_write_1(t,h,0x03,tmp); /* AUX WRITE */	\
}							\
/**/

#define IPAC_BUS_VAR AMD_BUS_VAR

#define IPAC_READ_1(reg,var)					\
	if((reg) & 0x80)					\
	{							\
		/* ISAC READ */					\
		IPAC_LATCH_REG(reg);				\
		var = bus_space_read_1(t,h,ITJC_PIB(reg));	\
	}							\
	else							\
	{							\
		/* ITJC READ */					\
		var = bus_space_read_1(t,h,(reg));		\
	}							\
/**/

#define IPAC_WRITE_1(reg,var)					\
	if((reg) & 0x80)					\
	{							\
		/* ISAC WRITE */				\
		IPAC_LATCH_REG(reg);				\
		bus_space_write_1(t,h,ITJC_PIB(reg),(var));	\
	}							\
	else							\
	{							\
		/* ITJC WRITE */				\
		bus_space_write_1(t,h,(reg),(var));		\
	}							\
/**/

#define IPAC_READ_MULTI_1(reg,ptr,len)				\
  if(len)							\
  {								\
	if((reg) & 0x80)					\
	{							\
		/* ISAC READ */					\
		IPAC_LATCH_REG(reg);				\
		bus_space_read_multi_1				\
			(t,h,ITJC_PIB(reg),(ptr),(len));	\
	}							\
	else							\
	{							\
		/* ITJC READ */					\
		bus_space_read_multi_1(t,h,(reg),(ptr),(len));	\
	}							\
  }								\
/**/

#define IPAC_WRITE_MULTI_1(reg,ptr,len)				\
  if(len)							\
  {								\
	if((reg) & 0x80)					\
	{							\
		/* ISAC WRITE */				\
		IPAC_LATCH_REG(reg);				\
		bus_space_write_multi_1				\
			(t,h,ITJC_PIB(reg),(ptr),(len));	\
	}							\
	else							\
	{							\
		/* ITJC WRITE */				\
		bus_space_write_multi_1(t,h,(reg),(ptr),(len));	\
	}							\
  }								\
/**/

static void
tiger300_isac_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
  IPAC_BUS_VAR(sc);

  /* read ISAC, HSCX A or HSCX B */
  IPAC_READ_MULTI_1(reg,ptr,len);
  return;
}

static void
tiger300_isac_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
  IPAC_BUS_VAR(sc);

  /* write ISAC, HSCX A or HSCX B */
  IPAC_WRITE_MULTI_1(reg,ptr,len);
  return;
}

static void
tiger300_isac_fsm_read FSM_READ_T(sc,f,ptr)
{
  IPAC_BUS_VAR(sc);
  register u_int8_t tmp;

  /* read CIRQ (ISAC) */
  IPAC_READ_1(REG_isac_cirq, tmp);

  *ptr = (tmp >> 2) & 0xf;

  return;
}

static void
tiger300_isac_fsm_write FSM_WRITE_T(sc,f,ptr)
{
  IPAC_BUS_VAR(sc);

  /* write CIRQ (ISAC) */
  IPAC_WRITE_1(REG_isac_cirq,
          (*ptr | sc->sc_config.i_cirq));
  return;
}

static void
tiger300_isac_chip_unselect CHIP_UNSELECT_T(sc)
{
  IPAC_BUS_VAR(sc);

  /*
   * write CMDR (ISAC/HSCX)
   * NOTE: A 5-8us delay must be after the
   * CMDR write to prevent too early
   * ISTA interrupts, like XPR.
   */

  /* write CMDR (ISAC) */
  if(sc->sc_fifo[d1r].i_cmdr |=
     sc->sc_fifo[d1t].i_cmdr)
  {
          IPAC_WRITE_1(REG_isac_cmdr,
                       sc->sc_fifo[d1r].i_cmdr);

	  /* delay a little after CMDR write */

	  DELAY(12);

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
  IPAC_WRITE_1(REG_isac_mask, 0xff);

  /* write MASK (ISAC) */
  IPAC_WRITE_1(REG_isac_mask,
                 sc->sc_config.i_mask);
  return;
}

static void
tiger300_isac_chip_status_read CHIP_STATUS_READ_T(sc)
{
  IPAC_BUS_VAR(sc);
  register u_int8_t tmp;

  /*
   * generic ISAC/HSCX code:
   */

  /* read ISTA (ISAC) */
  IPAC_READ_1(REG_isac_ista, tmp);
  sc->sc_config.i_ista |= tmp;

  if(tmp & 0x80 /* RME */)
  {
          /* read RBCL (ISAC) */
          IPAC_READ_1(REG_isac_rbcl, tmp);
          sc->sc_fifo[d1r].Z_chip = tmp;

          /* read RSTA (ISAC) */
          IPAC_READ_1(REG_isac_rsta, tmp);
          sc->sc_fifo[d1r].F_chip = tmp;
  }

  if(sc->sc_config.i_ista & 0x01)
  {
          /* read EXIR (ISAC) */
          IPAC_READ_1(REG_isac_exir, tmp);
          sc->sc_config.i_exir |= tmp;
  }

  /* check for SIN (1500us timeout) */
  if(sc->sc_config.i_ista & 0x02)
  {
          if(sc->sc_config.i_stcr & 0x0f) {
             sc->sc_config.i_stcr &= 0xf3;

             /* write STCR (ISAC) */
             IPAC_WRITE_1(REG_isac_stcr,
                          sc->sc_config.i_stcr);

             sc->sc_config.i_stcr &= 0xf0;
             SC_T125_WAIT_CLEAR(sc);
          }
  }
  return;
}

register_list_t
tiger300_isac_register_list[] =
{
  _ISAC_REGISTERS(__REG2LIST)

  { REG2OFF(t_aux_data)              , 0x03 },
  { REG2OFF(t_dma_oper)              , 0x01 },
  { REG2OFF(t_prct)                  , 0x00 },

  { 0, 0 }
};

register_list_t
tiger300_amd_register_list[] =
{
  { REG2OFF(a_lmr1)                  , 0xA3 },
  { REG2OFF(a_lmr2)                  , 0xA4 },
  { REG2OFF(a_lpr)                   , 0xA2 },

  { REG2OFF(t_aux_data)              , 0x03 },
  { REG2OFF(t_dma_oper)              , 0x01 },
  { REG2OFF(t_prct)                  , 0x00 },

  { 0, 0 }
};

#define T_AUX_CNTL (0xEF)  /* AUX4 is input, other AUXs are output */
#define T_INT0_MASK (0x03) /* tx_end and tx_int */
#define T_INT1_MASK (0x10) /* enable external IRQ */
#define T_PRCT (/*0x40|*/0x20) /* DMA edge trigger + 12CLK I/O-access */

#include <i4b/layer1/ihfc2/i4b_regdata.h>

static const struct regdata
tiger300_isac_amd_regdata[] =
{
	/*
	 * Tiger300/320 setup
	 */
	{ .reg = 0x00, .data = 0x00        }, /* disable reset  */
	{ .reg = 0x01, .data = 0x00        }, /* disable DMA    */
	{ .reg = 0x02, .data = T_AUX_CNTL  }, /* write AUX_CNTL */
	{ .reg = 0x03, .data = 0x08        }, /* write AUX_DATA
					       * (ISAC must be de-selected
					       *  during reset!)
					       */
	{ .reg = 0x04, .data = T_INT0_MASK }, /* write INT0_MASK */
	{ .reg = 0x05, .data = T_INT1_MASK }, /* write INT1_MASK */

	/*
	 * Tiger320 setup
	 */
	{ .reg = 0x29, .data = 0x00        }, /* write AUX_MASK            */
	{ .reg = 0x2A, .data = 0x00        }, /* write AUX_POLARITY        */
	{ .reg = 0x2B, .data = 0x00        }, /* write AUX_SELECT          */
	{ .reg = 0x2C, .data = 0x00        }, /* write FRAME_DELAY_COUNT   */
	{ .reg = 0x2D, .data = 0x00        }, /* write SERIAL_PORT_CONTROL */
	{ .reg = 0x2E, .data = 0x00        }, /* write SECOND_FRAME_COUNT  */
	{ .reg = 0x2F, .data = 0x00        }, /* write FRAME_SYNC_DELAY    */
 	{ .reg = 0x00, .data = 0x0F|T_PRCT }, /* enable  reset (after AUX setup) */
 	{ .reg = 0x00, .data = 0x00|T_PRCT }, /* disable reset  */

	/*
	 * If DMA-enable is not toggled,
	 * the Tiger300 will start sending
	 * zeros to the IOM-2 D-channel
	 * (after Tiger300 reset)!
	 */
	{ .reg = 0x01, .data = 0x01        }, /* enable  DMA    */
	{ .reg = 0x01, .data = 0x00        }, /* disable DMA    */

	/*
	 * AM79C32/30 setup
	 * (also see ``tiger300_amd_register_list'')
	 */
#define W1(_reg,_data)					\
	/* write a byte to an indirect register */	\
	{ .reg = ITJC_PIB(0x00), .data = (_reg)  },	\
	{ .reg = ITJC_PIB(0x01), .data = (_data) }	\
	/**/

	W1(0x41/*A_MCR1  */, 0x00 | 0x16), /* connect PP_1 to LIU_1 (duplex)    */
	W1(0x42/*A_MCR2  */, 0x00 | 0x27), /* connect PP_2 to LIU_2 (duplex)    */
	W1(0x43/*A_MCR3  */, 0x00 | 0x00), /* no connect (unused)               */
	W1(0x44/*A_MCR4  */, 0x00 | 0x00), /* normal bit order, disable int.    */
	W1(0x86/*A_DMR1  */, 0x00 | 0x01 /*XPR*/), /* enable int.               */
	W1(0x87/*A_DMR2  */, 0x00 | 0x04 /*XCOL*/|0x80/*XDU*/), /* enable int.  */
	W1(0x8E/*A_DMR3  */, 0x00 | 0x10 /*XME*/), /* enable int.               */
	W1(0x8F/*A_DMR4  */, 0x00 | 0x09), /* 16/10-byte RX/TX-threshold         */
	W1(0x92/*A_EFCR  */, 0x00 | 0x01), /* extended FIFO mode                */
#if 0
	W1(0xA6/*A_MF    */, 0x00 | 0x09), /* MultiFrame enabled                */
#else
	W1(0xA6/*A_MF    */, 0x00 | 0x00), /* MultiFrame disabled               */
#endif
	W1(0xA8/*A_MFQB  */, 0x00 | 0x0F), /* reset default                     */
	W1(0xC0/*A_PPCR1 */, 0x00 | 0x07), /* IOM-2 Master mode enable+activate */
	W1(0xC2/*A_PPIER */, 0x00 | 0x00), /* disable all PP/MF interrupts      */
	W1(0xC4/*A_CITDR0*/, 0x00 | 0x0F), /* C/I == 0xF                        */
	W1(0xC8/*A_PPCR2 */, 0x00 | 0x00), /* SCLK not inverted                 */

#if 0 /* hardware-HDLC only works when TIC-bus is disabled;
       * software-HDLC only works when TIC-bus is enabled and IOM-2 is pulled low.
       */
	W1(0xC9/*A_PPCR3 */, 0x00 | 0x0F), /* TIC-bus address = 7, TIC enable   */
#else
	W1(0xC9/*A_PPCR3 */, 0x00 | 0x00), /* TIC-bus address = 0, TIC disable  */
#endif
	W1(0x20/*A_INIT2 */, 0x00 | 0x00), /* reset default first time          */
	W1(0x20/*A_INIT2 */, 0x00 | 0x00), /* reset default second time         */
	W1(0x21/*A_INIT  */, A_INIT     ), /* NOTE: this entry must be last     */
#undef  W1

	{ .reg = 0xff      ,.data = 0xff}, /* END */
};

static void
tiger300_chip_reset CHIP_RESET_T(sc,error)
{
	struct sc_default *def = &sc->sc_default;
	const __typeof(tiger300_isac_amd_regdata[0]) *ptr = &tiger300_isac_amd_regdata[0];
	u_int32_t mwba_phys_start = sc->sc_resources.mwba_phys_start[0];
	u_int8_t tmp;
	IPAC_BUS_VAR(sc);

	/*
	 * setup MWBA by config writes,
	 * before other configuration is
	 * loaded:
	 */
	bus_space_write_4(t,h,0x08, mwba_phys_start + (TIGER300_TX_START)); /* transmit_start - ``including'' */
	bus_space_write_4(t,h,0x0C, mwba_phys_start + (TIGER300_TX_START + (TIGER300_TX_SIZE/2))); /* transmit_intr */
	bus_space_write_4(t,h,0x10, mwba_phys_start + (TIGER300_TX_END - 4)); /* transmit_end - ``including'' */
	                   /* 0x14 */
	bus_space_write_4(t,h,0x18, mwba_phys_start + (TIGER300_RX_START)); /* receive_start - ``including'' */
	bus_space_write_4(t,h,0x1C, mwba_phys_start + (TIGER300_RX_START + (TIGER300_RX_SIZE/2))); /* receive_intr */
	bus_space_write_4(t,h,0x20, mwba_phys_start + (TIGER300_RX_END - 4)); /* receive_end  - ``including'' */
	                   /* 0x24 */

	/*
	 * Enable busmaster so that
	 * chip can write to host
	 * memory (PCI only).
	 */
	pci_enable_busmaster(sc->sc_device);

	/*
	 * reset and reload configuration
	 */
	for(;
	    ptr->reg != 0xff;
	    ptr++)
	{
	  /* write register */
	  bus_space_write_1(t,h,ptr->reg, ptr->data);

	  /* delay after and during reset */
	  if(ptr->reg == 0x00)
	  {
	    DELAY(4000);
	  }
	}

	/*
	 * test chiptype
	 */

	tmp = bus_space_read_1(t,h,(ITJC_PIB(0x01)));

	IHFC_MSG("Reading DR: 0x%02x\n", tmp);

	if((tmp == A_INIT) || CHIP_IS_AMD(sc))
	{
	  /* AMD
	   *
	   * overrides for AMD chip
	   *
	   * update some functions, else try use
	   * what is compatible ...
	   */

	  I4B_DBASE_ADD(c_fsm_read          , &tiger300_amd_fsm_read);
	  I4B_DBASE_ADD(c_fsm_write         , &tiger300_amd_fsm_write);
	  I4B_DBASE_ADD(d_fsm_table         , &tiger300_amd_fsm_table);

	  I4B_DBASE_ADD(c_chip_read         , &tiger300_amd_chip_read);
	  I4B_DBASE_ADD(c_chip_write        , &tiger300_amd_chip_write);
	  I4B_DBASE_ADD(c_chip_unselect     , &tiger300_amd_chip_unselect);

	  I4B_DBASE_ADD(d_register_list     , &tiger300_amd_register_list[0]);

	  /* FIFO block sizes */
	  sc->sc_fifo[d1t].fm.i.block_size = 8; /* bytes */
	  sc->sc_fifo[d1r].fm.i.block_size = 16; /* bytes */

	  /* reset default (statemachine) */
	  sc->sc_config.a_lmr1  &=~0x10;
	}
	else
	{
	  if(tmp != 0x00)
	  {
	    IHFC_ERR("Unexpected register "
		     "value == 0x%02x(ignored)!\n", tmp);
	  }

	  /* ISAC
	   *
	   * IOM-2 Stop/go-bits must be disabled,
	   * because else the Tiger300 will block
	   * the ISAC D-channel-TX-FIFO:
	   */
	  sc->sc_config.i_mode &= ~0x07;
  	}
 	return;
}

static void
tiger300_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	IPAC_BUS_VAR(sc);
	u_int16_t Z_chip2;
	u_int16_t mwba_phys_start = sc->sc_resources.mwba_phys_start[0];

	/* disable interrupts */
	cli(sc);

	/* read the two first bytes of the current 
	 * read/write position, hence the total
	 * buffer size is less than 64K:
	 */
	(   Z_chip2) = bus_space_read_2(t,h,(f->fm.h.Zbase));
	(f->Z_chip)  = bus_space_read_2(t,h,(f->fm.h.Zbase));

	/* analyse/correct the counters */
	INC_COUNTER_TIME_CHECK(f->Z_chip,    Z_chip2);

	/* get offset relative to start of MWBA */
        (f->Z_chip) -= (mwba_phys_start);
  	(f->Z_chip) &= (0xffff);

	/* get offset in bytes */
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  (f->Z_chip) /= (4*4);
	}
	else
	{
	  (f->Z_chip) /= 4;
	}

	/* NOTE: it is only necessary to
	 * read F_drvr and Z_drvr once
	 * from the hardware:
	 */

	if(!(f->state &  ST_FZ_LOADED)) {
	     f->state |= ST_FZ_LOADED;

	     /* set Z_drvr (Zsize/2) ahead of
	      * Z_chip
	      */

	     if(((f->Z_drvr)  = ((f->Z_chip)+(f->fm.h.Zsize/2))) >= f->fm.h.Zend) {
	         (f->Z_drvr) -= (f->fm.h.Zsize);
	     }
	}

	/*
	 * In case of
	 * reset:
	 */

	(f->F_chip) = (f->F_drvr);

	/* enable interrupts */
	sti(sc);

	return;
}

static void
tiger300_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* TIGER - D - channel */
	  register u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
					(f->fm.h.Zdata)+((4*4)*(f->Z_drvr));
	  register u_int16_t fifo_len = (f->fm.h.Zend)-(f->Z_drvr);

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_drvr) += (len);

	  if(len >= fifo_len) 
	  {
	    /* Wrapped read 
	     * fifo.curr |---> fifo.end
	     *
	     * Compress data 4:1
	     */

	    len -= fifo_len;
	    while (fifo_len--)
	    {
	      *ptr++ =
		((fifo_ptr[0x0] & 3) << 0) |
		((fifo_ptr[0x4] & 3) << 2) |
		((fifo_ptr[0x8] & 3) << 4) |
		((fifo_ptr[0xC] & 3) << 6);

	      fifo_ptr += (4*4);
	    }

	    /* get ``min'' value */	    
	    (f->Z_drvr) -=       (f->fm.h.Zsize);
	    (fifo_ptr)  -= (4*4)*(f->fm.h.Zsize);
	  }

	  /* Continuous read
	   * fifo.curr |---> fifo.Z_drvr(after post increment)
	   *
	   * Compress data 4:1
	   */

	  while(len--)
	  {
	    *ptr++ = 
	      ((fifo_ptr[0x0] & 3) << 0) |
	      ((fifo_ptr[0x4] & 3) << 2) |
	      ((fifo_ptr[0x8] & 3) << 4) |
	      ((fifo_ptr[0xC] & 3) << 6);

	    fifo_ptr += (4*4);
	  }
	}
	else
	{
	  /* TIGER - B - channel */
	  register u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
					(f->fm.h.Zdata)+(4*(f->Z_drvr));
	  register u_int16_t fifo_len = (f->fm.h.Zend)-(f->Z_drvr);

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_drvr) += (len);

	  if(len >= fifo_len) 
	  {
	    /* Wrapped read 
	     * fifo.curr |---> fifo.end
	     */

	    len -= fifo_len;
	    while (fifo_len--)
	    {
	      *ptr++ = *fifo_ptr;
	                fifo_ptr += 4;
	    }

	    /* get ``min'' value */
	    (f->Z_drvr) -=   (f->fm.h.Zsize);
	    (fifo_ptr)  -= 4*(f->fm.h.Zsize);
	  }

	  /* Continuous read
	   * fifo.curr |---> fifo.Z_drvr(after post increment)
	   */

	  while(len--)
	  {
	    *ptr++ = *fifo_ptr;
		      fifo_ptr += 4;
	  }
	}

	return;
}

static void
tiger300_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
  if((FIFO_NO(f) == d1t) && PROT_IS_HDLC(f->prot))
  {
    IPAC_BUS_VAR(sc);

    if(CHIP_IS_AMD(sc))
    {
	/* AMD
	 *
	 * if the FIFO write is interrupted,
	 * extra threshold interrupts will be
	 * generated!
	 *
	 * disable interrupts
	 */
	cli(sc);

	if(!(f->i_ista & I_ISTA_WIP))
	{
	  /* write is in progress,
	   * also when frame length == 0
	   */
	  f->i_ista |= I_ISTA_WIP;

	  /* temporary workaround, hence the chip
	   * needs  to know how  many bytes there
	   * are in the transmit frame (excluding
	   * chip generated checksum)
	   */

	  /* write DTCR(indirect reg=0x85) */
	  bus_space_write_1(t,h,(ITJC_PIB(0x00)),0x85);

	  /* least significant byte first */
	  bus_space_write_1(t,h,(ITJC_PIB(0x01)), (f->buf_size & 0xff));

	  /* most significant byte last */
	  bus_space_write_1(t,h,(ITJC_PIB(0x01)), ((f->buf_size >> 8) & 0xff));
	}

	/* write DCTB(reg=0x04) */
	if(len)
	{
	    bus_space_write_multi_1(t,h,(ITJC_PIB(0x04)),(ptr),(len));
	}

	/* enable interrupts */
	sti(sc);
    }
    else
    {
      /* ISAC */

      /* cannot use generated fifo_write,
       * because .Zdata is not initialized
       */
      IPAC_WRITE_MULTI_1(REG_isac_data,ptr,len);
    }
  }
  else

  if(len)
  {
	f->last_byte = ptr[len-1];

	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* TIGER - D - channel */
	  register u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
					(f->fm.h.Zdata)+((4*4)*(f->Z_drvr));
	  register u_int16_t fifo_len = (f->fm.h.Zend)-(f->Z_drvr);

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_drvr) += (len);

	  if(len >= fifo_len) 
	  {
	    /* Wrapped write 
	     * fifo.curr |---> fifo.end
	     *
	     * Expand data 1:4
	     */

	    len -= fifo_len;
	    while (fifo_len--)
	    {
	      fifo_ptr[0x0]  =  (*ptr >> 0) | 0xfc;
	      fifo_ptr[0x4]  =  (*ptr >> 2) | 0xfc;
	      fifo_ptr[0x8]  =  (*ptr >> 4) | 0xfc;
	      fifo_ptr[0xC]  =  (*ptr >> 6) | 0xfc;
	      fifo_ptr += (4*4);  ptr++;
	    }

	    /* get ``min'' value */
	    (f->Z_drvr) -=       (f->fm.h.Zsize);
	    (fifo_ptr)  -= (4*4)*(f->fm.h.Zsize);
	  }

	  /* Continuous write
	   * fifo.curr |---> fifo.Z_drvr(after post increment)
	   *
	   * Expand data 1:4
	   */

	  while(len--)
	  {
	    fifo_ptr[0x0]  =  (*ptr >> 0) | 0xfc;
	    fifo_ptr[0x4]  =  (*ptr >> 2) | 0xfc;
	    fifo_ptr[0x8]  =  (*ptr >> 4) | 0xfc;
	    fifo_ptr[0xC]  =  (*ptr >> 6) | 0xfc;
	    fifo_ptr += (4*4); 	ptr++;
	  }
	}
	else
	{
	  /* TIGER - B - channel */
	  register u_int8_t *fifo_ptr = (sc->sc_resources.mwba_start[0])+
					(f->fm.h.Zdata)+(4*(f->Z_drvr));
	  register u_int16_t fifo_len = (f->fm.h.Zend)-(f->Z_drvr);

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_drvr) += (len);

	  if(len >= fifo_len) 
	  {
	    /* Wrapped write 
	     * fifo.curr |---> fifo.end
	     */

	    len -= fifo_len;
	    while (fifo_len--)
	    {
	          *fifo_ptr  = *ptr++;
		   fifo_ptr += 4;
	    }

	    /* get ``min'' value */
	    (f->Z_drvr) -=   (f->fm.h.Zsize);
	    (fifo_ptr)  -= 4*(f->fm.h.Zsize);
	  }

	  /* Continuous write
	   * fifo.curr |---> fifo.Z_drvr(after post increment)
	   */

	  while(len--)
	  {
	   *fifo_ptr = *ptr++;
	    fifo_ptr += 4;
	  }
	}
  }

  return;
}

static void
tiger300_fifo_write_filler FIFO_WRITE_FILLER_T(sc,f)
{
	if(FIFO_CMP(f,<,(b1t & b1r)))
	{
	  /* TIGER - D - channel 
	   * Fill buffer with 0xff [idle-] bytes
	   */
	  register u_int8_t *fifo_ptr  = (sc->sc_resources.mwba_start[0])+
					 (f->fm.h.Zdata)+((4*4)*(f->Z_drvr));
	  register u_int16_t fifo_len  = (f->fm.h.Zend)-(f->Z_drvr);
	  register u_int16_t len       = (f->Z_chip);

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_drvr) += (len);

	  if(len >= fifo_len)
	  {
	    /* Wrapped write 
	     * fifo.curr |---> fifo.end
	     */

	    len -= fifo_len;
	    while (fifo_len--)
	    {
	      fifo_ptr[0x0] = 0xff;
	      fifo_ptr[0x4] = 0xff;
	      fifo_ptr[0x8] = 0xff;
	      fifo_ptr[0xC] = 0xff;
	      fifo_ptr += (4*4);
	    }

	    /* get ``min'' value */
	    (f->Z_drvr) -=       (f->fm.h.Zsize);
	    (fifo_ptr)  -= (4*4)*(f->fm.h.Zsize);
	  }

	  /* Continuous write
	   * fifo.curr |---> fifo.Z_drvr(after post increment)
	   */

	  while(len--)
	  {
	      fifo_ptr[0x0] = 0xff;
	      fifo_ptr[0x4] = 0xff;
	      fifo_ptr[0x8] = 0xff;
	      fifo_ptr[0xC] = 0xff;
	      fifo_ptr += (4*4);
	  }
	}
	else
	{
	  /* TIGER - B - channel 
	   * Fill buffer with last byte, hence
	   * the hardware is not capable of repeating
	   * the last byte.
	   */
	  register u_int8_t  last_byte = f->last_byte;
	  register u_int8_t *fifo_ptr  = (sc->sc_resources.mwba_start[0])+
					 (f->fm.h.Zdata)+(4*(f->Z_drvr));
	  register u_int16_t fifo_len  = (f->fm.h.Zend)-(f->Z_drvr);
	  register u_int16_t len       = (f->Z_chip);

	  /* pre increment Z-counter (before `len` is changed) */
	  (f->Z_drvr) += (len);

	  if(len >= fifo_len)
	  {
	    /* Wrapped write 
	     * fifo.curr |---> fifo.end
	     */

	    len -= fifo_len;
	    while (fifo_len--)
	    {
	       *fifo_ptr  = last_byte;
	        fifo_ptr += 4;
	    }

	    /* get ``min'' value */
	    (f->Z_drvr) -=   (f->fm.h.Zsize);
	    (fifo_ptr)  -= 4*(f->fm.h.Zsize);
	  }

	  /* Continuous write
	   * fifo.curr |---> fifo.Z_drvr(after post increment)
	   */

	  while(len--)
	  {
	    *fifo_ptr = last_byte;
	     fifo_ptr += 4;
	  }
	}

#if 0
	/* nothing more can
	 * be written to the FIFO
	 */
	(f->Z_chip) = 0;
#endif

	return;
}

/* register format                                         interrupt-mask
 *
 * IR 0x01: D-channel Transmit buffer threshold            DMR1 0x01:
 * IR 0x02: D-channel Receive buffer threshold             DMR1 0x02:
 * IR 0x04: D-channel status
 *  DSR1 0x01: Valid Address (VA) or End of Address (EOA)  DMR3 0x01:
 *  DSR1 0x02: When a closing flag or error is received    DMR1 0x08:
 *  DSR1 0x40: When a closing flag is transmitted          DMR3 0x02:
 * IR 0x08: D-channel error
 *  DER 0x01: Current received packet has been aborted     DMR2 0x01:
 *  DER 0x02: Non-integer number of bytes received         DMR2 0x02:
 *  DER 0x04: Transmit Collision abort detected            DMR2 0x04:
 *  DER 0x08: Receive FCS error                            DMR2 0x08:
 *  DER 0x10: Receive Overflow error                       DMR2 0x10:
 *  DER 0x20: Receive Underflow error                      DMR2 0x20:
 *  DER 0x40: Receive Overrun error                        DMR2 0x40:
 *  DER 0x80: Transmit Underrun error                      DMR2 0x80:
 *  DSR2 0x04: Receive packet lost                         DMR3 0x40:
 * IR 0x10: -
 * IR 0x20: statemachine change
 * IR 0x40: D-channel status
 *  DSR2 0x01: Last byte of received packet                DMR3 0x04:
 *  DSR2 0x02: Receive byte available                      DMR3 0x08:
 *  DSR2 0x08: Last byte transmitted                       DMR3 0x10:
 *  DSR2 0x10: Transmit buffer available                   DMR3 0x20:
 *  DSR2 0x80: Start of second received packet             EFCR 0x02:
 * IR 0x80: -
 *
 * NOTE: only interrupt-output is masked. Registers are not masked!?
 */
static void
tiger300_chip_status_read CHIP_STATUS_READ_T(sc)
{
	IPAC_BUS_VAR(sc);
	u_int8_t tmp;

	/* unstable POWER may reset the chip!
	 */
	tmp = bus_space_read_1(t,h,0x00);
	if(tmp & 0xf)
	{
	  IHFC_ERR("External chip gone (resetting)!\n");

	  ihfc_reset(sc,NULL);
	}

	/* check for external interrupt
	 * and acknowledge by reading the
	 * external chip's status register(s)
	 */
 	tmp = bus_space_read_1(t, h, 7);

	if(!(tmp & 0x10))
	{
	  if(CHIP_IS_AMD(sc))
	  {
#define ir tmp
	    u_int8_t der,dsr1,dsr2;

	    /* read IR(reg=0x00) */
	    ir = bus_space_read_1(t,h,(ITJC_PIB(0x00)));

	    IHFC_MSG("IR=0x%02x\n", ir);

	    /* statemachine change (check first) */
	    if(ir & 0x20)
	    {
		sc->sc_config.s_int_s1 |= 0x40;
	    }

	    /*
	     * need to clear the sources of interrupts,
	     * else new interrupts will not be generated:
	     */

	    /* read DSR1(reg=0x02) */
	    dsr1 = bus_space_read_1(t,h,(ITJC_PIB(0x02)));

	    IHFC_MSG("DSR1=0x%02x\n", dsr1);

	    /* read DSR2(reg=0x07) */
	    dsr2 = bus_space_read_1(t,h,(ITJC_PIB(0x07)));

	    IHFC_MSG("DSR2=0x%02x\n", dsr2);

	    /* read DER(reg=0x03) */
	    der = bus_space_read_1(t,h,(ITJC_PIB(0x03)));

	    IHFC_MSG("DER=0x%02x\n", der);

	    /* check if write in progress */
	    if(sc->sc_fifo[d1t].i_ista & I_ISTA_WIP)
	    {
	      if(ir & 0x01)
	      {
		/* XPR - D channel transmit
		 * (can transmit 8 bytes to FIFO)
		 */
		QFIFO(sc,hi,d1t);
		sc->sc_fifo[d1t].i_ista |= I_ISTA_XPR;
	      }

	      if(der & (0x04|0x80))
	      {
		/* XDU or XCOL - D channel transmit */
                QFIFO(sc,hi,d1t);
                sc->sc_fifo[d1t].i_ista |= I_ISTA_ERR;
	      }
	    }
	    else
	    {
	      /* if(dsr1 & 0x40) */
	      if(dsr2 & 0x08)
	      {
		/* XPR - D channel transmit
		 * (last byte transmitted)
		 */
		QFIFO(sc,hi,d1t);
		sc->sc_fifo[d1t].i_ista |= I_ISTA_XPR;
	      }
	    }
#undef ir
	  }
	  else
	  {
	    /* call generated interrupt routine
	     * for ISAC
	     */
	    tiger300_isac_chip_status_read(sc);
	  }
	}

	/* check for Tiger300/320 interrupts
	 * and acknowledge all
	 */
	tmp = bus_space_read_1(t, h, 6);
	bus_space_write_1(t, h, 6, (tmp & T_INT0_MASK));

	if((tmp & T_INT0_MASK) || (sc->sc_default.o_POLLED_MODE))
	{
	  /* generate interrupt for
	   * B1-channel and B2-channel
	   * through the hfc_2bds0 driver
	   */
	  sc->sc_config.s_int_s1 |= 0x80;
	}

	/*
	 * toggle interrupt output in case interrupts are still
	 * active, assuming interrupt is edge triggered:
	 */
	bus_space_write_1(t, h, 4, 0x00); /* write INT0_MASK */
	bus_space_write_1(t, h, 5, 0x00); /* write INT1_MASK */

	bus_space_write_1(t, h, 4, T_INT0_MASK); /* write INT0_MASK */
	bus_space_write_1(t, h, 5, T_INT1_MASK); /* write INT1_MASK */

	return;
}

static void
tiger300_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	if((f == CONFIG_WRITE_UPDATE) ||
	   (f == CONFIG_WRITE_RELOAD))
	{
	  /* nothing to configure */
	}
	else
	{
	  /* reset FIFO */
	  f->Z_drvr = (f->fm.h.Zend - f->fm.h.Zsize); /* min */
	  f->Z_chip = (f->fm.h.Zsize);

	  /* fill FIFO with last byte,
	   * which is 0xFF after FIFO-setup
	   */
	  tiger300_fifo_write_filler(sc,f);
	}
	return;
}

static ihfc_fifo_program_t *
tiger300_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if(FIFO_NO(f) == d1t)
	{
	  if(PROT_IS_HDLC(f->prot))
	    program = &i4b_ipac_tx_program;
	  else
	    if(PROT_IS_TRANSPARENT(f->prot))
	      program = &i4b_hfc_tx_program;
	}
	else
	{
	  if(PROT_IS_TRANSPARENT(f->prot))
	  {
	    program = (FIFO_DIR(f) == transmit) ? 
			&i4b_hfc_tx_program :
			&i4b_hfc_rx_program;
	  }
	}
	return program;
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(tiger300_dbase_root)
{
  /* ISAC - functions */

  I4B_DBASE_ADD(c_chip_read        , &tiger300_isac_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &tiger300_isac_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &tiger300_isac_chip_unselect);
  I4B_DBASE_ADD(c_fsm_read         , &tiger300_isac_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &tiger300_isac_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);

  I4B_DBASE_ADD(d_register_list    , &tiger300_isac_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */

  /* ISAC/AMD - functions
   *
   * ``tiger300_chip_reset()'' will autodetect ISAC or AMD c/o - chip !
   */

  I4B_DBASE_ADD(c_chip_reset       , &tiger300_chip_reset);
  I4B_DBASE_ADD(c_chip_config_write, &tiger300_chip_config_write);
  I4B_DBASE_ADD(c_chip_status_read , &tiger300_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &tiger300_chip_status_check);
  I4B_DBASE_ADD(c_fifo_read        , &tiger300_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &tiger300_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &tiger300_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_select      , &tiger300_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx      , &tiger300_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre  , &tiger300_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_fz_read     , &tiger300_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_write_filler, &tiger300_fifo_write_filler);

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF(tiger300_fifo_map[d1t]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF(tiger300_fifo_map[d1r]));
  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF(tiger300_fifo_map[b1t]));
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF(tiger300_fifo_map[b1r]));
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF(tiger300_fifo_map[b2t]));
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF(tiger300_fifo_map[b2r]));

  I4B_DBASE_ADD(d_interrupt_delay  , (hz / 20));

  I4B_DBASE_ADD(desc               , "Tiger3xx 128K PCI ISDN adapter");
  I4B_DBASE_ADD(io_rid[0]          , PCIR_BAR(0));
  I4B_DBASE_ADD(mem_rid[0]         , PCIR_BAR(1));

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0     , 1); /* enable */

  I4B_DBASE_ADD(o_TIGER_MWBA       , 1); /* enable */

  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */

#undef LED_SCHEME
#define LED_SCHEME(m)				\
  m(p1, 0x00,YES) /* non-existent */		\
  m(d1, 0x80,YES)				\
  m(b1, 0x20,YES)				\
  m(b2, 0x40,YES)				\
    /**/

  I4B_ADD_LED_SUPPORT(LED_SCHEME);
}

I4B_PCI_DRIVER(/* NetJet-S */
	       .vid           = 0x0001e159);

/*
 * cleanup
 */

#undef LED_SCHEME
#undef IPAC_LATCH_REG
#undef AMD_BUS_VAR
#undef AMD_READ_1
#undef AMD_WRITE_1
#undef AMD_READ_MULTI_1
#undef AMD_WRITE_MULTI_1

#undef CHIP_IS_AMD

#undef A_INIT
#undef T_AUX_CNTL
#undef T_INT0_MASK
#undef T_INT1_MASK
#undef T_PRCT

#undef IPAC_BUS_VAR
#undef IPAC_READ_1
#undef IPAC_WRITE_1
#undef IPAC_READ_MULTI_1
#undef IPAC_WRITE_MULTI_1
#endif /* _I4B_TIGER300_H_ */
