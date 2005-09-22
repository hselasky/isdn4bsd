/*-
 * Copyright (c) 1998 Eivind Eklund. All rights reserved.
 *
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 1999 Ari Suutari. All rights reserved.
 *
 * Copyright (c) 1999, 2000 Udo Schweigert. All rights reserved.
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
 * Udo Schweigert had two more conditions:
 *   3. Neither the name of the author nor the names of any co-contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *   4. Altered versions must be plainly marked as such, and must not be
 *      misrepresented as being the original software and/or documentation.
 * 
 * Karsten Kiel originally wrote four initialization values for the IPAC.
 *
 *---------------------------------------------------------------------------
 *
 *	I4B Siemens IPAC v1.1+ ISDN Chipset Driver
 *	------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_IPAC1_H_
#define _I4B_IPAC1_H_

#include <i4b/layer1/ihfc2/i4b_isac.h>

/* imports */
#define ipac1_chip_reset_verify  isac_hscx_generic_chip_reset_verify
#define ipac1_fifo_get_program   isac_hscx_generic_fifo_get_program
#define ipac1_fsm_read           isac_hscx_generic_fsm_read
#define ipac1_fsm_write          isac_hscx_generic_fsm_write
#define ipac1_chip_unselect      isac_hscx_generic_chip_unselect

/*
 * (from i4b_isic_pnp.c ....)
 *
 * (from i4b_elsa_qs1p.c ....)
 *
 * ELSA Quickstep 1000pro PCI = ELSA MicroLink ISDN/PCI:
 * ====================================================
 *
 * #define MEM0_MAPOFF  0
 *
 * #define PCI_QS1000_DID       0x1000
 * #define PCI_QS1000_VID       0x1048
 *
 * #define PCI_QS1000_DEV_VID   0x10001048
 *
 *
 * masks for register encoded in base addr
 *
 *
 * #define ELSA_BASE_MASK        0x0ffff
 * #define ELSA_OFF_MASK         0xf0000
 *
 * register id's to be encoded in base addr
 *
 * #define ELSA_IDISAC           0x00000
 * #define ELSA_IDHSCXA          0x10000
 * #define ELSA_IDHSCXB          0x20000
 * #define ELSA_IDIPAC           0x40000
 *
 * offsets from base address
 *
 * #define ELSA_OFF_ALE          0x00
 * #define ELSA_OFF_RW           0x01
 *
 *
 * (from i4b_siemens_isurf.c ....)
 *
 * SIEMENS I-Surf 2.0 PnP:
 * =======================
 *
 * masks for register encoded in base addr
 *
 * #define SIE_ISURF_BASE_MASK   0x0ffff
 * #define SIE_ISURF_OFF_MASK    0xf0000
 *
 * register id's to be encoded in base addr
 *
 * #define SIE_ISURF_IDISAC      0x00000
 * #define SIE_ISURF_IDHSCXA     0x10000
 * #define SIE_ISURF_IDHSCXB     0x20000
 * #define SIE_ISURF_IDIPAC      0x40000
 *
 * offsets from base address
 *
 * #define SIE_ISURF_OFF_ALE     0x00
 * #define SIE_ISURF_OFF_RW      0x01
 *
 * (from i4b_asuscom_ipac.c ....)
 *
 * Asuscom ISDNlink 128K PnP:
 * ==========================
 *
 * The Asuscom ISDNlink 128K PnP ISA adapter is based on Siemens
 * IPAC chip (my card probes as ASU1690).      Older Asuscom ISA
 * cards are based on a different chipset containing two chips.
 * For those cards, one might want to try the Dynalink driver.
 *
 * masks for register encoded in base addr
 *
 * #define ASI_BASE_MASK        0x0ffff
 * #define ASI_OFF_MASK         0xf0000
 *
 * register id's to be encoded in base addr
 *
 * #define ASI_IDISAC           0x00000
 * #define ASI_IDHSCXA          0x10000
 * #define ASI_IDHSCXB          0x20000
 * #define ASI_IDIPAC           0x40000
 *
 * offsets from base address
 *
 * #define ASI_OFF_ALE          0x00
 * #define ASI_OFF_RW           0x01
 *
 */

#define IPAC_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.io_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.io_hdl[0];

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
ipac1_chip_reset CHIP_RESET_T(sc,error)
{
  IPAC_BUS_VAR(sc);

  /* perform IPAC reset
   *
   * write POTA2(IPAC)
   */
  IPAC_WRITE_1(REG_ipac_pota2, sc->sc_config.p_pota2 ^ 0x20);
  DELAY(4000); /* 4ms, according to manual */

  IPAC_WRITE_1(REG_ipac_pota2, sc->sc_config.p_pota2);
  DELAY(4000); /* 4ms, according to manual */

  ipac1_chip_reset_verify(sc, error);
  return;
}

static void
ipac1_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
  IPAC_BUS_VAR(sc);

  /* read ISAC, HSCX A or HSCX B */
  IPAC_READ_MULTI_1(reg,ptr,len);
  return;
}

static void
ipac1_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
  ipac1_chip_read(sc,(f->fm.h.Zdata),ptr,len);
  return;
}

static void
ipac1_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
  IPAC_BUS_VAR(sc);

  /* write ISAC, HSCX A or HSCX B */
  IPAC_WRITE_MULTI_1(reg,ptr,len);
  return;
}

static void
ipac1_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
  ipac1_chip_write(sc,(f->fm.h.Zdata),ptr,len);
  return;
}

static void
ipac1_chip_status_read CHIP_STATUS_READ_T(sc)
{
  IPAC_BUS_VAR(sc);
  register u_int8_t p_tmp,tmp;

  /*
   * generic IPAC code:
   */

  /* read ISTA(IPAC) */
  IPAC_READ_1(REG_ipac_ista, p_tmp);
  sc->sc_config.p_ista |= p_tmp;


  if(p_tmp & 0x01)
  {
          /* read EXIR (HSCX B) */
          IPAC_READ_1(REG_hscxB_exir, tmp);
          sc->sc_config.h_exir |= (tmp << 8);
  }

  if(p_tmp & 0x02)
  {
          /* read ISTA (HSCX B) */
          IPAC_READ_1(REG_hscxB_ista, tmp);
          sc->sc_config.h_ista |= (tmp << 8);

          if(tmp & 0x80 /* RME */)
          {
                  /* read RBCL (HSCX B) */
                  IPAC_READ_1(REG_hscxB_rbcl, tmp);
                  sc->sc_fifo[b2r].Z_chip = tmp;

                  /* read RSTA (HSCX B) */
                  IPAC_READ_1(REG_hscxB_rsta, tmp);
                  sc->sc_fifo[b2r].F_chip = tmp;
          }
  }

  if(p_tmp & 0x04)
  {
          /* read EXIR (HSCX A) */
          IPAC_READ_1(REG_hscxA_exir, tmp);
          sc->sc_config.h_exir |= tmp;
  }

  if(p_tmp & 0x08)
  {
          /* read ISTA (HSCX A) */
          IPAC_READ_1(REG_hscxA_ista, tmp);
          sc->sc_config.h_ista |= tmp;

          if(tmp & 0x80 /* RME */)
          {
                  /* read RBCL (HSCX A) */
                  IPAC_READ_1(REG_hscxA_rbcl, tmp);
                  sc->sc_fifo[b1r].Z_chip = tmp;

                  /* read RSTA (HSCX A) */
                  IPAC_READ_1(REG_hscxA_rsta, tmp);
                  sc->sc_fifo[b1r].F_chip = tmp;
          }
  }

  if(p_tmp & 0x10)
  {
          /* read EXIR (ISAC) */
          IPAC_READ_1(REG_isac_exir, tmp);
          sc->sc_config.i_exir |= tmp;
  }

  if(p_tmp & 0x20)
  {
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
  }
  return;
}


static void
ipac1_common_chip_reset CHIP_RESET_T(sc,error)
{
	bus_space_tag_t    tt  = sc->sc_resources.io_tag[1];
	bus_space_handle_t hh  = sc->sc_resources.io_hdl[1];

	/* call generated reset */
	ipac1_chip_reset(sc,error);

	switch(sc->sc_cookie) {
		case 1:
		  /* enable ELSA microlink card interrupt */
		  bus_space_write_1(tt, hh, 0x4c, 0x41);
		  break;

		default:
		  break;
	}
	return;
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(ipac1_dbase_root)
{
  I4B_DBASE_ADD(c_chip_reset       , &ipac1_common_chip_reset );
  I4B_DBASE_ADD(c_chip_read        , &ipac1_chip_read);
  I4B_DBASE_ADD(c_chip_write       , &ipac1_chip_write);
  I4B_DBASE_ADD(c_chip_unselect    , &ipac1_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read , &ipac1_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check, &default_chip_status_check);
  I4B_DBASE_ADD(c_fsm_read         , &ipac1_fsm_read);
  I4B_DBASE_ADD(c_fsm_write        , &ipac1_fsm_write);
  I4B_DBASE_ADD(d_fsm_table        , &isac_fsm_table);
  I4B_DBASE_ADD(c_fifo_read        , &ipac1_fifo_read);
  I4B_DBASE_ADD(c_fifo_write       , &ipac1_fifo_write);
  I4B_DBASE_ADD(c_fifo_get_program , &ipac1_fifo_get_program);

  I4B_DBASE_ADD(d_register_list    , &generic_ipac_register_list[0]);
  I4B_DBASE_ADD(d_channels         , 6);

  I4B_DBASE_ADD(d_interrupt_delay  , hz / (140 ));

  I4B_DBASE_ADD(o_BUS_TYPE_IOM2    , 1); /* enable */
  I4B_DBASE_ADD(o_IPAC             , 1); /* enable */

  I4B_DBASE_ADD(d_fifo_map[d1t]    , FM2OFF (isac_fifo_map[0]));
  I4B_DBASE_ADD(d_fifo_map[d1r]    , FM2OFF (isac_fifo_map[0]));

  I4B_DBASE_ADD(d_fifo_map[b1t]    , FM2OFF (isac_fifo_map[1+2])); 
  I4B_DBASE_ADD(d_fifo_map[b1r]    , FM2OFF (isac_fifo_map[1+2])); 
  I4B_DBASE_ADD(d_fifo_map[b2t]    , FM2OFF (isac_fifo_map[2+2])); 
  I4B_DBASE_ADD(d_fifo_map[b2r]    , FM2OFF (isac_fifo_map[2+2]));

  I4B_DBASE_ADD(desc               , "IPAC v1.x ISDN card");

  I4B_DBASE_ADD(o_RES_IRQ_0        , 1); /* enable */
  I4B_DBASE_ADD(o_RES_IOPORT_0     , 1); /* enable */
}

I4B_PNP_DRIVER(/* Siemens I-Surf 2.0 PnP */
	       .vid           = 0x2000254d);

I4B_PNP_DRIVER(/* Eicon DIVA 2.02 IPAC */
	       .vid           = 0xa100891c);

I4B_PNP_DRIVER(/* Asuscom ISDNLink 128 PnP (with IPAC) */
	       .vid           = 0x90167506);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(ipac1_dbase_root);

  I4B_DBASE_ADD(cookie          , 1);
  I4B_DBASE_ADD(desc            , "ELSA MicroLink ISDN/PCI");

  /* order of I/O - ports
   * is switched:
   */
  I4B_DBASE_ADD(io_rid[0]       , PCIR_BAR(3));
  I4B_DBASE_ADD(io_rid[1]       , PCIR_BAR(1));

  I4B_DBASE_ADD(o_RES_IRQ_0     , 1); /* enable (also enabled in root) */
  I4B_DBASE_ADD(o_RES_IOPORT_0  , 1); /* enable (also enabled in root) */
  I4B_DBASE_ADD(o_RES_IOPORT_1  , 1); /* enable */
}

I4B_PCI_DRIVER(/* ELSA MicroLink ISDN/PCI */
	       .vid           = 0x10001048);


#undef ipac1_chip_reset_verify
#undef ipac1_fifo_get_program
#undef ipac1_fsm_read
#undef ipac1_fsm_write
#undef ipac1_chip_unselect
#undef IPAC_BUS_VAR
#undef IPAC_READ_1
#undef IPAC_WRITE_1
#undef IPAC_READ_MULTI_1
#undef IPAC_WRITE_MULTI_1
#endif /* _I4B_IPAC1_H_ */
