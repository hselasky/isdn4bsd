/*-
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
 *	i4b_isac.h - ISAC/HSCX/IPAC (ISA,PnP,PCI) global include file
 * 	-------------------------------------------------------------
 *
 *	last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *	NOTE: ISAC-S TE is not capable of NT-mode, though the
 *	      data-lines can be switched as a replacement: SPCR:SDL (0x40)
 *
 *	NOTE: HSCX chips will send unstuffed 0xff-bytes when there
 *	      is no more data to transfer (XDU) in HDLC and transparent mode.
 *
 *	NOTE: If a TIN command is issued with less interval than the
 *	      timeout period, the timer will never elapse.
 *---------------------------------------------------------------------------*/

#ifndef _I4B_ISAC_H_
#define _I4B_ISAC_H_

/*---------------------------------------------------------------------------*
 * : ISAC EXIR table (unused)
 *
 *	const struct ihfc_EXIRtable { u_int8_t *desc; }
 *
 *	EXIRtable[8] =
 *	{
 *	  { "Watchdog Timer Overflow"		},
 *	  { "Subscriber Awake"			},
 *	  { "Monitor Status"			},
 *	  { "Rx Sync Xfer Overflow"		},
 *	  { "Rx Frame Overflow"			},
 *	  { "Protocol Error"			},
 *	  { "Tx Data Underrun"                  },
 *	  { "Tx Message Repeat"			},
 *	};
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*
 * : Table(s) related to statemachine
 * NOTE: if test functions should be used, o_DLOWPRI must
 * be set and ihfc_setup_softc(,) should be called to update
 * the configuration. Currently no test functions are supported.
 *
 * NOTE: LT-S/NT mode may need a separate table (not tested).
 *
 * #define isac_NT_table isac_fsm_table
 * #define isac_TE_table isac_fsm_table
 *---------------------------------------------------------------------------*/
fsm_t
isac_fsm_table =
{
  /* pending, active, can_up, can_down, command, index, description   */
  /*              p  a  u  d  c  i  description                       */
  .state[ 0] = {  0, 0, 1, 1, 1, 1, "Deactivate request"              },
  .state[ 4] = {  1, 0, 0, 1, 0, 0, "Level detected"                  },
  .state[ 6] = {  1, 0, 0, 0, 1, 1, "Error indication"                },
  .state[ 7] = {  0, 0, 0, 1, 0, 0, "Power-up"                        },
  .state[ 8] = {  1, 0, 0, 1, 0, 0, "Activate request downstream"     },
  .state[10] = {  0, 0, 0, 0, 0, 0, "Test indication"                 },
  .state[11] = {  0, 0, 0, 0, 0, 0, "Awake test indication"           },
  .state[12] = {  0, 1, 0, 0, 1, 0, "Activate indication (priority=8/9)" },
  .state[13] = {  0, 1, 0, 0, 1, 0, "Activate indication (priority=10/11)" },
  .state[15] = {  0, 0, 1, 0, 0, 0, "Deactivate indication downstream"},
                  /* NT-mode: "Deactivate indication upstream" */

  /* undefined states */
  .state[ 1] = { 0, 0, 0, 0, 0,  1, "Undefined state:  1!" },
  .state[ 2] = { 0, 0, 0, 0, 0,  2, "Undefined state:  2!" },
  .state[ 3] = { 0, 0, 0, 0, 0,  3, "Undefined state:  3!" },
  .state[ 5] = { 0, 0, 0, 0, 0,  5, "Undefined state:  5!" },
  .state[ 9] = { 0, 0, 0, 0, 0,  9, "Undefined state:  9!" },
  .state[14] = { 0, 0, 0, 0, 0, 14, "Undefined state: 14!" },

  /* value, description */
  .cmd[0]    = { 0x20, "Start activation"                },
  .cmd[1]    = { 0x3c, "Start deactivation"              },
};

/*---------------------------------------------------------------------------*
 * : HFC1 support
 * NOTE: This macro is used to convert IPAC registers to HFC1 registers.
 *---------------------------------------------------------------------------*/
#define REG2HFC1(reg) (				\
/* check if register				\
 * is ISAC and remove				\
 * 0x80 mask. Else the				\
 * IPAC register is not				\
 * supported.					\
 */						\
 ((reg) & 0x80) ? ((reg) & ~0x80) : 0x00	\
)

/*---------------------------------------------------------------------------*
 * : register definitions (sorted by name)
 *---------------------------------------------------------------------------*/
enum
{
  REG_hscxA_ccr1 = (0x2f | 0x00),
  REG_hscxA_ccr2 = (0x2c | 0x00),
  REG_hscxA_cmdr = (0x21 | 0x00),
  REG_hscxA_data = (0x00 | 0x00),
  REG_hscxA_exir = (0x24 | 0x00),
  REG_hscxA_ista = (0x20 | 0x00),
  REG_hscxA_mask = (0x20 | 0x00),
  REG_hscxA_mode = (0x22 | 0x00),
  REG_hscxA_rah2 = (0x27 | 0x00),
  REG_hscxA_rccr = (0x33 | 0x00),
  REG_hscxA_rlcr = (0x2e | 0x00),
  REG_hscxA_rsta = (0x27 | 0x00),
  REG_hscxA_stcr = (0x37 | 0x00),
  REG_hscxA_timr = (0x23 | 0x00),
  REG_hscxA_tsar = (0x31 | 0x00),
  REG_hscxA_tsax = (0x30 | 0x00),
  REG_hscxA_xad1 = (0x24 | 0x00),
  REG_hscxA_rbcl = (0x25 | 0x00),
  REG_hscxA_xad2 = (0x25 | 0x00),
  REG_hscxA_xbch = (0x2d | 0x00),
  REG_hscxA_xccr = (0x32 | 0x00),

  /* the  list  below  is  a
   * copy of the list  above
   * with the  exception  of
   * changing ``hscxA'' into
   * ``hscxB'' and ``| 0x00''
   * into ``| 0x40''
   */

  REG_hscxB_ccr1 = (0x2f | 0x40),
  REG_hscxB_ccr2 = (0x2c | 0x40),
  REG_hscxB_cmdr = (0x21 | 0x40),
  REG_hscxB_data = (0x00 | 0x40),
  REG_hscxB_exir = (0x24 | 0x40),
  REG_hscxB_ista = (0x20 | 0x40),
  REG_hscxB_mask = (0x20 | 0x40),
  REG_hscxB_mode = (0x22 | 0x40),
  REG_hscxB_rah2 = (0x27 | 0x40),
  REG_hscxB_rccr = (0x33 | 0x40),
  REG_hscxB_rlcr = (0x2e | 0x40),
  REG_hscxB_rsta = (0x27 | 0x40),
  REG_hscxB_stcr = (0x37 | 0x40),
  REG_hscxB_timr = (0x23 | 0x40),
  REG_hscxB_tsar = (0x31 | 0x40),
  REG_hscxB_tsax = (0x30 | 0x40),
  REG_hscxB_xad1 = (0x24 | 0x40),
  REG_hscxB_rbcl = (0x25 | 0x40),
  REG_hscxB_xad2 = (0x25 | 0x40),
  REG_hscxB_xbch = (0x2d | 0x40),
  REG_hscxB_xccr = (0x32 | 0x40),

  REG_ipac_acfg  = (0x03 | 0xc0),
  REG_ipac_aoe   = (0x04 | 0xc0),
  REG_ipac_atx   = (0x05 | 0xc0),
  REG_ipac_ista  = (0x01 | 0xc0),
  REG_ipac_mask  = (0x01 | 0xc0),
  REG_ipac_pota2 = (0x09 | 0xc0),

  REG_isac_adf1  = (0x38 | 0x80),
  REG_isac_adf2  = (0x39 | 0x80),
  REG_isac_cirq  = (0x31 | 0x80),
  REG_isac_cmdr  = (0x21 | 0x80),
  REG_isac_data  = (0x00 | 0x80),
  REG_isac_exir  = (0x24 | 0x80),
  REG_isac_ista  = (0x20 | 0x80),
  REG_isac_mask  = (0x20 | 0x80),
  REG_isac_mode  = (0x22 | 0x80),
  REG_isac_rsta  = (0x27 | 0x80),
  REG_isac_spcr  = (0x30 | 0x80),
  REG_isac_sqxr  = (0x3b | 0x80),
  REG_isac_star  = (0x21 | 0x80),
  REG_isac_star2 = (0x2b | 0x80),
  REG_isac_stcr  = (0x37 | 0x80),
  REG_isac_timr  = (0x23 | 0x80),
  REG_isac_rbcl  = (0x25 | 0x80),

  /* registers not present on PSB3186:
   * isac_adf2, isac_spcr, isac_star2,
   * isac_stcr
   *
   * NOTE: ISAC's ISTA moved to PSB's ISTAD(b_istad)
   *
   * NOTE: please check reset defaults for non-listed
   *       registers.
   */

  REG_psb3186_cir0 = (0x2e),
  REG_psb3186_cmdr = (0x21),
  REG_psb3186_data = (0x00),
  REG_psb3186_iom_cr = (0x57),
  REG_psb3186_ista = (0x60),
  REG_psb3186_istad = (0x20), /* XMR, XPR, RDO, RPF ... moved here!! */
  REG_psb3186_mask = (0x60),  /* NOTE: this register is not actually
			       * compatible with ISAC's MASK register
			       */
  REG_psb3186_maskd = (0x20),
  REG_psb3186_masktr = (0x39),
  REG_psb3186_mode = (0x22),
  REG_psb3186_msti = (0x59),
  REG_psb3186_rbcl = (0x26),
  REG_psb3186_rsta = (0x28),
  REG_psb3186_sqxr = (0x35),
  REG_psb3186_sres = (0x64),
  REG_psb3186_star = (0x21),
  REG_psb3186_sti = (0x58),
  REG_psb3186_timr = (0x24),
  REG_psb3186_tr_cr = (0x50),

  /*  REG_isacsx_asti = (?),*/
  REG_isacsx_data       = (0x00 | 0x80),
  REG_isacsx_auxi       = (0x61 | 0x80),
  REG_isacsx_auxm       = (0x61 | 0x80),
  REG_isacsx_cda1_cr    = (0x4e | 0x80),
  REG_isacsx_cda2_cr    = (0x4f | 0x80),
  REG_isacsx_cda_tsdp10 = (0x44 | 0x80),
  REG_isacsx_cda_tsdp11 = (0x45 | 0x80),
  REG_isacsx_cda_tsdp20 = (0x46 | 0x80),
  REG_isacsx_cda_tsdp21 = (0x47 | 0x80),
  REG_isacsx_cir0       = (0x2e | 0x80),
  REG_isacsx_cix0       = (0x2e | 0x80),
  REG_isacsx_cmdrd      = (0x21 | 0x80),
  REG_isacsx_codr1      = (0x2f | 0x80),
  REG_isacsx_codx1      = (0x2f | 0x80),
  REG_isacsx_dci_cr     = (0x53 | 0x80),
  REG_isacsx_id         = (0x64 | 0x80),
  REG_isacsx_iom_cr     = (0x57 | 0x80),
  REG_isacsx_ista       = (0x60 | 0x80),
  REG_isacsx_istad      = (0x20 | 0x80),
  REG_isacsx_istatr     = (0x38 | 0x80),
  REG_isacsx_mask       = (0x60 | 0x80),
  REG_isacsx_maskd      = (0x20 | 0x80),
  REG_isacsx_mcda       = (0x5b | 0x80),
  REG_isacsx_mconf      = (0x5f | 0x80),
  REG_isacsx_mon_cr     = (0x54 | 0x80),
  REG_isacsx_mor        = (0x5c | 0x80),
  REG_isacsx_mosr       = (0x5d | 0x80),
  REG_isacsx_mox        = (0x5c | 0x80),
  REG_isacsx_msta       = (0x5f | 0x80),
  REG_isacsx_msti       = (0x59 | 0x80),
  REG_isacsx_racgf2     = (0x3c | 0x80),
  REG_isacsx_rbchd      = (0x27 | 0x80),
  REG_isacsx_rbcld      = (0x26 | 0x80),
  REG_isacsx_rcda10     = (0x40 | 0x80),
  REG_isacsx_rcda11     = (0x41 | 0x80),
  REG_isacsx_rcda20     = (0x42 | 0x80),
  REG_isacsx_rcda21     = (0x43 | 0x80),
  REG_isacsx_rexmd1     = (0x23 | 0x80),
  REG_isacsx_rmasktr    = (0x39 | 0x80),
  REG_isacsx_rmocr      = (0x5e | 0x80),
  REG_isacsx_rmode1     = (0x62 | 0x80),
  REG_isacsx_rmode2     = (0x63 | 0x80),
  REG_isacsx_rmoded     = (0x22 | 0x80),
  REG_isacsx_rstad      = (0x28 | 0x80),
  REG_isacsx_rtimr1     = (0x24 | 0x80),
  REG_isacsx_rtimr2     = (0x65 | 0x80),
  REG_isacsx_rtmd       = (0x29 | 0x80),
  REG_isacsx_rtr_conf0  = (0x30 | 0x80),
  REG_isacsx_rtr_conf1  = (0x31 | 0x80),
  REG_isacsx_rtr_conf2  = (0x32 | 0x80),
  REG_isacsx_sap1       = (0x25 | 0x80),
  REG_isacsx_sap2       = (0x26 | 0x80),
  REG_isacsx_sds_conf   = (0x5a | 0x80),
  REG_isacsx_sds_cr     = (0x55 | 0x80),
  REG_isacsx_sqrr1      = (0x35 | 0x80),
  REG_isacsx_sqrr2      = (0x36 | 0x80),
  REG_isacsx_sqrr3      = (0x37 | 0x80),
  REG_isacsx_sqrx1      = (0x35 | 0x80),
  REG_isacsx_sres       = (0x64 | 0x80),
  /*  REG_isacsx_sta    = (? | 0x80),*/
  REG_isacsx_stard      = (0x21 | 0x80),
  REG_isacsx_sti        = (0x58 | 0x80),
  REG_isacsx_tr_cr      = (0x50 | 0x80),
  REG_isacsx_tr_tsdp_bc1= (0x4c | 0x80),
  REG_isacsx_tr_tsdp_bc2= (0x4d | 0x80),
  REG_isacsx_wacgf2     = (0x3c | 0x80),
  REG_isacsx_wcda10     = (0x40 | 0x80),
  REG_isacsx_wcda11     = (0x41 | 0x80),
  REG_isacsx_wcda20     = (0x42 | 0x80),
  REG_isacsx_wcda21     = (0x43 | 0x80),
  REG_isacsx_wexmd1     = (0x23 | 0x80),
  REG_isacsx_wmasktr    = (0x39 | 0x80),
  REG_isacsx_wmocr      = (0x5e | 0x80),
  REG_isacsx_wmode1     = (0x62 | 0x80),
  REG_isacsx_wmode2     = (0x63 | 0x80),
  REG_isacsx_wmoded     = (0x22 | 0x80),
  REG_isacsx_wtimr1     = (0x24 | 0x80),
  REG_isacsx_wtimr2     = (0x65 | 0x80),
  REG_isacsx_wtmd       = (0x29 | 0x80),
  REG_isacsx_wtr_conf0  = (0x30 | 0x80),
  REG_isacsx_wtr_conf1  = (0x31 | 0x80),
  REG_isacsx_wtr_conf2  = (0x32 | 0x80),
};

/*---------------------------------------------------------------------------*
 * : map of fifo registers for ISAC and HSCX chips
 *
 * NOTE: the driver may use virtual register(s) that are either
 *	 emulated or redirected.
 *
 * NOTE: when there is a HSCX or ISAC present it is assumed
 *	 that the ISAC will be linked to channels d1t and d1r.
 *
 * NOTE: usually [0] is ISAC and [1]+[2] are HSCX
 *       fifo maps.
 *---------------------------------------------------------------------------*/

I4B_FIFO_MAP_DECLARE(isac_fifo_map[]) =
{
  /*
   * IPAC D-channel FIFO map
   */

  [ 0].i=
  {
    .Zdata        = REG_isac_data,
    .block_size   = 0x20, /* = 32 bytes */
  },

  /*
   * HSCX B-channel FIFO map
   */

  [ 1].i=
  {
    .Zdata        = REG_hscxA_data,
    .remove_stat  = 1,    /* present */
    .block_size   = 0x20, /* = 32 bytes */
  },

  [ 2].i=
  {
    .Zdata        = REG_hscxB_data,
    .remove_stat  = 1,    /* present */
    .block_size   = 0x20, /* = 32 bytes */
  },

  /*
   * IPAC B-channel FIFO maps
   * PSB  B-channel FIFO maps
   *
   * 2 x 64 byte fifos
   */

  [ 3].i=
  {
    .Zdata        = REG_hscxA_data,
    .remove_stat  = 1,    /* present */
    .block_size   = 0x40, /* = 64 bytes */
  },

  [ 4].i=
  {
    .Zdata        = REG_hscxB_data,
    .remove_stat  = 1,    /* present */
    .block_size   = 0x40, /* = 64 bytes */
  },

  /*
   * unused FIFO maps 
   */

  [ 5].i= { },
  [ 6].i= { },
  [ 7].i= { },

  /*
   * HFC1 D-channel FIFO map
   */

  [ 8].i=
  {
    .Zdata        = REG2HFC1(REG_isac_data),
    .block_size   = 0x20, /* = 32 bytes */
  },
};

I4B_FIFO_MAP_DECLARE(psb3186_fifo_map[]) =
{
  /*
   * PSB3186 D-channel FIFO map
   */

  [ 0].i = {
    .Zdata        = REG_psb3186_data,
    .block_size   = 0x20, /* = 32 bytes = 64/2 bytes */
  },
};

/*---------------------------------------------------------------------------*
 * : IPAC register map definition
 * registers are tried loaded according to manual
 *---------------------------------------------------------------------------*/
#define CFG_ipac(x)    p_##x
#define CFG_isac(x)    i_##x
#define CFG_hscxA(x)   h_##x[0]
#define CFG_hscxB(x)   h_##x[1]
#define CFG_psb3186(x) b_##x

#define _ISAC_REGISTERS(macro)			\
     macro (isac , adf2   , )			\
     macro (isac , spcr   , )			\
     macro (isac , adf1   , )			\
     macro (isac , stcr   , )			\
     macro (isac , star2  , )			\
     macro (isac , mode   , )			\
     macro (isac , timr   , )			\
     macro (isac , mask   , )			\
     macro (isac , sqxr   , )			\
						\
/* end of _ISAC_REGISTERS(...) */

#define _HSCX_REGISTERS(macro,what)		\
     macro (hscx##what, xad1, )			\
     macro (hscx##what, xad2, )			\
     macro (hscx##what, rah2, )			\
     macro (hscx##what, xbch, )			\
     macro (hscx##what, rlcr, )			\
     macro (hscx##what, ccr2, )			\
     macro (hscx##what, xccr, )			\
     macro (hscx##what, rccr, )			\
     macro (hscx##what, tsax, )			\
     macro (hscx##what, tsar, )			\
     macro (hscx##what, mode, )			\
     macro (hscx##what, timr, )			\
     macro (hscx##what, mask, )			\
     macro (hscx##what, ccr1, )			\
						\
/* end of _HSCX_REGISTERS(...) */

#define _IPAC_REGISTERS(macro)			\
     macro (ipac, acfg    , )			\
     macro (ipac, atx     , )			\
     macro (ipac, aoe     , )			\
     macro (ipac, pota2   , )			\
     macro (ipac, mask    , )			\
						\
/* end of _IPAC_REGISTERS(...) */

#define b_mode i_mode /* same register */

#define _PSB3186_REGISTERS(macro)		\
     macro (psb3186, tr_cr   , )		\
     macro (psb3186, iom_cr  , )		\
     macro (psb3186, masktr  , )		\
     macro (psb3186, maskd   , )		\
     macro (psb3186, msti    , )		\
     macro (psb3186, sqxr    , )		\
     macro (psb3186, mode    , )		\
     macro (psb3186, timr    , )		\
     macro (psb3186, mask    , )		\
						\
/* end of _PSB3186_REGISTERS(...) */

#define __REG2LIST(chip,name,macro)		\
  { REG2OFF (CFG_##chip    (name)),		\
      macro (REG_##chip##_##name) },

#define _REG2HFC1(args...) __REG2LIST(args/*,*/REG2HFC1)

/*---------------------------------------------------------------------------*
 * : default register map for IPAC
 *---------------------------------------------------------------------------*/
register_list_t
generic_ipac_register_list[] =
{
  _IPAC_REGISTERS(__REG2LIST)
  _ISAC_REGISTERS(__REG2LIST)
  _HSCX_REGISTERS(__REG2LIST,A)
  _HSCX_REGISTERS(__REG2LIST,B)

  { 0, 0 }
};

/*---------------------------------------------------------------------------*
 * : default register map for ISAC / HSCX A/B
 *---------------------------------------------------------------------------*/
register_list_t
generic_isac_hscx_register_list[] =
{
  _ISAC_REGISTERS(__REG2LIST)
  _HSCX_REGISTERS(__REG2LIST,A)
  _HSCX_REGISTERS(__REG2LIST,B)

  { 0, 0 }
};

/*---------------------------------------------------------------------------*
 * : default register map for ISAC
 *---------------------------------------------------------------------------*/
register_list_t
generic_isac_only_register_list[] =
{
  _ISAC_REGISTERS(__REG2LIST)

  { 0, 0 }
};

/*---------------------------------------------------------------------------*
 * : default register map for HFC1 (excluding two non-mappable registers)
 *---------------------------------------------------------------------------*/
register_list_t
hfc1_register_list[] =
{
  _ISAC_REGISTERS(_REG2HFC1)

  { 0, 0 }
};

/*---------------------------------------------------------------------------*
 * : default register map for PSB3186
 *---------------------------------------------------------------------------*/
register_list_t
generic_psb3186_register_list[] =
{
  _PSB3186_REGISTERS(__REG2LIST)
  
  { 0, 0 }
};

/*---------------------------------------------------------------------------*
 * : generic ISAC/HSCX/IPAC code
 *---------------------------------------------------------------------------*/
#define ERROR_sc
#define IPAC_BUS_VAR(sc) ERROR_##sc /* check argument name */
#define IPAC_READ_MULTI_1(reg, ptr, len) CHIP_READ_MULTI_1(sc,reg,ptr,len)
#define IPAC_WRITE_MULTI_1(reg, ptr, len) CHIP_WRITE_MULTI_1(sc,reg,ptr,len)

static void __used
isac_hscx_generic_chip_reset_verify CHIP_RESET_T(sc,error)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp;

	/* read SPCR(ISAC) */
	IPAC_READ_MULTI_1(REG_isac_spcr, &temp, 1);
	if(temp & 0xff)
	{
		IHFC_ADD_ERR(error,"(SPCR(=0x%02x) != 0x00)",temp);
	}

	/* read MODE(ISAC) */
	IPAC_READ_MULTI_1(REG_isac_mode, &temp, 1);
	if(temp & 0xff)
	{
		IHFC_ADD_ERR(error,"(MODE(=0x%02x) != 0x00)",temp);
	}

	/* read STAR(ISAC) */
	IPAC_READ_MULTI_1(REG_isac_star, &temp, 1);
	if((temp & 0xC0) != 0x40)
	{
		IHFC_ADD_ERR(error,"(STAR(=0x%02x) != 0x4x)",temp);
	}
	return;
}

#if 0
static void __used
isac_hscx_generic_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	/* read ISAC, HSCX A or HSCX B */
	IPAC_READ_MULTI_1(reg,ptr,len);

	return;
}
#endif

static void __used
isac_hscx_generic_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	IPAC_BUS_VAR(sc);

	/* read FIFO */
	IPAC_READ_MULTI_1((f->fm.h.Zdata),ptr,len);
	return;
}

#if 0
static void __used
isac_hscx_generic_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	IPAC_BUS_VAR(sc);

	/* write ISAC, HSCX A or HSCX B */
	IPAC_WRITE_MULTI_1(reg,ptr,len);

	return;
}
#endif

static void __used
isac_hscx_generic_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	IPAC_BUS_VAR(sc);

	/* write FIFO */
	IPAC_WRITE_MULTI_1((f->fm.h.Zdata),ptr,len);
	return;
}

static ihfc_fifo_program_t * __used
isac_hscx_generic_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	if((FIFO_NO(f) == d1t) ||
	   (FIFO_NO(f) == d1r))
	{
		if(PROT_IS_HDLC(f->prot))
		{
		  program = (FIFO_DIR(f) == transmit) ?
		    &i4b_ipac_tx_program :
		    &i4b_ipac_rx_program;
		}
	}
	else
	{
		if(PROT_IS_HDLC(f->prot) ||
		   PROT_IS_TRANSPARENT(f->prot))
		{
		  program = (FIFO_DIR(f) == transmit) ?
		    &i4b_ipac_tx_program :
		    &i4b_ipac_rx_program;
		}
	}
	return program;
}

static void __used
isac_hscx_generic_fsm_read FSM_READ_T(sc,f,ptr)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp;

	/* read CIRQ (ISAC) */
	IPAC_READ_MULTI_1(REG_isac_cirq, &temp,1);

	*ptr = (temp >> 2) & 0xf;

	return;
}

static void __used
isac_hscx_generic_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp = (*ptr | sc->sc_config.i_cirq);

	/* write CIRQ (ISAC) */
	IPAC_WRITE_MULTI_1(REG_isac_cirq, &temp, 1);
	return;
}

static void __used
isac_hscx_generic_chip_unselect CHIP_UNSELECT_T(sc)
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
		IPAC_WRITE_MULTI_1(REG_isac_cmdr, &temp, 1);
		sc->sc_fifo[d1r].i_cmdr = 0;
		sc->sc_fifo[d1t].i_cmdr = 0;
	}

	temp = (sc->sc_fifo[b1r].i_cmdr | sc->sc_fifo[b1t].i_cmdr);

	/* write CMDR (HSCX A) */
	if(temp)
	{
		IPAC_WRITE_MULTI_1(REG_hscxA_cmdr, &temp, 1);
		sc->sc_fifo[b1r].i_cmdr = 0;
		sc->sc_fifo[b1t].i_cmdr = 0;
	}

	temp = (sc->sc_fifo[b2r].i_cmdr | sc->sc_fifo[b2t].i_cmdr);

	/* write CMDR (HSCX B) */
	if(temp)
	{
		IPAC_WRITE_MULTI_1(REG_hscxB_cmdr, &temp, 1);
		sc->sc_fifo[b2r].i_cmdr = 0;
		sc->sc_fifo[b2t].i_cmdr = 0;
	}

	/*
	 * take advantage of delay in
	 * resetting all MASKS, hence
	 * delay is needed after CMDR
	 * write. Resetting the MASKS
	 * also has the effect of
	 * clearing the IRQ which
	 * is good.
	 */

	temp = 0xFF;

	/* write MASK (ISAC) */
	IPAC_WRITE_MULTI_1(REG_isac_mask, &temp, 1);

	/* write MASK (HSCX A) */
	IPAC_WRITE_MULTI_1(REG_hscxA_mask, &temp, 1);

	/* write MASK (HSCX B) */
	IPAC_WRITE_MULTI_1(REG_hscxB_mask, &temp, 1);

	/* an extra delay after CMDR write, 
	 * is needed if the I/O is memory
	 * mapped. Typically transferring 
	 * one byte using I/O takes 1.5us.
	 */
	/* if(t == I386_BUS_SPACE_MEM) */
	DELAY(12);

	temp = sc->sc_config.i_mask;

	/* write MASK (ISAC) */
	IPAC_WRITE_MULTI_1(REG_isac_mask, &temp, 1);

	temp = sc->sc_config.h_mask[0];

	/* write MASK (HSCX A) */
	IPAC_WRITE_MULTI_1(REG_hscxA_mask, &temp, 1);

	temp = sc->sc_config.h_mask[1];

	/* write MASK (HSCX B) */
	IPAC_WRITE_MULTI_1(REG_hscxB_mask, &temp, 1);

	return;
}

static void __used
isac_hscx_generic_chip_status_read CHIP_STATUS_READ_T(sc)
{
	IPAC_BUS_VAR(sc);
	u_int8_t temp;

	/*
	 * generic ISAC/HSCX code:
	 */

	/* read ISTA (ISAC) */
	IPAC_READ_MULTI_1(REG_isac_ista, &temp, 1);
	sc->sc_config.i_ista |= temp;

	if(temp & 0x80 /* RME */)
	{
		/* read RBCL (ISAC) */
		IPAC_READ_MULTI_1(REG_isac_rbcl, &temp, 1);
		sc->sc_fifo[d1r].Z_chip = temp;
	  
		/* read RSTA (ISAC) */
		IPAC_READ_MULTI_1(REG_isac_rsta, &temp, 1);
		sc->sc_fifo[d1r].F_chip = temp;
	}

	if(sc->sc_config.i_ista & 0x01)
	{
		/* read EXIR (ISAC) */
		IPAC_READ_MULTI_1(REG_isac_exir, &temp, 1);
		sc->sc_config.i_exir |= temp;
	}

	/* check for SIN (1500us timeout) */
	if(sc->sc_config.i_ista & 0x02)
	{
		if(sc->sc_config.i_stcr & 0x0f)
		{
			temp = sc->sc_config.i_stcr & 0xf3;

			/* write STCR (ISAC) */
			IPAC_WRITE_MULTI_1(REG_isac_stcr, &temp, 1);

			sc->sc_config.i_stcr &= 0xf0;
			SC_T125_WAIT_CLEAR(sc);
		}
	}

	/* read ISTA (HSCX B) */
	IPAC_READ_MULTI_1(REG_hscxB_ista, &temp, 1);
	sc->sc_config.h_ista |= (temp << 8);

	if(temp & 0x80 /* RME */)
	{
		/* read RBCL (HSCX B) */
		IPAC_READ_MULTI_1(REG_hscxB_rbcl, &temp, 1);
		sc->sc_fifo[b2r].Z_chip = temp;

		/* read RSTA (HSCX B) */
		IPAC_READ_MULTI_1(REG_hscxB_rsta, &temp, 1);
		sc->sc_fifo[b2r].F_chip = temp;
	}

	if(sc->sc_config.h_ista & 0x0100)
	{
		/* read EXIR (HSCX B) */
		IPAC_READ_MULTI_1(REG_hscxB_exir, &temp, 1);
		sc->sc_config.h_exir |= (temp << 8);
	}

	if(sc->sc_config.h_ista & 0x0200)
	{
		/* read EXIR (HSCX A) */
		IPAC_READ_MULTI_1(REG_hscxA_exir, &temp, 1);
		sc->sc_config.h_exir |= temp;
	}

	if(sc->sc_config.h_ista & 0x0400)
	{
		/* read ISTA (HSCX A) */
		IPAC_READ_MULTI_1(REG_hscxA_ista, &temp, 1);
		sc->sc_config.h_ista |= temp;

		if(temp & 0x80 /* RME */)
		{
			/* read RBCL (HSCX A) */
			IPAC_READ_MULTI_1(REG_hscxA_rbcl, &temp, 1);
			sc->sc_fifo[b1r].Z_chip = temp;

			/* read RSTA (HSCX A) */
			IPAC_READ_MULTI_1(REG_hscxA_rsta, &temp, 1);
			sc->sc_fifo[b1r].F_chip = temp;
		}
	}
	return;
}

#undef ERROR_sc
#undef IPAC_BUS_VAR
#undef IPAC_READ_MULTI_1
#undef IPAC_WRITE_MULTI_1

#endif /* _I4B_ISAC_H_ */
