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
 *	i4b_hfcs2m.h - HFC-S2M driver module
 * 	------------------------------------
 *
 *	last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFCS2M_H_
#define _I4B_HFCS2M_H_

#if 0

static inline void
hfcs2m_chip_reset CHIP_RESET_T(sc,error)
{
}

register_list_t hfcs2m_register_list[] =
{
  { REG2OFF(s_cirm_pci),  0x00 },
  { REG2OFF(s_int_ctl),   0x13 },
  { REG2OFF(s_x_acc_en),  0x01 },

  /* NOTE: These registers are
   * loaded separately!
   *
   * { REG2OFF(s_fifo),   0x0f },
   * { REG2OFF(s_confer), 0xd0 },
   * { REG2OFF(s_ch_mask, 0xf4 },
   * { REG2OFF(s_int_mask,0xff },
   * { REG2OFF(s_con_hdlc,0xfa },
   * { REG2OFF(s_hdlc_par,0xfb },
   * { REG2OFF(s_channel, 0xfc },
   * { REG2OFF(s_slot,    0x10 },
   */
                
  { REG2OFF(s_receive0),  0x24 },
  { REG2OFF(s_rec_frame), 0x25 },
  { REG2OFF(s_transm0),   0x28 },
  { REG2OFF(s_transm1),   0x29 },
  { REG2OFF(s_trans_fra0),0x2c },
  { REG2OFF(s_trans_fra1),0x2d },
  { REG2OFF(s_trans_fra2),0x2e },
  { REG2OFF(s_receive_off),0x30},
  { REG2OFF(s_trans_off), 0x34 },
  { REG2OFF(s_mst_mode1), 0x15 },
  { REG2OFF(s_mst_mode0), 0x14 },
  { 0, 0 }
};

#endif

#endif /* _I4B_HFCS2M_H_ */
