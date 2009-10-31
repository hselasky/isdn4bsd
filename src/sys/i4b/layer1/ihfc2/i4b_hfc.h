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
 *	i4b_hfc.h - HFC-2B/2BDS0 (ISA,PnP,PCI,USB) global include file
 * 	--------------------------------------------------------------
 *
 * $FreeBSD: $
 *
 *      NOTE: HDLC emulation is used instead of hardware
 *            accelerated HDLC because of two reasons:
 *
 *            1) hardware transmit FIFO underruns will not be aborted
 *            with 0xff bytes. Instead a valid CRC and FLAG is sent,
 *            according to the manual.  This cause problems with
 *            protocols that depend on fixed fields after frame
 *            start. At least the older HFC CHIPS behave like that.
 *
 *            2) at least two bytes must be put into the FIFO before
 *            one BUSY/NON-BUSY sequence occurs else the CRC bytes
 *            will be inserted into the data stream, according to the
 *            manual.
 *
 *            With D-channel experiments show that, CRC bytes will not
 *            be inserted until after three BUSY / NON-BUSY sequences,
 *            when the FIFO was empty.  The reason  might be that the
 *            host-transmit D-channel must wait till it gets continue-
 *            signal from the S/T part, due to D- channel priority and
 *            collision detection, before the transmission is started.
 *            So this problem only exist for non D-channels using
 *            hardware HDLC.
 *
 *            In the worst case 125*6*20us == 15ms will be waisted
 *            waiting for BUSY/NON-BUSY transitions per second, which
 *            equals 1.512% CPU usage, and is currently not done.
 *
 *	      3) some older chips does not implement FIFO-reset, but
 *	      only chip-reset, which is deferred during usage. For
 *	      example when switching from HDLC-mode to [extended]
 *	      transparent mode a FIFO reset is necessary!
 *
 *      NOTE: HFC chips will repeat the last byte when there is no more
 *            data to transfer (XDU) in transparent mode.
 *
 *      NOTE: All HFC fifo counters are incremented:
 *              - start value: (f->fm.h.Zend - f->fm.h.Zsize) inclusive
 *              - end value  : (f->fm.h.Zend)                 exclusive
 *
 *      NOTE: 8Kbyte fifo mode is used instead 32kbyte fifo mode for the
 *            HFC-1/S/SP chips, hence it can always be configured without
 *            knowing the actual SRAM-size.
 *
 *      NOTE: The HFC-S-PCI FIFO needs about 250us to activate. This delay
 *            is not so important,  hence the MWBA already contains F- and
 *            Z-counter-values that will be restarted.
 *
 *      NOTE: bit[0] of STATUS/DISBUSY register, will remain zero
 *            some time after reset and until the chip is configured for
 *            master-mode.
 *
 *      NOTE: when the manual use the term ``must be set to 0'' or
 *            alike, about bits that can be written,   that  means
 *            these bits have undocumented functions!!
 *
 *      NOTE: when a PCM30 slot is disabled, the last byte written to
 *            that slot, is repeated, which may be undefined!
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFC_H_
#define _I4B_HFC_H_

/* import CRC table */

#include <i4b/layer1/i4b_hdlc.h>

static u_int8_t
hfcs_fifo_frame_check FIFO_FRAME_CHECK_T(sc,f,m)
{
	/* NOTE: After a Frame or Zdata hardware overflow  *
	 * it  appears not to be necessary  to  reset the  *
	 * HFC  for  continued  operation, if the  driver  *
	 * always  reads  "Z1-Z2+1" byte(s)  when  F1!=F2  *
	 * followed by an F2-counter  increment. The only  *
	 * backdraw about this is  that  the "STAT" field  *
	 * can read valid when the frame is actually bad.  *
	 * The  simple  solution to this is to re-CRC the  *
	 * frame including the "STAT" field to see if the  *
	 * resulting  CRC  is equal to 0x3933. Then we're  *
	 * more  certain  all  frames received  are  good, *
	 * without   having  to   do  ``advanced''  error  *
	 * checking.                                       */

	register u_int16_t  crc = 0xffff;
	register u_int16_t  len = m->m_len;
	register const u_int8_t * ptr = m->m_data;

	/* check length */
	if(len < 3)
	{
	    ptr = "Invalid frame: len < 3 bytes";
	    goto error_rx;
	}

	/* generate CRC */
	while(len--)
	{
	    crc = (HDLC_FCS_TAB[(u_int8_t)(*ptr++ ^ crc)]
		   ^ (u_int8_t)(crc >> 8));
	}

	/* check CRC */
	if(crc ^ 0x3933 /* (0xf0b8) */)
	{
	    ptr = "Invalid frame: checksum error";
	    goto error_rx;
	}

	len = m->m_len;
	ptr = m->m_data;

	/* check STAT */
	if(ptr[len-1] != 0x00)
	{
	    ptr = "Invalid frame: STAT-field != 0x00";
	    goto error_rx;
	}

	/* remove STAT+CRC bytes and update mbuf */
	m->m_len = (len - 3);

	/* success */
	return 0;

 error_rx:
	HDLC_ERR("(#%d) len=%d %s\n",
		 FIFO_NO(f), m->m_len, ptr);

	/* failure */
	return 1;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
#define HFC_USE_HARDWARE_HDLC NO /* it is possible to use YES */

/*---------------------------------------------------------------------------*
 * DISBUSYYDUBSID
 *---------------------------------------------------------------------------*/
#define DISBUSY(last,to,errcmd)				\
  if((last) & 0x100) {					\
     to = 255;						\
     /* read STATUS (DISBUSY) */			\
     while(to && (bus_space_read_1(t,h,1) & 1)) {	\
		DELAY(5);				\
		to--;					\
     }							\
     if(!to) { errcmd }					\
  }

/*---------------------------------------------------------------------------*
 * : Tables related to statemachine
 *
 * NOTE: In NT-mode STATE G3 can be locked
 *	 for stability reasons. (please see manual)
 *
 * #define hfc_NT_table hfc_fsm_table
 * #define hfc_TE_table hfc_fsm_table
 *---------------------------------------------------------------------------*/
fsm_t
hfc_fsm_table =
{
#define HFC_TE_OFFSET 0

  /* pending, active, can_up, can_down, command, index, description */
  .state[ 0] = { 0, 0, 0, 0, 1, 2, "F0: Reset"                  },
  .state[ 2] = { 0, 0, 0, 0, 0, 0, "F2: Sensing"                },
  .state[ 3] = { 0, 0, 1, 0, 0, 0, "F3: Deactivated"            },
  .state[ 4] = { 1, 0, 0, 1, 0, 0, "F4: Awaiting signal"        },
  .state[ 5] = { 1, 0, 0, 1, 0, 0, "F5: Identifying input"      },
  .state[ 6] = { 1, 0, 0, 1, 0, 0, "F6: Syncronized"            },
  .state[ 7] = { 0, 1, 0, 0, 0, 0, "F7: Activated"              },
  .state[ 8] = { 0, 0, 0, 0, 0, 0, "F8: Lost framing"           },

#define HFC_NT_OFFSET 9

  /* pending, active, can_up, can_down, command, index, description */
  .state[ 9] = { 0, 0, 0, 0, 1, 2, "G0: Reset"                  },
  .state[10] = { 0, 0, 1, 0, 0, 0, "G1: Deactive"               },
  .state[11] = { 0, 0, 0, 1, 1, 3, "G2: Pending activation"     },
  .state[12] = { 0, 1, 0, 1, 1, 4, "G3: Active"                 },
  .state[13] = { 0, 0, 1, 0, 0, 0, "G4: Pending deactivation"   }, 

#undef HFC_FSM_RESTART /* fsm does currently not need restart */

  /* undefined states */
  .state[ 1] = { 0, 0, 0, 0, 0,  1, "Undefined state:  1" },
  .state[14] = { 0, 0, 0, 0, 0, 14, "Undefined state: 14" },
  .state[15] = { 0, 0, 0, 0, 0, 15, "Undefined state: 15" },

  /* value, description */
  .cmd[0]    = { 0x60, "Start activation"                 },
  .cmd[1]    = { 0x40, "Start deactivation"               },
  .cmd[2]    = { 0x00, "Releasing statemachine"           },
  .cmd[3]    = { 0x80, "Allowing G2->G3 transition"       },
  .cmd[4]    = { 0x13, "Locking G3"                       },
  .cmd[5]    = { 0x17, "Locking F7"                       },
};

fsm_t
hfce1_fsm_table =
{
  /* pending, active, can_up, can_down, command, index, description */
  .state[ 0] = { 0, 0, 1, 0, 0, 0, "F0: Power off at TE"        },
  .state[ 1] = { 0, 1, 0, 1, 0, 0, "F1: Operational"            },
  .state[ 2] = { 0, 1, 0, 1, 0, 0, "F2: FC1"                    },
  .state[ 3] = { 1, 1, 0, 1, 0, 0, "F3: FC2"                    },
  .state[ 4] = { 1, 0, 0, 1, 0, 0, "F4: FC3"                    },
  .state[ 5] = { 0, 1, 0, 1, 0, 0, "F5: FC4"                    },
  .state[ 6] = { 0, 0, 0, 1, 0, 0, "F6: Power on at TE"         },

  /* pending, active, can_up, can_down, command, index, description */
  .state[ 9] = { 0, 0, 1, 0, 0, 0, "G0: Power off at NT"        },
  .state[10] = { 0, 1, 0, 1, 0, 0, "G1: Operational"            },
  .state[11] = { 0, 0, 0, 0, 0, 0, "G2: Undefined"              },
  .state[12] = { 1, 1, 0, 1, 0, 0, "G3: FC2"                    },
  .state[13] = { 0, 0, 0, 0, 0, 0, "G4: Undefined"              },
  .state[14] = { 1, 0, 0, 1, 0, 0, "G5: FC4"                    },
  .state[15] = { 0, 0, 0, 1, 0, 0, "G6: Power on at NT"         },

  /* undefined states */
  .state[ 7] = { 0, 0, 0, 0, 0, 7, "Undefined state: 7!" },
  .state[ 8] = { 0, 0, 0, 0, 0, 8, "Undefined state: 8!" },

  /* value, description */
  .cmd[0]    = { 0x60, "Start activation"                },
  .cmd[1]    = { 0x40, "Start deactivation"              },
  .cmd[2]    = { 0x00, "Releasing statemachine"          },
  .cmd[3]    = { 0x11, "Locking G1"                      },
  .cmd[4]    = { 0x11, "Locking F1"                      },
};

/*---------------------------------------------------------------------------*
 * : Map of fifo registers for HFC-1/S/SP/SPCI/USB
 *
 * HFC-1,S,SP,USB:
 * ===============
 * Z1 register value = READ(Zbase ^0) + (READ(Zbase ^4) << 8)
 * Z2 register value = READ(Zbase ^8) + (READ(Zbase ^C) << 8)
 * F1 register value = READ(Fbase ^0)
 * F2 register value = READ(Fbase ^4)
 *
 * HFC-SPCI NOTES:
 * ===============
 * Z1 register value = READ_MEM_2(Zbase ^0) (16-bit)
 * Z2 register value = READ_MEM_2(Zbase ^2) (16-bit)
 * F1 register value = READ_MEM_1(Fbase ^0) (8-bit)
 * F2 register value = READ_MEM_1(Fbase ^1) (8-bit)
 *
 *     - offsets are given in 8-bit granularity,
 *       relative to the start of the
 *       MemoryWindowBaseAddress.
 *
 *     - B-channel Zdata has been decremented by
 *       0x200, for optimization.
 *
 *     - D-channel Zbase has been decremented by
 *       0x10*4, for optimization.
 *
 * HFC-S USB NOTES:
 * ================
 *
 *     - the current driver does not use Zsize,
 *       Zdata, Fbase, Fsize and Fibase register
 *       access to transfer data, but the
 *       ``isochronous pipe interface''. Indexes
 *       18 to 21 are not used.
 *
 * NOTES:
 * ======
 *
 *     - OR 1,2,4,8 is used to manipulate the
 *       order of the registers, so that ``chip-
 *       incremented-registers'' are always
 *       first and ``driver-incremented-
 *       registers'' last.
 *
 *     - Fend = (0 + Fsize) + (0x10 if D-channel)
 *
 *      { [Zbase], [Zsize],  [Zdata], [Fbase], [Fsize], [Fibase], [Zend] }
 *---------------------------------------------------------------------------*/
I4B_FIFO_MAP_DECLARE(hfc_fifo_map[]) = {
/*-D-channel-HFC-SP---------------8/32K----------------------------------------*/
[ 0].h={ 8|   0x80,  0x200,   0xac, 4|   0xb0, 0x10,   0xa8, 0x0200, 0, 0}, /* D1_t */
[ 1].h={      0x81,  0x200,   0xbd,      0xb1, 0x10,   0xb9, 0x0200, 0, 0}, /* D1_r */
/*-D-channel-HFC-S----------------8/32k----------------------------------------*/
[ 2].h={ 8|   0x90,  0x200,   0x96, 4|   0x9a, 0x10,   0x92, 0x0200, 0, 0}, /* D1_t */
[ 3].h={      0x91,  0x200,   0xa7,      0x9b, 0x10,   0xa3, 0x0200, 0, 0}, /* D1_r */
/*-B-channel-HFC-1/S/SP-----------8k-------------------------------------------*/
[ 4].h={ 8|   0x80,  0x600,   0xac, 4|   0xb0, 0x20,   0xa8, 0x2000, 0, 0}, /* B1_t */
[ 5].h={      0x81,  0x600,   0xbd,      0xb1, 0x20,   0xb9, 0x2000, 0, 0}, /* B1_r */
[ 6].h={ 8|   0x82,  0x600,   0xae, 4|   0xb2, 0x20,   0xaa, 0x2000, 0, 0}, /* B2_t */
[ 7].h={      0x83,  0x600,   0xbf,      0xb3, 0x20,   0xbb, 0x2000, 0, 0}, /* B2_r */
/*-B-channel-HFC-1/S/SP-----------32k------------------------------------------*/
[ 8].h={ 8|   0x80, 0x1e00,   0xac, 4|   0xb0, 0x20,   0xa8, 0x2000, 0, 0}, /* B1_t */
[ 9].h={      0x81, 0x1e00,   0xbd,      0xb1, 0x20,   0xb9, 0x2000, 0, 0}, /* B1_r */
[10].h={ 8|   0x82, 0x1e00,   0xae, 4|   0xb2, 0x20,   0xaa, 0x2000, 0, 0}, /* B2_t */
[11].h={      0x83, 0x1e00,   0xbf,      0xb3, 0x20,   0xbb, 0x2000, 0, 0}, /* B2_r */
/*-D-channel-HFC-SPCI-------------32k------------------------------------------*/
[12].h={ 2| 0x2080,  0x200, 0x0000, 1| 0x20a0, 0x10, 0x20a0, 0x0200, 0, 0}, /* D1_t */
[13].h={    0x6080,  0x200, 0x4000,    0x60a0, 0x10, 0x60a1, 0x0200, 0, 0}, /* D1_r */
/*-B-channel-HFC-SPCI-------------32k------------------------------------------*/
[14].h={ 2| 0x2000, 0x1e00, 0x0000, 1| 0x2080, 0x20, 0x2080, 0x2000, 0, 0}, /* B1_t */
[15].h={    0x6000, 0x1e00, 0x4000,    0x6080, 0x20, 0x6081, 0x2000, 0, 0}, /* B1_r */
[16].h={ 2| 0x2100, 0x1e00, 0x2000, 1| 0x2180, 0x20, 0x2180, 0x2000, 0, 0}, /* B2_t */
[17].h={    0x6100, 0x1e00, 0x6000,    0x6180, 0x20, 0x6181, 0x2000, 0, 0}, /* B2_r */
/*BD-channel-HFC-USB---------------4K------------------------------------------*/
[18].h={ 2|   0x04,   0x80,   0x80, 1|   0x0c, 0x08,   0x0e, 0x0080, 0, 0}, /* BD_t */
[19].h={      0x04,   0x80,   0x80,      0x0c, 0x08,   0x0e, 0x0080, 0, 0}, /* BD_r */
/*BD-pipes-HFC-USB-----------------4K------------------------------------------*/
[20].h={ 2|   0x04,   0x08,   0x80, 1|   0x0c, 0x08,   0x0e, 0x0080, 0, 0}, /* BD_t */
[21].h={      0x04,   0x08,   0x80,      0x0c, 0x08,   0x0e, 0x0080, 0, 0}, /* BD_r */
};

#endif /* _I4B_HFC_H_ */
