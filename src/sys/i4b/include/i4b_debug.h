/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
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
 *	i4b_debug.h - I4B debug header file
 *	-----------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/
#ifndef _I4B_DEBUG_H_
#define _I4B_DEBUG_H_

#ifndef DO_I4B_DEBUG
#define DO_I4B_DEBUG 1	/* default: do debugging */
#endif

#undef  DO_I4B_MAXDEBUG

/* NOTE: the "kernel" and "/usr/src/usr.sbin/i4b" 
 * must be recompiled if debug masks are added 
 * or removed
 *
 * group | meaning
 * ------+--------------------------------
 *     0 | debug messages *
 *     1 | debug messages
 *     2 | debug errors
 *     4 | debug errors *
 *       |
 *
 * *console overflow danger: "syslogd" should not
 *  be running when enabling these
 */
#define I4B_DEBUG_FIELDS(m,n)\
\
    /* Layer 1 */\
\
m(n, L1_ERROR,     2, "general")\
m(n, L1_PRIM,      1, "PH primitive exchange")\
m(n, L1_BCHAN,     1, "B channel action")\
m(n, L1_H_ERR,     2, "HSCX")\
m(n, L1_H_IRQ,     0, "HSCX IRQ")\
m(n, L1_I_ERR,     2, "ISAC")\
m(n, L1_I_MSG,     1, "ISAC")\
m(n, L1_I_SETUP,   1, "ISAC setup")\
m(n, L1_F_MSG,     1, "FSM ")\
m(n, L1_F_ERR,     2, "FSM")\
m(n, L1_T_MSG,     1, "timer")\
m(n, L1_T_ERR,     2, "timer")\
m(n, L1_H_XFRERR,  4, "HSCX data xfer")\
m(n, L1_I_CICO,    1, "ISAC CICO")\
m(n, L1_S_MSG,     1, "soft-HDLC")\
m(n, L1_S_ERR,     4, "soft-HDLC")\
m(n, L1_HFC_DBG,   0, "HFC-S IRQ")\
m(n, L1_EC_MSG,    1, "echo cancel")\
m(n, L1_DTMF_MSG,  1, "DTMF detection")\
m(n, L1_UNKNOWN2,  1, "unknown")\
m(n, L1_UNKNOWN3,  1, "unknown")\
\
    /* Layer 2 */\
\
m(n, L2_ERROR,     2, "general")\
m(n, L2_PRIM,      1, "DL primitive exchange")\
m(n, L2_U_MSG,     1, "U frame")\
m(n, L2_U_ERR,     2, "U frame")\
m(n, L2_S_MSG,     1, "S frame")\
m(n, L2_S_ERR,     2, "S frame")\
m(n, L2_I_MSG,     1, "I frame")\
m(n, L2_I_ERR,     2, "I frame")\
m(n, L2_F_MSG,     1, "FSM")\
m(n, L2_F_ERR,     2, "FSM")\
m(n, L2_T_MSG,     1, "timer")\
m(n, L2_T_ERR,     2, "timer")\
m(n, L2_TEI_MSG,   1, "TEI")\
m(n, L2_TEI_ERR,   2, "TEI")\
m(n, L2_UNKNOWN0,  1, "unknown")\
m(n, L2_UNKNOWN1,  1, "unknown")\
m(n, L2_UNKNOWN2,  1, "unknown")\
m(n, L2_UNKNOWN3,  1, "unknown")\
\
    /* Layer 3 */\
\
m(n, L3_ERR,       2, "general")\
m(n, L3_MSG,       1, "general")\
m(n, L3_F_MSG,     1, "FSM")\
m(n, L3_F_ERR,     2, "FSM")\
m(n, L3_T_MSG,     1, "timer")\
m(n, L3_T_ERR,     2, "timer")\
m(n, L3_P_MSG,     1, "protocol")\
m(n, L3_P_ERR,     2, "protocol")\
m(n, L3_A_MSG,     1, "facility")\
m(n, L3_A_ERR,     2, "facility")\
m(n, L3_PRIM,      1, "Q.931 exchange")\
m(n, L3_UNKNOWN0,  1, "unknown")\
m(n, L3_UNKNOWN1,  1, "unknown")\
m(n, L3_UNKNOWN2,  1, "unknown")\
m(n, L3_UNKNOWN3,  1, "unknown")\
\
    /* Layer 4 */\
\
m(n, L4_ERR,       2, "general")\
m(n, L4_MSG,       1, "general")\
m(n, L4_TIMO,      1, "B-ch timeout")\
m(n, L4_DIALST,    1, "network driver dial state")\
m(n, L4_IPRDBG,    1, "ipr  driver")\
m(n, L4_RBCHDBG,   1, "rbch driver")\
m(n, L4_ISPDBG,    1, "isp  driver")\
m(n, L4_TELDBG,    1, "tel  driver")\
m(n, L4_INGDBG,    1, "ing  driver")\
m(n, L4_IAVCDBG,   1, "iavc driver")\
m(n, L4_CAPIDBG,   1, "capi driver")\
m(n, L4_UNKNOWN0,  1, "unknown")\
m(n, L4_UNKNOWN1,  1, "unknown")\
m(n, L4_UNKNOWN2,  1, "unknown")\
m(n, L4_UNKNOWN3,  1, "unknown")\

/* end of I4B_DEBUG_FIELDS(..) */

#if DO_I4B_DEBUG

#define I4B_DBG(layer, what, fmt, ...)		\
  if(i4b_debug_mask.what)			\
    { printf("i4b-L" #layer " %s: " fmt "\n",	\
	     __FUNCTION__ ,## __VA_ARGS__ ); }

#else

#define I4B_DBG(...) { }

#endif

#define NDBGL1(...) I4B_DBG(1, __VA_ARGS__)
#define NDBGL2(...) I4B_DBG(2, __VA_ARGS__)
#define NDBGL3(...) I4B_DBG(3, __VA_ARGS__)
#define NDBGL4(...) I4B_DBG(4, __VA_ARGS__)

#define I4B_DEBUG_MAKE_TABLE_0(n, what, group, desc)	\
  (((group) & 2) ?					\
   "(" #what ") " desc " errors" :			\
   "(" #what ") " desc " messages"),

#define I4B_DEBUG_MAKE_ENUM(n, what, group, desc) \
  what##_ENUM,

#define I4B_DEBUG_MAKE_CASE_SET(n, what, group, desc) \
  case what##_ENUM: (n)->what = value; break;

#define I4B_DEBUG_MAKE_CASE_GET(n, what, group, desc) \
  case what##_ENUM: if((n)->what) goto one; break;

#define I4B_DEBUG_MAKE_STRUCT(n, what, group, desc) \
  u_int8_t what : 1;

#define I4B_DEBUG_MAKE_MAX(n, what, group, desc) \
  .what = ((group) ? 1 : 0),

#define I4B_DEBUG_MAKE_ERR(n, what, group, desc) \
  .what = (((group) & 2) ? 1 : 0),

#define I4B_DEBUG_MAKE_ALL(n, what, group, desc) \
  .what = 1,

/*---------------------------------------------------------------------------*
 *	debugging mask definition
 *---------------------------------------------------------------------------*/
extern struct i4b_debug_mask {
  I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_STRUCT,)
} i4b_debug_mask;

/*---------------------------------------------------------------------------*
 *	max debugging mask
 *---------------------------------------------------------------------------*/
static const struct i4b_debug_mask i4b_debug_max = {
  I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_MAX,)
};

/*---------------------------------------------------------------------------*
 *	error debugging mask
 *---------------------------------------------------------------------------*/
static const struct i4b_debug_mask i4b_debug_err = {
  I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_ERR,)
};

/*---------------------------------------------------------------------------*
 *	full debugging mask
 *---------------------------------------------------------------------------*/
static const struct i4b_debug_mask i4b_debug_all = {
  I4B_DEBUG_FIELDS(I4B_DEBUG_MAKE_ALL,)
};

/*---------------------------------------------------------------------------*
 *	zero debugging mask
 *---------------------------------------------------------------------------*/
static const struct i4b_debug_mask i4b_debug_zero;

/*---------------------------------------------------------------------------*
 *	Layer1 statistics structure
 *---------------------------------------------------------------------------*/
typedef struct {

	/* transmit */
	u_int xdu; /* and xcol */

	/* receive */
	u_int vfr;
	u_int rdo;
	u_int crc;
	u_int rab;
	u_int rfo;
} chanstat_t;

/*---------------------------------------------------------------------------*
 *	LAPD/Q.921 statistics structure
 *---------------------------------------------------------------------------*/
typedef struct lapdstat {

	/* transmit */

	u_long	tx_i;		/* I, UI */
	u_long  tx_cntl;	/* RR, RNR, REJ, SABME, DISC, DM, UA */
	u_long	tx_tei;		/* TEI	*/

	/* receive */

	u_long	rx_i;		/* I    */
	u_long	rx_rr;		/* RR   */
	u_long	rx_rnr;		/* RNR  */
	u_long	rx_rej;		/* REJ  */
	u_long	rx_sabme;	/* SABME*/
	u_long	rx_tei;		/* TEI	*/
	u_long	rx_ui;		/* UI	*/
	u_long	rx_disc;	/* DISC */
	u_long	rx_xid;		/* XID	*/
	u_long	rx_dm;		/* DM	*/
	u_long	rx_ua;		/* UA	*/
	u_long	rx_frmr;	/* FRMR	*/	
	
	/* errors */

	u_long	err_rx;
} lapdstat_t;

/*---------------------------------------------------------------------------*
 *	I4B debug IOCTL structure
 *---------------------------------------------------------------------------*/
typedef struct {
	u_int32_t unit;
	u_int32_t chan;
	u_int32_t value;
	u_int32_t mask;
	u_char desc[64];

	struct i4b_debug_mask debug;
	lapdstat_t lapdstat;
	chanstat_t chanstat;
} i4b_debug_t;

typedef struct {
	u_int32_t uid;
	u_int32_t gid;
	u_int16_t mode;
	u_int16_t max_units;
	u_int16_t unused0;
	u_int16_t unused1;
} i4b_capi_delegate_t;

#define I4B_CTL_GET_DEBUG           _IOR ('C', 0, i4b_debug_t)
#define I4B_CTL_SET_DEBUG           _IOW ('C', 1, i4b_debug_t)
#define I4B_CTL_GET_CHIPSTAT        _IOWR('C', 2, i4b_debug_t) /* get chipset statistics               */
#define I4B_CTL_CLR_CHIPSTAT        _IOW ('C', 3, i4b_debug_t) /* clear chipset statistics             */
#define I4B_CTL_GET_LAPDSTAT        _IOWR('C', 4, i4b_debug_t) 
#define I4B_CTL_CLR_LAPDSTAT        _IOW ('C', 5, i4b_debug_t) 
#define I4B_CTL_RESET               _IOW ('C', 6, i4b_debug_t) /* chipset reset                        */
#define I4B_CTL_PH_ACTIVATE         _IOW ('C', 9, i4b_debug_t) /* activate PH-line                     */
#define I4B_CTL_PH_DEACTIVATE       _IOW ('C',10, i4b_debug_t) /* deactivate PH-line                   */
#define I4B_CTL_SET_PROTOCOL        _IOW ('C',11, i4b_debug_t) /* set default protocol for a channel   */
#define I4B_CTL_SET_N_SERIAL_NUMBER _IOW ('C',14, i4b_debug_t)
#define I4B_CTL_SET_N_DRIVER_TYPE   _IOW ('C',15, i4b_debug_t)
#define I4B_CTL_SET_POWER_SAVE      _IOW ('C',18, i4b_debug_t)
#define I4B_CTL_SET_POWER_ON        _IOW ('C',19, i4b_debug_t)
#define I4B_CTL_SET_PCM_MAPPING     _IOW ('C',23, i4b_debug_t)
#define I4B_CTL_SET_PCM_SLOT_END    _IOW ('C',24, i4b_debug_t)
#define I4B_CTL_SET_I4B_OPTIONS     _IOWR('C',25, i4b_debug_t)
#define  I4B_OPTION_NT_MODE           0x0001 /* If set, else TE-mode */
#define  I4B_OPTION_T1_MODE           0x0002 /* If set, else E1-mode */
#define  I4B_OPTION_DLOWPRI           0x0004 /* If set, else D-HI-PRI */
#define  I4B_OPTION_PCM_SLAVE         0x0008 /* If set, else PCM master */
#define  I4B_OPTION_PCM_SPEED_32      0x0010
#define  I4B_OPTION_PCM_SPEED_64      0x0020
#define  I4B_OPTION_PCM_SPEED_128     0x0040
#define  I4B_OPTION_POLLED_MODE       0x0080 /* If set, else standard mode */
#define  I4B_OPTION_LOCAL_LOOP        0x0100 /* If set, else disabled */
#define  I4B_OPTION_REMOTE_LOOP       0x0200 /* If set, else disabled */
#define  I4B_OPTION_NO_DIALTONE       0x0400 /* If set, else enabled */
#define  I4B_OPTION_NO_STATUS_ENQUIRY 0x0800 /* If set, else enabled */

/*---------------------------------------------------------------------------*
 *	I4B echo cancel debug IOCTL structure
 *---------------------------------------------------------------------------*/

typedef struct {
	uint32_t unit;
	uint32_t chan;
	uint32_t npoints;
	uint32_t what;
	uint32_t offset;
	 int32_t decimal_point;
	 int32_t ydata[128];
} i4b_ec_debug_t;

#define I4B_CTL_GET_EC_FIR_FILTER   _IOWR('C',26, i4b_ec_debug_t)

#endif /* _I4B_DEBUG_H_ */
