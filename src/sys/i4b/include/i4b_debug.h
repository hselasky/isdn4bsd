/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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

#define DO_I4B_DEBUG 1

#if (defined(DO_I4B_DEBUG) && (DO_I4B_DEBUG != 0))
#undef DO_I4B_DEBUG
#define DO_I4B_DEBUG 1
#else
#undef DO_I4B_DEBUG
#define DO_I4B_DEBUG 0
#endif

#undef DO_I4B_MAXDEBUG		/* default = disable ALL debug messages */

#if DO_I4B_DEBUG

extern unsigned int i4b_l1_debug;
extern unsigned int i4b_l2_debug;
extern unsigned int i4b_l3_debug;
extern unsigned int i4b_l4_debug;

#define _NDBG(layer, bits, fmt, args...)					\
	if(i4b_l##layer##_debug & (bits))					\
	{ printf("i4b-L" #layer " %s: " fmt "\n", __FUNCTION__ ,## args ); }

#else /* !DO_I4B_DEBUG */

#define _NDBG(args...)

#endif

#define NDBGL1(args...) _NDBG(1, args)
#define NDBGL2(args...) _NDBG(2, args)
#define NDBGL3(args...) _NDBG(3, args)
#define NDBGL4(args...) _NDBG(4, args)

#define YES(args...) args
#define NO(...)

/* NOTE: ``kernel'' and ``/usr/src/usr.sbin/i4b'' must be
 * recompiled if debug masks are added or removed
 *
 * group | meaning
 * ------+--------------------------------
 *     0 | debug messages *
 *     1 | debug messages
 *     2 | debug errors
 *     4 | debug errors *
 *       |
 *
 * *console overflow danger: ``syslogd'' should not
 *  be running when enabling these
 */

#define DEBUG_ENUM_1(layer, name, group, desc, first, last)	   \
	_##layer##name first (=0), /* LOG2 of L##layer##_##name */

#define DEBUG_ENUM_2(layer, name, group, desc, first, last)	    \
	L##layer##_##name = 1 << _##layer##name,

#define DEBUG_ENUM_3(layer, name, group, desc, first, last)			\
first (L##layer##_DEBUG_ERR=) (((group) & 2) ? L##layer##_##name:0)| last (0,)

#define DEBUG_ENUM_4(layer, name, group, desc, first, last)			\
first (L##layer##_DEBUG_MAX=) (((group) !=0) ? L##layer##_##name:0)| last (0,)

#ifdef DO_I4B_MAXDEBUG
#define DEBUG_ENUM_5(layer, name, group, desc, first, last)	\
first (L##layer##_DEBUG_DEFAULT=L##layer##_DEBUG_MAX,)
#else
#define DEBUG_ENUM_5(layer, name, group, desc, first, last)	\
first (L##layer##_DEBUG_DEFAULT=L##layer##_DEBUG_ERR,)
#endif

#define I4B_DEBUG_MASKS(m)							\
										\
 /* Layer 1 */									\
 m (1, ERROR,     2, "general",              YES, NO)/*general errors       */	\
 m (1, PRIM,      1, "PH primitive exchange",NO , NO)/*interlayer primitives*/	\
 m (1, BCHAN,     1, "B channel action",     NO , NO)/*B channel action     */	\
 m (1, H_ERR,     2, "HSCX",                 NO , NO)/*HSCX errors          */	\
 m (1, H_IRQ,     0, "HSCX IRQ",             NO , NO)/*HSCX IRQ messages    */	\
 m (1, I_ERR,     2, "ISAC",                 NO , NO)/*ISAC errors          */	\
 m (1, I_MSG,     1, "ISAC",                 NO , NO)/*ISAC messages        */	\
 m (1, I_SETUP,   1, "ISAC setup",           NO , NO)/*ISAC setup messages  */	\
 m (1, F_MSG,     1, "FSM ",                 NO , NO)/*FSM messages         */	\
 m (1, F_ERR,     2, "FSM",                  NO , NO)/*FSM errors           */	\
 m (1, T_MSG,     1, "timer",                NO , NO)/*Timer messages       */	\
 m (1, T_ERR,     2, "timer",                NO , NO)/*Timer errors         */	\
 m (1, H_XFRERR,  4, "HSCX data xfer",       NO , NO)/*HSCX data xfer errors*/	\
 m (1, I_CICO,    1, "ISAC CICO",            NO , NO)/*ISAC command in/out  */	\
 m (1, S_MSG,     1, "soft-HDLC",            NO , NO)/*soft-HDLC messages   */	\
 m (1, S_ERR,     4, "soft-HDLC",            NO , NO)/*soft-HDLC errors     */	\
 m (1, HFC_DBG,   0, "HFC-S IRQ",            NO , NO)/*HFC messages+IRQ     */	\
 m (1, UNKNOWN0,  1, "unknown",              NO , NO)/**/			\
 m (1, UNKNOWN1,  1, "unknown",              NO , NO)/**/			\
 m (1, UNKNOWN2,  1, "unknown",              NO , NO)/**/			\
 m (1, UNKNOWN3,  1, "unknown",              NO ,YES)/**/			\
										\
 /* Layer 2 */									\
 m (2, ERROR,     2, "general",              YES, NO)/*general errors       */	\
 m (2, PRIM,      1, "DL primitive exchange",NO , NO)/*interlayer primitives*/	\
 m (2, U_MSG,     1, "U frame",              NO , NO)/*U frame messages     */	\
 m (2, U_ERR,     2, "U frame",              NO , NO)/*U frame errors       */	\
 m (2, S_MSG,     1, "S frame",              NO , NO)/*S frame messages     */	\
 m (2, S_ERR,     2, "S frame",              NO , NO)/*S frame errors       */	\
 m (2, I_MSG,     1, "I frame",              NO , NO)/*I frame messages     */	\
 m (2, I_ERR,     2, "I frame",              NO , NO)/*I frame errors       */	\
 m (2, F_MSG,     1, "FSM",                  NO , NO)/*FSM messages         */	\
 m (2, F_ERR,     2, "FSM",                  NO , NO)/*FSM errors           */	\
 m (2, T_MSG,     1, "timer",                NO , NO)/*timer messages       */	\
 m (2, T_ERR,     2, "timer",                NO , NO)/*timer errors         */	\
 m (2, TEI_MSG,   1, "TEI",                  NO , NO)/*TEI messages         */	\
 m (2, TEI_ERR,   2, "TEI",                  NO , NO)/*TEI errors           */	\
 m (2, UNKNOWN0,  1, "unknown",              NO , NO)/**/			\
 m (2, UNKNOWN1,  1, "unknown",              NO , NO)/**/			\
 m (2, UNKNOWN2,  1, "unknown",              NO , NO)/**/			\
 m (2, UNKNOWN3,  1, "unknown",              NO ,YES)/**/			\
										\
 /* Layer 3 */									\
 m (3, ERR,       2, "general",              YES, NO)/*general errors       */	\
 m (3, MSG,       1, "general",              NO , NO)/*general message      */	\
 m (3, F_MSG,     1, "FSM",                  NO , NO)/*FSM messages         */	\
 m (3, F_ERR,     2, "FSM",                  NO , NO)/*FSM errors           */	\
 m (3, T_MSG,     1, "timer",                NO , NO)/*timer messages       */	\
 m (3, T_ERR,     2, "timer",                NO , NO)/*timer errors         */	\
 m (3, P_MSG,     1, "protocol",             NO , NO)/*protocol messages    */	\
 m (3, P_ERR,     2, "protocol",             NO , NO)/*protocol errors      */	\
 m (3, A_MSG,     1, "facility",             NO , NO)/*AOC messages         */	\
 m (3, A_ERR,     2, "facility",             NO , NO)/*AOC errors           */	\
 m (3, PRIM,      1, "Q.931 exchange",       NO , NO)/*messages exchanged   */	\
 m (3, UNKNOWN0,  1, "unknown",              NO , NO)/**/			\
 m (3, UNKNOWN1,  1, "unknown",              NO , NO)/**/			\
 m (3, UNKNOWN2,  1, "unknown",              NO , NO)/**/			\
 m (3, UNKNOWN3,  1, "unknown",              NO ,YES)/**/			\
										\
 /* Layer 4 */									\
 m (4, ERR,       2, "general",              YES, NO)/*general errors       */	\
 m (4, MSG,       1, "general",              NO , NO)/*general message      */	\
 m (4, TIMO,      1, "B-ch timeout",         NO , NO)/*B-ch idle timeout    */	\
 m (4, DIALST,    1, "network driver dial state", NO , NO)/*network driver  *	\
							   * dial states    */	\
 m (4, IPRDBG,    1, "ipr  driver",          NO , NO)/*ipr driver messages  */	\
 m (4, RBCHDBG,   1, "rbch driver",          NO , NO)/*rbch driver messages */	\
 m (4, ISPDBG,    1, "isp  driver",          NO , NO)/*isp driver messages  */	\
 m (4, TELDBG,    1, "tel  driver",          NO , NO)/*tel driver messages  */	\
 m (4, INGDBG,    1, "ing  driver",          NO , NO)/*ing driver messages  */	\
 m (4, IAVCDBG,   1, "iavc driver",          NO , NO)/*AVM B1 driver message*/	\
 m (4, CAPIDBG,   1, "capi driver",          NO , NO)/*CAPI driver messages */	\
 m (4, UNKNOWN0,  1, "unknown",              NO , NO)/**/			\
 m (4, UNKNOWN1,  1, "unknown",              NO , NO)/**/			\
 m (4, UNKNOWN2,  1, "unknown",              NO , NO)/**/			\
 m (4, UNKNOWN3,  1, "unknown",              NO ,YES)/**/			\
										\
/* end of I4B_DEBUG_MASKS(..) */

/*
 * generate enums
 */
enum {
  I4B_DEBUG_MASKS(DEBUG_ENUM_1)
  I4B_DEBUG_MASKS(DEBUG_ENUM_2)
  I4B_DEBUG_MASKS(DEBUG_ENUM_3)
  I4B_DEBUG_MASKS(DEBUG_ENUM_4)
  I4B_DEBUG_MASKS(DEBUG_ENUM_5)
};

/*
 * custom debugging (disabled)
 */
#if 0
#define L1_DEBUG_DEFAULT L1_DEBUG_DEFAULT
#define L2_DEBUG_DEFAULT L2_DEBUG_DEFAULT 
#define L3_DEBUG_DEFAULT L3_DEBUG_DEFAULT
#define L4_DEBUG_DEFAULT L4_DEBUG_DEFAULT
#endif

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
	u_int unit;
	u_int chan;
	u_int value;
	u_int debug[5]; /* used to get/set debug bits;
			 * debug[0] is not used
			 */
	u_char desc[64];

	lapdstat_t lapdstat;
	chanstat_t chanstat;
} i4b_debug_t;

#define I4B_CTL_GET_DEBUG           _IOR ('C', 0, i4b_debug_t)
#define I4B_CTL_SET_DEBUG           _IOW ('C', 1, i4b_debug_t)
#define I4B_CTL_GET_CHIPSTAT        _IOWR('C', 2, i4b_debug_t) /* get chipset statistics               */
#define I4B_CTL_CLR_CHIPSTAT        _IOW ('C', 3, i4b_debug_t) /* clear chipset statistics             */
#define I4B_CTL_GET_LAPDSTAT        _IOWR('C', 4, i4b_debug_t) 
#define I4B_CTL_CLR_LAPDSTAT        _IOW ('C', 5, i4b_debug_t) 
#define I4B_CTL_RESET               _IOW ('C', 6, i4b_debug_t) /* chipset reset                        */
#define I4B_CTL_SET_NT_MODE         _IOW ('C', 7, i4b_debug_t) /* set NT-linemode                      */
#define I4B_CTL_SET_TE_MODE         _IOW ('C', 8, i4b_debug_t) /* set TE-linemode (default)            */
#define I4B_CTL_PH_ACTIVATE         _IOW ('C', 9, i4b_debug_t) /* activate PH-line                     */
#define I4B_CTL_PH_DEACTIVATE       _IOW ('C',10, i4b_debug_t) /* deactivate PH-line                   */
#define I4B_CTL_SET_PROTOCOL        _IOW ('C',11, i4b_debug_t) /* set default protocol for a channel   */
#define I4B_CTL_SET_POLLED_MODE     _IOW ('C',12, i4b_debug_t) /* set polled mode                      */
#define I4B_CTL_SET_STANDARD_MODE   _IOW ('C',13, i4b_debug_t) /* set standard mode                    */
#define I4B_CTL_SET_N_SERIAL_NUMBER _IOW ('C',14, i4b_debug_t)
#define I4B_CTL_SET_N_DRIVER_TYPE   _IOW ('C',15, i4b_debug_t)
#define I4B_CTL_SET_HI_PRIORITY     _IOW ('C',16, i4b_debug_t)
#define I4B_CTL_SET_LO_PRIORITY     _IOW ('C',17, i4b_debug_t)
#define I4B_CTL_SET_POWER_SAVE      _IOW ('C',18, i4b_debug_t)
#define I4B_CTL_SET_POWER_ON        _IOW ('C',19, i4b_debug_t)

#endif /* _I4B_DEBUG_H_ */

