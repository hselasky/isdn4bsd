/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2004 Hans Petter Selasky. All rights reserved.
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
 *	dss1_l2.h - ISDN layer 2 (Q.921) definitions
 *	--------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _DSS1_L2_H_
#define _DSS1_L2_H_

#define DSS1_TEI_IS_ODD YES /* compatible with Q.921,
			     * which only allows 128 
			     * different TEI-values
			     */
#define DSS1_CR_IS_INVERTED YES /* compatible with Q.921,
				 * which inverts C/R-bit
				 */

#define DSS1_TEI_MAX	256 /* exclusive */

/* Q.921 system parameters (Q.921 03/93 pp 43) */

#define N200	3		/* max no of retransmissions */
#define N201DEF	260		/* max no of octetts in information field */
#define N202DEF	3		/* max no of TEI ID Request message transmissions */

#define T200DEF	(hz*1)		/* default T200 timer value = 1 second	*/
#define T201DEF	T200DEF		/* default T201 timer value = T200DEF	*/
#define T202DEF (hz*2)		/* default T202 timer value = 2 seconds */
#define T203DEF (hz*10)		/* default T203 timer value = 10 seconds*/

/* modulo 128 operations */

#define M128INC(v)				\
{						\
	(v)++;					\
	(v) %= 128;				\
}						\
/**/
			
#define M128DEC(v)				\
{						\
	(v)--;					\
	(v) %= 128;				\
}						\
/**/

/* C/R-bit:
 *
 * TE must invert input C/R-bit
 * NT must invert output C/R-bit
 */
#define CR_COMMAND  0x00
#define CR_RESPONSE 0x01


/* byte[0] -------------------------------------------------------------- */

#define OFF_SAPI	0	/* SAPI offset, HDLC flag is eaten by L1 */
#define     SAPI_CCP	  0x00	/* SAPI = 0 - call control procedures */
#define     SAPI_X25	  0x10	/* SAPI = 16 - X.25 packet procedures */
#define     SAPI_L2M	  0x3F	/* SAPI = 63 - Layer 2 management procedures */

/* byte[1] -------------------------------------------------------------- */

#define OFF_TEI		1	  /* TEI offset */
#define     TEI_BROADCAST   0xFF  /* broadcast TEI */
#define     TEI_POINT2POINT 0x01  /* fixed TEI used by the 
				   * point to point protocol
				   */

/* byte[2] -------------------------------------------------------------- */

#define OFF_TX_NR	2	/* current transmit sequence byte */
#define        NR_MAX     128	/* exclusive */

#define OFF_CNTL	2	/* 1st byte of control field; P/F-bit == 0 */

#define CNTLS_DESC(enum,value,desc) { enum, desc }
#define CNTLS(m)\
m(CNTL_DISC,	=0x43,"DISC - disconnect")\
m(CNTL_DM,	=0x0f,"DM - disconnected mode")\
m(CNTL_FRMR,	=0x87,"FRMR - frame rejected")\
m(CNTL_REJ,	=0x09,"REJ - I-frame rejected")\
m(CNTL_RNR,	=0x05,"RNR - Receiver Not Ready")\
m(CNTL_RR,	=0x01,"RR - Receiver Ready")\
m(CNTL_SABME,	=0x6f,"SABME - Set Asynchronous Balanced Mode Extended")\
m(CNTL_UA,	=0x63,"UA - Unnumbered Acknowledge")\
m(CNTL_UI,	=0x03,"UI - Unnumbered I-frame")\
m(CNTL_XID,	=0xaf,"XID")\
/**/

MAKE_ENUM(CNTLS);

#define	    CNTL_PF_BIT   0x10	/* Poll/Final bit */

/* byte[3] -------------------------------------------------------------- */

#define OFF_RX_NR      	3	/* last valid receive sequence number, 
				 * N(R), and P/F-bit
				 */
#define OFF_MEI		3	/* 2nd byte of control field */
#define     MEI_TEI_MANAGEMENT 0x0f

/* byte[4] . . byte[7] -------------------------------------------------- */

#define OFF_RI 		4	/* Ri */
#define OFF_RIL		4	/* Ri low byte */
#define OFF_RIH		5	/* Ri high byte */
#define OFF_MT		6	/* Message Type */
#define     MT_ID_REQUEST	 0x01 /* command  */
#define     MT_ID_ASSIGN	 0x02 /* response */
#define     MT_ID_DENY		 0x03 /* response */
#define     MT_ID_CHECK_REQUEST	 0x04 /* command  */
#define     MT_ID_CHECK_RESPONSE 0x05 /* response */
#define     MT_ID_REMOVE	 0x06 /* command  */
#define     MT_ID_VERIFY	 0x07 /* command  */

#define OFF_AI		7	/* Action Indicator  */

/* extract and insert macros for SAPI octett */

#define GET_SAPI(octett)       (((octett) >> 2) & 0x3f)
#define GET_CR_BIT(octett)     (((octett) & 0x02) != 0)
#define GET_EA_BIT(octett)     (((octett) & 0x01) != 0)
#define GET_NR(octett)         (((octett) >> 1) & 0x7f)

#define MAKE_SAPI(sapi,cr)	((((sapi) << 2) & 0xfc) | (((cr) & 0x01) << 1))

#define PUT_TEI(var, tei)	(*(u_int8_t *)&(var)) = (tei)

/* S-frame */
#define S_FRAME_LEN	4	/* length of a S-frame */

/* I-frame */
#define I_HEADER_LEN	4	/* length of I-header preceding L3 frame */

/* U-frames */
#define U_FRAME_LEN	3	/* length of an U-frame, excluding extra data */

/* TEI-frames */
#define TEI_FRAME_LEN	8      	/* minimum length of a TEI-frame */

/* ------------------------------------------------------------------------ */

#define L2_STATE_IS_TEI_ASSIGNED(state) ((state) > ST_L2_SEND_TEI)

#define L2_STATES_TIMEOUT_DELAY(enum,value,timeout_delay,timeout_state) timeout_delay
#define L2_STATES_TIMEOUT_STATE(enum,value,timeout_delay,timeout_state) timeout_state
#define          L2_STATES_DESC(enum,value,timeout_delay,timeout_state) #enum
#define L2_STATES(m)/*							\
m(------------------------,,--------,-------------------------)-*	\
m( state                  ,, timeout, timeout                 ) *	\
m(                        ,, delay  , state                   ) *	\
m(------------------------,,--------,-------------------------)-*/	\
m( ST_L2_PAUSE            ,, 0/*hz*/, ST_L2_PAUSE             )		\
m( ST_L2_SEND_TEI         ,, 1/*hz*/, ST_L2_SEND_TEI          )		\
m( ST_L2_SEND_SABME       ,, 2/*hz*/, ST_L2_SEND_TEI          )		\
m( ST_L2_SEND_UA          ,, 2/*hz*/, ST_L2_SEND_TEI          )		\
m( ST_L2_SINGLE_FRAME     ,, 8/*hz*/, ST_L2_SEND_TEI          )		\
m( ST_L2_MULTI_FRAME      ,,12/*hz*/, ST_L2_SINGLE_FRAME      )		\
/**/

MAKE_ENUM(L2_STATES,
	N_L2_STATES);

/*
 * setting state ST_L2_SEND_SABME means TEI is assigned
 * need SEND_UA and SEND_SABME in separate to avoid SABME feedback
 */

/* ------------------------------------------------------------------------ */

typedef struct DSS1_TCP_pipe {
  u_int8_t	__pn;	/* pipe number Quick Reference */
#define PIPE_NO(pipe) ((pipe)->__pn)

  u_int32_t	refcount;
  u_int8_t	state;  /* current state */

  struct usb_callout get_mbuf_callout;
  struct usb_callout set_state_callout;

  void		*L5_sc;

  u_int8_t	tei;	/* TEI value in use */

#define TEI_IS_AUTOMATIC(pipe) ((pipe)->serial_number >= 0x80)
  u_int16_t	serial_number;

# undef IFQ_MAXLEN
# define IFQ_MAXLEN (N_CALL_DESC * 4)
  /* it must at least be possible to buffer
   * one frame for each call!!
   */
  STRUCT_IFQUEUE;

  /* local window counters */

  u_int8_t	tx_window_size;
  u_int8_t	tx_window_length;

  /* local frame counters */

  u_int8_t	rx_nr;
  u_int8_t	tx_nr;
} DSS1_TCP_pipe_t;

typedef struct {
	u_int32_t	sc_unit;	/* unit number for this entry */

	i4b_controller_t *
			sc_cntl;

	u_int8_t	sc_tei_last;	/* last TEI value assigned to 
					 * a remote Terminal 
					 */
	u_int8_t	sc_received_frame;

	u_int8_t L1_deactivate_count;
	L1_activity_t L1_activity;
	L1_auto_activate_t L1_auto_activate;

#define NT_MODE(sc) ((sc)->sc_nt_mode)
#define TE_MODE(sc) (!NT_MODE(sc))
	u_int8_t	sc_nt_mode;
#define IS_PRIMARY_RATE(sc) ((sc)->sc_primary_rate)
	u_int8_t	sc_primary_rate;
#define IS_POINT_TO_POINT(sc) ((sc)->sc_point_to_point)
	u_int8_t	sc_point_to_point;

	STRUCT_IFQUEUE; /* queue of outgoing frames */

	struct usb_callout ID_REQUEST_callout;
	struct usb_callout L1_activity_callout;

	/* statistics */

#define PIPE_MAX     0x10
	struct DSS1_TCP_pipe sc_pipe[PIPE_MAX];
	/* pipe_adapter is ``sc_pipe[0]'' */

	struct DSS1_TCP_pipe *sc_current_pipe;
	struct mbuf          *sc_current_mbuf;
	u_int8_t	      sc_current_length;
	u_int8_t	      sc_current_tx_nr;

#define PIPE_FOREACH(pipe,pipe_var)		\
  for((pipe) = (pipe_var);			\
      (pipe) < ((pipe_var)+PIPE_MAX);		\
      (pipe)++)					\
/**/

	fifo_translator_t    *sc_fifo_translator;

} l2softc_t;

struct dss1_buffer {
  u_int8_t *start;      /* inclusive */
  u_int16_t offset;     /* offset from start */
  u_int16_t len;        /* length from start */
  u_int8_t  state;      /* used by Facility Decode */
  u_int8_t  op_value;   /* used by Facility Decode */
  int32_t   units;      /* used by Facility Decode */
};

extern u_int8_t  dss1_get_1(struct dss1_buffer *src, u_int16_t offset);
extern u_int8_t  dss1_get_valid(struct dss1_buffer *src, u_int16_t offset);
extern u_int16_t dss1_set_length(struct dss1_buffer *src, u_int16_t new_len);
extern void      dss1_buf_init(struct dss1_buffer *dst, void *start, u_int16_t len);

#endif /* _DSS1_L2_H_ */
