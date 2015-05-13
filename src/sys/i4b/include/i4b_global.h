/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *	i4b_global.h - I4B global kernel include file
 *	---------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_GLOBAL_H_
#define _I4B_GLOBAL_H_

#include <i4b/include/i4b_compat.h>

#include <i4b/include/i4b_controller.h>

#include <i4b/include/i4b_limits.h>

#ifndef min
#define min MIN
#endif

struct i4b_line_interconnect;
struct fifo_translator;
struct i4b_protocol;
struct buffer;
struct mbuf;
struct mtx;
struct sx;

/*---------------------------------------------------------------------------*
 *	local mutex support for single CPU systems
 *
 * NOTE: the current code assumes that interrupts are
 * enabled during interrupt.
 *---------------------------------------------------------------------------*/
#define I4B_DROP_GIANT YES

/*---------------------------------------------------------------------------*
 *	time handling
 *---------------------------------------------------------------------------*/
#include <sys/param.h>

#define SECOND		time_uptime

/*---------------------------------------------------------------------------*
 *	accounting definitions
 *---------------------------------------------------------------------------*/
#define I4B_ACCOUNTING 1 /* enable accounting */

#if I4B_ACCOUNTING

typedef struct bchan_statistics {
	int outbytes;
	int inbytes;
} bchan_statistics_t;

/* connection-lifetime statistics-structure */
struct i4b_accounting {
    u_int           sc_driver_unit;
    u_int           sc_driver_type;
    struct fifo_translator *sc_fifo_translator;
    struct usb_callout sc_callout;
    u_int           sc_iinb;   /* number of ISDN incoming bytes from HSCX */
    u_int           sc_ioutb;  /* number of ISDN outgoing bytes to HSCX */
    u_int           sc_inb;    /* number of incoming bytes after 
                                * decompression
                                */
    u_int           sc_outb;   /* number of outgoing bytes 
                                * before compression 
                                */
    u_int           sc_linb;   /* last number of bytes rx'd */
    u_int           sc_loutb;  /* last number of bytes tx'd */
    u_int           sc_inpkt;  /* incoming packets */
    u_int           sc_outpkt; /* outgoing packets */
    u_int           sc_fn;     /* flag */
};

extern void i4b_accounting_timeout(struct i4b_accounting *sc);

#define I4B_ACCOUNTING_INTERVAL 2 /* seconds */

/* zero is default value for ``struct i4b_accounting''.
 * Counters are reset by ``i4b_accounting_timeout'' at
 * disconnect
 */
#define I4B_ACCOUNTING_INIT(sc) { }
#define I4B_ACCOUNTING_UPDATE(sc,fifo_translator,driver_type,driver_unit) \
{									\
  if(fifo_translator)							\
  {									\
      /* connecting */							\
      usb_callout_init_mtx(&(sc)->sc_callout, (fifo_translator)->mtx, 0);	\
  }									\
  (sc)->sc_fifo_translator = fifo_translator;				\
  (sc)->sc_driver_type = driver_type;					\
  (sc)->sc_driver_unit = driver_unit;					\
  i4b_accounting_timeout(sc);						\
}									\
/**/
#endif

struct mbuf *i4b_getmbuf( int, int );

/*---------------------------------------------------------------------------*
 *	I4B-line-interconnect structure
 *
 * i4b_global_lock held: READ+WRITE
 * CNTL_LOCK() held    : READ
 *---------------------------------------------------------------------------*/
struct i4b_line_interconnect {
	cdid_t	cdid;
	uint8_t pcm_cable;
	uint8_t pcm_unused;
	uint16_t pcm_slot_rx;
	uint16_t pcm_slot_tx;
};

/*---------------------------------------------------------------------------*
 *	definition of source telephone number
 *---------------------------------------------------------------------------*/
struct i4b_src_telno {
	uint8_t ton;      /* source type of number */
	uint8_t scr_ind;  /* screening ind for incoming call */
	uint8_t prs_ind;  /* presentation ind for incoming call */

	u_char	telno[TELNO_MAX];     /* source number */
	u_char	subaddr[SUBADDR_MAX]; /* source subaddr */
};

/*---------------------------------------------------------------------------*
 *	definition of a call and all its parameters
 *---------------------------------------------------------------------------*/
typedef struct call_desc {
	cdid_t	cdid;			/* call descriptor id */

#define i4b_controller_by_cd(cd) ((cd)->p_cntl)

	struct i4b_controller *p_cntl;  /* ISDN controller */

	void *  pipe;			/* ISDN controller pipe */

	uint32_t cr;			/* call reference value	*/

	int	channel_id;		/* channel id value cannot be 
					 * changed when channel is allocated
					 */
	uint8_t channel_bprot;		/* channel B-protocol, BPROT_XXX */
	uint8_t channel_bsubprot;	/* channel B-sub-protocol, BSUBPROT_XXX */

	uint32_t driver_type;		/* driver-type to use for channel */
	uint32_t driver_unit;		/* driver-unit for above driver-type */

	uint32_t driver_type_copy;	/* copy of "driver_type" */
	uint32_t driver_unit_copy;	/* copy of "driver_unit" */

	uint16_t curr_max_packet_size;  /* used by CAPI */
	uint16_t new_max_packet_size;   /* used by CAPI */
	
	cause_t	cause_in;		/* cause value from remote */
	cause_t	cause_out;		/* cause value to remote */

	u_char	call_state;		/* from incoming SETUP */
	
	u_char	dst_telno[TELNO_MAX];	/* destination number (accumulated) */
	u_char *dst_telno_ptr;		/* pointer to end of destination number */
	u_char  dst_telno_part[TELNO_MAX]; /* destination number (last part received) */
	u_char  dst_telno_early[TELNO_MAX]; /* destination number (early part received) */

	u_char	dst_subaddr[SUBADDR_MAX]; /* destination subaddr */

	struct i4b_src_telno src[2];

	u_char	dst_ton;		/* destination type of number */

	u_char	state;			/* state of call descriptor */
	u_char	status_enquiry_timeout;

	struct fifo_translator * fifo_translator_capi_std; /* CAPI2.0 FIFO-translator */
	struct fifo_translator * fifo_translator_capi_bridge; /* CAPI2.0 FIFO-translator */
	struct fifo_translator * fifo_translator_tone_gen; /* tone gen. FIFO-translator */

	uint8_t ai_type; /* application interface type */
	void *   ai_ptr; /* application interface private pointer */

	uint8_t not_end_to_end_digital : 1; /* set if audio is transferred non digitally */
	uint8_t is_sms : 1;		 /* set if message is an SMS */
	uint8_t aocd_flag : 1;		 /* set if AOCD is used for unitlength calc. */
	uint8_t channel_allocated : 1;  /* set if a B-channel is allocated */
	uint8_t dir_incoming : 1;	 /* set if incoming call */
	uint8_t need_release : 1;	 /* set if needs to send release */
	uint8_t peer_responded : 1;	 /* set if got a message from the other end */
	uint8_t want_late_inband : 1;	 /* set if user wants inband information
					  * after the disconnect signal
					  */
	uint8_t sending_complete : 1;   /* set if sending of telephone number 
					  * is complete 
					  */
	uint8_t b_link_want_active : 1; /* set if B-channel should be connected */
	uint8_t call_is_on_hold : 1;    /* set if call descriptor is on hold */
	uint8_t call_is_retrieving : 1; /* set if retrieve is in progress */
	uint8_t received_src_telno_1 : 1; /* set if calling party number is received */
	uint8_t received_src_telno_2 : 1; /* set if second calling party number is received */

	uint8_t  setup_interleave; /* a counter */

	/* line interconnect fields */

	cdid_t li_cdid; /* cdid of peer */
	cdid_t li_cdid_last; /* cdid of peer */
	struct i4b_line_interconnect *li_data_ptr;

	const uint8_t *tone_gen_ptr;
	uint8_t  tone_gen_state; /* current state of tone generator */
	uint16_t tone_gen_pos;   /* current sine table position */

	uint16_t  connect_ind_count; /* number of connect indication
				      * messages sent to userland
				      */

	struct  usb_callout idle_callout;
	struct  usb_callout set_state_callout;

	u_char  idle_state;	/* wait for idle_time begin	*/
#define IST_NOT_STARTED 0	/* shorthold mode disabled 	*/
#define IST_STARTED	1	/* in early-hup window          */
#define IST_CHECK	2	/* in non-early-hup window      */

	time_t	connect_time;		/* time connect was made	*/
	time_t	last_active_time;	/* last time with activity	*/

	msg_shorthold_t shorthold_data;	/* shorthold data to use	*/

	time_t	last_aocd_time;		/* last time AOCD received	*/
	int	units;			/* number of AOCD charging units*/
	int	units_type;		/* units type: AOCD, AOCE	*/
	int	cunits;			/* calculated units		*/

	int	isdntxdelay;		/* isdn tx delay after connect	*/

	u_char	display[DISPLAY_MAX];	/* display information element	*/
	u_char	idate_time_data[8];
	u_char  idate_time_len;
	u_char  odate_time_data[8];
	u_char  odate_time_len;
	u_char	keypad[KEYPAD_MAX];	/* keypad facility		*/
	u_char  user_user[USER_USER_MAX]; /* user-user information element */
} call_desc_t;

extern struct mtx i4b_global_lock;
extern struct sx i4b_global_sx_lock;

extern uint32_t i4b_open_refcount;

/*---------------------------------------------------------------------------*
 *	
 *---------------------------------------------------------------------------*/
#define MAX_ERROR 256 /* bytes */

#define IS_ERROR(ptr) (((ptr) != 0) && (*(ptr) != 0))

#define ADD_ERROR(ptr,fmt,args...)			\
{ if((ptr) != 0) {					\
   snprintf(ptr,MAX_ERROR,"%s " fmt, ptr ,## args); } }	\
/**/

/* prototypes from i4b_trace.c */

struct i4b_trace_hdr;
extern void i4b_l1_trace_ind(struct i4b_trace_hdr *hdr, struct mbuf *m);

/* prototypes from i4b_l4.c */

extern int i4b_setup_driver(struct i4b_controller *cntl, uint32_t channel,
			    struct i4b_protocol *pp, uint32_t driver_type,
			    uint32_t driver_unit,struct call_desc *cd);

/* prototypes from i4b_l4mgmt.c */

extern void i4b_update_d_channel(struct i4b_controller *cntl);

#ifndef NOT
#define NOT(arg) _NOT(YES arg(() NO))
#define          _NOT(args...) args
#endif

#ifndef YES
#define YES(args...) args
#define NO(args...)
#endif

#endif /* _I4B_GLOBAL_H_ */
