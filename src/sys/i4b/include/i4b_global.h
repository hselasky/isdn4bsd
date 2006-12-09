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

#ifndef min
#define min MIN
#endif

struct i4b_line_interconnect;
struct fifo_translator;
struct i4b_protocol;
struct buffer;
struct mbuf;

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

#define SECOND		time_second

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
    struct __callout sc_callout;
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
      __callout_init_mtx(&(sc)->sc_callout, (fifo_translator)->mtx, 0);	\
  }									\
  (sc)->sc_fifo_translator = fifo_translator;				\
  (sc)->sc_driver_type = driver_type;					\
  (sc)->sc_driver_unit = driver_unit;					\
  i4b_accounting_timeout(sc);						\
}									\
/**/
#endif

/*---------------------------------------------------------------------------*
 *	
 *---------------------------------------------------------------------------*/
#define struct_ifqueue				\
        struct  mbuf *ifq_head;			\
        struct  mbuf *ifq_tail;			\
        int     ifq_len;			\
        int     ifq_maxlen;			\
        int     ifq_drops;			\
/**/

struct _ifqueue
{
    struct_ifqueue;
};

#define _IF_DRAIN(ifq) do {			\
        struct mbuf *m;				\
        for (;;) {				\
                _IF_DEQUEUE(ifq, m);		\
                if (m == NULL)			\
                        break;			\
                m_freem(m);			\
        }					\
} while (0)

#define IF_QUEUE_GET(what) what
#define _IF_QUEUE_GET(what) what

#ifndef _IF_ENQUEUE_HEAD
#define _IF_ENQUEUE_HEAD _IF_PREPEND
#endif

#ifndef _IF_QEMPTY
#define _IF_QEMPTY(ifq)	((ifq)->ifq_len == 0)
#endif

#ifndef _IF_HANDOFF
#define _IF_HANDOFF(ifq,m,ifp) _if_handoff(ifq,m)
#endif

#ifdef _IF_QFULL
static __inline int
_if_handoff(struct _ifqueue *ifq, struct mbuf *m)
{
        if (_IF_QFULL(ifq)) {
                _IF_DROP(ifq);
                m_freem(m);
                return (0);
        }
        _IF_ENQUEUE(ifq, m);
        return (1);
}
#endif

struct mbuf *i4b_getmbuf( int, int );

/*---------------------------------------------------------------------------*
 *	definition of DTMF detector
 *---------------------------------------------------------------------------*/
#define I4B_DTMF_N_FREQ 18 /* must be even */
#define I4B_DTMF_N_SAMPLES 102
#define I4B_DTMF_N_DIGITS 32 /* units, max */

struct i4b_dtmf_info_rx {
    int32_t w0[I4B_DTMF_N_FREQ];
    int32_t w1[I4B_DTMF_N_FREQ];
    int32_t max_gain;

    u_int8_t count;
    u_int8_t code;
    u_int8_t code_count;
    u_int8_t bsubprot;
    u_int8_t detected_fax_or_modem;
};

struct i4b_dtmf_info_tx {

    u_int16_t freq0[I4B_DTMF_N_DIGITS];
    u_int16_t freq1[I4B_DTMF_N_DIGITS];
    u_int16_t duration[I4B_DTMF_N_DIGITS];

    u_int16_t omega0;
    u_int16_t omega1;

    u_int8_t input_pos;
    u_int8_t output_pos;
    u_int8_t bsubprot;
    u_int8_t unused;
};

/*---------------------------------------------------------------------------*
 *	fifo-translator definition
 *---------------------------------------------------------------------------*/
typedef struct fifo_translator
{
  void  *L5_sc;
  void  *L5_fifo;

  /* called from Layer 1 */
  /* only used when protocol == P_TRANSPARENT_RING */

  void (*L5_TX_INTERRUPT)(struct fifo_translator *, struct buffer *);
# define L5_TX_INTERRUPT(f,output_buf)		\
   ((f)->L5_TX_INTERRUPT)(f,output_buf)

  void (*L5_RX_INTERRUPT)(struct fifo_translator *, struct buffer *);
# define L5_RX_INTERRUPT(f,buf_input)		\
   ((f)->L5_RX_INTERRUPT)(f,buf_input)

  /* NOTE: Do not call if "m == NULL" */
  void (*L5_PUT_MBUF)(struct fifo_translator *, struct mbuf *);
# define L5_PUT_MBUF(f,m)			\
   ((f)->L5_PUT_MBUF)(f,m)

  struct mbuf * 
       (*L5_GET_MBUF)(struct fifo_translator *);
# define L5_GET_MBUF(f)				\
   ((f)->L5_GET_MBUF)(f)

  struct mbuf *
       (*L5_ALLOC_MBUF)(struct fifo_translator *, u_int16_t, u_int16_t);
# define L5_ALLOC_MBUF(f,def_len,tr_len)	\
   ((f)->L5_ALLOC_MBUF(f,def_len,tr_len))

  /* NOTE: Do not call if "ptr == NULL" */
  void (*L5_PUT_DTMF)(struct fifo_translator *, u_int8_t *, u_int16_t len);
# define L5_PUT_DTMF(f,p,l)			\
   ((f)->L5_PUT_DTMF)(f,p,l)

  void  *L1_sc;
  void  *L1_fifo;
  
  /* only call L1_XXX when FIFO is configured! All L1_XXX functions must be
   * able to handle recursation, which means the functions call themselves
   * (with or without different parameters)
   *
   * L1_FIFO_SETUP must call L1_FIFO_START once
   */

  /* called from Layer 5 */
  void (*L1_FIFO_SETUP)(struct fifo_translator *, struct i4b_protocol *);
# define L1_FIFO_SETUP(f,protocol)		\
   ((f)->L1_FIFO_SETUP)(f,protocol)

  void (*L1_FIFO_START)(struct fifo_translator *);
# define L1_FIFO_START(f)			\
   ((f)->L1_FIFO_START)(f)

  void (*L1_FIFO_STAT)(struct fifo_translator *, bchan_statistics_t *bsp);
# define L1_FIFO_STAT(f,bsp)			\
   ((f)->L1_FIFO_STAT)(f,bsp)

  u_int32_t refcount;
  struct mtx *mtx;

  struct _ifqueue tx_queue;
  struct _ifqueue rx_queue;

  struct i4b_dtmf_info_rx dtmf_rx;
  struct i4b_dtmf_info_tx dtmf_tx;

} fifo_translator_t;

#define FT_INVALID ((struct fifo_translator *)1)
#define FT_UNUSED  ((struct fifo_translator *)0)

#define SC_LOCK(f,fifo_translator)		\
  __typeof(fifo_translator) f;			\
						\
  if(0)						\
  {						\
    goto f##reaccess;				\
    goto f##unlock;				\
  }						\
						\
  f##reaccess:					\
						\
  /* was disconnected */			\
  mtx_lock(&i4b_global_lock);			\
						\
  f = (fifo_translator);			\
						\
  if(f) /* connected */				\
  {						\
    enum { _connected_code = 1 };		\
						\
    __typeof(f->refcount)			\
      refcount = f->refcount;			\
						\
    /* change lock */				\
						\
    mtx_unlock(&i4b_global_lock);		\
    mtx_lock(f->mtx);				\
						\
    FIFO_TRANSLATOR_CHECK_CONNECTED(f);		\
  }						\
/**/

#define SC_UNLOCK(f)				\
f##unlock:					\
  mtx_unlock(f ? f->mtx : &i4b_global_lock);	\
/**/

#define FIFO_TRANSLATOR_ACCESS(			\
	f,fifo_translator,connected_code,	\
	not_connected_code)			\
{						\
  __typeof(fifo_translator) f;			\
						\
  if(0)						\
  {						\
    goto f##reaccess;				\
    goto f##recheck;				\
    goto f##unlock;				\
  }						\
						\
  f##reaccess:					\
    mtx_lock(&i4b_global_lock);			\
  f##recheck:					\
    f = (fifo_translator);			\
						\
  if(f) /* connected */				\
  {						\
    enum { _connected_code = 1 };		\
						\
    __typeof(f->refcount)			\
      refcount = f->refcount;			\
						\
    /* change lock */				\
						\
    mtx_unlock(&i4b_global_lock);		\
    mtx_lock(f->mtx);				\
						\
    FIFO_TRANSLATOR_CHECK_CONNECTED(f);		\
						\
    connected_code;				\
						\
    f##unlock:					\
      mtx_unlock(f->mtx);			\
  }						\
  else						\
  {						\
    enum { _not_connected_code = 1 };		\
						\
    not_connected_code;				\
						\
    mtx_unlock(&i4b_global_lock);		\
  }						\
}						\
/**/

#define FIFO_TRANSLATOR_CHECK_CONNECTED(f)	\
{						\
  if(_connected_code)				\
  {						\
    /* ... having f->mtx locked will		\
     * prevent f->refcount from changing,	\
     * hence f->refcount is only changed	\
     * when both i4b_global_lock and f->mtx	\
     * is locked ...				\
     */						\
    if(f->refcount != refcount)			\
    {						\
      /* was disconnected */			\
      mtx_unlock(f->mtx);			\
      goto f##reaccess;				\
    }						\
  }						\
}						\
/**/

/* NOTE: if ``FIFO_TRANSLATOR_SLEEP()'' fails it
 * will go to neither ``connected-code'' nor 
 * ''not-connected-code'', because the variables
 * are no longer protected by the current lock! -
 * it will just finish the access
 */
#define FIFO_TRANSLATOR_SLEEP(f,ident,priority,wmesg,timo,error)	\
{									\
  if(_connected_code)							\
  {									\
    error = msleep(ident, f->mtx, priority, wmesg, timo);		\
    if(error) goto f##unlock;						\
    FIFO_TRANSLATOR_CHECK_CONNECTED(f); /* after that ``error''		\
					 * has been checked		\
					 */				\
  }									\
}									\
/**/

#define FIFO_TRANSLATOR_GOTO_CONNECTED_CODE(				\
	f,ident,priority,wmesg,timo,error)				\
{									\
  if(_not_connected_code)						\
  {									\
    (error) = msleep(ident, &i4b_global_lock, priority, wmesg, timo);	\
    if(!(error)) goto f##recheck;					\
  }									\
}									\
/**/

/*---------------------------------------------------------------------------*
 *	definition of source telephone number
 *---------------------------------------------------------------------------*/
struct i4b_src_telno {
	u_int8_t ton;      /* source type of number */
	u_int8_t scr_ind;  /* screening ind for incoming call */
	u_int8_t prs_ind;  /* presentation ind for incoming call */

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

	u_int32_t cr;			/* call reference value	*/

	int	channel_id;		/* channel id value cannot be 
					 * changed when channel is allocated
					 */
	u_int8_t channel_bprot;		/* channel B-protocol, BPROT_XXX */
	u_int8_t channel_bsubprot;	/* channel B-sub-protocol, BSUBPROT_XXX */

	u_int32_t driver_type;		/* driver-type to use for channel */
	u_int32_t driver_unit;		/* driver-unit for above driver-type */

	u_int32_t driver_type_copy;	/* copy of "driver_type" */
	u_int32_t driver_unit_copy;	/* copy of "driver_unit" */

	uint16_t curr_max_packet_size;  /* used by CAPI */
	uint16_t new_max_packet_size;   /* used by CAPI */
	
	cause_t	cause_in;		/* cause value from remote */
	cause_t	cause_out;		/* cause value to remote */

	u_char	call_state;		/* from incoming SETUP */
	
	u_char	dst_telno[TELNO_MAX];	/* destination number (accumulated) */
	u_char *dst_telno_ptr;		/* pointer to end of destination number */
	u_char  dst_telno_part[TELNO_MAX]; /* destination number (last part received) */

	u_char	dst_subaddr[SUBADDR_MAX]; /* destination subaddr */

	struct i4b_src_telno src[2];

	u_char	dst_ton;		/* destination type of number */

	u_char	state;			/* state of call descriptor */
	u_char	status_enquiry_timeout;

	struct fifo_translator * fifo_translator_capi_std; /* CAPI2.0 FIFO-translator */
	struct fifo_translator * fifo_translator_capi_bridge; /* CAPI2.0 FIFO-translator */
	struct fifo_translator * fifo_translator_tone_gen; /* tone gen. FIFO-translator */

	u_int8_t ai_type; /* application interface type */
	void *   ai_ptr; /* application interface private pointer */

	u_int8_t is_sms : 1;		 /* set if message is an SMS */
	u_int8_t aocd_flag : 1;		 /* set if AOCD is used for unitlength calc. */
	u_int8_t channel_allocated : 1;  /* set if a B-channel is allocated */
	u_int8_t dir_incoming : 1;	 /* set if incoming call */
	u_int8_t need_release : 1;	 /* set if needs to send release */
	u_int8_t peer_responded : 1;	 /* set if got a message from the other end */
	u_int8_t want_late_inband : 1;	 /* set if user wants inband information
					  * after the disconnect signal
					  */
	u_int8_t sending_complete : 1;   /* set if sending of telephone number 
					  * is complete 
					  */
	u_int8_t b_link_want_active : 1; /* set if B-channel should be connected */
	u_int8_t call_is_on_hold : 1;    /* set if call descriptor is on hold */
	u_int8_t call_is_retrieving : 1; /* set if retrieve is in progress */
	u_int8_t received_src_telno_1 : 1; /* set if calling party number is received */
	u_int8_t received_src_telno_2 : 1; /* set if second calling party number is received */

	u_int8_t  setup_interleave; /* a counter */

	/* line interconnect fields */

	cdid_t li_cdid; /* cdid of peer */
	cdid_t li_cdid_last; /* cdid of peer */
	struct i4b_line_interconnect *li_data_ptr;

	u_int8_t *tone_gen_ptr;
	u_int8_t  tone_gen_state; /* current state of tone generator */
	u_int16_t tone_gen_pos;   /* current sine table position */

	uint16_t  connect_ind_count; /* number of connect indication
				      * messages sent to userland
				      */

	struct  __callout idle_callout;
	struct  __callout set_state_callout;

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

#define SET_BIT(var,offset,value)			\
{							\
  if(value)						\
    (var)[(offset) / (8*sizeof((var)[0]))] |=		\
	  (1 << ((offset) % (8*sizeof((var)[0]))));	\
  else							\
    (var)[(offset) / (8*sizeof((var)[0]))] &=		\
         ~(1 << ((offset) % (8*sizeof((var)[0]))));	\
}							\
/**/

#define GET_BIT(var,offset)			\
  ((var)[(offset) / (8*sizeof((var)[0]))] & 	\
   (1 << ((offset) % (8*sizeof((var)[0])))))	\
/**/

/*---------------------------------------------------------------------------*
 *	I4B-PCM-cable structure
 *
 * i4b_global_lock held: READ+WRITE
 *---------------------------------------------------------------------------*/
struct i4b_pcm_cable {
    u_int16_t slot_end;
    u_int32_t slot_bitmap[(I4B_PCM_SLOT_MAX + ((4*8)-1))/(4*8)];
};

extern struct i4b_pcm_cable i4b_pcm_cable[I4B_PCM_CABLE_MAX];

/*---------------------------------------------------------------------------*
 *	I4B-line-interconnect structure
 *
 * i4b_global_lock held: READ+WRITE
 * CNTL_LOCK() held    : READ
 *---------------------------------------------------------------------------*/
struct i4b_line_interconnect {
    cdid_t    cdid;
    u_int8_t  pcm_cable;
    u_int8_t  pcm_unused;
    u_int16_t pcm_slot_rx;
    u_int16_t pcm_slot_tx;
};

/*---------------------------------------------------------------------------*
 *	I4B-controller definition
 *---------------------------------------------------------------------------*/
struct lapdstat;
typedef struct i4b_controller {
	u_int8_t  unit;			/* controller unit number    */

	struct mtx  L1_lock_data;
	struct mtx *L1_lock_ptr;

	u_int8_t  dummy_zero_start[0];

	u_int8_t  allocated : 1; /* set if controller is allocated */
	u_int8_t  no_layer1_dialtone : 1; /* set if dialtone is not wanted */
	u_int8_t  attached : 1; /* set if controller is attached */
	u_int8_t  no_power_save : 1; /* set to disable power saving */
	u_int8_t  N_nt_mode : 1;  /* set if NT-mode is used */

	/*  --> Layer 2 */
	/* ============ */

	u_int16_t N_serial_number;
	u_int32_t N_protocol;		/* D-channel protocol        */
	u_int32_t N_driver_type;	/* D-channel driver type     */
#	define    N_driver_unit unit	/* D-channel driver unit     */

	struct fifo_translator *
                  N_fifo_translator;	/* D-channel fifo translator */
	u_int32_t N_cdid_count;
	u_int32_t N_cdid_end;		/* exclusive */

	struct lapdstat
		 *N_lapdstat;

	void	(*N_RETRIEVE_REQUEST)(struct call_desc *);
#	define    N_RETRIEVE_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_RETRIEVE_REQUEST)(cd)

	void	(*N_HOLD_REQUEST)(struct call_desc *);
#	define    N_HOLD_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_HOLD_REQUEST)(cd)
  
	void	(*N_DEFLECT_REQUEST)(struct call_desc *);
#	define    N_DEFLECT_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_DEFLECT_REQUEST)(cd)

	void	(*N_MCID_REQUEST)(struct call_desc *);
#	define    N_MCID_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_MCID_REQUEST)(cd)

	void	(*N_CONNECT_REQUEST)(struct call_desc *);
#	define    N_CONNECT_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_CONNECT_REQUEST)(cd)

	void	(*N_INFORMATION_REQUEST)(struct call_desc *);
#	define    N_INFORMATION_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_INFORMATION_REQUEST)(cd)

	void	(*N_CONNECT_RESPONSE)(struct call_desc *, int, int);
#	define    N_CONNECT_RESPONSE(cd,response,cause)		\
         ((i4b_controller_by_cd(cd))->N_CONNECT_RESPONSE)(cd,response,cause)

	void	(*N_DISCONNECT_REQUEST)(struct call_desc *, int);
#	define	  N_DISCONNECT_REQUEST(cd,cause)	\
         ((i4b_controller_by_cd(cd))->N_DISCONNECT_REQUEST)(cd,cause)

	void	(*N_ALERT_REQUEST)(struct call_desc *, int);
#	define    N_ALERT_REQUEST(cd,flag)		\
	 ((i4b_controller_by_cd(cd))->N_ALERT_REQUEST)(cd,flag)

	void	(*N_PROGRESS_REQUEST)(struct call_desc *);
#	define    N_PROGRESS_REQUEST(cd)		\
         ((i4b_controller_by_cd(cd))->N_PROGRESS_REQUEST)(cd)

	struct call_desc * 
		(*N_ALLOCATE_CD)(struct i4b_controller *, void *pipe, 
				 u_int crval, u_int ui_type, void *ui_ptr); 
#	define    N_ALLOCATE_CD(cntl,pipe,crval,ui_type,ui_ptr) \
	 ((cntl)->N_ALLOCATE_CD)(cntl,pipe,crval,ui_type,ui_ptr)

	void    (*N_FREE_CD)(struct call_desc *);
#	define    N_FREE_CD(cd)		\
         ((i4b_controller_by_cd(cd))->N_FREE_CD)(cd)

	/* utilization of the channels */
	u_int8_t  N_channel_utilization[(MAX_CHANNELS+7) / 8];

#define SET_CHANNEL_UTILIZATION(cntl,channel,value)	\
SET_BIT((cntl)->N_channel_utilization,channel,value)	\
/**/

#define GET_CHANNEL_UTILIZATION(cntl,channel)	\
GET_BIT((cntl)->N_channel_utilization,channel)	\
/**/

	struct call_desc *N_call_desc_start;
	struct call_desc *N_call_desc_end;

#define CD_FOREACH(cd,cntl)			\
  for((cd) = (cntl)->N_call_desc_start;		\
      (cd) < (cntl)->N_call_desc_end;		\
      (cd)++)					\
/**/

	struct i4b_line_interconnect *N_line_interconnect_start;
	struct i4b_line_interconnect *N_line_interconnect_end;

#define LI_FOREACH(li,cntl)			\
  for((li) = (cntl)->N_line_interconnect_start;	\
      (li) < (cntl)->N_line_interconnect_end;	\
      (li)++)					\
/**/

	/*  --> Layer 1 */
	/* ============ */

	void *    L1_sc;		/* layer 1 softc */
	void *    L1_fifo;		/* layer 1 FIFO */

	u_int16_t L1_channel_end;	/* number of channels */
	u_int8_t  L1_type;		/* layer 1 type	      */
	u_int8_t  L1_pcm_cable_end;	/* exclusive */
	u_int8_t  L1_pcm_cable_map[I4B_PCM_CABLE_MAX];

	struct fifo_translator * 
		(*L1_GET_FIFO_TRANSLATOR)(struct i4b_controller *, int channel);
#	define	  L1_GET_FIFO_TRANSLATOR(cntl,channel)	\
         ((cntl)->L1_GET_FIFO_TRANSLATOR)(cntl,channel)

	int 	(*L1_COMMAND_REQ)(struct i4b_controller *, int cmd, void *);
#	define    L1_COMMAND_REQ(cntl,cmd,arg) \
	      i4b_l1_command_req(cntl,cmd,arg)

	u_int8_t  dummy_zero_end[0];

} i4b_controller_t;

extern struct i4b_controller i4b_controller[MAX_CONTROLLERS];

extern struct mtx i4b_global_lock;

extern u_int32_t i4b_open_refcount;

#define CNTL_FIND(unit) (&i4b_controller[((unsigned)(unit)) % MAX_CONTROLLERS])
#define CNTL_LOCK(cntl)        mtx_lock((cntl)->L1_lock_ptr)
#define CNTL_LOCK_ASSERT(cntl) mtx_assert((cntl)->L1_lock_ptr, MA_OWNED)
#define CNTL_UNLOCK(cntl)      mtx_unlock((cntl)->L1_lock_ptr)
#define CNTL_GET_LOCK(cntl)    ((cntl)->L1_lock_ptr)

/*---------------------------------------------------------------------------*
 *	
 *---------------------------------------------------------------------------*/
#define MAX_ERROR 256 /* bytes */

#define IS_ERROR(ptr) (((ptr) != 0) && (*(ptr) != 0))

#define ADD_ERROR(ptr,fmt,args...)			\
{ if((ptr) != 0) {					\
   snprintf(ptr,MAX_ERROR,"%s " fmt, ptr ,## args); } }	\
/**/

/*---------------------------------------------------------------------------*
 *	L1-COMMAND-REQUEST definitions
 *---------------------------------------------------------------------------*/
enum
{
  CMR_DOWNLOAD,
  CMR_DIAGNOSTICS,
  CMR_SETTRACE,	/* set D-channel and B-channel trace */
  CMR_SET_L1_AUTO_ACTIVATE_VARIABLE,
  CMR_SET_L1_ACTIVITY_VARIABLE,
  CMR_PH_ACTIVATE,
  CMR_PH_DEACTIVATE,
  CMR_SET_POWER_SAVE,
  CMR_SET_POWER_ON,
  CMR_SET_I4B_OPTIONS,

  /* CAPI specific requests */

  CMR_CAPI_GET_MANUFACTURER,
  CMR_CAPI_GET_VERSION,
  CMR_CAPI_GET_SERIAL,
  CMR_CAPI_GET_PROFILE,

  /* other */
  CMR_INFO_REQUEST,
  CMR_SET_LAYER1_PROTOCOL,
  CMR_RESET,
  CMR_GET_CHIPSTAT,
  CMR_CLR_CHIPSTAT,
  CMR_SET_LAYER2_PROTOCOL,

  /* Primary Rate */
  CMR_DECODE_CHANNEL,
  CMR_ENCODE_CHANNEL,
  CMR_SET_CHANNEL_MAPPING,

  /* PCM */
  CMR_SET_PCM_MAPPING,

  /* echo cancelling */
  CMR_ENABLE_ECHO_CANCEL,
  CMR_DISABLE_ECHO_CANCEL,

  /* DTMF detection */
  CMR_ENABLE_DTMF_DETECT,
  CMR_DISABLE_DTMF_DETECT,
};

typedef u_int32_t L1_auto_activate_t;
typedef u_int8_t L1_activity_t;

/* prototypes from i4b_l1.c */

extern struct i4b_controller * i4b_controller_allocate(u_int8_t portable, u_int8_t, u_int8_t, u_int8_t *);
extern int i4b_l1_command_req(struct i4b_controller *cntl, int cmd, void *parm);
extern int i4b_l1_set_options(struct i4b_controller *cntl, u_int32_t mask, u_int32_t value);
extern int i4b_l1_bchan_tel_silence(unsigned char *data, int len);
extern int i4b_controller_attach(struct i4b_controller *cntl, u_int8_t *error);
extern void i4b_controller_detach(struct i4b_controller *cntl);
extern void i4b_controller_free(struct i4b_controller *cntl, u_int8_t sub_controllers);

/* prototypes from i4b_convert_xlaw.c */

extern const int16_t i4b_ulaw_to_signed[0x100];
extern const int16_t i4b_alaw_to_signed[0x100];
extern const int16_t i4b_sine_to_signed[8000];

extern const u_int8_t i4b_reverse_bits[0x100];

typedef u_int8_t (i4b_convert_rev_t)(int32_t);

extern i4b_convert_rev_t i4b_signed_to_ulaw;
extern i4b_convert_rev_t i4b_signed_to_alaw;

extern void
i4b_convert_bsubprot(u_int8_t *ptr, u_int32_t len, 
		     int32_t factor, int32_t divisor,
		     u_int8_t in_bsubprot, u_int8_t out_bsubprot);

/* prototypes from i4b_dtmf.c */

extern void
i4b_dtmf_init_rx(struct fifo_translator *ft, u_int8_t bsubprot);

extern void
i4b_dtmf_init_tx(struct fifo_translator *ft, u_int8_t bsubprot);

extern void
i4b_dtmf_queue_digit(struct fifo_translator *ft, u_int8_t digit,
		     u_int16_t tone_duration,
		     u_int16_t gap_duration);
extern void
i4b_dtmf_generate(struct fifo_translator *ft, struct mbuf **pp_m);

extern u_int16_t 
i4b_sqrt_32(u_int32_t a);

extern void
i4b_dtmf_detect(struct fifo_translator *ft, 
		u_int8_t *data_ptr, u_int16_t data_len);

/* prototypes from i4b_echo_cancel.c */

#define I4B_ECHO_CANCEL_SAMPLE_MAX 0x7FC0 /* units */
#define I4B_ECHO_CANCEL_F_SIZE       1024 /* samples */
#define I4B_ECHO_CANCEL_N_TAPS        256 /* samples */
#define I4B_ECHO_CANCEL_K_TAPS         32 /* samples */
#define I4B_ECHO_CANCEL_N_SUB           2 /* units */
#define I4B_ECHO_CANCEL_N_COPY          2 /* units */
#define I4B_ECHO_CANCEL_STEP            1 /* units */
#define I4B_ECHO_CANCEL_X_DP (1<<15)
#define I4B_ECHO_CANCEL_W_DP (I4B_ECHO_CANCEL_N_TAPS * \
			      I4B_ECHO_CANCEL_STEP * \
			      I4B_ECHO_CANCEL_X_DP * 64)
#define I4B_ECHO_CANCEL_ADAPT_COUNT     8000  /* samples */
#define I4B_ECHO_CANCEL_ADAPT_HIST         2  /* units */

#if (I4B_ECHO_CANCEL_K_TAPS > I4B_ECHO_CANCEL_N_TAPS)
#error "I4B_ECHO_CANCEL_K_TAPS is invalid"
#endif

struct i4b_echo_cancel {

    int32_t buf_W0[I4B_ECHO_CANCEL_N_TAPS]; /* FIR filter weights */
    int32_t buf_W1[I4B_ECHO_CANCEL_N_TAPS]; /* FIR filter weights  */
    int32_t buf_WA[I4B_ECHO_CANCEL_ADAPT_HIST][I4B_ECHO_CANCEL_N_TAPS];

    int32_t buf_PH[I4B_ECHO_CANCEL_N_SUB]; /* sum of power */

    int32_t low_pass_1;
    int32_t low_pass_2;

  u_int32_t noise_rem;
  u_int32_t cur_power_rx0; /* after echo cancelling */

  u_int32_t cur_power_rx1; /* before echo cancelling */
  u_int32_t cur_power_tx;

  u_int16_t offset_x; /* input offset for ring buffer 1 */

  u_int16_t offset_wr; /* input offset for ring buffer 2 */
  u_int16_t offset_rd; /* output offset for ring buffer 2 */

  u_int16_t pre_delay; /* pre delay length in sample units */
  u_int16_t cur_power_count;

  u_int16_t adapt_count;
  u_int16_t stable_count;

  u_int16_t rx_time;
  u_int16_t tx_time;

    int16_t coeffs_last_max_x;

    int16_t buf_XH[I4B_ECHO_CANCEL_N_SUB][2*I4B_ECHO_CANCEL_N_TAPS];
    int16_t buf_X0[3*I4B_ECHO_CANCEL_N_TAPS]; /* TX buffer */
    int16_t buf_E0[2*I4B_ECHO_CANCEL_N_TAPS]; /* error buffer */

  u_int8_t  buffer_y[2*I4B_ECHO_CANCEL_F_SIZE];

  u_int8_t  rx_speaking : 1;
  u_int8_t  tx_speaking : 1;
  u_int8_t  is_ulaw : 1;
  u_int8_t  coeffs_adapt : 1;
  u_int8_t  coeffs_bad : 1;
  u_int8_t  max_trained : 1;
  u_int8_t  last_byte;
  u_int8_t  adapt_index;
  u_int8_t  coeffs_wait;
};

extern void
i4b_echo_cancel_init(struct i4b_echo_cancel *ec, 
		     u_int16_t pre_delay, 
		     u_int8_t sub_bprot);
extern void
i4b_echo_cancel_update_feeder(struct i4b_echo_cancel *ec,
			      u_int16_t tx_time);
extern void
i4b_echo_cancel_feed(struct i4b_echo_cancel *ec, 
		     u_int8_t *ptr, u_int16_t len);
extern void
i4b_echo_cancel_update_merger(struct i4b_echo_cancel *ec,
			      u_int16_t rx_time);
extern void
i4b_echo_cancel_merge(struct i4b_echo_cancel *ec, 
		      u_int8_t *read_ptr, u_int16_t read_len);

/* prototypes from i4b_trace.c */

struct i4b_trace_hdr;
extern void i4b_l1_trace_ind(struct i4b_trace_hdr *hdr, struct mbuf *m);

/* prototypes from i4b_l4.c */

extern int i4b_setup_driver(struct i4b_controller *cntl, u_int32_t channel,
			    struct i4b_protocol *pp, u_int32_t driver_type,
			    u_int32_t driver_unit,struct call_desc *cd);

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

/* these definitions are temporary
 * and will be removed when the
 * new dynamic driver system
 * is in place
 */
#ifndef NI4BTEL
#define NI4BTEL    8
#endif
#ifndef NI4BRBCH
#define NI4BRBCH   8
#endif
#ifndef NI4BISPPP
#define NI4BISPPP  8
#endif
#ifndef NI4BIPR
#define NI4BIPR    8
#endif
#ifndef NI4BING
#define NI4BING    8
#endif

#endif /* _I4B_GLOBAL_H_ */
