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

struct fifo_translator;

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
struct mbuf;

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
 *	fifo-translator definition
 *---------------------------------------------------------------------------*/
struct buffer;
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

  /* NOTE: Do not call if m == NULL */
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

  void  *L1_sc;
  void  *L1_fifo;
  
  /* only call L1_XXX when FIFO is configured! All L1_XXX functions must be
   * able to handle recursation, which means the functions call themselves
   * (with or without different parameters)
   *
   * L1_FIFO_SETUP must call L1_FIFO_START once
   */

  /* called from Layer 5 */
  void (*L1_FIFO_SETUP)(struct fifo_translator *, int protocol);
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
 *	definition of a call and all its parameters
 *---------------------------------------------------------------------------*/
typedef struct call_desc {
	cdid_t	cdid;			/* call descriptor id		*/

	void	*pipe;			/* isdn controller pipe		*/

	u_int32_t cr;			/* call reference value		*/

	int	channel_id;		/* channel id value cannot be 
					 * changed when channel is allocated
					 */
	int	channel_bprot;		/* channel B-protocol, BPROT_XXX */

	int	driver_type;		/* driver-type to use for channel */
	int	driver_unit;		/* driver-unit for above driver-type */

	int	driver_type_copy;	/* copy of "driver_type" */
	int	driver_unit_copy;	/* copy of "driver_unit" */
	
	cause_t	cause_in;		/* cause value from remote */
	cause_t	cause_out;		/* cause value to remote */

	u_char	call_state;		/* from incoming SETUP */
	
	u_char	dst_telno[TELNO_MAX];	/* destination number (accumulated) */
	u_char *dst_telno_ptr;		/* pointer to end of destination number */
	u_char  dst_telno_part[TELNO_MAX]; /* destination number (last part received) */

	u_char	dst_subaddr[SUBADDR_MAX]; /* destination subaddr */
	u_char	src_telno[TELNO_MAX];	  /* source number */
	u_char	src_subaddr[SUBADDR_MAX]; /* source subaddr */

	u_char	dst_ton;		/* destination type of number */
	u_char	src_ton;		/* source type of number */

	u_char  scr_ind;		/* screening ind for incoming call */
	u_char	prs_ind;		/* presentation ind for incoming call */

	u_char	state;			/* state of call descriptor */
	u_char	status_enquiry_timeout;

	struct fifo_translator * fifo_translator_capi; /* CAPI2.0 FIFO-translator */
	struct fifo_translator * fifo_translator_tone_gen; /* tone gen. FIFO-translator */

	u_int8_t ai_type; /* application interface type */
	void *   ai_ptr; /* application interface private pointer */

	u_char  is_sms : 1;		/* set if message is an SMS */
	u_char	aocd_flag : 1;		/* AOCD used for unitlength calc */
	u_char	channel_allocated : 1;  /* set if a B-channel is allocated */
	u_char	dir_incoming : 1;	/* outgoing or incoming call */
	u_char	need_release : 1;	/* need release */
	u_char	peer_responded : 1;	/* got a message from the other end */
	u_char  want_late_inband : 1;	/* user wants inband information
					 * after the disconnect signal
					 */
	u_char  sending_complete : 1;   /* set if sending of telephone number 
					 * is complete 
					 */
	u_char b_link_want_active : 1;  /* set if B-channel should be connected */
	u_char call_is_on_hold : 1;     /* set if call descriptor is on hold */
  
	u_int8_t *tone_gen_ptr;
	u_int8_t  tone_gen_state; /* current state of tone generator */
	u_int8_t  setup_interleave;
	u_int16_t tone_gen_pos;   /* current sine table position */

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
	u_char	datetime[DATETIME_MAX];	/* date/time information element*/
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
 *	i4b-controller definition
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

#define N_CALL_DESC MAX_CHANNELS /* per controller */

	struct call_desc N_call_desc[N_CALL_DESC]; /* call descriptor array */

#define CD_FOREACH(cd,cd_var)			\
  for((cd) = (cd_var);				\
      (cd) < ((cd_var)+N_CALL_DESC);		\
      (cd)++)					\
/**/

	/*  --> Layer 1 */
	/* ============ */

	void *    L1_sc;		/* layer 1 softc */
	void *    L1_fifo;		/* layer 1 FIFO */

	u_int16_t L1_channel_end;	/* number of channels */
	u_int8_t  L1_type;		/* layer 1 type	      */

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
  CMR_SET_NT_MODE,
  CMR_SET_TE_MODE,
  CMR_SET_DCH_HI_PRI,
  CMR_SET_DCH_LO_PRI,
  CMR_PH_ACTIVATE,
  CMR_PH_DEACTIVATE,
  CMR_SET_POWER_SAVE,
  CMR_SET_POWER_ON,

  /* CAPI specific requests */

  CMR_CAPI_GET_MANUFACTURER,
  CMR_CAPI_GET_VERSION,
  CMR_CAPI_GET_SERIAL,
  CMR_CAPI_GET_PROFILE,

  /* other */
  CMR_INFO_REQUEST,
  CMR_SET_LAYER1_PROTOCOL,
  CMR_SET_STANDARD_MODE,
  CMR_SET_POLLED_MODE,
  CMR_RESET,
  CMR_GET_CHIPSTAT,
  CMR_CLR_CHIPSTAT,
  CMR_SET_LAYER2_PROTOCOL,

  /* Primary Rate */
  CMR_DECODE_CHANNEL,
  CMR_ENCODE_CHANNEL,
  CMR_SET_CHANNEL_MAPPING,
};

typedef u_int32_t L1_auto_activate_t;
typedef u_int8_t L1_activity_t;

/* prototypes from i4b_l1.c */

extern struct i4b_controller * i4b_controller_by_cd(struct call_desc *cd);
extern struct i4b_controller * i4b_controller_allocate(u_int8_t portable, u_int8_t, u_int8_t *);
extern int i4b_l1_command_req(struct i4b_controller *cntl, int cmd, void *parm);
extern int i4b_l1_bchan_tel_silence(unsigned char *data, int len);
extern int i4b_controller_attach(struct i4b_controller *cntl, u_int8_t *error);
extern void i4b_controller_detach(struct i4b_controller *cntl);
extern void i4b_controller_free(struct i4b_controller *cntl, u_int8_t sub_controllers);

/* prototypes from i4b_trace.c */

struct i4b_trace_hdr;
extern void i4b_l1_trace_ind(struct i4b_trace_hdr *hdr, struct mbuf *m);

/* prototypes from i4b_l4.c */

extern int i4b_setup_driver(struct i4b_controller *cntl,u_int channel,
			    u_int protocol,u_int driver_type,
			    u_int driver_unit,struct call_desc *cd);

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
