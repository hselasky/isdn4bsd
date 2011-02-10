/*-
 * Copyright (c) 2000-2004 Hans Petter Selasky. All rights reserved.
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
 *	i4b_ihfc2.h - common header file
 * 	--------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_IHFC2_H_
#define _I4B_IHFC2_H_

#ifdef __FreeBSD__
#define	IHFC_USB_ENABLED
#endif

#ifdef _KERNEL

# include <sys/types.h>
# include <sys/systm.h>
# include <sys/param.h>
# include <sys/unistd.h>
# include <sys/conf.h>		/* cdevsw stuff */
# include <sys/kernel.h>	/* SYSINIT stuff */
# include <sys/uio.h>		/* SYSINIT stuff */
# include <sys/malloc.h>
# include <sys/mbuf.h>
# include <sys/lock.h>
# include <sys/socket.h>
# include <sys/proc.h>		/* thread + msleep stuff */
# include <machine/bus.h>
# include <sys/callout.h>
# include <sys/bus.h>

# include <net/if.h>

# ifdef IHFC_USB_ENABLED
#  define usb2_config_td_cc ihfc_config_copy
#  define usb2_config_td_softc ihfc_sc
#  include <dev/usb/usb.h>
#  include <dev/usb/usbdi.h>
#  include <dev/usb/usbdi_util.h>
#  include <dev/usb/usb_process.h>
#  include <dev/usb/usb_debug.h>
#  include <dev/usb/usb_util.h>
#  include <dev/usb/usb_busdma.h>

#  include <i4b/layer1/ihfc3/usb2_config_td.h>
# endif

# ifdef __FreeBSD__
#  ifdef DEVFS
#   include <sys/devfsext.h>
#  endif
#  include <sys/filio.h>
# endif

# include <sys/fcntl.h>

# include <i4b/include/i4b_debug.h>
# include <i4b/include/i4b_ioctl.h>
# include <i4b/include/i4b_trace.h>
# include <i4b/include/i4b_global.h>

# include <i4b/layer1/ihfc3/i4b_ihfc2_ioctl.h>
# include <i4b/layer1/ihfc3/i4b_macro.h>

#else

# include </sys/i4b/layer1/ihfc3/i4b_macro.h>

# define bus_space_handle_t	int
# define usb2_xfer_handle	int
# define bus_space_tag_t	int
# define i4b_trace_hdr_t	int
# define callout_handle		{ }
# define usb2_callback		int
# define isdn_link_t		int
# define drvr_link_t		int
# define vm_paddr_t	      u_int
# define usb_xfer		{ }
# define device_t		int
# define resource		{ }
  struct usb_callout              { };
  struct ifqueue		{ };
# define mtx			{ }
# define tv			{ }

#endif

#ifndef __UA_TYPES_H__
#define __UA_TYPES_H__

/* the following structures are 
 * used to force the compiler to
 * generate un-aligned memory 
 * access code on processors that
 * do not support un-aligned
 * memory accesses:
 */

struct void_p {
  void *data;
} __packed;

struct u_int16_p {
  u_int16_t data;
} __packed;

struct u_int32_p {
  u_int32_t data;
} __packed;

struct u_int64_p {
  u_int64_t data;
} __packed;

typedef struct void_p    void_p_t;
typedef struct u_int16_p u_int16_p_t;
typedef struct u_int32_p u_int32_p_t;
typedef struct u_int64_p u_int64_p_t;
#endif

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

#ifndef __used
#define __used __attribute__((__used__))
#endif

static __inline void
memset_1(void *dst, u_int32_t fill, u_int32_t len)
{
    u_int8_t *ptr = (u_int8_t *)dst;
    u_int8_t rem;

    fill &= 0xff;

    while(len--)
    {
        *ptr++ = (u_int8_t)fill;

	if(!((ptr - ((u_int8_t *)0)) & 3))
	{
	    /* alignment is right */

	    rem = len & 3;
	    len /= 4;

	    fill |= fill << 8;
	    fill |= fill << 16;

	    while(len--)
	    {
	        *((u_int32_t *)ptr) = fill;
		ptr += 4;
	    }
	    len = rem;
	}
    }
    return;
}

/*---------------------------------------------------------------------------*
 * : global stuff
 *---------------------------------------------------------------------------*/
#if DO_I4B_DEBUG
#define IHFC_DEBUG(what,fmt,...)		\
  if(i4b_debug_mask.what)			\
    { printf("i4b-L1 %s: %s: "			\
	       fmt, sc->sc_nametmp,		\
	       __FUNCTION__ ,## __VA_ARGS__); }
#else
#define IHFC_DEBUG(...) { }
#endif

#define IHFC_MSG(args...)	IHFC_DEBUG(L1_HFC_DBG,args)
#define IHFC_ERR(args...)	IHFC_DEBUG(L1_ERROR  ,args)
#define HDLC_ERR(args...)	IHFC_DEBUG(L1_S_ERR  ,args)
#define HDLC_MSG(args...)	IHFC_DEBUG(L1_S_MSG  ,args)

#define IHFC_MAX_ERR MAX_ERROR
#define IHFC_ADD_ERR ADD_ERROR
#define IHFC_IS_ERR IS_ERROR

#define FIFO_LOGICAL_NO(f)		((f)->__flogical)
#define FIFO_NO(f)			((f)->__fn)
#define FIFO_DIR(f)			((f)->__fn & (transmit|receive))
#define FIFO_CMP(f,op,no)	       (((f)->__fn) op (no))

#define FIFO_FOREACH(f,sc)				\
for((f) = (sc)->sc_fifo_end ? &(sc)->sc_fifo[0] : NULL;	\
    (f) < (sc)->sc_fifo_end;				\
    (f) ++)

/*---------------------------------------------------------------------------*
 * : program queue management macros
 *
 * NOTE: Double queuing a FIFO will be ignored. The FIFO remains at the
 *       queue where it was first queued. FIFOs on a queue will be sorted
 *	 according to ``FIFO_NO(f)'' where zero is first.
 *
 * NOTE: When the queue is paused, the current fifo will remain first
 *       in the queue (see QDONE == 0)
 *
 * NOTE: Busy and wait conditions are not cleared by ihfc_reset()
 *       (see ``sc->sc_t125_wait'')
 *
 * ... setting a bit is faster than adding something to a queue ...
 *---------------------------------------------------------------------------*/
#define QFIFO(sc,pri,no)				\
{							\
  if((no) < IHFC_CHANNELS) {				\
       (sc)->sc_intr_status[(no) / SC_INTR_BITS] |=	\
	 (1<<((no) % SC_INTR_BITS));			\
  }							\
}

#define QDONE(sc) ((sc)->sc_intr_list_curr <=	\
		   &(sc)->sc_intr_list[0])

/*---------------------------------------------------------------------------*
 *	+----------------------------------------------+
 *	| Some timing values:           i386/i486/i586 |
 *	| ============================================ |
 *	| One io-read              takes about   1.5us |
 *	| 259 io-reads             takes about 388.5us |
 *	+----------------------------------------------+
 *---------------------------------------------------------------------------*/
#if 0
/* TODO: this definition of IO_WAIT
 * should be added to isa/isareg.h
 *
 * TODO: find the file that will define
 * IO_WAIT for both PC98 and i386
 */
#ifndef IO_WAIT
#define IO_WAIT 0xed /* i386 */
#endif

/* 1-2us delay */
#define SHORT_DELAY __asm("outb %%al, %0":: "ri" (IO_WAIT))
#endif

#define cli(sc) (sc)->sc_intr_temp = intr_disable()
#define sti(sc) intr_restore((sc)->sc_intr_temp)

/*---------------------------------------------------------------------------*
 *	Ramptables related to S/Q - channel (not used - disabled)
 *
 *	From TE's viewpoint:
 *	Q: commands to NT
 *	S: indications from NT
 *
 *	From NT's viewpoint:
 *	Q: indications from TE
 *	S: commands to TE
 *	
 *---------------------------------------------------------------------------*
 * const struct ihfc_SQtable { u_int8_t *string; }
 *
 *	ihfc_Q_table[16] =
 *{
 *	{ "Loss of Power indication"		},
 *	{ "ST request"				},
 *	{ 0					},
 *	{ "LoopBack request (B1/B2)"		},
 *	{ 0					},
 *	{ 0					},
 *	{ 0					},
 *	{ "LoopBack request (B1)"	       	},
 *	{ 0					},
 *	{ 0					},
 *	{ 0					},
 *	{ "LoopBack request (B2)"	       	},
 *	{ "V-DCE slave mode"			},
 *	{ "V-DTE slave mode"			},
 *	{ 0					},
 *	{ "Idle"			       	}
 *},
 *	ihfc_S_table[16] =
 *{
 *	{ "Idle"			       	},
 *	{ "ST Fail"				},
 *	{ "ST Pass"				},
 *	{ "Disruptive Operation Indication"	},
 *	{ "DTSE-OUT"				},
 *	{ "V-DCE master mode"			},
 *	{ "ST Indication"		       	},
 *	{ "DTSE-IN"				},
 *	{ "LoopBack indication (B1/B2)"		},
 *	{ "Loss of Received Signal indication"	},
 *	{ "LoopBack indication (B2)"		},
 *	{ "DTSE-IN and OUT"			},
 *	{ "LoopBack indication (B1)"		},
 *	{ "Loss of power indication"		}
 *};
 *
 *---------------------------------------------------------------------------*
 * MUTEX NOTES:
 *
 * In general: No routines should be called in parallel working on the
 * same data-structure. Especially routines which change ifqueues,
 * datapointers and datalengths.
 *
 * This driver only use one mutex, which is assigned by I4B when the
 * driver attaches to I4B. Currently the driver locks more times than
 * needed, which will be cleaned up later.
 *
 * The caller of routine-names using the word ``callback'' must make
 * sure that IHFC_CRIT_BEG() and IHFC_CRIT_END() is applied before
 * and after that the ``callback'' is called
 *
 * Still some external functions, for example mbuf-routines ??, used
 * by this driver need Giant, but it can run without Giant if those
 * functions are not called
 */

#define IHFC_LOCK(sc) mtx_lock((sc)->sc_mtx_p)
#define IHFC_UNLOCK(sc) mtx_unlock((sc)->sc_mtx_p)
#define IHFC_ASSERT_LOCKED(sc) mtx_assert((sc)->sc_mtx_p, MA_OWNED)

#define IHFC_WAKEUP(fifo)			\
{						\
  if((fifo)->state &  ST_TSLEEP)		\
  {						\
     (fifo)->state &=~ST_TSLEEP;		\
     wakeup(fifo);				\
  }						\
}						\
/**/

/*
 * NOTE: code assumes ISAC is connected to d1t and d1r
 *       and HSCX is connected to bxt and bxr.
 *
 * The order of the FIFOs is hard-coded and cannot
 * be changed.
 */
enum
{
        d1t             = 0,    /* - D1 (tx) */
        d1r             = 1,    /* - D1 (rx) */
        b1t             = 2,    /* - B1 (tx) */
        b1r             = 3,    /* - B1 (rx) */
        b2t             = 4,    /* - B2 (tx) */
        b2r             = 5,    /* - B2 (rx) */

        transmit        = 0,    /* - transmit dir */
        receive         = 1,    /* - receive dir */
};

#define IHFC_CHANNELS (32*2)

#if (IHFC_CHANNELS & 1)
#error "number of channels is odd"
#endif

#if ((IHFC_CHANNELS/2) > MAX_CHANNELS)
#error "too many channels, please update I4B"
#endif

/* only for 2B+1D - channel devices: */
#define GROUP_DCHAN(sc) ((sc)->sc_config.s_fifo_en & (0x10|0x20))
#define GROUP_BCHAN(sc) ((sc)->sc_config.s_fifo_en & (0x01|0x02|0x04|0x08))
#define GROUP_ANY(sc)   ((sc)->sc_config.s_fifo_en & (0x01|0x02|0x04|0x08|0x10|0x20))
#define GROUP_TX(sc)    ((sc)->sc_config.s_fifo_en & (0x01|0x04|0x10))
#define GROUP_RX(sc)    ((sc)->sc_config.s_fifo_en & (0x02|0x08|0x20))

/*---------------------------------------------------------------------------*
 *	Counter checking macros
 *---------------------------------------------------------------------------*
 *
 * INC_COUNTER_TIME_CHECK()
 *
 * This macro will analyse the two counter values passed to
 * it, and write back the correct value.  The counter values
 * should be read from the hardware with interrupts disabled,
 * for best results.
 *
 *
 * 32-bit incremental counter (full range used):
 * =============================================
 *    .
 *
 *  0x000000FE
 *  0x000000FF (8-bit-I/O-BUS-critical-point!)
 *  0x00000100
 *  0x00000101
 *  0x00000102
 *
 *    .
 *
 * Read order for reading a 32-bit incremental counter into
 * temporary variables ``Z_chip2'' and ``Z_chip'':
 *
 *                #-------------------------------#
 *                #   32-bit incremental counter  #
 * ---------------#-------+-------+-------+-------#
 *    variable    #  MSB  |       |       |  LSB  #
 * ---------------#-------+-------+-------+-------#
 *   ``Z_chip2''  #   4   |   3   |   2   |   1** #
 * ---------------#-------+-------+-------+-------#
 *   ``Z_chip''   #   8   |   7   |   6   |   5   #
 * ---------------#-------+-------+-------+-------#
 *
 * **reading this byte is optional
 *
 * The bytes must be read in the order (1),2,3,4,5,6,7 and 8;
 * If the hardware use a different byte order, use one-byte
 * access instead of multi-byte access !!
 *
 * For example i386 processors support 32-bit ISA I/O
 * instructions, but if the hardware only supports 8-bit I/O
 * access, the processor will use four 8-bit I/O accesses to
 * get the 32 bits ! (i386 will read in the order 1,2,3,4..8,
 * but other processors might not!?)
 *
 * Starting at MSB: If the two bytes in a column, for example
 * 4 and 8, are equal, there is nothing to do. If the two
 * bytes differ the bytes read between 4 and 8, that are less
 * significant, must either be cleared or set, depending on
 * the counter type:
 *
 * Incremental: Clear
 * Decremental: Set
 *
 * The correct counter value is now in bytes 5..8
 * (``Z_chip'') and should be copied to bytes 1..4
 * 
 * IMPLEMENTATION NOTES:
 * =====================
 *
 * - the ``INC_COUNTER_TIME_CHECK()'' aligns the counters to a power of two! 
 * - the ``INC_COUNTER_TIME_CHECK()'' does no range checking! If ``Z_chip''
 *   is out of range, then for:
 *
 *	- Incremental type: set ``Z_chip'' to  lowest, inclusive counter value
 *	- Decremental type: set ``Z_chip'' to highest, inclusive counter value
 *
 * EXAMPLE FOR READING AN INCREMENTAL COUNTER:
 * ===========================================
 *
 *      ... read counters from hardware ...
 *      ... assuming bus_space_xxx use i386 read order ...
 *
 *      (   Z_chip2) = bus_space_read_2(t,h,(f->fm.h.Zbase));
 *      (f->Z_chip)  = bus_space_read_2(t,h,(f->fm.h.Zbase));
 *
 *      ... analyse/correct the counters ...
 *
 *      INC_COUNTER_TIME_CHECK(f->Z_chip,    Z_chip2);
 *
 *      ... subtract any offsets ...
 *
 *      (f->Z_chip) -= (mwba_phys_start);
 *	(f->Z_chip) &= (0xffff);
 *
 *      ... range check (min: including; max: excluding) ...
 *
 *	INC_COUNTER_RANGE_CHECK(f->Z_chip,min,max);
 *
 *      ... update ...
 *
 *      Z_chip2 = f->Z_chip;
 *
 */
#define INC_COUNTER_CHECK_Xz_min
#define INC_COUNTER_CHECK_Yz_max
#define INC_COUNTER_CHECK_Xmin
#define INC_COUNTER_CHECK_Ymax
#define INC_COUNTER_CHECK_Z_chip2

#define INC_COUNTER_RANGE_CHECK(Z,min,max)	\
{						\
  /* check variable names */			\
  INC_COUNTER_CHECK_X##min;			\
  INC_COUNTER_CHECK_Y##max;			\
  if(((Z) <  min) ||				\
     ((Z) >= max))				\
  {						\
      (Z) =  min;				\
  }						\
}

#define __INC_COUNTER_TIME_CHECK(Z_chip,Z_chip2,mask)	\
	if((Z_chip2) & mask) { (Z_chip) &= mask; }

#define INC_COUNTER_TIME_CHECK(Z_chip, Z_chip2)			\
{								\
  /* make sure that the last					\
   * variable is Z_chip2:					\
   */								\
  INC_COUNTER_CHECK_##Z_chip2;					\
								\
  /* temporarily store the					\
   * changed bits, if any,					\
   * in ``Z_chip2'':						\
   */								\
  (Z_chip2) ^= (Z_chip);					\
								\
  /* if a byte differs						\
   * clear the byte(s) below,					\
   * starting at MSB:						\
   */								\
  __INC_COUNTER_TIME_CHECK(Z_chip,Z_chip2,0x00000000);		\
  __INC_COUNTER_TIME_CHECK(Z_chip,Z_chip2,0xFF000000);		\
  __INC_COUNTER_TIME_CHECK(Z_chip,Z_chip2,0xFFFF0000);		\
  __INC_COUNTER_TIME_CHECK(Z_chip,Z_chip2,0xFFFFFF00);		\
  __INC_COUNTER_TIME_CHECK(Z_chip,Z_chip2,0xFFFFFFFF); /**/	\
								\
  /* write back the correct value:				\
   */								\
  (Z_chip2) = (Z_chip);						\
}

/*---------------------------------------------------------------------------*
 *	Ring-buffer macros and structures
 *
 * case 1:
 * =======
 *                                     ->+
 * Buf_start   Dat_start   Dat_end     Buf_end
 * +-----------+-----------+-----------+
 * |           |XXXDATAXXXX|           |
 * +-----------+-----------+-----------+
 *
 * case 2:
 * =======
 *                                     ->+
 * Buf_start   Dat_end     Dat_start   Buf_end
 * +-----------+-----------+-----------+
 * |XXXXXXXXXXX|           |XXXDATAXXXX|
 * +-----------+-----------+-----------+
 *
 * The direction of read/write is forward/incremental.
 *
 * NOTE: the maximum number of bytes that can be read
 * from  a  buffer  is   ``Buf_size''   bytes    when
 * ``Dat_start = Buf_start'' and ``Dat_end = Buf_end''.
 * Else ``Buf_size - 1'' bytes !!
 *---------------------------------------------------------------------------*/
struct buffer {
	caddr_t  Buf_start; /* inclusive */
	caddr_t  Buf_end;   /* exclusive */
	caddr_t  Dat_start; /* inclusive */
	caddr_t	 Dat_end;   /* exclusive */
};

#define BUF_RANGE_CHECK YES /* signed ``len'' needed? */

/*
 * BUF_SETUP_READLEN
 * macro is used to setup
 * a ring-buffer for read.
 */

#define BUF_SETUP_READLEN(buf, d_start, d_end)	\
{	  (buf)->Dat_start =			\
	  (buf)->Buf_start = (d_start);		\
	  (buf)->Dat_end   =			\
	  (buf)->Buf_end   = (d_end);		\
}						\
/**/

/*
 * BUF_SETUP_WRITELEN
 * macro is used to setup
 * a ring-buffer for write.
 */

#define BUF_SETUP_WRITELEN(buf, d_start, d_end)	\
{	  (buf)->Dat_start =			\
	  (buf)->Dat_end   =			\
	  (buf)->Buf_start = (d_start);		\
	  (buf)->Buf_end   = (d_end);		\
}						\
/**/

/*
 * BUF_GET_READLEN
 * macro is used to get the
 * current non-wrapping read
 * data length ahead.
 *
 * - read pointer is (buf)->Dat_start
 */

#define BUF_GET_READLEN(buf,plen)		\
do {						\
  if((buf)->Dat_start >  (buf)->Dat_end)	\
  {						\
    if(      ((*plen) = ((buf)->Buf_end -	\
                         (buf)->Dat_start))	\
      BUF_RANGE_CHECK (<=)			\
  NOT(BUF_RANGE_CHECK)(==) 0)			\
    { /* case 1, reset counter */		\
      (buf)->Dat_start = (buf)->Buf_start;	\
    }						\
    else					\
    {						\
      /* case 2 */				\
      break;					\
    }						\
  }						\
  /* case 1 */					\
  (*plen) = ((buf)->Dat_end -			\
	     (buf)->Dat_start);			\
} while(0)					\
/**/

#define BUF_UPD_READPTR(buf,len)		\
	(buf)->Dat_start += len;		\
/**/

#define BUF_GET_WRAPPED_READLEN(buf,plen)	\
do {						\
  (*plen) =					\
    (						\
     ((buf)->Dat_start > (buf)->Dat_end) ?	\
     (((buf)->Buf_end - (buf)->Buf_start) -	\
      ((buf)->Dat_start - (buf)->Dat_end)) :	\
     ((buf)->Dat_end - (buf)->Dat_start)	\
     );						\
} while(0)					\
/**/

/*
 * BUF_GET_WRITELEN
 * macro is used to get the
 * current non-wrapping write
 * data length ahead.
 *
 * - write pointer is (buf)->Dat_end
 */

#define BUF_GET_WRITELEN(buf,plen)		\
do {						\
  if((buf)->Dat_end   >=  (buf)->Dat_start)	\
  {						\
    if((      ((*plen) = ((buf)->Buf_end -	\
                          (buf)->Dat_end))	\
              BUF_RANGE_CHECK (<=)		\
          NOT(BUF_RANGE_CHECK)(==) 0) &&	\
        ((buf)->Dat_start !=			\
         (buf)->Buf_start) )			\
    { /* case 2, reset counter */		\
      (buf)->Dat_end = (buf)->Buf_start;	\
    }						\
    else					\
    {						\
      /* case 1 */				\
      break;					\
    }						\
  }						\
  /* case 2 */					\
  (*plen) = ((buf)->Dat_start -			\
             (buf)->Dat_end) -1;		\
} while(0)					\
/**/

#define BUF_UPD_WRITEPTR(buf,len)		\
	(buf)->Dat_end += len;			\
/**/

#define BUF_GET_APPROXIMATE_WRAPPED_WRITELEN(buf,plen)	\
do {							\
  if((buf)->Dat_start <= (buf)->Dat_end)		\
  {							\
    /* case 1 */					\
    if(((*plen) =					\
        (((buf)->Buf_end - (buf)->Buf_start) -		\
         ((buf)->Dat_end - (buf)->Dat_start))))		\
    {							\
      /* only subtract when possible */			\
      (*plen)--;					\
    }							\
  }							\
  else							\
  {							\
    /* case 2 */					\
    (*plen) = ((buf)->Dat_start - (buf)->Dat_end);	\
    (*plen)--;						\
  }							\
} while(0)						\
/**/

/*
 * BUF_GET_READLEN() must be executed before BUF_GET_READPTR()
 *
 * and
 *
 * BUF_GET_WRITELEN() must be executed before BUF_GET_WRITEPTR(),
 *
 * hence the BUF_GET_XXXLEN() macros may change the read/write
 * pointers!
 */
#define BUF_GET_WRITEPTR(buf) (buf)->Dat_end
#define BUF_GET_READPTR(buf)  (buf)->Dat_start

/*---------------------------------------------------------------------------*
 *	structs related to i4b mbufs  				(all chips)
 *---------------------------------------------------------------------------*/
#define I4B_FREEMBUF(f,mbuf)			\
  m_freem(mbuf)

#define I4B_CLEANIFQ(f,ifqueue)			\
  IF_DRAIN(ifqueue)

/*---------------------------------------------------------------------------*
 *	structs related to ihfc_sc				(all chips)
 *---------------------------------------------------------------------------*/
struct ihfc_sc;
struct sc_fifo;

struct resource_id {
	struct resource *  res; /* resource            */
	u_int32_t          rid; /* resource ID         */
	u_int8_t       options; /* RF_XXX              */
	u_int8_t          type; /* SYS_RES_XXX         */
	u_int8_t        number; /* Eg. io_hdl[number]  */
};

struct resource_tab {
	void * const label_pre;  /* label ptr           */
	void * const label;      /* label ptr           */
	u_int8_t     type;       /* SYS_RES_XXX         */
	u_int8_t     number;     /* Eg. io_hdl[number]  */
  const u_int8_t   * description;
};

#define RESOURCES(m)				\
     m (IRQ   , irq   , 1, YES, NO ) /* irq must be unsetup-first */ \
     m (DRQ   , drq   , 1, NO , NO )		\
     m (IOPORT, ioport, 8, NO , NO )		\
     m (MEMORY, memory, 3, NO , YES)		\
						\
/* end of RESOURCES(..) */

#define RES_MACRO_1(NAME, name, indexes, first, last)	\
	IHFC_N##NAME = indexes NOT(last)(,)

#define RES_MACRO_2(NAME, name, indexes, first, last)	\
	REP(indexes,RES_MACRO_3,NAME)

#define RES_MACRO_3(NAME,no)				\
	unsigned o_RES_##NAME##_##no : 1;

#define RES_MACRO_4(NAME, name, indexes, first, last)	\
      { .label_pre   = &&name##_pre,			\
	.label       = &&name,				\
	.number      = indexes,				\
	.type        = SYS_RES_##NAME,			\
        .description = #NAME }  NOT(last)(,)

#define RES_MACRO_5(NAME, name, indexes, first, last) \
	first(__IHFC_RES = -1,)			      \
						      \
	REP(indexes,RES_MACRO_6,NAME)		      \
						      \
	last(IHFC_NRES)

#define RES_MACRO_6(NAME,no)			\
	IHFC_RES_##NAME##_##no##_OFFSET,

/*
 * generate enums
 */
enum
{
  RESOURCES(RES_MACRO_1), /* IHFC_NXXX */
  RESOURCES(RES_MACRO_5), /* IHFC_RES_XXX_X_OFFSET, IHFC_NRES */
};

/* temporary non-bus_space_alloc resources */
#define IHFC_NMWBA 1
#define IHFC_NUSB 16

struct sc_resources {
	struct resource_id     rid[IHFC_NRES];

	bus_space_tag_t     io_tag[IHFC_NIOPORT];/* I/O tag                     */
	bus_space_handle_t  io_hdl[IHFC_NIOPORT];/* I/O handle                  */
	void             * irq_tmp[IHFC_NIRQ];   /* temporary storage for IRQ   */
	bus_space_tag_t    mem_tag[IHFC_NMEMORY];/* memory mapped I/O tag       */
	bus_space_handle_t mem_hdl[IHFC_NMEMORY];/* memory mapped I/O handle    */
	vm_paddr_t mwba_phys_start[IHFC_NMWBA];  /* pointer to MWBA (phys.addr) */
	u_int8_t      * mwba_start[IHFC_NMWBA];  /* pointer to MWBA (virt.addr) */
	u_int32_t        mwba_size[IHFC_NMWBA];  /* size of MWBA in bytes       */
	u_int16_t              iio[IHFC_NIOPORT];/* internal IO                 */
	u_int16_t             iirq[IHFC_NIRQ];   /* internal IRQ                */

	/*
	 * NOTE: there should be
	 * one o_XXX for each
	 * resource. (see sc_default)
	 *
	 * NOTE: mwba_xxx should
	 * not be counted in
	 * IHFC_NRES.
	 */
#ifdef IHFC_USB_ENABLED
	struct usb_xfer * usb_xfer[IHFC_NUSB];
#endif

	i4b_controller_t * i4b_controller;
};

#define REGISTER_FOREACH(r,reg_list)		\
if(&reg_list[0]) /* check if reg_list present */\
     for((r) = &reg_list[0];			\
	 (r)->offset != 0;			\
	 (r)++)

#define OFF2REG(c, off)	((&(c.dummy))[off])
#define REG2OFF(reg)	((&(((struct sc_config * const)0)->reg)) -  \
			 (&(((struct sc_config * const)0)->dummy)))

struct register_list {
  u_int8_t offset;
  u_int8_t regval;
};

struct internal {
  u_int16_t value, internal;
};

/* definition of a FIFO processing program */

typedef u_int8_t (ihfc_fifo_program_t)(struct ihfc_sc *, struct sc_fifo *);

/* return values */

enum {
  PROGRAM_SLEEP,
  PROGRAM_LOOP,
  PROGRAM_DONE,
};

/*--------------------------------
 * struct sc_default
 *--------------------------------
 *
 * Some o_PTIONS are not switchable
 * during operation
 *
 * Defaults for "struct sc_default"
 * are listed in the file "i4b_default.h"
 */
struct sc_default {
  u_int32_t o_RES_start[0];
  RESOURCES(RES_MACRO_2)	      /* o_RES_XXX (must be first) */

  u_int32_t o_HFC_MWBA           : 1; /* 32kbyte memory aligned to 32K,
				       *  0xff filled, required  
				       */
  u_int32_t o_TIGER_MWBA         : 1; /* 16kbyte memory aligned to 4 bytes,
				       * 0x00 filled, required 
				       */
  u_int32_t o_BUS_TYPE_IOM2      : 1; /* set if ISAC IOM mode2 selected.
				       * Else ISAC IOM mode1 selected.
				       */
  u_int32_t o_8KFIFO             : 1; /* set if HFC 8k FIFO mode selected.
				       * Else HFC 32k FIFO mode selected.
				       */
  u_int32_t o_512KFIFO           : 1; /* set if HFC 512k FIFO mode selected.
				       * Else HFC 128k FIFO mode selected.
				       */
  u_int32_t o_PCI_DEVICE         : 1; /* set if device uses PCI ctrl. */
  u_int32_t o_POST_SETUP         : 1; /* set if post setup is done    */
  u_int32_t o_PORTABLE           : 1; /* set if device is portable    */
  u_int32_t o_EXTERNAL_RAM       : 1; /* set if the chip should use
				       * external RAM
				       */
  u_int32_t o_PRIVATE_FLAG_0     : 1; /* private driver flag */
  u_int32_t o_PRIVATE_FLAG_1     : 1; /* private driver flag */
  u_int32_t o_ISAC_NT            : 1; /* set if ISAC supports NT-mode */
  u_int32_t o_IPAC               : 1; /* set if IPAC is present. 
				       * Else ISAC/HSCX is present.
				       */
  u_int32_t o_TRANSPARENT_BYTE_REPETITION : 1; /* set if bytes are repeated
						* in [extended] transparent
						* mode
						*/
  u_int32_t o_ECHO_CANCEL_ENABLED : 1; /* set if echo canceller is supported */
  u_int32_t o_T125_WAIT          : 1; /* set if waiting for 125us timeout */
  u_int32_t o_BULK_READ_STALL    : 1; /* used by USB */
  u_int32_t o_BULK_WRITE_STALL   : 1; /* used by USB */
  u_int32_t o_INTR_READ_STALL    : 1; /* used by USB */

#define IS_NT_MODE(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_NT_MODE)
#define IS_T1_MODE(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_T1_MODE)
#define IS_DLOWPRI(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_DLOWPRI)
#define IS_PCM_SLAVE(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_PCM_SLAVE)
#define IS_PCM_SPEED_32(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_PCM_SPEED_32)
#define IS_PCM_SPEED_64(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_PCM_SPEED_64)
#define IS_PCM_SPEED_128(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_PCM_SPEED_128)
#define IS_POLLED_MODE(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_POLLED_MODE)
#define IS_LOCAL_LOOP(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_LOCAL_LOOP)
#define IS_REMOTE_LOOP(sc,su) \
  ((sc)->sc_state[su].i4b_option_value & I4B_OPTION_REMOTE_LOOP)

  u_int32_t i4b_option_mask;
  u_int32_t i4b_option_value;

  u_int8_t                      cookie;
  u_int8_t                      stdel_nt;  /* S/T delay for NT-mode */
  u_int8_t                      stdel_te;  /* S/T delay for TE-mode */
  u_int8_t                      double_clock; /* crystal selection */
  u_int8_t                      usb2_length;
  u_int8_t                      usb2_conf_no;
  u_int8_t                      usb2_iface_no;
  u_int8_t                      usb2_alt_iface_no;
  u_int8_t                      io_rid [IHFC_NIOPORT];
  u_int8_t                      mem_rid [IHFC_NMEMORY];

  /* masks can be applied to
   * different variables
   */
  u_int8_t			led_masks[0];

  u_int8_t			led_d1_mask;
  u_int8_t			led_b1_mask;
  u_int8_t			led_b2_mask;
  u_int8_t			led_p1_mask;
  u_int8_t			led_inverse_mask; /* inverse bits */

  u_int8_t			led_time_count;
  u_int8_t			led_time_count_sub;

  u_int8_t		      d_L1_type; 

  u_int8_t		      d_sub_controllers;

  u_int16_t		      d_channels; /* RX + TX */

  u_int32_t		      d_interrupt_delay;

  u_int32_t		      d_temp_size;

  const union fifo_map *      d_fifo_map[IHFC_CHANNELS];

  const struct register_list *d_register_list;

  const struct internal *       list_iio;
  const struct internal *       list_iirq;

  const char *                  desc;

#ifdef IHFC_USB_ENABLED
  const struct usb_config *    usb;
#endif

  const struct fsm_table *    d_fsm_table;

#ifdef __NetBSD__
#define IHFC_LEN_T size_t
#else
#define IHFC_LEN_T u_int16_t
#endif

# define CHIP_IDENTIFY_T(dev)			\
	(device_t dev)

  int (*c_chip_identify) CHIP_IDENTIFY_T( ) ;

# define CHIP_RESET_T(sc,error)			\
	(struct ihfc_sc         *sc,		\
	 register u_int8_t   *error)

  void (*c_chip_reset) CHIP_RESET_T(, ) ;

# define CHIP_STATUS_READ_T(sc)			\
	(struct ihfc_sc         *sc)

  void (*c_chip_status_read) CHIP_STATUS_READ_T( ) ;

# define CHIP_STATUS_CHECK_T(sc)		\
	(struct ihfc_sc         *sc)

  void (*c_chip_status_check) CHIP_STATUS_CHECK_T( ) ;

# define CHIP_UNSELECT_T(sc)			\
	(struct ihfc_sc         *sc)

  void (*c_chip_unselect) CHIP_UNSELECT_T( ) ;

# define CHIP_READ_T(sc,reg,ptr,len)       	\
	(struct ihfc_sc         *sc,		\
	 register IHFC_LEN_T    reg,		\
	 register u_int8_t     *ptr, /* dst */	\
	 register IHFC_LEN_T    len)

  void (*c_chip_read) CHIP_READ_T(,,, ) ;

# define CHIP_WRITE_T(sc,reg,ptr,len)		\
	(struct ihfc_sc         *sc,		\
	 register IHFC_LEN_T    reg,		\
	 register const u_int8_t*ptr, /* src */	\
	 register IHFC_LEN_T    len)

  void (*c_chip_write) CHIP_WRITE_T(,,, ) ;

# define IHFC_CONFIG_WRITE_RELOAD ((ihfc_fifo_t *)0)
# define IHFC_CONFIG_WRITE_UPDATE ((ihfc_fifo_t *)1)

# define CHIP_CONFIG_WRITE_T(sc,f)		\
        (struct ihfc_sc		*sc,		\
	 struct sc_fifo          *f)

  void (*c_chip_config_write) CHIP_CONFIG_WRITE_T(,) ;

# define FSM_READ_T(sc,f,ptr)			\
	(struct ihfc_sc         *sc,		\
	 struct sc_fifo          *f,		\
	 register u_int8_t     *ptr) /* dst */

  void (*c_fsm_read) FSM_READ_T(,, ) ;

# define FSM_WRITE_T(sc,f,ptr)			\
	(struct ihfc_sc         *sc,		\
	 struct sc_fifo          *f,		\
	 register const u_int8_t*ptr) /* src */

  void (*c_fsm_write) FSM_WRITE_T(,, ) ;

# define FIFO_GET_PROGRAM_T(sc,f)		\
	(register struct ihfc_sc*sc,		\
	 register struct sc_fifo *f)

ihfc_fifo_program_t *
       (*c_fifo_get_program) FIFO_GET_PROGRAM_T(, ) ;

# define FIFO_READ_T(sc,f,ptr,len)		\
	(struct ihfc_sc		*sc,		\
	 struct sc_fifo		 *f, /* rxf */	\
	 register u_int8_t     *ptr, /* dst */	\
	 register IHFC_LEN_T    len)

  void (*c_fifo_read) FIFO_READ_T(,,, ) ;

# define FIFO_WRITE_T(sc,f,ptr,len)		\
	(struct ihfc_sc         *sc,		\
	 struct sc_fifo          *f, /* txf */	\
	 register const u_int8_t*ptr,/* src */	\
	 register IHFC_LEN_T     len)

  void (*c_fifo_write) FIFO_WRITE_T(,,, ) ;

# define FIFO_WRITE_FILLER_T(sc,f)		\
	(struct ihfc_sc         *sc,		\
	 struct sc_fifo          *f) /* tx */

  void (*c_fifo_write_filler) FIFO_WRITE_FILLER_T(, ) ;

# define FIFO_GET_MEMORY_T(sc,f,start,end,len)	\
	(struct ihfc_sc         *sc,		\
	 struct sc_fifo          *f,		\
	 u_int8_t           **start,		\
	 u_int8_t             **end,		\
	 u_int16_t             *len) /* rx/tx */

  void (*c_fifo_get_memory) FIFO_GET_MEMORY_T(,,,, ) ;

# define FIFO_SELECT_T(sc,f)			\
	(struct ihfc_sc         *sc,		\
	 struct sc_fifo          *f) /* rx/tx */

  void (*c_fifo_select) FIFO_SELECT_T(, ) ;

# define FIFO_INC_FX_PRE_T(sc,f)	       	\
	(struct ihfc_sc         *sc,		\
	 register struct sc_fifo *f) /* rx/tx */

  void (*c_fifo_inc_fx_pre) FIFO_INC_FX_PRE_T(, ) ;

# define FIFO_INC_FX_T(sc,f)		       	\
	(struct ihfc_sc         *sc,		\
	 register struct sc_fifo  *f) /* rx/tx */

  void (*c_fifo_inc_fx) FIFO_INC_FX_T(, ) ;

# define FIFO_FZ_READ_T(sc,f)			\
	(struct ihfc_sc         *sc,		\
	 register struct sc_fifo *f) /* rx/tx */

  void (*c_fifo_fz_read) FIFO_FZ_READ_T(, ) ;

# define FIFO_FRAME_CHECK_T(sc,f,m)		\
	(struct ihfc_sc *   sc,			\
	 struct sc_fifo *   f,			\
	 struct mbuf *      m)       /* rx */

  u_int8_t (*c_fifo_frame_check) FIFO_FRAME_CHECK_T(,,) ;

# define FIFO_SYNC_T(sc)		       	\
	(struct ihfc_sc		*sc)

# define FIFO_FILTER_T(sc,f)			\
	(register struct ihfc_sc*sc,		\
	 register struct sc_fifo *f)

# define RXTX_INTERRUPT_T(ft,buf)		\
	(struct fifo_translator *ft,		\
	 struct buffer *buf)

} __packed;

/* when making an embedded product, one can set
 * these macros to point to the real functions:
 */
#define FIFO_FRAME_CHECK(sc,a...)        (sc)->sc_default.c_fifo_frame_check(sc,a)

#define FIFO_GET_PROGRAM(sc,a...)        (sc)->sc_default.c_fifo_get_program(sc,a)
#define FIFO_READ_MULTI_1(sc,a...)       (sc)->sc_default.c_fifo_read(sc,a) /* XXX could subtract len from Z_chip */
#define FIFO_WRITE_MULTI_1(sc,a...)      (sc)->sc_default.c_fifo_write(sc,a)
#define FIFO_WRITE_FILLER(sc,a...)       (sc)->sc_default.c_fifo_write_filler(sc,a)
#define FIFO_GET_MEMORY(sc,a...)         (sc)->sc_default.c_fifo_get_memory(sc,a)
#define FIFO_SELECT(sc,a...)             (sc)->sc_default.c_fifo_select(sc,a)
#define FIFO_INC_FX_PRE(sc,a...)         (sc)->sc_default.c_fifo_inc_fx_pre(sc,a)
#define FIFO_INC_FX(sc,a...)             (sc)->sc_default.c_fifo_inc_fx(sc,a)
#define FIFO_FZ_READ(sc,a...)            (sc)->sc_default.c_fifo_fz_read(sc,a)

#define CHIP_RESET(sc,a...)              (sc)->sc_default.c_chip_reset(sc,a)
#define CHIP_STATUS_READ(sc)             (sc)->sc_default.c_chip_status_read(sc)
#define CHIP_STATUS_CHECK(sc)            (sc)->sc_default.c_chip_status_check(sc)
#define CHIP_UNSELECT(sc)                (sc)->sc_default.c_chip_unselect(sc)

#define CHIP_READ_MULTI_1(sc,a...)       (sc)->sc_default.c_chip_read(sc,a)
#define CHIP_READ_1(sc,reg,dst)          (sc)->sc_default.c_chip_read(sc,reg,dst,1)
#define CHIP_SET_REG(sc,reg)             (sc)->sc_default.c_chip_read(sc,reg,0,0)

#define CHIP_WRITE_MULTI_1(sc,a...)      (sc)->sc_default.c_chip_write(sc,a)
#define CHIP_WRITE_1(sc,reg,src)         (sc)->sc_default.c_chip_write(sc,reg,src,1)

#define CHIP_CONFIG_WRITE(sc,a...)       (sc)->sc_default.c_chip_config_write(sc,a)

#define FSM_READ(sc,a...)                (sc)->sc_default.c_fsm_read(sc,a)
#define FSM_WRITE(sc,a...)               (sc)->sc_default.c_fsm_write(sc,a)

/* hfc_fifo_map: order of variables
 * is hard-coded and cannot be changed
 */
struct hfc_fifo_map {
	u_int16_t Zbase;
	u_int16_t Zsize;	/* data counter size */
	u_int16_t Zdata;	/* data register or offset */
	u_int16_t Fbase;
	u_int16_t Fsize;	/* frame counter size */
	u_int16_t Fibase;
	u_int16_t Zend;		/* wrap value, excluding, for Z-counter */

	/* allow a ring-device and
	 * a block-device at the
	 * same time:
	 *
	 * ISAC and HSCX fifo-map *
	 * ISAC only:
	 */
	u_int8_t  remove_stat:1;/* set if RSTA is present in RX fifo
				 * and should be removed.
				 */
	u_int8_t  block_size;	/* largest block that can be transferred.
				 * In receive direction block_size must
				 * be a power of two.
				 */
};

#define FM2OFF(name)		& name
/* #define OFF2FM(off)		 * fifo_map_start[off] * */

union fifo_map {
  struct hfc_fifo_map h,i;
};

struct hdlc {
	u_int8_t  flag;
	u_int16_t blevel;
	u_int16_t crc;
	u_int16_t ib;
	u_int32_t tmp;
};

struct sc_fifo {
        union fifo_map	fm;		/* FIFO register map 	  */
	u_int8_t	s_fifo_sel;    	/* HFC FIFO number(SP/USB)*/
	u_int8_t	s_par_hdlc;	/* HFC FIFO config reg.   */
	u_int8_t	s_con_hdlc;	/* HFC FIFO config reg.   */
	u_int8_t	sub_unit;	/* HFC sub-unit */
	u_int8_t	last_byte;	/* last byte transferred, if set  */

#	define		i_rsta F_chip	/* reuse variables (no conflicts) */
#	define		i_rbcl Z_chip	/* reuse variables (no conflicts) */
	u_int8_t        i_ista;		/* ISAC ISTA */
#	define		I_ISTA_ERR 0x01 /* any error RFO/RDO/XDU/XDO/XCOL */
#	define		I_ISTA_XPR 0x10
#	define		I_ISTA_WIP 0x20 /* Write In Progress (optional) */
#	define		I_ISTA_RPF 0x40
#	define		I_ISTA_RME 0x80 /* RME has priority over RPF */

  /* I_CMDR_XXX are hardcoded and cannot
   * be changed:
   */
	u_int8_t	i_cmdr;		 /* ISAC CMDR (to be written) */
#	define		I_CMDR_XRES 0x01 /* TX reset */
#	define		I_CMDR_XME  0x02 /* TX frame end */
#	define		I_CMDR_XTF  0x08 /* TX data end */
#	define		I_CMDR_RRES 0x40 /* RX reset */
#	define		I_CMDR_RMC  0x80 /* RX complete */

	struct hdlc	hdlc;		/* HDLC emulation	  */

	struct buffer   buf;		/* ring buffer pointers   */
	u_int16_t	buf_size;	/* buffer size (at start) */
	u_int16_t	buf_len;	/* buffer size (now)      */
	u_int8_t      *	buf_ptr;	/* buffer ptr  (now)      */

	u_int16_t	state;
#define ST_OPEN		(1<<0)		/* fifo in use  	  */
#define ST_TSLEEP	(1<<1)		/* fifo has called tsleep */
#define ST_NOBLOCK	(1<<2)

  /* program ST_'s: */
#define ST_FRAME_END 	(1<<3)		/* see ``framing rules'' (i4b_program.h)*/
#define ST_FRAME_ERROR	(1<<4)		/* see ``framing rules'' (i4b_program.h)*/
#define ST_RUNNING 	(1<<5)		/* set when fifo need not be called     */
#define ST_I4B_TRACE 	(1<<6)		/* I4B trace enable                     */
#define ST_FZ_LOADED	(1<<8)		/* set when Z-values have been loaded   */

#define ST_PROGRAM_MASK (ST_FRAME_END|ST_FRAME_ERROR/*|ST_FILLED*/|ST_FZ_LOADED|ST_RUNNING)

	struct i4b_protocol prot_curr;	/* HDLC, trans, fax  ...  */
	struct i4b_protocol prot_last;	/* HDLC, trans, fax  ...  */

	u_int8_t	program_state;
	ihfc_fifo_program_t *program;

        void	      (*filter) FIFO_FILTER_T(,);
					/* data processing filter */

	u_int8_t	default_prot;	/* default protocol for /dev/ihfcX.X */

/* temporarily store FIFO_NO() here, hence
 * calculating FIFO_NO from ``f'' needs
 * division and multiplication, in other
 * words it is slow on most CPU's ...
 */
	u_int8_t	__fn;
	u_int8_t	__flogical;

/*
 * Save some I/O-reads by having counters
 * in software:
 */

/* NOTE: all counters must have same type!! */

	/*
	 * F-counters
	 */
	caddr_t		F_ptr;		/* pointer to F_drvr in MWBA */
	u_int8_t	F_drvr;		/* driver incremented F-counter */
	u_int8_t	F_chip;		/* chip incremented F-counter */
#	define		F_MSB 0x80	/* please update! */

	/*
	 * Z-counters
	 */
	u_int8_t *	Z_ptr;		/* pointer to Z_drvr in MWBA or data */
	u_int16_t	Z_drvr; 	/* driver incremented Z-counter */
	u_int16_t	Z_chip;		/* chip incremented Z-counter */
	u_int16_t	Z_chip3;	/* replacement for a Z-list */
	u_int16_t	Z_min_free;	/* minimum free FIFO space */
	u_int16_t	Z_read_time;    /* time when Z-counters were read */
	u_int16_t	Z_chip_written; /* number of bytes in transmit FIFO */
#	define		Z_MSB 0x8000	/* please update! */

#	define Z_transfer_length Z_chip	/* the Z_chip variable is also used
					 * to store the maximum transfer
					 * length when calling f->filter()
					 * and should be updated !
					 */
	/* /dev/ihfcX.X interface */
	u_int16_t	mbuf_rem_length; /* remaining data length */
	struct mbuf *	mbuf;
	struct mbuf *	mbuf_dev;	/* used by /dev/ihfcX.X */
	struct _ifqueue	ifqueue;	/* used by /dev/ihfcX.X */

	u_int32_t	io_stat;
};

struct regdata {
	u_int8_t unused,dir,reg,data;
};

struct sc_config_buffer { /* used by USB */
	struct buffer  buf;
	struct regdata start[0], data[0x60], end[0];
};

/*
 * NOTE: Some soft-registers are used by more than one chiptype.
 * NOTE: Hence offsets into sc_config are stored in a
 *       8-bit variable, Sizeof(sc_config) must be less
 *	 than 256 bytes.
 */
struct sc_config {
#define REGISTERS(m) /* write only registers */ \
        m(dummy,                ,YES,NO )       \
        m(aux_data_0,           ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( am79c3x              ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(a_lmr1,               ,NO ,NO )       \
        m(a_lmr2,               ,NO ,NO )       \
        m(a_lpr,                ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( hfc (2bds0 ISA)      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(s_cirm,               ,NO ,NO )       \
        m(s_ctmt,               ,NO ,NO )       \
        m(s_int_m1,             ,NO ,NO )       \
        m(s_int_m2,             ,NO ,NO )       \
        m(s_mst_mode,           ,NO ,NO )       \
        m(s_clkdel,             ,NO ,NO )       \
        m(s_sctrl,              ,NO ,NO )       \
        m(s_test,               ,NO ,NO )       \
        m(s_connect,            ,NO ,NO )       \
        m(s_b1_ssl,             ,NO ,NO )       \
        m(s_b2_ssl,             ,NO ,NO )       \
        m(s_a1_ssl,             ,NO ,NO )       \
        m(s_a2_ssl,             ,NO ,NO )       \
        m(s_b1_rsl,             ,NO ,NO )       \
        m(s_b2_rsl,             ,NO ,NO )       \
        m(s_a1_rsl,             ,NO ,NO )       \
        m(s_a2_rsl,             ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( hfc (2bds0 PnP)      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(s_ctmt_pnp,           ,NO ,NO )       \
        m(s_mst_emod,           ,NO ,NO )       \
        m(s_sctrl_e,            ,NO ,NO )       \
        m(s_sctrl_r,            ,NO ,NO )       \
        m(s_trm,                ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( hfc (2bds0 PCI)      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(s_cirm_pci,           ,NO ,NO )       \
        m(s_ctmt_pci,           ,NO ,NO )       \
        m(s_int_m2_pci,         ,NO ,NO )       \
        m(s_fifo_en,            ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( tiger 300            ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(t_dma_oper,           ,NO ,NO )       \
        m(t_prct,               ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( isac                 ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(i_mask,               ,NO ,NO )       \
        m(i_adf2,               ,NO ,NO )       \
        m(i_spcr,               ,NO ,NO )       \
        m(i_sqxr,               ,NO ,NO )       \
        m(i_adf1,               ,NO ,NO )       \
        m(i_stcr,               ,NO ,NO )       \
        m(i_timr,               ,NO ,NO )       \
        m(i_mode,               ,NO ,NO )       \
        m(i_star2,              ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( hscx                 ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(h_mask, 2             ,NO ,NO )       \
        m(h_xad1, 2             ,NO ,NO )       \
        m(h_xad2, 2             ,NO ,NO )       \
        m(h_rah2, 2             ,NO ,NO )       \
        m(h_ccr2, 2             ,NO ,NO )       \
        m(h_xccr, 2             ,NO ,NO )       \
        m(h_rccr, 2             ,NO ,NO )       \
        m(h_tsax, 2             ,NO ,NO )       \
        m(h_tsar, 2             ,NO ,NO )       \
        m(h_timr, 2             ,NO ,NO )       \
        m(h_mode, 2             ,NO ,NO )       \
        m(h_xbch, 2             ,NO ,NO )       \
        m(h_rlcr, 2             ,NO ,NO )       \
        m(h_ccr1, 2             ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( ipac                 ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(p_mask,               ,NO ,NO )       \
        m(p_acfg,               ,NO ,NO )       \
        m(p_pota2,              ,NO ,NO )       \
        m(p_aoe,                ,NO ,NO )       \
        m(p_atx,                ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( psb3186              ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(b_iom_cr,             ,NO ,NO )       \
        m(b_maskd,              ,NO ,NO )       \
        m(b_timr,               ,NO ,NO )       \
        m(b_mask,               ,NO ,NO )       \
        m(b_masktr,             ,NO ,NO )       \
        m(b_msti,               ,NO ,NO )       \
        m(b_sqxr,               ,NO ,NO )       \
        m(b_timr2,              ,NO ,NO )       \
        m(b_tr_cr,              ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( wib (USB)            ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(w_imask,              ,NO ,NO )       \
        m(w_cmdr1,              ,NO ,NO )       \
        m(w_cmdr2,              ,NO ,NO )       \
        m(w_ctl,                ,NO ,NO )       \
        m(w_gcr,                ,NO ,NO )       \
        m(w_mocr,               ,NO ,NO )       \
        m(w_pie,                ,NO ,NO )       \
        m(w_po2,                ,NO ,NO )       \
        m(w_l1b1rs,             ,NO ,NO )       \
        m(w_l1b2rs,             ,NO ,NO )       \
        m(w_usbb1rs,            ,NO ,NO )       \
        m(w_usbb2rs,            ,NO ,NO )       \
        m(w_pcm1rs,             ,NO ,NO )       \
        m(w_pcm2rs,             ,NO ,NO )       \
       /*(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *( wib (PCI)            ,NO ,NO )*      \
        *(                      ,NO ,NO )*      \
        *(                      ,NO ,NO )*/     \
        m(w_imask_pci,          ,NO ,NO )       \
        m(w_d_exim,             ,NO ,NO )       \
        m(w_b1_exim,            ,NO ,NO )       \
        m(w_b2_exim,            ,NO ,NO )       \
        m(w_b1_adm1,            ,NO ,NO )       \
        m(w_b2_adm1,            ,NO ,NO )       \
        m(w_b1_adm2,            ,NO ,NO )       \
        m(w_b2_adm2,            ,NO ,NO )       \
        m(w_d_mode,             ,NO ,NO )       \
        m(w_b1_mode,            ,NO ,NO )       \
        m(w_b2_mode,            ,NO ,NO )       \
        m(w_timr1,              ,NO ,NO )       \
        m(w_timr2,              ,NO ,NO )       \
        m(w_ctl_pci,            ,NO ,NO )       \
        m(w_pctl,               ,NO ,NO )       \
        m(w_gcr_pci,            ,NO ,NO )       \
        m(w_sam,                ,NO ,NO )       \
        m(w_tam,                ,NO ,NO )       \
        m(w_sqx,                ,NO ,YES)

#define REG_MACRO_0(arg,indexes,first,last)				\
    first(u_int8_t) arg IF_NUMBER(indexes)([indexes]) NOT(last)(,)

#define REG_MACRO_1(arg,indexes,first,last)				\
    first({) IF_NUMBER(indexes)(REP(indexes,REG_MACRO_2,arg))		\
	 NOT(IF_NUMBER(indexes))([REG2OFF(arg)] = #arg,) last(})

#define REG_MACRO_2(arg,n)			\
    [REG2OFF(arg[n])] = #arg "[" #n "]",

	/*
	 * registers used
	 * by loadconfig:
	 *
	 */

	/* dummy
	 *
	 * The first entry of sc_config
	 * cannot be a register.   This
	 * variable can be used by low-
	 * priority ``fifo programs''.
	 */
	REGISTERS(REG_MACRO_0);

	/* union
	 * {
	 *   UNUSED_REGISTERS(REG_MACRO_0);
	 * };
	 */
#	define s_sctrl_e_pci s_sctrl_e
#	define s_sctrl_e_usb s_sctrl_e

	/*
	 * registers not used
	 * by loadconfig:
	 *
	 */

	/* hfc (2b)  write only: */
	u_int8_t s_cirm_0;
	u_int8_t s_ctmt_0;

	/* psb3186 write only */
	u_int8_t b_cir0;

	/* hfc (...) read only: */
	u_int8_t s_status;
	u_int8_t s_int_s1;
#       define   s_states w_cir /* not compatible */

	/* isac write only */
	/* NOTE: the highest value ``i_cirq'' can be set to is 0x07,
	 * because this variable is shared between different chips
	 * (ISAC/WINBOND).
	 *
	 * NOTE: if bit 2 is set (D-ch. low pri is true) only odd
	 * commands are allowed. This includes AR10/8 and DRC, but
	 * excludes other commands, that are not used by this
	 * driver. Use high priority if you need to use other C/I
	 * commands than these.
	 */
	u_int8_t i_cirq;

	/* isac read only */
	u_int8_t  i_ista;
	u_int16_t h_ista;

	u_int8_t  i_exir;
	u_int16_t h_exir;

	/* ipac read only */
	u_int8_t  p_ista;

	/* wib read only */
	u_int8_t  w_cir;

	/* generic fifo level/threshold register */
	u_int8_t  fifo_level;
};

struct sc_stack {
	struct timeval tv;
	struct timeval tv2;
};

/*===========================================================================*
 * Currently +70 registers are allocated                                     *
 *                                                                           *
 * RULE OF THUMB: If a register is not backwards compatible it               *
 *                should have its own new name and register index.           *
 *                                                                           *
 *      Abbreviations (group I):                                             *
 *      B_      psb 3xxx registers                                           *
 *      S_      hfc registers                                                *
 *      I_      isac registers                                               *
 *      T_      tiger registers                                              *
 *      H_      hscx registers                                               *
 *      W_      winbond registers                                            *
 *                                                                           *
 *      Abbreviations (group II):                                            *
 *      _isa    registers on isa version of chip                             *
 *      _pci    registers on pci version of chip                             *
 *                                                                           *
 *      Abbreviations (group III):                                           *
 *      Px      PCMx                                                         *
 *      Ax      AUXx                                                         *
 *      Bx      Broadbandx                                                   *
 *      Dx      ???x                                                         *
 *      Mx      Monitorx                                                     *
 *                                                                           *
 *      NOTE: Px and Bx are often associated. Eg. PCM channel 1 is           *
 *            supposed to be used for B1-channel external codecs.            *
 *            B1 can be connected to a different PCM channel                 *
 *            by altering the selected slot.                                 *
 *                                                                           *
 *      Abbreviations (group IV):                                            *
 *      _EN     enable                                                       *
 *      _LO     low                                                          *
 *                                                                           *
 * (Please refer to the manuals for register descriptions)                   *
 * (To do: Add more comments and mask definitions and check for errors)      *
 *---------------------------------------------------------------------------*
 * typedef enum reg {                                                        *
 *---------------+==================================================+--------*
 *               | HFC 2BDS0 (_ISA), HFC 2BDS0 PnP (_PNP) and       |        *
 *               | HFC 2BDS0 PCI (_PCI) registers and corresponding |        *
 *               | masks.                                           |        *
 *               +==================================================+        *
 *                                                                           *
 *---------------+ HFC USB begins here +-------------------------------------*
 *                                                                           *
 *              S_CIRM_USB      = 0x00,                                      *
 *              S_FIFO_USB      = 0x0f,                                      *
 *              S_INC_RES_F_USB = 0x0e,                                      *
 *              S_MST_MODE0_USB = 0x14,                                      *
 *              S_MST_MODE1_USB = 0x15,                                      *
 *              S_SLOT_USB      = 0x10,                                      *
 *              S_SLOT_DATA_USB = 0xd0,                                      *
 *              S_CHIP_ID       = 0x16,                                      *
 *              S_STATUS_USB    = 0x1c,                                      *
 *              S_STATES_USB    = 0x30,                                      *
 *              S_SCTRL_USB     = 0x31,                                      *
 *              S_SCTRL_E_USB   = 0x32,                                      *
 *              S_SCTRL_R_USB   = 0x33,                                      *
 *              S_SQ_REC_USB    = 0x34,                                      *
 *              S_SQ_SEND_USB   = 0x34,                                      *
 *              S_CLKDEL_USB    = 0x37,                                      *
 *              S_CON_HDLC_USB  = 0xfa,                                      *
 *              S_HDLC_PAR_USB  = 0xfb,                                      *
 *                                                                           *
 *---------------+ HFC 2BDS0 PCI begins here +-------------------------------*
 *                                                                           *
 *              S_CI_PCI        = 0x08,                                      *
 *              S_TRxR_PCI      = 0x0c,                                      *
 *              S_MON1_D_PCI    = 0x28,                                      *
 *              S_MON2_D_PCI    = 0x2c,                                      *
 *              S_FIFO_EN_PCI   = 0x44,                                      *
 *              S_TRM_PCI       = 0x48,                                      *
 *              S_B_MODE_PCI    = 0x4c,                                      *
 *              S_CHIP_ID_PCI   = 0x58,                                      *
 *              S_CIRM_PCI      = 0x60,                                      *
 *              S_CTMT_PCI      = 0x64,                                      *
 *              S_INT_M1_PCI    = 0x68,                                      *
 *              S_INT_M2_PCI    = 0x6c,                                      *
 *              S_STATUS_PCI    = 0x70,                                      *
 *              S_INT_S1_PCI    = 0x78,                                      *
 *              S_INT_S2_PCI    = 0x7c,                                      *
 *              S_B1_SSL_PCI    = 0x80,                                      *
 *              S_B2_SSL_PCI    = 0x84,                                      *
 *              S_A1_SSL_PCI    = 0x88,                                      *
 *              S_A2_SSL_PCI    = 0x8c,                                      *
 *              S_B1_RSL_PCI    = 0x90,                                      *
 *              S_B2_RSL_PCI    = 0x94,                                      *
 *              S_A1_RSL_PCI    = 0x98,                                      *
 *              S_A2_RSL_PCI    = 0x9c,                                      *
 *              S_MST_EMOD_PCI  = 0xb4,                                      *
 *              S_MST_MODE_PCI  = 0xb8,                                      *
 *              S_CONNECT_PCI   = 0xbc,                                      *
 *              S_STATES_PCI    = 0xc0,                                      *
 *              S_SCTRL_PCI     = 0xc4,                                      *
 *              S_SCTRL_E_PCI   = 0xc8,                                      *
 *              S_SCTRL_R_PCI   = 0xcc,                                      *
 *              S_SQ_REC_PCI    = 0xd0,                                      *
 *              S_SQ_SEND_PCI   = 0xd0,   = S_SQ_REC_PCI                     *
 *              S_CLKDEL_PCI    = 0xdc,                                      *
 *                                                                           *
 *---------------+ HFC 2BDS0 PnP begins here +-------------------------------*
 *                                                                           *
 *              S_CI_PNP        = 0x02,                                      *
 *              S_TRxR_PNP      = 0x03,                                      *
 *              S_MON1_D_PNP    = 0x0a,                                      *
 *              S_MON2_D_PNP    = 0x0b,                                      *
 *              S_FIFO_SEL_PNP  = 0x10,                                      *
 *              S_TRM_PNP       = 0x12,                                      *
 *              S_B_MODE_PNP    = 0x13,                                      *
 *              S_CHIP_ID_PNP   = 0x16,                                      *
 *              S_CIRM_PNP      = 0x18,                                      *
 *              S_CTMT_PNP      = 0x19,                                      *
 *              S_INT_M1_PNP    = 0x1a,                                      *
 *              S_INT_M2_PNP    = 0x1b,                                      *
 *              S_INT_S1_PNP    = 0x1e,                                      *
 *              S_INT_S2_PNP    = 0x1f,                                      *
 *              S_STATUS_PNP    = 0x1c,                                      *
 *              S_B1_SSL_PNP    = 0x20,                                      *
 *              S_B2_SSL_PNP    = 0x21,                                      *
 *              S_A1_SSL_PNP    = 0x22,                                      *
 *              S_A2_SSL_PNP    = 0x23,                                      *
 *              S_B1_RSL_PNP    = 0x24,                                      *
 *              S_B2_RSL_PNP    = 0x25,                                      *
 *              S_A1_RSL_PNP    = 0x26,                                      *
 *              S_A2_RSL_PNP    = 0x27,                                      *
 *              S_MST_EMOD_PNP  = 0x2d,                                      *
 *              S_MST_MODE_PNP  = 0x2e,                                      *
 *              S_CONNECT_PNP   = 0x2f,                                      *
 *              S_STATES_PNP    = 0x30,                                      *
 *              S_SCTRL_PNP     = 0x31,                                      *
 *              S_SCTRL_E_PNP   = 0x32,                                      *
 *              S_SCTRL_R_PNP   = 0x33,                                      *
 *              S_SQ_REC_PNP    = 0x34,                                      *
 *              S_SQ_SEND_PNP   = 0x34,    = S_SQ_REC_PNP                    *
 *              S_CLKDEL_PNP    = 0x37,                                      *
 *                                                                           *
 *---------------+ HFC 2BDS0 begins here +-----------------------------------*
 *                                                                           *
 *              S_CIRM_ISA      = 0x18,    = S_CIRM_PNP                      *
 *                HFC_8X8_EXT_RAM = 0x10,                                    *
 *                HFC_RESET_ENABLE= 0x08,                                    *
 *              S_CTMT_ISA      = 0x19,                                      *
 *                HFC_TIMER_RESET = 0x80,                                    *
 *                HFC_TIMER_MODE  = 0x20,                                    *
 *                HFC_B2_TRANSP_EN= 0x02,                                    *
 *                HFC_B1_TRANSP_EN= 0x01,                                    *
 *              S_INT_M1_ISA    = 0x1a,    = S_INT_M1_PNP = S_INT_M1_PCI     *
 *                HFC_M1_TIMER    = 0x80,                                    *
 *                HFC_M1_STATE    = 0x40,    (S/T statemachine)              *
 *                HFC_M1_D1_RX    = 0x20,                                    *
 *                HFC_M1_B2_RX    = 0x10,                                    *
 *                HFC_M1_B1_RX    = 0x08,                                    *
 *                HFC_M1_D1_TX    = 0x04,                                    *
 *                HFC_M1_B2_TX    = 0x02,                                    *
 *                HFC_M1_B1_TX    = 0x01,                                    *
 *              S_INT_M2_ISA    = 0x1b,                                      *
 *                HFC_M2_B2_INV   = 0x80,                                    *
 *                HFC_M2_B1_INV   = 0x40,                                    *
 *                HFC_M2_B2_56K   = 0x20,                                    *
 *                HFC_M2_B1_56K   = 0x10,                                    *
 *                HFC_M2_INT_OUT  = 0x08,                                    *
 *                HFC_M2_56K_LSB  = 0x04,                                    *
 *                HFC_M2_INT_BUSY = 0x01,                                    *
 *              S_INT_S1_ISA    = 0x1e,                                      *
 *                HFC_S1_TIMER    = 0x80,                                    *
 *                HFC_S1_STATE    = 0x40,                                    *
 *                HFC_S1_D1_RX    = 0x20,                                    *
 *                HFC_S1_B2_RX    = 0x10,                                    *
 *                HFC_S1_B1_RX    = 0x08,                                    *
 *                HFC_S1_D1_TX    = 0x04,                                    *
 *                HFC_S1_B2_TX    = 0x02,                                    *
 *                HFC_S1_B1_TX    = 0x01,                                    *
 *                 +--------------------------+                              *
 *                 | This register is equ-    |                              *
 *                 | ivalent to S_INT_S1_PNP  |                              *
 *                 | NOTE: All active int-    |                              *
 *                 | errupts are cleared on   |                              *
 *                 | read.                    |                              *
 *                 +--------------------------+                              *
 *              S_STATUS_ISA    = 0x1c,    = S_STATUS_PNP                    *
 *                HFC_STATUS_ANY  = 0x80,                                    *
 *                HFC_STATUS_FRAME= 0x40,                                    *
 *                HFC_STATUS_STATE= 0x20,                                    *
 *                HFC_STATUS_TIMER= 0x10,                                    *
 *                HFC_STATUS_NBUSY= 0x04,                                    *
 *                HFC_STATUS_BUSY = 0x01,                                    *
 *              S_B1_SSL_ISA    = 0x20,    = S_B1_SSL_PNP                    *
 *              S_B2_SSL_ISA    = 0x21,    = S_B2_SSL_PNP                    *
 *              S_A1_SSL_ISA    = 0x22,    = S_A1_SSL_PNP                    *
 *              S_A2_SSL_ISA    = 0x23,    = S_A2_SSL_PNP                    *
 *              S_B1_RSL_ISA    = 0x24,    = S_B1_RSL_PNP                    *
 *              S_B2_RSL_ISA    = 0x25,    = S_B2_RSL_PNP                    *
 *              S_A1_RSL_ISA    = 0x22,    = S_A1_SSL_PNP                    *
 *              S_A2_RSL_ISA    = 0x27,    = S_A2_RSL_PNP                    *
 *              S_MST_MODE_ISA  = 0x2e,    = S_MST_MODE_PNP                  *
 *                HFC_F0_TWO_CLKS = 0x08,    duration of F0-signal(244/488ns)*
 *                HFC_F0_NEGATIVE = 0x04,    polarity of F0-signal           *
 *                HFC_C4_C20_ON_P = 0x02,    polarity of C4- and C20-clock   *
 *              S_CONNECT_ISA   = 0x2f,                                      *
 *                HFC_P2_CON_OUT  = 0x20,                                    *
 *                HFC_B2_CON_PCM  = 0x18,                                    *
 *                HFC_P1_CON_OUT  = 0x04,                                    *
 *                HFC_B1_CON_PCM  = 0x03,                                    *
 *                 +----------------------+                                  *
 *                 | NOTE: When these val-|                                  *
 *                 | ues are not active,  |                                  *
 *                 | other actions are    |                                  *
 *                 | taken. Zero is de-   |                                  *
 *                 | fault. This register |                                  *
 *                 | is equivalent to     |                                  *
 *                 | S_CONNECT_PNP.       |                                  *
 *                 +----------------------+                                  *
 *              S_STATES_ISA    = 0x30,    = S_STATES_PNP                    *
 *              S_SCTRL_ISA     = 0x31,                                      *
 *                HFC_POWER_DOWN  = 0x80,    do not use!                     *
 *                HFC_TX_LO_CAP   = 0x40,    capacitive or non-capacitive    *
 *                HFC_96KHZ_TEST  = 0x20,                                    *
 *                HFC_SQ_ENABLE   = 0x10,                                    *
 *                HFC_D_LO_PRIO   = 0x08,                                    *
 *                HFC_NT_MODE     = 0x04,                                    *
 *                HFC_B2_TX_EN    = 0x02,                                    *
 *                HFC_B1_TX_EN    = 0x01,                                    *
 *              S_TEST_ISA      = 0x32,                                      *
 *                HFC_ST_AWAKE_EN = 0x01,                                    *
 *              S_SQ_REC        = 0x34,                                      *
 *              S_SQ_SEND       = 0x34,    = S_SQ_REC_ISA = S_SQ_REC_PNP     *
 *              S_CLKDEL_ISA    = 0x37,    = S_CLKDEL_PNP                    *
 *                                                                           *
 *---------------+=================================================+---------*
 *               | ISAC S/T chip registers and corresponding masks.|         *
 *               | Parts of this list was imported from i4b_isac.h |         *
 *               | which was written by Gary Jennejohn.            |         *
 *               +=================================================+         *
 *                                                                           *
 *              ISAC_FIFO_LEN  =  32,                                        *
 *                                                                           *
 *              I_ISTA          = 0x20,                                      *
 *                ISAC_ISTA_RME   = 0x80,                                    *
 *                ISAC_ISTA_RPF   = 0x40,                                    *
 *                ISAC_ISTA_RSC   = 0x20,                                    *
 *                ISAC_ISTA_XPR   = 0x10,                                    *
 *                ISAC_ISTA_TIN   = 0x08,                                    *
 *                ISAC_ISTA_CISQ  = 0x04,                                    *
 *                ISAC_ISTA_SIN   = 0x02,                                    *
 *                ISAC_ISTA_EXI   = 0x01,                                    *
 *              I_MASK          = 0x20,                                      *
 *                ISAC_MASK_RME   = 0x80,                                    *
 *                ISAC_MASK_RPF   = 0x40,                                    *
 *                ISAC_MASK_RSC   = 0x20,                                    *
 *                ISAC_MASK_XPR   = 0x10,                                    *
 *                ISAC_MASK_TIN   = 0x08,                                    *
 *                ISAC_MASK_CISQ  = 0x04,                                    *
 *                ISAC_MASK_SIN   = 0x02,                                    *
 *                ISAC_MASK_EXI   = 0x01,                                    *
 *                ISAC_MASK_ALL   = 0xff,                                    *
 *              I_STAR          = 0x21,                                      *
 *                ISAC_STAR_XDOV  = 0x80,                                    *
 *                ISAC_STAR_XFW   = 0x40,                                    *
 *                ISAC_STAR_XRNR  = 0x20,                                    *
 *                ISAC_STAR_RRNR  = 0x10,                                    *
 *                ISAC_STAR_MBR   = 0x08,                                    *
 *                ISAC_STAR_MAC1  = 0x04,                                    *
 *                ISAC_STAR_BVS   = 0x02,                                    *
 *                ISAC_STAR_MAC0  = 0x01,                                    *
 *              I_CMDR          = 0x21,                                      *
 *                ISAC_CMDR_RMC   = 0x80,                                    *
 *                ISAC_CMDR_RRES  = 0x40,                                    *
 *                ISAC_CMDR_RNR   = 0x20,                                    *
 *                ISAC_CMDR_STI   = 0x10,                                    *
 *                ISAC_CMDR_XTF   = 0x08,                                    *
 *                ISAC_CMDR_XIF   = 0x04,                                    *
 *                ISAC_CMDR_XME   = 0x02,                                    *
 *                ISAC_CMDR_XRES  = 0x01,                                    *
 *              I_MODE          = 0x22,                                      *
 *                ISAC_MODE_MDS2  = 0x80,                                    *
 *                ISAC_MODE_MDS1  = 0x40,                                    *
 *                ISAC_MODE_MDS0  = 0x20,                                    *
 *                ISAC_MODE_TMD   = 0x10,                                    *
 *                ISAC_MODE_RAC   = 0x08,                                    *
 *                ISAC_MODE_DIM2  = 0x04,                                    *
 *                ISAC_MODE_DIM1  = 0x02,                                    *
 *                ISAC_MODE_DIM0  = 0x01,                                    *
 *              I_TIMR          = 0x23,                                      *
 *              I_EXIR          = 0x24,         Response to EXIR:            *
 *                ISAC_EXIR_XMR   = 0x80,         CMDR_XRES                  *
 *                ISAC_EXIR_XDU   = 0x40,         CMDR_XRES                  *
 *                ISAC_EXIR_PCE   = 0x20,                                    *
 *                ISAC_EXIR_RFO   = 0x10,         CMDR_RMC+CMDR_RRES         *
 *                ISAC_EXIR_SOV   = 0x08,                                    *
 *                ISAC_EXIR_MOS   = 0x04,                                    *
 *                ISAC_EXIR_SAW   = 0x02,                                    *
 *                ISAC_EXIR_WOV   = 0x01,                                    *
 *              I_XAD1          = 0x24,                                      *
 *              I_RBCL          = 0x25,                                      *
 *              I_XAD2          = 0x25,                                      *
 *              I_SAPR          = 0x26,                                      *
 *              I_SAP1          = 0x26,                                      *
 *              I_RSTA          = 0x27,                                      *
 *                ISAC_RSTA_RDA   = 0x80,                                    *
 *                ISAC_RSTA_RDO   = 0x40,                                    *
 *                ISAC_RSTA_CRC   = 0x20,                                    *
 *                ISAC_RSTA_RAB   = 0x10,                                    *
 *                ISAC_RSTA_SA1   = 0x08,                                    *
 *                ISAC_RSTA_SA0   = 0x04,                                    *
 *                ISAC_RSTA_CR    = 0x02,                                    *
 *                ISAC_RSTA_TA    = 0x01,                                    *
 *              I_SAP2          = 0x27,                                      *
 *              I_TEI1          = 0x28,                                      *
 *              I_RHCR          = 0x29,                                      *
 *              I_TEI2          = 0x29,                                      *
 *              I_RBCH          = 0x2a,                                      *
 *                ISAC_RBCH_XAC   = 0x80,                                    *
 *                ISAC_RBCH_VN1   = 0x40,                                    *
 *                ISAC_RBCH_VN0   = 0x20,                                    *
 *                ISAC_RBCH_OV    = 0x10,                                    *
 *                 +----------------------+                                  *
 *                 | The other 4 bits are |                                  *
 *                 | the high bits of the |                                  *
 *                 | receive byte count.  |                                  *
 *                 +----------------------+                                  *
 *              I_STAR2         = 0x2b,                                      *
 *              I_SPCR          = 0x30,                                      *
 *                ISAC_SPCR_SPU   = 0x80,                                    *
 *                ISAC_SPCR_SAC   = 0x40,                                    *
 *                ISAC_SPCR_SPM   = 0x20,                                    *
 *                ISAC_SPCR_TLP   = 0x10,                                    *
 *                ISAC_SPCR_C1C1  = 0x08,                                    *
 *                ISAC_SPCR_C1C0  = 0x04,                                    *
 *                ISAC_SPCR_C2C1  = 0x02,                                    *
 *                ISAC_SPCR_C2C0  = 0x01,                                    *
 *              I_CIR0          = 0x31,                                      *
 *                ISAC_CIRR_SQC   = 0x80,                                    *
 *                ISAC_CIRR_BAS   = 0x40,                                    *
 *                 +----------------------+                                  *
 *                 | Bits 5-2 make up     |                                  *
 *                 | the C/I code. Bit[0] |                                  *
 *                 | is always zero.      |                                  *
 *                 +----------------------+                                  *
 *              I_CIX0          = 0x31,                                      *
 *              I_MOR0          = 0x32,                                      *
 *              I_MOX0          = 0x32,                                      *
 *              I_CIR1          = 0x33,                                      *
 *              I_CIX1          = 0x33,                                      *
 *              I_MOR1          = 0x34,                                      *
 *              I_MOX1          = 0x34,                                      *
 *              I_C1R           = 0x35,                                      *
 *              I_C2R           = 0x36,                                      *
 *              I_B1CR          = 0x37,                                      *
 *              I_STCR          = 0x37,                                      *
 *                ISAC_STCR_TSF   = 0x80,                                    *
 *                ISAC_STCR_TBA2  = 0x40,                                    *
 *                ISAC_STCR_TBA1  = 0x20,                                    *
 *                ISAC_STCR_TBA0  = 0x10,                                    *
 *                ISAC_STCR_ST1   = 0x08,                                    *
 *                ISAC_STCR_ST0   = 0x04,                                    *
 *                ISAC_STCR_SC1   = 0x02,                                    *
 *                ISAC_STCR_SC0   = 0x01,                                    *
 *              I_B2CR          = 0x38,                                      *
 *              I_ADF1          = 0x38,                                      *
 *                ISAC_ADF1_WTC1  = 0x80,                                    *
 *                ISAC_ADF1_WTC2  = 0x40,                                    *
 *                ISAC_ADF1_TEM   = 0x20,                                    *
 *                ISAC_ADF1_PFS   = 0x10,                                    *
 *                ISAC_ADF1_CFS   = 0x08,                                    *
 *                ISAC_ADF1_FC2   = 0x04,                                    *
 *                ISAC_ADF1_FC1   = 0x02,                                    *
 *                ISAC_ADF1_ITF   = 0x01,                                    *
 *              I_ADF2          = 0x39,                                      *
 *                ISAC_ADF2_IMS   = 0x80,                                    *
 *                 +----------------------+                                  *
 *                 | All other bits are 0.|                                  *
 *                 +----------------------+                                  *
 *              I_MOSR          = 0x3a,                                      *
 *              I_MOCR          = 0x3a,                                      *
 *                ISAC_MOCR_MRE1  = 0x80,                                    *
 *                ISAC_MOCR_MRC1  = 0x40,                                    *
 *                ISAC_MOCR_MXE1  = 0x20,                                    *
 *                ISAC_MOCR_MXC1  = 0x10,                                    *
 *                ISAC_MOCR_MRE0  = 0x08,                                    *
 *                ISAC_MOCR_MRC0  = 0x04,                                    *
 *                ISAC_MOCR_MXE0  = 0x02,                                    *
 *                ISAC_MOCR_MXC0  = 0x01,                                    *
 *              I_SQRR          = 0x3b,                                      *
 *                ISAC_SQRR_SYN   = 0x10,                                    *
 *                ISAC_SQRR_SQR2  = 0x04,                                    *
 *                ISAC_SQRR_SQR3  = 0x02,                                    *
 *                ISAC_SQRR_SQR4  = 0x01,                                    *
 *              I_SQXR          = 0x3b,                                      *
 *                ISAC_SQXR_IDC   = 0x80,                                    *
 *                ISAC_SQXR_CFS   = 0x40,                                    *
 *                ISAC_SQXR_CI1E  = 0x20,                                    *
 *                ISAC_SQXR_SQIE  = 0x10,                                    *
 *                ISAC_SQXR_SQX1  = 0x08,                                    *
 *                ISAC_SQXR_SQX2  = 0x04,                                    *
 *                ISAC_SQXR_SQX3  = 0x02,                                    *
 *                ISAC_SQXR_SQX4  = 0x01,                                    *
 *                                                                           *
 *---------------+=================================================+---------*
 *               | HSCX     chip registers and corresponding masks.|         *
 *               | Parts of this list was imported from i4b_hscx.h |         *
 *               | which was written by Gary Jennejohn.            |         *
 *               +=================================================+         *
 *                                                                           *
 *              HSCX_FIFO_LEN  =  0x20,    32 (dec)                          *
 *              HSCX_OFFS_A    =  0x00,   + B1-channel +                     *
 *              HSCX_OFFS_B    =  0x40,   + B2-channel +                     *
 *                                                                           *
 *              H_ISTA          = 0x20,                                      *
 *                HSCX_ISTA_RME   = 0x80,                                    *
 *                HSCX_ISTA_RPF   = 0x40,                                    *
 *                HSCX_ISTA_RSC   = 0x20,                                    *
 *                HSCX_ISTA_XPR   = 0x10,                                    *
 *                HSCX_ISTA_TIN   = 0x08,                                    *
 *                HSCX_ISTA_ICA   = 0x04,  + B2-channel only (IC-B) +        *
 *                HSCX_ISTA_EXI_A = 0x02,  + B2-channel only (IC-B) +        *
 *                HSCX_ISTA_EXI_B = 0x01,  + B2-channel only (IC-B) +        *
 *              H_MASK          = 0x20,                                      *
 *                HSCX_MASK_RME   = 0x80,                                    *
 *                HSCX_MASK_RPF   = 0x40,                                    *
 *                HSCX_MASK_RSC   = 0x20,                                    *
 *                HSCX_MASK_XPR   = 0x10,                                    *
 *                HSCX_MASK_TIN   = 0x08,                                    *
 *                HSCX_MASK_ICA   = 0x04,                                    *
 *                HSCX_MASK_EXI_A = 0x02,                                    *
 *                HSCX_MASK_EXI   = 0x01,                                    *
 *                HSCX_MASK_ALL   = 0xff,                                    *
 *              H_STAR          = 0x21,                                      *
 *                HSCX_STAR_XDOV  = 0x80,                                    *
 *                HSCX_STAR_XFW   = 0x40,                                    *
 *                HSCX_STAR_XRNR  = 0x20,                                    *
 *                HSCX_STAR_RRNR  = 0x10,                                    *
 *                HSCX_STAR_RLI   = 0x08,                                    *
 *                HSCX_STAR_CEC   = 0x04,                                    *
 *                HSCX_STAR_CTS   = 0x02,                                    *
 *                HSCX_STAR_WFA   = 0x01,                                    *
 *              H_CMDR          = 0x21,                                      *
 *                HSCX_CMDR_RMC   = 0x80,                                    *
 *                HSCX_CMDR_RHR   = 0x40,                                    *
 *                HSCX_CMDR_RNR   = 0x20,                                    *
 *                HSCX_CMDR_STI   = 0x10,                                    *
 *                HSCX_CMDR_XTF   = 0x08,                                    *
 *                HSCX_CMDR_XIF   = 0x04,                                    *
 *                HSCX_CMDR_XME   = 0x02,                                    *
 *                HSCX_CMDR_XRES  = 0x01,                                    *
 *              H_MODE          = 0x22,                                      *
 *                HSCX_MODE_MDS1  = 0x80,                                    *
 *                HSCX_MODE_MDS0  = 0x40,                                    *
 *                HSCX_MODE_ADM   = 0x20,                                    *
 *                HSCX_MODE_TMD   = 0x10,                                    *
 *                HSCX_MODE_RAC   = 0x08,                                    *
 *                HSCX_MODE_RTS   = 0x04,                                    *
 *                HSCX_MODE_TRS   = 0x02,                                    *
 *                HSCX_MODE_TLP   = 0x01,                                    *
 *              H_TIMR          = 0x23,                                      *
 *              H_EXIR          = 0x24,                                      *
 *                HSCX_EXIR_XMR   = 0x80,                                    *
 *                HSCX_EXIR_XDU   = 0x40,                                    *
 *                HSCX_EXIR_PCE   = 0x20,                                    *
 *                HSCX_EXIR_RFO   = 0x10,                                    *
 *                HSCX_EXIR_CSC   = 0x08,                                    *
 *                HSCX_EXIR_RFS   = 0x04,                                    *
 *                HSCX_EXIR_SAW   = 0x02,                                    *
 *                HSCX_EXIR_WOV   = 0x01,                                    *
 *              H_XAD1          = 0x24,                                      *
 *              H_RBCL          = 0x25,         == ISAC                      *
 *              H_XAD2          = 0x25,                                      *
 *              H_RAH1          = 0x26,                                      *
 *              H_RSTA          = 0x27,                                      *
 *                HSCX_RSTA_RDA   = 0x80,                                    *
 *                HSCX_RSTA_RDO   = 0x40,                                    *
 *                HSCX_RSTA_CRC   = 0x20,                                    *
 *                HSCX_RSTA_RAB   = 0x10,                                    *
 *                HSCX_RSTA_HA1   = 0x08,                                    *
 *                HSCX_RSTA_HA0   = 0x04,                                    *
 *                HSCX_RSTA_CR    = 0x02,                                    *
 *                HSCX_RSTA_LA    = 0x01,                                    *
 *              H_RAH2          = 0x27,                                      *
 *              H_RAL1          = 0x28,                                      *
 *              H_RHCR          = 0x29,                                      *
 *              H_RAL2          = 0x29,                                      *
 *              H_XBCL          = 0x2a,                                      *
 *              H_BGR           = 0x2b,                                      *
 *              H_CCR2          = 0x2c,                                      *
 *                + for clock mode 5 +                                       *
 *                HSCX_CCR2_SOC2  = 0x80,                                    *
 *                HSCX_CCR2_SOC1  = 0x40,                                    *
 *                HSCX_CCR2_XCS0  = 0x20,                                    *
 *                HSCX_CCR2_RCS0  = 0x10,                                    *
 *                HSCX_CCR2_TIO   = 0x08,                                    *
 *                HSCX_CCR2_CIE   = 0x04,                                    *
 *                HSCX_CCR2_RIE   = 0x02,                                    *
 *                HSCX_CCR2_DIV   = 0x01,                                    *
 *              H_XBCH          = 0x2d,         ISAC: regmove                *
 *              H_RBCH          = 0x2d,         ISAC: regmove                *
 *                HSCX_RBCH_DMA   = 0x80,                                    *
 *                HSCX_RBCH_NRM   = 0x40,                                    *
 *                HSCX_RBCH_CAS   = 0x20,                                    *
 *                HSCX_RBCH_OV    = 0x10,                                    *
 *                 +----------------------+                                  *
 *                 | The other 4 bits are |                                  *
 *                 | the high bits of the |                                  *
 *                 | receive byte count.  |                                  *
 *                 +----------------------+                                  *
 *              H_VSTR          = 0x2e,                                      *
 *              H_RLCR          = 0x2e,                                      *
 *              H_CCR1          = 0x2f,                                      *
 *                HSCX_CCR1_PU    = 0x80,                                    *
 *                + bits 6 and 5 are SC1 SC0 +                               *
 *                HSCX_CCR1_ODS   = 0x10,                                    *
 *                HSCX_CCR1_ITF   = 0x08,                                    *
 *                HSCX_CCR1_CM2   = 0x04,                                    *
 *                HSCX_CCR1_CM1   = 0x02,                                    *
 *                HSCX_CCR1_CM0   = 0x01,                                    *
 *              H_TSAX          = 0x30,                                      *
 *                HSCX_TSAX_XCS2  = 0x02,                                    *
 *                HSCX_TSAX_XCS1  = 0x01,                                    *
 *              H_TSAR          = 0x31,                                      *
 *                HSCX_TSAR_RCS2  = 0x02,                                    *
 *                HSCX_TSAR_RCS1  = 0x01,                                    *
 *              H_XCCR          = 0x32,                                      *
 *              H_RCCR          = 0x33,                                      *
 *                                                                           *
 *---------------+=================================================+---------*
 *               | IPAC     chip registers and corresponding masks.|         *
 *               | Parts of this list was imported from i4b_ipac.h |         *
 *               | which was written by Hellmuth Michaelis.        |         *
 *               +=================================================+         *
 *                                                                           *
 *              IPAC_D_FIFO_LEN=  0x20,    32 (dec)                          *
 *              IPAC_B_FIFO_LEN=  0x40,    64 (dec)                          *
 *              HSCX_OFFS      =  0x00,                                      *
 *              ISAC_OFFS      =  0x80,                                      *
 *              IPAC_OFFS      =  0xC0,                                      *
 *                                                                           *
 *              P_CONF         = 0x00,                                       *
 *                IPAC_CONF_AMP  = 0x80,                                     *
 *                IPAC_CONF_CFS  = 0x40,                                     *
 *                IPAC_CONF_TEM  = 0x20,                                     *
 *                IPAC_CONF_PDS  = 0x10,                                     *
 *                IPAC_CONF_IDH  = 0x08,                                     *
 *                IPAC_CONF_SGO  = 0x04,                                     *
 *                IPAC_CONF_ODS  = 0x02,                                     *
 *                IPAC_CONF_IOF  = 0x01,                                     *
 *              P_ISTA         = 0x01,                                       *
 *                IPAC_ISTA_INT1 = 0x80,                                     *
 *                IPAC_ISTA_INT0 = 0x40,                                     *
 *                IPAC_ISTA_ICD  = 0x20,                                     *
 *                IPAC_ISTA_EXD  = 0x10,                                     *
 *                IPAC_ISTA_ICA  = 0x08,                                     *
 *                IPAC_ISTA_EXA  = 0x04,                                     *
 *                IPAC_ISTA_ICB  = 0x02,                                     *
 *                IPAC_ISTA_EXB  = 0x01,                                     *
 *              P_MASK         = 0x01,                                       *
 *                IPAC_MASK_INT1 = 0x80,                                     *
 *                IPAC_MASK_INT0 = 0x40,                                     *
 *                IPAC_MASK_ICD  = 0x20,                                     *
 *                IPAC_MASK_EXD  = 0x10,                                     *
 *                IPAC_MASK_ICA  = 0x08,                                     *
 *                IPAC_MASK_EXA  = 0x04,                                     *
 *                IPAC_MASK_ICB  = 0x02,                                     *
 *                IPAC_MASK_EXB  = 0x01,                                     *
 *              P_ID           = 0x02,                                       *
 *                IPAC_V11       = 0x01,        IPAC Version 1.1             *
 *                IPAC_V12       = 0x02,        IPAC Version 1.2             *
 *              P_ACFG         = 0x03,                                       *
 *                IPAC_ACFG_OD7  = 0x80,                                     *
 *                IPAC_ACFG_OD6  = 0x40,                                     *
 *                IPAC_ACFG_OD5  = 0x20,                                     *
 *                IPAC_ACFG_OD4  = 0x10,                                     *
 *                IPAC_ACFG_OD3  = 0x08,                                     *
 *                IPAC_ACFG_OD2  = 0x04,                                     *
 *                IPAC_ACFG_EL1  = 0x02,                                     *
 *                IPAC_ACFG_EL2  = 0x01,                                     *
 *              P_AOE          = 0x04,                                       *
 *                IPAC_AOE_OE7   = 0x80,                                     *
 *                IPAC_AOE_OE6   = 0x40,                                     *
 *                IPAC_AOE_OE5   = 0x20,                                     *
 *                IPAC_AOE_OE4   = 0x10,                                     *
 *                IPAC_AOE_OE3   = 0x08,                                     *
 *                IPAC_AOE_OE2   = 0x04,                                     *
 *              P_ARX          = 0x05,                                       *
 *                IPAC_ARX_AR7   = 0x80,                                     *
 *                IPAC_ARX_AR6   = 0x40,                                     *
 *                IPAC_ARX_AR5   = 0x20,                                     *
 *                IPAC_ARX_AR4   = 0x10,                                     *
 *                IPAC_ARX_AR3   = 0x08,                                     *
 *                IPAC_ARX_AR2   = 0x04,                                     *
 *              P_ATX          = 0x05,                                       *
 *                IPAC_ATX_AT7   = 0x80,                                     *
 *                IPAC_ATX_AT6   = 0x40,                                     *
 *                IPAC_ATX_AT5   = 0x20,                                     *
 *                IPAC_ATX_AT4   = 0x10,                                     *
 *                IPAC_ATX_AT3   = 0x08,                                     *
 *                IPAC_ATX_AT2   = 0x04,                                     *
 *              P_PITA1        = 0x06,                                       *
 *                IPAC_PITA1_ENA  = 0x80,                                    *
 *                IPAC_PITA1_DUDD = 0x40,                                    *
 *              P_PITA2         = 0x07,                                      *
 *                IPAC_PITA2_ENA  = 0x80,                                    *
 *                IPAC_PITA2_DUDD = 0x40,                                    *
 *              P_POTA1         = 0x08,                                      *
 *                IPAC_POTA1_ENA  = 0x80,                                    *
 *                IPAC_POTA1_DUDD = 0x40,                                    *
 *              P_POTA2         = 0x09,                                      *
 *                IPAC_POTA2_ENA  = 0x80,                                    *
 *                IPAC_POTA2_DUDD = 0x40,                                    *
 *              P_PCFG          = 0x0a,                                      *
 *                IPAC_PCFG_DPS   = 0x80,                                    *
 *                IPAC_PCFG_ACL   = 0x40,                                    *
 *                IPAC_PCFG_LED   = 0x20,                                    *
 *                IPAC_PCFG_PLD   = 0x10,                                    *
 *                IPAC_PCFG_FBS   = 0x08,                                    *
 *                IPAC_PCFG_CSL2  = 0x04,                                    *
 *                IPAC_PCFG_CSL1  = 0x02,                                    *
 *                IPAC_PCFG_CSL0  = 0x01,                                    *
 *              P_SCFG          = 0x0b,                                      *
 *                IPAC_SCFG_PRI   = 0x80,                                    *
 *                IPAC_SCFG_TXD   = 0x40,                                    *
 *                IPAC_SCFG_TLEN  = 0x20,                                    *
 *              P_TIMR2         = 0x0c,                                      *
 *                IPAC_TIMR2_TMD  = 0x80,                                    *
 *                                                                           *
 *---------------+===================================================+-------*
 *               | Tiger300/320 ASIC registers and corresponding     |       *
 *               | masks. This list was imported from i4b_ijtc_pci.c |       *
 *               | which was written by Sergio Prallon.              |       *
 *               +===================================================+       *
 *                                                                           *
 *              PIB_OFFSET  =  0xc0,                                         *
 *                                                                           *
 *              T_RST_PIB_CL_TIME       = 0x00,  (short: t_prct)             *
 *                TIGER_PIB_MASK          = 0x30,                            *
 *                TIGER_PIB_12_CYCLES     = 0x20,                            *
 *                TIGER_PIB_5_CYCLES      = 0x10,                            *
 *                TIGER_PIB_3_CYCLES      = 0x00,                            *
 *                TIGER_RST_PULSE_COUNT   = 0x08,                            *
 *                TIGER_RST_SERIAL_PORT   = 0x04,                            *
 *                TIGER_RST_DMA_LOGIC     = 0x02,                            *
 *                TIGER_RST_EXTERNAL      = 0x01,                            *
 *                TIGER_RST_ALL           = 0x0f,                            *
 *              T_DMA_OPER              = 0x01,                              *
 *                TIGER_DMA_SELF_ADDR     = 0x00,                            *
 *                TIGER_DMA_NORMAL        = 0x80,                            *
 *                TIGER_DMA_ENABLE        = 0x01,                            *
 *              T_AUX_CNTL              = 0x02,                              *
 *              T_AUX_DATA              = 0x03,                              *
 *              T_INT0_MASK             = 0x04,                              *
 *                TIGER_DMA_LATCH_INT     = 0x40,                            *
 *                TIGER_PCI_TARGET_ABORT  = 0x20,                            *
 *                TIGER_PCI_MASTER_ABORT  = 0x10,                            *
 *                TIGER_DMA_RD_END        = 0x08,                            *
 *                TIGER_DMA_RD_INT        = 0x04,                            *
 *                TIGER_DMA_WR_END        = 0x02,                            *
 *                TIGER_DMA_WR_INT        = 0x01,                            *
 *              T_INT1_MASK             = 0x05,                              *
 *              T_INT0_STATUS           = 0x06,                              *
 *                TIGER_DMA_LATCH_INT     = 0x40,                            *
 *                TIGER_PCI_TARGET_ABORT  = 0x20,                            *
 *                TIGER_PCI_MASTER_ABORT  = 0x10,                            *
 *                TIGER_DMA_RD_END        = 0x08,                            *
 *                TIGER_DMA_RD_INT        = 0x04,                            *
 *                TIGER_DMA_WR_END        = 0x02,                            *
 *                TIGER_DMA_WR_INT        = 0x01,                            *
 *              T_INT1_STATUS           = 0x07,                              *
 *              T_DMA_WR_START_ADDR     = 0x08,                              *
 *              T_DMA_WR_INT_ADDR       = 0x0c,                              *
 *              T_DMA_WR_END_ADDR       = 0x10,                              *
 *              T_DMA_WR_CURR_ADDR      = 0x14,                              *
 *              T_DMA_RD_START_ADDR     = 0x18,                              *
 *              T_DMA_RD_INT_ADDR       = 0x1c,                              *
 *              T_DMA_RD_END_ADDR       = 0x20,                              *
 *              T_DMA_RD_CURR_ADDR      = 0x24,                              *
 *              T_PULSE_COUNTER         = 0x28,                              *
 *                                                                           *
 *---------------+===================================================+-------*
 *               | W6692x  (Winbond) This list was imported from     |       *
 *		 | i4b_w6692.h which was written by Dave Boyce.      |	     *
 *               +===================================================+       *
 *                                                                           *
 *              IWIC_DCHAN_FIFO_LEN = 64,                                    *
 *              IWIC_BCHAN_FIFO_LEN = 64,                                    *
 *                                                                           *
 *    +----------------------------+                                         *
 *    | D-Channel register offsets |                                         *
 *    +----------------------------+                                         *
 *              W_DRFIFO       = 0x00,    D channel receive FIFO             *
 *              W_DXFIFO       = 0x04,    D channel transmit FIFO            *
 *              W_DCMDR        = 0x08,    D channel command register         *
 *                DCMDR_RACK     = 0x80,                                     *
 *                DCMDR_RRST     = 0x40,                                     *
 *                DCMDR_STT      = 0x10,                                     *
 *                DCMDR_XMS      = 0x08,                                     *
 *                DCMDR_XME      = 0x02,                                     *
 *                DCMDR_XRST     = 0x01,                                     *
 *              W_DMODE        = 0x0c,    D channel mode control             *
 *                DMODE_MMS      = 0x80,                                     *
 *                DMODE_RACT     = 0x40,                                     *
 *                DMODE_TMS      = 0x10,                                     *
 *                DMODE_TEE      = 0x08,                                     *
 *                DMODE_MFD      = 0x04,                                     *
 *                DMODE_DLP      = 0x02,                                     *
 *                DMODE_RLP      = 0x01,                                     *
 *              W_DTIMR        = 0x10,    D channel timer control            *
 *              W_DEXIR        = 0x1c,    D channel extended interrupt       *
 *                DEXIR_RDOV     = 0x80,                                     *
 *                DEXIR_XDUN     = 0x40,                                     *
 *                DEXIR_XCOL     = 0x20,                                     *
 *                DEXIR_TIN2     = 0x10,                                     *
 *                DEXIR_MOC      = 0x08,                                     *
 *                DEXIR_ISC      = 0x04,                                     *
 *                DEXIR_TEXP     = 0x02,                                     *
 *                DEXIR_WEXP     = 0x01,                                     *
 *              W_DEXIM        = 0x20,    D channel extended interrupt mask  *
 *                DEXIM_RDOV     = 0x80,                                     *
 *                DEXIM_XDUN     = 0x40,                                     *
 *                DEXIM_XCOL     = 0x20,                                     *
 *                DEXIM_TIM2     = 0x10,                                     *
 *                DEXIM_MOC      = 0x08,                                     *
 *                DEXIM_ISC      = 0x04,                                     *
 *                DEXIM_TEXP     = 0x02,                                     *
 *                DEXIM_WEXP     = 0x01,                                     *
 *              W_DSTAR        = 0x24,    D channel status register          *
 *                DSTAR_XDOW     = 0x80,                                     *
 *                DSTAR_XBZ      = 0x20,                                     *
 *                DSTAR_DRDY     = 0x10,                                     *
 *              W_DRSTA        = 0x28,    D channel receive status           *
 *                DRSTA_RDOV     = 0x40,                                     *
 *                DRSTA_CRCE     = 0x20,                                     *
 *                DRSTA_RMB      = 0x10,                                     *
 *              W_DSAM         = 0x2c,    D channel address mask 1           *
 *              W_DSAP1        = 0x30,    D channel individual SAPI 1        *
 *              W_DSAP2        = 0x34,    D channel individual SAPI 2        *
 *              W_DTAM         = 0x38,    D channel address mask 2           *
 *              W_DTEI1        = 0x3c,    D channel individual TEI 1         *
 *              W_DTEI2        = 0x40,    D channel individual TEI 2         *
 *              W_DRBCH        = 0x44,    D channel rx frame byte count high *
 *              W_DRBCL        = 0x48,    D channel rx frame byte count low  *
 *              W_DCTL         = 0x54,    D channel control register         *
 *                DCTL_SRST      = 0x20,                                     *
 *                DCTL_TPS       = 0x04,                                     *
 *                                                                           *
 *      W_B1_CHAN_OFFSET = 0x80,   B1 channel offset                         *
 *      W_B2_CHAN_OFFSET = 0xC0,   B2 channel offset                         *
 *                                                                           *
 *    +---------------------------------------+                              *
 *    | B-channel register offsets(from base) |                              *
 *    +---------------------------------------+                              *
 *              W_BRFIFO       = 0x00,    B channel receive FIFO             *
 *              W_BXFIFO       = 0x04,    B channel transmit FIFO            *
 *              W_BCMDR        = 0x08,    B channel command register         *
 *                BCMDR_RACK     = 0x80,                                     *
 *                BCMDR_RRST     = 0x40,                                     *
 *                BCMDR_RACT     = 0x20,                                     *
 *                BCMDR_XMS      = 0x04,                                     *
 *                BCMDR_XME      = 0x02,                                     *
 *                BCMDR_XRST     = 0x01,                                     *
 *              W_BMODE        = 0x0c,    B channel mode control             *
 *                BMODE_MMS      = 0x80,                                     *
 *                BMODE_ITF      = 0x40,                                     *
 *                BMODE_EPCM     = 0x20,                                     *
 *                BMODE_BSW1     = 0x10,                                     *
 *                BMODE_BSW0     = 0x08,                                     *
 *                BMODE_SW56     = 0x04,                                     *
 *                BMODE_FTS1     = 0x02,                                     *
 *                BMODE_FTS0     = 0x01,                                     *
 *              W_BEXIR        = 0x10,    B channel extended interrupt       *
 *                BEXIR_RMR      = 0x40,                                     *
 *                BEXIR_RME      = 0x20,                                     *
 *                BEXIR_RDOV     = 0x10,                                     *
 *                BEXIR_XFR      = 0x02,                                     *
 *                BEXIR_XDUN     = 0x01,                                     *
 *              W_BEXIM        = 0x14,    B channel extended interrupt mask  *
 *                BEXIM_RMR      = 0x40,                                     *
 *                BEXIM_RME      = 0x20,                                     *
 *                BEXIM_RDOV     = 0x10,                                     *
 *                BEXIM_XFR      = 0x02,                                     *
 *                BEXIM_XDUN     = 0x01,                                     *
 *              W_BSTAR        = 0x18,    B channel status register          *
 *                BSTAR_RDOV     = 0x40,                                     *
 *                BSTAR_CRCE     = 0x20,                                     *
 *                BSTAR_RMB      = 0x10,                                     *
 *                BSTAR_XDOW     = 0x04,                                     *
 *                BSTAR_XBZ      = 0x01,                                     *
 *              W_BADM1        = 0x1c,    B channel address mask 1           *
 *              W_BADM2        = 0x20,    B channel address mask 2           *
 *              W_BADR1        = 0x24,    B channel address 1                *
 *              W_BADR2        = 0x28,    B channel address 2                *
 *              W_BRBCL        = 0x2c,    B channel rx frame byte count high *
 *              W_BRBCH        = 0x30,    B channel rx frame byte count low  *
 *                                                                           *
 *    +------------------------------------+                                 *
 *    | Remaining control register offsets |                                 *
 *    +------------------------------------+                                 *
 *              W_ISTA          = 0x14,   Interrupt status register          *
 *                ISTA_D_RMR      = 0x80,                                    *
 *                ISTA_D_RME      = 0x40,                                    *
 *                ISTA_D_XFR      = 0x20,                                    *
 *                ISTA_XINT1      = 0x10,                                    *
 *                ISTA_XINT0      = 0x08,                                    *
 *                ISTA_D_EXI      = 0x04,                                    *
 *                ISTA_B1_EXI     = 0x02,                                    *
 *                ISTA_B2_EXI     = 0x01,                                    *
 *              W_IMASK         = 0x18,    Interrupt mask register           *
 *                IMASK_D_RMR     = 0x80,                                    *
 *                IMASK_D_RME     = 0x40,                                    *
 *                IMASK_D_XFR     = 0x20,                                    *
 *                IMASK_XINT1     = 0x10,                                    *
 *                IMASK_XINT0     = 0x08,                                    *
 *                IMASK_D_EXI     = 0x04,                                    *
 *                IMASK_B1_EXI    = 0x02,                                    *
 *                IMASK_B2_EXI    = 0x01,                                    *
 *              W_TIMR2         = 0x4c,    Timer 2                           *
 *              W_L1_RC         = 0x50,    GCI layer 1 ready code            *
 *              W_CIR           = 0x58,    Command/Indication receive        *
 *                CIR_CE          = 0x07,                                    *
 *                CIR_DRD         = 0x00,                                    *
 *                CIR_LD          = 0x04,                                    *
 *                CIR_ARD         = 0x08,                                    *
 *                CIR_TI          = 0x0a,                                    *
 *                CIR_ATI         = 0x0b,                                    *
 *                CIR_AI8         = 0x0c,                                    *
 *                CIR_AI10        = 0x0d,                                    *
 *                CIR_CD          = 0x0f,                                    *
 *                CIR_SCC         = 0x80,                                    *
 *                CIR_ICC         = 0x40,                                    *
 *                CIR_CODR(i)     ((i) & 0x0f)                               *
 *              W_CIX           = 0x5c,    Command/Indication transmit       *
 *                CIX_ECK         = 0x00,                                    *
 *                CIX_RST         = 0x01,                                    *
 *                CIX_SCP         = 0x04,                                    *
 *                CIX_SSP         = 0x02,                                    *
 *                CIX_AR8         = 0x08,                                    *
 *                CIX_AR10        = 0x09,                                    *
 *                CIX_EAL         = 0x0a,                                    *
 *                CIX_DRC         = 0x0f,                                    *
 *              W_SQR           = 0x60,    S/Q channel receive register      *
 *                SQR_XIND1       = 0x80,                                    *
 *                SQR_XIND0       = 0x40,                                    *
 *                SQR_MSYN        = 0x20,                                    *
 *                SQR_SCIE        = 0x10,                                    *
 *                SQR_S(i)        ((i) & 0x0f)                               *
 *              W_SQX           = 0x64,    S/Q channel transmit register     *
 *                SQX_SCIE        = 0x10,                                    *
 *                SQX_Q(i)        ((i) & 0x0f)                               *
 *              W_PCTL          = 0x68,    Peripheral control register       *
 *              W_MOR           = 0x6c,    Monitor receive channel           *
 *              W_MOX           = 0x70,    Monitor transmit channel          *
 *              W_MOSR          = 0x74,    Monitor channel status register   *
 *              W_MOCR          = 0x78,    Monitor channel control register  *
 *              W_GCR           = 0x7c,    GCI mode control register         *
 *              W_XADDR         = 0xf4,    Peripheral address register       *
 *              W_XDATA         = 0xf8,    Peripheral data register          *
 *              W_EPCTL         = 0xfc,    Serial EEPROM control             *
 *                                                                           *
 *---------------+===================================================+-------*
 *               | W6694A  (Winbond)                                 |       *
 *               +===================================================+       *
 *                                                                           *
 *              W_FIFO_SIZE = 128                                            *
 *                                                                           *
 *              W_IMASK                 = 0x00,                              *
 *              W_CMDR1                 = 0x01,                              *
 *              W_CMDR2                 = 0x02,                              *
 *              W_CTL                   = 0x03,                              *
 *              W_GCR                   = 0x06,                              *
 *              W_MOCR                  = 0x07,                              *
 *              W_PIE                   = 0x0A,                              *
 *              W_PO1                   = 0x0B,                              *
 *              W_PO2                   = 0x0C,                              *
 *              W_PDATA                 = 0x0D,                              *
 *              W_L1B1RS                = 0x0E,                              *
 *              W_L1B2RS                = 0x0F,                              *
 *              W_USBB1RS               = 0x10,                              *
 *              W_USBB2RS               = 0x11,                              *
 *              W_PCM1RS                = 0x12,                              *
 *              W_PCM2RS                = 0x13,                              *
 *                                                                           *
 * } reg_t;                                                                  *
 *===========================================================================*/

struct fsm_state {
  u_int8_t pending  : 1;
  u_int8_t active   : 1;
  u_int8_t can_up   : 1; /* can activate   */
  u_int8_t can_down : 1; /* can deactivate */
  u_int8_t command  : 1;
  u_int8_t index    : 4;
  const char * description;
} __packed;

struct fsm_command {
  u_int8_t value;
  const char * description;
} __packed;

struct fsm_table {
  struct fsm_state state[16];
  struct fsm_command  cmd[8];
} __packed;

#define IHFC_SUB_CONTROLLERS_MAX 8

struct sc_state {

  u_int32_t i4b_option_value;

  L1_auto_activate_t *L1_auto_activate_ptr;
  L1_auto_activate_t L1_auto_activate_variable;

  L1_activity_t *L1_activity_ptr;
  L1_activity_t L1_activity_variable;

  i4b_controller_t *i4b_controller;

  struct usb_callout T3callout;	/* T3 callout */
  struct fsm_state state;	/* last known state */
};

struct sc_reg_temp {
  uint16_t reg;
  uint16_t data;
};

struct ihfc_config_copy {

};

/*---------------------------------------------------------------------------*
 * : IHFC softc
 *---------------------------------------------------------------------------*/
struct ihfc_sc {

	u_int8_t		sc_nametmp[16];

	device_t		sc_device;

	void			*sc_temp_ptr;

	i4b_trace_hdr_t		sc_trace_hdr;

	register_t		sc_intr_temp;

	struct sc_state		sc_state[IHFC_SUB_CONTROLLERS_MAX];

	u_int8_t		sc_chip_interrupt_called; /* interrupt is called */
#	define			SC_T125_WAIT_DELAY     (hz / 1000)
#	define			SC_T125_WAIT_SET(sc)   (sc)->sc_default.o_T125_WAIT = 1
#	define			SC_T125_WAIT_CLEAR(sc) (sc)->sc_default.o_T125_WAIT = 0
#	define			SC_T125_WAIT(sc)      ((sc)->sc_default.o_T125_WAIT)
			/* T125:
			 * =====
			 * delay is rounded up to 1.000ms == 8*125us
			 * NOTE: the current system timer
			 * has a granularity of 10ms!
			 *
			 * ``sc_t125_wait'' is used for
			 * call-back purposes instead of
			 * using msleep().
			 */

	struct mtx *		sc_mtx_p;	/* pointer to driver mutex */
	struct usb_page	sc_hw_page;
	struct usb_page_cache	sc_hw_page_cache;
	struct usb_dma_parent_tag	sc_hw_dma_parent_tag;
	struct usb_dma_tag	sc_hw_dma_tag;

	struct sc_resources	sc_resources;
	struct sc_reg_temp	sc_reg_temp;
#ifdef IHFC_USB_ENABLED
	struct usb2_config_td	sc_config_td;
#endif
	struct sc_config	sc_config;
	struct sc_config	sc_config2;	/* shadow config */
	struct sc_config_buffer sc_config_buffer; /* used by USB */

	struct sc_default	sc_default;
#       define sc_cookie        sc_default.cookie

	struct sc_stack		sc_stack;

	u_int16_t               sc_channel_mapping;

#	define SC_INTR_BITS 8 /* bits per "sc_intr_status" */
#	if (SC_INTR_BITS != 8)
#	error "need to change type of sc_intr_status"
#	endif

	/* interrupt management */
	struct sc_fifo **	sc_intr_list_curr;
	struct sc_fifo *	sc_intr_list[IHFC_CHANNELS];

	u_int8_t		sc_intr_status[(IHFC_CHANNELS+SC_INTR_BITS-1)/SC_INTR_BITS];
	u_int8_t		sc_intr_status_end[0];

	struct usb_callout	sc_pollout_timr;      /* T50 ms  */
	struct usb_callout	sc_pollout_timr_wait; /* T125 us */

	u_int8_t		sc_buffer[1024];
  
	struct sc_fifo *	sc_fifo_select_last; /* used by 
						      * FIFO_SELECT(,) 
						      */
	struct sc_fifo *	sc_fifo_end; /* used by FIFO_FOREACH(,) */
	struct sc_fifo 		sc_fifo[IHFC_CHANNELS];
	fifo_translator_t	sc_fifo_translator[IHFC_CHANNELS/2];
#define FIFO_TRANSLATOR(sc,f) (&(sc)->sc_fifo_translator[FIFO_NO(f)/2])

	/* ihfc application interface /dev/ihfc.XXX */
	struct cdev *		sc_test_dev[IHFC_CHANNELS/2];

	struct i4b_echo_cancel  sc_echo_cancel[IHFC_CHANNELS/2];

	u_int16_t		sc_f0_counter_offset;
	u_int32_t		sc_f0_counter_last;
};

/*---------------------------------------------------------------------------*
 * : Type definitions (struct)
 *---------------------------------------------------------------------------*/
typedef	       struct ihfc_sc	      ihfc_sc_t;
typedef	       struct sc_fifo	      ihfc_fifo_t;

typedef const  struct fsm_table       fsm_t;
typedef const  struct fsm_state       fsm_state_t;
typedef const  struct fsm_command     fsm_command_t;

typedef const  struct register_list   register_list_t;

#endif /* _I4B_IHFC2_H_ */
