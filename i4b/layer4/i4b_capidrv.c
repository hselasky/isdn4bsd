/*-
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
 *	i4b_capidrv.c - CAPI userland interface driver
 *	----------------------------------------------
 *
 * Locking order:
 *
 * 1. i4b_global_lock or a controller lock
 *
 * 2. CAPI application interface lock
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/ioccom.h>
#include <sys/malloc.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/poll.h>
#include <sys/conf.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/endian.h>
#include <sys/filio.h>
#include <sys/lock.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#define CAPI_MAKE_TRANSLATOR
#include <i4b/include/capi20.h>

#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

/* the following structure describes one CAPI application */

struct capi_ai_softc {

	u_int32_t sc_info_mask[MAX_CONTROLLERS]; /* used to limit information */
	u_int32_t sc_CIP_mask_1[MAX_CONTROLLERS]; /* used to select incoming calls */
	u_int32_t sc_CIP_mask_2[MAX_CONTROLLERS]; /* used to select incoming calls */

	u_int16_t sc_max_b_data_len;

	struct cdev *sc_dev;

	struct mtx sc_mtx; /* lock that protects this structure */

	u_int16_t sc_flags;
#define ST_OPEN              0x0001 /* set if AI opened */
#define ST_OPENING           0x0002 /* set if AI is opening */
#define ST_RD_SLEEP_WAKEUP   0x0004 /* set if AI needs wakeup */
#define ST_RD_SLEEP_ENTERED  0x0008 /* set if AI is sleeping */
#define ST_WR_SLEEP_WAKEUP   0x0010 /* set if AI needs wakeup */
#define ST_WR_SLEEP_ENTERED  0x0020 /* set if AI is sleeping */
#define ST_SELECT            0x0040 /* set if AI is selected by poll */
#define ST_CLOSING           0x0080 /* set if AI is closing */
#define ST_BLOCK             0x0100 /* set if I/O is blocking */
#define ST_IOCTL             0x0200 /* set if AI is doing a IOCTL */
#define ST_D_OPEN            0x0400 /* set if D-channel has been started */
#define ST_MBUF_LOST         0x0800 /* set if an mbuf was lost */

	struct _ifqueue sc_rdqueue;
	struct selinfo sc_selinfo;

	struct capi_ai_softc *sc_next;
};

static	d_open_t	capi_open;
static	d_close_t	capi_close;
static	d_read_t	capi_read;
static	d_write_t	capi_write;
static	d_ioctl_t	capi_ioctl;
static	d_poll_t	capi_poll;

static cdevsw_t capi_cdevsw = {
#ifdef D_VERSION
      .d_version  = D_VERSION,
#endif
      .d_open     = capi_open,
      .d_close    = capi_close,
      .d_read     = capi_read,
      .d_write    = capi_write,
      .d_ioctl    = capi_ioctl,
      .d_poll     = capi_poll,
      .d_name     = "capi",
};

#define DEV2SC(dev) (*((struct capi_ai_softc **)&((dev)->si_drv1)))

static struct capi_ai_softc *capi_ai_sc_root = NULL;

static struct cdev *capi_dev = NULL;

static eventhandler_tag capi_clone_tag = NULL;

static u_int32_t capi_units = 0;

#define CAPINAME "capi20"

static struct capi_ai_softc *
capi_ai_get_closed_sc(void)
{
	struct capi_ai_softc *sc;
	u_int8_t free_sc = 0;
	u_int8_t link_sc = 0;
	u_int32_t unit;

	static u_int8_t flag = 0;

	mtx_lock(&i4b_global_lock);

	while(flag)
	{
		flag |= 2;
		(void) msleep(&flag, &i4b_global_lock, 
		       PZERO, "CAPI AI create unit", 0);
	}

	flag = 1;

	sc = capi_ai_sc_root;

	unit = capi_units;

	mtx_unlock(&i4b_global_lock);


	/* first search for an existing closed device */

	while(sc)
	{
	    u_int8_t closed;

	    mtx_lock(&sc->sc_mtx);

	    closed = !(sc->sc_flags & (ST_OPEN|ST_CLOSING|ST_OPENING));

	    mtx_unlock(&sc->sc_mtx);

	    if(closed)
	    {
	        /* no need to create another unit */
		goto done;
	    }

	    sc = sc->sc_next;
	}


	/* check number of units */

	if(unit >= CAPI_APPLICATION_MAX)
	{
		goto done;
	}


	/* try to create another unit */

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK|M_ZERO);

	if(sc == NULL)
	{
		goto done;
	}

	/* setup softc */

#define IFQ_LIMIT_LOW (MAX_CONTROLLERS * MAX_CHANNELS * 16)
#define IFQ_LIMIT_HIGH (MAX_CONTROLLERS * MAX_CHANNELS * 20)

	sc->sc_rdqueue.ifq_maxlen = IFQ_LIMIT_HIGH; /* XXX dynamic ? */

	mtx_init(&sc->sc_mtx, "CAPI AI", NULL, MTX_DEF | MTX_RECURSE);

	sc->sc_dev = 
	  make_dev(&capi_cdevsw, unit+1, UID_ROOT, GID_WHEEL, 0600, 
		   CAPINAME ".%03x", unit);

	if(sc->sc_dev == NULL)
	{
		free_sc = 1;
		goto done;
	}

	DEV2SC(sc->sc_dev) = sc;

	link_sc = 1;

 done:
	if(free_sc)
	{
	    if(mtx_initialized(&sc->sc_mtx))
	    {
	        mtx_destroy(&sc->sc_mtx);
	    }

	    free(sc, M_DEVBUF);
	    sc = NULL;
	}

	mtx_lock(&i4b_global_lock);

	if(link_sc)
	{
	    /* insert "sc" into list */

	    sc->sc_next = capi_ai_sc_root;
	    capi_ai_sc_root = sc;
	    capi_units ++;
	}

	if(flag & 2)
	{
		wakeup(&flag);
	}
	flag = 0;

	if(sc)
	{
	    /* exit with "sc->sc_mtx" locked
	     * (need to do the locking here
	     *  to avoid locking order 
	     *  reversal later)
	     */
	    mtx_lock(&sc->sc_mtx);
	}

	mtx_unlock(&i4b_global_lock);

	return sc;
}

#if ((__FreeBSD_version >= 700001) || (__FreeBSD_version == 0))
#define I4B_UCRED struct ucred *ucred,
#else
#define I4B_UCRED
#endif

static void
capi_clone(void *arg, I4B_UCRED char *name, int namelen, struct cdev **dev)
{
	struct capi_ai_softc *sc;

        if(dev[0] != NULL)
	{
		return;
	}

        if(strcmp(name, CAPINAME) != 0)
	{
		return;
        }

#if ((__FreeBSD_version >= 700001) || (__FreeBSD_version == 0))
	if(suser_cred(ucred,0))
	{
		return;
	}
#endif
  	sc = capi_ai_get_closed_sc();

	if(sc == NULL)
	{
		return;
	}

	/* this flag is set to prevent
	 * too early recycling of this
	 * device, and is cleared by
	 * "capi_open()":
	 */
	sc->sc_flags |= ST_OPENING;

	dev[0] = sc->sc_dev;

	mtx_unlock(&sc->sc_mtx);

	dev_ref(dev[0]);
#if 0
	dev[0]->si_flags |= SI_CHEAPCLONE;
#endif
	return;
}

/*---------------------------------------------------------------------------*
 *	interface attach routine
 *---------------------------------------------------------------------------*/
static void
capi_ai_attach(void *dummy)
{
	/* check type of "capi_clone()": */
	dev_clone_fn capi_clone_ptr = &capi_clone;

	capi_clone_tag = EVENTHANDLER_REGISTER(dev_clone, capi_clone_ptr, 0, 1000);

	if(capi_clone_tag == NULL)
	{
	    printf("%s: %s: failed to register \"dev_clone\", "
		   "continuing!\n", __FILE__, __FUNCTION__);
	}

	/* make a device so that we are visible
	 * (some space is added to the device name 
	 *  so that this device does not conflict
	 *  with the cloning process)
	 */
	capi_dev = 
	  make_dev(&capi_cdevsw, 0, UID_ROOT, GID_WHEEL, 0600, CAPINAME " ");

	if(capi_dev == NULL)
	{
		printf("%s: %s: failed to make dummy device, "
		       "continuing!\n", __FILE__, __FUNCTION__);
	}
#if 0
	else 
	  DEV2SC(capi_dev) = NULL;
#endif
	printf("capi: CAPI call control device attached, v%d.%02d\n",
	       (CAPI_STACK_VERSION / 100), (CAPI_STACK_VERSION % 100));
	return;
}
SYSINIT(capi_ai_attach, SI_SUB_PSEUDO, SI_ORDER_ANY, capi_ai_attach, NULL);

/*---------------------------------------------------------------------------*
 *	capi_ai_putqueue - put message into application interface queue(s)
 *
 * NOTE: if "m1 == NULL" this is an indication that the system has lost
 *       an important CAPI message
 *---------------------------------------------------------------------------*/
static void
capi_ai_putqueue(struct capi_ai_softc *sc, 
		 u_int8_t sc_complement, struct mbuf *m1)
{
	struct capi_ai_softc *sc_exclude;
	struct capi_message_encoded *mp;
	struct mbuf *m2;

	if((sc == NULL) || (sc_complement))
	{
		/* broadcast 
		 *
		 * NOTE: the softc structures are
		 * never unlinked, so one only needs
		 * a valid starting point:
		 */
		sc_exclude = sc;

		mtx_lock(&i4b_global_lock);
		sc = capi_ai_sc_root;
		mtx_unlock(&i4b_global_lock);

		while(sc)
		{
			if(sc != sc_exclude)
			{
				/* m_copypacket() is used hence writeable
				 * copies are not required. This means that
				 * data pointed to by m_data can be shared.
				 * Else m_dup() must be used.
				 */
				if((m1 == NULL) || 
				   (sc->sc_next == NULL))
				{
				    /* save a copypacket call */
				    m2 = m1;
				    m1 = NULL;
				}
				else
				{
				    m2 = m_copypacket(m1, M_DONTWAIT);
				}

				capi_ai_putqueue(sc, 0, m2);
			}
			sc = sc->sc_next;
		}
	}
	else
	{
		mtx_lock(&sc->sc_mtx);

		if(!(sc->sc_flags & ST_OPEN))
		{
		    goto done;
		}

		/* check that there is an mbuf and
		 * that the queue is not full
		 */
		if((m1 == NULL) || 
		   (_IF_QLEN(&sc->sc_rdqueue) >= IFQ_LIMIT_HIGH))
		{
		    if(!(sc->sc_flags & ST_MBUF_LOST))
		    {
		        sc->sc_flags |= ST_MBUF_LOST;

			NDBGL4(L4_ERR, "Unrecoverable data loss!");
		    }
		    goto done;
		}

		mp = (void *)(m1->m_data);

		/* filter connect indications */

		if(mp->head.wCmd == htole16(CAPI_IND(CONNECT)))
		{
		    u_int32_t cip = le16toh(mp->data.CONNECT_IND.wCIP);
		    u_int8_t controller = le32toh(mp->head.dwCid) & 0xFF;

		    if(cip < 32)
		      cip = (1 << cip) | 1;
		    else
		      cip = 1;

		    if(controller >= MAX_CONTROLLERS)
		    {
		        /* application does not want this message */
			goto done;
		    }

		    if(!(sc->sc_CIP_mask_1[controller] & cip))
		    {
		        /* application does not want this message */
			goto done;
		    }
		}

		/* filter information indications */

		if(mp->head.wCmd == htole16(CAPI_IND(INFO)))
		{
		    u_int8_t controller = le32toh(mp->head.dwCid) & 0xFF;

		    if(controller >= MAX_CONTROLLERS)
		    {
		        /* application does not want this message */
			goto done;
		    }

		    if(sc->sc_info_mask[controller] == 0)
		    {
			/* application does not want this message */
			goto done;
		    }
		}

		/* filter B-channel data indications */

		if(mp->head.wCmd == htole16(CAPI_IND(DATA_B3)))
		{
			if(_IF_QLEN(&sc->sc_rdqueue) >= IFQ_LIMIT_LOW)
			{
				/* data overflow */
				goto done;
			}
		}

		_IF_ENQUEUE(&sc->sc_rdqueue, m1);
		m1 = NULL;

		if(sc->sc_flags & ST_RD_SLEEP_WAKEUP)
		{
			sc->sc_flags &= ~ST_RD_SLEEP_WAKEUP;
			wakeup(&sc->sc_rdqueue);
		}

		if(sc->sc_flags & ST_SELECT)
		{
			sc->sc_flags &= ~ST_SELECT;
			selwakeup(&sc->sc_selinfo);
		}

	done:
		mtx_unlock(&sc->sc_mtx);
	}

	if(m1)
	{
		m_freem(m1);
	}
	return;
}

#define CAPI_ID_NCCI (1 << 16)
#define CAPI_ID2CONTROLLER(cid) ((cid) & 0x7F)
#define CAPI_ID2CDID(cid) (((cid) % MAX_CONTROLLERS) | \
			   ((((cid) >> 8) & 0xFF) * MAX_CONTROLLERS))
#define CDID2CAPI_ID(cdid) (((cdid) % MAX_CONTROLLERS) |  \
			    ((cdid / MAX_CONTROLLERS) << 8))

#if ((MAX_CONTROLLERS > 0x80) || \
     (MAX_CONTROLLERS == 0) || \
     (CDID_REF_MAX > 0x100) || \
     (CDID_REF_MAX == 0))
#error "cannot convert between CDID and CAPI_ID: overflow"
#endif

/*---------------------------------------------------------------------------*
 *	extract CAPI telephone number and presentation
 *---------------------------------------------------------------------------*/
static void
capi_get_telno(struct call_desc *cd, u_int8_t *src, u_int16_t len, 
	       u_int8_t *dst, u_int16_t max_length, u_int8_t flag)
{
	if(flag & 1)
	{
		cd->scr_ind = SCR_NONE;
		cd->prs_ind = PRS_NONE;
	}

	if(len)
	{
		if((*src != 0x80) && (*src != 0x00))
		{
			NDBGL4(L4_MSG, "cdid=%d: unknown numbering "
			       "plan=0x%02x, flag=%d (ignored)",
			       cd->cdid, *src, flag);
		}
		src++;
		len--;

		if(len && (flag & 1))
		{
		    if(*src == 0x80)
		    {
		        cd->prs_ind = PRS_ALLOWED;
		    }
		    else
		    {
		        cd->prs_ind = PRS_RESTRICT;
		    }
		    /* cd->scr_ind = ?? */
		    src++;
		    len--;
		}
	}

	if(len > max_length)
	{
		NDBGL4(L4_ERR, "cdid=%d: truncating telephone "
		       "number from %d to %d bytes, flag=%d", 
		       cd->cdid, flag, len, max_length);
		len = max_length;
	}
	bcopy(src, dst, len);
	dst[len] = '\0'; /* zero terminate string ! */
	return;
}

/*---------------------------------------------------------------------------*
 *	compile CAPI telephone number and presentation
 *---------------------------------------------------------------------------*/
static u_int16_t
capi_put_telno(struct call_desc *cd, u_int8_t *src, u_int8_t *dst, 
	       u_int16_t len, u_int8_t type, u_int8_t flag)
{
	u_int8_t *dst_end;

	if(len < 2)
	{
	    /* invalid length */
	    return 0;
	}

	dst_end = dst + len;

	*dst++ = type; /* number PLAN */

	if(flag & 1)
	{
	    /* add presentation and screening indicator */
	    *dst++ = (cd->prs_ind == PRS_RESTRICT) ? 0xA0 : 0x80;
	}

	while(*src)
	{
	  if(dst >= dst_end)
	  {
	      NDBGL4(L4_ERR, "cdid=%d: truncating telephone "
		     "number to %d bytes!", 
		     cd->cdid, len);
	      break;
	  }
	  *dst++ = *src++;
	}
	return len - (dst_end - dst);
}

/*---------------------------------------------------------------------------*
 *	generate DATA-B3 confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_b3_conf(struct capi_message_encoded *msg, u_int16_t wDataHandle, 
		  u_int16_t wStatus)
{
	struct mbuf *m;
	struct {
	  /* XXX this structure is 
	   * hardcoded to save some CPU
	   */
	  struct CAPI_HEADER_ENCODED head;

	  u_int16_t wDataHandle;
	  u_int16_t wStatus;
	} __packed *mp;

	m = i4b_getmbuf(sizeof(*mp), M_NOWAIT);

	if(m)
	{
		mp = (void *)(m->m_data);

		mp->head.wLen = htole16(sizeof(*mp));
		mp->head.wApp = htole16(0);
		mp->head.wCmd = htole16(CAPI_CONF(DATA_B3));
		mp->head.wNum = htole16(msg->head.wNum);
		mp->head.dwCid = htole32(msg->head.dwCid);

		mp->wDataHandle = htole16(wDataHandle);
		mp->wStatus = htole16(wStatus);
	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	generate generic CAPI confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_conf(struct capi_message_encoded *msg, u_int16_t wCmd, 
		  u_int16_t wStatus)
{
	struct mbuf *m;
	struct {
	  /* XXX this structure is 
	   * hardcoded to save some CPU
	   */
	  struct CAPI_HEADER_ENCODED head;

	  u_int16_t wStatus;
	} __packed *mp;

	m = i4b_getmbuf(sizeof(*mp), M_NOWAIT);

	if(m)
	{
		mp = (void *)(m->m_data);

		mp->head.wLen = htole16(sizeof(*mp));
		mp->head.wApp = htole16(0);
		mp->head.wCmd = htole16(wCmd);
		mp->head.wNum = htole16(msg->head.wNum);
		mp->head.dwCid = htole32(msg->head.dwCid);

		mp->wStatus = htole16(wStatus);
	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	generate facility confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_facility_conf(struct capi_message_encoded *pmsg, 
			u_int16_t wSelector, 
			u_int16_t wInfo)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_FACILITY_CONF_DECODED fac_conf = { /* zero */ };

	u_int16_t len;

	CAPI_INIT(CAPI_FACILITY_CONF, &fac_conf);

	fac_conf.wInfo = htole16(wInfo);
	fac_conf.wSelector = htole16(wSelector);

	len = capi_encode(&msg.data, sizeof(msg.data), &fac_conf);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_CONF(FACILITY));
	msg.head.wNum = htole16(pmsg->head.wNum);
	msg.head.dwCid = htole32(pmsg->head.dwCid);

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	make CAPI connect B-channel active indication
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_connect_b3_active_ind(struct call_desc *cd)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_CONNECT_B3_ACTIVE_IND_DECODED connect_b3_active_ind = { /* zero */ };

	u_int16_t len;

	CAPI_INIT(CAPI_CONNECT_B3_ACTIVE_IND, &connect_b3_active_ind);

	len = capi_encode(&msg.data, sizeof(msg.data), &connect_b3_active_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(CONNECT_B3_ACTIVE));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid)|CAPI_ID_NCCI);

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	make CAPI connect B-channel indication
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_connect_b3_ind(struct call_desc *cd)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_CONNECT_B3_IND_DECODED connect_b3_ind = { /* zero */ };

	u_int16_t len;

	CAPI_INIT(CAPI_CONNECT_B3_IND, &connect_b3_ind);

	len = capi_encode(&msg.data, sizeof(msg.data), &connect_b3_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(CONNECT_B3));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid)|CAPI_ID_NCCI);

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}
	return m;
}

/*---------------------------------------------------------------------------*
 *	send CAPI connect indication
 *---------------------------------------------------------------------------*/
void
capi_ai_info_ind(struct call_desc *cd, u_int8_t complement, 
		 u_int16_t wInfoNumber, void *ptr, u_int16_t len)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_INFO_IND_DECODED info_ind = { /* zero */ };

	__KASSERT(((cd->ai_ptr == NULL) || 
		 (cd->ai_type == I4B_AI_CAPI) || complement), 
		("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	CAPI_INIT(CAPI_INFO_IND, &info_ind);

	info_ind.wInfoNum = wInfoNumber;
	info_ind.InfoElement.ptr = ptr;
	info_ind.InfoElement.len = len;

	len = capi_encode(&msg.data, sizeof(msg.data), &info_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(INFO));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid));

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}

	capi_ai_putqueue(cd->ai_ptr,complement,m);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI connect indication
 *
 * NOTE: the connect indication can be replayed multiple
 * times using the same cdid with updated information !
 *---------------------------------------------------------------------------*/
void
capi_ai_connect_ind(struct call_desc *cd)
{
	struct mbuf *m;
	u_int16_t len;

	struct capi_message_encoded msg;
	struct CAPI_CONNECT_IND_DECODED connect_ind;
	struct CAPI_ADDITIONAL_INFO_DECODED add_info;

	u_int8_t dst_telno[TELNO_MAX];
	u_int8_t src_telno[TELNO_MAX];
	u_int8_t dst_subaddr[SUBADDR_MAX];
	u_int8_t src_subaddr[SUBADDR_MAX];

	static const u_int8_t bc_bprot_none[] = { 0x04, 0x03, 0x80, 0x90, 0xA3 };
	static const u_int8_t bc_bprot_rhdlc[] = { 0x04, 0x02, 0x88, 0x90 };
	static const u_int8_t hlc_bprot_none[] = { 0x7D, 0x02, 0x91, 0x81 };

	__KASSERT(((cd->ai_ptr == NULL) || 
		   (cd->ai_type == I4B_AI_CAPI)), 
		  ("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	if((cd->channel_id == CHAN_ANY) || 
	   (cd->channel_id == CHAN_NOT_ANY))
	{
	    /* XXX just hold this back
	     * until further
	     */
	    return;
	}

	bzero(&connect_ind, sizeof(connect_ind));
	bzero(&add_info, sizeof(add_info));

	CAPI_INIT(CAPI_CONNECT_IND, &connect_ind);
	CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);

	switch(cd->channel_bprot) {
	case BPROT_NONE:
	case BPROT_RHDLC_DOV:
	  connect_ind.wCIP = CAPI_CIP_TELEPHONY;
	  connect_ind.BC.ptr = &bc_bprot_none;
	  connect_ind.BC.len = sizeof(bc_bprot_none);
	  connect_ind.HLC.ptr = &hlc_bprot_none;
	  connect_ind.HLC.len = sizeof(hlc_bprot_none);
	  break;

	default:
	  NDBGL4(L4_ERR, "cdid=%d: unknown bprot=%d, "
		 "fallback to CIP_UNRESTRICTED_DATA", 
		 cd->cdid, cd->channel_bprot);
	case BPROT_RHDLC:
	case BPROT_NONE_VOD:
	  connect_ind.wCIP = CAPI_CIP_UNRESTRICTED_DATA;
	  connect_ind.BC.ptr = &bc_bprot_rhdlc;
	  connect_ind.BC.len = sizeof(bc_bprot_rhdlc);
	  break;
	}

	connect_ind.dst_telno.ptr = &dst_telno[0];
	connect_ind.dst_telno.len = capi_put_telno
	  (cd, &cd->dst_telno[0], &dst_telno[0], 
	   sizeof(dst_telno), 0x80, 0);

	connect_ind.src_telno.ptr = &src_telno[0];
	connect_ind.src_telno.len = capi_put_telno
	  (cd, &cd->src_telno[0], &src_telno[0], 
	   sizeof(src_telno), 0x00, 1);

	connect_ind.dst_subaddr.ptr = &dst_subaddr[0];
	connect_ind.dst_subaddr.len = capi_put_telno
	  (cd, &cd->dst_subaddr[0], &dst_subaddr[0], 
	   sizeof(dst_subaddr), 0x80, 0);

	connect_ind.src_subaddr.ptr = &src_subaddr[0];
	connect_ind.src_subaddr.len = capi_put_telno
	  (cd, &cd->src_subaddr[0], &src_subaddr[0], 
	   sizeof(src_subaddr), 0x80, 0);

	len = capi_encode(&msg.data, sizeof(msg.data), &connect_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(CONNECT));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid));

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}

	capi_ai_putqueue(cd->ai_ptr,0,m);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI connect active indication
 *---------------------------------------------------------------------------*/
void
capi_ai_connect_active_ind(struct call_desc *cd)
{
	struct mbuf *m;

	u_int16_t len;

	struct capi_message_encoded msg;
	struct CAPI_CONNECT_ACTIVE_IND_DECODED connect_active_ind = { /* zero */ };

	__KASSERT(((cd->ai_ptr == NULL) || 
		 (cd->ai_type == I4B_AI_CAPI)), 
		("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	CAPI_INIT(CAPI_CONNECT_ACTIVE_IND, &connect_active_ind);

	len = capi_encode(&msg.data, sizeof(msg.data), &connect_active_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(CONNECT_ACTIVE));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid));

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}

	capi_ai_putqueue(cd->ai_ptr,0,m);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI B-channel disconnect indication
 *---------------------------------------------------------------------------*/
void
capi_ai_disconnect_b3_ind(struct call_desc *cd)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_DISCONNECT_B3_IND_DECODED disconnect_b3_ind = { /* zero */ };

	u_int16_t len;

	__KASSERT(((cd->ai_ptr == NULL) || 
		   (cd->ai_type == I4B_AI_CAPI)), 
		  ("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	CAPI_INIT(CAPI_DISCONNECT_B3_IND, &disconnect_b3_ind);

	disconnect_b3_ind.wReason = 0; /* clearing in accordance with protocol */

	len = capi_encode(&msg.data, sizeof(msg.data), &disconnect_b3_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(DISCONNECT_B3));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid)|CAPI_ID_NCCI);

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}

	capi_ai_putqueue(cd->ai_ptr,0,m);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI disconnect indication
 *---------------------------------------------------------------------------*/
void
capi_ai_disconnect_ind(struct call_desc *cd, u_int8_t complement)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_DISCONNECT_IND_DECODED disconnect_ind = { /* zero */ };

	u_int16_t len;

	__KASSERT(((cd->ai_ptr == NULL) || 
		 (cd->ai_type == I4B_AI_CAPI) || complement), 
		("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	CAPI_INIT(CAPI_DISCONNECT_IND, &disconnect_ind);

	disconnect_ind.wReason = 0x3400 | i4b_make_q850_cause(cd->cause_in);

	len = capi_encode(&msg.data, sizeof(msg.data), &disconnect_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(DISCONNECT));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid));

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}

	capi_ai_putqueue(cd->ai_ptr,complement,m);
	return;
}


/*---------------------------------------------------------------------------*
 *	capi_open - device driver open routine
 *---------------------------------------------------------------------------*/
static int
capi_open(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	struct capi_ai_softc *sc = DEV2SC(dev);
	int error = 0;

	if(sc == NULL)
	{
		return ENXIO;
	}

	mtx_lock(&sc->sc_mtx);
	if(sc->sc_flags & (ST_OPEN|ST_CLOSING))
	{
		error = EBUSY;
		goto done;
	}
	sc->sc_flags |= (ST_OPEN|ST_BLOCK);
	sc->sc_flags &= ~(ST_OPENING|ST_MBUF_LOST);

	bzero(&sc->sc_info_mask, sizeof(sc->sc_info_mask)); /* disable all information */
	bzero(&sc->sc_CIP_mask_1, sizeof(sc->sc_CIP_mask_1)); /* disable incoming calls */
	bzero(&sc->sc_CIP_mask_2, sizeof(sc->sc_CIP_mask_2)); /* disable incoming calls */

	sc->sc_max_b_data_len = BCH_MAX_DATALEN; /* set default receive length */

 done:
	mtx_unlock(&sc->sc_mtx);

	return error;
}

/*---------------------------------------------------------------------------*
 *	capi_close - device driver close routine
 *---------------------------------------------------------------------------*/
static int
capi_close(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	struct capi_ai_softc *sc = DEV2SC(dev);
	u_int8_t d_open;

	if(sc == NULL)
	{
		return ENXIO;
	}

	mtx_lock(&sc->sc_mtx);

	if((sc->sc_flags & ST_OPEN) &&
	   (!(sc->sc_flags & ST_CLOSING)))
	{
		sc->sc_flags |= ST_CLOSING;

		while(sc->sc_flags & (ST_RD_SLEEP_WAKEUP|
				      ST_RD_SLEEP_ENTERED|
				      ST_WR_SLEEP_WAKEUP|
				      ST_WR_SLEEP_ENTERED|
				      ST_IOCTL|
				      ST_OPENING))
		{
			if(sc->sc_flags & ST_RD_SLEEP_WAKEUP)
			{
				sc->sc_flags &= ~ST_RD_SLEEP_WAKEUP;
				wakeup(&sc->sc_rdqueue);
			}
#if 0
			if(sc->sc_flags & ST_WR_SLEEP_WAKEUP)
			{
				sc->sc_flags &= ~ST_WR_SLEEP_WAKEUP;
				wakeup(& ??? );
			}
#endif
			(void) msleep(sc, &sc->sc_mtx, PZERO,
				      "CAPI AI closing", 0);
		}

		d_open = ((sc->sc_flags & ST_D_OPEN) != 0);

		mtx_unlock(&sc->sc_mtx);

		/* disconnect any active calls on this
		 * application interface
		 */
		i4b_disconnect_by_appl_interface(I4B_AI_CAPI, sc);

		if(d_open)
		{
		    /* close D-channels */
		    i4b_update_all_d_channels(0);
		}

		/* release all reserved drivers */
		i4b_release_drivers_by_appl_interface(I4B_AI_CAPI, sc);

		mtx_lock(&sc->sc_mtx);

		/* free memory last */
		_IF_DRAIN(&sc->sc_rdqueue);

		/* clear flags last */
		sc->sc_flags &= ~(ST_OPEN|ST_CLOSING|ST_D_OPEN);
	}

	mtx_unlock(&sc->sc_mtx);

	return(0);
}

/*---------------------------------------------------------------------------*
 *	capi_read - device driver read routine
 *---------------------------------------------------------------------------*/
static int
capi_read(struct cdev *dev, struct uio *uio, int flag)
{
	struct capi_ai_softc *sc = DEV2SC(dev);
	struct mbuf *m1;
	struct mbuf *m2;
	int error = 0;

	if(sc == NULL)
	{
		return ENXIO;
	}

	mtx_lock(&sc->sc_mtx);

	if(sc->sc_flags & (ST_RD_SLEEP_ENTERED|ST_CLOSING|ST_MBUF_LOST))
	{
		/* only one thread at a time */
		mtx_unlock(&sc->sc_mtx);
		return EBUSY;
	}

	while(_IF_QEMPTY(&sc->sc_rdqueue))
	{
	    if(sc->sc_flags & ST_BLOCK)
	    {
		sc->sc_flags |= (ST_RD_SLEEP_WAKEUP|ST_RD_SLEEP_ENTERED);

		error = msleep(&sc->sc_rdqueue, &sc->sc_mtx,
			       (PZERO + 1) | PCATCH, "CAPI AI wait data", 0);

		sc->sc_flags &= ~(ST_RD_SLEEP_WAKEUP|ST_RD_SLEEP_ENTERED);

		if(sc->sc_flags & ST_CLOSING)
		{
			wakeup(sc);
			error = EIO;
		}
	    }
	    else
	    {
		error = EWOULDBLOCK;
	    }

	    if(error)
	    {
		mtx_unlock(&sc->sc_mtx);
		return error;
	    }
	}
	_IF_DEQUEUE(&sc->sc_rdqueue, m1);
	mtx_unlock(&sc->sc_mtx);

	if(m1 && m1->m_len)
	{
		m2 = m1;
		do {
		  error = uiomove(m2->m_data, m2->m_len, uio);
		} while((m2 = m2->m_next) && !error);
	}
	else
	{
		error = EIO;
	}

	if(m1)
	{
		m_freem(m1);
	}
	return(error);
}

/*---------------------------------------------------------------------------*
 *	capi_write - device driver write routine
 *---------------------------------------------------------------------------*/
static int
capi_write(struct cdev *dev, struct uio * uio, int flag)
{
	struct capi_ai_softc *sc = DEV2SC(dev);
	struct i4b_controller *cntl;
	struct call_desc *cd;
	struct mbuf *m1 = NULL;
	struct mbuf *m2;
	int error;

	u_int8_t response;
	u_int16_t cause;

	struct CAPI_ALERT_REQ_DECODED alert_req;
	struct CAPI_CONNECT_REQ_DECODED connect_req;
	struct CAPI_DATA_B3_REQ_DECODED data_b3_req;
	struct CAPI_INFO_REQ_DECODED info_req;
	struct CAPI_CONNECT_RESP_DECODED connect_resp;
	struct CAPI_CONNECT_B3_RESP_DECODED connect_b3_resp;
	struct CAPI_LISTEN_REQ_DECODED listen_req;
	struct CAPI_SELECT_B_PROTOCOL_REQ_DECODED select_b_protocol_req;
	struct CAPI_FACILITY_REQ_DECODED facility_req;

	struct CAPI_ADDITIONAL_INFO_DECODED add_info;
	struct CAPI_SENDING_COMPLETE_DECODED sending_complete;
	struct CAPI_B_PROTOCOL_DECODED b_protocol;

	struct capi_message_encoded msg;

	/* NOTE: one has got to read all data into 
	 * buffers before locking the controller,
	 * hence uiomove() can sleep
	 */
	if(sc == NULL)
	{
		error = ENXIO;
		goto done;
	}

	if(uio->uio_resid < sizeof(msg.head))
	{
		error = EINVAL;
		goto done;
	}

	error = uiomove(&msg.head, sizeof(msg.head), uio);

	if(error)
	{
		goto done;
	}

	/* convert byte order in header to host format */

	msg.head.wLen = le16toh(msg.head.wLen);
	msg.head.wApp = le16toh(msg.head.wApp);
	msg.head.wCmd = le16toh(msg.head.wCmd);
	msg.head.wNum = le16toh(msg.head.wNum);
	msg.head.dwCid = le32toh(msg.head.dwCid);

	/* pack the command */
	msg.head.wCmd = CAPI_COMMAND_PACK(msg.head.wCmd);

	/* verify length */

	if(msg.head.wLen < sizeof(msg.head))
	{
		error = EINVAL;
		goto done;
	}

	/* remove header from length */

	msg.head.wLen -= sizeof(msg.head);

	/* verify length */

	if((msg.head.wLen > uio->uio_resid) ||
	   (msg.head.wLen > sizeof(msg.data)))
	{
		error = ENOMEM;
		goto done;
	}

	/* get rest of header into buffer */

	error = uiomove(&msg.data, msg.head.wLen, uio);

	if(error)
	{
		goto done;
	}

	/* rest of data goes into a mbuf, if any.
	 * This implementation allows zero length
	 * frames in case of DATA-B3 request.
	 */
	if(msg.head.wCmd == CAPI_P_REQ(DATA_B3))
	{
		m1 = i4b_getmbuf(uio->uio_resid, M_WAITOK);

		if(m1 == NULL)
		{
			error = ENOMEM;
			goto done;
		}

		error = uiomove(m1->m_data, m1->m_len, uio);

		if(error)
		{
			goto done;
		}
	}
	else
	{
		m1 = NULL;
	}

	cntl = CNTL_FIND(CAPI_ID2CONTROLLER(msg.head.dwCid));

	if(cntl == NULL)
	{
		error = EINVAL;
		goto done;
	}

	mtx_lock(&sc->sc_mtx);

	if(sc->sc_flags & (ST_WR_SLEEP_ENTERED|ST_CLOSING))
	{
		/* only one thread at a time */
		mtx_unlock(&sc->sc_mtx);
		error = EBUSY;
		goto done;
	}

	sc->sc_flags |= ST_WR_SLEEP_ENTERED;

	mtx_unlock(&sc->sc_mtx);

	cd = NULL;

	CNTL_LOCK(cntl);

	if(cntl->N_fifo_translator)
	{
	    /* connected */
	    cd = cd_by_cdid(cntl, CAPI_ID2CDID(msg.head.dwCid));

	    if(!cd && (msg.head.wCmd == CAPI_P_REQ(CONNECT)))
	    {
	        /* allocate a new call-descriptor
		 * for an outgoing call
		 *
		 * cd's are allocated from a controller,
		 * because of locking
		 */
	        cd = N_ALLOCATE_CD(cntl,NULL,0,I4B_AI_CAPI,sc);
	    }
	}

	if(cd)
	{
	  /* check the call descriptor's 
	   * application interface before issuing
	   * any commands
	   */
	  if((cd->ai_type == I4B_AI_BROADCAST) ||
	     ((cd->ai_type == I4B_AI_CAPI) && (cd->ai_ptr == ((void *)sc))))
	  {
	    switch(msg.head.wCmd) {

	      /* send ALERT request */

	    case CAPI_P_REQ(ALERT): /* ================================ */

	      m2 = capi_make_conf(&msg, CAPI_CONF(ALERT), 0x0000);

	      if(m2)
	      {
		  CAPI_INIT(CAPI_ALERT_REQ, &alert_req);

		  capi_decode(&msg.data, msg.head.wLen, &alert_req);

		  CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);

		  capi_decode(alert_req.add_info.ptr,
			      alert_req.add_info.len, &add_info);

		  CAPI_INIT(CAPI_SENDING_COMPLETE, &sending_complete);

		  capi_decode(add_info.sending_complete.ptr,
			      add_info.sending_complete.len, 
			      &sending_complete);
#if 0
		  if(sending_complete.wMode == 1)
		  {
		      send sending complete or call proceeding;

		      N_ALERT_REQUEST will send call proceeding
		      before alert, in case of overlap sending
		  }
#endif
		  N_ALERT_REQUEST(cd);
	      }
	      goto send_confirmation;

	      /* connect request, dial out to remote */

	    case CAPI_P_REQ(CONNECT): /* ============================== */

	      /* update CID value first */

	      msg.head.dwCid = CDID2CAPI_ID(cd->cdid);

	      m2 = capi_make_conf(&msg, CAPI_CONF(CONNECT), 0x0000);

	      capi_ai_putqueue(sc, 0, m2);

	      if(m2 == NULL) break;

	      CAPI_INIT(CAPI_CONNECT_REQ, &connect_req);

	      capi_decode(&msg.data, msg.head.wLen, &connect_req);

	      CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);

	      capi_decode(connect_req.add_info.ptr,
			  connect_req.add_info.len, 
			  &add_info);

	      switch(connect_req.wCIP) {
	      case CAPI_CIP_SPEECH:
	      case CAPI_CIP_3100Hz_AUDIO:
	      case CAPI_CIP_TELEPHONY:
		cd->channel_bprot = BPROT_NONE;
		break;
      
	      default:
		NDBGL4(L4_ERR, "cdid=%d, unknown CIP "
		       "value=0x%04x, fallback to "
		       "raw HDLC", cd->cdid, 
		       connect_req.wCIP);

	      case CAPI_CIP_UNRESTRICTED_DATA:
		cd->channel_bprot = BPROT_RHDLC;
		break;
	      }

	      cd->driver_type = DRVR_CAPI_B3;
	      cd->driver_unit = 0;

	      cd->shorthold_data.shorthold_algorithm = SHA_FIXU;
	      cd->shorthold_data.unitlen_time = 60; /* seconds */
	      cd->shorthold_data.idle_time = 0; /* seconds (disabled) */
	      cd->shorthold_data.earlyhup_time = 0; /* seconds (disabled) */

	      cd->last_aocd_time = 0;
	      cd->aocd_flag = 0;

	      cd->cunits = 0;

	      capi_get_telno
		(cd, 
		 connect_req.dst_telno.ptr,
		 connect_req.dst_telno.len,
		 &cd->dst_telno[0], sizeof(cd->dst_telno)-1, 0);

	      capi_get_telno
		(cd,
		 connect_req.src_telno.ptr,
		 connect_req.src_telno.len,
		 &cd->src_telno[0], sizeof(cd->src_telno)-1, 1);

	      capi_get_telno
		(cd,
		 connect_req.dst_subaddr.ptr,
		 connect_req.dst_subaddr.len,
		 &cd->dst_subaddr[0], sizeof(cd->dst_subaddr)-1, 0);

	      capi_get_telno
		(cd,
		 connect_req.src_subaddr.ptr,
		 connect_req.src_subaddr.len,
		 &cd->src_subaddr[0], sizeof(cd->src_subaddr)-1, 0);

	      /* XXX if there is user-user info or a keypad-string
	       * in "add_info" one could have added that to 
	       * "cd->keypad[] and cd->sms[]"
	       */
	      cd->keypad[0] = '\0';
	      cd->sms[0] = '\0';
	      cd->display[0] = '\0';

	      SET_CAUSE_TYPE(cd->cause_in, CAUSET_I4B);
	      SET_CAUSE_VAL(cd->cause_in, CAUSE_I4B_NORMAL);

	      if(connect_req.b_protocol.len)
	      {
		  CAPI_INIT(CAPI_B_PROTOCOL, &b_protocol);

		  capi_decode(connect_req.b_protocol.ptr,
			      connect_req.b_protocol.len, 
			      &b_protocol);

		  if((b_protocol.wB1_protocol == 0) &&
		     (cd->channel_bprot == BPROT_NONE))
		  {
		      /* data over voice */
		      cd->channel_bprot = BPROT_RHDLC_DOV;
		  }

		  if((b_protocol.wB1_protocol == 1) &&
		     (cd->channel_bprot == BPROT_RHDLC))
		  {
		      /* voice over data */
		      cd->channel_bprot = BPROT_NONE_VOD;
		  }

		  if(b_protocol.wB1_protocol >= 2)
		  {
		      NDBGL4(L4_ERR, "cdid=%d: unsupported "
			     "B1 protocol=%d ignored!",
			     cd->cdid, b_protocol.wB1_protocol);
		  }

		  if(b_protocol.wB2_protocol != 1)
		  {
		      NDBGL4(L4_ERR, "cdid=%d: unsupported "
			     "B2 protocol=%d ignored!",
			     cd->cdid, b_protocol.wB2_protocol);
		  }

		  if(b_protocol.wB3_protocol != 0)
		  {
		      NDBGL4(L4_ERR, "cdid=%d, unsupported "
			     "B3 protocol=%d ignored!",
			     cd->cdid, b_protocol.wB3_protocol);
		  }
	      }

	      if(cd->channel_allocated == 0)
	      {
		      cd->channel_id = CHAN_ANY;

		      if(cntl->N_nt_mode ||
			 /* channel specified */			     
			 ((cd->channel_id >= 0) &&
			  (cd->channel_id < cntl->L1_channel_end)) ||
			 /* channel-less */
			 (cd->channel_id == CHAN_NOT_ANY)
			 /* || no channel available */)
		      {
			  cd_allocate_channel(cd);

			  if(cd->channel_allocated == 0)
			  {
			      SET_CAUSE_VAL(cd->cause_in, CAUSE_I4B_NOCHAN);

			      /* NOTE: N_FREE_CD() will send
			       * a disconnect indication
			       */
			      N_FREE_CD(cd);
			      break;
			  }
		      }
	      }
	      cd->isdntxdelay = 0; /* seconds (disabled) */
	      N_CONNECT_REQUEST(cd);
	      break;


	    case CAPI_P_REQ(DATA_B3): /* ============================== */

	      CAPI_INIT(CAPI_DATA_B3_REQ, &data_b3_req);

	      capi_decode(&msg.data, msg.head.wLen, &data_b3_req);

	      /* check that the B-channel is connected */

	      if(cd->fifo_translator == NULL)
	      {
		  NDBGL4(L4_ERR, "cdid=%d: B-channel data sent "
			 "when disconnected!", cd->cdid);

		  m2 = capi_make_b3_conf
		    (&msg, data_b3_req.wHandle, 0x2001);
		  goto send_confirmation;
	      }

	      /* double check the frame length */

	      if(data_b3_req.wLen != m1->m_len)
	      {
		  NDBGL4(L4_ERR, "cdid=%d: invalid B-channel framelength: "
			 "%d <> %d!", cd->cdid, data_b3_req.wLen, 
			 m1->m_len);

		  m2 = capi_make_b3_conf
		    (&msg, data_b3_req.wHandle, 0x2007);
		  goto send_confirmation;
	      }

	      /* check queue length */

	      if(_IF_QFULL(&cd->fifo_translator->tx_queue))
	      {
		  m2 = capi_make_b3_conf
		    (&msg, data_b3_req.wHandle, 0x1008);
		  goto send_confirmation;
	      }

	      /* make confirmation message */

	      m2 = capi_make_b3_conf
		(&msg, data_b3_req.wHandle, 0x0000);

	      if(m2 == NULL)
	      {
		  error = ENOMEM;
		  break;
	      }

	      /* the user application(s) will send 
	       * more data when the data confirmation
	       * is received. Therefore it is important
	       * that one sends the confirmation when
	       * the hardware is actually out of data 
	       * and not before, to avoid flooding 
	       * the buffers.
	       */
	      m2->m_next = m1;
	      m1 = NULL;

	      _IF_ENQUEUE(&cd->fifo_translator->tx_queue, m2);

	      L1_FIFO_START(cd->fifo_translator);
	      break;


	    case CAPI_P_REQ(MANUFACTURER): /* ========================= */
	      /* nothing to do - not supported */
	      break;


	    case CAPI_P_REQ(CONNECT_B3): /* =========================== */

	      /* update CID value first */

	      msg.head.dwCid = CDID2CAPI_ID(cd->cdid)|CAPI_ID_NCCI;

	      m2 = capi_make_conf(&msg, CAPI_CONF(CONNECT_B3), 0x0000);
	      capi_ai_putqueue(sc, 0, m2);

	      m2 = capi_make_connect_b3_active_ind(cd);
	      goto send_confirmation;


	    case CAPI_P_REQ(DISCONNECT_B3): /* ======================== */

	      /* disconnect request, actively terminate connection */

	    case CAPI_P_REQ(DISCONNECT): /* =========================== */
#if 0
	      /* pretend that there was a disconnect collision, so
	       * that one does not have to send any confirmation 
	       * messages !
	       */
	      m2 = capi_make_conf(&msg, CAPI_CONF(DISCONNECT_B3), 0x0000);
	      m2 = capi_make_conf(&msg, CAPI_CONF(DISCONNECT), 0x0000);
	      capi_ai_disconnect_b3_ind(cd);
#endif
	      i4b_link_bchandrvr(cd, 0);

	      /* preset causes with our cause */

	      cd->cause_in = cd->cause_out = 
		(CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;

	      /*
	       * the following will always call
	       * "i4b_l4_disconnect_ind()", which
	       * will send the CAPI disconnect 
	       * indications
	       */
	      N_DISCONNECT_REQUEST(cd, cd->cause_in);

	      cd = NULL; /* call descriptor is freed ! */

	      break;


	    case CAPI_P_REQ(INFO): /* ============================== */

	      m2 = capi_make_conf(&msg, CAPI_CONF(INFO), 0x0000);
	      if(m2)
	      {
		  u_int8_t *src, *src_end, *dst, *dst_end;

		  CAPI_INIT(CAPI_INFO_REQ, &info_req);

		  capi_decode(&msg.data, msg.head.wLen, &info_req);

		  CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);

		  capi_decode(info_req.add_info.ptr,
			      info_req.add_info.len, &add_info);

		  src = info_req.dst_telno.ptr;
		  src_end = ADD_BYTES(info_req.dst_telno.ptr, info_req.dst_telno.len);
		  dst = &cd->dst_telno[0];
		  dst_end = &cd->dst_telno[TELNO_MAX-1];

		  if(src < src_end)
		  {
		      /* skip number plan byte */
		      src++;
		  }
		  
		  /* find end */
		  while((dst < dst_end) && *dst) dst++;

		  /* append */
		  while((src < src_end) && (dst < dst_end) && *src)
		  {
		      *dst++ = *src++;
		  }

		  *dst = '\0'; /* zero terminate string ! */

		  N_INFORMATION_REQUEST(cd);
	      }
	      goto send_confirmation;


	    case CAPI_P_REQ(RESET_B3): /* ============================= */

	      m2 = capi_make_conf(&msg, CAPI_CONF(RESET_B3), 0x300D);

	      /* nothing to do - not supported */

	      goto send_confirmation;


	    case CAPI_P_REQ(SELECT_B_PROTOCOL): /* ==================== */

	      CAPI_INIT(CAPI_SELECT_B_PROTOCOL_REQ, &select_b_protocol_req);

	      capi_decode(&msg.data, msg.head.wLen, &select_b_protocol_req);

	      CAPI_INIT(CAPI_B_PROTOCOL, &b_protocol);

	      capi_decode(select_b_protocol_req.b_protocol.ptr,
			  select_b_protocol_req.b_protocol.len, 
			  &b_protocol);

	      if(b_protocol.wB1_protocol >= 2)
	      {
		  m2 = capi_make_conf(&msg, CAPI_CONF(SELECT_B_PROTOCOL), 
				      0x3001);
		  goto send_confirmation;
	      }
 
	      if(b_protocol.wB2_protocol != 1)
	      {
		  m2 = capi_make_conf(&msg, CAPI_CONF(SELECT_B_PROTOCOL), 
				      0x3002);
		  goto send_confirmation;
	      }

	      if(b_protocol.wB3_protocol != 0)
	      {
		  m2 = capi_make_conf(&msg, CAPI_CONF(SELECT_B_PROTOCOL), 
				      0x3003);
		  goto send_confirmation;
	      }

	      if(b_protocol.wB1_protocol == 0)
	      {
		  /* data over voice */
		  cd->channel_bprot = BPROT_RHDLC;
	      }
	      else /* if(b_protocol.wB1_protocol == 1) */
	      {
		  /* voice over data */
		  cd->channel_bprot = BPROT_NONE;
	      }

	      m2 = capi_make_conf(&msg, CAPI_CONF(SELECT_B_PROTOCOL), 
				  0x0000);
	      goto send_confirmation;


	    case CAPI_P_REQ(FACILITY): /* ============================= */

	      CAPI_INIT(CAPI_FACILITY_REQ, &facility_req);

	      capi_decode(&msg.data, msg.head.wLen, &facility_req);

	      /* not supported */

	      m2 = capi_make_facility_conf
		(&msg, facility_req.wSelector, 0x300B);

	      goto send_confirmation;


	      /* connect response, accept/reject/ignore incoming call */

	    case CAPI_P_RESP(CONNECT): /* ============================= */

	      if(cd->dir_incoming == 0)
	      {
		  break;
	      }

	      CAPI_INIT(CAPI_CONNECT_RESP, &connect_resp);

	      capi_decode(&msg.data, msg.head.wLen, &connect_resp);

	      if(connect_resp.wReject == 0)
	      {
			/* Accept call */
			cause = (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;
			response = SETUP_RESP_ACCEPT;
	      }
	      else if(connect_resp.wReject == 1)
	      {
			/* this response is not supported
			 * when there are multiple user
			 * interfaces, hence it will
			 * prevent other application interfaces
			 * from answering the call
			 */
			break;
	      }
	      else if((connect_resp.wReject >= 2) &&
		      (connect_resp.wReject <= 8))
	      {
		static const u_int8_t table[9] = 
		  { [0] = CAUSE_I4B_NORMAL, 
		    [1] = CAUSE_I4B_NORMAL, 
		    [2] = CAUSE_I4B_NORMAL,  /* Reject call, normal call clearing */
		    [3] = CAUSE_I4B_BUSY,    /* Reject call, user busy */
		    [4] = CAUSE_I4B_NOCHAN,  /* Reject call, requested circuit/channel not available */
		    [5] = CAUSE_I4B_REJECT,  /* Reject call, facility rejected */
		    [6] = CAUSE_I4B_NOCHAN,  /* Reject call, channel unacceptable */
		    [7] = CAUSE_I4B_INCOMP,  /* Reject call, incompatible destination */
		    [8] = CAUSE_I4B_OOO,     /* Reject call, destination out of order */
		  };

		cause = (CAUSET_I4B << 8) | table[connect_resp.wReject];
		response = SETUP_RESP_REJECT;
	      }
	      else if((connect_resp.wReject & 0xFF00) == 0x3400)
	      {
			cause = 
			  (CAUSET_Q850 << 8) | 
			  (connect_resp.wReject & 0x7F);
			response = SETUP_RESP_REJECT;
	      }
	      else
	      {
			cause = (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;
			response = SETUP_RESP_REJECT;
	      }

	      if(connect_resp.b_protocol.len)
	      {
		  CAPI_INIT(CAPI_B_PROTOCOL, &b_protocol);

		  capi_decode(connect_resp.b_protocol.ptr,
			      connect_resp.b_protocol.len, &b_protocol);

		  if(b_protocol.wB1_protocol == 0)
		  {
			cd->channel_bprot = BPROT_RHDLC;
		  }

		  if(b_protocol.wB1_protocol == 1)
		  {
			cd->channel_bprot = BPROT_NONE;
		  }

		  if(b_protocol.wB1_protocol >= 2)
		  {
			NDBGL4(L4_ERR, "cdid=%d, unsupported B1 protocol=%d, "
			       " (ignored)", cd->cdid, b_protocol.wB1_protocol);
		  }

		  if(b_protocol.wB2_protocol != 1)
		  {
			NDBGL4(L4_ERR, "cdid=%d, unsupported B2 protocol=%d, "
			       " (ignored)", cd->cdid, b_protocol.wB2_protocol);
		  }
		  if(b_protocol.wB3_protocol != 0)
		  {
			NDBGL4(L4_ERR, "cdid=%d, unsupported B3 protocol=%d, "
			       " (ignored)", cd->cdid, b_protocol.wB3_protocol);
		  }
	      }

	      /* set application interface before 
	       * disconnect indication
	       */
	      cd_set_appl_interface(cd,I4B_AI_CAPI,sc);

	      /* send disconnect indication
	       * to all other application interfaces
	       */
	      i4b_l4_disconnect_ind(cd,1);

	      cd->driver_type = DRVR_CAPI_B3;
	      cd->driver_unit = 0;

	      cd->shorthold_data.shorthold_algorithm = SHA_FIXU;
	      cd->shorthold_data.unitlen_time = 0;
	      cd->shorthold_data.earlyhup_time = 0;
	      cd->shorthold_data.idle_time = 0; /* seconds (disabled) */

	      cd->isdntxdelay = 0; /* seconds (disabled) */

	      N_CONNECT_RESPONSE(cd, response, cause);
	      break;

	    case CAPI_P_RESP(CONNECT_ACTIVE):
	      if((cd->dir_incoming) && (cd->ai_type == I4B_AI_CAPI))
	      {
		  /* incoming B-channel setup sequence:
		   *
		   * Application              CAPI
		   * <--- connect active ind
		   *      connect active resp ---> (here)
		   * <--- connect b3 ind
		   *      connect b3 resp --->
		   *
		   * <--- connect b3 active ind (common point)
		   *      connect b3 active resp --->
		   */

		  m2 = capi_make_connect_b3_ind(cd);
		  goto send_confirmation;
	      }
	      break;

	    case CAPI_P_RESP(CONNECT_B3):
	      if((cd->dir_incoming) && (cd->ai_type == I4B_AI_CAPI))
	      {
		  CAPI_INIT(CAPI_CONNECT_B3_RESP, &connect_b3_resp);

		  capi_decode(&msg.data, msg.head.wLen, &connect_b3_resp);

		  if(connect_b3_resp.wReject)
		  {
		      /* preset causes with our cause */

		      cd->cause_in = cd->cause_out = 
			(CAUSET_I4B << 8) | CAUSE_I4B_REJECT;

		      N_DISCONNECT_REQUEST(cd, cd->cause_in);

		      cd = NULL; /* call descriptor is freed ! */
		  }
		  else
		  {
		      m2 = capi_make_connect_b3_active_ind(cd);
		      goto send_confirmation;
		  }
	      }
	      break;

	    case CAPI_P_RESP(CONNECT_B3_ACTIVE):
	      if(i4b_link_bchandrvr(cd, 1))
	      {
		  /* XXX one could try to detect 
		   * failure earlier
		   */
		  NDBGL4(L4_ERR, "cdid=%d: could not connect "
			 "B-channel!", cd->cdid);
	      }
	      break;

	    case CAPI_P_RESP(CONNECT_B3_T90_ACTIVE):
	    case CAPI_P_RESP(DATA_B3):
	    case CAPI_P_RESP(DISCONNECT_B3):
	    case CAPI_P_RESP(DISCONNECT):
	    case CAPI_P_RESP(FACILITY):
	    case CAPI_P_RESP(INFO):
	    case CAPI_P_RESP(MANUFACTURER):
	    case CAPI_P_RESP(RESET_B3):
	      /* XXX could have checked for
	       * response errors
	       */
	      break;

	    send_confirmation:
	      capi_ai_putqueue(sc, 0, m2);
	      break;

	    default:

	      NDBGL4(L4_ERR, "cdid=%d: Unknown frame: "
		     "wCmd=0x%04x (ignored)!",
		     cd->cdid, msg.head.wCmd);

	      /* only return error
	       * when operation cannot
	       * be continued
	       *
	       * error = EINVAL; 
	       */
	      break;
	    }
	  }
	  else
	  {
		NDBGL4(L4_MSG, "cdid=%d: wCmd=0x%04x frame ignored. "
		       "Application interface mismatch!", 
		       cd->cdid, msg.head.wCmd);

		/* do nothing ! */
	  }
	}
	else
	{
	    switch(msg.head.wCmd) {
	    case CAPI_P_REQ(LISTEN):/* ================================ */

	      m2 = capi_make_conf(&msg, CAPI_CONF(LISTEN), 0x0000);

	      if(m2)
	      {
		  u_int8_t controller = (msg.head.dwCid & 0xFF) % MAX_CONTROLLERS;

		  CAPI_INIT(CAPI_LISTEN_REQ, &listen_req);

		  capi_decode(&msg.data, msg.head.wLen, &listen_req);

		  mtx_lock(&sc->sc_mtx);

		  sc->sc_info_mask[controller]  = listen_req.dwInfoMask;
		  sc->sc_CIP_mask_1[controller] = listen_req.dwCipMask1;
		  sc->sc_CIP_mask_2[controller] = listen_req.dwCipMask2;

		  mtx_unlock(&sc->sc_mtx);
	      }
	      goto send_confirmation;

	    case CAPI_P_REQ(CONNECT): /* ============================== */

	      m2 = capi_make_conf(&msg, CAPI_CONF(CONNECT), 0x2003);

	      goto send_confirmation;

	    case CAPI_P_RESP(CONNECT):
	    case CAPI_P_RESP(DISCONNECT_B3):
	    case CAPI_P_REQ(DISCONNECT_B3):
	    case CAPI_P_RESP(DISCONNECT):
	    case CAPI_P_REQ(DISCONNECT):
	    case CAPI_P_RESP(ALERT):
	    case CAPI_P_REQ(ALERT):
	    case CAPI_P_RESP(DATA_B3):
	    case CAPI_P_REQ(DATA_B3):
	    case CAPI_P_REQ(CONNECT_B3):
	    case CAPI_P_RESP(CONNECT_B3):
	    case CAPI_P_REQ(INFO):
	    case CAPI_P_RESP(INFO):
	    case CAPI_P_REQ(RESET_B3):
	    case CAPI_P_RESP(RESET_B3):
	    case CAPI_P_REQ(SELECT_B_PROTOCOL):
	    case CAPI_P_RESP(SELECT_B_PROTOCOL):
	    case CAPI_P_REQ(FACILITY):
	    case CAPI_P_RESP(FACILITY):
	    case CAPI_P_REQ(CONNECT_ACTIVE):
	    case CAPI_P_RESP(CONNECT_ACTIVE):

	      /* the call descriptor has been
	       * freed, so just ignore these
	       * last responses or requests
	       */
	      break;

	    default:
	      NDBGL4(L4_ERR, "wCID=0x%08x: Invalid frame or CID: "
		     "wCmd=0x%04x (ignored)!",
		     msg.head.dwCid, msg.head.wCmd);

	      /* only return error
	       * when operation cannot
	       * be continued
	       *
	       * error = EINVAL; 
	       */
	      break;
	    }
	}

	if(cntl)
	{
		CNTL_UNLOCK(cntl);
	}

	mtx_lock(&sc->sc_mtx);

	sc->sc_flags &= ~ST_WR_SLEEP_ENTERED;

	if(sc->sc_flags & ST_CLOSING)
	{
		wakeup(sc);
		error = EIO;
	}
	mtx_unlock(&sc->sc_mtx);

 done:
	if(m1)
	{
		m_freem(m1);
	}
	return error;
}

/*---------------------------------------------------------------------------*
 *	capi_ioctl - device driver ioctl routine
 *---------------------------------------------------------------------------*/
static int
capi_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	struct capi_ai_softc *sc = DEV2SC(dev);
	struct i4b_controller *cntl;
	int error = 0;

	if(sc == NULL)
	{
		return ENODEV;
	}

	mtx_lock(&sc->sc_mtx);

	if(sc->sc_flags & (ST_IOCTL|ST_CLOSING))
	{
		/* only one thread at a time */
		mtx_unlock(&sc->sc_mtx);
		return EBUSY;
	}

	sc->sc_flags |= ST_IOCTL;

	mtx_unlock(&sc->sc_mtx);


	switch(cmd) {

	case CAPI_REGISTER_REQ:
	{
		struct capi_register_req *req = (void *)data;

		if(req->max_b_data_len > BCH_MAX_DATALEN) {
		  req->max_b_data_len = BCH_MAX_DATALEN;
		}

		if(req->max_b_data_len < 128) {
		  req->max_b_data_len = 128;
		}

		mtx_lock(&sc->sc_mtx);
		sc->sc_max_b_data_len = req->max_b_data_len;
		mtx_unlock(&sc->sc_mtx);

		req->app_id = 0; /* unused */
		break;
	}

	case CAPI_START_D_CHANNEL_REQ:
	{
		u_int8_t d_open;

		mtx_lock(&sc->sc_mtx);
		if(!(sc->sc_flags & ST_D_OPEN))
		{
		    sc->sc_flags |= ST_D_OPEN;
		    d_open = 1;
		}
		else
		{
		    d_open = 0;
		}
		mtx_unlock(&sc->sc_mtx);

		if(d_open)
		{
		    /* open D-channels */
		    i4b_update_all_d_channels(1);
		}
		break;
	}

	case CAPI_GET_MANUFACTURER_REQ:
	{
		struct capi_get_manufacturer_req *req = (void *)data;

		/* get controller */

		cntl = CNTL_FIND(req->controller);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* set default */

		snprintf(&req->name[0], sizeof(req->name), 
			 "unknown controller %d", req->controller);

		/* forward ioctl to layer 1 
		 * (and ignore any errors)
		 */

		(void) L1_COMMAND_REQ(cntl, CMR_CAPI_GET_MANUFACTURER, data);

		break;
	}

	case CAPI_GET_VERSION_REQ:
	{
		struct capi_get_version_req *req = (void *)data;

		/* get controller */

		cntl = CNTL_FIND(req->controller);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* set defaults */

		req->version.CAPI_major = 2;
		req->version.CAPI_minor = 0;
		req->version.manufacturer_major = 0;
		req->version.manufacturer_minor = 0;
		req->version.BSD_major = I4B_VERSION;
		req->version.BSD_minor = I4B_REL;

		/* forward ioctl to layer 1 
		 * (and ignore any errors)
		 */

		(void) L1_COMMAND_REQ(cntl, CMR_CAPI_GET_VERSION, data);

		break;
	}

	case CAPI_GET_SERIAL_REQ:
	{
		struct capi_get_serial_req *req = (void *)data;

		/* get controller */

		cntl = CNTL_FIND(req->controller);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* set default */

		CNTL_LOCK(cntl);

		snprintf(&req->serial_number[0], sizeof(req->serial_number),
			 "%d", cntl->N_serial_number);

		CNTL_UNLOCK(cntl);

		/* forward ioctl to layer 1 
		 * (and ignore any errors)
		 */

		(void) L1_COMMAND_REQ(cntl, CMR_CAPI_GET_SERIAL, data);

		break;
	}

	case CAPI_IOCTL_TEST_REQ:
	{
		/* do nothing */
		break;
	}

	case CAPI_GET_PROFILE_REQ:
	{
		struct capi_get_profile_req *req = (void *)data;

		/* get controller */

		cntl = CNTL_FIND(req->controller);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* set defaults */

		bzero(&req->profile, sizeof(req->profile));

		req->profile.wNumCtlr = htole16(MAX_CONTROLLERS);
		req->profile.wNumBChannels = htole16(MAX_CHANNELS);
		req->profile.dwGlobalOptions = 
		  htole32(CAPI_PROFILE_INTERNAL_CTLR_SUPPORT);

		req->profile.dwB1ProtocolSupport = 
		  htole32((1 << CAPI_B1_HDLC_64) |
			  (1 << CAPI_B1_TRANSPARENT_64));

		req->profile.dwB2ProtocolSupport = 
		  htole32(/*(1 << CAPI_B2_ISO7776_X75_SLP) | XXX */
			  (1 << CAPI_B2_TRANSPARENT));

		req->profile.dwB3ProtocolSupport = 
		  htole32((1 << CAPI_B3_TRANSPARENT));

		/* forward ioctl to layer 1 
		 * (and ignore any errors)
		 */
		(void) L1_COMMAND_REQ(cntl, CMR_CAPI_GET_PROFILE, data);

		break;
	}

	case I4B_CTRL_DOWNLOAD:

		/* check credentials */

		if(suser(curthread))
		{
		    error = EPERM;
		    break;
		}

		/* get controller */

		cntl = CNTL_FIND(((cdid_t *)data)[0]);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* forward ioctl to layer 1 */
		error = i4b_controller_download(cntl, (void *)data);

		break;

	case FIONREAD:

		mtx_lock(&sc->sc_mtx);

		/* return the minimum length that can be read */

		if(_IF_QEMPTY(&sc->sc_rdqueue))
		    *(int *)data = 0;
		else
		    *(int *)data = sizeof(struct CAPI_HEADER_ENCODED);

		mtx_unlock(&sc->sc_mtx);

		break;
      
	case FIONBIO:

		mtx_lock(&sc->sc_mtx);

		if(*(int *)data)
		  sc->sc_flags &= ~ST_BLOCK;
		else
		  sc->sc_flags |= ST_BLOCK;

		mtx_unlock(&sc->sc_mtx);
		break;

	default:
		error = ENXIO;
		break;
	}

	mtx_lock(&sc->sc_mtx);

	sc->sc_flags &= ~ST_IOCTL;

	if(sc->sc_flags & ST_CLOSING)
	{
		wakeup(sc);
		error = EIO;
	}
	mtx_unlock(&sc->sc_mtx);

	return(error);
}

/*---------------------------------------------------------------------------*
 *	capi_poll - device driver poll routine
 *---------------------------------------------------------------------------*/
static int
capi_poll(struct cdev *dev, int events, struct thread *td)
{
	struct capi_ai_softc *sc = DEV2SC(dev);
	int revents = 0; /* events found */

	if(sc == NULL)
	{
		return POLLNVAL;
	}

	mtx_lock(&sc->sc_mtx);

	if((!_IF_QEMPTY(&sc->sc_rdqueue)) || (sc->sc_flags & ST_MBUF_LOST))
	{
		revents |= (events & (POLLIN|POLLRDNORM));
	}

	/* assume that one can always write data */
	revents |= (events & (POLLOUT|POLLWRNORM));

	if(revents == 0)
	{
		selrecord(td, &sc->sc_selinfo);
		sc->sc_flags |= ST_SELECT;
	}

	mtx_unlock(&sc->sc_mtx);

	return(revents);
}

/*---------------------------------------------------------------------------*
 *	feedback from daemon in case of dial problems
 *---------------------------------------------------------------------------*/
void
capi_response_to_user(msg_response_to_user_t *mrtu)
{
	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a new frame (mbuf) has been received
 *---------------------------------------------------------------------------*/
static void
capi_put_mbuf(struct fifo_translator *f, struct mbuf *m1)
{
	struct call_desc *cd = f->L5_sc;
	struct capi_ai_softc *sc = cd->ai_ptr;
	struct {
	  /* XXX this structure is 
	   * hardcoded to save some CPU
	   */
	  struct CAPI_HEADER_ENCODED head;

	  u_int32_t dwPtr_1;
	  u_int16_t wLen;
	  u_int16_t wHandle;
	  u_int16_t wFlags;
	  u_int64_t qwPtr_2;
	} __packed *mp;

	struct mbuf *m2;

	if(cd->ai_type == I4B_AI_CAPI)
	{
	    if((m2 = i4b_getmbuf(sizeof(*mp), M_NOWAIT)))
	    {
	        mp = (void *)(m2->m_data);

		mp->head.wLen = htole16(sizeof(*mp));
		mp->head.wApp = htole16(0); /* (x) */
		mp->head.wCmd = htole16(CAPI_IND(DATA_B3));
		mp->head.wNum = htole16(0); /* (x) */
		mp->head.dwCid  = htole32(CDID2CAPI_ID(cd->cdid)|CAPI_ID_NCCI);

		mp->wLen = htole16(0); /* (x) */
		mp->wHandle = htole16(0); /* (x) */
		mp->wFlags  = htole16(0);

		mp->dwPtr_1 = htole32(0); /* (x) */
		mp->qwPtr_2 = htole64(0); /* (x) */

		/* (x) these fields are filled out
		 * by the CAPI library
		 */

		m2->m_next = m1; /* store reference to data */

		/* it is not so critial if one
		 * looses B-channel data, so
		 * only call "capi_ai_putqueue()"
		 * when there is an mbuf
		 */
		capi_ai_putqueue(sc, 0, m2);
		return;
	    }
	}

	/* else no data can be received */
	m_freem(m1);
	return;
}

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when the last frame has been sent out and there is no
 *	further frame or mbuf
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_get_mbuf(struct fifo_translator *f)
{
	struct mbuf *m1;
	struct mbuf *m2;
	struct call_desc *cd = f->L5_sc;
	struct capi_ai_softc *sc = cd->ai_ptr;

	if(cd->ai_type == I4B_AI_CAPI)
	{
		_IF_DEQUEUE(&f->tx_queue, m1);

		if(m1)
		{
			/* split mbuf chain */
			m2 = m1;
			m1 = m1->m_next;
			m2->m_next = NULL;

			/* send acknowledge back */
			capi_ai_putqueue(sc, 0, m2);
		}
	}
	else
	{
		/* no data to send */
		m1 = NULL;
	}
	return m1;
}

/*---------------------------------------------------------------------------*
 *	this routine is called when the receive
 *	filter needs a new mbuf where data can be
 *	stored
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_alloc_mbuf(struct fifo_translator *f, u_int16_t def_len, u_int16_t tr_len)
{
	struct call_desc *cd = f->L5_sc;
	struct capi_ai_softc *sc = cd->ai_ptr;

	if(def_len > sc->sc_max_b_data_len)
	{
	   def_len = sc->sc_max_b_data_len;
	}
	return i4b_getmbuf(def_len, M_NOWAIT);
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for this driver
 *---------------------------------------------------------------------------*/
fifo_translator_t *
capi_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, u_int *protocol,
	      u_int driver_type, u_int driver_unit, call_desc_t *cd)
{
	if(!cd)
	{
		return FT_INVALID;
	}

	if(!protocol)
	{
		return cd->fifo_translator;
	}

#if 0
	XXX this is already done:
	cd->fifo_translator = f; 
#endif
	NDBGL4(L4_MSG, "capi, cdid=%d, protocol=%d", 
	       cd->cdid, *protocol);

	if(*protocol)
	{
		/* connected */

		f->L5_sc = cd;

		f->L5_PUT_MBUF = &capi_put_mbuf;
		f->L5_GET_MBUF = &capi_get_mbuf;
		f->L5_ALLOC_MBUF = &capi_alloc_mbuf;
	}
	return f;
}
