/*-
 * Copyright (c) 2005-2006 Hans Petter Selasky. All rights reserved.
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
#if __FreeBSD_version >= 700050
# include <sys/priv.h>
#endif
#include <net/if.h>
#if __FreeBSD_version >= 800000
#define suser(x) priv_check(x, PRIV_DRIVER)
#endif

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#define CAPI_MAKE_TRANSLATOR
#include <i4b/include/capi20.h>

#include <i4b/layer4/i4b_l4.h>

__FBSDID("$FreeBSD: $");

struct capi_delegate {
	uint32_t uid;
	uint32_t gid;
	uint16_t mode;
	uint16_t use_count;
	uint16_t max_count;
};

/* the following structure describes one CAPI application */

struct capi_ai_softc {

	u_int32_t sc_info_mask[MAX_CONTROLLERS]; /* used to limit information */
	u_int32_t sc_CIP_mask_1[MAX_CONTROLLERS]; /* used to select incoming calls */
	u_int32_t sc_CIP_mask_2[MAX_CONTROLLERS]; /* used to select incoming calls */

	u_int16_t sc_max_b_data_len;
#define MIN_B_DATA_LEN 128 /* bytes */

	u_int16_t sc_max_b_data_blocks;

	struct cdev *sc_dev;

	struct capi_delegate *sc_delegate;

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

	/* the following fields are only used by capi_write() */

	int sc_write_busy;

	struct capi_message_encoded sc_msg;
	struct CAPI_ALERT_REQ_DECODED sc_alert_req;
	struct CAPI_CONNECT_REQ_DECODED sc_connect_req;
	struct CAPI_DATA_B3_REQ_DECODED sc_data_b3_req;
	struct CAPI_INFO_REQ_DECODED sc_info_req;
	struct CAPI_CONNECT_RESP_DECODED sc_connect_resp;
	struct CAPI_CONNECT_B3_RESP_DECODED sc_connect_b3_resp;
	struct CAPI_LISTEN_REQ_DECODED sc_listen_req;
	struct CAPI_SELECT_B_PROTOCOL_REQ_DECODED sc_select_b_protocol_req;
	struct CAPI_FACILITY_REQ_DECODED sc_facility_req;
	struct CAPI_ADDITIONAL_INFO_DECODED sc_add_info;
	struct CAPI_SENDING_COMPLETE_DECODED sc_sending_complete;
	struct CAPI_B_PROTOCOL_DECODED sc_b_protocol;
	struct CAPI_FACILITY_REQ_DTMF_PARAM_DECODED sc_dtmf_req;
	struct CAPI_LINE_INTERCONNECT_PARAM_DECODED sc_li_param;
	struct CAPI_LI_CONN_REQ_PARAM_DECODED sc_li_conn_req_param;
	struct CAPI_LI_CONN_REQ_PART_DECODED sc_li_conn_req_part;
	struct CAPI_LI_DISC_REQ_PARAM_DECODED sc_li_disc_req_param;
	struct CAPI_LI_DISC_REQ_PART_DECODED sc_li_disc_req_part;
	struct CAPI_SUPPL_PARAM_DECODED sc_suppl_param;
	struct CAPI_FACILITY_REQ_CALL_DEFL_PARAM_DECODED sc_cd_req;
	struct CAPI_GENERIC_STRUCT_DECODED sc_gen_struct;
	struct CAPI_EC_FACILITY_PARM_DECODED sc_ec_parm;
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

#define CAPI_DELEGATES_MAX 16

static struct capi_ai_softc *capi_ai_sc_root = NULL;

/* the following structure is protected by "capi_ai_global_lock" */

static struct capi_delegate capi_delegate[CAPI_DELEGATES_MAX];

static uint16_t capi_delegate_count = 0;

static struct cdev *capi_dev = NULL;

static eventhandler_tag capi_clone_tag = NULL;

static u_int32_t capi_units = 0;

#define CAPINAME "capi20"

static struct capi_delegate *
capi_ai_find_delegate_by_uid(uint32_t uid)
{
	struct capi_delegate *dg = capi_delegate + 1;
	struct capi_delegate *dg_end = capi_delegate + capi_delegate_count;

	if (uid >= 0xffffffff) return NULL;

	while (dg != dg_end) {
	    if (dg->uid == uid) {
	        return dg;
	    }
	    dg++;
	}
	return NULL;
}

static struct capi_delegate *
capi_ai_find_delegate_by_gid(uint32_t gid)
{
	struct capi_delegate *dg = capi_delegate + 1;
	struct capi_delegate *dg_end = capi_delegate + capi_delegate_count;

	if (gid >= 0xffffffff) return NULL;

	while (dg != dg_end) {
	    if (dg->gid == gid) {
	        return dg;
	    }
	    dg++;
	}
	return NULL;
}

static uint8_t
capi_ai_update_delegate(i4b_capi_delegate_t *capi_dg)
{
	struct capi_delegate *dg;
	uint16_t mode;
	uint8_t error = 0;

	dg = capi_ai_find_delegate_by_uid(capi_dg->uid);
	mode = (capi_dg->mode & 0600) ? 0600 : 0000;

	if (dg == NULL) {
	    /* create a new delegate */
	    if (capi_delegate_count >= CAPI_DELEGATES_MAX) {
	        error = 1;
		goto try_gid;
	    }

	    dg = capi_delegate + capi_delegate_count;
	    capi_delegate_count ++;
	    dg->uid = capi_dg->uid;
	    dg->gid = 0xffffffff;
	    dg->use_count = 0;
	}

	/* simply update */
	dg->mode = mode;
	dg->max_count = capi_dg->max_units;

 try_gid:
	dg = capi_ai_find_delegate_by_gid(capi_dg->gid);
	mode = (capi_dg->mode & 0060) ? 0060 : 0000;

	if (dg == NULL) {
	    /* create a new delegate */
	    if (capi_delegate_count >= CAPI_DELEGATES_MAX) {
	        error = 1;
		goto done;
	    }
	    dg = capi_delegate + capi_delegate_count;
	    capi_delegate_count ++;
	    dg->uid = 0xffffffff;
	    dg->gid = capi_dg->gid;
	    dg->use_count = 0;
	}

	/* simply update */
	dg->mode = mode;
	dg->max_count = capi_dg->max_units;

 done:
	return error;
}

static void
capi_ai_global_lock(uint8_t do_lock)
{
	static uint8_t flag = 0;
	int error;

	mtx_assert(&i4b_global_lock, MA_OWNED);

	/* XXX we should replace this by a SX lock one day! */

	if (do_lock)
	{
	    while (flag)
	    {
		flag |= 2;
		error = msleep(&flag, &i4b_global_lock, 
			       0, "CAPI AI create unit", 0);
	    }
	    flag = 1;
	}
	else
	{
	    if (flag & 2)
	    {
		wakeup(&flag);
	    }
	    flag = 0;
	}
	return;
}

static struct capi_ai_softc *
capi_ai_get_closed_sc(struct capi_delegate *dg, struct thread *td)
{
	struct capi_ai_softc *sc;
	u_int8_t free_sc = 0;
	u_int8_t link_sc = 0;
	u_int32_t unit;

	mtx_lock(&i4b_global_lock);

	capi_ai_global_lock(1);

	sc = capi_ai_sc_root;

	unit = capi_units;

	mtx_unlock(&i4b_global_lock);

	if (dg == NULL) {

	    /* try by user first */
#ifdef __FreeBSD__
	    dg = capi_ai_find_delegate_by_uid(td->td_ucred->cr_ruid);
#else
	    dg = capi_ai_find_delegate_by_uid(
	        kauth_cred_geteuid(td->p_cred));
#endif
	    if ((dg == NULL) || 
		((dg->mode & 0600) == 0)) {

	        /* try by group second */
#ifdef __FreeBSD__
	        dg = capi_ai_find_delegate_by_gid(td->td_ucred->cr_rgid);
#else
		dg = capi_ai_find_delegate_by_gid(
		    kauth_cred_getegid(td->p_cred));
#endif
		if ((dg == NULL) || 
		    ((dg->mode & 0060) == 0)) {
			sc = NULL;
			goto done;
		}
	    }
	}

	/* first search for an existing closed device */

	while(sc)
	{
	    u_int8_t closed;

	    mtx_lock(&sc->sc_mtx);

	    if (sc->sc_delegate != dg) {
	        closed = 0;
	    } else {
	        closed = !(sc->sc_flags & (ST_OPEN|ST_CLOSING|ST_OPENING));
	    }

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

	if (dg->use_count >= dg->max_count)
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

#define IFQ_LIMIT_LOW (0xC0)
#define IFQ_LIMIT_HIGH (0x100)

	sc->sc_rdqueue.ifq_maxlen = IFQ_LIMIT_HIGH; /* XXX dynamic ? */

	mtx_init(&sc->sc_mtx, "CAPI AI", NULL, MTX_DEF | MTX_RECURSE);

	sc->sc_dev = 
	  make_dev(&capi_cdevsw, unit+1, (dg->uid < 0xffffffff) ? dg->uid : UID_ROOT,
					 (dg->gid < 0xffffffff) ? dg->gid : GID_WHEEL,
					  dg->mode, CAPINAME ".%03x", unit);

	if(sc->sc_dev == NULL)
	{
		free_sc = 1;
		goto done;
	}

	DEV2SC(sc->sc_dev) = sc;
	sc->sc_delegate = dg;

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

	    /* update delegate count */

	    dg->use_count ++;
	}

	capi_ai_global_lock(0);

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

#if ((__FreeBSD_version >= 700001) || (__FreeBSD_version == 0) || \
     ((__FreeBSD_version >= 600034) && (__FreeBSD_version < 700000)))
#define I4B_UCRED struct ucred *ucred,
#else
#define I4B_UCRED
#endif

static void
capi_clone(void *arg, I4B_UCRED char *name, int namelen, struct cdev **dev)
{
	struct capi_ai_softc *sc;
	struct thread *td;

        if(dev[0] != NULL)
	{
		return;
	}

        if(strcmp(name, CAPINAME) != 0)
	{
		return;
        }

	td = curthread;

#ifdef __FreeBSD__
	if (td->td_ucred == NULL)
#else
	if (td->p_cred == NULL)
#endif
	{
		/* sanity */
		return;
	}

	if (suser(td)) {
	    sc = capi_ai_get_closed_sc(NULL, td);
	} else {
	    sc = capi_ai_get_closed_sc(capi_delegate, td);
	}

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

	/* init root delegate */

	capi_delegate[0].uid = UID_ROOT;
	capi_delegate[0].gid = GID_WHEEL;
	capi_delegate[0].mode = 0600;
	capi_delegate[0].max_count = CAPI_APPLICATION_MAX;
	capi_delegate_count ++;

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

#define CAPI_PUTQUEUE_FLAG_SC_COMPLEMENT 0x01
#define CAPI_PUTQUEUE_FLAG_DROP_OK       0x02

/*---------------------------------------------------------------------------*
 *	capi_ai_putqueue - put message into application interface queue(s)
 *
 * NOTE: if "m1 == NULL" this is an indication that the system has lost
 *       an important CAPI message
 *---------------------------------------------------------------------------*/
static void
capi_ai_putqueue(struct capi_ai_softc *sc, 
		 u_int8_t flags, struct mbuf *m1, uint16_t *p_copy_count)
{
	struct capi_ai_softc *sc_exclude;
	struct capi_message_encoded *mp;
	struct mbuf *m2;

	if((sc == NULL) || (flags & CAPI_PUTQUEUE_FLAG_SC_COMPLEMENT))
	{
		/* remove the complement flag */

		flags &= ~CAPI_PUTQUEUE_FLAG_SC_COMPLEMENT;

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

				capi_ai_putqueue(sc,flags,m2,p_copy_count);
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
		    if(flags & CAPI_PUTQUEUE_FLAG_DROP_OK)
		    {
		        goto done;
		    }
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

		/* check if the frame can be dropped */

		if(flags & CAPI_PUTQUEUE_FLAG_DROP_OK)
		{
			if(_IF_QLEN(&sc->sc_rdqueue) >= IFQ_LIMIT_LOW)
			{
				/* data overflow */
				goto done;
			}
		}

		if (p_copy_count) {
		  (*p_copy_count) ++;
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
	       u_int8_t *dst, u_int16_t max_length, 
	       struct i4b_src_telno *p_src)
{
	u_int8_t temp;

	if(p_src)
	{
		p_src->scr_ind = SCR_NONE;
		p_src->prs_ind = PRS_NONE;
		p_src->ton = TON_OTHER;
	}

	if(len)
	{
		NDBGL4(L4_MSG, "cdid=%d: numbering "
		       "plan=0x%02x, p_src=%d (ignored)",
		       cd->cdid, src[0], p_src != NULL);

		if(p_src)
		{
		    temp = (src[0] & 0x70) >> 4;

		    p_src->ton = 
		      (temp == 1) ? TON_INTERNAT :
		      (temp == 2) ? TON_NATIONAL :
		      TON_OTHER;
		}

		src++;
		len--;

		if(len && p_src)
		{
		    /* presentation indicator */
		    p_src->prs_ind = 
		      ((src[0] & 0x60) == 0x20) ? PRS_RESTRICT : PRS_ALLOWED;

		    /* screening indicator */
		    switch (src[0] & 0x03) {
		    case 0:
			p_src->scr_ind = SCR_USR_NOSC;
			break;
		    case 1:
			p_src->scr_ind = SCR_USR_PASS;
			break;
		    case 2:
			p_src->scr_ind = SCR_USR_FAIL;
			break;
		    default:
			p_src->scr_ind = SCR_NET;
			break;
		    }

		    src++;
		    len--;
		}
	}

	if(len > max_length)
	{
		NDBGL4(L4_ERR, "cdid=%d: truncating telephone "
		       "number from %d to %d bytes, p_src=%d", 
		       cd->cdid, p_src != NULL, len, max_length);
		len = max_length;
	}
	bcopy(src, dst, len);
	dst[len] = '\0'; /* zero terminate string ! */
	return;
}

/*---------------------------------------------------------------------------*
 *	extract CAPI facility telephone number
 *---------------------------------------------------------------------------*/
static void
capi_get_fac_telno(struct call_desc *cd, u_int8_t *src, u_int16_t len, 
		   u_int8_t *dst, u_int16_t max_length)
{
	if(len)
	{
	    /* type of facility number */

	    src++;
	    len--;

	    if(len)
	    {
	        /* type of number and numbering plan */

	        src++;
		len--;

		if(len)
		{
		    /* presentation and screening indicator */

		    src++;
		    len--;
		}
	    }
	}

	if(len > max_length)
	{
		NDBGL4(L4_ERR, "cdid=%d: truncating telephone "
		       "number from %d to %d bytes", 
		       cd->cdid, len, max_length);
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
capi_put_telno(struct call_desc *cd, const u_int8_t *src, u_int8_t *dst, 
	       u_int16_t len, u_int8_t type, struct i4b_src_telno *p_src)
{
	u_int8_t *dst_end;

	if(len < 2)
	{
	    /* invalid length */
	    return 0;
	}

	dst_end = dst + len;

	*dst++ = type; /* number PLAN */

	if(p_src)
	{
	    u_int8_t temp;

	    /* set extension bit */
	    temp = 0x80;

	    /* add presentation indicator */
	    switch (p_src->prs_ind) {
	    case PRS_RESTRICT:
		temp |= 0x20;
		break;
	    case PRS_NNINTERW:
		temp |= 0x40;
		break;
	    default:
		break;
	    }

	    /* add screening indicator */
	    switch (p_src->scr_ind) {
	    case SCR_USR_PASS:
		temp |= 0x01;
		break;
	    case SCR_USR_FAIL:
		temp |= 0x02;
		break;
	    case SCR_NET:
		temp |= 0x03;
		break;
	    default:
		break;
	    }

	    *dst++ = temp;
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
 *	send a facility indication message
 *---------------------------------------------------------------------------*/
static void
capi_ai_facility_ind(struct call_desc *cd, u_int16_t wSelector, 
		     u_int8_t flags, void *param)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_FACILITY_IND_DECODED fac_ind;
	u_int16_t len;

	memset(&fac_ind, 0, sizeof(fac_ind));

	CAPI_INIT(CAPI_FACILITY_IND, &fac_ind);

	fac_ind.wSelector = wSelector;

	if(param)
	{
	    fac_ind.Param.ptr = param;
	    fac_ind.Param_STRUCT = IE_STRUCT_DECODED;
	}

	len = capi_encode(&msg.data, sizeof(msg.data), &fac_ind);
	len += sizeof(msg.head);

	/* fill out CAPI header */

	msg.head.wLen = htole16(len);
	msg.head.wApp = htole16(0);
	msg.head.wCmd = htole16(CAPI_IND(FACILITY));
	msg.head.wNum = htole16(0);
	msg.head.dwCid = htole32(CDID2CAPI_ID(cd->cdid));

	if((m = i4b_getmbuf(len, M_NOWAIT)))
	{
	    bcopy(&msg, m->m_data, m->m_len);
	}
	
	capi_ai_putqueue(cd->ai_ptr,flags,m,NULL);
	return;
}

/*---------------------------------------------------------------------------*
 *	generate facility confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_facility_conf(struct capi_message_encoded *pmsg, 
			u_int16_t wSelector, u_int16_t wInfo, void *param)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_FACILITY_CONF_DECODED fac_conf;
	u_int16_t len;

	memset(&fac_conf, 0, sizeof(fac_conf));

	CAPI_INIT(CAPI_FACILITY_CONF, &fac_conf);

	fac_conf.wInfo = wInfo;
	fac_conf.wSelector = wSelector;

	if(param)
	{
	    fac_conf.Param.ptr = param;
	    fac_conf.Param_STRUCT = IE_STRUCT_DECODED;
	}

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
 *	generate supplementary facility confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_fac_suppl_conf(struct capi_message_encoded *pmsg, 
			 u_int16_t wFunction, void *param)
{
	struct CAPI_SUPPL_PARAM_DECODED suppl;

	memset(&suppl, 0, sizeof(suppl));

	CAPI_INIT(CAPI_SUPPL_PARAM, &suppl);

	suppl.wFunction = wFunction;

	if(param)
	{
	    suppl.Param.ptr = param;
	    suppl.Param_STRUCT = IE_STRUCT_DECODED;
	}
	return capi_make_facility_conf(pmsg, 0x0003, 0x0000, &suppl);
}

/*---------------------------------------------------------------------------*
 *	generate supplementary facility confirmation message (type 1)
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_fac_suppl_conf_type1(struct capi_message_encoded *pmsg, 
				    u_int16_t wFunction, u_int16_t wResult)
{
	struct CAPI_FACILITY_CONF_CALL_DEFL_PARAM_DECODED conf;

	memset(&conf, 0, sizeof(conf));

	CAPI_INIT(CAPI_FACILITY_CONF_CALL_DEFL_PARAM, &conf);

	conf.wResult = wResult;

	return capi_make_fac_suppl_conf(pmsg, wFunction, &conf);
}

/*---------------------------------------------------------------------------*
 *	generate line interface support confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_li_supp_conf(struct capi_message_encoded *pmsg)
{
	struct CAPI_LI_SUPP_CONF_PARAM_DECODED li_supp_conf;
	struct CAPI_LINE_INTERCONNECT_PARAM_DECODED li_parm;

	bzero(&li_supp_conf, sizeof(li_supp_conf));
	CAPI_INIT(CAPI_LI_SUPP_CONF_PARAM, &li_supp_conf);

	bzero(&li_parm, sizeof(li_parm));
	CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &li_parm);

	li_parm.wFunction = 0x0000;
	li_parm.Param.ptr = &li_supp_conf;
	li_parm.Param_STRUCT = IE_STRUCT_DECODED;

	li_supp_conf.wInfo = 0x0000;
	li_supp_conf.dwSupportedServices = 0x00000001;
	li_supp_conf.dwInterconnectsCtrl = 1;
	li_supp_conf.dwParticipantsCtrl = 1;
	li_supp_conf.dwInterconnectsGlobal = 1;
	li_supp_conf.dwParticipantsGlobal = 1;

	return capi_make_facility_conf
	  (pmsg, 0x0005, 0x0000, &li_parm);
}

/*---------------------------------------------------------------------------*
 *	generate supplementary support confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_suppl_supp_conf(struct capi_message_encoded *pmsg)
{
	struct CAPI_FACILITY_CONF_GET_SUPPL_DECODED suppl_conf;

	memset(&suppl_conf, 0, sizeof(suppl_conf));

	CAPI_INIT(CAPI_FACILITY_CONF_GET_SUPPL, &suppl_conf);

	suppl_conf.dwServices = 
	  0x00000020 /* Call deflection */ |
	  0x00000040 /* MCID */ |
	  0x00000001 /* HOLD/RETRIEVE */;

	return capi_make_fac_suppl_conf
	  (pmsg, 0x0000, &suppl_conf);
}

/*---------------------------------------------------------------------------*
 *	generate line interface connect confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_li_conn_conf(struct capi_message_encoded *pmsg, 
		       u_int16_t wInfo, u_int32_t dwCid)
{
	struct CAPI_LI_CONN_CONF_PART_DECODED li_conn_conf_part;
	struct CAPI_GENERIC_STRUCT_DECODED gen_struct;
	struct CAPI_LI_CONN_CONF_PARAM_DECODED li_conn_conf;
	struct CAPI_LINE_INTERCONNECT_PARAM_DECODED li_parm;

	bzero(&li_conn_conf_part, sizeof(li_conn_conf_part));
	bzero(&gen_struct, sizeof(gen_struct));
	bzero(&li_conn_conf, sizeof(li_conn_conf));
	bzero(&li_parm, sizeof(li_parm));

	CAPI_INIT(CAPI_LI_CONN_CONF_PART, &li_conn_conf_part);
	CAPI_INIT(CAPI_GENERIC_STRUCT, &gen_struct);
	CAPI_INIT(CAPI_LI_CONN_CONF_PARAM, &li_conn_conf);
	CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &li_parm);

	li_parm.wFunction = 0x0001;
	li_parm.Param.ptr = &li_conn_conf;
	li_parm.Param_STRUCT = IE_STRUCT_DECODED;

	li_conn_conf.wInfo = wInfo;
	li_conn_conf.conn_conf_part.ptr = &gen_struct;
	li_conn_conf.conn_conf_part_STRUCT = IE_STRUCT_DECODED;

	gen_struct.Param.ptr = &li_conn_conf_part;
	gen_struct.Param_STRUCT = IE_STRUCT_DECODED;

	li_conn_conf_part.wInfo = wInfo;
	li_conn_conf_part.dwCid = dwCid;

	return capi_make_facility_conf
	  (pmsg, 0x0005, 0x0000, &li_parm);
}

/*---------------------------------------------------------------------------*
 *	generate line interface disconnect confirmation message
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_li_disc_conf(struct capi_message_encoded *pmsg, 
		       u_int16_t wInfo, u_int32_t dwCid)
{
	struct CAPI_LI_DISC_CONF_PART_DECODED li_disc_conf_part;
	struct CAPI_GENERIC_STRUCT_DECODED gen_struct;
	struct CAPI_LI_DISC_CONF_PARAM_DECODED li_disc_conf;
	struct CAPI_LINE_INTERCONNECT_PARAM_DECODED li_parm;

	bzero(&li_disc_conf_part, sizeof(li_disc_conf_part));
	bzero(&gen_struct, sizeof(gen_struct));
	bzero(&li_disc_conf, sizeof(li_disc_conf));
	bzero(&li_parm, sizeof(li_parm));

	CAPI_INIT(CAPI_LI_DISC_CONF_PART, &li_disc_conf_part);
	CAPI_INIT(CAPI_GENERIC_STRUCT, &gen_struct);
	CAPI_INIT(CAPI_LI_DISC_CONF_PARAM, &li_disc_conf);
	CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &li_parm);

	li_parm.wFunction = 0x0002;
	li_parm.Param.ptr = &li_disc_conf;
	li_parm.Param_STRUCT = IE_STRUCT_DECODED;

	li_disc_conf.wInfo = wInfo;
	li_disc_conf.disc_conf_part.ptr = &gen_struct;
	li_disc_conf.disc_conf_part_STRUCT = IE_STRUCT_DECODED;

	gen_struct.Param.ptr = &li_disc_conf_part;
	gen_struct.Param_STRUCT = IE_STRUCT_DECODED;

	li_disc_conf_part.wInfo = wInfo;
	li_disc_conf_part.dwCid = dwCid;

	return capi_make_facility_conf
	  (pmsg, 0x0005, 0x0000, &li_parm);
}

/*---------------------------------------------------------------------------*
 *	generate DTMF support confirmation
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_dtmf_conf(struct capi_message_encoded *pmsg, u_int16_t wInfo)
{
	struct CAPI_FACILITY_CONF_DTMF_PARAM_DECODED dtmf_conf;

	bzero(&dtmf_conf, sizeof(dtmf_conf));

	CAPI_INIT(CAPI_FACILITY_CONF_DTMF_PARAM, &dtmf_conf);

	dtmf_conf.wInfo = wInfo;

	return capi_make_facility_conf
	  (pmsg, 0x0001, 0x0000, &dtmf_conf);
}

/*---------------------------------------------------------------------------*
 *	generate echo cancel support confirmation
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_ec_supp_conf(struct capi_message_encoded *pmsg)
{
	struct CAPI_EC_PARM_SUPP_CONF_DECODED supp_conf;
	struct CAPI_EC_FACILITY_PARM_DECODED fac_parm;

	bzero(&supp_conf, sizeof(supp_conf));
	bzero(&fac_parm, sizeof(fac_parm));

	CAPI_INIT(CAPI_EC_PARM_SUPP_CONF, &supp_conf);
	CAPI_INIT(CAPI_EC_FACILITY_PARM, &fac_parm);

	fac_parm.wFunction = 0x0000;

	fac_parm.Parm.ptr = &supp_conf;
	fac_parm.Parm_STRUCT = IE_STRUCT_DECODED;

	return capi_make_facility_conf
	  (pmsg, 0x0008, 0x0000, &fac_parm);
}

/*---------------------------------------------------------------------------*
 *	generate echo cancel generic confirmation
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_ec_generic_conf(struct capi_message_encoded *pmsg, 
			  u_int16_t wFunction, u_int16_t wInfo)
{
	struct CAPI_EC_PARM_GENERIC_CONF_DECODED conf;
	struct CAPI_EC_FACILITY_PARM_DECODED fac_parm;

	bzero(&conf, sizeof(conf));
	bzero(&fac_parm, sizeof(fac_parm));

	CAPI_INIT(CAPI_EC_PARM_GENERIC_CONF, &conf);
	CAPI_INIT(CAPI_EC_FACILITY_PARM, &fac_parm);

	conf.wInfo = wInfo;

	fac_parm.wFunction = wFunction;

	fac_parm.Parm.ptr = &conf;
	fac_parm.Parm_STRUCT = IE_STRUCT_DECODED;

	return capi_make_facility_conf
	  (pmsg, 0x0008, 0x0000, &fac_parm);
}

/*---------------------------------------------------------------------------*
 *	send CAPI connect B-channel active indication
 *---------------------------------------------------------------------------*/
static void
capi_ai_connect_b3_active_ind(struct call_desc *cd)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_CONNECT_B3_ACTIVE_IND_DECODED connect_b3_active_ind;

	u_int16_t len;

	__KASSERT(((cd->ai_ptr == NULL) || 
		   (cd->ai_type == I4B_AI_CAPI)), 
		  ("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&connect_b3_active_ind, 0, sizeof(connect_b3_active_ind));

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

	capi_ai_putqueue(cd->ai_ptr,0,m,NULL);
	return;
}

/*---------------------------------------------------------------------------*
 *	make CAPI connect B-channel indication
 *---------------------------------------------------------------------------*/
static struct mbuf *
capi_make_connect_b3_ind(struct call_desc *cd)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_CONNECT_B3_IND_DECODED connect_b3_ind;

	u_int16_t len;

	memset(&connect_b3_ind, 0, sizeof(connect_b3_ind));

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
	struct CAPI_INFO_IND_DECODED info_ind;

	__KASSERT(((cd->ai_ptr == NULL) || 
		 (cd->ai_type == I4B_AI_CAPI) || complement), 
		("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&info_ind, 0, sizeof(info_ind));

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

	capi_ai_putqueue(cd->ai_ptr,complement ? 
			 CAPI_PUTQUEUE_FLAG_SC_COMPLEMENT : 0,m,NULL);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI connect indication
 *
 * NOTE: the connect indication can be replayed multiple
 * times using the same cdid with updated information !
 *---------------------------------------------------------------------------*/
void
capi_ai_connect_ind(struct call_desc *cd, uint16_t *p_copy_count)
{
	struct mbuf *m;
	u_int16_t len;
	u_int8_t temp;

	struct capi_message_encoded msg;
	struct CAPI_CONNECT_IND_DECODED connect_ind;
	struct CAPI_ADDITIONAL_INFO_DECODED add_info;

	u_int8_t dst_telno[TELNO_MAX];
	u_int8_t src_telno_1[TELNO_MAX];
	u_int8_t src_telno_2[TELNO_MAX];
	u_int8_t dst_subaddr[SUBADDR_MAX];
	u_int8_t src_subaddr[SUBADDR_MAX];

	static const u_int8_t bc_bprot_alaw[] = { 0x04, 0x03, 0x80, 0x90, 0xA3 };
	static const u_int8_t bc_bprot_ulaw[] = { 0x04, 0x03, 0x80, 0x90, 0xA2 };
	static const u_int8_t bc_bprot_reserved[] = { 0x04, 0x03, 0x80, 0x90, 0xA0 };
	static const u_int8_t bc_bprot_rhdlc[] = { 0x04, 0x02, 0x88, 0x90 };
	static const u_int8_t hlc_bprot_none[] = { 0x7D, 0x02, 0x91, 0x81 };
	static const u_int8_t sending_complete[] = { 0x01, 0x00 };

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

	  switch(cd->channel_bsubprot) {
	  case BSUBPROT_G711_ALAW:
	      connect_ind.BC.ptr = &bc_bprot_alaw;
	      connect_ind.BC.len = sizeof(bc_bprot_alaw);
	      break;
	  case BSUBPROT_G711_ULAW:
	      connect_ind.BC.ptr = &bc_bprot_ulaw;
	      connect_ind.BC.len = sizeof(bc_bprot_ulaw);
	      break;
	  default:
	      connect_ind.BC.ptr = &bc_bprot_reserved;
	      connect_ind.BC.len = sizeof(bc_bprot_reserved);
	      break;
	  }

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

	switch (cd->dst_ton) {
	case TON_INTERNAT:
		temp = 0x80 | 0x10;
		break;
	case TON_NATIONAL:
		temp = 0x80 | 0x20;
		break;
	default:
		temp = 0x80 | 0x00;
		break;
	}
	connect_ind.dst_telno.ptr = &dst_telno[0];
	connect_ind.dst_telno.len = capi_put_telno
	  (cd, &cd->dst_telno[0], &dst_telno[0], 
	   sizeof(dst_telno), temp, NULL);

	/* first calling party number */

	connect_ind.src_telno.ptr = &src_telno_1[0];
	connect_ind.src_telno.len = capi_put_telno
	  (cd, &cd->src[0].telno[0], &src_telno_1[0], 
	   sizeof(src_telno_1), 
	   (cd->src[0].ton == TON_INTERNAT) ? 0x10 :
	   (cd->src[0].ton == TON_NATIONAL) ? 0x20 : 0x00, &(cd->src[0]));

	/* second calling party number */

	connect_ind.src_telno_2.ptr = &src_telno_2[0];
	connect_ind.src_telno_2.len = capi_put_telno
	  (cd, &cd->src[1].telno[0], &src_telno_2[0], 
	   sizeof(src_telno_2), 
	   (cd->src[1].ton == TON_INTERNAT) ? 0x10 :
	   (cd->src[1].ton == TON_NATIONAL) ? 0x20 : 0x00, &(cd->src[1]));

	connect_ind.dst_subaddr.ptr = &dst_subaddr[0];
	connect_ind.dst_subaddr.len = capi_put_telno
	  (cd, &cd->dst_subaddr[0], &dst_subaddr[0], 
	   sizeof(dst_subaddr), 0x80, NULL);

	connect_ind.src_subaddr.ptr = &src_subaddr[0];
	connect_ind.src_subaddr.len = capi_put_telno
	  (cd, &cd->src[0].subaddr[0], &src_subaddr[0], 
	   sizeof(src_subaddr), 0x80, NULL);

	/* link in the additional info */

	connect_ind.add_info_STRUCT = IE_STRUCT_DECODED;
	connect_ind.add_info.ptr = &add_info;

	add_info.keypad.ptr = &cd->keypad[0];
	add_info.keypad.len = strlen(cd->keypad);

	add_info.useruser.ptr = &cd->user_user[0];
	add_info.useruser.len = strlen(cd->user_user);

	if(cd->sending_complete)
	{
	    add_info.sending_complete.ptr = &sending_complete;
	    add_info.sending_complete.len = sizeof(sending_complete);
	}

	connect_ind.display.ptr = &cd->display[0];
	connect_ind.display.len = strlen(cd->display);

	len = capi_encode(&msg.data, sizeof(msg.data), &connect_ind);
	len += sizeof(msg.head);

	/* check if we are not end to end digital */

	if (cd->not_end_to_end_digital) {
	    connect_ind.bFlag_1 |= 
	        CAPI_FLAG1_NOT_END_TO_END_DIGITAL;
	}

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

	capi_ai_putqueue(cd->ai_ptr,0,m,p_copy_count);
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
	struct CAPI_CONNECT_ACTIVE_IND_DECODED connect_active_ind;

	__KASSERT(((cd->ai_ptr == NULL) || 
		 (cd->ai_type == I4B_AI_CAPI)), 
		("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&connect_active_ind, 0, sizeof(connect_active_ind));

	CAPI_INIT(CAPI_CONNECT_ACTIVE_IND, &connect_active_ind);

	connect_active_ind.date_time.ptr = cd->idate_time_data;
	connect_active_ind.date_time.len = cd->idate_time_len;

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

	capi_ai_putqueue(cd->ai_ptr,0,m,NULL);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI B-channel disconnect indication
 *---------------------------------------------------------------------------*/
static void
capi_ai_disconnect_b3_ind(struct call_desc *cd)
{
	struct mbuf *m;
	struct capi_message_encoded msg;
	struct CAPI_DISCONNECT_B3_IND_DECODED disconnect_b3_ind;

	u_int16_t len;

	__KASSERT(((cd->ai_ptr == NULL) || 
		   (cd->ai_type == I4B_AI_CAPI)), 
		  ("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&disconnect_b3_ind, 0, sizeof(disconnect_b3_ind));

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

	capi_ai_putqueue(cd->ai_ptr,0,m,NULL);
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
	struct CAPI_DISCONNECT_IND_DECODED disconnect_ind;

	u_int16_t len;

	__KASSERT(((cd->ai_ptr == NULL) || 
		 (cd->ai_type == I4B_AI_CAPI) || complement), 
		("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&disconnect_ind, 0, sizeof(disconnect_ind));

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

	capi_ai_putqueue(cd->ai_ptr,complement ? 
			 CAPI_PUTQUEUE_FLAG_SC_COMPLEMENT : 0,m,NULL);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI inter-connect indication
 *---------------------------------------------------------------------------*/
static void
capi_ai_line_inter_connect_ind(struct call_desc *cd)
{
	struct CAPI_LINE_INTERCONNECT_PARAM_DECODED li_parm;
	struct CAPI_LI_CONN_IND_PARAM_DECODED li_conn_ind;

	__KASSERT(((cd->ai_ptr == NULL) || 
		   (cd->ai_type == I4B_AI_CAPI)), 
		  ("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&li_parm, 0, sizeof(li_parm));
	memset(&li_conn_ind, 0, sizeof(li_conn_ind));

	CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &li_parm);
	CAPI_INIT(CAPI_LI_CONN_IND_PARAM, &li_conn_ind);

	li_parm.wFunction = 0x0001; /* connect active */
	li_parm.Param.ptr = &li_conn_ind;
	li_parm.Param_STRUCT = IE_STRUCT_DECODED;

	li_conn_ind.dwCid = CDID2CAPI_ID(cd->li_cdid_last);

	capi_ai_facility_ind(cd, 0x0005 /* line interconnect */,
			     0 /* no flags */, &li_parm);
	return;
}

/*---------------------------------------------------------------------------*
 *	send CAPI inter-disconnect indication
 *---------------------------------------------------------------------------*/
static void
capi_ai_line_inter_disconnect_ind(struct call_desc *cd)
{
	struct CAPI_LINE_INTERCONNECT_PARAM_DECODED li_parm;
	struct CAPI_LI_DISC_IND_PARAM_DECODED li_disc_ind;

	__KASSERT(((cd->ai_ptr == NULL) || 
		   (cd->ai_type == I4B_AI_CAPI)), 
		  ("%s: %s: invalid parameters", __FILE__, __FUNCTION__));

	memset(&li_parm, 0, sizeof(li_parm));
	memset(&li_disc_ind, 0, sizeof(li_disc_ind));

	CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &li_parm);
	CAPI_INIT(CAPI_LI_DISC_IND_PARAM, &li_disc_ind);

	li_parm.wFunction = 0x0002; /* disconnect */
	li_parm.Param.ptr = &li_disc_ind;
	li_parm.Param_STRUCT = IE_STRUCT_DECODED;

	li_disc_ind.dwCid = CDID2CAPI_ID(cd->li_cdid_last);
	li_disc_ind.wServiceReason = 0x0000; /* user initiated */

	capi_ai_facility_ind(cd, 0x0005 /* line interconnect */,
			     0 /* no flags */, &li_parm);
	return;
}

/*---------------------------------------------------------------------------*
 *	capi_disconnect_broadcast - disconnect all broadcast applications
 *---------------------------------------------------------------------------*/
static void
capi_disconnect_broadcast(struct call_desc *cd, void *sc)
{
	if(cd->ai_type == I4B_AI_BROADCAST)
	{
	    /* set application interface before 
	     * disconnect indication
	     */
	    cd_set_appl_interface(cd,I4B_AI_CAPI,sc);

	    /* send disconnect indication
	     * to all other application interfaces
	     */
	    i4b_l4_disconnect_ind(cd,1);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	capi_connect_bridge - connect a call descriptor to a bridge
 *---------------------------------------------------------------------------*/
static void
capi_connect_bridge(struct call_desc *cd, void *sc, cdid_t dst_cdid)
{
    if(dst_cdid == CDID_UNUSED)
    {
        return;
    }

    if(cd && 
       (cd->li_cdid == CDID_UNUSED) &&
       (cd->li_data_ptr == NULL))
    {
        capi_disconnect_broadcast(cd, sc);

	cd->li_cdid = dst_cdid;
	cd->driver_type = DRVR_CAPI_BRIDGE;
	cd->driver_unit = 0;

	if(cd->channel_allocated)
	{
	    /* only do a bridge if there 
	     * is a channel allocated. Else
	     * the channel is maybe on hold.
	     */
	    mtx_lock(&i4b_global_lock);

	    cd->li_data_ptr = 
	      i4b_slot_li_alloc(cd->cdid, cd->li_cdid);

	    mtx_unlock(&i4b_global_lock);

	    if(i4b_link_bchandrvr(cd, 1))
	    {
	        NDBGL4(L4_MSG, "cdid=%d: could "
		       "not connect B-channel bridge "
		       "(maybe on hold)!", cd->cdid);
	    }
	}
    }
    return;
}

/*---------------------------------------------------------------------------*
 *	capi_disconnect_bridge - disconnect a call descriptor from a bridge
 *---------------------------------------------------------------------------*/
static void
capi_disconnect_bridge(struct call_desc *cd, void *sc, cdid_t dst_cdid)
{
    if(dst_cdid == CDID_UNUSED)
    {
        return;
    }

    if(cd && (cd->li_cdid == dst_cdid))
    {
        capi_disconnect_broadcast(cd, sc);

        if(cd->li_data_ptr)
	{
	    mtx_lock(&i4b_global_lock);

	    i4b_slot_li_free(cd->li_data_ptr);

	    mtx_unlock(&i4b_global_lock);

	    cd->li_data_ptr = NULL;
	}

	cd->li_cdid = CDID_UNUSED;
	cd->driver_type = DRVR_CAPI_B3;
	cd->driver_unit = 0;

	/* try to re-connect B-channel */

	if(i4b_link_bchandrvr(cd, 1))
        {
	    NDBGL4(L4_MSG, "cdid=%d: could "
		   "not connect B-channel bridge "
		   "(maybe on hold)!", cd->cdid);
	}
    }
    return;
}

/*---------------------------------------------------------------------------*
 *	capi_decode_b_protocol - decode the B_PROTOCOL structure
 *
 * Return values:
 *   0 success else failure
 *---------------------------------------------------------------------------*/
static uint8_t
capi_decode_b_protocol(struct call_desc *cd, 
		       struct CAPI_B_PROTOCOL_DECODED *b_prot)
{
    uint8_t error = 0;

    if (b_prot->wB1_protocol == 0) {
        if (cd->channel_bprot == BPROT_NONE) {
	    /* data over voice */
	    cd->channel_bprot = BPROT_RHDLC_DOV;
	} else {
	    cd->channel_bprot = BPROT_RHDLC;
	}
    } else if (b_prot->wB1_protocol == 1) {
        if (cd->channel_bprot == BPROT_RHDLC) {
	    /* voice over data */
	    cd->channel_bprot = BPROT_NONE_VOD;
	} else {
	    cd->channel_bprot = BPROT_NONE;
	}
    } else {
        NDBGL4(L4_MSG, "cdid=%d: unsupported "
	       "B1 protocol=%d ignored!",
	       cd->cdid, b_prot->wB1_protocol);
        error = 1;
    }

    if (b_prot->wB2_protocol != 1) {
        NDBGL4(L4_MSG, "cdid=%d: unsupported "
	       "B2 protocol=%d ignored!",
	       cd->cdid, b_prot->wB2_protocol);
	if (!error) {
	    error = 2;
	}
    }

    if(b_prot->wB3_protocol != 0) {
        NDBGL4(L4_MSG, "cdid=%d, unsupported "
	       "B3 protocol=%d ignored!",
	       cd->cdid, b_prot->wB3_protocol);
	if (!error) {
	    error = 3;
	}
    }

    cd->new_max_packet_size = b_prot->wMaxPacketSize;

    if (error) {
        cd->channel_bprot = BPROT_NONE;
    }

    /*
     * TODO: add support for decoding of
     * b_protocol.global_config
     */

    return error;
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

static const uint8_t cause_table[9] = {
	[0] = CAUSE_I4B_NORMAL, 
	[1] = CAUSE_I4B_NORMAL, 
	[2] = CAUSE_I4B_NORMAL,  /* Reject call, normal call clearing */
	[3] = CAUSE_I4B_BUSY,    /* Reject call, user busy */
	[4] = CAUSE_I4B_NOCHAN,  /* Reject call, requested circuit/channel not available */
	[5] = CAUSE_I4B_REJECT,  /* Reject call, facility rejected */
	[6] = CAUSE_I4B_NOCHAN,  /* Reject call, channel unacceptable */
	[7] = CAUSE_I4B_INCOMP,  /* Reject call, incompatible destination */
	[8] = CAUSE_I4B_OOO,     /* Reject call, destination out of order */
};

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

	/* NOTE: one has got to read all data into 
	 * buffers before locking the controller,
	 * hence uiomove() can sleep
	 */
	if(sc == NULL)
		return (ENXIO);

	mtx_lock(&sc->sc_mtx);
	error = (sc->sc_write_busy != 0);
	sc->sc_write_busy = 1;
	mtx_unlock(&sc->sc_mtx);

	if (error) {
		return (EBUSY);
	}

	if(uio->uio_resid < sizeof(sc->sc_msg.head))
	{
		error = EINVAL;
		goto done;
	}

	error = uiomove(&sc->sc_msg.head, sizeof(sc->sc_msg.head), uio);

	if(error)
	{
		goto done;
	}

	/* convert byte order in header to host format */

	sc->sc_msg.head.wLen = le16toh(sc->sc_msg.head.wLen);
	sc->sc_msg.head.wApp = le16toh(sc->sc_msg.head.wApp);
	sc->sc_msg.head.wCmd = le16toh(sc->sc_msg.head.wCmd);
	sc->sc_msg.head.wNum = le16toh(sc->sc_msg.head.wNum);
	sc->sc_msg.head.dwCid = le32toh(sc->sc_msg.head.dwCid);

	/* pack the command */
	sc->sc_msg.head.wCmd = CAPI_COMMAND_PACK(sc->sc_msg.head.wCmd);

	/* verify length */

	if(sc->sc_msg.head.wLen < sizeof(sc->sc_msg.head))
	{
		error = EINVAL;
		goto done;
	}

	/* remove header from length */

	sc->sc_msg.head.wLen -= sizeof(sc->sc_msg.head);

	/* verify length */

	if((sc->sc_msg.head.wLen > uio->uio_resid) ||
	   (sc->sc_msg.head.wLen > sizeof(sc->sc_msg.data)))
	{
		error = ENOMEM;
		goto done;
	}

	/* get rest of header into buffer */

	error = uiomove(&sc->sc_msg.data, sc->sc_msg.head.wLen, uio);

	if(error)
	{
		goto done;
	}

	/* rest of data goes into a mbuf, if any.
	 * This implementation allows zero length
	 * frames in case of DATA-B3 request.
	 */
	if(sc->sc_msg.head.wCmd == CAPI_P_REQ(DATA_B3))
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

	cntl = CNTL_FIND(CAPI_ID2CONTROLLER(sc->sc_msg.head.dwCid));

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
	    cd = cd_by_cdid(cntl, CAPI_ID2CDID(sc->sc_msg.head.dwCid));

	    if(!cd && (sc->sc_msg.head.wCmd == CAPI_P_REQ(CONNECT)))
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
	    switch(sc->sc_msg.head.wCmd) {

	      /* send ALERT request */

	    case CAPI_P_REQ(ALERT): /* ================================ */

	      if(cd->dir_incoming)
	      {
		  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(ALERT), 0x0000);

		  if(m2)
		  {
		      CAPI_INIT(CAPI_ALERT_REQ, &sc->sc_alert_req);

		      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_alert_req);

		      CAPI_INIT(CAPI_ADDITIONAL_INFO, &sc->sc_add_info);

		      capi_decode(sc->sc_alert_req.add_info.ptr,
				  sc->sc_alert_req.add_info.len, &sc->sc_add_info);

		      CAPI_INIT(CAPI_SENDING_COMPLETE, &sc->sc_sending_complete);

		      capi_decode(sc->sc_add_info.sending_complete.ptr,
				  sc->sc_add_info.sending_complete.len, 
				  &sc->sc_sending_complete);

		      N_ALERT_REQUEST(cd, 
				      (sc->sc_sending_complete.wMode == 0x0001) ?
				      /* send CALL PROCEEDING */ 1 :
				      /* send ALERT */ 0);
		  }
	      }
	      else
	      {
		  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(ALERT), 0x2001);
	      }
	      goto send_confirmation;

	      /* connect request, dial out to remote */

	    case CAPI_P_REQ(CONNECT): /* ============================== */

	      if(cd->dir_incoming)
	      {
		  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(CONNECT), 0x2001);
		  goto send_confirmation;
	      }

	      /* update CID value first */

	      sc->sc_msg.head.dwCid = CDID2CAPI_ID(cd->cdid);

	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(CONNECT), 0x0000);

	      capi_ai_putqueue(sc,0,m2,NULL);

	      if(m2 == NULL) break;

	      CAPI_INIT(CAPI_CONNECT_REQ, &sc->sc_connect_req);

	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_connect_req);

	      CAPI_INIT(CAPI_ADDITIONAL_INFO, &sc->sc_add_info);

	      capi_decode(sc->sc_connect_req.add_info.ptr,
			  sc->sc_connect_req.add_info.len, 
			  &sc->sc_add_info);

	      switch(sc->sc_connect_req.wCIP) {
	      case CAPI_CIP_SPEECH:
	      case CAPI_CIP_3100Hz_AUDIO:
	      case CAPI_CIP_TELEPHONY:
		cd->channel_bprot = BPROT_NONE;

		if (sc->sc_connect_req.BC.len >= 5) {
		    u_int8_t *temp = sc->sc_connect_req.BC.ptr;

		    switch(temp[4] & 0x1F) {
		    case 0x02:
		      cd->channel_bsubprot = BSUBPROT_G711_ULAW;
		      break;
		    case 0x03:
		      cd->channel_bsubprot = BSUBPROT_G711_ALAW;
		      break;
		    default:
		      cd->channel_bsubprot = BSUBPROT_UNKNOWN;
		      break;
		    }
		} else {
		    cd->channel_bsubprot = BSUBPROT_G711_ALAW;
		}
		break;
      
	      default:
		NDBGL4(L4_ERR, "cdid=%d, unknown CIP "
		       "value=0x%04x, fallback to "
		       "raw HDLC", cd->cdid, 
		       sc->sc_connect_req.wCIP);

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

	      /* get first destination party number */

	      capi_get_telno
		(cd, 
		 sc->sc_connect_req.dst_telno.ptr,
		 sc->sc_connect_req.dst_telno.len,
		 &(cd->dst_telno[0]), sizeof(cd->dst_telno)-1, NULL);

	      /* get first calling party number */

	      capi_get_telno
		(cd,
		 sc->sc_connect_req.src_telno.ptr,
		 sc->sc_connect_req.src_telno.len,
		 &(cd->src[0].telno[0]), sizeof(cd->src[0].telno)-1, 
		 &(cd->src[0]));

	      /* get second calling party number */

	      capi_get_telno
		(cd,
		 sc->sc_connect_req.src_telno_2.ptr,
		 sc->sc_connect_req.src_telno_2.len,
		 &(cd->src[1].telno[0]), sizeof(cd->src[1].telno)-1, 
		 &(cd->src[1]));

	      /* get first destination party subaddress */

	      capi_get_telno
		(cd,
		 sc->sc_connect_req.dst_subaddr.ptr,
		 sc->sc_connect_req.dst_subaddr.len,
		 &(cd->dst_subaddr[0]), sizeof(cd->dst_subaddr)-1, NULL);

	      /* get first calling party subaddress */

	      capi_get_telno
		(cd,
		 sc->sc_connect_req.src_subaddr.ptr,
		 sc->sc_connect_req.src_subaddr.len,
		 &(cd->src[0].subaddr[0]), sizeof(cd->src[0].subaddr)-1, NULL);


	      /* copy in keypad string */

	      if(sc->sc_add_info.keypad.len > (sizeof(cd->keypad)-1))
	      {
		  sc->sc_add_info.keypad.len = (sizeof(cd->keypad)-1);
	      }

	      bcopy(sc->sc_add_info.keypad.ptr, cd->keypad, 
		    sc->sc_add_info.keypad.len);

	      cd->keypad[sc->sc_add_info.keypad.len] = '\0';


	      /* copy in user-user string */

	      if(sc->sc_add_info.useruser.len > (sizeof(cd->user_user)-1))
	      {
		  sc->sc_add_info.useruser.len = (sizeof(cd->user_user)-1);
	      }

	      bcopy(sc->sc_add_info.useruser.ptr, cd->user_user,
		    sc->sc_add_info.useruser.len);

	      cd->user_user[sc->sc_add_info.useruser.len] = '\0';


	      /* copy in display string */

	      if(sc->sc_connect_req.display.len > (sizeof(cd->display)-1))
	      {
		  sc->sc_connect_req.display.len = (sizeof(cd->display)-1);
	      }

	      bcopy(sc->sc_connect_req.display.ptr, cd->display,
		    sc->sc_connect_req.display.len);

	      cd->display[sc->sc_connect_req.display.len] = '\0';

	      SET_CAUSE_TYPE(cd->cause_in, CAUSET_I4B);
	      SET_CAUSE_VAL(cd->cause_in, CAUSE_I4B_NORMAL);

	      if(sc->sc_connect_req.b_protocol.len)
	      {
		  CAPI_INIT(CAPI_B_PROTOCOL, &sc->sc_b_protocol);

		  capi_decode(sc->sc_connect_req.b_protocol.ptr,
			      sc->sc_connect_req.b_protocol.len, 
			      &sc->sc_b_protocol);

		  capi_decode_b_protocol(cd, &sc->sc_b_protocol);
	      }

	      if(sc->sc_connect_req.bFlag_0 & CAPI_FLAG0_WANT_LATE_INBAND)
	      {
		      cd->want_late_inband = 1;
	      }

	      CAPI_INIT(CAPI_SENDING_COMPLETE, &sc->sc_sending_complete);

	      capi_decode(sc->sc_add_info.sending_complete.ptr,
			  sc->sc_add_info.sending_complete.len, 
			  &sc->sc_sending_complete);

	      cd->sending_complete = (sc->sc_sending_complete.wMode == 0x0001);

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

	      CAPI_INIT(CAPI_DATA_B3_REQ, &sc->sc_data_b3_req);

	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_data_b3_req);

	      /* check that the B-channel is connected */

	      if((cd->fifo_translator_capi_std == NULL) || 
		 (cd->ai_type == I4B_AI_BROADCAST))
	      {
		  NDBGL4(L4_MSG, "cdid=%d: B-channel data sent "
			 "when disconnected!", cd->cdid);

		  m2 = capi_make_b3_conf
		    (&sc->sc_msg, sc->sc_data_b3_req.wHandle, 0x2001);
		  goto send_confirmation;
	      }

	      /* double check the frame length */

	      if(sc->sc_data_b3_req.wLen != m1->m_len)
	      {
		  NDBGL4(L4_ERR, "cdid=%d: invalid B-channel framelength: "
			 "%d <> %d!", cd->cdid, sc->sc_data_b3_req.wLen, 
			 m1->m_len);

		  m2 = capi_make_b3_conf
		    (&sc->sc_msg, sc->sc_data_b3_req.wHandle, 0x2007);
		  goto send_confirmation;
	      }

	      /* check queue length */

	      if(_IF_QFULL(&cd->fifo_translator_capi_std->tx_queue))
	      {
		  m2 = capi_make_b3_conf
		    (&sc->sc_msg, sc->sc_data_b3_req.wHandle, 0x1008);
		  goto send_confirmation;
	      }

	      /* make confirmation message */

	      m2 = capi_make_b3_conf
		(&sc->sc_msg, sc->sc_data_b3_req.wHandle, 0x0000);

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

	      _IF_ENQUEUE(&cd->fifo_translator_capi_std->tx_queue, m2);

	      L1_FIFO_START(cd->fifo_translator_capi_std);
	      break;


	    case CAPI_P_REQ(MANUFACTURER): /* ========================= */
	      /* nothing to do - not supported */
	      break;


	    case CAPI_P_REQ(CONNECT_B3): /* =========================== */

	      capi_disconnect_broadcast(cd, sc);

	      cd->driver_type = DRVR_CAPI_B3;
	      cd->driver_unit = 0;

	      /* update CID value first */

	      sc->sc_msg.head.dwCid = CDID2CAPI_ID(cd->cdid)|CAPI_ID_NCCI;

	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(CONNECT_B3), 0x0000);
	      capi_ai_putqueue(sc,0,m2,NULL);

	      if(i4b_link_bchandrvr(cd, 1))
	      {
		  /* XXX one could try to detect 
		   * failure earlier
		   */
		  NDBGL4(L4_MSG, "cdid=%d: could "
			 "not connect B-channel "
			 "(maybe on hold)!", cd->cdid);
	      }
	      break;

	    case CAPI_P_REQ(DISCONNECT_B3): /* ======================== */

	      if(cd->ai_type == I4B_AI_CAPI)
	      {
		  /* disconnect request, actively terminate connection */

		  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(DISCONNECT_B3), 0x0000);
		  capi_ai_putqueue(sc,0,m2,NULL);

		  (void)i4b_link_bchandrvr(cd, 0);
	      }
	      else
	      {
		  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(DISCONNECT_B3), 0x2001);
		  goto send_confirmation;
	      }
	      break;

	    case CAPI_P_REQ(DISCONNECT): /* =========================== */
#if 0
	      /* pretend that there was a disconnect collision, so
	       * that one does not have to send any confirmation 
	       * messages !
	       */
	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(DISCONNECT), 0x0000);
#endif
	      (void)i4b_link_bchandrvr(cd, 0);

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

	      CAPI_INIT(CAPI_INFO_REQ, &sc->sc_info_req);

	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_info_req);

	      CAPI_INIT(CAPI_ADDITIONAL_INFO, &sc->sc_add_info);

	      capi_decode(sc->sc_info_req.add_info.ptr,
			  sc->sc_info_req.add_info.len, &sc->sc_add_info);

	      if(cd->dir_incoming)
	      {
		  if(sc->sc_add_info.facility.len > 0)
		  {
		      /* 
		       * TODO: decode more of these messages 
		       */

		      /* check for progress request */
		      if(((u_int8_t *)(sc->sc_add_info.facility.ptr))[0] == 0x1e)
		      {
			  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(INFO), 0x0000);
			  if(m2)
			  {
			      N_PROGRESS_REQUEST(cd);
			  }
		      }
		      else
		      {
			  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(INFO), 0x300B);
		      }
		  }
		  else
		  {
		      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(INFO), 0x2001);
		  }
	      }
	      else
	      {
		  m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(INFO), 0x0000);
		  if(m2)
		  {
		      if(sc->sc_info_req.dst_telno.len > 0)
		      {
			  /* skip number plan byte */
			  sc->sc_info_req.dst_telno.len --;
			  sc->sc_info_req.dst_telno.ptr = 
			    ADD_BYTES(sc->sc_info_req.dst_telno.ptr,1);
		      }

		      i4b_l3_information_req(cd, sc->sc_info_req.dst_telno.ptr, 
					     sc->sc_info_req.dst_telno.len);
		  }
	      }
	      goto send_confirmation;


	    case CAPI_P_REQ(RESET_B3): /* ============================= */

	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(RESET_B3), 0x300D);

	      /* nothing to do - not supported */

	      goto send_confirmation;


	    case CAPI_P_REQ(SELECT_B_PROTOCOL): /* ==================== */

	      CAPI_INIT(CAPI_SELECT_B_PROTOCOL_REQ, &sc->sc_select_b_protocol_req);

	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_select_b_protocol_req);

	      if(sc->sc_select_b_protocol_req.b_protocol.len)
	      {
		  uint8_t berr;

		  CAPI_INIT(CAPI_B_PROTOCOL, &sc->sc_b_protocol);

		  capi_decode(sc->sc_select_b_protocol_req.b_protocol.ptr,
			      sc->sc_select_b_protocol_req.b_protocol.len, 
			      &sc->sc_b_protocol);

		  cd->channel_bprot = 0xff;

		  berr = capi_decode_b_protocol(cd, &sc->sc_b_protocol);

		  if (berr) {
		      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(SELECT_B_PROTOCOL), 
					  0x3000 | berr);
		      goto send_confirmation;
		  }
	      }

	      capi_disconnect_broadcast(cd, sc);

	      cd->driver_type = DRVR_CAPI_B3;
	      cd->driver_unit = 0;

	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(SELECT_B_PROTOCOL), 
				  0x0000);

	      if(cd->dir_incoming)
	      {
		  capi_ai_putqueue(sc,0,m2,NULL);

		  m2 = capi_make_connect_b3_ind(cd);
	      }
	      goto send_confirmation;


	    case CAPI_P_REQ(FACILITY): /* ============================= */

	      CAPI_INIT(CAPI_FACILITY_REQ, &sc->sc_facility_req);
 	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_facility_req);

	      if(sc->sc_facility_req.wSelector == 0x0001)
	      {
		  int err = EINVAL;

		  /* DTMF support */

		  CAPI_INIT(CAPI_FACILITY_REQ_DTMF_PARAM, &sc->sc_dtmf_req);
		  capi_decode(sc->sc_facility_req.Param.ptr,
			      sc->sc_facility_req.Param.len,
			      &sc->sc_dtmf_req);

		  switch(sc->sc_dtmf_req.wFunction) {
		  case 1:
		      /* enable DTMF detector */

		      if (cd->fifo_translator_capi_std) {

			err = L1_COMMAND_REQ(cntl, CMR_ENABLE_DTMF_DETECT, 
					     cd->fifo_translator_capi_std);
		      }
		      break;

		  case 2:
		      /* disable DTMF detector */

		      if (cd->fifo_translator_capi_std) {

			err = L1_COMMAND_REQ(cntl, CMR_DISABLE_DTMF_DETECT, 
					     cd->fifo_translator_capi_std);
		      }
		      break;

		  case 3:
		      /* generate DTMF tone(s) */

		      if (cd->fifo_translator_capi_std) {

			u_int16_t len = sc->sc_dtmf_req.Digits.len;
			u_int8_t *ptr = sc->sc_dtmf_req.Digits.ptr;

			while (len--) {
			    i4b_dtmf_queue_digit
			      (cd->fifo_translator_capi_std, *ptr++, 
			       sc->sc_dtmf_req.wToneDuration,
			       sc->sc_dtmf_req.wGapDuration);
			}
			err = 0;
		      }
		      break;
		  }
		  m2 = capi_make_dtmf_conf(&sc->sc_msg, err ? 
					   0x0002 /* unknown DTMF request */ : 
					   0x0000 /* success */);
		  goto send_confirmation;
	      }

	      if(sc->sc_facility_req.wSelector == 0x0008)
	      {
		  /* echo cancellation */
		  CAPI_INIT(CAPI_EC_FACILITY_PARM, &sc->sc_ec_parm);
		  capi_decode(sc->sc_facility_req.Param.ptr,
			      sc->sc_facility_req.Param.len,
			      &sc->sc_ec_parm);

		  if(sc->sc_ec_parm.wFunction == 0x0000)
		  {
		      /* get supported parameters */

		      m2 = capi_make_ec_supp_conf(&sc->sc_msg);
		      goto send_confirmation;
		  }
		  else if(sc->sc_ec_parm.wFunction == 0x0001)
		  {
		      /* enable echo canceller */

		      int err;

		      if (cd->fifo_translator_capi_std == NULL) {

			err = EINVAL;

		      } else {

			err = L1_COMMAND_REQ(cntl, CMR_ENABLE_ECHO_CANCEL, 
					     cd->fifo_translator_capi_std);
		      }

		      m2 = capi_make_ec_generic_conf
			(&sc->sc_msg, 0x0001, err ? 0x3011 : 0x0000);
		      goto send_confirmation;
		  }
		  else if(sc->sc_ec_parm.wFunction == 0x0002)
		  {
		      /* disable echo canceller */

		      int err;

		      if (cd->fifo_translator_capi_std == NULL) {

			err = EINVAL;

		      } else {

			err = L1_COMMAND_REQ(cntl, CMR_DISABLE_ECHO_CANCEL, 
					     cd->fifo_translator_capi_std);
		      }

		      m2 = capi_make_ec_generic_conf
			(&sc->sc_msg, 0x0002, err ? 0x3011 : 0x0000);
		      goto send_confirmation;
		  }
	      }

	      if(sc->sc_facility_req.wSelector == 0x0003)
	      {
		  /* supplementary services */
		  CAPI_INIT(CAPI_SUPPL_PARAM, &sc->sc_suppl_param);
		  capi_decode(sc->sc_facility_req.Param.ptr,
			      sc->sc_facility_req.Param.len,
			      &sc->sc_suppl_param);

		  if(sc->sc_suppl_param.wFunction == 0x0000)
		  {
		      /* get supplementary services */

		      m2 = capi_make_suppl_supp_conf(&sc->sc_msg);
		      goto send_confirmation;
		  }
		  else if(sc->sc_suppl_param.wFunction == 0x0002)
		  {
		      /* HOLD request */

		      N_HOLD_REQUEST(cd);

		      m2 = capi_make_fac_suppl_conf
			(&sc->sc_msg, 0x0002, NULL);
		      goto send_confirmation;
		  }
		  else if(sc->sc_suppl_param.wFunction == 0x0003)
		  {
		      /* RETRIEVE request */

		      N_RETRIEVE_REQUEST(cd);

		      m2 = capi_make_fac_suppl_conf
			(&sc->sc_msg, 0x0003, NULL);
		      goto send_confirmation;
		  }
		  else if(sc->sc_suppl_param.wFunction == 0x000D)
		  {
		      /* call deflection */

		      CAPI_INIT(CAPI_FACILITY_REQ_CALL_DEFL_PARAM, &sc->sc_cd_req);
		      capi_decode(sc->sc_suppl_param.Param.ptr,
				  sc->sc_suppl_param.Param.len,
				  &sc->sc_cd_req);

		      capi_get_fac_telno
			(cd, 
			 sc->sc_cd_req.dst_telno.ptr,
			 sc->sc_cd_req.dst_telno.len,
			 &(cd->dst_telno_part[0]), 
			 sizeof(cd->dst_telno_part)-1);

		      N_DEFLECT_REQUEST(cd);

		      m2 = capi_make_fac_suppl_conf_type1
			(&sc->sc_msg, 0x000D, 0x0000);
		      goto send_confirmation;
		  }
		  else if(sc->sc_suppl_param.wFunction == 0x000E)
		  {
		      /* MCID */

		      N_MCID_REQUEST(cd);

		      m2 = capi_make_fac_suppl_conf
			(&sc->sc_msg, 0x000E, NULL);
		      goto send_confirmation;
		  }
	      }

	      if(sc->sc_facility_req.wSelector == 0x0005)
	      {
		  /* line interconnect */

		  CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &sc->sc_li_param);
		  capi_decode(sc->sc_facility_req.Param.ptr, sc->sc_facility_req.Param.len, 
			      &sc->sc_li_param);

		  switch(sc->sc_li_param.wFunction) {
		  case 0x0000:
		      /* get supported services */
		      m2 = capi_make_li_supp_conf(&sc->sc_msg);
		      goto send_confirmation;

		  case 0x0001:
		      /* connect */
		      CAPI_INIT(CAPI_LI_CONN_REQ_PARAM, &sc->sc_li_conn_req_param);
		      capi_decode(sc->sc_li_param.Param.ptr, sc->sc_li_param.Param.len, 
				  &sc->sc_li_conn_req_param);

		      CAPI_INIT(CAPI_GENERIC_STRUCT, &sc->sc_gen_struct);
		      capi_decode(sc->sc_li_conn_req_param.conn_req_part.ptr,
				  sc->sc_li_conn_req_param.conn_req_part.len,
				  &sc->sc_gen_struct);

		      CAPI_INIT(CAPI_LI_CONN_REQ_PART, &sc->sc_li_conn_req_part);
		      capi_decode(sc->sc_gen_struct.Param.ptr,
				  sc->sc_gen_struct.Param.len,
				  &sc->sc_li_conn_req_part);

		      if(sc->sc_li_conn_req_param.dwDataPath != 0x00000000)
		      {
			  m2 = capi_make_li_conn_conf(&sc->sc_msg, 0x2008, 
						      sc->sc_li_conn_req_part.dwCid);
			  goto send_confirmation;
		      }

		      m2 = capi_make_li_conn_conf(&sc->sc_msg, 0x0000,
						  sc->sc_li_conn_req_part.dwCid);
		      capi_ai_putqueue(sc,0,m2,NULL);

		      capi_connect_bridge(cd, sc, 
					  CAPI_ID2CDID(sc->sc_li_conn_req_part.dwCid));
		      CNTL_UNLOCK(cntl);

		      cntl = CNTL_FIND(CAPI_ID2CONTROLLER(sc->sc_li_conn_req_part.dwCid));

		      if(cntl)
		      {
			  CNTL_LOCK(cntl);

			  cd = cd_by_cdid(cntl, CAPI_ID2CDID(sc->sc_li_conn_req_part.dwCid));

			  capi_connect_bridge(cd, sc,
					      CAPI_ID2CDID(sc->sc_msg.head.dwCid));
		      }
		      break;

		  case 0x0002:
		      /* disconnect */
		      CAPI_INIT(CAPI_LI_DISC_REQ_PARAM, &sc->sc_li_disc_req_param);
		      capi_decode(sc->sc_li_param.Param.ptr, sc->sc_li_param.Param.len, 
				  &sc->sc_li_disc_req_param);

		      CAPI_INIT(CAPI_GENERIC_STRUCT, &sc->sc_gen_struct);
		      capi_decode(sc->sc_li_disc_req_param.disc_req_part.ptr,
				  sc->sc_li_disc_req_param.disc_req_part.len,
				  &sc->sc_gen_struct);

		      CAPI_INIT(CAPI_LI_DISC_REQ_PART, &sc->sc_li_disc_req_part);
		      capi_decode(sc->sc_gen_struct.Param.ptr,
				  sc->sc_gen_struct.Param.len,
				  &sc->sc_li_disc_req_part);

		      m2 = capi_make_li_disc_conf(&sc->sc_msg, 0x0000,
						  sc->sc_li_disc_req_part.dwCid);
		      capi_ai_putqueue(sc,0,m2,NULL);

		      capi_disconnect_bridge(cd, sc,
					     CAPI_ID2CDID(sc->sc_li_disc_req_part.dwCid));
		      CNTL_UNLOCK(cntl);

		      cntl = CNTL_FIND(CAPI_ID2CONTROLLER(sc->sc_li_disc_req_part.dwCid));

		      if(cntl)
		      {
			  CNTL_LOCK(cntl);

			  cd = cd_by_cdid(cntl, CAPI_ID2CDID(sc->sc_li_disc_req_part.dwCid));
		      
			  capi_disconnect_bridge(cd, sc, 
						 CAPI_ID2CDID(sc->sc_msg.head.dwCid));
		      }
		      break;

		  default:
		      m2 = capi_make_facility_conf
			(&sc->sc_msg, sc->sc_facility_req.wSelector, 0x3011, NULL);
		      goto send_confirmation;
		  }
		  break;
	      }

	      /* not supported */

	      m2 = capi_make_facility_conf
		(&sc->sc_msg, sc->sc_facility_req.wSelector, 0x300B, NULL);

	      goto send_confirmation;


	      /* connect response, accept/reject/ignore incoming call */

	    case CAPI_P_RESP(CONNECT): /* ============================= */

	      if(cd->dir_incoming == 0)
	      {
		  /* ignore message */
		  break;
	      }

	      CAPI_INIT(CAPI_CONNECT_RESP, &sc->sc_connect_resp);

	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_connect_resp);

	      if (sc->sc_connect_resp.wReject < 9) {
		  cause = (CAUSET_I4B << 8) | cause_table[sc->sc_connect_resp.wReject];

		  if(sc->sc_connect_resp.wReject == 0) {

			/* accept the call */
			response = SETUP_RESP_ACCEPT;

		  } else if(sc->sc_connect_resp.wReject == 1) {

			/* ignore the call */
			response = SETUP_RESP_DNTCRE;

			N_CONNECT_RESPONSE(cd, response, cause);
			break;

		  } else {

			/* reject the call */
			response = SETUP_RESP_REJECT;
		  }
	      }
	      else if((sc->sc_connect_resp.wReject & 0xFF00) == 0x3400)
	      {
			cause = 
			  (CAUSET_Q850 << 8) | 
			  (sc->sc_connect_resp.wReject & 0x7F);
			response = SETUP_RESP_REJECT;
	      }
	      else
	      {
			cause = (CAUSET_I4B << 8) | CAUSE_I4B_NORMAL;
			response = SETUP_RESP_REJECT;
	      }

	      if(sc->sc_connect_resp.b_protocol.len)
	      {
		  uint8_t berr;

		  CAPI_INIT(CAPI_B_PROTOCOL, &sc->sc_b_protocol);

		  capi_decode(sc->sc_connect_resp.b_protocol.ptr,
			      sc->sc_connect_resp.b_protocol.len, &sc->sc_b_protocol);

		  cd->channel_bprot = 0xff;

		  berr = capi_decode_b_protocol(cd, &sc->sc_b_protocol);
	      }

	      capi_disconnect_broadcast(cd, sc);

	      cd->driver_type = DRVR_CAPI_B3;
	      cd->driver_unit = 0;

	      cd->shorthold_data.shorthold_algorithm = SHA_FIXU;
	      cd->shorthold_data.unitlen_time = 0;
	      cd->shorthold_data.earlyhup_time = 0;
	      cd->shorthold_data.idle_time = 0; /* seconds (disabled) */

	      cd->isdntxdelay = 0; /* seconds (disabled) */

	      cd->odate_time_len = min(sc->sc_connect_resp.date_time.len,
				       sizeof(cd->odate_time_data));

	      bcopy(sc->sc_connect_resp.date_time.ptr,
		    cd->odate_time_data,
		    cd->odate_time_len);

	      N_CONNECT_RESPONSE(cd, response, cause);
	      break;

	    case CAPI_P_RESP(CONNECT_ACTIVE):
	      if(cd->dir_incoming && (cd->ai_type == I4B_AI_CAPI))
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

		  if(cd->fifo_translator_capi_std == NULL)
		  {
		      m2 = capi_make_connect_b3_ind(cd);
		      goto send_confirmation;
		  }
	      }
	      break;

	    case CAPI_P_RESP(CONNECT_B3):
	      if(cd->dir_incoming && (cd->ai_type == I4B_AI_CAPI))
	      {
		  CAPI_INIT(CAPI_CONNECT_B3_RESP, &sc->sc_connect_b3_resp);

		  capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_connect_b3_resp);

		  if(sc->sc_connect_b3_resp.wReject)
		  {
		      /* preset causes with our cause */

		      cd->cause_in = cd->cause_out = 
			(CAUSET_I4B << 8) | CAUSE_I4B_REJECT;

		      N_DISCONNECT_REQUEST(cd, cd->cause_in);

		      cd = NULL; /* call descriptor is freed ! */
		  }
		  else
		  {
		      if(i4b_link_bchandrvr(cd, 1))
		      {
			  /* XXX one could try to detect 
			   * failure earlier
			   */
			  NDBGL4(L4_MSG, "cdid=%d: could "
				 "not connect B-channel "
				 "(maybe on hold)!", cd->cdid);
		      }
		  }
	      }
	      break;

	    case CAPI_P_RESP(CONNECT_B3_ACTIVE):
	      if(cd->ai_type == I4B_AI_CAPI)
	      {

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
	      capi_ai_putqueue(sc,0,m2,NULL);
	      break;

	    default:

	      NDBGL4(L4_ERR, "cdid=%d: Unknown frame: "
		     "wCmd=0x%04x (ignored)!",
		     cd->cdid, sc->sc_msg.head.wCmd);

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
		       cd->cdid, sc->sc_msg.head.wCmd);

		/* do nothing ! */
	  }
	}
	else
	{
	    switch(sc->sc_msg.head.wCmd) {
	    case CAPI_P_REQ(LISTEN):/* ================================ */

	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(LISTEN), 0x0000);

	      if(m2)
	      {
		  u_int8_t controller = (sc->sc_msg.head.dwCid & 0xFF) % MAX_CONTROLLERS;

		  CAPI_INIT(CAPI_LISTEN_REQ, &sc->sc_listen_req);

		  capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, &sc->sc_listen_req);

		  mtx_lock(&sc->sc_mtx);

		  sc->sc_info_mask[controller]  = sc->sc_listen_req.dwInfoMask;
		  sc->sc_CIP_mask_1[controller] = sc->sc_listen_req.dwCipMask1;
		  sc->sc_CIP_mask_2[controller] = sc->sc_listen_req.dwCipMask2;

		  mtx_unlock(&sc->sc_mtx);
	      }
	      goto send_confirmation;

	    case CAPI_P_REQ(CONNECT): /* ============================== */

	      m2 = capi_make_conf(&sc->sc_msg, CAPI_CONF(CONNECT), 0x2003);

	      goto send_confirmation;

	    case CAPI_P_REQ(FACILITY):

	      CAPI_INIT(CAPI_FACILITY_REQ, &sc->sc_facility_req);
 	      capi_decode(&sc->sc_msg.data, sc->sc_msg.head.wLen, 
			  &sc->sc_facility_req);

	      if(sc->sc_facility_req.wSelector == 0x0003)
	      {
		  /* supplementary services */
		  CAPI_INIT(CAPI_SUPPL_PARAM, &sc->sc_suppl_param);
		  capi_decode(sc->sc_facility_req.Param.ptr,
			      sc->sc_facility_req.Param.len,
			      &sc->sc_suppl_param);

		  if(sc->sc_suppl_param.wFunction == 0x0000)
		  {
		      /* get supplementary services */

		      m2 = capi_make_suppl_supp_conf(&sc->sc_msg);
		      goto send_confirmation;
		  }
	      }
	      if(sc->sc_facility_req.wSelector == 0x0005)
	      {
		  /* line interconnect */

		  CAPI_INIT(CAPI_LINE_INTERCONNECT_PARAM, &sc->sc_li_param);
		  capi_decode(sc->sc_facility_req.Param.ptr, sc->sc_facility_req.Param.len, 
			      &sc->sc_li_param);

		  if(sc->sc_li_param.wFunction == 0x0000)
		  {
		      /* get supported services */
		      m2 = capi_make_li_supp_conf(&sc->sc_msg);
		      goto send_confirmation;
		  }
	      }
	      break;

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
		     sc->sc_msg.head.dwCid, sc->sc_msg.head.wCmd);

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
	if (m1) {
		m_freem(m1);
	}

	mtx_lock(&sc->sc_mtx);
	sc->sc_write_busy = 0;
	mtx_unlock(&sc->sc_mtx);

	return (error);
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

		if(req->max_b_data_len < MIN_B_DATA_LEN) {
		   req->max_b_data_len = MIN_B_DATA_LEN;
		}

		if(req->max_b_data_blocks > 128) {
		   req->max_b_data_blocks = 128;
		}

		mtx_lock(&sc->sc_mtx);
		sc->sc_max_b_data_len = req->max_b_data_len;
		sc->sc_max_b_data_blocks = req->max_b_data_blocks;
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

	case CAPI_SET_STACK_VERSION_REQ:
	{
		u_int32_t *stack_version = (void *)data;

		/* reject invalid stack versions */
		if(stack_version[0] < 204)
		{
		    error = EINVAL;
		}
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
		  htole32(CAPI_PROFILE_INTERNAL_CTLR_SUPPORT|
			  CAPI_PROFILE_ECHO_CANCELLATION|
			  CAPI_PROFILE_DTMF_SUPPORT|
			  CAPI_PROFILE_SUPPLEMENTARY_SERVICES);

		CNTL_LOCK(cntl);
		if(cntl->L1_pcm_cable_end != 0)
		{
		    /* if line interconnect is configured,
		     * it is supported, though really one
		     * should also check layer 1:
		     */
		    req->profile.dwGlobalOptions |= 
		      htole32(CAPI_PROFILE_LINE_INTERCONNECT);
		}
		CNTL_UNLOCK(cntl);

		req->profile.dwB1ProtocolSupport = 
		  htole32((1 << CAPI_B1_HDLC_64) |
			  (1 << CAPI_B1_TRANSPARENT_64));

		req->profile.dwB2ProtocolSupport = 
		  htole32(/*(1 << CAPI_B2_ISO7776_X75_SLP) | XXX */
			  (1 << CAPI_B2_TRANSPARENT));

		req->profile.dwB3ProtocolSupport = 
		  htole32((1 << CAPI_B3_TRANSPARENT));

		/* forward ioctl to layer 1 (ignore any errors) */

		(void) L1_COMMAND_REQ(cntl, CMR_CAPI_GET_PROFILE, data);

		break;
	}

	case I4B_CTL_CAPI_DELEGATE:
	{
		i4b_capi_delegate_t *capi_dg = (void *)data;

		/* check credentials */

		if(suser(curthread))
		{
		    error = EPERM;
		    break;
		}

		if ((capi_dg->uid >= 0xffffffff) ||
		    (capi_dg->gid >= 0xffffffff))
		{
		    error = EINVAL;
		    break;
		}

		mtx_lock(&i4b_global_lock);

		capi_ai_global_lock(1);

		if (capi_ai_update_delegate(capi_dg)) {
		    error = ENOMEM;
		}

		capi_ai_global_lock(0);

		mtx_unlock(&i4b_global_lock);

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

	case I4B_CTRL_INFO_REQ:
	{
		msg_ctrl_info_req_t *mcir = (void *)data;

		/* get controller */

		cntl = CNTL_FIND(mcir->controller);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* forward IOCTL to layer 1 */
		error = L1_COMMAND_REQ(cntl, CMR_INFO_REQUEST, mcir);
		break;
	}

	case I4B_PROT_IND:
	{
		msg_prot_ind_t *mpi = (void *)data;

		/* check credentials */

		if(suser(curthread))
		{
		    error = EPERM;
		    break;
		}

		/* get controller */

		cntl = CNTL_FIND(mpi->controller);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* forward IOCTL to layer 1 */
		error = L1_COMMAND_REQ(cntl, CMR_SET_LAYER2_PROTOCOL, mpi);
		break;
	}

	case I4B_VR_REQ:
		i4b_version_request((void *)data);
		break;

	case I4B_CTL_SET_PCM_SLOT_END:
	{
		i4b_debug_t *dbg = (void *)data;
		struct i4b_pcm_cable *cable;

		/* check credentials */

		if(suser(curthread))
		{
		    error = EPERM;
		    break;
		}

		/* check parameter range */

		if((dbg->unit >= I4B_PCM_CABLE_MAX) ||
		   (dbg->value > I4B_PCM_SLOT_MAX) ||
		   (dbg->value & 1))
		{
		    error = EINVAL;
		    break;
		}

		cable = &i4b_pcm_cable[dbg->unit];

		mtx_lock(&i4b_global_lock);

		cable->slot_end = dbg->value;

		mtx_unlock(&i4b_global_lock);

		break;
	}

	case I4B_CTL_SET_I4B_OPTIONS:
		cmd = CMR_SET_I4B_OPTIONS;
		goto L1_command;

	case I4B_CTL_PH_ACTIVATE:
		cmd = CMR_PH_ACTIVATE;
		goto L1_command;

	case I4B_CTL_PH_DEACTIVATE:
		cmd = CMR_PH_DEACTIVATE;
		goto L1_command;

	case I4B_CTL_RESET:
		cmd = CMR_RESET;
		goto L1_command;

	case I4B_CTL_SET_POWER_SAVE:
		cmd = CMR_SET_POWER_SAVE;
		goto L1_command;

	case I4B_CTL_SET_POWER_ON:
		cmd = CMR_SET_POWER_ON;
		goto L1_command;

	case I4B_CTL_SET_PCM_MAPPING:
		cmd = CMR_SET_PCM_MAPPING;
		goto L1_command;

	case I4B_CTL_GET_EC_FIR_FILTER:
		cmd = CMR_GET_EC_FIR_FILTER;
		goto L1_command;

	L1_command:
	{
		i4b_debug_t *dbg = (void *)data;

		/* check credentials */

		if(suser(curthread))
		{
		    error = EPERM;
		    break;
		}

		/* get controller */

		cntl = CNTL_FIND(dbg->unit);

		if(cntl == NULL)
		{
		    error = EINVAL;
		    break;
		}

		/* forward IOCTL to layer 1 */
		error = L1_COMMAND_REQ(cntl, cmd, dbg);
		break;
	}

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

#define CAPI_CUSTOM_DTMF_IND(m,n) \
  m(n, BYTE_ARRAY, Digits, 1) \
  END

CAPI_MAKE_STRUCT(CAPI_CUSTOM_DTMF_IND);

/*---------------------------------------------------------------------------*
 *	this routine is called from the HSCX interrupt handler
 *	when a DTMF digit has been detected
 *---------------------------------------------------------------------------*/
static void
capi_put_dtmf(struct fifo_translator *f, u_int8_t *dtmf_ptr, u_int16_t dtmf_len)
{
	struct call_desc *cd = f->L5_sc;
	struct CAPI_CUSTOM_DTMF_IND_DECODED dtmf_data;

	memset(&dtmf_data, 0, sizeof(dtmf_data));

	CAPI_INIT(CAPI_CUSTOM_DTMF_IND, &dtmf_data);

	while (dtmf_len--) {

	    dtmf_data.Digits[0] = *dtmf_ptr++;

	    capi_ai_facility_ind(cd, 0x0001 /* DTMF selector */,
				 CAPI_PUTQUEUE_FLAG_DROP_OK, &dtmf_data);
	}
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
		capi_ai_putqueue(sc,CAPI_PUTQUEUE_FLAG_DROP_OK,m2,NULL);
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
			capi_ai_putqueue(sc,0,m2,NULL);
		}
	}
	else
	{
		/* no data to send */
		m1 = NULL;
	}

	i4b_dtmf_generate(f, &m1);

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

	if (def_len > cd->curr_max_packet_size) {
	    def_len = cd->curr_max_packet_size;
	}

	return i4b_getmbuf(def_len, M_NOWAIT);
}

/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for I4B-CAPI
 *---------------------------------------------------------------------------*/
fifo_translator_t *
capi_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
	      struct i4b_protocol *pp, u_int32_t driver_type, 
	      u_int32_t driver_unit, call_desc_t *cd)
{
	if(!cd)
	{
		return FT_INVALID;
	}

	if(!pp)
	{
		return cd->fifo_translator_capi_std;
	}

	cd->fifo_translator_capi_std = f; 

	NDBGL4(L4_MSG, "capi, cdid=%d, protocol_1=%d", 
	       cd->cdid, pp->protocol_1);

	if(pp->protocol_1)
	{
		struct capi_ai_softc *sc = cd->ai_ptr;

		/* connected */

		f->L5_sc = cd;

		f->L5_PUT_MBUF = &capi_put_mbuf;
		f->L5_PUT_DTMF = &capi_put_dtmf;
		f->L5_GET_MBUF = &capi_get_mbuf;
		f->L5_ALLOC_MBUF = &capi_alloc_mbuf;

		if (sc == NULL) {
		    pp->protocol_1 = P_DISABLE;
		} else {
		    mtx_lock(&sc->sc_mtx);
		    f->tx_queue.ifq_maxlen = sc->sc_max_b_data_blocks;

		    if (cd->new_max_packet_size == 0) {
		        cd->curr_max_packet_size = sc->sc_max_b_data_len;
		    } else {
		        cd->curr_max_packet_size = cd->new_max_packet_size;
		    }
		    if (cd->curr_max_packet_size < MIN_B_DATA_LEN) {
		        cd->curr_max_packet_size = MIN_B_DATA_LEN;
		    }
		    mtx_unlock(&sc->sc_mtx);
		}

		capi_ai_connect_b3_active_ind(cd);
	}
	else
	{
		/* disconnected */

		capi_ai_disconnect_b3_ind(cd);
	}
	return f;
}


/*---------------------------------------------------------------------------*
 *	setup the FIFO-translator for I4B-CAPI-BRIDGE
 *---------------------------------------------------------------------------*/
fifo_translator_t *
capi_bridge_setup_ft(i4b_controller_t *cntl, fifo_translator_t *f, 
		     struct i4b_protocol *pp, u_int32_t driver_type, 
		     u_int32_t driver_unit, call_desc_t *cd)
{
	struct i4b_line_interconnect *li;

	if(!cd)
	{
		return FT_INVALID;
	}

	if(!pp)
	{
		return cd->fifo_translator_capi_bridge;
	}

	cd->fifo_translator_capi_bridge = f; 

	NDBGL4(L4_MSG, "capi, cdid=%d, protocol_1=%d", 
	       cd->cdid, pp->protocol_1);

	if(pp->protocol_1)
	{
		/* connected */

		cd->li_cdid_last = cd->li_cdid;

		li = cd->li_data_ptr;

		if(li)
		{
		    pp->protocol_1 = P_BRIDGE;
		    pp->u.bridge.rx_slot = li->pcm_slot_rx;
		    pp->u.bridge.tx_slot = li->pcm_slot_tx;
		    pp->u.bridge.rx_cable = li->pcm_cable;
		    pp->u.bridge.tx_cable = li->pcm_cable;

		    capi_ai_line_inter_connect_ind(cd);
		}
		else
		{
		    pp->protocol_1 = P_DISABLE;
		}
	}
	else
	{
		/* disconnected */

		capi_ai_line_inter_disconnect_ind(cd);
	}
	return f;
}
