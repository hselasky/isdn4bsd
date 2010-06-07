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
 *	i4b_l4mgmt.c - layer 4 calldescriptor management utilites
 *	---------------------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>

#include <sys/socket.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#include <i4b/layer4/i4b_l4.h>

/*---------------------------------------------------------------------------*
 *	get a new unique CDID value,
 *	----------------------------
 *	which is used to uniquely identify a single call [-descriptor]
 *	in the communication between kernel and userland
 *---------------------------------------------------------------------------*/
static u_int
get_cdid(struct i4b_controller *cntl)
{
	u_int8_t timeout = 1;
	struct call_desc *cd;
	u_int new_cdid;

	CNTL_LOCK_ASSERT(cntl);

	if(cntl->N_cdid_end > CDID_REF_MAX)
	{
	  printf("%s: %s: controller %d: N_cdid_end set "
		 "too high: %d!\n", __FILE__, __FUNCTION__, 
		 cntl->unit, cntl->N_cdid_end);
	  cntl->N_cdid_end = CDID_REF_MAX;
	}

 again:
	/* NOTE: "N_cdid_xxx" must be multiplied by
	 * "MAX_CONTROLLERS" to get the real cdid value !
	 */

	/* get next ID */
	cntl->N_cdid_count++;

	/* range-check */
	if((cntl->N_cdid_count <= 0) ||
	   (cntl->N_cdid_count >= cntl->N_cdid_end))
	{
	  if(!timeout--)
	  {
	    /* no CDID value available */
	    return CDID_UNUSED;
	  }
	  cntl->N_cdid_count = 1;
	}

	new_cdid = MAKE_CDID(cntl->unit, cntl->N_cdid_count);

	/* check if ID already in use */
	CD_FOREACH(cd, cntl)
	{
		if(cd->cdid == new_cdid)
		{
			goto again;
		}
	}

	return new_cdid;
}

/*---------------------------------------------------------------------------*
 *      reserve a call descriptor for later usage
 *      ----------------------------------------
 *      searches the call descriptor array until an unused
 *      descriptor is found, gets a new call descriptor id
 *      and reserves it by putting the id into the cdid field.
 *      returns pointer to the call descriptor.
 *---------------------------------------------------------------------------*/
struct call_desc *
i4b_allocate_cd(struct i4b_controller *cntl)
{
	struct call_desc *cd;

	CNTL_LOCK_ASSERT(cntl);

	CD_FOREACH(cd, cntl)
	{
		if(cd->cdid == CDID_UNUSED)
		{
			/* clear call descriptor */
			bzero(cd, sizeof(*cd));

			cd->cdid = get_cdid(cntl);	/* fill in new cdid */
			cd->p_cntl = cntl;

			if(cd->cdid == CDID_UNUSED)
			{
				goto error;
			}

			usb_callout_init_mtx(&cd->idle_callout, 
					   CNTL_GET_LOCK(cntl), 0);

			usb_callout_init_mtx(&cd->set_state_callout, 
					   CNTL_GET_LOCK(cntl), 0);

			NDBGL4(L4_MSG, "found free cd - "
			       "cdid=%d", cd->cdid);

  			goto done;
		}
	}

 error:
	cd = NULL;

 done:
 	return(cd);
}

/*---------------------------------------------------------------------------*
 *      free a call descriptor
 *      ---------------------
 *      free an unused call descriptor by giving address of call descriptor
 *      and writing a 0 into the cdid field marking it as unused.
 *---------------------------------------------------------------------------*/
void
i4b_free_cd(struct call_desc *cd)
{
	NDBGL4(L4_MSG, "releasing cd - cdid=%u, cr=%d",
	       cd->cdid, cd->cr);

	cd->cdid = CDID_UNUSED;

	usb_callout_stop(&cd->idle_callout);
	usb_callout_stop(&cd->set_state_callout);

	return;
}

/*---------------------------------------------------------------------------*
 *	allocate channel for a call descriptor
 *---------------------------------------------------------------------------*/
void
cd_allocate_channel(struct call_desc *cd)
{
	struct i4b_controller *cntl = i4b_controller_by_cd(cd);

	CNTL_LOCK_ASSERT(cntl);

	if((cd->li_cdid) && (cd->li_data_ptr == NULL))
	{
	    mtx_lock(&i4b_global_lock);

	    cd->li_data_ptr = 
	      i4b_slot_li_alloc(cd->cdid, cd->li_cdid);

	    mtx_unlock(&i4b_global_lock);
	}

	if(cd->channel_allocated == 0)
	{
	    __typeof(cntl->L1_channel_end)
	      channel_end = cntl->L1_channel_end;

	    if((cd->channel_id >= 0) &&
	       (cd->channel_id < channel_end))
	    {
	        if(GET_CHANNEL_UTILIZATION(cntl,cd->channel_id) == 0)
		{
		    goto found;
		}
		else
		{
		    goto not_found;
		}
	    }
	    else
	    {
	        if(cd->channel_id == CHAN_NOT_ANY)
		{
		    goto not_any;
		}
		else
		{
		    for(cd->channel_id = 0;
			cd->channel_id < channel_end;
			cd->channel_id++)
		    {
		        if(GET_CHANNEL_UTILIZATION(cntl,cd->channel_id) == 0)
			{
			    goto found;
			}
		    }
		not_found:
		    cd->channel_id = CHAN_ANY;
		    goto done;
		found:
		    SET_CHANNEL_UTILIZATION(cntl,cd->channel_id,1);
		not_any:
		    cd->channel_allocated = 1;
		    goto done;
		}
	    }
	}
 done:
	return;
}

/*---------------------------------------------------------------------------*
 *	free channel for a call descriptor
 *---------------------------------------------------------------------------*/
void
cd_free_channel(struct call_desc *cd)
{
	struct i4b_controller *cntl = i4b_controller_by_cd(cd);

	CNTL_LOCK_ASSERT(cntl);

	if(cd->li_data_ptr)
	{
	    mtx_lock(&i4b_global_lock);

	    i4b_slot_li_free(cd->li_data_ptr);

	    mtx_unlock(&i4b_global_lock);

	    cd->li_data_ptr = NULL;
	}

	if(cd->channel_allocated)
	{
	    __typeof(cntl->L1_channel_end)
	      channel_end = cntl->L1_channel_end;

	    if((cd->channel_id >= 0) && 
	       (cd->channel_id < channel_end))
	    {
	        SET_CHANNEL_UTILIZATION(cntl,cd->channel_id,0);
	    }

	    cd->channel_id = CHAN_ANY;
	    cd->channel_allocated = 0;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	set application interface for a call descriptor
 *---------------------------------------------------------------------------*/
void
cd_set_appl_interface(struct call_desc *cd, u_int8_t appl_interface_type,
		      void *appl_interface_ptr)
{
	/* update application interface information */

	cd->ai_type = appl_interface_type;
	cd->ai_ptr = appl_interface_ptr;
	return;
}

/*---------------------------------------------------------------------------*
 *      return pointer to call descriptor by giving the call descriptor id
 *      ----------------------------------------------------------------
 *      lookup a call descriptor in the call descriptor array by looking
 *      at the cdid field. return pointer to call descriptor if found,
 *      else return NULL if not found.
 *---------------------------------------------------------------------------*/
struct call_desc *
cd_by_cdid(struct i4b_controller *cntl, unsigned int cdid)
{
	struct call_desc *cd;

	CNTL_LOCK_ASSERT(cntl);

	if(cdid != CDID_UNUSED)
	{
	    CD_FOREACH(cd, cntl)
	    {
		if(cd->cdid == cdid)
		{
			NDBGL4(L4_MSG, "found cdid - cdid=%u cr=%d",
			       cd->cdid, cd->cr);

			return(cd);
		}
	    }
	}
	return(NULL);
}

/*
 * Transmitted CR
 * +---------+----------------+
 * | CR_FLAG | callreference  |
 * +---------+----------------+
 *
 * CR_FLAG:
 *  0: originator of callreference
 *  1: non-originator of callreference
 *
 * incoming CR_FLAG is inverted
 * outgoing calls use "0" for CR_FLAG
 */

/*---------------------------------------------------------------------------*
 *      search call descriptor
 *      ---------------------
 *      This routine searches for the call descriptor for a passive controller
 *      given by unit number, callreference and callreference flag.
 *	It returns a pointer to the call descriptor if found, else a NULL.
 *---------------------------------------------------------------------------*/
struct call_desc *
cd_by_unitcr(struct i4b_controller *cntl, void *pipe, void *pipe_adapter, u_int cr)
{
	struct call_desc *cd;

	CNTL_LOCK_ASSERT(cntl);

	/* NT use CR to identify
	 * TE use TEI+CR to identify
	 */
	CD_FOREACH(cd, cntl)
	{
	    if((cd->cdid != CDID_UNUSED)  &&
	       (cd->cr == cr)             &&
	       ((cd->pipe == pipe_adapter) ||
		(cd->pipe == pipe)))
	    {
	        NDBGL4(L4_MSG, "found cd - cdid=%u, cr=%d",
		       cd->cdid, cd->cr);
		return(cd);
	    }
	}
	return(NULL);
}

/*---------------------------------------------------------------------------*
 *      disconnect active calls by application interface
 *---------------------------------------------------------------------------*/
void
i4b_disconnect_by_appl_interface(u_int8_t ai_type, void *ai_ptr)
{
	struct i4b_controller *cntl;
	struct call_desc *cd;

	for(cntl = &i4b_controller[0];
	    cntl < &i4b_controller[MAX_CONTROLLERS];
	    cntl++)
	{
	    CNTL_LOCK(cntl);

	    if(cntl->N_fifo_translator)
	    {
	       /* connected */
	       CD_FOREACH(cd, cntl)
	       {
		  if((cd->cdid != CDID_UNUSED) &&
		     (cd->ai_type == ai_type) &&
		     (cd->ai_ptr == ai_ptr))
		  {
		      /* preset causes with our cause */
		      cd->cause_in = cd->cause_out = 
			(CAUSET_I4B << 8)|CAUSE_I4B_NORMAL;

		      N_DISCONNECT_REQUEST(cd,cd->cause_in);

		      /* cd = NULL; call descriptor is freed ! */
		  }
	       }
	    }
	    CNTL_UNLOCK(cntl);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *      update D-channel
 *
 * called when device has been attached
 * or firmware has been loaded
 *---------------------------------------------------------------------------*/
void
i4b_update_d_channel(struct i4b_controller *cntl)
{
	struct i4b_protocol p;

	memset(&p, 0, sizeof(p));

	p.protocol_1 = cntl->N_protocol;

	i4b_setup_driver(cntl,
			 CHAN_D1,
			 &p,
			 cntl->N_driver_type,
			 cntl->unit,
			 0);
	return;
}

u_int32_t i4b_open_refcount = 0;

/*---------------------------------------------------------------------------*
 *      update all D-channels
 *
 * called when application interfaces are opened or closed
 *---------------------------------------------------------------------------*/
void 
i4b_update_all_d_channels(int open)
{
	struct i4b_controller *cntl;
	static u_int8_t flag = 0;

	mtx_lock(&i4b_global_lock);

	while(flag)
	{
		/* wait for other thread to 
		 * finish open or close 
		 */
		flag |= 2;
		(void) msleep(&flag, &i4b_global_lock,
			      PZERO, "I4B update D-channels", 0);
	}
	flag = 1;

	if(open)
	{
		i4b_open_refcount++;
	}
	else
	{
		if(i4b_open_refcount)
		{
			i4b_open_refcount--;
		}
	}

	if((i4b_open_refcount == 0) || 
	   ((i4b_open_refcount == 1) && open))
	{
		mtx_unlock(&i4b_global_lock);

		for(cntl = &i4b_controller[0];
		    cntl < &i4b_controller[MAX_CONTROLLERS];
		    cntl++)
		{
			i4b_update_d_channel(cntl);
		}

		mtx_lock(&i4b_global_lock);
	}

	if(flag & 2)
	{
		wakeup(&flag);
	}
	flag = 0;

	mtx_unlock(&i4b_global_lock);

	return;
}

static const u_int8_t MAKE_TABLE(I4B_CAUSES,Q850_CONV,[]);

/*---------------------------------------------------------------------------*
 *	return a valid q.931/q.850 cause from any of the internal causes
 *---------------------------------------------------------------------------*/
u_int8_t
i4b_make_q850_cause(cause_t cause)
{
	register u_int8_t ret = GET_CAUSE_VAL(cause);
	
	switch(GET_CAUSE_TYPE(cause)) {
	case CAUSET_Q850:
		ret = ret & 0x7F;
		break;

	case CAUSET_I4B:
		if(ret >= N_I4B_CAUSES)
		{
		    ret = CAUSE_I4B_NORMAL;
		}
		ret = I4B_CAUSES_Q850_CONV[ret];
		break;

	default:
		panic("%s: %s: unknown cause = 0x%04x!",
		      __FILE__, __FUNCTION__, cause);
		break;
	}
	return(ret);
}

/*===========================================================================*
 *	line interconnect routines
 *===========================================================================*/

static struct i4b_line_interconnect *
i4b_li_alloc(cdid_t cdid)
{
    struct i4b_controller *cntl = CNTL_FIND(cdid);
    struct i4b_line_interconnect *li;

    mtx_assert(&i4b_global_lock, MA_OWNED);

    if((cdid == CDID_UNUSED) ||
       (cntl == NULL))
    {
        li = NULL;
	goto done;
    }

    LI_FOREACH(li,cntl)
    {
        if(li->cdid == CDID_UNUSED)
	{
	    li->cdid = cdid;
	    goto done;
	}
    }

    li = NULL;

 done:
    return li;
}

static struct i4b_line_interconnect *
i4b_li_search(cdid_t cdid)
{
    struct i4b_controller *cntl = CNTL_FIND(cdid);
    struct i4b_line_interconnect *li;

    mtx_assert(&i4b_global_lock, MA_OWNED);

    if((cdid == CDID_UNUSED) ||
       (cntl == NULL))
    {
        li = NULL;
	goto done;
    }

    LI_FOREACH(li, cntl)
    {
        if(li->cdid == cdid)
	{
	    goto done;
	}
    }
    li = NULL;

 done:
    return li;
}

static void
i4b_li_free(struct i4b_line_interconnect *li)
{
    mtx_assert(&i4b_global_lock, MA_OWNED);

    bzero(li, sizeof(*li));

    return;
}

static struct i4b_line_interconnect *
__i4b_slot_li_alloc(cdid_t cdid, u_int8_t pcm_cable, u_int16_t pcm_slot_rx)
{
    static struct i4b_line_interconnect *li;
    struct i4b_pcm_cable *cable;
    u_int16_t pcm_end;
    u_int16_t pcm_slot_tx;

    mtx_assert(&i4b_global_lock, MA_OWNED);

    cable = &i4b_pcm_cable[pcm_cable];
    pcm_end = (cable->slot_end / 2);
    pcm_slot_tx = pcm_slot_rx;

    /* get the peer slot */

    if(pcm_slot_rx < pcm_end)
       pcm_slot_rx += pcm_end;
    else
       pcm_slot_rx -= pcm_end;

    if(GET_BIT(cable->slot_bitmap, pcm_slot_rx))
    {
        /* full */
        li = NULL;
	goto done;
    }

    li = i4b_li_alloc(cdid);

    if(li == NULL)
    {
        /* full */
        goto done;
    }

    SET_BIT(cable->slot_bitmap, pcm_slot_rx, 1);
    li->pcm_cable = pcm_cable;
    li->pcm_slot_rx = pcm_slot_rx;
    li->pcm_slot_tx = pcm_slot_tx;

 done:
    return li;
}

static u_int32_t
get_dword(u_int32_t *ptr, u_int16_t bit_offset)
{
    u_int32_t temp;
    u_int16_t dword_offset;

    dword_offset = bit_offset / (4*8);
    bit_offset %= (4*8);

    temp = (ptr[dword_offset] >> (bit_offset));

    if(bit_offset) {
      temp |= (ptr[dword_offset+1] << (32-bit_offset));
    }
    return temp;
}

struct i4b_line_interconnect *
i4b_slot_li_alloc(cdid_t cdid_src, cdid_t cdid_dst)
{
    struct i4b_controller *cntl_src = CNTL_FIND(cdid_src);
    struct i4b_controller *cntl_dst = CNTL_FIND(cdid_dst);
    struct i4b_pcm_cable *cable;
    struct i4b_line_interconnect *li_src = NULL;
    struct i4b_line_interconnect *li_dst;
    u_int32_t x;
    u_int32_t y;
    u_int32_t z;
    u_int32_t t;
    u_int16_t pcm_end;

    mtx_assert(&i4b_global_lock, MA_OWNED);

    if((cntl_src == NULL) ||
       (cntl_dst == NULL))
    {
	goto done;
    }

    /*
     * 1. see if the other end
     * has already allocated a
     * cable:
     */

    li_dst = i4b_li_search(cdid_dst);

    if(li_dst)
    {
	  li_src = __i4b_slot_li_alloc
	    (cdid_src, li_dst->pcm_cable, li_dst->pcm_slot_rx);
	  goto done;
    }

    /* 
     * 2. try to allocate a common slot 
     * on a common cable
     */

    for(x = 0; x < cntl_src->L1_pcm_cable_end; x++)
    {
        if(cntl_src->L1_pcm_cable_map[x] >= I4B_PCM_CABLE_MAX)
	{
	    /* invalid cable number */
	    continue;
	}

	for(y = 0; y < cntl_dst->L1_pcm_cable_end; y++)
	{
	    if(cntl_dst->L1_pcm_cable_map[y] != 
	       cntl_src->L1_pcm_cable_map[x])
	    {
	        /* not the same cable */
	        continue;
	    }

	    z = cntl_src->L1_pcm_cable_map[x];

	    cable = &i4b_pcm_cable[z];
	    pcm_end = (cable->slot_end/2);

	    for(y = 0; y < pcm_end; y += (4*8))
	    {
	        t = 
		  get_dword(cable->slot_bitmap, y) |
		  get_dword(cable->slot_bitmap, y + pcm_end);

		if(t == 0xffffffff)
		{
		    /* no slots free */
		    continue;
		}

		for( ; y < pcm_end; y++)
		{
		    if(t & (1 << (y % (4*8))))
		    {
		        /* no slot free */
		        continue;
		    }

		    li_src = __i4b_slot_li_alloc(cdid_src, z, y);
		    goto done;
		}
		break;
	    }
	    break;
	}
    }

 done:
    return li_src;
}

void
i4b_slot_li_free(struct i4b_line_interconnect *li)
{
    struct i4b_pcm_cable *cable;

    mtx_assert(&i4b_global_lock, MA_OWNED);

    cable = &i4b_pcm_cable[li->pcm_cable];

    SET_BIT(cable->slot_bitmap, li->pcm_slot_rx, 0);

    i4b_li_free(li);

    return;

}

