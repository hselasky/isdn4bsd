/*-
 * Copyright (c) 2000, 2001 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2000-2006 Hans Petter Selasky. All rights reserved.
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
 *	i4b_l1.c - ISDN4BSD layer 1
 *	---------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifdef I4B_GLOBAL_INCLUDE_FILE
#include I4B_GLOBAL_INCLUDE_FILE
#else
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/kernel.h>

#include <net/if.h>
#endif

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_global.h>

struct i4b_debug_mask i4b_debug_mask;
struct i4b_controller i4b_controller[MAX_CONTROLLERS]; /* controller description array */
struct i4b_pcm_cable i4b_pcm_cable[I4B_PCM_CABLE_MAX]; /* PCM cable array */

struct mtx i4b_global_lock;

/*---------------------------------------------------------------------------*
 *	i4b_controller_setup
 *---------------------------------------------------------------------------*/
static void
i4b_controller_setup(void *arg)
{
  struct i4b_controller *cntl;
  __typeof(cntl->unit) unit = 0;
  __typeof(cntl->unit) mask;

#if DO_I4B_DEBUG
  i4b_debug_mask = 
#ifdef DO_I4B_MAXDEBUG
    i4b_debug_max;
#else
    i4b_debug_err;
#endif
#endif

  mtx_init(&i4b_global_lock, "i4b_global_lock", 
	   NULL, MTX_DEF|MTX_RECURSE);

  for(cntl = &i4b_controller[0];
      cntl < &i4b_controller[MAX_CONTROLLERS];
      cntl++)
  {
	cntl->unit = unit;

	mtx_init(&cntl->L1_lock_data, "i4b_controller_lock", 
		 NULL, MTX_DEF|MTX_RECURSE);

#if (MAX_CONTROLLERS <= 8)
	mask = -1;
#else
	if((unit < 8) ||
	   (unit >= (MAX_CONTROLLERS-8))) 
	{
	    mask = -1;
	}
	else if((unit < 16) || 
		(unit >= (MAX_CONTROLLERS-16)))
	{
	    mask = -4;
	}
	else
	{
	    mask = -8;
	}
#endif
	cntl->L1_lock_ptr =
	  &(i4b_controller[unit & mask].L1_lock_data);

	unit++;
  }
  return;
}

SYSINIT(i4b_controller_setup, SI_SUB_LOCK, SI_ORDER_ANY, 
	i4b_controller_setup, NULL);

/*---------------------------------------------------------------------------*
 *	i4b_controller_reset
 *---------------------------------------------------------------------------*/
static void
i4b_controller_reset(struct i4b_controller *cntl)
{

  mtx_lock(&i4b_global_lock);

  bzero(&(cntl->dummy_zero_start[0]),
	(&(cntl->dummy_zero_end[0])) -
	(&(cntl->dummy_zero_start[0])));

  mtx_unlock(&i4b_global_lock);

  cntl->N_protocol = P_D_CHANNEL;
  cntl->N_driver_type = DRVR_D_CHANNEL;
  cntl->N_serial_number = cntl->unit + 0xABCD;

  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_allocate
 *
 * NOTE: all sub-controllers are under the same lock
 *---------------------------------------------------------------------------*/
struct i4b_controller *
i4b_controller_allocate(uint8_t portable, uint8_t sub_controllers, 
			uint8_t call_descriptors, uint8_t *error)
{
  struct i4b_controller *cntl;
  struct i4b_controller *cntl_end;
  struct i4b_controller *cntl_temp;
  struct call_desc *cd = NULL;
  struct i4b_line_interconnect *li = NULL;
  struct mtx *p_mtx;
  uint8_t x;

  if((sub_controllers == 0) ||
     (sub_controllers > MAX_CONTROLLERS))
  {
      ADD_ERROR(error, "%s: number of sub-controllers, "
		"%d, is invalid!\n", __FUNCTION__, 
		sub_controllers);
      cntl = NULL;
      goto done;
  }

  if((call_descriptors == 0) ||
     (call_descriptors > MAX_CHANNELS))
  {
      ADD_ERROR(error, "%s: number of call-descriptors, "
		"%d, is invalid!\n", __FUNCTION__,
		call_descriptors);
      cntl = NULL;
      goto done;
  }

  /* try to allocate call descriptors 
   * and line interconnect structures:
   */
  cd = malloc(call_descriptors * sub_controllers * sizeof(*cd),
	      M_DEVBUF, M_WAITOK|M_ZERO);

  li = malloc(call_descriptors * sub_controllers * sizeof(*li),
	      M_DEVBUF, M_WAITOK|M_ZERO);

  if((cd == NULL) ||
     (li == NULL))
  {
      ADD_ERROR(error, "%s: could not allocate "
		"call descriptors or line "
		"interconnect structures!\n",
		__FUNCTION__);
      cntl = NULL;
      goto done;
  }

  cntl_end = &i4b_controller[MAX_CONTROLLERS-sub_controllers];

  if(portable)
    cntl = cntl_end;
  else
    cntl = &i4b_controller[0];

 repeat:

  CNTL_LOCK(cntl);
  p_mtx = CNTL_GET_LOCK(cntl);
  cntl_temp = cntl;
  x = sub_controllers;

  while(x--)
  {
      if(cntl_temp->allocated ||
	 (CNTL_GET_LOCK(cntl_temp) != p_mtx))
      {
	  CNTL_UNLOCK(cntl);

	  if(portable)
	    cntl--;
	  else
	    cntl++;

	  if((cntl <= cntl_end) &&
	     (cntl >= &i4b_controller[0]))
	  {
	      goto repeat;
	  }

	  ADD_ERROR(error, "%s: cannot handle more than %d devices!",
		    __FUNCTION__, MAX_CONTROLLERS);

	  cntl = NULL;
	  goto done;
      }
      cntl_temp++;
  }

  cntl_temp = cntl;
  x = sub_controllers;

  while(x--)
  {
      i4b_controller_reset(cntl_temp);

      cntl_temp->allocated = 1;
      cntl_temp->N_call_desc_start = cd;
      cntl_temp->N_line_interconnect_start = li;
      cd += call_descriptors;
      li += call_descriptors;
      cntl_temp->N_call_desc_end = cd;
      cntl_temp->N_line_interconnect_end = li;
      cntl_temp++;
  }

  CNTL_UNLOCK(cntl);

 done:
  if(cntl == NULL)
  {
      if(cd)
      {
	  free(cd, M_DEVBUF);
      }
      if(li)
      {
	  free(li, M_DEVBUF);
      }
  }
  return cntl;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_attach
 *
 * TODO: could check that ``i4b_controller'' has been properly setup
 *---------------------------------------------------------------------------*/
int
i4b_controller_attach(struct i4b_controller *cntl, uint8_t *error)
{
  CNTL_LOCK(cntl);

  cntl->attached = 1;

  if(cntl->L1_type >= N_L1_TYPES)
  {
      cntl->L1_type = L1_TYPE_UNKNOWN;
  }

  CNTL_UNLOCK(cntl);

  i4b_update_d_channel(cntl);

#if 0
  i4b_l1_mph_status_ind((int)sc, STI_ATTACH);
#endif

  return IS_ERROR(error);
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_detach
 *---------------------------------------------------------------------------*/
void
i4b_controller_detach(struct i4b_controller *cntl)
{
  if(cntl)
  {
    CNTL_LOCK(cntl);

    if(cntl->attached) {
       cntl->attached = 0;

        /* set N_protocol before D-channel is updated */
        cntl->N_protocol = P_DISABLE;

	i4b_update_d_channel(cntl);
    }

    CNTL_UNLOCK(cntl);
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_free
 *---------------------------------------------------------------------------*/
void
i4b_controller_free(struct i4b_controller *cntl, uint8_t sub_controllers)
{
  struct call_desc *cd;
  struct i4b_line_interconnect *li;

  if(cntl && sub_controllers)
  {
      /* the sub-controllers should be 
       * all under the same lock!
       */
      CNTL_LOCK(cntl);

      cd = cntl->N_call_desc_start;
      li = cntl->N_line_interconnect_start;

      while(1)
      {
	  if(cntl->attached) {
	      i4b_controller_detach(cntl);
	  }

	  i4b_controller_reset(cntl);

	  cntl->allocated = 0;

	  if(--sub_controllers) {
	      cntl++;
	  } else {
	      break;
	  }
      }

      CNTL_UNLOCK(cntl);

      if(cd)
      {
	  free(cd, M_DEVBUF);
      }

      if(li)
      {
	  free(li, M_DEVBUF);
      }
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_l1_command_req
 *---------------------------------------------------------------------------*/
int
i4b_l1_command_req(struct i4b_controller *cntl, int cmd, void *data)
{
  int error = 0;
  uint32_t dbg_mask = 0;
  uint32_t dbg_value = 0;

  CNTL_LOCK(cntl);

  if(cntl->L1_COMMAND_REQ)
  {
    switch(cmd) {
    case CMR_SET_I4B_OPTIONS:
    {
	i4b_debug_t *dbg = (void *)data;

	/* save value of non-L1-options */
	dbg_mask = (dbg->mask & (I4B_OPTION_NO_DIALTONE |
	    I4B_OPTION_NO_STATUS_ENQUIRY));
	dbg_value = dbg_mask & dbg->value;

	if (dbg->mask & I4B_OPTION_NT_MODE) {
		if (dbg->value & I4B_OPTION_NT_MODE)
			cntl->N_nt_mode = 1;
		else
			cntl->N_nt_mode = 0;
	}
	if (dbg->mask & I4B_OPTION_NO_DIALTONE) {
		if (dbg->value & I4B_OPTION_NO_DIALTONE)
			cntl->no_layer1_dialtone = 1;
		else
			cntl->no_layer1_dialtone = 0;
	}
	if (dbg->mask & I4B_OPTION_NO_STATUS_ENQUIRY) {
		if (dbg->value & I4B_OPTION_NO_STATUS_ENQUIRY)
			cntl->no_layer3_status_enquiry = 1;
		else
			cntl->no_layer3_status_enquiry = 0;
	}
	break;
    }
    case CMR_SET_POWER_SAVE:
        cntl->no_power_save = 0;
	break;
    case CMR_SET_POWER_ON:
        cntl->no_power_save = 1;
	break;
    case CMR_INFO_REQUEST:
    {
        msg_ctrl_info_req_t *mcir = (void *)data;

	mcir->l1_type = cntl->L1_type;
	mcir->l1_channels = cntl->L1_channel_end;
	mcir->l1_serial = cntl->N_serial_number;
	mcir->l1_desc[0] = '\0';
	mcir->l1_no_dialtone = cntl->no_layer1_dialtone;
	mcir->l3_no_status_enquiry = cntl->no_layer3_status_enquiry;
	mcir->l1_no_power_save = cntl->no_power_save;
	mcir->l1_attached = cntl->attached;
	mcir->l2_driver_type = cntl->N_driver_type;
	break;
    }
    case CMR_SET_LAYER2_PROTOCOL:
    {
        msg_prot_ind_t *mpi = (void *)data;

	if ((cntl->N_driver_type != (uint32_t)mpi->driver_type) ||
	    (cntl->N_serial_number != (uint16_t)mpi->serial_number))
	{
	    /* disconnect current driver */

	    cntl->N_protocol = P_DISABLE;
	    i4b_update_d_channel(cntl);

	    cntl->N_driver_type = mpi->driver_type;
	    cntl->N_serial_number = mpi->serial_number;

	    /* connect new driver */

	    cntl->N_protocol = P_D_CHANNEL;
	    i4b_update_d_channel(cntl);
	}
	break;
    }
    case CMR_SET_PCM_MAPPING:
    {
        i4b_debug_t *dbg = (void *)data;
	uint8_t x = dbg->value;

	if(x > (sizeof(dbg->desc)/sizeof(dbg->desc[0]))) {
	   x = (sizeof(dbg->desc)/sizeof(dbg->desc[0]));
	}

	if(x > I4B_PCM_CABLE_MAX) {
	   x = I4B_PCM_CABLE_MAX;
	}

	cntl->L1_pcm_cable_end = x;

	while(x--) {
	  cntl->L1_pcm_cable_map[x] = dbg->desc[x];
	}
        break;
    }
    default:
      break;
    }
    error = (cntl->L1_COMMAND_REQ)(cntl,cmd,data);

    switch(cmd) {
    case CMR_SET_I4B_OPTIONS:
    {
	i4b_debug_t *dbg = (void *)data;

	/* restore non-L1-option mask and value */
	dbg->mask |= dbg_mask;
	dbg->value &= ~dbg_mask;
	dbg->value |= dbg_value;
	break;
    }
    default:
	break;
    }
  }
  else
  {
    /* no controller present */
    error = ENODEV;
  }

  CNTL_UNLOCK(cntl);

  return error;
}

/*---------------------------------------------------------------------------*
 *	i4b_l1_set_options
 *---------------------------------------------------------------------------*/
int
i4b_l1_set_options(struct i4b_controller *cntl, 
		   uint32_t mask, uint32_t value)
{
   i4b_debug_t dbg;

   bzero(&dbg, sizeof(dbg));

   dbg.value = value;
   dbg.mask = mask;
   return L1_COMMAND_REQ(cntl,CMR_SET_I4B_OPTIONS,&dbg);
}

/*---------------------------------------------------------------------------*
 *	telephony silence detection
 *
 * returns 1 when silence and 0 when activity
 *---------------------------------------------------------------------------*/
int
i4b_l1_bchan_tel_silence(unsigned char *data, register int len)
{
	register int j = (len / 2);

	/* subtract silence bytes from ``j'' */
	
	while(len--)
	{
		if((*data >= 0xaa) && (*data <= 0xac))
		{
			j--;
		}
		data++;
	}

	return((j < 0) ? 1 : 0);
}
