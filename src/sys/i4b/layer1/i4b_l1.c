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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/kernel.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_global.h>

unsigned int i4b_l1_debug = L1_DEBUG_DEFAULT;
unsigned int i4b_l2_debug = L2_DEBUG_DEFAULT;
unsigned int i4b_l3_debug = L3_DEBUG_DEFAULT;
unsigned int i4b_l4_debug = L4_DEBUG_DEFAULT;

i4b_controller_t i4b_controller[MAX_CONTROLLERS]; /* controller description array */

struct mtx i4b_global_lock;

/*---------------------------------------------------------------------------*
 *	i4b_controller_setup
 *---------------------------------------------------------------------------*/
static void
i4b_controller_setup(void *arg)
{
  i4b_controller_t *cntl;
  __typeof(cntl->unit) unit = 0;
  __typeof(cntl->unit) mask;

  mtx_init(&i4b_global_lock, "i4b_global_lock", 
	   NULL, MTX_DEF|MTX_RECURSE);

  for(cntl = &i4b_controller[0];
      cntl < &i4b_controller[MAX_CONTROLLERS];
      cntl++)
  {
	cntl->unit = unit;
	cntl->N_serial_number = unit + 0xABCD;

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
i4b_controller_reset(i4b_controller_t *cntl)
{
  cntl->N_protocol = P_D_CHANNEL;
  cntl->N_driver_type = DRVR_D_CHANNEL;

  cntl->N_nt_mode = 0;
  cntl->N_cdid_end = 0;

  cntl->L1_sc = NULL;
  cntl->L1_fifo = NULL;
  cntl->L1_channel_end = 0;

  cntl->L1_type = L1_TYPE_UNKNOWN;

  cntl->L1_GET_FIFO_TRANSLATOR = NULL;
  cntl->L1_COMMAND_REQ = NULL;

  cntl->no_layer1_dialtone = 0;

  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_allocate
 *
 * NOTE: all sub-controllers are under the same lock
 *---------------------------------------------------------------------------*/
i4b_controller_t *
i4b_controller_allocate(u_int8_t portable, u_int8_t sub_controllers, 
			u_int8_t *error)
{
  i4b_controller_t *cntl;
  i4b_controller_t *cntl_end;
  i4b_controller_t *cntl_temp;
  struct mtx *p_mtx;
  u_int8_t x;

  if((sub_controllers == 0) ||
     (sub_controllers > MAX_CONTROLLERS))
  {
      ADD_ERROR(error, "%s: number of sub-controllers, "
		"%d, is invalid!\n", __FUNCTION__, 
		sub_controllers);
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
      cntl_temp->allocated = 1;

      i4b_controller_reset(cntl_temp);

      cntl_temp++;
  }

  CNTL_UNLOCK(cntl);

 done:
  return cntl;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_attach
 *
 * TODO: could check that ``i4b_controller'' has been properly setup
 *---------------------------------------------------------------------------*/
int
i4b_controller_attach(i4b_controller_t *cntl, u_int8_t *error)
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
i4b_controller_detach(i4b_controller_t *cntl)
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
i4b_controller_free(i4b_controller_t *cntl, u_int8_t sub_controllers)
{
  if(cntl && sub_controllers)
  {
      /* the sub-controllers should be 
       * all under the same lock!
       */
      CNTL_LOCK(cntl);

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
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_by_cd
 *---------------------------------------------------------------------------*/
i4b_controller_t *
i4b_controller_by_cd(struct call_desc *cd)
{
  i4b_controller_t *cntl;
  for(cntl = &i4b_controller[0];
      cntl < &i4b_controller[MAX_CONTROLLERS];
      cntl++)
  {
    if((cd >= &cntl->N_call_desc[0]) &&
       (cd < &cntl->N_call_desc[N_CALL_DESC]))
    {
      return cntl;
    }
  }
  return 0;
}

/*---------------------------------------------------------------------------*
 *	i4b_l1_command_req
 *---------------------------------------------------------------------------*/
int
i4b_l1_command_req(struct i4b_controller *cntl, int cmd, void *parm)
{
  int error;

  CNTL_LOCK(cntl);

  if((cntl)->L1_COMMAND_REQ)
  {
    error = ((cntl)->L1_COMMAND_REQ)(cntl,cmd,parm);
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
