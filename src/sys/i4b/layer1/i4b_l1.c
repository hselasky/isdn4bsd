/*-
 * Copyright (c) 2000, 2001 Hellmuth Michaelis. All rights reserved.
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
 *	i4b_l1.c - isdn4bsd layer 1
 *	---------------------------
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

__FBSDID("$FreeBSD: $");

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
  __typeof(cntl->unit)
    unit = 0;

  mtx_init(&i4b_global_lock,"i4b_global_lock", NULL, MTX_DEF|MTX_RECURSE);

  for(cntl = &i4b_controller[0];
      cntl < &i4b_controller[MAX_CONTROLLERS];
      cntl++)
  {
	cntl->unit = unit;
	cntl->N_serial_number = unit + 0xABCD;

	mtx_init(&cntl->L1_lock,"i4b_controller_lock", 
		 NULL, MTX_DEF|MTX_RECURSE);

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

  cntl->L1_sc = 0;
  cntl->L1_channel_end = 0;

  cntl->L1_type = L1_TYPE_UNKNOWN;

  cntl->L1_GET_FIFO_TRANSLATOR = NULL;
  cntl->L1_COMMAND_REQ = NULL;

  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_allocate
 *---------------------------------------------------------------------------*/
i4b_controller_t *
i4b_controller_allocate(int portable, u_int8_t *error)
{
  i4b_controller_t *cntl;

  if(portable)
  {
    for(cntl = &i4b_controller[MAX_CONTROLLERS-1];
	cntl >= &i4b_controller[0];
	cntl--)
    {
      CNTL_LOCK(cntl);
      if(cntl->allocated == 0)
      {
	goto found;
      }
      CNTL_UNLOCK(cntl);
    }
  }
  else
  {
    for(cntl = &i4b_controller[0];
	cntl < &i4b_controller[MAX_CONTROLLERS];
	cntl++)
    {
      CNTL_LOCK(cntl);
      if(cntl->allocated == 0)
      {
      found:
	cntl->allocated = 1;

	i4b_controller_reset(cntl);

	CNTL_UNLOCK(cntl);

	goto done;
      }
      CNTL_UNLOCK(cntl);
    }
  }

  /* no controller found */
  cntl = 0;

  ADD_ERROR(error, "%s: cannot handle more than %d devices!",
	    __func__ , MAX_CONTROLLERS);
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

    /* set N_protocol before D-channel is updated */
    cntl->N_protocol = P_DISABLE;

    i4b_update_d_channel(cntl);

    CNTL_UNLOCK(cntl);
  }
  return;
}

/*---------------------------------------------------------------------------*
 *	i4b_controller_free
 *---------------------------------------------------------------------------*/
void
i4b_controller_free(i4b_controller_t *cntl)
{
  if(cntl)
  {
    CNTL_LOCK(cntl);

    i4b_controller_reset(cntl);

    cntl->allocated = 0;

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

#if defined (__FreeBSD__) && __FreeBSD__ < 5
u_int16_t mtx_level = 0;
#endif

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

/* EOF */

#if 0
L1_ACTIVE / DEACTIVE
i4b_l4_l12stat(cntl, 1, parm);
#endif
