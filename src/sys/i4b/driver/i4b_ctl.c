/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	i4b_ctl.c - i4b system control port driver
 *	------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/socket.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_global.h>

__FBSDID("$FreeBSD: $");

static u_int8_t openflag = 0;

static	d_open_t	i4bctlopen;
static	d_close_t	i4bctlclose;
static	d_ioctl_t	i4bctlioctl;

static cdevsw_t i4bctl_cdevsw = {
#ifdef D_VERSION
      .d_version =    D_VERSION,
#endif
      .d_open =       i4bctlopen,
      .d_close =      i4bctlclose,
      .d_ioctl =      i4bctlioctl,
      .d_name =       "i4bctl",
};

/*---------------------------------------------------------------------------*
 *	interface attach routine
 *---------------------------------------------------------------------------*/
static void
i4bctlattach(void *dummy)
{
	printf("i4bctl: ISDN system control port attached\n");
	make_dev(&i4bctl_cdevsw, 0, UID_ROOT, GID_WHEEL, 0600, "i4bctl");
	return;
}
SYSINIT(i4bctlattach, SI_SUB_PSEUDO, SI_ORDER_ANY, i4bctlattach, NULL);

/*---------------------------------------------------------------------------*
 *	i4bctlopen - device driver open routine
 *---------------------------------------------------------------------------*/
static int
i4bctlopen(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	if(minor(dev))
		return (ENXIO);

	if(openflag)
		return (EBUSY);
	
	openflag = 1;
	
	return (0);
}

/*---------------------------------------------------------------------------*
 *	i4bctlclose - device driver close routine
 *---------------------------------------------------------------------------*/
static int
i4bctlclose(struct cdev *dev, int flag, int fmt, struct thread *td)
{
	openflag = 0;
	return (0);
}

/*---------------------------------------------------------------------------*
 *	i4bctlioctl - device driver ioctl routine
 *---------------------------------------------------------------------------*/
static int
i4bctlioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
#if DO_I4B_DEBUG
	i4b_debug_t *dbg = (i4b_debug_t *)data;
	i4b_controller_t *cntl = 0;
	int error = 0;

	if(minor(dev))
	{
		return(ENODEV);
	}

	/* lookup cntl in general */
	if(IOCPARM_LEN(cmd) == sizeof(*dbg))
	{
		cntl = CNTL_FIND(dbg->unit);

		if(cntl == NULL)
		{
		  return EINVAL;
		}

		CNTL_LOCK(cntl);
	}

	switch(cmd) {
	    /* debug masks currently
	     * apply for all units:
	     */
	case I4B_CTL_GET_DEBUG:
	    dbg->debug = i4b_debug_mask;
	    break;

	case I4B_CTL_SET_DEBUG:
	    i4b_debug_mask = dbg->debug;
	    break;

	case I4B_CTL_SET_N_SERIAL_NUMBER:
	    if(cntl->N_fifo_translator)
	    {
	        /* connected */
	        error = EPERM;
	    }
	    else
	    {
	        cntl->N_serial_number = dbg->value;
	    }
	    break;

	case I4B_CTL_SET_N_DRIVER_TYPE:
	    if(cntl->N_fifo_translator)
	    {
	        /* connected */
	        error = EPERM;
	    }
	    else
	    {
	        cntl->N_driver_type = dbg->value;
	    }
	    break;

	case I4B_CTL_GET_LAPDSTAT:
	    if(cntl->N_lapdstat)
	    {
	        bcopy(cntl->N_lapdstat, &dbg->lapdstat,
		      sizeof(lapdstat_t));
	    }
	    else
	    {
	        bzero(&dbg->lapdstat,
		      sizeof(lapdstat_t));
	    }
	    break;

	case I4B_CTL_CLR_LAPDSTAT:
	    if(cntl->N_lapdstat)
	    {
	        bzero(cntl->N_lapdstat,
		      sizeof(lapdstat_t));
	    }
	    break;

	    /*
	     * These IOCTL's are optional and
	     * may not be implemented:
	     */
	case I4B_CTL_SET_I4B_OPTIONS:
	    cmd = CMR_SET_I4B_OPTIONS;
	    goto L1_command;

	case I4B_CTL_GET_CHIPSTAT:
	    cmd = CMR_GET_CHIPSTAT;
	    goto L1_command;

	case I4B_CTL_CLR_CHIPSTAT:
	    cmd = CMR_CLR_CHIPSTAT;
	    goto L1_command;

	case I4B_CTL_PH_ACTIVATE:
	    cmd = CMR_PH_ACTIVATE;
	    goto L1_command;

	case I4B_CTL_PH_DEACTIVATE:
	    cmd = CMR_PH_DEACTIVATE;
	    goto L1_command;

	case I4B_CTL_SET_PROTOCOL:
	    cmd = CMR_SET_LAYER1_PROTOCOL;
	    goto L1_command;

	case I4B_CTL_RESET:
	    cmd = CMR_RESET;
	    goto L1_command;

	L1_command:

	    /* forward IOCTL to lower layers */
	    L1_COMMAND_REQ(cntl, cmd, dbg);
	    break;

	default:
	    error = ENOTTY;
	    break;
	}

	if(cntl)
	{
	  CNTL_UNLOCK(cntl);
	}

	return(error);
#else
	return(ENODEV);
#endif /* DO_I4B_DEBUG */
}
