/*-
 * Copyright (c) 2015 Hans Petter Selasky. All rights reserved.
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
 */
#ifndef _I4B_COMPAT_H_
#define	_I4B_COMPAT_H_

#ifdef I4B_GLOBAL_INCLUDE_FILE
#else
#ifdef __FreeBSD__

# include <sys/conf.h>

# ifdef _KERNEL
#  include <machine/resource.h>
#  include <sys/bus.h>
#  include <dev/pci/pcireg.h>
#  include <dev/pci/pcivar.h>
#  include <isa/isavar.h>
#  include <isa/isa_common.h>
#  include <sys/socket.h>
#  include <net/if.h>
#  include <net/if_var.h>
#  include <netgraph/ng_message.h>
#  include <netgraph/ng_parse.h>
#  include <netgraph/netgraph.h>
#  include <sys/lockmgr.h>
#  include <sys/module.h>
#  include <sys/mutex.h>
#  include <sys/rman.h>
#  include <sys/rtprio.h>
#  include <sys/selinfo.h>
#  include <sys/sysctl.h>
#  include <sys/sx.h>
#  include <sys/filio.h>
#  include <sys/proc.h>
#  include <fs/devfs/devfs.h>
#  include <sys/callout.h>
#  ifndef __KASSERT
    typedef struct cdevsw cdevsw_t;
#   define __lockmgr lockmgr
#   define __KASSERT KASSERT
#  endif
# else
#  include <ncurses.h>
#  include <osreldate.h>
# endif

#else
#include <sys/freebsd_compat.h>
#endif
#endif
#endif
