#ifndef _I4B_COMPAT_H_
#define	_I4B_COMPAT_H_

#if defined(__FreeBSD__)

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
#  include <sys/callout.h> /* callout_xxx() */
#  ifndef __KASSERT
    typedef struct cdevsw cdevsw_t;
#   define __lockmgr lockmgr
#   define __KASSERT KASSERT
#  endif
#  ifndef usb_callout_init_mtx
#   define usb_callout_init_mtx(c,m,f) callout_init_mtx(&(c)->co,m,f)
#   define usb_callout_reset(c,t,f,d) callout_reset(&(c)->co,t,f,d)
#   define usb_callout_stop(c) callout_stop(&(c)->co)
#   define usb_callout_pending(c) callout_pending(&(c)->co)
#   ifndef usb2_callout_init_mtx
     struct usb_callout { struct callout co; };
#   endif
#  endif
# else
#  include <ncurses.h>
#  include <osreldate.h>
# endif

#else
#include <sys/freebsd_compat.h>
#endif
#endif
