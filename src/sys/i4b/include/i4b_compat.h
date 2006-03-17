#if defined(__FreeBSD__)

# include <sys/conf.h>

# ifdef _KERNEL
#  include <machine/resource.h>
#  include <sys/bus.h>
#  include <dev/sound/pcm/sound.h>
#  include <dev/sound/pcm/dsp.h>
#  include <dev/pci/pcireg.h>
#  include <dev/pci/pcivar.h>
#  include <isa/isavar.h>
#  include <isa/isa_common.h>
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
#  include <fs/devfs/devfs.h>
#  include <sys/callout.h> /* callout_xxx() */
#  ifndef __KASSERT
    typedef struct cdevsw cdevsw_t;
#   define __lockmgr lockmgr
#   define __KASSERT KASSERT
#  endif
#  ifndef __callout_init_mtx
#   define __callout_init_mtx(c,m,f) callout_init_mtx(&(c)->co,m,f)
#   define __callout_reset(c,t,f,d) callout_reset(&(c)->co,t,f,d)
#   define __callout_stop(c) callout_stop(&(c)->co)
#   define __callout_pending(c) callout_pending(&(c)->co)
    struct __callout { struct callout co; };
#  endif
# else
#  include <ncurses.h>
#  include <osreldate.h>
# endif

#else
#include <sys/freebsd_compat.h>
#endif
