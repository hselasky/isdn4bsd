#if defined(__FreeBSD__)

# include <sys/conf.h>

# ifdef _KERNEL
#  include <machine/resource.h>
#  include <dev/sound/pcm/sound.h>
#  include <dev/pci/pcireg.h>
#  include <dev/pci/pcivar.h>
#  include <isa/isavar.h>
#  include <isa/isa_common.h>
#  include <net/if_var.h>
#  include <netgraph/ng_message.h>
#  include <netgraph/ng_parse.h>
#  include <netgraph/netgraph.h>
#  include <sys/bus.h>
#  include <sys/lockmgr.h>
#  include <sys/module.h>
#  include <sys/mutex.h>
#  include <sys/rman.h>
#  include <sys/rtprio.h>
#  include <sys/selinfo.h>
#  include <sys/sysctl.h>
#  include <sys/sx.h>
#  include <fs/devfs/devfs.h>
# else
#  include <ncurses.h>
#  include <osreldate.h>
# endif

# ifdef _KERNEL
typedef struct cdevsw cdevsw_t;
# define __lockmgr lockmgr
# define __KASSERT KASSERT
# endif

#else
#include <sys/freebsd_compat.h>
#endif
