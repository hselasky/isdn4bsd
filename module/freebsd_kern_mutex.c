#include <sys/param.h>
#include <sys/systm.h>

#if (__NetBSD_Version__ < 500000000)
#include <freebsd_kern_mutex_v4.c>
#else
#include <freebsd_kern_mutex_v5.c>
#endif
