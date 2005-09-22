/*-
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/kern/kern_sysctl.c"
 */
#include <sys/param.h>
#include <sys/systm.h>

#include <sys/freebsd_compat.h>

struct sysctl_oid_list sysctl__children;

SYSCTL_NODE(, OID_AUTO, kern,    CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, sysctl,  CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, vm,      CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, vfs,     CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, net,     CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, debug,   CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, debug_sizeof, CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, hw,      CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, hw_bus,  CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, machdep, CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, user,    CTLFLAG_RW, NULL, NULL);
SYSCTL_NODE(, OID_AUTO, compat,  CTLFLAG_RW, NULL, NULL);

int
sysctl_handle_int(SYSCTL_HANDLER_ARGS)
{
    return 0;
}

