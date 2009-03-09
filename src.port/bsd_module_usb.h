/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
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

/* USB configuration */

#define	USB_NO_POLL 1
#define	USB_USE_CONDVAR 1
#define	USB_DEBUG 1
#define	USB_HAVE_UGEN 0

#define	USB_TD_GET_RUID(td) UID_ROOT
#define	USB_TD_GET_RGID(td) GID_OPERATOR
#define	USB_TD_GET_PROC(td) ((struct proc *)(td))
#define	USB_PROC_GET_GID(td) GID_OPERATOR
#define	USB_VNOPS_FO_TRUNCATE(...) EINVAL
#define	USB_VNOPS_FO_STAT(...) EINVAL
#define	USB_VNOPS_FO_CLOSE(fd, td, perr) do { *(perr) = 0; } while (0);

typedef int usb_handle_request_t (device_t dev, const void *req, void **pptr, uint16_t *plen, uint16_t offset, uint8_t is_complete);
