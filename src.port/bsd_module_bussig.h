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

/*
 * The functions in this file transform BSD bus events into signals.
 */
#if 0
extern SIGSELECT BsdEventBusId;
extern SIGSELECT UsbEventHandleControlRequestId;
#endif

enum {
	BSD_BUS_PROBE,
	BSD_BUS_ATTACH,
	BSD_BUS_DETACH,
	BSD_BUS_SUSPEND,
	BSD_BUS_RESUME,
};

enum {
	BSD_BUS_OK,
	BSD_BUS_ERROR,
};

enum {
	USB_CONTROL_OK,
	USB_CONTROL_STALL,
	USB_CONTROL_IGNORE,
};

struct bsd_bus_event {
#if 0
	union SIGNAL hdr;
#endif
	void   *ivars;			/* pointer to driver dependent data */
	int	unit;			/* 0,1,2,3 ... */
	int	what;			/* BSD_BUS_XXX */
	int	error;			/* zero means success, see
					 * BSD_BUS_ERROR */
};

struct usb_control_request {
#if 0
	union SIGNAL hdr;
#endif
	int	unit;			/* 0,1,2,3 ... */
	const void *req;		/* pointer to setup header */
	void  **pptr;			/* pointer to data buffer */
	uint16_t *plen;			/* pointer to length */
	uint16_t offset;		/* data offset */
	uint8_t	is_complete;		/* set when status stage has completed */
	uint8_t	error;			/* zero means success, see
					 * USB_CONTROL_XXX */
};

#if 0
int	bsd_bus_event(PROCESS proc, int unit, int what, void *ivars);
int	usb_handle_control_request(PROCESS proc, int unit, const void *req, void **pptr, uint16_t *plen, uint16_t offset, uint8_t is_complete);
#endif
void	usb_driver_loaded(void);
