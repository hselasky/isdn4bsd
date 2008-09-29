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

#include <bsd_module_all.h>

int
bsd_bus_event(PROCESS proc, int unit, int what, void *ivars)
{
	SIGSELECT sigsel[] = {1, BsdEventBusId};
	struct bsd_bus_event *SigP;
	int error;

	if (proc == 0) {
		return (ENXIO);
	}
	SigP = (void *)alloc(sizeof(struct bsd_bus_event), BsdEventBusId);
	SigP->ivars = ivars;
	SigP->unit = unit;
	SigP->what = what;
	SigP->error = 0;

	DROP_GIANT();
	/* forward message to the handler */
	send((void *)&SigP, proc);
	/* wait for reply */
	SigP = (void *)receive(sigsel);
	PICKUP_GIANT();

	error = SigP->error;
	free_buf((void *)&SigP);

	/* convert error code */
	switch (error) {
	case BSD_BUS_OK:
		error = 0;
		break;
	default:
		error = ENXIO;
		break;
	}
	return (error);
}

int
usb_handle_control_request(PROCESS proc, int unit, const void *req,
    void **pptr, uint16_t *plen, uint16_t offset, uint8_t is_complete)
{
	SIGSELECT sigsel[] = {1, UsbEventHandleControlRequestId};
	struct usb_control_request *SigP;
	int error;

	if (proc == 0) {
		return (ENXIO);
	}
	SigP = (void *)alloc(sizeof(struct usb_control_request),
	    UsbEventHandleControlRequestId);
	SigP->unit = unit;
	SigP->req = req;
	SigP->pptr = pptr;
	SigP->plen = plen;
	SigP->offset = offset;
	SigP->is_complete = is_complete;
	SigP->error = 0;

	DROP_GIANT();
	/* forward message to the handler */
	send((void *)&SigP, proc);
	/* wait for reply */
	SigP = (void *)receive(sigsel);
	PICKUP_GIANT();

	error = SigP->error;
	free_buf((void *)&SigP);

	/* convert the error code */
	switch (error) {
	case USB_CONTROL_OK:
		error = 0;
		break;
	case USB_CONTROL_STALL:
		error = ENOTTY;
		break;
	default:			/* ignore */
		error = ENXIO;
		break;
	}
	return (error);
}

extern void usb2_needs_explore_all(void);

void
usb_driver_loaded(void)
{
	usb2_needs_explore_all();
	return;
}
