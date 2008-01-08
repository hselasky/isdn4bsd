#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uhub.c,v 1.82 2007/06/30 20:18:44 imp Exp $");

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * USB spec: http://www.usb.org/developers/docs/usbspec.zip
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#define	UHUB_INTR_INTERVAL 250		/* ms */

#ifdef USB_DEBUG
#define	DPRINTF(sc,n,fmt,...)	\
  do { if (uhub_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uhub_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, uhub, CTLFLAG_RW, 0, "USB uhub");
SYSCTL_INT(_hw_usb_uhub, OID_AUTO, debug, CTLFLAG_RW, &uhub_debug, 0,
    "uhub debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

struct uhub_current_state {
	uint16_t port_change;
	uint16_t port_status;
};

struct uhub_softc {
	struct uhub_current_state sc_st;/* current state */
	device_t sc_dev;		/* base device */
	struct usbd_device *sc_udev;	/* USB device */
	struct usbd_xfer *sc_xfer[2];	/* interrupt xfer */
	uint8_t	sc_flags;
#define	UHUB_FLAG_INTR_STALL 0x02
	uint8_t	sc_name[32];
};

#define	UHUB_PROTO(sc) ((sc)->sc_udev->ddesc.bDeviceProtocol)
#define	UHUB_IS_HIGH_SPEED(sc) (UHUB_PROTO(sc) != UDPROTO_FSHUB)
#define	UHUB_IS_SINGLE_TT(sc) (UHUB_PROTO(sc) == UDPROTO_HSHUBSTT)

/* prototypes for type checking: */

static device_probe_t uhub_probe;
static device_attach_t uhub_attach;
static device_detach_t uhub_detach;

static bus_driver_added_t uhub_driver_added;
static bus_child_location_str_t uhub_child_location_string;
static bus_child_pnpinfo_str_t uhub_child_pnpinfo_string;

static usbd_callback_t uhub_intr_callback;
static usbd_callback_t uhub_intr_clear_stall_callback;

static const struct usbd_config uhub_config[2] = {

	[0] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_ANY,
		.mh.timeout = 0,
		.mh.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.bufsize = 0,		/* use wMaxPacketSize */
		.mh.callback = &uhub_intr_callback,
		.interval = UHUB_INTR_INTERVAL,
	},

	[1] = {
		.type = UE_CONTROL,
		.endpoint = 0,
		.direction = UE_DIR_ANY,
		.mh.timeout = 1000,	/* 1 second */
		.interval = 50,		/* 50ms */
		.mh.flags = {},
		.bufsize = sizeof(usb_device_request_t),
		.mh.callback = &uhub_intr_clear_stall_callback,
	},
};

/*
 * driver instance for "hub" connected to "usb"
 * and "hub" connected to "hub"
 */
static devclass_t uhub_devclass;

static driver_t uhub_driver =
{
	.name = "uhub",
	.methods = (device_method_t[]){
		DEVMETHOD(device_probe, uhub_probe),
		DEVMETHOD(device_attach, uhub_attach),
		DEVMETHOD(device_detach, uhub_detach),

		DEVMETHOD(device_suspend, bus_generic_suspend),
		DEVMETHOD(device_resume, bus_generic_resume),
		DEVMETHOD(device_shutdown, bus_generic_shutdown),

		DEVMETHOD(bus_child_location_str, uhub_child_location_string),
		DEVMETHOD(bus_child_pnpinfo_str, uhub_child_pnpinfo_string),
		DEVMETHOD(bus_driver_added, uhub_driver_added),
		{0, 0}
	},
	.size = sizeof(struct uhub_softc)
};

DRIVER_MODULE(uhub, usb, uhub_driver, uhub_devclass, 0, 0);
DRIVER_MODULE(uhub, uhub, uhub_driver, uhub_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uhub, usb, 1, 1, 1);

static void
uhub_intr_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct uhub_softc *sc = xfer->priv_sc;
	struct usbd_xfer *xfer_other = sc->sc_xfer[0];

	if (usbd_clear_stall_callback(xfer, xfer_other)) {
		DPRINTF(sc, 0, "stall cleared\n");
		sc->sc_flags &= ~UHUB_FLAG_INTR_STALL;
		usbd_transfer_start(xfer_other);
	}
	return;
}

static void
uhub_intr_callback(struct usbd_xfer *xfer)
{
	struct uhub_softc *sc = xfer->priv_sc;

	switch (USBD_GET_STATE(xfer)) {
	case USBD_ST_TRANSFERRED:
		DPRINTF(sc, 1, "\n");
		/*
		 * This is an indication that some port
		 * has changed status. Notify the bus
		 * event handler thread that we need
		 * to be explored again:
		 */
		usb_needs_explore(sc->sc_udev->bus,
		    USB_BUS_EXPLORE_TREE);

	case USBD_ST_SETUP:
		if (sc->sc_flags & UHUB_FLAG_INTR_STALL) {
			usbd_transfer_start(sc->sc_xfer[1]);
		} else {
			xfer->frlengths[0] = xfer->max_data_length;
			usbd_start_hardware(xfer);
		}
		return;

	default:			/* Error */
		if (xfer->error != USBD_CANCELLED) {
			/* start clear stall */
			sc->sc_flags |= UHUB_FLAG_INTR_STALL;
			usbd_transfer_start(sc->sc_xfer[1]);
		}
		return;
	}
}

/*------------------------------------------------------------------------*
 *	uhub_explore_sub - subroutine
 *
 * Return values:
 *    0: Success
 * Else: A control transaction failed
 *------------------------------------------------------------------------*/
static usbd_status_t
uhub_explore_sub(struct uhub_softc *sc, struct usbd_port *up)
{
	struct usbd_bus *bus;
	struct usbd_device *child;
	uint8_t refcount;
	usbd_status_t err;

	bus = sc->sc_udev->bus;
	err = 0;

	/* get driver added refcount from USB bus */
	refcount = bus->driver_added_refcount;

	/* get device assosiated with the given port */
	child = usbd_bus_port_get_device(bus, up);
	if (child == NULL) {
		/* nothing to do */
		goto done;
	}
	/* check if probe and attach should be done */

	if (child->driver_added_refcount != refcount) {
		child->driver_added_refcount = refcount;
		err = usbd_probe_and_attach(child,
		    USB_IFACE_INDEX_ANY);
		if (err) {
			goto done;
		}
	}
	/* start control transfer, if device mode */

	if (child->flags.usb_mode == USB_MODE_DEVICE) {
		usbd_default_transfer_setup(child);
	}
	/* if a HUB becomes present, do a recursive HUB explore */

	if (child->hub) {
		err = (child->hub->explore) (child);
	}
done:
	return (err);
}

/*------------------------------------------------------------------------*
 *	uhub_read_port_status - factored out code
 *------------------------------------------------------------------------*/
static usbd_status_t
uhub_read_port_status(struct uhub_softc *sc, uint8_t portno)
{
	usb_port_status_t ps;
	usbd_status_t err;

	err = usbreq_get_port_status(
	    sc->sc_udev, &usb_global_lock, &ps, portno);

	/* update status regardless of error */

	sc->sc_st.port_status = UGETW(ps.wPortStatus);
	sc->sc_st.port_change = UGETW(ps.wPortChange);

	/* debugging print */

	DPRINTF(sc, 3, "port %d, wPortStatus=0x%04x, "
	    "wPortChange=0x%04x, err=%s\n",
	    portno, sc->sc_st.port_status,
	    sc->sc_st.port_change, usbd_errstr(err));
	return (err);
}

/*------------------------------------------------------------------------*
 *	uhub_reattach_port
 *
 * Returns:
 *    0: Success
 * Else: A control transaction failed
 *------------------------------------------------------------------------*/
static usbd_status_t
uhub_reattach_port(struct uhub_softc *sc, uint8_t portno)
{
	struct usbd_device *child;
	struct usbd_device *udev;
	usbd_status_t err;
	uint8_t timeout;
	uint8_t speed;
	uint8_t usb_mode;

	DPRINTF(sc, 0, "reattaching port %d\n", portno);

	err = 0;
	timeout = 0;
	udev = sc->sc_udev;
	child = usbd_bus_port_get_device(udev->bus,
	    udev->hub->ports + portno - 1);

repeat:

	/* first clear the port connection change bit */

	err = usbreq_clear_port_feature
	    (udev, &usb_global_lock, portno, UHF_C_PORT_CONNECTION);

	if (err) {
		goto error;
	}
	/* detach any existing devices */

	if (child) {
		usbd_detach_device(child, USB_IFACE_INDEX_ANY, 1);
		usbd_free_device(child);
		child = NULL;
	}
	/* get fresh status */

	err = uhub_read_port_status(sc, portno);
	if (err) {
		goto error;
	}
	/* check if nothing is connected to the port */

	if (!(sc->sc_st.port_status & UPS_CURRENT_CONNECT_STATUS)) {
		goto error;
	}
	/* check if there is no power on the port and print a warning */

	if (!(sc->sc_st.port_status & UPS_PORT_POWER)) {
		DPRINTF(sc, 0, "WARNING: strange, connected port %d "
		    "has no power\n", portno);
	}
	/* check if the device is in Host Mode */

	if (!(sc->sc_st.port_status & UPS_PORT_MODE_DEVICE)) {

		DPRINTF(sc, 0, "Port %d is in Host Mode\n", portno);

		/* USB Host Mode */

		/* wait for maximum device power up time */

		usbd_delay_ms(udev, USB_PORT_POWERUP_DELAY);

		/* reset port, which implies enabling it */

		err = usbreq_reset_port
		    (udev, &usb_global_lock, portno);

		if (err) {
			DPRINTF(sc, -1, "port %d reset "
			    "failed, error=%s\n",
			    portno, usbd_errstr(err));
			goto error;
		}
		/* get port status again, it might have changed during reset */

		err = uhub_read_port_status(sc, portno);
		if (err) {
			goto error;
		}
		/* check if something changed during port reset */

		if ((sc->sc_st.port_change & UPS_C_CONNECT_STATUS) ||
		    (!(sc->sc_st.port_status & UPS_CURRENT_CONNECT_STATUS))) {
			if (timeout) {
				DPRINTF(sc, -1, "giving up port reset "
				    "- device vanished!\n");
				goto error;
			}
			timeout = 1;
			goto repeat;
		}
	} else {
		DPRINTF(sc, 0, "Port %d is in Device Mode\n", portno);
	}

	/*
	 * Figure out the device speed
	 */
	speed =
	    (sc->sc_st.port_status & UPS_HIGH_SPEED) ? USB_SPEED_HIGH :
	    (sc->sc_st.port_status & UPS_LOW_SPEED) ? USB_SPEED_LOW : USB_SPEED_FULL;

	/*
	 * Figure out the device mode
	 *
	 * NOTE: This part is currently FreeBSD specific.
	 */
	usb_mode =
	    (sc->sc_st.port_status & UPS_PORT_MODE_DEVICE) ?
	    USB_MODE_DEVICE : USB_MODE_HOST;

	/* need to create a new child */

	child = usbd_alloc_device(sc->sc_dev, udev->bus, udev,
	    udev->depth + 1, portno - 1, portno, speed, usb_mode);
	if (child == NULL) {
		DPRINTF(sc, -1, "could not allocate new device!\n");
		goto error;
	}
	return (0);			/* success */

error:
	if (child) {
		usbd_detach_device(child, USB_IFACE_INDEX_ANY, 1);
		usbd_free_device(child);
		child = NULL;
	}
	if (err == 0) {
		if (sc->sc_st.port_status & UPS_PORT_ENABLED) {
			err = usbreq_clear_port_feature
			    (sc->sc_udev, &usb_global_lock,
			    portno, UHF_PORT_ENABLE);
		}
	}
	if (err) {
		DPRINTF(sc, -1, "device problem (%s), "
		    "disabling port %d\n", usbd_errstr(err), portno);
	}
	return (err);
}

/*------------------------------------------------------------------------*
 *	uhub_suspend_resume_port
 *
 * Returns:
 *    0: Success
 * Else: A control transaction failed
 *------------------------------------------------------------------------*/
static usbd_status_t
uhub_suspend_resume_port(struct uhub_softc *sc, uint8_t portno)
{
	struct usbd_device *child;
	struct usbd_device *udev;
	uint8_t is_suspend;
	usbd_status_t err;

	DPRINTF(sc, 0, "port %d\n", portno);

	udev = sc->sc_udev;
	child = usbd_bus_port_get_device(udev->bus,
	    udev->hub->ports + portno - 1);

	/* first clear the port suspend change bit */

	err = usbreq_clear_port_feature
	    (udev, &usb_global_lock, portno, UHF_C_PORT_SUSPEND);

	if (err) {
		goto done;
	}
	/* get fresh status */

	err = uhub_read_port_status(sc, portno);
	if (err) {
		goto done;
	}
	/* get current state */

	if (sc->sc_st.port_status & UPS_SUSPEND) {
		is_suspend = 1;
	} else {
		is_suspend = 0;
	}
	/* do the suspend or resume */

	if (child) {
		err = usbd_suspend_resume(child, is_suspend);
	}
done:
	return (err);
}

/*------------------------------------------------------------------------*
 *	uhub_explore
 *
 * Returns:
 *     0: Success
 *  Else: Failure
 *------------------------------------------------------------------------*/
static usbd_status_t
uhub_explore(struct usbd_device *udev)
{
	struct usbd_hub *hub;
	struct uhub_softc *sc;
	struct usbd_port *up;
	usbd_status_t err;
	uint8_t portno;
	uint8_t x;

	hub = udev->hub;
	sc = hub->hubsoftc;

	DPRINTF(sc, 10, "udev=%p addr=%d\n", udev, udev->address);

	/* ignore hubs that are too deep */
	if (udev->depth > USB_HUB_MAX_DEPTH) {
		return (USBD_TOO_DEEP);
	}
	for (x = 0; x != hub->nports; x++) {
		up = hub->ports + x;
		portno = x + 1;

		err = uhub_read_port_status(sc, portno);
		if (err) {
			/* most likely the HUB is gone */
			break;
		}
		if (sc->sc_st.port_change & UPS_C_PORT_ENABLED) {
			err = usbreq_clear_port_feature(
			    udev, &usb_global_lock, portno, UHF_C_PORT_ENABLE);
			if (err) {
				/* most likely the HUB is gone */
				break;
			}
			if (sc->sc_st.port_change & UPS_C_CONNECT_STATUS) {
				/*
				 * Ignore the port error if the device
				 * has vanished !
				 */
			} else if (sc->sc_st.port_status & UPS_PORT_ENABLED) {
				DPRINTF(sc, -1, "illegal enable change, "
				    "port %d\n", portno);
			} else {

				if (up->restartcnt == USBD_RESTART_MAX) {
					/* XXX could try another speed ? */
					DPRINTF(sc, -1, "port error, giving up "
					    "port %d\n", portno);
				} else {
					sc->sc_st.port_change |= UPS_C_CONNECT_STATUS;
					up->restartcnt++;
				}
			}
		}
		if (sc->sc_st.port_change & UPS_C_CONNECT_STATUS) {
			err = uhub_reattach_port(sc, portno);
			if (err) {
				/* most likely the HUB is gone */
				break;
			}
		}
		if (sc->sc_st.port_change & UPS_C_SUSPEND) {
			err = uhub_suspend_resume_port(sc, portno);
			if (err) {
				/* most likely the HUB is gone */
				break;
			}
		}
		err = uhub_explore_sub(sc, up);
		if (err) {
			/* no device(s) present */
			continue;
		}
		/* explore succeeded - reset restart counter */
		up->restartcnt = 0;
	}
	return (USBD_NORMAL_COMPLETION);
}

static int
uhub_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_device_descriptor_t *dd = usbd_get_device_descriptor(uaa->device);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	/*
	 * The subclass for USB HUBs is ignored because it is 0 for some
	 * and 1 for others.
	 */

	if ((uaa->iface == NULL) && (dd->bDeviceClass == UDCLASS_HUB)) {
		return (UMATCH_DEVCLASS_DEVSUBCLASS);
	}
	return (UMATCH_NONE);
}

static int
uhub_attach(device_t dev)
{
	struct uhub_softc *sc = device_get_softc(dev);
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct usbd_device *udev = uaa->device;
	struct usbd_device *parent_hub = udev->parent_hub;
	struct usbd_hub *hub;
	usb_device_request_t req;
	usb_hub_descriptor_t hubdesc;
	uint16_t pwrdly;
	uint8_t x;
	uint8_t nports;
	uint8_t portno;
	uint8_t removable;
	uint8_t iface_index;
	usbd_status_t err;

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = udev;
	sc->sc_dev = dev;

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
	    device_get_nameunit(dev));

	usbd_set_device_desc(dev);

	err = usbd_set_config_index(udev, 0, 1);
	if (err) {
		DPRINTF(sc, -1, "configuration failed, error=%s\n",
		    usbd_errstr(err));
		goto error;
	}
	/*
	 * NOTE: "usbd_set_config_index()" will change variables in "udev" !
	 */
	DPRINTF(sc, 1, "depth=%d selfpowered=%d, parent=%p, "
	    "parent->selfpowered=%d\n",
	    udev->depth,
	    udev->flags.self_powered,
	    parent_hub,
	    parent_hub ?
	    parent_hub->flags.self_powered : 0);

	if (udev->depth > USB_HUB_MAX_DEPTH) {
		DPRINTF(sc, -1, "hub depth, %d, exceeded. HUB ignored!\n",
		    USB_HUB_MAX_DEPTH);
		goto error;
	}
	if (!udev->flags.self_powered && parent_hub &&
	    (!parent_hub->flags.self_powered)) {
		DPRINTF(sc, -1, "bus powered hub connected to "
		    "bus powered hub. HUB ignored!\n");
		goto error;
	}
	/* get hub descriptor */

	DPRINTF(sc, 1, "getting hub descriptor\n");

	req.bmRequestType = UT_READ_CLASS_DEVICE;
	req.bRequest = UR_GET_DESCRIPTOR;
	USETW2(req.wValue, UDESC_HUB, 0);
	USETW(req.wIndex, 0);
	USETW(req.wLength, USB_HUB_DESCRIPTOR_SIZE);

	err = usbd_do_request(udev, &usb_global_lock, &req, &hubdesc);

	nports = hubdesc.bNbrPorts;

	if (!err && (nports >= 8)) {
		uint16_t len = (USB_HUB_DESCRIPTOR_SIZE - 1) + ((nports + 7) / 8);

		USETW(req.wLength, len);
		err = usbd_do_request(udev, &usb_global_lock, &req, &hubdesc);
	}
	if (err) {
		DPRINTF(sc, -1, "getting hub descriptor failed,"
		    "error=%s\n", usbd_errstr(err));
		goto error;
	}
	if (hubdesc.bNbrPorts != nports) {
		DPRINTF(sc, -1, "number of ports changed!\n");
		goto error;
	}
	if (nports == 0) {
		DPRINTF(sc, -1, "portless HUB!\n");
		goto error;
	}
	hub = malloc(sizeof(hub[0]) + (sizeof(hub->ports[0]) * nports),
	    M_USBDEV, M_WAITOK | M_ZERO);

	if (hub == NULL) {
		goto error;
	}
	udev->hub = hub;

	/* init FULL-speed ISOCHRONOUS schedule */
	usbd_fs_isoc_schedule_init_all(hub->fs_isoc_schedule);

	/* initialize HUB structure */
	hub->hubsoftc = sc;
	hub->explore = &uhub_explore;
	hub->nports = hubdesc.bNbrPorts;
	hub->hubudev = udev;

	/* if self powered hub, give ports maximum current */
	if (udev->flags.self_powered) {
		hub->portpower = USB_MAX_POWER;
	} else {
		hub->portpower = USB_MIN_POWER;
	}

	/* set up interrupt pipe */
	iface_index = 0;
	err = usbd_transfer_setup(udev, &iface_index, sc->sc_xfer,
	    uhub_config, 2, sc, &usb_global_lock);
	if (err) {
		DPRINTF(sc, -1, "cannot setup interrupt transfer, "
		    "errstr=%s!\n", usbd_errstr(err));
		goto error;
	}
	/* wait with power off for a while */
	usbd_delay_ms(udev, USB_POWER_DOWN_TIME);

	/*
	 * To have the best chance of success we do things in the exact same
	 * order as Windoze98.  This should not be necessary, but some
	 * devices do not follow the USB specs to the letter.
	 *
	 * These are the events on the bus when a hub is attached:
	 *  Get device and config descriptors (see attach code)
	 *  Get hub descriptor (see above)
	 *  For all ports
	 *     turn on power
	 *     wait for power to become stable
	 * (all below happens in explore code)
	 *  For all ports
	 *     clear C_PORT_CONNECTION
	 *  For all ports
	 *     get port status
	 *     if device connected
	 *        wait 100 ms
	 *        turn on reset
	 *        wait
	 *        clear C_PORT_RESET
	 *        get port status
	 *        proceed with device attachment
	 */

	/* XXX should check for none, individual, or ganged power? */

	removable = 0;
	pwrdly = ((hubdesc.bPwrOn2PwrGood * UHD_PWRON_FACTOR) +
	    USB_EXTRA_POWER_UP_TIME);

	for (x = 0; x != nports; x++) {
		/* set up data structures */
		struct usbd_port *up = hub->ports + x;

		up->device_index = 0;
		up->restartcnt = 0;
		portno = x + 1;

		/* check if port is removable */
		if (!UHD_NOT_REMOV(&hubdesc, portno)) {
			removable++;
		}
		/* turn the power on */
		err = usbreq_set_port_feature
		    (udev, &usb_global_lock, portno, UHF_PORT_POWER);

		if (err) {
			DPRINTF(sc, -1, "port %d power on failed, %s\n",
			    portno, usbd_errstr(err));
		}
		DPRINTF(sc, 0, "turn on port %d power\n",
		    portno);

		/* wait for stable power */
		usbd_delay_ms(udev, pwrdly);
	}

	device_printf(dev, "%d port%s with %d "
	    "removable, %s powered\n", nports, (nports != 1) ? "s" : "",
	    removable, udev->flags.self_powered ? "self" : "bus");

	/* start the interrupt endpoint */

	mtx_lock(sc->sc_xfer[0]->priv_mtx);
	usbd_transfer_start(sc->sc_xfer[0]);
	mtx_unlock(sc->sc_xfer[0]->priv_mtx);

	return (0);

error:
	usbd_transfer_unsetup(sc->sc_xfer, 2);

	if (udev->hub) {
		free(udev->hub, M_USBDEV);
		udev->hub = NULL;
	}
	return (ENXIO);
}

/*
 * Called from process context when the hub is gone.
 * Detach all devices on active ports.
 */
static int
uhub_detach(device_t dev)
{
	struct uhub_softc *sc = device_get_softc(dev);
	struct usbd_hub *hub = sc->sc_udev->hub;
	struct usbd_device *child;
	uint8_t x;

	/* detach all children first */
	bus_generic_detach(dev);

	if (hub == NULL) {		/* must be partially working */
		return (0);
	}
	for (x = 0; x != hub->nports; x++) {

		child = usbd_bus_port_get_device(sc->sc_udev->bus, hub->ports + x);

		/*
		 * Subdevices are not freed, because the caller of
		 * uhub_detach() will do that. The function we are
		 * calling is NULL safe.
		 */
		usbd_detach_device(child, USB_IFACE_INDEX_ANY, 0);
		usbd_free_device(child);
	}

	usbd_transfer_unsetup(sc->sc_xfer, 2);

	free(hub, M_USBDEV);
	sc->sc_udev->hub = NULL;
	return (0);
}

static void
uhub_driver_added(device_t dev, driver_t *driver)
{
	usb_needs_probe_and_attach();
	return;
}

struct hub_result {
	struct usbd_device *udev;
	uint8_t	portno;
	uint8_t	iface_index;
};

static void
uhub_find_iface_index(struct usbd_hub *hub, device_t child,
    struct hub_result *res)
{
	struct usbd_interface *iface;
	struct usbd_device *udev;
	uint8_t nports;
	uint8_t x;
	uint8_t i;

	nports = hub->nports;
	for (x = 0; x != nports; x++) {
		udev = usbd_bus_port_get_device(hub->hubudev->bus,
		    hub->ports + x);
		if (!udev) {
			continue;
		}
		if (udev->global_dev == child) {
			res->iface_index = 0;
			res->udev = udev;
			res->portno = x + 1;
			return;
		}
		for (i = 0; i != USB_MAX_INTERFACES; i++) {
			iface = usbd_get_iface(udev, i);
			if (iface &&
			    (iface->subdev == child)) {
				res->iface_index = i;
				res->udev = udev;
				res->portno = x + 1;
				return;
			}
		}
	}
	res->iface_index = 0;
	res->udev = NULL;
	res->portno = 0;
	return;
}

static int
uhub_child_location_string(device_t parent, device_t child,
    char *buf, size_t buflen)
{
	struct uhub_softc *sc = device_get_softc(parent);
	struct usbd_hub *hub = sc->sc_udev->hub;
	struct hub_result res;

	mtx_lock(&usb_global_lock);
	uhub_find_iface_index(hub, child, &res);
	if (!res.udev) {
		DPRINTF(sc, 0, "device not on hub\n");
		if (buflen) {
			buf[0] = '\0';
		}
		goto done;
	}
	if (res.udev->probed == USBD_PROBED_IFACE_AND_FOUND) {
		snprintf(buf, buflen, "port=%i interface=%i",
		    res.portno, res.iface_index);
	} else {
		snprintf(buf, buflen, "port=%i", res.portno);
	}

done:
	mtx_unlock(&usb_global_lock);

	return (0);
}

static int
uhub_child_pnpinfo_string(device_t parent, device_t child,
    char *buf, size_t buflen)
{
	struct uhub_softc *sc = device_get_softc(parent);
	struct usbd_hub *hub = sc->sc_udev->hub;
	struct usbd_interface *iface;
	struct hub_result res;

	mtx_lock(&usb_global_lock);
	uhub_find_iface_index(hub, child, &res);
	if (!res.udev) {
		DPRINTF(sc, 0, "device not on hub\n");
		if (buflen) {
			buf[0] = '\0';
		}
		goto done;
	}
	iface = usbd_get_iface(res.udev, res.iface_index);

	if ((res.udev->probed == USBD_PROBED_IFACE_AND_FOUND) &&
	    iface && iface->idesc) {
		snprintf(buf, buflen, "vendor=0x%04x product=0x%04x "
		    "devclass=0x%02x devsubclass=0x%02x "
		    "sernum=\"%s\" "
		    "intclass=0x%02x intsubclass=0x%02x",
		    UGETW(res.udev->ddesc.idVendor),
		    UGETW(res.udev->ddesc.idProduct),
		    res.udev->ddesc.bDeviceClass,
		    res.udev->ddesc.bDeviceSubClass,
		    res.udev->serial,
		    iface->idesc->bInterfaceClass,
		    iface->idesc->bInterfaceSubClass);
	} else {
		snprintf(buf, buflen, "vendor=0x%04x product=0x%04x "
		    "devclass=0x%02x devsubclass=0x%02x "
		    "sernum=\"%s\"",
		    UGETW(res.udev->ddesc.idVendor),
		    UGETW(res.udev->ddesc.idProduct),
		    res.udev->ddesc.bDeviceClass,
		    res.udev->ddesc.bDeviceSubClass,
		    res.udev->serial);
	}

done:
	mtx_unlock(&usb_global_lock);

	return (0);
}
