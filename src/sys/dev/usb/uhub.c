#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uhub.c,v 1.74 2007/02/03 21:11:11 flz Exp $");

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

#define	UHUB_INTR_INTERVAL 250	/* ms */

#ifdef USB_DEBUG
#define DPRINTF(sc,n,fmt,...)	\
  do { if (uhub_debug > (n)) {	     \
      printf("%s:%s: " fmt, (sc)->sc_name, \
	     __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uhub_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uhub, CTLFLAG_RW, 0, "USB uhub");
SYSCTL_INT(_hw_usb_uhub, OID_AUTO, debug, CTLFLAG_RW, &uhub_debug, 0,
	   "uhub debug level");
#else
#define DPRINTF(...) do { } while (0)
#endif

struct uhub_softc {
	device_t		sc_dev;		/* base device */
	struct usbd_device	*sc_hub;	/* USB device */
	struct usbd_xfer	*sc_xfer[2];	/* interrupt xfer */
	uint8_t			sc_flags;
#define	UHUB_FLAG_RUNNING 0x01
#define	UHUB_FLAG_INTR_STALL 0x02
	uint8_t			sc_name[32];
};
#define	UHUB_PROTO(sc) ((sc)->sc_hub->ddesc.bDeviceProtocol)
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
      .type      = UE_INTERRUPT,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_ANY,
      .timeout   = 0,
      .flags     = (USBD_PIPE_BOF|USBD_SHORT_XFER_OK),
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &uhub_intr_callback,
      .interval  = UHUB_INTR_INTERVAL,
    },

    [1] = {
      .type      = UE_CONTROL,
      .endpoint  = 0,
      .direction = UE_DIR_ANY,
      .timeout   = 1000, /* 1 second */
      .interval  = 50, /* 50ms */
      .flags     = 0,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &uhub_intr_clear_stall_callback,
    },
};

/*
 * driver instance for "hub" connected to "usb" 
 * and "hub" connected to "hub"
 */
static devclass_t uhub_devclass;

static driver_t uhub_driver =
{
	.name    = "uhub",
	.methods = (device_method_t [])
        {
	  DEVMETHOD(device_probe, uhub_probe),
	  DEVMETHOD(device_attach, uhub_attach),
	  DEVMETHOD(device_detach, uhub_detach),

	  DEVMETHOD(device_suspend, bus_generic_suspend),
	  DEVMETHOD(device_resume, bus_generic_resume),
	  DEVMETHOD(device_shutdown, bus_generic_shutdown),

	  DEVMETHOD(bus_child_location_str, uhub_child_location_string),
	  DEVMETHOD(bus_child_pnpinfo_str, uhub_child_pnpinfo_string),
	  DEVMETHOD(bus_driver_added, uhub_driver_added),
	  {0,0}
	},
	.size    = sizeof(struct uhub_softc)
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

	USBD_CHECK_STATUS(xfer);

 tr_transferred:
	DPRINTF(sc, 1, "\n");
	/*
	 * This is an indication that some port
	 * has changed status. Notify the bus
	 * event handler thread that we need
	 * to be explored again:
	 */
	usb_needs_explore(sc->sc_hub);

 tr_setup:
	if (sc->sc_flags & UHUB_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[1]);
	} else {
	    usbd_start_hardware(xfer);
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* start clear stall */
	    sc->sc_flags |= UHUB_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[1]);
	}
	return;
}

static usbd_status
uhub_explore(struct usbd_device *udev)
{
	usb_hub_descriptor_t *hd = &udev->hub->hubdesc;
	struct uhub_softc *sc = udev->hub->hubsoftc;
	struct usbd_port *up;
	usbd_status err;
	uint16_t port;
	uint16_t change;
	uint16_t status;
	uint8_t speed;

	DPRINTF(sc, 10, "udev=%p addr=%d\n", udev, udev->address);

	if (!(sc->sc_flags & UHUB_FLAG_RUNNING))
	{
		return (USBD_NOT_STARTED);
	}

	/* ignore hubs that are too deep */
	if(udev->depth > USB_HUB_MAX_DEPTH)
	{
		return (USBD_TOO_DEEP);
	}

	for(port = 1; port <= hd->bNbrPorts; port++)
	{
		up = &udev->hub->ports[port-1];
		err = usbreq_get_port_status(udev, port, &up->status);
		if (err)
		{
			DPRINTF(sc, 0, "get port status failed, "
				"error=%s\n", usbd_errstr(err));
			continue;
		}
		status = UGETW(up->status.wPortStatus);
		change = UGETW(up->status.wPortChange);
		DPRINTF(sc, 3, "port %d status 0x%04x 0x%04x\n",
			port, status, change);
		if(change & UPS_C_PORT_ENABLED)
		{
			DPRINTF(sc, 0, "C_PORT_ENABLED 0x%x\n", change);
			usbreq_clear_port_feature(udev, port, UHF_C_PORT_ENABLE);
			if(change & UPS_C_CONNECT_STATUS)
			{
				/* ignore the port error
				 * if the device vanished
				 */
			}
			else if(status & UPS_PORT_ENABLED)
			{
				DPRINTF(sc, -1, "illegal enable change, "
					"port %d\n", port);
			}
			else
			{
				/* port error condition */
				if(up->restartcnt) /* no message first time */
				{
					DPRINTF(sc, -1, "port error, restarting "
						"port %d\n", port);
				}

				if(up->restartcnt++ < USBD_RESTART_MAX) {
					goto disconnect;
				} else {
					DPRINTF(sc, -1, "port error, giving up "
						"port %d\n", port);
				}
			}
		}
		if(!(change & UPS_C_CONNECT_STATUS))
		{
			DPRINTF(sc, 3, "port=%d !C_CONNECT_"
				"STATUS\n", port);
			/* no status change, just do recursive explore */
			if(up->device != NULL)
			{
				if(up->device->hub != NULL)
				{
					(up->device->hub->explore)(up->device);
				}
				else
				{
					/* allow drivers to be hot-plugged */

					if(up->last_refcount != usb_driver_added_refcount)
					{
						usbd_probe_and_attach
						  (sc->sc_dev, port, up);
					}
				}
			}
			continue;
		}

		/* we have a connect status change, handle it */

		DPRINTF(sc, 0, "status change hub=%d port=%d\n",
			udev->address, port);
		usbreq_clear_port_feature(udev, port, UHF_C_PORT_CONNECTION);
		/*usbreq_clear_port_feature(udev, port, UHF_C_PORT_ENABLE);*/
		/*
		 * If there is already a device on the port the change status
		 * must mean that is has disconnected.  Looking at the
		 * current connect status is not enough to figure this out
		 * since a new unit may have been connected before we handle
		 * the disconnect.
		 */
	disconnect:
		if(up->device != NULL)
		{
			/* disconnected */
			DPRINTF(sc, 0, "device addr=%d disappeared "
				"on port %d\n", up->device->address, port);
			usbd_free_device(up, 1);
			usbreq_clear_port_feature(udev, port,
						  UHF_C_PORT_CONNECTION);
		}
		if(!(status & UPS_CURRENT_CONNECT_STATUS))
		{
			/* nothing connected, just ignore it */
			DPRINTF(sc, 3, "port=%d !CURRENT_CONNECT_STATUS\n",
				port);
			continue;
		}

		/* connected */

		if(!(status & UPS_PORT_POWER))
		{
			DPRINTF(sc, -1, "strange, connected port %d "
				"has no power\n", port);
		}

		/* wait for maximum device power up time */
		usbd_delay_ms(udev, USB_PORT_POWERUP_DELAY);

		/* reset port, which implies enabling it */
		if(usbreq_reset_port(udev, port, &up->status))
		{
			DPRINTF(sc, -1, "port %d reset "
				"failed\n", port);
			continue;
		}

		/* get port status again, it might have changed during reset */
		err = usbreq_get_port_status(udev, port, &up->status);
		if(err)
		{
			DPRINTF(sc, 0, "get port status failed, "
				"error=%s\n", usbd_errstr(err));
			continue;
		}
		status = UGETW(up->status.wPortStatus);
		change = UGETW(up->status.wPortChange);
		if(!(status & UPS_CURRENT_CONNECT_STATUS))
		{
			/* nothing connected, just ignore it */
			DPRINTF(sc, 1, "port %d, device disappeared "
				"after reset\n", port);
			continue;
		}

		/* figure out device speed */
		speed = 
		  (status & UPS_HIGH_SPEED) ? USB_SPEED_HIGH :
		  (status & UPS_LOW_SPEED) ? USB_SPEED_LOW : USB_SPEED_FULL;

		/* get device info and set its address */
		err = usbd_new_device(sc->sc_dev, udev->bus,
				      udev->depth + 1, speed, port, up);
		if (err)
		{
			DPRINTF(sc, -1, "usb_new_device failed, "
				"error=%s\n", usbd_errstr(err));
			/* Avoid addressing problems by disabling. */
			/* usbd_reset_port(udev, port, &up->status); */

			/*
			 * The unit refused to accept a new address, or had
			 * some other serious problem.  Since we cannot leave
			 * at 0 we have to disable the port instead.
			 */
			DPRINTF(sc, -1, "device problem (%s), "
				"disabling port %d\n", usbd_errstr(err), port);
			usbreq_clear_port_feature(udev, port, UHF_PORT_ENABLE);
		}
		else
		{
			/* the port set up succeeded, reset error count */
			up->restartcnt = 0;

			if(up->device->hub)
			{
				(up->device->hub->explore)(up->device);
			}
		}
	}
	return (USBD_NORMAL_COMPLETION);
}

static int
uhub_probe(device_t dev)
{
        struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_device_descriptor_t *dd = usbd_get_device_descriptor(uaa->device);

	/*
	 * the subclass for hubs, is ignored,
	 * because it is 0 for some
	 * and 1 for others
	 */
	if((uaa->iface == NULL) && (dd->bDeviceClass == UDCLASS_HUB))
	{
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
	struct usbd_hub *hub;
	usb_device_request_t req;
	usb_hub_descriptor_t hubdesc;
	uint16_t port;
	uint16_t nports;
	uint16_t removable;
	uint16_t pwrdly;
	usbd_status err;

	if (sc == NULL) {
	    return ENOMEM;
	}

	sc->sc_hub = udev;
	sc->sc_dev = dev; 

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
		 device_get_nameunit(dev));

	usbd_set_desc(dev, udev);

	err = usbd_set_config_index(udev, 0, 1);
	if(err)
	{
		DPRINTF(sc, -1, "configuration failed, error=%s\n",
			usbd_errstr(err));
		goto error;
	}

	/* NOTE: "usbd_set_config_index()" will change
	 * variables in "udev" !
	 */
	DPRINTF(sc, 1, "depth=%d selfpowered=%d, parent=%p, "
		    "parent->selfpowered=%d\n",
		    udev->depth,
		    udev->self_powered, 
		    udev->powersrc->parent,
		    udev->powersrc->parent ?
		    udev->powersrc->parent->self_powered : 0);

	if(udev->depth > USB_HUB_MAX_DEPTH)
	{
		DPRINTF(sc, -1, "hub depth, %d, exceeded. HUB ignored!\n",
			USB_HUB_MAX_DEPTH);
		goto error;
	}

	if(!udev->self_powered && 
	    (udev->powersrc->parent != NULL) &&
	    (!udev->powersrc->parent->self_powered))
	{
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

	err = usbd_do_request(udev, &req, &hubdesc);

	nports = hubdesc.bNbrPorts;

	if(!err && (nports >= 8))
	{
		uint16_t len = (USB_HUB_DESCRIPTOR_SIZE-1) + ((nports+7) / 8);
		USETW(req.wLength, len);
		err = usbd_do_request(udev, &req, &hubdesc);
	}

	if(err)
	{
		DPRINTF(sc, -1, "getting hub descriptor failed,"
			"error=%s\n", usbd_errstr(err));
		goto error;
	}

	if(hubdesc.bNbrPorts != nports)
	{
		DPRINTF(sc, -1, "number of ports changed!\n");
		goto error;
	}

	if(nports == 0)
	{
		DPRINTF(sc, -1, "portless HUB!\n");
		goto error;
	}

	udev->hub = malloc((sizeof(hub[0]) + (sizeof(hub[0].ports[0]) * nports)),
			   M_USBDEV, M_WAITOK|M_ZERO);

	if(udev->hub == NULL)
	{
		goto error;
	}

	/* init FULL-speed ISOCHRONOUS schedule */
	usbd_fs_isoc_schedule_init_all(udev->hub->fs_isoc_schedule);

	/* set up interrupt pipe */
	err = usbd_transfer_setup(udev, 0, sc->sc_xfer, 
				  uhub_config, 2, sc, &usb_global_lock);
	if(err) 
	{
		DPRINTF(sc, -1, "cannot setup interrupt transfer, "
			"errstr=%s!\n", usbd_errstr(err));
		goto error;
	}

	hub = udev->hub;

	hub->hubsoftc = sc;
	hub->explore = uhub_explore;
	hub->hubdesc = hubdesc;

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

	for(port = 1;
	    port <= nports;
	    port++)
	{
		/* set up data structures */
		struct usbd_port *up = &hub->ports[port-1];
		up->device = NULL;
		up->parent = udev;
		up->portno = port;

		/* self powered hub, give ports maximum current */
		up->power = (udev->self_powered) ? 
		  USB_MAX_POWER :
		  USB_MIN_POWER ;

		up->restartcnt = 0;

		/* check if port is removable */
		if(!UHD_NOT_REMOV(&hubdesc, port))
		{
			removable++;
		}

		/* turn the power on */
		err = usbreq_set_port_feature(udev, port, UHF_PORT_POWER);
		if(err)
		{
			DPRINTF(sc, -1, "port %d power on failed, %s\n",
				port, usbd_errstr(err));
		}
		DPRINTF(sc, 0, "turn on port %d power\n", port);

		/* wait for stable power */
		usbd_delay_ms(udev, pwrdly);
	}

	device_printf(dev, "%d port%s with %d "
		      "removable, %s powered\n",
		      nports, (nports != 1) ? "s" : "",
		      removable, udev->self_powered ? "self" : "bus");

	/* the usual exploration will finish the setup */

	sc->sc_flags |= UHUB_FLAG_RUNNING;

	/* start the interrupt endpoint */

	mtx_lock(sc->sc_xfer[0]->priv_mtx);
	usbd_transfer_start(sc->sc_xfer[0]);
	mtx_unlock(sc->sc_xfer[0]->priv_mtx);

	return 0;

 error:
	usbd_transfer_unsetup(sc->sc_xfer, 2);

	if(udev->hub)
	{
		free(udev->hub, M_USBDEV);
		udev->hub = NULL;
	}
	return ENXIO;
}

/*
 * Called from process context when the hub is gone.
 * Detach all devices on active ports.
 */
static int
uhub_detach(device_t dev)
{
        struct uhub_softc *sc = device_get_softc(dev);
	struct usbd_hub *hub = sc->sc_hub->hub;
	struct usbd_port *up;
	uint16_t port;
	uint16_t nports;

	/* detach all children first */
	bus_generic_detach(dev);

	if(hub == NULL)		/* must be partially working */
	{
		return (0);
	}

	nports = hub->hubdesc.bNbrPorts;
	for(port = 0;
	    port < nports;
	    port++)
	{
		up = &hub->ports[port];
		if(up->device)
		{
			/* subdevices are not freed, because
			 * the caller of uhub_detach() will
			 * do that
			 */
			usbd_free_device(up, 0);
		}
	}

	usbd_transfer_unsetup(sc->sc_xfer, 2);

	free(hub, M_USBDEV);
	sc->sc_hub->hub = NULL;
	return (0);
}

static void
uhub_driver_added(device_t dev, driver_t *driver)
{
	usb_needs_probe_and_attach();
	return;
}

static int
uhub_child_location_string(device_t parent, device_t child, 
			   char *buf, size_t buflen)
{
	struct uhub_softc *sc = device_get_softc(parent);
	struct usbd_hub *hub = sc->sc_hub->hub;
	struct usbd_device *udev;
	uint16_t port;
	uint16_t nports;
	uint8_t iface_index;

	mtx_lock(&usb_global_lock);

	nports = hub->hubdesc.bNbrPorts;
        for(port = 0;
            port < nports;
            port++)
        {
		udev = hub->ports[port].device;
		if(udev)
		{
			device_t * subdev = 
			  &udev->subdevs[0];
			device_t * subdev_end = 
			  &udev->subdevs_end[0];

			iface_index = 0;

			while(subdev < subdev_end)
			{
				if(subdev[0] == child)
				{
					goto found;
				}

				subdev++;
				iface_index++;
			}
		}
	}

	mtx_unlock(&usb_global_lock);

	DPRINTF(sc, 0, "device not on hub\n");

	if(buflen)
	{
		buf[0] = '\0';
	}

	return 0;


 found:

	if(udev->probed == USBD_PROBED_IFACE_AND_FOUND)
	{
		snprintf(buf, buflen, "port=%i interface=%i",
			 port, iface_index);
	}
	else
	{
                snprintf(buf, buflen, "port=%i", port);
	}

	mtx_unlock(&usb_global_lock);

        return (0);
}

static int
uhub_child_pnpinfo_string(device_t parent, device_t child, 
			  char *buf, size_t buflen)
{
	struct uhub_softc *sc = device_get_softc(parent);
	struct usbd_hub *hub = sc->sc_hub->hub;
        struct usbd_interface *iface;
	struct usbd_device *udev;
	uint16_t port;
	uint16_t nports;
	uint8_t iface_index;

	mtx_lock(&usb_global_lock);

	nports = hub->hubdesc.bNbrPorts;
        for(port = 0;
            port < nports;
            port++)
        {
		udev = hub->ports[port].device;
		if(udev)
		{
			device_t * subdev = 
			  &udev->subdevs[0];
			device_t * subdev_end = 
			  &udev->subdevs_end[0];

			iface_index = 0;

			while(subdev < subdev_end)
			{
				if(subdev[0] == child)
				{
					goto found;
				}

				subdev++;
				iface_index++;
			}
		}
	}

	mtx_unlock(&usb_global_lock);

	DPRINTF(sc, 0, "device not on hub\n");

	if(buflen)
	{
		buf[0] = '\0';
	}

	return 0;

 found:

	iface = usbd_get_iface(udev, iface_index);

	if((udev->probed == USBD_PROBED_IFACE_AND_FOUND) &&
	   iface && iface->idesc)
	{
		snprintf(buf, buflen, "vendor=0x%04x product=0x%04x "
			 "devclass=0x%02x devsubclass=0x%02x "
			 "sernum=\"%s\" "
			 "intclass=0x%02x intsubclass=0x%02x",
			 UGETW(udev->ddesc.idVendor),
			 UGETW(udev->ddesc.idProduct),
			 udev->ddesc.bDeviceClass,
			 udev->ddesc.bDeviceSubClass,
			 &udev->serial[0],
			 iface->idesc->bInterfaceClass,
			 iface->idesc->bInterfaceSubClass);
	}
	else
	{
		snprintf(buf, buflen, "vendor=0x%04x product=0x%04x "
			 "devclass=0x%02x devsubclass=0x%02x "
			 "sernum=\"%s\"",
			 UGETW(udev->ddesc.idVendor), 
			 UGETW(udev->ddesc.idProduct),
			 udev->ddesc.bDeviceClass, 
			 udev->ddesc.bDeviceSubClass,
			 &udev->serial[0]);
	}

	mtx_unlock(&usb_global_lock);

        return (0);
}
