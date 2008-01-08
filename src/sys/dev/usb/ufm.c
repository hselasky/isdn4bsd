/*-
 * Copyright (c) 2001 M. Warner Losh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This code is based on ugen.c and ulpt.c developed by Lennart Augustsson.
 * This code includes software developed by the NetBSD Foundation, Inc. and
 * its contributors.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ufm.c,v 1.36 2007/06/21 14:42:33 imp Exp $");


#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/fcntl.h>
#include <sys/filio.h>
#include <sys/conf.h>
#include <sys/sysctl.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>

#include "usbdevs.h"

#include <dev/usb/dsbr100io.h>

#ifdef USB_DEBUG
#define	DPRINTF(sc,n,fmt,...)   \
  do { if (ufm_debug > (n)) {        \
      printf("%s:%s: " fmt, (sc)->sc_name, \
             __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ufm_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, ufm, CTLFLAG_RW, 0, "USB ufm");
SYSCTL_INT(_hw_usb_ufm, OID_AUTO, debug, CTLFLAG_RW,
    &ufm_debug, 0, "ufm debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

#define	UFM_CMD0		0x00
#define	UFM_CMD_SET_FREQ	0x01
#define	UFM_CMD2		0x02

struct ufm_softc {
	struct usb_cdev sc_cdev;
	struct mtx sc_mtx;

	struct usbd_device *sc_udev;

	uint32_t sc_unit;
	uint32_t sc_freq;

	uint8_t	sc_name[16];
};

/* prototypes */

static device_probe_t ufm_probe;
static device_attach_t ufm_attach;
static device_detach_t ufm_detach;

static int32_t
ufm_open(struct usb_cdev *dev, int32_t fflags,
    int32_t devtype, struct thread *td);

static int
ufm_do_req(struct ufm_softc *sc, int32_t fflags, uint8_t request,
    uint16_t value, uint16_t index, uint8_t *retbuf);
static int
	ufm_set_freq(struct ufm_softc *sc, caddr_t addr, int32_t fflags);

static int
	ufm_get_freq(struct ufm_softc *sc, caddr_t addr, int32_t fflags);

static int
	ufm_start(struct ufm_softc *sc, caddr_t addr, int32_t fflags);

static int
	ufm_stop(struct ufm_softc *sc, caddr_t addr, int32_t fflags);

static int
	ufm_get_stat(struct ufm_softc *sc, caddr_t addr, int32_t fflags);

static int
ufm_ioctl(struct usb_cdev *dev, u_long cmd, caddr_t addr,
    int32_t fflags, struct thread *td);

static devclass_t ufm_devclass;

static device_method_t ufm_methods[] = {
	DEVMETHOD(device_probe, ufm_probe),
	DEVMETHOD(device_attach, ufm_attach),
	DEVMETHOD(device_detach, ufm_detach),
	{0, 0}
};

static driver_t ufm_driver = {
	.name = "ufm",
	.methods = ufm_methods,
	.size = sizeof(struct ufm_softc),
};

MODULE_DEPEND(ufm, usb, 1, 1, 1);
DRIVER_MODULE(ufm, uhub, ufm_driver, ufm_devclass, usbd_driver_load, 0);

static int
ufm_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (UMATCH_NONE);
	}
	if (uaa->iface == NULL) {
		return (UMATCH_NONE);
	}
	if ((uaa->vendor == USB_VENDOR_CYPRESS) &&
	    (uaa->product == USB_PRODUCT_CYPRESS_FMRADIO)) {
		return (UMATCH_VENDOR_PRODUCT);
	}
	return (UMATCH_NONE);
}

static int
ufm_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ufm_softc *sc = device_get_softc(dev);
	const char *p_buf[2];
	char buf[16];
	int32_t error;

	if (sc == NULL) {
		return (ENOMEM);
	}
	sc->sc_udev = uaa->device;
	sc->sc_unit = device_get_unit(dev);

	snprintf(sc->sc_name, sizeof(sc->sc_name), "%s",
	    device_get_nameunit(dev));

	mtx_init(&(sc->sc_mtx), "ufm lock", NULL, MTX_DEF | MTX_RECURSE);

	usbd_set_device_desc(dev);

	snprintf(buf, sizeof(buf), "ufm%d", sc->sc_unit);

	p_buf[0] = buf;
	p_buf[1] = NULL;

	sc->sc_cdev.sc_open = &ufm_open;
	sc->sc_cdev.sc_ioctl = &ufm_ioctl;

	error = usb_cdev_attach(&(sc->sc_cdev), sc, &(sc->sc_mtx), p_buf,
	    UID_ROOT, GID_OPERATOR, 0644, 0, 0, 0, 0);
	if (error) {
		goto detach;
	}
	return (0);			/* success */

detach:
	ufm_detach(dev);
	return (ENXIO);
}

static int
ufm_detach(device_t dev)
{
	struct ufm_softc *sc = device_get_softc(dev);

	usb_cdev_detach(&(sc->sc_cdev));

	mtx_destroy(&(sc->sc_mtx));

	return (0);
}

static int32_t
ufm_open(struct usb_cdev *dev, int32_t fflags,
    int32_t devtype, struct thread *td)
{
	if ((fflags & (FWRITE | FREAD)) != (FWRITE | FREAD)) {
		return (EACCES);
	}
	return (0);
}

static int
ufm_do_req(struct ufm_softc *sc, int32_t fflags, uint8_t request,
    uint16_t value, uint16_t index, uint8_t *retbuf)
{
	int32_t error;

	usb_device_request_t req;
	uint8_t buf[1];

	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = request;
	USETW(req.wValue, value);
	USETW(req.wIndex, index);
	USETW(req.wLength, 1);

	error = usbd_do_request(sc->sc_udev, NULL, &req, buf);

	if (retbuf) {
		*retbuf = buf[0];
	}
	if (error) {
		return (ENXIO);
	}
	return (0);
}

static int
ufm_set_freq(struct ufm_softc *sc, caddr_t addr, int32_t fflags)
{
	int freq = *(int *)addr;

	/*
	 * Freq now is in Hz.  We need to convert it to the frequency
	 * that the radio wants.  This frequency is 10.7MHz above
	 * the actual frequency.  We then need to convert to
	 * units of 12.5kHz.  We add one to the IFM to make rounding
	 * easier.
	 */
	mtx_lock(&(sc->sc_mtx));
	sc->sc_freq = freq;
	mtx_unlock(&(sc->sc_mtx));

	freq = (freq + 10700001) / 12500;

	/* This appears to set the frequency */
	if (ufm_do_req(sc, fflags, UFM_CMD_SET_FREQ,
	    freq >> 8, freq, NULL) != 0) {
		return (EIO);
	}
	/* Not sure what this does */
	if (ufm_do_req(sc, fflags, UFM_CMD0,
	    0x96, 0xb7, NULL) != 0) {
		return (EIO);
	}
	return (0);
}

static int
ufm_get_freq(struct ufm_softc *sc, caddr_t addr, int32_t fflags)
{
	int *valp = (int *)addr;

	mtx_lock(&(sc->sc_mtx));
	*valp = sc->sc_freq;
	mtx_unlock(&(sc->sc_mtx));
	return (0);
}

static int
ufm_start(struct ufm_softc *sc, caddr_t addr, int32_t fflags)
{
	uint8_t ret;

	if (ufm_do_req(sc, fflags, UFM_CMD0,
	    0x00, 0xc7, &ret)) {
		return (EIO);
	}
	if (ufm_do_req(sc, fflags, UFM_CMD2,
	    0x01, 0x00, &ret)) {
		return (EIO);
	}
	if (ret & 0x1) {
		return (EIO);
	}
	return (0);
}

static int
ufm_stop(struct ufm_softc *sc, caddr_t addr, int32_t fflags)
{
	if (ufm_do_req(sc, fflags, UFM_CMD0,
	    0x16, 0x1C, NULL)) {
		return (EIO);
	}
	if (ufm_do_req(sc, fflags, UFM_CMD2,
	    0x00, 0x00, NULL)) {
		return (EIO);
	}
	return (0);
}

static int
ufm_get_stat(struct ufm_softc *sc, caddr_t addr, int32_t fflags)
{
	uint8_t ret;

	/*
	 * Note, there's a 240ms settle time before the status
	 * will be valid, so sleep that amount.
	 */

	usbd_pause_mtx(NULL, 250);

	if (ufm_do_req(sc, fflags, UFM_CMD0,
	    0x00, 0x24, &ret)) {
		return (EIO);
	}
	*(int *)addr = ret;

	return (0);
}

static int
ufm_ioctl(struct usb_cdev *dev, u_long cmd, caddr_t addr,
    int32_t fflags, struct thread *td)
{
	struct ufm_softc *sc = dev->sc_priv_ptr;
	int error = 0;

	usb_cdev_unlock(dev, fflags);

	switch (cmd) {
	case FM_SET_FREQ:
		error = ufm_set_freq(sc, addr, fflags);
		break;
	case FM_GET_FREQ:
		error = ufm_get_freq(sc, addr, fflags);
		break;
	case FM_START:
		error = ufm_start(sc, addr, fflags);
		break;
	case FM_STOP:
		error = ufm_stop(sc, addr, fflags);
		break;
	case FM_GET_STAT:
		error = ufm_get_stat(sc, addr, fflags);
		break;
	default:
		error = ENOTTY;
		break;
	}
	return (usb_cdev_lock(dev, fflags, error));
}
