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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ums.c,v 1.83 2007/01/17 03:50:45 markus Exp $");

/*
 * HID spec: http://www.usb.org/developers/devclass_docs/HID1_11.pdf
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/ioccom.h>
#include <sys/filio.h>
#include <sys/tty.h>
#include <sys/file.h>
#include <sys/vnode.h>
#include <sys/poll.h>
#include <sys/mouse.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_hid.h>

#ifdef USB_DEBUG
#define DPRINTF(n,fmt,...)						\
  do { if (ums_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ums_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ums, CTLFLAG_RW, 0, "USB ums");
SYSCTL_INT(_hw_usb_ums, OID_AUTO, debug, CTLFLAG_RW,
	   &ums_debug, 0, "ums debug level");
#else
#define DPRINTF(...)
#endif

#define DEV2SC(dev) (dev)->si_drv1

#define MOUSE_FLAGS_MASK (HIO_CONST|HIO_RELATIVE)
#define MOUSE_FLAGS (HIO_RELATIVE)

#define UMS_BUF_SIZE      8 /* bytes */
#define UMS_IFQ_MAXLEN   50 /* units */
#define UMS_N_TRANSFER    2 /* units */
#define UMS_BUTTON_MAX   31 /* exclusive, must be less than 32 */
#define UMS_BUT(i) ((i) < 3 ? (((i) + 2) % 3) : (i))

struct ums_softc {
  struct usb_cdev     sc_cdev;
  struct mtx          sc_mtx;
  struct __callout    sc_callout;
  struct hid_location sc_loc_x; 
  struct hid_location sc_loc_y;
  struct hid_location sc_loc_z;
  struct hid_location sc_loc_t;
  struct hid_location sc_loc_btn[UMS_BUTTON_MAX];
  mousehw_t           sc_hw;
  mousemode_t         sc_mode;
  mousestatus_t	      sc_status;

  struct usbd_xfer *  sc_xfer[UMS_N_TRANSFER];

  u_int32_t	      sc_flags;
#define UMS_FLAG_X_AXIS     0x0001
#define UMS_FLAG_Y_AXIS     0x0002
#define UMS_FLAG_Z_AXIS     0x0004
#define UMS_FLAG_T_AXIS     0x0008
#define UMS_FLAG_SBU        0x0010 /* spurious button up events */
#define UMS_FLAG_INTR_STALL 0x0020 /* set if transfer error */

  u_int8_t	      sc_buttons;
  u_int8_t	      sc_iid;
};

static void
ums_put_queue_timeout(void *__sc);

static void
ums_clear_stall_callback(struct usbd_xfer *xfer);

static void
ums_intr_callback(struct usbd_xfer *xfer);

static device_probe_t ums_probe;
static device_attach_t ums_attach;
static device_detach_t ums_detach;

static void
ums_start_read(struct usb_cdev *cdev);

static void
ums_stop_read(struct usb_cdev *cdev);

static int32_t
ums_open(struct usb_cdev *cdev, int32_t fflags, 
	 int32_t devtype, struct thread *td);
static int32_t
ums_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t addr, 
	  int32_t fflags, struct thread *td);
static void
ums_put_queue(struct ums_softc *sc, int32_t dx, int32_t dy, 
	      int32_t dz, int32_t dt, int32_t buttons);

static void
ums_put_queue_timeout(void *__sc)
{
	struct ums_softc *sc = __sc;

	mtx_assert(&(sc->sc_mtx), MA_OWNED);

	ums_put_queue(sc, 0, 0, 0, 0, 0);

	mtx_unlock(&(sc->sc_mtx));

	return;
}

static void
ums_clear_stall_callback(struct usbd_xfer *xfer)
{
	struct ums_softc *sc = xfer->priv_sc;

	USBD_CHECK_STATUS(xfer);

 tr_setup:
	/* start clear stall */
	usbd_clear_stall_tr_setup(xfer, sc->sc_xfer[0]);
        return;

 tr_transferred:
	usbd_clear_stall_tr_transferred(xfer, sc->sc_xfer[0]);

	sc->sc_flags &= ~UMS_FLAG_INTR_STALL;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;

 tr_error:
	/* bomb out */
	sc->sc_flags &= ~UMS_FLAG_INTR_STALL;
	DPRINTF(0, "clear stall failed, error=%s!\n",
		usbd_errstr(xfer->error));
	return;
}

static void
ums_intr_callback(struct usbd_xfer *xfer)
{
	struct ums_softc *sc = xfer->priv_sc;
	struct usbd_mbuf *m;
	u_int8_t *buf = xfer->buffer;
	u_int16_t len = xfer->actlen;
	int32_t buttons = 0;
	int32_t dx;
	int32_t dy;
	int32_t dz;
	int32_t dt;
	u_int8_t i;

	USBD_CHECK_STATUS(xfer);

 tr_transferred:
	DPRINTF(5, "sc=%p actlen=%d\n", sc, len);
	DPRINTF(5, "data = %02x %02x %02x %02x "
		"%02x %02x %02x %02x\n",
		(len > 0) ? buf[0] : 0, (len > 1) ? buf[1] : 0,
		(len > 2) ? buf[2] : 0, (len > 3) ? buf[3] : 0, 
		(len > 4) ? buf[4] : 0, (len > 5) ? buf[5] : 0, 
		(len > 6) ? buf[6] : 0, (len > 7) ? buf[7] : 0);

	/*
	 * The M$ Wireless Intellimouse 2.0 sends 1 extra leading byte of
	 * data compared to most USB mice. This byte frequently switches
	 * from 0x01 (usual state) to 0x02. I assume it is to allow
	 * extra, non-standard, reporting (say battery-life). However
	 * at the same time it generates a left-click message on the button
	 * byte which causes spurious left-click's where there shouldn't be.
	 * This should sort that.
	 * Currently it's the only user of UMS_FLAG_T_AXIS so use it as an identifier.
	 * We probably should switch to some more official quirk.
	 */
	if (sc->sc_iid) {
	    if (sc->sc_flags & UMS_FLAG_T_AXIS) {
	      if (*buf == 0x02) {
		  goto tr_setup;
	      }
	    } else {
	      if (*buf != sc->sc_iid) {
		  goto tr_setup;
	      }
	    }

	    if (len) {
	      len--;
	      buf++;
	    } else {
	      goto tr_setup;
	    }
	}

	dx = (sc->sc_flags & UMS_FLAG_X_AXIS) ?
	  hid_get_data(buf, len, &sc->sc_loc_x) : 0;

	dy = (sc->sc_flags & UMS_FLAG_Y_AXIS) ?
	  -hid_get_data(buf, len, &sc->sc_loc_y) : 0;

	dz = (sc->sc_flags & UMS_FLAG_Z_AXIS) ?
	  -hid_get_data(buf, len, &sc->sc_loc_z) : 0;

	dt = (sc->sc_flags & UMS_FLAG_T_AXIS) ?
	  -hid_get_data(buf, len, &sc->sc_loc_t) : 0;

	for (i = 0; i < sc->sc_buttons; i++) {
	    if (hid_get_data(buf, len, &sc->sc_loc_btn[i])) {
	        buttons |= (1 << UMS_BUT(i));
	    }
	}

	if (dx || dy || dz || dt || (buttons != sc->sc_status.button)) {

	    DPRINTF(5, "x:%d y:%d z:%d t:%d buttons:0x%08x\n",
		    dx, dy, dz, dt, buttons);

	    sc->sc_status.button = buttons;
	    sc->sc_status.dx += dx;
	    sc->sc_status.dy += dy;
	    sc->sc_status.dz += dz;
	    /* sc->sc_status.dt += dt;*/ /* no way to export this yet */

	    /*
	     * The Qtronix keyboard has a built in PS/2 port for a mouse.
	     * The firmware once in a while posts a spurious button up
	     * event. This event we ignore by doing a timeout for 50 msecs.
	     * If we receive dx=dy=dz=buttons=0 before we add the event to
	     * the queue.
	     * In any other case we delete the timeout event.
	     */
	    if ((sc->sc_flags & UMS_FLAG_SBU) &&
		(dx == 0) && (dy == 0) && (dz == 0) && (dt == 0) && 
		(buttons == 0)) {

	        __callout_reset(&(sc->sc_callout), hz / 20, 
				&ums_put_queue_timeout, sc);
	    } else {

	        __callout_stop(&(sc->sc_callout));

		ums_put_queue(sc, dx, dy, dz, dt, buttons);
	    }
	}

 tr_setup:
	if (sc->sc_flags & UMS_FLAG_INTR_STALL) {
	    usbd_transfer_start(sc->sc_xfer[1]);
	} else {
	    USBD_IF_POLL(&(sc->sc_cdev.sc_rdq_free), m);

	    if (m) {
	        usbd_start_hardware(xfer);
	    }
	}
	return;

 tr_error:
	if (xfer->error != USBD_CANCELLED) {
	    /* start clear stall */
	    sc->sc_flags |= UMS_FLAG_INTR_STALL;
	    usbd_transfer_start(sc->sc_xfer[1]);
	}
	return;
}

static const struct usbd_config ums_config[UMS_N_TRANSFER] = {

    [0] = {
      .type      = UE_INTERRUPT,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .flags     = USBD_SHORT_XFER_OK,
      .bufsize   = 0, /* use wMaxPacketSize */
      .callback  = &ums_intr_callback,
    },

    [1] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = sizeof(usb_device_request_t),
      .callback  = &ums_clear_stall_callback,
      .timeout   = 1000, /* 1 second */
    },
};

static int
ums_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;
	void *d_ptr;
	int32_t d_len;
	int32_t error = 0;

	DPRINTF(10, "\n");

	if (uaa->iface == NULL) {
	    return UMATCH_NONE;
	}

	id = usbd_get_interface_descriptor(uaa->iface);

	if ((id == NULL) || 
	    (id->bInterfaceClass != UICLASS_HID)) {
	    return UMATCH_NONE;
	}

	error = usbreq_read_report_desc(uaa->device, uaa->iface_index, 
					&d_ptr, &d_len, M_TEMP);
	if (error) {
	    return UMATCH_NONE;
	}

	if (hid_is_collection(d_ptr, d_len, 
			      HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_MOUSE)))
	    error = UMATCH_IFACECLASS;
	else
	    error = UMATCH_NONE;

	free(d_ptr, M_TEMP);

	return error;
}

static int
ums_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct ums_softc *sc = device_get_softc(dev);
	void *d_ptr = NULL;
	const char * p_buf[2];
	int32_t unit = device_get_unit(dev);
	int32_t d_len;
	int32_t isize;
	u_int32_t flags;
	int32_t err;
	u_int8_t i;
	char buf_1[16];

	DPRINTF(10, "sc=%p\n", sc);

	usbd_set_desc(dev, uaa->device);

	mtx_init(&(sc->sc_mtx), "ums lock", NULL, MTX_DEF|MTX_RECURSE);

	__callout_init_mtx(&(sc->sc_callout),
			   &(sc->sc_mtx), CALLOUT_RETURNUNLOCKED);

	err = usbd_transfer_setup(uaa->device, uaa->iface_index, sc->sc_xfer, 
				  ums_config, UMS_N_TRANSFER, sc, 
				  &(sc->sc_mtx));
	if (err) {
	    DPRINTF(0, "error=%s\n", usbd_errstr(err)) ;
	    goto detach;
	}

	err = usbreq_read_report_desc(uaa->device, uaa->iface_index, 
				      &d_ptr, &d_len, M_TEMP);
	if (err) {
	    device_printf(dev, "error reading report description\n");
	    goto detach;
	}

	if (hid_locate(d_ptr, d_len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_X),
			hid_input, &sc->sc_loc_x, &flags)) {

	    if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
	        sc->sc_flags |= UMS_FLAG_X_AXIS;
	    }
	}

	if (hid_locate(d_ptr, d_len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_Y),
		       hid_input, &sc->sc_loc_y, &flags)) {

	    if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
	        sc->sc_flags |= UMS_FLAG_Y_AXIS;
	    }
	}

	/* try to guess the Z activator: first check Z, then WHEEL */

	if (hid_locate(d_ptr, d_len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_Z),
		       hid_input, &sc->sc_loc_z, &flags) ||
	    hid_locate(d_ptr, d_len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_WHEEL),
		       hid_input, &sc->sc_loc_z, &flags) ||
	    hid_locate(d_ptr, d_len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_TWHEEL),
		       hid_input, &sc->sc_loc_z, &flags)) {

	    if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
	        sc->sc_flags |= UMS_FLAG_Z_AXIS;
	    }
	}

	/* The Microsoft Wireless Intellimouse 2.0 reports it's wheel
	 * using 0x0048, which is HUG_TWHEEL, and seems to expect you
	 * to know that the byte after the wheel is the tilt axis.
	 * There are no other HID axis descriptors other than X,Y and 
	 * TWHEEL
	 */
	if (hid_locate(d_ptr, d_len, HID_USAGE2(HUP_GENERIC_DESKTOP, HUG_TWHEEL),
		       hid_input, &sc->sc_loc_t, &flags)) {

	    sc->sc_loc_t.pos += 8;

	    if ((flags & MOUSE_FLAGS_MASK) == MOUSE_FLAGS) {
	        sc->sc_flags |= UMS_FLAG_T_AXIS;
	    }
	}

	/* figure out the number of buttons */

	for (i = 0; i < UMS_BUTTON_MAX; i++) {
	    if (!hid_locate(d_ptr, d_len, HID_USAGE2(HUP_BUTTON, (i+1)),
			    hid_input, &(sc->sc_loc_btn[i]), NULL)) {
	        break;
	    }
	}

	sc->sc_buttons = i;

	device_printf(dev, "%d buttons and [%s%s%s%s] coordinates\n",
		      (sc->sc_buttons),
		      (sc->sc_flags & UMS_FLAG_X_AXIS) ? "X" : "",
		      (sc->sc_flags & UMS_FLAG_Y_AXIS) ? "Y" : "",
		      (sc->sc_flags & UMS_FLAG_Z_AXIS) ? "Z" : "",
		      (sc->sc_flags & UMS_FLAG_T_AXIS) ? "T" : "");

	isize = hid_report_size(d_ptr, d_len, hid_input, &sc->sc_iid);

	if (isize > sc->sc_xfer[0]->length) {
	    DPRINTF(0, "WARNING: report size, %d bytes, is larger "
		    "than interrupt size, %d bytes!\n",
		    isize, sc->sc_xfer[0]->length);
	}

	free(d_ptr, M_TEMP);
	d_ptr = NULL;

#ifdef USB_DEBUG
	DPRINTF(0, "sc=%p\n", sc);
	DPRINTF(0, "X\t%d/%d\n", sc->sc_loc_x.pos, sc->sc_loc_x.size);
	DPRINTF(0, "Y\t%d/%d\n", sc->sc_loc_y.pos, sc->sc_loc_y.size);
	DPRINTF(0, "Z\t%d/%d\n", sc->sc_loc_z.pos, sc->sc_loc_z.size);
	DPRINTF(0, "T\t%d/%d\n", sc->sc_loc_t.pos, sc->sc_loc_t.size);

	for (i = 0; i < sc->sc_buttons; i++) {
	    DPRINTF(0, "B%d\t%d/%d\n",
		    i+1 , sc->sc_loc_btn[i].pos, sc->sc_loc_btn[i].size);
	}
	DPRINTF(0, "size=%d, id=%d\n", isize, sc->sc_iid);
#endif

	if (sc->sc_buttons > MOUSE_MSC_MAXBUTTON)
	    sc->sc_hw.buttons = MOUSE_MSC_MAXBUTTON;
	else
	    sc->sc_hw.buttons = sc->sc_buttons;

	sc->sc_hw.iftype = MOUSE_IF_USB;
	sc->sc_hw.type = MOUSE_MOUSE;
	sc->sc_hw.model = MOUSE_MODEL_GENERIC;
	sc->sc_hw.hwid = 0;

	sc->sc_mode.protocol = MOUSE_PROTO_MSC;
	sc->sc_mode.rate = -1;
	sc->sc_mode.resolution = MOUSE_RES_UNKNOWN;
	sc->sc_mode.accelfactor = 0;
	sc->sc_mode.level = 0;
	sc->sc_mode.packetsize = MOUSE_MSC_PACKETSIZE;
	sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
	sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;

	sc->sc_status.flags = 0;
	sc->sc_status.button = 0;
	sc->sc_status.obutton = 0;
	sc->sc_status.dx = 0;
	sc->sc_status.dy = 0;
	sc->sc_status.dz = 0;

	snprintf(buf_1, sizeof(buf_1), "ums%d", unit);

	p_buf[0] = buf_1;
	p_buf[1] = NULL;

	sc->sc_cdev.sc_start_read = &ums_start_read;
	sc->sc_cdev.sc_stop_read = &ums_stop_read;
	sc->sc_cdev.sc_open = &ums_open;
	sc->sc_cdev.sc_ioctl = &ums_ioctl;
	sc->sc_cdev.sc_flags |= (USB_CDEV_FLAG_FWD_SHORT|
				 USB_CDEV_FLAG_WAKEUP_RD_IMMED|
				 USB_CDEV_FLAG_WAKEUP_WR_IMMED);

	err = usb_cdev_attach(&(sc->sc_cdev), sc, &(sc->sc_mtx), p_buf,
			      UID_ROOT, GID_OPERATOR, 0644, 
			      UMS_BUF_SIZE, UMS_IFQ_MAXLEN,
			      1, 1 /* dummy write buffer */);
	if (err) {
	    goto detach;
	}

	return 0;

 detach:
	if (d_ptr) {
	    free(d_ptr, M_TEMP);
	}
	ums_detach(dev);
	return ENOMEM;
}

static int
ums_detach(device_t self)
{
	struct ums_softc *sc = device_get_softc(self);

	DPRINTF(0, "sc=%p\n", sc);

	usb_cdev_detach(&(sc->sc_cdev));

	usbd_transfer_unsetup(sc->sc_xfer, UMS_N_TRANSFER);

	__callout_drain(&(sc->sc_callout));

	mtx_destroy(&(sc->sc_mtx));

	return 0;
}

static void
ums_start_read(struct usb_cdev *cdev)
{
	struct ums_softc *sc = cdev->sc_priv_ptr;
	usbd_transfer_start(sc->sc_xfer[0]);
	return;
}

static void
ums_stop_read(struct usb_cdev *cdev)
{
	struct ums_softc *sc = cdev->sc_priv_ptr;
	usbd_transfer_stop(sc->sc_xfer[1]);
	usbd_transfer_stop(sc->sc_xfer[0]);
	__callout_stop(&(sc->sc_callout));
	return;
}


#if ((MOUSE_SYS_PACKETSIZE != 8) || \
     (MOUSE_MSC_PACKETSIZE != 5))
#error "Software assumptions are not met. Please update code."
#endif

static void
ums_put_queue(struct ums_softc *sc, int32_t dx, int32_t dy, 
	      int32_t dz, int32_t dt, int32_t buttons)
{
	u_int8_t buf[8];

	if (1) {

	    if (dx >  254)		dx =  254;
	    if (dx < -256)		dx = -256;
	    if (dy >  254)		dy =  254;
	    if (dy < -256)		dy = -256;
	    if (dz >  126)		dz =  126;
	    if (dz < -128)		dz = -128;
	    if (dt >  126)		dt =  126;
	    if (dt < -128)		dt = -128;

	    buf[0] = sc->sc_mode.syncmask[1];
	    buf[0] |= (~buttons) & MOUSE_MSC_BUTTONS;
	    buf[1] = dx >> 1;
	    buf[2] = dy >> 1;
	    buf[3] = dx - (dx >> 1);
	    buf[4] = dy - (dy >> 1);

	    if (sc->sc_mode.level == 1) {
	        buf[5] = dz >> 1;
		buf[6] = dz - (dz >> 1);
		buf[7] = (((~buttons) >> 3) & MOUSE_SYS_EXTBUTTONS);
	    }

	    usb_cdev_put_data(&(sc->sc_cdev), buf, 
			      sc->sc_mode.packetsize, 1);

	} else {
	    DPRINTF(0, "Buffer full, discarded packet\n");
	}

	return;
}

static void
ums_reset_buf(struct ums_softc *sc)
{
	struct usbd_mbuf *m;

	/* reset read queue */

	while(1) {
	    USBD_IF_DEQUEUE(&(sc->sc_cdev.sc_rdq_used), m);

	    if (m) {
	      USBD_IF_ENQUEUE(&(sc->sc_cdev.sc_rdq_free), m);
	    } else {
	      break;
	    }
	}
	return;
}

static int32_t
ums_open(struct usb_cdev *cdev, int32_t fflags, 
	 int32_t devtype, struct thread *td)
{
	struct ums_softc *sc = cdev->sc_priv_ptr;

	DPRINTF(1, "\n");

	/* reset status */

	sc->sc_status.flags = 0;
	sc->sc_status.button = 0;
	sc->sc_status.obutton = 0;
	sc->sc_status.dx = 0;
	sc->sc_status.dy = 0;
	sc->sc_status.dz = 0;
	/* sc->sc_status.dt = 0; */

	return 0;
}

static int32_t
ums_ioctl(struct usb_cdev *cdev, u_long cmd, caddr_t addr, 
	  int32_t fflags, struct thread *td)
{
	struct ums_softc *sc = cdev->sc_priv_ptr;
	mousemode_t mode;
	int error = 0;

	DPRINTF(1, "\n");

	switch(cmd) {
	case MOUSE_GETHWINFO:
		*(mousehw_t *)addr = sc->sc_hw;
		break;

	case MOUSE_GETMODE:
		*(mousemode_t *)addr = sc->sc_mode;
		break;

	case MOUSE_SETMODE:
		mode = *(mousemode_t *)addr;

		if (mode.level == -1) {
		    /* don't change the current setting */
		} else if ((mode.level < 0) || (mode.level > 1)) {
		    error = EINVAL;
		    goto done;
		} else {
		   sc->sc_mode.level = mode.level;
		}

		if (sc->sc_mode.level == 0) {
		    if (sc->sc_buttons > MOUSE_MSC_MAXBUTTON)
		        sc->sc_hw.buttons = MOUSE_MSC_MAXBUTTON;
		    else
		        sc->sc_hw.buttons = sc->sc_buttons;
		    sc->sc_mode.protocol = MOUSE_PROTO_MSC;
		    sc->sc_mode.packetsize = MOUSE_MSC_PACKETSIZE;
		    sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
		    sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_mode.level == 1) {
		    if (sc->sc_buttons > MOUSE_SYS_MAXBUTTON)
		        sc->sc_hw.buttons = MOUSE_SYS_MAXBUTTON;
		    else
		        sc->sc_hw.buttons = sc->sc_buttons;
		    sc->sc_mode.protocol = MOUSE_PROTO_SYSMOUSE;
		    sc->sc_mode.packetsize = MOUSE_SYS_PACKETSIZE;
		    sc->sc_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
		    sc->sc_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		ums_reset_buf(sc);
		break;

	case MOUSE_GETLEVEL:
		*(int *)addr = sc->sc_mode.level;
		break;

	case MOUSE_SETLEVEL:
		if (*(int *)addr < 0 || *(int *)addr > 1) {
		    error = EINVAL;
		    goto done;
		}

		sc->sc_mode.level = *(int *)addr;

		if (sc->sc_mode.level == 0) {
		    if (sc->sc_buttons > MOUSE_MSC_MAXBUTTON)
		        sc->sc_hw.buttons = MOUSE_MSC_MAXBUTTON;
		    else
		        sc->sc_hw.buttons = sc->sc_buttons;
		    sc->sc_mode.protocol = MOUSE_PROTO_MSC;
		    sc->sc_mode.packetsize = MOUSE_MSC_PACKETSIZE;
		    sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
		    sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_mode.level == 1) {
		    if (sc->sc_buttons > MOUSE_SYS_MAXBUTTON)
		        sc->sc_hw.buttons = MOUSE_SYS_MAXBUTTON;
		    else
		        sc->sc_hw.buttons = sc->sc_buttons;
		    sc->sc_mode.protocol = MOUSE_PROTO_SYSMOUSE;
		    sc->sc_mode.packetsize = MOUSE_SYS_PACKETSIZE;
		    sc->sc_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
		    sc->sc_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		ums_reset_buf(sc);
		break;

	case MOUSE_GETSTATUS: {
		mousestatus_t *status = (mousestatus_t *) addr;

		*status = sc->sc_status;
		sc->sc_status.obutton = sc->sc_status.button;
		sc->sc_status.button = 0;
		sc->sc_status.dx = 0;
		sc->sc_status.dy = 0;
		sc->sc_status.dz = 0;
		/* sc->sc_status.dt = 0; */

		if (status->dx || status->dy || status->dz /* || status->dt */) {
		    status->flags |= MOUSE_POSCHANGED;
		}

		if (status->button != status->obutton) {
		    status->flags |= MOUSE_BUTTONSCHANGED;
		}
		break;
	}
	default:
		error = ENOTTY;
	}

 done:
	return error;
}

static devclass_t ums_devclass;

static device_method_t ums_methods[] = {
    DEVMETHOD(device_probe, ums_probe),
    DEVMETHOD(device_attach, ums_attach),
    DEVMETHOD(device_detach, ums_detach),
    { 0, 0 }
};

static driver_t ums_driver = {
  .name    = "ums",
  .methods = ums_methods,
  .size    = sizeof(struct ums_softc),
};

DRIVER_MODULE(ums, uhub, ums_driver, ums_devclass, usbd_driver_load, 0);
MODULE_DEPEND(ums, usb, 1, 1, 1);
