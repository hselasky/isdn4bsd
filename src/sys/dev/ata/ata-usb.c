/*-
 * Copyright (c) 2006 - 2007 Søren Schmidt <sos@FreeBSD.org>
 * All rights reserved.
 *
 * Copyright (c) 2006 Hans Petter Selasky
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification, immediately at the beginning of the file.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/ata/ata-usb.c,v 1.7 2007/06/23 21:52:05 imp Exp $");

#include "opt_ata.h"
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/ata.h>
#include <sys/bio.h>
#include <sys/sema.h>
#include <vm/uma.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>

#include <dev/ata/ata-all.h>
#include <ata_if.h>

#define ATAUSB_BULK_SIZE (1<<17) /* don't set this lower than (1 << 16) */

/* Command Block Wrapper */
struct bbb_cbw {
    u_int8_t	signature[4];
#define		CBWSIGNATURE		0x43425355

    u_int8_t	tag[4];
    u_int8_t	transfer_length[4];
    u_int8_t	flags;
#define		CBWFLAGS_OUT  		0x00
#define		CBWFLAGS_IN  		0x80

    u_int8_t	lun;
    u_int8_t	length;
#define		CBWCDBLENGTH     	16

    u_int8_t	cdb[CBWCDBLENGTH];
} UPACKED;

/* Command Status Wrapper */
struct bbb_csw {
    u_int8_t	signature[4];
#define		CSWSIGNATURE     	0x53425355

    u_int8_t	tag[4];
    u_int8_t	residue[4];
    u_int8_t    status;
#define 	CSWSTATUS_GOOD   	0x0
#define		CSWSTATUS_FAILED 	0x1
#define		CSWSTATUS_PHASE  	0x2
} UPACKED;

/* USB-ATA 'controller' softc */
struct atausb_softc {
    struct bbb_cbw	cbw;
    struct bbb_csw	csw;
    struct mtx		locked_mtx;
    struct __callout	watchdog; 

    struct ata_channel *locked_ch;
    struct ata_channel *restart_ch;
    struct ata_request *ata_request;

#define ATAUSB_T_BBB_RESET1        0
#define ATAUSB_T_BBB_RESET2        1
#define ATAUSB_T_BBB_RESET3        2
#define ATAUSB_T_BBB_COMMAND       3
#define ATAUSB_T_BBB_DATA_READ     4
#define ATAUSB_T_BBB_DATA_RD_CS    5
#define ATAUSB_T_BBB_DATA_WRITE    6
#define ATAUSB_T_BBB_DATA_WR_CS    7
#define ATAUSB_T_BBB_STATUS        8
#define ATAUSB_T_BBB_MAX           9

#define ATAUSB_T_MAX ATAUSB_T_BBB_MAX

    struct usbd_xfer *  xfer[ATAUSB_T_MAX];
    caddr_t		ata_data;
    device_t		dev;

    u_int32_t		timeout;
    u_int32_t		ata_donecount;
    u_int32_t		ata_bytecount;

    u_int8_t		last_xfer_no;
    u_int8_t		reset_count;
    u_int8_t		usb_speed;
    u_int8_t		intr_stalled;
    u_int8_t		maxlun;
    u_int8_t		iface_no;
    u_int8_t		status_try;
};

static const int atausbdebug = 0;

/* prototypes */

static device_probe_t atausb_probe;
static device_attach_t atausb_attach;
static device_detach_t atausb_detach;

static usbd_callback_t atausb_t_bbb_reset1_callback;
static usbd_callback_t atausb_t_bbb_reset2_callback;
static usbd_callback_t atausb_t_bbb_reset3_callback;
static usbd_callback_t atausb_t_bbb_command_callback;
static usbd_callback_t atausb_t_bbb_data_read_callback;
static usbd_callback_t atausb_t_bbb_data_rd_cs_callback;
static usbd_callback_t atausb_t_bbb_data_write_callback;
static usbd_callback_t atausb_t_bbb_data_wr_cs_callback;
static usbd_callback_t atausb_t_bbb_status_callback;
static usbd_callback_t atausb_tr_error;

static void
atausb_watchdog(void *arg);

static void
atausb_transfer_start(struct atausb_softc *sc, u_int8_t xfer_no);

static void
atausb_t_bbb_data_clear_stall_callback(struct usbd_xfer *xfer,
				       u_int8_t next_xfer,
				       u_int8_t stall_xfer);
static int
ata_usbchannel_begin_transaction(struct ata_request *request);

static int
ata_usbchannel_end_transaction(struct ata_request *request);

static device_probe_t ata_usbchannel_probe;
static device_attach_t ata_usbchannel_attach;
static device_detach_t ata_usbchannel_detach;

static ata_setmode_t ata_usbchannel_setmode;
static ata_locking_t ata_usbchannel_locking;

/*
 * USB frontend part
 */

struct usbd_config atausb_config[ATAUSB_T_BBB_MAX] = {

    [ATAUSB_T_BBB_RESET1] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_reset1_callback,
      .timeout   = 5000, /* 5 seconds */
    },

    [ATAUSB_T_BBB_RESET2] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_reset2_callback,
      .timeout   = 5000, /* 5 seconds */
    },

    [ATAUSB_T_BBB_RESET3] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_reset3_callback,
      .timeout   = 5000, /* 5 seconds */
    },

    [ATAUSB_T_BBB_COMMAND] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = sizeof(struct bbb_cbw),
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_command_callback,
      .timeout   = 5000, /* 5 seconds */
    },

    [ATAUSB_T_BBB_DATA_READ] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .bufsize   = ATAUSB_BULK_SIZE,
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &atausb_t_bbb_data_read_callback,
      .timeout   = 0, /* overwritten later */
    },

    [ATAUSB_T_BBB_DATA_RD_CS] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_data_rd_cs_callback,
      .timeout   = 5000, /* 5 seconds */
    },

    [ATAUSB_T_BBB_DATA_WRITE] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_OUT,
      .bufsize   = ATAUSB_BULK_SIZE,
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_data_write_callback,
      .timeout   = 0, /* overwritten later */
    },

    [ATAUSB_T_BBB_DATA_WR_CS] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = UE_DIR_ANY,
      .bufsize   = sizeof(usb_device_request_t),
      .flags     = USBD_USE_DMA,
      .callback  = &atausb_t_bbb_data_wr_cs_callback,
      .timeout   = 5000, /* 5 seconds */
    },

    [ATAUSB_T_BBB_STATUS] = {
      .type      = UE_BULK,
      .endpoint  = UE_ADDR_ANY,
      .direction = UE_DIR_IN,
      .bufsize   = sizeof(struct bbb_csw),
      .flags     = (USBD_USE_DMA|USBD_SHORT_XFER_OK),
      .callback  = &atausb_t_bbb_status_callback,
      .timeout   = 5000, /* ms */
    },
};

static devclass_t atausb_devclass;

static device_method_t atausb_methods[] = {
    DEVMETHOD(device_probe, atausb_probe),
    DEVMETHOD(device_attach, atausb_attach),
    DEVMETHOD(device_detach, atausb_detach),
    { 0, 0 }
};

static driver_t atausb_driver = {
    .name    = "atausb",
    .methods = atausb_methods,
    .size    = sizeof(struct atausb_softc),
};

DRIVER_MODULE(atausb, uhub, atausb_driver, atausb_devclass, 0, 0);
MODULE_DEPEND(atausb, usb, 1, 1, 1);
MODULE_VERSION(atausb, 1);

static int
atausb_probe(device_t dev)
{
    struct usb_attach_arg *uaa = device_get_ivars(dev);
    usb_interface_descriptor_t *id;

    if (uaa->iface == NULL) {
	return UMATCH_NONE;
    }

    id = usbd_get_interface_descriptor(uaa->iface);
    if ((!id) || (id->bInterfaceClass != UICLASS_MASS)) {
	return UMATCH_NONE;
    }

    switch (id->bInterfaceSubClass) {
    case UISUBCLASS_QIC157:
    case UISUBCLASS_RBC:
    case UISUBCLASS_SCSI:
    case UISUBCLASS_SFF8020I:
    case UISUBCLASS_SFF8070I:
    case UISUBCLASS_UFI:
	switch (id->bInterfaceProtocol) {
	case UIPROTO_MASS_CBI:
	case UIPROTO_MASS_CBI_I:
	case UIPROTO_MASS_BBB:
	case UIPROTO_MASS_BBB_OLD:
	    return UMATCH_IFACECLASS_IFACESUBCLASS_IFACEPROTO;
	default:
	    return UMATCH_IFACECLASS_IFACESUBCLASS;
	}
	break;
    default:
	return UMATCH_IFACECLASS;
    }
}

static int
atausb_attach(device_t dev)
{
    struct atausb_softc *sc = device_get_softc(dev);
    struct usb_attach_arg *uaa = device_get_ivars(dev);
    usb_interface_descriptor_t *id;
    const char *proto, *subclass;
    usb_device_request_t request;
    u_int16_t i;
    u_int8_t maxlun;
    u_int8_t has_intr;
    int err;

    if (sc == NULL) {
        return ENOMEM;
    }

    usbd_set_desc(dev, uaa->device);

    sc->dev = dev;
    sc->maxlun = 0;
    sc->locked_ch = NULL;
    sc->restart_ch = NULL;
    sc->usb_speed = usbd_get_speed(uaa->device);
    mtx_init(&(sc->locked_mtx), "ATAUSB lock", NULL, (MTX_DEF|MTX_RECURSE)); 

    __callout_init_mtx(&(sc->watchdog),
		       &(sc->locked_mtx), CALLOUT_RETURNUNLOCKED);

    id = usbd_get_interface_descriptor(uaa->iface);
    switch (id->bInterfaceProtocol) {
    case UIPROTO_MASS_BBB:
    case UIPROTO_MASS_BBB_OLD:
	    proto = "Bulk-Only";
	    break;
    case UIPROTO_MASS_CBI:
	    proto = "CBI";
	    break;
    case UIPROTO_MASS_CBI_I:
	    proto = "CBI with CCI";
	    break;
    default:
	    proto = "Unknown";
    }

    switch (id->bInterfaceSubClass) {
    case UISUBCLASS_RBC:
	    subclass = "RBC";
	    break;
    case UISUBCLASS_QIC157:
    case UISUBCLASS_SFF8020I:
    case UISUBCLASS_SFF8070I:
	    subclass = "ATAPI";
	    break;
    case UISUBCLASS_SCSI:
	    subclass = "SCSI";
	    break;
    case UISUBCLASS_UFI:
	    subclass = "UFI";
	    break;
    default:
	    subclass = "Unknown";
    }

    has_intr = (id->bInterfaceProtocol == UIPROTO_MASS_CBI_I);
    sc->iface_no = id->bInterfaceNumber;

    device_printf(dev, "using %s over %s\n", subclass, proto);
    if (strcmp(proto, "Bulk-Only") ||
	(strcmp(subclass, "ATAPI") && strcmp(subclass, "SCSI"))) {
        goto detach;
    }

    err = usbd_transfer_setup(uaa->device, uaa->iface_index, sc->xfer, 
			      atausb_config, 
			      ATAUSB_T_BBB_MAX, sc, 
			      &(sc->locked_mtx));
    /* skip reset first time */
    sc->last_xfer_no = ATAUSB_T_BBB_COMMAND;

    if (err) {
        device_printf(sc->dev, "could not setup required "
		      "transfers, %s\n", usbd_errstr(err));
	goto detach;
    }

    /* get number of devices so we can add matching channels */
    request.bmRequestType = UT_READ_CLASS_INTERFACE;
    request.bRequest = 0xfe; /* GET_MAX_LUN; */
    USETW(request.wValue, 0);
    USETW(request.wIndex, sc->iface_no);
    USETW(request.wLength, sizeof(maxlun));
    err = usbd_do_request(uaa->device, &request, &maxlun);

    if (err) {
        if (bootverbose) {
	    device_printf(sc->dev, "get maxlun not supported %s\n",
			  usbd_errstr(err));
	}
    } else {
	sc->maxlun = maxlun;
	if (bootverbose) {
	    device_printf(sc->dev, "maxlun=%d\n", sc->maxlun);
	}
    }

    /* ata channels are children to this USB control device */
    for (i = 0; i <= sc->maxlun; i++) {
	if (!device_add_child(sc->dev, "ata",
			      devclass_find_free_unit(ata_devclass, 2))) {
	    device_printf(sc->dev, "failed to attach ata child device\n");
	    goto detach;
	}
    }
    bus_generic_attach(sc->dev);

    /* start the watchdog */

    mtx_lock(&(sc->locked_mtx));

    atausb_watchdog(sc);

    return 0;

 detach:
    atausb_detach(dev);
    return ENXIO;
}

static int
atausb_detach(device_t dev)
{
    struct atausb_softc *sc = device_get_softc(dev);
    device_t *children;
    int nchildren, i;

    mtx_lock(&(sc->locked_mtx));

    /* stop watchdog */
    __callout_stop(&(sc->watchdog));

    /* signal that device is going away */
    sc->last_xfer_no = ATAUSB_T_MAX;

    /* stop all transfers, if any */
    for (i = 0; i < ATAUSB_T_MAX; i++) {
        if (sc->xfer[i]) {
	    usbd_transfer_stop(sc->xfer[i]);
	}
    }

    mtx_unlock(&(sc->locked_mtx));

    /* detach & delete all children, if any */

    if (!device_get_children(dev, &children, &nchildren)) {
        for (i = 0; i < nchildren; i++) {
            device_delete_child(dev, children[i]);
	}
        free(children, M_TEMP);
    }

    usbd_transfer_unsetup(sc->xfer, ATAUSB_T_MAX);

    __callout_drain(&(sc->watchdog));

    mtx_destroy(&sc->locked_mtx);
    return 0;
}

static void
atausb_watchdog(void *arg)
{
    struct atausb_softc *sc = arg;

    mtx_assert(&(sc->locked_mtx), MA_OWNED);

#if 0
    __callout_reset(&(sc->watchdog), 
		    hz, &atausb_watchdog, sc);
#endif

    mtx_unlock(&(sc->locked_mtx));
    return;
}

static void
atausb_transfer_start(struct atausb_softc *sc, u_int8_t xfer_no)
{
    if (atausbdebug) {
	device_printf(sc->dev, "BBB transfer %d\n", xfer_no);
    }
    sc->last_xfer_no = xfer_no;
    usbd_transfer_start(sc->xfer[xfer_no]);
    return;
}

static void
atausb_t_bbb_reset1_callback(struct usbd_xfer *xfer)
{
    struct atausb_softc *sc = xfer->priv_sc;
    usb_device_request_t req;

    USBD_CHECK_STATUS(xfer);

 tr_error:
    atausb_tr_error(xfer);
    return;

 tr_transferred:
    atausb_transfer_start(sc, ATAUSB_T_BBB_RESET2);
    return;

 tr_setup:

    sc->reset_count ++;

    req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
    req.bRequest = 0xff; /* bulk-only reset */
    USETW(req.wValue, 0);
    req.wIndex[0] = sc->iface_no;
    req.wIndex[1] = 0;
    USETW(req.wLength, 0);

    usbd_copy_in(&(xfer->buf_data), 0, &req, sizeof(req));

    usbd_start_hardware(xfer);
    return;
}

static void
atausb_t_bbb_reset2_callback(struct usbd_xfer *xfer)
{
    atausb_t_bbb_data_clear_stall_callback(xfer, ATAUSB_T_BBB_RESET3,
					   ATAUSB_T_BBB_DATA_READ);
    return;
}

static void
atausb_t_bbb_reset3_callback(struct usbd_xfer *xfer)
{
    atausb_t_bbb_data_clear_stall_callback(xfer, ATAUSB_T_BBB_COMMAND,
					   ATAUSB_T_BBB_DATA_WRITE);
    return;
}

static void
atausb_t_bbb_data_clear_stall_callback(struct usbd_xfer *xfer, 
				       u_int8_t next_xfer,
				       u_int8_t stall_xfer)
{
    struct atausb_softc *sc = xfer->priv_sc;

    USBD_CHECK_STATUS(xfer);

 tr_error:
    atausb_tr_error(xfer);
    return;

 tr_transferred:
    atausb_transfer_start(sc, next_xfer);
    return;

 tr_setup:
    if (usbd_clear_stall_callback(xfer, sc->xfer[stall_xfer])) {
        goto tr_transferred;
    }
    return;
}

static void
atausb_t_bbb_command_callback(struct usbd_xfer *xfer)
{
    struct atausb_softc *sc = xfer->priv_sc;
    struct ata_request *request = sc->ata_request;
    struct ata_channel *ch;
    u_int32_t tag;

    USBD_CHECK_STATUS(xfer);

 tr_error:
    atausb_tr_error(xfer);
    return;

 tr_transferred:
    atausb_transfer_start
      (sc, ((request->flags & ATA_R_READ) ? ATAUSB_T_BBB_DATA_READ :
	    (request->flags & ATA_R_WRITE) ? ATAUSB_T_BBB_DATA_WRITE :
	    ATAUSB_T_BBB_STATUS));
    return;

 tr_setup:

    sc->reset_count = 0;
    sc->status_try = 0;

    if (request) {
        ch = device_get_softc(request->parent);

	sc->timeout = (request->timeout * 1000) + 5000;

	tag = UGETDW(sc->cbw.tag) + 1;

	USETDW(sc->cbw.signature, CBWSIGNATURE);
	USETDW(sc->cbw.tag, tag);
	USETDW(sc->cbw.transfer_length, request->bytecount);
	sc->cbw.flags = (request->flags & ATA_R_READ) ? CBWFLAGS_IN : CBWFLAGS_OUT;
	sc->cbw.lun = ch->unit;
	sc->cbw.length = 16;
	bzero(sc->cbw.cdb, 16);
	bcopy(request->u.atapi.ccb, sc->cbw.cdb, 12); /* XXX SOS */

	usbd_copy_in(&(xfer->buf_data), 0, &(sc->cbw), sizeof(sc->cbw));

	usbd_start_hardware(xfer);
    }
    return;
}

static void
atausb_t_bbb_data_read_callback(struct usbd_xfer *xfer)
{
    struct atausb_softc *sc = xfer->priv_sc; 
    u_int32_t max_bulk = (ATAUSB_BULK_SIZE - 
			  (ATAUSB_BULK_SIZE % xfer->max_packet_size));
    USBD_CHECK_STATUS(xfer);

 tr_error:
    if (xfer->error == USBD_CANCELLED) {
        atausb_tr_error(xfer);
    } else {
        atausb_transfer_start(sc, ATAUSB_T_BBB_DATA_RD_CS);
    }
    return;

 tr_transferred:

    usbd_copy_out(&(xfer->buf_data), 0, 
		  sc->ata_data, xfer->actlen);

    sc->ata_bytecount -= xfer->actlen;
    sc->ata_data += xfer->actlen;
    sc->ata_donecount += xfer->actlen;

    if (xfer->actlen < xfer->length) {
        /* short transfer */
        sc->ata_bytecount = 0;
    }

 tr_setup:

    if (atausbdebug > 1) {
        device_printf(sc->dev, "%s: max_bulk=%d, ata_bytecount=%d\n",
		      __FUNCTION__, max_bulk, sc->ata_bytecount);
    }

    if (sc->ata_bytecount == 0) {
        atausb_transfer_start(sc, ATAUSB_T_BBB_STATUS);
	return;
    }

    if (max_bulk > sc->ata_bytecount) {
        max_bulk = sc->ata_bytecount;
    }

    xfer->timeout = sc->timeout;
    xfer->length = max_bulk;

    usbd_start_hardware(xfer);
    return;
}

static void
atausb_t_bbb_data_rd_cs_callback(struct usbd_xfer *xfer)
{
    atausb_t_bbb_data_clear_stall_callback(xfer, ATAUSB_T_BBB_STATUS,
					   ATAUSB_T_BBB_DATA_READ);
    return;
}

static void
atausb_t_bbb_data_write_callback(struct usbd_xfer *xfer)
{
    struct atausb_softc *sc = xfer->priv_sc; 
    u_int32_t max_bulk = (ATAUSB_BULK_SIZE - 
			  (ATAUSB_BULK_SIZE % xfer->max_packet_size));
    USBD_CHECK_STATUS(xfer);

 tr_error:
    if (xfer->error == USBD_CANCELLED) {
        atausb_tr_error(xfer);
    } else {
        atausb_transfer_start(sc, ATAUSB_T_BBB_DATA_WR_CS);
    }
    return;

 tr_transferred:

    sc->ata_bytecount -= xfer->actlen;
    sc->ata_data += xfer->actlen;
    sc->ata_donecount += xfer->actlen;

 tr_setup:

    if (atausbdebug > 1) {
        device_printf(sc->dev, "%s: max_bulk=%d, ata_bytecount=%d\n",
		      __FUNCTION__, max_bulk, sc->ata_bytecount);
    }

    if (sc->ata_bytecount == 0) {
        atausb_transfer_start(sc, ATAUSB_T_BBB_STATUS);
	return;
    }

    if (max_bulk > sc->ata_bytecount) {
        max_bulk = sc->ata_bytecount;
    }

    xfer->timeout = sc->timeout;
    xfer->length = max_bulk;

    usbd_copy_in(&(xfer->buf_data), 0, 
		 sc->ata_data, max_bulk);

    usbd_start_hardware(xfer);
    return;
}

static void
atausb_t_bbb_data_wr_cs_callback(struct usbd_xfer *xfer)
{
    atausb_t_bbb_data_clear_stall_callback(xfer, ATAUSB_T_BBB_STATUS,
					   ATAUSB_T_BBB_DATA_WRITE);
    return;
}

static void
atausb_t_bbb_status_callback(struct usbd_xfer *xfer)
{
    struct atausb_softc *sc = xfer->priv_sc; 
    struct ata_request *request = sc->ata_request;
    u_int32_t residue;

    USBD_CHECK_STATUS(xfer);

 tr_error:
    if ((xfer->error == USBD_CANCELLED) ||
	(sc->status_try)) {
        atausb_tr_error(xfer);
    } else {
        sc->status_try = 1;
	atausb_transfer_start(sc, ATAUSB_T_BBB_DATA_RD_CS);
    }
    return;

 tr_transferred:

    if (xfer->actlen < sizeof(sc->csw)) {
        bzero(&(sc->csw), sizeof(sc->csw));
    }

    usbd_copy_out(&(xfer->buf_data), 0, &(sc->csw), xfer->actlen);

    if (request->flags & (ATA_R_READ | ATA_R_WRITE)) {
        request->donecount = sc->ata_donecount;
    }

    residue = UGETDW(sc->csw.residue);

    if (!residue) {
        residue = (request->bytecount - request->donecount);
    }

    if (residue > request->bytecount) {
        if (atausbdebug) {
	    device_printf(sc->dev, "truncating residue from %d "
			  "to %d bytes\n", residue, 
			  request->bytecount);
	}
        residue = request->bytecount;
    }

    /* check CSW and handle eventual error */
    if (UGETDW(sc->csw.signature) != CSWSIGNATURE) {
        if (atausbdebug) {
	    device_printf(sc->dev, "bad CSW signature 0x%08x != 0x%08x\n",
			  UGETDW(sc->csw.signature), CSWSIGNATURE);
	}
	goto tr_error;
    }
    else if (UGETDW(sc->csw.tag) != UGETDW(sc->cbw.tag)) {
        if (atausbdebug) {
	    device_printf(sc->dev, "bad CSW tag %d != %d\n",
			  UGETDW(sc->csw.tag), UGETDW(sc->cbw.tag));
	}
	goto tr_error;
    }
    else if (sc->csw.status > CSWSTATUS_PHASE) {
        if (atausbdebug) {
	    device_printf(sc->dev, "bad CSW status %d > %d\n",
			  sc->csw.status, CSWSTATUS_PHASE);
	}
	goto tr_error;
    }
    else if (sc->csw.status == CSWSTATUS_PHASE) {
        if (atausbdebug) {
	    device_printf(sc->dev, "phase error residue = %d\n", residue);
	}
	goto tr_error;
    }
    else if (request->donecount > request->bytecount) {
        if (atausbdebug) {
	    device_printf(sc->dev, "buffer overrun %d > %d\n",
			  request->donecount, request->bytecount);
	}
	goto tr_error;
    }
    else if (sc->csw.status == CSWSTATUS_FAILED) {
        if (atausbdebug) {
	    device_printf(sc->dev, "CSWSTATUS_FAILED\n");
	}
	request->error = ATA_E_ATAPI_SENSE_MASK;
    }

    sc->last_xfer_no = ATAUSB_T_BBB_COMMAND;

    sc->ata_request = NULL;

    if (atausbdebug > 1) {
        device_printf(sc->dev, "%s: depreciated unlock!\n",
		      __FUNCTION__);
    }

    mtx_unlock(xfer->priv_mtx); /* XXX depreciated! */

    ata_interrupt(device_get_softc(request->parent));

    mtx_lock(xfer->priv_mtx);
    return;

 tr_setup:
    usbd_start_hardware(xfer);
    return;
}

static void
atausb_tr_error(struct usbd_xfer *xfer)
{
    struct atausb_softc *sc = xfer->priv_sc;
    struct ata_request *request = sc->ata_request;

    if (xfer->error != USBD_CANCELLED) {

        if (atausbdebug) {
	    device_printf(sc->dev, "transfer failed, %s, in state %d "
			  "-> BULK reset\n", usbd_errstr(xfer->error), 
			  sc->last_xfer_no);
	}

	if (sc->reset_count < 16) {

	    /* start reset before any callback */

	    atausb_transfer_start(sc, ATAUSB_T_BBB_RESET1);
	} else {

	    /* suspend reset until next command */

	    sc->last_xfer_no = ATAUSB_T_BBB_RESET1;
	    sc->reset_count = 0;

	    device_printf(sc->dev, "timeout: giving up reset!\n");
	}
    }

    if (request) {
        request->result = (xfer->error == USBD_CANCELLED) ? ENXIO : EIO;
	sc->ata_request = NULL;

	if (atausbdebug > 1) {
	    device_printf(sc->dev, "%s: depreciated unlock!\n",
			  __FUNCTION__);
	}

	mtx_unlock(xfer->priv_mtx); /* XXX depreciated! */

	ata_interrupt(device_get_softc(request->parent));

	mtx_lock(xfer->priv_mtx);
    }
    return;
}

/*
 * ATA backend part
 */
struct atapi_inquiry {
    u_int8_t    device_type;
    u_int8_t    device_modifier;
    u_int8_t    version;
    u_int8_t    response_format;
    u_int8_t    length;
    u_int8_t    reserved[2];
    u_int8_t    flags;
    u_int8_t    vendor[8];
    u_int8_t    product[16];
    u_int8_t    revision[4];
 /* u_int8_t    crap[60]; */
} UPACKED;

static int
ata_usbchannel_begin_transaction(struct ata_request *request)
{
    struct atausb_softc *sc = 
    	device_get_softc(device_get_parent(request->parent));
    int error;

    if (atausbdebug > 1) {
	device_printf(request->dev, "begin_transaction %s\n",
		      ata_cmd2str(request));
    }

    mtx_lock(&sc->locked_mtx);

    /* sanity, just in case */
    if (sc->ata_request) {
        device_printf(request->dev, "begin is busy, "
		      "state = %d\n", sc->last_xfer_no);
	request->result = EBUSY;
	error = ATA_OP_FINISHED;
	goto done;
    }

    /* XXX SOS convert the request into 
     * the format used, only BBB for now
     */

    /* ATA/ATAPI IDENTIFY needs special treatment */
    if (!(request->flags & ATA_R_ATAPI)) {
	if (request->u.ata.command != ATA_ATAPI_IDENTIFY) {
	    device_printf(request->dev, "%s unsupported\n", 
			  ata_cmd2str(request));
	    request->result = EIO;
	    error = ATA_OP_FINISHED;
	    goto done;
	}
	request->flags |= ATA_R_ATAPI;
	bzero(request->u.atapi.ccb, 16);
	request->u.atapi.ccb[0] = ATAPI_INQUIRY;
	request->u.atapi.ccb[4] =  255; /* sizeof(struct atapi_inquiry); */
	request->data += 256;	/* arbitrary offset into ata_param */
	request->bytecount = 255; /* sizeof(struct atapi_inquiry); */
    }

    if (sc->last_xfer_no < ATAUSB_T_MAX) {

        sc->ata_request = request;
	sc->ata_bytecount = request->bytecount;
	sc->ata_data = request->data;
	sc->ata_donecount = 0;

        usbd_transfer_start(sc->xfer[sc->last_xfer_no]);
	error = ATA_OP_CONTINUES;
    } else {
        request->result = EIO;
	error = ATA_OP_FINISHED;
    }

 done:
    mtx_unlock(&sc->locked_mtx);
    return error;
}

static int
ata_usbchannel_end_transaction(struct ata_request *request)
{
    if (atausbdebug > 1) {
	device_printf(request->dev, "end_transaction %s\n",
		      ata_cmd2str(request));
    }
    
    /* XXX SOS convert the request from the format used, only BBB for now */

    /* ATA/ATAPI IDENTIFY needs special treatment */
    if ((request->flags & ATA_R_ATAPI) &&
	(request->u.atapi.ccb[0] == ATAPI_INQUIRY)) {
	struct ata_device *atadev = device_get_softc(request->dev);
	struct atapi_inquiry *inquiry = (struct atapi_inquiry *)request->data;
	u_int16_t *ptr;

	/* convert inquiry data into simple ata_param like format */
	atadev->param.config = ATA_PROTO_ATAPI | ATA_PROTO_ATAPI_12;
	atadev->param.config |= (inquiry->device_type & 0x1f) << 8;
	bzero(atadev->param.model, sizeof(atadev->param.model));
	strncpy(atadev->param.model, inquiry->vendor, 8);
	strcpy(atadev->param.model, "  ");
	strncpy(atadev->param.model, inquiry->product, 16);
	ptr = (u_int16_t*)(atadev->param.model + sizeof(atadev->param.model));
	while (--ptr >= (u_int16_t*)atadev->param.model) {
	    *ptr = ntohs(*ptr);
	}
	strncpy(atadev->param.revision, inquiry->revision, 4);
	ptr=(u_int16_t*)(atadev->param.revision+sizeof(atadev->param.revision));
	while (--ptr >= (u_int16_t*)atadev->param.revision) {
	    *ptr = ntohs(*ptr);
	}
	request->result = 0;
    }
    return ATA_OP_FINISHED;
}

static int
ata_usbchannel_probe(device_t dev)
{
    struct ata_channel *ch = device_get_softc(dev);
    device_t *children;
    int count, i;
    char buffer[32];

    /* take care of green memory */
    bzero(ch, sizeof(struct ata_channel));

    /* find channel number on this controller */
    if (!device_get_children(device_get_parent(dev), &children, &count)) {
        for (i = 0; i < count; i++) {
	    if (children[i] == dev)
	        ch->unit = i;
	}
	free(children, M_TEMP);
    }

    snprintf(buffer, sizeof(buffer), "USB lun %d", ch->unit);
    device_set_desc_copy(dev, buffer);

    return 0;
}

static int
ata_usbchannel_attach(device_t dev)
{
    struct ata_channel *ch = device_get_softc(dev);

    /* initialize the softc basics */
    ch->dev = dev;
    ch->state = ATA_IDLE;
    ch->hw.begin_transaction = ata_usbchannel_begin_transaction;
    ch->hw.end_transaction = ata_usbchannel_end_transaction;
    ch->hw.status = NULL;
    ch->hw.command = NULL;
    bzero(&ch->state_mtx, sizeof(struct mtx));
    mtx_init(&ch->state_mtx, "ATA state lock", NULL, MTX_DEF);
    bzero(&ch->queue_mtx, sizeof(struct mtx));
    mtx_init(&ch->queue_mtx, "ATA queue lock", NULL, MTX_DEF);
    TAILQ_INIT(&ch->ata_queue);

    /* XXX SOS reset the controller HW, the channel and device(s) */
    /* ATA_RESET(dev); */

    /* probe and attach device on this channel */
    ch->devices = ATA_ATAPI_MASTER;
    if (!ata_delayed_attach) {
        ata_identify(dev);
    }
    return 0;
}

static int
ata_usbchannel_detach(device_t dev)
{
    struct ata_channel *ch = device_get_softc(dev);
    device_t *children;
    int nchildren, i;

    /* detach & delete all children */
    if (!device_get_children(dev, &children, &nchildren)) {
        for (i = 0; i < nchildren; i++)
            if (children[i])
                device_delete_child(dev, children[i]);
        free(children, M_TEMP);
    }
    mtx_destroy(&ch->state_mtx);
    mtx_destroy(&ch->queue_mtx);
    return 0;
}

static void
ata_usbchannel_setmode(device_t parent, device_t dev)
{
    struct atausb_softc *sc = device_get_softc(GRANDPARENT(dev));
    struct ata_device *atadev = device_get_softc(dev);

    if (sc->usb_speed == USB_SPEED_HIGH)
	atadev->mode = ATA_USB2;
    else
	atadev->mode = ATA_USB1;
    return;
}

static int
ata_usbchannel_locking(device_t dev, int flags)
{
    struct atausb_softc *sc = device_get_softc(device_get_parent(dev));
    struct ata_channel *ch = device_get_softc(dev);
    int res = -1;

    mtx_lock(&sc->locked_mtx);
    switch (flags) {
    case ATA_LF_LOCK:
	if (sc->locked_ch == NULL)
	    sc->locked_ch = ch;
	if (sc->locked_ch != ch)
	    sc->restart_ch = ch;
	break;

    case ATA_LF_UNLOCK:
	if (sc->locked_ch == ch) {
	    sc->locked_ch = NULL;
	    if (sc->restart_ch) {
		ch = sc->restart_ch;
		sc->restart_ch = NULL;
		mtx_unlock(&sc->locked_mtx);
		ata_start(ch->dev);
		return res;
	    }
	}
	break;

    case ATA_LF_WHICH:
	break;
    }
    if (sc->locked_ch) {
	res = sc->locked_ch->unit;
    }
    mtx_unlock(&sc->locked_mtx);
    return res;
}

static device_method_t ata_usbchannel_methods[] = {
    /* device interface */
    DEVMETHOD(device_probe,         ata_usbchannel_probe),
    DEVMETHOD(device_attach,        ata_usbchannel_attach),
    DEVMETHOD(device_detach,        ata_usbchannel_detach),

    /* ATA methods */
    DEVMETHOD(ata_setmode,          ata_usbchannel_setmode),
    DEVMETHOD(ata_locking,          ata_usbchannel_locking),
 /* DEVMETHOD(ata_reset,            ata_usbchannel_reset), */

    { 0, 0 }
};

static driver_t ata_usbchannel_driver = {
    "ata",
    ata_usbchannel_methods,
    sizeof(struct ata_channel),
};

DRIVER_MODULE(ata, atausb, ata_usbchannel_driver, ata_devclass, 0, 0);
MODULE_DEPEND(atausb, ata, 1, 1, 1);
