#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/at9100_dci.c $");

/*-
 * Copyright (c) 2007 Hans Petter Selasky <hselasky@freebsd.org>
 * All rights reserved.
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
 * This file contains the driver for the AT91 series USB Device
 * Controller
 */

/*
 * Thanks to "David Brownell" for helping out regarding the hardware
 * endpoint profiles.
 */

/*
 * NOTE: The "fifo_bank" is not reset in hardware when the endpoint is
 * reset !
 *
 * NOTE: When the chip detects BUS-reset it will also reset the
 * endpoints, Function-address and more.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/malloc.h>

#define	usbd_config_td_cc at9100_dci_config_copy
#define	usbd_config_td_softc at9100_dci_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/at9100_dci.h>

#define	AT9100_DCI_BUS2SC(bus) \
   ((struct at9100_dci_softc *)(((uint8_t *)(bus)) - \
   POINTER_TO_UNSIGNED(&(((struct at9100_dci_softc *)0)->sc_bus))))

#ifdef USB_DEBUG
#define	DPRINTFN(n,fmt,...) do {			\
  if (at9100_dcidebug > (n)) {				\
    printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__);	\
  }							\
} while (0)

static int at9100_dcidebug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, at9100_dci, CTLFLAG_RW, 0, "USB at9100_dci");
SYSCTL_INT(_hw_usb_at9100_dci, OID_AUTO, debug, CTLFLAG_RW,
    &at9100_dcidebug, 0, "at9100_dci debug level");
#else
#define	DPRINTFN(...) do { } while (0)
#endif

#define	AT9100_DCI_INTR_ENDPT 1

/* prototypes */

struct usbd_bus_methods at9100_dci_bus_methods;
struct usbd_pipe_methods at9100_dci_device_bulk_methods;
struct usbd_pipe_methods at9100_dci_device_ctrl_methods;
struct usbd_pipe_methods at9100_dci_device_intr_methods;
struct usbd_pipe_methods at9100_dci_device_isoc_fs_methods;
struct usbd_pipe_methods at9100_dci_root_ctrl_methods;
struct usbd_pipe_methods at9100_dci_root_intr_methods;

static at9100_dci_cmd_t at9100_dci_setup_rx;
static at9100_dci_cmd_t at9100_dci_data_rx;
static at9100_dci_cmd_t at9100_dci_data_tx;
static at9100_dci_cmd_t at9100_dci_data_tx_sync;
static void at9100_dci_device_done(struct usbd_xfer *xfer, usbd_status_t error);
static void at9100_dci_do_poll(struct usbd_bus *bus);
static void at9100_dci_root_ctrl_poll(struct at9100_dci_softc *sc);
static void at9100_dci_standard_done(struct usbd_xfer *xfer);

static usbd_std_root_transfer_func_t at9100_dci_root_intr_done;
static usbd_std_root_transfer_func_t at9100_dci_root_ctrl_done;
static usbd_config_td_command_t at9100_dci_root_ctrl_task;

/*
 * NOTE: Some of the bits in the CSR register have inverse meaning so
 * we need a helper macro when acknowledging events:
 */
#define	AT91_CSR_ACK(csr, what) do {		\
  (csr) &= ~((AT91_UDP_CSR_FORCESTALL|		\
	      AT91_UDP_CSR_TXPKTRDY|		\
	      AT91_UDP_CSR_RXBYTECNT) ^ (what));\
  (csr) |= ((AT91_UDP_CSR_RX_DATA_BK0|		\
	     AT91_UDP_CSR_RX_DATA_BK1|		\
	     AT91_UDP_CSR_TXCOMP|		\
	     AT91_UDP_CSR_RXSETUP|		\
	     AT91_UDP_CSR_STALLSENT) ^ (what));	\
} while (0)

/*
 * Here is a list of what the chip supports.
 * Probably it supports more than listed here!
 */
static const struct usbd_hw_ep_profile
	at9100_dci_ep_profile[AT91_UDP_EP_MAX] = {

	[0] = {
		.max_frame_size = 8,
		.is_simplex = 1,
		.support_control = 1,
	},
	[1] = {
		.max_frame_size = 64,
		.is_simplex = 1,
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_isochronous = 1,
		.support_in = 1,
		.support_out = 1,
	},
	[2] = {
		.max_frame_size = 64,
		.is_simplex = 1,
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_isochronous = 1,
		.support_in = 1,
		.support_out = 1,
	},
	[3] = {
		/* can also do BULK */
		.max_frame_size = 8,
		.is_simplex = 1,
		.support_interrupt = 1,
		.support_in = 1,
		.support_out = 1,
	},
	[4] = {
		.max_frame_size = 256,
		.is_simplex = 1,
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_isochronous = 1,
		.support_in = 1,
		.support_out = 1,
	},
	[5] = {
		.max_frame_size = 256,
		.is_simplex = 1,
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_isochronous = 1,
		.support_in = 1,
		.support_out = 1,
	},
};

static void
at9100_dci_get_hw_ep_profile(struct usbd_device *udev,
    const struct usbd_hw_ep_profile **ppf, uint8_t ep_addr)
{
	if (ep_addr < AT91_UDP_EP_MAX) {
		*ppf = (at9100_dci_ep_profile + ep_addr);
	} else {
		*ppf = NULL;
	}
	return;
}

static void
at9100_dci_clocks_on(struct at9100_dci_softc *sc)
{
	if (sc->sc_flags.clocks_off &&
	    sc->sc_flags.port_powered) {

		DPRINTFN(4, "\n");

		if (sc->sc_clocks_on) {
			(sc->sc_clocks_on) (sc->sc_clocks_arg);
		}
		sc->sc_flags.clocks_off = 0;

		/* enable Transceiver */
		AT91_UDP_WRITE_4(sc, AT91_UDP_TXVC, 0);
	}
	return;
}

static void
at9100_dci_clocks_off(struct at9100_dci_softc *sc)
{
	if (!sc->sc_flags.clocks_off) {

		DPRINTFN(4, "\n");

		/* disable Transceiver */
		AT91_UDP_WRITE_4(sc, AT91_UDP_TXVC, AT91_UDP_TXVC_DIS);

		if (sc->sc_clocks_off) {
			(sc->sc_clocks_off) (sc->sc_clocks_arg);
		}
		sc->sc_flags.clocks_off = 1;
	}
	return;
}

static void
at9100_dci_pull_up(struct at9100_dci_softc *sc)
{
	/* pullup D+, if possible */

	if (!sc->sc_flags.d_pulled_up &&
	    sc->sc_flags.port_powered) {
		sc->sc_flags.d_pulled_up = 1;
		(sc->sc_pull_up) (sc->sc_pull_arg);
	}
	return;
}

static void
at9100_dci_pull_down(struct at9100_dci_softc *sc)
{
	/* pulldown D+, if possible */

	if (sc->sc_flags.d_pulled_up) {
		sc->sc_flags.d_pulled_up = 0;
		(sc->sc_pull_down) (sc->sc_pull_arg);
	}
	return;
}

static void
at9100_dci_wakeup_peer(struct at9100_dci_softc *sc)
{
	uint32_t temp;

	if (!(sc->sc_flags.status_suspend)) {
		return;
	}
	temp = AT91_UDP_READ_4(sc, AT91_UDP_GSTATE);

	if (!(temp & AT91_UDP_GSTATE_ESR)) {
		return;
	}
	AT91_UDP_WRITE_4(sc, AT91_UDP_GSTATE, temp);

	return;
}

static void
at9100_dci_rem_wakeup_set(struct usbd_device *udev, uint8_t is_on)
{
	struct at9100_dci_softc *sc;
	uint32_t temp;

	DPRINTFN(4, "is_on=%u\n", is_on);

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	sc = AT9100_DCI_BUS2SC(udev->bus);

	temp = AT91_UDP_READ_4(sc, AT91_UDP_GSTATE);

	if (is_on) {
		temp |= AT91_UDP_GSTATE_ESR;
	} else {
		temp &= ~AT91_UDP_GSTATE_ESR;
	}

	AT91_UDP_WRITE_4(sc, AT91_UDP_GSTATE, temp);

	return;
}

static void
at9100_dci_set_address(struct at9100_dci_softc *sc, uint8_t addr)
{
	DPRINTFN(4, "addr=%d\n", addr);

	AT91_UDP_WRITE_4(sc, AT91_UDP_FADDR, addr |
	    AT91_UDP_FADDR_EN);

	return;
}

static uint8_t
at9100_dci_setup_rx(struct at9100_dci_td *td)
{
	struct at9100_dci_softc *sc;
	usb_device_request_t req;
	uint32_t csr;
	uint32_t temp;
	uint16_t count;

	/* read out FIFO status */
	csr = bus_space_read_4(td->io_tag, td->io_hdl,
	    td->status_reg);

	DPRINTFN(4, "csr=0x%08x rem=%u\n", csr, td->remainder);

	temp = csr;
	temp &= (AT91_UDP_CSR_RX_DATA_BK0 |
	    AT91_UDP_CSR_RX_DATA_BK1 |
	    AT91_UDP_CSR_STALLSENT |
	    AT91_UDP_CSR_RXSETUP |
	    AT91_UDP_CSR_TXCOMP);

	if (!(csr & AT91_UDP_CSR_RXSETUP)) {
		/* abort any ongoing transfer */
		if (!td->did_stall) {
			DPRINTFN(4, "stalling\n");
			temp |= AT91_UDP_CSR_FORCESTALL;
			td->did_stall = 1;
		}
		goto not_complete;
	}
	/* get the packet byte count */
	count = (csr & AT91_UDP_CSR_RXBYTECNT) >> 16;

	/* verify data length */
	if (count != td->remainder) {
		DPRINTFN(-1, "Invalid SETUP packet "
		    "length, %d bytes\n", count);
		goto not_complete;
	}
	if (count != sizeof(req)) {
		DPRINTFN(-1, "Unsupported SETUP packet "
		    "length, %d bytes\n", count);
		goto not_complete;
	}
	/* receive data */
	bus_space_read_multi_1(td->io_tag, td->io_hdl,
	    td->fifo_reg, (void *)&req, sizeof(req));

	/* copy data into real buffer */
	usbd_copy_in(td->pc, 0, &req, sizeof(req));

	td->offset = sizeof(req);
	td->remainder = 0;

	/* get pointer to softc */
	sc = td->pc->xfer->usb_sc;

	/* sneak peek the set address */
	if ((req.bmRequestType == UT_WRITE_DEVICE) &&
	    (req.bRequest == UR_SET_ADDRESS)) {
		sc->sc_dv_addr = req.wValue[0] & 0x7F;
	} else {
		sc->sc_dv_addr = 0xFF;
	}

	/* sneak peek the endpoint direction */
	if (req.bmRequestType & UE_DIR_IN) {
		csr |= AT91_UDP_CSR_DIR;
	} else {
		csr &= ~AT91_UDP_CSR_DIR;
	}

	/* write the direction of the control transfer */
	AT91_CSR_ACK(csr, temp);
	bus_space_write_4(td->io_tag, td->io_hdl,
	    td->status_reg, csr);
	return (0);			/* complete */

not_complete:
	/* clear interrupts, if any */
	if (temp) {
		DPRINTFN(4, "clearing 0x%08x\n", temp);
		AT91_CSR_ACK(csr, temp);
		bus_space_write_4(td->io_tag, td->io_hdl,
		    td->status_reg, csr);
	}
	return (1);			/* not complete */

}

static uint8_t
at9100_dci_data_rx(struct at9100_dci_td *td)
{
	struct usbd_page_search buf_res;
	uint32_t csr;
	uint32_t temp;
	uint16_t count;
	uint8_t to;
	uint8_t got_short;

	to = 2;				/* don't loop forever! */
	got_short = 0;

	/* check if any of the FIFO banks have data */
repeat:
	/* read out FIFO status */
	csr = bus_space_read_4(td->io_tag, td->io_hdl,
	    td->status_reg);

	DPRINTFN(4, "csr=0x%08x rem=%u\n", csr, td->remainder);

	if (csr & AT91_UDP_CSR_RXSETUP) {
		if (td->remainder == 0) {
			/*
			 * We are actually complete and have
			 * received the next SETUP
			 */
			DPRINTFN(4, "faking complete\n");
			return (0);	/* complete */
		}
		/*
	         * USB Host Aborted the transfer.
	         */
		td->error = 1;
		return (0);		/* complete */
	}
	/* Make sure that "STALLSENT" gets cleared */
	temp = csr;
	temp &= AT91_UDP_CSR_STALLSENT;

	/* check status */
	if (!(csr & (AT91_UDP_CSR_RX_DATA_BK0 |
	    AT91_UDP_CSR_RX_DATA_BK1))) {
		if (temp) {
			/* write command */
			AT91_CSR_ACK(csr, temp);
			bus_space_write_4(td->io_tag, td->io_hdl,
			    td->status_reg, csr);
		}
		return (1);		/* not complete */
	}
	/* get the packet byte count */
	count = (csr & AT91_UDP_CSR_RXBYTECNT) >> 16;

	/* verify the packet byte count */
	if (count != td->max_packet_size) {
		if (count < td->max_packet_size) {
			/* we have a short packet */
			td->short_pkt = 1;
			got_short = 1;
		} else {
			/* invalid USB packet */
			td->error = 1;
			return (0);	/* we are complete */
		}
	}
	/* verify the packet byte count */
	if (count > td->remainder) {
		/* invalid USB packet */
		td->error = 1;
		return (0);		/* we are complete */
	}
	while (count > 0) {
		usbd_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* receive data */
		bus_space_read_multi_1(td->io_tag, td->io_hdl,
		    td->fifo_reg, buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* clear status bits */
	if (td->support_multi_buffer) {
		if (td->fifo_bank) {
			td->fifo_bank = 0;
			temp |= AT91_UDP_CSR_RX_DATA_BK1;
		} else {
			td->fifo_bank = 1;
			temp |= AT91_UDP_CSR_RX_DATA_BK0;
		}
	} else {
		temp |= (AT91_UDP_CSR_RX_DATA_BK0 |
		    AT91_UDP_CSR_RX_DATA_BK1);
	}

	/* write command */
	AT91_CSR_ACK(csr, temp);
	bus_space_write_4(td->io_tag, td->io_hdl,
	    td->status_reg, csr);

	/*
	 * NOTE: We may have to delay a little bit before
	 * proceeding after clearing the DATA_BK bits.
	 */

	/* check if we are complete */
	if ((td->remainder == 0) || got_short) {
		if (td->short_pkt) {
			/* we are complete */
			return (0);
		}
		/* else need to receive a zero length packet */
	}
	if (--to) {
		goto repeat;
	}
	return (1);			/* not complete */
}

static uint8_t
at9100_dci_data_tx(struct at9100_dci_td *td)
{
	struct usbd_page_search buf_res;
	uint32_t csr;
	uint32_t temp;
	uint16_t count;
	uint8_t to;

	to = 2;				/* don't loop forever! */

repeat:

	/* read out FIFO status */
	csr = bus_space_read_4(td->io_tag, td->io_hdl,
	    td->status_reg);

	DPRINTFN(4, "csr=0x%08x rem=%u\n", csr, td->remainder);

	if (csr & AT91_UDP_CSR_RXSETUP) {
		/*
	         * The current transfer was aborted
	         * by the USB Host
	         */
		td->error = 1;
		return (0);		/* complete */
	}
	/* Make sure that "STALLSENT" gets cleared */
	temp = csr;
	temp &= AT91_UDP_CSR_STALLSENT;

	if (csr & AT91_UDP_CSR_TXPKTRDY) {
		if (temp) {
			/* write command */
			AT91_CSR_ACK(csr, temp);
			bus_space_write_4(td->io_tag, td->io_hdl,
			    td->status_reg, csr);
		}
		return (1);		/* not complete */
	} else {
		/* clear TXCOMP and set TXPKTRDY */
		temp |= (AT91_UDP_CSR_TXCOMP |
		    AT91_UDP_CSR_TXPKTRDY);
	}

	count = td->max_packet_size;
	if (td->remainder < count) {
		/* we have a short packet */
		td->short_pkt = 1;
		count = td->remainder;
	}
	while (count > 0) {

		usbd_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* transmit data */
		bus_space_write_multi_1(td->io_tag, td->io_hdl,
		    td->fifo_reg, buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* write command */
	AT91_CSR_ACK(csr, temp);
	bus_space_write_4(td->io_tag, td->io_hdl,
	    td->status_reg, csr);

	/* check remainder */
	if (td->remainder == 0) {
		if (td->short_pkt) {
			return (0);	/* complete */
		}
		/* else we need to transmit a short packet */
	}
	if (--to) {
		goto repeat;
	}
	return (1);			/* not complete */
}

static uint8_t
at9100_dci_data_tx_sync(struct at9100_dci_td *td)
{
	struct at9100_dci_softc *sc;
	uint32_t csr;
	uint32_t temp;

#if 0
repeat:
#endif

	/* read out FIFO status */
	csr = bus_space_read_4(td->io_tag, td->io_hdl,
	    td->status_reg);

	DPRINTFN(4, "csr=0x%08x\n", csr);

	if (csr & AT91_UDP_CSR_RXSETUP) {
		DPRINTFN(4, "faking complete\n");
		/* Race condition */
		return (0);		/* complete */
	}
	temp = csr;
	temp &= (AT91_UDP_CSR_STALLSENT |
	    AT91_UDP_CSR_TXCOMP);

	/* check status */
	if (csr & AT91_UDP_CSR_TXPKTRDY) {
		goto not_complete;
	}
	if (!(csr & AT91_UDP_CSR_TXCOMP)) {
		goto not_complete;
	}
	sc = td->pc->xfer->usb_sc;
	if (sc->sc_dv_addr != 0xFF) {
		/*
		 * The AT91 has a special requirement with regard to
		 * setting the address and that is to write the new
		 * address before clearing TXCOMP:
		 */
		at9100_dci_set_address(sc, sc->sc_dv_addr);
	}
	/* write command */
	AT91_CSR_ACK(csr, temp);
	bus_space_write_4(td->io_tag, td->io_hdl,
	    td->status_reg, csr);

	return (0);			/* complete */

not_complete:
	if (temp) {
		/* write command */
		AT91_CSR_ACK(csr, temp);
		bus_space_write_4(td->io_tag, td->io_hdl,
		    td->status_reg, csr);
	}
	return (1);			/* not complete */
}

static uint8_t
at9100_dci_xfer_do_fifo(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc;
	struct at9100_dci_td *td;
	uint8_t temp;

	DPRINTFN(8, "\n");

	td = xfer->td_transfer_cache;
	while (1) {
		if ((td->func) (td)) {
			/* operation in progress */
			break;
		}
		if (((void *)td) == xfer->td_transfer_last) {
			goto done;
		}
		if (td->error) {
			goto done;
		} else if (td->remainder > 0) {
			/*
			 * We had a short transfer. If there is no alternate
			 * next, stop processing !
			 */
			if (!td->alt_next) {
				goto done;
			}
		}
		/*
		 * Fetch the next transfer descriptor and transfer
		 * some flags to the next transfer descriptor
		 */
		temp = 0;
		if (td->fifo_bank)
			temp |= 1;
		td = td->obj_next;
		xfer->td_transfer_cache = td;
		if (temp & 1)
			td->fifo_bank = 1;
	}
	return (1);			/* not complete */

done:
	sc = xfer->usb_sc;
	temp = (xfer->endpoint & UE_ADDR);

	/* update FIFO bank flag and multi buffer */
	if (td->fifo_bank) {
		sc->sc_ep_flags[temp].fifo_bank = 1;
	} else {
		sc->sc_ep_flags[temp].fifo_bank = 0;
	}

	/* compute all actual lengths */

	at9100_dci_standard_done(xfer);

	return (0);			/* complete */
}

static void
at9100_dci_interrupt_poll(struct at9100_dci_softc *sc)
{
	struct usbd_xfer *xfer;

	LIST_FOREACH(xfer, &sc->sc_bus.intr_list_head, interrupt_list) {
		if (!at9100_dci_xfer_do_fifo(xfer)) {
			/* queue callback for execution */
			usbd_callback_wrapper(xfer, NULL,
			    USBD_CONTEXT_CALLBACK);
		}
	}
	return;
}

static void
at9100_dci_vbus_interrupt(struct usbd_bus *bus, uint8_t is_on)
{
	struct at9100_dci_softc *sc = AT9100_DCI_BUS2SC(bus);

	DPRINTFN(4, "vbus = %u\n", is_on);

	mtx_lock(&(sc->sc_bus.mtx));
	if (is_on) {
		if (!sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 1;

			/* complete root HUB interrupt endpoint */

			usbd_std_root_transfer(&(sc->sc_root_intr),
			    &at9100_dci_root_intr_done);
		}
	} else {
		if (sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 0;
			sc->sc_flags.status_bus_reset = 0;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;

			/* complete root HUB interrupt endpoint */

			usbd_std_root_transfer(&(sc->sc_root_intr),
			    &at9100_dci_root_intr_done);
		}
	}

	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

void
at9100_dci_interrupt(struct at9100_dci_softc *sc)
{
	uint32_t status;

	mtx_lock(&(sc->sc_bus.mtx));

	status = AT91_UDP_READ_4(sc, AT91_UDP_ISR);
	status &= AT91_UDP_INT_DEFAULT;

	if (!status) {
		mtx_unlock(&(sc->sc_bus.mtx));
		return;
	}
	/* acknowledge interrupts */

	AT91_UDP_WRITE_4(sc, AT91_UDP_ICR, status);

	/* check for any bus state change interrupts */

	if (status & AT91_UDP_INT_BUS) {

		DPRINTFN(4, "real bus interrupt 0x%08x\n", status);

		if (status & AT91_UDP_INT_END_BR) {
			sc->sc_flags.status_bus_reset = 1;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;

			/* disable resume interrupt */
			AT91_UDP_WRITE_4(sc, AT91_UDP_IDR,
			    AT91_UDP_INT_RXRSM);
			/* enable suspend interrupt */
			AT91_UDP_WRITE_4(sc, AT91_UDP_IER,
			    AT91_UDP_INT_RXSUSP);
		}
		/*
	         * If RXRSM and RXSUSP is set at the same time we interpret
	         * that like RESUME. Resume is set when there is at least 3
	         * milliseconds of inactivity on the USB BUS.
	         */
		if (status & AT91_UDP_INT_RXRSM) {
			if (sc->sc_flags.status_suspend) {
				sc->sc_flags.status_suspend = 0;
				sc->sc_flags.change_suspend = 1;

				/* disable resume interrupt */
				AT91_UDP_WRITE_4(sc, AT91_UDP_IDR,
				    AT91_UDP_INT_RXRSM);
				/* enable suspend interrupt */
				AT91_UDP_WRITE_4(sc, AT91_UDP_IER,
				    AT91_UDP_INT_RXSUSP);
			}
		} else if (status & AT91_UDP_INT_RXSUSP) {
			if (!sc->sc_flags.status_suspend) {
				sc->sc_flags.status_suspend = 1;
				sc->sc_flags.change_suspend = 1;

				/* disable suspend interrupt */
				AT91_UDP_WRITE_4(sc, AT91_UDP_IDR,
				    AT91_UDP_INT_RXSUSP);

				/* enable resume interrupt */
				AT91_UDP_WRITE_4(sc, AT91_UDP_IER,
				    AT91_UDP_INT_RXRSM);
			}
		}
		/* complete root HUB interrupt endpoint */

		usbd_std_root_transfer(&(sc->sc_root_intr),
		    &at9100_dci_root_intr_done);
	}
	/* check for any endpoint interrupts */

	if (status & AT91_UDP_INT_EPS) {

		DPRINTFN(4, "real endpoint interrupt 0x%08x\n", status);

		at9100_dci_interrupt_poll(sc);
	}
	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

static void
at9100_dci_setup_standard_chain_sub(struct at9100_std_temp *temp)
{
	struct at9100_dci_td *td;

	/* get current Transfer Descriptor */
	td = temp->td_next;
	temp->td = td;

	/* prepare for next TD */
	temp->td_next = td->obj_next;

	/* fill out the Transfer Descriptor */
	td->func = temp->func;
	td->pc = temp->pc;
	td->offset = temp->offset;
	td->remainder = temp->len;
	td->fifo_bank = 0;
	td->error = 0;
	td->did_stall = 0;
	td->short_pkt = temp->short_pkt;
	td->alt_next = temp->setup_alt_next;
	return;
}

static void
at9100_dci_setup_standard_chain(struct usbd_xfer *xfer)
{
	struct at9100_std_temp temp;
	struct at9100_dci_softc *sc;
	struct at9100_dci_td *td;
	uint32_t x;
	uint8_t need_sync;
	uint8_t ep_no;

	DPRINTFN(8, "addr=%d endpt=%d sumlen=%d speed=%d\n",
	    xfer->address, UE_GET_ADDR(xfer->endpoint),
	    xfer->sumlen, usbd_get_speed(xfer->udev));

	temp.max_frame_size = xfer->max_frame_size;

	td = xfer->td_start[0];
	xfer->td_transfer_first = td;
	xfer->td_transfer_cache = td;

	/* setup temp */

	temp.td = NULL;
	temp.td_next = xfer->td_start[0];
	temp.setup_alt_next = xfer->flags_int.short_frames_ok;
	temp.offset = 0;

	sc = xfer->usb_sc;
	ep_no = (xfer->endpoint & UE_ADDR);

	/* check if we should prepend a setup message */

	if (xfer->flags_int.control_xfr) {
		if (xfer->flags_int.control_hdr) {

			temp.func = &at9100_dci_setup_rx;
			temp.len = xfer->frlengths[0];
			temp.pc = xfer->frbuffers + 0;
			temp.short_pkt = temp.len ? 1 : 0;

			at9100_dci_setup_standard_chain_sub(&temp);
		}
		x = 1;
	} else {
		x = 0;
	}

	if (x != xfer->nframes) {
		if (xfer->endpoint & UE_DIR_IN) {
			temp.func = &at9100_dci_data_tx;
			need_sync = 1;
		} else {
			temp.func = &at9100_dci_data_rx;
			need_sync = 0;
		}

		/* setup "pc" pointer */
		temp.pc = xfer->frbuffers + x;
	} else {
		need_sync = 0;
	}

	while (x != xfer->nframes) {

		/* DATA0 / DATA1 message */

		temp.len = xfer->frlengths[x];

		x++;

		if (x == xfer->nframes) {
			temp.setup_alt_next = 0;
		}
		if (temp.len == 0) {

			/* make sure that we send an USB packet */

			temp.short_pkt = 0;

		} else {

			/* regular data transfer */

			temp.short_pkt = (xfer->flags.force_short_xfer) ? 0 : 1;
		}

		at9100_dci_setup_standard_chain_sub(&temp);

		if (xfer->flags_int.isochronous_xfr) {
			temp.offset += temp.len;
		} else {
			/* get next Page Cache pointer */
			temp.pc = xfer->frbuffers + x;
		}
	}

	/* always setup a valid "pc" pointer for status and sync */
	temp.pc = xfer->frbuffers + 0;

	/* check if we need to sync */
	if (need_sync && !xfer->flags_int.isochronous_xfr) {

		/* we need a SYNC point after TX */
		temp.func = &at9100_dci_data_tx_sync;
		temp.len = 0;
		temp.short_pkt = 0;

		at9100_dci_setup_standard_chain_sub(&temp);
	}
	/* check if we should append a status stage */

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		/*
		 * Send a DATA1 message and invert the current
		 * endpoint direction.
		 */
		if (xfer->endpoint & UE_DIR_IN) {
			temp.func = &at9100_dci_data_rx;
			need_sync = 0;
		} else {
			temp.func = &at9100_dci_data_tx;
			need_sync = 1;
		}
		temp.len = 0;
		temp.short_pkt = 0;

		at9100_dci_setup_standard_chain_sub(&temp);
		if (need_sync) {
			/* we need a SYNC point after TX */
			temp.func = &at9100_dci_data_tx_sync;
			temp.len = 0;
			temp.short_pkt = 0;

			at9100_dci_setup_standard_chain_sub(&temp);
		}
	}
	/* must have at least one frame! */
	td = temp.td;
	xfer->td_transfer_last = td;

	/* setup the correct fifo bank */
	if (sc->sc_ep_flags[ep_no].fifo_bank) {
		td = xfer->td_transfer_first;
		td->fifo_bank = 1;
	}
	return;
}

static void
at9100_dci_timeout(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;

	DPRINTFN(0, "xfer=%p\n", xfer);

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	at9100_dci_device_done(xfer, USBD_ERR_TIMEOUT);

	/* queue callback for execution */
	usbd_callback_wrapper(xfer, NULL, USBD_CONTEXT_CALLBACK);

	mtx_unlock(&sc->sc_bus.mtx);

	return;
}

static void
at9100_dci_start_standard_chain(struct usbd_xfer *xfer)
{
	DPRINTFN(8, "\n");

	/* poll one time */
	if (at9100_dci_xfer_do_fifo(xfer)) {

		struct at9100_dci_softc *sc = xfer->usb_sc;
		uint8_t ep_no = xfer->endpoint & UE_ADDR;

		/*
		 * Only enable the endpoint interrupt when we are actually
		 * waiting for data, hence we are dealing with level
		 * triggered interrupts !
		 */
		AT91_UDP_WRITE_4(sc, AT91_UDP_IER, AT91_UDP_INT_EP(ep_no));

		DPRINTFN(14, "enable interrupts on endpoint %d\n", ep_no);

		/* queue up transfer on interrupt list */
		usbd_transfer_intr_enqueue(xfer);

		/* setup a timeout, if any */
		if (xfer->timeout && (!xfer->flags.use_polling)) {
			usb_callout_reset(&xfer->timeout_handle,
			    USBD_MS_TO_TICKS(xfer->timeout),
			    (void *)&at9100_dci_timeout, xfer);
		}
	} else {
		/*
		 * The USB transfer is complete. Queue callback for
		 * execution:
		 */
		usbd_callback_wrapper(xfer, NULL,
		    USBD_CONTEXT_CALLBACK);
	}
	return;
}

static void
at9100_dci_root_intr_done(struct usbd_xfer *xfer,
    struct usbd_std_root_transfer *std)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;

	DPRINTFN(8, "\n");

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USBD_STD_ROOT_TR_PRE_DATA) {
		if (std->state == USBD_STD_ROOT_TR_PRE_CALLBACK) {
			/* transfer transferred */
			at9100_dci_device_done(xfer, std->err);
		}
		goto done;
	}
	/* setup buffer */
	std->ptr = sc->sc_hub_idata;
	std->len = sizeof(sc->sc_hub_idata);

	/* set port bit */
	sc->sc_hub_idata[0] = 0x02;	/* we only have one port */

done:
	return;
}

static usbd_status_t
at9100_dci_standard_done_sub(struct usbd_xfer *xfer)
{
	struct at9100_dci_td *td;
	uint32_t len;
	uint8_t error;

	DPRINTFN(8, "\n");

	td = xfer->td_transfer_cache;

	do {
		len = td->remainder;

		if (xfer->aframes != xfer->nframes) {
			/*
		         * Verify the length and subtract
		         * the remainder from "frlengths[]":
		         */
			if (len > xfer->frlengths[xfer->aframes]) {
				td->error = 1;
			} else {
				xfer->frlengths[xfer->aframes] -= len;
			}
		}
		/* Check for transfer error */
		if (td->error) {
			/* the transfer is finished */
			error = 1;
			td = NULL;
			break;
		}
		/* Check for short transfer */
		if (len > 0) {
			if (xfer->flags_int.short_frames_ok) {
				/* follow alt next */
				if (td->alt_next) {
					td = td->obj_next;
				} else {
					td = NULL;
				}
			} else {
				/* the transfer is finished */
				td = NULL;
			}
			error = 0;
			break;
		}
		td = td->obj_next;

		/* this USB frame is complete */
		error = 0;
		break;

	} while (0);

	/* update transfer cache */

	xfer->td_transfer_cache = td;

	return (error ?
	    USBD_ERR_STALLED : USBD_ERR_NORMAL_COMPLETION);
}

static void
at9100_dci_standard_done(struct usbd_xfer *xfer)
{
	usbd_status_t err = 0;

	DPRINTFN(12, "xfer=%p pipe=%p transfer done\n",
	    xfer, xfer->pipe);

	/* reset scanner */

	xfer->td_transfer_cache = xfer->td_transfer_first;

	if (xfer->flags_int.control_xfr) {

		if (xfer->flags_int.control_hdr) {

			err = at9100_dci_standard_done_sub(xfer);
		}
		xfer->aframes = 1;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}
	while (xfer->aframes != xfer->nframes) {

		err = at9100_dci_standard_done_sub(xfer);
		xfer->aframes++;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		err = at9100_dci_standard_done_sub(xfer);
	}
done:
	at9100_dci_device_done(xfer, err);
	return;
}

/*------------------------------------------------------------------------*
 *	at9100_dci_device_done
 *
 * NOTE: this function can be called more than one time on the
 * same USB transfer!
 *------------------------------------------------------------------------*/
static void
at9100_dci_device_done(struct usbd_xfer *xfer, usbd_status_t error)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;
	uint8_t ep_no;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	DPRINTFN(1, "xfer=%p, pipe=%p, error=%d\n",
	    xfer, xfer->pipe, error);

	if (xfer->flags_int.usb_mode == USB_MODE_DEVICE) {
		ep_no = (xfer->endpoint & UE_ADDR);

		/* disable endpoint interrupt */
		AT91_UDP_WRITE_4(sc, AT91_UDP_IDR, AT91_UDP_INT_EP(ep_no));

		DPRINTFN(14, "disable interrupts on endpoint %d\n", ep_no);
	}
	/* dequeue transfer and start next transfer */
	usbd_transfer_dequeue(xfer, error);
	return;
}

static void
at9100_dci_set_stall(struct usbd_device *udev, struct usbd_xfer *xfer,
    struct usbd_pipe *pipe)
{
	struct at9100_dci_softc *sc;
	uint32_t csr_val;
	uint8_t csr_reg;

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	DPRINTFN(4, "pipe=%p\n", pipe);

	if (xfer) {
		/* cancel any ongoing transfers */
		at9100_dci_device_done(xfer, USBD_ERR_STALLED);
	}
	/* set FORCESTALL */
	sc = AT9100_DCI_BUS2SC(udev->bus);
	csr_reg = (pipe->edesc->bEndpointAddress & UE_ADDR);
	csr_reg = AT91_UDP_CSR(csr_reg);
	csr_val = AT91_UDP_READ_4(sc, csr_reg);
	AT91_CSR_ACK(csr_val, AT91_UDP_CSR_FORCESTALL);
	AT91_UDP_WRITE_4(sc, csr_reg, csr_val);
	return;
}

static void
at9100_dci_reset_ep(struct usbd_device *udev, uint8_t ep_no)
{
	struct at9100_dci_softc *sc;
	const struct usbd_hw_ep_profile *pf;
	uint32_t csr_val;
	uint32_t temp;
	uint8_t csr_reg;
	uint8_t to;

	/* get softc */
	sc = AT9100_DCI_BUS2SC(udev->bus);

	/* get endpoint profile */
	at9100_dci_get_hw_ep_profile(udev, &pf, ep_no);

	/* reset FIFO */
	AT91_UDP_WRITE_4(sc, AT91_UDP_RST, AT91_UDP_RST_EP(ep_no));
	AT91_UDP_WRITE_4(sc, AT91_UDP_RST, 0);

	/*
	 * NOTE: One would assume that a FIFO reset would release the
	 * FIFO banks aswell, but it doesn't! We have to do this
	 * manually!
	 */
	csr_reg = AT91_UDP_CSR(ep_no);

	/* release FIFO banks, if any */
	for (to = 0; to != 2; to++) {

		/* get csr value */
		csr_val = AT91_UDP_READ_4(sc, csr_reg);

		if (csr_val & (AT91_UDP_CSR_RX_DATA_BK0 |
		    AT91_UDP_CSR_RX_DATA_BK1)) {
			/* clear status bits */
			if (pf->support_multi_buffer) {
				if (sc->sc_ep_flags[ep_no].fifo_bank) {
					sc->sc_ep_flags[ep_no].fifo_bank = 0;
					temp = AT91_UDP_CSR_RX_DATA_BK1;
				} else {
					sc->sc_ep_flags[ep_no].fifo_bank = 1;
					temp = AT91_UDP_CSR_RX_DATA_BK0;
				}
			} else {
				temp = (AT91_UDP_CSR_RX_DATA_BK0 |
				    AT91_UDP_CSR_RX_DATA_BK1);
			}
		} else {
			temp = 0;
		}

		/* clear FORCESTALL */
		temp |= AT91_UDP_CSR_STALLSENT;

		AT91_CSR_ACK(csr_val, temp);
		AT91_UDP_WRITE_4(sc, csr_reg, csr_val);
	}
	return;
}

static void
at9100_dci_clear_stall(struct usbd_device *udev, struct usbd_pipe *pipe)
{
	DPRINTFN(4, "pipe=%p\n", pipe);

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	/* reset endpoint */
	at9100_dci_reset_ep(udev,
	    (pipe->edesc->bEndpointAddress & UE_ADDR));

	return;
}

/*------------------------------------------------------------------------*
 *	at9100_dci_set_config
 *
 * NOTE: Calling this function builds on the assumtion that
 * you have a configuration that is valid !
 *------------------------------------------------------------------------*/
static void
at9100_dci_set_config(struct usbd_device *udev,
    usb_config_descriptor_t *cd)
{
	struct at9100_dci_softc *sc;
	usb_endpoint_descriptor_t *ed;
	uint32_t csr_val = 0;
	uint8_t n;
	uint8_t ep_type;
	uint8_t ep_dir;

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	if (udev->flags.usb_mode == USB_MODE_HOST) {
		/* this is the Root HUB */
		return;
	}
	sc = AT9100_DCI_BUS2SC(udev->bus);
	AT91_CSR_ACK(csr_val, 0);

	/* disable everything, but the control 0 endpoint */

	for (n = 1; n != AT91_UDP_EP_MAX; n++) {

		/* disable endpoint */
		AT91_UDP_WRITE_4(sc, AT91_UDP_CSR(n), csr_val);

		/* reset endpoint */
		at9100_dci_reset_ep(udev, n);
	}

	if (cd == NULL) {
		/* nothing more to do */
		return;
	}
	ed = NULL;

	/* enable all endpoints in the configuration */

	while ((ed = (void *)usbd_desc_foreach(cd, (void *)ed))) {

		if ((ed->bDescriptorType != UDESC_ENDPOINT) ||
		    (ed->bLength < sizeof(*ed))) {
			continue;
		}
		/* enable endpoint */
		csr_val &= ~AT91_UDP_CSR_ET_MASK;
		csr_val |= AT91_UDP_CSR_EPEDS;

		n = (ed->bEndpointAddress & UE_ADDR);
		ep_type = (ed->bmAttributes & UE_XFERTYPE);
		ep_dir = (ed->bEndpointAddress & (UE_DIR_IN | UE_DIR_OUT));

		if (ep_type == UE_CONTROL) {
			csr_val |= AT91_UDP_CSR_ET_CTRL;
		} else {
			if (ep_type == UE_BULK) {
				csr_val |= AT91_UDP_CSR_ET_BULK;
			} else if (ep_type == UE_INTERRUPT) {
				csr_val |= AT91_UDP_CSR_ET_INT;
			} else {
				csr_val |= AT91_UDP_CSR_ET_ISO;
			}
			if (ep_dir & UE_DIR_IN) {
				csr_val |= AT91_UDP_CSR_ET_DIR_IN;
			}
		}

		/* enable endpoint */
		AT91_UDP_WRITE_4(sc, AT91_UDP_CSR(n), csr_val);
	}
	return;
}

usbd_status_t
at9100_dci_init(struct at9100_dci_softc *sc)
{
	uint32_t csr_val;
	uint8_t n;

	DPRINTFN(0, "start\n");

	/* set up the bus structure */
	sc->sc_bus.usbrev = USBREV_1_1;
	sc->sc_bus.methods = &at9100_dci_bus_methods;

	mtx_lock(&(sc->sc_bus.mtx));

	/* turn on clocks */

	if (sc->sc_clocks_on) {
		(sc->sc_clocks_on) (sc->sc_clocks_arg);
	}
	/* wait a little for things to stabilise */
	DELAY(1000);

	/* disable and clear all interrupts */
	AT91_UDP_WRITE_4(sc, AT91_UDP_IDR, 0xFFFFFFFF);
	AT91_UDP_WRITE_4(sc, AT91_UDP_ICR, 0xFFFFFFFF);

	csr_val = 0;
	AT91_CSR_ACK(csr_val, 0);

	/* disable and reset all endpoints */

	for (n = 0; n != AT91_UDP_EP_MAX; n++) {

		/* disable endpoint */
		AT91_UDP_WRITE_4(sc, AT91_UDP_CSR(n), csr_val);
	}

	/* enable the interrupts we want */

	AT91_UDP_WRITE_4(sc, AT91_UDP_IER, AT91_UDP_INT_BUS);

	/* turn off clocks */

	at9100_dci_clocks_off(sc);

	mtx_unlock(&(sc->sc_bus.mtx));

	/* catch any lost interrupts */

	at9100_dci_do_poll(&(sc->sc_bus));

	return (0);			/* success */
}

void
at9100_dci_uninit(struct at9100_dci_softc *sc)
{
	mtx_lock(&(sc->sc_bus.mtx));

	/* disable and clear all interrupts */
	AT91_UDP_WRITE_4(sc, AT91_UDP_IDR, 0xFFFFFFFF);
	AT91_UDP_WRITE_4(sc, AT91_UDP_ICR, 0xFFFFFFFF);

	sc->sc_flags.port_powered = 0;
	sc->sc_flags.status_vbus = 0;
	sc->sc_flags.status_bus_reset = 0;
	sc->sc_flags.status_suspend = 0;
	sc->sc_flags.change_suspend = 0;
	sc->sc_flags.change_connect = 1;

	at9100_dci_pull_down(sc);
	at9100_dci_clocks_off(sc);
	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

void
at9100_dci_suspend(struct at9100_dci_softc *sc)
{
	return;
}

void
at9100_dci_resume(struct at9100_dci_softc *sc)
{
	return;
}

static void
at9100_dci_do_poll(struct usbd_bus *bus)
{
	struct at9100_dci_softc *sc = AT9100_DCI_BUS2SC(bus);

	mtx_lock(&(sc->sc_bus.mtx));
	at9100_dci_interrupt_poll(sc);
	at9100_dci_root_ctrl_poll(sc);
	mtx_unlock(&(sc->sc_bus.mtx));
	return;
}

/*------------------------------------------------------------------------*
 * at9100 bulk support
 *------------------------------------------------------------------------*/
static void
at9100_dci_device_bulk_open(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_device_bulk_close(struct usbd_xfer *xfer)
{
	at9100_dci_device_done(xfer, USBD_ERR_CANCELLED);
	return;
}

static void
at9100_dci_device_bulk_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
at9100_dci_device_bulk_start(struct usbd_xfer *xfer)
{
	/* setup TDs */
	at9100_dci_setup_standard_chain(xfer);
	at9100_dci_start_standard_chain(xfer);
	return;
}

struct usbd_pipe_methods at9100_dci_device_bulk_methods =
{
	.open = at9100_dci_device_bulk_open,
	.close = at9100_dci_device_bulk_close,
	.enter = at9100_dci_device_bulk_enter,
	.start = at9100_dci_device_bulk_start,
};

/*------------------------------------------------------------------------*
 * at9100 control support
 *------------------------------------------------------------------------*/
static void
at9100_dci_device_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_device_ctrl_close(struct usbd_xfer *xfer)
{
	at9100_dci_device_done(xfer, USBD_ERR_CANCELLED);
	return;
}

static void
at9100_dci_device_ctrl_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
at9100_dci_device_ctrl_start(struct usbd_xfer *xfer)
{
	/* setup TDs */
	at9100_dci_setup_standard_chain(xfer);
	at9100_dci_start_standard_chain(xfer);
	return;
}

struct usbd_pipe_methods at9100_dci_device_ctrl_methods =
{
	.open = at9100_dci_device_ctrl_open,
	.close = at9100_dci_device_ctrl_close,
	.enter = at9100_dci_device_ctrl_enter,
	.start = at9100_dci_device_ctrl_start,
};

/*------------------------------------------------------------------------*
 * at9100 interrupt support
 *------------------------------------------------------------------------*/
static void
at9100_dci_device_intr_open(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_device_intr_close(struct usbd_xfer *xfer)
{
	at9100_dci_device_done(xfer, USBD_ERR_CANCELLED);
	return;
}

static void
at9100_dci_device_intr_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
at9100_dci_device_intr_start(struct usbd_xfer *xfer)
{
	/* setup TDs */
	at9100_dci_setup_standard_chain(xfer);
	at9100_dci_start_standard_chain(xfer);
	return;
}

struct usbd_pipe_methods at9100_dci_device_intr_methods =
{
	.open = at9100_dci_device_intr_open,
	.close = at9100_dci_device_intr_close,
	.enter = at9100_dci_device_intr_enter,
	.start = at9100_dci_device_intr_start,
};

/*------------------------------------------------------------------------*
 * at9100 full speed isochronous support
 *------------------------------------------------------------------------*/
static void
at9100_dci_device_isoc_fs_open(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_device_isoc_fs_close(struct usbd_xfer *xfer)
{
	at9100_dci_device_done(xfer, USBD_ERR_CANCELLED);
	return;
}

static void
at9100_dci_device_isoc_fs_enter(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;
	uint32_t temp;
	uint32_t nframes;

	DPRINTFN(5, "xfer=%p next=%d nframes=%d\n",
	    xfer, xfer->pipe->isoc_next, xfer->nframes);

	/* get the current frame index */

	nframes = AT91_UDP_READ_4(sc, AT91_UDP_FRM);

	/*
	 * check if the frame index is within the window where the frames
	 * will be inserted
	 */
	temp = (nframes - xfer->pipe->isoc_next) & AT91_UDP_FRM_MASK;

	if ((LIST_FIRST(&(xfer->pipe->list_head)) == NULL) ||
	    (temp < xfer->nframes)) {
		/*
		 * If there is data underflow or the pipe queue is
		 * empty we schedule the transfer a few frames ahead
		 * of the current frame position. Else two isochronous
		 * transfers might overlap.
		 */
		xfer->pipe->isoc_next = (nframes + 3) & AT91_UDP_FRM_MASK;
		DPRINTFN(2, "start next=%d\n", xfer->pipe->isoc_next);
	}
	/*
	 * compute how many milliseconds the insertion is ahead of the
	 * current frame position:
	 */
	temp = (xfer->pipe->isoc_next - nframes) & AT91_UDP_FRM_MASK;

	/*
	 * pre-compute when the isochronous transfer will be finished:
	 */
	xfer->isoc_time_complete =
	    usbd_isoc_time_expand(&(sc->sc_bus), nframes) + temp +
	    xfer->nframes;

	/* compute frame number for next insertion */
	xfer->pipe->isoc_next += xfer->nframes;

	/* setup TDs */
	at9100_dci_setup_standard_chain(xfer);
	return;
}

static void
at9100_dci_device_isoc_fs_start(struct usbd_xfer *xfer)
{
	/* start TD chain */
	at9100_dci_start_standard_chain(xfer);
	return;
}

struct usbd_pipe_methods at9100_dci_device_isoc_fs_methods =
{
	.open = at9100_dci_device_isoc_fs_open,
	.close = at9100_dci_device_isoc_fs_close,
	.enter = at9100_dci_device_isoc_fs_enter,
	.start = at9100_dci_device_isoc_fs_start,
};

/*------------------------------------------------------------------------*
 * at9100 root control support
 *------------------------------------------------------------------------*
 * simulate a hardware HUB by handling
 * all the necessary requests
 *------------------------------------------------------------------------*/

static void
at9100_dci_root_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_root_ctrl_close(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;

	if (sc->sc_root_ctrl.xfer == xfer) {
		sc->sc_root_ctrl.xfer = NULL;
	}
	at9100_dci_device_done(xfer, USBD_ERR_CANCELLED);
	return;
}

/*
 * USB descriptors for the virtual Root HUB:
 */

static const usb_device_descriptor_t at9100_dci_devd = {
	.bLength = sizeof(usb_device_descriptor_t),
	.bDescriptorType = UDESC_DEVICE,
	.bcdUSB = {0x00, 0x02},
	.bDeviceClass = UDCLASS_HUB,
	.bDeviceSubClass = UDSUBCLASS_HUB,
	.bDeviceProtocol = UDPROTO_HSHUBSTT,
	.bMaxPacketSize = 64,
	.bcdDevice = {0x00, 0x01},
	.iManufacturer = 1,
	.iProduct = 2,
	.bNumConfigurations = 1,
};

static const usb_device_qualifier_t at9100_dci_odevd = {
	.bLength = sizeof(usb_device_qualifier_t),
	.bDescriptorType = UDESC_DEVICE_QUALIFIER,
	.bcdUSB = {0x00, 0x02},
	.bDeviceClass = UDCLASS_HUB,
	.bDeviceSubClass = UDSUBCLASS_HUB,
	.bDeviceProtocol = UDPROTO_FSHUB,
	.bMaxPacketSize0 = 0,
	.bNumConfigurations = 0,
};

static const struct at9100_dci_config_desc at9100_dci_confd = {
	.confd = {
		.bLength = sizeof(usb_config_descriptor_t),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength[0] = sizeof(at9100_dci_confd),
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_SELF_POWERED,
		.bMaxPower = 0,
	},
	.ifcd = {
		.bLength = sizeof(usb_interface_descriptor_t),
		.bDescriptorType = UDESC_INTERFACE,
		.bNumEndpoints = 1,
		.bInterfaceClass = UICLASS_HUB,
		.bInterfaceSubClass = UISUBCLASS_HUB,
		.bInterfaceProtocol = UIPROTO_HSHUBSTT,
	},

	.endpd = {
		.bLength = sizeof(usb_endpoint_descriptor_t),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = (UE_DIR_IN | AT9100_DCI_INTR_ENDPT),
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 8,
		.bInterval = 255,
	},
};

static const usb_hub_descriptor_t at9100_dci_hubd = {
	.bDescLength = sizeof(usb_hub_descriptor_t),
	.bDescriptorType = UDESC_HUB,
	.bNbrPorts = 1,
	.wHubCharacteristics[0] =
	(UHD_PWR_NO_SWITCH | UHD_OC_INDIVIDUAL) & 0xFF,
	.wHubCharacteristics[1] =
	(UHD_PWR_NO_SWITCH | UHD_OC_INDIVIDUAL) >> 16,
	.bPwrOn2PwrGood = 50,
	.bHubContrCurrent = 0,
	.DeviceRemovable = {0},		/* port is removable */
};

#define	STRING_LANG \
  0x09, 0x04,				/* American English */

#define	STRING_VENDOR \
  'A', 0, 'T', 0, 'M', 0, 'E', 0, 'L', 0

#define	STRING_PRODUCT \
  'D', 0, 'C', 0, 'I', 0, ' ', 0, 'R', 0, \
  'o', 0, 'o', 0, 't', 0, ' ', 0, 'H', 0, \
  'U', 0, 'B', 0,

USB_MAKE_STRING_DESC(STRING_LANG, at9100_dci_langtab);
USB_MAKE_STRING_DESC(STRING_VENDOR, at9100_dci_vendor);
USB_MAKE_STRING_DESC(STRING_PRODUCT, at9100_dci_product);

static void
at9100_dci_root_ctrl_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
at9100_dci_root_ctrl_start(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;

	sc->sc_root_ctrl.xfer = xfer;

	usbd_config_td_queue_command(
	    &(sc->sc_config_td), NULL, &at9100_dci_root_ctrl_task, 0, 0);

	return;
}

static void
at9100_dci_root_ctrl_task(struct at9100_dci_softc *sc,
    struct at9100_dci_config_copy *cc, uint16_t refcount)
{
	at9100_dci_root_ctrl_poll(sc);
	return;
}

static void
at9100_dci_root_ctrl_done(struct usbd_xfer *xfer,
    struct usbd_std_root_transfer *std)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;
	uint16_t value;
	uint16_t index;
	uint8_t use_polling;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USBD_STD_ROOT_TR_SETUP) {
		if (std->state == USBD_STD_ROOT_TR_PRE_CALLBACK) {
			/* transfer transferred */
			at9100_dci_device_done(xfer, std->err);
		}
		goto done;
	}
	/* buffer reset */
	std->ptr = USBD_ADD_BYTES(&(sc->sc_hub_temp), 0);
	std->len = 0;

	value = UGETW(std->req.wValue);
	index = UGETW(std->req.wIndex);

	use_polling = xfer->flags.use_polling;

	/* demultiplex the control request */

	switch (std->req.bmRequestType) {
	case UT_READ_DEVICE:
		switch (std->req.bRequest) {
		case UR_GET_DESCRIPTOR:
			goto tr_handle_get_descriptor;
		case UR_GET_CONFIG:
			goto tr_handle_get_config;
		case UR_GET_STATUS:
			goto tr_handle_get_status;
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_DEVICE:
		switch (std->req.bRequest) {
		case UR_SET_ADDRESS:
			goto tr_handle_set_address;
		case UR_SET_CONFIG:
			goto tr_handle_set_config;
		case UR_CLEAR_FEATURE:
			goto tr_valid;	/* nop */
		case UR_SET_DESCRIPTOR:
			goto tr_valid;	/* nop */
		case UR_SET_FEATURE:
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_ENDPOINT:
		switch (std->req.bRequest) {
		case UR_CLEAR_FEATURE:
			switch (UGETW(std->req.wValue)) {
			case UF_ENDPOINT_HALT:
				goto tr_handle_clear_halt;
			case UF_DEVICE_REMOTE_WAKEUP:
				goto tr_handle_clear_wakeup;
			default:
				goto tr_stalled;
			}
			break;
		case UR_SET_FEATURE:
			switch (UGETW(std->req.wValue)) {
			case UF_ENDPOINT_HALT:
				goto tr_handle_set_halt;
			case UF_DEVICE_REMOTE_WAKEUP:
				goto tr_handle_set_wakeup;
			default:
				goto tr_stalled;
			}
			break;
		case UR_SYNCH_FRAME:
			goto tr_valid;	/* nop */
		default:
			goto tr_stalled;
		}
		break;

	case UT_READ_ENDPOINT:
		switch (std->req.bRequest) {
		case UR_GET_STATUS:
			goto tr_handle_get_ep_status;
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_INTERFACE:
		switch (std->req.bRequest) {
		case UR_SET_INTERFACE:
			goto tr_handle_set_interface;
		case UR_CLEAR_FEATURE:
			goto tr_valid;	/* nop */
		case UR_SET_FEATURE:
		default:
			goto tr_stalled;
		}
		break;

	case UT_READ_INTERFACE:
		switch (std->req.bRequest) {
		case UR_GET_INTERFACE:
			goto tr_handle_get_interface;
		case UR_GET_STATUS:
			goto tr_handle_get_iface_status;
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_CLASS_INTERFACE:
	case UT_WRITE_VENDOR_INTERFACE:
		/* XXX forward */
		break;

	case UT_READ_CLASS_INTERFACE:
	case UT_READ_VENDOR_INTERFACE:
		/* XXX forward */
		break;

	case UT_WRITE_CLASS_DEVICE:
		switch (std->req.bRequest) {
		case UR_CLEAR_FEATURE:
			goto tr_valid;
		case UR_SET_DESCRIPTOR:
		case UR_SET_FEATURE:
			break;
		default:
			goto tr_stalled;
		}
		break;

	case UT_WRITE_CLASS_OTHER:
		switch (std->req.bRequest) {
		case UR_CLEAR_FEATURE:
			goto tr_handle_clear_port_feature;
		case UR_SET_FEATURE:
			goto tr_handle_set_port_feature;
		case UR_CLEAR_TT_BUFFER:
		case UR_RESET_TT:
		case UR_STOP_TT:
			goto tr_valid;

		default:
			goto tr_stalled;
		}
		break;

	case UT_READ_CLASS_OTHER:
		switch (std->req.bRequest) {
		case UR_GET_TT_STATE:
			goto tr_handle_get_tt_state;
		case UR_GET_STATUS:
			goto tr_handle_get_port_status;
		default:
			goto tr_stalled;
		}
		break;

	case UT_READ_CLASS_DEVICE:
		switch (std->req.bRequest) {
		case UR_GET_DESCRIPTOR:
			goto tr_handle_get_class_descriptor;
		case UR_GET_STATUS:
			goto tr_handle_get_class_status;

		default:
			goto tr_stalled;
		}
		break;
	default:
		goto tr_stalled;
	}
	goto tr_valid;

tr_handle_get_descriptor:
	switch (value >> 8) {
	case UDESC_DEVICE:
		if (value & 0xff) {
			goto tr_stalled;
		}
		std->len = sizeof(at9100_dci_devd);
		std->ptr = USBD_ADD_BYTES(&at9100_dci_devd, 0);
		goto tr_valid;
	case UDESC_CONFIG:
		if (value & 0xff) {
			goto tr_stalled;
		}
		std->len = sizeof(at9100_dci_confd);
		std->ptr = USBD_ADD_BYTES(&at9100_dci_confd, 0);
		goto tr_valid;
	case UDESC_STRING:
		switch (value & 0xff) {
		case 0:		/* Language table */
			std->len = sizeof(at9100_dci_langtab);
			std->ptr = USBD_ADD_BYTES(&at9100_dci_langtab, 0);
			goto tr_valid;

		case 1:		/* Vendor */
			std->len = sizeof(at9100_dci_vendor);
			std->ptr = USBD_ADD_BYTES(&at9100_dci_vendor, 0);
			goto tr_valid;

		case 2:		/* Product */
			std->len = sizeof(at9100_dci_product);
			std->ptr = USBD_ADD_BYTES(&at9100_dci_product, 0);
			goto tr_valid;
		default:
			break;
		}
		break;
	default:
		goto tr_stalled;
	}
	goto tr_stalled;

tr_handle_get_config:
	std->len = 1;
	sc->sc_hub_temp.wValue[0] = sc->sc_conf;
	goto tr_valid;

tr_handle_get_status:
	std->len = 2;
	USETW(sc->sc_hub_temp.wValue, UDS_SELF_POWERED);
	goto tr_valid;

tr_handle_set_address:
	if (value & 0xFF00) {
		goto tr_stalled;
	}
	sc->sc_rt_addr = value;
	goto tr_valid;

tr_handle_set_config:
	if (value >= 2) {
		goto tr_stalled;
	}
	sc->sc_conf = value;
	goto tr_valid;

tr_handle_get_interface:
	std->len = 1;
	sc->sc_hub_temp.wValue[0] = 0;
	goto tr_valid;

tr_handle_get_tt_state:
tr_handle_get_class_status:
tr_handle_get_iface_status:
tr_handle_get_ep_status:
	std->len = 2;
	USETW(sc->sc_hub_temp.wValue, 0);
	goto tr_valid;

tr_handle_set_halt:
tr_handle_set_interface:
tr_handle_set_wakeup:
tr_handle_clear_wakeup:
tr_handle_clear_halt:
	goto tr_valid;

tr_handle_clear_port_feature:
	if (index != 1) {
		goto tr_stalled;
	}
	DPRINTFN(8, "UR_CLEAR_PORT_FEATURE on port %d\n", index);

	switch (value) {
	case UHF_PORT_SUSPEND:
		at9100_dci_wakeup_peer(sc);
		break;

	case UHF_PORT_ENABLE:
		sc->sc_flags.port_enabled = 0;
		break;

	case UHF_PORT_TEST:
	case UHF_PORT_INDICATOR:
	case UHF_C_PORT_ENABLE:
	case UHF_C_PORT_OVER_CURRENT:
	case UHF_C_PORT_RESET:
		/* nops */
		break;
	case UHF_PORT_POWER:
		sc->sc_flags.port_powered = 0;
		at9100_dci_pull_down(sc);
		at9100_dci_clocks_off(sc);
		break;
	case UHF_C_PORT_CONNECTION:
		sc->sc_flags.change_connect = 0;
		break;
	case UHF_C_PORT_SUSPEND:
		sc->sc_flags.change_suspend = 0;
		break;
	default:
		std->err = USBD_ERR_IOERROR;
		goto done;
	}
	goto tr_valid;

tr_handle_set_port_feature:
	if (index != 1) {
		goto tr_stalled;
	}
	DPRINTFN(8, "UR_SET_PORT_FEATURE\n");

	switch (value) {
	case UHF_PORT_ENABLE:
		sc->sc_flags.port_enabled = 1;
		break;
	case UHF_PORT_SUSPEND:
	case UHF_PORT_RESET:
	case UHF_PORT_TEST:
	case UHF_PORT_INDICATOR:
		/* nops */
		break;
	case UHF_PORT_POWER:
		sc->sc_flags.port_powered = 1;
		break;
	default:
		std->err = USBD_ERR_IOERROR;
		goto done;
	}
	goto tr_valid;

tr_handle_get_port_status:

	DPRINTFN(8, "UR_GET_PORT_STATUS\n");

	if (index != 1) {
		goto tr_stalled;
	}
	if (sc->sc_flags.status_vbus) {
		at9100_dci_clocks_on(sc);
		at9100_dci_pull_up(sc);
	} else {
		at9100_dci_pull_down(sc);
		at9100_dci_clocks_off(sc);
	}

	/* Select FULL-speed and Device Side Mode */

	value = UPS_PORT_MODE_DEVICE;

	if (sc->sc_flags.port_powered) {
		value |= UPS_PORT_POWER;
	}
	if (sc->sc_flags.port_enabled) {
		value |= UPS_PORT_ENABLED;
	}
	if (sc->sc_flags.status_vbus &&
	    sc->sc_flags.status_bus_reset) {
		value |= UPS_CURRENT_CONNECT_STATUS;
	}
	if (sc->sc_flags.status_suspend) {
		value |= UPS_SUSPEND;
	}
	USETW(sc->sc_hub_temp.ps.wPortStatus, value);

	value = 0;

	if (sc->sc_flags.change_connect) {
		value |= UPS_C_CONNECT_STATUS;

		if (sc->sc_flags.status_vbus &&
		    sc->sc_flags.status_bus_reset) {
			uint32_t csr = 0;

			/* enable the control endpoint */
			AT91_CSR_ACK(csr, AT91_UDP_CSR_ET_CTRL |
			    AT91_UDP_CSR_EPEDS);
			/* write to FIFO control register */
			AT91_UDP_WRITE_4(sc, AT91_UDP_CSR(0), csr);

			/* reset endpoint flags */
			bzero(sc->sc_ep_flags, sizeof(sc->sc_ep_flags));
		}
	}
	if (sc->sc_flags.change_suspend) {
		value |= UPS_C_SUSPEND;
	}
	USETW(sc->sc_hub_temp.ps.wPortChange, value);
	std->len = sizeof(sc->sc_hub_temp.ps);
	goto tr_valid;

tr_handle_get_class_descriptor:
	if (value & 0xFF) {
		goto tr_stalled;
	}
	std->ptr = USBD_ADD_BYTES(&at9100_dci_hubd, 0);
	std->len = sizeof(at9100_dci_hubd);
	goto tr_valid;

tr_stalled:
	std->err = USBD_ERR_STALLED;
tr_valid:
done:
	return;
}

static void
at9100_dci_root_ctrl_poll(struct at9100_dci_softc *sc)
{
	usbd_std_root_transfer(&(sc->sc_root_ctrl),
	    &at9100_dci_root_ctrl_done);
	return;
}

struct usbd_pipe_methods at9100_dci_root_ctrl_methods =
{
	.open = at9100_dci_root_ctrl_open,
	.close = at9100_dci_root_ctrl_close,
	.enter = at9100_dci_root_ctrl_enter,
	.start = at9100_dci_root_ctrl_start,
};

/*------------------------------------------------------------------------*
 * at9100 root interrupt support
 *------------------------------------------------------------------------*/
static void
at9100_dci_root_intr_open(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_root_intr_close(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;

	if (sc->sc_root_intr.xfer == xfer) {
		sc->sc_root_intr.xfer = NULL;
	}
	at9100_dci_device_done(xfer, USBD_ERR_CANCELLED);
	return;
}

static void
at9100_dci_root_intr_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
at9100_dci_root_intr_start(struct usbd_xfer *xfer)
{
	struct at9100_dci_softc *sc = xfer->usb_sc;

	sc->sc_root_intr.xfer = xfer;
	return;
}

struct usbd_pipe_methods at9100_dci_root_intr_methods =
{
	.open = at9100_dci_root_intr_open,
	.close = at9100_dci_root_intr_close,
	.enter = at9100_dci_root_intr_enter,
	.start = at9100_dci_root_intr_start,
};

static void
at9100_dci_xfer_setup(struct usbd_setup_params *parm)
{
	const struct usbd_hw_ep_profile *pf;
	struct at9100_dci_softc *sc;
	struct usbd_xfer *xfer;
	void *last_obj;
	uint32_t ntd;
	uint32_t n;
	uint8_t ep_no;

	sc = AT9100_DCI_BUS2SC(parm->udev->bus);
	xfer = parm->curr_xfer;

	/*
	 * setup xfer
	 */
	xfer->usb_sc = sc;

	/*
	 * NOTE: This driver does not use any of the parameters that
	 * are computed from the following values. Just set some
	 * reasonable dummies:
	 */
	parm->hc_max_packet_size = 0x500;
	parm->hc_max_packet_count = 1;
	parm->hc_max_frame_size = 0x500;

	usbd_transfer_setup_sub(parm);

	/*
	 * compute maximum number of TDs
	 */
	if (parm->methods == &at9100_dci_device_ctrl_methods) {

		ntd = xfer->nframes + 1 /* STATUS */ + 1 /* SYNC */ ;

	} else if (parm->methods == &at9100_dci_device_bulk_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else if (parm->methods == &at9100_dci_device_intr_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else if (parm->methods == &at9100_dci_device_isoc_fs_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else {

		ntd = 0;
	}

	/*
	 * check if "usbd_transfer_setup_sub" set an error
	 */
	if (parm->err) {
		return;
	}
	/*
	 * allocate transfer descriptors
	 */
	last_obj = NULL;

	/*
	 * get profile stuff
	 */
	if (ntd) {

		ep_no = xfer->endpoint & UE_ADDR;
		at9100_dci_get_hw_ep_profile(xfer->udev, &pf, ep_no);

		if (pf == NULL) {
			/* should not happen */
			parm->err = USBD_ERR_INVAL;
			return;
		}
	} else {
		ep_no = 0;
		pf = NULL;
	}

	/* align data */
	parm->size[0] += ((-parm->size[0]) & (USB_HOST_ALIGN - 1));

	for (n = 0; n != ntd; n++) {

		struct at9100_dci_td *td;

		if (parm->buf) {

			td = USBD_ADD_BYTES(parm->buf, parm->size[0]);

			/* init TD */
			td->io_tag = sc->sc_io_tag;
			td->io_hdl = sc->sc_io_hdl;
			td->max_packet_size = xfer->max_packet_size;
			td->status_reg = AT91_UDP_CSR(ep_no);
			td->fifo_reg = AT91_UDP_FDR(ep_no);
			if (pf->support_multi_buffer) {
				td->support_multi_buffer = 1;
			}
			td->obj_next = last_obj;

			last_obj = td;
		}
		parm->size[0] += sizeof(*td);
	}

	xfer->td_start[0] = last_obj;
	return;
}

static void
at9100_dci_xfer_unsetup(struct usbd_xfer *xfer)
{
	return;
}

static void
at9100_dci_pipe_init(struct usbd_device *udev, usb_endpoint_descriptor_t *edesc,
    struct usbd_pipe *pipe)
{
	struct at9100_dci_softc *sc = AT9100_DCI_BUS2SC(udev->bus);

	DPRINTFN(1, "pipe=%p, addr=%d, endpt=%d, mode=%d (%d)\n",
	    pipe, udev->address,
	    edesc->bEndpointAddress, udev->flags.usb_mode,
	    sc->sc_rt_addr);

	if (udev->address == sc->sc_rt_addr) {

		if (udev->flags.usb_mode != USB_MODE_HOST) {
			/* not supported */
			return;
		}
		switch (edesc->bEndpointAddress) {
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &at9100_dci_root_ctrl_methods;
			break;
		case UE_DIR_IN | AT9100_DCI_INTR_ENDPT:
			pipe->methods = &at9100_dci_root_intr_methods;
			break;
		default:
			/* do nothing */
			break;
		}
	} else {

		if (udev->flags.usb_mode != USB_MODE_DEVICE) {
			/* not supported */
			return;
		}
		if (udev->speed != USB_SPEED_FULL) {
			/* not supported */
			return;
		}
		switch (edesc->bmAttributes & UE_XFERTYPE) {
		case UE_CONTROL:
			pipe->methods = &at9100_dci_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &at9100_dci_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			pipe->methods = &at9100_dci_device_isoc_fs_methods;
			break;
		case UE_BULK:
			pipe->methods = &at9100_dci_device_bulk_methods;
			break;
		default:
			/* do nothing */
			break;
		}
	}
	return;
}

struct usbd_bus_methods at9100_dci_bus_methods =
{
	.pipe_init = &at9100_dci_pipe_init,
	.xfer_setup = &at9100_dci_xfer_setup,
	.xfer_unsetup = &at9100_dci_xfer_unsetup,
	.do_poll = &at9100_dci_do_poll,
	.set_config = &at9100_dci_set_config,
	.get_hw_ep_profile = &at9100_dci_get_hw_ep_profile,
	.set_stall = &at9100_dci_set_stall,
	.clear_stall = &at9100_dci_clear_stall,
	.vbus_interrupt = &at9100_dci_vbus_interrupt,
	.rem_wakeup_set = &at9100_dci_rem_wakeup_set,
};
