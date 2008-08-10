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
 * This file contains the driver for the Mentor Graphics Inventra USB
 * 2.0 High Speed Dual-Role controller.
 *
 * NOTE: The current implementation only supports Device Side Mode!
 */

#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_error.h>
#include <dev/usb2/include/usb2_defs.h>

#define	USB_DEBUG_VAR musbotgdebug
#define	usb2_config_td_cc musbotg_config_copy
#define	usb2_config_td_softc musbotg_softc

#include <dev/usb2/core/usb2_core.h>
#include <dev/usb2/core/usb2_debug.h>
#include <dev/usb2/core/usb2_busdma.h>
#include <dev/usb2/core/usb2_process.h>
#include <dev/usb2/core/usb2_config_td.h>
#include <dev/usb2/core/usb2_sw_transfer.h>
#include <dev/usb2/core/usb2_transfer.h>
#include <dev/usb2/core/usb2_device.h>
#include <dev/usb2/core/usb2_hub.h>
#include <dev/usb2/core/usb2_util.h>

#include <dev/usb2/controller/usb2_controller.h>
#include <dev/usb2/controller/usb2_bus.h>
#include <dev/usb2/controller/musb2_otg.h>

#define	MUSBOTG_INTR_ENDPT 1

#define	MUSBOTG_BUS2SC(bus) \
   ((struct musbotg_softc *)(((uint8_t *)(bus)) - \
   USB_P2U(&(((struct musbotg_softc *)0)->sc_bus))))

#define	MUSBOTG_PC2SC(pc) \
   MUSBOTG_BUS2SC((pc)->tag_parent->info->bus)

#ifdef USB_DEBUG
static int musbotgdebug = 0;

SYSCTL_NODE(_hw_usb2, OID_AUTO, musbotg, CTLFLAG_RW, 0, "USB musbotg");
SYSCTL_INT(_hw_usb2_musbotg, OID_AUTO, debug, CTLFLAG_RW,
    &musbotgdebug, 0, "Debug level");
#endif

/* prototypes */

struct usb2_bus_methods musbotg_bus_methods;
struct usb2_pipe_methods musbotg_device_bulk_methods;
struct usb2_pipe_methods musbotg_device_ctrl_methods;
struct usb2_pipe_methods musbotg_device_intr_methods;
struct usb2_pipe_methods musbotg_device_isoc_methods;
struct usb2_pipe_methods musbotg_root_ctrl_methods;
struct usb2_pipe_methods musbotg_root_intr_methods;

static musbotg_cmd_t musbotg_setup_rx;
static musbotg_cmd_t musbotg_setup_data_rx;
static musbotg_cmd_t musbotg_setup_data_tx;
static musbotg_cmd_t musbotg_setup_status;
static musbotg_cmd_t musbotg_data_rx;
static musbotg_cmd_t musbotg_data_tx;
static void musbotg_device_done(struct usb2_xfer *xfer, usb2_error_t error);
static void musbotg_do_poll(struct usb2_bus *bus);
static void musbotg_root_ctrl_poll(struct musbotg_softc *sc);
static void musbotg_standard_done(struct usb2_xfer *xfer);

static usb2_sw_transfer_func_t musbotg_root_intr_done;
static usb2_sw_transfer_func_t musbotg_root_ctrl_done;
static usb2_config_td_command_t musbotg_root_ctrl_task;

/*
 * Here is a configuration that the chip supports.
 */
static const struct usb2_hw_ep_profile musbotg_ep_profile[2] = {

	[0] = {
		.max_frame_size = 64,	/* fixed */
		.is_simplex = 1,
		.support_control = 1,
	},

	[1] = {
		.max_frame_size = (3 * 1024),
		.is_simplex = 0,	/* duplex */
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_isochronous = 1,
		.support_in = 1,
		.support_out = 1,
	},
};

static void
musbotg_get_hw_ep_profile(struct usb2_device *udev,
    const struct usb2_hw_ep_profile **ppf, uint8_t ep_addr)
{
	struct musbotg_softc *sc;

	sc = MUSBOTG_BUS2SC(udev->bus);

	if (ep_addr < sc->sc_ep_max) {
		if (ep_addr == 0) {
			/* control endpoint */
			*ppf = musbotg_ep_profile;
		} else {
			/* non-control endpoints */
			*ppf = musbotg_ep_profile + 1;
		}
	} else {
		*ppf = NULL;
	}
	return;
}

static void
musbotg_clocks_on(struct musbotg_softc *sc)
{
	if (sc->sc_flags.clocks_off &&
	    sc->sc_flags.port_powered) {

		DPRINTFN(4, "\n");

		if (sc->sc_clocks_on) {
			(sc->sc_clocks_on) (sc->sc_clocks_arg);
		}
		sc->sc_flags.clocks_off = 0;

		/* XXX enable Transceiver */
	}
	return;
}

static void
musbotg_clocks_off(struct musbotg_softc *sc)
{
	if (!sc->sc_flags.clocks_off) {

		DPRINTFN(4, "\n");

		/* XXX disable Transceiver */

		if (sc->sc_clocks_off) {
			(sc->sc_clocks_off) (sc->sc_clocks_arg);
		}
		sc->sc_flags.clocks_off = 1;
	}
	return;
}

static void
musbotg_pull_common(struct musbotg_softc *sc, uint8_t on)
{
	uint8_t temp;

	temp = MUSB2_READ_1(sc, MUSB2_REG_POWER);
	if (on)
		temp |= MUSB2_MASK_SOFTC;
	else
		temp &= ~MUSB2_MASK_SOFTC;

	MUSB2_WRITE_1(sc, MUSB2_REG_POWER, temp);
	return;
}

static void
musbotg_pull_up(struct musbotg_softc *sc)
{
	/* pullup D+, if possible */

	if (!sc->sc_flags.d_pulled_up &&
	    sc->sc_flags.port_powered) {
		sc->sc_flags.d_pulled_up = 1;
		musbotg_pull_common(sc, 1);
	}
	return;
}

static void
musbotg_pull_down(struct musbotg_softc *sc)
{
	/* pulldown D+, if possible */

	if (sc->sc_flags.d_pulled_up) {
		sc->sc_flags.d_pulled_up = 0;
		musbotg_pull_common(sc, 0);
	}
	return;
}

static void
musbotg_wakeup_peer(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc = xfer->usb2_sc;
	uint8_t temp;
	uint8_t use_polling;

	if (!(sc->sc_flags.status_suspend)) {
		return;
	}
	use_polling = mtx_owned(xfer->priv_mtx) ? 1 : 0;

	temp = MUSB2_READ_1(sc, MUSB2_REG_POWER);
	temp |= MUSB2_MASK_RESUME;
	MUSB2_WRITE_1(sc, MUSB2_REG_POWER, temp);

	/* wait 10 milliseconds */
	if (use_polling) {
		/* polling */
		DELAY(10000);
	} else {
		/* Wait for reset to complete. */
		if (usb2_config_td_sleep
		    (&(sc->sc_config_td),
		    (hz / 100))) {
			/* ignore */
		}
	}

	temp = MUSB2_READ_1(sc, MUSB2_REG_POWER);
	temp &= ~MUSB2_MASK_RESUME;
	MUSB2_WRITE_1(sc, MUSB2_REG_POWER, temp);
	return;
}

static void
musbotg_rem_wakeup_set(struct usb2_device *udev, uint8_t is_on)
{
	DPRINTFN(4, "is_on=%u\n", is_on);
	return;
}

static void
musbotg_set_address(struct musbotg_softc *sc, uint8_t addr)
{
	DPRINTFN(4, "addr=%d\n", addr);
	addr &= 0x7F;
	MUSB2_WRITE_1(sc, MUSB2_REG_FADDR, addr);
	return;
}

static uint8_t
musbotg_setup_rx(struct musbotg_td *td)
{
	struct musbotg_softc *sc;
	struct usb2_device_request req;
	uint16_t count;
	uint8_t csr;

	/* get pointer to softc */
	sc = MUSBOTG_PC2SC(td->pc);

	/* select endpoint 0 */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, 0);

	/* read out FIFO status */
	csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);

	DPRINTFN(4, "csr=0x%02x\n", csr);

	/*
	 * UNDOCUMENTED: If DATAEND is set we should not call the
	 * callback, hence the status stage is not complete.
	 */
	if (csr & MUSB2_MASK_CSR0L_DATAEND) {
		/* wait for interrupt */
		goto not_complete;
	}
	if (csr & MUSB2_MASK_CSR0L_SENTSTALL) {
		/* clear SENTSTALL */
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, 0);
		/* get latest status */
		csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);
		/* update EP0 state */
		sc->sc_ep0_busy = 0;
	}
	if (csr & MUSB2_MASK_CSR0L_SETUPEND) {
		/* clear SETUPEND */
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
		    MUSB2_MASK_CSR0L_SETUPEND_CLR);
		/* get latest status */
		csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);
		/* update EP0 state */
		sc->sc_ep0_busy = 0;
	}
	if (sc->sc_ep0_busy) {
		/* abort any ongoing transfer */
		if (!td->did_stall) {
			DPRINTFN(4, "stalling\n");
			MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
			    MUSB2_MASK_CSR0L_SENDSTALL);
			td->did_stall = 1;
		}
		goto not_complete;
	}
	if (!(csr & MUSB2_MASK_CSR0L_RXPKTRDY)) {
		goto not_complete;
	}
	/* get the packet byte count */
	count = MUSB2_READ_2(sc, MUSB2_REG_RXCOUNT);

	/* verify data length */
	if (count != td->remainder) {
		DPRINTFN(0, "Invalid SETUP packet "
		    "length, %d bytes\n", count);
		goto not_complete;
	}
	if (count != sizeof(req)) {
		DPRINTFN(0, "Unsupported SETUP packet "
		    "length, %d bytes\n", count);
		goto not_complete;
	}
	/* receive data */
	bus_space_read_multi_1(sc->sc_io_tag, sc->sc_io_hdl,
	    MUSB2_REG_EPFIFO(0), (void *)&req, sizeof(req));

	/* copy data into real buffer */
	usb2_copy_in(td->pc, 0, &req, sizeof(req));

	td->offset = sizeof(req);
	td->remainder = 0;

	/* set pending command */
	sc->sc_ep0_cmd = MUSB2_MASK_CSR0L_RXPKTRDY_CLR;

	/* we need set stall or dataend after this */
	sc->sc_ep0_busy = 1;

	/* sneak peek the set address */
	if ((req.bmRequestType == UT_WRITE_DEVICE) &&
	    (req.bRequest == UR_SET_ADDRESS)) {
		sc->sc_dv_addr = req.wValue[0] & 0x7F;
	} else {
		sc->sc_dv_addr = 0xFF;
	}
	return (0);			/* complete */

not_complete:
	return (1);			/* not complete */
}

/* Control endpoint only data handling functions (RX/TX/SYNC) */

static uint8_t
musbotg_setup_data_rx(struct musbotg_td *td)
{
	struct usb2_page_search buf_res;
	struct musbotg_softc *sc;
	uint16_t count;
	uint8_t csr;
	uint8_t got_short;

	/* get pointer to softc */
	sc = MUSBOTG_PC2SC(td->pc);

	/* select endpoint 0 */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, 0);

	/* check if a command is pending */
	if (sc->sc_ep0_cmd) {
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, sc->sc_ep0_cmd);
		sc->sc_ep0_cmd = 0;
	}
	/* read out FIFO status */
	csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);

	DPRINTFN(4, "csr=0x%02x\n", csr);

	got_short = 0;

	if (csr & (MUSB2_MASK_CSR0L_SETUPEND |
	    MUSB2_MASK_CSR0L_SENTSTALL)) {
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
	if (!(csr & MUSB2_MASK_CSR0L_RXPKTRDY)) {
		return (1);		/* not complete */
	}
	/* get the packet byte count */
	count = MUSB2_READ_2(sc, MUSB2_REG_RXCOUNT);

	/* verify the packet byte count */
	if (count != td->max_frame_size) {
		if (count < td->max_frame_size) {
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
		usb2_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* receive data */
		bus_space_read_multi_1(sc->sc_io_tag, sc->sc_io_hdl,
		    MUSB2_REG_EPFIFO(0), buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* check if we are complete */
	if ((td->remainder == 0) || got_short) {
		if (td->short_pkt) {
			/* we are complete */
			sc->sc_ep0_cmd = MUSB2_MASK_CSR0L_RXPKTRDY_CLR;
			return (0);
		}
		/* else need to receive a zero length packet */
	}
	/* write command - need more data */
	MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
	    MUSB2_MASK_CSR0L_RXPKTRDY_CLR);
	return (1);			/* not complete */
}

static uint8_t
musbotg_setup_data_tx(struct musbotg_td *td)
{
	struct usb2_page_search buf_res;
	struct musbotg_softc *sc;
	uint16_t count;
	uint8_t csr;

	/* get pointer to softc */
	sc = MUSBOTG_PC2SC(td->pc);

	/* select endpoint 0 */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, 0);

	/* check if a command is pending */
	if (sc->sc_ep0_cmd) {
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, sc->sc_ep0_cmd);
		sc->sc_ep0_cmd = 0;
	}
	/* read out FIFO status */
	csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);

	DPRINTFN(4, "csr=0x%02x\n", csr);

	if (csr & (MUSB2_MASK_CSR0L_SETUPEND |
	    MUSB2_MASK_CSR0L_SENTSTALL)) {
		/*
	         * The current transfer was aborted
	         * by the USB Host
	         */
		td->error = 1;
		return (0);		/* complete */
	}
	if (!(csr & MUSB2_MASK_CSR0L_TXPKTRDY)) {
		return (1);		/* not complete */
	}
	count = td->max_frame_size;
	if (td->remainder < count) {
		/* we have a short packet */
		td->short_pkt = 1;
		count = td->remainder;
	}
	while (count > 0) {

		usb2_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* transmit data */
		bus_space_write_multi_1(sc->sc_io_tag, sc->sc_io_hdl,
		    MUSB2_REG_EPFIFO(0), buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* check remainder */
	if (td->remainder == 0) {
		if (td->short_pkt) {
			sc->sc_ep0_cmd = MUSB2_MASK_CSR0L_TXPKTRDY;
			return (0);	/* complete */
		}
		/* else we need to transmit a short packet */
	}
	/* write command */
	MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
	    MUSB2_MASK_CSR0L_TXPKTRDY);

	return (1);			/* not complete */
}

static uint8_t
musbotg_setup_status(struct musbotg_td *td)
{
	struct musbotg_softc *sc;
	uint8_t csr;

	/* get pointer to softc */
	sc = MUSBOTG_PC2SC(td->pc);

	/* select endpoint 0 */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, 0);

	if (sc->sc_ep0_busy) {
		sc->sc_ep0_busy = 0;
		sc->sc_ep0_cmd |= MUSB2_MASK_CSR0L_DATAEND;
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, sc->sc_ep0_cmd);
		sc->sc_ep0_cmd = 0;
	}
	/* read out FIFO status */
	csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);

	DPRINTFN(4, "csr=0x%02x\n", csr);

	if (csr & MUSB2_MASK_CSR0L_DATAEND) {
		/* wait for interrupt */
		return (1);		/* not complete */
	}
	if (sc->sc_dv_addr != 0xFF) {
		/* write function address */
		musbotg_set_address(sc, sc->sc_dv_addr);
	}
	return (0);			/* complete */
}

static uint8_t
musbotg_data_rx(struct musbotg_td *td)
{
	struct usb2_page_search buf_res;
	struct musbotg_softc *sc;
	uint16_t count;
	uint8_t csr;
	uint8_t to;
	uint8_t got_short;

	to = 2;				/* don't loop forever! */
	got_short = 0;

	/* get pointer to softc */
	sc = MUSBOTG_PC2SC(td->pc);

	/* select endpoint */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, td->ep_no);

repeat:
	/* read out FIFO status */
	csr = MUSB2_READ_1(sc, MUSB2_REG_RXCSRL);

	DPRINTFN(4, "csr=0x%02x\n", csr);

	/* clear overrun */
	if (csr & MUSB2_MASK_CSRL_RXOVERRUN) {
		/* make sure we don't clear "RXPKTRDY" */
		MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL,
		    MUSB2_MASK_CSRL_RXPKTRDY);
	}
	/* check status */
	if (!(csr & MUSB2_MASK_CSRL_RXPKTRDY)) {
		return (1);		/* not complete */
	}
	/* get the packet byte count */
	count = MUSB2_READ_2(sc, MUSB2_REG_RXCOUNT);

	/* verify the packet byte count */
	if (count != td->max_frame_size) {
		if (count < td->max_frame_size) {
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
		usb2_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* receive data */
		bus_space_read_multi_1(sc->sc_io_tag, sc->sc_io_hdl,
		    MUSB2_REG_EPFIFO(td->ep_no), buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* clear status bits */
	MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL, 0);

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
musbotg_data_tx(struct musbotg_td *td)
{
	struct usb2_page_search buf_res;
	struct musbotg_softc *sc;
	uint16_t count;
	uint8_t csr;
	uint8_t to;

	to = 2;				/* don't loop forever! */

	/* get pointer to softc */
	sc = MUSBOTG_PC2SC(td->pc);

	/* select endpoint */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, td->ep_no);

repeat:

	/* read out FIFO status */
	csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);

	DPRINTFN(4, "csr=0x%02x\n", csr);

	if (csr & (MUSB2_MASK_CSRL_TXINCOMP |
	    MUSB2_MASK_CSRL_TXUNDERRUN)) {
		/* clear status bits */
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, 0);
	}
	if (csr & MUSB2_MASK_CSRL_TXPKTRDY) {
		return (1);		/* not complete */
	}
	count = td->max_frame_size;
	if (td->remainder < count) {
		/* we have a short packet */
		td->short_pkt = 1;
		count = td->remainder;
	}
	while (count > 0) {

		usb2_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* transmit data */
		bus_space_write_multi_1(sc->sc_io_tag, sc->sc_io_hdl,
		    MUSB2_REG_EPFIFO(td->ep_no), buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* write command */
	MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
	    MUSB2_MASK_CSRL_TXPKTRDY);

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
musbotg_xfer_do_fifo(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc;
	struct musbotg_td *td;

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
		td = td->obj_next;
		xfer->td_transfer_cache = td;
	}
	return (1);			/* not complete */

done:
	sc = xfer->usb2_sc;

	/* compute all actual lengths */

	musbotg_standard_done(xfer);

	return (0);			/* complete */
}

static void
musbotg_interrupt_poll(struct musbotg_softc *sc)
{
	struct usb2_xfer *xfer;

repeat:
	TAILQ_FOREACH(xfer, &(sc->sc_bus.intr_q.head), wait_entry) {
		if (!musbotg_xfer_do_fifo(xfer)) {
			/* queue has been modified */
			goto repeat;
		}
	}
	return;
}

static void
musbotg_vbus_interrupt(struct usb2_bus *bus, uint8_t is_on)
{
	struct musbotg_softc *sc = MUSBOTG_BUS2SC(bus);

	DPRINTFN(4, "vbus = %u\n", is_on);

	mtx_lock(&(sc->sc_bus.mtx));
	if (is_on) {
		if (!sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 1;

			/* complete root HUB interrupt endpoint */

			usb2_sw_transfer(&(sc->sc_root_intr),
			    &musbotg_root_intr_done);
		}
	} else {
		if (sc->sc_flags.status_vbus) {
			sc->sc_flags.status_vbus = 0;
			sc->sc_flags.status_bus_reset = 0;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;

			/* complete root HUB interrupt endpoint */

			usb2_sw_transfer(&(sc->sc_root_intr),
			    &musbotg_root_intr_done);
		}
	}

	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

void
musbotg_interrupt(struct musbotg_softc *sc)
{
	uint8_t usb_status;
	uint8_t rx_status;
	uint8_t tx_status;
	uint8_t temp;

	mtx_lock(&(sc->sc_bus.mtx));

	/* read all interrupt registers */
	usb_status = MUSB2_READ_1(sc, MUSB2_REG_INTUSB);

	/* read all FIFO interrupts */
	rx_status = MUSB2_READ_2(sc, MUSB2_REG_INTRX);
	tx_status = MUSB2_READ_2(sc, MUSB2_REG_INTTX);

	/* check for any bus state change interrupts */

	if (usb_status & (MUSB2_MASK_IRESET |
	    MUSB2_MASK_IRESUME | MUSB2_MASK_ISUSP)) {

		DPRINTFN(4, "real bus interrupt 0x%08x\n", usb_status);

		if (usb_status & MUSB2_MASK_IRESET) {

			/* set correct state */
			sc->sc_flags.status_bus_reset = 1;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;

			/* determine line speed */
			temp = MUSB2_READ_1(sc, MUSB2_REG_POWER);
			if (temp & MUSB2_MASK_HSMODE)
				sc->sc_flags.status_high_speed = 1;
			else
				sc->sc_flags.status_high_speed = 0;

			temp = MUSB2_READ_1(sc, MUSB2_REG_INTUSBE);
			/* disable resume interrupt */
			temp &= ~MUSB2_MASK_IRESUME;
			/* enable suspend interrupt */
			temp |= MUSB2_MASK_ISUSP;
			MUSB2_WRITE_1(sc, MUSB2_REG_INTUSBE, temp);
		}
		/*
	         * If RXRSM and RXSUSP is set at the same time we interpret
	         * that like RESUME. Resume is set when there is at least 3
	         * milliseconds of inactivity on the USB BUS.
	         */
		if (usb_status & MUSB2_MASK_IRESUME) {
			if (sc->sc_flags.status_suspend) {
				sc->sc_flags.status_suspend = 0;
				sc->sc_flags.change_suspend = 1;

				temp = MUSB2_READ_1(sc, MUSB2_REG_INTUSBE);
				/* disable resume interrupt */
				temp &= ~MUSB2_MASK_IRESUME;
				/* enable suspend interrupt */
				temp |= MUSB2_MASK_ISUSP;
				MUSB2_WRITE_1(sc, MUSB2_REG_INTUSBE, temp);
			}
		} else if (usb_status & MUSB2_MASK_ISUSP) {
			if (!sc->sc_flags.status_suspend) {
				sc->sc_flags.status_suspend = 1;
				sc->sc_flags.change_suspend = 1;

				temp = MUSB2_READ_1(sc, MUSB2_REG_INTUSBE);
				/* disable suspend interrupt */
				temp &= ~MUSB2_MASK_ISUSP;
				/* enable resume interrupt */
				temp |= MUSB2_MASK_IRESUME;
				MUSB2_WRITE_1(sc, MUSB2_REG_INTUSBE, temp);
			}
		}
		/* complete root HUB interrupt endpoint */

		usb2_sw_transfer(&(sc->sc_root_intr),
		    &musbotg_root_intr_done);
	}
	/* check for any endpoint interrupts */

	if (rx_status || tx_status) {

		DPRINTFN(4, "real endpoint interrupt "
		    "rx=0x%04x, tx=0x%04x\n", rx_status, tx_status);

		musbotg_interrupt_poll(sc);
	}
	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

static void
musbotg_setup_standard_chain_sub(struct musbotg_std_temp *temp)
{
	struct musbotg_td *td;

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
	td->error = 0;
	td->did_stall = 0;
	td->short_pkt = temp->short_pkt;
	td->alt_next = temp->setup_alt_next;
	return;
}

static void
musbotg_setup_standard_chain(struct usb2_xfer *xfer)
{
	struct musbotg_std_temp temp;
	struct musbotg_softc *sc;
	struct musbotg_td *td;
	uint32_t x;
	uint8_t ep_no;

	DPRINTFN(8, "addr=%d endpt=%d sumlen=%d speed=%d\n",
	    xfer->address, UE_GET_ADDR(xfer->endpoint),
	    xfer->sumlen, usb2_get_speed(xfer->udev));

	temp.max_frame_size = xfer->max_frame_size;

	td = xfer->td_start[0];
	xfer->td_transfer_first = td;
	xfer->td_transfer_cache = td;

	/* setup temp */

	temp.td = NULL;
	temp.td_next = xfer->td_start[0];
	temp.setup_alt_next = xfer->flags_int.short_frames_ok;
	temp.offset = 0;

	sc = xfer->usb2_sc;
	ep_no = (xfer->endpoint & UE_ADDR);

	/* check if we should prepend a setup message */

	if (xfer->flags_int.control_xfr) {
		if (xfer->flags_int.control_hdr) {

			temp.func = &musbotg_setup_rx;
			temp.len = xfer->frlengths[0];
			temp.pc = xfer->frbuffers + 0;
			temp.short_pkt = temp.len ? 1 : 0;

			musbotg_setup_standard_chain_sub(&temp);
		}
		x = 1;
	} else {
		x = 0;
	}

	if (x != xfer->nframes) {
		if (xfer->endpoint & UE_DIR_IN) {
			if (xfer->flags_int.control_xfr)
				temp.func = &musbotg_setup_data_tx;
			else
				temp.func = &musbotg_data_tx;
		} else {
			if (xfer->flags_int.control_xfr)
				temp.func = &musbotg_setup_data_rx;
			else
				temp.func = &musbotg_data_rx;
		}

		/* setup "pc" pointer */
		temp.pc = xfer->frbuffers + x;
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

		musbotg_setup_standard_chain_sub(&temp);

		if (xfer->flags_int.isochronous_xfr) {
			temp.offset += temp.len;
		} else {
			/* get next Page Cache pointer */
			temp.pc = xfer->frbuffers + x;
		}
	}

	/* always setup a valid "pc" pointer for status and sync */
	temp.pc = xfer->frbuffers + 0;

	/* check if we should append a status stage */

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		/*
		 * Send a DATA1 message and invert the current
		 * endpoint direction.
		 */
		temp.func = &musbotg_setup_status;
		temp.len = 0;
		temp.short_pkt = 0;

		musbotg_setup_standard_chain_sub(&temp);
	}
	/* must have at least one frame! */
	td = temp.td;
	xfer->td_transfer_last = td;
	return;
}

static void
musbotg_timeout(void *arg)
{
	struct usb2_xfer *xfer = arg;
	struct musbotg_softc *sc = xfer->usb2_sc;

	DPRINTFN(1, "xfer=%p\n", xfer);

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	musbotg_device_done(xfer, USB_ERR_TIMEOUT);

	mtx_unlock(&sc->sc_bus.mtx);

	return;
}

static void
musbotg_ep_int_set(struct usb2_xfer *xfer, uint8_t on)
{
	struct musbotg_softc *sc = xfer->usb2_sc;
	uint16_t temp;
	uint8_t ep_no = xfer->endpoint & UE_ADDR;

	/*
	 * Only enable the endpoint interrupt when we are
	 * actually waiting for data, hence we are dealing
	 * with level triggered interrupts !
	 */
	if (ep_no == 0) {
		temp = MUSB2_READ_2(sc, MUSB2_REG_INTTXE);
		if (on)
			temp |= MUSB2_MASK_EPINT(0);
		else
			temp &= ~MUSB2_MASK_EPINT(0);

		MUSB2_WRITE_2(sc, MUSB2_REG_INTTXE, temp);
	} else {
		if (USB_GET_DATA_ISREAD(xfer)) {
			temp = MUSB2_READ_2(sc, MUSB2_REG_INTRXE);
			if (on)
				temp |= MUSB2_MASK_EPINT(ep_no);
			else
				temp &= ~MUSB2_MASK_EPINT(ep_no);
			MUSB2_WRITE_2(sc, MUSB2_REG_INTRXE, temp);
		} else {
			temp = MUSB2_READ_2(sc, MUSB2_REG_INTTXE);
			if (on)
				temp |= MUSB2_MASK_EPINT(ep_no);
			else
				temp &= ~MUSB2_MASK_EPINT(ep_no);
			MUSB2_WRITE_2(sc, MUSB2_REG_INTTXE, temp);
		}
	}
	return;
}

static void
musbotg_start_standard_chain(struct usb2_xfer *xfer)
{
	DPRINTFN(8, "\n");

	/* poll one time */
	if (musbotg_xfer_do_fifo(xfer)) {

		musbotg_ep_int_set(xfer, 1);

		DPRINTFN(14, "enabled interrupts on endpoint\n");

		/* put transfer on interrupt queue */
		usb2_transfer_enqueue(&(xfer->udev->bus->intr_q), xfer);

		/* start timeout, if any */
		if (xfer->timeout != 0) {
			usb2_transfer_timeout_ms(xfer,
			    &musbotg_timeout, xfer->timeout);
		}
	}
	return;
}

static void
musbotg_root_intr_done(struct usb2_xfer *xfer,
    struct usb2_sw_transfer *std)
{
	struct musbotg_softc *sc = xfer->usb2_sc;

	DPRINTFN(8, "\n");

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USB_SW_TR_PRE_DATA) {
		if (std->state == USB_SW_TR_PRE_CALLBACK) {
			/* transfer transferred */
			musbotg_device_done(xfer, std->err);
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

static usb2_error_t
musbotg_standard_done_sub(struct usb2_xfer *xfer)
{
	struct musbotg_td *td;
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
	    USB_ERR_STALLED : USB_ERR_NORMAL_COMPLETION);
}

static void
musbotg_standard_done(struct usb2_xfer *xfer)
{
	usb2_error_t err = 0;

	DPRINTFN(12, "xfer=%p pipe=%p transfer done\n",
	    xfer, xfer->pipe);

	/* reset scanner */

	xfer->td_transfer_cache = xfer->td_transfer_first;

	if (xfer->flags_int.control_xfr) {

		if (xfer->flags_int.control_hdr) {

			err = musbotg_standard_done_sub(xfer);
		}
		xfer->aframes = 1;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}
	while (xfer->aframes != xfer->nframes) {

		err = musbotg_standard_done_sub(xfer);
		xfer->aframes++;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		err = musbotg_standard_done_sub(xfer);
	}
done:
	musbotg_device_done(xfer, err);
	return;
}

/*------------------------------------------------------------------------*
 *	musbotg_device_done
 *
 * NOTE: this function can be called more than one time on the
 * same USB transfer!
 *------------------------------------------------------------------------*/
static void
musbotg_device_done(struct usb2_xfer *xfer, usb2_error_t error)
{
	mtx_assert(xfer->usb2_mtx, MA_OWNED);

	DPRINTFN(2, "xfer=%p, pipe=%p, error=%d\n",
	    xfer, xfer->pipe, error);

	if (xfer->flags_int.usb2_mode == USB_MODE_DEVICE) {

		musbotg_ep_int_set(xfer, 0);

		DPRINTFN(14, "disabled interrupts on endpoint\n");
	}
	/* dequeue transfer and start next transfer */
	usb2_transfer_done(xfer, error);
	return;
}

static void
musbotg_set_stall(struct usb2_device *udev, struct usb2_xfer *xfer,
    struct usb2_pipe *pipe)
{
	struct musbotg_softc *sc;
	uint8_t ep_no;

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	DPRINTFN(4, "pipe=%p\n", pipe);

	if (xfer) {
		/* cancel any ongoing transfers */
		musbotg_device_done(xfer, USB_ERR_STALLED);
	}
	/* set FORCESTALL */
	sc = MUSBOTG_BUS2SC(udev->bus);

	ep_no = (pipe->edesc->bEndpointAddress & UE_ADDR);

	/* select endpoint */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, ep_no);

	if (pipe->edesc->bEndpointAddress & UE_DIR_IN) {
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
		    MUSB2_MASK_CSRL_TXSENDSTALL);
	} else {
		MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL,
		    MUSB2_MASK_CSRL_RXSENDSTALL);
	}
	return;
}

static void
musbotg_clear_stall_sub(struct musbotg_softc *sc, uint16_t wMaxPacket,
    uint8_t ep_no, uint8_t ep_type, uint8_t ep_dir)
{
	uint8_t csr;

	if (ep_type == UE_CONTROL) {
		/* clearing stall is not needed */
		return;
	}
	/* select endpoint */
	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, ep_no);

	if (ep_dir == UE_DIR_IN) {

		/* Configure endpoint */
		switch (ep_type) {
		case UE_INTERRUPT:
			MUSB2_WRITE_1(sc, MUSB2_REG_TXMAXP, wMaxPacket);
			MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRH,
			    MUSB2_MASK_CSRH_TXMODE);
			break;
		case UE_ISOCHRONOUS:
			MUSB2_WRITE_1(sc, MUSB2_REG_TXMAXP, wMaxPacket);
			MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRH,
			    MUSB2_MASK_CSRH_TXMODE |
			    MUSB2_MASK_CSRH_TXISO);
			break;
		case UE_BULK:
			MUSB2_WRITE_1(sc, MUSB2_REG_TXMAXP, wMaxPacket);
			MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRH,
			    MUSB2_MASK_CSRH_TXMODE);
			break;
		default:
			break;
		}

		/* Need to flush twice in case of double bufring */
		csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);
		if (csr & MUSB2_MASK_CSRL_TXFIFONEMPTY) {
			MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
			    MUSB2_MASK_CSRL_TXFFLUSH);
			csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);
			if (csr & MUSB2_MASK_CSRL_TXFIFONEMPTY) {
				MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
				    MUSB2_MASK_CSRL_TXFFLUSH);
				csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);
			}
		}
		/* reset data toggle */
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL,
		    MUSB2_MASK_CSRL_TXDT_CLR);
		MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, 0);
		csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);

		/* clear sent stall */
		if (csr & MUSB2_MASK_CSRL_TXSENTSTALL) {
			MUSB2_WRITE_1(sc, MUSB2_REG_TXCSRL, 0);
			csr = MUSB2_READ_1(sc, MUSB2_REG_TXCSRL);
		}
	} else {

		/* Configure endpoint */
		switch (ep_type) {
		case UE_INTERRUPT:
			MUSB2_WRITE_1(sc, MUSB2_REG_RXMAXP, wMaxPacket);
			MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRH,
			    MUSB2_MASK_CSRH_RXNYET);
			break;
		case UE_ISOCHRONOUS:
			MUSB2_WRITE_1(sc, MUSB2_REG_RXMAXP, wMaxPacket);
			MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRH,
			    MUSB2_MASK_CSRH_RXNYET |
			    MUSB2_MASK_CSRH_RXISO);
			break;
		case UE_BULK:
			MUSB2_WRITE_1(sc, MUSB2_REG_RXMAXP, wMaxPacket);
			MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRH, 0);
			break;
		default:
			break;
		}

		/* Need to flush twice in case of double bufring */

		csr = MUSB2_READ_1(sc, MUSB2_REG_RXCSRL);
		if (csr & MUSB2_MASK_CSRL_RXPKTRDY) {
			MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL,
			    MUSB2_MASK_CSRL_RXFFLUSH);
			csr = MUSB2_READ_1(sc, MUSB2_REG_RXCSRL);
			if (csr & MUSB2_MASK_CSRL_RXPKTRDY) {
				MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL,
				    MUSB2_MASK_CSRL_RXFFLUSH);
				csr = MUSB2_READ_1(sc, MUSB2_REG_RXCSRL);
			}
		}
		/* reset data toggle */
		MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL,
		    MUSB2_MASK_CSRL_RXDT_CLR);
		MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL, 0);
		csr = MUSB2_READ_1(sc, MUSB2_REG_RXCSRL);

		/* clear sent stall */
		if (csr & MUSB2_MASK_CSRL_RXSENTSTALL) {
			MUSB2_WRITE_1(sc, MUSB2_REG_RXCSRL, 0);
		}
	}
	return;
}

static void
musbotg_clear_stall(struct usb2_device *udev, struct usb2_pipe *pipe)
{
	struct musbotg_softc *sc;
	struct usb2_endpoint_descriptor *ed;

	DPRINTFN(4, "pipe=%p\n", pipe);

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	/* check mode */
	if (udev->flags.usb2_mode != USB_MODE_DEVICE) {
		/* not supported */
		return;
	}
	/* get softc */
	sc = MUSBOTG_BUS2SC(udev->bus);

	/* get endpoint descriptor */
	ed = pipe->edesc;

	/* reset endpoint */
	musbotg_clear_stall_sub(sc,
	    UGETW(ed->wMaxPacketSize),
	    (ed->bEndpointAddress & UE_ADDR),
	    (ed->bmAttributes & UE_XFERTYPE),
	    (ed->bEndpointAddress & (UE_DIR_IN | UE_DIR_OUT)));
	return;
}

usb2_error_t
musbotg_init(struct musbotg_softc *sc)
{
	uint8_t nrx;
	uint8_t ntx;

	DPRINTFN(1, "start\n");

	/* set up the bus structure */
	sc->sc_bus.usbrev = USB_REV_2_0;
	sc->sc_bus.methods = &musbotg_bus_methods;

	mtx_lock(&(sc->sc_bus.mtx));

	/* turn on clocks */

	if (sc->sc_clocks_on) {
		(sc->sc_clocks_on) (sc->sc_clocks_arg);
	}
	/* wait a little for things to stabilise */
	DELAY(1000);

	/* disable and clear all interrupts */

	MUSB2_WRITE_1(sc, MUSB2_REG_INTUSBE, 0);
	MUSB2_WRITE_2(sc, MUSB2_REG_INTTXE, 0);
	MUSB2_WRITE_2(sc, MUSB2_REG_INTRXE, 0);

	if (MUSB2_READ_1(sc, MUSB2_REG_INTUSB)) {
		/* ignore */
	}
	if (MUSB2_READ_2(sc, MUSB2_REG_INTTX)) {
		/* ignore */
	}
	if (MUSB2_READ_2(sc, MUSB2_REG_INTRX)) {
		/* ignore */
	}
	/* disable pullup */

	musbotg_pull_common(sc, 0);

	/* wait a little bit (10ms) */

	DELAY(10000);

	/* enable double packet buffering */

	MUSB2_WRITE_2(sc, MUSB2_REG_RXDBDIS, 0);
	MUSB2_WRITE_2(sc, MUSB2_REG_TXDBDIS, 0);

	/* enable HighSpeed and ISO Update flags */

	MUSB2_WRITE_1(sc, MUSB2_REG_POWER,
	    MUSB2_MASK_HSENAB | MUSB2_MASK_ISOUPD);

	/* disable testmode */

	MUSB2_WRITE_1(sc, MUSB2_REG_TESTMODE, 0);

	/* set default value */

	MUSB2_WRITE_1(sc, MUSB2_REG_MISC, 0);

	/* select endpoint index 0 */

	MUSB2_WRITE_1(sc, MUSB2_REG_EPINDEX, 0);

	/* read out number of endpoints */

	nrx =
	    (MUSB2_READ_1(sc, MUSB2_REG_EPINFO) / 16);

	ntx =
	    (MUSB2_READ_1(sc, MUSB2_REG_EPINFO) % 16);

	DPRINTFN(2, "RX/TX endpoints: %u/%u\n", nrx, ntx);

	sc->sc_ep_max = (nrx < ntx) ? nrx : ntx;

	/* read out configuration data */

	sc->sc_conf_data = MUSB2_READ_1(sc, MUSB2_REG_CONFDATA);

	DPRINTFN(2, "Config Data: 0x%02x\n",
	    sc->sc_conf_data);

	DPRINTFN(2, "HW version: 0x%04x\n",
	    MUSB2_READ_1(sc, MUSB2_REG_HWVERS));

	/* turn on default interrupts */

	MUSB2_WRITE_1(sc, MUSB2_REG_INTUSBE,
	    MUSB2_MASK_IRESET);

	musbotg_clocks_off(sc);

	mtx_unlock(&(sc->sc_bus.mtx));

	/* catch any lost interrupts */

	musbotg_do_poll(&(sc->sc_bus));

	return (0);			/* success */
}

void
musbotg_uninit(struct musbotg_softc *sc)
{
	mtx_lock(&(sc->sc_bus.mtx));

	/* disable all interrupts */
	MUSB2_WRITE_1(sc, MUSB2_REG_INTUSBE, 0);
	MUSB2_WRITE_2(sc, MUSB2_REG_INTTXE, 0);
	MUSB2_WRITE_2(sc, MUSB2_REG_INTRXE, 0);

	sc->sc_flags.port_powered = 0;
	sc->sc_flags.status_vbus = 0;
	sc->sc_flags.status_bus_reset = 0;
	sc->sc_flags.status_suspend = 0;
	sc->sc_flags.change_suspend = 0;
	sc->sc_flags.change_connect = 1;

	musbotg_pull_down(sc);
	musbotg_clocks_off(sc);
	mtx_unlock(&(sc->sc_bus.mtx));
	return;
}

void
musbotg_suspend(struct musbotg_softc *sc)
{
	return;
}

void
musbotg_resume(struct musbotg_softc *sc)
{
	return;
}

static void
musbotg_do_poll(struct usb2_bus *bus)
{
	struct musbotg_softc *sc = MUSBOTG_BUS2SC(bus);

	mtx_lock(&(sc->sc_bus.mtx));
	musbotg_interrupt_poll(sc);
	musbotg_root_ctrl_poll(sc);
	mtx_unlock(&(sc->sc_bus.mtx));
	return;
}

/*------------------------------------------------------------------------*
 * musbotg bulk support
 *------------------------------------------------------------------------*/
static void
musbotg_device_bulk_open(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_bulk_close(struct usb2_xfer *xfer)
{
	musbotg_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
musbotg_device_bulk_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_bulk_start(struct usb2_xfer *xfer)
{
	/* setup TDs */
	musbotg_setup_standard_chain(xfer);
	musbotg_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods musbotg_device_bulk_methods =
{
	.open = musbotg_device_bulk_open,
	.close = musbotg_device_bulk_close,
	.enter = musbotg_device_bulk_enter,
	.start = musbotg_device_bulk_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * musbotg control support
 *------------------------------------------------------------------------*/
static void
musbotg_device_ctrl_open(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_ctrl_close(struct usb2_xfer *xfer)
{
	musbotg_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
musbotg_device_ctrl_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_ctrl_start(struct usb2_xfer *xfer)
{
	/* setup TDs */
	musbotg_setup_standard_chain(xfer);
	musbotg_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods musbotg_device_ctrl_methods =
{
	.open = musbotg_device_ctrl_open,
	.close = musbotg_device_ctrl_close,
	.enter = musbotg_device_ctrl_enter,
	.start = musbotg_device_ctrl_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * musbotg interrupt support
 *------------------------------------------------------------------------*/
static void
musbotg_device_intr_open(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_intr_close(struct usb2_xfer *xfer)
{
	musbotg_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
musbotg_device_intr_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_intr_start(struct usb2_xfer *xfer)
{
	/* setup TDs */
	musbotg_setup_standard_chain(xfer);
	musbotg_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods musbotg_device_intr_methods =
{
	.open = musbotg_device_intr_open,
	.close = musbotg_device_intr_close,
	.enter = musbotg_device_intr_enter,
	.start = musbotg_device_intr_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * musbotg full speed isochronous support
 *------------------------------------------------------------------------*/
static void
musbotg_device_isoc_open(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_device_isoc_close(struct usb2_xfer *xfer)
{
	musbotg_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
musbotg_device_isoc_enter(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc = xfer->usb2_sc;
	uint32_t temp;
	uint32_t nframes;
	uint32_t fs_frames;

	DPRINTFN(5, "xfer=%p next=%d nframes=%d\n",
	    xfer, xfer->pipe->isoc_next, xfer->nframes);

	/* get the current frame index */

	nframes = MUSB2_READ_2(sc, MUSB2_REG_FRAME);

	/*
	 * check if the frame index is within the window where the frames
	 * will be inserted
	 */
	temp = (nframes - xfer->pipe->isoc_next) & MUSB2_MASK_FRAME;

	if (usb2_get_speed(xfer->udev) == USB_SPEED_HIGH) {
		fs_frames = (xfer->nframes + 7) / 8;
	} else {
		fs_frames = xfer->nframes;
	}

	if ((xfer->pipe->is_synced == 0) ||
	    (temp < fs_frames)) {
		/*
		 * If there is data underflow or the pipe queue is
		 * empty we schedule the transfer a few frames ahead
		 * of the current frame position. Else two isochronous
		 * transfers might overlap.
		 */
		xfer->pipe->isoc_next = (nframes + 3) & MUSB2_MASK_FRAME;
		xfer->pipe->is_synced = 1;
		DPRINTFN(2, "start next=%d\n", xfer->pipe->isoc_next);
	}
	/*
	 * compute how many milliseconds the insertion is ahead of the
	 * current frame position:
	 */
	temp = (xfer->pipe->isoc_next - nframes) & MUSB2_MASK_FRAME;

	/*
	 * pre-compute when the isochronous transfer will be finished:
	 */
	xfer->isoc_time_complete =
	    usb2_isoc_time_expand(&(sc->sc_bus), nframes) + temp +
	    fs_frames;

	/* compute frame number for next insertion */
	xfer->pipe->isoc_next += fs_frames;

	/* setup TDs */
	musbotg_setup_standard_chain(xfer);
	return;
}

static void
musbotg_device_isoc_start(struct usb2_xfer *xfer)
{
	/* start TD chain */
	musbotg_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods musbotg_device_isoc_methods =
{
	.open = musbotg_device_isoc_open,
	.close = musbotg_device_isoc_close,
	.enter = musbotg_device_isoc_enter,
	.start = musbotg_device_isoc_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * musbotg root control support
 *------------------------------------------------------------------------*
 * simulate a hardware HUB by handling
 * all the necessary requests
 *------------------------------------------------------------------------*/
static void
musbotg_root_ctrl_open(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_root_ctrl_close(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc = xfer->usb2_sc;

	if (sc->sc_root_ctrl.xfer == xfer) {
		sc->sc_root_ctrl.xfer = NULL;
	}
	musbotg_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

/*
 * USB descriptors for the virtual Root HUB:
 */

static const struct usb2_device_descriptor musbotg_devd = {
	.bLength = sizeof(struct usb2_device_descriptor),
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

static const struct usb2_device_qualifier musbotg_odevd = {
	.bLength = sizeof(struct usb2_device_qualifier),
	.bDescriptorType = UDESC_DEVICE_QUALIFIER,
	.bcdUSB = {0x00, 0x02},
	.bDeviceClass = UDCLASS_HUB,
	.bDeviceSubClass = UDSUBCLASS_HUB,
	.bDeviceProtocol = UDPROTO_FSHUB,
	.bMaxPacketSize0 = 0,
	.bNumConfigurations = 0,
};

static const struct musbotg_config_desc musbotg_confd = {
	.confd = {
		.bLength = sizeof(struct usb2_config_descriptor),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength[0] = sizeof(musbotg_confd),
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_SELF_POWERED,
		.bMaxPower = 0,
	},
	.ifcd = {
		.bLength = sizeof(struct usb2_interface_descriptor),
		.bDescriptorType = UDESC_INTERFACE,
		.bNumEndpoints = 1,
		.bInterfaceClass = UICLASS_HUB,
		.bInterfaceSubClass = UISUBCLASS_HUB,
		.bInterfaceProtocol = UIPROTO_HSHUBSTT,
	},

	.endpd = {
		.bLength = sizeof(struct usb2_endpoint_descriptor),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = (UE_DIR_IN | MUSBOTG_INTR_ENDPT),
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 8,
		.bInterval = 255,
	},
};

static const struct usb2_hub_descriptor_min musbotg_hubd = {
	.bDescLength = sizeof(musbotg_hubd),
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
  'M', 0, 'e', 0, 'n', 0, 't', 0, 'o', 0, 'r', 0, ' ', 0, \
  'G', 0, 'r', 0, 'a', 0, 'p', 0, 'h', 0, 'i', 0, 'c', 0, 's', 0

#define	STRING_PRODUCT \
  'O', 0, 'T', 0, 'G', 0, ' ', 0, 'R', 0, \
  'o', 0, 'o', 0, 't', 0, ' ', 0, 'H', 0, \
  'U', 0, 'B', 0,

USB_MAKE_STRING_DESC(STRING_LANG, musbotg_langtab);
USB_MAKE_STRING_DESC(STRING_VENDOR, musbotg_vendor);
USB_MAKE_STRING_DESC(STRING_PRODUCT, musbotg_product);

static void
musbotg_root_ctrl_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_root_ctrl_start(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc = xfer->usb2_sc;

	sc->sc_root_ctrl.xfer = xfer;

	usb2_config_td_queue_command(
	    &(sc->sc_config_td), NULL, &musbotg_root_ctrl_task, 0, 0);

	return;
}

static void
musbotg_root_ctrl_task(struct musbotg_softc *sc,
    struct musbotg_config_copy *cc, uint16_t refcount)
{
	musbotg_root_ctrl_poll(sc);
	return;
}

static void
musbotg_root_ctrl_done(struct usb2_xfer *xfer,
    struct usb2_sw_transfer *std)
{
	struct musbotg_softc *sc = xfer->usb2_sc;
	uint16_t value;
	uint16_t index;
	uint8_t use_polling;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USB_SW_TR_SETUP) {
		if (std->state == USB_SW_TR_PRE_CALLBACK) {
			/* transfer transferred */
			musbotg_device_done(xfer, std->err);
		}
		goto done;
	}
	/* buffer reset */
	std->ptr = USB_ADD_BYTES(&(sc->sc_hub_temp), 0);
	std->len = 0;

	value = UGETW(std->req.wValue);
	index = UGETW(std->req.wIndex);

	use_polling = mtx_owned(xfer->priv_mtx) ? 1 : 0;

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
		std->len = sizeof(musbotg_devd);
		std->ptr = USB_ADD_BYTES(&musbotg_devd, 0);
		goto tr_valid;
	case UDESC_CONFIG:
		if (value & 0xff) {
			goto tr_stalled;
		}
		std->len = sizeof(musbotg_confd);
		std->ptr = USB_ADD_BYTES(&musbotg_confd, 0);
		goto tr_valid;
	case UDESC_STRING:
		switch (value & 0xff) {
		case 0:		/* Language table */
			std->len = sizeof(musbotg_langtab);
			std->ptr = USB_ADD_BYTES(&musbotg_langtab, 0);
			goto tr_valid;

		case 1:		/* Vendor */
			std->len = sizeof(musbotg_vendor);
			std->ptr = USB_ADD_BYTES(&musbotg_vendor, 0);
			goto tr_valid;

		case 2:		/* Product */
			std->len = sizeof(musbotg_product);
			std->ptr = USB_ADD_BYTES(&musbotg_product, 0);
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
		musbotg_wakeup_peer(xfer);
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
		musbotg_pull_down(sc);
		musbotg_clocks_off(sc);
		break;
	case UHF_C_PORT_CONNECTION:
		sc->sc_flags.change_connect = 0;
		break;
	case UHF_C_PORT_SUSPEND:
		sc->sc_flags.change_suspend = 0;
		break;
	default:
		std->err = USB_ERR_IOERROR;
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
		std->err = USB_ERR_IOERROR;
		goto done;
	}
	goto tr_valid;

tr_handle_get_port_status:

	DPRINTFN(8, "UR_GET_PORT_STATUS\n");

	if (index != 1) {
		goto tr_stalled;
	}
	if (sc->sc_flags.status_vbus) {
		musbotg_clocks_on(sc);
		musbotg_pull_up(sc);
	} else {
		musbotg_pull_down(sc);
		musbotg_clocks_off(sc);
	}

	/* Select Device Side Mode */
	value = UPS_PORT_MODE_DEVICE;

	if (sc->sc_flags.status_high_speed) {
		value |= UPS_HIGH_SPEED;
	}
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
			/* reset EP0 state */
			sc->sc_ep0_busy = 0;
			sc->sc_ep0_cmd = 0;
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
	std->ptr = USB_ADD_BYTES(&musbotg_hubd, 0);
	std->len = sizeof(musbotg_hubd);
	goto tr_valid;

tr_stalled:
	std->err = USB_ERR_STALLED;
tr_valid:
done:
	return;
}

static void
musbotg_root_ctrl_poll(struct musbotg_softc *sc)
{
	usb2_sw_transfer(&(sc->sc_root_ctrl),
	    &musbotg_root_ctrl_done);
	return;
}

struct usb2_pipe_methods musbotg_root_ctrl_methods =
{
	.open = musbotg_root_ctrl_open,
	.close = musbotg_root_ctrl_close,
	.enter = musbotg_root_ctrl_enter,
	.start = musbotg_root_ctrl_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 0,
};

/*------------------------------------------------------------------------*
 * musbotg root interrupt support
 *------------------------------------------------------------------------*/
static void
musbotg_root_intr_open(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_root_intr_close(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc = xfer->usb2_sc;

	if (sc->sc_root_intr.xfer == xfer) {
		sc->sc_root_intr.xfer = NULL;
	}
	musbotg_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
musbotg_root_intr_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_root_intr_start(struct usb2_xfer *xfer)
{
	struct musbotg_softc *sc = xfer->usb2_sc;

	sc->sc_root_intr.xfer = xfer;
	return;
}

struct usb2_pipe_methods musbotg_root_intr_methods =
{
	.open = musbotg_root_intr_open,
	.close = musbotg_root_intr_close,
	.enter = musbotg_root_intr_enter,
	.start = musbotg_root_intr_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

static void
musbotg_xfer_setup(struct usb2_setup_params *parm)
{
	const struct usb2_hw_ep_profile *pf;
	struct musbotg_softc *sc;
	struct usb2_xfer *xfer;
	void *last_obj;
	uint32_t ntd;
	uint32_t n;
	uint8_t ep_no;

	sc = MUSBOTG_BUS2SC(parm->udev->bus);
	xfer = parm->curr_xfer;

	/*
	 * setup xfer
	 */
	xfer->usb2_sc = sc;

	/*
	 * NOTE: This driver does not use any of the parameters that
	 * are computed from the following values. Just set some
	 * reasonable dummies:
	 */
	parm->hc_max_packet_size = 0x400;
	parm->hc_max_frame_size = 0x400;

	if ((parm->methods == &musbotg_device_isoc_methods) ||
	    (parm->methods == &musbotg_device_intr_methods))
		parm->hc_max_packet_count = 3;
	else
		parm->hc_max_packet_count = 1;

	usb2_transfer_setup_sub(parm);

	/*
	 * compute maximum number of TDs
	 */
	if (parm->methods == &musbotg_device_ctrl_methods) {

		ntd = xfer->nframes + 1 /* STATUS */ + 1 /* SYNC */ ;

	} else if (parm->methods == &musbotg_device_bulk_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else if (parm->methods == &musbotg_device_intr_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else if (parm->methods == &musbotg_device_isoc_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else {

		ntd = 0;
	}

	/*
	 * check if "usb2_transfer_setup_sub" set an error
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
		musbotg_get_hw_ep_profile(xfer->udev, &pf, ep_no);

		if (pf == NULL) {
			/* should not happen */
			parm->err = USB_ERR_INVAL;
			return;
		}
	} else {
		ep_no = 0;
		pf = NULL;
	}

	/* align data */
	parm->size[0] += ((-parm->size[0]) & (USB_HOST_ALIGN - 1));

	for (n = 0; n != ntd; n++) {

		struct musbotg_td *td;

		if (parm->buf) {

			td = USB_ADD_BYTES(parm->buf, parm->size[0]);

			/* init TD */
			td->max_frame_size = xfer->max_frame_size;
			td->ep_no =
			    (xfer->pipe->edesc->bEndpointAddress & UE_ADDR);
			td->obj_next = last_obj;

			last_obj = td;
		}
		parm->size[0] += sizeof(*td);
	}

	xfer->td_start[0] = last_obj;
	return;
}

static void
musbotg_xfer_unsetup(struct usb2_xfer *xfer)
{
	return;
}

static void
musbotg_pipe_init(struct usb2_device *udev, struct usb2_endpoint_descriptor *edesc,
    struct usb2_pipe *pipe)
{
	struct musbotg_softc *sc = MUSBOTG_BUS2SC(udev->bus);

	DPRINTFN(2, "pipe=%p, addr=%d, endpt=%d, mode=%d (%d)\n",
	    pipe, udev->address,
	    edesc->bEndpointAddress, udev->flags.usb2_mode,
	    sc->sc_rt_addr);

	if (udev->device_index == sc->sc_rt_addr) {

		if (udev->flags.usb2_mode != USB_MODE_HOST) {
			/* not supported */
			return;
		}
		switch (edesc->bEndpointAddress) {
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &musbotg_root_ctrl_methods;
			break;
		case UE_DIR_IN | MUSBOTG_INTR_ENDPT:
			pipe->methods = &musbotg_root_intr_methods;
			break;
		default:
			/* do nothing */
			break;
		}
	} else {

		if (udev->flags.usb2_mode != USB_MODE_DEVICE) {
			/* not supported */
			return;
		}
		if ((udev->speed != USB_SPEED_FULL) &&
		    (udev->speed != USB_SPEED_HIGH)) {
			/* not supported */
			return;
		}
		switch (edesc->bmAttributes & UE_XFERTYPE) {
		case UE_CONTROL:
			pipe->methods = &musbotg_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &musbotg_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			pipe->methods = &musbotg_device_isoc_methods;
			break;
		case UE_BULK:
			pipe->methods = &musbotg_device_bulk_methods;
			break;
		default:
			/* do nothing */
			break;
		}
	}
	return;
}

struct usb2_bus_methods musbotg_bus_methods =
{
	.pipe_init = &musbotg_pipe_init,
	.xfer_setup = &musbotg_xfer_setup,
	.xfer_unsetup = &musbotg_xfer_unsetup,
	.do_poll = &musbotg_do_poll,
	.get_hw_ep_profile = &musbotg_get_hw_ep_profile,
	.set_stall = &musbotg_set_stall,
	.clear_stall = &musbotg_clear_stall,
	.vbus_interrupt = &musbotg_vbus_interrupt,
	.rem_wakeup_set = &musbotg_rem_wakeup_set,
};