/*-
 * Copyright (c) 2008 Hans Petter Selasky <hselasky@freebsd.org>
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
 * This file contains the driver for the USS820 series USB Device
 * Controller
 *
 * NOTE: The datasheet does not document everything!
 */

#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_revision.h>
#include <dev/usb2/include/usb2_error.h>
#include <dev/usb2/include/usb2_defs.h>

#define	USB_DEBUG_VAR uss820dcidebug
#define	usb2_config_td_cc uss820dci_config_copy
#define	usb2_config_td_softc uss820dci_softc

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
#include <dev/usb2/controller/uss820dci.h>

#define	USS820_DCI_BUS2SC(bus) \
   ((struct uss820dci_softc *)(((uint8_t *)(bus)) - \
   USB_P2U(&(((struct uss820dci_softc *)0)->sc_bus))))

#define	USS820_DCI_PC2SC(pc) \
   USS820_DCI_BUS2SC((pc)->tag_parent->info->bus)

#ifdef USB_DEBUG
static int uss820dcidebug = 0;

SYSCTL_NODE(_hw_usb2, OID_AUTO, uss820dci, CTLFLAG_RW, 0, "USB uss820dci");
SYSCTL_INT(_hw_usb2_uss820dci, OID_AUTO, debug, CTLFLAG_RW,
    &uss820dcidebug, 0, "uss820dci debug level");
#endif

#define	USS820_DCI_INTR_ENDPT 1

/* prototypes */

struct usb2_bus_methods uss820dci_bus_methods;
struct usb2_pipe_methods uss820dci_device_bulk_methods;
struct usb2_pipe_methods uss820dci_device_ctrl_methods;
struct usb2_pipe_methods uss820dci_device_intr_methods;
struct usb2_pipe_methods uss820dci_device_isoc_fs_methods;
struct usb2_pipe_methods uss820dci_root_ctrl_methods;
struct usb2_pipe_methods uss820dci_root_intr_methods;

static uss820dci_cmd_t uss820dci_setup_rx;
static uss820dci_cmd_t uss820dci_data_rx;
static uss820dci_cmd_t uss820dci_data_tx;
static uss820dci_cmd_t uss820dci_data_tx_sync;
static void uss820dci_device_done(struct usb2_xfer *xfer, usb2_error_t error);
static void uss820dci_do_poll(struct usb2_bus *bus);
static void uss820dci_root_ctrl_poll(struct uss820dci_softc *sc);
static void uss820dci_standard_done(struct usb2_xfer *xfer);
static void uss820dci_intr_set(struct usb2_xfer *xfer, uint8_t set);
static void uss820dci_update_shared_1(struct uss820dci_softc *sc, uint8_t reg, uint8_t keep_mask, uint8_t set_mask);

static usb2_sw_transfer_func_t uss820dci_root_intr_done;
static usb2_sw_transfer_func_t uss820dci_root_ctrl_done;
static usb2_config_td_command_t uss820dci_root_ctrl_task;

/*
 * Here is a list of what the USS820D chip can support. The main
 * limitation is that the sum of the buffer sizes must be less than
 * 1120 bytes.
 */
static const struct usb2_hw_ep_profile
	uss820dci_ep_profile[] = {

	[0] = {
		.max_frame_size = 32,
		.is_simplex = 0,
		.support_control = 1,
	},
	[1] = {
		.max_frame_size = 64,
		.is_simplex = 0,
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_in = 1,
		.support_out = 1,
	},
	[2] = {
		.max_frame_size = 8,
		.is_simplex = 0,
		.support_multi_buffer = 1,
		.support_bulk = 1,
		.support_interrupt = 1,
		.support_in = 1,
		.support_out = 1,
	},
	[3] = {
		.max_frame_size = 256,
		.is_simplex = 0,
		.support_multi_buffer = 1,
		.support_isochronous = 1,
		.support_in = 1,
		.support_out = 1,
	},
};

static void
uss820dci_update_shared_1(struct uss820dci_softc *sc, uint8_t reg,
    uint8_t keep_mask, uint8_t set_mask)
{
	uint8_t temp;

	USS820_WRITE_1(sc, USS820_PEND, 1);
	temp = USS820_READ_1(sc, reg);
	temp &= (keep_mask);
	temp |= (set_mask);
	USS820_WRITE_1(sc, reg, temp);
	USS820_WRITE_1(sc, USS820_PEND, 0);
	return;
}

static void
uss820dci_get_hw_ep_profile(struct usb2_device *udev,
    const struct usb2_hw_ep_profile **ppf, uint8_t ep_addr)
{
	if (ep_addr == 0) {
		*ppf = uss820dci_ep_profile + 0;
	} else if (ep_addr < 5) {
		*ppf = uss820dci_ep_profile + 1;
	} else if (ep_addr < 7) {
		*ppf = uss820dci_ep_profile + 2;
	} else if (ep_addr == 7) {
		*ppf = uss820dci_ep_profile + 3;
	} else {
		*ppf = NULL;
	}
	return;
}

static void
uss820dci_pull_up(struct uss820dci_softc *sc)
{
	uint8_t temp;

	/* pullup D+, if possible */

	if (!sc->sc_flags.d_pulled_up &&
	    sc->sc_flags.port_powered) {
		sc->sc_flags.d_pulled_up = 1;

		DPRINTF(0, "\n");

		temp = USS820_READ_1(sc, USS820_MCSR);
		temp |= USS820_MCSR_DPEN;
		USS820_WRITE_1(sc, USS820_MCSR, temp);
	}
	return;
}

static void
uss820dci_pull_down(struct uss820dci_softc *sc)
{
	uint8_t temp;

	/* pulldown D+, if possible */

	if (sc->sc_flags.d_pulled_up) {
		sc->sc_flags.d_pulled_up = 0;

		DPRINTF(0, "\n");

		temp = USS820_READ_1(sc, USS820_MCSR);
		temp &= ~USS820_MCSR_DPEN;
		USS820_WRITE_1(sc, USS820_MCSR, temp);
	}
	return;
}

static void
uss820dci_wakeup_peer(struct uss820dci_softc *sc)
{
	if (!(sc->sc_flags.status_suspend)) {
		return;
	}
	DPRINTF(-1, "not supported\n");

	return;
}

static void
uss820dci_rem_wakeup_set(struct usb2_device *udev, uint8_t is_on)
{
	struct uss820dci_softc *sc;
	uint8_t temp;

	DPRINTF(4, "is_on=%u\n", is_on);

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	sc = USS820_DCI_BUS2SC(udev->bus);

	temp = USS820_READ_1(sc, USS820_SCR);

	if (is_on) {
		temp |= USS820_SCR_RWUPE;
	} else {
		temp &= ~USS820_SCR_RWUPE;
	}

	USS820_WRITE_1(sc, USS820_SCR, temp);

	return;
}

static void
uss820dci_set_address(struct uss820dci_softc *sc, uint8_t addr)
{
	DPRINTF(4, "addr=%d\n", addr);

	USS820_WRITE_1(sc, USS820_FADDR, addr);

	return;
}

static uint8_t
uss820dci_setup_rx(struct uss820dci_td *td)
{
	struct uss820dci_softc *sc;
	struct usb2_device_request req;
	uint16_t count;
	uint8_t rx_stat;
	uint8_t temp;

	/* select the correct endpoint */
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->ep_reg, td->ep_index);

	/* read out FIFO status */
	rx_stat = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_stat_reg);

	/* get pointer to softc */
	sc = USS820_DCI_PC2SC(td->pc);

	DPRINTF(4, "rx_stat=0x%02x rem=%u\n", rx_stat, td->remainder);

	if (!(rx_stat & USS820_RXSTAT_RXSETUP)) {
		/* abort any ongoing transfer */
		if (!td->did_stall) {
			DPRINTF(4, "stalling\n");

			/* set stall */

			uss820dci_update_shared_1(sc, USS820_EPCON, 0xFF,
			    (USS820_EPCON_TXSTL | USS820_EPCON_RXSTL));

			td->did_stall = 1;
		}
		goto not_complete;
	}
	/* clear stall and all I/O */
	uss820dci_update_shared_1(sc, USS820_EPCON,
	    0xFF ^ (USS820_EPCON_TXSTL |
	    USS820_EPCON_RXSTL |
	    USS820_EPCON_RXIE |
	    USS820_EPCON_TXOE), 0);

	/* clear end overwrite flag */
	uss820dci_update_shared_1(sc, USS820_RXSTAT,
	    0xFF ^ USS820_RXSTAT_EDOVW, 0);

	/* get the packet byte count */
	count = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_count_low_reg);
	count |= (bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_count_high_reg) << 8);
	count &= 0x3FF;

	/* verify data length */
	if (count != td->remainder) {
		DPRINTF(-1, "Invalid SETUP packet "
		    "length, %d bytes\n", count);
		goto not_complete;
	}
	if (count != sizeof(req)) {
		DPRINTF(-1, "Unsupported SETUP packet "
		    "length, %d bytes\n", count);
		goto not_complete;
	}
	/* receive data */
	bus_space_read_multi_1(td->io_tag, td->io_hdl,
	    td->rx_fifo_reg, (void *)&req, sizeof(req));

	/* read out FIFO status */
	rx_stat = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_stat_reg);

	if (rx_stat & (USS820_RXSTAT_EDOVW |
	    USS820_RXSTAT_STOVW)) {
		DPRINTF(0, "new SETUP packet received\n");
		return (1);		/* not complete */
	}
	/* clear receive setup bit */
	uss820dci_update_shared_1(sc, USS820_RXSTAT,
	    0xFF ^ (USS820_RXSTAT_RXSETUP |
	    USS820_RXSTAT_EDOVW |
	    USS820_RXSTAT_STOVW), 0);

	/* set RXFFRC bit */
	temp = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_cntl_reg);
	temp |= USS820_RXCON_RXFFRC;
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->rx_cntl_reg, temp);

	/* copy data into real buffer */
	usb2_copy_in(td->pc, 0, &req, sizeof(req));

	td->offset = sizeof(req);
	td->remainder = 0;

	/* sneak peek the set address */
	if ((req.bmRequestType == UT_WRITE_DEVICE) &&
	    (req.bRequest == UR_SET_ADDRESS)) {
		sc->sc_dv_addr = req.wValue[0] & 0x7F;
	} else {
		sc->sc_dv_addr = 0xFF;
	}
	return (0);			/* complete */

not_complete:
	/* clear end overwrite flag, if any */
	if (rx_stat & USS820_RXSTAT_RXSETUP) {
		uss820dci_update_shared_1(sc, USS820_RXSTAT,
		    0xFF ^ (USS820_RXSTAT_EDOVW |
		    USS820_RXSTAT_STOVW |
		    USS820_RXSTAT_RXSETUP), 0);
	}
	return (1);			/* not complete */

}

static uint8_t
uss820dci_data_rx(struct uss820dci_td *td)
{
	struct usb2_page_search buf_res;
	uint16_t count;
	uint8_t rx_flag;
	uint8_t rx_stat;
	uint8_t rx_cntl;
	uint8_t to;
	uint8_t got_short;

	to = 2;				/* don't loop forever! */
	got_short = 0;

	/* select the correct endpoint */
	bus_space_write_1(td->io_tag, td->io_hdl, td->ep_reg, td->ep_index);

	/* check if any of the FIFO banks have data */
repeat:
	/* read out FIFO flag */
	rx_flag = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_flag_reg);
	/* read out FIFO status */
	rx_stat = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_stat_reg);

	DPRINTF(4, "rx_stat=0x%02x rx_flag=0x%02x rem=%u\n",
	    rx_stat, rx_flag, td->remainder);

	if (rx_stat & (USS820_RXSTAT_RXSETUP |
	    USS820_RXSTAT_RXSOVW |
	    USS820_RXSTAT_EDOVW)) {
		if (td->remainder == 0) {
			/*
			 * We are actually complete and have
			 * received the next SETUP
			 */
			DPRINTF(4, "faking complete\n");
			return (0);	/* complete */
		}
		/*
	         * USB Host Aborted the transfer.
	         */
		td->error = 1;
		return (0);		/* complete */
	}
	/* check for errors */
	if (rx_flag & (USS820_RXFLG_RXOVF |
	    USS820_RXFLG_RXURF)) {
		DPRINTF(4, "overflow or underflow\n");
		/* should not happen */
		td->error = 1;
		return (0);		/* complete */
	}
	/* check status */
	if (!(rx_flag & (USS820_RXFLG_RXFIF0 |
	    USS820_RXFLG_RXFIF1))) {

		/* read out EPCON register */
		/* enable RX input */
		if (!td->did_stall) {
			uss820dci_update_shared_1(USS820_DCI_PC2SC(td->pc),
			    USS820_EPCON, 0xFF, USS820_EPCON_RXIE);
			td->did_stall = 1;
		}
		return (1);		/* not complete */
	}
	/* get the packet byte count */
	count = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_count_low_reg);

	count |= (bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_count_high_reg) << 8);
	count &= 0x3FF;

	DPRINTF(4, "count=0x%04x\n", count);

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
		usb2_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* receive data */
		bus_space_read_multi_1(td->io_tag, td->io_hdl,
		    td->rx_fifo_reg, buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* set RXFFRC bit */
	rx_cntl = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_cntl_reg);
	rx_cntl |= USS820_RXCON_RXFFRC;
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->rx_cntl_reg, rx_cntl);

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
uss820dci_data_tx(struct uss820dci_td *td)
{
	struct usb2_page_search buf_res;
	uint16_t count;
	uint16_t count_copy;
	uint8_t rx_stat;
	uint8_t tx_flag;
	uint8_t to;

	/* select the correct endpoint */
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->ep_reg, td->ep_index);

	to = 2;				/* don't loop forever! */

repeat:
	/* read out TX FIFO flags */
	tx_flag = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->tx_flag_reg);

	/* read out RX FIFO status last */
	rx_stat = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_stat_reg);

	DPRINTF(4, "rx_stat=0x%02x tx_flag=0x%02x rem=%u\n",
	    rx_stat, tx_flag, td->remainder);

	if (rx_stat & (USS820_RXSTAT_RXSETUP |
	    USS820_RXSTAT_RXSOVW |
	    USS820_RXSTAT_EDOVW)) {
		/*
	         * The current transfer was aborted
	         * by the USB Host
	         */
		td->error = 1;
		return (0);		/* complete */
	}
	if (tx_flag & (USS820_TXFLG_TXOVF |
	    USS820_TXFLG_TXURF)) {
		td->error = 1;
		return (0);		/* complete */
	}
	if (tx_flag & USS820_TXFLG_TXFIF0) {
		if (tx_flag & USS820_TXFLG_TXFIF1) {
			return (1);	/* not complete */
		}
	}
	if ((!td->support_multi_buffer) &&
	    (tx_flag & (USS820_TXFLG_TXFIF0 |
	    USS820_TXFLG_TXFIF1))) {
		return (1);		/* not complete */
	}
	count = td->max_packet_size;
	if (td->remainder < count) {
		/* we have a short packet */
		td->short_pkt = 1;
		count = td->remainder;
	}
	count_copy = count;
	while (count > 0) {

		usb2_get_page(td->pc, td->offset, &buf_res);

		/* get correct length */
		if (buf_res.length > count) {
			buf_res.length = count;
		}
		/* transmit data */
		bus_space_write_multi_1(td->io_tag, td->io_hdl,
		    td->tx_fifo_reg, buf_res.buffer, buf_res.length);

		/* update counters */
		count -= buf_res.length;
		td->offset += buf_res.length;
		td->remainder -= buf_res.length;
	}

	/* post-write high packet byte count first */
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->tx_count_high_reg, count_copy >> 8);

	/* post-write low packet byte count last */
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->tx_count_low_reg, count_copy);

	/*
	 * Enable TX output, which must happen after that we have written
	 * data into the FIFO. This is undocumented.
	 */
	if (!td->did_stall) {
		uss820dci_update_shared_1(USS820_DCI_PC2SC(td->pc),
		    USS820_EPCON, 0xFF, USS820_EPCON_TXOE);
		td->did_stall = 1;
	}
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
uss820dci_data_tx_sync(struct uss820dci_td *td)
{
	struct uss820dci_softc *sc;
	uint8_t rx_stat;
	uint8_t tx_flag;

	/* select the correct endpoint */
	bus_space_write_1(td->io_tag, td->io_hdl,
	    td->ep_reg, td->ep_index);

	/* read out TX FIFO flag */
	tx_flag = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->tx_flag_reg);

	/* read out RX FIFO status last */
	rx_stat = bus_space_read_1(td->io_tag, td->io_hdl,
	    td->rx_stat_reg);

	DPRINTF(4, "rx_stat=0x%02x rem=%u\n", rx_stat, td->remainder);

	if (rx_stat & (USS820_RXSTAT_RXSETUP |
	    USS820_RXSTAT_RXSOVW |
	    USS820_RXSTAT_EDOVW)) {
		DPRINTF(4, "faking complete\n");
		/* Race condition */
		return (0);		/* complete */
	}
	DPRINTF(4, "tx_flag=0x%02x rem=%u\n",
	    tx_flag, td->remainder);

	if (tx_flag & (USS820_TXFLG_TXOVF |
	    USS820_TXFLG_TXURF)) {
		td->error = 1;
		return (0);		/* complete */
	}
	if (tx_flag & (USS820_TXFLG_TXFIF0 |
	    USS820_TXFLG_TXFIF1)) {
		return (1);		/* not complete */
	}
	sc = USS820_DCI_PC2SC(td->pc);
	if (sc->sc_dv_addr != 0xFF) {
		/* write function address */
		uss820dci_set_address(sc, sc->sc_dv_addr);
	}
	return (0);			/* complete */
}

static uint8_t
uss820dci_xfer_do_fifo(struct usb2_xfer *xfer)
{
	struct uss820dci_td *td;

	DPRINTF(8, "\n");

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
		 * Fetch the next transfer descriptor.
		 */
		td = td->obj_next;
		xfer->td_transfer_cache = td;
	}
	return (1);			/* not complete */

done:
	/* compute all actual lengths */

	uss820dci_standard_done(xfer);

	return (0);			/* complete */
}

static void
uss820dci_interrupt_poll(struct uss820dci_softc *sc)
{
	struct usb2_xfer *xfer;

repeat:
	TAILQ_FOREACH(xfer, &(sc->sc_bus.intr_q.head), wait_entry) {
		if (!uss820dci_xfer_do_fifo(xfer)) {
			/* queue has been modified */
			goto repeat;
		}
	}
	return;
}

static void
uss820dci_wait_suspend(struct uss820dci_softc *sc, uint8_t on)
{
	uint8_t scr;
	uint8_t scratch;

	scr = USS820_READ_1(sc, USS820_SCR);
	scratch = USS820_READ_1(sc, USS820_SCRATCH);

	if (on) {
		scr |= USS820_SCR_IE_SUSP;
		scratch &= ~USS820_SCRATCH_IE_RESUME;
	} else {
		scr &= ~USS820_SCR_IE_SUSP;
		scratch |= USS820_SCRATCH_IE_RESUME;
	}

	USS820_WRITE_1(sc, USS820_SCR, scr);
	USS820_WRITE_1(sc, USS820_SCRATCH, scratch);
	return;
}

void
uss820dci_interrupt(struct uss820dci_softc *sc)
{
	uint8_t ssr;
	uint8_t event;

	mtx_lock(&(sc->sc_bus.mtx));

	ssr = USS820_READ_1(sc, USS820_SSR);

	ssr &= (USS820_SSR_SUSPEND |
	    USS820_SSR_RESUME |
	    USS820_SSR_RESET);

	/* acknowledge all interrupts */

	uss820dci_update_shared_1(sc, USS820_SSR, 0, 0);

	/* check for any bus state change interrupts */

	if (ssr) {

		event = 0;

		if (ssr & USS820_SSR_RESET) {
			sc->sc_flags.status_bus_reset = 1;
			sc->sc_flags.status_suspend = 0;
			sc->sc_flags.change_suspend = 0;
			sc->sc_flags.change_connect = 1;

			/* disable resume interrupt */
			uss820dci_wait_suspend(sc, 1);

			event = 1;
		}
		/*
	         * If "RESUME" and "SUSPEND" is set at the same time
	         * we interpret that like "RESUME". Resume is set when
	         * there is at least 3 milliseconds of inactivity on
	         * the USB BUS.
	         */
		if (ssr & USS820_SSR_RESUME) {
			if (sc->sc_flags.status_suspend) {
				sc->sc_flags.status_suspend = 0;
				sc->sc_flags.change_suspend = 1;
				/* disable resume interrupt */
				uss820dci_wait_suspend(sc, 1);
				event = 1;
			}
		} else if (ssr & USS820_SSR_SUSPEND) {
			if (!sc->sc_flags.status_suspend) {
				sc->sc_flags.status_suspend = 1;
				sc->sc_flags.change_suspend = 1;
				/* enable resume interrupt */
				uss820dci_wait_suspend(sc, 0);
				event = 1;
			}
		}
		if (event) {

			DPRINTF(0, "real bus interrupt 0x%02x\n", ssr);

			/* complete root HUB interrupt endpoint */

			usb2_sw_transfer(&(sc->sc_root_intr),
			    &uss820dci_root_intr_done);
		}
	}
	/* acknowledge all SBI interrupts */
	uss820dci_update_shared_1(sc, USS820_SBI, 0, 0);

	/* acknowledge all SBI1 interrupts */
	uss820dci_update_shared_1(sc, USS820_SBI1, 0, 0);

	/* poll all active transfers */
	uss820dci_interrupt_poll(sc);

	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

static void
uss820dci_setup_standard_chain_sub(struct uss820_std_temp *temp)
{
	struct uss820dci_td *td;

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
uss820dci_setup_standard_chain(struct usb2_xfer *xfer)
{
	struct uss820_std_temp temp;
	struct uss820dci_softc *sc;
	struct uss820dci_td *td;
	uint32_t x;
	uint8_t need_sync;
	uint8_t ep_no;

	DPRINTF(8, "addr=%d endpt=%d sumlen=%d speed=%d\n",
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

			temp.func = &uss820dci_setup_rx;
			temp.len = xfer->frlengths[0];
			temp.pc = xfer->frbuffers + 0;
			temp.short_pkt = temp.len ? 1 : 0;

			uss820dci_setup_standard_chain_sub(&temp);
		}
		x = 1;
	} else {
		x = 0;
	}

	if (x != xfer->nframes) {
		if (xfer->endpoint & UE_DIR_IN) {
			temp.func = &uss820dci_data_tx;
			need_sync = 1;
		} else {
			temp.func = &uss820dci_data_rx;
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

		uss820dci_setup_standard_chain_sub(&temp);

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
		temp.func = &uss820dci_data_tx_sync;
		temp.len = 0;
		temp.short_pkt = 0;

		uss820dci_setup_standard_chain_sub(&temp);
	}
	/* check if we should append a status stage */

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		/*
		 * Send a DATA1 message and invert the current
		 * endpoint direction.
		 */
		if (xfer->endpoint & UE_DIR_IN) {
			temp.func = &uss820dci_data_rx;
			need_sync = 0;
		} else {
			temp.func = &uss820dci_data_tx;
			need_sync = 1;
		}
		temp.len = 0;
		temp.short_pkt = 0;

		uss820dci_setup_standard_chain_sub(&temp);
		if (need_sync) {
			/* we need a SYNC point after TX */
			temp.func = &uss820dci_data_tx_sync;
			temp.len = 0;
			temp.short_pkt = 0;

			uss820dci_setup_standard_chain_sub(&temp);
		}
	}
	/* must have at least one frame! */
	td = temp.td;
	xfer->td_transfer_last = td;

	return;
}

static void
uss820dci_timeout(void *arg)
{
	struct usb2_xfer *xfer = arg;
	struct uss820dci_softc *sc = xfer->usb2_sc;

	DPRINTF(0, "xfer=%p\n", xfer);

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	uss820dci_device_done(xfer, USB_ERR_TIMEOUT);

	mtx_unlock(&sc->sc_bus.mtx);

	return;
}

static void
uss820dci_intr_set(struct usb2_xfer *xfer, uint8_t set)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;
	uint8_t ep_no = (xfer->endpoint & UE_ADDR);
	uint8_t ep_reg;
	uint8_t temp;

	DPRINTF(14, "endpoint 0x%02x\n", xfer->endpoint);

	if (ep_no > 3) {
		ep_reg = USS820_SBIE1;
	} else {
		ep_reg = USS820_SBIE;
	}

	ep_no &= 3;
	ep_no = 1 << (2 * ep_no);

	if (xfer->flags_int.control_xfr) {
		if (xfer->flags_int.control_hdr) {
			ep_no <<= 1;	/* RX interrupt only */
		} else {
			ep_no |= (ep_no << 1);	/* RX and TX interrupt */
		}
	} else {
		if (!(xfer->endpoint & UE_DIR_IN)) {
			ep_no <<= 1;
		}
	}
	temp = USS820_READ_1(sc, ep_reg);
	if (set) {
		temp |= ep_no;
	} else {
		temp &= ~ep_no;
	}
	USS820_WRITE_1(sc, ep_reg, temp);
	return;
}

static void
uss820dci_start_standard_chain(struct usb2_xfer *xfer)
{
	DPRINTF(8, "\n");

	/* poll one time */
	if (uss820dci_xfer_do_fifo(xfer)) {

		/*
		 * Only enable the endpoint interrupt when we are
		 * actually waiting for data, hence we are dealing
		 * with level triggered interrupts !
		 */
		uss820dci_intr_set(xfer, 1);

		/* put transfer on interrupt queue */
		usb2_transfer_enqueue(&(xfer->udev->bus->intr_q), xfer);

		/* start timeout, if any */
		if (xfer->timeout != 0) {
			usb2_transfer_timeout_ms(xfer,
			    &uss820dci_timeout, xfer->timeout);
		}
	}
	return;
}

static void
uss820dci_root_intr_done(struct usb2_xfer *xfer,
    struct usb2_sw_transfer *std)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;

	DPRINTF(8, "\n");

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USB_SW_TR_PRE_DATA) {
		if (std->state == USB_SW_TR_PRE_CALLBACK) {
			/* transfer transferred */
			uss820dci_device_done(xfer, std->err);
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
uss820dci_standard_done_sub(struct usb2_xfer *xfer)
{
	struct uss820dci_td *td;
	uint32_t len;
	uint8_t error;

	DPRINTF(8, "\n");

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
uss820dci_standard_done(struct usb2_xfer *xfer)
{
	usb2_error_t err = 0;

	DPRINTF(12, "xfer=%p pipe=%p transfer done\n",
	    xfer, xfer->pipe);

	/* reset scanner */

	xfer->td_transfer_cache = xfer->td_transfer_first;

	if (xfer->flags_int.control_xfr) {

		if (xfer->flags_int.control_hdr) {

			err = uss820dci_standard_done_sub(xfer);
		}
		xfer->aframes = 1;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}
	while (xfer->aframes != xfer->nframes) {

		err = uss820dci_standard_done_sub(xfer);
		xfer->aframes++;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		err = uss820dci_standard_done_sub(xfer);
	}
done:
	uss820dci_device_done(xfer, err);
	return;
}

/*------------------------------------------------------------------------*
 *	uss820dci_device_done
 *
 * NOTE: this function can be called more than one time on the
 * same USB transfer!
 *------------------------------------------------------------------------*/
static void
uss820dci_device_done(struct usb2_xfer *xfer, usb2_error_t error)
{
	mtx_assert(xfer->usb2_mtx, MA_OWNED);

	DPRINTF(1, "xfer=%p, pipe=%p, error=%d\n",
	    xfer, xfer->pipe, error);

	if (xfer->flags_int.usb2_mode == USB_MODE_DEVICE) {
		uss820dci_intr_set(xfer, 0);
	}
	/* dequeue transfer and start next transfer */
	usb2_transfer_done(xfer, error);
	return;
}

static void
uss820dci_set_stall(struct usb2_device *udev, struct usb2_xfer *xfer,
    struct usb2_pipe *pipe)
{
	struct uss820dci_softc *sc;
	uint8_t ep_no;
	uint8_t ep_type;
	uint8_t ep_dir;
	uint8_t temp;

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	DPRINTF(4, "pipe=%p\n", pipe);

	if (xfer) {
		/* cancel any ongoing transfers */
		uss820dci_device_done(xfer, USB_ERR_STALLED);
	}
	/* set FORCESTALL */
	sc = USS820_DCI_BUS2SC(udev->bus);
	ep_no = (pipe->edesc->bEndpointAddress & UE_ADDR);
	ep_dir = (pipe->edesc->bEndpointAddress & (UE_DIR_IN | UE_DIR_OUT));
	ep_type = (pipe->edesc->bmAttributes & UE_XFERTYPE);

	if (ep_type == UE_CONTROL) {
		/* should not happen */
		return;
	}
	USS820_WRITE_1(sc, USS820_EPINDEX, ep_no);

	if (ep_dir == UE_DIR_IN) {
		temp = USS820_EPCON_TXSTL;
	} else {
		temp = USS820_EPCON_RXSTL;
	}
	uss820dci_update_shared_1(sc, USS820_EPCON, 0xFF, temp);
	return;
}

static void
uss820dci_clear_stall_sub(struct uss820dci_softc *sc,
    uint8_t ep_no, uint8_t ep_type, uint8_t ep_dir)
{
	uint8_t temp;

	if (ep_type == UE_CONTROL) {
		/* clearing stall is not needed */
		return;
	}
	/* select endpoint index */
	USS820_WRITE_1(sc, USS820_EPINDEX, ep_no);

	/* clear stall and disable I/O transfers */
	if (ep_dir == UE_DIR_IN) {
		temp = 0xFF ^ (USS820_EPCON_TXOE |
		    USS820_EPCON_TXSTL);
	} else {
		temp = 0xFF ^ (USS820_EPCON_RXIE |
		    USS820_EPCON_RXSTL);
	}
	uss820dci_update_shared_1(sc, USS820_EPCON, temp, 0);

	if (ep_dir == UE_DIR_IN) {
		/* reset data toggle */
		USS820_WRITE_1(sc, USS820_TXSTAT,
		    USS820_TXSTAT_TXSOVW);

		/* reset FIFO */
		temp = USS820_READ_1(sc, USS820_TXCON);
		temp |= USS820_TXCON_TXCLR;
		USS820_WRITE_1(sc, USS820_TXCON, temp);
		temp &= ~USS820_TXCON_TXCLR;
		USS820_WRITE_1(sc, USS820_TXCON, temp);
	} else {

		/* reset data toggle */
		uss820dci_update_shared_1(sc, USS820_RXSTAT,
		    0, USS820_RXSTAT_RXSOVW);

		/* reset FIFO */
		temp = USS820_READ_1(sc, USS820_RXCON);
		temp |= USS820_RXCON_RXCLR;
		temp &= ~USS820_RXCON_RXFFRC;
		USS820_WRITE_1(sc, USS820_RXCON, temp);
		temp &= ~USS820_RXCON_RXCLR;
		USS820_WRITE_1(sc, USS820_RXCON, temp);
	}
	return;
}

static void
uss820dci_clear_stall(struct usb2_device *udev, struct usb2_pipe *pipe)
{
	struct uss820dci_softc *sc;
	struct usb2_endpoint_descriptor *ed;

	mtx_assert(&(udev->bus->mtx), MA_OWNED);

	DPRINTF(4, "pipe=%p\n", pipe);

	/* check mode */
	if (udev->flags.usb2_mode != USB_MODE_DEVICE) {
		/* not supported */
		return;
	}
	/* get softc */
	sc = USS820_DCI_BUS2SC(udev->bus);

	/* get endpoint descriptor */
	ed = pipe->edesc;

	/* reset endpoint */
	uss820dci_clear_stall_sub(sc,
	    (ed->bEndpointAddress & UE_ADDR),
	    (ed->bmAttributes & UE_XFERTYPE),
	    (ed->bEndpointAddress & (UE_DIR_IN | UE_DIR_OUT)));

	return;
}

usb2_error_t
uss820dci_init(struct uss820dci_softc *sc)
{
	const struct usb2_hw_ep_profile *pf;
	uint8_t n;
	uint8_t temp;

	DPRINTF(0, "start\n");

	/* set up the bus structure */
	sc->sc_bus.usbrev = USB_REV_1_1;
	sc->sc_bus.methods = &uss820dci_bus_methods;

	mtx_lock(&(sc->sc_bus.mtx));

	/* we always have VBUS */
	sc->sc_flags.status_vbus = 1;

	/* reset the chip */
	USS820_WRITE_1(sc, USS820_SCR, USS820_SCR_SRESET);
	DELAY(100);
	USS820_WRITE_1(sc, USS820_SCR, 0);

	/* wait for reset to complete */
	for (n = 0;; n++) {

		temp = USS820_READ_1(sc, USS820_MCSR);

		if (temp & USS820_MCSR_INIT) {
			break;
		}
		if (n == 100) {
			mtx_unlock(&(sc->sc_bus.mtx));
			return (USB_ERR_INVAL);
		}
		/* wait a little for things to stabilise */
		DELAY(100);
	}

	/* do a pulldown */
	uss820dci_pull_down(sc);

	/* wait 10ms for pulldown to stabilise */

	DELAY(10000);

	/* check hardware revision */
	temp = USS820_READ_1(sc, USS820_REV);

	if (temp < 0x13) {
		mtx_unlock(&(sc->sc_bus.mtx));
		return (USB_ERR_INVAL);
	}
	/* enable interrupts */
	USS820_WRITE_1(sc, USS820_SCR,
	    USS820_SCR_T_IRQ |
	    USS820_SCR_IE_RESET |
	    USS820_SCR_IE_SUSP |
	    USS820_SCR_IRQPOL);

	/* enable interrupts */
	USS820_WRITE_1(sc, USS820_SCRATCH,
	    USS820_SCRATCH_IE_RESUME);

	/* enable features */
	USS820_WRITE_1(sc, USS820_MCSR,
	    USS820_MCSR_BDFEAT |
	    USS820_MCSR_FEAT);

	sc->sc_flags.mcsr_feat = 1;

	/* disable interrupts */
	USS820_WRITE_1(sc, USS820_SBIE, 0);

	/* disable interrupts */
	USS820_WRITE_1(sc, USS820_SBIE1, 0);

	/* disable all endpoints */
	for (n = 0; n != USS820_EP_MAX; n++) {

		/* select endpoint */
		USS820_WRITE_1(sc, USS820_EPINDEX, n);

		/* disable endpoint */
		uss820dci_update_shared_1(sc, USS820_EPCON, 0, 0);
	}

	/*
	 * Initialise default values for some registers that cannot be
	 * changed during operation!
	 */
	for (n = 0; n != USS820_EP_MAX; n++) {

		uss820dci_get_hw_ep_profile(NULL, &pf, n);

		if (pf->support_isochronous) {
			if (pf->max_frame_size <= 64) {
				temp = (USS820_TXCON_FFSZ_16_64 |
				    USS820_TXCON_TXISO |
				    USS820_TXCON_ATM);
			} else if (pf->max_frame_size <= 256) {
				temp = (USS820_TXCON_FFSZ_64_256 |
				    USS820_TXCON_TXISO |
				    USS820_TXCON_ATM);
			} else if (pf->max_frame_size <= 512) {
				temp = (USS820_TXCON_FFSZ_8_512 |
				    USS820_TXCON_TXISO |
				    USS820_TXCON_ATM);
			} else {	/* 1024 bytes */
				temp = (USS820_TXCON_FFSZ_32_1024 |
				    USS820_TXCON_TXISO |
				    USS820_TXCON_ATM);
			}
		} else {
			if ((pf->max_frame_size <= 8) &&
			    (sc->sc_flags.mcsr_feat)) {
				temp = (USS820_TXCON_FFSZ_8_512 |
				    USS820_TXCON_ATM);
			} else if (pf->max_frame_size <= 16) {
				temp = (USS820_TXCON_FFSZ_16_64 |
				    USS820_TXCON_ATM);
			} else if ((pf->max_frame_size <= 32) &&
			    (sc->sc_flags.mcsr_feat)) {
				temp = (USS820_TXCON_FFSZ_32_1024 |
				    USS820_TXCON_ATM);
			} else {	/* 64 bytes */
				temp = (USS820_TXCON_FFSZ_64_256 |
				    USS820_TXCON_ATM);
			}
		}

		/* need to configure the chip early */

		USS820_WRITE_1(sc, USS820_EPINDEX, n);
		USS820_WRITE_1(sc, USS820_TXCON, temp);
		USS820_WRITE_1(sc, USS820_RXCON, temp);

		if (pf->support_control) {
			temp = USS820_EPCON_CTLEP |
			    USS820_EPCON_RXSPM |
			    USS820_EPCON_RXIE |
			    USS820_EPCON_RXEPEN |
			    USS820_EPCON_TXOE |
			    USS820_EPCON_TXEPEN;
		} else {
			temp = USS820_EPCON_RXEPEN | USS820_EPCON_TXEPEN;
		}

		uss820dci_update_shared_1(sc, USS820_EPCON, 0xFF, temp);
	}

	mtx_unlock(&(sc->sc_bus.mtx));

	/* catch any lost interrupts */

	uss820dci_do_poll(&(sc->sc_bus));

	return (0);			/* success */
}

void
uss820dci_uninit(struct uss820dci_softc *sc)
{
	uint8_t temp;

	mtx_lock(&(sc->sc_bus.mtx));

	/* disable all interrupts */
	temp = USS820_READ_1(sc, USS820_SCR);
	temp &= ~USS820_SCR_T_IRQ;
	USS820_WRITE_1(sc, USS820_SCR, temp);

	sc->sc_flags.port_powered = 0;
	sc->sc_flags.status_vbus = 0;
	sc->sc_flags.status_bus_reset = 0;
	sc->sc_flags.status_suspend = 0;
	sc->sc_flags.change_suspend = 0;
	sc->sc_flags.change_connect = 1;

	uss820dci_pull_down(sc);
	mtx_unlock(&(sc->sc_bus.mtx));

	return;
}

void
uss820dci_suspend(struct uss820dci_softc *sc)
{
	return;
}

void
uss820dci_resume(struct uss820dci_softc *sc)
{
	return;
}

static void
uss820dci_do_poll(struct usb2_bus *bus)
{
	struct uss820dci_softc *sc = USS820_DCI_BUS2SC(bus);

	mtx_lock(&(sc->sc_bus.mtx));
	uss820dci_interrupt_poll(sc);
	uss820dci_root_ctrl_poll(sc);
	mtx_unlock(&(sc->sc_bus.mtx));
	return;
}

/*------------------------------------------------------------------------*
 * at91dci bulk support
 *------------------------------------------------------------------------*/
static void
uss820dci_device_bulk_open(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_bulk_close(struct usb2_xfer *xfer)
{
	uss820dci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
uss820dci_device_bulk_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_bulk_start(struct usb2_xfer *xfer)
{
	/* setup TDs */
	uss820dci_setup_standard_chain(xfer);
	uss820dci_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods uss820dci_device_bulk_methods =
{
	.open = uss820dci_device_bulk_open,
	.close = uss820dci_device_bulk_close,
	.enter = uss820dci_device_bulk_enter,
	.start = uss820dci_device_bulk_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * at91dci control support
 *------------------------------------------------------------------------*/
static void
uss820dci_device_ctrl_open(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_ctrl_close(struct usb2_xfer *xfer)
{
	uss820dci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
uss820dci_device_ctrl_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_ctrl_start(struct usb2_xfer *xfer)
{
	/* setup TDs */
	uss820dci_setup_standard_chain(xfer);
	uss820dci_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods uss820dci_device_ctrl_methods =
{
	.open = uss820dci_device_ctrl_open,
	.close = uss820dci_device_ctrl_close,
	.enter = uss820dci_device_ctrl_enter,
	.start = uss820dci_device_ctrl_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * at91dci interrupt support
 *------------------------------------------------------------------------*/
static void
uss820dci_device_intr_open(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_intr_close(struct usb2_xfer *xfer)
{
	uss820dci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
uss820dci_device_intr_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_intr_start(struct usb2_xfer *xfer)
{
	/* setup TDs */
	uss820dci_setup_standard_chain(xfer);
	uss820dci_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods uss820dci_device_intr_methods =
{
	.open = uss820dci_device_intr_open,
	.close = uss820dci_device_intr_close,
	.enter = uss820dci_device_intr_enter,
	.start = uss820dci_device_intr_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * at91dci full speed isochronous support
 *------------------------------------------------------------------------*/
static void
uss820dci_device_isoc_fs_open(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_device_isoc_fs_close(struct usb2_xfer *xfer)
{
	uss820dci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
uss820dci_device_isoc_fs_enter(struct usb2_xfer *xfer)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;
	uint32_t temp;
	uint32_t nframes;

	DPRINTF(5, "xfer=%p next=%d nframes=%d\n",
	    xfer, xfer->pipe->isoc_next, xfer->nframes);

	/* get the current frame index - we don't need the high bits */

	nframes = USS820_READ_1(sc, USS820_SOFL);

	/*
	 * check if the frame index is within the window where the
	 * frames will be inserted
	 */
	temp = (nframes - xfer->pipe->isoc_next) & USS820_SOFL_MASK;

	if ((xfer->pipe->is_synced == 0) ||
	    (temp < xfer->nframes)) {
		/*
		 * If there is data underflow or the pipe queue is
		 * empty we schedule the transfer a few frames ahead
		 * of the current frame position. Else two isochronous
		 * transfers might overlap.
		 */
		xfer->pipe->isoc_next = (nframes + 3) & USS820_SOFL_MASK;
		xfer->pipe->is_synced = 1;
		DPRINTF(2, "start next=%d\n", xfer->pipe->isoc_next);
	}
	/*
	 * compute how many milliseconds the insertion is ahead of the
	 * current frame position:
	 */
	temp = (xfer->pipe->isoc_next - nframes) & USS820_SOFL_MASK;

	/*
	 * pre-compute when the isochronous transfer will be finished:
	 */
	xfer->isoc_time_complete =
	    usb2_isoc_time_expand(&(sc->sc_bus), nframes) + temp +
	    xfer->nframes;

	/* compute frame number for next insertion */
	xfer->pipe->isoc_next += xfer->nframes;

	/* setup TDs */
	uss820dci_setup_standard_chain(xfer);
	return;
}

static void
uss820dci_device_isoc_fs_start(struct usb2_xfer *xfer)
{
	/* start TD chain */
	uss820dci_start_standard_chain(xfer);
	return;
}

struct usb2_pipe_methods uss820dci_device_isoc_fs_methods =
{
	.open = uss820dci_device_isoc_fs_open,
	.close = uss820dci_device_isoc_fs_close,
	.enter = uss820dci_device_isoc_fs_enter,
	.start = uss820dci_device_isoc_fs_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * at91dci root control support
 *------------------------------------------------------------------------*
 * simulate a hardware HUB by handling
 * all the necessary requests
 *------------------------------------------------------------------------*/

static void
uss820dci_root_ctrl_open(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_root_ctrl_close(struct usb2_xfer *xfer)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;

	if (sc->sc_root_ctrl.xfer == xfer) {
		sc->sc_root_ctrl.xfer = NULL;
	}
	uss820dci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

/*
 * USB descriptors for the virtual Root HUB:
 */

static const struct usb2_device_descriptor uss820dci_devd = {
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

static const struct usb2_device_qualifier uss820dci_odevd = {
	.bLength = sizeof(struct usb2_device_qualifier),
	.bDescriptorType = UDESC_DEVICE_QUALIFIER,
	.bcdUSB = {0x00, 0x02},
	.bDeviceClass = UDCLASS_HUB,
	.bDeviceSubClass = UDSUBCLASS_HUB,
	.bDeviceProtocol = UDPROTO_FSHUB,
	.bMaxPacketSize0 = 0,
	.bNumConfigurations = 0,
};

static const struct uss820dci_config_desc uss820dci_confd = {
	.confd = {
		.bLength = sizeof(struct usb2_config_descriptor),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength[0] = sizeof(uss820dci_confd),
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
		.bEndpointAddress = (UE_DIR_IN | USS820_DCI_INTR_ENDPT),
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 8,
		.bInterval = 255,
	},
};

static const struct usb2_hub_descriptor_min uss820dci_hubd = {
	.bDescLength = sizeof(uss820dci_hubd),
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
  'A', 0, 'G', 0, 'E', 0, 'R', 0, 'E', 0

#define	STRING_PRODUCT \
  'D', 0, 'C', 0, 'I', 0, ' ', 0, 'R', 0, \
  'o', 0, 'o', 0, 't', 0, ' ', 0, 'H', 0, \
  'U', 0, 'B', 0,

USB_MAKE_STRING_DESC(STRING_LANG, uss820dci_langtab);
USB_MAKE_STRING_DESC(STRING_VENDOR, uss820dci_vendor);
USB_MAKE_STRING_DESC(STRING_PRODUCT, uss820dci_product);

static void
uss820dci_root_ctrl_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_root_ctrl_start(struct usb2_xfer *xfer)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;

	sc->sc_root_ctrl.xfer = xfer;

	usb2_config_td_queue_command(
	    &(sc->sc_config_td), NULL, &uss820dci_root_ctrl_task, 0, 0);

	return;
}

static void
uss820dci_root_ctrl_task(struct uss820dci_softc *sc,
    struct uss820dci_config_copy *cc, uint16_t refcount)
{
	uss820dci_root_ctrl_poll(sc);
	return;
}

static void
uss820dci_root_ctrl_done(struct usb2_xfer *xfer,
    struct usb2_sw_transfer *std)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;
	uint16_t value;
	uint16_t index;
	uint8_t use_polling;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USB_SW_TR_SETUP) {
		if (std->state == USB_SW_TR_PRE_CALLBACK) {
			/* transfer transferred */
			uss820dci_device_done(xfer, std->err);
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
		std->len = sizeof(uss820dci_devd);
		std->ptr = USB_ADD_BYTES(&uss820dci_devd, 0);
		goto tr_valid;
	case UDESC_CONFIG:
		if (value & 0xff) {
			goto tr_stalled;
		}
		std->len = sizeof(uss820dci_confd);
		std->ptr = USB_ADD_BYTES(&uss820dci_confd, 0);
		goto tr_valid;
	case UDESC_STRING:
		switch (value & 0xff) {
		case 0:		/* Language table */
			std->len = sizeof(uss820dci_langtab);
			std->ptr = USB_ADD_BYTES(&uss820dci_langtab, 0);
			goto tr_valid;

		case 1:		/* Vendor */
			std->len = sizeof(uss820dci_vendor);
			std->ptr = USB_ADD_BYTES(&uss820dci_vendor, 0);
			goto tr_valid;

		case 2:		/* Product */
			std->len = sizeof(uss820dci_product);
			std->ptr = USB_ADD_BYTES(&uss820dci_product, 0);
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
	DPRINTF(8, "UR_CLEAR_PORT_FEATURE on port %d\n", index);

	switch (value) {
	case UHF_PORT_SUSPEND:
		uss820dci_wakeup_peer(sc);
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
		uss820dci_pull_down(sc);
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
	DPRINTF(8, "UR_SET_PORT_FEATURE\n");

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

	DPRINTF(8, "UR_GET_PORT_STATUS\n");

	if (index != 1) {
		goto tr_stalled;
	}
	if (sc->sc_flags.status_vbus) {
		uss820dci_pull_up(sc);
	} else {
		uss820dci_pull_down(sc);
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
	std->ptr = USB_ADD_BYTES(&uss820dci_hubd, 0);
	std->len = sizeof(uss820dci_hubd);
	goto tr_valid;

tr_stalled:
	std->err = USB_ERR_STALLED;
tr_valid:
done:
	return;
}

static void
uss820dci_root_ctrl_poll(struct uss820dci_softc *sc)
{
	usb2_sw_transfer(&(sc->sc_root_ctrl),
	    &uss820dci_root_ctrl_done);
	return;
}

struct usb2_pipe_methods uss820dci_root_ctrl_methods =
{
	.open = uss820dci_root_ctrl_open,
	.close = uss820dci_root_ctrl_close,
	.enter = uss820dci_root_ctrl_enter,
	.start = uss820dci_root_ctrl_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 0,
};

/*------------------------------------------------------------------------*
 * at91dci root interrupt support
 *------------------------------------------------------------------------*/
static void
uss820dci_root_intr_open(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_root_intr_close(struct usb2_xfer *xfer)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;

	if (sc->sc_root_intr.xfer == xfer) {
		sc->sc_root_intr.xfer = NULL;
	}
	uss820dci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
uss820dci_root_intr_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_root_intr_start(struct usb2_xfer *xfer)
{
	struct uss820dci_softc *sc = xfer->usb2_sc;

	sc->sc_root_intr.xfer = xfer;
	return;
}

struct usb2_pipe_methods uss820dci_root_intr_methods =
{
	.open = uss820dci_root_intr_open,
	.close = uss820dci_root_intr_close,
	.enter = uss820dci_root_intr_enter,
	.start = uss820dci_root_intr_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

static void
uss820dci_xfer_setup(struct usb2_setup_params *parm)
{
	const struct usb2_hw_ep_profile *pf;
	struct uss820dci_softc *sc;
	struct usb2_xfer *xfer;
	void *last_obj;
	uint32_t ntd;
	uint32_t n;
	uint8_t ep_no;

	sc = USS820_DCI_BUS2SC(parm->udev->bus);
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
	parm->hc_max_packet_size = 0x500;
	parm->hc_max_packet_count = 1;
	parm->hc_max_frame_size = 0x500;

	usb2_transfer_setup_sub(parm);

	/*
	 * compute maximum number of TDs
	 */
	if (parm->methods == &uss820dci_device_ctrl_methods) {

		ntd = xfer->nframes + 1 /* STATUS */ + 1 /* SYNC */ ;

	} else if (parm->methods == &uss820dci_device_bulk_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else if (parm->methods == &uss820dci_device_intr_methods) {

		ntd = xfer->nframes + 1 /* SYNC */ ;

	} else if (parm->methods == &uss820dci_device_isoc_fs_methods) {

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
		uss820dci_get_hw_ep_profile(xfer->udev, &pf, ep_no);

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

		struct uss820dci_td *td;

		if (parm->buf) {

			td = USB_ADD_BYTES(parm->buf, parm->size[0]);

			/* init TD */
			td->io_tag = sc->sc_io_tag;
			td->io_hdl = sc->sc_io_hdl;
			td->max_packet_size = xfer->max_packet_size;
			td->rx_stat_reg = USS820_GET_REG(sc, USS820_RXSTAT);
			td->tx_stat_reg = USS820_GET_REG(sc, USS820_TXSTAT);
			td->rx_flag_reg = USS820_GET_REG(sc, USS820_RXFLG);
			td->tx_flag_reg = USS820_GET_REG(sc, USS820_TXFLG);
			td->rx_fifo_reg = USS820_GET_REG(sc, USS820_RXDAT);
			td->tx_fifo_reg = USS820_GET_REG(sc, USS820_TXDAT);
			td->rx_count_low_reg = USS820_GET_REG(sc, USS820_RXCNTL);
			td->rx_count_high_reg = USS820_GET_REG(sc, USS820_RXCNTH);
			td->tx_count_low_reg = USS820_GET_REG(sc, USS820_TXCNTL);
			td->tx_count_high_reg = USS820_GET_REG(sc, USS820_TXCNTH);
			td->rx_cntl_reg = USS820_GET_REG(sc, USS820_RXCON);
			td->tx_cntl_reg = USS820_GET_REG(sc, USS820_TXCON);
			td->pend_reg = USS820_GET_REG(sc, USS820_PEND);
			td->ep_reg = USS820_GET_REG(sc, USS820_EPINDEX);
			td->ep_index = ep_no;
			if (pf->support_multi_buffer &&
			    (parm->methods != &uss820dci_device_ctrl_methods)) {
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
uss820dci_xfer_unsetup(struct usb2_xfer *xfer)
{
	return;
}

static void
uss820dci_pipe_init(struct usb2_device *udev, struct usb2_endpoint_descriptor *edesc,
    struct usb2_pipe *pipe)
{
	struct uss820dci_softc *sc = USS820_DCI_BUS2SC(udev->bus);

	DPRINTF(1, "pipe=%p, addr=%d, endpt=%d, mode=%d (%d)\n",
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
			pipe->methods = &uss820dci_root_ctrl_methods;
			break;
		case UE_DIR_IN | USS820_DCI_INTR_ENDPT:
			pipe->methods = &uss820dci_root_intr_methods;
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
		if (udev->speed != USB_SPEED_FULL) {
			/* not supported */
			return;
		}
		switch (edesc->bmAttributes & UE_XFERTYPE) {
		case UE_CONTROL:
			pipe->methods = &uss820dci_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &uss820dci_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			pipe->methods = &uss820dci_device_isoc_fs_methods;
			break;
		case UE_BULK:
			pipe->methods = &uss820dci_device_bulk_methods;
			break;
		default:
			/* do nothing */
			break;
		}
	}
	return;
}

struct usb2_bus_methods uss820dci_bus_methods =
{
	.pipe_init = &uss820dci_pipe_init,
	.xfer_setup = &uss820dci_xfer_setup,
	.xfer_unsetup = &uss820dci_xfer_unsetup,
	.do_poll = &uss820dci_do_poll,
	.get_hw_ep_profile = &uss820dci_get_hw_ep_profile,
	.set_stall = &uss820dci_set_stall,
	.clear_stall = &uss820dci_clear_stall,
	.rem_wakeup_set = &uss820dci_rem_wakeup_set,
};