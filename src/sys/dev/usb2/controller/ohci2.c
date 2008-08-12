/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
 * Copyright (c) 1998 The NetBSD Foundation, Inc. All rights reserved.
 * Copyright (c) 1998 Lennart Augustsson. All rights reserved.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ohci.c,v 1.172 2008/05/13 20:58:08 marius Exp $");

/*
 * USB Open Host Controller driver.
 *
 * OHCI spec: http://www.compaq.com/productinfo/development/openhci.html
 * USB spec: http://www.usb.org/developers/docs/usbspec.zip
 */

#include <dev/usb2/include/usb2_standard.h>
#include <dev/usb2/include/usb2_mfunc.h>
#include <dev/usb2/include/usb2_error.h>
#include <dev/usb2/include/usb2_defs.h>

#define	USB_DEBUG_VAR ohcidebug
#define	usb2_config_td_cc ohci_config_copy
#define	usb2_config_td_softc ohci_softc

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
#include <dev/usb2/controller/ohci2.h>

#define	OHCI_BUS2SC(bus) ((ohci_softc_t *)(((uint8_t *)(bus)) - \
   USB_P2U(&(((ohci_softc_t *)0)->sc_bus))))

#ifdef USB_DEBUG
static int ohcidebug = 0;

SYSCTL_NODE(_hw_usb2, OID_AUTO, ohci, CTLFLAG_RW, 0, "USB ohci");
SYSCTL_INT(_hw_usb2_ohci, OID_AUTO, debug, CTLFLAG_RW,
    &ohcidebug, 0, "ohci debug level");
static void ohci_dumpregs(ohci_softc_t *);
static void ohci_dump_tds(ohci_td_t *);
static uint8_t ohci_dump_td(ohci_td_t *);
static void ohci_dump_ed(ohci_ed_t *);
static uint8_t ohci_dump_itd(ohci_itd_t *);
static void ohci_dump_itds(ohci_itd_t *);

#endif

#define	OBARR(sc) bus_space_barrier((sc)->sc_io_tag, (sc)->sc_io_hdl, 0, (sc)->sc_io_size, \
			BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE)
#define	OWRITE1(sc, r, x) \
 do { OBARR(sc); bus_space_write_1((sc)->sc_io_tag, (sc)->sc_io_hdl, (r), (x)); } while (0)
#define	OWRITE2(sc, r, x) \
 do { OBARR(sc); bus_space_write_2((sc)->sc_io_tag, (sc)->sc_io_hdl, (r), (x)); } while (0)
#define	OWRITE4(sc, r, x) \
 do { OBARR(sc); bus_space_write_4((sc)->sc_io_tag, (sc)->sc_io_hdl, (r), (x)); } while (0)
#define	OREAD1(sc, r) (OBARR(sc), bus_space_read_1((sc)->sc_io_tag, (sc)->sc_io_hdl, (r)))
#define	OREAD2(sc, r) (OBARR(sc), bus_space_read_2((sc)->sc_io_tag, (sc)->sc_io_hdl, (r)))
#define	OREAD4(sc, r) (OBARR(sc), bus_space_read_4((sc)->sc_io_tag, (sc)->sc_io_hdl, (r)))

#define	OHCI_INTR_ENDPT 1

extern struct usb2_bus_methods ohci_bus_methods;
extern struct usb2_pipe_methods ohci_device_bulk_methods;
extern struct usb2_pipe_methods ohci_device_ctrl_methods;
extern struct usb2_pipe_methods ohci_device_intr_methods;
extern struct usb2_pipe_methods ohci_device_isoc_methods;
extern struct usb2_pipe_methods ohci_root_ctrl_methods;
extern struct usb2_pipe_methods ohci_root_intr_methods;

static usb2_config_td_command_t ohci_root_ctrl_task;
static void ohci_root_ctrl_poll(struct ohci_softc *sc);
static void ohci_do_poll(struct usb2_bus *bus);
static void ohci_device_done(struct usb2_xfer *xfer, usb2_error_t error);

static usb2_sw_transfer_func_t ohci_root_intr_done;
static usb2_sw_transfer_func_t ohci_root_ctrl_done;
static void ohci_timeout(void *arg);
static uint8_t ohci_check_transfer(struct usb2_xfer *xfer);

struct ohci_std_temp {
	struct usb2_page_cache *pc;
	ohci_td_t *td;
	ohci_td_t *td_next;
	uint32_t average;
	uint32_t td_flags;
	uint32_t len;
	uint16_t max_frame_size;
	uint8_t	shortpkt;
	uint8_t	setup_alt_next;
	uint8_t	short_frames_ok;
};

static struct ohci_hcca *
ohci_get_hcca(ohci_softc_t *sc)
{
	usb2_pc_cpu_invalidate(&sc->sc_hw.hcca_pc);
	return (sc->sc_hcca_p);
}

void
ohci_iterate_hw_softc(struct usb2_bus *bus, usb2_bus_mem_sub_cb_t *cb)
{
	struct ohci_softc *sc = OHCI_BUS2SC(bus);
	uint32_t i;

	cb(bus, &sc->sc_hw.hcca_pc, &(sc->sc_hw.hcca_pg),
	    sizeof(ohci_hcca_t), OHCI_HCCA_ALIGN);

	cb(bus, &sc->sc_hw.ctrl_start_pc, &(sc->sc_hw.ctrl_start_pg),
	    sizeof(ohci_ed_t), OHCI_ED_ALIGN);

	cb(bus, &sc->sc_hw.bulk_start_pc, &(sc->sc_hw.bulk_start_pg),
	    sizeof(ohci_ed_t), OHCI_ED_ALIGN);

	cb(bus, &sc->sc_hw.isoc_start_pc, &(sc->sc_hw.isoc_start_pg),
	    sizeof(ohci_ed_t), OHCI_ED_ALIGN);

	for (i = 0; i != OHCI_NO_EDS; i++) {
		cb(bus, sc->sc_hw.intr_start_pc + i, sc->sc_hw.intr_start_pg + i,
		    sizeof(ohci_ed_t), OHCI_ED_ALIGN);
	}
	return;
}

static usb2_error_t
ohci_controller_init(ohci_softc_t *sc)
{
	struct usb2_page_search buf_res;
	uint32_t i;
	uint32_t ctl;
	uint32_t ival;
	uint32_t hcr;
	uint32_t fm;
	uint32_t per;
	uint32_t desca;

	/* Determine in what context we are running. */
	ctl = OREAD4(sc, OHCI_CONTROL);
	if (ctl & OHCI_IR) {
		/* SMM active, request change */
		DPRINTF("SMM active, request owner change\n");
		OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_OCR);
		for (i = 0; (i < 100) && (ctl & OHCI_IR); i++) {
			DELAY(1000 * 1);
			ctl = OREAD4(sc, OHCI_CONTROL);
		}
		if (ctl & OHCI_IR) {
			device_printf(sc->sc_bus.bdev,
			    "SMM does not respond, resetting\n");
			OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
			goto reset;
		}
	} else {
		DPRINTF("cold started\n");
reset:
		/* controller was cold started */
		DELAY(1000 * USB_BUS_RESET_DELAY);
	}

	/*
	 * This reset should not be necessary according to the OHCI spec, but
	 * without it some controllers do not start.
	 */
	DPRINTF("%s: resetting\n", device_get_nameunit(sc->sc_bus.bdev));
	OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
	DELAY(1000 * USB_BUS_RESET_DELAY);

	/* we now own the host controller and the bus has been reset */
	ival = OHCI_GET_IVAL(OREAD4(sc, OHCI_FM_INTERVAL));

	OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_HCR);	/* Reset HC */
	/* nominal time for a reset is 10 us */
	for (i = 0; i < 10; i++) {
		DELAY(10);
		hcr = OREAD4(sc, OHCI_COMMAND_STATUS) & OHCI_HCR;
		if (!hcr) {
			break;
		}
	}
	if (hcr) {
		device_printf(sc->sc_bus.bdev, "reset timeout\n");
		return (USB_ERR_IOERROR);
	}
#ifdef USB_DEBUG
	if (ohcidebug > 15) {
		ohci_dumpregs(sc);
	}
#endif

	/* The controller is now in SUSPEND state, we have 2ms to finish. */

	/* set up HC registers */
	usb2_get_page(&sc->sc_hw.hcca_pc, 0, &buf_res);
	OWRITE4(sc, OHCI_HCCA, buf_res.physaddr);

	usb2_get_page(&sc->sc_hw.ctrl_start_pc, 0, &buf_res);
	OWRITE4(sc, OHCI_CONTROL_HEAD_ED, buf_res.physaddr);

	usb2_get_page(&sc->sc_hw.bulk_start_pc, 0, &buf_res);
	OWRITE4(sc, OHCI_BULK_HEAD_ED, buf_res.physaddr);

	/* disable all interrupts and then switch on all desired interrupts */
	OWRITE4(sc, OHCI_INTERRUPT_DISABLE, OHCI_ALL_INTRS);
	OWRITE4(sc, OHCI_INTERRUPT_ENABLE, sc->sc_eintrs | OHCI_MIE);
	/* switch on desired functional features */
	ctl = OREAD4(sc, OHCI_CONTROL);
	ctl &= ~(OHCI_CBSR_MASK | OHCI_LES | OHCI_HCFS_MASK | OHCI_IR);
	ctl |= OHCI_PLE | OHCI_IE | OHCI_CLE | OHCI_BLE |
	    OHCI_RATIO_1_4 | OHCI_HCFS_OPERATIONAL;
	/* And finally start it! */
	OWRITE4(sc, OHCI_CONTROL, ctl);

	/*
	 * The controller is now OPERATIONAL.  Set a some final
	 * registers that should be set earlier, but that the
	 * controller ignores when in the SUSPEND state.
	 */
	fm = (OREAD4(sc, OHCI_FM_INTERVAL) & OHCI_FIT) ^ OHCI_FIT;
	fm |= OHCI_FSMPS(ival) | ival;
	OWRITE4(sc, OHCI_FM_INTERVAL, fm);
	per = OHCI_PERIODIC(ival);	/* 90% periodic */
	OWRITE4(sc, OHCI_PERIODIC_START, per);

	/* Fiddle the No OverCurrent Protection bit to avoid chip bug. */
	desca = OREAD4(sc, OHCI_RH_DESCRIPTOR_A);
	OWRITE4(sc, OHCI_RH_DESCRIPTOR_A, desca | OHCI_NOCP);
	OWRITE4(sc, OHCI_RH_STATUS, OHCI_LPSC);	/* Enable port power */
	DELAY(1000 * OHCI_ENABLE_POWER_DELAY);
	OWRITE4(sc, OHCI_RH_DESCRIPTOR_A, desca);

	/*
	 * The AMD756 requires a delay before re-reading the register,
	 * otherwise it will occasionally report 0 ports.
	 */
	sc->sc_noport = 0;
	for (i = 0; (i < 10) && (sc->sc_noport == 0); i++) {
		DELAY(1000 * OHCI_READ_DESC_DELAY);
		sc->sc_noport = OHCI_GET_NDP(OREAD4(sc, OHCI_RH_DESCRIPTOR_A));
	}

#ifdef USB_DEBUG
	if (ohcidebug > 5) {
		ohci_dumpregs(sc);
	}
#endif
	return (USB_ERR_NORMAL_COMPLETION);
}

static struct ohci_ed *
ohci_init_ed(struct usb2_page_cache *pc)
{
	struct usb2_page_search buf_res;
	struct ohci_ed *ed;

	usb2_get_page(pc, 0, &buf_res);

	ed = buf_res.buffer;

	ed->ed_self = htole32(buf_res.physaddr);
	ed->ed_flags = htole32(OHCI_ED_SKIP);
	ed->page_cache = pc;

	return (ed);
}

usb2_error_t
ohci_init(ohci_softc_t *sc)
{
	struct usb2_page_search buf_res;
	uint16_t i;
	uint16_t bit;
	uint16_t x;
	uint16_t y;

	mtx_lock(&sc->sc_bus.mtx);

	DPRINTF("start\n");

	sc->sc_eintrs = OHCI_NORMAL_INTRS;

	/*
	 * Setup all ED's
	 */

	sc->sc_ctrl_p_last =
	    ohci_init_ed(&sc->sc_hw.ctrl_start_pc);

	sc->sc_bulk_p_last =
	    ohci_init_ed(&sc->sc_hw.bulk_start_pc);

	sc->sc_isoc_p_last =
	    ohci_init_ed(&sc->sc_hw.isoc_start_pc);

	for (i = 0; i != OHCI_NO_EDS; i++) {
		sc->sc_intr_p_last[i] =
		    ohci_init_ed(sc->sc_hw.intr_start_pc + i);
	}

	/*
	 * the QHs are arranged to give poll intervals that are
	 * powers of 2 times 1ms
	 */
	bit = OHCI_NO_EDS / 2;
	while (bit) {
		x = bit;
		while (x & bit) {
			ohci_ed_t *ed_x;
			ohci_ed_t *ed_y;

			y = (x ^ bit) | (bit / 2);

			/*
			 * the next QH has half the poll interval
			 */
			ed_x = sc->sc_intr_p_last[x];
			ed_y = sc->sc_intr_p_last[y];

			ed_x->next = NULL;
			ed_x->ed_next = ed_y->ed_self;

			x++;
		}
		bit >>= 1;
	}

	if (1) {

		ohci_ed_t *ed_int;
		ohci_ed_t *ed_isc;

		ed_int = sc->sc_intr_p_last[0];
		ed_isc = sc->sc_isoc_p_last;

		/* the last (1ms) QH */
		ed_int->next = ed_isc;
		ed_int->ed_next = ed_isc->ed_self;
	}
	usb2_get_page(&sc->sc_hw.hcca_pc, 0, &buf_res);

	sc->sc_hcca_p = buf_res.buffer;

	/*
	 * Fill HCCA interrupt table.  The bit reversal is to get
	 * the tree set up properly to spread the interrupts.
	 */
	for (i = 0; i != OHCI_NO_INTRS; i++) {
		sc->sc_hcca_p->hcca_interrupt_table[i] =
		    sc->sc_intr_p_last[i | (OHCI_NO_EDS / 2)]->ed_self;
	}
	/* flush all cache into memory */

	usb2_bus_mem_flush_all(&sc->sc_bus, &ohci_iterate_hw_softc);

	/* set up the bus struct */
	sc->sc_bus.methods = &ohci_bus_methods;

	usb2_callout_init_mtx(&sc->sc_tmo_rhsc, &sc->sc_bus.mtx,
	    CALLOUT_RETURNUNLOCKED);

#ifdef USB_DEBUG
	if (ohcidebug > 15) {
		for (i = 0; i != OHCI_NO_EDS; i++) {
			printf("ed#%d ", i);
			ohci_dump_ed(sc->sc_intr_p_last[i]);
		}
		printf("iso ");
		ohci_dump_ed(sc->sc_isoc_p_last);
	}
#endif

	sc->sc_bus.usbrev = USB_REV_1_0;

	if (ohci_controller_init(sc)) {
		mtx_unlock(&sc->sc_bus.mtx);
		return (USB_ERR_INVAL);
	} else {
		mtx_unlock(&sc->sc_bus.mtx);
		/* catch any lost interrupts */
		ohci_do_poll(&sc->sc_bus);
		return (USB_ERR_NORMAL_COMPLETION);
	}
}

/*
 * shut down the controller when the system is going down
 */
void
ohci_detach(struct ohci_softc *sc)
{
	mtx_lock(&sc->sc_bus.mtx);

	usb2_callout_stop(&sc->sc_tmo_rhsc);

	OWRITE4(sc, OHCI_INTERRUPT_DISABLE, OHCI_ALL_INTRS);
	OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);

	mtx_unlock(&sc->sc_bus.mtx);

	DELAY(1000 * 50);		/* XXX let stray task complete */

	usb2_callout_drain(&sc->sc_tmo_rhsc);

	return;
}

/* NOTE: suspend/resume is called from
 * interrupt context and cannot sleep!
 */
void
ohci_suspend(ohci_softc_t *sc)
{
	uint32_t ctl;

	mtx_lock(&sc->sc_bus.mtx);

#ifdef USB_DEBUG
	DPRINTF("\n");
	if (ohcidebug > 2) {
		ohci_dumpregs(sc);
	}
#endif

	ctl = OREAD4(sc, OHCI_CONTROL) & ~OHCI_HCFS_MASK;
	if (sc->sc_control == 0) {
		/*
		 * Preserve register values, in case that APM BIOS
		 * does not recover them.
		 */
		sc->sc_control = ctl;
		sc->sc_intre = OREAD4(sc, OHCI_INTERRUPT_ENABLE);
	}
	ctl |= OHCI_HCFS_SUSPEND;
	OWRITE4(sc, OHCI_CONTROL, ctl);
	DELAY(1000 * USB_RESUME_WAIT);

	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

void
ohci_resume(ohci_softc_t *sc)
{
	uint32_t ctl;

	mtx_lock(&sc->sc_bus.mtx);

#ifdef USB_DEBUG
	DPRINTF("\n");
	if (ohcidebug > 2) {
		ohci_dumpregs(sc);
	}
#endif
	/* some broken BIOSes never initialize the Controller chip */
	ohci_controller_init(sc);

	if (sc->sc_intre) {
		OWRITE4(sc, OHCI_INTERRUPT_ENABLE,
		    sc->sc_intre & (OHCI_ALL_INTRS | OHCI_MIE));
	}
	if (sc->sc_control)
		ctl = sc->sc_control;
	else
		ctl = OREAD4(sc, OHCI_CONTROL);
	ctl |= OHCI_HCFS_RESUME;
	OWRITE4(sc, OHCI_CONTROL, ctl);
	DELAY(1000 * USB_RESUME_DELAY);
	ctl = (ctl & ~OHCI_HCFS_MASK) | OHCI_HCFS_OPERATIONAL;
	OWRITE4(sc, OHCI_CONTROL, ctl);
	DELAY(1000 * USB_RESUME_RECOVERY);
	sc->sc_control = sc->sc_intre = 0;

	mtx_unlock(&sc->sc_bus.mtx);

	/* catch any lost interrupts */
	ohci_do_poll(&sc->sc_bus);

	return;
}

#ifdef USB_DEBUG
static void
ohci_dumpregs(ohci_softc_t *sc)
{
	struct ohci_hcca *hcca;

	DPRINTF("ohci_dumpregs: rev=0x%08x control=0x%08x command=0x%08x\n",
	    OREAD4(sc, OHCI_REVISION),
	    OREAD4(sc, OHCI_CONTROL),
	    OREAD4(sc, OHCI_COMMAND_STATUS));
	DPRINTF("               intrstat=0x%08x intre=0x%08x intrd=0x%08x\n",
	    OREAD4(sc, OHCI_INTERRUPT_STATUS),
	    OREAD4(sc, OHCI_INTERRUPT_ENABLE),
	    OREAD4(sc, OHCI_INTERRUPT_DISABLE));
	DPRINTF("               hcca=0x%08x percur=0x%08x ctrlhd=0x%08x\n",
	    OREAD4(sc, OHCI_HCCA),
	    OREAD4(sc, OHCI_PERIOD_CURRENT_ED),
	    OREAD4(sc, OHCI_CONTROL_HEAD_ED));
	DPRINTF("               ctrlcur=0x%08x bulkhd=0x%08x bulkcur=0x%08x\n",
	    OREAD4(sc, OHCI_CONTROL_CURRENT_ED),
	    OREAD4(sc, OHCI_BULK_HEAD_ED),
	    OREAD4(sc, OHCI_BULK_CURRENT_ED));
	DPRINTF("               done=0x%08x fmival=0x%08x fmrem=0x%08x\n",
	    OREAD4(sc, OHCI_DONE_HEAD),
	    OREAD4(sc, OHCI_FM_INTERVAL),
	    OREAD4(sc, OHCI_FM_REMAINING));
	DPRINTF("               fmnum=0x%08x perst=0x%08x lsthrs=0x%08x\n",
	    OREAD4(sc, OHCI_FM_NUMBER),
	    OREAD4(sc, OHCI_PERIODIC_START),
	    OREAD4(sc, OHCI_LS_THRESHOLD));
	DPRINTF("               desca=0x%08x descb=0x%08x stat=0x%08x\n",
	    OREAD4(sc, OHCI_RH_DESCRIPTOR_A),
	    OREAD4(sc, OHCI_RH_DESCRIPTOR_B),
	    OREAD4(sc, OHCI_RH_STATUS));
	DPRINTF("               port1=0x%08x port2=0x%08x\n",
	    OREAD4(sc, OHCI_RH_PORT_STATUS(1)),
	    OREAD4(sc, OHCI_RH_PORT_STATUS(2)));

	hcca = ohci_get_hcca(sc);

	DPRINTF("         HCCA: frame_number=0x%04x done_head=0x%08x\n",
	    le32toh(hcca->hcca_frame_number),
	    le32toh(hcca->hcca_done_head));
	return;
}
static void
ohci_dump_tds(ohci_td_t *std)
{
	for (; std; std = std->obj_next) {
		if (ohci_dump_td(std)) {
			break;
		}
	}
	return;
}

static uint8_t
ohci_dump_td(ohci_td_t *std)
{
	uint32_t td_flags;
	uint8_t temp;

	usb2_pc_cpu_invalidate(std->page_cache);

	td_flags = le32toh(std->td_flags);
	temp = (std->td_next == 0);

	printf("TD(%p) at 0x%08x: %s%s%s%s%s delay=%d ec=%d "
	    "cc=%d\ncbp=0x%08x next=0x%08x be=0x%08x\n",
	    std, le32toh(std->td_self),
	    (td_flags & OHCI_TD_R) ? "-R" : "",
	    (td_flags & OHCI_TD_OUT) ? "-OUT" : "",
	    (td_flags & OHCI_TD_IN) ? "-IN" : "",
	    ((td_flags & OHCI_TD_TOGGLE_MASK) == OHCI_TD_TOGGLE_1) ? "-TOG1" : "",
	    ((td_flags & OHCI_TD_TOGGLE_MASK) == OHCI_TD_TOGGLE_0) ? "-TOG0" : "",
	    OHCI_TD_GET_DI(td_flags),
	    OHCI_TD_GET_EC(td_flags),
	    OHCI_TD_GET_CC(td_flags),
	    le32toh(std->td_cbp),
	    le32toh(std->td_next),
	    le32toh(std->td_be));

	return (temp);
}

static uint8_t
ohci_dump_itd(ohci_itd_t *sitd)
{
	uint32_t itd_flags;
	uint16_t i;
	uint8_t temp;

	usb2_pc_cpu_invalidate(sitd->page_cache);

	itd_flags = le32toh(sitd->itd_flags);
	temp = (sitd->itd_next == 0);

	printf("ITD(%p) at 0x%08x: sf=%d di=%d fc=%d cc=%d\n"
	    "bp0=0x%08x next=0x%08x be=0x%08x\n",
	    sitd, le32toh(sitd->itd_self),
	    OHCI_ITD_GET_SF(itd_flags),
	    OHCI_ITD_GET_DI(itd_flags),
	    OHCI_ITD_GET_FC(itd_flags),
	    OHCI_ITD_GET_CC(itd_flags),
	    le32toh(sitd->itd_bp0),
	    le32toh(sitd->itd_next),
	    le32toh(sitd->itd_be));
	for (i = 0; i < OHCI_ITD_NOFFSET; i++) {
		printf("offs[%d]=0x%04x ", i,
		    (uint32_t)le16toh(sitd->itd_offset[i]));
	}
	printf("\n");

	return (temp);
}

static void
ohci_dump_itds(ohci_itd_t *sitd)
{
	for (; sitd; sitd = sitd->obj_next) {
		if (ohci_dump_itd(sitd)) {
			break;
		}
	}
	return;
}

static void
ohci_dump_ed(ohci_ed_t *sed)
{
	uint32_t ed_flags;
	uint32_t ed_headp;

	usb2_pc_cpu_invalidate(sed->page_cache);

	ed_flags = le32toh(sed->ed_flags);
	ed_headp = le32toh(sed->ed_headp);

	printf("ED(%p) at 0x%08x: addr=%d endpt=%d maxp=%d flags=%s%s%s%s%s\n"
	    "tailp=0x%08x headflags=%s%s headp=0x%08x nexted=0x%08x\n",
	    sed, le32toh(sed->ed_self),
	    OHCI_ED_GET_FA(ed_flags),
	    OHCI_ED_GET_EN(ed_flags),
	    OHCI_ED_GET_MAXP(ed_flags),
	    (ed_flags & OHCI_ED_DIR_OUT) ? "-OUT" : "",
	    (ed_flags & OHCI_ED_DIR_IN) ? "-IN" : "",
	    (ed_flags & OHCI_ED_SPEED) ? "-LOWSPEED" : "",
	    (ed_flags & OHCI_ED_SKIP) ? "-SKIP" : "",
	    (ed_flags & OHCI_ED_FORMAT_ISO) ? "-ISO" : "",
	    le32toh(sed->ed_tailp),
	    (ed_headp & OHCI_HALTED) ? "-HALTED" : "",
	    (ed_headp & OHCI_TOGGLECARRY) ? "-CARRY" : "",
	    le32toh(sed->ed_headp),
	    le32toh(sed->ed_next));
	return;
}

#endif

static void
ohci_transfer_intr_enqueue(struct usb2_xfer *xfer)
{
	/* check for early completion */
	if (ohci_check_transfer(xfer)) {
		return;
	}
	/* put transfer on interrupt queue */
	usb2_transfer_enqueue(&xfer->udev->bus->intr_q, xfer);

	/* start timeout, if any */
	if (xfer->timeout != 0) {
		usb2_transfer_timeout_ms(xfer, &ohci_timeout, xfer->timeout);
	}
	return;
}

#define	OHCI_APPEND_QH(sed,td_self,last) (last) = _ohci_append_qh(sed,td_self,last)
static ohci_ed_t *
_ohci_append_qh(ohci_ed_t *sed, uint32_t td_self, ohci_ed_t *last)
{
	DPRINTFN(11, "%p to %p\n", sed, last);

	/* (sc->sc_bus.mtx) must be locked */

	sed->next = last->next;
	sed->ed_next = last->ed_next;
	sed->ed_tailp = 0;
	sed->ed_headp = td_self;

	sed->prev = last;

	usb2_pc_cpu_flush(sed->page_cache);

	/*
	 * the last->next->prev is never followed: sed->next->prev = sed;
	 */

	last->next = sed;
	last->ed_next = sed->ed_self;

	usb2_pc_cpu_flush(last->page_cache);

	return (sed);
}

#define	OHCI_REMOVE_QH(sed,last) (last) = _ohci_remove_qh(sed,last)
static ohci_ed_t *
_ohci_remove_qh(ohci_ed_t *sed, ohci_ed_t *last)
{
	DPRINTFN(11, "%p from %p\n", sed, last);

	/* (sc->sc_bus.mtx) must be locked */

	/* only remove if not removed from a queue */
	if (sed->prev) {

		sed->prev->next = sed->next;
		sed->prev->ed_next = sed->ed_next;

		usb2_pc_cpu_flush(sed->prev->page_cache);

		if (sed->next) {
			sed->next->prev = sed->prev;
			usb2_pc_cpu_flush(sed->next->page_cache);
		}
		/*
		 * terminate transfer in case the transferred packet was
		 * short so that the ED still points at the last used TD
		 */
		sed->ed_flags |= htole32(OHCI_ED_SKIP);
		sed->ed_headp = sed->ed_tailp;

		last = ((last == sed) ? sed->prev : last);

		sed->prev = 0;

		usb2_pc_cpu_flush(sed->page_cache);
	}
	return (last);
}

static void
ohci_isoc_done(struct usb2_xfer *xfer)
{
	uint8_t nframes;
	uint32_t *plen = xfer->frlengths;
	volatile uint16_t *olen;
	uint16_t len = 0;
	ohci_itd_t *td = xfer->td_transfer_first;

	while (1) {
		if (td == NULL) {
			panic("%s:%d: out of TD's\n",
			    __FUNCTION__, __LINE__);
		}
#ifdef USB_DEBUG
		if (ohcidebug > 5) {
			DPRINTF("isoc TD\n");
			ohci_dump_itd(td);
		}
#endif
		usb2_pc_cpu_invalidate(td->page_cache);

		nframes = td->frames;
		olen = &td->itd_offset[0];

		if (nframes > 8) {
			nframes = 8;
		}
		while (nframes--) {
			len = le16toh(*olen);

			if ((len >> 12) == OHCI_CC_NOT_ACCESSED) {
				len = 0;
			} else {
				len &= ((1 << 12) - 1);
			}

			if (len > *plen) {
				len = 0;/* invalid length */
			}
			*plen = len;
			plen++;
			olen++;
		}

		if (((void *)td) == xfer->td_transfer_last) {
			break;
		}
		td = td->obj_next;
	}

	xfer->aframes = xfer->nframes;
	ohci_device_done(xfer, USB_ERR_NORMAL_COMPLETION);
	return;
}

#ifdef USB_DEBUG
static const char *const
	ohci_cc_strs[] =
{
	"NO_ERROR",
	"CRC",
	"BIT_STUFFING",
	"DATA_TOGGLE_MISMATCH",

	"STALL",
	"DEVICE_NOT_RESPONDING",
	"PID_CHECK_FAILURE",
	"UNEXPECTED_PID",

	"DATA_OVERRUN",
	"DATA_UNDERRUN",
	"BUFFER_OVERRUN",
	"BUFFER_UNDERRUN",

	"reserved",
	"reserved",
	"NOT_ACCESSED",
	"NOT_ACCESSED"
};

#endif

static usb2_error_t
ohci_non_isoc_done_sub(struct usb2_xfer *xfer)
{
	ohci_td_t *td;
	ohci_td_t *td_alt_next;
	uint32_t temp;
	uint32_t phy_start;
	uint32_t phy_end;
	uint32_t td_flags;
	uint16_t cc;

	td = xfer->td_transfer_cache;
	td_alt_next = td->alt_next;
	td_flags = 0;

	while (1) {

		usb2_pc_cpu_invalidate(td->page_cache);
		phy_start = le32toh(td->td_cbp);
		td_flags = le32toh(td->td_flags);
		cc = OHCI_TD_GET_CC(td_flags);

		if (phy_start) {
			/*
			 * short transfer - compute the number of remaining
			 * bytes in the hardware buffer:
			 */
			phy_end = le32toh(td->td_be);
			temp = (OHCI_PAGE(phy_start ^ phy_end) ?
			    (OHCI_PAGE_SIZE + 1) : 0x0001);
			temp += OHCI_PAGE_OFFSET(phy_end);
			temp -= OHCI_PAGE_OFFSET(phy_start);

			if (temp > td->len) {
				/* guard against corruption */
				cc = OHCI_CC_STALL;
			} else if (xfer->aframes != xfer->nframes) {
				/*
				 * subtract remaining length from
				 * "frlengths[]"
				 */
				xfer->frlengths[xfer->aframes] -= temp;
			}
		}
		/* Check for last transfer */
		if (((void *)td) == xfer->td_transfer_last) {
			td = NULL;
			break;
		}
		/* Check transfer status */
		if (cc) {
			/* the transfer is finished */
			td = NULL;
			break;
		}
		/* Check for short transfer */
		if (phy_start) {
			if (xfer->flags_int.short_frames_ok) {
				/* follow alt next */
				td = td->alt_next;
			} else {
				/* the transfer is finished */
				td = NULL;
			}
			break;
		}
		td = td->obj_next;

		if (td->alt_next != td_alt_next) {
			/* this USB frame is complete */
			break;
		}
	}

	/* update transfer cache */

	xfer->td_transfer_cache = td;

	DPRINTFN(16, "error cc=%d (%s)\n",
	    cc, ohci_cc_strs[cc]);

	return ((cc == 0) ? USB_ERR_NORMAL_COMPLETION :
	    (cc == OHCI_CC_STALL) ? USB_ERR_STALLED : USB_ERR_IOERROR);
}

static void
ohci_non_isoc_done(struct usb2_xfer *xfer)
{
	usb2_error_t err = 0;

	DPRINTFN(13, "xfer=%p pipe=%p transfer done\n",
	    xfer, xfer->pipe);

#ifdef USB_DEBUG
	if (ohcidebug > 10) {
		ohci_dump_tds(xfer->td_transfer_first);
	}
#endif

	/* reset scanner */

	xfer->td_transfer_cache = xfer->td_transfer_first;

	if (xfer->flags_int.control_xfr) {

		if (xfer->flags_int.control_hdr) {

			err = ohci_non_isoc_done_sub(xfer);
		}
		xfer->aframes = 1;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}
	while (xfer->aframes != xfer->nframes) {

		err = ohci_non_isoc_done_sub(xfer);
		xfer->aframes++;

		if (xfer->td_transfer_cache == NULL) {
			goto done;
		}
	}

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		err = ohci_non_isoc_done_sub(xfer);
	}
done:
	ohci_device_done(xfer, err);
	return;
}

/*------------------------------------------------------------------------*
 *	ohci_check_transfer_sub
 *------------------------------------------------------------------------*/
static void
ohci_check_transfer_sub(struct usb2_xfer *xfer)
{
	ohci_td_t *td;
	ohci_ed_t *ed;
	uint32_t phy_start;
	uint32_t td_flags;
	uint32_t td_next;
	uint16_t cc;

	td = xfer->td_transfer_cache;

	while (1) {

		usb2_pc_cpu_invalidate(td->page_cache);
		phy_start = le32toh(td->td_cbp);
		td_flags = le32toh(td->td_flags);
		td_next = le32toh(td->td_next);

		/* Check for last transfer */
		if (((void *)td) == xfer->td_transfer_last) {
			/* the transfer is finished */
			td = NULL;
			break;
		}
		/* Check transfer status */
		cc = OHCI_TD_GET_CC(td_flags);
		if (cc) {
			/* the transfer is finished */
			td = NULL;
			break;
		}
		/*
	         * Check if we reached the last packet
	         * or if there is a short packet:
	         */

		if (((td_next & (~0xF)) == OHCI_TD_NEXT_END) || phy_start) {
			/* follow alt next */
			td = td->alt_next;
			break;
		}
		td = td->obj_next;
	}

	/* update transfer cache */

	xfer->td_transfer_cache = td;

	if (td) {

		ed = xfer->qh_start[xfer->flags_int.curr_dma_set];

		ed->ed_headp = td->td_self;
		usb2_pc_cpu_flush(ed->page_cache);

		DPRINTFN(13, "xfer=%p following alt next\n", xfer);
	}
	return;
}

/*------------------------------------------------------------------------*
 *	ohci_check_transfer
 *
 * Return values:
 *    0: USB transfer is not finished
 * Else: USB transfer is finished
 *------------------------------------------------------------------------*/
static uint8_t
ohci_check_transfer(struct usb2_xfer *xfer)
{
	ohci_ed_t *ed;
	uint32_t ed_flags;
	uint32_t ed_headp;
	uint32_t ed_tailp;

	DPRINTFN(13, "xfer=%p checking transfer\n", xfer);

	ed = xfer->qh_start[xfer->flags_int.curr_dma_set];

	usb2_pc_cpu_invalidate(ed->page_cache);
	ed_flags = le32toh(ed->ed_flags);
	ed_headp = le32toh(ed->ed_headp);
	ed_tailp = le32toh(ed->ed_tailp);

	if ((ed_flags & OHCI_ED_SKIP) ||
	    (ed_headp & OHCI_HALTED) ||
	    (((ed_headp ^ ed_tailp) & (~0xF)) == 0)) {
		if (xfer->pipe->methods == &ohci_device_isoc_methods) {
			/* isochronous transfer */
			ohci_isoc_done(xfer);
		} else {
			if (xfer->flags_int.short_frames_ok) {
				ohci_check_transfer_sub(xfer);
				if (xfer->td_transfer_cache) {
					/* not finished yet */
					return (0);
				}
			}
			/* store data-toggle */
			if (ed_headp & OHCI_TOGGLECARRY) {
				xfer->pipe->toggle_next = 1;
			} else {
				xfer->pipe->toggle_next = 0;
			}

			/* non-isochronous transfer */
			ohci_non_isoc_done(xfer);
		}
		return (1);
	}
	DPRINTFN(13, "xfer=%p is still active\n", xfer);
	return (0);
}

static void
ohci_rhsc_enable(ohci_softc_t *sc)
{
	DPRINTFN(5, "\n");

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	sc->sc_eintrs |= OHCI_RHSC;
	OWRITE4(sc, OHCI_INTERRUPT_ENABLE, OHCI_RHSC);

	/* acknowledge any RHSC interrupt */
	OWRITE4(sc, OHCI_INTERRUPT_STATUS, OHCI_RHSC);

	usb2_sw_transfer(&sc->sc_root_intr,
	    &ohci_root_intr_done);

	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

static void
ohci_interrupt_poll(ohci_softc_t *sc)
{
	struct usb2_xfer *xfer;

repeat:
	TAILQ_FOREACH(xfer, &sc->sc_bus.intr_q.head, wait_entry) {
		/*
		 * check if transfer is transferred
		 */
		if (ohci_check_transfer(xfer)) {
			/* queue has been modified */
			goto repeat;
		}
	}
	return;
}

/*------------------------------------------------------------------------*
 *	ohci_interrupt - OHCI interrupt handler
 *
 * NOTE: Do not access "sc->sc_bus.bdev" inside the interrupt handler,
 * hence the interrupt handler will be setup before "sc->sc_bus.bdev"
 * is present !
 *------------------------------------------------------------------------*/
void
ohci_interrupt(ohci_softc_t *sc)
{
	struct ohci_hcca *hcca;
	uint32_t status;
	uint32_t done;

	mtx_lock(&sc->sc_bus.mtx);

	hcca = ohci_get_hcca(sc);

	DPRINTFN(16, "real interrupt\n");

#ifdef USB_DEBUG
	if (ohcidebug > 15) {
		ohci_dumpregs(sc);
	}
#endif

	done = le32toh(hcca->hcca_done_head);

	/*
	 * The LSb of done is used to inform the HC Driver that an interrupt
	 * condition exists for both the Done list and for another event
	 * recorded in HcInterruptStatus. On an interrupt from the HC, the
	 * HC Driver checks the HccaDoneHead Value. If this value is 0, then
	 * the interrupt was caused by other than the HccaDoneHead update
	 * and the HcInterruptStatus register needs to be accessed to
	 * determine that exact interrupt cause. If HccaDoneHead is nonzero,
	 * then a Done list update interrupt is indicated and if the LSb of
	 * done is nonzero, then an additional interrupt event is indicated
	 * and HcInterruptStatus should be checked to determine its cause.
	 */
	if (done != 0) {
		status = 0;

		if (done & ~OHCI_DONE_INTRS) {
			status |= OHCI_WDH;
		}
		if (done & OHCI_DONE_INTRS) {
			status |= OREAD4(sc, OHCI_INTERRUPT_STATUS);
		}
		hcca->hcca_done_head = 0;

		usb2_pc_cpu_flush(&sc->sc_hw.hcca_pc);
	} else {
		status = OREAD4(sc, OHCI_INTERRUPT_STATUS) & ~OHCI_WDH;
	}

	status &= ~OHCI_MIE;
	if (status == 0) {
		/*
		 * nothing to be done (PCI shared
		 * interrupt)
		 */
		goto done;
	}
	OWRITE4(sc, OHCI_INTERRUPT_STATUS, status);	/* Acknowledge */

	status &= sc->sc_eintrs;
	if (status == 0) {
		goto done;
	}
	if (status & (OHCI_SO | OHCI_RD | OHCI_UE | OHCI_RHSC)) {
#if 0
		if (status & OHCI_SO) {
			/* XXX do what */
		}
#endif
		if (status & OHCI_RD) {
			printf("%s: resume detect\n", __FUNCTION__);
			/* XXX process resume detect */
		}
		if (status & OHCI_UE) {
			printf("%s: unrecoverable error, "
			    "controller halted\n", __FUNCTION__);
			OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
			/* XXX what else */
		}
		if (status & OHCI_RHSC) {
			/*
			 * Disable RHSC interrupt for now, because it will be
			 * on until the port has been reset.
			 */
			sc->sc_eintrs &= ~OHCI_RHSC;
			OWRITE4(sc, OHCI_INTERRUPT_DISABLE, OHCI_RHSC);

			usb2_sw_transfer(&sc->sc_root_intr,
			    &ohci_root_intr_done);

			/* do not allow RHSC interrupts > 1 per second */
			usb2_callout_reset(&sc->sc_tmo_rhsc, hz,
			    (void *)&ohci_rhsc_enable, sc);
		}
	}
	status &= ~(OHCI_RHSC | OHCI_WDH | OHCI_SO);
	if (status != 0) {
		/* Block unprocessed interrupts. XXX */
		OWRITE4(sc, OHCI_INTERRUPT_DISABLE, status);
		sc->sc_eintrs &= ~status;
		printf("%s: blocking intrs 0x%x\n",
		    __FUNCTION__, status);
	}
	/* poll all the USB transfers */
	ohci_interrupt_poll(sc);

done:
	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

/*
 * called when a request does not complete
 */
static void
ohci_timeout(void *arg)
{
	struct usb2_xfer *xfer = arg;
	ohci_softc_t *sc = xfer->usb2_sc;

	DPRINTF("xfer=%p\n", xfer);

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	ohci_device_done(xfer, USB_ERR_TIMEOUT);

	mtx_unlock(&sc->sc_bus.mtx);

	return;
}

static void
ohci_do_poll(struct usb2_bus *bus)
{
	struct ohci_softc *sc = OHCI_BUS2SC(bus);

	mtx_lock(&sc->sc_bus.mtx);
	ohci_interrupt_poll(sc);
	ohci_root_ctrl_poll(sc);
	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

static void
ohci_setup_standard_chain_sub(struct ohci_std_temp *temp)
{
	struct usb2_page_search buf_res;
	ohci_td_t *td;
	ohci_td_t *td_next;
	ohci_td_t *td_alt_next;
	uint32_t buf_offset;
	uint32_t average;
	uint32_t len_old;
	uint8_t shortpkt_old;
	uint8_t precompute;

	td_alt_next = NULL;
	buf_offset = 0;
	shortpkt_old = temp->shortpkt;
	len_old = temp->len;
	precompute = 1;

	/* software is used to detect short incoming transfers */

	if ((temp->td_flags & htole32(OHCI_TD_DP_MASK)) == htole32(OHCI_TD_IN)) {
		temp->td_flags |= htole32(OHCI_TD_R);
	} else {
		temp->td_flags &= ~htole32(OHCI_TD_R);
	}

restart:

	td = temp->td;
	td_next = temp->td_next;

	while (1) {

		if (temp->len == 0) {

			if (temp->shortpkt) {
				break;
			}
			/* send a Zero Length Packet, ZLP, last */

			temp->shortpkt = 1;
			average = 0;

		} else {

			average = temp->average;

			if (temp->len < average) {
				if (temp->len % temp->max_frame_size) {
					temp->shortpkt = 1;
				}
				average = temp->len;
			}
		}

		if (td_next == NULL) {
			panic("%s: out of OHCI transfer descriptors!", __FUNCTION__);
		}
		/* get next TD */

		td = td_next;
		td_next = td->obj_next;

		/* check if we are pre-computing */

		if (precompute) {

			/* update remaining length */

			temp->len -= average;

			continue;
		}
		/* fill out current TD */
		td->td_flags = temp->td_flags;

		/* the next TD uses TOGGLE_CARRY */
		temp->td_flags &= ~htole32(OHCI_TD_TOGGLE_MASK);

		if (average == 0) {

			td->td_cbp = 0;
			td->td_be = ~0;
			td->len = 0;

		} else {

			usb2_get_page(temp->pc, buf_offset, &buf_res);
			td->td_cbp = htole32(buf_res.physaddr);
			buf_offset += (average - 1);

			usb2_get_page(temp->pc, buf_offset, &buf_res);
			td->td_be = htole32(buf_res.physaddr);
			buf_offset++;

			td->len = average;

			/* update remaining length */

			temp->len -= average;
		}

		if ((td_next == td_alt_next) && temp->setup_alt_next) {
			/* we need to receive these frames one by one ! */
			td->td_flags &= htole32(~OHCI_TD_INTR_MASK);
			td->td_flags |= htole32(OHCI_TD_SET_DI(1));
			td->td_next = htole32(OHCI_TD_NEXT_END);
		} else {
			if (td_next) {
				/* link the current TD with the next one */
				td->td_next = td_next->td_self;
			}
		}

		td->alt_next = td_alt_next;

		usb2_pc_cpu_flush(td->page_cache);
	}

	if (precompute) {
		precompute = 0;

		/* setup alt next pointer, if any */
		if (temp->short_frames_ok) {
			if (temp->setup_alt_next) {
				td_alt_next = td_next;
			}
		} else {
			/* we use this field internally */
			td_alt_next = td_next;
		}

		/* restore */
		temp->shortpkt = shortpkt_old;
		temp->len = len_old;
		goto restart;
	}
	temp->td = td;
	temp->td_next = td_next;

	return;
}

static void
ohci_setup_standard_chain(struct usb2_xfer *xfer, ohci_ed_t **ed_last)
{
	struct ohci_std_temp temp;
	struct usb2_pipe_methods *methods;
	ohci_ed_t *ed;
	ohci_td_t *td;
	uint32_t ed_flags;
	uint32_t x;

	DPRINTFN(9, "addr=%d endpt=%d sumlen=%d speed=%d\n",
	    xfer->address, UE_GET_ADDR(xfer->endpoint),
	    xfer->sumlen, usb2_get_speed(xfer->udev));

	temp.average = xfer->max_usb2_frame_size;
	temp.max_frame_size = xfer->max_frame_size;

	/* toggle the DMA set we are using */
	xfer->flags_int.curr_dma_set ^= 1;

	/* get next DMA set */
	td = xfer->td_start[xfer->flags_int.curr_dma_set];

	xfer->td_transfer_first = td;
	xfer->td_transfer_cache = td;

	temp.td = NULL;
	temp.td_next = td;
	temp.setup_alt_next = xfer->flags_int.short_frames_ok;
	temp.short_frames_ok = xfer->flags_int.short_frames_ok;

	methods = xfer->pipe->methods;

	/* check if we should prepend a setup message */

	if (xfer->flags_int.control_xfr) {
		if (xfer->flags_int.control_hdr) {

			temp.td_flags = htole32(OHCI_TD_SETUP | OHCI_TD_NOCC |
			    OHCI_TD_TOGGLE_0 | OHCI_TD_NOINTR);

			temp.len = xfer->frlengths[0];
			temp.pc = xfer->frbuffers + 0;
			temp.shortpkt = temp.len ? 1 : 0;

			ohci_setup_standard_chain_sub(&temp);

			/*
			 * XXX assume that the setup message is
			 * contained within one USB packet:
			 */
			xfer->pipe->toggle_next = 1;
		}
		x = 1;
	} else {
		x = 0;
	}
	temp.td_flags = htole32(OHCI_TD_NOCC | OHCI_TD_NOINTR);

	/* set data toggle */

	if (xfer->pipe->toggle_next) {
		temp.td_flags |= htole32(OHCI_TD_TOGGLE_1);
	} else {
		temp.td_flags |= htole32(OHCI_TD_TOGGLE_0);
	}

	/* set endpoint direction */

	if (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN) {
		temp.td_flags |= htole32(OHCI_TD_IN);
	} else {
		temp.td_flags |= htole32(OHCI_TD_OUT);
	}

	while (x != xfer->nframes) {

		/* DATA0 / DATA1 message */

		temp.len = xfer->frlengths[x];
		temp.pc = xfer->frbuffers + x;

		x++;

		if (x == xfer->nframes) {
			temp.setup_alt_next = 0;
		}
		if (temp.len == 0) {

			/* make sure that we send an USB packet */

			temp.shortpkt = 0;

		} else {

			/* regular data transfer */

			temp.shortpkt = (xfer->flags.force_short_xfer) ? 0 : 1;
		}

		ohci_setup_standard_chain_sub(&temp);
	}

	/* check if we should append a status stage */

	if (xfer->flags_int.control_xfr &&
	    !xfer->flags_int.control_act) {

		/*
		 * Send a DATA1 message and invert the current endpoint
		 * direction.
		 */

		/* set endpoint direction and data toggle */

		if (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN) {
			temp.td_flags = htole32(OHCI_TD_OUT |
			    OHCI_TD_NOCC | OHCI_TD_TOGGLE_1 | OHCI_TD_SET_DI(1));
		} else {
			temp.td_flags = htole32(OHCI_TD_IN |
			    OHCI_TD_NOCC | OHCI_TD_TOGGLE_1 | OHCI_TD_SET_DI(1));
		}

		temp.len = 0;
		temp.pc = NULL;
		temp.shortpkt = 0;

		ohci_setup_standard_chain_sub(&temp);
	}
	td = temp.td;

	td->td_next = htole32(OHCI_TD_NEXT_END);
	td->td_flags &= ~htole32(OHCI_TD_INTR_MASK);
	td->td_flags |= htole32(OHCI_TD_SET_DI(1));

	usb2_pc_cpu_flush(td->page_cache);

	/* must have at least one frame! */

	xfer->td_transfer_last = td;

#ifdef USB_DEBUG
	if (ohcidebug > 8) {
		DPRINTF("nexttog=%d; data before transfer:\n",
		    xfer->pipe->toggle_next);
		ohci_dump_tds(xfer->td_transfer_first);
	}
#endif

	ed = xfer->qh_start[xfer->flags_int.curr_dma_set];

	ed_flags = (OHCI_ED_SET_FA(xfer->address) |
	    OHCI_ED_SET_EN(UE_GET_ADDR(xfer->endpoint)) |
	    OHCI_ED_SET_MAXP(xfer->max_frame_size));

	ed_flags |= (OHCI_ED_FORMAT_GEN | OHCI_ED_DIR_TD);

	if (xfer->udev->speed == USB_SPEED_LOW) {
		ed_flags |= OHCI_ED_SPEED;
	}
	ed->ed_flags = htole32(ed_flags);

	usb2_pc_cpu_flush(ed->page_cache);

	td = xfer->td_transfer_first;

	OHCI_APPEND_QH(ed, td->td_self, *ed_last);

	if (methods == &ohci_device_bulk_methods) {
		ohci_softc_t *sc = xfer->usb2_sc;

		OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_BLF);
	}
	if (methods == &ohci_device_ctrl_methods) {
		ohci_softc_t *sc = xfer->usb2_sc;

		OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_CLF);
	}
	return;
}

static void
ohci_root_intr_done(struct usb2_xfer *xfer,
    struct usb2_sw_transfer *std)
{
	ohci_softc_t *sc = xfer->usb2_sc;
	uint32_t hstatus;
	uint16_t i;
	uint16_t m;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USB_SW_TR_PRE_DATA) {
		if (std->state == USB_SW_TR_PRE_CALLBACK) {
			/* transfer transferred */
			ohci_device_done(xfer, std->err);
		}
		goto done;
	}
	/* setup buffer */
	std->ptr = sc->sc_hub_idata;
	std->len = sizeof(sc->sc_hub_idata);

	/* clear any old interrupt data */
	bzero(sc->sc_hub_idata, sizeof(sc->sc_hub_idata));

	hstatus = OREAD4(sc, OHCI_RH_STATUS);
	DPRINTF("sc=%p xfer=%p hstatus=0x%08x\n",
	    sc, xfer, hstatus);

	/* set bits */
	m = (sc->sc_noport + 1);
	if (m > (8 * sizeof(sc->sc_hub_idata))) {
		m = (8 * sizeof(sc->sc_hub_idata));
	}
	for (i = 1; i < m; i++) {
		/* pick out CHANGE bits from the status register */
		if (OREAD4(sc, OHCI_RH_PORT_STATUS(i)) >> 16) {
			sc->sc_hub_idata[i / 8] |= 1 << (i % 8);
			DPRINTF("port %d changed\n", i);
		}
	}
done:
	return;
}

/* NOTE: "done" can be run two times in a row,
 * from close and from interrupt
 */
static void
ohci_device_done(struct usb2_xfer *xfer, usb2_error_t error)
{
	struct usb2_pipe_methods *methods = xfer->pipe->methods;
	ohci_softc_t *sc = xfer->usb2_sc;
	ohci_ed_t *ed;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);


	DPRINTFN(2, "xfer=%p, pipe=%p, error=%d\n",
	    xfer, xfer->pipe, error);

	ed = xfer->qh_start[xfer->flags_int.curr_dma_set];
	if (ed) {
		usb2_pc_cpu_invalidate(ed->page_cache);
	}
	if (methods == &ohci_device_bulk_methods) {
		OHCI_REMOVE_QH(ed, sc->sc_bulk_p_last);
	}
	if (methods == &ohci_device_ctrl_methods) {
		OHCI_REMOVE_QH(ed, sc->sc_ctrl_p_last);
	}
	if (methods == &ohci_device_intr_methods) {
		OHCI_REMOVE_QH(ed, sc->sc_intr_p_last[xfer->qh_pos]);
	}
	if (methods == &ohci_device_isoc_methods) {
		OHCI_REMOVE_QH(ed, sc->sc_isoc_p_last);
	}
	xfer->td_transfer_first = NULL;
	xfer->td_transfer_last = NULL;

	/* dequeue transfer and start next transfer */
	usb2_transfer_done(xfer, error);
	return;
}

/*------------------------------------------------------------------------*
 * ohci bulk support
 *------------------------------------------------------------------------*/
static void
ohci_device_bulk_open(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_device_bulk_close(struct usb2_xfer *xfer)
{
	ohci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
ohci_device_bulk_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_device_bulk_start(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	/* setup TD's and QH */
	ohci_setup_standard_chain(xfer, &sc->sc_bulk_p_last);

	/* put transfer on interrupt queue */
	ohci_transfer_intr_enqueue(xfer);
	return;
}

struct usb2_pipe_methods ohci_device_bulk_methods =
{
	.open = ohci_device_bulk_open,
	.close = ohci_device_bulk_close,
	.enter = ohci_device_bulk_enter,
	.start = ohci_device_bulk_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * ohci control support
 *------------------------------------------------------------------------*/
static void
ohci_device_ctrl_open(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_device_ctrl_close(struct usb2_xfer *xfer)
{
	ohci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
ohci_device_ctrl_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_device_ctrl_start(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	/* setup TD's and QH */
	ohci_setup_standard_chain(xfer, &sc->sc_ctrl_p_last);

	/* put transfer on interrupt queue */
	ohci_transfer_intr_enqueue(xfer);
	return;
}

struct usb2_pipe_methods ohci_device_ctrl_methods =
{
	.open = ohci_device_ctrl_open,
	.close = ohci_device_ctrl_close,
	.enter = ohci_device_ctrl_enter,
	.start = ohci_device_ctrl_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * ohci interrupt support
 *------------------------------------------------------------------------*/
static void
ohci_device_intr_open(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;
	uint16_t best;
	uint16_t bit;
	uint16_t x;

	best = 0;
	bit = OHCI_NO_EDS / 2;
	while (bit) {
		if (xfer->interval >= bit) {
			x = bit;
			best = bit;
			while (x & bit) {
				if (sc->sc_intr_stat[x] <
				    sc->sc_intr_stat[best]) {
					best = x;
				}
				x++;
			}
			break;
		}
		bit >>= 1;
	}

	sc->sc_intr_stat[best]++;
	xfer->qh_pos = best;

	DPRINTFN(3, "best=%d interval=%d\n",
	    best, xfer->interval);
	return;
}

static void
ohci_device_intr_close(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	sc->sc_intr_stat[xfer->qh_pos]--;

	ohci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
ohci_device_intr_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_device_intr_start(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	/* setup TD's and QH */
	ohci_setup_standard_chain(xfer, &sc->sc_intr_p_last[xfer->qh_pos]);

	/* put transfer on interrupt queue */
	ohci_transfer_intr_enqueue(xfer);
	return;
}

struct usb2_pipe_methods ohci_device_intr_methods =
{
	.open = ohci_device_intr_open,
	.close = ohci_device_intr_close,
	.enter = ohci_device_intr_enter,
	.start = ohci_device_intr_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * ohci isochronous support
 *------------------------------------------------------------------------*/
static void
ohci_device_isoc_open(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_device_isoc_close(struct usb2_xfer *xfer)
{
	/**/
	ohci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
ohci_device_isoc_enter(struct usb2_xfer *xfer)
{
	struct usb2_page_search buf_res;
	ohci_softc_t *sc = xfer->usb2_sc;
	struct ohci_hcca *hcca;
	uint32_t buf_offset;
	uint32_t nframes;
	uint32_t ed_flags;
	uint32_t *plen;
	uint16_t itd_offset[OHCI_ITD_NOFFSET];
	uint16_t length;
	uint8_t ncur;
	ohci_itd_t *td;
	ohci_itd_t *td_last = NULL;
	ohci_ed_t *ed;

	hcca = ohci_get_hcca(sc);

	nframes = le32toh(hcca->hcca_frame_number);

	DPRINTFN(6, "xfer=%p isoc_next=%u nframes=%u hcca_fn=%u\n",
	    xfer, xfer->pipe->isoc_next, xfer->nframes, nframes);

	if ((xfer->pipe->is_synced == 0) ||
	    (((nframes - xfer->pipe->isoc_next) & 0xFFFF) < xfer->nframes) ||
	    (((xfer->pipe->isoc_next - nframes) & 0xFFFF) >= 128)) {
		/*
		 * If there is data underflow or the pipe queue is empty we
		 * schedule the transfer a few frames ahead of the current
		 * frame position. Else two isochronous transfers might
		 * overlap.
		 */
		xfer->pipe->isoc_next = (nframes + 3) & 0xFFFF;
		xfer->pipe->is_synced = 1;
		DPRINTFN(3, "start next=%d\n", xfer->pipe->isoc_next);
	}
	/*
	 * compute how many milliseconds the insertion is ahead of the
	 * current frame position:
	 */
	buf_offset = ((xfer->pipe->isoc_next - nframes) & 0xFFFF);

	/*
	 * pre-compute when the isochronous transfer will be finished:
	 */
	xfer->isoc_time_complete =
	    (usb2_isoc_time_expand(&sc->sc_bus, nframes) + buf_offset +
	    xfer->nframes);

	/* get the real number of frames */

	nframes = xfer->nframes;

	buf_offset = 0;

	plen = xfer->frlengths;

	/* toggle the DMA set we are using */
	xfer->flags_int.curr_dma_set ^= 1;

	/* get next DMA set */
	td = xfer->td_start[xfer->flags_int.curr_dma_set];

	xfer->td_transfer_first = td;

	ncur = 0;
	length = 0;

	while (nframes--) {
		if (td == NULL) {
			panic("%s:%d: out of TD's\n",
			    __FUNCTION__, __LINE__);
		}
		itd_offset[ncur] = length;
		buf_offset += *plen;
		length += *plen;
		plen++;
		ncur++;

		if (			/* check if the ITD is full */
		    (ncur == OHCI_ITD_NOFFSET) ||
		/* check if we have put more than 4K into the ITD */
		    (length & 0xF000) ||
		/* check if it is the last frame */
		    (nframes == 0)) {

			/* fill current ITD */
			td->itd_flags = htole32(
			    OHCI_ITD_NOCC |
			    OHCI_ITD_SET_SF(xfer->pipe->isoc_next) |
			    OHCI_ITD_NOINTR |
			    OHCI_ITD_SET_FC(ncur));

			td->frames = ncur;
			xfer->pipe->isoc_next += ncur;

			if (length == 0) {
				/* all zero */
				td->itd_bp0 = 0;
				td->itd_be = ~0;

				while (ncur--) {
					td->itd_offset[ncur] =
					    htole16(OHCI_ITD_MK_OFFS(0));
				}
			} else {
				usb2_get_page(xfer->frbuffers, buf_offset - length, &buf_res);
				length = OHCI_PAGE_MASK(buf_res.physaddr);
				buf_res.physaddr =
				    OHCI_PAGE(buf_res.physaddr);
				td->itd_bp0 = htole32(buf_res.physaddr);
				usb2_get_page(xfer->frbuffers, buf_offset - 1, &buf_res);
				td->itd_be = htole32(buf_res.physaddr);

				while (ncur--) {
					itd_offset[ncur] += length;
					itd_offset[ncur] =
					    OHCI_ITD_MK_OFFS(itd_offset[ncur]);
					td->itd_offset[ncur] =
					    htole16(itd_offset[ncur]);
				}
			}
			ncur = 0;
			length = 0;
			td_last = td;
			td = td->obj_next;

			if (td) {
				/* link the last TD with the next one */
				td_last->itd_next = td->itd_self;
			}
			usb2_pc_cpu_flush(td_last->page_cache);
		}
	}

	/* update the last TD */
	td_last->itd_flags &= ~htole32(OHCI_ITD_NOINTR);
	td_last->itd_flags |= htole32(OHCI_ITD_SET_DI(0));
	td_last->itd_next = 0;

	usb2_pc_cpu_flush(td_last->page_cache);

	xfer->td_transfer_last = td_last;

#ifdef USB_DEBUG
	if (ohcidebug > 8) {
		DPRINTF("data before transfer:\n");
		ohci_dump_itds(xfer->td_transfer_first);
	}
#endif
	ed = xfer->qh_start[xfer->flags_int.curr_dma_set];

	if (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN)
		ed_flags = (OHCI_ED_DIR_IN | OHCI_ED_FORMAT_ISO);
	else
		ed_flags = (OHCI_ED_DIR_OUT | OHCI_ED_FORMAT_ISO);

	ed_flags |= (OHCI_ED_SET_FA(xfer->address) |
	    OHCI_ED_SET_EN(UE_GET_ADDR(xfer->endpoint)) |
	    OHCI_ED_SET_MAXP(xfer->max_frame_size));

	if (xfer->udev->speed == USB_SPEED_LOW) {
		ed_flags |= OHCI_ED_SPEED;
	}
	ed->ed_flags = htole32(ed_flags);

	usb2_pc_cpu_flush(ed->page_cache);

	td = xfer->td_transfer_first;

	OHCI_APPEND_QH(ed, td->itd_self, sc->sc_isoc_p_last);
	return;
}

static void
ohci_device_isoc_start(struct usb2_xfer *xfer)
{
	/* put transfer on interrupt queue */
	ohci_transfer_intr_enqueue(xfer);
	return;
}

struct usb2_pipe_methods ohci_device_isoc_methods =
{
	.open = ohci_device_isoc_open,
	.close = ohci_device_isoc_close,
	.enter = ohci_device_isoc_enter,
	.start = ohci_device_isoc_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

/*------------------------------------------------------------------------*
 * ohci root control support
 *------------------------------------------------------------------------*
 * simulate a hardware hub by handling
 * all the necessary requests
 *------------------------------------------------------------------------*/

static void
ohci_root_ctrl_open(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_root_ctrl_close(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	if (sc->sc_root_ctrl.xfer == xfer) {
		sc->sc_root_ctrl.xfer = NULL;
	}
	ohci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

/* data structures and routines
 * to emulate the root hub:
 */
static const
struct usb2_device_descriptor ohci_devd =
{
	sizeof(struct usb2_device_descriptor),
	UDESC_DEVICE,			/* type */
	{0x00, 0x01},			/* USB version */
	UDCLASS_HUB,			/* class */
	UDSUBCLASS_HUB,			/* subclass */
	UDPROTO_FSHUB,			/* protocol */
	64,				/* max packet */
	{0}, {0}, {0x00, 0x01},		/* device id */
	1, 2, 0,			/* string indicies */
	1				/* # of configurations */
};

static const
struct ohci_config_desc ohci_confd =
{
	.confd = {
		.bLength = sizeof(struct usb2_config_descriptor),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength[0] = sizeof(ohci_confd),
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_SELF_POWERED,
		.bMaxPower = 0,		/* max power */
	},

	.ifcd = {
		.bLength = sizeof(struct usb2_interface_descriptor),
		.bDescriptorType = UDESC_INTERFACE,
		.bNumEndpoints = 1,
		.bInterfaceClass = UICLASS_HUB,
		.bInterfaceSubClass = UISUBCLASS_HUB,
		.bInterfaceProtocol = UIPROTO_FSHUB,
	},

	.endpd = {
		.bLength = sizeof(struct usb2_endpoint_descriptor),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN | OHCI_INTR_ENDPT,
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 32,/* max packet (255 ports) */
		.bInterval = 255,
	},
};

static const
struct usb2_hub_descriptor ohci_hubd =
{
	0,				/* dynamic length */
	UDESC_HUB,
	0,
	{0, 0},
	0,
	0,
	{0},
};

static void
ohci_root_ctrl_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_root_ctrl_start(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	sc->sc_root_ctrl.xfer = xfer;

	usb2_config_td_queue_command
	    (&sc->sc_config_td, NULL, &ohci_root_ctrl_task, 0, 0);

	return;
}

static void
ohci_root_ctrl_task(struct ohci_softc *sc,
    struct ohci_config_copy *cc, uint16_t refcount)
{
	ohci_root_ctrl_poll(sc);
	return;
}

static void
ohci_root_ctrl_done(struct usb2_xfer *xfer,
    struct usb2_sw_transfer *std)
{
	ohci_softc_t *sc = xfer->usb2_sc;
	char *ptr;
	uint32_t port;
	uint32_t v;
	uint16_t value;
	uint16_t index;
	uint8_t l;
	uint8_t use_polling;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (std->state != USB_SW_TR_SETUP) {
		if (std->state == USB_SW_TR_PRE_CALLBACK) {
			/* transfer transferred */
			ohci_device_done(xfer, std->err);
		}
		goto done;
	}
	/* buffer reset */
	std->ptr = sc->sc_hub_desc.temp;
	std->len = 0;

	value = UGETW(std->req.wValue);
	index = UGETW(std->req.wIndex);

	use_polling = mtx_owned(xfer->priv_mtx) ? 1 : 0;

	DPRINTFN(3, "type=0x%02x request=0x%02x wLen=0x%04x "
	    "wValue=0x%04x wIndex=0x%04x\n",
	    std->req.bmRequestType, std->req.bRequest,
	    UGETW(std->req.wLength), value, index);

#define	C(x,y) ((x) | ((y) << 8))
	switch (C(std->req.bRequest, std->req.bmRequestType)) {
	case C(UR_CLEAR_FEATURE, UT_WRITE_DEVICE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
		/*
		 * DEVICE_REMOTE_WAKEUP and ENDPOINT_HALT are no-ops
		 * for the integrated root hub.
		 */
		break;
	case C(UR_GET_CONFIG, UT_READ_DEVICE):
		std->len = 1;
		sc->sc_hub_desc.temp[0] = sc->sc_conf;
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		switch (value >> 8) {
		case UDESC_DEVICE:
			if ((value & 0xff) != 0) {
				std->err = USB_ERR_IOERROR;
				goto done;
			}
			std->len = sizeof(ohci_devd);
			sc->sc_hub_desc.devd = ohci_devd;
			break;

		case UDESC_CONFIG:
			if ((value & 0xff) != 0) {
				std->err = USB_ERR_IOERROR;
				goto done;
			}
			std->len = sizeof(ohci_confd);
			std->ptr = USB_ADD_BYTES(&ohci_confd, 0);
			break;

		case UDESC_STRING:
			switch (value & 0xff) {
			case 0:	/* Language table */
				ptr = "\001";
				break;

			case 1:	/* Vendor */
				ptr = sc->sc_vendor;
				break;

			case 2:	/* Product */
				ptr = "OHCI root HUB";
				break;

			default:
				ptr = "";
				break;
			}

			std->len = usb2_make_str_desc
			    (sc->sc_hub_desc.temp,
			    sizeof(sc->sc_hub_desc.temp),
			    ptr);
			break;

		default:
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		break;
	case C(UR_GET_INTERFACE, UT_READ_INTERFACE):
		std->len = 1;
		sc->sc_hub_desc.temp[0] = 0;
		break;
	case C(UR_GET_STATUS, UT_READ_DEVICE):
		std->len = 2;
		USETW(sc->sc_hub_desc.stat.wStatus, UDS_SELF_POWERED);
		break;
	case C(UR_GET_STATUS, UT_READ_INTERFACE):
	case C(UR_GET_STATUS, UT_READ_ENDPOINT):
		std->len = 2;
		USETW(sc->sc_hub_desc.stat.wStatus, 0);
		break;
	case C(UR_SET_ADDRESS, UT_WRITE_DEVICE):
		if (value >= USB_MAX_DEVICES) {
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		sc->sc_addr = value;
		break;
	case C(UR_SET_CONFIG, UT_WRITE_DEVICE):
		if ((value != 0) && (value != 1)) {
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		sc->sc_conf = value;
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_DEVICE):
	case C(UR_SET_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_SET_FEATURE, UT_WRITE_ENDPOINT):
		std->err = USB_ERR_IOERROR;
		goto done;
	case C(UR_SET_INTERFACE, UT_WRITE_INTERFACE):
		break;
	case C(UR_SYNCH_FRAME, UT_WRITE_ENDPOINT):
		break;
		/* Hub requests */
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_OTHER):
		DPRINTFN(9, "UR_CLEAR_PORT_FEATURE "
		    "port=%d feature=%d\n",
		    index, value);
		if ((index < 1) ||
		    (index > sc->sc_noport)) {
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		port = OHCI_RH_PORT_STATUS(index);
		switch (value) {
		case UHF_PORT_ENABLE:
			OWRITE4(sc, port, UPS_CURRENT_CONNECT_STATUS);
			break;
		case UHF_PORT_SUSPEND:
			OWRITE4(sc, port, UPS_OVERCURRENT_INDICATOR);
			break;
		case UHF_PORT_POWER:
			/* Yes, writing to the LOW_SPEED bit clears power. */
			OWRITE4(sc, port, UPS_LOW_SPEED);
			break;
		case UHF_C_PORT_CONNECTION:
			OWRITE4(sc, port, UPS_C_CONNECT_STATUS << 16);
			break;
		case UHF_C_PORT_ENABLE:
			OWRITE4(sc, port, UPS_C_PORT_ENABLED << 16);
			break;
		case UHF_C_PORT_SUSPEND:
			OWRITE4(sc, port, UPS_C_SUSPEND << 16);
			break;
		case UHF_C_PORT_OVER_CURRENT:
			OWRITE4(sc, port, UPS_C_OVERCURRENT_INDICATOR << 16);
			break;
		case UHF_C_PORT_RESET:
			OWRITE4(sc, port, UPS_C_PORT_RESET << 16);
			break;
		default:
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		switch (value) {
		case UHF_C_PORT_CONNECTION:
		case UHF_C_PORT_ENABLE:
		case UHF_C_PORT_SUSPEND:
		case UHF_C_PORT_OVER_CURRENT:
		case UHF_C_PORT_RESET:
			/* enable RHSC interrupt if condition is cleared. */
			if ((OREAD4(sc, port) >> 16) == 0) {
				ohci_rhsc_enable(sc);
				mtx_lock(&sc->sc_bus.mtx);
			}
			break;
		default:
			break;
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_CLASS_DEVICE):
		if ((value & 0xff) != 0) {
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		v = OREAD4(sc, OHCI_RH_DESCRIPTOR_A);

		sc->sc_hub_desc.hubd = ohci_hubd;
		sc->sc_hub_desc.hubd.bNbrPorts = sc->sc_noport;
		USETW(sc->sc_hub_desc.hubd.wHubCharacteristics,
		    (v & OHCI_NPS ? UHD_PWR_NO_SWITCH :
		    v & OHCI_PSM ? UHD_PWR_GANGED : UHD_PWR_INDIVIDUAL)
		/* XXX overcurrent */
		    );
		sc->sc_hub_desc.hubd.bPwrOn2PwrGood = OHCI_GET_POTPGT(v);
		v = OREAD4(sc, OHCI_RH_DESCRIPTOR_B);

		for (l = 0; l < sc->sc_noport; l++) {
			if (v & 1) {
				sc->sc_hub_desc.hubd.DeviceRemovable[l / 8] |= (1 << (l % 8));
			}
			v >>= 1;
		}
		sc->sc_hub_desc.hubd.bDescLength =
		    8 + ((sc->sc_noport + 7) / 8);
		std->len = sc->sc_hub_desc.hubd.bDescLength;
		break;

	case C(UR_GET_STATUS, UT_READ_CLASS_DEVICE):
		std->len = 16;
		bzero(sc->sc_hub_desc.temp, 16);
		break;
	case C(UR_GET_STATUS, UT_READ_CLASS_OTHER):
		DPRINTFN(9, "get port status i=%d\n",
		    index);
		if ((index < 1) ||
		    (index > sc->sc_noport)) {
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		v = OREAD4(sc, OHCI_RH_PORT_STATUS(index));
		DPRINTFN(9, "port status=0x%04x\n", v);
		USETW(sc->sc_hub_desc.ps.wPortStatus, v);
		USETW(sc->sc_hub_desc.ps.wPortChange, v >> 16);
		std->len = sizeof(sc->sc_hub_desc.ps);
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_CLASS_DEVICE):
		std->err = USB_ERR_IOERROR;
		goto done;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_OTHER):
		if ((index < 1) ||
		    (index > sc->sc_noport)) {
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		port = OHCI_RH_PORT_STATUS(index);
		switch (value) {
		case UHF_PORT_ENABLE:
			OWRITE4(sc, port, UPS_PORT_ENABLED);
			break;
		case UHF_PORT_SUSPEND:
			OWRITE4(sc, port, UPS_SUSPEND);
			break;
		case UHF_PORT_RESET:
			DPRINTFN(6, "reset port %d\n", index);
			OWRITE4(sc, port, UPS_RESET);
			for (v = 0;; v++) {
				if (v < 12) {
					if (use_polling) {
						/* polling */
						DELAY(USB_PORT_ROOT_RESET_DELAY * 1000);
					} else {
						l = usb2_config_td_sleep
						    (&sc->sc_config_td,
						    (hz * USB_PORT_ROOT_RESET_DELAY) / 1000);
					}

					if ((OREAD4(sc, port) & UPS_RESET) == 0) {
						break;
					}
				} else {
					std->err = USB_ERR_TIMEOUT;
					goto done;
				}
			}
			DPRINTFN(9, "ohci port %d reset, status = 0x%04x\n",
			    index, OREAD4(sc, port));
			break;
		case UHF_PORT_POWER:
			DPRINTFN(3, "set port power %d\n", index);
			OWRITE4(sc, port, UPS_PORT_POWER);
			break;
		default:
			std->err = USB_ERR_IOERROR;
			goto done;
		}
		break;
	default:
		std->err = USB_ERR_IOERROR;
		goto done;
	}
done:
	return;
}

static void
ohci_root_ctrl_poll(struct ohci_softc *sc)
{
	usb2_sw_transfer(&sc->sc_root_ctrl,
	    &ohci_root_ctrl_done);
	return;
}

struct usb2_pipe_methods ohci_root_ctrl_methods =
{
	.open = ohci_root_ctrl_open,
	.close = ohci_root_ctrl_close,
	.enter = ohci_root_ctrl_enter,
	.start = ohci_root_ctrl_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 0,
};

/*------------------------------------------------------------------------*
 * ohci root interrupt support
 *------------------------------------------------------------------------*/
static void
ohci_root_intr_open(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_root_intr_close(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	if (sc->sc_root_intr.xfer == xfer) {
		sc->sc_root_intr.xfer = NULL;
	}
	ohci_device_done(xfer, USB_ERR_CANCELLED);
	return;
}

static void
ohci_root_intr_enter(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_root_intr_start(struct usb2_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb2_sc;

	sc->sc_root_intr.xfer = xfer;
	return;
}

struct usb2_pipe_methods ohci_root_intr_methods =
{
	.open = ohci_root_intr_open,
	.close = ohci_root_intr_close,
	.enter = ohci_root_intr_enter,
	.start = ohci_root_intr_start,
	.enter_is_cancelable = 1,
	.start_is_cancelable = 1,
};

static void
ohci_xfer_setup(struct usb2_setup_params *parm)
{
	struct usb2_page_search page_info;
	struct usb2_page_cache *pc;
	ohci_softc_t *sc;
	struct usb2_xfer *xfer;
	void *last_obj;
	uint32_t ntd;
	uint32_t nitd;
	uint32_t nqh;
	uint32_t n;

	sc = OHCI_BUS2SC(parm->udev->bus);
	xfer = parm->curr_xfer;

	/*
	 * setup xfer
	 */
	xfer->usb2_sc = sc;

	parm->hc_max_packet_size = 0x500;
	parm->hc_max_packet_count = 1;
	parm->hc_max_frame_size = OHCI_PAGE_SIZE;

	/*
	 * calculate ntd and nqh
	 */
	if (parm->methods == &ohci_device_ctrl_methods) {
		xfer->flags_int.bdma_enable = 1;

		usb2_transfer_setup_sub(parm);

		nitd = 0;
		ntd = ((2 * xfer->nframes) + 1	/* STATUS */
		    + (xfer->max_data_length / xfer->max_usb2_frame_size));
		nqh = 1;

	} else if (parm->methods == &ohci_device_bulk_methods) {
		xfer->flags_int.bdma_enable = 1;

		usb2_transfer_setup_sub(parm);

		nitd = 0;
		ntd = ((2 * xfer->nframes)
		    + (xfer->max_data_length / xfer->max_usb2_frame_size));
		nqh = 1;

	} else if (parm->methods == &ohci_device_intr_methods) {
		xfer->flags_int.bdma_enable = 1;

		usb2_transfer_setup_sub(parm);

		nitd = 0;
		ntd = ((2 * xfer->nframes)
		    + (xfer->max_data_length / xfer->max_usb2_frame_size));
		nqh = 1;

	} else if (parm->methods == &ohci_device_isoc_methods) {
		xfer->flags_int.bdma_enable = 1;

		usb2_transfer_setup_sub(parm);

		nitd = ((xfer->max_data_length / OHCI_PAGE_SIZE) +
		    ((xfer->nframes + OHCI_ITD_NOFFSET - 1) / OHCI_ITD_NOFFSET) +
		    1 /* EXTRA */ );
		ntd = 0;
		nqh = 1;

	} else {

		usb2_transfer_setup_sub(parm);

		nitd = 0;
		ntd = 0;
		nqh = 0;
	}

alloc_dma_set:

	if (parm->err) {
		return;
	}
	last_obj = NULL;

	for (n = 0; n != ntd; n++) {

		ohci_td_t *td;

		if (usb2_transfer_setup_sub_malloc(
		    parm, &page_info, &pc, sizeof(*td),
		    OHCI_TD_ALIGN)) {
			parm->err = USB_ERR_NOMEM;
			break;
		}
		if (parm->buf) {

			td = page_info.buffer;

			/* init TD */
			td->td_self = htole32(page_info.physaddr);
			td->obj_next = last_obj;
			td->page_cache = pc;

			last_obj = td;

			usb2_pc_cpu_flush(pc);
		}
	}

	for (n = 0; n != nitd; n++) {

		ohci_itd_t *itd;

		if (usb2_transfer_setup_sub_malloc(
		    parm, &page_info, &pc, sizeof(*itd),
		    OHCI_ITD_ALIGN)) {
			parm->err = USB_ERR_NOMEM;
			break;
		}
		if (parm->buf) {

			itd = page_info.buffer;

			/* init TD */
			itd->itd_self = htole32(page_info.physaddr);
			itd->obj_next = last_obj;
			itd->page_cache = pc;

			last_obj = itd;

			usb2_pc_cpu_flush(pc);
		}
	}

	xfer->td_start[xfer->flags_int.curr_dma_set] = last_obj;

	last_obj = NULL;

	for (n = 0; n != nqh; n++) {

		ohci_ed_t *ed;

		if (usb2_transfer_setup_sub_malloc(
		    parm, &page_info, &pc, sizeof(*ed),
		    OHCI_ED_ALIGN)) {
			parm->err = USB_ERR_NOMEM;
			break;
		}
		if (parm->buf) {

			ed = page_info.buffer;

			/* init QH */
			ed->ed_self = htole32(page_info.physaddr);
			ed->obj_next = last_obj;
			ed->page_cache = pc;

			last_obj = ed;

			usb2_pc_cpu_flush(pc);
		}
	}

	xfer->qh_start[xfer->flags_int.curr_dma_set] = last_obj;

	if (!xfer->flags_int.curr_dma_set) {
		xfer->flags_int.curr_dma_set = 1;
		goto alloc_dma_set;
	}
	return;
}

static void
ohci_pipe_init(struct usb2_device *udev, struct usb2_endpoint_descriptor *edesc,
    struct usb2_pipe *pipe)
{
	ohci_softc_t *sc = OHCI_BUS2SC(udev->bus);

	DPRINTFN(2, "pipe=%p, addr=%d, endpt=%d, mode=%d (%d)\n",
	    pipe, udev->address,
	    edesc->bEndpointAddress, udev->flags.usb2_mode,
	    sc->sc_addr);

	if (udev->flags.usb2_mode != USB_MODE_HOST) {
		/* not supported */
		return;
	}
	if (udev->device_index == sc->sc_addr) {
		switch (edesc->bEndpointAddress) {
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &ohci_root_ctrl_methods;
			break;
		case UE_DIR_IN | OHCI_INTR_ENDPT:
			pipe->methods = &ohci_root_intr_methods;
			break;
		default:
			/* do nothing */
			break;
		}
	} else {
		switch (edesc->bmAttributes & UE_XFERTYPE) {
		case UE_CONTROL:
			pipe->methods = &ohci_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &ohci_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			if (udev->speed == USB_SPEED_FULL) {
				pipe->methods = &ohci_device_isoc_methods;
			}
			break;
		case UE_BULK:
			if (udev->speed != USB_SPEED_LOW) {
				pipe->methods = &ohci_device_bulk_methods;
			}
			break;
		default:
			/* do nothing */
			break;
		}
	}
	return;
}

static void
ohci_xfer_unsetup(struct usb2_xfer *xfer)
{
	return;
}

static void
ohci_get_dma_delay(struct usb2_bus *bus, uint32_t *pus)
{
	/*
	 * Wait until hardware has finished any possible use of the
	 * transfer descriptor(s) and QH
	 */
	*pus = (1125);			/* microseconds */
	return;
}

struct usb2_bus_methods ohci_bus_methods =
{
	.pipe_init = ohci_pipe_init,
	.xfer_setup = ohci_xfer_setup,
	.xfer_unsetup = ohci_xfer_unsetup,
	.do_poll = ohci_do_poll,
	.get_dma_delay = ohci_get_dma_delay,
};
