/*	$NetBSD: ohci.c,v 1.138 2003/02/08 03:32:50 ichiro Exp $	*/

/* Also, already ported:
 *	$NetBSD: ohci.c,v 1.140 2003/05/13 04:42:00 gson Exp $
 *	$NetBSD: ohci.c,v 1.141 2003/09/10 20:08:29 mycroft Exp $
 *	$NetBSD: ohci.c,v 1.142 2003/10/11 03:04:26 toshii Exp $
 *	$NetBSD: ohci.c,v 1.143 2003/10/18 04:50:35 simonb Exp $
 *	$NetBSD: ohci.c,v 1.144 2003/11/23 19:18:06 augustss Exp $
 *	$NetBSD: ohci.c,v 1.145 2003/11/23 19:20:25 augustss Exp $
 *	$NetBSD: ohci.c,v 1.146 2003/12/29 08:17:10 toshii Exp $
 *	$NetBSD: ohci.c,v 1.147 2004/06/22 07:20:35 mycroft Exp $
 *	$NetBSD: ohci.c,v 1.148 2004/06/22 18:27:46 mycroft Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ohci.c,v 1.164 2006/09/07 00:06:41 imp Exp $");

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
 * USB Open Host Controller driver.
 *
 * OHCI spec: http://www.compaq.com/productinfo/development/openhci.html
 * USB spec: http://www.usb.org/developers/docs/usbspec.zip
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/queue.h> /* LIST_XXX() */
#include <sys/lock.h>
#include <sys/malloc.h>

#define INCLUDE_PCIXXX_H

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/ohci.h>

#define MS_TO_TICKS(ms) (((ms) * hz) / 1000)
#define OHCI_BUS2SC(bus) ((ohci_softc_t *)(((u_int8_t *)(bus)) - \
   POINTER_TO_UNSIGNED(&(((ohci_softc_t *)0)->sc_bus))))

#ifdef USB_DEBUG
#undef DPRINTF
#undef DPRINTFN
#define DPRINTF(x)	{ if (ohcidebug) { printf("%s: ", __FUNCTION__); printf x ; } }
#define DPRINTFN(n,x)	{ if (ohcidebug > (n)) { printf("%s: ", __FUNCTION__); printf x ; } }
int ohcidebug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ohci, CTLFLAG_RW, 0, "USB ohci");
SYSCTL_INT(_hw_usb_ohci, OID_AUTO, debug, CTLFLAG_RW,
	   &ohcidebug, 0, "ohci debug level");
static void		ohci_dumpregs(ohci_softc_t *);
static void		ohci_dump_tds(ohci_td_t *);
static uint8_t		ohci_dump_td(ohci_td_t *);
static void		ohci_dump_ed(ohci_ed_t *);
static uint8_t		ohci_dump_itd(ohci_itd_t *);
static void		ohci_dump_itds(ohci_itd_t *);
#endif

#define OBARR(sc) bus_space_barrier((sc)->iot, (sc)->ioh, 0, (sc)->sc_size, \
			BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE)
#define OWRITE1(sc, r, x) \
 do { OBARR(sc); bus_space_write_1((sc)->iot, (sc)->ioh, (r), (x)); } while (0)
#define OWRITE2(sc, r, x) \
 do { OBARR(sc); bus_space_write_2((sc)->iot, (sc)->ioh, (r), (x)); } while (0)
#define OWRITE4(sc, r, x) \
 do { OBARR(sc); bus_space_write_4((sc)->iot, (sc)->ioh, (r), (x)); } while (0)
#define OREAD1(sc, r) (OBARR(sc), bus_space_read_1((sc)->iot, (sc)->ioh, (r)))
#define OREAD2(sc, r) (OBARR(sc), bus_space_read_2((sc)->iot, (sc)->ioh, (r)))
#define OREAD4(sc, r) (OBARR(sc), bus_space_read_4((sc)->iot, (sc)->ioh, (r)))

#define OHCI_INTR_ENDPT 1

extern struct usbd_bus_methods ohci_bus_methods;
extern struct usbd_pipe_methods ohci_device_bulk_methods;
extern struct usbd_pipe_methods ohci_device_ctrl_methods;
extern struct usbd_pipe_methods ohci_device_intr_methods;
extern struct usbd_pipe_methods ohci_device_isoc_methods;
extern struct usbd_pipe_methods ohci_root_ctrl_methods;
extern struct usbd_pipe_methods ohci_root_intr_methods;

#define SC_HW_PHYSADDR(sc,what) \
  ((sc)->sc_hw_page.physaddr + \
   POINTER_TO_UNSIGNED(&(((struct ohci_hw_softc *)0)->what)))

static usbd_status
ohci_controller_init(ohci_softc_t *sc)
{
	int i;
	u_int32_t s, ctl, ival, hcr, fm, per, desca;

	/* Determine in what context we are running. */
	ctl = OREAD4(sc, OHCI_CONTROL);
	if(ctl & OHCI_IR)
	{
		/* SMM active, request change */
		DPRINTF(("SMM active, request owner change\n"));
		s = OREAD4(sc, OHCI_COMMAND_STATUS);
		OWRITE4(sc, OHCI_COMMAND_STATUS, s | OHCI_OCR);
		for(i = 0; (i < 100) && (ctl & OHCI_IR); i++)
		{
			DELAY(1000*1);
			ctl = OREAD4(sc, OHCI_CONTROL);
		}
		if((ctl & OHCI_IR) == 0)
		{
			device_printf(sc->sc_bus.bdev, "SMM does not respond, resetting\n");
			OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
			goto reset;
		}
#if 0
/* Don't bother trying to reuse the BIOS init, we'll reset it anyway. */
	} else if((ctl & OHCI_HCFS_MASK) != OHCI_HCFS_RESET) {
		/* BIOS started controller. */
		DPRINTF(("BIOS active\n"));
		if((ctl & OHCI_HCFS_MASK) != OHCI_HCFS_OPERATIONAL) {
			OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_OPERATIONAL);
			DELAY(1000*USB_RESUME_DELAY);
		}
#endif
	} else {
		DPRINTF(("cold started\n"));
	reset:
		/* controller was cold started */
		DELAY(1000*USB_BUS_RESET_DELAY);
	}

	/*
	 * This reset should not be necessary according to the OHCI spec, but
	 * without it some controllers do not start.
	 */
	DPRINTF(("%s: resetting\n", device_get_nameunit(sc->sc_bus.bdev)));
	OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
	DELAY(1000*USB_BUS_RESET_DELAY);

	/* we now own the host controller and the bus has been reset */
	ival = OHCI_GET_IVAL(OREAD4(sc, OHCI_FM_INTERVAL));

	OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_HCR); /* Reset HC */
	/* nominal time for a reset is 10 us */
	for(i = 0; i < 10; i++)
	{
		DELAY(10);
		hcr = OREAD4(sc, OHCI_COMMAND_STATUS) & OHCI_HCR;
		if(!hcr)
		{
			break;
		}
	}
	if (hcr)
	{
		device_printf(sc->sc_bus.bdev, "reset timeout\n");
		return (USBD_IOERROR);
	}
#ifdef USB_DEBUG
	if(ohcidebug > 15)
	{
		ohci_dumpregs(sc);
	}
#endif

	/* The controller is now in SUSPEND state, we have 2ms to finish. */

	/* set up HC registers */
	OWRITE4(sc, OHCI_HCCA, SC_HW_PHYSADDR(sc,hcca));
	OWRITE4(sc, OHCI_CONTROL_HEAD_ED, SC_HW_PHYSADDR(sc,ctrl_start));
	OWRITE4(sc, OHCI_BULK_HEAD_ED, SC_HW_PHYSADDR(sc,bulk_start));
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
	per = OHCI_PERIODIC(ival); /* 90% periodic */
	OWRITE4(sc, OHCI_PERIODIC_START, per);

	/* Fiddle the No OverCurrent Protection bit to avoid chip bug. */
	desca = OREAD4(sc, OHCI_RH_DESCRIPTOR_A);
	OWRITE4(sc, OHCI_RH_DESCRIPTOR_A, desca | OHCI_NOCP);
	OWRITE4(sc, OHCI_RH_STATUS, OHCI_LPSC); /* Enable port power */
	DELAY(1000*OHCI_ENABLE_POWER_DELAY);
	OWRITE4(sc, OHCI_RH_DESCRIPTOR_A, desca);

	/*
	 * The AMD756 requires a delay before re-reading the register,
	 * otherwise it will occasionally report 0 ports.
	 */
  	sc->sc_noport = 0;
 	for(i = 0; (i < 10) && (sc->sc_noport == 0); i++)
	{
 		DELAY(1000*OHCI_READ_DESC_DELAY);
 		sc->sc_noport = OHCI_GET_NDP(OREAD4(sc, OHCI_RH_DESCRIPTOR_A));
 	}

#ifdef USB_DEBUG
	if(ohcidebug > 5)
	{
		ohci_dumpregs(sc);
	}
#endif
	return (USBD_NORMAL_COMPLETION);
}

usbd_status
ohci_init(ohci_softc_t *sc)
{
	struct ohci_hw_softc *hw_ptr;
	u_int i;
	u_int16_t bit;
	u_int16_t x;
	u_int16_t y;

	mtx_lock(&sc->sc_bus.mtx);

	DPRINTF(("start\n"));

	hw_ptr = sc->sc_hw_ptr;

	sc->sc_eintrs = OHCI_NORMAL_INTRS;

	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_PREWRITE);

	/*
	 * setup self pointers
	 */
	hw_ptr->ctrl_start.ed_self = htole32(SC_HW_PHYSADDR(sc,ctrl_start));
	hw_ptr->ctrl_start.ed_flags = htole32(OHCI_ED_SKIP);
	hw_ptr->ctrl_start.page = &(sc->sc_hw_page);
	sc->sc_ctrl_p_last = &(hw_ptr->ctrl_start);

	hw_ptr->bulk_start.ed_self = htole32(SC_HW_PHYSADDR(sc,bulk_start));
	hw_ptr->bulk_start.ed_flags = htole32(OHCI_ED_SKIP);
	hw_ptr->bulk_start.page = &(sc->sc_hw_page);
	sc->sc_bulk_p_last = &(hw_ptr->bulk_start);

	hw_ptr->isoc_start.ed_self = htole32(SC_HW_PHYSADDR(sc,isoc_start));
	hw_ptr->isoc_start.ed_flags = htole32(OHCI_ED_SKIP);
	hw_ptr->isoc_start.page = &(sc->sc_hw_page);
	sc->sc_isoc_p_last = &(hw_ptr->isoc_start);

	for(i = 0;
	    i < OHCI_NO_EDS;
	    i++)
	{
		hw_ptr->intr_start[i].ed_self = htole32(SC_HW_PHYSADDR(sc,intr_start[i]));
		hw_ptr->intr_start[i].ed_flags = htole32(OHCI_ED_SKIP);
		hw_ptr->intr_start[i].page = &(sc->sc_hw_page);
		sc->sc_intr_p_last[i] = &(hw_ptr->intr_start[i]);
	}

	/*
	 * the QHs are arranged to give poll intervals that are
	 * powers of 2 times 1ms
	 */
	bit = OHCI_NO_EDS/2;
	while(bit)
	{
		x = bit;
		while(x & bit)
		{
			y = (x ^ bit)|(bit/2);
			/* the next QH has half the
			 * poll interval
			 */
			hw_ptr->intr_start[x].next = NULL;
			hw_ptr->intr_start[x].ed_next =
			  hw_ptr->intr_start[y].ed_self;
			x++;
		}
		bit >>= 1;
	}

	/* the last (1ms) QH */
	hw_ptr->intr_start[0].next = &(hw_ptr->isoc_start);
	hw_ptr->intr_start[0].ed_next = hw_ptr->isoc_start.ed_self;

	/*
	 * Fill HCCA interrupt table.  The bit reversal is to get
	 * the tree set up properly to spread the interrupts.
	 */
	for(i = 0;
	    i < OHCI_NO_INTRS;
	    i++)
	{
		hw_ptr->hcca.hcca_interrupt_table[i] =
		  hw_ptr->intr_start[i|(OHCI_NO_EDS/2)].ed_self;
	}

	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_POSTWRITE);

	LIST_INIT(&sc->sc_interrupt_list_head);

	/* set up the bus struct */
	sc->sc_bus.methods = &ohci_bus_methods;

	__callout_init_mtx(&sc->sc_tmo_rhsc,  &sc->sc_bus.mtx, 
			   CALLOUT_RETURNUNLOCKED);

#ifdef USB_DEBUG
	if(ohcidebug > 15)
	{
		for(i = 0; 
		    i < OHCI_NO_EDS; 
		    i++)
		{
			printf("ed#%d ", i);
			ohci_dump_ed(&(hw_ptr->intr_start[i]));
		}
		printf("iso ");
		ohci_dump_ed(&(hw_ptr->isoc_start));
	}
#endif

	sc->sc_control = sc->sc_intre = 0;

	device_printf(sc->sc_bus.bdev, " ");

	sc->sc_bus.usbrev = USBREV_1_0;

	if(ohci_controller_init(sc))
	{
		mtx_unlock(&sc->sc_bus.mtx);
		return (USBD_INVAL);
	}
	else
	{
		mtx_unlock(&sc->sc_bus.mtx);
		return (USBD_NORMAL_COMPLETION);
	}
}

/*
 * shut down the controller when the system is going down
 */
void
ohci_detach(struct ohci_softc *sc)
{
	mtx_lock(&sc->sc_bus.mtx);

	__callout_stop(&sc->sc_tmo_rhsc);

	OWRITE4(sc, OHCI_INTERRUPT_DISABLE, OHCI_ALL_INTRS);
	OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);

	DELAY(1000*300); /* XXX let stray task complete */

	mtx_unlock(&sc->sc_bus.mtx);

	__callout_drain(&(sc->sc_tmo_rhsc));

	return;
}

/* NOTE: suspend/resume is called from
 * interrupt context and cannot sleep!
 */
void
ohci_suspend(ohci_softc_t *sc)
{
	u_int32_t ctl;
	mtx_lock(&sc->sc_bus.mtx);

#ifdef USB_DEBUG
	DPRINTF(("\n"));
	if(ohcidebug > 2)
	{
		ohci_dumpregs(sc);
	}
#endif

	ctl = OREAD4(sc, OHCI_CONTROL) & ~OHCI_HCFS_MASK;
	if(sc->sc_control == 0)
	{
		/*
		 * Preserve register values, in case that APM BIOS
		 * does not recover them.
		 */
		sc->sc_control = ctl;
		sc->sc_intre = OREAD4(sc, OHCI_INTERRUPT_ENABLE);
	}
	ctl |= OHCI_HCFS_SUSPEND;
	OWRITE4(sc, OHCI_CONTROL, ctl);
	DELAY(1000*USB_RESUME_WAIT);

	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

void
ohci_resume(ohci_softc_t *sc)
{
	u_int32_t ctl;
	mtx_lock(&sc->sc_bus.mtx);

#ifdef USB_DEBUG
	DPRINTF(("\n"));
	if(ohcidebug > 2)
	{
		ohci_dumpregs(sc);
	}
#endif
	/* some broken BIOSes never initialize the Controller chip */
	ohci_controller_init(sc);

	if(sc->sc_intre)
	{
		OWRITE4(sc, OHCI_INTERRUPT_ENABLE,
			sc->sc_intre & (OHCI_ALL_INTRS | OHCI_MIE));
	}

	if(sc->sc_control)
		ctl = sc->sc_control;
	else
		ctl = OREAD4(sc, OHCI_CONTROL);
	ctl |= OHCI_HCFS_RESUME;
	OWRITE4(sc, OHCI_CONTROL, ctl);
	DELAY(1000*USB_RESUME_DELAY);
	ctl = (ctl & ~OHCI_HCFS_MASK) | OHCI_HCFS_OPERATIONAL;
	OWRITE4(sc, OHCI_CONTROL, ctl);
	DELAY(1000*USB_RESUME_RECOVERY);
	sc->sc_control = sc->sc_intre = 0;

	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

#ifdef USB_DEBUG
static void
ohci_dumpregs(ohci_softc_t *sc)
{
	struct ohci_hw_softc *hw_ptr;

	DPRINTF(("ohci_dumpregs: rev=0x%08x control=0x%08x command=0x%08x\n",
		 OREAD4(sc, OHCI_REVISION),
		 OREAD4(sc, OHCI_CONTROL),
		 OREAD4(sc, OHCI_COMMAND_STATUS)));
	DPRINTF(("               intrstat=0x%08x intre=0x%08x intrd=0x%08x\n",
		 OREAD4(sc, OHCI_INTERRUPT_STATUS),
		 OREAD4(sc, OHCI_INTERRUPT_ENABLE),
		 OREAD4(sc, OHCI_INTERRUPT_DISABLE)));
	DPRINTF(("               hcca=0x%08x percur=0x%08x ctrlhd=0x%08x\n",
		 OREAD4(sc, OHCI_HCCA),
		 OREAD4(sc, OHCI_PERIOD_CURRENT_ED),
		 OREAD4(sc, OHCI_CONTROL_HEAD_ED)));
	DPRINTF(("               ctrlcur=0x%08x bulkhd=0x%08x bulkcur=0x%08x\n",
		 OREAD4(sc, OHCI_CONTROL_CURRENT_ED),
		 OREAD4(sc, OHCI_BULK_HEAD_ED),
		 OREAD4(sc, OHCI_BULK_CURRENT_ED)));
	DPRINTF(("               done=0x%08x fmival=0x%08x fmrem=0x%08x\n",
		 OREAD4(sc, OHCI_DONE_HEAD),
		 OREAD4(sc, OHCI_FM_INTERVAL),
		 OREAD4(sc, OHCI_FM_REMAINING)));
	DPRINTF(("               fmnum=0x%08x perst=0x%08x lsthrs=0x%08x\n",
		 OREAD4(sc, OHCI_FM_NUMBER),
		 OREAD4(sc, OHCI_PERIODIC_START),
		 OREAD4(sc, OHCI_LS_THRESHOLD)));
	DPRINTF(("               desca=0x%08x descb=0x%08x stat=0x%08x\n",
		 OREAD4(sc, OHCI_RH_DESCRIPTOR_A),
		 OREAD4(sc, OHCI_RH_DESCRIPTOR_B),
		 OREAD4(sc, OHCI_RH_STATUS)));
	DPRINTF(("               port1=0x%08x port2=0x%08x\n",
		 OREAD4(sc, OHCI_RH_PORT_STATUS(1)),
		 OREAD4(sc, OHCI_RH_PORT_STATUS(2))));

	hw_ptr = sc->sc_hw_ptr;

	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_PREREAD);
	DPRINTF(("         HCCA: frame_number=0x%04x done_head=0x%08x\n",
		 le32toh(hw_ptr->hcca.hcca_frame_number),
		 le32toh(hw_ptr->hcca.hcca_done_head)));
	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_POSTREAD);
	return;
}
static void
ohci_dump_tds(ohci_td_t *std)
{
	for(; std; std = std->obj_next)
	{
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

	usbd_page_sync(std->page, BUS_DMASYNC_PREREAD);

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

	usbd_page_sync(std->page, BUS_DMASYNC_POSTREAD);
	return temp;
}

static uint8_t
ohci_dump_itd(ohci_itd_t *sitd)
{
	uint32_t itd_flags;
	uint16_t i;
	uint8_t temp;

	usbd_page_sync(sitd->page, BUS_DMASYNC_PREREAD);

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
	for(i = 0; i < OHCI_ITD_NOFFSET; i++)
	{
		printf("offs[%d]=0x%04x ", i,
		       (u_int)le16toh(sitd->itd_offset[i]));
	}
	printf("\n");

	usbd_page_sync(sitd->page, BUS_DMASYNC_POSTREAD);
	return temp;
}

static void
ohci_dump_itds(ohci_itd_t *sitd)
{
	for(; sitd; sitd = sitd->obj_next)
	{
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
	u_int32_t ed_headp;

	usbd_page_sync(sed->page, BUS_DMASYNC_PREREAD);

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

	usbd_page_sync(sed->page, BUS_DMASYNC_POSTREAD);
	return;
}
#endif


#define OHCI_APPEND_QH(sed,td_self,last) (last) = _ohci_append_qh(sed,td_self,last)
static ohci_ed_t *
_ohci_append_qh(ohci_ed_t *sed, uint32_t td_self, ohci_ed_t *last)
{
	DPRINTFN(10, ("%p to %p\n", sed, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_sync(sed->page, BUS_DMASYNC_PREWRITE);

	sed->next = last->next;
	sed->ed_next = last->ed_next;
	sed->ed_tailp = 0;
	sed->ed_headp = td_self;

	sed->prev = last;

	usbd_page_sync(sed->page, BUS_DMASYNC_POSTWRITE);
	usbd_page_sync(last->page, BUS_DMASYNC_PREWRITE);

	/* the last->next->prev is never followed:
	 * sed->next->prev = sed;
	 */

	last->next = sed;
	last->ed_next = sed->ed_self;

	usbd_page_sync(last->page, BUS_DMASYNC_POSTWRITE);

	return(sed);
}

#define OHCI_REMOVE_QH(sed,last) (last) = _ohci_remove_qh(sed,last)
static ohci_ed_t *
_ohci_remove_qh(ohci_ed_t *sed, ohci_ed_t *last)
{
	DPRINTFN(10, ("%p from %p\n", sed, last));

	/* (sc->sc_bus.mtx) must be locked */

	/* only remove if not removed from a queue */
	if(sed->prev)
	{
		usbd_page_sync(sed->prev->page, BUS_DMASYNC_PREWRITE);

		sed->prev->next = sed->next;
		sed->prev->ed_next = sed->ed_next;

		usbd_page_sync(sed->prev->page, BUS_DMASYNC_POSTWRITE);

		if(sed->next)
		{
			sed->next->prev = sed->prev;
		}

		usbd_page_sync(sed->page, BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

		/* terminate transfer in case the
		 * transferred packet was short so
		 * that the ED still points at the
		 * last used TD
		 */
		sed->ed_flags |= htole32(OHCI_ED_SKIP);
		sed->ed_headp = sed->ed_tailp;

		usbd_page_sync(sed->page, BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);

		last = ((last == sed) ? sed->prev : last);

		sed->prev = 0;
	}
	return(last);
}

static void
ohci_device_done(struct usbd_xfer *xfer, usbd_status error);

static void
ohci_isoc_done(struct usbd_xfer *xfer)
{
	u_int8_t nframes;
	u_int32_t actlen = 0;
	u_int16_t *plen = xfer->frlengths;
	__volatile__ u_int16_t *olen;
	u_int16_t len = 0;
	ohci_itd_t *td = xfer->td_transfer_first;

	while(1)
	{
		if(td == NULL)
		{
		    panic("%s:%d: out of TD's\n",
			  __FUNCTION__, __LINE__);
		}

#ifdef USB_DEBUG
		if(ohcidebug > 5)
		{
			DPRINTFN(-1,("isoc TD\n"));
			ohci_dump_itd(td);
		}
#endif
		usbd_page_sync(td->page, BUS_DMASYNC_PREREAD);

		nframes = td->frames;
		olen = &td->itd_offset[0];

		if (nframes > 8) {
		    nframes = 8;
		}

		while(nframes--)
		{
			len = le16toh(*olen);

			if((len >> 12) == OHCI_CC_NOT_ACCESSED)
			{
				len = 0;
			}
			else
			{
				len &= ((1<<12)-1);
			}

			if (len > *plen) {
			    len = 0; /* invalid length */
			}

			*plen = len;
			actlen += len;
			plen++;
			olen++;
		}

		usbd_page_sync(td->page, BUS_DMASYNC_POSTREAD);

		if(((void *)td) == xfer->td_transfer_last)
		{
			break;
		}

		td = td->obj_next;
	}
	xfer->actlen = actlen;
	ohci_device_done(xfer,USBD_NORMAL_COMPLETION);
	return;
}

#ifdef USB_DEBUG
static const char * const
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

static void
ohci_non_isoc_done(struct usbd_xfer *xfer)
{
	u_int16_t cc = 0;
	u_int32_t actlen = 0;
	u_int32_t len;
	u_int32_t temp;
	u_int32_t phy_start;
	u_int32_t phy_end;
	uint32_t td_flags;
	ohci_td_t *td = xfer->td_transfer_first;

	DPRINTFN(12, ("xfer=%p pipe=%p transfer done\n",
		      xfer, xfer->pipe));

#ifdef USB_DEBUG
	if(ohcidebug > 10)
	{
		ohci_dump_tds(td);
	}
#endif

	while(1)
	{
		if(td == NULL)
		{
			panic("%s:%d: out of TD's\n",
			      __FUNCTION__, __LINE__);
		}

		usbd_page_sync(td->page, BUS_DMASYNC_PREREAD);

		phy_start = le32toh(td->td_cbp);
		td_flags = le32toh(td->td_flags);

		len = td->len;
		if (phy_start) {
			/* compute the number of remaining
			 * bytes in the hardware buffer:
			 */
			phy_end = le32toh(td->td_be);
			temp = (OHCI_PAGE(phy_start ^ phy_end) ? 
				(OHCI_PAGE_SIZE+1) : 0x0001);
			temp += OHCI_PAGE_OFFSET(phy_end);
			temp -= OHCI_PAGE_OFFSET(phy_start);

			if (temp > len) {
			    /* guard against corruption */
			    len = 0;
			} else {
			    len -= temp;
			}
		}

		usbd_page_sync(td->page, BUS_DMASYNC_POSTREAD);

		DPRINTFN(10, ("len=%d\n", len));

		actlen += len;

		cc = OHCI_TD_GET_CC(td_flags);
		if (cc) {
			DPRINTFN(15,("error cc=%d (%s)\n",
				     cc, ohci_cc_strs[cc]));
			break;
		}

		if (phy_start) {
			/* short transfer */
			break;
		}

		if (((void *)td) == xfer->td_transfer_last) {
			break;
		}

		td = td->obj_next;
	}

	DPRINTFN(10, ("actlen=%d\n", actlen));

	xfer->actlen = actlen;

	ohci_device_done(xfer, 
			 (cc == 0) ? USBD_NORMAL_COMPLETION :
			 (cc == OHCI_CC_STALL) ? USBD_STALLED : USBD_IOERROR);
	return;
}

/* returns one when transfer is finished 
 * and callback must be called; else zero
 */
static u_int8_t
ohci_check_transfer(struct usbd_xfer *xfer, struct thread *ctd)
{
	ohci_ed_t *ed = xfer->qh_start;
	uint32_t ed_flags;
	uint32_t ed_headp;
	uint32_t ed_tailp;

	if(xfer->usb_thread != ctd)
	{
	    /* cannot call this transfer 
	     * back due to locking !
	     */
	    return 0;
	}

	DPRINTFN(12, ("xfer=%p checking transfer\n", xfer));

	usbd_page_sync(ed->page, BUS_DMASYNC_PREREAD);
	ed_flags = le32toh(ed->ed_flags);
	ed_headp = le32toh(ed->ed_headp);
	ed_tailp = le32toh(ed->ed_tailp);
	usbd_page_sync(ed->page, BUS_DMASYNC_POSTREAD);

	if ((ed_flags & OHCI_ED_SKIP) ||
	    (ed_headp & OHCI_HALTED) ||
	    (((ed_headp ^ ed_tailp) & (~0xF)) == 0))
	{
		if(xfer->pipe->methods == &ohci_device_isoc_methods)
		{
			/* isochronous transfer */
			ohci_isoc_done(xfer);
		}
		else
		{
			/* store data-toggle */
			if (ed_headp & OHCI_TOGGLECARRY) {
				xfer->pipe->toggle_next = 1;
			} else {
				xfer->pipe->toggle_next = 0;
			}

			/* non-isochronous transfer */
			ohci_non_isoc_done(xfer);
		}
		return 1;
	}

	DPRINTFN(12, ("xfer=%p is still active\n", xfer));
	return 0;
}

static void
ohci_rhsc_enable(ohci_softc_t *sc)
{
	struct usbd_callback_info info[1];
	struct usbd_xfer *xfer;

	DPRINTFN(4, ("\n"));

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	sc->sc_eintrs |= OHCI_RHSC;
	OWRITE4(sc, OHCI_INTERRUPT_ENABLE, OHCI_RHSC);

	/* acknowledge any RHSC interrupt */
	OWRITE4(sc, OHCI_INTERRUPT_STATUS, OHCI_RHSC);

	xfer = sc->sc_intrxfer;

 	if(xfer)
	{
	    /* transfer is transferred */
	    ohci_device_done(xfer, USBD_NORMAL_COMPLETION);

	    /* queue callback */
	    info[0].xfer = xfer;
	    info[0].refcount = xfer->usb_refcount;

	    xfer->usb_root->memory_refcount++;

	    mtx_unlock(&sc->sc_bus.mtx);

	    usbd_do_callback(&info[0],&info[1]);
	}
	else
	{
	    mtx_unlock(&sc->sc_bus.mtx);
	}
	return;
}

static void
ohci_interrupt_td(ohci_softc_t *sc, struct thread *ctd)
{
	enum { FINISH_LIST_MAX = 16 };

	struct usbd_callback_info info[FINISH_LIST_MAX];
	struct usbd_callback_info *ptr = &info[0];
	struct ohci_hw_softc *hw_ptr;
	struct usbd_xfer *xfer;
	u_int32_t status;
	u_int32_t done;
	u_int8_t need_repeat = 0;

	mtx_lock(&sc->sc_bus.mtx);

	if(sc->sc_bus.bdev == NULL)
	{
		/* too early interrupt */
		goto done;
	}

	if(ctd)
	{
		/* the poll thread should not read
		 * any status registers that will
		 * clear interrupts!
		 */
		goto repeat;
	}

	sc->sc_bus.no_intrs++;
	hw_ptr = sc->sc_hw_ptr;

	DPRINTFN(15,("%s: real interrupt\n",
		     device_get_nameunit(sc->sc_bus.bdev)));

#ifdef USB_DEBUG
	if(ohcidebug > 15)
	{
		DPRINTF(("%s:\n", device_get_nameunit(sc->sc_bus.bdev)));
		ohci_dumpregs(sc);
	}
#endif

	status = 0;

	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);
	done = le32toh(hw_ptr->hcca.hcca_done_head);

	/* The LSb of done is used to inform the HC Driver that an interrupt
	 * condition exists for both the Done list and for another event
	 * recorded in HcInterruptStatus. On an interrupt from the HC, the HC
	 * Driver checks the HccaDoneHead Value. If this value is 0, then the
	 * interrupt was caused by other than the HccaDoneHead update and the
	 * HcInterruptStatus register needs to be accessed to determine that
	 * exact interrupt cause. If HccaDoneHead is nonzero, then a Done list
	 * update interrupt is indicated and if the LSb of done is nonzero,
	 * then an additional interrupt event is indicated and
	 * HcInterruptStatus should be checked to determine its cause.
	 */
	if(done != 0)
	{
		if(done & ~OHCI_DONE_INTRS)
		{
			status |= OHCI_WDH;
		}
		if(done & OHCI_DONE_INTRS)
		{
			status |= OREAD4(sc, OHCI_INTERRUPT_STATUS);
			done &= ~OHCI_DONE_INTRS;
		}
		hw_ptr->hcca.hcca_done_head = 0;
	}
	else
	{
		status = OREAD4(sc, OHCI_INTERRUPT_STATUS) & ~OHCI_WDH;
	}

	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);

	if(status == 0)		/* nothing to be done (PCI shared interrupt) */
	{
		goto done;
	}

	status &= ~OHCI_MIE;
	OWRITE4(sc, OHCI_INTERRUPT_STATUS, status); /* Acknowledge */

	status &= sc->sc_eintrs;
	if(status == 0)
	{
		goto done;
	}

#if 0
	if(status & OHCI_SO)
	{
		/* XXX do what */
	}
#endif
	if(status & OHCI_RD)
	{
		device_printf(sc->sc_bus.bdev, "resume detect\n");
		/* XXX process resume detect */
	}
	if(status & OHCI_UE)
	{
		device_printf(sc->sc_bus.bdev, "unrecoverable error, "
			      "controller halted\n");
		OWRITE4(sc, OHCI_CONTROL, OHCI_HCFS_RESET);
		/* XXX what else */
	}
	if(status & OHCI_RHSC)
	{
		xfer = sc->sc_intrxfer;

		if(xfer)
		{
		    ohci_device_done(xfer, USBD_NORMAL_COMPLETION);

		    /* queue callback */
		    ptr->xfer = xfer;
		    ptr->refcount = xfer->usb_refcount;
		    ptr++;
		    xfer->usb_root->memory_refcount++;
		}

		/*
		 * Disable RHSC interrupt for now, because it will be
		 * on until the port has been reset.
		 */
		sc->sc_eintrs &= ~OHCI_RHSC;
		OWRITE4(sc, OHCI_INTERRUPT_DISABLE, OHCI_RHSC);

		/* do not allow RHSC interrupts > 1 per second */
		__callout_reset(&sc->sc_tmo_rhsc, hz,
				(void *)(void *)ohci_rhsc_enable, sc);
	}

	status &= ~(OHCI_RHSC|OHCI_WDH|OHCI_SO);
	if(status != 0)
	{
		/* Block unprocessed interrupts. XXX */
		OWRITE4(sc, OHCI_INTERRUPT_DISABLE, status);
		sc->sc_eintrs &= ~status;
		device_printf(sc->sc_bus.bdev, 
			      "blocking intrs 0x%x\n", status);
	}

	/*
	 * when the host controller interrupts because a transfer
	 * is completed, all active transfers are checked!
	 */

 repeat:
	LIST_FOREACH(xfer, &sc->sc_interrupt_list_head, interrupt_list)
	{
		/* check if transfer is
		 * transferred 
		 */
		if(ohci_check_transfer(xfer, ctd))
		{
		    /* queue callback */
		    ptr->xfer = xfer;
		    ptr->refcount = xfer->usb_refcount;
		    ptr++;

		    xfer->usb_root->memory_refcount++;

		    /* check queue length */
		    if(ptr >= &info[FINISH_LIST_MAX])
		    {
		        need_repeat = 1;
			break;
		    }
		}
	}

 done:
	mtx_unlock(&sc->sc_bus.mtx);

	usbd_do_callback(&info[0],ptr);
	if(need_repeat)
	{
		ptr = &info[0];

		need_repeat = 0;
		mtx_lock(&sc->sc_bus.mtx);

		goto repeat;
	}
	return;
}

void
ohci_interrupt(ohci_softc_t *sc)
{
	ohci_interrupt_td(sc, NULL);
	return;
}

/*
 * called when a request does not complete
 */
static void
ohci_timeout(struct usbd_xfer *xfer)
{
	struct usbd_callback_info info[1];
	ohci_softc_t *sc = xfer->usb_sc;

	DPRINTF(("xfer=%p\n", xfer));

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	ohci_device_done(xfer, USBD_TIMEOUT);

	/* queue callback */
	info[0].xfer = xfer;
	info[0].refcount = xfer->usb_refcount;

	xfer->usb_root->memory_refcount++;

	mtx_unlock(&sc->sc_bus.mtx);

	usbd_do_callback(&info[0],&info[1]);

	return;
}

static void
ohci_do_poll(struct usbd_bus *bus)
{
	ohci_interrupt_td(OHCI_BUS2SC(bus), curthread);
	return;
}

#define ohci_add_interrupt_info(sc, xfer) \
	LIST_INSERT_HEAD(&(sc)->sc_interrupt_list_head, (xfer), interrupt_list)

static void
ohci_remove_interrupt_info(struct usbd_xfer *xfer)
{
	if((xfer)->interrupt_list.le_prev)
	{
		LIST_REMOVE((xfer), interrupt_list);
		(xfer)->interrupt_list.le_prev = NULL;
	}
	return;
}

static void
ohci_setup_standard_chain(struct usbd_xfer *xfer, ohci_ed_t **ed_last)
{
	struct usbd_page_search buf_res;
	/* the OHCI hardware can handle at most one 4k crossing per TD */
	u_int32_t average = (OHCI_PAGE_SIZE - (OHCI_PAGE_SIZE % 
					       xfer->max_packet_size));
	u_int32_t td_flags;
	uint32_t ed_flags;
	u_int32_t buf_offset;
	u_int32_t len = xfer->length;
	u_int8_t isread;
	u_int8_t shortpkt = 0;
	ohci_td_t *td;
	ohci_td_t *td_last = NULL;
	ohci_ed_t *ed;

	DPRINTFN(8, ("addr=%d endpt=%d len=%d speed=%d\n", 
		     xfer->address, UE_GET_ADDR(xfer->endpoint),
		     xfer->length, xfer->udev->speed));

	td = (xfer->td_transfer_first = xfer->td_start);

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	if(xfer->pipe->methods == &ohci_device_ctrl_methods)
	{
		/* the first byte is "bmRequestType" */

		isread = *((u_int8_t *)(buf_res.buffer));
		isread &= UT_READ;

		/*
		 * check length ?
		 */
		xfer->pipe->toggle_next = 1;

		usbd_page_sync(td->page, BUS_DMASYNC_PREWRITE);

		/* SETUP message */

		td->td_flags = htole32(OHCI_TD_SETUP | OHCI_TD_NOCC |
				       OHCI_TD_TOGGLE_0 | OHCI_TD_NOINTR);

		td->td_cbp = htole32(buf_res.physaddr);

		buf_offset += (sizeof(usb_device_request_t) - 1);
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		td->td_be = htole32(buf_res.physaddr);
		td->len = sizeof(usb_device_request_t);

		buf_offset += 1;
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		len -= sizeof(usb_device_request_t);
		td_last = td;
		td = td->obj_next;

		if (td) {
		    /* link the last TD with the next one */
		    td_last->td_next = td->td_self;
		}

		usbd_page_sync(td_last->page, BUS_DMASYNC_POSTWRITE);
	}
	else
	{
		isread = (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN);

		if(xfer->length == 0)
		{
			/* must allow access to "td_last",
			 * so xfer->length cannot be zero!
			 */
			printf("%s: setting USBD_FORCE_SHORT_XFER!\n",
			       __FUNCTION__);
			xfer->flags |= USBD_FORCE_SHORT_XFER;
		}
	}

	td_flags = htole32(OHCI_TD_NOCC | OHCI_TD_NOINTR);

 	if(xfer->flags & USBD_SHORT_XFER_OK)
	{
		td_flags |= htole32(OHCI_TD_R);
	}
	if(xfer->pipe->toggle_next)
	{
		td_flags |= htole32(OHCI_TD_TOGGLE_1);
	}
	else
	{
		td_flags |= htole32(OHCI_TD_TOGGLE_0);
	}
	if(isread)
        {
		td_flags |= htole32(OHCI_TD_IN);
	}
	else
	{
		td_flags |= htole32(OHCI_TD_OUT);
	}

	while(1)
	{
		if(len == 0)
		{
			if(xfer->flags & USBD_FORCE_SHORT_XFER)
			{
				if(shortpkt)
				{
					break;
				}
			}
			else
			{
				break;
			}
		}

		if(len < average)
		{
			if((len % xfer->max_packet_size) || 
			   (len == 0))
			{
				shortpkt = 1;
			}

			average = len;
		}

		if(td == NULL)
		{
			panic("%s: software wants to write more data "
			      "than there is in the buffer!", __FUNCTION__);
		}

		usbd_page_sync(td->page, BUS_DMASYNC_PREWRITE);

		/* fill out TD */

		td->td_flags = td_flags;

		/* the next TD uses TOGGLE_CARRY */
		td_flags &= htole32(~OHCI_TD_TOGGLE_MASK);

		if(average == 0)
		{
			td->td_cbp = 0;
			td->td_be = ~0;
		}
		else
		{
			td->td_cbp = htole32(buf_res.physaddr);

			buf_offset += (average - 1);
			usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

			td->td_be = htole32(buf_res.physaddr);

			buf_offset += 1;
			usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);
		}
		td->len = average;

		len -= average;
		td_last = td;
		td = td->obj_next;

		if (td) {
		    /* link the last TD with the next one */
		    td_last->td_next = td->td_self;
		}

		usbd_page_sync(td_last->page, BUS_DMASYNC_POSTWRITE);
	}

	if(xfer->pipe->methods == &ohci_device_ctrl_methods)
	{
		usbd_page_sync(td->page, BUS_DMASYNC_PREWRITE);

		/* STATUS message */

		td_flags = (OHCI_TD_NOCC | OHCI_TD_TOGGLE_1 | 
			    OHCI_TD_SET_DI(1));

		if (isread) {
			td_flags |= OHCI_TD_OUT;
		} else {
			td_flags |= OHCI_TD_IN;
		}

		td->td_flags = htole32(td_flags);
		td->td_cbp = 0;
		td->td_be = ~0;
		td->len = 0;

		td_last = td;

		usbd_page_sync(td_last->page, BUS_DMASYNC_POSTWRITE);
	}

	usbd_page_sync(td_last->page, BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

	td_last->td_next = 0;
	td_last->td_flags &= htole32(~OHCI_TD_INTR_MASK);
	td_last->td_flags |= htole32(OHCI_TD_SET_DI(1));

	usbd_page_sync(td_last->page, BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);

	/* must have at least one frame! */

	xfer->td_transfer_last = td_last;

#ifdef USB_DEBUG
	if(ohcidebug > 8)
	{
		DPRINTF(("nexttog=%d; data before transfer:\n",
			 xfer->pipe->toggle_next));
		ohci_dump_tds(xfer->td_start);
	}
#endif

	ed = xfer->qh_start;

	usbd_page_sync(ed->page, BUS_DMASYNC_PREWRITE);

	ed_flags = (OHCI_ED_SET_FA(xfer->address)|
		    OHCI_ED_SET_EN(UE_GET_ADDR(xfer->endpoint))|
		    OHCI_ED_SET_MAXP(xfer->max_packet_size));

	ed_flags |= (OHCI_ED_FORMAT_GEN | OHCI_ED_DIR_TD);
	
	if (xfer->udev->speed == USB_SPEED_LOW) {
		ed_flags |= OHCI_ED_SPEED;
	}

	ed->ed_flags = htole32(ed_flags);

	usbd_page_sync(ed->page, BUS_DMASYNC_POSTWRITE);

	td = xfer->td_transfer_first;

	OHCI_APPEND_QH(ed, td->td_self, *ed_last);

	if(xfer->pipe->methods == &ohci_device_bulk_methods)
	{
		ohci_softc_t *sc = xfer->usb_sc;
		OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_BLF);
	}

	if(xfer->pipe->methods == &ohci_device_ctrl_methods)
	{
		ohci_softc_t *sc = xfer->usb_sc;
		OWRITE4(sc, OHCI_COMMAND_STATUS, OHCI_CLF);
	}
	return;
}

static void
ohci_root_intr_done(ohci_softc_t *sc, struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	u_int8_t *p;
	u_int16_t i;
	u_int16_t m;
	u_int32_t hstatus;

	if(sc->sc_intrxfer)
	{
		/* disable further interrupts */
		sc->sc_intrxfer = NULL;

		/* clear all bits */
		usbd_bzero(&(xfer->buf_data), 0, xfer->length);

		hstatus = OREAD4(sc, OHCI_RH_STATUS);
		DPRINTF(("sc=%p xfer=%p hstatus=0x%08x\n",
			 sc, xfer, hstatus));

		/* set bits */
		m = (xfer->length * 8);
		i = (sc->sc_noport + 1);
		m = min(m,i);
		for(i = 1; i < m; i++)
		{
			/* pick out CHANGE bits from the status register */
			if(OREAD4(sc, OHCI_RH_PORT_STATUS(i)) >> 16)
			{
				usbd_get_page(&(xfer->buf_data), i/8, &buf_res);
				p = buf_res.buffer;
				*p |= 1 << (i % 8);

				DPRINTF(("port %d changed\n", i));
			}
		}
		xfer->actlen = xfer->length;
	}
	return;
}

/* NOTE: "done" can be run two times in a row,
 * from close and from interrupt
 */
static void
ohci_device_done(struct usbd_xfer *xfer, usbd_status error)
{
	ohci_softc_t *sc = xfer->usb_sc;
	ohci_ed_t *ed;
	u_int8_t need_delay;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	need_delay = 0;

	DPRINTFN(1,("xfer=%p, pipe=%p length=%d error=%d\n",
		    xfer, xfer->pipe, xfer->actlen, error));

	for(ed = xfer->qh_start; ed; ed = ed->obj_next)
	{
		usbd_page_sync(ed->page, BUS_DMASYNC_PREREAD);

		if((!(ed->ed_flags & htole32(OHCI_ED_SKIP))) &&
		   (!(ed->ed_headp & htole32(OHCI_HALTED))) &&
		   ((ed->ed_headp ^ ed->ed_tailp) & htole32(-0x10)))
		{
			need_delay = 1;
		}

		usbd_page_sync(ed->page, BUS_DMASYNC_POSTREAD);
	}

	if(xfer->pipe->methods == &ohci_device_bulk_methods)
	{
		OHCI_REMOVE_QH(xfer->qh_start, sc->sc_bulk_p_last);
	}

	if(xfer->pipe->methods == &ohci_device_ctrl_methods)
	{
		OHCI_REMOVE_QH(xfer->qh_start, sc->sc_ctrl_p_last);
	}

	if(xfer->pipe->methods == &ohci_device_intr_methods)
	{
		OHCI_REMOVE_QH(xfer->qh_start, sc->sc_intr_p_last[xfer->qh_pos]);
	}

	if(xfer->pipe->methods == &ohci_device_isoc_methods)
	{
		OHCI_REMOVE_QH(xfer->qh_start, sc->sc_isoc_p_last);
	}

	xfer->td_transfer_first = NULL;
	xfer->td_transfer_last = NULL;

	/* finish root interrupt transfer
	 * (will update xfer->buffer and xfer->actlen)
	 */
	if(xfer->pipe->methods == &ohci_root_intr_methods)
	{
		ohci_root_intr_done(sc, xfer);
	}

	/* stop timeout */
	__callout_stop(&xfer->timeout_handle);

	/* remove interrupt info */
	ohci_remove_interrupt_info(xfer);

	/* wait until hardware has finished any possible
	 * use of the transfer and QH
	 *
	 * hardware finishes in 1 millisecond
	 */
	DELAY(need_delay ? (2*1000) : (5));

	if(error)
	{
		/* next transfer needs to clear stall */
		xfer->pipe->clearstall = 1;
	}

	/* transfer is transferred ! */
	usbd_transfer_done(xfer,error);

	/* dequeue transfer (and start next transfer)
	 *
	 * if two transfers are queued, the second
	 * transfer must be started before the
	 * first is called back!
	 */
	usbd_transfer_dequeue(xfer);

	return;
}

/*---------------------------------------------------------------------------*
 * ohci bulk support
 *---------------------------------------------------------------------------*/
static void
ohci_device_bulk_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ohci_device_bulk_close(struct usbd_xfer *xfer)
{
	ohci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ohci_device_bulk_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ohci_device_bulk_start(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;

	DPRINTFN(3, ("xfer=%p len=%d\n",
		     xfer, xfer->length));

	/* setup TD's and QH */
	ohci_setup_standard_chain(xfer, &sc->sc_bulk_p_last);

	/**/
	ohci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ohci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods ohci_device_bulk_methods = 
{
  .open = ohci_device_bulk_open,
  .close = ohci_device_bulk_close,
  .enter = ohci_device_bulk_enter,
  .start = ohci_device_bulk_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

/*---------------------------------------------------------------------------*
 * ohci control support
 *---------------------------------------------------------------------------*/
static void
ohci_device_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ohci_device_ctrl_close(struct usbd_xfer *xfer)
{
	ohci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ohci_device_ctrl_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ohci_device_ctrl_start(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;

	/* setup TD's and QH */
	ohci_setup_standard_chain(xfer, &sc->sc_ctrl_p_last);

	/**/
	ohci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ohci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods ohci_device_ctrl_methods = 
{
  .open = ohci_device_ctrl_open,
  .close = ohci_device_ctrl_close,
  .enter = ohci_device_ctrl_enter,
  .start = ohci_device_ctrl_start,
  .copy_in = usbd_std_ctrl_copy_in,
  .copy_out = usbd_std_ctrl_copy_out,
};

/*---------------------------------------------------------------------------*
 * ohci interrupt support
 *---------------------------------------------------------------------------*/
static void
ohci_device_intr_open(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;
	u_int16_t best;
	u_int16_t bit;
	u_int16_t x;

	best = 0;
	bit = OHCI_NO_EDS/2;
	while(bit)
	{
		if(xfer->interval >= bit)
		{
			x = bit;
			best = bit;
			while(x & bit)
			{
				if(sc->sc_intr_stat[x] < 
				   sc->sc_intr_stat[best])
				{
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

	DPRINTFN(2, ("best=%d interval=%d\n",
		     best, xfer->interval));
	return;
}

static void
ohci_device_intr_close(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;

	sc->sc_intr_stat[xfer->qh_pos]--;

	ohci_device_done(xfer,USBD_CANCELLED);
	return;
}

static void
ohci_device_intr_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ohci_device_intr_start(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;

	DPRINTFN(3,("xfer=%p len=%d\n",
		    xfer, xfer->length));

	/* setup TD's and QH */
	ohci_setup_standard_chain(xfer, &sc->sc_intr_p_last[xfer->qh_pos]);

	/**/
	ohci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ohci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods ohci_device_intr_methods = 
{
  .open = ohci_device_intr_open,
  .close = ohci_device_intr_close,
  .enter = ohci_device_intr_enter,
  .start = ohci_device_intr_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

/*---------------------------------------------------------------------------*
 * ohci isochronous support
 *---------------------------------------------------------------------------*/
static void
ohci_device_isoc_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ohci_device_isoc_close(struct usbd_xfer *xfer)
{
	/**/
	ohci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ohci_device_isoc_enter(struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	ohci_softc_t *sc = xfer->usb_sc;
	struct ohci_hw_softc *hw_ptr = sc->sc_hw_ptr;
	u_int32_t buf_offset;
	u_int32_t nframes;
	u_int32_t bp0;
	u_int32_t end_phy;
	uint32_t ed_flags;
	u_int16_t *plen;
	uint16_t itd_offset[OHCI_ITD_NOFFSET];
	u_int8_t ncur;
	u_int8_t allzero = 1;
	ohci_itd_t *td;
	ohci_itd_t *td_last = NULL;
	ohci_ed_t *ed;

	DPRINTFN(5,("xfer=%p next=%d nframes=%d\n",
		    xfer, xfer->pipe->isoc_next, xfer->nframes));

	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_PREREAD);
	nframes = le32toh(hw_ptr->hcca.hcca_frame_number);
	usbd_page_sync(&(sc->sc_hw_page), BUS_DMASYNC_POSTREAD);

	if((((nframes - xfer->pipe->isoc_next) & ((1<<16)-1)) < xfer->nframes) ||
	   (((xfer->pipe->isoc_next - nframes) & ((1<<16)-1)) >= 256))
	{
		/* not in use yet, schedule it a few frames ahead */
		/* data underflow */
		xfer->pipe->isoc_next = (nframes + 5) & ((1<<16)-1);
		DPRINTFN(2,("start next=%d\n", xfer->pipe->isoc_next));
	}

	nframes = xfer->nframes;

	if(nframes == 0)
	{
		/* transfer transferred */
		ohci_device_done(xfer, USBD_NORMAL_COMPLETION);

		/* call callback recursively */
		__usbd_callback(xfer);

		return;
	}

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	plen = xfer->frlengths;

	td = xfer->td_start;

	xfer->td_transfer_first = td;

	ncur = 0;

	bp0 = OHCI_PAGE(buf_res.physaddr);

	end_phy = 0;

	while(nframes--)
	{
		if(td == NULL)
		{
			panic("%s:%d: out of TD's\n",
			      __FUNCTION__, __LINE__);
		}

		itd_offset[ncur] = htole16(OHCI_ITD_MK_OFFS
					   (buf_res.physaddr - bp0));
		if (*plen) {
		    allzero = 0;
		    buf_offset += (*plen) -1;
		    usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		    end_phy = buf_res.physaddr;
		    buf_offset += 1;
		    usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);
		}

		plen++;
		ncur++;

		if((ncur == OHCI_ITD_NOFFSET) ||
		   (OHCI_PAGE(buf_res.physaddr) != bp0) ||
		   (nframes == 0))
		{
			usbd_page_sync(td->page, BUS_DMASYNC_PREWRITE);

			/* fill current ITD */
			td->itd_flags = htole32(
				OHCI_ITD_NOCC |
				OHCI_ITD_SET_SF(xfer->pipe->isoc_next) |
				OHCI_ITD_NOINTR |
				OHCI_ITD_SET_FC(ncur));

			if (allzero) {
			    td->itd_bp0 = 0;
			    td->itd_be = ~0;
			} else {
			    td->itd_bp0 = htole32(bp0);
			    td->itd_be = htole32(end_phy);
			}
			td->frames = ncur;

			xfer->pipe->isoc_next += ncur;
			bp0 = OHCI_PAGE(buf_res.physaddr);
			while (ncur--) {
			    td->itd_offset[ncur] = itd_offset[ncur];
			}
			ncur = 0;
			allzero = 1;
			td_last = td;
			td = td->obj_next;

			if (td) {
			    /* link the last TD with the next one */
			    td_last->itd_next = td->itd_self;
			}

			usbd_page_sync(td_last->page, BUS_DMASYNC_POSTWRITE);
		}
	}

	usbd_page_sync(td_last->page, BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

	/* update the last TD */
	td_last->itd_flags &= htole32(~OHCI_ITD_NOINTR);
	td_last->itd_flags |= htole32(OHCI_ITD_SET_DI(0));
	td_last->itd_next = 0;

	usbd_page_sync(td_last->page, BUS_DMASYNC_POSTWRITE|BUS_DMASYNC_POSTREAD);

	xfer->td_transfer_last = td_last;

#ifdef USB_DEBUG
	if(ohcidebug > 8)
	{
		DPRINTF(("data before transfer:\n"));
		ohci_dump_itds(xfer->td_start);
	}
#endif
	ed = xfer->qh_start;

	usbd_page_sync(ed->page, BUS_DMASYNC_PREWRITE);

	if(UE_GET_DIR(xfer->endpoint) == UE_DIR_IN)
		ed_flags = (OHCI_ED_DIR_IN|OHCI_ED_FORMAT_ISO);
	else
		ed_flags = (OHCI_ED_DIR_OUT|OHCI_ED_FORMAT_ISO);

	ed_flags |= (OHCI_ED_SET_FA(xfer->address)|
		     OHCI_ED_SET_EN(UE_GET_ADDR(xfer->endpoint))|
		     OHCI_ED_SET_MAXP(xfer->max_packet_size));

	if(xfer->udev->speed == USB_SPEED_LOW)
	{
		ed_flags |= OHCI_ED_SPEED;
	}

	ed->ed_flags = htole32(ed_flags);

	usbd_page_sync(ed->page, BUS_DMASYNC_POSTWRITE);

	td = xfer->td_transfer_first;

	OHCI_APPEND_QH(ed, td->itd_self, sc->sc_isoc_p_last);

	/**/
	ohci_add_interrupt_info(sc, xfer);

	if(!xfer->timeout)
	{
		/* in case the frame start number is wrong */
		xfer->timeout = 1000/4;
	}

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ohci_timeout, xfer);
	}

	/* enqueue transfer 
	 * (so that it can be aborted through pipe abort)
	 */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ohci_device_isoc_start(struct usbd_xfer *xfer)
{
	/* already started, nothing to do */
	return;
}

struct usbd_pipe_methods ohci_device_isoc_methods = 
{
  .open = ohci_device_isoc_open,
  .close = ohci_device_isoc_close,
  .enter = ohci_device_isoc_enter,
  .start = ohci_device_isoc_start,
  .copy_in = usbd_std_isoc_copy_in,
  .copy_out = usbd_std_isoc_copy_out,
};

/*---------------------------------------------------------------------------*
 * ohci root control support
 *---------------------------------------------------------------------------*
 * simulate a hardware hub by handling
 * all the necessary requests
 *---------------------------------------------------------------------------*/

static void
ohci_root_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ohci_root_ctrl_close(struct usbd_xfer *xfer)
{
	ohci_device_done(xfer,USBD_CANCELLED);
	return;
}

/* data structures and routines
 * to emulate the root hub:
 */
static const
usb_device_descriptor_t ohci_devd =
{
	sizeof(usb_device_descriptor_t),
	UDESC_DEVICE,		/* type */
	{0x00, 0x01},		/* USB version */
	UDCLASS_HUB,		/* class */
	UDSUBCLASS_HUB,		/* subclass */
	UDPROTO_FSHUB,		/* protocol */
	64,			/* max packet */
	{0},{0},{0x00,0x01},	/* device id */
	1,2,0,			/* string indicies */
	1			/* # of configurations */
};

static const
usb_config_descriptor_t ohci_confd =
{
	sizeof(usb_config_descriptor_t),
	UDESC_CONFIG,
	{USB_CONFIG_DESCRIPTOR_SIZE +
	 USB_INTERFACE_DESCRIPTOR_SIZE +
	 USB_ENDPOINT_DESCRIPTOR_SIZE},
	1,
	1,
	0,
	UC_SELF_POWERED,
	0			/* max power */
};

static const
usb_interface_descriptor_t ohci_ifcd =
{
	sizeof(usb_interface_descriptor_t),
	UDESC_INTERFACE,
	0,
	0,
	1,
	UICLASS_HUB,
	UISUBCLASS_HUB,
	UIPROTO_FSHUB,
	0
};

static const
usb_endpoint_descriptor_t ohci_endpd =
{
	sizeof(usb_endpoint_descriptor_t),
	UDESC_ENDPOINT,
	UE_DIR_IN | OHCI_INTR_ENDPT,
	UE_INTERRUPT,
	{8, 0},			/* max packet */
	255
};

static const
usb_hub_descriptor_t ohci_hubd =
{
	0, /* dynamic length */
	UDESC_HUB,
	0,
	{0,0},
	0,
	0,
	{0},
};

static void
ohci_root_ctrl_enter(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;
	u_int32_t port;
	u_int32_t v;
	u_int16_t len;
	u_int16_t value;
	u_int16_t index;
	u_int16_t l;
	u_int16_t totlen = 0;
	u_int16_t i;
	union {
	  usb_status_t stat;
	  usb_port_status_t ps;
	  usb_device_request_t req;
	  usb_device_descriptor_t devd;
	  usb_hub_descriptor_t hubd;
	  u_int8_t str_temp[128];
	  u_int8_t byte_temp;
	} u;
	usbd_status err;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	if (xfer->length < sizeof(u.req)) {
	    err = USBD_INVAL;
	    goto done;
	}

	/* set default actual length */
	xfer->actlen = sizeof(u.req);

	/* copy out "request" */
	usbd_copy_out(&(xfer->buf_data), 0, &u.req, sizeof(u.req));

	len = (xfer->length - sizeof(u.req));

	if (len != UGETW(u.req.wLength)) {
	    err = USBD_INVAL;
	    goto done;
	}

	value = UGETW(u.req.wValue);
	index = UGETW(u.req.wIndex);

	DPRINTFN(2,("type=0x%02x request=0x%02x wLen=0x%04x "
		    "wValue=0x%04x wIndex=0x%04x\n",
		    u.req.bmRequestType, u.req.bRequest,
		    len, value, index));

#define C(x,y) ((x) | ((y) << 8))
	switch(C(u.req.bRequest, u.req.bmRequestType)) {
	case C(UR_CLEAR_FEATURE, UT_WRITE_DEVICE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
		/*
		 * DEVICE_REMOTE_WAKEUP and ENDPOINT_HALT are no-ops
		 * for the integrated root hub.
		 */
		break;
	case C(UR_GET_CONFIG, UT_READ_DEVICE):
		if(len > 0)
		{
		    u.byte_temp = sc->sc_conf;
		    totlen = 1;
		    usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				 &u, totlen);
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		switch(value >> 8) {
		case UDESC_DEVICE:
			if((value & 0xff) != 0)
			{
				err = USBD_IOERROR;
				goto done;
			}
			totlen = min(len, sizeof(u.devd));

			u.devd = ohci_devd;
#if 0
			USETW(u.devd.idVendor,
			      sc->sc_id_vendor);
#endif
			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &u, totlen);
			break;

		case UDESC_CONFIG:
			if((value & 0xff) != 0)
			{
				err = USBD_IOERROR;
				goto done;
			}
			totlen = l = min(len, sizeof(ohci_confd));
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &ohci_confd, l);

			l = min(len, sizeof(ohci_ifcd));
			totlen += l;
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req) +
				     sizeof(ohci_confd), &ohci_ifcd, l);

			l = min(len, sizeof(ohci_endpd));
			totlen += l;
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req) +
				     sizeof(ohci_confd) + sizeof(ohci_ifcd),
				     &ohci_endpd, l);
			break;

		case UDESC_STRING:
			if(len == 0)
			{
				break;
			}
			switch (value & 0xff) {
			case 0: /* Language table */
			    totlen = usbd_make_str_desc
			      (u.str_temp, sizeof(u.str_temp), 
			       "\001");
			    break;

			case 1: /* Vendor */
			    totlen = usbd_make_str_desc
			      (u.str_temp, sizeof(u.str_temp), 
			       sc->sc_vendor);
			    break;

			case 2: /* Product */
			    totlen = usbd_make_str_desc
			      (u.str_temp, sizeof(u.str_temp), 
			       "OHCI root hub");
			    break;

			default:
			    totlen = usbd_make_str_desc
			      (u.str_temp, sizeof(u.str_temp),
			       "");
			    break;
			}
			if (totlen > len) {
			    totlen = len;
			}
			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &u, totlen);
			break;

		default:
			err = USBD_IOERROR;
			goto done;
		}
		break;
	case C(UR_GET_INTERFACE, UT_READ_INTERFACE):
		if(len > 0)
		{
		    u.byte_temp = 0;
		    totlen = 1;
		    usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				 &u, totlen);
		}
		break;
	case C(UR_GET_STATUS, UT_READ_DEVICE):
		if(len > 1)
		{
		    USETW(u.stat.wStatus,UDS_SELF_POWERED);
		    totlen = 2;
		    usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				 &u, totlen);
		}
		break;
	case C(UR_GET_STATUS, UT_READ_INTERFACE):
	case C(UR_GET_STATUS, UT_READ_ENDPOINT):
		if(len > 1)
		{
		    USETW(u.stat.wStatus, 0);
		    totlen = 2;
		    usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				 &u, totlen);
		}
		break;
	case C(UR_SET_ADDRESS, UT_WRITE_DEVICE):
		if(value >= USB_MAX_DEVICES)
		{
			err = USBD_IOERROR;
			goto done;
		}
		sc->sc_addr = value;
		break;
	case C(UR_SET_CONFIG, UT_WRITE_DEVICE):
		if((value != 0) && (value != 1))
		{
			err = USBD_IOERROR;
			goto done;
		}
		sc->sc_conf = value;
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_DEVICE):
	case C(UR_SET_FEATURE, UT_WRITE_INTERFACE):
	case C(UR_SET_FEATURE, UT_WRITE_ENDPOINT):
		err = USBD_IOERROR;
		goto done;
	case C(UR_SET_INTERFACE, UT_WRITE_INTERFACE):
		break;
	case C(UR_SYNCH_FRAME, UT_WRITE_ENDPOINT):
		break;
	/* Hub requests */
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_CLEAR_FEATURE, UT_WRITE_CLASS_OTHER):
		DPRINTFN(8, ("UR_CLEAR_PORT_FEATURE "
			     "port=%d feature=%d\n",
			     index, value));
		if((index < 1) ||
		   (index > sc->sc_noport))
		{
			err = USBD_IOERROR;
			goto done;
		}
		port = OHCI_RH_PORT_STATUS(index);
		switch(value) {
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
			err = USBD_IOERROR;
			goto done;
		}
		switch(value) {
		case UHF_C_PORT_CONNECTION:
		case UHF_C_PORT_ENABLE:
		case UHF_C_PORT_SUSPEND:
		case UHF_C_PORT_OVER_CURRENT:
		case UHF_C_PORT_RESET:
			/* enable RHSC interrupt if condition is cleared. */
			if((OREAD4(sc, port) >> 16) == 0)
			{
				mtx_lock(&sc->sc_bus.mtx);
				ohci_rhsc_enable(sc);
			}
			break;
		default:
			break;
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_CLASS_DEVICE):
		if((value & 0xff) != 0)
		{
			err = USBD_IOERROR;
			goto done;
		}

		v = OREAD4(sc, OHCI_RH_DESCRIPTOR_A);

		u.hubd = ohci_hubd;
		u.hubd.bNbrPorts = sc->sc_noport;
		USETW(u.hubd.wHubCharacteristics,
		      (v & OHCI_NPS ? UHD_PWR_NO_SWITCH :
		       v & OHCI_PSM ? UHD_PWR_GANGED : UHD_PWR_INDIVIDUAL)
		      /* XXX overcurrent */
		      );
		u.hubd.bPwrOn2PwrGood = OHCI_GET_POTPGT(v);
		v = OREAD4(sc, OHCI_RH_DESCRIPTOR_B);

		for(l = 0; l < sc->sc_noport; l++)
		{
		    if (v & 1) {
		        u.hubd.DeviceRemovable[l/8] |= (1 << (l % 8));
		    }
		    v >>= 1;
		}
		u.hubd.bDescLength = (USB_HUB_DESCRIPTOR_SIZE-1) + ((sc->sc_noport + 7)/8);
		totlen = min(len, u.hubd.bDescLength);
		usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
			     &u, totlen);
		break;

	case C(UR_GET_STATUS, UT_READ_CLASS_DEVICE):
		if(len < 4)
		{
		    err = USBD_IOERROR;
		    goto done;
		}
		usbd_bzero(&(xfer->buf_data), sizeof(u.req), len);
		totlen = len;
		break;
	case C(UR_GET_STATUS, UT_READ_CLASS_OTHER):
		DPRINTFN(8,("get port status i=%d\n",
			    index));
		if((index < 1) ||
		   (index > sc->sc_noport))
		{
		    err = USBD_IOERROR;
		    goto done;
		}
		if(len < 4)
		{
		    err = USBD_IOERROR;
		    goto done;
		}
		v = OREAD4(sc, OHCI_RH_PORT_STATUS(index));
		DPRINTFN(8,("port status=0x%04x\n", v));
		USETW(u.ps.wPortStatus, v);
		USETW(u.ps.wPortChange, v >> 16);
		totlen = min(len, sizeof(u.ps));
		usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
			     &u, totlen);
		break;
	case C(UR_SET_DESCRIPTOR, UT_WRITE_CLASS_DEVICE):
		err = USBD_IOERROR;
		goto done;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_DEVICE):
		break;
	case C(UR_SET_FEATURE, UT_WRITE_CLASS_OTHER):
		if((index < 1) || 
		   (index > sc->sc_noport))
		{
		    err = USBD_IOERROR;
		    goto done;
		}
		port = OHCI_RH_PORT_STATUS(index);
		switch(value) {
		case UHF_PORT_ENABLE:
			OWRITE4(sc, port, UPS_PORT_ENABLED);
			break;
		case UHF_PORT_SUSPEND:
			OWRITE4(sc, port, UPS_SUSPEND);
			break;
		case UHF_PORT_RESET:
			DPRINTFN(5,("reset port %d\n", index));
			OWRITE4(sc, port, UPS_RESET);
			for(i = 0; i < 5; i++)
			{
				DELAY(1000*USB_PORT_ROOT_RESET_DELAY);

				if((OREAD4(sc, port) & UPS_RESET) == 0)
				{
					break;
				}
			}
			DPRINTFN(8,("ohci port %d reset, status = 0x%04x\n",
				    index, OREAD4(sc, port)));
			break;
		case UHF_PORT_POWER:
			DPRINTFN(2,("set port power %d\n", index));
			OWRITE4(sc, port, UPS_PORT_POWER);
			break;
		default:
			err = USBD_IOERROR;
			goto done;
		}
		break;
	default:
		err = USBD_IOERROR;
		goto done;
	}

	xfer->actlen = totlen + sizeof(u.req);
	err = USBD_NORMAL_COMPLETION;

 done:
	/* transfer transferred */
	ohci_device_done(xfer,err);

	/* call callback recursively */
	__usbd_callback(xfer);

	return;
}

static void
ohci_root_ctrl_start(struct usbd_xfer *xfer)
{
	/* not used */
	return;
}

struct usbd_pipe_methods ohci_root_ctrl_methods = 
{
  .open = ohci_root_ctrl_open,
  .close = ohci_root_ctrl_close,
  .enter = ohci_root_ctrl_enter,
  .start = ohci_root_ctrl_start,
  .copy_in = usbd_std_ctrl_copy_in,
  .copy_out = usbd_std_ctrl_copy_out,
};

/*---------------------------------------------------------------------------*
 * ohci root interrupt support
 *---------------------------------------------------------------------------*/
static void
ohci_root_intr_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ohci_root_intr_close(struct usbd_xfer *xfer)
{
	ohci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ohci_root_intr_enter(struct usbd_xfer *xfer)
{
	DPRINTFN(3, ("xfer=%p len=%d\n",
		     xfer, xfer->length));

	/* enqueue transfer 
	 * (so that it can be aborted through pipe abort)
	 */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ohci_root_intr_start(struct usbd_xfer *xfer)
{
	ohci_softc_t *sc = xfer->usb_sc;

	/* only one transfer at a time 
	 * (sc_intrxfer is cleared by ohci_root_intr_done())
	 */
	sc->sc_intrxfer = xfer;
	return;
}

struct usbd_pipe_methods ohci_root_intr_methods = 
{
  .open = ohci_root_intr_open,
  .close = ohci_root_intr_close,
  .enter = ohci_root_intr_enter,
  .start = ohci_root_intr_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

#define ADD_BYTES(ptr,size) ((void *)(((u_int8_t *)(ptr)) + (size)))

static usbd_status
ohci_xfer_setup(struct usbd_device *udev,
		u_int8_t iface_index,
		struct usbd_xfer **pxfer,
		const struct usbd_config *setup_start,
		const struct usbd_config *setup_end)
{
	enum { max_frames = 128 };
	struct usbd_page_info page_info;
	struct usbd_xfer dummy;
	ohci_softc_t *sc = OHCI_BUS2SC(udev->bus);
	const struct usbd_config *setup;
	struct usbd_memory_info *info;
	struct usbd_page *page_ptr;
	struct usbd_xfer *xfer;
	u_int32_t size[2];
	u_int32_t total_size[2];
	u_int32_t ntd;
	u_int32_t nitd;
	u_int32_t nqh;
	u_int32_t n;
	void *buf;
	void *last_obj;
	usbd_status error = 0;

	buf = NULL;
	page_ptr = NULL;
	total_size[0] = 0;
	total_size[1] = 0;

 repeat:
	size[0] = 0;
	size[1] = 0;

	/* align data to 8 byte boundary */
	size[0] += ((-size[0]) & (USB_HOST_ALIGN-1));

	if (buf)
	{
	    info = ADD_BYTES(buf,size[0]);

	    info->memory_base = buf;
	    info->memory_size = total_size[0];

	    info->page_base = page_ptr;
	    info->page_size = total_size[1];

	    info->usb_mtx = &sc->sc_bus.mtx;
	}
	else
	{
	    info = NULL;
	}

	size[0] += sizeof(info[0]);

	for(setup = setup_start;
	    setup < setup_end;
	    setup++)
	{
	  ntd = 0;
	  nitd = 0;
	  nqh = 0;

	  /* align data to 8 byte boundary */
	  size[0] += ((-size[0]) & (USB_HOST_ALIGN-1));

	  if(buf)
	  {
		xfer = ADD_BYTES(buf,size[0]);
		*pxfer++ = xfer;
	  }
	  else
	  {
		/* need dummy xfer to 
		 * calculate ntd and nqh !
		 */
		xfer = &dummy;
		bzero(&dummy, sizeof(dummy));
	  }

	  /*
	   * setup xfer
	   */
	  xfer->usb_sc = sc;
	  xfer->usb_mtx = &sc->sc_bus.mtx;
	  xfer->usb_root = info;
	  xfer->address = udev->address;
	  xfer->pipe = usbd_get_pipe(udev, iface_index, setup);

	  if(!xfer->pipe)
	  {
		error = USBD_NO_PIPE;
		DPRINTF(("no pipe for endpoint %d\n",
			 setup->endpoint));
		goto done;
	  }
	  else
	  {
		usbd_std_transfer_setup(xfer, setup, 0x500, 0x500, 1);

		/*
		 * calculate ntd and nqh
		 */
		if((xfer->pipe->methods == &ohci_device_ctrl_methods) ||
		   (xfer->pipe->methods == &ohci_device_bulk_methods) ||
		   (xfer->pipe->methods == &ohci_device_intr_methods))
		{
			nitd = 0;
			ntd = (1+ /* SETUP */ 1+ /* STATUS */
			       1+  /* SHORTPKT */ 1 /* EXTRA */) +
			  (xfer->length / (OHCI_PAGE_SIZE/2)) /* DATA */;
		}
		else if(xfer->pipe->methods == &ohci_device_isoc_methods)
		{
			if(xfer->nframes == 0)
			{
			    error = USBD_ZERO_FRAMES_IN_ISOC_MODE;
			    DPRINTF(("frames == 0 in isochronous mode; "
				     "endpoint 0x%02x\n", setup->endpoint));
			    goto done;
			}

			if(xfer->nframes >= max_frames)
			{
			    error = USBD_INVAL;
			    DPRINTF(("isochronous frame-limit "
				     "exceeded by 0x%x frames; endpoint 0x%02x\n",
				     setup->frames - max_frames,
				     setup->endpoint));
			    goto done;
			}

			nitd = ((xfer->length / OHCI_PAGE_SIZE) +
				((xfer->nframes + OHCI_ITD_NOFFSET - 1) / OHCI_ITD_NOFFSET) + 
				1 /* EXTRA */);
			ntd = 0;
		}
		else
		{
			nitd = 0;
			ntd = 0;
		}

		if((xfer->pipe->methods == &ohci_device_ctrl_methods) ||
		   (xfer->pipe->methods == &ohci_device_bulk_methods) ||
		   (xfer->pipe->methods == &ohci_device_intr_methods) ||
		   (xfer->pipe->methods == &ohci_device_isoc_methods))
		{
			nqh = 1;
		}
		else
		{
			nqh = 0;
		}
	  }

	  size[0] += sizeof(xfer[0]);

	  if(xfer->nframes)
	  {
		/* align data to 8 byte boundary */
		size[0] += ((-size[0]) & (USB_HOST_ALIGN-1));

		xfer->frlengths = ADD_BYTES(buf,size[0]);

		size[0] += (xfer->nframes * sizeof(xfer->frlengths[0]));

		xfer->frlengths_old = ADD_BYTES(buf,size[0]);

		size[0] += (xfer->nframes * sizeof(xfer->frlengths[0]));
	  }

	  if (!(xfer->flags & USBD_USE_DMA)) {

	      /* align data to 8 byte boundary */
	      size[0] += ((-size[0]) & (USB_HOST_ALIGN-1));

	      xfer->buffer = ADD_BYTES(buf,size[0]);

	      size[0] += xfer->length;
	  }

	  /*****/

	  /* align data to 8 byte boundary */
	  size[1] += ((-size[1]) & (USB_HOST_ALIGN-1));

	  usbd_page_set_start(&(xfer->buf_data), page_ptr, size[1]);

	  size[1] += xfer->length;

	  usbd_page_set_end(&(xfer->buf_data), page_ptr, size[1]);

	  /* align data */
	  size[1] += ((-size[1]) & (OHCI_ITD_ALIGN-1));

	  last_obj = NULL;

	  for(n = 0;
	      n < ntd;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					 sizeof(ohci_td_t));
	    if(buf)
	    {
		register ohci_td_t *td;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_sync(page_info.page, BUS_DMASYNC_PREWRITE);

		td = page_info.buffer;

		/* init TD */
		td->td_self = htole32(page_info.physaddr);
		td->obj_next = last_obj;
		td->page = page_info.page;

		last_obj = td;

		usbd_page_sync(page_info.page, BUS_DMASYNC_POSTWRITE);
	    }
	    size[1] += sizeof(ohci_td_t);
	  }

	  for(n = 0;
	      n < nitd;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					 sizeof(ohci_itd_t));
	    if(buf)
	    {
		register ohci_itd_t *itd;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_sync(page_info.page, BUS_DMASYNC_PREWRITE);

		itd = page_info.buffer;

		/* init TD */
		itd->itd_self = htole32(page_info.physaddr);
		itd->obj_next = last_obj;
		itd->page = page_info.page;

		last_obj = itd;

		usbd_page_sync(page_info.page, BUS_DMASYNC_POSTWRITE);
	    }
	    size[1] += sizeof(ohci_itd_t);
	  }

	  xfer->td_start = last_obj;

	  /* align data */
	  size[1] += ((-size[1]) & (OHCI_ED_ALIGN-1));

	  last_obj = NULL;

	  for(n = 0;
	      n < nqh;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1],
					 sizeof(ohci_ed_t));
	    if(buf)
	    {
		register ohci_ed_t *ed;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_sync(page_info.page, BUS_DMASYNC_PREWRITE);

		ed = page_info.buffer;

		/* init QH */
		ed->ed_self = htole32(page_info.physaddr);
		ed->obj_next = last_obj;
		ed->page = page_info.page;

		last_obj = ed;

		usbd_page_sync(page_info.page, BUS_DMASYNC_POSTWRITE);
	    }
	    size[1] += sizeof(ohci_ed_t);
	  }
	  xfer->qh_start = last_obj;
	}

	if (buf || error) {
	    goto done;
	}

	/* compute number of USB pages required */
	total_size[1] = (size[1] + USB_PAGE_SIZE - 1) / USB_PAGE_SIZE;

	/* align data to 8 byte boundary */
	size[0] += ((-size[0]) & (USB_HOST_ALIGN-1));

	/* store offset temporarily */
	n = size[0];

	size[0] += (sizeof(*page_ptr) * total_size[1]);

	/* store total buffer size */
	total_size[0] = size[0];

	/* allocate zeroed memory */
	buf = malloc(total_size[0], M_USB, M_WAITOK|M_ZERO);

	if (buf == NULL) {
	    error = USBD_NOMEM;
	    DPRINTF(("cannot allocate memory block for "
		     "configuration (%d bytes)\n", 
		     total_size[0]));
	    goto done;
	}

	page_ptr = ADD_BYTES(buf,n);

	if (usbd_page_alloc(sc->sc_bus.dma_tag,
			    page_ptr, total_size[1])) {
	    free(buf, M_USB);
	    error = USBD_NOMEM;
	    DPRINTF(("cannot allocate memory block for "
		     "configuration (%d USB pages)\n", 
		     total_size[1]));
	    goto done;
	}

	goto repeat;
 done:
	return error;
}

static void
ohci_pipe_init(struct usbd_device *udev, usb_endpoint_descriptor_t *edesc, 
               struct usbd_pipe *pipe)
{
	ohci_softc_t *sc = OHCI_BUS2SC(udev->bus);

	DPRINTFN(1, ("pipe=%p, addr=%d, endpt=%d (%d)\n",
		     pipe, udev->address,
		     edesc->bEndpointAddress, sc->sc_addr));

	if(udev->address == sc->sc_addr)
	{
		switch (edesc->bEndpointAddress)
		{
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &ohci_root_ctrl_methods;
			break;
		case UE_DIR_IN | OHCI_INTR_ENDPT:
			pipe->methods = &ohci_root_intr_methods;
			break;
		default:
			panic("invalid endpoint address: 0x%02x",
			      edesc->bEndpointAddress);
			break;
		}
	}
	else
        {
		switch (edesc->bmAttributes & UE_XFERTYPE)
		{
		case UE_CONTROL:
			pipe->methods = &ohci_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &ohci_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			pipe->methods = &ohci_device_isoc_methods;
			break;
		case UE_BULK:
			pipe->methods = &ohci_device_bulk_methods;
			break;
		}
	}
	return;
}

struct usbd_bus_methods ohci_bus_methods = 
{
	.pipe_init  = ohci_pipe_init,
	.xfer_setup = ohci_xfer_setup,
	.do_poll    = ohci_do_poll,
};
