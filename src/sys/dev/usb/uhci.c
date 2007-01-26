/*	$NetBSD: uhci.c,v 1.170 2003/02/19 01:35:04 augustss Exp $	*/

/*	Also already incorporated from NetBSD:
 *	$NetBSD: uhci.c,v 1.172 2003/02/23 04:19:26 simonb Exp $
 *	$NetBSD: uhci.c,v 1.173 2003/05/13 04:41:59 gson Exp $
 *	$NetBSD: uhci.c,v 1.175 2003/09/12 16:18:08 mycroft Exp $
 *	$NetBSD: uhci.c,v 1.176 2003/11/04 19:11:21 mycroft Exp $
 *	$NetBSD: uhci.c,v 1.177 2003/12/29 08:17:10 toshii Exp $
 *	$NetBSD: uhci.c,v 1.178 2004/03/02 16:32:05 martin Exp $
 *	$NetBSD: uhci.c,v 1.180 2004/07/17 20:12:03 mycroft Exp $
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/uhci.c,v 1.171 2006/09/07 00:06:41 imp Exp $");


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
 * USB Universal Host Controller driver.
 * Handles e.g. PIIX3 and PIIX4.
 *
 * UHCI spec: http://developer.intel.com/design/USB/UHCI11D.htm
 * USB spec: http://www.usb.org/developers/docs/usbspec.zip
 * PIIXn spec: ftp://download.intel.com/design/intarch/datashts/29055002.pdf
 *             ftp://download.intel.com/design/intarch/datashts/29056201.pdf
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
#include <dev/usb/uhci.h>

#define MS_TO_TICKS(ms) (((ms) * hz) / 1000)
#define UHCI_BUS2SC(bus) ((uhci_softc_t *)(((u_int8_t *)(bus)) - \
   POINTER_TO_UNSIGNED(&(((uhci_softc_t *)0)->sc_bus))))

#ifdef USB_DEBUG
#undef DPRINTF
#undef DPRINTFN
#define DPRINTF(x)	{ if (uhcidebug) { printf("%s: ", __FUNCTION__); printf x ; } }
#define DPRINTFN(n,x)	{ if (uhcidebug > (n)) { printf("%s: ", __FUNCTION__); printf x ; } }
int uhcidebug = 0;
int uhcinoloop = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uhci, CTLFLAG_RW, 0, "USB uhci");
SYSCTL_INT(_hw_usb_uhci, OID_AUTO, debug, CTLFLAG_RW,
	   &uhcidebug, 0, "uhci debug level");
SYSCTL_INT(_hw_usb_uhci, OID_AUTO, loop, CTLFLAG_RW,
	   &uhcinoloop, 0, "uhci noloop");
static void
uhci_dumpregs(uhci_softc_t *sc);
static void
uhci_dump_tds(uhci_td_t *td);
#endif

#define UBARR(sc) bus_space_barrier((sc)->sc_io_tag, (sc)->sc_io_hdl, 0, (sc)->sc_io_size, \
			BUS_SPACE_BARRIER_READ|BUS_SPACE_BARRIER_WRITE)
#define UWRITE1(sc, r, x) \
 do { UBARR(sc); bus_space_write_1((sc)->sc_io_tag, (sc)->sc_io_hdl, (r), (x)); \
 } while (/*CONSTCOND*/0)
#define UWRITE2(sc, r, x) \
 do { UBARR(sc); bus_space_write_2((sc)->sc_io_tag, (sc)->sc_io_hdl, (r), (x)); \
 } while (/*CONSTCOND*/0)
#define UWRITE4(sc, r, x) \
 do { UBARR(sc); bus_space_write_4((sc)->sc_io_tag, (sc)->sc_io_hdl, (r), (x)); \
 } while (/*CONSTCOND*/0)
#define UREAD1(sc, r) (UBARR(sc), bus_space_read_1((sc)->sc_io_tag, (sc)->sc_io_hdl, (r)))
#define UREAD2(sc, r) (UBARR(sc), bus_space_read_2((sc)->sc_io_tag, (sc)->sc_io_hdl, (r)))
#define UREAD4(sc, r) (UBARR(sc), bus_space_read_4((sc)->sc_io_tag, (sc)->sc_io_hdl, (r)))

#define UHCICMD(sc, cmd) UWRITE2(sc, UHCI_CMD, cmd)
#define UHCISTS(sc) UREAD2(sc, UHCI_STS)

#define UHCI_RESET_TIMEOUT 100	/* ms, reset timeout */

#define UHCI_INTR_ENDPT 1

#define ADD_BYTES(ptr,size) ((void *)(((u_int8_t *)(ptr)) + (size)))

#define	SC_HW_PHYSADDR(sc,what) \
  ((sc)->sc_hw_page.physaddr + \
   POINTER_TO_UNSIGNED(&(((struct uhci_hw_softc *)0)->what)))

extern struct usbd_bus_methods uhci_bus_methods;
extern struct usbd_pipe_methods uhci_device_bulk_methods;
extern struct usbd_pipe_methods uhci_device_ctrl_methods;
extern struct usbd_pipe_methods uhci_device_intr_methods;
extern struct usbd_pipe_methods uhci_device_isoc_methods;
extern struct usbd_pipe_methods uhci_root_ctrl_methods;
extern struct usbd_pipe_methods uhci_root_intr_methods;

void
uhci_reset(uhci_softc_t *sc)
{
	u_int n;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	DPRINTF(("resetting the HC\n"));

	/* disable interrupts */

	UWRITE2(sc, UHCI_INTR, 0);

	/* global reset */

	UHCICMD(sc, UHCI_CMD_GRESET);

	/* wait */

	DELAY(1000*USB_BUS_RESET_DELAY);

	/* terminate all transfers */

	UHCICMD(sc, UHCI_CMD_HCRESET);

	/* the reset bit goes low when the controller is done */

	n = UHCI_RESET_TIMEOUT;
	while(n--)
	{
		/* wait one millisecond */

		DELAY(1000);

		if(!(UREAD2(sc, UHCI_CMD) & UHCI_CMD_HCRESET))
		{
			goto done_1;
		}
	}

	device_printf(sc->sc_bus.bdev,
		      "controller did not reset\n");

 done_1:

	n = 10;
	while(n--)
	{
		/* wait one millisecond */

		DELAY(1000);

		/* check if HC is stopped */
		if(UREAD2(sc, UHCI_STS) & UHCI_STS_HCH)
		{
			goto done_2;
		}
	}

	device_printf(sc->sc_bus.bdev,
		      "controller did not stop\n");

 done_2:

	/* reload the configuration */

	UWRITE4(sc, UHCI_FLBASEADDR, SC_HW_PHYSADDR(sc,pframes));
	UWRITE2(sc, UHCI_FRNUM, sc->sc_saved_frnum);
	UWRITE1(sc, UHCI_SOF, sc->sc_saved_sof);
	return;
}

static void
uhci_start(uhci_softc_t *sc)
{
	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	DPRINTFN(1,("enabling\n"));

	/* enable interrupts */

	UWRITE2(sc, UHCI_INTR, 
		(UHCI_INTR_TOCRCIE|
		 UHCI_INTR_RIE|
		 UHCI_INTR_IOCE|
		 UHCI_INTR_SPIE));

	/* assume 64 byte packets at frame end and
	 * start HC controller
	 */

	UHCICMD(sc, (UHCI_CMD_MAXP|UHCI_CMD_RS));

	u_int8_t n = 10;
	while(n--)
	{
		/* wait one millisecond */

		DELAY(1000);

		/* check that controller has started */

		if(!(UREAD2(sc, UHCI_STS) & UHCI_STS_HCH))
		{
			goto done;
		}
	}

	device_printf(sc->sc_bus.bdev,
		      "cannot start HC controller\n");

 done:
	return;
}

usbd_status
uhci_init(uhci_softc_t *sc)
{
	struct uhci_hw_softc *hw_ptr;
	u_int16_t bit;
	u_int16_t x;
	u_int16_t y;

	mtx_lock(&sc->sc_bus.mtx);

	hw_ptr = sc->sc_hw_ptr;

	DPRINTF(("start\n"));

#ifdef USB_DEBUG
	if(uhcidebug > 2)
	{
		uhci_dumpregs(sc);
	}
#endif

	sc->sc_saved_sof = 0x40; /* default value */
	sc->sc_saved_frnum = 0; /* default frame number */

	usbd_page_dma_exit(&(sc->sc_hw_page));

	/*
	 * setup self pointers
	 */
	hw_ptr->ls_ctl_start.qh_self = 
	  htole32(SC_HW_PHYSADDR(sc,ls_ctl_start)|UHCI_PTR_QH);
	hw_ptr->ls_ctl_start.page = 
	  &(sc->sc_hw_page);

	hw_ptr->hs_ctl_start.qh_self = 
	  htole32(SC_HW_PHYSADDR(sc,hs_ctl_start)|UHCI_PTR_QH);
	hw_ptr->hs_ctl_start.page = 
	  &(sc->sc_hw_page);

	hw_ptr->bulk_start.qh_self = 
	  htole32(SC_HW_PHYSADDR(sc,bulk_start)|UHCI_PTR_QH);
	hw_ptr->bulk_start.page =
	  &(sc->sc_hw_page);

	hw_ptr->last_qh.qh_self = 
	  htole32(SC_HW_PHYSADDR(sc,last_qh)|UHCI_PTR_QH);
	hw_ptr->last_qh.page =
	  &(sc->sc_hw_page);

	hw_ptr->last_td.td_self = 
	  htole32(SC_HW_PHYSADDR(sc,last_td)|UHCI_PTR_TD);
	hw_ptr->last_td.page =
	  &(sc->sc_hw_page);

	for(x = 0;
	    x < UHCI_VFRAMELIST_COUNT;
	    x++)
	{
		hw_ptr->isoc_start[x].td_self = 
		  htole32(SC_HW_PHYSADDR(sc,isoc_start[x])|UHCI_PTR_TD);
		hw_ptr->isoc_start[x].page = 
		  &(sc->sc_hw_page);
		sc->sc_isoc_p_last[x] = 
		  &(hw_ptr->isoc_start[x]);
	}

	for(x = 0;
	    x < UHCI_IFRAMELIST_COUNT;
	    x++)
	{
		hw_ptr->intr_start[x].qh_self = 
		  htole32(SC_HW_PHYSADDR(sc,intr_start[x])|UHCI_PTR_QH);
		hw_ptr->intr_start[x].page =
		  &(sc->sc_hw_page);
		sc->sc_intr_p_last[x] = 
		  &(hw_ptr->intr_start[x]);
	}

	/*
	 * the QHs are arranged to give poll intervals that are
	 * powers of 2 times 1ms
	 */
	bit = UHCI_IFRAMELIST_COUNT/2;
	while(bit)
	{
		x = bit;
		while(x & bit)
		{
			y = (x ^ bit)|(bit/2);
			/* the next QH has half the
			 * poll interval
			 */
			hw_ptr->intr_start[x].h_next = NULL;
			hw_ptr->intr_start[x].qh_h_next = 
			  hw_ptr->intr_start[y].qh_self;
			hw_ptr->intr_start[x].e_next = NULL;
			hw_ptr->intr_start[x].qh_e_next = htole32(UHCI_PTR_T);
			x++;
		}
		bit >>= 1;
	}

	/* start QH for interrupt traffic */
	hw_ptr->intr_start[0].h_next = &(hw_ptr->ls_ctl_start);
	hw_ptr->intr_start[0].qh_h_next = hw_ptr->ls_ctl_start.qh_self;
	hw_ptr->intr_start[0].e_next = 0;
	hw_ptr->intr_start[0].qh_e_next = htole32(UHCI_PTR_T);

	for(x = 0;
	    x < UHCI_VFRAMELIST_COUNT;
	    x++)
	{
		/* start TD for isochronous traffic */
		hw_ptr->isoc_start[x].next = NULL;
		hw_ptr->isoc_start[x].td_next = 
		  hw_ptr->intr_start[x|(UHCI_IFRAMELIST_COUNT/2)].qh_self;
		hw_ptr->isoc_start[x].td_status = htole32(UHCI_TD_IOS);
		hw_ptr->isoc_start[x].td_token = htole32(0);
		hw_ptr->isoc_start[x].td_buffer = htole32(0);
	}

	/* start QH where low speed control traffic will be queued */
	hw_ptr->ls_ctl_start.h_next = &(hw_ptr->hs_ctl_start);
	hw_ptr->ls_ctl_start.qh_h_next = hw_ptr->hs_ctl_start.qh_self;
	hw_ptr->ls_ctl_start.e_next = 0;
	hw_ptr->ls_ctl_start.qh_e_next = htole32(UHCI_PTR_T);

	sc->sc_ls_ctl_p_last = &(hw_ptr->ls_ctl_start);

	/* start QH where high speed control traffic will be queued */
	hw_ptr->hs_ctl_start.h_next = &(hw_ptr->bulk_start);
	hw_ptr->hs_ctl_start.qh_h_next = hw_ptr->bulk_start.qh_self;
	hw_ptr->hs_ctl_start.e_next = 0;
	hw_ptr->hs_ctl_start.qh_e_next = htole32(UHCI_PTR_T);

	sc->sc_hs_ctl_p_last = &(hw_ptr->hs_ctl_start);

	/* start QH where bulk traffic will be queued */
	hw_ptr->bulk_start.h_next = &(hw_ptr->last_qh);
	hw_ptr->bulk_start.qh_h_next = hw_ptr->last_qh.qh_self;
	hw_ptr->bulk_start.e_next = 0;
	hw_ptr->bulk_start.qh_e_next = htole32(UHCI_PTR_T);

	sc->sc_bulk_p_last = &(hw_ptr->bulk_start);

	/* end QH which is used for looping the QHs */
	hw_ptr->last_qh.h_next = 0;
	hw_ptr->last_qh.qh_h_next = htole32(UHCI_PTR_T);	/* end of QH chain */
	hw_ptr->last_qh.e_next = &(hw_ptr->last_td);
	hw_ptr->last_qh.qh_e_next = hw_ptr->last_td.td_self;

	/* end TD which hangs from the last QH, 
	 * to avoid a bug in the PIIX that 
	 * makes it run berserk otherwise
	 */
	hw_ptr->last_td.next = 0;
	hw_ptr->last_td.td_next = htole32(UHCI_PTR_T);
	hw_ptr->last_td.td_status = htole32(0); /* inactive */
	hw_ptr->last_td.td_token = htole32(0);
	hw_ptr->last_td.td_buffer = htole32(0);

	/* setup UHCI framelist */

	for(x = 0;
	    x < UHCI_FRAMELIST_COUNT;
	    x++)
	{
		hw_ptr->pframes[x] = hw_ptr->isoc_start[x % UHCI_VFRAMELIST_COUNT].td_self;
	}

	usbd_page_dma_enter(&(sc->sc_hw_page));

	LIST_INIT(&sc->sc_interrupt_list_head);

	/* set up the bus struct */
	sc->sc_bus.methods = &uhci_bus_methods;

	/* reset the controller */
	uhci_reset(sc); 

	/* start the controller */
	uhci_start(sc);

	mtx_unlock(&sc->sc_bus.mtx);

	return 0;
}

/* NOTE: suspend/resume is called from
 * interrupt context and cannot sleep!
 */

void
uhci_suspend(uhci_softc_t *sc)
{
	mtx_lock(&sc->sc_bus.mtx);

#ifdef USB_DEBUG
	if(uhcidebug > 2)
	{
		uhci_dumpregs(sc);
	}
#endif
	/* save some state if BIOS doesn't */

	sc->sc_saved_frnum = UREAD2(sc, UHCI_FRNUM);
	sc->sc_saved_sof = UREAD1(sc, UHCI_SOF);

	/* stop the controller */

	uhci_reset(sc);

	/* enter global suspend */

	UHCICMD(sc, UHCI_CMD_EGSM);

	DELAY(1000*USB_RESUME_WAIT);

	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

void
uhci_resume(uhci_softc_t *sc)
{
	mtx_lock(&sc->sc_bus.mtx);

	/* reset the controller */

	uhci_reset(sc);	

	/* force global resume */

	UHCICMD(sc, UHCI_CMD_FGR);

	DELAY(1000*USB_RESUME_DELAY);

	/* and start traffic again */

	uhci_start(sc);

#ifdef USB_DEBUG
	if(uhcidebug > 2)
	{
		uhci_dumpregs(sc);
	}
#endif

	mtx_unlock(&sc->sc_bus.mtx);
	return;
}

#ifdef USB_DEBUG
static void
uhci_dumpregs(uhci_softc_t *sc)
{
	DPRINTFN(-1,("%s regs: cmd=%04x, sts=%04x, intr=%04x, frnum=%04x, "
		     "flbase=%08x, sof=%04x, portsc1=%04x, portsc2=%04x\n",
		     device_get_nameunit(sc->sc_bus.bdev),
		     UREAD2(sc, UHCI_CMD),
		     UREAD2(sc, UHCI_STS),
		     UREAD2(sc, UHCI_INTR),
		     UREAD2(sc, UHCI_FRNUM),
		     UREAD4(sc, UHCI_FLBASEADDR),
		     UREAD1(sc, UHCI_SOF),
		     UREAD2(sc, UHCI_PORTSC1),
		     UREAD2(sc, UHCI_PORTSC2)));
	return;
}

static uint8_t
uhci_dump_td(uhci_td_t *p)
{
	uint32_t td_next;
	uint32_t td_status;
	uint32_t td_token;
	uint8_t temp;

	usbd_page_dma_exit(p->page);

	td_next = le32toh(p->td_next);
	td_status = le32toh(p->td_status);
	td_token = le32toh(p->td_token);

	/* Check whether the link pointer in this TD marks
	 * the link pointer as end of queue:
	 */
	temp = ((td_next & UHCI_PTR_T) || (td_next == 0));

	printf("TD(%p) at 0x%08x = link=0x%08x status=0x%08x "
	       "token=0x%08x buffer=0x%08x\n",
	       p, 
	       le32toh(p->td_self),
	       td_next,
	       td_status,
	       td_token,
	       le32toh(p->td_buffer));

	printf("TD(%p) td_next=%s%s%s td_status=%s%s%s%s%s%s%s%s%s%s%s, errcnt=%d, actlen=%d pid=%02x,"
	       "addr=%d,endpt=%d,D=%d,maxlen=%d\n",
	       p,
	       (td_next & 1) ? "-T" : "",
	       (td_next & 2) ? "-Q" : "",
	       (td_next & 4) ? "-VF" : "",
	       (td_status & UHCI_TD_BITSTUFF) ? "-BITSTUFF" : "",
	       (td_status & UHCI_TD_CRCTO) ? "-CRCTO" : "",
	       (td_status & UHCI_TD_NAK) ? "-NAK" : "",
	       (td_status & UHCI_TD_BABBLE) ? "-BABBLE" : "",
	       (td_status & UHCI_TD_DBUFFER) ? "-DBUFFER" : "",
	       (td_status & UHCI_TD_STALLED) ? "-STALLED" : "",
	       (td_status & UHCI_TD_ACTIVE) ? "-ACTIVE" : "",
	       (td_status & UHCI_TD_IOC) ? "-IOC" : "",
	       (td_status & UHCI_TD_IOS) ? "-IOS" : "",
	       (td_status & UHCI_TD_LS) ? "-LS" : "",
	       (td_status & UHCI_TD_SPD) ? "-SPD" : "",
	       UHCI_TD_GET_ERRCNT(td_status),
	       UHCI_TD_GET_ACTLEN(td_status),
	       UHCI_TD_GET_PID(td_token),
	       UHCI_TD_GET_DEVADDR(td_token),
	       UHCI_TD_GET_ENDPT(td_token),
	       UHCI_TD_GET_DT(td_token),
	       UHCI_TD_GET_MAXLEN(td_token));

	usbd_page_dma_enter(p->page);

	return temp;
}

static uint8_t
uhci_dump_qh(uhci_qh_t *sqh)
{
	uint8_t temp;
	uint32_t qh_h_next;
	uint32_t qh_e_next;

	usbd_page_dma_exit(sqh->page);

	qh_h_next = le32toh(sqh->qh_h_next);
	qh_e_next = le32toh(sqh->qh_e_next);

	DPRINTFN(-1,("QH(%p) at 0x%08x: h_next=0x%08x e_next=0x%08x\n", sqh,
		     le32toh(sqh->qh_self), qh_h_next, qh_e_next));

	temp = ((((sqh->h_next != NULL) && !(qh_h_next & UHCI_PTR_T)) ? 1 : 0) |
		(((sqh->e_next != NULL) && !(qh_e_next & UHCI_PTR_T)) ? 2 : 0));

	usbd_page_dma_enter(sqh->page);
	return temp;
}

static void
uhci_dump_all(uhci_softc_t *sc)
{
	struct uhci_hw_softc *hw_ptr = sc->sc_hw_ptr;

	uhci_dumpregs(sc);
	printf("intrs=%d\n", sc->sc_bus.no_intrs);
	uhci_dump_qh(&(hw_ptr->ls_ctl_start));
	uhci_dump_qh(&(hw_ptr->hs_ctl_start));
	uhci_dump_qh(&(hw_ptr->bulk_start));
	uhci_dump_qh(&(hw_ptr->last_qh));
	return;
}

static void
uhci_dump_qhs(uhci_qh_t *sqh)
{
	uint8_t temp;

	temp = uhci_dump_qh(sqh);

	/* uhci_dump_qhs displays all the QHs and TDs from the given QH onwards
	 * Traverses sideways first, then down.
	 *
	 * QH1
	 * QH2
	 * No QH
	 * TD2.1
	 * TD2.2
	 * TD1.1
	 * etc.
	 *
	 * TD2.x being the TDs queued at QH2 and QH1 being referenced from QH1.
	 */

	if (temp & 1)
		uhci_dump_qhs(sqh->h_next);
	else
		DPRINTF(("No QH\n"));

	if (temp & 2)
		uhci_dump_tds(sqh->e_next);
	else
		DPRINTF(("No TD\n"));

	return;
}

static void
uhci_dump_tds(uhci_td_t *td)
{
	for(;
	    td != NULL; 
	    td = td->obj_next)
	{
		if (uhci_dump_td(td)) {
			break;
		}
	}
	return;
}
#if 0
static void
uhci_dump_iis(struct uhci_softc *sc)
{
	struct usbd_xfer *xfer;

	printf("interrupt list:\n");
	LIST_FOREACH(xfer, &sc->sc_interrupt_list_head, interrupt_list)
	{
		usbd_dump_xfer(xfer);
	}
	return;
}
#endif
#endif

/*
 * Let the last QH loop back to the high speed control transfer QH.
 * This is what intel calls "bandwidth reclamation" and improves
 * USB performance a lot for some devices.
 * If we are already looping, just count it.
 */
static void
uhci_add_loop(uhci_softc_t *sc)
{
	struct uhci_hw_softc *hw_ptr;

#ifdef USB_DEBUG
	if(uhcinoloop)
	{
		return;
	}
#endif
	if(++sc->sc_loops == 1)
	{
		DPRINTFN(5,("add\n"));

		hw_ptr = sc->sc_hw_ptr;

		usbd_page_dma_exit(&(sc->sc_hw_page));

		/* NOTE: we don't loop back the soft pointer */
#if 0
		hw_ptr->last_qh.qh_h_next =
		  hw_ptr->hs_ctl_start.qh_self;
#else
		hw_ptr->last_qh.qh_h_next =
		  hw_ptr->bulk_start.qh_self;
#endif
		usbd_page_dma_enter(&(sc->sc_hw_page));
	}
	return;
}

static void
uhci_rem_loop(uhci_softc_t *sc)
{
	struct uhci_hw_softc *hw_ptr;

#ifdef USB_DEBUG
	if(uhcinoloop)
	{
		return;
	}
#endif
	if(--sc->sc_loops == 0)
	{
		DPRINTFN(5,("remove\n"));
		hw_ptr = sc->sc_hw_ptr;

		usbd_page_dma_exit(&(sc->sc_hw_page));
		hw_ptr->last_qh.qh_h_next = htole32(UHCI_PTR_T);
		usbd_page_dma_enter(&(sc->sc_hw_page));
	}
	return;
}

#define UHCI_APPEND_TD(std,last) (last) = _uhci_append_td(std,last)
static uhci_td_t *
_uhci_append_td(uhci_td_t *std, uhci_td_t *last)
{
	DPRINTFN(10, ("%p to %p\n", std, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(std->page);

	std->next = last->next;
	std->td_next = last->td_next;

	std->prev = last;

	usbd_page_dma_enter(std->page);
	usbd_page_dma_exit(last->page);

	/* the last->next->prev is never followed:
	 * std->next->prev = std;
	 */
	last->next = std;
	last->td_next = std->td_self;

	usbd_page_dma_enter(last->page);

	return(std);
}

#define UHCI_APPEND_QH(sqh,td,last) (last) = _uhci_append_qh(sqh,td,last)
static uhci_qh_t *
_uhci_append_qh(uhci_qh_t *sqh, uhci_td_t *td, uhci_qh_t *last)
{
	DPRINTFN(10, ("%p to %p\n", sqh, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(sqh->page);

	sqh->e_next = td;
	sqh->qh_e_next = td->td_self;

	sqh->h_next = last->h_next;
	sqh->qh_h_next = last->qh_h_next;

	sqh->h_prev = last;

	usbd_page_dma_enter(sqh->page);
	usbd_page_dma_exit(last->page);

	/* the last->h_next->h_prev is never followed:
	 * sqh->h_next->h_prev = sqh;
	 */

	last->h_next = sqh;
	last->qh_h_next = sqh->qh_self;

	usbd_page_dma_enter(last->page);

	return(sqh);
}
/**/

#define UHCI_REMOVE_TD(std,last) (last) = _uhci_remove_td(std,last)
static uhci_td_t *
_uhci_remove_td(uhci_td_t *std, uhci_td_t *last)
{
	DPRINTFN(10, ("%p from %p\n", std, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(std->prev->page);

	std->prev->next = std->next;
	std->prev->td_next = std->td_next;

	usbd_page_dma_enter(std->prev->page);

	if(std->next)
	{
		usbd_page_dma_exit(std->next->page);
		std->next->prev = std->prev;
		usbd_page_dma_enter(std->next->page);
	}

	return((last == std) ? std->prev : last);
}

#define UHCI_REMOVE_QH(sqh,last) (last) = _uhci_remove_qh(sqh,last)
static uhci_qh_t *
_uhci_remove_qh(uhci_qh_t *sqh, uhci_qh_t *last)
{
	DPRINTFN(10, ("%p from %p\n", sqh, last));

	/* (sc->sc_bus.mtx) must be locked */

	/* only remove if not removed from a queue */
	if(sqh->h_prev)
	{
		usbd_page_dma_exit(sqh->h_prev->page);

		sqh->h_prev->h_next = sqh->h_next;
		sqh->h_prev->qh_h_next = sqh->qh_h_next;

		usbd_page_dma_enter(sqh->h_prev->page);

		if(sqh->h_next)
		{
			usbd_page_dma_exit(sqh->h_next->page);
			sqh->h_next->h_prev = sqh->h_prev;
			usbd_page_dma_enter(sqh->h_next->page);
		}

		usbd_page_dma_exit(sqh->page);

		/* set the Terminate-bit in the e_next of the QH,
		 * in case the transferred packet was short so
		 * that the QH still points at the last used TD
		 */
		sqh->qh_e_next = htole32(UHCI_PTR_T);

		usbd_page_dma_enter(sqh->page);

		last = ((last == sqh) ? sqh->h_prev : last);

		sqh->h_prev = 0;
	}
	return(last);
}

static void
uhci_device_done(struct usbd_xfer *xfer, usbd_status error);

static u_int8_t
uhci_isoc_done(uhci_softc_t *sc, struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	struct usbd_page_search fix_res;
	u_int32_t nframes = xfer->nframes;
	u_int32_t actlen = 0;
	uint32_t status;
	u_int16_t *plen = xfer->frlengths;
	u_int16_t len = 0;
	u_int16_t temp;
	u_int8_t need_delay = 0;
	uhci_td_t *td = xfer->td_transfer_first;
	uhci_td_t **pp_last = &sc->sc_isoc_p_last[xfer->qh_pos];

	DPRINTFN(12, ("xfer=%p pipe=%p transfer done\n",
		      xfer, xfer->pipe));

	while(nframes--)
	{
	  if(td == NULL)
	  {
		panic("%s:%d: out of TD's\n",
		      __FUNCTION__, __LINE__);
	  }

	  if(pp_last >= &sc->sc_isoc_p_last[UHCI_VFRAMELIST_COUNT])
	  {
		pp_last = &sc->sc_isoc_p_last[0];
	  }
#ifdef USB_DEBUG
	  if(uhcidebug > 5)
	  {
		DPRINTFN(-1,("isoc TD\n"));
		uhci_dump_td(td);
	  }
#endif
	  usbd_page_dma_exit(td->page);
	  status = le32toh(td->td_status);
	  usbd_page_dma_enter(td->page);

	  /* check for active transfers */

	  if (status & UHCI_TD_ACTIVE) {
		need_delay = 1;
	  }

	  len = UHCI_TD_GET_ACTLEN(status);

	  if(len > *plen)
	  {
		len = *plen;
	  }

	  *plen = len;
	  actlen += len;

	  if (td->fixup_src_offset != 0xffffffff) {

	      usbd_get_page(&(xfer->buf_data), td->fixup_dst_offset, &buf_res);
	      usbd_get_page(&(xfer->buf_fixup), td->fixup_src_offset, &fix_res);
 
	      temp = min(buf_res.length, len);

	      usbd_page_dma_exit(fix_res.page);

	      usbd_page_dma_exit(buf_res.page);
	      bcopy(fix_res.buffer, buf_res.buffer, temp);
	      usbd_page_dma_enter(buf_res.page);

	      len -= temp;

	      if (len) {

		  usbd_get_page(&(xfer->buf_data), 
				td->fixup_dst_offset + temp, &buf_res);

		  usbd_page_dma_exit(buf_res.page);
		  bcopy(ADD_BYTES(fix_res.buffer, temp), 
			buf_res.buffer, len);
		  usbd_page_dma_enter(buf_res.page);
	      }

	      usbd_page_dma_enter(fix_res.page);
	  }

	  /* remove TD from schedule */
	  UHCI_REMOVE_TD(td, *pp_last);

	  pp_last++;
	  plen++;
	  td = td->obj_next;
	}
	xfer->actlen = actlen;

	return need_delay;
}

static void
uhci_non_isoc_done(struct usbd_xfer *xfer)
{
	uint32_t status = 0;
	uint32_t token = 0;
	u_int32_t actlen = 0;
	uhci_td_t *td = xfer->td_transfer_first;

	DPRINTFN(12, ("xfer=%p pipe=%p transfer done\n",
		      xfer, xfer->pipe));

#ifdef USB_DEBUG
	if(uhcidebug > 10)
	{
		uhci_dump_tds(td);
	}
#endif

	/* the transfer is done, compute actual length and status */
	for (;
	     td != NULL;
	     td = td->obj_next)
	{
		usbd_page_dma_exit(td->page);
		status = le32toh(td->td_status);
		token = le32toh(td->td_token);
		usbd_page_dma_enter(td->page);

		if (status & (UHCI_TD_ACTIVE|UHCI_TD_STALLED)) {
			break;
		}

		actlen += UHCI_TD_GET_ACTLEN(status);

		if (((void *)td) == xfer->td_transfer_last) {
			td = NULL;
			break;
		}
	}

	/* if there are left over TDs 
	 * the toggle needs to be updated
	 */
	if(td != NULL)
	{
		xfer->pipe->toggle_next = (token & UHCI_TD_SET_DT(1)) ? 1 : 0;
	}

	DPRINTFN(10, ("actlen=%d\n", actlen));

	xfer->actlen = actlen;

#ifdef USB_DEBUG
	if (status & UHCI_TD_ERROR) {
		DPRINTFN(10,
			 ("error, addr=%d, endpt=0x%02x, "
			  "status=%s%s%s%s%s%s%s%s%s%s%s\n",
			  xfer->address,
			  xfer->endpoint,
			  (status & UHCI_TD_BITSTUFF) ? "-BITSTUFF" : "",
			  (status & UHCI_TD_CRCTO) ? "-CRCTO" : "",
			  (status & UHCI_TD_NAK) ? "-NAK" : "",
			  (status & UHCI_TD_BABBLE) ? "-BABBLE" : "",
			  (status & UHCI_TD_DBUFFER) ? "-DBUFFER" : "",
			  (status & UHCI_TD_STALLED) ? "-STALLED" : "",
			  (status & UHCI_TD_ACTIVE) ? "-ACTIVE" : "",
			  (status & UHCI_TD_IOC) ? "-IOC" : "",
			  (status & UHCI_TD_IOS) ? "-IOS" : "",
			  (status & UHCI_TD_LS) ? "-LS" : "",
			  (status & UHCI_TD_SPD) ? "-SPD" : ""));
	}
#endif
	uhci_device_done(xfer, (status & UHCI_TD_STALLED) ? 
			 USBD_STALLED : 
			 USBD_NORMAL_COMPLETION);
	return;
}

/* returns one when transfer is finished 
 * and callback must be called; else zero
 */
static u_int8_t
uhci_check_transfer(struct usbd_xfer *xfer, struct thread *ctd)
{
	uint32_t status;
	uint32_t token;
	uhci_td_t *td;

	if(xfer->usb_thread != ctd)
	{
	    /* cannot call this transfer 
	     * back due to locking !
	     */
	    goto done;
	}

	DPRINTFN(15, ("xfer=%p checking transfer\n", xfer));

	td = xfer->td_transfer_last;

	if(xfer->pipe->methods == &uhci_device_isoc_methods)
	{
		/* isochronous transfer */

		usbd_page_dma_exit(td->page);
		status = le32toh(td->td_status);
		usbd_page_dma_enter(td->page);

		if(!(status & UHCI_TD_ACTIVE)) {
			uhci_device_done(xfer,USBD_NORMAL_COMPLETION);
			goto transferred;
		}
	}
	else
	{
		/* non-isochronous transfer */

		/*
		 * check whether there is an error somewhere
		 * in the middle, or whether there was a short
		 * packet (SPD and not ACTIVE)
		 */
		for(td = xfer->td_transfer_cache;
		    td != NULL;
		    td = td->obj_next)
		{
			usbd_page_dma_exit(td->page);

			status = le32toh(td->td_status);
			token = le32toh(td->td_token);

			usbd_page_dma_enter(td->page);

			/* if there is an active TD 
			 * the transfer isn't done
			 */
			if (status & UHCI_TD_ACTIVE) {
				/* update cache */
				xfer->td_transfer_cache = td;
				goto done;
			}

			/* any kind of error makes
			 * the transfer done 
			 */
			if (status & UHCI_TD_STALLED) {
				break;
			}

			/* a short packet also makes
			 * the transfer done
			 */
			if ((status & UHCI_TD_SPD) &&
			    (UHCI_TD_GET_ACTLEN(status) <
			     UHCI_TD_GET_MAXLEN(token))) {
				break;
			}

			if (((void *)td) == xfer->td_transfer_last) {
				td = NULL;
				break;
			}
		}
		uhci_non_isoc_done(xfer);
		goto transferred;
	}

 done:
	DPRINTFN(12, ("xfer=%p is still active\n", xfer));
	return 0;

 transferred:
	return 1;
}

static void
uhci_interrupt_td(uhci_softc_t *sc, struct thread *ctd)
{
	enum { FINISH_LIST_MAX = 16 };

	struct usbd_callback_info info[FINISH_LIST_MAX];
	struct usbd_callback_info *ptr = &info[0];
	struct usbd_xfer *xfer;
	u_int32_t status;
	u_int8_t need_repeat = 0;

	mtx_lock(&sc->sc_bus.mtx);

	/*
	 * It can happen that an interrupt will be delivered to
	 * us before the device has been fully attached and the
	 * softc struct has been configured. Usually this happens
	 * when kldloading the USB support as a module after the
	 * system has been booted. If we detect this condition,
	 * we need to squelch the unwanted interrupts until we're
	 * ready for them.
	 */
	if(sc->sc_bus.bdev == NULL) /* XXX */
	{
#if 0
		UWRITE2(sc, UHCI_STS, 0xFFFF);	/* ack pending interrupts */
		uhci_reset(sc);			/* stop the controller */
		UWRITE2(sc, UHCI_INTR, 0);	/* disable interrupts */
#endif
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

	DPRINTFN(15,("%s: real interrupt\n",
		     device_get_nameunit(sc->sc_bus.bdev)));

#ifdef USB_DEBUG
	if(uhcidebug > 15)
	{
		DPRINTF(("%s\n", device_get_nameunit(sc->sc_bus.bdev)));
		uhci_dumpregs(sc);
	}
#endif
	status = UREAD2(sc, UHCI_STS) & UHCI_STS_ALLINTRS;
	if(status == 0)
	{
		/* the interrupt was not for us */
		goto done;
	}

	if(status & UHCI_STS_RD)
	{
#ifdef USB_DEBUG
		device_printf(sc->sc_bus.bdev,
			      "resume detect\n");
#endif
	}
	if(status & UHCI_STS_HSE)
	{
		device_printf(sc->sc_bus.bdev,
			      "host system error\n");
	}
	if(status & UHCI_STS_HCPE)
	{
		device_printf(sc->sc_bus.bdev,
			      "host controller process error\n");
	}
	if(status & UHCI_STS_HCH)
	{
		/* no acknowledge needed */
		device_printf(sc->sc_bus.bdev,
			      "host controller halted\n");
#ifdef USB_DEBUG
		uhci_dump_all(sc);
#endif
	}

	/* get acknowledge bits */
	status &= (UHCI_STS_USBINT|
		   UHCI_STS_USBEI|
		   UHCI_STS_RD|
		   UHCI_STS_HSE|
		   UHCI_STS_HCPE);

	if(status == 0)
	{
		/* nothing to acknowledge */
		goto done;
	}

	/* acknowledge interrupts */
	UWRITE2(sc, UHCI_STS, status); 

	/*
	 * when the host controller interrupts because a transfer
	 * is completed, all active transfers must be checked!
	 * The UHCI controller does not have a queue for finished
	 * transfers
	 */
 repeat:
	LIST_FOREACH(xfer, &sc->sc_interrupt_list_head, interrupt_list)
	{
		/* check if transfer is
		 * transferred 
		 */
		if(uhci_check_transfer(xfer, ctd))
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
uhci_interrupt(uhci_softc_t *sc)
{
	uhci_interrupt_td(sc, NULL);
	return;
}

/*
 * called when a request does not complete
 */
static void
uhci_timeout(struct usbd_xfer *xfer)
{
	struct usbd_callback_info info[1];
	uhci_softc_t *sc = xfer->usb_sc;

	DPRINTF(("xfer=%p\n", xfer));

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	uhci_device_done(xfer, USBD_TIMEOUT);

	/* queue callback */
	info[0].xfer = xfer;
	info[0].refcount = xfer->usb_refcount;

	xfer->usb_root->memory_refcount++;

	mtx_unlock(&sc->sc_bus.mtx);

	usbd_do_callback(&info[0],&info[1]);

	return;
}

static void
uhci_do_poll(struct usbd_bus *bus)
{
	uhci_interrupt_td(UHCI_BUS2SC(bus), curthread);
	return;
}

#define uhci_add_interrupt_info(sc, xfer) \
	LIST_INSERT_HEAD(&(sc)->sc_interrupt_list_head, (xfer), interrupt_list)

static void
uhci_remove_interrupt_info(struct usbd_xfer *xfer)
{
	if((xfer)->interrupt_list.le_prev)
	{
		LIST_REMOVE((xfer), interrupt_list);
		(xfer)->interrupt_list.le_prev = NULL;
	}
	return;
}

static uhci_td_t *
uhci_setup_standard_chain(struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	u_int32_t td_status;
	u_int32_t td_token;
	u_int32_t average = xfer->max_packet_size;
	u_int32_t buf_offset;
	u_int32_t len = xfer->length;
	u_int8_t isread;
	u_int8_t shortpkt = 0;
	uhci_td_t *td;
	uhci_td_t *td_last = NULL;

	DPRINTFN(8, ("addr=%d endpt=%d len=%d speed=%d\n", 
		     xfer->address, UE_GET_ADDR(xfer->endpoint),
		     xfer->length, xfer->udev->speed));

	td = (xfer->td_transfer_first = 
	      xfer->td_transfer_cache = xfer->td_start);

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	td_status = htole32(UHCI_TD_ZERO_ACTLEN(UHCI_TD_SET_ERRCNT(3)|
						UHCI_TD_ACTIVE));

	if(xfer->udev->speed == USB_SPEED_LOW)
	{
		td_status |= htole32(UHCI_TD_LS);
	}

 	if(xfer->flags & USBD_SHORT_XFER_OK)
	{
		/* set UHCI_TD_SPD */
		td_status |= htole32(UHCI_TD_SPD);
	}

	if(xfer->pipe->methods == &uhci_device_ctrl_methods)
	{
		/* the first byte is "bmRequestType" */

		isread = *((u_int8_t *)(buf_res.buffer));
		isread &= UT_READ;

		/*
		 * check length ?
		 */

		xfer->pipe->toggle_next = 1;

		/* SETUP message */

		usbd_page_dma_exit(td->page);

		td->td_status = td_status & htole32(~UHCI_TD_SPD);
		td->td_token = 
		  htole32(UHCI_TD_SET_ENDPT(xfer->endpoint)) |
		  htole32(UHCI_TD_SET_DEVADDR(xfer->address)) |
		  htole32(UHCI_TD_SET_MAXLEN(sizeof(usb_device_request_t))) |
		  htole32(UHCI_TD_PID_SETUP)|
		  htole32(UHCI_TD_SET_DT(0));

		td->td_buffer = htole32(buf_res.physaddr);

		buf_offset += sizeof(usb_device_request_t);
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);
		len -= sizeof(usb_device_request_t);
		td_last = td;
		td = td->obj_next;

		if (td) {
		    /* link the last TD with the next one */
		    td_last->td_next = td->td_self;
		}

		usbd_page_dma_enter(td_last->page);
	}
	else
	{
		isread = (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN);

		if(xfer->length == 0)
		{
			/* must allow access to "td_last",
			 * so xfer->length cannot be zero!
			 */
			printf("%s: setting USBD_FORCE_SHORT_XFER!\n", __FUNCTION__);
			xfer->flags |= USBD_FORCE_SHORT_XFER;
		}
	}

	td_token =
	  htole32(UHCI_TD_SET_ENDPT(xfer->endpoint)) |
	  htole32(UHCI_TD_SET_DEVADDR(xfer->address)) |
	  htole32(UHCI_TD_SET_MAXLEN(average)) |
	  (isread ? htole32(UHCI_TD_PID_IN) : htole32(UHCI_TD_PID_OUT));

	if(xfer->pipe->toggle_next)
	{
		td_token |= htole32(UHCI_TD_SET_DT(1));
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
			shortpkt = 1;
			average = len;

			/* update length */
			td_token &= htole32(~(0x7ff<<21));
			td_token |= htole32(UHCI_TD_SET_MAXLEN(average));
		}

		if(td == NULL)
		{
			panic("%s: software wants to write more data "
			      "than there is in the buffer!", __FUNCTION__);
		}

		usbd_page_dma_exit(td->page);

		/* fill out current TD */
       
		td->td_status = td_status;
		td->td_token = td_token;
		td->td_buffer = htole32(buf_res.physaddr);

		td_token ^= htole32(UHCI_TD_SET_DT(1));

		buf_offset += average;
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);
		len -= average;
		td_last = td;
		td = td->obj_next;

		if (td) {
		    /* link the last TD with the next one */
		    td_last->td_next = td->td_self;
		}

		usbd_page_dma_enter(td_last->page);
	}

	/* set interrupt bit */
	td_status |= htole32(UHCI_TD_IOC);

	if(xfer->pipe->methods == &uhci_device_ctrl_methods)
	{
		/* STATUS message */

		/* update length and PID */
		td_token &= htole32(~(0x7ff<<21));
		td_token |= htole32(UHCI_TD_SET_MAXLEN(0)|UHCI_TD_SET_DT(1));
		td_token ^= htole32(0x88);

		usbd_page_dma_exit(td->page);

		td->td_token = td_token;
		td->td_buffer = htole32(0);

		td_last = td;

		usbd_page_dma_enter(td_last->page);
	}

	usbd_page_dma_exit(td_last->page);

	td_last->td_next = htole32(UHCI_PTR_T);
	td_last->td_status = td_status;

	usbd_page_dma_enter(td_last->page);

	/* must have at least one frame! */

	xfer->td_transfer_last = td_last;

	/* store data-toggle */
	if(td_token & htole32(UHCI_TD_SET_DT(1)))
	{
		xfer->pipe->toggle_next = 1;
	}
	else
	{
		xfer->pipe->toggle_next = 0;
	}

#ifdef USB_DEBUG
	if(uhcidebug > 8)
	{
		DPRINTF(("nexttog=%d; data before transfer:\n",
			 xfer->pipe->toggle_next));
		uhci_dump_tds(xfer->td_start);
	}
#endif
	return(xfer->td_start);
}

/* NOTE: "done" can be run two times in a row,
 * from close and from interrupt
 */

static void
uhci_device_done(struct usbd_xfer *xfer, usbd_status error)
{
	uhci_softc_t *sc = xfer->usb_sc;
	uhci_td_t *td;
	uhci_qh_t *qh;
	u_int8_t need_delay;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	need_delay = 0;

	DPRINTFN(1,("xfer=%p, pipe=%p length=%d error=%d\n",
		    xfer, xfer->pipe, xfer->actlen, error));

	for(qh = xfer->qh_start; qh; qh = qh->obj_next)
	{
		usbd_page_dma_exit(qh->page);

		if(!(qh->qh_e_next & htole32(UHCI_PTR_T)))
		{
			need_delay = 1;
		}

		qh->e_next = 0;
		qh->qh_e_next = htole32(UHCI_PTR_T);

		usbd_page_dma_enter(qh->page);
	}

	if(xfer->flags & USBD_BANDWIDTH_RECLAIMED)
	{
		xfer->flags &= ~USBD_BANDWIDTH_RECLAIMED;
		uhci_rem_loop(sc);
	}

	if(xfer->pipe->methods == &uhci_device_bulk_methods)
	{
		UHCI_REMOVE_QH(xfer->qh_start, sc->sc_bulk_p_last);
	}

	if(xfer->pipe->methods == &uhci_device_ctrl_methods)
	{
		if(xfer->udev->speed == USB_SPEED_LOW)
		{
			UHCI_REMOVE_QH(xfer->qh_start, sc->sc_ls_ctl_p_last);
		}
		else
		{
			UHCI_REMOVE_QH(xfer->qh_start, sc->sc_hs_ctl_p_last);
		}
	}

	if(xfer->pipe->methods == &uhci_device_intr_methods)
	{
		UHCI_REMOVE_QH(xfer->qh_start, sc->sc_intr_p_last[xfer->qh_pos]);
	}

	/* finish isochronous transfers or clear active bit
	 * (will update xfer->actlen and xfer->frlengths;
	 *  should only be called once)
	 */
	if(xfer->td_transfer_first &&
	   xfer->td_transfer_last)
	{
		if(xfer->pipe->methods == &uhci_device_isoc_methods)
		{
			if(uhci_isoc_done(sc, xfer))
			{
				need_delay = 1;
			}
		}

		if(need_delay)
		{
			td = xfer->td_transfer_first;

			while(1)
			{
				if(td == NULL)
				{
					panic("%s:%d: out of TD's\n",
					      __FUNCTION__, __LINE__);
				}

				usbd_page_dma_exit(td->page);

				td->td_status &= 
				  htole32(~(UHCI_TD_ACTIVE|UHCI_TD_IOC));

				usbd_page_dma_enter(td->page);

				if (((void *)td) == xfer->td_transfer_last) {
					td = NULL;
					break;
				}
				td = td->obj_next;
			}
		}
		xfer->td_transfer_first = NULL;
		xfer->td_transfer_last = NULL;
	}

	/* stop timeout */
	__callout_stop(&xfer->timeout_handle);

	/* remove interrupt info */
	uhci_remove_interrupt_info(xfer);

	/* wait until hardware has finished any possible
	 * use of the transfer and QH
	 *
	 * hardware finishes in 1 millisecond
	 */
	DELAY(need_delay ? ((3*1000)/2) : UHCI_QH_REMOVE_DELAY);

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
 * uhci bulk support
 *---------------------------------------------------------------------------*/
static void
uhci_device_bulk_open(struct usbd_xfer *xfer)
{
	return;
}

static void
uhci_device_bulk_close(struct usbd_xfer *xfer)
{
	uhci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
uhci_device_bulk_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
uhci_device_bulk_start(struct usbd_xfer *xfer)
{
	uhci_softc_t *sc = xfer->usb_sc;
	uhci_td_t *td;
	uhci_qh_t *qh;

	DPRINTFN(3, ("xfer=%p len=%d\n",
		     xfer, xfer->length));

	/* setup TD's */
	td = uhci_setup_standard_chain(xfer);

	/* setup QH */
	qh = xfer->qh_start;

	UHCI_APPEND_QH(qh, td, sc->sc_bulk_p_last);
	uhci_add_loop(sc);
	xfer->flags |= USBD_BANDWIDTH_RECLAIMED;

	/**/
	uhci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)uhci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods uhci_device_bulk_methods = 
{
  .open = uhci_device_bulk_open,
  .close = uhci_device_bulk_close,
  .enter = uhci_device_bulk_enter,
  .start = uhci_device_bulk_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

/*---------------------------------------------------------------------------*
 * uhci control support
 *---------------------------------------------------------------------------*/
static void
uhci_device_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
uhci_device_ctrl_close(struct usbd_xfer *xfer)
{
	uhci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
uhci_device_ctrl_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
uhci_device_ctrl_start(struct usbd_xfer *xfer)
{
	uhci_softc_t *sc = xfer->usb_sc;
	uhci_qh_t *qh;
	uhci_td_t *td;

	/* setup TD's */
	td = uhci_setup_standard_chain(xfer);

	/* setup QH */
	qh = xfer->qh_start;

	/* NOTE: some devices choke on bandwidth-
	 * reclamation for control transfers
	 */
	if(xfer->udev->speed == USB_SPEED_LOW)	
	{
		UHCI_APPEND_QH(qh, td, sc->sc_ls_ctl_p_last);
	}
	else
	{
		UHCI_APPEND_QH(qh, td, sc->sc_hs_ctl_p_last);
	}

	/**/
	uhci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)uhci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods uhci_device_ctrl_methods = 
{
  .open = uhci_device_ctrl_open,
  .close = uhci_device_ctrl_close,
  .enter = uhci_device_ctrl_enter,
  .start = uhci_device_ctrl_start,
  .copy_in = usbd_std_ctrl_copy_in,
  .copy_out = usbd_std_ctrl_copy_out,
};

/*---------------------------------------------------------------------------*
 * uhci interrupt support
 *---------------------------------------------------------------------------*/
static void
uhci_device_intr_open(struct usbd_xfer *xfer)
{
	uhci_softc_t *sc = xfer->usb_sc;
	u_int16_t best;
	u_int16_t bit;
	u_int16_t x;

	best = 0;
	bit = UHCI_IFRAMELIST_COUNT/2;
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
uhci_device_intr_close(struct usbd_xfer *xfer)
{
	uhci_softc_t *sc = xfer->usb_sc;

	sc->sc_intr_stat[xfer->qh_pos]--;

	uhci_device_done(xfer,USBD_CANCELLED);
	return;
}

static void
uhci_device_intr_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
uhci_device_intr_start(struct usbd_xfer *xfer)
{
	uhci_softc_t *sc = xfer->usb_sc;
	uhci_qh_t *qh;
	uhci_td_t *td;

	DPRINTFN(3,("xfer=%p len=%d\n",
		    xfer, xfer->length));

	/* setup TD's */
	td = uhci_setup_standard_chain(xfer);

	/* setup QH */
	qh = xfer->qh_start;

	/* enter QHs into the controller data structures */
	UHCI_APPEND_QH(qh, td, sc->sc_intr_p_last[xfer->qh_pos]);

	/**/
	uhci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)uhci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods uhci_device_intr_methods = 
{
  .open = uhci_device_intr_open,
  .close = uhci_device_intr_close,
  .enter = uhci_device_intr_enter,
  .start = uhci_device_intr_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

/*---------------------------------------------------------------------------*
 * uhci isochronous support
 *---------------------------------------------------------------------------*/
static void
uhci_device_isoc_open(struct usbd_xfer *xfer)
{
	uhci_td_t *td;
	u_int32_t td_token;

	td_token = 
	  (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN) ? 
		  UHCI_TD_IN (0, xfer->endpoint, xfer->address, 0) :
		  UHCI_TD_OUT(0, xfer->endpoint, xfer->address, 0) ;

	td_token = htole32(td_token);

	/* initialize all TD's */

	for(td = xfer->td_start; td; td = td->obj_next)
	{
		usbd_page_dma_exit(td->page);

		/* mark TD as inactive */
		td->td_status = htole32(UHCI_TD_IOS);
		td->td_token = td_token;

		usbd_page_dma_enter(td->page);
	}
	return;
}

static void
uhci_device_isoc_close(struct usbd_xfer *xfer)
{
	uhci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
uhci_device_isoc_enter(struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	struct usbd_page_search fix_res;
	struct usbd_page_search tmp_res;
	uhci_softc_t *sc = xfer->usb_sc;
	u_int32_t buf_offset;
	u_int32_t fix_offset;
	u_int32_t nframes;
	u_int16_t *plen;
#ifdef USB_DEBUG
	u_int8_t once = 1;
#endif
	u_int8_t isread = (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN);
	uhci_td_t *td;
	uhci_td_t *td_last = NULL;
	uhci_td_t **pp_last;

	DPRINTFN(5,("xfer=%p next=%d nframes=%d\n",
		    xfer, xfer->pipe->isoc_next, xfer->nframes));

	nframes = UREAD2(sc, UHCI_FRNUM);

	if(((nframes - xfer->pipe->isoc_next) & 
	    (UHCI_VFRAMELIST_COUNT-1)) < xfer->nframes)
	{
		/* not in use yet, schedule it a few frames ahead */
		/* data underflow */
		xfer->pipe->isoc_next = (nframes + 3) & (UHCI_VFRAMELIST_COUNT-1);
		DPRINTFN(2,("start next=%d\n", xfer->pipe->isoc_next));
	}

	xfer->isoc_complete_time = (xfer->pipe->isoc_next + xfer->nframes) % USBD_ISOC_TIME_MAX;

	nframes = xfer->nframes;

	if(nframes == 0)
	{
		/* transfer transferred */
		uhci_device_done(xfer, USBD_NORMAL_COMPLETION);

		/* call callback recursively */
		__usbd_callback(xfer);

		return;
	}

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	fix_offset = 0;
	usbd_get_page(&(xfer->buf_fixup), fix_offset, &fix_res);

	plen = xfer->frlengths;

	td = (xfer->td_transfer_first = xfer->td_start);

	pp_last = &sc->sc_isoc_p_last[xfer->pipe->isoc_next];

	/* store starting position */

	xfer->qh_pos = xfer->pipe->isoc_next;

	while(nframes--)
	{
		if(td == NULL)
		{
			panic("%s:%d: out of TD's\n",
			      __FUNCTION__, __LINE__);
		}

		if(pp_last >= &sc->sc_isoc_p_last[UHCI_VFRAMELIST_COUNT])
		{
			pp_last = &sc->sc_isoc_p_last[0];
		}

		if(*plen > xfer->max_frame_size)
		{
#ifdef USB_DEBUG
			if(once)
			{
				once = 0;
				printf("%s: frame length(%d) exceeds %d "
				       "bytes (frame truncated)\n", 
				       __FUNCTION__, *plen, 
				       xfer->max_frame_size);
			}
#endif
			*plen = xfer->max_frame_size;
		}

		usbd_page_dma_exit(td->page);

		/* reuse td_token from last transfer */

		td->td_token &= htole32(~UHCI_TD_MAXLEN_MASK);
		td->td_token |= htole32(UHCI_TD_SET_MAXLEN(*plen));

		if (buf_res.length < *plen) {

		    /* need to do a fixup */
		    td->td_buffer = htole32(fix_res.physaddr);

		    if (!isread) {

			/*
			 * The UHCI driver cannot handle
			 * page crossings, so a fixup is
			 * needed:
			 *
			 *  +----+----+ - - -
			 *  | YYY|Y   |
			 *  +----+----+ - - -
			 *     \    \
			 *      \    \
			 *       +----+
			 *       |YYYY|  (fixup)
			 *       +----+
			 */

			td->fixup_dst_offset = 0xffffffff;
			td->fixup_src_offset = 0xffffffff;

			usbd_get_page(&(xfer->buf_data), buf_offset + 
				      buf_res.length, &tmp_res);

			/* copy data to fixup location */

			usbd_page_dma_exit(fix_res.page);

			usbd_page_dma_exit(buf_res.page);
			bcopy(buf_res.buffer, fix_res.buffer, buf_res.length);
			usbd_page_dma_enter(buf_res.page);

			usbd_page_dma_exit(buf_res.page);
			bcopy(tmp_res.buffer, ADD_BYTES(fix_res.buffer, 
							buf_res.length),
			      *plen - buf_res.length);
			usbd_page_dma_enter(buf_res.page);

			usbd_page_dma_enter(fix_res.page);

		    } else {
			td->fixup_dst_offset = buf_offset;
			td->fixup_src_offset = fix_offset;
		    }

		    /* prepare next fixup */
		    fix_offset += xfer->max_packet_size;
		    usbd_get_page(&(xfer->buf_fixup), fix_offset, &fix_res);

		} else {
		    td->td_buffer = htole32(buf_res.physaddr);
		    td->fixup_dst_offset = 0xffffffff;
		    td->fixup_src_offset = 0xffffffff;
		}

		/* update status */
		if(nframes == 0)
		{
			td->td_status = htole32
			  (UHCI_TD_ZERO_ACTLEN
			   (UHCI_TD_SET_ERRCNT(0)|
			    UHCI_TD_ACTIVE|
			    UHCI_TD_IOS|
			    UHCI_TD_IOC));
		}
		else
		{
			td->td_status = htole32
			  (UHCI_TD_ZERO_ACTLEN
			   (UHCI_TD_SET_ERRCNT(0)|
			    UHCI_TD_ACTIVE|
			    UHCI_TD_IOS));
		}

		usbd_page_dma_enter(td->page);

#ifdef USB_DEBUG
		if(uhcidebug > 5)
		{
			DPRINTFN(5,("TD %d\n", nframes));
			uhci_dump_td(td);
		}
#endif
		/* insert TD into schedule */
		UHCI_APPEND_TD(td, *pp_last);
		pp_last++;

		buf_offset += *plen;
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);
		plen++;
		td_last = td;
		td = td->obj_next;
	}

	xfer->td_transfer_last = td_last;

	/* update isoc_next */
	xfer->pipe->isoc_next = (pp_last - &sc->sc_isoc_p_last[0]) &
	  (UHCI_VFRAMELIST_COUNT-1);

	/**/
	uhci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)uhci_timeout, xfer);
	}

	/* enqueue transfer 
	 * (so that it can be aborted through pipe abort)
	 */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
uhci_device_isoc_start(struct usbd_xfer *xfer)
{
	/* already started, nothing to do */
	return;
}

struct usbd_pipe_methods uhci_device_isoc_methods = 
{
  .open = uhci_device_isoc_open,
  .close = uhci_device_isoc_close,
  .enter = uhci_device_isoc_enter,
  .start = uhci_device_isoc_start,
  .copy_in = usbd_std_isoc_copy_in,
  .copy_out = usbd_std_isoc_copy_out,
};

/*---------------------------------------------------------------------------*
 * uhci root control support
 *---------------------------------------------------------------------------*
 * simulate a hardware hub by handling
 * all the necessary requests
 *---------------------------------------------------------------------------*/

static void
uhci_root_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
uhci_root_ctrl_close(struct usbd_xfer *xfer)
{
	uhci_device_done(xfer,USBD_CANCELLED);
	return;
}

/* data structures and routines
 * to emulate the root hub:
 */

static const
usb_device_descriptor_t uhci_devd =
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
usb_config_descriptor_t uhci_confd =
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
usb_interface_descriptor_t uhci_ifcd = 
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
usb_endpoint_descriptor_t uhci_endpd = 
{
	sizeof(usb_endpoint_descriptor_t),
	UDESC_ENDPOINT,
	UE_DIR_IN | UHCI_INTR_ENDPT,
	UE_INTERRUPT,
	{8},
	255
};

static const
usb_hub_descriptor_t uhci_hubd_piix = 
{
	USB_HUB_DESCRIPTOR_SIZE,
	UDESC_HUB,
	2,
	{ UHD_PWR_NO_SWITCH | UHD_OC_INDIVIDUAL, 0 },
	50,			/* power on to power good */
	0,
	{ 0x00 },		/* both ports are removable */
};

/*
 * The USB hub protocol requires that SET_FEATURE(PORT_RESET) also
 * enables the port, and also states that SET_FEATURE(PORT_ENABLE)
 * should not be used by the USB subsystem.  As we cannot issue a
 * SET_FEATURE(PORT_ENABLE) externally, we must ensure that the port
 * will be enabled as part of the reset.
 *
 * On the VT83C572, the port cannot be successfully enabled until the
 * outstanding "port enable change" and "connection status change"
 * events have been reset.
 */
static usbd_status
uhci_portreset(uhci_softc_t *sc, int index)
{
	int lim, port, x;

	if(index == 1)
		port = UHCI_PORTSC1;
	else if(index == 2)
		port = UHCI_PORTSC2;
	else
		return (USBD_IOERROR);

	x = URWMASK(UREAD2(sc, port));
	UWRITE2(sc, port, x | UHCI_PORTSC_PR);

	DELAY(1000*USB_PORT_ROOT_RESET_DELAY);

	DPRINTFN(3,("uhci port %d reset, status0 = 0x%04x\n",
		    index, UREAD2(sc, port)));

	x = URWMASK(UREAD2(sc, port));
	UWRITE2(sc, port, x & ~UHCI_PORTSC_PR);

	DELAY(100);

	DPRINTFN(3,("uhci port %d reset, status1 = 0x%04x\n",
		    index, UREAD2(sc, port)));

	x = URWMASK(UREAD2(sc, port));
	UWRITE2(sc, port, x  | UHCI_PORTSC_PE);

	lim = 10;
	while(lim--)
	{
		DELAY(1000*USB_PORT_RESET_DELAY);

		x = UREAD2(sc, port);

		DPRINTFN(3,("uhci port %d iteration %u, status = 0x%04x\n",
			    index, lim, x));

		if(!(x & UHCI_PORTSC_CCS))
		{
			/*
			 * No device is connected (or was disconnected
			 * during reset).  Consider the port reset.
			 * The delay must be long enough to ensure on
			 * the initial iteration that the device
			 * connection will have been registered.  50ms
			 * appears to be sufficient, but 20ms is not.
			 */
			DPRINTFN(3,("uhci port %d loop %u, device detached\n",
				    index, lim));
			goto done;
		}

		if(x & (UHCI_PORTSC_POEDC | UHCI_PORTSC_CSC))
		{
			/*
			 * Port enabled changed and/or connection
			 * status changed were set.  Reset either or
			 * both raised flags (by writing a 1 to that
			 * bit), and wait again for state to settle.
			 */
			UWRITE2(sc, port, URWMASK(x) |
				(x & (UHCI_PORTSC_POEDC | UHCI_PORTSC_CSC)));
			continue;
		}

		if(x & UHCI_PORTSC_PE)
		{
			/* port is enabled */
			goto done;
		}

		UWRITE2(sc, port, URWMASK(x) | UHCI_PORTSC_PE);
	}

	DPRINTFN(1,("uhci port %d reset timed out\n", index));
	return (USBD_TIMEOUT);

 done:
	DPRINTFN(3,("uhci port %d reset, status2 = 0x%04x\n",
		    index, UREAD2(sc, port)));

	sc->sc_isreset = 1;
	return (USBD_NORMAL_COMPLETION);
}

static void
uhci_root_ctrl_enter(struct usbd_xfer *xfer)
{
	uhci_softc_t *sc = xfer->usb_sc;
	u_int32_t port;
	u_int32_t x;
	u_int16_t len;
	u_int16_t value;
	u_int16_t index;
	u_int16_t status;
	u_int16_t change;
	u_int16_t l;
	u_int16_t totlen = 0;
	union {
	  usb_status_t stat;
	  usb_port_status_t ps;
	  usb_device_request_t req;
	  usb_device_descriptor_t devd;
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

			u.devd = uhci_devd;
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
			totlen = l = min(len, sizeof(uhci_confd));
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &uhci_confd, l);

			l = min(len, sizeof(uhci_ifcd));
			totlen += l;
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req) +
				     sizeof(uhci_confd), &uhci_ifcd, l);

			l = min(len, sizeof(uhci_endpd));
			totlen += l;
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req) +
				     sizeof(uhci_confd) + sizeof(uhci_ifcd),
				     &uhci_endpd, l);
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
			       "UHCI root hub");
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
		DPRINTFN(3, ("UR_CLEAR_PORT_FEATURE "
			     "port=%d feature=%d\n",
			     index, value));
		if(index == 1)
			port = UHCI_PORTSC1;
		else if(index == 2)
			port = UHCI_PORTSC2;
		else
		{
			err = USBD_IOERROR;
			goto done;
		}
		switch(value) {
		case UHF_PORT_ENABLE:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x & ~UHCI_PORTSC_PE);
			break;
		case UHF_PORT_SUSPEND:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x & ~UHCI_PORTSC_SUSP);
			break;
		case UHF_PORT_RESET:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x & ~UHCI_PORTSC_PR);
			break;
		case UHF_C_PORT_CONNECTION:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_CSC);
			break;
		case UHF_C_PORT_ENABLE:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_POEDC);
			break;
		case UHF_C_PORT_OVER_CURRENT:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_OCIC);
			break;
		case UHF_C_PORT_RESET:
			sc->sc_isreset = 0;
			err = USBD_NORMAL_COMPLETION;
			goto done;
		case UHF_PORT_CONNECTION:
		case UHF_PORT_OVER_CURRENT:
		case UHF_PORT_POWER:
		case UHF_PORT_LOW_SPEED:
		case UHF_C_PORT_SUSPEND:
		default:
			err = USBD_IOERROR;
			goto done;
		}
		break;
	case C(UR_GET_BUS_STATE, UT_READ_CLASS_OTHER):
		if(index == 1)
			port = UHCI_PORTSC1;
		else if(index == 2)
			port = UHCI_PORTSC2;
		else
		{
			err = USBD_IOERROR;
			goto done;
		}
		if(len > 0)
		{
			u.byte_temp = ((UREAD2(sc, port) & UHCI_PORTSC_LS) >>
				       UHCI_PORTSC_LS_SHIFT);
			totlen = 1;
			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &u, totlen);
		}
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_CLASS_DEVICE):
		if((value & 0xff) != 0)
		{
			err = USBD_IOERROR;
			goto done;
		}
		totlen = min(len, USB_HUB_DESCRIPTOR_SIZE);
		usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
			     &uhci_hubd_piix, totlen);
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
		if(index == 1)
			port = UHCI_PORTSC1;
		else if(index == 2)
			port = UHCI_PORTSC2;
		else
		{
			err = USBD_IOERROR;
			goto done;
		}
		if(len < 4)
		{
			err = USBD_IOERROR;
			goto done;
		}
		x = UREAD2(sc, port);
		status = change = 0;
		if(x & UHCI_PORTSC_CCS)
			status |= UPS_CURRENT_CONNECT_STATUS;
		if(x & UHCI_PORTSC_CSC)
			change |= UPS_C_CONNECT_STATUS;
		if(x & UHCI_PORTSC_PE)
			status |= UPS_PORT_ENABLED;
		if(x & UHCI_PORTSC_POEDC)
			change |= UPS_C_PORT_ENABLED;
		if(x & UHCI_PORTSC_OCI)
			status |= UPS_OVERCURRENT_INDICATOR;
		if(x & UHCI_PORTSC_OCIC)
			change |= UPS_C_OVERCURRENT_INDICATOR;
		if(x & UHCI_PORTSC_SUSP)
			status |= UPS_SUSPEND;
		if(x & UHCI_PORTSC_LSDA)
			status |= UPS_LOW_SPEED;
		status |= UPS_PORT_POWER;
		if(sc->sc_isreset)
			change |= UPS_C_PORT_RESET;
		USETW(u.ps.wPortStatus, status);
		USETW(u.ps.wPortChange, change);
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
		if(index == 1)
			port = UHCI_PORTSC1;
		else if(index == 2)
			port = UHCI_PORTSC2;
		else
		{
			err = USBD_IOERROR;
			goto done;
		}
		switch(value) {
		case UHF_PORT_ENABLE:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_PE);
			break;
		case UHF_PORT_SUSPEND:
			x = URWMASK(UREAD2(sc, port));
			UWRITE2(sc, port, x | UHCI_PORTSC_SUSP);
			break;
		case UHF_PORT_RESET:
			err = uhci_portreset(sc, index);
			goto done;
		case UHF_PORT_POWER:
			/* pretend we turned on power */
			err = USBD_NORMAL_COMPLETION;
			goto done;
		case UHF_C_PORT_CONNECTION:
		case UHF_C_PORT_ENABLE:
		case UHF_C_PORT_OVER_CURRENT:
		case UHF_PORT_CONNECTION:
		case UHF_PORT_OVER_CURRENT:
		case UHF_PORT_LOW_SPEED:
		case UHF_C_PORT_SUSPEND:
		case UHF_C_PORT_RESET:
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
	uhci_device_done(xfer,err);

	/* call callback recursively */
	__usbd_callback(xfer);

	return;
}

static void
uhci_root_ctrl_start(struct usbd_xfer *xfer)
{
	/* not used */
	return;
}

struct usbd_pipe_methods uhci_root_ctrl_methods = 
{
  .open = uhci_root_ctrl_open,
  .close = uhci_root_ctrl_close,
  .enter = uhci_root_ctrl_enter,
  .start = uhci_root_ctrl_start,
  .copy_in = usbd_std_ctrl_copy_in,
  .copy_out = usbd_std_ctrl_copy_out,
};

/*---------------------------------------------------------------------------*
 * uhci root interrupt support
 *---------------------------------------------------------------------------*/
static void
uhci_root_intr_open(struct usbd_xfer *xfer)
{
	return;
}

static void
uhci_root_intr_close(struct usbd_xfer *xfer)
{
	uhci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
uhci_root_intr_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer 
	 * (so that it can be aborted through pipe abort)
	 */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
uhci_root_intr_check(struct usbd_xfer *xfer);

static void
uhci_root_intr_start(struct usbd_xfer *xfer)
{
	DPRINTFN(3, ("xfer=%p len=%d\n",
		     xfer, xfer->length));

	__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->interval),
			(void *)(void *)uhci_root_intr_check, xfer);
	return;
}

/*
 * this routine is executed periodically and simulates interrupts
 * from the root controller interrupt pipe for port status change
 */
static void
uhci_root_intr_check(struct usbd_xfer *xfer)
{
	struct usbd_callback_info info[1];
	uhci_softc_t *sc = xfer->usb_sc;
	u_int8_t buf[1];

	DPRINTFN(20,("\n"));

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	buf[0] = 0;

	if(UREAD2(sc, UHCI_PORTSC1) & (UHCI_PORTSC_CSC|UHCI_PORTSC_OCIC))
	{
	    buf[0] |= 1<<1;
	}

	if(UREAD2(sc, UHCI_PORTSC2) & (UHCI_PORTSC_CSC|UHCI_PORTSC_OCIC))
	{
	    buf[0] |= 1<<2;
	}

	if((buf[0] == 0) || !(UREAD2(sc, UHCI_CMD) & UHCI_CMD_RS))
	{
	    /* no change or controller not running,
	     * try again in a while
	     */
	    uhci_root_intr_start(xfer);

	    mtx_unlock(&sc->sc_bus.mtx);
	}
	else
	{
	    if (xfer->length) {
	        xfer->actlen = 1;
		usbd_copy_in(&(xfer->buf_data), 0, buf, 1);
	    } else {
	        xfer->actlen = 0;
	    }

	    /* transfer is transferred */
	    uhci_device_done(xfer, USBD_NORMAL_COMPLETION);

	    /* queue callback */
	    info[0].xfer = xfer;
	    info[0].refcount = xfer->usb_refcount;

	    xfer->usb_root->memory_refcount++;

	    mtx_unlock(&sc->sc_bus.mtx);

	    usbd_do_callback(&info[0],&info[1]);
	}
	return;
}

struct usbd_pipe_methods uhci_root_intr_methods = 
{
  .open = uhci_root_intr_open,
  .close = uhci_root_intr_close,
  .enter = uhci_root_intr_enter,
  .start = uhci_root_intr_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

static usbd_status
uhci_xfer_setup(struct usbd_device *udev,
		u_int8_t iface_index,
		struct usbd_xfer **pxfer,
		const struct usbd_config *setup_start,
		const struct usbd_config *setup_end)
{
	struct usbd_page_info page_info;
	struct usbd_xfer dummy;
	uhci_softc_t *sc = UHCI_BUS2SC(udev->bus);
	const struct usbd_config *setup;
	struct usbd_memory_info *info;
	struct usbd_page *page_ptr;
	struct usbd_xfer *xfer;
	u_int32_t size[2];
	u_int32_t total_size[2];
	u_int32_t ntd;
	u_int32_t nqh;
	u_int32_t nfixup;
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

	if(buf)
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
		 * compute ntd and nqh
		 */
		if((xfer->pipe->methods == &uhci_device_ctrl_methods) ||
		   (xfer->pipe->methods == &uhci_device_bulk_methods) ||
		   (xfer->pipe->methods == &uhci_device_intr_methods))
		{
			nqh = 1;
			ntd = (1+ /* SETUP */ 1+ /* STATUS */
			       1  /* SHORTPKT */) +
			  (xfer->length / xfer->max_packet_size) /* DATA */;
		}

		if(xfer->pipe->methods == &uhci_device_isoc_methods)
		{
			if(xfer->nframes >= UHCI_VFRAMELIST_COUNT)
			{
				error = USBD_INVAL;
				DPRINTF(("isochronous frame-limit "
					 "exceeded by 0x%x frames; "
					 "endpoint 0x%02x\n",
					 setup->frames - UHCI_VFRAMELIST_COUNT,
					 setup->endpoint));
				goto done;
			}
			if(xfer->nframes == 0)
			{
				error = USBD_ZERO_FRAMES_IN_ISOC_MODE;
				DPRINTF(("frames == 0 in isochronous mode; "
					 "endpoint 0x%02x\n", setup->endpoint));
				goto done;
			}
			ntd = xfer->nframes;
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

	  /*
	   * NOTE: the UHCI controller requires that
	   * every packet must be continuous on
	   * the same USB memory page !
	   */

	  if (xfer->pipe->methods == &uhci_device_isoc_methods) {

	      size[1] += xfer->length;
	      nfixup = (xfer->length / USB_PAGE_SIZE) + 1;

	  } else {

	      n = xfer->length;
	      nfixup = 0;

	      if (xfer->pipe->methods == &uhci_device_ctrl_methods) {
		  if (n >= sizeof(usb_device_request_t)) {

		      size[1] += usbd_page_fit_obj(page_ptr, size[1], 
						   sizeof(usb_device_request_t));
		      size[1] += sizeof(usb_device_request_t);

		      n -= sizeof(usb_device_request_t);
		  }
	      }

	      while (n >= xfer->max_packet_size) {

		  size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					       xfer->max_packet_size);
		  size[1] += xfer->max_packet_size;

		  n -= xfer->max_packet_size;
	      }

	      size[1] += usbd_page_fit_obj(page_ptr, size[1], n);
	      size[1] += n;
	  }

	  usbd_page_set_end(&(xfer->buf_data), page_ptr, size[1]);

	  /* align data to 8 byte boundary */
	  size[1] += ((-size[1]) & (USB_HOST_ALIGN-1));

   	  usbd_page_set_start(&(xfer->buf_fixup), page_ptr, size[1]);

	  for (n = 0; n < nfixup; n++) {

	      size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					   xfer->max_frame_size);

	      size[1] += xfer->max_frame_size;
	  }

	  usbd_page_set_end(&(xfer->buf_fixup), page_ptr, size[1]);

	  size[1] += ((-size[1]) & (UHCI_TD_ALIGN-1)); /* align data */

	  last_obj = NULL;

	  for(n = 0;
	      n < ntd;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					 sizeof(uhci_td_t));
	    if(buf)
	    {
		register uhci_td_t *td;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_dma_exit(page_info.page);

		td = page_info.buffer;

		/* init TD */
		if((xfer->pipe->methods == &uhci_device_bulk_methods) ||
		   (xfer->pipe->methods == &uhci_device_ctrl_methods) ||
		   (xfer->pipe->methods == &uhci_device_intr_methods))
		{
		    /* set depth first bit */
		    td->td_self = htole32(page_info.physaddr|UHCI_PTR_TD|UHCI_PTR_VF);
		} else {
		    td->td_self = htole32(page_info.physaddr|UHCI_PTR_TD);
		}

		td->obj_next = last_obj;
		td->page = page_info.page;

		last_obj = td;

		usbd_page_dma_enter(page_info.page);
	    }
	    size[1] += sizeof(uhci_td_t);
	  }

 	  xfer->td_start = last_obj;

	  size[1] += ((-size[1]) & (UHCI_QH_ALIGN-1)); /* align data */

	  last_obj = NULL;

	  for(n = 0;
	      n < nqh;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1],
					 sizeof(uhci_qh_t));
	    if(buf)
	    {
		register uhci_qh_t *qh;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_dma_exit(page_info.page);

		qh = page_info.buffer;

		/* init QH */
		qh->qh_self = htole32(page_info.physaddr|UHCI_PTR_QH);
		qh->obj_next = last_obj;
		qh->page = page_info.page;

		last_obj = qh;

		usbd_page_dma_enter(page_info.page);
	    }
	    size[1] += sizeof(uhci_qh_t);
	  }
 	  xfer->qh_start = last_obj;
	}

	if(buf || error) {
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

	if(buf == NULL) {
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
uhci_pipe_init(struct usbd_device *udev, usb_endpoint_descriptor_t *edesc, 
               struct usbd_pipe *pipe)
{
	uhci_softc_t *sc = UHCI_BUS2SC(udev->bus);

	DPRINTFN(1, ("pipe=%p, addr=%d, endpt=%d (%d)\n",
		     pipe, udev->address,
		     edesc->bEndpointAddress, sc->sc_addr));

	if(udev->address == sc->sc_addr)
	{
		switch (edesc->bEndpointAddress)
		{
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &uhci_root_ctrl_methods;
			break;
		case UE_DIR_IN | UHCI_INTR_ENDPT:
			pipe->methods = &uhci_root_intr_methods;
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
			pipe->methods = &uhci_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &uhci_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			pipe->methods = &uhci_device_isoc_methods;
			break;
		case UE_BULK:
			pipe->methods = &uhci_device_bulk_methods;
			break;
		}
	}
	return;
}

struct usbd_bus_methods uhci_bus_methods = 
{
	.pipe_init  = uhci_pipe_init,
	.xfer_setup = uhci_xfer_setup,
	.do_poll    = uhci_do_poll,
};
