/*-
 * Copyright (c) 2004 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) and by Charles M. Hannum.
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
 * USB Enhanced Host Controller Driver, a.k.a. USB 2.0 controller.
 *
 * The EHCI 0.96 spec can be found at
 * http://developer.intel.com/technology/usb/download/ehci-r096.pdf
 * The EHCI 1.0 spec can be found at
 * http://developer.intel.com/technology/usb/download/ehci-r10.pdf
 * and the USB 2.0 spec at
 * http://www.usb.org/developers/docs/usb_20.zip
 *
 */

/*
 * TODO:
 * 1) command failures are not recovered correctly
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ehci.c,v 1.52 2006/10/19 01:15:58 iedowse Exp $");

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
#include <dev/usb/ehci.h>

#define MS_TO_TICKS(ms) (((ms) * hz) / 1000)
#define EHCI_BUS2SC(bus) ((ehci_softc_t *)(((u_int8_t *)(bus)) - \
   POINTER_TO_UNSIGNED(&(((ehci_softc_t *)0)->sc_bus))))

#ifdef USB_DEBUG
#define DPRINTF(x)	{ if (ehcidebug) { printf("%s: ", __FUNCTION__); printf x ; } }
#define DPRINTFN(n,x)	{ if (ehcidebug > (n)) { printf("%s: ", __FUNCTION__); printf x ; } }
int ehcidebug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ehci, CTLFLAG_RW, 0, "USB ehci");
SYSCTL_INT(_hw_usb_ehci, OID_AUTO, debug, CTLFLAG_RW,
	   &ehcidebug, 0, "ehci debug level");

static void
ehci_dump_regs(ehci_softc_t *sc);
static void
ehci_dump_sqh(ehci_qh_t *sqh);
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif

#define EHCI_INTR_ENDPT 1

extern struct usbd_bus_methods ehci_bus_methods;
extern struct usbd_pipe_methods ehci_device_bulk_methods;
extern struct usbd_pipe_methods ehci_device_ctrl_methods;
extern struct usbd_pipe_methods ehci_device_intr_methods;
extern struct usbd_pipe_methods ehci_device_isoc_fs_methods;
extern struct usbd_pipe_methods ehci_device_isoc_hs_methods;
extern struct usbd_pipe_methods ehci_root_ctrl_methods;
extern struct usbd_pipe_methods ehci_root_intr_methods;

#define SC_HW_PHYSADDR(sc,what) \
  ((sc)->sc_hw_page.physaddr +	\
   POINTER_TO_UNSIGNED(&(((struct ehci_hw_softc *)0)->what)))

usbd_status
ehci_init(ehci_softc_t *sc)
{
	struct ehci_hw_softc *hw_ptr;
	u_int32_t version, sparams, cparams, hcr;
	u_int i;
	u_int16_t x;
	u_int16_t y;
	u_int16_t bit;
	usbd_status err = 0;

	mtx_lock(&sc->sc_bus.mtx);

	hw_ptr = sc->sc_hw_ptr;

	DPRINTF(("start\n"));

	LIST_INIT(&sc->sc_interrupt_list_head);

	__callout_init_mtx(&sc->sc_tmo_pcd, &sc->sc_bus.mtx, 
			   CALLOUT_RETURNUNLOCKED);

#ifdef USB_DEBUG
	if(ehcidebug > 2)
	{
		ehci_dump_regs(sc);
	}
#endif

	sc->sc_offs = EREAD1(sc, EHCI_CAPLENGTH);

	version = EREAD2(sc, EHCI_HCIVERSION);
	device_printf(sc->sc_bus.bdev, "EHCI version %x.%x\n",
		      version >> 8, version & 0xff);

	sparams = EREAD4(sc, EHCI_HCSPARAMS);
	DPRINTF(("sparams=0x%x\n", sparams));

	sc->sc_noport = EHCI_HCS_N_PORTS(sparams);
	cparams = EREAD4(sc, EHCI_HCCPARAMS);
	DPRINTF(("cparams=0x%x\n", cparams));

	if(EHCI_HCC_64BIT(cparams))
	{
		DPRINTF(("HCC uses 64-bit structures\n"));

		/* MUST clear segment register if 64 bit capable */
		EWRITE4(sc, EHCI_CTRLDSSEGMENT, 0);
	}

	sc->sc_bus.usbrev = USBREV_2_0;

	/* Reset the controller */
	DPRINTF(("%s: resetting\n", device_get_nameunit(sc->sc_bus.bdev)));
	EOWRITE4(sc, EHCI_USBCMD, 0);	/* Halt controller */
	DELAY(1000*1);
	EOWRITE4(sc, EHCI_USBCMD, EHCI_CMD_HCRESET);
	for(i = 0; i < 100; i++)
	{
		DELAY(1000*1);
		hcr = EOREAD4(sc, EHCI_USBCMD) & EHCI_CMD_HCRESET;
		if(!hcr)
		{
			break;
		}
	}

	if (hcr)
	{
		device_printf(sc->sc_bus.bdev, "reset timeout\n");
		err = USBD_IOERROR;
		goto done;
	}

	/* use current frame-list-size selection
	 * 0: 1024*4 bytes
	 * 1:  512*4 bytes
	 * 2:  256*4 bytes
	 * 3:      unknown
	 */
	if(EHCI_CMD_FLS(EOREAD4(sc, EHCI_USBCMD)) == 3)
	{
		device_printf(sc->sc_bus.bdev, "invalid frame-list-size\n");
		err = USBD_IOERROR;
		goto done;
	}

	/* set up the bus struct */
	sc->sc_bus.methods = &ehci_bus_methods;

	sc->sc_eintrs = EHCI_NORMAL_INTRS;

	usbd_page_dma_exit(&(sc->sc_hw_page));

	for(i = 0; i < EHCI_VIRTUAL_FRAMELIST_COUNT; i++)
	{
		hw_ptr->intr_start[i].qh_self = 
		  htole32(SC_HW_PHYSADDR(sc,intr_start[i])|EHCI_LINK_QH);

		hw_ptr->intr_start[i].qh_endp = 
		  htole32(EHCI_QH_SET_EPS(EHCI_QH_SPEED_HIGH));
		hw_ptr->intr_start[i].qh_endphub = 
		  htole32(EHCI_QH_SET_MULT(1));
		hw_ptr->intr_start[i].qh_curqtd = 0;

		hw_ptr->intr_start[i].qh_qtd.qtd_next = 
		  htole32(EHCI_LINK_TERMINATE);
		hw_ptr->intr_start[i].qh_qtd.qtd_altnext = 
		  htole32(EHCI_LINK_TERMINATE);
		hw_ptr->intr_start[i].qh_qtd.qtd_status = 
		  htole32(EHCI_QTD_HALTED);

		hw_ptr->intr_start[i].page =
		  &(sc->sc_hw_page);

		sc->sc_intr_p_last[i] = 
		  &(hw_ptr->intr_start[i]);
	}

	/*
	 * the QHs are arranged to give poll intervals that are
	 * powers of 2 times 1ms
	 */
	bit = EHCI_VIRTUAL_FRAMELIST_COUNT/2;
	while(bit)
	{
		x = bit;
		while(x & bit)
		{
			y = (x ^ bit)|(bit/2);

			/* the next QH has half the
			 * poll interval
			 */
			hw_ptr->intr_start[x].qh_link =
			  hw_ptr->intr_start[y].qh_self;
			x++;
		}
		bit >>= 1;
	}

	/* the last (1ms) QH terminates */
	hw_ptr->intr_start[0].qh_link = htole32(EHCI_LINK_TERMINATE);

	for(i = 0; i < EHCI_VIRTUAL_FRAMELIST_COUNT; i++)
	{
		/* initialize full speed isochronous */

		hw_ptr->isoc_fs_start[i].sitd_self = 
		  htole32(SC_HW_PHYSADDR(sc,isoc_fs_start[i])|EHCI_LINK_SITD);

		hw_ptr->isoc_fs_start[i].sitd_back = 
		  htole32(EHCI_LINK_TERMINATE);

		hw_ptr->isoc_fs_start[i].sitd_next = 
		  hw_ptr->intr_start[i|(EHCI_VIRTUAL_FRAMELIST_COUNT/2)].qh_self;

		hw_ptr->isoc_fs_start[i].page =
		  &(sc->sc_hw_page);

		sc->sc_isoc_fs_p_last[i] = 
		  &(hw_ptr->isoc_fs_start[i]);


		/* initialize high speed isochronous */

		hw_ptr->isoc_hs_start[i].itd_self = 
		  htole32(SC_HW_PHYSADDR(sc,isoc_hs_start[i])|EHCI_LINK_ITD);

		hw_ptr->isoc_hs_start[i].itd_next = 
		  hw_ptr->isoc_fs_start[i].sitd_self;

		hw_ptr->isoc_hs_start[i].page =
		  &(sc->sc_hw_page);

		sc->sc_isoc_hs_p_last[i] = 
		  &(hw_ptr->isoc_hs_start[i]);
	}

	/*
	 * execution order:
	 * pframes -> high speed isochronous -> 
	 *    full speed isochronous -> interrupt QH's
	 */
	for(i = 0; i < EHCI_FRAMELIST_COUNT; i++)
	{
		hw_ptr->pframes[i] = hw_ptr->isoc_hs_start
		  [i & (EHCI_VIRTUAL_FRAMELIST_COUNT-1)].itd_self;
	}

	/* setup sync list pointer */
	EOWRITE4(sc, EHCI_PERIODICLISTBASE, SC_HW_PHYSADDR(sc,pframes[0]));


	/* init dummy QH that starts the async list */

	hw_ptr->async_start.qh_self = 
	  htole32(SC_HW_PHYSADDR(sc,async_start)|EHCI_LINK_QH);

	/* fill the QH */
	hw_ptr->async_start.qh_endp =
	    htole32(EHCI_QH_SET_EPS(EHCI_QH_SPEED_HIGH) | EHCI_QH_HRECL);
	hw_ptr->async_start.qh_endphub = htole32(EHCI_QH_SET_MULT(1));
	hw_ptr->async_start.qh_link = hw_ptr->async_start.qh_self;
	hw_ptr->async_start.qh_curqtd = 0;

	/* fill the overlay qTD */
	hw_ptr->async_start.qh_qtd.qtd_next = htole32(EHCI_LINK_TERMINATE);
	hw_ptr->async_start.qh_qtd.qtd_altnext = htole32(EHCI_LINK_TERMINATE);
	hw_ptr->async_start.qh_qtd.qtd_status = htole32(EHCI_QTD_HALTED);

	/* fill the page pointer */
	hw_ptr->async_start.page =
	  &(sc->sc_hw_page);

	sc->sc_async_p_last = 
	  &(hw_ptr->async_start);

	usbd_page_dma_enter(&(sc->sc_hw_page));

#ifdef USB_DEBUG
	if(ehcidebug)
	{
		ehci_dump_sqh(&(hw_ptr->async_start));
	}
#endif

	/* setup async list pointer */
	EOWRITE4(sc, EHCI_ASYNCLISTADDR, SC_HW_PHYSADDR(sc,async_start)|EHCI_LINK_QH);


	/* enable interrupts */
	EOWRITE4(sc, EHCI_USBINTR, sc->sc_eintrs);

	/* turn on controller */
	EOWRITE4(sc, EHCI_USBCMD,
		 EHCI_CMD_ITC_1 | /* 1 microframes interrupt delay */
		 (EOREAD4(sc, EHCI_USBCMD) & EHCI_CMD_FLS_M) |
		 EHCI_CMD_ASE |
		 EHCI_CMD_PSE |
		 EHCI_CMD_RS);

	/* Take over port ownership */
	EOWRITE4(sc, EHCI_CONFIGFLAG, EHCI_CONF_CF);

	for(i = 0; i < 100; i++)
	{
		DELAY(1000*1);
		hcr = EOREAD4(sc, EHCI_USBSTS) & EHCI_STS_HCH;
		if(!hcr)
		{
			break;
		}
	}
	if (hcr)
	{
		device_printf(sc->sc_bus.bdev, "run timeout\n");
		err = USBD_IOERROR;
		goto done;
	}

 done:
	mtx_unlock(&sc->sc_bus.mtx);
	return (err);
}

/*
 * shut down the controller when the system is going down
 */
void
ehci_detach(struct ehci_softc *sc)
{
	mtx_lock(&sc->sc_bus.mtx);

	__callout_stop(&sc->sc_tmo_pcd);

	EOWRITE4(sc, EHCI_USBINTR, sc->sc_eintrs);
	EOWRITE4(sc, EHCI_USBCMD, 0);
	EOWRITE4(sc, EHCI_USBCMD, EHCI_CMD_HCRESET);

	DELAY(1000*300); /* XXX let stray task complete */

	mtx_unlock(&sc->sc_bus.mtx);

	__callout_drain(&(sc->sc_tmo_pcd));

	return;
}

void
ehci_suspend(struct ehci_softc *sc)
{
	u_int32_t cmd;
	u_int32_t hcr;
	u_int8_t i;

	mtx_lock(&sc->sc_bus.mtx);

	for(i = 1; i <= sc->sc_noport; i++)
	{
		cmd = EOREAD4(sc, EHCI_PORTSC(i));
		if(((cmd & EHCI_PS_PO) == 0) &&
		   ((cmd & EHCI_PS_PE) == EHCI_PS_PE))
		{
			EOWRITE4(sc, EHCI_PORTSC(i),
				 cmd | EHCI_PS_SUSP);
		}
	}

	sc->sc_cmd = EOREAD4(sc, EHCI_USBCMD);

	cmd = sc->sc_cmd & ~(EHCI_CMD_ASE | EHCI_CMD_PSE);
	EOWRITE4(sc, EHCI_USBCMD, cmd);

	for(i = 0; i < 100; i++)
	{
		hcr = EOREAD4(sc, EHCI_USBSTS) &
		  (EHCI_STS_ASS | EHCI_STS_PSS);

		if(hcr == 0)
		{
			break;
		}

		DELAY(1000*1);
	}

	if(hcr != 0)
	{
		device_printf(sc->sc_bus.bdev, "reset timeout\n");
	}

	cmd &= ~EHCI_CMD_RS;
	EOWRITE4(sc, EHCI_USBCMD, cmd);

	for (i = 0; i < 100; i++)
	{
		hcr = EOREAD4(sc, EHCI_USBSTS) & EHCI_STS_HCH;
		if (hcr == EHCI_STS_HCH)
		{
			break;
		}

		DELAY(1000*1);
	}

	if (hcr != EHCI_STS_HCH)
	{
		device_printf(sc->sc_bus.bdev, 
			      "config timeout\n");
	}

	mtx_unlock(&sc->sc_bus.mtx);
 	return;
}

void
ehci_resume(struct ehci_softc *sc)
{
	u_int32_t cmd;
	u_int32_t hcr;
	u_int8_t i;

	mtx_lock(&sc->sc_bus.mtx);

	/* restore things in case the bios doesn't */
	EOWRITE4(sc, EHCI_CTRLDSSEGMENT, 0);
	EOWRITE4(sc, EHCI_PERIODICLISTBASE, SC_HW_PHYSADDR(sc,pframes[0]));
	EOWRITE4(sc, EHCI_ASYNCLISTADDR, SC_HW_PHYSADDR(sc,async_start)|EHCI_LINK_QH);

	EOWRITE4(sc, EHCI_USBINTR, sc->sc_eintrs);

	hcr = 0;
	for(i = 1; i <= sc->sc_noport; i++)
	{
	    cmd = EOREAD4(sc, EHCI_PORTSC(i));
	    if(((cmd & EHCI_PS_PO) == 0) &&
	       ((cmd & EHCI_PS_SUSP) == EHCI_PS_SUSP))
	    {
	        EOWRITE4(sc, EHCI_PORTSC(i),
			 cmd | EHCI_PS_FPR);
		hcr = 1;
	    }
	}

	if(hcr)
	{
		DELAY(1000*USB_RESUME_WAIT);

		for(i = 1; i <= sc->sc_noport; i++)
		{
			cmd = EOREAD4(sc, EHCI_PORTSC(i));
			if(((cmd & EHCI_PS_PO) == 0) &&
			   ((cmd & EHCI_PS_SUSP) == EHCI_PS_SUSP))
			{
				EOWRITE4(sc, EHCI_PORTSC(i),
					 cmd & ~EHCI_PS_FPR);
			}
		}
	}

	EOWRITE4(sc, EHCI_USBCMD, sc->sc_cmd);

	for(i = 0; i < 100; i++)
	{
		hcr = EOREAD4(sc, EHCI_USBSTS) & EHCI_STS_HCH;
		if(hcr != EHCI_STS_HCH)
		{
			break;
		}

		DELAY(1000*1);
	}
	if(hcr == EHCI_STS_HCH)
	{
		device_printf(sc->sc_bus.bdev, "config timeout\n");
	}

	mtx_unlock(&sc->sc_bus.mtx);

	DELAY(1000*USB_RESUME_WAIT);
	return;
}

void
ehci_shutdown(ehci_softc_t *sc)
{
	DPRINTF(("stopping the HC\n"));

	mtx_lock(&sc->sc_bus.mtx);

	EOWRITE4(sc, EHCI_USBCMD, 0);	/* Halt controller */
	EOWRITE4(sc, EHCI_USBCMD, EHCI_CMD_HCRESET);

	mtx_unlock(&sc->sc_bus.mtx);
}

#ifdef USB_DEBUG
static void
ehci_dump_regs(ehci_softc_t *sc)
{
	u_int32_t i;

	i = EOREAD4(sc, EHCI_USBCMD);
	printf("cmd=0x%08x\n", i);

	if(i &  EHCI_CMD_ITC_1)
	  printf(" EHCI_CMD_ITC_1\n");
	if(i &  EHCI_CMD_ITC_2)
	  printf(" EHCI_CMD_ITC_2\n");
	if(i &  EHCI_CMD_ITC_4)
	  printf(" EHCI_CMD_ITC_4\n");
	if(i &  EHCI_CMD_ITC_8)
	  printf(" EHCI_CMD_ITC_8\n");
	if(i &  EHCI_CMD_ITC_16)
	  printf(" EHCI_CMD_ITC_16\n");
	if(i &  EHCI_CMD_ITC_32)
	  printf(" EHCI_CMD_ITC_32\n");
	if(i &  EHCI_CMD_ITC_64)
	  printf(" EHCI_CMD_ITC_64\n");
	if(i & EHCI_CMD_ASPME)
	  printf(" EHCI_CMD_ASPME\n");
	if(i & EHCI_CMD_ASPMC)
	  printf(" EHCI_CMD_ASPMC\n");
	if(i & EHCI_CMD_LHCR)
	  printf(" EHCI_CMD_LHCR\n");
	if(i & EHCI_CMD_IAAD)
	  printf(" EHCI_CMD_IAAD\n");
	if(i & EHCI_CMD_ASE)
	  printf(" EHCI_CMD_ASE\n");
	if(i & EHCI_CMD_PSE)
	  printf(" EHCI_CMD_PSE\n");
	if(i & EHCI_CMD_FLS_M)
	  printf(" EHCI_CMD_FLS_M\n");
	if(i & EHCI_CMD_HCRESET)
	  printf(" EHCI_CMD_HCRESET\n");
	if(i & EHCI_CMD_RS)
	  printf(" EHCI_CMD_RS\n");

	i = EOREAD4(sc, EHCI_USBSTS);

	printf("sts=0x%08x\n", i);

	if(i & EHCI_STS_ASS)
	  printf(" EHCI_STS_ASS\n");
	if(i & EHCI_STS_PSS)
	  printf(" EHCI_STS_PSS\n");
	if(i & EHCI_STS_REC)
	  printf(" EHCI_STS_REC\n");
	if(i & EHCI_STS_HCH)
	  printf(" EHCI_STS_HCH\n");
	if(i & EHCI_STS_IAA)
	  printf(" EHCI_STS_IAA\n");
	if(i & EHCI_STS_HSE)
	  printf(" EHCI_STS_HSE\n");
	if(i & EHCI_STS_FLR)
	  printf(" EHCI_STS_FLR\n");
	if(i & EHCI_STS_PCD)
	  printf(" EHCI_STS_PCD\n");
	if(i & EHCI_STS_ERRINT)
	  printf(" EHCI_STS_ERRINT\n");
	if(i & EHCI_STS_INT)
	  printf(" EHCI_STS_INT\n");

	printf("ien=0x%08x\n",
	       EOREAD4(sc, EHCI_USBINTR));
	printf("frindex=0x%08x ctrdsegm=0x%08x periodic=0x%08x async=0x%08x\n",
	       EOREAD4(sc, EHCI_FRINDEX),
	       EOREAD4(sc, EHCI_CTRLDSSEGMENT),
	       EOREAD4(sc, EHCI_PERIODICLISTBASE),
	       EOREAD4(sc, EHCI_ASYNCLISTADDR));
	for(i = 1; i <= sc->sc_noport; i++)
	{
		printf("port %d status=0x%08x\n", i,
		       EOREAD4(sc, EHCI_PORTSC(i)));
	}
	return;
}

static void
ehci_dump_link(u_int32_t link, int type)
{
	link = le32toh(link);
	printf("0x%08x", link);
	if (link & EHCI_LINK_TERMINATE)
		printf("<T>");
	else {
		printf("<");
		if (type) {
			switch (EHCI_LINK_TYPE(link)) {
			case EHCI_LINK_ITD: printf("ITD"); break;
			case EHCI_LINK_QH: printf("QH"); break;
			case EHCI_LINK_SITD: printf("SITD"); break;
			case EHCI_LINK_FSTN: printf("FSTN"); break;
			}
		}
		printf(">");
	}
	return;
}

static void
ehci_dump_qtd(ehci_qtd_t *qtd)
{
	u_int32_t s;

	printf("  next="); ehci_dump_link(qtd->qtd_next, 0);
	printf(" altnext="); ehci_dump_link(qtd->qtd_altnext, 0);
	printf("\n");
	s = le32toh(qtd->qtd_status);
	printf("  status=0x%08x: toggle=%d bytes=0x%x ioc=%d c_page=0x%x\n",
	       s, EHCI_QTD_GET_TOGGLE(s), EHCI_QTD_GET_BYTES(s),
	       EHCI_QTD_GET_IOC(s), EHCI_QTD_GET_C_PAGE(s));
	printf("    cerr=%d pid=%d stat=%s%s%s%s%s%s%s%s\n", 
	       EHCI_QTD_GET_CERR(s), EHCI_QTD_GET_PID(s), 
	       (s & EHCI_QTD_ACTIVE) ? "ACTIVE" : "NOT_ACTIVE",
	       (s & EHCI_QTD_HALTED) ? "-HALTED" : "",
	       (s & EHCI_QTD_BUFERR) ? "-BUFERR" : "",
	       (s & EHCI_QTD_BABBLE) ? "-BABBLE" : "",
	       (s & EHCI_QTD_XACTERR) ? "-XACTERR" : "",
	       (s & EHCI_QTD_MISSEDMICRO) ? "-MISSED" : "",
	       (s & EHCI_QTD_SPLITXSTATE) ? "-SPLIT" : "",
	       (s & EHCI_QTD_PINGSTATE) ? "-PING" : "");

	for(s = 0; s < 5; s++)
	{
		printf("  buffer[%d]=0x%08x\n", s, 
		       le32toh(qtd->qtd_buffer[s]));
	}
	for(s = 0; s < 5; s++)
	{
		printf("  buffer_hi[%d]=0x%08x\n", s, 
		       le32toh(qtd->qtd_buffer_hi[s]));
	}
	return;
}

static uint8_t
ehci_dump_sqtd(ehci_qtd_t *sqtd)
{
	uint8_t temp;
	usbd_page_dma_exit(sqtd->page);
	printf("QTD(%p) at 0x%08x:\n", sqtd, le32toh(sqtd->qtd_self));
	ehci_dump_qtd(sqtd);
	temp = (sqtd->qtd_next & htole32(EHCI_LINK_TERMINATE)) ? 1 : 0;
	usbd_page_dma_enter(sqtd->page);
	return temp;
}

static void
ehci_dump_sqtds(ehci_qtd_t *sqtd)
{
	u_int16_t i;
	uint8_t stop;

	stop = 0;
	for(i = 0; sqtd && (i < 20) && !stop; sqtd = sqtd->obj_next, i++)
	{
		stop = ehci_dump_sqtd(sqtd);
	}
	if(sqtd)
	{
		printf("dump aborted, too many TDs\n");
	}
	return;
}

static void
ehci_dump_sqh(ehci_qh_t *qh)
{
	u_int32_t endp, endphub;

	usbd_page_dma_exit(qh->page);
	printf("QH(%p) at 0x%08x:\n", qh, le32toh(qh->qh_self) & ~0x1F);
	printf("  link="); ehci_dump_link(qh->qh_link, 1); printf("\n");
	endp = le32toh(qh->qh_endp);
	printf("  endp=0x%08x\n", endp);
	printf("    addr=0x%02x inact=%d endpt=%d eps=%d dtc=%d hrecl=%d\n",
	       EHCI_QH_GET_ADDR(endp), EHCI_QH_GET_INACT(endp),
	       EHCI_QH_GET_ENDPT(endp),  EHCI_QH_GET_EPS(endp),
	       EHCI_QH_GET_DTC(endp), EHCI_QH_GET_HRECL(endp));
	printf("    mpl=0x%x ctl=%d nrl=%d\n",
	       EHCI_QH_GET_MPL(endp), EHCI_QH_GET_CTL(endp),
	       EHCI_QH_GET_NRL(endp));
	endphub = le32toh(qh->qh_endphub);
	printf("  endphub=0x%08x\n", endphub);
	printf("    smask=0x%02x cmask=0x%02x huba=0x%02x port=%d mult=%d\n",
	       EHCI_QH_GET_SMASK(endphub), EHCI_QH_GET_CMASK(endphub),
	       EHCI_QH_GET_HUBA(endphub), EHCI_QH_GET_PORT(endphub),
	       EHCI_QH_GET_MULT(endphub));
	printf("  curqtd="); ehci_dump_link(qh->qh_curqtd, 0); printf("\n");
	printf("Overlay qTD:\n");
	ehci_dump_qtd((void *)&qh->qh_qtd);
	usbd_page_dma_enter(qh->page);
	return;
}

static void
ehci_dump_sitd(ehci_sitd_t *sitd)
{
	usbd_page_dma_exit(sitd->page);
	printf("SITD(%p) at 0x%08x\n", sitd, le32toh(sitd->sitd_self) & ~0x1F);
	printf(" next=0x%08x\n", le32toh(sitd->sitd_next));
	printf(" portaddr=0x%08x dir=%s addr=%d endpt=0x%x port=0x%x huba=0x%x\n", 
	       le32toh(sitd->sitd_portaddr),
	       (sitd->sitd_portaddr & htole32(EHCI_SITD_SET_DIR_IN)) 
	       ? "in" : "out" , 
	       EHCI_SITD_GET_ADDR(le32toh(sitd->sitd_portaddr)),
	       EHCI_SITD_GET_ENDPT(le32toh(sitd->sitd_portaddr)), 
	       EHCI_SITD_GET_PORT(le32toh(sitd->sitd_portaddr)), 
	       EHCI_SITD_GET_HUBA(le32toh(sitd->sitd_portaddr)));
	printf(" mask=0x%08x\n", le32toh(sitd->sitd_mask));
	printf(" status=0x%08x <%s> len=0x%x\n", le32toh(sitd->sitd_status),
	       (sitd->sitd_status & htole32(EHCI_SITD_ACTIVE)) ? "ACTIVE" : "",
	       EHCI_SITD_GET_LEN(le32toh(sitd->sitd_status)));
	printf(" back=0x%08x, bp=0x%08x,0x%08x,0x%08x,0x%08x\n",
		le32toh(sitd->sitd_back),
		le32toh(sitd->sitd_bp[0]),
		le32toh(sitd->sitd_bp[1]),
		le32toh(sitd->sitd_bp_hi[0]),
		le32toh(sitd->sitd_bp_hi[1]));
	usbd_page_dma_enter(sitd->page);
	return;
}

static void
ehci_dump_itd(ehci_itd_t *itd)
{
	usbd_page_dma_exit(itd->page);
	printf("ITD(%p) at 0x%08x\n", itd, le32toh(itd->itd_self) & ~0x1F);
	printf(" next=0x%08x\n", le32toh(itd->itd_next));
	printf(" status[0]=0x%08x; <%s>\n", le32toh(itd->itd_status[0]),
	       (itd->itd_status[0] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[1]=0x%08x; <%s>\n", le32toh(itd->itd_status[1]),
	       (itd->itd_status[1] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[2]=0x%08x; <%s>\n", le32toh(itd->itd_status[2]),
	       (itd->itd_status[2] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[3]=0x%08x; <%s>\n", le32toh(itd->itd_status[3]),
	       (itd->itd_status[3] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[4]=0x%08x; <%s>\n", le32toh(itd->itd_status[4]),
	       (itd->itd_status[4] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[5]=0x%08x; <%s>\n", le32toh(itd->itd_status[5]),
	       (itd->itd_status[5] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[6]=0x%08x; <%s>\n", le32toh(itd->itd_status[6]),
	       (itd->itd_status[6] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" status[7]=0x%08x; <%s>\n", le32toh(itd->itd_status[7]),
	       (itd->itd_status[7] & htole32(EHCI_ITD_ACTIVE)) ? "ACTIVE" : "");
	printf(" bp[0]=0x%08x\n", le32toh(itd->itd_bp[0]));
	printf("  addr=0x%02x; endpt=0x%01x\n",
	       EHCI_ITD_GET_ADDR(le32toh(itd->itd_bp[0])),
	       EHCI_ITD_GET_ENDPT(le32toh(itd->itd_bp[0])));
	printf(" bp[1]=0x%08x\n", le32toh(itd->itd_bp[1]));
	printf(" dir=%s; mpl=0x%02x\n",
	       (le32toh(itd->itd_bp[1]) & EHCI_ITD_SET_DIR_IN) ? "in" : "out",
	       EHCI_ITD_GET_MPL(le32toh(itd->itd_bp[1])));
	printf(" bp[2..6]=0x%08x,0x%08x,0x%08x,0x%08x,0x%08x\n",
		le32toh(itd->itd_bp[2]),
		le32toh(itd->itd_bp[3]),
		le32toh(itd->itd_bp[4]),
		le32toh(itd->itd_bp[5]),
		le32toh(itd->itd_bp[6]));
	printf(" bp_hi=0x%08x,0x%08x,0x%08x,0x%08x,\n"
	       "       0x%08x,0x%08x,0x%08x\n",
		le32toh(itd->itd_bp_hi[0]),
		le32toh(itd->itd_bp_hi[1]),
		le32toh(itd->itd_bp_hi[2]),
		le32toh(itd->itd_bp_hi[3]),
		le32toh(itd->itd_bp_hi[4]),
		le32toh(itd->itd_bp_hi[5]),
		le32toh(itd->itd_bp_hi[6]));
	usbd_page_dma_enter(itd->page);
	return;
}

static void
ehci_dump_isoc(ehci_softc_t *sc)
{
	ehci_itd_t *itd;
	ehci_sitd_t *sitd;
	u_int16_t max = 1000;
	u_int16_t pos;

	pos = (EOREAD4(sc, EHCI_FRINDEX) / 8) & 
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	printf("%s: isochronous dump from frame 0x%03x:\n",
	       __FUNCTION__, pos);

	itd = sc->sc_isoc_hs_p_last[pos];
	sitd = sc->sc_isoc_fs_p_last[pos];

	while(itd && max && max--)
	{
		ehci_dump_itd(itd);
		itd = itd->prev;
	}

	while(sitd && max && max--)
	{
		ehci_dump_sitd(sitd);
		sitd = sitd->prev;
	}
	return;
}
#endif

#define EHCI_APPEND_FS_TD(std,last) (last) = _ehci_append_fs_td(std,last)
static ehci_sitd_t *
_ehci_append_fs_td(ehci_sitd_t *std, ehci_sitd_t *last)
{
	DPRINTFN(10, ("%p to %p\n", std, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(std->page);

	std->next = last->next;
	std->sitd_next = last->sitd_next;

	std->prev = last;

	usbd_page_dma_enter(std->page);
	usbd_page_dma_exit(last->page);

	/* the last->next->prev is never followed:
	 * std->next->prev = std;
	 */
	last->next = std;
	last->sitd_next = std->sitd_self;

	usbd_page_dma_enter(last->page);

	return(std);
}

#define EHCI_APPEND_HS_TD(std,last) (last) = _ehci_append_hs_td(std,last)
static ehci_itd_t *
_ehci_append_hs_td(ehci_itd_t *std, ehci_itd_t *last)
{
	DPRINTFN(10, ("%p to %p\n", std, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(std->page);

	std->next = last->next;
	std->itd_next = last->itd_next;

	std->prev = last;

	usbd_page_dma_enter(std->page);
	usbd_page_dma_exit(last->page);

	/* the last->next->prev is never followed:
	 * std->next->prev = std;
	 */
	last->next = std;
	last->itd_next = std->itd_self;

	usbd_page_dma_enter(last->page);

	return(std);
}

#define EHCI_APPEND_QH(sqh,last) (last) = _ehci_append_qh(sqh,last)
static ehci_qh_t *
_ehci_append_qh(ehci_qh_t *sqh, ehci_qh_t *last)
{
	DPRINTFN(10, ("%p to %p\n", sqh, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(sqh->page);

	sqh->next = last->next;
	sqh->qh_link = last->qh_link;

	sqh->prev = last;

	usbd_page_dma_enter(sqh->page);
	usbd_page_dma_exit(last->page);

	/* the last->next->prev is never followed:
	 * sqh->next->prev = sqh;
	 */

	last->next = sqh;
	last->qh_link = sqh->qh_self;

	usbd_page_dma_enter(last->page);

#ifdef USB_DEBUG
	if(ehcidebug > 5)
	{
		printf("%s:\n", __FUNCTION__);
		ehci_dump_sqh(sqh);
	}
#endif
	return(sqh);
}

#define EHCI_REMOVE_FS_TD(std,last) (last) = _ehci_remove_fs_td(std,last)
static ehci_sitd_t *
_ehci_remove_fs_td(ehci_sitd_t *std, ehci_sitd_t *last)
{
	DPRINTFN(10, ("%p from %p\n", std, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(std->prev->page);

	std->prev->next = std->next;
	std->prev->sitd_next = std->sitd_next;

	usbd_page_dma_enter(std->prev->page);

	if(std->next)
	{
		usbd_page_dma_exit(std->next->page);
		std->next->prev = std->prev;
		usbd_page_dma_enter(std->next->page);
	}
	return((last == std) ? std->prev : last);
}

#define EHCI_REMOVE_HS_TD(std,last) (last) = _ehci_remove_hs_td(std,last)
static ehci_itd_t *
_ehci_remove_hs_td(ehci_itd_t *std, ehci_itd_t *last)
{
	DPRINTFN(10, ("%p from %p\n", std, last));

	/* (sc->sc_bus.mtx) must be locked */

	usbd_page_dma_exit(std->prev->page);

	std->prev->next = std->next;
	std->prev->itd_next = std->itd_next;

	usbd_page_dma_enter(std->prev->page);

	if(std->next)
	{
		usbd_page_dma_exit(std->next->page);
		std->next->prev = std->prev;
		usbd_page_dma_enter(std->next->page);
	}
	return((last == std) ? std->prev : last);
}

#define EHCI_REMOVE_QH(sqh,last) (last) = _ehci_remove_qh(sqh,last)
static ehci_qh_t *
_ehci_remove_qh(ehci_qh_t *sqh, ehci_qh_t *last)
{
	DPRINTFN(10, ("%p from %p\n", sqh, last));

	/* (sc->sc_bus.mtx) must be locked */

	/* only remove if not removed from a queue */
	if(sqh->prev)
	{
		usbd_page_dma_exit(sqh->prev->page);

		sqh->prev->next = sqh->next;
		sqh->prev->qh_link = sqh->qh_link;

		usbd_page_dma_enter(sqh->prev->page);

		if(sqh->next)
		{
			usbd_page_dma_exit(sqh->next->page);
			sqh->next->prev = sqh->prev;
			usbd_page_dma_enter(sqh->next->page);
		}

		usbd_page_dma_exit(sqh->page);

		/* set the Terminate-bit in the e_next of the QH,
		 * in case the transferred packet was short so
		 * that the QH still points at the last used TD
		 */

		sqh->qh_qtd.qtd_next = htole32(EHCI_LINK_TERMINATE);

		usbd_page_dma_enter(sqh->page);

		last = ((last == sqh) ? sqh->prev : last);

		sqh->prev = 0;
	}
	return(last);
}

static void
ehci_device_done(struct usbd_xfer *xfer, usbd_status error);

static void
ehci_non_isoc_done(struct usbd_xfer *xfer)
{
	uint32_t temp;
	u_int32_t status = 0;
	u_int32_t actlen = 0;
	u_int16_t len = 0;
	u_int16_t last_len = 0;
	u_int8_t last_toggle = 0;
	ehci_qtd_t *td = xfer->td_transfer_first;

	DPRINTFN(12, ("xfer=%p pipe=%p transfer done\n",
		      xfer, xfer->pipe));

#ifdef USB_DEBUG
	if(ehcidebug > 10)
	{
		ehci_dump_sqtds(td);
	}
#endif

	/* the transfer is done, compute actual length and status */
	for (;
	     td != NULL;
	     td = td->obj_next)
	{
		usbd_page_dma_exit(td->page);
		temp = le32toh(td->qtd_status);
		usbd_page_dma_enter(td->page);

		if (temp & EHCI_QTD_ACTIVE) {
			break;
		}

		status = temp;

		len = EHCI_QTD_GET_BYTES(status);

		/* The status length should always be
		 * less than or equal to the setup 
		 * length!
		 */
		if (len <= td->len) {
			last_len = td->len - len;
			actlen += last_len;
		} else {
			/* should not happen */
			DPRINTFN(0, ("Invalid status length, "
				     "0x%04x/0x%04x bytes\n", len, td->len));
			last_len = 0;
		}

		/* Make a copy of the data toggle */
		last_toggle = td->toggle_curr;

		/* Check if this is the last transfer */
		if (((void *)td) == xfer->td_transfer_last) {
			if (len == 0) {
			    /* halt is ok if descriptor is last,
			     * and complete:
			     */
			    status &= ~EHCI_QTD_HALTED;
			}
			td = NULL;
			break;
		}
	}

	/* update data toggle */

	if ((last_len == 0) ||
	    (((last_len + xfer->max_packet_size - 1) /
	      xfer->max_packet_size) & 1)) {
	    last_toggle = !last_toggle;
	}

	xfer->pipe->toggle_next = last_toggle;

	DPRINTFN(10, ("actlen=%d\n", actlen));

	xfer->actlen = actlen;

#ifdef USB_DEBUG
	if (status & EHCI_QTD_STATERRS) {
		DPRINTFN(10,
			 ("error, addr=%d, endpt=0x%02x, "
			  "status=%s%s%s%s%s%s%s%s\n",
			  xfer->address,
			  xfer->endpoint,
			  (status & EHCI_QTD_ACTIVE) ? "ACTIVE" : "NOT_ACTIVE",
			  (status & EHCI_QTD_HALTED) ? "-HALTED" : "",
			  (status & EHCI_QTD_BUFERR) ? "-BUFERR" : "",
			  (status & EHCI_QTD_BABBLE) ? "-BABBLE" : "",
			  (status & EHCI_QTD_XACTERR) ? "-XACTERR" : "",
			  (status & EHCI_QTD_MISSEDMICRO) ? "-MISSED" : "",
			  (status & EHCI_QTD_SPLITXSTATE) ? "-SPLIT" : "",
			  (status & EHCI_QTD_PINGSTATE) ? "-PING" : ""));
	}
#endif
	ehci_device_done(xfer, (status & EHCI_QTD_HALTED) ? 
			 USBD_STALLED : 
			 USBD_NORMAL_COMPLETION);
	return;
}

/* returns one when transfer is finished 
 * and callback must be called else zero
 */
static u_int8_t
ehci_check_transfer(struct usbd_xfer *xfer, struct thread *ctd)
{
	uint32_t status;

	if(xfer->usb_thread != ctd)
	{
	    /* cannot call this transfer 
	     * back due to locking !
	     */
	    goto done;
	}

	DPRINTFN(12, ("xfer=%p checking transfer\n", xfer));

	if(xfer->pipe->methods == &ehci_device_isoc_fs_methods)
	{
		ehci_sitd_t *td = xfer->td_transfer_last;

		/* isochronous full speed transfer */

		usbd_page_dma_exit(td->page);

		status = le32toh(td->sitd_status);

		usbd_page_dma_enter(td->page);

		if (!(status & EHCI_SITD_ACTIVE)) {
			ehci_device_done(xfer, USBD_NORMAL_COMPLETION);
			goto transferred;
		}
	}
	else if(xfer->pipe->methods == &ehci_device_isoc_hs_methods)
	{
		ehci_itd_t *td = xfer->td_transfer_last;

		/* isochronous high speed transfer */

		usbd_page_dma_exit(td->page);

		status = ((!(td->itd_status[0] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[1] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[2] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[3] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[4] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[5] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[6] & htole32(EHCI_ITD_ACTIVE))) &&
			  (!(td->itd_status[7] & htole32(EHCI_ITD_ACTIVE))));

		usbd_page_dma_enter(td->page);

		if (status) {
			ehci_device_done(xfer, USBD_NORMAL_COMPLETION);
			goto transferred;
		}
	}
	else
	{
		ehci_qtd_t *td;

		/* non-isochronous transfer */

		/* check whether there is an error somewhere
		 * in the middle, or whether there was a short
		 * packet (SPD and not ACTIVE)
		 */
		for (td = xfer->td_transfer_cache;
		     td != NULL;
		     td = td->obj_next)
		{
			usbd_page_dma_exit(td->page);

			status = le32toh(td->qtd_status);

			usbd_page_dma_enter(td->page);

			/* if there is an active TD 
			 * the transfer isn't done
			 */
			if (status & EHCI_QTD_ACTIVE) {
				/* update cache */
				xfer->td_transfer_cache = td;
				goto done;
			}

			/* any kind of error makes
			 * the transfer done 
			 */
			if (status & EHCI_QTD_HALTED) {
				break;
			}

			/* a short packet also makes
			 * the transfer done
			 */
			if (EHCI_QTD_GET_BYTES(status)) {
				break;
			}

			if (((void *)td) == xfer->td_transfer_last) {
				td = NULL;
				break;
			}
		}
		ehci_non_isoc_done(xfer);
		goto transferred;
	}

 done:
	DPRINTFN(12, ("xfer=%p is still active\n", xfer));
	return 0;

 transferred:
	return 1;
}

static void
ehci_pcd_enable(ehci_softc_t *sc)
{
	struct usbd_callback_info info[1];
	struct usbd_xfer *xfer;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	sc->sc_eintrs |= EHCI_STS_PCD;
	EOWRITE4(sc, EHCI_USBINTR, sc->sc_eintrs);

	/* acknowledge any PCD interrupt */
	EOWRITE4(sc, EHCI_USBSTS, EHCI_STS_PCD);

	xfer = sc->sc_intrxfer;

 	if(xfer)
	{
	    /* transfer is transferred */
	    ehci_device_done(xfer, USBD_NORMAL_COMPLETION);

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
ehci_interrupt_td(ehci_softc_t *sc, struct thread *ctd)
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
	if(sc->sc_bus.bdev == NULL)
	{
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
	if(ehcidebug > 15)
	{
		DPRINTF(("%s\n", device_get_nameunit(sc->sc_bus.bdev)));
		ehci_dump_regs(sc);
	}
#endif

	status = EHCI_STS_INTRS(EOREAD4(sc, EHCI_USBSTS));
	if(status == 0)
	{
		/* the interrupt was not for us */
		goto done;
	}

	if(!(status & sc->sc_eintrs))
	{
		goto done;
	}

	EOWRITE4(sc, EHCI_USBSTS, status); /* acknowledge */

	status &= sc->sc_eintrs;

	if(status & EHCI_STS_IAA)
	{
		DPRINTF(("door bell\n"));
		wakeup(&sc->sc_async_p_last);
	}

	if(status & EHCI_STS_HSE)
	{
		device_printf(sc->sc_bus.bdev, "unrecoverable error, "
			      "controller halted\n");
#ifdef USB_DEBUG
		ehci_dump_regs(sc);
		ehci_dump_isoc(sc);
#endif
	}

	if(status & EHCI_STS_PCD)
	{
		xfer = sc->sc_intrxfer;

		if(xfer)
		{
		    ehci_device_done(xfer, USBD_NORMAL_COMPLETION);

		    /* queue callback */
		    ptr->xfer = xfer;
		    ptr->refcount = xfer->usb_refcount;
		    ptr++;

		    xfer->usb_root->memory_refcount++;
		}

		/*
		 * Disable PCD interrupt for now, because it will be
		 * on until the port has been reset.
		 */
		sc->sc_eintrs &= ~EHCI_STS_PCD;
		EOWRITE4(sc, EHCI_USBINTR, sc->sc_eintrs);

		/* do not allow RHSC interrupts > 1 per second */
		__callout_reset(&sc->sc_tmo_pcd, hz,
				(void *)(void *)ehci_pcd_enable, sc);
	}

	status &= ~(EHCI_STS_INT | EHCI_STS_ERRINT | EHCI_STS_PCD | EHCI_STS_IAA);

	if(status != 0)
	{
		/* block unprocessed interrupts */
		sc->sc_eintrs &= ~status;
		EOWRITE4(sc, EHCI_USBINTR, sc->sc_eintrs);
		device_printf(sc->sc_bus.bdev, "blocking intrs 0x%x\n",
			      status);
	}

 repeat:
	LIST_FOREACH(xfer, &sc->sc_interrupt_list_head, interrupt_list)
	{
		/* check if transfer is
		 * transferred 
		 */
		if(ehci_check_transfer(xfer, ctd))
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
ehci_interrupt(ehci_softc_t *sc)
{
	ehci_interrupt_td(sc, NULL);
	return;
}

/*
 * called when a request does not complete
 */
static void
ehci_timeout(struct usbd_xfer *xfer)
{
	struct usbd_callback_info info[1];
	ehci_softc_t *sc = xfer->usb_sc;

	DPRINTF(("xfer=%p\n", xfer));

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	/* transfer is transferred */
	ehci_device_done(xfer, USBD_TIMEOUT);

	/* queue callback */
	info[0].xfer = xfer;
	info[0].refcount = xfer->usb_refcount;

	xfer->usb_root->memory_refcount++;

	mtx_unlock(&sc->sc_bus.mtx);

	usbd_do_callback(&info[0],&info[1]);

	return;
}

static void
ehci_do_poll(struct usbd_bus *bus)
{
	ehci_interrupt_td(EHCI_BUS2SC(bus), curthread);
	return;
}

#define ehci_add_interrupt_info(sc, xfer) \
	LIST_INSERT_HEAD(&(sc)->sc_interrupt_list_head, (xfer), interrupt_list)

static void
ehci_remove_interrupt_info(struct usbd_xfer *xfer)
{
	if((xfer)->interrupt_list.le_prev)
	{
		LIST_REMOVE((xfer), interrupt_list);
		(xfer)->interrupt_list.le_prev = NULL;
	}
	return;
}

static void
ehci_setup_standard_chain(struct usbd_xfer *xfer, ehci_qh_t **qh_last)
{
	struct usbd_page_search buf_res;
	/* the EHCI hardware can handle at most five 4k crossing per TD */
	u_int32_t average = (EHCI_PAGE_SIZE - (EHCI_PAGE_SIZE % 
					       xfer->max_packet_size));
	u_int32_t qtd_status;
	uint32_t qh_endp;
	uint32_t qh_endphub;
	u_int32_t buf_offset;
	u_int32_t len = xfer->length;
	u_int32_t c_error = 
	  (xfer->udev->speed == USB_SPEED_HIGH) ? 0 : 
	  htole32(EHCI_QTD_SET_CERR(3));
	u_int8_t isread;
	u_int8_t shortpkt = 0;
	u_int8_t force_short;
	ehci_qtd_t *td;
	ehci_qtd_t *td_last = NULL;
	ehci_qh_t *qh;

	DPRINTFN(8, ("addr=%d endpt=%d len=%d speed=%d\n", 
		     xfer->address, UE_GET_ADDR(xfer->endpoint),
		     xfer->length, xfer->udev->speed));

	td = (xfer->td_transfer_first = 
	      xfer->td_transfer_cache = xfer->td_start);

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	force_short = (xfer->flags & USBD_FORCE_SHORT_XFER) ? 1 : 0;

	if(xfer->pipe->methods == &ehci_device_ctrl_methods)
	{
		/* the first byte is "bmRequestType" */

		isread = *((u_int8_t *)(buf_res.buffer));
		isread &= UT_READ;

		/*
		 * check length ?
		 */
		xfer->pipe->toggle_next = 1;

		usbd_page_dma_exit(td->page);

		/* SETUP message */

		td->qtd_status = c_error | htole32
		  (EHCI_QTD_ACTIVE |
		   EHCI_QTD_SET_PID(EHCI_QTD_PID_SETUP) |
		   EHCI_QTD_SET_TOGGLE(0) |
		   EHCI_QTD_SET_BYTES(sizeof(usb_device_request_t)));

		td->qtd_buffer[0] = htole32(buf_res.physaddr);
		td->qtd_buffer_hi[0] = 0;

		buf_offset += sizeof(usb_device_request_t);
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		td->qtd_buffer[1] = 
		  htole32(buf_res.physaddr & (~0xFFF));
		td->qtd_buffer_hi[1] = 0;
		td->toggle_curr = 0;

		td->len = sizeof(usb_device_request_t);
		len -= sizeof(usb_device_request_t);
		td_last = td;
		td = td->obj_next;

		if (td) {
		    /* link the last TD with the next one */
		    td_last->qtd_next = td->qtd_self;
		    /* short transfers should terminate the transfer: */
		    td_last->qtd_altnext = htole32(EHCI_LINK_TERMINATE);
		}

		usbd_page_dma_enter(td_last->page);
	}
	else
	{
		isread = (UE_GET_DIR(xfer->endpoint) == UE_DIR_IN);

		if (xfer->length == 0) {
			/* When the length is zero we
			 * queue a short packet!
			 * This also makes "td_last"
			 * non-zero.
			 */
			DPRINTFN(0, ("short transfer!\n"));
			force_short = 1;
		}
	}

	qtd_status = c_error | (isread ?
	  htole32
	  (EHCI_QTD_ACTIVE |
	   EHCI_QTD_SET_PID(EHCI_QTD_PID_IN)) :
	  htole32
	  (EHCI_QTD_ACTIVE |
	   EHCI_QTD_SET_PID(EHCI_QTD_PID_OUT)));

	if(xfer->pipe->toggle_next)
	{
		qtd_status |= htole32(EHCI_QTD_SET_TOGGLE(1));
	}

	while(1)
	{
		if(len == 0)
		{
			if (force_short)
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

		usbd_page_dma_exit(td->page);

		/* fill out current TD */

		td->qtd_status = 
		  qtd_status | htole32(EHCI_QTD_SET_BYTES(average));

		td->qtd_buffer[0] = htole32(buf_res.physaddr);
		td->qtd_buffer_hi[0] = 0;

		buf_offset += average;
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		td->qtd_buffer[1] = 
		  htole32(buf_res.physaddr & (~0xFFF));
		td->qtd_buffer_hi[1] = 0;

		td->toggle_curr = xfer->pipe->toggle_next;
		td->len = average;

		/* adjust the toggle based on the 
		 * number of packets in this qtd
		 */
		if ((average == 0) ||
		    (((average + xfer->max_packet_size - 1) / 
		      xfer->max_packet_size) & 1))
		{
		    xfer->pipe->toggle_next =
		      xfer->pipe->toggle_next ? 0 : 1;

		    qtd_status ^= htole32(EHCI_QTD_TOGGLE_MASK);
		}

		len -= average;
		td_last = td;
		td = td->obj_next;

		if (td) {
		    /* link the last TD with the next one */
		    td_last->qtd_next = td->qtd_self;
		    /* short transfers should terminate the transfer: */
		    td_last->qtd_altnext = htole32(EHCI_LINK_TERMINATE);
		}

		usbd_page_dma_enter(td_last->page);
	}

	if(xfer->pipe->methods == &ehci_device_ctrl_methods)
	{
		usbd_page_dma_exit(td->page);

		/* STATUS message */

		td->qtd_status = c_error | (isread ?
		  htole32
		  (EHCI_QTD_ACTIVE |
		   EHCI_QTD_SET_PID(EHCI_QTD_PID_OUT) |
		   EHCI_QTD_SET_TOGGLE(1) |
		   EHCI_QTD_IOC) :
		  htole32
		  (EHCI_QTD_ACTIVE |
		   EHCI_QTD_SET_PID(EHCI_QTD_PID_IN) |
		   EHCI_QTD_SET_TOGGLE(1) |
		   EHCI_QTD_IOC));

		td->qtd_buffer[0] = 0; 
		td->qtd_buffer_hi[0] = 0;

		td->len = 0;
		td_last = td;

		usbd_page_dma_enter(td_last->page);
	}

	usbd_page_dma_exit(td_last->page);

	/* the last TD terminates the transfer: */
	td_last->qtd_next = htole32(EHCI_LINK_TERMINATE);
	td_last->qtd_altnext = htole32(EHCI_LINK_TERMINATE);
	td_last->qtd_status |= htole32(EHCI_QTD_IOC);

	usbd_page_dma_enter(td_last->page);

	/* must have at least one frame! */

	xfer->td_transfer_last = td_last;

#ifdef USB_DEBUG
	if(ehcidebug > 8)
	{
		DPRINTF(("nexttog=%d; data before transfer:\n",
			 xfer->pipe->toggle_next));
		ehci_dump_sqtds(xfer->td_start);
	}
#endif

	qh = xfer->qh_start;

	usbd_page_dma_exit(qh->page);

	/* the "qh_link" field is filled when the QH is added */

	qh_endp =
	  (EHCI_QH_SET_ADDR(xfer->address) |
	   EHCI_QH_SET_ENDPT(UE_GET_ADDR(xfer->endpoint)) |
	   EHCI_QH_DTC |
	   EHCI_QH_SET_MPL(xfer->max_packet_size) |
	   EHCI_QH_SET_NRL(8) /* XXX */ 
	   );

	switch (xfer->udev->speed) {
	case USB_SPEED_LOW:  
	  qh_endp |= (EHCI_QH_SET_EPS(EHCI_QH_SPEED_LOW)|
		      EHCI_QH_CTL);  
	  break;
	case USB_SPEED_FULL: 
	  qh_endp |= (EHCI_QH_SET_EPS(EHCI_QH_SPEED_FULL)|
		      EHCI_QH_CTL);
	  break;
	case USB_SPEED_HIGH: 
	  qh_endp |= (EHCI_QH_SET_EPS(EHCI_QH_SPEED_HIGH)); 
	  break;
	default:
	  panic("%s: bad device speed %d!", __FUNCTION__, xfer->udev->speed);
	  break;
	}

	if(xfer->pipe->methods != &ehci_device_ctrl_methods)
	{
		qh_endp &= ~EHCI_QH_CTL;
	}

	qh->qh_endp = htole32(qh_endp);

	qh_endphub =
	  (EHCI_QH_SET_MULT(xfer->max_packet_count & 3)|
	   EHCI_QH_SET_CMASK(0xf0));

	if(xfer->udev->myhsport)
	{
		qh_endphub |= 
		  (EHCI_QH_SET_HUBA(xfer->udev->myhsport->parent->address)|
		   EHCI_QH_SET_PORT(xfer->udev->myhsport->portno));
	}

	if(xfer->pipe->methods == &ehci_device_intr_methods)
	{
		/* execute the transfer one time per 1ms */
		qh_endphub |= EHCI_QH_SET_SMASK(0x04);
	}

	qh->qh_endphub = htole32(qh_endphub);
	qh->qh_curqtd = htole32(0);

	/* fill the overlay qTD */
	qh->qh_qtd.qtd_status = htole32(0);

	td = xfer->td_transfer_first;

	qh->qh_qtd.qtd_next = td->qtd_self;
	qh->qh_qtd.qtd_altnext = htole32(EHCI_LINK_TERMINATE);

	usbd_page_dma_enter(qh->page);

	EHCI_APPEND_QH(qh, *qh_last);
	return;
}

static void
ehci_root_intr_done(ehci_softc_t *sc, struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	u_int8_t *p;
	u_int16_t i;
	u_int16_t m;

	if(sc->sc_intrxfer)
	{
		/* disable further interrupts */
		sc->sc_intrxfer = NULL;

		/* clear all bits */
		usbd_bzero(&(xfer->buf_data), 0, xfer->length);

		/* set bits */
		m = (xfer->length * 8);
		i = (sc->sc_noport + 1);
		m = min(m,i);
		for(i = 1; i < m; i++)
		{
			/* pick out CHANGE bits from the status register */
			if(EOREAD4(sc, EHCI_PORTSC(i)) & EHCI_PS_CLEAR)
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

static u_int8_t
ehci_isoc_fs_done(ehci_softc_t *sc, struct usbd_xfer *xfer)
{
	u_int32_t nframes = xfer->nframes;
	u_int32_t actlen = 0;
	uint32_t status;
	u_int16_t *plen = xfer->frlengths;
	u_int16_t len = 0;
	u_int8_t need_delay = 0;
	ehci_sitd_t *td = xfer->td_transfer_first;
	ehci_sitd_t **pp_last = &sc->sc_isoc_fs_p_last[xfer->qh_pos];

	DPRINTFN(12, ("xfer=%p pipe=%p transfer done\n",
		      xfer, xfer->pipe));

	while(nframes--)
	{
	  if(td == NULL)
	  {
		panic("%s:%d: out of TD's\n",
		      __FUNCTION__, __LINE__);
	  }

	  if(pp_last >= &sc->sc_isoc_fs_p_last[EHCI_VIRTUAL_FRAMELIST_COUNT])
	  {
		pp_last = &sc->sc_isoc_fs_p_last[0];
	  }

#ifdef USB_DEBUG
	  if(ehcidebug > 15)
	  {
		DPRINTFN(15,("isoc FS-TD\n"));
		ehci_dump_sitd(td);
	  }
#endif
	  usbd_page_dma_exit(td->page);
	  status = le32toh(td->sitd_status);
	  usbd_page_dma_enter(td->page);

	  /* check for active transfers */
	  if (status & EHCI_SITD_ACTIVE) {
		need_delay = 1;
	  }

	  len = EHCI_SITD_GET_LEN(status);

	  if(*plen >= len)
	  {
		len = *plen - len;
	  }
	  else
	  {
		len = 0;
	  }

	  *plen = len;
	  actlen += len;

	  /* remove FS-TD from schedule */
	  EHCI_REMOVE_FS_TD(td, *pp_last);

	  pp_last++;
	  plen++;
	  td = td->obj_next;
	}
	xfer->actlen = actlen;

	return need_delay;
}

static u_int8_t
ehci_isoc_hs_done(ehci_softc_t *sc, struct usbd_xfer *xfer)
{
	u_int32_t nframes = xfer->nframes;
	u_int32_t actlen = 0;
	uint32_t status;
	u_int16_t *plen = xfer->frlengths;
	u_int16_t len = 0;
	u_int8_t td_no = 0;
	u_int8_t need_delay = 0;
	ehci_itd_t *td = xfer->td_transfer_first;
	ehci_itd_t **pp_last = &sc->sc_isoc_hs_p_last[xfer->qh_pos];

	DPRINTFN(12, ("xfer=%p pipe=%p transfer done\n",
		      xfer, xfer->pipe));

	while(nframes--)
	{
	  if(td == NULL)
	  {
		panic("%s:%d: out of TD's\n",
		      __FUNCTION__, __LINE__);
	  }

	  if(pp_last >= &sc->sc_isoc_hs_p_last[EHCI_VIRTUAL_FRAMELIST_COUNT])
	  {
		pp_last = &sc->sc_isoc_hs_p_last[0];
	  }

#ifdef USB_DEBUG
	  if(ehcidebug > 15)
	  {
		DPRINTFN(15,("isoc HS-TD\n"));
		ehci_dump_itd(td);
	  }
#endif

	  usbd_page_dma_exit(td->page);
	  status = le32toh(td->itd_status[td_no]);
	  usbd_page_dma_enter(td->page);

	  if (status & EHCI_ITD_ACTIVE) {
		need_delay = 1;
	  }

	  len = EHCI_ITD_GET_LEN(status);

	  if(*plen >= len)
	  {
		len = *plen - len;
	  }
	  else
	  {
		len = 0;
	  }

	  *plen = len;
	  actlen += len;

	  plen++;
	  td_no++;

	  if((td_no == 8) || (nframes == 0))
	  {
		/* remove HS-TD from schedule */
		EHCI_REMOVE_HS_TD(td, *pp_last);
		pp_last++;

		td_no = 0;
		td = td->obj_next;
	  }
	}
	xfer->actlen = actlen;

	return need_delay;
}

/* NOTE: "done" can be run two times in a row,
 * from close and from interrupt
 */
static void
ehci_device_done(struct usbd_xfer *xfer, usbd_status error)
{
	ehci_softc_t *sc = xfer->usb_sc;
	u_int8_t need_delay;

	mtx_assert(&sc->sc_bus.mtx, MA_OWNED);

	need_delay = 0;

	DPRINTFN(1,("xfer=%p, pipe=%p length=%d error=%d\n",
		    xfer, xfer->pipe, xfer->actlen, error));

	/* ... could check for not-completed transfers, 
	 * instead of setting need_delay ...
	 */
	if((error == USBD_CANCELLED) ||
	   (error == USBD_TIMEOUT))
	{
		if(xfer->flags & USBD_DEV_TRANSFERRING)
		{
			need_delay = 1;
		}
	}

	if((xfer->pipe->methods == &ehci_device_bulk_methods) ||
	   (xfer->pipe->methods == &ehci_device_ctrl_methods))
	{
#ifdef USB_DEBUG
		if(ehcidebug > 8)
		{
			DPRINTF(("nexttog=%d; data after transfer:\n",
				 xfer->pipe->toggle_next));
			ehci_dump_sqtds(xfer->td_start);
		}
#endif

		EHCI_REMOVE_QH(xfer->qh_start, sc->sc_async_p_last);
	}

	if(xfer->pipe->methods == &ehci_device_intr_methods)
	{
		EHCI_REMOVE_QH(xfer->qh_start, sc->sc_intr_p_last[xfer->qh_pos]);
	}

	/* finish isochronous transfers 
	 * (will update xfer->actlen and xfer->frlengths;
	 *  should only be called once)
	 */
	if(xfer->td_transfer_first &&
	   xfer->td_transfer_last)
	{
		if(xfer->pipe->methods == &ehci_device_isoc_fs_methods)
		{
			if(ehci_isoc_fs_done(sc, xfer))
			{
				need_delay = 1;
			}
		}

		if(xfer->pipe->methods == &ehci_device_isoc_hs_methods)
		{
			if(ehci_isoc_hs_done(sc, xfer))
			{
				need_delay = 1;
			}
		}

		xfer->td_transfer_first = NULL;
		xfer->td_transfer_last = NULL;
	}

	/* finish root interrupt transfer
	 * (will update xfer->buffer and xfer->actlen)
	 */
	if(xfer->pipe->methods == &ehci_root_intr_methods)
	{
		ehci_root_intr_done(sc, xfer);
	}

	/* stop timeout */
	__callout_stop(&xfer->timeout_handle);

	/* remove interrupt info (if any) */
	ehci_remove_interrupt_info(xfer);

	if ((xfer->pipe->methods != &ehci_root_ctrl_methods) &&
	    (xfer->pipe->methods != &ehci_root_intr_methods)) {

	  if(((xfer->pipe->methods == &ehci_device_ctrl_methods) ||
	      (xfer->pipe->methods == &ehci_device_bulk_methods)) &&
	     (sc->sc_doorbell_disable == 0)) {

		u_int32_t to = 100*1000;

		/*
		 * This implementation does not use the doorbell
		 * interrupt. The reason is that one gets more
		 * throughput by waiting for the doorbell inline,
		 * than by using the doorbell interrupt. A typical
		 * result reading from an USB mass storage disk:
		 *
		 * With doorbell interrupt: 27 seconds for 512MB
		 * With inline doorbell: 21 seconds for 512MB
		 *
		 * Although the inline version causes a slightly
		 * higher CPU usage, it does not block the system
		 * in any way, because this USB driver uses its
		 * own mutex.
		 *
		 * --hps
		 */

		/* wait for doorbell ~32us */
		EOWRITE4(sc, EHCI_USBCMD, 
			 EOREAD4(sc, EHCI_USBCMD) | EHCI_CMD_IAAD);

		while(EOREAD4(sc, EHCI_USBCMD) & EHCI_CMD_IAAD)
		{
		    if(!to--)
		    {
		        printf("%s: doorbell timeout "
			       "(disabling)\n", __FUNCTION__);
			sc->sc_doorbell_disable = 1;
			break;
		    }
		    DELAY(1);
		}

		/* acknowledge any doorbell interrupt 
		 * (SiS chipsets require this)
		 */
		EOWRITE4(sc, EHCI_USBSTS, EHCI_STS_IAA);

	  } else {
		/* wait until the hardware has finished any possible
		 * use of the transfer descriptor and QH
		 *
		 * in case "need_delay" is set, 
		 * wait until the next isochronous 
		 * frame is started
		 */
		DELAY(need_delay ? ((3*1000)/(2*8)) : (5));
	  }
	}

	if(error)
	{
		/* next transfer needs to clear stall */
		xfer->pipe->clearstall = 1;
	}

	/* transfer transferred (no callback!) */
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
 * ehci bulk support
 *---------------------------------------------------------------------------*/
static void
ehci_device_bulk_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ehci_device_bulk_close(struct usbd_xfer *xfer)
{
	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ehci_device_bulk_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ehci_device_bulk_start(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;

	DPRINTFN(3, ("xfer=%p len=%d\n",
		     xfer, xfer->length));

	/* setup TD's and QH */
	ehci_setup_standard_chain(xfer, &sc->sc_async_p_last);

	/**/
	ehci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ehci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods ehci_device_bulk_methods = 
{
  .open = ehci_device_bulk_open,
  .close = ehci_device_bulk_close,
  .enter = ehci_device_bulk_enter,
  .start = ehci_device_bulk_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

/*---------------------------------------------------------------------------*
 * ehci control support
 *---------------------------------------------------------------------------*/
static void
ehci_device_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ehci_device_ctrl_close(struct usbd_xfer *xfer)
{
	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ehci_device_ctrl_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ehci_device_ctrl_start(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;

	/* setup TD's and QH */
	ehci_setup_standard_chain(xfer, &sc->sc_async_p_last);

	/**/
	ehci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ehci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods ehci_device_ctrl_methods = 
{
  .open = ehci_device_ctrl_open,
  .close = ehci_device_ctrl_close,
  .enter = ehci_device_ctrl_enter,
  .start = ehci_device_ctrl_start,
  .copy_in = usbd_std_ctrl_copy_in,
  .copy_out = usbd_std_ctrl_copy_out,
};

/*---------------------------------------------------------------------------*
 * ehci interrupt support
 *---------------------------------------------------------------------------*/
static void
ehci_device_intr_open(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;
	u_int16_t best;
	u_int16_t bit;
	u_int16_t x;

	best = 0;
	bit = EHCI_VIRTUAL_FRAMELIST_COUNT/2;
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
ehci_device_intr_close(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;

	sc->sc_intr_stat[xfer->qh_pos]--;

	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ehci_device_intr_enter(struct usbd_xfer *xfer)
{
	/* enqueue transfer */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ehci_device_intr_start(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;

	DPRINTFN(3,("xfer=%p len=%d\n",
		    xfer, xfer->length));

	/* setup TD's and QH */
	ehci_setup_standard_chain(xfer, &sc->sc_intr_p_last[xfer->qh_pos]);

	/**/
	ehci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ehci_timeout, xfer);
	}
	return;
}

struct usbd_pipe_methods ehci_device_intr_methods = 
{
  .open = ehci_device_intr_open,
  .close = ehci_device_intr_close,
  .enter = ehci_device_intr_enter,
  .start = ehci_device_intr_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

/*---------------------------------------------------------------------------*
 * ehci full speed isochronous support
 *---------------------------------------------------------------------------*/
static void
ehci_device_isoc_fs_open(struct usbd_xfer *xfer)
{
	ehci_sitd_t *td;
	u_int32_t sitd_portaddr;

	sitd_portaddr = 
	  EHCI_SITD_SET_ADDR(xfer->address)|
	  EHCI_SITD_SET_ENDPT(UE_GET_ADDR(xfer->endpoint));

	if(UE_GET_DIR(xfer->endpoint) == UE_DIR_IN)
	{
		sitd_portaddr |= EHCI_SITD_SET_DIR_IN;
	}

	if(xfer->udev->myhsport)
	{
		sitd_portaddr |= 
			EHCI_SITD_SET_HUBA(xfer->udev->myhsport->parent->address)|
			EHCI_SITD_SET_PORT(xfer->udev->myhsport->portno);
	}

	sitd_portaddr = htole32(sitd_portaddr);

	/* initialize all TD's */

	for(td = xfer->td_start; td; td = td->obj_next)
	{
		usbd_page_dma_exit(td->page);

		td->sitd_portaddr = sitd_portaddr;

		/* TODO: make some kind of automatic SMASK/CMASK selection
		 * based on micro-frame usage
		 *
		 * micro-frame usage 
		 * (8 microframes per 1ms)
		 *
		 * 0: isoc-IN
		 * 1: isoc-OUT
		 * 2: interrupt transfers
		 * .
		 * .
		 * 7:
		 */
		if(UE_GET_DIR(xfer->endpoint) == UE_DIR_IN)
		{
			td->sitd_mask = htole32(EHCI_SITD_SET_SMASK(0x01)|
						EHCI_SITD_SET_CMASK(0xFC));
		}
		else
		{
			td->sitd_mask = htole32(EHCI_SITD_SET_SMASK(0x02)|
						EHCI_SITD_SET_CMASK(0xF8));
		}
		td->sitd_back = htole32(EHCI_LINK_TERMINATE);

		usbd_page_dma_enter(td->page);
	}
	return;
}

static void
ehci_device_isoc_fs_close(struct usbd_xfer *xfer)
{
	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ehci_device_isoc_fs_enter(struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	ehci_softc_t *sc = xfer->usb_sc;
	u_int32_t buf_offset;
	u_int32_t nframes;
	uint32_t temp;
	u_int16_t *plen;
#ifdef USB_DEBUG
	u_int8_t once = 1;
#endif
	ehci_sitd_t *td;
	ehci_sitd_t *td_last = NULL;
	ehci_sitd_t **pp_last;

	DPRINTFN(5,("xfer=%p next=%d nframes=%d\n",
		    xfer, xfer->pipe->isoc_next, xfer->nframes));

	/* get the current frame index */

	nframes = EOREAD4(sc, EHCI_FRINDEX) / 8;

	/* check if the frame index is within
	 * the window where the frames will be
	 * inserted
	 */
	buf_offset = (nframes - xfer->pipe->isoc_next) & 
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	if (buf_offset < xfer->nframes) {
		/* not in use yet, schedule it a few frames ahead */
		/* data underflow */
		xfer->pipe->isoc_next = (nframes + 3) & 
		  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);
		DPRINTFN(2,("start next=%d\n", xfer->pipe->isoc_next));
	}

	/* compute how many milliseconds the
	 * insertion is ahead of the current
	 * frame position:
	 */
	buf_offset = (xfer->pipe->isoc_next - nframes) & 
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	/* pre-compute when the isochronous transfer
	 * will be finished:
	 */
	xfer->isoc_time_complete = 
	  usbd_isoc_time_expand(&(sc->sc_bus), nframes) + buf_offset + 
	  xfer->nframes;

	/* get the real number of frames */

	nframes = xfer->nframes;

	if(nframes == 0)
	{
		/* transfer transferred */
		ehci_device_done(xfer, USBD_NORMAL_COMPLETION);

		/* call callback recursively */
		__usbd_callback(xfer);

		return;
	}

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	plen = xfer->frlengths;

	td = (xfer->td_transfer_first = xfer->td_start);

	pp_last = &sc->sc_isoc_fs_p_last[xfer->pipe->isoc_next];

	/* store starting position */

	xfer->qh_pos = xfer->pipe->isoc_next;

	while(nframes--)
	{
		if(td == NULL)
		{
			panic("%s:%d: out of TD's\n",
			      __FUNCTION__, __LINE__);
		}

		if(pp_last >= &sc->sc_isoc_fs_p_last[EHCI_VIRTUAL_FRAMELIST_COUNT])
		{
			pp_last = &sc->sc_isoc_fs_p_last[0];
		}

		/* reuse sitd_portaddr and sitd_back from last transfer */

		/* TODO: implement support for multiple transactions */
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

		td->sitd_bp[0] = htole32(buf_res.physaddr);

		buf_offset += *plen;
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		temp = buf_res.physaddr & (~0xFFF);

		if(UE_GET_DIR(xfer->endpoint) == UE_DIR_OUT)
		{
			temp |= 1; /* T-count == 1*/
		}

		td->sitd_bp[1] = htole32(temp);

		if(nframes == 0)
		{
			td->sitd_status = htole32
			  (EHCI_SITD_IOC|
			   EHCI_SITD_ACTIVE|
			   EHCI_SITD_SET_LEN(*plen));
		}
		else
		{
			td->sitd_status = htole32
			  (EHCI_SITD_ACTIVE|
			   EHCI_SITD_SET_LEN(*plen));
		}
		usbd_page_dma_enter(td->page);

#ifdef USB_DEBUG
		if(ehcidebug > 15)
		{
			DPRINTFN(15,("FS-TD %d\n", nframes));
			ehci_dump_sitd(td);
		}
#endif
		/* insert TD into schedule */
		EHCI_APPEND_FS_TD(td, *pp_last);
		pp_last++;

		plen++;
		td_last = td;
		td = td->obj_next;
	}

	xfer->td_transfer_last = td_last;

	/* update isoc_next */
	xfer->pipe->isoc_next = (pp_last - &sc->sc_isoc_fs_p_last[0]) &
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	/**/
	ehci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ehci_timeout, xfer);
	}

	/* enqueue transfer 
	 * (so that it can be aborted through pipe abort)
	 */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ehci_device_isoc_fs_start(struct usbd_xfer *xfer)
{
	/* already started, nothing to do */
	return;
}

struct usbd_pipe_methods ehci_device_isoc_fs_methods = 
{
  .open = ehci_device_isoc_fs_open,
  .close = ehci_device_isoc_fs_close,
  .enter = ehci_device_isoc_fs_enter,
  .start = ehci_device_isoc_fs_start,
  .copy_in = usbd_std_isoc_copy_in,
  .copy_out = usbd_std_isoc_copy_out,
};

/*---------------------------------------------------------------------------*
 * ehci high speed isochronous support
 *---------------------------------------------------------------------------*/
static void
ehci_device_isoc_hs_open(struct usbd_xfer *xfer)
{
	ehci_itd_t *td;
	uint32_t temp;

	/* initialize all TD's */

	for(td = xfer->td_start; td; td = td->obj_next)
	{
		usbd_page_dma_exit(td->page);

		/* set TD inactive */
		td->itd_status[0] = 0;
		td->itd_status[1] = 0;
		td->itd_status[2] = 0;
		td->itd_status[3] = 0;
		td->itd_status[4] = 0;
		td->itd_status[5] = 0;
		td->itd_status[6] = 0;
		td->itd_status[7] = 0;

		/* set endpoint and address */
		td->itd_bp[0] = htole32
		  (EHCI_ITD_SET_ADDR(xfer->address)|
		   EHCI_ITD_SET_ENDPT(UE_GET_ADDR(xfer->endpoint)));

		temp = EHCI_ITD_SET_MPL(xfer->max_packet_size & 0x7FF);

		/* set direction */
		if(UE_GET_DIR(xfer->endpoint) == UE_DIR_IN)
		{
			temp |= EHCI_ITD_SET_DIR_IN;
		}

		/* set maximum packet size */
		td->itd_bp[1] = htole32(temp);

		/* set transfer multiplier */
		td->itd_bp[2] = htole32(xfer->max_packet_count & 3);

		usbd_page_dma_enter(td->page);
	}
	return;
}

static void
ehci_device_isoc_hs_close(struct usbd_xfer *xfer)
{
	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ehci_device_isoc_hs_enter(struct usbd_xfer *xfer)
{
	struct usbd_page_search buf_res;
	ehci_softc_t *sc = xfer->usb_sc;
	bus_size_t page_addr;
	u_int32_t status;
	u_int32_t buf_offset;
	u_int32_t nframes;
	u_int16_t *plen;
	u_int8_t page_no;
	u_int8_t td_no;
#ifdef USB_DEBUG
	u_int8_t once = 1;
#endif
	ehci_itd_t *td;
	ehci_itd_t *td_last = NULL;
	ehci_itd_t **pp_last;

	DPRINTFN(5,("xfer=%p next=%d nframes=%d\n",
		    xfer, xfer->pipe->isoc_next, xfer->nframes));

	/* get the current frame index */

	nframes = EOREAD4(sc, EHCI_FRINDEX) / 8;

	/* check if the frame index is within
	 * the window where the frames will be
	 * inserted
	 */
	buf_offset = (nframes - xfer->pipe->isoc_next) & 
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	if (buf_offset < xfer->nframes) {
		/* not in use yet, schedule it a few frames ahead */
		/* data underflow */
		xfer->pipe->isoc_next = (nframes + 3) & 
		  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);
		DPRINTFN(2,("start next=%d\n", xfer->pipe->isoc_next));
	}

	/* compute how many milliseconds the
	 * insertion is ahead of the current
	 * frame position:
	 */
	buf_offset = (xfer->pipe->isoc_next - nframes) & 
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	/* pre-compute when the isochronous transfer
	 * will be finished:
	 */
	xfer->isoc_time_complete = 
	  usbd_isoc_time_expand(&(sc->sc_bus), nframes) + buf_offset + 
	  ((xfer->nframes+7)/8);

	/* get the real number of frames */

	nframes = xfer->nframes;

	if(nframes == 0)
	{
		/* transfer transferred */
		ehci_device_done(xfer, USBD_NORMAL_COMPLETION);

		/* call callback recursively */
		__usbd_callback(xfer);

		return;
	}

	buf_offset = 0;
	usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

	page_addr = buf_res.physaddr & ~0xFFF;
	page_no = 0;
	td_no = 0;

	plen = xfer->frlengths;

	td = (xfer->td_transfer_first = xfer->td_start);

	pp_last = &sc->sc_isoc_hs_p_last[xfer->pipe->isoc_next];

	/* store starting position */

	xfer->qh_pos = xfer->pipe->isoc_next;

	while(nframes--)
	{
		if(td == NULL)
		{
			panic("%s:%d: out of TD's\n",
			      __FUNCTION__, __LINE__);
		}

		if(pp_last >= &sc->sc_isoc_hs_p_last[EHCI_VIRTUAL_FRAMELIST_COUNT])
		{
			pp_last = &sc->sc_isoc_hs_p_last[0];
		}

		/* range check */
		if(*plen > xfer->max_frame_size)
		{
#ifdef USB_DEBUG
			if(once)
			{
				once = 0;
				printf("%s: frame length(%d) exceeds %d bytes "
				       "(frame truncated)\n", 
				       __FUNCTION__, *plen, xfer->max_frame_size);
			}
#endif
			*plen = xfer->max_frame_size;
		}

		if(td_no == 0)
		{
			usbd_page_dma_exit(td->page);

			/* update page address */
			td->itd_bp[page_no] &= htole32(0xFFF);
			td->itd_bp[page_no] |= htole32(page_addr);

			if(nframes < 7)
			{
				/* clear all status in case
				 * some are not initialized
				 */
				td->itd_status[0] = 0;
				td->itd_status[1] = 0;
				td->itd_status[2] = 0;
				td->itd_status[3] = 0;
				td->itd_status[4] = 0;
				td->itd_status[5] = 0;
				td->itd_status[6] = 0;
				td->itd_status[7] = 0;
			}
		}

		/* compute status */
		if(nframes == 0)
		{
			status = 
			  (EHCI_ITD_SET_LEN(*plen)|
			   EHCI_ITD_ACTIVE|
			   EHCI_ITD_IOC|
			   EHCI_ITD_SET_PG(page_no)|
			   (buf_res.physaddr & 0xFFF));
		}
		else
		{
			status = 
			  (EHCI_ITD_SET_LEN(*plen)|
			   EHCI_ITD_ACTIVE|
			   EHCI_ITD_SET_PG(page_no)|
			   (buf_res.physaddr & 0xFFF));
		}

		buf_offset += *plen;
		usbd_get_page(&(xfer->buf_data), buf_offset, &buf_res);

		if((buf_res.physaddr ^ page_addr) & ~0xFFF)
		{
			/* new page needed */
			page_addr = buf_res.physaddr & ~0xFFF;
			page_no++;

			if(page_no < 7)
			{
				/* update page address */
				td->itd_bp[page_no] &= htole32(0xFFF);
				td->itd_bp[page_no] |= htole32(page_addr);
			}
		}

		if(page_no < 7)
		{
			/* activate the transfer */
			td->itd_status[td_no] = htole32(status);
		}
		else
		{
			/* pretend that the transfer has finished */
			td->itd_status[td_no] = (nframes == 0) ? 
			  htole32(EHCI_ITD_IOC) : 0;
#ifdef USB_DEBUG
			if(once)
			{
				once = 0;
				printf("%s: isoc limit reached! "
				       "Max %d bytes per 8 frames. Frame skipped.\n",
				       __FUNCTION__, (6 << 12));
			}
#endif
		}

		plen++;
		td_no++;

		if((td_no == 8) || (nframes == 0))
		{
			usbd_page_dma_enter(td->page);
#ifdef USB_DEBUG
			if(ehcidebug > 15)
			{
				DPRINTFN(15,("HS-TD %d\n", nframes));
				ehci_dump_itd(td);
			}
#endif
			/* insert TD into schedule */
			EHCI_APPEND_HS_TD(td, *pp_last);
			pp_last++;

			page_no = 0;
			td_no = 0;
			td_last = td;
			td = td->obj_next;
		}
	}

	xfer->td_transfer_last = td_last;

	/* update isoc_next */
	xfer->pipe->isoc_next = (pp_last - &sc->sc_isoc_hs_p_last[0]) &
	  (EHCI_VIRTUAL_FRAMELIST_COUNT-1);

	/**/
	ehci_add_interrupt_info(sc, xfer);

	if(xfer->timeout && (!(xfer->flags & USBD_USE_POLLING)))
	{
		__callout_reset(&xfer->timeout_handle, MS_TO_TICKS(xfer->timeout),
				(void *)(void *)ehci_timeout, xfer);
	}

	/* enqueue transfer 
	 * (so that it can be aborted through pipe abort)
	 */
	usbd_transfer_enqueue(xfer);
	return;
}

static void
ehci_device_isoc_hs_start(struct usbd_xfer *xfer)
{
	/* already started, nothing to do */
	return;
}

struct usbd_pipe_methods ehci_device_isoc_hs_methods = 
{
  .open = ehci_device_isoc_hs_open,
  .close = ehci_device_isoc_hs_close,
  .enter = ehci_device_isoc_hs_enter,
  .start = ehci_device_isoc_hs_start,
  .copy_in = usbd_std_isoc_copy_in,
  .copy_out = usbd_std_isoc_copy_out,
};

/*---------------------------------------------------------------------------*
 * ehci root control support
 *---------------------------------------------------------------------------*
 * simulate a hardware hub by handling
 * all the necessary requests
 *---------------------------------------------------------------------------*/

static void
ehci_root_ctrl_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ehci_root_ctrl_close(struct usbd_xfer *xfer)
{
	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

/* data structures and routines
 * to emulate the root hub:
 */

static const
usb_device_descriptor_t ehci_devd = 
{
	sizeof(usb_device_descriptor_t),
	UDESC_DEVICE,		/* type */
	{0x00, 0x02},		/* USB version */
	UDCLASS_HUB,		/* class */
	UDSUBCLASS_HUB,		/* subclass */
	UDPROTO_HSHUBSTT,	/* protocol */
	64,			/* max packet */
	{0},{0},{0x00,0x01},	/* device id */
	1,2,0,			/* string indicies */
	1			/* # of configurations */
};

static const
usb_device_qualifier_t ehci_odevd = 
{
	sizeof(usb_device_qualifier_t), 
	UDESC_DEVICE_QUALIFIER,	/* type */
	{0x00, 0x02},		/* USB version */
	UDCLASS_HUB,		/* class */
	UDSUBCLASS_HUB,		/* subclass */
	UDPROTO_FSHUB,		/* protocol */
	64,			/* max packet */
	1,			/* # of configurations */
	0
};

static const
usb_config_descriptor_t ehci_confd = 
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
usb_interface_descriptor_t ehci_ifcd = 
{
	sizeof(usb_interface_descriptor_t),
	UDESC_INTERFACE,
	0,
	0,
	1,
	UICLASS_HUB,
	UISUBCLASS_HUB,
	UIPROTO_HSHUBSTT,
	0
};

static const
usb_endpoint_descriptor_t ehci_endpd =
{
	sizeof(usb_endpoint_descriptor_t),
	UDESC_ENDPOINT,
	UE_DIR_IN | EHCI_INTR_ENDPT,
	UE_INTERRUPT,
	{8, 0},			/* max packet */
	255
};

static const
usb_hub_descriptor_t ehci_hubd =
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
ehci_disown(ehci_softc_t *sc, int index, int lowspeed)
{
	u_int32_t port;
	u_int32_t v;

	DPRINTF(("index=%d lowspeed=%d\n", index, lowspeed));

	port = EHCI_PORTSC(index);
	v = EOREAD4(sc, port) &~ EHCI_PS_CLEAR;
	EOWRITE4(sc, port, v | EHCI_PS_PO);
}

static void
ehci_root_ctrl_enter(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;
	u_int32_t port;
	u_int32_t v;
	u_int16_t i;
	u_int16_t len;
	u_int16_t value;
	u_int16_t index;
	u_int16_t l;
	u_int16_t totlen = 0;
	union {
	  usb_status_t stat;
	  usb_port_status_t ps;
	  usb_device_request_t req;
	  usb_hub_descriptor_t hubd;
	  usb_device_descriptor_t devd;
	  usb_device_qualifier_t odevd;
	  usb_config_descriptor_t confd;
	  usb_interface_descriptor_t ifcd;
	  usb_endpoint_descriptor_t endpd;
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

			u.devd = ehci_devd;
#if 0
			USETW(u.devd.idVendor,
			      sc->sc_id_vendor);
#endif
			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &u, totlen);
			break;
		/*
		 * We can't really operate at another speed, 
		 * but the specification says we need this
		 * descriptor:
		 */
		case UDESC_DEVICE_QUALIFIER:
			if((value & 0xff) != 0)
			{
				err = USBD_IOERROR;
				goto done;
			}
			totlen = min(len, sizeof(ehci_odevd));
			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &ehci_odevd, totlen);
			break;
		/*
		 * We can't really operate at another speed, 
		 * but the specification says we need this 
		 * descriptor:
		 */
		case UDESC_OTHER_SPEED_CONFIGURATION:
		case UDESC_CONFIG:
			if((value & 0xff) != 0)
			{
				err = USBD_IOERROR;
				goto done;
			}
			totlen = l = min(len, sizeof(u.confd));
			len -= l;

			u.confd = ehci_confd;
			u.confd.bDescriptorType = (value >> 8);

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req),
				     &u, l);

			l = min(len, sizeof(ehci_ifcd));
			totlen += l;
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req) +
				     sizeof(u.confd), &ehci_ifcd, l);

			l = min(len, sizeof(ehci_endpd));
			totlen += l;
			len -= l;

			usbd_copy_in(&(xfer->buf_data), sizeof(u.req) +
				     sizeof(u.confd) + sizeof(u.ifcd),
				     &ehci_endpd, l);
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
			       "EHCI root hub");
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
		DPRINTFN(8, ("UR_CLEAR_PORT_FEATURE\n"));

		if((index < 1) ||
		   (index > sc->sc_noport))
		{
		    err = USBD_IOERROR;
		    goto done;
		}

		port = EHCI_PORTSC(index);
		v = EOREAD4(sc, port) &~ EHCI_PS_CLEAR;
		switch(value) {
		case UHF_PORT_ENABLE:
			EOWRITE4(sc, port, v &~ EHCI_PS_PE);
			break;
		case UHF_PORT_SUSPEND:
			EOWRITE4(sc, port, v &~ EHCI_PS_SUSP);
			break;
		case UHF_PORT_POWER:
			EOWRITE4(sc, port, v &~ EHCI_PS_PP);
			break;
		case UHF_PORT_TEST:
			DPRINTFN(2,("clear port test "
				    "%d\n", index));
			break;
		case UHF_PORT_INDICATOR:
			DPRINTFN(2,("clear port ind "
				    "%d\n", index));
			EOWRITE4(sc, port, v &~ EHCI_PS_PIC);
			break;
		case UHF_C_PORT_CONNECTION:
			EOWRITE4(sc, port, v | EHCI_PS_CSC);
			break;
		case UHF_C_PORT_ENABLE:
			EOWRITE4(sc, port, v | EHCI_PS_PEC);
			break;
		case UHF_C_PORT_SUSPEND:
			/* how? */
			break;
		case UHF_C_PORT_OVER_CURRENT:
			EOWRITE4(sc, port, v | EHCI_PS_OCC);
			break;
		case UHF_C_PORT_RESET:
			sc->sc_isreset = 0;
			break;
		default:
			err = USBD_IOERROR;
			goto done;
		}
#if 0
		switch(value) {
		case UHF_C_PORT_CONNECTION:
		case UHF_C_PORT_ENABLE:
		case UHF_C_PORT_SUSPEND:
		case UHF_C_PORT_OVER_CURRENT:
		case UHF_C_PORT_RESET:
			/* Enable RHSC interrupt if condition is cleared. */
			if((OREAD4(sc, port) >> 16) == 0)
			{
				mtx_lock(&sc->sc_bus.mtx);
				ehci_pcd_enable(sc);
			}
			break;
		default:
			break;
		}
#endif
		break;
	case C(UR_GET_DESCRIPTOR, UT_READ_CLASS_DEVICE):
		if((value & 0xff) != 0)
		{
		    err = USBD_IOERROR;
		    goto done;
		}

		v = EOREAD4(sc, EHCI_HCSPARAMS);

		u.hubd = ehci_hubd;
		u.hubd.bNbrPorts = sc->sc_noport;
		USETW(u.hubd.wHubCharacteristics,
		      (EHCI_HCS_PPC(v) ? UHD_PWR_INDIVIDUAL : UHD_PWR_NO_SWITCH) |
		      (EHCI_HCS_P_INDICATOR(EREAD4(sc, EHCI_HCSPARAMS)) ?
		       UHD_PORT_IND : 0));
		u.hubd.bPwrOn2PwrGood = 200; /* XXX can't find out? */
		for(l = 0; l < sc->sc_noport; l++)
		{
			/* XXX can't find out? */
			u.hubd.DeviceRemovable[l/8] &= ~(1 << (l % 8));
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
		v = EOREAD4(sc, EHCI_PORTSC(index));
		DPRINTFN(8,("port status=0x%04x\n", v));
		i = UPS_HIGH_SPEED;
		if(v & EHCI_PS_CS)	i |= UPS_CURRENT_CONNECT_STATUS;
		if(v & EHCI_PS_PE)	i |= UPS_PORT_ENABLED;
		if(v & EHCI_PS_SUSP)	i |= UPS_SUSPEND;
		if(v & EHCI_PS_OCA)	i |= UPS_OVERCURRENT_INDICATOR;
		if(v & EHCI_PS_PR)	i |= UPS_RESET;
		if(v & EHCI_PS_PP)	i |= UPS_PORT_POWER;
		USETW(u.ps.wPortStatus, i);
		i = 0;
		if(v & EHCI_PS_CSC)	i |= UPS_C_CONNECT_STATUS;
		if(v & EHCI_PS_PEC)	i |= UPS_C_PORT_ENABLED;
		if(v & EHCI_PS_OCC)	i |= UPS_C_OVERCURRENT_INDICATOR;
		if(sc->sc_isreset)	i |= UPS_C_PORT_RESET;
		USETW(u.ps.wPortChange, i);
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
		port = EHCI_PORTSC(index);
		v = EOREAD4(sc, port) &~ EHCI_PS_CLEAR;
		switch(value) {
		case UHF_PORT_ENABLE:
			EOWRITE4(sc, port, v | EHCI_PS_PE);
			break;
		case UHF_PORT_SUSPEND:
			EOWRITE4(sc, port, v | EHCI_PS_SUSP);
			break;
		case UHF_PORT_RESET:
			DPRINTFN(5,("reset port %d\n", index));
			if(EHCI_PS_IS_LOWSPEED(v))
			{
				/* Low speed device, give up ownership. */
				ehci_disown(sc, index, 1);
				break;
			}
			/* Start reset sequence. */
			v &= ~ (EHCI_PS_PE | EHCI_PS_PR);
			EOWRITE4(sc, port, v | EHCI_PS_PR);
			/* Wait for reset to complete. */
			DELAY(1000*USB_PORT_ROOT_RESET_DELAY);

			/* Terminate reset sequence. */
			EOWRITE4(sc, port, v);
			/* Wait for HC to complete reset. */
			DELAY(1000*EHCI_PORT_RESET_COMPLETE);
			v = EOREAD4(sc, port);
			DPRINTF(("ehci after reset, status=0x%08x\n", v));
			if(v & EHCI_PS_PR)
			{
				device_printf(sc->sc_bus.bdev,
					      "port reset timeout\n");
				err = USBD_TIMEOUT;
				goto done;
			}
			if(!(v & EHCI_PS_PE))
			{
				/* Not a high speed device, 
				 * give up ownership.
				 */
				ehci_disown(sc, index, 0);
				break;
			}
			sc->sc_isreset = 1;
			DPRINTF(("ehci port %d reset, status = 0x%08x\n",
				 index, v));
			break;

		case UHF_PORT_POWER:
			DPRINTFN(2,("set port power %d\n", index));
			EOWRITE4(sc, port, v | EHCI_PS_PP);
			break;

		case UHF_PORT_TEST:
			DPRINTFN(2,("set port test %d\n", index));
			break;

		case UHF_PORT_INDICATOR:
			DPRINTFN(2,("set port ind %d\n", index));
			EOWRITE4(sc, port, v | EHCI_PS_PIC);
			break;

		default:
			err = USBD_IOERROR;
			goto done;
		}
		break;
	case C(UR_CLEAR_TT_BUFFER, UT_WRITE_CLASS_OTHER):
	case C(UR_RESET_TT, UT_WRITE_CLASS_OTHER):
	case C(UR_GET_TT_STATE, UT_READ_CLASS_OTHER):
	case C(UR_STOP_TT, UT_WRITE_CLASS_OTHER):
		break;
	default:
		err = USBD_IOERROR;
		goto done;
	}

	xfer->actlen = totlen + sizeof(u.req);
	err = USBD_NORMAL_COMPLETION;

 done:
	/* transfer transferred */
	ehci_device_done(xfer, err);

	/* call callback recursively */
	__usbd_callback(xfer);

	return;
}

static void
ehci_root_ctrl_start(struct usbd_xfer *xfer)
{
	/* not used */
	return;
}

struct usbd_pipe_methods ehci_root_ctrl_methods = 
{
  .open = ehci_root_ctrl_open,
  .close = ehci_root_ctrl_close,
  .enter = ehci_root_ctrl_enter,
  .start = ehci_root_ctrl_start,
  .copy_in = usbd_std_ctrl_copy_in,
  .copy_out = usbd_std_ctrl_copy_out,
};

/*---------------------------------------------------------------------------*
 * ehci root interrupt support
 *---------------------------------------------------------------------------*/
static void
ehci_root_intr_open(struct usbd_xfer *xfer)
{
	return;
}

static void
ehci_root_intr_close(struct usbd_xfer *xfer)
{
	ehci_device_done(xfer, USBD_CANCELLED);
	return;
}

static void
ehci_root_intr_enter(struct usbd_xfer *xfer)
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
ehci_root_intr_start(struct usbd_xfer *xfer)
{
	ehci_softc_t *sc = xfer->usb_sc;

	/* only one transfer at at time 
	 * (sc_intrxfer is cleared by ehci_root_intr_done())
	 */
	sc->sc_intrxfer = xfer;
	return;
}

struct usbd_pipe_methods ehci_root_intr_methods = 
{
  .open = ehci_root_intr_open,
  .close = ehci_root_intr_close,
  .enter = ehci_root_intr_enter,
  .start = ehci_root_intr_start,
  .copy_in = usbd_std_bulk_intr_copy_in,
  .copy_out = usbd_std_bulk_intr_copy_out,
};

#define ADD_BYTES(ptr,size) ((void *)(((u_int8_t *)(ptr)) + (size)))

static usbd_status
ehci_xfer_setup(struct usbd_device *udev,
		u_int8_t iface_index,
		struct usbd_xfer **pxfer,
		const struct usbd_config *setup_start,
		const struct usbd_config *setup_end)
{
	struct usbd_page_info page_info;
	struct usbd_xfer dummy;
	ehci_softc_t *sc = EHCI_BUS2SC(udev->bus);
	const struct usbd_config *setup;
	struct usbd_memory_info *info;
	struct usbd_page *page_ptr;
	struct usbd_xfer *xfer;
	u_int32_t size[2];
	u_int32_t total_size[2];
	u_int32_t nqtd;
	u_int32_t nqh;
	u_int32_t nsitd;
	u_int32_t nitd;
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
	  nqtd = 0;
	  nqh = 0;
	  nsitd = 0;
	  nitd = 0;

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
		 * calculate nqtd and nqh !
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
		/*
		 * compute maximum number of some structures
		 */
		if((xfer->pipe->methods == &ehci_device_ctrl_methods) ||
		   (xfer->pipe->methods == &ehci_device_bulk_methods) ||
		   (xfer->pipe->methods == &ehci_device_intr_methods))
		{
			if ((xfer->pipe->methods == &ehci_device_intr_methods) &&
			    (udev->speed == USB_SPEED_HIGH))
			    usbd_std_transfer_setup(xfer, setup, 0x400, 0xC00, 3);
			else
			    usbd_std_transfer_setup(xfer, setup, 0x400, 0x400, 1);
			nqh = 1;
			nqtd = (1+ /* SETUP */ 1+ /* STATUS */
			       1  /* SHORTPKT */) +
			  (xfer->length / (EHCI_PAGE_SIZE/2)); /* DATA */
		}
		else if(xfer->pipe->methods == &ehci_device_isoc_fs_methods)
		{
			usbd_std_transfer_setup(xfer, setup, 188, 188, 1);

			if(xfer->nframes == 0)
			{
				error = USBD_ZERO_FRAMES_IN_ISOC_MODE;
				DPRINTF(("frames == 0 in isochronous mode; "
					 "endpoint 0x%02x\n", setup->endpoint));
				goto done;
			}
			if(xfer->nframes >= EHCI_VIRTUAL_FRAMELIST_COUNT)
			{
				error = USBD_INVAL;
				DPRINTF(("isochronous frame-limit "
					 "exceeded by 0x%x frames; "
					 "endpoint 0x%02x\n",
					 setup->frames - 
					 EHCI_VIRTUAL_FRAMELIST_COUNT,
					 setup->endpoint));
				goto done;
			}
			nsitd = xfer->nframes;
		}
		else if(xfer->pipe->methods == &ehci_device_isoc_hs_methods)
		{
			usbd_std_transfer_setup(xfer, setup, 0x400, 0xC00, 3);

			if(xfer->nframes == 0)
			{
				error = USBD_ZERO_FRAMES_IN_ISOC_MODE;
				DPRINTF(("frames == 0 in isochronous mode; "
					 "endpoint 0x%02x\n", setup->endpoint));
				goto done;
			}
			if(xfer->nframes >= (8*EHCI_VIRTUAL_FRAMELIST_COUNT))
			{
				error = USBD_INVAL;
				DPRINTF(("isochronous frame-limit "
					 "exceeded by 0x%x frames; "
					 "endpoint 0x%02x\n",
					 setup->frames - 
					 (8*EHCI_VIRTUAL_FRAMELIST_COUNT),
					 setup->endpoint));
				goto done;
			}
			nitd = (xfer->nframes + 7) / 8;
		}
		else
		{
			usbd_std_transfer_setup(xfer, setup, 0x400, 0x400, 1);
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

	  /* memory is allocated at 
	   * highest alignment 
	   * which is first
	   */
	  if(nitd)
	  {
		/* align data */
		size[1] += ((-size[1]) & (EHCI_ITD_ALIGN-1));
	  }

	  if(nsitd)
	  {
		/* align data */
		size[1] += ((-size[1]) & (EHCI_SITD_ALIGN-1));
	  }

	  if(nqtd)
	  {
		/* align data */
		size[1] += ((-size[1]) & (EHCI_QTD_ALIGN-1));
	  }

	  last_obj = NULL;

	  for(n = 0;
	      n < nitd;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					 sizeof(ehci_itd_t));
	    if(buf)
	    {
		register ehci_itd_t *td;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_dma_exit(page_info.page);

		td = page_info.buffer;

		/* init TD */
		td->itd_self = htole32(page_info.physaddr|EHCI_LINK_ITD);
		td->obj_next = last_obj;
		td->page = page_info.page;

		last_obj = td;

		usbd_page_dma_enter(page_info.page);
	    }
	    size[1] += sizeof(ehci_itd_t);
	  }

	  for(n = 0;
	      n < nsitd;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					 sizeof(ehci_sitd_t));
	    if(buf)
	    {
	        register ehci_sitd_t *td;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_dma_exit(page_info.page);

		td = page_info.buffer;

		/* init TD */
		td->sitd_self = htole32(page_info.physaddr|EHCI_LINK_SITD);
		td->obj_next = last_obj;
		td->page = page_info.page;

		last_obj = td;

		usbd_page_dma_enter(page_info.page);
	    }
	    size[1] += sizeof(ehci_sitd_t);
	  }

	  for(n = 0;
	      n < nqtd;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1], 
					 sizeof(ehci_qtd_t));
	    if(buf)
	    {
		register ehci_qtd_t *qtd;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_dma_exit(page_info.page);

		qtd = page_info.buffer;

		/* init TD */
		qtd->qtd_self = htole32(page_info.physaddr);
		qtd->obj_next = last_obj;
		qtd->page = page_info.page;

		last_obj = qtd;

		usbd_page_dma_enter(page_info.page);
	    }
	    size[1] += sizeof(ehci_qtd_t);
	  }

	  xfer->td_start = last_obj;

	  if(nqh)
	  {
		/* align data */
		size[1] += ((-size[1]) & (EHCI_QH_ALIGN-1));
	  }

	  last_obj = NULL;

	  for(n = 0;
	      n < nqh;
	      n++)
	  {
	    size[1] += usbd_page_fit_obj(page_ptr, size[1],
					 sizeof(ehci_qh_t));
	    if(buf)
	    {
		register ehci_qh_t *qh;

		usbd_page_get_info(page_ptr, size[1], &page_info);

		usbd_page_dma_exit(page_info.page);

		qh = page_info.buffer;

		/* init QH */
		qh->qh_self = htole32(page_info.physaddr|EHCI_LINK_QH);
		qh->obj_next = last_obj;
		qh->page = page_info.page;

		last_obj = qh;

		usbd_page_dma_enter(page_info.page);
	    }
	    size[1] += sizeof(ehci_qh_t);
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
ehci_pipe_init(struct usbd_device *udev, usb_endpoint_descriptor_t *edesc, 
               struct usbd_pipe *pipe)
{
	ehci_softc_t *sc = EHCI_BUS2SC(udev->bus);

	DPRINTFN(1, ("pipe=%p, addr=%d, endpt=%d (%d)\n",
		     pipe, udev->address,
		     edesc->bEndpointAddress, sc->sc_addr));

	if(udev->address == sc->sc_addr)
	{
		switch (edesc->bEndpointAddress)
		{
		case USB_CONTROL_ENDPOINT:
			pipe->methods = &ehci_root_ctrl_methods;
			break;
		case UE_DIR_IN | EHCI_INTR_ENDPT:
			pipe->methods = &ehci_root_intr_methods;
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
			pipe->methods = &ehci_device_ctrl_methods;
			break;
		case UE_INTERRUPT:
			pipe->methods = &ehci_device_intr_methods;
			break;
		case UE_ISOCHRONOUS:
			if(udev->speed == USB_SPEED_HIGH)
			{
				pipe->methods = &ehci_device_isoc_hs_methods;
			}
			else
			{
				pipe->methods = &ehci_device_isoc_fs_methods;
			}
			break;
		case UE_BULK:
			pipe->methods = &ehci_device_bulk_methods;
			break;
		}
	}
	return;
}

struct usbd_bus_methods ehci_bus_methods = 
{
	.pipe_init  = ehci_pipe_init,
	.xfer_setup = ehci_xfer_setup,
	.do_poll    = ehci_do_poll,
};
