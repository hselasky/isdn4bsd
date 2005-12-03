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

#ifndef _OHCI_H_
#define _OHCI_H_

/*** PCI config registers ***/

#define PCI_CBMEM		0x10	/* configuration base memory */

#define PCI_INTERFACE_OHCI	0x10

/*** OHCI registers */

#define OHCI_REVISION		0x00	/* OHCI revision # */
#define  OHCI_REV_LO(rev)	((rev) & 0xf)
#define  OHCI_REV_HI(rev)	(((rev)>>4) & 0xf)
#define  OHCI_REV_LEGACY(rev)	((rev) & 0x100)

#define OHCI_CONTROL		0x04
#define  OHCI_CBSR_MASK		0x00000003 /* Control/Bulk Service Ratio */
#define  OHCI_RATIO_1_1		0x00000000
#define  OHCI_RATIO_1_2		0x00000001
#define  OHCI_RATIO_1_3		0x00000002
#define  OHCI_RATIO_1_4		0x00000003
#define  OHCI_PLE		0x00000004 /* Periodic List Enable */
#define  OHCI_IE		0x00000008 /* Isochronous Enable */
#define  OHCI_CLE		0x00000010 /* Control List Enable */
#define  OHCI_BLE		0x00000020 /* Bulk List Enable */
#define  OHCI_HCFS_MASK		0x000000c0 /* HostControllerFunctionalState */
#define  OHCI_HCFS_RESET	0x00000000
#define  OHCI_HCFS_RESUME	0x00000040
#define  OHCI_HCFS_OPERATIONAL	0x00000080
#define  OHCI_HCFS_SUSPEND	0x000000c0
#define  OHCI_IR		0x00000100 /* Interrupt Routing */
#define  OHCI_RWC		0x00000200 /* Remote Wakeup Connected */
#define  OHCI_RWE		0x00000400 /* Remote Wakeup Enabled */
#define OHCI_COMMAND_STATUS	0x08
#define  OHCI_HCR		0x00000001 /* Host Controller Reset */
#define  OHCI_CLF		0x00000002 /* Control List Filled */
#define  OHCI_BLF		0x00000004 /* Bulk List Filled */
#define  OHCI_OCR		0x00000008 /* Ownership Change Request */
#define  OHCI_SOC_MASK		0x00030000 /* Scheduling Overrun Count */
#define OHCI_INTERRUPT_STATUS	0x0c
#define  OHCI_SO		0x00000001 /* Scheduling Overrun */
#define  OHCI_WDH		0x00000002 /* Writeback Done Head */
#define  OHCI_SF		0x00000004 /* Start of Frame */
#define  OHCI_RD		0x00000008 /* Resume Detected */
#define  OHCI_UE		0x00000010 /* Unrecoverable Error */
#define  OHCI_FNO		0x00000020 /* Frame Number Overflow */
#define  OHCI_RHSC		0x00000040 /* Root Hub Status Change */
#define  OHCI_OC		0x40000000 /* Ownership Change */
#define  OHCI_MIE		0x80000000 /* Master Interrupt Enable */
#define OHCI_INTERRUPT_ENABLE	0x10
#define OHCI_INTERRUPT_DISABLE	0x14
#define OHCI_HCCA		0x18
#define OHCI_PERIOD_CURRENT_ED	0x1c
#define OHCI_CONTROL_HEAD_ED	0x20
#define OHCI_CONTROL_CURRENT_ED	0x24
#define OHCI_BULK_HEAD_ED	0x28
#define OHCI_BULK_CURRENT_ED	0x2c
#define OHCI_DONE_HEAD		0x30
#define OHCI_FM_INTERVAL	0x34
#define  OHCI_GET_IVAL(s)	((s) & 0x3fff)
#define  OHCI_GET_FSMPS(s)	(((s) >> 16) & 0x7fff)
#define  OHCI_FIT		0x80000000
#define OHCI_FM_REMAINING	0x38
#define OHCI_FM_NUMBER		0x3c
#define OHCI_PERIODIC_START	0x40
#define OHCI_LS_THRESHOLD	0x44
#define OHCI_RH_DESCRIPTOR_A	0x48
#define  OHCI_GET_NDP(s)	((s) & 0xff)
#define  OHCI_PSM		0x0100     /* Power Switching Mode */
#define  OHCI_NPS		0x0200	   /* No Power Switching */
#define  OHCI_DT		0x0400     /* Device Type */
#define  OHCI_OCPM		0x0800     /* Overcurrent Protection Mode */
#define  OHCI_NOCP		0x1000     /* No Overcurrent Protection */
#define  OHCI_GET_POTPGT(s)	((s) >> 24)
#define OHCI_RH_DESCRIPTOR_B	0x4c
#define OHCI_RH_STATUS		0x50
#define  OHCI_LPS		0x00000001 /* Local Power Status */
#define  OHCI_OCI		0x00000002 /* OverCurrent Indicator */
#define  OHCI_DRWE		0x00008000 /* Device Remote Wakeup Enable */
#define  OHCI_LPSC		0x00010000 /* Local Power Status Change */
#define  OHCI_CCIC		0x00020000 /* OverCurrent Indicator Change */
#define  OHCI_CRWE		0x80000000 /* Clear Remote Wakeup Enable */
#define OHCI_RH_PORT_STATUS(n)	(0x50 + ((n)*4)) /* 1 based indexing */

#define OHCI_LES (OHCI_PLE | OHCI_IE | OHCI_CLE | OHCI_BLE)
#define OHCI_ALL_INTRS (OHCI_SO | OHCI_WDH | OHCI_SF | OHCI_RD | OHCI_UE | \
                        OHCI_FNO | OHCI_RHSC | OHCI_OC)
#define OHCI_NORMAL_INTRS (OHCI_WDH | OHCI_RD | OHCI_UE | OHCI_RHSC)

#define OHCI_FSMPS(i) (((i-210)*6/7) << 16)
#define OHCI_PERIODIC(i) ((i)*9/10)

#define OHCI_NO_INTRS 32
struct ohci_hcca {
	__volatile__ u_int32_t	hcca_interrupt_table[OHCI_NO_INTRS];
	__volatile__ u_int32_t	hcca_frame_number;
	__volatile__ u_int32_t	hcca_done_head;
	__volatile__ u_int32_t	fill[64-OHCI_NO_INTRS-1-1];
#define OHCI_DONE_INTRS 1
} UPACKED;
#define OHCI_HCCA_SIZE 256
#define OHCI_HCCA_ALIGN 256 /* bytes */

#define OHCI_PAGE_SIZE 0x1000
#define OHCI_PAGE(x) ((x) &~ 0xfff)
#define OHCI_PAGE_OFFSET(x) ((x) & 0xfff)
#define OHCI_PAGE_MASK(x) ((x) & 0xfff)

#define OHCI_ED_ALIGN 16 /* bytes */

typedef struct ohci_ed {
	__volatile__ u_int32_t	ed_flags;
#define OHCI_ED_GET_FA(s)	((s) & 0x7f)
#define OHCI_ED_ADDRMASK	0x0000007f
#define OHCI_ED_SET_FA(s)	(s)
#define OHCI_ED_GET_EN(s)	(((s) >> 7) & 0xf)
#define OHCI_ED_SET_EN(s)	((s) << 7)
#define OHCI_ED_DIR_MASK	0x00001800
#define  OHCI_ED_DIR_TD		0x00000000
#define  OHCI_ED_DIR_OUT	0x00000800
#define  OHCI_ED_DIR_IN		0x00001000
#define OHCI_ED_SPEED		0x00002000
#define OHCI_ED_SKIP		0x00004000
#define OHCI_ED_FORMAT_GEN	0x00000000
#define OHCI_ED_FORMAT_ISO	0x00008000
#define OHCI_ED_GET_MAXP(s)	(((s) >> 16) & 0x07ff)
#define OHCI_ED_SET_MAXP(s)	((s) << 16)
#define OHCI_ED_MAXPMASK	(0x7ff << 16)
	__volatile__ u_int32_t	ed_tailp;
	__volatile__ u_int32_t	ed_headp;
#define OHCI_HALTED		0x00000001
#define OHCI_TOGGLECARRY	0x00000002
#define OHCI_HEADMASK		0xfffffffc
	__volatile__ u_int32_t	ed_next;

  /* 
   * extra information needed:
   */
	u_int32_t	ed_self;

	struct ohci_ed *next;
	struct ohci_ed *prev;

	u_int8_t fill[(-(4*sizeof(u_int32_t))
		       -(1*sizeof(u_int32_t))
		       -(2*sizeof(void *))) & (OHCI_ED_ALIGN-1)];
} UPACKED ohci_ed_t;

#define OHCI_TD_ALIGN 16 /* bytes */

typedef struct ohci_td {
	__volatile__ u_int32_t	td_flags;
#define OHCI_TD_R		0x00040000		/* Buffer Rounding  */
#define OHCI_TD_DP_MASK		0x00180000		/* Direction / PID */
#define  OHCI_TD_SETUP		0x00000000
#define  OHCI_TD_OUT		0x00080000
#define  OHCI_TD_IN		0x00100000
#define OHCI_TD_GET_DI(x)	(((x) >> 21) & 7)	/* Delay Interrupt */
#define OHCI_TD_SET_DI(x)	((x) << 21)
#define  OHCI_TD_NOINTR		0x00e00000
#define  OHCI_TD_INTR_MASK	0x00e00000
#define OHCI_TD_TOGGLE_CARRY	0x00000000
#define OHCI_TD_TOGGLE_0	0x02000000
#define OHCI_TD_TOGGLE_1	0x03000000
#define OHCI_TD_TOGGLE_MASK	0x03000000
#define OHCI_TD_GET_EC(x)	(((x) >> 26) & 3)	/* Error Count */
#define OHCI_TD_GET_CC(x)	((x) >> 28)		/* Condition Code */
#define  OHCI_TD_NOCC		0xf0000000
	__volatile__ u_int32_t	td_cbp;		/* Current Buffer Pointer */
	__volatile__ u_int32_t	td_next;	/* Next TD */
	__volatile__ u_int32_t	td_be;		/* Buffer End */

  /*
   * extra information needed:
   */
	u_int32_t		td_self;
  
	struct ohci_td *	next;

	u_int16_t		len;

	u_int8_t fill[(-(4*sizeof(u_int32_t))
		       -(1*sizeof(u_int32_t))
		       -(1*sizeof(void *))
		       -(1*sizeof(u_int16_t))) & (OHCI_TD_ALIGN-1)];
} UPACKED ohci_td_t;


#define OHCI_ITD_NOFFSET 8
#define OHCI_ITD_ALIGN 32 /* bytes */
typedef struct ohci_itd {
	__volatile__ u_int32_t	itd_flags;
#define OHCI_ITD_GET_SF(x)	((x) & 0x0000ffff)
#define OHCI_ITD_SET_SF(x)	((x) & 0xffff)
#define OHCI_ITD_GET_DI(x)	(((x) >> 21) & 7)	/* Delay Interrupt */
#define OHCI_ITD_SET_DI(x)	((x) << 21)
#define  OHCI_ITD_NOINTR	0x00e00000
#define OHCI_ITD_GET_FC(x)	((((x) >> 24) & 7)+1)	/* Frame Count */
#define OHCI_ITD_SET_FC(x)	(((x)-1) << 24)
#define OHCI_ITD_GET_CC(x)	((x) >> 28)		/* Condition Code */
#define  OHCI_ITD_NOCC		0xf0000000
	__volatile__ u_int32_t	itd_bp0;			/* Buffer Page 0 */
	__volatile__ u_int32_t	itd_next;			/* Next ITD */
	__volatile__ u_int32_t	itd_be;				/* Buffer End */
	__volatile__ u_int16_t	itd_offset[OHCI_ITD_NOFFSET];	/* Buffer offsets and Status */
#define OHCI_ITD_PAGE_SELECT	0x00001000
#define OHCI_ITD_MK_OFFS(len)	(0xe000 | ((len) & 0x1fff))
#define OHCI_ITD_PSW_LENGTH(x)	((x) & 0xfff)		/* Transfer length */
#define OHCI_ITD_PSW_GET_CC(x)	((x) >> 12)		/* Condition Code */

  /*
   * extra information needed
   */
	u_int32_t	itd_self;

	struct ohci_itd *next;

	u_int8_t frames;

	u_int8_t fill[(-(4*sizeof(u_int32_t))
		       -(OHCI_ITD_NOFFSET*sizeof(u_int16_t))
		       -(1*sizeof(u_int32_t))
		       -(1*sizeof(void *))
		       -(1*sizeof(u_int8_t))) & (OHCI_ITD_ALIGN-1)];
} UPACKED ohci_itd_t;

#define OHCI_CC_NO_ERROR		0
#define OHCI_CC_CRC			1
#define OHCI_CC_BIT_STUFFING		2
#define OHCI_CC_DATA_TOGGLE_MISMATCH	3
#define OHCI_CC_STALL			4
#define OHCI_CC_DEVICE_NOT_RESPONDING	5
#define OHCI_CC_PID_CHECK_FAILURE	6
#define OHCI_CC_UNEXPECTED_PID		7
#define OHCI_CC_DATA_OVERRUN		8
#define OHCI_CC_DATA_UNDERRUN		9
#define OHCI_CC_BUFFER_OVERRUN		12
#define OHCI_CC_BUFFER_UNDERRUN		13
#define OHCI_CC_NOT_ACCESSED		15

/* Some delay needed when changing certain registers. */
#define OHCI_ENABLE_POWER_DELAY	5
#define OHCI_READ_DESC_DELAY	5

#define OHCI_NO_EDS (2*OHCI_NO_INTRS)

typedef struct ohci_softc {
	/*
	 * hardware structures first
	 */

	struct ohci_hcca	sc_hcca;
	ohci_ed_t		sc_ctrl_start;
	ohci_ed_t		sc_bulk_start;
	ohci_ed_t		sc_isoc_start;
	ohci_ed_t		sc_intr_start[OHCI_NO_EDS];

	/*
	 * software structures
	 */
	ohci_ed_t *		sc_ctrl_p_last;
	ohci_ed_t *		sc_bulk_p_last;
	ohci_ed_t *		sc_isoc_p_last;
	ohci_ed_t *		sc_intr_p_last[OHCI_NO_EDS];
	u_int16_t		sc_intr_stat[OHCI_NO_EDS];

	struct usbd_bus		sc_bus; /* base device */
	u_int32_t		sc_physaddr;

	bus_space_tag_t iot;
	bus_space_handle_t ioh;
	bus_size_t sc_size;

#if defined(__FreeBSD__)
	void *ih;

	struct resource *io_res;
	struct resource *irq_res;
#endif
	u_int32_t sc_eintrs;		/* enabled interrupts */

	u_int8_t sc_noport;
	u_int8_t sc_addr;		/* device address */
	u_int8_t sc_conf;		/* device configuration */

	struct usbd_xfer *sc_intrxfer;

	u_int8_t sc_vendor[16];
	int sc_id_vendor;

#if defined(__NetBSD__) || defined(__OpenBSD__)
	void *sc_powerhook;		/* cookie from power hook */
	void *sc_shutdownhook;		/* cookie from shutdown hook */
#endif
	u_int32_t sc_control;		/* Preserved during suspend/standby */
	u_int32_t sc_intre;

	LIST_HEAD(, usbd_xfer) sc_interrupt_list_head;

	struct callout sc_tmo_rhsc;
} UPACKED ohci_softc_t;

usbd_status
ohci_init(ohci_softc_t *sc);

void
ohci_detach(struct ohci_softc *sc);

void
ohci_suspend(ohci_softc_t *sc);

void
ohci_resume(ohci_softc_t *sc);

void
ohci_interrupt(ohci_softc_t *sc);

#endif /* _OHCI_H_ */
