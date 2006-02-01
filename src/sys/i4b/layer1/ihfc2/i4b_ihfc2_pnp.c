/*-
 * Copyright (c) 2000-2004 Hans Petter Selasky. All rights reserved.
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
 *
 *---------------------------------------------------------------------------
 *
 *	i4b_ihfc2_pnp.c - common ISA/PnP/PCI/USB-bus interface
 *	------------------------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <i4b/layer1/ihfc2/i4b_ihfc2.h>
#include <i4b/layer1/ihfc2/i4b_ihfc2_ext.h>

__FBSDID("$FreeBSD: $");

#ifndef PCIR_BARS
#define PCIR_BARS       0x10
#endif

#ifndef PCIR_BAR
#define PCIR_BAR(x)     (PCIR_BARS + ((x) * 4))
#endif

/*---------------------------------------------------------------------------*
 * : prototypes
 *---------------------------------------------------------------------------*/
static device_probe_t  ihfc_pnp_probe;
static device_attach_t ihfc_pnp_attach;
static device_detach_t ihfc_pnp_detach;

/*---------------------------------------------------------------------------*
 * : register bus- and device- routines
 *---------------------------------------------------------------------------*/
static devclass_t ihfc_devclass;

static driver_t ihfc_pnp_driver =
{
	.name    = "ihfc",
	.methods = (device_method_t [])
	{
          DEVMETHOD(device_probe,       ihfc_pnp_probe),
          DEVMETHOD(device_attach,      ihfc_pnp_attach),
          DEVMETHOD(device_detach,      ihfc_pnp_detach),

          DEVMETHOD(bus_print_child,    bus_generic_print_child),
          DEVMETHOD(bus_driver_added,   bus_generic_driver_added),
          { 0, 0 }
	},
	.size    = sizeof(ihfc_sc_t),
};

DRIVER_MODULE(ihfcpnp, isa , ihfc_pnp_driver, ihfc_devclass, 0, 0);
DRIVER_MODULE(ihfcpnp, pci , ihfc_pnp_driver, ihfc_devclass, 0, 0);
#ifdef IHFC_USB_ENABLED
DRIVER_MODULE(ihfcpnp, uhub, ihfc_pnp_driver, ihfc_devclass, 0, 0);
#endif
MODULE_DEPEND(ihfcpnp, isa, 1,1,1);
MODULE_DEPEND(ihfcpnp, pci, 1,1,1);
#ifdef IHFC_USB_ENABLED
MODULE_DEPEND(ihfcpnp, usb, 1,1,1);
#endif
MODULE_VERSION(ihfcpnp, 1);

/*---------------------------------------------------------------------------*
 * : database support routines and structure(s)
 *
 *
 * NOTE: ``i386 opcodes are used as database tags''. Writing a dedicated
 *       database language to initialize sc_default, saves very few bytes.
 *
 * NOTE: any quirks should be put at the end of the DBASE
 *
 * NOTE: DBASE is sensitive to order.
 *
 * NOTE: a DBASE may import one or more other DBASEs
 *
 * NOTE: structures must _not_ be declared inside ``value'', hence
 *	 these structures will be declared on the stack! (see below)
 *
 *---------------------------------------------------------------------------*/
#define I4B_DBASE(tag)							\
	static void COUNT(dbase) (register struct sc_default *def);	\
	   __typeof(COUNT(dbase)) * const tag = &COUNT(dbase);		\
	static void COUNT(dbase) (register struct sc_default *def)	\
/**/

#define I4B_DBASE_IMPORT(dbase) (dbase)(def)

#define I4B_DBASE_ADD(var, value) def->var = value

#define I4B_ADD_LED_SUPPORT(led_scheme)					\
led_scheme(I4B_LED_MACRO_0)						\
I4B_DBASE_ADD(led_inverse_mask, (0 led_scheme(I4B_LED_MACRO_1)));	\
/**/

#define I4B_LED_MACRO_0(chan,mask,led_turns_on)	\
I4B_DBASE_ADD(led_##chan##_mask,(mask));	\
/**/

#define I4B_LED_MACRO_1(chan,mask,led_turns_on)		\
NOT(led_turns_on /* when mask is ored */)(|(mask))	\
/**/

/*---------------------------------------------------------------------------*
 * : global fifo_map support macros and structure(s)
 *---------------------------------------------------------------------------*/
#define I4B_FIFO_MAP_DECLARE(name)			\
 	static const union fifo_map name		\
	  __attribute__((__section__("ihfc_fifo_maps"),	\
			 __aligned__(1),__used__))	\
/**/

/*---------------------------------------------------------------------------*
 * : global driver support macros and structure(s)
 *
 * NOTE: drivers are linked to the last dbase COUNT(dbase)
 * NOTE: the compiler does not support nameless declarations
 *       so the UNIQUE() macro is used to generate a name
 *---------------------------------------------------------------------------*/
#define I4B_DRIVER_DECLARE(type)					\
	static const struct drvr_id					\
	  __attribute__((__section__("ihfc_" #type "_id_start"),	\
			 __aligned__(1),__used__)) UNIQUE(drvr_id)	\
/**/

struct drvr_id
{
  u_int32_t       vid;  /* for ISA cards this
                         * variable is used as
                         * card number.
                         */

  void (*dbase)(struct sc_default *);
};

/*---------------------------------------------------------------------------*
 * : USB,PCI,PNP and ISA driver support macro
 *---------------------------------------------------------------------------*/
#define I4B_USB_DRIVER(args...)\
	I4B_DRIVER_DECLARE(usb) = { .dbase = &COUNT(dbase), args }

#define I4B_PCI_DRIVER(args...)\
	I4B_DRIVER_DECLARE(pci) = { .dbase = &COUNT(dbase), args }

#define I4B_PNP_DRIVER(args...)\
	I4B_DRIVER_DECLARE(pnp) = { .dbase = &COUNT(dbase), args }

#define I4B_ISA_DRIVER(args...)\
	I4B_DRIVER_DECLARE(isa) = { .dbase = &COUNT(dbase), args }

/*---------------------------------------------------------------------------*
 * : driver modules
 *---------------------------------------------------------------------------*/
/* disable modules that are not finished,
 * and that are not used by other modules
 */
#define _I4B_HFCS2M_H_
#define _I4B_PSB2152_H__
#define _I4B_PSB3186_H__

extern struct drvr_id
  ihfc_usb_id_start[0],
  ihfc_usb_id_end[0],
  ihfc_pci_id_start[0],
  ihfc_pci_id_end[0],
  ihfc_pnp_id_start[0],
  ihfc_pnp_id_end[0],
  ihfc_isa_id_start[0],
  ihfc_isa_id_end[0];

/* make sure that the names are UNIQUE() */
#include <i4b/layer1/ihfc2/i4b_count.h>

/* DEFAULT */
#include <i4b/layer1/ihfc2/i4b_default.h>

/* HFC-1/S/SP/SPCI/USB/E1 */
#include <i4b/layer1/ihfc2/i4b_hfc1.h>
#include <i4b/layer1/ihfc2/i4b_hfcs.h>
#include <i4b/layer1/ihfc2/i4b_hfcsp.h>
#include <i4b/layer1/ihfc2/i4b_hfcspci.h>
#ifdef IHFC_USB_ENABLED
#include <i4b/layer1/ihfc2/i4b_hfcsusb.h>
#endif
#include <i4b/layer1/ihfc2/i4b_hfcs2m.h>
#include <i4b/layer1/ihfc2/i4b_hfce1.h>

/* ISAC / HSCX / IPAC */
#include <i4b/layer1/ihfc2/i4b_avm_a1.h>
#include <i4b/layer1/ihfc2/i4b_avm_pnp.h>

#ifndef FREEBSD_NO_ISA
#include <i4b/layer1/ihfc2/i4b_ctx_s0P.h>
#include <i4b/layer1/ihfc2/i4b_diva.h>
#include <i4b/layer1/ihfc2/i4b_drn_ngo.h>
#include <i4b/layer1/ihfc2/i4b_dynalink.h>
#include <i4b/layer1/ihfc2/i4b_elsa_pcc16.h>
#endif

#include <i4b/layer1/ihfc2/i4b_ipac1.h>
#include <i4b/layer1/ihfc2/i4b_psb2152.h>
#include <i4b/layer1/ihfc2/i4b_psb3186.h>

#ifndef FREEBSD_NO_ISA
#include <i4b/layer1/ihfc2/i4b_tel_s08.h>
#include <i4b/layer1/ihfc2/i4b_tel_s016.h>
#include <i4b/layer1/ihfc2/i4b_tel_s0163.h>
#include <i4b/layer1/ihfc2/i4b_usr_sti.h>
#endif

/* Tiger 300 */
#include <i4b/layer1/ihfc2/i4b_tiger300_amd.h>
#include <i4b/layer1/ihfc2/i4b_tiger300_isac.h>

/* WINBOND */
#include <i4b/layer1/ihfc2/i4b_wibpci.h>
#ifdef IHFC_USB_ENABLED
#include <i4b/layer1/ihfc2/i4b_wibusb.h>
#endif

/* make sure the names are UNIQUE() */
#include <i4b/layer1/ihfc2/i4b_count.h>

/*---------------------------------------------------------------------------*
 * : Get internal value for chip specific setup
 *---------------------------------------------------------------------------*/
static u_int16_t
ihfc_get_internal(const struct internal *list, u_int16_t value, u_int16_t override)
{
	if(override)
	{
	  return override;
	}

	if(list)
	{
	  while(1)
	  {
	    if(list->value == value) break;
	    if(list->value == 0) break;

	    list++;
	  }

	  /*
	   * In the case of no
	   * match the last
	   * entry is always
	   * returned; This
	   * provides for a
	   * default internal
	   * value;
	   */

	  return list->internal;
	}

	/*
	 * If no list is present
	 * ``value'' is returned
	 * as internal value;
	 */

	return value;
}

/*---------------------------------------------------------------------------*
 * : Resource identification lookup
 *
 * NOTE: PCI devices has a device specific ID value
 *       for each MEM/IOPORT resource that must
 *       be supplied to bus_alloc_resource.
 *       ISA and PnP devices usually use the
 *	 sequence 0,1,2,3,4 ... and so on.
 *---------------------------------------------------------------------------*/
static u_int32_t
ihfc_get_rid(const u_int8_t *rid, u_int8_t number)
{
	u_int8_t start_number = number;

	while(number--)
	{
	  if(rid[number] == rid[start_number])
	  {
	    /* A duplicate RID means that the
	     * current entry is unused, hence
	     * RID values must be different!
	     * Return default RID:
	     */

	    return start_number;
	  }
	}

	/*
	 * return RID from [driver-DBASE] list
	 */

	return rid[start_number];
}

/*---------------------------------------------------------------------------*
 * : Resource allocation
 *---------------------------------------------------------------------------*/
static int
ihfc_alloc_all_resources(register ihfc_sc_t *sc, device_t dev,
			 u_int32_t flags, u_int8_t *error)
{
	__typeof(sc->sc_default.o_RES_start[0])
	  o_RES = sc->sc_default.o_RES_start[0];

	struct sc_default  *def = &sc->sc_default;
	struct resource_id *rid = &sc->sc_resources.rid[0];
static const
	struct resource_tab table[] =
	{
	  RESOURCES(RES_MACRO_4)
	};

const   struct resource_tab *ptr;

	u_int8_t number;

	/* allocate i4b-controller */
	if((sc->sc_resources.i4b_controller =
	    i4b_controller_allocate(sc->sc_default.o_PORTABLE, error)))
	{
	  /* update mutex */
	  sc->sc_mtx = &sc->sc_resources.i4b_controller->L1_lock;
	}

	/* check size of o_RES */
	if((sizeof(o_RES)*8) < IHFC_NRES)
	{
	  IHFC_ADD_ERR(error,
		       "Too many bits for o_RES!");
	}

	/* allocate all resources */
	for(ptr = &table[0];
	    ptr < &table[sizeof(table)/sizeof(table[0])];
	    ptr++)
	{
	  for(number = 0;
	      number < (ptr->number);
	      number++, o_RES >>= 1, rid++)
	  {
		/* check if resource is not
		 * needed or already allocated:
		 */
		if(!(o_RES & 1) || (rid->res))
		{
		  continue;
		}

		/* setup some defaults
		 */
		rid->type    = (ptr->type);
		rid->number  = (number);
		rid->rid     = (number);
		rid->options = RF_ACTIVE;

		/* prepare setup */
		goto *(ptr->label_pre);

	  ioport_pre:
		rid->rid     = ihfc_get_rid(&def->io_rid[0],(number));
		goto alloc;

	  memory_pre:
		rid->rid     = ihfc_get_rid(&def->mem_rid[0],(number));
		goto alloc;

	  irq_pre:
		/* IRQ sharing for PnP/ISA devices
		 * in software, is currently
		 * not supported ...
		 */
		rid->options = (def->o_PCI_DEVICE) ?
				(RF_ACTIVE|RF_SHAREABLE) : (RF_ACTIVE);
		goto alloc;

	  drq_pre:
		goto alloc;

	  alloc:
		/* allocate resource */
		if((rid->res = bus_alloc_resource
		    (dev, rid->type, &rid->rid, 0, ~0, 1, rid->options)) == 0)
		{
		  IHFC_ADD_ERR(error,
			       "Missing SYS_RES_%s (#%d,rid=0x%02x)",
			       ptr->description,
			       rid->number,
			       rid->rid);
		  /* continue allocating to
		   * see if more resources are missing
		   */
		  continue;
		}

		/* continue setup */
		goto *(ptr->label);

	  ioport:
		/* setup io_tag and io_hdl */
		sc->sc_resources.io_tag[number] = rman_get_bustag(rid->res);
		sc->sc_resources.io_hdl[number] = rman_get_bushandle(rid->res);

		/*
		 * Find internal IO-port
		 */
		sc->sc_resources.iio[number] = ihfc_get_internal
		  (def->list_iio, rman_get_start(rid->res),
		   ((flags >> 20) & 0xfff));

		IHFC_MSG("Internal I/O-address: 0x%04x\n",
			 sc->sc_resources.iio[number]);

		/*
		 * Enable ioport (PCI only)
		 */
		if(def->o_PCI_DEVICE)
		{
		  pci_enable_io(dev, SYS_RES_IOPORT);
		}
		continue;

	  memory:
		/* setup mem_tag and mem_hdl */
		sc->sc_resources.mem_tag[number] = rman_get_bustag(rid->res);
		sc->sc_resources.mem_hdl[number] = rman_get_bushandle(rid->res);

		/*
		 * Enable memory (PCI only)
		 */
		if(def->o_PCI_DEVICE)
		{
		  pci_enable_io(dev, SYS_RES_MEMORY);
		}
		continue;
	  irq:
		/*
		 * Find internal IRQ
		 */
		sc->sc_resources.iirq[number] = ihfc_get_internal
		  (def->list_iirq, rman_get_start(rid->res),
		   ((flags >> 16) & 0xf));

		IHFC_MSG("Internal IRQ-address: 0x%x\n",
			 sc->sc_resources.iirq[number]);

#if 0
		/*
		 * Do some checking
		 */
		if(rid->res->r_sharehead)
		{
		  if(!def->o_PCI_DEVICE)
		  {
		    printf("%s: This device was assigned a "
			   "shared IRQ and may not work.\n",
			   sc->sc_name);

		    if(def->d_interrupt_delay >= 2)
		    {
		      /* delay is suitable for this system.
		       * Enable polling. NOTE: IRQ handler
		       * should  not  be  freed  even   if
		       * polling is in use (unless the IRQ
		       * line  has  been   electronically
		       * disconnected)!
		       */
		      printf("%s: This device will use a pace maker "
			     "in addition to the shared IRQ!\n",
			     sc->sc_name);

		      def->o_POLLED_MODE = 1;
		    }
		  }
		}
#endif
		continue;

	  drq:
		continue;
	  }
	}

	/*
	 * MWBA(Memory Window Base Address in PC-host memory)
	 *
	 * NOTE: This MWBA must be aligned to (1<<15) bytes.
	 *
	 */
	if(def->o_HFC_MWBA)
	{
		if(!sc->sc_resources.mwba_start[0])
		{
		    sc->sc_resources.mwba_size [0] = (1<<15);
		    sc->sc_resources.mwba_start[0] = 
		      usb_alloc_mem(device_get_dma_tag(dev),(1<<15),15);

		    if(!sc->sc_resources.mwba_start[0])
		    {
			IHFC_ADD_ERR(error,
				     "Could not allocate memory(32Kbyte) "
				     "for MWBA!");
		    }
		    else
		    {
			bzero(sc->sc_resources.mwba_start[0], (1<<15));
		    }
		}
	}
	/*
	 * TIGER MWBA(Memory Window Base Address in PC-host memory)
	 *
	 * NOTE: The size of this memory can be selected, and
	 *       is currently set to 16Kbyte.
	 *       The alignment is 4 bytes.
	 */
	if(def->o_TIGER_MWBA)
	{
		if(!sc->sc_resources.mwba_start[0])
		{
		    sc->sc_resources.mwba_size [0] = (1<<14);
		    sc->sc_resources.mwba_start[0] = 
		      usb_alloc_mem(device_get_dma_tag(dev),(1<<14),2);

		    if(!sc->sc_resources.mwba_start[0])
		    {
			IHFC_ADD_ERR(error,
				     "Could not allocate memory(16Kbyte) "
				     "for MWBA!");
		    }
		    else
		    {
			/*
			 * Default is 0xff
			 */
			memset(sc->sc_resources.mwba_start[0],0xff,(1<<14));
		    }
		}
	}

	/* get physical memory address */
	if(sc->sc_resources.mwba_start[0])
	{
		sc->sc_resources.mwba_phys_start[0] =
		  usb_vtophys(sc->sc_resources.mwba_start[0],
			      sc->sc_resources.mwba_size[0]);
	}

#ifdef IHFC_USB_ENABLED
	/*
	 * USB setup
	 */
	if(def->usb)
	{
		struct usb_attach_arg   *uaa      = device_get_ivars(dev);
		struct usbd_device      *udev     = uaa->device;
		int			 err;

		/* hence USB doesn't announce
		 * any devices do it here:
		 */
		printf("%s: <%s>\n",
		       sc->sc_name, device_get_desc(dev));

		if(def->usb_length > IHFC_NUSB) {
		   def->usb_length = IHFC_NUSB;

		   /* continue setup, though this error
		    * is critical
		    */
		   IHFC_ADD_ERR(error, "def->usb_length > IHFC_NUSB");
		}

		if(def->usb_conf_no == 0)
		{
			IHFC_ADD_ERR(error, "def->usb_conf_no == 0");
			goto done;
		}

		/* set the configuration which bConfigurationValue matches
		 * def->usb_conf_no. usbd_set_config_index() will also do some
		 * setup which is needed by the USB driver.
		 */
		err = usbd_set_config_index(udev, def->usb_conf_no, 0);
		if(err) goto usb_err;

		/* set wanted alternate config
		 */
		err = usbreq_set_interface(udev, def->usb_iface_no, def->usb_alt_iface_no);
		if(err) goto usb_err;

		/* setup transfers */
		err = usbd_transfer_setup(udev, def->usb_iface_no, &sc->sc_resources.usb_xfer[0],
					  &def->usb[0], def->usb_length,
					  sc, sc->sc_mtx, NULL);
		if(err)
		{
		  usb_err:
		    /* get USB error string
		     */
		    IHFC_ADD_ERR(error, "%s", usbd_errstr(err));
		    goto done;
		}
	}
 done:
#endif /* IHFC_USB_ENABLED */

	return IHFC_IS_ERR(error);
}

static int
ihfc_post_setup(ihfc_sc_t *sc, device_t dev, u_int8_t *error)
{
	struct resource_id *rid = &sc->sc_resources.rid[IHFC_RES_IRQ_0_OFFSET];

	/*
	 * setup interrupt handler last so
	 * that interrupt doesn't occur before
	 * the softc has been setup.
	 */
	if(rid->res && !sc->sc_default.o_POLLED_MODE)
	{
	    IHFC_MSG("Setting up IRQ\n");

	    if(bus_setup_intr(dev, rid->res, INTR_TYPE_NET
			      I4B_DROP_GIANT(|INTR_MPSAFE), 
			      ((void *)(void *)CHIP_INTERRUPT),
			      sc, &sc->sc_resources.irq_tmp[0]))
	    {
	      IHFC_ADD_ERR(error,
			   "Error setting up interrupt "
			   "handler #0!");
	    }
	}
	
	return IHFC_IS_ERR(error);
}

/*---------------------------------------------------------------------------*
 * : Resource deallocation
 *
 * PCI NOTE: Busmaster should be disabled before any
 *           used memory maps are freed.
 *---------------------------------------------------------------------------*/
static void
ihfc_unsetup_resource(device_t dev)
{
	ihfc_sc_t *sc = device_get_softc(dev);
	struct resource_id *rid;
	u_int8_t x;

	IHFC_MSG("\n");

	/* free i4b-controller */
	i4b_controller_free(sc->sc_resources.i4b_controller);

	rid = &sc->sc_resources.rid[0];

	if(sc->sc_default.o_PCI_DEVICE)
	{
	      pci_disable_io(dev, SYS_RES_MEMORY);
	      pci_disable_io(dev, SYS_RES_IOPORT);
	      pci_disable_busmaster(dev);
	}

	for(x = IHFC_NRES; x--; rid++)
	{
		if(rid->res != 0)
		{
		  if(rid->type == SYS_RES_IRQ)
		  {
		      void *tmp = sc->sc_resources.irq_tmp[rid->number];

		      if(tmp)
		      {
			/* NOTE: bus_teardown_intr takes the
			 * cookie and not the address of the
			 * cookie(&irq_tmp[a]) as argument!
			 */ 
			bus_teardown_intr(dev, rid->res, tmp);
		      }
		  }

		  bus_release_resource(dev, rid->type, rid->rid, rid->res);
		}
	}

#ifdef IHFC_USB_ENABLED
	/*
	 * USB unsetup
	 */
	usbd_transfer_unsetup(&sc->sc_resources.usb_xfer[0], IHFC_NUSB);
#endif

	/* release allocated
	 * memory (must be last)
	 */
	if(sc->sc_resources.mwba_start[0])
        {
		usb_free_mem(sc->sc_resources.mwba_start[0],
			     sc->sc_resources.mwba_size[0]);
	}

	bzero(&sc->sc_resources,sizeof(sc->sc_resources));
	return;
}

/*---------------------------------------------------------------------------*
 * : unsetup/shutdown card
 *---------------------------------------------------------------------------*/
static int
ihfc_unsetup(device_t dev, u_int8_t *error, u_int8_t level)
{
	ihfc_sc_t *sc = device_get_softc(dev);
	ihfc_fifo_t *f;

	switch(level) {
	case 3:
	  ihfc_unsetup_ldev(sc);
	case 2:
	  ihfc_unsetup_i4b(sc);
	case 1:
	  ihfc_unsetup_softc(sc);
	case 0:
	  ihfc_unsetup_resource(dev);
	}

	/*
	 * Stop timeouts if running
	 */

	callout_stop(&sc->sc_pollout_timr);
	callout_stop(&sc->sc_pollout_timr_wait);
	callout_stop(&sc->sc_statemachine.T3callout);

	/*
	 * Destroy mutex (reverse order)
	 */

	mtx_destroy(&sc->sc_mtx3);

	FIFO_FOREACH(f,sc)
	{
#if 0
	  mtx_destroy(&f->ifqueue.ifq_mtx);
#endif
	}

	if(IHFC_IS_ERR(error))
	{
	  device_printf(dev,"ERROR(s): %s\n", error);
	  return ENXIO;
	}
	return 0;
}

/*---------------------------------------------------------------------------*
 * : default probe routine
 *---------------------------------------------------------------------------*/
static int
ihfc_pnp_probe(device_t dev)
{
	ihfc_sc_t *        sc =  device_get_softc(dev);
        ihfc_fifo_t *      f;
        u_int32_t       flags =  device_get_flags(dev);
        u_int8_t         name = *device_get_name(device_get_parent(dev));

	u_int8_t         error[IHFC_MAX_ERR];

        const struct drvr_id * id       = NULL;
	const struct drvr_id * id_end   = NULL;

        u_int32_t n, vid = 0, lid = 0, cid = 0;

	error[0] = 0;

	if(!sc)
	{
	  return ENOMEM;
	}

	/*
	 * For ISA cards the value returned by
	 * "device_get_flag()" is defined as follows:
	 *
	 * bits[ 0.. 7] = card number
	 * bits[ 8..15] = unused (default 0)
	 * bits[16..19] = internal irq (see hw manual or code)
	 * bits[20..31] = internal io  (see hw manual or code)
	 */

      	if(flags)
	{
		/* Probe ISA */
		id       = &ihfc_isa_id_start[0];
		id_end   = &ihfc_isa_id_end[0];

		vid = (flags & 0xff);
	}
	else
	{
	  if(name == 'i')
	  {
		/* Probe PNP */
		id       = &ihfc_pnp_id_start[0];
		id_end   = &ihfc_pnp_id_end[0];

		lid = isa_get_logicalid(dev);
		vid = isa_get_vendorid(dev);
		cid = isa_get_compatid(dev);
	  }

	  if(name == 'p')
	  {
		/* Probe PCI */
		id       = &ihfc_pci_id_start[0];
		id_end   = &ihfc_pci_id_end[0];

		vid = pci_get_devid(dev);

		/* XXX Tiger Jet is reusing 
		 * chip ID's
		 */
		if((vid == 0x0001e159) &&
		   (pci_read_config(dev, 0x2c, 4) == 0x0001b119))
		{
		    /* this is a Firewire Device */
		    return ENXIO;
		}
	  }

#ifdef IHFC_USB_ENABLED
	  if(name == 'u')
	  {
		/* Probe USB */
		id       = &ihfc_usb_id_start[0];
		id_end   = &ihfc_usb_id_end[0];

		vid = usb_get_devid(dev);
	  }
#endif
	}

	/* device_printf(dev,"ID: %08x , %08x, %08x, flags: %08x\n",
	 *   lid, vid, cid, flags);
	 *
	 * Search and find
	 * a driver
	 *
	 *
	 * if(id == NULL) return ENXIO;
	 * (will be checked below)
	 */

	 for( ; id < id_end; id++)
	  if((id->vid == vid) ||
	     (id->vid == lid) ||
	     (id->vid == cid))
	  {
	    if(id->dbase)
	    {
#if DO_I4B_DEBUG
		if(bootverbose)
		{
		    i4b_l1_debug |= L1_HFC_DBG | L1_ERROR;
		}
#endif
		/*
		 * bzero in case the softc was
		 * already used:
		 */

		bzero(sc, sizeof(*sc));

		/*
		 * call default_sc_default()
		 */
		(default_sc_default)(&sc->sc_default);

		/*
		 * call id->dbase()
		 */
		(id->dbase)(&sc->sc_default);

		/*
		 * identify chip resources
		 * ("sc" should be available through
		 *  device_get_softc())
		 */
		if(sc->sc_default.c_chip_identify)
		{
		    if((sc->sc_default.c_chip_identify)(dev))
		    {
		        return ENXIO;
		    }
		}

		/*
		 * Set description
		 * NOTE: PnP/USB cards provide a description
		 *       from the PnP/USB-(E)EPROM.
		 */
		if(sc->sc_default.desc)
		{
		    device_set_desc(dev, sc->sc_default.desc);
		}

		/*
		 * Add o_PCI_DEVICE if device
		 * is on the PCI bus:
		 */
		if(!flags)
		{
			if(name == 'p')
			{
				sc->sc_default.o_PCI_DEVICE = 1;
			}
		}

		/*
		 * Allocate resources and probe hardware
		 */

		sc->sc_device = dev;
		sc->sc_unit   = device_get_unit(dev);

	/* BUG in kern/subr_bus.c:
	 *	device_get_nameunit(dev) is not constant.
	 *	sc->sc_name   = device_get_nameunit(dev);
	 *
	 * TEMPORARY:
	 */
		snprintf(&sc->sc_nametmp[0],
			 sizeof(sc->sc_nametmp),
			 "%s", device_get_nameunit(dev));

		sc->sc_name = &sc->sc_nametmp[0];

		/*
		 * NOTE: if the last argument of callout_init is
		 * zero, Giant will be applied before timeout.
		 * Else Giant will be dropped. To make this
		 * clear the definition of callout_init
		 * should be
		 * ``callout_init(struct callout *c, int drop_giant)``
		 * instead of
		 * ``callout_init(struct callout *c, int mpsafe)``
		 * which may imply the oposite.
		 */

		callout_init(&sc->sc_statemachine.T3callout, I4B_DROP_GIANT(1+)0);
		callout_init(&sc->sc_pollout_timr_wait, I4B_DROP_GIANT(1+)0);
		callout_init(&sc->sc_pollout_timr, I4B_DROP_GIANT(1+)0);

		/*
		 * Setup mutex in case of early
		 * interrupts.
		 */

		sc->sc_mtx = &sc->sc_mtx3;

		mtx_init(&sc->sc_mtx3, sc->sc_name, "ihfc_drv", 
			 MTX_DEF|MTX_RECURSE);

		if(sc->sc_default.d_channels > IHFC_CHANNELS)
		{
		    device_printf(dev, "number of channels, receive "
				  "and transmit, is reduced to %d!\n",
				  IHFC_CHANNELS);
		    sc->sc_default.d_channels = IHFC_CHANNELS;
		}

		if(sc->sc_default.d_channels & 1)
		{
		    device_printf(dev, "warning: number of channels, receive "
				  "and transmit, is odd! (continuing)\n");
		}

		if(sc->sc_default.d_channels == 0)
		{
		    device_printf(dev, "number of channels, receive "
				  "and transmit, defaults to 6!\n");
		    sc->sc_default.d_channels = 6;
		}

		/* setup FIFO end */
		sc->sc_fifo_end =
		  &sc->sc_fifo[sc->sc_default.d_channels];

		IHFC_MSG("this system has %d transmit and receive channels",
			 sc->sc_default.d_channels);

		if(sc->sc_default.d_L1_type == 0)
		{
		    /* set default */
		    sc->sc_default.d_L1_type = L1_TYPE_ISDN_BRI;
		}

#if (IHFC_CHANNELS < 6)
#error "please update code, (IHFC_CHANNELS < 6)"
#endif
		/*
		 * Setup fifo numbers for HFC-SP chip
		 */
	        sc->sc_fifo[d1t].s_fifo_sel = 0x04; /* chip: D1 channel  */
	        sc->sc_fifo[d1r].s_fifo_sel = 0x05; /* chip: D1 channel  */
		sc->sc_fifo[b1t].s_fifo_sel = 0x00; /* chip: B1 channel  */
		sc->sc_fifo[b1r].s_fifo_sel = 0x01; /* chip: B1 channel  */
		sc->sc_fifo[b2t].s_fifo_sel = 0x02; /* chip: B2 channel  */
		sc->sc_fifo[b2r].s_fifo_sel = 0x03; /* chip: B2 channel  */
#if 0
		sc->sc_fifo[b3t].s_fifo_sel = 0x06; /* chip: PCM channel */
		sc->sc_fifo[b3r].s_fifo_sel = 0x07; /* chip: PCM channel */
#endif
		/*
		 * Set default protocol for /dev/ihfcX.XX,
		 * setup __fn for FIFO_DIR() and FIFO_NO()
		 * use and setup fifo maps for all fifos.
		 * Also initialize mtx for all ifqueues.
		 */
		FIFO_FOREACH(f,sc)
		{
		    n = f - &sc->sc_fifo[0];

		    f->__fn = n;

		    f->default_prot = P_TRANS_RING;

		    if(sc->sc_default.d_fifo_map[n])
		    {
		    	bcopy(sc->sc_default.d_fifo_map[n], &f->fm,
			      sizeof(union fifo_map));
		    }
#if 0
		    mtx_init(&f->ifqueue.ifq_mtx, sc->sc_name,"ihfc_ifq", 
			     MTX_DEF);
#endif
		}

		/*
		 * D-channel transparent mode
		 * is not supported by all chips.
		 * Use HDLC instead:
		 */
		sc->sc_fifo[d1r].default_prot = P_HDLC;
		sc->sc_fifo[d1t].default_prot = P_HDLC;
#if 0
		/* for testing purpose */
		sc->sc_default.d_interrupt_delay = hz / 10;
#endif
		/* Setup statemachine
		 * variables
		 */
		sc->sc_statemachine.L1_auto_activate_ptr = 
		  &sc->sc_statemachine.L1_auto_activate_variable;
		sc->sc_statemachine.L1_activity_ptr = 
		  &sc->sc_statemachine.L1_activity_variable;

		/*
		 * Allocate and setup
		 * all resources
		 */
		if(ihfc_alloc_all_resources(sc, dev, flags, &error[0]))
		{
		  goto err;
		}

		/*
		 * Setup softc
		 * and reset chip
		 */
		if(ihfc_setup_softc(sc, &error[0]))
		{
		  goto err;
		}

		return 0; /* success */
	    }
	  }

	return ENXIO;
 err:
	return ihfc_unsetup(dev,&error[0],0);
}

/*---------------------------------------------------------------------------*
 * : default attach routine
 *---------------------------------------------------------------------------*/
static int
ihfc_pnp_attach(device_t dev)
{
	ihfc_sc_t  *sc = device_get_softc(dev);

	u_int8_t error[IHFC_MAX_ERR];

	/* */
	error[0] = 0;

	/*
	 * Setup interrupt
	 */
	if(ihfc_post_setup(sc, dev, &error[0]))
	{
	  return ihfc_unsetup(dev, &error[0], 0);
	}

	/* NOTE: ihfc_setup_i4b will generate
	 * a new sc->sc_name that matches the
	 * mux unit. sc->sc_name will be used
	 * to generate /dev/ihfcxx in
	 * ihfc_setup_ldev. 
	 */

	if(ihfc_setup_i4b(sc, &error[0]))
	{
	  return ihfc_unsetup(dev, &error[0], 2);
	}

	if(ihfc_setup_ldev(sc, &error[0]))
	{
	  return ihfc_unsetup(dev, &error[0], 3);
	}
	return 0;
}

/*---------------------------------------------------------------------------*
 * : default shutdown routine
 *---------------------------------------------------------------------------*/
static int
ihfc_pnp_detach(device_t dev)
{
	return ihfc_unsetup(dev,"",3);
}
