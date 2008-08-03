S=     ${.CURDIR}/../../..

.PATH: $S/dev/usb2/controller

KMOD=  usb2_controller
SRCS= 
SRCS+= bus_if.h usb2_if.h device_if.h vnode_if.h 
SRCS+= opt_usb.h pci_if.h opt_bus.h card_if.h

#
# The "usb2_controller.c" file must be first to 
# resolve a "MODULE_DRIVER()" race.
#

SRCS+= usb2_controller.c

.if defined(HAS_ATMELARM)
SRCS+= at91dci_atmelarm.c
SRCS+= ohci2_atmelarm.c
.endif

SRCS+= at91dci.c
SRCS+= ehci2.c
SRCS+= ehci2_pci.c
SRCS+= musb2_otg.c
SRCS+= ohci2.c
SRCS+= ohci2_pci.c
SRCS+= uhci2.c
SRCS+= uhci2_pci.c
SRCS+= uss820dci.c
SRCS+= uss820dci_pccard.c

.include <bsd.kmod.mk>
