.if defined(SIGBASED_IO)
SRCS+=freebsd_bussig.c
SRCS+=freebsd_tq.c
.endif
.if defined(SIMULATOR)
SRCS+=simulator_main.c
.endif
SRCS+=freebsd_atmelarm.c
SRCS+=freebsd_bus.c
SRCS+=freebsd_sx.c
SRCS+=freebsd_mutex.c
SRCS+=freebsd_sysctl.c
SRCS+=freebsd_main.c
SRCS+=freebsd_condvar.c
SRCS+=freebsd_busspace.c
SRCS+=freebsd_callout.c
SRCS+=freebsd_malloc.c
SRCS+=freebsd_proc.c
SRCS+=freebsd_misc.c
SRCS+=freebsd_busdma.c
SRCS+=freebsd_evhandler.c
SRCS+=bsd_strlcat.c
SRCS+=bsd_strlcpy.c
SRCS+=bsd_strlen.c
SRCS+=bsd_strncmp.c
SRCS+=bsd_strstr.c
SRCS+=bsd_panic.c
SRCS+=bsd_printf.c
SRCS+=bsd_bcmp.c
SRCS+=bsd_bcopy.c
SRCS+=bsd_bzero.c
SRCS+=bsd_div.c
SRCS+=bsd_memset.c
SRCS+=bsd_strcmp.c
SRCS+=bsd_strcasecmp.c
SRCS+=atmegadci.c
SRCS+=atmegadci_atmelarm.c
SRCS+=at91dci.c
# SRCS+=at91dci_atmelarm.c
SRCS+=ehci.c
# SRCS+=ehci_pci.c
SRCS+=ohci.c
# SRCS+=ohci_atmelarm.c
# SRCS+=ohci_pci.c
SRCS+=uhci.c
# SRCS+=uhci_pci.c
SRCS+=usb_controller.c
SRCS+=uss820dci.c
# SRCS+=uss820dci_pccard.c
SRCS+=musb_otg.c
SRCS+=musb_otg_atmelarm.c
SRCS+=usb_busdma.c
SRCS+=usb_compat_linux.c
SRCS+=usb_core.c
SRCS+=usb_debug.c
SRCS+=usb_device.c
SRCS+=usb_dynamic.c
SRCS+=usb_error.c
SRCS+=usb_handle_request.c
SRCS+=usb_hid.c
SRCS+=usb_hub.c
SRCS+=usb_lookup.c
SRCS+=usb_mbuf.c
SRCS+=usb_msctest.c
SRCS+=usb_parse.c
SRCS+=usb_process.c
SRCS+=usb_request.c
SRCS+=usb_sw_transfer.c
SRCS+=usb_transfer.c
SRCS+=usb_util.c
#SRCS+=umass.c
SRCS+=ustorage_fs.c
SRCS+=usb_template.c
SRCS+=usb_template_cdce.c
SRCS+=usb_template_msc.c
SRCS+=usb_template_mtp.c
#SRCS+=usic.c

