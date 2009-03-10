/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Tue Mar 10 17:46:51 CET 2009
 */
DRIVER_MODULE(atmelarm, atmelarm, atmelarm_driver, atmelarm_devclass, atmelarm_mod_load, 0)
DRIVER_MODULE(atmegadci, atmelarm, atmegadci_driver, atmegadci_devclass, 0, 0)
DRIVER_MODULE(usbus, ohci, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, uhci, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, ehci, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, at91_udp, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, uss820, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(musbotg, atmelarm, musbotg_driver, musbotg_devclass, 0, 0)
DRIVER_MODULE(usb_linux, uhub, usb_linux_driver, usb_linux_devclass, NULL, 0)
DRIVER_MODULE(uhub, usbus, uhub_driver, uhub_devclass, 0, 0)
DRIVER_MODULE(uhub, uhub, uhub_driver, uhub_devclass, NULL, 0)
DRIVER_MODULE(ustorage_fs, uhub, ustorage_fs_driver, ustorage_fs_devclass, NULL, 0)
