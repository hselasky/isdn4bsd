/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Sun Aug 24 19:00:14 CEST 2008
 */
DRIVER_MODULE(atmelarm, atmelarm, atmelarm_driver, atmelarm_devclass, atmelarm_mod_load, 0)
DRIVER_MODULE(usbus, ohci, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, uhci, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, ehci, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, at91_udp, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(usbus, uss820, usb2_driver, usb2_devclass, 0, 0)
DRIVER_MODULE(musbotg, atmelarm, musbotg_driver, musbotg_devclass, 0, 0)
DRIVER_MODULE(usb_linux, ushub, usb_linux_driver, usb_linux_devclass, NULL, 0)
DRIVER_MODULE(ushub, usbus, uhub_driver, uhub_devclass, 0, 0)
DRIVER_MODULE(ushub, ushub, uhub_driver, uhub_devclass, NULL, 0)
DRIVER_MODULE(ustorage_fs, ushub, ustorage_fs_driver, ustorage_fs_devclass, NULL, 0)
