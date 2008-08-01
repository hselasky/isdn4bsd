/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Fri Aug  1 15:46:56 CEST 2008
 */
SYSUNINIT(usb2_dev_uninit, SI_SUB_KICK_SCHEDULER, SI_ORDER_ANY, usb2_dev_uninit, NULL)
SYSUNINIT(usb2_bus_unload, SI_SUB_KLD, SI_ORDER_ANY, usb2_bus_unload, NULL)
SYSUNINIT(Giant_sysuninit, SI_SUB_LOCK, SI_ORDER_MIDDLE, mtx_sysuninit, &Giant)
SYSUNINIT(cv_module_uninit, SI_SUB_LOCK, SI_ORDER_MIDDLE, cv_module_uninit, NULL)
SYSUNINIT(callout_module_uninit, SI_SUB_LOCK, SI_ORDER_MIDDLE, callout_module_uninit, NULL)
