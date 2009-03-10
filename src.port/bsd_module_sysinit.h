/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Tue Mar 10 17:46:51 CET 2009
 */
SYSINIT(module_load, SI_SUB_KLD, SI_ORDER_FIRST, &device_kld_load, NULL)
SYSINIT(Giant_sysinit, SI_SUB_LOCK, SI_ORDER_MIDDLE, mtx_sysinit, &Giant)
SYSINIT(cv_module_init, SI_SUB_LOCK, SI_ORDER_MIDDLE, cv_module_init, NULL)
SYSINIT(callout_module_init, SI_SUB_LOCK, SI_ORDER_MIDDLE, callout_module_init, NULL)
SYSINIT(callout_sysinit, SI_SUB_KLD, SI_ORDER_FIRST, &callout_sysinit, NULL)
SYSINIT(eventhandler_init, SI_SUB_DRIVERS, SI_ORDER_FIRST, eventhandler_init, NULL)
SYSINIT(usb2_post_init, SI_SUB_KICK_SCHEDULER, SI_ORDER_ANY, usb2_post_init, NULL)
SYSINIT(usb2_temp_init, SI_SUB_LOCK, SI_ORDER_FIRST, usb2_temp_init, NULL)
