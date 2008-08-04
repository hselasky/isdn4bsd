/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Mon Aug  4 17:41:35 CEST 2008
 */
SYSINIT(usb2_dev_init, SI_SUB_KLD, SI_ORDER_FIRST, usb2_dev_init, NULL)
SYSINIT(usb2_dev_init_post, SI_SUB_KICK_SCHEDULER, SI_ORDER_FIRST, usb2_dev_init_post, NULL)
SYSINIT(usb2_post_init, SI_SUB_KICK_SCHEDULER, SI_ORDER_ANY, usb2_post_init, NULL)
SYSINIT(module_load, SI_SUB_KLD, SI_ORDER_FIRST, &device_kld_load, NULL)
SYSINIT(module_unload, SI_SUB_KLD, SI_ORDER_FIRST, &device_kld_unload, NULL)
SYSINIT(Giant_sysinit, SI_SUB_LOCK, SI_ORDER_MIDDLE, mtx_sysinit, &Giant)
SYSINIT(cv_module_init, SI_SUB_LOCK, SI_ORDER_MIDDLE, cv_module_init, NULL)
SYSINIT(callout_module_init, SI_SUB_LOCK, SI_ORDER_MIDDLE, callout_module_init, NULL)
