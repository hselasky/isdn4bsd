/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Thu Jul 24 18:38:01 CEST 2008
 */
SYSINIT(usb2_post_init, SI_SUB_KICK_SCHEDULER, SI_ORDER_ANY, usb2_post_init, NULL)
SYSINIT(usb2_dev_init, SI_SUB_KLD, SI_ORDER_FIRST, usb2_dev_init, NULL)
SYSINIT(usb2_dev_init_post, SI_SUB_KICK_SCHEDULER, SI_ORDER_FIRST, usb2_dev_init_post, NULL)
SYSINIT(usb2_quirk_init, SI_SUB_LOCK, SI_ORDER_FIRST, usb2_quirk_init, NULL)
SYSINIT(usb2_temp_init, SI_SUB_LOCK, SI_ORDER_FIRST, usb2_temp_init, NULL)
