/*
 * This file contains automatically generated module data.
 * Please do not edit.
 * Date: Sat Sep 20 16:35:24 CEST 2008
 */
#define BUS_ALLOC_RESOURCE(dev, ...) \
  (((bus_alloc_resource_t *)(device_get_method(dev, "bus_alloc_resource")))(dev,## __VA_ARGS__))
#define BUS_CHILD_LOCATION_STR(dev, ...) \
  (((bus_child_location_str_t *)(device_get_method(dev, "bus_child_location_str")))(dev,## __VA_ARGS__))
#define BUS_CHILD_PNPINFO_STR(dev, ...) \
  (((bus_child_pnpinfo_str_t *)(device_get_method(dev, "bus_child_pnpinfo_str")))(dev,## __VA_ARGS__))
#define BUS_DRIVER_ADDED(dev, ...) \
  (((bus_driver_added_t *)(device_get_method(dev, "bus_driver_added")))(dev,## __VA_ARGS__))
#define BUS_FREE_RESOURCE(dev, ...) \
  (((bus_free_resource_t *)(device_get_method(dev, "bus_free_resource")))(dev,## __VA_ARGS__))
#define BUS_PRINT_CHILD(dev, ...) \
  (((bus_print_child_t *)(device_get_method(dev, "bus_print_child")))(dev,## __VA_ARGS__))
#define BUS_SETUP_INTERRUPT(dev, ...) \
  (((bus_setup_interrupt_t *)(device_get_method(dev, "bus_setup_interrupt")))(dev,## __VA_ARGS__))
#define BUS_TEARDOWN_INTERRUPT(dev, ...) \
  (((bus_teardown_interrupt_t *)(device_get_method(dev, "bus_teardown_interrupt")))(dev,## __VA_ARGS__))
#define DEVICE_ATTACH(dev, ...) \
  (((device_attach_t *)(device_get_method(dev, "device_attach")))(dev,## __VA_ARGS__))
#define DEVICE_DETACH(dev, ...) \
  (((device_detach_t *)(device_get_method(dev, "device_detach")))(dev,## __VA_ARGS__))
#define DEVICE_PROBE(dev, ...) \
  (((device_probe_t *)(device_get_method(dev, "device_probe")))(dev,## __VA_ARGS__))
#define DEVICE_RESUME(dev, ...) \
  (((device_resume_t *)(device_get_method(dev, "device_resume")))(dev,## __VA_ARGS__))
#define DEVICE_SHUTDOWN(dev, ...) \
  (((device_shutdown_t *)(device_get_method(dev, "device_shutdown")))(dev,## __VA_ARGS__))
#define DEVICE_SUSPEND(dev, ...) \
  (((device_suspend_t *)(device_get_method(dev, "device_suspend")))(dev,## __VA_ARGS__))
#define USB2_HANDLE_REQUEST(dev, ...) \
  (((usb2_handle_request_t *)(device_get_method(dev, "usb2_handle_request")))(dev,## __VA_ARGS__))
