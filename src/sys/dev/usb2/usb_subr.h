/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _USB_SUBR_H_
#define _USB_SUBR_H_

#define USBD_STATUS_DESC(enum,value) #enum
#define USBD_STATUS(m)\
m(USBD_NORMAL_COMPLETION,=0 /* must be zero*/)\
/* errors */\
m(USBD_PENDING_REQUESTS,)\
m(USBD_NOT_STARTED,)\
m(USBD_INVAL,)\
m(USBD_NOMEM,)\
m(USBD_CANCELLED,)\
m(USBD_BAD_ADDRESS,)\
m(USBD_BAD_BUFSIZE,)\
m(USBD_BAD_FLAG,)\
m(USBD_NO_CALLBACK,)\
m(USBD_SYNC_TRANSFER_MUST_USE_DEFAULT_CALLBACK,)\
m(USBD_IN_USE,)\
m(USBD_NO_ADDR,)\
m(USBD_NO_PIPE,)\
m(USBD_ZERO_FRAMES_IN_ISOC_MODE,)\
m(USBD_SET_ADDR_FAILED,)\
m(USBD_NO_POWER,)\
m(USBD_TOO_DEEP,)\
m(USBD_IOERROR,)\
m(USBD_NOT_CONFIGURED,)\
m(USBD_TIMEOUT,)\
m(USBD_SHORT_XFER,)\
m(USBD_STALLED,)\
m(USBD_INTERRUPTED,)\
/**/

MAKE_ENUM(USBD_STATUS,
	N_USBD_STATUS);

struct usbd_xfer;
struct usbd_pipe;
struct usbd_bus;
struct usbd_config;
struct usbd_device;
struct usbd_interface;
struct usbd_memory_info;
struct __callout;
struct module;
struct bus_dma_tag;

typedef u_int8_t usbd_status;

typedef void (*usbd_callback_t)(struct usbd_xfer *);
#ifdef USB_COMPAT_OLD
typedef void (*usbd_callback)(struct usbd_xfer *, void *, usbd_status);
typedef struct usbd_xfer *usbd_xfer_handle;
typedef struct usbd_device *usbd_device_handle;
typedef struct usbd_pipe *usbd_pipe_handle;
typedef struct usbd_interface *usbd_interface_handle;
typedef void *usbd_private_handle;
#endif

struct usbd_bus_methods {
	void (*pipe_init)(struct usbd_device *udev, 
			  usb_endpoint_descriptor_t *edesc, 
			  struct usbd_pipe *pipe);
	void (*do_poll)(struct usbd_bus *);

	usbd_status (*xfer_setup)(struct usbd_device *udev,
				  u_int8_t iface_index, 
				  struct usbd_xfer **pxfer, 
				  const struct usbd_config *setup_start, 
				  const struct usbd_config *setup_end);
};

struct usbd_pipe_methods {
	void (*open)(struct usbd_xfer *xfer);
	void (*close)(struct usbd_xfer *xfer);
	void (*enter)(struct usbd_xfer *xfer);
	void (*start)(struct usbd_xfer *xfer);
};

struct usbd_port {
	usb_port_status_t	status;
	u_int16_t		power;	/* mA of current on port */
	u_int8_t		portno;
	u_int8_t		restartcnt;
	u_int8_t		last_refcount;
#define USBD_RESTART_MAX 5
	struct usbd_device     *device;	/* connected device */
	struct usbd_device     *parent;	/* the ports hub */
};

struct usbd_hub {
	usbd_status	      (*explore)(struct usbd_device *hub);
	void		       *hubsoftc;
	usb_hub_descriptor_t	hubdesc;
	struct usbd_port        ports[0];
};

/*****/

struct usbd_bus {
	/* filled by HC driver */
	device_t                bdev; /* base device, host adapter */
	struct usbd_bus_methods	*methods;

	/* filled by USB driver */
	struct usbd_port	root_port; /* dummy port for root hub */
	struct usbd_device *	devices[USB_MAX_DEVICES];
	u_int8_t		is_exploring;
	u_int8_t		wait_explore;
	u_int8_t		needs_explore;/* a hub signalled a change
					       * this variable is protected by
					       * "usb_global_lock"
					       */
	u_int8_t		use_polling;
	u_int8_t		usbrev;	/* USB revision */
#define USBREV_UNKNOWN	0
#define USBREV_PRE_1_0	1
#define USBREV_1_0	2
#define USBREV_1_1	3
#define USBREV_2_0	4
#define USBREV_STR { "unknown", "pre 1.0", "1.0", "1.1", "2.0" }

 	struct usb_device_stats	stats;
	struct mtx		mtx;
 	struct proc *		event_thread;
 	u_int			no_intrs;
};

struct usbd_interface {
#ifdef USB_COMPAT_OLD
	struct usbd_device	   *udev;
#endif
	usb_interface_descriptor_t *idesc;
	u_int8_t		    alt_index;
};

#define usbd_clear_endpoint_toggle(pipe) { \
(pipe)->clearstall = 0; (pipe)->toggle_next = 0; }

struct usbd_pipe {
#ifdef USB_COMPAT_OLD
	struct usbd_device *	  udev;
	struct usbd_xfer *	  alloc_xfer;
#endif
	usb_endpoint_descriptor_t *edesc;
	LIST_HEAD(, usbd_xfer)	  list_head;
	u_int16_t		  isoc_next;
	u_int8_t		  toggle_next;
	u_int8_t		  refcount;
	u_int8_t		  clearstall;
	u_int8_t		  iface_index;
	/* default pipe does not use ``iface_index'' */

	/* filled by HC driver */
	struct usbd_pipe_methods  *methods;
};

struct usbd_device {
	struct usbd_bus	       *bus;           /* our controller */
	struct usbd_pipe        default_pipe;  /* pipe 0 */
	usb_endpoint_descriptor_t default_ep_desc; /* for pipe 0 */
	u_int8_t		address;       /* device addess */
	u_int8_t		config;	       /* current configuration # */
	u_int8_t		depth;         /* distance from root hub */
	u_int8_t		speed;         /* low/full/high speed */
	u_int8_t		self_powered;  /* flag for self powered */
	u_int16_t		power;         /* mA the device uses */
	int16_t			langid;	       /* language for strings */
#define USBD_NOLANG (-1)
	usb_event_cookie_t	cookie;	       /* unique connection id */
	struct usbd_port *	powersrc;      /* upstream hub port, or 0 */
	struct usbd_port *	myhsport;       /* closest high speed port */
	struct usbd_device *	myhub;	       /* upstream hub */

	usb_device_descriptor_t ddesc;         /* device descriptor */

	usb_config_descriptor_t *cdesc;	       /* full config descr */
	const struct usbd_quirks *quirks;  /* device quirks, always set */
	struct usbd_hub	*	hub;           /* only if this is a hub */

	device_t                subdevs[USB_MAX_ENDPOINTS]; /* array of all sub-devices */
	device_t                subdevs_end[0];
	struct usbd_interface   ifaces[USB_MAX_ENDPOINTS]; /* array of all interfaces */
	struct usbd_interface   ifaces_end[0];
	struct usbd_pipe        pipes[USB_MAX_ENDPOINTS]; /* array of all pipes */
	struct usbd_pipe        pipes_end[0];

	u_int8_t                ifaces_no_probe[(USB_MAX_ENDPOINTS + 7) / 8];
#define USBD_SET_IFACE_NO_PROBE(udev, ii) \
  { (udev)->ifaces_no_probe[(ii) >> 3] |= (1 << ((ii) & 7)); }
#define USBD_CLR_IFACE_NO_PROBE(udev, ii) \
  { (udev)->ifaces_no_probe[(ii) >> 3] &= ~(1 << ((ii) & 7)); }
#define USBD_GET_IFACE_NO_PROBE(udev, ii) \
  ((udev)->ifaces_no_probe[(ii) >> 3] & (1 << ((ii) & 7)))

	u_int8_t                probed; /* probe state */
#define USBD_PROBED_NOTHING              0 /* default value */
#define USBD_PROBED_SPECIFIC_AND_FOUND   1
#define USBD_PROBED_IFACE_AND_FOUND      2
#define USBD_PROBED_GENERIC_AND_FOUND    3

	u_int8_t		serial[32];
 };

struct usbd_config {
	u_int8_t	type;		/* pipe type */
	u_int8_t	endpoint;	/* pipe number */

	u_int8_t	direction;	/* pipe direction */
	u_int8_t	interval;	/* interrupt interval in milliseconds;
					 * used by interrupt pipes
					 */
#define USBD_DEFAULT_INTERVAL	0

	u_int16_t	timeout;	/* milliseconds */

	u_int8_t	frames;		/* number of frames
					 * used in isochronous
					 * mode
					 */

	u_int8_t	index;	/* pipe index to use, if more
				 * than one descriptor matches
				 * type, address, direction ...
				 */

	u_int32_t	flags;	/* flags */
#define USBD_SYNCHRONOUS         0x0001 /* wait for completion */
#define USBD_FORCE_SHORT_XFER    0x0002 /* force a short packet last */
#if (USBD_SHORT_XFER_OK != 0x0004)
#define USBD_SHORT_XFER_OK       0x0004 /* allow short reads 
					 * NOTE: existing software
					 * expects USBD_SHORT_XFER_OK
					 * to have a value of 0x4. This
					 * flag is also exported by usb.h
					 */
#endif
#ifdef USB_COMPAT_OLD
#define USBD_CUSTOM_CLEARSTALL   0x0008 /* used to disable automatic clear-stall
					 * when a device reset request is needed
					 * in addition to the clear stall request
					 */
#endif
#define USBD_DEV_OPEN            0x0010
#define USBD_DEV_RECURSED_1      0x0020
#define USBD_DEV_RECURSED_2      0x0040
#define USBD_DEV_TRANSFERRING    0x0080
#define USBD_BANDWIDTH_RECLAIMED 0x0100
#define USBD_USE_POLLING         0x0200 /* used to make synchronous transfers
					 * use polling instead of sleep/wakeup
					 */
#define USBD_SELF_DESTRUCT       0x0400 /* set if callback is allowed to unsetup itself */
#define USBD_UNUSED_3            0x0800
#define USBD_UNUSED_4            0x1000
#define USBD_UNUSED_5            0x2000
#define USBD_UNUSED_6            0x4000
#define USBD_UNUSED_7            0x8000

	u_int32_t	bufsize;       	/* total pipe buffer size in bytes */
	usbd_callback_t	callback;
};

#define USBD_TRANSFER_IN_PROGRESS(xfer)		\
	((xfer)->flags & USBD_DEV_TRANSFERRING)

struct usbd_xfer {
	struct usbd_pipe *	pipe;
	struct usbd_device *	udev;
	void *			buffer;
 	void *			priv_sc;
	void *			priv_fifo;
	struct mtx *		priv_mtx;
	struct usbd_xfer *	clearstall_xfer;
	u_int32_t		length; /* bytes */
	u_int32_t		actlen; /* bytes */

	u_int32_t		flags;

	u_int32_t		timeout; /* milliseconds */
#define USBD_NO_TIMEOUT 0
#define USBD_DEFAULT_TIMEOUT 5000 /* 5000 ms = 5 seconds */

	usbd_status		error;
	usbd_callback_t		callback;

	/* for isochronous transfers */
	u_int16_t *		frlengths;
	u_int32_t		nframes;

	/*
	 * used by HC driver
	 */

	void *			usb_sc;
	struct mtx *		usb_mtx;
	struct usbd_memory_info *usb_root;
	struct thread *		usb_thread;
	u_int32_t		usb_refcount;

	/* pipe_list is used to start next transfer */

	LIST_ENTRY(usbd_xfer)	pipe_list; 

	/* interrupt_list is used to check
	 * for finished transfers
	 */

	LIST_ENTRY(usbd_xfer)	interrupt_list;

	struct __callout	timeout_handle;

	u_int8_t		address;
	u_int8_t		endpoint;
	u_int8_t		interval; /* milliseconds */

	u_int16_t		max_packet_size;

	u_int32_t		physbuffer;

	void *			td_start;
	void *			td_end;

	void *			td_transfer_first;
	void *			td_transfer_last;

	void *			qh_start;
	void *			qh_end;

	u_int16_t		qh_pos;
#ifdef USB_COMPAT_OLD
	struct usbd_xfer *	alloc_xfer; /* the real transfer */
	void *			alloc_ptr;
	u_int32_t		alloc_len;
	void *			d_copy_ptr;
	void *			d_copy_src;
	void *			d_copy_dst;
	u_int32_t		d_copy_len;
	usbd_callback		d_callback;

	void *			f_copy_ptr;
	void *			f_copy_src;
	void *			f_copy_dst;
	u_int32_t		f_copy_len;
#endif
};

typedef void (usbd_unsetup_callback_t)(struct usbd_memory_info *info);

struct usbd_memory_info {
    void *         memory_base;
    u_int32_t      memory_size;
    u_int32_t      memory_refcount;
    void *         priv_sc;
    struct mtx *   priv_mtx;
    struct mtx *   usb_mtx;
    usbd_unsetup_callback_t *priv_func;
};

struct usbd_callback_info {
    struct usbd_xfer *xfer;
    u_int32_t refcount;
};

/*---------------------------------------------------------------------------*
 * structures used by probe and attach
 *---------------------------------------------------------------------------*/
struct usb_devno {
    u_int16_t ud_vendor;
    u_int16_t ud_product;
} __packed;

#define usb_lookup(tbl, vendor, product) usb_match_device			\
	((const struct usb_devno *)(tbl), (sizeof (tbl) / sizeof ((tbl)[0])),	\
	 sizeof ((tbl)[0]), (vendor), (product))				\
/**/

#define	USB_PRODUCT_ANY		0xffff

struct usb_attach_arg
{
	int			port;
	int			configno;
	int			iface_index;
	int			vendor;
	int			product;
	int			release;
	int			matchlvl;
	struct usbd_device     *device;	/* current device */
	struct usbd_interface  *iface; /* current interface */
	int			usegeneric;
	struct usbd_interface  *ifaces_start; /* all interfaces */
	struct usbd_interface  *ifaces_end; /* exclusive */
#ifdef USB_COMPAT_OLD
	int			nifaces;
	struct usbd_interface * ifaces[USB_MAX_ENDPOINTS];
#endif
};

/* return values for device_probe() method: */

#define UMATCH_VENDOR_PRODUCT_REV			(-10)
#define UMATCH_VENDOR_PRODUCT				(-20)
#define UMATCH_VENDOR_DEVCLASS_DEVPROTO			(-30)
#define UMATCH_DEVCLASS_DEVSUBCLASS_DEVPROTO		(-40)
#define UMATCH_DEVCLASS_DEVSUBCLASS			(-50)
#define UMATCH_VENDOR_PRODUCT_REV_CONF_IFACE		(-60)
#define UMATCH_VENDOR_PRODUCT_CONF_IFACE		(-70)
#define UMATCH_VENDOR_IFACESUBCLASS_IFACEPROTO		(-80)
#define UMATCH_VENDOR_IFACESUBCLASS			(-90)
#define UMATCH_IFACECLASS_IFACESUBCLASS_IFACEPROTO	(-100)
#define UMATCH_IFACECLASS_IFACESUBCLASS			(-110)
#define UMATCH_IFACECLASS				(-120)
#define UMATCH_IFACECLASS_GENERIC			(-130)
#define UMATCH_GENERIC					(-140)
#define UMATCH_NONE					(ENXIO)

/*---------------------------------------------------------------------------*
 * prototypes
 *---------------------------------------------------------------------------*/

/* routines from usb_subr.c */

void
usbd_devinfo(struct usbd_device *udev, int showclass, char *cp);

const char *
usbd_errstr(usbd_status err);

void
usb_delay_ms(struct usbd_bus *bus, u_int ms);

void
usbd_delay_ms(struct usbd_device *udev, u_int ms);

struct usb_hid_descriptor;
struct usb_hid_descriptor *
usbd_get_hdesc(usb_config_descriptor_t *cd, usb_interface_descriptor_t *id);

usb_interface_descriptor_t *
usbd_find_idesc(usb_config_descriptor_t *cd, int iface_index, int alt_index);

usb_endpoint_descriptor_t *
usbd_find_edesc(usb_config_descriptor_t *cd, int iface_index, int alt_index,
		int endptidx);

usb_descriptor_t *
usbd_find_descriptor(usb_config_descriptor_t *cd, int type, int subtype);

#define USBD_SUBTYPE_ANY (-1)

int
usbd_get_no_alts(usb_config_descriptor_t *cd, int ifaceno);

usbd_status
usbd_search_and_set_config(struct usbd_device *udev, int no, int msg);

usbd_status
usbd_set_config_index(struct usbd_device *udev, int index, int msg);

int
usbd_fill_deviceinfo(struct usbd_device *udev, struct usb_device_info *di,
		     int usedev);

usbd_status
usbd_fill_iface_data(struct usbd_device *udev, int iface_index, int alt_index);

usbd_status
usbd_probe_and_attach(device_t parent, 
		      int port, struct usbd_port *up);

usbd_status
usbd_new_device(device_t parent, struct usbd_bus *bus, int depth,
		int speed, int port, struct usbd_port *up);

void
usbd_free_device(struct usbd_port *up, u_int8_t free_subdev);

void
usb_detach_wait(device_t dv);

void
usb_detach_wakeup(device_t dv);

struct usbd_interface *
usbd_get_iface(struct usbd_device *udev, u_int8_t iface_index);

void
usbd_set_desc(device_t dev, struct usbd_device *udev);

/* routines from usb.c */

#if 0
extern struct mtx usb_global_lock;
#else
/* XXX currently only the Giant lock can sleep */
#define usb_global_lock Giant
#endif

void
usbd_add_dev_event(int type, struct usbd_device *udev);

void
usbd_add_drv_event(int type, struct usbd_device *udev, device_t dev);

void
usb_needs_explore(struct usbd_device *udev);

extern u_int8_t usb_driver_added_refcount;

void
usb_needs_probe_and_attach(void);

#ifdef __FreeBSD__
#define device_get_dma_tag(dev) NULL

void *
usb_alloc_mem(struct bus_dma_tag *tag, u_int32_t size, u_int8_t align_power);

bus_size_t
usb_vtophys(void *ptr, u_int32_t size);

void
usb_free_mem(void *ptr, u_int32_t size);
#endif

/* routines from usb_transfer.c */

#ifdef USB_DEBUG

void
usbd_dump_iface(struct usbd_interface *iface);

void
usbd_dump_device(struct usbd_device *udev);

void
usbd_dump_queue(struct usbd_pipe *pipe);

void
usbd_dump_pipe(struct usbd_pipe *pipe);

void
usbd_dump_xfer(struct usbd_xfer *xfer);

#endif

u_int32_t
usb_get_devid(device_t dev);

struct usbd_pipe *
usbd_get_pipe(struct usbd_device *udev, u_int8_t iface_index,
	      const struct usbd_config *setup);

usbd_status
usbd_interface_count(struct usbd_device *udev, u_int8_t *count);

usbd_status
usbd_transfer_setup(struct usbd_device *udev,
		    u_int8_t iface_index,
		    struct usbd_xfer **pxfer,
		    const struct usbd_config *setup_start,
		    u_int16_t n_setup,
		    void *priv_sc,
		    struct mtx *priv_mtx,
		    usbd_unsetup_callback_t *priv_func);

void
usbd_transfer_unsetup(struct usbd_xfer **pxfer, u_int16_t n_setup);

void
usbd_start_hardware(struct usbd_xfer *xfer);

void
usbd_transfer_start_safe(struct usbd_xfer *xfer);

void
usbd_transfer_start(struct usbd_xfer *xfer);

void
usbd_transfer_stop(struct usbd_xfer *xfer);

void
__usbd_callback(struct usbd_xfer *xfer);

void
usbd_do_callback(struct usbd_callback_info *ptr, 
		 struct usbd_callback_info *limit);
void
usbd_transfer_done(struct usbd_xfer *xfer, usbd_status error);

void
usbd_transfer_enqueue(struct usbd_xfer *xfer);

void
usbd_transfer_dequeue(struct usbd_xfer *xfer);

void
usbd_default_callback(struct usbd_xfer *xfer);

usbd_status
usbd_do_request(struct usbd_device *udev, usb_device_request_t *req, void *data);

usbd_status
usbd_do_request_flags(struct usbd_device *udev, usb_device_request_t *req,
		      void *data, u_int32_t flags, int *actlen,
		      u_int32_t timeout);

void
usbd_clearstall_callback(struct usbd_xfer *xfer);

void
usbd_do_poll(struct usbd_device *udev);

void
usbd_set_polling(struct usbd_device *udev, int on);

int
usbd_ratecheck(struct timeval *last);

const struct usb_devno *
usb_match_device(const struct usb_devno *tbl, u_int nentries, u_int size,
		 u_int16_t vendor, u_int16_t product);

int
usbd_driver_load(struct module *mod, int what, void *arg);

#ifdef USB_COMPAT_OLD
usbd_status
usbd_transfer(struct usbd_xfer *xfer);

usbd_status
usbd_sync_transfer(struct usbd_xfer *xfer);

void *
usbd_alloc_buffer(struct usbd_xfer *xfer, u_int32_t size);

void
usbd_free_buffer(struct usbd_xfer *xfer);

void
usbd_get_xfer_status(struct usbd_xfer *xfer, void **priv,
		     void **buffer, u_int32_t *count, usbd_status *status);

struct usbd_xfer *
usbd_alloc_xfer(struct usbd_device *dev);

usbd_status 
usbd_free_xfer(struct usbd_xfer *xfer);

usbd_status 
usbd_open_pipe(struct usbd_interface *iface, u_int8_t address,
               u_int8_t flags, struct usbd_pipe **pipe);

usbd_status 
usbd_open_pipe_intr(struct usbd_interface *iface, u_int8_t address,
                    u_int8_t flags, struct usbd_pipe **pipe,
                    void *priv, void *buffer, u_int32_t len,
                    usbd_callback callback, int ival);

usbd_status
usbd_setup_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
                void *priv, void *buffer, u_int32_t length,
                u_int32_t flags, u_int32_t timeout,
                usbd_callback callback);

usbd_status
usbd_setup_default_xfer(struct usbd_xfer *xfer, struct usbd_device *dev,
                        void *priv, u_int32_t timeout,
                        usb_device_request_t *req, void *buffer,
                        u_int32_t length, u_int16_t flags,
                        usbd_callback callback);

usbd_status
usbd_setup_isoc_xfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
                     void *priv, u_int16_t *frlengths, u_int32_t nframes, 
		     u_int16_t flags, usbd_callback callback);

usbd_status
usbd_bulk_transfer(struct usbd_xfer *xfer, struct usbd_pipe *pipe,
                   u_int16_t flags, u_int32_t timeout, void *buf,
                   u_int32_t *size, char *lbl);

#define usbd_intr_transfer usbd_bulk_transfer

usbd_status 
usbd_abort_pipe(struct usbd_pipe *pipe);

usbd_status 
usbd_abort_default_pipe(struct usbd_device *udev);

usbd_status
usbd_close_pipe(struct usbd_pipe *pipe);

usbd_status 
usbd_clear_endpoint_stall(struct usbd_pipe *pipe);

usbd_status 
usbd_clear_endpoint_stall_async(struct usbd_pipe *pipe);

usbd_status 
usbd_endpoint_count(struct usbd_interface *iface, u_int8_t *count);

void
usbd_interface2device_handle(struct usbd_interface *iface,
                             struct usbd_device **udev);

struct usbd_device *
usbd_pipe2device_handle(struct usbd_pipe *pipe);

usbd_status 
usbd_device2interface_handle(struct usbd_device *udev,
                             u_int8_t iface_index, struct usbd_interface **iface);

usb_endpoint_descriptor_t *
usbd_interface2endpoint_descriptor(struct usbd_interface *iface, u_int8_t index);

usb_endpoint_descriptor_t *
usbd_get_endpoint_descriptor(struct usbd_interface *iface, u_int8_t address);

#endif /* USB_COMPAT_OLD */

/* routines from usb_requests.c */

usbd_status
usbreq_reset_port(struct usbd_device *udev, int port, usb_port_status_t *ps);

usbd_status
usbreq_get_desc(struct usbd_device *udev, int type, int index,
		int len, void *desc, int timeout);

usbd_status
usbreq_get_string_any(struct usbd_device *udev, int si, char *buf, int len);

usbd_status
usbreq_get_string_desc(struct usbd_device *udev, int sindex, int langid,
		       usb_string_descriptor_t *sdesc, int *plen);

usbd_status
usbreq_get_config_desc(struct usbd_device *udev, int confidx,
		       usb_config_descriptor_t *d);

usbd_status
usbreq_get_config_desc_full(struct usbd_device *udev, int conf, void *d, int size);

usbd_status
usbreq_get_device_desc(struct usbd_device *udev, usb_device_descriptor_t *d);

usbd_status
usbreq_get_interface(struct usbd_device *udev, u_int8_t iface_index,
		     u_int8_t *aiface);

usbd_status
usbreq_set_interface(struct usbd_device *udev, u_int8_t iface_index,
		     u_int8_t altno);

usbd_status
usbreq_get_device_status(struct usbd_device *udev, usb_status_t *st);

usbd_status
usbreq_get_hub_descriptor(struct usbd_device *udev, usb_hub_descriptor_t *hd);

usbd_status
usbreq_get_hub_status(struct usbd_device *udev, usb_hub_status_t *st);

usbd_status
usbreq_set_address(struct usbd_device *udev, int addr);

usbd_status
usbreq_get_port_status(struct usbd_device *udev, int port, usb_port_status_t *ps);

usbd_status
usbreq_clear_hub_feature(struct usbd_device *udev, int sel);

usbd_status
usbreq_set_hub_feature(struct usbd_device *udev, int sel);

usbd_status
usbreq_clear_port_feature(struct usbd_device *udev, int port, int sel);

usbd_status
usbreq_set_port_feature(struct usbd_device *udev, int port, int sel);

usbd_status
usbreq_set_protocol(struct usbd_device *udev, u_int8_t iface_index,
		    u_int16_t report);

#ifdef USB_COMPAT_OLD
usbd_status
usbreq_set_report_async(struct usbd_device *udev, u_int8_t iface_index,
			u_int8_t type, u_int8_t id, void *data, int len);
#endif

usbd_status
usbreq_set_report(struct usbd_device *udev, u_int8_t iface_index,
		  u_int8_t type, u_int8_t id, void *data, int len);

usbd_status
usbreq_get_report(struct usbd_device *udev, u_int8_t iface_index,
		  u_int8_t type, u_int8_t id, void *data, int len);

usbd_status
usbreq_set_idle(struct usbd_device *udev, u_int8_t iface_index,
		int duration, int id);

usbd_status
usbreq_get_report_descriptor(struct usbd_device *udev, int ifcno,
			     int size, void *d);

usbd_status
usbreq_read_report_desc(struct usbd_device *udev, u_int8_t iface_index,
 			void **descp, int *sizep, usb_malloc_type mem);

usbd_status
usbreq_set_config(struct usbd_device *udev, int conf);

usbd_status
usbreq_get_config(struct usbd_device *udev, u_int8_t *conf);

/**/
#define usbd_get_device_descriptor(udev) (&(udev)->ddesc)
#define usbd_get_config_descriptor(udev) ((udev)->cdesc)
#define usbd_get_interface_descriptor(iface) ((iface)->idesc)
#define usbd_get_interface_altindex(iface) ((iface)->alt_index)
#define usbd_get_quirks(udev) ((udev)->quirks)
#define usbd_get_speed(udev) ((udev)->speed)
#define usbd_get_hid_descriptor usbd_get_hdesc
#define usbd_set_config_no usbd_search_and_set_config

/* helper for computing offsets */
#define POINTER_TO_UNSIGNED(ptr) \
  (((u_int8_t *)(ptr)) - ((u_int8_t *)0))

#endif /* _USB_SUBR_H_ */
