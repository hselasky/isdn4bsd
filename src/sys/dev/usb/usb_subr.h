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
#define	_USB_SUBR_H_

/*
 * The "USBD_STATUS" macro defines all the USB error codes.
 * "USBD_ERR_NORMAL_COMPLETION" is not an error code. The reason all
 * the error codes have been put inside a macro is so that the
 * definition can be reused at various places in the code.
 */
#define	USBD_STATUS_DESC(enum,value) #enum
#define	USBD_STATUS(m)\
m(USBD_ERR_NORMAL_COMPLETION,)\
m(USBD_ERR_PENDING_REQUESTS,)\
m(USBD_ERR_NOT_STARTED,)\
m(USBD_ERR_INVAL,)\
m(USBD_ERR_NOMEM,)\
m(USBD_ERR_CANCELLED,)\
m(USBD_ERR_BAD_ADDRESS,)\
m(USBD_ERR_BAD_BUFSIZE,)\
m(USBD_ERR_BAD_FLAG,)\
m(USBD_ERR_NO_CALLBACK,)\
m(USBD_ERR_IN_USE,)\
m(USBD_ERR_NO_ADDR,)\
m(USBD_ERR_NO_PIPE,)\
m(USBD_ERR_ZERO_NFRAMES,)\
m(USBD_ERR_ZERO_MAXP,)\
m(USBD_ERR_SET_ADDR_FAILED,)\
m(USBD_ERR_NO_POWER,)\
m(USBD_ERR_TOO_DEEP,)\
m(USBD_ERR_IOERROR,)\
m(USBD_ERR_NOT_CONFIGURED,)\
m(USBD_ERR_TIMEOUT,)\
m(USBD_ERR_SHORT_XFER,)\
m(USBD_ERR_STALLED,)\
m(USBD_ERR_INTERRUPTED,)\
m(USBD_ERR_DMA_LOAD_FAILED,)\
m(USBD_ERR_BAD_CONTEXT,)\
m(USBD_ERR_NO_ROOT_HUB,)\
m(USBD_ERR_NO_INTR_THREAD,)\
					/**/

/*
 * The MAKE_ENUM macro will transform the USBD_STATUS macro into
 * enums.
 */
MAKE_ENUM(USBD_STATUS,
    N_USBD_STATUS);

/*
 * The following macro will return the current state of an USB
 * transfer like defined by the "USBD_ST_XXX" enums.
 */
#define	USBD_GET_STATE(xfer) ((xfer)->usb_state)

/*
 * The following macro will tell if an USB transfer is currently
 * receiving or transferring data.
 */
#define	USBD_GET_DATA_ISREAD(xfer) (((xfer)->flags_int.usb_mode == \
	USB_MODE_DEVICE) ? ((xfer->endpoint & UE_DIR_IN) ? 0 : 1) : \
	((xfer->endpoint & UE_DIR_IN) ? 1 : 0))

/*
 * The following macro will return the previous element of a singly
 * linked list.
 */
#undef LIST_PREV
#define	LIST_PREV(head,elm,field) \
  (((elm) == LIST_FIRST(head)) ? ((__typeof(elm))0) : \
   ((__typeof(elm))(((uint8_t *)((elm)->field.le_prev)) - \
		    ((uint8_t *)&LIST_NEXT((__typeof(elm))0,field)))))

/* USB transfer states */

enum {
	USBD_ST_SETUP,
	USBD_ST_TRANSFERRED,
	USBD_ST_ERROR,
};

/* prototypes */

struct usbd_xfer;
struct usbd_pipe;
struct usbd_bus;
struct usbd_clone;
struct usbd_config;
struct usbd_dma_tag;
struct usbd_dma_parent_tag;
struct usbd_device;
struct usbd_hw_ep_profile;
struct usbd_interface;
struct usbd_memory_info;
struct usbd_std_root_transfer;
struct usbd_setup_params;
struct usbd_ifqueue;
struct usbd_temp_setup;
struct usb_callout;
struct usb_temp_device_desc;
struct usb_temp_data;
struct module;
struct malloc_type;
struct proc;
struct usb_device;			/* Linux compat */
struct cdev;
struct intr_event;

typedef uint8_t usbd_status_t;

typedef void (usbd_callback_t)(struct usbd_xfer *);

typedef void (usbd_std_root_transfer_func_t)(struct usbd_xfer *, struct usbd_std_root_transfer *);

/* USB modes */

enum {
	USB_MODE_HOST = 0,		/* default */
	USB_MODE_DEVICE = 1,
	USB_MODE_MAX = 2,
};

/* Standard root transfer function state argument values */

enum {
	USBD_STD_ROOT_TR_SETUP,
	USBD_STD_ROOT_TR_STATUS,
	USBD_STD_ROOT_TR_PRE_DATA,
	USBD_STD_ROOT_TR_POST_DATA,
	USBD_STD_ROOT_TR_PRE_CALLBACK,
};

/* Definition of USB contexts */

enum {
	USBD_CONTEXT_UNDEFINED,
	USBD_CONTEXT_START,
	USBD_CONTEXT_STOP,
	USBD_CONTEXT_CALLBACK,
};

/*
 * The following structure is used when generating USB descriptors
 * from USB templates.
 */
struct usbd_temp_setup {
	void   *buf;
	uint32_t size;
	uint8_t	usb_speed;
	uint8_t	self_powered;
	uint8_t	bNumEndpoints;
	uint8_t	bInterfaceNumber;
	uint8_t	bAlternateSetting;
	uint8_t	bConfigurationValue;
	usbd_status_t err;
};

/*
 * The following structure is used to define all the USB BUS
 * callbacks.
 */
struct usbd_bus_methods {

	/* USB Device and Host mode - Mandatory */

	void    (*pipe_init) (struct usbd_device *udev, usb_endpoint_descriptor_t *edesc, struct usbd_pipe *pipe);
	void    (*do_poll) (struct usbd_bus *);
	void    (*xfer_setup) (struct usbd_setup_params *parm);
	void    (*xfer_unsetup) (struct usbd_xfer *xfer);
	void    (*get_dma_delay) (struct usbd_bus *, uint32_t *pdelay);

	/* USB Device and Host mode - Optional */

	void    (*set_config) (struct usbd_device *udev, usb_config_descriptor_t *cd);

	/* USB Device mode only - Mandatory */

	void    (*get_hw_ep_profile) (struct usbd_device *udev, const struct usbd_hw_ep_profile **ppf, uint8_t ep_addr);
	void    (*set_stall) (struct usbd_device *udev, struct usbd_xfer *xfer, struct usbd_pipe *pipe);
	void    (*clear_stall) (struct usbd_device *udev, struct usbd_pipe *pipe);
	void    (*rem_wakeup_set) (struct usbd_device *udev, uint8_t is_on);

	/* USB Device mode only - Optional */
	void    (*vbus_interrupt) (struct usbd_bus *, uint8_t is_on);
};

/*
 * The following structure is used to define all the USB pipe
 * callbacks.
 */
struct usbd_pipe_methods {

	/* USB Device and Host mode: */

	void    (*open) (struct usbd_xfer *xfer);
	void    (*close) (struct usbd_xfer *xfer);
	void    (*enter) (struct usbd_xfer *xfer);
	void    (*start) (struct usbd_xfer *xfer);
};

/*
 * The following structure defines an USB port.
 */
struct usbd_port {
	uint8_t	restartcnt;
#define	USBD_RESTART_MAX 5
	uint8_t	device_index;		/* zero means not valid */
	uint8_t	usb_mode:1;		/* current USB mode */
	uint8_t	unused:7;
};

/*
 * The following structure defines how many bytes are
 * left in an 1ms USB time slot.
 */
struct usbd_fs_isoc_schedule {
	uint16_t total_bytes;
	uint8_t	frame_bytes;
	uint8_t	frame_slot;
};

/*
 * The following structure is used to keep the state of a standard
 * root transfer.
 */
struct usbd_std_root_transfer {
	usb_device_request_t req;
	struct usbd_xfer *xfer;
	uint8_t *ptr;
	uint16_t len;
	uint8_t	state;
	usbd_status_t err;
};

/*
 * The following structure defines an USB HUB.
 */
struct usbd_hub {
	struct usbd_fs_isoc_schedule fs_isoc_schedule[USB_ISOC_TIME_MAX];
	struct usbd_device *hubudev;	/* the HUB device */
	usbd_status_t (*explore) (struct usbd_device *hub);
	void   *hubsoftc;
	uint32_t uframe_usage[USB_HS_MICRO_FRAMES_MAX];
	uint16_t portpower;		/* mA per USB port */
	uint8_t	isoc_last_time;
	uint8_t	nports;
	struct usbd_port ports[0];
};

/*
 * The following structure defines physical and non kernel virtual
 * address of a memory page having size USB_PAGE_SIZE.
 */
struct usbd_page {
	bus_size_t physaddr;
	void   *buffer;			/* non Kernel Virtual Address */
};

/*
 * The following structure is used when needing the kernel virtual
 * pointer and the physical address belonging to an offset in an USB
 * page cache.
 */
struct usbd_page_search {
	void   *buffer;
	bus_size_t physaddr;
	uint32_t length;
};

/*
 * The following structure is used to keep information about a DMA
 * memory allocation.
 */
struct usbd_page_cache {

#ifdef __FreeBSD__
	bus_dma_tag_t tag;
	bus_dmamap_t map;
#endif
#ifdef __NetBSD__
	bus_dma_tag_t tag;
	bus_dmamap_t map;
	bus_dma_segment_t *p_seg;
#endif
	struct usbd_page *page_start;
	struct usbd_dma_parent_tag *tag_parent;	/* always set */
	void   *buffer;			/* virtual buffer pointer */
#ifdef __NetBSD__
	int	n_seg;
#endif
	uint32_t page_offset_buf;
	uint32_t page_offset_end;
	uint8_t	isread:1;		/* set if we are currently reading
					 * from the memory. Else write. */
	uint8_t	ismultiseg:1;		/* set if we can have multiple
					 * segments */
};

/*
 * The following structure is used when setting up an array of USB
 * transfers.
 */
struct usbd_setup_params {
	struct usbd_dma_tag *dma_tag_p;
	struct usbd_page *dma_page_ptr;
	struct usbd_page_cache *dma_page_cache_ptr;	/* these will be
							 * auto-freed */
	struct usbd_page_cache *xfer_page_cache_ptr;	/* these will not be
							 * auto-freed */
	struct usbd_device *udev;
	struct usbd_xfer *curr_xfer;
	const struct usbd_config *curr_setup;
	const struct usbd_config_sub *curr_setup_sub;
	const struct usbd_pipe_methods *methods;
	void   *buf;
	uint32_t *xfer_length_ptr;

	uint32_t size[7];
	uint32_t bufsize;
	uint32_t bufsize_max;
	uint32_t hc_max_frame_size;

	uint16_t hc_max_packet_size;
	uint8_t	hc_max_packet_count;
	uint8_t	speed;
	uint8_t	dma_tag_max;
	usbd_status_t err;
};

/*
 * The following structure keeps information about what a hardware USB
 * endpoint supports.
 */
struct usbd_hw_ep_profile {
	uint16_t max_frame_size;
	uint8_t	is_simplex:1;
	uint8_t	support_multi_buffer:1;
	uint8_t	support_bulk:1;
	uint8_t	support_control:1;
	uint8_t	support_interrupt:1;
	uint8_t	support_isochronous:1;
	uint8_t	support_in:1;		/* IN-token is supported */
	uint8_t	support_out:1;		/* OUT-token is supported */
};

/*
 * The following structure is used when trying to allocate hardware
 * endpoints for an USB configuration in USB device side mode.
 */
struct usbd_sw_ep_scratch {
	const struct usbd_hw_ep_profile *pf;
	uint16_t max_frame_size;
	uint8_t	hw_endpoint_out;
	uint8_t	hw_endpoint_in;
	uint8_t	needs_ep_type;
	uint8_t	needs_in:1;
	uint8_t	needs_out:1;
};

/*
 * The following structure is used when trying to allocate hardware
 * endpoints for an USB configuration in USB device side mode.
 */
struct usbd_hw_ep_scratch {
	struct usbd_sw_ep_scratch ep[USB_MAX_ENDPOINTS];
	struct usbd_sw_ep_scratch *ep_max;
	usb_config_descriptor_t *cd;
	struct usbd_device *udev;
	struct usbd_bus_methods *methods;
	uint8_t	bmOutAlloc[(USB_MAX_ENDPOINTS + 15) / 16];
	uint8_t	bmInAlloc[(USB_MAX_ENDPOINTS + 15) / 16];
};

/*
 * The following typedef defines the USB DMA load done callback.
 */

typedef void (usbd_dma_callback_t)(struct usbd_dma_parent_tag *udpt);

/*
 * The following structure describes the parent USB DMA tag.
 */
struct usbd_dma_parent_tag {
#ifdef __FreeBSD__
	struct cv cv[1];		/* internal condition variable */
#endif

	bus_dma_tag_t tag;		/* always set */

	struct mtx *mtx;		/* private mutex, always set */
	struct usbd_memory_info *info;	/* used by the callback function */
	usbd_dma_callback_t *func;	/* load complete callback function */
	struct usbd_dma_tag *utag_first;/* pointer to first USB DMA tag */

	uint8_t	dma_error;		/* set if DMA load operation failed */
	uint8_t	dma_bits;		/* number of DMA address lines */
	uint8_t	utag_max;		/* number of USB DMA tags */
};

/*
 * The following structure describes an USB DMA tag.
 */
struct usbd_dma_tag {
#ifdef __NetBSD__
	bus_dma_segment_t *p_seg;
#endif
	struct usbd_dma_parent_tag *tag_parent;
	bus_dma_tag_t tag;

	uint32_t align;
	uint32_t size;
#ifdef __NetBSD__
	uint32_t n_seg;
#endif
};

/* USB BUS explore commands */

#define	USB_BUS_EXPLORE_TREE 0
#define	USB_BUS_EXPLORE_PROBE 1
#define	USB_BUS_EXPLORE_STOP 2
#define	USB_BUS_EXPLORE_SYNC 3

/**/

#define	USB_BUS_DMA_TAG_MAX 8

/* The following structure defines the explore state of an USB BUS. */

struct usbd_bus_needs {
	uint8_t	sync:1;			/* Set if explore thread sync is
					 * needed. */
	uint8_t	wakeup:1;		/* Set if explore thread is sleeping. */
	uint8_t	explore:1;		/* Set if a HUB signalled a change. */
	uint8_t	probe_attach:1;		/* Set if a HUB signalled that a new
					 * USB driver was loaded. */
	uint8_t	teardown:1;		/* Set if we are tearing down the
					 * explore thread */
	uint8_t	unused:3;
};

/*
 * The following structure defines an USB BUS. There is one USB BUS
 * for every Host or Device controller.
 */
struct usbd_bus {
	struct usb_device_stats stats;
	struct mtx mtx;			/* this mutex protects the USB
					 * hardware */
	LIST_HEAD(, usbd_xfer) intr_list_head;	/* driver interrupt list */

	device_t bdev;			/* filled by HC driver */

	struct usbd_dma_parent_tag dma_parent_tag[1];
	struct usbd_dma_tag dma_tags[USB_BUS_DMA_TAG_MAX];

	eventhandler_tag usb_clone_tag;
	struct usbd_bus_methods *methods;	/* filled by HC driver */
	struct usbd_device *devices[USB_MAX_DEVICES];
	struct proc *event_thread;
	struct cdev *usb_cdev;
	struct usbd_clone *usb_clone_root;

	uint32_t uframe_usage[USB_HS_MICRO_FRAMES_MAX];

	uint16_t isoc_time_last;	/* in milliseconds */

	struct usbd_bus_needs needs;	/* write protected by "bus->mtx" */

	uint8_t	alloc_failed;		/* Set if memory allocation failed. */
	uint8_t	driver_added_refcount;	/* Current driver generation count */
	uint8_t	usbrev;			/* USB revision */
	uint8_t	usb_clone_count;
#define	USB_BUS_MAX_CLONES 128

/* Definition of USB revisions */

#define	USBREV_UNKNOWN	0
#define	USBREV_PRE_1_0	1
#define	USBREV_1_0	2
#define	USBREV_1_1	3
#define	USBREV_2_0	4
#define	USBREV_2_5	5
#define	USBREV_STR { "unknown", "pre 1.0", "1.0", "1.1", "2.0", "2.5" }
	uint8_t	usb_name[32];
};

/*
 * The following structure defines an USB interface.
 */
struct usbd_interface {
	usb_interface_descriptor_t *idesc;
	device_t subdev;
	uint8_t	alt_index;
};

/*
 * The following structure defines an USB pipe which is equal to an
 * USB endpoint.
 */
struct usbd_pipe {
	LIST_HEAD(, usbd_xfer) list_head;

	usb_endpoint_descriptor_t *edesc;
	struct usbd_pipe_methods *methods;	/* set by HC driver */

	uint16_t isoc_next;
	uint16_t refcount;

	uint8_t	toggle_next:1;		/* next data toggle value */
	uint8_t	is_stalled:1;		/* set if pipe is stalled */
	uint8_t	unused:5;
	uint8_t	iface_index;		/* not used by "default pipe" */
};

/*
 * The following structure defines the USB device flags.
 */
struct usbd_device_flags {
	uint8_t	usb_mode:1;		/* USB mode (see USB_MODE_XXX) */
	uint8_t	self_powered:1;		/* set if USB device is self powered */
	uint8_t	suspended:1;		/* set if USB device is suspended */
	uint8_t	no_strings:1;		/* set if USB device does not support
					 * strings */
	uint8_t	detaching:1;		/* set if USB device is detaching */
	uint8_t	remote_wakeup:1;	/* set if remote wakeup is enabled */
	uint8_t	unused:2;
};

/*
 * The following structure defines an USB device. There exists one of
 * these structures for every USB device.
 */
struct usbd_device {
	struct sx default_sx[1];
	struct mtx default_mtx[1];

	struct usbd_interface ifaces[USB_MAX_INTERFACES];
	struct usbd_interface ifaces_end[0];

	struct usbd_pipe default_pipe;	/* Control Endpoint 0 */
	struct usbd_pipe pipes[USB_MAX_ENDPOINTS];
	struct usbd_pipe pipes_end[0];

#if (USB_MAX_ENDPOINTS < ((2*UE_ADDR) + 2))
#error "USB_MAX_ENDPOINTS must be increased!"
#endif

	struct usbd_bus *bus;		/* our USB BUS */
	device_t parent_dev;		/* parent device */
	struct usbd_device *parent_hub;
	const struct usbd_quirks *quirks;	/* device quirks, always set */
	usb_config_descriptor_t *cdesc;	/* full config descr */
	struct usbd_hub *hub;		/* only if this is a hub */
	device_t global_dev;
	struct usb_device *linux_dev;
	struct usbd_xfer *default_xfer[1];
	struct usb_temp_data *usb_template_ptr;

	uint16_t refcount;
#define	USB_DEV_REFCOUNT_MAX 0xffff

	uint16_t power;			/* mA the device uses */
	uint16_t langid;		/* language for strings */

	uint8_t	address;		/* device addess */
	uint8_t	device_index;		/* device index in "bus->devices" */
	uint8_t	curr_config_no;		/* current configuration # */
	uint8_t	depth;			/* distance from root HUB */
	uint8_t	speed;			/* low/full/high speed */
	uint8_t	port_index;		/* parent HUB port index */
	uint8_t	port_no;		/* parent HUB port number */
	uint8_t	hs_hub_addr;		/* high-speed HUB address */
	uint8_t	hs_port_no;		/* high-speed HUB port number */
	uint8_t	driver_added_refcount;	/* our driver added generation count */

	/* the "flags" field is write-protected by "bus->mtx" */

	struct usbd_device_flags flags;

	usb_endpoint_descriptor_t default_ep_desc;	/* for pipe 0 */
	usb_device_descriptor_t ddesc;	/* device descriptor */

	uint8_t	ifaces_no_probe[(USB_MAX_INTERFACES + 7) / 8];
#define	USBD_SET_IFACE_NO_PROBE(udev, ii) \
  { (udev)->ifaces_no_probe[(ii) >> 3] |= (1 << ((ii) & 7)); }
#define	USBD_CLR_IFACE_NO_PROBE(udev, ii) \
  { (udev)->ifaces_no_probe[(ii) >> 3] &= ~(1 << ((ii) & 7)); }
#define	USBD_GET_IFACE_NO_PROBE(udev, ii) \
  ((udev)->ifaces_no_probe[(ii) >> 3] & (1 << ((ii) & 7)))

	uint8_t	probed;			/* probe state */
#define	USBD_PROBED_NOTHING              0	/* default value */
#define	USBD_PROBED_SPECIFIC_AND_FOUND   1
#define	USBD_PROBED_IFACE_AND_FOUND      2

	uint8_t	serial[64];		/* serial number */
	uint8_t	manufacturer[64];	/* manufacturer string */
	uint8_t	product[64];		/* product string */

	union {
		struct usbd_hw_ep_scratch hw_ep_scratch[1];
		struct usbd_temp_setup temp_setup[1];
		uint8_t	data[128];
	}	scratch[1];
};

/*
 * The following structure defines a set of USB transfer flags.
 */
struct usbd_xfer_flags {
	uint8_t	force_short_xfer:1;	/* force a short transmit transfer
					 * last */
	uint8_t	short_xfer_ok:1;	/* allow short receive transfers */
	uint8_t	short_frames_ok:1;	/* allow short frames */
	uint8_t	pipe_bof:1;		/* block pipe on failure */
	uint8_t	use_polling:1;		/* poll until transfer is finished */
	uint8_t	proxy_buffer:1;		/* makes buffer size a factor of
					 * "max_frame_size" */
	uint8_t	ext_buffer:1;		/* uses external DMA buffer */
	uint8_t	manual_status:1;	/* non automatic status stage on
					 * control transfers */
	uint8_t	no_pipe_ok:1;		/* set if "USBD_ERR_NO_PIPE" error can
					 * be ignored */
	uint8_t	stall_pipe:1;		/* set if the endpoint belonging to
					 * this USB transfer should be stalled
					 * before starting this transfer! */
};

/*
 * The following structure defines a set of internal USB transfer
 * flags.
 */
struct usbd_xfer_flags_int {
	uint16_t control_rem;		/* remainder in bytes */

	uint8_t	open:1;			/* set if USB pipe has been opened */
	uint8_t	recursed_1:1;		/* see "usbd_callback_wrapper()" */
	uint8_t	recursed_2:1;		/* see "usbd_callback_wrapper()" */
	uint8_t	transferring:1;		/* set if an USB transfer is in
					 * progress */
	uint8_t	did_dma_delay:1;	/* set if we waited for HW DMA */
	uint8_t	draining:1;		/* set if we are draining an USB
					 * transfer */
	uint8_t	started:1;		/* keeps track of started or stopped */
	uint8_t	bandwidth_reclaimed:1;
	uint8_t	control_xfr:1;		/* set if control transfer */
	uint8_t	control_hdr:1;		/* set if control header should be
					 * sent */
	uint8_t	control_act:1;		/* set if control transfer is active */

	uint8_t	short_frames_ok:1;	/* filtered version */
	uint8_t	short_xfer_ok:1;	/* filtered version */
	uint8_t	bdma_enable:1;		/* filtered version (only set if
					 * hardware supports DMA) */
	uint8_t	bdma_no_post_sync:1;	/* set if the USB callback wrapper
					 * should not do the BUS-DMA post sync
					 * operation */
	uint8_t	isochronous_xfr:1;	/* set if isochronous transfer */
	uint8_t	usb_mode:1;		/* shadow copy of "udev->usb_mode" */
	uint8_t	context:2;		/* see USBD_CONTEXT_XXX */
	uint8_t	curr_dma_set:1;		/* used by USB HC/DC driver */
};

/*
 * The following structure defines the symmetric part of an USB config
 * structure.
 */
struct usbd_config_sub {
	usbd_callback_t *callback;	/* USB transfer callback */
	uint32_t bufsize;		/* total pipe buffer size in bytes */
	uint32_t frames;		/* maximum number of USB frames */
	uint16_t interval;		/* interval in milliseconds */
#define	USBD_DEFAULT_INTERVAL	0
	uint16_t timeout;		/* transfer timeout in milliseconds */
	struct usbd_xfer_flags flags;	/* transfer flags */
};

/*
 * The following structure define an USB configuration, that basically
 * is used when setting up an USB transfer.
 */
struct usbd_config {
	struct usbd_config_sub mh;	/* parameters for USB_MODE_HOST */
	struct usbd_config_sub md;	/* parameters for USB_MODE_DEVICE */
	uint8_t	type;			/* pipe type */
	uint8_t	endpoint;		/* pipe number */
	uint8_t	direction;		/* pipe direction */
	uint8_t	ep_index;		/* pipe index match to use */
	uint8_t	if_index;		/* "ifaces" index to use */
};

/*
 * The following structure defines an USB transfer.
 */
struct usbd_xfer {
	struct usb_callout timeout_handle;
	struct usbd_page_cache *buf_fixup;	/* fixup buffer(s) */
	LIST_ENTRY(usbd_xfer) interrupt_list;	/* used by HC driver */
	LIST_ENTRY(usbd_xfer) done_list;	/* used by HC driver */
	LIST_ENTRY(usbd_xfer) pipe_list;	/* used by HC driver */
	LIST_ENTRY(usbd_xfer) dma_list;/* used by BUS-DMA */

	struct usbd_page *dma_page_ptr;
	struct usbd_pipe *pipe;		/* our USB pipe */
	struct usbd_device *udev;
	struct mtx *priv_mtx;		/* cannot be changed during operation */
	struct mtx *usb_mtx;		/* used by HC driver */
	struct usbd_memory_info *usb_root;	/* used by HC driver */
	const void *usb_thread;
	/*
         * The value of "usb_thread" is used to tell who has reserved
         * the USB transfer for callback:
         *
         * case "NULL":
	 *       - "usbd_transfer_start"
         *       - "usbd_transfer_stop"
         *       - "{ehci,ohci,uhci}_interrupt"
         *       - "{ehci,ohci,uhci}_timeout"
	 *
         * case "xfer->usb_root":
         *       - "usbd_callback_intr_td"
	 *
         * Else:
         *       - the given thread in case of polling
         */
	void   *usb_sc;			/* used by HC driver */
	void   *qh_start[2];		/* used by HC driver */
	void   *td_start[2];		/* used by HC driver */
	void   *td_transfer_first;	/* used by HC driver */
	void   *td_transfer_last;	/* used by HC driver */
	void   *td_transfer_cache;	/* used by HC driver */
	void   *priv_sc;		/* device driver data pointer 1 */
	void   *priv_fifo;		/* device driver data pointer 2 */
	void   *local_buffer;
	uint32_t *frlengths;
	struct usbd_page_cache *frbuffers;
	usbd_callback_t *callback;

	uint32_t max_usb_frame_size;
	uint32_t max_data_length;
	uint32_t sumlen;		/* sum of all lengths in bytes */
	uint32_t actlen;		/* actual length in bytes */
	uint32_t timeout;		/* milliseconds */
#define	USBD_NO_TIMEOUT 0
#define	USBD_DEFAULT_TIMEOUT 5000	/* 5000 ms = 5 seconds */

	uint32_t nframes;		/* number of USB frames to transfer */
	uint32_t aframes;		/* actual number of USB frames
					 * transferred */

	uint16_t max_packet_size;
	uint16_t max_frame_size;
	uint16_t qh_pos;
	uint16_t isoc_time_complete;	/* in ms */
	uint16_t interval;		/* milliseconds */

	uint8_t	address;		/* physical USB address */
	uint8_t	endpoint;		/* physical USB endpoint */
	uint8_t	max_packet_count;
	uint8_t	usb_smask;
	uint8_t	usb_cmask;
	uint8_t	usb_uframe;
	uint8_t	usb_state;

	usbd_status_t error;

	struct usbd_xfer_flags flags;
	struct usbd_xfer_flags_int flags_int;
};

/*
 * The following structure is used to keep information about memory
 * that should be automatically freed at the moment all USB transfers
 * have been freed.
 */
struct usbd_memory_info {
	LIST_HEAD(, usbd_xfer) dma_head;
	LIST_HEAD(, usbd_xfer) done_head;
	struct usbd_dma_parent_tag dma_parent_tag;

	struct proc *done_thread;
	void   *memory_base;
	struct mtx *priv_mtx;
	struct mtx *usb_mtx;
	struct usbd_page_cache *dma_page_cache_start;
	struct usbd_page_cache *dma_page_cache_end;
	struct usbd_page_cache *xfer_page_cache_start;
	struct usbd_page_cache *xfer_page_cache_end;
	struct usbd_xfer *dma_curr_xfer;
	struct usbd_bus *bus;

	uint32_t memory_size;
	uint32_t memory_refcount;
	uint32_t setup_refcount;
	uint32_t page_size;
	uint32_t dma_nframes;		/* number of page caches to load */
	uint32_t dma_currframe;		/* currect page cache number */
	uint32_t dma_frlength_0;	/* length of page cache zero */

	uint8_t	dma_error;		/* set if virtual memory could not be
					 * loaded */
	uint8_t	dma_refcount;		/* set if we are waiting for a
					 * callback */

	uint8_t	done_sleep;		/* set if done thread is sleeping */
};

/*
 * The following structure defines a minimum re-implementation of the
 * mbuf system in the kernel.
 */
struct usbd_mbuf {
	uint8_t *cur_data_ptr;
	uint8_t *min_data_ptr;
	struct usbd_mbuf *usbd_nextpkt;
	struct usbd_mbuf *usbd_next;

	uint32_t cur_data_len;
	uint32_t max_data_len:31;
	uint32_t last_packet:1;
};

/*
 * The following structure defines a minimum re-implementation of the
 * ifqueue structure in the kernel.
 */
struct usbd_ifqueue {
	struct usbd_mbuf *ifq_head;
	struct usbd_mbuf *ifq_tail;

	int32_t	ifq_len;
	int32_t	ifq_maxlen;
};

#define	USBD_IF_ENQUEUE(ifq, m) do {		\
    (m)->usbd_nextpkt = NULL;			\
    if ((ifq)->ifq_tail == NULL)		\
        (ifq)->ifq_head = (m);			\
    else					\
        (ifq)->ifq_tail->usbd_nextpkt = (m);	\
    (ifq)->ifq_tail = (m);			\
    (ifq)->ifq_len++;				\
  } while (0)

#define	USBD_IF_DEQUEUE(ifq, m) do {				\
    (m) = (ifq)->ifq_head;					\
    if (m) {							\
        if (((ifq)->ifq_head = (m)->usbd_nextpkt) == NULL) {	\
	     (ifq)->ifq_tail = NULL;				\
	}							\
	(m)->usbd_nextpkt = NULL;				\
	(ifq)->ifq_len--;					\
    }								\
  } while (0)

#define	USBD_IF_PREPEND(ifq, m) do {		\
      (m)->usbd_nextpkt = (ifq)->ifq_head;	\
      if ((ifq)->ifq_tail == NULL) {		\
	  (ifq)->ifq_tail = (m);		\
      }						\
      (ifq)->ifq_head = (m);			\
      (ifq)->ifq_len++;				\
  } while (0)

#define	USBD_IF_QFULL(ifq)   ((ifq)->ifq_len >= (ifq)->ifq_maxlen)
#define	USBD_IF_QLEN(ifq)    ((ifq)->ifq_len)
#define	USBD_IF_POLL(ifq, m) ((m) = (ifq)->ifq_head)

#define	USBD_MBUF_RESET(m) do {			\
    (m)->cur_data_ptr = (m)->min_data_ptr;	\
    (m)->cur_data_len = (m)->max_data_len;	\
    (m)->last_packet = 0;			\
  } while (0)

/*------------------------------------------------------------------------*
 * structures used by the USB config thread system
 *------------------------------------------------------------------------*/
struct usbd_config_td_softc;
struct usbd_config_td_cc;

typedef void (usbd_config_td_command_t)(struct usbd_config_td_softc *sc, struct usbd_config_td_cc *cc, uint16_t reference);
typedef void (usbd_config_td_end_of_commands_t)(struct usbd_config_td_softc *sc);

/*
 * The following structure defines an USB config thread.
 */
struct usbd_config_td {
	struct usbd_ifqueue cmd_free;
	struct usbd_ifqueue cmd_used;

	struct proc *config_thread;
	struct mtx *p_mtx;
	void   *p_softc;
	void   *p_cmd_queue;
	usbd_config_td_end_of_commands_t *p_end_of_commands;

	uint8_t	wakeup_config_td;
	uint8_t	wakeup_config_td_gone;

	uint8_t	flag_config_td_sleep;
	uint8_t	flag_config_td_gone;
};

/*
 * The following structure defines a command that should be executed
 * using the USB config thread system.
 */
struct usbd_config_td_item {
	usbd_config_td_command_t *command_func;
	uint16_t command_ref;
} __aligned(USB_HOST_ALIGN);

/*------------------------------------------------------------------------*
 * structures used by probe and attach
 *------------------------------------------------------------------------*/
struct usb_devno {
	uint16_t ud_vendor;
	uint16_t ud_product;

	/*
	 * XXX this structure should be extended to also contain some
	 * flags --hps
	 */
} __packed;

#define	usb_lookup(tbl, vendor, product)			\
	usb_match_device((const struct usb_devno *)(tbl),	\
	  (sizeof(tbl) / sizeof((tbl)[0])), sizeof((tbl)[0]),	\
	  (vendor), (product))					\
					/**/

#define	USB_PRODUCT_ANY		0xffff

struct usb_attach_arg {
	device_t temp_dev;		/* for internal use */
	struct usbd_device *device;	/* current device */
	struct usbd_interface *iface;	/* current interface */

	uint16_t vendor;
	uint16_t product;
	uint16_t release;

	uint8_t	port;
	uint8_t	configno;
	uint8_t	iface_index;
	uint8_t	usegeneric;
	uint8_t	usb_mode:1;		/* see USB_MODE_XXX */
	uint8_t	unused:7;
};

/* return values for device_probe() method: */

#define	UMATCH_VENDOR_PRODUCT_REV			(-10)
#define	UMATCH_VENDOR_PRODUCT				(-20)
#define	UMATCH_VENDOR_DEVCLASS_DEVPROTO			(-30)
#define	UMATCH_DEVCLASS_DEVSUBCLASS_DEVPROTO		(-40)
#define	UMATCH_DEVCLASS_DEVSUBCLASS			(-50)
#define	UMATCH_VENDOR_PRODUCT_REV_CONF_IFACE		(-60)
#define	UMATCH_VENDOR_PRODUCT_CONF_IFACE		(-70)
#define	UMATCH_VENDOR_IFACESUBCLASS_IFACEPROTO		(-80)
#define	UMATCH_VENDOR_IFACESUBCLASS			(-90)
#define	UMATCH_IFACECLASS_IFACESUBCLASS_IFACEPROTO	(-100)
#define	UMATCH_IFACECLASS_IFACESUBCLASS			(-110)
#define	UMATCH_IFACECLASS				(-120)
#define	UMATCH_IFACECLASS_GENERIC			(-130)
#define	UMATCH_GENERIC					(-140)
#define	UMATCH_NONE					(ENXIO)

/*------------------------------------------------------------------------*
 *	prototypes from "usb_subr.c"
 *------------------------------------------------------------------------*/

#ifdef __FreeBSD__
#if (__FreeBSD_version >= 700020)
#define	device_get_dma_tag(dev) bus_get_dma_tag(dev)
#else
#define	device_get_dma_tag(dev) NULL	/* XXX */
#endif
#endif

typedef void (usbd_bus_mem_sub_cb_t)(struct usbd_bus *bus, struct usbd_page_cache *pc, struct usbd_page *pg, uint32_t size, uint32_t align);
typedef void (usbd_bus_mem_cb_t)(struct usbd_bus *bus, usbd_bus_mem_sub_cb_t *scb);
void	usbd_devinfo(struct usbd_device *udev, char *dst_ptr, uint16_t dst_len);
const char *usbd_errstr(usbd_status_t err);
void	usb_delay_ms(struct usbd_bus *bus, uint32_t ms);
void	usbd_delay_ms(struct usbd_device *udev, uint32_t ms);
void	usbd_pause_mtx(struct mtx *mtx, uint32_t ms);
usb_descriptor_t *usbd_desc_foreach(usb_config_descriptor_t *cd, usb_descriptor_t *desc);
usb_interface_descriptor_t *usbd_find_idesc(usb_config_descriptor_t *cd, uint8_t iface_index, uint8_t alt_index);
usb_endpoint_descriptor_t *usbd_find_edesc(usb_config_descriptor_t *cd, uint8_t iface_index, uint8_t alt_index, uint8_t ep_index);
void   *usbd_find_descriptor(struct usbd_device *udev, void *id, uint8_t iface_index, uint8_t type, uint8_t type_mask, uint8_t subtype, uint8_t subtype_mask);
uint16_t usbd_get_no_alts(usb_config_descriptor_t *cd, uint8_t ifaceno);
uint8_t	usbd_intr_schedule_adjust(struct usbd_device *udev, int16_t len, uint8_t slot);
void	usbd_fs_isoc_schedule_init_all(struct usbd_fs_isoc_schedule *fss);
uint16_t usbd_fs_isoc_schedule_isoc_time_expand(struct usbd_device *udev, struct usbd_fs_isoc_schedule **pp_start, struct usbd_fs_isoc_schedule **pp_end, uint16_t isoc_time);
uint8_t	usbd_fs_isoc_schedule_alloc(struct usbd_fs_isoc_schedule *fss, uint8_t *pstart, uint16_t len);
usbd_status_t usbd_set_config_no(struct usbd_device *udev, uint8_t no, uint8_t msg);
usbd_status_t usbd_set_config_index(struct usbd_device *udev, uint8_t index, uint8_t msg);
usbd_status_t usbd_set_alt_interface_index(struct usbd_device *udev, uint8_t iface_index, uint8_t alt_index);
int	usbd_fill_deviceinfo(struct usbd_device *udev, struct usb_device_info *di);
void	usbd_detach_device(struct usbd_device *udev, uint8_t iface_index, uint8_t free_subdev);
usbd_status_t usbd_probe_and_attach(struct usbd_device *udev, uint8_t iface_index);
usbd_status_t usbd_suspend_resume(struct usbd_device *udev, uint8_t do_suspend);

struct usbd_device *usbd_alloc_device(device_t parent_dev, struct usbd_bus *bus, struct usbd_device *parent_hub, uint8_t depth, uint8_t port_index, uint8_t port_no, uint8_t speed, uint8_t usb_mode);
void	usbd_free_device(struct usbd_device *udev);
usbd_status_t usbd_fill_iface_data(struct usbd_device *udev, uint8_t iface_index, uint8_t alt_index);
struct usbd_device *usbd_ref_device(struct usbd_bus *bus, uint8_t addr);
void	usbd_unref_device(struct usbd_device *udev);
struct usbd_interface *usbd_get_iface(struct usbd_device *udev, uint8_t iface_index);
void	usbd_set_device_desc(device_t dev);
void   *usbd_alloc_mbufs(struct malloc_type *type, struct usbd_ifqueue *ifq, uint32_t block_size, uint16_t block_number);
void	usbd_get_page(struct usbd_page_cache *cache, uint32_t offset, struct usbd_page_search *res);
void	usbd_copy_in(struct usbd_page_cache *cache, uint32_t offset, const void *ptr, uint32_t len);
void	usbd_m_copy_in(struct usbd_page_cache *cache, uint32_t dst_offset, struct mbuf *m, uint32_t src_offset, uint32_t src_len);
int	usbd_uiomove(struct usbd_page_cache *pc, struct uio *uio, uint32_t pc_offset, uint32_t len);
void	usbd_copy_out(struct usbd_page_cache *cache, uint32_t offset, void *ptr, uint32_t len);
void	usbd_bzero(struct usbd_page_cache *cache, uint32_t offset, uint32_t len);
void	usbd_dma_tag_create(struct usbd_dma_tag *udt, uint32_t size, uint32_t align);
void	usbd_dma_tag_destroy(struct usbd_dma_tag *udt);
uint8_t	usbd_pc_alloc_mem(struct usbd_page_cache *pc, struct usbd_page *pg, uint32_t size, uint32_t align);
void	usbd_pc_free_mem(struct usbd_page_cache *pc);
void	usbd_pc_load_mem(struct usbd_page_cache *pc, uint32_t size);
void	usbd_pc_cpu_invalidate(struct usbd_page_cache *pc);
void	usbd_pc_cpu_flush(struct usbd_page_cache *pc);
uint8_t	usbd_pc_dmamap_create(struct usbd_page_cache *pc, uint32_t size);
void	usbd_pc_dmamap_destroy(struct usbd_page_cache *pc);
uint8_t	usbd_make_str_desc(void *ptr, uint16_t max_len, const char *s);
uint32_t mtx_drop_recurse(struct mtx *mtx);
void	mtx_pickup_recurse(struct mtx *mtx, uint32_t recurse_level);
uint8_t	usbd_config_td_setup(struct usbd_config_td *ctd, void *priv_sc, struct mtx *priv_mtx, usbd_config_td_end_of_commands_t *p_func_eoc, uint16_t item_size, uint16_t item_count);
void	usbd_config_td_stop(struct usbd_config_td *ctd);
void	usbd_config_td_unsetup(struct usbd_config_td *ctd);
void	usbd_config_td_queue_command(struct usbd_config_td *ctd, usbd_config_td_command_t *pre_func, usbd_config_td_command_t *post_func, uint16_t command_qcount, uint16_t command_ref);
uint8_t	usbd_config_td_is_gone(struct usbd_config_td *ctd);
uint8_t	usbd_config_td_sleep(struct usbd_config_td *ctd, uint32_t timeout);
struct mbuf *usbd_ether_get_mbuf(void);
int32_t	device_delete_all_children(device_t dev);
uint16_t usbd_isoc_time_expand(struct usbd_bus *bus, uint16_t isoc_time_curr);
void	usbd_dma_tag_setup(struct usbd_dma_parent_tag *udpt, struct usbd_dma_tag *udt, bus_dma_tag_t dmat, struct mtx *mtx, usbd_dma_callback_t *func, struct usbd_memory_info *info, uint8_t ndmabits, uint8_t nudt);
struct usbd_dma_tag *usbd_dma_tag_find(struct usbd_dma_parent_tag *updt, uint32_t size, uint32_t align);
void	usbd_dma_tag_unsetup(struct usbd_dma_parent_tag *udpt);
void	usbd_bus_mem_flush_all(struct usbd_bus *bus, usbd_bus_mem_cb_t *cb);
uint8_t	usbd_bus_mem_alloc_all(struct usbd_bus *bus, bus_dma_tag_t dmat, usbd_bus_mem_cb_t *cb);
void	usbd_bus_mem_free_all(struct usbd_bus *bus, usbd_bus_mem_cb_t *cb);
uint8_t	usbd_transfer_setup_sub_malloc(struct usbd_setup_params *parm, struct usbd_page_search *info, struct usbd_page_cache **ppc, uint32_t size, uint32_t align);
struct usbd_device *usbd_bus_port_get_device(struct usbd_bus *bus, struct usbd_port *up);
void	usbd_bus_port_set_device(struct usbd_bus *bus, struct usbd_port *up, struct usbd_device *udev, uint8_t device_index);

/*------------------------------------------------------------------------*
 *	prototypes from "usb.c"
 *------------------------------------------------------------------------*/

#if 0
extern struct mtx usb_global_lock;

#else
/* XXX currently only the Giant lock can sleep */
#define	usb_global_lock Giant
#endif

void	usb_needs_explore(struct usbd_bus *bus, uint8_t what);
void	usb_needs_probe_and_attach(void);

/*------------------------------------------------------------------------*
 *	prototypes from "usb_template.c"
 *------------------------------------------------------------------------*/

void	usbd_temp_get_desc(struct usbd_device *udev, usb_device_request_t *req, const void **pPtr, uint16_t *pLen);
usbd_status_t usbd_temp_setup(struct usbd_device *udev, const struct usb_temp_device_desc *tdd);
void	usbd_temp_unsetup(struct usbd_device *udev);

/*------------------------------------------------------------------------*
 *	prototypes from "usb_transfer.c"
 *------------------------------------------------------------------------*/

#ifdef USB_DEBUG
void	usbd_dump_iface(struct usbd_interface *iface);
void	usbd_dump_device(struct usbd_device *udev);
void	usbd_dump_queue(struct usbd_pipe *pipe);
void	usbd_dump_pipe(struct usbd_pipe *pipe);
void	usbd_dump_xfer(struct usbd_xfer *xfer);

#endif

uint32_t usb_get_devid(device_t dev);
struct usbd_pipe *usbd_get_pipe_by_addr(struct usbd_device *udev, uint8_t ea_val);
struct usbd_pipe *usbd_get_pipe(struct usbd_device *udev, uint8_t iface_index, const struct usbd_config *setup);
usbd_status_t usbd_interface_count(struct usbd_device *udev, uint8_t *count);
void	usbd_transfer_setup_sub(struct usbd_setup_params *parm);
usbd_status_t usbd_transfer_setup(struct usbd_device *udev, const uint8_t *ifaces, struct usbd_xfer **pxfer, const struct usbd_config *setup_start, uint16_t n_setup, void *priv_sc, struct mtx *priv_mtx);
void	usbd_transfer_unsetup(struct usbd_xfer **pxfer, uint16_t n_setup);
void	usbd_std_root_transfer(struct usbd_std_root_transfer *std, usbd_std_root_transfer_func_t *func);
void	usbd_start_hardware(struct usbd_xfer *xfer);
void	usbd_bdma_pre_sync(struct usbd_xfer *xfer);
void	usbd_bdma_post_sync(struct usbd_xfer *xfer);
void	usbd_transfer_start(struct usbd_xfer *xfer);
void	usbd_transfer_stop(struct usbd_xfer *xfer);
void	usbd_transfer_drain(struct usbd_xfer *xfer);
void	usbd_set_frame_data(struct usbd_xfer *xfer, void *ptr, uint32_t frindex);
void	usbd_set_frame_offset(struct usbd_xfer *xfer, uint32_t offset, uint32_t frindex);
void	usbd_callback_wrapper(struct usbd_xfer *xfer, void *ctd, uint8_t context);
void	usbd_transfer_clear_stall(struct usbd_xfer *xfer);
void	usbd_transfer_set_stall(struct usbd_xfer *xfer);
void	usbd_transfer_intr_enqueue(struct usbd_xfer *xfer);
void	usbd_transfer_enqueue(struct usbd_xfer *xfer);
void	usbd_transfer_dequeue(struct usbd_xfer *xfer, usbd_status_t error);
void	usbd_default_transfer_setup(struct usbd_device *udev);
usbd_status_t usbd_do_request_flags(struct usbd_device *udev, struct mtx *mtx, usb_device_request_t *req, void *data, uint32_t flags, uint16_t *actlen, uint32_t timeout);
void	usbd_fill_get_report(usb_device_request_t *req, uint8_t iface_no, uint8_t type, uint8_t id, uint16_t size);
void	usbd_fill_set_report(usb_device_request_t *req, uint8_t iface_no, uint8_t type, uint8_t id, uint16_t size);
void	usbd_clear_data_toggle(struct usbd_device *udev, struct usbd_pipe *pipe);
uint8_t	usbd_clear_stall_callback(struct usbd_xfer *xfer1, struct usbd_xfer *xfer2);
void	usbd_do_poll(struct usbd_device *udev);
void	usbd_set_polling(struct usbd_device *udev, int32_t on);
const struct usb_devno *usb_match_device(const struct usb_devno *tbl, uint32_t nentries, uint32_t size, uint16_t vendor, uint16_t product);
int32_t	usbd_driver_load(struct module *mod, int32_t what, void *arg);

/*------------------------------------------------------------------------*
 *	prototypes from "usb_requests.c"
 *------------------------------------------------------------------------*/

usbd_status_t usbreq_reset_port(struct usbd_device *udev, struct mtx *mtx, uint8_t port);
usbd_status_t usbreq_get_desc(struct usbd_device *udev, struct mtx *mtx, void *desc, uint16_t min_len, uint16_t max_len, uint16_t id, uint8_t type, uint8_t index, uint8_t retries);
usbd_status_t usbreq_get_string_any(struct usbd_device *udev, struct mtx *mtx, char *buf, uint16_t len, uint8_t string_index);
usbd_status_t usbreq_get_string_desc(struct usbd_device *udev, struct mtx *mtx, void *sdesc, uint16_t max_len, uint16_t lang_id, uint8_t string_index);
usbd_status_t usbreq_get_config_desc(struct usbd_device *udev, struct mtx *mtx, usb_config_descriptor_t *d, uint8_t conf_index);
usbd_status_t usbreq_get_config_desc_full(struct usbd_device *udev, struct mtx *mtx, void *d, uint16_t size, uint8_t conf_index);
usbd_status_t usbreq_get_device_desc(struct usbd_device *udev, struct mtx *mtx, usb_device_descriptor_t *d);
usbd_status_t usbreq_get_alt_interface_no(struct usbd_device *udev, struct mtx *mtx, uint8_t *alt_iface_no, uint8_t iface_index);
usbd_status_t usbreq_set_alt_interface_no(struct usbd_device *udev, struct mtx *mtx, uint8_t iface_index, uint8_t alt_no);
usbd_status_t usbreq_get_device_status(struct usbd_device *udev, struct mtx *mtx, usb_status_t *st);
usbd_status_t usbreq_get_hub_descriptor(struct usbd_device *udev, struct mtx *mtx, usb_hub_descriptor_t *hd);
usbd_status_t usbreq_get_hub_status(struct usbd_device *udev, struct mtx *mtx, usb_hub_status_t *st);
usbd_status_t usbreq_set_address(struct usbd_device *udev, struct mtx *mtx, uint16_t addr);
usbd_status_t usbreq_get_port_status(struct usbd_device *udev, struct mtx *mtx, usb_port_status_t *ps, uint8_t port);
usbd_status_t usbreq_clear_hub_feature(struct usbd_device *udev, struct mtx *mtx, uint16_t sel);
usbd_status_t usbreq_set_hub_feature(struct usbd_device *udev, struct mtx *mtx, uint16_t sel);
usbd_status_t usbreq_clear_port_feature(struct usbd_device *udev, struct mtx *mtx, uint8_t port, uint16_t sel);
usbd_status_t usbreq_set_port_feature(struct usbd_device *udev, struct mtx *mtx, uint8_t port, uint16_t sel);
usbd_status_t usbreq_set_protocol(struct usbd_device *udev, struct mtx *mtx, uint8_t iface_index, uint16_t report);
usbd_status_t usbreq_set_report(struct usbd_device *udev, struct mtx *mtx, void *data, uint16_t len, uint8_t iface_index, uint8_t type, uint8_t id);
usbd_status_t usbreq_get_report(struct usbd_device *udev, struct mtx *mtx, void *data, uint16_t len, uint8_t iface_index, uint8_t type, uint8_t id);
usbd_status_t usbreq_set_idle(struct usbd_device *udev, struct mtx *mtx, uint8_t iface_index, uint8_t duration, uint8_t id);
usbd_status_t usbreq_get_report_descriptor(struct usbd_device *udev, struct mtx *mtx, void *d, uint16_t size, uint8_t iface_index);
usbd_status_t usbreq_set_config(struct usbd_device *udev, struct mtx *mtx, uint8_t conf);
usbd_status_t usbreq_get_config(struct usbd_device *udev, struct mtx *mtx, uint8_t *pconf);

/**/
#define	usbd_get_device_descriptor(udev) (&(udev)->ddesc)
#define	usbd_get_config_descriptor(udev) ((udev)->cdesc)
#define	usbd_get_interface_descriptor(iface) ((iface)->idesc)
#define	usbd_get_interface_altindex(iface) ((iface)->alt_index)
#define	usbd_get_quirks(udev) ((udev)->quirks)
#define	usbd_get_speed(udev) ((udev)->speed)
#define	usbd_do_request(u,m,r,d) \
  usbd_do_request_flags(u,m,r,d,0,NULL,USBD_DEFAULT_TIMEOUT)

/* helper for computing offsets */
#define	POINTER_TO_UNSIGNED(ptr) \
  (((const uint8_t *)(ptr)) - ((const uint8_t *)0))

#define	USBD_ADD_BYTES(ptr,size) \
  ((void *)(POINTER_TO_UNSIGNED(ptr) + (size)))

#define	USBD_MS_TO_TICKS(ms) \
  (((uint32_t)((((uint32_t)(ms)) * ((uint32_t)(hz))) + 1023)) / 1024)

/*------------------------------------------------------------------------*
 *	prototypes from "usb_cdev.c"
 *------------------------------------------------------------------------*/

struct usb_cdev;
struct cdev;
struct mtx;

int32_t	usb_cdev_sleep(struct usb_cdev *sc, int32_t fflags, uint32_t timeout);
void	usb_cdev_wakeup(struct usb_cdev *sc);
void	usb_cdev_unlock(struct usb_cdev *sc, int32_t fflags);
int32_t	usb_cdev_lock(struct usb_cdev *sc, int32_t fflags, int32_t error);
int32_t	usb_cdev_attach(struct usb_cdev *sc, void *priv_sc, struct mtx *priv_mtx, const char **pp_dev, uid_t _uid, gid_t _gid, int32_t _perms, uint32_t rd_size, uint16_t rd_packets, uint32_t wr_size, uint16_t wr_packets);
void	usb_cdev_detach(struct usb_cdev *sc);
uint32_t usb_cdev_put_bytes_max(struct usb_cdev *sc);
void	usb_cdev_put_data(struct usb_cdev *sc, struct usbd_page_cache *pc, uint32_t offset, uint32_t len, uint8_t what);
void	usb_cdev_put_data_linear(struct usb_cdev *sc, void *ptr, uint32_t len, uint8_t what);
void	usb_cdev_put_data_error(struct usb_cdev *sc);
uint8_t	usb_cdev_get_data(struct usb_cdev *sc, struct usbd_page_cache *pc, uint32_t offset, uint32_t len, uint32_t *actlen, uint8_t what);
uint8_t	usb_cdev_get_data_linear(struct usb_cdev *sc, void *ptr, uint32_t len, uint32_t *actlen, uint8_t what);
void	usb_cdev_get_data_error(struct usb_cdev *sc);
uint8_t	usb_cdev_opened(struct usb_cdev *sc);

typedef int32_t (usb_cdev_open_t)(struct usb_cdev *sc, int32_t fflags, int32_t mode, struct thread *td);
typedef int32_t (usb_cdev_ioctl_t)(struct usb_cdev *sc, u_long cmd, caddr_t addr, int32_t fflags, struct thread *td);
typedef void (usb_cdev_cmd_t)(struct usb_cdev *sc);

struct usb_cdev {
	struct usbd_ifqueue sc_rdq_free;
	struct usbd_ifqueue sc_rdq_used;
	struct usbd_ifqueue sc_wrq_free;
	struct usbd_ifqueue sc_wrq_used;
	struct selinfo sc_read_sel;
	struct selinfo sc_write_sel;

	/* various pointers */

	void   *sc_rdq_pointer;
	void   *sc_wrq_pointer;
	struct mtx *sc_mtx_ptr;
	void   *sc_priv_ptr;
	void   *sc_fifo_ptr;
#define	USB_CDEV_COUNT 4
	struct cdev *sc_cdev[USB_CDEV_COUNT];
	struct cdev *sc_last_cdev;
	struct proc *sc_async_rd;	/* process that wants SIGIO */
	struct proc *sc_async_wr;	/* process that wants SIGIO */

	/* multiplexer functions */

	usb_cdev_open_t *sc_open;
	usb_cdev_ioctl_t *sc_ioctl;
	usb_cdev_cmd_t *sc_start_read;
	usb_cdev_cmd_t *sc_stop_read;
	usb_cdev_cmd_t *sc_start_write;
	usb_cdev_cmd_t *sc_stop_write;

	uint32_t sc_cur_context;
	uint32_t sc_flags;

	/* synchronization flags */

#define	USB_CDEV_FLAG_GONE	      0x00000001
#define	USB_CDEV_FLAG_FLUSHING_WRITE  0x00000002

#define	USB_CDEV_FLAG_OPEN_READ	      0x00000004
#define	USB_CDEV_FLAG_OPEN_WRITE      0x00000008

#define	USB_CDEV_FLAG_SLEEP_READ      0x00000010
#define	USB_CDEV_FLAG_SLEEP_WRITE     0x00000020

#define	USB_CDEV_FLAG_SLEEP_IOCTL_RD  0x00000040
#define	USB_CDEV_FLAG_SLEEP_IOCTL_WR  0x00000080

#define	USB_CDEV_FLAG_WAKEUP_READ     0x00000100
#define	USB_CDEV_FLAG_WAKEUP_WRITE    0x00000200

#define	USB_CDEV_FLAG_WAKEUP_IOCTL_RD 0x00000400
#define	USB_CDEV_FLAG_WAKEUP_IOCTL_WR 0x00000800

#define	USB_CDEV_FLAG_SELECT_READ     0x00001000
#define	USB_CDEV_FLAG_SELECT_WRITE    0x00002000

#define	USB_CDEV_FLAG_CLOSING_READ    0x00004000
#define	USB_CDEV_FLAG_CLOSING_WRITE   0x00008000

#define	USB_CDEV_FLAG_ERROR_READ      0x00010000	/* can be set to
							 * indicate error */
#define	USB_CDEV_FLAG_ERROR_WRITE     0x00020000	/* can be set to
							 * indicate error */

	/* other flags */

#define	USB_CDEV_FLAG_READ_ONLY       0x00040000	/* device is read only */
#define	USB_CDEV_FLAG_WRITE_ONLY      0x00080000	/* device is write only */
#define	USB_CDEV_FLAG_WAKEUP_RD_IMMED 0x00100000	/* wakeup read thread
							 * immediately */
#define	USB_CDEV_FLAG_WAKEUP_WR_IMMED 0x00200000	/* wakeup write thread
							 * immediately */

	uint8_t	sc_wakeup_read;		/* dummy */
	uint8_t	sc_wakeup_write;	/* dummy */
	uint8_t	sc_wakeup_flush;	/* dummy */
	uint8_t	sc_wakeup_close_read;	/* dummy */
	uint8_t	sc_wakeup_close_write;	/* dummy */
	uint8_t	sc_wakeup_detach;	/* dummy */
	uint8_t	sc_wakeup_ioctl;	/* dummy */
	uint8_t	sc_wakeup_ioctl_rdwr;	/* dummy */
	uint8_t	sc_first_open;		/* set when first device is being
					 * opened */
};

/* prototypes from "usb_compat_linux.c" */
void	usb_linux_free_usb_device(struct usb_device *dev);

/* USB clone support */
struct usbd_clone {
	struct mtx mtx;
	struct usb_cdev cdev;

	struct usbd_bus *bus;
	struct usbd_clone *next;

	uint8_t	unit;
};

#endif					/* _USB_SUBR_H_ */
