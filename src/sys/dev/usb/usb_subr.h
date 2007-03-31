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

#define	USBD_STATUS_DESC(enum,value) #enum
#define	USBD_STATUS(m)\
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
struct usbd_ifqueue;
struct __callout;
struct module;
struct malloc_type;
struct proc;
struct usb_hid_descriptor;

typedef uint8_t usbd_status;

typedef void (usbd_callback_t)(struct usbd_xfer *);

struct usbd_bus_methods {
	void (*pipe_init)(struct usbd_device *udev, usb_endpoint_descriptor_t *edesc, struct usbd_pipe *pipe);
	void (*do_poll)(struct usbd_bus *);
	usbd_status (*xfer_setup)(struct usbd_device *udev, uint8_t iface_index, struct usbd_xfer **pxfer, const struct usbd_config *setup_start, const struct usbd_config *setup_end);
};

struct usbd_pipe_methods {
	void (*open)(struct usbd_xfer *xfer);
	void (*close)(struct usbd_xfer *xfer);
	void (*enter)(struct usbd_xfer *xfer);
	void (*start)(struct usbd_xfer *xfer);
	void (*copy_in)(struct usbd_xfer *xfer);
	void (*copy_out)(struct usbd_xfer *xfer);
};

struct usbd_port {
	struct usbd_device	*device; /* connected device */
	struct usbd_device	*parent; /* the ports hub */
	uint16_t		power; /* mA of current on port */
	usb_port_status_t	status;
	uint8_t			portno;
	uint8_t			restartcnt;
#define	USBD_RESTART_MAX 5
	uint8_t			last_refcount;
};

struct usbd_hub {
	usbd_status	      (*explore)(struct usbd_device *hub);
	void		       *hubsoftc;
	usb_hub_descriptor_t	hubdesc;
	struct usbd_port        ports[0];
};

#define	USB_PAGE_SIZE PAGE_SIZE

struct usbd_page {
	void			*buffer;
	bus_size_t		physaddr;

#ifdef __FreeBSD__
	bus_dma_tag_t		tag;
	bus_dmamap_t		map;
	register_t		intr_temp;
#endif

#ifdef __NetBSD__
	bus_dma_tag_t		tag;
	bus_dmamap_t		map;
	bus_dma_segment_t	seg;
	register_t		intr_temp;
	int32_t			seg_count;
#endif
	uint32_t		length;
	uint16_t		exit_level;
};

struct usbd_page_search {
	void			*buffer;
	struct usbd_page	*page;
	bus_size_t		physaddr;
	uint32_t		length;
};

struct usbd_page_info {
	struct usbd_page	*page;
	void			*buffer;
	bus_size_t		physaddr;
};

struct usbd_page_cache {
	struct usbd_page	*page_start;
	struct usbd_page	*page_end;
	struct usbd_page	*page_cur;
	uint32_t		page_offset_buf;
	uint32_t		page_offset_cur;
};

struct usbd_bus {
 	struct usb_device_stats	stats;
	struct mtx		mtx;
	struct usbd_port	root_port; /* dummy port for root hub */

	device_t		bdev; /* filled by HC driver */
	bus_dma_tag_t		dma_tag; /* filled by HC driver */
	struct usbd_bus_methods	*methods; /* filled by HC driver */
	struct usbd_device	*devices[USB_MAX_DEVICES];
 	struct proc		*event_thread;

 	uint32_t		no_intrs;

	uint16_t		isoc_time_last; /* ms */
#define USBD_ISOC_TIME_MAX 128 /* ms */

	uint8_t			is_exploring;
	uint8_t			wait_explore;
	uint8_t			needs_explore;/* Set if a hub signalled a change.
					       * This variable is protected by
					       * "usb_global_lock"
					       */
	uint8_t			use_polling;
	uint8_t			usbrev;	/* USB revision */
#define	USBREV_UNKNOWN	0
#define	USBREV_PRE_1_0	1
#define	USBREV_1_0	2
#define	USBREV_1_1	3
#define	USBREV_2_0	4
#define	USBREV_STR { "unknown", "pre 1.0", "1.0", "1.1", "2.0" }
};

struct usbd_interface {
	usb_interface_descriptor_t *idesc;
	uint8_t			alt_index;
};

struct usbd_pipe {
	LIST_HEAD(, usbd_xfer)	list_head;

	usb_endpoint_descriptor_t *edesc;
	struct usbd_pipe_methods  *methods; /* set by HC driver */

	uint16_t		isoc_next;

	uint8_t			toggle_next;
	uint8_t			refcount;
	uint8_t			clearstall;
	uint8_t			iface_index; /* not used by "default pipe" */ 
};

struct usbd_device {
	struct usbd_pipe        default_pipe;  /* pipe 0 */
	struct usbd_interface   ifaces[USB_MAX_ENDPOINTS]; /* array of all interfaces */
	struct usbd_interface   ifaces_end[0];
	struct usbd_pipe        pipes[USB_MAX_ENDPOINTS]; /* array of all pipes */
	struct usbd_pipe        pipes_end[0];

	struct usbd_bus		*bus; /* our controller */
	struct usbd_port	*powersrc; /* upstream hub port, or 0 */
	struct usbd_port	*myhsport; /* closest high speed port */
	struct usbd_device	*myhub; /* upstream hub */
	const struct usbd_quirks *quirks; /* device quirks, always set */
	usb_config_descriptor_t *cdesc; /* full config descr */
	struct usbd_hub		*hub; /* only if this is a hub */
	device_t                subdevs[USB_MAX_ENDPOINTS]; /* array of all sub-devices */
	device_t                subdevs_end[0];

	usb_event_cookie_t	cookie;	       /* unique connection id */

	uint16_t 		power;         /* mA the device uses */
	int16_t			langid;	       /* language for strings */
#define	USBD_NOLANG (-1)

	uint8_t			address; /* device addess */
	uint8_t			config; /* current configuration # */
	uint8_t			depth; /* distance from root hub */
	uint8_t			speed; /* low/full/high speed */
	uint8_t			self_powered; /* flag for self powered */

	usb_endpoint_descriptor_t default_ep_desc; /* for pipe 0 */
	usb_device_descriptor_t ddesc;         /* device descriptor */

	uint8_t                ifaces_no_probe[(USB_MAX_ENDPOINTS + 7) / 8];
#define	USBD_SET_IFACE_NO_PROBE(udev, ii) \
  { (udev)->ifaces_no_probe[(ii) >> 3] |= (1 << ((ii) & 7)); }
#define	USBD_CLR_IFACE_NO_PROBE(udev, ii) \
  { (udev)->ifaces_no_probe[(ii) >> 3] &= ~(1 << ((ii) & 7)); }
#define	USBD_GET_IFACE_NO_PROBE(udev, ii) \
  ((udev)->ifaces_no_probe[(ii) >> 3] & (1 << ((ii) & 7)))

	uint8_t                probed; /* probe state */
#define	USBD_PROBED_NOTHING              0 /* default value */
#define	USBD_PROBED_SPECIFIC_AND_FOUND   1
#define	USBD_PROBED_IFACE_AND_FOUND      2
#define	USBD_PROBED_GENERIC_AND_FOUND    3

	uint8_t			serial[32]; /* serial number */
};

struct usbd_config {
	usbd_callback_t	*callback;

	uint32_t	flags;	/* flags */
#define	USBD_SYNCHRONOUS         0x0001 /* wait for completion */
#define	USBD_FORCE_SHORT_XFER    0x0002 /* force a short packet last */
#if (USBD_SHORT_XFER_OK != 0x0004)
#define	USBD_SHORT_XFER_OK       0x0004 /* allow short reads 
					 * NOTE: existing software
					 * expects USBD_SHORT_XFER_OK
					 * to have a value of 0x4. This
					 * flag is also exported by usb.h
					 */
#endif
#define	USBD_CUSTOM_CLEARSTALL   0x0008 /* used to disable automatic clear-stall
					 * when a device reset request is needed
					 * in addition to the clear stall request
					 */
#define	USBD_DEV_OPEN            0x0010
#define	USBD_DEV_RECURSED_1      0x0020
#define	USBD_DEV_RECURSED_2      0x0040
#define	USBD_DEV_TRANSFERRING    0x0080
#define	USBD_BANDWIDTH_RECLAIMED 0x0100
#define	USBD_USE_POLLING         0x0200 /* used to make synchronous transfers
					 * use polling instead of sleep/wakeup
					 */
#define	USBD_UNUSED_3            0x0400
#define	USBD_USE_DMA             0x0800
#define	USBD_UNUSED_4            0x1000
#define	USBD_UNUSED_5            0x2000
#define	USBD_UNUSED_6            0x4000
#define	USBD_UNUSED_7            0x8000

	uint32_t	bufsize;       	/* total pipe buffer size in bytes */

	uint16_t	timeout;	/* milliseconds */

	uint16_t	frames;		/* number of frames
					 * used in isochronous
					 * mode
					 */
	uint8_t		type;		/* pipe type */
	uint8_t		endpoint;	/* pipe number */

	uint8_t		direction;	/* pipe direction */
	uint8_t		interval;	/* interrupt interval in milliseconds;
					 * used by interrupt pipes
					 */
#define	USBD_DEFAULT_INTERVAL	0

	uint8_t		index;	/* pipe index to use, if more
				 * than one descriptor matches
				 * type, address, direction ...
				 */
};

#define	USBD_TRANSFER_IN_PROGRESS(xfer)		\
	((xfer)->flags & USBD_DEV_TRANSFERRING)

struct usbd_xfer {
	struct __callout 	timeout_handle;
	struct usbd_page_cache 	buf_data; /* buffer page cache */
	struct usbd_page_cache 	buf_fixup; /* fixup buffer */
	LIST_ENTRY(usbd_xfer) 	interrupt_list; /* used by HC driver */
	LIST_ENTRY(usbd_xfer) 	pipe_list; /* used by HC driver */

	struct usbd_pipe 	*pipe;
	struct usbd_device 	*udev;
	struct usbd_xfer 	*clearstall_xfer;
	struct mtx 		*priv_mtx;
	struct mtx 		*usb_mtx; /* used by HC driver */
	struct usbd_memory_info	*usb_root; /* used by HC driver */
	struct thread 		*usb_thread; /* used by HC driver */
	void 			*usb_sc; /* used by HC driver */
	void 			*qh_start; /* used by HC driver */
	void 			*td_start; /* used by HC driver */
	void			*td_transfer_first; /* used by HC driver */
	void			*td_transfer_last; /* used by HC driver */
        void			*td_transfer_cache; /* used by HC driver */
 	void 			*priv_sc;
	void 			*priv_fifo;
	void 			*buffer;
	uint16_t 		*frlengths;
	uint16_t 		*frlengths_old;
	usbd_callback_t 	*callback;

	uint32_t		length; /* bytes */
	uint32_t		actlen; /* bytes */
	uint32_t		flags;
	uint32_t		timeout; /* milliseconds */
#define	USBD_NO_TIMEOUT 0
#define	USBD_DEFAULT_TIMEOUT 5000 /* 5000 ms = 5 seconds */

	uint32_t		nframes; /* for isochronous transfers */
	uint32_t		usb_refcount; /* used by HC driver */

	uint16_t		max_packet_size;
	uint16_t		max_frame_size;
	uint16_t		qh_pos;
	uint16_t		isoc_time_complete; /* in ms */

	uint8_t			address;
	uint8_t			endpoint;
	uint8_t			interval; /* milliseconds */
	uint8_t			max_packet_count;

	usbd_status		error;
};

struct usbd_memory_info {
	void			*memory_base;
	struct mtx		*priv_mtx;
	struct mtx		*usb_mtx;
	struct usbd_page 	*page_base;

	uint32_t		memory_size;
	uint32_t		memory_refcount;
	uint32_t		setup_refcount;
	uint32_t		page_size;
};

struct usbd_callback_info {
	struct usbd_xfer	*xfer;
	uint32_t		refcount;
};

struct usbd_mbuf {
	uint8_t			*cur_data_ptr;
	uint8_t			*min_data_ptr;
	struct usbd_mbuf	*usbd_nextpkt;
	struct usbd_mbuf	*usbd_next;

	uint32_t		cur_data_len;
	uint32_t		max_data_len;
};

struct usbd_ifqueue {
	struct usbd_mbuf	*ifq_head;
	struct usbd_mbuf	*ifq_tail;

	int32_t			ifq_len;
	int32_t			ifq_maxlen;
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
  } while (0)

/*---------------------------------------------------------------------------*
 * structures used by the usbd config thread system
 *---------------------------------------------------------------------------*/
struct usbd_config_td_softc;
struct usbd_config_td_cc;

typedef void (usbd_config_td_command_t)(struct usbd_config_td_softc *sc, struct usbd_config_td_cc *cc, uint16_t reference);
typedef void (usbd_config_td_end_of_commands_t)(struct usbd_config_td_softc *sc);

struct usbd_config_td {
	struct usbd_ifqueue		cmd_free;
	struct usbd_ifqueue		cmd_used;

	struct proc			*config_thread;
	struct mtx			*p_mtx;
	void				*p_softc;
	void				*p_cmd_queue;
	usbd_config_td_end_of_commands_t *p_end_of_commands;

	uint8_t				wakeup_config_td;
	uint8_t				wakeup_config_td_gone;

	uint8_t				flag_config_td_sleep;
	uint8_t				flag_config_td_gone;
};

struct usbd_config_td_item {
	usbd_config_td_command_t	*command_func;
	uint16_t			command_ref;
} __aligned(USB_HOST_ALIGN);

/*---------------------------------------------------------------------------*
 * structures used by probe and attach
 *---------------------------------------------------------------------------*/
struct usb_devno {
	uint16_t			ud_vendor;
	uint16_t			ud_product;
} __packed;

#define	usb_lookup(tbl, vendor, product) usb_match_device			\
	((const struct usb_devno *)(tbl), (sizeof (tbl) / sizeof ((tbl)[0])),	\
	 sizeof ((tbl)[0]), (vendor), (product))				\
/**/

#define	USB_PRODUCT_ANY		0xffff

struct usb_attach_arg {
	struct usbd_device		*device;	/* current device */
	struct usbd_interface 		*iface; /* current interface */
	struct usbd_interface		*ifaces_start; /* all interfaces */
	struct usbd_interface		*ifaces_end; /* exclusive */

	uint16_t			vendor;
	uint16_t			product;
	uint16_t			release;

	uint8_t				port;
	uint8_t				configno;
	uint8_t				iface_index;
	uint8_t				usegeneric;
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

/*---------------------------------------------------------------------------*
 * prototypes
 *---------------------------------------------------------------------------*/

/* prototypes from usb_subr.c */

#define	USBD_SUBTYPE_ANY (-1)

#ifdef __FreeBSD__
#if (__FreeBSD_version >= 700020)
#define	device_get_dma_tag(dev) bus_get_dma_tag(dev)
#else
#define	device_get_dma_tag(dev) NULL /* XXX */
#endif
#endif

void		usbd_devinfo(struct usbd_device *udev, int32_t showclass, char *dst_ptr, uint16_t dst_len);
const char *	usbd_errstr(usbd_status err);
void		usb_delay_ms(struct usbd_bus *bus, uint32_t ms);
void		usbd_delay_ms(struct usbd_device *udev, uint32_t ms);
usb_descriptor_t *usbd_desc_foreach(usb_config_descriptor_t *cd, usb_descriptor_t *desc);
struct usb_hid_descriptor *usbd_get_hdesc(usb_config_descriptor_t *cd, usb_interface_descriptor_t *id);
usb_interface_descriptor_t *usbd_find_idesc(usb_config_descriptor_t *cd, uint16_t iface_index, uint16_t alt_index);
usb_endpoint_descriptor_t *usbd_find_edesc(usb_config_descriptor_t *cd, uint16_t iface_index, uint16_t alt_index, uint16_t endptidx);
usb_descriptor_t *usbd_find_descriptor(usb_config_descriptor_t *cd, int32_t type, int32_t subtype);
int		usbd_get_no_alts(usb_config_descriptor_t *cd, uint8_t ifaceno);
usbd_status	usbd_search_and_set_config(struct usbd_device *udev, int32_t no, int32_t msg);
usbd_status	usbd_set_config_index(struct usbd_device *udev, int32_t index, int32_t msg);
int		usbd_fill_deviceinfo(struct usbd_device *udev, struct usb_device_info *di, int32_t usedev);
usbd_status	usbd_fill_iface_data(struct usbd_device *udev, int32_t iface_index, int32_t alt_index);
usbd_status	usbd_probe_and_attach(device_t parent, int32_t port, struct usbd_port *up);
usbd_status	usbd_new_device(device_t parent, struct usbd_bus *bus, int32_t depth, int32_t speed, int32_t port, struct usbd_port *up);
void		usbd_free_device(struct usbd_port *up, uint8_t free_subdev);
void		usb_detach_wait(device_t dv);
void		usb_detach_wakeup(device_t dv);
struct usbd_interface *usbd_get_iface(struct usbd_device *udev, uint8_t iface_index);
void		usbd_set_desc(device_t dev, struct usbd_device *udev);
void *		usbd_alloc_mbufs(struct malloc_type *type, struct usbd_ifqueue *ifq, uint32_t block_size, uint16_t block_number);
void		usbd_get_page(struct usbd_page_cache *cache, uint32_t offset, struct usbd_page_search *res);
void		usbd_copy_in(struct usbd_page_cache *cache, uint32_t offset, const void *ptr, uint32_t len);
void		usbd_m_copy_in(struct usbd_page_cache *cache, uint32_t dst_offset, struct mbuf *m, uint32_t src_offset, uint32_t src_len);
void		usbd_copy_out(struct usbd_page_cache *cache, uint32_t offset, void *ptr, uint32_t len);
void		usbd_bzero(struct usbd_page_cache *cache, uint32_t offset, uint32_t len);
uint8_t		usbd_page_alloc(bus_dma_tag_t tag, struct usbd_page *page, uint32_t npages);
void		usbd_page_free(struct usbd_page *page, uint32_t npages);
void		usbd_page_get_info(struct usbd_page *page, uint32_t size, struct usbd_page_info *info);
void		usbd_page_set_start(struct usbd_page_cache *pc, struct usbd_page *page_ptr, uint32_t size);
void		usbd_page_set_end(struct usbd_page_cache *pc, struct usbd_page *page_ptr,uint32_t size);
uint32_t	usbd_page_fit_obj(struct usbd_page *page, uint32_t size, uint32_t obj_len);
void *		usbd_mem_alloc(bus_dma_tag_t parent, struct usbd_page *page, uint32_t size, uint8_t align_power);
void		usbd_mem_free(struct usbd_page *page);
bus_dma_tag_t	usbd_dma_tag_alloc(bus_dma_tag_t parent, uint32_t size, uint32_t alignment);
void		usbd_dma_tag_free(bus_dma_tag_t tag);
void *		usbd_mem_alloc_sub(bus_dma_tag_t tag, struct usbd_page *page, uint32_t size, uint32_t alignment);
void		usbd_mem_free_sub(struct usbd_page *page);
void		usbd_page_dma_exit(struct usbd_page *page);
void		usbd_page_dma_enter(struct usbd_page *page);
void		usbd_std_transfer_setup(struct usbd_xfer *xfer, const struct usbd_config *setup, uint16_t max_packet_size, uint16_t max_frame_size, uint8_t max_packet_count);
uint8_t		usbd_make_str_desc(void *ptr, uint16_t max_len, const char *s);
uint32_t	mtx_drop_recurse(struct mtx *mtx);
void		mtx_pickup_recurse(struct mtx *mtx, uint32_t recurse_level);
uint8_t		usbd_config_td_setup(struct usbd_config_td *ctd, void *priv_sc, struct mtx *priv_mtx, usbd_config_td_end_of_commands_t *p_func_eoc, uint16_t item_size, uint16_t item_count);
void		usbd_config_td_stop(struct usbd_config_td *ctd);
void		usbd_config_td_unsetup(struct usbd_config_td *ctd);
void		usbd_config_td_queue_command(struct usbd_config_td *ctd, usbd_config_td_command_t *pre_func, usbd_config_td_command_t *post_func, uint16_t command_qcount, uint16_t command_ref);
uint8_t		usbd_config_td_is_gone(struct usbd_config_td *ctd);
uint8_t		usbd_config_td_sleep(struct usbd_config_td *ctd, uint32_t timeout);
struct mbuf *	usbd_ether_get_mbuf(void);
int32_t		device_delete_all_children(device_t dev);
uint16_t	usbd_get_max_packet_size(usb_endpoint_descriptor_t *edesc);
uint16_t	usbd_get_max_packet_count(usb_endpoint_descriptor_t *edesc);
uint16_t	usbd_get_max_frame_size(usb_endpoint_descriptor_t *edesc);
void		usbd_set_max_packet_size_count(usb_endpoint_descriptor_t *edesc, uint16_t size, uint16_t count);
uint16_t	usbd_isoc_time_expand(struct usbd_bus *bus, uint16_t isoc_time_curr);

/* prototypes from usb.c */

#if 0
extern struct mtx usb_global_lock;
#else
/* XXX currently only the Giant lock can sleep */
#define	usb_global_lock Giant
#endif

extern uint8_t usb_driver_added_refcount;

void		usbd_add_dev_event(int32_t type, struct usbd_device *udev);
void		usbd_add_drv_event(int32_t type, struct usbd_device *udev, device_t dev);
void		usb_needs_explore(struct usbd_device *udev);
void		usb_needs_probe_and_attach(void);

/* prototypes from usb_transfer.c */

#ifdef USB_DEBUG
void		usbd_dump_iface(struct usbd_interface *iface);
void		usbd_dump_device(struct usbd_device *udev);
void		usbd_dump_queue(struct usbd_pipe *pipe);
void		usbd_dump_pipe(struct usbd_pipe *pipe);
void		usbd_dump_xfer(struct usbd_xfer *xfer);
#endif

uint32_t	usb_get_devid(device_t dev);
struct usbd_pipe *usbd_get_pipe(struct usbd_device *udev, uint8_t iface_index, const struct usbd_config *setup);
usbd_status	usbd_interface_count(struct usbd_device *udev, uint8_t *count);
usbd_status	usbd_transfer_setup(struct usbd_device *udev, uint8_t iface_index, struct usbd_xfer **pxfer, const struct usbd_config *setup_start, uint16_t n_setup, void *priv_sc, struct mtx *priv_mtx);
void		usbd_transfer_unsetup(struct usbd_xfer **pxfer, uint16_t n_setup);
void		usbd_std_isoc_copy_in(struct usbd_xfer *xfer);
void		usbd_std_isoc_copy_out(struct usbd_xfer *xfer);
void		usbd_std_bulk_intr_copy_in(struct usbd_xfer *xfer);
void		usbd_std_bulk_intr_copy_out(struct usbd_xfer *xfer);
void		usbd_std_ctrl_copy_in(struct usbd_xfer *xfer);
void		usbd_std_ctrl_copy_out(struct usbd_xfer *xfer);
void		usbd_start_hardware(struct usbd_xfer *xfer);
void		usbd_transfer_start_safe(struct usbd_xfer *xfer);
void		usbd_transfer_start(struct usbd_xfer *xfer);
void		usbd_transfer_stop(struct usbd_xfer *xfer);
void		__usbd_callback(struct usbd_xfer *xfer);
void		usbd_do_callback(struct usbd_callback_info *ptr, struct usbd_callback_info *limit);
void		usbd_transfer_done(struct usbd_xfer *xfer, usbd_status error);
void		usbd_transfer_enqueue(struct usbd_xfer *xfer);
void		usbd_transfer_dequeue(struct usbd_xfer *xfer);
void		usbd_default_callback(struct usbd_xfer *xfer);
usbd_status	usbd_do_request(struct usbd_device *udev, usb_device_request_t *req, void *data);
usbd_status	usbd_do_request_flags(struct usbd_device *udev, usb_device_request_t *req, void *data, uint32_t flags, int32_t *actlen, uint32_t timeout);
usbd_status	usbd_do_request_mtx(struct usbd_device *udev, struct mtx *mtx, usb_device_request_t *req, void *data);
usbd_status	usbd_do_request_flags_mtx(struct usbd_device *udev, struct mtx *mtx, usb_device_request_t *req, void *data, uint32_t flags, int32_t *actlen, uint32_t timeout);
void		usbd_fill_get_report(usb_device_request_t *req, uint8_t iface_no, uint8_t type, uint8_t id, uint16_t size);
void		usbd_fill_set_report(usb_device_request_t *req, uint8_t iface_no, uint8_t type, uint8_t id, uint16_t size);
void		usbd_clear_stall_tr_setup(struct usbd_xfer *xfer1, struct usbd_xfer *xfer2);
void		usbd_clear_stall_tr_transferred(struct usbd_xfer *xfer1, struct usbd_xfer *xfer2);
void		usbd_clearstall_callback(struct usbd_xfer *xfer);
void		usbd_do_poll(struct usbd_device *udev);
void		usbd_set_polling(struct usbd_device *udev, int32_t on);
int32_t		usbd_ratecheck(struct timeval *last);
const struct usb_devno * usb_match_device(const struct usb_devno *tbl, uint32_t nentries, uint32_t size, uint16_t vendor, uint16_t product);
int32_t		usbd_driver_load(struct module *mod, int32_t what, void *arg);

/* prototypes from usb_requests.c */

usbd_status	usbreq_reset_port(struct usbd_device *udev, int32_t port, usb_port_status_t *ps);
usbd_status	usbreq_get_desc(struct usbd_device *udev, int32_t type, int32_t index, int32_t len, void *desc, int32_t timeout);
usbd_status	usbreq_get_string_any(struct usbd_device *udev, int32_t si, char *buf, int32_t len);
usbd_status	usbreq_get_string_desc(struct usbd_device *udev, int32_t sindex, int32_t langid, usb_string_descriptor_t *sdesc, int32_t *plen);
usbd_status	usbreq_get_config_desc(struct usbd_device *udev, int32_t confidx, usb_config_descriptor_t *d);
usbd_status	usbreq_get_config_desc_full(struct usbd_device *udev, int32_t conf, void *d, int32_t size);
usbd_status	usbreq_get_device_desc(struct usbd_device *udev, usb_device_descriptor_t *d);
usbd_status	usbreq_get_interface(struct usbd_device *udev, uint8_t iface_index, uint8_t *aiface);
usbd_status	usbreq_set_interface(struct usbd_device *udev, uint8_t iface_index, uint8_t altno);
usbd_status	usbreq_get_device_status(struct usbd_device *udev, usb_status_t *st);
usbd_status	usbreq_get_hub_descriptor(struct usbd_device *udev, usb_hub_descriptor_t *hd);
usbd_status	usbreq_get_hub_status(struct usbd_device *udev, usb_hub_status_t *st);
usbd_status	usbreq_set_address(struct usbd_device *udev, int32_t addr);
usbd_status	usbreq_get_port_status(struct usbd_device *udev, int32_t port, usb_port_status_t *ps);
usbd_status	usbreq_clear_hub_feature(struct usbd_device *udev, int32_t sel);
usbd_status	usbreq_set_hub_feature(struct usbd_device *udev, int32_t sel);
usbd_status	usbreq_clear_port_feature(struct usbd_device *udev, int32_t port, int32_t sel);
usbd_status	usbreq_set_port_feature(struct usbd_device *udev, int32_t port, int32_t sel);
usbd_status	usbreq_set_protocol(struct usbd_device *udev, uint8_t iface_index, uint16_t report);
usbd_status	usbreq_set_report(struct usbd_device *udev, uint8_t iface_index, uint8_t type, uint8_t id, void *data, int32_t len);
usbd_status	usbreq_get_report(struct usbd_device *udev, uint8_t iface_index, uint8_t type, uint8_t id, void *data, int32_t len);
usbd_status	usbreq_set_idle(struct usbd_device *udev, uint8_t iface_index, int32_t duration, int32_t id);
usbd_status	usbreq_get_report_descriptor(struct usbd_device *udev, int32_t ifcno, int32_t size, void *d);
usbd_status	usbreq_read_report_desc(struct usbd_device *udev, uint8_t iface_index, void **descp, int32_t *sizep, usb_malloc_type mem);
usbd_status	usbreq_set_config(struct usbd_device *udev, int32_t conf);
usbd_status	usbreq_get_config(struct usbd_device *udev, uint8_t *conf);

/**/
#define	usbd_get_device_descriptor(udev) (&(udev)->ddesc)
#define	usbd_get_config_descriptor(udev) ((udev)->cdesc)
#define	usbd_get_interface_descriptor(iface) ((iface)->idesc)
#define	usbd_get_interface_altindex(iface) ((iface)->alt_index)
#define	usbd_get_quirks(udev) ((udev)->quirks)
#define	usbd_get_speed(udev) ((udev)->speed)
#define	usbd_get_hid_descriptor usbd_get_hdesc
#define	usbd_set_config_no usbd_search_and_set_config

/* helper for computing offsets */
#define	POINTER_TO_UNSIGNED(ptr) \
  (((uint8_t *)(ptr)) - ((uint8_t *)0))

/* prototypes from "usb_cdev.c" */

struct usb_cdev;
struct cdev;
struct mtx;

int32_t		usb_cdev_sleep(struct usb_cdev *sc, int32_t fflags, uint32_t timeout);
void		usb_cdev_wakeup(struct usb_cdev *sc);
void		usb_cdev_unlock(struct usb_cdev *sc, int32_t fflags);
int32_t		usb_cdev_lock(struct usb_cdev *sc, int32_t fflags, int32_t error);
int32_t		usb_cdev_attach(struct usb_cdev *sc, void *priv_sc, struct mtx *priv_mtx, const char **pp_dev, uid_t _uid, gid_t _gid, int32_t _perms, uint32_t rd_size, uint16_t rd_packets, uint32_t wr_size, uint16_t wr_packets); 
void		usb_cdev_detach(struct usb_cdev *sc);
void		usb_cdev_put_data(struct usb_cdev *sc, uint8_t *buf, uint32_t len, uint8_t what);
void		usb_cdev_put_data_error(struct usb_cdev *sc);
uint8_t		usb_cdev_get_data(struct usb_cdev *sc, uint8_t *buf, uint32_t len, uint32_t *actlen, uint8_t what);
void		usb_cdev_get_data_error(struct usb_cdev *sc);

typedef int32_t (usb_cdev_open_t)(struct usb_cdev *sc, int32_t fflags, int32_t mode, struct thread *td);
typedef int32_t (usb_cdev_ioctl_t)(struct usb_cdev *sc, u_long cmd, caddr_t addr, int32_t fflags, struct thread *td);
typedef void (usb_cdev_cmd_t)(struct usb_cdev *sc);

struct usb_cdev {
	struct usbd_ifqueue	sc_rdq_free;
	struct usbd_ifqueue	sc_rdq_used;
	struct usbd_ifqueue	sc_wrq_free;
	struct usbd_ifqueue 	sc_wrq_used;
	struct selinfo 		sc_read_sel;
	struct selinfo		sc_write_sel;

	/* various pointers */

	void 			*sc_rdq_pointer;
	void 			*sc_wrq_pointer;
	struct mtx 		*sc_mtx_ptr;
	void 			*sc_priv_ptr;
#define	USB_CDEV_COUNT 4
	struct cdev 		*sc_cdev[USB_CDEV_COUNT];
	struct cdev 		*sc_last_cdev;
	struct proc 		*sc_async_rd; /* process that wants SIGIO */
	struct proc 		*sc_async_wr; /* process that wants SIGIO */

	/* multiplexer functions */

	usb_cdev_open_t 	*sc_open;
	usb_cdev_ioctl_t 	*sc_ioctl;
	usb_cdev_cmd_t		*sc_start_read;
	usb_cdev_cmd_t		*sc_stop_read;
	usb_cdev_cmd_t		*sc_start_write;
	usb_cdev_cmd_t		*sc_stop_write;

	uint32_t		sc_cur_context;
	uint32_t		sc_flags;

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

#define	USB_CDEV_FLAG_ERROR_READ      0x00010000 /* can be set to indicate error */
#define	USB_CDEV_FLAG_ERROR_WRITE     0x00020000 /* can be set to indicate error */

	/* other flags */

#define	USB_CDEV_FLAG_FWD_SHORT       0x00040000 /* can be set to forward short transfers */
#define	USB_CDEV_FLAG_READ_ONLY       0x00080000 /* device is read only */
#define	USB_CDEV_FLAG_WRITE_ONLY      0x00100000 /* device is write only */
#define	USB_CDEV_FLAG_WAKEUP_RD_IMMED 0x00200000 /* wakeup read thread immediately */
#define	USB_CDEV_FLAG_WAKEUP_WR_IMMED 0x00400000 /* wakeup write thread immediately */

	uint8_t			sc_wakeup_read; /* dummy */
	uint8_t			sc_wakeup_write; /* dummy */
	uint8_t			sc_wakeup_flush; /* dummy */
	uint8_t			sc_wakeup_close_read; /* dummy */
	uint8_t			sc_wakeup_close_write; /* dummy */
	uint8_t			sc_wakeup_detach; /* dummy */
	uint8_t			sc_wakeup_ioctl; /* dummy */
	uint8_t			sc_wakeup_ioctl_rdwr; /* dummy */
	uint8_t			sc_first_open; /* set when first device is being opened */
};

#endif /* _USB_SUBR_H_ */
