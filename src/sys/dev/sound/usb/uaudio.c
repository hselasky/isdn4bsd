/*	$NetBSD: uaudio.c,v 1.91 2004/11/05 17:46:14 kent Exp $	*/
/*	$FreeBSD: src/sys/dev/sound/usb/uaudio.c,v 1.20 2006/02/05 17:47:26 netchild Exp $ */

/*-
 * Copyright (c) 1999 The NetBSD Foundation, Inc.
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

/*
 * USB audio specs: http://www.usb.org/developers/devclass_docs/audio10.pdf
 *                  http://www.usb.org/developers/devclass_docs/frmts10.pdf
 *                  http://www.usb.org/developers/devclass_docs/termt10.pdf
 */

/*
 * Also merged:
 *  $NetBSD: uaudio.c,v 1.94 2005/01/15 15:19:53 kent Exp $
 *  $NetBSD: uaudio.c,v 1.95 2005/01/16 06:02:19 dsainty Exp $
 *  $NetBSD: uaudio.c,v 1.96 2005/01/16 12:46:00 kent Exp $
 *  $NetBSD: uaudio.c,v 1.97 2005/02/24 08:19:38 martin Exp $
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/reboot.h>	/* for bootverbose */

#include <dev/usb2/usb_port.h>
#include <dev/usb2/usb.h>
#include <dev/usb2/usb_subr.h>
#include <dev/usb2/usb_quirks.h>

#include <dev/sound/pcm/sound.h>
#include <dev/sound/chip.h>
#include "feeder_if.h"

#include <dev/sound/usb/uaudioreg.h>
#include <dev/sound/usb/uaudio.h>

#ifdef USB_DEBUG
#define DPRINTF(n,fmt,...)	\
  do { if (uaudio_debug > (n)) {		\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int uaudio_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, uaudio, CTLFLAG_RW, 0, "USB uaudio");
SYSCTL_INT(_hw_usb_uaudio, OID_AUTO, debug, CTLFLAG_RW,
	   &uaudio_debug, 0, "uaudio debug level");
#else
#define DPRINTF(...)
#endif

static u_int32_t uaudio_default_rate = 96000;
static u_int8_t  uaudio_default_bits = 32;
static u_int8_t  uaudio_default_channels = 2;

#define UAUDIO_NCHANBUFS        2 /* number of outstanding request */
#define UAUDIO_NFRAMES         25 /* ms of sound in each request */
#define UAUDIO_RECURSE_LIMIT   24 /* rounds */
#define UAUDIO_DEFAULT_BUFSZ  (16*1024) /* bytes */



#define MAKE_WORD(h,l) (((h) << 8) | (l))
#define BIT_TEST(bm,bno) (((bm)[(bno) / 8] >> (7 - ((bno) % 8))) & 1)

struct uaudio_mixer_node {
	int32_t		minval;
	int32_t		maxval;
#define MIX_MAX_CHAN 8
	int32_t		wValue[MIX_MAX_CHAN]; /* using nchan */
	u_int32_t	delta;
	u_int32_t	mul;
	u_int32_t	ctl;

	u_int16_t	wData[MIX_MAX_CHAN]; /* using nchan */
	u_int16_t	wIndex;

	u_int8_t	update[(MIX_MAX_CHAN+7)/8];
	u_int8_t	nchan;
	u_int8_t	type;
#define MIX_ON_OFF	1
#define MIX_SIGNED_16	2
#define MIX_UNSIGNED_16	3
#define MIX_SIGNED_8	4
#define MIX_SELECTOR	5
#define MIX_UNKNOWN     6
#define MIX_SIZE(n) ((((n) == MIX_SIGNED_16) || \
		      ((n) == MIX_UNSIGNED_16)) ? 2 : 1)
#define MIX_UNSIGNED(n) ((n) == MIX_UNSIGNED_16)

#define MAX_SELECTOR_INPUT_PIN 256
	u_int8_t	slctrtype[MAX_SELECTOR_INPUT_PIN];
	u_int8_t	class;

	struct uaudio_mixer_node *next;
};

struct uaudio_chan {
	struct pcmchan_caps pcm_cap; /* capabilities */
	struct usbd_memory_wait mem_wait;

	struct snd_dbuf *pcm_buf;
  const struct usbd_config *usb_cfg;
	struct mtx * pcm_mtx; /* lock protecting this structure */
	struct uaudio_softc * priv_sc;
	struct pcm_channel * pcm_ch;
	struct usbd_xfer * xfer[UAUDIO_NCHANBUFS];
  const struct usb_audio_streaming_interface_descriptor *p_asid;
  const struct usb_audio_streaming_type1_descriptor *p_asf1d;
  const struct usb_audio_streaming_endpoint_descriptor *p_sed;
  const usb_endpoint_descriptor_audio_t *p_ed1;
  const usb_endpoint_descriptor_audio_t *p_ed2;
  const struct uaudio_format *p_fmt;

	u_int8_t * buf;		/* pointer to buffer */
	u_int8_t * start;	/* upper layer buffer start */
	u_int8_t * end;		/* upper layer buffer end */
	u_int8_t * cur;		/* current position in upper layer buffer */

	u_int32_t block_size;
	u_int32_t sample_rate;
	u_int32_t format;
	u_int32_t pcm_format[2];

	u_int16_t bytes_per_frame;

	u_int8_t valid;
	u_int8_t iface_index;
	u_int8_t iface_alt_index;
};

struct uaudio_softc {
	struct sbuf         sc_sndstat;
	struct sndcard_func sc_sndcard_func;
	struct uaudio_chan  sc_rec_chan;
	struct uaudio_chan  sc_play_chan;
	struct usbd_memory_wait sc_mixer_mem;

	struct usbd_device * sc_udev;
	struct usbd_xfer * sc_mixer_xfer[1];
	struct mtx * sc_mixer_lock;
	struct uaudio_mixer_node * sc_mixer_root;
	struct uaudio_mixer_node * sc_mixer_curr;

	u_int32_t	sc_buffer_size;
	u_int32_t	sc_mix_info;
	u_int32_t	sc_recsrc_info;

	u_int16_t	sc_audio_rev;
	u_int16_t	sc_mixer_count;

	u_int8_t	sc_sndstat_valid;
	u_int8_t	sc_mixer_iface_index;
	u_int8_t	sc_mixer_iface_no;
	u_int8_t	sc_mixer_chan;
	u_int8_t	sc_pcm_registered : 1;
	u_int8_t	sc_mixer_init : 1;
	u_int8_t	sc_unused : 6;
};

struct uaudio_search_result {
	u_int8_t bit_input[(256+7)/8];
	u_int8_t bit_output[(256+7)/8];
	u_int8_t bit_visited[(256+7)/8];
	u_int8_t recurse_level;
	u_int8_t id_max;
};

struct uaudio_terminal_node {
	union {
	    const usb_descriptor_t *desc;
	    const struct usb_audio_input_terminal *it;
	    const struct usb_audio_output_terminal *ot;
	    const struct usb_audio_mixer_unit_0 *mu;
	    const struct usb_audio_selector_unit *su;
	    const struct usb_audio_feature_unit *fu;
	    const struct usb_audio_processing_unit_0 *pu;
	    const struct usb_audio_extension_unit_0 *eu;
	} u;
	struct uaudio_search_result usr;
	struct uaudio_terminal_node *root;
};

struct uaudio_format {
	u_int16_t wFormat;
	u_int8_t  bPrecision;
	u_int32_t freebsd_fmt;
	const char * description;
};

static const struct uaudio_format uaudio_formats[] = {

    {UA_FMT_PCM8,  8, AFMT_U8,      "8-bit U-LE PCM" },
    {UA_FMT_PCM8, 16, AFMT_U16_LE, "16-bit U-LE PCM" },
    {UA_FMT_PCM8, 24, AFMT_U24_LE, "24-bit U-LE PCM" },
    {UA_FMT_PCM8, 32, AFMT_U32_LE, "32-bit U-LE PCM" },

    {UA_FMT_PCM,   8, AFMT_S8,      "8-bit S-LE PCM" },
    {UA_FMT_PCM,  16, AFMT_S16_LE, "16-bit S-LE PCM" },
    {UA_FMT_PCM,  24, AFMT_S24_LE, "24-bit S-LE PCM" },
    {UA_FMT_PCM,  32, AFMT_S32_LE, "32-bit S-LE PCM" },

    {UA_FMT_ALAW,  8, AFMT_A_LAW,  "8-bit A-Law" },
    {UA_FMT_MULAW, 8, AFMT_MU_LAW, "8-bit mu-Law" },

    {0,0,0,NULL}
};

#define UAC_OUTPUT	0
#define UAC_INPUT	1
#define UAC_EQUAL	2
#define UAC_RECORD	3
#define UAC_NCLASSES	4

#ifdef USB_DEBUG
static const char *uac_names[] = {
    "outputs", "inputs", "equalization", "record"
};
#endif

/* prototypes */

static device_probe_t uaudio_probe;
static device_attach_t uaudio_attach;
static device_detach_t uaudio_detach;

#ifdef USB_DEBUG
static void
uaudio_chan_dump_ep_desc(const usb_endpoint_descriptor_audio_t *ed);
#endif

static void
uaudio_chan_fill_info_sub(struct uaudio_softc *sc, struct usbd_device *udev,
			  u_int32_t rate, u_int16_t fps, u_int8_t channels, 
			  u_int8_t bit_resolution);

static void
uaudio_chan_fill_info(struct uaudio_softc *sc, struct usbd_device *udev);

static void
uaudio_chan_play_callback(struct usbd_xfer *xfer);

static void
uaudio_chan_record_callback(struct usbd_xfer *xfer);

static void
uaudio_mixer_add_ctl_sub(struct uaudio_softc *sc, 
			 struct uaudio_mixer_node *mc);
static void
uaudio_mixer_add_ctl(struct uaudio_softc *sc, struct uaudio_mixer_node *mc);

static void
uaudio_mixer_add_input(struct uaudio_softc *sc, 
		       const struct uaudio_terminal_node *iot, int id);
static void
uaudio_mixer_add_output(struct uaudio_softc *sc, 
			const struct uaudio_terminal_node *iot, int id);
static void
uaudio_mixer_add_mixer(struct uaudio_softc *sc, 
		       const struct uaudio_terminal_node *iot, int id);
static void
uaudio_mixer_add_selector(struct uaudio_softc *sc, 
			  const struct uaudio_terminal_node *iot, int id);
static u_int32_t
uaudio_mixer_feature_get_bmaControls(const struct usb_audio_feature_unit *d, 
				     u_int8_t index);
static void
uaudio_mixer_add_feature(struct uaudio_softc *sc, 
			 const struct uaudio_terminal_node *iot, int id);
static void
uaudio_mixer_add_processing_updown(struct uaudio_softc *sc, 
				   const struct uaudio_terminal_node *iot, 
				   int id);
static void
uaudio_mixer_add_processing(struct uaudio_softc *sc, 
			    const struct uaudio_terminal_node *iot, int id);
static void
uaudio_mixer_add_extension(struct uaudio_softc *sc, 
			   const struct uaudio_terminal_node *iot, int id);
static const void *
uaudio_mixer_verify_desc(const void *arg, u_int32_t len);

#ifdef USB_DEBUG
static void
uaudio_mixer_dump_cluster(u_int8_t id, const struct uaudio_terminal_node *iot);
#endif

static struct usb_audio_cluster
uaudio_mixer_get_cluster(u_int8_t id, const struct uaudio_terminal_node *iot);

#ifdef USB_DEBUG
static const char *
uaudio_mixer_get_terminal_name(u_int16_t terminal_type);
#endif

static u_int16_t
uaudio_mixer_determine_class(const struct uaudio_terminal_node *iot, 
			     struct uaudio_mixer_node *mix);
static const u_int16_t 
uaudio_mixer_feature_name(const struct uaudio_terminal_node *iot, 
			  struct uaudio_mixer_node *mix);

static const struct uaudio_terminal_node *
uaudio_mixer_get_input(const struct uaudio_terminal_node *iot, u_int8_t index);

static const struct uaudio_terminal_node *
uaudio_mixer_get_output(const struct uaudio_terminal_node *iot, 
			u_int8_t index);
static void
uaudio_mixer_find_inputs_sub(struct uaudio_terminal_node *root, 
			     const u_int8_t *p_id, u_int8_t n_id, 
			     struct uaudio_search_result *info);
static void
uaudio_mixer_find_outputs_sub(struct uaudio_terminal_node *root, u_int8_t id, 
			      u_int8_t n_id,
			      struct uaudio_search_result *info);
static void
uaudio_mixer_fill_info(struct uaudio_softc *sc, struct usbd_device *udev, 
		       void *desc);
static u_int16_t
uaudio_mixer_get(struct usbd_device *udev, u_int8_t what,
		 struct uaudio_mixer_node *mc);
static void
uaudio_mixer_write_cfg_callback(struct usbd_xfer *xfer);

static usbd_status
uaudio_set_speed(struct usbd_device *udev, u_int8_t endpt, u_int32_t speed);

static int
uaudio_mixer_signext(u_int8_t type, int val);

static int
uaudio_mixer_bsd2value(struct uaudio_mixer_node *mc, int32_t val);

static void
uaudio_mixer_ctl_set(struct uaudio_softc *sc, struct uaudio_mixer_node *mc,
		     u_int8_t chan, int32_t val);
static void
uaudio_mixer_init(struct uaudio_softc *sc);

static const struct usbd_config uaudio_cfg_record_full_speed[UAUDIO_NCHANBUFS] = {
    [0] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = UAUDIO_NFRAMES,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_record_callback,
    },

    [1] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = UAUDIO_NFRAMES,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_record_callback,
    },
};

static const struct usbd_config uaudio_cfg_record_high_speed[UAUDIO_NCHANBUFS] = {
    [0] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = (UAUDIO_NFRAMES * 8),
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_record_callback,
    },

    [1] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_IN,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = (UAUDIO_NFRAMES * 8),
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_record_callback,
    },
};

static const struct usbd_config uaudio_cfg_play_full_speed[UAUDIO_NCHANBUFS] = {
    [0] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = UAUDIO_NFRAMES,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_play_callback,
    },

    [1] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = UAUDIO_NFRAMES,
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_play_callback,
    },
};

static const struct usbd_config uaudio_cfg_play_high_speed[UAUDIO_NCHANBUFS] = {
    [0] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = (UAUDIO_NFRAMES * 8),
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_play_callback,
    },

    [1] = {
      .type      = UE_ISOCHRONOUS,
      .endpoint  = -1, /* any */
      .direction = UE_DIR_OUT,
      .bufsize   = 0, /* use "wMaxPacketSize * frames" */
      .frames    = (UAUDIO_NFRAMES * 8),
      .flags     = USBD_SHORT_XFER_OK,
      .callback  = &uaudio_chan_play_callback,
    },
};

static const struct usbd_config uaudio_mixer_config[1] = {
    [0] = {
      .type      = UE_CONTROL,
      .endpoint  = 0x00, /* Control pipe */
      .direction = -1,
      .bufsize   = (sizeof(usb_device_request_t) + 4),
      .callback  = &uaudio_mixer_write_cfg_callback,
      .timeout   = 1000, /* 1 second */
    },
};


static devclass_t uaudio_devclass;

static device_method_t uaudio_methods[] = {
    DEVMETHOD(device_probe, uaudio_probe),
    DEVMETHOD(device_attach, uaudio_attach),
    DEVMETHOD(device_detach, uaudio_detach),
    DEVMETHOD(device_suspend, bus_generic_suspend),
    DEVMETHOD(device_resume, bus_generic_resume),
    DEVMETHOD(device_shutdown, bus_generic_shutdown),
    DEVMETHOD(bus_print_child, bus_generic_print_child),
    { 0, 0 }
};

static driver_t uaudio_driver = {
    .name    = "uaudio",
    .methods = uaudio_methods,
    .size    = sizeof(struct uaudio_softc),
};

static int
uaudio_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_interface_descriptor_t *id;

	if (uaa->iface == NULL) {
	    return UMATCH_NONE;
	}

	id = usbd_get_interface_descriptor(uaa->iface);

	/* trigger on the control interface */

	if ((id == NULL) ||
	    (id->bInterfaceClass != UICLASS_AUDIO) ||
	    (id->bInterfaceSubClass != UISUBCLASS_AUDIOCONTROL) ||
	    (usbd_get_quirks(uaa->device)->uq_flags & UQ_BAD_AUDIO)) {
	    return UMATCH_NONE;
	}

	return UMATCH_IFACECLASS_IFACESUBCLASS;
}

static int
uaudio_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uaudio_softc *sc = device_get_softc(dev);
	usb_interface_descriptor_t *id;
	device_t child;

	sc->sc_play_chan.priv_sc = sc;
	sc->sc_rec_chan.priv_sc = sc;
	sc->sc_udev = uaa->device;

	usbd_set_desc(dev, uaa->device);

	id = usbd_get_interface_descriptor(uaa->iface);

	uaudio_chan_fill_info(sc, uaa->device);

	uaudio_mixer_fill_info(sc, uaa->device, id);

	sc->sc_mixer_iface_index = uaa->iface_index;
	sc->sc_mixer_iface_no = id->bInterfaceNumber;

	DPRINTF(0, "audio rev %d.%02x\n", 
		sc->sc_audio_rev >> 8, 
		sc->sc_audio_rev & 0xff);

	DPRINTF(0, "%d mixer controls\n", 
		sc->sc_mixer_count);

	if (sc->sc_play_chan.valid) {
	    device_printf(dev, "Play: %d Hz, %d ch, %s format\n",
			  sc->sc_play_chan.sample_rate,
			  sc->sc_play_chan.p_asf1d->bNrChannels,
			  sc->sc_play_chan.p_fmt->description);
	} else {
	    device_printf(dev, "No playback!\n");
	}

	if (sc->sc_rec_chan.valid) {
	    device_printf(dev, "Record: %d Hz, %d ch, %s format\n",
			  sc->sc_rec_chan.sample_rate,
			  sc->sc_rec_chan.p_asf1d->bNrChannels,
			  sc->sc_rec_chan.p_fmt->description);
	} else {
	    device_printf(dev, "No recording!\n");
	}		      

	device_printf(dev, "WARNING: Unplugging the device while "
		      "it is in use will cause a panic!\n");

	DPRINTF(0, "doing child attach\n");

	/* attach the children */

	sc->sc_sndcard_func.func = SCF_PCM;

	child = device_add_child(dev, "pcm", -1);

	if (child == NULL) {
	    DPRINTF(0, "out of memory\n");
	    goto detach;
	}

	device_set_ivars(child, &(sc->sc_sndcard_func));

	if (bus_generic_attach(dev)) {
	    DPRINTF(0, "child attach failed\n");
	    goto detach;
	}

	return 0; /* success */

 detach:
	uaudio_detach(dev);
	return ENXIO;
}

int
uaudio_attach_sub(device_t dev, kobj_class_t mixer_class, kobj_class_t chan_class)
{
	struct uaudio_softc *sc = device_get_softc(device_get_parent(dev));
	struct snddev_info *d = device_get_softc(dev);
	char status[SND_STATUSLEN];

	sc->sc_buffer_size = pcm_getbuffersize(dev, 4096, UAUDIO_DEFAULT_BUFSZ, 65536);

	if (bootverbose) {
	    device_printf(dev, "using a default buffer "
			  "size of %u bytes\n", sc->sc_buffer_size);
	}

	uaudio_mixer_init(sc);

	if (!(sc->sc_mix_info & SOUND_MIXER_PCM)) {

	    DPRINTF(0, "emulating master volume\n");

	    if (d) {
	        /*
		 * Emulate missing pcm mixer controller
		 * through FEEDER_VOLUME
		 */
	        d->flags |= SD_F_SOFTVOL;
	    }
	}

	if (mixer_init(dev, mixer_class, sc)) {
	    goto detach;
	}

	sc->sc_mixer_init = 1;

	snprintf(status, sizeof(status), "at ? %s", PCM_KLDSTRING(snd_uaudio));

	if (pcm_register(dev, sc, 
			 sc->sc_play_chan.valid ? 1 : 0, 
			 sc->sc_rec_chan.valid ? 1 : 0)) {
	    goto detach;
	}

	sc->sc_pcm_registered = 1;

	if (sc->sc_play_chan.valid) {
	    pcm_addchan(dev, PCMDIR_PLAY, chan_class, sc);
	}

	if (sc->sc_rec_chan.valid) {
	    pcm_addchan(dev, PCMDIR_REC, chan_class, sc);
	}

	pcm_setstatus(dev, status);

	return 0; /* success */

 detach:
	uaudio_detach_sub(dev);
	return ENXIO;
}

int
uaudio_detach_sub(device_t dev)
{
	struct uaudio_softc *sc = device_get_softc(device_get_parent(dev));
	int error = 0;

	if (sc->sc_pcm_registered) {
	    error = pcm_unregister(dev);
	} else {
	    if (sc->sc_mixer_init) {
	        error = mixer_uninit(dev);
	    }
	}

	if (error) {
	    panic("Please don't detach your sound device "
		  "while it is in use. This is not supported yet!\n");
	}
	return 0; /* success */
}

static int
uaudio_detach(device_t dev)
{
	struct uaudio_softc *sc = device_get_softc(dev);

	if (bus_generic_detach(dev)) {
	    DPRINTF(0, "detach failed!\n");
	}

	sbuf_delete(&(sc->sc_sndstat));
	sc->sc_sndstat_valid = 0;

	return 0;
}

/*========================================================================*
 * AS - Audio Stream - routines
 *========================================================================*/

#ifdef USB_DEBUG
static void
uaudio_chan_dump_ep_desc(const usb_endpoint_descriptor_audio_t *ed)
{
	if (ed) {
	    DPRINTF(0, "endpoint=%p bLength=%d bDescriptorType=%d \n"
		    "bEndpointAddress=%d bmAttributes=0x%x \n"
		    "wMaxPacketSize=%d bInterval=%d \n"
		    "bRefresh=%d bSynchAddress=%d\n",
		    ed, ed->bLength, ed->bDescriptorType, 
		    ed->bEndpointAddress, ed->bmAttributes, 
		    UGETW(ed->wMaxPacketSize), ed->bInterval, 
		    ed->bRefresh, ed->bSynchAddress);
	}
	return;
}
#endif

static void
uaudio_chan_fill_info_sub(struct uaudio_softc *sc, struct usbd_device *udev,
			u_int32_t rate, u_int16_t fps, u_int8_t channels, 
			u_int8_t bit_resolution)
{
	usb_descriptor_t *desc = NULL;
	const struct usb_audio_streaming_interface_descriptor *asid = NULL;
	const struct usb_audio_streaming_type1_descriptor *asf1d = NULL;
	const struct usb_audio_streaming_endpoint_descriptor *sed = NULL;
	const usb_endpoint_descriptor_audio_t *ed1 = NULL;
	const usb_endpoint_descriptor_audio_t *ed2 = NULL;
	usb_config_descriptor_t *cd = usbd_get_config_descriptor(udev);
	usb_interface_descriptor_t *id;
	const struct uaudio_format *p_fmt;
	struct uaudio_chan * chan;
	u_int16_t curidx = 0xFFFF;
	u_int16_t lastidx = 0xFFFF;
	u_int16_t alt_index = 0;
	u_int16_t wFormat;
	u_int8_t ep_dir;
	u_int8_t ep_type;
	u_int8_t ep_sync;
	u_int8_t bChannels;
	u_int8_t bBitResolution;
	u_int8_t x;
	u_int8_t audio_if = 0;
	u_int8_t sample_size;

	while ((desc = usbd_desc_foreach(cd, desc))) {

	    if ((desc->bDescriptorType == UDESC_INTERFACE) &&
		(desc->bLength >= sizeof(*id))) {

	        id = (void *)desc;

	        if(id->bInterfaceNumber != lastidx) {
		    lastidx = id->bInterfaceNumber;
		    curidx++;
		    alt_index = 0;

		} else {
		    alt_index++;
		}

		if ((id->bInterfaceClass == UICLASS_AUDIO) &&
		    (id->bInterfaceSubClass == UISUBCLASS_AUDIOSTREAM)) {
		    audio_if = 1;
		} else {
		    audio_if = 0;
		}

		asid = NULL;
		asf1d = NULL;
		ed1 = NULL;
		ed2 = NULL;
		sed = NULL;
	    }

	    if ((desc->bDescriptorType == UDESC_CS_INTERFACE) &&
		(desc->bDescriptorSubtype == AS_GENERAL) &&
		(desc->bLength >= sizeof(*asid))) {
	        if (asid == NULL) {
		    asid = (void *)desc;
		}
	    }

	    if ((desc->bDescriptorType == UDESC_CS_INTERFACE) &&
		(desc->bDescriptorSubtype == FORMAT_TYPE) &&
		(desc->bLength >= sizeof(*asf1d))) {
	        if (asf1d == NULL) {
		    asf1d = (void *)desc;
		    if (asf1d->bFormatType != FORMAT_TYPE_I) {
		        DPRINTF(10, "ignored bFormatType = %d\n",
				asf1d->bFormatType);
		        asf1d = NULL;
			continue;
		    }
		    if (asf1d->bLength < (sizeof(*asf1d) +
					  (asf1d->bSamFreqType == 0) ? 6 :
					  (asf1d->bSamFreqType * 3))) {
		        DPRINTF(10, "'asf1d' descriptor is too short\n");
			asf1d = NULL;
			continue;
		    }
		}
	    }

	    if ((desc->bDescriptorType == UDESC_ENDPOINT) &&
		(desc->bLength >= sizeof(*ed1))) {
	        if (ed1 == NULL) {
		    ed1 = (void *)desc;
		    if (UE_GET_XFERTYPE(ed1->bmAttributes) != UE_ISOCHRONOUS) {
		        ed1 = NULL;
		    }
		} else {
		    if (ed2 == NULL) {
		        ed2 = (void *)desc;
			if (UE_GET_XFERTYPE(ed2->bmAttributes) != UE_ISOCHRONOUS) {
			    ed2 = NULL;
			    continue;
			}
			if (ed2->bSynchAddress != 0) {
			    DPRINTF(10, "invalid endpoint: bSynchAddress != 0\n");
			    ed2 = NULL;
			    continue;
			}
			if (ed2->bEndpointAddress != ed1->bSynchAddress) {
			    DPRINTF(10, "invalid endpoint addresses: "
				    "ep[0]->bSynchAddress=0x%x "
				    "ep[1]->bEndpointAddress=0x%x\n",
				    ed1->bSynchAddress,
				    ed2->bEndpointAddress);
			    ed2 = NULL;
			    continue;
			}
		    }
		}
	    }

	    if ((desc->bDescriptorType == UDESC_CS_ENDPOINT) &&
		(desc->bDescriptorSubtype == AS_GENERAL) &&
		(desc->bLength >= sizeof(*sed))) {
	        if (sed == NULL) {
		    sed = (void *)desc;
		}
	    }

	    if (audio_if && asid && asf1d && ed1 && sed) {

		ep_dir = UE_GET_DIR(ed1->bEndpointAddress);
		ep_type = UE_GET_ISO_TYPE(ed1->bmAttributes);
		ep_sync = 0;

		if ((usbd_get_quirks(udev)->uq_flags & UQ_AU_INP_ASYNC) &&
		    (ep_dir == UE_DIR_IN) && (ep_type == UE_ISO_ADAPT)) {
		    ep_type = UE_ISO_ASYNC;
		}

		if ((ep_dir == UE_DIR_IN) && (ep_type == UE_ISO_ADAPT)) {
		    ep_sync = 1;
		}

		if ((ep_dir != UE_DIR_IN) && (ep_type == UE_ISO_ASYNC)) {
		    ep_sync = 1;
		}

		if (ep_sync && (!ed2)) {
		    continue;
		}

		/* we can't handle endpoints that need a sync pipe yet */

		if (ep_sync) {
		    DPRINTF(0, "skipped sync interface\n");
		    audio_if = 0;
		    continue;
		}

		wFormat = UGETW(asid->wFormatTag);
		bChannels = asf1d->bNrChannels;
		bBitResolution = asf1d->bBitResolution;

		if (asf1d->bSamFreqType == 0) {
		    DPRINTF(15, "Sample rate: %d-%dHz\n", 
			    UA_SAMP_LO(asf1d), UA_SAMP_HI(asf1d));

		    if ((rate >= UA_SAMP_LO(asf1d)) &&
			(rate <= UA_SAMP_HI(asf1d))) {
		        goto found_rate;
		    }

		} else {

		    for (x = 0; x < asf1d->bSamFreqType; x++) {
		        DPRINTF(15, "Sample rate = %dHz\n",
				UA_GETSAMP(asf1d, x));

		        if (rate == UA_GETSAMP(asf1d, x)) {
			    goto found_rate;
			}
		    }
		}

		audio_if = 0;
		continue;

	    found_rate:

		for (p_fmt = uaudio_formats;
		     p_fmt->wFormat;
		     p_fmt++) {
		    if ((p_fmt->wFormat == wFormat) &&
			(p_fmt->bPrecision == bBitResolution)) {
		        goto found_format;
		    }
		}

		audio_if = 0;
		continue;

	    found_format:

		if ((bChannels == channels) &&
		    (bBitResolution == bit_resolution)) {

		    chan = (ep_dir == UE_DIR_IN) ?
		      &(sc->sc_rec_chan) : 
		      &(sc->sc_play_chan);

		    if ((chan->valid == 0) && usbd_get_iface(udev, curidx)) {

		        chan->valid = 1;
#ifdef USB_DEBUG
			uaudio_chan_dump_ep_desc(ed1);
			uaudio_chan_dump_ep_desc(ed2);

			if (sed->bmAttributes & UA_SED_FREQ_CONTROL) {
			    DPRINTF(1, "FREQ_CONTROL\n");
			}
			if (sed->bmAttributes & UA_SED_PITCH_CONTROL) {
			    DPRINTF(1, "PITCH_CONTROL\n");
			}
#endif
		        DPRINTF(0, "Sample rate = %dHz, channels = %d, "
				"bits = %d, format = %s\n", rate, channels, 
				bit_resolution, p_fmt->description);

			chan->sample_rate = rate;
			chan->p_asid = asid;
			chan->p_asf1d = asf1d;
			chan->p_ed1 = ed1;
			chan->p_ed2 = ed2;
			chan->p_fmt = p_fmt;
			chan->p_sed = sed;
			chan->iface_index = curidx;
			chan->iface_alt_index = alt_index;

			chan->usb_cfg =
			  (ep_dir == UE_DIR_IN) ?
			  ((fps == 1000) ? 
			   uaudio_cfg_record_full_speed :
			   uaudio_cfg_record_high_speed) :
			  ((fps == 1000) ? 
			   uaudio_cfg_play_full_speed :
			   uaudio_cfg_play_high_speed);


			sample_size = ((chan->p_asf1d->bNrChannels * 
					chan->p_asf1d->bBitResolution) / 8);

			chan->bytes_per_frame = ((rate / fps) * sample_size);

			if (sc->sc_sndstat_valid) {
			    sbuf_printf(&(sc->sc_sndstat), "\n\t"
					"mode %d.%d:(%s) %dch, %d/%dbit, %s, %dHz",
					curidx, alt_index, 
					(ep_dir == UE_DIR_IN) ? "input" : "output",
					asf1d->bNrChannels, asf1d->bBitResolution,
					asf1d->bSubFrameSize * 8, 
					p_fmt->description, rate);
			}
		    }
		}

		audio_if = 0;
		continue;
	    }
        }
        return;
}

static void
uaudio_chan_fill_info(struct uaudio_softc *sc, struct usbd_device *udev)
{
	u_int32_t rate = uaudio_default_rate;
	u_int32_t z;
	u_int16_t fps = (usbd_get_speed(udev) == USB_SPEED_HIGH) ? 8000 : 1000;
	u_int8_t bits = uaudio_default_bits;
	u_int8_t y;
	u_int8_t channels = uaudio_default_channels;
	u_int8_t x;

	bits -= (bits % 8);
	rate -= (rate % fps);

	if (sbuf_new(&(sc->sc_sndstat), NULL, 4096, SBUF_AUTOEXTEND)) {
	    sc->sc_sndstat_valid = 1;
	}

	/* try to search for a valid config */

	for (x = channels; x; x--) {
	    for (y = bits; y; y -= 8) {
	        for (z = rate; z; z -= fps) {
		    uaudio_chan_fill_info_sub(sc, udev, z, fps, x, y);

		    if (sc->sc_rec_chan.valid &&
			sc->sc_play_chan.valid) {
		        goto done;
		    }
		}
	    }
	}

 done:
	if (sc->sc_sndstat_valid) {
	    sbuf_finish(&(sc->sc_sndstat));
	}
	return;
}

static void
uaudio_chan_play_callback(struct usbd_xfer *xfer)
{
	struct uaudio_chan *ch = xfer->priv_sc;
	u_int16_t * p_len = xfer->frlengths;
	u_int8_t * buf = xfer->buffer;
	u_int32_t n;
	u_int32_t total = (ch->bytes_per_frame * xfer->nframes);

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    return;
	}

 tr_transferred:
	if (xfer->actlen < total) {
	    DPRINTF(0, "short transfer, "
		    "%d of %d bytes\n", xfer->actlen, total);
	}

	chn_intr(ch->pcm_ch);

 tr_setup:
	for (n = 0; n < xfer->nframes; n++) {
	    p_len[n] = ch->bytes_per_frame;
	}

	if (total > xfer->length) {
	    DPRINTF(0, "bytes per transfer, %d, "
		    "exceeds maximum, %d!\n", 
		    total, xfer->length);
	    return;
	}

	if (ch->end == ch->start) {
	    DPRINTF(0, "no buffer!\n");
	    return;
	}

	DPRINTF(5, "transfer %d bytes\n", total);

	while (total > 0) {

	    n = (ch->end - ch->cur);
	    if (n > total) {
	        n = total;
	    }

	    bcopy(ch->cur, buf, n);

	    total -= n;
	    ch->cur += n;
	    buf += n;

	    if (ch->cur >= ch->end) {
	        ch->cur = ch->start;
	    }
	}

	usbd_start_hardware(xfer);
	return;
}

static void
uaudio_chan_record_callback(struct usbd_xfer *xfer)
{
	struct uaudio_chan *ch = xfer->priv_sc;
	u_int8_t * buf1 = xfer->buffer;
	u_int8_t * buf2 = xfer->buffer;
	u_int16_t * p_len = xfer->frlengths;
	u_int32_t n;
	u_int32_t m;
	u_int32_t total = (xfer->nframes * ch->bytes_per_frame);

	USBD_CHECK_STATUS(xfer);

 tr_error:
	if (xfer->error == USBD_CANCELLED) {
	    return;
	}

 tr_transferred:
	if (xfer->actlen < total) {
	    DPRINTF(0, "short transfer, "
		    "%d of %d bytes\n", xfer->actlen, total);
	} else {
	    DPRINTF(5, "transferred %d bytes\n", xfer->actlen);
	}

	for (n = 0; n < xfer->nframes; n++) {

	    while (p_len[n] > 0) {

	        m = (ch->end - ch->cur);

		if (m > p_len[n]) {
		    m = p_len[n];
		}

		bcopy(buf1, ch->cur, m);

		p_len[n] -= m;
		buf1 += m;
		ch->cur += m;

		if (ch->cur >= ch->end) {
		    ch->cur = ch->start;
		}
	    }

	    buf2 += ch->bytes_per_frame;
	    buf1 = buf2;
	}

	chn_intr(ch->pcm_ch);

 tr_setup:
	for (n = 0; n < xfer->nframes; n++) {
	    p_len[n] = ch->bytes_per_frame;
	}

	if (total > xfer->length) {
	    DPRINTF(0, "bytes per transfer, %d, "
		    "exceeds maximum, %d!\n", 
		    total, xfer->length);
	    return;
	}

	if (ch->end == ch->start) {
	    DPRINTF(0, "no buffer!\n");
	    return;
	}

	usbd_start_hardware(xfer);
	return;
}

void *
uaudio_chan_init(struct uaudio_softc *sc, struct snd_dbuf *b, 
		 struct pcm_channel *c, int dir)
{
	struct uaudio_chan *ch = ((dir == PCMDIR_PLAY) ? 
				  &(sc->sc_play_chan) : &(sc->sc_rec_chan));
	u_int8_t endpoint;
	u_int8_t iface_index;
	u_int8_t alt_index;

	ch->buf = malloc(sc->sc_buffer_size, M_DEVBUF, M_WAITOK | M_ZERO);

	if (ch->buf == NULL) {
	    goto error;
	}

	if (sndbuf_setup(b, ch->buf, sc->sc_buffer_size) != 0) {
	    goto error;
	}

	ch->start = ch->buf;
	ch->end = ch->buf + sc->sc_buffer_size;
	ch->cur = ch->buf;
	ch->pcm_ch = c;
	ch->pcm_mtx = c->lock;
	ch->pcm_buf = b;

	if (ch->pcm_mtx == NULL) {
	    DPRINTF(0, "ERROR: PCM channels does not have a mutex!\n");
	    goto error;
	}

	/* setup play/record format */

	ch->pcm_cap.fmtlist = ch->pcm_format;

	ch->pcm_format[0] = 0;
	ch->pcm_format[1] = 0;

	ch->pcm_cap.minspeed = ch->sample_rate;
	ch->pcm_cap.maxspeed = ch->sample_rate;

	ch->pcm_cap.fmtlist[0] = ch->p_fmt->freebsd_fmt;

	if (ch->p_asf1d->bNrChannels == 2) {
	    ch->pcm_cap.fmtlist[0] |= AFMT_STEREO;
	}

	ch->pcm_cap.fmtlist[1] = 0;


	/* set alternate interface corresponding to the mode */

	endpoint = ch->p_ed1->bEndpointAddress;
	iface_index = ch->iface_index;
	alt_index = ch->iface_alt_index;

	DPRINTF(0, "endpoint=0x%02x, speed=%d, iface=%d alt=%d\n",
		endpoint, ch->sample_rate, iface_index, alt_index);

	if (usbreq_set_interface(sc->sc_udev, iface_index, alt_index)) {
	    DPRINTF(0, "setting of alternate index failed!\n");
	    goto error;
	}

	USBD_SET_IFACE_NO_PROBE(sc->sc_udev, iface_index);

	/*
	 * If just one sampling rate is supported,
	 * no need to call "uaudio_set_speed()".
	 * Roland SD-90 freezes by a SAMPLING_FREQ_CONTROL request.
	 */
	if (ch->p_asf1d->bSamFreqType != 1) {
	    if (uaudio_set_speed(sc->sc_udev, endpoint, ch->sample_rate)) {
	        DPRINTF(0, "setting of sample rate failed!\n");
		goto error;
	    }
	}

	if (usbd_transfer_setup(sc->sc_udev, iface_index, ch->xfer,
				ch->usb_cfg, UAUDIO_NCHANBUFS, ch,
				ch->pcm_mtx, &(ch->mem_wait))) {
	    DPRINTF(0, "could not allocate USB transfers!\n");
	    goto error;
	}

	return ch;

 error:
	uaudio_chan_free(ch);
	return NULL;
}

int
uaudio_chan_free(struct uaudio_chan *ch)
{
	if (ch->buf != NULL) {
	    free(ch->buf, M_DEVBUF);
	    ch->buf = NULL;
	}

	usbd_transfer_unsetup(ch->xfer, UAUDIO_NCHANBUFS);

	if (ch->pcm_mtx) {
	    usbd_transfer_drain(&(ch->mem_wait), ch->pcm_mtx);
	}

	ch->valid = 0;

	return 0;
}

u_int32_t
uaudio_chan_set_param_blocksize(struct uaudio_chan *ch, u_int32_t blocksize)
{
	struct uaudio_softc *sc = ch->priv_sc;

	if (blocksize) {
	    RANGE(blocksize, 128, sc->sc_buffer_size / 2);
	    if (sndbuf_resize(ch->pcm_buf, 
			      sc->sc_buffer_size/blocksize, blocksize) != 0) {
	        ch->block_size = blocksize;
	    }
	}
	return ch->block_size;
}

int
uaudio_chan_set_param_speed(struct uaudio_chan *ch, u_int32_t speed)
{
	if (speed != ch->sample_rate) {
	    DPRINTF(0, "rate conversion required\n");
	}
	return ch->sample_rate;
}

int
uaudio_chan_getptr(struct uaudio_chan *ch)
{
	return (ch->cur - ch->start);
}

struct pcmchan_caps *
uaudio_chan_getcaps(struct uaudio_chan *ch)
{
	return (&(ch->pcm_cap));
}

int
uaudio_chan_set_param_format(struct uaudio_chan *ch, u_int32_t format)
{
	ch->format = format;
	return 0;
}

int
uaudio_chan_start(struct uaudio_chan *ch)
{
	ch->cur = ch->start;

#if (UAUDIO_NCHANBUFS != 2)
#error "please update code"
#endif
	if (ch->xfer[0]) {
	    usbd_transfer_start(ch->xfer[0]);
	}

	if (ch->xfer[1]) {
	    usbd_transfer_start(ch->xfer[1]);
	}
	return 0;
}

int
uaudio_chan_stop(struct uaudio_chan *ch)
{
#if (UAUDIO_NCHANBUFS != 2)
#error "please update code"
#endif

	if (ch->xfer[0]) {
	    usbd_transfer_stop(ch->xfer[0]);
	}

	if (ch->xfer[1]) {
	    usbd_transfer_stop(ch->xfer[1]);
	}
	return 0;
}

/*========================================================================*
 * AC - Audio Controller - routines
 *========================================================================*/

static void
uaudio_mixer_add_ctl_sub(struct uaudio_softc *sc, struct uaudio_mixer_node *mc)
{
	struct uaudio_mixer_node *p_mc_new = 
	  malloc(sizeof(*p_mc_new), M_USBDEV, M_WAITOK);

	if (p_mc_new) {
	    bcopy(mc, p_mc_new, sizeof(*p_mc_new));
	    p_mc_new->next = sc->sc_mixer_root;
	    sc->sc_mixer_root = p_mc_new;
	    sc->sc_mixer_count++;
	} else {
	    DPRINTF(0, "out of memory\n");
	}
	return;
}

static void
uaudio_mixer_add_ctl(struct uaudio_softc *sc, struct uaudio_mixer_node *mc)
{
	int32_t res;

	if (mc->class < UAC_NCLASSES) {
	    DPRINTF(0, "adding %s.%d\n",
		    uac_names[mc->class], mc->ctl);
	} else {
	    DPRINTF(0, "adding %d\n", mc->ctl);
	}

	mc->delta = 0;
	if (mc->type == MIX_ON_OFF) {
	    mc->minval = 0;
	    mc->maxval = 1;
	} else if (mc->type == MIX_SELECTOR) {

	} else {

	    /* determine min and max values */

	    mc->minval = uaudio_mixer_get(sc->sc_udev, GET_MIN, mc);

	    mc->minval = uaudio_mixer_signext(mc->type, mc->minval);

	    mc->maxval = uaudio_mixer_get(sc->sc_udev, GET_MAX, mc);

	    mc->maxval = 1 + uaudio_mixer_signext(mc->type, mc->maxval);

	    mc->mul = mc->maxval - mc->minval;
	    if (mc->mul == 0) {
	        mc->mul = 1;
	    }

	    res = uaudio_mixer_get(sc->sc_udev, GET_RES, mc);
	    if (res > 0) {
	        mc->delta = ((res * 255) + (mc->mul/2)) / mc->mul;
	    }
	}

	if (mc->maxval < mc->minval) {
	    mc->maxval = mc->minval;
	}

	uaudio_mixer_add_ctl_sub(sc, mc);

#ifdef USB_DEBUG
	if (uaudio_debug > 2) {
	    u_int8_t i;
	    for (i = 0; i < mc->nchan; i++) {
	        DPRINTF(0, "[mix] wValue=%04x\n", mc->wValue[0]);
	    }
	    DPRINTF(0, "[mix] wIndex=%04x type=%d ctl='%d' "
		    "min=%d max=%d\n",
		    mc->wIndex, mc->type, mc->ctl,
		    mc->minval, mc->maxval);
	}
#endif
	return;
}

static void
uaudio_mixer_add_input(struct uaudio_softc *sc, 
		       const struct uaudio_terminal_node *iot, int id)
{
#ifdef USB_DEBUG
	const struct usb_audio_input_terminal *d = iot[id].u.it;

	DPRINTF(2, "bTerminalId=%d wTerminalType=0x%04x "
		 "bAssocTerminal=%d bNrChannels=%d wChannelConfig=%d "
		 "iChannelNames=%d\n",
		 d->bTerminalId, UGETW(d->wTerminalType), d->bAssocTerminal,
		 d->bNrChannels, UGETW(d->wChannelConfig),
		 d->iChannelNames);
#endif
	return;
}

static void
uaudio_mixer_add_output(struct uaudio_softc *sc, 
			const struct uaudio_terminal_node *iot, int id)
{
#ifdef USB_DEBUG
	const struct usb_audio_output_terminal *d = iot[id].u.ot;

	DPRINTF(2, "bTerminalId=%d wTerminalType=0x%04x "
		 "bAssocTerminal=%d bSourceId=%d iTerminal=%d\n",
		 d->bTerminalId, UGETW(d->wTerminalType), d->bAssocTerminal,
		 d->bSourceId, d->iTerminal);
#endif
	return;
}

static void
uaudio_mixer_add_mixer(struct uaudio_softc *sc, 
		       const struct uaudio_terminal_node *iot, int id)
{
	struct uaudio_mixer_node mix;

	const struct usb_audio_mixer_unit_0 *d0 = iot[id].u.mu;
	const struct usb_audio_mixer_unit_1 *d1;

	u_int32_t bno; /* bit number */
	u_int32_t p; /* bit number accumulator */
	u_int32_t mo; /* matching outputs */
	u_int32_t mc; /* matching channels */
	u_int32_t ichs; /* input channels */
	u_int32_t ochs; /* output channels */
	u_int32_t c;
	u_int32_t chs;  /* channels */
	u_int32_t i;
	u_int32_t o;

	DPRINTF(2, "bUnitId=%d bNrInPins=%d\n",
		d0->bUnitId, d0->bNrInPins);

	/* compute the number of input channels */

	ichs = 0;
	for (i = 0; i < d0->bNrInPins; i++) {
	    ichs += (uaudio_mixer_get_cluster(d0->baSourceId[i], iot)
		     .bNrChannels);
	}

	d1 = (const void *)(d0->baSourceId + d0->bNrInPins);

	/* and the number of output channels */

	ochs = d1->bNrChannels;

	DPRINTF(2, "ichs=%d ochs=%d\n", ichs, ochs);

	bzero(&mix, sizeof(mix));

	mix.wIndex = MAKE_WORD(d0->bUnitId, sc->sc_mixer_iface_no);
	uaudio_mixer_determine_class(&iot[id], &mix);
	mix.type = MIX_SIGNED_16;

	if (uaudio_mixer_verify_desc(d0, ((ichs * ochs)+7)/8) == NULL) {
	    return;
	}

	for (p = i = 0; i < d0->bNrInPins; i++) {
	    chs = uaudio_mixer_get_cluster(d0->baSourceId[i], iot).bNrChannels;
	    mc = 0;
	    for (c = 0; c < chs; c++) {
	        mo = 0;
		for (o = 0; o < ochs; o++) {
		    bno = ((p + c) * ochs) + o;
		      if (BIT_TEST(d1->bmControls, bno)) {
			  mo++;
		      }
		}
		if (mo == 1) {
		    mc++;
		}
	    }
	    if ((mc == chs) && (chs <= MIX_MAX_CHAN)) {

	        /* repeat bit-scan */

	        mc = 0;
		for (c = 0; c < chs; c++) {
		    for (o = 0; o < ochs; o++) {
		        bno = ((p + c) * ochs) + o;
			  if (BIT_TEST(d1->bmControls, bno)) {
			      mix.wValue[mc++] = MAKE_WORD(p+c+1, o+1);
			  }
		    }
		}
		mix.nchan = chs;
		uaudio_mixer_add_ctl(sc, &mix);
	    } else {
	        /* XXX */
	    }
	    p += chs;
	}
	return;
}

static void
uaudio_mixer_add_selector(struct uaudio_softc *sc, 
			  const struct uaudio_terminal_node *iot, int id)
{
	const struct usb_audio_selector_unit *d = iot[id].u.su;
	struct uaudio_mixer_node mix;
	struct uaudio_mixer_node dummy;
	u_int16_t i;

	DPRINTF(2, "bUnitId=%d bNrInPins=%d\n",
		d->bUnitId, d->bNrInPins);

	if (d->bNrInPins == 0) {
	    return;
	}

	bzero(&mix, sizeof(mix));

	mix.wIndex = MAKE_WORD(d->bUnitId, sc->sc_mixer_iface_no);
	mix.wValue[0] = MAKE_WORD(0, 0);
	uaudio_mixer_determine_class(&iot[id], &mix);
	mix.nchan = 1;
	mix.type = MIX_SELECTOR;

	mix.ctl = SOUND_MIXER_NRDEVICES;	/* XXXXX */
	mix.minval = 1;
	mix.maxval = d->bNrInPins;

	if (mix.maxval > MAX_SELECTOR_INPUT_PIN) {
	    mix.maxval = MAX_SELECTOR_INPUT_PIN;
	}

	mix.mul = (mix.maxval - mix.minval);
	for (i = 0; i < MAX_SELECTOR_INPUT_PIN; i++) {
	    mix.slctrtype[i] = SOUND_MIXER_NRDEVICES;
	}

	for (i = 0; i < mix.maxval; i++) {
		mix.slctrtype[i] = uaudio_mixer_feature_name
		  (&iot[d->baSourceId[i]], &dummy);
	}

	uaudio_mixer_add_ctl(sc, &mix);
	return;
}

static u_int32_t
uaudio_mixer_feature_get_bmaControls(const struct usb_audio_feature_unit *d, 
				     u_int8_t index)
{
	u_int32_t temp = 0;
	u_int32_t offset = (index * d->bControlSize);

	if (d->bControlSize > 0) {
	    temp |= d->bmaControls[offset];
	    if (d->bControlSize > 1) {
	        temp |= d->bmaControls[offset+1] << 8;
		if (d->bControlSize > 2) {
		    temp |= d->bmaControls[offset+2] << 16;
		    if (d->bControlSize > 3) {
		        temp |= d->bmaControls[offset+3] << 24;
		    }
		}
	    }
	}
	return temp;
}

static void
uaudio_mixer_add_feature(struct uaudio_softc *sc, 
			 const struct uaudio_terminal_node *iot, int id)
{
	const struct usb_audio_feature_unit *d = iot[id].u.fu;
	struct uaudio_mixer_node mix;
	u_int32_t fumask;
	u_int32_t mmask;
	u_int32_t cmask;
	u_int16_t mixernumber;
	u_int8_t  nchan;
	u_int8_t  chan;
	u_int8_t  ctl;
	u_int8_t  i;

	if (d->bControlSize == 0) {
	    return;
	}

	bzero(&mix, sizeof(mix));

	nchan = (d->bLength - 7) / d->bControlSize;
	mmask = uaudio_mixer_feature_get_bmaControls(d,0);
	cmask = 0;

	if (nchan == 0) {
	    return;
	}

	/* figure out what we can control */

	for (chan = 1; chan < nchan; chan++) {
	    DPRINTF(9, "chan=%d mask=%x\n",
		     chan, uaudio_mixer_feature_get_bmaControls(d,chan));

	    cmask |= uaudio_mixer_feature_get_bmaControls(d,chan);
	}

	if (nchan > MIX_MAX_CHAN) {
	    nchan = MIX_MAX_CHAN;
	}

	mix.wIndex = MAKE_WORD(d->bUnitId, sc->sc_mixer_iface_no);

	for (ctl = 1; ctl <= LOUDNESS_CONTROL; ctl++) {

	    fumask = FU_MASK(ctl);

	    DPRINTF(4, "ctl=%d fumask=0x%04x\n",
		     ctl, fumask);

	    if (mmask & fumask) {
	        mix.nchan = 1;
		mix.wValue[0] = MAKE_WORD(ctl, 0);
	    } else if (cmask & fumask) {
	        mix.nchan = nchan - 1;
		for (i = 1; i < nchan; i++) {
		    if (uaudio_mixer_feature_get_bmaControls(d,i) & fumask)
		        mix.wValue[i-1] = MAKE_WORD(ctl, i);
		    else
		        mix.wValue[i-1] = -1;
		}
	    } else {
	        continue;
	    }

	    mixernumber = uaudio_mixer_feature_name(&iot[id], &mix);

	    switch (ctl) {
	    case MUTE_CONTROL:
	        mix.type = MIX_ON_OFF;
		mix.ctl = SOUND_MIXER_NRDEVICES;
		break;

	    case VOLUME_CONTROL:
	        mix.type = MIX_SIGNED_16;
		mix.ctl = mixernumber;
		break;

	    case BASS_CONTROL:
	        mix.type = MIX_SIGNED_8;
		mix.ctl = SOUND_MIXER_BASS;
		break;

	    case MID_CONTROL:
	        mix.type = MIX_SIGNED_8;
		mix.ctl = SOUND_MIXER_NRDEVICES;	/* XXXXX */
		break;

	    case TREBLE_CONTROL:
	        mix.type = MIX_SIGNED_8;
		mix.ctl = SOUND_MIXER_TREBLE;
		break;

	    case GRAPHIC_EQUALIZER_CONTROL:
	        continue; /* XXX don't add anything */
		break;

	    case AGC_CONTROL:
	        mix.type = MIX_ON_OFF;
		mix.ctl = SOUND_MIXER_NRDEVICES;	/* XXXXX */
		break;

	    case DELAY_CONTROL:
	        mix.type = MIX_UNSIGNED_16;
		mix.ctl = SOUND_MIXER_NRDEVICES;	/* XXXXX */
		break;

	    case BASS_BOOST_CONTROL:
	        mix.type = MIX_ON_OFF;
		mix.ctl = SOUND_MIXER_NRDEVICES;	/* XXXXX */
		break;

	    case LOUDNESS_CONTROL:
	        mix.type = MIX_ON_OFF;
		mix.ctl = SOUND_MIXER_LOUD; /* Is this correct ? */
		break;

	    default:
	        mix.type = MIX_UNKNOWN;
		break;
	    }

	    if (mix.type != MIX_UNKNOWN) {
	        uaudio_mixer_add_ctl(sc, &mix);
	    }
	}
	return;
}

static void
uaudio_mixer_add_processing_updown(struct uaudio_softc *sc, 
			     const struct uaudio_terminal_node *iot, int id)
{
	const struct usb_audio_processing_unit_0 *d0 = iot[id].u.pu;
	const struct usb_audio_processing_unit_1 *d1 = 
	  (const void *)(d0->baSourceId + d0->bNrInPins);
	const struct usb_audio_processing_unit_updown *ud = 
	  (const void *)(d1->bmControls + d1->bControlSize);
	struct uaudio_mixer_node mix;
	u_int8_t i;

	if (uaudio_mixer_verify_desc(d0, sizeof(*ud)) == NULL) {
	    return;
	}

	if (uaudio_mixer_verify_desc(d0, sizeof(*ud) + (2*ud->bNrModes)) 
	    == NULL) {
	    return;
	}

	DPRINTF(2, "bUnitId=%d bNrModes=%d\n",
		 d0->bUnitId, ud->bNrModes);

	if (!(d1->bmControls[0] & UA_PROC_MASK(UD_MODE_SELECT_CONTROL))) {
	    DPRINTF(0, "no mode select\n");
	    return;
	}

	bzero(&mix, sizeof(mix));

	mix.wIndex = MAKE_WORD(d0->bUnitId, sc->sc_mixer_iface_no);
	mix.nchan = 1;
	mix.wValue[0] = MAKE_WORD(UD_MODE_SELECT_CONTROL, 0);
	uaudio_mixer_determine_class(&iot[id], &mix);
	mix.type = MIX_ON_OFF;	/* XXX */

	for (i = 0; i < ud->bNrModes; i++) {
	    DPRINTF(2, "i=%d bm=0x%x\n", i, UGETW(ud->waModes[i]));
	    /* XXX */
	}

	uaudio_mixer_add_ctl(sc, &mix);
	return;
}

static void
uaudio_mixer_add_processing(struct uaudio_softc *sc, 
			    const struct uaudio_terminal_node *iot, int id)
{
	const struct usb_audio_processing_unit_0 *d0 = iot[id].u.pu;
	const struct usb_audio_processing_unit_1 *d1 = 
	  (const void *)(d0->baSourceId + d0->bNrInPins);
	struct uaudio_mixer_node mix;
	u_int16_t ptype;

	bzero(&mix, sizeof(mix));

	ptype = UGETW(d0->wProcessType);

	DPRINTF(2, "wProcessType=%d bUnitId=%d "
		 "bNrInPins=%d\n", ptype, d0->bUnitId, d0->bNrInPins);

	if (d1->bControlSize == 0) {
	    return;
	}

	if (d1->bmControls[0] & UA_PROC_ENABLE_MASK) {
	    mix.wIndex = MAKE_WORD(d0->bUnitId, sc->sc_mixer_iface_no);
	    mix.nchan = 1;
	    mix.wValue[0] = MAKE_WORD(XX_ENABLE_CONTROL, 0);
	    uaudio_mixer_determine_class(&iot[id], &mix);
	    mix.type = MIX_ON_OFF;
	    uaudio_mixer_add_ctl(sc, &mix);
	}

	switch(ptype) {
	case UPDOWNMIX_PROCESS:
	    uaudio_mixer_add_processing_updown(sc, iot, id);
	    break;

	case DOLBY_PROLOGIC_PROCESS:
	case P3D_STEREO_EXTENDER_PROCESS:
	case REVERBATION_PROCESS:
	case CHORUS_PROCESS:
	case DYN_RANGE_COMP_PROCESS:
	default:
	    DPRINTF(0, "unit %d, type=%d is not implemented\n",
		    d0->bUnitId, ptype);
	    break;
	}
	return;
}

static void
uaudio_mixer_add_extension(struct uaudio_softc *sc, 
			   const struct uaudio_terminal_node *iot, int id)
{
	const struct usb_audio_extension_unit_0 *d0 = iot[id].u.eu;
	const struct usb_audio_extension_unit_1 *d1 = 
	  (const void *)(d0->baSourceId + d0->bNrInPins);
	struct uaudio_mixer_node mix;

	DPRINTF(2, "bUnitId=%d bNrInPins=%d\n",
		 d0->bUnitId, d0->bNrInPins);

	if (usbd_get_quirks(sc->sc_udev)->uq_flags & UQ_AU_NO_XU) {
	    return;
	}

	if (d1->bControlSize == 0) {
	    return;
	}

	if (d1->bmControls[0] & UA_EXT_ENABLE_MASK) {

	    bzero(&mix, sizeof(mix));

	    mix.wIndex = MAKE_WORD(d0->bUnitId, sc->sc_mixer_iface_no);
	    mix.nchan = 1;
	    mix.wValue[0] = MAKE_WORD(UA_EXT_ENABLE, 0);
	    uaudio_mixer_determine_class(&iot[id], &mix);
	    mix.type = MIX_ON_OFF;

	    uaudio_mixer_add_ctl(sc, &mix);
	}
	return;
}

static const void *
uaudio_mixer_verify_desc(const void *arg, u_int32_t len)
{
	const struct usb_audio_mixer_unit_1 *d1;
	const struct usb_audio_extension_unit_1 *e1;
	const struct usb_audio_processing_unit_1 *u1;

	union {
	  const usb_descriptor_t *desc;
	  const struct usb_audio_input_terminal *it;
	  const struct usb_audio_output_terminal *ot;
	  const struct usb_audio_mixer_unit_0 *mu;
	  const struct usb_audio_selector_unit *su;
	  const struct usb_audio_feature_unit *fu;
	  const struct usb_audio_processing_unit_0 *pu;
	  const struct usb_audio_extension_unit_0 *eu;
	} u;

	u.desc = arg;

	if (u.desc == NULL) {
	    goto error;
	}

	if (u.desc->bDescriptorType != UDESC_CS_INTERFACE) {
	    goto error;
	}

	switch (u.desc->bDescriptorSubtype) {
	case UDESCSUB_AC_INPUT:
	    len += sizeof(*u.it);
	    break;

	case UDESCSUB_AC_OUTPUT:
	    len += sizeof(*u.ot);
	    break;

	case UDESCSUB_AC_MIXER:
	    len += sizeof(*u.mu);

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    len += u.mu->bNrInPins;

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    d1 = (const void *)(u.mu->baSourceId + u.mu->bNrInPins);

	    len += sizeof(*d1);
	    break;

	case UDESCSUB_AC_SELECTOR:
	    len += sizeof(*u.su);

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    len += u.su->bNrInPins;
	    break;

	case UDESCSUB_AC_FEATURE:
	    len += (sizeof(*u.fu) + 1);
	    break;

	case UDESCSUB_AC_PROCESSING:
	    len += sizeof(*u.pu);

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    len += u.pu->bNrInPins;

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    u1 = (const void *)(u.pu->baSourceId + u.pu->bNrInPins);

	    len += sizeof(*u1);

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    len += u1->bControlSize;

	    break;

	case UDESCSUB_AC_EXTENSION:
	    len += sizeof(*u.eu);

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    len += u.eu->bNrInPins;

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    e1 = (const void *)(u.eu->baSourceId + u.eu->bNrInPins);

	    len += sizeof(*e1);

	    if (u.desc->bLength < len) {
	        goto error;
	    }

	    len += e1->bControlSize;
	    break;

	default:
	    goto error;
	}

	if (u.desc->bLength < len) {
	    goto error;
	}
	return u.desc;

 error:
	if (u.desc) {
	    DPRINTF(0, "invalid descriptor, type=%d, "
		    "sub_type=%d, len=%d of %d bytes\n",
		    u.desc->bDescriptorType, 
		    u.desc->bDescriptorSubtype, 
		    u.desc->bLength, len);
	}
	return NULL;
}

#ifdef USB_DEBUG
static void
uaudio_mixer_dump_cluster(u_int8_t id, const struct uaudio_terminal_node *iot)
{
	static const char *channel_names[16] = {
		"LEFT", "RIGHT", "CENTER", "LFE",
		"LEFT_SURROUND", "RIGHT_SURROUND", "LEFT_CENTER", "RIGHT_CENTER",
		"SURROUND", "LEFT_SIDE", "RIGHT_SIDE", "TOP",
		"RESERVED12", "RESERVED13", "RESERVED14", "RESERVED15",
	};
	u_int16_t cc;
	u_int8_t i;
	const struct usb_audio_cluster cl = uaudio_mixer_get_cluster(id, iot);

	cc = UGETW(cl.wChannelConfig);

	DPRINTF(0, "cluster: bNrChannels=%u iChannelNames=%u wChannelConfig="
		"0x%04x:\n", cl.iChannelNames, cl.bNrChannels, cc);

	for (i = 0; cc; i++) {
	    if (cc & 1) {
	        DPRINTF(0, " - %s\n", channel_names[i]);
	    }
	    cc >>= 1;
	}
	return;
}
#endif

static struct usb_audio_cluster
uaudio_mixer_get_cluster(u_int8_t id, const struct uaudio_terminal_node *iot)
{
	struct usb_audio_cluster r;
	const usb_descriptor_t *dp;
	u_int8_t i;

	for (i = 0; i < UAUDIO_RECURSE_LIMIT; i++) { /* avoid infinite loops */
		dp = iot[id].u.desc;
		if (dp == NULL) {
		    goto error;
		}
		switch (dp->bDescriptorSubtype) {
		case UDESCSUB_AC_INPUT:
			r.bNrChannels = iot[id].u.it->bNrChannels;
			r.wChannelConfig[0] = iot[id].u.it->wChannelConfig[0];
			r.wChannelConfig[1] = iot[id].u.it->wChannelConfig[1];
			r.iChannelNames = iot[id].u.it->iChannelNames;
			goto done;

		case UDESCSUB_AC_OUTPUT:
			id = iot[id].u.ot->bSourceId;
			break;

		case UDESCSUB_AC_MIXER:
			r = *(const struct usb_audio_cluster *)
				&iot[id].u.mu->baSourceId[iot[id].u.mu->
							  bNrInPins];
			goto done;

		case UDESCSUB_AC_SELECTOR:
			if (iot[id].u.su->bNrInPins > 0) {
			    /* XXX This is not really right */
			    id = iot[id].u.su->baSourceId[0];
			}
			break;

		case UDESCSUB_AC_FEATURE:
			id = iot[id].u.fu->bSourceId;
			break;

		case UDESCSUB_AC_PROCESSING:
			r = *((const struct usb_audio_cluster *)
			      &iot[id].u.pu->baSourceId[iot[id].u.pu->
							bNrInPins]);
			goto done;

		case UDESCSUB_AC_EXTENSION:
			r = *((const struct usb_audio_cluster *)
			      &iot[id].u.eu->baSourceId[iot[id].u.eu->
							bNrInPins]);
			goto done;

		default:
			goto error;
		}
	}
 error:
	DPRINTF(0, "bad data\n");
	bzero(&r, sizeof(r));
 done:
	return r;
}

#ifdef USB_DEBUG

struct uaudio_tt_to_string {
    u_int16_t terminal_type;
    const char *desc;
};

static const struct uaudio_tt_to_string uaudio_tt_to_string[] = {

  /* USB terminal types */
  { UAT_UNDEFINED,		"UAT_UNDEFINED" },
  { UAT_STREAM,			"UAT_STREAM" },
  { UAT_VENDOR,			"UAT_VENDOR" },

  /* input terminal types */
  { UATI_UNDEFINED,		"UATI_UNDEFINED" },
  { UATI_MICROPHONE,		"UATI_MICROPHONE" },
  { UATI_DESKMICROPHONE, 	"UATI_DESKMICROPHONE" },
  { UATI_PERSONALMICROPHONE, 	"UATI_PERSONALMICROPHONE" },
  { UATI_OMNIMICROPHONE, 	"UATI_OMNIMICROPHONE" },
  { UATI_MICROPHONEARRAY, 	"UATI_MICROPHONEARRAY" },
  { UATI_PROCMICROPHONEARR, 	"UATI_PROCMICROPHONEARR" },

  /* output terminal types */
  { UATO_UNDEFINED,		"UATO_UNDEFINED" },
  { UATO_SPEAKER,		"UATO_SPEAKER" },
  { UATO_HEADPHONES,		"UATO_HEADPHONES" },
  { UATO_DISPLAYAUDIO,		"UATO_DISPLAYAUDIO" },
  { UATO_DESKTOPSPEAKER,	"UATO_DESKTOPSPEAKER" },
  { UATO_ROOMSPEAKER,		"UATO_ROOMSPEAKER" },
  { UATO_COMMSPEAKER,		"UATO_COMMSPEAKER" },
  { UATO_SUBWOOFER,		"UATO_SUBWOOFER" },

  /* bidir terminal types */
  { UATB_UNDEFINED,		"UATB_UNDEFINED" },
  { UATB_HANDSET,		"UATB_HANDSET" },
  { UATB_HEADSET,		"UATB_HEADSET" },
  { UATB_SPEAKERPHONE,		"UATB_SPEAKERPHONE" },
  { UATB_SPEAKERPHONEESUP, 	"UATB_SPEAKERPHONEESUP" },
  { UATB_SPEAKERPHONEECANC, 	"UATB_SPEAKERPHONEECANC" },

  /* telephony terminal types */
  { UATT_UNDEFINED,		"UATT_UNDEFINED" },
  { UATT_PHONELINE,		"UATT_PHONELINE" },
  { UATT_TELEPHONE,		"UATT_TELEPHONE" },
  { UATT_DOWNLINEPHONE,		"UATT_DOWNLINEPHONE" },

  /* external terminal types */
  { UATE_UNDEFINED,		"UATE_UNDEFINED" },
  { UATE_ANALOGCONN,		"UATE_ANALOGCONN" },
  { UATE_LINECONN,		"UATE_LINECONN" },
  { UATE_LEGACYCONN,		"UATE_LEGACYCONN" },
  { UATE_DIGITALAUIFC,		"UATE_DIGITALAUIFC" },
  { UATE_SPDIF,			"UATE_SPDIF" },
  { UATE_1394DA,		"UATE_1394DA" },
  { UATE_1394DV,		"UATE_1394DV" },

  /* embedded function terminal types */
  { UATF_UNDEFINED,		"UATF_UNDEFINED" },
  { UATF_CALIBNOISE,		"UATF_CALIBNOISE" },
  { UATF_EQUNOISE,		"UATF_EQUNOISE" },
  { UATF_CDPLAYER,		"UATF_CDPLAYER" },
  { UATF_DAT,			"UATF_DAT" },
  { UATF_DCC,			"UATF_DCC" },
  { UATF_MINIDISK,		"UATF_MINIDISK" },
  { UATF_ANALOGTAPE,		"UATF_ANALOGTAPE" },
  { UATF_PHONOGRAPH,		"UATF_PHONOGRAPH" },
  { UATF_VCRAUDIO,		"UATF_VCRAUDIO" },
  { UATF_VIDEODISCAUDIO,	"UATF_VIDEODISCAUDIO" },
  { UATF_DVDAUDIO,		"UATF_DVDAUDIO" },
  { UATF_TVTUNERAUDIO,		"UATF_TVTUNERAUDIO" },
  { UATF_SATELLITE,		"UATF_SATELLITE" },
  { UATF_CABLETUNER,		"UATF_CABLETUNER" },
  { UATF_DSS,			"UATF_DSS" },
  { UATF_RADIORECV,		"UATF_RADIORECV" },
  { UATF_RADIOXMIT,		"UATF_RADIOXMIT" },
  { UATF_MULTITRACK,		"UATF_MULTITRACK" },
  { UATF_SYNTHESIZER,		"UATF_SYNTHESIZER" },

  /* unknown */
  { 0x0000,			"UNKNOWN" },
};

static const char *
uaudio_mixer_get_terminal_name(u_int16_t terminal_type)
{
	const struct uaudio_tt_to_string *uat = uaudio_tt_to_string;

	while (uat->terminal_type) {
	    if (uat->terminal_type == terminal_type) {
	        break;
	    }
	    uat++;
	}
	if (uat->terminal_type == 0) {
	    DPRINTF(0, "unknown terminal type (0x%04x)", terminal_type);
	}
	return uat->desc;
}
#endif

static u_int16_t
uaudio_mixer_determine_class(const struct uaudio_terminal_node *iot, 
			     struct uaudio_mixer_node *mix)
{
	u_int16_t terminal_type = 0x0000;
	const struct uaudio_terminal_node *input[2];
	const struct uaudio_terminal_node *output[2];

	input[0] = uaudio_mixer_get_input(iot, 0);
	input[1] = uaudio_mixer_get_input(iot, 1);

	output[0] = uaudio_mixer_get_output(iot, 0);
	output[1] = uaudio_mixer_get_output(iot, 1);

	/*
	 * check if there is only
	 * one output terminal:
	 */
	if (output[0] && (!output[1])) {
	    terminal_type = UGETW(output[0]->u.ot->wTerminalType);
	}

	/*
	 * If the only output terminal is USB,
	 * the class is UAC_RECORD.
	 */
	if ((terminal_type & 0xff00) == (UAT_UNDEFINED & 0xff00)) {

	    mix->class = UAC_RECORD;
	    if (input[0] && (!input[1])) {
	        terminal_type = UGETW(input[0]->u.it->wTerminalType);
	    } else {
	        terminal_type = 0;
	    }
	    goto done;
	}

	/*
	 * if the unit is connected to just 
	 * one input terminal, the 
	 * class is UAC_INPUT:
	 */
	if (input[0] && (!input[1])) {
	    mix->class = UAC_INPUT;
	    terminal_type = UGETW(input[0]->u.it->wTerminalType);
	    goto done;
	}

	/*
	 * Otherwise, the class is UAC_OUTPUT.
	 */
	mix->class = UAC_OUTPUT;
 done:
	return terminal_type;
}

struct uaudio_tt_to_feature {
    u_int16_t terminal_type;
    u_int16_t feature;
};

static const struct uaudio_tt_to_feature uaudio_tt_to_feature[] = {

  { UAT_STREAM, SOUND_MIXER_PCM },

  { UATI_MICROPHONE, SOUND_MIXER_MIC },
  { UATI_DESKMICROPHONE, SOUND_MIXER_MIC },
  { UATI_PERSONALMICROPHONE, SOUND_MIXER_MIC },
  { UATI_OMNIMICROPHONE, SOUND_MIXER_MIC },
  { UATI_MICROPHONEARRAY, SOUND_MIXER_MIC },
  { UATI_PROCMICROPHONEARR, SOUND_MIXER_MIC },

  { UATO_SPEAKER, SOUND_MIXER_SPEAKER },
  { UATO_DESKTOPSPEAKER, SOUND_MIXER_SPEAKER },
  { UATO_ROOMSPEAKER, SOUND_MIXER_SPEAKER },
  { UATO_COMMSPEAKER, SOUND_MIXER_SPEAKER },

  { UATE_ANALOGCONN, SOUND_MIXER_LINE },
  { UATE_LINECONN, SOUND_MIXER_LINE },
  { UATE_LEGACYCONN, SOUND_MIXER_LINE },

  { UATE_DIGITALAUIFC, SOUND_MIXER_ALTPCM },
  { UATE_SPDIF, SOUND_MIXER_ALTPCM },
  { UATE_1394DA, SOUND_MIXER_ALTPCM },
  { UATE_1394DV, SOUND_MIXER_ALTPCM },

  { UATF_CDPLAYER, SOUND_MIXER_CD },

  { UATF_SYNTHESIZER, SOUND_MIXER_SYNTH },

  { UATF_VIDEODISCAUDIO, SOUND_MIXER_VIDEO },
  { UATF_DVDAUDIO, SOUND_MIXER_VIDEO },
  { UATF_TVTUNERAUDIO, SOUND_MIXER_VIDEO },

  /* telephony terminal types */
  { UATT_UNDEFINED, SOUND_MIXER_PHONEIN }, /* SOUND_MIXER_PHONEOUT */
  { UATT_PHONELINE, SOUND_MIXER_PHONEIN }, /* SOUND_MIXER_PHONEOUT */
  { UATT_TELEPHONE, SOUND_MIXER_PHONEIN }, /* SOUND_MIXER_PHONEOUT */
  { UATT_DOWNLINEPHONE, SOUND_MIXER_PHONEIN }, /* SOUND_MIXER_PHONEOUT */

  { UATF_RADIORECV, SOUND_MIXER_RADIO },
  { UATF_RADIOXMIT, SOUND_MIXER_RADIO },

  { UAT_UNDEFINED, SOUND_MIXER_VOLUME },
  { UAT_VENDOR, SOUND_MIXER_VOLUME },
  { UATI_UNDEFINED, SOUND_MIXER_VOLUME },

  /* output terminal types */
  { UATO_UNDEFINED, SOUND_MIXER_VOLUME },
  { UATO_DISPLAYAUDIO, SOUND_MIXER_VOLUME },
  { UATO_SUBWOOFER, SOUND_MIXER_VOLUME },
  { UATO_HEADPHONES, SOUND_MIXER_VOLUME },

  /* bidir terminal types */
  { UATB_UNDEFINED, SOUND_MIXER_VOLUME },
  { UATB_HANDSET, SOUND_MIXER_VOLUME },
  { UATB_HEADSET, SOUND_MIXER_VOLUME },
  { UATB_SPEAKERPHONE, SOUND_MIXER_VOLUME },
  { UATB_SPEAKERPHONEESUP, SOUND_MIXER_VOLUME },
  { UATB_SPEAKERPHONEECANC, SOUND_MIXER_VOLUME },

  /* external terminal types */
  { UATE_UNDEFINED, SOUND_MIXER_VOLUME },

  /* embedded function terminal types */
  { UATF_UNDEFINED, SOUND_MIXER_VOLUME },
  { UATF_CALIBNOISE, SOUND_MIXER_VOLUME },
  { UATF_EQUNOISE, SOUND_MIXER_VOLUME },
  { UATF_DAT, SOUND_MIXER_VOLUME },
  { UATF_DCC, SOUND_MIXER_VOLUME },
  { UATF_MINIDISK, SOUND_MIXER_VOLUME },
  { UATF_ANALOGTAPE, SOUND_MIXER_VOLUME },
  { UATF_PHONOGRAPH, SOUND_MIXER_VOLUME },
  { UATF_VCRAUDIO, SOUND_MIXER_VOLUME },
  { UATF_SATELLITE, SOUND_MIXER_VOLUME },
  { UATF_CABLETUNER, SOUND_MIXER_VOLUME },
  { UATF_DSS, SOUND_MIXER_VOLUME },
  { UATF_MULTITRACK, SOUND_MIXER_VOLUME },
  { 0xffff, SOUND_MIXER_VOLUME },

  /* default */
  { 0x0000, SOUND_MIXER_VOLUME },
};

static const u_int16_t 
uaudio_mixer_feature_name(const struct uaudio_terminal_node *iot, 
			  struct uaudio_mixer_node *mix)
{
	const struct uaudio_tt_to_feature *uat = uaudio_tt_to_feature;
	u_int16_t terminal_type = uaudio_mixer_determine_class(iot, mix);

	if ((mix->class == UAC_RECORD) && (terminal_type == 0)) {
	    return SOUND_MIXER_IMIX;
	}

	while(uat->terminal_type) {
	    if (uat->terminal_type == terminal_type) {
	        break;
	    }
	    uat++;
	}

	DPRINTF(0, "terminal_type=%s (0x%04x) -> %d\n",
		uaudio_mixer_get_terminal_name(terminal_type),
		terminal_type, uat->feature);

	return (uat->feature);
}

const static struct uaudio_terminal_node *
uaudio_mixer_get_input(const struct uaudio_terminal_node *iot, u_int8_t index)
{
    struct uaudio_terminal_node *root = iot->root;
    u_int8_t n;

    n = iot->usr.id_max;
    do {
        if (iot->usr.bit_input[n/8] & (1 << (n % 8))) {
	    if (!index--) {
	        return (root + n);
	    }
	}
    } while(n--);

    return NULL;
}

const static struct uaudio_terminal_node *
uaudio_mixer_get_output(const struct uaudio_terminal_node *iot, u_int8_t index)
{
    struct uaudio_terminal_node *root = iot->root;
    u_int8_t n;

    n = iot->usr.id_max;
    do {
        if (iot->usr.bit_output[n/8] & (1 << (n % 8))) {
	    if (!index--) {
	        return (root + n);
	    }
	}
    } while(n--);

    return NULL;
}

static void
uaudio_mixer_find_inputs_sub(struct uaudio_terminal_node *root, 
			     const u_int8_t *p_id, u_int8_t n_id, 
			     struct uaudio_search_result *info)
{
	struct uaudio_terminal_node *iot;
	u_int8_t n;
	u_int8_t i;

	if (info->recurse_level >= UAUDIO_RECURSE_LIMIT) {
	    return;
	}

	info->recurse_level ++;

	for (n = 0; n < n_id; n++) {

	    i = p_id[n];

	    if (info->bit_visited[i/8] & (1 << (i % 8))) {
	        /* don't go into a circle */
	        DPRINTF(0, "avoided going into a circle at id=%d!\n", i);
	        continue;
	    } else {
	        info->bit_visited[i/8] |= (1 << (i % 8));
	    }

	    iot = (root + i);

	    if (iot->u.desc == NULL) {
	        continue;
	    }

	    switch (iot->u.desc->bDescriptorSubtype) {
	    case UDESCSUB_AC_INPUT:
	        info->bit_input[i/8] |= (1 << (i % 8));
		break;

	    case UDESCSUB_AC_FEATURE:
		uaudio_mixer_find_inputs_sub
		  (root, &(iot->u.fu->bSourceId), 1, info);
		break;

	    case UDESCSUB_AC_OUTPUT:
		uaudio_mixer_find_inputs_sub
		  (root, &(iot->u.ot->bSourceId), 1, info);
		break;

	    case UDESCSUB_AC_MIXER:
		uaudio_mixer_find_inputs_sub
		  (root, iot->u.mu->baSourceId,
		   iot->u.mu->bNrInPins, info); 
		break;

	    case UDESCSUB_AC_SELECTOR:
		uaudio_mixer_find_inputs_sub
		  (root, iot->u.su->baSourceId,
		   iot->u.su->bNrInPins, info);
		break;

	    case UDESCSUB_AC_PROCESSING:
		uaudio_mixer_find_inputs_sub
		  (root, iot->u.pu->baSourceId,
		   iot->u.pu->bNrInPins, info);
		break;

	    case UDESCSUB_AC_EXTENSION:
		uaudio_mixer_find_inputs_sub
		  (root, iot->u.eu->baSourceId,
		   iot->u.eu->bNrInPins, info);
		break;

	    case UDESCSUB_AC_HEADER:
	    default:
		break;
	    }
	}
	info->recurse_level --;
	return;
}

static void
uaudio_mixer_find_outputs_sub(struct uaudio_terminal_node *root, u_int8_t id, 
			      u_int8_t n_id, struct uaudio_search_result *info)
{
	struct uaudio_terminal_node *iot = (root + id);
	u_int8_t j;

	j = n_id;
	do {
	  if ((j != id) && ((root + j)->u.desc) &&
	      ((root + j)->u.desc->bDescriptorSubtype == UDESCSUB_AC_OUTPUT)) {

	        /* 
		 * "j" (output) <--- virtual wire <--- "id" (input)
		 *
		 * if "j" has "id" on the input, 
		 * then "id" have "j" on the output, because
		 * they are connected:
		 */
	        if ((root + j)->usr.bit_input[id/8] & (1 << (id % 8))) {
		    iot->usr.bit_output[j/8] |= (1 << (j % 8));
		}
	    }
	} while(j--);

	return;
}

static void
uaudio_mixer_fill_info(struct uaudio_softc *sc, struct usbd_device *udev, 
		       void *desc)
{
	const struct usb_audio_control_descriptor *acdp;
	usb_config_descriptor_t *cd = usbd_get_config_descriptor(udev);
	const usb_descriptor_t *dp;
	const struct usb_audio_unit *au;
	struct uaudio_terminal_node * iot = NULL;
	u_int16_t wTotalLen;
	u_int8_t ID_max = 0; /* inclusive */
	u_int8_t i;

	desc = usbd_desc_foreach(cd, desc);

	if (desc == NULL) {
	    DPRINTF(0, "no Audio Control header\n");
	    goto done;
	}

	acdp = desc;

     	if ((acdp->bLength < sizeof(*acdp)) ||
	    (acdp->bDescriptorType != UDESC_CS_INTERFACE) ||
	    (acdp->bDescriptorSubtype != UDESCSUB_AC_HEADER)) {
	    DPRINTF(0, "invalid Audio Control header\n");
	    goto done;
	}

	wTotalLen = UGETW(cd->wTotalLength);
	sc->sc_audio_rev = UGETW(acdp->bcdADC);

	DPRINTF(2, "found AC header, vers=%03x, len=%d\n",
		 sc->sc_audio_rev, wTotalLen);

	if (sc->sc_audio_rev != UAUDIO_VERSION) {

	    if (usbd_get_quirks(udev)->uq_flags & UQ_BAD_ADC) {

	    } else {
	        DPRINTF(0, "invalid audio version\n");
		goto done;
	    }
	}

	iot = malloc(sizeof(struct uaudio_terminal_node) * 256, M_TEMP, 
		     M_NOWAIT | M_ZERO);

	if (iot == NULL) {
	    DPRINTF(0, "no memory!\n");
	    goto done;
	}

	while ((desc = usbd_desc_foreach(cd, desc))) {

	    dp = desc;

	    if (dp->bLength > wTotalLen) {
	        break;
	    } else {
	        wTotalLen -= dp->bLength;
	    }

	    au = uaudio_mixer_verify_desc(dp, 0);

	    if (au) {
	        iot[au->bUnitId].u.desc = (const void *)au;
		if (au->bUnitId > ID_max) {
		    ID_max = au->bUnitId;
		}
	    }
	}

	DPRINTF(0, "Maximum ID=%d\n", ID_max);

	/*
	 * determine sourcing inputs for 
	 * all nodes in the tree:
	 */
	i = ID_max;
	do {
	  uaudio_mixer_find_inputs_sub(iot, &i, 1, &((iot+i)->usr));
	} while(i--);

	/*
	 * determine outputs for
	 * all nodes in the tree:
	 */
	i = ID_max;
	do {
	  uaudio_mixer_find_outputs_sub(iot, i, ID_max, &((iot+i)->usr));
	} while(i--);

	/* set "id_max" and "root" */

	i = ID_max;
	do {
	    (iot + i)->usr.id_max = ID_max;
	    (iot + i)->root = iot;
	} while(i--);

#ifdef USB_DEBUG
	i = ID_max;
	do {
	    u_int8_t j;

	    if (iot[i].u.desc == NULL) {
	        continue;
	    }

	    DPRINTF(0, "id %d:\n", i);

	    switch (iot[i].u.desc->bDescriptorSubtype) {
	    case UDESCSUB_AC_INPUT:
	        DPRINTF(0, " - AC_INPUT type=%s\n", 
			uaudio_mixer_get_terminal_name
			(UGETW(iot[i].u.it->wTerminalType)));
		uaudio_mixer_dump_cluster(i, iot);
		break;

	    case UDESCSUB_AC_OUTPUT:
	        DPRINTF(0, " - AC_OUTPUT type=%s "
			"src=%d\n", uaudio_mixer_get_terminal_name
			(UGETW(iot[i].u.ot->wTerminalType)), 
			iot[i].u.ot->bSourceId);
		break;

	    case UDESCSUB_AC_MIXER:
	        DPRINTF(0, " - AC_MIXER src:\n");
		for (j = 0; j < iot[i].u.mu->bNrInPins; j++) {
		    DPRINTF(0, "   - %d\n", iot[i].u.mu->baSourceId[j]);
		}
		uaudio_mixer_dump_cluster(i, iot);
		break;

	    case UDESCSUB_AC_SELECTOR:
	        DPRINTF(0, " - AC_SELECTOR src:\n");
		for (j = 0; j < iot[i].u.su->bNrInPins; j++) {
		    DPRINTF(0, "   - %d\n", iot[i].u.su->baSourceId[j]);
		}
		break;

	    case UDESCSUB_AC_FEATURE:
	        DPRINTF(0, " - AC_FEATURE src=%d\n", iot[i].u.fu->bSourceId);
		break;

	    case UDESCSUB_AC_PROCESSING:
	        DPRINTF(0, " - AC_PROCESSING src:\n");
		for (j = 0; j < iot[i].u.pu->bNrInPins; j++) {
		    DPRINTF(0, "   - %d\n", iot[i].u.pu->baSourceId[j]);
		}
		uaudio_mixer_dump_cluster(i, iot);
		break;

	    case UDESCSUB_AC_EXTENSION:
	        DPRINTF(0, " - AC_EXTENSION src:\n");
		for (j = 0; j < iot[i].u.eu->bNrInPins; j++) {
		    DPRINTF(0, "%d ", iot[i].u.eu->baSourceId[j]);
		}
		uaudio_mixer_dump_cluster(i, iot);
		break;

	    default:
	        DPRINTF(0, "unknown audio control (subtype=%d)\n",
			iot[i].u.desc->bDescriptorSubtype);
	    }

	    DPRINTF(0, "Inputs to this ID are:\n");

	    j = ID_max;
	    do {
	      if (iot[i].usr.bit_input[j/8] & (1 << (j % 8))) {
		  DPRINTF(0, "  -- ID=%d\n", j);
	      }
	    } while(j--);

	    DPRINTF(0, "Outputs from this ID are:\n");

	    j = ID_max;
	    do {
	      if (iot[i].usr.bit_output[j/8] & (1 << (j % 8))) {
		  DPRINTF(0, "  -- ID=%d\n", j);
	      }
	    } while(j--);

	} while(i--);
#endif

	/*
	 * scan the config to create a linked
	 * list of "mixer" nodes:
	 */

	i = ID_max;
	do {
	    dp = iot[i].u.desc;

	    if (dp == NULL) {
	        continue;
	    }

	    DPRINTF(10, "id=%d subtype=%d\n",
		    i, dp->bDescriptorSubtype);

	    switch (dp->bDescriptorSubtype) {
	    case UDESCSUB_AC_HEADER:
	        DPRINTF(0, "unexpected AC header\n");
		break;

	    case UDESCSUB_AC_INPUT:
	        uaudio_mixer_add_input(sc, iot, i);
		break;

	    case UDESCSUB_AC_OUTPUT:
	        uaudio_mixer_add_output(sc, iot, i);
		break;

	    case UDESCSUB_AC_MIXER:
	        uaudio_mixer_add_mixer(sc, iot, i);
		break;

	    case UDESCSUB_AC_SELECTOR:
	        uaudio_mixer_add_selector(sc, iot, i);
		break;

	    case UDESCSUB_AC_FEATURE:
	        uaudio_mixer_add_feature(sc, iot, i);
		break;

	    case UDESCSUB_AC_PROCESSING:
	        uaudio_mixer_add_processing(sc, iot, i);
		break;

	    case UDESCSUB_AC_EXTENSION:
	        uaudio_mixer_add_extension(sc, iot, i);
		break;

	    default:
	        DPRINTF(0, "bad AC desc subtype=0x%02x\n",
			dp->bDescriptorSubtype);
		break;
	    }

	} while(i--);

 done:
	if (iot) {
	    free(iot, M_TEMP);
	}

	return;
}

static u_int16_t
uaudio_mixer_get(struct usbd_device *udev, u_int8_t what,
	      struct uaudio_mixer_node *mc)
{
	usb_device_request_t req;
	u_int16_t val;
	u_int16_t len = MIX_SIZE(mc->type);
	u_int8_t data[4];
	usbd_status err;

	if (mc->wValue[0] == -1) {
	    return 0;
	}

	req.bmRequestType = UT_READ_CLASS_INTERFACE;
	req.bRequest = what;
	USETW(req.wValue, mc->wValue[0]);
	USETW(req.wIndex, mc->wIndex);
	USETW(req.wLength, len);

	err = usbd_do_request(udev, &req, data);
	if (err) {
	    DPRINTF(0, "err=%s\n", usbd_errstr(err));
	    return 0;
	}
	if (len < 1) {
	    data[0] = 0;
	}
	if (len < 2) {
	    data[1] = 0;
	}

	val = (data[0] | (data[1] << 8));

	DPRINTF(2, "val=%d\n", val);

	return val;
}

static void
uaudio_mixer_write_cfg_callback(struct usbd_xfer *xfer)
{
	usb_device_request_t *req = xfer->buffer;
	struct uaudio_softc *sc = xfer->priv_sc;
	struct uaudio_mixer_node *mc = sc->sc_mixer_curr;
	u_int16_t len;
	u_int8_t repeat = 1;
	u_int8_t update;
	u_int8_t chan;

	USBD_CHECK_STATUS(xfer);

 tr_error:
	DPRINTF(0, "error=%s\n", usbd_errstr(xfer->error));

 tr_transferred:
 tr_setup:

	if (mc == NULL) {
	    mc = sc->sc_mixer_root;
	    sc->sc_mixer_curr = mc;
	    sc->sc_mixer_chan = 0;
	    repeat = 0;
	}

	while (mc) {
	    while (sc->sc_mixer_chan < mc->nchan) {

	        len = MIX_SIZE(mc->type);

	        chan = sc->sc_mixer_chan;

		sc->sc_mixer_chan++;

	        update = ((mc->update[chan/8] & (1 << (chan % 8))) &&
			  (mc->wValue[chan] != -1));

		mc->update[chan/8] &= ~(1 << (chan % 8));

		if (update) {

		    req->bmRequestType = UT_WRITE_CLASS_INTERFACE;
		    req->bRequest = SET_CUR;
		    USETW(req->wValue, mc->wValue[chan]);
		    USETW(req->wIndex, mc->wIndex);
		    USETW(req->wLength, len);

		    if (len > 0) {
		        req->bData[0] = (mc->wData[chan] & 0xFF);
		    }
		    if (len > 1) {
		        req->bData[1] = (mc->wData[chan] >> 8) & 0xFF;
		    }

		    xfer->length = (sizeof(*req)+len);
		    usbd_start_hardware(xfer);
		    return;
		}
	    }

	    mc = mc->next;
	    sc->sc_mixer_curr = mc;
	    sc->sc_mixer_chan = 0;
	}

	if (repeat) {
	    goto tr_setup;
	}
	return;
}

static usbd_status
uaudio_set_speed(struct usbd_device *udev, u_int8_t endpt, u_int32_t speed)
{
	usb_device_request_t req;
	u_int8_t data[3];

	DPRINTF(5, "endpt=%d speed=%u\n", endpt, speed);

	req.bmRequestType = UT_WRITE_CLASS_ENDPOINT;
	req.bRequest = SET_CUR;
	USETW2(req.wValue, SAMPLING_FREQ_CONTROL, 0);
	USETW(req.wIndex, endpt);
	USETW(req.wLength, 3);
	data[0] = speed;
	data[1] = speed >> 8;
	data[2] = speed >> 16;

	return usbd_do_request(udev, &req, data);
}

static int
uaudio_mixer_signext(u_int8_t type, int val)
{
	if (!MIX_UNSIGNED(type)) {
	    if (MIX_SIZE(type) == 2) {
	        val = (int16_t)val;
	    } else {
	        val = (int8_t)val;
	    }
	}
	return val;
}

static int
uaudio_mixer_bsd2value(struct uaudio_mixer_node *mc, int32_t val)
{
	DPRINTF(5, "type=%03x val=%d min=%d max=%d ",
		    mc->type, val, mc->minval, mc->maxval);

	if (mc->type == MIX_ON_OFF) {
	    val = (val != 0);
	} else if (mc->type == MIX_SELECTOR) {
	    if ((val < mc->minval) || 
		(val > mc->maxval)) {
	        val = mc->minval;
	    }
	} else {
	    val = (((val + (mc->delta/2)) * mc->mul) / 255) + mc->minval;
	}

	DPRINTF(5, "val=%d\n", val);
	return val;
}

static void
uaudio_mixer_ctl_set(struct uaudio_softc *sc, struct uaudio_mixer_node *mc,
		  u_int8_t chan, int32_t val)
{
	val = uaudio_mixer_bsd2value(mc, val);

	mc->update[chan/8] |= (1 << (chan % 8));
	mc->wData[chan] = val;

	/* start the transfer, if not already started */

	usbd_transfer_start(sc->sc_mixer_xfer[0]);

	return;
}

static void
uaudio_mixer_init(struct uaudio_softc *sc)
{
	struct uaudio_mixer_node *mc;
	int32_t i;

	for (mc = sc->sc_mixer_root; mc;
	     mc = mc->next) {

	    if (mc->ctl != SOUND_MIXER_NRDEVICES) {
	        /* Set device mask bits. 
		 * See /usr/include/machine/soundcard.h 
		 */
	        sc->sc_mix_info |= (1 << mc->ctl);
	    }

	    if ((mc->ctl == SOUND_MIXER_NRDEVICES) && 
		(mc->type == MIX_SELECTOR) && 
		(mc->class == UAC_RECORD)) {

	        for (i = mc->minval; (i > 0) && (i <= mc->maxval); i++) {
		    if (mc->slctrtype[i - 1] == SOUND_MIXER_NRDEVICES) {
		        continue;
		    }
		    sc->sc_recsrc_info |= 1 << mc->slctrtype[i - 1];
		}
	    }
	}
	return;
}

int
uaudio_mixer_init_sub(struct uaudio_softc *sc, struct snd_mixer *m)
{
	DPRINTF(0, "\n");

	sc->sc_mixer_lock = mixer_get_lock(m);

	if (usbd_transfer_setup(sc->sc_udev, sc->sc_mixer_iface_index,
				sc->sc_mixer_xfer, uaudio_mixer_config, 1, sc,
				sc->sc_mixer_lock, &(sc->sc_mixer_mem))) {
	    DPRINTF(0, "could not allocate USB transfer for audio mixer!\n");
	    return ENOMEM;
	}

	mix_setdevs(m,	sc->sc_mix_info);
	mix_setrecdevs(m, sc->sc_recsrc_info);
	return 0;
}

int
uaudio_mixer_uninit_sub(struct uaudio_softc *sc)
{
	DPRINTF(0, "\n");

	usbd_transfer_unsetup(sc->sc_mixer_xfer, 1);

	if (sc->sc_mixer_lock) {
	    usbd_transfer_drain(&(sc->sc_mixer_mem), sc->sc_mixer_lock);
	}
	return 0;
}

void
uaudio_mixer_set(struct uaudio_softc *sc, unsigned type, 
		 unsigned left, unsigned right)
{
	struct uaudio_mixer_node *mc;

	for (mc = sc->sc_mixer_root; mc;
	     mc = mc->next) {

	    if (mc->ctl == type) {
	        if (mc->nchan == 2) {
		    /* set Right */
		    uaudio_mixer_ctl_set(sc, mc, 1, (int)(right*255)/100);
		}
		/* set Left or Mono */
		uaudio_mixer_ctl_set(sc, mc, 0, (int)(left*255)/100);
	    }
	}
	return;
}

u_int32_t
uaudio_mixer_setrecsrc(struct uaudio_softc *sc, u_int32_t src)
{
	struct uaudio_mixer_node *mc;
	int32_t i;

	for (mc = sc->sc_mixer_root; mc;
	     mc = mc->next) {

	    if ((mc->ctl == SOUND_MIXER_NRDEVICES) && 
		(mc->type == MIX_SELECTOR) && 
		(mc->class == UAC_RECORD)) {
	        for (i = mc->minval; (i > 0) && (i <= mc->maxval); i++) {
		    if (src != (1 << mc->slctrtype[i - 1])) {
		        continue;
		    }
		    uaudio_mixer_ctl_set(sc, mc, 0, i);
		    src = (1 << mc->slctrtype[i - 1]);
		}
	    }
	}
	return src;
}

DRIVER_MODULE(uaudio, uhub, uaudio_driver, uaudio_devclass, usbd_driver_load, 0);
MODULE_DEPEND(uaudio, usb, 1, 1, 1);
MODULE_VERSION(uaudio, 1);
