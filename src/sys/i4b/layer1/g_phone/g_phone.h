/*-
 * Copyright (c) 2014 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _G_PHONE_H_
#define	_G_PHONE_H_

#define	G_PHONE_MINFRAMES		20	/* units */
#define	G_PHONE_BPF			((8000 * 2) / 1000)	/* bytes/frame */
#define	G_PHONE_BUFSIZE			(G_PHONE_BPF * G_PHONE_MINFRAMES)
#define	G_PHONE_CONFIG_INDEX		0
#define	G_PHONE_IFACE_INDEX		0
#define	G_PHONE_PKT_LEN			16	/* bytes */
#define	G_PHONE_CMD_MAX			128

enum {
	G_PHONE_XFER_INTR,
	G_PHONE_XFER_ISOC_IN_0,
	G_PHONE_XFER_ISOC_IN_1,
	G_PHONE_XFER_ISOC_OUT_0,
	G_PHONE_XFER_ISOC_OUT_1,
	G_PHONE_XFER_MAX,
};

enum {
	G_CMD_SET_CONFIG,
	G_CMD_AUDIO_GET_MIN,
	G_CMD_AUDIO_GET_MAX,
	G_CMD_AUDIO_GET_RES,
	G_CMD_AUDIO_SET_VOLUME,
	G_CMD_AUDIO_SET_RATE,
	G_CMD_MAX
};

struct g_phone_softc {

	struct dss1_lite sc_dl;

	struct callout sc_callout;

	uint8_t	sc_intr_input_pos;
	uint8_t	sc_intr_output_pos;
	uint8_t	sc_intr_data[G_PHONE_CMD_MAX][G_PHONE_PKT_LEN];

	struct mtx *sc_pmtx;

	struct usb_device *sc_udev;
	struct usb_xfer *sc_xfer[G_PHONE_XFER_MAX];

	uint8_t	sc_buffer[G_PHONE_BUFSIZE * 4];

	uint8_t	sc_command_data[G_PHONE_PKT_LEN];
	uint8_t	sc_volume_setting[32];
	uint8_t	sc_volume_limit[32];
	uint8_t	sc_sample_rate[32];

	uint8_t	sc_hook_off;		/* set if hook is off */
	uint8_t	sc_ring_timeout;
};

#endif					/* _G_PHONE_H_ */
