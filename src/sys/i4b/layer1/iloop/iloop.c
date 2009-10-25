/*-
 * Copyright (c) 2009 Hans Petter Selasky. All rights reserved.
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
 *
 *---------------------------------------------------------------------------
 *
 *      ISDN loopback driver for ISDN4BSD
 *      ---------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/uio.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/lock.h>
#include <sys/socket.h>
#include <sys/proc.h>
#include <sys/callout.h>
#include <net/if.h>
#include <sys/fcntl.h>

#include <i4b/include/i4b_compat.h>
#include <i4b/include/i4b_controller.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_lite.h>

#include <i4b/layer1/iloop/iloop.h>

static dss1_lite_set_ring_t iloop_set_ring;

static void iloop_uninit(void *arg);
static void iloop_init(void *arg);

static const struct dss1_lite_methods iloop_dl_methods = {
	DSS1_LITE_DEFAULT_METHODS,
	.set_ring = iloop_set_ring,
	.support_echo_cancel = 1,
};

static void
iloop_set_ring(struct dss1_lite *pdl, uint8_t on)
{
	struct iloop_softc *sc = pdl->dl_softc;

	if (on)
		sc->sc_ringing = 3 * ILOOP_FPS;
	else
		sc->sc_ringing = 0;
}

static void
iloop_timeout(void *arg)
{
	struct iloop_softc *sc = arg;
	struct dss1_lite_fifo *f = &sc->sc_dl.dl_fifo[sc->sc_dl.dl_audio_channel];
	uint16_t i;
	uint16_t timestamp;

	usb_callout_reset(&sc->sc_callout, hz / ILOOP_FPS,
	    &iloop_timeout, sc);

	/* make sure the DSS1 code gets a chance to run */
	dss1_lite_process(&sc->sc_dl);

	sc->sc_timestamp += ILOOP_BPS;

	if (sc->sc_ringing != 0) {
		sc->sc_ringing--;
		if (sc->sc_ringing == 0)
			dss1_lite_hook_off(&sc->sc_dl);
	}
	if (sc->sc_dl.dl_audio_channel == 0)
		return;

	timestamp = sc->sc_timestamp + ILOOP_BPS;

	f->tx_timestamp = sc->sc_timestamp;

	for (i = 0; i != ILOOP_BPS; i++) {
		dss1_lite_l5_put_sample(&sc->sc_dl, f, sc->sc_buffer[i]);
	}

	dss1_lite_l5_put_sample_complete(&sc->sc_dl, f);

	for (i = 0; i != ILOOP_BPS; i++) {
		sc->sc_buffer[i] = dss1_lite_l5_get_sample(&sc->sc_dl, f);
	}

	dss1_lite_l5_get_sample_complete(&sc->sc_dl, f);

	f->rx_timestamp = timestamp;
}

static struct iloop_softc *sc_curr = NULL;

static void
iloop_init(void *arg)
{
	struct iloop_softc *sc;
	struct i4b_controller *ctrl;

	sc = malloc(sizeof(*sc), M_TEMP, M_WAITOK | M_ZERO);
	if (sc == NULL)
		return;

	sc_curr = sc;

	ctrl = i4b_controller_allocate(1, 1, 4, NULL);
	if (ctrl == NULL) {
		printf("iloop0: Could not allocate I4B controller.\n");
		return;
	}
	sc->sc_pmtx = CNTL_GET_LOCK(ctrl);

	sc->sc_dl.dl_softc = sc;

	usb_callout_init_mtx(&sc->sc_callout, sc->sc_pmtx, 0);

	printf("iloop0: I4B Loopback device (attached)\n");

	if (dss1_lite_attach(&sc->sc_dl, NULL,
	    ctrl, &iloop_dl_methods)) {
		printf("iloop0: DSS1 lite attach failed\n");
		goto error;
	}
	mtx_lock(sc->sc_pmtx);
	iloop_timeout(sc);
	mtx_unlock(sc->sc_pmtx);

	return;

error:
	i4b_controller_free(ctrl, 1);

	iloop_uninit(NULL);
}

static void
iloop_uninit(void *arg)
{
	struct iloop_softc *sc = sc_curr;

	if (sc == NULL)
		return;

	printf("iloop0: I4B Loopback device (detached)\n");

	dss1_lite_detach(&sc->sc_dl);

	usb_callout_stop(&sc->sc_callout);

	usb_callout_drain(&sc->sc_callout);

	free(sc, M_TEMP);
}

SYSINIT(iloop_init, SI_SUB_PSEUDO, SI_ORDER_ANY, iloop_init, NULL);
SYSUNINIT(iloop_init, SI_SUB_PSEUDO, SI_ORDER_ANY, iloop_init, NULL);
