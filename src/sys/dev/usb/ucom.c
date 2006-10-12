/*	$NetBSD: ucom.c,v 1.40 2001/11/13 06:24:54 lukem Exp $	*/

/*-
 * Copyright (c) 2001-2003, 2005 Shunsuke Akiyama <akiyama@jp.FreeBSD.org>.
 * All rights reserved.
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: src/sys/dev/usb/ucom.c,v 1.60 2006/09/07 00:06:41 imp Exp $");

/*-
 * Copyright (c) 1998, 2000 The NetBSD Foundation, Inc.
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/tty.h>
#include <sys/serial.h>
#include <sys/taskqueue.h>

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

__FBSDID("$FreeBSD: src/sys/dev/usb/ucom.c $");

#ifdef USB_DEBUG
#define DPRINTF(n,fmt,...)						\
  do { if (ucom_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ucom_debug = 0;
SYSCTL_NODE(_hw_usb, OID_AUTO, ucom, CTLFLAG_RW, 0, "USB ucom");
SYSCTL_INT(_hw_usb_ucom, OID_AUTO, debug, CTLFLAG_RW,
	   &ucom_debug, 0, "ucom debug level");
#else
#define DPRINTF(...)
#endif

static uint8_t
ucom_units_alloc(uint32_t sub_units, uint32_t *p_root_unit);

static void
ucom_units_free(uint32_t root_unit, uint32_t sub_units);

static int
ucom_attach_sub(struct ucom_softc *sc);

static void
ucom_detach_sub(struct ucom_softc *sc);

static void
ucom_shutdown(struct ucom_softc *sc);

static int
ucom_open(struct tty *tp, struct cdev *dev);

static void
ucom_close(struct tty *tp);

static int
ucom_ioctl(struct tty *tp, u_long cmd, void *data, 
	   int flag, struct thread *td);

static int
ucom_modem(struct tty *tp, int sigon, int sigoff);

static void
ucom_break(struct tty *tp, int onoff);

static void
ucom_dtr(struct ucom_softc *sc, u_int8_t onoff);

static void
ucom_rts(struct ucom_softc *sc, u_int8_t onoff);

static void
ucom_notify(void *arg, int pending);

static int
ucom_param(struct tty *tp, struct termios *t);

static void
ucom_start_write(struct tty *tp);

static void
ucom_stop_write(struct tty *tp, int fflags);

static moduledata_t ucom_mod = {
	"ucom",
	NULL,
	NULL
};

DECLARE_MODULE(ucom, ucom_mod, SI_SUB_DRIVERS, SI_ORDER_MIDDLE);
MODULE_DEPEND(ucom, usb, 1, 1, 1);
MODULE_VERSION(ucom, UCOM_MODVER);

#define UCOM_UNIT_MAX 0x1000 /* exclusive */

static uint8_t ucom_bitmap[(UCOM_UNIT_MAX+7)/8];

static uint8_t
ucom_units_alloc(uint32_t sub_units, uint32_t *p_root_unit)
{
    uint32_t n;
    uint32_t o;
    uint32_t x;
    uint32_t max = UCOM_UNIT_MAX - (UCOM_UNIT_MAX % sub_units);
    uint8_t error = 1;

    mtx_lock(&Giant);

    for (n = 0; n < max; n += sub_units) {

        /* check for free consecutive bits */

        for (o = 0; o < sub_units; o++) {

	    x = n + o;

	    if (ucom_bitmap[x/8] & (1<< (x % 8))) {
	        goto skip;
	    }
	}

	/* allocate */

        for (o = 0; o < sub_units; o++) {

	    x = n + o;

	    ucom_bitmap[x/8] |= (1<< (x % 8));
	}

	*p_root_unit = n;

	error = 0;

	break;

    skip: ;
    }

    mtx_unlock(&Giant);

    return error;
}

static void
ucom_units_free(uint32_t root_unit, uint32_t sub_units)
{
    uint32_t x;

    mtx_lock(&Giant);

    while(sub_units--) {
        x = root_unit + sub_units;
	ucom_bitmap[x/8] &= ~(1<<(x%8));
    }

    mtx_unlock(&Giant);

    return;
}

/*
 * "N" sub_units are setup at a time. All sub-units will
 * be given sequential unit numbers. The number of 
 * sub-units can be used to differentiate among 
 * different types of devices.
 *
 * The mutex pointed to by "p_mtx" is applied before all
 * callbacks are called back. Also "p_mtx" must be applied
 * before calling into the ucom-layer! Currently only Giant
 * is supported.
 */
int
ucom_attach(struct ucom_softc *sc, uint32_t sub_units, void *parent, 
	    const struct ucom_callback *callback, struct mtx *p_mtx)
{
	uint32_t n;
	uint32_t root_unit;
	int error = 0;

	if ((p_mtx != &Giant) || /* XXX TTY layer requires Giant */
	    (sc == NULL) ||
	    (sub_units == 0) ||
	    (callback == NULL)) {
	    return EINVAL;
	}

	if (ucom_units_alloc(sub_units, &root_unit)) {
	    return ENOMEM;
	}

	for (n = 0; n < sub_units; n++, sc++) {
	    sc->sc_unit = root_unit + n;
	    sc->sc_parent_mtx = p_mtx;
	    sc->sc_parent = parent;
	    sc->sc_callback = callback;

	    mtx_lock(&Giant); /* XXX TTY layer */
	    error = ucom_attach_sub(sc);
	    mtx_unlock(&Giant); /* XXX TTY layer */

	    if (error) {
	        ucom_detach(sc - n, n);
	        ucom_units_free(root_unit + n, sub_units - n);
		break;
	    }
	    sc->sc_flag |= UCOM_FLAG_ATTACHED;
	}
	return error;
}

/* NOTE: the following function will do nothing if 
 * structure pointed to by "sc" is zero.
 */
void
ucom_detach(struct ucom_softc *sc, uint32_t sub_units)
{
	uint32_t n;

	for (n = 0; n < sub_units; n++, sc++) {
	    if (sc->sc_flag & UCOM_FLAG_ATTACHED) {

		mtx_lock(&Giant); /* XXX TTY layer */
		ucom_detach_sub(sc);
		mtx_unlock(&Giant); /* XXX TTY layer */

	        ucom_units_free(sc->sc_unit, 1);

		/* avoid duplicate detach: */
		sc->sc_flag &= ~UCOM_FLAG_ATTACHED;
	    }
	}
	return;
}

static int
ucom_attach_sub(struct ucom_softc *sc)
{
	struct tty *tp;
	int error = 0;

	tp = ttyalloc();

	if (tp == NULL) {
	    error = ENOMEM;
	    goto done;
	}

	tp->t_sc = sc;
	tp->t_oproc = ucom_start_write;
	tp->t_param = ucom_param;
	tp->t_stop  = ucom_stop_write;
	tp->t_break = ucom_break;
	tp->t_open  = ucom_open;
	tp->t_close = ucom_close;
	tp->t_modem = ucom_modem;
	tp->t_ioctl = ucom_ioctl;

	DPRINTF(0, "tp = %p, unit = %d\n", tp, sc->sc_unit);

#if !(defined(TS_CALLOUT) || (__FreeBSD_version >= 700022))
#define TS_CALLOUT NULL, sc->sc_unit, MINOR_CALLOUT /* compile fix for FreeBSD 6.x */
#endif
	error = ttycreate(tp, TS_CALLOUT, "U%d", sc->sc_unit);
	if (error) {
	    ttyfree(tp);
	    goto done;
	}

	sc->sc_tty = tp;

	TASK_INIT(&(sc->sc_task), 0, &ucom_notify, sc);

	DPRINTF(0, "ttycreate: ttyU%d\n", sc->sc_unit);

 done:
	return error;
}

static void
ucom_detach_sub(struct ucom_softc *sc)
{
	struct tty *tp = sc->sc_tty;

	DPRINTF(0, "sc = %p, tp = %p\n", sc, sc->sc_tty);

	sc->sc_flag |= UCOM_FLAG_GONE;
	sc->sc_flag &= ~(UCOM_FLAG_READ_ON|
			 UCOM_FLAG_WRITE_ON);
	if (tp) {

	    ttygone(tp);

	    if (tp->t_state & TS_ISOPEN) {
	        ucom_close(tp);
	    }

	    /* make sure that read and write 
	     * transfers are stopped
	     */
	    if (sc->sc_callback->ucom_stop_read) {
	        (sc->sc_callback->ucom_stop_read)(sc);
	    }

	    if (sc->sc_callback->ucom_stop_write) {
	        (sc->sc_callback->ucom_stop_write)(sc);
	    }

	    taskqueue_drain(taskqueue_swi_giant, &(sc->sc_task));

	    ttyfree(tp);
	}
	return;
}

static void
ucom_shutdown(struct ucom_softc *sc)
{
	struct tty *tp = sc->sc_tty;

	mtx_assert(&Giant, MA_OWNED);

	DPRINTF(0, "\n");
	/*
	 * Hang up if necessary:
	 */
	if (tp->t_cflag & HUPCL) {
	    ucom_modem(tp, 0, SER_DTR);
	}
	return;
}

static int
ucom_open(struct tty *tp, struct cdev *dev)
{
	struct ucom_softc *sc = tp->t_sc;
	int error;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return ENXIO;
	}

	/*
	 * wait a little for previous commands
	 * to be flushed out:
	 */
	error = ttysleep(tp, tp, PCATCH | TTIPRI, "ucomsd", hz);

	if (error && (error != EWOULDBLOCK)) {
	    return error;
	}

	DPRINTF(0, "tp = %p\n", tp);

	sc->sc_poll = 0;
	sc->sc_lsr = 0;
	sc->sc_msr = 0;
	sc->sc_mcr = 0;

	ucom_modem(tp, SER_DTR | SER_RTS, 0);

	if (sc->sc_callback->ucom_open) {

	    error = (sc->sc_callback->ucom_open)(sc);

	    if (error) {
	        ucom_shutdown(sc);
		return error;
	    }
	}

	sc->sc_flag |= UCOM_FLAG_READ_ON;

	if (sc->sc_callback->ucom_start_read) {
	    (sc->sc_callback->ucom_start_read)(sc);
	}

	sc->sc_poll = 1;

	return 0;
}

static void
ucom_close(struct tty *tp)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(&Giant, MA_OWNED);

	DPRINTF(0, "tp=%p\n", tp);

	tp->t_state &= ~TS_BUSY;

	ucom_shutdown(sc);

	sc->sc_flag &= ~(UCOM_FLAG_READ_ON|
			 UCOM_FLAG_WRITE_ON);

	if (sc->sc_callback->ucom_stop_read) {
	    (sc->sc_callback->ucom_stop_read)(sc);
	}

	if (sc->sc_callback->ucom_stop_write) {
	    (sc->sc_callback->ucom_stop_write)(sc);
	}

	if (sc->sc_callback->ucom_close) {
	    (sc->sc_callback->ucom_close)(sc);
	}
	return;
}

static int
ucom_ioctl(struct tty *tp, u_long cmd, void *data, 
	   int flag, struct thread *td)
{
	struct ucom_softc *sc = tp->t_sc;
	int error;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return EIO;
	}

	DPRINTF(0, "cmd = 0x%08lx\n", cmd);

	error = ENOTTY;
	if (sc->sc_callback->ucom_ioctl) {
	    error = (sc->sc_callback->ucom_ioctl)(sc, cmd, data, flag, td);
	}
	return error;
}

static int
ucom_modem(struct tty *tp, int sigon, int sigoff)
{
	struct ucom_softc *sc = tp->t_sc;
	u_int8_t onoff;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return 0;
	}

	if ((sigon == 0) && (sigoff == 0)) {

	    if (sc->sc_mcr & SER_DTR) {
	        sigon |= SER_DTR;
	    }
	    if (sc->sc_mcr & SER_RTS) {
	        sigon |= SER_RTS;
	    }

	    if (sc->sc_msr & SER_CTS) {
	        sigon |= SER_CTS;
	    }
	    if (sc->sc_msr & SER_DCD) {
	        sigon |= SER_DCD;
	    }
	    if (sc->sc_msr & SER_DSR) {
	        sigon |= SER_DSR;
	    }
	    if (sc->sc_msr & SER_RI) {
	        sigon |= SER_RI;
	    }
	    return sigon;
	}

	if (sigon & SER_DTR) {
	    sc->sc_mcr |= SER_DTR;
	}
	if (sigoff & SER_DTR) {
	    sc->sc_mcr &= ~SER_DTR;
	}
	if (sigon & SER_RTS) {
	    sc->sc_mcr |= SER_RTS;
	}
	if (sigoff & SER_RTS) {
	    sc->sc_mcr &= ~SER_RTS;
	}

	onoff = (sc->sc_mcr & SER_DTR) ? 1 : 0;
	ucom_dtr(sc, onoff);

	onoff = (sc->sc_mcr & SER_RTS) ? 1 : 0;
	ucom_rts(sc, onoff);

	return 0;
}

static void
ucom_break(struct tty *tp, int onoff)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return;
	}

	DPRINTF(0, "onoff = %d\n", onoff);
	if (sc->sc_callback->ucom_set_break) {
	    (sc->sc_callback->ucom_set_break)(sc, onoff);
	}
	return;
}

static void
ucom_dtr(struct ucom_softc *sc, u_int8_t onoff)
{
	DPRINTF(0, "onoff = %d\n", onoff);

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_callback->ucom_set_dtr) {
	    (sc->sc_callback->ucom_set_dtr)(sc, onoff);
	}
	return;
}

static void
ucom_rts(struct ucom_softc *sc, u_int8_t onoff)
{
	DPRINTF(0, "onoff = %d\n", onoff);

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_callback->ucom_set_rts) {
	    (sc->sc_callback->ucom_set_rts)(sc, onoff);
	}
	return;
}

void
ucom_status_change(struct ucom_softc *sc)
{
	u_int8_t old_msr;
	u_int8_t onoff;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_callback->ucom_get_status == NULL) {
	    sc->sc_lsr = 0;
	    sc->sc_msr = 0;
	    return;
	}

	old_msr = sc->sc_msr;

	(sc->sc_callback->ucom_get_status)(sc, &(sc->sc_lsr), &(sc->sc_msr));

	if ((sc->sc_msr ^ old_msr) & SER_DCD) {
	    if (sc->sc_poll == 0) {
	        return;
	    }
	    onoff = (sc->sc_msr & SER_DCD) ? 1 : 0;
	    DPRINTF(0, "DCD changed to %d\n", onoff);

	    sc->sc_last_status = onoff;

	    /* deferred notifying to the TTY layer */
	    taskqueue_enqueue(taskqueue_swi_giant, &(sc->sc_task));
	}
	return;
}

static void
ucom_notify(void *arg, int pending)
{
	struct ucom_softc *sc = arg;
	struct tty *tp = sc->sc_tty;

	ttyld_modem(tp, sc->sc_last_status);

	return;
}

static int
ucom_param(struct tty *tp, struct termios *t)
{
	struct ucom_softc *sc = tp->t_sc;
	int error;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return EIO;
	}

	DPRINTF(0, "sc = %p\n", sc);

	/* Check requested parameters. */
	if (t->c_ospeed < 0) {
	    DPRINTF(0, "negative ospeed\n");
	    return EINVAL;
	}
	if (t->c_ispeed && (t->c_ispeed != t->c_ospeed)) {
	    DPRINTF(0, "mismatch ispeed and ospeed\n");
	    return EINVAL;
	}

	/*
	 * If there were no changes, don't do anything.  This avoids dropping
	 * input and improves performance when all we did was frob things like
	 * VMIN and VTIME.
	 */
	if ((tp->t_ospeed == t->c_ospeed) &&
	    (tp->t_cflag == t->c_cflag)) {
	    return 0;
	}

	/* And copy to tty. */
	tp->t_ispeed = 0;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	if (sc->sc_callback->ucom_param == NULL) {
	    return 0;
	}

#if 0
	sc->sc_flag &= ~UCOM_FLAG_READ_ON;

	if (sc->sc_callback->ucom_stop_read) {
	    (sc->sc_callback->ucom_stop_read)(sc);
	}
#endif

	error = (sc->sc_callback->ucom_param)(sc, t);
	if (error) {
	    DPRINTF(0, "callback error = %d\n", error);
	    return error;
	}

	ttsetwater(tp);

	if (t->c_cflag & CRTS_IFLOW) {
	    sc->sc_flag |= UCOM_FLAG_RTS_IFLOW;
	} else if (sc->sc_flag & UCOM_FLAG_RTS_IFLOW) {
	    sc->sc_flag &= ~UCOM_FLAG_RTS_IFLOW;
	    ucom_modem(tp, SER_RTS, 0);
	}

	ttyldoptim(tp);

#if 0
	sc->sc_flag |= UCOM_FLAG_READ_ON;

	if (sc->sc_callback->ucom_start_read) {
	    (sc->sc_callback->ucom_start_read)(sc);
	}
#endif

	return 0;
}

static void
ucom_start_write(struct tty *tp)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(&Giant, MA_OWNED);

	DPRINTF(0, "sc = %p\n", sc);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return;
	}

	tp->t_state |= TS_BUSY;

	sc->sc_flag |= UCOM_FLAG_WRITE_ON;

	if (sc->sc_callback->ucom_start_write) {
	    (sc->sc_callback->ucom_start_write)(sc);
	}
	return;
}

static void
ucom_stop_write(struct tty *tp, int fflags)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(&Giant, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
	    return;
	}

	DPRINTF(0, "fflags=%d\n", fflags);

	if (fflags & FWRITE) {
	    DPRINTF(0, "write\n");
	    if (tp->t_state & TS_BUSY) {
	        /* XXX do what? */
	        if (!(tp->t_state & TS_TTSTOP)) {
		    tp->t_state |= TS_FLUSH;
		}
	    }
	}

	ucom_start_write(tp);

	DPRINTF(0, "done\n");

	return;
}

/*
 * the following function returns 
 * 1 if data is available, else 0
 */

u_int8_t
ucom_get_data(struct ucom_softc *sc, u_int8_t *buf, u_int32_t len,
	      u_int32_t *actlen)
{
	struct tty *tp = sc->sc_tty;
	u_int32_t cnt;

	mtx_assert(&Giant, MA_OWNED);

	actlen[0] = 0;

	if (!(sc->sc_flag & UCOM_FLAG_WRITE_ON)) {
	    return 0; /* multiport device polling */
	}

	if (tp->t_state & TS_TBLOCK) {
	    if ((sc->sc_mcr & SER_RTS) &&
		(sc->sc_flag & UCOM_FLAG_RTS_IFLOW)) {
	        DPRINTF(0, "clear RTS\n");
		ucom_modem(tp, 0, SER_RTS);
	    }
	} else {
	    if (!(sc->sc_mcr & SER_RTS) &&
		(tp->t_rawq.c_cc <= tp->t_ilowat) &&
		(sc->sc_flag & UCOM_FLAG_RTS_IFLOW)) {
	        DPRINTF(0, "set RTS\n");
		ucom_modem(tp, SER_RTS, 0);
	    }
	}

	if (tp->t_state & (TS_TIMEOUT | TS_TTSTOP)) {
	    DPRINTF(0, "stopped\n");
	    goto done;
	}

	cnt = q_to_b(&(tp->t_outq), buf, len);

	if (cnt > len) {
	    DPRINTF(0, "invalid length, %d bytes\n", cnt);
	    goto done;
	}

	if (cnt == 0) {
	    DPRINTF(0, "cnt == 0\n");
	    goto done;
	}

	actlen[0] = cnt;

	ttwwakeup(tp);

	return 1;

 done:
	tp->t_state &= ~(TS_BUSY | TS_FLUSH);

	ttwwakeup(tp);

	return 0;
}

void
ucom_put_data(struct ucom_softc *sc, u_int8_t *ptr, u_int16_t len)
{
	struct tty *tp = sc->sc_tty;
	u_int16_t lostcc;

	mtx_assert(&Giant, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_READ_ON)) {
	    return; /* multiport device polling */
	}

	/* set a flag to prevent recursation */

	if (len == 0) {
	    goto done;
	}

	if (tp->t_state & TS_CAN_BYPASS_L_RINT) {
	    if (((tp->t_rawq.c_cc + len) > tp->t_ihiwat) &&
		((sc->sc_flag & UCOM_FLAG_RTS_IFLOW) ||
		 (tp->t_iflag & IXOFF)) &&
		(!(tp->t_state & TS_TBLOCK))) {
	        ttyblock(tp);
	    }

	    lostcc = b_to_q(ptr, len, &(tp->t_rawq));

	    tp->t_rawcc += len;

	    ttwakeup(tp);

	    if ((tp->t_state & TS_TTSTOP) &&
		((tp->t_iflag & IXANY) ||
		 (tp->t_cc[VSTART] == tp->t_cc[VSTOP]))) {
	        tp->t_state &= ~TS_TTSTOP;
		tp->t_lflag &= ~FLUSHO;
		ucom_start_write(tp);
	    }
	    if (lostcc > 0) {
	        DPRINTF(0, "tp=%p, lost %d "
			"chars\n", tp, lostcc);
	    }
	} else {
	    /* pass characters to tty layer */
	    while (len) {
	        DPRINTF(7, "char = 0x%02x\n", *ptr);

		if (ttyld_rint(tp, *ptr) == -1) {

		    /* XXX what should we do? */

		    DPRINTF(0, "tp=%p, lost %d "
			    "chars\n", tp, len);
		    break;
		}
		len--;
		ptr++;
	    }
	}

 done:
	if ((sc->sc_flag & UCOM_FLAG_RTS_IFLOW) && 
	    (!(sc->sc_mcr & SER_RTS)) &&
	    (!(tp->t_state & TS_TBLOCK))) {
	    ucom_modem(tp, SER_RTS, 0);
	}
	return;
}
