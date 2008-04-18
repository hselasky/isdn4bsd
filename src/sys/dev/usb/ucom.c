/*	$NetBSD: ucom.c,v 1.40 2001/11/13 06:24:54 lukem Exp $	*/

/*-
 * Copyright (c) 2001-2003, 2005, 2008
 *	Shunsuke Akiyama <akiyama@jp.FreeBSD.org>.
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
__FBSDID("$FreeBSD: src/sys/dev/usb/ucom.c,v 1.66 2008/03/25 23:46:24 sam Exp $");

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

/*
 * NOTE: all function names beginning like "ucom_cfg_" can only
 * be called from within the config thread function !
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/tty.h>
#include <sys/serial.h>

#define	usbd_config_td_cc ucom_config_copy
#define	usbd_config_td_softc ucom_softc

#include <dev/usb/usb_port.h>
#include <dev/usb/usb.h>
#include <dev/usb/usb_subr.h>
#include <dev/usb/usb_quirks.h>
#include <dev/usb/usb_cdc.h>

#include <dev/usb/ucomvar.h>

#ifdef USB_DEBUG
#define	DPRINTF(n,fmt,...)						\
  do { if (ucom_debug > (n)) {						\
      printf("%s: " fmt, __FUNCTION__,## __VA_ARGS__); } } while (0)

static int ucom_debug = 0;

SYSCTL_NODE(_hw_usb, OID_AUTO, ucom, CTLFLAG_RW, 0, "USB ucom");
SYSCTL_INT(_hw_usb_ucom, OID_AUTO, debug, CTLFLAG_RW,
    &ucom_debug, 0, "ucom debug level");
#else
#define	DPRINTF(...) do { } while (0)
#endif

struct ucom_config_copy {
	struct ucom_softc *cc_softc;
	uint8_t	cc_flag0;
	uint8_t	cc_flag1;
	uint8_t	cc_flag2;
	uint8_t	cc_flag3;
};

static usbd_config_td_command_t ucom_config_copy;
static usbd_config_td_command_t ucom_cfg_start_transfers;
static usbd_config_td_command_t ucom_cfg_open;
static usbd_config_td_command_t ucom_cfg_close;
static usbd_config_td_command_t ucom_cfg_break;
static usbd_config_td_command_t ucom_cfg_dtr;
static usbd_config_td_command_t ucom_cfg_rts;
static usbd_config_td_command_t ucom_cfg_status_change;
static usbd_config_td_command_t ucom_cfg_param;

static int ucom_modevent(module_t mod, int type, void *data);
static uint8_t ucom_units_alloc(uint32_t sub_units, uint32_t *p_root_unit);
static void ucom_units_free(uint32_t root_unit, uint32_t sub_units);
static int ucom_attach_sub(struct ucom_softc *sc);
static void ucom_detach_sub(struct ucom_softc *sc);
static void ucom_queue_command(struct ucom_softc *sc, usbd_config_td_command_t *cmd, int flag);
static void ucom_shutdown(struct ucom_softc *sc);
static void ucom_start_transfers(struct ucom_softc *sc);
static int ucom_open(struct tty *tp, struct cdev *dev);
static void ucom_close(struct tty *tp);
static int ucom_ioctl(struct tty *tp, u_long cmd, void *data, int flag, struct thread *td);
static int ucom_modem(struct tty *tp, int sigon, int sigoff);
static void ucom_break(struct tty *tp, int onoff);
static void ucom_dtr(struct ucom_softc *sc, uint8_t onoff);
static void ucom_rts(struct ucom_softc *sc, uint8_t onoff);
static int ucom_param(struct tty *tp, struct termios *t);
static void ucom_start_write(struct tty *tp);
static void ucom_stop_write(struct tty *tp, int fflags);

static moduledata_t ucom_mod = {
	"ucom",
	ucom_modevent,
	NULL
};

DECLARE_MODULE(ucom, ucom_mod, SI_SUB_DRIVERS, SI_ORDER_MIDDLE);
MODULE_DEPEND(ucom, usb, 1, 1, 1);
MODULE_VERSION(ucom, UCOM_MODVER);

static int
ucom_modevent(module_t mod, int type, void *data)
{
	;				/* style fix */
	switch (type) {
	case MOD_LOAD:
		break;
	case MOD_UNLOAD:
		break;
	default:
		return (EOPNOTSUPP);
		break;
	}
	return (0);
}

#define	UCOM_UNIT_MAX 0x1000		/* exclusive */
#define	UCOM_SUB_UNIT_MAX 0x100		/* exclusive */

static uint8_t ucom_bitmap[(UCOM_UNIT_MAX + 7) / 8];

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

			if (ucom_bitmap[x / 8] & (1 << (x % 8))) {
				goto skip;
			}
		}

		/* allocate */

		for (o = 0; o < sub_units; o++) {

			x = n + o;

			ucom_bitmap[x / 8] |= (1 << (x % 8));
		}

		*p_root_unit = n;

		error = 0;

		break;

skip:		;
	}

	mtx_unlock(&Giant);

	return (error);
}

static void
ucom_units_free(uint32_t root_unit, uint32_t sub_units)
{
	uint32_t x;

	mtx_lock(&Giant);

	while (sub_units--) {
		x = root_unit + sub_units;
		ucom_bitmap[x / 8] &= ~(1 << (x % 8));
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
ucom_attach(struct ucom_super_softc *ssc, struct ucom_softc *sc,
    uint32_t sub_units, void *parent,
    const struct ucom_callback *callback, struct mtx *p_mtx)
{
	uint32_t n;
	uint32_t root_unit;
	int error = 0;

	if ((p_mtx != &Giant) ||	/* XXX TTY layer requires Giant */
	    (sc == NULL) ||
	    (sub_units == 0) ||
	    (sub_units > UCOM_SUB_UNIT_MAX) ||
	    (callback == NULL)) {
		return (EINVAL);
	}
	if (ucom_units_alloc(sub_units, &root_unit)) {
		return (ENOMEM);
	}
	if (usbd_config_td_setup
	    (&(ssc->sc_config_td), sc, p_mtx, NULL,
	    sizeof(struct ucom_config_copy), 24 * sub_units)) {
		ucom_units_free(root_unit, sub_units);
		return (ENOMEM);
	}
	for (n = 0; n < sub_units; n++, sc++) {
		sc->sc_unit = root_unit + n;
		sc->sc_local_unit = n;
		sc->sc_super = ssc;
		sc->sc_parent_mtx = p_mtx;
		sc->sc_parent = parent;
		sc->sc_callback = callback;

		mtx_lock(&Giant);	/* XXX TTY layer */
		error = ucom_attach_sub(sc);
		mtx_unlock(&Giant);	/* XXX TTY layer */

		if (error) {
			ucom_detach(ssc, sc - n, n);
			ucom_units_free(root_unit + n, sub_units - n);
			break;
		}
		sc->sc_flag |= UCOM_FLAG_ATTACHED;
	}
	return (error);
}

/* NOTE: the following function will do nothing if
 * the structure pointed to by "ssc" and "sc" is zero.
 */
void
ucom_detach(struct ucom_super_softc *ssc, struct ucom_softc *sc,
    uint32_t sub_units)
{
	uint32_t n;

	usbd_config_td_stop(&(ssc->sc_config_td));

	for (n = 0; n < sub_units; n++, sc++) {
		if (sc->sc_flag & UCOM_FLAG_ATTACHED) {

			mtx_lock(&Giant);	/* XXX TTY layer */
			ucom_detach_sub(sc);
			mtx_unlock(&Giant);	/* XXX TTY layer */

			ucom_units_free(sc->sc_unit, 1);

			/* avoid duplicate detach: */
			sc->sc_flag &= ~UCOM_FLAG_ATTACHED;
		}
	}

	usbd_config_td_unsetup(&(ssc->sc_config_td));

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
	tp->t_stop = ucom_stop_write;
	tp->t_break = ucom_break;
	tp->t_open = ucom_open;
	tp->t_close = ucom_close;
	tp->t_modem = ucom_modem;
	tp->t_ioctl = ucom_ioctl;

	DPRINTF(0, "tp = %p, unit = %d\n", tp, sc->sc_unit);

#if !(defined(TS_CALLOUT) || (__FreeBSD_version >= 700022))
#define	TS_CALLOUT NULL, sc->sc_unit, MINOR_CALLOUT	/* compile fix for
							 * FreeBSD 6.x */
#endif
	error = ttycreate(tp, TS_CALLOUT, "U%d", sc->sc_unit);
	if (error) {
		ttyfree(tp);
		goto done;
	}
	sc->sc_tty = tp;

	DPRINTF(0, "ttycreate: ttyU%d\n", sc->sc_unit);

done:
	return (error);
}

static void
ucom_detach_sub(struct ucom_softc *sc)
{
	struct tty *tp = sc->sc_tty;

	DPRINTF(0, "sc = %p, tp = %p\n", sc, sc->sc_tty);

	/* the config thread is stopped when we get here */

	sc->sc_flag |= UCOM_FLAG_GONE;
	sc->sc_flag &= ~(UCOM_FLAG_HL_READY |
	    UCOM_FLAG_LL_READY);
	if (tp) {

		ttygone(tp);

		if (tp->t_state & TS_ISOPEN) {
			ucom_close(tp);
		}
		/*
		 * make sure that read and write transfers are stopped
		 */
		if (sc->sc_callback->ucom_stop_read) {
			(sc->sc_callback->ucom_stop_read) (sc);
		}
		if (sc->sc_callback->ucom_stop_write) {
			(sc->sc_callback->ucom_stop_write) (sc);
		}
		ttyfree(tp);
	}
	return;
}

static void
ucom_config_copy(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	cc->cc_softc = sc + (refcount % UCOM_SUB_UNIT_MAX);
	cc->cc_flag0 = (refcount / (1 * UCOM_SUB_UNIT_MAX)) % 2;
	cc->cc_flag1 = (refcount / (2 * UCOM_SUB_UNIT_MAX)) % 2;
	cc->cc_flag2 = (refcount / (4 * UCOM_SUB_UNIT_MAX)) % 2;
	cc->cc_flag3 = (refcount / (8 * UCOM_SUB_UNIT_MAX)) % 2;
	return;
}

static void
ucom_queue_command(struct ucom_softc *sc, usbd_config_td_command_t *cmd, int flag)
{
	struct ucom_super_softc *ssc = sc->sc_super;

	usbd_config_td_queue_command
	    (&(ssc->sc_config_td), &ucom_config_copy,
	    cmd, (cmd == &ucom_cfg_status_change) ? 1 : 0,
	    ((sc->sc_local_unit % UCOM_SUB_UNIT_MAX) +
	    (flag ? UCOM_SUB_UNIT_MAX : 0)));
	return;
}

static void
ucom_shutdown(struct ucom_softc *sc)
{
	struct tty *tp = sc->sc_tty;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	DPRINTF(0, "\n");

	/*
	 * Hang up if necessary:
	 */
	if (tp->t_cflag & HUPCL) {
		ucom_modem(tp, 0, SER_DTR);
	}
	return;
}

/*
 * Return values:
 *    0: normal delay
 * else: config thread is gone
 */
uint8_t
ucom_cfg_sleep(struct ucom_softc *sc, uint32_t timeout)
{
	struct ucom_super_softc *ssc = sc->sc_super;

	return (usbd_config_td_sleep(&(ssc->sc_config_td), timeout));
}

/*
 * Return values:
 *    0: normal
 * else: config thread is gone
 */
uint8_t
ucom_cfg_is_gone(struct ucom_softc *sc)
{
	struct ucom_super_softc *ssc = sc->sc_super;

	return (usbd_config_td_is_gone(&(ssc->sc_config_td)));
}

static void
ucom_cfg_start_transfers(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	sc = cc->cc_softc;

	if (!(sc->sc_flag & UCOM_FLAG_LL_READY)) {
		return;
	}
	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		/* TTY device closed */
		return;
	}
	sc->sc_flag |= UCOM_FLAG_GP_DATA;

	if (sc->sc_callback->ucom_start_read) {
		(sc->sc_callback->ucom_start_read) (sc);
	}
	if (sc->sc_callback->ucom_start_write) {
		(sc->sc_callback->ucom_start_write) (sc);
	}
	return;
}

static void
ucom_start_transfers(struct ucom_softc *sc)
{
	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return;
	}
	/*
	 * do a direct call first, to get hardware buffers flushed
	 */

	if (sc->sc_callback->ucom_start_read) {
		(sc->sc_callback->ucom_start_read) (sc);
	}
	if (sc->sc_callback->ucom_start_write) {
		(sc->sc_callback->ucom_start_write) (sc);
	}
	if (!(sc->sc_flag & UCOM_FLAG_GP_DATA)) {
		ucom_queue_command(sc, &ucom_cfg_start_transfers, 0);
	}
	return;
}

static void
ucom_cfg_open(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	sc = cc->cc_softc;

	DPRINTF(0, "\n");

	if (sc->sc_flag & UCOM_FLAG_LL_READY) {

		/* already opened */

	} else {

		sc->sc_flag |= UCOM_FLAG_LL_READY;

		if (sc->sc_callback->ucom_cfg_open) {
			(sc->sc_callback->ucom_cfg_open) (sc);

			/* wait a little */
			ucom_cfg_sleep(sc, hz / 10);
		}
	}
	return;
}

static int
ucom_open(struct tty *tp, struct cdev *dev)
{
	struct ucom_softc *sc = tp->t_sc;
	int error;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (sc->sc_flag & UCOM_FLAG_GONE) {
		return (ENXIO);
	}
	if (sc->sc_flag & UCOM_FLAG_HL_READY) {
		/* already opened */
		return (0);
	}
	DPRINTF(0, "tp = %p\n", tp);

	if (sc->sc_callback->ucom_pre_open) {
		/*
		 * give the lower layer a chance to disallow TTY open, for
		 * example if the device is not present:
		 */
		error = (sc->sc_callback->ucom_pre_open) (sc);
		if (error) {
			return (error);
		}
	}
	sc->sc_flag |= UCOM_FLAG_HL_READY;

	/* Disable transfers */
	sc->sc_flag &= ~UCOM_FLAG_GP_DATA;

	sc->sc_lsr = 0;
	sc->sc_msr = 0;
	sc->sc_mcr = 0;

	ucom_queue_command(sc, &ucom_cfg_open, 0);

	ucom_start_transfers(sc);

	ucom_modem(tp, SER_DTR | SER_RTS, 0);

	ucom_break(tp, 0);

	ucom_status_change(sc);

	return (0);
}

static void
ucom_cfg_close(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	sc = cc->cc_softc;

	DPRINTF(0, "\n");

	if (sc->sc_flag & UCOM_FLAG_LL_READY) {

		sc->sc_flag &= ~(UCOM_FLAG_LL_READY |
		    UCOM_FLAG_GP_DATA);

		if (sc->sc_callback->ucom_cfg_close) {
			(sc->sc_callback->ucom_cfg_close) (sc);
		}
	} else {
		/* already closed */
	}
	return;
}

static void
ucom_close(struct tty *tp)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	DPRINTF(0, "tp=%p\n", tp);

	tp->t_state &= ~TS_BUSY;

	ucom_shutdown(sc);

	ucom_queue_command(sc, &ucom_cfg_close, 0);

	sc->sc_flag &= ~(UCOM_FLAG_HL_READY |
	    UCOM_FLAG_WR_START |
	    UCOM_FLAG_RTS_IFLOW);

	if (sc->sc_callback->ucom_stop_read) {
		(sc->sc_callback->ucom_stop_read) (sc);
	}
	if (sc->sc_callback->ucom_stop_write) {
		(sc->sc_callback->ucom_stop_write) (sc);
	}
	return;
}

static int
ucom_ioctl(struct tty *tp, u_long cmd, void *data,
    int flag, struct thread *td)
{
	struct ucom_softc *sc = tp->t_sc;
	int error;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return (EIO);
	}
	DPRINTF(0, "cmd = 0x%08lx\n", cmd);

	error = ENOTTY;
	if (sc->sc_callback->ucom_ioctl) {
		error = (sc->sc_callback->ucom_ioctl) (sc, cmd, data, flag, td);
	}
	return (error);
}

static int
ucom_modem(struct tty *tp, int sigon, int sigoff)
{
	struct ucom_softc *sc = tp->t_sc;
	uint8_t onoff;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return (0);
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
		return (sigon);
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

	return (0);
}

static void
ucom_cfg_break(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	sc = cc->cc_softc;

	if (!(sc->sc_flag & UCOM_FLAG_LL_READY)) {
		return;
	}
	DPRINTF(0, "onoff=%d\n", cc->cc_flag0);

	if (sc->sc_callback->ucom_cfg_set_break) {
		(sc->sc_callback->ucom_cfg_set_break) (sc, cc->cc_flag0);
	}
	return;
}

static void
ucom_break(struct tty *tp, int onoff)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return;
	}
	DPRINTF(0, "onoff = %d\n", onoff);

	ucom_queue_command(sc, &ucom_cfg_break, onoff);
	return;
}

static void
ucom_cfg_dtr(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	sc = cc->cc_softc;

	if (!(sc->sc_flag & UCOM_FLAG_LL_READY)) {
		return;
	}
	DPRINTF(0, "onoff=%d\n", cc->cc_flag0);

	if (sc->sc_callback->ucom_cfg_set_dtr) {
		(sc->sc_callback->ucom_cfg_set_dtr) (sc, cc->cc_flag0);
	}
	return;
}

static void
ucom_dtr(struct ucom_softc *sc, uint8_t onoff)
{
	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return;
	}
	DPRINTF(0, "onoff = %d\n", onoff);

	ucom_queue_command(sc, &ucom_cfg_dtr, onoff);
	return;
}

static void
ucom_cfg_rts(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	sc = cc->cc_softc;

	DPRINTF(0, "onoff=%d\n", cc->cc_flag0);

	if (!(sc->sc_flag & UCOM_FLAG_LL_READY)) {
		return;
	}
	if (sc->sc_callback->ucom_cfg_set_rts) {
		(sc->sc_callback->ucom_cfg_set_rts) (sc, cc->cc_flag0);
	}
	return;
}

static void
ucom_rts(struct ucom_softc *sc, uint8_t onoff)
{
	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return;
	}
	DPRINTF(0, "onoff = %d\n", onoff);

	ucom_queue_command(sc, &ucom_cfg_rts, onoff);

	return;
}

static void
ucom_cfg_status_change(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	struct tty *tp;

	uint8_t new_msr;
	uint8_t new_lsr;
	uint8_t onoff;

	sc = cc->cc_softc;
	tp = sc->sc_tty;

	if (!(sc->sc_flag & UCOM_FLAG_LL_READY)) {
		return;
	}
	if (sc->sc_callback->ucom_cfg_get_status == NULL) {
		return;
	}
	/* get status */

	new_msr = 0;
	new_lsr = 0;

	(sc->sc_callback->ucom_cfg_get_status) (sc, &new_lsr, &new_msr);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		/* TTY device closed */
		return;
	}
	onoff = ((sc->sc_msr ^ new_msr) & SER_DCD);

	sc->sc_msr = new_msr;
	sc->sc_lsr = new_lsr;

	if (onoff) {

		onoff = (sc->sc_msr & SER_DCD) ? 1 : 0;

		DPRINTF(0, "DCD changed to %d\n", onoff);

		ttyld_modem(tp, onoff);
	}
	return;
}

void
ucom_status_change(struct ucom_softc *sc)
{
	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return;
	}
	DPRINTF(0, "\n");

	ucom_queue_command(sc, &ucom_cfg_status_change, 0);
	return;
}

static void
ucom_cfg_param(struct ucom_softc *sc, struct ucom_config_copy *cc,
    uint16_t refcount)
{
	struct termios t_copy;

	sc = cc->cc_softc;

	if (!(sc->sc_flag & UCOM_FLAG_LL_READY)) {
		return;
	}
	if (sc->sc_callback->ucom_cfg_param == NULL) {
		return;
	}
	t_copy = sc->sc_termios_copy;

	(sc->sc_callback->ucom_cfg_param) (sc, &t_copy);

	/* wait a little */
	ucom_cfg_sleep(sc, hz / 10);

	return;
}

static int
ucom_param(struct tty *tp, struct termios *t)
{
	struct ucom_softc *sc = tp->t_sc;
	uint8_t opened;
	int error;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	opened = 0;
	error = 0;

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {

		/* XXX the TTY layer should call "open()" first! */

		error = ucom_open(tp, NULL);
		if (error) {
			goto done;
		}
		opened = 1;
	}
	DPRINTF(0, "sc = %p\n", sc);

	/* Check requested parameters. */
	if (t->c_ospeed < 0) {
		DPRINTF(0, "negative ospeed\n");
		error = EINVAL;
		goto done;
	}
	if (t->c_ispeed && (t->c_ispeed != t->c_ospeed)) {
		DPRINTF(0, "mismatch ispeed and ospeed\n");
		error = EINVAL;
		goto done;
	}
	/*
	 * If there were no changes, don't do anything.  This avoids dropping
	 * input and improves performance when all we did was frob things like
	 * VMIN and VTIME.
	 */
	if ((tp->t_ospeed == t->c_ospeed) &&
	    (tp->t_cflag == t->c_cflag)) {
		error = 0;
		goto done;
	}
	/* And copy to tty. */
	tp->t_ispeed = 0;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	if (sc->sc_callback->ucom_pre_param) {
		/* Let the lower layer verify the parameters */
		error = (sc->sc_callback->ucom_pre_param) (sc, t);
		if (error) {
			DPRINTF(0, "callback error = %d\n", error);
			goto done;
		}
	}
	/* Make a copy of the termios parameters */
	sc->sc_termios_copy = *t;

	/* Disable transfers */
	sc->sc_flag &= ~UCOM_FLAG_GP_DATA;

	/* Queue baud rate programming command first */
	ucom_queue_command(sc, &ucom_cfg_param, 0);

	/* Queue transfer enable command last */
	ucom_start_transfers(sc);

	ttsetwater(tp);

	if (t->c_cflag & CRTS_IFLOW) {
		sc->sc_flag |= UCOM_FLAG_RTS_IFLOW;
	} else if (sc->sc_flag & UCOM_FLAG_RTS_IFLOW) {
		sc->sc_flag &= ~UCOM_FLAG_RTS_IFLOW;
		ucom_modem(tp, SER_RTS, 0);
	}
	ttyldoptim(tp);

done:
	if (error) {
		if (opened) {
			ucom_close(tp);
		}
	}
	return (error);
}

static void
ucom_start_write(struct tty *tp)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	DPRINTF(0, "sc = %p\n", sc);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
		return;
	}
	tp->t_state |= TS_BUSY;

	sc->sc_flag |= UCOM_FLAG_WR_START;

	ucom_start_transfers(sc);

	return;
}

static void
ucom_stop_write(struct tty *tp, int fflags)
{
	struct ucom_softc *sc = tp->t_sc;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if (!(sc->sc_flag & UCOM_FLAG_HL_READY)) {
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
	/* Flush out any leftover data */
	ucom_start_write(tp);

	DPRINTF(0, "done\n");

	return;
}

/*
 * the following function returns
 * 1 if data is available, else 0
 */

uint8_t
ucom_get_data(struct ucom_softc *sc, struct usbd_page_cache *pc,
    uint32_t offset, uint32_t len, uint32_t *actlen)
{
	struct usbd_page_search res;
	struct tty *tp = sc->sc_tty;
	uint32_t cnt;
	uint32_t offset_orig;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	actlen[0] = 0;

	if ((!(sc->sc_flag & UCOM_FLAG_HL_READY)) ||
	    (!(sc->sc_flag & UCOM_FLAG_GP_DATA)) ||
	    (!(sc->sc_flag & UCOM_FLAG_WR_START))) {
		return (0);		/* multiport device polling */
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
	offset_orig = offset;

	while (len != 0) {

		usbd_get_page(pc, offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		cnt = q_to_b(&(tp->t_outq), res.buffer, res.length);

		offset += cnt;
		len -= cnt;

		if (cnt < res.length) {
			/* end of buffer */
			break;
		}
	}

	actlen[0] = offset - offset_orig;

	DPRINTF(0, "cnt=%d\n", actlen[0]);

	if (actlen[0] == 0) {
		goto done;
	}
	ttwwakeup(tp);

	return (1);

done:
	tp->t_state &= ~(TS_BUSY | TS_FLUSH);

	ttwwakeup(tp);

	return (0);
}

void
ucom_put_data(struct ucom_softc *sc, struct usbd_page_cache *pc,
    uint32_t offset, uint32_t len)
{
	struct usbd_page_search res;
	struct tty *tp = sc->sc_tty;
	uint32_t cnt;

	mtx_assert(sc->sc_parent_mtx, MA_OWNED);

	if ((!(sc->sc_flag & UCOM_FLAG_HL_READY)) ||
	    (!(sc->sc_flag & UCOM_FLAG_GP_DATA))) {
		return;			/* multiport device polling */
	}
	/* set a flag to prevent recursation ? */

	while (len > 0) {

		usbd_get_page(pc, offset, &res);

		if (res.length > len) {
			res.length = len;
		}
		len -= res.length;
		offset += res.length;

		if (tp->t_state & TS_CAN_BYPASS_L_RINT) {

			if (((tp->t_rawq.c_cc + res.length) > tp->t_ihiwat) &&
			    ((sc->sc_flag & UCOM_FLAG_RTS_IFLOW) ||
			    (tp->t_iflag & IXOFF)) &&
			    (!(tp->t_state & TS_TBLOCK))) {
				ttyblock(tp);
			}
			cnt = b_to_q(res.buffer, res.length, &(tp->t_rawq));

			tp->t_rawcc += res.length;

			ttwakeup(tp);

			if ((tp->t_state & TS_TTSTOP) &&
			    ((tp->t_iflag & IXANY) ||
			    (tp->t_cc[VSTART] == tp->t_cc[VSTOP]))) {
				tp->t_state &= ~TS_TTSTOP;
				tp->t_lflag &= ~FLUSHO;
				ucom_start_write(tp);
			}
			if (cnt > 0) {
				DPRINTF(0, "tp=%p, lost %d "
				    "chars\n", tp, cnt);
			}
		} else {

			uint8_t *buf;

			/* pass characters to tty layer */

			buf = res.buffer;

			for (cnt = 0; cnt != res.length; cnt++) {

				if (ttyld_rint(tp, buf[cnt]) == -1) {

					/* XXX what should we do? */

					DPRINTF(0, "tp=%p, lost %d "
					    "chars\n", tp, res.length - cnt);
					break;
				}
			}
		}
	}

	if ((sc->sc_flag & UCOM_FLAG_RTS_IFLOW) &&
	    (!(sc->sc_mcr & SER_RTS)) &&
	    (!(tp->t_state & TS_TBLOCK))) {
		ucom_modem(tp, SER_RTS, 0);
	}
	return;
}
