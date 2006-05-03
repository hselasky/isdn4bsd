/*
 * Copyright (c) 2001 Cubical Solutions Ltd. All rights reserved.
 *
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
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
#ifndef _I4B_CAPI_MSGS_H_
#define _I4B_CAPI_MSGS_H_

/*
//  CAPI message handlers called by higher layers
*/

extern void capi_listen_req(capi_softc_t *sc, u_int32_t CIP);
extern void capi_alert_req(capi_softc_t *sc, call_desc_t *cd);
extern void capi_connect_req(capi_softc_t *sc, call_desc_t *cd);
extern void capi_connect_b3_req(capi_softc_t *sc, call_desc_t *cd);
extern void capi_connect_resp(capi_softc_t *sc, call_desc_t *cd);
extern void capi_data_b3_req(capi_softc_t *sc, int chan, struct mbuf *m);
extern void capi_disconnect_req(capi_softc_t *sc, call_desc_t *cd);

/*
//  CAPI message handlers called by the receive routine
*/

extern void capi_listen_conf(capi_softc_t *sc, struct mbuf *m);
extern void capi_info_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_alert_conf(capi_softc_t *sc, struct mbuf *m);
extern void capi_connect_conf(capi_softc_t *sc, struct mbuf *m);
extern void capi_connect_active_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_connect_b3_conf(capi_softc_t *sc, struct mbuf *m);
extern void capi_connect_b3_active_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_connect_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_connect_b3_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_data_b3_conf(capi_softc_t *sc, struct mbuf *m);
extern void capi_data_b3_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_disconnect_conf(capi_softc_t *sc, struct mbuf *m);
extern void capi_disconnect_b3_ind(capi_softc_t *sc, struct mbuf *m);
extern void capi_disconnect_ind(capi_softc_t *sc, struct mbuf *m);

#endif /* _I4B_CAPI_MSGS_H_ */
