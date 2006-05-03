/*-
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
 *
 * capi/capi_msgs.c	The CAPI I4B message handlers.
 *
 * $FreeBSD: src/sys/i4b/capi/capi_msgs.c $
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>
#include <i4b/include/i4b_global.h>

#include <i4b/include/capi20.h>

#include <i4b/layer4/i4b_l4.h>

#include <i4b/capi/capi.h>
#include <i4b/capi/capi_msgs.h>

/*
//  Transmission of CAPI messages
//  -----------------------------
*/

static u_int16_t
capi_send_decoded(struct i4b_controller *cntl, u_int16_t wCmd, 
		  u_int16_t wMsgNum, u_int32_t dwCid, struct mbuf *m2, 
		  void *p_msg)
{
    struct CAPI_MESSAGE_ENCODED msg;
    u_int16_t len;
    u_int16_t error;
    struct mbuf *m1;

    CNTL_LOCK_ASSERT(cntl);

    len = capi_encode(&msg.data, sizeof(msg.data), p_msg);
    len += sizeof(msg.head);

    msg.head.wLen = htole16(len);
    msg.head.wApp = htole16(I4BCAPI_APPLID);
    msg.head.wCmd = htole16(wCmd);
    msg.head.wNum = htole16(wMsgNum);
    msg.head.dwCid = htole32(dwCid);

    m1 = i4b_getmbuf(len, M_NOWAIT);

    if (m1) {
        bcopy(&msg, m1->m_data, m1->m_len);
	m1->m_next = m2;
	error = (sc->send)(sc, m1);
    } else {
        error = 0x1008; /* OS resource error */
	m_free(m2);
    }

    if (error) {
        printf("WARNING: %s: CAPI cannot send "
	       "cmd=0x%04x, len=%d bytes!\n", 
	       __FUNCTION__, wCmd,len);

	/* XXX do what: 
	 * Tear down all active connections?
	 * And reset controller?
	 */
    }
    return error;
}

static u_int16_t
capi_send_decoded_std(struct i4b_controller *cntl, u_int16_t wCmd, 
		    u_int32_t dwCid, void *p_msg)
{
    struct capi_softc *sc = cntl->L1_softc;
    return capi_send_decoded(cntl, wCmd, (sc->sc_msg_id)++, dwCid, NULL, p_msg);
}

static u_int16_t
capi_send_decoded_b3(struct i4b_controller *cntl, u_int16_t wCmd, 
		     u_int32_t dwCid, struct mbuf *m2, void *p_msg)
{
    struct capi_softc *sc = cntl->L1_softc;
    return capi_send_decoded(cntl, wCmd, (sc->sc_msg_id)++, dwCid, m2, p_msg);
}

static u_int16_t
capi_send_decoded_reply(struct i4b_controller *cntl, u_int16_t wCmd,
			struct mbuf *m_in, void *p_msg)
{
    struct capi_softc *sc = cntl->L1_softc;
    struct CAPI_HEADER_ENCODED *hdr = (void *)(m_in->m_data);
    return capi_send_decoded(cntl, wCmd, hdr->wMsgNum, hdr->dwCid, NULL, p_msg);
}

static void
capi_decode_mbuf(struct mbuf *m, void *p_msg)
{
    capi_decode(((u_int8_t *)(m->m_data)) + 12, 
		m->m_len - 12, p_msg);
}

/*
//  Administrative messages:
//  ------------------------
*/

u_int16_t
capi_listen_req(struct i4b_controller *cntl, u_int32_t dwCIP)
{
    struct CAPI_LISTEN_REQ_DECODED listen_req;

    u_int16_t error;

    bzero(&listen_req, sizeof(listen_req));

    CAPI_INIT(CAPI_LISTEN_REQ, &listen_req);

    listen_req.dwCipMask1 = dwCIP;

    error = capi_send_decoded_std(cntl, CAPI_REQ(LISTEN), 
				  1 /* controller */, &listen_req);
    return error;
}

void
capi_listen_conf(struct i4b_controller *cntl, struct mbuf *m_in)
{
    struct CAPI_LISTEN_CONF listen_conf;

    CAPI_INIT(CAPI_LISTEN_CONF, &listen_conf);

    capi_decode_mbuf(m_in, &listen_conf);

    if (listen_conf.wInfo == 0) {

	/* We are now listening. */

	sc->sc_state = C_UP;
	ctrl_desc[sc->ctrl_unit].dl_est = DL_UP;

	i4b_l4_l12stat(sc->ctrl_unit, 1, 1);
	i4b_l4_l12stat(sc->ctrl_unit, 2, 1);

    } else {
	/* XXX sc->sc_state = C_DOWN ? XXX */
	printf("capi%d: can't listen, wInfo=0x%04x\n", 
	       sc->sc_unit, wInfo);
    }
    return;
}

u_int16_t
capi_info_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_INFO_RESP_DECODED info_resp;

    u_int16_t error;

    bzero(&info_resp, sizeof(info_resp));

    CAPI_INIT(CAPI_INFO_RESP, &info_resp);

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(INFO),
				    m_in, &info_resp);
    return error;
}

u_int16_t
capi_alert_req(capi_softc_t *sc, call_desc_t *cd)
{
    struct CAPI_ALERT_REQ_DECODED alert_req;

    u_int16_t error;

    bzero(&alert_req, sizeof(alert_req));

    CAPI_INIT(CAPI_ALERT_REQ, &alert_req);

    error = capi_send_decoded_std(cd->p_cntl, CAPI_REQ(ALERT), 
				  cd->capi_dwCid, &alert_req);
    return error;
}

void
capi_alert_conf(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_ALERT_CONF_DECODED alert_conf;

    CAPI_INIT(CAPI_ALERT_CONF, &alert_conf);

    capi_decode_mbuf(m_in, &alert_conf);

    if (alert_conf.wInfo) {
	printf("WARNING: %s: alert info=0x%04x\n", 
	       __FUNCTION__, alert_conf.wInfo);
    }
    return;
}

/*
//  Outgoing call setup:
//  --------------------
//
//             CAPI_CONNECT_REQ -->
//                              <-- CAPI_CONNECT_CONF
//                       (notify Layer 4)
//                              <-- CAPI_CONNECT_ACTIVE_IND
//     CAPI_CONNECT_ACTIVE_RESP -->
//          CAPI_CONNECT_B3_REQ -->
//                              <-- CAPI_CONNECT_B3_CONF
//                              <-- CAPI_CONNECT_B3_ACTIVE_IND
//  CAPI_CONNECT_B3_ACTIVE_RESP -->
//                       (notify Layer 4)
*/

u_int16_t
capi_connect_req(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_REQ_DECODED conn_req;
    struct CAPI_ADDITIONAL_INFO_DECODED add_info;
    struct CAPI_B_PROTOCOL_DECODED b_prot;

    u_int8_t dst_telno[TELNO_MAX+1];
    u_int8_t src_telno[TELNO_MAX+2];

    u_int8_t dst_subaddr[SUBADDR_MAX+1];
    u_int8_t src_subaddr[SUBADDR_MAX+1];

    u_int16_t error;

    static const u_int8_t sending_complete[2] = { 0x01, 0x00 };
    static const u_int8_t sending_not_complete[2] = { 0x00, 0x00 };

    bzero(&conn_req, sizeof(conn_req));
    bzero(&add_info, sizeof(add_info));
    bzero(&b_prot, sizeof(b_prot));

    CAPI_INIT(CAPI_CONNECT_REQ, &conn_req);
    CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);
    CAPI_INIT(CAPI_B_PROTOCOL, &b_prot);

    /* recursivly encode some structures */

    conn_req.b_protocol.ptr = &b_prot;
    conn_req.b_protocol_STRUCT = IE_STRUCT_DECODED;

    conn_req.add_info.ptr = &add_info;
    conn_req.add_info_STRUCT = IE_STRUCT_DECODED;

    /* initialize state and message number */

    cd->capi_state = B_CONNECT_CONF;
    cd->capi_msg_num = sc->sc_msgid++;

#warning "Use separate message number for connect request!"

    /* initialize message */

    switch (cd->channel_bprot) {
    case BPROT_NONE:
    case BPROT_RHDLC_DOV:
        /* Telephony */
	conn_req.wCIP = 0x0010;
	break;

    default:
        printf("WARNING: %s: Unknown protocol, %d!\n",
	       __FUNCTION__, cd->channel_bprot);

    case BPROT_RHDLC:
    case BPROT_NONE_VOD:
        /* Unrestricted digital */
        conn_req.wCIP = 0x0002;
	break;
    }

    /* copy destination telephone number */

    dst_telno[0] = 0x80;
    strlcpy(dst_telno+1, cd->dst_telno, sizeof(dst_telno)-1);

    conn_req.dst_telno.ptr = dst_telno;
    conn_req.dst_telno.len = strlen(dst_telno+1)+1;

    /* copy source telephone number */

    /* set Type Of Number */

    switch(cd->src[0].ton) {
    case TON_INTERNAT:
      src_telno[0] = 0x10;
      break;
    case TON_NATIONAL:
      src_telno[0] = 0x20;
      break;
    default:
      src_telno[0] = 0x00;
      break;
    }

    /* set Presentation Indicator */

    switch(cd->src[0].prs_ind) {
    case PRS_RESTRICT:
      src_telno[1] = (0x80|0x20);
      break;
    default:
      src_telno[1] = (0x80|0x00);
      break;
    }

    strlcpy(src_telno+2, cd->src[0].telno, sizeof(src_telno)-2);

    conn_req.src_telno.ptr = src_telno;
    conn_req.src_telno.len = strlen(src_telno+2)+2;

    /* copy destination subaddress */

    dst_subaddr[0] = 0x80;
    strlcpy(dst_subaddr+1, cd->dst_subaddr, sizeof(dst_subaddr)-1);

    conn_req.dst_subaddr.ptr = dst_subaddr;
    conn_req.dst_subaddr.len = strlen(dst_subaddr+1)+1;

    /* copy source subaddress */

    src_subaddr[0] = 0x80;
    strlcpy(src_subaddr+1, cd->src_subaddr, sizeof(src_subaddr)-1);

    conn_req.src_subaddr.ptr = src_subaddr;
    conn_req.src_subaddr.len = strlen(src_subaddr+1)+1;

#warning "XXX: always use transparent mode"

    b_prot.wB1_protocol = 
      ((cd->channel_bprot == BPROT_NONE) ||
       (cd->channel_bprot == BPROT_NONE_VOD)) ? 
      1 /* transparent */ : 
      0 /* HDLC */;

    b_prot.wB2_protocol = 1; /* transparent */

    add_info.keypad.ptr = cd->keypad;
    add_info.keypad.len = strlen(cd->keypad);

    add_info.useruser.ptr = cd->user_user;
    add_info.useruser.len = strlen(cd->user_user);

    if (cd->sending_complete) {
      add_info.sending_complete.ptr = &sending_complete;
      add_info.sending_complete.len = sizeof(sending_complete);
    } else {
      add_info.sending_complete.ptr = &sending_not_complete;
      add_info.sending_complete.len = sizeof(sending_not_complete);
    }

    /* transmit the message */

    error = capi_send_decoded_std(cd->p_cntl, CAPI_REQ(CONNECT), 
				  1 /* controller */, &conn_req);
    return error;
}

void
capi_connect_conf(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_CONF_DECODED conn_conf;

    bzero(&conn_conf, sizeof(conn_conf));

    CAPI_INIT(CAPI_CONNECT_CONF, &conn_conf);

    capi_decode_mbuf(m_in, &conn_conf);

#warning "set cd->capi_dwCid"

    if (conn_conf.wInfo == 0x0000) {

	sc->sc_bchan[bch].state = B_CONNECT_ACTIVE_IND;
	sc->sc_bchan[bch].ncci = PLCI;

	i4b_l4_proceeding_ind(cd);

    } else {

#warning "Update:"

	SET_CAUSE_TV(cd->cause_out, CAUSET_I4B, CAUSE_I4B_L1ERROR);

	cd_set_state(cd, ST_L3_U0);

	printf("capi%d: can't connect out, info=%04x\n", sc->sc_unit, Info);
    }
    return;
}

u_int16_t
capi_connect_active_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_ACTIVE_RESP_DECODED conn_act_resp;

    u_int16_t error;

    bzero(&conn_act_resp, sizeof(conn_act_resp));

    CAPI_INIT(CAPI_CONNECT_ACTIVE_RESP, &conn_act_resp);

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(CONNECT_ACTIVE), 
				    m_in, &conn_act_resp);

#error "Redesign:"

    if (!(cd->dir_incoming)) {

	capi_connect_b3_req(sc, cd);

    } else {
	sc->sc_bchan[bch].state = B_CONNECT_B3_IND;
    }

    return error;
}

u_int16_t
capi_connect_b3_req(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_B3_REQ_DECODED conn_b3_req;

    u_int16_t error;

    bzero(&conn_b3_req, sizeof(conn_b3_req));

    CAPI_INIT(CAPI_CONNECT_B3_REQ, &conn_b3_req);

    cd->capi_state = B_CONNECT_B3_CONF;

    error = capi_send_decoded_std(cd->p_cntl, CAPI_REQ(CONNECT_B3), 
				  cd->capi_dwCid, &conn_b3_req);
    return error;
}

void
capi_connect_b3_conf(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_B3_CONF conn_b3_conf;

    CAPI_INIT(CAPI_CONNECT_B3_CONF, &conn_b3_conf);

    capi_decode_mbuf(m_in, &conn_b3_conf);

    if (cd->capi_state == B_CONNECT_B3_CONF) {
        return;
    }

    if (conn_b3_conf.wInfo == 0x0000) {

	sc->sc_bchan[bch].ncci = NCCI;
	sc->sc_bchan[bch].state = B_CONNECT_B3_ACTIVE_IND;

    } else {

	SET_CAUSE_TV(cd->cause_in, CAUSET_I4B, CAUSE_I4B_OOO); /* XXX */
	i4b_l4_disconnect_ind(cd);
	freecd_by_cd(cd);

	ctrl_desc[sc->ctrl_unit].bch_state[bch] = BCH_ST_RSVD;

	printf("capi%d: can't connect_b3 out, info=%04x\n", sc->sc_unit, Info);

	capi_disconnect_req(sc, cd);
    }
    return;
}

u_int16_t
capi_connect_b3_active_resp(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_B3_ACTIVE_RESP_DECODED conn_b3_act_resp;

    u_int16_t error;

    bzero(&conn_b3_act_resp, sizeof(conn_b3_act_resp));

    CAPI_INIT(CAPI_CONNECT_B3_ACTIVE_RESP, &conn_b3_act_resp);

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(CONNECT_B3_ACTIVE),
				    m_in, &conn_b3_act_resp);
    return error;
}

void
capi_connect_b3_active_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_B3_ACTIVE_IND_DECODED conn_b3_act_ind;

    CAPI_INIT(CAPI_CONNECT_B3_ACTIVE_IND, &conn_b3_act_ind);

    capi_decode_mbuf(m_in, &conn_b3_act_ind);

    if (cd->capi_state != B_CONNECT_B3_ACTIVE_IND) {
#warning "XXX wrong to return here!"
        return;
    }

    capi_connect_b3_active_resp(cd, m_in);

    cd->capi_state = B_CONNECTED;

    i4b_l4_connect_active_ind(cd);
    return;
}

/*
//  Incoming call setup:
//  --------------------
//
//                              <-- CAPI_CONNECT_IND
//                       (consult Layer 4)
//            CAPI_CONNECT_RESP -->
//                              <-- CAPI_CONNECT_ACTIVE_IND
//     CAPI_CONNECT_ACTIVE_RESP -->
//                              <-- CAPI_CONNECT_B3_IND
//         CAPI_CONNECT_B3_RESP -->
//                              <-- CAPI_CONNECT_B3_ACTIVE_IND
//  CAPI_CONNECT_B3_ACTIVE_RESP -->
//                       (notify Layer 4)
*/

static void
get_ton_prs_src(const u_int8_t *ptr, u_int16_t len, 
		struct i4b_src_telno *src)
{
    /* get Type Of Number */

    src->ton = TON_OTHER;

    if(len) {
      switch(*ptr & 0x70) {
      case 0x10:
	src->ton = TON_INTERNAT;
	break;
      case 0x20:
	src->ton = TON_NATIONAL;
	break;
      }
      ptr++;
      len--;
    }

    /* get Presentation byte */

    src->prs_ind = PRS_ALLOWED;
    src->scr_ind = SCR_NONE;

    if(len) {
      switch(*ptr & 0x60) {
      case 0x20:
	src->prs_ind = PRS_RESTRICT;
	break;
      case 0x40:
	src->prs_ind = PRS_NNINTERW;
	break;
      case 0x60:
	src->prs_ind = PRS_RESERVED;
	break;
      }
      switch(*ptr & 0x03) {
      case 0x00:
	src->scr_ind = SCR_USR_NOSC;
	break;
      case 0x01:
	src->scr_ind = SCR_USR_PASS;
	break;
      case 0x02:
	src->scr_ind = SCR_USR_FAIL;
	break;
      case 0x03:
	src->scr_ind = SCR_NET;
	break;
      }
      ptr++;
      len--;
    }
    return;
}

static void
get_multi_1(const u_int8_t *src_ptr, u_int16_t src_len,
	    u_int8_t *dst_ptr, u_int16_t dst_len, u_int16_t skip)
{
    if (src_len < skip) {
        src_len = 0;
    } else {
        src_len -= skip;
	src_ptr += skip;
    }

    if (src_len > dst_len) {
        src_len = dst_len;
    }

    bcopy(src_ptr, dst_ptr, src_len);
    dst_ptr[src_len] = '\0';

    return;
}

void
capi_connect_ind(struct i4b_controller *cntl, struct mbuf *m_in)
{
    struct CAPI_CONNECT_IND_DECODED conn_ind;
    struct CAPI_ADDITIONAL_INFO_DECODED add_info;
    struct CAPI_HEADER_ENCODED *hdr = (void *)(m_in->m_data);
    struct call_desc *cd;

    CAPI_INIT(CAPI_CONNECT_IND, &conn_ind);
    CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);

    /* perform recursive decoding */

    conn_ind.add_info.ptr = &add_info;
    conn_ind.add_info_STRUCT = IE_STRUCT_DECODED;

    capi_decode_mbuf(m_in, &conn_ind);

    cd = N_ALLOCATE_CD(cntl, ((void *)1), 0, 0, NULL);

    if (cd == NULL) {
       /* just let it time out */
       return;
    }

    cd_allocate_channel(cd);

    if (cd->channel_allocated == 0) {
        cd_set_state(cd, ST_L3_U0);
	return;
    }

    cd->capi_dwCid   = hdr->dwCid;
    cd->capi_wMsgNum = hdr->wNum;

    switch (conn_ind.wCIP) {
    default:
        NDBGL4(L4_CAPIDBG, "capi%d: cdid=%d, unknown CIP = %d", 
	       CDID2CONTROLLER(cd->cdid), cd->cdid, conn_ind.wCIP);

    case 0x0010:
    case 0x0001: 
        cd->channel_bprot = BPROT_NONE; 
	break;
    case 0x0002: 
        cd->channel_bprot = BPROT_RHDLC; 
	break;
    }

    /* get destination telephone number */

    get_multi_1(conn_ind.dst_telno.ptr,
		conn_ind.dst_telno.len,
		cd->dst_telno,
		sizeof(cd->dst_telno)-1, 1);

    /* get source telephone number */

    get_ton_prs_src(conn_ind.src_telno.ptr,
		    conn_ind.src_telno.len, &(cd->src[0]));

    get_multi_1(conn_ind.src_telno.ptr,
		conn_ind.src_telno.len,
		cd->src[0].telno,
		sizeof(cd->src[0].telno)-1, 2);

    /* get second source telephone number */

    get_ton_prs_src(conn_ind.src_telno_2.ptr,
		    conn_ind.src_telno_2.len, &(cd->src[1]));

    get_multi_1(conn_ind.src_telno_2.ptr,
		conn_ind.src_telno_2.len,
		cd->src[1].telno,
		sizeof(cd->src[1].telno)-1, 2);

    /* get destination subaddress number */

    get_multi_1(conn_ind.dst_subaddr.ptr,
		conn_ind.dst_subaddr.len,
		cd->dst_subaddr,
		sizeof(cd->dst_subaddr)-1, 1);

    /* get source subaddress number */

    get_multi_1(conn_ind.src_subaddr.ptr,
		conn_ind.src_subaddr.len,
		cd->src_subaddr,
		sizeof(cd->src_subaddr)-1, 1);

    /* get keypad string */

    get_multi_1(add_info.keypad.ptr,
		add_info.keypad.len,
		cd->keypad,
		sizeof(cd->keypad)-1, 0);

    /* get user-user string */

    get_multi_1(add_info.useruser.ptr,
		add_info.useruser.len,
		cd->user_user,
		sizeof(cd->user_user)-1, 0);

    /* get sending complete */

    if (add_info.sending_complete.len >= 2) {
        u_int8_t *ptr = add_info.sending_complete.ptr;

	if((ptr[0] == 0x01) &&
	   (ptr[1] == 0x00)) {

	   cd->sending_complete = 1;
	}
    }

    i4b_l4_connect_ind(cd);

    return;
}

u_int16_t
capi_connect_resp(struct call_desc *cd, struct mbuf *m_in, u_int16_t wReject)
{
    struct CAPI_CONNECT_RESP_DECODED conn_resp;
    struct CAPI_B_PROTOCOL_DECODED b_prot;
    struct ADDITIONAL_INFO add_info;

    u_int8_t dst_telno[TELNO_MAX+1];

    u_int16_t error;

    bzero(&conn_resp, sizeof(conn_resp));
    bzero(&b_prot, sizeof(b_prot));
    bzero(&add_info, sizeof(add_info));

    CAPI_INIT(CAPI_CONNECT_RESP, &conn_resp);
    CAPI_INIT(CAPI_B_PROTOCOL, &b_prot);
    CAPI_INIT(CAPI_ADDITIONAL_INFO, &add_info);

    /* recursivly encode some structures */

    conn_req.b_protocol.ptr = &b_prot;
    conn_req.b_protocol_STRUCT = IE_STRUCT_DECODED;

    conn_req.add_info.ptr = &add_info;
    conn_req.add_info_STRUCT = IE_STRUCT_DECODED;

    /**/

    XXX msgid = (u_int16_t) cd->event;
    XXX PLCI = (u_int32_t) cd->Q931state;

#warning "XXX: always use transparent mode"

    b_prot.wB1_protocol = 
      ((cd->channel_bprot == BPROT_NONE) ||
       (cd->channel_bprot == BPROT_NONE_VOD)) ? 
      1 /* transparent */ : 
      0 /* HDLC */;

    b_prot.wB2_protocol = 1; /* transparent */

    /* copy destination telephone number */

    dst_telno[0] = 0x80;
    strlcpy(dst_telno+1, cd->dst_telno, sizeof(dst_telno)-1);

    conn_resp.dst_telno.ptr = dst_telno;
    conn_resp.dst_telno.len = strlen(dst_telno+1)+1;

    /* set reject variable */

    conn_resp.wReject = wReject;

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(CONNECT), 
				    m_in, &conn_req);

#if 0
 Move this elsewhere !

    switch (cd->response) {
    case SETUP_RESP_ACCEPT:
      cd->capi_state = B_CONNECT_ACTIVE_IND;
	ctrl_desc[sc->ctrl_unit].bch_state[cd->channelid] = BCH_ST_USED;
	msg = capimsg_setu16(msg, 0); /* Accept the call */
	break;

    case SETUP_RESP_REJECT:
	sc->sc_bchan[cd->channelid].state = B_FREE;
	ctrl_desc[sc->ctrl_unit].bch_state[cd->channelid] = BCH_ST_FREE;
	msg = capimsg_setu16(msg, 2); /* Reject, normal call clearing */
	break;

    case SETUP_RESP_DNTCRE:
	sc->sc_bchan[cd->channelid].state = B_FREE;
	ctrl_desc[sc->ctrl_unit].bch_state[cd->channelid] = BCH_ST_FREE;
	if (sc->sc_nbch == 30) {
	    /* With PRI, we can't really ignore calls  -- normal clearing */
	    msg = capimsg_setu16(msg, (0x3480|CAUSE_Q850_NCCLR));
	} else {
	    msg = capimsg_setu16(msg, 1); /* Ignore */
	}
	break;

    default:
	sc->sc_bchan[cd->channelid].state = B_FREE;
	ctrl_desc[sc->ctrl_unit].bch_state[cd->channelid] = BCH_ST_FREE;
	msg = capimsg_setu16(msg, (0x3480|CAUSE_Q850_CALLREJ));
    }
#endif
    return error;
}

u_int16_t
capi_connect_b3_resp(struct call_desc *cd, struct mbuf *m_in, 
		     u_int16_t wReject)
{
    struct CAPI_CONNECT_B3_RESP_DECODED conn_b3_resp;

    u_int16_t error;

    bzero(&conn_b3_resp, sizeof(conn_b3_resp));

    CAPI_INIT(CAPI_CONNECT_B3_RESP, &conn_b3_resp);

    conn_b3_resp.wReject = wReject;

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(CAPI_CONNECT_B3), 
				    m_in, &conn_b3_resp);
    return error;
}

u_int16_t
capi_connect_b3_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_CONNECT_B3_IND_DECODED conn_b3_ind;

    u_int16_t error;

    CAPI_INIT(CAPI_CONNECT_B3_IND, &conn_b3_ind);

    capi_decode_mbuf(m_in, &conn_b3_ind);

    error = capi_connect_b3_resp(cd, m_in, 0x0000 /* Accept */);

    return error;
}

/*
//  Data transfer:
//  --------------
*/

u_int16_t
capi_data_b3_req(capi_softc_t *sc, int chan, struct mbuf *m_b3)
{
    struct CAPI_DATA_B3_REQ_DECODED data_b3_req;

    u_int16_t error;

    bzero(&data_b3_req, sizeof(data_b3_req));

    CAPI_INIT(CAPI_DATA_B3_REQ, &data_b3_req);

    if(sizeof(void *) <= 4)
      data_b3_req.dwPtr_1 = (u_int32_t)(m_b3->m_data);
    else
      data_b3_req.dwPtr_2 = (u_int64_t)(m_b3->m_data);

    data_b3_req.wLen = (m_b3->m_len);

    cd->capi_busy = 1;

    error = capi_send_decoded_b3(cd->p_cntl, CAPI_REQ(DATA_B3), 
				 cd->capi_dwCid, m_b3, &data_b3_req);
    return error;
}

void
capi_data_b3_conf(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_DATA_B3_CONF_DECODED data_b3_conf;

    CAPI_INIT(CAPI_DATA_B3_CONF, &data_b3_conf);

    capi_decode_mbuf(m_in, &data_b3_conf);

    if (data_b3_conf.wInfo == 0x0000) {
	sc->sc_bchan[handle].busy = 0;
	capi_start_tx(sc, handle);

    } else {
	printf("WARNING: %s[cdid=%d], wInfo=0x%04x\n",
	       __FUNCTION__, cd->cdid, data_b3_conf.wInfo);
    }
    return;
}

u_int16_t
capi_data_b3_resp(struct call_desc *cd, struct mbuf *m_in, u_int16_t wHandle)
{
    struct CAPI_DATA_B3_RESP_DECODED data_b3_resp;

    u_int16_t error;

    CAPI_INIT(CAPI_DATA_B3_RESP, &data_b3_resp);

    bzero(&data_b3_resp, sizeof(data_b3_resp));

    data_b3_resp.wHandle = wHandle;

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(DATA_B3), 
				    m_in, &data_b3_resp);
    return error;
}

u_int16_t
capi_data_b3_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_DATA_B3_IND_DECODED data_b3_ind;

    u_int16_t error;

    CAPI_INIT(CAPI_DATA_B3_IND, &data_b3_ind);

    capi_decode_mbuf(m_in, &data_b3_ind);

#warning "FIXME:"

    if (sc->sc_bchan[bch].bprot == BPROT_RHDLC) {
	    /* HDLC drivers use rx_mbuf */

	    sc->sc_bchan[bch].in_mbuf = m_in->m_next;
	    sc->sc_bchan[bch].rxcount += m_in->m_next->m_len;
	    m_in->m_next = NULL; /* driver frees */

	    (*sc->sc_bchan[bch].capi_drvr_linktab->bch_rx_data_ready)(
		sc->sc_bchan[bch].capi_drvr_linktab->unit);

    } else {
	    /* Telephony drivers use rx_queue */

	    if (!_IF_QFULL(&sc->sc_bchan[bch].rx_queue)) {
		_IF_ENQUEUE(&sc->sc_bchan[bch].rx_queue, m_in->m_next);
		sc->sc_bchan[bch].rxcount += m_in->m_next->m_len;
		m_in->m_next = NULL; /* driver frees */
	    }

	    (*sc->sc_bchan[bch].capi_drvr_linktab->bch_rx_data_ready)(
		sc->sc_bchan[bch].capi_drvr_linktab->unit);
    }

    error = capi_data_b3_resp(cd, m_in, data_b3_ind.wHandle);

    return error;
}

/*
//  Connection teardown:
//  --------------------
*/

u_int16_t
capi_disconnect_req(struct call_desc *cd, struct mbuf *m_in)
{
    u_int16_t error;

    struct CAPI_DISCONNECT_REQ_DECODED disc_req;

    bzero(&disc_req, sizeof(disc_req));

    CAPI_INIT(CAPI_DISCONNECT_REQ, &disc_req);

    error = capi_send_decoded_std(cd->p_cntl, CAPI_REQ(DISCONNECT), 
				  cd->capi_dwCid, &disc_req);

    cd_set_state(cd, ST_L3_U0);

    return error;
}

void
capi_disconnect_conf(struct call_desc *cd, struct mbuf *m_in)
{
    return;
}

u_int16_t
capi_disconnect_b3_resp(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_DISCONNECT_B3_RESP_DECODED disc_b3_resp;

    u_int16_t error;

    bzero(&disc_b3_resp, sizeof(disc_b3_resp));

    CAPI_INIT(CAPI_DISCONNECT_B3_RESP, &disc_b3_resp);

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(DISCONNECT_B3), 
				    m_in, &disc_b3_resp);
    return error;
}

u_int16_t
capi_disconnect_b3_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_DISCONNECT_B3_IND_DECODED disc_b3_ind;

    u_int16_t error;

    CAPI_INIT(CAPI_DISCONNECT_B3_IND, &disc_b3_ind);

    capi_decode_mbuf(m_in, &disc_b3_ind);

    /* XXX update bchan state? XXX */

    error = capi_disconnect_b3_resp(cd, m_in);

    return error;
}

u_int16_t
capi_disconnect_resp(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_DISCONNECT_RESP_DECODED disc_resp;

    u_int16_t error;

    bzero(&disc_resp, sizeof(disc_resp));

    CAPI_INIT(CAPI_DISCONNECT_RESP, &disc_resp);

    error = capi_send_decoded_reply(cd->p_cntl, CAPI_RESP(DISCONNECT), 
				    m_in, &disc_resp);
    return error;
}

u_int16_t
capi_disconnect_ind(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_DISCONNECT_IND_DECODED disc_ind;

    u_int16_t error;

    CAPI_INIT(CAPI_DISCONNECT_IND, &disc_ind);

    capi_decode(data, len, &disc_ind);

    if ((disc_ind.wReason & 0xff00) == 0x3400) {
      SET_CAUSE_TV(cd->cause_in, CAUSET_Q850, (disc_ind.wReason & 0x7f));
    } else {
      SET_CAUSE_TV(cd->cause_in, CAUSET_I4B, CAUSE_I4B_NORMAL);
    }

    error = capi_disconnect_resp(cd, m_in);

    cd_set_state(cd, ST_L3_U0);

    return error;
}

/*
//  Protocol selection
//  ------------------
*/
void
capi_select_b3_protocol_conf(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_SELECT_B_PROTOCOL_CONF_DECODED prot_conf;

    CAPI_INIT(CAPI_SELECT_B_PROTOCOL_CONF, &prot_conf);

    capi_decode_mbuf(m_in, &prot_conf);

    if (prot_conf.wInfo == 0x0000) {


    } else {

        printf("WARNING: %s[cdid=%d], wInfo=0x%04x\n",
	       __FUNCTION__, cd->cdid, prot_conf.wInfo);
    }
    return;
}

u_int16_t
capi_select_b3_protocol_req(struct call_desc *cd, struct mbuf *m_in)
{
    struct CAPI_SELECT_B_PROTOCOL_REQ_DECODED prot_req;
    struct CAPI_B_PROTOCOL_DECODED b_prot;

    u_int16_t error;

    bzero(&prot_req, sizeof(prot_req));
    bzero(&b_prot, sizeof(b_prot));

    CAPI_INIT(CAPI_SELECT_B_PROTOCOL_REQ, &prot_req);
    CAPI_INIT(CAPI_B_PROTOCOL, &b_prot);

    /* recursivly encode some structures */

    prot_req.b_protocol.ptr = &b_prot;
    prot_req.b_protocol_STRUCT = IE_STRUCT_DECODED;

    b_prot.wB1_protocol = 1; /* transparent */
    b_prot.wB2_protocol = 1; /* transparent */
    b_prot.wB3_protocol = 0; /* transparent */

    error = capi_send_decoded_std(cd->p_cntl, CAPI_REQ(SELECT_B_PROTOCOL),
				  cd->dwCid, &prot_req);
    return error;
}
