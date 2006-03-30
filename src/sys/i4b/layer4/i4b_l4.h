/*-
 * Copyright (c) 1997, 2001 Hellmuth Michaelis. All rights reserved.
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
 *	i4b_l4.h - kernel interface to userland header file
 *	---------------------------------------------------
 *
 * $FreeBSD: src/sys/i4b/layer4/i4b_l4.h,v 1.8 2001/10/18 11:53:49 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_L4_H_
#define _I4B_L4_H_

#define i4b_release_drivers_by_appl_interface(args...) \
  NDBGL4(L4_MSG, "implement i4b_release_drivers_by_appl_interface(" #args ");")

/* application interface types */

#define I4B_AI_BROADCAST  0 /* all interfaces (value is hardcoded) */
#define I4B_AI_I4B        1
#define I4B_AI_CAPI       2
#define I4B_AI_UNUSED     3
#define I4B_AI_MAX        4 /* value is exclusive */

/* prototypes */

struct call_desc;
struct i4b_controller;
struct i4b_ai_softc;
struct isdn_download_request;
struct i4b_line_interconnect;

extern void i4b_version_request(msg_vr_req_t *mvr);

extern int i4b_controller_download(struct i4b_controller *cntl, 
				   struct isdn_download_request *req);

extern void i4b_ai_putqueue(struct i4b_ai_softc *sc, 
			    u_int8_t sc_complement, struct mbuf *m);

extern void i4b_ai_connect_ind(struct call_desc *cd, 
			       struct i4b_ai_softc *ai_ptr);

extern int i4b_link_bchandrvr(call_desc_t *cd, int activate);

extern void i4b_l4_alert_ind(struct call_desc *cd);
extern void i4b_l4_charging_ind(struct call_desc *cd);
extern void i4b_l4_connect_active_ind(struct call_desc *cd);
extern void i4b_l4_connect_ind(struct call_desc *cd);
extern void i4b_l4_information_ind(call_desc_t *cd);
extern void i4b_l4_pre_disconnect_ind(struct call_desc *cd);
extern void i4b_l4_disconnect_ind(struct call_desc *cd, u_int8_t complement);
extern void i4b_l4_idle_timeout_ind(struct call_desc *cd);
extern void i4b_l4_proceeding_ind(struct call_desc *cd, 
				  u_int8_t sending_complete);
extern void i4b_l4_retrieve_ind(call_desc_t *cd);
extern void i4b_l4_hold_ind(call_desc_t *cd);

extern void i4b_l4_negcomplete_ind(struct call_desc *cd);

extern void i4b_l4_accounting(int, int, int, int, int, int, int, int, int);
extern void i4b_l4_setup_timeout(struct call_desc *cd);

extern void i4b_l3_information_req(struct call_desc *cd, u_int8_t *ptr, u_int16_t len);

extern void i4b_l4_dialout(int driver, int driver_unit );
extern void i4b_l4_dialoutnumber(int driver, int driver_unit, int cmdlen, 
				 char *cmd);
extern void i4b_l4_keypad(int driver, int driver_unit, int cmdlen, char *cmd);
extern void i4b_l4_drvrdisc(int driver, int driver_unit);
extern void i4b_l4_drvranswer(int driver, int driver_unit);
extern void i4b_l4_drvrreject(int driver, int driver_unit);

extern void i4b_l4_ifstate_changed(struct call_desc *cd, int new_state );
extern void i4b_l4_packet_ind(int, int, int, struct mbuf *pkt);
extern void i4b_l4_l12stat(int controller, int layer, int state);
extern void i4b_l4_teiasg(int controller, int tei);

/* prototypes from i4b_l4mgmt.c */

extern struct call_desc * i4b_allocate_cd(struct i4b_controller *cntl);
extern struct call_desc * cd_by_cdid(struct i4b_controller *cntl, 
				     unsigned int cdid);
extern struct call_desc * cd_by_unitcr(struct i4b_controller *cntl, 
				       void *pipe, void *pipe_adapter, 
				       u_int cr);

extern void i4b_free_cd(struct call_desc *cd);
extern void cd_allocate_channel(struct call_desc *cd);
extern void cd_free_channel(struct call_desc *cd);

extern void cd_set_appl_interface(struct call_desc *cd, 
				  u_int8_t appl_interface_type,
				  void *appl_interface_ptr);

extern void i4b_disconnect_by_appl_interface(u_int8_t ai_type, void *ai_ptr);
extern void i4b_update_all_d_channels(int);

extern u_int8_t i4b_make_q850_cause(cause_t cause);

extern struct i4b_line_interconnect * 
i4b_slot_li_alloc(cdid_t cdid_src, cdid_t cdid_dst);

extern void
i4b_slot_li_free(struct i4b_line_interconnect *li);

/* prototypes from i4b_capidrv.c */

struct capi_ai_softc;

extern void capi_ai_info_ind(struct call_desc *cd, u_int8_t complement, 
			     u_int16_t wInfoNumber, void *ptr, u_int16_t len);
extern void capi_ai_connect_ind(struct call_desc *cd);
extern void capi_ai_connect_active_ind(struct call_desc *cd);
extern void capi_ai_disconnect_ind(struct call_desc *cd, u_int8_t complement);

/* other prototypes */

typedef void (response_to_user_t)(msg_response_to_user_t *);
typedef struct fifo_translator * (setup_ft_t)(struct i4b_controller *cntl,
	 struct fifo_translator *f, struct i4b_protocol *protocol, 
	 u_int32_t driver_type, u_int32_t driver_unit, struct call_desc *cd);

extern     response_to_user_t
       ibc_response_to_user,
       ing_response_to_user,
       ipr_response_to_user,
  i4bisppp_response_to_user,
      rbch_response_to_user,
       tel_response_to_user,
  ihfc_dev_response_to_user,
      dss1_response_to_user,
     diehl_response_to_user,
   tina_dd_response_to_user,
    amv_b1_response_to_user;

extern       setup_ft_t
         ibc_setup_ft,
         ing_setup_ft,
         ipr_setup_ft,
    i4bisppp_setup_ft,
        rbch_setup_ft,
         tel_setup_ft,
    tel_dial_setup_ft,
    ihfc_dev_setup_ft,
        dss1_setup_ft,
        capi_setup_ft,
 capi_bridge_setup_ft,
       diehl_setup_ft,
     tina_dd_setup_ft,
      amv_b1_setup_ft;

#endif /* _I4B_L4_H_ */
