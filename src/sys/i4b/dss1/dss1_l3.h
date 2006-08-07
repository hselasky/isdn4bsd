/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2004 Hans Petter Selasky. All rights reserved.
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
 *      dss1_l3.h - layer 3
 *      -------------------
 * 
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _DSS1_L3_H_
#define _DSS1_L3_H_

#define STATUS_ENQUIRY_TIMEOUT 8 /* timeouts ~ 8*8 = 64 seconds */

#define T303VAL	(hz* 4) /* tx_setup	   : 4 seconds timeout		*/
#define T305VAL	(hz*30) /* tx_disconnect   : 30 seconds timeout		*/
#define T308VAL	(hz* 4) /* tx_release	   : 4 seconds timeout		*/
#define T309VAL	(hz*90) /* DL re-establish : 90 seconds timeout		*/
#define T310VAL	(hz*60) /* call proceeding : 
			 * waiting for     :
			 * CONNECT         : 30-120 seconds timeout
			 */
#define T313VAL	(hz* 4) /* tx_connect      : 4 seconds timeout		*/
#define T400DEF	(hz*10) /*                 : 10 seconds timeout		*/

/*---------------------------------------------------------------------------*
 * L3_STATES
 *
 * update cd_set_state, if adding ST_XXX_TOs
 *
 * states must be sorted, smallest value first
 *---------------------------------------------------------------------------*/
#define     L3_STATES_Q931_CONV(state,value,timeout_delay,timeout_state,desc,q931_conv) q931_conv
#define          L3_STATES_DESC(state,value,timeout_delay,timeout_state,desc,q931_conv) #state " - " desc
#define L3_STATES_TIMEOUT_STATE(state,value,timeout_delay,timeout_state,desc,q931_conv) timeout_state
#define L3_STATES_TIMEOUT_DELAY(state,value,timeout_delay,timeout_state,desc,q931_conv) timeout_delay
#define L3_STATES(m)/*								\
m(----------------,,--------,-------------,-------------------------,------)-*	\
m(                ,,timeout , timeout     ,                         , q931 ) *	\
m( state          ,,delay   , state       , desc                    , conv.) *	\
m(----------------,,--------,-------------,-------------------------,------)-*/	\
m( ST_L3_U0       ,, 0/*hz*/, ST_L3_U0    , "Null"                  , 0x00  )	\
										\
m( ST_L3_OUTGOING ,, 8/*hz*/, ST_L3_U0    , "Outgoing initialized"  , 0x00  )	\
m( ST_L3_U1       ,, 8/*hz*/, ST_L3_U0    , "Outgoing setup (U1)"   , 0x01  )	\
m( ST_L3_U2       ,,16/*hz*/, ST_L3_U0    , "Outgoing setup (U2)"   , 0x02  )	\
m( ST_L3_U2_ACK   ,,16/*hz*/, ST_L3_U0    , "Outgoing setup (U2)"   , 0x02  )	\
m( ST_L3_U3       ,, 8/*hz*/, ST_L3_U3_TO , "Outgoing proceeding"   , 0x03  )	\
m( ST_L3_U3_TO    ,, 4/*hz*/, ST_L3_U0    , "Outgoing proceeding"   , 0x03  )	\
										\
m( ST_L3_U4       ,, 8/*hz*/, ST_L3_U4_TO , "Outgoing delivered"    , 0x04  )	\
m( ST_L3_U4_TO    ,, 4/*hz*/, ST_L3_U0    , "Outgoing delivered"    , 0x04  )	\
										\
m( ST_L3_INCOMING ,, 8/*hz*/, ST_L3_U0    , "Incoming initialized"  , 0x00  )	\
m( ST_L3_IN_ACK   ,,16/*hz*/, ST_L3_U0    , "Incoming initialized"  , 0x19  )	\
m( ST_L3_U6       ,, 8/*hz*/, ST_L3_U6_TO , "Incoming present"      , 0x06  )	\
m( ST_L3_U6_TO    ,, 4/*hz*/, ST_L3_U0    , "Incoming present"      , 0x06  )	\
m( ST_L3_U7       ,, 8/*hz*/, ST_L3_U7_TO , "Incoming alerted"      , 0x07  )	\
m( ST_L3_U7_TO    ,, 4/*hz*/, ST_L3_U0    , "Incoming alerted"      , 0x07  )	\
m( ST_L3_U8       ,, 4/*hz*/, ST_L3_U0    , "Incoming connecting"   , 0x08  )	\
										\
m( ST_L3_UA       ,, 8/*hz*/, ST_L3_UA_TO , "Active"                , 0x0A  )	\
m( ST_L3_UA_TO    ,, 4/*hz*/, ST_L3_U0    , "Active"                , 0x0A  )	\
										\
m( ST_L3_UC       ,, 8/*hz*/, ST_L3_UC_TO , "Disconnected"          , 0x0C  )	\
m( ST_L3_UC_TO    ,, 4/*hz*/, ST_L3_U0    , "Disconnected"          , 0x0C  )	\
/**/

/*---------------------------------------------------------------------------*
 * L3_EVENTS
 *
 * the L3_EVENTS are sorted in the order IN, NEUTRAL, OUT, which is
 * hardcoded, to help filtering away invalid events
 *
 * LOCAL_INCOMING events overlap LOCAL_OUTGOING events !
 *---------------------------------------------------------------------------*/
#define L3_EVENTS_DESC(enum,value,desc) #enum " - " desc
#define L3_EVENTS(macro)			\
        L3_EVENTS_LOCAL_INCOMING(macro)		\
        L3_EVENTS_NEUTRAL(macro)		\
        L3_EVENTS_LOCAL_OUTGOING(macro)		\
/**/

#define L3_EVENT_IS_LOCAL_INCOMING(event)	\
((event) < (L3_EVENTS_LOCAL_INCOMING(1+ NO)	\
	    L3_EVENTS_NEUTRAL(1+ NO) 0))	\
/**/

#define L3_EVENT_IS_LOCAL_OUTGOING(event)	\
((event) >= (L3_EVENTS_LOCAL_INCOMING(1+ NO)	\
	     0))				\
/**/

/* NOTE: enum ``EV_L3_ILL'' must have the
 * memory-default-value of zero
 */
#define L3_EVENTS_LOCAL_INCOMING(m) \
m( EV_L3_ILL         ,, "illegal event")\
\
m( EV_L3_PROGRESSRQ  ,, "L4 PROGRESS REQUEST")\
m( EV_L3_ALERTRQ     ,, "L4 ALERT REQUEST")\
m( EV_L3_PROCEEDINGRQ,, "L4 CALL PROCEEDING REQUEST")\
m( EV_L3_SETACRS     ,, "L4 ACCEPT RESPONSE")\
m( EV_L3_SETRJRS     ,, "L4 REJECT RESPONSE")\
m( EV_L3_DEFLECTRQ   ,, "L4 DEFLECT REQUEST")\
m( EV_L3_MCIDRQ      ,, "L4 MCID REQUEST")\
m( EV_L3_SETUP       ,, "received SETUP")\
m( EV_L3_CONACK      ,, "received CONNECT ACKNOWLEDGE")\
/**/

#define L3_EVENTS_NEUTRAL(m)\
m( EV_L3_STATUS      ,, "received STATUS")\
m( EV_L3_RELEASE     ,, "RELEASE")\
m( EV_L3_STATENQ     ,, "received STATUS ENQUIRY")\
m( EV_L3_INFO        ,, "received INFORMATION")\
m( EV_L3_FACILITY    ,, "received FACILITY")\
m( EV_L3_RESTART_IND ,, "received RESTART INDICATION")\
m( EV_L3_RESTART_ACK ,, "received RESTART ACKNOWLEDGE")\
m( EV_L3_HOLD_IND    ,, "received HOLD INDICATION")\
m( EV_L3_RETRIEVE_IND,, "received RETRIEVE INDICATION")\
m( EV_L3_RETRIEVE_ACK,, "received RETRIEVE ACKNOWLEDGE")\
m( EV_L3_DISCONNECT  ,, "received DISCONNECT INDICATION")\
m( EV_L3_INFORQ      ,, "L4 INFORMATION REQUEST")\
m( EV_L3_HOLD_REQ    ,, "L4 HOLD REQUEST")\
m( EV_L3_RETRIEVE_REQ,, "L4 RETRIEVE REQUEST")\
/**/

#define L3_EVENTS_LOCAL_OUTGOING(m)\
m( EV_L3_SETUPRQ     ,, "L4 SETUP REQUEST")\
m( EV_L3_SETUPAK     ,, "received SETUP ACKNOWLEDGE")\
m( EV_L3_CALLPRC     ,, "received CALL PROCEEDING")\
m( EV_L3_ALERT       ,, "received ALERT")\
m( EV_L3_CONNECT     ,, "received CONNECT")\
m( EV_L3_PROGIND     ,, "received PROGRESS INDICATION")\
/**/

/*---------------------------------------------------------------------------*
 * make enums
 *---------------------------------------------------------------------------*/
MAKE_ENUM(L3_STATES,
	N_L3_STATES);
MAKE_ENUM(L3_EVENTS,
	N_L3_EVENTS);

#ifdef _KERNEL
struct call_desc;
struct dss1_buffer;
extern void dss1_facility_decode(call_desc_t *cd, struct dss1_buffer *buf);
#endif

/* extension bit */
#define EXT_LAST		0x80	/* last octett */

/**/
#define CODESET_MASK		0x07
#define CRLENGTH_MASK		0x0f

/* for outgoing causes */
#define IEI_CAUSE_LEN		2
#define CAUSE_STD_LOC_OUT	0x80	/* std = CCITT, loc = user */
#define CAUSE_STD_LOC_PUBLIC    0x82    /* std = CCITT, loc = 
					 *   public network serving local user
					 */

#define IEI_BEARERCAP_LEN	2	/* 2 octetts length */

/* BC: information xfer capability */
#define IT_CAP_SPEECH		(0x00|EXT_LAST)	
#define IT_CAP_UNR_DIG_INFO	(0x08|EXT_LAST)
#define IT_CAP_R_DIG_INFO	(0x09|EXT_LAST)
#define IT_CAP_AUDIO_3100Hz	(0x10|EXT_LAST)
#define IT_CAP_UNR_DIG_TONES    (0x11|EXT_LAST)
#define IT_CAP_VIDEO		(0x18|EXT_LAST)

/* BC: information xfer rate	*/
#define	IT_RATE_64K		0x90	
#define	IT_RATE_56K		0x8f

/* layer1 protocol G.711 A-law	*/
#define	IT_UL1_G711U		0xa2
#define	IT_UL1_G711A		0xa3

#define IE_CHAN_ID_NO		0x00	/* no channel			*/
#define IE_CHAN_ID_B1		0x01	/* B1 channel			*/
#define IE_CHAN_ID_B2		0x02	/* B2 channel			*/
#define IE_CHAN_ID_ANY		0x03	/* ANY channel			*/

#define NUMBER_TYPE_LEN		0x01	/* without number string !	*/

#define	NUMBER_TYPE_NSAP	0x80	/* subaddr: type=NSAP		*/
#define	NUMBER_TYPE_PLAN       	0x81    /* type of numbering plan	*/

#define IEI_CALLSTATE_LEN	1	/* length of callstate		*/

#define IEI_USERUSER_LEN	0	/* without message string !	*/

/* protocol discriminators */
#define PD_Q931		0x08	/* Q.931/I.451 */

/* variable length information element identifiers */
#define Q931_INFORMATION_ELEMENTS_TABLE_0(enum,value,desc,isdndecode_func) { enum, desc, isdndecode_func }
#define Q931_INFORMATION_ELEMENTS(m)\
m(IEI_CONADDR     ,=0x0c, "connected address",              NULL)\
m(IEI_TERMCAP     ,=0x24, "terminal capabilities",          NULL)\
m(IEI_KEYPECHO    ,=0x30, "keypad echo",                    NULL)\
m(IEI_SWITCHHOOK  ,=0x36, "switchhook",                     NULL)\
\
/* Q.931 variable length information element identifiers */\
\
m(IEI_SEGMMSG     ,=0x00, "segmented message",              NULL)\
m(IEI_BEARERCAP   ,=0x04, "bearer capability",              &f_bc)\
m(IEI_CAUSE       ,=0x08, "cause",                          &f_cause)\
m(IEI_CALLID      ,=0x10, "call identity",                  NULL)\
m(IEI_CALLSTATE   ,=0x14, "call state",                     &f_cstat)\
m(IEI_CHANNELID   ,=0x18, "channel identification",         &f_chid)\
m(IEI_PROGRESSI   ,=0x1e, "progress indicator",             &f_progi)\
m(IEI_NETSPCFAC   ,=0x20, "network specific facilities",    NULL)\
m(IEI_NOTIFIND    ,=0x27, "notification indicator",         &f_notify)\
m(IEI_DISPLAY     ,=0x28, "display",                        &f_displ)\
m(IEI_DATETIME    ,=0x29, "date/time",                      &f_date)\
m(IEI_KEYPAD      ,=0x2c, "keypad facility",                &f_keypad)\
m(IEI_SIGNAL      ,=0x34, "signal",                         NULL)\
m(IEI_INFRATE     ,=0x40, "information rate",               NULL)\
m(IEI_ETETDEL     ,=0x42, "end-to-end transit delay",       NULL)\
m(IEI_TDELSELIND  ,=0x43, "transit delay selection and indication", NULL)\
m(IEI_PLBPARMS    ,=0x44, "packet layer binary parameters", NULL)\
m(IEI_PLWSIZE     ,=0x45, "packet layer window size",       NULL)\
m(IEI_PSIZE       ,=0x46, "packet size",                    NULL)\
m(IEI_CUG         ,=0x47, "closed user group",              NULL)\
m(IEI_REVCHRGI    ,=0x4a, "reverse charging information",   NULL)\
m(IEI_CALLINGPN   ,=0x6c, "calling party number",           &f_cnu)\
m(IEI_CALLINGPS   ,=0x6d, "calling party subaddress",       NULL)\
m(IEI_CALLEDPN    ,=0x70, "called party number",            &f_cnu)\
m(IEI_CALLEDPS    ,=0x71, "called party subaddress",        NULL)\
m(IEI_REDIRNO     ,=0x74, "redirecting number",             &f_cnu)\
m(IEI_TRNSEL      ,=0x78, "transit network selection",      NULL)\
m(IEI_RESTARTI    ,=0x79, "restart indicator",              NULL)\
m(IEI_LLCOMPAT    ,=0x7c, "low layer compatibility",        NULL)\
m(IEI_HLCOMPAT    ,=0x7d, "high layer compatibility",       &f_hlc)\
m(IEI_USERUSER    ,=0x7e, "user-user",                      &f_uu)\
m(IEI_ESCAPE      ,=0x7f, "escape for extension",           NULL)\
\
/* Q.932 variable length information element identifiers */\
\
m(IEI_EXTFAC      ,=0x0d, "extended facility",              NULL)\
m(IEI_FACILITY    ,=0x1c, "facility",                       &f_fac)\
m(IEI_INFOREQ     ,=0x32, "information request",            NULL)\
m(IEI_FEATACT     ,=0x38, "feature activation",             NULL)\
m(IEI_FEATIND     ,=0x39, "feature indication",             NULL)\
m(IEI_SERVPID     ,=0x3a, "service profile identification", NULL)\
m(IEI_ENDPTID     ,=0x3b, "endpoint identifier",            NULL)\
\
/* Q.933 variable length information element identifiers */\
\
m(IEI_DATALCID    ,=0x19, "data link connection identifier",NULL)\
m(IEI_LLCOREP     ,=0x48, "link layer core parameters",     NULL)\
m(IEI_LLPROTP     ,=0x49, "link layer protocol parameters", NULL)\
m(IEI_X213PRI     ,=0x50, "X.213 priority",                 NULL)\
m(IEI_REPORTT     ,=0x51, "report type",                    NULL)\
m(IEI_LNKITYVERF  ,=0x53, "link integrity verification",    NULL)\
m(IEI_PVCSTAT     ,=0x57, "PVC status",                     NULL)\
\
/* Q.95x variable length information element identifiers */\
\
m(IEI_PRECLEV     ,=0x41, "precedence level",               NULL)\
m(IEI_CONCTDNO    ,=0x4c, "connected number",               &f_cnu)\
m(IEI_CONCTDSA    ,=0x4d, "connected subaddress",           NULL)\
m(IEI_REDICNNO    ,=0x76, "redirection number",             &f_cnu)\
/**/

MAKE_ENUM(Q931_INFORMATION_ELEMENTS);

/* Q.931 single octett information element identifiers */
#define IEI_SHIFT	0x90	/* shift codeset			*/
#define	 SHIFT_LOCK	0x08	/* shift codeset, locking shift bit	*/
#define IEI_MDSC	0xa0	/* more data AND/OR sending complete	*/
#define IEI_SENDCOMPL	0xa1	/* sending complete			*/
#define IEI_CONGLEVEL	0xb0	/* congestion level			*/
#define IEI_REPEATIND	0xd0	/* repeat indicator			*/

/* codesets */
#define	CODESET_0	0	/* codeset 0, normal DSS1 codeset	*/

/* Q.931/Q.932 message types (see Q.931 03/93 p10 and p311) */
#define Q931_MESSAGE_TYPES_EVENT(enum,value,event) [enum] = event
#define Q931_MESSAGE_TYPES_DESC( enum,value,event) [enum] = #enum
#define Q931_MESSAGE_TYPES(m)\
\
/* escape to nationally specific message type */\
\
m(ESCAPE                  ,=0x00, EV_L3_ILL     )\
\
/* call establishment messages */\
\
m(ALERT/*ING*/            ,=0x01, EV_L3_ALERT	)\
m(CALL_PROCEEDING         ,=0x02, EV_L3_CALLPRC	)\
m(PROGRESS                ,=0x03, EV_L3_PROGIND	)\
m(SETUP                   ,=0x05, EV_L3_SETUP	)\
m(CONNECT                 ,=0x07, EV_L3_CONNECT	)\
m(SETUP_ACKNOWLEDGE       ,=0x0d, EV_L3_SETUPAK	)\
m(CONNECT_ACKNOWLEDGE     ,=0x0f, EV_L3_CONACK	)\
\
/* call information phase messages */\
\
m(USER_INFORMATION        ,=0x20, EV_L3_ILL	)\
m(SUSPEND_REJECT          ,=0x21, EV_L3_ILL	)\
m(RESUME_REJECT           ,=0x22, EV_L3_ILL	)\
m(HOLD                    ,=0x24, EV_L3_HOLD_IND)\
m(SUSPEND                 ,=0x25, EV_L3_ILL	)\
m(RESUME                  ,=0x26, EV_L3_ILL	)\
m(HOLD_ACKNOWLEDGE        ,=0x28, EV_L3_ILL	)\
m(SUSPEND_ACKNOWLEDGE     ,=0x2d, EV_L3_ILL	)\
m(RESUME_ACKNOWLEDGE      ,=0x2e, EV_L3_ILL	)\
m(HOLD_REJECT             ,=0x30, EV_L3_ILL	)\
m(RETRIEVE                ,=0x31, EV_L3_RETRIEVE_IND)\
m(RETRIEVE_ACKNOWLEDGE    ,=0x33, EV_L3_RETRIEVE_ACK)\
m(RETRIEVE_REJECT         ,=0x37, EV_L3_ILL	)\
\
/* call clearing */\
\
m(DETACH                  ,=0x40, EV_L3_ILL     )\
m(DISCONNECT              ,=0x45, EV_L3_DISCONNECT )\
m(RESTART                 ,=0x46, EV_L3_RESTART_IND )\
m(DETACH_ACKNOWLEDGE      ,=0x48, EV_L3_ILL	)\
m(RELEASE                 ,=0x4d, EV_L3_RELEASE	)\
m(RESTART_ACKNOWLEDGE     ,=0x4e, EV_L3_RESTART_ACK )\
m(RELEASE_COMPLETE        ,=0x5a, EV_L3_RELEASE	)\
\
/* misc messages */\
\
m(SEGMENT                 ,=0x60, EV_L3_ILL	)\
m(FACILITY                ,=0x62, EV_L3_FACILITY)\
m(REGISTER                ,=0x64, EV_L3_ILL	)\
m(CANCEL_ACKNOWLEDGE      ,=0x68, EV_L3_ILL	)\
m(FACILITY_ACKNOWLEDGE    ,=0x6a, EV_L3_ILL	)\
m(REGISTER_ACKNOWLEDGE    ,=0x6c, EV_L3_ILL	)\
m(NOTIFY                  ,=0x6e, EV_L3_ILL	)\
m(CANCEL_REJECT           ,=0x70, EV_L3_ILL	)\
m(FACILITY_REJECT         ,=0x72, EV_L3_ILL	)\
m(REGISTER_REJECT         ,=0x74, EV_L3_ILL	)\
m(STATUS_ENQUIRY          ,=0x75, EV_L3_STATENQ	)\
m(CONGESTION_CONTROL      ,=0x79, EV_L3_ILL	)\
m(INFORMATION             ,=0x7b, EV_L3_INFO	)\
m(STATUS                  ,=0x7d, EV_L3_STATUS	)\
/**/

MAKE_ENUM(Q931_MESSAGE_TYPES);

#endif /* _DSS1_L3_H_ */

