/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	i4b_ioctl.h - messages kernel <--> userland
 *	-------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_IOCTL_H_
#define _I4B_IOCTL_H_

#include <i4b/include/i4b_limits.h>

/*---------------------------------------------------------------------------*
 *	version and release number for ISDN4BSD package
 *---------------------------------------------------------------------------*/
#define I4B_VERSION    2                /* version number */
#define I4B_REL        0                /* release number */
#define I4B_STEP       13               /* release step   */

/*---------------------------------------------------------------------------*
 * date/time format in I4B log messages
 * ------------------------------------
 * Being year 2000 clean is not easy with the current state of the
 * ANSI C library standard and it's implementation for some locales.
 * You might like to use the "%c" format of "strftime" sometimes,
 * but this breaks Y2K in some locales. Also the old standard logfile
 * format "%d.%m.%y %H:%M:%S" is non compliant.
 * NetBSD's current toolset warns about this problems, and we compile
 * with -Werror, so this problems need to be resolved.
 *---------------------------------------------------------------------------*/
#define I4B_TIME_FORMAT	"%d.%m.%Y %H:%M:%S"

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
#ifndef MAKE_ENUM
#define _MAKE_ENUM(enum,value,arg...)		\
	enum value,				\
/**/

#define MAKE_ENUM(macro,end...)	\
enum { macro(_MAKE_ENUM) end }	\
/**/
#endif

/*
 * check if ``[] = xxx'' is needed
 * must give in a table-size ``[number]'' when 
 * ``[] = xxx'' is used!
 */
#define __MAKE_TABLE(a...) a	/* double pass to expand all macros */
#define _MAKE_TABLE(a...) (a),	/* add comma */
#define MAKE_TABLE(m,field,p,a...) m##_##field p = { __MAKE_TABLE(m(m##_##field _MAKE_TABLE)) a }

#define INDEXES(table) (sizeof(table) / sizeof(table[0]))

/*---------------------------------------------------------------------------*
 *	controller types
 *---------------------------------------------------------------------------*/
#define L1_TYPES_DEFAULT_DRIVER_TYPE(enum,value,desc,default_driver_type) default_driver_type
#define L1_TYPES_DEFAULT_DRIVER_DESC(enum,value,desc,default_driver_type) desc
#define L1_TYPES(m)\
m(L1_TYPE_UNKNOWN  ,,"unknown"                          , DRVR_DUMMY         )\
m(L1_TYPE_ISDN_BRI ,,"passive ISDN (Basic Rate, 2xB)"   , DRVR_DSS1_TE       )\
m(L1_TYPE_DAIC     ,,"Diehl active"                     , DRVR_DIEHL_TE      )\
m(L1_TYPE_TINADD   ,,"Stollmann Tina-dd active"         , DRVR_TINA_DD_TE    )\
m(L1_TYPE_AVMB1    ,,"AVM B1 active"                    , DRVR_AMV_B1_TE     )\
m(L1_TYPE_CAPI     ,,"active CAPI 2.0"                  , DRVR_CAPI_TE       )\
m(L1_TYPE_ISDN_PRI ,,"passive ISDN (Primary Rate, 30xB)", DRVR_DSS1_P2P_TE   )\
/**/

MAKE_ENUM(L1_TYPES,
	N_L1_TYPES);

/*---------------------------------------------------------------------------*
 *	in case the source or destination telephone number is empty
 *---------------------------------------------------------------------------*/
#define TELNO_EMPTY	"NotAvailable"

/*---------------------------------------------------------------------------*
 * userland driver types
 * ---------------------
 * a "driver" is defined here as a piece of software interfacing an
 * ISDN channel
 *---------------------------------------------------------------------------*/
#define I4B_DRIVERS_DEVICE(    enum,value,desc,dev) #dev
#define I4B_DRIVERS_ISDND_RC_5(enum,value,desc,dev) "IT(Ar " #enum ") " desc " "
#define I4B_DRIVERS_NAME(      enum,value,desc,dev) #enum
#define I4B_DRIVERS_LLIST(     enum,value,desc,dev) { #enum, desc, enum },
#define I4B_DRIVERS_DLIST(     enum,value,desc,dev) #enum ", "
#define I4B_DRIVERS(m)		\
	I4B_B_DRIVERS(m)	\
	I4B_D_DRIVERS(m)
/**/

/* B-channel drivers */

#define I4B_B_DRIVERS(m) \
m(DRVR_IBC        ,, "BSD/OS point to point driver", null)\
m(DRVR_ING        ,, "NetGraph driver", null)\
m(DRVR_IPR        ,, "IP over raw HDLC interface driver", null)\
m(DRVR_ISPPP      ,, "sync Kernel PPP interface driver", null)\
m(DRVR_RBCH       ,, "raw B-channel interface driver", rbch)\
m(DRVR_TEL        ,, "telephone or speech interface driver", tel)\
m(DRVR_IHFC_DEV   ,, "ihfc interface driver", null)\
m(DRVR_CAPI_B3    ,, "CAPI application interface driver", null)\
m(DRVR_DIAL_GEN   ,, "I4B dialtone generator", null)\
m(DRVR_CAPI_BRIDGE,, "CAPI bridge driver", null)\
/**/

/* D-channel drivers */

#define I4B_D_DRIVERS(m) \
m(DRVR_DSS1_TE    ,,\
  "This is the terminal side, point to multipoint driver, "\
  "for the DSS1 or 'euro-ISDN' protocol. This driver "\
  "supports ITU Recommendations Q.921 and Q.931."\
  ""						,null)\
m(DRVR_DSS1_NT    ,,\
  "This is the network side, multipoint to point driver, "\
  "for the DSS1 or 'euro-ISDN' protocol. This driver "\
  "supports ITU Recommendations Q.921 and Q.931."\
  ""						,null)\
m(DRVR_CAPI_TE    ,, "", null)\
m(DRVR_DIEHL_TE   ,, "", null)\
m(DRVR_TINA_DD_TE ,, "", null)\
m(DRVR_AMV_B1_TE  ,, "", null)\
m(DRVR_D_CHANNEL  ,,\
  "This driver selects the default D-channel driver." \
  "", null)\
m(DRVR_D64S       ,,\
 "This driver can be used on single B-channel leased lines."\
 ""						,null)\
m(DRVR_DUMMY      ,, "", null)\
m(DRVR_DSS1_P2P_TE,,\
  "This is the terminal side, point to point driver, "\
  "for the DSS1 or 'euro-ISDN' protocol. This driver "\
  "supports ITU Recommendations Q.921 and Q.931."\
  ""						,null)\
m(DRVR_DSS1_P2P_NT,,\
  "This is the network side, point to point driver, "\
  "for the DSS1 or 'euro-ISDN' protocol. This driver "\
  "supports ITU Recommendations Q.921 and Q.931."\
  ""						,null)\
/**/

/*---------------------------------------------------------------------------*
 *	I4B-protocol structure
 *---------------------------------------------------------------------------*/
struct i4b_protocol {

    uint16_t protocol_1; /* I4B_PROTOCOLS() */
    uint16_t protocol_2;
    uint16_t protocol_3;
    uint16_t protocol_4; /* I4B_B_SUB_PROTOCOLS() */

    union {
        struct {
	    uint16_t rx_slot;
	    uint16_t tx_slot;

	    uint8_t  rx_cable;
	    uint8_t  tx_cable;
	} bridge;
        struct {
	    uint8_t  echo_cancel_enable;
	    uint8_t  dtmf_detect_enable;
	} transp;
    } u;
};

#define I4B_PROTOCOLS_DESC(enum,value,desc) #enum desc
#define I4B_PROTOCOLS(m)\
m(P_DISABLE              ,= 0,)\
\
/* hardware protocols */\
\
m(P_HDLC                 ,= 1,)\
m(P_FAX_G4               ,= 2,)\
m(P_FAX_G3               ,= 3,)\
\
/* software protocols */\
\
m(P_AUDIO                ,=10,)\
m(P_TRANS                ,=10,)\
m(P_TRANSPARENT          ,=10,)\
m(P_AUDIO_RING           ,=11,) /* ring buffer audio / mmap */	\
m(P_TRANS_RING           ,=11,) /* ring buffer trans / mmap */	\
m(P_TRANSPARENT_RING     ,=11,) /* */				\
m(P_HDLC_EMU             ,=12,)\
m(P_FAX_G3_EMU           ,=13,)\
m(P_FAX_G4_EMU           ,=14,)\
m(P_MODEM_EMU            ,=15,)\
m(P_HDLC_RING            ,=16,) /* not supported yet */		\
m(P_HDLC_EMU_D           ,=17,)\
\
/* external switching */\
\
m(P_BRIDGE               ,=20,)\
m(P_CONFERENCE           ,=21,)\
\
/* virtual protocols,			\
 * cannot be used with FIFO_SETUP(,)	\
 */					\
\
m(P_D_CHANNEL            ,=32,)\
/**/

#define PROT_IS_HL_VBR(prot) \
(((prot)->protocol_1 != P_DISABLE) && \
 ((prot)->protocol_1 != P_TRANSPARENT) && \
 ((prot)->protocol_1 != P_TRANSPARENT_RING)) /* Higher Layer Variable Bit Rate */

#define PROT_IS_HDLC(prot) ((prot)->protocol_1 == P_HDLC) /* hardware HDLC */
#define PROT_IS_TRANSPARENT(prot) (((prot)->protocol_1 >= 10) && \
				   ((prot)->protocol_1 < 20)) /* software codecs */

#define I4B_B_PROTOCOLS_ISDND_RC_5(       enum,value,protocol,desc,dss1_conv) "IT(Ar " #enum ")" desc " "
#define I4B_B_PROTOCOLS_ISDND_CONFIG_DESC(enum,value,protocol,desc,dss1_conv) #enum "\t\t#" desc
#define I4B_B_PROTOCOLS_PROTOCOL(         enum,value,protocol,desc,dss1_conv) protocol
#define I4B_B_PROTOCOLS_DSS1_CONV(        enum,value,protocol,desc,dss1_conv) dss1_conv
#define I4B_B_PROTOCOLS(m) \
m(BPROT_DISABLE      ,, P_DISABLE      ,\
  "disable sending and reception of data"		   ,0x00)\
m(BPROT_NONE_VOD     ,, P_TRANSPARENT  ,\
  "no protocol at all, raw data (Voice over Data)"         ,0x88)\
m(BPROT_RHDLC        ,, P_HDLC         ,\
  "HDLC protocol: flag, data, crc, flag"	           ,0x88)\
m(BPROT_RHDLC_DOV    ,, P_HDLC         ,\
  "HDLC protocol: flag, data, crc, flag (Data over Voice)" ,0x80)\
m(BPROT_NONE         ,, P_TRANSPARENT  ,\
"no protocol at all, raw data"				   ,0x80)\
m(BPROT_NONE_3_1_KHZ ,, P_TRANSPARENT  ,\
  "no protocol at all, raw data (Fax over Voice)"          ,0x90)\
/**/

#define I4B_B_SUB_PROTOCOLS(m) \
m(BSUBPROT_UNKNOWN   ,, )   \
m(BSUBPROT_G711_ALAW ,, )   \
m(BSUBPROT_G711_ULAW ,, )   \
m(BSUBPROT_PLAIN_ALAW ,, )   \
m(BSUBPROT_PLAIN_ULAW ,, )   \
m(BSUBPROT_SIGNED_8BIT ,, )  \
m(BSUBPROT_MUTED ,, )  \
/**/

MAKE_ENUM(I4B_DRIVERS,
	N_I4B_DRIVERS);

MAKE_ENUM(I4B_PROTOCOLS);

MAKE_ENUM(I4B_B_PROTOCOLS,
	N_I4B_B_PROTOCOLS);

MAKE_ENUM(I4B_B_SUB_PROTOCOLS,
	N_I4B_B_SUB_PROTOCOLS);

/*---------------------------------------------------------------------------*
 * causes data type
 *---------------------------------------------------------------------------*/
typedef	unsigned int cause_t;		/* 32 bit unsigned int	*/

/*---------------------------------------------------------------------------*
 * call descriptor id (cdid) definitions
 *---------------------------------------------------------------------------*/
typedef unsigned int cdid_t;

#define CDID_UNUSED	0	/* cdid is invalid and unused */
#define CDID2CONTROLLER(cdid) ((cdid) % I4B_MAX_CONTROLLERS)
#define CDID2CALLREFERENCE(cdid) ((cdid) / I4B_MAX_CONTROLLERS)
#define MAKE_CDID(controller,callreference) (((controller) % I4B_MAX_CONTROLLERS)| \
			((callreference) * I4B_MAX_CONTROLLERS))

/*---------------------------------------------------------------------------*
 *	The shorthold algorithm to use
 *---------------------------------------------------------------------------*/
#define SHA_FIXU	0    /* timeout algorithm for fix unit charging */
#define SHA_VARU	1    /* timeout algorithm for variable unit charging */

/*---------------------------------------------------------------------------*
 *	The shorthold data struct
 *---------------------------------------------------------------------------*/
typedef struct {
	int	shorthold_algorithm;	/* shorthold algorithm to use	*/
	int	unitlen_time;		/* length of a charging unit	*/
	int	idle_time;		/* time without activity on b ch*/
	int	earlyhup_time;		/* safety area at end of unit	*/
} msg_shorthold_t;


/****************************************************************************

        outgoing call:
        --------------

                userland                kernel
                --------                ------

                CDID_REQ ----------------->

                    <------------------ cdid
        
                CONNECT_REQ -------------->

                INFORMATION_REQ ---------->  (if overlap sending)

                    <------------------ PROCEEDING_IND (if connect req ok)

                    <------------------ CONNECT_ACTIVE_IND (if connection ok)

                or

                    <------------------ DISCONNECT_IND (if connection failed)
                    
                

        incoming call:
        --------------

                userland                kernel
                --------                ------

                    <------------------ CONNECT_IND (this message can be
                                                     replayed one or more times,
                                                     with updated destination
                                                     telephone number)

                    <------------------ INFORMATION_IND (if overlap sending 
                                                         in NT-mode)

   ALERT_REQ / CALL_PROCEEDING ----------->

                CONNECT_RESP ------------->

                    <------------------ CONNECT_ACTIVE_IND (if accepted)



        active disconnect:
        ------------------

                userland                kernel
                --------                ------

                DISCONNECT_REQ ------------>

                    <------------------ DISCONNECT_IND
                    

        passive disconnect:
        -------------------

                userland                kernel
                --------                ------

                    <------------------ DISCONNECT_IND


        setup of B-channel:
        -------------------

                userland                kernel
                --------                ------

     LINK_B_CHANNEL_DRIVER_REQ ------------>


****************************************************************************/


/*===========================================================================*
 *===========================================================================*
 *	"read" messages from kernel -> userland
 *
 * all data-structures must start with ``msg_hdr_t xxx;''
 *===========================================================================* 
 *===========================================================================*/

 
/*---------------------------------------------------------------------------*
 *	message header, included in every message
 *---------------------------------------------------------------------------*/
typedef struct {
	uint8_t	type;		/* message identifier		*/
	cdid_t		cdid;		/* call descriptor id		*/
} msg_hdr_t;

enum
{
  MSG_CONNECT_IND,
  MSG_CONNECT_ACTIVE_IND,
  MSG_DISCONNECT_IND,
  MSG_DIALOUT_IND,
  MSG_IDLE_TIMEOUT_IND,
  MSG_ACCT_IND,
  MSG_CHARGING_IND,
  MSG_PROCEEDING_IND,
  MSG_ALERT_IND,
  MSG_DRVRDISC_REQ,
  MSG_DRVRANSWER_REQ,
  MSG_L12STAT_IND,
  MSG_TEIASG_IND,
  MSG_NEGCOMP_IND,
  MSG_IFSTATE_CHANGED_IND,
  MSG_DIALOUTNUMBER_IND,
  MSG_PACKET_IND,
  MSG_KEYPAD_IND,
  MSG_DRVRREJECT_REQ,
  MSG_INFORMATION_IND,
  MSG_PRE_DISCONNECT_IND,
  MSG_HOLD_IND,
  MSG_RETRIEVE_IND,
};

/*---------------------------------------------------------------------------*
 *	connect indication
 *		indicates incoming connection
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		*/
	int		channel;	/* channel number		*/
#define  CHAN_D1  0		/* hardcoded value                      */
#define  CHAN_B1  1		/* hardcoded value                      */
#define  CHAN_B2  2		/* hardcoded value                      */
#define  CHAN_ANY (-1)		/* outgoing, not possible for incoming	*/
#define  CHAN_NOT_ANY  (-2)    	/* call waiting (CW) for incoming	*/
	int		bprot;	/* channel b-protocol, see BPROT_XXX	*/
	char		dst_telno[TELNO_MAX];	/* destination telno	*/
	char		dst_subaddr[SUBADDR_MAX];	/* dest subaddr */
	char		src_telno[TELNO_MAX];	/* source telno		*/
	char		src_subaddr[SUBADDR_MAX];	/* src subaddr	*/
	u_char		dst_ton;		/* dest. type of number */
	u_char		src_ton;		/* src. type of number	*/
#define  TON_OTHER    0		/* other type of number 		*/
#define  TON_INTERNAT 1		/* international number			*/
#define  TON_NATIONAL 2		/* national number			*/
	u_char		scr_ind;/* screening indicator			*/
#define  SCR_NONE     0		/* no screening indicator transmitted	*/
#define  SCR_USR_NOSC 1		/* screening user provided, not screened*/
#define  SCR_USR_PASS 2		/* screening user provided, verified & passed */
#define  SCR_USR_FAIL 3		/* screening user provided, verified & failed */
#define  SCR_NET      4		/* screening network provided		*/
	u_char		prs_ind;/* presentation indicator		*/
#define  PRS_NONE     0		/* no presentation indicator transmitted*/
#define  PRS_ALLOWED  1		/* presentation allowed			*/
#define  PRS_RESTRICT 2		/* presentation restricted		*/
#define  PRS_NNINTERW 3		/* number not available due to interworking */
#define  PRS_RESERVED 4		/* reserved				*/
	char		display[DISPLAY_MAX];	/* content of display IE*/
	char		user_user[USER_USER_MAX]; /* content of useruser IE*/
	u_char          sending_complete : 1;
	u_char          unused : 7;
} msg_connect_ind_t;

/*---------------------------------------------------------------------------*
 *	information
 *		indicates additional dial digits in the destination
 *		telephone number (incoming calls only)
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		*/
	char		dst_telno[TELNO_MAX];	/* dial digit string 	*/
} msg_information_ind_t;

/*---------------------------------------------------------------------------*
 *	connect active indication
 *		indicates active connection
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		   */
	int		channel;	/* channel number actually used    */
	char		datetime[DATETIME_MAX];	/* content of date/time IE */
} msg_connect_active_ind_t;

/*---------------------------------------------------------------------------*
 *	disconnect indication or pre-disconnect indication
 *		indicates a disconnect
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	cause_t		cause;		/* cause code		*/
} msg_disconnect_ind_t;

typedef msg_disconnect_ind_t msg_pre_disconnect_ind_t;

/*---------------------------------------------------------------------------*
 *	negotiation complete
 *		indicates an interface is completely up & running
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
} msg_negcomplete_ind_t;

/*---------------------------------------------------------------------------*
 *	interface changes internal state
 *		indicates an interface has somehow switched its FSM
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		state;	       	/* new interface state */
} msg_ifstatechg_ind_t;

/*---------------------------------------------------------------------------*
 *	initiate a call to a remote site
 *		i.e. the IP driver got a packet and wants a connection
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		driver;		/* driver type		*/
	int		driver_unit;	/* driver unit number	*/
} msg_dialout_ind_t;

/*---------------------------------------------------------------------------*
 *	dial a number
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		driver;		/* driver type		*/
	int		driver_unit;	/* driver unit number	*/
	int		cmdlen;		/* length of string	*/
	int		subaddrlen;	/* length of subaddr	*/
	char		cmd[TELNO_MAX];	/* the number to dial	*/	
	char		subaddr[SUBADDR_MAX];	/* dest subaddr	*/	
} msg_dialoutnumber_ind_t;

/*---------------------------------------------------------------------------*
 *	send keypad string
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		driver;		/* driver type		*/
	int		driver_unit;	/* driver unit number	*/
	int		cmdlen;		/* length of string	*/
	char		cmd[KEYPAD_MAX];/* keypad string	*/	
} msg_keypad_ind_t;

/*---------------------------------------------------------------------------*
 *	idle timeout disconnect sent indication
 *		kernel has sent disconnect request because of b-ch idle
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
} msg_idle_timeout_ind_t;

/*---------------------------------------------------------------------------*
 *	accounting information from userland interface driver to daemon
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		*/
	int		accttype;	/* accounting type		*/
#define  ACCT_DURING 0
#define  ACCT_FINAL  1
	int		driver;		/* driver type			*/
	int		driver_unit;	/* driver unit number		*/
	int		ioutbytes;	/* ISDN # of bytes sent		*/
	int		iinbytes;	/* ISDN # of bytes received	*/
	int		outbps;		/* bytes per sec out		*/
	int		inbps;		/* bytes per sec in		*/
	int		outbytes;	/* driver # of bytes sent	*/
	int		inbytes;	/* driver # of bytes received	*/
} msg_accounting_ind_t;

/*---------------------------------------------------------------------------*
 *	charging information from isdn driver to daemon
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		*/
	int		units;		/* number of units		*/
	int		units_type;	/* type of units info		*/
#define  CHARGE_INVALID	0	/* invalid, unknown */
#define  CHARGE_AOCD	1	/* advice of charge during call */
#define  CHARGE_AOCE	2	/* advice of charge at end of call */
#define  CHARGE_CALC	3	/* locally calculated from rates information */
} msg_charging_ind_t;

/*---------------------------------------------------------------------------*
 *	call proceeding indication
 *		indicates outgoing SETUP has been acknowleged
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		   */
	u_char          sending_complete; /* set if dest. number is complete */
} msg_proceeding_ind_t;

/*---------------------------------------------------------------------------*
 *	call retrieve indication
 *		indicates that the remote end has retrieved the call
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		   */
} msg_retrieve_ind_t;

/*---------------------------------------------------------------------------*
 *	call hold indication
 *		indicates that the remote end has put the call on hold
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		   */
} msg_hold_ind_t;

/*---------------------------------------------------------------------------*
 *	alert indication
 *		indicates remote user side "rings"
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header		   */
} msg_alert_ind_t;

/*---------------------------------------------------------------------------*
 *	driver requests to disconnect line
 *	driver requests to answer incoming call
 *	driver requests to reject incoming call
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		driver;		/* driver type		*/
	int		driver_unit;	/* driver unit number	*/
} msg_drvrdisc_req_t, msg_drvranswer_req_t, msg_drvrreject_req_t;

/*---------------------------------------------------------------------------*
 *	connect packet logging
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		driver;		/* driver type		*/
	int		driver_unit;	/* driver unit number	*/
	int		direction_out;	/* 0=in 1=out		*/
#define MAX_PACKET_LOG	40		/* space for IP and TCP header	*/
	uint8_t	pktdata[MAX_PACKET_LOG];
} msg_packet_ind_t;

/*---------------------------------------------------------------------------*
 *	state of layer 1/2
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		layer;		/* layer number (1/2)	*/
#define LAYER_ONE	1
#define LAYER_TWO	2
	int		state;		/* state info		*/
#define LAYER_IDLE	0
#define LAYER_ACTIVE	1
} msg_l12stat_ind_t;

/*---------------------------------------------------------------------------*
 *	TEI assignment messages
 *---------------------------------------------------------------------------*/
typedef struct {
	msg_hdr_t	header;		/* common header	*/
	int		tei;		/* TEI or -1 if invalid */
} msg_teiasg_ind_t;

/*===========================================================================*
 *===========================================================================*
 *	"ioctl" messages from userland -> kernel
 *
 * all data-structures must start with "cdid_t xxx;"
 *
 * IMPORTANT: please set the data-structure to zero before
 *            setting any parameters, so that unused parameters
 *            get a default value !
 *
 *===========================================================================* 
 *===========================================================================*/


/*---------------------------------------------------------------------------*
 *	request a unique cdid (to setup an outgoing call)
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		cdid;			/* call descriptor id	*/
} msg_cdid_req_t;
 
#define	I4B_CDID_REQ		_IOWR('4', 0, cdid_t)

/*---------------------------------------------------------------------------*
 *	connect request
 *		requests an outgoing connection
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		cdid;		/* call descriptor id		     */
	int		channel;	/* channel to use		     */
	int		txdelay;	/* tx delay after connect	     */
	int		bprot;		/* channel b-protocol		     */
	int		driver;		/* driver to route b channel data to */
	int		driver_unit;	/*      unit number for above driver */
	msg_shorthold_t	shorthold_data;	/* the shorthold data		     */
	unsigned short	unitlen_method;	/* how to calculate the unitlength   */
#define  ULEN_METHOD_STATIC  0	/* use unitlen_time value (see above) */
#define  ULEN_METHOD_DYNAMIC 1	/* use AOCD */	
	unsigned short  sending_not_complete : 1;
	unsigned short  unused : 15;
	char		dst_telno[TELNO_MAX];	/* destination telephone no  */
	char		dst_subaddr[SUBADDR_MAX];	/* dest subaddr      */
	char		src_telno[TELNO_MAX];	/* source telephone number   */
	char		src_subaddr[SUBADDR_MAX];	/* source subaddr    */
	char		keypad[KEYPAD_MAX];	/* keypad string 	     */	
} msg_connect_req_t;

#define	I4B_CONNECT_REQ	_IOW('4', 1, msg_connect_req_t)

/*---------------------------------------------------------------------------*
 *	information request
 *		sends additional dial digits (outgoing calls only)
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		cdid;		/* call descriptor id		     */
	char		dst_telno[TELNO_MAX];	/* dial digit string to send */
} msg_information_req_t;

#define	I4B_INFORMATION_REQ _IOW('4', 2, msg_information_req_t)

/*---------------------------------------------------------------------------*
 *	connect response
 *		this is the answer to an incoming connect indication
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t	cdid;		/* call descriptor id			*/
	int	response;	/* what to do with incoming call	*/
#define  SETUP_RESP_DNTCRE 0	/* don't care, call is not for me	*/
#define  SETUP_RESP_REJECT 1	/* reject call				*/
#define  SETUP_RESP_ACCEPT 2	/* accept call				*/
	cause_t	cause;		/* cause for case SETUP_RESP_REJECT	*/
		/* the following are only used for SETUP_RESP_ACCEPT !! */
	int	txdelay;	/* tx delay after connect		*/
	int	bprot;		/* B chan protocol			*/
	int	driver;		/* driver to route b channel data to	*/
	int	driver_unit;	/*      unit number for above driver	*/
	int	max_idle_time;	/* max time without activity on b ch	*/	
} msg_connect_resp_t;
	
#define	I4B_CONNECT_RESP	_IOW('4', 3, msg_connect_resp_t)

/*---------------------------------------------------------------------------*
 *	disconnect request
 *		active disconnect request
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t	cdid;		/* call descriptor id			*/
	cause_t	cause;		/* protocol independent cause		*/
} msg_disconnect_req_t;
	
#define	I4B_DISCONNECT_REQ	_IOW('4', 4, msg_disconnect_req_t)

/*---------------------------------------------------------------------------*
 *	controller info request
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t	  controller;	/* controller number */
        uint16_t l1_channels;  /* number of channels provided */
	uint32_t l1_serial;    /* serial number used */
	uint8_t  l1_type;  	/* controller type passive/active */
	char     l1_desc[64];	/* controller description, zero terminated */
	char     l1_state[64]; /* controller state, zero terminated */
	u_char    l1_active : 1;/* set if layer 1 is active */
	u_char    l1_no_dialtone : 1; 
	u_char    l1_no_power_save : 1;
	u_char    l1_attached : 1;
	u_char    l3_no_status_enquiry : 1; 
	u_char    l1_unused : 3;
	uint32_t l2_driver_type;
} msg_ctrl_info_req_t;
	
#define	I4B_CTRL_INFO_REQ	_IOWR('4', 5, msg_ctrl_info_req_t)

/*---------------------------------------------------------------------------*
 *	response to user
 *		status report to the driver which requested a dialout
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		controller;	/* not used, but must be zero	     */
	u_int		driver_type;	/* driver to route data to           */
	u_int		driver_unit;	/*      unit number for above driver */
	u_int		status;		/* state of dialout request	     */

#define DSTATS_IPHONE_TABLE(enum,value,desc,iphone_event,iphone_exitvalue) { iphone_event, iphone_exitvalue, desc }
#define DSTATS_DESC(enum,value,desc,iphone_event,iphone_exitvalue) desc
#define DSTATS(m)\
m(DSTAT_NOT_CONNECTED  ,, "disconnected"  , EV_DISCONNECT,2)\
m(DSTAT_INCOMING_CALL  ,, "incoming call" , EV_CALL_IN,2 /*??*/)\
m(DSTAT_CONNECTED      ,, "connected"	  , EV_CONNECTED,0)\
\
/* outgoing call failures */\
\
m(DSTAT_BUSY           ,, "disconnected, remote is busy", EV_DISCONNECT,1)\
m(DSTAT_NOA            ,, "disconnected, no answer", EV_DISCONNECT,3)\
m(DSTAT_TFAIL          ,, "disconnected, temporary dialout failure", EV_DISCONNECT,2)\
m(DSTAT_PFAIL          ,, "disconnected, permanent dialout failure", EV_DISCONNECT,2)\
m(DSTAT_INONLY         ,, "disconnected, dialout not allowed", EV_DISCONNECT,2)\
m(DSTAT_NO_ENTRY       ,, "disconnected, no valid configuration entry", EV_DISCONNECT,2)\
/**/

#define DSTAT_IS_DIAL_FAILURE(status) 		\
  (((status) != DSTAT_NOT_CONNECTED) &&		\
   ((status) != DSTAT_INCOMING_CALL) &&		\
   ((status) != DSTAT_CONNECTED))		\
/**/

	cause_t		cause;		/* exact i4b cause */
	char		src_telno[TELNO_MAX]; /**/
	char		src_display[DISPLAY_MAX]; /**/
	char		src_user_user[USER_USER_MAX]; /**/
} msg_response_to_user_t;

MAKE_ENUM(DSTATS,
        N_DSTATS);

#define	I4B_RESPONSE_TO_USER	_IOW('4', 6, msg_response_to_user_t)

/*---------------------------------------------------------------------------*
 *	timeout value update
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		cdid;		/* call descriptor id */
	msg_shorthold_t	shorthold_data;
} msg_timeout_upd_t;

#define	I4B_TIMEOUT_UPD		_IOW('4', 7, msg_timeout_upd_t)

/*---------------------------------------------------------------------------*
 *	send alert request or call proceeding
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		cdid;		/* call descriptor id */
	u_char		send_call_proceeding : 1;
	u_char		unused : 7;
} msg_alert_req_t;

#define	I4B_ALERT_REQ		_IOW('4', 8, msg_alert_req_t)

/*---------------------------------------------------------------------------*
 *	set ISDN protocol for a controller
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		controller;	/* controller number		*/
	int		driver_type;	/* ISDN D-channel driver type   */
	int		serial_number;	/* ISDN D-channel serial number */
} msg_prot_ind_t;

#define I4B_PROT_IND		_IOW('4', 9, msg_prot_ind_t)

/*---------------------------------------------------------------------------*
 *	request version and release info from kernel part
 *
 * NOTE: I4B writes the version over the cdid, and assumes that the kernel
 *	 has pre-zeroed the read-only data!
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t	  version;      /* version number */
	uint32_t release;      /* release number */
	uint32_t step;	        /* release step number */	
	uint32_t max_controllers; /* maximum number of controllers in system */
	uint32_t max_channels; /* maximum number of channels in system */
} msg_vr_req_t;

#define I4B_VR_REQ              _IOR('4',10, msg_vr_req_t)

/*---------------------------------------------------------------------------*
 *	replay connect indication (incoming calls only)
 *---------------------------------------------------------------------------*/
typedef struct {
	cdid_t		cdid;		/* call descriptor id */
} msg_connect_replay_req_t;

#define I4B_CONNECT_REPLAY_REQ _IOW('4',11, msg_connect_replay_req_t)

/*---------------------------------------------------------------------------*
 *	connect/disconnect B-channel driver 
 *
 * This request is typically used after that the connect
 * indication has been received, to setup the B-channel driver.
 *
 * If the call is outgoing, this request can be used after
 * that the proceeding indication has been received.
 *
 * If the call is incoming and the controller is set to
 * Network-mode, this request can be used after that
 * the connect indication has been received.
 */
typedef struct {
	cdid_t		cdid;		/* call descriptor id */
	u_char		activate;       /* set if connect else disconnect */
	int		driver;		/* driver to route B-channel data to */
	int		driver_unit;	/* unit number for above driver */
} msg_link_b_channel_driver_req_t;

#define I4B_LINK_B_CHANNEL_DRIVER_REQ _IOW('4',12, msg_link_b_channel_driver_req_t)

/*---------------------------------------------------------------------------*
 *	protocol download to active cards
 *---------------------------------------------------------------------------*/
struct isdn_dr_prot {
	uint32_t bytecount;	/* length of code */
	uint8_t *microcode;	/* pointer to microcode */
};

struct isdn_download_request {
	cdid_t controller;	/* controller number */
	uint32_t numprotos;	/* number of protocols pointed 
				 * to by "protocols"
				 */
	struct isdn_dr_prot *protocols;
};

#define	I4B_CTRL_DOWNLOAD	_IOW('4', 20, struct isdn_download_request)

/*---------------------------------------------------------------------------*
 *	generic diagnostic interface for active cards
 *---------------------------------------------------------------------------*/
struct isdn_diagnostic_request {
	cdid_t controller;     	/* controller number */
	uint32_t cmd;		/* diagnostic command to execute */
	uint32_t in_param_len;	/* length of additional input parameter */
#define I4B_ACTIVE_DIAGNOSTIC_MAXPARAMLEN	0x10000
	void *in_param_ptr;	/* optional input parameter */
	uint32_t out_param_len;	/* available output space */
	void *out_param_ptr;	/* output data goes here */
};

#define	I4B_ACTIVE_DIAGNOSTIC	_IOW('4', 21, struct isdn_diagnostic_request)

#endif /* _I4B_IOCTL_H_ */
