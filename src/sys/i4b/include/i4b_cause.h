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
 *	i4b_cause.h - causes and cause handling for i4b
 *	-----------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_CAUSE_H_
#define _I4B_CAUSE_H_

#ifndef MAKE_ENUM
#define _MAKE_ENUM(enum,value,arg...)		\
	enum value,

#define MAKE_ENUM(macro,end...)			\
	enum { macro(_MAKE_ENUM) end }
#endif

/*---------------------------------------------------------------------------*
 *	ISDN4BSD internal causes specification
 *---------------------------------------------------------------------------*/

#define CAUSE_VAL	0x000000ff	/* cause value			*/
#define CAUSE_TYPE	0x0000ff00	/* cause type			*/
#define  CAUSET_Q850	0		/* value coded according to Q.850 */
#define  CAUSET_I4B	1		/* I4B protocol independent causes*/

#define GET_CAUSE_VAL(cause)	  ((cause) & 0xff)
#define SET_CAUSE_VAL(dest, val)  ((dest) = ((dest & 0xffffff00) | \
						(val & 0x000000ff)))

#define GET_CAUSE_TYPE(cause)	   (((cause) >> 8) & 0xff)
#define SET_CAUSE_TYPE(dest, type) ((dest) = ((dest & 0xffff00ff) | \
						((type << 8) & 0x0000ff00)))

#define SET_CAUSE_TV(dest, type, val) ((dest) = ((val & 0x000000ff) | \
						 ((type << 8) & 0x0000ff00)))

/*
 * CAUSET_I4B - protocol independent cause values
 */
#define I4B_CAUSES_Q850_CONV(enum,value,desc,q850_conv) [enum] = q850_conv
#define I4B_CAUSES_DESC(     enum,value,desc,q850_conv) [enum] = desc
#define I4B_CAUSES(m)\
m(CAUSE_I4B_NORMAL        ,,"normal call clearing",CAUSE_Q850_NCCLR)\
m(CAUSE_I4B_BUSY          ,,"user busy",CAUSE_Q850_USRBSY)\
m(CAUSE_I4B_NOCHAN        ,,"no circuit-channel available",CAUSE_Q850_NOCAVAIL)\
m(CAUSE_I4B_INCOMP        ,,"incompatible source or destination",CAUSE_Q850_INCDEST)\
m(CAUSE_I4B_REJECT        ,,"call rejected",CAUSE_Q850_CALLREJ)\
m(CAUSE_I4B_OOO           ,,"destination out of order",CAUSE_Q850_DSTOOORDR)\
m(CAUSE_I4B_TMPFAIL       ,,"temporary failure",CAUSE_Q850_TMPFAIL)\
m(CAUSE_I4B_L1ERROR       ,,"layer 1 error or persistent deactivation",CAUSE_Q850_USRBSY)\
m(CAUSE_I4B_L2ERROR       ,,"layer 2 error",CAUSE_Q850_USRBSY)\
m(CAUSE_I4B_LLDIAL        ,,"no dialout on leased line",CAUSE_Q850_USRBSY)\
/**/

MAKE_ENUM(I4B_CAUSES, 
        N_I4B_CAUSES);

/*
 * CAUSET_Q850 - causes defined in Q.850 
 */
#define Q850_CAUSES_DESC(enum,value,desc) [enum] = desc
#define Q850_CAUSES(m)\
m(CAUSE_Q850_SHUTDN       ,=0x00, "normal D-channel shutdown")\
m(CAUSE_Q850_NUNALLC      ,=0x01, "unallocated (unassigned) number")\
m(CAUSE_Q850_NRTTN        ,=0x02, "no route to specified transit network (national use)")\
m(CAUSE_Q850_NRTDST       ,=0x03, "no route to destination")\
m(CAUSE_Q850_SSINFTN      ,=0x04, "send special information tone")\
m(CAUSE_Q850_MDIALTP      ,=0x05, "misdialled trunk prefix (national use)")\
m(CAUSE_Q850_CHUNACC      ,=0x06, "channel unacceptable")\
m(CAUSE_Q850_CALLAWD      ,=0x07, "call awarded and being delivered in an established channel")\
m(CAUSE_Q850_PREEMPT      ,=0x08, "preemption")\
m(CAUSE_Q850_PREECRR      ,=0x09, "preemption - circuit reserved for reuse")\
m(CAUSE_Q850_NCCLR        ,=0x10, "normal call clearing")\
m(CAUSE_Q850_USRBSY       ,=0x11, "user busy")\
m(CAUSE_Q850_NOUSRRSP     ,=0x12, "no user responding")\
m(CAUSE_Q850_NOANSWR      ,=0x13, "no answer from user (user alerted)")\
m(CAUSE_Q850_SUBSABS      ,=0x14, "subscriber absent")\
m(CAUSE_Q850_CALLREJ      ,=0x15, "call rejected")\
m(CAUSE_Q850_NUCHNG       ,=0x16, "number changed")\
m(CAUSE_Q850_NONSELUC     ,=0x1a, "non-selected user clearing")\
m(CAUSE_Q850_DSTOOORDR    ,=0x1b, "destination out of order")\
m(CAUSE_Q850_INVNUFMT     ,=0x1c, "invalid number format")\
m(CAUSE_Q850_FACREJ       ,=0x1d, "facility rejected")\
m(CAUSE_Q850_STENQRSP     ,=0x1e, "response to STATUS ENQUIRY")\
m(CAUSE_Q850_NORMUNSP     ,=0x1f, "normal, unspecified")\
m(CAUSE_Q850_NOCAVAIL     ,=0x22, "no circuit-channel available")\
m(CAUSE_Q850_NETOOORDR    ,=0x26, "network out of order")\
m(CAUSE_Q850_PFMCDOOSERV  ,=0x27, "permanent frame mode connection out of service")\
m(CAUSE_Q850_PFMCOPER     ,=0x28, "permanent frame mode connection operational")\
m(CAUSE_Q850_TMPFAIL      ,=0x29, "temporary failure")\
m(CAUSE_Q850_SWEQCONG     ,=0x2a, "switching equipment congestion")\
m(CAUSE_Q850_ACCINFDIS    ,=0x2b, "access information discarded")\
m(CAUSE_Q850_REQCNOTAV    ,=0x2c, "requested circuit/channel not available")\
m(CAUSE_Q850_PRECALBLK    ,=0x2e, "precedence call blocked")\
m(CAUSE_Q850_RESUNAVAIL   ,=0x2f, "resources unavailable, unspecified")\
m(CAUSE_Q850_QOSUNAVAIL   ,=0x31, "quality of service unavailable")\
m(CAUSE_Q850_REQSERVNS    ,=0x32, "requested facility not subscribed")\
m(CAUSE_Q850_OCBARRCUG    ,=0x35, "outgoing calls barred within CUG")\
m(CAUSE_Q850_ICBARRCUG    ,=0x36, "incoming calls barred within CUG")\
m(CAUSE_Q850_ICBARRCUG1   ,=0x37, "incoming calls barred within CUG")\
m(CAUSE_Q850_BCAPNAUTH    ,=0x39, "bearer capability not authorized")\
m(CAUSE_Q850_BCAPNAVAIL   ,=0x3a, "bearer capability not presently available")\
m(CAUSE_Q850_INCSTOACISC  ,=0x3e, "inconsistenciy in designated outgoing access information and subscriber class")\
m(CAUSE_Q850_SOONOTAVAIL  ,=0x3f, "service or option not available, unspecified")\
m(CAUSE_Q850_BCAPNOTIMPL  ,=0x41, "bearer capability not implemented")\
m(CAUSE_Q850_CHTYPNIMPL   ,=0x42, "channel type not implemented")\
m(CAUSE_Q850_REQFACNIMPL  ,=0x45, "requested facility not implemented")\
m(CAUSE_Q850_ORDINBCAVL   ,=0x46, "only restricted digital information bearer capability is available")\
m(CAUSE_Q850_SOONOTIMPL   ,=0x4f, "service or option not implemented, unspecified")\
m(CAUSE_Q850_INVCLRFVAL   ,=0x51, "invalid call reference value")\
m(CAUSE_Q850_IDCHDNOEX    ,=0x52, "identified channel does not exist")\
m(CAUSE_Q850_SUSCAEXIN    ,=0x53, "a suspended call exists, but this call identity does not")\
m(CAUSE_Q850_CLIDINUSE    ,=0x54, "call identity in use")\
m(CAUSE_Q850_NOCLSUSP     ,=0x55, "no call suspended")\
m(CAUSE_Q850_CLIDCLRD     ,=0x56, "call having the requested call identity has been cleared")\
m(CAUSE_Q850_UNOTMEMCUG   ,=0x57, "user not member of CUG")\
m(CAUSE_Q850_INCDEST      ,=0x58, "incompatible destination")\
m(CAUSE_Q850_NONEXCUG     ,=0x5a, "non-existent CUG")\
m(CAUSE_Q850_INVNTWSEL    ,=0x5b, "invalid transit network selection")\
m(CAUSE_Q850_INVMSG       ,=0x5f, "invalid message, unspecified")\
m(CAUSE_Q850_MIEMISS      ,=0x60, "mandatory information element is missing")\
m(CAUSE_Q850_MSGTNI       ,=0x61, "message type non-existent or not implemented")\
m(CAUSE_Q850_MSGNCMPT     ,=0x62, "message not compatible with call state or message type non-existent or not implemented")\
m(CAUSE_Q850_IENENI       ,=0x63, "information element/parameter non-existent or not implemented")\
m(CAUSE_Q850_INVIEC       ,=0x64, "invalid information element contents")\
m(CAUSE_Q850_MSGNCWCS     ,=0x65, "message not compatible with call state")\
m(CAUSE_Q850_RECOTIMEXP   ,=0x66, "recovery on timer expiry")\
m(CAUSE_Q850_PARMNENIPO   ,=0x67, "parameter non-existent or not implemented, passed on")\
m(CAUSE_Q850_MSGUNRDPRM   ,=0x6e, "message with unrecognized parameter, discarded")\
m(CAUSE_Q850_PROTERR      ,=0x6f, "protocol error, unspecified")\
m(CAUSE_Q850_INTWRKU      ,=0x7f, "interworking, unspecified")\
/**/

MAKE_ENUM(Q850_CAUSES);

/*
 * causes defined in 1TR6
 * - obsolete german national ISDN
 */
#define ITR6_CAUSES_DESC(enum,value,desc) [enum] = desc
#define ITR6_CAUSES(m)\
m(CAUSE_1TR6_SHUTDN       ,=0x00, "normal D-channel shutdown")\
m(CAUSE_1TR6_ICRV         ,=0x01, "invalid call reference value")\
m(CAUSE_1TR6_BSNI         ,=0x03, "bearer service not implemented")\
m(CAUSE_1TR6_CIDNE        ,=0x07, "call identity does not exist")\
m(CAUSE_1TR6_CIIU         ,=0x08, "call identity in use")\
m(CAUSE_1TR6_NCA          ,=0x0A, "no channel available")\
m(CAUSE_1TR6_RFNI         ,=0x10, "requested facility not implemented")\
m(CAUSE_1TR6_RFNS         ,=0x11, "requested facility not subscribed")\
m(CAUSE_1TR6_OCB          ,=0x20, "outgoing calls barred")\
m(CAUSE_1TR6_UAB          ,=0x21, "user access busy")\
m(CAUSE_1TR6_NECUG        ,=0x22, "non existent CUG")\
m(CAUSE_1TR6_NECUG1       ,=0x23, "non existent CUG")\
m(CAUSE_1TR6_SPV          ,=0x25, "kommunikationsbeziehung als SPV nicht erlaubt")\
m(CAUSE_1TR6_DNO          ,=0x35, "destination not obtainable")\
m(CAUSE_1TR6_NC           ,=0x38, "number changed")\
m(CAUSE_1TR6_OOO          ,=0x39, "out of order")\
m(CAUSE_1TR6_NUR          ,=0x3A, "no user responding")\
m(CAUSE_1TR6_UB           ,=0x3B, "user busy")\
m(CAUSE_1TR6_ICB          ,=0x3D, "incoming calls barred")\
m(CAUSE_1TR6_CR           ,=0x3E, "call rejected")\
m(CAUSE_1TR6_NCO          ,=0x59, "network congestion")\
m(CAUSE_1TR6_RUI          ,=0x5A, "remote user initiated")\
m(CAUSE_1TR6_LPE          ,=0x70, "local procedure error")\
m(CAUSE_1TR6_RPE          ,=0x71, "remote procedure error")\
m(CAUSE_1TR6_RUS          ,=0x72, "remote user suspended")\
m(CAUSE_1TR6_RUR          ,=0x73, "remote user resumed")\
m(CAUSE_1TR6_UIDL         ,=0x7F, "user info discarded locally")\

MAKE_ENUM(ITR6_CAUSES);

#endif /*_I4B_CAUSE_H_*/
