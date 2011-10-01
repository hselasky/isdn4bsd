/*-
 * Copyright (c) 1997, 2000 Hellmuth Michaelis. All rights reserved.
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
 *	facility.c - decode Q.932 facilities
 *	------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndecode/facility.c,v 1.7 2000/10/09 14:22:41 hm Exp $
 *
 *---------------------------------------------------------------------------
 *
 *	- Q.932 (03/93) Generic Procedures for the Control of
 *		ISDN Supplementaty Services
 *	- Q.950 (03/93) Supplementary Services Protocols, Structure and
 *		General Principles
 *	- ETS 300 179 (10/92) Advice Of Charge: charging information during
 *		the call (AOC-D) supplementary service Service description
 *	- ETS 300 180 (10/92) Advice Of Charge: charging information at the
 *		end of call (AOC-E) supplementary service Service description
 *	- ETS 300 181 (04/93) Advice Of Charge (AOC) supplementary service
 *		Functional capabilities and information flows
 *	- ETS 300 182 (04/93) Advice Of Charge (AOC) supplementary service
 *		Digital Subscriber Signalling System No. one (DSS1) protocol
 *	- X.208 Specification of Abstract Syntax Notation One (ASN.1)
 *	- X.209 Specification of Basic Encoding Rules for
 *		Abstract Syntax Notation One (ASN.1) 
 *	- "ASN.1 Abstract Syntax Notation One", Walter Gora, DATACOM-Verlag
 *		1992, 3rd Edition (ISBN 3-89238-062-7) (german !)
 *
 *---------------------------------------------------------------------------*/

#include "decode.h"
#include "facility.h"

static void do_component(struct buffer *dst, struct buffer *src);
static const char *uni_str(int code);
static const char *opval_str(int val);
static const char *bid_str(int val);
static void next_state(struct buffer *dst, struct buffer *src, 
		       u_int8_t class, u_int8_t form, u_int8_t code, int val);

/*---------------------------------------------------------------------------*
 *	decode Q.931/Q.932 facility info element
 *---------------------------------------------------------------------------*/
void
q932_facility(struct buffer *dst, struct buffer *src)
{
	bsprintf(dst, "[facility (Q.932): ");

	src->offset += 2;

	bsprintf(dst, "Protocol=");
	
	switch(get_1(src,0) & 0x1f) {
	case FAC_PROTO_ROP:
	    bsprintf(dst, "Remote Operations Protocol\n");
	    break;

	case FAC_PROTO_CMIP:
	    bsprintf(dst, "CMIP Protocol (Q.941), "
		     "UNSUPPORTED!]\n");
	    return;

	case FAC_PROTO_ACSE:
	    bsprintf(dst, "ACSE Protocol (X.217/X.227), "
		     "UNSUPPORTED!]\n");
	    return;

	default:
	    bsprintf(dst, "Unknown Protocol (val = 0x%x), "
		     "UNSUPPORTED!]\n", get_1(src,0) & 0x1f);
	    return;
	}

	/* next byte */

	src->offset++;

	/* initialize variables for do_component */

	src->state = ST_EXP_COMP_TYP;	

	/* decode facility */
	
	do_component(dst, src);

	/* replace last newline, if any */
	if(get_1(dst,dst->offset-1) == '\n')
	{
	   dst->offset--;
	}

	bsprintf(dst, "]");

	return;
}

/*---------------------------------------------------------------------------*
 *	handle a component recursively
 *---------------------------------------------------------------------------*/
static void
do_component(struct buffer *dst, struct buffer *src)
{
	u_int8_t  comp_tag_class;	/* component tag class */
	u_int8_t  comp_tag_form;	/* component form: constructor or primitive */
	u_int8_t  comp_tag_code;	/* component code depending on class */
	u_int16_t comp_length;		/* component length */
	u_int16_t i;
	u_int8_t  j;
	u_int8_t  shift;

	while(get_valid(src,0))
	{	
	    /*----------------------------------------*
	     * first component element: component tag *
	     *----------------------------------------*/
	
	    /* tag class bits */

	    j = get_1(src,0);

	    bsprintf(dst, "\t0x%02x Tag: ", j);	

	    comp_tag_class = (j & 0xc0) >> 6;
	
	    switch(comp_tag_class)
	    {
		case FAC_TAGCLASS_UNI:
			bsprintf(dst, "Universal");
			break;
		case FAC_TAGCLASS_APW:
			bsprintf(dst, "Applic-wide");
			break;
		case FAC_TAGCLASS_COS:
			bsprintf(dst, "Context-spec");
			break;
		case FAC_TAGCLASS_PRU:
			bsprintf(dst, "Private");
			break;
	    }

	    /* tag form bit */

	    comp_tag_form = (j & 0x20) >> 5;
	
	    bsprintf(dst, ", ");

	    if(comp_tag_form == FAC_TAGFORM_CON)
	    {
		bsprintf(dst, "Constructor");
	    }
	    else
	    {
		bsprintf(dst, "Primitive");
	    }

	    /* tag code bits */

	    comp_tag_code = j & 0x1f;
	
	    bsprintf(dst, ", ");	

	    if(comp_tag_code == 0x1f)
	    {
		comp_tag_code = 0;
		shift = 0;
		
		src->offset++;
		j = get_1(src,0);

		while(j & 0x80)
		{
			comp_tag_code |= (j & 0x7f) << shift;
			shift += 7;
			src->offset++;
			j = get_1(src,0);
		}
		comp_tag_code |= (j & 0x7f) << shift;
		bsprintf(dst, "%d (ext)\n", comp_tag_code);
	    }
	    else
	    {
		if(comp_tag_class == FAC_TAGCLASS_UNI)
		{
			bsprintf(dst, "%s (%d)\n", uni_str(comp_tag_code), comp_tag_code);
		}
		else 
		{
			bsprintf(dst, "code = %d\n", comp_tag_code);
		}
	    }

	    src->offset++;
	
	    /*--------------------------------------------*
	     * second component element: component length *
	     *--------------------------------------------*/

	    j = get_1(src,0);
	
	    bsprintf(dst, "\t0x%02x Len: ", j);

	    comp_length = 0;
	
	    if(j & 0x80)
	    {
		i = j & 0x7f;
		shift = 0;
		comp_length = 0;

		while(i--)
		{
			src->offset++;
			comp_length |= get_1(src,0) << shift;
			shift += 8;
		}
		bsprintf(dst, "%d (long form)\n", comp_length);
	    }
	    else
	    {
		comp_length = j & 0x7f;
		bsprintf(dst, "%d (short form)\n", comp_length);
	    }

	    next_state(dst, src, comp_tag_class, comp_tag_form, comp_tag_code, -1);

	    src->offset++;
	
	    if(comp_length)
	    {

		/*---------------------------------------------*/
		/* third component element: component contents */
		/*---------------------------------------------*/
			
		if(comp_tag_form)	/* == constructor */
		{
			u_int16_t old_len;
			old_len = set_length(src, src->offset + comp_length);

			do_component(dst, src);

			src->offset = src->len;
			src->len = old_len;
		}
		else 
		{
			int val = 0;		
			if(comp_tag_class == FAC_TAGCLASS_UNI)
			{
				switch(comp_tag_code)
				{
					case FAC_CODEUNI_INT:
					case FAC_CODEUNI_ENUM:
					case FAC_CODEUNI_BOOL:
						if(comp_length)
						{
							bsprintf(dst, "\t");
							shift = 0;
							i = comp_length;
							while(i--)
							{
							    j = get_1(src,0);
							    bsprintf(dst, "0x%02x ", j);
							    val |= j << shift;
							    src->offset++;
							    shift += 8;
							}
							bsprintf(dst, "\n\t");

							bsprintf(dst, "Val: %d\n", val);
						}
						break;
					default:	
						if(comp_length)
						{
							bsprintf(dst, "\t");
							
							i = comp_length;
							while(i--)
							{
							    j = get_1(src,0);

							    bsprintf(dst, "0x%02x = %d (%c)", j, j, 
								     isprint(j) ? j : '.');
							    src->offset++;
							}
							bsprintf(dst, "\n");

						}
						break;
				}
			}
	
			else	/* comp_tag_class != FAC_TAGCLASS_UNI */
			{
				if(comp_length)
				{
					bsprintf(dst, "\t");

					i = comp_length;
					shift = 0;
					while(i--)
					{
						j = get_1(src,0);

						bsprintf(dst, "0x%02x ", j);
						val |= j << shift;
						src->offset++;
						shift += 8;
					}
					bsprintf(dst, "\n\t" "Val: %d\n", val);
				}
			}
			next_state(dst, src, comp_tag_class, comp_tag_form, comp_tag_code, val);
		}
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	print universal id type
 *---------------------------------------------------------------------------*/
static const char *
uni_str(int code)
{
	static const char *tbl[] = {
		"BOOLEAN",
		"INTEGER",
		"BIT STRING",
		"OCTET STRING",
		"NULL",
		"OBJECT IDENTIFIER",
		"OBJECT DESCRIPTOR",
		"EXTERNAL",
		"REAL",
		"ENUMERATED",
		"RESERVED11",
		"RESERVED12",
		"RESERVED13",
		"RESERVED14",
		"RESERVED15",
		"SEQUENCE",
		"SET",
		"NUMERIC STRING",
		"PRINTABLE STRING",
		"TELETEX STRING",
		"ISO646 STRING",
		"IA5 STRING",
		"GRAPHIC STRING",
		"GENERAL STRING"
	};

	if((code >= 1) && (code <= FAC_CODEUNI_GNSTR))
		return(tbl[code-1]);
	else
		return("ERROR, Value out of Range!");
}

/*---------------------------------------------------------------------------*
 *	print operation value 
 *---------------------------------------------------------------------------*/
static const char *
opval_str(int val)
{
	static char buffer[80];
	const char *r;
	
	switch(val)
	{
		case FAC_OPVAL_UUS:
			r = "uUs";
			break;
		case FAC_OPVAL_CUG:
			r = "cUGCall";
			break;
		case FAC_OPVAL_MCID:
			r = "mCIDRequest";
			break;
		case FAC_OPVAL_BTPY:
			r = "beginTPY";
			break;
		case FAC_OPVAL_ETPY:
			r = "endTPY";
			break;
		case FAC_OPVAL_ECT:
			r = "eCTRequest";
			break;
		case FAC_OPVAL_DIV_ACT:
			r = "activationDiversion";
			break;
		case FAC_OPVAL_DIV_DEACT:
			r = "deactivationDiversion";
			break;
		case FAC_OPVAL_DIV_ACTSN:
			r = "activationStatusNotificationDiv";
			break;
		case FAC_OPVAL_DIV_DEACTSN:
			r = "deactivationStatusNotificationDiv";
			break;
		case FAC_OPVAL_DIV_INTER:
			r = "interrogationDiversion";
			break;
		case FAC_OPVAL_DIV_INFO:
			r = "diversionInformation";
			break;
		case FAC_OPVAL_DIV_CALLDEF:
			r = "callDeflection";
			break;
		case FAC_OPVAL_DIV_CALLRER:
			r = "callRerouting";
			break;
		case FAC_OPVAL_DIV_LINF2:
			r = "divertingLegInformation2";
			break;
		case FAC_OPVAL_DIV_INVS:
			r = "invokeStatus";
			break;
		case FAC_OPVAL_DIV_INTER1:
			r = "interrogationDiversion1";
			break;
		case FAC_OPVAL_DIV_LINF1:
			r = "divertingLegInformation1";
			break;
		case FAC_OPVAL_DIV_LINF3:
			r = "divertingLegInformation3";
			break;
		case FAC_OPVAL_ER_CRCO:
			r = "explicitReservationCreationControl";
			break;
		case FAC_OPVAL_ER_MGMT:
			r = "explicitReservationManagement";
			break;
		case FAC_OPVAL_ER_CANC:
			r = "explicitReservationCancel";
			break;
		case FAC_OPVAL_MLPP_QUERY:
			r = "mLPP lfb Query";
			break;
		case FAC_OPVAL_MLPP_CALLR:
			r = "mLPP Call Request";
			break;
		case FAC_OPVAL_MLPP_CALLP:
			r = "mLPP Call Preemption";
			break;
		case FAC_OPVAL_AOC_REQ:
			r = "chargingRequest";
			break;
		case FAC_OPVAL_AOC_S_CUR:
			r = "aOCSCurrency";
			break;
		case FAC_OPVAL_AOC_S_SPC:
			r = "aOCSSpecialArrangement";
			break;
		case FAC_OPVAL_AOC_D_CUR:
			r = "aOCDCurrency";
			break;
		case FAC_OPVAL_AOC_D_UNIT:
			r = "aOCDChargingUnit";
			break;
		case FAC_OPVAL_AOC_E_CUR:
			r = "aOCECurrency";
			break;
		case FAC_OPVAL_AOC_E_UNIT:
			r = "aOCEChargingUnit";
			break;
		case FAC_OPVAL_AOC_IDOFCRG:
			r = "identificationOfCharge";
			break;
		case FAC_OPVAL_CONF_BEG:
			r = "beginConf";
			break;
		case FAC_OPVAL_CONF_ADD:
			r = "addConf";
			break;
		case FAC_OPVAL_CONF_SPLIT:
			r = "splitConf";
			break;
		case FAC_OPVAL_CONF_DROP:
			r = "dropConf";
			break;
		case FAC_OPVAL_CONF_ISOLATE:
			r = "isolateConf";
			break;
		case FAC_OPVAL_CONF_REATT:
			r = "reattachConf";
			break;
		case FAC_OPVAL_CONF_PDISC:
			r = "partyDISC";
			break;
		case FAC_OPVAL_CONF_FCONF:
			r = "floatConf";
			break;
		case FAC_OPVAL_CONF_END:
			r = "endConf";
			break;
		case FAC_OPVAL_CONF_IDCFE:
			r = "indentifyConferee";
			break;
		case FAC_OPVAL_REVC_REQ:
			r = "requestREV";
			break;
		default:
			snprintf(&buffer[0], sizeof(buffer), "unknown operation value %d!", val);
			r = &buffer[0];
	}
	return(r);
}

/*---------------------------------------------------------------------------*
 *	billing id string
 *---------------------------------------------------------------------------*/
static const char *
bid_str(int val)
{
	static char buffer[80];
	const char *r;
	
	switch(val)
	{
		case 0:
			r = "normalCharging";
			break;
		case 1:
			r = "reverseCharging";
			break;
		case 2:
			r = "creditCardCharging";
			break;
		case 3:
			r = "callForwardingUnconditional";
			break;
		case 4:
			r = "callForwardingBusy";
			break;
		case 5:
			r = "callForwardingNoReply";
			break;
		case 6:
			r = "callDeflection";
			break;
		case 7:
			r = "callTransfer";
			break;
		default:
			snprintf(&buffer[0], sizeof(buffer), 
				 "unknown billing-id value %d!", val);
			r = &buffer[0];
	}
	return(r);
}

/*---------------------------------------------------------------------------*
 *	invoke component
 *---------------------------------------------------------------------------*/
static void
F_1_1(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_1_1, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          invokeComponent\n");
		src->state = ST_EXP_INV_ID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result
 *---------------------------------------------------------------------------*/
static void
F_1_2(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_1_2, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          returnResult\n");
		src->state = ST_EXP_RR_INV_ID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return error
 *---------------------------------------------------------------------------*/
static void
F_1_3(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_1_3, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          returnError\n");
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject
 *---------------------------------------------------------------------------*/
static void
F_1_4(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_1_4, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          reject\n");
		src->state = ST_EXP_REJ_INV_ID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result: invoke id
 *---------------------------------------------------------------------------*/
static void
F_RJ2(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RJ2, val = %d\n", val);
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          InvokeIdentifier = %d\n", val);
		src->state = ST_EXP_REJ_OP_VAL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject, general problem
 *---------------------------------------------------------------------------*/
static void
F_RJ30(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RJ30, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          General problem\n");
	}
	else
	{
		switch(val)
		{
			case 0:
				bsprintf(dst, "\t          problem = unrecognized component\n");
				break;
			case 1:
				bsprintf(dst, "\t          problem = mistyped component\n");
				break;
			case 2:
				bsprintf(dst, "\t          problem = badly structured component\n");
				break;
			default:
				bsprintf(dst, "\t          problem = unknown problem code 0x%x\n", val);
				break;
		}
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject, invoke problem
 *---------------------------------------------------------------------------*/
static void
F_RJ31(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RJ31, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          Invoke problem\n");
	}
	else
	{
		switch(val)
		{
			case 0:
				bsprintf(dst, "\t          problem = duplicate invocation\n");
				break;
			case 1:
				bsprintf(dst, "\t          problem = unrecognized operation\n");
				break;
			case 2:
				bsprintf(dst, "\t          problem = mistyped argument\n");
				break;
			case 3:
				bsprintf(dst, "\t          problem = resource limitation\n");
				break;
			case 4:
				bsprintf(dst, "\t          problem = initiator releasing\n");
				break;
			case 5:
				bsprintf(dst, "\t          problem = unrecognized linked identifier\n");
				break;
			case 6:
				bsprintf(dst, "\t          problem = linked resonse unexpected\n");
				break;
			case 7:
				bsprintf(dst, "\t          problem = unexpected child operation\n");
				break;
			default:
				bsprintf(dst, "\t          problem = unknown problem code 0x%x\n", val);
				break;
		}
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject, return result problem
 *---------------------------------------------------------------------------*/
static void
F_RJ32(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RJ32, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          Return result problem\n");
	}
	else
	{
		switch(val)
		{
			case 0:
				bsprintf(dst, "\t          problem = unrecognized invocation\n");
				break;
			case 1:
				bsprintf(dst, "\t          problem = return response unexpected\n");
				break;
			case 2:
				bsprintf(dst, "\t          problem = mistyped result\n");
				break;
			default:
				bsprintf(dst, "\t          problem = unknown problem code 0x%x\n", val);
				break;
		}
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject, return error problem
 *---------------------------------------------------------------------------*/
static void
F_RJ33(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RJ33, val = %d\n", val);
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          Return error problem\n");
	}
	else
	{
		switch(val)
		{
			case 0:
				bsprintf(dst, "\t          problem = unrecognized invocation\n");
				break;
			case 1:
				bsprintf(dst, "\t          problem = error response unexpected\n");
				break;
			case 2:
				bsprintf(dst, "\t          problem = unrecognized error\n");
				break;
			case 3:
				bsprintf(dst, "\t          problem = unexpected error\n");
				break;
			case 4:
				bsprintf(dst, "\t          problem = mistyped parameter\n");
				break;
			default:
				bsprintf(dst, "\t          problem = unknown problem code 0x%x\n", val);
				break;
		}
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	invoke component: invoke id
 *---------------------------------------------------------------------------*/
static void
F_2(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_2, val = %d\n", val);
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          InvokeIdentifier = %d\n", val);		
		src->state = ST_EXP_OP_VAL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result: invoke id
 *---------------------------------------------------------------------------*/
static void
F_RR2(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RR2, val = %d\n", val);
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          InvokeIdentifier = %d\n", val);
		src->state = ST_EXP_RR_OP_VAL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	invoke component: operation value
 *---------------------------------------------------------------------------*/
static void
F_3(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_3, val = %d\n", val);
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          Operation Value = %s (%d)\n", opval_str(val), val);
		src->state = ST_EXP_INFO;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result: operation value
 *---------------------------------------------------------------------------*/
static void
F_RR3(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RR3, val = %d\n", val);
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          Operation Value = %s (%d)\n", opval_str(val), val);
		src->state = ST_EXP_RR_RESULT;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result: RESULT
 *---------------------------------------------------------------------------*/
static void
F_RRR(struct buffer *dst, struct buffer *src, int val)
{
	(void)val;
	(void)dst;

#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_RRR, val = %d\n", val);
#endif
	src->state = ST_EXP_NIX;
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_4(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_4, val = %d\n", val);	
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          specificChargingUnits\n");
		src->state = ST_EXP_RUL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_4_1(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_4_1, val = %d\n", val);	
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          freeOfCharge\n");
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_4_2(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_4_2, val = %d\n", val);	
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          chargeNotAvailable\n");
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_5(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_5, val = %d\n", val);	
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          recordedUnitsList [1]\n");
		src->state = ST_EXP_RU;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_6(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_6, val = %d\n", val);	
#endif
	if(val == -1)
	{
		bsprintf(dst, "\t          RecordedUnits\n");
		src->state = ST_EXP_RNOU;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_7(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_7, val = %d\n", val);	
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          NumberOfUnits = %d\n", val);
		src->state = ST_EXP_TOCI;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_8(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_8, val = %d\n", val);	
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          typeOfChargingInfo = %s\n", val == 0 ? "subTotal" : "total");
		src->state = ST_EXP_DBID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *
 *---------------------------------------------------------------------------*/
static void
F_9(struct buffer *dst, struct buffer *src, int val)
{
#ifdef ST_DEBUG
	bsprintf(dst, "next_state: exec F_9, val = %d\n", val);	
#endif
	if(val != -1)
	{
		bsprintf(dst, "\t          AOCDBillingId = %s (%d)\n", bid_str(val), val);
		src->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	state table
 *---------------------------------------------------------------------------*/
static struct statetab {
	u_int8_t currstate;		/* input: current state we are in */
	u_int8_t form;			/* input: current tag form */
	u_int8_t class;			/* input: current tag class */
	u_int8_t code;			/* input: current tag code */
	void (*func)(struct buffer *, struct buffer *, int);	/* output: func to exec */
} statetab[] = {

/*	 current state		tag form		tag class		tag code		function	*/
/*	 ---------------------  ----------------------  ----------------------  ---------------------- 	----------------*/

/* invoke */

	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	1,			F_1_1		},
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	2,			F_1_2		},
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	3,			F_1_3		},
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	4,			F_1_4		},
	{ST_EXP_INV_ID,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	F_2		},
	{ST_EXP_OP_VAL,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	F_3		},
	{ST_EXP_INFO,		FAC_TAGFORM_CON,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_SEQ,	F_4		},
	{ST_EXP_INFO,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_NULL,	F_4_1		},
	{ST_EXP_INFO,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	1,			F_4_2		},
	{ST_EXP_RUL,		FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	1,			F_5		},
	{ST_EXP_RU,		FAC_TAGFORM_CON,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_SEQ,	F_6		},
	{ST_EXP_RNOU,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	F_7		},
	{ST_EXP_TOCI,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	2,			F_8		},
	{ST_EXP_DBID,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	3,			F_9		},

/* return result */
	
	{ST_EXP_RR_INV_ID,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	F_RR2		},
	{ST_EXP_RR_OP_VAL,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	F_RR3		},
	{ST_EXP_RR_RESULT,	FAC_TAGFORM_CON,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_SET,	F_RRR		},

/*	 current state		tag form		tag class		tag code		function	*/
/*	 ---------------------  ----------------------  ----------------------  ---------------------- 	----------------*/
/* reject */
	
	{ST_EXP_REJ_INV_ID,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	F_RJ2		},
	{ST_EXP_REJ_OP_VAL,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	0,			F_RJ30		},
	{ST_EXP_REJ_OP_VAL,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	1,			F_RJ31		},
	{ST_EXP_REJ_OP_VAL,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	2,			F_RJ32		},
	{ST_EXP_REJ_OP_VAL,	FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	3,			F_RJ33		},

/* end */
	
	{ -1,			-1,			-1,			-1,			NULL		}
};	
	
/*---------------------------------------------------------------------------*
 *	state decode for do_component
 *---------------------------------------------------------------------------*/
static void
next_state(struct buffer *dst, struct buffer *src, 
	   u_int8_t class, u_int8_t form, u_int8_t code, int val)
{
	u_int8_t i;

#ifdef ST_DEBUG
	bsprintf(dst, "next_state: class=%d, form=%d, code=%d, val=%d\n", 
		 class, form, code, val);
#endif

	for(i=0; ; i++)
	{
		if((statetab[i].currstate > src->state) ||
		   (statetab[i].func == NULL))
		{
			break;
		}

		if((statetab[i].currstate == src->state) &&
		   (statetab[i].form == form)		 &&
		   (statetab[i].class == class)		 &&
		   (statetab[i].code == code))
		{
			(*statetab[i].func)(dst, src, val);
			break;
		}
	}
	return;
}
