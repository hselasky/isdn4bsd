/*-
 * Copyright (c) 1997, 2000 Hellmuth Michaelis. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *	layer3_subr.c - subroutines for IE decoding
 *	-------------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndecode/layer3_subr.c,v 1.8 2001/04/26 08:40:35 kris Exp $
 *
 * NOTE: all indexes must be 16-bit
 *---------------------------------------------------------------------------*/

#include "decode.h"

static const char *
get_standard(u_int8_t j)
{
  return
    (j == 0) ? "CCITT" :
    (j == 1) ? "ISO/IEC" :
    (j == 2) ? "National" : 
    (j == 3) ? "Special/Local" : 
    "Unknown";
}

static const char *
get_state(u_int8_t j)
{
  return
    (j == 0) ? "Null" :
    (j == 1) ? "Call initiated" :
    (j == 2) ? "Overlap sending" :
    (j == 3) ? "Outgoing call proceeding" :
    (j == 4) ? "Call delivered" :
    (j == 6) ? "Call present" :
    (j == 7) ? "Call received" :
    (j == 8) ? "Connect request" :
    (j == 9) ? "Incoming call proceeding" :
    (j == 10) ? "Active" :
    (j == 11) ? "Disconnect request" :
    (j == 12) ? "Disconnect indication" :
    (j == 15) ? "Suspend request" :
    (j == 17) ? "Resume request" :
    (j == 19) ? "Release request" :
    (j == 22) ? "Call abort" :
    (j == 25) ? "Overlap receiving" :
    (j == 0x3d) ? "Restart request" :
    (j == 0x3e) ? "Restart" :
    "Unknown";
}

static const char *
get_location(u_int8_t j)
{
  return 
    (j == 0) ? "User" :
    (j == 1) ? "Private network serving local user" :
    (j == 2) ? "Public network serving local user" :
    (j == 3) ? "Transit network" :
    (j == 4) ? "Public network serving remote user" :
    (j == 5) ? "Private network serving remote user" :
    (j == 6) ? "Network beyond interworking point" :
    "ERROR: undefined/reserved";
}

static const char *
get_endpoint_desc(u_int8_t j)
{
  return
    (j == 1) ? "Call is not end-to-end ISDN" :
    (j == 2) ? "Destination address is non-ISDN" :
    (j == 3) ? "Origination address is non-ISDN" :
    (j == 4) ? "Call has returned to the ISDN" :
    (j == 5) ? "Interworking occured, Service change" :
    (j == 8) ? "In-band info or appropriate pattern now available" :
    "ERROR: undefined/reserved";
}

static const char *
get_recommendation(u_int8_t j)
{
  return
    (j == 0) ? "Q.931" :
    (j == 3) ? "X.21" :
    (j == 4) ? "X.25" :
    (j == 5) ? "Q.1031/Q.1051" :
    "Reserved";
}

static const char *
get_bearer_cap(u_int8_t j)
{
  return
    (j == 0x00) ? "speech" :
    (j == 0x08) ? "unrestricted digital information" :
    (j == 0x09) ? "restricted digital information" :
    (j == 0x10) ? "3.1 kHz audio" :
    (j == 0x11) ? "unrestricted digital information with tones" :
    (j == 0x18) ? "video" :
    "reserved";
}

static const char *
get_line_mode(u_int8_t j)
{
  return
    (j == 0) ? "circuit" :
    (j == 2) ? "packet" :
    "reserved";
}

static const char *
get_layer1_prot(u_int8_t j)
{
  return
    (j == 0x01) ? "V.110/X.30" :
    (j == 0x02) ? "G.711 u-Law" :
    (j == 0x03) ? "G.711 a-Law" :
    (j == 0x04) ? "G.721 ADPCM/I.460" :
    (j == 0x05) ? "H.221/H.242" :
    (j == 0x07) ? "non-CCITT rate adaption" :
    (j == 0x08) ? "V.120" :
    (j == 0x09) ? "X.31" :
    "reserved";
}

static const char *
get_q850_cause(u_int8_t code)
{
    static const char * const
      MAKE_TABLE(Q850_CAUSES,DESC,[]);
    const char *temp;

    if(code < INDEXES(Q850_CAUSES_DESC))
    {
        temp = Q850_CAUSES_DESC[code];
    }
    else
    {
        temp = NULL;
    }

    if(temp == NULL)
    {
        temp = "ERROR, unknown cause value!";
    }
    return temp;
}

/*---------------------------------------------------------------------------*
 *	dump remainder of I-frame
 *---------------------------------------------------------------------------*/
static void
dump_remainder(struct buffer *dst, struct buffer *src, u_int16_t i)
{
	u_int16_t off = src->offset;
	u_int8_t j;
	while(get_valid(src,i))
	{
	    j = get_1(src,i);

	    bsprintline(3, dst, off+i, j, 0xff, "Unknown byte: 0x%02x, %c", 
			j, isprint(j) ? j : '?');
	    i++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	show status information
 *---------------------------------------------------------------------------*/
static void
f_cstat(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	u_int8_t  j;

	j = get_1(src,2) & 0x60;

	bsprintline(3, dst, off+2, j, 0x60, "Coding standard = %s", 
		    get_standard(j >> 5));

	j = get_1(src,2) & 0x1f;

	bsprintline(3, dst, off+2, j, 0x1f, "state = %s", 
		    get_state(j));

	return;
}

/*---------------------------------------------------------------------------*
 *	decode keypad IE
 *---------------------------------------------------------------------------*/
static void
f_keypad(struct buffer *dst, struct buffer *src)
{
	u_int8_t  j;

	while(get_valid(src,2))
	{
	    j = get_1(src,2);

	    bsprintline(3, dst, src->offset+2, j, 0xff, "data = 0x%02x, %c",
			j, isprint(j) ? j : '.');

	    src->offset ++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode notification IE (Q.932, p44)
 *---------------------------------------------------------------------------*/
static void
f_notify(struct buffer *dst, struct buffer *src)
{
	const u_int8_t *temp;
	u_int16_t off = src->offset;
	u_int8_t  j;

	j = get_1(src,2);

	temp =
	  (j == 0x80) ? "user suspended" :
	  (j == 0x81) ? "user resumed" :
	  (j == 0x82) ? "bearer service changed" :

	  (j == 0x83) ? "BER coded information" :

	  (j == 0xc2) ? "conference established" :
	  (j == 0xc3) ? "conference disconnected" :
	  (j == 0xc4) ? "other party added" :
	  (j == 0xc5) ? "isolated" :
	  (j == 0xc6) ? "reattached" :
	  (j == 0xc7) ? "other party isolated" :
	  (j == 0xc8) ? "other party reattached" :
	  (j == 0xc9) ? "other party split" :
	  (j == 0xca) ? "other party disconnected" :
	  (j == 0xcb) ? "conference floating" :
	  (j == 0xcc) ? "conference disconnected, preemption" :
	  (j == 0xcf) ? "conference floating, server user preempted" :

	  (j == 0xe0) ? "call is a waiting call" :
	  (j == 0xe8) ? "diversion activated" :
	  (j == 0xe9) ? "call transferred, alerting" :
	  (j == 0xea) ? "call transferred, active" :
	  (j == 0xee) ? "reverse charging" :

	  (j == 0xf9) ? "remote hold" :
	  (j == 0xfa) ? "remote retrieval" :
	  (j == 0xfb) ? "call is diverting" :
	  "undefined";

	bsprintline(3, dst, off+2, j, 0xff, "Notification = %s (0x%02x)",
		    temp, j);

	while(get_valid(src,3))
	{
	    j = get_1(src,3);

	    bsprintline(3, dst, off+3, j, 0xff, "data = 0x%02x, %c",
			j, isprint(j) ? j : '.');

	    src->offset ++;
	    off ++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	channel identification
 *---------------------------------------------------------------------------*/
static void
f_chid(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	u_int8_t j = get_1(src,2);
 	u_int8_t p = j & 0x20;
	u_int8_t iid = j & 0x40;

	extension(3, dst, off+2, j, 0x80);

	bsprintline(3, dst, off+2, j, 0x40, "Interface Id present = %s", (j & 0x40) ? "Yes" : "No");

	bsprintline(3, dst, off+2, j, 0x20, "Interface Type = %s", (j & 0x20) ? "Other (PRI)" : "BRI");

	bsprintline(3, dst, off+2, j, 0x10, "Spare");

	bsprintline(3, dst, off+2, j, 0x08, "Channel is %s", (j & 0x08) ? "exclusive" : "preferred");

	bsprintline(3, dst, off+2, j, 0x04, "Channel is%s the D-Channel", (j & 0x04) ? "" : " not");

	j &= 0x3;

	bsprintline(3, dst, off+2, j, 0x03, "Channel = %s", 
		    (j == 0) ? "no channel" :
		    (j == 1) ? (p ? "B-X" : "B-1") :
		    (j == 2) ? (p ? "reserved" : "B-2") :
		    (j == 3) ? "any channel" :
		    "unknown");

	if(iid)
	{
	    j = get_1(src,3);

	    extension(3, dst, off+3, j, 0x80);

	    bsprintline(3, dst, off+3, j, 0x7F, "Interface identifier");

	    off++;
	    src->offset++;
	}

	if(p)
	{
	    /* primary rate */

	    j = get_1(src,3);

	    extension(3, dst, off+3, j, 0x80);

	    bsprintline(3, dst, off+3, j, 0x60, "Coding standard = %s", 
			get_standard((j & 0x60) >> 5));

	    bsprintline(3, dst, off+3, j, 0x20, "channel is indicated by the %s",
			(j & 0x20) ? "slot map" : "following octet");

	    j &= 0xf;

	    bsprintline(3, dst, off+3, j, 0x0F, "unit = %s",
			(j == 3) ? "B-channel" :
			(j == 6) ? "H0-channel" :
			(j == 8) ? "H11-channel" :
			(j == 9) ? "H12-channel" :
			"unknown");

	    j = get_1(src,4);

	    extension(3, dst, off+4, j, 0x80);

	    bsprintline(3, dst, off+4, j, 0x7F, "channel number = 0x%02x (time-slot)", j & 0x7F);
	}
	dump_remainder(dst,src,5);
	return;
}

/*---------------------------------------------------------------------------*
 *	facility
 *---------------------------------------------------------------------------*/
static void
f_fac(struct buffer *dst, struct buffer *src)
{
	q932_facility(dst, src);
	dump_remainder(dst,src,0);
	return;
}

/*---------------------------------------------------------------------------*
 *	progress indicator
 *---------------------------------------------------------------------------*/
static void
f_progi(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	u_int8_t j;

	j = get_1(src,2) & 0x60;

	bsprintline(3, dst, off+2, j, 0x60, "Coding standard = %s",
		    get_standard(j >> 5));

	j = get_1(src,2) & 0x0f;

	bsprintline(3, dst, off+2, j, 0x0f, "location = %s",
		    get_location(j));

	j = get_1(src,3) & 0x7f;

	bsprintline(3, dst, off+3, j, 0x7f, "description = %s",
		    get_endpoint_desc(j));

	dump_remainder(dst,src,4);
	return;
}

/*---------------------------------------------------------------------------*
 *	display string
 *---------------------------------------------------------------------------*/
static void
f_displ(struct buffer *dst, struct buffer *src)
{
	const u_int8_t *indent = "L3                ";
	u_int16_t i = 2;
	u_int8_t j;

	bsprintf(dst, "%sdisplay = ", indent);

	while(get_valid(src,i))
	{
	    j = get_1(src,i);

	    bsprintf(dst, "%c", isprint(j) ? j : '?');
	    i++;

	    if(((i % 32) == 2) && (get_valid(src,i)))
	      bsprintf(dst, "\n%s", indent);
	}

	bsprintf(dst,"\n");
	return;
}

/*---------------------------------------------------------------------------*
 *	date
 *---------------------------------------------------------------------------*/
static void
f_date(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	u_int8_t j;

	j = get_1(src,2);
	bsprintline(3, dst, off+2, j, 0xff, "Year = %02d", j);

	j = get_1(src,3);
	if(j)
	{
	  bsprintline(3, dst, off+3, j, 0xff, "Month = %02d", j);

	  j = get_1(src,4);
	  if(j)
	  {
	    bsprintline(3, dst, off+4, j, 0xff, "Day = %02d", j);

	    j = get_1(src,5);
	    if(j)
	    {
	      bsprintline(3, dst, off+5, j, 0xff, "Hour = %02d", j);

	      j = get_1(src,6);
	      bsprintline(3, dst, off+6, j, 0xff, "Minute = %02d", j);

	      j = get_1(src,7);
	      bsprintline(3, dst, off+7, j, 0xff, "Second = %02d", j);
	    }
	  }
	}

	dump_remainder(dst,src,8);
	return;
}

/*---------------------------------------------------------------------------*
 *	decode and print the cause
 *---------------------------------------------------------------------------*/
static void
f_cause(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	u_int16_t i = 2;
	u_int8_t j;

	j = get_1(src,i);

	extension(3, dst, off+i, j, 0x80);

	bsprintline(3, dst, off+i, j, 0x60, "Coding standard = %s",
		    get_standard((j & 0x60) >> 5));

	bsprintline(3, dst, off+i, j, 0x10, "Spare");

	bsprintline(3, dst, off+i, j, 0x0f, "Location = %s", 
		    get_location(j & 0x0f));
	
	i++;

	if(!(j & 0x80))
	{
	  j = get_1(src,i);

	  extension(3, dst, off+i, j, 0x80);

	  bsprintline(3, dst, off+i, j, 0x7f, 
		      "Recommendation = %s", 
		      get_recommendation(j & 0x7f));

	  i++;
	}

	j = get_1(src,i);

	extension(3, dst, off+i, j, 0x80);
	
	bsprintline(3, dst, off+i, j, 0x7f, 
		    "Cause = %d: Q.850: %s", 
		    (j & 0x7f), get_q850_cause(j & 0x7f));

	i++;
	while(get_valid(src,i))
	{
	    j = get_1(src,i);

	    bsprintline(3, dst, off+i, j, 0xff, 
			"Diagnostics = 0x%02x %c", j, isprint(j) ? j : '?');
	    i++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode and print the bearer capability
 *---------------------------------------------------------------------------*/
static void
f_bc(struct buffer *dst, struct buffer *src)
{
        const u_int8_t *temp;
	u_int16_t off = src->offset;
	u_int16_t i = 2;
	u_int8_t j;
	u_int8_t prot;

	j = get_1(src,2);

	extension(3, dst, off+i, j, 0x80);
	
	bsprintline(3, dst, off+i, j, 0x60, "Coding standard = %s", 
		    get_standard((j & 0x60) >> 5));

	j &= 0x1f;

	bsprintline(3, dst, off+i, j, 0x1f, "Capability = 0x%02x, %s",
		    j, get_bearer_cap(j));

	i++;

	extension(3, dst, off+i, get_1(src,i), 0x80);

	j = get_1(src,i) & 0x60;
	
	bsprintline(3, dst, off+i, j, 0x60, "Mode = 0x%02x, %s",
		    j, get_line_mode(j >> 5));
	
	j = get_1(src,i) & 0x1f;

	bsprintline(3, dst, off+i, get_1(src,i), 0x1f, "Rate = 0x%02x, %s", 
		    j, 
		    (j == 0x00) ? "packet mode" :
		    (j == 0x10) ? "64 kbit/s" :
		    (j == 0x11) ? "2 x 64 kbit/s" :
		    (j == 0x13) ? "384 kbit/s" :
		    (j == 0x15) ? "1536 kbit/s" :
		    (j == 0x17) ? "1920 kbit/s" :
		    (j == 0x18) ? "Multirate" :
		    "reserved");
	i++;

	if(j == 0x18)
	{
	   j = get_1(src,i);

	   extension(3, dst, off+i, j, 0x80);
	   bsprintline(3, dst, off+i, j, 0x7f, 
		       "Rate multiplier = %d", j & 0x7f);
	   i++;
	}

	if(get_valid(src,i))
	{
	    j = get_1(src,i);

	    extension(3, dst, off+1, j, 0x80);

	    j = get_1(src,i) & 0x60;

	    bsprintline(3, dst, off+i, j, 0x60, 
			"Layer 1 identity = %d", 
			(j / 0x20));

	    j = get_1(src,i) & 0x1f;
	    prot = j;

	    bsprintline(3, dst, off+i, j, 0x1f, 
			"Layer 1 Protocol = 0x%02x, %s", 
			j, get_layer1_prot(j));

	    i++;

	    if(!(get_1(src,i-1) & 0x80))
	    {
		j = get_1(src,i);

		bsprintline(3, dst, off+i, j, 0x40,
			    "%s", (j & 0x40) ? "async" : "sync");

		bsprintline(3, dst, off+i, j, 0x20,
			    "%s", (j & 0x20) ? 
			    "in-band negotiation possible" :
			    "in-band negotiation not possible");

	        j = get_1(src,i) & 0x1f;

		bsprintline(3, dst, off+i, j, 0x1f, 
			    "user rate = 0x%02x, %s", j,
			    (j == 0x01) ? "0.6 kbit/s" :
			    (j == 0x02) ? "1.2 kbit/s" :
			    (j == 0x03) ? "2.4 kbit/s" :
			    (j == 0x04) ? "3.6 kbit/s" :
			    (j == 0x05) ? "4.8 kbit/s" :
			    (j == 0x06) ? "7.2 kbit/s" :
			    (j == 0x07) ? "8 kbit/s" :
			    (j == 0x08) ? "9.6 kbit/s" :
			    (j == 0x09) ? "14.4 kbit/s" :
			    (j == 0x0a) ? "16 kbit/s" :
			    (j == 0x0b) ? "19.2 kbit/s" :
			    (j == 0x0c) ? "32 kbit/s" :
			    (j == 0x0e) ? "48 kbit/s" :
			    (j == 0x0f) ? "56 kbit/s" :
			    (j == 0x15) ? "0.1345 kbit/s" :
			    (j == 0x16) ? "0.100 kbit/s" :
			    (j == 0x17) ? "0.075/1.2 kbit/s" :
			    (j == 0x18) ? "1.2/0.075 kbit/s" :
			    (j == 0x19) ? "0.050 kbit/s" :
			    (j == 0x1a) ? "0.075 kbit/s" :
			    (j == 0x1b) ? "0.110 kbit/s" :
			    (j == 0x1c) ? "0.150 kbit/s" :
			    (j == 0x1d) ? "0.200 kbit/s" :
			    (j == 0x1e) ? "0.300 kbit/s" :
			    (j == 0x1f) ? "12 kbit/s" :
			    "reserved");
		i++;
	    }

	    if(!(get_1(src,i-1) & 0x80))
	    {
		if(prot == 1) /* v.110, X.30 */
		{
		    j = get_1(src,i) & 0x60;

		    bsprintline(3, dst, off+i, j, 0x60,
				"intermediate rate = %s",
				(j == 0x00) ? "not used" :
				(j == 0x20) ? "8 kbit/s" :
				(j == 0x40) ? "16 kbit/s" :
				(j == 0x60) ? "32 kbit/s" :
				"unknown");

		    j = get_1(src,i);

		    bsprintline(3, dst, off+i, j, 0x10,
				(j & 0x10) ?
				"Required to send data with network independent clock" :
				"Not required to send data with network independent clock");

		    bsprintline(3, dst, off+i, j, 0x08,
				(j & 0x08) ?
				"Can accept data with network independent clock" :
				"Cannot accept data with network independent clock" );

		    bsprintline(3, dst, off+i, j, 0x04,
				(j & 0x04) ?
				"Required to send data with flow control mechanism" :
				"Not required to send data with flow control mechanism");

		    bsprintline(3, dst, off+i, j, 0x02,
				(j & 0x02) ? 
				"Can accept data with flow control mechanism" :
				"Cannot accept data with flow control mechanism");
		}
		else if(prot == 0x08) /* v.120 */
		{
		    j = get_1(src,i);

		    bsprintline(3, dst, off+i, j, 0x40,
				(j & 0x40) ? "Rate adaption header included" :
				"Rate adaption header not included");

		    bsprintline(3, dst, off+i, j, 0x20,
				(j & 0x20) ? "Multiple frame establishment supported" :
				"Multiple frame establishment not supported. Only UI frames allowed.");

		    bsprintline(3, dst, off+i, j, 0x10,
				(j & 0x10) ? "Protocol sensitive mode of operation" :
				"Bit transparent mode of operation");

		    bsprintline(3, dst, off+i, j, 0x08,
				(j & 0x08) ? "Full protocol negotiation" :
				"Default, LLI = 256 only");

		    bsprintline(3, dst, off+i, j, 0x04,
				(j & 0x04) ? 
				"Message originator is \"Assignor only\"" :
				"Message originator is \"Default assignee\"");

		    bsprintline(3, dst, off+i, j, 0x02,
				(j & 0x02) ? 
				"Negotiation is done in-band using logical link zero" :
				"Negotiation is done with USER INFORMATION messages "
				"on a temporary signalling connection");

		}
		else
		{
		    j = get_1(src,i);

		    bsprintline(3, dst, off+i, j, 0xFF, 
				"Unknown byte: 0x%02x", j);
		}
		i++;
	    }

	    if(!(get_1(src,i-1) & 0x80))
	    {
	        j = get_1(src,i) & 0x60;

		bsprintline(3, dst, off+i, j, 0x60,
			    "number of stop bits = %s",
			    (j == 0x00) ? "Not used" :
			    (j == 0x01) ? "1 bit" :
			    (j == 0x02) ? "5 bits" :
			    (j == 0x03) ? "2 bits" :
			    "unknown");

	        j = get_1(src,i) & 0x18;

		bsprintline(3, dst, off+i, j, 0x18,
			    "number of data bits = %s",
			    (j == 0x00) ? "Not used" :
			    (j == 0x01) ? "5 bits" :
			    (j == 0x02) ? "7 bits" :
			    (j == 0x03) ? "8 bits" :
			    "unknown");

	        j = get_1(src,i) & 0x07;

		bsprintline(3, dst, off+i, j, 0x07,
			    "parity = %s",
			    (j == 0x00) ? "Odd" :
			    (j == 0x02) ? "Even" :
			    (j == 0x03) ? "None" :
			    (j == 0x04) ? "Forced to 0" :
			    (j == 0x05) ? "Forced to 1" :
			    "unknown");
		i++;
	    }

	    if(!(get_1(src,i-1) & 0x80))
	    {
	        j = get_1(src,i) & 0x3f;

		bsprintline(3, dst, off+i, j, 0x3f,
			    "modem type = %s",
			    (j == 17) ? "V.21" :
			    (j == 18) ? "V.22" :
			    (j == 19) ? "V.22 bis" :
			    (j == 20) ? "V.23" :
			    (j == 21) ? "V.26" :
			    (j == 22) ? "V.26 bis" :
			    (j == 23) ? "V.26 ter" :
			    (j == 24) ? "V.27" :
			    (j == 25) ? "V.27 bis" :
			    (j == 26) ? "V.27 ter" :
			    (j == 27) ? "V.29" :
			    (j == 29) ? "V.32" :
			    "unknown");
		i++;
	    }

	    /* find the last byte */

	    while(get_valid(src,i) && 
		  (!(get_1(src,i-1) & 0x80)))
	    {
	        j = get_1(src,i);

		bsprintline(3, dst, off+i, j, 0xFF, "Unknown byte: 0x%02x", j);

	        i++;
	    }
	}

	if(get_valid(src,i))
	{
	    j = get_1(src,i) & 0x7f;

	    temp = 
	      (j == 0x42) ? "Q.921/I.441" :
	      (j == 0x46) ? "X.25 link" :
	      "unknown";

	    bsprintline(3, dst, off+i, j, 0x7f,
			"layer2 = %s (0x%02x)", temp, j);
	    i++;
	}

	if(get_valid(src,i))
	{
	    j = get_1(src,i) & 0x7f;

	    temp =
	      (j == 0x62) ? "Q.921/I.441" :
	      (j == 0x66) ? "X.25 packet" :
	      "unknown";

	    bsprintline(3, dst, off+i, j, 0x7f,
			"layer3 = %s (0x%02x)", temp, j);
	    i++;
	}

	dump_remainder(dst,src,i);
	return;
}

/*---------------------------------------------------------------------------*
 *	decode and print the ISDN (telephone) number
 *---------------------------------------------------------------------------*/
static void
f_cnu(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	const char *temp;
	u_int16_t i = 2;
	u_int8_t j;

	j = get_1(src,i);

	extension(3, dst, off+i, j, 0x80);

	j = (get_1(src,i) & 0x70) >> 4;

	temp = 
	  (j == 0) ? "Unknown" :
	  (j == 1) ? "International number" :
	  (j == 2) ? "National number" :
	  (j == 3) ? "Network specific number" :
	  (j == 4) ? "Subscriber number" :
	  (j == 6) ? "Abbreviated number" :
	  "Reserved (7)";

	bsprintline(3, dst, off+i, get_1(src,i), 0x70, 
		    "Type = %s", temp);

	j = (get_1(src,i) & 0x0f);

	temp = 
	  (j == 0) ? "Unknown" :
	  (j == 1) ? "ISDN (E.164)" :
	  (j == 3) ? "Data (X.121)" :
	  (j == 4) ? "Telex (F.69)" :
	  (j == 8) ? "National" :
	  (j == 9) ? "Private" :
	  "Reserved";

	bsprintline(3, dst, off+i, get_1(src,i), 0x0f, 
		    "Plan = %s (0x%x)", temp, j);

	i++;
	
	if(!(get_1(src,i-1) & 0x80))
	{
	    extension(3, dst, off+i, get_1(src,i), 0x80);

	    j = (get_1(src,i) & 0x60) >> 5;
	    
	    temp =
	      (j == 0) ? "allowed" :
	      (j == 1) ? "restricted" :
	      (j == 2) ? "number not available" :
	      (j == 3) ? "reserved" :
	      "unknown";

	    bsprintline(3, dst, off+i, get_1(src,i), 0x60, 
			"Presentation = %s", temp);

	    bsprintline(3, dst, off+i, get_1(src,i), 0x1c, "Spare");

	    j = (get_1(src,i) & 0x03);

	    temp = 
	      (j == 0) ? "user provided, not screened" :
	      (j == 1) ? "user provided, verified & passed" :
	      (j == 2) ? "user provided, verified & failed" :
	      (j == 3) ? "network provided" :
	      "unknown";

	    bsprintline(3, dst, off+i, j, 0x03, 
			"Screening = %s", temp);
	    i++;
	}

	if(!(get_1(src,i-1) & 0x80))
	{
	    extension(3, dst, off+i, get_1(src,i), 0x80);

	    j = get_1(src,i) & 0x0f;

	    temp =
	      (j == 0x0) ? "unknown" :
	      (j == 0x1) ? "call forwarding busy" :
	      (j == 0x2) ? "call forwarding unconditional" :
	      (j == 0xa) ? "called DTE" :
	      (j == 0xf) ? "call forwarding unconditional" :
	      "unknown";

	    bsprintline(3, dst, off+i, j, 0x0f,
			"Reason for diversion = %s (0x%02x)", 
			temp, j);
	    i++;
	}

	while(get_valid(src,i))
	{
	    j = get_1(src,i);

	    bsprintline(3, dst, off+i, j, 0xff, 
			"Number digit = %c", isprint(j) ? j : '?');
	    i++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode and print HL comatibility
 *---------------------------------------------------------------------------*/
static void
f_hlc(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	const char *temp;
	u_int16_t i = 2;
	u_int8_t j;

	j = get_1(src,i);

	extension(3, dst, off+i, j, 0x80);
	
	bsprintline(3, dst, off+i, j, 0x60, 
		    "Coding standard = %s", 
		    get_standard((j >> 5) & 0x03));

	bsprintline(3, dst, off+i, j, 0x1c, "Interpretation = %s", 
		    (((j >> 2) & 0x07) == 0x04) ? "first" : "reserved");

	bsprintline(3, dst, off+i, j, 0x03, "Presentation = %s", 
		    ((j & 0x03) == 0x01) ? "High layer protocol profile" : "reserved");

	i++;

	j = get_1(src,i);

	extension(3, dst, off+i, j, 0x80);

	j &= 0x7f;

	temp =
	  (j == 0x01) ? "Telephony" :
	  (j == 0x04) ? "Fax Group 2/3 (F.182)" :
	  (j == 0x21) ? "Fax Group 4 I (F.184)" :
	  (j == 0x24) ? "Teletex (F.230) or Fax Group 4 II/III (F.184)" :
	  (j == 0x28) ? "Teletex (F.220)" :
	  (j == 0x31) ? "Teletex (F.200)" :
	  (j == 0x32) ? "Videotex (F.300/T.102)" :
	  (j == 0x33) ? "Videotex (F.300/T.101)" :
	  (j == 0x35) ? "Telex (F.60)" :
	  (j == 0x38) ? "MHS (X.400)" :
	  (j == 0x41) ? "OSI (X.200)" :
	  (j == 0x5e) ? "Maintenance" :
	  (j == 0x5f) ? "Management" :
	  (j == 0x60) ? "Audio visual (F.721)" :
	  "Reserved";

	bsprintline(3, dst, off+i, j, 0x7f, 
		    "Characteristics = %s (0x%02x)", temp, j);
	i++;

	if(!(get_1(src,i-1) & 0x80))
	{
	    j = get_1(src,i);

	    extension(3, dst, off+i, j, 0x80);

	    j &= 0x7f;

	    temp =
	      (j == 0x01) ? "Telephony" :
	      (j == 0x04) ? "Fax Group 2/3 (F.182)" :
	      (j == 0x21) ? "Fax Group 4 I (F.184)" :
	      (j == 0x24) ? "Teletex (F.230) or Fax Group 4 II/III (F.184)" :
	      (j == 0x28) ? "Teletex (F.220)" :
	      (j == 0x31) ? "Teletex (F.200)" :
	      (j == 0x32) ? "Videotex (F.300/T.102)" :
	      (j == 0x33) ? "Videotex (F.300/T.101)" :
	      (j == 0x35) ? "Telex (F.60)" :
	      (j == 0x38) ? "MHS (X.400)" :
	      (j == 0x41) ? "OSI (X.200)" :
	      (j == 0x5e) ? "Maintenance" :
	      (j == 0x5f) ? "Management" :
	      (j == 0x60) ? "Audio visual (F.721)" :
	      "Reserved";

	    bsprintline(3, dst, off+i, j, 0x7f, 
			"Ext. characteristics = %s (0x%02x)", temp, j);

	    i++;
	}

	dump_remainder(dst,src,i);
	return;
}

/*---------------------------------------------------------------------------*
 *	user-user message
 *---------------------------------------------------------------------------*/
static void
f_uu(struct buffer *dst, struct buffer *src)
{
	const char *temp;
	u_int16_t off = src->offset;
	u_int16_t i = 2;
	u_int8_t j;
	
	j = get_1(src,2);

	temp = 
	  (j == 0) ? "user-specific" :
	  (j == 1) ? "OSI high layer" :
	  (j == 2) ? "X.244" :
	  (j == 3) ? "reserved for sys mgmt" :
	  (j == 4) ? "IA5 characters" :
	  (j == 5) ? "X.208/X.209" :
	  (j == 7) ? "V.120" :
	  (j == 8) ? "Q.931/I.451" :
	  ((j >= 0x10) && (j <= 0x3f)) ? "reserved incl X.31" :
	  ((j >= 0x40) && (j <= 0x4f)) ? "national use" :
	  ((j >= 0x50) && (j <= 0xfe)) ? "reserved incl X.31" :
	  "reserved";

	bsprintline(3, dst, off+i, j, 0xff, 
		    "protocol = %s (0x%02x)", temp, j);
	i++;

	while(get_valid(src,i))
	{
	    j = get_1(src,i);

	    if(isprint(j))
	      bsprintline(3, dst, off+i, j, 0xff, "user information = %c", j);
	    else
	      bsprintline(3, dst, off+i, j, 0xff, "user information = 0x%02x", j);
	    i++;
	}
	return;
}

const struct ie MAKE_TABLE(Q931_INFORMATION_ELEMENTS,TABLE_0,[],
			   { 0xff, "unknown information element", NULL });
