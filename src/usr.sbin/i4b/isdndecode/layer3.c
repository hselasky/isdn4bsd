/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	layer3.c - decode and print contents of a layer 3 I-frame
 *	---------------------------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndecode/layer3.c,v 1.9 2002/03/26 15:13:02 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#include "decode.h"

static const char * const
  MAKE_TABLE(Q931_MESSAGE_TYPES,DESC,[]);

static const u_int8_t *
get_pd_description(u_int8_t j)
{
  return
    (j <= 0x07)                  ? "User-User IE" :
    (j == 0x08)                  ? "Q.931/I.451/DSS1" :
    (j == 0x40)                  ? "1TR6, N0" :
    (j == 0x41)                  ? "1TR6, N1" :
    (j == 0xaa)                  ? "Euracom specific" : 
    ((j >= 0x10) && (j <= 0x3f)) ? "Other Layer 3 or X.25" :
    ((j >= 0x40) && (j <= 0x4f)) ? "National Use" :
    ((j >= 0x50) && (j <= 0xfe)) ? "Other Layer 3 or X.25" :
    "unknown";
}

/*---------------------------------------------------------------------------*
 *	decode Q.931 protocol
 *
 * returns non-zero if error
 *---------------------------------------------------------------------------*/
u_int8_t
layer3_dss1(struct buffer *dst, struct buffer *src)
{
	const u_int8_t *temp;
	u_int16_t off = src->offset;
	u_int8_t  codeset = 0;
	u_int8_t  codeset_temp = 0;
	u_int8_t  codeset_next = 0;
	u_int16_t i = 0;
	u_int8_t  j;

	if(get_valid(src,0) == 0)
	{
	    /* no I-frame */
	    return 0;
	}

	/* protocol discriminator */

	j = get_1(src,i);

	if(j != 0x08)
	{
	    return 1;
	}
	bsprintline(3, dst, off+i, j, 0xff, 
		    "Protocol = %s (0x%02x)", 
		    get_pd_description(j), j);
	i++;

	/* call reference */

	j = get_1(src,i) & 0x0f;
	
	bsprintline(3, dst, off+i, get_1(src,i), 0xf0, "Call Reference");

	bsprintline(3, dst, off+i, get_1(src,i), 0x0f, "Call Reference length is %d byte%s%s", 
		    j, (j == 1) ? "" : "s", j ? "" : " (Dummy CR)");

	if(j && j--)
	{
	    i++;
	    bsprintline(3, dst, off+i, get_1(src,i), 0x80, "Call Reference is sent %s call originator", (get_1(src,i) & 0x80) ? "to" : "from");
	    bsprintline(3, dst, off+i, get_1(src,i), 0x7f, "Call Reference = %d = 0x%02x", (get_1(src,i) & 0x7f), (get_1(src,i) & 0x7f));
	}

	while(j && j--)
	{
	    i++;
	    bsprintline(3, dst, off+i, get_1(src,i), 0xff, "Call reference = %d = 0x%02x", 
			get_1(src,i), get_1(src,i));
	}

	i++;

	/* message type */	

	j = get_1(src,i) & 0x80;

	bsprintline(3, dst, off+i, j, 0x80, "Message type extension = %d", j ? 1 : 0);

	j = get_1(src,i) & 0x7f;

	if((j < INDEXES(Q931_MESSAGE_TYPES_DESC)) && Q931_MESSAGE_TYPES_DESC[j])
	  temp = Q931_MESSAGE_TYPES_DESC[j];
	else
	  temp = "UNKNOWN";

	bsprintline(3, dst, off+i, j, 0x7f, 
		    "Message type = %s (0x%02x)", temp, j);
	i++;
	
	/* information elements */

	src->offset += i;
	i = 0;

	while(get_valid(src,0))
	{
	    off = src->offset;
	    codeset = codeset_temp;
	    codeset_temp = codeset_next;

	    j = get_1(src,0);

	    bsprintline(3, dst, off+0, j, 0x80, "%s Information element", 
		       (j & 0x80) ? "Single octet" : "Variable length");

	    if(j & 0x80)
	    {
	        /* single octett info element type 1 */

	        if((j & 0x70) == 0x00)
		{
		    temp = "Reserved";
		    bsprintline(3, dst, off+0, j, 0x70, "Reserved");
		    bsprintline(3, dst, off+0, j, 0x0f, "Reserved, content of IE");
		}
		else if((j & 0x70) == 0x10)
		{
		    codeset_next = codeset;
		    codeset_temp = (j & 0x07);

		    if(j & 0x08)
		    {
		        /* shift lock */
		        codeset_next = codeset_temp;
		    }

		    bsprintline(3, dst, off+0, j, 0x70, "Shift");
		    bsprintline(3, dst, off+0, j, 0x08, "%s shift", 
			       (j & 0x08) ? "Non-locking" : "Locking");

		    j &= 0x07;

		    temp =
		      (j == 0) ? "Not applicable" :
		      (j == 4) ? "Codeset 4 (ISO/IEC)" :
		      (j == 5) ? "Codeset 5 (National use)" :
		      (j == 6) ? "Codeset 6 (Local network specific)" :
		      (j == 7) ? "Codeset 7 (User specific)" :
		      "Reserved";

		    bsprintline(3, dst, off+0, j, 0x07, 
			       "%s (0x%02x)", temp, j);
		}
		else if((j & 0x70) == 0x30)
		{
		    bsprintline(3, dst, off+0, j, 0x70, "Congestion Level");

		    j &= 0x0f;

		    temp =
		      (j == 0x00) ? "receiver ready" :
		      (j == 0x0f) ? "receiver not ready" :
		      "reserved";

		    bsprintline(3, dst, off+0, j, 0x0f, 
			       "Congestion Level = %s (0x%02x)", temp, j);
		}
		else if((j & 0x70) == 0x50)
		{
		    bsprintline(3, dst, off+0, j, 0x70, "Repeat Indicator");

		    j &= 0x0f;

		    temp =
		      (j == 0x02) ? "Prioritized list for selecting one possibility" :
		      "reserved";

		    bsprintline(3, dst, off+0, j, 0x0f, 
			       "Repeat indication = %s (0x%02x)", temp, j);
		}
			
		/* single octett info element type 2 */

		else if((j & 0x7f) == 0x20)
		{
		    bsprintline(3, dst, off+0, j, 0x7f, "More data");
		}
		else if((j & 0x7f) == 0x21)
		{
		    bsprintline(3, dst, off+0, j, 0x7f, "Sending complete");
		}
		else
		{
		    bsprintline(3, dst, off+0, j, 0xff, 
			       "UNKNOWN single octet IE = 0x%02x", j);
		}
		src->offset += 1;
		continue;
	    }

	    if(codeset == 0)
	    {
	        const struct ie *iep = &Q931_INFORMATION_ELEMENTS_TABLE_0[0];

		while(1)
		{
		    if((iep->code == j) ||
		       (iep->code == 0xff))
		    {
		        break;
		    }
		    iep++;
		}

		j = get_1(src,1);

		bsprintline(3, dst, off+0, get_1(src,0), 0x7f, "IE = %s", iep->name);
		bsprintline(3, dst, off+1, j, 0xff, "IE Length = %d bytes", j);

		if(iep->func)
		{
		    u_int16_t old_len;

		    old_len = set_length(src, src->offset + j + 2);

		    (iep->func)(dst, src);

		    src->offset = src->len;
		    src->len = old_len;
		    continue;
		}
	    }
	    else
	    {
	        bsprintf(dst, "UNKNOWN CODESET=%d, IE=0x%02x\n", 
			 codeset, j);
	    }

	    j = get_1(src,1);

	    bsprintline(3, dst, src->offset+1, j, 0xff, "length = %d bytes", j);

	    src->offset += 2;

	    while(j--)
	    {
	        i = get_1(src,0);

	        bsprintline(3, dst, src->offset, i, 0xff,
			    "unknown byte = 0x%02x, %c", 
			    i, isprint(i) ? i : '.');

		src->offset ++;
	    }
	}
	return 0;
}


/*---------------------------------------------------------------------------*
 *	get 1TR6 cause description
 *---------------------------------------------------------------------------*/
static const char *
itr6_get_cause(u_int8_t code)
{
	static const u_int8_t * const
	  MAKE_TABLE(ITR6_CAUSES,DESC,[]);
	const u_int8_t *temp;

	if(code < INDEXES(ITR6_CAUSES_DESC))
	{
	    temp = ITR6_CAUSES_DESC[code];
	}
	else
	{
	    temp = NULL;
	}

	if(temp == NULL)
	{
	    temp = "unknown";
	}
	return temp;
}

/*---------------------------------------------------------------------------*
 *	decode and print 1TR6 cause information element
 *---------------------------------------------------------------------------*/
static void
itr6_cause(struct buffer *dst, struct buffer *src, u_int16_t i, u_int16_t end)
{
	const u_int8_t *temp;
	u_int16_t off = src->offset;
	u_int8_t j;

	/* skip length and type */
	i += 2;

	if(get_valid(src,i))
	{
	    j = get_1(src,i);
	    bsprintline(3, dst, off+i, j, 0x7f, "cause = %s (0x%02x)",
			itr6_get_cause(j), j);
	    i++;
	}
	else
	{
	    bsprintf(dst, "L3 --> cause = %s\n",
		     itr6_get_cause(0));
	}

	if(get_valid(src,i))
	{
	    j = get_1(src,i) & 0x0f;

	    temp = 
	      (j == 0x04) ? "public network" :
	      (j == 0x05) ? "private network" :
	      (j == 0x0f) ? "no information" :
	      "unknown";

	    bsprintline(3, dst, off+i, j, 0x0f, "location = %s (0x%02x)",
			temp, j);
	    i++;
	}

	while(i < end)
	{
	    j = get_1(src,i);

	    bsprintline(3, dst, off+i, j, 0xff, "extra byte = 0x%02x, %c",
			j, isprint(j) ? j : '.');
	    i++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode and print 1TR6 telephone number
 *---------------------------------------------------------------------------*/
static void
itr6_address(struct buffer *dst, struct buffer *src, u_int16_t i, u_int16_t end)
{
	const u_int8_t *temp;
	u_int16_t off = src->offset;
	u_int8_t j;

	/* skip type and length */
	i += 2;

	j = get_1(src,i) & 0x70;

	temp = 
	  (j == 0x10) ? "international" :
	  (j == 0x20) ? "national" :
	  "unknown";

	bsprintline(3, dst, off+i, j, 0x70, "type = %s (0x%02x)",
		    temp, j / 0x10);

	j = get_1(src,i) & 0x0f;

	temp = 
	  (j == 1) ? "ISDN" : 
	  "unknown";

	bsprintline(3, dst, off+i, j, 0x0f, "plan = %s (0x%02x)",
		    temp, j);

	i++;

	while(i < end)
	{
	  j = get_1(src,i);

	  if(isprint(j))
	    bsprintline(3, dst, off+i, j, 0xff, "digit = %c", j);
	  else
	    bsprintline(3, dst, off+i, j, 0xff, "digit = 0x%02x", j);

	  i++;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	decode the German national specific 1TR6 protocol
 *
 * returns non-zero if error
 *---------------------------------------------------------------------------*/
u_int8_t
layer3_1tr6(struct buffer *dst, struct buffer *src)
{
	const u_int8_t *temp;
	u_int8_t  codeset = 0;
	u_int8_t  codeset_next = 0;
	u_int8_t  codeset_temp = 0;
	u_int16_t off = src->offset;
	u_int16_t end;
	u_int16_t i = 0;
	u_int8_t  j;
	u_int8_t  pd;

	if(!get_valid(src,0))
	{
	    /* no I-frame */
	    return 0;
	}

	/* protocol discriminator */

	pd = get_1(src,i);

	if((pd != 0x40) &&
	   (pd != 0x41))
	{
	    return 1;
	}

	bsprintline(3, dst, off+i, pd, 0xff, 
		    "Protocol = %s (0x%02x)", 
		    get_pd_description(pd), pd);

	i++;

	/* call reference */

	j = get_1(src,i) & 0x0f;

	bsprintline(3, dst, off+i, get_1(src,i), 0xf0, "Call Reference");

	bsprintline(3, dst, off+i, get_1(src,i), 0x0f, "Call Reference length is %d byte%s%s", 
		    j, (j == 1) ? "" : "s", j ? "" : " (Dummy CR)");

	if(j && j--)
	{
	    i++;
	    bsprintline(3, dst, off+i, get_1(src,i), 0x80, "Call Reference direction = %s call originator", (get_1(src,i) & 0x80) ? "to" : "from");
	    bsprintline(3, dst, off+i, get_1(src,i), 0x7f, "Call Reference = %d = 0x%02x", (get_1(src,i) & 0x7f), (get_1(src,i) & 0x7f));
	}

	while(j && j--)
	{
	    i++;
	    bsprintline(3, dst, off+i, get_1(src,i), 0xff, "Call reference = %d = 0x%02x", 
			get_1(src,i), get_1(src,i));
	}

	i++;

	/* extension bit */

	j = get_1(src,i) & 0x80;

	bsprintline(3, dst, off+i, j, 0x80, "Message type extension = %d", j ? 1 : 0);

	/* message type */

	j = get_1(src,i) & 0x7f;

	if(pd == 0x40)	/* protocol discriminator N0 */
	{
	    temp = 
	      (j == 0x61) ? "REGISTER INDICATION" :
	      (j == 0x62) ? "CANCEL INDICATION" :
	      (j == 0x63) ? "FACILITY STATUS" :
	      (j == 0x64) ? "STATUS ACKNOWLEDGE" :
	      (j == 0x65) ? "STATUS REJECT" :
	      (j == 0x66) ? "FACILITY INFORMATION" :
	      (j == 0x67) ? "INFORMATION ACKNOWLEDGE" :
	      (j == 0x68) ? "INFORMATION REJECT" :
	      (j == 0x75) ? "CLOSE" :
	      (j == 0x77) ? "CLOSE ACKNOWLEDGE" :
	      "UNKNOWN";
	}
	else if(pd == 0x41)
	{
	    temp =
	      (j == 0x00) ? "ESCAPE" :
	      (j == 0x01) ? "ALERT" :
	      (j == 0x02) ? "CALL SENT" :
	      (j == 0x07) ? "CONNECT" :
	      (j == 0x0f) ? "CONNECT ACKNOWLEDGE" :
	      (j == 0x05) ? "SETUP" :
	      (j == 0x0d) ? "SETUP ACKNOWLEDGE" :	    
	      (j == 0x26) ? "RESUME" :
	      (j == 0x2e) ? "RESUME ACKNOWLEDGE" :
	      (j == 0x22) ? "RESUME REJECT" :
	      (j == 0x25) ? "SUSPEND" :
	      (j == 0x2d) ? "SUSPEND ACKNOWLEDGE" :
	      (j == 0x21) ? "SUSPEND REJECT" :
	      (j == 0x20) ? "USER INFORMATION" :
	      (j == 0x40) ? "DETACH" :
	      (j == 0x45) ? "DISCONNECT" :
	      (j == 0x4d) ? "RELEASE" :
	      (j == 0x5a) ? "RELEASE ACKNOWLEDGE" :
	      (j == 0x6e) ? "CANCEL ACKNOWLEDGE" :
	      (j == 0x67) ? "CANCEL REJECT" :
	      (j == 0x69) ? "CONGESTION CONTROL" :
	      (j == 0x60) ? "FACILITY" :
	      (j == 0x68) ? "FACILITY ACKNOWLEDGE" :
	      (j == 0x66) ? "FACILITY CANCEL" :
	      (j == 0x64) ? "FACILITY REGISTER" :
	      (j == 0x65) ? "FACILITY REJECT" :
	      (j == 0x6d) ? "INFORMATION" :
	      (j == 0x6c) ? "REGISTER ACKNOWLEDGE" :
	      (j == 0x6f) ? "REGISTER REJECT" :
	      (j == 0x63) ? "STATUS" :
	      "UNKNOWN";
	}
	else
	{
	    temp = "UNKNOWN";
	}

	bsprintline(3, dst, off+i, j, 0x7f, 
		    "Message type = %s (0x%02x)", temp, j);

	/* other information elements */

	i++;
	
	while(get_valid(src,i))
	{
	    codeset = codeset_temp;
	    codeset_temp = codeset_next;

	    j = get_1(src,i);

	    bsprintline(3, dst, off+i, j, 0x80, "%s info element",
			(j & 0x80) ? "single octett" : "variable length");

	    if(j & 0x80)
	    {
	        /* single octett info element */

	        switch(j & 0x70) {
		case 0x00:	/* reserved */
		    bsprintline(3, dst, off+i, j, 0x70, "info element type = reserved (0x%x)",
				(j & 0x70) / 0x10);
		    bsprintline(3, dst, off+i, j, 0x0F, "reserved single "
				"octett info (0x%02x)", j & 0x0F);
		    break;

		case 0x10:	/* shift */
		    bsprintline(3, dst, off+i, j, 0x70, "info element type = "
				"codeset shift (0x%x)", (j & 0x70) / 0x10);
		    codeset_next = codeset;
		    codeset_temp = (j & 0x07);
		    if(!(j & 0x08))
		    {
		        /* shift lock */
		        codeset_next = codeset_temp;
		    }
		    bsprintline(3, dst, off+i, j, 0x07, "shift to 0x%x %s",
				codeset_temp, (j & 0x08) ? "" : "(locked)");
		    break;

		case 0x20:	/* more data */
		    bsprintline(3, dst, off+i, j, 0x7F, "info element type = "
				"more data (0x%02x)", j & 0x7F);
		    break;

		case 0x30:	/* congestion level */
		    bsprintline(3, dst, off+i, j, 0x70, "info element type = "
				"more data (0x%02x)", j & 0x7F);

		    bsprintline(3, dst, off+i, j, 0x0f, "congestion "
				"level = %d", j & 0xF);
		    break;

		default:
		    bsprintline(3, dst, off+i, j, 0x7F, "info element type = "
				"unknown (0x%02x)", j & 0x7F);
		    break;
		}
		i++;	/* next */
	    }
	    else
	    {
	        /* variable length info element */

	        j = get_1(src,i);
		end = i + get_1(src,i+1) + 2;

	        if(codeset == 0)
		{
		    switch(j) {
		    case 0x08:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "cause (0x%02x)", j & 0x7F);

			itr6_cause(dst, src, i, end);
			goto next;
						
		    case 0x0c:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "connect address (0x%02x)", j & 0x7F);

			itr6_address(dst, src, i, end);
			goto next;

		    case 0x10:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "call identity (0x%02x)", j & 0x7F);
			break;

		    case 0x18:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "channel identity (0x%02x)", j & 0x7F);

			i += 2;

			j = get_1(src,i) & 0x08;

			bsprintline(3, dst, off+i, j, 0x08, j ? "exclusive channel" : "preferred channel");

			j = get_1(src,i) & 0x03;

			temp = 
			  (j == 0) ? "no channel" :
			  (j == 1) ? "B-1" :
			  (j == 2) ? "B-2" :
			  (j == 3) ? "any channel" :
			  "unknown";

		        bsprintline(3, dst, off+i, j, 0x03, "channel = %s", temp);
			goto next;

		    case 0x20:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "network specific facilities (0x%02x)", j & 0x7F);

			i += 3;
			j = get_1(src,i);

			temp = 
			  (j == 1)    ? "Sperre" :
			  (j == 2)    ? "AWS 1" :
			  (j == 3)    ? "AWS 2" :
			  (j == 0xe)  ? "Konferenz" :
			  (j == 0xf)  ? "B-Kan uebern." :
			  (j == 0x10) ? "aktvrg. ghlt. Vbdg." :
			  (j == 0x11) ? "3er Konf" :
			  (j == 0x12) ? "1seitg D/G Wechsel" :
			  (j == 0x13) ? "2seitig D/G Wechsel" :
			  (j == 0x14) ? "Rufnr. identifiz." :
			  (j == 0x15) ? "GBG" :
			  (j == 0x17) ? "ueberg. Ruf" :
			  (j == 0x1a) ? "um/weitergel. Ruf" :
			  (j == 0x1b) ? "unterdr. A-Rufnr." :
			  (j == 0x1e) ? "Verbdg. deaktivieren" :
			  (j == 0x1d) ? "Verbdg. aktivieren" :
			  (j == 0x1f) ? "SPV" :
			  (j == 0x23) ? "Rueckw. 2seitg. DW" :
			  (j == 0x24) ? "Anrufumltg. priv. Netz" :
			  "undefined";

			bsprintline(3, dst, off+i, j, 0xff, "facility = %s (0x%02x)",
				    temp, j);

			i++;
			j = get_1(src,i);
			bsprintline(3, dst, off+i, j, 0xff, "serv = 0x%02x", j);

			i++;
			j = get_1(src,i);
			bsprintline(3, dst, off+i, j, 0xff, "ainfo = 0x%02x", j);

			i++;

			while(i < end)
			{
			    j = get_1(src,i);
			    bsprintline(3, dst, off+i, j, 0xff, "extra data = 0x%02x, %c", 
					j, isprint(j) ? j : '.');
			    i++;
			}
			goto next;

		    case 0x28:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "display (0x%02x)", j & 0x7F);
			break;

		    case 0x2c:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "keypad (0x%02x)", j & 0x7F);
			break;

		    case 0x6c:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "originator address (0x%02x)", j & 0x7F);

			itr6_address(dst, src, i, end);
			goto next;

		    case 0x70:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "destination address (0x%02x)", j & 0x7F);

			itr6_address(dst, src, i, end);
			goto next;

		    case 0x7e:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "user-user information (0x%02x)", j & 0x7F);
			break;

		    case 0x7f:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "reserved (0x%02x)", j & 0x7F);
			break;

		    default:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "unknown (0x%02x)", j & 0x7F);
			break;
		    }
		}
		else if(codeset == 6)
		{
		    switch(j) {
		    case 0x01:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "service indication (0x%02x)", j & 0x7F);

			i += 2;
			j = get_1(src,i);

			temp =
			  (j == 0x01) ? "phone" :
			  (j == 0x02) ? "a/b" :
			  (j == 0x03) ? "X.21" :
			  (j == 0x04) ? "fax g4" :
			  (j == 0x05) ? "btx" :
			  (j == 0x07) ? "64k data" :
			  (j == 0x08) ? "X.25" :
			  (j == 0x09) ? "teletex" :
			  (j == 0x0a) ? "mixed" :
			  (j == 0x0d) ? "temex" :
			  (j == 0x0e) ? "picturephone" :
			  (j == 0x0f) ? "btx (new)" :
			  (j == 0x10) ? "videophone" :
			  "unknown";

		        bsprintline(3, dst, off+i, j, 0xff, "protocol = %s (0x%02x)",
				    temp, j);

			i++;
			j = get_1(src,i);
			bsprintline(3, dst, off+i, j, 0xff, "ainfo = 0x%02x", j);
			goto next;

		    case 0x02:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "charging information (0x%02x)", j & 0x7F);
			break;

		    case 0x03:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "date (0x%02x)", j & 0x7F);
			i += 2;
			while(i < end)
			{
			    j = get_1(src,i);
			    bsprintline(3, dst, off+i, j, 0xff, "data = 0x%02x, %c", j,
					isprint(j) ? j : '.');
			    i++;
			}
			goto next;

		    case 0x05:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "facility select (0x%02x)", j & 0x7F);
			break;

		    case 0x06:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "status of facilities (0x%02x)", j & 0x7F);
			break;

		    case 0x07:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "status of called party (0x%02x)", j & 0x7F);
			i += 2;
			j = get_1(src,i);

			temp = 
			  (j == 1) ? "no information]" :
			  (j == 2) ? "is being called" :
			  "undefined";

			bsprintline(3, dst, off+i, j, 0xff, "status = %s, 0x%02x",
				    temp, j);
			goto next;

		    case 0x08:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "additional transmit attributes (0x%02x)", j & 0x7F);

			i += 2;
			while(i < end)
			{
			  j = get_1(src,i) & 0x80;

			  bsprintline(3, dst, off+i, j, 0x80, "flag = %s", 
				      j ? "request" : "indication");

			  j = get_1(src,i) & 0x70;

			  temp = 
			    (j == 0x00) ? "none" :
			    (j == 0x10) ? "1" :
			    (j == 0x20) ? "2" :
			    (j == 0x30) ? "3" :
			    "undefined value";

			  bsprintline(3, dst, off+i, j, 0x70, "number of "
				      "satellite links = s", temp);
				      
			}
			goto next;

		    default:
		        bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				    "unknown (0x%02x)", j & 0x7F);
			break;
		    }
		}
		else
		{
		    bsprintline(3, dst, off+i, j, 0x7f, "info element type = "
				"0x%02x, codeset %d is not supported, cannot decode", 
				j & 0x7F, codeset);
		}

		i += 2;

		while(i < end)
		{
		    j = get_1(src,i);
		    bsprintline(3, dst, off+i, j, 0xff, "extra byte = 0x%02x, %c",
				j, isprint(j) ? j : '.');
		    i++;
		}
next:
		i = end;
	    }
	}
	return 0;
}

/*---------------------------------------------------------------------------*
 *	decode unknown I-frame
 *
 * returns non-zero if error
 *---------------------------------------------------------------------------*/
u_int8_t
layer3_unknown(struct buffer *dst, struct buffer *src)
{
	u_int16_t off = src->offset;
	u_int8_t  j;

	if(get_valid(src,0) == 0)
	{
	    /* no I-frame */
	    return 0;
	}

	/* protocol discriminator */

	j = get_1(src,0);

	bsprintline(3, dst, off+0, j, 0xff, 
		    "Protocol = %s (0x%02x)", 
		    get_pd_description(j), j);

 	src->offset ++;
	dump_raw(dst, src, "Layer3");
	return 0;
}
