/*-
 * Copyright (c) 1996 Gary Jennejohn.  All rights reserved.
 *
 * Copyright (c) 1997, 1999 Hellmuth Michaelis. All rights reserved.
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
 *	layer2.c - decode and print layer 2 (Q.921) information
 *	-------------------------------------------------------
 *
 * $FreeBSD: src/usr.sbin/i4b/isdndecode/layer2.c,v 1.7 2000/10/09 14:22:41 hm Exp $
 *
 *---------------------------------------------------------------------------*/

#include "decode.h"
                
/*---------------------------------------------------------------------------*
 *	decode poll bit
 *---------------------------------------------------------------------------*/
static void
poll_bit(struct buffer *dst, struct buffer *src,
	 u_int16_t offset, u_int8_t mask)
{
    u_int8_t data = get_1(src, offset);

    bsprintline(dst, src, offset, mask,
		"P/F, Poll = %s", (data & mask) ? 
		"Immediate Response Required" : 
		"No Immediate Response Required");
    return;
}

/*---------------------------------------------------------------------------*
 *	decode final bit
 *---------------------------------------------------------------------------*/
static void
final_bit(struct buffer *dst, struct buffer *src,
	 u_int16_t offset, u_int8_t mask)
{
    u_int8_t data = get_1(src, offset);

    bsprintline(dst, src, offset, mask, "P/F, Final = %s", 
		(data & mask) ? "Result of Poll" : "No Result of Poll");
    return;
}

/*---------------------------------------------------------------------------*
 *	decode protocol specified in Q.921
 *
 * NOTE: this function will adjust the buffer offset, "src->offset"
 *---------------------------------------------------------------------------*/
void
layer2(struct buffer *dst, struct buffer *src, u_int8_t dir)
{
	u_int8_t j, sap, tei, cmd, offset = 0;

	/* address high */

	sap = (get_1(src,0) >> 2) & 0x3f;

	bsprintline(dst, src, offset, 0xfc, 
		   "SAPI = %d (%s)", sap, 
		   ((sap == 0))                 ? "Call Control" :
		   ((sap >= 1) && (sap <= 15))  ? "Reserved" :
		   ((sap == 16))                ? "X.25" :
		   ((sap >= 17) && (sap <= 31)) ? "Reserved" :
		   ((sap == 63))                ? "Layer 2 Management" :
		   "Not available for Q.921");

	cmd = (get_1(src,0) & 0x02) >> 1;

	if(dir == FROM_TE)
	{
	    cmd ^= 1;
	}

	bsprintline(dst, src, offset, 0x02, 
		   "C/R = %s", cmd ? "Command" : "Response");

	extension(dst, src, offset, 0x01);
	offset++;

	/* address low */

	tei = get_1(src,1) >> 1;

	bsprintline(dst, src, offset, 0xfe, 
		    "TEI = %d = 0x%02x (%s)", tei, tei, 
		    (tei <=  63) ? "Non-automatic TEI" :
		    (tei <= 126) ? "Automatic TEI" :
		    "Broadcast TEI");

	extension(dst, src, offset, 0x01);
	offset++;

	/* control 1 */

	j = get_1(src,2);
	
	if((j & 0x03) == 0x03)
	{
	    u_int8_t pf;

	    pf = j & 0x10;

	    j &= 0xef;

	    /* U-frame */

	    if(j == 0x6f)
	    {
	        /* SABME */
			
	        bsprintline(dst, src, offset, 0xef, 
			   "U-Frame: SABME (Set Asynchonous Balanced Mode)");
		poll_bit(dst, src, offset, 0x10);
		offset++;
	    }
	    else if(j == 0x0f)
	    {
	        /* DM */

	        bsprintline(dst, src, offset, 0xef, 
			   "U-Frame: DM (Disconnected Mode)");
		final_bit(dst, src, offset, 0x10);
		offset++;
	    }
	    else if(j == 0x03)
	    {
	        /* UI */

	        bsprintline(dst, src, offset, 0xef, 
			   "U-Frame: UI (Unnumbered Information)");
		poll_bit(dst, src, offset, 0x10);
		offset++;
	
		if((sap == 63) && (get_1(src,3) == 0x0f))
		{

		    /* TEI management */


		    bsprintline(dst, src, offset, 0xff, "MEI "
				"(Management Entity Identifier)");
		    offset++;
		    bsprintline(dst, src, offset, 0xff, "Ri = "
				"0x%04x (Reference number high)", 
				(get_1(src,4) << 8) | get_1(src,5));
		    offset++;
		    bsprintline(dst, src, offset, 0xff, "Ri "
				"(Reference Number low)");
		    offset++;
	
		    j = get_1(src,6);

		    bsprintline(dst, src, offset, 0xff, "TEI Identity "
				"message = %s (0x%02x)", 
				(j == 0x01) ? "REQUEST" :
				(j == 0x02) ? "ASSIGNED" :
				(j == 0x03) ? "DENIED" :
				(j == 0x04) ? "CHECK REQUEST" :
				(j == 0x05) ? "CHECK RESPONSE" :
				(j == 0x06) ? "REMOVE" :
				(j == 0x07) ? "VERIFY" :
				"UNKNOWN",
				j);
		    offset++;

		    j = (get_1(src,7) >> 1) & 0x7F;
	
		    if(j == 127) {
		        bsprintline(dst, src, offset, 0xfe, 
				    "Ai = %d (Action Indicator = %s)", j, 
				    (j == 0x01) ? "Any TEI value acceptable" :
				    (j == 0x03) ? "No TEI Value available" :
				    (j == 0x04) ? "Check all TEI values" :
				    (j == 0x06) ? "Request for removal of all TEI values" :
				    "");
		    } else {
		        bsprintline(dst, src, offset, 0xfe, "Ai = %d "
				    "(Action Indicator)", j);
		    }

		    extension(dst, src, offset, 0x01);
		    offset++;
		}
	    }
	    else if(j == 0x43)
	    {
	        /* DISC */

	        bsprintline(dst, src, offset, 0xef, "U-Frame: DISC "
			    "(Disconnect)");
		poll_bit(dst, src, offset, 0x10);
		offset++;
	    }
	    else if(j == 0x63)
	    {
	        /* UA */

	        bsprintline(dst, src, offset, 0xef, "U-Frame: UA "
			    "(Unnumbered Acknowledge)");
		final_bit(dst, src, offset, 0x10);
		offset++;
	    }
	    else if(j == 0x87)
	    {
	        /* FRMR */

	        bsprintline(dst, src, offset, 0xef, "U-Frame: FRMR "
			    "(Frame Reject)");
		final_bit(dst, src, offset, 0x10);
		offset++;
	    }
	    else if(j == 0xaf)
	    {
	        /* XID */

	        bsprintline(dst, src, offset, 0xef, "U-Frame: XID "
			    "(Exchange Identification)");
		if(cmd)
		  poll_bit(dst, src, offset, 0x10);
		else
		  final_bit(dst, src, offset, 0x10);
		offset++;
	    }
	    else
	    {
	        bsprintline(dst, src, offset, 0xef, "U-Frame: "
			    "UNKNOWN (0x%02x)", j);
	        bsprintline(dst, src, offset, 0x10, "U-Frame: "
			    "poll/final = %d", pf ? 1 : 0);
	    }
	}
	else if((j & 0x03) == 0x01)
	{
	    /* S-frame */

	    bsprintline(dst, src, offset, 0xff, "S-Frame: %s", 
		       (j == 0x01) ? "RR (Receiver Ready)" :
		       (j == 0x05) ? "RNR (Receiver Not Ready)" :
		       (j == 0x09) ? "REJ (Reject)" :
		       "Unknown");
	    offset++;
		
	    bsprintline(dst, src, offset, 0xfe, 
			"N(R) = %d (receive sequence number)", 
			(get_1(src,3) >> 1) & 0x7f);
	    if(cmd)
	      poll_bit(dst, src, offset, 0x01);
	    else
	      final_bit(dst, src, offset, 0x01);

	    offset++;
		
	}
	else if((j & 0x01) == 0x00)
	{
	    /* I-frame */

	    bsprintline(dst, src, offset, 0xfe, 
		       "N(S) = %d (send sequence number)", 
			(j >> 1) & 0x7f);

	    bsprintline(dst, src, offset, 0x01, 
		       "I-Frame: Information transfer");
	    offset++;
		
	    bsprintline(dst, src, offset, 0xfe, 
		       "N(R) = %d (receive sequence number)", 
			(get_1(src,3) >> 1) & 0x7f);

	    poll_bit(dst, src, offset, 0x01);

	    offset++;
	}

	/* adjust for layer 3 decoding */

	src->offset += offset;
	return;
}
