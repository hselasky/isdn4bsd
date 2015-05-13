/*-
 * Copyright (c) 2000 Hans Petter Selasky. All rights reserved.
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
 *	i4b_hdlc.h - software-HDLC header file
 *	--------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_HDLC_H_
#define _I4B_HDLC_H_

extern const uint16_t HDLC_FCS_TAB[256];
extern const uint16_t HDLC_BIT_TAB[256];

/*---------------------------------------------------------------------------*
 *      HDLC_DECODE
 *      ===========
 *
 *      uint8_t : flag, blevel
 *      uint16_t: crc, ib, tmp, tmp2, len
 *
 *      next: 'continue' or 'goto xxx'
 *
 *      nfr: this is the place where you should setup
 *           'len' and 'dst' for the new frame, so that
 *           'dst' may be written at most 'len' times.
 *           One of cfr, rab or rdo will be after nfr.
 *           NOTE: If 'len' is set to zero, the decoder
 *           will currently not reset; Temporarily the
 *           decoder can be reset by setting flag to '-1'
 *           in nfr.
 *
 *      cfr: complete frame
 *      rab: read abort
 *      rdo: read data overflow
 *
 *      rdd: read data (read byte is stored in 'tmp2')
 *
 *      d: dummy
 *
 *      NOTE: bits[8..15] of tmp2 may be used to store custom data/flags
 *      NOTE: each time 'dst' is written, 'len' will be decreased by one.
 *      NOTE: these variables have to be 'suspended' / 'resumed' somehow:
 *              flag, blevel, crc, ib, tmp, len
 *      NOTE: zero is default value for all variables.
 *      NOTE: unsigned type must be used for all variables.
 *
 *      NOTE: the "zero inserted bit" of "a stuff sequence",
 *            can be the first bit of "a flag sequence".
 *
 *      NOTE: frames can end with only one flag==0x7e byte,
 *            followed by one or more idle==0xff bytes.
 *            (see SINGLE_FLAG_SEQUENCE in the code below)
 *
 *      NOTE: the frame size is rounded down to the nearest 8 bits,
 *            skipping leftover bits, which is not compatible with
 *            Recommendation Q.921 Chapter 2.9 section c).
 *            (see INTEGER_ONLY in the code below)
 *
 *      NOTE: setting "flag = len = 0" will reset the decoder.
 *
 * Overview:
 *        +--------------<-[nfr]---------+--<-[start]
 *        |                              |
 *        | +---<-+      - [cfr] -       |
 *        | |     |    /           \     |
 *        +-+[rdd]+->------[rab]----->---+
 *                     \           /
 *                       - [rdo] -
 *---------------------------------------------------------------------------*/

#define LO8(x) ((uint8_t)(x))  /* least significant byte (lowest byte) */

#define HDLC_DECODE(dst, len, tmp, tmp2, blevel, ib, crc, flag, rddcmd, nfrcmd,	\
		    cfrcmd, rabcmd, rdocmd, nextcmd, d)				\
										\
	rddcmd;									\
										\
	ib  += HDLC_BIT_TAB[LO8(tmp2)];						\
										\
	if (LO8(ib) >= 5)							\
	{									\
		if (ib & 0x20)		/* de-stuff (msb) */			\
		{								\
			if (LO8(tmp2) == 0x7e) goto j0##d;			\
			tmp2 += (tmp2 & 0x7f);					\
			blevel--;						\
										\
			if ((ib += 0x100) & 0xc) tmp2 |= 1; /* */		\
		}								\
										\
		ib &= ~0xe0;							\
										\
		if (LO8(ib) == 6)	/* flag seq (lsb) */			\
		{								\
		 j0##d: if (flag >= 2)						\
			{							\
			  j00##d:						\
				crc ^= 0xf0b8;					\
				len += ((4 - flag) & 3); /* remove CRC bytes */	\
				/* #ifdef INTEGER_ONLY				\
				 * if(blevel != ( 8 - ((ib >> 8) & 0xf) )) {	\
				 *	 * frame  does	 not  have		\
				 *	 * integer number of bytes		\
				 *	crc |= 0xffff;				\
				 * }						\
				 * #endif					\
				 */						\
				cfrcmd;						\
				len = 0;					\
			}							\
										\
			flag   = 1;						\
			blevel = ((ib >> 8) & 0xf);				\
			tmp    = ((LO8(tmp2)) >> blevel);			\
			blevel = (8 - blevel);					\
										\
			ib >>= 12;						\
										\
			nextcmd;						\
		}								\
		if (LO8(ib) >= 7)	/* abort (msb & lsb) */			\
		{								\
			if (flag >= 2)						\
			{							\
				/* #ifdef SINGLE_FLAG_SEQUENCE			\
				 * check if the sequence  ``x0111111 01111111''	\
				 * (lsb to msb)	  was	in   the   stream   and	\
				 * recognize this as a valid closing flag.	\
				 */						\
				if( /* (LO8(tmp2) == 0xfe) && */		\
				   (LO8(ib) == 0x16))				\
				{ goto j00##d; }				\
				/* #endif */					\
				rabcmd;						\
				len = 0;					\
			}							\
										\
			flag = 0;						\
										\
			ib >>= 12;						\
										\
			nextcmd;						\
		}								\
		if (LO8(ib) == 5)	/* de-stuff (lsb) */			\
		{								\
			tmp2 = ((tmp2 | (tmp2 + 1)) & ~0x1);			\
			blevel--;						\
		}								\
		if (blevel > 7)		/* EO - bits */				\
		{								\
			/* (blevel == -1) || (blevel == -2)			\
			 * Original code:					\
			 * tmp |= ((LO8(tmp2)) >> (8 - (blevel &= 7)));		\
			 */							\
			blevel &= 7;						\
			if(blevel & 1)						\
			{							\
			  tmp |= (LO8(tmp2) >> 1);				\
			}							\
			else							\
			{							\
			  tmp |= (LO8(tmp2) >> 2);				\
			}							\
										\
			ib >>= 12;						\
										\
			nextcmd;						\
		}								\
	}									\
										\
	tmp |= (LO8(tmp2)) << blevel;						\
										\
	if (!len--)								\
	{									\
		len++;								\
										\
		if (!flag++) { flag--; goto j5##d;} /* hunt mode */		\
										\
		switch (flag)							\
		{   case 2:		/* new frame */				\
			nfrcmd;							\
			crc = -1;						\
			if (!len--) { len++; flag++; goto j4##d; }		\
			goto j3##d;						\
		    case 3:		/* CRC (lsb's) */			\
		    case 4:		/* CRC (msb's) */			\
			goto j4##d;						\
		    case 5:		/* RDO */				\
			rdocmd;							\
			flag = 0;						\
		   default:							\
			goto j5##d;						\
		}								\
	}									\
	else									\
	{									\
	 j3##d: dst = (LO8(tmp));						\
	 j4##d: crc = (HDLC_FCS_TAB[LO8(tmp ^ crc)] ^ (LO8(crc >> 8)));		\
	}									\
										\
 j5##d: ib >>= 12;								\
	tmp >>= 8;								\
										\
/*------ end of HDLC_DECODE -------------------------------------------------*/

/*---------------------------------------------------------------------------*
 *      HDLC_ENCODE
 *      ===========
 *
 *      uint8_t : flag, src
 *      uint16_t: tmp2, blevel, ib, crc, len
 *      uint32_t: tmp
 *
 *      gfr: this is the place where you free the last [mbuf] chain, and get
 *           the next one. If a mbuf is available the code should setup 'len'
 *           and 'src' so that 'src' can be read 'len' times. If no mbuf is
 *           available, leave 'len' and 'src' untouched.
 *
 *      nmb: if your implementation accept/use chained mbufs, this is the
 *           place where you update 'len' and 'src' to send the next mbuf
 *           of the chain that makes up a frame.  If no further mbufs are
 *           available leave 'len' and 'src' untouched.   This is not the
 *           place where you free the mbuf. Leave the block empty if your
 *           implementation does not accept/use chained mbufs.
 *
 *      wrd: write data (output = (uint8_t)tmp)
 *
 *      d: dummy
 *
 *      NOTE: setting flag to '-2' and len to '0', the encoder will send
 *            two or more abort bytes
 *      NOTE: these variables have to be 'suspended' / 'resumed' somehow:
 *              flag, blevel, crc, ib, tmp, len
 *      NOTE: zero is default value for all variables.
 *      NOTE: each time 'src' is read, 'len' is decreased by one.
 *      NOTE: neither cmd's should exit through 'goto' or 'break' statements.
 *
 *      NOTE: D-channel encoding will add at least two IDLE bytes(0xff)
 *            after a closing flag sequence(0x7e).   B-channel encoding
 *            will not do this.
 *
 *      NOTE: B-channel encoding requires that the last byte is repeated,
 *            because it uses flag-interleaving.
 *
 *      NOTE: the original encoder would  not stuff any (xx011111)
 *            (lsb to msb) bytes before  flag sequence (01111110).
 *            Instead  of  ``xx011111 0 01111110''   the   encoder
 *            would  output  ``xx011111 01111110''.  HDLC_DECODE()
 *            accepts  both  versions,  given  that  ``the integer
 *            frame length check'' is disabled.  If  code size  is
 *            important,  you can comment out the code that starts
 *            with  ``#ifdef STUFF_XX011111''   until   the   next
 *            ``#endif'' in the macro below.   If compability with
 *            existing ISDN hardware is important,  you should use
 *            the macro like it is.
 *
 * Overview:
 *        +--------------<-[gfr]---------+--<-[start]
 *        |                              |
 *        | +------------------<-+       |
 *        | |+---<-+             |       |
 *        | ||     |             |       |
 *        +-++[wrd]+->-----[nmb]-+->-----+
 *---------------------------------------------------------------------------*/

#ifndef R
#define R(args...) args
#endif

#define HDLC_ENCODE_TYPE(b_channel, d_channel) R b_channel

#define HDLC_ENCODE(src, len, tmp, tmp2, blevel, ib, crc, flag, gfrcmd, nmbcmd,	\
		    wrdcmd, d)							\
										\
	if (blevel >= 0x800) { blevel -= 0x800; goto j4##d; }			\
										\
	if (!len--)								\
	{									\
		len++;								\
										\
	HDLC_ENCODE_TYPE							\
	(									\
	 (/* default encoding */						\
		switch(++flag)							\
		{ default:			/* abort */			\
			tmp  = blevel = 0;	/* zero is default */		\
			tmp2 = 0xff;						\
			/* #ifdef STUFF_XX011111 */				\
			ib = 0x0000;						\
			/* #endif */						\
			goto j3##d;						\
		  case 1:			/* 1st time FS */		\
		  case 2:			/* 2nd time FS */		\
			/* #ifdef STUFF_XX011111 */				\
			ib >>= 12;						\
			if(ib == 5) { /* stuff the ``xx011111 special case'' */	\
			  blevel += 0x100;	/* (optional) */		\
			}							\
			/* #endif */						\
			tmp2 = 0x7e;						\
			goto j3##d;						\
		  case 3:							\
			gfrcmd;			/* get new frame */		\
			if (!len--)						\
			{							\
				len++;						\
				flag--;		/* don't proceed */		\
				tmp2 = 0x7e;					\
				goto j3##d;	/* final FS */			\
			}							\
			else							\
			{							\
				crc = -1;					\
				ib  = 0;					\
				goto j1##d;	/* first byte */		\
			}							\
		  case 4:							\
			nmbcmd;			/* get next mbuf in chain */	\
			if (!len--)						\
			{							\
				len++;						\
				crc ^= -1;					\
				tmp2 = (LO8(crc));				\
				goto j2##d;	/* CRC (lsb's) */		\
			}							\
			else							\
			{							\
				flag--;						\
				goto j1##d;	/* proceed with the frame */	\
			}							\
		  case 5:							\
			tmp2  = (LO8(crc >> 8));				\
			flag  = 1;		/* continue at ``case 2'' */	\
			goto j2##d;		/* CRC (msb's) */		\
		}								\
	 ),									\
	 (/* D-channel encoding */						\
		switch(++flag)							\
		{ default:			/* abort */			\
		  case 1:			/* IDLE byte 1 */		\
		  case 2:			/* IDLE byte 2 */		\
			tmp2 = 0xff;						\
			goto j3##d;						\
		  case 3:							\
			gfrcmd;			/* get new frame */		\
			if (len == 0)						\
			{							\
				flag--;		/* don't proceed */		\
				tmp2 = 0xff;					\
				goto j3##d;	/* final IDLE byte */		\
			}							\
			else							\
			{							\
				crc = -1;					\
				ib = 0;		/* zero is default */		\
				tmp = 0x7eff;					\
				blevel = 0x800;	/* byte align frame */		\
				goto j4##d;	/* IDLE byte #3 and first FS */	\
			}							\
		  case 4:							\
			nmbcmd;			/* get next mbuf in chain */	\
			if (!len--)						\
			{							\
				len++;						\
				crc ^= -1;					\
				tmp2 = (LO8(crc));				\
				goto j2##d;	/* CRC (lsb's) */		\
			}							\
			else							\
			{							\
				flag--;						\
				goto j1##d;	/* proceed with the frame */	\
			}							\
		  case 5:							\
			tmp2  = (LO8(crc >> 8));				\
			goto j2##d;		/* CRC (msb's) */		\
		  case 6:							\
			/* #ifdef STUFF_XX011111 */				\
			ib >>= 12;  /* stuff the ``xx011111 special case'' */	\
			if(ib == 5) {		/* (optional) */		\
			  blevel += 0x100;					\
			}							\
			/* #endif */						\
			tmp2  = 0x7e;		/* final FS */			\
			flag  =	0;		/* 2 IDLE bytes after FS */	\
			goto j3##d;						\
		}								\
	 )									\
	) /* end of HDLC_ENCODE_TYPE */						\
	}									\
	else									\
	{ j1##d	:								\
		tmp2 =(LO8(src));						\
		crc =(HDLC_FCS_TAB[LO8(crc ^ tmp2)] ^ (LO8(crc >> 8)));		\
	  j2##d:								\
										\
		ib >>= 12;							\
		ib  += HDLC_BIT_TAB[LO8(tmp2)];					\
										\
		if (LO8(ib) >= 5)	/* stuffing */				\
		{								\
			blevel &= ~0xff;					\
										\
			if (ib & 0xc0)		/* bit stuff (msb) */		\
			{							\
				if (ib & 0x80)					\
				{						\
				  tmp2 += (tmp2 & (0xff * 0x80));		\
				}						\
				else						\
				{						\
				  tmp2 += (tmp2 & (0xff * 0x40));		\
				}						\
										\
				if(ib >= 0x5000) ib -= 0x5000;			\
				blevel++;					\
			}							\
										\
			ib &= ~0xf0;						\
										\
			if (LO8(ib) >= 5)	/* bit stuff (lsb) */		\
			{							\
				register uint8_t _ib_tmp = ((ib - (ib >> 8)	\
							      + 1) & 7);	\
										\
				tmp2 += (tmp2 & (~(0x1f >> _ib_tmp)));		\
				blevel++;					\
										\
				if (LO8(ib) >= 10)	/* bit stuff (msb) */	\
				{						\
					tmp2 += (tmp2 & (~(0x7ff >> _ib_tmp)));	\
					blevel++;				\
				}						\
				if (ib & 0x8000)	/* bit walk */		\
				{						\
					ib -= (LO8(ib) >= 10) ? 10 : 5;		\
					ib  = (LO8(ib) << 12);			\
				}						\
			}							\
										\
			tmp    |= (tmp2 << (LO8(blevel >> 8)));			\
			blevel += ((LO8(blevel)) << 8);				\
		}								\
		else		/* no stuffing */				\
		{								\
		  j3##d:tmp    |= (tmp2 << (LO8(blevel >> 8)));			\
		}								\
	}									\
										\
 j4##d:	wrdcmd;									\
	tmp >>= 8;								\

/*------ end of HDLC_ENCODE -------------------------------------------------*/

#endif /* _I4B_HDLC_H_ */
