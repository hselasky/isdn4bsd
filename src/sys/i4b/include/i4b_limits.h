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
 *	i4b_limits.h - definition of I4B limits
 *	---------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_LIMITS_H_
#define	_I4B_LIMITS_H_

/*---------------------------------------------------------------------------*
 *	max number of controllers under I4B
 *
 * NOTE: should be power of two
 *---------------------------------------------------------------------------*/
#define	MAX_CONTROLLERS 64		/* units */

/*---------------------------------------------------------------------------*
 *	max number of channels
 *
 * NOTE: must be power of two
 *---------------------------------------------------------------------------*/
#define	MAX_CHANNELS    64		/* units */

/*---------------------------------------------------------------------------*
 *	max number of PCM cables
 *---------------------------------------------------------------------------*/
#define	I4B_PCM_CABLE_MAX    8		/* exclusive */
#define	I4B_PCM_SLOT_MAX   128		/* exclusive */

/*---------------------------------------------------------------------------*
 *	max length of some strings (includes the '\0' - character!)
 *---------------------------------------------------------------------------*/
#define	TELNO_MAX	41		/* max length of a telephone number */
#define	SUBADDR_MAX	21		/* max length of a subaddress */
#define	DISPLAY_MAX	91		/* max length of display information */
#define	DATETIME_MAX	21		/* max length of datetime information */
#define	KEYPAD_MAX	35		/* max length of a keypad string */
#define	USER_USER_MAX  129		/* max length of a user-user string */

/*---------------------------------------------------------------------------*
 *	channel parameters
 *---------------------------------------------------------------------------*/
#define	BCH_MAX_DATALEN	2048		/* max length of a B channel frame */
#define	DCH_MAX_DATALEN  264		/* max length of a D channel frame */

/*---------------------------------------------------------------------------*
 *	call descriptor id (cdid) definitions
 *---------------------------------------------------------------------------*/
#define	CDID_REF_MAX    0x100		/* exclusive */
#define	CDID_MAX        (CDID_REF_MAX * MAX_CONTROLLERS)	/* exclusive */

/*---------------------------------------------------------------------------*
 *	driver count definitions
 *---------------------------------------------------------------------------*/
#ifndef NI4BTEL
#define	NI4BTEL    8
#endif
#ifndef NI4BRBCH
#define	NI4BRBCH   8
#endif
#ifndef NI4BISPPP
#define	NI4BISPPP  8
#endif
#ifndef NI4BIPR
#define	NI4BIPR    8
#endif
#ifndef NI4BING
#define	NI4BING    8
#endif

#endif					/* _I4B_LIMITS_H_ */
