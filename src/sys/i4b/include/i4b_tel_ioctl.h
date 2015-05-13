/*-
 * Copyright (c) 1997, 2002  Hellmuth Michaelis. All rights reserved.
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
 *	i4b_tel_ioctl.h telephony interface ioctls
 *	------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_TEL_IOCTL_H_
#define _I4B_TEL_IOCTL_H_

/*===========================================================================*
 *	/dev/i4btel<n> devices (audio data)
 *===========================================================================*/
 
/* supported audio format conversions */

#define CVT_NONE	0		/* ISDN line: XXX, user: A-law */
#define CVT_ALAW2ULAW	1		/* ISDN line: XXX, user: u-law */
#define CVT_ULAW2ALAW	2		/* ISDN line: XXX, user: A-law */
#define CVT_8BIT_SIGNED 3		/* ISDN line: XXX, user: 8-bit signed */

/*---------------------------------------------------------------------------*
 *	get / set audio format 
 *---------------------------------------------------------------------------*/

#define	I4B_TEL_GETAUDIOFMT	_IOR('A', 0, int)
#define	I4B_TEL_SETAUDIOFMT	_IOW('A', 1, int)
#define	I4B_TEL_EMPTYINPUTQUEUE	_IOW('A', 2, int)

/*---------------------------------------------------------------------------*
 *	request version and release info from kernel part
 *---------------------------------------------------------------------------*/

#define I4B_TEL_VR_REQ		_IOR('A', 3, msg_vr_req_t)

/*---------------------------------------------------------------------------*
 *	send tones out of the tel interface
 *---------------------------------------------------------------------------*/

#define I4B_TEL_MAXTONES 32

struct i4b_tel_tones {
	uint16_t frequency_1[I4B_TEL_MAXTONES]; /* Hz */
	uint16_t frequency_2[I4B_TEL_MAXTONES]; /* Hz */
	uint16_t duration[I4B_TEL_MAXTONES]; /* samples */
};

#define I4B_TEL_TONES		_IOW('A', 4, struct i4b_tel_tones)

/*---------------------------------------------------------------------------*
 *	soundbridge ioctl's
 *---------------------------------------------------------------------------*/
#define I4B_TEL_SET_SOUNDBRIDGE		_IO('A', 5)
#define I4B_TEL_ENABLE_MUTE		_IO('A', 6)
#define I4B_TEL_DISABLE_MUTE		_IO('A', 7)
#define I4B_TEL_SET_AUDIOAMP	       _IOW('A', 8, uint8_t)
#define I4B_TEL_GET_AUDIOAMP	       _IOR('A', 9, uint8_t)
#define             AUDIOAMP_MIN 0x00 /* inclusive */
#define             AUDIOAMP_MAX 0xFF /* inclusive */
#define             AUDIOAMP_DP  (1<<5)
#define I4B_TEL_ENABLE_LARGE_BUFFER	_IO('A',10)
#define I4B_TEL_DISABLE_LARGE_BUFFER	_IO('A',11)

/*---------------------------------------------------------------------------*
 *	other ioctl's
 *---------------------------------------------------------------------------*/
#define I4B_TEL_GET_TELNO		_IOR('A',12, uint8_t [TELNO_MAX])
#define I4B_TEL_GET_DISPLAY		_IOR('A',13, uint8_t [DISPLAY_MAX])
#define I4B_TEL_GET_SMS			_IOR('A',14, uint8_t [USER_USER_MAX])

/*===========================================================================*
 *	/dev/i4bteld<n> devices (dialer interface)
 *===========================================================================*/

/* dialer commands */

#define CMD_DIAL        'D'     /* dial the following number string */
#define CMD_HANGUP      'H'     /* hangup or ignore incoming call */
#define CMD_KEYP        'K'     /* send keypad string */

#define CMD_ANSWER	'A'	/* answer incoming call */
#define CMD_REJECT	'R'	/* reject incoming call */

/* dialer responses (see DSTAT_XXX in i4b_ioctl.h) */

#endif /* _I4B_TEL_IOCTL_H_ */
