/*-
 *
 * Copyright (c) 2005 Henk Vergonet. All rights reserved.
 *
 * Copyright (c) 2009 Hans Petter Selasky. All rights reserved.
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
 *      Yealink driver for ISDN4BSD
 *      ---------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

/*
 * NOTE: The yealink.c driver in Linux has been used as hardware
 * documentation when making this driver.
 */
static const struct yealink_lcd_map yealink_lcd_map[YEALINK_LCD_LINE5_OFFSET] = {
	YEALINK_SEG('1', 0, 0, 22, 2, 22, 2, 0, 0, 0, 0, 0, 0, 0, 0),
	YEALINK_SEG('8', 20, 1, 20, 2, 20, 4, 20, 8, 21, 4, 21, 2, 21, 1),
	YEALINK_SYM('.', 22, 1, "M"),
	YEALINK_SEG('e', 18, 1, 18, 2, 18, 4, 18, 1, 19, 2, 18, 1, 19, 1),
	YEALINK_SEG('8', 16, 1, 16, 2, 16, 4, 16, 8, 17, 4, 17, 2, 17, 1),
	YEALINK_SYM('.', 15, 8, "D"),
	YEALINK_SEG('M', 14, 1, 14, 2, 14, 4, 14, 1, 15, 4, 15, 2, 15, 1),
	YEALINK_SEG('8', 12, 1, 12, 2, 12, 4, 12, 8, 13, 4, 13, 2, 13, 1),
	YEALINK_SYM('.', 11, 8, ":"),
	YEALINK_SEG('8', 10, 1, 10, 2, 10, 4, 10, 8, 11, 4, 11, 2, 11, 1),
	YEALINK_SEG('8', 8, 1, 8, 2, 8, 4, 8, 8, 9, 4, 9, 2, 9, 1),
	YEALINK_SYM('.', 7, 1, "IN"),
	YEALINK_SYM('.', 7, 2, "OUT"),
	YEALINK_SYM('.', 7, 4, "STORE"),
	YEALINK_SEG('1', 0, 0, 5, 1, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0),
	YEALINK_SEG('8', 4, 1, 4, 2, 4, 4, 4, 8, 5, 8, 5, 4, 5, 2),
	YEALINK_SEG('8', 2, 1, 2, 2, 2, 4, 2, 8, 3, 4, 3, 2, 3, 1),
	YEALINK_SYM('.', 23, 2, "NEW"),
	YEALINK_SYM('.', 23, 4, "REP"),
	YEALINK_SYM('.', 1, 8, "SU"),
	YEALINK_SYM('.', 1, 4, "MO"),
	YEALINK_SYM('.', 1, 2, "TU"),
	YEALINK_SYM('.', 1, 1, "WE"),
	YEALINK_SYM('.', 0, 1, "TH"),
	YEALINK_SYM('.', 0, 2, "FR"),
	YEALINK_SYM('.', 0, 4, "SA"),
	YEALINK_SEG('8', 22, 16, 22, 32, 22, 64, 22, 128, 23, 128, 23, 64, 23, 32),
	YEALINK_SEG('8', 20, 16, 20, 32, 20, 64, 20, 128, 21, 128, 21, 64, 21, 32),
	YEALINK_SEG('8', 18, 16, 18, 32, 18, 64, 18, 128, 19, 128, 19, 64, 19, 32),
	YEALINK_SEG('8', 16, 16, 16, 32, 16, 64, 16, 128, 17, 128, 17, 64, 17, 32),
	YEALINK_SEG('8', 14, 16, 14, 32, 14, 64, 14, 128, 15, 128, 15, 64, 15, 32),
	YEALINK_SEG('8', 12, 16, 12, 32, 12, 64, 12, 128, 13, 128, 13, 64, 13, 32),
	YEALINK_SEG('8', 10, 16, 10, 32, 10, 64, 10, 128, 11, 128, 11, 64, 11, 32),
	YEALINK_SEG('8', 8, 16, 8, 32, 8, 64, 8, 128, 9, 128, 9, 64, 9, 32),
	YEALINK_SEG('8', 6, 16, 6, 32, 6, 64, 6, 128, 7, 128, 7, 64, 7, 32),
	YEALINK_SEG('8', 4, 16, 4, 32, 4, 64, 4, 128, 5, 128, 5, 64, 5, 32),
	YEALINK_SEG('8', 2, 16, 2, 32, 2, 64, 2, 128, 3, 128, 3, 64, 3, 32),
	YEALINK_SEG('8', 0, 16, 0, 32, 0, 64, 0, 128, 1, 128, 1, 64, 1, 32),
	YEALINK_SYM('.', 24, 0x01, "LED"),
	YEALINK_SYM('.', 25, 0x01, "DIALTONE"),
	YEALINK_SYM('.', 26, 0x24, "RINGTONE"),
};
