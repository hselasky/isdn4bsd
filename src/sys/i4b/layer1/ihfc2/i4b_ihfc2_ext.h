/*-
 * Copyright (c) 2000-2004 Hans Petter Selasky. All rights reserved.
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
 *	i4b_ihfc2_ext.h - common prototypes
 *	-----------------------------------
 *
 *      last edit-date: [ ]
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef _I4B_IHFC2_EXT_H_
#define _I4B_IHFC2_EXT_H_

/* prototypes from "i4b_ihfc_pnp.c" */

/* prototypes from "i4b_ihfc_l1if.c" */
u_int8_t	ihfc_setup_i4b		(ihfc_sc_t *sc, u_int8_t *error);
void		ihfc_unsetup_i4b	(ihfc_sc_t *sc);
void		ihfc_i4b_putmbuf	(ihfc_sc_t *sc, ihfc_fifo_t *f, struct mbuf *m);
struct mbuf *   ihfc_i4b_getmbuf	(ihfc_sc_t *sc, ihfc_fifo_t *f);
void		ihfc_trace_info		(ihfc_sc_t *sc, const u_int8_t *desc);

/* prototypes from "i4b_ihfc_drv.c" */
u_int8_t	ihfc_setup_softc	(ihfc_sc_t *sc, u_int8_t *error);
void		ihfc_unsetup_softc	(ihfc_sc_t *sc);
void		ihfc_chip_interrupt     (ihfc_sc_t *sc);
u_int8_t	ihfc_fifos_active	(ihfc_sc_t *sc);
u_int8_t	ihfc_fifo_setup		(ihfc_sc_t *sc, ihfc_fifo_t *f);
void		ihfc_fifo_call		(ihfc_sc_t *sc, ihfc_fifo_t *f);
void		ihfc_reset		(ihfc_sc_t *sc, u_int8_t *error);

void		fsm_update		(ihfc_sc_t *sc, u_int8_t flag);

/* prototypes from "i4b_ihfc_dev.c" */
u_int8_t	ihfc_setup_ldev		(ihfc_sc_t *sc, u_int8_t *);
void		ihfc_unsetup_ldev	(ihfc_sc_t *sc);

/* prototypes from "i4b_program.h" */

ihfc_fifo_program_t
  i4b_hfc_tx_program_new,
  i4b_hfc_tx_program,
  i4b_hfc_rx_program,
  i4b_ipac_tx_program,
  i4b_ipac_rx_program,
  i4b_unknown_program;

#endif /* _I4B_IHFC2_EXT_H_ */
