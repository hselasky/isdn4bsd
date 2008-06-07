/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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
 *	capilib.h 
 *	---------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#ifndef __CAPILIB_H__
#define __CAPILIB_H__

/* 
 * This structure is used to cache incoming B-channel data!
 */
struct data_buffer {

  struct data_buffer * next; /* pointer to next data buffer on free list */

  uint8_t state;
# define   ST_FREE 0 /* default */
# define   ST_USED 1

  uint8_t unused; /* for sake of alignment */

  uint16_t wHandle; /* copy of wHandle from data-b3-indication */

  struct capi_message_encoded msg;

  uint8_t data[0]; /* B-channel data follows */

} __packed;

/* 
 * This structure stores information about a registered application
 */
struct app_softc {

  uint32_t            sc_app_id; /* applet number (internal) */
  uint16_t            sc_app_id_real; /* applet number (external) */

  int                 sc_fd; /* CAPI device filenumber */

  void *              sc_msg_start_ptr; /* pointer to start of 
                                          * messages (inclusive) 
                                          */
  void *              sc_msg_end_ptr;   /* pointer to end of 
                                          * messages (exclusive) 
                                          */
  uint32_t            sc_msg_size; /* size of a message in bytes */

  uint32_t            sc_msg_data_size; /* size of data part
                                          * of a message in bytes
                                          */

  struct data_buffer * sc_msg_free_list; /* pointer to first free message */

  uint32_t             sc_max_connections;

  uint8_t              sc_backend;

  uint8_t              sc_temp[64+16];

  struct app_softc *   sc_next;
};

struct debug {
   uint8_t what;
   uint16_t size;
   uint16_t offset;
   const char *field;
} __packed;

struct capi20_backend {
    uint8_t bBackendType;
#define	CAPI_BACKEND_TYPE_I4B 0
#define	CAPI_BACKEND_TYPE_BINTEC 1
    char sHostName[64];
    char sServName[16];
    char sUserName[64];
    char sPassWord[64];
};

extern int capilib_get_message_sub(struct app_softc *sc, void *buf, uint16_t msg_len);
extern int capilib_bintec_do_ioctl(struct app_softc *sc, uint32_t cmd, void *data);
extern struct app_softc * capilib_alloc_app_bintec(struct capi20_backend *be);
extern struct app_softc * capilib_alloc_app_sub(struct capi20_backend *be);
extern struct app_softc * capilib_alloc_app(struct capi20_backend *be);
extern void capilib_free_app(struct app_softc *sc);

#endif /* __CAPILIB_H__ */
