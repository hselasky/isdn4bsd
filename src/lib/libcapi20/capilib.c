/*-
 * Copyright (c) 2000-2003 Thomas Wintergerst. All rights reserved.
 *
 * Copyright (c) 2005-2008 Hans Petter Selasky. All rights reserved.
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
 *	capilib.c - shared library to access the CAPI user interface
 *	-------------------------------------------------------------
 *
 * $FreeBSD: $
 *
 * NOTE: this library is not multi-thread safe
 *
 *---------------------------------------------------------------------------*/

/* system includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/endian.h>
#include <sys/ioctl.h>
#include <sys/filio.h>
#include <poll.h>
#include <errno.h>
#include <err.h>

#define CAPI_MAKE_IOCTL

#include <i4b/include/capi20.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>

#include "capilib.h"

static struct app_softc *app_sc_root;

static struct app_softc *
capilib_find_app_by_id(uint32_t app_id)
{
	struct app_softc *sc;

	sc = app_sc_root;

	while(sc)
	{
	        if(sc->sc_app_id == app_id)
		{
			break;
		}
		sc = sc->sc_next;
	}
	return sc;
}

struct app_softc *
capilib_alloc_app_sub(struct capi20_backend *cbe)
{
	struct app_softc *sc;
	static uint32_t app_id = 0;

	/* find an unused ID first */

 again:
	sc = capilib_find_app_by_id(app_id);

	if(sc)
	{
	    app_id++;
	    app_id &= 0xFFFF; /* limit APP ID range */
	    goto again;
	}

	sc = malloc(sizeof(*sc));

	if(sc == NULL)
	{
	    return NULL;
	}

	/* set default value */

	bzero(sc, sizeof(*sc));

	sc->sc_app_id = app_id;
	sc->sc_backend = cbe->bBackendType;

	/* insert "sc" into list */

	sc->sc_next = app_sc_root;
	app_sc_root = sc;

	return sc;
}

void
capilib_free_app(struct app_softc *sc)
{
	struct app_softc **sc_pp = &app_sc_root;
	struct app_softc  *sc_p = app_sc_root;

	/* unlink "sc" first */

	while(sc_p)
	{
	    if(sc_p == sc)
	    {
	        *sc_pp = sc_p->sc_next;
		break;
	    }
	    sc_pp = &sc->sc_next;
	    sc_p = sc->sc_next;
	}

	/* release all resources allocated by this "sc" */

	if(sc->sc_fd > 0)
	{
	    close(sc->sc_fd);
	}

	if(sc->sc_msg_start_ptr)
	{
	    free(sc->sc_msg_start_ptr);
	}

	/* free softc */

	free(sc);

	return;
}

static void
capilib_free_data_buffer_by_cid(struct app_softc *sc, uint32_t dwCid)
{
	struct data_buffer *mp;

	for(mp = sc->sc_msg_start_ptr;
	    ((void *)mp) < sc->sc_msg_end_ptr;
	    mp = ADD_BYTES(mp, sc->sc_msg_size))
	{
	    if((mp->state == ST_USED) &&
	       (HEADER_CID(&(mp->msg)) == dwCid))
	    {
	        mp->next = sc->sc_msg_free_list;
		mp->state = ST_FREE;

		sc->sc_msg_free_list = mp;
	    }
	}
	return;
}

static __inline struct data_buffer *
capilib_find_data_buffer_by_handle(struct app_softc *sc, uint16_t wHandle)
{
	struct data_buffer *mp =
	  ADD_BYTES(sc->sc_msg_start_ptr, 
		    sc->sc_msg_size * wHandle);

	if((((void *)mp)>= sc->sc_msg_start_ptr) &&
	   (((void *)mp) < sc->sc_msg_end_ptr))
	{
	    if(mp->state == ST_USED)
	    {
	        return mp;
	    }
	}
	return NULL;
}

static __inline uint16_t
capilib_find_handle_by_data_buffer(struct app_softc *sc, 
    struct data_buffer *mp)
{
	if((((void *)mp) >= sc->sc_msg_start_ptr) &&
	   (((void *)mp) < sc->sc_msg_end_ptr))
	{
	    return ((((uint8_t *)mp) - 
		     ((uint8_t *)sc->sc_msg_start_ptr))
		    / sc->sc_msg_size);
	}
	return -1;
}

static int
capilib_do_ioctl_sub(struct app_softc *sc, uint32_t cmd, void *data)
{
	switch (sc->sc_backend) {
	case CAPI_BACKEND_TYPE_I4B:
	    return (ioctl(sc->sc_fd, cmd, data));
	case CAPI_BACKEND_TYPE_BINTEC:
	    return (capilib_bintec_do_ioctl(sc, cmd, data));
	default:
	    return (-1);
	}
}

static uint16_t
capilib_do_ioctl(struct capi20_backend *cbe, uint32_t cmd, void *data)
{
	struct app_softc *sc;

	sc = capilib_alloc_app(cbe);
    	if (sc == NULL) {
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	/* do IOCTL */
	if (capilib_do_ioctl_sub(sc, cmd, data) < 0) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	capilib_free_app(sc);
	return (0);
}

/*---------------------------------------------------------------------------*
 *	capi20_be_alloc_i4b - Allocate a I4B CAPI backend
 *
 * @param cbe_pp	 Pointer to pointer that should be pointed to backend.
 *
 * @retval 0             Backend allocation was successful.
 *
 * @retval Else          An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_be_alloc_i4b(struct capi20_backend **cbe_pp)
{
	struct capi20_backend *cbe;

	if (cbe_pp == NULL) {
		return (CAPI_ERROR_INVALID_PARAM);
	}

	cbe = malloc(sizeof(*cbe));
	if(cbe == NULL) {
		return (CAPI_ERROR_OS_RESOURCE_ERROR);
	}

	*cbe_pp = cbe;

	bzero(cbe, sizeof(*cbe));

	cbe->bBackendType = CAPI_BACKEND_TYPE_I4B;

	return (0);
}

/*---------------------------------------------------------------------------*
 *	capi_be_free - Free a CAPI backend
 *
 * @param be             Pointer to backend.
 *---------------------------------------------------------------------------*/
void
capi20_be_free(struct capi20_backend *cbe)
{
	if (cbe) {
	    /* clear any username and password */
	    bzero(cbe, sizeof(*cbe));
	    /* free memory */
	    free(cbe);
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	capilib_alloc_app_i4b - Allocate an I4B CAPI application
 *
 * @param cbe                      Pointer to CAPI backend.
 *
 * @retval 0                       Application allocation failed.
 *
 * @retval Else                    Pointer to CAPI Application.
 *---------------------------------------------------------------------------*/
static struct app_softc *
capilib_alloc_app_i4b(struct capi20_backend *cbe)
{
	struct app_softc *sc;

	sc = capilib_alloc_app_sub(cbe);
	if (sc == NULL) {
	    return NULL;
	}

	sc->sc_fd = open(CAPI_DEVICE_NAME, O_RDWR | O_NONBLOCK);

	if (sc->sc_fd < 0) {
	    capilib_free_app(sc);
	    return NULL;
	}

	return (sc);
}

struct app_softc *
capilib_alloc_app(struct capi20_backend *cbe)
{
	struct app_softc *sc;
	switch (cbe->bBackendType) {
	case CAPI_BACKEND_TYPE_I4B:
		sc = capilib_alloc_app_i4b(cbe);
		break;
	case CAPI_BACKEND_TYPE_BINTEC:
		sc = capilib_alloc_app_bintec(cbe);
		break;
	default:
		sc = NULL;
		break;
	}
	return (sc);
}

/*---------------------------------------------------------------------------*
 *	capi20_register - CAPI application registration
 *
 * @param cbe                      Pointer to CAPI backend.
 *
 * @param max_logical_connections  Maximum number of active B-channel 
 *                                 connections.
 *
 * @param max_b_data_blocks        Maximum number of unacknowledged incoming
 *                                 data blocks per B-channel connection.
 *
 * @param max_b_data_len           Maximum data block length to use.
 *
 * @param app_id_ptr               Pointer to where the application ID should
 *                                 be stored.
 *
 * @retval 0                       Application registration was successful.
 *
 * @retval Else                    An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_register(struct capi20_backend *cbe,
		uint32_t max_logical_connections,
		uint32_t max_b_data_blocks,
		uint32_t max_b_data_len,
		uint32_t stack_version,
		uint32_t *app_id_ptr)
{
	struct app_softc *sc;
	struct data_buffer *mp;
	struct capi_register_req req = { 0 };
	uint32_t size;
	uint32_t max_msg_data_size;
	int32_t temp = 1;

	if (app_id_ptr == NULL) {
	    return CAPI_ERROR_INVALID_PARAM;
	}

	app_id_ptr[0] = 0-1;
	if (stack_version != CAPI_STACK_VERSION) {
	    return CAPI_ERROR_UNSUPPORTED_VERSION;
	}

	sc = capilib_alloc_app(cbe);
	if (sc == NULL) {
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	/* set stack version first */
	if (capilib_do_ioctl_sub(sc, CAPI_SET_STACK_VERSION_REQ, 
	    &stack_version) < 0) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_UNSUPPORTED_VERSION;
	}

	/* compute the maximum message size that we can receive */
	max_msg_data_size =
         sizeof(mp->msg) + max_b_data_len;

	/* register this application */
	req.max_logical_connections = max_logical_connections;
	req.max_b_data_blocks       = max_b_data_blocks;
	req.max_b_data_len          = max_b_data_len;
	req.max_msg_data_size       = max_msg_data_size;
	req.pUserName               = cbe->sUserName;
	req.pPassWord               = cbe->sPassWord;

	if (capilib_do_ioctl_sub(sc, CAPI_REGISTER_REQ, &req) < 0) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	if ((req.max_b_data_len != max_b_data_len) ||
	    (req.max_msg_data_size != max_msg_data_size)) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_INVALID_BUFFER_SIZE;
	}

	sc->sc_msg_data_size = max_msg_data_size;
	sc->sc_app_id_real = req.app_id;
	sc->sc_max_connections = req.max_logical_connections;

	sc->sc_msg_size = 
	  sizeof(*mp) + req.max_b_data_len;

	size = (req.max_logical_connections *
		req.max_b_data_blocks) + 1;

	if((size < 1) ||
	   (size > 65535) || 
	   (sc->sc_msg_data_size < sizeof(mp->msg)) ||
	   (sc->sc_msg_data_size > 65535) ||
	   (sc->sc_msg_size < sizeof(*mp)) ||
	   (sc->sc_msg_size > 65535))
	{
	    /* invalid parameters */
	    capilib_free_app(sc);
	    return CAPI_ERROR_INVALID_PARAM;
	}

	/* get total size of cache in bytes */
	size *= sc->sc_msg_size;

	/* allocate memory for messages */
	sc->sc_msg_start_ptr = malloc(size);

	sc->sc_msg_end_ptr = ADD_BYTES(sc->sc_msg_start_ptr, size);

	if (sc->sc_msg_start_ptr == NULL) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	/* initialize messages */

	mp = sc->sc_msg_start_ptr;

	while(1)
	{
	    mp->state = ST_FREE;
	    mp->next = ADD_BYTES(mp, sc->sc_msg_size);

	    if(((void *)(mp->next)) < sc->sc_msg_end_ptr)
	    {
	        mp = mp->next;
	    }
	    else
	    {
		mp->next = NULL;
		break;
	    }
	}

	sc->sc_msg_free_list = sc->sc_msg_start_ptr;
   
	/* start D-channel driver */

	if (capilib_do_ioctl_sub(sc, CAPI_START_D_CHANNEL_REQ, NULL) < 0) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	/* set non-blocking operation last ! */
	if (capilib_do_ioctl_sub(sc, FIONBIO, &temp) < 0) {
	    capilib_free_app(sc);
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	app_id_ptr[0] = sc->sc_app_id;

	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_release - release a CAPI application
 *
 * @param app_id               Application id from registration.
 *
 * @retval 0                   Application release was successful.
 *
 * @retval Else                An error happened.
 *
 *---------------------------------------------------------------------------*/
uint16_t
capi20_release(uint32_t app_id)
{
    struct app_softc *sc;

    sc = capilib_find_app_by_id(app_id);
    
    if(sc == NULL)
    {
        /* application ID does not exist */
        return CAPI_ERROR_INVALID_APPLICATION_ID;
    }
   
    /* free this application 
     * (and close file descriptor)
     */
    capilib_free_app(sc);

    return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_put_message - send a CAPI message
 *
 * @param app_id               Application id from registration.
 *
 * @param buf_ptr              Pointer to the CAPI message to send.
 *
 * @retval 0                   The message was accepted.
 *
 * @retval Else                An error happened. The message was
 *                             not accepted.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_put_message(uint32_t app_id, void *buf_ptr)
{
	struct app_softc * sc;
	struct data_buffer *mp = NULL;
	struct capi_message_encoded *pmsg = buf_ptr;
	struct iovec iov[3];
	uint16_t error = 0;
	uint16_t cmd;
	uint16_t app_id_old;

	sc = capilib_find_app_by_id(app_id);
	if (sc == NULL) {
	    /* application ID does not exist */
	    return CAPI_ERROR_INVALID_APPLICATION_ID;
	}

	cmd = HEADER_CMD(pmsg);

	if((cmd == htole16(CAPI_REQ(DATA_B3))) ||
	   (cmd == htole16(CAPI_P_REQ(DATA_B3))))
	{
	    iov[1].iov_base = (void *) pmsg;
	    iov[1].iov_len = le16toh(HEADER_LEN(pmsg));

	    if(sizeof(void *) <= 4)
	    {
	        uint32_t temp;

		if(iov[1].iov_len < (30-8))
		{
		    /* fatal error */
		    capilib_free_app(sc);
		    return CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
		}

		temp = le32toh(pmsg->data.DATA_B3_REQ.dwPtr_1);

		iov[2].iov_base = temp + ((uint8_t *)0);
		iov[2].iov_len = le16toh(pmsg->data.DATA_B3_REQ.wLen);
	    }
	    else if(sizeof(void *) <= 8)
	    {
	        uint64_t temp;

		if(iov[1].iov_len < 30)
		{
		    /* fatal error */
		    capilib_free_app(sc);
		    return CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
		}

		temp = le64toh(pmsg->data.DATA_B3_REQ.qwPtr_2);

		iov[2].iov_base = temp + ((uint8_t *)0);
		iov[2].iov_len = le16toh(pmsg->data.DATA_B3_REQ.wLen);
	    }
	    else
	    {
	        /* fatal error */
	        capilib_free_app(sc);
	        return CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
	    }

	    /* extra checks */

	    if((iov[2].iov_len == 0) || 
	       (iov[2].iov_base == NULL))
	    {
	        /* set length to zero */
	        iov[2].iov_len = 0;
		iov[2].iov_base = NULL;

		/* update length */
		pmsg->data.DATA_B3_REQ.wLen = 0;
	    }
	}
	else
	{
	    /* if the message is a data-b3-response, 
	     * release the cache of the message 
	     * referenced by "wHandle"
	     */
	    if((cmd == htole16(CAPI_RESP(DATA_B3))) ||
	       (cmd == htole16(CAPI_P_RESP(DATA_B3))))
	    {
		mp = capilib_find_data_buffer_by_handle(
		    sc, pmsg->data.DATA_B3_RESP.wHandle);

		if(mp == NULL)
		{
		    /* ignore invalid data handle */
		    return 0;
		}

		/* restore initial data handle */
		pmsg->data.DATA_B3_RESP.wHandle =
		  mp->wHandle;
	    }

	    iov[1].iov_base = (void *) pmsg;
	    iov[1].iov_len = le16toh(HEADER_LEN(pmsg));

	    iov[2].iov_base = NULL;
	    iov[2].iov_len = 0;
	}

	/* update application ID */
	app_id_old = HEADER_APP(pmsg);
	HEADER_APP(pmsg) = sc->sc_app_id_real;

	if (sc->sc_backend == CAPI_BACKEND_TYPE_BINTEC) {
		uint32_t temp;
	  
		temp = (iov[1].iov_len + 
			iov[2].iov_len) + 2;

		if (temp > 65535) {
		    /* fatal error */
		    error = CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
		    goto done;
		}

		/* need to rewrite the command value */

		cmd = le16toh(HEADER_CMD(pmsg));
		cmd = capi20_command_unpack(cmd);
		HEADER_CMD(pmsg) = htole16(cmd);

		/* big endian length field preceedes CAPI message */

		sc->sc_temp[0] = (temp >> 8);
		sc->sc_temp[1] = (temp & 0xFF);

		iov[0].iov_base = (void *)(sc->sc_temp);
		iov[0].iov_len = 2; /* bytes */
	} else {
		iov[0].iov_base = NULL;
		iov[0].iov_len = 0;
	}

	while (writev(sc->sc_fd, iov, 3) < 0)
	{
	    if (errno == EINTR) {
	        continue;
	    }

	    error = 
	      (errno == ENXIO) ? CAPI_ERROR_CAPI_NOT_INSTALLED :
	      (errno == EINVAL) ? CAPI_ERROR_ILLEGAL_MSG_PARAMETER :
	      CAPI_ERROR_OS_RESOURCE_ERROR;

	    break;
	}

 done:
	/* restore application ID in case the 
	 * application is reusing this message
	 */
	HEADER_APP(pmsg) = app_id_old;

	if(mp)
	{
	    /* free data buffer last */

	    mp->next = sc->sc_msg_free_list;
	    mp->state = ST_FREE;

	    sc->sc_msg_free_list = mp;
	}

	if (error) {
	    /* fatal error */
 	    capilib_free_app(sc);
	}
	return error;
}

int
capilib_get_message_sub(struct app_softc *sc, void *buf, uint16_t msg_len)
{
	uint16_t temp;
	int len;

	if (sc->sc_backend == CAPI_BACKEND_TYPE_BINTEC) {

	    /* need to get the length bytes first */
	    while ((len = read(sc->sc_fd, sc->sc_temp, 2)) < 0) {
	        if (errno == EINTR) {
		    continue;
		}
		return (-1);
	    }

	    if (len != 2) {
	        /* An invalid length is an irrecoverable error */
	        errno = ENODEV;
	        return (-1);
	    }

	    temp = ((sc->sc_temp[0] << 8) |
	      (sc->sc_temp[1])) - 2;

	    if (temp > msg_len) {
		/* An invalid length is an irrecoverable error */
	        errno = ENODEV;
		return (-1);
	    } else {
	        msg_len = temp;
	    }
	}

	while ((len = read(sc->sc_fd, buf, msg_len)) < 0) {
	    if (errno == EINTR) {
	        continue;
	    }
	    break;
	}
	return (len);
}

/*---------------------------------------------------------------------------*
 *	capi20_get_message - receive a CAPI message
 *
 * NOTE: if the message is a data-b3-indication the message pointer is
 * valid until the corresponding data-b3-confirmation has been
 * sent. Else the message pointer is only valid until the next
 * capi20_get_message() call!
 *
 * @param app_id               Application id from registration.
 *
 * @param buf_pp               Pointer to the pointer that should be
 *                             pointed to the CAPI message.
 *
 * @retval 0                   A message is available and its address is 
 *                             stored in the pointer pointed to by "buf_pp".
 *
 * @retval Else                An error happened and the pointer pointed to
 *                             by "buf_pp" is set to zero.
 *---------------------------------------------------------------------------*/
uint16_t 
capi20_get_message(uint32_t app_id, uint8_t **buf_pp)
{
	struct app_softc *sc;
	struct data_buffer *mp;
	uint8_t *data_ptr;
	uint16_t cmd;
	uint16_t msg_len;
	uint16_t retval;
	int len;

	if(buf_pp == NULL)
	{
	    /* the application usually has to
	     * respond to messages, so
	     * skipping messages this
	     * way is not allowed !
	     */
	    return CAPI_ERROR_INVALID_PARAM;
	}

	sc = capilib_find_app_by_id(app_id);
    
	if(sc == NULL)
	{
	    /* application ID does not exist */
	    retval = CAPI_ERROR_INVALID_APPLICATION_ID;
	    goto error;
	}

	mp = sc->sc_msg_free_list;

	if(mp == NULL)
	{
	    /* there is no data buffer where
	     * the message can be stored
	     */
	    retval = CAPI_ERROR_BUSY;
	    goto error;
	}

 again:
	len = capilib_get_message_sub(sc, &mp->msg, sc->sc_msg_data_size);

	if (len < 0) {

	    /* read error */

	    if((errno == EBUSY) ||
	       (errno == ENODEV))
	    {
	      /* there is an unrecoverable
	       * data error and one has just
	       * got to free this application
	       * and close its file descriptor !
	       */
	      capilib_free_app(sc);
	    }

	    retval = 
	      ((errno == EAGAIN) ? CAPI_ERROR_GET_QUEUE_EMPTY :
	       (errno == EWOULDBLOCK) ? CAPI_ERROR_GET_QUEUE_EMPTY :
	       (errno == ENXIO) ? CAPI_ERROR_CAPI_NOT_INSTALLED :
	       (errno == EINVAL) ? CAPI_ERROR_ILLEGAL_MSG_PARAMETER :
	       CAPI_ERROR_OS_RESOURCE_ERROR);
	    goto error;
	}

	/* ignore too short messages */

	if(len < sizeof(struct CAPI_HEADER_ENCODED))
	{
	    goto again;
	}

	msg_len = le16toh(HEADER_LEN(&mp->msg));

	/* ignore messages with invalid length */

	if((len < msg_len) ||
	   (msg_len < sizeof(struct CAPI_HEADER_ENCODED)))
	{
	    /* shouldn't happen */
	    goto again;
	}

	cmd = HEADER_CMD(&(mp->msg));

	if((cmd == htole16(CAPI_IND(DATA_B3))) ||
	   (cmd == htole16(CAPI_P_IND(DATA_B3))))
	{
	    /* store a copy of the data handle */
	    mp->wHandle =
	      mp->msg.data.DATA_B3_IND.wHandle;

	    /* update wHandle */
	    mp->msg.data.DATA_B3_IND.wHandle =
	      capilib_find_handle_by_data_buffer(sc, mp);

	    /* update wLen */
	    mp->msg.data.DATA_B3_IND.wLen = 
	      htole16(len - msg_len);

	    data_ptr = ADD_BYTES(&mp->msg, msg_len);

	    /* update data pointer */

	    if(sizeof(void *) <= 4)
	    {
	        uint32_t temp;

		if(msg_len < (30-8))
		{
		    /* shouldn't happen */
		    goto again;
		}

		temp = data_ptr - ((uint8_t *)0);

		mp->msg.data.DATA_B3_IND.dwPtr_1 = htole32(temp);
		mp->msg.data.DATA_B3_IND.qwPtr_2 = 0;
	    }
	    else if(sizeof(void *) <= 8)
	    {
	        uint64_t temp;

		if(msg_len < 30)
		{
		    /* shouldn't happen */
		    goto again;
		}

		temp = data_ptr - ((uint8_t *)0);

		mp->msg.data.DATA_B3_IND.dwPtr_1 = 0;
		mp->msg.data.DATA_B3_IND.qwPtr_2 = htole64(temp);
	    }
	    else
	    {
	        /* shouldn't happen */
	        goto again;
	    }

	    if(mp->next == NULL)
	    {
	        /* out of buffer */

		struct {
		  struct CAPI_HEADER_ENCODED head;
		  struct CAPI_DATA_B3_RESP_ENCODED data;
		} __packed resp = { { 0 } };

		/* acknowledge the frame here, 
		 * hence the application won't
		 * acknowledge it!
		 */
		resp.head = mp->msg.head;
		resp.head.wCmd = htole16(CAPI_P_RESP(DATA_B3));
		resp.head.wLen = htole16(sizeof(resp));
		resp.data.wHandle = mp->wHandle;

		retval = capi20_put_message(app_id, &resp);
		if (retval) {
		    goto error;
		}

	        /* this last data buffer
		 * is needed to receive
		 * call control information
		 */
		retval = CAPI_ERROR_GET_QUEUE_OVERFLOW;
		goto error;
	    }

	    /* remove data buffer from 
	     * free list
	     */
	    sc->sc_msg_free_list = mp->next;
	    mp->next = NULL;
	    mp->state = ST_USED;
	}
	else if((cmd == htole16(CAPI_IND(DISCONNECT_B3))) ||
		(cmd == htole16(CAPI_P_IND(DISCONNECT_B3))))
	{
	    /* free all data buffers associated with
	     * this B-channel connection
	     */
	    capilib_free_data_buffer_by_cid(sc, HEADER_CID(&(mp->msg)));
	}

	/* update application id */

	HEADER_APP(&(mp->msg)) = htole16(sc->sc_app_id);

	/* store pointer to the message */
	buf_pp[0] = (uint8_t *)&(mp->msg);
	return 0;

 error:
	buf_pp[0] = NULL;
	return retval;
}

/*---------------------------------------------------------------------------*
 *	capi20_wait_for_message - wait for an incoming CAPI message
 *
 *
 * @param app_id               Application id from registration.
 *
 * @param timeval_ptr          If set wait will timeout. Else wait
 *                             is infinite or until an error happens.
 *
 * @retval 0                    A new CAPI message can be received 
 *                              by calling "capi20_get_message()".
 *
 * @retval CAPI_ERROR_GET_QUEUE_EMPTY  The wait has timed out.
 *
 * @retval Else                 An error has happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_wait_for_message(uint32_t app_id, struct timeval *timeval_ptr)
{
	struct pollfd  pollfd = { 0 };
	struct timeval tvEnd;
	struct timeval tvCurr;
	struct timeval tvTmp;
	int            timeout;
	int            error;

	/* get file number */
	pollfd.fd = capi20_fileno(app_id);

	if(pollfd.fd < 0)
	{
	    return CAPI_ERROR_INVALID_APPLICATION_ID;
	}

	pollfd.events = (POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | 
			 POLLERR | POLLHUP | POLLNVAL);
	pollfd.revents = 0;

	/* determine the end time value; needed if the poll 
	 * operation is interrupted by a signal and it must 
	 * be restarted with a reduced timeout
	 */
	timerclear(&tvEnd);
	(void) gettimeofday(&tvCurr, NULL);

	if(timeval_ptr != NULL)
	{
	    timeradd (&tvCurr, timeval_ptr, &tvEnd);
	}
	else
	{
	    tvEnd = tvCurr;
	}
   
	/* loop to (re-)start the poll operation 
	 * if interrupted by a signal 
	 */
	do
	{
	    /* compute the timeout in ms */
	    if(timeval_ptr != NULL)
	    {
	        timerclear (&tvCurr);

		(void) gettimeofday (&tvCurr, NULL);

		if(timercmp (&tvEnd, &tvCurr, > ))
		{
		    timersub (&tvEnd, &tvCurr, &tvTmp);

		    if(tvTmp.tv_sec > 1000) 
		    {
		        tvTmp.tv_sec = 1000;
		    }

		    if(tvTmp.tv_usec > (1000*1000))
		    {
		        tvTmp.tv_usec = (1000*1000);
		    }

		    timeout = (tvTmp.tv_sec * 1000) + (tvTmp.tv_usec / 1000);
		}
		else
		{
		    timeout = 0;
		}
	    }
	    else
	    {
	        timeout = INFTIM;
	    }

	    /* do a poll on the file descriptor */
	    error = poll(&pollfd, 1, timeout);

	} while((error < 0) && (errno == EINTR));
   
	/* evaluate the result of the poll call */
	if(error == 0)
	{
	    /* timeout occurred, no CAPI message available */
	    return CAPI_ERROR_GET_QUEUE_EMPTY;
	}

	if(error < 0)
	{
	    if((errno == EINVAL) || (errno == EFAULT))
	      return CAPI_ERROR_INVALID_APPLICATION_ID;
	    else
	      return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	/* message available */
	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_get_manufacturer - get manufacturer name of a controller
 *
 * @param cbe                    Pointer to CAPI backend.
 *
 * @param controller            Controller number.
 *
 * @param buf_ptr               Pointer to where the string should be stored.
 *
 * @param buf_len               Maximum string length, including 
 *                              terminating zero. Default is 64.
 *
 * @retval 0                    No error.
 *
 * @retval Else                 An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_get_manufacturer(struct capi20_backend *cbe, uint32_t controller,
    char *buf_ptr, uint16_t buf_len)
{
	struct capi_get_manufacturer_req req = { 0 };
	uint16_t error;

	if (buf_len == 0) {
	    /* nothing to do */
	    return 0;
	}

	if (buf_ptr == NULL) {
	    return (CAPI_ERROR_INVALID_PARAM);
	}

	req.controller = controller;

	error = capilib_do_ioctl(cbe, CAPI_GET_MANUFACTURER_REQ, &req);

	if(error)
	{
	    return error;
	}

        snprintf(buf_ptr, buf_len, "%s", &req.name[0]);
	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_get_version - get version of CAPI implementation
 *
 * @param cbe                    Pointer to CAPI backend.
 *
 * @param controller            Controller number.
 *
 * @param buf_ptr               Pointer to a series of bytes
 *                              where the version should be stored.
 *                              See "capi20.h" and "struct capi_get_version_req"
 *                              for more information.
 *
 * @param buf_len               Number of bytes wanted.
 *                              Default is 8.
 *
 * @retval 0                    No error.
 *
 * @retval Else                 An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_get_version(struct capi20_backend *cbe, uint32_t controller,
    char *buf_ptr, uint16_t buf_len)
{
	struct capi_get_version_req req = { 0 };
	uint16_t error;

	if (buf_len == 0) {
	    /* nothing to do */
	    return 0;
	}

	if (buf_ptr == NULL) {
	    return (CAPI_ERROR_INVALID_PARAM);
	}

	req.controller = controller;

	error = capilib_do_ioctl(cbe, CAPI_GET_VERSION_REQ, &req);

	if(error)
	{
	    return error;
	}

	/* fill in stack version */

	req.version.stack_major = CAPI_STACK_VERSION / 100;
	req.version.stack_minor = CAPI_STACK_VERSION % 100;

	if(buf_len > sizeof(req.version))
	{
	    bzero(ADD_BYTES(buf_ptr, sizeof(req.version)), 
		  buf_len - sizeof(req.version));

	    /* truncate */
	    buf_len = sizeof(req.version);
	}

	bcopy(&req.version, buf_ptr, buf_len);

	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_get_serial_number - get serial number of a controller
 *
 * @param cbe                    Pointer to CAPI backend.
 *
 * @param controller            Controller number.
 *
 * @param buf_ptr               Pointer to where the string should be stored.
 *
 * @param buf_len               Maximum string length, including 
 *                              terminating zero. Default is 8.
 *
 * @retval 0                    No error.
 *
 * @retval Else                 An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_get_serial_number(struct capi20_backend *cbe, uint32_t controller,
   char *buf_ptr, uint16_t buf_len)
{
	struct capi_get_serial_req req = { 0 };
	uint16_t error;

	if (buf_len == 0) {
	    /* nothing to do */
	    return 0;
	}

	if (buf_ptr == NULL) {
	    return (CAPI_ERROR_INVALID_PARAM);
	}

	req.controller = controller;

	error = capilib_do_ioctl(cbe, CAPI_GET_SERIAL_REQ, &req);

	if(error)
	{
	    return error;
	}

	if(buf_len > sizeof(req.serial_number))
	{
	    bzero(ADD_BYTES(buf_ptr, sizeof(req.serial_number)), 
		  buf_len - sizeof(req.serial_number));

	    /* truncate */
	    buf_len = sizeof(req.serial_number);
	}

	/* make sure that there is a terminating zero */
	req.serial_number[buf_len-1] = 0;

	bcopy(&req.serial_number, buf_ptr, buf_len);

	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_get_profile - get the profile of a controller
 *
 * @param cbe                    Pointer to CAPI backend.
 *
 * @param controller            Controller number.
 *
 * @param buf_ptr               Pointer to where the profile should be stored.
 *                              The profile is stored in host byte-endian 
 *                              format!
 *
 * @param buf_len               Maximum profile length in bytes. Default is 
 *                              "sizeof(struct CAPI_PROFILE_DATA_ENCODED)".
 *
 * @retval 0                    No error.
 *
 * @retval Else                 An error happened.
 *---------------------------------------------------------------------------*/
uint16_t 
capi20_get_profile(struct capi20_backend *cbe, uint32_t controller, 
    void *buf_ptr, uint16_t buf_len)
{
	struct capi_get_profile_req req = { 0 };
	uint16_t error;

	if (buf_len == 0) {
	    /* nothing to do */
	    return 0;
	}

	if (buf_ptr == NULL) {
	    return (CAPI_ERROR_INVALID_PARAM);
	}

	req.controller = controller;

	error = capilib_do_ioctl(cbe, CAPI_GET_PROFILE_REQ, &req);

	if(error)
	{
	    return error;
	}

	if(buf_len > sizeof(req.profile))
	{
	    bzero(ADD_BYTES(buf_ptr, sizeof(req.profile)), 
		  buf_len - sizeof(req.profile));

	    /* truncate */
	    buf_len = sizeof(req.profile);
	}

	bcopy(&req.profile, buf_ptr, buf_len);

	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_is_installed - test if CAPI is installed
 *
 * @param cbe                    Pointer to CAPI backend.
 *
 * @retval 0                              CAPI is installed.
 * @retval CAPI_ERROR_CAPI_NOT_INSTALLED  CAPI is not installed.
 * @retval Else                           An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_is_installed(struct capi20_backend *cbe)
{
	/* CAPI is installed even if there are no
	 * controllers, because some controllers
	 * are pluggable !
	 */
	return (capilib_do_ioctl(cbe, CAPI_IOCTL_TEST_REQ, NULL));
}

/*---------------------------------------------------------------------------*
 *	capi20_fileno - get the file number associated with an application
 *
 *
 * @param app_id               The application id from registration.
 *
 * @retval -1                  "app_id" is not a valid application id for the
 *                             calling process.
 *
 * @retval Else                The file number that can be used for polling
 *                             is returned.
 *---------------------------------------------------------------------------*/
int
capi20_fileno(uint32_t app_id)
{
	struct app_softc *sc;

	sc = capilib_find_app_by_id(app_id);

	if(sc == NULL)
	{
	    /* invalid "app_id" */
	    return -1;
	}

	if(sc->sc_fd <= 0)
	{
	    /* invalid file number */
	    return -1;
	}
	return sc->sc_fd;
}


/*---------------------------------------------------------------------------*
 *	capi_firmware_download - reset and/or download firmware for an
 *	                         active controller
 *
 * This call is used to download software and settings to active 
 * controllers. How many data blocks and their content is driver specific.
 *
 * @note This call is only allowed if the calling process' user id is 0 or the
 *       user is member of the group with id 0.
 *
 * @param cbe                    Pointer to CAPI backend.
 *
 * @param controller            Controller number.
 *
 * @param protocols_ptr         Pointer to array of "struct isdn_dr_prot".
 *
 * @param protocols_len         Number of array units. If zero, controller
 *                              will only be reset and disabled.
 *
 * @retval 0                    No error.
 *
 * @retval Else                 An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi_firmware_download(struct capi20_backend *cbe, uint32_t controller, 
  struct isdn_dr_prot *protocols_ptr, uint16_t protocols_len)
{
	struct isdn_download_request req = { 0 };

	if(protocols_ptr == NULL) 
	{
	    /* nothing to do */
	    return 0;
	}

	req.controller = controller;
	req.numprotos = protocols_len;
	req.protocols = protocols_ptr;

	return (capilib_do_ioctl(cbe, I4B_CTRL_DOWNLOAD, &req));
}

/*---------------------------------------------------------------------------*
 *	capilib_setup_message_decoded - setup CAPI translation tags
 *
 * @param mp                   Pointer to decoded CAPI message.
 *
 * @retval Any                 Packed command.
 *---------------------------------------------------------------------------*/
static uint16_t
capilib_setup_message_decoded(struct capi_message_decoded *mp)
{
	uint8_t B_PROTOCOL_INIT = 0;
	uint8_t ADDITIONAL_INFO_INIT = 0;
	uint16_t cmd;

	cmd = mp->head.wCmd;

	/* the command is packed so that
	 * switching becomes more effective
	 */
	cmd = CAPI_COMMAND_PACK(cmd);

#define B_PROTOCOL_STRUCT (mp->B_PROTOCOL)
#define ADDITIONAL_INFO_STRUCT (mp->ADDITIONAL_INFO)

	switch(cmd) 
	{
	    CAPI_COMMANDS(CAPI_MAKE_CASES, mp);

	default:
	    ((uint8_t *)&(mp->data))[0] = IE_END;
	    break;
	}

	if(B_PROTOCOL_INIT)
	{
	    CAPI_INIT_2(CAPI_B_PROTOCOL, &B_PROTOCOL_STRUCT);
	}

	if(ADDITIONAL_INFO_INIT)
	{
	    CAPI_INIT_2(CAPI_ADDITIONAL_INFO, &ADDITIONAL_INFO_STRUCT);
	}

#undef B_PROTOCOL_STRUCT 
#undef ADDITIONAL_INFO_STRUCT
	return cmd;
}

/*---------------------------------------------------------------------------*
 *	capi_translate_to_message_decoded - translate a CAPI message
 *
 * @param mp                   Pointer to where the CAPI message 
 *                             should be decoded.
 *
 * @param buf_ptr              Pointer to CAPI message that should
 *                             be decoded. It is assumed that the 
 *                             header is valid.
 *---------------------------------------------------------------------------*/
void
capi_translate_to_message_decoded(struct capi_message_decoded *mp, 
				  void *buf_ptr)
{
	uint16_t buf_len;

	/* set default value */
	bzero(mp, sizeof(*mp));

	/* setup decode tags */
	CAPI_INIT(CAPI_HEADER, &(mp->head));

	/* decode header */
	capi20_decode(buf_ptr, sizeof(struct CAPI_HEADER_ENCODED), &(mp->head));

#if !defined(CAPI_NO_COMPAT_CODE)
	/* be nice with Linux applications, 
	 * before the command is packed
	 */

	mp->Command = mp->head.wCmd & 0xFF;
	mp->Subcommand = mp->head.wCmd >> 8;
	mp->Messagenumber = mp->head.wNum;
#endif

	/* length should already have been verified */

	buf_len = mp->head.wLen - sizeof(struct CAPI_HEADER_ENCODED);
	buf_ptr = ADD_BYTES(buf_ptr, sizeof(struct CAPI_HEADER_ENCODED));

	/* setup decode tags and pack command */
	mp->head.wCmd = 
	  capilib_setup_message_decoded(mp);

	/* decode rest of message */
	capi20_decode(buf_ptr, buf_len, &(mp->data));
	return;
}

/*---------------------------------------------------------------------------*
 *	capi_get_message_decoded - receive a decoded CAPI message
 *
 * @param mp                   Pointer to where the CAPI message 
 *                             should be decoded.
 *
 * @param app_id               Application id from registration.
 *
 * @retval 0                   A message is available.
 *
 * @retval Else                An error happened. No message available.
 *---------------------------------------------------------------------------*/
uint16_t
capi_get_message_decoded(struct capi_message_decoded *mp, uint32_t app_id)
{
	uint8_t *buf_ptr;
	uint16_t error;

	if(mp == NULL)
	{
	    return CAPI_ERROR_INVALID_PARAM;
	}
	
	error = capi20_get_message(app_id, &buf_ptr);

	if(error)
	{
	    return error;
	}

	capi_translate_to_message_decoded(mp, buf_ptr);

	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi_translate_from_message_decoded - translate a CAPI message
 *
 * @param mp                   Pointer to decoded CAPI message.
 *
 * @param buf_ptr              Pointer to where CAPI message
 *                             should be encoded.
 *
 * @param buf_len              Maximum encoded length.
 *
 *---------------------------------------------------------------------------*/
void
capi_translate_from_message_decoded(struct capi_message_decoded *mp, 
				    void *buf_ptr, uint16_t buf_len)
{
  	uint16_t len;

	/* setup decode tags */
	CAPI_INIT(CAPI_HEADER, &(mp->head));

	/* setup decode tags and pack command */
	mp->head.wCmd = 
	  capilib_setup_message_decoded(mp);

	len = capi20_encode(buf_ptr, buf_len, &(mp->head));

	len += capi20_encode(ADD_BYTES(buf_ptr, len), 
			   buf_len - len, &(mp->data));

	/* update length */
	((u_int16_p_t *)(buf_ptr))->data = htole16(len);

	return;
}

/*---------------------------------------------------------------------------*
 *	capi_put_message_decoded - send a decoded CAPI message
 *
 * @param mp                   Pointer to decoded CAPI message.
 *
 * @retval 0                   The message was accepted.
 *
 * @retval Else                An error happened. The message was
 *                             not accepted.
 *---------------------------------------------------------------------------*/
uint16_t
capi_put_message_decoded(struct capi_message_decoded *mp)
{
	struct capi_message_encoded msg;

	capi_translate_from_message_decoded(mp, &msg, sizeof(msg));

	return capi20_put_message(mp->head.wApp, &msg);
}

/*---------------------------------------------------------------------------*
 *	capi20_get_errstr - get error description
 *
 * @param wError               Error number to lookup description for.
 *
 * @retval Any                 Pointer to a zero terminated string.
 *---------------------------------------------------------------------------*/
const char *
capi20_get_errstr(uint16_t wError)
{
	struct error
	{
	    uint16_t wError;
	    const char * desc;
	};

	static const struct error errors[] =
	{
	    CAPI_ERRORS(CAPI_MAKE_ERROR_DESC,)
	    { 0, NULL }
	};

	const struct error *ptr = &errors[0];

	static const char *
	  MAKE_TABLE(Q850_CAUSES,DESC,[0x80]);

	if(wError == 0)
	{
	    return ("no error");
	}

	if((wError & 0xFF00) == 0x3600)
	{
	    return ("0x36XX: unknown supplementary service error");
	}

	if((wError & 0xFF00) == 0x3700)
	{
	    return ("0x37XX: unknown supplementary service context error");
	}

	if((wError & 0xFF00) == 0x3400)
	{
	    wError &= 0x7F;

	    if(Q850_CAUSES_DESC[wError])
	      return (Q850_CAUSES_DESC[wError]);
	    else
	      return ("0x34XX: unknown Q.850 cause");
	}

	while(ptr->desc)
	{
	    if(ptr->wError == wError)
	    {
		return (ptr->desc);
	    }
	    ptr++;
	}
	return ("unknown CAPI error");
}

static uint16_t
capilib_print_byte_array(char *dst, uint16_t len, 
		      uint8_t *buf_ptr, uint16_t buf_len)
{
        uint16_t len_old = len;
	uint16_t temp;
	uint8_t c;
	uint8_t e = 1;

	while(buf_len--)
	{
	    c = buf_ptr[0];

	    /* only pass non-control characters
	     * (see "man ascii"
	     */
	    if((c < 0x20) || (c > 0x7e)) c = '?';

	    temp = snprintf(dst, len, "0x%02x '%c'%s%s",
			    buf_ptr[0], c, buf_len ? ", " : ".",
			    ((!(e & 3)) && (buf_len)) ? 
			    "\n                     "
			    "            = " : "" );
	    if(temp > len) temp = len;

	    dst += temp;
	    len -= temp;

	    e++;

	    buf_ptr++;

	}

	temp = snprintf(dst, len, "\n");

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	/* return number of bytes used */
	return (len_old - len);
}

static uint16_t
capilib_print_struct(char *dst, uint16_t len, uint8_t *buf_ptr)
{
	uint16_t len_old = len;
	uint16_t buf_len;
	uint16_t temp;

	/* get STRUCT length */

	if(buf_ptr == NULL)
	{
	    buf_len = 0;
	} 
	else if(buf_ptr[0] == 0xff)
	{
	    buf_len = buf_ptr[1] | (buf_ptr[2] << 8);
	    buf_len += 3;

	    if(buf_len < 3) buf_len = -1;
	}
	else
	{
	    buf_len = buf_ptr[0];
	    buf_len += 1;

	    if(buf_len < 1) buf_len = -1;
	}

	if(buf_len)
	{
	    temp = capilib_print_byte_array(dst, len, buf_ptr, buf_len);
	}
	else
	{
	    temp = snprintf(dst, len, "(empty)\n");
	}

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	/* return number of bytes used */
	return (len_old - len);
}

static const struct debug debug[] = {
  CAPI_COMMANDS(CAPI_MAKE_DEBUG_1,)
  { IE_END, 0, 0, NULL }
};

/*---------------------------------------------------------------------------*
 *	capi_message_decoded_to_string - print a decoded message
 *
 * @param dst                  Pointer to where string should be written.
 *
 * @param len                  Maximum string length including terminating
 *                             zero.
 *
 * @param mp                   Pointer to decoded CAPI message.
 *
 * @retval Any                 Length of string excluding terminating
 *                             zero.
 *---------------------------------------------------------------------------*/
uint16_t
capi_message_decoded_to_string(char *dst, uint16_t len, 
			       const struct capi_message_decoded *mp)
{
    const struct debug *ptr = &debug[0];

    uint16_t len_old = len;
    uint16_t temp;
    void *var;

    /* lookup command */

    while(ptr->field)
    {
      if((ptr->what == IE_END) &&
	 ((ptr->size == mp->head.wCmd) ||
	  (ptr->offset == mp->head.wCmd)))
      {
	  break;
      }
      ptr++;
    }

    /* dump header */

    if(ptr->field)
    {
        temp = snprintf(dst, len, "%s {\n", ptr->field);
	ptr++;
    }
    else
    {
        temp = snprintf(dst, len, "UNKNOWN {\n");
    }

    if(temp > len) temp = len;

    dst += temp;
    len -= temp;

    temp = snprintf
      (dst, len,
       " header {\n"
       "  WORD       %-20s= 0x%04x\n"
       "  WORD       %-20s= 0x%04x\n"
       "  WORD       %-20s= 0x%04x\n"
       "  WORD       %-20s= 0x%04x\n"
       "  DWORD      %-20s= 0x%08x\n"
       " }\n"
       " data {\n",
       
       "wLen", mp->head.wLen,
       "wApp", mp->head.wApp,
       "wCmd", mp->head.wCmd,
       "wNum", mp->head.wNum,
       "dwCid", mp->head.dwCid);

    if(temp > len) temp = len;

    dst += temp;
    len -= temp;

    /* dump data */

    while(1)
    {
      var = ADD_BYTES(mp, ptr->offset);

      switch(ptr->what) {
      default:
	goto done;

      case IE_BYTE:
	temp = snprintf(dst, len, "  BYTE       %-20s= 0x%02x\n",
			ptr->field, ((uint8_t *)(var))[0]);
	break;

      case IE_WORD:
	temp = snprintf(dst, len, "  WORD       %-20s= 0x%04x\n",
			ptr->field, ((u_int16_p_t *)(var))->data);
	break;

      case IE_DWORD:
	temp = snprintf(dst, len, "  DWORD      %-20s= 0x%08x\n",
			ptr->field, ((u_int32_p_t *)(var))->data);
	break;

      case IE_QWORD:
	temp = snprintf(dst, len, "  QWORD      %-20s= 0x%08x%08x\n",
			ptr->field, 
			(uint32_t)(((u_int64_p_t *)(var))->data >> 32),
			(uint32_t)(((u_int64_p_t *)(var))->data >> 0));
	break;

      case IE_STRUCT:
	temp = snprintf(dst, len, "  STRUCT     %-20s= ", ptr->field);

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	temp = capilib_print_struct(dst, len, ((void_p_t *)(var))->data);
	break;

      case IE_BYTE_ARRAY:
	temp = snprintf(dst, len, "  BYTE_ARRAY %-20s= ", ptr->field);

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	temp = capilib_print_byte_array(dst, len, var, ptr->size);
	break;
      }

      if(temp > len) temp = len;

      dst += temp;
      len -= temp;
      ptr++;
    }

 done:

      temp = snprintf(dst, len, " }\n" "}\n");

      if(temp > len) temp = len;

      dst += temp;
      len -= temp;

      /* return number of bytes used 
       * (excluding terminating zero)
       */
      if(len_old && (len_old == len)) len--;

      return (len_old - len);
}

/*---------------------------------------------------------------------------*
 *	capi_get_command_string - get command description
 *
 * @param wCmd                 Command number.
 *
 * @retval Any                 Pointer to zero terminated string.
 *---------------------------------------------------------------------------*/
const char *
capi_get_command_string(uint16_t wCmd)
{
	const struct debug *ptr = &debug[0];

	/* lookup command */

	while(ptr->field)
	{
	    if((ptr->what == IE_END) &&
	       ((ptr->size == wCmd) ||
		(ptr->offset == wCmd)))
	    {
	        return (ptr->field);
	    }
	    ptr++;
	}
	return ("UNKNOWN");
}

/*---------------------------------------------------------------------------*
 *	capi20_decode
 *
 * @param ptr                  Encoded message pointer.
 * @param len                  Encoded message length.
 * @param ie                   Decoded message pointer.
 *
 * @retval Any                 Actual encoded length that was decoded.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_decode(void *ptr, uint16_t len, void *ie)
{
	uint16_t len_old = len;
	uint8_t what;

	while(1)
	{
	  what = ((uint8_t *)(ie))[0];
	  ie = ADD_BYTES(ie, 1);

	  switch(what) {
	  case IE_END:
	    goto done;

	  case IE_BYTE:
	    if(len >= 1)
	    {
	      ((uint8_t *)(ie))[0] =
		((uint8_t *)(ptr))[0];
	      ie = ADD_BYTES(ie, 1);
	      ptr = ADD_BYTES(ptr, 1);
	      len -= 1;
	      break;
	    }
	    ((uint8_t *)(ie))[0] = 0;
	    ie = ADD_BYTES(ie, 1);
	    goto ie_error;

	  case IE_WORD:
	    if(len >= 2)
	    {
	      ((u_int16_p_t *)(ie))->data =
		le16toh(((u_int16_p_t *)(ptr))->data);

	      ie = ADD_BYTES(ie, 2);
	      ptr = ADD_BYTES(ptr, 2);
	      len -= 2;
	      break;
	    }
	    ((u_int16_p_t *)(ie))->data = 0;
	    ie = ADD_BYTES(ie, 2);
	    goto ie_error;

	  case IE_DWORD:
	    if(len >= 4)
	    {
	      ((u_int32_p_t *)(ie))->data =
		le32toh(((u_int32_p_t *)(ptr))->data);

	      ie = ADD_BYTES(ie, 4);
	      ptr = ADD_BYTES(ptr, 4);
	      len -= 4;
	      break;
	    }
	    ((u_int32_p_t *)(ie))->data = 0;
	    ie = ADD_BYTES(ie, 4);
	    goto ie_error;

	  case IE_QWORD:
	    if(len >= 8)
	    {
	      ((u_int64_p_t *)(ie))->data = 
		le64toh(((u_int64_p_t *)(ptr))->data);

	      ie = ADD_BYTES(ie, 8);
	      ptr = ADD_BYTES(ptr, 8);
	      len -= 8;
	      break;
	    }
	    ((u_int64_p_t *)(ie))->data = 0;
	    ie = ADD_BYTES(ie, 8);
	    goto ie_error;

	  ie_error:
	    /* skip remaining data */
	    ptr = ADD_BYTES(ptr, len);
	    len = 0;
	    break;

	  case IE_STRUCT_CAPI:
	    /* pre-store pointer to CAPI structure */
	    ((void_p_t *)(ADD_BYTES(ie,2)))->data = ptr;

	  case IE_STRUCT_DECODED:
	  case IE_STRUCT_DECODED_EMPTY:
	  case IE_STRUCT: /* default */
	    {
	      register uint16_t temp;

	      if(len >= 1)
	      {
		  if(((uint8_t *)(ptr))[0] == 0xFF)
		  {
		      /* length is escaped */

		      if(len >= 3)
		      {
			  temp = ((uint8_t *)(ptr))[2];
			  temp <<= 8;
			  temp |= ((uint8_t *)(ptr))[1];

			  ptr = ADD_BYTES(ptr, 3);
			  len -= 3;
		      }
		      else
		      {
			  temp = 255; /* dummy */
		      }
		  }
		  else
		  {
		      temp = ((uint8_t *)(ptr))[0];

		      ptr = ADD_BYTES(ptr, 1);
		      len -= 1;
		  }
	      }
	      else
	      {
		  temp = 255; /* dummy */
	      }

	      /* check structure length */

	      if(temp > len)
	      {
		  /* skip remaining data and
		   * set lengths to zero
		   */
		  ptr = ADD_BYTES(ptr, len);
		  len = 0;
		  temp = 0;
	      }

	      /* store length */
	      if(what == IE_STRUCT)
	      {
		  ((u_int16_p_t *)(ie))->data = temp;
		  ie = ADD_BYTES(ie, 2);

		  /* store pointer to data structure */
		  ((void_p_t *)(ie))->data = ptr;
	      }
	      else
	      {
		  /* this field should not be used */
		  ((u_int16_p_t *)(ie))->data = 0;
		  ie = ADD_BYTES(ie, 2);

		  /* ((void_p_t *)(ie))->data should already have
		   * been set !
		   */
	      }

	      if((what == IE_STRUCT_DECODED) || 
		 (what == IE_STRUCT_DECODED_EMPTY))
	      {
		  /* need to update the structure type, 
		   * hence the application might want to 
		   * check this field !
		   */
		  (((uint8_t *)(ie)) - 3)[0] = (temp) ?
		    IE_STRUCT_DECODED : IE_STRUCT_DECODED_EMPTY;

		  capi20_decode(ptr, temp, ((void_p_t *)(ie))->data);
	      }
	      else
	      {
		  if(temp == 0)
		  {
		      /* In case this is a CAPI structure, one
		       * needs to set the pointer to a valid 
		       * structure, hence the real structure
		       * might have an invalid length field !
		       * Else try to catch the cases where the
		       * software is looking up fields before
		       * checking the length!
		       */
		      static const uint8_t empty_struct[8] = { 0 };

		      /* structure is empty or non-existing */
		      ((void_p_t *)(ie))->data = (void *)&empty_struct;
		  }
	      }
	      ie = ADD_BYTES(ie, sizeof(void *));
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  case IE_BYTE_ARRAY:
	    {
	      register uint16_t temp;

	      temp = ((u_int16_p_t *)(ie))->data;

	      ie = ADD_BYTES(ie, 2);

	      if(temp > len)
	      {
		  /* part of array is zero ! */
		  bzero(ADD_BYTES(ie,len), temp-len);
		  temp = len;
	      }

	      bcopy(ptr, ie, temp);

	      ie = ADD_BYTES(ie, temp);
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  default:
	    /* invalid information element - should not happen */
	    goto done;
	  }
	}
 done:
	return (len_old - len);
}

/*---------------------------------------------------------------------------*
 *	capi20_encode
 *
 * @param ptr                  Encoded message pointer.
 * @param len                  Encoded message length.
 * @param ie                   Decoded message pointer.
 *
 * @retval Any                 Actual encoded length.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_encode(void *ptr, uint16_t len, void *ie)
{
	uint16_t len_old = len;
	uint8_t what;

	while(1)
	{
	  what = ((uint8_t *)(ie))[0];
	  ie = ADD_BYTES(ie, 1);

	  switch(what) {
	  case IE_END:
	    goto done;

	  case IE_BYTE:
	    if(len < 1) goto error; /* overflow */

	    ((uint8_t *)(ptr))[0] = 
	      ((uint8_t *)(ie))[0];
	    ie = ADD_BYTES(ie, 1);
	    ptr = ADD_BYTES(ptr, 1);
	    len -= 1;
	    break;

	  case IE_WORD:
	    if(len < 2) goto error; /* overflow */

	    ((u_int16_p_t *)(ptr))->data =
	      htole16(((u_int16_p_t *)(ie))->data);
	    ie = ADD_BYTES(ie, 2);
	    ptr = ADD_BYTES(ptr, 2);
	    len -= 2;
	    break;

	  case IE_DWORD:
	    if(len < 4) goto error; /* overflow */

	    ((u_int32_p_t *)(ptr))->data =
	      htole32(((u_int32_p_t *)(ie))->data);
	    ie = ADD_BYTES(ie, 4);
	    ptr = ADD_BYTES(ptr, 4);
	    len -= 4;
	    break;

	  case IE_QWORD:
	    if(len < 8) goto error; /* overflow */

	    ((u_int64_p_t *)(ptr))->data =
	      htole64(((u_int64_p_t *)(ie))->data);
	    ie = ADD_BYTES(ie, 8);
	    ptr = ADD_BYTES(ptr, 8);
	    len -= 8;
	    break;

	  case IE_STRUCT_DECODED_EMPTY:
	    if(len < 1) goto error; /* overflow */

	    ((uint8_t *)(ptr))[0] = 0;
	    ie = ADD_BYTES(ie, sizeof(void *)+2);
	    ptr = ADD_BYTES(ptr, 1);
	    len -= 1;
	    break;

	  case IE_STRUCT_DECODED:
	  case IE_STRUCT_CAPI:
	  case IE_STRUCT:
	    {
	      register uint16_t temp;
	      register void *var;

	      temp = ((u_int16_p_t *)(ie))->data;
	      ie = ADD_BYTES(ie, 2);
	      var = ((void_p_t *)(ie))->data;
	      ie = ADD_BYTES(ie, sizeof(void *));

	      if(what == IE_STRUCT_CAPI)
	      {
		  if(var == NULL)
		  {
		      temp = 0;
		  }
		  else
		  {
		    /* get length field from CAPI structure */

		    if(((uint8_t *)(var))[0] == 0xFF)
		    {
		      temp = ((((uint8_t *)(var))[1]) |
			      (((uint8_t *)(var))[2] << 8));
		      var = ADD_BYTES(var, 3);
		    }
		    else
		    {
		      temp = ((uint8_t *)(var))[0];
		      var = ADD_BYTES(var, 1);
		    }
		  }
	      }

	      if(what == IE_STRUCT_DECODED)
	      {
		  if(len < 1) goto error; /* overflow */

		  temp = capi20_encode(ADD_BYTES(ptr,1),len-1,var);

		  if(temp >= 0x00FF)
		  {
		      register uint16_t temp2 = temp;

		      if((len-temp2) < 3) goto error; /* overflow */

		      /* move all data up two bytes */
		      while(temp2--)
		      {
			  *(((uint8_t *)(ptr)) + 3 + temp2) =
			    *(((uint8_t *)(ptr)) + 1 + temp2);
		      }
		  }
	      }

	      if(temp >= 0x00FF)
	      {
		if((len < 3) || (temp > (len - 3)))
		{
		    goto error; /* overflow */
		}

		((uint8_t *)(ptr))[0] = 0xFF;
		((uint8_t *)(ptr))[1] = temp;
		((uint8_t *)(ptr))[2] = temp >> 8;
		ptr = ADD_BYTES(ptr, 3);
		len -= 3;
	      }
	      else
	      {
		if((len < 1) || (temp > (len - 1)))
		{
		    goto error; /* overflow */
		}

		((uint8_t *)(ptr))[0] = temp;
		ptr = ADD_BYTES(ptr, 1);
		len -= 1;
	      }

	      if(what != IE_STRUCT_DECODED)
	      {
		  bcopy(var, ptr, temp);
	      }
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  case IE_BYTE_ARRAY:
	    {
	      register uint16_t temp;

	      temp = ((u_int16_p_t *)(ie))->data;

	      if(len < temp) goto error; /* overflow */

	      ie = ADD_BYTES(ie, 2);

	      bcopy(ie, ptr, temp);
	      ie = ADD_BYTES(ie, temp);
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  default:
	    /* invalid information element - should not happen */
	    goto error;
	  }
	}
 error:
 done:
	return (len_old - len);
}

/*------------------------------------------------------------------------*
 *	capi20_command_unpack
 *
 * @param cmd                  Any CAPI command
 *
 * @retval Any                 Unpacked version of CAPI command
 *------------------------------------------------------------------------*/
uint16_t
capi20_command_unpack(uint16_t wCmd)
{
#define	CAPI_COMMAND_UNPACK_SUB(n, ENUM, value)			\
  case CAPI_P_REQ(ENUM): wCmd = CAPI_REQ(ENUM); break;		\
  case CAPI_P_IND(ENUM): wCmd = CAPI_IND(ENUM); break;		\
  case CAPI_P_CONF(ENUM): wCmd = CAPI_CONF(ENUM); break;	\
  case CAPI_P_RESP(ENUM): wCmd = CAPI_RESP(ENUM); break;

  switch (wCmd) {
    CAPI_COMMANDS(CAPI_COMMAND_UNPACK_SUB, )
    default:
      break;
  }
  return (wCmd);
}

/*------------------------------------------------------------------------*
 *	capi20_command_pack
 *
 * @param cmd                  Any CAPI command
 *
 * @retval Any                 Packed version of CAPI command
 *------------------------------------------------------------------------*/
uint16_t
capi20_command_pack(uint16_t wCmd)
{
#define	CAPI_COMMAND_PACK_SUB(n, ENUM, value)			\
  case CAPI_REQ(ENUM): wCmd = CAPI_P_REQ(ENUM); break;		\
  case CAPI_IND(ENUM): wCmd = CAPI_P_IND(ENUM); break;		\
  case CAPI_CONF(ENUM): wCmd = CAPI_P_CONF(ENUM); break;	\
  case CAPI_RESP(ENUM): wCmd = CAPI_P_RESP(ENUM); break;

  switch (wCmd) {
    CAPI_COMMANDS(CAPI_COMMAND_PACK_SUB, )
    default:
      break;
  }
  return (wCmd);
}

