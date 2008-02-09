/*-
 * Copyright (c) 2000-2003 Thomas Wintergerst. All rights reserved.
 *
 * Copyright (c) 2005-2006 Hans Petter Selasky. All rights reserved.
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
#define CAPI_MAKE_TRANSLATOR
#define panic(args...) err(1, args)

#include <i4b/include/capi20.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_cause.h>

/* this structure is used to cache incoming 
 * B-channel data!
 */
struct data_buffer
{
  struct data_buffer * next; /* pointer to next data buffer on free list */

  u_int8_t state;
# define   ST_FREE 0 /* default */
# define   ST_USED 1

  u_int8_t unused; /* for sake of alignment */

  u_int16_t wHandle; /* copy of wHandle from data-b3-indication */

  struct capi_message_encoded msg;

  u_int8_t data[0]; /* B-channel data follows */

} __packed;

/* this structure stores information about
 * a registered application
 */
struct app_softc
{
  u_int32_t            sc_app_id; /* applet number (internal) */
  u_int16_t            sc_app_id_real; /* applet number (external) */

  int                  sc_fd; /* CAPI device filenumber */

  void *               sc_msg_start_ptr; /* pointer to start of 
					  * messages (inclusive) 
					  */
  void *               sc_msg_end_ptr;   /* pointer to end of 
					  * messages (exclusive) 
					  */
  u_int32_t            sc_msg_size; /* size of a message in bytes */

  u_int32_t            sc_msg_data_size; /* size of data part
					  * of a message in bytes
					  */

  struct data_buffer * sc_msg_free_list; /* pointer to first free message */

  u_int32_t            sc_max_connections;

  struct app_softc *   sc_next;
};

static struct app_softc *app_sc_root;

static struct app_softc *
find_app_by_id(u_int32_t app_id)
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

static struct app_softc *
alloc_app()
{
	struct app_softc *sc;
	u_int32_t app_id = 0;

	/* find an unused ID first */

 again:
	sc = find_app_by_id(app_id);

	if(sc)
	{
	    app_id++;
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

	/* insert "sc" into list */

	sc->sc_next = app_sc_root;
	app_sc_root = sc;

	return sc;
}

static void
free_app(struct app_softc *sc)
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

static int
open_capi(struct app_softc *sc)
{
	sc->sc_fd = open(CAPI_DEVICE_NAME, O_RDWR | O_NONBLOCK);

	return sc->sc_fd;
}

static void
free_data_buffer_by_cid(struct app_softc *sc, u_int32_t dwCid)
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
find_data_buffer_by_handle(struct app_softc *sc, u_int16_t wHandle)
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

static __inline u_int16_t
find_handle_by_data_buffer(struct app_softc *sc, struct data_buffer *mp)
{
	if((((void *)mp) >= sc->sc_msg_start_ptr) &&
	   (((void *)mp) < sc->sc_msg_end_ptr))
	{
	    return ((((u_int8_t *)mp) - 
		     ((u_int8_t *)sc->sc_msg_start_ptr))
		    / sc->sc_msg_size);
	}
	return -1;
}

static u_int16_t
capi_do_ioctl(u_int32_t cmd, void *data)
{
	struct app_softc *sc;

	sc = alloc_app();

	if(sc == NULL)
	{
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	if(open_capi(sc) < 0)
	{
	    free_app(sc);
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	if(ioctl(sc->sc_fd, cmd, data) < 0)
	{
	    free_app(sc);
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	free_app(sc);
	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_register - CAPI application registration
 *
 * @param max_logical_connections  Maximum number of active B-channel 
 *                                 connections.

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
u_int16_t
capi20_register(u_int32_t max_logical_connections,
		u_int32_t max_b_data_blocks,
		u_int32_t max_b_data_len,
		u_int32_t *app_id_ptr,
		u_int32_t stack_version)
{
	struct app_softc *sc;
	struct data_buffer *mp;
	struct capi_register_req req = { /* zero */ };
	u_int32_t size;
	int32_t temp = 1;

	if(app_id_ptr == NULL)
	{
	    return CAPI_ERROR_INVALID_PARAM;
	}

	if(stack_version != CAPI_STACK_VERSION)
	{
	   return CAPI_ERROR_UNSUPPORTED_VERSION;
	}

	sc = alloc_app();

	if(sc == NULL)
	{
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_OS_RESOURCE_ERROR;
	}

	if(open_capi(sc) < 0)
	{
	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	/* set stack version first */
	if(ioctl(sc->sc_fd, CAPI_SET_STACK_VERSION_REQ, &stack_version) < 0)
	{
	   /* an error here typically means that the
	    * header files and this library must be 
	    * re-compiled and re-installed
	    */
	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_UNSUPPORTED_VERSION;
	}

	/* set non-blocking operation */
	if(ioctl(sc->sc_fd, FIONBIO, &temp) < 0)
	{
	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	/* register this application */
	req.max_logical_connections = max_logical_connections;
	req.max_b_data_blocks       = max_b_data_blocks;
	req.max_b_data_len          = max_b_data_len;

	if(ioctl(sc->sc_fd, CAPI_REGISTER_REQ, &req) < 0)
	{
	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}

	if(req.max_b_data_len != max_b_data_len)
	{
	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_INVALID_BUFFER_SIZE;
	}

	sc->sc_app_id_real = req.app_id;
	sc->sc_max_connections = req.max_logical_connections;

	sc->sc_msg_data_size = 
	  sizeof(mp->msg) + req.max_b_data_len;

	sc->sc_msg_size = 
	  sizeof(*mp) + req.max_b_data_len;

	size = (req.max_logical_connections *
		req.max_b_data_blocks) + 1;

	if((size < 1) ||
	   (size > 65536) || 
	   (sc->sc_msg_data_size < sizeof(mp->msg)) ||
	   (sc->sc_msg_data_size > 65536) ||
	   (sc->sc_msg_size < sizeof(*mp)) ||
	   (sc->sc_msg_size > 65536))
	{
	    /* invalid parameters */

	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_INVALID_PARAM;
	}

	/* get total size of cache in bytes */
	size *= sc->sc_msg_size;

	/* allocate memory for messages */
	sc->sc_msg_start_ptr = malloc(size);

	sc->sc_msg_end_ptr = ADD_BYTES(sc->sc_msg_start_ptr, size);

	if(sc->sc_msg_start_ptr == NULL)
	{
	    free_app(sc);
	    app_id_ptr[0] = -1;
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
   
	app_id_ptr[0] = sc->sc_app_id;

	/* start D-channel driver */

	if(ioctl(sc->sc_fd, CAPI_START_D_CHANNEL_REQ, NULL) < 0)
	{
	    free_app(sc);
	    app_id_ptr[0] = -1;
	    return CAPI_ERROR_CAPI_NOT_INSTALLED;
	}
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
u_int16_t
capi20_release(u_int32_t app_id)
{
    struct app_softc *sc;

    sc = find_app_by_id(app_id);
    
    if(sc == NULL)
    {
        /* application ID does not exist */
        return CAPI_ERROR_INVALID_APPLICATION_ID;
    }
   
    /* free this application 
     * (and close file descriptor)
     */
    free_app(sc);

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
u_int16_t
capi20_put_message(u_int32_t app_id, void *buf_ptr)
{
	struct app_softc * sc;
	struct data_buffer *mp = NULL;
	struct capi_message_encoded *pmsg = buf_ptr;
	struct iovec iov[2];
	u_int16_t error = 0;
	u_int16_t cmd;
	u_int16_t app_id_old;

	sc = find_app_by_id(app_id);
    
	if(sc == NULL)
	{
	    /* application ID does not exist */
	    return CAPI_ERROR_INVALID_APPLICATION_ID;
	}

	cmd = HEADER_CMD(pmsg);

	if((cmd == htole16(CAPI_REQ(DATA_B3))) ||
	   (cmd == htole16(CAPI_P_REQ(DATA_B3))))
	{
	    iov[0].iov_base = (void *) pmsg;
	    iov[0].iov_len = le16toh(HEADER_LEN(pmsg));

	    if(sizeof(void *) <= 4)
	    {
	        u_int32_t temp;

		if(iov[0].iov_len < (30-8))
		{
		    return CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
		}

		temp = le32toh(pmsg->data.DATA_B3_REQ.dwPtr_1);

		iov[1].iov_base = temp + ((uint8_t *)0);
		iov[1].iov_len = le16toh(pmsg->data.DATA_B3_REQ.wLen);
	    }
	    else if(sizeof(void *) <= 8)
	    {
	        u_int64_t temp;

		if(iov[0].iov_len < 30)
		{
		    return CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
		}

		temp = le64toh(pmsg->data.DATA_B3_REQ.qwPtr_2);

		iov[1].iov_base = temp + ((uint8_t *)0);
		iov[1].iov_len = le16toh(pmsg->data.DATA_B3_REQ.wLen);
	    }
	    else
	    {
	        return CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
	    }

	    /* extra checks */

	    if((iov[1].iov_len == 0) || 
	       (iov[1].iov_base == NULL))
	    {
	        /* set length to zero */
	        iov[1].iov_len = 0;
		iov[1].iov_base = NULL;

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
		mp = find_data_buffer_by_handle(sc, pmsg->data.DATA_B3_RESP.wHandle);

		if(mp == NULL)
		{
		    /* ignore invalid data handle */
		    return 0;
		}

		/* restore initial data handle */
		pmsg->data.DATA_B3_RESP.wHandle =
		  mp->wHandle;
	    }

	    iov[0].iov_base = (void *) pmsg;
	    iov[0].iov_len = le16toh(HEADER_LEN(pmsg));

	    iov[1].iov_base = NULL;
	    iov[1].iov_len = 0;
	}

	/* update application ID */
	app_id_old = HEADER_APP(pmsg);
	HEADER_APP(pmsg) = sc->sc_app_id_real;

	if(writev(sc->sc_fd, &iov[0], 2) < 0)
	{
	    error = 
	      (errno == ENXIO) ? CAPI_ERROR_CAPI_NOT_INSTALLED :
	      (errno == EINVAL) ? CAPI_ERROR_ILLEGAL_MSG_PARAMETER :
	      CAPI_ERROR_OS_RESOURCE_ERROR;
	}

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

	if(error)
	{
	    /* this usually means that operation
	     * cannot be continued
	     */
	    free_app(sc);
	}
	return error;
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
u_int16_t 
capi20_get_message(u_int32_t app_id, u_int8_t **buf_pp)
{
	struct app_softc *sc;
	struct data_buffer *mp;
	uint8_t *data_ptr;
	u_int16_t cmd;
	u_int16_t msg_len;
	u_int16_t retval;
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

	sc = find_app_by_id(app_id);
    
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
	len = read(sc->sc_fd, &mp->msg, sc->sc_msg_data_size);

	if(len < 0)
	{
	    /* read error */

	    if((errno == EBUSY) ||
	       (errno == ENODEV))
	    {
	      /* there is an unrecoverable
	       * data error and one has just
	       * got to free this application
	       * and close its file descriptor !
	       */
	      free_app(sc);
	    }

	    retval = 
	      ((errno == EWOULDBLOCK) ? CAPI_ERROR_GET_QUEUE_EMPTY :
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
	      find_handle_by_data_buffer(sc, mp);

	    /* update wLen */
	    mp->msg.data.DATA_B3_IND.wLen = 
	      htole16(len - msg_len);

	    data_ptr = ADD_BYTES(&mp->msg, msg_len);

	    /* update data pointer */

	    if(sizeof(void *) <= 4)
	    {
	        u_int32_t temp;

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
	        u_int64_t temp;

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
		} __packed resp = { /* zero */ };

		/* acknowledge the frame here, 
		 * hence the application won't
		 * acknowledge it!
		 */
		resp.head = mp->msg.head;
		resp.head.wCmd = htole16(CAPI_P_RESP(DATA_B3));
		resp.head.wLen = htole16(sizeof(resp));
		resp.data.wHandle = mp->wHandle;

		if(write(sc->sc_fd, &resp, sizeof(resp)) < 0)
		{
		    /* this error is unrecoverable and
		     * all active calls must be
		     * disconnected including the 
		     * call that caused the failure !
		     */
		    free_app(sc);

		    retval = CAPI_ERROR_OS_RESOURCE_ERROR;
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
	    free_data_buffer_by_cid(sc, HEADER_CID(&(mp->msg)));
	}

	/* update application id */

	HEADER_APP(&(mp->msg)) = htole16(sc->sc_app_id);

	/* store pointer to the message */
	buf_pp[0] = (u_int8_t *)&(mp->msg);
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
u_int16_t
capi20_wait_for_message(u_int32_t app_id, struct timeval *timeval_ptr)
{
	struct pollfd  pollfd = { /* zero */ };
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
u_int16_t
capi20_get_manufacturer(u_int32_t controller, u_int8_t *buf_ptr, 
			u_int16_t buf_len)
{
	struct capi_get_manufacturer_req req = { /* zero */ };
	u_int16_t error;

	if((buf_ptr == NULL) ||
	   (buf_len == 0))
	{
	    /* nothing to do */
	    return 0;
	}

	req.controller = controller;

	error = capi_do_ioctl(CAPI_GET_MANUFACTURER_REQ, &req);

	if(error)
	{
	    return error;
	}

        snprintf((void *)buf_ptr, buf_len, "%s", &req.name[0]);
	return 0;
}

/*---------------------------------------------------------------------------*
 *	capi20_get_version - get version of CAPI implementation
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
u_int16_t
capi20_get_version(u_int32_t controller, u_int8_t *buf_ptr, u_int16_t buf_len)
{
	u_int16_t error;

	struct capi_get_version_req req = { /* zero */ };

	if((buf_ptr == NULL) ||
	   (buf_len == 0))
	{
	    /* nothing to do */
	    return 0;
	}

	req.controller = controller;

	error = capi_do_ioctl(CAPI_GET_VERSION_REQ, &req);

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
u_int16_t
capi20_get_serial_number(u_int32_t controller, u_int8_t *buf_ptr,
			 u_int16_t buf_len)
{
	u_int16_t error;

	struct capi_get_serial_req req = { /* zero */ };

	if((buf_ptr == NULL) ||
	   (buf_len == 0))
	{
	    /* nothing to do */
	    return 0;
	}

	req.controller = controller;

	error = capi_do_ioctl(CAPI_GET_SERIAL_REQ, &req);

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
u_int16_t 
capi20_get_profile(u_int32_t controller, void *buf_ptr, u_int16_t buf_len)
{
	u_int16_t error;

	struct capi_get_profile_req req = { /* zero */ };

	if((buf_ptr == NULL) ||
	   (buf_len == 0))
	{
	    /* nothing to do */
	    return 0;
	}

	req.controller = controller;

	error = capi_do_ioctl(CAPI_GET_PROFILE_REQ, &req);

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
 * @retval 0                              CAPI is installed.
 * @retval CAPI_ERROR_CAPI_NOT_INSTALLED  CAPI is not installed.
 * @retval Else                           An error happened.
 *---------------------------------------------------------------------------*/
u_int16_t
capi20_is_installed(void)
{
	/* CAPI is installed even if there are no
	 * controllers, because some controllers
	 * are pluggable !
	 */
	return capi_do_ioctl(CAPI_IOCTL_TEST_REQ, NULL);
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
capi20_fileno(u_int32_t app_id)
{
	struct app_softc *sc;

	sc = find_app_by_id(app_id);

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
u_int16_t
capi_firmware_download(u_int32_t controller, struct isdn_dr_prot *protocols_ptr,
		       u_int16_t protocols_len)
{
	struct isdn_download_request req = { /* zero */ };

	if(protocols_ptr == NULL) 
	{
	    /* nothing to do */
	    return 0;
	}

	req.controller = controller;
	req.numprotos = protocols_len;
	req.protocols = protocols_ptr;

	return capi_do_ioctl(I4B_CTRL_DOWNLOAD, &req);
}

/*---------------------------------------------------------------------------*
 *	capi_setup_message_decoded - setup CAPI translation tags
 *
 * @param mp                   Pointer to decoded CAPI message.
 *
 * @retval Any                 Packed command.
 *---------------------------------------------------------------------------*/
static u_int16_t
capi_setup_message_decoded(struct capi_message_decoded *mp)
{
	u_int8_t B_PROTOCOL_INIT = 0;
	u_int8_t ADDITIONAL_INFO_INIT = 0;
	u_int16_t cmd;

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
	    ((u_int8_t *)&(mp->data))[0] = IE_END;
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
	u_int16_t buf_len;

	/* set default value */
	bzero(mp, sizeof(*mp));

	/* setup decode tags */
	CAPI_INIT(CAPI_HEADER, &(mp->head));

	/* decode header */
	capi_decode(buf_ptr, sizeof(struct CAPI_HEADER_ENCODED), &(mp->head));

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
	  capi_setup_message_decoded(mp);

	/* decode rest of message */
	capi_decode(buf_ptr, buf_len, &(mp->data));
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
u_int16_t
capi_get_message_decoded(struct capi_message_decoded *mp, u_int32_t app_id)
{
	u_int8_t *buf_ptr;
	u_int16_t error;

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
				    void *buf_ptr, u_int16_t buf_len)
{
  	u_int16_t len;

	/* setup decode tags */
	CAPI_INIT(CAPI_HEADER, &(mp->head));

	/* setup decode tags and pack command */
	mp->head.wCmd = 
	  capi_setup_message_decoded(mp);

	len = capi_encode(buf_ptr, buf_len, &(mp->head));

	len += capi_encode(ADD_BYTES(buf_ptr, len), 
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
u_int16_t
capi_put_message_decoded(struct capi_message_decoded *mp)
{
	struct capi_message_encoded msg;

	capi_translate_from_message_decoded(mp, &msg, sizeof(msg));

	return capi20_put_message(mp->head.wApp, &msg);
}

/*---------------------------------------------------------------------------*
 *	capi_get_error_string - get error description
 *
 * @param wError               Error number to lookup description for.
 *
 * @retval Any                 Pointer to a zero terminated string.
 *---------------------------------------------------------------------------*/
const u_int8_t *
capi_get_error_string(u_int16_t wError)
{
	struct error
	{
	    u_int16_t wError;
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
	    return ((const void *)("no error"));
	}

	if((wError & 0xFF00) == 0x3600)
	{
	    return ((const void *)
		    ("0x36XX: unknown supplementary service error"));
	}

	if((wError & 0xFF00) == 0x3700)
	{
	    return ((const void *)
		    ("0x37XX: unknown supplementary service context error"));
	}

	if((wError & 0xFF00) == 0x3400)
	{
	    wError &= 0x7F;

	    if(Q850_CAUSES_DESC[wError])
	      return ((const void *)
		      (Q850_CAUSES_DESC[wError]));
	    else
	      return ((const void *)
		      ("0x34XX: unknown Q.850 cause"));
	}

	while(ptr->desc)
	{
	    if(ptr->wError == wError)
	    {
		return ((const void *)(ptr->desc));
	    }
	    ptr++;
	}
	return ((const void *)
		("unknown CAPI error"));
}

static u_int16_t
capi_print_byte_array(u_int8_t *dst, u_int16_t len, 
		      u_int8_t *buf_ptr, u_int16_t buf_len)
{
        u_int16_t len_old = len;
	u_int16_t temp;
	u_int8_t c;
	u_int8_t e = 1;

	while(buf_len--)
	{
	    c = buf_ptr[0];

	    /* only pass non-control characters
	     * (see "man ascii"
	     */
	    if((c < 0x20) || (c > 0x7e)) c = '?';

	    temp = snprintf((void *)dst, len, "0x%02x '%c'%s%s",
			    buf_ptr[0], c, buf_len ? ", " : ".",
			    ((!(e & 3)) && (buf_len)) ? 
			    "\n                     "
			    "            = " : "" );
	    if(temp > len) temp = len;

	    dst += temp;
	    len -= temp;

	    e++;

	    buf_ptr++;

	};
	temp = snprintf((void *)dst, len, "\n");

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	/* return number of bytes used */
	return (len_old - len);
}

static u_int16_t
capi_print_struct(u_int8_t *dst, u_int16_t len, u_int8_t *buf_ptr)
{
	u_int16_t len_old = len;
	u_int16_t buf_len;
	u_int16_t temp;

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
	    temp = capi_print_byte_array(dst, len, buf_ptr, buf_len);
	}
	else
	{
	    temp = snprintf((void *)dst, len, "(empty)\n");
	}

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	/* return number of bytes used */
	return (len_old - len);
}

 struct debug {
   u_int8_t what;
   u_int16_t size;
   u_int16_t offset;
   const char *field;
} __packed;

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
u_int16_t
capi_message_decoded_to_string(u_int8_t *dst, u_int16_t len, 
			       const struct capi_message_decoded *mp)
{
    const struct debug *ptr = &debug[0];

    u_int16_t len_old = len;
    u_int16_t temp;
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
        temp = snprintf((void *)dst, len, "%s {\n", ptr->field);
	ptr++;
    }
    else
    {
        temp = snprintf((void *)dst, len, "UNKNOWN {\n");
    }

    if(temp > len) temp = len;

    dst += temp;
    len -= temp;

    temp = snprintf
      ((void *)dst, len,
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
	temp = snprintf((void *)dst, len, "  BYTE       %-20s= 0x%02x\n",
			ptr->field, ((u_int8_t *)(var))[0]);
	break;

      case IE_WORD:
	temp = snprintf((void *)dst, len, "  WORD       %-20s= 0x%04x\n",
			ptr->field, ((u_int16_p_t *)(var))->data);
	break;

      case IE_DWORD:
	temp = snprintf((void *)dst, len, "  DWORD      %-20s= 0x%08x\n",
			ptr->field, ((u_int32_p_t *)(var))->data);
	break;

      case IE_QWORD:
	temp = snprintf((void *)dst, len, "  QWORD      %-20s= 0x%08x%08x\n",
			ptr->field, 
			(u_int32_t)(((u_int64_p_t *)(var))->data >> 32),
			(u_int32_t)(((u_int64_p_t *)(var))->data >> 0));
	break;

      case IE_STRUCT:
	temp = snprintf((void *)dst, len, "  STRUCT     %-20s= ", ptr->field);

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	temp = capi_print_struct(dst, len, ((void_p_t *)(var))->data);
	break;

      case IE_BYTE_ARRAY:
	temp = snprintf((void *)dst, len, "  BYTE_ARRAY %-20s= ", ptr->field);

	if(temp > len) temp = len;

	dst += temp;
	len -= temp;

	temp = capi_print_byte_array(dst, len, var, ptr->size);
	break;
      }

      if(temp > len) temp = len;

      dst += temp;
      len -= temp;
      ptr++;
    }

 done:

      temp = snprintf((void *)dst, len, " }\n" "}\n");

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
const u_int8_t *
capi_get_command_string(u_int16_t wCmd)
{
	const struct debug *ptr = &debug[0];

	/* lookup command */

	while(ptr->field)
	{
	    if((ptr->what == IE_END) &&
	       ((ptr->size == wCmd) ||
		(ptr->offset == wCmd)))
	    {
	        return ((const void *)(ptr->field));
	    }
	    ptr++;
	}
	return ((const void *)("UNKNOWN"));
}
