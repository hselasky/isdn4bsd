/*-
 * Copyright (c) 2015 Hans Petter Selasky. All rights reserved.
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
 */

#ifdef I4B_GLOBAL_INCLUDE_FILE
#include I4B_GLOBAL_INCLUDE_FILE
#endif

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <sys/syspool.h>
#include <sys/syspublic.h>
#include <sys/kernel.h>
#include <sys/kern_sx.h>
#include <sys/kern_thread.h>

#include <lwip/api.h>

#define	CAPI_WORKERS	4

SYSPOOL_GROW(LWIP_TCP_PCB_POOL, CAPI_WORKERS);
SYSPOOL_GROW(LWIP_NETCONN_POOL, CAPI_WORKERS);
SYSPOOL_GROW(LWIP_NETBUF_POOL, 3 * CAPI_WORKERS);
SYSPOOL_GROW(LWIP_TCP_PCB_LISTEN_POOL, 1);
SYSPOOL_GROW(capi_ai_softc_pool, CAPI_WORKERS);
SYSPOOL_GROW(cdev_pool, 1);
SYSPOOL_GROW(file_pool, CAPI_WORKERS);
SYSPOOL_GROW(zone_mbuf, 64);
SYSPOOL_GROW(zone_clust, 4);

#define	CAPI_MAKE_IOCTL
#include <i4b/include/capi20.h>

#define	CAPISERVER_HDR_SIZE 4		/* bytes */

/* define supported commands */
enum {
	CAPISERVER_CMD_CAPI_MSG,
	CAPISERVER_CMD_CAPI_REGISTER,
	CAPISERVER_CMD_CAPI_MANUFACTURER,
	CAPISERVER_CMD_CAPI_VERSION,
	CAPISERVER_CMD_CAPI_SERIAL,
	CAPISERVER_CMD_CAPI_PROFILE,
	CAPISERVER_CMD_CAPI_START,
};

#define	CAPISERVER_BUF_MAX (2048 + 256)

extern struct cdev *capi_dev;

struct capiconn {
	struct file *file;
	struct netconn *conn;
	struct netbuf *rxbuf;
	uint8_t *curr_recv_data;
	uint16_t curr_recv_len;
	uint8_t	buffer[CAPISERVER_BUF_MAX] __aligned(4);
};

static struct capiconn capiconn[CAPI_WORKERS];
static struct netconn *capiconn_listen;
static struct sx capiconn_sx;

static void
capiserver_close(struct capiconn *cc)
{
	struct netconn *netconn;

	netconn = cc->conn;
	if (netconn != NULL) {
		cc->conn = NULL;
		netconn_close(netconn);
		netconn_delete(netconn);
	}
	if (cc->rxbuf != NULL) {
		netbuf_delete(cc->rxbuf);
		cc->rxbuf = NULL;
	}
}

static int
capiserver_write(struct capiconn *cc, void *buf, uint16_t len, int flags)
{
	if (netconn_write(cc->conn, buf, len, flags) != ERR_OK)
		return (-1);
	return (len);
}

static int
capiserver_read(struct capiconn *cc, void *buf, uint16_t len)
{
	int retval = 0;

	while (len > 0) {
		uint16_t rem;

		if (cc->rxbuf == NULL) {
			err_t err;

			err = netconn_recv(cc->conn, &cc->rxbuf);
			if (err == ERR_CLSD) {
				cc->rxbuf = NULL;
				capiserver_close(cc);
				return (-1);
			}
			if (err == ERR_TIMEOUT) {
				return (-2);
			}
			if (err != ERR_OK) {
				cc->rxbuf = NULL;
				capiserver_close(cc);
				return (-1);
			}
			netconn_set_recvtimeout(cc->conn, 0);
			netbuf_data(cc->rxbuf,
			    (void **)&cc->curr_recv_data,
			    &cc->curr_recv_len);
		}
		/* find minimum length */
		rem = MIN(cc->curr_recv_len, len);

		if (buf != NULL)
			memcpy(buf, cc->curr_recv_data, rem);

		cc->curr_recv_data += rem;
		cc->curr_recv_len = (uint16_t)(cc->curr_recv_len - rem);
		if (buf != NULL)
			buf = (uint8_t *)buf + rem;
		len -= rem;
		retval += rem;

		if (cc->curr_recv_len == 0) {
			if (netbuf_next(cc->rxbuf) == -1) {
				netbuf_delete(cc->rxbuf);
				cc->rxbuf = NULL;
			} else {
				netbuf_data(cc->rxbuf,
				    (void **)&cc->curr_recv_data,
				    &cc->curr_recv_len);
			}
		}
	}
	return (retval);
}

#define	CAPI_FWD(x) (x) = le32toh(x)
#define	CAPI_REV(x) (x) = htole32(x)

static int
capiserver_ioctl(struct capiconn *cc, uint32_t cmd, uint32_t ioctl_cmd,
    void *buffer, ssize_t length)
{
	uint8_t header[CAPISERVER_HDR_SIZE] __aligned(4);
	int err;

	if (length != IOCPARM_LEN(ioctl_cmd)) {
		err = EINVAL;
		goto error;
	}
	if (ioctl_cmd == CAPI_REGISTER_REQ) {
		struct capi_register_req *req = buffer;

		CAPI_FWD(req->max_logical_connections);
		CAPI_FWD(req->max_b_data_blocks);
		CAPI_FWD(req->max_b_data_len);
		CAPI_FWD(req->max_msg_data_size);
		CAPI_FWD(req->app_id);
	} else if (ioctl_cmd == CAPI_GET_MANUFACTURER_REQ ||
		    ioctl_cmd == CAPI_GET_VERSION_REQ ||
		    ioctl_cmd == CAPI_GET_SERIAL_REQ ||
	    ioctl_cmd == CAPI_GET_PROFILE_REQ) {
		uint32_t *pcontroller = buffer;

		CAPI_FWD(*pcontroller);
	}
	if ((err = cdev_ioctl(cc->file, ioctl_cmd, buffer)) != 0)
		goto error;

	if (ioctl_cmd == CAPI_REGISTER_REQ) {
		struct capi_register_req *req = buffer;

		CAPI_REV(req->max_logical_connections);
		CAPI_REV(req->max_b_data_blocks);
		CAPI_REV(req->max_b_data_len);
		CAPI_REV(req->max_msg_data_size);
		CAPI_REV(req->app_id);
	} else if (ioctl_cmd == CAPI_GET_MANUFACTURER_REQ ||
		    ioctl_cmd == CAPI_GET_VERSION_REQ ||
		    ioctl_cmd == CAPI_GET_SERIAL_REQ ||
	    ioctl_cmd == CAPI_GET_PROFILE_REQ) {
		uint32_t *pcontroller = buffer;

		CAPI_REV(*pcontroller);
	}
	header[0] = length & 0xFF;
	header[1] = length >> 8;
	header[2] = cmd;
	header[3] = 0;

	if (capiserver_write(cc, header, sizeof(header), NETCONN_COPY | NETCONN_MORE) != sizeof(header))
		return (-1);
	if (capiserver_write(cc, buffer, length, NETCONN_COPY) != length)
		return (-1);
	return (0);

error:
	header[0] = 0;
	header[1] = 0;
	header[2] = cmd;
	if (err < 256)
		header[3] = err;
	else
		header[3] = EINVAL;

	if (capiserver_write(cc, header, sizeof(header), NETCONN_COPY) != sizeof(header))
		return (-1);
	return (0);
}

static void
capiserver(struct capiconn *cc)
{
	uint8_t header[CAPISERVER_HDR_SIZE] __aligned(4);
	ssize_t length;
	uint8_t cmd;

	while (1) {
		length = cdev_read(cc->file, cc->buffer, CAPISERVER_BUF_MAX);
		if (length > 0) {
			header[0] = length & 0xFF;
			header[1] = length >> 8;
			header[2] = CAPISERVER_CMD_CAPI_MSG;
			header[3] = 0;

			if (capiserver_write(cc, header, sizeof(header), NETCONN_COPY | NETCONN_MORE) != sizeof(header))
				goto done;
			if (capiserver_write(cc, cc->buffer, length, NETCONN_COPY) != length)
				goto done;
		}
		netconn_set_recvtimeout(cc->conn, 1);
		length = capiserver_read(cc, header, CAPISERVER_HDR_SIZE);
		if (length == -2)
			continue;
		if (length != CAPISERVER_HDR_SIZE)
			goto done;

		length = header[0] | (header[1] << 8);
		cmd = header[2];

		netconn_set_recvtimeout(cc->conn, 0);
		if (length > CAPISERVER_BUF_MAX) {
			/* dump all data */
			if (capiserver_read(cc, NULL, length) != length)
				goto done;
			continue;
		}
		if (capiserver_read(cc, cc->buffer, length) != length)
			goto done;

		switch (cmd) {
		case CAPISERVER_CMD_CAPI_MSG:
			if (cdev_write(cc->file, cc->buffer, length) != length)
				goto done;
			break;
		case CAPISERVER_CMD_CAPI_REGISTER:
			if (capiserver_ioctl(cc, cmd, CAPI_REGISTER_REQ, cc->buffer, length))
				goto done;
			break;
		case CAPISERVER_CMD_CAPI_MANUFACTURER:
			if (capiserver_ioctl(cc, cmd, CAPI_GET_MANUFACTURER_REQ, cc->buffer, length))
				goto done;
			break;
		case CAPISERVER_CMD_CAPI_VERSION:
			if (capiserver_ioctl(cc, cmd, CAPI_GET_VERSION_REQ, cc->buffer, length))
				goto done;
			break;
		case CAPISERVER_CMD_CAPI_SERIAL:
			if (capiserver_ioctl(cc, cmd, CAPI_GET_SERIAL_REQ, cc->buffer, length))
				goto done;
			break;
		case CAPISERVER_CMD_CAPI_PROFILE:
			if (capiserver_ioctl(cc, cmd, CAPI_GET_PROFILE_REQ, cc->buffer, length))
				goto done;
			break;
		case CAPISERVER_CMD_CAPI_START:
			if (capiserver_ioctl(cc, cmd, CAPI_START_D_CHANNEL_REQ, cc->buffer, length))
				goto done;
			break;
		default:
			break;
		}
	}
done:	;
}

static void
capilisten(void *arg)
{
  	sx_init(&capiconn_sx, "CAPI");
	capiconn_listen = netconn_new(NETCONN_TCP);
	if (capiconn_listen == NULL)
		panic("Out of memory\n");

	if (netconn_bind(capiconn_listen, NULL, 2663))
		panic("Cannot bind\n");

	if (netconn_listen(capiconn_listen))
		panic("Cannot listen\n");
}
SYSINIT(capilisten, SI_SUB_APPLICATIONS + 1, SI_ORDER_FIRST, &capilisten, NULL);

static void
capithread(void *arg)
{
	struct capiconn *cc = arg;

	while (1) {
		/* Wait for connection */
		sx_xlock(&capiconn_sx);
		if (netconn_accept(capiconn_listen, &cc->conn) != ERR_OK)
			cc->conn = NULL;
		sx_xunlock(&capiconn_sx);
		if (cc->conn != NULL) {
			cc->file = cdev_open(capi_dev);
			if (cc->file != NULL) {
				int nb = 1;

				cdev_ioctl(cc->file, FIONBIO, (void *)&nb);
				netconn_set_sendtimeout(cc->conn, 4000 /* ms */ );
				capiserver(cc);
				cdev_close(cc->file);
			}
			capiserver_close(cc);
		}
	}
}
CREATE_THREAD_ORDERED(capithread_0, SI_SUB_APPLICATIONS + 1, SI_ORDER_SECOND, 2048, TD_PRIO_MED, &capithread, capiconn + 0);
CREATE_THREAD_ORDERED(capithread_1, SI_SUB_APPLICATIONS + 1, SI_ORDER_SECOND, 2048, TD_PRIO_MED, &capithread, capiconn + 1);
CREATE_THREAD_ORDERED(capithread_2, SI_SUB_APPLICATIONS + 1, SI_ORDER_SECOND, 2048, TD_PRIO_MED, &capithread, capiconn + 2);
CREATE_THREAD_ORDERED(capithread_3, SI_SUB_APPLICATIONS + 1, SI_ORDER_SECOND, 2048, TD_PRIO_MED, &capithread, capiconn + 3);
