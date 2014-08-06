/*-
 * Copyright (c) 2014 Hans Petter Selasky. All rights reserved.
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

/* system includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/endian.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioccom.h>
#include <sys/filio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <errno.h>
#include <sysexits.h>
#include <fcntl.h>
#include <pthread.h>

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

static int
capiserver_read(int fd, char *buf, int len)
{
	int retval = 0;

	while (len > 0) {
		int delta;

		delta = read(fd, buf, len);
		if (delta == 0)
			return (-1);	/* hangup */
		if (delta < 0)
			return (-1);
		buf += delta;
		len -= delta;
		retval += delta;
	}
	return (retval);
}

static int
capiserver_listen(const char *host, const char *port, int buffer)
{
	struct addrinfo hints;
	struct addrinfo *res;
	struct addrinfo *res0;
	int error;
	int flag;
	int s;

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags |= AI_NUMERICHOST;

	if ((error = getaddrinfo(host, port, &hints, &res)))
		return (-1);

	res0 = res;

	do {
		if ((s = socket(res0->ai_family, res0->ai_socktype,
		    res0->ai_protocol)) < 0)
			continue;

		flag = 1;
		setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &flag, (int)sizeof(flag));
		setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &flag, (int)sizeof(flag));

		setsockopt(s, SOL_SOCKET, SO_SNDBUF, &buffer, (int)sizeof(buffer));
		setsockopt(s, SOL_SOCKET, SO_RCVBUF, &buffer, (int)sizeof(buffer));

		if (bind(s, res0->ai_addr, res0->ai_addrlen) == 0) {
			if (listen(s, 1) == 0)
				break;
		}
		close(s);
		s = -1;
	} while ((res0 = res0->ai_next) != NULL);

	freeaddrinfo(res);

	return (s);
}

#define	CAPI_FWD(x) (x) = le32toh(x)
#define	CAPI_REV(x) (x) = htole32(x)

static int
capiserver_ioctl(int tcp_fd, uint32_t cmd, int capi_fd,
    uint32_t ioctl_cmd, void *buffer, ssize_t length)
{
	uint8_t header[CAPISERVER_HDR_SIZE] __aligned(4);

	if (length != IOCPARM_LEN(ioctl_cmd)) {
		errno = EINVAL;
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
	if (ioctl(capi_fd, ioctl_cmd, buffer) != 0)
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

	if (write(tcp_fd, header, sizeof(header)) != sizeof(header))
		return (-1);

	if (length != 0 && write(tcp_fd, buffer, length) != length)
		return (-1);

	return (0);

error:
	header[0] = 0;
	header[1] = 0;
	header[2] = cmd;
	if (errno < 256)
		header[3] = errno;
	else
		header[3] = EINVAL;

	if (write(tcp_fd, header, sizeof(header)) != sizeof(header))
		return (-1);
	return (0);
}

struct capiserver_arg {
	int	capi_fd;
	int	tcp_fd;
};

#define	CAPISERVER_BUF_MAX 65536

static void *
capiserver(void *_parg)
{
	uint8_t header[CAPISERVER_HDR_SIZE] __aligned(4);
	uint8_t *buffer;
	struct capiserver_arg *parg = _parg;
	struct pollfd fds[2];
	ssize_t length;
	uint8_t cmd;

	buffer = malloc(CAPISERVER_BUF_MAX);
	if (buffer == NULL)
		goto done;

	while (1) {
		fds[0].fd = parg->capi_fd;
		fds[0].events = (POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI |
		    POLLERR | POLLHUP | POLLNVAL);
		fds[0].revents = 0;

		fds[1].fd = parg->tcp_fd;
		fds[1].events = (POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI |
		    POLLERR | POLLHUP | POLLNVAL);
		fds[1].revents = 0;

		if (poll(fds, 2, -1) < 0)
			goto done;

		if (fds[0].revents != 0) {
			length = read(parg->capi_fd, buffer, CAPISERVER_BUF_MAX);
			if (length < 0)
				goto done;

			header[0] = length & 0xFF;
			header[1] = length >> 8;
			header[2] = CAPISERVER_CMD_CAPI_MSG;
			header[3] = 0;

			if (write(parg->tcp_fd, header, sizeof(header)) != sizeof(header))
				goto done;

			if (write(parg->tcp_fd, buffer, length) != length)
				goto done;
		}
		if (fds[1].revents != 0) {
			if (capiserver_read(parg->tcp_fd, buffer,
			    CAPISERVER_HDR_SIZE) != CAPISERVER_HDR_SIZE)
				goto done;

			length = buffer[0] | (buffer[1] << 8);
			cmd = buffer[2];

			if (capiserver_read(parg->tcp_fd, buffer, length) != length)
				goto done;

			switch (cmd) {
			case CAPISERVER_CMD_CAPI_MSG:
				if (write(parg->capi_fd, buffer, length) != length)
					goto done;
				break;
			case CAPISERVER_CMD_CAPI_REGISTER:
				if (capiserver_ioctl(parg->tcp_fd, cmd, parg->capi_fd,
				    CAPI_REGISTER_REQ, buffer, length))
					goto done;
				break;
			case CAPISERVER_CMD_CAPI_MANUFACTURER:
				if (capiserver_ioctl(parg->tcp_fd, cmd, parg->capi_fd,
				    CAPI_GET_MANUFACTURER_REQ, buffer, length))
					goto done;
				break;
			case CAPISERVER_CMD_CAPI_VERSION:
				if (capiserver_ioctl(parg->tcp_fd, cmd, parg->capi_fd,
				    CAPI_GET_VERSION_REQ, buffer, length))
					goto done;
				break;
			case CAPISERVER_CMD_CAPI_SERIAL:
				if (capiserver_ioctl(parg->tcp_fd, cmd, parg->capi_fd,
				    CAPI_GET_SERIAL_REQ, buffer, length))
					goto done;
				break;
			case CAPISERVER_CMD_CAPI_PROFILE:
				if (capiserver_ioctl(parg->tcp_fd, cmd, parg->capi_fd,
				    CAPI_GET_PROFILE_REQ, buffer, length))
					goto done;
				break;
			case CAPISERVER_CMD_CAPI_START:
				if (capiserver_ioctl(parg->tcp_fd, cmd, parg->capi_fd,
				    CAPI_START_D_CHANNEL_REQ, buffer, length))
					goto done;
				break;
			default:
				break;
			}
		}
	}
done:
	close(parg->capi_fd);
	close(parg->tcp_fd);
	free(parg);
	free(buffer);
	return (NULL);
}

static void
capiserver_usage(void)
{
	fprintf(stderr,
	    "\n"
	    "\n" "capiserver - CAPI server, version %d.%02d, compiled %s %s"
	    "\n" "usage: capiserver [-B] [-b 127.0.0.1] [-p 2663] [-h]"
	    "\n" "       -B            run in background"
	    "\n" "       -b <addr>     bind address"
	    "\n" "       -p <port>     bind port"
	    "\n" "       -h            show usage"
	    "\n"
	    ,CAPI_STACK_VERSION / 100, CAPI_STACK_VERSION % 100, __DATE__, __TIME__);
}

int
main(int argc, char **argv)
{
	const char *params = "Bb:p:h";
	const char *host = "127.0.0.1";
	const char *port = "2663";
	int do_fork = 0;
	int opt;
	int s;
	int f;
	int c;
	int d;

	while ((opt = getopt(argc, argv, params)) != -1) {
		switch (opt) {
		case 'b':
			host = optarg;
			break;
		case 'p':
			port = optarg;
			break;
		case 'B':
			do_fork = 1;
			break;
		default:
			capiserver_usage();
			return (EX_USAGE);
		}
	}

	if (do_fork) {
		if (fork() != 0)
			return (0);
	}
	s = capiserver_listen(host, port, CAPISERVER_BUF_MAX);
	if (s < 0) {
		printf("Could not bind to '%s' and '%s'\n", host, port);
		return (0);
	}
	while (1) {
		struct capiserver_arg *parg;
		pthread_t dummy;

		f = accept(s, NULL, NULL);
		if (f < 0)
			break;

		parg = malloc(sizeof(*parg));
		if (parg == NULL) {
			close(f);
			continue;
		}
		c = open(CAPI_DEVICE_NAME, O_RDWR);
		if (c < 0) {
			close(f);
			continue;
		}
		d = 0;
		if (ioctl(c, FIONBIO, &d) != 0) {
			close(c);
			close(f);
			continue;
		}
		d = 0;
		if (ioctl(f, FIONBIO, &d) != 0) {
			close(c);
			close(f);
			continue;
		}
		parg->capi_fd = c;
		parg->tcp_fd = f;

		if (pthread_create(&dummy, NULL, &capiserver, parg) != 0) {
			free(parg);
			close(c);
			close(f);
		}
	}
	return (0);
}
