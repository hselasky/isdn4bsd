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
#include <i4b/include/capi20.h>
#include <errno.h>

/*---------------------------------------------------------------------------*
 *	usage display and exit
 *---------------------------------------------------------------------------*/
static void
usage(void)
{
	fprintf(stderr,
	    "\n"
	    "\n" "capimonitor - print information about incoming calls"
	    "\n" "usage: capimonitor [-u controller]"
	    "\n" "       -u <unit>     specify controller unit to listen to"
	    "\n"
	    ,CAPI_STACK_VERSION / 100, CAPI_STACK_VERSION % 100);

	exit(1);
}

#define	TELNO_MAX 128			/* including terminating zero */

static struct capi20_backend *cbe_p;
static uint32_t app_id;
static uint8_t src_telno[TELNO_MAX];
static uint8_t dst_telno[TELNO_MAX];
static uint8_t controller = 0;

static uint16_t
capi_send_listen_request(unsigned controller, uint8_t incoming_calls)
{
	struct capi_message_decoded msg;

	bzero(&msg, sizeof(msg));

	msg.head.wApp = app_id;
	msg.head.wCmd = CAPI_P_REQ(LISTEN);
	msg.head.dwCid = controller;

	msg.data.LISTEN_REQ.dwInfoMask =
	    CAPI_INFO_MASK_CAUSE |
	    CAPI_INFO_MASK_DATE_TIME |
	    CAPI_INFO_MASK_DISPLAY |
	    CAPI_INFO_MASK_USER_USER |
	    CAPI_INFO_MASK_CALL_PROGRESSION |
	    CAPI_INFO_MASK_FACILITY |
	    CAPI_INFO_MASK_CHARGING |
	    CAPI_INFO_MASK_CALLED_PARTY_NUMBER |
	    CAPI_INFO_MASK_CHANNEL_ID |
	    CAPI_INFO_MASK_REDIRECTION_INFO |
	    CAPI_INFO_MASK_SENDING_COMPLETE;

	msg.data.LISTEN_REQ.dwCipMask1 =
	    1 |
	    CAPI_CIP_MASK1(SPEECH) |
	    CAPI_CIP_MASK1(UNRESTRICTED_DATA) |
	    CAPI_CIP_MASK1(3100Hz_AUDIO) |
	    CAPI_CIP_MASK1(7kHz_AUDIO) |
	    CAPI_CIP_MASK1(UNRESTRICTED_DATA_TONES) |
	    CAPI_CIP_MASK1(TELEPHONY) |
	    CAPI_CIP_MASK1(FAX_G2_G3) |
	    CAPI_CIP_MASK1(7kHz_TELEPHONY);

	return capi_put_message_decoded(&msg);
}

static uint16_t
capi_send_connect_resp(uint32_t cid, uint16_t wReject)
{
	struct capi_message_decoded msg;

	bzero(&msg, sizeof(msg));

	msg.head.wApp = app_id;
	msg.head.wCmd = CAPI_P_RESP(CONNECT);
	msg.head.dwCid = cid;

	msg.data.CONNECT_RESP.wReject = wReject;

	CONNECT_RESP_BPROTOCOL(&msg) = CAPI_DEFAULT;
	CONNECT_RESP_ADDITIONALINFO(&msg) = CAPI_DEFAULT;

	return capi_put_message_decoded(&msg);
}

static void
capi_decode_struct(void *ptr, struct capi_struct *mp)
{
	uint16_t len;

	if (((uint8_t *)(ptr))[0] == 0xFF) {
		len =
		    ((uint8_t *)(ptr))[1] |
		    (((uint8_t *)(ptr))[2] << 8);
		ptr = ADD_BYTES(ptr, 3);
	} else {
		len =
		    ((uint8_t *)(ptr))[0];
		ptr = ADD_BYTES(ptr, 1);
	}
	mp->ptr = ptr;
	mp->len = len;
	return;
}

/*---------------------------------------------------------------------------*
 *	handle an incoming CAPI message
 *---------------------------------------------------------------------------*/
static void
capi_message_handler(const struct capi_message_decoded *mp)
{
	struct capi_message_decoded msg;
	struct call_desc *cd;
	struct capi_struct str;

	uint8_t buffer[2048];

	msg.head.wCmd = 0;

	switch (mp->head.wCmd) {

	case CAPI_P_CONF(LISTEN):
		if (mp->data.LISTEN_CONF.wInfo) {
			fprintf(stderr, "cannot listen, wInfo=0x%04x\n",
			    mp->data.LISTEN_CONF.wInfo);
		}
		break;

		/*
		 * CAPI indications
		 */

	case CAPI_P_IND(CONNECT):

		capi_decode_struct(mp->data.CONNECT_IND.dst_telno.ptr, &str);

		if (str.len >= TELNO_MAX)
			str.len = TELNO_MAX - 1;

		if (str.len >= 1)
			str.len -= 1;

		str.ptr = ADD_BYTES(str.ptr, 1);

		bcopy(str.ptr, dst_telno, str.len);
		dst_telno[str.len] = 0;

		capi_decode_struct(mp->data.CONNECT_IND.src_telno.ptr, &str);

		if (str.len >= TELNO_MAX)
			str.len = TELNO_MAX - 1;

		if (str.len >= 1)
			str.len -= 1;

		if (str.len >= 1)
			str.len -= 1;

		str.ptr = ADD_BYTES(str.ptr, 2);

		bcopy(str.ptr, src_telno, str.len);
		src_telno[str.len] = 0;

		/*
		 * NOTE: The FreeBSD CAPI kernel will filter the incoming
		 * number for invalid characters!
		 */
		printf("Incoming call from '%s' to '%s'\n", src_telno, dst_telno);

		/* ignore call */
		capi_send_connect_resp(mp->head.dwCid, 1);
		break;

	default:
		/* nothing to do */
		return;
	}
	return;
}

static void
loop()
{
	int error;

	while (1) {
		struct pollfd pfd[1];

		bzero(&pfd, sizeof(pfd));

		pfd[0].fd = capi20_fileno(app_id);
		pfd[0].events = POLLIN | POLLRDNORM;

		error = poll(pfd, 1, -1);

		if (error == -1) {
			fprintf(stderr, "%s: %s: poll error: %s\n",
			    __FILE__, __FUNCTION__, strerror(errno));
			break;
		}
		if (error > 0) {
			if (pfd[0].revents & (POLLIN | POLLRDNORM)) {
				struct capi_message_decoded msg;

				while (1) {
					if (capi_get_message_decoded(&msg, app_id))
						break;

					capi_message_handler(&msg);
				}
			}
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	list all controllers installed
 *---------------------------------------------------------------------------*/
static uint16_t
capi_init_all_controllers(uint8_t listen)
{
	struct CAPI_PROFILE_DATA_ENCODED profile;
	uint16_t error;
	uint16_t unit;

	/* get number of installed controllers */

	error = capi20_get_profile(cbe_p, 0, &profile, sizeof(profile));
	if (error) {
		fprintf(stderr, "%s: %s: get profile returned error: %s\n",
		    __FILE__, __FUNCTION__,
		    capi20_get_errstr(error));

		return error;
	}
	unit = le16toh(profile.wNumCtlr);
	if (unit == 0) {
		fprintf(stderr, "%s: %s: no controllers installed\n",
		    __FILE__, __FUNCTION__);
		return 0;
	}
	while (unit--) {
		error = capi_send_listen_request(unit + 1, listen);
		if (error) {
			fprintf(stderr,
			    "%s: %s: capi send listen request failed, controller=0x%02x, error=%s\n",
			    __FILE__, __FUNCTION__, unit + 1, capi20_get_errstr(error));
		}
	}
	return 0;
}

int
main(int argc, char **argv)
{
	uint32_t temp;
	uint16_t error;
	int c;

	error = capi20_be_alloc_i4b(&cbe_p);
	if (error) {
		fprintf(stderr, "%s: %s: could not allocate I4B CAPI backend, error=%s\n",
		    __FILE__, __FUNCTION__, capi20_get_errstr(error));
		return -1;
	}
	while ((c = getopt(argc, argv, "u:")) != -1) {
		switch (c) {
		case 'u':
			controller = atoi(optarg);
			break;
		case '?':
		default:
			usage();
			break;
		}
	}

	error = capi20_is_installed(cbe_p);
	if (error) {
		fprintf(stderr, "CAPI 2.0 not installed! "
		    "Or insufficient access rights.\n");
		return (-1);
	}
	/* register at CAPI, only two connections will be established */
	error = capi20_register(cbe_p, 2, 7, 400, CAPI_STACK_VERSION, &temp);
	if (error) {
		fprintf(stderr, "%s: %s: could not register by CAPI, error=%s\n",
		    __FILE__, __FUNCTION__, capi20_get_errstr(error));
		return (-1);
	}
	app_id = temp;

	(void)capi_init_all_controllers(1);

	loop();

	/* release CAPI application */
	error = capi20_release(app_id);
	if (error) {
		fprintf(stderr, "%s: %s: could not release CAPI application! error=%s\n",
		    __FILE__, __FUNCTION__, capi20_get_errstr(error));
		return (-1);
	}
	return (0);
}
