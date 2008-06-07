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
 *	bintec.c - Implementation of Remote CAPI over TCP
 *	-------------------------------------------------
 *
 * $FreeBSD: $
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
#include <sys/socket.h>
#include <poll.h>
#include <errno.h>
#include <err.h>
#include <netdb.h>
#include <md5.h>

#define CAPI_MAKE_IOCTL

/* CAPI 2.0 include */
#include <i4b/include/capi20.h>

#include "capilib.h"

/* BinTec specific CAPI 2.0 commands */
enum {
  CAPI_BINTEC_REGISTER_REQ = 0xFFF2,
  CAPI_BINTEC_REGISTER_CONF = 0xFFF3,
  CAPI_BINTEC_MANUFACT_REQ = 0xFFFA,
  CAPI_BINTEC_MANUFACT_CONF = 0xFFFB,
  CAPI_BINTEC_SERIAL_REQ = 0xFFFE,
  CAPI_BINTEC_SERIAL_CONF = 0xFFFF,
  CAPI_BINTEC_VERSION_REQ = 0xFFFC,
  CAPI_BINTEC_VERSION_CONF = 0xFFFD,
  CAPI_BINTEC_PROFILE_REQ = 0xFFE0,
  CAPI_BINTEC_PROFILE_CONF = 0xFFE1,
  CAPI_BINTEC_ALIVE_IND = 0xFFF0,
  CAPI_BINTEC_ALIVE_RESP = 0xFFF1,
  CAPI_BINTEC_CONTROL_REQ = 0x00FF,
  CAPI_BINTEC_CONTROL_CONF = 0x01FF,
  CAPI_BINTEC_CONTROL_IND = 0x02FF,
  CAPI_BINTEC_CONTROL_RESP = 0x03FF,
};

/* BinTec specific control type values */
enum {
  CTRL_CAPIREC_ON = 0x01,
  CTRL_CAPIREC_OFF = 0x02,
  CTRL_CAPIREC_PLAY = 0x03,
  CTRL_TRACELEVEL = 0x04,
  CTRL_BOARD_CONFIG = 0x05,
  CTRL_BOARD_LOAD = 0x06,
  CTRL_BOARD_UNLOAD = 0x07,
  CTRL_LOOPBACK = 0x08,
  CTRL_CAPISTATE = 0x09,
  CTRL_CMDEXEC = 0x0a,
  CTRL_ISDNREC_ON = 0x0c,
  CTRL_ISDNREC_OFF = 0x0d,
  CTRL_ISDNREC_PLAY = 0x0e,
  CTRL_ISDNREC_CLEAR = 0x0f,
  CTRL_STATIST = 0x10,
  CTRL_EAZMAPPING = 0x11,
  CTRL_ISDNREC_TRACE = 0x12,
  CTRL_CAPIREC_TRACE = 0x13,
  CTRL_CAPI_NEWCALL = 0x14,
  CTRL_CAPI_RMVCALL = 0x15,
  CTRL_CAPI_STATCALL = 0x16,
  CTRL_CAPI_CHGCALL = 0x17,
  CTRL_CAPI_L1STATE = 0x18,
  CTRL_GETCHALLENGE = 0x19,
  CTRL_SETUSER = 0x1a,
};

/* -------- */

#define CAPI_BINTEC_MAKE_STRUCT(ENUM)	\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_REQ)	\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_CONF)	\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_IND)	\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_RESP)

/* -------- */

#define CAPI_BINTEC_PROFILE_REQ(m,n) \
  m(n, WORD  , wController,)\
  END

#define CAPI_BINTEC_PROFILE_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  m(n, WORD  , wNumCtlr,)\
  m(n, WORD  , wNumBChannels,)\
  m(n, DWORD , dwGlobalOptions,)\
  m(n, DWORD , dwB1ProtocolSupport,)\
  m(n, DWORD , dwB2ProtocolSupport,)\
  m(n, DWORD , dwB3ProtocolSupport,)\
  m(n, BYTE_ARRAY, bReserved, 24)\
  m(n, BYTE_ARRAY, bManufacturerSpecific, 20)\
  END

#define CAPI_BINTEC_PROFILE_IND(m,n) \
  END

#define CAPI_BINTEC_PROFILE_RESP(m,n) \
  END

/* -------- */

#define CAPI_BINTEC_REGISTER_REQ(m,n) \
  m(n, DWORD , dwBuffer,)\
  m(n, WORD  , wNMess,)\
  m(n, WORD  , wNConn,)\
  m(n, WORD  , wNDBlock,)\
  m(n, WORD  , wDBlockSize,)\
  m(n, BYTE  , bVersion,)\
  END

#define CAPI_BINTEC_REGISTER_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_BINTEC_REGISTER_IND(m,n) \
  END

#define CAPI_BINTEC_REGISTER_RESP(m,n) \
  END

/* -------- */

#define CAPI_BINTEC_RELEASE_REQ(m,n) \
  m(n, WORD  , wAppId,)\
  END

#define CAPI_BINTEC_RELEASE_CONF(m,n) \
  m(n, WORD  , wInfo,) \
  END

#define CAPI_BINTEC_RELEASE_IND(m,n) \
  END

#define CAPI_BINTEC_RELEASE_RESP(m,n) \
  END

/* -------- */

#define CAPI_BINTEC_MANUFACT_REQ(m,n) \
  END

#define CAPI_BINTEC_MANUFACT_CONF(m,n) \
  m(n, STRUCT, sManufact,)\
  END

#define CAPI_BINTEC_MANUFACT_IND(m,n) \
  END

#define CAPI_BINTEC_MANUFACT_RESP(m,n) \
  END

/* -------- */

#define CAPI_BINTEC_VERSION_REQ(m,n) \
  END

#define CAPI_BINTEC_VERSION_CONF(m,n) \
  m(n, WORD , wCapiVersion,)\
  m(n, WORD , wManufactVersion,)\
  m(n, STRUCT, sVersion,)\
  END

#define CAPI_BINTEC_VERSION_IND(m,n) \
  END

#define CAPI_BINTEC_VERSION_RESP(m,n) \
  END

/* -------- */

#define CAPI_BINTEC_SERIAL_REQ(m,n) \
  END

#define CAPI_BINTEC_SERIAL_CONF(m,n) \
  m(n, STRUCT, sSerial,)\
  END

#define CAPI_BINTEC_SERIAL_IND(m,n) \
  END

#define CAPI_BINTEC_SERIAL_RESP(m,n) \
  END

/* -------- */

#define CAPI_BINTEC_CONTROL_REQ(m,n) \
  m(n, WORD  , wContrl, )\
  m(n, WORD  , wType, )\
  m(n, STRUCT, sData, )\
  END

#define CAPI_BINTEC_CONTROL_CONF(m,n) \
  m(n, WORD  , wContrl, )\
  m(n, WORD  , wType, )\
  m(n, WORD  , wInfo, )\
  m(n, STRUCT, sData, )\
  END

#define CAPI_BINTEC_CONTROL_IND(m,n) \
  m(n, WORD  , wContrl, )\
  m(n, WORD  , wType, )\
  m(n, STRUCT, sData, )\
  END

#define CAPI_BINTEC_CONTROL_RESP(m,n) \
  m(n, WORD  , wContrl, )\
  m(n, WORD  , wType, )\
  m(n, STRUCT, sData, )\
  END

CAPI_BINTEC_MAKE_STRUCT(BINTEC_REGISTER)
CAPI_BINTEC_MAKE_STRUCT(BINTEC_MANUFACT)
CAPI_BINTEC_MAKE_STRUCT(BINTEC_SERIAL)
CAPI_BINTEC_MAKE_STRUCT(BINTEC_VERSION)
CAPI_BINTEC_MAKE_STRUCT(BINTEC_PROFILE)
CAPI_BINTEC_MAKE_STRUCT(BINTEC_RELEASE)
CAPI_BINTEC_MAKE_STRUCT(BINTEC_CONTROL)

/*---------------------------------------------------------------------------*
 *	capi20_be_alloc_bintec - Allocate a BinTec CAPI backend
 *
 * @param hostname       Pointer to zero terminated host name string.
 *
 * @param servname       Pointer to zero terminated service name string.
 *
 * @param username       Pointer to zero terminated username string.
 *
 * @param password       Pointer to zero terminated password string.
 *
 * @param cbe_pp	 Pointer to pointer that should be pointed to backend.
 *
 * @retval 0             Backend allocation was successful.
 *
 * @retval Else          An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capi20_be_alloc_bintec(const char *hostname, const char *servname, 
    const char *username, const char *password, struct capi20_backend **cbe_pp)
{
	struct capi20_backend *cbe;

	if ((cbe_pp == NULL) ||
	    (hostname == NULL)) {
		return (CAPI_ERROR_INVALID_PARAM);
	}

	/* set default TCP port, if any */
	if (servname == NULL) {
	    servname = "2662";
	}

	/* set default username, if any */
	if (username == NULL) {
	    username = "";
	}

	/* set default password, if any */
	if (password == NULL) {
	    password = "";
	}

	cbe = malloc(sizeof(*cbe));
	if(cbe == NULL) {
		return (CAPI_ERROR_OS_RESOURCE_ERROR);
	}

	*cbe_pp = cbe;

	bzero(cbe, sizeof(*cbe));

	cbe->bBackendType = CAPI_BACKEND_TYPE_BINTEC;
	strlcpy(cbe->sHostName, hostname, sizeof(cbe->sHostName));
	strlcpy(cbe->sServName, servname, sizeof(cbe->sServName));
	strlcpy(cbe->sUserName, username, sizeof(cbe->sUserName));
	strlcpy(cbe->sPassWord, password, sizeof(cbe->sPassWord));

	return (0);
}

/*---------------------------------------------------------------------------*
 *	capilib_alloc_app_bintec - Allocate a BinTec CAPI application
 *
 * @param cbe                      Pointer to CAPI backend.
 *
 * @retval 0                       Application allocation failed.
 *
 * @retval Else                    Pointer to CAPI Application.
 *---------------------------------------------------------------------------*/
struct app_softc *
capilib_alloc_app_bintec(struct capi20_backend *cbe)
{
	struct addrinfo hints;
	struct addrinfo *res;
	struct addrinfo *res0;
	struct app_softc *sc;
	int s;
	int error;

	sc = capilib_alloc_app_sub(cbe);
	if (sc == NULL) {
	    return NULL;
	}

	bzero(&hints, sizeof(hints));

	hints.ai_family = PF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	error = getaddrinfo(cbe->sHostName, cbe->sServName, &hints, &res0);
	if (error) {
	    capilib_free_app(sc);
	    return NULL;
	}

	s = -1;
	for (res = res0; res; res = res->ai_next) {
	    s = socket(res->ai_family, res->ai_socktype,
		       res->ai_protocol);
	    if (s < 0) {
		continue;
	    }
	    if (connect(s, res->ai_addr, res->ai_addrlen) < 0) {
		close(s);
		s = -1;
		continue;
	    }
	    break;
	}

	freeaddrinfo(res0);

	if (s < 0) {
	    capilib_free_app(sc);
	    return NULL;
	}

	sc->sc_fd = s;

	return (sc);
}

/*---------------------------------------------------------------------------*
 *	capilib_bintec_do_cmd - BinTec common command wrapper
 *
 * @param sc             Pointer to Application Softc.
 *
 * @param wReqCmd        Command value to send.
 *
 * @param wConfCmd       Command value to expect back.
 *
 * @param pReq           Pointer to *DECODED request.
 *
 * @param pConf          Pointer to *DECODED confirmation.
 *
 * @retval 0             Success.
 *
 * @retval Else          An error happened.
 *---------------------------------------------------------------------------*/
uint16_t
capilib_bintec_do_cmd(struct app_softc *sc, uint16_t wReqCmd, 
    uint16_t wConfCmd, void *pReq, void *pConf)
{
	struct timeval tv;
	struct CAPI_HEADER_DECODED head;
	uint8_t buffer[256];
	int len;
	uint16_t error;
	uint16_t off;

	/* setup decode tags */
	CAPI_INIT(CAPI_HEADER, &head);

	/* encode all data */
	len = capi20_encode(buffer, sizeof(buffer), &head);
	len += capi20_encode(buffer + len, sizeof(buffer) - len, pReq);

	head.wLen = len;
	head.wApp = 0;
	head.wCmd = wReqCmd;
	head.wNum = 0;
	head.dwCid = 0;

	/* re-encode header */
	len = capi20_encode(buffer, sizeof(buffer), &head);

	/* send the message accross */
	error = capi20_put_message(sc->sc_app_id, buffer);
	if (error) {
	    return (error);
	}

	tv.tv_sec = 6;
	tv.tv_usec = 0;

	/* wait for a reply */
	error = capi20_wait_for_message(sc->sc_app_id, &tv);
	if (error) {
	    return (error);
	}

	/* try to read out message - XXX this can hang */
	len = capilib_get_message_sub(sc, buffer, sizeof(buffer));
	if (len < 0) {
	    return (CAPI_ERROR_INVALID_BUFFER_SIZE);
	}

	off = capi20_decode(buffer, len, &head);
	off += capi20_decode(buffer + off, len - off, pConf);

	if (head.wCmd != wConfCmd) {
	    return (CAPI_ERROR_ILLEGAL_COMMAND);
	}
	return (0);
}

/*---------------------------------------------------------------------------*
 *	capilib_bintec_do_ioctl - BinTec IOCTL wrapper
 *
 * @param sc             Pointer to Application Softc.
 *
 * @param cmd            Command to perform.
 *
 * @param data           Associated command data.
 *
 * @retval 0             Success.
 *
 * @retval Else          An error happened.
 *---------------------------------------------------------------------------*/
int
capilib_bintec_do_ioctl(struct app_softc *sc, uint32_t cmd, void *data)
{
	union {
	  struct CAPI_BINTEC_PROFILE_REQ_DECODED prof;
	  struct CAPI_BINTEC_REGISTER_REQ_DECODED reg;
	  struct CAPI_BINTEC_RELEASE_REQ_DECODED rel;
	  struct CAPI_BINTEC_MANUFACT_REQ_DECODED man;
	  struct CAPI_BINTEC_VERSION_REQ_DECODED ver;
	  struct CAPI_BINTEC_SERIAL_REQ_DECODED ser;
	  struct CAPI_BINTEC_CONTROL_REQ_DECODED ctrl;
	} req;

	union {
	  struct CAPI_BINTEC_SERIAL_CONF_DECODED ser;
	  struct CAPI_BINTEC_VERSION_CONF_DECODED ver;
	  struct CAPI_BINTEC_MANUFACT_CONF_DECODED man;
	  struct CAPI_BINTEC_RELEASE_CONF_DECODED rel;
	  struct CAPI_BINTEC_REGISTER_CONF_DECODED reg;
	  struct CAPI_BINTEC_PROFILE_CONF_DECODED prof;
	  struct CAPI_BINTEC_CONTROL_CONF_DECODED ctrl;
	} conf;

	MD5_CTX md5Context[1];

	uint16_t error;
	uint16_t len;

	bzero(&req, sizeof(req));
	bzero(&conf, sizeof(conf));

	switch (cmd) {
	case FIONBIO:
	    return (ioctl(sc->sc_fd, FIONBIO, data));

	case CAPI_REGISTER_REQ: {

	    struct capi_register_req *ptr = data;

	    CAPI_INIT(CAPI_BINTEC_CONTROL_REQ, &req.ctrl);
	    CAPI_INIT(CAPI_BINTEC_CONTROL_CONF, &conf.ctrl);

	    req.ctrl.wType = CTRL_GETCHALLENGE;

	    error = capilib_bintec_do_cmd(sc,
	      CAPI_BINTEC_CONTROL_REQ, 
	      CAPI_BINTEC_CONTROL_CONF,
	      &req, &conf);
	    if (error) {
	        break;
	    }

	    if ((conf.ctrl.sData.len == 0) ||
		(conf.ctrl.wType != CTRL_GETCHALLENGE)) {
	        error = CAPI_ERROR_ILLEGAL_MSG_PARAMETER;
		break;
	    }

	    if (ptr->pUserName[0]) {
	        
	        /* we need to authenticate */

	        len = strlen(ptr->pUserName);

		req.ctrl.wType = CTRL_SETUSER;
		req.ctrl.sData.ptr = sc->sc_temp;
		req.ctrl.sData.len = len + MD5_DIGEST_LENGTH;

		if (req.ctrl.sData.len > sizeof(sc->sc_temp)) {
		    error = CAPI_ERROR_INVALID_BUFFER_SIZE;
		    break;
		}

		bcopy(ptr->pUserName, sc->sc_temp, len);

		/* compute digest */

	        MD5Init(md5Context);
		MD5Update(md5Context, (void *)(ptr->pUserName), len);
		MD5Update(md5Context, conf.ctrl.sData.ptr,
		 conf.ctrl.sData.len);
		MD5Update(md5Context, (void *)(ptr->pPassWord), 
		 strlen(ptr->pPassWord));
		MD5Final(sc->sc_temp + len, md5Context);

		/* return digest to BinTec router */

		error = capilib_bintec_do_cmd(sc,
		  CAPI_BINTEC_CONTROL_REQ, 
		  CAPI_BINTEC_CONTROL_CONF,
		  &req, &conf);

		if (error) {
		    break;
		}

		if ((conf.ctrl.wInfo != 1) ||
		    (conf.ctrl.wType != CTRL_SETUSER)) {
		    error = CAPI_ERROR_ILLEGAL_IDENTIFIER;
		    break;
		}
	    }

	    bzero(&req, sizeof(req));
	    bzero(&conf, sizeof(conf));

	    CAPI_INIT(CAPI_BINTEC_REGISTER_REQ, &req.reg);
	    CAPI_INIT(CAPI_BINTEC_REGISTER_CONF, &conf.reg);

	    req.reg.wNMess = ptr->max_msg_data_size;
	    req.reg.wNConn = ptr->max_logical_connections;
	    req.reg.wNDBlock = ptr->max_b_data_blocks;
	    req.reg.wDBlockSize = ptr->max_b_data_len;
	    req.reg.bVersion = 2; /* CAPI 2.0 */

	    error = capilib_bintec_do_cmd(sc,
	      CAPI_BINTEC_REGISTER_REQ, 
	      CAPI_BINTEC_REGISTER_CONF,
	      &req, &conf);
	    if (error) {
	      break;
	    }
	    error = conf.reg.wInfo;
	    if (error) {
	      break;
	    }
	    ptr->app_id = 1; /* dummy */
	    break;
	}

	case CAPI_GET_MANUFACTURER_REQ: {
	    struct capi_get_manufacturer_req *ptr = data;

	    CAPI_INIT(CAPI_BINTEC_MANUFACT_REQ, &req.man);
	    CAPI_INIT(CAPI_BINTEC_MANUFACT_CONF, &conf.man);

	    error = capilib_bintec_do_cmd(sc,
	      CAPI_BINTEC_MANUFACT_REQ, 
	      CAPI_BINTEC_MANUFACT_CONF,
	      &req, &conf);

	    if (error) {
	      break;
	    }
	    if (conf.man.sManufact.len > sizeof(ptr->name)) {
	        conf.man.sManufact.len = sizeof(ptr->name);
	    }
	    strlcpy((void *)(ptr->name), conf.man.sManufact.ptr, 
		    conf.man.sManufact.len);
	    break;
	}

	case CAPI_GET_VERSION_REQ: {
	    struct capi_get_version_req *ptr = data;

	    CAPI_INIT(CAPI_BINTEC_VERSION_REQ, &req.ver);
	    CAPI_INIT(CAPI_BINTEC_VERSION_CONF, &conf.ver);

	    error = capilib_bintec_do_cmd(sc,
	      CAPI_BINTEC_VERSION_REQ, 
	      CAPI_BINTEC_VERSION_CONF,
	      &req, &conf);

	    if (error) {
	      break;
	    }
	    ptr->version.CAPI_major = 
	      conf.ver.wCapiVersion >> 8;
	    ptr->version.CAPI_minor = 
	      conf.ver.wCapiVersion & 0xFF;
	    ptr->version.manufacturer_major = 
	      conf.ver.wManufactVersion >> 8;
	    ptr->version.manufacturer_minor = 
	      conf.ver.wManufactVersion & 0xFF;
	    break;
	}

	case CAPI_GET_SERIAL_REQ: {
	    struct capi_get_serial_req *ptr = data;

	    CAPI_INIT(CAPI_BINTEC_SERIAL_REQ, &req.ser);
	    CAPI_INIT(CAPI_BINTEC_SERIAL_CONF, &conf.ser);

	    error = capilib_bintec_do_cmd(sc,
	      CAPI_BINTEC_SERIAL_REQ, 
	      CAPI_BINTEC_SERIAL_CONF,
	      &req, &conf);

	    if (error) {
	      break;
	    }

	    if (conf.ser.sSerial.len > sizeof(ptr->serial_number)) {
	        conf.ser.sSerial.len = sizeof(ptr->serial_number);
	    }
	    strlcpy((void *)(ptr->serial_number), conf.ser.sSerial.ptr, 
		    conf.ser.sSerial.len);
	    break;
	}

	case CAPI_GET_PROFILE_REQ: {

	    struct capi_get_profile_req *ptr = data;

	    CAPI_INIT(CAPI_BINTEC_PROFILE_REQ, &req.prof);
	    CAPI_INIT(CAPI_BINTEC_PROFILE_CONF, &conf.prof);

	    req.prof.wController = ptr->controller;

	    error = capilib_bintec_do_cmd(sc,
	      CAPI_BINTEC_PROFILE_REQ, 
	      CAPI_BINTEC_PROFILE_CONF,
	      &req, &conf);

	    if (error) {
	        break;
	    }
	    error = conf.prof.wInfo;
	    if (error) {
	        break;
	    }
	    ptr->profile.wNumCtlr = 
	      htole16(conf.prof.wNumCtlr);
	    ptr->profile.wNumBChannels = 
	      htole16(conf.prof.wNumBChannels);
	    ptr->profile.dwGlobalOptions = 
	      htole32(conf.prof.dwGlobalOptions);
	    ptr->profile.dwB1ProtocolSupport = 
	      htole32(conf.prof.dwB1ProtocolSupport);
	    ptr->profile.dwB2ProtocolSupport = 
	      htole32(conf.prof.dwB2ProtocolSupport);
	    ptr->profile.dwB3ProtocolSupport = 
	      htole32(conf.prof.dwB3ProtocolSupport);
	    bcopy(conf.prof.bReserved,
		  ptr->profile.bReserved,
		  sizeof(ptr->profile.bReserved));
	    bcopy(conf.prof.bManufacturerSpecific,
		  ptr->profile.bManufacturerSpecific,
		  sizeof(ptr->profile.bManufacturerSpecific));
	    break;
	}

	case CAPI_SET_STACK_VERSION_REQ:
	case CAPI_IOCTL_TEST_REQ:
	case CAPI_START_D_CHANNEL_REQ:
	    return (0);

	default:
	    errno = ENOTTY;
	    return (-1);
	}

	if (error) {
	    errno = ENOTTY;
	    return (-1);
	}

	return (0); /* success */
}

