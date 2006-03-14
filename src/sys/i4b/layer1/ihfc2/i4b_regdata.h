/*-
 * Copyright (c) 2003 Hans Petter Selasky. All rights reserved.
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
 *	Register-data buffering support
 *	-------------------------------
 *
 * $FreeBSD: $
 *
 *      last edit-date: []
 *
 *---------------------------------------------------------------------------*/
#ifndef _REGDATA_H_
#define _REGDATA_H_

/* Checklist for regdata:
 * =====================
 * check that the right xfer is used
 * check that .bufsize == REGDATA_BUFFER_SIZE
 * check that regdata_reset() is added to chip_reset()
 */

#ifdef IHFC_USB_ENABLED

static u_int8_t
regdata_put_byte(ihfc_sc_t *sc, u_int8_t dir, u_int8_t reg, u_int8_t data)
{
	struct regdata *ptr;
	int len;

	BUF_GET_WRITELEN(&sc->sc_config_buffer.buf,&len);

	if(len < sizeof(*ptr))
	{
		/* buffer overflow:
		 *
		 * If the buffer overflows ihfc_chip_reset()
		 * should be called, so that the buffer is
		 * reset and all configuration data is re-
		 * loaded. The buffer must be large enough to
		 * hold the full configuration for the chip.
		 */
		IHFC_ERR("Buffer overflow!\n");
		ihfc_reset(sc,NULL);
		return 0;
	}

	/*
	 * put dir, reg and data
	 * to buffer
	 */

	ptr = (void *)BUF_GET_WRITEPTR(&sc->sc_config_buffer.buf);

	ptr->dir = dir;
	ptr->reg = reg;
	ptr->data = data;

	ptr++;

	BUF_GET_WRITEPTR(&sc->sc_config_buffer.buf) = (void *)ptr;
	return 1;
}

static u_int8_t
regdata_get_byte(ihfc_sc_t *sc, struct regdata *buf)
{
	struct regdata *ptr;
	int len;

	BUF_GET_READLEN(&sc->sc_config_buffer.buf,&len);

	/*
	 * get dir, reg and data
	 * from buffer [if any]
	 */

	if(len >= sizeof(*ptr))
	{
		ptr = (void *)BUF_GET_READPTR(&sc->sc_config_buffer.buf);

		*buf = *ptr;

		ptr++;

		BUF_GET_READPTR(&sc->sc_config_buffer.buf) = (void *)ptr;
		return 1;
	}
	else
	{
		return 0;
	}
}

static u_int16_t
regdata_count(ihfc_sc_t *sc, u_int8_t dir, u_int8_t reg)
{
	struct regdata buf;
	u_int16_t count, len;

	count = 0;

	BUF_GET_WRAPPED_READLEN(&sc->sc_config_buffer.buf,&len);

	for(;
	    len >= sizeof(sc->sc_config_buffer.data[0]);
	    len -= sizeof(sc->sc_config_buffer.data[0]))
	{
		regdata_get_byte(sc, &buf);

		if((buf.dir == dir) &&
		   (buf.reg == reg))
		{
			count++;
		}

		regdata_put_byte(sc, buf.dir, buf.reg, buf.data);
	}
	return count;
}

struct regdata_usb {
	u_int8_t data[32-2];

	u_int8_t current_register;
	u_int8_t current_data;
} __packed;

#define REGDATA_XFER_WRITE 0
#define REGDATA_BUFFER_WRITE_SIZE sizeof(struct regdata_usb)
#define REGDATA_XFER_READ 1
#define REGDATA_BUFFER_READ_SIZE sizeof(struct regdata_usb)

static void
regdata_usb_update (ihfc_sc_t *sc)
{
  struct usbd_xfer *xfer_rd = sc->sc_resources.usb_xfer[REGDATA_XFER_READ];
  struct usbd_xfer *xfer_wr = sc->sc_resources.usb_xfer[REGDATA_XFER_WRITE];
  struct regdata buf;

  mtx_assert(xfer_rd->priv_mtx, MA_OWNED);
  mtx_assert(xfer_wr->priv_mtx, MA_OWNED);

  if(!USBD_TRANSFER_IN_PROGRESS(xfer_rd) &&
     !USBD_TRANSFER_IN_PROGRESS(xfer_wr))
  {
    if(regdata_get_byte(sc, &buf))
    {
	((struct regdata_usb *)xfer_rd->buffer)->current_register = buf.reg;
	((struct regdata_usb *)xfer_rd->buffer)->current_data = buf.data;
	((struct regdata_usb *)xfer_wr->buffer)->current_register = buf.reg;
	((struct regdata_usb *)xfer_wr->buffer)->current_data = buf.data;

	/* this call might recurse: */
	usbd_transfer_start(sc->sc_resources.usb_xfer[buf.dir]);
    }
  }
  return;
}

static void
regdata_usb_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	while(len--)
	{
	  if(!regdata_put_byte(sc, REGDATA_XFER_WRITE, reg & 0xff, *ptr))
	  {
		break;
	  }
	  ptr++;
	}
	regdata_usb_update(sc);
	return;
}

static void
regdata_usb_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	if(ptr != NULL)
	{
		IHFC_ERR("ptr != NULL: Cannot read registers directly!\n");
	}
	while(len--)
	{
	  if(!regdata_put_byte(sc, REGDATA_XFER_READ, reg & 0xff, 0 /* no data */))
	  {
		break;
	  }
	  ptr++;
	}
	regdata_usb_update(sc);
	return;
}
#endif /* IHFC_USB_ENABLED */

#endif /* _REGDATA_H_ */
