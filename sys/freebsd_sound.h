/*-
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * NOTE: This is a lite implementation of "FreeBSD/src/sys/sys/dev/pcm/sound.h"
 */
#ifndef __FREEBSD_PCM_SOUND_H__
#define __FREEBSD_PCM_SOUND_H__

typedef struct _snd_chan_param {
    u_long      play_rate;      /* sampling rate                        */
    u_long      rec_rate;       /* sampling rate                        */
    u_long      play_format;    /* everything describing the format     */
    u_long      rec_format;     /* everything describing the format     */
} snd_chan_param;

#define AIOGFMT    _IOR('f', 12, snd_chan_param)   /* get format */
#define AIOSFMT    _IOWR('f', 12, snd_chan_param)  /* sets format */

#define AFMT_QUERY      0x00000000      /* Return current format */
#define AFMT_MU_LAW     0x00000001      /* Logarithmic mu-law */
#define AFMT_A_LAW      0x00000002      /* Logarithmic A-law */
#define AFMT_U8         0x00000008      /* Unsigned 8-bit */
#define AFMT_S8         0x00000040      /* Signed 8-bit */

#define AFMT_FULLDUPLEX 0x80000000 

typedef struct _snd_capabilities {
    u_long      rate_min, rate_max;     /* min-max sampling rate */
    u_long      formats;
    u_long      bufsize; /* DMA buffer size */
    u_long      mixers; /* bitmap of available mixers */
    u_long      inputs; /* bitmap of available inputs (per mixer) */
    u_short     left, right;    /* how many levels are supported */
} snd_capabilities;

#define AIOGCAP _IOWR('A', 15, snd_capabilities)        /* get capabilities */

struct snd_size {
    int play_size;
    int rec_size;
};

#define AIOGSIZE    _IOR('A', 11, struct snd_size)/* read current blocksize */
#define AIOSSIZE    _IOWR('A', 11, struct snd_size)  /* sets blocksize */

#define SD_F_SIMPLEX            0x00000001
#define SD_F_AUTOVCHAN          0x00000002
#define SD_F_PRIO_RD            0x10000000
#define SD_F_PRIO_WR            0x20000000
#define SD_F_PRIO_SET           (SD_F_PRIO_RD | SD_F_PRIO_WR)
#define SD_F_DIR_SET            0x40000000
#define SD_F_TRANSIENT          0xf0000000

#define CHN_F_CLOSING           0x00000004  /* a pending close */
#define CHN_F_ABORTING          0x00000008  /* a pending abort */
#define CHN_F_RUNNING           0x00000010  /* dma is running */
#define CHN_F_TRIGGERED         0x00000020
#define CHN_F_NOTRIGGER         0x00000040
#define CHN_F_BUSY              0x00001000  /* has been opened  */
#define CHN_F_HAS_SIZE          0x00002000  /* user set block size */
#define CHN_F_NBIO              0x00004000  /* do non-blocking i/o */
#define CHN_F_MAPPED            0x00010000  /* has been mmap()ed */
#define CHN_F_DEAD              0x00020000
#define CHN_F_BADSETTING        0x00040000
#define CHN_F_SETBLOCKSIZE      0x00080000
#define CHN_F_VIRTUAL           0x10000000  /* not backed by hardware */
#define CHN_F_RESET             (CHN_F_BUSY | CHN_F_DEAD | CHN_F_VIRTUAL)

struct snd_dbuf {
    u_int8_t *buf;
    unsigned int bufsize;
    volatile int dl; /* transfer size */
    volatile int rp; /* pointers to the ready area */
    volatile int rl; /* length of ready area */
};

struct pcm_channel {
    struct snd_dbuf *bufhard, *bufsoft;
    u_int32_t flags;
};

#define chn_rel(args...)
#define chn_start(args...)
#define chn_intr(args...)
#define chn_get(dev,rd,wr,flag) do {		\
    (rd)[0] = NULL;				\
    (wr)[0] = NULL;				\
} while(0)

#endif
