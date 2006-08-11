/*-
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
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
 *
 * i4b_dtmf.c - An integer DTMF generator and detector
 *
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/kernel.h>
#include <net/if.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_trace.h>
#include <i4b/include/i4b_global.h>

/*
 * K[] = 2.0 * cos ( 2.0 * pi() * f / 8000.0 ) * 16384
 */

static const
int16_t K[I4B_DTMF_N_FREQ] = {

   /* 1st harmonic */

   27980, /* 697Hz */
   26956, /* 770Hz */
   25701, /* 852Hz */
   24219, /* 941Hz */
   19073, /* 1209Hz */
   16325, /* 1336Hz */
   13085, /* 1477Hz */
    9315, /* 1633Hz */

   /* 2nd harmonic */

   14739, /* 1406Hz (adjusted) */
   11221, /* 1555Hz (adjusted) */
    7549, /* 1704Hz */
    3032, /* 1882Hz */
  -10565, /* 2418Hz */
  -16503, /* 2672Hz */
  -22318, /* 2954Hz */
  -27472, /* 3266Hz */

   /* FAX tones */

   21281, /* 1100Hz */
   -2571, /* 2100Hz */
};

static const
u_int8_t code_to_ascii[16] = {
    '1', '4', '7', '*',
    '2', '5', '8', '0',
    '3', '6', '9', '#',
    'A', 'B', 'C', 'D',
};

struct dtmf_to_freq {
    u_int16_t f0; /* Hz */
    u_int16_t f1; /* Hz */
    u_int8_t key;
};

static const 
struct dtmf_to_freq dtmf_to_freq[] = {
    { 941, 1477, '#' },
    { 941, 1209, '*' },
    { 941, 1336, '0' },
    { 697, 1209, '1' },
    { 697, 1336, '2' },
    { 697, 1477, '3' },
    { 770, 1209, '4' },
    { 770, 1336, '5' },
    { 770, 1477, '6' },
    { 852, 1209, '7' },
    { 852, 1336, '8' },
    { 852, 1477, '9' },
    { 697, 1633, 'A' },
    { 770, 1633, 'B' },
    { 852, 1633, 'C' },
    { 941, 1633, 'D' },
    {   0,    0,  0  },
};

void
i4b_dtmf_init_rx(struct fifo_translator *ft, u_int8_t bsubprot)
{
    I4B_DBG(1, L1_DTMF_MSG, "init RX");

    bzero(&(ft->dtmf_rx), sizeof(ft->dtmf_rx));

    ft->dtmf_rx.bsubprot = bsubprot;

    return;
}

void
i4b_dtmf_init_tx(struct fifo_translator *ft, u_int8_t bsubprot)
{
    I4B_DBG(1, L1_DTMF_MSG, "init TX");

    bzero(&(ft->dtmf_tx), sizeof(ft->dtmf_tx));

    ft->dtmf_tx.bsubprot = bsubprot;

    return;
}

void
i4b_dtmf_queue_digit(struct fifo_translator *ft, u_int8_t digit,
		     u_int16_t tone_duration,
		     u_int16_t gap_duration)
{
    u_int8_t next_pos0 = (ft->dtmf_tx.input_pos +1) % I4B_DTMF_N_DIGITS;
    u_int8_t next_pos1 = (next_pos0 + 1) % I4B_DTMF_N_DIGITS;
    u_int8_t n;
    u_int8_t i;

    if ((next_pos0 == ft->dtmf_tx.input_pos) ||
	(next_pos1 == ft->dtmf_tx.input_pos)) {
        /* buffer overflow */
        I4B_DBG(1, L1_DTMF_MSG, "buffer overflow!");
        return;
    }

    if (tone_duration > 0x1FFF) {
        tone_duration = 0x1FFF;
    }

    if (tone_duration == 0) {
        tone_duration = 40; /* default */
    }

    /* convert to number of samples */
    tone_duration *= 8;

    if (gap_duration > 0x1FFF) {
        gap_duration = 0x1FFF;
    }

    if (gap_duration == 0) {
        gap_duration = 40; /* default */
    }

    /* convert to number of samples */
    gap_duration *= 8;

    /* lookup the frequency */
    for (n = 0; ; n++) {
      if ((dtmf_to_freq[n].key == digit) ||
	  (dtmf_to_freq[n].key == 0)) {
	    break;
	}
    }

    i = ft->dtmf_tx.output_pos;

    ft->dtmf_tx.freq0[i] = dtmf_to_freq[n].f0;
    ft->dtmf_tx.freq1[i] = dtmf_to_freq[n].f1;
    ft->dtmf_tx.duration[i] = tone_duration;

    i = next_pos0;

    ft->dtmf_tx.freq0[i] = 0;
    ft->dtmf_tx.freq1[i] = 0;
    ft->dtmf_tx.duration[i] = gap_duration;

    ft->dtmf_tx.output_pos = next_pos1;

    /* start the transmitter, if not already started */
    L1_FIFO_START(ft);

    return;
}

void
i4b_dtmf_generate(struct fifo_translator *ft, struct mbuf **pp_m)
{
    struct mbuf *m = *pp_m;
    u_int8_t *ptr;
    u_int32_t len;
    u_int32_t i = ft->dtmf_tx.input_pos;
    int32_t temp;
    u_int8_t silence = 0;
    u_int8_t allocated = 0;

    if (i == ft->dtmf_tx.output_pos) {
        /* nothing to do */
        return;
    }

    if (m == NULL) {

        m = i4b_getmbuf(800, M_NOWAIT);

	if (m == NULL) {
	    /* nothing to do */
	    return;
	}

	*pp_m = m;

        silence = ((ft->dtmf_tx.bsubprot == BSUBPROT_G711_ULAW) ?
		   i4b_signed_to_ulaw(0) :
		   i4b_signed_to_alaw(0));
	allocated = 1;
    }

    len = m->m_len;
    ptr = m->m_data;

    while (len--) {

        if (ft->dtmf_tx.duration[i] == 0) {

	    ft->dtmf_tx.omega0 = 0;
	    ft->dtmf_tx.omega1 = 0;

	    i++;

	    if (i >= I4B_DTMF_N_DIGITS) {
	        i = 0;
	    }

	    if (i == ft->dtmf_tx.output_pos) {

	        if (allocated) {

		    /* fill rest of buffer with silence */

		    *ptr = silence;

		    while (len--) {
		        *ptr++ = silence;
		    }
		}
	        break;
	    }

	} else {
	    ft->dtmf_tx.duration[i] --;
	}

	temp = ((i4b_sine_to_signed[ft->dtmf_tx.omega0] +
		 i4b_sine_to_signed[ft->dtmf_tx.omega1]) / 4);

	*ptr++ = ((ft->dtmf_tx.bsubprot == BSUBPROT_G711_ULAW) ?
		  i4b_signed_to_ulaw(temp) :
		  i4b_signed_to_alaw(temp));

	ft->dtmf_tx.omega0 += ft->dtmf_tx.freq0[i];

	if (ft->dtmf_tx.omega0 >= 8000) {
	    ft->dtmf_tx.omega0 -= 8000;
	}

	ft->dtmf_tx.omega1 += ft->dtmf_tx.freq1[i];

	if (ft->dtmf_tx.omega1 >= 8000) {
	    ft->dtmf_tx.omega1 -= 8000;
	}
    }
    return;
}

void
i4b_dtmf_detect(struct fifo_translator *ft, 
		u_int8_t *data_ptr, u_int16_t data_len)
{
    int32_t w2[I4B_DTMF_N_FREQ];
    int32_t sample;
    int32_t sample_cp;
    u_int32_t found;
    u_int32_t n;
    u_int8_t temp;
    u_int8_t code;

    while (data_len--) {

        sample = ((ft->dtmf_rx.bsubprot == BSUBPROT_G711_ALAW) ?
		  i4b_alaw_to_signed[*data_ptr] :
		  i4b_ulaw_to_signed[*data_ptr]); 

	data_ptr++;

	/* update "max_gain" */

	if (ft->dtmf_rx.max_gain > 0x100) {
	    ft->dtmf_rx.max_gain -= 8;
	} else {
	    ft->dtmf_rx.max_gain = 0x100;
	}

	if (sample == -0x8000) {
	    sample = -0x7FFF;
	}

	sample_cp = sample;

	/* normalize input data */

	sample = ((sample * 0x8000) - sample) / ft->dtmf_rx.max_gain;

	/* XXX the AGC could be better! XXX */

	/* check if sound is too loud */

	if ((sample > 0x7FFF) ||
	    (sample < -0x7FFF))
	{
	    if (sample_cp < 0) {
	        sample_cp = -sample_cp;
	    }

	    if (sample_cp < 0x100) {
	        sample_cp = 0x100;
	    }

	    ft->dtmf_rx.max_gain = sample_cp;

	    /* clip sound */

	    sample = ((sample < 0) ? -0x7FFF : 0x7FFF);
	}

	/* compute Goertzel filters */

	for (n = 0; n < I4B_DTMF_N_FREQ; n += 2) {

	    /* partial loop unrolling */

	    w2[n] = (((K[n] * (ft->dtmf_rx.w1[n] / (1<<7))) / (1<<7)) - 
		     (ft->dtmf_rx.w0[n]) + sample);

	    ft->dtmf_rx.w0[n] = ft->dtmf_rx.w1[n];
	    ft->dtmf_rx.w1[n] = w2[n];

	    w2[n+1] = (((K[n+1] * (ft->dtmf_rx.w1[n+1] / (1<<7))) / (1<<7)) - 
		     (ft->dtmf_rx.w0[n+1]) + sample);

	    ft->dtmf_rx.w0[n+1] = ft->dtmf_rx.w1[n+1];
	    ft->dtmf_rx.w1[n+1] = w2[n+1];
	}

	ft->dtmf_rx.count++;

	if (ft->dtmf_rx.count == I4B_DTMF_N_SAMPLES) {

	    found = 0;

	    for (n = 0; n < I4B_DTMF_N_FREQ; n++) {

	        ft->dtmf_rx.w0[n] /= (1<<7);
		ft->dtmf_rx.w1[n] /= (1<<7);

		w2[n] = ((ft->dtmf_rx.w0[n]*ft->dtmf_rx.w0[n]) + 
			 (ft->dtmf_rx.w1[n]*ft->dtmf_rx.w1[n]) -
			 (((K[n] * ft->dtmf_rx.w0[n]) / (1<<7)) * 
			  (ft->dtmf_rx.w1[n] / (1<<7))));

		found |= (w2[n] >= (1 << 24)) << n;

		I4B_DBG(1, L1_DTMF_MSG, "tone[%d]=%d %s",
			n, w2[n], (found & (1<<n)) ? "over threshold" : 
			"under threshold");
	    }

	    if (found == (1<<16)) {

	        /* fax tone 1.1 kHz */
	        code = 'X';

	    } else if (found == (1<<17)) {

	        /* fax tone 2.1 kHz */
	        code = 'Y';

	    } else {

	        /* DTMF tone */
	        temp = (found & 0xF);

		if (temp == 1) {
		    code = 0;
		} else if (temp == 2) {
		    code = 1;
		} else if (temp == 4) {
		    code = 2;
		} else if (temp == 8) {
		    code = 3;
		} else {
		    code = 0;
		    goto done;
		}

		if (found & 0xFFFFFF00) {
		    code = 0;
		    goto done;
		}

		temp = ((found >> 4) & 0xF);

		if (temp == 1) {
		    code |= 0x0;
		} else if (temp == 2) {
		    code |= 0x4;
		} else if (temp == 4) {
		    code |= 0x8;
		} else if (temp == 8) {
		    code |= 0xC;
		} else {
		    code = 0;
		    goto done;
		}

		code = code_to_ascii[code];
	    }
	done:
	    if (ft->dtmf_rx.code != code) {
	        ft->dtmf_rx.code = code;
		if (code) {
		    L5_PUT_DTMF(ft, &code, 1);
		}
	    }

	    /* reset detector */
	    bzero(ft->dtmf_rx.w1, sizeof(ft->dtmf_rx.w1));
	    bzero(ft->dtmf_rx.w0, sizeof(ft->dtmf_rx.w0));
	    ft->dtmf_rx.count = 0;
	    ft->dtmf_rx.max_gain = 0;
	}
    }
    return;
}
