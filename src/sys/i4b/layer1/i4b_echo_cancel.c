/*-
 * Copyright (c) 2006 Hans Petter Selasky. All rights reserved.
 * Copyright (c) 2004-2005 DFS Deutsche Flugsicherung. All rights reserved.
 * Copyright (c) 2004-2005 Andre Adrian. All rights reserved.
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
 * i4b_echo_cancel.c - An integer, dual sub-band, Normalized Least 
 *                     Mean Square, NLMS, echo canceller.
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

struct ec_stats {
  u_int32_t sum_y;
    int32_t max_y;
    int16_t max_x;
};

#if (I4B_ECHO_CANCEL_N_SUB != 0x2)
#error "Please update code!"
#endif

#if (I4B_ECHO_CANCEL_K_TAPS != 0x20)
#error "Please update code!"
#endif

#if (I4B_ECHO_CANCEL_W_DP != 0x20000000)
#error "Please update code!"
#endif

static const int32_t buf_H[2][32] = {
  {
             7,
     -11392754,
            -7,
      13145486,
             7,
     -15535574,
            -7,
      18987924,
             7,
     -24413045,
            -7,
      34178263,
             7,
     -56963772,
            -7,
     170891318,
     268435456,
     170891318,
            -7,
     -56963772,
             7,
      34178263,
            -7,
     -24413045,
             7,
      18987924,
            -7,
     -15535574,
             7,
      13145486,
            -7,
             0,
  },
  {
            -7,
      11392754,
             7,
     -13145486,
            -7,
      15535574,
             7,
     -18987924,
            -7,
      24413045,
             7,
     -34178263,
            -7,
      56963772,
             7,
    -170891318,
     268435456,
    -170891318,
             7,
      56963772,
            -7,
     -34178263,
             7,
      24413045,
            -7,
     -18987924,
             7,
      15535574,
            -7,
     -13145486,
             7,
             0,
  },
};

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_init - initialize echo canceller
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_init(struct i4b_echo_cancel *ec, 
		     u_int16_t pre_delay,
		     u_int8_t sub_bprot)
{
    bzero(ec, sizeof(*ec));

    ec->adapt_count = I4B_ECHO_CANCEL_ADAPT_COUNT;

    ec->noise_rem = 1;

    ec->pre_delay = pre_delay;
    ec->last_byte = 0xFF;

    ec->is_ulaw = (sub_bprot != BSUBPROT_G711_ALAW);

    ec->offset_x = (I4B_ECHO_CANCEL_N_TAPS-1);

    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_noise - a perceptual white noise generator
 *---------------------------------------------------------------------------*/
static __inline int32_t
i4b_echo_cancel_noise(struct i4b_echo_cancel *ec)
{
    u_int32_t temp;
    const u_int32_t prime = 0xFFFF1D;

    if (ec->noise_rem & 1) {
        ec->noise_rem += prime;
    }

    ec->noise_rem /= 2;

    temp = ec->noise_rem;

    /* unsigned to signed conversion */

    temp ^= 0x800000;
    if (temp & 0x800000) {
        temp |= (-0x800000);
    }

    return temp;
}

/*---------------------------------------------------------------------------*
 * i4b_subract_safe - safe subtraction
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_subtract_safe(int16_t a, int16_t b)
{
    int32_t temp = a - b;

    /* make room for adding 
     * a little white noise
     */
    if (temp < -I4B_ECHO_CANCEL_SAMPLE_MAX) {
        temp = -I4B_ECHO_CANCEL_SAMPLE_MAX;
    } else if (temp > I4B_ECHO_CANCEL_SAMPLE_MAX) {
        temp = I4B_ECHO_CANCEL_SAMPLE_MAX;
    }
    return temp;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_hp_f1 - IIR filter number 1, high pass to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp_f1(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->low_pass_1 += ((in * (1<<8)) - (ec->low_pass_1 / (1<<8)));
    return i4b_subtract_safe(in, (ec->low_pass_1 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_hp_f2 - IIR filter number 2, high pass to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp_f2(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->low_pass_2 += ((in * (1<<8)) - (ec->low_pass_2 / (1<<8)));
    return i4b_subtract_safe(in, (ec->low_pass_2 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_stats - compute coefficient statistics
 *---------------------------------------------------------------------------*/
static struct ec_stats
i4b_echo_cancel_stats(int32_t *p_coeffs)
{
    struct ec_stats stats;
    int32_t temp;
    u_int16_t n;

    stats.max_y = 0;
    stats.max_x = 0;
    stats.sum_y = 0;

    for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {

        temp = p_coeffs[n];

	if (temp < 0) {
	    if (temp == -0x80000000) {
	        /* avoid overflow */
	        temp = 0x7FFFFFFF;
	    } else {
	        temp = -temp;
	    }
	}

	stats.sum_y += temp / I4B_ECHO_CANCEL_N_TAPS;

	if (temp > stats.max_y) {
	    stats.max_y = temp;
	    stats.max_x = n;
	}
    }
    return stats;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_clamp - sample rangecheck
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_clamp(int32_t val)
{
    val /= I4B_ECHO_CANCEL_X_DP;

    if (val < -I4B_ECHO_CANCEL_SAMPLE_MAX) {
        val = -I4B_ECHO_CANCEL_SAMPLE_MAX;
    } else if (val > I4B_ECHO_CANCEL_SAMPLE_MAX) {
        val = I4B_ECHO_CANCEL_SAMPLE_MAX;
    }
    return val;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_inner_product - compute inner-product
 *---------------------------------------------------------------------------*/
static int32_t
i4b_echo_cancel_inner_product(const int16_t *p_x, const int32_t *p_c, u_int16_t n)
{
    int32_t temp = 0;
    while(n--) {
        temp += (*p_x) * ((*p_c) / (I4B_ECHO_CANCEL_W_DP / I4B_ECHO_CANCEL_X_DP));
	p_x++;
	p_c++;
    }
    return temp;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_lms - an implementation of a self adapting FIR filter
 *
 * inputs:
 *   x0: sample from speaker
 *   y0: sample from microphone with echo from speaker
 *
 * outputs:
 *   sample from microphone without echo from speaker
 *---------------------------------------------------------------------------*/
static int16_t
i4b_echo_cancel_lms(struct i4b_echo_cancel *ec, int16_t x0, int16_t y0)
{
    int32_t y1;
    int32_t y2;
    int32_t y3;

    u_int16_t n;
    u_int16_t p;

    ec->buf_X0[ec->offset_x] = x0;

    /* compute the estimated signal */

    y1 = i4b_echo_cancel_inner_product
      (ec->buf_X0 + ec->offset_x, ec->buf_W1, I4B_ECHO_CANCEL_N_TAPS);

    y1 = (y0 * I4B_ECHO_CANCEL_X_DP) - y1;

    /* store error */

    ec->buf_E0[ec->offset_x] = i4b_echo_cancel_clamp(y1);

    /* iterate over the sub-bands */

    for (p = 0; p < I4B_ECHO_CANCEL_N_SUB; p++) {

        /* compute X-sub-band */

        y2 = i4b_echo_cancel_inner_product
	  (ec->buf_X0 + ec->offset_x, buf_H[p], I4B_ECHO_CANCEL_K_TAPS);
      
	/* store X-sub-band */

	ec->buf_XH[p][ec->offset_x] = i4b_echo_cancel_clamp(y2);

	/* compute E-sub-band */

        y2 = i4b_echo_cancel_inner_product
	  (ec->buf_E0 + ec->offset_x, buf_H[p], I4B_ECHO_CANCEL_K_TAPS);

	/* compute sum of XH squared (optimized) */

	y3 = ec->buf_XH[p][ec->offset_x];

	ec->buf_PH[p] += (y3 * y3) / I4B_ECHO_CANCEL_N_TAPS;

	y3 = ec->buf_XH[p][ec->offset_x + I4B_ECHO_CANCEL_N_TAPS];

	ec->buf_PH[p] -= (y3 * y3) / I4B_ECHO_CANCEL_N_TAPS;

	y3 = ec->buf_PH[p];

	if (ec->coeffs_adapt) {

	    /* update W1 */

	    if (y3 > (64 * 64)) {

	        y2 /= y3;

		if (y2) {
		    for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {
		        ec->buf_W1[n] += y2 * ec->buf_XH[p][ec->offset_x + n];
		    }
		}
	    }
	}
    }

    if (ec->offset_x == 0) {

        ec->offset_x = (I4B_ECHO_CANCEL_N_TAPS-1);

        bcopy(ec->buf_X0,
	      ec->buf_X0 + I4B_ECHO_CANCEL_N_TAPS,
	      sizeof(ec->buf_X0[0]) * I4B_ECHO_CANCEL_N_TAPS);

        bcopy(ec->buf_E0,
	      ec->buf_E0 + I4B_ECHO_CANCEL_N_TAPS,
	      sizeof(ec->buf_E0[0]) * I4B_ECHO_CANCEL_N_TAPS);

	for (p = 0; p < I4B_ECHO_CANCEL_N_SUB; p++) {

	    bcopy(ec->buf_XH[p],
		  ec->buf_XH[p] + I4B_ECHO_CANCEL_N_TAPS,
		  sizeof(ec->buf_XH[0][0]) * I4B_ECHO_CANCEL_N_TAPS);
	}

    } else {
        ec->offset_x --;
    }

    y1 = i4b_echo_cancel_clamp(y1);

    /*
     * Add a little comfort noise
     */
    y1 += i4b_echo_cancel_noise(ec) / (1<<20);

    return y1;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_pwr - a simple echo suppressor
 *
 * inputs:
 *   x: sample from speaker
 *   y: sample from microphone (after FIR filter)
 *   z: sample from microphone (before FIR filter)
 *
 * outputs:
 *   sample from microphone without echo
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_pwr(struct i4b_echo_cancel *ec, 
		    int16_t x, int16_t y, int16_t z)
{
    const u_int32_t max = 256; /* this constant can be tuned */
    const u_int32_t diff_level = 64 * 64; /* this constant can be tuned */
    const u_int32_t echo_level = 32; /* this constant can be tuned */
    const u_int32_t lim_coeffs = (I4B_ECHO_CANCEL_W_DP / 
				  I4B_ECHO_CANCEL_N_TAPS);
    const u_int32_t min_level = (echo_level * echo_level * max);

    struct ec_stats stats;

    ec->cur_power_tx += (x * x) / max;
    ec->cur_power_rx0 += (y * y) / max; /* after echo cancel */
    ec->cur_power_rx1 += (z * z) / max; /* before echo cancel */
    ec->cur_power_count ++;

    if (ec->cur_power_count >= max) {

        ec->rx_speaking = ((ec->cur_power_rx0 > min_level) &&
			   (ec->cur_power_rx0 > 
			    (ec->cur_power_tx/diff_level)));

	ec->tx_speaking = ((ec->cur_power_tx > min_level) &&
			   (ec->cur_power_tx > 
			    (ec->cur_power_rx0/diff_level)));

	ec->coeffs_bad = ((ec->cur_power_rx1 > min_level) &&
			  ((ec->cur_power_rx0/4) > 
			   ec->cur_power_rx1));

	ec->coeffs_adapt = (ec->tx_speaking && 
			    (!(ec->rx_speaking)));

	stats = i4b_echo_cancel_stats(ec->buf_W1);

	I4B_DBG(1, L1_EC_MSG, "rx/tx=%d tx/rx=%d rx0=%d rx1=%d "
		"%dr/%dt/%db/%da xm=%d ym=%d",
		ec->cur_power_rx0 / (ec->cur_power_tx ? ec->cur_power_tx : 1),
		ec->cur_power_tx / (ec->cur_power_rx0 ? ec->cur_power_rx0 : 1),
		ec->cur_power_rx0, ec->cur_power_rx1, 
		ec->rx_speaking, ec->tx_speaking, 
		ec->coeffs_bad, ec->coeffs_adapt,
		stats.max_x, (int32_t)(stats.max_y / 
				       (I4B_ECHO_CANCEL_W_DP / 
					(1 << 16))));

	/* check adapt boolean */

	if (ec->coeffs_adapt) {
	    if (ec->coeffs_wait) {
	        ec->coeffs_wait--;
		ec->coeffs_adapt = 0;
	    }
	} else {
	    if (ec->adapt_count == 0) {
	        ec->coeffs_wait = I4B_ECHO_CANCEL_ADAPT_HIST + 1;
	    }
	}

	if (ec->coeffs_bad) {

	    /* coeffs are bad, reset echo canceller */

	    bzero(ec->buf_W0, sizeof(ec->buf_W0));
	    bzero(ec->buf_W1, sizeof(ec->buf_W1));
	    bzero(ec->buf_WA, sizeof(ec->buf_WA));

	} else {

	    if (ec->coeffs_adapt) {

	        /* check the coefficients */

	        if (stats.sum_y >= lim_coeffs) {

		    /* make sure that the coefficients 
		     * do not diverge
		     */
		    I4B_DBG(1, L1_EC_MSG, "reverting bad current coeffs!");

		    bcopy(ec->buf_W0,
			  ec->buf_W1, sizeof(ec->buf_W1));
		} else {
		    bcopy(ec->buf_W1,
			  ec->buf_W0, sizeof(ec->buf_W0));

		    bcopy(ec->buf_W1,
			  ec->buf_WA[ec->adapt_index],sizeof(ec->buf_WA[0]));

		    /* update adapt index */

		    ec->adapt_index ++;
		    ec->adapt_index %= I4B_ECHO_CANCEL_ADAPT_HIST;
		}
	    } else {

	        u_int8_t n;

		for (n = 0; n < I4B_ECHO_CANCEL_ADAPT_HIST; n++) {
		    if (n != ec->adapt_index) {
		        bcopy(ec->buf_WA[ec->adapt_index],
			      ec->buf_WA[n],
			      sizeof(ec->buf_WA[0]));
		    }
		}

		bcopy(ec->buf_WA[ec->adapt_index],
		      ec->buf_W1, sizeof(ec->buf_W1));

		bcopy(ec->buf_WA[ec->adapt_index],
		      ec->buf_W0, sizeof(ec->buf_W0));
	    }
	}

	ec->cur_power_count = 0;
	ec->cur_power_tx = 0;
	ec->cur_power_rx0 = 0;
	ec->cur_power_rx1 = 0;
    }

#if 1
    if (ec->tx_speaking && 
	ec->adapt_count) {
        ec->adapt_count--;
	/* brute force muting */
	y /= 64;
    }
#endif
    return y;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_update_feeder - set feed state for echo canceller
 *
 * input:
 *   tx_time: time in units of 125us, when the TX-buffer was empty
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_update_feeder(struct i4b_echo_cancel *ec,
			      u_int16_t tx_time)
{
    ec->tx_time = tx_time;
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_feed - feed data from speaker, with pre-delay
 *
 * input:
 *   ptr: pointer to input samples, bitreversed A-law or u-law
 *   len: length of data
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_feed(struct i4b_echo_cancel *ec, 
		     u_int8_t *ptr, u_int16_t len)
{
    if(len) {
        ec->last_byte = ptr[len-1];
    }

    /* extra range check, just in case */

    if (ec->offset_wr > (I4B_ECHO_CANCEL_F_SIZE-1)) {
        ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
    }

    while(len) {

        ec->buffer_y[ec->offset_wr] = *ptr;

	if (ec->offset_wr == 0) {

	    /* update input offset */

            ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
	    bcopy(ec->buffer_y, ec->buffer_y + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buffer_y[0]));

	    /* update output offset */

	    ec->offset_rd += I4B_ECHO_CANCEL_F_SIZE;
	    if (ec->offset_rd > ((2*I4B_ECHO_CANCEL_F_SIZE)-1)) {
	        ec->offset_rd = ((2*I4B_ECHO_CANCEL_F_SIZE)-1);
	    }

	} else {
            ec->offset_wr --;
	}

	ptr++;
	len--;
    }
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_update_merger - update the echo canceller sound merger
 *
 * input:
 *   rx_time: time in units of 125us, when the RX-buffer was empty
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_update_merger(struct i4b_echo_cancel *ec,
			      u_int16_t rx_time)
{
    u_int16_t d_time;
    u_int16_t d_len;

    ec->rx_time = rx_time;

    /* compute hardware delay time */

    d_time = (ec->tx_time - ec->rx_time) + ec->pre_delay;

    if (d_time > (I4B_ECHO_CANCEL_F_SIZE-1)) {
        d_time = (I4B_ECHO_CANCEL_F_SIZE-1);
    }

    /* extra range checks, just in case */

    if (ec->offset_wr > (I4B_ECHO_CANCEL_F_SIZE-1)) {
        ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
    }

    if (ec->offset_rd > ((2*I4B_ECHO_CANCEL_F_SIZE)-1)) {
        ec->offset_rd = ((2*I4B_ECHO_CANCEL_F_SIZE)-1);
    }

    if (ec->offset_rd < ec->offset_wr) {
        ec->offset_rd = ec->offset_wr;
    }

    /* compute the current read length */

    d_len = ec->offset_rd - ec->offset_wr;

    /* check that time and length matches */

    if (((d_time > 4) && 
	 (d_len < (d_time-4))) ||
	(d_len > (d_time+4))) {

        I4B_DBG(1, L1_EC_MSG, "Adjusting pre-delay buffer "
		"from %d to %d bytes!", d_len, d_time);

	/* adjust */

	ec->offset_rd = d_time + ec->offset_wr;
    }
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_merge - remove echo from microphone sound
 *
 * input:
 *   read_ptr: pointer to input samples, bitreversed A-law or u-law
 *   read_len: length of data
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_merge(struct i4b_echo_cancel *ec, 
		      u_int8_t *read_ptr, u_int16_t read_len)
{
    u_int8_t once = 0;
      int16_t sample_x; /* sample from speaker */
      int16_t sample_y; /* sample from microphone */
      int16_t sample_z; /* sample from microphone (copy) */

    const int16_t *convert_fwd = 
      ((ec->is_ulaw) ? i4b_ulaw_to_signed : i4b_alaw_to_signed);

    i4b_convert_rev_t *convert_rev = 
      ((ec->is_ulaw) ? i4b_signed_to_ulaw : i4b_signed_to_alaw);

    while(read_len) {

        if (ec->offset_rd <= ec->offset_wr) {
	    sample_x = convert_fwd[ec->last_byte];

	    if (once == 0) {
	        once = 1;
		I4B_DBG(1, L1_EC_MSG, "repeating last byte");
	    }
	} else {
	    sample_x = convert_fwd[ec->buffer_y[ec->offset_rd]];
	    ec->offset_rd--;
	}

	sample_y = convert_fwd[read_ptr[0]];

	/* high pass sound to remove DC */

	sample_x = i4b_echo_cancel_hp_f1(ec, sample_x);
	sample_y = i4b_echo_cancel_hp_f2(ec, sample_y);
	sample_z = sample_y;

	/* filter */

	sample_y = i4b_echo_cancel_lms(ec, sample_x, sample_y);
	sample_y = i4b_echo_cancel_pwr(ec, sample_x, sample_y, sample_z);

	/* update */

	read_ptr[0] = convert_rev(sample_y);

        read_len--;
	read_ptr++;
    }
    return;
}
