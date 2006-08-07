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
 * i4b_echo_cancel.c - A Normalized Least Mean Squares, NLMS, echo canceller
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
 * i4b_echo_cancel_hp_f1 - IIR filter number 1, high pass to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp_f1(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->low_pass_1 += ((in * (1<<8)) - (ec->low_pass_1 / (1<<8)));
    return (in - (ec->low_pass_1 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_hp_f2 - IIR filter number 2, high pass to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp_f2(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->low_pass_2 += ((in * (1<<8)) - (ec->low_pass_2 / (1<<8)));
    return (in - (ec->low_pass_2 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_lms - an implementation of a self adapting FIR filter
 *
 * inputs:
 *   x: sample from speaker
 *   y: sample from microphone
 *
 * outputs:
 *   sample from microphone without echo
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_lms(struct i4b_echo_cancel *ec, int16_t x, int16_t __y)
{
    u_int16_t n;
    int32_t power;
    int32_t factor;
    int32_t sample1 = 0;
    int32_t sample2 = 0;
    int32_t y = (__y * (1<<16));

    /*
     * store "x" in "buffer_x[]" and compute the 
     * squared of the last "I4B_ECHO_CANCEL_N_TAPS"
     * "x" values divided by "I4B_ECHO_CANCEL_N_TAPS" 
     * (optimized):
     */

    ec->buffer_x[ec->offset_x] = x;

    ec->avg_power_tx += (((x * x) + (I4B_ECHO_CANCEL_N_TAPS-1))
			 / I4B_ECHO_CANCEL_N_TAPS);

    x = ec->buffer_x[ec->offset_x + I4B_ECHO_CANCEL_N_TAPS];

    ec->avg_power_tx -= (((x * x) + (I4B_ECHO_CANCEL_N_TAPS-1))
			 / I4B_ECHO_CANCEL_N_TAPS);

    power = ec->avg_power_tx;

    /*
     * subtract the computed echo from the 
     * recorded sound. The following loop 
     * makes up a so-called FIR filter, which
     * means "Finite Impulse Response":
     */
    for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n += 2) {

        /* partial loop unrolled */

        sample1 += (ec->buffer_x[ec->offset_x + n] * 
		    (ec->coeffs_cur[n] / I4B_ECHO_CANCEL_COEFF_DP));

	sample2 += (ec->buffer_x[ec->offset_x + n + 1] * 
		    (ec->coeffs_cur[n + 1] / I4B_ECHO_CANCEL_COEFF_DP));
    }

    y -= (sample1 + sample2);

    /*
     * add a little noise to
     * influence the filter
     * noise:
     */
    y += i4b_echo_cancel_noise(ec) / 16;

    if (ec->coeffs_adapt) {

        if (power < (4*I4B_ECHO_CANCEL_COEFF_FACTOR)) {
            /*
	     * avoid division by zero and
	     * coefficient overflow:
	     */
            power = 4;
	} else {
            power /= I4B_ECHO_CANCEL_COEFF_FACTOR;
	}

	factor = (y / power);

        /* update the coefficients using the
	 * Normalized Least Means Squared,
	 * NLMS, method:
	 */
        for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n += 2) {

	    /* partial loop unrolled */

	    ec->coeffs_cur[n] += (ec->buffer_x[ec->offset_x + n] * factor);

	    ec->coeffs_cur[n+1] += (ec->buffer_x[ec->offset_x + n + 1] * factor);
	}
    }

#if 1
    if (ec->debug_count == 0) {
          int32_t mm_y = 0.0;
	u_int16_t mm_x = 0;

        ec->debug_count = 8000;

	for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {
	  if (ec->coeffs_cur[n] > mm_y) {
	      mm_y = ec->coeffs_cur[n];
	      mm_x = n;
	  }
	  if (ec->coeffs_cur[n] < -mm_y) {
	      mm_y = -ec->coeffs_cur[n];
	      mm_x = n;
	  }
	}

	I4B_DBG(1, L1_EC_MSG, "DC1=%d, DC2=%d,tx=%d,"
		"rx=%d,mm_x=%d,mm_y=%d,adapt=%d", 
		ec->low_pass_1 / (1<<16), 
		ec->low_pass_2 / (1<<16), 
		ec->tx_speaking, 
		ec->rx_speaking,
		mm_x, mm_y / I4B_ECHO_CANCEL_COEFF_DP,
		ec->coeffs_adapt);

    } else {
        ec->debug_count --;
    }
#endif

    if (ec->offset_x == 0) {

        /* update input offset */

        ec->offset_x = (I4B_ECHO_CANCEL_N_TAPS-1);
        bcopy(ec->buffer_x, ec->buffer_x + I4B_ECHO_CANCEL_N_TAPS,
	      I4B_ECHO_CANCEL_N_TAPS * sizeof(ec->buffer_x[0]));
    } else {
        ec->offset_x --;
    }

    return (y / (1<<16));
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
    const u_int32_t max = 512; /* this constant can be tuned */
    const u_int32_t level = 32; /* this constant can be tuned */
    const u_int32_t lim_coeffs = (I4B_ECHO_CANCEL_COEFF_DP * 
				  ((1<<16) / I4B_ECHO_CANCEL_N_TAPS));
    u_int32_t sum_coeffs = 0;
    u_int16_t n;
    void *ptr1;
    void *ptr2;

    ec->cur_power_tx += (x * x) / max;
    ec->cur_power_rx += (y * y) / max;
    ec->cur_power_count ++;

    if (ec->cur_power_count >= max) {

	ec->rx_speaking = ((ec->cur_power_rx > (level * level * max)) && 
			   (ec->cur_power_rx > (ec->cur_power_tx/
						(level * level))));

	ec->tx_speaking = ((ec->cur_power_tx > (level * level * max)) &&
			   (ec->cur_power_tx > (ec->cur_power_rx/
						(level * level))));

	ec->coeffs_adapt = (ec->tx_speaking && (!(ec->rx_speaking)));

	I4B_DBG(1, L1_EC_MSG, "rx/tx = %d, tx/rx = %d %dr/%dt",
		ec->cur_power_rx / (ec->cur_power_tx ? ec->cur_power_tx : 1),
		ec->cur_power_tx / (ec->cur_power_rx ? ec->cur_power_rx : 1),
		ec->rx_speaking, ec->tx_speaking);

	ptr1 = (ec->data_toggle) ? ec->coeffs_old_0 : ec->coeffs_old_1;
	ptr2 = (ec->data_toggle) ? ec->coeffs_old_1 : ec->coeffs_old_0;

	ec->data_toggle = !(ec->data_toggle);

	/* check the coefficients */

	for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {

	    if (ec->coeffs_cur[n] < 0)
	      sum_coeffs += -(ec->coeffs_cur[n] / I4B_ECHO_CANCEL_N_TAPS);
	    else
	      sum_coeffs += (ec->coeffs_cur[n] / I4B_ECHO_CANCEL_N_TAPS);
	}

	if (sum_coeffs >= lim_coeffs) {

	    /* make sure that the coefficients 
	     * do not diverge
	     */
	    I4B_DBG(1, L1_EC_MSG, "reverting bad coeffs!");
	}

	if (ec->coeffs_adapt && (sum_coeffs < lim_coeffs)) {

	    /* make a backup of the coefficients */

	    bcopy(ec->coeffs_cur, ptr1, sizeof(ec->coeffs_cur));

	} else {

	    /* restore the coefficients */

	    bcopy(ptr1, ec->coeffs_cur, sizeof(ec->coeffs_cur));

	    bcopy(ptr1, ptr2, sizeof(ec->coeffs_cur));
	}

	ec->cur_power_count = 0;
	ec->cur_power_tx = 0;
	ec->cur_power_rx = 0;
    }

    if (ec->tx_speaking && 
	ec->adapt_count) {
        ec->adapt_count--;

	if (ec->tx_speaking) {
	    /* brute force muting */
	    y /= 64;
	}
    }
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

	read_ptr[0] = convert_rev(sample_y);

        read_len--;
	read_ptr++;
    }
    return;
}
