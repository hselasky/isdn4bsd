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
 * i4b_echo_cancel.c - An echo canceller
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
		     u_int8_t is_ulaw)
{
    bzero(ec, sizeof(*ec));

    ec->adapt_count = 8000;

    ec->noise_rem = 1;

    ec->pre_delay = pre_delay;
    ec->last_byte = 0xFF;
    ec->is_ulaw = is_ulaw;

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
 * i4b_echo_cancel_hp1 - IIR high pass filter number 1, to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp1(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->high_pass_1 += ((in * (1<<8)) - (ec->high_pass_1 / (1<<8)));
    return (in - (ec->high_pass_1 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_hp2 - IIR high pass filter number 1, to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp2(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->high_pass_2 += ((in * (1<<8)) - (ec->high_pass_2 / (1<<8)));
    return (in - (ec->high_pass_2 / (1<<16)));
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
i4b_echo_cancel_lms(struct i4b_echo_cancel *ec, int16_t x, int16_t y)
{
    u_int16_t n;
    int32_t sample1 = 0;
    int32_t sample2 = 0;
    int32_t temp1 = 0;
    int32_t temp2 = 0;
    int32_t max1 = 0;
    int32_t max2 = 0;
    int32_t sum1 = 0;
    int32_t sum2 = 0;

    ec->buffer_1[ec->offset_1] = x; // + (i4b_echo_cancel_noise(ec) / (1<<18));

    /*
     * subtract the computed echo from the 
     * recorded sound. The following loop 
     * makes up a so-called FIR filter, which
     * means "Finite Impulse Response":
     */
    for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n += 2) {

        /* partial loop unrolled */

        sample1 += (ec->buffer_1[ec->offset_1 + n] * 
		    (ec->coeffs[n] / (1<<(16-I4B_ECHO_CANCEL_COEFF_ADJ))));

        sample2 += (ec->buffer_1[ec->offset_1 + n + 1] * 
		    (ec->coeffs[n+1] / (1<<(16-I4B_ECHO_CANCEL_COEFF_ADJ))));
    }

    y -= ((sample1 + sample2) / (1<<16));

    if (ec->adapt_enabled) {

        /* update coefficients, filter learning */

        for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n += 2) {

	    /* partial loop unrolled */

	    ec->coeffs[n] += (((ec->buffer_1[ec->offset_1 + n] * y) /
				ec->adapt_divisor) * ec->adapt_factor);

	    temp1 = (((ec->coeffs[n] < 0) ? ec->coeffs[n] : -ec->coeffs[n]) 
		     / I4B_ECHO_CANCEL_N_TAPS);

	    sum1 += temp1;

	    if (temp1 < max1) {
	        max1 = temp1;
	    }

	    ec->coeffs[n+1] += (((ec->buffer_1[ec->offset_1 + n + 1] * y) /
				  ec->adapt_divisor) * ec->adapt_factor);

	    temp2 = (((ec->coeffs[n+1] < 0) ? ec->coeffs[n+1] : -ec->coeffs[n+1])
		     / I4B_ECHO_CANCEL_N_TAPS);

	    sum2 += temp2;

	    if (temp2 < max2) {
	       max2 = temp2;
	    }
	}

	if (max2 < max1) {
	    max1 = max2;
	}

	sum1 += sum2;

	if ((max1 <= -(1<<(32-I4B_ECHO_CANCEL_COEFF_ADJ))) ||
	    (sum1 <= -((1<<(32-I4B_ECHO_CANCEL_COEFF_ADJ)) / 
		       I4B_ECHO_CANCEL_N_TAPS))) {

	    I4B_DBG(1, L1_EC_MSG, "Resetting bad coeffs "
		    "max1=0x%08x, sum1=0x%08x!", max1, sum1);

	    bzero(ec->coeffs, sizeof(ec->coeffs));

	    ec->adapt_count = 8000;
	}
    }
#if 1
    if (ec->debug_count == 0) {
        int32_t mm_y = 0, mm_x = 0;

        ec->debug_count = 8000;

	for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {
	  if (ec->coeffs[n] > mm_y) {
	      mm_y = ec->coeffs[n];
	      mm_x = n;
	  }
	  if (ec->coeffs[n] < -mm_y) {
	      mm_y = -ec->coeffs[n];
	      mm_x = n;
	  }
	}

	I4B_DBG(1, L1_EC_MSG,  "> DC1=%d,DC2=%d,tx=%d,rx=%d,mm_x=%d,mm_y=0x%08x", 
		ec->high_pass_1 / (1<<16), 
		ec->high_pass_2 / (1<<16), 
		ec->tx_speaking, 
		ec->rx_speaking,
		mm_x, mm_y);
    } else {
        ec->debug_count --;
    }
#endif

    if (ec->offset_1 == 0) {

        /* update input offset */

        ec->offset_1 = (I4B_ECHO_CANCEL_N_TAPS-1);
        bcopy(ec->buffer_1, ec->buffer_1 + I4B_ECHO_CANCEL_N_TAPS,
	      I4B_ECHO_CANCEL_N_TAPS * sizeof(ec->buffer_1[0]));
    } else {
        ec->offset_1 --;
    }

    return y;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_pwr - a simple echo suppressor
 *
 * inputs:
 *   x: sample from speaker
 *   y: sample from microphone
 *
 * outputs:
 *   sample from microphone without echo
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_pwr(struct i4b_echo_cancel *ec, int16_t x, int16_t y)
{

    ec->cur_power_tx += (x * x) / 256;
    ec->cur_power_rx += (y * y) / 256;
    ec->cur_power_count ++;

    if (ec->cur_power_count >= 256) {

        ec->rx_speaking = ((ec->cur_power_rx > 1024) && 
			   (ec->cur_power_rx > (ec->cur_power_tx/4)));

	ec->tx_speaking = ((ec->cur_power_tx > 1024) && 
			   (ec->cur_power_tx > (ec->cur_power_rx/4)));

	ec->adapt_enabled = ec->tx_speaking;
	ec->adapt_factor = 1;
	ec->adapt_divisor = 1024;
    }

    if (ec->adapt_enabled && ec->adapt_count) {
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

        ec->buffer_2[ec->offset_wr] = *ptr;

	if (ec->offset_wr == 0) {

	    /* update input offset */

            ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
	    bcopy(ec->buffer_2, ec->buffer_2 + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buffer_2[0]));

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

        I4B_DBG(1, L1_EC_MSG,  "Adjusting pre-delay buffer "
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
    u_int8_t  once = 0;
      int16_t sample_x;
      int16_t sample_y;

    const int16_t *convert_fwd = (ec->is_ulaw) ? i4b_ulaw_to_signed : i4b_alaw_to_signed;
    i4b_convert_rev_t *convert_rev = (ec->is_ulaw) ? i4b_signed_to_ulaw : i4b_signed_to_alaw;

    while(read_len) {

        if (ec->offset_rd <= ec->offset_wr) {
	    sample_x = convert_fwd[ec->last_byte];

	    if (once == 0) {
	        once = 1;
	        I4B_DBG(1, L1_EC_MSG, "repeating last byte!");
	    }

	} else {
	    sample_x = convert_fwd[ec->buffer_2[ec->offset_rd]];
	    ec->offset_rd--;
	}

	sample_y = convert_fwd[read_ptr[0]];

	/* high pass sound to remove DC */

	sample_x = i4b_echo_cancel_hp1(ec, sample_x);
	sample_y = i4b_echo_cancel_hp2(ec, sample_y);

	/* filter */

	sample_y = i4b_echo_cancel_lms(ec, sample_x, sample_y);
	sample_y = i4b_echo_cancel_pwr(ec, sample_x, sample_y);

	read_ptr[0] = convert_rev(sample_y);

        read_len--;
	read_ptr++;
    }
    return;
}
