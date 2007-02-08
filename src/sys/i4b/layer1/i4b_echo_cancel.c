/*-
 * Copyright (c) 2006-2007 Hans Petter Selasky. All rights reserved.
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
 * i4b_echo_cancel.c - An integer, trippel sub-band, Normalized Least 
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

static int32_t
i4b_echo_cancel_inner_product_H0(register const int16_t *p_x)
{
#if (I4B_ECHO_CANCEL_X_DP != 0x8000)
#error "Please regenerate this code!"
#endif
#if (I4B_ECHO_CANCEL_K_TAPS != 0x20)
#error "Please regenerate this code!"
#endif
    /* 1kHz low-pass filter */
    register int32_t temp = 0;
    temp += ((p_x[1] + p_x[31]) * -491);
    temp += ((p_x[2] + p_x[30]) * -745);
    temp += ((p_x[3] + p_x[29]) * -567);
    temp += ((p_x[5] + p_x[27]) * 670);
    temp += ((p_x[6] + p_x[26]) * 1043);
    temp += ((p_x[7] + p_x[25]) * 819);
    temp += ((p_x[9] + p_x[23]) * -1053);
    temp += ((p_x[10] + p_x[22]) * -1738);
    temp += ((p_x[11] + p_x[21]) * -1475);
    temp += ((p_x[13] + p_x[19]) * 2458);
    temp += ((p_x[14] + p_x[18]) * 5215);
    temp += ((p_x[15] + p_x[17]) * 7375);
    temp += ((p_x[16]) * 8192);
    return temp;
}

static int32_t
i4b_echo_cancel_inner_product_H1(register const int16_t *p_x)
{
#if (I4B_ECHO_CANCEL_X_DP != 0x8000)
#error "Please regenerate this code!"
#endif
#if (I4B_ECHO_CANCEL_K_TAPS != 0x20)
#error "Please regenerate this code!"
#endif
    /* 2kHz low-pass filter */
    register int32_t temp = 0;
    temp += ((p_x[1] + p_x[31]) * -695);
    temp += ((p_x[3] + p_x[29]) * 802);
    temp += ((p_x[5] + p_x[27]) * -948);
    temp += ((p_x[7] + p_x[25]) * 1158);
    temp += ((p_x[9] + p_x[23]) * -1490);
    temp += ((p_x[11] + p_x[21]) * 2086);
    temp += ((p_x[13] + p_x[19]) * -3476);
    temp += ((p_x[15] + p_x[17]) * 10430);
    temp += ((p_x[16]) * 16384);
    return temp;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_init - initialize echo canceller
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_init(struct i4b_echo_cancel *ec, 
		     u_int16_t pre_delay,
		     u_int8_t sub_bprot)
{
    bzero(ec, sizeof(*ec));

    ec->last_byte = 0xFF;

    ec->adapt_count = I4B_ECHO_CANCEL_ADAPT_COUNT;

    ec->coeffs_wait = I4B_ECHO_CANCEL_ADAPT_HIST;

    ec->noise_rem = 1;

    ec->pre_delay = pre_delay;

    ec->is_ulaw = (sub_bprot != BSUBPROT_G711_ALAW);

    ec->offset_x = (I4B_ECHO_CANCEL_K_TAPS-1);

    ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);

    ec->offset_rd = (I4B_ECHO_CANCEL_F_SIZE-1);

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
i4b_echo_cancel_inner_product(const int16_t *p_x, const int32_t *p_c, uint16_t n)
{
    int64_t temp = 0;
    while (n >= 16) {
        temp += p_x[0] * p_c[0];
        temp += p_x[1] * p_c[1];
        temp += p_x[2] * p_c[2];
        temp += p_x[3] * p_c[3];
        temp += p_x[4] * p_c[4];
        temp += p_x[5] * p_c[5];
        temp += p_x[6] * p_c[6];
        temp += p_x[7] * p_c[7];
        temp += p_x[8] * p_c[8];
        temp += p_x[9] * p_c[9];
        temp += p_x[10] * p_c[10];
        temp += p_x[11] * p_c[11];
        temp += p_x[12] * p_c[12];
        temp += p_x[13] * p_c[13];
        temp += p_x[14] * p_c[14];
        temp += p_x[15] * p_c[15];

	p_c += 16;
	p_x += 16;
	n -= 16;
    }
    return (temp / (I4B_ECHO_CANCEL_W_DP /
		    I4B_ECHO_CANCEL_X_DP));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_add_product - add product
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_add_product(int32_t *p_w, const int16_t *p_x, const int32_t y1,
			    uint16_t n)
{
    while (n >= 16) {
	p_w[0] += y1 * p_x[0];
	p_w[1] += y1 * p_x[1];
	p_w[2] += y1 * p_x[2];
	p_w[3] += y1 * p_x[3];
	p_w[4] += y1 * p_x[4];
	p_w[5] += y1 * p_x[5];
	p_w[6] += y1 * p_x[6];
	p_w[7] += y1 * p_x[7];
	p_w[8] += y1 * p_x[8];
	p_w[9] += y1 * p_x[9];
	p_w[10] += y1 * p_x[10];
	p_w[11] += y1 * p_x[11];
	p_w[12] += y1 * p_x[12];
	p_w[13] += y1 * p_x[13];
	p_w[14] += y1 * p_x[14];
	p_w[15] += y1 * p_x[15];

	p_w += 16;
	p_x += 16;
	n -= 16;
    }
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_reset_adjust - reset all adjustments
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_reset_adjust(struct i4b_echo_cancel *ec)
{
    /* coeffs are bad, reset echo canceller */

    I4B_DBG(1, L1_EC_MSG, "resetting EC");

    bzero(ec->buf_W, sizeof(ec->buf_W));

    ec->stable_count = 0;
    ec->max_trained = 0;
    ec->max_coeff_set = 0;
    ec->coeffs_last_valid = 0;

    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_offset_adjust - adjust coefficients 
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_offset_adjust(struct i4b_echo_cancel *ec)
{
    int32_t temp0;
    int32_t temp1;

    uint16_t p;
    uint16_t n;

    if ((ec->offset_adjust < -16) ||
	(ec->offset_adjust > 16)) {

	  ec->offset_rd += ec->offset_adjust;
	  ec->offset_adjust = 0;

	  for (p = 0; p < I4B_ECHO_CANCEL_N_SUB; p++) {

	      temp0 = 0;
	      for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {
		  temp1 = ec->buf_XH[p][ec->offset_rd + n];
		  temp0 += (temp1 * temp1) / I4B_ECHO_CANCEL_N_TAPS;
	      }
	      ec->buf_PH[p] = temp0;
	  }

	  i4b_echo_cancel_reset_adjust(ec);

	  I4B_DBG(1, L1_EC_MSG, "adjust reset to %d", ec->offset_rd);

    } else if (ec->offset_adjust < 0) {

	for (p = 0; p < I4B_ECHO_CANCEL_N_SUB; p++) {

	    temp0 = ec->buf_XH[p][ec->offset_rd];

	    ec->buf_PH[p] += (temp0 * temp0) / I4B_ECHO_CANCEL_N_TAPS;

	    temp0 = ec->buf_XH[p][ec->offset_rd + I4B_ECHO_CANCEL_N_TAPS];

	    ec->buf_PH[p] -= (temp0 * temp0) / I4B_ECHO_CANCEL_N_TAPS;
	}

	for (p = 0; p < I4B_ECHO_CANCEL_W_SUB; p++) {
	    for (n = (I4B_ECHO_CANCEL_N_TAPS-1); n > 0; n--) {
	        ec->buf_W[p][n] = ec->buf_W[p][n-1];
	    }
	    ec->buf_W[p][n] = 0;
	}

	if (ec->coeffs_last_max_x < (I4B_ECHO_CANCEL_N_TAPS-1)) {
	    ec->coeffs_last_max_x ++;
	}

	ec->offset_rd --;
	ec->offset_adjust ++;

        I4B_DBG(1, L1_EC_MSG, "adjusting down to %d", ec->offset_rd);

    } else if (ec->offset_adjust > 0) {

	for (p = 0; p < I4B_ECHO_CANCEL_N_SUB; p++) {

	    temp0 = ec->buf_XH[p][ec->offset_rd];

	    ec->buf_PH[p] -= (temp0 * temp0) / I4B_ECHO_CANCEL_N_TAPS;

	    temp0 = ec->buf_XH[p][ec->offset_rd + I4B_ECHO_CANCEL_N_TAPS];

	    ec->buf_PH[p] += (temp0 * temp0) / I4B_ECHO_CANCEL_N_TAPS;
	}

	for (p = 0; p < I4B_ECHO_CANCEL_W_SUB; p++) {
	    for (n = 0; n < (I4B_ECHO_CANCEL_N_TAPS-1); n++) {
	        ec->buf_W[p][n] = ec->buf_W[p][n+1];
	    }
	    ec->buf_W[p][n] = 0;
	}

	if (ec->coeffs_last_max_x > 0) {
	    ec->coeffs_last_max_x --;
	}

	ec->offset_rd ++;
	ec->offset_adjust --;

        I4B_DBG(1, L1_EC_MSG, "adjusting up to %d", ec->offset_rd);
    }
    return;
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
i4b_echo_cancel_lms(struct i4b_echo_cancel *ec, int16_t y0)
{
#if (I4B_ECHO_CANCEL_N_SUB != 0x3)
#error "Please update i4b_echo_cancel_lms()!"
#endif

    int32_t y1;
    int32_t y2;

    u_int16_t p;

    int16_t lp_1kHz;
    int16_t lp_2kHz;
    int16_t temp;
    int16_t EH[I4B_ECHO_CANCEL_N_SUB];

    /* compute estimated echo */

    y1 = i4b_echo_cancel_inner_product
      (ec->buf_X0 + ec->offset_rd,
       ec->buf_W[0], I4B_ECHO_CANCEL_N_TAPS);

    /* store error */

    y0 -= i4b_echo_cancel_clamp(y1);

    ec->buf_E0[ec->offset_x] = y0;

    /* compute E-sub-band */

    lp_1kHz = i4b_echo_cancel_clamp(i4b_echo_cancel_inner_product_H0
	(ec->buf_E0 + ec->offset_x));

    /* compute E-sub-band */

    lp_2kHz = i4b_echo_cancel_clamp(i4b_echo_cancel_inner_product_H1
	(ec->buf_E0 + ec->offset_x));

    /* compute E-sub-band */

    temp = ec->buf_E0[ec->offset_x + (I4B_ECHO_CANCEL_K_TAPS/2)];

    /* store E-sub-bands */

    EH[0] = lp_1kHz; /* low-pass filter */
    EH[1] = lp_2kHz - lp_1kHz; /* band-pass filter */
    EH[2] = temp - lp_2kHz; /* high-pass filter */

    /* iterate over the sub-bands */

    for (p = 0; p < I4B_ECHO_CANCEL_N_SUB; p++) {

        /* get error */

        y1 = EH[p] * I4B_ECHO_CANCEL_X_DP;

	/* compute sum of XH squared (optimized) */

	y2 = ec->buf_XH[p][ec->offset_rd];

	ec->buf_PH[p] += (y2 * y2) / I4B_ECHO_CANCEL_N_TAPS;

	y2 = ec->buf_XH[p][ec->offset_rd + I4B_ECHO_CANCEL_N_TAPS];

	ec->buf_PH[p] -= (y2 * y2) / I4B_ECHO_CANCEL_N_TAPS;

	y2 = ec->buf_PH[p];

	/* update W1 */

	if (ec->coeffs_adapt) {

	    if (y2 > (64 * 64)) {

	        y1 /= y2;

		if (y1) {
		    i4b_echo_cancel_add_product(ec->buf_W[0], ec->buf_XH[p] + 
						ec->offset_rd, y1, I4B_ECHO_CANCEL_N_TAPS);
		}
	    }
	}
    }

    if (ec->offset_x == 0) {

        ec->offset_x = (I4B_ECHO_CANCEL_K_TAPS-1);

	bcopy(ec->buf_E0,
	      ec->buf_E0 + I4B_ECHO_CANCEL_K_TAPS,
	      sizeof(ec->buf_E0[0]) * I4B_ECHO_CANCEL_K_TAPS);

    } else {
        ec->offset_x --;
    }

    /*
     * Add a little comfort noise
     */
    y0 += (i4b_echo_cancel_noise(ec) / (1<<21));

    return y0;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_quick_train - quickly figure out the peak coefficient
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_quick_train(struct i4b_echo_cancel *ec, u_int16_t max_x)
{
    int64_t factor;

    int32_t pa;
    int32_t pb;

    u_int16_t n;

    if (ec->stable_count > 10) {
        return;
    }

    pa = 0;
    pb = 0;

    /* directly compute the amplitude factor of the echo */

    for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {

        pa += ((ec->buf_X0[ec->offset_x + 1 + n + max_x] *
		ec->buf_X0[ec->offset_x + 1 + n + max_x]) 
	       / I4B_ECHO_CANCEL_N_TAPS);

	pb += ((ec->buf_X0[ec->offset_x + 1 + n + max_x] *
		ec->buf_E0[ec->offset_x + 1 + n]) 
	       / I4B_ECHO_CANCEL_N_TAPS);
    }

    if ((pa > (64*64)) &&
	(pa > pb)) {

        factor = pb;
	factor *= I4B_ECHO_CANCEL_W_DP;
	factor /= pa;

	ec->buf_W[0][max_x] += factor;

	I4B_DBG(1, L1_EC_MSG, "forcing max = %d / 65536",
		ec->buf_W[0][max_x] / 
		(I4B_ECHO_CANCEL_W_DP / (1<<16)));

	/* zero all the other coefficients */

	for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {
	    if (n != max_x) {
	        ec->buf_W[0][n] = 0;
	    }
	}

	ec->max_trained = 1;
	ec->stable_count ++;
    }
    return;
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
		    int16_t y, int16_t z)
{
    const u_int32_t max = 256; /* this constant can be tuned */
    const u_int32_t diff_level = 64 * 64; /* this constant can be tuned */
    const u_int32_t echo_level = 32; /* this constant can be tuned */
    const u_int32_t lim_coeffs = (I4B_ECHO_CANCEL_W_DP / 
				  I4B_ECHO_CANCEL_N_TAPS);
    const u_int32_t min_level = (echo_level * echo_level * max);

    struct ec_stats stats;

    int16_t dx;
    int16_t x;

    x = ec->buf_X0[ec->offset_rd];

    ec->cur_power_tx += (x * x) / max;
    ec->cur_power_rx0 += (y * y) / max; /* after echo cancel */
    ec->cur_power_rx1 += (z * z) / max; /* before echo cancel */
    ec->cur_power_count ++;

    if (ec->cur_power_count >= max) {

        i4b_echo_cancel_offset_adjust(ec);

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

	stats = i4b_echo_cancel_stats(ec->buf_W[0]);

	/* check adapt boolean */

	if (ec->coeffs_adapt) {
	    if (ec->coeffs_wait) {
	        ec->coeffs_wait--;
		ec->coeffs_adapt = 0;
	    }
	} else {
	    if (ec->adapt_count == 0) {
	        ec->coeffs_wait = I4B_ECHO_CANCEL_ADAPT_HIST;
	    }
	}

	/* check for stable "max_x" */

	if (ec->max_coeff_set == 0) {
	    ec->max_coeff_set = 1;
	    ec->coeffs_last_max_x = stats.max_x;
	}

	dx = stats.max_x - ec->coeffs_last_max_x;

	if ((dx >= 4) || (dx <= -4)) {

	    i4b_echo_cancel_reset_adjust(ec);

	} else {

	    if (ec->coeffs_adapt) {

	        /* check the coefficients */

	        if (stats.sum_y >= lim_coeffs) {

		    /* make sure that the coefficients 
		     * do not diverge
		     */
		    I4B_DBG(1, L1_EC_MSG, "reverting bad current coeffs!");

		    bcopy(ec->buf_W[2],
			  ec->buf_W[0], sizeof(ec->buf_W[0]));

		    ec->coeffs_last_valid = 0;
		} else {
		    switch(ec->coeffs_last_valid) {
		    case 0:
		        bcopy(ec->buf_W[0],
			      ec->buf_W[1], sizeof(ec->buf_W[1]));
			ec->coeffs_last_valid = 1;
			break;
		    case 1:
		        bcopy(ec->buf_W[0],
			      ec->buf_W[3], sizeof(ec->buf_W[3]));
			ec->coeffs_last_valid = 2;
			break;
		    case 2:
		        bcopy(ec->buf_W[1],
			      ec->buf_W[2], sizeof(ec->buf_W[2]));
		        bcopy(ec->buf_W[0],
			      ec->buf_W[1], sizeof(ec->buf_W[1]));
			ec->coeffs_last_valid = 3;
			break;
		    case 3:
		        bcopy(ec->buf_W[3],
			      ec->buf_W[2], sizeof(ec->buf_W[2]));
		        bcopy(ec->buf_W[0],
			      ec->buf_W[3], sizeof(ec->buf_W[3]));
			ec->coeffs_last_valid = 2;
			break;
		    }

		    if (ec->stable_count < 8) {
		        ec->stable_count ++;
		    } else {
		        i4b_echo_cancel_quick_train(ec, stats.max_x);
		    }
		}

	    } else {

	        ec->coeffs_last_valid = 0;

	        bcopy(ec->buf_W[2],
		      ec->buf_W[0], sizeof(ec->buf_W[0]));
	    }
	}

	I4B_DBG(1, L1_EC_MSG, "rx/tx=%d tx/rx=%d rx0=%d rx1=%d "
		"%dr/%dt/%db/%da/%dT xm=%d ym=%d",
		ec->cur_power_rx0 / (ec->cur_power_tx ? ec->cur_power_tx : 1),
		ec->cur_power_tx / (ec->cur_power_rx0 ? ec->cur_power_rx0 : 1),
		ec->cur_power_rx0, ec->cur_power_rx1, 
		ec->rx_speaking, ec->tx_speaking, 
		ec->coeffs_bad, ec->coeffs_adapt,
		ec->max_trained,
		stats.max_x, (int32_t)(stats.max_y / 
				       (I4B_ECHO_CANCEL_W_DP / 
					(1 << 16))));

	ec->cur_power_count = 0;
	ec->cur_power_tx = 0;
	ec->cur_power_rx0 = 0;
	ec->cur_power_rx1 = 0;
    }

#if 0
    if (!(ec->max_trained)) {
        y = z;
    }
#endif

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
#if (I4B_ECHO_CANCEL_N_SUB != 0x3)
#error "Please update i4b_echo_cancel_feed()!"
#endif

    const int16_t *convert_fwd = 
      ((ec->is_ulaw) ? i4b_ulaw_to_signed : i4b_alaw_to_signed);

    int16_t lp_1kHz;
    int16_t lp_2kHz;
    int16_t temp;

    if (len) {
	ec->last_byte = ptr[len-1];
    }

    /* extra range check, just in case */

    if (ec->offset_wr > (I4B_ECHO_CANCEL_F_SIZE-1)) {
        ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
    }

    while(len) {

        temp = convert_fwd[*ptr];

	/* high pass sound to remove DC */

	temp = i4b_echo_cancel_hp_f1(ec, temp);

	/* store sample */

        ec->buf_X0[ec->offset_wr] = temp;

	/* compute X-sub-band */

	lp_1kHz = i4b_echo_cancel_clamp(i4b_echo_cancel_inner_product_H0
		(ec->buf_X0 + ec->offset_wr));

	/* compute X-sub-band */

	lp_2kHz = i4b_echo_cancel_clamp(i4b_echo_cancel_inner_product_H1
		(ec->buf_X0 + ec->offset_wr));

	/* compute X-sub-band */

	temp = ec->buf_X0[ec->offset_wr + (I4B_ECHO_CANCEL_K_TAPS/2)];

	/* store X-sub-bands */

	ec->buf_XH[0][ec->offset_wr] = lp_1kHz; /* low-pass filter */
	ec->buf_XH[1][ec->offset_wr] = lp_2kHz - lp_1kHz; /* band-pass filter */
	ec->buf_XH[2][ec->offset_wr] = temp - lp_2kHz; /* high-pass filter */

	if (ec->offset_wr == 0) {

	    /* update input offset */

            ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);

	    /* move data to new location */

	    bcopy(ec->buf_X0, ec->buf_X0 + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buf_X0[0]));

	    bcopy(ec->buf_XH[0], ec->buf_XH[0] + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buf_XH[0][0]));

	    bcopy(ec->buf_XH[1], ec->buf_XH[1] + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buf_XH[0][0]));

	    bcopy(ec->buf_XH[2], ec->buf_XH[2] + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buf_XH[0][0]));

	    /* update output offset */

	    ec->offset_rd += I4B_ECHO_CANCEL_F_SIZE;

	    if (ec->offset_rd > (I4B_ECHO_CANCEL_R_MAX-1)) {
	        ec->offset_rd = (I4B_ECHO_CANCEL_R_MAX-1);
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

    if (d_time > (I4B_ECHO_CANCEL_T_MAX-1)) {
        d_time = (I4B_ECHO_CANCEL_T_MAX-1);
    }

    /* extra range checks, just in case */

    if (ec->offset_wr > (I4B_ECHO_CANCEL_F_SIZE-1)) {
        ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
    }

    if (ec->offset_rd > (I4B_ECHO_CANCEL_R_MAX-1)) {
        ec->offset_rd = (I4B_ECHO_CANCEL_R_MAX-1);
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

	ec->offset_adjust = d_time - d_len;
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
    int16_t sample_y; /* sample from microphone */
    int16_t sample_z; /* sample from microphone (copy) */

    const int16_t *convert_fwd = 
      ((ec->is_ulaw) ? i4b_ulaw_to_signed : i4b_alaw_to_signed);

    i4b_convert_rev_t *convert_rev = 
      ((ec->is_ulaw) ? i4b_signed_to_ulaw : i4b_signed_to_alaw);

    while(read_len) {

	sample_y = convert_fwd[*read_ptr];

	/* high pass sound to remove DC */

	sample_y = i4b_echo_cancel_hp_f2(ec, sample_y);
	sample_z = sample_y;

	/* filter */

	sample_y = i4b_echo_cancel_lms(ec, sample_y);
	sample_y = i4b_echo_cancel_pwr(ec, sample_y, sample_z);

	/* update */

	read_ptr[0] = convert_rev(sample_y);

        read_len--;
	read_ptr++;

	/* increment read pointer */

        if (ec->offset_rd <= ec->offset_wr) {
	    I4B_DBG(1, L1_EC_MSG, "repeating last byte");
	    break;
	} else {
	    ec->offset_rd--;
	}
    }
    return;
}
