/*-
 * Copyright (c) 2006-2007 Hans Petter Selasky. All rights reserved.
 * Copyright (c) 2005 Jon Recker. All Rights Reserved. *
 * Copyright (c) 2005 Ken Cooke. All Rights Reserved. *
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
 * i4b_echo_cancel.c - An integer, Fast Fourier Transform, FFT,
 *                     Normalized Least Mean Square, 256-TAP,
 *                     Echo Canceller, EC.
 *
 * * The FFT part was inspired by code written by Jon Recker and Ken Cooke.
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

#define I32(x) ((int32_t)(x))
#define L64(x) ((int64_t)(int32_t)(x))
#define U64(x) ((uint64_t)(x))

#define SQRT1_2 0x2d413ccd /* sqrt(0.5) * (1 << 30) */
#define MUL_SHIFT30(a,b) I32(U64(L64(a) * L64(b)) >> 30)
#define MUTE_COUNT 4000

struct i4b_echo_cancel_stats {
   uint64_t sum_y;
    int32_t max_y;
    int16_t max_x;
};

static void
i4b_echo_cancel_fft(struct i4b_complex *data, uint8_t inverse);

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_init - initialize echo canceller
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_init(struct i4b_echo_cancel *ec, 
		     uint16_t pre_delay,
		     uint8_t sub_bprot)
{
    bzero(ec, sizeof(*ec));

    ec->last_byte = 0xFF;

    ec->noise_rem = 1;

    ec->pre_delay = pre_delay;

    ec->is_ulaw = (sub_bprot != BSUBPROT_G711_ALAW);

    ec->offset_x = (I4B_ECHO_CANCEL_N_TAPS-1);

    ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);

    ec->offset_rd = (I4B_ECHO_CANCEL_R_SIZE-1);

    ec->mute_count = MUTE_COUNT;

    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_noise - a perceptual white noise generator
 *---------------------------------------------------------------------------*/
static __inline int32_t
i4b_echo_cancel_noise(struct i4b_echo_cancel *ec)
{
    uint32_t temp;
    const uint32_t prime = 0xFFFF1D;

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
    ec->low_pass_1 += ((I32(in) * (1<<8)) - (ec->low_pass_1 / (1<<8)));
    return i4b_subtract_safe(in, (ec->low_pass_1 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_hp_f2 - IIR filter number 2, high pass to remove DC
 *---------------------------------------------------------------------------*/
static __inline int16_t
i4b_echo_cancel_hp_f2(struct i4b_echo_cancel *ec, int16_t in)
{
    ec->low_pass_2 += ((I32(in) * (1<<8)) - (ec->low_pass_2 / (1<<8)));
    return i4b_subtract_safe(in, (ec->low_pass_2 / (1<<16)));
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_abs - compute the absolute value of the argument
 *---------------------------------------------------------------------------*/
static __inline int32_t
i4b_echo_cancel_abs(int32_t temp)
{
    if (temp < 0) {
        if (temp == -0x80000000) {
	    /* avoid overflow */
	    temp = 0x7FFFFFFF;
	} else {
	    temp = -temp;
	}
    }
    return temp;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_sqrt64 - compute the square root of the argument
 *---------------------------------------------------------------------------*/
static uint32_t
i4b_echo_cancel_sqrt64(uint64_t a)
{
    uint64_t b = 0x4000000000000000ULL;

    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x7000000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x3000000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c00000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc00000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x700000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x300000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c0000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc0000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x70000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x30000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x7000000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x3000000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c00000000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc00000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x700000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x300000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c0000000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc0000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x70000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x30000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c000000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x7000000000ULL;
    } else {
        b >>= 1;
        b ^= 0x3000000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c00000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc00000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x700000000ULL;
    } else {
        b >>= 1;
        b ^= 0x300000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c0000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc0000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x70000000ULL;
    } else {
        b >>= 1;
        b ^= 0x30000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c000000ULL;
    } else {
        b >>= 1;
        b ^= 0xc000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x7000000ULL;
    } else {
        b >>= 1;
        b ^= 0x3000000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c00000ULL;
    } else {
        b >>= 1;
        b ^= 0xc00000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x700000ULL;
    } else {
        b >>= 1;
        b ^= 0x300000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c0000ULL;
    } else {
        b >>= 1;
        b ^= 0xc0000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x70000ULL;
    } else {
        b >>= 1;
        b ^= 0x30000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c000ULL;
    } else {
        b >>= 1;
        b ^= 0xc000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x7000ULL;
    } else {
        b >>= 1;
        b ^= 0x3000ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c00ULL;
    } else {
        b >>= 1;
        b ^= 0xc00ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x700ULL;
    } else {
        b >>= 1;
        b ^= 0x300ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1c0ULL;
    } else {
        b >>= 1;
        b ^= 0xc0ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x70ULL;
    } else {
        b >>= 1;
        b ^= 0x30ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1cULL;
    } else {
        b >>= 1;
        b ^= 0xcULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x7ULL;
    } else {
        b >>= 1;
        b ^= 0x3ULL;
    }
    if (a >= b) {
        a -= b;
        b >>= 1;
        b ^= 0x1ULL;
    } else {
        b >>= 1;
    }
    return b;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_stats - compute coefficient statistics
 *---------------------------------------------------------------------------*/
static struct i4b_echo_cancel_stats
i4b_echo_cancel_stats(const int32_t *data)
{
    struct i4b_echo_cancel_stats stats;
    int32_t temp;
    uint16_t n;

    stats.max_y = 0;
    stats.max_x = 0;
    stats.sum_y = 0;

    for (n = 0; n < I4B_ECHO_CANCEL_N_TAPS; n++) {

	temp = *(data + n);
	temp = i4b_echo_cancel_abs(temp);

	stats.sum_y += temp;

	if (temp > stats.max_y) {
	    stats.max_y = temp;
	    stats.max_x = n;
	}
    }
    return stats;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_coeffs_reset - reset all coefficients
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_coeffs_reset(struct i4b_echo_cancel *ec)
{
    I4B_DBG(1, L1_EC_MSG, "resetting EC");

    /* coeffs are bad, reset echo canceller */

    bzero(ec->buf_HR, sizeof(ec->buf_HR));
    bzero(ec->buf_ET, sizeof(ec->buf_ET));
    bzero(ec->buf_ED, sizeof(ec->buf_ED));

    ec->adapt_step = 0;
    ec->coeffs_state = 0;
    ec->coeffs_adapt = 0;
    ec->echo_level_max = 0;
    ec->offset_x = I4B_ECHO_CANCEL_N_TAPS-1;
    ec->mute_count = MUTE_COUNT;
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_offset_adjust - adjust coefficients 
 *---------------------------------------------------------------------------*/
static uint8_t
i4b_echo_cancel_offset_adjust(struct i4b_echo_cancel *ec)
{
    uint16_t p;
    uint16_t n;

    uint8_t retval;

    retval = 0;

    if ((ec->offset_adjust < -16) ||
	(ec->offset_adjust > 16)) {

	ec->offset_rd += ec->offset_adjust;

	/* range check */

	if ((ec->offset_rd > (I4B_ECHO_CANCEL_R_SIZE-1)) ||
	    (ec->offset_rd < ec->offset_wr)) {

	    I4B_DBG(1, L1_EC_MSG, "cannot do adjustment of %d samples",
		    (int32_t)(ec->offset_adjust));

	    /* revert */

	    ec->offset_rd -= ec->offset_adjust;

	} else {

	    /* clear adjust offset */

	    ec->offset_adjust = 0;

	    /* reset adaption */

	    retval = 1;
	}

    } else if ((ec->offset_adjust < 0) &&
	       (ec->offset_rd > ec->offset_wr)) {

	ec->offset_rd --;
	ec->offset_adjust ++;

	for (p = 0; p < I4B_ECHO_CANCEL_W_SUB; p++) {
	    for (n = (I4B_ECHO_CANCEL_N_TAPS-1); n > 0; n--) {
	        ec->buf_HR[p][n] = ec->buf_HR[p][n-1];
	    }
	    ec->buf_HR[p][n] = 0;
	}

	I4B_DBG(1, L1_EC_MSG, "adjusting EC down");

    } else if ((ec->offset_adjust > 0) &&
	       (ec->offset_rd < (I4B_ECHO_CANCEL_R_SIZE-1))) {

	ec->offset_rd ++;
	ec->offset_adjust --;

	for (p = 0; p < I4B_ECHO_CANCEL_W_SUB; p++) {
	    for (n = 0; n < (I4B_ECHO_CANCEL_N_TAPS-1); n++) {
	        ec->buf_HR[p][n] = ec->buf_HR[p][n+1];
	    }
	    ec->buf_HR[p][n] = 0;
	}

	I4B_DBG(1, L1_EC_MSG, "adjusting EC up");
    }

    return retval;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_step_divide
 *---------------------------------------------------------------------------*/
static int32_t
i4b_echo_cancel_step_divide(int64_t t, int64_t u, const uint8_t adapt_step)
{
    enum {
      STEPF = (4),
      STEP0 = (STEPF * 64), /* start slow */
      STEP1 = (STEPF * 8),
      STEP2 = (STEPF * 2),
      STEP3 = (STEPF * 4),
      STEP4 = (STEPF * 8),
      STEP5 = (STEPF * 16),
      MAX = (I4B_ECHO_CANCEL_HR_DP *
	     I4B_ECHO_CANCEL_N_COMPLEX) / 8,
    };

    switch(adapt_step) {
    case 0:
        t /= STEP0;
	break;
    case 1:
        t /= STEP1;
	break;
    case 2:
        t /= STEP2;
	break;
    case 3:
        t /= STEP3;
	break;
    case 4:
        t /= STEP4;
	break;
    default:
        t /= STEP5;
	break;
    }

    if (t < 0) {
        t = -t;
	u = -u;
    }

    t *= I4B_ECHO_CANCEL_N_COMPLEX;
    u /= I4B_ECHO_CANCEL_N_COMPLEX;

    if (u) {
      t /= u;

      if (t > MAX) {
	t = MAX;
      } else if (t < -MAX) {
	t = -MAX;
      }
    } else {
      t = 0;
    }

    return I32(t);
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_compute_power - update power averages on speaker signal
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_compute_power(struct i4b_echo_cancel *ec)
{
    enum {
      MIN = (16 * I4B_ECHO_CANCEL_N_COMPLEX * 
	     I4B_ECHO_CANCEL_PRE_M),
    };

    uint64_t u;

    int32_t r;
    int32_t dx;
    int32_t dy;

    uint16_t i;
    uint16_t j;

    uint16_t cmin = 0;

    for (i = 1; i < I4B_ECHO_CANCEL_N_TAPS; i++) {
	
        j = I4B_ECHO_CANCEL_N_COMPLEX-i;

	dx = (ec->buf_XD[i].x + ec->buf_XD[j].x);
	dy = (ec->buf_XD[i].y - ec->buf_XD[j].y);

	u = ((L64(dx) * L64(dx)) + (L64(dy) * L64(dy)));

	r = i4b_echo_cancel_sqrt64(u);

	ec->buf_AA[i] += (r - ec->buf_AA[i]) / 4;

	if (ec->buf_AA[i] < MIN) {
	    ec->buf_AA[i] = 0;
	    ec->buf_AP[i] = 0;
	    cmin ++;
	} else {
	    ec->buf_AP[i] = L64(ec->buf_AA[i]) * L64(r);
	}
    }

    I4B_DBG(1, L1_EC_MSG, "cmin=%d", cmin);
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_convolute - complex multiplication that results in
 *			       convolution in the time domain
 * Input:
 *  ec->buf_XD
 *  ec->buf_ED
 *
 * Output:
 *  ec->buf_ED = ec->buf_ED * ec->buf_XD
 *---------------------------------------------------------------------------*/
static void
i4b_echo_cancel_convolute(struct i4b_echo_cancel *ec, const uint8_t divide)
{
    enum {
        MAX = I4B_ECHO_CANCEL_HR_DP * I4B_ECHO_CANCEL_N_COMPLEX,
    };

    int64_t t;
    int64_t u;

    int32_t dx;
    int32_t dy;
    int32_t ex;
    int32_t ey;

    uint16_t i;
    uint16_t j;

    u = 0;

    for (i = 1; i < I4B_ECHO_CANCEL_N_TAPS; i++) {

	j = I4B_ECHO_CANCEL_N_COMPLEX-i;

	dx = (ec->buf_XD[i].x + ec->buf_XD[j].x);
	dy = (ec->buf_XD[i].y - ec->buf_XD[j].y);

	ex = (ec->buf_ED[i].x + ec->buf_ED[j].x);
	ey = (ec->buf_ED[i].y - ec->buf_ED[j].y);

	if (divide) {
	
	    u = ec->buf_AP[i];

	    u /= (I4B_ECHO_CANCEL_PRE_M *
		  I4B_ECHO_CANCEL_PRE_M * 2);

	    if (u == 0) {
	        /* too little signal */
	        ec->buf_ED[i].x = 0;
		ec->buf_ED[j].x = 0;
		ec->buf_ED[i].y = 0;
		ec->buf_ED[j].y = 0;
		continue;
	    }
	}

	t = ((L64(dx) * L64(ex)) - (L64(dy) * L64(ey)));

	if (divide) {
	    t = i4b_echo_cancel_step_divide(t,u,ec->adapt_step);
	} else {
	    t /= (I4B_ECHO_CANCEL_HR_DP * 4);
	}

	ec->buf_ED[i].x = I32(t);
	ec->buf_ED[j].x = I32(t);

	t = ((L64(dy) * L64(ex)) + (L64(dx) * L64(ey)));

	if (divide) {
	    t = i4b_echo_cancel_step_divide(t,u,ec->adapt_step);
	} else {
	    t /= (I4B_ECHO_CANCEL_HR_DP * 4);
	}
	ec->buf_ED[i].y = I32(t);
	ec->buf_ED[j].y = -I32(t);
    }

    /* handle phase-less components */

    if (divide) {

        /* 0Hz - force to zero */

        ec->buf_ED[0].x = 0;
	ec->buf_ED[0].y = 0;

	/* 4kHz - force to zero */

	ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS].x = 0;
	ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS].y = 0;

    } else {

        /* 0Hz */

	dx = ec->buf_XD[0].x;
	ex = ec->buf_ED[0].x;

	t = (L64(dx) * L64(ex));

	t /= I4B_ECHO_CANCEL_HR_DP;

	ec->buf_ED[0].x = I32(t);
	ec->buf_ED[0].y = 0;

	/* 4kHz */

	dx = ec->buf_XD[I4B_ECHO_CANCEL_N_TAPS].x;
	ex = ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS].x;

	t = (L64(dx) * L64(ex));

	t /= I4B_ECHO_CANCEL_HR_DP;

	ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS].x = I32(t);
	ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS].y = 0;
    }
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_lms_pwr - an implementation of a self adapting FIR filter
 *
 * inputs:
 *   y0: sample from local microphone with echo from local speaker
 *
 * outputs:
 *   sample from local microphone without echo from local speaker
 *---------------------------------------------------------------------------*/
static int16_t
i4b_echo_cancel_lms_pwr(struct i4b_echo_cancel *ec, int32_t y0)
{
    enum {
        MAX_Y = (I4B_ECHO_CANCEL_PRE_M * I4B_ECHO_CANCEL_SAMPLE_MAX)
    };

    struct i4b_echo_cancel_stats stats;

    struct i4b_complex *pa;
    struct i4b_complex *pa_end;

    int16_t *pb;
    int32_t *pc;

    int32_t tx;
    int32_t ty;

    uint32_t sum_speaker;

    int32_t y0_copy;

    uint16_t x;
    uint16_t max_samples;
    uint16_t max_old_echo;
    uint16_t max_speaker;

    uint8_t coeffs_adapt;

    /*
     * Buffer ordering overview. In the
     * text below, time goes forward when
     * the number following the letter
     * increments:
     * ==================================
     *
     * buf_XD:
     * X0 X1 X2 X3 | X-4 X-3 X-2 X-1
     *
     * buf_X0:
     * X3 X2 X1 X0 X-1 X-2 X-3 X-4
     *
     * buf_HR:
     * H0 H1 H2 H3
     *
     * buf_ED:
     * Y3 Y2 Y1 Y0
     *
     */

    /*
     * Convolution overview:
     * =====================
     *
     *  E3 H0 | X00 X01 X02 X03 | X-4 X-3 X-2 X-1 |
     *  E2 H1 | X-1 X00 X01 X02 | X03 X-4 X-3 X-2 |
     *  E1 H2 | X-2 X-1 X00 X01 | X02 X03 X-4 X-3 |
     * +E0 H3 | X-3 X-2 X-1 X00 | X01 X02 X03 X-4 |
     * -------+-----------------+-----------------+-
     * =      | Y00 Y01 Y02 Y03 |   <discarded>   |
     * =      | H03 H02 H01 H00 |   <discarded>   |
     *
     *
     * Updates to "Hxx" are approximately computed like:
     * REVERSE(REVERSE(E) * X)
     */

    /* make a copy of "y0" */

    y0_copy = y0;

    /* pre amplify echo */

    y0 *= I4B_ECHO_CANCEL_PRE_M;

    /* compute maximum echo */

    if (y0 < 0) {
      if (ec->echo_level_max < (-y0)) {
	  ec->echo_level_max = (-y0);
      }
    } else {
      if (ec->echo_level_max < y0) {
	  ec->echo_level_max = y0;
      }
    }

    /* subtract out the computed echo */

    y0 -= ec->buf_ED[ec->offset_x].x;

    /* range check (important) */

    if (y0 > MAX_Y) {
        y0 = MAX_Y;
    } else if (y0 < -MAX_Y) {
        y0 = -MAX_Y;
    }

    /* add a little comfort noise
     * which is within the margin of
     * the range check above:
     */
    y0 += (i4b_echo_cancel_noise(ec) / 
	   ((1<<21) / I4B_ECHO_CANCEL_PRE_M));

    /* store echo */

    ec->buf_ED[ec->offset_x].x = y0;

    /* compute final output sample */

    if (ec->adapt_step > 0) {
        /* our dirty mute trick */
        if (ec->mute_count && ec->coeffs_adapt) {
	    y0 /= 64;
	}
        y0 /= I4B_ECHO_CANCEL_PRE_M;
    } else {
        /* our dirty mute trick */
        if (ec->mute_count && ec->coeffs_adapt) {
	    y0_copy /= 64;
	}
        y0 = y0_copy;
    }

    /* decrement "ec->offset_x" */

    if (ec->offset_x == 0) {

	/* if there is no speaker data,
	 * there is nothing to do!
	 */
	if (ec->offset_rd <= ec->offset_wr) {
	    I4B_DBG(1, L1_EC_MSG, "first check out of data");
	    goto no_data;
	}

	/* compute the maximum speaker level */

        tx = 0;
	sum_speaker = 0;

	for (x = 0; x < I4B_ECHO_CANCEL_N_TAPS; x++) {
	    ty = ec->buf_X0[ec->offset_rd + x];
	    if (ty < 0) {
	        ty = -ty;
	    }
	    if (ty > tx) {
	        tx = ty;
	    }
	    sum_speaker += ty;
	}

	max_speaker = tx;

	/* de-amplify maximum echo level */

	ec->echo_level_max /= I4B_ECHO_CANCEL_PRE_M;

	/* make some decicions */

	coeffs_adapt = ((sum_speaker > (64 * I4B_ECHO_CANCEL_N_TAPS)) &&
			(max_speaker > 128) &&
			(ec->echo_level_max < (max_speaker/2)));

	/* store adaption decision */

	if (coeffs_adapt) {
	    if (ec->coeffs_adapt < 1) {
	        ec->coeffs_adapt ++;
		coeffs_adapt = 0;
	    }
	} else {
	    ec->coeffs_adapt = 0;
	}

	/* reset max echo level */

	ec->echo_level_max = 0;

	/* make a copy of the echo first,
	 * we need it later, and at the same
	 * time clear the unused part of 
	 * the echo buffer:
	 */
	pa = ec->buf_ED;
	pa_end = ec->buf_ED + I4B_ECHO_CANCEL_N_TAPS;
	pc = ec->buf_ET;

	while (pa != pa_end) {
	    (*pc) = pa->x;
	    pa->y = 0;
	    pa += I4B_ECHO_CANCEL_N_TAPS;
	    pa->x = 0;
	    pa->y = 0;
	    pa -= (I4B_ECHO_CANCEL_N_TAPS-1);
	    pc += 1;
	}

	/* transform error into "sine/cosine" domain */

	i4b_echo_cancel_fft(ec->buf_ED, 0);

	/* keep buf_AP[] up to date */

	i4b_echo_cancel_compute_power(ec);

	/* update filter, buf_HR[0], based on echo */

	if (coeffs_adapt) {

	    /* do convolution: buf_ED = buf_XD * buf_ED / buf_AP */

	    i4b_echo_cancel_convolute(ec, 1);

	    /* transform error back into "time" domain */

	    i4b_echo_cancel_fft(ec->buf_ED, 1);

	    /* update buf_HR[] */

	    pa = ec->buf_ED + (I4B_ECHO_CANCEL_N_TAPS-1);
	    pa_end = ec->buf_ED -1;
	    pc = ec->buf_HR[0];

	    while (pa != pa_end) {
	      (*pc) += pa->x / I4B_ECHO_CANCEL_N_COMPLEX;
	      pc ++;
	      pa --;
	    }
	}

	/* adjust offset [if any], before computing 
	 * the maximum read length and statistics:
	 */
	if (i4b_echo_cancel_offset_adjust(ec)) {
	    I4B_DBG(1, L1_EC_MSG, "offset adjust -> reset");
	    goto no_data;
	}

	stats = i4b_echo_cancel_stats(ec->buf_HR[0]);

	if (coeffs_adapt) {

	    /* compute adaption step size */

	    if ((stats.sum_y / 64) >= stats.max_y)
	      ec->adapt_step = 0;
	    else if ((stats.sum_y / 32) >= stats.max_y)
	      ec->adapt_step = 1;
	    else  if ((stats.sum_y / 8) >= stats.max_y)
	      ec->adapt_step = 2;
	    else  if ((stats.sum_y / 4) >= stats.max_y)
	      ec->adapt_step = 3;
	    else  if ((stats.sum_y / 3) >= stats.max_y)
	      ec->adapt_step = 4;
	    else
	      ec->adapt_step = 5;

	    /* check the coefficients */

	    if (stats.sum_y >= (I4B_ECHO_CANCEL_HR_DP * 
				I4B_ECHO_CANCEL_N_COMPLEX * 2)) {

	        /* make sure that the coefficients 
		 * do not diverge
		 */
	        I4B_DBG(1, L1_EC_MSG, "reverting bad current coeffs!");

		bcopy(ec->buf_HR[2],
		      ec->buf_HR[0], sizeof(ec->buf_HR[0]));

		ec->coeffs_state = 0;

	    } else {

	        switch(ec->coeffs_state) {
		case 0:
		    bcopy(ec->buf_HR[0],
			  ec->buf_HR[1], sizeof(ec->buf_HR[1]));
		    ec->coeffs_state = 1;
		    break;
		case 1:
		    bcopy(ec->buf_HR[0],
			  ec->buf_HR[3], sizeof(ec->buf_HR[3]));
		    ec->coeffs_state = 2;
		    break;
		case 2:
		    bcopy(ec->buf_HR[1],
			  ec->buf_HR[2], sizeof(ec->buf_HR[2]));
		    bcopy(ec->buf_HR[0],
			  ec->buf_HR[1], sizeof(ec->buf_HR[1]));
		    ec->coeffs_state = 3;
		    break;
		case 3:
		    bcopy(ec->buf_HR[3],
			  ec->buf_HR[2], sizeof(ec->buf_HR[2]));
		    bcopy(ec->buf_HR[0],
			  ec->buf_HR[3], sizeof(ec->buf_HR[3]));
		    ec->coeffs_state = 2;
		    break;
		}
	    }
	} else {

	    ec->coeffs_state = 0;

	    bcopy(ec->buf_HR[2],
		  ec->buf_HR[0], sizeof(ec->buf_HR[0]));
	}

	/* copy impulse response to temporary buffer */

	pa = ec->buf_ED;
	pa_end = ec->buf_ED + I4B_ECHO_CANCEL_N_TAPS;
	pc = ec->buf_HR[0];

	while (pa != pa_end) {
	    pa->x = (*pc) / I4B_ECHO_CANCEL_N_COMPLEX;
	    pa->y = 0;
	    pa += I4B_ECHO_CANCEL_N_TAPS;
	    pa->x = 0;
	    pa->y = 0;
	    pa -= (I4B_ECHO_CANCEL_N_TAPS-1);
	    pc += 1;
	}

	/* transform impulse response into "sine/cosine" domain */

	i4b_echo_cancel_fft(ec->buf_ED, 0);

	/* compute maximum read length */

	max_samples = (ec->offset_rd - ec->offset_wr);

	/* range check */

	if (max_samples > I4B_ECHO_CANCEL_N_TAPS) {
	    max_samples = I4B_ECHO_CANCEL_N_TAPS;
	}

	if (max_samples == 0) {
	    I4B_DBG(1, L1_EC_MSG, "second check out of data");
	    goto no_data;
	}

	/* store new offsets */

	ec->offset_x = max_samples-1;

	if (coeffs_adapt) {

	    /* update mute count */

	    if (ec->mute_count >= max_samples) {
	        ec->mute_count -= max_samples;
	    } else {
	        ec->mute_count = 0;
	    }
	}

	/* print some debugging messages */

	I4B_DBG(1, L1_EC_MSG, "tx=0x%04x mc=0x%x xm=0x%02x ym=%d/%d "
		"block=%d astep=%x adapt=%d f=0x%02x",
		(uint32_t)max_speaker, (uint32_t)(ec->mute_count),
		(int32_t)(stats.max_x), (int32_t)(stats.max_y), 
		(int32_t)(I4B_ECHO_CANCEL_HR_DP * I4B_ECHO_CANCEL_N_COMPLEX), 
		(int32_t)max_samples, (uint32_t)(ec->adapt_step),
		coeffs_adapt, (int32_t)((0x10 * stats.sum_y) / 
		(stats.max_y ? stats.max_y : 1)));

	/* load buf_XD[] */

	pa = ec->buf_XD;
	pa_end = ec->buf_XD + I4B_ECHO_CANCEL_N_TAPS;
	pb = (ec->buf_X0 + ec->offset_rd +
	      I4B_ECHO_CANCEL_N_TAPS - max_samples);

	while (pa != pa_end) {

	    pa->x = (*pb) * I4B_ECHO_CANCEL_PRE_M;
	    pa->y = 0;
	    pa += I4B_ECHO_CANCEL_N_TAPS;
	    pb += I4B_ECHO_CANCEL_N_TAPS;

	    pa->x = (*pb) * I4B_ECHO_CANCEL_PRE_M;
	    pa->y = 0;
	    pa -= (I4B_ECHO_CANCEL_N_TAPS-1);
	    pb -= (I4B_ECHO_CANCEL_N_TAPS+1);
	}

	/* transform local speaker data into the "sine/cosine" domain */

	i4b_echo_cancel_fft(ec->buf_XD, 0);

	/* do convolution: buf_ED = buf_XD * buf_ED (buf_HR) */

	i4b_echo_cancel_convolute(ec, 0);

	/* transform estimated echo back into the "time" domain */

	i4b_echo_cancel_fft(ec->buf_ED, 3);

	/* copy in old echo */

	max_old_echo = (I4B_ECHO_CANCEL_N_TAPS - max_samples);

	for (x = 0; x < max_old_echo; x++) {
	    ec->buf_ED[x].x = ec->buf_ET[max_old_echo-1-x];
	}

	/* reverse the order of the echo samples */

	for (x = 0; x < (I4B_ECHO_CANCEL_N_TAPS/2); x++) {
	    tx = ec->buf_ED[x].x;
	    ec->buf_ED[x].x = ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS-1-x].x;
	    ec->buf_ED[I4B_ECHO_CANCEL_N_TAPS-1-x].x = tx;
	}

    } else {
        ec->offset_x --;
    }

 done:
    return y0;

 no_data:
    i4b_echo_cancel_coeffs_reset(ec);
    goto done;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_update_feeder - set feed state for echo canceller
 *
 * input:
 *   tx_time: time in units of 125us, when the TX-buffer was empty
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_update_feeder(struct i4b_echo_cancel *ec,
			      uint16_t tx_time)
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
		     uint8_t *ptr, uint16_t len)
{
    const int16_t *convert_fwd = 
      ((ec->is_ulaw) ? i4b_ulaw_to_signed : i4b_alaw_to_signed);

    int16_t temp;

    if (len) {
	ec->last_byte = ptr[len-1];
    }

    /* extra range check, just in case */

    if (ec->offset_wr > (I4B_ECHO_CANCEL_F_SIZE-1)) {
        ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);
    }

    while(len--) {

        temp = convert_fwd[*ptr];
	ptr++;

	/* high pass sound to remove DC */

	temp = i4b_echo_cancel_hp_f1(ec, temp);

	/* store sample */

        ec->buf_X0[ec->offset_wr] = temp;

	if (ec->offset_wr == 0) {

	    /* update input offset */

            ec->offset_wr = (I4B_ECHO_CANCEL_F_SIZE-1);

	    /* move data to new location */

	    bcopy(ec->buf_X0, ec->buf_X0 + I4B_ECHO_CANCEL_F_SIZE,
		  I4B_ECHO_CANCEL_F_SIZE * sizeof(ec->buf_X0[0]));

	    /* update output offset */

	    ec->offset_rd += I4B_ECHO_CANCEL_F_SIZE;

	    if (ec->offset_rd > (I4B_ECHO_CANCEL_R_SIZE-1)) {
	        ec->offset_rd = (I4B_ECHO_CANCEL_R_SIZE-1);
	    }

	} else {
            ec->offset_wr --;
	}
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
i4b_echo_cancel_update_merger(struct i4b_echo_cancel *ec, uint16_t rx_time)
{
    uint16_t d_time;
    uint16_t d_len;

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

    if (ec->offset_rd > (I4B_ECHO_CANCEL_R_SIZE-1)) {
        ec->offset_rd = (I4B_ECHO_CANCEL_R_SIZE-1);
    }

    if (ec->offset_rd < ec->offset_wr) {
        ec->offset_rd = ec->offset_wr;
    }

    /* compute the current read length */

    d_len = (ec->offset_rd - ec->offset_wr);

    /* check that time and length matches */

    if (((d_time > 4) && 
	 (d_len < (d_time-4))) ||
	(d_len > (d_time+4))) {

        I4B_DBG(1, L1_EC_MSG, "Adjusting pre-delay buffer "
		"from %d to %d bytes!", d_len, d_time);

	/* adjust */

	ec->offset_adjust = (d_time - d_len);
    }
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_merge - remove echo from local microphone sound
 *
 * input:
 *   read_ptr: pointer to input samples, bitreversed A-law or u-law
 *   read_len: length of data
 *---------------------------------------------------------------------------*/
void
i4b_echo_cancel_merge(struct i4b_echo_cancel *ec, 
		      uint8_t *read_ptr, uint16_t read_len)
{
    int16_t sample_y; /* sample from local microphone */

    const int16_t *convert_fwd = 
      ((ec->is_ulaw) ? i4b_ulaw_to_signed : i4b_alaw_to_signed);

    i4b_convert_rev_t *convert_rev = 
      ((ec->is_ulaw) ? i4b_signed_to_ulaw : i4b_signed_to_alaw);

    while(read_len--) {

	sample_y = convert_fwd[*read_ptr];

	/* high pass sound to remove DC */

	sample_y = i4b_echo_cancel_hp_f2(ec, sample_y);

	/* filter */

	sample_y = i4b_echo_cancel_lms_pwr(ec, sample_y);

	/* update */

	*read_ptr = convert_rev(sample_y);
	read_ptr++;

	/* decrement read pointer */

        if (ec->offset_rd <= ec->offset_wr) {
	    I4B_DBG(1, L1_EC_MSG, "repeating last byte");
	    break;
	} else {
	    ec->offset_rd--;
	}
    }
    return;
}

/*---------------------------------------------------------------------------*
 * i4b_echo_cancel_fft - I4B echo cancel Fast Fourier Transform
 *
 * NOTE: this code has been generated
 * NOTE: do not edit
 *---------------------------------------------------------------------------*/
#if (!(I4B_ECHO_CANCEL_P_COMPLEX & 1))
static void
i4b_echo_cancel_radix4_fp(struct i4b_complex *data, uint16_t n)
{
    struct i4b_complex *data_end;

    int32_t ar, ai, br, bi, cr, ci, dr, di;

    data_end = data + n;

    while (data != data_end) {

        ar = data[0].x + data[1].x;
	br = data[0].x - data[1].x;
	ai = data[0].y + data[1].y;
	bi = data[0].y - data[1].y;
	cr = data[2].x + data[3].x;
	dr = data[2].x - data[3].x;
	ci = data[2].y + data[3].y;
	di = data[2].y - data[3].y;

	data[0].x = (ar + cr);
	data[2].x = (ar - cr);
	data[0].y = (ai + ci);
	data[2].y = (ai - ci);
	data[1].x = (br + di);
	data[3].x = (br - di);
	data[1].y = (bi - dr);
	data[3].y = (bi + dr);

	data += 4;
    }
    return;
}
#endif

#if (I4B_ECHO_CANCEL_P_COMPLEX & 1)
static void
i4b_echo_cancel_radix8_fp(struct i4b_complex *data, uint16_t n)
{
    struct i4b_complex *data_end;

    int32_t ar, ai, br, bi, cr, ci, dr, di;
    int32_t sr, si, tr, ti, ur, ui, vr, vi;
    int32_t wr, wi, xr, xi, yr, yi, zr, zi;

    data_end = data + n;

    while (data != data_end) {

	ar = data[0].x + data[1].x;
	br = data[0].x - data[1].x;
	ai = data[0].y + data[1].y;
	bi = data[0].y - data[1].y;
	cr = data[2].x + data[3].x;
	dr = data[2].x - data[3].x;
	ci = data[2].y + data[3].y;
	di = data[2].y - data[3].y;

	sr = (ar + cr);
	ur = (ar - cr);
	si = (ai + ci);
	ui = (ai - ci);
	tr = (br - di);
	vr = (br + di);
	ti = (bi + dr);
	vi = (bi - dr);

	ar = data[4].x + data[5].x;
	br = data[4].x - data[5].x;
	ai = data[4].y + data[5].y;
	bi = data[4].y - data[5].y;
	cr = data[6].x + data[7].x;
	dr = data[6].x - data[7].x;
	ci = data[6].y + data[7].y;
	di = data[6].y - data[7].y;

	wr = (ar + cr);
	yr = (ar - cr);
	wi = (ai + ci);
	yi = (ai - ci);

	data[0].x = sr + wr;
	data[4].x = sr - wr;
	data[0].y = si + wi;
	data[4].y = si - wi;
	data[2].x = ur + yi;
	data[6].x = ur - yi;
	data[2].y = ui - yr;
	data[6].y = ui + yr;

	ar = br - di;
	cr = br + di;
	ai = bi + dr;
	ci = bi - dr;

	xr = MUL_SHIFT30(SQRT1_2, ar - ai);
	xi = MUL_SHIFT30(SQRT1_2, ar + ai);
	zr = MUL_SHIFT30(SQRT1_2, cr - ci);
	zi = MUL_SHIFT30(SQRT1_2, cr + ci);

	data[3].x = tr - xr;
	data[7].x = tr + xr;
	data[3].y = ti - xi;
	data[7].y = ti + xi;
	data[1].x = vr + zi;
	data[5].x = vr - zi;
	data[1].y = vi - zr;
	data[5].y = vi + zr;

	data += 8;
    }
    return;
}
#endif

static void
i4b_echo_cancel_radix4_cr(struct i4b_complex *data0,
			  const struct i4b_complex *table0,
			  uint16_t step, uint16_t n)
{
    struct i4b_complex *data;
    struct i4b_complex *data_end0;
    struct i4b_complex *data_end1;
    const struct i4b_complex *table;

    int32_t ar, ai, br, bi, cr, ci, dr, di, tr, ti;
    int32_t wd, ws, wi;

    while (step < n) {

        data = data0;
	data_end0 = data0 + n;

	while (data != data_end0) {

	    table = table0;
	    data_end1 = data + step;

	    while (data != data_end1) {

	        ar = data[0].x;
		ai = data[0].y;
		data += step;
				
		ws = table[0].x;
		wi = table[0].y;
		br = data[0].x;
		bi = data[0].y;
		wd = ws + (2*wi);
		tr = MUL_SHIFT30(wi, br + bi);
		br = MUL_SHIFT30(wd, br) - tr;
		bi = MUL_SHIFT30(ws, bi) + tr;
		data += step;

		ws = table[1].x;
		wi = table[1].y;
		cr = data[0].x;
		ci = data[0].y;
		wd = ws + (2*wi);
		tr = MUL_SHIFT30(wi, cr + ci);
		cr = MUL_SHIFT30(wd, cr) - tr;
		ci = MUL_SHIFT30(ws, ci) + tr;
		data += step;

		ws = table[2].x;
		wi = table[2].y;
		dr = data[0].x;
		di = data[0].y;
		wd = ws + (2*wi);
		tr = MUL_SHIFT30(wi, dr + di);
		dr = MUL_SHIFT30(wd, dr) - tr;
		di = MUL_SHIFT30(ws, di) + tr;
		table += 3;

		tr = ar;
		ti = ai;
		ar = tr - br;
		ai = ti - bi;
		br = tr + br;
		bi = ti + bi;

		tr = cr;
		ti = ci;
		cr = tr + dr;
		ci = di - ti;
		dr = tr - dr;
		di = di + ti;

		data[0].x = ar + ci;
		data[0].y = ai + dr;
		data -= step;
		data[0].x = br - cr;
		data[0].y = bi - di;
		data -= step;
		data[0].x = ar - ci;
		data[0].y = ai - dr;
		data -= step;
		data[0].x = br + cr;
		data[0].y = bi + di;
		data += 1;
	    }
	    data += (3*step);
	}
	table0 += (3*step);
	step *= 4;
    }
    return;
}

#if (I4B_ECHO_CANCEL_N_COMPLEX == 0x100)
static void
i4b_echo_cancel_fft(struct i4b_complex *data, uint8_t inverse)
{
  static const uint16_t bit_rev[0xf2] = {
	0x010, 0x008, 0x020, 0x004, 0x028, 0x014, 0x030, 0x00c, 
	0x034, 0x02c, 0x038, 0x01c, 0x040, 0x002, 0x044, 0x022, 
	0x048, 0x012, 0x04c, 0x032, 0x050, 0x00a, 0x052, 0x04a, 
	0x054, 0x02a, 0x058, 0x01a, 0x05c, 0x03a, 0x060, 0x006, 
	0x062, 0x046, 0x064, 0x026, 0x068, 0x016, 0x06a, 0x056, 
	0x06c, 0x036, 0x070, 0x00e, 0x072, 0x04e, 0x074, 0x02e, 
	0x076, 0x06e, 0x078, 0x01e, 0x07a, 0x05e, 0x07c, 0x03e, 
	0x080, 0x001, 0x082, 0x041, 0x084, 0x021, 0x086, 0x061, 
	0x088, 0x011, 0x08a, 0x051, 0x08c, 0x031, 0x08e, 0x071, 
	0x090, 0x009, 0x091, 0x089, 0x092, 0x049, 0x094, 0x029, 
	0x096, 0x069, 0x098, 0x019, 0x09a, 0x059, 0x09c, 0x039, 
	0x09e, 0x079, 0x0a0, 0x005, 0x0a1, 0x085, 0x0a2, 0x045, 
	0x0a4, 0x025, 0x0a6, 0x065, 0x0a8, 0x015, 0x0a9, 0x095, 
	0x0aa, 0x055, 0x0ac, 0x035, 0x0ae, 0x075, 0x0b0, 0x00d, 
	0x0b1, 0x08d, 0x0b2, 0x04d, 0x0b4, 0x02d, 0x0b5, 0x0ad, 
	0x0b6, 0x06d, 0x0b8, 0x01d, 0x0b9, 0x09d, 0x0ba, 0x05d, 
	0x0bc, 0x03d, 0x0be, 0x07d, 0x0c0, 0x003, 0x0c1, 0x083, 
	0x0c2, 0x043, 0x0c4, 0x023, 0x0c5, 0x0a3, 0x0c6, 0x063, 
	0x0c8, 0x013, 0x0c9, 0x093, 0x0ca, 0x053, 0x0cc, 0x033, 
	0x0cd, 0x0b3, 0x0ce, 0x073, 0x0d0, 0x00b, 0x0d1, 0x08b, 
	0x0d2, 0x04b, 0x0d3, 0x0cb, 0x0d4, 0x02b, 0x0d5, 0x0ab, 
	0x0d6, 0x06b, 0x0d8, 0x01b, 0x0d9, 0x09b, 0x0da, 0x05b, 
	0x0dc, 0x03b, 0x0dd, 0x0bb, 0x0de, 0x07b, 0x0e0, 0x007, 
	0x0e1, 0x087, 0x0e2, 0x047, 0x0e3, 0x0c7, 0x0e4, 0x027, 
	0x0e5, 0x0a7, 0x0e6, 0x067, 0x0e8, 0x017, 0x0e9, 0x097, 
	0x0ea, 0x057, 0x0eb, 0x0d7, 0x0ec, 0x037, 0x0ed, 0x0b7, 
	0x0ee, 0x077, 0x0f0, 0x00f, 0x0f1, 0x08f, 0x0f2, 0x04f, 
	0x0f3, 0x0cf, 0x0f4, 0x02f, 0x0f5, 0x0af, 0x0f6, 0x06f, 
	0x0f7, 0x0ef, 0x0f8, 0x01f, 0x0f9, 0x09f, 0x0fa, 0x05f, 
	0x0fb, 0x0df, 0x0fc, 0x03f, 0x0fd, 0x0bf, 0x0fe, 0x07f, 
	0, 0
  };

  static const struct i4b_complex fft_table[0xfc] = {
	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x43103086, -0x0323ecbf }, { 0x418d2621, -0x01921560 }, { 0x4488e37f, -0x04b54825 },
	{ 0x45f704f7, -0x0645e9b0 }, { 0x43103086, -0x0323ecbf }, { 0x48b2b336, -0x09640838 },
	{ 0x48b2b336, -0x09640838 }, { 0x4488e37f, -0x04b54825 }, { 0x4c77a88f, -0x0e05c136 },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x4da1fab5, -0x0f8cfcbe }, { 0x475a5c78, -0x07d59396 }, { 0x52beac9f, -0x17088531 },
	{ 0x4fd288dd, -0x1294062f }, { 0x48b2b336, -0x09640838 }, { 0x553805f3, -0x1b5d100a },
	{ 0x51d1dc80, -0x158f9a76 }, { 0x49ffd418, -0x0af10a23 }, { 0x573b2635, -0x1f8ba4dc },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x553805f3, -0x1b5d100a }, { 0x4c77a88f, -0x0e05c136 }, { 0x59d438e6, -0x275ff453 },
	{ 0x569cc31c, -0x1e2b5d39 }, { 0x4da1fab5, -0x0f8cfcbe }, { 0x5a6690ae, -0x2afad26a },
	{ 0x57cc15bd, -0x20e70f33 }, { 0x4ec05432, -0x1111d263 }, { 0x5a7b7f1b, -0x2e5a1070 },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x5987b08a, -0x261feffa }, { 0x50d86e6d, -0x14135c95 }, { 0x592d59db, -0x34534f41 },
	{ 0x5a12e721, -0x2899e64b }, { 0x51d1dc80, -0x158f9a76 }, { 0x57cc15bd, -0x36e5068b },
	{ 0x5a6690ae, -0x2afad26a }, { 0x52beac9f, -0x17088531 }, { 0x55f104dc, -0x392a9643 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x5a6690ae, -0x2f6bbe45 }, { 0x5471e2e7, -0x19ef7944 }, { 0x50d86e6d, -0x3cc511d9 },
	{ 0x5a12e721, -0x317900d7 }, { 0x553805f3, -0x1b5d100a }, { 0x4da1fab5, -0x3e14fdf8 },
	{ 0x5987b08a, -0x3367c090 }, { 0x55f104dc, -0x1cc66e9a }, { 0x49ffd418, -0x3f0ec9f5 },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x57cc15bd, -0x36e5068b }, { 0x573b2635, -0x1f8ba4dc }, { 0x418d2621, -0x3ffb10c2 },
	{ 0x569cc31c, -0x387165e4 }, { 0x57cc15bd, -0x20e70f33 }, { 0x3cc85709, -0x3fec43c7 },
	{ 0x553805f3, -0x39daf5e9 }, { 0x584f7b59, -0x223d66a9 }, { 0x37af354d, -0x3f84c8e2 },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x51d1dc80, -0x3c42420a }, { 0x592d59db, -0x24da0a9a }, { 0x2c9caf6d, -0x3dae81cf },
	{ 0x4fd288dd, -0x3d3e82ae }, { 0x5987b08a, -0x261feffa }, { 0x26b2a795, -0x3c42420a },
	{ 0x4da1fab5, -0x3e14fdf8 }, { 0x59d438e6, -0x275ff453 }, { 0x2092f05f, -0x3a8269a3 },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x48b2b336, -0x3f4eaaff }, { 0x5a43b190, -0x29cd9578 }, { 0x13d4ae08, -0x361214b1 },
	{ 0x45f704f7, -0x3fb11b48 }, { 0x5a6690ae, -0x2afad26a }, { 0x0d47d097, -0x3367c090 },
	{ 0x43103086, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2c216eab }, { 0x06a886a1, -0x30761c18 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x3cc85709, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2e5a1070 }, {-0x06a886a1, -0x29cd9578 },
	{ 0x396b3199, -0x3fb11b48 }, { 0x5a6690ae, -0x2f6bbe45 }, {-0x0d47d097, -0x261feffa },
	{ 0x35eaa2c7, -0x3f4eaaff }, { 0x5a43b190, -0x30761c18 }, {-0x13d4ae08, -0x223d66a9 },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x2e88013a, -0x3e14fdf8 }, { 0x59d438e6, -0x32744494 }, {-0x2092f05f, -0x19ef7944 },
	{ 0x2aaa7c7f, -0x3d3e82ae }, { 0x5987b08a, -0x3367c090 }, {-0x26b2a795, -0x158f9a76 },
	{ 0x26b2a795, -0x3c42420a }, { 0x592d59db, -0x34534f41 }, {-0x2c9caf6d, -0x1111d263 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x1e7de5df, -0x39daf5e9 }, { 0x584f7b59, -0x361214b1 }, {-0x37af354d, -0x07d59396 },
	{ 0x1a4608ab, -0x387165e4 }, { 0x57cc15bd, -0x36e5068b }, {-0x3cc85709, -0x0323ecbf },
	{ 0x15fdf758, -0x36e5068b }, { 0x573b2635, -0x37af8159 }, {-0x418d2621,  0x01921560 },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x0d47d097, -0x3367c090 }, { 0x55f104dc, -0x392a9643 }, {-0x49ffd418,  0x0af10a23 },
	{ 0x08df1a8d, -0x317900d7 }, { 0x553805f3, -0x39daf5e9 }, {-0x4da1fab5,  0x0f8cfcbe },
	{ 0x0470ebdc, -0x2f6bbe45 }, { 0x5471e2e7, -0x3a8269a3 }, {-0x50d86e6d,  0x14135c95 },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x0470ebdc, -0x2afad26a }, { 0x52beac9f, -0x3bb6276e }, {-0x55f104dc,  0x1cc66e9a },
	{-0x08df1a8d, -0x2899e64b }, { 0x51d1dc80, -0x3c42420a }, {-0x57cc15bd,  0x20e70f33 },
	{-0x0d47d097, -0x261feffa }, { 0x50d86e6d, -0x3cc511d9 }, {-0x592d59db,  0x24da0a9a },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x15fdf758, -0x20e70f33 }, { 0x4ec05432, -0x3dae81cf }, {-0x5a7b7f1b,  0x2c216eab },
	{-0x1a4608ab, -0x1e2b5d39 }, { 0x4da1fab5, -0x3e14fdf8 }, {-0x5a6690ae,  0x2f6bbe45 },
	{-0x1e7de5df, -0x1b5d100a }, { 0x4c77a88f, -0x3e71e759 }, {-0x59d438e6,  0x32744494 },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x26b2a795, -0x158f9a76 }, { 0x49ffd418, -0x3f0ec9f5 }, {-0x573b2635,  0x37af8159 },
	{-0x2aaa7c7f, -0x1294062f }, { 0x48b2b336, -0x3f4eaaff }, {-0x553805f3,  0x39daf5e9 },
	{-0x2e88013a, -0x0f8cfcbe }, { 0x475a5c78, -0x3f84c8e2 }, {-0x52beac9f,  0x3bb6276e },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },
	{-0x35eaa2c7, -0x09640838 }, { 0x4488e37f, -0x3fd39b5b }, {-0x4c77a88f,  0x3e71e759 },
	{-0x396b3199, -0x0645e9b0 }, { 0x43103086, -0x3fec43c7 }, {-0x48b2b336,  0x3f4eaaff },
	{-0x3cc85709, -0x0323ecbf }, { 0x418d2621, -0x3ffb10c2 }, {-0x4488e37f,  0x3fd39b5b },
    };

    struct i4b_complex *data_end;
    const uint16_t *p;

    /* In-place index bit-reversal */
    p = bit_rev;
    while (*p) {
      int32_t t;
      uint16_t a,b;

      a = *(p + 0);
      b = *(p + 1);
      t = data[b].x;
      data[b].x = data[a].x;
      data[a].x = t;

      t = data[b].y;
      data[b].y = data[a].y;
      data[a].y = t;

      a = *(p + 2);
      b = *(p + 3);
      p += 4;
      t = data[b].x;
      data[b].x = data[a].x;
      data[a].x = t;

      t = data[b].y;
      data[b].y = data[a].y;
      data[a].y = t;
    }

    /* Do the FFT */
    i4b_echo_cancel_radix4_fp(data, I4B_ECHO_CANCEL_N_COMPLEX);
    i4b_echo_cancel_radix4_cr(data, fft_table, 4, I4B_ECHO_CANCEL_N_COMPLEX);

    data_end = data + I4B_ECHO_CANCEL_N_COMPLEX;

    /* Check for inverse transform */
    if (inverse & 1) {
      if (inverse & 2) {
        while (data != data_end) {
          /* loop unrolling for higher performance */
          data[0].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[1].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[2].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[3].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[4].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[5].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[6].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[7].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data += 8;
        }
      }
    } else {
      while (data != data_end) {
        /* loop unrolling for higher performance */
        data[0].y = -data[0].y;
        data[1].y = -data[1].y;
        data[2].y = -data[2].y;
        data[3].y = -data[3].y;
        data[4].y = -data[4].y;
        data[5].y = -data[5].y;
        data[6].y = -data[6].y;
        data[7].y = -data[7].y;
        data += 8;
      }
    }
    return;
}
#endif

#if (I4B_ECHO_CANCEL_N_COMPLEX == 0x200)
static void
i4b_echo_cancel_fft(struct i4b_complex *data, uint8_t inverse)
{
  static const uint16_t bit_rev[0x1e2] = {
	0x020, 0x008, 0x030, 0x018, 0x040, 0x004, 0x048, 0x024, 
	0x050, 0x014, 0x058, 0x034, 0x060, 0x00c, 0x064, 0x04c, 
	0x068, 0x02c, 0x070, 0x01c, 0x074, 0x05c, 0x078, 0x03c, 
	0x080, 0x002, 0x084, 0x042, 0x088, 0x022, 0x08c, 0x062, 
	0x090, 0x012, 0x094, 0x052, 0x098, 0x032, 0x09c, 0x072, 
	0x0a0, 0x00a, 0x0a2, 0x08a, 0x0a4, 0x04a, 0x0a8, 0x02a, 
	0x0ac, 0x06a, 0x0b0, 0x01a, 0x0b2, 0x09a, 0x0b4, 0x05a, 
	0x0b8, 0x03a, 0x0bc, 0x07a, 0x0c0, 0x006, 0x0c2, 0x086, 
	0x0c4, 0x046, 0x0c8, 0x026, 0x0ca, 0x0a6, 0x0cc, 0x066, 
	0x0d0, 0x016, 0x0d2, 0x096, 0x0d4, 0x056, 0x0d8, 0x036, 
	0x0da, 0x0b6, 0x0dc, 0x076, 0x0e0, 0x00e, 0x0e2, 0x08e, 
	0x0e4, 0x04e, 0x0e6, 0x0ce, 0x0e8, 0x02e, 0x0ea, 0x0ae, 
	0x0ec, 0x06e, 0x0f0, 0x01e, 0x0f2, 0x09e, 0x0f4, 0x05e, 
	0x0f6, 0x0de, 0x0f8, 0x03e, 0x0fa, 0x0be, 0x0fc, 0x07e, 
	0x100, 0x001, 0x102, 0x081, 0x104, 0x041, 0x106, 0x0c1, 
	0x108, 0x021, 0x10a, 0x0a1, 0x10c, 0x061, 0x10e, 0x0e1, 
	0x110, 0x011, 0x112, 0x091, 0x114, 0x051, 0x116, 0x0d1, 
	0x118, 0x031, 0x11a, 0x0b1, 0x11c, 0x071, 0x11e, 0x0f1, 
	0x120, 0x009, 0x121, 0x109, 0x122, 0x089, 0x124, 0x049, 
	0x126, 0x0c9, 0x128, 0x029, 0x12a, 0x0a9, 0x12c, 0x069, 
	0x12e, 0x0e9, 0x130, 0x019, 0x131, 0x119, 0x132, 0x099, 
	0x134, 0x059, 0x136, 0x0d9, 0x138, 0x039, 0x13a, 0x0b9, 
	0x13c, 0x079, 0x13e, 0x0f9, 0x140, 0x005, 0x141, 0x105, 
	0x142, 0x085, 0x144, 0x045, 0x146, 0x0c5, 0x148, 0x025, 
	0x149, 0x125, 0x14a, 0x0a5, 0x14c, 0x065, 0x14e, 0x0e5, 
	0x150, 0x015, 0x151, 0x115, 0x152, 0x095, 0x154, 0x055, 
	0x156, 0x0d5, 0x158, 0x035, 0x159, 0x135, 0x15a, 0x0b5, 
	0x15c, 0x075, 0x15e, 0x0f5, 0x160, 0x00d, 0x161, 0x10d, 
	0x162, 0x08d, 0x164, 0x04d, 0x165, 0x14d, 0x166, 0x0cd, 
	0x168, 0x02d, 0x169, 0x12d, 0x16a, 0x0ad, 0x16c, 0x06d, 
	0x16e, 0x0ed, 0x170, 0x01d, 0x171, 0x11d, 0x172, 0x09d, 
	0x174, 0x05d, 0x175, 0x15d, 0x176, 0x0dd, 0x178, 0x03d, 
	0x179, 0x13d, 0x17a, 0x0bd, 0x17c, 0x07d, 0x17e, 0x0fd, 
	0x180, 0x003, 0x181, 0x103, 0x182, 0x083, 0x184, 0x043, 
	0x185, 0x143, 0x186, 0x0c3, 0x188, 0x023, 0x189, 0x123, 
	0x18a, 0x0a3, 0x18c, 0x063, 0x18d, 0x163, 0x18e, 0x0e3, 
	0x190, 0x013, 0x191, 0x113, 0x192, 0x093, 0x194, 0x053, 
	0x195, 0x153, 0x196, 0x0d3, 0x198, 0x033, 0x199, 0x133, 
	0x19a, 0x0b3, 0x19c, 0x073, 0x19d, 0x173, 0x19e, 0x0f3, 
	0x1a0, 0x00b, 0x1a1, 0x10b, 0x1a2, 0x08b, 0x1a3, 0x18b, 
	0x1a4, 0x04b, 0x1a5, 0x14b, 0x1a6, 0x0cb, 0x1a8, 0x02b, 
	0x1a9, 0x12b, 0x1aa, 0x0ab, 0x1ac, 0x06b, 0x1ad, 0x16b, 
	0x1ae, 0x0eb, 0x1b0, 0x01b, 0x1b1, 0x11b, 0x1b2, 0x09b, 
	0x1b3, 0x19b, 0x1b4, 0x05b, 0x1b5, 0x15b, 0x1b6, 0x0db, 
	0x1b8, 0x03b, 0x1b9, 0x13b, 0x1ba, 0x0bb, 0x1bc, 0x07b, 
	0x1bd, 0x17b, 0x1be, 0x0fb, 0x1c0, 0x007, 0x1c1, 0x107, 
	0x1c2, 0x087, 0x1c3, 0x187, 0x1c4, 0x047, 0x1c5, 0x147, 
	0x1c6, 0x0c7, 0x1c8, 0x027, 0x1c9, 0x127, 0x1ca, 0x0a7, 
	0x1cb, 0x1a7, 0x1cc, 0x067, 0x1cd, 0x167, 0x1ce, 0x0e7, 
	0x1d0, 0x017, 0x1d1, 0x117, 0x1d2, 0x097, 0x1d3, 0x197, 
	0x1d4, 0x057, 0x1d5, 0x157, 0x1d6, 0x0d7, 0x1d8, 0x037, 
	0x1d9, 0x137, 0x1da, 0x0b7, 0x1db, 0x1b7, 0x1dc, 0x077, 
	0x1dd, 0x177, 0x1de, 0x0f7, 0x1e0, 0x00f, 0x1e1, 0x10f, 
	0x1e2, 0x08f, 0x1e3, 0x18f, 0x1e4, 0x04f, 0x1e5, 0x14f, 
	0x1e6, 0x0cf, 0x1e7, 0x1cf, 0x1e8, 0x02f, 0x1e9, 0x12f, 
	0x1ea, 0x0af, 0x1eb, 0x1af, 0x1ec, 0x06f, 0x1ed, 0x16f, 
	0x1ee, 0x0ef, 0x1f0, 0x01f, 0x1f1, 0x11f, 0x1f2, 0x09f, 
	0x1f3, 0x19f, 0x1f4, 0x05f, 0x1f5, 0x15f, 0x1f6, 0x0df, 
	0x1f7, 0x1df, 0x1f8, 0x03f, 0x1f9, 0x13f, 0x1fa, 0x0bf, 
	0x1fb, 0x1bf, 0x1fc, 0x07f, 0x1fd, 0x17f, 0x1fe, 0x0ff, 
	0, 0
  };

  static const struct i4b_complex fft_table[0x1f8] = {
	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x45f704f7, -0x0645e9b0 }, { 0x43103086, -0x0323ecbf }, { 0x48b2b336, -0x09640838 },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x4fd288dd, -0x1294062f }, { 0x48b2b336, -0x09640838 }, { 0x553805f3, -0x1b5d100a },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x569cc31c, -0x1e2b5d39 }, { 0x4da1fab5, -0x0f8cfcbe }, { 0x5a6690ae, -0x2afad26a },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x5a12e721, -0x2899e64b }, { 0x51d1dc80, -0x158f9a76 }, { 0x57cc15bd, -0x36e5068b },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x5a12e721, -0x317900d7 }, { 0x553805f3, -0x1b5d100a }, { 0x4da1fab5, -0x3e14fdf8 },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x569cc31c, -0x387165e4 }, { 0x57cc15bd, -0x20e70f33 }, { 0x3cc85709, -0x3fec43c7 },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x4fd288dd, -0x3d3e82ae }, { 0x5987b08a, -0x261feffa }, { 0x26b2a795, -0x3c42420a },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x45f704f7, -0x3fb11b48 }, { 0x5a6690ae, -0x2afad26a }, { 0x0d47d097, -0x3367c090 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x396b3199, -0x3fb11b48 }, { 0x5a6690ae, -0x2f6bbe45 }, {-0x0d47d097, -0x261feffa },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x2aaa7c7f, -0x3d3e82ae }, { 0x5987b08a, -0x3367c090 }, {-0x26b2a795, -0x158f9a76 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x1a4608ab, -0x387165e4 }, { 0x57cc15bd, -0x36e5068b }, {-0x3cc85709, -0x0323ecbf },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x08df1a8d, -0x317900d7 }, { 0x553805f3, -0x39daf5e9 }, {-0x4da1fab5,  0x0f8cfcbe },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x08df1a8d, -0x2899e64b }, { 0x51d1dc80, -0x3c42420a }, {-0x57cc15bd,  0x20e70f33 },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x1a4608ab, -0x1e2b5d39 }, { 0x4da1fab5, -0x3e14fdf8 }, {-0x5a6690ae,  0x2f6bbe45 },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x2aaa7c7f, -0x1294062f }, { 0x48b2b336, -0x3f4eaaff }, {-0x553805f3,  0x39daf5e9 },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },
	{-0x396b3199, -0x0645e9b0 }, { 0x43103086, -0x3fec43c7 }, {-0x48b2b336,  0x3f4eaaff },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x418d2621, -0x01921560 }, { 0x40c7d2be, -0x00c90e90 }, { 0x424ff28f, -0x025b0caf },
	{ 0x43103086, -0x0323ecbf }, { 0x418d2621, -0x01921560 }, { 0x4488e37f, -0x04b54825 },
	{ 0x4488e37f, -0x04b54825 }, { 0x424ff28f, -0x025b0caf }, { 0x46aa0d6d, -0x070de172 },
	{ 0x45f704f7, -0x0645e9b0 }, { 0x43103086, -0x0323ecbf }, { 0x48b2b336, -0x09640838 },
	{ 0x475a5c78, -0x07d59396 }, { 0x43cdd89b, -0x03ecadd0 }, { 0x4aa22037, -0x0bb6ecf0 },
	{ 0x48b2b336, -0x09640838 }, { 0x4488e37f, -0x04b54825 }, { 0x4c77a88f, -0x0e05c136 },
	{ 0x49ffd418, -0x0af10a23 }, { 0x454149fd, -0x057db403 }, { 0x4e32a957, -0x104fb80f },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x4c77a88f, -0x0e05c136 }, { 0x46aa0d6d, -0x070de172 }, { 0x5156b6d9, -0x14d1e243 },
	{ 0x4da1fab5, -0x0f8cfcbe }, { 0x475a5c78, -0x07d59396 }, { 0x52beac9f, -0x17088531 },
	{ 0x4ec05432, -0x1111d263 }, { 0x4807eb4b, -0x089cf868 }, { 0x5409ed4c, -0x19372a64 },
	{ 0x4fd288dd, -0x1294062f }, { 0x48b2b336, -0x09640838 }, { 0x553805f3, -0x1b5d100a },
	{ 0x50d86e6d, -0x14135c95 }, { 0x495aada3, -0x0a2abb59 }, { 0x56488dc5, -0x1d79775c },
	{ 0x51d1dc80, -0x158f9a76 }, { 0x49ffd418, -0x0af10a23 }, { 0x573b2635, -0x1f8ba4dc },
	{ 0x52beac9f, -0x17088531 }, { 0x4aa22037, -0x0bb6ecf0 }, { 0x580f7b19, -0x2192e09b },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x5471e2e7, -0x19ef7944 }, { 0x4bde1089, -0x0d415013 }, { 0x595c3e2b, -0x257db64c },
	{ 0x553805f3, -0x1b5d100a }, { 0x4c77a88f, -0x0e05c136 }, { 0x59d438e6, -0x275ff453 },
	{ 0x55f104dc, -0x1cc66e9a }, { 0x4d0e4de2, -0x0ec9a7f3 }, { 0x5a2d0957, -0x29348938 },
	{ 0x569cc31c, -0x1e2b5d39 }, { 0x4da1fab5, -0x0f8cfcbe }, { 0x5a6690ae, -0x2afad26a },
	{ 0x573b2635, -0x1f8ba4dc }, { 0x4e32a957, -0x104fb80f }, { 0x5a80baf6, -0x2cb2324c },
	{ 0x57cc15bd, -0x20e70f33 }, { 0x4ec05432, -0x1111d263 }, { 0x5a7b7f1b, -0x2e5a1070 },
	{ 0x584f7b59, -0x223d66a9 }, { 0x4f4af5d2, -0x11d34440 }, { 0x5a56deec, -0x2ff1d9c7 },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x592d59db, -0x24da0a9a }, { 0x5057081a, -0x135410c3 }, { 0x59afaf4c, -0x32eefdea },
	{ 0x5987b08a, -0x261feffa }, { 0x50d86e6d, -0x14135c95 }, { 0x592d59db, -0x34534f41 },
	{ 0x59d438e6, -0x275ff453 }, { 0x5156b6d9, -0x14d1e243 }, { 0x588c1405, -0x35a5793d },
	{ 0x5a12e721, -0x2899e64b }, { 0x51d1dc80, -0x158f9a76 }, { 0x57cc15bd, -0x36e5068b },
	{ 0x5a43b190, -0x29cd9578 }, { 0x5249daa2, -0x164c7dde }, { 0x56eda1a0, -0x3811884d },
	{ 0x5a6690ae, -0x2afad26a }, { 0x52beac9f, -0x17088531 }, { 0x55f104dc, -0x392a9643 },
	{ 0x5a7b7f1b, -0x2c216eab }, { 0x53304df6, -0x17c3a932 }, { 0x54d69714, -0x3a2fcee9 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x5a7b7f1b, -0x2e5a1070 }, { 0x5409ed4c, -0x19372a64 }, { 0x5249daa2, -0x3bfd5cc5 },
	{ 0x5a6690ae, -0x2f6bbe45 }, { 0x5471e2e7, -0x19ef7944 }, { 0x50d86e6d, -0x3cc511d9 },
	{ 0x5a43b190, -0x30761c18 }, { 0x54d69714, -0x1aa6c82c }, { 0x4f4af5d2, -0x3d77b192 },
	{ 0x5a12e721, -0x317900d7 }, { 0x553805f3, -0x1b5d100a }, { 0x4da1fab5, -0x3e14fdf8 },
	{ 0x59d438e6, -0x32744494 }, { 0x55962bc0, -0x1c1249d9 }, { 0x4bde1089, -0x3e9cc077 },
	{ 0x5987b08a, -0x3367c090 }, { 0x55f104dc, -0x1cc66e9a }, { 0x49ffd418, -0x3f0ec9f5 },
	{ 0x592d59db, -0x34534f41 }, { 0x56488dc5, -0x1d79775c }, { 0x4807eb4b, -0x3f6af2e4 },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x584f7b59, -0x361214b1 }, { 0x56eda1a0, -0x1edc1953 }, { 0x43cdd89b, -0x3fe12acc },
	{ 0x57cc15bd, -0x36e5068b }, { 0x573b2635, -0x1f8ba4dc }, { 0x418d2621, -0x3ffb10c2 },
	{ 0x573b2635, -0x37af8159 }, { 0x57854dde, -0x2039f90f }, { 0x3f35b59e, -0x3ffec42e },
	{ 0x569cc31c, -0x387165e4 }, { 0x57cc15bd, -0x20e70f33 }, { 0x3cc85709, -0x3fec43c7 },
	{ 0x55f104dc, -0x392a9643 }, { 0x580f7b19, -0x2192e09b }, { 0x3a45e1f7, -0x3fc395fa },
	{ 0x553805f3, -0x39daf5e9 }, { 0x584f7b59, -0x223d66a9 }, { 0x37af354d, -0x3f84c8e2 },
	{ 0x5471e2e7, -0x3a8269a3 }, { 0x588c1405, -0x22e69ac8 }, { 0x350536f1, -0x3f2ff24a },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x52beac9f, -0x3bb6276e }, { 0x58fb0569, -0x2434f333 }, { 0x2f7afdfd, -0x3e44a5ef },
	{ 0x51d1dc80, -0x3c42420a }, { 0x592d59db, -0x24da0a9a }, { 0x2c9caf6d, -0x3dae81cf },
	{ 0x50d86e6d, -0x3cc511d9 }, { 0x595c3e2b, -0x257db64c }, { 0x29aee694, -0x3d02f757 },
	{ 0x4fd288dd, -0x3d3e82ae }, { 0x5987b08a, -0x261feffa }, { 0x26b2a795, -0x3c42420a },
	{ 0x4ec05432, -0x3dae81cf }, { 0x59afaf4c, -0x26c0b163 }, { 0x23a8fb94, -0x3b6ca4c5 },
	{ 0x4da1fab5, -0x3e14fdf8 }, { 0x59d438e6, -0x275ff453 }, { 0x2092f05f, -0x3a8269a3 },
	{ 0x4c77a88f, -0x3e71e759 }, { 0x59f54bef, -0x27fdb2a7 }, { 0x1d719810, -0x3983e1e8 },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x49ffd418, -0x3f0ec9f5 }, { 0x5a2d0957, -0x29348938 }, { 0x17115bc0, -0x374b54cf },
	{ 0x48b2b336, -0x3f4eaaff }, { 0x5a43b190, -0x29cd9578 }, { 0x13d4ae08, -0x361214b1 },
	{ 0x475a5c78, -0x3f84c8e2 }, { 0x5a56deec, -0x2a650526 }, { 0x10911f04, -0x34c61237 },
	{ 0x45f704f7, -0x3fb11b48 }, { 0x5a6690ae, -0x2afad26a }, { 0x0d47d097, -0x3367c090 },
	{ 0x4488e37f, -0x3fd39b5b }, { 0x5a72c63b, -0x2b8ef77d }, { 0x09f9e6a2, -0x31f79948 },
	{ 0x43103086, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2c216eab }, { 0x06a886a1, -0x30761c18 },
	{ 0x418d2621, -0x3ffb10c2 }, { 0x5a80baf6, -0x2cb2324c }, { 0x0354d742, -0x2ee3cebf },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x3e68fb62, -0x3ffb10c2 }, { 0x5a80baf6, -0x2dce88aa }, {-0x0354d742, -0x2b8ef77d },
	{ 0x3cc85709, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2e5a1070 }, {-0x06a886a1, -0x29cd9578 },
	{ 0x3b1e5336, -0x3fd39b5b }, { 0x5a72c63b, -0x2ee3cebf }, {-0x09f9e6a2, -0x27fdb2a7 },
	{ 0x396b3199, -0x3fb11b48 }, { 0x5a6690ae, -0x2f6bbe45 }, {-0x0d47d097, -0x261feffa },
	{ 0x37af354d, -0x3f84c8e2 }, { 0x5a56deec, -0x2ff1d9c7 }, {-0x10911f04, -0x2434f333 },
	{ 0x35eaa2c7, -0x3f4eaaff }, { 0x5a43b190, -0x30761c18 }, {-0x13d4ae08, -0x223d66a9 },
	{ 0x341dbfd3, -0x3f0ec9f5 }, { 0x5a2d0957, -0x30f88020 }, {-0x17115bc0, -0x2039f90f },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x306c2624, -0x3e71e759 }, { 0x59f54bef, -0x31f79948 }, {-0x1d719810, -0x1c1249d9 },
	{ 0x2e88013a, -0x3e14fdf8 }, { 0x59d438e6, -0x32744494 }, {-0x2092f05f, -0x19ef7944 },
	{ 0x2c9caf6d, -0x3dae81cf }, { 0x59afaf4c, -0x32eefdea }, {-0x23a8fb94, -0x17c3a932 },
	{ 0x2aaa7c7f, -0x3d3e82ae }, { 0x5987b08a, -0x3367c090 }, {-0x26b2a795, -0x158f9a76 },
	{ 0x28b1b545, -0x3cc511d9 }, { 0x595c3e2b, -0x33de87df }, {-0x29aee694, -0x135410c3 },
	{ 0x26b2a795, -0x3c42420a }, { 0x592d59db, -0x34534f41 }, {-0x2c9caf6d, -0x1111d263 },
	{ 0x24ada23d, -0x3bb6276e }, { 0x58fb0569, -0x34c61237 }, {-0x2f7afdfd, -0x0ec9a7f3 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x2092f05f, -0x3a8269a3 }, { 0x588c1405, -0x35a5793d }, {-0x350536f1, -0x0a2abb59 },
	{ 0x1e7de5df, -0x39daf5e9 }, { 0x584f7b59, -0x361214b1 }, {-0x37af354d, -0x07d59396 },
	{ 0x1c6427aa, -0x392a9643 }, { 0x580f7b19, -0x367c9a7e }, {-0x3a45e1f7, -0x057db403 },
	{ 0x1a4608ab, -0x387165e4 }, { 0x57cc15bd, -0x36e5068b }, {-0x3cc85709, -0x0323ecbf },
	{ 0x1823dc7d, -0x37af8159 }, { 0x57854dde, -0x374b54cf }, {-0x3f35b59e, -0x00c90e90 },
	{ 0x15fdf758, -0x36e5068b }, { 0x573b2635, -0x37af8159 }, {-0x418d2621,  0x01921560 },
	{ 0x13d4ae08, -0x361214b1 }, { 0x56eda1a0, -0x3811884d }, {-0x43cdd89b,  0x03ecadd0 },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x0f7944a7, -0x34534f41 }, { 0x56488dc5, -0x38cf166a }, {-0x4807eb4b,  0x089cf868 },
	{ 0x0d47d097, -0x3367c090 }, { 0x55f104dc, -0x392a9643 }, {-0x49ffd418,  0x0af10a23 },
	{ 0x0b145041, -0x32744494 }, { 0x55962bc0, -0x3983e1e8 }, {-0x4bde1089,  0x0d415013 },
	{ 0x08df1a8d, -0x317900d7 }, { 0x553805f3, -0x39daf5e9 }, {-0x4da1fab5,  0x0f8cfcbe },
	{ 0x06a886a1, -0x30761c18 }, { 0x54d69714, -0x3a2fcee9 }, {-0x4f4af5d2,  0x11d34440 },
	{ 0x0470ebdc, -0x2f6bbe45 }, { 0x5471e2e7, -0x3a8269a3 }, {-0x50d86e6d,  0x14135c95 },
	{ 0x0238a1c6, -0x2e5a1070 }, { 0x5409ed4c, -0x3ad2c2e8 }, {-0x5249daa2,  0x164c7dde },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x0238a1c6, -0x2c216eab }, { 0x53304df6, -0x3b6ca4c5 }, {-0x54d69714,  0x1aa6c82c },
	{-0x0470ebdc, -0x2afad26a }, { 0x52beac9f, -0x3bb6276e }, {-0x55f104dc,  0x1cc66e9a },
	{-0x06a886a1, -0x29cd9578 }, { 0x5249daa2, -0x3bfd5cc5 }, {-0x56eda1a0,  0x1edc1953 },
	{-0x08df1a8d, -0x2899e64b }, { 0x51d1dc80, -0x3c42420a }, {-0x57cc15bd,  0x20e70f33 },
	{-0x0b145041, -0x275ff453 }, { 0x5156b6d9, -0x3c84d497 }, {-0x588c1405,  0x22e69ac8 },
	{-0x0d47d097, -0x261feffa }, { 0x50d86e6d, -0x3cc511d9 }, {-0x592d59db,  0x24da0a9a },
	{-0x0f7944a7, -0x24da0a9a }, { 0x5057081a, -0x3d02f757 }, {-0x59afaf4c,  0x26c0b163 },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x13d4ae08, -0x223d66a9 }, { 0x4f4af5d2, -0x3d77b192 }, {-0x5a56deec,  0x2a650526 },
	{-0x15fdf758, -0x20e70f33 }, { 0x4ec05432, -0x3dae81cf }, {-0x5a7b7f1b,  0x2c216eab },
	{-0x1823dc7d, -0x1f8ba4dc }, { 0x4e32a957, -0x3de2f148 }, {-0x5a80baf6,  0x2dce88aa },
	{-0x1a4608ab, -0x1e2b5d39 }, { 0x4da1fab5, -0x3e14fdf8 }, {-0x5a6690ae,  0x2f6bbe45 },
	{-0x1c6427aa, -0x1cc66e9a }, { 0x4d0e4de2, -0x3e44a5ef }, {-0x5a2d0957,  0x30f88020 },
	{-0x1e7de5df, -0x1b5d100a }, { 0x4c77a88f, -0x3e71e759 }, {-0x59d438e6,  0x32744494 },
	{-0x2092f05f, -0x19ef7944 }, { 0x4bde1089, -0x3e9cc077 }, {-0x595c3e2b,  0x33de87df },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x24ada23d, -0x17088531 }, { 0x4aa22037, -0x3eeb3348 }, {-0x580f7b19,  0x367c9a7e },
	{-0x26b2a795, -0x158f9a76 }, { 0x49ffd418, -0x3f0ec9f5 }, {-0x573b2635,  0x37af8159 },
	{-0x28b1b545, -0x14135c95 }, { 0x495aada3, -0x3f2ff24a }, {-0x56488dc5,  0x38cf166a },
	{-0x2aaa7c7f, -0x1294062f }, { 0x48b2b336, -0x3f4eaaff }, {-0x553805f3,  0x39daf5e9 },
	{-0x2c9caf6d, -0x1111d263 }, { 0x4807eb4b, -0x3f6af2e4 }, {-0x5409ed4c,  0x3ad2c2e8 },
	{-0x2e88013a, -0x0f8cfcbe }, { 0x475a5c78, -0x3f84c8e2 }, {-0x52beac9f,  0x3bb6276e },
	{-0x306c2624, -0x0e05c136 }, { 0x46aa0d6d, -0x3f9c2bfb }, {-0x5156b6d9,  0x3c84d497 },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },
	{-0x341dbfd3, -0x0af10a23 }, { 0x454149fd, -0x3fc395fa }, {-0x4e32a957,  0x3de2f148 },
	{-0x35eaa2c7, -0x09640838 }, { 0x4488e37f, -0x3fd39b5b }, {-0x4c77a88f,  0x3e71e759 },
	{-0x37af354d, -0x07d59396 }, { 0x43cdd89b, -0x3fe12acc }, {-0x4aa22037,  0x3eeb3348 },
	{-0x396b3199, -0x0645e9b0 }, { 0x43103086, -0x3fec43c7 }, {-0x48b2b336,  0x3f4eaaff },
	{-0x3b1e5336, -0x04b54825 }, { 0x424ff28f, -0x3ff4e5e0 }, {-0x46aa0d6d,  0x3f9c2bfb },
	{-0x3cc85709, -0x0323ecbf }, { 0x418d2621, -0x3ffb10c2 }, {-0x4488e37f,  0x3fd39b5b },
	{-0x3e68fb62, -0x01921560 }, { 0x40c7d2be, -0x3ffec42e }, {-0x424ff28f,  0x3ff4e5e0 },
    };

    struct i4b_complex *data_end;
    const uint16_t *p;

    /* In-place index bit-reversal */
    p = bit_rev;
    while (*p) {
      int32_t t;
      uint16_t a,b;

      a = *(p + 0);
      b = *(p + 1);
      t = data[b].x;
      data[b].x = data[a].x;
      data[a].x = t;

      t = data[b].y;
      data[b].y = data[a].y;
      data[a].y = t;

      a = *(p + 2);
      b = *(p + 3);
      p += 4;
      t = data[b].x;
      data[b].x = data[a].x;
      data[a].x = t;

      t = data[b].y;
      data[b].y = data[a].y;
      data[a].y = t;
    }

    /* Do the FFT */
    i4b_echo_cancel_radix8_fp(data, I4B_ECHO_CANCEL_N_COMPLEX);
    i4b_echo_cancel_radix4_cr(data, fft_table, 8, I4B_ECHO_CANCEL_N_COMPLEX);

    data_end = data + I4B_ECHO_CANCEL_N_COMPLEX;

    /* Check for inverse transform */
    if (inverse & 1) {
      if (inverse & 2) {
        while (data != data_end) {
          /* loop unrolling for higher performance */
          data[0].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[1].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[2].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[3].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[4].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[5].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[6].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[7].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data += 8;
        }
      }
    } else {
      while (data != data_end) {
        /* loop unrolling for higher performance */
        data[0].y = -data[0].y;
        data[1].y = -data[1].y;
        data[2].y = -data[2].y;
        data[3].y = -data[3].y;
        data[4].y = -data[4].y;
        data[5].y = -data[5].y;
        data[6].y = -data[6].y;
        data[7].y = -data[7].y;
        data += 8;
      }
    }
    return;
}
#endif

#if (I4B_ECHO_CANCEL_N_COMPLEX == 0x400)
static void
i4b_echo_cancel_fft(struct i4b_complex *data, uint8_t inverse)
{
  static const uint16_t bit_rev[0x3e2] = {
	0x020, 0x010, 0x040, 0x008, 0x050, 0x028, 0x060, 0x018, 
	0x068, 0x058, 0x070, 0x038, 0x080, 0x004, 0x088, 0x044, 
	0x090, 0x024, 0x098, 0x064, 0x0a0, 0x014, 0x0a4, 0x094, 
	0x0a8, 0x054, 0x0b0, 0x034, 0x0b8, 0x074, 0x0c0, 0x00c, 
	0x0c4, 0x08c, 0x0c8, 0x04c, 0x0d0, 0x02c, 0x0d4, 0x0ac, 
	0x0d8, 0x06c, 0x0e0, 0x01c, 0x0e4, 0x09c, 0x0e8, 0x05c, 
	0x0ec, 0x0dc, 0x0f0, 0x03c, 0x0f4, 0x0bc, 0x0f8, 0x07c, 
	0x100, 0x002, 0x104, 0x082, 0x108, 0x042, 0x10c, 0x0c2, 
	0x110, 0x022, 0x114, 0x0a2, 0x118, 0x062, 0x11c, 0x0e2, 
	0x120, 0x012, 0x122, 0x112, 0x124, 0x092, 0x128, 0x052, 
	0x12c, 0x0d2, 0x130, 0x032, 0x134, 0x0b2, 0x138, 0x072, 
	0x13c, 0x0f2, 0x140, 0x00a, 0x142, 0x10a, 0x144, 0x08a, 
	0x148, 0x04a, 0x14c, 0x0ca, 0x150, 0x02a, 0x152, 0x12a, 
	0x154, 0x0aa, 0x158, 0x06a, 0x15c, 0x0ea, 0x160, 0x01a, 
	0x162, 0x11a, 0x164, 0x09a, 0x168, 0x05a, 0x16a, 0x15a, 
	0x16c, 0x0da, 0x170, 0x03a, 0x172, 0x13a, 0x174, 0x0ba, 
	0x178, 0x07a, 0x17c, 0x0fa, 0x180, 0x006, 0x182, 0x106, 
	0x184, 0x086, 0x188, 0x046, 0x18a, 0x146, 0x18c, 0x0c6, 
	0x190, 0x026, 0x192, 0x126, 0x194, 0x0a6, 0x198, 0x066, 
	0x19a, 0x166, 0x19c, 0x0e6, 0x1a0, 0x016, 0x1a2, 0x116, 
	0x1a4, 0x096, 0x1a6, 0x196, 0x1a8, 0x056, 0x1aa, 0x156, 
	0x1ac, 0x0d6, 0x1b0, 0x036, 0x1b2, 0x136, 0x1b4, 0x0b6, 
	0x1b8, 0x076, 0x1ba, 0x176, 0x1bc, 0x0f6, 0x1c0, 0x00e, 
	0x1c2, 0x10e, 0x1c4, 0x08e, 0x1c6, 0x18e, 0x1c8, 0x04e, 
	0x1ca, 0x14e, 0x1cc, 0x0ce, 0x1d0, 0x02e, 0x1d2, 0x12e, 
	0x1d4, 0x0ae, 0x1d6, 0x1ae, 0x1d8, 0x06e, 0x1da, 0x16e, 
	0x1dc, 0x0ee, 0x1e0, 0x01e, 0x1e2, 0x11e, 0x1e4, 0x09e, 
	0x1e6, 0x19e, 0x1e8, 0x05e, 0x1ea, 0x15e, 0x1ec, 0x0de, 
	0x1ee, 0x1de, 0x1f0, 0x03e, 0x1f2, 0x13e, 0x1f4, 0x0be, 
	0x1f6, 0x1be, 0x1f8, 0x07e, 0x1fa, 0x17e, 0x1fc, 0x0fe, 
	0x200, 0x001, 0x202, 0x101, 0x204, 0x081, 0x206, 0x181, 
	0x208, 0x041, 0x20a, 0x141, 0x20c, 0x0c1, 0x20e, 0x1c1, 
	0x210, 0x021, 0x212, 0x121, 0x214, 0x0a1, 0x216, 0x1a1, 
	0x218, 0x061, 0x21a, 0x161, 0x21c, 0x0e1, 0x21e, 0x1e1, 
	0x220, 0x011, 0x221, 0x211, 0x222, 0x111, 0x224, 0x091, 
	0x226, 0x191, 0x228, 0x051, 0x22a, 0x151, 0x22c, 0x0d1, 
	0x22e, 0x1d1, 0x230, 0x031, 0x232, 0x131, 0x234, 0x0b1, 
	0x236, 0x1b1, 0x238, 0x071, 0x23a, 0x171, 0x23c, 0x0f1, 
	0x23e, 0x1f1, 0x240, 0x009, 0x241, 0x209, 0x242, 0x109, 
	0x244, 0x089, 0x246, 0x189, 0x248, 0x049, 0x24a, 0x149, 
	0x24c, 0x0c9, 0x24e, 0x1c9, 0x250, 0x029, 0x251, 0x229, 
	0x252, 0x129, 0x254, 0x0a9, 0x256, 0x1a9, 0x258, 0x069, 
	0x25a, 0x169, 0x25c, 0x0e9, 0x25e, 0x1e9, 0x260, 0x019, 
	0x261, 0x219, 0x262, 0x119, 0x264, 0x099, 0x266, 0x199, 
	0x268, 0x059, 0x269, 0x259, 0x26a, 0x159, 0x26c, 0x0d9, 
	0x26e, 0x1d9, 0x270, 0x039, 0x271, 0x239, 0x272, 0x139, 
	0x274, 0x0b9, 0x276, 0x1b9, 0x278, 0x079, 0x27a, 0x179, 
	0x27c, 0x0f9, 0x27e, 0x1f9, 0x280, 0x005, 0x281, 0x205, 
	0x282, 0x105, 0x284, 0x085, 0x286, 0x185, 0x288, 0x045, 
	0x289, 0x245, 0x28a, 0x145, 0x28c, 0x0c5, 0x28e, 0x1c5, 
	0x290, 0x025, 0x291, 0x225, 0x292, 0x125, 0x294, 0x0a5, 
	0x296, 0x1a5, 0x298, 0x065, 0x299, 0x265, 0x29a, 0x165, 
	0x29c, 0x0e5, 0x29e, 0x1e5, 0x2a0, 0x015, 0x2a1, 0x215, 
	0x2a2, 0x115, 0x2a4, 0x095, 0x2a5, 0x295, 0x2a6, 0x195, 
	0x2a8, 0x055, 0x2a9, 0x255, 0x2aa, 0x155, 0x2ac, 0x0d5, 
	0x2ae, 0x1d5, 0x2b0, 0x035, 0x2b1, 0x235, 0x2b2, 0x135, 
	0x2b4, 0x0b5, 0x2b6, 0x1b5, 0x2b8, 0x075, 0x2b9, 0x275, 
	0x2ba, 0x175, 0x2bc, 0x0f5, 0x2be, 0x1f5, 0x2c0, 0x00d, 
	0x2c1, 0x20d, 0x2c2, 0x10d, 0x2c4, 0x08d, 0x2c5, 0x28d, 
	0x2c6, 0x18d, 0x2c8, 0x04d, 0x2c9, 0x24d, 0x2ca, 0x14d, 
	0x2cc, 0x0cd, 0x2ce, 0x1cd, 0x2d0, 0x02d, 0x2d1, 0x22d, 
	0x2d2, 0x12d, 0x2d4, 0x0ad, 0x2d5, 0x2ad, 0x2d6, 0x1ad, 
	0x2d8, 0x06d, 0x2d9, 0x26d, 0x2da, 0x16d, 0x2dc, 0x0ed, 
	0x2de, 0x1ed, 0x2e0, 0x01d, 0x2e1, 0x21d, 0x2e2, 0x11d, 
	0x2e4, 0x09d, 0x2e5, 0x29d, 0x2e6, 0x19d, 0x2e8, 0x05d, 
	0x2e9, 0x25d, 0x2ea, 0x15d, 0x2ec, 0x0dd, 0x2ed, 0x2dd, 
	0x2ee, 0x1dd, 0x2f0, 0x03d, 0x2f1, 0x23d, 0x2f2, 0x13d, 
	0x2f4, 0x0bd, 0x2f5, 0x2bd, 0x2f6, 0x1bd, 0x2f8, 0x07d, 
	0x2f9, 0x27d, 0x2fa, 0x17d, 0x2fc, 0x0fd, 0x2fe, 0x1fd, 
	0x300, 0x003, 0x301, 0x203, 0x302, 0x103, 0x304, 0x083, 
	0x305, 0x283, 0x306, 0x183, 0x308, 0x043, 0x309, 0x243, 
	0x30a, 0x143, 0x30c, 0x0c3, 0x30d, 0x2c3, 0x30e, 0x1c3, 
	0x310, 0x023, 0x311, 0x223, 0x312, 0x123, 0x314, 0x0a3, 
	0x315, 0x2a3, 0x316, 0x1a3, 0x318, 0x063, 0x319, 0x263, 
	0x31a, 0x163, 0x31c, 0x0e3, 0x31d, 0x2e3, 0x31e, 0x1e3, 
	0x320, 0x013, 0x321, 0x213, 0x322, 0x113, 0x323, 0x313, 
	0x324, 0x093, 0x325, 0x293, 0x326, 0x193, 0x328, 0x053, 
	0x329, 0x253, 0x32a, 0x153, 0x32c, 0x0d3, 0x32d, 0x2d3, 
	0x32e, 0x1d3, 0x330, 0x033, 0x331, 0x233, 0x332, 0x133, 
	0x334, 0x0b3, 0x335, 0x2b3, 0x336, 0x1b3, 0x338, 0x073, 
	0x339, 0x273, 0x33a, 0x173, 0x33c, 0x0f3, 0x33d, 0x2f3, 
	0x33e, 0x1f3, 0x340, 0x00b, 0x341, 0x20b, 0x342, 0x10b, 
	0x343, 0x30b, 0x344, 0x08b, 0x345, 0x28b, 0x346, 0x18b, 
	0x348, 0x04b, 0x349, 0x24b, 0x34a, 0x14b, 0x34c, 0x0cb, 
	0x34d, 0x2cb, 0x34e, 0x1cb, 0x350, 0x02b, 0x351, 0x22b, 
	0x352, 0x12b, 0x353, 0x32b, 0x354, 0x0ab, 0x355, 0x2ab, 
	0x356, 0x1ab, 0x358, 0x06b, 0x359, 0x26b, 0x35a, 0x16b, 
	0x35c, 0x0eb, 0x35d, 0x2eb, 0x35e, 0x1eb, 0x360, 0x01b, 
	0x361, 0x21b, 0x362, 0x11b, 0x363, 0x31b, 0x364, 0x09b, 
	0x365, 0x29b, 0x366, 0x19b, 0x368, 0x05b, 0x369, 0x25b, 
	0x36a, 0x15b, 0x36b, 0x35b, 0x36c, 0x0db, 0x36d, 0x2db, 
	0x36e, 0x1db, 0x370, 0x03b, 0x371, 0x23b, 0x372, 0x13b, 
	0x373, 0x33b, 0x374, 0x0bb, 0x375, 0x2bb, 0x376, 0x1bb, 
	0x378, 0x07b, 0x379, 0x27b, 0x37a, 0x17b, 0x37c, 0x0fb, 
	0x37d, 0x2fb, 0x37e, 0x1fb, 0x380, 0x007, 0x381, 0x207, 
	0x382, 0x107, 0x383, 0x307, 0x384, 0x087, 0x385, 0x287, 
	0x386, 0x187, 0x388, 0x047, 0x389, 0x247, 0x38a, 0x147, 
	0x38b, 0x347, 0x38c, 0x0c7, 0x38d, 0x2c7, 0x38e, 0x1c7, 
	0x390, 0x027, 0x391, 0x227, 0x392, 0x127, 0x393, 0x327, 
	0x394, 0x0a7, 0x395, 0x2a7, 0x396, 0x1a7, 0x398, 0x067, 
	0x399, 0x267, 0x39a, 0x167, 0x39b, 0x367, 0x39c, 0x0e7, 
	0x39d, 0x2e7, 0x39e, 0x1e7, 0x3a0, 0x017, 0x3a1, 0x217, 
	0x3a2, 0x117, 0x3a3, 0x317, 0x3a4, 0x097, 0x3a5, 0x297, 
	0x3a6, 0x197, 0x3a7, 0x397, 0x3a8, 0x057, 0x3a9, 0x257, 
	0x3aa, 0x157, 0x3ab, 0x357, 0x3ac, 0x0d7, 0x3ad, 0x2d7, 
	0x3ae, 0x1d7, 0x3b0, 0x037, 0x3b1, 0x237, 0x3b2, 0x137, 
	0x3b3, 0x337, 0x3b4, 0x0b7, 0x3b5, 0x2b7, 0x3b6, 0x1b7, 
	0x3b8, 0x077, 0x3b9, 0x277, 0x3ba, 0x177, 0x3bb, 0x377, 
	0x3bc, 0x0f7, 0x3bd, 0x2f7, 0x3be, 0x1f7, 0x3c0, 0x00f, 
	0x3c1, 0x20f, 0x3c2, 0x10f, 0x3c3, 0x30f, 0x3c4, 0x08f, 
	0x3c5, 0x28f, 0x3c6, 0x18f, 0x3c7, 0x38f, 0x3c8, 0x04f, 
	0x3c9, 0x24f, 0x3ca, 0x14f, 0x3cb, 0x34f, 0x3cc, 0x0cf, 
	0x3cd, 0x2cf, 0x3ce, 0x1cf, 0x3d0, 0x02f, 0x3d1, 0x22f, 
	0x3d2, 0x12f, 0x3d3, 0x32f, 0x3d4, 0x0af, 0x3d5, 0x2af, 
	0x3d6, 0x1af, 0x3d7, 0x3af, 0x3d8, 0x06f, 0x3d9, 0x26f, 
	0x3da, 0x16f, 0x3db, 0x36f, 0x3dc, 0x0ef, 0x3dd, 0x2ef, 
	0x3de, 0x1ef, 0x3e0, 0x01f, 0x3e1, 0x21f, 0x3e2, 0x11f, 
	0x3e3, 0x31f, 0x3e4, 0x09f, 0x3e5, 0x29f, 0x3e6, 0x19f, 
	0x3e7, 0x39f, 0x3e8, 0x05f, 0x3e9, 0x25f, 0x3ea, 0x15f, 
	0x3eb, 0x35f, 0x3ec, 0x0df, 0x3ed, 0x2df, 0x3ee, 0x1df, 
	0x3ef, 0x3df, 0x3f0, 0x03f, 0x3f1, 0x23f, 0x3f2, 0x13f, 
	0x3f3, 0x33f, 0x3f4, 0x0bf, 0x3f5, 0x2bf, 0x3f6, 0x1bf, 
	0x3f7, 0x3bf, 0x3f8, 0x07f, 0x3f9, 0x27f, 0x3fa, 0x17f, 
	0x3fb, 0x37f, 0x3fc, 0x0ff, 0x3fd, 0x2ff, 0x3fe, 0x1ff, 
	0, 0
  };

  static const struct i4b_complex fft_table[0x3fc] = {
	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x43103086, -0x0323ecbf }, { 0x418d2621, -0x01921560 }, { 0x4488e37f, -0x04b54825 },
	{ 0x45f704f7, -0x0645e9b0 }, { 0x43103086, -0x0323ecbf }, { 0x48b2b336, -0x09640838 },
	{ 0x48b2b336, -0x09640838 }, { 0x4488e37f, -0x04b54825 }, { 0x4c77a88f, -0x0e05c136 },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x4da1fab5, -0x0f8cfcbe }, { 0x475a5c78, -0x07d59396 }, { 0x52beac9f, -0x17088531 },
	{ 0x4fd288dd, -0x1294062f }, { 0x48b2b336, -0x09640838 }, { 0x553805f3, -0x1b5d100a },
	{ 0x51d1dc80, -0x158f9a76 }, { 0x49ffd418, -0x0af10a23 }, { 0x573b2635, -0x1f8ba4dc },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x553805f3, -0x1b5d100a }, { 0x4c77a88f, -0x0e05c136 }, { 0x59d438e6, -0x275ff453 },
	{ 0x569cc31c, -0x1e2b5d39 }, { 0x4da1fab5, -0x0f8cfcbe }, { 0x5a6690ae, -0x2afad26a },
	{ 0x57cc15bd, -0x20e70f33 }, { 0x4ec05432, -0x1111d263 }, { 0x5a7b7f1b, -0x2e5a1070 },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x5987b08a, -0x261feffa }, { 0x50d86e6d, -0x14135c95 }, { 0x592d59db, -0x34534f41 },
	{ 0x5a12e721, -0x2899e64b }, { 0x51d1dc80, -0x158f9a76 }, { 0x57cc15bd, -0x36e5068b },
	{ 0x5a6690ae, -0x2afad26a }, { 0x52beac9f, -0x17088531 }, { 0x55f104dc, -0x392a9643 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x5a6690ae, -0x2f6bbe45 }, { 0x5471e2e7, -0x19ef7944 }, { 0x50d86e6d, -0x3cc511d9 },
	{ 0x5a12e721, -0x317900d7 }, { 0x553805f3, -0x1b5d100a }, { 0x4da1fab5, -0x3e14fdf8 },
	{ 0x5987b08a, -0x3367c090 }, { 0x55f104dc, -0x1cc66e9a }, { 0x49ffd418, -0x3f0ec9f5 },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x57cc15bd, -0x36e5068b }, { 0x573b2635, -0x1f8ba4dc }, { 0x418d2621, -0x3ffb10c2 },
	{ 0x569cc31c, -0x387165e4 }, { 0x57cc15bd, -0x20e70f33 }, { 0x3cc85709, -0x3fec43c7 },
	{ 0x553805f3, -0x39daf5e9 }, { 0x584f7b59, -0x223d66a9 }, { 0x37af354d, -0x3f84c8e2 },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x51d1dc80, -0x3c42420a }, { 0x592d59db, -0x24da0a9a }, { 0x2c9caf6d, -0x3dae81cf },
	{ 0x4fd288dd, -0x3d3e82ae }, { 0x5987b08a, -0x261feffa }, { 0x26b2a795, -0x3c42420a },
	{ 0x4da1fab5, -0x3e14fdf8 }, { 0x59d438e6, -0x275ff453 }, { 0x2092f05f, -0x3a8269a3 },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x48b2b336, -0x3f4eaaff }, { 0x5a43b190, -0x29cd9578 }, { 0x13d4ae08, -0x361214b1 },
	{ 0x45f704f7, -0x3fb11b48 }, { 0x5a6690ae, -0x2afad26a }, { 0x0d47d097, -0x3367c090 },
	{ 0x43103086, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2c216eab }, { 0x06a886a1, -0x30761c18 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x3cc85709, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2e5a1070 }, {-0x06a886a1, -0x29cd9578 },
	{ 0x396b3199, -0x3fb11b48 }, { 0x5a6690ae, -0x2f6bbe45 }, {-0x0d47d097, -0x261feffa },
	{ 0x35eaa2c7, -0x3f4eaaff }, { 0x5a43b190, -0x30761c18 }, {-0x13d4ae08, -0x223d66a9 },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x2e88013a, -0x3e14fdf8 }, { 0x59d438e6, -0x32744494 }, {-0x2092f05f, -0x19ef7944 },
	{ 0x2aaa7c7f, -0x3d3e82ae }, { 0x5987b08a, -0x3367c090 }, {-0x26b2a795, -0x158f9a76 },
	{ 0x26b2a795, -0x3c42420a }, { 0x592d59db, -0x34534f41 }, {-0x2c9caf6d, -0x1111d263 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x1e7de5df, -0x39daf5e9 }, { 0x584f7b59, -0x361214b1 }, {-0x37af354d, -0x07d59396 },
	{ 0x1a4608ab, -0x387165e4 }, { 0x57cc15bd, -0x36e5068b }, {-0x3cc85709, -0x0323ecbf },
	{ 0x15fdf758, -0x36e5068b }, { 0x573b2635, -0x37af8159 }, {-0x418d2621,  0x01921560 },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x0d47d097, -0x3367c090 }, { 0x55f104dc, -0x392a9643 }, {-0x49ffd418,  0x0af10a23 },
	{ 0x08df1a8d, -0x317900d7 }, { 0x553805f3, -0x39daf5e9 }, {-0x4da1fab5,  0x0f8cfcbe },
	{ 0x0470ebdc, -0x2f6bbe45 }, { 0x5471e2e7, -0x3a8269a3 }, {-0x50d86e6d,  0x14135c95 },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x0470ebdc, -0x2afad26a }, { 0x52beac9f, -0x3bb6276e }, {-0x55f104dc,  0x1cc66e9a },
	{-0x08df1a8d, -0x2899e64b }, { 0x51d1dc80, -0x3c42420a }, {-0x57cc15bd,  0x20e70f33 },
	{-0x0d47d097, -0x261feffa }, { 0x50d86e6d, -0x3cc511d9 }, {-0x592d59db,  0x24da0a9a },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x15fdf758, -0x20e70f33 }, { 0x4ec05432, -0x3dae81cf }, {-0x5a7b7f1b,  0x2c216eab },
	{-0x1a4608ab, -0x1e2b5d39 }, { 0x4da1fab5, -0x3e14fdf8 }, {-0x5a6690ae,  0x2f6bbe45 },
	{-0x1e7de5df, -0x1b5d100a }, { 0x4c77a88f, -0x3e71e759 }, {-0x59d438e6,  0x32744494 },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x26b2a795, -0x158f9a76 }, { 0x49ffd418, -0x3f0ec9f5 }, {-0x573b2635,  0x37af8159 },
	{-0x2aaa7c7f, -0x1294062f }, { 0x48b2b336, -0x3f4eaaff }, {-0x553805f3,  0x39daf5e9 },
	{-0x2e88013a, -0x0f8cfcbe }, { 0x475a5c78, -0x3f84c8e2 }, {-0x52beac9f,  0x3bb6276e },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },
	{-0x35eaa2c7, -0x09640838 }, { 0x4488e37f, -0x3fd39b5b }, {-0x4c77a88f,  0x3e71e759 },
	{-0x396b3199, -0x0645e9b0 }, { 0x43103086, -0x3fec43c7 }, {-0x48b2b336,  0x3f4eaaff },
	{-0x3cc85709, -0x0323ecbf }, { 0x418d2621, -0x3ffb10c2 }, {-0x4488e37f,  0x3fd39b5b },

	{ 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 }, { 0x40000000,  0x00000000 },
	{ 0x40c7d2be, -0x00c90e90 }, { 0x406438d0, -0x006487c4 }, { 0x412accd5, -0x012d936c },
	{ 0x418d2621, -0x01921560 }, { 0x40c7d2be, -0x00c90e90 }, { 0x424ff28f, -0x025b0caf },
	{ 0x424ff28f, -0x025b0caf }, { 0x412accd5, -0x012d936c }, { 0x436f57c2, -0x038851a3 },
	{ 0x43103086, -0x0323ecbf }, { 0x418d2621, -0x01921560 }, { 0x4488e37f, -0x04b54825 },
	{ 0x43cdd89b, -0x03ecadd0 }, { 0x41eeddb0, -0x01f69374 }, { 0x459c7d5b, -0x05e1d61b },
	{ 0x4488e37f, -0x04b54825 }, { 0x424ff28f, -0x025b0caf }, { 0x46aa0d6d, -0x070de172 },
	{ 0x454149fd, -0x057db403 }, { 0x42b063d1, -0x02bf801b }, { 0x47b17c54, -0x08395024 },
	{ 0x45f704f7, -0x0645e9b0 }, { 0x43103086, -0x0323ecbf }, { 0x48b2b336, -0x09640838 },
	{ 0x46aa0d6d, -0x070de172 }, { 0x436f57c2, -0x038851a3 }, { 0x49ad9bc3, -0x0a8defc3 },
	{ 0x475a5c78, -0x07d59396 }, { 0x43cdd89b, -0x03ecadd0 }, { 0x4aa22037, -0x0bb6ecf0 },
	{ 0x4807eb4b, -0x089cf868 }, { 0x442bb227, -0x0451004e }, { 0x4b902b5d, -0x0cdee5fa },
	{ 0x48b2b336, -0x09640838 }, { 0x4488e37f, -0x04b54825 }, { 0x4c77a88f, -0x0e05c136 },
	{ 0x495aada3, -0x0a2abb59 }, { 0x44e56bbd, -0x0519845f }, { 0x4d5883b7, -0x0f2b6510 },
	{ 0x49ffd418, -0x0af10a23 }, { 0x454149fd, -0x057db403 }, { 0x4e32a957, -0x104fb80f },
	{ 0x4aa22037, -0x0bb6ecf0 }, { 0x459c7d5b, -0x05e1d61b }, { 0x4f06067f, -0x1172a0d8 },
	{ 0x4b418bbf, -0x0c7c5c1f }, { 0x45f704f7, -0x0645e9b0 }, { 0x4fd288dd, -0x1294062f },
	{ 0x4bde1089, -0x0d415013 }, { 0x4650dff2, -0x06a9edca }, { 0x50981eb1, -0x13b3cefb },
	{ 0x4c77a88f, -0x0e05c136 }, { 0x46aa0d6d, -0x070de172 }, { 0x5156b6d9, -0x14d1e243 },
	{ 0x4d0e4de2, -0x0ec9a7f3 }, { 0x47028c8d, -0x0771c3b3 }, { 0x520e40cc, -0x15ee2738 },
	{ 0x4da1fab5, -0x0f8cfcbe }, { 0x475a5c78, -0x07d59396 }, { 0x52beac9f, -0x17088531 },
	{ 0x4e32a957, -0x104fb80f }, { 0x47b17c54, -0x08395024 }, { 0x5367eb04, -0x1820e3b1 },
	{ 0x4ec05432, -0x1111d263 }, { 0x4807eb4b, -0x089cf868 }, { 0x5409ed4c, -0x19372a64 },
	{ 0x4f4af5d2, -0x11d34440 }, { 0x485da888, -0x09008b6b }, { 0x54a4a56a, -0x1a4b4128 },
	{ 0x4fd288dd, -0x1294062f }, { 0x48b2b336, -0x09640838 }, { 0x553805f3, -0x1b5d100a },
	{ 0x5057081a, -0x135410c3 }, { 0x49070a84, -0x09c76dd9 }, { 0x55c4021d, -0x1c6c7f4a },
	{ 0x50d86e6d, -0x14135c95 }, { 0x495aada3, -0x0a2abb59 }, { 0x56488dc5, -0x1d79775c },
	{ 0x5156b6d9, -0x14d1e243 }, { 0x49ad9bc3, -0x0a8defc3 }, { 0x56c59d6b, -0x1e83e0eb },
	{ 0x51d1dc80, -0x158f9a76 }, { 0x49ffd418, -0x0af10a23 }, { 0x573b2635, -0x1f8ba4dc },
	{ 0x5249daa2, -0x164c7dde }, { 0x4a5155d7, -0x0b540983 }, { 0x57a91df3, -0x2090ac4e },
	{ 0x52beac9f, -0x17088531 }, { 0x4aa22037, -0x0bb6ecf0 }, { 0x580f7b19, -0x2192e09b },
	{ 0x53304df6, -0x17c3a932 }, { 0x4af23271, -0x0c19b375 }, { 0x586e34c7, -0x22922b5f },
	{ 0x539eba46, -0x187de2a7 }, { 0x4b418bbf, -0x0c7c5c1f }, { 0x58c542c6, -0x238e7674 },
	{ 0x5409ed4c, -0x19372a64 }, { 0x4b902b5d, -0x0cdee5fa }, { 0x59149d88, -0x2487abf8 },
	{ 0x5471e2e7, -0x19ef7944 }, { 0x4bde1089, -0x0d415013 }, { 0x595c3e2b, -0x257db64c },
	{ 0x54d69714, -0x1aa6c82c }, { 0x4c2b3a84, -0x0da39978 }, { 0x599c1e78, -0x2670801b },
	{ 0x553805f3, -0x1b5d100a }, { 0x4c77a88f, -0x0e05c136 }, { 0x59d438e6, -0x275ff453 },
	{ 0x55962bc0, -0x1c1249d9 }, { 0x4cc359ec, -0x0e67c65a }, { 0x5a048896, -0x284bfe30 },
	{ 0x55f104dc, -0x1cc66e9a }, { 0x4d0e4de2, -0x0ec9a7f3 }, { 0x5a2d0957, -0x29348938 },
	{ 0x56488dc5, -0x1d79775c }, { 0x4d5883b7, -0x0f2b6510 }, { 0x5a4db7a7, -0x2a19813f },
	{ 0x569cc31c, -0x1e2b5d39 }, { 0x4da1fab5, -0x0f8cfcbe }, { 0x5a6690ae, -0x2afad26a },
	{ 0x56eda1a0, -0x1edc1953 }, { 0x4deab226, -0x0fee6e0e }, { 0x5a779247, -0x2bd8692c },
	{ 0x573b2635, -0x1f8ba4dc }, { 0x4e32a957, -0x104fb80f }, { 0x5a80baf6, -0x2cb2324c },
	{ 0x57854dde, -0x2039f90f }, { 0x4e79df95, -0x10b0d9d0 }, { 0x5a8209f1, -0x2d881ae8 },
	{ 0x57cc15bd, -0x20e70f33 }, { 0x4ec05432, -0x1111d263 }, { 0x5a7b7f1b, -0x2e5a1070 },
	{ 0x580f7b19, -0x2192e09b }, { 0x4f06067f, -0x1172a0d8 }, { 0x5a6d1b04, -0x2f2800af },
	{ 0x584f7b59, -0x223d66a9 }, { 0x4f4af5d2, -0x11d34440 }, { 0x5a56deec, -0x2ff1d9c7 },
	{ 0x588c1405, -0x22e69ac8 }, { 0x4f8f217e, -0x1233bbac }, { 0x5a38ccc2, -0x30b78a36 },
	{ 0x58c542c6, -0x238e7674 }, { 0x4fd288dd, -0x1294062f }, { 0x5a12e721, -0x317900d7 },
	{ 0x58fb0569, -0x2434f333 }, { 0x50152b48, -0x12f422db }, { 0x59e53152, -0x32362ce0 },
	{ 0x592d59db, -0x24da0a9a }, { 0x5057081a, -0x135410c3 }, { 0x59afaf4c, -0x32eefdea },
	{ 0x595c3e2b, -0x257db64c }, { 0x50981eb1, -0x13b3cefb }, { 0x597265b4, -0x33a363ec },
	{ 0x5987b08a, -0x261feffa }, { 0x50d86e6d, -0x14135c95 }, { 0x592d59db, -0x34534f41 },
	{ 0x59afaf4c, -0x26c0b163 }, { 0x5117f6af, -0x1472b8a6 }, { 0x58e091bd, -0x34feb0a6 },
	{ 0x59d438e6, -0x275ff453 }, { 0x5156b6d9, -0x14d1e243 }, { 0x588c1405, -0x35a5793d },
	{ 0x59f54bef, -0x27fdb2a7 }, { 0x5194ae52, -0x1530d881 }, { 0x582fe805, -0x36479a8f },
	{ 0x5a12e721, -0x2899e64b }, { 0x51d1dc80, -0x158f9a76 }, { 0x57cc15bd, -0x36e5068b },
	{ 0x5a2d0957, -0x29348938 }, { 0x520e40cc, -0x15ee2738 }, { 0x5760a5d6, -0x377daf8a },
	{ 0x5a43b190, -0x29cd9578 }, { 0x5249daa2, -0x164c7dde }, { 0x56eda1a0, -0x3811884d },
	{ 0x5a56deec, -0x2a650526 }, { 0x5284a96e, -0x16aa9d7e }, { 0x56731317, -0x38a08403 },
	{ 0x5a6690ae, -0x2afad26a }, { 0x52beac9f, -0x17088531 }, { 0x55f104dc, -0x392a9643 },
	{ 0x5a72c63b, -0x2b8ef77d }, { 0x52f7e3a6, -0x17663410 }, { 0x55678237, -0x39afb314 },
	{ 0x5a7b7f1b, -0x2c216eab }, { 0x53304df6, -0x17c3a932 }, { 0x54d69714, -0x3a2fcee9 },
	{ 0x5a80baf6, -0x2cb2324c }, { 0x5367eb04, -0x1820e3b1 }, { 0x543e5008, -0x3aaadea6 },
	{ 0x5a82799a, -0x2d413ccd }, { 0x539eba46, -0x187de2a7 }, { 0x539eba46, -0x3b20d79f },
	{ 0x5a80baf6, -0x2dce88aa }, { 0x53d4bb35, -0x18daa52f }, { 0x52f7e3a6, -0x3b91af97 },
	{ 0x5a7b7f1b, -0x2e5a1070 }, { 0x5409ed4c, -0x19372a64 }, { 0x5249daa2, -0x3bfd5cc5 },
	{ 0x5a72c63b, -0x2ee3cebf }, { 0x543e5008, -0x19937162 }, { 0x5194ae52, -0x3c63d5d1 },
	{ 0x5a6690ae, -0x2f6bbe45 }, { 0x5471e2e7, -0x19ef7944 }, { 0x50d86e6d, -0x3cc511d9 },
	{ 0x5a56deec, -0x2ff1d9c7 }, { 0x54a4a56a, -0x1a4b4128 }, { 0x50152b48, -0x3d21086d },
	{ 0x5a43b190, -0x30761c18 }, { 0x54d69714, -0x1aa6c82c }, { 0x4f4af5d2, -0x3d77b192 },
	{ 0x5a2d0957, -0x30f88020 }, { 0x5507b76a, -0x1b020d6d }, { 0x4e79df95, -0x3dc905c5 },
	{ 0x5a12e721, -0x317900d7 }, { 0x553805f3, -0x1b5d100a }, { 0x4da1fab5, -0x3e14fdf8 },
	{ 0x59f54bef, -0x31f79948 }, { 0x55678237, -0x1bb7cf24 }, { 0x4cc359ec, -0x3e5b9393 },
	{ 0x59d438e6, -0x32744494 }, { 0x55962bc0, -0x1c1249d9 }, { 0x4bde1089, -0x3e9cc077 },
	{ 0x59afaf4c, -0x32eefdea }, { 0x55c4021d, -0x1c6c7f4a }, { 0x4af23271, -0x3ed87efc },
	{ 0x5987b08a, -0x3367c090 }, { 0x55f104dc, -0x1cc66e9a }, { 0x49ffd418, -0x3f0ec9f5 },
	{ 0x595c3e2b, -0x33de87df }, { 0x561d338e, -0x1d2016e9 }, { 0x49070a84, -0x3f3f9cac },
	{ 0x592d59db, -0x34534f41 }, { 0x56488dc5, -0x1d79775c }, { 0x4807eb4b, -0x3f6af2e4 },
	{ 0x58fb0569, -0x34c61237 }, { 0x56731317, -0x1dd28f15 }, { 0x47028c8d, -0x3f90c8da },
	{ 0x58c542c6, -0x3536cc53 }, { 0x569cc31c, -0x1e2b5d39 }, { 0x45f704f7, -0x3fb11b48 },
	{ 0x588c1405, -0x35a5793d }, { 0x56c59d6b, -0x1e83e0eb }, { 0x44e56bbd, -0x3fcbe75f },
	{ 0x584f7b59, -0x361214b1 }, { 0x56eda1a0, -0x1edc1953 }, { 0x43cdd89b, -0x3fe12acc },
	{ 0x580f7b19, -0x367c9a7e }, { 0x5714cf59, -0x1f340597 }, { 0x42b063d1, -0x3ff0e3b6 },
	{ 0x57cc15bd, -0x36e5068b }, { 0x573b2635, -0x1f8ba4dc }, { 0x418d2621, -0x3ffb10c2 },
	{ 0x57854dde, -0x374b54cf }, { 0x5760a5d6, -0x1fe2f64c }, { 0x406438d0, -0x3fffb10c },
	{ 0x573b2635, -0x37af8159 }, { 0x57854dde, -0x2039f90f }, { 0x3f35b59e, -0x3ffec42e },
	{ 0x56eda1a0, -0x3811884d }, { 0x57a91df3, -0x2090ac4e }, { 0x3e01b6c9, -0x3ff84a3c },
	{ 0x569cc31c, -0x387165e4 }, { 0x57cc15bd, -0x20e70f33 }, { 0x3cc85709, -0x3fec43c7 },
	{ 0x56488dc5, -0x38cf166a }, { 0x57ee34e6, -0x213d20e9 }, { 0x3b89b18d, -0x3fdab1da },
	{ 0x55f104dc, -0x392a9643 }, { 0x580f7b19, -0x2192e09b }, { 0x3a45e1f7, -0x3fc395fa },
	{ 0x55962bc0, -0x3983e1e8 }, { 0x582fe805, -0x21e84d77 }, { 0x38fd0460, -0x3fa6f229 },
	{ 0x553805f3, -0x39daf5e9 }, { 0x584f7b59, -0x223d66a9 }, { 0x37af354d, -0x3f84c8e2 },
	{ 0x54d69714, -0x3a2fcee9 }, { 0x586e34c7, -0x22922b5f }, { 0x365c91b3, -0x3f5d1d1d },
	{ 0x5471e2e7, -0x3a8269a3 }, { 0x588c1405, -0x22e69ac8 }, { 0x350536f1, -0x3f2ff24a },
	{ 0x5409ed4c, -0x3ad2c2e8 }, { 0x58a918c7, -0x233ab414 }, { 0x33a942d2, -0x3efd4c54 },
	{ 0x539eba46, -0x3b20d79f }, { 0x58c542c6, -0x238e7674 }, { 0x3248d382, -0x3ec52fa0 },
	{ 0x53304df6, -0x3b6ca4c5 }, { 0x58e091bd, -0x23e1e118 }, { 0x30e40795, -0x3e87a10c },
	{ 0x52beac9f, -0x3bb6276e }, { 0x58fb0569, -0x2434f333 }, { 0x2f7afdfd, -0x3e44a5ef },
	{ 0x5249daa2, -0x3bfd5cc5 }, { 0x59149d88, -0x2487abf8 }, { 0x2e0dd60b, -0x3dfc4419 },
	{ 0x51d1dc80, -0x3c42420a }, { 0x592d59db, -0x24da0a9a }, { 0x2c9caf6d, -0x3dae81cf },
	{ 0x5156b6d9, -0x3c84d497 }, { 0x59453a25, -0x252c0e4f }, { 0x2b27aa27, -0x3d5b65d2 },
	{ 0x50d86e6d, -0x3cc511d9 }, { 0x595c3e2b, -0x257db64c }, { 0x29aee694, -0x3d02f757 },
	{ 0x5057081a, -0x3d02f757 }, { 0x597265b4, -0x25cf01c8 }, { 0x28328564, -0x3ca53e09 },
	{ 0x4fd288dd, -0x3d3e82ae }, { 0x5987b08a, -0x261feffa }, { 0x26b2a795, -0x3c42420a },
	{ 0x4f4af5d2, -0x3d77b192 }, { 0x599c1e78, -0x2670801b }, { 0x252f6e72, -0x3bda0bf0 },
	{ 0x4ec05432, -0x3dae81cf }, { 0x59afaf4c, -0x26c0b163 }, { 0x23a8fb94, -0x3b6ca4c5 },
	{ 0x4e32a957, -0x3de2f148 }, { 0x59c262d5, -0x2710830c }, { 0x221f70d7, -0x3afa1606 },
	{ 0x4da1fab5, -0x3e14fdf8 }, { 0x59d438e6, -0x275ff453 }, { 0x2092f05f, -0x3a8269a3 },
	{ 0x4d0e4de2, -0x3e44a5ef }, { 0x59e53152, -0x27af0472 }, { 0x1f039c91, -0x3a05a9fe },
	{ 0x4c77a88f, -0x3e71e759 }, { 0x59f54bef, -0x27fdb2a7 }, { 0x1d719810, -0x3983e1e8 },
	{ 0x4bde1089, -0x3e9cc077 }, { 0x5a048896, -0x284bfe30 }, { 0x1bdd05bc, -0x38fd1ca5 },
	{ 0x4b418bbf, -0x3ec52fa0 }, { 0x5a12e721, -0x2899e64b }, { 0x1a4608ab, -0x387165e4 },
	{ 0x4aa22037, -0x3eeb3348 }, { 0x5a20676c, -0x28e76a38 }, { 0x18acc42d, -0x37e0c9c3 },
	{ 0x49ffd418, -0x3f0ec9f5 }, { 0x5a2d0957, -0x29348938 }, { 0x17115bc0, -0x374b54cf },
	{ 0x495aada3, -0x3f2ff24a }, { 0x5a38ccc2, -0x2981428c }, { 0x1573f315, -0x36b113fe },
	{ 0x48b2b336, -0x3f4eaaff }, { 0x5a43b190, -0x29cd9578 }, { 0x13d4ae08, -0x361214b1 },
	{ 0x4807eb4b, -0x3f6af2e4 }, { 0x5a4db7a7, -0x2a19813f }, { 0x1233b09f, -0x356e64b3 },
	{ 0x475a5c78, -0x3f84c8e2 }, { 0x5a56deec, -0x2a650526 }, { 0x10911f04, -0x34c61237 },
	{ 0x46aa0d6d, -0x3f9c2bfb }, { 0x5a5f274b, -0x2ab02072 }, { 0x0eed1d87, -0x34192bd6 },
	{ 0x45f704f7, -0x3fb11b48 }, { 0x5a6690ae, -0x2afad26a }, { 0x0d47d097, -0x3367c090 },
	{ 0x454149fd, -0x3fc395fa }, { 0x5a6d1b04, -0x2b451a55 }, { 0x0ba15cbe, -0x32b1dfca },
	{ 0x4488e37f, -0x3fd39b5b }, { 0x5a72c63b, -0x2b8ef77d }, { 0x09f9e6a2, -0x31f79948 },
	{ 0x43cdd89b, -0x3fe12acc }, { 0x5a779247, -0x2bd8692c }, { 0x085192fe, -0x3138fd35 },
	{ 0x43103086, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2c216eab }, { 0x06a886a1, -0x30761c18 },
	{ 0x424ff28f, -0x3ff4e5e0 }, { 0x5a7e8cad, -0x2c6a0747 }, { 0x04fee669, -0x2faf06da },
	{ 0x418d2621, -0x3ffb10c2 }, { 0x5a80baf6, -0x2cb2324c }, { 0x0354d742, -0x2ee3cebf },
	{ 0x40c7d2be, -0x3ffec42e }, { 0x5a8209f1, -0x2cf9ef0a }, { 0x01aa7e20, -0x2e148567 },
	{ 0x40000000, -0x40000000 }, { 0x5a82799a, -0x2d413ccd }, { 0x00000000, -0x2d413ccd },
	{ 0x3f35b59e, -0x3ffec42e }, { 0x5a8209f1, -0x2d881ae8 }, {-0x01aa7e20, -0x2c6a0747 },
	{ 0x3e68fb62, -0x3ffb10c2 }, { 0x5a80baf6, -0x2dce88aa }, {-0x0354d742, -0x2b8ef77d },
	{ 0x3d99d932, -0x3ff4e5e0 }, { 0x5a7e8cad, -0x2e148567 }, {-0x04fee669, -0x2ab02072 },
	{ 0x3cc85709, -0x3fec43c7 }, { 0x5a7b7f1b, -0x2e5a1070 }, {-0x06a886a1, -0x29cd9578 },
	{ 0x3bf47cfc, -0x3fe12acc }, { 0x5a779247, -0x2e9f291c }, {-0x085192fe, -0x28e76a38 },
	{ 0x3b1e5336, -0x3fd39b5b }, { 0x5a72c63b, -0x2ee3cebf }, {-0x09f9e6a2, -0x27fdb2a7 },
	{ 0x3a45e1f7, -0x3fc395fa }, { 0x5a6d1b04, -0x2f2800af }, {-0x0ba15cbe, -0x2710830c },
	{ 0x396b3199, -0x3fb11b48 }, { 0x5a6690ae, -0x2f6bbe45 }, {-0x0d47d097, -0x261feffa },
	{ 0x388e4a89, -0x3f9c2bfb }, { 0x5a5f274b, -0x2faf06da }, {-0x0eed1d87, -0x252c0e4f },
	{ 0x37af354d, -0x3f84c8e2 }, { 0x5a56deec, -0x2ff1d9c7 }, {-0x10911f04, -0x2434f333 },
	{ 0x36cdfa7c, -0x3f6af2e4 }, { 0x5a4db7a7, -0x30343668 }, {-0x1233b09f, -0x233ab414 },
	{ 0x35eaa2c7, -0x3f4eaaff }, { 0x5a43b190, -0x30761c18 }, {-0x13d4ae08, -0x223d66a9 },
	{ 0x350536f1, -0x3f2ff24a }, { 0x5a38ccc2, -0x30b78a36 }, {-0x1573f315, -0x213d20e9 },
	{ 0x341dbfd3, -0x3f0ec9f5 }, { 0x5a2d0957, -0x30f88020 }, {-0x17115bc0, -0x2039f90f },
	{ 0x33344659, -0x3eeb3348 }, { 0x5a20676c, -0x3138fd35 }, {-0x18acc42d, -0x1f340597 },
	{ 0x3248d382, -0x3ec52fa0 }, { 0x5a12e721, -0x317900d7 }, {-0x1a4608ab, -0x1e2b5d39 },
	{ 0x315b7064, -0x3e9cc077 }, { 0x5a048896, -0x31b88a67 }, {-0x1bdd05bc, -0x1d2016e9 },
	{ 0x306c2624, -0x3e71e759 }, { 0x59f54bef, -0x31f79948 }, {-0x1d719810, -0x1c1249d9 },
	{ 0x2f7afdfd, -0x3e44a5ef }, { 0x59e53152, -0x32362ce0 }, {-0x1f039c91, -0x1b020d6d },
	{ 0x2e88013a, -0x3e14fdf8 }, { 0x59d438e6, -0x32744494 }, {-0x2092f05f, -0x19ef7944 },
	{ 0x2d93393a, -0x3de2f148 }, { 0x59c262d5, -0x32b1dfca }, {-0x221f70d7, -0x18daa52f },
	{ 0x2c9caf6d, -0x3dae81cf }, { 0x59afaf4c, -0x32eefdea }, {-0x23a8fb94, -0x17c3a932 },
	{ 0x2ba46d53, -0x3d77b192 }, { 0x599c1e78, -0x332b9e5e }, {-0x252f6e72, -0x16aa9d7e },
	{ 0x2aaa7c7f, -0x3d3e82ae }, { 0x5987b08a, -0x3367c090 }, {-0x26b2a795, -0x158f9a76 },
	{ 0x29aee694, -0x3d02f757 }, { 0x597265b4, -0x33a363ec }, {-0x28328564, -0x1472b8a6 },
	{ 0x28b1b545, -0x3cc511d9 }, { 0x595c3e2b, -0x33de87df }, {-0x29aee694, -0x135410c3 },
	{ 0x27b2f254, -0x3c84d497 }, { 0x59453a25, -0x34192bd6 }, {-0x2b27aa27, -0x1233bbac },
	{ 0x26b2a795, -0x3c42420a }, { 0x592d59db, -0x34534f41 }, {-0x2c9caf6d, -0x1111d263 },
	{ 0x25b0dee8, -0x3bfd5cc5 }, { 0x59149d88, -0x348cf191 }, {-0x2e0dd60b, -0x0fee6e0e },
	{ 0x24ada23d, -0x3bb6276e }, { 0x58fb0569, -0x34c61237 }, {-0x2f7afdfd, -0x0ec9a7f3 },
	{ 0x23a8fb94, -0x3b6ca4c5 }, { 0x58e091bd, -0x34feb0a6 }, {-0x30e40795, -0x0da39978 },
	{ 0x22a2f4f8, -0x3b20d79f }, { 0x58c542c6, -0x3536cc53 }, {-0x3248d382, -0x0c7c5c1f },
	{ 0x219b9884, -0x3ad2c2e8 }, { 0x58a918c7, -0x356e64b3 }, {-0x33a942d2, -0x0b540983 },
	{ 0x2092f05f, -0x3a8269a3 }, { 0x588c1405, -0x35a5793d }, {-0x350536f1, -0x0a2abb59 },
	{ 0x1f8906be, -0x3a2fcee9 }, { 0x586e34c7, -0x35dc0969 }, {-0x365c91b3, -0x09008b6b },
	{ 0x1e7de5df, -0x39daf5e9 }, { 0x584f7b59, -0x361214b1 }, {-0x37af354d, -0x07d59396 },
	{ 0x1d719810, -0x3983e1e8 }, { 0x582fe805, -0x36479a8f }, {-0x38fd0460, -0x06a9edca },
	{ 0x1c6427aa, -0x392a9643 }, { 0x580f7b19, -0x367c9a7e }, {-0x3a45e1f7, -0x057db403 },
	{ 0x1b559f0e, -0x38cf166a }, { 0x57ee34e6, -0x36b113fe }, {-0x3b89b18d, -0x0451004e },
	{ 0x1a4608ab, -0x387165e4 }, { 0x57cc15bd, -0x36e5068b }, {-0x3cc85709, -0x0323ecbf },
	{ 0x19356efa, -0x3811884d }, { 0x57a91df3, -0x371871a5 }, {-0x3e01b6c9, -0x01f69374 },
	{ 0x1823dc7d, -0x37af8159 }, { 0x57854dde, -0x374b54cf }, {-0x3f35b59e, -0x00c90e90 },
	{ 0x17115bc0, -0x374b54cf }, { 0x5760a5d6, -0x377daf8a }, {-0x406438d0,  0x006487c4 },
	{ 0x15fdf758, -0x36e5068b }, { 0x573b2635, -0x37af8159 }, {-0x418d2621,  0x01921560 },
	{ 0x14e9b9e4, -0x367c9a7e }, { 0x5714cf59, -0x37e0c9c3 }, {-0x42b063d1,  0x02bf801b },
	{ 0x13d4ae08, -0x361214b1 }, { 0x56eda1a0, -0x3811884d }, {-0x43cdd89b,  0x03ecadd0 },
	{ 0x12bede75, -0x35a5793d }, { 0x56c59d6b, -0x3841bc80 }, {-0x44e56bbd,  0x0519845f },
	{ 0x11a855df, -0x3536cc53 }, { 0x569cc31c, -0x387165e4 }, {-0x45f704f7,  0x0645e9b0 },
	{ 0x10911f04, -0x34c61237 }, { 0x56731317, -0x38a08403 }, {-0x47028c8d,  0x0771c3b3 },
	{ 0x0f7944a7, -0x34534f41 }, { 0x56488dc5, -0x38cf166a }, {-0x4807eb4b,  0x089cf868 },
	{ 0x0e60d193, -0x33de87df }, { 0x561d338e, -0x38fd1ca5 }, {-0x49070a84,  0x09c76dd9 },
	{ 0x0d47d097, -0x3367c090 }, { 0x55f104dc, -0x392a9643 }, {-0x49ffd418,  0x0af10a23 },
	{ 0x0c2e4c88, -0x32eefdea }, { 0x55c4021d, -0x395782d4 }, {-0x4af23271,  0x0c19b375 },
	{ 0x0b145041, -0x32744494 }, { 0x55962bc0, -0x3983e1e8 }, {-0x4bde1089,  0x0d415013 },
	{ 0x09f9e6a2, -0x31f79948 }, { 0x55678237, -0x39afb314 }, {-0x4cc359ec,  0x0e67c65a },
	{ 0x08df1a8d, -0x317900d7 }, { 0x553805f3, -0x39daf5e9 }, {-0x4da1fab5,  0x0f8cfcbe },
	{ 0x07c3f6e9, -0x30f88020 }, { 0x5507b76a, -0x3a05a9fe }, {-0x4e79df95,  0x10b0d9d0 },
	{ 0x06a886a1, -0x30761c18 }, { 0x54d69714, -0x3a2fcee9 }, {-0x4f4af5d2,  0x11d34440 },
	{ 0x058cd4a2, -0x2ff1d9c7 }, { 0x54a4a56a, -0x3a596442 }, {-0x50152b48,  0x12f422db },
	{ 0x0470ebdc, -0x2f6bbe45 }, { 0x5471e2e7, -0x3a8269a3 }, {-0x50d86e6d,  0x14135c95 },
	{ 0x0354d742, -0x2ee3cebf }, { 0x543e5008, -0x3aaadea6 }, {-0x5194ae52,  0x1530d881 },
	{ 0x0238a1c6, -0x2e5a1070 }, { 0x5409ed4c, -0x3ad2c2e8 }, {-0x5249daa2,  0x164c7dde },
	{ 0x011c565e, -0x2dce88aa }, { 0x53d4bb35, -0x3afa1606 }, {-0x52f7e3a6,  0x17663410 },
	{ 0x00000000, -0x2d413ccd }, { 0x539eba46, -0x3b20d79f }, {-0x539eba46,  0x187de2a7 },
	{-0x011c565e, -0x2cb2324c }, { 0x5367eb04, -0x3b470753 }, {-0x543e5008,  0x19937162 },
	{-0x0238a1c6, -0x2c216eab }, { 0x53304df6, -0x3b6ca4c5 }, {-0x54d69714,  0x1aa6c82c },
	{-0x0354d742, -0x2b8ef77d }, { 0x52f7e3a6, -0x3b91af97 }, {-0x55678237,  0x1bb7cf24 },
	{-0x0470ebdc, -0x2afad26a }, { 0x52beac9f, -0x3bb6276e }, {-0x55f104dc,  0x1cc66e9a },
	{-0x058cd4a2, -0x2a650526 }, { 0x5284a96e, -0x3bda0bf0 }, {-0x56731317,  0x1dd28f15 },
	{-0x06a886a1, -0x29cd9578 }, { 0x5249daa2, -0x3bfd5cc5 }, {-0x56eda1a0,  0x1edc1953 },
	{-0x07c3f6e9, -0x29348938 }, { 0x520e40cc, -0x3c201995 }, {-0x5760a5d6,  0x1fe2f64c },
	{-0x08df1a8d, -0x2899e64b }, { 0x51d1dc80, -0x3c42420a }, {-0x57cc15bd,  0x20e70f33 },
	{-0x09f9e6a2, -0x27fdb2a7 }, { 0x5194ae52, -0x3c63d5d1 }, {-0x582fe805,  0x21e84d77 },
	{-0x0b145041, -0x275ff453 }, { 0x5156b6d9, -0x3c84d497 }, {-0x588c1405,  0x22e69ac8 },
	{-0x0c2e4c88, -0x26c0b163 }, { 0x5117f6af, -0x3ca53e09 }, {-0x58e091bd,  0x23e1e118 },
	{-0x0d47d097, -0x261feffa }, { 0x50d86e6d, -0x3cc511d9 }, {-0x592d59db,  0x24da0a9a },
	{-0x0e60d193, -0x257db64c }, { 0x50981eb1, -0x3ce44fb7 }, {-0x597265b4,  0x25cf01c8 },
	{-0x0f7944a7, -0x24da0a9a }, { 0x5057081a, -0x3d02f757 }, {-0x59afaf4c,  0x26c0b163 },
	{-0x10911f04, -0x2434f333 }, { 0x50152b48, -0x3d21086d }, {-0x59e53152,  0x27af0472 },
	{-0x11a855df, -0x238e7674 }, { 0x4fd288dd, -0x3d3e82ae }, {-0x5a12e721,  0x2899e64b },
	{-0x12bede75, -0x22e69ac8 }, { 0x4f8f217e, -0x3d5b65d2 }, {-0x5a38ccc2,  0x2981428c },
	{-0x13d4ae08, -0x223d66a9 }, { 0x4f4af5d2, -0x3d77b192 }, {-0x5a56deec,  0x2a650526 },
	{-0x14e9b9e4, -0x2192e09b }, { 0x4f06067f, -0x3d9365a8 }, {-0x5a6d1b04,  0x2b451a55 },
	{-0x15fdf758, -0x20e70f33 }, { 0x4ec05432, -0x3dae81cf }, {-0x5a7b7f1b,  0x2c216eab },
	{-0x17115bc0, -0x2039f90f }, { 0x4e79df95, -0x3dc905c5 }, {-0x5a8209f1,  0x2cf9ef0a },
	{-0x1823dc7d, -0x1f8ba4dc }, { 0x4e32a957, -0x3de2f148 }, {-0x5a80baf6,  0x2dce88aa },
	{-0x19356efa, -0x1edc1953 }, { 0x4deab226, -0x3dfc4419 }, {-0x5a779247,  0x2e9f291c },
	{-0x1a4608ab, -0x1e2b5d39 }, { 0x4da1fab5, -0x3e14fdf8 }, {-0x5a6690ae,  0x2f6bbe45 },
	{-0x1b559f0e, -0x1d79775c }, { 0x4d5883b7, -0x3e2d1ea8 }, {-0x5a4db7a7,  0x30343668 },
	{-0x1c6427aa, -0x1cc66e9a }, { 0x4d0e4de2, -0x3e44a5ef }, {-0x5a2d0957,  0x30f88020 },
	{-0x1d719810, -0x1c1249d9 }, { 0x4cc359ec, -0x3e5b9393 }, {-0x5a048896,  0x31b88a67 },
	{-0x1e7de5df, -0x1b5d100a }, { 0x4c77a88f, -0x3e71e759 }, {-0x59d438e6,  0x32744494 },
	{-0x1f8906be, -0x1aa6c82c }, { 0x4c2b3a84, -0x3e87a10c }, {-0x599c1e78,  0x332b9e5e },
	{-0x2092f05f, -0x19ef7944 }, { 0x4bde1089, -0x3e9cc077 }, {-0x595c3e2b,  0x33de87df },
	{-0x219b9884, -0x19372a64 }, { 0x4b902b5d, -0x3eb14563 }, {-0x59149d88,  0x348cf191 },
	{-0x22a2f4f8, -0x187de2a7 }, { 0x4b418bbf, -0x3ec52fa0 }, {-0x58c542c6,  0x3536cc53 },
	{-0x23a8fb94, -0x17c3a932 }, { 0x4af23271, -0x3ed87efc }, {-0x586e34c7,  0x35dc0969 },
	{-0x24ada23d, -0x17088531 }, { 0x4aa22037, -0x3eeb3348 }, {-0x580f7b19,  0x367c9a7e },
	{-0x25b0dee8, -0x164c7dde }, { 0x4a5155d7, -0x3efd4c54 }, {-0x57a91df3,  0x371871a5 },
	{-0x26b2a795, -0x158f9a76 }, { 0x49ffd418, -0x3f0ec9f5 }, {-0x573b2635,  0x37af8159 },
	{-0x27b2f254, -0x14d1e243 }, { 0x49ad9bc3, -0x3f1fac00 }, {-0x56c59d6b,  0x3841bc80 },
	{-0x28b1b545, -0x14135c95 }, { 0x495aada3, -0x3f2ff24a }, {-0x56488dc5,  0x38cf166a },
	{-0x29aee694, -0x135410c3 }, { 0x49070a84, -0x3f3f9cac }, {-0x55c4021d,  0x395782d4 },
	{-0x2aaa7c7f, -0x1294062f }, { 0x48b2b336, -0x3f4eaaff }, {-0x553805f3,  0x39daf5e9 },
	{-0x2ba46d53, -0x11d34440 }, { 0x485da888, -0x3f5d1d1d }, {-0x54a4a56a,  0x3a596442 },
	{-0x2c9caf6d, -0x1111d263 }, { 0x4807eb4b, -0x3f6af2e4 }, {-0x5409ed4c,  0x3ad2c2e8 },
	{-0x2d93393a, -0x104fb80f }, { 0x47b17c54, -0x3f782c30 }, {-0x5367eb04,  0x3b470753 },
	{-0x2e88013a, -0x0f8cfcbe }, { 0x475a5c78, -0x3f84c8e2 }, {-0x52beac9f,  0x3bb6276e },
	{-0x2f7afdfd, -0x0ec9a7f3 }, { 0x47028c8d, -0x3f90c8da }, {-0x520e40cc,  0x3c201995 },
	{-0x306c2624, -0x0e05c136 }, { 0x46aa0d6d, -0x3f9c2bfb }, {-0x5156b6d9,  0x3c84d497 },
	{-0x315b7064, -0x0d415013 }, { 0x4650dff2, -0x3fa6f229 }, {-0x50981eb1,  0x3ce44fb7 },
	{-0x3248d382, -0x0c7c5c1f }, { 0x45f704f7, -0x3fb11b48 }, {-0x4fd288dd,  0x3d3e82ae },
	{-0x33344659, -0x0bb6ecf0 }, { 0x459c7d5b, -0x3fbaa740 }, {-0x4f06067f,  0x3d9365a8 },
	{-0x341dbfd3, -0x0af10a23 }, { 0x454149fd, -0x3fc395fa }, {-0x4e32a957,  0x3de2f148 },
	{-0x350536f1, -0x0a2abb59 }, { 0x44e56bbd, -0x3fcbe75f }, {-0x4d5883b7,  0x3e2d1ea8 },
	{-0x35eaa2c7, -0x09640838 }, { 0x4488e37f, -0x3fd39b5b }, {-0x4c77a88f,  0x3e71e759 },
	{-0x36cdfa7c, -0x089cf868 }, { 0x442bb227, -0x3fdab1da }, {-0x4b902b5d,  0x3eb14563 },
	{-0x37af354d, -0x07d59396 }, { 0x43cdd89b, -0x3fe12acc }, {-0x4aa22037,  0x3eeb3348 },
	{-0x388e4a89, -0x070de172 }, { 0x436f57c2, -0x3fe70620 }, {-0x49ad9bc3,  0x3f1fac00 },
	{-0x396b3199, -0x0645e9b0 }, { 0x43103086, -0x3fec43c7 }, {-0x48b2b336,  0x3f4eaaff },
	{-0x3a45e1f7, -0x057db403 }, { 0x42b063d1, -0x3ff0e3b6 }, {-0x47b17c54,  0x3f782c30 },
	{-0x3b1e5336, -0x04b54825 }, { 0x424ff28f, -0x3ff4e5e0 }, {-0x46aa0d6d,  0x3f9c2bfb },
	{-0x3bf47cfc, -0x03ecadd0 }, { 0x41eeddb0, -0x3ff84a3c }, {-0x459c7d5b,  0x3fbaa740 },
	{-0x3cc85709, -0x0323ecbf }, { 0x418d2621, -0x3ffb10c2 }, {-0x4488e37f,  0x3fd39b5b },
	{-0x3d99d932, -0x025b0caf }, { 0x412accd5, -0x3ffd3969 }, {-0x436f57c2,  0x3fe70620 },
	{-0x3e68fb62, -0x01921560 }, { 0x40c7d2be, -0x3ffec42e }, {-0x424ff28f,  0x3ff4e5e0 },
	{-0x3f35b59e, -0x00c90e90 }, { 0x406438d0, -0x3fffb10c }, {-0x412accd5,  0x3ffd3969 },
    };

    struct i4b_complex *data_end;
    const uint16_t *p;

    /* In-place index bit-reversal */
    p = bit_rev;
    while (*p) {
      int32_t t;
      uint16_t a,b;

      a = *(p + 0);
      b = *(p + 1);
      t = data[b].x;
      data[b].x = data[a].x;
      data[a].x = t;

      t = data[b].y;
      data[b].y = data[a].y;
      data[a].y = t;

      a = *(p + 2);
      b = *(p + 3);
      p += 4;
      t = data[b].x;
      data[b].x = data[a].x;
      data[a].x = t;

      t = data[b].y;
      data[b].y = data[a].y;
      data[a].y = t;
    }

    /* Do the FFT */
    i4b_echo_cancel_radix4_fp(data, I4B_ECHO_CANCEL_N_COMPLEX);
    i4b_echo_cancel_radix4_cr(data, fft_table, 4, I4B_ECHO_CANCEL_N_COMPLEX);

    data_end = data + I4B_ECHO_CANCEL_N_COMPLEX;

    /* Check for inverse transform */
    if (inverse & 1) {
      if (inverse & 2) {
        while (data != data_end) {
          /* loop unrolling for higher performance */
          data[0].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[1].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[2].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[3].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[4].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[5].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[6].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data[7].x /= I4B_ECHO_CANCEL_N_COMPLEX;
          data += 8;
        }
      }
    } else {
      while (data != data_end) {
        /* loop unrolling for higher performance */
        data[0].y = -data[0].y;
        data[1].y = -data[1].y;
        data[2].y = -data[2].y;
        data[3].y = -data[3].y;
        data[4].y = -data[4].y;
        data[5].y = -data[5].y;
        data[6].y = -data[6].y;
        data[7].y = -data[7].y;
        data += 8;
      }
    }
    return;
}
#endif
