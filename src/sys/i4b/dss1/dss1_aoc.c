/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
 *
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
 *---------------------------------------------------------------------------
 *
 *	dss1_aoc.c - Advice Of Charge support / Facility support
 *	--------------------------------------------------------
 *
 * $FreeBSD: $
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/types.h>
#include <sys/mbuf.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_l2.h>
#include <i4b/dss1/dss1_l3.h>
#include <i4b/dss1/dss1_aoc.h>

static void do_component(struct dss1_buffer *sc, u_int8_t *end);
static void next_state(struct dss1_buffer *sc, u_int8_t class, 
		       u_int8_t form, u_int8_t code, int32_t val);

static u_int8_t
get_byte(struct dss1_buffer *buf)
{
    u_int8_t temp = dss1_get_1(buf,0);
    buf->offset++;
    return temp;
}

static u_int32_t
get_value(struct dss1_buffer *buf, u_int32_t len, u_int32_t max)
{
    u_int32_t temp = 0;
    u_int8_t shift = 0;

    if(len > 255) {
       len = 255;
    }

    while(len--)
    {
        temp |= get_byte(buf) << shift;
	shift += 8;

	if(shift >= 32)
	{
	    /* skip rest */
	    buf->offset += len;
	    break;
	}
    }

    if(temp > max) {
       temp = max;
    }

    return temp;
}

/*---------------------------------------------------------------------------*
 *	decode Q.931/Q.932 facility info element
 *---------------------------------------------------------------------------*/
void
dss1_facility_decode(call_desc_t *cd, struct dss1_buffer *buf)
{
	cd->units_type = CHARGE_INVALID;
	cd->units = -1;			

	src->offset += 2;
	
	switch(dss1_get_1(buf,0) & 0x1f) {
	case FAC_PROTO_ROP:
	    NDBGL3(L3_A_MSG, "Remote Operations Protocol");
	    break;

	case FAC_PROTO_CMIP:
	    NDBGL3(L3_A_MSG, "CMIP Protocol (Q.941), UNSUPPORTED");
	    goto done;

	case FAC_PROTO_ACSE:
	    NDBGL3(L3_A_MSG, "ACSE Protocol (X.217/X.227), UNSUPPORTED!");
	    goto done;

	default:
	    NDBGL3(L3_A_ERR, "Unknown Protocol, UNSUPPORTED!");
	    goto done;
	}

	/* next byte */
	
	src->offset++;

	/* initialize variables for do_component */

	src->state = ST_EXP_COMP_TYP;
	src->op_value = -1;
	src->units = 0;

	/* decode facility */
	
	do_component(cd, src);

	switch(src->op_value) {

	case FAC_OPVAL_AOC_D_CUR:
	    cd->units_type = CHARGE_AOCD;
	    cd->units = 0;
	    i4b_l4_charging_ind(cd);
	    break;
			
	case FAC_OPVAL_AOC_D_UNIT:
	    cd->units_type = CHARGE_AOCD;
	    cd->units = buf->units;
	    i4b_l4_charging_ind(cd);
	    break;
			
	case FAC_OPVAL_AOC_E_CUR:
	    cd->units_type = CHARGE_AOCE;
	    cd->units = 0;
	    i4b_l4_charging_ind(cd);
	    break;
			
	case FAC_OPVAL_AOC_E_UNIT:
	    cd->units_type = CHARGE_AOCE;
	    cd->units = buf->units;
	    i4b_l4_charging_ind(cd);
	    break;

	case FAC_OPVAL_MCID:
	    break;

	case FAC_OPVAL_DIV_CALLDEF:
	    break;

	default:
	    break;
	}

 done:
	return;
}

/*---------------------------------------------------------------------------*
 *	handle a component recursively
 *---------------------------------------------------------------------------*/
static void
do_component(struct call_desc *cd, struct dss1_buffer *buf)
{
	u_int8_t comp_tag_class; /* component tag class */
	u_int8_t comp_tag_form;  /* component form: constructor or primitive */
	u_int8_t comp_tag_code;  /* component code depending on class */
	u_int8_t comp_length;    /* component length */
	u_int8_t temp;

	while(dss1_get_valid(buf,0))
	{
	    /*----------------------------------------*
	     * first component element: component tag *
	     *----------------------------------------*/

	    temp = get_byte(buf);

	    comp_tag_class = (temp & 0xc0) >> 6;
	
	    comp_tag_form = (temp & 0x20) >> 5;
	
	    comp_tag_code = (temp & 0x1f);
	
	    if(comp_tag_code == 0x1f)
	    {
		u_int32_t value = 0;
		u_int8_t shift = 0;

		do {
		  temp = get_byte(buf);

		  if(shift < 32)
		  {
		      value |= (temp & 0x7F) << shift;
		      shift += 7;
		  }

		} while(temp & 0x80);

		if(value > 0xFF)
		{
		   value = 0xFF; /* invalid */
		}

		comp_tag_code = value;
	    }

	    /*--------------------------------------------*
	     * second component element: component length *
	     *--------------------------------------------*/

	    temp = get_byte(buf);

	    if(temp & 0x80)
	    {
	        comp_length = get_value(buf, temp & 0x7f, 0xFF);
	    }
	    else
	    {
		comp_length = temp & 0x7f;
	    }

	    next_state(buf, comp_tag_class, comp_tag_form, comp_tag_code, -1);
	
	    /*---------------------------------------------*
	     * third component element: component contents *
	     *---------------------------------------------*/

	    if(comp_tag_form)
	    {
		/* constructor */
	        u_int16_t old_len;
		old_len = set_length(buf, buf->offset + comp_length);

		do_component(cd, buf);

		buf->offset = buf->len; /* set new offset */
		buf->len = old_len; /* set new length */
	    }
	    else
	    {
		u_int32_t value = 0;

		if(comp_tag_class == FAC_TAGCLASS_UNI)
		{
			switch(comp_tag_code) {
			case FAC_CODEUNI_INT:
			case FAC_CODEUNI_ENUM:
			case FAC_CODEUNI_BOOL:
			    value = get_value(buf, comp_length, -1);
			    break;

			default:
			    /* skip */
			    buf->offset += comp_length;
			    break;
			}
		}
		else
		{
			value = get_value(buf, comp_length, -1);
		}

		next_state(buf, comp_tag_class, comp_tag_form, comp_tag_code, value);
	    }
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	invoke component
 *---------------------------------------------------------------------------*/
static void
F_1_1(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_INV_ID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result
 *---------------------------------------------------------------------------*/
static void
F_1_2(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return error
 *---------------------------------------------------------------------------*/
static void
F_1_3(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject
 *---------------------------------------------------------------------------*/
static void
F_1_4(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	invoke id
 *---------------------------------------------------------------------------*/
static void
F_2(struct dss1_buffer *buf, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Invoke ID = %d", val);
		buf->state = ST_EXP_OP_VAL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	operation value
 *---------------------------------------------------------------------------*/
static void
F_3(struct dss1_buffer *buf, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Operation Value = %d", val);
	
		buf->op_value = val;
		
		if((val == FAC_OPVAL_AOC_D_UNIT) || 
		   (val == FAC_OPVAL_AOC_E_UNIT))
		{
			buf->units = 0;
			buf->state = ST_EXP_INFO;
		}
		else
		{
			buf->state = ST_EXP_NIX;
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	specific charging units
 *---------------------------------------------------------------------------*/
static void
F_4(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_RUL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	free of charge
 *---------------------------------------------------------------------------*/
static void
F_4_1(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		NDBGL3(L3_A_MSG, "Free of Charge");
		/* XXX buf->units = 0; */
		buf->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	charge not available
 *---------------------------------------------------------------------------*/
static void
F_4_2(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		NDBGL3(L3_A_MSG, "Charge not available");
		/* XXX buf->units = -1; */
		buf->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	recorded units list
 *---------------------------------------------------------------------------*/
static void
F_5(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_RU;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	recorded units
 *---------------------------------------------------------------------------*/
static void
F_6(struct dss1_buffer *buf, int32_t val)
{
	if(val == -1)
	{
		buf->state = ST_EXP_RNOU;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	number of units
 *---------------------------------------------------------------------------*/
static void
F_7(struct dss1_buffer *buf, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Number of Units = %d", val);
		buf->units = val;
		buf->state = ST_EXP_TOCI;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	subtotal/total
 *---------------------------------------------------------------------------*/
static void
F_8(struct dss1_buffer *buf, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Subtotal/Total = %d", val);
		buf->state = ST_EXP_DBID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	billing_id
 *---------------------------------------------------------------------------*/
static void
F_9(struct dss1_buffer *buf, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Billing ID = %d", val);
		buf->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	AOC state table
 *---------------------------------------------------------------------------*/
static const struct statetab {
	u_int8_t currstate;	/* input: current state we are in */
	u_int8_t form;		/* input: current tag form */
	u_int8_t class;		/* input: current tag class */
	u_int8_t code;		/* input: current tag code */
	void (*func)(struct dss1_buffer *, int32_t); /* output: func to exec */
} statetab[] = {

/*	 current state		tag form		tag class		tag code		function	*/
/*	 ---------------------  ----------------------  ----------------------  ---------------------- 	----------------*/
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	1,			&F_1_1		},
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	2,			&F_1_2		},
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	3,			&F_1_3		},
	{ST_EXP_COMP_TYP,	FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	4,			&F_1_4		},
	{ST_EXP_INV_ID,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	&F_2		},
	{ST_EXP_OP_VAL,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	&F_3		},
	{ST_EXP_INFO,		FAC_TAGFORM_CON,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_SEQ,	&F_4		},
	{ST_EXP_INFO,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_NULL,	&F_4_1		},
	{ST_EXP_INFO,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	1,			&F_4_2		},
	{ST_EXP_RUL,		FAC_TAGFORM_CON,	FAC_TAGCLASS_COS,	1,			&F_5		},
	{ST_EXP_RU,		FAC_TAGFORM_CON,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_SEQ,	&F_6		},
	{ST_EXP_RNOU,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_UNI,	FAC_CODEUNI_INT,	&F_7		},
	{ST_EXP_TOCI,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	2,			&F_8		},
	{ST_EXP_DBID,		FAC_TAGFORM_PRI,	FAC_TAGCLASS_COS,	3,			&F_9		}
};	
	
/*---------------------------------------------------------------------------*
 *	state decode for do_component
 *---------------------------------------------------------------------------*/
static void
next_state(struct dss1_buffer *buf, u_int8_t class, u_int8_t form, u_int8_t code, int32_t val)
{
	const struct statetab *st = &statetab[0];
	const struct statetab *st_end = &statetab[sizeof(statetab)/sizeof(statetab[0])];

	while(1)
	{
		if(st >= st_end) break;
		if(st->currstate > buf->state) break;

		if((st->currstate == buf->state) &&
		   (st->form == form)           &&
		   (st->class == class)         &&
		   (st->code == code))
		{
			(st->func)(buf, val);
			break;
		}
		st++;
	}
	return;
}

