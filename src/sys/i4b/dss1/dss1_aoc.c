/*-
 * Copyright (c) 1997, 2002 Hellmuth Michaelis. All rights reserved.
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
 *	dss1_aoc.c - Advice Of Charge support
 *	-------------------------------------
 *
 *---------------------------------------------------------------------------*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/types.h>
#include <sys/mbuf.h>

#include <i4b/include/i4b_debug.h>
#include <i4b/include/i4b_ioctl.h>
#include <i4b/include/i4b_global.h>

#include <i4b/dss1/dss1_l3.h>
#include <i4b/dss1/dss1_aoc.h>

__FBSDID("$FreeBSD: $");

struct aoc_state {
  u_int8_t   state;
  u_int8_t * byte_ptr;
  u_int8_t * byte_ptr_end;
  u_int8_t   operation_value;
  int32_t    units;
};

static void do_component(struct aoc_state *sc, u_int8_t *end);
static void next_state(struct aoc_state *sc, u_int8_t class, u_int8_t form, u_int8_t code, int32_t val);

static u_int8_t
get_byte(struct aoc_state *sc)
{
    u_int8_t temp = (sc->byte_ptr < sc->byte_ptr_end) ? sc->byte_ptr[0] : 0;
    sc->byte_ptr ++;
    return temp;
}

static u_int32_t
get_value(struct aoc_state *sc, u_int32_t len, u_int32_t max)
{
    u_int32_t temp = 0;
    u_int8_t shift = 0;

    while(len--)
    {
        temp |= get_byte(sc) << shift;
	shift += 8;

	if(shift >= 32)
	{
	    /* skip rest */
	    sc->byte_ptr += len;
	    break;
	}
    }

    if(temp > max)
    {
       temp = max;
    }

    return temp;
}

/*---------------------------------------------------------------------------*
 *	decode Q.931/Q.932 facility info element
 *
 * returns 0 on success
 * returns 1 on error
 *---------------------------------------------------------------------------*/
u_int8_t
dss1_aoc(u_int8_t *buf, call_desc_t *cd)
{
	u_int8_t len;
	struct aoc_state sc;

	bzero(&sc, sizeof(sc));

	cd->units_type = CHARGE_INVALID;
	cd->units = -1;			
	
	buf++;		/* length */

	len = *buf;

	buf++;		/* protocol profile */

	switch(*buf & 0x1f)
	{
		case FAC_PROTO_ROP:
			break;

		case FAC_PROTO_CMIP:
			NDBGL3(L3_A_MSG, "CMIP Protocol (Q.941), UNSUPPORTED");
			return 1;
			break;

		case FAC_PROTO_ACSE:
			NDBGL3(L3_A_MSG, "ACSE Protocol (X.217/X.227), UNSUPPORTED!");
			return 1;
			break;

		default:
			NDBGL3(L3_A_ERR, "Unknown Protocol, UNSUPPORTED!");
			return 1;
			break;
	}

	NDBGL3(L3_A_MSG, "Remote Operations Protocol");

	/* next byte */
	
	buf++;
	if(len)
	  len--;

	/* initialize variables for do_component */
	
	sc.byte_ptr = buf;
	sc.byte_ptr_end = buf + len;
	sc.state = ST_EXP_COMP_TYP;	

	/* decode facility */
	
	do_component(&sc, buf + len);

	switch(sc.operation_value)
	{
		case FAC_OPVAL_AOC_D_CUR:
			cd->units_type = CHARGE_AOCD;
			cd->units = 0;
			break;
			
		case FAC_OPVAL_AOC_D_UNIT:
			cd->units_type = CHARGE_AOCD;
			cd->units = sc.units;
			break;
			
		case FAC_OPVAL_AOC_E_CUR:
			cd->units_type = CHARGE_AOCE;
			cd->units = 0;
			break;
			
		case FAC_OPVAL_AOC_E_UNIT:
			cd->units_type = CHARGE_AOCE;
			cd->units = sc.units;
			break;

		default:
			cd->units_type = CHARGE_INVALID;
			cd->units = -1;
			return 1;
	}
	return 0;
}

/*---------------------------------------------------------------------------*
 *	handle a component recursively
 *---------------------------------------------------------------------------*/
static void
do_component(struct aoc_state *sc, u_int8_t *end)
{
	u_int8_t comp_tag_class; /* component tag class */
	u_int8_t comp_tag_form;  /* component form: constructor or primitive */
	u_int8_t comp_tag_code;  /* component code depending on class */
	u_int8_t comp_length;    /* component length */
	u_int8_t temp;

again:
	/*----------------------------------------*
	 * first component element: component tag *
	 *----------------------------------------*/

	temp = get_byte(sc);

	comp_tag_class = (temp & 0xc0) >> 6;
	
	comp_tag_form = (temp & 0x20) >> 5;
	
	comp_tag_code = (temp & 0x1f);
	
	if(comp_tag_code == 0x1f)
	{
		u_int32_t value = 0;
		u_int8_t shift = 0;

		do {
		  temp = get_byte(sc);

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

	temp = get_byte(sc);

	if(temp & 0x80)
	{
		comp_length = get_value(sc, temp & 0x7f, 0xFF);
	}
	else
	{
		comp_length = temp & 0x7f;
	}

	next_state(sc, comp_tag_class, comp_tag_form, comp_tag_code, -1);
	
	/*---------------------------------------------*
	 * third component element: component contents *
	 *---------------------------------------------*/

	if(comp_tag_form)
	{
		/* constructor */

		do_component(sc, sc->byte_ptr + comp_length);
	}
	else
	{
		u_int32_t value = 0;

		if(comp_tag_class == FAC_TAGCLASS_UNI)
		{
			switch(comp_tag_code)
			{
				case FAC_CODEUNI_INT:
				case FAC_CODEUNI_ENUM:
				case FAC_CODEUNI_BOOL:

					value = get_value(sc, comp_length, -1);
					break;

				default:
					/* skip */
					sc->byte_ptr += comp_length;
					break;
			}
		}
		else
		{
			value = get_value(sc, comp_length, -1);
		}

		next_state(sc, comp_tag_class, comp_tag_form, comp_tag_code, value);
	}

	if(sc->byte_ptr < end)
	{
	    goto again;
	}

	if(sc->byte_ptr != end)
	{
	    NDBGL3(L3_A_ERR, "pointer offset from end: %d bytes",
		   sc->byte_ptr - end);
	}

	sc->byte_ptr = end;

	return;
}

/*---------------------------------------------------------------------------*
 *	invoke component
 *---------------------------------------------------------------------------*/
static void
F_1_1(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_INV_ID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return result
 *---------------------------------------------------------------------------*/
static void
F_1_2(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	return error
 *---------------------------------------------------------------------------*/
static void
F_1_3(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	reject
 *---------------------------------------------------------------------------*/
static void
F_1_4(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	invoke id
 *---------------------------------------------------------------------------*/
static void
F_2(struct aoc_state *sc, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Invoke ID = %d", val);
		sc->state = ST_EXP_OP_VAL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	operation value
 *---------------------------------------------------------------------------*/
static void
F_3(struct aoc_state *sc, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Operation Value = %d", val);
	
		sc->operation_value = val;
		
		if((val == FAC_OPVAL_AOC_D_UNIT) || 
		   (val == FAC_OPVAL_AOC_E_UNIT))
		{
			sc->units = 0;
			sc->state = ST_EXP_INFO;
		}
		else
		{
			sc->state = ST_EXP_NIX;
		}
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	specific charging units
 *---------------------------------------------------------------------------*/
static void
F_4(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_RUL;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	free of charge
 *---------------------------------------------------------------------------*/
static void
F_4_1(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		NDBGL3(L3_A_MSG, "Free of Charge");
		/* XXX sc->units = 0; */
		sc->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	charge not available
 *---------------------------------------------------------------------------*/
static void
F_4_2(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		NDBGL3(L3_A_MSG, "Charge not available");
		/* XXX sc->units = -1; */
		sc->state = ST_EXP_NIX;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	recorded units list
 *---------------------------------------------------------------------------*/
static void
F_5(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_RU;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	recorded units
 *---------------------------------------------------------------------------*/
static void
F_6(struct aoc_state *sc, int32_t val)
{
	if(val == -1)
	{
		sc->state = ST_EXP_RNOU;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	number of units
 *---------------------------------------------------------------------------*/
static void
F_7(struct aoc_state *sc, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Number of Units = %d", val);
		sc->units = val;
		sc->state = ST_EXP_TOCI;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	subtotal/total
 *---------------------------------------------------------------------------*/
static void
F_8(struct aoc_state *sc, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Subtotal/Total = %d", val);
		sc->state = ST_EXP_DBID;
	}
	return;
}

/*---------------------------------------------------------------------------*
 *	billing_id
 *---------------------------------------------------------------------------*/
static void
F_9(struct aoc_state *sc, int32_t val)
{
	if(val != -1)
	{
		NDBGL3(L3_A_MSG, "Billing ID = %d", val);
		sc->state = ST_EXP_NIX;
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
	void (*func)(struct aoc_state *, int32_t); /* output: func to exec */
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
next_state(struct aoc_state *sc, u_int8_t class, u_int8_t form, u_int8_t code, int32_t val)
{
	const struct statetab *st = &statetab[0];
	const struct statetab *st_end = &statetab[sizeof(statetab)/sizeof(statetab[0])];

	while(1)
	{
		if(st >= st_end) break;
		if(st->currstate > sc->state) break;

		if((st->currstate == sc->state) &&
		   (st->form == form)           &&
		   (st->class == class)         &&
		   (st->code == code))
		{
			(st->func)(sc, val);
			break;
		}
		st++;
	}
	return;
}

