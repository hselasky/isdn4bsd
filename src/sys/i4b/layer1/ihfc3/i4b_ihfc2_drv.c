/*-
 * Copyright (c) 2000-2005 Hans Petter Selasky. All rights reserved.
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
 *	i4b_ihfc2_drv.c - driver interface
 *	----------------------------------
 *
 * $FreeBSD: $
 *
 *	NOTE: INTEL CPU's use 8-bit I/O access for the ISA-BUS, so
 *	      counter registers may be unstable even if
 *	      bus_space_read_4 is used.
 *
 *	Prefixes:
 *		         Z: Z-counter or unwrapped FIFO transfer length related
 *			 F: F-counter related
 *			 f: ihfc_fifo_t related
 *			sc: ihfc_sc_t related
 *			s_: ihfc register related
 *			i_: isac register related
 *			h_: hscx register related
 *			w_: winbond register related
 *			p_: psb register related
 *---------------------------------------------------------------------------*/

#include <i4b/layer1/ihfc3/i4b_ihfc2.h>
#include <i4b/layer1/ihfc3/i4b_ihfc2_ext.h>

static void
ihfc_fifo_program(register ihfc_sc_t *sc);

static void
ihfc_fifo_program_reset(ihfc_sc_t *sc, ihfc_fifo_t *f);

static void
__ihfc_chip_interrupt(ihfc_sc_t *sc);

/*---------------------------------------------------------------------------*
 * : reg_get_desc - ``provide human readable debugging support''
 *---------------------------------------------------------------------------*/
static __inline const u_char *
reg_get_desc(u_int8_t off)
{
    const u_char * desc [] = REGISTERS(REG_MACRO_1);

    if(off >= (sizeof(desc)/sizeof(desc[0])))
    {
        return "unknown";
    }
    else
    {
        return desc[off];
    }
}

/*---------------------------------------------------------------------------*
 * : ihfc_fifos_active() - search for active FIFOs
 *---------------------------------------------------------------------------*/
u_int8_t
ihfc_fifos_active(ihfc_sc_t *sc)
{
    ihfc_fifo_t *f;
    u_int8_t n;

    for(n = 0; 
	n < sc->sc_default.d_sub_controllers;
	n++)
    {
        if(sc->sc_state[n].state.active ||
	   sc->sc_state[n].state.pending)
	{
	    /* NOTE: the search below will not be
	     * very lengthy, hence the D-channel 
	     * is usually active, and its FIFO
	     * is checked first!
	     */
	    FIFO_FOREACH(f,sc)
	    {
	        if((f->prot_curr.protocol_1 != P_DISABLE) &&
		   (sc->sc_state[f->sub_unit].state.active ||
		    (sc->sc_state[f->sub_unit].state.pending)))
		{
		    return 1;
		}
	    }
	    break;
	}
    }
    return 0;
}

/*---------------------------------------------------------------------------*
 * : ihfc_config_write_sub
 *---------------------------------------------------------------------------*/
void
ihfc_config_write_sub(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
    register_list_t *r;

    /*
     * configure chip,
     * write new configuration
     */
    REGISTER_FOREACH(r,sc->sc_default.d_register_list) {
	u_int8_t *data  = &OFF2REG(sc->sc_config, r->offset);
	u_int8_t *data2 = &OFF2REG(sc->sc_config2, r->offset);

	if ((f == IHFC_CONFIG_WRITE_RELOAD) || (*data != *data2)) {
		IHFC_MSG("0x%02x->%s (0x%02x).\n",
			 *data, reg_get_desc(r->offset), r->regval);
		/*
		 * Write 8-bit register
		 */

		CHIP_WRITE_1(sc, r->regval, data);

		/*
		 * Update shadow config
		 */

		*data2 = *data;
	}
    }
    return;
}

/*---------------------------------------------------------------------------*
 * : write config (sc_config)					(ALL CHIPS)
 *
 * Which register(s) that will be written to chip, depends on the
 * second parameter given to this function, called `f':
 *
 * `f == &sc->sc_fifo[...]'   :  All register(s) used in `struct sc_fifo' ***
 * `f == IHFC_CONFIG_WRITE_RELOAD' :  All register(s) used in `struct sc_config'
 * `f == IHFC_CONFIG_WRITE_UPDATE' :  Only changed register(s) ------ || ------
 *
 *
 * *** NOTE: if `f == &sc->sc_fifo[...]' that means:
 *	- reset f->program 
 *	- reset f->filter
 *	- reset fifo hardware
 *	- re-configure fifo hardware if needed
 *
 * 16-bit and 32-bit registers must be added to a custom `c_chip_config_write'.
 *
 * This routine is called from fifo_setup(,), ihfc_fsm_update(,),
 * chip_reset(,) and fifo_link(,) ...
 *---------------------------------------------------------------------------*/
static void
ihfc_config_write(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
  struct sc_config *c = &sc->sc_config;

  if ((f == IHFC_CONFIG_WRITE_UPDATE) ||
      (f == IHFC_CONFIG_WRITE_RELOAD)) {

	/* Update interrupt delay for
	 * ihfc_poll_interrupt
	 *
	 * Enable timers with interval 64ms,
	 * 50ms,  25ms  or  other  interval,
	 * when line is activated. Else dis-
	 * able the timers (sleep mode).
	 */
	if(ihfc_fifos_active(sc))
	{
	  /* decrease interrupt delay when
	   * the line is activated.
	   * 
	   * The code below works a bit like
	   * (a % (1*hz)).  The chip
	   * specific delay should be less
	   * than 1*hz.
	   */
	  if(sc->sc_default.d_interrupt_delay >= (1*hz))
	  {
	    sc->sc_default.d_interrupt_delay -= 1*hz;
	  }

	  c->s_int_m1   |=  0x80; /* enable t50 */
	  c->s_ctmt_0   |=  0x04; /* enable t50 */
	  c->i_mask     &= ~0x08; /* enable TIN */
	  c->b_mask     &= ~0x08; /* enable AUX */
	  c->t_dma_oper |=  0x01; /* enable DMA */
	}
	else
	{
	  /* increase interrupt delay when
	   * the line is deactivated.
	   *
	   * NOTE: this delay must be
	   * shorter than ``IHFC_T3_DELAY''.
	   */
	  if(sc->sc_default.d_interrupt_delay < (hz / 4))
	  {
	    sc->sc_default.d_interrupt_delay += 1*hz;
	  }

	  c->s_int_m1   &= ~0x80; /* disable t50 */
	  c->s_ctmt_0   &= ~0x04; /* disable t50 */
	  c->i_mask     |=  0x08; /* disable TIN */
	  c->b_mask     |=  0x08; /* disable AUX */
	  c->t_dma_oper &= ~0x01; /* disable DMA */
	}
  }
  else
  {
	/* configure fifo */
	/* currently nothing to configure */

    /* XXX */

	/* NOTE: ``ihfc_fifo_program_reset(,)'' will
	 * setup dummies for f->program and f->filter
	 *
	 * forward program to reset and
	 * do some checking
	 */
	ihfc_fifo_program_reset(sc,f);
  }

  /* NOTE: s_ctmt_0 is written to chip
   * by``CHIP_CONFIG_WRITE(,)''
   */

  /* ??
   * Call chip specific configuration routine
   * last, hence the ``CHIP_CONFIG_WRITE(,)''
   * routine can call filter(s), which must be
   * setup by ``ihfc_fifo_program_reset(,)'':
   */
  CHIP_CONFIG_WRITE(sc,f);

  return;
}

/*---------------------------------------------------------------------------*
 * : reset chip(s)						(ALL CHIPS)
 *
 * NOTE: this routine may be called from a low-priority fifo
 *
 * NOTE: reset pins should be connected so that when one chip is
 * 	 reset, all chips are reset.
 *
 *---------------------------------------------------------------------------*/
void
ihfc_reset(ihfc_sc_t *sc, u_int8_t *error)
{
	ihfc_fifo_t *f;
	u_int8_t sub_unit;

	IHFC_ASSERT_LOCKED(sc);

	/* reset config buffer (used by USB) */
	BUF_SETUP_WRITELEN(&sc->sc_config_buffer.buf,
			   (void *)&sc->sc_config_buffer.start[0],
			   (void *)&sc->sc_config_buffer.end[0]);
	/*
	 * ========
	 *  step 1 
	 * ========
	 *
	 * reset chip:
	 */
	CHIP_RESET(sc,error);

	if(IHFC_IS_ERR(error))
	{
		IHFC_ADD_ERR(error,
			     "- Reset did not detect chip! "
			     "(A hard reboot may help)");
		return;
	}

	/* reset ``sc_fifo_select_last'' */
	sc->sc_fifo_select_last = (ihfc_fifo_t *)0;

	/* reload configuration first */
	ihfc_config_write(sc, IHFC_CONFIG_WRITE_RELOAD);

	/*
	 * ========
	 *  step 2
	 * ========
	 *
	 * reset FIFO:
	 */
	FIFO_FOREACH(f,sc)
	{
		/* configure and reset fifo */
		ihfc_config_write(sc,f);
	}

	/*
	 * ========
	 *  step 3
	 * ========
	 *
	 * call interrupt handler to clear any
	 * early interrupts, to run queued
	 * FIFOs and to start any timers:
	 */
	__ihfc_chip_interrupt(sc);

	/*
	 * ========
	 *  step 4
	 * ========
	 *
	 * release   statemachine  and
	 * try to re-activate the line
	 * if it was already activated.
	 *
	 * NOTE: ``ihfc_fsm_update(,)'' will
	 * also call ``ihfc_config_write(,)''
	 */
	for(sub_unit = 0; 
	    sub_unit < sc->sc_default.d_sub_controllers; 
	    sub_unit++)
	{
	    struct sc_state *st = &sc->sc_state[sub_unit];

	    /* reset state description */
	    st->state.description = (const char *)0;

	    ihfc_fsm_update
	      (sc, st->i4b_controller->L1_fifo, 
	       st->state.active ? 1 : 0);
	}

	/* return success */
	return;
}

/*---------------------------------------------------------------------------*
 * : statemachine read hardware					(ALL CHIPS)
 *---------------------------------------------------------------------------*/
static fsm_state_t *
fsm_read(register ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	u_int8_t state = 0;

	FSM_READ(sc,f,&state);

	return &(sc->sc_default.d_fsm_table->state[state & 0xf]);
}

/*---------------------------------------------------------------------------*
 * : statemachine write hardware				(ALL CHIPS)
 *---------------------------------------------------------------------------*/
static void
fsm_write(register ihfc_sc_t *sc, ihfc_fifo_t *f, u_int8_t flag)
{
	fsm_command_t *fsm = &(sc->sc_default.d_fsm_table->cmd[flag & 0x7]);

	/*
	 * Generate debug output:
	 */

	IHFC_MSG("%s, cmd[%d]=0x%02x\n",
		 fsm->description ? 
		 (const char *)(fsm->description) : 
		 (const char *)("unknown"), 
		 flag, fsm->value);

	FSM_WRITE(sc,f,&fsm->value);

	return;
}

/*---------------------------------------------------------------------------*
 * : fsm_T3_expire (PH layer)                                   (ALL CHIPS)
 *---------------------------------------------------------------------------*/
static void
fsm_T3_expire(void *arg)
{
	struct i4b_controller *cntl = arg;
	ihfc_sc_t *sc;
	ihfc_fifo_t *f;

	CNTL_LOCK_ASSERT(cntl);

	sc = cntl->L1_sc;
	f = cntl->L1_fifo;

	IHFC_MSG("T3 timeout.\n");

	if (sc)
	{
	    /* timeout command (through ihfc_fsm_update) */
	    ihfc_fsm_update(sc,f,3);
	}

	return;
}

#define IHFC_T3_DELAY (4*hz) /* PH - activation timeout */

/*---------------------------------------------------------------------------*
 * : physical layer state machine handler function             (ALL CHIPS)
 *
 *	flag: 0 = Read state and process commands
 *	      1 = Activate
 *	      2 = Deactivate
 *	      3 = Activate timeout
 *
 *	NOTE: Before the HFC or ISAC chips start sending any B- or
 *	      D-channel data to the cable, the statemachine must be
 *	      properly activated. This can be checked by polling
 *	      ``sc->sc_statemachine.state.active'' which is set to
 *	      one when this is true. The driver should call
 *	      "ihfc_fsm_update(sc,1)" when data is waiting and
 *	      ``sc->sc_statemachine.state.active'' is zero.
 *
 *      NOTE: When activated the HFC and ISAC chips transmit data on
 *            all channels, but only 1's, ``high impedance'', which
 *            do not disturb other connections.
 *
 *	NOTE: The state(4-bit number) of the S0-line is looked up in
 *	      in a table with ((1<<4) ==) 16 entries, fsm_table, to hide
 *	      chip differences.  In general it is the chip's job to handle
 *	      state transitions, with one or two exceptions.
 *
 *      NOTE: Test signal can be used to ensure proper synchronization.
 *
 *      NOTE: The FIFO structure pointed to by "f" is just used to
 *            get the "sub_unit" in a convenient way.
 *---------------------------------------------------------------------------*/
void
ihfc_fsm_update(ihfc_sc_t *sc, ihfc_fifo_t *f, u_int8_t flag)
{
	struct sc_state *st = &(sc->sc_state[f->sub_unit]);
	struct fsm_state fsm_state;

	IHFC_ASSERT_LOCKED(sc);

	/*
	 * Get current state (rx/downstream)
	 */
	fsm_state = *fsm_read(sc,f);;

	/*
	 * Generate debug output:
	 */

	if (fsm_state.description)
	{
	    if (fsm_state.description != st->state.description)
	    {
	        ihfc_trace_info(sc, f, fsm_state.description);

		IHFC_MSG("%s. (p%d,a%d,c%d,u%d,d%d,i%d)\n",
			 fsm_state.description,
			 fsm_state.pending,
			 fsm_state.active,
			 fsm_state.command,
			 fsm_state.can_up,
			 fsm_state.can_down,
			 fsm_state.index);
	    }
	}
	else
	{
	    if (fsm_state.index != st->state.index)
	    {
	        IHFC_ERR("Illegal state: %d!\n", 
			 fsm_state.index);
	    }
	}

	/*
	 * command request:
	 * execute
	 */
	if(fsm_state.command)
	{
	    fsm_write(sc,f,fsm_state.index);
	}

	if(flag == 0)
	{
	    /*
	     * if the line goes down while the upper 
	     * layers are holding references on the
	     * auto activation, try a re-activation:
	     */
	    if((st->L1_auto_activate_ptr[0]) &&
	       (!fsm_state.active) &&
	       (!usb2_callout_pending(&st->T3callout)))
	    {
	        flag = 3;
	    }
	}

	/*
	 * user activation:
	 * set trycount
	 */
	if(flag == 1)
	{
	    st->L1_auto_activate_variable = 1;
	}

	/*
	 * user deactivation:
	 * clear trycount
	 */
	if(flag == 2)
	{
	    st->L1_auto_activate_variable = 0;
	}

	/*
	 * user activation
	 * timeout, T3
	 */
	if(flag == 3)
	{
	    if(fsm_state.active)
	    {
	        flag = 0;
	    }
	    else
	    {
	        if(st->L1_auto_activate_ptr[0] == 0)
		{
		    flag = 2;
		}
	    }
	}

	/*
	 * Deactivate command:
	 */

	if((flag == 2) || (flag == 3))
	{
	    /* index 1 is deactivate
	     * command in all tables
	     */
	    if(fsm_state.can_down)
	    {
	        fsm_write(sc,f,1);
	    }
	}

	/*
	 * Activate command:
	 */

	if((flag == 1) || (flag == 3))
	{
	    /* index 0 is activate
	     * command in all tables
	     */
	    if(fsm_state.can_up)
	    {
	        fsm_write(sc,f,0);
	    }

	    if(!usb2_callout_pending(&st->T3callout))
	    {
	        usb2_callout_reset(&st->T3callout, IHFC_T3_DELAY, 
				&fsm_T3_expire, st->i4b_controller);
	    }
	}

	/* update upper layers */

	if(st->state.active != fsm_state.active)
	{
	    *(st->L1_activity_ptr) = fsm_state.active;
#if 0
	    /* send an activation signal to L4 */
	    i4b_l4_l12stat(sc->sc_i4bunit,1,fsm_state.active);
	    i4b_l4_l12stat(sc->sc_i4bunit,2,fsm_state.active);
#endif
	}

	/* store state in softc */

	st->state = fsm_state;

	/*
	 * Update timer
	 */

	ihfc_config_write(sc, IHFC_CONFIG_WRITE_UPDATE);
	return;
}

/*---------------------------------------------------------------------------*
 * : include generic fifo processing programs
 *---------------------------------------------------------------------------*/
#include <i4b/layer1/ihfc3/i4b_program.h>

/*---------------------------------------------------------------------------*
 * : global filter support macros and structure(s)
 *---------------------------------------------------------------------------*/
struct filter_info {
	u_int8_t  unused;
	u_int8_t  direction;
	u_int16_t buffersize; /* in bytes */
	u_int16_t protocol[4];
  void (*rxtx_interrupt) RXTX_INTERRUPT_T(,);
  void (*filter) FIFO_FILTER_T(,);
};

#define I4B_FILTER_INFO_DECLARE(name)				\
	static const struct filter_info name			\
	  __attribute__((__section__("ihfc_filter_info_start"),	\
			 __aligned__(1),__used__))		\
/**/
/*
 * NOTE: __aligned__(1) is used to make sure that
 *	 the compiler doesn't put any bytes between
 *	 the structures!
 */

#define I4B_FILTER_EXPORT()				\
	I4B_FILTER_INFO_DECLARE(UNIQUE(ihfc_filter_info))

extern struct filter_info
  ihfc_filter_info_start[0],
  ihfc_filter_info_end[0];

#include <i4b/layer1/ihfc3/i4b_count.h>
#include <i4b/layer1/ihfc3/i4b_filter.h>

/*---------------------------------------------------------------------------*
 * : setup/unsetup a temporary data buffer
 *---------------------------------------------------------------------------*/
static u_int8_t
ihfc_buffer_setup(ihfc_sc_t *sc, ihfc_fifo_t *f, u_int16_t buffersize)
{
	/* free old buffer */
	if(f->buf.Buf_start)
	{
	  free(f->buf.Buf_start, M_CACHE);

	  /* clear counters */
	  bzero(&f->buf, sizeof(f->buf));
	}

	/* get new buffer */
	if(buffersize)
	{
		if(!f->buf.Buf_start)
		{
			if(!(f->buf.Buf_start = malloc(buffersize,
						       M_CACHE,
						       M_NOWAIT|M_ZERO)))
			{
				IHFC_ERR("malloc == 0!\n");
				return 1;
			}

			/* setup data pointers */
			f->buf.Dat_start =
			f->buf.Dat_end   = f->buf.Buf_start;
			f->buf.Buf_end   = f->buf.Buf_start + buffersize;
		}
	}
  
	IHFC_MSG("soft buffer(%d bytes) %s.\n", buffersize,
		 buffersize ? "enabled" : "disabled");

	return 0;
}

/*---------------------------------------------------------------------------*
 * : fifo link routine
 * NOTE: first match wins
 *---------------------------------------------------------------------------*/
static void
ihfc_fifo_link(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	const struct filter_info *filt;
	__typeof(((struct filter_info *)0)->rxtx_interrupt) *rxtx_interrupt;
	__typeof(((struct filter_info *)0)->buffersize) buffersize = 0;
	u_int16_t protocol;
	u_int8_t direction;

	IHFC_ASSERT_LOCKED(sc);

	rxtx_interrupt = (FIFO_DIR(f) == transmit) ?
	  &FIFO_TRANSLATOR(sc,f)->L5_TX_INTERRUPT :
	  &FIFO_TRANSLATOR(sc,f)->L5_RX_INTERRUPT ;

 reconfigure:

	if(f->prot_curr.protocol_1 == P_HDLC_EMU)
	{
	  /* NOTE: transmit channels that doesn't
	   * repeat the last byte, or are shared,
	   * should use the D-channel HDLC encoder(!)
	   */
	  if((FIFO_LOGICAL_NO(f) == d1t) ||
	     (!sc->sc_default.o_TRANSPARENT_BYTE_REPETITION))
	  {
	      f->prot_curr.protocol_1 = P_HDLC_EMU_D;
	  }
	}

	/* search for program */
	f->program = FIFO_GET_PROGRAM(sc,f);

	/* NOTE: some FIFO program setup has
	 * been put in ihfc_config_write(,))
	 */
	if(f->program == NULL)
	{
	    /* allow HDLC fallback
	     * to HDLC_EMU
	     */
	    if(f->prot_curr.protocol_1 == P_HDLC)
	    {
	        f->prot_curr.protocol_1 = P_HDLC_EMU;
		goto reconfigure;
	    }
	    goto error;
	}

	/* search for filter */
	protocol = f->prot_curr.protocol_1;
	direction = FIFO_DIR(f);
	f->filter = NULL;

	if(protocol != P_DISABLE)
	{
	    for(filt = &ihfc_filter_info_start[0];
		filt < &ihfc_filter_info_end[0];
		filt++)
	    {
	        if(((filt->protocol[0] == protocol) ||
		    (filt->protocol[1] == protocol) ||
		    (filt->protocol[2] == protocol) ||
		    (filt->protocol[3] == protocol)) &&
		   (filt->direction == direction))
		{
		    /* set buffersize */
		    buffersize = filt->buffersize;

		    /* select filter */
		    f->filter = filt->filter;

		    /* select rxtx_interrupt */
		    if(filt->rxtx_interrupt)
		    {
		        *rxtx_interrupt = filt->rxtx_interrupt;
		    }

		    /* first match wins */
		    break;
		}
	    }
	}

	/* setup buffer [if any] */
	if(ihfc_buffer_setup(sc,f,buffersize))
	{
	    goto error;
	}
	return;

 error:
	/* no match for this
	 * combination
	 */
	if(f->prot_curr.protocol_1 != P_DISABLE)
	{
	    IHFC_ERR("fifo(#%d) cannot be configured "
		     "for protocol %d: %s\n",
		     FIFO_NO(f), f->prot_curr.protocol_1,
		     !f->program ? "no program" : "");
	}

	/* set defaults */

	*rxtx_interrupt = NULL;

	f->program = NULL;
	f->filter = NULL;
	f->prot_curr.protocol_1 = P_DISABLE;

	if(ihfc_buffer_setup(sc,f, 0))
	{
	    /* ignore */
	}
	return;
}

/*---------------------------------------------------------------------------*
 * : ihfc interrupt routine
 *---------------------------------------------------------------------------*/
static void
__ihfc_chip_interrupt(ihfc_sc_t *sc)
{
	/* Interrupts are generated by software and hardware,
	 * so this routine may recurse:
	 *
	 * ihfc_chip_interrupt()->ihfc_fifo_program()->
	 *  ihfc_fifo_setup()->ihfc_fifo_call()->ihfc_chip_interrupt()
	 *
	 * When this routine recurses, the second
	 * call will return immediately.  However,
	 * new FIFOs queued will be detected by
	 * ihfc_fifo_program().
	 */

	mtx_assert(sc->sc_mtx_p, MA_OWNED);

	/* set ``sc_chip_interrupt_called'' */
	if(!sc->sc_chip_interrupt_called)
	{   sc->sc_chip_interrupt_called = 1;

	    /* read status */
	    CHIP_STATUS_READ(sc);

#if DO_I4B_DEBUG
	    if(i4b_debug_mask.L1_HFC_DBG)
	    {
	      microtime(&sc->sc_stack.tv);
	      IHFC_MSG("del=%08d ista=0x%04x, exir=0x%04x, "
		       "h_ista=0x%04x, h_exir=0x%04x, s_int_s1=0x%02x\n",
		       (u_int32_t)(sc->sc_stack.tv.tv_usec - 
				   sc->sc_stack.tv2.tv_usec),
		       sc->sc_config.i_ista, sc->sc_config.i_exir,
		       sc->sc_config.h_ista, sc->sc_config.h_exir,
		       sc->sc_config.s_int_s1);

	      sc->sc_stack.tv2 = 
		sc->sc_stack.tv;
	    }
#endif

	    /* check status */
	    CHIP_STATUS_CHECK(sc);

	    /* do fifo processing, if any */
	    ihfc_fifo_program(sc);

	    /* unselect chip, write commands ... */
	    CHIP_UNSELECT(sc);

	    /*
	     * Check if polling is active
	     */
	    if(IS_POLLED_MODE(sc,0))
	    {
	      if(SC_T125_WAIT(sc))
	      {
		if(SC_T125_WAIT_DELAY == 0)
		{
		  /* temporarily use ``DELAY()''
		   * instead of ``callout()''
		   *
		   * DELAY(200);
		   * 
		   * Danger of infinity
		   * loops ...
		   *
		   * goto status_read;
		   */
		}

		/* delay 1 millisecond (command delay) */
		if(!usb2_callout_pending(&sc->sc_pollout_timr_wait))
		{
		    usb2_callout_reset(&sc->sc_pollout_timr_wait,
				    SC_T125_WAIT_DELAY,
				    (void *)(void *)&__ihfc_chip_interrupt, sc);
		}
	      }

	      /* delay 50 millisecond (data delay) */
	      if(!usb2_callout_pending(&sc->sc_pollout_timr))
	      {
		usb2_callout_reset(&sc->sc_pollout_timr,
				sc->sc_default.d_interrupt_delay,
				(void *)(void *)&__ihfc_chip_interrupt, sc);
	      }
	    }

	    /* clear ``sc_chip_interrupt_called'' */
	    sc->sc_chip_interrupt_called = 0;
	}
	return;
}

/*---------------------------------------------------------------------------*
 * : ihfc interrupt routine
 *---------------------------------------------------------------------------*/
void
ihfc_chip_interrupt(void *arg)
{
	ihfc_sc_t *sc = (ihfc_sc_t *)arg;

	/* locking should be done by caller */

	IHFC_LOCK(sc); 

	__ihfc_chip_interrupt(sc);

	IHFC_UNLOCK(sc);

	return;
}

/*---------------------------------------------------------------------------*
 * : call a FIFO, for example to start transmission
 *
 * NOTE: recursion of this routine is allowed !
 *
 * NOTE: a FIFO should not call itself !
 *
 *---------------------------------------------------------------------------*/
void
ihfc_fifo_call(ihfc_sc_t *sc, ihfc_fifo_t *f)
{
	IHFC_ASSERT_LOCKED(sc);

	if(!(f->state &  ST_RUNNING)) {
	     f->state |= ST_RUNNING;

	    /*------------------------------------------+
	     | The system is designed to do write after |
	     | read.             Avoid calling the same |
	     | FIFO more times than needed.             |
	     +------------------------------------------*/

	    /* enqueue fifo */
	    QFIFO(sc,lo,FIFO_NO(f));

	    /* call interrupt */
	    if(!SC_T125_WAIT(sc))
	    {
	        __ihfc_chip_interrupt(sc);
	    }
	    else
	    {
		/*----------------------------+
		 | FIFO call is handled by    |
		 | the interrupt handler.     |
		 +----------------------------*/
	    }

	    /* this routine cannot sleep, hence
	     * it will cause ``sleeping with mutex''
	     * errors
	     */
	}
	return;
}

#define c (&sc->sc_config) /* save some code */

/*---------------------------------------------------------------------------*
 * : ihfc_fifo_setup - setup a FIFO channel 			(ALL CHIPS)
 *
 * NOTE: The protocol is passed by setting "f->prot_curr" before calling
 *	 this function.    "f->prot_curr" may be set to "disable" during
 *	 setup to indicate failure.      This function returns true when
 *	 "f->prot_curr" is disabled. Else it returns false.
 *
 * NOTE: When a channel(tx) is not in use only idle bytes (``0xff'')
 *	 should be transmitted.
 *
 * NOTE: When possible the ISAC chip will be used to generate idle bytes
 *	 for B-channel(s),       by switching the B-channel(s) an unused
 *	 IOM2-channel.  Else a bit in the HSCX MODE register is switched
 *	 with the same purpose instead of changing the TX/RX slot number
 *	 which may cause noise sent to the ISDN-line,  when the computer
 *	 is shut down (?)
 *
 * NOTE: some manuals use ``transparent mode'' about HDLC mode
 *	 and ``extended transparent mode'' about what is here
 *	 called transparent mode (audio).
 *
 * NOTE: this function will be called at shutdown and detach,
 *	 disabling all fifos.
 *
 * NOTE: RPF, RME, RFO, XPR, XDU and XCOL interrupts should always
 *       be enabled. (see ihfc_config_default)
 *---------------------------------------------------------------------------*/
static void
ihfc_fifo_setup_soft(register ihfc_sc_t *sc, register ihfc_fifo_t *f)
{
	u_int8_t aux_data_0_mask= 0;

	u_int8_t a_lmr1_mask    = 0;
	u_int8_t b_tr_cr_mask   = 0;

	u_int8_t h_mode_mask[2] = { 0, 0 };
	u_int8_t h_tsar_mask[2] = { 0, 0 };
	u_int8_t h_tsax_mask[2] = { 0, 0 };

	u_int8_t i_spcr_mask    = 0;

	u_int8_t s_connect_mask = 0;
	u_int8_t s_con_hdlc_mask= 0;
	u_int8_t s_ctmt_mask    = 0;
	u_int8_t s_sctrl_e_mask = 0;
	u_int8_t s_fifo_en_mask = 0;
	u_int8_t s_sctrl_mask   = 0;

	u_int8_t w_b1_mode_mask = 0;
	u_int8_t w_b2_mode_mask = 0;
	u_int8_t w_cmdr1_mask   = 0;
	u_int8_t w_cmdr2_mask   = 0;

	/*
	 * reset last [transmit-] byte
	 */
	f->last_byte = 0xff;

	/*
	 * Setup what bits can be changed:
	 */
	if(FIFO_DIR(f) == transmit)
	{
	  s_con_hdlc_mask = 0x40;
	}
	else
	{
	  s_con_hdlc_mask = 0x20;
	}

	switch(FIFO_NO(f)) {
	case d1t: /* d1 - Transmit */
		s_fifo_en_mask  = 0x10; /* chip: D1 channel     */
		s_sctrl_e_mask  = 0x04; /* chip: D1 channel     */
		w_cmdr1_mask    = 0xa0; /* chip: D1 channel     */
		aux_data_0_mask = sc->sc_default.led_masks[d1t/2];
		break;

	case d1r: /* d1 - Receive */
		s_fifo_en_mask  = 0x20; /* chip: D1 channel     */
		w_cmdr1_mask    = 0x50; /* chip: D1 channel     */
		b_tr_cr_mask    = 0x80; /* chip: D1 channel     */
		break;

	case b1t: /* b1 - Transmit */
		h_mode_mask[0]  = 0xff; /* chip: B1 protocol    */
	        h_tsax_mask[0]  = 0xfc; /* chip: B1 slot        */
		i_spcr_mask     = 0x0c; /* chip: B1 channel     */
		s_ctmt_mask     = 0x01; /* chip: B1 protocol    */
		s_fifo_en_mask  = 0x01; /* chip: B1 channel     */
		s_connect_mask  = 0x02; /* chip: B1 channel     */
		s_sctrl_mask    = 0x01; /* chip: B1 channel     */
		w_cmdr2_mask    = 0xa0; /* chip: B1 channel     */
		w_b1_mode_mask  = 0x80; /* chip: B1 protocol    */
		b_tr_cr_mask    = 0x08; /* chip: B1 channel     */
		a_lmr1_mask     = 0x01; /* chip: B1 channel     */
		aux_data_0_mask = sc->sc_default.led_masks[b1t/2];
		break;

	case b1r: /* b1 - Receive */
		h_mode_mask[0]  = 0xfb; /* chip: B1 protocol    */
		h_tsar_mask[0]  = 0xfc; /* chip: B1 slot        */
		s_ctmt_mask     = 0x01; /* chip: B1 protocol    */
  		s_fifo_en_mask  = 0x02; /* chip: B1 channel     */
		s_connect_mask  = 0x01; /* chip: B1 channel     */
		w_cmdr2_mask    = 0x50; /* chip: B1 channel     */
		w_b1_mode_mask  = 0x80; /* chip: B1 protocol    */
		b_tr_cr_mask    = 0x20; /* chip: B1 channel     */
		break;

	case b2t: /* b2 - Transmit */
		h_mode_mask[1]  = 0xff; /* chip: B2 protocol    */
		h_tsax_mask[1]  = 0xfc; /* chip: B2 slot        */
		i_spcr_mask     = 0x03; /* chip: B2 channel     */
		s_ctmt_mask     = 0x02; /* chip: B2 protocol    */
  		s_fifo_en_mask  = 0x04; /* chip: B2 channel     */
    		s_connect_mask  = 0x10; /* chip: B2 channel     */
		s_sctrl_mask    = 0x02; /* chip: B2 channel     */
		w_cmdr2_mask    = 0x0a; /* chip: B2 channel     */
		w_b2_mode_mask  = 0x80; /* chip: B2 protocol    */
		b_tr_cr_mask    = 0x10; /* chip: B2 channel     */
		a_lmr1_mask     = 0x02; /* chip: B2 channel     */
		aux_data_0_mask = sc->sc_default.led_masks[b2t/2];
		break;

	case b2r: /* b2 - Receive */
		h_mode_mask[1] = 0xfb; /* chip: B2 protocol    */
		h_tsar_mask[1] = 0xfc; /* chip: B2 slot        */
		s_ctmt_mask    = 0x02; /* chip: B2 protocol    */
  		s_fifo_en_mask = 0x08; /* chip: B2 channel     */
		s_connect_mask = 0x08; /* chip: B2 channel     */
		w_cmdr2_mask   = 0x05; /* chip: B2 channel     */
		w_b2_mode_mask = 0x80; /* chip: B2 protocol    */
		b_tr_cr_mask   = 0x40; /* chip: B2 channel     */
		break;
	}

	if(IS_LOCAL_LOOP(sc,0))
	{
	  f->s_con_hdlc  |=  0xC0;
	  c->s_connect   |=  0x36;

	  c->s_b1_ssl     =  0x80;
	  c->s_b1_rsl     =  0xC0;

	  c->s_b2_ssl     =  0x81;
	  c->s_b2_rsl     =  0xC1;

	  c->s_a1_ssl     =  0x82;
	  c->s_a1_rsl     =  0xC2;

	  c->s_a2_ssl     =  0x83;
	  c->s_a2_rsl     =  0xC3;

	  /* disable changes to connect bits */
	  s_connect_mask = 0;
	  s_con_hdlc_mask = 0;	
	}
	else
	{
	  /* NOTE: you need to restart
	   * all channels after o_LOCAL_LOOP
	   * was enabled !!
	   */
	  f->s_con_hdlc  &= ~0x80;
	  c->s_connect   &= ~0x24;

	  /* HFC-NOTE: When an ISDN transmit
	   * channel is not in use it is connected
	   * to SLOT-4 of the PCM30's STIO1 input,
	   * which must be enabled first, and is
	   * assumed to be unused.  The HFC will
	   * pull the STIO's high internally. This
	   * is done to assure that unused ISDN
	   * transmit channels send 0xFF bytes
	   * only:
	   */

	  c->s_b1_ssl     =  0x00;
	  c->s_b1_rsl     =  0xC4; /* enable B1-PCM-INPUT-SLOT ! */

	  c->s_b2_ssl     =  0x00;
	  c->s_b2_rsl     =  0xC4; /* enable B2-PCM-INPUT-SLOT ! */

	  c->s_a1_ssl     =  0x00;
	  c->s_a1_rsl     =  0xC4; /* enable A1-PCM-INPUT-SLOT ! */

	  c->s_a2_ssl     =  0x00;
	  c->s_a2_rsl     =  0xC4; /* enable A2-PCM-INPUT-SLOT ! */
	}

	/* disable HFC FIFO space first and then enable */
	f->Z_min_free = f->fm.h.Zsize;

	/*
	 * NOTE: If other protocols are added later, please make sure that 
	 *       your protocol resets its configured registers here:
	 *
	 * Reset configuration
	 */

      	if(PROT_IS_TRANSPARENT(&(f->prot_curr)))
	{
		/* Transparent mode */
		c->s_ctmt_0   |= s_ctmt_mask;	         /* trans enable */
		c->s_ctmt     |= s_ctmt_mask;	         /* trans enable */
		c->s_ctmt_pnp |= s_ctmt_mask;	         /* trans enable */
		c->s_ctmt_pci |= s_ctmt_mask;	         /* trans enable */
		f->s_con_hdlc |= 0x02;		         /* trans enable */

		c->h_mode[0]  &= ~h_mode_mask[0]; 	 /* */
		c->h_mode[1]  &= ~h_mode_mask[1]; 	 /* */
		c->h_mode[0]  |=  h_mode_mask[0] & 0xE4; /* trans enable */
		c->h_mode[1]  |=  h_mode_mask[1] & 0xE4; /* trans enable */

		c->w_b1_mode  |=  w_b1_mode_mask;
		c->w_b2_mode  |=  w_b2_mode_mask;

		if((f->prot_curr.protocol_1 != P_HDLC_EMU) &&
		   (f->prot_curr.protocol_1 != P_HDLC_EMU_D))
		{
		  /* default AUDIO limit == 100ms of data */

		  /* limit fifo usage to 800 bytes(100ms)
		   * - Assuming that about 400 bytes(50ms)
		   * is needed for each interrupt:
		   */
		  f->Z_min_free -= 800;
		}
		else
		{
		  /* default CODEC limit == 250ms of data */
		  f->Z_min_free -= 2000;
		}
	}

	if(PROT_IS_HDLC(&(f->prot_curr)))
	{
		/* HDLC mode */
		c->s_ctmt_0   &= ~s_ctmt_mask;	         /* HDLC enable */
		c->s_ctmt     &= ~s_ctmt_mask;	         /* HDLC enable */
		c->s_ctmt_pnp &= ~s_ctmt_mask;	         /* HDLC enable */
		c->s_ctmt_pci &= ~s_ctmt_mask;	         /* HDLC enable */
		f->s_con_hdlc &= ~0x02;		         /* HDLC enable */ 

		c->h_mode[0]  &= ~h_mode_mask[0];        /* */
		c->h_mode[1]  &= ~h_mode_mask[1]; 	 /* */
		c->h_mode[0]  |=  h_mode_mask[0] & 0x8C; /* HDLC enable */
		c->h_mode[1]  |=  h_mode_mask[1] & 0x8C; /* HDLC enable */

		c->w_b1_mode  &= ~w_b1_mode_mask;
		c->w_b2_mode  &= ~w_b2_mode_mask;

		f->Z_min_free = (HFC_MAX_FRAMES-1)*(f->Z_min_free/HFC_MAX_FRAMES);

		if(f->Z_min_free < (HFC_MAX_FRAMES-1))
		{
		    /* need at least one byte per frame */
		    f->Z_min_free = (HFC_MAX_FRAMES-1);

		    IHFC_MSG("f->Z_min_free < (HFC_MAX_FRAMES-1)\n");
		}
	}

	if(HFC_MAX_FRAMES >= f->fm.h.Fsize)
	{
	  IHFC_MSG("HFC_MAX_FRAMES >= f->fm.h.Fsize\n");
	}

	if(f->Z_min_free & Z_MSB)
	{
	   f->Z_min_free = 0;
	}

	/* NOTE: D_SEND must be forced to 0xC0 using
	 * D-reset-bit in register SCTRL_E, when
	 * fifo[d1t] is disabled !
	 */

	c->aux_data_0 ^= sc->sc_default.led_inverse_mask;

	if(f->prot_curr.protocol_1 != P_DISABLE)
	{
		c->aux_data_0    |=  aux_data_0_mask;      /* led  enable */
		c->a_lmr1        |=  a_lmr1_mask;          /* send enable */
		c->b_tr_cr       |=  b_tr_cr_mask; /* rec. or send enable */
		c->w_cmdr1       |=  w_cmdr1_mask;         /* fifo enable */
		c->w_cmdr2       |=  w_cmdr2_mask;         /* fifo enable */
		c->h_mode[0]     |=  h_mode_mask[0] & 4;   /* send enable */
		c->h_mode[1]     |=  h_mode_mask[1] & 4;   /* send enable */
		c->i_spcr	 |=  i_spcr_mask & 0x0a;   /* send enable */
		c->s_connect     &= ~s_connect_mask;/*rec. or send enable */
		c->s_sctrl       |=  s_sctrl_mask;         /* send enable */
		c->s_fifo_en     |=  s_fifo_en_mask;	   /* mem. enable */
		c->s_sctrl_e_pci &= ~s_sctrl_e_mask;       /* send enable */
		f->s_con_hdlc	 |=  0x04;		   /* fifo enable */
		f->s_con_hdlc    &= ~s_con_hdlc_mask;/*rec.or send enable */
	}
	else
	{
		c->aux_data_0    &= ~aux_data_0_mask;      /* led  disable */
		c->a_lmr1        &= ~a_lmr1_mask;          /* send disable */
		c->b_tr_cr       &= ~b_tr_cr_mask; /* rec. or send disable */
		c->w_cmdr1       &= ~w_cmdr1_mask;         /* fifo disable */
		c->w_cmdr2       &= ~w_cmdr2_mask;         /* fifo disable */
		c->h_mode[0]     &= ~(h_mode_mask[0] & 4); /* send disable */
		c->h_mode[1]     &= ~(h_mode_mask[1] & 4); /* send disable */
		c->i_spcr	 &= ~i_spcr_mask;	   /* send disable */
		c->s_connect     |=  s_connect_mask;/*rec. or send disable */
		c->s_sctrl       &= ~s_sctrl_mask;         /* send disable */
		c->s_fifo_en     &= ~s_fifo_en_mask;	   /* mem. disable */
		c->s_sctrl_e_pci |=  s_sctrl_e_mask;       /* send disable */
		f->s_con_hdlc	 &= ~0x0E;		   /* fifo disable */
		f->s_con_hdlc    |=  s_con_hdlc_mask;/*rec.or send disable */
	}

	c->aux_data_0 ^= sc->sc_default.led_inverse_mask;

	if(1)
	{
	  u_int8_t tmp;
	  ihfc_fifo_t *f_rx;
	  ihfc_fifo_t *f_tx;

	  /* bits[5..7] must be the same
	   * in registers ``f_rx->s_con_hdlc''
	   * and ``f_tx->s_con_hdlc''
	   */
	  if(FIFO_DIR(f) == transmit)
	  {
	    f_rx = f - transmit + receive;
	    f_tx = f;
	  }
	  else
	  {
	    f_rx = f;
	    f_tx = f - receive + transmit;
	  }

	  tmp = 
	    (f_rx->s_con_hdlc & 0x20)|
	    (f_tx->s_con_hdlc & 0x40);

	  f_rx->s_con_hdlc &= ~(0x20|0x40);
	  f_tx->s_con_hdlc &= ~(0x20|0x40);

	  /* NOTE: FIFO must be enabled: */
	  f_rx->s_con_hdlc |= tmp|0x04;
	  f_tx->s_con_hdlc |= tmp|0x04;
	}
 	return;
}

u_int8_t
ihfc_fifo_setup(register ihfc_sc_t *sc, register ihfc_fifo_t *f)
{
	IHFC_ASSERT_LOCKED(sc);

	IHFC_MSG("fifo(#%d) prot(%d).\n",
		 FIFO_NO(f), f->prot_curr.protocol_1);

	/*
	 * Free f->mbuf if nonzero and
	 * free all mbuf(s) in ifqueue if any.
	 *
	 * also see ``ihfc_fifo_link()'' !
	 */
 	I4B_FREEMBUF(f,f->mbuf);
	I4B_FREEMBUF(f,f->mbuf_dev);
#if 0
	I4B_CLEANIFQ(f,&f->ifqueue);
#else
	_IF_DRAIN(&f->ifqueue);
#endif

	/*
	 * Clear some variables
	 */
	f->mbuf     = NULL;
	f->mbuf_dev = NULL;
	f->buf_len  = 0;
	f->buf_size = 0;
	f->buf_ptr  = NULL;
 	f->io_stat  = 0;

#if 0
	/* it is the program's job to
	 * manage its ST_'s
	 */
	f->state   &= ~ST_PROGRAM_MASK;
#endif

	f->state  &= ~ST_RUNNING; /* make sure the reset
				   * program gets called
				   */

 	bzero(&f->hdlc,sizeof(f->hdlc));

	/*
	 * Setup some defaults
	 */
	f->ifqueue.ifq_maxlen = IFQ_MAXLEN;

	/*
	 * Try linking the FIFO first
	 * (f->prot_curr.protocol_1 may be changed!)
	 *
	 * NOTE: ihfc_config_write will 
	 * reset the FIFO program!
	 */
	ihfc_fifo_link(sc,f);

	ihfc_fifo_setup_soft(sc,f);

	/* init echo cancel */
	if((f->prot_curr.protocol_1 == P_TRANSPARENT) &&
	   (FIFO_DIR(f) == receive))
	{
	    struct i4b_echo_cancel *ec = &(sc->sc_echo_cancel[FIFO_NO(f)/2]);
	    i4b_echo_cancel_init(ec, -8, f->prot_curr.protocol_4);
	}

	/* init DTMF detector */
	if(f->prot_curr.protocol_1 == P_TRANSPARENT)
	{
	    struct fifo_translator *ft = FIFO_TRANSLATOR(sc,f);
	    if (FIFO_DIR(f) == receive) {
	        i4b_dtmf_init_rx(ft, f->prot_curr.protocol_4);
	    } else {
	        i4b_dtmf_init_tx(ft, f->prot_curr.protocol_4);
	    }
	}

	/* reset and configure FIFO first */
	ihfc_config_write(sc,f);

	/* 
	 * Setup timer with interval 64ms, 50ms, 25ms
	 * or other interval and update changed registers
	 */
	ihfc_config_write(sc, IHFC_CONFIG_WRITE_UPDATE);

	/* update last protocol */
	f->prot_last = f->prot_curr;

	/* call fifo program */
	ihfc_fifo_call(sc,f);

	return (f->prot_curr.protocol_1 == P_DISABLE);
}

/*---------------------------------------------------------------------------*
 * : default config values (sc_config)
 *
 * NOTE: Master-mode register(s) should be loaded last.
 * NOTE: The default config structure does not setup all registers.
 *	 Registers which are not listed will get the value zero.
 *---------------------------------------------------------------------------*/
static const struct sc_config ihfc_config_default =
{
	.aux_data_0        = 0x00 | 0x00, /* common aux_data register used
					   * for leds. This register should
					   * be setup by the drivers reset routine,
					   * if it needs any other value than zero.
 					   */
	/* am79c30 write only: */
	.a_lmr1            = 0x00 | 0x40, /* disable B1/B2 TX, enable Fa+F+S0 */
	.a_lmr2            = 0x00 | 0x58, /* int. enable F3+F7+F8 */
	.a_lpr             = 0x00 | 0x00, /* D-ch. high pri. */

	/* hfc (2b) write only: */
	.s_cirm_0          = 0xc0 | 0x00, /* int. OFF selected */
	.s_ctmt_0          = 0xe0 | 0x0b, /* t50ms + trans */

	/* hfc (2bds0 ISA) write only: */
	.s_cirm            = 0x00 | 0x00, /* int. OFF selected */
	.s_ctmt            = 0x00 | 0x0b, /* t50ms + trans */
	.s_int_m1          = 0x00 | 0x40, /* C/I int. enable */
	.s_int_m2          = 0x00 | 0x08, /* int. enable */
	.s_mst_mode        = 0x00 | 0x01, /* set master mode */
	.s_clkdel          = 0x00 | 0x00,
	.s_connect         = 0x00 | 0x10|0x08|0x02|0x01,
					  /* rec. and send ``disable'' */
	.s_sctrl           = 0x00 | 0x00, /* disable TX-FIFOs */
	.s_test            = 0x00 | 0x01, /* awake enable
					   * (NOTE: other bits _must_
					   *   be zero!)
					   */
	.s_b1_ssl          = 0x00 | 0x00,
	.s_b1_rsl          = 0x00 | 0x00,
	.s_b2_ssl          = 0x00 | 0x00,
	.s_b2_rsl          = 0x00 | 0x00,

	/* hfc (2bds0 PnP) write only: */
	.s_ctmt_pnp        = 0x00 | 0x17, /* t50ms + trans */
	.s_sctrl_e         = 0x00 | (0x09^8),
					  /* when changing ``sctrl_e''
					   * please check:
					   *
					   * HFC-SP (p.45),
					   * HFC-SPCI (p.35) and
					   * HFC-SUSB (p.54) manual
					   *
					   * awake enable, D_U enable,
					   * automatic G2->G3 transition
					   *
					   * NOTE: D_U is enabled because
					   * frames are repeated in software.
					   */
	.s_sctrl_r         = 0x00 | 0x03, /* enable RX-FIFOs */
	.s_mst_emod        = 0x00 | 0x00,
	.s_trm             = 0x00 | 0x00, /* */

	/* hfc (2bds0 PCI) write only: */
	.s_cirm_pci        = 0x00 | 0x00,
	.s_ctmt_pci        = 0x00 | 0x17, /* t50ms + trans */
	.s_int_m2_pci      = 0x00 | 0x08, /* int. enable */
	.s_fifo_en         = 0x00 | 0x00,

	/* tiger write only: */
	.t_dma_oper        = 0x00 | 0x00, /* DMA disabled */
	.t_prct            = 0x00 | 0x20, /* DMA edge trigger + 12CLK I/O */
        
	/* psb3186 write only: */
	.b_cir0		   = 0x00 | 0x0e, /* TIC bus address = 7, reset default */
	.b_mask            = 0x00 | 0x00, /* enable all interrupts */
	.b_iom_cr	   = 0x00 | 0x08, /* enable B-clk */
	.b_maskd	   = 0x00 | 0x00, /* enable all interrupts */
	.b_msti		   = 0x00 | 0xfe, /* enable STI 1 interrupt */
	.b_masktr	   = 0x00 | 0xff, /* disable all TRAN interrupts */
	.b_tr_cr	   = 0x00 | 0x00, /* all channels disabled */
	.b_timr            = 0x00 | 0x01, /* (one shot mode, 64ms * (1+1) = 128ms) */
#if 0
	.b_sqxr		   = 0x00 | 0x4f, /* multiframe enable */
#else
	.b_sqxr		   = 0x00 | 0x0f, /* multiframe disable */
#endif

	/* ipac write only: */
	.p_acfg            = 0x00 | 0x00, /* reset default */
	.p_aoe             = 0x00 | 0x3c, /* enable INT0/1 output (aux 6+7) */
	.p_atx             = 0x00 | 0xff, /* use high level output
					   * for all aux lines
					   */
	.p_mask            = 0x00 | 0xC0, /* reset default */
	.p_pota2           = 0x00 | 0x00, /* reset default */
        
	/*
	 * NOTE: After  reset  the  ISAC  and  HSCX  chips enable  all
	 * interrupts,  MASK == 0x00.  Disabling interrupts has little
	 * effect,  and  rather   cause  IRQ  failure, hence  the ISAC
	 * generates   interrupts   anyway:      For  example  if  the
	 * CIRQ-interrupt is disabled when the  CIRQ (statemachine) is
	 * active,  the ISAC will issue two extra interrupts where the
	 * CIRQ-state reads deactivated.    Interrupts are disabled by
	 * not responding to them.   That means: When the ISAC or HSCX
	 * chips signal  TransmitPoolReady the driver should not issue
	 * an  XTF-command  unless  there  is data and consequently no
	 * further  IRQ's  occurr.   On  the other  hand,  if  XTF  is
	 * executed when no data has been written to the fifo,   a new
	 * IRQ will be generated some  microseconds  later and the IRQ
	 * starts looping, using alot of CPU. The same is true for the
	 * RFIFO. If RME or RPF is not responded by a command no
	 * further interrupts will occurr.
	 *
	 * The exception  is  disabling  non-grounded-input interrupt-
	 * pins, which are prone to random generation of interrupts.
	 */
        
	/* isac write only: */
	.i_adf1            = 0x00 | 0x00,
	.i_adf2            = 0x00 | 0x00,
	.i_spcr            = 0x00 | 0x00, /* B1/B2 send disable (Test: 0x10) */
	.i_mask            = 0x00 | 0x00, /* enable all interrupts */
	.i_timr            = 0x00 | 0x01, /* (one shot mode, 64ms * (1+1) = 128ms) */
	.i_stcr            = 0x00 | 0x70, /* TIC bus address = 7 */
	.i_sqxr            = 0x00 | 0x0f, /* master, clock always active */
	.i_mode            = 0x00 | 0xc9, /* I_MODE:
					   * bit0 set    : stop/go-bit enabled
					   * bit1 cleared: IOM2-TIC bus access enabled
					   * bit2 set    : reserved
					   * bit3 set    : receiver enabled
					   * bit4 set    : reserved
					   * bit5..7     : Transparent mode 0 selected
					   */
#if 0
	.i_star2           = 0x00 | 0x04, /* multiframe enable */
#else
	.i_star2           = 0x00 | 0x00, /* multiframe not used */
#endif
        
	/* hscx write only: */
#define HSCX_CONFIG(a)							   \
	.h_ccr1[a]         = 0x00 | 0x80, /* power up HSCX */		   \
	.h_mask[a]         = 0x00 | 0x00, /* enable all IRQ's */	   \
	.h_mode[a]         = 0x00 | 0xE0, /* trans. mode + send disable */ \
	.h_xad1[a]         = 0x00 | 0xff, /* */				   \
	.h_xad2[a]         = 0x00 | 0xff, /* */				   \
	.h_rah2[a]         = 0x00 | 0xff, /* */				   \
	.h_ccr2[a]         = 0x00 | 0x30, /* */				   \
	.h_xccr[a]         = 0x00 | 0x07, /* bits per time slot */	   \
	.h_rccr[a]         = 0x00 | 0x07, /* bits per time slot */	   \
	.h_timr[a]         = 0x00 | 0x00, /* non-auto mode */		   \
	.h_rlcr[a]         = 0x00 | 0x00, /* non-auto mode */		   \
	.h_xbch[a]         = 0x00 | 0x00, /* */
        
	HSCX_CONFIG(0)
	HSCX_CONFIG(1)
#undef  HSCX_CONFIG

	/* wib (usb) write only: */
        .w_imask           = 0x00 | 0x00,  /* enable all interrupts */
        .w_cmdr1           = 0x00 | 0x00,  /* D-ch(RX+TX) disable */
        .w_cmdr2           = 0x00 | 0x00,  /* B1/B2-ch(RX+TX) disable */
        .w_ctl             = 0x00 | 0x00,  /* */
        .w_gcr             = 0x00 | 0x00,
        .w_mocr            = 0x00 | 0x00,
        .w_pie             = 0x00 | 0xf8,  /* enable leds 0..2 */
        .w_po2             = 0x00 | 0x00,  /* all leds off */
        .w_l1b1rs          = 0x00 | 0x04,  /* receive from L1 */
        .w_l1b2rs          = 0x00 | 0x05,  /* receive from L1 */
        .w_usbb1rs         = 0x00 | 0x02,  /* receive from L1 */
        .w_usbb2rs         = 0x00 | 0x03,  /* receive from L1 */
        .w_pcm1rs          = 0x00 | 0x00,  /* receive from PCM1 */
        .w_pcm2rs          = 0x00 | 0x01,  /* receive from PCM2 */

	/* wib (pci) write only: */
	.w_imask_pci       = 0x00 | 0x18,  /* enable all interrupts except
					    * XINT0 and XINT1, which
					    * sometimes trigger without
					    * any reason (probably not
					    * grounded).
					    */
	.w_d_exim          = 0x00 | 0x00,  /* enable all interrupts */
	.w_b1_exim         = 0x00 | 0x00,  /* enable all interrutps */
	.w_b2_exim         = 0x00 | 0x00,  /* enable all interrutps */
	.w_b1_adm1         = 0x00 | 0xff,  /* address comparison disabled */
	.w_b2_adm1         = 0x00 | 0xff,  /* address comparison disabled */
	.w_b1_adm2         = 0x00 | 0xff,  /* address comparison disabled */
	.w_b2_adm2         = 0x00 | 0xff,  /* address comparison disabled */
	.w_d_mode          = 0x00 | 0x44,  /* D-channel (rx/tx) enable, multiframe disable */
	.w_b1_mode         = 0x00 | 0x80,  /* extended transparent mode, 0xff interframe fill */
	.w_b2_mode         = 0x00 | 0x80,  /* extended transparent mode, 0xff interframe fill */
	.w_timr1           = 0x00 | 0x01,  /* timer1 100ms delay (one shot, enabled) */
	.w_timr2           = 0x00 | 0x00,  /* timer2 disabled */
	.w_ctl_pci         = 0x00 | 0x00,
	.w_pctl            = 0x00 | 0x00,  /* all outputs disabled */
	.w_gcr_pci         = 0x00 | 0x00,  /* reset default */
	.w_sam             = 0x00 | 0xff,  /* address comparison disabled */
	.w_tam             = 0x00 | 0xff,  /* address comparison disabled */
	.w_sqx             = 0x00 | 0x0f,  /* disable S/Q interrupt */
};

u_int8_t
ihfc_setup_softc(register ihfc_sc_t *sc, u_int8_t *error)
{
	ihfc_fifo_t *f;

        IHFC_LOCK(sc);

        if(!(sc->sc_default.o_POST_SETUP)) {
             sc->sc_default.o_POST_SETUP = 1;

             /*
              * Load default values
              */

             sc->sc_config = ihfc_config_default;

             /*
              * Next time this function
              * is called, only changes
              * will be updated
              */

	     /*
	      * LEDs
	      * set power led on
	      */
	     c->aux_data_0 =
	       sc->sc_default.led_inverse_mask ^ 
	       sc->sc_default.led_p1_mask;

	     FIFO_FOREACH(f,sc)
	     {
	         ihfc_fifo_setup_soft(sc,f);
	     }
        }

        if(sc->sc_default.stdel_nt == 0) {
           sc->sc_default.stdel_nt = 0x6c;
        }

        if(sc->sc_default.stdel_te == 0) {
           sc->sc_default.stdel_te = 0x0f;
        }

        /* ===============================
         * clear configuration to all
         * sc->sc_default.o_XXX's disabled
         * ===============================
         */

        /* am79c30 write only: */
        c->a_lmr2         &= ~0x03; /* clear loopback */
        c->a_lpr          &= ~0x0f; /* select D-ch. high pri. */

        /* isac write only: */
        c->i_adf1         |=  0x04; /* B-ch. at IOM ch. 2 */
        c->i_adf2         &= ~0x80; /* select IOM mode 1 */
        c->i_cirq         &= ~0x07; /* select IOM mode 1 + D-ch. high pri. */
        c->b_cir0         &= ~0x10; /* select D-ch. high pri. */

        if(sc->sc_default.o_ISAC_NT)
	{
          /* these features are not changed
           * unless the ISAC is capable of
           * NT-mode ...
           */
          c->i_spcr       &= ~0x10; /* clear test mode */
          c->i_stcr       |=  0x70; /* TIC bus address = 7 */
          c->b_cir0       |=  0x0e; /* TIC bus address = 7 */
          c->i_mode       &= ~0x02;
        }
	else
	{
          c->i_spcr       &= ~0x50; /* clear test mode + lineswitch */
        }

        /* hscx write only: */
#define HSCX_CONFIG(a)                                                          \
        c->h_tsax[a]      = 0x00 | 0x07; /* unused (?) */            \
        c->h_tsar[a]      = 0x00 | 0x07; /* unused (?) */            \
                                                                                \
        c->h_ccr1[a]     &= ~0x07;      /* clear clock mode */       \
        c->h_ccr1[a]     |= (sc->sc_default.o_IPAC ? 0x2 : 0x5); /* set clock mode */

        HSCX_CONFIG(0);
        HSCX_CONFIG(1);
#undef  HSCX_CONFIG

        /* hfc(2b/2bds0) write only: */
        c->s_cirm_0     &= ~0x17; /* 32K fifo mode */
        c->s_cirm_0     |= (sc->sc_resources.iirq[0] & 0x7);
        c->s_cirm       &= ~0x17; /* 32K fifo mode + switch to unused intr pin */
        c->s_cirm       |= (sc->sc_resources.iirq[0] & 0x7);
        c->s_int_m2     |= 0x08; /* int. output enable (ISA) */
        c->s_int_m2_pci |= 0x08; /* int. output enable (PCI) */

        c->s_clkdel     &= ~0x7f; /* */
        c->s_clkdel     |= (sc->sc_default.stdel_te);
        c->s_sctrl      &= ~0x4C; /* TE-mode + D-ch. high priority */

        /* wibusb write only: */
        c->w_cmdr1      &= ~0x03;
        c->w_d_mode     &= ~0x03;

        /* ===========================
         * enable features requested
         * by driver or user
         * ===========================
         */

        if(sc->sc_default.o_BUS_TYPE_IOM2)
	{
          c->i_adf1         &= ~0x04; /* B-ch. at IOM ch. 0 */
          c->i_adf2         |=  0x80; /* select IOM mode 2 */
          c->i_cirq         |=  0x03; /* select IOM mode 2 */
          c->h_tsax[0]       =  0x2f; /* tx slot 1 */
          c->h_tsar[0]       =  0x2f; /* rx slot 1 */
          c->h_tsax[1]       =  0x03; /* tx slot 0 */
          c->h_tsar[1]       =  0x03; /* rx slot 0 */
        }

        if(sc->sc_default.o_8KFIFO)
        {
          c->s_cirm_0       |= 0x10; /* 8K fifo mode */
          c->s_cirm         |= 0x10; /* 8K fifo mode */
        }

        if(IS_LOCAL_LOOP(sc,0))
        {
          c->a_lmr2         |= 0x02; /* D-CH */
          c->i_spcr         |= 0x10;
          c->w_cmdr1        |= 0x02;
          c->w_d_mode       |= 0x02;
        }

        if(IS_REMOTE_LOOP(sc,0))
        {
          c->a_lmr2         |= 0x01; /* D-CH */
          c->w_cmdr1        |= 0x01;
          c->w_d_mode       |= 0x01;
        }

        /*
         * The hardware interrupt signal should be
         * switched to an unused pin, when
         * enabling POLLED_MODE.
         *
         * NOTE: in polled mode all interrupt
         *       masks must remain. Currently
         *       the ISAC/HSCX chips does not
         *       support that  the  interrupt
         *       output may be switched off.
         *       If you have an ISA card, you
         *       may have to remove the IRQ
         *       jumper...
         *
         * NOTE: PCI and PnP controllers support
         *       disabling  of  interrupts    in
         *       hardware,    but the kernel has
         *       no routines for this (TODO)
         */

        if(IS_POLLED_MODE(sc,0))
        {
                c->s_cirm_0     &= ~0x07; /* switch intr. to unused pin */
                c->s_cirm       &= ~0x07; /* switch intr. to unused pin */
                c->s_int_m2     &= ~0x08; /* interrupt output disable */
                c->s_int_m2_pci &= ~0x08; /* interrupt output disable */
                /* timer is started by a ``__ihfc_chip_interrupt()'' call */
        }

        /*
         * Select line-mode:
         *
         * NOTE: TE-mode connects to NT-mode and
         *       NT-mode connects to TE-mode.
         *       (TErminal-mode is default)
         */

        if(IS_NT_MODE(sc,0))
        {
                c->s_sctrl         |= 0x44; /* NT+non-cap line mode */
                c->s_clkdel        &= ~0x7f;
                c->s_clkdel        |= (sc->sc_default.stdel_nt);
                
                if(sc->sc_default.o_ISAC_NT)
		{
                  c->i_stcr          &= ~0x70; /* TIC bus address = 0 */
                  c->b_cir0          &= ~0x0e; /* TIC bus address = 0 */
                  c->i_mode          |=  0x02; /* ISAC NT-mode(?) */
                }
		else
		{
		  /* try to switch data-lines */
                  c->i_spcr          |= 0x40;
		}
        }

        /*
         * Select low D-channel priority:
         *
         * All D-channel data sent to the NT, is
         * echoed back using the Echo-channel.
         * This allows sharing of the D-channel.
         *
         * By default all devices send ``one''
         * bits to the D-channel. Devices that
         * detect a ``zero'' bit when a ``one''
         * bit was sent, must start sending
         * ``one'' bits until 8 or 10 ``one''
         * bits, which number is called the
         * ``priority level'', are detected in the
         * Echo-channel.
         *
         * The ``priority level'' is incremented
         * following a successfully transmitted
         * frame and decremented when a higher
         * count has been satisfied.
         *
         * High priority means counting eight 1's
         * in the echo channel.      Low priority
         * means counting  ten  1's  in  the echo
         * channel.
         *
         * The D-channel ``bit-echo-back-delay''
         * limits the maximum length of an ISDN
         * line.
         */

        if(IS_DLOWPRI(sc,0))
        {
                c->a_lpr        |=  0x01;
                c->b_cir0       |=  0x10;
                c->i_cirq       |=  0x04;
                c->s_sctrl      |=  0x08;
        }

        /* reset and reload
         * configuration
         */
        ihfc_reset(sc,error);

        IHFC_UNLOCK(sc);

        return IHFC_IS_ERR(error);
}

/*---------------------------------------------------------------------------*
 * : shutdown chip, disable sending of data to cable
 *
 * NOTE: For Teles 16.3c and AcerISDN P10 the oscillator must _not_ be
 *       powered down hence this will stop the external PnP-emulator.
 *       Without the PnP-emulator the boards will not be detected after
 *       soft-reboot.
 * NOTE: Disabling ISAC interrupts will automatically cause an CIRQ IRQ,
 *       so ISAC interrupts are left enabled.
 * NOTE: ISAC should be left in power-up state because external PnP-chips may
 *       depend on the oscillator.
 *---------------------------------------------------------------------------*/
void
ihfc_unsetup_softc(register ihfc_sc_t *sc)
{
        ihfc_fifo_t *f;

        IHFC_LOCK(sc);

        FIFO_FOREACH(f,sc)
        {
          /* disable all channels
           *
           * A T125 wait condition may be active
           * when the CPU gets here, but this will
           * not block updating the configuration.
           */
          f->prot_curr.protocol_1 = P_DISABLE;
          ihfc_fifo_setup(sc,f);
        }

        /* hfc (2b) */
        c->s_cirm_0          &= ~0x07; /* disable interrupts */

        /* hfc (s/sp) */
        c->s_int_m1          &=  0x00; /* disable interrupts */
        c->s_int_m2          &= ~0x09; /* disable interrupts */
        c->s_sctrl           &= ~0x83; /* send 1's only + enable oscillator */

        /* hfc (spci) */
        c->s_int_m2_pci      &= ~0x09; /* disable interrupts */
        c->s_fifo_en         &=  0x00; /* disable all fifos */

#if 0
        /* hfc (susb) */
        c->s_int_m2_usb      &= ~0x84; /* disable interrupts */
#endif

        /* isac */
        c->i_spcr            &= ~0x0f; /* send 1's only */

        /* hscx */
        c->h_mode[0]         &= ~0x04; /* send disable */
        c->h_mode[1]         &= ~0x04; /* send disable */
        c->h_ccr1[0]         &= ~0x80; /* power down   */
        c->h_ccr1[1]         &= ~0x80; /* power down   */

        /* tiger300 */
        c->t_prct            |=  0x0F; /* ISAC+TIGER reset */

        /* wibusb */
        c->w_cmdr1           &= ~0x30; /* D-ch(RX+TX) disable */
        c->w_cmdr2           &= ~0x33; /* B1/B2-ch(RX+TX) disable */

        /* wibpci */
        c->w_imask           |=  0xff; /* disable all interrupts */
        c->w_b1_mode         &= ~0x40; /* 0xff as inter frame fill */
        c->w_b2_mode         &= ~0x40; /* 0xff as inter frame fill */

        /* write the config (flush all registers) */
        ihfc_config_write(sc, IHFC_CONFIG_WRITE_RELOAD);

        IHFC_UNLOCK(sc);
}
