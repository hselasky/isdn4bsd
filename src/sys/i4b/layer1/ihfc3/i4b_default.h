#ifndef _I4B_DEFAULT_H_
#define _I4B_DEFAULT_H_

static void
default_chip_reset CHIP_RESET_T(sc,error)
{
	return;
}

static void
default_chip_status_read CHIP_STATUS_READ_T(sc)
{
	return;
}

static void
default_chip_status_check CHIP_STATUS_CHECK_T(sc)
{
	/* NOTE: some drivers depend on this
	 * routine checking both HFC and IPAC
	 * interrupts!
	 */

	/*
	 * HSCX interrupts
	 */

	if(sc->sc_config.h_ista)
	{
#if 0
	  if((sc->sc_config.h_ista & 0x00c0) == 0x00c0) IHFC_ERR("RME and RPF\n");
	  if((sc->sc_config.h_ista & 0xc000) == 0xc000) IHFC_ERR("RME and RPF\n");
#endif
	  /* RME or RPF - B1 channel receive */
	  if(sc->sc_config.h_ista & 0x00c0)
	  {
		QFIFO(sc,hi,b1r);
		sc->sc_fifo[b1r].i_ista |=
		      (sc->sc_config.h_ista & 0x0080) ?
			I_ISTA_RME :
			I_ISTA_RPF ;
	  }

	  /* RME or RPF - B2 channel receive */	
	  if(sc->sc_config.h_ista & 0xc000)
	  {
		QFIFO(sc,hi,b2r);
		sc->sc_fifo[b2r].i_ista |=
		      (sc->sc_config.h_ista & 0x8000) ?
			I_ISTA_RME :
			I_ISTA_RPF ;
	  }

	  /* XPR - B1 channel transmit */
	  if(sc->sc_config.h_ista & 0x0010)
	  {
		QFIFO(sc,hi,b1t);
		sc->sc_fifo[b1t].i_ista |= I_ISTA_XPR;
	  }

	  /* XPR - B2 channel transmit */
	  if(sc->sc_config.h_ista & 0x1000)
	  {
		QFIFO(sc,hi,b2t);
		sc->sc_fifo[b2t].i_ista |= I_ISTA_XPR;
	  }

	  /* check EXIR A and EXIR B */
	  if(sc->sc_config.h_ista & 0x0300)
	  {
	    /* RFO - B1 channel receive */
	    if(sc->sc_config.h_exir & 0x0010)
	    {
		QFIFO(sc,hi,b1r);
		sc->sc_fifo[b1r].i_ista |= I_ISTA_ERR;
	    }
	    /* RFO - B2 channel receive */
	    if(sc->sc_config.h_exir & 0x1000)
	    {
		QFIFO(sc,hi,b2r);
		sc->sc_fifo[b2r].i_ista |= I_ISTA_ERR;
	    }
	    /* XDU - B1 channel transmit */
	    if(sc->sc_config.h_exir & 0x0040)
	    {
		QFIFO(sc,hi,b1t);
		sc->sc_fifo[b1t].i_ista |= I_ISTA_ERR;
	    }
	    /* XDU - B2 channel transmit  */
	    if(sc->sc_config.h_exir & 0x4000)
	    {
		QFIFO(sc,hi,b2t);
		sc->sc_fifo[b2t].i_ista |= I_ISTA_ERR;
	    }
	    /* clear all interrupts */
	    sc->sc_config.h_exir = 0;
	  }
	  /* clear all interrupts */
	  sc->sc_config.h_ista = 0;
	}

	/*
	 * ISAC interrupts
	 */

	if(sc->sc_config.i_ista)
	{
	  /* CISQ */
	  if(sc->sc_config.i_ista & 0x04)
	  {
		ihfc_fsm_update(sc,&sc->sc_fifo[0],0);
	  }

	  /* RME or RPF - D channel receive */
	  if(sc->sc_config.i_ista & 0xc0)
	  {	 
		QFIFO(sc,hi,d1r);
		sc->sc_fifo[d1r].i_ista |=
		      (sc->sc_config.i_ista & 0x80) ?
			I_ISTA_RME :
			I_ISTA_RPF ;
	  }

	  /* XPR - D channel transmit */
	  if(sc->sc_config.i_ista & 0x10)
	  {
		QFIFO(sc,hi,d1t);
		sc->sc_fifo[d1t].i_ista |= I_ISTA_XPR;
	  }

	  /* check EXIR */
	  if(sc->sc_config.i_ista & 0x01)
	  {
	    /* RFO - D channel receive */
	    if(sc->sc_config.i_exir & 0x10)
	    {
		QFIFO(sc,hi,d1r);
		sc->sc_fifo[d1r].i_ista |= I_ISTA_ERR;
	    }

	    /* XDU or XCOL - D channel transmit */
	    if(sc->sc_config.i_exir & (0x40 /* XDU */ | 0x80 /* XCOL */))
	    {
		QFIFO(sc,hi,d1t);
		sc->sc_fifo[d1t].i_ista |= I_ISTA_ERR;
	    }

	    /* clear all interrupts */
	    sc->sc_config.i_exir = 0;
	  }

	  /* TIN */
	  if(sc->sc_config.i_ista & 0x08)
	  {
		QFIFO(sc,lo,d1t);
		QFIFO(sc,lo,b1t);
		QFIFO(sc,lo,b2t);
	  }
	  /* clear all interrupts */
	  sc->sc_config.i_ista = 0;
    	}

	/*
	 * HFC-1/S/SP/SPCI interrupts
	 */

	if(sc->sc_config.s_int_s1)
	{
	  /* statemachine changed */
	  if(sc->sc_config.s_int_s1 & 0x40)
	  {
		ihfc_fsm_update(sc,&sc->sc_fifo[0],0);
	  }

	  /* Timer 64ms, 50ms, 25ms ... */
	  if(sc->sc_config.s_int_s1 & 0x80)
	  {
		QFIFO(sc,lo,b1r);
		QFIFO(sc,lo,b1t);
		QFIFO(sc,lo,b2r);
		QFIFO(sc,lo,b2t);
		QFIFO(sc,lo,d1r);
		QFIFO(sc,lo,d1t);
	  }
#if 0
	  /* B1 channel RX interrupt */
	  if(sc->sc_config.s_int_s1 & 0x08)
	  {
		QFIFO(sc,lo,b1r);
	  }

	  /* B2 channel RX interrupt */
	  if(sc->sc_config.s_int_s1 & 0x10)
	  {
		QFIFO(sc,lo,b2r);
	  }

	  /* D1 channel RX interrupt */
	  if(sc->sc_config.s_int_s1 & 0x20)
	  {
		QFIFO(sc,lo,d1r);
	  }

	  /* B1 channel TX interrupt */
	  if(sc->sc_config.s_int_s1 & 0x01)
	  {
		QFIFO(sc,lo,b1t);
	  }

	  /* B2 channel TX interrupt */
	  if(sc->sc_config.s_int_s1 & 0x02)
	  {
		QFIFO(sc,lo,b2t);
	  }

	  /* D1 channel TX interrupt */
	  if(sc->sc_config.s_int_s1 & 0x04)
	  {
		QFIFO(sc,lo,d1t);
	  }
#endif
	  /* clear all interrupts */
	  sc->sc_config.s_int_s1 = 0;
	}

	return;
}

static void
default_chip_unselect CHIP_UNSELECT_T(sc)
{
	return;
}

static void
default_chip_read CHIP_READ_T(sc,reg,ptr,len)       	
{
	bzero(ptr, len);
	return;
}

static void
default_chip_write CHIP_WRITE_T(sc,reg,ptr,len)		
{
	return;
}

static void
default_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)		
{
	if((f == IHFC_CONFIG_WRITE_UPDATE) ||
	   (f == IHFC_CONFIG_WRITE_RELOAD)) {
		ihfc_config_write_sub(sc,f);
	}
	return;
}

static void
default_fsm_read FSM_READ_T(sc,f,ptr)
{
	ptr[0] = 0;
	return;
}

static void
default_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	return;
}

#if 0
static void
default_fifo_read FIFO_READ_T(sc,f,ptr,len)		
{
	bzero(ptr, len);
	return;
}

static void
default_fifo_write FIFO_WRITE_T(sc,f,ptr,len)		
{
	return;
}
#endif

static void
default_fifo_write_filler FIFO_WRITE_FILLER_T(sc,f)
{
	return;
}

static void
default_fifo_get_memory FIFO_GET_MEMORY_T(sc,f,p_start,p_end,p_len)
{
	*p_len = MIN(sizeof(sc->sc_buffer), f->Z_chip);
	*p_start = &sc->sc_buffer[0];
	*p_end = &sc->sc_buffer[*p_len];

	IHFC_MSG("len=%d\n", *p_len);
	return;
}

static void
default_fifo_select FIFO_SELECT_T(sc,f)
{
	return;
}

static void
default_fifo_inc_fx_pre FIFO_INC_FX_PRE_T(sc,f)
{
	return;
}

static void
default_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	return;
}

static void
default_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	f->Z_chip = f->Z_drvr = 0;
	f->F_chip = f->F_drvr = 0;
	return;
}

static uint8_t
default_fifo_frame_check FIFO_FRAME_CHECK_T(sc,f,m)
{
	return 0; /* valid frame */
}

register_list_t
default_register_list[] =
{
	{ 0, 0 }
};

fsm_t
default_fsm_table;

I4B_DBASE(default_sc_default)
{
  /* defaults for multiplexed routines */

  I4B_DBASE_ADD(c_chip_reset		, &default_chip_reset);
  I4B_DBASE_ADD(c_chip_unselect		, &default_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_check	, &default_chip_status_check);
  I4B_DBASE_ADD(c_chip_status_read	, &default_chip_status_read);
  I4B_DBASE_ADD(c_chip_read		, &default_chip_read);
  I4B_DBASE_ADD(c_chip_write		, &default_chip_write);
  I4B_DBASE_ADD(c_chip_config_write	, &default_chip_config_write);

  I4B_DBASE_ADD(c_fsm_read		, &default_fsm_read);
  I4B_DBASE_ADD(c_fsm_write		, &default_fsm_write);
  I4B_DBASE_ADD(d_fsm_table		, &default_fsm_table);

#if 0
  /* these must always be implemented */
  I4B_DBASE_ADD(c_fifo_read		, &default_fifo_read);
  I4B_DBASE_ADD(c_fifo_write		, &default_fifo_write);
#endif
  I4B_DBASE_ADD(c_fifo_select		, &default_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre	, &default_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_inc_fx		, &default_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read		, &default_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_frame_check	, &default_fifo_frame_check);
  I4B_DBASE_ADD(c_fifo_write_filler	, &default_fifo_write_filler);
  I4B_DBASE_ADD(c_fifo_get_memory       , &default_fifo_get_memory);

  /* delay 50 milliseconds (default) */
  I4B_DBASE_ADD(d_interrupt_delay       , hz / 20);
  I4B_DBASE_ADD(d_register_list		, &default_register_list[0]);
}

#endif /* _I4B_DEFAULT_H_ */
