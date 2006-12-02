/*-
 * Copyright (c) 1998-2003 Cologne Chip AG. All rights reserved.
 *
 * Copyright (c) 1998-2003 Moving Bytes Communications, 
 *                         Systementwicklung GmbH. All rights reserved.
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
 *	i4b_hfc4s8s.h - HFC-4S/8S PCI driver module, Basic Rate
 * 	-------------------------------------------------------
 *
 * $FreeBSD: $
 *
 * The chip specification is available from: http://www.colognechip.com
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFC4S8S_H_
#define _I4B_HFC4S8S_H_

#include <i4b/layer1/ihfc2/i4b_hfc.h>

/* imports */

#define hfc4s8s_fifo_inc_fx_pre     default_fifo_inc_fx_pre
#define hfc4s8s_fifo_frame_check    hfcs_fifo_frame_check
#define hfc4s8s_fsm_table           hfc_fsm_table

/* HFC-4S/8S registers used by this driver,
 * sorted alphabetically:
 */

/* write registers */

#define REG_hfc4s8s_a_ch_msk_write 0xF4 
#define REG_hfc4s8s_a_channel_write 0xFC 
#define REG_hfc4s8s_a_con_hdlc_write 0xFA 
#define REG_hfc4s8s_a_conf_write 0xD1 
#define REG_hfc4s8s_a_fifo_data_noinc_write 0x84 
#define REG_hfc4s8s_a_fifo_seq_write 0xFD 
#define REG_hfc4s8s_a_inc_res_fifo_write 0x0E 
#define REG_hfc4s8s_a_irq_msk_write 0xFF 
#define REG_hfc4s8s_a_sl_cfg_write 0xD0 
#define REG_hfc4s8s_a_st_b1_tx_write 0x3C 
#define REG_hfc4s8s_a_st_b2_tx_write 0x3D 
#define REG_hfc4s8s_a_st_clk_dly_write 0x37 
#define REG_hfc4s8s_a_st_ctrl0_write 0x31 
#define REG_hfc4s8s_a_st_ctrl1_write 0x32 
#define REG_hfc4s8s_a_st_ctrl2_write 0x33 
#define REG_hfc4s8s_a_st_d_tx_write 0x3E 
#define REG_hfc4s8s_a_st_sq_write 0x34 
#define REG_hfc4s8s_a_st_sta_write 0x30 
#define REG_hfc4s8s_a_subch_cfg_write 0xFB 

#define REG_hfc4s8s_r_bert_wd_md_write 0x1B 
#define REG_hfc4s8s_r_brg_pcm_cfg_write 0x02 
#define REG_hfc4s8s_r_cirm_write 0x00 
#define REG_hfc4s8s_r_conf_en_write 0x18 
#define REG_hfc4s8s_r_ctrl_write 0x01 
#define REG_hfc4s8s_r_dtmf_n_write 0x1D 
#define REG_hfc4s8s_r_dtmf_write 0x1C 
#define REG_hfc4s8s_r_fifo_md_write 0x0D 
#define REG_hfc4s8s_r_fifo_write 0x0F 
#define REG_hfc4s8s_r_first_fifo_write 0x0B 
#define REG_hfc4s8s_r_fsm_idx_write 0x0F 
#define REG_hfc4s8s_r_gpio_en0_write 0x42 
#define REG_hfc4s8s_r_gpio_en1_write 0x43 
#define REG_hfc4s8s_r_gpio_out0_write 0x40 
#define REG_hfc4s8s_r_gpio_out1_write 0x41 
#define REG_hfc4s8s_r_gpio_sel_write 0x44 
#define REG_hfc4s8s_r_irq_ctrl_write 0x13 
#define REG_hfc4s8s_r_irqmsk_misc_write 0x11 
#define REG_hfc4s8s_r_pcm_md0_write 0x14 
#define REG_hfc4s8s_r_pcm_md1_write 0x15 
#define REG_hfc4s8s_r_pcm_md2_write 0x15 
#define REG_hfc4s8s_r_pwm0_write 0x38 
#define REG_hfc4s8s_r_pwm1_write 0x39 
#define REG_hfc4s8s_r_pwm_md_write 0x46 
#define REG_hfc4s8s_r_ram_addr0_write 0x08 
#define REG_hfc4s8s_r_ram_addr1_write 0x09 
#define REG_hfc4s8s_r_ram_addr2_write 0x0A 
#define REG_hfc4s8s_r_ram_misc_write 0x0C
#define REG_hfc4s8s_r_sci_msk_write 0x12 
#define REG_hfc4s8s_r_sh0h_write 0x15 
#define REG_hfc4s8s_r_sh0l_write 0x15 
#define REG_hfc4s8s_r_sh1h_write 0x15 
#define REG_hfc4s8s_r_sh1l_write 0x15 
#define REG_hfc4s8s_r_sl_sel0_write 0x15 
#define REG_hfc4s8s_r_sl_sel1_write 0x15 
#define REG_hfc4s8s_r_sl_sel2_write 0x15 
#define REG_hfc4s8s_r_sl_sel3_write 0x15 
#define REG_hfc4s8s_r_sl_sel4_write 0x15 
#define REG_hfc4s8s_r_sl_sel5_write 0x15 
#define REG_hfc4s8s_r_sl_sel6_write 0x15 
#define REG_hfc4s8s_r_sl_sel7_write 0x15 
#define REG_hfc4s8s_r_slot_write 0x10 
#define REG_hfc4s8s_r_st_sel_write 0x16 
#define REG_hfc4s8s_r_st_sync_write 0x17 
#define REG_hfc4s8s_r_ti_wd_write 0x1A 

/* read registers */

#define REG_hfc4s8s_a_f12_read 0x0C 
#define REG_hfc4s8s_a_f1_read 0x0C 
#define REG_hfc4s8s_a_f2_read 0x0D 
#define REG_hfc4s8s_a_st_b1_rx_read 0x3C 
#define REG_hfc4s8s_a_st_b2_rx_read 0x3D 
#define REG_hfc4s8s_a_st_d_rx_read 0x3E 
#define REG_hfc4s8s_a_st_e_rx_read 0x3F 
#define REG_hfc4s8s_a_st_sta_read 0x30 
#define REG_hfc4s8s_a_st_sq_read 0x34 
#define REG_hfc4s8s_a_z12_read 0x04 
#define REG_hfc4s8s_a_z1_read 0x04 
#define REG_hfc4s8s_a_z1h_read 0x05 
#define REG_hfc4s8s_a_z1l_read 0x04 
#define REG_hfc4s8s_a_z2_read 0x06 
#define REG_hfc4s8s_a_z2h_read 0x07 
#define REG_hfc4s8s_a_z2l_read 0x06 

#define REG_hfc4s8s_r_bert_ech_read 0x1B 
#define REG_hfc4s8s_r_bert_ecl_read 0x1A 
#define REG_hfc4s8s_r_bert_sta_read 0x17 
#define REG_hfc4s8s_r_chip_id_read 0x16 
#define REG_hfc4s8s_r_chip_rv_read 0x1F 
#define REG_hfc4s8s_r_conf_oflow_read 0x14 
#define REG_hfc4s8s_r_f0_cnth_read 0x19 
#define REG_hfc4s8s_r_f0_cntl_read 0x18 
#define REG_hfc4s8s_r_gpi_in0_read 0x44 
#define REG_hfc4s8s_r_gpi_in1_read 0x45 
#define REG_hfc4s8s_r_gpi_in2_read 0x46 
#define REG_hfc4s8s_r_gpi_in3_read 0x47 
#define REG_hfc4s8s_r_gpio_in0_read 0x40 
#define REG_hfc4s8s_r_gpio_in1_read 0x41 
#define REG_hfc4s8s_r_int_data_read 0x88 
#define REG_hfc4s8s_r_irq_fifo_bl0_read 0xC8 
#define REG_hfc4s8s_r_irq_fifo_bl1_read 0xC9 
#define REG_hfc4s8s_r_irq_fifo_bl2_read 0xCA 
#define REG_hfc4s8s_r_irq_fifo_bl3_read 0xCB 
#define REG_hfc4s8s_r_irq_fifo_bl4_read 0xCC 
#define REG_hfc4s8s_r_irq_fifo_bl5_read 0xCD 
#define REG_hfc4s8s_r_irq_fifo_bl6_read 0xCE 
#define REG_hfc4s8s_r_irq_fifo_bl7_read 0xCF 
#define REG_hfc4s8s_r_irq_misc_read 0x11 
#define REG_hfc4s8s_r_irq_oview_read 0x10 
#define REG_hfc4s8s_r_ram_use_read 0x15 
#define REG_hfc4s8s_r_sci_read 0x12 
#define REG_hfc4s8s_r_status_read 0x1C

/* read/write registers */

#define REG_hfc4s8s_a_fifo_data 0x80 
#define REG_hfc4s8s_r_ram_data 0xC0 

#define HFC4S8S_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.mem_hdl[0];

#define HFC4S8S_READ_1(reg,var)			\
	(var) = bus_space_read_1(t,h,(reg))

#define HFC4S8S_WRITE_1(reg,var)		\
	bus_space_write_1(t,h,(reg),(var))

static u_int8_t
hfc4s8s_stable_read_1(ihfc_sc_t *sc, bus_size_t offset)
{
	HFC4S8S_BUS_VAR(sc);
	register u_int8_t temp1;
	register u_int8_t temp2;
	register u_int8_t to = 16;

	temp2 = bus_space_read_1(t,h,offset);

	do {
	    temp1 = bus_space_read_1(t,h,offset);

	    if(temp2 == temp1)
	    {
	        break;
	    }
	    temp2 = temp1;

	} while(--to);

	if((to == 0) && sc->sc_default.o_PRIVATE_FLAG_1)
	{
	    sc->sc_default.o_PRIVATE_FLAG_1 = 0;

	    IHFC_ERR("stable read timed out, "
		     "offset=0x%02x!", 
		     (u_int32_t)offset);
	}
	return temp1;
}

static void
hfc4s8s_fifo_wait_disbusy(ihfc_sc_t *sc)
{
	HFC4S8S_BUS_VAR(sc);
	u_int8_t temp = 1;
	u_int16_t to = 500;

	/* wait 15us at 33MHz or
	 * 7.5us at 66MHz
	 *
	 * maximum delay is 1us according
	 * to manual
	 */
	while(--to && (temp & 1))
	{
	    HFC4S8S_READ_1(REG_hfc4s8s_r_status_read, temp);
	}

	if(to == 0)
	{
	    /* don't post the error too many times,
	     * hence this function is called 
	     * very often !
	     */
	    if(sc->sc_default.o_PRIVATE_FLAG_1)
	    {
	        sc->sc_default.o_PRIVATE_FLAG_1 = 0;

		IHFC_ERR("wait disbusy timed out!\n");
	    }
	    else
	    {
	        IHFC_MSG("wait disbusy timed out!\n");
	    }
	}
	return;
}

static void
hfc4s8s_chip_slots_init(ihfc_sc_t *sc)
{
	/* not used */
	return;
}

static void
hfc4s8s_chip_reset CHIP_RESET_T(sc,error)
{
	HFC4S8S_BUS_VAR(sc);
	u_int16_t temp;
	u_int16_t unit;
	u_int16_t Z_min; /* inclusive */
	u_int16_t Z_max; /* exclusive */
	u_int8_t F_min; /* inclusive */
	u_int8_t F_max; /* exclusive */
	u_int8_t pcm_md0;
	ihfc_fifo_t *f;

#if 0
	/* clear the CIP registers, hence these are
	 * not cleared by reset (not used by 
	 * memory mapped I/O)
	 */
	bus_space_write_1(t,h,4,0);
	bus_space_write_1(t,h,5,0);
	bus_space_write_1(t,h,6,0);
	bus_space_write_1(t,h,7,0);
#endif
	/* set clock speed */

	temp = (sc->sc_default.double_clock ? 0x20 : 0x00);

	HFC4S8S_WRITE_1(REG_hfc4s8s_r_brg_pcm_cfg_write, temp);

	temp = 0;

	if(sc->sc_default.o_EXTERNAL_RAM)
	{
	    temp |= 8;
	}
#if 0
	/*
	 * existing boards are 33MHz only, so
	 * the following is always false
	 */
	if(66 MHZ PCI BUS)
	{
	    temp |= 4;
	}
#endif
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_ctrl_write, temp);

	/* NOTE: counter maximum values are exclusive here,
	 * but in the manual they are inclusive !
	 */
	if(sc->sc_default.o_EXTERNAL_RAM)
	{
	    if(sc->sc_default.o_512KFIFO)
	    {
	        /* 512K RAM */
	        F_min = 0x00;
		F_max = 0x20;
		Z_min = 0x00C0;
		Z_max = 0x2000;
		temp = 2;
	    }
	    else
	    {
	        /* 128K */
	        F_min = 0x00;
		F_max = 0x20;
		Z_min = 0x00C0;
		Z_max = 0x0800;
		temp = 1;
	    }
	}
	else
	{
	    F_min = 0x00;
	    F_max = 0x10;
	    Z_min = 0x0080;
	    Z_max = 0x0200;
	    temp = 0;
	}

	/* set "FZ_MD" bit so that the Z2 counter, 
	 * in transmit direction, points to the 
	 * current chip read position, and not to 
	 * the beginning of the current frame:
	 */
	temp |= 0x80;

	HFC4S8S_WRITE_1(REG_hfc4s8s_r_ram_misc_write, temp);

	/* initialize FIFO maps */

	FIFO_FOREACH(f,sc)
	{
	    struct sc_state *st;

	    f->fm.h.Zsize = Z_max - Z_min;
	    f->fm.h.Fsize = F_max - F_min;
	    f->fm.h.Zend  = Z_max;

	    f->sub_unit = FIFO_NO(f) / 6;
	    FIFO_LOGICAL_NO(f) = FIFO_NO(f) % 6;

	    st = &sc->sc_state[f->sub_unit];

	    st->i4b_controller->L1_fifo = &sc->sc_fifo[f->sub_unit * 6];
	    st->i4b_controller->L1_channel_end = 3;

	    /* need to reorder the FIFO's so that
	     * the D-channel is first and 
	     * the B-channels last
	     */
	    temp = (f->sub_unit * 8) | FIFO_DIR(f);

	    switch(FIFO_LOGICAL_NO(f) / 2) {
	    case 1:
	      temp |= 0; /* B1-channel */
	      break;
	    case 2:
	      temp |= 2; /* B2-channel */
	      break;
	    default:
	      temp |= 4; /* D-channel */
	      break;
	    }

	    f->s_fifo_sel = temp;
	}

	/* reset chip */

	HFC4S8S_WRITE_1(REG_hfc4s8s_r_cirm_write, 0x08);
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_cirm_write, 0x00);

	DELAY(1000); /* wait 1 ms */

	/* read status */
	HFC4S8S_READ_1(REG_hfc4s8s_r_status_read, temp);
	if(temp & 0x01)
	{
	    IHFC_ADD_ERR(error, "(BUSY != 0)");
	}

	HFC4S8S_WRITE_1(REG_hfc4s8s_r_fifo_md_write, 0x00);

	/* initialize internal data */

	if(sc->sc_default.o_EXTERNAL_RAM)
	  temp = 0x2800;
	else
	  temp = 0x1800;

	do {
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_ram_addr0_write, (temp & 0xFF));
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_ram_addr1_write, (temp >> 8));
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_ram_addr2_write, (0x00));
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_ram_data, 0xFF);

	    temp++;
	} while(temp & 0xFF);

	/* 1. PCM interface */

	/* disable all time slots */

	for(temp = 0; temp < 256; temp++)
	{
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_slot_write, temp);
	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_sl_cfg_write, 0x00);
	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_conf_write, 0x00);
	}

	pcm_md0 = (IS_PCM_SLAVE(sc,0) ? 0x00 : 0x01);

	/* PCM slave or master, and select md1 register */
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_pcm_md0_write, pcm_md0|0x90);

	/* set PCM speed */
 	HFC4S8S_WRITE_1(REG_hfc4s8s_r_pcm_md1_write, 
			IS_PCM_SPEED_128(sc,0) ? 0x20 :
			IS_PCM_SPEED_64(sc,0) ? 0x10 : 0x00);

	/* select md2 register */
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_pcm_md0_write, pcm_md0|0xA0);

	/* PCM is synchronized to E1 receive */
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_pcm_md2_write, 0x00);

	/* wait until PCM reset sequence is finished */
	DELAY(4*125);

	hfc4s8s_chip_slots_init(sc);

	for(unit = 0; 
	    unit < sc->sc_default.d_sub_controllers; 
	    unit++)
	{
	    /* select S/T interface */
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_st_sel_write, unit);

	    temp = IS_NT_MODE(sc,unit) ? 0x6C : 0x0E;

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_st_clk_dly_write, temp);

	    temp = 0x03; /* enable B-transmit-channels */

	    if(IS_DLOWPRI(sc,unit)) temp |= 0x08;
	    if(IS_NT_MODE(sc,unit)) temp |= 0x04;

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_st_ctrl0_write, temp);

	    temp = 0x00; /* reset default  */

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_st_ctrl1_write, temp);

	    temp = 0x03; /* enable B-receive-channels */

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_st_ctrl2_write, temp);
	}

	/* enable disbusy timeout warning */

	sc->sc_default.o_PRIVATE_FLAG_1 = 1;

	/* enable global interrupt
	 * - low is active
	 */
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_irq_ctrl_write, 
			IS_POLLED_MODE(sc,0) ? 0x00 : 0x08);
	return;
}

static void
hfc4s8s_fifo_select FIFO_SELECT_T(sc,f)
{
	HFC4S8S_BUS_VAR(sc);

	HFC4S8S_WRITE_1(REG_hfc4s8s_r_st_sel_write, f->sub_unit);

	HFC4S8S_WRITE_1(REG_hfc4s8s_r_fifo_write, f->s_fifo_sel);

	hfc4s8s_fifo_wait_disbusy(sc);

	sc->sc_fifo_select_last = f;

	return;
}

static void
hfc4s8s_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	HFC4S8S_BUS_VAR(sc);

	struct sc_fifo *f_last;
	u_int8_t temp;
	u_int8_t cable;

	if((f == CONFIG_WRITE_UPDATE) ||
	   (f == CONFIG_WRITE_RELOAD))
	{
	    /* enable interrupts:
	     * - statemachine change
	     * - timer elapsed
	     */
	    temp = 0x0C; /* enable timer, 1.024s */

	    if(ihfc_fifos_active(sc))
	    {
	        /* enable timer, 32ms */
	        temp = 0x07;
	    }

	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_ti_wd_write, temp);

	    /*
	     * enabling interrupts can generate
	     * a new interrupt immediately !
	     */
	    HFC4S8S_WRITE_1(REG_hfc4s8s_r_irqmsk_misc_write, 0x02);
	}
	else
	{
	    f_last = sc->sc_fifo_select_last;

	    hfc4s8s_fifo_select(sc,f);

	    if(f->prot_last.protocol_1 == P_BRIDGE)
	    {
	        temp = (FIFO_DIR(f) == transmit) ?
		  ((f->prot_last.u.bridge.tx_slot << 1)|transmit) :
		  ((f->prot_last.u.bridge.rx_slot << 1)|receive);

		/* disable time slot */
		HFC4S8S_WRITE_1(REG_hfc4s8s_r_slot_write, temp);
		HFC4S8S_WRITE_1(REG_hfc4s8s_a_sl_cfg_write, 0x00);
	    }

	    if((f->prot_curr.protocol_1 == P_DISABLE) &&
	       (FIFO_DIR(f) == receive))
	    {
	        /* disable FIFO */
	        HFC4S8S_WRITE_1(REG_hfc4s8s_a_con_hdlc_write, 0x00);
		goto fifo_unselect;
	    }

	    if(f->prot_curr.protocol_1 == P_BRIDGE)
	    {
	        temp = 0xC2;
	    }
	    else
	    {
	        if(FIFO_LOGICAL_NO(f) == d1t)
		  temp = 1; /* 0xFF inter frame fill */
		else 
		  temp = 0; /* 0x7E inter frame fill */

	        if(PROT_IS_TRANSPARENT(&(f->prot_curr)) ||
		   (f->prot_curr.protocol_1 == P_DISABLE))
		{
		    temp |= 2; /* extended transparent mode */
		}
		else
		{
		    /* recompute "Z_min_free"
		     *
		     * If there is too much data
		     * in the buffers, the response
		     * times will increase. 
		     * So put a limit on the FIFO
		     * usage:
		     */
		    f->Z_min_free = f->fm.h.Zsize - 2000; /* max 250ms */
		    if(f->Z_min_free & Z_MSB) f->Z_min_free = 0;
		}

		/*
		 * make sure that the
		 * FIFO is enabled
		 */
		temp |= 4;
	    }

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_con_hdlc_write, temp);

	    if((FIFO_LOGICAL_NO(f) == d1t) ||
	       (FIFO_LOGICAL_NO(f) == d1r))
	        temp = 0x02; /* process 2 bits */
	    else 
	        temp = 0x00; /* process 8 bits */

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_subch_cfg_write, temp);

	    /* reset FIFO and LOST bit */

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_inc_res_fifo_write, 0x06);

	    hfc4s8s_fifo_wait_disbusy(sc);

	    if(FIFO_DIR(f) == transmit)
	    {
	        /* define the bytes that are sent
		 * to the bus:
		 */
	        HFC4S8S_WRITE_1(REG_hfc4s8s_a_fifo_data_noinc_write, 0xff);
	    }

	    if(f->prot_curr.protocol_1 == P_BRIDGE)
	    {
	        temp = (FIFO_DIR(f) == transmit) ?
		  ((f->prot_curr.u.bridge.tx_slot << 1)|transmit) :
		  ((f->prot_curr.u.bridge.rx_slot << 1)|receive);

		cable = (FIFO_DIR(f) == transmit) ? 
		  (f->prot_curr.u.bridge.tx_cable) :
		  (f->prot_curr.u.bridge.rx_cable ^ 0x01);

		cable = ((cable << 6) | 0x80 | f->s_fifo_sel);

		/* enable time slot */
		HFC4S8S_WRITE_1(REG_hfc4s8s_r_slot_write, temp);
		HFC4S8S_WRITE_1(REG_hfc4s8s_a_sl_cfg_write, cable);
	    }

	fifo_unselect:

	    if(f_last)
	    {
	        /* restore the selected FIFO, in case
		 * this routine is called recursivly:
		 */
	        hfc4s8s_fifo_select(sc, f_last);
	    }
	}
	return;
}

static void
hfc4s8s_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	HFC4S8S_BUS_VAR(sc);
	u_int16_t temp;

	/* pre increment Z-counter (before "len" is changed) */
	f->Z_drvr += len;

	if(f->Z_drvr >= f->fm.h.Zend)
	{
 	   f->Z_drvr -= f->fm.h.Zsize;
	}

	/* NOTE: xxx_multi_xxx will crash if the length 
	 * is equal to zero on i386, so only call these
	 * when the length is non-zero ! This bug should 
	 * be fixed one day.
	 */
	temp = len & 3;

	if(temp)
	{
	    bus_space_read_multi_1(t,h,REG_hfc4s8s_a_fifo_data,ptr,temp);
	    ptr += temp;
	}

	len /= 4;

	if(len)
	{
	    bus_space_read_multi_4(t,h,REG_hfc4s8s_a_fifo_data,(u_int32_t *)ptr,len);
	}
	return;
}

static void
hfc4s8s_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	/* not used */
	return;
}

static void
hfc4s8s_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	HFC4S8S_BUS_VAR(sc);
	u_int16_t temp;

	if(len)
	{
	    f->last_byte = ptr[len-1];

	    /* pre increment Z-counter, before "len" is changed */
	    f->Z_drvr += len;

	    if(f->Z_drvr >= f->fm.h.Zend)
	    {
	        f->Z_drvr -= f->fm.h.Zsize;
	    }

	    temp = len & 3;
	    if(temp)
	    {
	        bus_space_write_multi_1(t,h,REG_hfc4s8s_a_fifo_data,ptr,temp);
		ptr += temp;
	    }

	    len /= 4;

	    if(len)
	    {
	        bus_space_write_multi_4(t,h,REG_hfc4s8s_a_fifo_data,(const u_int32_t *)ptr,len);
	    }
	}
	return;
}

static void
hfc4s8s_fifo_write_filler FIFO_WRITE_FILLER_T(sc,f)
{
	u_int8_t fill[32];
	u_int16_t len;

	/* XXX the byte repeating by the 
	 * chip doesn't work properly,
	 * so until further, do it 
	 * in software.
	 */

	if(f->Z_chip)
	{
	    len = min(f->Z_chip, sizeof(fill));
	    memset_1(&fill[0], f->last_byte, len);

	    do {
	        f->Z_chip -= len;
		hfc4s8s_fifo_write(sc,f,&fill[0],len);
		len = min(f->Z_chip,sizeof(fill));
	    } while(len);
	}
	return;
}

static void
hfc4s8s_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	/* not used */
	return;
}

static void
hfc4s8s_fsm_read FSM_READ_T(sc,f,ptr)
{
	HFC4S8S_BUS_VAR(sc);
	u_int8_t temp;

	/* select S/T */
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_st_sel_write, f->sub_unit);

	/* read STATES */
	HFC4S8S_READ_1(REG_hfc4s8s_a_st_sta_read, temp);
	*ptr = (temp +
		(IS_NT_MODE(sc,f->sub_unit) ? 
		 HFC_NT_OFFSET : 
		 HFC_TE_OFFSET)) & 0xf;
	return;
}

static void
hfc4s8s_fsm_write FSM_WRITE_T(sc,f,ptr)
{
	HFC4S8S_BUS_VAR(sc);

	/* select S/T */
	HFC4S8S_WRITE_1(REG_hfc4s8s_r_st_sel_write, f->sub_unit);

	/* write STATES */
	HFC4S8S_WRITE_1(REG_hfc4s8s_a_st_sta_write, *ptr);

#ifdef HFC_FSM_RESTART
	if((*ptr) & 0x10) {
	    /* 5.21us delay */

	    DELAY(10);

	    HFC4S8S_WRITE_1(REG_hfc4s8s_a_st_sta_write, ((*ptr) ^ 0x10));
	}
#endif
	return;
}

static void
hfc4s8s_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	HFC4S8S_BUS_VAR(sc);

	/* increment F-counter */

	f->F_drvr ++;
	if(f->F_drvr >= f->fm.h.Fsize)
	{
	    f->F_drvr = 0;
	}

	/* increment FIFO and clear LOST bit */

	HFC4S8S_WRITE_1(REG_hfc4s8s_a_inc_res_fifo_write, (0x04|0x01));

	hfc4s8s_fifo_wait_disbusy(sc);
#if 1
	/* NOTE: reload counter values
	 * in case cache is invalid:
	 */
	f->state &= ~ST_FZ_LOADED;
#endif
	return;
}

static void
hfc4s8s_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	HFC4S8S_BUS_VAR(sc);
	u_int8_t  Z_base;
	u_int8_t  F_base;
	u_int8_t  temp;
	u_int16_t Z_chip2;

	if(FIFO_DIR(f) == transmit)
	{
	    Z_base = 0x06; /* Z2 */
	    F_base = 0x0D; /* F2 */
	}
	else
	{
	    Z_base = 0x04; /* Z1 */
	    F_base = 0x0C; /* F1 */
	}

	cli(sc);

	/* get current time */

	HFC4S8S_READ_1(REG_hfc4s8s_r_f0_cntl_read, temp);

	f->Z_read_time = temp;

	HFC4S8S_READ_1(REG_hfc4s8s_r_f0_cnth_read, temp);

	f->Z_read_time |= (temp << 8);


	/* read F- and Z-counters respectively */

	/* fx */
	f->F_chip = hfc4s8s_stable_read_1(sc,F_base);

	/* zx - high */
	Z_chip2 = hfc4s8s_stable_read_1(sc,Z_base ^ 1) << 8;

	/* zx - low */
	f->Z_chip = hfc4s8s_stable_read_1(sc,Z_base);

	/* NOTE: it is only necessary to
	 * read F_drvr and Z_drvr once
	 * from the hardware:
	 */
	if(!(f->state &  ST_FZ_LOADED)) {
	     f->state |= ST_FZ_LOADED;

	     /* fx */
	     f->F_drvr = bus_space_read_1(t,h,F_base ^ 1);

	     /* zx - low */
	     f->Z_drvr = bus_space_read_1(t,h,Z_base ^ 2);

	     /* zx - high */
	     f->Z_drvr|= bus_space_read_1(t,h,Z_base ^ 3) << 8;
	}

	/* zx - high */
	f->Z_chip|= hfc4s8s_stable_read_1(sc,Z_base ^ 1) << 8;

	/* analyse/correct the counters */
	INC_COUNTER_TIME_CHECK(f->Z_chip, Z_chip2);

	/* extra range check */

	if(f->F_chip >= f->fm.h.Fsize)
	{
	    f->F_chip = 0;
	}

	if(f->F_drvr >= f->fm.h.Fsize)
	{
	    f->F_drvr = 0;
	}

	sti(sc);

	return;
}

static void
hfc4s8s_chip_status_read CHIP_STATUS_READ_T(sc)
{
	HFC4S8S_BUS_VAR(sc);
	register u_int8_t temp;

	/* read status */
	HFC4S8S_READ_1(REG_hfc4s8s_r_irq_misc_read, temp);

	/* timer elapsed */
	if(temp & 2)
	{
	    for(temp = 0; 
		temp < sc->sc_default.d_sub_controllers; 
		temp++)
	    {
	        ihfc_fsm_update(sc, sc->sc_state[temp].
				i4b_controller->L1_fifo, 0);
	    }

#if ((IHFC_CHANNELS < 48) || (SC_INTR_BITS != 8))
#error "please update this code, (IHFC_CHANNELS < 48) || (SC_INTR_BITS != 8)"
#endif
	    if(sc->sc_default.d_sub_controllers == 2)
	    {
	        sc->sc_intr_status[0] = 0xFF;
		sc->sc_intr_status[1] = 0x0F;
	    }
	    else if(sc->sc_default.d_sub_controllers == 4)
	    {
	        sc->sc_intr_status[0] = 0xFF;
		sc->sc_intr_status[1] = 0xFF;
		sc->sc_intr_status[2] = 0xFF;
	    }
	    else if(sc->sc_default.d_sub_controllers == 8)
	    {
	        sc->sc_intr_status[0] = 0xFF;
		sc->sc_intr_status[1] = 0xFF;
		sc->sc_intr_status[2] = 0xFF;
	        sc->sc_intr_status[3] = 0xFF;
		sc->sc_intr_status[4] = 0xFF;
		sc->sc_intr_status[5] = 0xFF;
	    }
	}
	return;
}

static void
hfc4s8s_chip_status_check CHIP_STATUS_CHECK_T(sc)
{
	/* nothing to do */
	return;
}

static void
hfc4s8s_chip_unselect CHIP_UNSELECT_T(sc)
{
	/* to start the last selected FIFO
	 * one has got to select a new FIFO
	 */
	if(QDONE(sc) && sc->sc_fifo_select_last)
	{
	    if(FIFO_DIR(sc->sc_fifo_select_last) == transmit)
	    {
	        /* start transmission of data on 
		 * last transmit FIFO:
		 */
	        hfc4s8s_fifo_select(sc, sc->sc_fifo_select_last);
	    }
	    sc->sc_fifo_select_last = NULL;
	}
	return;
}

static ihfc_fifo_program_t *
hfc4s8s_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	/* NOTE: the D-channel should only be used
	 * in HDLC-mode, though transparent
	 * D-channel mode is supported for
	 * debugging purposes:
	 */
	if(PROT_IS_HDLC(&(f->prot_curr)) ||
	   (PROT_IS_TRANSPARENT(&(f->prot_curr)) &&
	    (FIFO_LOGICAL_NO(f) != d1t) &&
	    (FIFO_LOGICAL_NO(f) != d1r)))
	{
	    program = (FIFO_DIR(f) == transmit) ? 
	      &i4b_hfc_tx_program_new :
	      &i4b_hfc_rx_program;
	}

	if(f->prot_curr.protocol_1 == P_BRIDGE)
	{
	    program = &i4b_unknown_program;
	}
	return program;
}

register_list_t
hfc4s8s_register_list [] =
{
	{ 0, 0 }
};

#if (IHFC_CHANNELS < 48)
#error "HFC-8S needs 48 channels, RX + TX"
#endif

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfc4s8s_dbase_root)
{
  I4B_DBASE_ADD(c_chip_read          , &hfc4s8s_chip_read);
  I4B_DBASE_ADD(c_chip_write         , &hfc4s8s_chip_write);
  I4B_DBASE_ADD(c_chip_reset         , &hfc4s8s_chip_reset);
  I4B_DBASE_ADD(c_chip_unselect      , &hfc4s8s_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read   , &hfc4s8s_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check  , &hfc4s8s_chip_status_check);
  I4B_DBASE_ADD(c_chip_config_write  , &hfc4s8s_chip_config_write);

  I4B_DBASE_ADD(c_fsm_read           , &hfc4s8s_fsm_read);
  I4B_DBASE_ADD(c_fsm_write          , &hfc4s8s_fsm_write);
  I4B_DBASE_ADD(d_fsm_table          , &hfc4s8s_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program   , &hfc4s8s_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read          , &hfc4s8s_fifo_read);
  I4B_DBASE_ADD(c_fifo_write         , &hfc4s8s_fifo_write);
  I4B_DBASE_ADD(c_fifo_write_filler  , &hfc4s8s_fifo_write_filler);
  I4B_DBASE_ADD(c_fifo_select        , &hfc4s8s_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx        , &hfc4s8s_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read       , &hfc4s8s_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre    , &hfc4s8s_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_frame_check   , &hfc4s8s_fifo_frame_check);

  I4B_DBASE_ADD(d_register_list      , &hfc4s8s_register_list[0]);
  I4B_DBASE_ADD(d_L1_type            , L1_TYPE_ISDN_BRI);

  /* delay 25 milliseconds */ 
  I4B_DBASE_ADD(d_interrupt_delay    , hz / 40);
 
  I4B_DBASE_ADD(o_RES_IRQ_0          , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0       , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */
  I4B_DBASE_ADD(o_ECHO_CANCEL_ENABLED, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask      , (I4B_OPTION_POLLED_MODE|
					I4B_OPTION_NT_MODE|
					I4B_OPTION_DLOWPRI|
					I4B_OPTION_PCM_SLAVE|
					I4B_OPTION_PCM_SPEED_32|
					I4B_OPTION_PCM_SPEED_64|
					I4B_OPTION_PCM_SPEED_128));
  I4B_DBASE_ADD(i4b_option_value     , (I4B_OPTION_PCM_SLAVE|
					I4B_OPTION_PCM_SPEED_64));
#if 0
  I4B_DBASE_ADD(o_EXTERNAL_RAM       , 1); /* enable */
  I4B_DBASE_ADD(o_512KFIFO           , 1); /* enable */
#endif

  I4B_DBASE_ADD(io_rid[0]            , PCIR_BAR(0));
  I4B_DBASE_ADD(mem_rid[0]           , PCIR_BAR(1));
}

/*
 * double_clock = 0: crystal frequency is 24.576 MHz
 * double_clock = 1: crystal frequency is 49.152 MHz
 */

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfc2s_dbase_root)
{
  I4B_DBASE_IMPORT(hfc4s8s_dbase_root);

  I4B_DBASE_ADD(d_channels           , 6*2);
  I4B_DBASE_ADD(d_sub_controllers    , 2);
  I4B_DBASE_ADD(desc                 , "HFC-2S PCI ISDN adapter");
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfc2s_dbase_root);
  I4B_DBASE_ADD(double_clock, 1);
}

I4B_PCI_DRIVER(/* HFC-2S */
	       .vid = 0x08b41397,
	       .sub = 0xb5661397);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfc4s_dbase_root)
{
  I4B_DBASE_IMPORT(hfc4s8s_dbase_root);

  I4B_DBASE_ADD(d_channels           , 6*4);
  I4B_DBASE_ADD(d_sub_controllers    , 4);
  I4B_DBASE_ADD(desc                 , "HFC-4S PCI ISDN adapter");
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfc4s_dbase_root);
  I4B_DBASE_ADD(double_clock, 0);
}

I4B_PCI_DRIVER(/* HFC-4S generic */
	       .vid = 0x08b41397,
	       .sub = 0x08b41397);

I4B_PCI_DRIVER(/* HFC-4S */
	       .vid = 0x903010b5,
	       .sub = 0x31361397);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfc4s_dbase_root);
  I4B_DBASE_ADD(double_clock, 1);
}

I4B_PCI_DRIVER(/* HFC-4S */
	       .vid = 0x08b41397,
	       .sub = 0xb5201397);

I4B_PCI_DRIVER(/* HFC-4S */
	       .vid = 0x08b41397,
	       .sub = 0xb5501397);

I4B_PCI_DRIVER(/* HFC-4S */
	       .vid = 0x08b41397,
	       .sub = 0xb6201397);

I4B_PCI_DRIVER(/* HFC-4S Beronet card */
	       .vid = 0x08b41397,
	       .sub = 0xb5601397);

I4B_PCI_DRIVER(/* HFC-4S Swyx QuadBRI */
	       .vid = 0x08b41397,
	       .sub = 0xb5401397);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfc8s_dbase_root)
{
  I4B_DBASE_IMPORT(hfc4s8s_dbase_root);

  I4B_DBASE_ADD(d_channels           , 6*8);
  I4B_DBASE_ADD(d_sub_controllers    , 8);
  I4B_DBASE_ADD(desc                 , "HFC-8S PCI ISDN adapter");
}

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfc8s_dbase_root);
  I4B_DBASE_ADD(double_clock, 0);
}

I4B_PCI_DRIVER(/* HFC-8S generic */
	       .vid = 0x16b81397,
	       .sub = 0x16b81397);

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(COUNT())
{
  I4B_DBASE_IMPORT(hfc8s_dbase_root);
  I4B_DBASE_ADD(double_clock, 1);
}

I4B_PCI_DRIVER(/* HFC-8S */
	       .vid = 0x16b81397,
	       .sub = 0xb5211397);

I4B_PCI_DRIVER(/* HFC-8S */
	       .vid = 0x16b81397,
	       .sub = 0xb5221397);

I4B_PCI_DRIVER(/* HFC-8S */
	       .vid = 0x16b81397,
	       .sub = 0xb5521397);

I4B_PCI_DRIVER(/* HFC-8S */
	       .vid = 0x16b81397,
	       .sub = 0xb6221397);

I4B_PCI_DRIVER(/* HFC-8S Beronet card */
	       .vid = 0x16b81397,
	       .sub = 0xb5621397);

/* cleanup */

#undef hfc4s8s_fifo_inc_fx_pre
#undef hfc4s8s_fifo_frame_check
#undef hfc4s8s_fsm_table

#undef REG_hfc4s8s_a_ch_msk_write
#undef REG_hfc4s8s_a_channel_write
#undef REG_hfc4s8s_a_con_hdlc_write
#undef REG_hfc4s8s_a_conf_write
#undef REG_hfc4s8s_a_fifo_data_noinc_write
#undef REG_hfc4s8s_a_fifo_seq_write
#undef REG_hfc4s8s_a_inc_res_fifo_write
#undef REG_hfc4s8s_a_irq_msk_write
#undef REG_hfc4s8s_a_sl_cfg_write
#undef REG_hfc4s8s_a_st_b1_tx_write
#undef REG_hfc4s8s_a_st_b2_tx_write
#undef REG_hfc4s8s_a_st_clk_dly_write
#undef REG_hfc4s8s_a_st_ctrl0_write
#undef REG_hfc4s8s_a_st_ctrl1_write
#undef REG_hfc4s8s_a_st_ctrl2_write
#undef REG_hfc4s8s_a_st_d_tx_write
#undef REG_hfc4s8s_a_st_sq_write
#undef REG_hfc4s8s_a_st_sta_write
#undef REG_hfc4s8s_a_subch_cfg_write

#undef REG_hfc4s8s_r_bert_wd_md_write
#undef REG_hfc4s8s_r_brg_pcm_cfg_write
#undef REG_hfc4s8s_r_cirm_write
#undef REG_hfc4s8s_r_conf_en_write
#undef REG_hfc4s8s_r_ctrl_write
#undef REG_hfc4s8s_r_dtmf_n_write
#undef REG_hfc4s8s_r_dtmf_write
#undef REG_hfc4s8s_r_fifo_md_write
#undef REG_hfc4s8s_r_fifo_write
#undef REG_hfc4s8s_r_first_fifo_write
#undef REG_hfc4s8s_r_fsm_idx_write
#undef REG_hfc4s8s_r_gpio_en0_write
#undef REG_hfc4s8s_r_gpio_en1_write
#undef REG_hfc4s8s_r_gpio_out0_write
#undef REG_hfc4s8s_r_gpio_out1_write
#undef REG_hfc4s8s_r_gpio_sel_write
#undef REG_hfc4s8s_r_irq_ctrl_write
#undef REG_hfc4s8s_r_irqmsk_misc_write
#undef REG_hfc4s8s_r_pcm_md0_write
#undef REG_hfc4s8s_r_pcm_md1_write
#undef REG_hfc4s8s_r_pcm_md2_write
#undef REG_hfc4s8s_r_pwm0_write
#undef REG_hfc4s8s_r_pwm1_write
#undef REG_hfc4s8s_r_pwm_md_write
#undef REG_hfc4s8s_r_ram_addr0_write
#undef REG_hfc4s8s_r_ram_addr1_write
#undef REG_hfc4s8s_r_ram_addr2_write
#undef REG_hfc4s8s_r_ram_misc_write
#undef REG_hfc4s8s_r_sci_msk_write
#undef REG_hfc4s8s_r_sh0h_write
#undef REG_hfc4s8s_r_sh0l_write
#undef REG_hfc4s8s_r_sh1h_write
#undef REG_hfc4s8s_r_sh1l_write
#undef REG_hfc4s8s_r_sl_sel0_write
#undef REG_hfc4s8s_r_sl_sel1_write
#undef REG_hfc4s8s_r_sl_sel2_write
#undef REG_hfc4s8s_r_sl_sel3_write
#undef REG_hfc4s8s_r_sl_sel4_write
#undef REG_hfc4s8s_r_sl_sel5_write
#undef REG_hfc4s8s_r_sl_sel6_write
#undef REG_hfc4s8s_r_sl_sel7_write
#undef REG_hfc4s8s_r_slot_write
#undef REG_hfc4s8s_r_st_sel_write
#undef REG_hfc4s8s_r_st_sync_write
#undef REG_hfc4s8s_r_ti_wd_write

#undef REG_hfc4s8s_a_f12_read
#undef REG_hfc4s8s_a_f1_read
#undef REG_hfc4s8s_a_f2_read
#undef REG_hfc4s8s_a_st_b1_rx_read
#undef REG_hfc4s8s_a_st_b2_rx_read
#undef REG_hfc4s8s_a_st_d_rx_read
#undef REG_hfc4s8s_a_st_e_rx_read
#undef REG_hfc4s8s_a_st_sta_read
#undef REG_hfc4s8s_a_st_sq_read
#undef REG_hfc4s8s_a_z12_read
#undef REG_hfc4s8s_a_z1_read
#undef REG_hfc4s8s_a_z1h_read
#undef REG_hfc4s8s_a_z1l_read
#undef REG_hfc4s8s_a_z2_read
#undef REG_hfc4s8s_a_z2h_read
#undef REG_hfc4s8s_a_z2l_read

#undef REG_hfc4s8s_r_bert_ech_read
#undef REG_hfc4s8s_r_bert_ecl_read
#undef REG_hfc4s8s_r_bert_sta_read
#undef REG_hfc4s8s_r_chip_id_read
#undef REG_hfc4s8s_r_chip_rv_read
#undef REG_hfc4s8s_r_conf_oflow_read
#undef REG_hfc4s8s_r_f0_cnth_read
#undef REG_hfc4s8s_r_f0_cntl_read
#undef REG_hfc4s8s_r_gpi_in0_read
#undef REG_hfc4s8s_r_gpi_in1_read
#undef REG_hfc4s8s_r_gpi_in2_read
#undef REG_hfc4s8s_r_gpi_in3_read
#undef REG_hfc4s8s_r_gpio_in0_read
#undef REG_hfc4s8s_r_gpio_in1_read
#undef REG_hfc4s8s_r_int_data_read
#undef REG_hfc4s8s_r_irq_fifo_bl0_read
#undef REG_hfc4s8s_r_irq_fifo_bl1_read
#undef REG_hfc4s8s_r_irq_fifo_bl2_read
#undef REG_hfc4s8s_r_irq_fifo_bl3_read
#undef REG_hfc4s8s_r_irq_fifo_bl4_read
#undef REG_hfc4s8s_r_irq_fifo_bl5_read
#undef REG_hfc4s8s_r_irq_fifo_bl6_read
#undef REG_hfc4s8s_r_irq_fifo_bl7_read
#undef REG_hfc4s8s_r_irq_misc_read
#undef REG_hfc4s8s_r_irq_oview_read
#undef REG_hfc4s8s_r_ram_use_read
#undef REG_hfc4s8s_r_sci_read
#undef REG_hfc4s8s_r_status_read

#undef REG_hfc4s8s_a_fifo_data
#undef REG_hfc4s8s_r_ram_data

#undef HFC4S8S_BUS_VAR
#undef HFC4S8S_READ_1
#undef HFC4S8S_WRITE_1

#endif /* _I4B_HFC4S8S_H_ */
