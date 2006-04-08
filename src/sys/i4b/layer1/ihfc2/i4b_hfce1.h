/*-
 * Copyright (c) 1998-2003 Cologne Chip AG. All rights reserved.
 *
 * Copyright (c) 1998-2003 Moving Bytes Communications, 
 *                         Systementwicklung GmbH. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
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
 *	i4b_hfce1.h - HFC-E1 PCI driver module, Primary Rate
 * 	----------------------------------------------------
 *
 * $FreeBSD: $
 *
 * The chip specification is available from: http://www.colognechip.com
 *---------------------------------------------------------------------------*/
#ifndef _I4B_HFCE1_H_
#define _I4B_HFCE1_H_

#include <i4b/layer1/ihfc2/i4b_hfc.h>

/* imports */

#define hfce1_fifo_inc_fx_pre     default_fifo_inc_fx_pre
#define hfce1_fifo_frame_check    hfcs_fifo_frame_check

/* HFC-E1 registers used by this driver,
 * sorted alphabetically:
 */

#define REG_hfce1_cirm              0x00
#define REG_hfce1_con_hdlc          0xfa
#define REG_hfce1_conf              0xd1
#define REG_hfce1_crc_ech           0x35
#define REG_hfce1_crc_ecl           0x34
#define REG_hfce1_ctrl              0x01
#define REG_hfce1_e_ech             0x37
#define REG_hfce1_e_ecl             0x36
#define REG_hfce1_fas_ech           0x31
#define REG_hfce1_fas_ecl           0x30
#define REG_hfce1_fifo_data         0x80
#define REG_hfce1_fifo_data_noinc   0x84
#define REG_hfce1_fifo_md           0x0d
#define REG_hfce1_fifo_sel          0x0f
#define REG_hfce1_inc_res_fifo      0x0e
#define REG_hfce1_irq_ctrl          0x13
#define REG_hfce1_irq_misc          0x11
#define REG_hfce1_jatt_cfg          0x2f
#define REG_hfce1_jatt_sta          0x2b
#define REG_hfce1_los0              0x22
#define REG_hfce1_los1              0x23
#define REG_hfce1_pcm_md0           0x14
#define REG_hfce1_pcm_md1           0x15
#define REG_hfce1_pcm_md2           0x15
#define REG_hfce1_pwm0              0x38
#define REG_hfce1_pwm1              0x39
#define REG_hfce1_pwm_md            0x46
#define REG_hfce1_ram_addr0         0x08
#define REG_hfce1_ram_addr1         0x09
#define REG_hfce1_ram_addr2         0x0a
#define REG_hfce1_ram_data          0xc0
#define REG_hfce1_ram_misc          0x0c
#define REG_hfce1_rx0               0x24
#define REG_hfce1_rx_offs           0x30
#define REG_hfce1_rx_sl0_cfg0       0x25
#define REG_hfce1_rx_sl0_cfg1       0x26
#define REG_hfce1_sa6_val13_ech     0x39
#define REG_hfce1_sa6_val13_ecl     0x38
#define REG_hfce1_sa6_val23_ech     0x3b
#define REG_hfce1_sa6_val23_ecl     0x3a
#define REG_hfce1_sl0_cfg1          0x2e
#define REG_hfce1_sl_cfg            0xd0
#define REG_hfce1_slip              0x2c
#define REG_hfce1_slot              0x10
#define REG_hfce1_states            0x20
#define REG_hfce1_status            0x1c
#define REG_hfce1_sync_ctrl         0x35
#define REG_hfce1_sync_out          0x31
#define REG_hfce1_ti_wd             0x1a
#define REG_hfce1_tx0               0x28
#define REG_hfce1_tx1               0x29
#define REG_hfce1_tx_offs           0x34
#define REG_hfce1_tx_sl0            0x2d
#define REG_hfce1_tx_sl0_cfg0       0x2c
#define REG_hfce1_vio_ech           0x33
#define REG_hfce1_vio_ecl           0x32

/*
 * NOTE: when the D-channel and B-channels are not in use, 
 * output bits must be forced to "1" by writing to SCTRL and
 * SCTRL_E registers, and not just by disabling the FIFO !
 */

#define HFCE1_BUS_VAR(sc)					\
	bus_space_tag_t    t = sc->sc_resources.mem_tag[0];	\
	bus_space_handle_t h = sc->sc_resources.mem_hdl[0];

#define HFCE1_READ_1(reg,var)			\
{						\
	(var) = bus_space_read_1(t,h,(reg));	\
}

#define HFCE1_WRITE_1(reg,var)			\
{						\
	bus_space_write_1(t,h,(reg),(var));	\
}

static u_int8_t
hfce1_stable_read_1(ihfc_sc_t *sc, bus_size_t offset)
{
	HFCE1_BUS_VAR(sc);
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
hfce1_fifo_wait_disbusy(ihfc_sc_t *sc)
{
	HFCE1_BUS_VAR(sc);
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
	    HFCE1_READ_1(REG_hfce1_status, temp);
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
hfce1_chip_slots_init(ihfc_sc_t *sc)
{
	/* not used */
	return;
}

static void
hfce1_chip_reset CHIP_RESET_T(sc,error)
{
	HFCE1_BUS_VAR(sc);
	u_int16_t temp;
	u_int16_t Z_min; /* inclusive */
	u_int16_t Z_max; /* exclusive */
	u_int8_t F_min; /* inclusive */
	u_int8_t F_max; /* exclusive */
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

	HFCE1_WRITE_1(REG_hfce1_ctrl, temp);

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

	HFCE1_WRITE_1(REG_hfce1_ram_misc, temp);

	/* initialize FIFO maps */

	temp = 0x00;

	FIFO_FOREACH(f,sc)
	{
	    f->fm.h.Zsize = Z_max - Z_min;
	    f->fm.h.Fsize = F_max - F_min;
	    f->fm.h.Zend  = Z_max;

	    /* need to reorder the FIFO's so that
	     * the D-channel is first and 
	     * the B-channels last
	     */
	    if(sc->sc_channel_mapping == 0)
	    {
	      /* default */

	      if(FIFO_NO(f) == 0x00)
		  temp = 0x20;

	      if(FIFO_NO(f) == 0x02)
		  temp = 0x00;

	      if(FIFO_NO(f) == 0x20)
		  temp = 0x02;

	      if(FIFO_NO(f) == 0x3E)
		  temp = -0x3E;

	      f->s_fifo_sel = FIFO_NO(f) + temp;
	    }
	    else
	    {
	      int32_t channel_id = FIFO_NO(f) / 2;

	      L1_COMMAND_REQ(sc->sc_resources.i4b_controller, CMR_ENCODE_CHANNEL, &channel_id);

	      f->s_fifo_sel = ((channel_id * 2) | FIFO_DIR(f)) & 0x3F;
	    }
	}

	/* reset chip */

	HFCE1_WRITE_1(REG_hfce1_cirm, 0x08);
	HFCE1_WRITE_1(REG_hfce1_cirm, 0x00);

	DELAY(1000); /* wait 1 ms */

	/* read status */
	HFCE1_READ_1(REG_hfce1_status, temp);
	if(temp & 0x01)
	{
	    IHFC_ADD_ERR(error, "(BUSY != 0)");
	}

	HFCE1_WRITE_1(REG_hfce1_fifo_md, 0x00);

	/* initialize internal data */

	if(sc->sc_default.o_EXTERNAL_RAM)
	  temp = 0x2800;
	else
	  temp = 0x1800;

	do {
	    HFCE1_WRITE_1(REG_hfce1_ram_addr0, (temp & 0xFF));
	    HFCE1_WRITE_1(REG_hfce1_ram_addr1, (temp >> 8));
	    HFCE1_WRITE_1(REG_hfce1_ram_addr2, (0x00));
	    HFCE1_WRITE_1(REG_hfce1_ram_data, 0xFF);

	    temp++;
	} while(temp & 0xFF);

	/* 1. PCM interface */

	/* disable all time slots */

	for(temp = 0; temp < 256; temp++)
	{
	    HFCE1_WRITE_1(REG_hfce1_slot, temp);
	    HFCE1_WRITE_1(REG_hfce1_sl_cfg, 0x00);
	    HFCE1_WRITE_1(REG_hfce1_conf, 0x00);
	}

	/* PCM slave or master */
	HFCE1_WRITE_1(REG_hfce1_pcm_md0, 
		      IS_PCM_SLAVE(sc,0) ? 0x90 : 0x91);

	/* set PCM speed */
	HFCE1_WRITE_1(REG_hfce1_pcm_md1, 
		      IS_PCM_SPEED_128(sc,0) ? 0x20 :
		      IS_PCM_SPEED_64(sc,0) ? 0x10 : 0x00);

	/* PCM is synchronized to E1 receive */
	HFCE1_WRITE_1(REG_hfce1_pcm_md2, 0x00);

	/* wait until PCM reset sequence is finished */
	DELAY(4*125);

	hfce1_chip_slots_init(sc);

	if(IS_NT_MODE(sc,0))
	{
	    /* NT-mode */

	    /* 2. E1 transmitter */

	    /* HDB3 code, output driver enabled */

	    HFCE1_WRITE_1(REG_hfce1_tx0, 0x81);

	    /* transmitter tandem mode, tristate gap disabled */

	    HFCE1_WRITE_1(REG_hfce1_tx1, 0x60);

	    /* E1 timeslot 0 Sa4-Sa8 bits are 
	     * taken from TX_FR1 register 
	     */

	    HFCE1_WRITE_1(REG_hfce1_tx_sl0_cfg0, 0x0);

	    /* Sa4-Sa8 bits set to 1 */

	    HFCE1_WRITE_1(REG_hfce1_tx_sl0, 0xf8);

	    /* E1 time slot 0 automatically generated by HFC-E1, 
	     * CRC4, inverted CRC4 error indication bits are 
	     * transmitted
	     */
	    HFCE1_WRITE_1(REG_hfce1_sl0_cfg1, 0x31);



	    /* 3. E1 receiver */

	    /* receive HDB3 code */

	    HFCE1_WRITE_1(REG_hfce1_rx0, 0x01);

	    /* loss of signal is detected after 256 bits (1 frame) */

	    HFCE1_WRITE_1(REG_hfce1_los0,  0x0f);

	    /* LOS alarm is cleared if 16 transitions 
	     * are detected within the last 256 bits
	     */
	    HFCE1_WRITE_1(REG_hfce1_los1, 0x0f);

	    /* automatic error recovery, 
	     * automatic re-synchronization
	     */
	    HFCE1_WRITE_1(REG_hfce1_rx_sl0_cfg0, 0x06);

	    /* multiframe mode enabled, 
	     * multiframe error leads to loss of synchronization
	     */
	    HFCE1_WRITE_1(REG_hfce1_rx_sl0_cfg1, 0x03);



	    /* 4. Synchronization mode settings */


	    /* clock synchronization is determined from PCM pin F0IO
	     * TX and RX E1 frame phase offset 0
	     * automatic frequency search after  3 mismatches
	     * transmit jitter attenuator enabled
	     */

	    HFCE1_WRITE_1(REG_hfce1_sync_ctrl, 0x05);

	    /* undocumented internal register */

	    HFCE1_WRITE_1(REG_hfce1_jatt_cfg, 0x9c);

	    /* IPATS tester compliance bits are set */

	    HFCE1_WRITE_1(REG_hfce1_sync_out, 0xe0);



	    /* 5. E1 pulse amplitude and receive sensitivity */

	    /* enable PWM0, push to 0 only */

	    HFCE1_WRITE_1(REG_hfce1_pwm_md, 0x20);

	    /* amplitude setting, can be adjusted */

	    HFCE1_WRITE_1(REG_hfce1_pwm0, 0x50);

	    /* PWM1, this setting is ignored because PWM1 is disabled */

	    HFCE1_WRITE_1(REG_hfce1_pwm1, 0xff);

 

	    /* 6. line interface activation procedure */

	    /* set E1 layer 1 G1 state (operational) */
	    HFCE1_WRITE_1(REG_hfce1_states, 0x11);
	}
	else
	{
	    /* TE-mode (default) */

	    /* 2. E1 transmitter */

	    /* HDB3 code, output driver enabled */

	    HFCE1_WRITE_1(REG_hfce1_tx0, 0x81);

	    /* transmitter tandem mode, tristate gap disabled */

	    HFCE1_WRITE_1(REG_hfce1_tx1, 0x60);

	    /* E1 timeslot 0 Sa4-Sa8 bits are taken from TX_FR1 register */

	    HFCE1_WRITE_1(REG_hfce1_tx_sl0_cfg0, 0x00);

	    /* Sa4-Sa8 bits set to 1 */

	    HFCE1_WRITE_1(REG_hfce1_tx_sl0, 0xf8);

	    /* E1 time slot 0 automatically generated by HFC-E1, 
	     * CRC4, inverted CRC4 error indication bits are transmitted
	     */

	    HFCE1_WRITE_1(REG_hfce1_sl0_cfg1, 0x31);



	    /* 3. E1 receiver */

	    /* receive HDB3 code */

	    HFCE1_WRITE_1(REG_hfce1_rx0, 0x01);

	    /* loss of signal is detected after 256 bits (1 frame) */

	    HFCE1_WRITE_1(REG_hfce1_los0, 0x0f);

	    /* LOS alarm is cleared if 16 transitions are detected 
	     * within the last 256 bits
	     */

	    HFCE1_WRITE_1(REG_hfce1_los1, 0x0f);

	    /* automatic error recovery, 
	     * automatic re-synchronization 
	     */

	    HFCE1_WRITE_1(REG_hfce1_rx_sl0_cfg0, 0x06);

	    /* multiframe mode enabled, 
	     * multiframe error leads to loss of synchronization
	     */

	    HFCE1_WRITE_1(REG_hfce1_rx_sl0_cfg1, 0x03);



	    /* 4. Synchronization mode settings */

	    /* clock synchronization derived from E1 receive
	     * TX and RX E1 frame phase offset arbitrary
	     * automatic frequency search after  3 mismatches
	     * transmit jitter attenuator enabled
	     */

	    HFCE1_WRITE_1(REG_hfce1_sync_ctrl, 0x02);

	    /* undocumented internal register */

	    HFCE1_WRITE_1(REG_hfce1_jatt_cfg, 0x9c);

	    /* IPATS tester compliance bits are set */

	    HFCE1_WRITE_1(REG_hfce1_sync_out, 0xe0);



	    /* 5. E1 pulse amplitude and receive sensitivity */

	    /* PWM register settings */

	    /* enable PWM0, push to 0 only */

	    HFCE1_WRITE_1(REG_hfce1_pwm_md, 0x20);

	    /* amplitude setting, can be adjusted  */

	    HFCE1_WRITE_1(REG_hfce1_pwm0, 0x50);

	    /* PWM1, this setting is ignored because PWM1 is disabled */

	    HFCE1_WRITE_1(REG_hfce1_pwm1, 0xff);



	    /* 6. line interface activation procedure */

	    /* release E1 layer 1 state machine */

	    HFCE1_WRITE_1(REG_hfce1_states, 0x00);
	}

	/* initialize RX and TX elastic data buffers */
	HFCE1_WRITE_1(REG_hfce1_rx_offs, 0x06);
	HFCE1_WRITE_1(REG_hfce1_tx_offs, 0x06);

	/* make sure that the elastic buffers get set */

	sc->sc_default.o_PRIVATE_FLAG_0 = 1;

	/* enable disbusy timeout warning */

	sc->sc_default.o_PRIVATE_FLAG_1 = 1;

	/* enable timer: 32 ms */

	HFCE1_WRITE_1(REG_hfce1_ti_wd, 0x07);

	/* enable global interrupt
	 * - low is active
	 */
	HFCE1_WRITE_1(REG_hfce1_irq_ctrl, 
		      IS_POLLED_MODE(sc,0) ? 0x00 : 0x08);
	return;
}

static void
hfce1_fifo_select FIFO_SELECT_T(sc,f)
{
	HFCE1_BUS_VAR(sc);

	HFCE1_WRITE_1(REG_hfce1_fifo_sel, f->s_fifo_sel);

	hfce1_fifo_wait_disbusy(sc);

	sc->sc_fifo_select_last = f;

	return;
}

static void
hfce1_chip_config_write CHIP_CONFIG_WRITE_T(sc,f)
{
	HFCE1_BUS_VAR(sc);

	u_int8_t temp;
	u_int8_t cable;

	if((f == CONFIG_WRITE_UPDATE) ||
	   (f == CONFIG_WRITE_RELOAD))
	{
	    /* enable interrupts:
	     * - statemachine change
	     * - timer elapsed
	     */
	    temp = 0x01;

	    if(ihfc_fifos_active(sc))
	    {
	        /* enable timer */
	        temp |= 0x02;
	    }

	    /*
	     * enabling interrupts can generate
	     * a new interrupt immediately !
	     */
	    HFCE1_WRITE_1(REG_hfce1_irq_misc, temp);
	}
	else
	{
	    hfce1_fifo_select(sc,f);

	    if(f->prot_last.protocol_1 == P_BRIDGE)
	    {
	        temp = (FIFO_DIR(f) == transmit) ?
		  ((f->prot_last.u.bridge.tx_slot << 1)|transmit) :
		  ((f->prot_last.u.bridge.rx_slot << 1)|receive);

		/* disable time slot */
		HFCE1_WRITE_1(REG_hfce1_slot, temp);
		HFCE1_WRITE_1(REG_hfce1_sl_cfg, 0x00);
	    }

	    if((f->prot_curr.protocol_1 == P_DISABLE) &&
	       (FIFO_DIR(f) == receive))
	    {
	        /* disable FIFO */
	        HFCE1_WRITE_1(REG_hfce1_con_hdlc, 0x00);
		return;
	    }

	    if(FIFO_NO(f) == d1t)
	        temp = 1; /* 0xFF inter frame fill */
	    else 
	        temp = 0; /* 0x7E inter frame fill */

	    if(PROT_IS_TRANSPARENT(&(f->prot_curr)) ||
	       (f->prot_curr.protocol_1 == P_BRIDGE) ||
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

	    HFCE1_WRITE_1(REG_hfce1_con_hdlc, temp);

	    /* reset FIFO and LOST bit */

	    HFCE1_WRITE_1(REG_hfce1_inc_res_fifo, 0x06);

	    hfce1_fifo_wait_disbusy(sc);

	    if(FIFO_DIR(f) == transmit)
	    {
	        /* define the bytes that are sent
		 * to the bus:
		 */
	        HFCE1_WRITE_1(REG_hfce1_fifo_data_noinc, 0xff);
	    }

	    if(f->prot_curr.protocol_1 == P_BRIDGE)
	    {
	        temp = (FIFO_DIR(f) == transmit) ?
		  ((f->prot_curr.u.bridge.tx_slot << 1)|transmit) :
		  ((f->prot_curr.u.bridge.rx_slot << 1)|receive);

		cable = (FIFO_DIR(f) == transmit) ? 
		  f->prot_curr.u.bridge.tx_cable :
		  f->prot_curr.u.bridge.rx_cable;

		cable = ((cable << 6) | 0x80 | f->s_fifo_sel);

		/* enable time slot */
		HFCE1_WRITE_1(REG_hfce1_slot, temp);
		HFCE1_WRITE_1(REG_hfce1_sl_cfg, cable);
	    }
	}
	return;
}

static void
hfce1_fifo_read FIFO_READ_T(sc,f,ptr,len)
{
	HFCE1_BUS_VAR(sc);
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
	    bus_space_read_multi_1(t,h,REG_hfce1_fifo_data,ptr,temp);
	    ptr += temp;
	}

	len /= 4;

	if(len)
	{
	    bus_space_read_multi_4(t,h,REG_hfce1_fifo_data,(u_int32_t *)ptr,len);
	}
	return;
}

static void
hfce1_chip_read CHIP_READ_T(sc,reg,ptr,len)
{
	/* not used */
	return;
}

static void
hfce1_fifo_write FIFO_WRITE_T(sc,f,ptr,len)
{
	HFCE1_BUS_VAR(sc);
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
	        bus_space_write_multi_1(t,h,REG_hfce1_fifo_data,ptr,temp);
		ptr += temp;
	    }

	    len /= 4;

	    if(len)
	    {
	        bus_space_write_multi_4(t,h,REG_hfce1_fifo_data,(const u_int32_t *)ptr,len);
	    }
	}
	return;
}

static void
hfce1_fifo_write_filler FIFO_WRITE_FILLER_T(sc,f)
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
		hfce1_fifo_write(sc,f,&fill[0],len);
		len = min(f->Z_chip,sizeof(fill));
	    } while(len);
	}
	return;
}

static void
hfce1_chip_write CHIP_WRITE_T(sc,reg,ptr,len)
{
	/* not used */
	return;
}

static void
hfce1_fsm_read FSM_READ_T(sc,f,ptr)
{
	HFCE1_BUS_VAR(sc);
	u_int8_t temp;

	/* read STATES */
	HFCE1_READ_1(REG_hfce1_states, temp);
	*ptr = ((temp & 7) +
		(IS_NT_MODE(sc,0) ?
		 HFC_NT_OFFSET : HFC_TE_OFFSET)) & 0xf;
	return;
}

static void
hfce1_fsm_write FSM_WRITE_T(sc,f,ptr)
{
#if 0
	HFCE1_BUS_VAR(sc);

	if(*ptr == 0x60)
	  old fashioned activation;
	else if(*ptr == 0x40)
	  old fashioned deactivation;
	else
	  just release state machine

	/* write STATES */
	HFCE1_WRITE_1(REG_hfce1_states, *ptr);
#endif
	return;
}

static void
hfce1_fifo_inc_fx FIFO_INC_FX_T(sc,f)
{
	HFCE1_BUS_VAR(sc);

	/* increment F-counter */

	f->F_drvr ++;
	if(f->F_drvr >= f->fm.h.Fsize)
	{
	    f->F_drvr = 0;
	}

	HFCE1_WRITE_1(REG_hfce1_inc_res_fifo, 0x04);

	hfce1_fifo_wait_disbusy(sc);

	HFCE1_WRITE_1(REG_hfce1_inc_res_fifo, 0x01);

	hfce1_fifo_wait_disbusy(sc);

#if 1
	/* NOTE: reload counter values
	 * in case cache is invalid:
	 */
	f->state &= ~ST_FZ_LOADED;
#endif
	return;
}

static void
hfce1_fifo_fz_read FIFO_FZ_READ_T(sc,f)
{
	HFCE1_BUS_VAR(sc);
	u_int8_t  Z_base;
	u_int8_t  F_base;
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

	/* read F- and Z-counters respectively */

	/* fx */
	f->F_chip = hfce1_stable_read_1(sc,F_base);

	/* zx - high */
	Z_chip2 = hfce1_stable_read_1(sc,Z_base ^ 1) << 8;

	/* zx - low */
	f->Z_chip = hfce1_stable_read_1(sc,Z_base);

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
	f->Z_chip|= hfce1_stable_read_1(sc,Z_base ^ 1) << 8;

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
hfce1_chip_status_read CHIP_STATUS_READ_T(sc)
{
	HFCE1_BUS_VAR(sc);
	register u_int8_t temp;

	/* read status */
	HFCE1_READ_1(REG_hfce1_irq_misc, temp);

	/* statemachine changed */
	if(temp & 1)
	{
	    ihfc_fsm_update(sc, &sc->sc_fifo[0], 0);
	}

	/* timer elapsed */
	if(temp & 2)
	{
#if ((IHFC_CHANNELS < (31*2)) || (SC_INTR_BITS != 8))
#error "please update this code, (IHFC_CHANNELS < (31*2)) || (SC_INTR_BITS != 8)"
#endif
	    sc->sc_intr_status[0] = 0xFF;
	    sc->sc_intr_status[1] = 0xFF;
	    sc->sc_intr_status[2] = 0xFF;
	    sc->sc_intr_status[3] = 0xFF;
	    sc->sc_intr_status[4] = 0xFF;
	    sc->sc_intr_status[5] = 0xFF;
	    sc->sc_intr_status[6] = 0xFF;
	    sc->sc_intr_status[7] = 0x3F;
	}

	if(sc->sc_default.o_PRIVATE_FLAG_0)
	{
	    HFCE1_READ_1(REG_hfce1_jatt_sta, temp);

	    /* check if the E1 transmit clock 
	     * is fully synchronized
	     *
	     * (this can take up to one minute)
	     */
	    if((temp & 0x60) == 0x60)
	    {
	        sc->sc_default.o_PRIVATE_FLAG_0 = 0;

		/* initialize RX and TX elastic data buffers 
		 * after that E1 transmit clock is synchronized
		 */
		HFCE1_WRITE_1(REG_hfce1_rx_offs, 0x06);
		HFCE1_WRITE_1(REG_hfce1_tx_offs, 0x06);

		IHFC_MSG("E1 transmit clock fully synchronized!\n");
	    }
	}
	return;
}

static void
hfce1_chip_status_check CHIP_STATUS_CHECK_T(sc)
{
	/* nothing to do */
	return;
}

static void
hfce1_chip_unselect CHIP_UNSELECT_T(sc)
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
	        hfce1_fifo_select(sc, sc->sc_fifo_select_last);
	    }
	    sc->sc_fifo_select_last = NULL;
	}
	return;
}

static ihfc_fifo_program_t *
hfce1_fifo_get_program FIFO_GET_PROGRAM_T(sc,f)
{
	ihfc_fifo_program_t *program = NULL;

	/* NOTE: the D-channel should only be used
	 * in HDLC-mode, though transparent
	 * D-channel mode is supported for
	 * debugging purposes:
	 */
	if(PROT_IS_HDLC(&(f->prot_curr)) ||
	   PROT_IS_TRANSPARENT(&(f->prot_curr)))
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
hfce1_register_list [] =
{
	{ 0, 0 }
};

#if (IHFC_CHANNELS < 62)
#error "HFC-E1 needs 62 channels, RX + TX"
#endif

#include <i4b/layer1/ihfc2/i4b_count.h>

I4B_DBASE(hfce1_dbase_root)
{
  I4B_DBASE_ADD(c_chip_read          , &hfce1_chip_read);
  I4B_DBASE_ADD(c_chip_write         , &hfce1_chip_write);
  I4B_DBASE_ADD(c_chip_reset         , &hfce1_chip_reset);
  I4B_DBASE_ADD(c_chip_unselect      , &hfce1_chip_unselect);
  I4B_DBASE_ADD(c_chip_status_read   , &hfce1_chip_status_read);
  I4B_DBASE_ADD(c_chip_status_check  , &hfce1_chip_status_check);
  I4B_DBASE_ADD(c_chip_config_write  , &hfce1_chip_config_write);

  I4B_DBASE_ADD(c_fsm_read           , &hfce1_fsm_read);
  I4B_DBASE_ADD(c_fsm_write          , &hfce1_fsm_write);
  I4B_DBASE_ADD(d_fsm_table          , &hfce1_fsm_table);

  I4B_DBASE_ADD(c_fifo_get_program   , &hfce1_fifo_get_program);
  I4B_DBASE_ADD(c_fifo_read          , &hfce1_fifo_read);
  I4B_DBASE_ADD(c_fifo_write         , &hfce1_fifo_write);
  I4B_DBASE_ADD(c_fifo_write_filler  , &hfce1_fifo_write_filler);
  I4B_DBASE_ADD(c_fifo_select        , &hfce1_fifo_select);
  I4B_DBASE_ADD(c_fifo_inc_fx        , &hfce1_fifo_inc_fx);
  I4B_DBASE_ADD(c_fifo_fz_read       , &hfce1_fifo_fz_read);
  I4B_DBASE_ADD(c_fifo_inc_fx_pre    , &hfce1_fifo_inc_fx_pre);
  I4B_DBASE_ADD(c_fifo_frame_check   , &hfce1_fifo_frame_check);

  I4B_DBASE_ADD(d_register_list      , &hfce1_register_list[0]);
  I4B_DBASE_ADD(d_channels           , 62);
  I4B_DBASE_ADD(d_L1_type            , L1_TYPE_ISDN_PRI);

  /* delay 25 milliseconds */ 
  I4B_DBASE_ADD(d_interrupt_delay    , hz / 40);
 
  I4B_DBASE_ADD(desc                 , "HFC-E1 2MBit/s PCI ISDN adapter");

  I4B_DBASE_ADD(o_RES_IRQ_0          , 1); /* enable */
  I4B_DBASE_ADD(o_RES_MEMORY_0       , 1); /* enable */
  I4B_DBASE_ADD(o_TRANSPARENT_BYTE_REPETITION, 1); /* enable */

  I4B_DBASE_ADD(i4b_option_mask      , (I4B_OPTION_POLLED_MODE|
					I4B_OPTION_NT_MODE|
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

I4B_PCI_DRIVER(/* generic ID */
	       .vid = 0x30B11397);

/* cleanup */

#undef hfce1_fifo_inc_fx_pre
#undef hfce1_fifo_frame_check
#undef REG_hfce1_cirm
#undef REG_hfce1_con_hdlc
#undef REG_hfce1_crc_ech
#undef REG_hfce1_crc_ecl
#undef REG_hfce1_ctrl
#undef REG_hfce1_e_ech
#undef REG_hfce1_e_ecl
#undef REG_hfce1_fas_ech
#undef REG_hfce1_fas_ecl
#undef REG_hfce1_fifo_data
#undef REG_hfce1_fifo_data_noinc
#undef REG_hfce1_fifo_md
#undef REG_hfce1_fifo_sel
#undef REG_hfce1_inc_res_fifo
#undef REG_hfce1_irq_ctrl
#undef REG_hfce1_irq_misc
#undef REG_hfce1_jatt_cfg
#undef REG_hfce1_jatt_sta
#undef REG_hfce1_los0
#undef REG_hfce1_los1
#undef REG_hfce1_pcm_md0
#undef REG_hfce1_pcm_md1
#undef REG_hfce1_pcm_md2
#undef REG_hfce1_pwm0
#undef REG_hfce1_pwm1
#undef REG_hfce1_pwm_md
#undef REG_hfce1_ram_addr0
#undef REG_hfce1_ram_addr1
#undef REG_hfce1_ram_addr2
#undef REG_hfce1_ram_data
#undef REG_hfce1_ram_misc
#undef REG_hfce1_rx0
#undef REG_hfce1_rx_offs
#undef REG_hfce1_rx_sl0_cfg0
#undef REG_hfce1_rx_sl0_cfg1
#undef REG_hfce1_sa6_val13_ech
#undef REG_hfce1_sa6_val13_ecl
#undef REG_hfce1_sa6_val23_ech
#undef REG_hfce1_sa6_val23_ecl
#undef REG_hfce1_sl0_cfg1
#undef REG_hfce1_sl_cfg
#undef REG_hfce1_slip
#undef REG_hfce1_slot
#undef REG_hfce1_states
#undef REG_hfce1_status
#undef REG_hfce1_sync_ctrl
#undef REG_hfce1_sync_out
#undef REG_hfce1_ti_wd
#undef REG_hfce1_tx0
#undef REG_hfce1_tx1
#undef REG_hfce1_tx_offs
#undef REG_hfce1_tx_sl0
#undef REG_hfce1_tx_sl0_cfg0
#undef REG_hfce1_vio_ech
#undef REG_hfce1_vio_ecl
#undef HFCE1_BUS_VAR
#undef HFCE1_READ_1
#undef HFCE1_WRITE_1

#endif /* _I4B_HFCE1_H_ */
