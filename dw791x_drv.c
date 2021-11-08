/*
 * Dongwoon Anatech haptic driver for DW791X series
 * Copyright(c) 2018 - 2019 Dongwoon Anatech Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_gpio.h>
#include <linux/ktime.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/of_irq.h>

#include "dw791x_drv.h"
#include "dw791x_wave.h"

/* Debug mask value for haptic driver 
 * usage: echo [debug_mask] > /sys/module/dw791x_drv/parameters/dw791x_debug_mask
 */
int dw791x_debug_mask = 1;
module_param_named(dw791x_debug_mask, dw791x_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/***********************For define****************************/
/*for playing haptic wave*/
#define HAPTIC_WAVE
#define RTP_FW_FILE "haptic.bin"
#define FW_FILE "dwa_hpitic_mem_wave.bin"
#define RTP_TEST_FW_FILE "RTP_TEST_625ms.bin"

#define I2C_BUF_SIZE 1024

/************************For waveform***************************/
// ID List for Memory Playback
#ifdef DW791X_GAMMA
static dw791x_waveform_data gamma[] = {
	{WAVEFORM_ID_NONE, NULL, sizeof(NULL)},
	{WAVEFORM_ID_GMA_MEM1, gma_mem1, sizeof(gma_mem1)},
	{WAVEFORM_ID_GMA_MEM2, gma_mem2, sizeof(gma_mem2)},
	{WAVEFORM_ID_GMA_MEM3, gma_mem3, sizeof(gma_mem3)},
	{WAVEFORM_ID_GMA_MEM4, gma_mem4, sizeof(gma_mem4)},
	{WAVEFORM_ID_GMA_MEM5, gma_mem5, sizeof(gma_mem5)},
	{WAVEFORM_ID_GMA_MEM6, gma_mem6, sizeof(gma_mem6)},
	{WAVEFORM_ID_GMA_MEM7, gma_mem7, sizeof(gma_mem7)},
	{WAVEFORM_ID_SINE165, sine165, sizeof(sine165)},
	{WAVEFORM_ID_AUTO_BRAKE165, auto_brake_wave, sizeof(auto_brake_wave)}, //autobrake waveform for f0 = 165hz
	{WAVEFORM_ID_SINE145, sine185, sizeof(sine145)},
	{WAVEFORM_ID_SINE185, sine145, sizeof(sine185)},
};
#endif
// ID List for Memory Playback
#ifdef DW791X_COIN0832
static dw791x_waveform_data coin0832[] = {
	{WAVEFORM_ID_NONE, NULL, sizeof(NULL)},
	{WAVEFORM_ID_COIN0832_MEM1, coin0832_mem1, sizeof(coin0832_mem1)},
	{WAVEFORM_ID_COIN0832_MEM2, coin0832_mem2, sizeof(coin0832_mem2)},
	{WAVEFORM_ID_COIN0832_MEM3, coin0832_mem3, sizeof(coin0832_mem3)},
	{WAVEFORM_ID_COIN0832_MEM4, coin0832_mem4, sizeof(coin0832_mem4)},
	{WAVEFORM_ID_COIN0832_MEM5, coin0832_mem5, sizeof(coin0832_mem5)},
};
#endif
// ID List for Memory Playback
#ifdef DW791X_COIN1040
static dw791x_waveform_data coin1040[] = {
	{WAVEFORM_ID_NONE, NULL, sizeof(NULL)},
	{WAVEFORM_ID_COIN1040_MEM1, coin1040_mem1, sizeof(coin1040_mem1)},
	{WAVEFORM_ID_COIN1040_MEM2, coin1040_mem2, sizeof(coin1040_mem2)},
	{WAVEFORM_ID_COIN1040_MEM3, coin1040_mem3, sizeof(coin1040_mem3)},
	{WAVEFORM_ID_COIN1040_MEM4, coin1040_mem4, sizeof(coin1040_mem4)},
};
#endif

/**************************************************************/

#ifdef CONFIG_DW791X_A2V
/* =====================================================================================
function : dw791x_a2v_seq_write
descript : A2V data write function
====================================================================================== */

int dw791x_a2v_rtp_write(u8 *data, u32 size)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(g_dw791x->dev);

	if (dw791x->haptic_wave_on == 0)
	{
		dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_mode, RTP);
		dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
		dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.rtp_input, 0, RAM_ADDR8, (u8 *)data, size);
	}
	else
	{
		dw_debug("bypass for haptic wave play!\n");
	}

	return 0;
}

int dw791x_a2v_seq_write(int delta, int on_off, int freq_band)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(g_dw791x->dev);
	dw791x_byte_write(dw791x, DW7914_PWM, 0x05);
	if (dw791x->haptic_wave_on == 0)
	{
		dw791x_byte_write(dw791x, DW7914_VD_CLAMP, delta);
		if (on_off == 1)
		{
			if (freq_band == LOW_FREQUENCY)
			{
				dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x08); //  0x08
				dw791x_byte_write(dw791x, DW7914_WAV_SEQ_L0, 0x5);
				dw791x_byte_write(dw791x, DW7914_MEM_LOOP, 0x0);
				dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
				dw_debug("haptic wave play Low freq\n");
			}
			else if (freq_band == MIDDLE_FREQUENCY)
			{
				dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x0A);
				dw791x_byte_write(dw791x, DW7914_WAV_SEQ_L0, 0x5);
				dw791x_byte_write(dw791x, DW7914_MEM_LOOP, 0x0);
				dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
				dw_debug("haptic wave play MIDDLE freq\n");
			}
			else if (freq_band == HIGH_FREQUENCY)
			{
				dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x0B); //0x0B
				dw791x_byte_write(dw791x, DW7914_WAV_SEQ_L0, 0x5);
				dw791x_byte_write(dw791x, DW7914_MEM_LOOP, 0x0);
				dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
				dw_debug("haptic wave play HIGH freq\n");
			}
			else if (freq_band == MAX_FREQUENCY)
			{
				dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_STOP);
				dw_debug("haptic wave stop for unknown band\n");
			}
		}
		else
		{
			dw791x_byte_write(dw791x, DW7914_VD_CLAMP, 0x0);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_STOP);
			dw_debug("haptic wave stop\n");
		}
	}
	else
	{
		dw_debug("bypass for haptic wave play!\n");
	}
	return 0;
}
#endif // CONFIG_DW791X_A2V

/* =====================================================================================
function : dw791x_byte_write
====================================================================================== */
static int dw791x_byte_write(struct dw791x_priv *dw791x, u8 addr, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(dw791x->dwclient, addr, data);

	if (ret < 0)
	{
		dw_err("%s i2c byte write fail.!\n", dw791x->dev_name);
	}

	return ret;
}

/* =====================================================================================
function : dw791x_word_write
====================================================================================== */
static int dw791x_word_write(struct dw791x_priv *dw791x, u8 addr, u32 data)
{
	int ret;
	u8 xbuf[3];
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc = dw791x->dwclient;

	memset(xbuf, 0, sizeof(xbuf));

	xbuf[0] = (u8)addr;
	xbuf[1] = data >> 8;
	xbuf[2] = data;

	xfer[0].addr = CHIP_ID;
	xfer[0].len = 3;
	xfer[0].flags = 0;
	xfer[0].buf = xbuf;
	ret = i2c_transfer(i2c_fnc->adapter, xfer, 1);

	return ret;
}

/* =====================================================================================
function : dw791x_byte_read
====================================================================================== */
static int dw791x_byte_read(struct dw791x_priv *dw791x, u8 addr)
{
	int ret;

	ret = i2c_smbus_read_byte_data(dw791x->dwclient, addr);

	if (ret < 0)
	{
		dw_err("%s i2c byte read fail.!\n", dw791x->dev_name);
	}

	return ret;
}
#ifdef CONFIG_DW791X_A2V
int dw791x_byte_read_ex(u8 addr)
{
	int ret;

	ret = i2c_smbus_read_byte_data(g_dw791x->dwclient, addr);

	if (ret < 0)
	{
		dw_err("%s i2c byte read fail.!\n", g_dw791x->dev_name);
	}

	return ret;
}
#endif
/* =====================================================================================
function : dw791x_vd_clamp_set
descript : voltage level control function
====================================================================================== */
static void dw791x_vd_clamp_set(struct dw791x_priv *dw791x, u32 vol)
{
	u8 clamp;
	int ret = 0;

	clamp = vol / 40;

	if (dw791x->device == DW7912)
	{
		ret = dw791x_byte_write(dw791x, DW7912_VD_CLAMP, clamp);
	}

	else if (dw791x->device == DW7914)
	{
		ret = dw791x_byte_write(dw791x, DW7914_VD_CLAMP, clamp);
	}

	if (ret < 0)
		dw_err("%s i2c vd clamp set fail.!\n", dw791x->dev_name);
	else
		dw_debug("vd clamp: %dmV\n", vol);
}

/* =====================================================================================
function : dw791x_mem_vd_clamp_set
descript : voltage level control function for memory mode
====================================================================================== */
static void dw791x_mem_vd_clamp_set(struct dw791x_priv *dw791x, dw791x_waveform_id wave_id, u32 vol)
{
	u8 clamp;
	int ret = 0;
	u8 header_info[7] = {
		0,
	};
	uint16_t ram_addr = 0;
	int i;

	if (wave_id == WAVEFORM_ID_NONE)
	{
		dw_err("waveform id is zero\n");
		return;
	}

	if (wave_id < WAVEFORM_ID_MAX)
	{

		ram_addr = (wave_id - 1) * 5 + 1;
		header_info[0] = ram_addr >> 8 & 0xff;
		header_info[1] = ram_addr & 0xff;

		for (i = 0; i < 5; i++)
		{
			ret = dw791x_byte_write(dw791x, DW7914_RAM_ADDR_H, (ram_addr >> 8) & 0xff);
			ret = dw791x_byte_write(dw791x, DW7914_RAM_ADDR_L, ram_addr & 0xff);
			ram_addr++;

			header_info[i + 2] = dw791x_byte_read(dw791x, DW7914_RAM_DATA);
		}

		clamp = vol / 40;
		header_info[6] = clamp;

		if (dw791x->device == DW7914)
		{
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, 0, RAM_ADDR8, header_info, 7);
		}

		if (ret < 0)
			dw_err("%s i2c mem vd clamp set fail.!\n", dw791x->dev_name);
		else
		{
			dw_debug("mem vd clamp: %dmV\n", vol);
		}
	}
	else
	{
		dw_err("Invailed waveform id : %d\n", wave_id);
	}

	return;
}

/* =====================================================================================
function : dw791x_mem_gain_set
descript : dw791x device gain for only memory play mode
			mute_on is suppored only DW7914.
====================================================================================== */
static int dw791x_mem_gain_set(struct dw791x_priv *dw791x, u8 gain)
{
	int ret = 0;

	if (dw791x->device == DW7912)
	{
		ret = dw791x_byte_write(dw791x, DW7912_MEM_GAIN, gain);
	}
	else if (dw791x->device == DW7914)
	{
		ret = dw791x_byte_write(dw791x, DW7914_MEM_GAIN, gain);
	}

	if (ret < 0)
		dw_err("memory gain set fail.\n");

	return ret;
}
#ifdef DW791X_INT
/* =====================================================================================
function : dw791x_intz_en_set
descript : interrupt pin of dw791x device set.
		DW791X_INTEN[0] = SCP, DW791X_INTEN[1] = OCP, DW791X_INTEN[2] = TSD, DW791X_INTEN[3] = UVLO
		DW791X_INTEN[4] = PROCESS_DONE,
		
		DW7912_INTEN[5] = PROGRAM_ERR,      DW7914_INTEN[5] = FIFO_EMPTY
		DW7912_INTEN[6] = FIFO_FULL,        DW7914_INTEN[6] = FIFO_FULL
		                                    DW7914_INTEN[7] = TRIG_IN
====================================================================================== */
static int dw791x_intz_en_set(struct dw791x_priv *dw791x, u8 intz)
{
	int ret = 0;

	if (dw791x->device == DW7912)
	{
		ret = dw791x_byte_write(dw791x, DW7912_INTEN, intz);
	}
	else if (dw791x->device == DW7914)
	{
		ret = dw791x_byte_write(dw791x, DW7914_INTEN, intz);
	}

	if (ret < 0)
		dw_err("intz_en set fail.\n");
	else
		dw_err("intz_en set %x\n", intz);
	return ret;
}
#endif
/* =====================================================================================
function : dw791x_status_get
descript : dw791x device status get
			plz refer to specification.
====================================================================================== */
static int dw791x_status_get(struct dw791x_priv *dw791x)
{
	int ret = -1;

	if (dw791x->device == DW7912)
	{
		dw791x->dw791x_haptics.status0 = (u8)dw791x_byte_read(dw791x, DW791x_STATUS);
		ret = 0;
		dw_debug("status0 : %x\n", dw791x->dw791x_haptics.status0);
	}
	else if (dw791x->device == DW7914)
	{
		dw791x->dw791x_haptics.status0 = (u8)dw791x_byte_read(dw791x, DW7914_STATUS0);
		dw791x->dw791x_haptics.status1 = (u8)dw791x_byte_read(dw791x, DW7914_STATUS1);
		ret = 0;
		dw_debug("status0 : %x, status1 : %x\n", dw791x->dw791x_haptics.status0, dw791x->dw791x_haptics.status1);
	}

	if (ret < 0)
		dw_err("status get fail.\n");

	return ret;
}

/* =====================================================================================
function : dw791x_bst_option
descript : dw791x device boost control option
====================================================================================== */
#ifdef BST_OPT
static void dw791x_bst_option(struct dw791x_priv *dw791x, u32 lv, u32 limit, u32 ofs)
{
	u8 set = 0;
	int ret = 0;

	if (dw791x->device == DW7912)
	{
		set = limit << 5 | (ofs / 80);
		ret = dw791x_byte_write(dw791x, DW7912_BST_OPTION, set);
	}

	if (dw791x->device == DW7914)
	{
		set = lv << 6 | limit << 4 | (ofs / 80);
		ret = dw791x_byte_write(dw791x, DW7914_BST_OPTION, set);
	}

	if (ret < 0)
		dw_err("%s i2c boost option set fail.!\n", dw791x->dev_name);
	else
		dw_debug("%s boost offset: %dmV\n", dw791x->dev_name, set);
}
#endif

/* =====================================================================================
function : dw791x_fifo_size_set
descript : FIFO size set function
			value(2) = mean(2KB) / value(4) = mean(4KB) / value(6) = mean(6KB)
====================================================================================== */
static void dw791x_fifo_size_set(struct dw791x_priv *dw791x, u32 size)
{
	int ret = 0;
	u32 fifo_set;

	fifo_set = SRAM_BASE - (1024 * size + 4);
	dw_debug("fifo size: %dbyte\n", 1024 * size);
	ret = dw791x_word_write(dw791x, dw791x->dw791x_reg_addr.fifo_addrh, fifo_set);

	if (ret < 0)
		dw_err("%s i2c fifo size set fail.!\n", dw791x->dev_name);
}

/* =====================================================================================
function : dw791x_fifo_size_set
descript : FIFO size set function
			value(2) = mean(2KB) / value(4) = mean(4KB) / value(6) = mean(6KB)
====================================================================================== */
static int dw791x_fifo_level_get(struct dw791x_priv *dw791x)
{
	int ret = 0;
	u8 rx[2];
	int fifo_level = 0;

	rx[0] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_level);
	rx[1] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_level + 1);
	fifo_level = ((u32)rx[0] << 8) | rx[1];

	if (ret < 0)
		dw_err("%s i2c fifo_level get fail.!\n", dw791x->dev_name);

	dw_debug("fifo_level: %d\n", fifo_level);

	return fifo_level;
}

/* =====================================================================================
function : dw7914_mode_set
descript : only DW7914 devices are supported.
====================================================================================== */
static void dw7914_mode_set(struct dw791x_priv *dw791x, u8 type, u8 bitset)
{
	int ret, reg;

	reg = dw791x_byte_read(dw791x, DW7914_MODE);

	if (bitset == BITSET)
	{
		reg = reg | type;
	}
	else
	{
		reg = reg & ~type;
	}

	ret = dw791x_byte_write(dw791x, DW7914_MODE, reg);

	if (ret < 0)
		dw_err("%s i2c mode register set fail.!\n", dw791x->dev_name);
	else
		dw_debug("mode register set: %x\n", reg);
}

/* =====================================================================================
function : dw7924_trigctrl_set
descript : dw7914 trig control setup.
			only DW7914 devices are supported.
====================================================================================== */
static void dw7914_trigctrl_set(struct dw791x_priv *dw791x, u8 type, u8 bitset)
{
	int ret = 0;
	unsigned int reg = 0;

	reg = dw791x_byte_read(dw791x, DW7914_TRIG_CTRL);

	if (bitset == BITSET)
	{
		reg = reg | type;
	}
	else
	{
		reg = reg & (~type);
	}

	ret = dw791x_word_write(dw791x, DW7914_TRIG_CTRL, reg);

	dw_debug("trig ctrl register set: %x\n", reg);

	if (ret < 0)
		dw_err("%s i2c trig register set fail.!\n", dw791x->dev_name);
}

/* =====================================================================================
function : dw7914_autovib_set
descript : dw7914 auto track & brake ctrl setup.
			only DW7914 devices are supported.

			Exmaple Design Info
			- LRA F0 IN : 3F 00 AA 
			- Track Pattern : 34 80 (follow setting value)
			- Track Loop & Track0 cycle : 34 80
			- Track_Play : 35 09 (106ms)
			- Track_Idle : 36 00 (10.66ms)
			- Track0 Wave : 37 00
			- Track1 Wave : 38 01
			- NULL=GND time : 3c 33 (100us)
			- ZXD time : 3d 67 ( 450us)
			- Actuator conversions range : 147Hz ~ 193Hz	
====================================================================================== */
static void dw7914_autovib_set(struct dw791x_priv *dw791x)
{
	// setp 0: ------ auto track wave transfer
	transfer_atuovib_wave(dw791x, (u8 *)auto_track_base_m1);

	// step 1: ------ control mode setting
	dw7914_mode_set(dw791x, D14_AUTO_TRACK, BITSET);

	// step 2: ------ register setting
	dw791x_word_write(dw791x, DW7914_LRA_F0_INH, 0xAA);
	dw791x_byte_write(dw791x, DW7914_TRACK_CTRL0, 0x80);
	dw791x_byte_write(dw791x, DW7914_TRACK_CTRL1, 0x09);
	dw791x_byte_write(dw791x, DW7914_TRACK_CTRL2, 0x00);
	dw791x_byte_write(dw791x, DW7914_TRACK0_WAVE, 0x00);
	dw791x_byte_write(dw791x, DW7914_TRACK1_WAVE, 0x01);
	dw791x_byte_write(dw791x, DW7914_ZXD_CTRL1, 0x33);
	dw791x_byte_write(dw791x, DW7914_ZXD_CTRL2, 0x67);
}

/* =====================================================================================
function : dw7914_autobrake_set
descript : dw7914 auto brake ctrl setup.
			only DW7914 devices are supported.	
====================================================================================== */
static void dw7914_autobrake_set(struct dw791x_priv *dw791x, int on_off)
{
	if (dw791x->device == DW7914)
	{
		if (on_off == 1)
		{
			dw7914_mode_set(dw791x, D14_AUTO_BRAKE, BITSET);
			dw791x_byte_write(dw791x, DW7914_BRAKE0_MWAVE, 0x09);
			dw791x_byte_write(dw791x, DW7914_BRAKE1_MWAVE, 0x00);
			dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x28);

			dw791x_Mem_Write(dw791x, (u8 *)auto_brake_wave, sizeof(auto_brake_wave)); //autobrake waveform for f0 = 165hz
		}
		else
		{
			dw7914_mode_set(dw791x, D14_AUTO_BRAKE, BITCLR);
			dw791x_byte_write(dw791x, DW7914_BRAKE0_MWAVE, 0x00);
			dw791x_byte_write(dw791x, DW7914_BRAKE1_MWAVE, 0x00);
		}
		dw_debug("set autobrake %s\n", on_off ? "ON" : "OFF");
	}
	else
		dw_err("auto brake is not support.\n");

	return;
}

/* =====================================================================================
function : dw7914_checksum
descript : dw7914 memory check-sum check function
			only DW7914 devices are supported.
			- page : sram(12KB) memory model select
			- return 0 (sucess)
			- return -1 (fail)
====================================================================================== */
static int dw7914_checksum(struct dw791x_priv *dw791x, u32 type, u32 page)
{
	int ret = 0;
	u8 *p = NULL, rx[4];
	u32 point, gsum;

	dw791x_fifo_size_set(dw791x, FIFO_4KB);

	if (request_firmware(&dw791x->fw, FW_FILE, &dw791x->dwclient->dev) == 0)
	{
		p = (u8 *)dw791x->fw->data;
		point = (1 + page) * BIN_BASE;
		rx[3] = p[point - 4];
		rx[2] = p[point - 3];
		rx[1] = p[point - 2];
		rx[0] = p[point - 1];
		dw791x->checksum = (u32)rx[3] << 24 | (u32)rx[2] << 16 | (u32)rx[1] << 8 | (u32)rx[0] << 0;
		dw_debug("fw_name= %s fw_size= %ld fw_checksum= %x\n", FW_FILE, dw791x->fw->size, dw791x->checksum);
	}
	release_firmware(dw791x->fw);

	dw7914_mode_set(dw791x, D14_CHKSUM, BITSET);
	msleep(20);
	rx[3] = dw791x_byte_read(dw791x, DW7914_RAM_CHKSUM3);
	rx[2] = dw791x_byte_read(dw791x, DW7914_RAM_CHKSUM2);
	rx[1] = dw791x_byte_read(dw791x, DW7914_RAM_CHKSUM1);
	rx[0] = dw791x_byte_read(dw791x, DW7914_RAM_CHKSUM0);
	gsum = (u32)rx[3] << 24 | (u32)rx[2] << 16 | (u32)rx[1] << 8 | (u32)rx[0] << 0;

	if (gsum == dw791x->checksum)
	{
		ret = 0;
		dw_debug("memory checksum passed!: %x\n", gsum);
	}
	else
	{
		ret = -1;
		dw_debug("memory checksum failed!: %x, %x\n", gsum, dw791x->checksum);
	}

	return ret;
}

/* =====================================================================================
function : dw791x_seq_write
descript : bulk I2C transfer function
====================================================================================== */
static int dw791x_seq_write(struct dw791x_priv *dw791x, u32 addr, u32 ram_addr, u32 ram_bit, u8 *data, u32 size)
{
	int ret = 0;
	u8 *xbuf;
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc = dw791x->dwclient;

	xbuf = kmalloc(I2C_BUF_SIZE + 4, GFP_KERNEL);
	if (!xbuf)
	{
		kfree(xbuf);
		return -1;
	}

	if (size > I2C_BUF_SIZE + 3)
	{
		ret = -1;
		dw_debug("The transferable size has been exceeded.\n");
		return ret;
	}

	if (ram_bit == RAM_ADDR8)
	{
		xbuf[0] = (u8)addr;
		xfer[0].addr = CHIP_ID;
		xfer[0].len = size + 1;
		xfer[0].flags = 0;
		xfer[0].buf = xbuf;
		memcpy((u8 *)xbuf + 1, (u8 *)data, size);
	}
	else if (ram_bit == RAM_ADDR16)
	{
		xbuf[0] = addr;
		xbuf[1] = ram_addr >> 8;
		xbuf[2] = (u8)ram_addr;

		xfer[0].addr = CHIP_ID;
		xfer[0].len = size + 3;
		xfer[0].flags = 0;
		xfer[0].buf = xbuf;
		memcpy((u8 *)xbuf + 3, (u8 *)data, size);
	}
	ret = i2c_transfer(i2c_fnc->adapter, xfer, 1);

	kfree(xbuf);

	return ret;
}

static int dw791x_Mem_Write(struct dw791x_priv *dw791x, u8 *data, u32 size)
{
	u32 reg;
	int ret = 0;

	ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, 0, RAM_ADDR8, (u8 *)data, 7);
	reg = (int)data[2] << 8 | data[3];
	ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, reg, RAM_ADDR16, (u8 *)data + 7, size - 7);

	if (ret < 0)
		dw_debug("memory write fail\n");

	return ret;
}

/* =====================================================================================
function : dw791x_play_mode_sel
descript : select the play mode
====================================================================================== */
static int dw791x_play_mode_sel(struct dw791x_priv *dw791x, int play_mode)
{
	int ret = 0;

	dw_debug("play_mode : %d\n", play_mode);

	if (dw791x->device == DW7912)
	{
		switch (play_mode)
		{
		case HAPTIC_STOP:
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_STOP);
			break;
		case HAPTIC_TICK:
#ifdef DW791X_COIN0832
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x02);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_COIN1040
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x04);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_GAMMA
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
			break;
		case HAPTIC_SHORT_PRESS:
#ifdef DW791X_COIN0832
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x05);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_COIN1040
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_GAMMA
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x02);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
			break;
		case HAPTIC_LONG_PRESS:
#ifdef DW791X_COIN0832
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_COIN1040
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_GAMMA
			dw791x_byte_write(dw791x, DW7912_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 0x03);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
			break;
		case HAPTIC_WAVE_PLAY:
			dw791x_vd_clamp_set(dw791x, dw791x->dw791x_haptics.vdclamp);
			dw791x_byte_write(dw791x, DW7912_MODE, 0x00);
			queue_work(dw791x->haptic_wave_work, &dw791x->wave_work);
			break;
		case HAPTIC_RTP_TEST:
			if (request_firmware(&dw791x->fw, FW_FILE, &dw791x->dwclient->dev) == 0)
			{
				dw_debug(" fw_name= %s fw_size= %ld\n", FW_FILE, dw791x->fw->size);
				request_transfer_mem_wave(dw791x, 0, 0, (u8 *)dw791x->fw->data, dw791x->fw->size);
			}
			release_firmware(dw791x->fw);
			dw791x_byte_write(dw791x, DW7912_WAVQ0, 1);
			dw791x_byte_write(dw791x, DW7912_WAVQ1, 2);
			dw791x_byte_write(dw791x, DW7912_WAVQ2, 3);
			dw791x_byte_write(dw791x, DW7912_WAVQ3, 4);
			dw791x_byte_write(dw791x, DW7912_WAVQ4, 5);
			dw791x_byte_write(dw791x, DW7912_WAVQ5, 6);
			dw791x_byte_write(dw791x, DW7912_WAVQ6, 7);
			dw791x_byte_write(dw791x, DW7912_WAVQ7, 8);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
			break;
		}
	}
	else if (dw791x->device == DW7914)
	{
		switch (play_mode)
		{
		case HAPTIC_STOP:
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_STOP);
			break;
		case HAPTIC_TICK: // pixel -------------------------- jog dial
#ifdef DW791X_COIN0832
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x05);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x02);
			dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x2C);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_COIN1040
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x04);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_GAMMA
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, DW7914_WAV_SEQ_L0, 0x0);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
			break;
		case HAPTIC_SHORT_PRESS:
#ifdef DW791X_COIN0832
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x05);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x05);
			dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x0C);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_COIN1040
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_GAMMA
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x01);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x02);
			dw791x_byte_write(dw791x, DW7914_WAV_SEQ_L0, 0x0);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
			break;
		case HAPTIC_LONG_PRESS:
#ifdef DW791X_COIN0832
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x05);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x01);
			dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x0D);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_COIN1040
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x05);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x02);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
#ifdef DW791X_GAMMA
			dw791x_byte_write(dw791x, DW7914_PWM, 0x01);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x05);
			dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x03);
			dw791x_byte_write(dw791x, DW7914_WAV_SEQ_L0, 0x0);
			dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
#endif
			break;
		case HAPTIC_RTP_TEST: // haptic ------------------------- short press
			dw791x_vd_clamp_set(dw791x, dw791x->dw791x_haptics.vdclamp);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x00);
			//request_transfer_rtp_wave((u8*)dw7914_RTP_TEST_625ms, 10);	// 6.24sec play
			break;
		case HAPTIC_WAVE_PLAY: // request f/w for RTP
			dw791x->haptic_wave_on = 1;
			dw791x_vd_clamp_set(dw791x, dw791x->dw791x_haptics.vdclamp);
			dw791x_byte_write(dw791x, DW7914_MODE, 0x00);
			queue_work(dw791x->haptic_wave_work, &dw791x->wave_work);
			break;
		case HAPTIC_TEST:
			dw7914_autovib_set(dw791x);
			dw7914_checksum(dw791x, 0, 0);
			dw7914_trigctrl_set(dw791x, 0, 0);
			dw791x_vd_clamp_set(dw791x, dw791x->dw791x_haptics.vdclamp);
#ifdef BST_OPT
			dw791x_bst_option(dw791x, 0, 0, 0);
#endif
			if (request_firmware(&dw791x->fw, FW_FILE, &dw791x->dwclient->dev) == 0)
			{
				dw_debug(" fw_name= %s fw_size= %ld\n", FW_FILE, dw791x->fw->size);
				request_transfer_mem_wave(dw791x, 0, 1, (u8 *)dw791x->fw->data, dw791x->fw->size);
			}
			release_firmware(dw791x->fw);

			if (request_firmware(&dw791x->fw, RTP_TEST_FW_FILE, &dw791x->dwclient->dev) == 0)
			{
				dw_debug("fw_name= %s fw_size= %ld\n", RTP_TEST_FW_FILE, dw791x->fw->size);
				request_transfer_rtp_wave(dw791x, (u8 *)dw791x->fw->data, 2);
			}
			release_firmware(dw791x->fw);
			break;
		}
	}

	return ret;
}

/* =====================================================================================
function : transfer_autovib_wave
descript : transfer auto track brake memory wave
			only DW7914 devices are supported.
			header(pure header 5byte), body(pure wave data), not include memory address.
====================================================================================== */
static int transfer_atuovib_wave(struct dw791x_priv *dw791x, u8 *wave)
{
	int ret, i, loop, tail, wave_size;
	int start_addr, set_addr, trans_size;

	// setp 0: ---------------------------- dw7914 memory mode set
	ret = 0;
	dw7914_mode_set(dw791x, D14_MEM_MODE, MEM);

	// setp 1: ---------------------------- dw7914 header data transfer
	set_addr = (u32)wave[0] << 8 | wave[1];
	dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, set_addr, RAM_ADDR16, (u8 *)wave + 2, 5);

	// step 2: ---------------------------- dw7914 body data transfer
	trans_size = I2C_TRANS_MEM;
	set_addr = (int)wave[2] << 8 | wave[3];
	wave_size = (int)(0xF & wave[4]) << 8 | wave[5];

	dw_debug("auto vib body wave size: %d\n", wave_size);

	loop = wave_size / trans_size;
	tail = wave_size % trans_size;

	for (i = 0; i < loop; i++)
	{
		start_addr = set_addr + i * trans_size;
		dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, start_addr, RAM_ADDR16, (u8 *)wave + 7 + i * trans_size, trans_size);
	}

	if (tail > 0)
	{
		start_addr = set_addr + loop * trans_size;
		dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, start_addr, RAM_ADDR16, (u8 *)wave + 7 + loop * trans_size, tail);
	}

	dw_debug("auto vib memory data write done!\n");

	return ret;
}

/* =====================================================================================
function : request_transfer_rtp_wave
descript : request transfer rtp wave function
			Only DW7914 devices are supported.
header   : 5byte[0xAE, size, size, size, size] total size 32bit
data	 : rtp wave data
====================================================================================== */
static int request_transfer_rtp_wave(struct dw791x_priv *dw791x, u8 *wave, u32 repeat)
{
	u8 rx[2];
	u32 ret = 0;
	u32 run, curcnt, trans_size, play_size;
	u32 empty_lock, fifo_level, fifo_size, wait_t;

	// step 0: ------------------ timer set check & clear & size read
	run = curcnt = empty_lock = fifo_level = 0;
#ifdef HAPTIC_WAVE
	play_size = dw791x->fw->size;
#else
	play_size = (u32)wave[1] << 24 | (u32)wave[2] << 16 | (u32)wave[3] << 8 | (u32)wave[4] << 0;
#endif

	if (dw791x->device == DW7912)
	{
		dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_STOP);
	}
	else if (dw791x->device == DW7914)
	{
		dw7914_mode_set(dw791x, D14_FIFO_FLUSH, BITSET);
		dw7914_mode_set(dw791x, D14_MEM_MODE, RTP);
		dw791x_byte_read(dw791x, DW7914_STATUS0);
		dw791x_byte_read(dw791x, DW7914_STATUS1);
	}

	// step 1: ------------------ fifo set size read
	rx[0] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_addrh);
	rx[1] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_addrl);
	fifo_size = SRAM_BASE - ((u32)rx[0] << 8 | rx[1]);
	dw_debug("fifo area size: %dbyte\n", fifo_size);

	// step 2 : ----------------- short time play
	if (play_size < I2C_TRANS_RTP)
	{
#ifdef HAPTIC_WAVE
		ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.rtp_input, 0, RAM_ADDR8, (unsigned char *)wave, play_size);
#else
		ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.rtp_input, 0, RAM_ADDR8, (unsigned char *)wave + 5, play_size);
#endif
		dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
		dw_debug("rtp play done! size: %d\n", play_size);
		return ret;
	}

	while (!run)
	{
		// step 3: ------------------ fifo level check
		if (dw791x->device == DW7912)
		{
			rx[0] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_level);
			fifo_level = (u32)rx[0] * 64 + 64;
		}
		else if (dw791x->device == DW7914)
		{
			rx[0] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_level);
			rx[1] = dw791x_byte_read(dw791x, dw791x->dw791x_reg_addr.fifo_level + 1);
			fifo_level = ((u32)rx[0] << 8) | rx[1];
		}

		// step 4: ------------------ i2c transfer size define
		if ((fifo_size - fifo_level) > I2C_TRANS_RTP)
		{
			trans_size = I2C_TRANS_RTP;
		}
		else
		{
			trans_size = fifo_size - fifo_level;
		}

		// step 5: ------------------ wave data transfer loop
		if (curcnt < play_size)
		{
			if ((curcnt + trans_size) > play_size)
			{
				trans_size = play_size - curcnt;
			}

			if (trans_size > 0)
			{
#ifdef HAPTIC_WAVE
				ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.rtp_input, 0, RAM_ADDR8, (unsigned char *)wave + curcnt, trans_size); // fifo full write
				curcnt += trans_size;
#else
				ret = dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.rtp_input, 0, RAM_ADDR8, (unsigned char *)wave + 5 + curcnt, trans_size); // fifo full write
				curcnt += trans_size;
#endif
			}

			if (empty_lock == 0)
			{
				if (fifo_level > (fifo_size / 2))
				{
					empty_lock = 1;
					dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_back, DRV_PLAY);
				}
			}

			if (empty_lock == 1)
			{
				rx[0] = dw791x_byte_read(dw791x, DW7914_STATUS0);
				rx[1] = dw791x_byte_read(dw791x, DW7914_STATUS1);

				if ((rx[0] & 0x8F) || (rx[1] & 0xFE))
				{
					run = 1;
					dw791x->haptic_wave_on = 0;
					dw_err("An event occurred during rtp operation! status0:%x, status1:%x\n", rx[0], rx[1]);
					break;
				}
				if ((rx[1] & 0x01) == 0)
				{
					run = 1;
					dw791x->haptic_wave_on = 0;
					dw_err("rtp operation stop!. status0:%x, status1:%x\n", rx[0], rx[1]);
					break;
				}
			}

			if (fifo_level > (fifo_size - fifo_size / 3)) // wait interrupt delay
			{
#ifdef HAPTIC_WAVE
				wait_t = usecs_to_jiffies(100); // set 100us fixed
#else
				wait_t = msecs_to_jiffies(10);						   // set 10ms fixed
#endif
				schedule_timeout_interruptible(wait_t);
			}
		}

		// step 6: ------------------ wave data transfer done!
		else
		{ // play stop
			if (repeat > 1)
			{
				curcnt = 0;
				repeat--;
			}
			else
			{
				run = 1;
				dw791x->haptic_wave_on = 0;
				dw_debug("The rtp operation is finished. size: %d, repeat: %d\n", curcnt, repeat);
				break;
			}
		}
	}

	return ret;
}

/* =====================================================================================
function : request_transfer_mem_wave
descript : request transfer rtp mem function
			only use binary waves.  memory addresses not included.
page	 : 12kb bin file model start page 0 ~ n
point	 : memory header call number 0: all wirte, 1: header no.1
pw_data  : bin file data pointer
size	 : bin wave size
====================================================================================== */
static int request_transfer_mem_wave(struct dw791x_priv *dw791x, u32 page, u32 point, u8 *fw_data, u32 size)
{
	int ret = 0;
	u32 i, loop, tail, wave_size, start_addr, set_addr, trans_size, fw_point;

	// setp 0: ---------------------------- dw791x memory mode set
	trans_size = I2C_TRANS_MEM;
	ret = dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_mode, MEM);

	if (ret < 0)
		return ret;

	// step 1: ---------------------------- dw791x find header data pointer
	if (point > 0)
	{
		set_addr = (point * 5) - 4;
		fw_point = set_addr + BIN_BASE * page;
		dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, set_addr, RAM_ADDR16, (u8 *)fw_data + fw_point, 5);

		set_addr = (u32)(fw_data[fw_point + 0]) << 8 | fw_data[fw_point + 1];
		wave_size = (u32)(0xF & fw_data[fw_point + 2]) << 8 | fw_data[fw_point + 3];
	}
	else
	{
		set_addr = 0;
		wave_size = BIN_BASE;
		fw_point = (1 + page) * BIN_BASE;
	}
	dw_debug("request trans page: %d point: %d size: %d\n", page, point, wave_size);

	// step 2: ---------------------------- dw791x find body data pointer
	loop = wave_size / trans_size;
	tail = wave_size % trans_size;
	dw_debug("set addr:%d loop:%d tail:%d\n", set_addr, loop, tail);

	for (i = 0; i < loop; i++)
	{
		start_addr = set_addr + (i * trans_size);
		fw_point = start_addr + BIN_BASE * page;
		dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, start_addr, RAM_ADDR16, (u8 *)fw_data + fw_point, trans_size);
	}

	if (tail > 0)
	{
		start_addr = set_addr + (loop * trans_size);
		fw_point = start_addr + BIN_BASE * page;
		dw791x_seq_write(dw791x, dw791x->dw791x_reg_addr.mem_input, start_addr, RAM_ADDR16, (u8 *)fw_data + fw_point, tail);
	}

	dw_debug("requeset firmware transfer complete!\n");

	return ret;
}

static void dw791x_mem_init(struct dw791x_priv *dw791x, dw791x_waveform_data *data, dw791x_waveform_id max)
{
	int i = 0;

	for (i = 1; i < WAVEFORM_ID_MAX; i++)
	{
		dw791x_Mem_Write(dw791x, (u8 *)data[i].wave_data, data[i].size);
	}

	return;
}

/* =====================================================================================
function : dw791x_device_init
descript : apply device default setting.
			auto detected devices model.
====================================================================================== */
static int dw791x_device_init(struct dw791x_priv *dw791x)
{
	int ret = -1;

	// step 0: --------------- find the device
	dw791x->device = dw791x_byte_read(dw791x, DW791x_PID);
	ret = dw791x_status_get(dw791x);

	// step 1: --------------- change the register according to the device.
	if (dw791x->device == DW7912)
	{
		dw791x->dw791x_reg_addr.play_mode = DW7912_MODE;
		dw791x->dw791x_reg_addr.play_back = DW7912_PLAYBACK;
		dw791x->dw791x_reg_addr.rtp_input = DW7912_RTP_INPUT;
		dw791x->dw791x_reg_addr.fifo_level = DW7912_RTP_INPUT;
		dw791x->dw791x_reg_addr.mem_input = DW7912_RAM_ADDR_H;
		dw791x->dw791x_reg_addr.fifo_addrh = DW7912_FIFO_ADDRH;
		dw791x->dw791x_reg_addr.fifo_addrl = DW7912_FIFO_ADDRL;
		dw791x->version = DW7912_VERSION;
#ifdef DW791X_GAMMA
		dw791x_mem_init(dw791x, gamma, WAVEFORM_ID_MAX);
#ifdef DW791X_INT
		ret = dw791x_intz_en_set(dw791x, 0x0F);
#endif
#endif
#ifdef DW791X_COIN0832
		dw791x_mem_init(dw791x, coin0832, WAVEFORM_ID_MAX);
#ifdef DW791X_INT
		ret = dw791x_intz_en_set(dw791x, 0x0F);
#endif
#endif
#ifdef DW791X_COIN1040
		dw791x_mem_init(dw791x, coin1040, WAVEFORM_ID_MAX);
#ifdef DW791X_INT
		ret = dw791x_intz_en_set(dw791x, 0x0F);
#endif
#endif
		ret = dw791x_byte_write(dw791x, DW7912_WAVQ0, 1);
		ret = dw791x_mem_gain_set(dw791x, 0);
		ret = dw791x_byte_write(dw791x, dw791x->dw791x_reg_addr.play_mode, MEM);

		snprintf(dw791x->dev_name, 8, "DW7912");
	}
	else if (dw791x->device == DW7914)
	{
		dw791x->dw791x_reg_addr.play_mode = DW7914_MODE;
		dw791x->dw791x_reg_addr.play_back = DW7914_PLAYBACK;
		dw791x->dw791x_reg_addr.rtp_input = DW7914_RTP_INPUT;
		dw791x->dw791x_reg_addr.mem_input = DW7914_RAM_ADDR_H;
		dw791x->dw791x_reg_addr.fifo_addrh = DW7914_FIFO_ADDRH;
		dw791x->dw791x_reg_addr.fifo_addrl = DW7914_FIFO_ADDRL;
		dw791x->dw791x_reg_addr.fifo_level = DW7914_FIFO_STATUSH;
		dw791x->version = DW7914_VERSION;
		dw791x_fifo_size_set(dw791x, FIFO_8KB);
		dw791x_vd_clamp_set(dw791x, dw791x->dw791x_haptics.vdclamp);
		ret = dw791x_byte_write(dw791x, DW7914_MODE, 0x05); // mem mode
		ret = dw791x_mem_gain_set(dw791x, 0);
#ifdef DW791X_GAMMA
		ret = dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x03);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE0_MWAVE, 0x00);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE1_MWAVE, 0x04);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x28);

		dw791x_mem_init(dw791x, gamma, WAVEFORM_ID_MAX);
		dw7914_autobrake_set(dw791x, BITCLR);
#ifdef DW791X_INT
		ret = dw791x_intz_en_set(dw791x, 0x0F);
		ret = dw791x_byte_write(dw791x, DW7914_TRIG_DET_EN, 0x09); // Trigger1 Falling detect.

		ret = dw791x_byte_write(dw791x, DW7914_TRIG_CTRL, 0x65);
		ret = dw791x_byte_write(dw791x, DW7914_TRIG1_F_WAVE, 0x1);
//		ret = dw791x_byte_write(dw791x, DW7914_TRIG2_F_WAVE, 0x1);
//		ret = dw791x_byte_write(dw791x, DW7914_TRIG3_F_WAVE, 0x1);
#endif
#endif
#ifdef DW7914_COIN0832
		ret = dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x01);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE0_MWAVE, 0x03);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE1_MWAVE, 0x04);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x0D);

		dw791x_mem_init(dw791x, coin0832, WAVEFORM_ID_MAX);
		dw7914_autobrake_set(dw791x, BITCLR);
#ifdef DW791X_INT
		ret = dw791x_intz_en_set(dw791x, 0x8F);
		ret = dw791x_byte_write(dw791x, DW7914_TRIG_DET_EN, 0x09); // Trigger1 Falling detect.

		ret = dw791x_byte_write(dw791x, DW7914_TRIG_CTRL, 0x65);
		ret = dw791x_byte_write(dw791x, DW7914_TRIG1_F_WAVE, 0x1);
//		ret = dw791x_byte_write(dw791x, DW7914_TRIG2_F_WAVE, 0x1);
//		ret = dw791x_byte_write(dw791x, DW7914_TRIG3_F_WAVE, 0x1);
#endif
#endif
#ifdef DW7914_COIN1040
		ret = dw791x_byte_write(dw791x, DW7914_WAVQ0, 0x02);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE0_MWAVE, 0x00);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE1_MWAVE, 0x03);
		ret = dw791x_byte_write(dw791x, DW7914_BRAKE_MCTRL, 0x28);

		dw791x_mem_init(dw791x, coin1040, WAVEFORM_ID_MAX);
		dw7914_autobrake_set(dw791x, BITCLR);
#ifdef DW791X_INT
		ret = dw791x_intz_en_set(dw791x, 0x8F);
		ret = dw791x_byte_write(dw791x, DW7914_TRIG_DET_EN, 0x09); // Trigger1 Falling detect.

		ret = dw791x_byte_write(dw791x, DW7914_TRIG_CTRL, 0x65);
		ret = dw791x_byte_write(dw791x, DW7914_TRIG1_F_WAVE, 0x1);
//		ret = dw791x_byte_write(dw791x, DW7914_TRIG2_F_WAVE, 0x1);
//		ret = dw791x_byte_write(dw791x, DW7914_TRIG3_F_WAVE, 0x1);
#endif
#endif

		snprintf(dw791x->dev_name, 8, "DW7914A");
	}
	dw_err("DWA %s driver version %x\n", dw791x->dev_name, dw791x->version);

	return ret;
}

static void haptic_play_work(struct work_struct *work)
{
	struct dw791x_priv *dw791x = container_of(work, struct dw791x_priv, wave_work);

	dw_debug("start haptic play work!!\n");
	if (request_firmware(&dw791x->fw, RTP_FW_FILE, &dw791x->dwclient->dev) == 0)
	{
		dw_debug("fw_name= %s fw_size= %ld\n", RTP_FW_FILE, dw791x->fw->size);
#ifdef HAPTIC_WAVE
		request_transfer_rtp_wave(dw791x, (u8 *)dw791x->fw->data, 1); // 10	// 6.24sec play
#else
		request_transfer_rtp_wave(dw791x, (u8 *)dw791x->fw->data, 10); // 6.24sec play
#endif
	}
	release_firmware(dw791x->fw);
	dw791x->haptic_wave_on = 0;

	return;
}

enum led_brightness dw791x_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static void dw791x_brightness_set(struct led_classdev *cdev,
								  enum led_brightness level)
{
}

static ssize_t enableVIB_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int mode = 0;

	sscanf(buf, "%d", &mode);
	dw_debug("enableVIB_set cmd: buf=%s\n", buf);

	dw791x_play_mode_sel(dw791x, mode);
	return count;
}

static ssize_t enableVIB_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int ret = 0;

	ret = dw791x_byte_read(dw791x, DW791x_PID);
	dw_debug("product id : %x\n", ret);

	ret = dw791x_byte_read(dw791x, DW791x_STATUS);
	dw_debug("chip status : %x\n", ret);

	return snprintf(buf, PAGE_SIZE, "[VIB] status = %x\n", ret);
}

static ssize_t patternVIB_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	return count;
}

static ssize_t patternVIB_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	int ret = 0;

	return ret;
}

/* =====================================================================================
function : dw791x_regread_set
descript : adb command
           # echo [address] > reg_read
====================================================================================== */
static ssize_t dw791x_regread_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	unsigned int reg_addr = 0;
	int ret = 0;

	sscanf(buf, "0x%2x", &reg_addr);

	dw791x->reg_read = (reg_addr & 0xFF);

	if (dw791x->device == DW7912)
	{
		if (dw791x->reg_read < 0 || dw791x->reg_read > 0x2F)
		{
			dw791x->reg_read = 0x0;
			dw_err("Invailed register address.\n");
			return count;
		}
		ret = dw791x_byte_read(dw791x, dw791x->reg_read);
		dw_debug("regisger read[%04x] = %x\n", dw791x->reg_read, ret);
	}
	else if (dw791x->device == DW7914)
	{
		if (dw791x->reg_read < 0 || dw791x->reg_read > 0x5F)
		{
			dw791x->reg_read = 0;
			dw_err("Invailed register address.\n");
			return count;
		}
		ret = dw791x_byte_read(dw791x, dw791x->reg_read);
		dw_debug("regisger read[%04x] = %x\n", dw791x->reg_read, ret);
	}
	else
		dw_err("register read fail. (id = %d)\n", dw791x->device);

	return count;
}

static ssize_t dw791x_regread_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	u32 ret;

	ret = dw791x_byte_read(dw791x, dw791x->reg_read);
	dw_debug("regisger read[%04x] = %x\n", dw791x->reg_read, ret);

	return snprintf(buf, PAGE_SIZE, "[0x%04x] = %x\n", dw791x->reg_read, ret);
}

/* =====================================================================================
function : dw791x_regwrite_set
descript : adb command
           # echo [address] [value] > reg_write
====================================================================================== */
static ssize_t dw791x_regwrite_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	unsigned int reg_addr = 0;
	unsigned int reg_wr = 0;
	int ret = 0;

	sscanf(buf, "0x%2x 0x%2x", &reg_addr, &reg_wr);

	reg_addr = (reg_addr & 0xFF);
	reg_wr = (reg_wr & 0xFF);

	if (dw791x->device == DW7912)
	{
		if (reg_addr < 0 || reg_addr > 0x2F)
		{
			reg_addr = 0x0;
			dw_err("Invailed register address.\n");
			return count;
		}
		ret = dw791x_byte_write(dw791x, reg_addr, reg_wr);
		dw_debug("regisger write[%04x] = %x\n", reg_addr, dw791x_byte_read(dw791x, reg_addr));
	}
	else if (dw791x->device == DW7914)
	{
		if (reg_addr < 0 || reg_addr > 0x5F)
		{
			reg_addr = 0;
			dw_err("Invailed register address.\n");
			return count;
		}
		ret = dw791x_byte_write(dw791x, reg_addr, reg_wr);
		dw_debug("regisger write[%04x] = %x\n", reg_addr, dw791x_byte_read(dw791x, reg_addr));
	}
	else
		dw_debug("register read fail. (id = %d)\n", dw791x->device);

	return count;
}

/* =====================================================================================
function : innermem_read_set
descript : function for reading ram data
           adb command
           # echo [address] [size] > inner_mem
====================================================================================== */
static ssize_t innermem_read_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int mem_address = 0, size = 0;
	int i, ret;

	sscanf(buf, "%d %d", &mem_address, &size);

	for (i = 0; i < size; i++)
	{

		if (dw791x->device == DW7912)
		{
			ret = dw791x_byte_write(dw791x, DW7912_RAM_ADDR_H, (mem_address >> 8) & 0xff);
			ret = dw791x_byte_write(dw791x, DW7912_RAM_ADDR_L, mem_address & 0xff);
			dw_debug("inner mem read[%04x] = %x\n", mem_address, dw791x_byte_read(dw791x, DW7912_RAM_DATA));
		}
		else if (dw791x->device == DW7914)
		{
			ret = dw791x_byte_write(dw791x, DW7914_RAM_ADDR_H, (mem_address >> 8) & 0xff);
			ret = dw791x_byte_write(dw791x, DW7914_RAM_ADDR_L, mem_address & 0xff);
			dw_debug("inner mem read[%04x] = %x\n", mem_address, dw791x_byte_read(dw791x, DW7914_RAM_DATA));
		}
		else
			dw_debug("inner memory read fail. (id = %d)\n", dw791x->device);

		mem_address++;
	}

	return count;
}

static ssize_t innermem_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	int ret = 0;

	return ret;
}

static ssize_t dw791x_swreset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int sw_reset = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &sw_reset);

	if (dw791x->device == DW7912)
	{
		ret = dw791x_byte_write(dw791x, DW7912_SW_RESET, sw_reset);
	}
	else if (dw791x->device == DW7914)
	{
		ret = dw791x_byte_write(dw791x, DW7914_SW_RESET, sw_reset);
	}
	else
	{
		dw_err("sw reset fail. (id = %x)\n", dw791x->device);
		return count;
	}

	dw_debug(" %s sw reset!!\n", (dw791x->device == DW7912) ? "dw7912" : "dw7914");

	return count;
}

static ssize_t vdclamp_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int vdclamp = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &vdclamp);
	if (ret < 0)
	{
		dw_err("kstrtouint failed\n");
		return ret;
	}
	dw791x->dw791x_haptics.vdclamp = vdclamp;

	dw_debug("vdclamp_set voltage = %d\n", vdclamp);

	dw791x_vd_clamp_set(dw791x, dw791x->dw791x_haptics.vdclamp);

	return count;
}

static ssize_t vdclamp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.vdclamp);
}

static ssize_t mem_vdclamp_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int waveform_id = -1;
	int vdclamp = 0;

	sscanf(buf, "%d %d", &waveform_id, &vdclamp);

	dw791x->dw791x_haptics.mem_vdclamp = vdclamp;

	if (waveform_id < WAVEFORM_ID_MAX)
	{
		dw791x_mem_vd_clamp_set(dw791x, waveform_id, dw791x->dw791x_haptics.mem_vdclamp);
		dw_debug("waveform_id : %d vdclamp_set voltage = %d\n", waveform_id, vdclamp);
	}
	else if (waveform_id == WAVEFORM_ID_NONE)
	{
		dw_debug("waveform id is zero\n");
	}

	return count;
}

static ssize_t mem_vdclamp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.mem_vdclamp);
}

static ssize_t autobrake_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int autobrake = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &autobrake);
	if (ret < 0)
	{
		dw_err("kstrtouint failed\n");
		return ret;
	}
	dw791x->dw791x_haptics.auto_brake = autobrake;

	dw_debug("autobrake_set = %d\n", autobrake);

	dw7914_autobrake_set(dw791x, dw791x->dw791x_haptics.auto_brake);

	return count;
}

static ssize_t autobrake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.auto_brake);
}

static ssize_t fifolevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int fifo_level;

	fifo_level = dw791x_fifo_level_get(dw791x);

	return snprintf(buf, PAGE_SIZE, "%d\n", fifo_level);
}

static ssize_t trigger_test_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);
	int trigger_num = 0;
	int on_off = 0;

	sscanf(buf, "%d %d", &trigger_num, &on_off);

	if (trigger_num == 1)
	{
		if (on_off)
			gpio_set_value(dw791x->trigger1_gpio, 1);
		else
			gpio_set_value(dw791x->trigger1_gpio, 0);
	}
	if (trigger_num == 2)
	{
		if (on_off)
			gpio_set_value(dw791x->trigger2_gpio, 1);
		else
			gpio_set_value(dw791x->trigger2_gpio, 0);
	}
	if (trigger_num == 3)
	{
		if (on_off)
			gpio_set_value(dw791x->trigger3_gpio, 1);
		else
			gpio_set_value(dw791x->trigger3_gpio, 0);
	}

	dw_debug("trigger_test_set = %d on_off = %d\n", trigger_num, on_off);

	return count;
}

static ssize_t trigger_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.auto_brake);
}

static struct device_attribute dw791x_haptics_attrs[] = {
	__ATTR(enableVIB, 0664, enableVIB_show, enableVIB_set),
	__ATTR(patternVIB, 0664, patternVIB_show, patternVIB_set),
	__ATTR(vdclamp, 0664, vdclamp_show, vdclamp_set),
	__ATTR(auto_brake, 0664, autobrake_show, autobrake_set),
	__ATTR(fifolevel, 0664, fifolevel_show, NULL),
	__ATTR(mem_vdclamp, 0664, mem_vdclamp_show, mem_vdclamp_set),
	__ATTR(inner_mem, 0664, innermem_read_show, innermem_read_set),
	__ATTR(sw_reset, 0664, NULL, dw791x_swreset_set),
	__ATTR(reg_read, 0664, dw791x_regread_show, dw791x_regread_set),
	__ATTR(trigger_test, 0664, trigger_test_show, trigger_test_set),
	__ATTR(reg_write, 0664, NULL, dw791x_regwrite_set),
};

/* /sys/class/leds/vibrator */
static ssize_t activate_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.activate);
}

static ssize_t activate_store(struct device *dev,
							  struct device_attribute *attr, const char *buf,
							  size_t count)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	dw_debug("%s %s(%dms)\n", __func__, buf, dw791x->dw791x_haptics.duration);

	if (buf[0] == '1')
	{
		if (dw791x->dw791x_haptics.duration == DURATION_4MS)
		{
			dw791x_play_mode_sel(dw791x, HAPTIC_TICK);
		}
		else if (dw791x->dw791x_haptics.duration == DURATION_10MS)
		{
			dw791x_play_mode_sel(dw791x, HAPTIC_SHORT_PRESS);
		}
		else if (dw791x->dw791x_haptics.duration == DURATION_12MS)
		{
			dw791x_play_mode_sel(dw791x, HAPTIC_LONG_PRESS);
		}
	}
	else if (buf[0] == '0')
	{
		dw_debug("%s buf :%s stop activate\n", __func__, buf);
	}

	return count;
}

static ssize_t duration_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.duration);
}

static ssize_t duration_store(struct device *dev,
							  struct device_attribute *attr, const char *buf,
							  size_t count)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);
	int ret;
	int val;
	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
	{
		dw_err("kstrtouint failed\n");
		return ret;
	}

	dw_debug("duration: %d\n", val);
	dw791x->dw791x_haptics.duration = val;

	return count;
}

static ssize_t state_show(struct device *dev,
						  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.state);
}

static ssize_t state_store(struct device *dev,
						   struct device_attribute *attr, const char *buf,
						   size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static ssize_t rtp_input_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.rtp_input_node);
}

static ssize_t rtp_input_store(struct device *dev,
							   struct device_attribute *attr, const char *buf,
							   size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static ssize_t mode_show(struct device *dev,
						 struct device_attribute *attr, char *buf)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return 0;
}

static ssize_t mode_store(struct device *dev,
						  struct device_attribute *attr, const char *buf,
						  size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static ssize_t scale_show(struct device *dev,
						  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.scale);
}

static ssize_t scale_store(struct device *dev,
						   struct device_attribute *attr, const char *buf,
						   size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static ssize_t ctrl_loop_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.ctrl_loop);
}

static ssize_t ctrl_loop_store(struct device *dev,
							   struct device_attribute *attr, const char *buf,
							   size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static ssize_t set_sequencer_store(struct device *dev,
								   struct device_attribute *attr,
								   const char *buf, size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static ssize_t lp_trigger_effect_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_dev = dev_get_drvdata(dev);
	struct dw791x_priv *dw791x = container_of(led_dev, struct dw791x_priv, cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dw791x->dw791x_haptics.lp_trigger_effect);
}

static ssize_t lp_trigger_effect_store(struct device *dev,
									   struct device_attribute *attr,
									   const char *buf, size_t count)
{
	//	struct led_classdev *led_dev = dev_get_drvdata(dev);
	//	struct dw791x_priv *dw791x = container_of(led_dev,struct dw791x_priv,cdev);

	return count;
}

static struct device_attribute dw791x_vibrator_attrs[] = {
	__ATTR(activate, 0664, activate_show, activate_store),
	__ATTR(duration, 0664, duration_show, duration_store),
	__ATTR(state, 0664, state_show, state_store),
	__ATTR(rtp_input, 0664, rtp_input_show, rtp_input_store),
	__ATTR(mode, 0664, mode_show, mode_store),
	__ATTR(scale, 0664, scale_show, scale_store),
	__ATTR(ctrl_loop, 0664, ctrl_loop_show, ctrl_loop_store),
	__ATTR(set_sequencer, 0664, NULL, set_sequencer_store),
	__ATTR(lp_trigger_effect, 0664, lp_trigger_effect_show, lp_trigger_effect_store),
};

static int dw791x_vibrator_sysfs_init(struct dw791x_priv *dw791x)
{
	int ret = 0;
	int i;

	dw791x->cdev.name = "vibrator";
	dw791x->cdev.brightness_get = dw791x_brightness_get;
	dw791x->cdev.brightness_set = dw791x_brightness_set;
	dw791x->cdev.max_brightness = 100;

	ret = led_classdev_register(dw791x->dev, &dw791x->cdev);

	if (ret < 0)
	{
		dw_err("%s devm_led_classdev_register failed\n", __func__);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(dw791x_haptics_attrs); i++)
	{
		ret = sysfs_create_file(&dw791x->dev->kobj, &dw791x_haptics_attrs[i].attr);
		if (ret < 0)
		{
			dw_err("haptics sysfs create failed\n");
			return -1;
		}
	}
	/*Link /sys/devices/i2c-N/N-address -> /sys/dongwoon_haptic_drv */
	ret = sysfs_create_link(NULL, &dw791x->dev->kobj, "dongwoon_haptic_drv");

	if (dw791x->dw791x_kobj == NULL)
	{
		dw_err("%s:subsystem_register_failed\n", __func__);
	}

	for (i = 0; i < ARRAY_SIZE(dw791x_vibrator_attrs); i++)
	{
		ret = sysfs_create_file(&dw791x->cdev.dev->kobj, &dw791x_vibrator_attrs[i].attr);
		if (ret < 0)
		{
			dw_err("vibrator sysfs create failed\n");
			return -1;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id dw791x_i2c_dt_ids[] = {
	{.compatible = "dwanatech,dw791x"},
	{}};
#endif

static const struct i2c_device_id dw791x_drv_id[] = {
	{"dw7914", 0},
	{}};
MODULE_DEVICE_TABLE(i2c, dw791x_drv_id);

#ifdef DW791X_INT
static void dw791x_intz_work(struct work_struct *work)
{
	struct dw791x_priv *dw791x = container_of(work, struct dw791x_priv, intz_en_work);
	int ret = 0;

	ret = dw791x_status_get(dw791x);
	dw_debug("dw791x_intz_work %d\n", ret);

	return;
}

static irqreturn_t intz_handler(int irq, void *device_id)
{
	struct dw791x_priv *dw791x = (struct dw791x_priv *)device_id;

	queue_work(dw791x->intz_en_workq, &dw791x->intz_en_work);

	return IRQ_HANDLED;
}

static int irq_rtp_exit(struct dw791x_priv *dw791x)
{
	free_irq(dw791x->intz_en_irq_num, NULL);
	cancel_work_sync(&dw791x->intz_en_work);
	destroy_workqueue(dw791x->intz_en_workq);

	return 0;
}

#endif
#ifdef CONFIG_OF
static int dw791x_parse_dt(struct device *dev, struct dw791x_priv *dw791x)
{
	struct device_node *np = dev->of_node;
	unsigned int value;
	int ret = 0;

	if (!np)
		return -1;

#if 0 // if use enable pin, plz uncomment
	dw791x->dw791x_en = of_get_named_gpio(np, "dw791x,dw791x-en", 0);
	if(!gpio_is_valid(dw791x->dw791x_en)) 
	{
		dw_err("dw791x_en(%u) is invalid\n", dw791x->dw791x_en);
	}
	else
	{
		ret = gpio_request(dw791x->dw791x_en, "dw791x,dw791x-en");
		if (ret < 0) {
			dw_err("dw791x_en(%u) is invalid\n", dw791x->dw791x_en);
			return ret;
		}
		gpio_direction_output(dw791x->dw791x_en, 1);  // dw791x enable.
	}
#endif
#ifdef DW791X_INT
	dw791x->intz_en = of_get_named_gpio(np, "dw791x,intz_en", 0);
	if (!gpio_is_valid(dw791x->intz_en))
	{
		dw_err("trigger1_gpio(%u) is invalid\n", dw791x->intz_en);
	}
	else
	{
		ret = gpio_request(dw791x->intz_en, "intz_en");
		if (ret < 0)
		{
			dw_err("intz_en(%u) is invalid\n", dw791x->intz_en);
			return ret;
		}
		ret = gpio_direction_input(dw791x->intz_en);

		if (ret < 0)
		{
			dw_err("Can't set GPIO direction, error %i\n", ret);
			gpio_free(dw791x->intz_en);
			return -EINVAL;
		}
		else
			dw_err("GPIO direction input: %d\n", dw791x->intz_en);

		dw791x->intz_en_irq_num = gpio_to_irq(dw791x->intz_en);

		dw_err("IRQ Line: %d\n", dw791x->intz_en_irq_num);

		ret = request_irq(dw791x->intz_en_irq_num, (irq_handler_t)intz_handler, IRQF_TRIGGER_FALLING, "intz_en_irq", NULL);

		if (ret < 0)
		{
			irq_rtp_exit(dw791x);
			dw_err("IRQ requset error: %d\n", ret);
		}

		dw791x->intz_en_workq = create_singlethread_workqueue("intz_en_wrokqueue");
		if (!dw791x->intz_en_workq)
		{
			dw_err("error when creating intz_en_wrokqueue\n");
			return -1;
		}
		INIT_WORK(&dw791x->intz_en_work, dw791x_intz_work);
	}
#endif

	dw791x->trigger1_gpio = of_get_named_gpio(np, "dw791x,trigger1-gpio", 0);
	if (!gpio_is_valid(dw791x->trigger1_gpio))
	{
		dw_err("trigger1_gpio(%u) is invalid\n", dw791x->trigger1_gpio);
	}
	else
	{
		ret = gpio_request(dw791x->trigger1_gpio, "trigger1-gpio");
		if (ret < 0)
		{
			dw_err("trigger1_gpio(%u) is invalid\n", dw791x->trigger1_gpio);
			return ret;
		}

		gpio_direction_output(dw791x->trigger1_gpio, 0);
	}
#if 0 // if use trigger 2 and trigger 3, plz uncomment

	dw791x->trigger2_gpio = of_get_named_gpio(np, "dw791x,trigger2-gpio", 0);
	if(!gpio_is_valid(dw791x->trigger2_gpio))
	{
		dw_err("trigger2_gpio(%u) is invalid\n", dw791x->trigger2_gpio);
	}
	else
	{
		ret = gpio_request(dw791x->trigger2_gpio, "trigger2-gpio");
		if (ret < 0) {
			dw_err("trigger2_gpio(%u) is invalid\n", dw791x->trigger2_gpio);
			return ret;
		}
		gpio_direction_output(dw791x->trigger2_gpio, 0);
	}

	dw791x->trigger3_gpio = of_get_named_gpio(np, "dw791x,trigger3-gpio", 0);
	if(!gpio_is_valid(dw791x->trigger3_gpio)) 
	{
		dw_err("trigger3_gpio(%u) is invalid\n", dw791x->trigger3_gpio);
	}
	else
	{
		ret = gpio_request(dw791x->trigger3_gpio, "trigger3-gpio");
		if (ret < 0) {
			dw_err("trigger3_gpio(%u) is invalid\n", dw791x->trigger3_gpio);
			return ret;
		}
		gpio_direction_output(dw791x->trigger3_gpio, 0);
	}
#endif
	if (!of_property_read_u32(np, "dw791x,vdclamp", &value))
	{
		dw791x->dw791x_haptics.vdclamp = value;
	}
	else
	{
		dw791x->dw791x_haptics.vdclamp = 0;
		dw_err("vdclamp is not set.\n");
	}

	return 0;
}
#else
static int dw791x_parse_dt(struct device *dev, struct dw791x_priv *dw791x)
{
	return NULL;
}
#endif

static int dw791x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct dw791x_priv *dw791x;
	int err = 0;

	dw_err("enter\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dw_err("I2C check failed\n");
		return -ENODEV;
	}

	dw791x = devm_kzalloc(&client->dev, sizeof(struct dw791x_priv), GFP_KERNEL);
	if (!dw791x)
	{
		dw_err("no memory\n");
		return -ENOMEM;
	}

	g_dw791x = dw791x;
	dw791x->dev = &client->dev;
	dw791x->dwclient = client;
	i2c_set_clientdata(client, dw791x);
	dev_set_drvdata(&client->dev, dw791x);

	if (client->dev.of_node)
	{
		dw_debug("of node parse\n");
		err = dw791x_parse_dt(&client->dev, dw791x);
		if (err)
		{
			dw_err("of node parse error\n");
			return -ENODEV;
		}
	}

	err = dw791x_device_init(dw791x);
	if (err)
	{
		dw_err("device init error\n");
		//		goto drv2624_i2c_probe_err;
	}

	err = dw791x_vibrator_sysfs_init(dw791x);

	dw791x->haptic_wave_work = create_singlethread_workqueue("haptic_work_queue");
	if (!dw791x->haptic_wave_work)
	{
		dw_err("error when creating haptic_work_queue\n");
		return -1;
	}
	INIT_WORK(&dw791x->wave_work, haptic_play_work);

	dw_err("device init success\n");
	return 0;
	//drv2624_i2c_probe_err:
	//	dw_err("dw791x probe failed. err = %d\n",err);
	//	return err;
}

static int dw791x_remove(struct i2c_client *client)
{
	struct dw791x_priv *dw791x = i2c_get_clientdata(client);
#ifdef DW791X_INT
	irq_rtp_exit(dw791x);
#endif
	cancel_work_sync(&dw791x->wave_work);
	destroy_workqueue(dw791x->haptic_wave_work);

	i2c_set_clientdata(client, NULL);
	mutex_destroy(&dw791x->dev_lock);

	kfree(dw791x);

	return 0;
}

#ifdef CONFIG_PM
static int dw791x_suspend(struct device *dev)
{
	return 0;
}

static int dw791x_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(dw791x_pm_ops, dw791x_suspend, dw791x_resume);

#define DW7912_VIBRATOR_PM_OPS (&dw791x_pm_ops)
#else
#define DW7912_VIBRATOR_PM_OPS NULL
#endif

static struct i2c_driver dw791x_i2c_driver = {
	.probe = dw791x_probe,
	.remove = dw791x_remove,
	.id_table = dw791x_drv_id,
	.driver = {
		.name = DW791X_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw791x_i2c_dt_ids),
#endif
#ifdef CONFIG_PM
		.pm = DW7912_VIBRATOR_PM_OPS,
#endif
	},
};

module_i2c_driver(dw791x_i2c_driver);

MODULE_DESCRIPTION("Vibrator DW791x haptic driver");
MODULE_AUTHOR("jks8051@dwanatech.com");
MODULE_LICENSE("GPL/BSD Dual");
