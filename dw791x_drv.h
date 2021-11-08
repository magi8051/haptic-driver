#ifndef __DW791X_DRV_H__
#define __DW791X_DRV_H__

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

#define DW791X_DRIVER_NAME "dw791x-haptic"

#define dw_debug(fmt, args...)                                   \
	do                                                           \
	{                                                            \
		if (unlikely(dw791x_debug_mask))                         \
			pr_info("[dw791x] %s:" fmt, __FUNCTION__, ####args); \
	} while (0)

#define dw_err(fmt, args...)                                \
	do                                                      \
	{                                                       \
		pr_err("[dw791x] %s:" fmt, __FUNCTION__, ####args); \
	} while (0)

#define BST_OPT
#define TRIG_OPT
#define DW791X_INT // if not use intz pin, plz comment

#define DW7912_VERSION 0x7912F001
#define DW7914_VERSION 0x7914F002

/* Haptic Motor Type */
#define DW791X_GAMMA
//#define DW791X_COIN0832
//#define DW791X_COIN1040

/*----------- Common Register ------------*/
#define DW7912 0x32
#define DW7914 0x40
#define CHIP_ID 0x59
#define DW791x_PID 0x00
#define DW791x_STATUS 0x01

#define SRAM_BASE 0x4000
#define BIN_BASE 0x3000
#define RAM_ADDR8 0
#define RAM_ADDR16 16
#define I2C_TRANS_RTP 1024
#define I2C_TRANS_MEM 1024
#define FIFO_2KB 2
#define FIFO_4KB 4
#define FIFO_8KB 8

/*----------- Common Bit Register ---------*/
#define BITSET 1
#define BITCLR 0
#define RTP 0
#define MEM 1
#define BRAKE 4
#define PWM_24K 0
#define PWM_48K 1
#define PWM_96K 2
#define PWM_12K 3
#define BST_ADAPT 8
#define BST_LUMP 4
#define BST_BYPASS 2

/*----------- DW7912 Register ------------*/
#define DW7912_INTEN 0x02
#define DW7912_MODE 0x03
#define DW7912_PWM 0x04
#define DW7912_BST_OUTPUT 0x05
#define DW7912_BST_MODE 0x06
#define DW7912_VD_CLAMP 0x08
#define DW7912_PLAYBACK 0x09
#define DW7912_RTP_INPUT 0x0A
#define DW7912_MEM_GAIN 0x0B
#define DW7912_WAVQ0 0x0C
#define DW7912_WAVQ1 0x0D
#define DW7912_WAVQ2 0x0E
#define DW7912_WAVQ3 0x0F
#define DW7912_WAVQ4 0x10
#define DW7912_WAVQ5 0x11
#define DW7912_WAVQ6 0x12
#define DW7912_WAVQ7 0x13
#define DW7912_MEM_LOOP 0x18
#define DW7912_TRIG_RWAV 0x19
#define DW7912_TRIG_FWAV 0x1A
#define DW7912_RAM_ADDR_H 0x1B
#define DW7912_RAM_ADDR_L 0x1C
#define DW7912_RAM_DATA 0x1D
#define DW7912_FIFO_ADDRH 0x1E
#define DW7912_FIFO_ADDRL 0x1F
#define DW7912_BST_OPTION 0x23
#define DW7912_SW_RESET 0x2F

/*----------- DW7914 Register ------------*/
#define DW7914_STATUS0 0x01
#define DW7914_INTEN 0x02
#define DW7914_TRIG_CTRL 0x03
#define DW7914_PWM 0x04
#define DW7914_BST_OUTPUT 0x05
#define DW7914_BST_OPTION 0x06
#define DW7914_BST_MODE 0x08
#define DW7914_VD_CLAMP 0x0A
#define DW7914_MODE 0x0B
#define DW7914_PLAYBACK 0x0C
#define DW7914_RTP_INPUT 0x0D
#define DW7914_MEM_GAIN 0x0E
#define DW7914_WAVQ0 0x0F
#define DW7914_WAVQ1 0x10
#define DW7914_WAVQ2 0x11
#define DW7914_WAVQ3 0x12
#define DW7914_WAVQ4 0x13
#define DW7914_WAVQ5 0x14
#define DW7914_WAVQ6 0x15
#define DW7914_WAVQ7 0x16
#define DW7914_WAV_SEQ_L0 0x17
#define DW7914_WAV_SEQ_L1 0x18
#define DW7914_WAV_SEQ_L2 0x19
#define DW7914_WAV_SEQ_L3 0x1A
#define DW7914_MEM_LOOP 0x1B
#define DW7914_BRAKE0_MWAVE 0x1C
#define DW7914_BRAKE1_MWAVE 0x1D
#define DW7914_BRAKE_MCTRL 0x1E
#define DW7914_TRIG1_R_WAVE 0x1F
#define DW7914_BRAKE0_R1_WAVE 0x20
#define DW7914_BRAKE1_R1_WAVE 0x21
#define DW7914_TRIG1_F_WAVE 0x22
#define DW7914_BRAKE0_F1_WAVE 0x23
#define DW7914_BRAKE1_F1_WAVE 0x24
#define DW7914_BRAKE_T1_CTRL 0x25
#define DW7914_TRIG2_R_WAVE 0x26
#define DW7914_BRAKE0_R2_WAVE 0x27
#define DW7914_BRAKE1_R2_WAVE 0x28
#define DW7914_TRIG2_F_WAVE 0x29
#define DW7914_BRAKE0_F2_WAVE 0x2A
#define DW7914_BRAKE1_F2_WAVE 0x2B
#define DW7914_BRAKE_T2_CTRL 0x2C
#define DW7914_TRIG3_R_WAVE 0x2D
#define DW7914_BRAKE0_R3_WAVE 0x2E
#define DW7914_BRAKE1_R3_WAVE 0x2F
#define DW7914_TRIG3_F_WAVE 0x30
#define DW7914_BRAKE0_F3_WAVE 0x31
#define DW7914_BRAKE1_F3_WAVE 0x32
#define DW7914_BRAKE_T3_CTRL 0x33
#define DW7914_TRACK_CTRL0 0x34
#define DW7914_TRACK_CTRL1 0x35
#define DW7914_TRACK_CTRL2 0x36
#define DW7914_TRACK0_WAVE 0x37
#define DW7914_TRACK1_WAVE 0x38
#define DW7914_BRAKE0_TWAVE 0x39
#define DW7914_BRAKE1_TWAVE 0x3A
#define DW7914_BRAKE_AT_CTRL 0x3B
#define DW7914_ZXD_CTRL1 0x3C
#define DW7914_ZXD_CTRL2 0x3D
#define DW7914_LRA_F0_CAL 0x3E
#define DW7914_LRA_F0_INH 0x3F
#define DW7914_LRA_F0_INL 0x40
#define DW7914_LRA_F0_OS 0x41
#define DW7914_LRA_F0_MH 0x42
#define DW7914_LRA_F0_ML 0x43
#define DW7914_STATUS1 0x44
#define DW7914_TRIG_DET_EN 0x45
#define DW7914_RAM_ADDR_H 0x46
#define DW7914_RAM_ADDR_L 0x47
#define DW7914_RAM_DATA 0x48
#define DW7914_FIFO_ADDRH 0x49
#define DW7914_FIFO_ADDRL 0x4A
#define DW7914_FIFO_LV_H 0x4B
#define DW7914_FIFO_LV_L 0x4C
#define DW7914_FIFO_STATUSH 0x4D
#define DW7914_FIFO_STATUSL 0x4E
#define DW7914_RAM_CHKSUM3 0x4F
#define DW7914_RAM_CHKSUM2 0x50
#define DW7914_RAM_CHKSUM1 0x51
#define DW7914_RAM_CHKSUM0 0x52
#define DW7914_SW_RESET 0x5F

/* DW7914 MODE Register */
#define D14_FIFO_FLUSH 0x20
#define D14_CHKSUM 0x10
#define D14_AUTO_TRACK 0x08
#define D14_AUTO_BRAKE 0x04
#define D14_DIGA_MODE 0x02
#define D14_MEM_MODE 0x01

/* DW7914 TRIG CTRL Register */
#define D14_SOUND_MODE 0x80
#define D14_TRIG2_MASTER 0x40
#define D14_TRIG1_MODE 0x20
#define D14_TRIG1_LEVEL 0x10
#define D14_TRIG_PRIORITY 0x01

/* DW7914 TRIGGER enable bit */
#define D14_TRIG_RST (1)
#define D14_TRIG1_R_EN (1 << 2)
#define D14_TRIG1_F_EN (1 << 3)
#define D14_TRIG2_R_EN (1 << 4)
#define D14_TRIG2_F_EN (1 << 5)
#define D14_TRIG3_R_EN (1 << 6)
#define D14_TRIG3_F_EN (1 << 7)

/* DW7914 ATUO TRACK_BRAKE Register */
#define D14_TRACK_PATTERN 0x01

enum duration_status
{
	DURATION_4MS = 4,
	DURATION_10MS = 10,
	DURATION_12MS = 12
};

enum dw791x_on
{
	DRV_STOP = 0,
	DRV_PLAY
};

enum dw791x_playmode
{
	HAPTIC_STOP = 0,
	HAPTIC_TICK,
	HAPTIC_SHORT_PRESS,
	HAPTIC_LONG_PRESS,
	HAPTIC_RTP_TEST,
	HAPTIC_WAVE_PLAY,
	HAPTIC_TEST
};

enum dw791x_freq_band
{
	LOW_FREQUENCY,
	MIDDLE_FREQUENCY,
	HIGH_FREQUENCY,
	MAX_FREQUENCY
};

#ifdef DW791X_GAMMA
typedef enum
{
	WAVEFORM_ID_NONE = 0,
	WAVEFORM_ID_GMA_MEM1, // TICK event
	WAVEFORM_ID_GMA_MEM2, // SHORT PRESS(CLICK) event
	WAVEFORM_ID_GMA_MEM3, // LONG PRESS event
	WAVEFORM_ID_GMA_MEM4,
	WAVEFORM_ID_GMA_MEM5,
	WAVEFORM_ID_GMA_MEM6,
	WAVEFORM_ID_GMA_MEM7,
	WAVEFORM_ID_SINE165,
	WAVEFORM_ID_SINE145,
	WAVEFORM_ID_SINE185,
	WAVEFORM_ID_AUTO_BRAKE165,
	WAVEFORM_ID_MAX
} dw791x_waveform_id;
#endif

#ifdef DW791X_COIN0832
typedef enum
{
	WAVEFORM_ID_NONE = 0,
	WAVEFORM_ID_COIN0832_MEM1, // TICK event
	WAVEFORM_ID_COIN0832_MEM2, // SHORT PRESS(CLICK) event
	WAVEFORM_ID_COIN0832_MEM3, // LONG PRESS event
	WAVEFORM_ID_COIN0832_MEM4,
	WAVEFORM_ID_COIN0832_MEM5,
	WAVEFORM_ID_MAX
} dw791x_waveform_id;
#endif

#ifdef DW791X_COIN1040
typedef enum
{
	WAVEFORM_ID_NONE = 0,
	WAVEFORM_ID_COIN1040_MEM1, // TICK event
	WAVEFORM_ID_COIN1040_MEM2, // SHORT PRESS(CLICK) event
	WAVEFORM_ID_COIN1040_MEM3, // LONG PRESS event
	WAVEFORM_ID_COIN1040_MEM4,
	WAVEFORM_ID_MAX
} dw791x_waveform_id;
#endif

typedef struct
{
	dw791x_waveform_id dw_wave_id;
	const u8 *wave_data;
	ssize_t size;
} dw791x_waveform_data;

struct autovib_reg
{
	u8 track_ctrl0;
	u8 track_ctrl1;
	u8 track_ctrl2;
	u8 track0_wave;
	u8 track1_wave;
	u8 brake0_twave;
	u8 brake1_twave;
	u8 brake_at_ctrl;
};

struct dw791x_data
{
	int activate;
	int duration;
	int state;
	int rtp_input_node;
	int mode;
	int set_sequencer;
	int scale;
	int ctrl_loop;
	int lp_trigger_effect;
	int a2v_vib_lock;
	int irq_gpio;
	int vdclamp;
	int auto_brake;
	int mem_vdclamp;

	u8 status0;
	u8 status1;
};

struct dw791x_reg
{
	u8 rtp_input;
	u8 mem_input;
	u8 play_back;
	u8 fifo_level;
	u8 fifo_addrh;
	u8 fifo_addrl;
	u8 play_mode;
};

struct dw791x_priv
{
	struct device *dev;
	struct led_classdev cdev;
	struct i2c_client *dwclient;
	struct mutex dev_lock;
	struct autovib_reg autovib;
	struct dw791x_data dw791x_haptics;
	struct dw791x_reg dw791x_reg_addr;
	struct kobject *dw791x_kobj;

	struct work_struct wave_work;
	struct workqueue_struct *haptic_wave_work;
#ifdef DW791X_INT
	struct work_struct intz_en_work;
	struct workqueue_struct *intz_en_workq;
#endif
	const struct firmware *fw;

	int dw791x_en;
	int intz_en;
	int intz_en_irq_num;
	int trigger1_gpio;
	int trigger2_gpio;
	int trigger3_gpio;
	int haptic_wave_on;
	u8 reg_read;
	u32 device;
	u32 checksum;
	u32 version;
	u8 play_mode;
	int jstest;
	s8 dev_name[8];
};

static struct dw791x_priv *g_dw791x = NULL;

static int dw791x_byte_write(struct dw791x_priv *dw791x, u8 addr, u8 data);
static int dw791x_word_write(struct dw791x_priv *dw791x, u8 addr, u32 data);
static int dw791x_byte_read(struct dw791x_priv *dw791x, u8 addr);
static int dw791x_Mem_Write(struct dw791x_priv *dw791x, u8 *data, u32 size);
static void dw791x_mem_vd_clamp_set(struct dw791x_priv *dw791x, dw791x_waveform_id wave_id, u32 vol);
static int dw791x_play_mode_sel(struct dw791x_priv *dw791x, int play_mode);
static void dw791x_fifo_size_set(struct dw791x_priv *dw791x, u32 size);
static void dw7914_trigctrl_set(struct dw791x_priv *dw791x, u8 type, u8 bitset);
static int dw791x_seq_write(struct dw791x_priv *dw791x, u32 addr, u32 ram_addr, u32 ram_bit, u8 *data, u32 size);
static void dw7914_autovib_set(struct dw791x_priv *dw791x);
static int dw7914_checksum(struct dw791x_priv *dw791x, u32 type, u32 page);
static int transfer_atuovib_wave(struct dw791x_priv *dw791x, u8 *wave);
static int request_transfer_rtp_wave(struct dw791x_priv *dw791x, u8 *wave, u32 repeat);
static int request_transfer_mem_wave(struct dw791x_priv *dw791x, u32 page, u32 point, u8 *fw_data, u32 size);
#endif
