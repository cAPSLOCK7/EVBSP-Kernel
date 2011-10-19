/*
 * File Name		: sound/arm/emxx-mixer.h
 * Function		: AK4648 CODEC
 * Release Version 	: Ver 1.02
 * Release Date		: 2010/10/15
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#include <linux/version.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <mach/pwc.h>
#include <mach/smu.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "emxx-mixer.h"
#include "emxx-pcm.h"

/* static initial value */
#define ID_VALUE		NULL
#define I2C_CODEC_CLIENT_VALUE	NULL;

#ifdef AUDIO_MAKING_DEBUG
#include <linux/moduleparam.h>
#endif

/*** debug code by the making ->*/
#ifdef AUDIO_MAKING_DEBUG
#define FNC_ENTRY	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "entry:%s\n", __FUNCTION__); \
	}
#define FNC_EXIT	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "exit:%s:%d\n", __FUNCTION__ , __LINE__); \
	}
#define d0b(fmt, args...)	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		## args);
#define d1b(fmt, args...)	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		## args); \
	}
#define d7b(fmt, args...)	\
	if (debug == 7 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		## args); \
	}
#define d8b(fmt, args...)	\
	if (debug == 8 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , \
		## args); \
	}
#define d9b(fmt, args...)	\
	if (debug == 9 || debug >= 9) {	\
		printk(KERN_INFO fmt , ## args); \
	}
#else
#define FNC_ENTRY
#define FNC_EXIT
#define d0b(fmt, args...)
#define d1b(fmt, args...)
#define d7b(fmt, args...)
#define d8b(fmt, args...)
#define d9b(fmt, args...)
#endif
/*<- debug code by the making ***/

#include <linux/delay.h>

#define CODEC_WRITE(a, d, m)    i2c_codec_write(a, d, m)

#define TYPE_VOL        0x01
#define TYPE_SW         0x02
#define TYPE_BL         0x04

#define IDX(a)  (a & 0xffff)

/* VOL(integer) */
enum {
	MIXER_VOL_PLAYBACK = TYPE_VOL << 16 | 0x0000,
	MIXER_VOL_CAPTURE,

	MIXER_INT_NUM = IDX(MIXER_VOL_CAPTURE) + 1,
};

/* SW (enum) */
enum {
	MIXER_SW_CAPTURE_SOURCE = TYPE_SW << 16 | 0x0000,
	MIXER_SW_CAPTURE_CHANNEL_MODE,
	MIXER_SW_SAMPLING_RATE,
	MIXER_SW_PLAYBACK,

	MIXER_ENUM_NUM = IDX(MIXER_SW_PLAYBACK) + 1,
};

/* SW (boolean) */
enum {
	MIXER_SW_CODEC_POWER_BL = TYPE_BL << 16 | 0x0000,

	MIXER_BL_NUM = IDX(MIXER_SW_CODEC_POWER_BL) + 1,
};

static const char *in_sw_control_texts[] = {
	"MIC", "Line"
};

static const char *pmad_sw_control_texts[] = {
	"Stereo", "Mono Lch", "Mono Rch", "MIX"
};

static const char *fs_sw_control_texts[] = {
	"7.35kHz", "8kHz", "11.025kHz", "12kHz", "14.7kHz", "16kHz",
	"22.05kHz", "24kHz", "29.4kHz", "32kHz", "44.1kHz", "48kHz"
};

static const char *out_sw_control_texts[] = {
	"Line", "Headphone"
};

struct integer_info {
	unsigned int count;     /* count of values */
	long val_int_min;       /* R: minimum value */
	long val_int_max;       /* R: maximum value */
	long val_int_step;      /* R: step (0 variable) */
	int value[2];
};

struct enum_info {
	char **texts;
	unsigned int items;
	int value;
};

struct boolean_info {
	int value;
};

struct emxx_codec_mixer {
	struct snd_card *card;
	spinlock_t mixer_lock;
	struct integer_info *vol_info;
	struct enum_info *enum_info;
	struct boolean_info *bl_info;
	int power_on;
	struct mutex power_mutex;
};

static struct integer_info volume_info[MIXER_INT_NUM] = {
	{ /* MIXER_VOL_PLAYBACK */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 100,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_CAPTURE */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 100,
		.val_int_step = 1,
	},
};

#define NUM_OF(v) (sizeof(v) / sizeof(v[0]))
static struct enum_info enum_info[MIXER_ENUM_NUM] = {
	{ /* MIXER_SW_CAPTURE_SOURCE */
		.texts = (char **)in_sw_control_texts,
		.items = NUM_OF(in_sw_control_texts),
	},
	{ /* MIXER_SW_CAPTURE_CHANNEL_MODE */
		.texts = (char **)pmad_sw_control_texts,
		.items = NUM_OF(pmad_sw_control_texts),
	},
	{ /* MIXER_SW_SAMPLING_RATE */
		.texts = (char **)fs_sw_control_texts,
		.items = NUM_OF(fs_sw_control_texts),
	},
	{ /* MIXER_SW_PLAYBACK */
		.texts = (char **)out_sw_control_texts,
		.items = NUM_OF(out_sw_control_texts),
	},
};

static struct boolean_info boolean_info[MIXER_BL_NUM];

static struct emxx_codec_mixer codec_mixer = {
	.vol_info = &volume_info[0],
	.enum_info = &enum_info[0],
	.bl_info = &boolean_info[0],
	.power_on = 0, /* off */
};

static pcm_ctrl_t pcm_sett = {
	.func = {
		.mode_sel       = PCM_MODE_3,
		.m_s            = PCM_SLAVE_MODE,
		.tx_tim         = PCM_TX_04_WORD,
		},
	.cyc = {
		.cyc_val        = 0x1f,
		.sib            = 0x0f,
		.rx_pd          = PCM_PADDING_ON,
		.sob            = 0x0f,
		.tx_pd          = PCM_PADDING_ON,
		},
	.cyc2 = {
		.cyc_val2       = 0x1f,
		.sib2           = 0x1f,
		.sob2           = 0x1f,
		},
};


/* Local prototypes */
#ifdef CODEC_TEST_DEBUG
static int i2c_codec_read(unsigned char reg, unsigned char *data);
#endif /* CODEC_TEST_DEBUG */
static unsigned char i2c_codec_reg[AK4648REG_MAX];
static int codec_i2c_probe(struct i2c_client *client,
 const struct i2c_device_id *id);
static int codec_i2c_remove(struct i2c_client *client);

static struct i2c_device_id codec_i2c_idtable[] = {
	{ I2C_SLAVE_CODEC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, codec_i2c_idtable);

static struct i2c_driver i2c_codec_driver = {
	.driver.name    = "i2c for codec",
	.id             = I2C_DRIVERID_I2CDEV, /* Fake ID */
	.id_table       = codec_i2c_idtable,
	.probe          = codec_i2c_probe,
	.remove         = codec_i2c_remove,
};
static struct i2c_client *i2c_codec_client = I2C_CODEC_CLIENT_VALUE;


/*
 * i2c functions
 */
static int codec_i2c_probe(struct i2c_client *client,
 const struct i2c_device_id *id)
{
	i2c_codec_client = client;
	return 0;
}

static int codec_i2c_remove(struct i2c_client *client)
{
	i2c_codec_client = NULL;
	return 0;
}

static int i2c_codec_cleanup(void)
{
	i2c_del_driver(&i2c_codec_driver);
	return 0;
}

static int i2c_codec_inserted(void)
{
	return i2c_add_driver(&i2c_codec_driver);
}

static int i2c_codec_init(void)
{
	int res = 0;
	unsigned char reg = 0;

	res = i2c_codec_inserted();
	if (res == 0) {
		if (i2c_codec_client == NULL) {
			i2c_codec_cleanup();
			printk(KERN_ERR "codec_i2c_found_proc() not called!\n");
			return -EIO;
		}
		res = i2c_master_send(i2c_codec_client, &reg, 1);
		if (res > 0) {
			res =
				i2c_master_recv(i2c_codec_client, i2c_codec_reg,
						AK4648REG_MAX);
			if (res > 0)
				res = 0;

		}
		if (res != 0) {
			i2c_codec_cleanup();
			printk(KERN_ERR "send or recv failed!\n");
		}
	} else {
		printk(KERN_ERR "i2c codec inserted failed!\n");
	}

	return res;
}

static int i2c_codec_write(unsigned char reg, unsigned char data,
			   unsigned char mask)
{
	int res = 0;
	unsigned char buf[2];

	if ((AK4648REG_MAX <= reg) || ((data & ~(mask)) != 0))
		return -EINVAL;

	data = (i2c_codec_reg[reg] & ~(mask)) | (data & mask);

	i2c_codec_reg[reg] = data;

	buf[0] = reg;
	buf[1] = data;

	res = i2c_master_send(i2c_codec_client, buf, 2);
	if (res > 0)
		res = 0;
	else
		printk(KERN_ERR "i2c codec write failed!\n");

	return res;
}

#ifdef CODEC_TEST_DEBUG
static int i2c_codec_read(unsigned char reg, unsigned char *data)
{
	int res = 0;
	unsigned char buf = 0;

	if (i2c_codec_client == NULL) {
		printk(KERN_ERR "i2c codec not available!\n");
		return -EIO;
	}

	if (AK4648REG_MAX <= reg)
		return -EINVAL;

	res = i2c_master_send(i2c_codec_client, &buf, 1);
	if (res <= 0) {
		printk(KERN_ERR "i2c codec send failed!\n");
		return res;
	}

	res = i2c_master_recv(i2c_codec_client, i2c_codec_reg, AK4648REG_MAX);
	if (res > 0) {
		*data = i2c_codec_reg[reg];
		res = 0;
	} else
		printk(KERN_ERR "i2c codec recv failed!\n");

	return res;
}
EXPORT_SYMBOL(i2c_codec_read);
#endif /* CODEC_TEST_DEBUG */

static int codec_init(void)
{
	struct emxx_codec_mixer *codec = &codec_mixer;
	int i, j;
	FNC_ENTRY

	/* volume(integer) */
	for (i = 0; i < MIXER_INT_NUM; i++) {
		for (j = 0; j < (codec->vol_info[IDX(i)].count); j++) {
			/* 80 */
			codec->vol_info[IDX(i)].value[j] = 80;
		}
	}

	/* switch control(enum) */
	/* MIC */
	codec->enum_info[IDX(MIXER_SW_CAPTURE_SOURCE)].value = 0;
	/* Stereo */
	codec->enum_info[IDX(MIXER_SW_CAPTURE_CHANNEL_MODE)].value = 0;
	/* 8kHz */
	codec->enum_info[IDX(MIXER_SW_SAMPLING_RATE)].value = 1;
	/* Line */
	codec->enum_info[IDX(MIXER_SW_PLAYBACK)].value = 0;

	/* switch control(boolean) */
	/* "OFF" */
	codec->bl_info[IDX(MIXER_SW_CODEC_POWER_BL)].value = 0;

	FNC_EXIT return 0;
}

#ifdef CONFIG_EMXX_ANDROID
int codec_power_on(void)
#else
static int codec_power_on(void)
#endif
{
	struct emxx_codec_mixer *codec = &codec_mixer;
	int res = 0;
	int power_value = 0;

	power_value = codec->bl_info[IDX(MIXER_SW_CODEC_POWER_BL)].value;
	codec_init();
	codec->bl_info[IDX(MIXER_SW_CODEC_POWER_BL)].value = power_value;

	mutex_lock(&codec->power_mutex);
	if (codec->power_on) {
		mutex_unlock(&codec->power_mutex);
		return 0;
	}

	/* codec PDN pin "L" -> "H" */
	gpio_set_value(GPIO_AUDIO_RST, 1);
	/* PDN pin "L" needs 150ns */

#ifdef CONFIG_MACH_EMEV
	outl(SMU_PLLSEL_OSC1 | SMU_DIV(2), SMU_REFCLKDIV);
#elif defined(CONFIG_MACH_EMGR)
	outl(SMU_PLLSEL_OSC0 | SMU_DIV(1), SMU_REFCLKDIV);
#endif
	emxx_open_clockgate(EMXX_CLK_REF);

	res = i2c_codec_init();
	if (res != 0)
		goto err1;

	/* Clock Set Up Sequence */
	/* M_S */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
			       AK4648BIT_M_S, AK4648BIT_M_S);
	if (res < 0)
		goto err1;

	/* PLL BCKO DIF */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_1,
#ifdef CONFIG_MACH_EMEV
	  (AK4648BIT_PLL2 | AK4648BIT_PLL1 | AK4648BIT_DIF1 | AK4648BIT_BCKO),
#elif defined(CONFIG_MACH_EMGR)
	  (AK4648BIT_PLL2 | AK4648BIT_DIF1 | AK4648BIT_BCKO),
#endif
	  (AK4648BIT_PLL_MASK | AK4648BIT_BCKO | AK4648BIT_DIF_MASK));

	if (res < 0)
		goto err1;
	/* FS */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_2,
			       AK4648BIT_FS_8KHZ, AK4648BIT_FS_MASK);
	if (res < 0)
		goto err1;
	/* PMVCM */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       AK4648BIT_PMVCM, AK4648BIT_PMVCM);
	if (res < 0)
		goto err1;
	/* PMPLL */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
			       AK4648BIT_PMPLL, AK4648BIT_PMPLL);
	if (res < 0)
		goto err1;
	schedule_timeout_uninterruptible(AK4648_WAIT_PLL_LOCK);

	/* Stereo Lineout Sequence */
	/* DACL */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_1,
			       AK4648BIT_DACL, AK4648BIT_DACL);
	if (res < 0)
		goto err1;
	/* IVL */
	res = CODEC_WRITE(AK4648REG_LCH_INPUT_VOLUME_CONTROL,
			       0x91, AK4648_INPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* IVR */
	res = CODEC_WRITE(AK4648REG_RCH_INPUT_VOLUME_CONTROL,
			       0x91, AK4648_INPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* DVL */
	res = CODEC_WRITE(AK4648REG_LCH_DIGITAL_VOLUME_CONTROL,
			       ((AK4648_OUTPUT_VOL_MAX * (100 - 80)) / 100),
			       AK4648_OUTPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* DVR */
	res = CODEC_WRITE(AK4648REG_RCH_DIGITAL_VOLUME_CONTROL,
			       ((AK4648_OUTPUT_VOL_MAX * (100 - 80)) / 100),
			       AK4648_OUTPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* DVOLC */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_3,
			       0x00, AK4648BIT_DVOLC);
	if (res < 0)
		goto err1;
	/* LOPS */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
			       AK4648BIT_LOPS, AK4648BIT_LOPS);
	if (res < 0)
		goto err1;
	/* PMLO PMDAC */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       (AK4648BIT_PMLO | AK4648BIT_PMDAC),
			       (AK4648BIT_PMLO | AK4648BIT_PMDAC));
	if (res < 0)
		goto err1;
	schedule_timeout_uninterruptible(AK4648_WAIT_LO_LOCK);
	/* LOPS */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
			       0x00, AK4648BIT_LOPS);
	if (res < 0)
		goto err1;

	/* Mic Input Recording Sequence */
	/* PMMP MGAIN0 */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_1, AK4648BIT_PMMP,
			       (AK4648BIT_PMMP | AK4648BIT_MGAIN0));
	if (res < 0)
		goto err1;
	/* IN */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
			       0x00, AK4648BIT_IN_MASK);
	if (res < 0)
		goto err1;
	/* INL */
	res = CODEC_WRITE(AK4648REG_LCH_INPUT_VOLUME_CONTROL,
			       ((AK4648_INPUT_VOL_MAX * 80) / 100),
			       AK4648_INPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* INR */
	res = CODEC_WRITE(AK4648REG_RCH_INPUT_VOLUME_CONTROL,
			       ((AK4648_INPUT_VOL_MAX * 80) / 100),
			       AK4648_INPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* IVOLC */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_4,
			       0x00, AK4648BIT_IVOLC);
	if (res < 0)
		goto err1;
	/* ZTM WTM */
	res = CODEC_WRITE(AK4648REG_TIMER_SELECT,
			       (AK4648BIT_ZTM1 | AK4648BIT_WTM1),
			       (AK4648BIT_ZTM_MASK | AK4648BIT_WTM_MASK));
	if (res < 0)
		goto err1;
	/* REF */
	res = CODEC_WRITE(AK4648REG_ALC_MODE_CONTROL_2,
			       ((AK4648_INPUT_VOL_MAX * 80) / 100),
			       AK4648_INPUT_VOL_MAX);
	if (res < 0)
		goto err1;
	/* RGAIN1 LMTH1 */
	res = CODEC_WRITE(AK4648REG_ALC_MODE_CONTROL_3, 0x00,
			       (AK4648BIT_RGAIN1 | AK4648BIT_LMTH1));
	if (res < 0)
		goto err1;
	/* ALC ZELMN RGAIN0 LMTH0 */
	res = CODEC_WRITE(AK4648REG_ALC_MODE_CONTROL_1,
			       (AK4648BIT_ALC | AK4648BIT_LMTH0),
			       (AK4648BIT_ALC | AK4648BIT_ZELMN |
				AK4648BIT_LMAT_MASK |
				AK4648BIT_RGAIN0 | AK4648BIT_LMTH0));
	if (res < 0)
		goto err1;
	/* PMADL */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       AK4648BIT_PMADL, AK4648BIT_PMADL);
	if (res < 0)
		goto err1;
	/* PMADR */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
			       AK4648BIT_PMADR, AK4648BIT_PMADR);
	if (res < 0)
		goto err1;
	/* MIX */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_5,
			       0x00, AK4648BIT_MIX);
	if (res < 0)
		goto err1;

	codec->power_on++;
	mutex_unlock(&codec->power_mutex);

	return res;

err1:
	/* codec PDN pin "H" -> "L" */
	gpio_set_value(GPIO_AUDIO_RST, 0);
	/* PDN pin "L" needs 150ns */
	udelay(1);

	mutex_unlock(&codec->power_mutex);

	return res;
}

#ifdef CONFIG_EMXX_ANDROID
int codec_power_off(void)
#else
static int codec_power_off(void)
#endif
{
	struct emxx_codec_mixer *codec = &codec_mixer;
	int res = 0;

	mutex_lock(&codec->power_mutex);
	if (codec->power_on == 0) {
		mutex_unlock(&codec->power_mutex);
		return 0;
	}

	/* Mic Input Recording Sequence */
	/* PMADR */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
			       0x00, AK4648BIT_PMADR);
	if (res < 0)
		goto out;
	/* PMADL */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       0x00, AK4648BIT_PMADL);
	if (res < 0)
		goto out;
	/* PMMP */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_1, 0x00, AK4648BIT_PMMP);
	if (res < 0)
		goto out;

	/* Headphone Sequence */
	/* HPMTN */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
			       0x00, AK4648BIT_HPMTN);
	if (res < 0)
		goto out;
	schedule_timeout_uninterruptible(AK4648_WAIT_HPMTN_DIS_LOCK);
	/* PMHPL PMHPR */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2, 0x00,
			       (AK4648BIT_PMHPL | AK4648BIT_PMHPR));
	if (res < 0)
		goto out;
	/* PMDAC */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       0x00, AK4648BIT_PMDAC);
	if (res < 0)
		goto out;
	/* DACH */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_4,
			       0x00, AK4648BIT_DACH);
	if (res < 0)
		goto out;

	/* Stereo Lineout Sequence */
	/* LOPS */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
			       AK4648BIT_LOPS, AK4648BIT_LOPS);
	if (res < 0)
		goto out;
	/* PMLO PMDAC */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       0x00, (AK4648BIT_PMLO | AK4648BIT_PMDAC));
	if (res < 0)
		goto out;
	schedule_timeout_uninterruptible(AK4648_WAIT_LO_LOCK);
	/* DACL */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_1,
			       0x00, AK4648BIT_DACL);
	if (res < 0)
		goto out;
	/* LOPS */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
			       0x00, AK4648BIT_LOPS);
	if (res < 0)
		goto out;

	/* Stop of Clock Sequence */
	/* PMPLL */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
			       0x00, AK4648BIT_PMPLL);
	if (res < 0)
		goto out;
	schedule_timeout_uninterruptible(AK4648_WAIT_PLL_LOCK);
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       0x00, AK4648BIT_PMVCM);
	if (res < 0)
		goto out;


	i2c_codec_cleanup();

	emxx_close_clockgate(EMXX_CLK_REF);

	/* codec PDN pin "H" -> "L" */
	gpio_set_value(GPIO_AUDIO_RST, 0);
	/* PDN pin "L" needs 150ns */
	udelay(1);

	codec->power_on--;
out:
	mutex_unlock(&codec->power_mutex);

	return res;
}

static inline int emxx_codec_playback_volume(int *val)
{
	unsigned char reg;
	int res = 0;

	d7b("lch:%d rch:%d\n", *(val), *(val + 1));

	if (*(val) > 100)
		*(val) = 100;

	if (*(val + 1) > 100)
		*(val + 1) = 100;

	reg = (AK4648_OUTPUT_VOL_MAX * (100 - *(val))) / 100;

	/* DVL */
	res = CODEC_WRITE(AK4648REG_LCH_DIGITAL_VOLUME_CONTROL,
			       reg, 0xff);
	if (res < 0)
		return res;

	reg = (AK4648_OUTPUT_VOL_MAX * (100 - *(val + 1))) / 100;

	/* DVR */
	res = CODEC_WRITE(AK4648REG_RCH_DIGITAL_VOLUME_CONTROL,
			       reg, 0xff);
	if (res < 0)
		return res;

	return 0;
}

static inline int emxx_codec_capture_volume(int *val)
{
	unsigned char reg1, reg2;
	int res = 0;

	d7b("lch:%d rch:%d\n", *(val), *(val + 1));

	if (*(val) > 100)
		*(val) = 100;

	if (*(val + 1) > 100)
		*(val + 1) = 100;

	reg1 = (AK4648_INPUT_VOL_MAX * *(val)) / 100;

	reg2 = (AK4648_INPUT_VOL_MAX * *(val + 1)) / 100;

	/* IVL */
	res = CODEC_WRITE(AK4648REG_LCH_INPUT_VOLUME_CONTROL,
			       reg1, 0xff);
	if (res < 0)
		return res;

	/* IVR */
	res = CODEC_WRITE(AK4648REG_RCH_INPUT_VOLUME_CONTROL,
			       reg2, 0xff);
	if (res < 0)
		return res;

	res = CODEC_WRITE(AK4648REG_ALC_MODE_CONTROL_1,
			       0x00, AK4648BIT_ALC);
	if (res < 0)
		return res;

	res = CODEC_WRITE(AK4648REG_ALC_MODE_CONTROL_2, reg1, 0xff);
	if (res < 0)
		return res;

	schedule_timeout_uninterruptible(AK4648_WAIT_ALC_LOCK);

	res = CODEC_WRITE(AK4648REG_ALC_MODE_CONTROL_1,
			       AK4648BIT_ALC, AK4648BIT_ALC);
	if (res < 0)
		return res;

	return 0;
}

static int emxx_codec_capture_source_sw(int *val)
{
	unsigned char inx = 0;
	unsigned char pmmp = 0;
	int res = 0;

	switch (val[0]) {
	case 0: /* MIC */
		inx = 0x00;
		pmmp = AK4648BIT_PMMP;
		break;
	case 1: /* Line */
		inx = AK4648BIT_INL0 | AK4648BIT_INR0;
		pmmp = AK4648BIT_PMMP;
		break;
	}

	/* INL INR */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
			       inx, AK4648BIT_IN_MASK);
	if (res < 0)
		return res;

	/* PMMP */
	res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_1,
			       pmmp, AK4648BIT_PMMP);
	if (res < 0)
		return res;

	return 0;
}

static int emxx_codec_capture_channel_mode_sw(int *val)
{
	unsigned char pmadl = 0;
	unsigned char pmadr = 0;
	unsigned char mix = 0;
	int res = 0;

	switch (val[0]) {
	case 0: /* Stereo */
		pmadl = AK4648BIT_PMADL;
		pmadr = AK4648BIT_PMADR;
		mix = 0x00;
		break;
	case 1: /* Mono Lch */
		pmadl = AK4648BIT_PMADL;
		pmadr = 0x00;
		mix = 0x00;
		break;
	case 2: /* Mono Rch */
		pmadl = 0x00;
		pmadr = AK4648BIT_PMADR;
		mix = 0x00;
		break;
	case 3: /* MIX */
		pmadl = AK4648BIT_PMADL;
		pmadr = AK4648BIT_PMADR;
		mix = AK4648BIT_MIX;
		break;
	}

	/* PMADL */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
			       pmadl, AK4648BIT_PMADL);
	if (res < 0)
		return res;
	/* PMADR */
	res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
			       pmadr, AK4648BIT_PMADR);
	if (res < 0)
		return res;
	/* MIX */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_5,
			       mix, AK4648BIT_MIX);
	if (res < 0)
		return res;

	return 0;
}

static int emxx_codec_sampling_rate_sw(long val)
{
	unsigned char fs = 0;
	int res = 0;
	struct enum_info *info = &enum_info[IDX(MIXER_SW_SAMPLING_RATE)];

	if (val < 8000) {
		/* 7.35kHz */
		fs = AK4648BIT_FS_7_35KHZ;
		info->value = 0;
	} else if (val < 11025) {
		/* 8kHz */
		fs = AK4648BIT_FS_8KHZ;
		info->value = 1;
	} else if (val < 12000) {
		/* 11.025kHz */
		fs = AK4648BIT_FS_11_025KHZ;
		info->value = 2;
	} else if (val < 14700) {
		/* 12kHz */
		fs = AK4648BIT_FS_12KHZ;
		info->value = 3;
	} else if (val < 16000) {
		/* 14.7kHz */
		fs = AK4648BIT_FS_14_7KHZ;
		info->value = 4;
	} else if (val < 22050) {
		/* 16kHz */
		fs = AK4648BIT_FS_16KHZ;
		info->value = 5;
	} else if (val < 24000) {
		/* 22.05kHz */
		fs = AK4648BIT_FS_22_05KHZ;
		info->value = 6;
	} else if (val < 29400) {
		/* 24kHz */
		fs = AK4648BIT_FS_24KHZ;
		info->value = 7;
	} else if (val < 32000) {
		/* 29.4kHz */
		fs = AK4648BIT_FS_29_4KHZ;
		info->value = 8;
	} else if (val < 44100) {
		/* 32kHz */
		fs = AK4648BIT_FS_32KHZ;
		info->value = 9;
	} else if (val < 48000) {
		/* 44.1kHz */
		fs = AK4648BIT_FS_44_1KHZ;
		info->value = 10;
	} else {
		/* 48kHz */
		fs = AK4648BIT_FS_48KHZ;
		info->value = 11;
	}

	/* FS */
	res = CODEC_WRITE(AK4648REG_MODE_CONTROL_2, fs, AK4648BIT_FS_MASK);

	return res;
}

static int emxx_codec_playback_sw(int *val)
{
	int res = 0;

	switch (val[0]) {
	case 0: /* Line */
		/* Headphone power off */
		/* HPMTN */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
				       0x00, AK4648BIT_HPMTN);
		if (res < 0)
			return res;
		schedule_timeout_uninterruptible(AK4648_WAIT_HPMTN_DIS_LOCK);
		/* PMHPL PMHPR */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2, 0x00,
				       (AK4648BIT_PMHPL | AK4648BIT_PMHPR));
		if (res < 0)
			return res;
		/* PMDAC */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
				       0x00, AK4648BIT_PMDAC);
		if (res < 0)
			return res;
		/* DACH */
		res = CODEC_WRITE(AK4648REG_MODE_CONTROL_4,
				       0x00, AK4648BIT_DACH);
		if (res < 0)
			return res;

		/* Stereo Lineout power on */
		/* DACL */
		res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_1,
				       AK4648BIT_DACL, AK4648BIT_DACL);
		if (res < 0)
			return res;
		/* LOPS */
		res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				       AK4648BIT_LOPS, AK4648BIT_LOPS);
		if (res < 0)
			return res;
		/* PMLO PMDAC */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
				       (AK4648BIT_PMLO | AK4648BIT_PMDAC),
				       (AK4648BIT_PMLO | AK4648BIT_PMDAC));
		if (res < 0)
			return res;
		schedule_timeout_uninterruptible(AK4648_WAIT_LO_LOCK);
		/* LOPS */
		res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				       0x00, AK4648BIT_LOPS);
		if (res < 0)
			return res;
		break;

	case 1: /* Headphone */
		/* Stereo Lineout power off */
		/* LOPS */
		res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				       AK4648BIT_LOPS, AK4648BIT_LOPS);
		if (res < 0)
			return res;
		/* PMLO PMDAC */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
				0x00, (AK4648BIT_PMLO | AK4648BIT_PMDAC));
		if (res < 0)
			return res;
		schedule_timeout_uninterruptible(AK4648_WAIT_LO_LOCK);
		/* LOPS */
		res = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				       0x00, AK4648BIT_LOPS);
		if (res < 0)
			return res;

		/* Headphone power on */
		/* DACH */
		res = CODEC_WRITE(AK4648REG_MODE_CONTROL_4,
				       AK4648BIT_DACH, AK4648BIT_DACH);
		if (res < 0)
			return res;
		/* HPG vol -> 0dB(defult) */

		/* PMDAC */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
				       AK4648BIT_PMDAC, AK4648BIT_PMDAC);
		if (res < 0)
			return res;
		/* PMHPL PMHPR */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
				       (AK4648BIT_PMHPL | AK4648BIT_PMHPR),
				       (AK4648BIT_PMHPL | AK4648BIT_PMHPR));
		if (res < 0)
			return res;
		/* HPMTN */
		res = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_2,
				       AK4648BIT_HPMTN, AK4648BIT_HPMTN);
		if (res < 0)
			return res;
		schedule_timeout_uninterruptible(AK4648_WAIT_HPMTN_EN_LOCK);
		break;
	}

	return 0;
}

static int emxx_codec_mixer_write(int addr, struct emxx_codec_mixer *codec)
{
	int res = 0;
	FNC_ENTRY

	switch (addr) {
	case MIXER_VOL_PLAYBACK:
		res = emxx_codec_playback_volume(
			&codec->vol_info[IDX(addr)].value[0]);
		break;
	case MIXER_VOL_CAPTURE:
		res = emxx_codec_capture_volume(
			&codec->vol_info[IDX(addr)].value[0]);
		break;
	case MIXER_SW_CAPTURE_SOURCE:
		res = emxx_codec_capture_source_sw(
			&codec->enum_info[IDX(addr)].value);
		break;
	case MIXER_SW_CAPTURE_CHANNEL_MODE:
		res = emxx_codec_capture_channel_mode_sw(
			&codec->enum_info[IDX(addr)].value);
		break;
	case MIXER_SW_SAMPLING_RATE:
	{
		long val = 0;

		switch (codec->enum_info[IDX(addr)].value) {
		case 0:
			val = 7350;
			break;
		case 1:
			val = 8000;
			break;
		case 2:
			val = 11025;
			break;
		case 3:
			val = 12000;
			break;
		case 4:
			val = 14700;
			break;
		case 5:
			val = 16000;
			break;
		case 6:
			val = 22050;
			break;
		case 7:
			val = 24000;
			break;
		case 8:
			val = 29400;
			break;
		case 9:
			val = 32000;
			break;
		case 10:
			val = 44100;
			break;
		case 11:
			val = 48000;
			break;
		}

		res = emxx_codec_sampling_rate_sw(val);

		break;
	}
	case MIXER_SW_PLAYBACK:
		res = emxx_codec_playback_sw(
			&codec->enum_info[IDX(addr)].value);
		break;
	case MIXER_SW_CODEC_POWER_BL:
		if (codec->bl_info[IDX(addr)].value)
			res = codec_power_on();
		else
			res = codec_power_off();

		break;
	}

	FNC_EXIT return res;
}

#define EMXX_CODEC_INTEGER(xname, xindex, addr) \
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
	  .info = emxx_codec_integer_info, .get = emxx_codec_integer_get, \
	  .put = emxx_codec_integer_put, .private_value = addr }

static int emxx_codec_integer_info(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_info *uinfo)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	FNC_ENTRY

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = codec->vol_info[IDX(addr)].count;
	uinfo->value.integer.min  = codec->vol_info[IDX(addr)].val_int_min;
	uinfo->value.integer.max  = codec->vol_info[IDX(addr)].val_int_max;
	uinfo->value.integer.step = codec->vol_info[IDX(addr)].val_int_step;

	FNC_EXIT return 0;
}

static int emxx_codec_integer_get(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	int cnt = codec->vol_info[IDX(addr)].count;
	int i;
	FNC_ENTRY

	spin_lock_irqsave(&codec->mixer_lock, flags);
	for (i = 0; i < cnt; i++) {
		ucontrol->value.integer.value[i]  =
			codec->vol_info[IDX(addr)].value[i];
	}
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	FNC_EXIT return 0;
}

static int emxx_codec_integer_put(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	int min  = codec->vol_info[IDX(addr)].val_int_min;
	int max  = codec->vol_info[IDX(addr)].val_int_max;
	int step = codec->vol_info[IDX(addr)].val_int_step;
	int i, change = 0;
	int cnt = codec->vol_info[IDX(addr)].count;
	int volume[2];
	int res = 0;
	FNC_ENTRY

	if (codec->power_on == 0)
		FNC_EXIT return -EINVAL;

	for (i = 0; i < cnt; i++) {
		d8b("value.integer.value[%d]=%ld\n",
			i, ucontrol->value.integer.value[i]);
		volume[i] = (ucontrol->value.integer.value[i] / step) * step;
		if (volume[i] < min)
			volume[i] = min;
		if (volume[i] > max)
			volume[i] = max;
	}

	spin_lock_irqsave(&codec->mixer_lock, flags);
	for (i = 0; i < cnt; i++) {
		change |= (codec->vol_info[IDX(addr)].value[i] != volume[i]);
		codec->vol_info[IDX(addr)].value[i] = volume[i];
		d8b("volume[%d]=%d\n", i, volume[i]);
	}
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	if (change) {
		res = emxx_codec_mixer_write(addr, codec);
		if (res < 0)
			FNC_EXIT return res;

	}

	FNC_EXIT return change;
}

#define EMXX_CODEC_ENUM(xname, xindex, addr) \
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
	  .info = emxx_codec_enum_info, .get = emxx_codec_enum_get, \
	  .put = emxx_codec_enum_put, .private_value = addr }

static int emxx_codec_enum_info(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_info *uinfo)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	int items = codec->enum_info[IDX(addr)].items;
	FNC_ENTRY

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = items;
	if (uinfo->value.enumerated.item > (items - 1))
		uinfo->value.enumerated.item = (items - 1);
	strcpy(uinfo->value.enumerated.name,
	       codec->enum_info[IDX(addr)].texts[uinfo->value.enumerated.item]);

	FNC_EXIT return 0;
}

static int emxx_codec_enum_get(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	FNC_ENTRY

	spin_lock_irqsave(&codec->mixer_lock, flags);
	ucontrol->value.enumerated.item[0] = codec->enum_info[IDX(addr)].value;
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	FNC_EXIT return 0;
}

static int emxx_codec_enum_put(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change, val;
	int addr = kcontrol->private_value;
	int items = codec->enum_info[IDX(addr)].items;
	int res = 0;
	FNC_ENTRY

	if (codec->power_on == 0)
		FNC_EXIT return -EINVAL;

	d8b("value.enumerated.item[0]=%d\n",
	 ucontrol->value.enumerated.item[0]);
	if (ucontrol->value.enumerated.item[0] > (items - 1))
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0];

	spin_lock_irqsave(&codec->mixer_lock, flags);
	change = (codec->enum_info[IDX(addr)].value != val);
	codec->enum_info[IDX(addr)].value = val;
	d8b("val=%d\n", val);
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	if (change) {
		res = emxx_codec_mixer_write(addr, codec);
		if (res < 0)
			FNC_EXIT return res;

	}

	FNC_EXIT return change;
}

#define EMXX_CODEC_BOOLEAN(xname, xindex, addr) \
	{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
	  .info = emxx_codec_boolean_info, .get = emxx_codec_boolean_get, \
	  .put = emxx_codec_boolean_put, .private_value = addr }

static int emxx_codec_boolean_info(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_info *uinfo)
{
	FNC_ENTRY

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	FNC_EXIT return 0;
}

static int emxx_codec_boolean_get(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	FNC_ENTRY

	spin_lock_irqsave(&codec->mixer_lock, flags);
	ucontrol->value.integer.value[0] = codec->bl_info[IDX(addr)].value;
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	FNC_EXIT return 0;
}

static int emxx_codec_boolean_put(struct snd_kcontrol *kcontrol,
 struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change, addr = kcontrol->private_value;
	int val;
	int res = 0;
	FNC_ENTRY

	if ((codec->power_on == 0) && (addr != MIXER_SW_CODEC_POWER_BL))
		FNC_EXIT return -EINVAL;

	d8b("value.integer.value[0]=%ld\n", ucontrol->value.integer.value[0]);
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irqsave(&codec->mixer_lock, flags);
	change = (codec->bl_info[IDX(addr)].value != val);
	codec->bl_info[IDX(addr)].value = val;
	d8b("val=%d\n", val);
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	if (change) {
		res = emxx_codec_mixer_write(addr, codec);
		if (res < 0)
			FNC_EXIT return res;

	}

	FNC_EXIT return change;
}

static struct snd_kcontrol_new emxx_codec_controls[] = {
	EMXX_CODEC_INTEGER(
		"Playback Volume", 0, MIXER_VOL_PLAYBACK),
	EMXX_CODEC_INTEGER(
		"Capture Volume", 0, MIXER_VOL_CAPTURE),
	EMXX_CODEC_ENUM(
		"Capture Source Switch", 0, MIXER_SW_CAPTURE_SOURCE),
	EMXX_CODEC_ENUM(
		"Capture Channel Mode Switch",
		0, MIXER_SW_CAPTURE_CHANNEL_MODE),
	EMXX_CODEC_ENUM(
		"Sampling Rate Switch", 0, MIXER_SW_SAMPLING_RATE),
	EMXX_CODEC_ENUM(
		"Playback Switch", 0, MIXER_SW_PLAYBACK),
	EMXX_CODEC_BOOLEAN(
		"CODEC Power Switch", 0, MIXER_SW_CODEC_POWER_BL),
};

static int __init emxx_codec_mixer_new(struct snd_card *card)
{
	unsigned int idx;
	int err;
	FNC_ENTRY

	if (snd_BUG_ON(!card))
		return -EINVAL;

	spin_lock_init(&codec_mixer.mixer_lock);
	mutex_init(&codec_mixer.power_mutex);
	strcpy(card->mixername, "emxx mixer");

	codec_mixer.card = card;

	for (idx = 0; idx < ARRAY_SIZE(emxx_codec_controls); idx++) {
		err = snd_ctl_add(card,
			snd_ctl_new1(&emxx_codec_controls[idx], &codec_mixer));
		if (err < 0)
			FNC_EXIT return err;

	}

	return 0;
}


static struct snd_pcm *emxx_codec;

static unsigned int rates[] = {
	7350, 8000, 11025, 12000, 14700, 16000,
	22050, 24000, 29400, 32000, 44100, 48000,
};

static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
	.count  = ARRAY_SIZE(rates),
	.list   = rates,
	.mask   = 0,
};

static int emxx_codec_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct emxx_codec_mixer *codec = &codec_mixer;
	int err;
	FNC_ENTRY

	if (codec->power_on == 0)
		FNC_EXIT return -EINVAL;

	runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
	runtime->hw.rates = (SNDRV_PCM_RATE_8000 |
			     SNDRV_PCM_RATE_11025 |
			     SNDRV_PCM_RATE_16000 |
			     SNDRV_PCM_RATE_22050 |
			     SNDRV_PCM_RATE_32000 |
			     SNDRV_PCM_RATE_44100 |
			     SNDRV_PCM_RATE_48000 |
			     SNDRV_PCM_RATE_KNOT);
	runtime->hw.rate_min = 8000;
	runtime->hw.rate_max = 48000;
	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	err = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		FNC_EXIT return err;

	err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					&hw_constraints_rates);
	if (err < 0)
		FNC_EXIT return err;

	FNC_EXIT return 0;
}

static void emxx_codec_shutdown(struct snd_pcm_substream *substream)
{
	FNC_ENTRY
	FNC_EXIT return;
}

static int emxx_codec_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	FNC_ENTRY

	ret = emxx_codec_sampling_rate_sw(runtime->rate);

	FNC_EXIT return ret;
}

static struct audio_stream emxx_codec_out = {
	.id                     = "pcm_m2p",
	.dma_ch                 = EMXX_DMAC_M2P_SIO1,
};

static struct audio_stream emxx_codec_in = {
	.id                     = "pcm_p2m",
	.dma_ch                 = EMXX_DMAC_P2M_SIO1,
};

static struct emxx_pcm_client emxx_codec_client = {
	.pcm_ch                 = EMXX_PCM_CH0,
	.startup                = emxx_codec_startup,
	.shutdown               = emxx_codec_shutdown,
	.prepare                = emxx_codec_prepare,
};


/* codec sound card */

struct snd_card *emxx_codec_card;

static char *id = ID_VALUE; /* ID for this card */
module_param(id, charp, 0444);

#ifdef CONFIG_PM
static int emxx_codec_suspend(struct platform_device *dev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(dev);
	struct emxx_codec_mixer *codec = &codec_mixer;
	int ret = 0;

	switch (state.event) {
	case PM_EVENT_SUSPEND:
		break;
	default:
		break;
	}

	mutex_lock(&codec->power_mutex);
	if (card) {
		ret = emxx_pcm_suspend(dev, state);
		if (ret == 0) {
			if (codec->power_on == 0)
				goto out;

			/* LOPS Power Save Mode */ /* 40H */
			ret = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				AK4648BIT_LOPS, AK4648BIT_LOPS);
			if (ret)
				goto out;
			/* PMLO Power Down Mode */ /* 08H */
			ret = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
				0, AK4648BIT_PMLO | AK4648BIT_PMDAC
				 | AK4648BIT_PMMIN | AK4648BIT_PMADL);
			if (ret)
				goto out;
			schedule_timeout_uninterruptible(AK4648_WAIT_LO_LOCK);
			ret = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
				0, AK4648BIT_PMADR);
			if (ret)
				goto out;

			/* LOPS Set0 */ /* 40H */
			ret = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				0, AK4648BIT_LOPS);
			if (ret)
				goto out;
		}
	}
out:
	mutex_unlock(&codec->power_mutex);

	return ret;
}

static int emxx_codec_resume(struct platform_device *dev)
{
	struct snd_card *card = platform_get_drvdata(dev);
	struct emxx_codec_mixer *codec = &codec_mixer;
	int ret = 0;

	mutex_lock(&codec->power_mutex);
	if (card) {
		ret = emxx_pcm_resume(dev);
		if (ret == 0) {
			if (codec->power_on == 0)
				goto out;

			/* LOPS Power Save Mode */ /* 03 */
			ret = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				AK4648BIT_LOPS, AK4648BIT_LOPS);
			if (ret)
				goto out;

			/* PMLO Power Down Mode */ /* 01 */
			ret = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_1,
				AK4648BIT_PMLO | AK4648BIT_PMDAC
				 | AK4648BIT_PMMIN | AK4648BIT_PMADL,
				AK4648BIT_PMLO | AK4648BIT_PMDAC
				 | AK4648BIT_PMMIN | AK4648BIT_PMADL);
			if (ret)
				goto out;
			schedule_timeout_uninterruptible(AK4648_WAIT_LO_LOCK);

			ret = CODEC_WRITE(AK4648REG_POWER_MANAGEMENT_3,
				 AK4648BIT_PMADR, AK4648BIT_PMADR);
			if (ret)
				goto out;

			/* LOPS Set0 */ /* 03 */
			ret = CODEC_WRITE(AK4648REG_SIGNAL_SELECT_2,
				0, AK4648BIT_LOPS);
			if (ret)
				goto out;

		}
	}
out:
	mutex_unlock(&codec->power_mutex);

	return ret;
}
#else
#define emxx_codec_suspend     NULL
#define emxx_codec_resume      NULL
#endif

static int emxx_codec_probe(struct platform_device *devptr)
{
	struct snd_card *card = NULL;
	int ret;
	FNC_ENTRY

	codec_init();

	card = snd_card_new(SNDRV_DEFAULT_IDX1, id, THIS_MODULE, 0);
	if (card == NULL)
		FNC_EXIT return -ENOMEM;

	snd_card_set_dev(card, &devptr->dev);

	emxx_codec_card = card;

	emxx_codec_client.s[SNDRV_PCM_STREAM_PLAYBACK] = &emxx_codec_out;
	emxx_codec_client.s[SNDRV_PCM_STREAM_CAPTURE] = &emxx_codec_in;
	emxx_codec_client.sett = &pcm_sett;

	ret = emxx_pcm_new(card, &emxx_codec_client, &emxx_codec);
	if (ret)
		goto err;

	ret = emxx_pcm_set_ctrl(emxx_codec_client.pcm_regs, &pcm_sett);
	if (ret)
		goto err;

	ret = emxx_codec_mixer_new(card);
	if (ret)
		goto err;

	snprintf(card->shortname, sizeof(card->shortname),
		 "%s", "emxx-codec");
	snprintf(card->longname, sizeof(card->longname),
		 "%s (%s)", "sound codec", card->mixername);

	ret = snd_card_register(card);
	if (ret == 0) {
#ifdef AUDIO_MAKING_DEBUG
		printk(KERN_INFO "Starting sound codec. with debug : M%d\n",
		 debug);
#else
		printk(KERN_INFO "Starting sound codec.\n");
#endif
		platform_set_drvdata(devptr, card);
		FNC_EXIT return 0;
	}

err:
	if (card)
		snd_card_free(card);

	FNC_EXIT return ret;
}

static int emxx_codec_remove(struct platform_device *devptr)
{
	FNC_ENTRY
	emxx_pcm_free(&emxx_codec_client);
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	FNC_EXIT return 0;
}


static struct platform_driver emxx_codec_driver = {
	.probe          = emxx_codec_probe,
	.remove         = __devexit_p(emxx_codec_remove),
#ifdef CONFIG_PM
	.suspend        = emxx_codec_suspend,
	.resume         = emxx_codec_resume,
#endif
	.driver         = {
		.name   = "pcm",
	},
};

static int __init emxx_codec_init(void)
{
	return platform_driver_register(&emxx_codec_driver);
}

static void __exit emxx_codec_exit(void)
{
	platform_driver_unregister(&emxx_codec_driver);

}


module_init(emxx_codec_init);
module_exit(emxx_codec_exit);

MODULE_DESCRIPTION("sound codec driver for emxx chip");
MODULE_LICENSE("GPL");

