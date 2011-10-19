/*
 * File Name		: sound/arm/emxx-cs8427.c
 * Function		: CS8427 CODEC
 * Release Version 	: Ver 1.10
 * Release Date		: 2010/04/19
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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <asm/unaligned.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/cs8427.h>
#include <sound/asoundef.h>

#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <mach/pwc.h>
#include <mach/pmu.h>
#include <mach/pm.h>
#include <mach/gpio.h>
#include <sound/initval.h>
#include <sound/hwdep.h>

#include "emxx-pcm.h"

#define CS8427_SOC_PROC 1
#if CS8427_SOC_PROC
#include <linux/proc_fs.h>
#endif

#define DEV_NAME "CS8427"

#define printk_err(fmt, arg...) \
	do {                     \
		printk(KERN_ERR DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_wrn(fmt, arg...) \
	do {                     \
		printk(KERN_WARNING DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_info(fmt, arg...) \
	do {                      \
		printk(KERN_INFO DEV_NAME ": " fmt, ## arg); \
	} while (0)

#define _DEBUG_CS8427  0x00 /* 00008421(bit) */
			 /* 0x01: debug function in
			  * 0x02: debug function out
			  * 0x40: debug note
			  */

#if _DEBUG_CS8427 != 0
#define printk_dbg(level, fmt, arg...) \
	do {                            \
		if (level > 0) \
			printk(KERN_INFO DEV_NAME ": %s: " fmt, \
			__func__, ## arg); \
	} while (0)
#else
#define printk_dbg(level, fmt, arg...) \
	;
#endif

#define CS8427_CTR_REG_MAX 0x14

static void snd_cs8427_reset(void);
static int snd_cs8427_codec_create(unsigned int reset_timeout);
static int i2c_cs8427_write(unsigned char reg, unsigned char data);
static int i2c_cs8427_read(unsigned char reg, unsigned char *data);
static int snd_cs8427_spdif_pcm(unsigned int rate);
static int snd_cs8427_spdif_build(struct snd_pcm_substream *play_substream,
				struct snd_pcm_substream *cap_substream);

struct cs8427_stream {
	struct snd_pcm_substream *substream;
	char hw_status[24];			/* hardware status */
	char def_status[24];		/* default status */
	char pcm_status[24];		/* PCM private status */
	char hw_udata[32];
	struct snd_kcontrol *pcm_ctl;
};

struct cs8427 {
	/* map of first 1 + 13 registers */
	unsigned char regmap[CS8427_CTR_REG_MAX];
	unsigned int rate;
	unsigned int reset_timeout;
	struct cs8427_stream playback;
	struct cs8427_stream capture;
};

struct emxx_spdif {
	unsigned int cs8427_timeout;    /* CS8427 reset timeout in HZ/100 */
	struct snd_pcm_substream *playback_con_substream;
	struct snd_pcm_substream *capture_con_substream;
};

static char *id; 		/* ID for this card */
module_param(id, charp, 0444);
static struct snd_pcm *emxx_cs8427_codec;
static struct snd_card *emxx_cs8427_card;
static struct cs8427 *cs8427_chip;
static struct emxx_spdif spdif;
static struct i2c_client *i2c_cs8427_client;
static struct pcm_regs *p_pcm_regs;
spinlock_t cs8427_lock;

static pcm_ctrl_t pcm_sett = {
	.func = {
		.mode_sel       = PCM_MODE_2,
		.m_s            = PCM_MASTER_MODE,
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

static struct audio_stream emxx_cs8427_codec_out = {
	.id         = "pcm1_m2p",
	.dma_ch     = EMXX_DMAC_M2P_SIO3,
};

static struct audio_stream emxx_cs8427_codec_in = {
	.id         = "pcm1_p2m",
	.dma_ch     = EMXX_DMAC_P2M_SIO3,
};

static unsigned int rates[] = { 32000, 44100, 48000, 96000 };

static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
	.count  = ARRAY_SIZE(rates),
	.list   = rates,
	.mask   = 0,
};

#ifdef CS8427_SOC_PROC
struct cs8427_reg_info {
	char *name;
	unsigned char reg;
};

struct cs8427_reg_info cs8427_reg[] = {
	{"CS8427_REG_CONTROL1", CS8427_REG_CONTROL1},
	{"CS8427_REG_CONTROL2", CS8427_REG_CONTROL2},
	{"CS8427_REG_DATAFLOW", CS8427_REG_DATAFLOW},
	{"CS8427_REG_CLOCKSOURCE", CS8427_REG_CLOCKSOURCE},
	{"CS8427_REG_SERIALINPUT", CS8427_REG_SERIALINPUT},
	{"CS8427_REG_SERIALOUTPUT", CS8427_REG_SERIALOUTPUT},
	{"CS8427_REG_INT1STATUS", CS8427_REG_INT1STATUS},
	{"CS8427_REG_INT2STATUS", CS8427_REG_INT2STATUS},
	{"CS8427_REG_INT1MASK", CS8427_REG_INT1MASK},
	{"CS8427_REG_INT1MODEMSB", CS8427_REG_INT1MODEMSB},
	{"CS8427_REG_INT1MODELSB", CS8427_REG_INT1MODELSB},
	{"CS8427_REG_INT2MASK", CS8427_REG_INT2MASK},
	{"CS8427_REG_INT2MODEMSB", CS8427_REG_INT2MODEMSB},
	{"CS8427_REG_INT2MODELSB", CS8427_REG_INT2MODELSB},
	{"CS8427_REG_RECVCSDATA", CS8427_REG_RECVCSDATA},
	{"CS8427_REG_RECVERRORS", CS8427_REG_RECVERRORS},
	{"CS8427_REG_RECVERRMASK", CS8427_REG_RECVERRMASK},
	{"CS8427_REG_CSDATABUF", CS8427_REG_CSDATABUF},
	{"CS8427_REG_UDATABUF", CS8427_REG_UDATABUF},
	{"CS8427_REG_QSUBCODE0", CS8427_REG_QSUBCODE + 0},
	{"CS8427_REG_QSUBCODE1", CS8427_REG_QSUBCODE + 1},
	{"CS8427_REG_QSUBCODE2", CS8427_REG_QSUBCODE + 2},
	{"CS8427_REG_QSUBCODE3", CS8427_REG_QSUBCODE + 3},
	{"CS8427_REG_QSUBCODE4", CS8427_REG_QSUBCODE + 4},
	{"CS8427_REG_QSUBCODE4", CS8427_REG_QSUBCODE + 5},
	{"CS8427_REG_QSUBCODE6", CS8427_REG_QSUBCODE + 6},
	{"CS8427_REG_QSUBCODE7", CS8427_REG_QSUBCODE + 7},
	{"CS8427_REG_QSUBCODE8", CS8427_REG_QSUBCODE + 8},
	{"CS8427_REG_QSUBCODE9", CS8427_REG_QSUBCODE + 9},
	{"CS8427_REG_OMCKRMCKRATIO", CS8427_REG_OMCKRMCKRATIO},
	{"CS8427_REG_CORU_DATABUF0", CS8427_REG_CORU_DATABUF + 0},
	{"CS8427_REG_CORU_DATABUF1", CS8427_REG_CORU_DATABUF + 1},
	{"CS8427_REG_CORU_DATABUF2", CS8427_REG_CORU_DATABUF + 2},
	{"CS8427_REG_CORU_DATABUF3", CS8427_REG_CORU_DATABUF + 3},
	{"CS8427_REG_CORU_DATABUF4", CS8427_REG_CORU_DATABUF + 4},
	{"CS8427_REG_CORU_DATABUF5", CS8427_REG_CORU_DATABUF + 5},
	{"CS8427_REG_CORU_DATABUF6", CS8427_REG_CORU_DATABUF + 6},
	{"CS8427_REG_CORU_DATABUF7", CS8427_REG_CORU_DATABUF + 7},
	{"CS8427_REG_CORU_DATABUF8", CS8427_REG_CORU_DATABUF + 8},
	{"CS8427_REG_CORU_DATABUF9", CS8427_REG_CORU_DATABUF + 9},
	{"CS8427_REG_CORU_DATABUF10", CS8427_REG_CORU_DATABUF + 10},
	{"CS8427_REG_CORU_DATABUF11", CS8427_REG_CORU_DATABUF + 11},
	{"CS8427_REG_CORU_DATABUF12", CS8427_REG_CORU_DATABUF + 12},
	{"CS8427_REG_CORU_DATABUF13", CS8427_REG_CORU_DATABUF + 13},
	{"CS8427_REG_CORU_DATABUF14", CS8427_REG_CORU_DATABUF + 14},
	{"CS8427_REG_CORU_DATABUF15", CS8427_REG_CORU_DATABUF + 15},
	{"CS8427_REG_CORU_DATABUF16", CS8427_REG_CORU_DATABUF + 16},
	{"CS8427_REG_CORU_DATABUF17", CS8427_REG_CORU_DATABUF + 17},
	{"CS8427_REG_CORU_DATABUF18", CS8427_REG_CORU_DATABUF + 18},
	{"CS8427_REG_CORU_DATABUF19", CS8427_REG_CORU_DATABUF + 19},
	{"CS8427_REG_CORU_DATABUF20", CS8427_REG_CORU_DATABUF + 20},
	{"CS8427_REG_CORU_DATABUF21", CS8427_REG_CORU_DATABUF + 21},
	{"CS8427_REG_CORU_DATABUF22", CS8427_REG_CORU_DATABUF + 22},
	{"CS8427_REG_CORU_DATABUF23", CS8427_REG_CORU_DATABUF + 23},
	{"CS8427_REG_ID_AND_VER", CS8427_REG_ID_AND_VER},
};

static struct proc_dir_entry *cs8427_proc_entry;

void print_cs8427_reg(void)
{
	int res = 0, i;
	unsigned char buf, data;

	if (i2c_cs8427_client == NULL) {
		printk_err("i2c cs8427 not available!\n");
		return;
	}

	for (i = 0;
	     i < sizeof(cs8427_reg) / sizeof(struct cs8427_reg_info);
	     i++) {
		buf = cs8427_reg[i].reg;
		res = i2c_master_send(i2c_cs8427_client, &buf, 1);
		if (res <= 0)
			printk_err("i2c cs8427 send failed%d!\n", res);

		res = i2c_master_recv(i2c_cs8427_client, &data, 1);
		if (res > 0) {
			printk_dbg((_DEBUG_CS8427&0x04),
			 "cs8427 RegName:%s, RegAddr:0x%02x, RegVal:0x%02x\n",
			 cs8427_reg[i].name, buf, data);
		} else
			printk_err("i2c cs8427 recv failed!\n");
	}
}

static ssize_t cs8427_proc_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	print_cs8427_reg();
	return 0;
}

static int cs8427_proc_write(struct file *file, const char __user *buffer,
				unsigned long count, void *data)
{
	char                 buf[80];
	char                 *pbuf, *value;
	unsigned int         i = 0, reg = 0;
	unsigned char        reg_val, reg2_val;
	char                 cmd = 0;
	unsigned long        length;
	long                 tmp;

	if (count < 3)
		return -EINVAL;
	length = count > 80 ? 80 : count;
	if (copy_from_user(&buf[0], buffer, length))
		return -EFAULT;
	buf[length - 1] = '\0';
	pbuf = &buf[0];
	printk(KERN_INFO "USEAGE: \
			echo \"r 0xreg\" > /proc/driver/cs8427codec\n");
	printk(KERN_INFO "USEAGE: \
			echo \"w 0xreg 0xvalue\" > /proc/driver/cs8427codec\n");

	value = strsep(&pbuf, " ");
	if (value != NULL) {
		cmd = *value;
		if ((cmd != 'r') && (cmd != 'w'))
			return -EINVAL;
	} else
		return -EINVAL;

	value = strsep(&pbuf, " ");
	if ((value != NULL) && (*value != ' '))
		strict_strtol(value, 16, &tmp);
	else
		return -EINVAL;
	i = tmp;

	if (cmd == 'w') {
		value = strsep(&pbuf, " ");
		if ((value != NULL) && (*value != ' '))
			strict_strtol(value, 16, &tmp);
		else
			return -EINVAL;
		reg = tmp;
	}

	if ('r' == cmd) {
		if (i > 0x55) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		i2c_cs8427_read(i, &reg_val);
		printk(KERN_INFO "0x[%02x]=0x%02x\n", i, reg_val);
	} else if ('w' == cmd) {
		if (i > 0x55) {
			printk(KERN_ERR "invalid value!\n");
			goto error;
		}
		reg_val = (unsigned char)reg;
		i2c_cs8427_write(i, reg_val);
		i2c_cs8427_read(i, &reg2_val);
		printk(KERN_INFO "write 0x%02x to 0x[%02x], read back 0x%02x\n",
			reg_val, i, reg2_val);
	} else {
		printk(KERN_ERR "unknow opt!\n");
		goto error;
	}

	return count;
error:
	printk(KERN_INFO "r/w index(0x%%02x) value(0x%%02x)\n");
	return count;
}
#endif

static int emxx_cs8427_codec_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE
			| SNDRV_PCM_FMTBIT_S24_3LE;

	runtime->hw.rates = (SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_KNOT);
	runtime->hw.rate_min = 8000;
	runtime->hw.rate_max = 96000;
	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	err = snd_pcm_hw_constraint_integer(runtime,
		SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		return err;
	snd_pcm_set_sync(substream);
	snd_pcm_hw_constraint_msbits(runtime, 0, 32, 16);

	err = snd_pcm_hw_constraint_list(runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates);
	if (err < 0)
		return err;

	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static void emxx_cs8427_codec_shutdown(struct snd_pcm_substream *substream)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");

	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return;
}

static int emxx_cs8427_codec_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;
	char data = 0;
	snd_pcm_format_t format = substream->runtime->format;
	u64 formats = (1ULL << format);
	unsigned long val;

	val = readl(SMU_USIB0SCLKDIV) & 0x0000FFFF;
	switch (runtime->rate) {
	case 32000:
		printk_dbg((_DEBUG_CS8427&0x04),
			"Sampling Frequency 32KHz\n");
		val |= 0x006F0000;
		break;

	case 44100:
		printk_dbg((_DEBUG_CS8427&0x04),
			"Sampling Frequency 44.1KHz\n");
		val |= 0x02030000;
		break;

	case 48000:
		printk_dbg((_DEBUG_CS8427&0x04),
			"Sampling Frequency 48KHz\n");
		val |= 0x004A0000;
		break;

	case 96000:
		printk_dbg((_DEBUG_CS8427&0x04),
			"Sampling Frequency 96KHz\n");
		val |= 0x00240000;
		break;

	default:
		return -EINVAL;
	}
	writel(val, SMU_USIB0SCLKDIV);

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	ret = snd_cs8427_spdif_pcm(runtime->rate);
	if (formats & SNDRV_PCM_FMTBIT_S16_LE) {
		printk_dbg((_DEBUG_CS8427&0x04), "SNDRV_PCM_FMTBIT_S16_LE\n");
		pcm_sett.cyc.sib = 0x0f;
		pcm_sett.cyc.sob = 0x0f;
		pcm_sett.cyc.rx_pd = PCM_PADDING_ON;
		pcm_sett.cyc.tx_pd = PCM_PADDING_ON;
		printk_dbg((_DEBUG_CS8427&0x04), "Use p_pcm_regs\n");
		ret = emxx_pcm_set_ctrl(p_pcm_regs, &pcm_sett);
		if (ret)
			return ret;
		i2c_cs8427_read(CS8427_REG_SERIALINPUT, &data);
		data &= ~CS8427_SIRESMASK;
		data |= CS8427_SIRES16;
		i2c_cs8427_write(CS8427_REG_SERIALINPUT, data);

		i2c_cs8427_read(CS8427_REG_SERIALOUTPUT, &data);
		data &= ~CS8427_SORESMASK;
		data |= CS8427_SORES16;
		i2c_cs8427_write(CS8427_REG_SERIALOUTPUT, data);
	} else if (formats &
		(SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE)) {
		printk_dbg((_DEBUG_CS8427&0x04), "SNDRV_PCM_FMTBIT_S24_LE\n");
		pcm_sett.cyc.sib = 0x17;
		pcm_sett.cyc.sob = 0x17;
		pcm_sett.cyc.rx_pd = PCM_PADDING_OFF;
		pcm_sett.cyc.tx_pd = PCM_PADDING_OFF;
		printk_dbg((_DEBUG_CS8427&0x04), "Use p_pcm_regs\n");
		ret = emxx_pcm_set_ctrl(p_pcm_regs, &pcm_sett);
		if (ret)
			return ret;
		i2c_cs8427_read(CS8427_REG_SERIALINPUT, &data);
		data &= ~CS8427_SIRESMASK;
		data |= CS8427_SIRES24;
		i2c_cs8427_write(CS8427_REG_SERIALINPUT, data);

		i2c_cs8427_read(CS8427_REG_SERIALOUTPUT, &data);
		data &= ~CS8427_SORESMASK;
		data |= CS8427_SORES24;
		i2c_cs8427_write(CS8427_REG_SERIALOUTPUT, data);
	} else
		printk_wrn("Unkown. It is neither 16 bits, nor 24 bits. \n");

	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return ret;
}

static struct emxx_pcm_client emxx_cs8427_codec_client = {
	.pcm_ch                 = EMXX_PCM_CH1,
	.startup                = emxx_cs8427_codec_startup,
	.shutdown               = emxx_cs8427_codec_shutdown,
	.prepare                = emxx_cs8427_codec_prepare,
};

int __devinit snd_spdif_init_cs8427(struct emxx_spdif *spdif)
{
	int err;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	err = snd_cs8427_codec_create((500 * HZ) / 1000);
	if (err < 0) {
		printk_err("CS8427 initialization failed\n");
		return err;
	}
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static irqreturn_t cs8427_callback(int irq, void *dev_id)
{
	static irqreturn_t ret = IRQ_NONE;
	char status1, status2;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	disable_irq(GPIO_DAIT_INT);
	i2c_cs8427_read(CS8427_REG_INT1STATUS, &status1);
	printk_dbg((_DEBUG_CS8427&0x04),
		"CS8427_REG_INT1STATUS=0x%02x\n", status1);
	i2c_cs8427_read(CS8427_REG_INT2STATUS, &status2);
	printk_dbg((_DEBUG_CS8427&0x04),
		"CS8427_REG_INT2STATUS=0x%02x\n", status2);
	enable_irq(GPIO_DAIT_INT);
	ret = IRQ_HANDLED;
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return ret;
}

static int cs8427_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err;
	unsigned char data;
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	i2c_cs8427_client = client;

	err = i2c_cs8427_read(CS8427_REG_ID_AND_VER, &data);
	if (err < 0) {
		printk_err("Read id error.\n");
		return err;
	}
	printk_info("Version: 0x%x\n", data);

	printk_dbg((_DEBUG_CS8427&0x020), "Leave\n");
	return 0;
}

static int cs8427_i2c_remove(struct i2c_client *client)
{
	i2c_cs8427_client = NULL;
	return 0;
}

static struct i2c_device_id cs8427_i2c_idtable[] = {
	{ I2C_SLAVE_SPDIF_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, cs8427_i2c_idtable);

static struct i2c_driver i2c_cs8427_driver = {
	.driver.name    = "I2c for cs8427",
	.id             = I2C_DRIVERID_I2CDEV, /* Fake ID */
	.id_table       = cs8427_i2c_idtable,
	.probe          = cs8427_i2c_probe,
	.remove         = cs8427_i2c_remove,
};

static int snd_spdif_init_i2c(void)
{
	int res = 0;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	res = i2c_add_driver(&i2c_cs8427_driver);
	if (res == 0) {
		if (i2c_cs8427_client == NULL) {
			i2c_del_driver(&i2c_cs8427_driver);
			printk_err("codec_i2c_found_proc() not called!\n");
			return -EIO;
		}
	} else
		printk_err("i2c codec inserted failed! res=%d\n", res);

	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return res;
}

static int emxx_cs8427_probe(struct platform_device *devptr)
{
	struct snd_card *card = NULL;
	struct emxx_spdif *pspdif = NULL;
	int ret;
	unsigned long val;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
#ifdef CS8427_SOC_PROC
	cs8427_proc_entry = create_proc_entry("driver/cs8427codec", 0, NULL);
	if (cs8427_proc_entry) {
		cs8427_proc_entry->data = emxx_cs8427_codec;
		cs8427_proc_entry->read_proc = cs8427_proc_read;
		cs8427_proc_entry->write_proc = cs8427_proc_write;
	}
#endif

	spin_lock_init(&cs8427_lock);
	pspdif = &spdif;
	card = snd_card_new(SNDRV_DEFAULT_IDX1, id, THIS_MODULE, 0);
	if (card == NULL)
		return -ENOMEM;

	snd_card_set_dev(card, &devptr->dev);
	printk_dbg((_DEBUG_CS8427&0x04),
		"GPIO_DAIT_RST=%d\n", gpio_get_value(GPIO_DAIT_RST));
	gpio_set_value(GPIO_DAIT_RST, 1);
	mdelay(10);
	printk_dbg((_DEBUG_CS8427&0x04),
		"GPIO_DAIT_RST=%d\n", gpio_get_value(GPIO_DAIT_RST));
	gpio_set_value(GPIO_DAIT_RST, 0);
	mdelay(10);
	printk_dbg((_DEBUG_CS8427&0x04),
		"GPIO_DAIT_RST=%d\n", gpio_get_value(GPIO_DAIT_RST));
	gpio_set_value(GPIO_DAIT_RST, 1);
	mdelay(10);
	printk_dbg((_DEBUG_CS8427&0x04),
		"GPIO_DAIT_RST=%d\n", gpio_get_value(GPIO_DAIT_RST));

	writel(0x00000000, SMU_OSC0CTRL1);	/* 11.2896MHz */
	udelay(1000);

	val = readl(CHG_DRIVE5) & 0xFFFFFFF0;
	val |= 0x0000000F; /* HSI_1,HSI_0 12mA */
	writel(val, CHG_DRIVE5);

	val = readl(SMU_USIB0SCLKDIV) & 0x0000FFFF;
	val |= 0x02030000; /* OSC0 44.1KHz */
	writel(val, SMU_USIB0SCLKDIV);

	writel((readl(CHG_PINSEL_G032) & 0xFFFFF0FF), CHG_PINSEL_G032);
	writel((readl(CHG_PINSEL_G096) & 0xFF87FFFF), CHG_PINSEL_G096);
	writel(((readl(CHG_PULL13) & 0xFFF0FFFF) | 0x00040000), CHG_PULL13);
	writel(((readl(CHG_PULL0) & 0xFFFFF0FF) | 0x00000C00), CHG_PULL0);
	writel((readl(CHG_PINSEL_USI) & 0xFFFFFFCF), CHG_PINSEL_USI);
	writel((readl(CHG_PULL16) & ~0x4000), CHG_PULL16);

	val = readl(CHG_DRIVE3) & 0x0FF0FFFF;
	val |= 0x000A0000; /* USI3_1, USI3_0 : 8mA */
	writel(val, CHG_DRIVE3);

	ret = snd_spdif_init_i2c();
	if (ret < 0) {
		printk_err("Spdif init i2c error.\n");
		goto err;
	}

	pspdif->cs8427_timeout = 500;
	ret = snd_spdif_init_cs8427(pspdif);
	if (ret < 0)
		goto err;

	emxx_cs8427_card = card;

	emxx_cs8427_codec_client.s[SNDRV_PCM_STREAM_PLAYBACK] =
		&emxx_cs8427_codec_out;
	emxx_cs8427_codec_client.s[SNDRV_PCM_STREAM_CAPTURE] =
		&emxx_cs8427_codec_in;
	emxx_cs8427_codec_client.sett = &pcm_sett;

	ret = emxx_pcm_new(card, &emxx_cs8427_codec_client, &emxx_cs8427_codec);
	if (ret)
		goto err;

	p_pcm_regs = emxx_cs8427_codec_client.pcm_regs;
	printk_dbg((_DEBUG_CS8427&0x04), "Set p_pcm_regs\n");
	ret = emxx_pcm_set_ctrl(emxx_cs8427_codec_client.pcm_regs, &pcm_sett);
	if (ret)
		goto err;

	strcpy(card->mixername, "emxx spdif");

	/* assign channels to iec958 */
	ret = snd_cs8427_spdif_build(emxx_cs8427_codec->streams[0].substream,
		emxx_cs8427_codec->streams[1].substream);
	if (ret < 0)
		return ret;

	snprintf(card->shortname, sizeof(card->shortname),
		"%s", "emxx-cs8427-codec");
	snprintf(card->longname, sizeof(card->longname),
		"%s (%s)", "sound emxx cs8427 codec", card->mixername);

	ret = snd_card_register(card);
	if (ret == 0) {
#ifdef AUDIO_MAKING_DEBUG
		printk_info("Starting sound emxx cs8427 codec. "
				"with debug : M%d\n",
				debug);
#else
		printk_info("Starting sound emxx cs8427 codec.\n");
#endif
		platform_set_drvdata(devptr, card);
	}

	ret = request_irq(GPIO_DAIT_INT, cs8427_callback, 0, "spdif", NULL);
	if (ret)
		printk_err("fail in request_irq(GPIO_DAIT_INT)\n");
	disable_irq(GPIO_DAIT_INT);
	enable_irq(GPIO_DAIT_INT);

#ifdef CS8427_SOC_PROC
	print_cs8427_reg();
#endif
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;

err:
	if (card)
		snd_card_free(card);

	return ret;
}

static int emxx_cs8427_remove(struct platform_device *devptr)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");

#ifdef CS8427_SOC_PROC
	remove_proc_entry("driver/cs8427codec", 0);
#endif
	emxx_pcm_free(&emxx_cs8427_codec_client);
	snd_card_free(platform_get_drvdata(devptr));
	i2c_del_driver(&i2c_cs8427_driver);
	platform_set_drvdata(devptr, NULL);
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

#ifdef CONFIG_PM
static int emxx_cs8427_suspend(struct platform_device *dev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(dev);
	int ret = 0;
	int i = 0;
	char data = 0;

	switch (state.event) {
	case PM_EVENT_SUSPEND:
		break;
	default:
		break;
	}

	if (card)
		ret = emxx_pcm_suspend(dev, state);

	for (i = 0; i < CS8427_CTR_REG_MAX; i++)
		i2c_cs8427_read(i, &cs8427_chip->regmap[i]);

	i2c_cs8427_read(CS8427_REG_CLOCKSOURCE, &data);
	data &= ~CS8427_RUN;
	i2c_cs8427_write(CS8427_REG_CLOCKSOURCE, data);

	return ret;
}

static int emxx_cs8427_resume(struct platform_device *dev)
{
	struct snd_card *card = platform_get_drvdata(dev);
	int ret = 0;
	int i = 0;

	if (card)
		ret = emxx_pcm_resume(dev);

	for (i = 0; i < CS8427_CTR_REG_MAX; i++) {
		/*CS8427_REG_CLOCKSOURCE is set at the end.*/
		if (i == CS8427_REG_CLOCKSOURCE)
			continue;

		/*Read Only: */
		if ((i == CS8427_REG_INT1STATUS) ||
			(i == CS8427_REG_INT2STATUS) ||
			(i == CS8427_REG_RECVCSDATA) ||
			(i == CS8427_REG_RECVERRORS))
			continue;

		i2c_cs8427_write(i, cs8427_chip->regmap[i]);
	}

	cs8427_chip->regmap[CS8427_REG_CLOCKSOURCE] |= CS8427_RUN;
	i2c_cs8427_write(CS8427_REG_CLOCKSOURCE,
			cs8427_chip->regmap[CS8427_REG_CLOCKSOURCE]);
	return ret;
}
#endif

static struct platform_driver emxx_cs8427_driver = {
	.probe      = emxx_cs8427_probe,
	.remove     = __devexit_p(emxx_cs8427_remove),
#ifdef CONFIG_PM
	.suspend    = emxx_cs8427_suspend,
	.resume     = emxx_cs8427_resume,
#endif
	.driver     = {
		.name   = "pcm1",
	},
};

static int i2c_cs8427_write(unsigned char reg, unsigned char data)
{
	int res = 0;
	unsigned char buf[2];

	buf[0] = reg & 0x7f;
	buf[1] = data;

	res = i2c_master_send(i2c_cs8427_client, buf, 2);
	if (res > 0)
		res = 0;
	else
		printk_err("I2c cs8427 write failed!\n");

	return res;
}

static int i2c_cs8427_read(unsigned char reg, unsigned char *data)
{
	int res = 0;

	if (i2c_cs8427_client == NULL) {
		printk_err("I2c cs8427 not available!\n");
		return -EIO;
	}

	res = i2c_master_send(i2c_cs8427_client, &reg, 1);
	if (res <= 0) {
		printk_err("I2c cs8427 send failed%d!\n", res);
		return res;
	}

	res = i2c_master_recv(i2c_cs8427_client, data, 1);
	if (res > 0)
		res = 0;
	else
		printk_err("I2c cs8427 recv failed!\n");

	return res;
}

static unsigned char swapbits(unsigned char val)
{
	int bit;
	unsigned char res = 0;
	for (bit = 0; bit < 8; bit++) {
		res <<= 1;
		res |= val & 1;
		val >>= 1;
	}
	return res;
}

static int snd_cs8427_select_corudata(int udata)
{
	int err;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	if (udata)
		udata = CS8427_BSEL;
	else
		udata = 0;

	if (udata != (cs8427_chip->regmap[CS8427_REG_CSDATABUF] & udata)) {
		cs8427_chip->regmap[CS8427_REG_CSDATABUF] &= ~CS8427_BSEL;
		cs8427_chip->regmap[CS8427_REG_CSDATABUF] |= udata;
		err = i2c_cs8427_write(CS8427_REG_CSDATABUF,
			cs8427_chip->regmap[CS8427_REG_CSDATABUF]);
		if (err < 0)
			return err;
	}
	printk_dbg((_DEBUG_CS8427&0x01), "Leave\n");
	return 0;
}

static int snd_cs8427_send_corudata(int udata, unsigned char *ndata, int count)
{
	char *hw_data;
	char data[32];
	int err, idx;
	int i = 0;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	if (udata)
		hw_data = cs8427_chip->playback.hw_udata;
	else
		hw_data = cs8427_chip->playback.hw_status;

	err = memcmp(hw_data, ndata, count);
	if (err == 0)
		return 0;

	err = snd_cs8427_select_corudata(udata);
	if (err < 0)
		return err;

	memcpy(hw_data, ndata, count);

	if (udata) {
		memset(data, 0, sizeof(data));
		err = memcmp(hw_data, data, count);
		if (err == 0) {
			cs8427_chip->regmap[CS8427_REG_UDATABUF] &=
				~CS8427_UBMMASK;
			cs8427_chip->regmap[CS8427_REG_UDATABUF] |=
				CS8427_UBMZEROS | CS8427_EFTUI;
			err = i2c_cs8427_write(CS8427_REG_UDATABUF,
				cs8427_chip->regmap[CS8427_REG_UDATABUF]);
			return err;
		}
	}
	for (idx = 0; idx < count; idx++)
		data[idx + 1] = swapbits(ndata[idx]);

	for (i = 0; i < (count + 1); i++) {
		err = i2c_cs8427_write(CS8427_REG_CORU_DATABUF+i, data[i]);
		if (err < 0)
			return -EIO;
	}
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");

	return 0;
}

static int snd_cs8427_codec_create(unsigned int reset_timeout)
{
	int err;
	unsigned char data;
	unsigned char buf[24];

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	err = i2c_cs8427_read(CS8427_REG_ID_AND_VER, &data);
	if (err < 0) {
		printk_err("Read id error.\n");
		return err;
	}
	if (data != CS8427_VER8427A) {
		/* give second chance */
		printk_wrn("Invalid CS8427 signature 0x%x: try again...\n",
			data);
		err = i2c_cs8427_read(CS8427_REG_ID_AND_VER, &data);
		if (err < 0) {
			printk_err("Read id error.\n");
			return err;
		}
		if (data != CS8427_VER8427A) {
			printk_err("Invalid CS8427 signature 0x%x.\n", data);
			printk_err("Initialization is not completed\n");
			return -EFAULT;
		}
	}

	cs8427_chip = kmalloc(sizeof(struct cs8427), GFP_KERNEL);
	if (cs8427_chip == NULL) {
		printk_err("Malloc error.\n");
		return -ENOMEM;
	}

	/* turn off run bit while making changes to configuration */
	err = i2c_cs8427_write(CS8427_REG_CLOCKSOURCE, 0x00);
	if (err < 0) {
		printk_err("Turn off error!\n");
		goto __fail;
	}

	/* 1. send initial values */

	/* CS8427_REG_CONTROL1: RMCK to OMCK, valid PCM audio,*/
	/* disable mutes, TCBL=output                         */
	err = i2c_cs8427_write(CS8427_REG_CONTROL1, 0);
	if (err < 0) {
		printk_err("Write control1 error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_CONTROL1] = 0;

	/* CS8427_REG_CONTROL2: hold last valid audio sample, */
	/* RMCK=256*Fs, normal stereo operation               */
	err = i2c_cs8427_write(CS8427_REG_CONTROL2, 0);
	if (err < 0) {
		printk_err("Write control2 error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_CONTROL2] =      0;

	/* CS8427_REG_DATAFLOW: output drivers normal operation, */
	/* input serial port to AES3 transmitter without PLL     */
	err = i2c_cs8427_write(CS8427_REG_DATAFLOW, CS8427_TXDSERIAL
		| CS8427_SPDSERIAL);
	if (err < 0) {
		printk_err("Write data flow error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_DATAFLOW] = CS8427_TXDSERIAL
		| CS8427_SPDSERIAL;

	/* CS8427_REG_SERIALINPUT:                                   */
	/* Serial audio input port data format = I2S, 24-bit, 64*Fsi */
	err = i2c_cs8427_write(CS8427_REG_SERIALINPUT, CS8427_SIDEL
		| CS8427_SILRPOL | CS8427_SIRES16);
	if (err < 0) {
		printk_err("Write serialinput error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_SERIALINPUT] =
		CS8427_SIDEL | CS8427_SILRPOL | CS8427_SIRES16;

	/* CS8427_REG_CLOCKSOURCE:
	   Run off, CMCK=256*Fs,
	   output time base = recovered input clock,
	   input time base = recovered input clock,
	   recovered input clock source is ILRCK changed to AES3INPUT
	   (workaround, see snd_cs8427_reset) */
	err = i2c_cs8427_write(CS8427_REG_CLOCKSOURCE,
		CS8427_RXDILRCK | CS8427_OUTC);
	if (err < 0) {
		printk_err("Write clocksource error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_CLOCKSOURCE] =
		CS8427_RXDILRCK | CS8427_OUTC;

	/* CS8427_REG_SERIALOUTPUT:                                   */
	/* Serial audio output port data format = I2S, 24-bit, 64*Fsi */
	err = i2c_cs8427_write(CS8427_REG_SERIALOUTPUT,
		CS8427_SOMS | CS8427_SORES16 | CS8427_SODEL | CS8427_SOLRPOL);
	if (err < 0) {
		printk_err("Write serialoutput error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_SERIALOUTPUT] =
		CS8427_SOMS | CS8427_SORES16 | CS8427_SODEL | CS8427_SOLRPOL;

	/* 2. Turn off CS8427 interrupt stuff that is not used in hardware */
	/* from address 9 to 15 */
	err = i2c_cs8427_write(CS8427_REG_INT1MASK, 0xc7);
	if (err < 0) {
		printk_err("Write CS8427_REG_INT1MASK error!\n");
		goto __fail;
	}

	err = i2c_cs8427_write(CS8427_REG_INT1MODEMSB, 0x0);
	if (err < 0) {
		printk_err("Write CS8427_REG_INT1MODEMSB error!\n");
		goto __fail;
	}

	err = i2c_cs8427_write(CS8427_REG_INT1MODELSB, 0x0);
	if (err < 0) {
		printk_err("Write CS8427_REG_INT1MODELSB error!\n");
		goto __fail;
	}

	err = i2c_cs8427_write(CS8427_REG_INT2MASK, 0x0e);
	if (err < 0) {
		printk_err("Write CS8427_REG_INT2MASK error!\n");
		goto __fail;
	}

	err = i2c_cs8427_write(CS8427_REG_INT2MODEMSB, 0x0);
	if (err < 0) {
		printk_err("Write CS8427_REG_INT2MODEMSB error!\n");
		goto __fail;
	}

	err = i2c_cs8427_write(CS8427_REG_INT2MODELSB, 0x0);
	if (err < 0) {
		printk_err("Write CS8427_REG_INT2MODELSB error!\n");
		goto __fail;
	}

	err = i2c_cs8427_write(CS8427_REG_RECVERRORS, 0x0);
	if (err < 0) {
		printk_err("Write CS8427_REG_RECVERRORS error!\n");
		goto __fail;
	}

	/* 3. send transfer initialization sequence */

	/* CS8427_REG_RECVERRMASK: unmask the input PLL clock, V, confidence, */
	/* biphase, parity status bits                                        */
	/* CS8427_UNLOCK | CS8427_V | CS8427_CONF | CS8427_BIP | CS8427_PAR.  */
	err = i2c_cs8427_write(CS8427_REG_RECVERRMASK, 0x7f);
	if (err < 0) {
		printk_err("Write CS8427_REG_RECVERRMASK error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_RECVERRMASK] = 0x7f;

	/*
	CS8427_REG_CSDATABUF:
	Registers 32-55 window to CS buffer
	Inhibit D->E transfers from overwriting first 5 bytes of CS data.
	Inhibit D->E transfers (all) of CS data.
	Allow E->F transfer of CS data.
	One byte mode; both A/B channels get same written CB data.
	A channel info is output to chip's EMPH* pin.
	*/
	err = i2c_cs8427_write(CS8427_REG_CSDATABUF,
		CS8427_CBMR | CS8427_DETCI);
	if (err < 0) {
		printk_err("Write CS8427_REG_CSDATABUF error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_CSDATABUF] = CS8427_CBMR | CS8427_DETCI;

	/*
	CS8427_REG_UDATABUF:
	Use internal buffer to transmit User (U) data.
	Chip's U pin is an output.
	Transmit all O's for user data.
	Inhibit D->E transfers.
	Inhibit E->F transfers.
	*/
	err = i2c_cs8427_write(CS8427_REG_UDATABUF, CS8427_UD
		| CS8427_EFTUI | CS8427_DETUI);
	if (err < 0) {
		printk_err("Write CS8427_REG_UDATABUF error!\n");
		goto __fail;
	}
	cs8427_chip->regmap[CS8427_REG_UDATABUF] = CS8427_UD
		| CS8427_EFTUI | CS8427_DETUI;

	/* write default channel status bytes */
	put_unaligned_le32(SNDRV_PCM_DEFAULT_CON_SPDIF, buf);
	memset(buf + 4, 0, 24 - 4);
	err = snd_cs8427_send_corudata(0, buf, 24);
	if (err < 0) {
		printk_err("Write default channel status bytes error!\n");
		goto __fail;
	}

	memcpy(cs8427_chip->playback.def_status, buf, 24);
	memcpy(cs8427_chip->playback.pcm_status, buf, 24);

	/* turn on run bit and rock'n'roll */
	if (reset_timeout < 1)
		reset_timeout = 1;

	cs8427_chip->reset_timeout = reset_timeout;
	snd_cs8427_reset();

	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;

__fail:
	kfree(cs8427_chip);
	cs8427_chip = NULL;
	return err < 0 ? err : -EIO;
}

/*
 * Reset the chip using run bit, also lock PLL using ILRCK and
 * put back AES3INPUT. This workaround is described in latest
 * CS8427 datasheet, otherwise TXDSERIAL will not work.
 */
static void snd_cs8427_reset(void)
{
	unsigned long end_time;
	int err;
	unsigned char data;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	i2c_cs8427_read(CS8427_REG_CLOCKSOURCE, &data);
	data |= CS8427_RUN | CS8427_RXDILRCK;
	i2c_cs8427_write(CS8427_REG_CLOCKSOURCE, data);
	cs8427_chip->regmap[CS8427_REG_CLOCKSOURCE] =  data;
	printk_dbg((_DEBUG_CS8427&0x04), "0x%08x\n",
		cs8427_chip->regmap[CS8427_REG_CLOCKSOURCE]);
	udelay(200);
	end_time = jiffies + cs8427_chip->reset_timeout;
	while (time_after_eq(end_time, jiffies)) {
		err = i2c_cs8427_read(CS8427_REG_RECVERRORS, &data);
		if (err < 0) {
			printk_err("Read CS8427_REG_RECVERRORS error.\n");
			break;
		}
		if (!(data & CS8427_UNLOCK))
			break;
		schedule_timeout_uninterruptible(1);
	}
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
}

static int snd_cs8427_in_status_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 255;
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_in_status_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned char data;
	int ret;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	ret = i2c_cs8427_read(kcontrol->private_value, &data);
	if (ret < 0)
		return ret;
	ucontrol->value.integer.value[0] = data;
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_qsubcode_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = 10;
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_qsubcode_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned char reg = CS8427_REG_QSUBCODE;
	int err;
	int i = 0;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	err = i2c_master_send(i2c_cs8427_client, &reg, 1);
	if (err < 0) {
		printk_err("unable to send register 0x%x byte " "to CS8427\n",
			reg);
		return err < 0 ? err : -EIO;
	}

	for (i = 0; i < 10; i++) {
		err = i2c_cs8427_read((CS8427_REG_QSUBCODE + (unsigned char)i),
			&ucontrol->value.bytes.data[i]);
		if (err < 0)
			printk_err("Read CS8427_REG_QSUBCODE error.\n");
	}
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_spdif_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_spdif_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	memcpy(ucontrol->value.iec958.status,
		cs8427_chip->playback.def_status, 24);
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_spdif_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	unsigned char *status;
	struct snd_pcm_runtime *runtime;
	int err, change;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	if (kcontrol->private_value)
		status = cs8427_chip->playback.pcm_status;
	else
		status = cs8427_chip->playback.def_status;

	if (cs8427_chip->playback.substream)
		runtime = cs8427_chip->playback.substream->runtime;
	else
		runtime = NULL;

	err = memcmp(ucontrol->value.iec958.status, status, 24);
	if (err != 0)
		change = 1;
	else
		change = 0;

	memcpy(status, ucontrol->value.iec958.status, 24);
	if (change &&
		(kcontrol->private_value ? runtime != NULL : runtime == NULL)) {
		err = snd_cs8427_send_corudata(0, status, 24);
		if (err < 0)
			change = err;
	}
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return change;
}

static int snd_cs8427_spdif_mask_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_spdif_mask_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	memset(ucontrol->value.iec958.status, 0xff, 24);
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static struct snd_kcontrol_new snd_cs8427_iec958_controls[] = {
{
	.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
	.info =		snd_cs8427_in_status_info,
	.name =		"IEC958 CS8427 Input Status",
	.access =	(SNDRV_CTL_ELEM_ACCESS_READ
		| SNDRV_CTL_ELEM_ACCESS_VOLATILE),
	.get =		snd_cs8427_in_status_get,
	.private_value = 15,
},
{
	.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
	.info =		snd_cs8427_in_status_info,
	.name =		"IEC958 CS8427 Error Status",
	.access =	(SNDRV_CTL_ELEM_ACCESS_READ
		| SNDRV_CTL_ELEM_ACCESS_VOLATILE),
	.get =		snd_cs8427_in_status_get,
	.private_value = 16,
},
{
	.access =	SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
	.name =		SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
	.info =		snd_cs8427_spdif_mask_info,
	.get =		snd_cs8427_spdif_mask_get,
},
{
	.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
	.name =		SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
	.info =		snd_cs8427_spdif_info,
	.get =		snd_cs8427_spdif_get,
	.put =		snd_cs8427_spdif_put,
	.private_value = 0
},
{
	.access =	(SNDRV_CTL_ELEM_ACCESS_READWRITE
		| SNDRV_CTL_ELEM_ACCESS_INACTIVE),
	.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
	.name =		SNDRV_CTL_NAME_IEC958("", PLAYBACK, PCM_STREAM),
	.info =		snd_cs8427_spdif_info,
	.get =		snd_cs8427_spdif_get,
	.put =		snd_cs8427_spdif_put,
	.private_value = 1
},
{
	.iface =	SNDRV_CTL_ELEM_IFACE_PCM,
	.info =		snd_cs8427_qsubcode_info,
	.name =		"IEC958 Q-subcode Capture Default",
	.access =	(SNDRV_CTL_ELEM_ACCESS_READ
		| SNDRV_CTL_ELEM_ACCESS_VOLATILE),
	.get =		snd_cs8427_qsubcode_get
}
};

static int snd_cs8427_spdif_build(struct snd_pcm_substream *play_substream,
		struct snd_pcm_substream *cap_substream)
{
	struct snd_kcontrol *kctl;
	unsigned int idx;
	int err;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");

	if (!play_substream || !cap_substream)
		return -EINVAL;
	for (idx = 0; idx < ARRAY_SIZE(snd_cs8427_iec958_controls); idx++) {
		kctl = snd_ctl_new1(&snd_cs8427_iec958_controls[idx], NULL);
		if (kctl == NULL)
			return -ENOMEM;
		kctl->id.device = play_substream->pcm->device;
		kctl->id.subdevice = play_substream->number;
		err = snd_ctl_add(emxx_cs8427_card, kctl);
		if (err < 0)
			return err;
		if (!strcmp(kctl->id.name,
		SNDRV_CTL_NAME_IEC958("", PLAYBACK, PCM_STREAM)))
			cs8427_chip->playback.pcm_ctl = kctl;
	}

	cs8427_chip->playback.substream = play_substream;
	cs8427_chip->capture.substream = cap_substream;
	if (!cs8427_chip->playback.pcm_ctl)
		return -EIO;

	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return 0;
}

static int snd_cs8427_spdif_pcm(unsigned int rate)
{
	char status;
	int err = 0;
#if 1	/* not necessity...? */
	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	i2c_cs8427_read(CS8427_REG_CORU_DATABUF, &status);
	if (status & IEC958_AES0_PROFESSIONAL) {
		status &= ~IEC958_AES0_PRO_FS;
		switch (rate) {
		case 32000:
			status |= IEC958_AES0_PRO_FS_32000;
			break;
		case 44100:
			status |= IEC958_AES0_PRO_FS_44100;
			break;
		case 48000:
			status |= IEC958_AES0_PRO_FS_48000;
			break;
		default:
			status |= IEC958_AES0_PRO_FS_NOTID;
			break;
		}
		i2c_cs8427_write(CS8427_REG_CORU_DATABUF, status);
		cs8427_chip->playback.pcm_status[0] = status;
	} else {
		i2c_cs8427_read(CS8427_REG_CORU_DATABUF+3, &status);
		status &= ~IEC958_AES3_CON_FS;
		switch (rate) {
		case 32000:
			status |= IEC958_AES3_CON_FS_32000;
			break;
		case 44100:
			status |= IEC958_AES3_CON_FS_44100;
			break;
		case 48000:
			status |= IEC958_AES3_CON_FS_48000;
			break;
		}
		i2c_cs8427_write(CS8427_REG_CORU_DATABUF+3, status);
		cs8427_chip->playback.pcm_status[3] = status;
	}
	err = snd_cs8427_send_corudata(0, cs8427_chip->playback.pcm_status, 24);
	if (err > 0) {
		snd_ctl_notify(emxx_cs8427_card,
				SNDRV_CTL_EVENT_MASK_VALUE,
				&cs8427_chip->playback.pcm_ctl->id);
	}
#endif
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return err;
}

static int __init emxx_cs8427_module_init(void)
{
	int err;

	printk_dbg((_DEBUG_CS8427&0x01), "Enter\n");
	err = platform_driver_register(&emxx_cs8427_driver);
	if (err < 0) {
		printk_err("Driver register error!\n");
		return err;
	}
	printk_dbg((_DEBUG_CS8427&0x02), "Leave\n");
	return err;
}

static void __exit emxx_cs8427_module_exit(void)
{
	free_irq(GPIO_DAIT_INT, NULL);
	kfree(cs8427_chip);
	cs8427_chip = NULL;
	platform_driver_unregister(&emxx_cs8427_driver);
}

module_init(emxx_cs8427_module_init);
module_exit(emxx_cs8427_module_exit);

MODULE_DESCRIPTION("IEC958 (S/PDIF) receiver & transmitter");
MODULE_LICENSE("GPL");
