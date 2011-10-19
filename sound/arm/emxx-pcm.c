/*
 *  File Name		: sound/arm/emxx-pcm.c
 *  Function		: PCM
 *  Release Version 	: Ver 1.08
 *  Release Date	: 2010/10/12
 *
 *  Copyright (c) 2010 Renesas Electronics Corporation
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY;
 *  without even the implied warrnty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program;
 *  If not, write to the Free Software Foundation, Inc., 59 Temple Place -
 *  Suite 330, Boston,
 *  MA 02111-1307, USA.
 *
 */

#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/hwdep.h>

#include <asm/irq.h>
#include <asm/dma.h>

#include <mach/gpio.h>
#include <mach/pm.h>
#include <mach/pwc.h>
#include <mach/pmu.h>
#include <mach/pcm_irq.h>

#include "emxx-pcm.h"
/* #define EMXX_PCM_PACK_PELIOD */

#ifdef EMXX_PCM_MMAP_ENABLE
#include <linux/dma-mapping.h>
#endif

#define PCM0_TX(s)	(s->dma_ch == EMXX_DMAC_M2P_SIO1)
#define PCM1_TX(s)	(s->dma_ch == EMXX_DMAC_M2P_SIO3)

#ifdef EMXX_PCM_USE_PDMA

#define IS_PCM0(p)	(p.irq == INT_SIO1)
#define IS_PCM1(p)	(p.irq == INT_SIO3)

#define NULLBUF_SIZE	128
#define SRAM_BASE	EMXX_SRAM_BASE
#define SRAM_PDMA_SIZE	((EMXX_SRAM_SIZE/2)-NULLBUF_SIZE)	/* 64k */

/****  Prototype definition  ****/
#define PDMA_BASE		IO_ADDRESS(EMXX_PDMA_BASE)

#define PDMA_DMA_SEL		(PDMA_BASE + 0x00)
#define PDMA_CONT		(PDMA_BASE + 0x04)
#define PDMA_STATUS		(PDMA_BASE + 0x08)
#define PDMA_RCV_CANCEL		(PDMA_BASE + 0x0C)
#define PDMA_END		(PDMA_BASE + 0x10)
#define PDMA_RSV_ADD		(PDMA_BASE + 0x20)
#define PDMA_RSV_LENG		(PDMA_BASE + 0x24)
#define PDMA_RUN_ADD		(PDMA_BASE + 0x28)
#define PDMA_RUN_LENG		(PDMA_BASE + 0x2C)
#define PDMA_INT_STATUS		(PDMA_BASE + 0x30)
#define PDMA_INT_RAW_STATUS	(PDMA_BASE + 0x34)
#define PDMA_INT_ENABLE		(PDMA_BASE + 0x38)
#define PDMA_INT_ENABLE_CL	(PDMA_BASE + 0x3C)
#define PDMA_INT_REQ_CL		(PDMA_BASE + 0x40)
#define PDMA_RUN_ADP		(PDMA_BASE + 0x50)
#define PDMA_HWORD_SWAP		(PDMA_BASE + 0x54)
#define PDMA_TMP		(PDMA_BASE + 0x58)

#endif

/* static init value */
#define ALSA_PDMA_CALLBACK_VALUE	NULL
#define ALSA_PDMA_DATA_VALUE		NULL
#define PDMA_INIT_VALUE			0
#define PDMA_IN_USE_VALUE		0
#define POWER_FLAG_VALUE		0

/*** debug code by the making ->*/
#ifdef AUDIO_MAKING_DEBUG
#include <linux/moduleparam.h>
#define FNC_ENTRY       \
	if (debug == 1 || debug >= 9) { \
		printk(KERN_INFO "entry:%s\n", __func__); \
	}
#define FNC_EXIT        \
	if (debug == 1 || debug >= 9) { \
		printk(KERN_INFO "exit:%s:%d\n", __func__ , __LINE__); \
	}
#define d0b(fmt, args...)       \
		printk(KERN_INFO "%s:%d: " fmt , __func__ , __LINE__ , ## args);
#define d1b(fmt, args...)       \
	if (debug == 1 || debug >= 9) { \
		printk(KERN_INFO "%s:%d: " fmt , \
		 __func__ , __LINE__ , ## args); \
	}
#define d2b(fmt, args...)       \
	if (debug == 2 || debug >= 9) { \
		printk(KERN_INFO "%s:%d: " fmt , \
		__func__ , __LINE__ , ## args); \
	}
#define d3b(fmt, args...)       \
	if (debug == 3 || debug >= 9) { \
		printk(KERN_INFO " --M%d-- "fmt , debug , ## args); \
	}
#define d4b(fmt, args...)       \
	if (debug == 4 || debug >= 9) { \
		printk(KERN_INFO " --M%d-- "fmt , debug , ## args); \
	}
#define d5b(fmt, args...)       \
	if (debug == 5 || debug >= 9) { \
		printk(KERN_INFO " --M%d-- "fmt , debug , ## args); \
	}
#define d6b(fmt, args...)       \
	if (debug == 6 || debug >= 9) { \
		printk(KERN_INFO " --M%d-- "fmt , debug , ## args); \
	}
int debug = 10;
EXPORT_SYMBOL(debug);
module_param(debug,  int, 0644);
#else
#define FNC_ENTRY
#define FNC_EXIT
#define d0b(fmt, args...)
#define d1b(fmt, args...)
#define d2b(fmt, args...)
#define d3b(fmt, args...)
#define d4b(fmt, args...)
#define d5b(fmt, args...)
#define d6b(fmt, args...)
#endif
/*<- debug code by the making ***/

/* Type definitions */

/* PMx_TXRX_EN Masks */
#define PCM_TX_EN	0x01
#define PCM_RX_EN	0x02

#define NULLBUF_SIZE	128

static int power_flag = POWER_FLAG_VALUE;
#ifndef EMXX_PCM_USE_PDMA
static unsigned char dma_nullbuf[NULLBUF_SIZE];
#endif

/*
 * Data for managing PCM channels
 */

struct emxx_pcm {
	char *name;		/* irq name. */
	u_int base;		/* base address */
	u_int clock_s;		/* SCLK */
	u_int clock_p;		/* PCLK */
	u_int clkctl;		/* CLKCTRL */
	u_int reset;		/* reset */
#ifdef EMXX_PCM_INTERRUPT_ENABLE
	u_int irq;		/* irq */
	int irq_enable;		/* enable/free irq flag */
	spinlock_t irq_lock;	/* enable/free irq lock */
	struct snd_pcm *pcm;	/* arg */
#endif
	u_int8_t in_use;	/* PCM in use flag. */
};

static struct emxx_pcm pcm_ch[EMXX_PCM_MAX_CHANNELS] = {
	{
		.name = "emxx-pcm0",
		.base = EMXX_SIO1_BASE + 0x2000,
		.clock_s = EMXX_CLK_USIA_S1_S,
		.clock_p = EMXX_CLK_USIA_S1_H | EMXX_CLK_USIA_S1_P,
		.clkctl = EMXX_CLKCTRL_USIAS1PCLK,
		.reset = EMXX_RST_USIA_S1_A | EMXX_RST_USIA_S1_S,
#ifdef EMXX_PCM_INTERRUPT_ENABLE
		.irq = INT_SIO1,
		.irq_enable = 0,
#endif
		.in_use = 0,
	},
	{
		.name = "emxx-pcm1",
		.base = EMXX_SIO3_BASE + 0x2000,
		.clock_s = EMXX_CLK_USIB_S3_S,
		.clock_p = EMXX_CLK_USIB_S3_H | EMXX_CLK_USIB_S3_P,
		.clkctl = EMXX_CLKCTRL_USIBS3PCLK,
		.reset = EMXX_RST_USIB_S3_A | EMXX_RST_USIB_S3_S,
#ifdef EMXX_PCM_INTERRUPT_ENABLE
		.irq = INT_SIO3,
		.irq_enable = 0,
#endif
		.in_use = 0,
	},
};

/*
 * prototypes
 */
static int emxx_pcm_hwdep_open(struct snd_hwdep *hw, struct file *file);
static int emxx_pcm_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
				 unsigned int cmd, unsigned long arg);
static int emxx_pcm_hwdep_release(struct snd_hwdep *hw, struct file *file);

static snd_pcm_uframes_t output_ptr;
static snd_pcm_uframes_t chk_appl_ptr;
static int last_flag;
static int wrap_flag;

/* PDMA Source */
#ifdef EMXX_PCM_USE_PDMA

typedef irqreturn_t (*pcm_callback_t)(int irq_no, void *data);

static pcm_callback_t alsa_pdma_callback = ALSA_PDMA_CALLBACK_VALUE;
static void *alsa_pdma_data = ALSA_PDMA_DATA_VALUE;
static int pdma_init = PDMA_INIT_VALUE;
static int pdma_in_use = PDMA_IN_USE_VALUE;

static void emxx_stop_pdma(void)
{
	writel(1, PDMA_END);
/*	emxx_pm_pdma_suspend_disable(); */
	pdma_init = 0;
}

static irqreturn_t pdma_callback(int irq, void *data)
{
	writel(1, PDMA_INT_REQ_CL);
	return alsa_pdma_callback(irq, alsa_pdma_data);
}

static void emxx_pdma_deinit(void)
{
	emxx_reset_device(EMXX_RST_PDMA);
	emxx_close_clockgate(EMXX_CLK_PDMA);
}

static void emxx_pdma_init(void)
{
	emxx_open_clockgate(EMXX_CLK_PDMA);
	emxx_unreset_device(EMXX_RST_PDMA);
	emxx_clkctrl_on(EMXX_CLKCTRL_PDMA);

	writel(0x01, PDMA_HWORD_SWAP);
}

static int emxx_request_pdma(const char *device_id, pcm_callback_t callback,
 void *data)
{
	int err;

	if (pdma_in_use == 1)
		return -EBUSY;

/*	emxx_clkctrl_off(EMXX_CLKCTRL_SRC); */
	err = emev_pdma_request_irq(pdma_callback, IRQF_DISABLED,
				device_id, data, EMEV_PCMMODE_ALSA);
	if (err != 0) {
		printk(KERN_INFO
		 "%s(): unable to request IRQ %d for DMA channel (%s)\n",
		 __func__, INT_PDMA, device_id);
		return err;
	}

	alsa_pdma_callback = callback;
	alsa_pdma_data = data;

	pdma_in_use = 1;

	writel(1, PDMA_DMA_SEL);
	writel(1, PDMA_INT_ENABLE);
	pdma_init = 1;

	return 0;
}

/*
 * Release DMA channel
 */
static void emxx_free_pdma(void)
{
	emxx_stop_pdma();
/*	emxx_clkctrl_on(EMXX_CLKCTRL_SRC); */

	emev_pdma_free_irq(alsa_pdma_data, EMEV_PCMMODE_ALSA);

	pdma_in_use = 0;
	alsa_pdma_callback = NULL;
	alsa_pdma_data = NULL;
}


/*
 * Setup and enable DMA.
 */
static int emxx_start_pdma(dma_addr_t src_ptr, u_int size)
{
	if (pdma_init == 0) {
		writel(1, PDMA_DMA_SEL);
		writel(1, PDMA_INT_ENABLE);
	}

/*	emxx_pm_pdma_suspend_enable(); */

	writel(src_ptr, PDMA_RSV_ADD);
	writel(size / 4, PDMA_RSV_LENG);

	writel(1, PDMA_CONT);

	return 0;
}

/*
 * Acquire DMA channel status.
 */
static int emxx_pdma_status(void)
{
	unsigned int status;

	status = readl(PDMA_STATUS);

	switch (status) {
	case 1:
	case 2:
		return 1;
	default:
		return 0;
	}
}

/*
 * Acquire DMA channel status (register writable flag).
 */
static int emxx_pdma_writable_flag(void)
{
	unsigned int status;

	status = readl(PDMA_STATUS);

	switch (status) {
	case 2:
		return 1;
	default:
		return 0;
	}
}

static void emxx_clear_pdma(void)
{
	emxx_stop_pdma();
}

dma_addr_t emxx_get_pdma_pos(void)
{
	unsigned int pos;
	unsigned int addr;
	unsigned int leng;

	pos = readl(PDMA_RUN_ADP);
	addr = readl(PDMA_RUN_ADD);
	leng = readl(PDMA_RUN_LENG) * 4;

	if ((addr <= pos) && (pos < (addr+leng)))
		;
	else
		pos = addr;

	return pos;
}

static irqreturn_t audio_pdma_callback(int irq, void *data);

#endif

/* DMA staff */

static int audio_dma_request(struct audio_stream *s,
 void (*callback)(void *, int, int))
{
	int ret;
	FNC_ENTRY

	output_ptr = 0;
	wrap_flag = 0;
	last_flag = 0;
	chk_appl_ptr = 0;

#ifdef EMXX_PCM_USE_PDMA
	if (PCM0_TX(s))
		ret = emxx_request_pdma(s->id, audio_pdma_callback, s);
	else {
		ret = emxx_request_dma(s->dma_ch, s->id,
		 (dma_callback_t)callback, s, &s->dma_regs);
	}
#else
	ret = emxx_request_dma(s->dma_ch, s->id, (dma_callback_t)callback, s,
	 &s->dma_regs);
#endif
	if (ret < 0)
		printk(KERN_ERR "unable to grab audio dma 0x%x\n", s->dma_ch);

	FNC_EXIT return ret;
}

static void audio_dma_free(struct audio_stream *s)
{
	FNC_ENTRY
#ifdef EMXX_PCM_USE_PDMA
	if (PCM0_TX(s))
		emxx_free_pdma();
	else
		emxx_free_dma(s->dma_ch);
#else
	emxx_free_dma(s->dma_ch);
#endif
	s->dma_regs = 0;
	FNC_EXIT return;
}

static u_int audio_get_dma_pos(struct audio_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int offset;
	unsigned int pos;
	unsigned long flags;
	dma_addr_t addr;
	FNC_ENTRY

	if (last_flag) {
		offset = frames_to_bytes(
			 runtime, runtime->period_size) * s->period;
	} else {
		spin_lock_irqsave(&s->dma_lock, flags);
#ifdef EMXX_PCM_USE_PDMA
		if (PCM0_TX(s)) {
			addr = emxx_get_pdma_pos();
			if (addr == 0)
				/* already stopped. */
				addr = runtime->dma_addr;
			else
				addr += EMXX_SRAM_BASE;
		} else {
			addr = emxx_get_dma_pos(s->dma_ch);
		}
#else
		addr = emxx_get_dma_pos(s->dma_ch);
#endif
		offset = addr - runtime->dma_addr;
		spin_unlock_irqrestore(&s->dma_lock, flags);
	}

	/* d6b("offset = %d : bit = %d\n", offset, runtime->frame_bits); */
	pos = bytes_to_frames(runtime, offset);
	if (pos >= runtime->buffer_size)
		pos = 0;

	/* d3b("offset = 0x%08x - 0x%08x\n", addr, runtime->dma_addr); */

	FNC_EXIT return pos;
}

/*
 * this stops the dma and clears the dma ptrs
 */
static void audio_stop_dma(struct audio_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	int loop_count = 0;
	unsigned int loop_limit;
	FNC_ENTRY

	d2b("PCM TXRX_DIS(0x%08x) : 0x%08x\n",
		(u_int)s->txrx_dis, s->txrx_id);

	d3b("%s stop dma ch : %d, pos : %d\n",
	 ((PCM_TX_EN == s->txrx_id) ? "TX" : "RX"), s->dma_ch,
	 audio_get_dma_pos(s));

	if (runtime->control->appl_ptr == output_ptr) {
		if (runtime->control->appl_ptr != chk_appl_ptr)
			goto _Next;

		loop_limit = (((frames_to_bytes(runtime, runtime->period_size)
			+ NULLBUF_SIZE)/4) * 10000) / runtime->rate;
		/* this case pcm last output now */
#ifdef EMXX_PCM_USE_PDMA
		if (PCM0_TX(s)) {
			while (0 != emxx_pdma_status()) {
				udelay(100);
				if (++loop_count == loop_limit)
					break;
			}
		} else {
			while (0 != emxx_dma_status(s->dma_ch)) {
				udelay(100);
				if (++loop_count == loop_limit)
					break;
			}
		}
#else
		while (0 != emxx_dma_status(s->dma_ch)) {
			udelay(100);
			if (++loop_count == loop_limit)
				break;
		}
#endif
	}
_Next:;

	spin_lock_irqsave(&s->dma_lock, flags);
	s->active = 0;
	s->period = 0;
	output_ptr = 0;
	wrap_flag = 0;
	last_flag = 0;
	/* this stops the dma channel and clears the buffer ptrs */
#ifdef EMXX_PCM_USE_PDMA
	if (PCM0_TX(s))
		emxx_clear_pdma();
	else
		emxx_clear_dma(s->dma_ch);
#else
	emxx_clear_dma(s->dma_ch);
#endif
	spin_unlock_irqrestore(&s->dma_lock, flags);

	*s->txrx_dis = s->txrx_id;

	FNC_EXIT return;
}


#define ERRMSG_CANNOT_QUEUE "audio_process_dma: cannot queue DMA buffer (%i)\n"
static void audio_process_dma(struct audio_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;
	unsigned int kick = 0;
	int ret;
	unsigned int true_dma_size = 0;
#ifdef EMXX_PCM_USE_PDMA
	unsigned int writable_flag = 0;
#endif
	FNC_ENTRY

#ifdef EMXX_PCM_USE_PDMA
	if (PCM0_TX(s))
		kick = emxx_pdma_status();
	else
		kick = emxx_dma_status(s->dma_ch);
#else
	kick = emxx_dma_status(s->dma_ch);
#endif
	s->add_period = 0;
	/* must be set here - only valid for running streams,
	   not for forced_clock dma fills  */
	runtime = substream->runtime;
	while (s->active && s->periods < runtime->periods) {
		if (!PCM0_TX(s) && !PCM1_TX(s)) {
			dma_size = frames_to_bytes(runtime,
			 runtime->period_size);
			s->add_period = 1;
		} else {
			if (runtime->control->appl_ptr < output_ptr) {
				/* boundary over */
				/*  output_ptr modify is after pcm output. */
				dma_size = runtime->boundary - output_ptr;
				dma_size += runtime->control->appl_ptr;
			} else {
				if (((output_ptr != 0) || wrap_flag) &&
				 (output_ptr == runtime->control->appl_ptr)) {
					/* new data not yet transfered
					 from user area. or data finished. */
					return;
				}
				dma_size = runtime->control->appl_ptr
				 - output_ptr;
			}

#ifdef EMXX_PCM_PACK_PELIOD
#ifdef EMXX_PCM_USE_PDMA
			if ((kick == 1) &&
				((runtime->status->state
				  == SNDRV_PCM_STATE_PREPARED)
				 || (runtime->status->state
				  == SNDRV_PCM_STATE_RUNNING))) {
				if (dma_size < runtime->buffer_size/2) {
					/* pdma if stop then not break? */
					break;
				}
			}
#endif
			if (dma_size >= (runtime->buffer_size/2)) {
				s->add_period =
				 (runtime->buffer_size/2)/runtime->period_size;
				if (s->add_period == 0)
					s->add_period = 1;

				dma_size = runtime->period_size * s->add_period;
			} else {
				s->add_period = (dma_size
				 + (runtime->period_size-1))
				 /runtime->period_size;
				if (s->add_period == 0)
					s->add_period = 1;

			}
#else
			if (dma_size > runtime->period_size)
				dma_size = runtime->period_size;
			s->add_period = 1;
#endif
			if (PCM1_TX(s)) {
				true_dma_size = dma_size;
				dma_size = runtime->period_size;
			}

			dma_size = frames_to_bytes(runtime, dma_size);
		}

		if (s->old_offset) {
			/* a little trick, we need resume from old position */
			offset = frames_to_bytes(runtime, s->old_offset - 1);
			s->old_offset = 0;
			s->periods = 0;
			s->period = offset / dma_size;
			offset %= dma_size;
			dma_size = dma_size - offset;
			if (!dma_size)
				continue;               /* special case */
		} else {
			offset = frames_to_bytes(
			 runtime, runtime->period_size) * s->period;
			/* snd_assert(dma_size <= DMA_BUF_SIZE, ); */
		}

#ifdef EMXX_PCM_USE_PDMA
		if (PCM0_TX(s))
			writable_flag = emxx_pdma_writable_flag();
		else
			writable_flag = emxx_dma_writable_flag(s->dma_ch);

		if (0 == writable_flag) {
#else
		if (0 == emxx_dma_writable_flag(s->dma_ch)) {
#endif
			dma_addr_t dma_ptr;
			dma_addr_t dst_ptr;
			if (s->txrx_id == PCM_TX_EN) {
				dma_ptr = runtime->dma_addr + offset;
				dst_ptr = s->phys_xq;
#ifdef EMXX_PCM_USE_PDMA
				if (!PCM0_TX(s)) {
					if (runtime->sample_bits == 24) {
						s->dma_regs->aoff =
						 EMXX_DMAC_OFFSET_SUB | 1;
						s->dma_regs->leng =
						 (dma_size * 4) / 3;
						s->dma_regs->asize = 4;
					} else {
						s->dma_regs->aoff = 0;
						s->dma_regs->asize = dma_size;
						s->dma_regs->leng = dma_size;
					}
					if (runtime->sample_bits == 16) {
						s->dma_regs->mode =
						 (EMXX_DMAC_AMODE_BIT32
						  | EMXX_DMAC_ENDI_R1032
						  | EMXX_DMAC_ENDI_W3210);
					} else {
						s->dma_regs->mode =
						 (EMXX_DMAC_AMODE_BIT32
						  | EMXX_DMAC_ENDI_R3210
						  | EMXX_DMAC_ENDI_W3210);
					}
				}
#else
				if (runtime->sample_bits == 24) {
					s->dma_regs->aoff =
					 EMXX_DMAC_OFFSET_SUB | 1;
					s->dma_regs->leng =
					 (dma_size * 4) / 3;
					s->dma_regs->asize = 4;
				} else {
					s->dma_regs->aoff = 0;
					s->dma_regs->asize = dma_size;
					s->dma_regs->leng = dma_size;
				}
				if (runtime->sample_bits == 16) {
					s->dma_regs->mode =
						(EMXX_DMAC_AMODE_BIT32
						 | EMXX_DMAC_ENDI_R1032
						 | EMXX_DMAC_ENDI_W3210);
				} else {
					s->dma_regs->mode =
						(EMXX_DMAC_AMODE_BIT32
						 | EMXX_DMAC_ENDI_R3210
						 | EMXX_DMAC_ENDI_W3210);
				}
#endif
			} else {
				dma_ptr = s->phys_xq;
				dst_ptr = runtime->dma_addr + offset;
				s->dma_regs->boff = 0;
				s->dma_regs->bsize = dma_size;
				s->dma_regs->leng = dma_size;
				s->dma_regs->mode = (EMXX_DMAC_AMODE_BIT32
						   | EMXX_DMAC_ENDI_R3210
						   | EMXX_DMAC_ENDI_W1032);
			}
			d1b("dma ch   : %d\n", s->dma_ch);
			d1b("dma_ptr  : 0x%08x\n", dma_ptr);
			d1b("dst_ptr  : 0x%08x\n", dst_ptr);
			d1b("dma_size : %d\n", dma_size);
			d1b("bk count : %d\n", runtime->periods);

			if (0 == kick) {
				if (PCM0_TX(s) || PCM1_TX(s)) {
					chip->pcm_regs->txq = 0;
					*s->txrx_en = s->txrx_id;
					chip->pcm_regs->txq = 0;
					chip->pcm_regs->txq = 0;
				} else
					*s->txrx_en = s->txrx_id;

				d2b("PCM TXRX_EN(0x%08x) : 0x%08x\n",
					(u_int)s->txrx_en, s->txrx_id);
				udelay(50);     /* LRCLK1 22u: x2+a = 50 */
				kick = 1;
			}
#ifdef EMXX_PCM_USE_PDMA
			if (PCM0_TX(s)) {
				ret = emxx_start_pdma(dma_ptr, dma_size);
			} else {
				ret = emxx_start_dma(s->dma_ch,
						      dma_ptr,
						      0,
						      dst_ptr,
						      EMXX_DMAC_INT_LENG_EN);
			}
#else
			ret = emxx_start_dma(s->dma_ch,
					      dma_ptr,
					      0,
					      dst_ptr,
					      EMXX_DMAC_INT_LENG_EN);
#endif
			if (ret) {
				s->add_period = 0;
				printk(KERN_ERR ERRMSG_CANNOT_QUEUE, ret);
				FNC_EXIT return;
			}

			if (PCM1_TX(s)) {
				dma_size = frames_to_bytes(runtime,
				 true_dma_size);
			}
			if (PCM0_TX(s) || PCM1_TX(s)) {
				output_ptr += bytes_to_frames(runtime,
				 dma_size);
				if (output_ptr >= runtime->boundary) {
					output_ptr -= runtime->boundary;
					wrap_flag = 1;
				}
			}

		} else {
			s->add_period = 0;
			break;
		}

		d3b("%s%02d\n", ((PCM_TX_EN == s->txrx_id) ? "t" : "r"),
		 s->period);
		d4b("dma ptr : 0x%08x\n", runtime->dma_addr + offset);

		s->period += s->add_period;
		s->period %= runtime->periods;
		s->periods += s->add_period;
	}

	FNC_EXIT return;
}


#define ERRMSG_CANNOT_QUEUE2 "pcm_output_silence: cannot queue DMA buffer.\n"

static int pcm_output_silence(struct audio_stream *s)
{
	unsigned int dma_size;
	dma_addr_t dma_ptr;
	dma_addr_t dst_ptr;
	int ret;

	/* output silece data */
#ifdef EMXX_PCM_USE_PDMA
	dma_ptr = SRAM_BASE + SRAM_PDMA_SIZE;
#else
	dma_ptr = (dma_addr_t)virt_to_phys(dma_nullbuf);
#endif
	dst_ptr = s->phys_xq;
	dma_size = NULLBUF_SIZE;/* byte */

#ifdef EMXX_PCM_USE_PDMA
	if (PCM0_TX(s)) {
		/* 0: already stop... set last.
		   1: transfer
		   2: not writable */
		ret = readl(PDMA_STATUS);
		if (ret == 1)
			ret = emxx_start_pdma(dma_ptr, dma_size);
		else if (ret == 2)
			return -EBUSY;

		/*ret == 0 then ret = 0 */
	} else {
		if (0 != emxx_dma_writable_flag(s->dma_ch))
			return -EBUSY;
		s->dma_regs->aoff = 0;
		s->dma_regs->asize = NULLBUF_SIZE;
		s->dma_regs->leng = NULLBUF_SIZE;

		ret = emxx_start_dma(s->dma_ch,
				      dma_ptr,
				      0,
				      dst_ptr,
				      EMXX_DMAC_INT_LENG_EN);
	}
#else
	if (0 != emxx_dma_writable_flag(s->dma_ch)) {
		/* printk(KERN_ERR ERRMSG_CANNOT_QUEUE2); */
		return -EBUSY;
	}
	s->dma_regs->aoff = 0;
	s->dma_regs->asize = NULLBUF_SIZE;
	s->dma_regs->leng = NULLBUF_SIZE;

	ret = emxx_start_dma(s->dma_ch,
			      dma_ptr,
			      0,
			      dst_ptr,
			      EMXX_DMAC_INT_LENG_EN);
#endif

	if (ret) {
		s->add_period = 0;
		printk(KERN_ERR ERRMSG_CANNOT_QUEUE2);
		return -EBUSY;
	}
	last_flag = 1;

	return ret;
}

static void audio_dma_callback(void *data, int intsts, int intrawsts)
{
	struct audio_stream *s = data;
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int out_flag = 0;
	FNC_ENTRY

	if (PCM0_TX(s) || PCM1_TX(s)) {
		if (runtime->control->appl_ptr == output_ptr)
			out_flag = 1;
	}

	/*
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active) {
		/* maybe clear "output_ptr = 0" in dma_stop() */
		snd_pcm_period_elapsed(s->stream);
		d6b("pos : %d\n", audio_get_dma_pos(s));
		d3b("pos : %d\n", audio_get_dma_pos(s));
	}

	if (!s->tx_spin && s->periods > 0)
		s->periods -= s->add_period;

	if (PCM0_TX(s) || PCM1_TX(s)) {
		if ((s->active != 0) &&
			 ((out_flag == 1)
			  || (runtime->control->appl_ptr == output_ptr))) {
			chk_appl_ptr = runtime->control->appl_ptr;
			pcm_output_silence(s);
		} else
			audio_process_dma(s);

	} else
		audio_process_dma(s);

	FNC_EXIT return;
}

#ifdef EMXX_PCM_USE_PDMA
static irqreturn_t audio_pdma_callback(int irq, void *data)
{
	struct audio_stream *s = data;
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int out_flag = 0;
	FNC_ENTRY

	if (PCM0_TX(s) || PCM1_TX(s)) {
		if (runtime->control->appl_ptr == output_ptr)
			out_flag = 1;
	}

	/*
	 * If we are getting a callback for an active stream then we inform
	 * the PCM middle layer we've finished a period
	 */
	if (s->active) {
		snd_pcm_period_elapsed(s->stream);
		d6b("pos : %d\n", audio_get_dma_pos(s));
		d3b("pos : %d\n", audio_get_dma_pos(s));
	}

	if (output_ptr != runtime->control->appl_ptr) {
		if (!s->tx_spin && s->periods > 0)
			s->periods -= s->add_period;
		audio_process_dma(s);
	}

	if (PCM0_TX(s) || PCM1_TX(s)) {
		if ((s->active != 0) &&
			 ((out_flag == 1)
			  || (runtime->control->appl_ptr == output_ptr))) {
			chk_appl_ptr = runtime->control->appl_ptr;
			pcm_output_silence(s);
		}
	}

	FNC_EXIT return IRQ_HANDLED;
}
#endif

/* PCM setting */

/* HW params & free */

static const struct snd_pcm_hardware emxx_pcm_hardware = {
	.info                   = (SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_MMAP_VALID |
				   SNDRV_PCM_INFO_PAUSE |
				   SNDRV_PCM_INFO_RESUME),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,

#ifdef EMXX_PCM_USE_PDMA
	.buffer_bytes_max	= 64 * 1024 - NULLBUF_SIZE,
	.period_bytes_min	= 64,
	.period_bytes_max	= SRAM_PDMA_SIZE/2,
	.periods_min		= 2,
	.periods_max		= 32,
#else
	.buffer_bytes_max       = 0xFFFE * 16,
	.period_bytes_min       = 64,
	.period_bytes_max       = 0xFFFE,
	.periods_min            = 2,
	.periods_max            = 16,
#endif
	.fifo_size              = 32,
};

#ifdef EMXX_PCM_INTERRUPT_ENABLE

#include <linux/interrupt.h>

/* PMx_INT Masks */
#define PCM_INT_TX_FRE          0x01
#define PCM_INT_TX_URE          0x02
#define PCM_INT_TX_ORE          0x04
#define PCM_INT_TX_WEN          0x08
#define PCM_INT_RX_FRE          0x10
#define PCM_INT_RX_URE          0x20
#define PCM_INT_RX_ORE          0x40
#define PCM_INT_RX_REN          0x80
#define PCM_INT_TX_STP          (1 << 8)
#define PCM_INT_RX_STP          (1 << 12)

static void emxx_pcm_kick(unsigned long inData)
{
	struct audio_stream *s = (struct audio_stream *)inData;
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(s->stream);
	int wait, mode, val, status;
	FNC_ENTRY

	mode = *s->func_sel &  (EMXX_PCM_FUNC_MODE0 |
				EMXX_PCM_FUNC_MODE1 |
				EMXX_PCM_FUNC_MODE2 |
				EMXX_PCM_FUNC_MODE3 |
				EMXX_PCM_FUNC_MODE4 |
				EMXX_PCM_FUNC_MODE5 |
				EMXX_PCM_FUNC_MODE6);

	switch (mode) {
	case EMXX_PCM_FUNC_MODE5:
	case EMXX_PCM_FUNC_MODE6:
		val = (((chip->pcm_regs->cycle & 0xff) *
			(((chip->pcm_regs->cycle >> 8) & 0x1f) + 1)) +
		       ((chip->pcm_regs->cycle2 & 0xff) *
			(((chip->pcm_regs->cycle >> 8) & 0x1f) + 1)));
		break;
	/* mode0-4 */
	default:
		val = (chip->pcm_regs->cycle & 0xff) + 1;
		break;
	}

	wait = (((1000000 / s->stream->runtime->rate) * val) +
		((1000000 % s->stream->runtime->rate) ? 1 : 0));

	udelay(wait);

#ifdef EMXX_PCM_USE_PDMA
	if (PCM0_TX(s)) {
		if (s->stream->runtime->status->state
		 == SNDRV_PCM_STATE_RUNNING) {
			status = 1;
		} else
			status = 0;

	} else {
		status = emxx_dma_status(s->dma_ch);
	}
#else
	status = emxx_dma_status(s->dma_ch);
#endif
	if (1 == status) {
		*s->txrx_en = s->txrx_id;
	} else {
		unsigned long flags;

		local_irq_save(flags);
		audio_stop_dma(s);
		audio_process_dma(s);
		local_irq_restore(flags);
	}

	FNC_EXIT return;
}

DECLARE_TASKLET(emxx_pcm_tx_tasklet, emxx_pcm_kick, 0);
DECLARE_TASKLET(emxx_pcm_rx_tasklet, emxx_pcm_kick, 0);

static irqreturn_t audio_pcm_callback(int irq, void *dev_id)
{
	struct snd_pcm *pcm = (struct snd_pcm *) dev_id;
	struct emxx_pcm_client *chip = pcm->private_data;
	static irqreturn_t ret = IRQ_NONE;
	FNC_ENTRY

	d2b("PCM STATUS(0x%08x) : 0x%08x\n",
		(u_int)&chip->pcm_regs->status, chip->pcm_regs->status);

	if (chip->pcm_regs->status & ~(PCM_INT_TX_FRE | PCM_INT_TX_URE |
				       PCM_INT_TX_ORE | PCM_INT_RX_FRE |
				       PCM_INT_RX_URE | PCM_INT_RX_ORE)) {

		chip->pcm_regs->clear = chip->pcm_regs->status;
		chip->pcm_regs->txrx_dis = PCM_TX_EN | PCM_RX_EN;

		printk(KERN_INFO "%s(): unable to pcm status 0x%08x\n",
		       __func__, chip->pcm_regs->status);
		return ret;
	}

	if (chip->pcm_regs->status & (PCM_INT_TX_FRE | PCM_INT_TX_URE
	 | PCM_INT_TX_ORE)) {

		chip->pcm_regs->clear = PCM_INT_TX_FRE | PCM_INT_TX_URE |
					PCM_INT_TX_STP;
		chip->pcm_regs->txrx_dis = PCM_TX_EN;
		d2b("PCM TXRX_EN(0x%08x) : 0x%08x\n",
		 (u_int)&chip->pcm_regs->txrx_en, chip->pcm_regs->txrx_en);

		if (chip->s[SNDRV_PCM_STREAM_PLAYBACK]) {
			struct audio_stream *s;

			s = chip->s[SNDRV_PCM_STREAM_PLAYBACK];
			if (s->active) {
				emxx_pcm_tx_tasklet.data = (unsigned long)s;
				tasklet_schedule(&emxx_pcm_tx_tasklet);
			}
			ret = IRQ_HANDLED;
		} else {
			printk(KERN_INFO "%s(): unable to pcm tx status 0x%08x\n",
			       __func__, chip->pcm_regs->status);
			return ret;
		}
	}
	if (chip->pcm_regs->status & (PCM_INT_RX_FRE | PCM_INT_RX_URE
	 | PCM_INT_RX_ORE)) {

		chip->pcm_regs->clear = PCM_INT_RX_FRE | PCM_INT_RX_STP;
		chip->pcm_regs->txrx_dis = PCM_RX_EN;
		d2b("PCM TXRX_EN(0x%08x) : 0x%08x\n",
			(u_int)&chip->pcm_regs->txrx_en,
			chip->pcm_regs->txrx_en);

		if (chip->s[SNDRV_PCM_STREAM_CAPTURE]) {
			struct audio_stream *s;

			s = chip->s[SNDRV_PCM_STREAM_CAPTURE];
			if (s->active) {
				emxx_pcm_rx_tasklet.data = (unsigned long)s;
				tasklet_schedule(&emxx_pcm_rx_tasklet);
			}
			ret = IRQ_HANDLED;
		} else {
			printk(KERN_INFO "%s(): unable to pcm rx status 0x%08x\n",
			       __func__, chip->pcm_regs->status);
			return ret;
		}
	}

	FNC_EXIT return ret;
}
#endif

static int emxx_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
#ifdef EMXX_PCM_USE_PDMA
	int ret = 0;
	FNC_ENTRY

	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	if (PCM0_TX(chip->s[substream->pstr->stream])) {
		struct snd_pcm_runtime *runtime = substream->runtime;

		snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
		runtime->dma_bytes = params_buffer_bytes(hw_params);
	} else {
		ret = snd_pcm_lib_malloc_pages(substream,
					       params_buffer_bytes(hw_params));
	}
	FNC_EXIT return ret;
#else
	int ret;
	FNC_ENTRY
	ret = snd_pcm_lib_malloc_pages(substream,
	 params_buffer_bytes(hw_params));
	FNC_EXIT return ret;
#endif
}

static int emxx_pcm_hw_free(struct snd_pcm_substream *substream)
{
#ifdef EMXX_PCM_USE_PDMA
	int ret = 0;
	FNC_ENTRY

	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	if (PCM0_TX(chip->s[substream->pstr->stream]))
		snd_pcm_set_runtime_buffer(substream, NULL);
	else
		ret = snd_pcm_lib_free_pages(substream);
	FNC_EXIT return ret;
#else
	int ret;
	FNC_ENTRY
	ret = snd_pcm_lib_free_pages(substream);
	FNC_EXIT return ret;
#endif
}

/* trigger & timer */

static int emxx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	int stream_id = substream->pstr->stream;
	struct audio_stream *s = chip->s[stream_id];
	int err = 0;
	FNC_ENTRY

	/* note local interrupts are already disabled in the midlevel code */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		d1b("SNDRV_PCM_TRIGGER_START\n");
		/* requested stream startup */
		s->active = 1;
		audio_process_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		d1b("SNDRV_PCM_TRIGGER_STOP\n");
		/* requested stream shutdown */
		audio_stop_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		d1b("SNDRV_PCM_TRIGGER_SUSPEND\n");
		s->active = 0;

#ifdef EMXX_PCM_USE_PDMA
		if (PCM0_TX(s))
			emxx_stop_pdma();
		else
			emxx_stop_dma(s->dma_ch);
#else
		emxx_stop_dma(s->dma_ch);
#endif
		s->old_offset = audio_get_dma_pos(s) + 1;
		s->periods = 0;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		d1b("SNDRV_PCM_TRIGGER_RESUME\n");
		s->active = 1;
		s->tx_spin = 0;
		audio_process_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		d1b("SNDRV_PCM_TRIGGER_PAUSE_PUSH\n");
#ifdef EMXX_PCM_USE_PDMA
		if (PCM0_TX(s))
			emxx_stop_pdma();
		else
			emxx_stop_dma(s->dma_ch);
#else
		emxx_stop_dma(s->dma_ch);
#endif
		s->active = 0;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		d1b("SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n");
		s->active = 1;
		if (s->old_offset) {
			s->tx_spin = 0;
			audio_process_dma(s);
			break;
		}
		break;
	default:
		err = -EINVAL;
		break;
	}
	FNC_EXIT return err;
}

static int emxx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	struct audio_stream *s = chip->s[substream->pstr->stream];
	FNC_ENTRY

	s->period = 0;
	s->periods = 0;

	/* set requested samplerate */
	FNC_EXIT return chip->prepare(substream);
}

static snd_pcm_uframes_t emxx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	struct audio_stream *s = chip->s[substream->pstr->stream];
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t ret;
	FNC_ENTRY

	spin_lock(&s->dma_lock);
	if (output_ptr != runtime->control->appl_ptr) {
		if (!s->tx_spin && s->periods > 0)
			s->periods -= s->add_period;
		audio_process_dma(s);
	}
	spin_unlock(&s->dma_lock);

	ret = audio_get_dma_pos(chip->s[substream->pstr->stream]);
	FNC_EXIT return ret;
}

#ifdef EMXX_PCM_MMAP_ENABLE
static int emxx_pcm_mmap(struct snd_pcm_substream *substream,
 struct vm_area_struct *vma)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
#ifdef EMXX_PCM_USE_PDMA
	struct emxx_pcm_client *chip;
#endif
	FNC_ENTRY

#ifdef EMXX_PCM_USE_PDMA
	chip = snd_pcm_substream_chip(substream);

	if (PCM0_TX(chip->s[substream->pstr->stream])) {
		unsigned long user_size;
		user_size = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;

		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		ret = remap_pfn_range(vma, vma->vm_start,
		 (runtime->dma_addr >> PAGE_SHIFT) + vma->vm_pgoff,
		 runtime->dma_bytes, vma->vm_page_prot);
	} else {
		ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
					    runtime->dma_area,
					    runtime->dma_addr,
					    runtime->dma_bytes);
	}
#else
	ret = dma_mmap_writecombine(substream->pcm->card->dev, vma,
				    runtime->dma_area,
				    runtime->dma_addr,
				    runtime->dma_bytes);
#endif

	FNC_EXIT return ret;
}
#endif

static int emxx_pcm_open(struct snd_pcm_substream *substream)
{
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	int ret;
	unsigned long flags;
	FNC_ENTRY

#ifdef CONFIG_EMXX_ANDROID
	{
		extern int codec_power_on(void);
		codec_power_on();
	}
#endif

#ifdef EMXX_PCM_USE_PDMA
	if (IS_PCM0(pcm_ch[chip->pcm_ch])) {
		if (PCM0_TX(chip->s[substream->pstr->stream])) {
			emxx_pdma_deinit();
			emxx_pdma_init();
		} else {
			writel(0, PDMA_DMA_SEL);
		}
	}
	memset((char *)(EMXX_SRAM_VIRT) + SRAM_PDMA_SIZE, 0, NULLBUF_SIZE);
#else
	memset(dma_nullbuf, 0, NULLBUF_SIZE);
#endif
	chip->s[substream->pstr->stream]->stream = substream;

	substream->runtime->hw = emxx_pcm_hardware;

	local_irq_save(flags);
	power_flag++;
	local_irq_restore(flags);

	ret = chip->startup(substream);
	if (ret < 0)
		goto err1;

	ret = audio_dma_request(chip->s[substream->pstr->stream],
				audio_dma_callback);
	if (ret < 0)
		goto err2;

#ifdef EMXX_PCM_INTERRUPT_ENABLE
	if (chip->s[substream->pstr->stream]->txrx_id == PCM_TX_EN) {
		chip->pcm_regs->enset = PCM_INT_TX_FRE | PCM_INT_TX_URE
		 | PCM_INT_TX_ORE;
	} else {
		chip->pcm_regs->enset = PCM_INT_RX_FRE | PCM_INT_RX_URE
		 | PCM_INT_RX_ORE;
	}
	spin_lock_irqsave(&pcm_ch[chip->pcm_ch].irq_lock, flags);
	if ((pcm_ch[chip->pcm_ch].irq_enable)++ == 0)
		enable_irq(pcm_ch[chip->pcm_ch].irq);
	spin_unlock_irqrestore(&pcm_ch[chip->pcm_ch].irq_lock, flags);
#endif

	FNC_EXIT return ret;
err2:
	chip->shutdown(substream);
err1:
	local_irq_save(flags);
	power_flag--;
	local_irq_restore(flags);

	chip->s[substream->pstr->stream]->stream = NULL;

	FNC_EXIT return ret;
}

static int emxx_pcm_close(struct snd_pcm_substream *substream)
{
	struct emxx_pcm_client *chip = snd_pcm_substream_chip(substream);
	unsigned long flags;
	FNC_ENTRY

	audio_dma_free(chip->s[substream->pstr->stream]);

#ifdef EMXX_PCM_INTERRUPT_ENABLE
	spin_lock_irqsave(&pcm_ch[chip->pcm_ch].irq_lock, flags);
	if (--(pcm_ch[chip->pcm_ch].irq_enable) == 0)
		disable_irq(pcm_ch[chip->pcm_ch].irq);
	spin_unlock_irqrestore(&pcm_ch[chip->pcm_ch].irq_lock, flags);
#ifdef EMXX_PCM_USE_PDMA
	if (IS_PCM0(pcm_ch[chip->pcm_ch])) {
		emxx_pdma_deinit();
		emxx_pdma_init();
	}
#endif
	if (chip->s[substream->pstr->stream]->txrx_id == PCM_TX_EN) {
		chip->pcm_regs->enclr = PCM_INT_TX_FRE | PCM_INT_TX_URE |
					PCM_INT_TX_ORE | PCM_INT_TX_WEN |
					PCM_INT_TX_STP;
		chip->pcm_regs->clear = PCM_INT_TX_FRE | PCM_INT_TX_URE |
					PCM_INT_TX_STP;
	} else {
		chip->pcm_regs->enclr = PCM_INT_RX_FRE | PCM_INT_RX_URE |
					PCM_INT_RX_ORE | PCM_INT_RX_REN |
					PCM_INT_RX_STP;
		chip->pcm_regs->clear = PCM_INT_RX_FRE | PCM_INT_RX_STP;
	}
	d2b("PCM TXRX_EN(0x%08x) : 0x%08x\n",
		(u_int)&chip->pcm_regs->txrx_en, chip->pcm_regs->txrx_en);
#endif

	chip->shutdown(substream);

	local_irq_save(flags);
	power_flag--;
	local_irq_restore(flags);

	chip->s[substream->pstr->stream]->stream = NULL;

	FNC_EXIT return 0;
}

static struct snd_pcm_ops emxx_pcm_ops = {
	.open           = emxx_pcm_open,
	.close          = emxx_pcm_close,
	.ioctl          = snd_pcm_lib_ioctl,
	.hw_params      = emxx_pcm_hw_params,
	.hw_free        = emxx_pcm_hw_free,
	.prepare        = emxx_pcm_prepare,
	.trigger        = emxx_pcm_trigger,
	.pointer        = emxx_pcm_pointer,
#ifdef EMXX_PCM_MMAP_ENABLE
	.mmap           = emxx_pcm_mmap,
#endif
};

#ifdef CONFIG_PM
int emxx_pcm_suspend(struct platform_device *dev, pm_message_t state)
{
#ifndef EMXX_PCM_USE_PDMA
	struct snd_card *card = platform_get_drvdata(dev);
#endif
	FNC_ENTRY

	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (power_flag != 0) {
#ifdef EMXX_PCM_USE_PDMA
			break;
#else
			if (((struct pcm_regs *)
				IO_ADDRESS(pcm_ch[card->number].base))->
					txrx_en == 0)
				break;
#endif
			printk(KERN_INFO "pcm busy\n");
			FNC_EXIT return -EBUSY;
		}
		break;
	default:
		break;
	}

	FNC_EXIT return 0;
}
EXPORT_SYMBOL(emxx_pcm_suspend);

int emxx_pcm_resume(struct platform_device *dev)
{
	FNC_ENTRY

	FNC_EXIT return 0;
}
EXPORT_SYMBOL(emxx_pcm_resume);
#endif

int emxx_pcm_free(struct emxx_pcm_client *client)
{
	int i;
	FNC_ENTRY

	if (pcm_ch[client->pcm_ch].in_use) {
#ifdef EMXX_PCM_INTERRUPT_ENABLE
#ifdef EMXX_PCM_USE_PDMA
		if (IS_PCM0(pcm_ch[client->pcm_ch]))
			emev_pcm0_free_irq((void *)pcm_ch[client->pcm_ch].pcm,
							EMEV_PCMMODE_ALSA);
		else
#endif
		free_irq(pcm_ch[client->pcm_ch].irq,
			 (void *)pcm_ch[client->pcm_ch].pcm);
#endif
		emxx_clkctrl_off(pcm_ch[client->pcm_ch].clkctl);
		emxx_reset_device(pcm_ch[client->pcm_ch].reset);
		d2b("emxx_pmu_reset_device(0x%08x)\n",
				pcm_ch[client->pcm_ch].reset);
		emxx_close_clockgate(pcm_ch[client->pcm_ch].clock_s);
		emxx_close_clockgate(pcm_ch[client->pcm_ch].clock_p);
		pcm_ch[client->pcm_ch].in_use = 0;
	}

	for (i = 0; i < EMXX_PCM_MAX_CHANNELS; i++) {
		if (pcm_ch[i].in_use)
			break;
	}

	FNC_EXIT return 0;
}
EXPORT_SYMBOL(emxx_pcm_free);


#ifdef EMXX_PCM_USE_PDMA
static int emxx_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream,
					    unsigned long addr, size_t size)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	buf->addr = addr;
	buf->area = ioremap_wc(buf->addr, size);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;

	return 0;
}
#endif

int emxx_pcm_new(struct snd_card *card, struct emxx_pcm_client *client,
 struct snd_pcm **rpcm)
{
	struct snd_hwdep *hw;
	struct snd_pcm *pcm;
	int play = client->s[SNDRV_PCM_STREAM_PLAYBACK] ? 1 : 0;
	int capt = client->s[SNDRV_PCM_STREAM_CAPTURE] ? 1 : 0;
	int ret = -EBUSY;
	FNC_ENTRY

	d2b("play : %d, capt : %d\n", play, capt);

	if (pcm_ch[client->pcm_ch].in_use)
		goto out;

	ret = snd_hwdep_new(card, pcm_ch[client->pcm_ch].name, client->pcm_ch,
	 &hw);

	if (ret)
		goto out;

	hw->private_data = client;

	hw->ops.open = emxx_pcm_hwdep_open;
	hw->ops.ioctl = emxx_pcm_hwdep_ioctl;
	hw->ops.release = emxx_pcm_hwdep_release;

	ret = snd_pcm_new(card, pcm_ch[client->pcm_ch].name, client->pcm_ch,
	 play, capt, &pcm);

	if (ret)
		goto out;

#ifdef EMXX_PCM_USE_PDMA
	if (!IS_PCM0(pcm_ch[client->pcm_ch])) {
		snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
		 NULL, 128 * 1024, 128 * 1024);
	}
#else
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
					      /* snd_pcm_dma_flags(0)*/ NULL,
					      128 * 1024, 128 * 1024);
#endif
	pcm->private_data = client;

	client->pcm_regs = (struct pcm_regs *)IO_ADDRESS(
	 pcm_ch[client->pcm_ch].base);

	d2b("pcm base : 0x%08x\n", pcm_ch[client->pcm_ch].base);
	d2b("pcm reg  : 0x%08x\n", IO_ADDRESS(pcm_ch[client->pcm_ch].base));
	d2b("pcm txq  : 0x%08x\n",
	 (dma_addr_t)&((struct pcm_regs *)pcm_ch[client->pcm_ch].base)->txq);

	if (play) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;
		struct snd_pcm_substream *substream;
		struct audio_stream *s = client->s[stream];

		snd_pcm_set_ops(pcm, stream, &emxx_pcm_ops);
		substream = pcm->streams[stream].substream;
#ifdef EMXX_PCM_USE_PDMA
		if (IS_PCM0(pcm_ch[client->pcm_ch])) {
			/* BANK 10 -- SRAM */
			ret = emxx_pcm_preallocate_dma_buffer(pcm, stream,
			 SRAM_BASE, SRAM_PDMA_SIZE);
			if (ret)
				goto out;
		}
#endif
		strncpy(substream->name, client->s[stream]->id,
		 sizeof(substream->name));
		d2b("name : %s \n", substream->name);
		s->phys_xq = (dma_addr_t)&(
		 (struct pcm_regs *)pcm_ch[client->pcm_ch].base)->txq;
		s->txrx_id = PCM_TX_EN;
		s->txrx_en = &client->pcm_regs->txrx_en;
		s->txrx_dis = &client->pcm_regs->txrx_dis;
		s->func_sel = &client->pcm_regs->func_sel;
		spin_lock_init(&s->dma_lock);
	}
	if (capt) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;
		struct snd_pcm_substream *substream;
		struct audio_stream *s = client->s[stream];

		snd_pcm_set_ops(pcm, stream, &emxx_pcm_ops);
		substream = pcm->streams[stream].substream;
#ifdef EMXX_PCM_USE_PDMA
		if (IS_PCM0(pcm_ch[client->pcm_ch])) {
			ret = snd_pcm_lib_preallocate_pages(substream,
			 SNDRV_DMA_TYPE_DEV, NULL, 64 * 1024, 64 * 1024);
			if (ret)
				goto out;
		}
#endif
		strncpy(substream->name, client->s[stream]->id,
		 sizeof(substream->name));
		d2b("name : %s \n", substream->name);
		s->phys_xq = (dma_addr_t)&(
		 (struct pcm_regs *)pcm_ch[client->pcm_ch].base)->rxq;
		s->txrx_id = PCM_RX_EN;
		s->txrx_en = &client->pcm_regs->txrx_en;
		s->txrx_dis = &client->pcm_regs->txrx_dis;
		s->func_sel = &client->pcm_regs->func_sel;
		spin_lock_init(&s->dma_lock);
	}

	if (rpcm)
		*rpcm = pcm;

	d2b("emxx_pmu_unreset_device(0x%08x)\n", pcm_ch[client->pcm_ch].reset);
	emxx_open_clockgate(pcm_ch[client->pcm_ch].clock_p);
	emxx_open_clockgate(pcm_ch[client->pcm_ch].clock_s);
	emxx_clkctrl_off(pcm_ch[client->pcm_ch].clkctl);
	emxx_unreset_device(pcm_ch[client->pcm_ch].reset);
	emxx_clkctrl_on(pcm_ch[client->pcm_ch].clkctl);
	if (client->sett->func.m_s == PCM_SLAVE_MODE)
		emxx_close_clockgate(pcm_ch[client->pcm_ch].clock_s);

#ifdef EMXX_PCM_INTERRUPT_ENABLE
	client->pcm_regs->enclr = PCM_INT_TX_FRE | PCM_INT_TX_URE |
				  PCM_INT_TX_ORE | PCM_INT_TX_WEN |
				  PCM_INT_RX_FRE | PCM_INT_RX_URE |
				  PCM_INT_RX_ORE | PCM_INT_RX_REN |
				  PCM_INT_TX_STP | PCM_INT_RX_STP;
	client->pcm_regs->clear = PCM_INT_TX_FRE | PCM_INT_TX_URE |
				  PCM_INT_RX_FRE |
				  PCM_INT_TX_STP | PCM_INT_RX_STP;
	client->pcm_regs->txrx_dis = PCM_TX_EN | PCM_RX_EN;
	if (play && PCM1_TX(client->s[SNDRV_PCM_STREAM_PLAYBACK]))
		client->pcm_regs->txrx_en = PCM_RX_EN;

	d2b("PCM TXRX_EN(0x%08x) : 0x%08x\n",
		(u_int)&client->pcm_regs->txrx_en, client->pcm_regs->txrx_en);

#ifdef EMXX_PCM_USE_PDMA
	if (IS_PCM0(pcm_ch[client->pcm_ch]))
		ret = emev_pcm0_request_irq(audio_pcm_callback,
				IRQF_DISABLED, pcm_ch[client->pcm_ch].name,
				(void *)pcm, EMEV_PCMMODE_ALSA);
	else
#endif
	ret = request_irq(pcm_ch[client->pcm_ch].irq,
			  audio_pcm_callback, IRQF_DISABLED,
			  pcm_ch[client->pcm_ch].name, (void *)pcm);
	disable_irq(pcm_ch[client->pcm_ch].irq);
	spin_lock_init(&pcm_ch[client->pcm_ch].irq_lock);

	pcm_ch[client->pcm_ch].pcm = pcm;
#endif

	if (0 == ret)
		pcm_ch[client->pcm_ch].in_use = 1;
out:
	FNC_EXIT return ret;
}
EXPORT_SYMBOL(emxx_pcm_new);


static int emxx_pcm_hwdep_open(struct snd_hwdep *hw, struct file *file)
{
	FNC_ENTRY

	FNC_EXIT return 0;
}

static int emxx_pcm_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
				 unsigned int cmd, unsigned long arg)
{
	struct emxx_pcm_client *chip = hw->private_data;
	pcm_ctrl_t sett;
	int ret = 0;

	FNC_ENTRY
	/* dispatch based on command */
	switch (cmd) {
	case SNDRV_EMXX_IOCTL_PCM_SET_CTRL:
		d2b("SNDRV_EMXX_IOCTL_PCM_SET_CTRL cmd=0x%08x\n", cmd);
		if (copy_from_user(&sett, (pcm_ctrl_t *) arg, sizeof(sett))
		 != 0)
			FNC_EXIT return -EFAULT;

		ret = emxx_pcm_set_ctrl(chip->pcm_regs, &sett);

		if (ret < 0)
			FNC_EXIT return ret;

		*chip->sett = sett;

		FNC_EXIT return ret;

	case SNDRV_EMXX_IOCTL_PCM_GET_CTRL:
		d2b("SNDRV_EMXX_IOCTL_PCM_GET_CTRL cmd=0x%08x\n", cmd);
		sett = *chip->sett;
		FNC_EXIT return copy_to_user((void *)arg, &sett, sizeof(sett));

	default:
		d2b("not supported cmd=0x%08x\n", cmd);
		FNC_EXIT return -EINVAL;
	}

	return 0;
}

static int emxx_pcm_hwdep_release(struct snd_hwdep *hw, struct file *file)
{
	FNC_ENTRY

	FNC_EXIT return 0;
}

int emxx_pcm_set_ctrl(struct pcm_regs *regs, pcm_ctrl_t *s)
{
	pcm_ctrl_t *sett = s;
	unsigned int func_val = 0;
	unsigned int cycle_val = 0;
	unsigned int cycle2_val = 0;

	if (sett == NULL)
		return -EINVAL;

	/* mode_sel : Operation mode */
	switch (sett->func.mode_sel) {
	case PCM_MODE_0:        /* mode 0 */
		func_val |= EMXX_PCM_FUNC_MODE0;
		break;
	case PCM_MODE_1:        /* mode 1 */
		func_val |= EMXX_PCM_FUNC_MODE1;
		break;
	case PCM_MODE_2:        /* mode 2 (I2S Format) */
		func_val |= EMXX_PCM_FUNC_MODE2;
		break;
	case PCM_MODE_3:        /* mode 3 (MSB First)  */
		func_val |= EMXX_PCM_FUNC_MODE3;
		break;
	case PCM_MODE_4:        /* mode 4 (LSB First)  */
		func_val = EMXX_PCM_FUNC_MODE4;
		break;
	case PCM_MODE_5:        /* mode 5 (Multi Channel Mode) */
		func_val |= EMXX_PCM_FUNC_MODE5;
		break;
	case PCM_MODE_6:        /* mode 6 (Multi Channel Mode) */
		func_val |= EMXX_PCM_FUNC_MODE6;
		break;
	default:
		return -EINVAL;
	}

	/* m_s : Master/Slave */
	switch (sett->func.m_s) {
	case PCM_MASTER_MODE:   /* Master */
		func_val |= EMXX_PCM_FUNC_MASTER;
		break;
	case PCM_SLAVE_MODE:    /* Slave  */
		func_val |= EMXX_PCM_FUNC_SLAVE;
		break;
	default:
		return -EINVAL;
	}

	/* tx_tim : Transmission timing */
	if (sett->func.tx_tim > EMXX_PCM_TX_WORD_MAX)
		return -EINVAL;

	func_val |= (unsigned int)(sett->func.tx_tim) << EMXX_PCM_TX_WORD_SHIFT;

	switch (sett->func.mode_sel) {
	case PCM_MODE_0:        /* mode 0 */
	case PCM_MODE_1:        /* mode 1 */
	case PCM_MODE_2:        /* mode 2 (I2S Format) */
	case PCM_MODE_3:        /* mode 3 (MSB First)  */
	case PCM_MODE_4:        /* mode 4 (LSB First)  */
		/* cyc_val : Frame length */
		if ((0x3f < sett->cyc.cyc_val) || (sett->cyc.cyc_val < 0x07))
			return -EINVAL;

		if ((sett->cyc.cyc_val < sett->cyc.sib)
		    || (sett->cyc.cyc_val < sett->cyc.sob)) {
			return -EINVAL;
		}
		cycle_val |= sett->cyc.cyc_val;

		/* sib : Data bit length (PMx_SI) */
		if ((0x1f < sett->cyc.sib) || (sett->cyc.sib < 0x07))
			return -EINVAL;

		cycle_val |= sett->cyc.sib << 8;

		/* rx_pd : Data padding (RX) */
		if ((sett->cyc.rx_pd != PCM_PADDING_OFF)
		    && (sett->cyc.rx_pd != PCM_PADDING_ON)) {
			return -EINVAL;
		}
		if (sett->cyc.rx_pd == PCM_PADDING_ON) {
			if ((sett->cyc.sib != 0x0f)
			    && (sett->cyc.sib != 0x07)) {
				d2b("[warning]PADDING_ON\tSIB : 0x%08x\n",
					sett->cyc.sib);
			}
			cycle_val |= sett->cyc.rx_pd << 15;
		}

		/* sob : Data bit length (PMx_SO) */
		if ((0x1f < sett->cyc.sob) || (sett->cyc.sob < 0x07))
			return -EINVAL;

		cycle_val |= sett->cyc.sob << 16;

		/* tx_pd : Data padding (TX) */
		if ((sett->cyc.tx_pd != PCM_PADDING_OFF)
		    && (sett->cyc.tx_pd != PCM_PADDING_ON)) {
			return -EINVAL;
		}
		if (sett->cyc.tx_pd == PCM_PADDING_ON) {
			if ((sett->cyc.sob != 0x0f)
			    && (sett->cyc.sob != 0x07)) {
				d2b("[warning]PADDING_ON\tSOB : 0x%08x\n",
					sett->cyc.sob);
			}
			cycle_val |= sett->cyc.tx_pd << 23;
		}
		break;
	case PCM_MODE_5:        /* mode 5 (Multi Channel Mode) */
	case PCM_MODE_6:        /* mode 6 (Multi Channel Mode) */
		/* cyc_val : Frame length */
		if (0x80 < sett->cyc.cyc_val)
			return -EINVAL;

		if ((sett->cyc.cyc_val == 0) && (sett->cyc2.cyc_val2 == 0))
			return -EINVAL;

		cycle_val |= sett->cyc.cyc_val;

		/* sib : Data bit length (PMx_SI) */
		if ((0x1f < sett->cyc.sib) || (sett->cyc.sib < 0x07))
			return -EINVAL;

		cycle_val |= sett->cyc.sib << 8;

		/* rx_pd : Data padding (RX) */
		if (sett->cyc.rx_pd != PCM_PADDING_OFF)
			return -EINVAL;

		/* sob : Data bit length (PMx_SO) */
		if ((0x1f < sett->cyc.sob) || (sett->cyc.sob < 0x07))
			return -EINVAL;

		cycle_val |= sett->cyc.sob << 16;

		/* tx_pd : Data padding (TX) */
		if (sett->cyc.tx_pd != PCM_PADDING_OFF)
			return -EINVAL;

		/* cyc_val2 : Frame length */
		if (0x80 < sett->cyc2.cyc_val2)
			return -EINVAL;

		cycle2_val |= sett->cyc2.cyc_val2;

		/* sib2 : Data bit length (PMx_SI) */
		if ((0x1f < sett->cyc2.sib2) || (sett->cyc2.sib2 < 0x07))
			return -EINVAL;

		cycle2_val |= sett->cyc2.sib2 << 8;

		/* sob2 : Data bit length (PMx_SO) */
		if ((0x1f < sett->cyc2.sob2) || (sett->cyc2.sob2 < 0x07))
			return -EINVAL;

		cycle2_val |= sett->cyc2.sob2 << 16;
		break;

	default:
		return -EINVAL;
	}

	if (regs != NULL) {
		/* set PMx_FUNC_SEL */
		regs->func_sel = func_val;

		/* set PMx_CYCLE1 */
		regs->cycle = cycle_val;

		if ((sett->func.mode_sel == PCM_MODE_5)
		    || (sett->func.mode_sel == PCM_MODE_6)) {
			/* set PMx_CYCLE2 */
			regs->cycle2 = cycle2_val;
		}
	}

	return 0;
}
EXPORT_SYMBOL(emxx_pcm_set_ctrl);

MODULE_DESCRIPTION("emxx PCM DMA module");
MODULE_LICENSE("GPL");

