/*
 * File Name		: sound/arm/emxx-pcm.h
 * Function		: PCM
 * Release Version 	: Ver 1.05
 * Release Date		: 2010/10/05
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

/* #define AUDIO_MAKING_DEBUG 1 */

#include <linux/platform_device.h>

#include <sound/emxx_pcm_ioctl.h>

#include <mach/dma.h>
#include <mach/smu.h>
#include <mach/pmu.h>

#define EMXX_PCM_INTERRUPT_ENABLE 1
#define EMXX_PCM_MMAP_ENABLE 1

#ifdef CONFIG_MACH_EMGR
#define EMXX_PCM_USE_PDMA
#endif

#define EMXX_PCM_CH0           0x00
#define EMXX_PCM_CH1           0x01
#define EMXX_PCM_MAX_CHANNELS  2

/* func_sel set */
#define EMXX_PCM_TX_WORD_MAX           7
#define EMXX_PCM_TX_WORD_SHIFT         16

#define EMXX_PCM_FUNC_MASTER           0x00000008
#define EMXX_PCM_FUNC_SLAVE            0x00000010
#define EMXX_PCM_FUNC_MODE0            0x00000000
#define EMXX_PCM_FUNC_MODE1            0x00000001
#define EMXX_PCM_FUNC_MODE2            0x00000002
#define EMXX_PCM_FUNC_MODE3            0x00000003
#define EMXX_PCM_FUNC_MODE4            0x00000004
#define EMXX_PCM_FUNC_MODE5            0x00000005
#define EMXX_PCM_FUNC_MODE6            0x00000006
#define EMXX_PCM_FUNC_MODE7            0x00000007

/* cycle set */
#define EMXX_PCM_CYCLE_SHT_TX_PD       23
#define EMXX_PCM_CYCLE_SHT_SOB         16
#define EMXX_PCM_CYCLE_SHT_RX_PD       15
#define EMXX_PCM_CYCLE_SHT_SIB         8
#define EMXX_PCM_CYCLE_SHT_CYC         0

#define reg_volatile volatile

struct pcm_regs {
	reg_volatile u32 func_sel;  /* +0x00 (r/w) */
	reg_volatile u32 txrx_en;   /* +0x04 (r/w) */
	reg_volatile u32 txrx_dis;  /* +0x08 (w) */
	reg_volatile u32 cycle;     /* +0x0c (r/w) */
	reg_volatile u32 raw;       /* +0x10 (r) */
	reg_volatile u32 status;    /* +0x14 (r) */
	reg_volatile u32 enset;     /* +0x18 (r/w) */
	reg_volatile u32 enclr;     /* +0x1c (w) */
	reg_volatile u32 clear;     /* +0x20 (w) */
	reg_volatile u32 txq;       /* +0x24 (r/w) */
	reg_volatile u32 rxq;       /* +0x28 (r) */
	reg_volatile u32 fifo_p;    /* +0x2c (r) */
	reg_volatile u32 cycle2;    /* +0x30 (r/w) */
};

struct audio_stream {
	char *id;               /* identification string */
	int dma_ch;
	dma_regs_t *dma_regs;   /* points to our DMA registers */
	dma_addr_t phys_xq;     /* PCM Data buffer address */
	u32 txrx_id;            /* stream id and PCM enable bit */
	reg_volatile u32 *txrx_en;  /* PCM enable register */
	reg_volatile u32 *txrx_dis; /* PCM disable register */
#ifdef EMXX_PCM_INTERRUPT_ENABLE
	reg_volatile u32 *func_sel; /* PCM func register */
#endif
	int active:1;           /* we are using this stream for transfer now */
	int period;             /* current transfer period */
	int periods;            /* current count of periods registerd in the
				   DMA engine */
	int add_period;         /* add transfer period */
	int tx_spin;            /* are we recoding - flag used to do
				   DMA trans. for sync */
	unsigned int old_offset;
	spinlock_t dma_lock;    /* for locking in DMA operations
				   (see dma-sa1100.c in the kernel) */

	struct snd_pcm_substream *stream;
};

struct emxx_pcm_client {
	struct audio_stream *s[2];
	int pcm_ch;
	struct pcm_regs *pcm_regs;      /* points to our PCM registers */
	pcm_ctrl_t *sett;
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
};

extern int emxx_pcm_new(struct snd_card *, struct emxx_pcm_client *,
 struct snd_pcm **);
extern int emxx_pcm_free(struct emxx_pcm_client *);
extern int emxx_pcm_set_ctrl(struct pcm_regs *, pcm_ctrl_t *);
#ifdef CONFIG_PM
extern int emxx_pcm_suspend(struct platform_device *, pm_message_t);
extern int emxx_pcm_resume(struct platform_device *);
#endif

#ifdef AUDIO_MAKING_DEBUG
extern int debug;
#endif

