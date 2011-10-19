/*
 *  File Name       : linux/arch/arm/mach-emxx/dma.c
 *  Function        : dmac
 *  Release Version : Ver 1.03
 *  Release Date    : 2010/10/21
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/dma.h>

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/pmu.h>
#include <mach/pm.h>
#include <mach/dma.h>

#include "dma.h"

/****  Prototype definition  ****/

static int setup_M2M(struct dma_regs *regs, dma_addr_t src_ptr, u_int size,
	dma_addr_t dst_ptr, u_int32_t mode);
static int setup_M2P(struct dma_regs *regs, dma_addr_t src_ptr, u_int size,
	dma_addr_t dst_ptr, u_int32_t mode);
static int setup_P2M(struct dma_regs *regs, dma_addr_t src_ptr, u_int size,
	dma_addr_t dst_ptr, u_int32_t mode);

/****  Structured data definition  ****/

/* for physical channel #0 (M2M) */
struct emxx_dmal_t dma_pch0_lch[EMXX_DMAC_P0_MAX_L_CHANNELS] = {
	{
		.name    = "ARM M->M0",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M0,
		.device  = 0,
	},
	{
		.name    = "ARM M->M1",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M1,
		.device  = 1,
	},
	{
		.name    = "ARM M->M2",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M2,
		.device  = 2,
	},
	{
		.name    = "ARM M->M3",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M3,
		.device  = 3,
	},
	{
		.name    = "ARM M->M4",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M4,
		.device  = 4,
	},
	{
		.name    = "ARM M->M5",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M5,
		.device  = 5,
	},
	{
		.name    = "ARM M->M6",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M6,
		.device  = 6,
	},
	{
		.name    = "ARM M->M7",
		.defmode = EMXX_DMAC_DEFMODE_32BIT,
		.irq     = INT_M2M7,
		.device  = 7,
	},
};

/* for physical channel #1 (M2P)*/
struct emxx_dmal_t dma_pch1_lch[EMXX_DMAC_P1_MAX_L_CHANNELS] = {
	{
		.name    = "M->P0",
		.irq     = INT_M2P0,
		.device  = CONFIG_EMXX_DMA_LCH0,
	},
	{
		.name    = "M->P1",
		.irq     = INT_M2P1,
		.device  = CONFIG_EMXX_DMA_LCH1,
	},
	{
		.name    = "M->P2",
		.irq     = INT_M2P2,
		.device  = CONFIG_EMXX_DMA_LCH2,
	},
	{
		.name    = "M->P3",
		.irq     = INT_M2P3,
		.device  = CONFIG_EMXX_DMA_LCH3,
	},
	{
		.name    = "M->P4",
		.irq     = INT_M2P4,
		.device  = CONFIG_EMXX_DMA_LCH4,
	},
	{
		.name    = "M->P5",
		.irq     = INT_M2P5,
		.device  = CONFIG_EMXX_DMA_LCH5,
	},
	{
		.name    = "M->P6",
		.irq     = INT_M2P6,
		.device  = CONFIG_EMXX_DMA_LCH6,
	},
	{
		.name    = "M->P7",
		.irq     = INT_M2P7,
		.device  = CONFIG_EMXX_DMA_LCH7,
	},

};
/* for physical channel #2 (P2M) */
struct emxx_dmal_t dma_pch2_lch[EMXX_DMAC_P2_MAX_L_CHANNELS] = {
	{
		.name    = "P->M0",
		.irq     = INT_P2M0,
		.device  = CONFIG_EMXX_DMA_LCH0,
	},
	{
		.name    = "P->M1",
		.irq     = INT_P2M1,
		.device  = CONFIG_EMXX_DMA_LCH1,
	},
	{
		.name    = "P->M2",
		.irq     = INT_P2M2,
		.device  = CONFIG_EMXX_DMA_LCH2,
	},
	{
		.name    = "P->M3",
		.irq     = INT_P2M3,
		.device  = CONFIG_EMXX_DMA_LCH3,
	},
	{
		.name    = "P->M4",
		.irq     = INT_P2M4,
		.device  = CONFIG_EMXX_DMA_LCH4,
	},
	{
		.name    = "P->M5",
		.irq     = INT_P2M5,
		.device  = CONFIG_EMXX_DMA_LCH5,
	},
	{
		.name    = "P->M6",
		.irq     = INT_P2M6,
		.device  = CONFIG_EMXX_DMA_LCH6,
	},
	{
		.name    = "P->M7",
		.irq     = INT_P2M7,
		.device  = CONFIG_EMXX_DMA_LCH7,
	},
};

u_int32_t dma_defmode[EMXX_DMAC_MAX_DEVICE_CHANNELS] = {
	EMXX_DMAC_DEFMODE_32BIT,	/* 0 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 1 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 2 */
	EMXX_DMAC_DEFMODE_16BIT,	/* 3 */
	EMXX_DMAC_DEFMODE_8BIT,		/* 4 */
	EMXX_DMAC_DEFMODE_8BIT,		/* 5 */
	EMXX_DMAC_DEFMODE_8BIT,		/* 6 */
	EMXX_DMAC_DEFMODE_8BIT,		/* 7 */
	EMXX_DMAC_DEFMODE_8BIT,		/* 8 */
	EMXX_DMAC_DEFMODE_8BIT,		/* 9 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 10 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 11 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 12 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 13 */
#ifdef CONFIG_MACH_EMEV
	EMXX_DMAC_DEFMODE_32BIT,	/* 14 */
	EMXX_DMAC_DEFMODE_32BIT,	/* 15 */
#endif
};

/* for every physical channels (3ch) */
struct emxx_dmap_t dma_pch[EMXX_DMAC_MAX_P_CHANNELS] = {
	{ /* PCH #0 (M2M)*/
		.control   = (struct cntsts_t *)IO_ADDRESS(EMXX_DMA_M2M_BASE),
		.intstat   =
		  (struct intparm_t *)IO_ADDRESS(EMXX_DMA_M2M_BASE + 0x0100),
		.parbase   =
		  (struct dma_regs *)IO_ADDRESS(EMXX_DMA_M2M_BASE + 0x1000),
		.lch       = dma_pch0_lch,
		.lchno     = EMXX_DMAC_P0_MAX_L_CHANNELS,
		.not_int   = ~(EMXX_DMAC_INT_LENG_RD | EMXX_DMAC_INT_BLOCK_RD),
		.setup_dma = setup_M2M,
	},
	{ /* PCH #1 (M2P) */
		.control   = (struct cntsts_t *)IO_ADDRESS(EMXX_DMA_M2P_BASE),
		.intstat   =
		  (struct intparm_t *)IO_ADDRESS(EMXX_DMA_M2P_BASE + 0x0100),
		.parbase   =
		  (struct dma_regs *)IO_ADDRESS(EMXX_DMA_M2P_BASE + 0x1000),
		.lch       = dma_pch1_lch,
		.lchno     = EMXX_DMAC_P1_MAX_L_CHANNELS,
		.not_int   = ~(EMXX_DMAC_INT_LENG_RD),
		.setup_dma = setup_M2P,
	},
	{ /* PCH #2 (P2M) */
		.control   = (struct cntsts_t *)IO_ADDRESS(EMXX_DMA_P2M_BASE),
		.intstat   =
		  (struct intparm_t *)IO_ADDRESS(EMXX_DMA_P2M_BASE + 0x0100),
		.parbase   =
		  (struct dma_regs *)IO_ADDRESS(EMXX_DMA_P2M_BASE + 0x1000),
		.lch       = dma_pch2_lch,
		.lchno     = EMXX_DMAC_P2_MAX_L_CHANNELS,
		.not_int   = ~(EMXX_DMAC_INT_LENG_RD),
		.setup_dma = setup_P2M,
	},
};


/*
 * Check channel number
 */
static inline int check_validity_channel(int channel)
{
	u_int pchno;
	u_int devno;

	pchno = EMXX_DMAC_PCHNO(channel);
	devno = EMXX_DMAC_DEVICENO(channel);

	if ((channel & ~(EMXX_DMAC_MASK_CHNO | EMXX_DMAC_MASK_OWNER)) == 0) {
		if (((pchno == 0) && (devno < EMXX_DMAC_P0_MAX_L_CHANNELS))
		  || (((pchno == 1) || (pchno == 2))
			&& (devno < EMXX_DMAC_MAX_DEVICE_CHANNELS)
			&& !((1 << devno) & EMXX_DMAC_NONE_DEVICE_PCH)))
			return 0;
	}
	return 1;
}

/*
 * Get logical channel number
 */
static int get_lch_number(int channel, struct emxx_dmal_t *lch)
{
	int i;
	int device_num = EMXX_DMAC_DEVICENO(channel);

	for (i = 0; i < EMXX_DMAC_P1_MAX_L_CHANNELS; i++) {
		if (lch->device == device_num)
			break;
		lch++;
	}
	return i;
}

/*
 * Functions for default settings.
 *
 * Each function sets DMA up to appropriate default setting when
 * `size' is not 0. Skip parameter settings except source and destination
 * addresses when `size' is 0 so that client driver can use special mode
 * like repeat transfer, timeout, etc.
 */

/* PCH #0 (M2M) */
static int setup_M2M(struct dma_regs *regs, dma_addr_t src_ptr, u_int size,
		dma_addr_t dst_ptr, u_int32_t mode)
{
	u_int blocksize, offset;

	if (size == 0) {
		/* short cut for special transfer mode */
		regs->aadd = src_ptr;
		regs->badd = dst_ptr;
		return 0;
	}

	/*
	 * size != 0
	 * Handle just two contiguous memory region, source and destination.
	 * Simple transfer, but these may be overlapped.
	 */
	if ((size & ~EMXX_DMAC_MASK_LENGTH) != 0)
		return 1;

	/* Determine which mode to use. */
	offset = 0 | EMXX_DMAC_OFFSET_ADD; /* default is forward transfer */

	 /* ignore wrap-around case */
	if (dst_ptr <= src_ptr && src_ptr < dst_ptr + size) {
		if (src_ptr < dst_ptr + EMXX_DMAC_SIZE_BLKSIZES) {
			/* Difference between src_ptr and dst_ptr must be
			   greater than EMXX_DMAC_SIZE_BLKSIZES. */
			return 1;
		}
		/*
		 * overlap, copy forward
		 * It may not need to use multi-block transfer.
		 */
		if (src_ptr < dst_ptr + EMXX_DMAC_SIZE_BLKSIZEL)
			blocksize = EMXX_DMAC_SIZE_BLKSIZES;
		else
			blocksize = EMXX_DMAC_SIZE_BLKSIZEL;

	/* ignore wrap-around case */
	} else if (src_ptr < dst_ptr && dst_ptr < src_ptr + size) {
		u_int sharesize, isolatesize;

		isolatesize = dst_ptr - src_ptr;
		if (isolatesize < EMXX_DMAC_SIZE_BLKSIZES) {
			/* Difference between src_ptr and dst_ptr must be
			   greater than EMXX_DMAC_SIZE_BLKSIZES. */
			return 1;
		}
		sharesize = size - isolatesize;
		/* overlap, copy backward */
		if (size % isolatesize == 0) {
			/*
			 * It's most likely that backward copy is used for video
			 * console back scroll.
			 * In that case, offset between source and destination
			 * is equal to one line, and size, most likely, can be
			 * divided by it. (Assume one line, does not fit for
			 * scrolling multiple lines.)
			 */
			 blocksize = isolatesize;
		} else {
			/*
			 * Copy with block size EMXX_DMAC_SIZE_BLKSIZEMIN.
			 * This will cause inefficiency around bus traffic.
			 * It may be better that this case does not exist and
			 * handle as error.
			 */
			blocksize = EMXX_DMAC_SIZE_BLKSIZEMIN;
		}
		src_ptr += size - blocksize;
		dst_ptr += size - blocksize;
		offset = blocksize * 2 | EMXX_DMAC_OFFSET_SUB;
	} else if ((size & ~EMXX_DMAC_MASK_BLKSIZE) == 0) {
		/*
		 * does not overlap, small enough to use single block transfer
		 */
		blocksize = size;
	} else {
		/*
		 * does not overlap, greater than 65534, use multiple block
		 * transfer
		 */
		blocksize = EMXX_DMAC_SIZE_BLKSIZEL;
	}
	/* setup DMAC registers */
	regs->aadd = src_ptr;
	regs->badd = dst_ptr;
	regs->aoff = offset;
	regs->boff = offset;
	regs->asize_count = 0;
	regs->bsize_count = 0;
	regs->leng = size;
	regs->size = blocksize;
	regs->mode = mode;

	return 0;
}

/* PCH #1 (M2P) */
static int setup_M2P(struct dma_regs *regs, dma_addr_t src_ptr, u_int size,
		   dma_addr_t dst_ptr, u_int32_t mode)
{
	u_int blocksize;

	if (size == 0) {
		/* short cut for special transfer mode */
		regs->aadd = src_ptr;
		regs->badd = dst_ptr;
		return 0;
	}

	if ((size & ~EMXX_DMAC_MASK_LENGTH) != 0)
		return 1;

	if ((size & ~EMXX_DMAC_MASK_BLKSIZE) == 0) {
		/* use single block transfer */
		blocksize = size;
	} else {
		/* use multiple block transfer */
		blocksize = EMXX_DMAC_SIZE_BLKSIZEL;
	}
	regs->aadd = src_ptr;
	regs->badd = dst_ptr;
	regs->aoff = 0;
	regs->asize_count = 0;
	regs->asize = blocksize;
	regs->leng = size;
	regs->mode = mode;

	return 0;
}

/* PCH #2 (P2M) */
static int setup_P2M(struct dma_regs *regs, dma_addr_t src_ptr, u_int size,
		   dma_addr_t dst_ptr, u_int32_t mode)
{
	u_int blocksize;

	if (size == 0) {
		/* short cut for special transfer mode */
		regs->aadd = src_ptr;
		regs->badd = dst_ptr;
		return 0;
	}

	if ((size & ~EMXX_DMAC_MASK_LENGTH) != 0)
		return 1;

	if ((size & ~EMXX_DMAC_MASK_BLKSIZE) == 0) {
		/* use single block transfer */
		blocksize = size;
	} else {
		/* use multiple block transfer */
		blocksize = EMXX_DMAC_SIZE_BLKSIZEL;
	}

	regs->aadd = src_ptr;
	regs->badd = dst_ptr;
	regs->boff = 0;
	regs->bsize_count = 0;
	regs->bsize = blocksize;
	regs->leng = size;
	regs->mode = mode;

	return 0;
}

/*
 * Interrupt Handler
 */
static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	struct emxx_dmap_t *pch = NULL;
	struct emxx_dmal_t *lch;
	struct dma_regs    *regs = dev_id;
	struct intparm_t   *intcnt;
	int    lch_num;
	int    stshift;
	u_int  sts;
	u_int  rawsts;
	u_int  base;

	/* We can compute channel number from address of control registers */
	base = (u_int)regs & EMXX_DMAC_MASK_BASE;
	switch (base) {
	case IO_ADDRESS(EMXX_DMA_M2M_BASE):
		pch = &dma_pch[0];
		break;
	case IO_ADDRESS(EMXX_DMA_M2P_BASE):
		pch = &dma_pch[1];
		break;
	case IO_ADDRESS(EMXX_DMA_P2M_BASE):
		pch = &dma_pch[2];
		break;
	default:
		return IRQ_NONE;
	}

	lch_num = EMXX_DMAC_PARBASE2LCHNO(regs);
	if (lch_num >= pch->lchno)
		return IRQ_NONE;

	/*
	 * Check interrupt status.
	 *  If it's not for this channel then return.
	 */
	intcnt = pch->intstat + lch_num / 4;

	stshift = (lch_num % 4) * 8;
	sts = (intcnt->stat >> stshift) & 0xff;
	/* Some bits are not cause of interrupt. */
	if ((sts & pch->not_int) == 0)
		return IRQ_NONE;

	rawsts = (intcnt->raw_stat >> stshift) & 0xff;

	/* clear interrupt status */
	intcnt->clear = sts << stshift;

	/* call client driver's interrupt handling routine */
	lch = pch->lch + lch_num;
	lch->callback(lch->data, sts, rawsts);

	return IRQ_HANDLED;
}


/****  External functions  ****/

/*
 * Acquire DMA channel
 *   emxx_request_dma()
 */
int emxx_request_dma(int channel, const char *device_id,
	 dma_callback_t callback, void *data, struct dma_regs **dma_regs)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct dma_regs    *regs;
	int err;
	int owner;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return -ENODEV;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return -ENODEV;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return -ENODEV;

	lch = pch->lch + lch_num;
	if (lch->in_use != DMA_NO_USE)
		return -EBUSY;

	regs = pch->parbase + lch_num;
	owner = EMXX_DMAC_OWNER(channel);

	if (owner == EMXX_DMAC_OWNER(EMXX_DMAC_OWNER_ACPU)) {
		err = request_irq(lch->irq, dma_irq_handler,
			IRQF_DISABLED, device_id, regs);
		if (err != 0) {
			printk(KERN_INFO "%s(): unable to request IRQ %d for DMA channel (%s)\n",
				__func__, lch->irq, device_id);
			return err;
		}
	}

	if (EMXX_DMAC_PCHNO(channel) != 0)
		lch->defmode = dma_defmode[EMXX_DMAC_DEVICENO(channel)];

	/* Supply TCLK */
	if (lch_num == 0) {
		if (EMXX_DMAC_PCHNO(channel) == 1)
			emxx_open_clockgate(EMXX_CLK_M2P_T);
		if (EMXX_DMAC_PCHNO(channel) == 2)
			emxx_open_clockgate(EMXX_CLK_P2M_T);
	}

	/*
	 * RESET does not initialize some DMAC registers.
	 * Set DMA channel to known state (for single block transfer) here
	 * so that stabilizing phenomena caused by client code
	 * lacks register setting (bug).
	 * Use cold start address where most likely boot ROM exists.
	 */
	(void)pch->setup_dma(regs, 0x0, 0x1000, 0x0, lch->defmode);

	/* Hold information for further DMAC operations. */
	lch->in_use = DMA_USE;
	lch->owner = owner;
	lch->callback = callback;
	lch->data = data;

	/* return pointer to control registers. */
	if (dma_regs != NULL)
		*dma_regs = regs;

	return 0;
}
EXPORT_SYMBOL(emxx_request_dma);

/*
 * Release DMA channel
 */
void emxx_free_dma(int channel)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct dma_regs    *regs;
	int err;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return;

	lch = pch->lch + lch_num;
	if (lch->in_use != DMA_USE)
		return;

	if (lch->owner != EMXX_DMAC_OWNER(channel))
		return;

	/* Stop DMA */
	emxx_clear_dma(channel);

	/* Stop TCLK */
	if (lch_num == 0) {
		if (EMXX_DMAC_PCHNO(channel) == 1)
			emxx_close_clockgate(EMXX_CLK_M2P_T);
		if (EMXX_DMAC_PCHNO(channel) == 2)
			emxx_close_clockgate(EMXX_CLK_P2M_T);
	}

	/* Release interrupt */
	if (lch->owner == EMXX_DMAC_OWNER(EMXX_DMAC_OWNER_ACPU)) {
		regs = pch->parbase + lch_num;
		free_irq(lch->irq, regs);
	}

	lch->in_use = DMA_NO_USE;
	lch->callback = NULL;
	lch->data = NULL;

	return;
}
EXPORT_SYMBOL(emxx_free_dma);

/*
 * Setup and enable DMA.
 */
int emxx_start_dma(int channel, dma_addr_t src_ptr, u_int size,
			   dma_addr_t dst_ptr, int intmask)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct dma_regs    *regs;
	struct intparm_t   *intcnt;
	struct cntsts_t    *base;
	int stshift;
	int err;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return -ENODEV;

	if ((intmask & ~EMXX_DMAC_MASK_INT) != 0)
		return -EINVAL;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return -ENODEV;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return -ENODEV;

	lch = pch->lch + lch_num;
	if (lch->in_use != DMA_USE)
		return -ENXIO;

	if (lch->owner != EMXX_DMAC_OWNER(channel))
		return -ENODEV;

	if ((intmask != 0) && (lch->callback == NULL))
		return -EINVAL;

	/* Set DMA channel up to appropriate default setting. */
	regs = pch->parbase + lch_num;
	err = pch->setup_dma(regs, src_ptr, size, dst_ptr, lch->defmode);
	if (err != 0)
		return -EINVAL;

	if (EMXX_DMAC_PCHNO(channel) != 0)
		regs->pch = lch->device;

	/*
	 * When size > 65534 or overlapped source and destination, we need to
	 * use multiple block transfer.
	 * But we don't need block interrupt, so mask it when size != 0.
	 */
	if (size != 0)
		intmask &= ~EMXX_DMAC_INT_BLOCK_EN;

	/*
	 * Enable interrupt.
	 * At first, disable all interrupt, then enable desired interrupt.
	 */
	intcnt = pch->intstat + lch_num / 4;

	stshift = (lch_num % 4) * 8;
	intcnt->disable = 0xff << stshift;
	intcnt->clear = 0xff << stshift;
	intcnt->enable = (unsigned char)intmask << stshift;

	/* Enable DMA. */
	base = pch->control;
	base->start = 1 << lch_num;

	return 0;
}
EXPORT_SYMBOL(emxx_start_dma);

/*
 * Setup and enable M2M DMA.
 */
int emxx_start_m2m_dma(unsigned int m2m_cont, unsigned int intmask)
{
	struct emxx_dmal_t *lch;
	struct intparm_t   *intcnt;
	struct cntsts_t    *base;
	int i, stshift;

	/* cont check */
	if ((m2m_cont & ~0xff) || !m2m_cont)
		return -ENODEV;

	/* mask check */
	if ((intmask & ~EMXX_DMAC_MASK_INT) != 0)
		return -EINVAL;

	for (i = 0; i < EMXX_DMAC_P0_MAX_L_CHANNELS; i++) {
		if (m2m_cont & (1 << i)) {
			lch = &dma_pch0_lch[i];
			if (lch->in_use != DMA_USE)
				return -ENXIO;

			if (intmask && (lch->callback == NULL))
				return -EINVAL;

			intcnt = dma_pch[0].intstat + i / 4;
			stshift = (i % 4) * 8;
			intcnt->disable = 0xff << stshift;
			intcnt->clear = 0xff << stshift;
			intcnt->enable = (unsigned char)intmask << stshift;
		}
	}

	/* Enable DMA. */
	base = dma_pch[0].control;
	base->start = m2m_cont;

	return 0;
}
EXPORT_SYMBOL(emxx_start_m2m_dma);


/*
 * Acquire DMA channel status.
 */
int emxx_dma_status(int channel)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct cntsts_t    *base;
	int stat;
	int err;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return -ENODEV;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return -ENODEV;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return -ENODEV;

	lch = pch->lch + lch_num;
	/* does not check `in_use'. can read status of any DMA channel */

	/* Read status register, return corresponding bit. */
	base = pch->control;
	stat = (base->status >> lch_num) & 1;

	return stat;
}
EXPORT_SYMBOL(emxx_dma_status);

/*
 * Acquire DMA channel status (register writable flag).
 */
int emxx_dma_writable_flag(int channel)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct cntsts_t    *base;
	int stat;
	int err;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return -ENODEV;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return -ENODEV;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return -ENODEV;

	lch = pch->lch + lch_num;
	/* does not check `in_use'. can read status of any DMA channel */

	/* Read status register, return corresponding bit. */
	base = pch->control;
	stat = (base->status >> (16 + lch_num)) & 1;

	return stat;
}
EXPORT_SYMBOL(emxx_dma_writable_flag);

/*
 * Acquire all DMA channel status.
 * check all DMA channels,
 * return 1 if there is enabled channel.
 * Don't start DMA channel while calling this function.
 */
int emxx_dma_busy(void)
{
	struct cntsts_t    *base;
	int p;

	for (p = 0 ; p < EMXX_DMAC_MAX_P_CHANNELS ; p++) {
		base = dma_pch[p].control;
		if (base->status != 0)
			return 1;
	}

	return 0;
}
EXPORT_SYMBOL(emxx_dma_busy);

/*
 * Stop DMA channel.
 */
void emxx_clear_dma(int channel)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct cntsts_t    *base;
	struct intparm_t   *intcnt;
	int stshift;
	int err;
	int stat;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return ;

	lch = pch->lch + lch_num;
	if (lch->in_use != DMA_USE)
		return;
	if (lch->owner != EMXX_DMAC_OWNER(channel))
		return;

	/* disable all interrupt */
	intcnt = pch->intstat + lch_num / 4;

	stshift = (lch_num % 4) * 8;
	intcnt->disable = 0xff << stshift;
	intcnt->clear = 0xff << stshift;

	/* stop DMA */
	base = pch->control;
	base->stop = 1 << lch_num;
	base->stop = 1 << lch_num;
	/* Until DMA stops, polling */
	for (stat = 1; stat != 0;)
		stat = (base->status >> lch_num) & 1 ;
}
EXPORT_SYMBOL(emxx_clear_dma);

/*
 * Stop DMA channel too.
 */
void emxx_reset_dma(int channel)
{
	emxx_clear_dma(channel);
}
EXPORT_SYMBOL(emxx_reset_dma);

/*
 * Retrieve current DMA position.
 */
dma_addr_t emxx_get_dma_pos(int channel)
{
	struct emxx_dmap_t *pch;
	struct emxx_dmal_t *lch;
	struct dma_regs    *regs;
	int err;
	int lch_num;

	err = check_validity_channel(channel);
	if (err != 0)
		return 0;

	pch = &dma_pch[EMXX_DMAC_PCHNO(channel)];
	if (pch->lch == NULL)
		return 0;

	lch_num = get_lch_number(channel, pch->lch);
	if (lch_num >= pch->lchno)
		return 0;

	lch = pch->lch + lch_num;
	/* does not check `in_use'. can read position of any DMA channel */
	regs = pch->parbase + lch_num;

	switch (EMXX_DMAC_PCHNO(channel)) {
	case 0:
	case 1:
		return regs->aadp;
		break;
	case 2:
		return regs->badp;
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(emxx_get_dma_pos);

/*
 * Stop DMA.
 * We'd like to suspend DMA instead of stopping, if we can.
 * Unfortunately, DMAC on EMXX can't suspend, we stop DMA at this moment.
 * (can't resume)
 */
void emxx_stop_dma(int channel)
{
	emxx_clear_dma(channel);
}
EXPORT_SYMBOL(emxx_stop_dma);

static int dma_suspend(struct platform_device *dev, pm_message_t state)
{
	switch (state.event) {
	case PM_EVENT_SUSPEND:
		if (emxx_dma_busy())
			return -EBUSY;
		break;
	default:
		break;
	}
	return 0;
}

static int dma_resume(struct platform_device *dev)
{
	return 0;
}

static int dma_probe(struct platform_device *dev)
{
	int i;

	/* open all clock gate for DMAC (except TCLK) */
	emxx_open_clockgate(EMXX_CLK_M2M | EMXX_CLK_M2M_P);
	emxx_open_clockgate(EMXX_CLK_M2P | EMXX_CLK_M2P_P);
	emxx_open_clockgate(EMXX_CLK_P2M | EMXX_CLK_P2M_P);
	/* release reset */
	emxx_unreset_device(EMXX_RST_M2M);
	emxx_unreset_device(EMXX_RST_M2P);
	emxx_unreset_device(EMXX_RST_P2M);
	/* autoclock on */
	emxx_clkctrl_on(EMXX_CLKCTRL_M2M);
	emxx_clkctrl_on(EMXX_CLKCTRL_M2P);
	emxx_clkctrl_on(EMXX_CLKCTRL_P2M);
	emxx_clkctrl_on(EMXX_CLKCTRL_M2MPCLK);
	emxx_clkctrl_on(EMXX_CLKCTRL_M2PPCLK);
	emxx_clkctrl_on(EMXX_CLKCTRL_P2MPCLK);

	writel(EMXX_DMAC_M2P_CONF_TRANS,
		IO_ADDRESS(EMXX_DMA_M2P_BASE) + EMXX_DMAC_M2P_CONF);

	for (i = 0 ; i < EMXX_DMAC_P0_MAX_L_CHANNELS ; i++) {
		dma_pch0_lch[i].callback = NULL;
		dma_pch0_lch[i].data     = NULL;
		dma_pch0_lch[i].in_use   = DMA_NO_USE;
	}
	for (i = 0 ; i < EMXX_DMAC_P1_MAX_L_CHANNELS ; i++) {
		dma_pch1_lch[i].callback = NULL;
		dma_pch1_lch[i].data     = NULL;
		dma_pch1_lch[i].in_use   = DMA_NO_USE;
	}
	for (i = 0 ; i < EMXX_DMAC_P2_MAX_L_CHANNELS ; i++) {
		dma_pch2_lch[i].callback = NULL;
		dma_pch2_lch[i].data     = NULL;
		dma_pch2_lch[i].in_use   = DMA_NO_USE;
	}

	return 0;
}

static struct platform_driver dma_driver = {
	.probe   = dma_probe,
	.suspend = dma_suspend,
	.resume  = dma_resume,
	.driver  = {
		.name  = "dma",
		.owner = THIS_MODULE,
	},
};


/*
 * Initialize
 */
static int __init emxx_dma_init(void)
{
	return platform_driver_register(&dma_driver);
}

arch_initcall(emxx_dma_init);

