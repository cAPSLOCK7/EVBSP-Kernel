/*
 *  File Name	    : arch/arm/mach-emxx/dma.h
 *  Function	    : dmac
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/06/22
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
 *
 */

#ifndef __ARCH_MACH_EMXX_DMA_H
#define __ARCH_MACH_EMXX_DMA_H


#define EMXX_DMAC_M2P_CONF		0x0010
#define EMXX_DMAC_M2P_CONF_TRANS	8

/****  Macro definition  ****/

#define EMXX_DMAC_MASK_CHNO	0x000000ff
#define	EMXX_DMAC_MASK_OWNER	0x03000000
#define EMXX_DMAC_MASK_BLKSIZE 	0x0000ffff
#define EMXX_DMAC_MASK_LENGTH	0x00ffffff
#define EMXX_DMAC_MASK_INT	(EMXX_DMAC_INT_TIME_EN \
				| EMXX_DMAC_INT_ERROR_EN \
				| EMXX_DMAC_INT_BLOCK_EN \
				| EMXX_DMAC_INT_LENG_EN)
#define	EMXX_DMAC_SIZE_BLKSIZEL	0x00001000
#define	EMXX_DMAC_SIZE_BLKSIZES	0x00000080
#define	EMXX_DMAC_SIZE_BLKSIZEMIN	0x00000001

#define EMXX_DMAC_P0_MAX_L_CHANNELS	8
#define EMXX_DMAC_P1_MAX_L_CHANNELS	8
#define EMXX_DMAC_P2_MAX_L_CHANNELS	EMXX_DMAC_P1_MAX_L_CHANNELS

#define EMXX_DMAC_MASK_BASE	0xffff0000


/****  Macro function  ****/

#define EMXX_DMAC_PCHNO(x)        (((x) & 0xe0) >> 5)
#define EMXX_DMAC_DEVICENO(x)     ((x) & 0x1f)
#define EMXX_DMAC_PARBASE2LCHNO(x) ((((long)(x)) >> 8) & 0xf)
#define EMXX_DMAC_OWNER(x)        (((x) & EMXX_DMAC_MASK_OWNER) >> 24)


/****  Structure definition  ****/

/*
 * EMXX DMAC register arrangement
 */
#define reg_volatile	volatile

/* control registers */
struct cntsts_t {
    reg_volatile u_int32_t start;	/* +0x00 DMA start control */
    reg_volatile u_int32_t status;	/* +0x04 DMA status */
    reg_volatile u_int32_t stop;	/* +0x08 DMA stop control */
};

/*
 * interrupt related parameter registers
 */
struct intparm_t {
    reg_volatile u_int32_t stat;	/* +0x00 interrupt status */
    reg_volatile u_int32_t raw_stat;	/* +0x04 interrupt raw status */
    reg_volatile u_int32_t enable; 	/* +0x08 interrupt enable */
    reg_volatile u_int32_t disable;	/* +0x0c interrupt disable */
    reg_volatile u_int32_t clear;	/* +0x10 interrupt clear */
    reg_volatile u_int32_t rsv[3];	/* +0x14,0x18,0x1c Reserved */
};

/*
 * Data for managing logical DMA channels
 */
struct emxx_dmal_t {
    const char      *name;	/* irq name of L-channel. */
    dma_callback_t   callback;	/* call-back function from interrupt. */
    void            *data;	/* argument of call-back function. */
    u_int32_t        defmode;	/* default transfer mode. */
    u_int8_t         in_use;	/* DMA in use flag. */
    u_int8_t         owner;	/* Holder of this LCH. (ACPU/CCPU/DSP) */
    u_int8_t         irq;	/* IRQ number */
    u_int8_t         device;	/* lch device number */
};

/* for `in_use' setting */
#define DMA_NO_USE	0
#define DMA_USE 	1

/*
 * Data for managing physical DMA channels
 */
struct emxx_dmap_t {
	struct cntsts_t    *control; /* control/status regs. */
	struct intparm_t   *intstat; /* interrupt control regs. */
	struct dma_regs    *parbase; /* base of parameter registers array. */
	struct emxx_dmal_t *lch;     /* control structures for logical CH. */
	u_int32_t	lchno;	     /* No. of logical channel. */
	u_int32_t	not_int;     /* Interrupt status register contains
					some bits that are not cause of
					interrupt, just indicate status.
					We must mask out them. How stupid! */
	int (*setup_dma)(dma_regs_t *, dma_addr_t, u_int,
			dma_addr_t, u_int32_t);
};

#endif /* __ARCH_MACH_EMXX_DMA_H */
