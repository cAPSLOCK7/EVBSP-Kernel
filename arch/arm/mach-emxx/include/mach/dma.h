/*
 *  File Name	    : arch/arm/mach-emxx/include/mach/dma.h
 *  Function	    : dmac
 *  Release Version : Ver 1.01
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
 *
 */

#ifndef __ASM_ARCH_EMXX_DMA_H
#define __ASM_ARCH_EMXX_DMA_H

#define EMXX_DMAC_MAX_P_CHANNELS	3

#define	EMXX_DMAC_OWNER_ACPU		0x00000000
#define	EMXX_DMAC_OWNER_DSP		0x01000000

/* DMAC logical channel ID */

/* P0 for ACPU */
#define EMXX_DMAC_M2M_ACPU_LCH0	(0x00|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH0	EMXX_DMAC_M2M_ACPU_LCH0
#define EMXX_DMAC_M2M_ACPU_LCH1	(0x01|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH1	EMXX_DMAC_M2M_ACPU_LCH1
#define EMXX_DMAC_M2M_ACPU_LCH2	(0x02|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH2	EMXX_DMAC_M2M_ACPU_LCH2
#define EMXX_DMAC_M2M_ACPU_LCH3	(0x03|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH3	EMXX_DMAC_M2M_ACPU_LCH3
#define EMXX_DMAC_M2M_ACPU_LCH4	(0x04|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH4	EMXX_DMAC_M2M_ACPU_LCH4
#define EMXX_DMAC_M2M_ACPU_LCH5	(0x05|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH5	EMXX_DMAC_M2M_ACPU_LCH5
#define EMXX_DMAC_M2M_ACPU_LCH6	(0x06|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH6	EMXX_DMAC_M2M_ACPU_LCH6
#define EMXX_DMAC_M2M_ACPU_LCH7	(0x07|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2M_LCH7	EMXX_DMAC_M2M_ACPU_LCH7
/* P0 for DSP */
#define EMXX_DMAC_M2M_DSP_LCH0	(0x00|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH1	(0x01|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH2	(0x02|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH3	(0x03|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH4	(0x04|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH5	(0x05|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH6	(0x06|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2M_DSP_LCH7	(0x07|EMXX_DMAC_OWNER_DSP)

#define EMXX_M2M_DMA_LCH(x)	(1 << x)

/* P1 for ACPU */
#define EMXX_DMAC_M2P_ACPU_MMM		(0x20|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_MMM		EMXX_DMAC_M2P_ACPU_MMM
#define EMXX_DMAC_M2P_ACPU_MSPGDF	(0x21|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_MSPGDF		EMXX_DMAC_M2P_ACPU_MSPGDF
#define EMXX_DMAC_M2P_ACPU_MSPPB	(0x22|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_MSPPB		EMXX_DMAC_M2P_ACPU_MSPPB
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_M2P_ACPU_CRP		(0x23|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_CRP		EMXX_DMAC_M2P_ACPU_CRP
#else	/* CONFIG_MACH_EMGR */
#define EMXX_DMAC_M2P_ACPU_UART4	(0x24|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_UART4		EMXX_DMAC_M2P_ACPU_UART4
#define EMXX_DMAC_M2P_ACPU_UART5	(0x25|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_UART5		EMXX_DMAC_M2P_ACPU_UART5
#endif
#define EMXX_DMAC_M2P_ACPU_UART0	(0x26|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_UART0		EMXX_DMAC_M2P_ACPU_UART0
#define EMXX_DMAC_M2P_ACPU_UART1	(0x27|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_UART1		EMXX_DMAC_M2P_ACPU_UART1
#define EMXX_DMAC_M2P_ACPU_UART2	(0x28|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_UART2		EMXX_DMAC_M2P_ACPU_UART2
#define EMXX_DMAC_M2P_ACPU_UART3	(0x29|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_UART3		EMXX_DMAC_M2P_ACPU_UART3
#define EMXX_DMAC_M2P_ACPU_SIO0		(0x2A|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_SIO0		EMXX_DMAC_M2P_ACPU_SIO0
#define EMXX_DMAC_M2P_ACPU_SIO1		(0x2B|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_SIO1		EMXX_DMAC_M2P_ACPU_SIO1
#define EMXX_DMAC_M2P_ACPU_SIO2		(0x2C|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_SIO2		EMXX_DMAC_M2P_ACPU_SIO2
#define EMXX_DMAC_M2P_ACPU_SIO3		(0x2D|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_SIO3		EMXX_DMAC_M2P_ACPU_SIO3
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_M2P_ACPU_SIO4		(0x2E|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_SIO4		EMXX_DMAC_M2P_ACPU_SIO4
#define EMXX_DMAC_M2P_ACPU_SIO5		(0x2F|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_M2P_SIO5		EMXX_DMAC_M2P_ACPU_SIO5
#endif
/* P1 for DSP */
#define EMXX_DMAC_M2P_DSP_MMM		(0x20|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_MSPGDF	(0x21|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_MSPPB		(0x22|EMXX_DMAC_OWNER_DSP)
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_M2P_DSP_CRP		(0x23|EMXX_DMAC_OWNER_DSP)
#else	/* CONFIG_MACH_EMGR */
#define EMXX_DMAC_M2P_DSP_UART4		(0x24|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_UART5		(0x25|EMXX_DMAC_OWNER_DSP)
#endif
#define EMXX_DMAC_M2P_DSP_UART0		(0x26|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_UART1		(0x27|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_UART2		(0x28|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_UART3		(0x29|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_SIO0		(0x2A|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_SIO1		(0x2B|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_SIO2		(0x2C|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_SIO3		(0x2D|EMXX_DMAC_OWNER_DSP)
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_M2P_DSP_SIO4		(0x2E|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_M2P_DSP_SIO5		(0x2F|EMXX_DMAC_OWNER_DSP)
#endif

/* P2 for ACPU */
#define EMXX_DMAC_P2M_ACPU_MMM		(0x40|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_MMM		EMXX_DMAC_P2M_ACPU_MMM
#define EMXX_DMAC_P2M_ACPU_MSPGDF	(0x41|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_MSPGDF		EMXX_DMAC_P2M_ACPU_MSPGDF
#define EMXX_DMAC_P2M_ACPU_MSPPB	(0x42|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_MSPPB		EMXX_DMAC_P2M_ACPU_MSPPB
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_P2M_ACPU_CRP		(0x43|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_CRP		EMXX_DMAC_P2M_ACPU_CRP
#else	/* CONFIG_MACH_EMGR */
#define EMXX_DMAC_P2M_ACPU_UART4	(0x44|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_UART4		EMXX_DMAC_P2M_ACPU_UART4
#define EMXX_DMAC_P2M_ACPU_UART5	(0x45|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_UART5		EMXX_DMAC_P2M_ACPU_UART5
#endif
#define EMXX_DMAC_P2M_ACPU_UART0	(0x46|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_UART0		EMXX_DMAC_P2M_ACPU_UART0
#define EMXX_DMAC_P2M_ACPU_UART1	(0x47|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_UART1		EMXX_DMAC_P2M_ACPU_UART1
#define EMXX_DMAC_P2M_ACPU_UART2	(0x48|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_UART2		EMXX_DMAC_P2M_ACPU_UART2
#define EMXX_DMAC_P2M_ACPU_UART3	(0x49|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_UART3		EMXX_DMAC_P2M_ACPU_UART3
#define EMXX_DMAC_P2M_ACPU_SIO0		(0x4A|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_SIO0		EMXX_DMAC_P2M_ACPU_SIO0
#define EMXX_DMAC_P2M_ACPU_SIO1		(0x4B|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_SIO1		EMXX_DMAC_P2M_ACPU_SIO1
#define EMXX_DMAC_P2M_ACPU_SIO2		(0x4C|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_SIO2		EMXX_DMAC_P2M_ACPU_SIO2
#define EMXX_DMAC_P2M_ACPU_SIO3		(0x4D|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_SIO3		EMXX_DMAC_P2M_ACPU_SIO3
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_P2M_ACPU_SIO4		(0x4E|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_SIO4		EMXX_DMAC_P2M_ACPU_SIO4
#define EMXX_DMAC_P2M_ACPU_SIO5		(0x4F|EMXX_DMAC_OWNER_ACPU)
#define EMXX_DMAC_P2M_SIO5		EMXX_DMAC_P2M_ACPU_SIO5
#endif
/* P2 for DSP */
#define EMXX_DMAC_P2M_DSP_MMM		(0x40|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_MSPGDF	(0x41|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_MSPPB		(0x42|EMXX_DMAC_OWNER_DSP)
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_P2M_DSP_CRP		(0x43|EMXX_DMAC_OWNER_DSP)
#else	/* CONFIG_MACH_EMGR */
#define EMXX_DMAC_P2M_DSP_UART4		(0x44|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_UART5		(0x45|EMXX_DMAC_OWNER_DSP)
#endif
#define EMXX_DMAC_P2M_DSP_UART0		(0x46|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_UART1		(0x47|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_UART2		(0x48|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_UART3		(0x49|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_SIO0		(0x4A|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_SIO1		(0x4B|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_SIO2		(0x4C|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_SIO3		(0x4D|EMXX_DMAC_OWNER_DSP)
#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_P2M_DSP_SIO4		(0x4E|EMXX_DMAC_OWNER_DSP)
#define EMXX_DMAC_P2M_DSP_SIO5		(0x4F|EMXX_DMAC_OWNER_DSP)
#endif

#ifdef CONFIG_MACH_EMEV
#define EMXX_DMAC_MAX_DEVICE_CHANNELS	16
#define EMXX_DMAC_NONE_DEVICE_PCH	((1 << 4) | (1 << 5))
#else	/* CONFIG_MACH_EMGR */
#define EMXX_DMAC_MAX_DEVICE_CHANNELS	14
#define EMXX_DMAC_NONE_DEVICE_PCH	(1 << 3)
#endif

#define reg_volatile	volatile

struct dma_regs {
    reg_volatile u32    aadd;	/* +0x00 source (start) address */
    reg_volatile u32    aadp;	/* +0x04 source address pointer */
    reg_volatile u32    aoff;	/* +0x08 source address offset (lower 16bit) */
    reg_volatile u32    asize;	/* +0x0c source block size (lower 16bit) */
    reg_volatile u32    asize_count;/* +0x10 source block count (lower 4bit) */
    reg_volatile u32    rfu0;	/* +0x14 */
    reg_volatile u32    rfu1;	/* +0x18 */
    reg_volatile u32    rfu2;	/* +0x1c */
    reg_volatile u32    badd;	/* +0x20 destination (start) address */
    reg_volatile u32    badp;	/* +0x24 destination address pointer */
    reg_volatile u32    boff;	/* +0x28 destination address offset
					 (lower 16bit) */
    reg_volatile u32    bsize;	/* +0x2c destination block size (lower 16bit) */
				/* +0x30 destination block count (lower 4bit) */
    reg_volatile u32    bsize_count;
    reg_volatile u32    rfu3;	/* +0x34 */
    reg_volatile u32    rfu4;	/* +0x38 */
    reg_volatile u32    rfu5;	/* +0x3c */
    reg_volatile u32    leng;	/* +0x40 length (lower 24bit) */
    reg_volatile u32    rcount;	/* +0x44 read length count (lower 24bit) */
    reg_volatile u32    wcount;	/* +0x48 write length count (lower 24bit) */
    reg_volatile u32    size;	/* +0x4c block size (lower 16bit) */
    reg_volatile u32    mode;	/* +0x50 mode (lower 2bit and bit8) */
    reg_volatile u32    time;	/* +0x54 timer (lower 24bit) */
    reg_volatile u32    time_count; /* +0x58 timer count (lower 24bit) */
    reg_volatile u32    pch;	/* +0x5c channel number */
    reg_volatile u32    rfu6[64 - 24];	/* +0x60 fill up to offset of 0xff */
};
#define dma_regs_t	struct dma_regs

/* for `aoff' `boff' setting */
#define EMXX_DMAC_OFFSET_ADD	0x00000000
#define EMXX_DMAC_OFFSET_SUB	0x00010000

/* for `mode' setting */
#define EMXX_DMAC_BMODE_REPEAT	0x100
#define EMXX_DMAC_AMODE_REPEAT	0x1

/* setting for M2M */
#define EMXX_DMAC_MODE_ORDER_BLOCK	(0x0 << 6)
#define EMXX_DMAC_MODE_ORDER_UV		(0x1 << 6)
#define EMXX_DMAC_MODE_ORDER_BURST	(0x2 << 6)
#define EMXX_DMAC_MODE_TYPE_NORMAL	(0x0 << 4)
#define EMXX_DMAC_MODE_TYPE_FILLMODE	(0x1 << 4)
#define EMXX_DMAC_MODE_TYPE_MASKMODE	(0x2 << 4)

/* setting for M2P and M2P */
#define EMXX_DMAC_AMODE_BIT32	(0x0 << 4)
#define EMXX_DMAC_AMODE_BIT16	(0x1 << 4)
#define EMXX_DMAC_AMODE_BIT8	(0x2 << 4)

/* setting for LCH0 */
#define EMXX_DMAC_AMODE_TIME	0x4

/* `mode' default setting */
#define EMXX_DMAC_DEFMODE_32BIT	(EMXX_DMAC_AMODE_BIT32 | \
				  EMXX_DMAC_ENDI_R3210 | \
				  EMXX_DMAC_ENDI_W3210)
#define EMXX_DMAC_DEFMODE_16BIT	(EMXX_DMAC_AMODE_BIT16 | \
				  EMXX_DMAC_ENDI_R10 | \
				  EMXX_DMAC_ENDI_W10)
#define EMXX_DMAC_DEFMODE_8BIT	(EMXX_DMAC_AMODE_BIT8  | \
				  EMXX_DMAC_ENDI_R3210 | \
				  EMXX_DMAC_ENDI_W3210)

/* Endian conversion (byte lane selections) */
#define EMXX_DMAC_ENDI_R		16
#define EMXX_DMAC_ENDI_W		24
#define EMXX_DMAC_ENDI(HH, HL, LH, LL, RW) \
		((((HH) << 6) | ((HL) << 4) | ((LH) << 2) | (LL)) << (RW))
/* Endian(Read) for 32bit */
#define EMXX_DMAC_ENDI_R0123	EMXX_DMAC_ENDI(0, 1, 2, 3, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R0132	EMXX_DMAC_ENDI(0, 1, 3, 2, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R0213	EMXX_DMAC_ENDI(0, 2, 1, 3, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R0231	EMXX_DMAC_ENDI(0, 2, 3, 1, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R0312	EMXX_DMAC_ENDI(0, 3, 1, 2, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R0321	EMXX_DMAC_ENDI(0, 3, 2, 1, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R1023	EMXX_DMAC_ENDI(1, 0, 2, 3, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R1032	EMXX_DMAC_ENDI(1, 0, 3, 2, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R1203	EMXX_DMAC_ENDI(1, 2, 0, 3, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R1230	EMXX_DMAC_ENDI(1, 2, 3, 0, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R1302	EMXX_DMAC_ENDI(1, 3, 0, 2, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R1320	EMXX_DMAC_ENDI(1, 3, 2, 0, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R2013	EMXX_DMAC_ENDI(2, 0, 1, 3, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R2031	EMXX_DMAC_ENDI(2, 0, 3, 1, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R2103	EMXX_DMAC_ENDI(2, 1, 0, 3, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R2130	EMXX_DMAC_ENDI(2, 1, 3, 0, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R2301	EMXX_DMAC_ENDI(2, 3, 0, 1, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R2310	EMXX_DMAC_ENDI(2, 3, 1, 0, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R3012	EMXX_DMAC_ENDI(3, 0, 1, 2, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R3021	EMXX_DMAC_ENDI(3, 0, 2, 1, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R3102	EMXX_DMAC_ENDI(3, 1, 0, 2, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R3120	EMXX_DMAC_ENDI(3, 1, 2, 0, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R3201	EMXX_DMAC_ENDI(3, 2, 0, 1, EMXX_DMAC_ENDI_R)
#define EMXX_DMAC_ENDI_R3210	EMXX_DMAC_ENDI(3, 2, 1, 0, EMXX_DMAC_ENDI_R)
/* Endian(Write) for 32bit */
#define EMXX_DMAC_ENDI_W0123	EMXX_DMAC_ENDI(0, 1, 2, 3, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W0132	EMXX_DMAC_ENDI(0, 1, 3, 2, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W0213	EMXX_DMAC_ENDI(0, 2, 1, 3, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W0231	EMXX_DMAC_ENDI(0, 2, 3, 1, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W0312	EMXX_DMAC_ENDI(0, 3, 1, 2, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W0321	EMXX_DMAC_ENDI(0, 3, 2, 1, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W1023	EMXX_DMAC_ENDI(1, 0, 2, 3, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W1032	EMXX_DMAC_ENDI(1, 0, 3, 2, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W1203	EMXX_DMAC_ENDI(1, 2, 0, 3, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W1230	EMXX_DMAC_ENDI(1, 2, 3, 0, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W1302	EMXX_DMAC_ENDI(1, 3, 0, 2, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W1320	EMXX_DMAC_ENDI(1, 3, 2, 0, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W2013	EMXX_DMAC_ENDI(2, 0, 1, 3, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W2031	EMXX_DMAC_ENDI(2, 0, 3, 1, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W2103	EMXX_DMAC_ENDI(2, 1, 0, 3, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W2130	EMXX_DMAC_ENDI(2, 1, 3, 0, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W2301	EMXX_DMAC_ENDI(2, 3, 0, 1, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W2310	EMXX_DMAC_ENDI(2, 3, 1, 0, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W3012	EMXX_DMAC_ENDI(3, 0, 1, 2, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W3021	EMXX_DMAC_ENDI(3, 0, 2, 1, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W3102	EMXX_DMAC_ENDI(3, 1, 0, 2, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W3120	EMXX_DMAC_ENDI(3, 1, 2, 0, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W3201	EMXX_DMAC_ENDI(3, 2, 0, 1, EMXX_DMAC_ENDI_W)
#define EMXX_DMAC_ENDI_W3210	EMXX_DMAC_ENDI(3, 2, 1, 0, EMXX_DMAC_ENDI_W)
/* Endian(Read) for 16bit */
#define EMXX_DMAC_ENDI_R01	EMXX_DMAC_ENDI_R2301
#define EMXX_DMAC_ENDI_R10	EMXX_DMAC_ENDI_R3210
/* Endian(Write) for 16bit */
#define EMXX_DMAC_ENDI_W01	EMXX_DMAC_ENDI_W2301
#define EMXX_DMAC_ENDI_W10	EMXX_DMAC_ENDI_W3210

/* for checking interrupt status */
#define EMXX_DMAC_INT_TIME_WR	0x80
#define EMXX_DMAC_INT_ERROR_WR	0x40
#define EMXX_DMAC_INT_BLOCK_WR	0x20
#define EMXX_DMAC_INT_LENG_WR	0x10
#define EMXX_DMAC_INT_TIME_RD	0x08
#define EMXX_DMAC_INT_ERROR_RD	0x04
#define EMXX_DMAC_INT_BLOCK_RD	0x02	/* Just indicate status
					   on PCH0, Cause INT
					   on only PCH2. */
#define EMXX_DMAC_INT_LENG_RD	0x01	/* Never cause INT
					   on any PCH's.
					   Just indicate status. */

/* for setting interrupt mask */
#define EMXX_DMAC_INT_TIME_EN	0x08
#define EMXX_DMAC_INT_ERROR_EN	0x04
#define EMXX_DMAC_INT_BLOCK_EN	0x02
#define EMXX_DMAC_INT_LENG_EN	0x01

typedef void	(*dma_callback_t)(void *data, int intsts, int intrawsts);


extern int		emxx_request_dma(int channel, const char *device_id,
				dma_callback_t callback, void *data,
				dma_regs_t **dma_regs);
extern void		emxx_free_dma(int channel);
extern int		emxx_start_dma(int channel, dma_addr_t src_ptr,
				u_int size, dma_addr_t dst_ptr, int intmask);
extern int		emxx_start_m2m_dma(
				unsigned int m2m_cont, unsigned int intmask);
extern int		emxx_dma_status(int channel);
extern int		emxx_dma_busy(void);
extern void		emxx_clear_dma(int channel);
extern void		emxx_reset_dma(int channel);
extern dma_addr_t	emxx_get_dma_pos(int channel);
extern void		emxx_stop_dma(int channel);
extern int		emxx_dma_writable_flag(int channel);

#endif /* __ASM_ARCH_EMXX_DMA_H */
