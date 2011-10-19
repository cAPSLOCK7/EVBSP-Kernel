/*
 *  File Name		: linux/drivers/mmc/host/emxx_sdc.h
 *  Function		: MMC
 *  Release Version	: Ver 1.02
 *  Release Date	: 2010/06/23
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#ifndef __DRIVERS_MMC_EMXX_SDC_H
#define __DRIVERS_MMC_EMXX_SDC_H

#define EMXX_MMC_BASE		(IO_ADDRESS(EMXX_SDC_BASE))

/* MMC Registers */
#define EMXX_MMC_CMD		(EMXX_MMC_BASE + 0x00)
#define EMXX_MMC_ARG0		(EMXX_MMC_BASE + 0x08)
#define EMXX_MMC_ARG1		(EMXX_MMC_BASE + 0x0C)
#define EMXX_MMC_STOP		(EMXX_MMC_BASE + 0x10)
#define EMXX_MMC_SECCNT		(EMXX_MMC_BASE + 0x14)
#define EMXX_MMC_RSP0		(EMXX_MMC_BASE + 0x18)
#define EMXX_MMC_RSP1		(EMXX_MMC_BASE + 0x1C)
#define EMXX_MMC_RSP2		(EMXX_MMC_BASE + 0x20)
#define EMXX_MMC_RSP3		(EMXX_MMC_BASE + 0x24)
#define EMXX_MMC_RSP4		(EMXX_MMC_BASE + 0x28)
#define EMXX_MMC_RSP5		(EMXX_MMC_BASE + 0x2C)
#define EMXX_MMC_RSP6		(EMXX_MMC_BASE + 0x30)
#define EMXX_MMC_RSP7		(EMXX_MMC_BASE + 0x34)
#define EMXX_MMC_INFO1		(EMXX_MMC_BASE + 0x38)
#define EMXX_MMC_INFO2		(EMXX_MMC_BASE + 0x3C)
#define EMXX_MMC_INFO1_MASK	(EMXX_MMC_BASE + 0x40)
#define EMXX_MMC_INFO2_MASK	(EMXX_MMC_BASE + 0x44)
#define EMXX_MMC_CLK_CTRL	(EMXX_MMC_BASE + 0x48)
#define EMXX_MMC_SIZE		(EMXX_MMC_BASE + 0x4C)
#define EMXX_MMC_OPTION		(EMXX_MMC_BASE + 0x50)
#define EMXX_MMC_ERR_STS1	(EMXX_MMC_BASE + 0x58)
#define EMXX_MMC_ERR_STS2	(EMXX_MMC_BASE + 0x5C)
#define EMXX_MMC_BUF0		(EMXX_MMC_BASE + 0x60)
#define EMXX_SDIO_MODE		(EMXX_MMC_BASE + 0x68)
#define EMXX_SDIO_INFO1		(EMXX_MMC_BASE + 0x6C)
#define EMXX_SDIO_INFO1_MASK	(EMXX_MMC_BASE + 0x70)

#define EMXX_MMC_CC_EXT_MODE	(EMXX_MMC_BASE + 0x1B0)
#define EMXX_MMC_SOFT_RST	(EMXX_MMC_BASE + 0x1C0)

#define EMXX_MMC_USER		(EMXX_MMC_BASE + 0x200)
#define EMXX_MMC_USER2		(EMXX_MMC_BASE + 0x204)

#define EMXX_MMC_RST_CTRL	(EMXX_MMC_BASE + 0x210)
#define EMXX_MMC_BUSIF_CTRL	(EMXX_MMC_BASE + 0x214)
#define EMXX_MMC_ERR_MHADDR_L	(EMXX_MMC_BASE + 0x218)
#define EMXX_MMC_DMAMSK_CTRL	(EMXX_MMC_BASE + 0x21C)
#define EMXX_MMC_TXMEM_ADDR0L	(EMXX_MMC_BASE + 0x220)
#define EMXX_MMC_RXMEM_ADDR0L	(EMXX_MMC_BASE + 0x228)
#define EMXX_MMC_SECTOR_LEN0	(EMXX_MMC_BASE + 0x230)
#define EMXX_MMC_BLOCK_LEN	(EMXX_MMC_BASE + 0x240)
#define EMXX_MMC_TRANS_START	(EMXX_MMC_BASE + 0x260)
#define EMXX_MMC_INT_MASK	(EMXX_MMC_BASE + 0x270)
#define EMXX_MMC_INT_RAW	(EMXX_MMC_BASE + 0x274)
#define EMXX_MMC_INT_ORG	(EMXX_MMC_BASE + 0x278)
#define EMXX_MMC_INT_CLR	(EMXX_MMC_BASE + 0x27C)
#define EMXX_MMC_ERR_MHADDR_H	(EMXX_MMC_BASE + 0x288)
#define EMXX_MMC_TXMEM_ADDR0H	(EMXX_MMC_BASE + 0x290)
#define EMXX_MMC_RXMEM_ADDR0H	(EMXX_MMC_BASE + 0x298)

/* Command */
#define MMC_CMD_ACMD		(1 << 6)
#define MMC_CMD_RSP_NONE	(3 << 8)
#define MMC_CMD_RSP_R1		(4 << 8)
#define MMC_CMD_RSP_R1B		(5 << 8)
#define MMC_CMD_RSP_R2		(6 << 8)
#define MMC_CMD_RSP_R3		(7 << 8)
#define MMC_CMD_DATA		(1 << 11)
#define MMC_CMD_READ		(1 << 12)
#define MMC_CMD_MULTI		(1 << 13)

/* Stop */
#define MMC_STOP_STOP		(1 << 0)
#define MMC_STOP_MULTI		(1 << 8)

/* Interrupts */
#define MMC_INFO_EOC		(1 << 0)
#define MMC_INFO_RWEND		(1 << 2)

#define MMC_INFO_CMDERR		(1 << (0+16))
#define MMC_INFO_CRCERR		(1 << (1+16))
#define MMC_INFO_ENDERR		(1 << (2+16))
#define MMC_INFO_DTO		(1 << (3+16))
#define MMC_INFO_ILAW		(1 << (4+16))
#define MMC_INFO_ILAR		(1 << (5+16))
#define MMC_INFO_CTO		(1 << (6+16))
#define MMC_INFO_DAT0		(1 << (7+16))
#define MMC_INFO_BRE		(1 << (8+16))
#define MMC_INFO_BWE		(1 << (9+16))
#define MMC_INFO_CB		(1 << (14+16))
#define MMC_INFO_ILA		(1 << (15+16))

#define MMC_INFO_ALLERR		0x807F0000
#define MMC_INFO_MASK		0x0300FFFE

#define MMC_SDIO_INFO1_MASK	0xFFFF

/* Clock */
#define MMC_CLOCK_DIV2		(0 << 0)
#define MMC_CLOCK_DIV4		(1 << 0)
#define MMC_CLOCK_DIV8		(2 << 0)
#define MMC_CLOCK_DIV16		(4 << 0)
#define MMC_CLOCK_DIV32		(8 << 0)
#define MMC_CLOCK_DIV64		(16 << 0)
#define MMC_CLOCK_DIV128	(32 << 0)
#define MMC_CLOCK_DIV256	(64 << 0)
#define MMC_CLOCK_DIV512	(128 << 0)
#define MMC_CLOCK_EN		(1 << 8)
#define MMC_CLOCK_OFFEN		(1 << 9)
#define MMC_CLOCK_HIGH		(1 << 10)
#define MMC_CLOCK_DIV_MASK	0xFF

/* Option */
#define MMC_OPTION_WIDTH4	(1 << 15) /* 0:WIDTH14 1:WIDTH1 */

/* Buffer mode */
#define MMC_CC_EXT_MODE_FIFO	0x0000
#define MMC_CC_EXT_MODE_SD_DMA	0x0002

/* Reset */
#define MMC_SOFT_RST_RESET	0x0000
#define MMC_SOFT_RST_UNRESET	0x0007

/* RST_CTRL */
#define MMC_RST_CTRL_RESET	0x0007
#define MMC_RST_CTRL_UNRESET	0x0000

/* BUSIF_CTRL */
#define MMC_BUSIF_MODE_SINGLE	(0 << 1)
#define MMC_BUSIF_MODE_INCR4	(1 << 1)
#define MMC_BUSIF_MODE_INCR8	(2 << 1)
#define MMC_BUSIF_MODE_INCR16	(3 << 1)
#define MMC_BUSIF_DMA_ENABLE	(1 << 0)
#define MMC_BUSIF_DMA_DISABLE	(0 << 0)

/* DMAMSK_CTRL */
#define MMC_DMAMSK_W		(1 << 1)
#define MMC_DMAMSK_R		(1 << 0)
#define MMC_DMAMSK_MASK		(3 << 0)

/* DMA_INT */
#define MMC_DMAINT		(1 << 0)
#define MMC_DMAERR		(1 << 3)
#define MMC_DMAINT_ALL		(MMC_DMAINT | MMC_DMAERR)

#ifdef CONFIG_MACH_EMGR
#define MMC_BASE_CLOCK		88000000
#else	/* CONFIG_MACH_EMEV) */
#define MMC_BASE_CLOCK		133000000
#endif

#define EMXX_MAX_SEGS	128	/* MAX 64KByte */

/* Private structure */
struct emxx_sdc_host {
	struct	mmc_host	*mmc;
	struct	mmc_request	*req;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	mmc_command	*stop;

	struct	mutex		mutex;
	spinlock_t		lock;

	u8		bus_width;
	u8		app_mode;
	u32		power_mode;
	u32		clock;
	u32		base_clock;

	u32		info_mask;

	u32		data_len;
	u32		dma_dir;
	u32		dma_num;
	u32		dma_count;
	u32		dma_len[EMXX_MAX_SEGS];
	u32		dma_address[EMXX_MAX_SEGS];

	int		suspended;
	int		irq;
	int		detect_irq;
	int		connect;
};

#endif /* __DRIVERS_MMC_EMXX_SDC_H */
