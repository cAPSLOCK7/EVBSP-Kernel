/*
 *  File Name		: linux/drivers/mmc/host/emxx_sdio.h
 *  Function		: MMC
 *  Release Version	: Ver 1.04
 *  Release Date	: 2010/09/02
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

#ifndef __DRIVERS_MMC_EMXX_SDIO_H
#define __DRIVERS_MMC_EMXX_SDIO_H

#include <linux/io.h>

#define EMXX_MMC_SDIO0_BASE	IO_ADDRESS(EMXX_SDIO0_BASE)
#define EMXX_MMC_SDIO1_BASE	IO_ADDRESS(EMXX_SDIO1_BASE)
#ifdef CONFIG_MACH_EMEV
#define EMXX_MMC_SDIO2_BASE	IO_ADDRESS(EMXX_SDIO2_BASE)
#endif

#define SDIO_SYSADD		0x0000
#define SDIO_BLOCK		0x0004
#define SDIO_ARG		0x0008
#define SDIO_MODE_CMD		0x000C
#define SDIO_RSP01		0x0010
#define SDIO_RSP23		0x0014
#define SDIO_RSP45		0x0018
#define SDIO_RSP67		0x001C
#define SDIO_BUF		0x0020
#define SDIO_STATE		0x0024
#define SDIO_HP_BW		0x0028
#define SDIO_CLK_TOUT_RST	0x002C
#define SDIO_INT_STS		0x0030
#define SDIO_INT_STSEN		0x0034
#define SDIO_INT_SIGEN		0x0038
#define SDIO_CMD12_ERR		0x003C
#define SDIO_CONF		0x0040
#define SDIO_MAXREG		0x0048
#define SDIO_ERRINT_FORCE	0x0050
#define SDIO_ADMA_ERR		0x0054
#define SDIO_ADMA_SYSADD	0x0058
#define SDIO_CEATA		0x0080
#define SDIO_SLOTINT_STS	0x00FC
#define SDIO_AMBA0		0x0100
#define SDIO_AMBA1		0x0104

#define SDIO_DELAY		0xE000
#define SDIO_GIO0		0xE004
#define SDIO_GIO1		0xE008
#define SDIO_MODEN		0xF000

/* register set value define */
/* BLOCK(32bit) */
#define SDIO_BLK_COUNT(x)		((x) << 16)
#define SDIO_BLK_BOUND_4K		(0 << 12)
#define SDIO_BLK_BOUND_8K		(1 << 12)
#define SDIO_BLK_BOUND_16K		(2 << 12)
#define SDIO_BLK_BOUND_32K		(3 << 12)
#define SDIO_BLK_BOUND_64K		(4 << 12)
#define SDIO_BLK_BOUND_128K		(5 << 12)
#define SDIO_BLK_BOUND_256K		(6 << 12)
#define SDIO_BLK_BOUND_512K		(7 << 12)
#define SDIO_BLK_BOUND_MASK		(0x7 << 12)
#define SDIO_BLK_LENGTH(x)		((x) << 0)

/* MODE(16bit) + CMD(16bit) */
#define SDIO_MODE_MULTI			(1 << 5)
#define SDIO_MODE_READ			(1 << 4)
#define SDIO_MODE_ACMD12		(1 << 2)
#define SDIO_MODE_BLK_COUNT_EN		(1 << 1)
#define SDIO_MODE_DMA_EN		(1 << 0)

#define SDIO_CMD_INDEX(x)		((x & 0x3f) << 24)
#define SDIO_CMD_DATA			(1 << 21)
#define SDIO_CMD_INDEX_CHK		(1 << 20)
#define SDIO_CMD_CRC_CHK		(1 << 19)
#define SDIO_CMD_RESP_NONE		(0 << 16)
#define SDIO_CMD_RESP_136		(1 << 16)
#define SDIO_CMD_RESP_48		(2 << 16)
#define SDIO_CMD_RESP_48B		(3 << 16)

/* STATE(32bit) */
#define SDIO_STATE_DAT0			(1 << 20)
#define SDIO_STATE_WP			(1 << 19)
#define SDIO_STATE_CD			(1 << 18)
#define SDIO_STATE_STABLE		(1 << 17)
#define SDIO_STATE_INSERT		(1 << 16)
#define SDIO_STATE_RDEN			(1 << 11)
#define SDIO_STATE_WREN			(1 << 10)
#define SDIO_STATE_RD_ACTIVE		(1 << 9)
#define SDIO_STATE_WR_ACTIVE		(1 << 8)
#define SDIO_STATE_DAT_ACTIVE		(1 << 2)
#define SDIO_STATE_DAT_INHIBIT		(1 << 1)
#define SDIO_STATE_CMD_INHIBIT		(1 << 0)

/* HOST(8bit) + POWER(8bit) + WAKEUP(16bit) */
#define SDIO_HOST_CDSEL			(1 << 7)
#define SDIO_HOST_CDTL			(1 << 6)
#define SDIO_HOST_MMC8B			(1 << 5)
#define SDIO_HOST_SDMA			(0 << 3)
#define SDIO_HOST_ADMA1			(1 << 3)
#define SDIO_HOST_ADMA32		(2 << 3)
#define SDIO_HOST_ADMA64		(3 << 3)
#define SDIO_HOST_HS			(1 << 2)
#define SDIO_HOST_WIDTH			(1 << 1)
#define SDIO_HOST_LED			(1 << 0)

#define SDIO_POWER_VOLT_18		(5 << 9)
#define SDIO_POWER_VOLT_30		(6 << 9)
#define SDIO_POWER_VOLT_33		(7 << 9)
#define SDIO_POWER_POWER		(1 << 8)

/* CLKCTRL(16bit) + TIMEOUT(8bit) + SOFTRST(8bit) */
#define SDIO_CLK_CLKDIV1		(0x00 << 8)
#define SDIO_CLK_CLKDIV2		(0x01 << 8)
#define SDIO_CLK_CLKDIV4		(0x02 << 8)
#define SDIO_CLK_CLKDIV8		(0x04 << 8)
#define SDIO_CLK_CLKDIV16		(0x08 << 8)
#define SDIO_CLK_CLKDIV32		(0x10 << 8)
#define SDIO_CLK_CLKDIV64		(0x20 << 8)
#define SDIO_CLK_CLKDIV128		(0x40 << 8)
#define SDIO_CLK_CLKDIV256		(0x80 << 8)
#define SDIO_CLK_SDCLKEN		(1 << 2)
#define SDIO_CLK_CLKSTA			(1 << 1)
#define SDIO_CLK_CLKEN			(1 << 0)
#define SDIO_CLK_MASK			(0xffff << 0)

#define SDIO_TIMEOUT_COUNT_MIN		(0 << 16)
#define SDIO_TIMEOUT_COUNT_MAX		(0xE << 16)

#define SDIO_SOFTRST_DATA		(1 << 26)
#define SDIO_SOFTRST_CMD		(1 << 25)
#define SDIO_SOFTRST_ALL		(1 << 24)

/* INTERRUPTS(32bit) */
#define SDIO_INT_SDMA_ERR		(1 << 28)
#define SDIO_INT_ADMA_ERR		(1 << 25)
#define SDIO_INT_CMD12_ERR		(1 << 24)
#define SDIO_INT_DATA_END		(1 << 22)
#define SDIO_INT_DATA_CRC		(1 << 21)
#define SDIO_INT_DATA_TOUT		(1 << 20)
#define SDIO_INT_CMD_INDEX		(1 << 19)
#define SDIO_INT_CMD_END		(1 << 18)
#define SDIO_INT_CMD_CRC		(1 << 17)
#define SDIO_INT_CMD_TOUT		(1 << 16)
#define SDIO_INT_ERR			(1 << 15)
#define SDIO_INT_SDIO_INT		(1 << 8)
#define SDIO_INT_CARD_REM		(1 << 7)
#define SDIO_INT_CARD_INS		(1 << 6)
#define SDIO_INT_RREADY			(1 << 5)
#define SDIO_INT_WREADY			(1 << 4)
#define SDIO_INT_DMA			(1 << 3)
#define SDIO_INT_BGE			(1 << 2)
#define SDIO_INT_TRANCOMP		(1 << 1)
#define SDIO_INT_CMDCOMP		(1 << 0)

#define SDIO_INT_ALLERR			0x137f0000
#define SDIO_INT_MASK	\
		(SDIO_INT_ALLERR | SDIO_INT_CMDCOMP | \
		 SDIO_INT_WREADY | SDIO_INT_RREADY  | SDIO_INT_DMA)

/* AMBA0(32bit) */
#define SDIO_AMBA0_TMODE_INCR4		(0x00 << 0)
#define SDIO_AMBA0_TMODE_INCR8		(0x01 << 0)
#define SDIO_AMBA0_TMODE_INCR16		(0x02 << 0)
#define SDIO_AMBA0_TMODE_SINGLE		(0x04 << 0)

/* DELAY(32bit) */
#define SDIO_DELAY_REVERSE		(0x01 << 4)
#define SDIO_DELAY_0_0ns		(0x00 << 0)
#define SDIO_DELAY_0_5ns		(0x01 << 0)
#define SDIO_DELAY_1_0ns		(0x02 << 0)
#define SDIO_DELAY_1_5ns		(0x03 << 0)
#define SDIO_DELAY_2_0ns		(0x04 << 0)
#define SDIO_DELAY_2_5ns		(0x05 << 0)
#define SDIO_DELAY_3_0ns		(0x06 << 0)
#define SDIO_DELAY_3_5ns		(0x07 << 0)
#define SDIO_DELAY_4_0ns		(0x08 << 0)
#define SDIO_DELAY_4_5ns		(0x09 << 0)
#define SDIO_DELAY_5_0ns		(0x0a << 0)
#define SDIO_DELAY_5_5ns		(0x0b << 0)
#define SDIO_DELAY_6_0ns		(0x0c << 0)
#define SDIO_DELAY_6_5ns		(0x0d << 0)
#define SDIO_DELAY_7_0ns		(0x0e << 0)
#define SDIO_DELAY_7_5ns		(0x0f << 0)
#define SDIO_DELAY_MASK			(0x0f << 0)

/* MODEN(32bit) */
#define SDIO_MODEN_ENABLE		(0x01 << 0)

/* GIO0(32bit) */
#define SDIO_GIO0_HSENA			(0x01 << 31)
#define SDIO_GIO0_DELAYSEL		(0x01 << 30)
#define SDIO_GIO0_DETECT		(0x01 << 15)

/* GIO1(32bit) */
#define SDIO_GIO1_WP			(0x01 << 30)
#define SDIO_GIO1_INTSEL		(0x01 << 29)
#define SDIO_GIO1_SLOT_INT		(0x01 << 28)

#define EMXX_MMC_SMU_DIV3	(SMU_PLLSEL_PLL3 | SMU_DIV(3)) /* 76MHz */
#define EMXX_MMC_SMU_DIV5	(SMU_PLLSEL_PLL3 | SMU_DIV(5)) /* 46MHz */
#define EMXX_MMC_SMU_DIV6	(SMU_PLLSEL_PLL3 | SMU_DIV(6)) /* 38MHz */

#define EMXX_CHG_SDIO0_CKO	(1 << (50-32))	/* SDIO0_CKO -> GIO P50 */
#define EMXX_CHG_SDIO1_CKO	(1 << (61-32))	/* SDIO1_CKO -> GIO P61 */
#define EMXX_CHG_SDIO2_CKO	(1 << (97-96))	/* SDIO2_CKO -> GIO P97 */

#define EMXX_MAX_SEGS		128

#define SDIO_ADMA_BUF_SIZE	(EMXX_MAX_SEGS * 8 * 2)
#define SDIO_ADMA_ALIGN_SIZE	(EMXX_MAX_SEGS * 4)

/* Private structure */
struct emxx_mmc_host {
	u32 	base;
	int	pdev_id;
	int	irq;
	int	detect_irq;
	int	detect_gpio;
	int	wp_gpio;
	u32	clk;
	u32	sclk;
	u32	reset;
	u32	pin_sel;
	u32	pin_cko;
	u32	base_clock;
	u32	clkdiv;
	u32	normal_base;
	u32	init_base;
	int	connect;

	struct	mmc_host	*mmc;
	struct	mmc_request	*req;
	struct	mmc_command	*cmd;
	struct	mmc_data	*data;
	struct	mmc_command	*stop;

	struct	mutex		mutex;
	spinlock_t		lock;

	u8	bus_width;
	u8	timing;
	u32	power_mode;
	u32	clock;

	u32	data_len;
	u32	dma_num;
	u32	dma_dir;
	u32	dma_count;
	dma_addr_t	adma_addr;
	u8		*adma_virt_addr;
	dma_addr_t	adma_align_addr;
	u8		*adma_align_virt_addr;
#if defined(MMC_USE_SDMA) || defined(MMC_USE_CPUSEND)
	u32	dma_address[EMXX_MAX_SEGS];
	u32	dma_len[EMXX_MAX_SEGS];
#endif

	int	suspended;
	int	idle_suspend;
};

extern struct platform_driver emxx_mmc_driver2;

#endif /* __DRIVERS_MMC_EMXX_SDIO_H */
