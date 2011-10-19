/*
 *  File Name       : arch/arm/mach-emxx/include/mach/hardware.h
 *  Function        : hardware
 *  Release Version : Ver 1.03
 *  Release Date    : 2010/12/09
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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/io.h>

/* ************************************************************************
 *   EMXX address map
 * ***********************************************************************/

/* BANK 0/1 -- AB0 [External Memory/Device] */
#define EMXX_BANK0_BASE		0x00000000
#define EMXX_BANK1A_BASE	0x20000000
#define EMXX_BANK1B_BASE	0x30000000

#define EMXX_AB_BASE		0x2fff0000

/* CPU Local */
#define EMXX_CPU_LOCAL_BASE	0x1e000000
#define EMXX_CPU_LOCAL_SIZE	SZ_1M
#define EMXX_SCU_BASE		(EMXX_CPU_LOCAL_BASE + 0x0000)
#define EMXX_L2CC_BASE		(EMXX_CPU_LOCAL_BASE + 0xa000)

#define EMXX_CPU_LOCAL_VIRT	(IO_BASE + 0x03000000)
#define EMXX_SCU_VIRT		(EMXX_CPU_LOCAL_VIRT + 0x0000)
#define EMXX_L2CC_VIRT		(EMXX_CPU_LOCAL_VIRT + 0xa000)


/* DRAM */
#define EMXX_SDRAM_BASE		0x40000000

/* Image Trans */
#define EMXX_IMC_DATA_BASE	0xc0000000
#define EMXX_IMCW_DATA_BASE	0xc8000000
#define EMXX_ROT_DATA_BASE	0xd0000000
#define EMXX_SIZ_DATA_BASE	0xf8000000

/* SRAM */
#define EMXX_SRAM_BASE		0xf0000000
#define EMXX_SRAM_SIZE		SZ_128K
#define EMXX_SRAM_VIRT		(IO_BASE + 0x04000000)

/*
 * Peripherals
 */
#define EMXX_INTERNAL_IO_BASE1	0xe0000000	/* Shard Device */
#define EMXX_INTERNAL_IO_SIZE1	0x00020000	/* 128 KByte */
#define EMXX_INTERNAL_IO_BASE2	0xe0020000	/* Strong Ordered */
#define EMXX_INTERNAL_IO_SIZE2	0x00020000	/* 128 KByte */
#define EMXX_INTERNAL_IO_BASE3	0xe0040000	/* Shard Device */
#define EMXX_INTERNAL_IO_SIZE3	0x02FC0000	/* 47.75 MByte */

#define EMXX_TIMER0_BASE	0xe0000000
#define EMXX_TIMER1_BASE	0xe0000100
#define EMXX_TIMER2_BASE	0xe0000200
#define EMXX_TIMER3_BASE	0xe0000300
#define EMXX_WDT0_BASE		0xe0001000
#define EMXX_WDT1_BASE		0xe0001100
#define EMXX_WDT2_BASE		0xe0001200
#define EMXX_WDT3_BASE		0xe0001300
#define EMXX_WDT4_BASE		0xe0001400
#define EMXX_TG0_BASE		0xe0002000
#define EMXX_TG1_BASE		0xe0002100
#define EMXX_TG2_BASE		0xe0002200
#define EMXX_TG3_BASE		0xe0002300
#define EMXX_TG4_BASE		0xe0002400
#define EMXX_TG5_BASE		0xe0002500
#define EMXX_SIO1_BASE		0xe0010000
#define EMXX_INTA_CPU_BASE	0xe0020000
#define EMXX_INTA_DIST_BASE	0xe0028000
#define EMXX_INTA_TIM_BASE	0xe0030000
#define EMXX_LCD_BASE		0xe0040000
#define EMXX_GPIO_BASE		0xe0050000
#define EMXX_IIC0_BASE		0xe0070000
#define EMXX_MEMC_BASE		0xe00a0000
#define EMXX_AFS_BASE		0xe00c0000
#define EMXX_INTA_D_BASE	0xe00e0000
#define EMXX_PDMA_BASE		0xe00f0000
#define EMXX_PMU_BASE		0xe0100000
#define EMXX_SMU_BASE		0xe0110000
#define EMXX_SIO0_BASE		0xe0120000
#define EMXX_CHG_BASE		0xe0140000
#define EMXX_IRR_BASE		0xe0150000
#define EMXX_STI_BASE		0xe0180000
#ifdef CONFIG_MACH_EMEV
#define EMXX_CHG1_BASE		0xe1000000
#endif
#define EMXX_UART0_BASE		0xe1020000
#define EMXX_UART1_BASE		0xe1030000
#define EMXX_UART2_BASE		0xe1040000
#define EMXX_UART3_BASE		0xe1050000
#define EMXX_DMA_M2P_BASE	0xe1070000
#define EMXX_DMA_P2M_BASE	0xe1080000
#define EMXX_DMA_M2M_BASE	0xe1090000
#define EMXX_IIC11_BASE		0xe10a0000
#define EMXX_CAM_BASE		0xe10b0000
#define EMXX_SIO2_BASE		0xe10c0000
#define EMXX_SIO3_BASE		0xe10d0000
#ifdef CONFIG_MACH_EMEV
#define EMXX_SIO4_BASE		0xe10e0000
#define EMXX_SIO5_BASE		0xe10f0000
#endif
#define EMXX_PWM_BASE		0xe1130000
#ifdef CONFIG_MACH_EMEV
#define EMXX_HSI_BASE		0xe1140000
#endif
#define EMXX_DTV_BASE		0xe1150000
#define EMXX_SIZ_BASE		0xe1180000
#define EMXX_AVE_BASE		0xe1190000
#ifdef COFNIG_MACH_EMGR
#define EMXX_UART4_BASE		0xe11a0000
#define EMXX_UART5_BASE		0xe11b0000
#endif
#define EMXX_DCV_BASE		0xe1200000
#define EMXX_NTS_BASE		0xe1210000
#define EMXX_ROT_BASE		0xe1220000
#define EMXX_IMC_BASE		0xe1260000
#define EMXX_IMCW_BASE		0xe1270000
#ifdef CONFIG_MACH_EMEV
#define EMXX_CRP_BASE		0xe2000000
#endif
#define EMXX_SDC_BASE		0xe2100000
#define EMXX_CFI_BASE		0xe2200000
#define EMXX_MSP_BASE		0xe2300000
#define EMXX_MMM_BASE		0xe2400000
#define EMXX_USBS0_BASE		0xe2700000
#define EMXX_USBS1_BASE		0xe2800000
#define EMXX_SDIO0_BASE		0xe2900000
#define EMXX_SDIO1_BASE		0xe2a00000
#ifdef CONFIG_MACH_EMEV
#define EMXX_SDIO2_BASE		0xe2b00000
#define EMXX_A3D_BASE		0xe8000000
#endif
#ifdef COFNIG_MACH_EMGR
#define EMXX_A2D_BASE		0xe8000000
#endif

/* *********************
 *  EMXX Chip Revision
 * *********************/
#define EMXX_REV_MASK		0x000000ff
#define EMXX_REV_ES1		0x00000010
#define EMXX_REV_ES2		0x00000020
#define EMXX_REV_ES3		0x00000030

/*
 * serial base baud
 */
#ifdef CONFIG_EMXX_PLL3_238MHZ
/* PLL3 238MHZ */
#define EMXX_BASE_BAUD (237567000 / 2)
#else
/* PLL3 230MHZ */
#define EMXX_BASE_BAUD (229376000 / 2)
#endif

/*
 * Include platform-specific definition
 */
#if defined(CONFIG_EMEV_BOARD_EVA) || defined(CONFIG_EMGR_BOARD_EVA)
#include <mach/emev_board.h>
#endif

#endif /* __ASM_ARCH_HARDWARE_H */
