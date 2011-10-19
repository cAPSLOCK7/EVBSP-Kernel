/*
 *  File Name	    : arch/arm/mach-emxx/spi.h
 *  Function	    : SPI1 interface
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/02/05
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#ifndef __ARCH_MACH_EMXX_SPI_H
#define __ARCH_MACH_EMXX_SPI_H

#include <mach/spi.h>

/*
 * SPI register
 */
#define SP1_ADDR  IO_ADDRESS(EMXX_SIO2_BASE + 0x1000)

#define SPx_TX_DATA_PHYS(ADDR)		((ADDR) + 0x00001010)
#define SPx_RX_DATA_PHYS(ADDR)		((ADDR) + 0x00001014)

#define SPx_POL_VAL_SPI1		SPI_POL_VAL_SPI1

/* SPI_CONFIG.pol */
#define SPI_POL_CSW_16SCLK      (0x0F << 12)
#define SPI_POL_CSW_8SCLK       (0x07 << 12)
#define SPI_POL_CSW_4SCLK       (0x03 << 12)
#define SPI_POL_CSW_2SCLK       (0x01 << 12)

#define SPI_POL_CK7_DLY_ON      (0x01 << 27)
#define SPI_POL_CK7_DLY_OFF     (0x00 << 27)
#define SPI_POL_CK7_POL_NEG     (0x01 << 26)
#define SPI_POL_CK7_POL_POS     (0x00 << 26)
#define SPI_POL_CS7_POL_NEG     (0x01 << 25)
#define SPI_POL_CS7_POL_POS     (0x00 << 25)

#define SPI_POL_CK6_DLY_ON      (0x01 << 24)
#define SPI_POL_CK6_DLY_OFF     (0x00 << 24)
#define SPI_POL_CK6_POL_NEG     (0x01 << 23)
#define SPI_POL_CK6_POL_POS     (0x00 << 23)
#define SPI_POL_CS6_POL_NEG     (0x01 << 22)
#define SPI_POL_CS6_POL_POS     (0x00 << 22)

#define SPI_POL_CK5_DLY_ON      (0x01 << 21)
#define SPI_POL_CK5_DLY_OFF     (0x00 << 21)
#define SPI_POL_CK5_POL_NEG     (0x01 << 20)
#define SPI_POL_CK5_POL_POS     (0x00 << 20)
#define SPI_POL_CS5_POL_NEG     (0x01 << 19)
#define SPI_POL_CS5_POL_POS     (0x00 << 19)

#define SPI_POL_CK4_DLY_ON      (0x01 << 18)
#define SPI_POL_CK4_DLY_OFF     (0x00 << 18)
#define SPI_POL_CK4_POL_NEG     (0x01 << 17)
#define SPI_POL_CK4_POL_POS     (0x00 << 17)
#define SPI_POL_CS4_POL_NEG     (0x01 << 16)
#define SPI_POL_CS4_POL_POS     (0x00 << 16)

#define SPI_POL_CK3_DLY_ON      (0x01 << 11)
#define SPI_POL_CK3_DLY_OFF     (0x00 << 11)
#define SPI_POL_CK3_POL_NEG     (0x01 << 10)
#define SPI_POL_CK3_POL_POS     (0x00 << 10)
#define SPI_POL_CS3_POL_NEG     (0x01 << 9)
#define SPI_POL_CS3_POL_POS     (0x00 << 9)

#define SPI_POL_CK2_DLY_ON      (0x01 << 8)
#define SPI_POL_CK2_DLY_OFF     (0x00 << 8)
#define SPI_POL_CK2_POL_NEG     (0x01 << 7)
#define SPI_POL_CK2_POL_POS     (0x00 << 7)
#define SPI_POL_CS2_POL_NEG     (0x01 << 6)
#define SPI_POL_CS2_POL_POS     (0x00 << 6)

#define SPI_POL_CK1_DLY_ON      (0x01 << 5)
#define SPI_POL_CK1_DLY_OFF     (0x00 << 5)
#define SPI_POL_CK1_POL_NEG     (0x01 << 4)
#define SPI_POL_CK1_POL_POS     (0x00 << 4)
#define SPI_POL_CS1_POL_NEG     (0x01 << 3)
#define SPI_POL_CS1_POL_POS     (0x00 << 3)

#define SPI_POL_CK0_DLY_ON      (0x01 << 2)
#define SPI_POL_CK0_DLY_OFF     (0x00 << 2)
#define SPI_POL_CK0_POL_NEG     (0x01 << 1)
#define SPI_POL_CK0_POL_POS     (0x00 << 1)
#define SPI_POL_CS0_POL_NEG     (0x01 << 0)
#define SPI_POL_CS0_POL_POS     (0x00 << 0)

/* SPI_POL: default value */
#define SPI_POL_VAL_SPI1        (SPI_POL_CSW_8SCLK | \
		SPI_POL_CK7_DLY_OFF | SPI_POL_CK7_POL_NEG | \
		SPI_POL_CS7_POL_NEG | SPI_POL_CK6_DLY_OFF | \
		SPI_POL_CK6_POL_NEG | SPI_POL_CS6_POL_NEG | \
		SPI_POL_CK5_DLY_OFF | SPI_POL_CK5_POL_NEG | \
		SPI_POL_CS5_POL_NEG | SPI_POL_CK4_DLY_OFF | \
		SPI_POL_CK4_POL_NEG | SPI_POL_CS4_POL_NEG | \
		SPI_POL_CK3_DLY_OFF | SPI_POL_CK3_POL_NEG | \
		SPI_POL_CS3_POL_NEG | SPI_POL_CK2_DLY_OFF | \
		SPI_POL_CK2_POL_NEG | SPI_POL_CS2_POL_NEG | \
		SPI_POL_CK1_DLY_OFF | SPI_POL_CK1_POL_NEG | \
		SPI_POL_CS1_POL_NEG | SPI_POL_CK0_DLY_OFF | \
		SPI_POL_CK0_POL_NEG | SPI_POL_CS0_POL_NEG)


#define SPx_CONTROL_TX_EMP		(0x01 << 15)
#define SPx_CONTROL_RX_FULL		(0x01 << 14)
#define SPx_CONTROL_RST			(0x01 << 8)
#define SPx_CONTROL_TX_FULL		(0x01 << 7)
#define SPx_CONTROL_RX_EMP		(0x01 << 6)
#define SPx_CONTROL_WRT			(0x01 << 3)
#define SPx_CONTROL_RD			(0x01 << 2)
#define SPx_CONTROL_STOP		(0x01 << 1)
#define SPx_CONTROL_START		(0x01 << 0)

#define SPx_STATUS_TX_STOP		(0x01 << 6)
#define SPx_STATUS_RX_STOP		(0x01 << 5)
#define SPx_STATUS_TERR			(0x01 << 4)
#define SPx_STATUS_RDV			(0x01 << 3)
#define SPx_STATUS_END			(0x01 << 2)
#define SPx_STATUS_TX_UDR		(0x01 << 1)
#define SPx_STATUS_RX_OVR		(0x01 << 0)
#define SPx_STATUS_ALLERR		(SPx_STATUS_TERR | SPx_STATUS_TX_UDR | \
						SPx_STATUS_RX_OVR)

#define SPx_RAW_STATUS_TX_STOP_RAW	(0x01 << 6)
#define SPx_RAW_STATUS_RX_STOP_RAW	(0x01 << 5)
#define SPx_RAW_STATUS_TERR_RAW		(0x01 << 4)
#define SPx_RAW_STATUS_RDV_RAW		(0x01 << 3)
#define SPx_RAW_STATUS_END_RAW		(0x01 << 2)
#define SPx_RAW_STATUS_TX_UDR_RAW	(0x01 << 1)
#define SPx_RAW_STATUS_RX_OVR_RAW	(0x01 << 0)

#define SPx_ENSET_TX_STOP_EN		(0x01 << 6)
#define SPx_ENSET_RX_STOP_EN		(0x01 << 5)
#define SPx_ENSET_TERR_EN		(0x01 << 4)
#define SPx_ENSET_RDV_EN		(0x01 << 3)
#define SPx_ENSET_END_EN		(0x01 << 2)
#define SPx_ENSET_TX_UDR_EN		(0x01 << 1)
#define SPx_ENSET_RX_OVR_EN		(0x01 << 0)
#define SPx_ENSET_ALLERR_EN		(SPx_ENSET_TERR_EN | \
			SPx_ENSET_TX_UDR_EN | SPx_ENSET_RX_OVR_EN)

#define SPx_ENCLR_TX_STOP_MASK		(0x01 << 6)
#define SPx_ENCLR_RX_STOP_MASK		(0x01 << 5)
#define SPx_ENCLR_TERR_MASK		(0x01 << 4)
#define SPx_ENCLR_RDV_MASK		(0x01 << 3)
#define SPx_ENCLR_END_MASK		(0x01 << 2)
#define SPx_ENCLR_TX_UDR_MASK		(0x01 << 1)
#define SPx_ENCLR_RX_OVR_MASK		(0x01 << 0)
#define SPx_ENCLR_ALLERR_MASK		(SPx_ENCLR_TERR_MASK | \
			SPx_ENCLR_TX_UDR_MASK | SPx_ENCLR_RX_OVR_MASK)
#define SPx_ENCLR_ALL_MASK		(SPx_ENCLR_TX_STOP_MASK   \
					 | SPx_ENCLR_RX_STOP_MASK \
					 | SPx_ENCLR_TERR_MASK    \
					 | SPx_ENCLR_RDV_MASK     \
					 | SPx_ENCLR_END_MASK     \
					 | SPx_ENCLR_TX_UDR_MASK  \
					 | SPx_ENCLR_RX_OVR_MASK)

#define SPx_FFCLR_TX_STOP_CLR		(0x01 << 6)
#define SPx_FFCLR_RX_STOP_CLR		(0x01 << 5)
#define SPx_FFCLR_TERR_CLR		(0x01 << 4)
#define SPx_FFCLR_RDV_CLR		(0x01 << 3)
#define SPx_FFCLR_END_CLR		(0x01 << 2)
#define SPx_FFCLR_TX_UDR_CLR		(0x01 << 1)
#define SPx_FFCLR_RX_OVR_CLR		(0x01 << 0)
#define SPx_FFCLR_ALL_CLR		(SPx_FFCLR_TX_STOP_CLR   \
					 | SPx_FFCLR_RX_STOP_CLR \
					 | SPx_FFCLR_TERR_CLR    \
					 | SPx_FFCLR_RDV_CLR     \
					 | SPx_FFCLR_END_CLR     \
					 | SPx_FFCLR_TX_UDR_CLR  \
					 | SPx_FFCLR_RX_OVR_CLR)

#define SPx_CONTROL2_TX_STOP_MODE	(0x01 << 9)
#define SPx_CONTROL2_RX_STOP_MODE	(0x01 << 8)
#define SPx_CONTROL2_RX_FIFO_FULL_MASK	((0xFF << 16) | (0xFF << 0))

#define DEVNO_IS_INVALID(dev_no)	(dev_no != SPI_DEV_SP1)

/*
 * device info
 */
#define SPI_NAME			"spi"
#define SPI_DIR				"spi"

#if 0
/* SPI0 is Unsupported */
#define SPI_MINOR_SPI0			0
#endif
#define SPI_MINOR_SPI1			1

/*
 * transfer status
 */
#define SPI_STOP			0
#define SPI_ACTIVE			1

/*
 * buffer status
 */
#define SPI_BUFF_FULL			(0x01 << 0)
#define SPI_BUFF_EMPTY			(0x01 << 1)
#define SPI_BUFF_OVER_ERR		(0x01 << 2)

/*
 * other define
 */
#define SPI_BUFSIZE_MAX			65536	/* buffer size max [byte] */
#define SPI_BLKSIZE_MAX			32768	/* block size max [byte] */
#define SPI_BLKNUM_MAX			16	/* block number max */
#define SPI_BLKSIZE			1024	/* block size default [byte] */

#define SPI_OFF				0	/* clock/reset off */
#define SPI_ON				1	/* clock/reset on */
#define SPI_RESET			2	/* reset on->off */

#define SPI_FIFO_MAX			32	/* fifo count max */
#define SPI_DMA_BLOCK_SIZE_MAX		65534	/* dma block size max [byte] */

/*
 * spi register info
 */
struct spi_regs{
	volatile unsigned int mode;		/* SPx_MODE       (0x00) */
	volatile unsigned int pol;		/* SPx_POL        (0x04) */
	volatile unsigned int control;		/* SPx_CONTROL    (0x08) */
	unsigned int:32; 			/* reserved       (0x0c) */
	volatile unsigned int tx_data;		/* SPx_TX_DATA    (0x10) */
	volatile unsigned int rx_data;		/* SPx_RX_DATA    (0x14) */
	volatile unsigned int status;		/* SPx_STATUS     (0x18) */
	volatile unsigned int raw_status;	/* SPx_RAW_STATUS (0x1c) */
	volatile unsigned int enset;		/* SPx_ENSET      (0x20) */
	volatile unsigned int enclr;		/* SPx_ENCLR      (0x24) */
	volatile unsigned int ffclr;		/* SPx_FFCLR      (0x28) */
	unsigned int:32; 			/* reserved       (0x2c) */
	unsigned int:32; 			/* reserved       (0x30) */
	volatile unsigned int control2;		/* SPx_CONTROL2   (0x34) */
	volatile unsigned int tiecs;		/* SPx_TIECS      (0x38) */
};

/*
 * buffer info
 */
struct spi_buf_t{
	dma_addr_t dma_addr;	/* dma transfer start address (physics) */
	unsigned int top;	/* buffer top address (virtual) */
	unsigned int end;	/* buffer end address (virtual) */
	unsigned int dma_ptr;	/* buffer pointer for dma/cpu transfer */
	unsigned int usr_ptr;	/* buffer pointer for read/write */
	unsigned int blknum;	/* block number */
	unsigned int state;	/* buffer status */
};

/*
 * transfer info
 */
struct spi_trans_t{
	dma_regs_t *dma_regs;	/* dma_regs_t */
	int const rx_lch;	/* dma rx channel no */
	int const tx_lch;	/* dma tx channel no */
	unsigned int const rx_data;	/* rx data transfer address (physics) */
	unsigned int const tx_data;	/* tx data transfer address (physics) */
	unsigned int int_spi;	/* int spi no */
	unsigned int dma_err;	/* dma error status */
	unsigned int spi_err;	/* spi error status */
	unsigned int state;	/* transfer status */
	wait_queue_head_t wait;	/* wait queue */
	spinlock_t spinlock;	/* spin lock */
	struct spi_buf_t buf;		/* buffer info */
};

/*
 * smu info
 */
struct spi_smu_t{
	unsigned int pclk;	/* pclk */
	unsigned int sclk;	/* sclk */
	unsigned int pclk_ctrl;	/* pclk ctrl */
	unsigned int sclk_ctrl;	/* sclk ctrl */
	unsigned int reset;	/* reset */
	unsigned int div_sclk;	/* divsclk */
};

/*
 * spi data info
 */
struct spi_data_t{
	struct spi_regs   *regs;	/* spi register info */
	struct spi_mode_info *mode;	/* spi mode info */
	unsigned int pol;	/* spi pol info */
	struct spi_smu_t *smu;		/* smu info */
	struct spi_trans_t *trans;	/* transfer info */
	mode_t opened;		/* open flag */
	unsigned int k_flag;            /* kernel flag */
	struct semaphore sem_rw;        /* semaphore for open/release */
	struct semaphore sem_open;	/* semaphore for open/release */
};

#endif	/* __ARCH_MACH_EMXX_SPI_H */
