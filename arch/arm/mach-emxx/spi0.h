/*
 *  File Name	    : arch/arm/mach-emxx/spi0.h
 *  Function	    : SPI0 interface
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/08/09
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

#ifndef __ARCH_MACH_EMXX_SPI0_H
#define __ARCH_MACH_EMXX_SPI0_H

#include <mach/spi.h>

/*
 * SPI register
 */
#define SIO0_SWITCHEN		IO_ADDRESS(EMXX_SIO0_BASE + 0x00)
#define SIO0_MODE_SWITCH	IO_ADDRESS(EMXX_SIO0_BASE + 0x04)

#define SP0_ADDR  IO_ADDRESS(EMXX_SIO0_BASE + 0x1000)

#define SPx_TX_DATA_PHYS(ADDR)          ((ADDR) + 0x00000010)
#define SPx_RX_DATA_PHYS(ADDR)          ((ADDR) + 0x00000014)

#define SPx_CONTROL_TX_EMP              (0x01 << 15)
#define SPx_CONTROL_RX_FULL             (0x01 << 14)
#define SPx_CONTROL_RST                 (0x01 << 8)
#define SPx_CONTROL_TX_FULL             (0x01 << 7)
#define SPx_CONTROL_RX_EMP              (0x01 << 6)
#define SPx_CONTROL_WRT                 (0x01 << 3)
#define SPx_CONTROL_RD                  (0x01 << 2)
#define SPx_CONTROL_STOP                (0x01 << 1)
#define SPx_CONTROL_START               (0x01 << 0)

#define SPx_STATUS_TX_STOP              (0x01 << 6)
#define SPx_STATUS_RX_STOP              (0x01 << 5)
#define SPx_STATUS_TERR                 (0x01 << 4)
#define SPx_STATUS_RDV                  (0x01 << 3)
#define SPx_STATUS_END                  (0x01 << 2)
#define SPx_STATUS_TX_UDR               (0x01 << 1)
#define SPx_STATUS_RX_OVR               (0x01 << 0)
#define SPx_STATUS_TX_ALLERR            (SPx_STATUS_TX_UDR)
#define SPx_STATUS_RX_ALLERR            (SPx_STATUS_TERR \
					 | SPx_STATUS_RX_OVR)
#define SPx_STATUS_ALLERR               (SPx_STATUS_TX_ALLERR \
					 | SPx_STATUS_RX_ALLERR)

#define SPx_RAW_STATUS_TX_STOP_RAW      (0x01 << 6)
#define SPx_RAW_STATUS_RX_STOP_RAW      (0x01 << 5)
#define SPx_RAW_STATUS_TERR_RAW         (0x01 << 4)
#define SPx_RAW_STATUS_RDV_RAW          (0x01 << 3)
#define SPx_RAW_STATUS_END_RAW          (0x01 << 2)
#define SPx_RAW_STATUS_TX_UDR_RAW       (0x01 << 1)
#define SPx_RAW_STATUS_RX_OVR_RAW       (0x01 << 0)
#define SPx_RAW_STATUS_TX_ALLERR_RAW    (SPx_RAW_STATUS_TX_UDR_RAW)
#define SPx_RAW_STATUS_RX_ALLERR_RAW    (SPx_RAW_STATUS_TERR_RAW \
					 | SPx_RAW_STATUS_RX_OVR_RAW)
#define SPx_RAW_STATUS_ALLERR_RAW       (SPx_RAW_STATUS_TX_ALLERR_RAW \
					 | SPx_RAW_STATUS_RX_ALLERR_RAW)

#define SPx_ENSET_TX_STOP_EN            (0x01 << 6)
#define SPx_ENSET_RX_STOP_EN            (0x01 << 5)
#define SPx_ENSET_TERR_EN               (0x01 << 4)
#define SPx_ENSET_RDV_EN                (0x01 << 3)
#define SPx_ENSET_END_EN                (0x01 << 2)
#define SPx_ENSET_TX_UDR_EN             (0x01 << 1)
#define SPx_ENSET_RX_OVR_EN             (0x01 << 0)
#define SPx_ENSET_TX_ALLERR_EN          (SPx_ENSET_TX_UDR_EN)
#define SPx_ENSET_RX_ALLERR_EN          (SPx_ENSET_TERR_EN \
					 | SPx_ENSET_RX_OVR_EN)

#define SPx_ENCLR_TX_STOP_MASK          (0x01 << 6)
#define SPx_ENCLR_RX_STOP_MASK          (0x01 << 5)
#define SPx_ENCLR_TERR_MASK             (0x01 << 4)
#define SPx_ENCLR_RDV_MASK              (0x01 << 3)
#define SPx_ENCLR_END_MASK              (0x01 << 2)
#define SPx_ENCLR_TX_UDR_MASK           (0x01 << 1)
#define SPx_ENCLR_RX_OVR_MASK           (0x01 << 0)
#define SPx_ENCLR_TX_ALLERR_MASK        (SPx_ENCLR_TX_UDR_MASK)
#define SPx_ENCLR_RX_ALLERR_MASK        (SPx_ENCLR_TERR_MASK \
					 | SPx_ENCLR_RX_OVR_MASK)
#define SPx_ENCLR_TX_ALL_MASK           (SPx_ENCLR_TX_STOP_MASK	\
					 | SPx_ENCLR_END_MASK	\
					 | SPx_ENCLR_TX_ALLERR_MASK)
#define SPx_ENCLR_RX_ALL_MASK           (SPx_ENCLR_RX_STOP_MASK	\
					 | SPx_ENCLR_RDV_MASK	\
					 | SPx_ENCLR_RX_ALLERR_MASK)

#define SPx_FFCLR_TX_STOP_CLR           (0x01 << 6)
#define SPx_FFCLR_RX_STOP_CLR           (0x01 << 5)
#define SPx_FFCLR_TERR_CLR              (0x01 << 4)
#define SPx_FFCLR_RDV_CLR               (0x01 << 3)
#define SPx_FFCLR_END_CLR               (0x01 << 2)
#define SPx_FFCLR_TX_UDR_CLR            (0x01 << 1)
#define SPx_FFCLR_RX_OVR_CLR            (0x01 << 0)
#define SPx_FFCLR_ALL_CLR               (SPx_FFCLR_TX_STOP_CLR	 \
					 | SPx_FFCLR_RX_STOP_CLR \
					 | SPx_FFCLR_TERR_CLR	 \
					 | SPx_FFCLR_RDV_CLR	 \
					 | SPx_FFCLR_END_CLR	 \
					 | SPx_FFCLR_TX_UDR_CLR	 \
					 | SPx_FFCLR_RX_OVR_CLR)

#define SPx_CONTROL2_TX_STOP_MODE       (0x01 << 9)
#define SPx_CONTROL2_RX_STOP_MODE       (0x01 << 8)

#define SPx_CONTROL2_RX_FIFO_FULL_MASK  ((0xFF << 16) | (0xFF << 0))


/*
 * device info
 */
#define SPI_NAME                        "spi0"

/*
 * transfer status
 */
#define SPI_UNUSED                      0
#define SPI_READ                        1
#define SPI_WRITE                       2
#define SPI_RW                          (SPI_READ | SPI_WRITE)

/*
 * other define
 */
#define SPI_DMA_MAXSIZE                 0x00FFFFFC
#define SPI_DMA_BLOCK_MAXSIZE           0x0000FFFC

#define SPI_CSW_MASK                    0x0000F000      /* csw */
#define SPI_POL_MASK                    0x00000007      /* pol */

#define SPI_POL_SP0	\
	(((SPI_POL_SP0_CS7 & SPI_POL_MASK) << 25) | \
	((SPI_POL_SP0_CS6 & SPI_POL_MASK) << 22) | \
	((SPI_POL_SP0_CS5 & SPI_POL_MASK) << 19) | \
	((SPI_POL_SP0_CS4 & SPI_POL_MASK) << 16) | \
	((SPI_POL_SP0_CS3 & SPI_POL_MASK) << 9)  | \
	((SPI_POL_SP0_CS2 & SPI_POL_MASK) << 6)  | \
	((SPI_POL_SP0_CS1 & SPI_POL_MASK) << 3)  | \
	(SPI_POL_SP0_CS0))
/*
 * spi register info
 */
struct spi_regs {
	volatile unsigned int mode;             /* SPx_MODE       (0x00) */
	volatile unsigned int pol;              /* SPx_POL        (0x04) */
	volatile unsigned int control;          /* SPx_CONTROL    (0x08) */
	unsigned int:32;                       /* reserved       (0x0c) */
	volatile unsigned int tx_data;          /* SPx_TX_DATA    (0x10) */
	volatile unsigned int rx_data;          /* SPx_RX_DATA    (0x14) */
	volatile unsigned int status;           /* SPx_STATUS     (0x18) */
	volatile unsigned int raw_status;       /* SPx_RAW_STATUS (0x1c) */
	volatile unsigned int enset;            /* SPx_ENSET      (0x20) */
	volatile unsigned int enclr;            /* SPx_ENCLR      (0x24) */
	volatile unsigned int ffclr;            /* SPx_FFCLR      (0x28) */
	unsigned int:32;                       /* reserved       (0x2c) */
	unsigned int:32;                       /* reserved       (0x30) */
	volatile unsigned int control2;         /* SPx_CONTROL2   (0x34) */
	volatile unsigned int tiecs;            /* SPx_TIECS      (0x38) */
};

/*
 * buffer info
 */
struct spi_buf {
	dma_addr_t dma_addr;    /* physics address */
	unsigned int addr;      /* virtual address */
};

/*
 * transfer info
 */
struct spi_trans {
	dma_regs_t *dma_rx_regs;        /* dma rx register */
	dma_regs_t *dma_tx_regs;        /* dma tx register */
	int const rx_lch;               /* dma rx channel no */
	int const tx_lch;               /* dma tx channel no */
	unsigned int const rx_data;     /* rx data transfer address (physics) */
	unsigned int const tx_data;     /* tx data transfer address (physics) */
	unsigned int int_spi;           /* int spi no */
	unsigned int dma_err;           /* dma error status */
	unsigned int spi_err;           /* spi error status */
	unsigned int size;              /* transfer size */
	unsigned int state;             /* transfer status */
	wait_queue_head_t wait;         /* wait queue */
	spinlock_t spinlock;            /* spin lock */
	struct spi_buf buf;             /* buffer info */
};

/*
 * smu info
 */
struct spi_smu {
	unsigned int pclk;      /* pclk */
	unsigned int sclk;      /* sclk */
	unsigned int pclk_ctrl; /* pclk ctrl */
	unsigned int sclk_ctrl; /* sclk ctrl */
	unsigned int reset;     /* reset */
	unsigned int sclk_div;  /* SPIx_SCLK_DIV */
};

/*
 * spi data info
 */
struct spi_data {
	struct spi_regs *regs;          /* spi register info */
	struct spi_trans *trans;        /* transfer info */
	SPI_CONFIG *config;             /* spi config info */
	struct spi_smu *smu;            /* smu info */
	struct semaphore sem;           /* semaphore */
	unsigned int probe;             /* probe */
	unsigned int excl;              /* exclusive control method */
	unsigned int pol;               /* pol */
};

#endif	/* __ARCH_MACH_EMXX_SPI0_H */
