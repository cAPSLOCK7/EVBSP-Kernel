/*
 *  File Name	    : arch/arm/mach-emxx/include/mach/spi.h
 *  Function	    : EMMA Mobile series SPI interface
 *  Release Version : Ver 1.02
 *  Release Date    : 2010/12/24
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

#ifndef __ASM_ARCH_EMXX_SPI_H
#define __ASM_ARCH_EMXX_SPI_H

/* SPI_CONFIG */
struct spi_config_t{
	unsigned char dev;      /* device */
	unsigned int pol;       /* poll */
	unsigned int sclk;      /* clock */

/* spi_write_without_intr only */
	unsigned int size;      /* byte size */
	unsigned int mode;      /* mode */

/* spi_read/spi_write/spi_cmd_read only */
	unsigned int nbr;       /* bit length (read) */
	unsigned int nbw;       /* bit length (write) */
	unsigned int cs_sel;    /* chip select */
	unsigned int m_s;       /* master/slave */
	unsigned int dma;       /* dma on/off */
	unsigned int tiecs;     /* tiecs */
};

#define SPI_CONFIG struct spi_config_t

/* SPI_CONFIG.nbr/nbw (8 to 32) */
/* spi_mode_t.nb */
#define SPI_NB_8BIT              8
#define SPI_NB_16BIT            16
#define SPI_NB_24BIT            24
#define SPI_NB_32BIT            32

/* SPI_CONFIG.cs_sel */
/* spi_mode_t.cs_sel */
#define SPI_CS_SEL_CS0          0
#define SPI_CS_SEL_CS1          1
#define SPI_CS_SEL_CS2          2
#define SPI_CS_SEL_CS3          3
#define SPI_CS_SEL_CS4          4
#define SPI_CS_SEL_CS5          5
#define SPI_CS_SEL_CS6          6
#define SPI_CS_SEL_CS7          7

/* SPI_CONFIG.m_s */
/* spi_mode_t.m_s */
#define SPI_M_S_MASTER          0
#define SPI_M_S_SLAVE           1

/* SPI_CONFIG.dma */
/* spi_mode_t.dma */
#define SPI_DMA_OFF             0
#define SPI_DMA_ON              1

/* SPI_CONFIG.sclk */
/* spi_mode_t.sclk */
#define SPI_SCLK_1500KHZ        0
#define SPI_SCLK_3MHZ           1
#define SPI_SCLK_6MHZ           2
#define SPI_SCLK_12MHZ          3
#define SPI_SCLK_24MHZ          4
#define SPI_SCLK_48MHZ          5

/* SPI_CONFIG.tiecs */
#define SPI_TIECS_NORMAL        0
#define SPI_TIECS_FIXED         1

/* flags */
#define SPI_BLOCK               0
#define SPI_NONBLOCK            1
#define SPI_RW_2CYCLE           2

/*
 * spi mode info
 */
struct spi_mode_info{
	unsigned int nb;        /* bit length */
	unsigned int cs_sel;    /* chip select */
	unsigned int m_s;       /* master/slave */
	unsigned int dma;       /* dma on/off */
	unsigned int blksize;   /* block size */
	unsigned int sclk;      /* clock */
};
#define spi_mode_t struct spi_mode_info

/* read/write mode*/
#define SPI_READ_MODE      1
#define SPI_WRITE_MODE     2

/*
 * operation command
 */
#define SPI_CMD_RX_START        _IO('s', 0x01)
#define SPI_CMD_RX_STOP         _IO('s', 0x02)

/*
 * setting command
 */
#define SPI_CMD_GET_MODE        _IOR('s', 0x11, spi_mode_t)
#define SPI_CMD_SET_MODE        _IOW('s', 0x12, spi_mode_t)


extern int emxx_spi_read(unsigned char dev, char *buf,
			  unsigned int count, unsigned char block_mode);

extern int emxx_spi_write(unsigned char dev, const char *buf,
			   unsigned int count, unsigned char block_mode);

extern int emxx_spi_init(unsigned char dev, unsigned char rw_mode);

extern int emxx_spi_end(unsigned char dev);

extern int emxx_spi_setmode(unsigned char dev, spi_mode_t *mode);

extern int emxx_spi_getmode(unsigned char dev, spi_mode_t *mode);


/* SPI_CONFIG.dev */
#define SPI_DEV_SP0             0
#define SPI_DEV_SP1             1
#define SPI_DEV_SP2             2

/* SPI_CONFIG.pol */
#define SPI_CSW_1CLK            (0 << 12)
#define SPI_CSW_2CLK            (1 << 12)
#define SPI_CSW_4CLK            (3 << 12)
#define SPI_CSW_8CLK            (7 << 12)
#define SPI_CSW_16CLK           (15 << 12)

#define SPI_CK_DLY_OFF          (0 << 2)
#define SPI_CK_DLY_ON           (1 << 2)

#define SPI_CK_POL_POS          (0 << 1)
#define SPI_CK_POL_NEG          (1 << 1)

#define SPI_CS_POL_POS          (0 << 0)
#define SPI_CS_POL_NEG          (1 << 0)

/* SPI_CONFIG.pol : default value */
#define SPI_POL_SP0_CS0		\
	(SPI_CSW_8CLK | SPI_CK_DLY_ON | SPI_CK_POL_POS | SPI_CS_POL_POS)
#define SPI_POL_SP0_CS1		\
	(SPI_CSW_8CLK | SPI_CK_DLY_ON | SPI_CK_POL_POS | SPI_CS_POL_NEG)
#define SPI_POL_SP0_CS2		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_POS | SPI_CS_POL_NEG)
#ifdef CONFIG_EMGR_TI_PMIC
#define SPI_POL_SP0_CS3		\
	(SPI_CSW_8CLK | SPI_CK_DLY_ON | SPI_CK_POL_POS | SPI_CS_POL_POS)
#else
#define SPI_POL_SP0_CS3		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_POS | SPI_CS_POL_POS)
#endif
#define SPI_POL_SP0_CS4		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_POS | SPI_CS_POL_POS)
#define SPI_POL_SP0_CS5		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_POS | SPI_CS_POL_POS)
#define SPI_POL_SP0_CS6		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_POS | SPI_CS_POL_POS)
#define SPI_POL_SP0_CS7		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_POS | SPI_CS_POL_POS)

#define SPI_POL_SP1_CS0		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS1		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS2		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS3		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS4		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS5		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS6		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP1_CS7		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)

#define SPI_POL_SP2_CS0		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS1		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS2		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS3		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS4		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS5		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS6		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)
#define SPI_POL_SP2_CS7		\
	(SPI_CSW_8CLK | SPI_CK_DLY_OFF | SPI_CK_POL_NEG | SPI_CS_POL_NEG)

extern int spi_read(SPI_CONFIG *config, char *buf, unsigned long phys,
		    unsigned int count, unsigned int flags);
extern int spi_write(SPI_CONFIG *config, char *buf, unsigned long phys,
		     unsigned int count, unsigned int flags);
extern int spi_cmd_read(SPI_CONFIG *config, char *cmd, char *buf,
			unsigned int flags);

#endif	/* __ASM_ARCH_EMXX_SPI_H */
