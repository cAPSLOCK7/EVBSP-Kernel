/*
 *  File Name		: linux/drivers/ide/emxx_ide.h
 *  Function		: Compact Flash interface header files in IDE mode
 *  Release Version	: Ver 1.02
 *  Release Date	: 2010/08/06
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warrnty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; If not, write to the Free Software Foundation, Inc., 59
 *  Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#ifndef __EMXX_IDE_H
#define __EMXX_IDE_H

#include <mach/hardware.h>

#define EMXX_CFI_IDE_BASE	(IO_ADDRESS(EMXX_CFI_BASE))

/*CFI Registers */
#define EMXX_CFI_TASK_FILE	(EMXX_CFI_IDE_BASE + 0x01F0)
#define EMXX_CFI_CTL_MODE	(EMXX_CFI_IDE_BASE + 0x03F0)

#define	EMXX_CFI_CONTROL_0	(EMXX_CFI_IDE_BASE + 0xC000)
#define	EMXX_CFI_CONTROL_1	(EMXX_CFI_IDE_BASE + 0xC004)
#define EMXX_CFI_CONTROL_2	(EMXX_CFI_IDE_BASE + 0xC008)
#define EMXX_CFI_INTERRUPT	(EMXX_CFI_IDE_BASE + 0xC00C)
#define EMXX_CFI_STATUS		(EMXX_CFI_IDE_BASE + 0xC010)
#define EMXX_CFI_TIMING_0	(EMXX_CFI_IDE_BASE + 0xC014)
#define EMXX_CFI_TIMING_1	(EMXX_CFI_IDE_BASE + 0xC018)
#define EMXX_CFI_TIMING_2	(EMXX_CFI_IDE_BASE + 0xC01C)
#define EMXX_CFI_TIMING_3	(EMXX_CFI_IDE_BASE + 0xC020)
#define EMXX_CFI_VERSION_0	(EMXX_CFI_IDE_BASE + 0xC024)
#define EMXX_CFI_VERSION_1	(EMXX_CFI_IDE_BASE + 0xC028)
#define EMXX_CFI_DATA		(EMXX_CFI_IDE_BASE + 0xC02C)
#define EMXX_CFI_CRC		(EMXX_CFI_IDE_BASE + 0xC030)
#define EMXX_CFI_EXTENTION_0	(EMXX_CFI_IDE_BASE + 0xC034)
#define EMXX_CFI_EXTENTION_1	(EMXX_CFI_IDE_BASE + 0xC038)
#define EMXX_CFI_EXTENTION_2	(EMXX_CFI_IDE_BASE + 0xC03C)
#define EMXX_CFI_EXTENTION_3	(EMXX_CFI_IDE_BASE + 0xC040)
#define EMXX_CFI_EXTENTION_4	(EMXX_CFI_IDE_BASE + 0xC044)
#define EMXX_CFI_AMBASE		(EMXX_CFI_IDE_BASE + 0xC048)
#define EMXX_CFI_CMBASE		(EMXX_CFI_IDE_BASE + 0xC04C)
#define EMXX_CFI_IOBASE		(EMXX_CFI_IDE_BASE + 0xC050)
#define EMXX_CFI_RESET_CTRL	(EMXX_CFI_IDE_BASE + 0xE000)
#define EMXX_CFI_BUSIF_CTRL	(EMXX_CFI_IDE_BASE + 0xE004)
#define EMXX_CFI_TXMEM_ADDR	(EMXX_CFI_IDE_BASE + 0xE008)
#define EMXX_CFI_RXMEM_ADDR	(EMXX_CFI_IDE_BASE + 0xE00C)
#define EMXX_CFI_PIO_ADDR	(EMXX_CFI_IDE_BASE + 0xE010)
#define EMXX_CFI_PIO_SECTOR	(EMXX_CFI_IDE_BASE + 0xE014)
#define EMXX_CFI_SECTOR_LENGTH	(EMXX_CFI_IDE_BASE + 0xE018)
#define EMXX_CFI_BLOCK_LENGTH	(EMXX_CFI_IDE_BASE + 0xE01C)
#define EMXX_CFI_BLOCK_INDEX	(EMXX_CFI_IDE_BASE + 0xE020)
#define EMXX_CFI_TRANS_START	(EMXX_CFI_IDE_BASE + 0xE024)
#define EMXX_CFI_INT_MASK	(EMXX_CFI_IDE_BASE + 0xE030)
#define EMXX_CFI_INT_RAW	(EMXX_CFI_IDE_BASE + 0xE034)
#define EMXX_CFI_INT_ORG	(EMXX_CFI_IDE_BASE + 0xE038)
#define EMXX_CFI_INT_CLR	(EMXX_CFI_IDE_BASE + 0xE03C)

/*register bit defination*/
#define CFI_CONTROL_0_IDE		(1<<4)
#define CFI_CONTROL_0_IDEM_PIO		(0<<2)
#define CFI_CONTROL_0_IDEM_MULTI	(1<<2)
#define CFI_CONTROL_0_IDEM_ULTRA	(2<<2)
#define CFI_CONTROL_0_33V		(9<<8)
#define CFI_CONTROL_0_0V		(0<<8)
#define CFI_CONTROL_0_HRST		(1<<15)

#define CFI_BURST_MODE_0		0
#define CFI_BURST_MODE_1		1
#define CFI_BURST_MODE_2		2
#define CFI_BURST_MODE_3		3

/* defination for Control1 */
#define CFI_CONTROL1_AINTE	(1 << 7)
#define CFI_CONTROL1_OINTE	(1 << 6)
#define CFI_CONTROL1_UINTE	(1 << 5)
#define CFI_CONTROL1_BINTE	(1 << 4)
#define CFI_CONTROL1_CINTE	(1 << 3)
#define CFI_CONTROL1_RINTE	(1 << 2)
#define CFI_CONTROL1_DINTE	(1 << 1)
#define CFI_CONTROL1_EINTE	(1 << 0)

/*defination for interrupt */
#define CFI_INT_DBERR		(1 << 7)
#define CFI_INT_DOVER		(1 << 6)
#define CFI_INT_DUNR		(1 << 5)
#define CFI_INT_BVDS		(1 << 4)
#define CFI_INT_CDS		(1 << 3)
#define CFI_INT_RDYS		(1 << 2)
#define CFI_INT_DMAEND		(1 << 1)
#define CFI_INT_ENDFLG		(1 << 0)

/*defination for status */
#define CFI_STAT_RDY		(1 << 10)
#define CFI_STAT_WP		(1 << 9)
#define CFI_STAT_CD2		(1 << 8)
#define CFI_STAT_CD1		(1 << 7)
#define CFI_STAT_VS2		(1 << 6)
#define CFI_STAT_VS1		(1 << 5)
#define CFI_STAT_IPK		(1 << 4)
#define CFI_STAT_BVD2		(1 << 3)
#define CFI_STAT_BVD1		(1 << 2)
#define CFI_STAT_CVCC		(1 << 1)
#define CFI_STAT_CDV		(1 << 0)

#endif /*__EMXX_IDE_H */
