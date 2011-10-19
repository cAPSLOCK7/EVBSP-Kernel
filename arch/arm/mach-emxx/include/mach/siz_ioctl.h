/*
*  File Name       : arch/arm/mach-emxx/include/mach/siz_ioctl.h
*  Function        : User - Driver I/F of SIZ Driver
*  Release Version : Ver 1.00
*  Release Date    : 2010.02.16
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

#ifndef _SIZ_IOCTL_H_
#define _SIZ_IOCTL_H_

#include <mach/siz_common.h>
#include <mach/sizrot2_dma.h>

/*===========================================================================*/
/* logical device name of SIZ driver                                         */
/*===========================================================================*/
#define DEV_SIZ "/dev/siz"

/*===========================================================================*/
/* IOCTLs to SIZ driver                                                      */
/*===========================================================================*/
#define IOC_SIZ_MAGIC           ('N')
/* SIZ */
#define EMXX_REQUEST_SIZ \
	_IOR(IOC_SIZ_MAGIC, 10, struct emxx_siz_info)
#define EMXX_SET_SIZ \
	_IOW(IOC_SIZ_MAGIC, 11, struct emxx_set_siz_info)
#define EMXX_RESET_SIZ \
	_IOW(IOC_SIZ_MAGIC, 12, unsigned long)
#define EMXX_FREE_SIZ \
	_IOW(IOC_SIZ_MAGIC, 13, unsigned long)
/* DMA */
#define EMXX_SET_DMA_TO_SIZ \
	_IOW(IOC_SIZ_MAGIC, 30, struct emxx_set_dma_to_siz_info)
#define EMXX_START_DMA_TO_SIZ \
	_IOW(IOC_SIZ_MAGIC, 31, unsigned long)

/*===========================================================================*/
/* IOCTL : EMXX_SET_SIZ                                                      */
/*===========================================================================*/
struct emxx_set_siz_info {
	unsigned long         id;
	struct emxx_siz_param param;
};

/*===========================================================================*/
/* IOCTL : EMXX_SET_DMA_TO_SIZ                                               */
/*===========================================================================*/
struct emxx_set_dma_to_siz_info {
	unsigned long         id;
	struct emxx_dma_param param;
};

#endif /* _SIZ_IOCTL_H_ */
