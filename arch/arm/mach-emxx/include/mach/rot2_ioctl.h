/*
*  File Name       : arch/arm/mach-emxx/include/mach/rot2_ioctl.h
*  Function        : User - Driver I/F of ROT Driver
*  Release Version : Ver 1.00
*  Release Date    : 2010.02.23
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

#ifndef _ROT2_IOCTL_H_
#define _ROT2_IOCTL_H_

#include <mach/rot2_common.h>
#include <mach/sizrot2_dma.h>

/*===========================================================================*/
/* logical device name of ROT driver                                         */
/*===========================================================================*/
#define DEV_ROT_0 "/dev/rot0"

/*===========================================================================*/
/* IOCTLs to ROT driver                                                      */
/*===========================================================================*/
#define IOC_ROT_MAGIC           ('N')
/* ROT */
#define EMXX_REQUEST_ROT \
	_IOR(IOC_ROT_MAGIC, 20, struct emxx_rot_info)
#define EMXX_SET_ROT \
	_IOW(IOC_ROT_MAGIC, 21, struct emxx_set_rot_info)
#define EMXX_FREE_ROT \
	_IOW(IOC_ROT_MAGIC, 22, unsigned long)
/* DMA */
#define EMXX_SET_DMA_TO_ROT \
	_IOW(IOC_ROT_MAGIC, 40, struct emxx_set_dma_to_rot_info)
#define EMXX_START_DMA_TO_ROT \
	_IOW(IOC_ROT_MAGIC, 41, unsigned long)

/*===========================================================================*/
/* IOCTL : EMXX_SET_ROT                                                      */
/*===========================================================================*/
struct emxx_set_rot_info {
	unsigned long         id;
	struct emxx_rot_param param;
};

/*===========================================================================*/
/* IOCTL : EMXX_SET_DMA_TO_ROT                                               */
/*===========================================================================*/
struct emxx_set_dma_to_rot_info {
	unsigned long         id;
	struct emxx_dma_param param;
};

#endif /* _ROT2_IOCTL_H_ */
