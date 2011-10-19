/*
*  File Name       : arch/arm/mach-emxx/include/mach/rot2.h
*  Function        : Driver - Driver I/F of ROT Driver
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

#ifndef _ROT2_H_
#define _ROT2_H_

#include <mach/rot2_common.h>
#include <mach/sizrot2_dma.h>

/*===========================================================================*/
/* extern declarations of public functions                                   */
/*===========================================================================*/
/* ROT */
extern int emxx_request_rot(int channel, struct emxx_rot_info *arg);
extern int emxx_set_rot(unsigned long id, struct emxx_rot_param *arg);
extern int emxx_free_rot(unsigned long id);
/* DMA */
extern int emxx_set_dma_to_rot(unsigned long id, struct emxx_dma_param *arg);
extern int emxx_start_dma_to_rot(unsigned long id, dma_callback_func callback);

/*===========================================================================*/
/* Definitions                                                               */
/*===========================================================================*/
/* channel of emxx_request_rot */
#define ROT_LCH0                0

#endif /* _ROT2_H_ */
