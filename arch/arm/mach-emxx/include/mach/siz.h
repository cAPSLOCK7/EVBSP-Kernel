/*
*  File Name       : arch/arm/mach-emxx/include/mach/siz.h
*  Function        : Driver - Driver I/F of SIZ Driver
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

#ifndef _SIZ_H_
#define _SIZ_H_

#include <mach/siz_common.h>
#include <mach/sizrot2_dma.h>

/*===========================================================================*/
/* extern declarations of public functions                                   */
/*===========================================================================*/
/* SIZ */
extern int emxx_request_siz(struct emxx_siz_info *arg);
extern int emxx_set_siz(unsigned long id, struct emxx_siz_param *arg);
extern int emxx_reset_siz(unsigned long id);
extern int emxx_free_siz(unsigned long id);
/* DMA */
extern int emxx_set_dma_to_siz(unsigned long id, struct emxx_dma_param *arg);
extern int emxx_start_dma_to_siz(unsigned long id, dma_callback_func callback);

#endif /* _SIZ_H_ */
