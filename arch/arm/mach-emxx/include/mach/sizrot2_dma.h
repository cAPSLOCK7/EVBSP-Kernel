/*
*  File Name       : arch/arm/mach-emxx/include/mach/sizrot2_dma.h
*  Function        : DMA I/F of SIZ/ROT Driver
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

#ifndef _SIZROT2_DMA_H_
#define _SIZROT2_DMA_H_

/*===========================================================================*/
/* DMA parameter information structure                                       */
/*===========================================================================*/
struct emxx_dma_param {
	unsigned long src_hsize;	/* pix  */
	unsigned long src_vsize;	/* pix  */
	unsigned long src_hskipyrgb;	/* byte */
	unsigned long src_hskipuv;	/* byte */
	unsigned long src_hskipv;	/* byte */
	unsigned long src_adryrgb;	/* phys */
	unsigned long src_adruv;	/* phys */
	unsigned long src_adrv;		/* phys */
	unsigned long src_format;
};

/*===========================================================================*/
/* Callback function type                                                    */
/*===========================================================================*/
typedef void (*dma_callback_func)(int flag);

/*===========================================================================*/
/* Definitions                                                               */
/*===========================================================================*/
/* emxx_dma_param.src_format   */
/* SIZ, ROT Random Mode */
#define M2M_DMA_FORMAT_RGB888	0
#define M2M_DMA_FORMAT_RGB565	1
#define M2M_DMA_FORMAT_YUV422IL	2
#define M2M_DMA_FORMAT_YUV422SP	3
#define M2M_DMA_FORMAT_YUV422PL	4
#define M2M_DMA_FORMAT_YUV420SP	5
#define M2M_DMA_FORMAT_YUV420PL	6
/* ROT Raster Order Mode */
#define M2M_DMA_FORMAT_RGB565_RASTER	2
#define M2M_DMA_FORMAT_RGB888_RASTER	3
#define M2M_DMA_FORMAT_YUV444_RASTER	3
#define M2M_DMA_FORMAT_ARGB8888_RASTER	4

/* flag of dma_callback_func    */
/* Success (DMA->SIZ->ROT process is completed) */
#define M2M_DMA_CALLBACK_SUCCESS	0
/* Error                                        */
#define M2M_DMA_CALLBACK_ERROR		-1

#endif /* _SIZROT2_DMA_H_ */
