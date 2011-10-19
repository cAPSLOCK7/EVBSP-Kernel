/*
*  File Name       : arch/arm/mach-emxx/include/mach/rot2_common.h
*  Function        : ROT Driver I/F and ROT MMIO definitions
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

#ifndef _ROT2_COMMON_H_
#define _ROT2_COMMON_H_

/*===========================================================================*/
/* ROT Driver Structures                                                     */
/*===========================================================================*/

/* ROT resource information */
struct emxx_rot_info {
	unsigned long id;
	/* Input address for Source Image */
	unsigned long adryrgb;		/* phys */
	unsigned long adruv;		/* phys */
	unsigned long adrv;		/* phys */
	unsigned char device;		/* request device id */
	unsigned int  timeout;		/* timeout time */
};

/* ROT parameter information */
struct emxx_rot_param {
	unsigned long mode;		/* ROT2_LCHx_MODE       */
	unsigned long src_hsize;	/* ROT2_LCHx_SRCHSIZE   */
	unsigned long src_vsize;	/* ROT2_LCHx_SRCVSIZE   */
	unsigned long src_format;	/* ROT2_LCHx_SRCFMT     */
	unsigned long dst_adryrgb;	/* ROT2_LCHx_DSTADRYRGB */
	unsigned long dst_adruv;	/* ROT2_LCHx_DSTADRUV   */
	unsigned long dst_adrv;		/* ROT2_LCHx_DSTADRV    */
	unsigned long dst_bytelane;	/* ROT2_LCHx_DSTBL      */
	unsigned long input_mode;	/* 0: Random Mode or
					   1: Raster Order Mode */
};

/*===========================================================================*/
/* Definitions                                                               */
/*===========================================================================*/

/* Alarm Data : func */
#ifndef FUNC_SIZ
#define FUNC_SIZ	1
#define FUNC_ROT_CH0	2
#endif

/* Alarm Data : dev_now, dev_err */
/* struct emxx_rot_info.device */
#ifndef DEV_FB
#define DEV_FB		1
#define DEV_LCD		2
#define DEV_DSP		3
#define DEV_CAM		4
#define DEV_OTHER	5
#endif

/* struct emxx_rot_param.input_mode */
#define RANDOM_MODE	0
#define RASTER_MODE	1

/*===========================================================================*/
/* ROT register definitions                                                  */
/*===========================================================================*/
#define ROT2_LCH0_MODE		0x00
#define ROT2_LCH0_SRCHSIZE	0x04
#define ROT2_LCH0_SRCVSIZE	0x08
#define ROT2_LCH0_SRCFMT	0x0C
#define ROT2_LCH0_DSTADRYRGB	0x10
#define ROT2_LCH0_DSTADRUV	0x14
#define ROT2_LCH0_DSTADRV	0x18
#define ROT2_LCH0_DSTBL		0x1C
#define ROT2_HISTCTRL		0x40
#define ROT2_HISTCLR		0x44
#define ROT2_HIST0		0x48
#define ROT2_HIST1		0x4C
#define ROT2_HIST2		0x50
#define ROT2_HIST3		0x54
#define ROT2_HIST4		0x58
#define ROT2_HIST5		0x5C
#define ROT2_HIST6		0x60
#define ROT2_HIST7		0x64

/*===========================================================================*/
/* offset from ROT2_LCHx_MODE                                                */
/*===========================================================================*/
#define ROT2_LCHx_MODE		0x00
#define ROT2_LCHx_SRCHSIZE	0x04
#define ROT2_LCHx_SRCVSIZE	0x08
#define ROT2_LCHx_SRCFMT	0x0C
#define ROT2_LCHx_DSTADRYRGB	0x10
#define ROT2_LCHx_DSTADRUV	0x14
#define ROT2_LCHx_DSTADRV	0x18
#define ROT2_LCHx_DSTBL		0x1C

/*===========================================================================*/
/* ROT register bit assigns                                                  */
/*===========================================================================*/

/*------------------------------*/
/* ROT2_LCH0_MODE               */
/*------------------------------*/
#define ROT2_MODE_MODE_BIT	0x00000003
#define ROT2_MODE_MODE_SFT	0
#define ROT2_MODE_MODE_0	0x0
#define ROT2_MODE_MODE_90	0x1
#define ROT2_MODE_MODE_180	0x2
#define ROT2_MODE_MODE_270	0x3

#define ROT2_MODE_XMIRROR_BIT	0x00000004
#define ROT2_MODE_XMIRROR_SFT	2
#define ROT2_MODE_XMIRROR	0x4

#define ROT2_MODE_YMIRROR_BIT	0x00000008
#define ROT2_MODE_YMIRROR_SFT	3
#define ROT2_MODE_YMIRROR	0x8

#define ROT2_MODE_BOUNDARY_BIT	0x00000030
#define ROT2_MODE_BOUNDARY_SFT	4
#define ROT2_MODE_BOUNDARY_2_12	0x00
#define ROT2_MODE_BOUNDARY_2_13	0x10
#define ROT2_MODE_BOUNDARY_2_14	0x20
#define ROT2_MODE_BOUNDARY_2_15	0x30

/*------------------------------*/
/* ROT2_LCH0_SRCFMT             */
/*------------------------------*/
#ifndef ROT2_SRCFMT_BIT
#define ROT2_SRCFMT_BIT		0x00000007
#define ROT2_SRCFMT_SFT		0
/* Random Mode */
#define ROT2_FORMAT_RGB565	1
#define ROT2_FORMAT_YUV422IL	2
#define ROT2_FORMAT_YUV422SP	3
#define ROT2_FORMAT_YUV422PL	4
#define ROT2_FORMAT_YUV420SP	5
#define ROT2_FORMAT_YUV420PL	6
/* Raster Order Mode */
#define ROT2_FORMAT_RGB565_RASTER	2
#define ROT2_FORMAT_RGB888_RASTER	3
#define ROT2_FORMAT_YUV444_RASTER	3
#define ROT2_FORMAT_ARGB8888_RASTER	4
#endif /* ROT2_SRCFMT_BIT */

/*------------------------------*/
/* ROT2_LCH0_DSTADRV            */
/*------------------------------*/
#define ROT2_DSTADRV_RANDOM_MODE	0x00000000
#define ROT2_DSTADRV_RASTER_ORDER_MODE	0x0000000F

#ifndef ROT2_BL_DATAx_BYTE0
/*------------------------------*/
/* BL                           */
/* (temporary declaration)      */
/*------------------------------*/
#define ROT2_BL_DATAx_BYTE0	0x0
#define ROT2_BL_DATAx_BYTE1	0x1
#define ROT2_BL_DATAx_BYTE2	0x2
#define ROT2_BL_DATAx_BYTE3	0x3

/*------------------------------*/
/* ROT2_LCH0_DSTBL              */
/*------------------------------*/
#define ROT2_DSTBL_DATA3_BIT	0x000000C0
#define ROT2_DSTBL_DATA3_SFT	0x6
#define ROT2_DSTBL_DATA3_BYTE0	(ROT2_BL_DATAx_BYTE0 << ROT2_DSTBL_DATA3_SFT)
#define ROT2_DSTBL_DATA3_BYTE1	(ROT2_BL_DATAx_BYTE1 << ROT2_DSTBL_DATA3_SFT)
#define ROT2_DSTBL_DATA3_BYTE2	(ROT2_BL_DATAx_BYTE2 << ROT2_DSTBL_DATA3_SFT)
#define ROT2_DSTBL_DATA3_BYTE3	(ROT2_BL_DATAx_BYTE3 << ROT2_DSTBL_DATA3_SFT)

#define ROT2_DSTBL_DATA2_BIT	0x00000030
#define ROT2_DSTBL_DATA2_SFT	0x4
#define ROT2_DSTBL_DATA2_BYTE0	(ROT2_BL_DATAx_BYTE0 << ROT2_DSTBL_DATA2_SFT)
#define ROT2_DSTBL_DATA2_BYTE1	(ROT2_BL_DATAx_BYTE1 << ROT2_DSTBL_DATA2_SFT)
#define ROT2_DSTBL_DATA2_BYTE2	(ROT2_BL_DATAx_BYTE2 << ROT2_DSTBL_DATA2_SFT)
#define ROT2_DSTBL_DATA2_BYTE3	(ROT2_BL_DATAx_BYTE3 << ROT2_DSTBL_DATA2_SFT)

#define ROT2_DSTBL_DATA1_BIT	0x0000000C
#define ROT2_DSTBL_DATA1_SFT	0x2
#define ROT2_DSTBL_DATA1_BYTE0	(ROT2_BL_DATAx_BYTE0 << ROT2_DSTBL_DATA1_SFT)
#define ROT2_DSTBL_DATA1_BYTE1	(ROT2_BL_DATAx_BYTE1 << ROT2_DSTBL_DATA1_SFT)
#define ROT2_DSTBL_DATA1_BYTE2	(ROT2_BL_DATAx_BYTE2 << ROT2_DSTBL_DATA1_SFT)
#define ROT2_DSTBL_DATA1_BYTE3	(ROT2_BL_DATAx_BYTE3 << ROT2_DSTBL_DATA1_SFT)

#define ROT2_DSTBL_DATA0_BIT	0x00000003
#define ROT2_DSTBL_DATA0_SFT	0x0
#define ROT2_DSTBL_DATA0_BYTE0	(ROT2_BL_DATAx_BYTE0 << ROT2_DSTBL_DATA0_SFT)
#define ROT2_DSTBL_DATA0_BYTE1	(ROT2_BL_DATAx_BYTE1 << ROT2_DSTBL_DATA0_SFT)
#define ROT2_DSTBL_DATA0_BYTE2	(ROT2_BL_DATAx_BYTE2 << ROT2_DSTBL_DATA0_SFT)
#define ROT2_DSTBL_DATA0_BYTE3	(ROT2_BL_DATAx_BYTE3 << ROT2_DSTBL_DATA0_SFT)
#endif /* ROT2_BL_DATAx_BYTE0 */

/*------------------------------*/
/* ROT2_HISTCTRL                */
/*------------------------------*/
#define ROT2_HISTCTRL_HISTEN_BIT	0x00000001
#define ROT2_HISTCTRL_HISTEN_SFT	0
#define ROT2_HISTCTRL_HISTEN_DISABLE	0x0
#define ROT2_HISTCTRL_HISTEN_ENABLE	0x1

#define ROT2_HISTCTRL_HISTBYTE_BIT	0x00000002
#define ROT2_HISTCTRL_HISTBYTE_SFT	1
#define ROT2_HISTCTRL_HISTBYTE_Y	0x0
#define ROT2_HISTCTRL_HISTBYTE_UV	0x2

#define ROT2_HISTCTRL_HISTCH_BIT	0x00000004
#define ROT2_HISTCTRL_HISTCH_SFT	2
#define ROT2_HISTCTRL_HISTCH_CH0	0x0

/*===========================================================================*/
/* ROT register reset                                                        */
/*===========================================================================*/
#define ROT2_MODE_RESET		0x00000000
#define ROT2_SRCHSIZE_RESET	0x00000000
#define ROT2_SRCVSIZE_RESET	0x00000000
#define ROT2_SRCFMT_RESET	0x00000000
#define ROT2_DSTADRYRGB_RESET	0x00000000
#define ROT2_DSTADRUV_RESET	0x00000000
#define ROT2_DSTADRV_RESET	0x00000000
#define ROT2_DSTBL_RESET	0x000000E4
#define ROT2_HISTCTRL_RESET	0x00000000
#define ROT2_HISTCLR_RESET	0x00000000
#define ROT2_HIST0_RESET	0x00000000
#define ROT2_HIST1_RESET	0x00000000
#define ROT2_HIST2_RESET	0x00000000
#define ROT2_HIST3_RESET	0x00000000
#define ROT2_HIST4_RESET	0x00000000
#define ROT2_HIST5_RESET	0x00000000
#define ROT2_HIST6_RESET	0x00000000
#define ROT2_HIST7_RESET	0x00000000

#endif /* _ROT2_COMMON_H_ */
