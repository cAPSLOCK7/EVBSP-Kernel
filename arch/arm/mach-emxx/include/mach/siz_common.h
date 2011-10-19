/*
*  File Name       : arch/arm/mach-emxx/include/mach/siz_common.h
*  Function        : SIZ Driver I/F and SIZ MMIO definitions
*  Release Version : Ver 1.02
*  Release Date    : 2010.06.08
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

#ifndef _SIZ_COMMON_H_
#define _SIZ_COMMON_H_

/*===========================================================================*/
/* SIZ Driver Structures                                                     */
/*===========================================================================*/
/* SIZ resource information */
struct emxx_siz_info {
	unsigned long id;
	/* Input address for Source Image */
	unsigned long adryrgb;		/* phys */
	unsigned long adruv;		/* phys */
	unsigned long adrv;		/* phys */
	unsigned char device;		/* request device id */
	unsigned int  timeout;		/* timeout time */
};

/* SIZ parameter information */
struct emxx_siz_param {
	unsigned long src_hsize;	/* SIZ_SRCHSIZE      */
	unsigned long src_vsize;	/* SIZ_SRCVSIZE      */
	unsigned long src_format;	/* SIZ_SRCFMT        */
	unsigned long dst_hsize;	/* SIZ_DSTHSIZE      */
	unsigned long dst_vsize;	/* SIZ_DSTVSIZE      */
	unsigned long dst_hskip;	/* SIZ_DSTHSKIP      */
	unsigned long dst_adryrgb;	/* SIZ_DSTADRYRGB    */
	unsigned long dst_adruv;	/* SIZ_DSTADRUV      */
	unsigned long dst_adrv;		/* SIZ_DSTADRV       */
	unsigned long dst_format;	/* SIZ_DSTFMT        */
	unsigned long dst_bytelane;	/* SIZ_DSTBL         */
	unsigned long hstep;		/* SIZ_HSTEP         */
	unsigned long vstep;		/* SIZ_VSTEP         */
	unsigned long dst_hcrop;	/* SIZ_DSTHCROP      */
	unsigned long dst_vcrop;	/* SIZ_DSTVCROP      */
	unsigned long rot_dst_adryrgb;	/* SIZ_ROTDSTADRYRGB */
	unsigned long rot_dst_adruv;	/* SIZ_ROTDSTADRUV   */
	unsigned long rot_mode;		/* SIZ_ROTMODE       */
	unsigned long rot_dst_format;	/* SIZ_ROTDSTFMT     */
	unsigned long filter_option;	/* SIZ_FILTOPT       */
	unsigned long filt0;		/* SIZ_FILT0         */
	unsigned long filt1;		/* SIZ_FILT1         */
	unsigned long filt2;		/* SIZ_FILT2         */
	unsigned long filt3;		/* SIZ_FILT3         */
	unsigned long filt4;		/* SIZ_FILT4         */
	unsigned long filt5;		/* SIZ_FILT5         */
	unsigned long filt6;		/* SIZ_FILT6         */
	unsigned long filt7;		/* SIZ_FILT7         */
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
/* struct emxx_siz_info.device */
#ifndef DEV_FB
#define DEV_FB		1
#define DEV_LCD		2
#define DEV_DSP		3
#define DEV_CAM		4
#define DEV_OTHER	5
#endif

/*===========================================================================*/
/* SIZ register definitions                                                  */
/*===========================================================================*/
#define SIZ_SRCHSIZE		0x00
#define SIZ_SRCVSIZE		0x04
#define SIZ_SRCFMT		0x08
#define SIZ_DSTHSIZE		0x0C
#define SIZ_DSTVSIZE		0x10
#define SIZ_DSTHSKIP		0x14
#define SIZ_DSTADRYRGB		0x18
#define SIZ_DSTADRUV		0x1C
#define SIZ_DSTADRV		0x20
#define SIZ_DSTFMT		0x24
#define SIZ_DSTBL		0x28
#define SIZ_HSTEP		0x2C
#define SIZ_VSTEP		0x30
#define SIZ_FILTOPT		0x34
#define SIZ_DSTHCROP		0x38
#define SIZ_DSTVCROP		0x3C
#define SIZ_ROTDSTADRYRGB	0x40
#define SIZ_ROTDSTADRUV		0x44
#define SIZ_ROTMODE		0x48
#define SIZ_ROTDSTFMT		0x4C
#define SIZ_STAT		0x50
#define SIZ_FILT0		0x54
#define SIZ_FILT1		0x58
#define SIZ_FILT2		0x5C
#define SIZ_FILT3		0x60
#define SIZ_FILT4		0x64
#define SIZ_FILT5		0x68
#define SIZ_FILT6		0x6C
#define SIZ_FILT7		0x70
#define SIZ_COEF_R0		0x74
#define SIZ_COEF_R1		0x78
#define SIZ_COEF_R2		0x7C
#define SIZ_COEF_R3		0x80
#define SIZ_COEF_G0		0x84
#define SIZ_COEF_G1		0x88
#define SIZ_COEF_G2		0x8C
#define SIZ_COEF_G3		0x90
#define SIZ_COEF_B0		0x94
#define SIZ_COEF_B1		0x98
#define SIZ_COEF_B2		0x9C
#define SIZ_COEF_B3		0xA0

/*===========================================================================*/
/* SIZ register bit assigns                                                  */
/*===========================================================================*/

/*------------------------------*/
/* SIZ_SRCFMT                   */
/* SIZ_DSTFMT                   */
/*------------------------------*/
#define SIZ_FORMAT_BIT		0x00000007
#define SIZ_FORMAT_SFT		0
#define SIZ_FORMAT_RGB888	0
#define SIZ_FORMAT_RGB565	1
#define SIZ_FORMAT_YUV422IL	2
#define SIZ_FORMAT_YUV422SP	3
#define SIZ_FORMAT_YUV422PL	4
#define SIZ_FORMAT_YUV420SP	5
#define SIZ_FORMAT_YUV420PL	6

/*------------------------------*/
/* BL                           */
/* (temporary declaration)      */
/*------------------------------*/
#define SIZ_BL_DATAx_BYTE0	0x0
#define SIZ_BL_DATAx_BYTE1	0x1
#define SIZ_BL_DATAx_BYTE2	0x2
#define SIZ_BL_DATAx_BYTE3	0x3

/*------------------------------*/
/* SIZ_DSTBL                    */
/*------------------------------*/
#define SIZ_DSTBL_DATA3_BIT	0x000000C0
#define SIZ_DSTBL_DATA3_SFT	0x6
#define SIZ_DSTBL_DATA3_BYTE0	(SIZ_BL_DATAx_BYTE0 << SIZ_DSTBL_DATA3_SFT)
#define SIZ_DSTBL_DATA3_BYTE1	(SIZ_BL_DATAx_BYTE1 << SIZ_DSTBL_DATA3_SFT)
#define SIZ_DSTBL_DATA3_BYTE2	(SIZ_BL_DATAx_BYTE2 << SIZ_DSTBL_DATA3_SFT)
#define SIZ_DSTBL_DATA3_BYTE3	(SIZ_BL_DATAx_BYTE3 << SIZ_DSTBL_DATA3_SFT)

#define SIZ_DSTBL_DATA2_BIT	0x00000030
#define SIZ_DSTBL_DATA2_SFT	0x4
#define SIZ_DSTBL_DATA2_BYTE0	(SIZ_BL_DATAx_BYTE0 << SIZ_DSTBL_DATA2_SFT)
#define SIZ_DSTBL_DATA2_BYTE1	(SIZ_BL_DATAx_BYTE1 << SIZ_DSTBL_DATA2_SFT)
#define SIZ_DSTBL_DATA2_BYTE2	(SIZ_BL_DATAx_BYTE2 << SIZ_DSTBL_DATA2_SFT)
#define SIZ_DSTBL_DATA2_BYTE3	(SIZ_BL_DATAx_BYTE3 << SIZ_DSTBL_DATA2_SFT)

#define SIZ_DSTBL_DATA1_BIT	0x0000000C
#define SIZ_DSTBL_DATA1_SFT	0x2
#define SIZ_DSTBL_DATA1_BYTE0	(SIZ_BL_DATAx_BYTE0 << SIZ_DSTBL_DATA1_SFT)
#define SIZ_DSTBL_DATA1_BYTE1	(SIZ_BL_DATAx_BYTE1 << SIZ_DSTBL_DATA1_SFT)
#define SIZ_DSTBL_DATA1_BYTE2	(SIZ_BL_DATAx_BYTE2 << SIZ_DSTBL_DATA1_SFT)
#define SIZ_DSTBL_DATA1_BYTE3	(SIZ_BL_DATAx_BYTE3 << SIZ_DSTBL_DATA1_SFT)

#define SIZ_DSTBL_DATA0_BIT	0x00000003
#define SIZ_DSTBL_DATA0_SFT	0x0
#define SIZ_DSTBL_DATA0_BYTE0	(SIZ_BL_DATAx_BYTE0 << SIZ_DSTBL_DATA0_SFT)
#define SIZ_DSTBL_DATA0_BYTE1	(SIZ_BL_DATAx_BYTE1 << SIZ_DSTBL_DATA0_SFT)
#define SIZ_DSTBL_DATA0_BYTE2	(SIZ_BL_DATAx_BYTE2 << SIZ_DSTBL_DATA0_SFT)
#define SIZ_DSTBL_DATA0_BYTE3	(SIZ_BL_DATAx_BYTE3 << SIZ_DSTBL_DATA0_SFT)

/*------------------------------*/
/* SIZ_FILTOPT                  */
/*------------------------------*/
#define SIZ_FILTOPT_MODE_BIT			0x00000007
#define SIZ_FILTOPT_MODE_SFT			0
#define SIZ_FILTOPT_MODE_4x4FILT_MINUS		0x0
#define SIZ_FILTOPT_MODE_4x4FILT_PLUS		0x1
#define SIZ_FILTOPT_MODE_2x2FILT_WITH_BUFFER	0x2
#define SIZ_FILTOPT_MODE_2x2FILT		0x4
#define SIZ_FILTOPT_TABLE_WRITE_MODE		0x6

#define SIZ_FILTER_DEFAULT			0x7	/* SIZ Driver Default */
							/* <Linear Filter>    */
#define SIZ_FILTER_SMOOTHING			0x8	/* SIZ Driver Default */
							/* <Smoothing Filter> */

#define SIZ_FILTOPT_FILTER_THROUGH_BIT		0x00000030
#define SIZ_FILTOPT_FILTER_THROUGH_SFT		4
#define SIZ_FILTOPT_X_FILTER_THROUGH		0x10
#define SIZ_FILTOPT_Y_FILTER_THROUGH		0x20

#define SIZ_FILTOPT_COLCONV_BIT			0x00007F00
#define SIZ_FILTOPT_COLCONV_SFT			8

/*------------------------------*/
/* SIZ_ROTMODE                  */
/*------------------------------*/
#define SIZ_ROTMODE_MODE_BIT	0x00000003
#define SIZ_ROTMODE_MODE_SFT	0
#define SIZ_ROTMODE_MODE_0	0x0
#define SIZ_ROTMODE_MODE_90	0x1
#define SIZ_ROTMODE_MODE_180	0x2
#define SIZ_ROTMODE_MODE_270	0x3

#define SIZ_ROTMODE_XMIRROR_BIT	0x00000004
#define SIZ_ROTMODE_XMIRROR_SFT	2
#define SIZ_ROTMODE_XMIRROR	0x4

#define SIZ_ROTMODE_YMIRROR_BIT	0x00000008
#define SIZ_ROTMODE_YMIRROR_SFT	3
#define SIZ_ROTMODE_YMIRROR	0x8

/*------------------------------*/
/* SIZ_ROTDSTFMT                */
/*------------------------------*/
#define SIZ_ROTDSTFMT_BIT	0x00000007
#define SIZ_ROTDSTFMT_SFT	0
#define SIZ_ROTDSTFMT_OFF	0   /* Disable Rotation channel */
#define SIZ_ROTDSTFMT_YUV422IL	2
#define SIZ_ROTDSTFMT_YUV422SP	3
#define SIZ_ROTDSTFMT_YUV420SP	5

/*------------------------------*/
/* SIZ_STAT                     */
/*------------------------------*/
#define SIZ_STAT_BIT		0x00000001
#define SIZ_STAT_SFT		0
#define SIZ_STAT_INACTIVE	0
#define SIZ_STAT_ACTIVE		1

/*------------------------------*/
/* SIZ_FILTx                    */
/*------------------------------*/
#define SIZ_FILT_COEF0_BIT	0xFE000000
#define SIZ_FILT_COEF0_SFT	25
#define SIZ_FILT_COEF1_BIT	0x01FF0000
#define SIZ_FILT_COEF1_SFT	16
#define SIZ_FILT_COEF3_BIT	0x0000FE00
#define SIZ_FILT_COEF3_SFT	9
#define SIZ_FILT_COEF2_BIT	0x000001FF
#define SIZ_FILT_COEF2_SFT	0

/*===========================================================================*/
/* SIZ register reset                                                        */
/*===========================================================================*/
#define SIZ_SRCHSIZE_RESET	0x00000000
#define SIZ_SRCVSIZE_RESET	0x00000000
#define SIZ_SRCFMT_RESET	0x00000000
#define SIZ_DSTHSIZE_RESET	0x00000000
#define SIZ_DSTVSIZE_RESET	0x00000000
#define SIZ_DSTHSKIP_RESET	0x00000000
#define SIZ_DSTADRYRGB_RESET	0x00000000
#define SIZ_DSTADRUV_RESET	0x00000000
#define SIZ_DSTADRV_RESET	0x00000000
#define SIZ_DSTFMT_RESET	0x00000000
#define SIZ_DSTBL_RESET		0x000000E4
#define SIZ_HSTEP_RESET		0x00000100
#define SIZ_VSTEP_RESET		0x00000100
#define SIZ_FILTOPT_RESET	0x00000000
#define SIZ_DSTHCROP_RESET	0x00000000
#define SIZ_DSTVCROP_RESET	0x00000000
#define SIZ_ROTDSTADRYRGB_RESET	0x00000000
#define SIZ_ROTDSTADRUV_RESET	0x00000000
#define SIZ_ROTMODE_RESET	0x00000000
#define SIZ_ROTDSTFMT_RESET	0x00000000
#define SIZ_STAT_RESET		0x00000000
#define SIZ_FILT0_RESET		0x00F80008
#define SIZ_FILT1_RESET		0x00E80018
#define SIZ_FILT2_RESET		0x00D80028
#define SIZ_FILT3_RESET		0x00C80038
#define SIZ_FILT4_RESET		0x00B80048
#define SIZ_FILT5_RESET		0x00A80058
#define SIZ_FILT6_RESET		0x00980068
#define SIZ_FILT7_RESET		0x00880078

#endif /* _SIZ_COMMON_H_ */
