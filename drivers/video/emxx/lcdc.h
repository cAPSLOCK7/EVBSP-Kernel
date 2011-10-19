/*
 * File Name       : drivers/video/emxx/lcdc.h
 * Function        : LCDC MMIO definitions
 * Release Version : Ver 1.15
 * Release Date    : 2010.06.16
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
 */

#ifndef _LCDC_H_
#define _LCDC_H_


#include <mach/hardware.h>


/*****************************************************************************
 * LCDC MMIO definitions
 *****************************************************************************/
#define LCD_CONTROL		0x0000
#define LCD_QOS			0x0004
#define LCD_DATAREQ		0x0008
#define LCD_LCDOUT		0x0010
#define LCD_BUSSEL		0x0014
#define LCD_STATUS		0x0018
#define LCD_BACKCOLOR		0x001C
#define LCD_AREAADR_ODD		0x0020
#define LCD_AREAADR_EVEN	0x0024
#define LCD_HOFFSET		0x0028
#define LCD_IFORMAT		0x002C
#define LCD_RESIZE		0x0030
#define LCD_HTOTAL		0x0040
#define LCD_HAREA		0x0044
#define LCD_HEDGE1		0x0048
#define LCD_HEDGE2		0x004C
#define LCD_VTOTAL		0x0050
#define LCD_VAREA		0x0054
#define LCD_VEDGE1		0x0058
#define LCD_VEDGE2		0x005C
#define LCD_INTSTATUS		0x0060
#define LCD_INTRAWSTATUS	0x0064
#define LCD_INTENSET		0x0068
#define LCD_INTENCLR		0x006C
#define LCD_INTFFCLR		0x0070
#define LCD_FRAMECOUNT		0x0074
#define LCD_COEF_Y0		0x0080
#define LCD_COEF_Y1		0x0084
#define LCD_COEF_Y2		0x0088
#define LCD_COEF_Y3		0x008C
#define LCD_COEF_U0		0x0090
#define LCD_COEF_U1		0x0094
#define LCD_COEF_U2		0x0098
#define LCD_COEF_U3		0x009C
#define LCD_COEF_V0		0x00A0
#define LCD_COEF_V1		0x00A4
#define LCD_COEF_V2		0x00A8
#define LCD_COEF_V3		0x00AC
#define LCD_BYTELANE		0x00C0
#define LCD_VSYNC_CONT		0x00D0
#define LCD_VOFFSET_ODD		0x00D8
#define LCD_VOFFSET_EVEN	0x00DC
#define LCD_VAREA_EVEN		0x00E0
#define LCD_VEDGE1_EVEN		0x00E8
#define LCD_VEDGE2_EVEN		0x00EC
#define LCD_CURSOR_EN		0x0200
#define LCD_CURSOR_OPE		0x0204
#define LCD_CURSOR_POSH		0x0208
#define LCD_CURSOR_POSV		0x020C
#define LCD_CURSOR_RAMSEL	0x0210
#define LCD_CURSOR_STATUS	0x0214
#define LCD_CURSOR_TABLE	0x0400
#define LCD_CURSOR_DATA0	0x1000
#define LCD_CURSOR_DATA1	0x2000


/*------------------------------*/
/* LCD_CONTROL                  */
/*------------------------------*/
#define LCD_LCLK_SEL_BIT	0x0000C000
#define LCD_LCLK_SEL_SFT	14
#define LCD_LCLK_PLL_CLK	0x00000000
#define LCD_LCLK_18V_PCLK	0x00008000
#define LCD_LCLK_33V_PCLK	0x0000C000

#define LCD_DIV_SEL_BIT		0x00002000
#define LCD_DIV_SEL_SFT		13

#define LCD_OUT_SEL_BIT		0x00000300
#define LCD_OUT_SEL_SFT		8
#define LCD_OUT_SEL_LCD		0x00000000
#define LCD_OUT_SEL_YUV		0x00000100
#define LCD_OUT_SEL_TCON	0x00000200

#define LCD_PI_SEL_BIT		0x00000040
#define LCD_PI_SEL_SFT		6
#define LCD_PI_SEL_PROGRESSIVE	0x00000000
#define LCD_PI_SEL_INTERLACE	0x00000040

#define LCD_OFORMAT_BIT		0x00000030
#define LCD_OFORMAT_SFT		4
#define LCD_OFORMAT_RGB888	0x00000000
#define LCD_OFORMAT_RGB666	0x00000010
#define LCD_OFORMAT_RGB565	0x00000020

#define LCD_CLKPOL_BIT		0x00000008
#define LCD_CLKPOL_SFT		3
#define LCD_CLKPOL_RISING	0x00000000
#define LCD_CLKPOL_FALLING	0x00000008

#define LCD_HPOL_BIT		0x00000004
#define LCD_HPOL_SFT		2
#define LCD_HPOL_POSITIVE	0x00000000
#define LCD_HPOL_NEGATIVE	0x00000004

#define LCD_VPOL_BIT		0x00000002
#define LCD_VPOL_SFT		1
#define LCD_VPOL_POSITIVE	0x00000000
#define LCD_VPOL_NEGATIVE	0x00000002

#define LCD_ENPOL_BIT		0x00000001
#define LCD_ENPOL_SFT		0
#define LCD_ENPOL_HIGH		0x00000000
#define LCD_ENPOL_LOW		0x00000001

/*------------------------------*/
/* LCD_QOS                      */
/*------------------------------*/
#define LCD_QOSEN_BIT		0x00000400
#define LCD_QOSEN_SFT		10
#define LCD_QOSEN_DISABLE	0x00000000
#define LCD_QOSEN_ENABLE	0x00000400

#define LCD_QOSVALUE_BIT	0x000003FF
#define LCD_QOSVALUE_SFT	0

/*------------------------------*/
/* LCD_DATAREQ                  */
/*------------------------------*/
#define LCD_DATAREQ_BIT		0x00000007
#define LCD_DATAREQ_SFT		0

/*------------------------------*/
/* LCD_LCDOUT                   */
/*------------------------------*/
#define LCD_LCDOUT_BIT		0x00000001
#define LCD_LCDOUT_SFT		0
#define LCD_LCDOUT_START	1
#define LCD_LCDOUT_STOP		0

/*------------------------------*/
/* LCD_BUSSEL                   */
/*------------------------------*/
#define LCD_BUSSEL_BIT		0x00000007
#define LCD_BUSSEL_SFT		0
#define LCD_BUSSEL_LOCAL	0x00000000
#define LCD_BUSSEL_DIRECT	0x00000001
#define LCD_BUSSEL_WB_LOCAL	0x00000002
#define LCD_BUSSEL_WB_DIRECT	0x00000003
#define LCD_BUSSEL_BLACK	0x00000004
#define LCD_BUSSEL_BACKCOLOR	0x00000005

/*------------------------------*/
/* LCD_STATUS                   */
/*------------------------------*/
#define LCD_MODSTATUS_BIT	0x00000700
#define LCD_MODSTATUS_SFT	8
#define LCD_MODSTATUS_LOCAL	0x00000000
#define LCD_MODSTATUS_DIRECT	0x00000100
#define LCD_MODSTATUS_WB_LOCAL	0x00000200
#define LCD_MODSTATUS_WB_DIRECT	0x00000300
#define LCD_MODSTATUS_BLACK	0x00000400
#define LCD_MODSTATUS_BACKCOLOR	0x00000500

#define LCD_STATUS_BIT		0x00000001
#define LCD_STATUS_SFT		0
#define LCD_STATUS_OFF		0x00000000
#define LCD_STATUS_ON		0x00000001

/*------------------------------*/
/* LCD_BACKCOLOR                */
/*------------------------------*/
#define LCD_BGRED_BIT		0x00FF0000
#define LCD_BGRED_SFT		16
#define LCD_BGGREEN_BIT		0x0000FF00
#define LCD_BGGREEN_SFT		8
#define LCD_BGBLUE_BIT		0x000000FF
#define LCD_BGBLUE_SFT		0

/*------------------------------*/
/* LCD_IFORMAT                  */
/*------------------------------*/
#define LCD_IFORMAT_BIT		0x00000003
#define LCD_IFORMAT_SFT		0
#define LCD_IFORMAT_RGB888	0x00000000
#define LCD_IFORMAT_RGB666	0x00000001
#define LCD_IFORMAT_RGB565	0x00000002

/*------------------------------*/
/* LCD_RESIZE                   */
/*------------------------------*/
#define LCD_RESIZE_BIT		0x00000001
#define LCD_RESIZE_SFT		0
#define LCD_RESIZE_OFF		0x00000000
#define LCD_RESIZE_ON		0x00000001

/*------------------------------*/
/* LCD_INTSTATUS                */
/*------------------------------*/
/*------------------------------*/
/* LCD_INTRAWSTATUS             */
/*------------------------------*/
/*------------------------------*/
/* LCD_INTENSET                 */
/*------------------------------*/
/*------------------------------*/
/* LCD_INTENCLR                 */
/*------------------------------*/
/*------------------------------*/
/* LCD_INTFFCLR                 */
/*------------------------------*/
#define LCD_FIELD_BIT		0x00000020
#define LCD_FIELD_SFT		5
#define LCD_WBTRACE_BIT		0x00000010
#define LCD_WBTRACE_SFT		4
#define LCD_FRMCOUNT_BIT	0x00000008
#define LCD_FRMCOUNT_SFT	3
#define LCD_LCDSTOP_BIT		0x00000004
#define LCD_LCDSTOP_SFT		2
#define LCD_UNDERRUN_BIT	0x00000002
#define LCD_UNDERRUN_SFT	1
#define LCD_LCDVS_BIT		0x00000001
#define LCD_LCDVS_SFT		0
#define LCD_INT_ALL_BIT \
	(LCD_FIELD_BIT | LCD_WBTRACE_BIT | LCD_FRMCOUNT_BIT | \
	LCD_LCDSTOP_BIT | LCD_UNDERRUN_BIT | LCD_LCDVS_BIT)

/*------------------------------*/
/* LCD_FRAMECOUNT               */
/*------------------------------*/
#define LCD_FCCLR_BIT		0x01000000
#define LCD_FCCLR_SFT		24
#define LCD_FCEN_BIT		0x00010000
#define LCD_FCEN_SFT		16
#define LCD_ACTF_BIT		0x0000FF00
#define LCD_ACTF_SFT		8
#define LCD_INTF_BIT		0x000000FF
#define LCD_INTF_SFT		0

/*------------------------------*/
/* LCD_VSYNC_CONT               */
/*------------------------------*/
#define LCD_VOFFSET_EN_BIT	0x00000001
#define LCD_VTOTAL_SEL_BIT	0x00000002
#define LCD_VEDGE_SEL_BIT	0x00000004


#endif /* _LCDC_H_ */
