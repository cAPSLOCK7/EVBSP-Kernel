/*
 * File Name       : arch/arm/mach-emxx/include/mach/imc.h
 * Function        : IMC MMIO definitions
 * Release Version : Ver 1.03
 * Release Date    : 2010.06.07
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

#ifndef _IMC_H_
#define _IMC_H_


#include <mach/hardware.h>


/*****************************************************************************
 * IMC MMIO definitions
 *****************************************************************************/
#define IMC_CONTROL		0x0000
#define IMC_REFRESH		0x0004
#define IMC_DATAREQ		0x0008
#define IMC_START		0x0010
#define IMC_STATUS		0x0014
#define IMC_CPUBUFSEL		0x0018
#define IMC_STOP		0x001C
#define IMC_GAMMA_EN		0x0020
#define IMC_GAMMA_ADR		0x0024
#define IMC_GAMMA_DATA		0x0028
#define IMC_WB_AREAADR_P	0x0040
#define IMC_WB_HOFFSET		0x0044
#define IMC_WB_FORMAT		0x0048
#define IMC_WB_SIZE		0x004C
#define IMC_WB_AREAADR_Q	0x0050
#define IMC_WB_BUFSEL		0x0054
#define IMC_WB_MPOSITION	0x0058
#define IMC_WB_MSIZE		0x005C
#define IMC_BACKCOLOR		0x0060
#define IMC_WB_BYTELANE		0x0064
#define IMC_WB_SCANMODE		0x0070
#define IMC_MIRROR		0x0100
#define IMC_YGAINOFFSET		0x0104
#define IMC_UGAINOFFSET		0x0108
#define IMC_VGAINOFFSET		0x010C
#define IMC_YUV2RGB		0x0110
#define IMC_COEF_R0		0x0114
#define IMC_COEF_R1		0x0118
#define IMC_COEF_R2		0x011C
#define IMC_COEF_R3		0x0120
#define IMC_COEF_G0		0x0124
#define IMC_COEF_G1		0x0128
#define IMC_COEF_G2		0x012C
#define IMC_COEF_G3		0x0130
#define IMC_COEF_B0		0x0134
#define IMC_COEF_B1		0x0138
#define IMC_COEF_B2		0x013C
#define IMC_COEF_B3		0x0140
#define IMC_ALPHASEL0		0x0150
#define IMC_ALPHASEL1		0x0154
#define IMC_BURST_EN		0x0160
#define IMC_THRESHOLD		0x0164
#define IMC_L0_CONTROL		0x0200
#define IMC_L0_FORMAT		0x0204
#define IMC_L0_BUFSEL		0x0208
#define IMC_L0_BYTELANE		0x020C
#define IMC_L0_KEYENABLE	0x0210
#define IMC_L0_KEYCOLOR		0x0214
#define IMC_L0_ALPHA		0x0218
#define IMC_L0_RESIZE		0x0220
#define IMC_L0_MIRROR		0x0224
#define IMC_L0_OFFSET		0x0230
#define IMC_L0_FRAMEADR_P	0x0234
#define IMC_L0_FRAMEADR_Q	0x0240
#define IMC_L0_POSITION		0x0250
#define IMC_L0_SIZE		0x0254
#define IMC_L0_MPOSITION	0x0260
#define IMC_L0_MSIZE		0x0264
#define IMC_L0_SCANMODE		0x0270
#define IMC_L1A_CONTROL		0x0300
#define IMC_L1A_FORMAT		0x0304
#define IMC_L1A_BUFSEL		0x0308
#define IMC_L1A_BYTELANE	0x030C
#define IMC_L1A_KEYENABLE	0x0310
#define IMC_L1A_KEYCOLOR	0x0314
#define IMC_L1A_ALPHA		0x0318
#define IMC_L1A_RESIZE		0x0320
#define IMC_L1A_MIRROR		0x0324
#define IMC_L1A_OFFSET		0x0330
#define IMC_L1A_FRAMEADR_P	0x0334
#define IMC_L1A_FRAMEADR_Q	0x0340
#define IMC_L1A_POSITION	0x0350
#define IMC_L1A_SIZE		0x0354
#define IMC_L1A_MPOSITION	0x0360
#define IMC_L1A_MSIZE		0x0364
#define IMC_L1A_SCANMODE	0x0370
#define IMC_L1B_CONTROL		0x0400
#define IMC_L1B_FORMAT		0x0404
#define IMC_L1B_BUFSEL		0x0408
#define IMC_L1B_BYTELANE	0x040C
#define IMC_L1B_KEYENABLE	0x0410
#define IMC_L1B_KEYCOLOR	0x0414
#define IMC_L1B_ALPHA		0x0418
#define IMC_L1B_RESIZE		0x0420
#define IMC_L1B_MIRROR		0x0424
#define IMC_L1B_OFFSET		0x0430
#define IMC_L1B_FRAMEADR_P	0x0434
#define IMC_L1B_FRAMEADR_Q	0x0440
#define IMC_L1B_POSITION	0x0450
#define IMC_L1B_SIZE		0x0454
#define IMC_L1B_MPOSITION	0x0460
#define IMC_L1B_MSIZE		0x0464
#define IMC_L1B_SCANMODE	0x0470
#define IMC_L1C_CONTROL		0x0500
#define IMC_L1C_FORMAT		0x0504
#define IMC_L1C_BUFSEL		0x0508
#define IMC_L1C_BYTELANE	0x050C
#define IMC_L1C_KEYENABLE	0x0510
#define IMC_L1C_KEYCOLOR	0x0514
#define IMC_L1C_ALPHA		0x0518
#define IMC_L1C_RESIZE		0x0520
#define IMC_L1C_MIRROR		0x0524
#define IMC_L1C_OFFSET		0x0530
#define IMC_L1C_FRAMEADR_P	0x0534
#define IMC_L1C_FRAMEADR_Q	0x0540
#define IMC_L1C_POSITION	0x0550
#define IMC_L1C_SIZE		0x0554
#define IMC_L1C_MPOSITION	0x0560
#define IMC_L1C_MSIZE		0x0564
#define IMC_L1C_SCANMODE	0x0570
#define IMC_L2A_CONTROL		0x0600
#define IMC_L2A_FORMAT		0x0604
#define IMC_L2A_BUFSEL		0x0608
#define IMC_L2A_BYTELANE	0x060C
#define IMC_L2A_RESIZE		0x0620
#define IMC_L2A_MIRROR		0x0624
#define IMC_L2A_OFFSET		0x0630
#define IMC_L2A_FRAMEADR_YP	0x0634
#define IMC_L2A_FRAMEADR_UP	0x0638
#define IMC_L2A_FRAMEADR_VP	0x063C
#define IMC_L2A_FRAMEADR_YQ	0x0640
#define IMC_L2A_FRAMEADR_UQ	0x0644
#define IMC_L2A_FRAMEADR_VQ	0x0648
#define IMC_L2A_POSITION	0x0650
#define IMC_L2A_SIZE		0x0654
#define IMC_L2A_MPOSITION	0x0660
#define IMC_L2A_MSIZE		0x0664
#define IMC_L2A_SCANMODE	0x0670
#define IMC_L2B_CONTROL		0x0700
#define IMC_L2B_FORMAT		0x0704
#define IMC_L2B_BUFSEL		0x0708
#define IMC_L2B_BYTELANE	0x070C
#define IMC_L2B_RESIZE		0x0720
#define IMC_L2B_MIRROR		0x0724
#define IMC_L2B_OFFSET		0x0730
#define IMC_L2B_FRAMEADR_YP	0x0734
#define IMC_L2B_FRAMEADR_UP	0x0738
#define IMC_L2B_FRAMEADR_VP	0x073C
#define IMC_L2B_FRAMEADR_YQ	0x0740
#define IMC_L2B_FRAMEADR_UQ	0x0744
#define IMC_L2B_FRAMEADR_VQ	0x0748
#define IMC_L2B_POSITION	0x0750
#define IMC_L2B_SIZE		0x0754
#define IMC_L2B_MPOSITION	0x0760
#define IMC_L2B_MSIZE		0x0764
#define IMC_L2B_SCANMODE	0x0770
#define IMC_BG_FORMAT		0x0804
#define IMC_BG_BUFSEL		0x0808
#define IMC_BG_BYTELANE		0x080C
#define IMC_BG_RESIZE		0x0820
#define IMC_BG_MIRROR		0x0824
#define IMC_BG_OFFSET		0x0830
#define IMC_BG_FRAMEADR_P	0x0834
#define IMC_BG_FRAMEADR_Q	0x0840
#define IMC_BG_MPOSITION	0x0860
#define IMC_BG_MSIZE		0x0864
#define IMC_BG_SCANMODE		0x0870
#define IMC_INTSTATUS		0x0900
#define IMC_INTRAWSTATUS	0x0904
#define IMC_INTENSET		0x0908
#define IMC_INTENCLR		0x090C
#define IMC_INTFFCLR		0x0910
#define IMC_ERRORADR_R		0x0914
#define IMC_ERRORADR_W		0x0918
#define IMC_ERRORADR_SW		0x091C
#define IMC_ROUND_EN		0x1008
#define IMC_MONSEL		0x0A00
#define IMC_RESERVED		0xFFFC


/*------------------------------*/
/* IMC_CONTROL                  */
/*------------------------------*/
#define IMC_DBGMODE_BIT		0x80000000
#define IMC_DBGMODE_SFT		31

#define IMC_CLKCNT_BIT		0x000FFF00
#define IMC_CLKCNT_SFT		8
#define IMC_CLKCNT_FR_BUSAC	0x00000100
#define IMC_CLKCNT_FR_FORTRANS	0x00000200
#define IMC_CLKCNT_MD_BUSAC	0x00000400
#define IMC_CLKCNT_MD_FORTRANS	0x00000800
#define IMC_CLKCNT_BK_BUSAC	0x00001000
#define IMC_CLKCNT_BK_FORTRANS	0x00002000
#define IMC_CLKCNT_GAMMA	0x00004000
#define IMC_CLKCNT_LAYER_MIX	0x00008000
#define IMC_CLKCNT_LCDC_IF	0x00010000
#define IMC_CLKCNT_AXIMR_IF	0x00020000
#define IMC_CLKCNT_AXISW_IF	0x00040000
#define IMC_CLKCNT_AXIMW_IF	0x00080000
#define IMC_CLKCNT_ALL	(IMC_CLKCNT_FR_BUSAC | IMC_CLKCNT_FR_FORTRANS | \
			IMC_CLKCNT_MD_BUSAC | IMC_CLKCNT_MD_FORTRANS | \
			IMC_CLKCNT_BK_BUSAC | IMC_CLKCNT_BK_FORTRANS | \
			IMC_CLKCNT_GAMMA | IMC_CLKCNT_LAYER_MIX | \
			IMC_CLKCNT_LCDC_IF | IMC_CLKCNT_AXIMR_IF | \
			IMC_CLKCNT_AXISW_IF | IMC_CLKCNT_AXIMW_IF)

#define IMC_FORMAT_BIT		0x00000006
#define IMC_FORMAT_SFT		1
#define IMC_FORMAT_RGB888	0x00000000
#define IMC_FORMAT_RGB666	0x00000002
#define IMC_FORMAT_RGB565	0x00000004

#define IMC_START_MODE_BIT	0x00000001
#define IMC_START_MODE_SFT	0
#define IMC_START_MODE_LCD_SYNC		0x00000000/* LCD-synchronous mode */
#define IMC_START_MODE_IMMEDIATE	0x00000001/* Immediate startup mode */

/*------------------------------*/
/* IMC_REFRESH                  */
/*------------------------------*/
#define IMC_UPDATE_BIT		0x00000001
#define IMC_UPDATE_SFT		0

#define IMC_UPDATE_ON		0x00000001
#define IMC_UPDATE_OFF		0x00000000

/*------------------------------*/
/* IMC_DATAREQ                  */
/*------------------------------*/
#define IMC_DATAREQ_BIT		0x000003FF
#define IMC_DATAREQ_SFT		0

/*------------------------------*/
/* IMC_START                    */
/*------------------------------*/
#define IMC_IMCSTART_BIT	0x00000001
#define IMC_IMCSTART_SFT	0

#define IMC_IMCSTART_ON		0x00000001
#define IMC_IMCSTART_OFF	0x00000000

/*------------------------------*/
/* IMC_STATUS                   */
/*------------------------------*/
#define IMC_STATUS_BIT		0x00000003
#define IMC_STATUS_SFT		0

#define IMC_STATUS_SINGLE_WB	0x00000003
#define IMC_STATUS_LCDOUT_WB	0x00000002
#define IMC_STATUS_LCDOUT	0x00000001
#define IMC_STATUS_STOP		0x00000000

/*------------------------------*/
/* IMC_CPUBUFSEL                */
/*------------------------------*/
#define IMC_CPUBUFSEL_BIT	0x00000001
#define IMC_CPUBUFSEL_SFT	0

#define IMC_CPUBUFSEL_P		0x00000000
#define IMC_CPUBUFSEL_Q		0x00000001

/*------------------------------*/
/* IMC_STOP                     */
/*------------------------------*/
#define IMC_IMCSTOP_BIT		0x00000001
#define IMC_IMCSTOP_SFT		0

#define IMC_IMCSTOP_ON		0x00000001
#define IMC_IMCSTOP_OFF		0x00000000

/*------------------------------*/
/* IMC_GAMMA_EN                 */
/*------------------------------*/
#define IMC_PROCESS_BIT		0x00000300
#define IMC_PROCESS_SFT		8
#define IMC_PROCESS_SYNTHESIS	0x00000000
#define IMC_PROCESS_FRONT	0x00000100
#define IMC_PROCESS_MIDDLE	0x00000200
#define IMC_PROCESS_BACK	0x00000300

#define IMC_FRAME_BIT		0x00000007
#define IMC_FRAME_SFT		0
#define IMC_FRAME_SYNTHESIS	0x00000001
#define IMC_FRAME_L0		0x00000001
#define IMC_FRAME_L1A		0x00000001
#define IMC_FRAME_L1B		0x00000002
#define IMC_FRAME_L1C		0x00000004
#define IMC_FRAME_L2A		0x00000001
#define IMC_FRAME_L2B		0x00000002
#define IMC_FRAME_BG		0x00000004

#define IMC_GAMMA_EN_OFF	0x00000000

/*------------------------------*/
/* IMC_GAMMA_ADR                */
/*------------------------------*/
#define IMC_GAMMAADR_BIT	0x000000FF
#define IMC_GAMMAADR_SFT	0

/*------------------------------*/
/* IMC_GAMMA_DATA               */
/*------------------------------*/
#define IMC_GAMMARED_BIT	0x00FF0000
#define IMC_GAMMARED_SFT	16
#define IMC_GAMMAGREEN_BIT	0x0000FF00
#define IMC_GAMMAGREEN_SFT	8
#define IMC_GAMMABLUE_BIT	0x000000FF
#define IMC_GAMMABLUE_SFT	0

/*------------------------------*/
/* IMC_WB_AREAADR_P/Q           */
/*------------------------------*/
#define IMC_WB_AREAADR_BIT	0xFFFFFFFFF
#define IMC_WB_AREAADR_SFT	0

/*------------------------------*/
/* IMC_WB_HOFFSET               */
/*------------------------------*/
#define IMC_WB_HOFFSET_BIT	0x0000FFFF
#define IMC_WB_HOFFSET_SFT	0

/*------------------------------*/
/* IMC_WB_FORMAT                */
/*------------------------------*/
#define IMC_WB_FORMAT_BIT	0x00000007
#define IMC_WB_FORMAT_SFT	0

#define IMC_WB_FORMAT_RGB888	0
#define IMC_WB_FORMAT_RGB666	1
#define IMC_WB_FORMAT_RGB565	2
#define IMC_WB_FORMAT_ARGB8888	4
#define IMC_WB_FORMAT_ARGB4444	5

/*------------------------------*/
/* IMC_WB_SIZE                  */
/*------------------------------*/
#define IMC_WB_VSIZE_BIT	0x0FFF0000
#define IMC_WB_VSIZE_SFT	16

#define IMC_WB_HSIZE_BIT	0x00000FFF
#define IMC_WB_HSIZE_SFT	0

/*------------------------------*/
/* IMC_WB_BUFSEL                */
/*------------------------------*/
#define IMC_WB_BUFSEL_BIT	0x00000003
#define IMC_WB_BUFSEL_SFT	0

#define IMC_WB_BUFSEL_P		0
#define IMC_WB_BUFSEL_Q		1
#define IMC_WB_BUFSEL_CPU	2
#define IMC_WB_BUFSEL_INV_CPU	3
#define IMC_WB_BUFSEL_P_ODD		0
#define IMC_WB_BUFSEL_Q_ODD		1
#define IMC_WB_BUFSEL_CPU_ODD		2
#define IMC_WB_BUFSEL_INV_CPU_ODD	3

/*------------------------------*/
/* IMC_WB_MPOSITION             */
/*------------------------------*/
#define IMC_WB_MPOSY_BIT	0x3FFF0000
#define IMC_WB_MPOSY_SFT	16

#define IMC_WB_MPOSX_BIT	0x00003FFF
#define IMC_WB_MPOSX_SFT	0

/*------------------------------*/
/* IMC_WB_MSIZE                 */
/*------------------------------*/
#define IMC_WB_MSIZEY_BIT	0x3FFF0000
#define IMC_WB_MSIZEY_SFT	16

#define IMC_WB_MSIZEX_BIT	0x00003FFF
#define IMC_WB_MSIZEX_SFT	0

/*------------------------------*/
/* IMC_BACKCOLOR                */
/*------------------------------*/
#define IMC_BGR_BIT		0x00FFFFFF
#define IMC_BGR_SFT		0

/*------------------------------*/
/* IMC_WB_BYTELANE              */
/*------------------------------*/
#define IMC_WB_BYTELANE_BIT	0x0000FF00
#define IMC_WB_BYTELANE_SFT	8

#define IMC_WB_BYTELANE_ARGB	0x0000E400
#define IMC_WB_BYTELANE_ABGR	0x0000C600
#define IMC_WB_BYTELANE_RGBA	0x00009300
#define IMC_WB_BYTELANE_BGRA	0x00001B00

/*------------------------------*/
/* IMC_WB_SCANMODE              */
/*------------------------------*/
#define IMC_WB_CURFIELD_BIT	0x00000002
#define IMC_WB_CURFIELD_SFT	1

#define IMC_WB_SCANMODE_BIT	0x00000001
#define IMC_WB_SCANMODE_SFT	0
#define IMC_WB_SCANMODE_PROGRESSIVE	0x00000000
#define IMC_WB_SCANMODE_INTERLACED	0x00000001

/*------------------------------*/
/* IMC_MIRROR                   */
/*------------------------------*/
#define IMC_MIRROR_BIT		0x00000003
#define IMC_MIRROR_SFT		0

#define IMC_MIRROR_NO_FLIP	0x00000000
#define IMC_MIRROR_H_FLIP	0x00000001
#define IMC_MIRROR_V_FLIP	0x00000002
#define IMC_MIRROR_HV_FLIP	0x00000003

/*------------------------------*/
/* IMC_Y/U/VGAINOFFSET          */
/*------------------------------*/
#define IMC_YUV_OFFSET_BIT	0x0000FF00
#define IMC_YUV_OFFSET_SFT	8
#define IMC_YUV_GAIN_BIT	0x000000FF
#define IMC_YUV_GAIN_SFT	0

/*------------------------------*/
/* IMC_YUV2RGB                  */
/*------------------------------*/
#define IMC_DITHER_BIT		0x00000100
#define IMC_DITHER_SFT		8
#define IMC_DITHER_OFF		0x00000000
#define IMC_DITHER_ON		0x00000100

#define IMC_TRANSMODE_BIT	0x00000003
#define IMC_TRANSMODE_SFT	0

#define IMC_TRANSMODE_BT601	0x00000000	/* ITU-R BT.601-compliant */
#define IMC_TRANSMODE_BT709	0x00000001	/* ITU-R BT.709-compliant */
#define IMC_TRANSMODE_CUSTOM1	0x00000002	/* Custom coefficient (Y value
						   is subtracted by 16) */
#define IMC_TRANSMODE_CUSTOM2	0x00000003	/* Custom coefficient (Y value
						   is used as is) */

/*----------------------------------------------*/
/* IMC_COEF_R0/R1/R2/R3/G0/G1/G2/G3/B0/B1/B2/B3 */
/*----------------------------------------------*/
#define IMC_COEF_BIT		0x000007FF
#define IMC_COEF_SFT		0

/*------------------------------*/
/* IMC_ALPHASEL0/1              */
/*------------------------------*/
#define IMC_REVERSE_BIT		0x00000100
#define IMC_REVERSE_SFT		8

#define IMC_ALPHA_BIT		0x000000FF
#define IMC_ALPHA_SFT		0

/*------------------------------*/
/* IMC_BURST_EN                 */
/*------------------------------*/
#define IMC_BREN_BIT		0x00000100
#define IMC_BREN_SFT		8

#define IMC_BWEN_BIT		0x00000001
#define IMC_BWEN_SFT		0

/*------------------------------*/
/* IMC_THRESHOLD                */
/*------------------------------*/
#define IMC_THRESHOLD_R_BIT	0x00003F00
#define IMC_THRESHOLD_R_SFT	8

#define IMC_THRESHOLD_W_BIT	0x0000003F
#define IMC_THRESHOLD_W_SFT	0

/*-------------------------------*/
/* IMC_L0/1A/1B/1C/2A/2B_CONTROL */
/*-------------------------------*/
#define IMC_Lx_CONTROL_BIT	0x00000001
#define IMC_Lx_CONTROL_SFT	0
#define IMC_Lx_CONTROL_DISABLE	0x00000000
#define IMC_Lx_CONTROL_ENABLE	0x00000001

/*------------------------------*/
/* IMC_L0/L1A/L1B/L1C_FORMAT    */
/*------------------------------*/
#define IMC_L01x_FORMAT_BIT	0x00000007
#define IMC_L01x_FORMAT_SFT	0
#define IMC_L01x_FORMAT_RGB888	0
#define IMC_L01x_FORMAT_RGB666	1
#define IMC_L01x_FORMAT_RGB565	2
#define IMC_L01x_FORMAT_ARGB8888	4
#define IMC_L01x_FORMAT_ARGB4444	5
#define IMC_L01x_FORMAT_ARGB1555	6

/*------------------------------*/
/* IMC_L2A/L2B_FORMAT           */
/*------------------------------*/
#define IMC_L2x_FORMAT_BIT	0x0000000F
#define IMC_L2x_FORMAT_SFT	0
#define IMC_L2x_FORMAT_RGB888	0
#define IMC_L2x_FORMAT_RGB666	1
#define IMC_L2x_FORMAT_RGB565	2
#define IMC_L2x_FORMAT_YUV422I	8
#define IMC_L2x_FORMAT_YUV422SP	9
#define IMC_L2x_FORMAT_YUV422P	10
#define IMC_L2x_FORMAT_YUV444	12
#define IMC_L2x_FORMAT_YUV420SP	13
#define IMC_L2x_FORMAT_YUV420P	14

/*------------------------------*/
/* IMC_BG_FORMAT                */
/*------------------------------*/
#define IMC_BG_FORMAT_BIT	0x00000007
#define IMC_BG_FORMAT_SFT	0
#define IMC_BG_FORMAT_RGB888	0
#define IMC_BG_FORMAT_RGB666	1
#define IMC_BG_FORMAT_RGB565	2
#define IMC_BG_FORMAT_BLACK	3
#define IMC_BG_FORMAT_BACKCOLOR	4

/*--------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_BUFSEL */
/*--------------------------------------*/
#define IMC_Lx_BUFSEL_BIT	0x00000003
#define IMC_Lx_BUFSEL_SFT	0

#define IMC_Lx_BUFSEL_P		0
#define IMC_Lx_BUFSEL_Q		1
#define IMC_Lx_BUFSEL_CPU	2
#define IMC_Lx_BUFSEL_INV_CPU	3
#define IMC_Lx_BUFSEL_P_ODD		0
#define IMC_Lx_BUFSEL_Q_ODD		1
#define IMC_Lx_BUFSEL_CPU_ODD		2
#define IMC_Lx_BUFSEL_INV_CPU_ODD	3

/*--------------------------------*/
/* IMC_L0/L1A/L1B/L1C/BG_BYTELANE */
/*--------------------------------*/
#define IMC_Lx_BYTELANE_BIT	0x0000FF00
#define IMC_Lx_BYTELANE_SFT	8

#define IMC_Lx_BYTELANE_ARGB	0x0000E400
#define IMC_Lx_BYTELANE_ABGR	0x0000C600
#define IMC_Lx_BYTELANE_RGBA	0x00009300
#define IMC_Lx_BYTELANE_BGRA	0x00001B00

/*------------------------------*/
/* IMC_L2A/L2B_BYTELANE         */
/*------------------------------*/
#define IMC_L2x_Y_BYTELANE_BIT	0x0000FF00
#define IMC_L2x_Y_BYTELANE_SFT	8
#define IMC_L2x_UV_BYTELANE_BIT	0x000000FF
#define IMC_L2x_UV_BYTELANE_SFT	0

#define IMC_L2x_BYTELANE_422I_YYUV	0x0000E400
#define IMC_L2x_BYTELANE_422I_YUYV	0x0000D800
#define IMC_L2x_BYTELANE_INIT		0x0000E4E4

/*------------------------------*/
/* IMC_L0/L1A/L1B/L1C_KEYENABLE */
/*------------------------------*/
#define IMC_Lx_KEYEN_BIT	0x00000001
#define IMC_Lx_KEYEN_SFT	0
#define IMC_Lx_KEYEN_DISABLE	0x00000000
#define IMC_Lx_KEYEN_ENABLE	0x00000001

/*------------------------------*/
/* IMC_L0/L1A/L1B/L1C_KEYCOLOR  */
/*------------------------------*/
#define IMC_Lx_KEYR_BIT		0x00FF0000
#define IMC_Lx_KEYR_SFT		16
#define IMC_Lx_KEYG_BIT		0x0000FF00
#define IMC_Lx_KEYG_SFT		8
#define IMC_Lx_KEYB_BIT		0x000000FF
#define IMC_Lx_KEYB_SFT		0

/*------------------------------*/
/* IMC_L0/L1A/L1B/L1C_ALPHA     */
/*------------------------------*/
#define IMC_Lx_ALPHASEL_BIT		0x00010000
#define IMC_Lx_ALPHASEL_SFT		16
#define IMC_Lx_REVERSE_BIT		0x00000100
#define IMC_Lx_REVERSE_SFT		8
#define IMC_Lx_ALPHA_BIT		0x000000FF
#define IMC_Lx_ALPHA_SFT		0
#define IMC_Lx_ALPHA_OPAQUE		255
#define IMC_Lx_ALPHA_TRANSPARENT	0

/*--------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_RESIZE */
/*--------------------------------------*/
#define IMC_Lx_RESIZE_BIT	0x00000001
#define IMC_Lx_RESIZE_SFT	0
#define IMC_Lx_RESIZE_DISABLE	0x00000000
#define IMC_Lx_RESIZE_ENABLE	0x00000001

/*--------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_MIRROR */
/*--------------------------------------*/
#define IMC_Lx_MIRROR_BIT	0x00000003
#define IMC_Lx_MIRROR_SFT	0

#define IMC_Lx_MIRROR_NO_FLIP	0x00000000
#define IMC_Lx_MIRROR_H_FLIP	0x00000001
#define IMC_Lx_MIRROR_V_FLIP	0x00000002
#define IMC_Lx_MIRROR_HV_FLIP	0x00000003

/*--------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_OFFSET */
/*--------------------------------------*/
#define IMC_Lx_OFFSET_BIT	0x0000FFFF
#define IMC_Lx_OFFSET_SFT	0

/*-------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B_POSITION */
/*-------------------------------------*/
#define IMC_Lx_POSY_BIT		0x0FFF0000
#define IMC_Lx_POSY_SFT		16
#define IMC_Lx_POSX_BIT		0x00000FFF
#define IMC_Lx_POSX_SFT		0

/*---------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B_SIZE */
/*---------------------------------*/
#define IMC_Lx_SIZEY_BIT	0x0FFF0000
#define IMC_Lx_SIZEX_BIT	0x00000FFF
#define IMC_L0_SIZEY_BIT	0x0FFF0000
#define IMC_L0_SIZEX_BIT	0x00000FFF
#define IMC_L1_SIZEY_BIT	0x0FFF0000
#define IMC_L1_SIZEX_BIT	0x00000FFF
#define IMC_L2_SIZEY_BIT	0x0FFF0000
#define IMC_L2_SIZEX_BIT	0x00000FFF

#define IMC_Lx_SIZEY_SFT	16
#define IMC_Lx_SIZEX_SFT	0

/*-----------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_MPOSITION */
/*-----------------------------------------*/
#define IMC_Lx_MPOSY_BIT	0x3FFF0000
#define IMC_Lx_MPOSY_SFT	16
#define IMC_Lx_MPOSX_BIT	0x00003FFF
#define IMC_Lx_MPOSX_SFT	0

#define IMC_Lx_MPOSY_MIN	0x00000000
#define IMC_Lx_MPOSY_MAX	0x3FFF0000
#define IMC_Lx_MPOSX_MIN	0x00000000
#define IMC_Lx_MPOSX_MAX	0x00003FFF

/*-------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_MSIZE */
/*-------------------------------------*/
#define IMC_Lx_MSIZEY_BIT	0x3FFF0000
#define IMC_Lx_MSIZEY_SFT	16
#define IMC_Lx_MSIZEX_BIT	0x00003FFF
#define IMC_Lx_MSIZEX_SFT	0

#define IMC_Lx_MSIZEY_MIN	0x00000000
#define IMC_Lx_MSIZEY_MAX	0x3FFF0000
#define IMC_Lx_MSIZEX_MIN	0x00000000
#define IMC_Lx_MSIZEX_MAX	0x00003FFF

/*----------------------------------------*/
/* IMC_L0/L1A/L1B/L1C/L2A/L2B/BG_SCANMODE */
/*----------------------------------------*/
#define IMC_Lx_SCANMODE_BIT	0x00000001
#define IMC_Lx_SCANMODE_SFT	0
#define IMC_Lx_SCANMODE_PROGRESSIVE	0x00000000
#define IMC_Lx_SCANMODE_INTERLACED	0x00000001

/*------------------------------*/
/* IMC_INTSTATUS                */
/*------------------------------*/
/*------------------------------*/
/* IMC_INTRAWSTATUS             */
/*------------------------------*/
/*------------------------------*/
/* IMC_INTENSET                 */
/*------------------------------*/
/*------------------------------*/
/* IMC_INTENCLR                 */
/*------------------------------*/
/*------------------------------*/
/* IMC_INTFFCLR                 */
/*------------------------------*/
#define IMC_PWBEND_BIT		0x00000040	/* Frame WB end interrupt */
#define IMC_PWBEND_SFT		6
#define IMC_SAXIWERR_BIT	0x00000020	/* AXI slave write side error
						   response interrupt */
#define IMC_SAXIWERR_SFT	5
#define IMC_MAXIRERR_BIT	0x00000010	/* AXI master read side error
						   response interrupt */
#define IMC_MAXIRERR_SFT	4
#define IMC_MAXIWERR_BIT	0x00000008	/* AXI master write side error
						   response interrupt */
#define IMC_MAXIWERR_SFT	3
#define IMC_WBEND_BIT		0x00000004	/* Field WB end interrupt */
#define IMC_WBEND_SFT		2
#define IMC_OVERRUN_BIT		0x00000002	/* Overrun interrupt */
#define IMC_OVERRUN_SFT		1
#define IMC_REFRESH_BIT		0x00000001	/* Register update end
						   interrupt */
#define IMC_REFRESH_SFT		0

#define IMC_INT_ALL_BIT		(IMC_PWBEND_BIT | IMC_SAXIWERR_BIT | \
				IMC_MAXIRERR_BIT | IMC_MAXIWERR_BIT | \
				IMC_WBEND_BIT | IMC_OVERRUN_BIT | \
				IMC_REFRESH_BIT)

/*------------------------------*/
/* IMC_ROUND_EN                 */
/*------------------------------*/
#define IMC_ROUND_EN_BIT	0x00000001
#define IMC_ROUND_EN_SFT	0

#define IMC_ROUND_EN_ON		0x00000001
#define IMC_ROUND_EN_OFF	0x00000000

/*------------------------------*/
/* IMC_RESERVED                 */
/*------------------------------*/
#define IMC_ECO343ENZ		0x80000000
#define IMC_AXOR_MB		0x40000000
#define IMC_AXOR_FM		0x20000000
#define IMC_AMODE_FM		0x02000000
#define IMC_AMODE_MB		0x01000000

#endif /* _IMC_H_ */
