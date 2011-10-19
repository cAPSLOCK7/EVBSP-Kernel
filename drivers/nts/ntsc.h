/*
 * File Name       : drivers/nts/ntsc.h
 * Function        : NTSC MMIO definitions
 * Release Version : Ver 1.00
 * Release Date    : 2010.09.24
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

#ifndef _NTSC_H_
#define _NTSC_H_


#include <mach/hardware.h>


/*****************************************************************************
 * NTSC MMIO definitions
 *****************************************************************************/
#define NTS_CONTROL		0x0000
#define NTS_OUT			0x0004
#define NTS_STATUS		0x0008
#define NTS_YAREAAD_A		0x000C
#define NTS_YAREAAD_B		0x0010
#define NTS_YAREAAD_C		0x0014
#define NTS_UVAREAAD_A		0x0018
#define NTS_UVAREAAD_B		0x001C
#define NTS_UVAREAAD_C		0x0020
#define NTS_HOFFSET		0x0024
#define NTS_FRAMESEL		0x0028
#define NTS_INTSTATUS		0x0060
#define NTS_INTRAWSTATUS	0x0064
#define NTS_INTENSET		0x0068
#define NTS_INTENCLR		0x006C
#define NTS_INTFFCLR		0x0070
#define NTS_ERRORADR		0x0074
#define NTS_SWRESET		0x0078


/*------------------------------*/
/* NTS_CONTROL                  */
/*------------------------------*/
#define NTS_FORMAT_SEL_BIT	0x00000010
#define NTS_FORMAT_SEL_SFT	4
#define NTS_FORMAT_SEL_YUV420	0x00000010
#define NTS_FORMAT_SEL_YUV422	0x00000000

#define NTS_UPSCALE_BIT		0x00000008
#define NTS_UPSCALE_SFT		3
#define NTS_UPSCALE_ON		0x00000008
#define NTS_UPSCALE_OFF		0x00000000

#define NTS_OUTMODE_BIT		0x00000004
#define NTS_OUTMODE_SFT		2
#define NTS_OUTMODE_PAL		0x00000004
#define NTS_OUTMODE_NTSC	0x00000000

#define NTS_CLKPOL_BIT		0x00000002
#define NTS_CLKPOL_SFT		1
#define NTS_CLKPOL_RISING	0x00000000
#define NTS_CLKPOL_FALLING	0x00000002

#define NTS_ENDIAN_BIT		0x00000001
#define NTS_ENDIAN_SFT		0
#define NTS_ENDIAN_BIGEN	0x00000001
#define NTS_ENDIAN_LITTLEEN	0x00000000

/*------------------------------*/
/* NTS_OUT                      */
/*------------------------------*/
#define NTS_NTSOUT_BIT		0x00000003
#define NTS_NTSOUT_SFT		0
#define NTS_NTSOUT_ENABLE	0x00000003
#define NTS_NTSOUT_ENABLE_BLUE	0x00000002
#define NTS_NTSOUT_ENABLE_BLACK	0x00000001
#define NTS_NTSOUT_DISABLE	0x00000000

/*------------------------------*/
/* NTS_STATUS                   */
/*------------------------------*/
#define NTS_STATUS_BIT		0x00000003
#define NTS_STATUS_SFT		0
#define NTS_STATUS_ENABLE	0x00000003
#define NTS_STATUS_ENABLE_BLUE	0x00000002
#define NTS_STATUS_ENABLE_BLACK	0x00000001
#define NTS_STATUS_DISABLE	0x00000000

/*------------------------------*/
/* NTS_AREAAD offset            */
/*------------------------------*/
#define NTS_AREAAD_A		0x00000000
#define NTS_AREAAD_B		0x00000004
#define NTS_AREAAD_C		0x00000008

/*------------------------------*/
/* NTS_YAREAAD_A/B/C            */
/*------------------------------*/
#define NTS_YAREAADR_BIT	0xFFFFFFFF
#define NTS_YAREAADR_SFT	0
#define NTS_YAREAADR_ALINE	0x00000003

/*------------------------------*/
/* NTS_UVAREAAD_A/B/C           */
/*------------------------------*/
#define NTS_UVAREAADR_BIT	0xFFFFFFFF
#define NTS_UVAREAADR_SFT	0
#define NTS_UVAREAADR_ALINE	0x00000003

/*------------------------------*/
/* NTS_HOFFSET                  */
/*------------------------------*/
#define NTS_HOFFSET_BIT		0x00001FFF
#define NTS_HOFFSET_SFT		0
#define NTS_HOFFSET_ALINE	0x00000003

/*------------------------------*/
/* NTS_FRAMESEL                 */
/*------------------------------*/
#define NTS_AREASTATUS_BIT	0x0000000C
#define NTS_AREASTATUS_SFT	2
#define NTS_AREASTATUS_BUFC	0x0000000C
#define NTS_AREASTATUS_BUFB	0x00000008
#define NTS_AREASTATUS_BUFA	0x00000004
#define NTS_AREASTATUS_BUFINIT	0x00000000

#define NTS_AREASEL_BIT		0x00000003
#define NTS_AREASEL_SFT		0
#define NTS_AREASEL_BUFC	0x00000003
#define NTS_AREASEL_BUFB	0x00000002
#define NTS_AREASEL_BUFA	0x00000001
#define NTS_AREASEL_BUFDISABLE	0x00000000

/*------------------------------*/
/* NTS_INTSTATUS                */
/*------------------------------*/
/*------------------------------*/
/* NTS_INTRAWSTATUS             */
/*------------------------------*/
/*------------------------------*/
/* NTS_INTENSET                 */
/*------------------------------*/
/*------------------------------*/
/* NTS_INTENCLR                 */
/*------------------------------*/
/*------------------------------*/
/* NTS_INTFFCLR                 */
/*------------------------------*/
#define NTS_UNDERRUN_BIT	0x00000008
#define NTS_UNDERRUN_SFT	3
#define NTS_DMASTOP_BIT		0x00000004
#define NTS_DMASTOP_SFT		2
#define NTS_DMAERR_BIT		0x00000002
#define NTS_DMAERR_SFT		1
#define NTS_NTSVS_BIT		0x00000001
#define NTS_NTSVS_SFT		0
#define NTS_INT_ALL_BIT \
	(NTS_UNDERRUN_BIT | NTS_DMASTOP_BIT | \
	NTS_DMAERR_BIT | NTS_NTSVS_BIT)

/*------------------------------*/
/* NTS_ERRORADR                 */
/*------------------------------*/
#define NTS_ERRADR_BIT		0xFFFFFFFC
#define NTS_FCCLR_SFT		2

#define NTS_LOCK_BIT		0x00000001
#define NTS_LOCK_SFT		0

/*------------------------------*/
/* NTS_SWRESET                  */
/*------------------------------*/
#define NTS_SWRESET_BIT		0x00000001
#define NTS_SWRESET_SFT		0
#define NTS_SWRESET_ON		0x00000001
#define NTS_SWRESET_OFF		0x00000000


#endif /* _NTSC_H_ */
