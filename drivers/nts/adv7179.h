/*
 * File Name       : drivers/nts/ADV7179.h
 * Function        : ADV7179 definitions
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

#ifndef _ADV7179_H_
#define _ADV7179_H_


/*****************************************************************************
 * ADV7179 definitions
 *****************************************************************************/

/*                                                           R/W   Bit  Reset */
/*----------------------------------------------------------------------------
 * Function Settings Register
 */
/* Mode Register 0                                           R/W   7:0  0x00 */
#define ADV7179_MODE0				(0x00)

/* Mode Register 1                                           R/W   7:0  0x10 */
#define ADV7179_MODE1				(0x01)

/* Mode Register 2                                           R/W   7:0  0x00 */
#define ADV7179_MODE2				(0x02)

/* Mode Register 3                                           R/W   7:0  0x00 */
#define ADV7179_MODE3				(0x03)

/* Mode Register 4                                           R/W   7:0  0x10 */
#define ADV7179_MODE4				(0x04)

/* Timing Register 0                                         R/W   7:0  0x00 */
#define ADV7179_TIMING0				(0x07)

/* Timing Register 1                                         R/W   7:0  0x00 */
#define ADV7179_TIMING1				(0x08)

/* Subcareer frequency Register 0                            R/W   7:0  0x16 */
#define ADV7179_SUBCAREER_FREQUENCY0		(0x09)

/* Subcareer frequency Register 1                            R/W   7:0  0x7C */
#define ADV7179_SUBCAREER_FREQUENCY1		(0x0A)

/* Subcareer frequency Register 2                            R/W   7:0  0xF0 */
#define ADV7179_SUBCAREER_FREQUENCY2		(0x0B)

/* Subcareer frequency Register 3                            R/W   7:0  0x21 */
#define ADV7179_SUBCAREER_FREQUENCY3		(0x0C)

/* Subcareer phase Register                                  R/W   7:0  0x00 */
#define ADV7179_SUBCAREER_PHASE			(0x0D)

/* Closed capshoning ext Register 0                          R/W   7:0  0x00 */
#define ADV7179_CLOSED_CAPSHONING_EXT0		(0x0E)

/* Closed capshoning ext Register 1                          R/W   7:0  0x00 */
#define ADV7179_CLOSED_CAPSHONING_EXT1		(0x0F)

/* Closed capshoning Register 0                              R/W   7:0  0x00 */
#define ADV7179_CLOSED_CAPSHONING0		(0x10)

/* Closed capshoning Register 1                              R/W   7:0  0x00 */
#define ADV7179_CLOSED_CAPSHONING1		(0x11)

/* Pedestal control Register 0                               R/W   7:0  0x00 */
#define ADV7179_PEDESTAL_CONTROL0		(0x12)

/* Pedestal control Register 1                               R/W   7:0  0x00 */
#define ADV7179_PEDESTAL_CONTROL1		(0x13)

/* Pedestal control Register 2                               R/W   7:0  0x00 */
#define ADV7179_PEDESTAL_CONTROL2		(0x14)

/* Pedestal control Register 3                               R/W   7:0  0x00 */
#define ADV7179_PEDESTAL_CONTROL3		(0x15)

/* CGMS WSS Register 0                                       R/W   7:0  0x00 */
#define ADV7179_CGMS_WSS0			(0x16)

/* CGMS WSS Register 1                                       R/W   7:0  0x00 */
#define ADV7179_CGMS_WSS1			(0x17)

/* CGMS WSS Register 2                                       R/W   7:0  0x00 */
#define ADV7179_CGMS_WSS2			(0x18)

/* Teletext request control Register                         R/W   7:0  0x00 */
#define ADV7179_TELETEXT_REQUEST_CONTROL	(0x19)

#define ADV7179_REG_MAX				ADV7179_TELETEXT_REQUEST_CONTROL






/*----------------------------------------------------------------------------
 * Function Settings Register
 */
/*------------------------------*/
/* ADV7179_MODE0                */
/*------------------------------*/
#define MODE0_CHROMA_FILTER_SELECT_BIT			0xE0
#define MODE0_CHROMA_FILTER_SELECT_SFT			0x05
#define MODE0_CHROMA_FILTER_SELECT_1_3MHZ_LOWPASS	0x00
#define MODE0_CHROMA_FILTER_SELECT_0_65MHZ_LOWPASS	0x20
#define MODE0_CHROMA_FILTER_SELECT_1_0MHZ_LOWPASS	0x40
#define MODE0_CHROMA_FILTER_SELECT_2_0MHZ_LOWPASS	0x60
#define MODE0_CHROMA_FILTER_SELECT_CIF			0xA0
#define MODE0_CHROMA_FILTER_SELECT_QCIF			0xC0

#define MODE0_LUMA_FILTER_SELECT_BIT			0x1C
#define MODE0_LUMA_FILTER_SELECT_SFT			0x02
#define MODE0_LUMA_FILTER_SELECT_LOWPASS_NTSC		0x00
#define MODE0_LUMA_FILTER_SELECT_LOWPASS_PAL		0x04 /* ? */
#define MODE0_LUMA_FILTER_SELECT_NOTCH_NTSC		0x08
#define MODE0_LUMA_FILTER_SELECT_NOTCH_PAL		0x04 /* ? */
#define MODE0_LUMA_FILTER_SELECT_EXTENDED_MODE		0x10
#define MODE0_LUMA_FILTER_SELECT_CIF			0x14
#define MODE0_LUMA_FILTER_SELECT_QCIF			0x18

#define MODE0_OUTPUT_VIDEO_SELECT_BIT			0x03
#define MODE0_OUTPUT_VIDEO_SELECT_SFT			0x00
#define MODE0_OUTPUT_VIDEO_SELECT_NTSC			0x00
#define MODE0_OUTPUT_VIDEO_SELECT_PAL			0x01
#define MODE0_OUTPUT_VIDEO_SELECT_PAL_M			0x02


/*------------------------------*/
/* ADV7179_MODE1                */
/*------------------------------*/
#define MODE1_COLORBAR_CONTROL_BIT			0x80
#define MODE1_COLORBAR_CONTROL_SFT			0x07
#define MODE1_COLORBAR_CONTROL_DISABLE			0x00
#define MODE1_COLORBAR_CONTROL_ENABLE			0x80

#define MODE1_DAC_A_CONTROL_BIT				0x40
#define MODE1_DAC_A_CONTROL_SFT				0x06
#define MODE1_DAC_A_CONTROL_NORMAL			0x00
#define MODE1_DAC_A_CONTROL_POWERDOWN			0x40

#define MODE1_DAC_B_CONTROL_BIT				0x20
#define MODE1_DAC_B_CONTROL_SFT				0x05
#define MODE1_DAC_B_CONTROL_NORMAL			0x00
#define MODE1_DAC_B_CONTROL_POWERDOWN			0x20

#define MODE1_DAC_C_CONTROL_BIT				0x08
#define MODE1_DAC_C_CONTROL_SFT				0x03
#define MODE1_DAC_C_CONTROL_NORMAL			0x00
#define MODE1_DAC_C_CONTROL_POWERDOWN			0x08

#define MODE1_CLOSED_CAPTIONING_FIELD_BIT		0x06
#define MODE1_CLOSED_CAPTIONING_FIELD_SFT		0x01
#define MODE1_CLOSED_CAPTIONING_FIELD_NO_DATAOUT	0x00
#define MODE1_CLOSED_CAPTIONING_FIELD_ODD_ONLY		0x02
#define MODE1_CLOSED_CAPTIONING_FIELD_EVEN_ONLY		0x04
#define MODE1_CLOSED_CAPTIONING_FIELD_DATAOUT		0x06

#define MODE1_INTERLACE_CONTROL_BIT			0x01
#define MODE1_INTERLACE_CONTROL_SFT			0x00
#define MODE1_INTERLACE_CONTROL_INTERLACED		0x00
#define MODE1_INTERLACE_CONTROL_NONINTERLACED		0x01


/*------------------------------*/
/* ADV7179_MODE2                */
/*------------------------------*/
#define MODE2_LOW_POWER_MODE_BIT			0x40
#define MODE2_LOW_POWER_MODE_SFT			0x06
#define MODE2_LOW_POWER_MODE_DISABLE			0x00
#define MODE2_LOW_POWER_MODE_ENABLE			0x40

#define MODE2_BURST_CONTROL_BIT				0x20
#define MODE2_BURST_CONTROL_SFT				0x05
#define MODE2_BURST_CONTROL_ENABLE			0x00
#define MODE2_BURST_CONTROL_DISABLE			0x20

#define MODE2_CHROMINANCE_CONTROL_BIT			0x10
#define MODE2_CHROMINANCE_CONTROL_SFT			0x04
#define MODE2_CHROMINANCE_CONTROL_ENABLE		0x00
#define MODE2_CHROMINANCE_CONTROL_DISABLE		0x10

#define MODE2_ACTIVEVIDEO_LINE_BIT			0x08
#define MODE2_ACTIVEVIDEO_LINE_SFT			0x03
#define MODE2_ACTIVEVIDEO_LINE_720PIX			0x00
#define MODE2_ACTIVEVIDEO_LINE_710_702PIX		0x08

#define MODE2_GENLOCK_CONTROL_BIT			0x06
#define MODE2_GENLOCK_CONTROL_SFT			0x01
#define MODE2_GENLOCK_CONTROL_DISABLE			0x00
#define MODE2_GENLOCK_CONTROL_SUBCARRIER_RESET_PIN	0x02
#define MODE2_GENLOCK_CONTROL_RTC_PIN			0x06

#define MODE2_AQUARE_PIXEL_CONTROL_BIT			0x01
#define MODE2_AQUARE_PIXEL_CONTROL_SFT			0x00
#define MODE2_AQUARE_PIXEL_CONTROL_DISABLE		0x00
#define MODE2_AQUARE_PIXEL_CONTROL_ENABLE		0x01


/*------------------------------*/
/* ADV7179_MODE3                */
/*------------------------------*/
#define MODE3_INPUT_DEFAULT_COLOR_BIT			0x80
#define MODE3_INPUT_DEFAULT_COLOR_SFT			0x07
#define MODE3_INPUT_DEFAULT_COLOR_DISABLE		0x00
#define MODE3_INPUT_DEFAULT_COLOR_ENABLE		0x80

#define MODE3_TTXREQ_BIT_MODE_CONTROL_BIT		0x40
#define MODE3_TTXREQ_BIT_MODE_CONTROL_SFT		0x06
#define MODE3_TTXREQ_BIT_MODE_CONTROL_DISABLE		0x00
#define MODE3_TTXREQ_BIT_MODE_CONTROL_ENABLE		0x40

#define MODE3_TELETEXT_BIT				0x20
#define MODE3_TELETEXT_SFT				0x05
#define MODE3_TELETEXT_DISABLE				0x00
#define MODE3_TELETEXT_ENABLE				0x20

#define MODE3_CHROMA_OUTPUT_SELECT_BIT			0x10
#define MODE3_CHROMA_OUTPUT_SELECT_SFT			0x04
#define MODE3_CHROMA_OUTPUT_SELECT_DISABLE		0x00
#define MODE3_CHROMA_OUTPUT_SELECT_ENABLE		0x10

#define MODE3_DAC_OUTPUT_BIT				0x08
#define MODE3_DAC_OUTPUT_SFT				0x03
#define MODE3_DAC_OUTPUT_SCART				0x00
#define MODE3_DAC_OUTPUT_EUROSCART			0x08

#define MODE3_VBI_OPEN_BIT				0x04
#define MODE3_VBI_OPEN_SFT				0x02
#define MODE3_VBI_OPEN_DISABLE				0x00
#define MODE3_VBI_OPEN_ENABLE				0x04


/*------------------------------*/
/* ADV7179_MODE4                */
/*------------------------------*/
#define MODE4_PEDESTAL_CONTROL_BIT			0x10
#define MODE4_PEDESTAL_CONTROL_SFT			0x04
#define MODE4_PEDESTAL_CONTROL_OFF			0x00
#define MODE4_PEDESTAL_CONTROL_ON			0x10

#define MODE4_VSYNC_3H_BIT				0x08
#define MODE4_VSYNC_3H_SFT				0x03
#define MODE4_VSYNC_3H_DISABLE				0x00
#define MODE4_VSYNC_3H_ENABLE				0x08

#define MODE4_RGB_SYNC_BIT				0x04
#define MODE4_RGB_SYNC_SFT				0x02
#define MODE4_RGB_SYNC_DISABLE				0x00
#define MODE4_RGB_SYNC_ENABLE				0x04

#define MODE4_RGBYUV_CONTROL_BIT			0x02
#define MODE4_RGBYUV_CONTROL_SFT			0x01
#define MODE4_RGBYUV_CONTROL_RGBOUTPUT			0x00
#define MODE4_RGBYUV_CONTROL_YUVOUTPUT			0x02

#define MODE4_OUTPUT_SELECT_BIT				0x01
#define MODE4_OUTPUT_SELECT_SFT				0x00
#define MODE4_OUTPUT_SELECT_YCOUTPUT			0x00
#define MODE4_OUTPUT_SELECT_RGBYUVOUTPUT		0x01


/*------------------------------*/
/* ADV7179_SUBCAREER_FREQUENCY0-3 */
/*------------------------------*/
#define SUBCAREER_FREQUENCY_BIT				0xFF
#define SUBCAREER_FREQUENCY_SFT				0x00





#endif /* _ADV7179_H_ */


