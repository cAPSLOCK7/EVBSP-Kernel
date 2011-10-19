/*
 * File Name       : drivers/video/emxx/emxx_lcd_common.h
 * Function        : LCD Driver internal definitions
 * Release Version : Ver 1.00
 * Release Date    : 2010.04.01
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

#ifndef _EMXX_LCD_COMMON_H_
#define _EMXX_LCD_COMMON_H_

/********************************************************
 *  Definitions                                         *
 *******************************************************/
/*
 * Flags
 */
/* DSP mix ON/OFF */
#define MIX_DSP_OFF		0
#define MIX_DSP_ON		1
/* DSP pause status */
#define PAUSE_DSP_OFF		0
#define PAUSE_DSP_ON		1
/* callback to V4L2 status */
#define CALLBACK_V4L2_OFF	0
#define CALLBACK_V4L2_ON	1
/* update screen status */
#define UPDATE_OFF		0
#define UPDATE_ON		1
/* display frame */
#define DISPLAY_FRAME_NO_A	0
#define DISPLAY_FRAME_NO_B	1

/* IMC layer setup */
#define SET_LAYER_2D		1
#define SET_LAYER_V4L2		4
#define CLR_LAYER_BIT		8
#define CLR_LAYER_V4L2		(CLR_LAYER_BIT | SET_LAYER_V4L2)

/*
 * Error type
 */
#define LCD_SUCCESS					0
#define LCD_INIT_ERROR__LCDC_MMIO_NOT_RESERVE		-1
#define LCD_INIT_ERROR__LCDC_MMIO_NOT_REMAP		-2
#define LCD_INIT_ERROR__LCDM_INIT			-3
#define LCD_INIT_ERROR__IMC_INIT			-4
#define LCD_INIT_ERROR__LCD_IRQ_NOT_REQUEST		-5
#define LCD_INIT_ERROR__LCDM_START			-6


/*
 * LCD H/W define
 */
#define LCD_FLAME_A				0x00000001
#define LCD_FLAME_B				0x00000002


/*
 * timer
 */
/* (MIX_DSP_TIMEOUT * 10ms) is waittime */
#define MIX_DSP_TIMEOUT		10
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
/* (FRAMECACHE_TIMEOUT * 10ms) is waittime */
#define FRAMECACHE_TIMEOUT	10
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
#define LCD_TIMER_STOP		0
#define LCD_TIMER_START		1


#endif /* _EMXX_LCD_COMMON_H_ */
