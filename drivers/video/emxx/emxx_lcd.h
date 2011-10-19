/*
 * File Name       : drivers/video/emxx/emxx_lcd.h
 * Function        : LCD Driver definitions
 * Release Version : Ver 1.03
 * Release Date    : 2010.09.02
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


#ifndef _EMXX_LCD_H_
#define _EMXX_LCD_H_



/********************************************************
 *  Variables                                           *
 *******************************************************/
extern unsigned int  uiInverseFlag_tmp;	/* inverse mode Flag */

/* LCDC MMIO */
extern char         *LCDCMmioV;

/* V4L2 field */
extern unsigned int v4l2_field;

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
extern int     IMC_reset_flg;
#endif /* CONFIG_PM || CONFIG_DPM */
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
extern unsigned long save_ckrqmode;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

/* FBIOBLANK */
struct lcd_blank_state {
	int current_mode;
	int lcd_backlight;
	int lcd_output;
	int lcd_clock;
};
#define LCD_BLANK_STATE struct lcd_blank_state
extern LCD_BLANK_STATE blank_state;


/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
extern int          emxx_lcd_blank(int blank_mode);
extern int          emxx_lcd_set_v4l2_image(V4L2_IMAGE_INFO *v4l2_image_info);
extern int          emxx_lcd_set_fb_image(FB_IMAGE_INFO *fb_image_info);
extern int   __init emxx_lcd_init_module(void);
extern void         emxx_lcd_exit_module(void);
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
extern int          emxx_lcd_suspend(struct platform_device *dev,
		     pm_message_t state);
extern int          emxx_lcd_resume(struct platform_device *dev);
#endif /* CONFIG_PM || CONFIG_DPM */

extern irqreturn_t  lcd_irq_handler(int irq, void *dev_id);

extern void         lcd_callback_imc_refresh(void);
extern void         lcd_callback_imc_wb(int status);


#endif /* _EMXX_LCD_H_ */
