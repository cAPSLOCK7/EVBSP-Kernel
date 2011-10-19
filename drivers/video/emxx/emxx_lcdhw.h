/*
 * File Name       : drivers/video/emxx/emxx_lcdhw.h
 * Function        : LCD Driver (H/W Control)
 * Release Version : Ver 1.18
 * Release Date    : 2011.01.24
 *
 * Copyright (C) 2010, 2011 Renesas Electronics Corporation
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


#ifndef _EMXX_LCDHW_H_
#define _EMXX_LCDHW_H_


/********************************************************
 * HSYNC / VSYNC timing definitions                     *
 *******************************************************/
#define HAREA_LCD		FRONT_WIDTH_LCD
#define HPULSE_LCD		1
#define HFRONTP_LCD		6
#define HBACKP_LCD		4
#define VAREA_LCD		FRONT_HEIGHT_LCD
#define VPULSE_LCD		1
#define VFRONTP_LCD		1
#define VBACKP_LCD		6

#define HAREA_1080I		FRONT_WIDTH_1080I
#define HPULSE_1080I		44
#define HFRONTP_1080I		88
#define HBACKP_1080I		148
#define VAREA_1080I		FRONT_HEIGHT_1080I
#define VPULSE_1080I		5
#define VFRONTP_1080I		2
#define VBACKP_1080I		15

#define HAREA_720P		FRONT_WIDTH_720P
#define HPULSE_720P		40
#define HFRONTP_720P		110
#define HBACKP_720P		220
#define VAREA_720P		FRONT_HEIGHT_720P
#define VPULSE_720P		5
#define VFRONTP_720P		5
#define VBACKP_720P		20


/********************************************************
 *  Macros                                              *
 *******************************************************/
#define LCD_FIFO_REQ_UNMASK	writel((readl(SMU_DFS_FIFO_REQMASK) & ~0x01),\
					 SMU_DFS_FIFO_REQMASK)
#define LCD_FIFO_REQ_MASK	writel((readl(SMU_DFS_FIFO_REQMASK) | 0x01),\
					 SMU_DFS_FIFO_REQMASK)


/********************************************************
 *  Variables                                           *
 *******************************************************/
extern int init_is_first;
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
extern int direct_path;
extern int direct_reserved;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
extern enum EMXX_FB_OUTPUT_MODE lcdc_output_mode;
extern int lcd_field;
#define FIELD_NONE -1
#define FIELD_ODD  0
#define FIELD_EVEN 1
extern int refresh_reserved;

extern struct emxx_imc_info		imc_info;

/* IMC layer */
extern struct l01_param			fb_layer;
extern struct l2_param			v4l2_layer;
extern struct emxx_imc_update_vsync	ImcNxtVsync;
extern struct imc_keycolor_param	ImcNxtKeycol;
extern struct imc_bytelane_param	ImcNxtBytelane;


/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
/* ------------------ LCD control function --------------------------------- */
extern int           change_frame(void);
extern int           init_lcdhw(void);
extern void          exit_lcdhw(void);
extern int           change_output(enum EMXX_FB_OUTPUT_MODE old_mode,
		      enum EMXX_FB_OUTPUT_MODE new_mode);


/* ------------------ LCD module initialize function ----------------------- */
extern int           lcd_module_hw_standby(void);
extern int           lcd_module_hw_wakeup(void);

/* ------------------ LCD H/W control function ----------------------------- */
extern void          lcd_hw_save_reg(void);
extern void          lcd_hw_restore_reg(void);
extern void          lcd_hw_start(void);
extern void          lcd_hw_stop(void);
extern void          lcd_hw_reset(void);
extern void          lcd_hw_unreset(void);
extern void          lcd_hw_backlight_off(void);
extern void          lcd_hw_backlight_on(void);

/* ------------------ LCDC rgister check function -------------------------- */
extern unsigned long lcd_hw_chk_status(void);
extern unsigned long lcd_hw_chk_bussel(void);
extern unsigned long lcd_hw_chk_int_status(void);
extern unsigned long lcd_hw_chk_int_rawstatus(void);

/* ------------------ LCDC interruput control function --------------------- */
extern void          lcd_hw_int_enable(void);
extern void          lcd_hw_int_disable(void);
extern void          lcd_hw_int_factor_clr(void);

/* ------------------ IMC initialize function ------------------------------ */
extern int           imc_hw_restore_reg(void);

/* ------------------ IMC control function --------------------------------- */
extern int imc_hw_set_update_vsync(struct emxx_imc_update_vsync *vsync);
extern int imc_hw_set_update_reserve(int type, int mixdsp, int update);


#endif /* _EMXX_LCDHW_H_ */

