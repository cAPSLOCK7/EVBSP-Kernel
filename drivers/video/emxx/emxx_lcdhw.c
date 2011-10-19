/*
 * File Name       : drivers/video/emxx/emxx_lcdhw.c
 * Function        : LCD Driver (H/W Control)
 * Release Version : Ver 1.30
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


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define _DEBUG_LCDHW  0x00 /* 00008421(bit) */
			   /* 0x01: debug function in
			    * 0x02: debug function out
			    * 0x04: debug IMC
			    * 0x08: debug frame change
			    * 0x10: debug INTERLACE TB/BT
			    * 0x40: debug FBIOBLANK
			    */


#define DEV_NAME "emxx_lcdhw"


/********************************************************
 *  Macros                                              *
 *******************************************************/
#define printk_err(fmt, arg...) \
	do {                     \
		printk(KERN_ERR DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_wrn(fmt, arg...) \
	do {                     \
		printk(KERN_WARNING DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_info(fmt, arg...) \
	do {                      \
		printk(KERN_INFO DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#if _DEBUG_LCDHW
#define printk_dbg(level, fmt, arg...) \
	do {                            \
		if (level > 0) \
			printk(KERN_DEBUG DEV_NAME ": %s: " fmt, \
			__func__, ## arg); \
	} while (0)
#else
#define printk_dbg(level, fmt, arg...) \
	;
#endif


/********************************************************
 *  Include Files                                       *
 *******************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/system.h>

#include <mach/irqs.h>
#include <mach/spi.h>
#ifdef CONFIG_MACH_EMEV
#include <mach/smu.h>
#endif
#include <mach/pmu.h>
#include <mach/hardware.h>
#include <mach/emxx_mem.h>

#include <mach/gpio.h>

#include <mach/imc.h>
#include <mach/emxx_imc.h>
#include "../../imc/emxx_imc.h"

#include "lcdc.h"
#include "emxx_common.h"
#include "emxx_lcd_common.h"
#include "emxx_lcdhw.h"
#include "emxx_lcd.h"


/********************************************************
 *  IMC request structure                               *
 *******************************************************/
struct vsync_param {
	/* for struct emxx_imc_update_vsync */
	unsigned long			imc_cpubufsel;
	struct imc_wb_param		param_wb;
	unsigned long			imc_mirror;
	struct imc_alphasel_param	param_alphasel;
	unsigned long			imc_l0_scanmode;
	unsigned long			imc_l1a_scanmode;
	unsigned long			imc_l1b_scanmode;
	unsigned long			imc_l1c_scanmode;
	unsigned long			imc_l2a_scanmode;
	unsigned long			imc_l2b_scanmode;
	unsigned long			imc_bg_scanmode;
};

struct emxx_imc_param {
	struct emxx_imc_preset		imc_preset;
	struct emxx_imc_update_vsync	imc_vsync;
	struct emxx_imc_update_reserve	imc_reserve;

	/* for struct emxx_imc_preset */
	unsigned long			imc_control;
	unsigned long			imc_datareq;
	struct imc_gamma_param		param_gamma;
	struct imc_yuv_param		param_yuv;
	struct imc_burst_param		param_burst;

	/* for struct emxx_imc_update_vsync */
	struct vsync_param		param_vsync;

	/* for struct emxx_imc_update_reserve */
	struct l01_param		param_l0;
	struct l01_param		param_l1a;
	struct l01_param		param_l1b;
	struct l01_param		param_l1c;
	struct l2_param			param_l2a;
	struct l2_param			param_l2b;
	struct bg_param			param_bg;

	imc_callback_func_refresh	callback_refresh;
	imc_callback_func_wb		callback_wb;
};

       struct emxx_imc_info           imc_info;
static struct emxx_imc_param          *imc_param;
static struct emxx_imc_preset         *imc_preset;
static struct emxx_imc_update_vsync   *imc_vsync;
static struct emxx_imc_update_reserve *imc_reserve;

       struct l01_param			fb_layer;
       struct l2_param			v4l2_layer;
       struct emxx_imc_update_vsync	ImcNxtVsync;
       struct vsync_param		ImcNxtVsync_Param;


/********************************************************
 * IMC register initialize                              *
 *******************************************************/
#define IMC_CONTROL_INIT		0x00000000
#define IMC_REFRESH_INIT		0x00000000
#define IMC_DATAREQ_INIT		0x00000100

#define IMC_CPUBUFSEL_INIT		0x00000000

#define IMC_GAMMA_EN_INIT		0x00000000
#define IMC_GAMMA_ADR_INIT		0x00000000

#define IMC_WB_AREAADR_P_INIT		0x00000000
#define IMC_WB_HOFFSET_INIT		0x00000000
#define IMC_WB_FORMAT_INIT		0x00000000
#define IMC_WB_SIZE_INIT		0x00000000
#define IMC_WB_AREAADR_Q_INIT		0x00000000
#define IMC_WB_BUFSEL_INIT		0x00000000
#define IMC_WB_MPOSITION_INIT		0x00000000
#define IMC_WB_MSIZE_INIT		0x00000000
#define IMC_BACKCOLOR_INIT		0x00000000
#define IMC_WB_BYTELANE_INIT		0x0000E400
#define IMC_WB_SCANMODE_INIT		0x00000000

#define IMC_MIRROR_INIT			0x00000000

#define IMC_YGAINOFFSET_INIT		0x00000080
#define IMC_UGAINOFFSET_INIT		0x00000080
#define IMC_VGAINOFFSET_INIT		0x00000080
#define IMC_YUV2RGB_INIT		0x00000000
#define IMC_COEF_R0_INIT		0x00000000
#define IMC_COEF_R1_INIT		0x00000000
#define IMC_COEF_R2_INIT		0x00000000
#define IMC_COEF_R3_INIT		0x00000000
#define IMC_COEF_G0_INIT		0x00000000
#define IMC_COEF_G1_INIT		0x00000000
#define IMC_COEF_G2_INIT		0x00000000
#define IMC_COEF_G3_INIT		0x00000000
#define IMC_COEF_B0_INIT		0x00000000
#define IMC_COEF_B1_INIT		0x00000000
#define IMC_COEF_B2_INIT		0x00000000
#define IMC_COEF_B3_INIT		0x00000000

#define IMC_ALPHASEL0_INIT		0x00000000
#define IMC_ALPHASEL1_INIT		0x00000000

#define IMC_BURST_EN_INIT		0x00000101
#define IMC_THRESHOLD_INIT		0x00001010

#define IMC_L0_CONTROL_INIT		0x00000000
#define IMC_L0_FORMAT_INIT		0x00000000
#define IMC_L0_BUFSEL_INIT		0x00000000
#define IMC_L0_BYTELANE_INIT		0x0000E400
#define IMC_L0_KEYENABLE_INIT		0x00000000
#define IMC_L0_KEYCOLOR_INIT		0x00000000
#define IMC_L0_ALPHA_INIT		0x00000000
#define IMC_L0_RESIZE_INIT		0x00000000
#define IMC_L0_MIRROR_INIT		0x00000000
#define IMC_L0_OFFSET_INIT		0x00000000
#define IMC_L0_FRAMEADR_P_INIT		0x00000000
#define IMC_L0_FRAMEADR_Q_INIT		0x00000000
#define IMC_L0_POSITION_INIT		0x00000000
#define IMC_L0_SIZE_INIT		0x00000000
#define IMC_L0_MPOSITION_INIT		0x00000000
#define IMC_L0_MSIZE_INIT		0x00000000
#define IMC_L0_SCANMODE_INIT		0x00000000

#define IMC_L1A_CONTROL_INIT		0x00000000
#define IMC_L1A_FORMAT_INIT		0x00000000
#define IMC_L1A_BUFSEL_INIT		0x00000000
#define IMC_L1A_BYTELANE_INIT		0x0000E400
#define IMC_L1A_KEYENABLE_INIT		0x00000000
#define IMC_L1A_KEYCOLOR_INIT		0x00000000
#define IMC_L1A_ALPHA_INIT		0x00000000
#define IMC_L1A_RESIZE_INIT		0x00000000
#define IMC_L1A_MIRROR_INIT		0x00000000
#define IMC_L1A_OFFSET_INIT		0x00000000
#define IMC_L1A_FRAMEADR_P_INIT		0x00000000
#define IMC_L1A_FRAMEADR_Q_INIT		0x00000000
#define IMC_L1A_POSITION_INIT		0x00000000
#define IMC_L1A_SIZE_INIT		0x00000000
#define IMC_L1A_MPOSITION_INIT		0x00000000
#define IMC_L1A_MSIZE_INIT		0x00000000
#define IMC_L1A_SCANMODE_INIT		0x00000000

#define IMC_L1B_CONTROL_INIT		0x00000000
#define IMC_L1B_FORMAT_INIT		0x00000000
#define IMC_L1B_BUFSEL_INIT		0x00000000
#define IMC_L1B_BYTELANE_INIT		0x0000E400
#define IMC_L1B_KEYENABLE_INIT		0x00000000
#define IMC_L1B_KEYCOLOR_INIT		0x00000000
#define IMC_L1B_ALPHA_INIT		0x00000000
#define IMC_L1B_RESIZE_INIT		0x00000000
#define IMC_L1B_MIRROR_INIT		0x00000000
#define IMC_L1B_OFFSET_INIT		0x00000000
#define IMC_L1B_FRAMEADR_P_INIT		0x00000000
#define IMC_L1B_FRAMEADR_Q_INIT		0x00000000
#define IMC_L1B_POSITION_INIT		0x00000000
#define IMC_L1B_SIZE_INIT		0x00000000
#define IMC_L1B_MPOSITION_INIT		0x00000000
#define IMC_L1B_MSIZE_INIT		0x00000000
#define IMC_L1B_SCANMODE_INIT		0x00000000

#define IMC_L1C_CONTROL_INIT		0x00000000
#define IMC_L1C_FORMAT_INIT		0x00000000
#define IMC_L1C_BUFSEL_INIT		0x00000000
#define IMC_L1C_BYTELANE_INIT		0x0000E400
#define IMC_L1C_KEYENABLE_INIT		0x00000000
#define IMC_L1C_KEYCOLOR_INIT		0x00000000
#define IMC_L1C_ALPHA_INIT		0x00000000
#define IMC_L1C_RESIZE_INIT		0x00000000
#define IMC_L1C_MIRROR_INIT		0x00000000
#define IMC_L1C_OFFSET_INIT		0x00000000
#define IMC_L1C_FRAMEADR_P_INIT		0x00000000
#define IMC_L1C_FRAMEADR_Q_INIT		0x00000000
#define IMC_L1C_POSITION_INIT		0x00000000
#define IMC_L1C_SIZE_INIT		0x00000000
#define IMC_L1C_MPOSITION_INIT		0x00000000
#define IMC_L1C_MSIZE_INIT		0x00000000
#define IMC_L1C_SCANMODE_INIT		0x00000000

#define IMC_L2A_CONTROL_INIT		0x00000000
#define IMC_L2A_FORMAT_INIT		0x00000000
#define IMC_L2A_BUFSEL_INIT		0x00000000
#define IMC_L2A_BYTELANE_INIT		0x0000E4E4
#define IMC_L2A_RESIZE_INIT		0x00000000
#define IMC_L2A_MIRROR_INIT		0x00000000
#define IMC_L2A_OFFSET_INIT		0x00000000
#define IMC_L2A_FRAMEADR_YP_INIT	0x00000000
#define IMC_L2A_FRAMEADR_UP_INIT	0x00000000
#define IMC_L2A_FRAMEADR_VP_INIT	0x00000000
#define IMC_L2A_FRAMEADR_YQ_INIT	0x00000000
#define IMC_L2A_FRAMEADR_UQ_INIT	0x00000000
#define IMC_L2A_FRAMEADR_VQ_INIT	0x00000000
#define IMC_L2A_POSITION_INIT		0x00000000
#define IMC_L2A_SIZE_INIT		0x00000000
#define IMC_L2A_MPOSITION_INIT		0x00000000
#define IMC_L2A_MSIZE_INIT		0x00000000
#define IMC_L2A_SCANMODE_INIT		0x00000000

#define IMC_L2B_CONTROL_INIT		0x00000000
#define IMC_L2B_FORMAT_INIT		0x00000000
#define IMC_L2B_BUFSEL_INIT		0x00000000
#define IMC_L2B_BYTELANE_INIT		0x0000E4E4
#define IMC_L2B_RESIZE_INIT		0x00000000
#define IMC_L2B_MIRROR_INIT		0x00000000
#define IMC_L2B_OFFSET_INIT		0x00000000
#define IMC_L2B_FRAMEADR_YP_INIT	0x00000000
#define IMC_L2B_FRAMEADR_UP_INIT	0x00000000
#define IMC_L2B_FRAMEADR_VP_INIT	0x00000000
#define IMC_L2B_FRAMEADR_YQ_INIT	0x00000000
#define IMC_L2B_FRAMEADR_UQ_INIT	0x00000000
#define IMC_L2B_FRAMEADR_VQ_INIT	0x00000000
#define IMC_L2B_POSITION_INIT		0x00000000
#define IMC_L2B_SIZE_INIT		0x00000000
#define IMC_L2B_MPOSITION_INIT		0x00000000
#define IMC_L2B_MSIZE_INIT		0x00000000
#define IMC_L2B_SCANMODE_INIT		0x00000000

#define IMC_BG_FORMAT_INIT		0x00000000
#define IMC_BG_BUFSEL_INIT		0x00000000
#define IMC_BG_BYTELANE_INIT		0x0000E400
#define IMC_BG_RESIZE_INIT		0x00000000
#define IMC_BG_MIRROR_INIT		0x00000000
#define IMC_BG_OFFSET_INIT		0x00000000
#define IMC_BG_FRAMEADR_P_INIT		0x00000000
#define IMC_BG_FRAMEADR_Q_INIT		0x00000000
#define IMC_BG_MPOSITION_INIT		0x00000000
#define IMC_BG_MSIZE_INIT		0x00000000
#define IMC_BG_SCANMODE_INIT		0x00000000


/********************************************************
 * LCDC register initialize                             *
 *******************************************************/
/* Control for LCDC      : RGB888 output */
#define LCD_CONTROL_INIT	LCD_OFORMAT_RGB888

/* Input Format select   : RGB888 input  */
#define LCD_IFORMAT_INIT	LCD_IFORMAT_RGB888

#define LCD_QOS_INIT		0x00000180
#define LCD_DATAREQ_INIT	0x00000001
/* Access Bus Select             : Local Bus         */
#define LCD_BUSSEL_UPDATE	LCD_BUSSEL_LOCAL
/* Access Bus Select             : Black Only        */
#define LCD_BUSSEL_INIT		LCD_BUSSEL_BLACK
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
/* AREA Address for Frame Buffer :                       */
#define LCD_AREAADR_ODD_INIT	FRAMEBUF_START
#define LCD_AREAADR_EVEN_INIT	(FRAMEBUF_START + FRAMEBUF_LENGTH / 2)
#else /* CONFIG_EMXX_LCD_FRAMECACHE */
/* AREA Address for Frame Buffer :                       */
#define LCD_AREAADR_ODD_INIT	0x00000000
#define LCD_AREAADR_EVEN_INIT	0x00000000
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
#define LCD_BACKCOL_INIT \
 (0x00 << LCD_BGRED_SFT | 0x00 << LCD_BGGREEN_SFT | 0x00 << LCD_BGBLUE_SFT)

#define LCD_COEF_Y0_INIT	0x0042
#define LCD_COEF_Y1_INIT	0x0081
#define LCD_COEF_Y2_INIT	0x0019
#define LCD_COEF_Y3_INIT	0x0010
#define LCD_COEF_U0_INIT	0x07DA
#define LCD_COEF_U1_INIT	0x07B6
#define LCD_COEF_U2_INIT	0x0070
#define LCD_COEF_U3_INIT	0x0080
#define LCD_COEF_V0_INIT	0x0070
#define LCD_COEF_V1_INIT	0x07A2
#define LCD_COEF_V2_INIT	0x07EE
#define LCD_COEF_V3_INIT	0x0080
#define LCD_BYTELANE_INIT	0x008D


/********************************************************
 * LCD Module initialize command                        *
 *******************************************************/
#define LCDM_SPI_CMD_POWERON	81
#if 0
#define LCDM_SPI_CMD_START	1
#endif
#define LCDM_SPI_CMD_POWEROFF	4
#define LCDM_SPI_CMD_STANDBY	1
#define LCDM_SPI_CMD_WAKEUP	1

const unsigned int lcdm_spi_cmd_poweron[LCDM_SPI_CMD_POWERON][2] = {
	{0x03, 0x01}, {0x00, 0x00}, {0x01, 0x01}, {0x04, 0x00}, {0x05, 0x14},
	{0x06, 0x24}, {0x10, 0xd7}, {0x11, 0x00}, {0x12, 0x00}, {0x13, 0x55},
	{0x14, 0x01}, {0x15, 0x70}, {0x16, 0x1e}, {0x17, 0x25}, {0x18, 0x25},
	{0x19, 0x02}, {0x1a, 0x02}, {0x1b, 0xa0}, {0x20, 0x2f}, {0x21, 0x0f},
	{0x22, 0x0f}, {0x23, 0x0f}, {0x24, 0x0f}, {0x25, 0x0f}, {0x26, 0x0f},
	{0x27, 0x00}, {0x28, 0x02}, {0x29, 0x02}, {0x2a, 0x02}, {0x2b, 0x0f},
	{0x2c, 0x0f}, {0x2d, 0x0f}, {0x2e, 0x0f}, {0x2f, 0x0f}, {0x30, 0x0f},
	{0x31, 0x0f}, {0x32, 0x00}, {0x33, 0x02}, {0x34, 0x02}, {0x35, 0x02},
	{0x50, 0x0c}, {0x53, 0x42}, {0x54, 0x42}, {0x55, 0x41}, {0x56, 0x14},
	{0x59, 0x88}, {0x5a, 0x01}, {0x5b, 0x00}, {0x5c, 0x02}, {0x5d, 0x0c},
	{0x5e, 0x1c}, {0x5f, 0x27}, {0x62, 0x49}, {0x63, 0x27}, {0x66, 0x76},
	{0x67, 0x27}, {0x70, 0x01}, {0x71, 0x0e}, {0x72, 0x02}, {0x73, 0x0c},
	{0x76, 0x0c}, {0x79, 0x30}, {0x82, 0x00}, {0x83, 0x00}, {0x84, 0xfc},
	{0x86, 0x00}, {0x88, 0x00}, {0x8a, 0x00}, {0x8b, 0x00}, {0x8c, 0x00},
	{0x8d, 0xfc}, {0x8f, 0x00}, {0x91, 0x00}, {0x93, 0x00}, {0x94, 0x00},
	{0x95, 0x00}, {0x96, 0xfc}, {0x98, 0x00}, {0x9a, 0x00}, {0x9c, 0x00},
	{0x9d, 0x00}
};

const unsigned int lcdm_spi_cmd_poweroff[LCDM_SPI_CMD_POWEROFF][2] = {
	{0x10, 0x05}, {0x10, 0x01}, {0x10, 0x00}, {0x03, 0x01}
};

const unsigned int lcdm_spi_cmd_standby[LCDM_SPI_CMD_STANDBY][2] = {
	{0x02, 0x01}
};

const unsigned int lcdm_spi_cmd_wakeup[LCDM_SPI_CMD_WAKEUP][2] = {
	{0x02, 0x00}
};


/********************************************************
 *  Variables                                           *
 *******************************************************/
       int init_is_first   = 1;
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
       int direct_path     = 0;
       int direct_reserved = 0;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
       enum EMXX_FB_OUTPUT_MODE lcdc_output_mode = EMXX_FB_OUTPUT_MODE_LCD;
       int lcd_field = FIELD_NONE;
       int refresh_reserved = 0;


static unsigned int harea	= HAREA_LCD;
static unsigned int hpulse	= HPULSE_LCD;
static unsigned int hfrontp	= HFRONTP_LCD;
static unsigned int hbackp	= HBACKP_LCD;
static unsigned int varea	= VAREA_LCD;
static unsigned int vpulse	= VPULSE_LCD;
static unsigned int vfrontp	= VFRONTP_LCD;
static unsigned int vbackp	= VBACKP_LCD;


/********************************************************
 *  Prototype declarations of local function            *
 *******************************************************/
/*
 * LCD Driver (H/W)
 */

/* ------------------ LCD initialize function ------------------------------ */
static inline void   change_pinsel_wvga(void);
static inline void   change_pinsel_hdmi(void);
static inline void   unreset_lcdc(void);
static inline void   unreset_usi3(void);
static void          lcd_hw_init(void);
static void          lcd_controller_hw_init(void);
static inline void   lcd_controller_hw_init_lcdsize(void);

/* ------------------ LCD module initialize function ----------------------- */
static void          lcd_module_hw_unreset(void);
static void          lcd_module_hw_reset(void);
static int           lcd_module_hw_power_on(void);
static int           lcd_module_hw_power_off(void);
static int           lcd_module_hw_spiw(unsigned char cmd, unsigned char param);

/* ------------------ IMC initialize function ------------------------------ */
static int           imc_hw_init(void);

static void         *imc_hw_init_preset_gamma(
 struct imc_gamma_param *param_gamma);
static void         *imc_hw_init_preset_yuv(struct imc_yuv_param *param_yuv);
static void         *imc_hw_init_preset_burst(
 struct imc_burst_param *param_burst);
static void         *imc_hw_init_vsync_wb(struct imc_wb_param *param_wb);
static void         *imc_hw_init_vsync_alphasel(struct imc_alphasel_param *
						param_alphasel);
static void         *imc_hw_init_reserve_l0(struct l01_param *param_l0);
static void         *imc_hw_init_reserve_l1a(struct l01_param *param_l1a);
static void         *imc_hw_init_reserve_l1b(struct l01_param *param_l1b);
static void         *imc_hw_init_reserve_l1c(struct l01_param *param_l1c);
static void         *imc_hw_init_reserve_l2a(struct l2_param *param_l2a);
static void         *imc_hw_init_reserve_l2b(struct l2_param *param_l2b);
static void         *imc_hw_init_reserve_bg(struct bg_param *param_bg);


/* ------------------ inline function -------------------------------------- */
inline void chk_errno(int errno, char **cnum)
{
	switch (errno) {
	case ENODEV:
		*cnum = "ENODEV";
		break;
	case EINVAL:
		*cnum = "EINVAL";
		break;
	case EPERM:
		*cnum = "EPERM";
		break;
	case EIO:
		*cnum = "EIO";
		break;
	case EAGAIN:
		*cnum = "EAGAIN";
		break;
	default:
		*cnum = "****";
		break;
	}
}


/* ------------------ LCD control function --------------------------------- */
/******************************************************************************
* MODULE   : change_output
* FUNCTION : change LCDC output
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int change_output(enum EMXX_FB_OUTPUT_MODE old_mode,
 enum EMXX_FB_OUTPUT_MODE new_mode)
{
	unsigned long imc_wb_scanmode;

	lcd_hw_backlight_off();
	lcd_module_hw_standby();
	lcd_module_hw_power_off();
	lcd_hw_stop();
	lcd_module_hw_reset();

	refresh_reserved = 0;

	lcdc_output_mode = new_mode;
	switch (lcdc_output_mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		harea	= HAREA_LCD;
		hpulse	= HPULSE_LCD;
		hfrontp	= HFRONTP_LCD;
		hbackp	= HBACKP_LCD;
		varea	= VAREA_LCD;
		vpulse	= VPULSE_LCD;
		vfrontp	= VFRONTP_LCD;
		vbackp	= VBACKP_LCD;
		imc_wb_scanmode  = IMC_WB_SCANMODE_PROGRESSIVE;
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
		harea	= HAREA_1080I;
		hpulse	= HPULSE_1080I;
		hfrontp	= HFRONTP_1080I;
		hbackp	= HBACKP_1080I;
		varea	= VAREA_1080I;
		vpulse	= VPULSE_1080I;
		vfrontp	= VFRONTP_1080I;
		vbackp	= VBACKP_1080I;
		imc_wb_scanmode  = IMC_WB_SCANMODE_INTERLACED;
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		harea	= HAREA_720P;
		hpulse	= HPULSE_720P;
		hfrontp	= HFRONTP_720P;
		hbackp	= HBACKP_720P;
		varea	= VAREA_720P;
		vpulse	= VPULSE_720P;
		vfrontp	= VFRONTP_720P;
		vbackp	= VBACKP_720P;
		imc_wb_scanmode  = IMC_WB_SCANMODE_PROGRESSIVE;
		break;
	}

	switch (lcdc_output_mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		/* WVGA LCD */
		change_pinsel_wvga();
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		/* HDMI */
		change_pinsel_hdmi();
		unreset_usi3();
		break;
	}

#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	direct_path = 0;
	direct_reserved = 0;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
	lcd_controller_hw_init();
	lcd_controller_hw_init_lcdsize();

	ImcNxtVsync.wb->hoffset  = harea * BITS_PER_PIXEL / 8;
	ImcNxtVsync.wb->size     = (harea << IMC_WB_HSIZE_SFT) |
				   (varea << IMC_WB_VSIZE_SFT);
	ImcNxtVsync.wb->scanmode = imc_wb_scanmode;

	if (imc_hw_set_update_vsync(&ImcNxtVsync))
		printk_wrn("IMC update_vsync() failed.\n");

	lcd_module_hw_unreset();
	lcd_module_hw_power_on();
	if (blank_state.lcd_output == 1) {
		lcd_hw_start();
		lcd_module_hw_wakeup();
	}
	if (blank_state.lcd_backlight == 1)
		lcd_hw_backlight_on();

	return 0;
}


/******************************************************************************
* MODULE   : change_frame
* FUNCTION : change LCD display page
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int change_frame(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	direct_path = 0;
	direct_reserved = 0;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
	if ((lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) ||
	     (blank_state.lcd_backlight != 0))
		writel(LCD_BUSSEL_UPDATE,  LCDCMmioV + LCD_BUSSEL);

	return 0;
}


/******************************************************************************
* MODULE   : init_lcdhw
* FUNCTION : LCD(H/W) initialized
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int init_lcdhw(void)
{
	int iRet = 0;
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	save_ckrqmode = readl(SMU_CKRQ_MODE);
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

	/* initialize static parameters */
	imc_param   = NULL;
	imc_preset  = NULL;
	imc_vsync   = NULL;
	imc_reserve = NULL;

	/* LCD Module unreset           */
	lcd_module_hw_unreset();

	/* LCD Module power on          */
	iRet = lcd_module_hw_power_on();
	if (iRet) {
		printk_err("fail in LCD module initialize\n");
		return LCD_INIT_ERROR__LCDM_INIT;
	}

	/* LCDC initialize              */
	lcd_hw_init();

	/* install LCDC irq handler     */
	if (request_irq(INT_LCD, lcd_irq_handler, 0, DEV_NAME, NULL)) {
		printk_err("fail in request_irq(INT_LCD)\n");
		return LCD_INIT_ERROR__LCD_IRQ_NOT_REQUEST;
	}

	/* LCDC register initialize     */
	lcd_controller_hw_init();

	/* lcd display size set         */
	lcd_controller_hw_init_lcdsize();

	/* IMC initialize               */
	if (imc_hw_init()) {
		printk_err("fail in IMC initialize\n");
		return LCD_INIT_ERROR__IMC_INIT;
	}

	/* LCDC start                   */
	lcd_hw_start();

	/* LCD Module wake up           */
	lcd_module_hw_wakeup();

	/* BackLight on                 */
	lcd_hw_backlight_on();

	return 0;
}


/******************************************************************************
* MODULE   : exit_lcdhw
* FUNCTION : LCD(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
void exit_lcdhw(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	lcd_hw_backlight_off();
	lcd_module_hw_standby();
	lcd_module_hw_power_off();
	lcd_hw_stop();
	lcd_module_hw_reset();

	free_irq(INT_LCD, NULL);

	lcd_hw_reset();
}


/* ------------------ LCD H/W control function ----------------------------- */
/********************************************************
 *  save/restore Function Definitions                   *
 *******************************************************/
/******************************************************************************
* MODULE   : lcd_hw_save_reg
* FUNCTION : save LCD(H/W) register data
* RETURN   : 0     : success
* NOTE     : none
******************************************************************************/
void lcd_hw_save_reg(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
}


/******************************************************************************
* MODULE   : lcd_hw_restore_reg
* FUNCTION : restore LCD(H/W) register data
* RETURN   : 0     : success
* NOTE     : none
******************************************************************************/
void lcd_hw_restore_reg(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	/* restore LCD framebuffer address */
	lcd_controller_hw_init();

	/* init LCD size */
	lcd_controller_hw_init_lcdsize();
}


/******************************************************************************
* MODULE   : lcd_hw_start
* FUNCTION : start LCD(H/W)
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_start(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	printk_dbg((_DEBUG_LCDHW & 0x40), "<start LCDOUT>\n");

	/* enable auto frequency control */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) < EMXX_REV_ES2) {
#endif
	/* unmask LCD (bit0=0) */
	LCD_FIFO_REQ_UNMASK;
#ifdef CONFIG_MACH_EMEV
	}
#endif

	writel(LCD_LCDOUT_START, LCDCMmioV + LCD_LCDOUT);
}


/******************************************************************************
* MODULE   : lcd_hw_stop
* FUNCTION : stop LCD(H/W)
* RETURN   : 0     : success
* NOTE     : none
******************************************************************************/
void lcd_hw_stop(void)
{
	unsigned long ulRegVal32;
	int wait_max = 40;
	int wait_cnt = 0;

	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	printk_dbg((_DEBUG_LCDHW & 0x40), "<stop LCDOUT>\n");
	writel(LCD_LCDOUT_STOP, LCDCMmioV + LCD_LCDOUT);

	/* wait for LCD disp off (max 40ms) */
	do {
		ulRegVal32 = lcd_hw_chk_int_rawstatus();
		if (ulRegVal32 & LCD_LCDSTOP_BIT) {
			printk_dbg((_DEBUG_LCDHW & 0x01), "wait(%dms)\n",
			 wait_cnt);
			wait_cnt = wait_max;
		} else {
			if (wait_cnt < wait_max)
				mdelay(1);
		}
		wait_cnt++;
	} while (wait_cnt <= wait_max);

	/* wait for IMC stopped */
	imc_hw_wait_stop(IMC_CH0);

	/* disable auto frequency control */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) < EMXX_REV_ES2) {
#endif
	/* mask LCD (bit0=1) */
	LCD_FIFO_REQ_MASK;
#ifdef CONFIG_MACH_EMEV
	}
#endif

	refresh_reserved = 0;
	lcd_field = FIELD_NONE;
}


/******************************************************************************
* MODULE   : lcd_hw_reset
* FUNCTION : LCD(H/W) reset
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_reset(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	printk_dbg((_DEBUG_LCDHW & 0x40), "<reset and close clockgate>\n");

	/* Reset LCD */
	emxx_clkctrl_off(EMXX_CLKCTRL_LCD);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCDC);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCDPCLK);
	emxx_reset_device(EMXX_RST_LCD_A | EMXX_RST_LCD);
	emxx_close_clockgate(EMXX_CLK_LCD_C | EMXX_CLK_LCD_L | EMXX_CLK_LCD_P |
	 EMXX_CLK_LCD);
}


/******************************************************************************
* MODULE   : lcd_hw_unreset
* FUNCTION : LCD(H/W) unreset
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_unreset(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	printk_dbg((_DEBUG_LCDHW & 0x40), "<open clockgate and unreset>\n");

	/* UnReset LCD */
	emxx_open_clockgate(EMXX_CLK_LCD_C | EMXX_CLK_LCD_L | EMXX_CLK_LCD_P |
	 EMXX_CLK_LCD);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCD);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCDC);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCDPCLK);
	emxx_unreset_device(EMXX_RST_LCD_A | EMXX_RST_LCD);
	emxx_clkctrl_on(EMXX_CLKCTRL_LCD);
	emxx_clkctrl_on(EMXX_CLKCTRL_LCDC);
	emxx_clkctrl_on(EMXX_CLKCTRL_LCDPCLK);
}


/******************************************************************************
* MODULE   : lcd_hw_backlight_off
* FUNCTION : LCD(H/W) backlight control(off)
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_backlight_off(void)
{
	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
		printk_dbg((_DEBUG_LCDHW & 0x40), "<backlight Off>\n");
		/* LED1_EN:OFF LED1_RAMP:OFF LED2_EN:OFF LED2_RAMP:OFF */
		pwc_write(DA9052_LEDCONT_REG, 0x00, 0x0f);
		/* BOOST_EN:OFF LED1_IN_EN:OFF LED2_IN_EN:OFF */
		pwc_write(DA9052_BOOST_REG, 0x00, 0x07);
	}
}


/******************************************************************************
* MODULE   : lcd_hw_backlight_on
* FUNCTION : LCD(H/W) backlight control(on)
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_backlight_on(void)
{
	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
		printk_dbg((_DEBUG_LCDHW & 0x40), "<backlight On>\n");
		mdelay(110);
		/* BOOST_EN:ON LED1_IN_EN:ON LED2_IN_EN:ON */
		pwc_write(DA9052_BOOST_REG, 0x07, 0x07);
		 /* LED1_EN:ON LED1_RAMP:ON LED2_EN:ON LED2_RAMP:ON */
		pwc_write(DA9052_LEDCONT_REG, 0x0f, 0x0f);
	}
}


/* ------------------ LCDC rgister check function -------------------------- */
/*****************************************************************************
* MODULE   : lcd_hw_chk_status
* FUNCTION : check LCDC status
* RETURN   :  0 : success
*            -1 : failed
* NOTE     : none
*****************************************************************************/
unsigned long lcd_hw_chk_status(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	/* check lcd status */
	return (unsigned long)readl(LCDCMmioV + LCD_STATUS);
}


/*****************************************************************************
* MODULE   : lcd_hw_chk_int_status
* FUNCTION : check LCDC bussel
* RETURN   : none
* NOTE     : none
******************************************************************************/
unsigned long lcd_hw_chk_bussel(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	/*  check bussel */
	return (unsigned long)readl(LCDCMmioV + LCD_BUSSEL);
}


/*****************************************************************************
* MODULE   : lcd_hw_chk_int_status
* FUNCTION : check LCDC interrupt status
* RETURN   : none
* NOTE     : none
******************************************************************************/
unsigned long lcd_hw_chk_int_status(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	/*  check Interuppt Status */
	return (unsigned long)readl(LCDCMmioV + LCD_INTSTATUS);
}


/*****************************************************************************
* MODULE   : lcd_hw_chk_int_rawstatus
* FUNCTION : check LCDC interrupt RAW status
* RETURN   : none
* NOTE     : none
******************************************************************************/
unsigned long lcd_hw_chk_int_rawstatus(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	/*  check Interuppt RAW Status */
	return (unsigned long)readl(LCDCMmioV + LCD_INTRAWSTATUS);
}


/* ------------------ LCDC interruput control function --------------------- */
/*****************************************************************************
* MODULE   : lcd_hw_int_enable
* FUNCTION : enable LCDC VSYNC interrupt
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_int_enable(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	/*  Interuppt Enable Set */
	writel(LCD_UNDERRUN_BIT, LCDCMmioV + LCD_INTENSET);
	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_HDMI_1080I)
		writel(LCD_LCDVS_BIT | LCD_FIELD_BIT, LCDCMmioV + LCD_INTENSET);
}


/*****************************************************************************
* MODULE   : lcd_hw_int_disable
* FUNCTION : disable LCDC interrupts
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_int_disable(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	/* Interrupt Enable Clear */
	writel(LCD_INT_ALL_BIT, LCDCMmioV + LCD_INTENCLR);
}


/*****************************************************************************
* MODULE   : lcd_hw_int_factor_clr
* FUNCTION : disable LCDC interrupts
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_hw_int_factor_clr(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	/* Interrupt Status Clear */
	writel(LCD_INT_ALL_BIT, LCDCMmioV + LCD_INTFFCLR);
}


/* ------------------ LCD initialize function ------------------------------ */
static inline void change_pinsel_wvga(void)
{
	/********************************/
	/* port / terminal switching    */
	/********************************/
	printk_dbg(_DEBUG_LCDHW,
	 "SMU_LCDLCLKDIV = 0x%08x\n", readl(SMU_LCDLCLKDIV));
	writel(0x00000009, SMU_LCDLCLKDIV); /* 229.376Mhz / 10 = 22.938Mhz */

#ifdef CONFIG_MACH_EMEV
	/* select the GPIO pin functions GIO_P18 - GIO_P23, GIO_P32 - GIO_P43 */
	printk_dbg(_DEBUG_LCDHW,
	 "CHG_PINSEL_G000 = 0x%08x\n", readl(CHG_PINSEL_G000));
	writel((readl(CHG_PINSEL_G000) & ~0x00F40000), CHG_PINSEL_G000);
	printk_dbg(_DEBUG_LCDHW,
	 "CHG_PINSEL_G032 = 0x%08x\n", readl(CHG_PINSEL_G032));
	writel((readl(CHG_PINSEL_G032) & ~0x00000FFF), CHG_PINSEL_G032);

	/* enable the 3.3 V LCD */
	printk_dbg(_DEBUG_LCDHW,
	 "CHG_LCD_ENABLE = 0x%08x\n", readl(CHG_LCD_ENABLE));
	writel(0x00000001, CHG_LCD_ENABLE);

	/* selects the 3.3 V LCD pin function in mode 0 */
	printk_dbg(_DEBUG_LCDHW,
	 "CHG_PINSEL_LCD3 = 0x%08x\n", readl(CHG_PINSEL_LCD3));
	writel(0x00000000, CHG_PINSEL_LCD3);

	/* enable input GIO_P18 - GIO_P23, GIO_P32 - GIO_P43 */
	printk_dbg(_DEBUG_LCDHW, "CHG_PULL2 = 0x%08x\n", readl(CHG_PULL2));
	printk_dbg(_DEBUG_LCDHW, "CHG_PULL4 = 0x%08x\n", readl(CHG_PULL4));
	printk_dbg(_DEBUG_LCDHW, "CHG_PULL5 = 0x%08x\n", readl(CHG_PULL5));
	writel((readl(CHG_PULL2) | 0x44444400), CHG_PULL2);
	writel((readl(CHG_PULL4) | 0x44444444), CHG_PULL4);
	writel((readl(CHG_PULL5) | 0x00004444), CHG_PULL5);
	printk_dbg(_DEBUG_LCDHW, "CHG_PULL2 = 0x%08x\n", readl(CHG_PULL2));
	printk_dbg(_DEBUG_LCDHW, "CHG_PULL4 = 0x%08x\n", readl(CHG_PULL4));
	printk_dbg(_DEBUG_LCDHW, "CHG_PULL5 = 0x%08x\n", readl(CHG_PULL5));
#endif
}


static inline void change_pinsel_hdmi(void)
{
	/********************************/
	/* port / terminal switching    */
	/********************************/
	printk_dbg(_DEBUG_LCDHW,
	 "SMU_LCDLCLKDIV = 0x%08x\n", readl(SMU_LCDLCLKDIV));
	/* 229.376Mhz / 10 = 22.938Mhz */
	writel(0x00000009, SMU_LCDLCLKDIV);

#ifdef CONFIG_MACH_EMEV
	printk_dbg(_DEBUG_LCDHW,
	 "SMU_USIB0SCLKDIV = 0x%08x\n", readl(SMU_USIB0SCLKDIV));
	/* OSC0 11.2896Mhz / 256 = 44.1Khz */
	writel(0x02FF000F, SMU_USIB0SCLKDIV);

	/* IIC0 IIC1 */
	writel((readl(CHG_PINSEL_G032) & 0xFFFFF0FF), CHG_PINSEL_G032);

	/* LCD3 PXCLKB CLK_I HS VS DE */
	writel((readl(CHG_PINSEL_G000) & 0xFF0FFFFF), CHG_PINSEL_G000);

	/* USI3 */
	writel((readl(CHG_PINSEL_G096) & 0xFF87FFFF), CHG_PINSEL_G096);

	/* CHG mask release  GIO_006 */
	writel(((readl(CHG_PULL13) & 0xFFF0FFFF) | 0x00040000), CHG_PULL13);
	/* CHG mask release  GIO_020, schmitt */
	writel(((readl(CHG_PULL0) & 0xFFFFF0FF) | 0x00000C00),  CHG_PULL0);

	/* select YUV */
	writel(0x00000401, CHG_PINSEL_LCD3);
	/* select USI3 */
	writel((readl(CHG_PINSEL_USI) & 0xFFFFFFCF), CHG_PINSEL_USI);

	/* LCD pin drive 12mA */
	writel(0x03FFF000, CHG_DRIVE0);
#endif
}


static inline void unreset_lcdc(void)
{
	/********************************/
	/* clock supply                 */
	/********************************/
	emxx_open_clockgate(EMXX_CLK_LCD_C | EMXX_CLK_LCD_L | EMXX_CLK_LCD_P |
	 EMXX_CLK_LCD);

	/********************************/
	/* clock auto control -> OFF    */
	/********************************/
	emxx_clkctrl_off(EMXX_CLKCTRL_LCD);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCDC);
	emxx_clkctrl_off(EMXX_CLKCTRL_LCDPCLK);

	/********************************/
	/* LCDC unreset                 */
	/********************************/
	emxx_unreset_device(EMXX_RST_LCD_A | EMXX_RST_LCD);

	/********************************/
	/* clock auto control -> ON     */
	/********************************/
	emxx_clkctrl_on(EMXX_CLKCTRL_LCD);
	emxx_clkctrl_on(EMXX_CLKCTRL_LCDC);
	emxx_clkctrl_on(EMXX_CLKCTRL_LCDPCLK);
}


static inline void unreset_usi3(void)
{
	/********************************/
	/* clock supply                 */
	/********************************/
	emxx_open_clockgate(EMXX_CLK_USIB_S3_H | EMXX_CLK_USIB_S3_S |
	 EMXX_CLK_USIB_S3_P);

	/********************************/
	/* clock auto control -> OFF    */
	/********************************/
	emxx_clkctrl_off(EMXX_CLKCTRL_USIBS3);
	emxx_clkctrl_off(EMXX_CLKCTRL_USIBS3PCLK);
	emxx_clkctrl_off(EMXX_CLKCTRL_USIBS3SCLK);

	/********************************/
	/* LCDC unreset                 */
	/********************************/
	emxx_unreset_device(EMXX_RST_USIB_S3_A | EMXX_RST_USIB_S3_S);

	/********************************/
	/* clock auto control -> ON     */
	/********************************/
	emxx_clkctrl_on(EMXX_CLKCTRL_USIBS3);
	emxx_clkctrl_on(EMXX_CLKCTRL_USIBS3PCLK);
	emxx_clkctrl_on(EMXX_CLKCTRL_USIBS3SCLK);
}


/*****************************************************************************
* MODULE   : lcd_hw_init
* FUNCTION : LCD(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void lcd_hw_init(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	switch (lcdc_output_mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		/* WVGA LCD */
		change_pinsel_wvga();
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		/* HDMI */
		change_pinsel_hdmi();
		break;
	}

	unreset_lcdc();

	if ((lcdc_output_mode == EMXX_FB_OUTPUT_MODE_HDMI_1080I) ||
	    (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_HDMI_720P))
		unreset_usi3();
}


/******************************************************************************
* MODULE   : lcd_controller_hw_init
* FUNCTION : LCD(H/W) initialized
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void lcd_controller_hw_init(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	lcd_hw_int_factor_clr();
	lcd_hw_int_disable();
	lcd_hw_int_enable();
	switch (lcdc_output_mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		/* Control for LCDC  */
		writel(LCD_CONTROL_INIT, LCDCMmioV + LCD_CONTROL);
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
		/* Control for LCDC  */
		writel(LCD_LCLK_33V_PCLK |
		       LCD_OUT_SEL_YUV |
		       LCD_PI_SEL_INTERLACE |
		       LCD_OFORMAT_RGB888 |
		       LCD_CLKPOL_RISING |
		       LCD_HPOL_NEGATIVE |
		       LCD_VPOL_NEGATIVE |
		       LCD_ENPOL_HIGH,
		       LCDCMmioV + LCD_CONTROL);
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		/* Control for LCDC  */
		writel(LCD_LCLK_33V_PCLK |
		       LCD_OUT_SEL_YUV |
		       LCD_PI_SEL_PROGRESSIVE |
		       LCD_OFORMAT_RGB888 |
		       LCD_CLKPOL_RISING |
		       LCD_HPOL_NEGATIVE |
		       LCD_VPOL_NEGATIVE |
		       LCD_ENPOL_HIGH,
		       LCDCMmioV + LCD_CONTROL);
		break;
	}

	lcd_field = FIELD_NONE;

	/* QoS for LCDC      */
	writel(LCD_QOS_INIT,     LCDCMmioV + LCD_QOS);
	/* Data Request Timing for LCDC */
	writel(LCD_DATAREQ_INIT, LCDCMmioV + LCD_DATAREQ);
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	/* AREA Address for Frame Buffer       */
	writel(LCD_AREAADR_ODD_INIT, LCDCMmioV + LCD_AREAADR_ODD);
	writel(LCD_AREAADR_EVEN_INIT, LCDCMmioV + LCD_AREAADR_EVEN);
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
	/* Input Format      */
	writel(LCD_IFORMAT_INIT, LCDCMmioV + LCD_IFORMAT);
	/* Background color  */
	writel(LCD_BACKCOL_INIT, LCDCMmioV + LCD_BACKCOLOR);
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	if (direct_path) {
		/* Access Bus Select(Pwoer Management) */
		writel(LCD_BUSSEL_DIRECT, LCDCMmioV + LCD_BUSSEL);
	} else {
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
		/* Access Bus Select(Pwoer Management) */
		writel(LCD_BUSSEL_INIT,   LCDCMmioV + LCD_BUSSEL);
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	}
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

	writel(LCD_COEF_Y0_INIT, LCDCMmioV + LCD_COEF_Y0);
	writel(LCD_COEF_Y1_INIT, LCDCMmioV + LCD_COEF_Y1);
	writel(LCD_COEF_Y2_INIT, LCDCMmioV + LCD_COEF_Y2);
	writel(LCD_COEF_Y3_INIT, LCDCMmioV + LCD_COEF_Y3);
	writel(LCD_COEF_U0_INIT, LCDCMmioV + LCD_COEF_U0);
	writel(LCD_COEF_U1_INIT, LCDCMmioV + LCD_COEF_U1);
	writel(LCD_COEF_U2_INIT, LCDCMmioV + LCD_COEF_U2);
	writel(LCD_COEF_U3_INIT, LCDCMmioV + LCD_COEF_U3);
	writel(LCD_COEF_V0_INIT, LCDCMmioV + LCD_COEF_V0);
	writel(LCD_COEF_V1_INIT, LCDCMmioV + LCD_COEF_V1);
	writel(LCD_COEF_V2_INIT, LCDCMmioV + LCD_COEF_V2);
	writel(LCD_COEF_V3_INIT, LCDCMmioV + LCD_COEF_V3);
	writel(LCD_BYTELANE_INIT, LCDCMmioV + LCD_BYTELANE);

	init_is_first = 0;
}


/******************************************************************************
* MODULE   : lcd_controller_hw_init_lcdsize
* FUNCTION : LCD(H/W) size initialize
* RETURN   : none
* NOTE     : none
******************************************************************************/
static inline void lcd_controller_hw_init_lcdsize(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	/* Horizontal Synchronizing Signal Total Width  */
	writel(harea + hfrontp + hpulse + hbackp, LCDCMmioV + LCD_HTOTAL);
	/* Horizontal Display Area Width                */
	writel(harea,				  LCDCMmioV + LCD_HAREA);
	/* Horizontal Synchronizing Signal Edge Point 1 */
	writel(hfrontp,				  LCDCMmioV + LCD_HEDGE1);
	/* Horizontal Synchronizing Signal Edge Point 2 */
	writel(hfrontp + hpulse,		  LCDCMmioV + LCD_HEDGE2);
	/* Vertical Synchronizing Signal Total Width    */
	switch (lcdc_output_mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		writel(varea + vfrontp + vpulse + vbackp,
		 LCDCMmioV + LCD_VTOTAL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			writel(0, LCDCMmioV + LCD_VSYNC_CONT);
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		writel(varea + vfrontp + vpulse + vbackp,
		 LCDCMmioV + LCD_VTOTAL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1) {
#endif
			writel(LCD_VOFFSET_EN_BIT, LCDCMmioV + LCD_VSYNC_CONT);
			writel(hfrontp,		   LCDCMmioV + LCD_VOFFSET_ODD);
#ifdef CONFIG_MACH_EMEV
		}
#endif
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
		writel(varea / 2 + vfrontp + vpulse + vbackp,
		 LCDCMmioV + LCD_VTOTAL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1) {
#endif
			writel((LCD_VOFFSET_EN_BIT | LCD_VTOTAL_SEL_BIT |
			 LCD_VEDGE_SEL_BIT),	 LCDCMmioV + LCD_VSYNC_CONT);
			writel(hfrontp,		 LCDCMmioV + LCD_VOFFSET_ODD);
			writel((harea + hfrontp + hpulse + hbackp) / 2 +
			 hfrontp,		 LCDCMmioV + LCD_VOFFSET_EVEN);
			writel(varea / 2 + vfrontp + vpulse + vbackp + 1,
						 LCDCMmioV + LCD_VAREA_EVEN);
			writel(vfrontp,		 LCDCMmioV + LCD_VEDGE1_EVEN);
			writel(vfrontp + vpulse, LCDCMmioV + LCD_VEDGE2_EVEN);
#ifdef CONFIG_MACH_EMEV
		}
#endif
		break;
	}
	/* Vertical Display Area Width                  */
	writel(varea,				  LCDCMmioV + LCD_VAREA);
	/* Vertical Synchronizing Signal Edge Point 1   */
	writel(vfrontp,				  LCDCMmioV + LCD_VEDGE1);
	/* Vertical Synchronizing Signal Edge Point 2   */
	writel(vfrontp + vpulse,		  LCDCMmioV + LCD_VEDGE2);
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	/* Horizontal OFFSET Size                       */
	writel(harea * BITS_PER_PIXEL / 8, LCDCMmioV + LCD_HOFFSET);
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
}


/* ------------------ LCD module initialize function ----------------------- */
/*****************************************************************************
* MODULE   : lcd_module_hw_unreset
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_module_hw_unreset(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		gpio_direction_output(GPIO_LCD_RST, 0x1);
		mdelay(1);
	}
}


/*****************************************************************************
* MODULE   : lcd_module_hw_reset
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_module_hw_reset(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");
	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD)
		gpio_direction_output(GPIO_LCD_RST, 0x0);
}


/*****************************************************************************
* MODULE   : lcd_module_hw_power_on
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
int lcd_module_hw_power_on(void)
{
	int iRet = 0, i;
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		for (i = 0; i < LCDM_SPI_CMD_POWERON; i++) {
			iRet = lcd_module_hw_spiw(lcdm_spi_cmd_poweron[i][0],
			 lcdm_spi_cmd_poweron[i][1]);
			if (iRet < 0) {
				printk_err("LCD Module power on failed!\n");
				return iRet;
			}
		}
		udelay(20);
	}
	return iRet;
}


/*****************************************************************************
* MODULE   : lcd_module_hw_power_off
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
int lcd_module_hw_power_off(void)
{
	int iRet = 0, i;
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		for (i = 0; i < LCDM_SPI_CMD_POWEROFF; i++) {
			iRet = lcd_module_hw_spiw(lcdm_spi_cmd_poweroff[i][0],
			 lcdm_spi_cmd_poweroff[i][1]);
			if (iRet < 0) {
				printk_err("LCD Module power off failed!\n");
				return iRet;
			}
			udelay(20);
		}
	}
	return iRet;
}


/*****************************************************************************
* MODULE   : lcd_module_hw_standby
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
int lcd_module_hw_standby(void)
{
	int iRet = 0, i;
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		for (i = 0; i < LCDM_SPI_CMD_STANDBY; i++) {
			iRet = lcd_module_hw_spiw(lcdm_spi_cmd_standby[i][0],
			 lcdm_spi_cmd_standby[i][1]);
			if (iRet < 0) {
				printk_err("LCD Module initialize failed!\n");
				return iRet;
			}
		}
		mdelay(40);
	}
	return iRet;
}


/*****************************************************************************
* MODULE   : lcd_module_hw_wakeup
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
int lcd_module_hw_wakeup(void)
{
	int iRet = 0, i;
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) {
		for (i = 0; i < LCDM_SPI_CMD_WAKEUP; i++) {
			iRet = lcd_module_hw_spiw(lcdm_spi_cmd_wakeup[i][0],
			 lcdm_spi_cmd_wakeup[i][1]);
			if (iRet < 0) {
				printk_err("LCD Module initialize failed!\n");
				return iRet;
			}
		}
	}
	return iRet;
}


/* ------------------ spi control function --------------------------------- */
/*****************************************************************************
* MODULE   : lcd_module_hw_spiw
* FUNCTION :
* RETURN   :    0 : success
*          : other: failed
* NOTE     : none
******************************************************************************/
static int lcd_module_hw_spiw(unsigned char cmd, unsigned char param)
{
	char *cRet;
	int   iRet;
	struct spi_config_t spi_conf;
	unsigned int tx_data =
		(0 << 24) | (cmd << 16) | (1 << 8) | (param << 0);

	spi_conf.dev    = SPI_DEV_SP0;
	spi_conf.nbw    = SPI_NB_32BIT;
	spi_conf.nbr    = 0;
	spi_conf.cs_sel = SPI_CS_SEL_CS1;
	spi_conf.m_s    = SPI_M_S_MASTER;
	spi_conf.dma    = SPI_DMA_OFF;
	spi_conf.pol    = SPI_POL_SP0_CS1;
	spi_conf.sclk   = SPI_SCLK_3MHZ;
	spi_conf.tiecs  = SPI_TIECS_NORMAL;

	iRet = spi_write(&spi_conf, (char *)&tx_data, 0, 4, 0);
	if (iRet < 0) {
		chk_errno(iRet, &cRet);
		printk_err("spi_write(%s) error\n", cRet);
		return iRet;
	}
	printk_dbg((_DEBUG_LCDHW & 0x01), "->cmd(0x%02x) ->param(0x%02x)\n",
	 cmd, param);
	return 0;
}



/* ------------------ IMC initialize function ------------------------------ */
/*****************************************************************************
* MODULE   : imc_hw_init
* FUNCTION : IMC(H/W) initialized
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static int imc_hw_init(void)
{
	unsigned long sync, format;
	int iRet;

	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	memset(&imc_info, 0, sizeof(struct emxx_imc_info));
	imc_info.device  = DEV_LCD;
	imc_info.timeout = 0;
	if (emxx_request_imc(IMC_CH0, &imc_info)) {
		printk_err("could not acquire IMC channel\n");
		return -EBUSY;
	}

	if (!imc_param) {
		imc_param = kmalloc(sizeof(struct emxx_imc_param), GFP_KERNEL);
		if (!imc_param) {
			printk_err("could not allocate memory "
				"for \"IMC info\"\n");
			return -ENOMEM;
		}
		memset(imc_param, 0, sizeof(struct emxx_imc_param));

		imc_preset  = &imc_param->imc_preset;
		imc_vsync   = &imc_param->imc_vsync;
		imc_reserve = &imc_param->imc_reserve;

		imc_vsync->cpubufsel    =
		 &imc_param->param_vsync.imc_cpubufsel;
		imc_vsync->wb           =
		 &imc_param->param_vsync.param_wb;
		imc_vsync->mirror       =
		 &imc_param->param_vsync.imc_mirror;
		imc_vsync->alphasel     =
		 &imc_param->param_vsync.param_alphasel;
		imc_vsync->l0_scanmode  =
		 &imc_param->param_vsync.imc_l0_scanmode;
		imc_vsync->l1a_scanmode =
		 &imc_param->param_vsync.imc_l1a_scanmode;
		imc_vsync->l1b_scanmode =
		 &imc_param->param_vsync.imc_l1b_scanmode;
		imc_vsync->l1c_scanmode =
		 &imc_param->param_vsync.imc_l1c_scanmode;
		imc_vsync->l2a_scanmode =
		 &imc_param->param_vsync.imc_l2a_scanmode;
		imc_vsync->l2b_scanmode =
		 &imc_param->param_vsync.imc_l2b_scanmode;
		imc_vsync->bg_scanmode  =
		 &imc_param->param_vsync.imc_bg_scanmode;

		/* set callback function */
		imc_param->callback_refresh = lcd_callback_imc_refresh;
		imc_param->callback_wb      = lcd_callback_imc_wb;
		printk_dbg((_DEBUG_LCDHW & 0x04), "kmalloc()\n");
	}

	/**********************
	 * set_callback
	 **********************/
	emxx_imc_set_callback(imc_info.id, imc_param->callback_refresh,
	 imc_param->callback_wb);
	printk_dbg((_DEBUG_LCDHW & 0x04), "set_callback()\n");

	/**********************
	 * set_preset
	 **********************/
	sync   = IMC_START_MODE_LCD_SYNC;	/* VSYNC syncro mode */
	format = IMC_FORMAT_RGB888;

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
		imc_param->imc_control  = (format | sync);
#ifdef CONFIG_MACH_EMEV
	else
		imc_param->imc_control  = (format | sync | IMC_CLKCNT_ALL);
#endif
	imc_preset->imc_control = &imc_param->imc_control;
	imc_param->imc_datareq  = IMC_DATAREQ_INIT;
	imc_preset->imc_datareq = &imc_param->imc_datareq;
	imc_preset->gamma       =
		imc_hw_init_preset_gamma(&imc_param->param_gamma);
	imc_preset->yuv         =
		imc_hw_init_preset_yuv(&imc_param->param_yuv);
	imc_preset->burst       =
		imc_hw_init_preset_burst(&imc_param->param_burst);

	iRet = emxx_imc_set_preset(imc_info.id, imc_preset);
	if (iRet)
		return -1;

	printk_dbg((_DEBUG_LCDHW & 0x04), "set_preset()\n");

	/**********************
	 * set_update_vsync
	 **********************/
	ImcNxtVsync_Param.imc_cpubufsel = IMC_CPUBUFSEL_INIT;
	ImcNxtVsync_Param.imc_mirror    = uiInverseFlag_tmp = IMC_MIRROR_INIT;
	ImcNxtVsync_Param.imc_l0_scanmode  = IMC_L0_SCANMODE_INIT;
	ImcNxtVsync_Param.imc_l1a_scanmode = IMC_L1A_SCANMODE_INIT;
	ImcNxtVsync_Param.imc_l1b_scanmode = IMC_L1B_SCANMODE_INIT;
	ImcNxtVsync_Param.imc_l1c_scanmode = IMC_L1C_SCANMODE_INIT;
	ImcNxtVsync_Param.imc_l2a_scanmode = IMC_L2A_SCANMODE_INIT;
	ImcNxtVsync_Param.imc_l2b_scanmode = IMC_L2B_SCANMODE_INIT;
	ImcNxtVsync_Param.imc_bg_scanmode  = IMC_BG_SCANMODE_INIT;

	ImcNxtVsync.cpubufsel    = &ImcNxtVsync_Param.imc_cpubufsel;
	ImcNxtVsync.wb           =
	 imc_hw_init_vsync_wb(&ImcNxtVsync_Param.param_wb);
	ImcNxtVsync.mirror       = &ImcNxtVsync_Param.imc_mirror;
	ImcNxtVsync.alphasel     =
	 imc_hw_init_vsync_alphasel(&ImcNxtVsync_Param.param_alphasel);
	ImcNxtVsync.l0_scanmode  = &ImcNxtVsync_Param.imc_l0_scanmode;
	ImcNxtVsync.l1a_scanmode = &ImcNxtVsync_Param.imc_l1a_scanmode;
	ImcNxtVsync.l1b_scanmode = &ImcNxtVsync_Param.imc_l1b_scanmode;
	ImcNxtVsync.l1c_scanmode = &ImcNxtVsync_Param.imc_l1c_scanmode;
	ImcNxtVsync.l2a_scanmode = &ImcNxtVsync_Param.imc_l2a_scanmode;
	ImcNxtVsync.l2b_scanmode = &ImcNxtVsync_Param.imc_l2b_scanmode;
	ImcNxtVsync.bg_scanmode  = &ImcNxtVsync_Param.imc_bg_scanmode;

	iRet = imc_hw_set_update_vsync(&ImcNxtVsync);
	if (iRet)
		return -1;

	printk_dbg((_DEBUG_LCDHW & 0x04), "update_vsync()\n");

	/**********************
	 * set_update_reserve
	 **********************/
	imc_reserve->l0  = imc_hw_init_reserve_l0(&imc_param->param_l0);
	imc_reserve->l1a = imc_hw_init_reserve_l1a(&imc_param->param_l1a);
	imc_reserve->l1b = imc_hw_init_reserve_l1b(&imc_param->param_l1b);
	imc_reserve->l1c = imc_hw_init_reserve_l1c(&imc_param->param_l1c);
	imc_reserve->l2a = imc_hw_init_reserve_l2a(&imc_param->param_l2a);
	imc_reserve->l2b = imc_hw_init_reserve_l2b(&imc_param->param_l2b);
	imc_reserve->bg  = imc_hw_init_reserve_bg(&imc_param->param_bg);

	iRet = emxx_imc_set_update_reserve(imc_info.id, imc_reserve,
	 imc_param->callback_refresh);
	if (iRet)
		return -1;

	printk_dbg((_DEBUG_LCDHW & 0x04), "update_reserve()\n");

	return 0;
}


#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/******************************************************************************
* MODULE   : imc_hw_restore_reg
* FUNCTION : restore IMC(H/W) register
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int imc_hw_restore_reg(void)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!imc_param) {
		printk_err("not allocate memory for \"IMC info\"\n");
		return -ENOMEM;
	}

	/**********************
	 * set_preset
	 **********************/
	if (emxx_imc_set_preset(imc_info.id, imc_preset)) {
		printk_err("set_preset() failed!\n");
		return -1;
	}
	printk_dbg((_DEBUG_LCDHW & 0x04), "set_preset()\n");

	/**********************
	 * set_update_vsync
	 **********************/
	if (imc_hw_set_update_vsync(&ImcNxtVsync)) {
		printk_err("set_update_vsync() failed!\n");
		return -1;
	}
	printk_dbg((_DEBUG_LCDHW & 0x04), "set_update_vsync()\n");

	IMC_reset_flg = 0;
	return 0;
}
#endif /* CONFIG_PM || CONFIG_DPM */


/*****************************************************************************
* MODULE   : imc_hw_init_preset_gamma
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_preset_gamma(struct imc_gamma_param *param_gamma)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_gamma)
		return NULL;
	memset(param_gamma, 0, sizeof(struct imc_gamma_param));

	param_gamma->en   = IMC_GAMMA_EN_INIT;
	param_gamma->adr  = IMC_GAMMA_ADR_INIT;
	param_gamma->data = NULL;

	return param_gamma;
}


/*****************************************************************************
* MODULE   : imc_hw_init_preset_yuv
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_preset_yuv(struct imc_yuv_param *param_yuv)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_yuv)
		return NULL;
	memset(param_yuv, 0, sizeof(struct imc_yuv_param));

	param_yuv->ygain   = IMC_YGAINOFFSET_INIT;
	param_yuv->ugain   = IMC_UGAINOFFSET_INIT;
	param_yuv->vgain   = IMC_VGAINOFFSET_INIT;
	param_yuv->yuv2rgb = IMC_YUV2RGB_INIT;
	if ((param_yuv->yuv2rgb & IMC_TRANSMODE_BIT) == IMC_TRANSMODE_CUSTOM1 ||
	    (param_yuv->yuv2rgb & IMC_TRANSMODE_BIT) == IMC_TRANSMODE_CUSTOM2) {
		param_yuv->coef_r[0] = IMC_COEF_R0_INIT;
		param_yuv->coef_r[1] = IMC_COEF_R1_INIT;
		param_yuv->coef_r[2] = IMC_COEF_R2_INIT;
		param_yuv->coef_r[3] = IMC_COEF_R3_INIT;
		param_yuv->coef_g[0] = IMC_COEF_G0_INIT;
		param_yuv->coef_g[1] = IMC_COEF_G1_INIT;
		param_yuv->coef_g[2] = IMC_COEF_G2_INIT;
		param_yuv->coef_g[3] = IMC_COEF_G3_INIT;
		param_yuv->coef_b[0] = IMC_COEF_B0_INIT;
		param_yuv->coef_b[1] = IMC_COEF_B1_INIT;
		param_yuv->coef_b[2] = IMC_COEF_B2_INIT;
		param_yuv->coef_b[3] = IMC_COEF_B3_INIT;
	}

	return param_yuv;
}


/*****************************************************************************
* MODULE   : imc_hw_init_preset_burst
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_preset_burst(struct imc_burst_param *param_burst)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_burst)
		return NULL;
	memset(param_burst, 0, sizeof(struct imc_burst_param));

	param_burst->burst_en  = IMC_BURST_EN_INIT;
	param_burst->threshold = IMC_THRESHOLD_INIT;

	return param_burst;
}


/*****************************************************************************
* MODULE   : imc_hw_init_vsync_wb
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_vsync_wb(struct imc_wb_param *param_wb)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_wb)
		return NULL;
	memset(param_wb, 0, sizeof(struct imc_wb_param));

	param_wb->areaadr_p = LCD_AREAADR_ODD_INIT;
	param_wb->areaadr_q = LCD_AREAADR_EVEN_INIT;
	param_wb->hoffset   = FRONT_WIDTH_LCD * BITS_PER_PIXEL / 8;
	param_wb->format    = IMC_WB_FORMAT_RGB888;
	param_wb->size      = (FRONT_WIDTH_LCD << IMC_WB_HSIZE_SFT) |
			      (FRONT_HEIGHT_LCD << IMC_WB_VSIZE_SFT);
	param_wb->bufsel    = IMC_WB_BUFSEL_INIT;
	param_wb->mposition = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	param_wb->msize     = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;
	param_wb->color     = IMC_BACKCOLOR_INIT;
	param_wb->bytelane  = IMC_WB_BYTELANE_INIT;
	param_wb->scanmode  = IMC_WB_SCANMODE_INIT;

	return param_wb;
}


/*****************************************************************************
* MODULE   : imc_hw_init_vsync_alphasel
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_vsync_alphasel(
 struct imc_alphasel_param *param_alphasel)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_alphasel)
		return NULL;
	memset(param_alphasel, 0, sizeof(struct imc_alphasel_param));

	param_alphasel->alphasel0 = IMC_ALPHASEL0_INIT;
	param_alphasel->alphasel1 = IMC_ALPHASEL1_INIT;

	return param_alphasel;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_l0
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_l0(struct l01_param *param_l0)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_l0)
		return NULL;
	memset(param_l0, 0, sizeof(struct l01_param));

	param_l0->control    = IMC_L0_CONTROL_INIT;
	param_l0->format     = IMC_L0_FORMAT_INIT;
	param_l0->bufsel     = IMC_L0_BUFSEL_INIT;
	param_l0->bytelane   = IMC_L0_BYTELANE_INIT;
	param_l0->keyenable  = IMC_L0_KEYENABLE_INIT;
	param_l0->keycolor   = IMC_L0_KEYCOLOR_INIT;
	param_l0->alpha      = IMC_L0_ALPHA_INIT;
	param_l0->resize     = IMC_L0_RESIZE_INIT;
	param_l0->mirror     = IMC_L0_MIRROR_INIT;
	param_l0->offset     = IMC_L0_OFFSET_INIT;
	param_l0->frameadr_p = IMC_L0_FRAMEADR_P_INIT;
	param_l0->frameadr_q = IMC_L0_FRAMEADR_Q_INIT;
	param_l0->position   = IMC_L0_POSITION_INIT;
	param_l0->size       = IMC_L0_SIZE_INIT;
	param_l0->mposition  = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	param_l0->msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_l0;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_l1a
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_l1a(struct l01_param *param_l1a)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_l1a)
		return NULL;
	memset(param_l1a, 0, sizeof(struct l01_param));

	param_l1a->format     = IMC_L01x_FORMAT_RGB565;
	param_l1a->offset     = FRONT_WIDTH_V_LCD * BYTES_PER_PIXEL;

	param_l1a->control    = IMC_L1A_CONTROL_INIT;
	param_l1a->bufsel     = IMC_L1A_BUFSEL_INIT;
	param_l1a->bytelane   = IMC_L1A_BYTELANE_INIT;
	param_l1a->keyenable  = IMC_L1A_KEYENABLE_INIT;
	param_l1a->keycolor   = IMC_L1A_KEYCOLOR_INIT;
	param_l1a->alpha      = IMC_L1A_ALPHA_INIT;
	param_l1a->resize     = IMC_L1A_RESIZE_INIT;
	param_l1a->mirror     = IMC_L1A_MIRROR_INIT;
	param_l1a->frameadr_p = IMC_L1A_FRAMEADR_P_INIT;
	param_l1a->frameadr_q = IMC_L1A_FRAMEADR_Q_INIT;
	param_l1a->position   = IMC_L1A_POSITION_INIT;
	param_l1a->size       = IMC_L1A_SIZE_INIT;
	param_l1a->mposition  = IMC_L1A_MPOSITION_INIT;
	param_l1a->msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_l1a;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_l1b
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_l1b(struct l01_param *param_l1b)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_l1b)
		return NULL;
	memset(param_l1b, 0, sizeof(struct l01_param));

	param_l1b->control    = IMC_L1B_CONTROL_INIT;
	param_l1b->format     = IMC_L1B_FORMAT_INIT;
	param_l1b->bufsel     = IMC_L1B_BUFSEL_INIT;
	param_l1b->bytelane   = IMC_L1B_BYTELANE_INIT;
	param_l1b->keyenable  = IMC_L1B_KEYENABLE_INIT;
	param_l1b->keycolor   = IMC_L1B_KEYCOLOR_INIT;
	param_l1b->alpha      = IMC_L1B_ALPHA_INIT;
	param_l1b->resize     = IMC_L1B_RESIZE_INIT;
	param_l1b->mirror     = IMC_L1B_MIRROR_INIT;
	param_l1b->offset     = IMC_L1B_OFFSET_INIT;
	param_l1b->frameadr_p = IMC_L1B_FRAMEADR_P_INIT;
	param_l1b->frameadr_q = IMC_L1B_FRAMEADR_Q_INIT;
	param_l1b->position   = IMC_L1B_POSITION_INIT;
	param_l1b->size       = IMC_L1B_SIZE_INIT;
	param_l1b->mposition  = IMC_L1B_MPOSITION_INIT;
	param_l1b->msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_l1b;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_l1c
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_l1c(struct l01_param *param_l1c)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_l1c)
		return NULL;
	memset(param_l1c, 0, sizeof(struct l01_param));

	param_l1c->control    = IMC_L1C_CONTROL_INIT;
	param_l1c->format     = IMC_L1C_FORMAT_INIT;
	param_l1c->bufsel     = IMC_L1C_BUFSEL_INIT;
	param_l1c->bytelane   = IMC_L1C_BYTELANE_INIT;
	param_l1c->keyenable  = IMC_L1C_KEYENABLE_INIT;
	param_l1c->keycolor   = IMC_L1C_KEYCOLOR_INIT;
	param_l1c->alpha      = IMC_L1C_ALPHA_INIT;
	param_l1c->resize     = IMC_L1C_RESIZE_INIT;
	param_l1c->mirror     = IMC_L1C_MIRROR_INIT;
	param_l1c->offset     = IMC_L1C_OFFSET_INIT;
	param_l1c->frameadr_p = IMC_L1C_FRAMEADR_P_INIT;
	param_l1c->frameadr_q = IMC_L1C_FRAMEADR_Q_INIT;
	param_l1c->position   = IMC_L1C_POSITION_INIT;
	param_l1c->size       = IMC_L1C_SIZE_INIT;
	param_l1c->mposition  = IMC_L1C_MPOSITION_INIT;
	param_l1c->msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_l1c;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_l2a
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_l2a(struct l2_param *param_l2a)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_l2a)
		return NULL;
	memset(param_l2a, 0, sizeof(struct l2_param));

	param_l2a->control     = IMC_L2A_CONTROL_INIT;
	param_l2a->format      = IMC_L2A_FORMAT_INIT;
	param_l2a->bufsel      = IMC_L2A_BUFSEL_INIT;
	param_l2a->bytelane    = IMC_L2A_BYTELANE_INIT;
	param_l2a->resize      = IMC_L2A_RESIZE_INIT;
	param_l2a->mirror      = IMC_L2A_MIRROR_INIT;
	param_l2a->offset      = IMC_L2A_OFFSET_INIT;
	param_l2a->frameadr_yp = IMC_L2A_FRAMEADR_YP_INIT;
	param_l2a->frameadr_up = IMC_L2A_FRAMEADR_UP_INIT;
	param_l2a->frameadr_vp = IMC_L2A_FRAMEADR_VP_INIT;
	param_l2a->frameadr_yq = IMC_L2A_FRAMEADR_YQ_INIT;
	param_l2a->frameadr_uq = IMC_L2A_FRAMEADR_UQ_INIT;
	param_l2a->frameadr_vq = IMC_L2A_FRAMEADR_VQ_INIT;
	param_l2a->position    = IMC_L2A_POSITION_INIT;
	param_l2a->size        = IMC_L2A_SIZE_INIT;
	param_l2a->mposition   = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	param_l2a->msize       = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_l2a;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_l2b
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_l2b(struct l2_param *param_l2b)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_l2b)
		return NULL;
	memset(param_l2b, 0, sizeof(struct l2_param));

	param_l2b->control     = IMC_L2B_CONTROL_INIT;
	param_l2b->format      = IMC_L2B_FORMAT_INIT;
	param_l2b->bufsel      = IMC_L2B_BUFSEL_INIT;
	param_l2b->bytelane    = IMC_L2B_BYTELANE_INIT;
	param_l2b->resize      = IMC_L2B_RESIZE_INIT;
	param_l2b->mirror      = IMC_L2B_MIRROR_INIT;
	param_l2b->offset      = IMC_L2B_OFFSET_INIT;
	param_l2b->frameadr_yp = IMC_L2B_FRAMEADR_YP_INIT;
	param_l2b->frameadr_up = IMC_L2B_FRAMEADR_UP_INIT;
	param_l2b->frameadr_vp = IMC_L2B_FRAMEADR_VP_INIT;
	param_l2b->frameadr_yq = IMC_L2B_FRAMEADR_YQ_INIT;
	param_l2b->frameadr_uq = IMC_L2B_FRAMEADR_UQ_INIT;
	param_l2b->frameadr_vq = IMC_L2B_FRAMEADR_VQ_INIT;
	param_l2b->position    = IMC_L2B_POSITION_INIT;
	param_l2b->size        = IMC_L2B_SIZE_INIT;
	param_l2b->mposition   = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	param_l2b->msize       = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_l2b;
}


/*****************************************************************************
* MODULE   : imc_hw_init_reserve_bg
* FUNCTION :
* RETURN   : char *
* NOTE     : none
******************************************************************************/
static void *imc_hw_init_reserve_bg(struct bg_param *param_bg)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "\n");

	if (!param_bg)
		return NULL;
	memset(param_bg, 0, sizeof(struct bg_param));

	param_bg->format     = IMC_BG_FORMAT_BACKCOLOR;
	param_bg->bufsel     = IMC_BG_BUFSEL_INIT;
	param_bg->bytelane   = IMC_BG_BYTELANE_INIT;
	param_bg->resize     = IMC_BG_RESIZE_INIT;
	param_bg->mirror     = IMC_BG_MIRROR_INIT;
	param_bg->offset     = IMC_BG_OFFSET_INIT;
	param_bg->frameadr_p = IMC_BG_FRAMEADR_P_INIT;
	param_bg->frameadr_q = IMC_BG_FRAMEADR_Q_INIT;
	param_bg->mposition  = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	param_bg->msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	return param_bg;
}


/*****************************************************************************
* MODULE   : imc_hw_set_update_vsync
* FUNCTION :
* RETURN   :  0: success
*          : -1: failed
* NOTE     : none
******************************************************************************/
int imc_hw_set_update_vsync(struct emxx_imc_update_vsync *vsync)
{
	printk_dbg((_DEBUG_LCDHW & 0x01), "in\n");

	if (emxx_imc_set_update_vsync(imc_info.id, vsync)) {
		printk_dbg((_DEBUG_LCDHW & 0x04),
		 "emxx_imc_set_update_vsync() failed\n");
		return -1;
	}

	printk_dbg((_DEBUG_LCDHW & 0x04),
	 "emxx_imc_set_update_vsync() success\n");

	*imc_vsync->cpubufsel    = *vsync->cpubufsel;
	*imc_vsync->mirror       = *vsync->mirror;
	*imc_vsync->l0_scanmode  = *vsync->l0_scanmode;
	*imc_vsync->l1a_scanmode = *vsync->l1a_scanmode;
	*imc_vsync->l1b_scanmode = *vsync->l1b_scanmode;
	*imc_vsync->l1c_scanmode = *vsync->l1c_scanmode;
	*imc_vsync->l2a_scanmode = *vsync->l2a_scanmode;
	*imc_vsync->l2b_scanmode = *vsync->l2b_scanmode;
	*imc_vsync->bg_scanmode  = *vsync->bg_scanmode;
	memcpy(imc_vsync->wb, vsync->wb, sizeof(struct imc_wb_param));
	memcpy(imc_vsync->alphasel, vsync->alphasel,
	 sizeof(struct imc_alphasel_param));

	printk_dbg((_DEBUG_LCDHW & 0x02), "out\n");
	return 0;
}


/*****************************************************************************
* MODULE   : imc_hw_set_update_reserve
* FUNCTION :
* RETURN   :  0: success
*          : -1: failed
* NOTE     : none
******************************************************************************/
int imc_hw_set_update_reserve(int type, int mixdsp, int update)
{
	struct l01_param tmp_fb_layer;
	int iRet;
	printk_dbg((_DEBUG_LCDHW & 0x01), "in\n");

	/**********************
	 * set_update_reserve
	 **********************/
	if (type == SET_LAYER_2D) {
		memcpy(&tmp_fb_layer, &fb_layer, sizeof(struct l01_param));
		if (mixdsp != MIX_DSP_ON)
			tmp_fb_layer.keyenable = IMC_Lx_KEYEN_DISABLE;

		/* set 2D(DispBuffer) layer */
		printk_dbg((_DEBUG_LCDHW & 0x04), "   set2D\n");
		imc_reserve->l1a = &tmp_fb_layer;
	} else if (type == SET_LAYER_V4L2) {
		/* set v4l2 layer */
		printk_dbg((_DEBUG_LCDHW & 0x04), "   setV4L2\n");
		imc_reserve->l1a = &fb_layer;
		imc_reserve->l2b = &v4l2_layer;
	} else if (type == CLR_LAYER_V4L2) {
		memcpy(&tmp_fb_layer, &fb_layer, sizeof(struct l01_param));
		if (mixdsp != MIX_DSP_ON)
			tmp_fb_layer.keyenable = IMC_Lx_KEYEN_DISABLE;

		/* clr v4l2 layer */
		printk_dbg((_DEBUG_LCDHW & 0x04), "   clrV4L2\n");
		imc_reserve->l1a = &tmp_fb_layer;
		imc_reserve->l2b =
			imc_hw_init_reserve_l2b(&imc_param->param_l2b);
	} else {
		printk_dbg((_DEBUG_LCDHW & 0x04), "   set----\n");
		return -1;
	}

	if (update ==  UPDATE_ON) {
		iRet = emxx_imc_set_update_reserve(imc_info.id, imc_reserve,
		 imc_param->callback_refresh);
		if (iRet)
			return -1;
		if (lcd_field == FIELD_NONE) {
			printk_dbg((_DEBUG_LCDHW & 0x10), "Set REFRESH\n");
			emxx_imc_set_refresh(imc_info.id);
		} else {
			if (v4l2_field == V4L2_TOP_BOTTOM) {
				if (lcd_field == FIELD_EVEN) {
					printk_dbg((_DEBUG_LCDHW & 0x10),
					 "Set REFRESH\n");
					emxx_imc_set_refresh(imc_info.id);
				} else {
					printk_dbg((_DEBUG_LCDHW & 0x10),
					 "Reserve REFRESH\n");
					refresh_reserved = 1;
				}
			} else { /* v4l2_field == V4L2_BOTTOM_TOP */
				if (lcd_field == FIELD_ODD) {
					printk_dbg((_DEBUG_LCDHW & 0x10),
					 "Set REFRESH\n");
					emxx_imc_set_refresh(imc_info.id);
				} else {
					printk_dbg((_DEBUG_LCDHW & 0x10),
					 "Reserve REFRESH\n");
					refresh_reserved = 1;
				}
			}
		}
	}
	printk_dbg((_DEBUG_LCDHW & 0x02), "out\n");

	return 0;
}


