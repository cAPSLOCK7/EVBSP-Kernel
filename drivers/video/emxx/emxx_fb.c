/*
 * File Name       : drivers/video/emxx/emxx_fb.c
 * Function        : FraemBuffer Driver
 * Release Version : Ver 1.22
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

/*
 *
 *  framebuffer driver for EMXX -- based on vfb.c
 *
 */

/*
 *  linux/drivers/video/vfb.c -- Virtual frame buffer device
 *
 *      Copyright (C) 2002 James Simmons
 *
 *	Copyright (C) 1997 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */


/********************************************************
 *  Include Files                                       *
 *******************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <linux/sched.h>

#include <linux/platform_device.h>
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <mach/pm.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM || CONFIG_DPM */

#include <linux/uaccess.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/emxx_mem.h>
#include <mach/fbcommon.h>
#ifdef CONFIG_EMXX_IMC
#include "../../imc/emxx_imc.h"
#endif
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
#include "../console/fbcon.h"
#endif	/* CONFIG_FRAMEBUFFER_CONSOLE */

#include "emxx_common.h"
#include "emxx_lcd_common.h"
#include "emxx_fb.h"
#include "emxx_lcd.h"
#include "emxx_lcdhw.h"
#include "emxx_fb_blit.h"
#ifdef CONFIG_EMXX_NTS
#include "../../nts/emxx_nts.h"
#endif


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define _DEBUG_FB 0x00 /* 00008421(bit) */
		       /* 0x01: debug function in
			* 0x02: debug function out
			* 0x04: debug buffer
			* 0x08: debug frame buffer status
			* 0x10: debug update timer
			*/

#define TIMEOUT_MSLEEP	0


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define DEV_NAME "emxx_fb"


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
		printk(KERN_INFO DEV_NAME ": " fmt, ## arg); \
	} while (0)

#if _DEBUG_FB
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
 *  Definitions                                         *
 *******************************************************/
/* init function */
#define FB_INIT_ERROR__SMEM_NOT_RESERVE		-1
#define FB_INIT_ERROR__SMEM_NOT_REMAP		-2
#define FB_INIT_ERROR__FB_NOT_ALLOC		-3
#define FB_INIT_ERROR__FB_CMAP_NOT_ALLOC	-4
#define FB_INIT_ERROR__FB_NOT_REGIST		-5
#define FB_INIT_ERROR__LCD_NOT_INITIALIZE	-6

/* frame buffer page */
#define FB_NO_A				0
#define FB_NO_B				1

/* frame buffer status */
#define IS_NODATA			0
#define NOW_WRITING_TO_FB		1
#define ALREADY_DISPLAY			2
#define ENABLE_WRITE			3

/* timer */
#define TIMER_OFF			0
#define TIMER_ON			1
#define TIMEOUT_TIME			60 /* (ms) */
#define FB_TIMER_STOP			0
#define FB_TIMER_START			1

/* CHK_SCRN timer */
#define RECHK_TIME			10  /* (ms) */

/* timer default setting */
#define DEFALT_UPDATE_TIMER_LCD		TIMER_OFF

/* mmap */
#define EMXX_FB_MMAP_MAX_SIZE	FB_FRAME_BUFFER_SIZE


/* Number of FB devices */
#define EMXX_FB_DEVICES			1


/********************************************************
 *  Structure                                           *
 *******************************************************/
/* image infomation from 2D API */
struct fb_other_image_info {
	int  iMaskColFlg;
	int  iMaskCol;
	int  iAlpha;
	int  iInvFlg;
	int  iPage;
	int  iUpDate;
};
#define FB_OTHER_IMAGE_INFO struct fb_other_image_info

/* image infomation from 2D API */
struct _emxx_fb_par {
	struct fb_info          *fb_inf;
	atomic_t                 use_count;
	struct semaphore         sem_fb_image_info;	/* semafore for
							   fb_image_info */
	struct semaphore         sem_fb_other_info_A;	/* semafore for
							   fb_other_info_A */
	struct semaphore         sem_fb_other_info_B;	/* semafore for
							   fb_other_info_B */
	struct semaphore         sem_outputmode;/* semafore for output_mode */
	struct semaphore         sem_copybit;	/* semafore for copybit */
	spinlock_t               fb_lock;	/* spin lock fb timer flag */
#if (TIMEOUT_MSLEEP == 0)
	wait_queue_head_t        wait_fb_chk_scrn;
#endif
	struct workqueue_struct *emxx_fb_workqueue;
	struct work_struct       wk_timeout_update_bottom_half;
	u32                      pseudo_palette[256];
	int                      devflag; /* 0 : /dev/fb0   1 : /dev/fb1  */
#ifdef CONFIG_FB_EMXX_PANDISP
	int                      old_offset;
#endif	/* CONFIG_FB_EMXX_PANDISP */
	enum EMXX_FB_OUTPUT_MODE output_mode;
};
#define emxx_fb_par struct _emxx_fb_par

/* screen infomation default data */
static struct fb_var_screeninfo emxx_fb_default = {
	.xres           = FRONT_WIDTH_LCD,
	.yres           = FRONT_HEIGHT_LCD,
	.xres_virtual   = FRONT_WIDTH_V_LCD,
#ifdef CONFIG_FB_EMXX_PANDISP
	.yres_virtual   = (FRONT_HEIGHT_V_LCD * 2),
#else	/* CONFIG_FB_EMXX_PANDISP */
	.yres_virtual   = FRONT_HEIGHT_V_LCD,
#endif	/* CONFIG_FB_EMXX_PANDISP */
	.xoffset        = 0,
	.yoffset        = 0,
	.grayscale      = 0,
#ifdef CONFIG_FB_EMXX_ARGB8888
	.bits_per_pixel = 32,
 #ifdef CONFIG_FB_EMXX_ABGR8888
	.red            = {0, 8, 0},
	.green          = {8, 8, 0},
	.blue           = {16, 8, 0},
	.transp         = {24, 8, 0},
 #else
	.red            = {16, 8, 0},
	.green          = {8, 8, 0},
	.blue           = {0, 8, 0},
	.transp         = {24, 8, 0},
 #endif
#elif defined(CONFIG_FB_EMXX_BGR888)
	.bits_per_pixel = 24,
	.red            = {0, 8, 0},
	.green          = {8, 8, 0},
	.blue           = {16, 8, 0},
	.transp         = {0, 0, 0},
#else
	.bits_per_pixel = 16,
	.red            = {11, 5, 0},
	.green          = {5, 6, 0},
	.blue           = {0, 5, 0},
	.transp         = {0, 0, 0},
#endif
	.activate       = 0,
	.height         = -1,
	.width          = -1,
	.pixclock       = 0,
	.left_margin    = HFRONTP_LCD,
	.right_margin   = HBACKP_LCD,
	.upper_margin   = VFRONTP_LCD,
	.lower_margin   = VBACKP_LCD,
	.hsync_len      = HPULSE_LCD,
	.vsync_len      = VPULSE_LCD,
	.sync           = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode          = FB_VMODE_NONINTERLACED,
};


/* screen infomation fix data */
static struct fb_fix_screeninfo emxx_fb_fix = {
	.id        = "EMXX FB",
	.type      = FB_TYPE_PACKED_PIXELS,
	.visual    = FB_VISUAL_TRUECOLOR,
#ifdef CONFIG_FB_EMXX_PANDISP
	.xpanstep  = 0,
	.ypanstep  = 1,
	.ywrapstep = 0,
#else
	.xpanstep  = 1,
	.ypanstep  = 1,
	.ywrapstep = 1,
#endif
	.accel     = FB_ACCEL_NONE,
};

struct emxx_fb_struct {
	int                   is_first;
	/* image infomation to LCD */
	FB_IMAGE_INFO         fb_image_info;
	/* frame buffer A */
	FB_OTHER_IMAGE_INFO   fb_other_info_A;
	/* frame buffer B */
	FB_OTHER_IMAGE_INFO   fb_other_info_B;

	/* Disp Buffer (2D) */
	ulong   DispBufA;
	ulong   DispBufB;
	ulong   DispBufLength;
	char   *DispBufBV;
	char   *DispBufAV;

	/* Timer */
	struct timer_list     emxx_fb_timer;
	int                   iTimeoutTime;
	emxx_fb_par          *emxx_fb_dev;
	/* flags */
	int                   iFBStatus_A;
	int                   iFBStatus_B;
	int                   iNowDispPage;   /* now on display page */
	int                   iNextDispPage;  /* next display page   */
	int                   iMixModeFlg;
	int                   iUpdateTimerFlag;
	int                   iTimerStatusFlag;
	int                   iDpmSuspendFlag;
};

struct emxx_fb_drvdata {
	struct fb_info *info[2];
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct platform_device *dev;
	struct early_suspend    early_suspend;
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM || CONFIG_DPM */
};

/********************************************************
 *  Variables                                           *
 *******************************************************/
static struct emxx_fb_struct    fb_data[EMXX_FB_DEVICES];
static struct emxx_fb_drvdata   drvdata;
/* Smem */
static  ulong             Smem       = SMEM_START;
static  ulong             SmemLength = SMEM_LENGTH;
	char             *SmemV;
static  ulong             DispBufAllLength = EMXX_FB_MMAP_MAX_SIZE;

/********************************************************
 *  Prototype declarations of local function            *
 *******************************************************/
static int emxx_fb_ioc_chk_scrn(struct fb_info *info, unsigned long arg);
static int fb_chk_scrn(struct scrn_mode *FBScrnMode, struct fb_info *info);
static int fb_update_scrn(struct scrn_modes *FBSrcnModes, struct fb_info *info,
 unsigned int cmd);
static int emxx_fb_ioc_get_modes(struct fb_info *info, unsigned long arg);
static int emxx_fb_ioc_set_modes(struct fb_info *info, unsigned int cmd,
 unsigned long arg);
static int emxx_fb_ioc_update_scrn(struct fb_info *info, unsigned int cmd,
 unsigned long arg);
static int emxx_fb_ioc_get_output(struct fb_info *info, unsigned long arg);
static int emxx_fb_ioc_set_output(struct fb_info *info, unsigned long arg);

#ifdef CONFIG_EMXX_ANDROID
static int emxx_fb_ioc_blit(struct fb_info *info, unsigned long arg);
#endif
static void set_lcd_size(enum EMXX_FB_OUTPUT_MODE mode, emxx_fb_par *par);
static void set_lcd_var(enum EMXX_FB_OUTPUT_MODE mode, emxx_fb_par *par);
static int set_image_info_to_2DAPI(int iFBNo, struct scrn_modes *fb_scrn_modes,
 struct fb_info *info);
static int chk_fb_status(int iFBStatus);
static int chk_input_modes(struct scrn_modes fb_scrn_modes);
static int set_image_info_from_2DAPI(int iFBNo,
	struct scrn_modes *fb_scrn_modes, struct fb_info *info);
static int set_image_info_to_LCD(int iFBNo, struct fb_info *info);
static int emxx_fb_open(struct fb_info *info, int user);
static int emxx_fb_release(struct fb_info *info, int user);
static u_long get_line_length(int xres_virtual, int bpp);
static int emxx_fb_check_var(struct fb_var_screeninfo *var,
 struct fb_info *info);
static int emxx_fb_set_par(struct fb_info *info);
static int emxx_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
 u_int transp, struct fb_info *info);
static int emxx_fb_blank(int blank_mode, struct fb_info *info);
static int emxx_fb_pan_display(struct fb_var_screeninfo *var,
 struct fb_info *info);
static int emxx_fb_remove(struct platform_device *device);
static int  __init emxx_fb_probe(struct platform_device *device);
static void __init emxx_fb_probe_error(int iErrorMode,
 struct fb_info *info_0, struct fb_info *info_1, int fb_num);
static void set_buffer_address(int fb_num);
static void set_info(struct fb_info *info, emxx_fb_par **par, int fb_num);
static void set_flag_init(int fb_num);
static void set_val_init(struct fb_info *info, int fb_num);
static void timer_init(struct fb_info *info);
static void timeout_update(unsigned long data);
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
static int emxx_fb_suspend(struct platform_device *dev, pm_message_t state);
static int emxx_fb_suspend_sub(struct platform_device *dev,
 pm_message_t state, int fb_num);
static int emxx_fb_resume(struct platform_device *dev);
static int emxx_fb_resume_sub(struct platform_device *dev, int fb_num);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void emxx_fb_early_suspend(struct early_suspend *h);
static void emxx_fb_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM || CONFIG_DPM */
static int emxx_fb_ioctl(struct fb_info *info, unsigned int cmd,
 unsigned long arg);
static int emxx_fb_mmap(struct fb_info *info, struct vm_area_struct *vma);

/* workqueue function */
static void timeout_update_bottom_half_do(struct work_struct *num);


/********************************************************
 *  Function Structure                                  *
 *******************************************************/
/* fb ops */
static struct fb_ops emxx_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_open        = emxx_fb_open,
	.fb_release     = emxx_fb_release,
	.fb_check_var   = emxx_fb_check_var,
	.fb_set_par     = emxx_fb_set_par,
	.fb_setcolreg   = emxx_fb_setcolreg,
	.fb_blank       = emxx_fb_blank,
	.fb_pan_display = emxx_fb_pan_display,
	.fb_fillrect    = cfb_fillrect,  /* fb common function */
	.fb_copyarea    = cfb_copyarea,  /* fb common function */
	.fb_imageblit   = cfb_imageblit, /* fb common function */
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	.fb_cursor      = soft_cursor,   /* fb common function */
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */
	.fb_ioctl       = emxx_fb_ioctl,
	.fb_mmap        = emxx_fb_mmap,
};

/* platform driver */
static struct platform_driver emxx_fb_driver = {
	.probe       = emxx_fb_probe,
	.remove      = emxx_fb_remove,

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend     = emxx_fb_suspend,
	.resume      = emxx_fb_resume,
#endif
#endif /* CONFIG_PM || CONFIG_DPM */

	.driver.name = DEV_NAME,
	.driver.bus  = &platform_bus_type,
};


/********************************************************
 *  Function Definitions                                *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_fb_ioctl
* FUNCTION : frame buffer driver ioctl
* RETURN   :  0      : success
*            -1      :
*            -EFAULT :
*            -EBUSY  :
*            -ENXIO  :
*            -ENOIOCTLCMD:
* NOTE     : none
******************************************************************************/
int emxx_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case EMXX_FB_CHKSCRN:
		/* check frame buffer status        */
		return emxx_fb_ioc_chk_scrn(info, arg);

	case EMXX_FB_GET_MODES:
		/* get frame buffer infomation      */
		return emxx_fb_ioc_get_modes(info, arg);

	case EMXX_FB_SET_MODES:
		/* Set or Update frame buffer infomation   */
		return emxx_fb_ioc_set_modes(info, cmd, arg);

	case EMXX_FB_UPDATE_SCRN:
		return emxx_fb_ioc_update_scrn(info, cmd, arg);

	case EMXX_FB_GET_OUTPUT:
		/* get output device information */
		return emxx_fb_ioc_get_output(info, arg);

	case EMXX_FB_SET_OUTPUT:
		/* Set output device information */
		return emxx_fb_ioc_set_output(info, arg);

#ifdef CONFIG_EMXX_ANDROID
	case EMXX_FB_BLIT:
		/* copybit */
		return emxx_fb_ioc_blit(info, arg);
#endif

	default:
		printk_dbg((_DEBUG_FB & 0x01),
		 "ioctl(*** unknown request ***)\n");
		return -ENOIOCTLCMD;
	}
}


/******************************************************************************
* MODULE   : emxx_fb_ioc_chk_scrn
* FUNCTION : frame buffer driver ioctl EMXX_FB_CHKSCRN
* RETURN   :  0      : success
*            -EFAULT :
*            timeout value : timeout
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_chk_scrn(struct fb_info *info, unsigned long arg)
{
	struct scrn_mode       FBScrnMode;
	emxx_fb_par   *par  = (emxx_fb_par *)info->par;

	printk_dbg((_DEBUG_FB & 0x01), "ioctl(CHKSCRN) <start>\n");

	down(&par->sem_outputmode);

	/* get scrn mode info */
	if (copy_from_user(&FBScrnMode,
		(struct scrn_mode *) arg, sizeof(struct scrn_mode))) {
		up(&par->sem_outputmode);
		return -EFAULT;
	}

	/* check frame buffer status */
	FBScrnMode.page0 = chk_fb_status(fb_data[par->devflag].iFBStatus_A);
	FBScrnMode.page1 = chk_fb_status(fb_data[par->devflag].iFBStatus_B);

	/* if arg->timeout=0, then break */
	if (FBScrnMode.timeout <= 0) {
		if (copy_to_user((struct scrn_mode *) arg,
			&FBScrnMode, sizeof(struct scrn_mode))) {
			up(&par->sem_outputmode);
			return -EFAULT;
		}
		up(&par->sem_outputmode);
		return 0;
	}

	fb_chk_scrn(&FBScrnMode, info);

	/* return scrn mode info */
	if (copy_to_user((struct scrn_mode *) arg,
	 &FBScrnMode, sizeof(struct scrn_mode))) {
		up(&par->sem_outputmode);
		return -EFAULT;
	}
	if (FBScrnMode.page0 == WAITING_FOR_ON_DISPLAY
	 && FBScrnMode.page1 == WAITING_FOR_ON_DISPLAY) {
		up(&par->sem_outputmode);
		return FBScrnMode.timeout;
	}

	up(&par->sem_outputmode);

	printk_dbg((_DEBUG_FB & 0x02), "ioctl(CHKSCRN) <end>\n");
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_ioc_get_modes
* FUNCTION : frame buffer driver ioctl EMXX_FB_GET_MODES
* RETURN   :  0      : success
*            -EFAULT :
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_get_modes(struct fb_info *info, unsigned long arg)
{
	struct scrn_modes    FBScrnModes;
	emxx_fb_par *par  = (emxx_fb_par *)info->par;

	printk_dbg((_DEBUG_FB & 0x01), "ioctl(GET_MODES) <start>\n");

	down(&par->sem_outputmode);

	/* set now on display infomation */
	set_image_info_to_2DAPI(fb_data[par->devflag].iNowDispPage,
	 &FBScrnModes, info);

	/* copy to user scrn mode infomation */
	if (copy_to_user((struct scrn_modes *) arg,
		&FBScrnModes, sizeof(struct scrn_modes))) {
		up(&par->sem_outputmode);
		return -EFAULT;
	}

	up(&par->sem_outputmode);

	printk_dbg((_DEBUG_FB & 0x02), "(ioctl(GET_MODES) <end>\n");
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_ioc_set_modes
* FUNCTION : frame buffer driver ioctl EMXX_FB_SET_MODES
* RETURN   :  0      : success
*            -1      :
*            -EBUSY  :
*            -ENXIO  :
*            -EFAULT :
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_set_modes(struct fb_info *info, unsigned int cmd,
 unsigned long arg)
{
	return emxx_fb_ioc_update_scrn(info, cmd, arg);
}


/******************************************************************************
* MODULE   : emxx_fb_ioc_update_scrn
* FUNCTION : frame buffer driver ioctl EMXX_FB_UPDATE_SCRN
* RETURN   :  0      : success
*            -1      :
*            -EBUSY  :
*            -EFAULT :
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_update_scrn(struct fb_info *info, unsigned int cmd,
 unsigned long arg)
{
	struct scrn_modes    FBSrcnModes;
	emxx_fb_par *par = (emxx_fb_par *)info->par;
	int           ret;

	printk_dbg((_DEBUG_FB & 0x01), "ioctl(UPDATE_SCRN) <start>\n");

	down(&par->sem_outputmode);

	/* copy fb image info from 2D API */
	if (copy_from_user(&FBSrcnModes, (struct scrn_modes *) arg,
	 sizeof(struct scrn_modes))) {
		up(&par->sem_outputmode);
		return -EFAULT;
	}

	/* check fb image infomation from 2D API */
	if (chk_input_modes(FBSrcnModes)) {
		up(&par->sem_outputmode);
		return -1;
	}
	ret = fb_update_scrn(&FBSrcnModes, info, cmd);
	up(&par->sem_outputmode);

	printk_dbg((_DEBUG_FB & 0x02), "ioctl(UPDATE_SCRN) <end>\n");
	return ret;
}


/******************************************************************************
* MODULE   : emxx_fb_ioc_get_output
* FUNCTION : frame buffer driver ioctl EMXX_FB_GET_OUTPUT
* RETURN   :  0      : success
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_get_output(struct fb_info *info, unsigned long arg)
{
	enum EMXX_FB_OUTPUT_MODE mode;
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	printk_dbg((_DEBUG_FB & 0x01), "<start>\n");

	down(&par->sem_outputmode);
	mode = par->output_mode;
	/* copy to user EMXX_FB_OUTPUT_MODE infomation */
	if (copy_to_user((enum EMXX_FB_OUTPUT_MODE *) arg, &mode,
			  sizeof(enum EMXX_FB_OUTPUT_MODE))) {
		up(&par->sem_outputmode);
		return -EFAULT;
	}
	up(&par->sem_outputmode);

	printk_dbg((_DEBUG_FB & 0x02), "< end >\n");
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_ioc_set_output
* FUNCTION : frame buffer driver ioctl EMXX_FB_SET_OUTPUT
* RETURN   :  0      : success
*            -EFAULT :
*            -EINVAL :
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_set_output(struct fb_info *info, unsigned long arg)
{
	enum EMXX_FB_OUTPUT_MODE mode;
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	printk_dbg((_DEBUG_FB & 0x01), "<start>\n");

	down(&par->sem_outputmode);

	/* copy from user EMXX_FB_OUTPUT_MODE infomation */
	if (copy_from_user(&mode, (enum EMXX_FB_OUTPUT_MODE *) arg,
			    sizeof(enum EMXX_FB_OUTPUT_MODE))) {
		up(&par->sem_outputmode);
		return -EFAULT;
	}

	switch (mode) {
	case EMXX_FB_OUTPUT_MODE_LCD:
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		break;
	default:
		printk_dbg((_DEBUG_FB & 0x01),
		 "enum EMXX_FB_OUTPUT_MODE value is out of range\n");
		up(&par->sem_outputmode);
		return -EINVAL;
		break;
	}

	set_lcd_var(mode, par);
	set_lcd_size(mode, par);

	par->output_mode = mode;

	up(&par->sem_outputmode);

	printk_dbg((_DEBUG_FB & 0x02), "< end >\n");
	return 0;
}


#ifdef CONFIG_EMXX_ANDROID
/******************************************************************************
* MODULE   : emxx_fb_ioc_blit
* FUNCTION : frame buffer driver ioctl EMXX_FB_BLIT
* RETURN   :  0      : success
*            -EFAULT :
*            -EINVAL :
* NOTE     : none
******************************************************************************/
static int emxx_fb_ioc_blit(struct fb_info *info, unsigned long arg)
{
	struct emxx_fb_blit_req req;
	struct emxx_fb_blit_req_list req_list;
	int i;
	int ret = 0;
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	printk_dbg((_DEBUG_FB & 0x01), "ioctl(BLIT) <start>\n");

	if (copy_from_user(&req_list, (struct emxx_fb_blit_req_list *)arg,
	    sizeof(struct emxx_fb_blit_req_list)))
		return -EFAULT;

	down(&par->sem_copybit);

	for (i = 0; i < req_list.count; i++) {
		struct emxx_fb_blit_req_list *list =
			(struct emxx_fb_blit_req_list *)arg;
		if (copy_from_user(&req, &list->req[i],
		    sizeof(struct emxx_fb_blit_req))) {
			ret = -EFAULT;
			goto fail;
		}
		ret = emxx_fb_blit(info, &req);
		if (ret)
			goto fail;
	}

fail:
	up(&par->sem_copybit);

	printk_dbg((_DEBUG_FB & 0x02), "ioctl(BLIT) <end>\n");
	return ret;
}
#endif


/******************************************************************************
* MODULE   : set_lcd_size
* FUNCTION : output_mode change
* RETURN   : node
* NOTE     : none
******************************************************************************/
static void set_lcd_size(enum EMXX_FB_OUTPUT_MODE mode, emxx_fb_par *par)
{
	printk_dbg((_DEBUG_FB & 0x01), "<start>\n");

	down(&par->sem_fb_image_info);

	switch (mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		fb_data[par->devflag].fb_image_info.image_data.hsize =
		 FRONT_WIDTH_LCD;
		fb_data[par->devflag].fb_image_info.image_data.vsize =
		 FRONT_HEIGHT_LCD;
		fb_data[par->devflag].fb_image_info.image_data.size  =
		 FRONT_WIDTH_V_LCD * BYTES_PER_PIXEL;
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
		fb_data[par->devflag].fb_image_info.image_data.hsize =
		 FRONT_WIDTH_1080I;
		fb_data[par->devflag].fb_image_info.image_data.vsize =
		 FRONT_HEIGHT_1080I;
		fb_data[par->devflag].fb_image_info.image_data.size  =
		 FRONT_WIDTH_V_1080I * BYTES_PER_PIXEL;
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		fb_data[par->devflag].fb_image_info.image_data.hsize =
		 FRONT_WIDTH_720P;
		fb_data[par->devflag].fb_image_info.image_data.vsize =
		 FRONT_HEIGHT_720P;
		fb_data[par->devflag].fb_image_info.image_data.size  =
		 FRONT_WIDTH_V_720P * BYTES_PER_PIXEL;
		break;
	}

	up(&par->sem_fb_image_info);

	printk_dbg((_DEBUG_FB & 0x01), "< end >\n");
}


/******************************************************************************
* MODULE   : set_lcd_var
* FUNCTION : output_mode
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void set_lcd_var(enum EMXX_FB_OUTPUT_MODE mode, emxx_fb_par *par)
{
	printk_dbg((_DEBUG_FB & 0x01), "<start>\n");

	down(&par->sem_fb_image_info);

	switch (mode) {
	default:
	case EMXX_FB_OUTPUT_MODE_LCD:
		par->fb_inf->var = emxx_fb_default;
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_1080I:
		par->fb_inf->var = emxx_fb_default;
		par->fb_inf->var.xres         = FRONT_WIDTH_1080I;
		par->fb_inf->var.yres         = FRONT_HEIGHT_1080I;
		par->fb_inf->var.xres_virtual = FRONT_WIDTH_V_1080I;
#ifdef CONFIG_FB_EMXX_PANDISP
		par->fb_inf->var.yres_virtual = (FRONT_HEIGHT_V_1080I * 2);
#else	/* CONFIG_FB_EMXX_PANDISP */
		par->fb_inf->var.yres_virtual = FRONT_HEIGHT_V_1080I;
#endif	/* CONFIG_FB_EMXX_PANDISP */
		par->fb_inf->var.left_margin  = HFRONTP_1080I;
		par->fb_inf->var.right_margin = HBACKP_1080I;
		par->fb_inf->var.upper_margin = VFRONTP_1080I;
		par->fb_inf->var.lower_margin = VBACKP_1080I;
		par->fb_inf->var.hsync_len    = HPULSE_1080I;
		par->fb_inf->var.vsync_len    = VPULSE_1080I;
		par->fb_inf->var.vmode        = FB_VMODE_INTERLACED;
		break;
	case EMXX_FB_OUTPUT_MODE_HDMI_720P:
		par->fb_inf->var = emxx_fb_default;
		par->fb_inf->var.xres         = FRONT_WIDTH_720P;
		par->fb_inf->var.yres         = FRONT_HEIGHT_720P;
		par->fb_inf->var.xres_virtual = FRONT_WIDTH_V_720P;
#ifdef CONFIG_FB_EMXX_PANDISP
		par->fb_inf->var.yres_virtual = (FRONT_HEIGHT_V_720P * 2);
#else	/* CONFIG_FB_EMXX_PANDISP */
		par->fb_inf->var.yres_virtual = FRONT_HEIGHT_V_720P;
#endif	/* CONFIG_FB_EMXX_PANDISP */
		par->fb_inf->var.left_margin  = HFRONTP_720P;
		par->fb_inf->var.right_margin = HBACKP_720P;
		par->fb_inf->var.upper_margin = VFRONTP_720P;
		par->fb_inf->var.lower_margin = VBACKP_720P;
		par->fb_inf->var.hsync_len    = HPULSE_720P;
		par->fb_inf->var.vsync_len    = VPULSE_720P;
		par->fb_inf->var.vmode        = FB_VMODE_NONINTERLACED;
		break;
	}
#ifdef CONFIG_FB_EMXX_ABGR8888
 #ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
		par->fb_inf->var.red.offset    = 16;
		par->fb_inf->var.green.offset  = 8;
		par->fb_inf->var.blue.offset   = 0;
		par->fb_inf->var.transp.offset = 24;
	}
 #endif
#endif
	emxx_fb_set_par(par->fb_inf);

	up(&par->sem_fb_image_info);

	printk_dbg((_DEBUG_FB & 0x01), "< end >\n");
}


/******************************************************************************
* MODULE   : set_image_info_to_2DAPI
* FUNCTION : set other image infomation (frame bufferA/B) to 2D API
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int set_image_info_to_2DAPI(int iFBNo, struct scrn_modes *fb_scrn_modes,
 struct fb_info *info)
{
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	/* set fb A other image infomation */
	if (iFBNo == FB_NO_A) {
		down(&par->sem_fb_other_info_A); /* lock fb_other_info_A */
		fb_scrn_modes->tc_enable =
		 fb_data[par->devflag].fb_other_info_A.iMaskColFlg;
		fb_scrn_modes->t_color   =
		 fb_data[par->devflag].fb_other_info_A.iMaskCol;
		fb_scrn_modes->alpha     =
		 fb_data[par->devflag].fb_other_info_A.iAlpha;
		fb_scrn_modes->rot       =
		 fb_data[par->devflag].fb_other_info_A.iInvFlg;
		fb_scrn_modes->page      =
		 fb_data[par->devflag].fb_other_info_A.iPage;
		fb_scrn_modes->update    =
		 fb_data[par->devflag].fb_other_info_A.iUpDate;
		up(&par->sem_fb_other_info_A);	/* unlock fb_other_info_A */
	} else {
		/* set fb B other image infomation */
		down(&par->sem_fb_other_info_B); /* lock fb_other_info_B */
		fb_scrn_modes->tc_enable =
		 fb_data[par->devflag].fb_other_info_B.iMaskColFlg;
		fb_scrn_modes->t_color   =
		 fb_data[par->devflag].fb_other_info_B.iMaskCol;
		fb_scrn_modes->alpha     =
		 fb_data[par->devflag].fb_other_info_B.iAlpha;
		fb_scrn_modes->rot       =
		 fb_data[par->devflag].fb_other_info_B.iInvFlg;
		fb_scrn_modes->page      =
		 fb_data[par->devflag].fb_other_info_B.iPage;
		fb_scrn_modes->update    =
		 fb_data[par->devflag].fb_other_info_B.iUpDate;
		up(&par->sem_fb_other_info_B);	/* unlock fb_other_info_B */
	}
	return 0;
}


/******************************************************************************
* MODULE   : chk_fb_status
* FUNCTION : check frame buffer (A/B) status
* RETURN   : 0 : update enable
*            1 : update disable
* NOTE     : none
******************************************************************************/
static int chk_fb_status(int iFBStatus)
{
	int iRet;

	/* check frame buffer status */
	if (iFBStatus == IS_NODATA || iFBStatus == ENABLE_WRITE)
		iRet = OUT_OF_DISPLAY;
	else
		iRet = WAITING_FOR_ON_DISPLAY;

	return iRet;
}


/******************************************************************************
* MODULE   : chk_input_modes
* FUNCTION : check input scrn_modes value from 2D API
* RETURN   : 0       : correct
*            -EINVAL : incorrect
* NOTE     : none
******************************************************************************/
static int chk_input_modes(struct scrn_modes fb_scrn_modes)
{
	/* check mask color flag */
	if ((fb_scrn_modes.tc_enable != FB_MASK_COLOR_DISP_OFF)
	 && (fb_scrn_modes.tc_enable != FB_MASK_COLOR_DISP_ON)) {
		printk_wrn("mask color flag incorrect  tc_enable = %d\n",
		 fb_scrn_modes.tc_enable);
		return -EINVAL;
	}

#if defined(CONFIG_FB_EMXX_ARGB8888) || defined(CONFIG_FB_EMXX_BGR888)
	/* check mask color */
	if ((fb_scrn_modes.t_color < 0x00)
	 || (0xFFFFFF < fb_scrn_modes.t_color)) {
		printk_wrn("mask color incorrect  t_color = %d\n",
		 fb_scrn_modes.t_color);
		return -EINVAL;
	}
#else
	/* check mask color */
	if ((fb_scrn_modes.t_color < 0x00)
	 || (0xFFFF < fb_scrn_modes.t_color)) {
		printk_wrn("mask color incorrect  t_color = %d\n",
		 fb_scrn_modes.t_color);
		return -EINVAL;
	}
#endif

	/* check alpha */
	if ((fb_scrn_modes.alpha < 0x00) || (0xFF < fb_scrn_modes.alpha)) {
		printk_wrn("alpha incorrect  alpha = %d\n",
		 fb_scrn_modes.alpha);
		return -EINVAL;
	}

	/* check rot */
	if ((fb_scrn_modes.rot != NO_INVERSE)
	 && (fb_scrn_modes.rot != UDRL_INVERSE)) {
		printk_wrn("rot incorrect  rot = %d\n", fb_scrn_modes.rot);
		return -EINVAL;
	}

	/* check page */
	if ((fb_scrn_modes.page != FB_NO_A)
	 && (fb_scrn_modes.page != FB_NO_B)) {
		printk_wrn("page incorrect  page = %d\n", fb_scrn_modes.page);
		return -EINVAL;
	}

	/* check update */
	if ((fb_scrn_modes.update != UPDATE_FLAG_OFF)
	 && (fb_scrn_modes.update != UPDATE_FLAG_ON)) {
		printk_wrn("update incorrect  update = %d\n",
		 fb_scrn_modes.update);
		return -EINVAL;
	}

	return 0;
}


/******************************************************************************
* MODULE   : set_image_info_from_2DAPI
* FUNCTION : set image infomation from LCD to fb local
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int set_image_info_from_2DAPI(int iFBNo,
 struct scrn_modes *fb_scrn_modes, struct fb_info *info)
{
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	/* set FB_A image infomation from 2D API */
	if (iFBNo == FB_NO_A) {
		fb_data[par->devflag].iFBStatus_A = NOW_WRITING_TO_FB;

		down(&par->sem_fb_other_info_A); /* lock fb_other_info_A */
		fb_data[par->devflag].fb_other_info_A.iMaskColFlg =
		 fb_scrn_modes->tc_enable;
		fb_data[par->devflag].fb_other_info_A.iMaskCol    =
		 fb_scrn_modes->t_color;
		fb_data[par->devflag].fb_other_info_A.iAlpha      =
		 fb_scrn_modes->alpha;
		fb_data[par->devflag].fb_other_info_A.iInvFlg     =
		 fb_scrn_modes->rot;
		fb_data[par->devflag].fb_other_info_A.iPage       =
		 fb_scrn_modes->page;
		fb_data[par->devflag].fb_other_info_A.iUpDate     =
		 fb_scrn_modes->update;
		up(&par->sem_fb_other_info_A); /* unlock fb_other_info_A */
	} else {
		/* set FB_B image infomation from 2D API */
		fb_data[par->devflag].iFBStatus_B = NOW_WRITING_TO_FB;

		down(&par->sem_fb_other_info_B); /* lock fb_other_info_B */
		fb_data[par->devflag].fb_other_info_B.iMaskColFlg =
		 fb_scrn_modes->tc_enable;
		fb_data[par->devflag].fb_other_info_B.iMaskCol    =
		 fb_scrn_modes->t_color;
		fb_data[par->devflag].fb_other_info_B.iAlpha      =
		 fb_scrn_modes->alpha;
		fb_data[par->devflag].fb_other_info_B.iInvFlg     =
		 fb_scrn_modes->rot;
		fb_data[par->devflag].fb_other_info_B.iPage       =
		 fb_scrn_modes->page;
		fb_data[par->devflag].fb_other_info_B.iUpDate     =
		 fb_scrn_modes->update;
		up(&par->sem_fb_other_info_B); /* unlock fb_other_info_B */
	}
	printk_dbg((_DEBUG_FB & 0x08), "[> set_2DAPI:   A(%d) B(%d)\n",
	 fb_data[par->devflag].iFBStatus_A, fb_data[par->devflag].iFBStatus_B);

	return 0;
}


/******************************************************************************
* MODULE   : set_image_info_to_LCD
* FUNCTION : set image infomation to LCD
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int set_image_info_to_LCD(int iFBNo, struct fb_info *info)
{
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	/* set fb A infomation */
	if (iFBNo == DISP_BUFA) {
		printk_dbg((_DEBUG_FB & 0x04), "DispBufA(%08lx)\n",
		 fb_data[par->devflag].DispBufA);
		fb_data[par->devflag].fb_image_info.image_data.yrgbaddr =
		 fb_data[par->devflag].DispBufA;

		/* set other image infomation */
		down(&par->sem_fb_other_info_A); /* lock fb_other_info_A */
		fb_data[par->devflag].fb_image_info.maskcolr     =
		 fb_data[par->devflag].fb_other_info_A.iMaskCol;
		fb_data[par->devflag].fb_image_info.maskcolrflg  =
		 fb_data[par->devflag].fb_other_info_A.iMaskColFlg;
		fb_data[par->devflag].fb_image_info.alpha        =
		 fb_data[par->devflag].fb_other_info_A.iAlpha;
		fb_data[par->devflag].fb_image_info.invflg       =
		 fb_data[par->devflag].fb_other_info_A.iInvFlg;
		fb_data[par->devflag].fb_image_info.mix_buf_page =
		 fb_data[par->devflag].fb_other_info_A.iPage;
		up(&par->sem_fb_other_info_A);  /* unlock fb_other_info_A */
	} else {
	/* set fb B infomation */
		printk_dbg((_DEBUG_FB & 0x04), "DispBufB(%08lx)\n",
		 fb_data[par->devflag].DispBufB);
		fb_data[par->devflag].fb_image_info.image_data.yrgbaddr =
		 fb_data[par->devflag].DispBufB;

		/* set other image infomation */
		down(&par->sem_fb_other_info_B); /* lock fb_other_info_B */
		fb_data[par->devflag].fb_image_info.maskcolr     =
		 fb_data[par->devflag].fb_other_info_B.iMaskCol;
		fb_data[par->devflag].fb_image_info.maskcolrflg  =
		 fb_data[par->devflag].fb_other_info_B.iMaskColFlg;
		fb_data[par->devflag].fb_image_info.alpha        =
		 fb_data[par->devflag].fb_other_info_B.iAlpha;
		fb_data[par->devflag].fb_image_info.invflg       =
		 fb_data[par->devflag].fb_other_info_B.iInvFlg;
		fb_data[par->devflag].fb_image_info.mix_buf_page =
		 fb_data[par->devflag].fb_other_info_B.iPage;
		up(&par->sem_fb_other_info_B);  /* unlock fb_other_info_B */
	}

	return 0;
}


/******************************************************************************
* MODULE   : fb_update_scrn
* FUNCTION : update frame buffer infomation
* RETURN   :  0      : success
*            -EBUSY  :
* NOTE     : none
******************************************************************************/
static int fb_update_scrn(struct scrn_modes *FBSrcnModes, struct fb_info *info,
 unsigned int cmd)
{
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	printk_dbg((_DEBUG_FB & 0x01), "ioctl(UPDATE_SCRN) <start>\n");

	down(&par->sem_fb_image_info);		/* lock fb_image_info */

	/* set fb image infomation from 2D API */
	set_image_info_from_2DAPI(FBSrcnModes->page, FBSrcnModes, info);

	/* set fb image infomaition to LCD */
	set_image_info_to_LCD(FBSrcnModes->page, info);

	fb_data[par->devflag].iNextDispPage = FBSrcnModes->page;

	/* when cmd is EMXX_FB_SET_MODES, set fb image update flag off */
	if (cmd == EMXX_FB_SET_MODES)
		fb_data[par->devflag].fb_image_info.update_flag = FB_UPDATE_OFF;
	else {
		/* when cmd is EMXX_FB_UPDATE_MODES,
		   set fb image update flag on */
		/* absolutely update data */
		if (FBSrcnModes->update == UPDATE_FLAG_ON)
			fb_data[par->devflag].fb_image_info.update_flag =
			 FB_ABSOLUTERY_UPDATE;
		else {
			/* when ONLY_2D_MODE, update data */
			fb_data[par->devflag].fb_image_info.update_flag =
			 FB_UPDATE_ON;
		}
	}

	fb_data[par->devflag].fb_image_info.output_mode = par->output_mode;

	/* set image infomation to LCD driver */
	printk_dbg((_DEBUG_FB & 0x10), "ioctl(UPDATE_SCRN): request LCD\n");

	printk_dbg((_DEBUG_FB & 0x02), "toLCD: PaddrYRGB(%lx)\n",
	 fb_data[par->devflag].fb_image_info.image_data.yrgbaddr);
	if (emxx_lcd_set_fb_image(
		&fb_data[par->devflag].fb_image_info)) {
		up(&par->sem_fb_image_info); /* unlock fb_image_info */
		return -EBUSY;
	}

	up(&par->sem_fb_image_info);		/* unlock fb_image_info */

#ifndef CONFIG_FB_EMXX_PANDISP
	if (FBSrcnModes->page == DISP_BUFA)
		info->screen_base =
			(char __iomem *)fb_data[par->devflag].DispBufAV;
	else
		info->screen_base =
			(char __iomem *)fb_data[par->devflag].DispBufBV;

#endif	/* CONFIG_FB_EMXX_PANDISP */

	printk_dbg((_DEBUG_FB & 0x02), "ioctl(UPDATE_SCRN) <end>\n");
	return 0;
}


/******************************************************************************
* MODULE   : fb_chk_scrn
* FUNCTION : frame buffer driver ioctl
* RETURN   :  0      : success
*            -EFAULT :
* NOTE     : none
******************************************************************************/
static int fb_chk_scrn(struct scrn_mode *FBScrnMode, struct fb_info *info)
{
#if (TIMEOUT_MSLEEP == 0)
	int             iWaitTime  = 0;
	int             iTotalTime = 0;
	emxx_fb_par   *par  = (emxx_fb_par *)info->par;
#else /* TIMEOUT_MSLEEP == 1 */
	int             iTotalTime = 0;
	struct timeval  tStartTime;
	struct timeval  tNowTime;
	emxx_fb_par   *par  = (emxx_fb_par *)info->par;

	/* get start time */
	do_gettimeofday(&tStartTime);
#endif
	printk_dbg((_DEBUG_FB & 0x01), "ioctl(CHKSCRN) <start>\n");

	/* if both A and B states are "WAITING_FOR_ON_DISPLAY", */
	/*    then waits during the time specified by "timeout" */
	if (FBScrnMode->page0 == WAITING_FOR_ON_DISPLAY
	 && FBScrnMode->page1 == WAITING_FOR_ON_DISPLAY) {
#if (TIMEOUT_MSLEEP == 0)
		/* loop until timeout or enable write */
		do {
			/* timeout time (msec to jiffies) */
			iWaitTime = (RECHK_TIME + 1000/HZ-1) / (1000/HZ);

			wait_event_interruptible_timeout(par->wait_fb_chk_scrn,
			((fb_data[par->devflag].iFBStatus_A == IS_NODATA) ||
			(fb_data[par->devflag].iFBStatus_A == ENABLE_WRITE) ||
			(fb_data[par->devflag].iFBStatus_B == IS_NODATA) ||
			(fb_data[par->devflag].iFBStatus_B == ENABLE_WRITE)),
			iWaitTime);

			iTotalTime += RECHK_TIME;

			/* re-check frame buffer status */
			FBScrnMode->page0 =
			 chk_fb_status(fb_data[par->devflag].iFBStatus_A);
			FBScrnMode->page1 =
			 chk_fb_status(fb_data[par->devflag].iFBStatus_B);

			/* when over timeout time, return */
			if (iTotalTime > FBScrnMode->timeout)
				break;

		} while (FBScrnMode->page0 == WAITING_FOR_ON_DISPLAY
		 && FBScrnMode->page1 == WAITING_FOR_ON_DISPLAY);
#else /* TIMEOUT_MSLEEP == 1 */
		/* loop until timeout or enable write */
		do {
			/* sleep 1ms */
			msleep(RECHK_TIME);

			/* get time */
			do_gettimeofday(&tNowTime);
			if (tNowTime.tv_sec > tStartTime.tv_sec) {
				iTotalTime =
				 (tNowTime.tv_sec - tStartTime.tv_sec) * 1000;
				if (tNowTime.tv_usec > tStartTime.tv_usec) {
					iTotalTime +=
						(tNowTime.tv_usec -
						tStartTime.tv_usec) / 1000;
				else
					iTotalTime -=
						(tStartTime.tv_usec -
						tNowTime.tv_usec) / 1000;

			else
				iTotalTime =
				 (tNowTime.tv_usec - tStartTime.tv_usec) / 1000;


			/* re-check frame buffer status */
			FBScrnMode->page0 =
			 chk_fb_status(fb_data[par->devflag].iFBStatus_A);
			FBScrnMode->page1 =
			 chk_fb_status(fb_data[par->devflag].iFBStatus_B);

			/* when over timeout time, return */
			if (iTotalTime > FBScrnMode->timeout)
				break;

		} while (FBScrnMode->page0 == WAITING_FOR_ON_DISPLAY
		 && FBScrnMode->page1 == WAITING_FOR_ON_DISPLAY);
#endif
	}
	printk_dbg((_DEBUG_FB & 0x08), "ioctl(CHKSCRN): A(%d) B(%d)\n",
	 FBScrnMode->page0, FBScrnMode->page1);

	printk_dbg((_DEBUG_FB & 0x02), "ioctl(CHKSCRN) <end>\n");
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_mmap
* FUNCTION : mmap
* RETURN   : 0       : success
*          : -EINVAL : input value error
*          : -EAGAIN : run again
* NOTE     : none
******************************************************************************/
int emxx_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	off = vma->vm_pgoff << PAGE_SHIFT;

	if (off < DispBufAllLength) {
		start = (unsigned long)Smem;
		len = PAGE_ALIGN((start & ~PAGE_MASK) + DispBufAllLength);
	} else
		return -EINVAL;

	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	/* Accessing memory will be done non-cached. */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	/* To stop the swapper from even considering these pages */
	vma->vm_flags |= (VM_IO | VM_RESERVED);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
	 vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_open
* FUNCTION : open qfb driver
* RETURN   : 0      : success
*            -EBUSY : LCD initialize failed
* NOTE     : none
******************************************************************************/
static int emxx_fb_open(struct fb_info *info, int user)
{
	emxx_fb_par *par      = (emxx_fb_par *)info->par;

	if (fb_data[par->devflag].is_first) {
		/* initialize LCD driver */
		if (init_lcdhw() != 0) {
			printk_wrn("initialize LCD failed.\n");
			return -EBUSY;
		}

		printk_dbg((_DEBUG_FB & 0x10), "queue_work\n");
		queue_work(par->emxx_fb_workqueue,
		 &par->wk_timeout_update_bottom_half);
		timer_init(info);
		fb_data[par->devflag].is_first = 0;
	}

	atomic_inc(&par->use_count);

	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_release
* FUNCTION : close qfb driver
* RETURN   : 0       : success
*            -EINVAL :
* NOTE     : none
******************************************************************************/
static int emxx_fb_release(struct fb_info *info, int user)
{
	emxx_fb_par *par = (emxx_fb_par *)info->par;
	u32           count;

	down(&par->sem_outputmode);

	down(&par->sem_fb_image_info);		/* lock fb_image_info */
	count = atomic_read(&par->use_count);
	if (count == 0) {
		up(&par->sem_fb_image_info);	/* unlock fb_image_info */
		up(&par->sem_outputmode);
		return -EINVAL;
	}

	if (count == 1) {
		fb_data[par->devflag].iUpdateTimerFlag =
		 DEFALT_UPDATE_TIMER_LCD;
		fb_data[par->devflag].iTimeoutTime     = TIMEOUT_TIME;
		del_timer(&fb_data[par->devflag].emxx_fb_timer);
		fb_data[par->devflag].iTimerStatusFlag = FB_TIMER_STOP;
		if ((fb_data[par->devflag].iUpdateTimerFlag ==
		 TIMER_ON) &&
		 (fb_data[par->devflag].iMixModeFlg ==
		 ONLY_2D_MODE)) {
			fb_data[par->devflag].emxx_fb_timer.expires =
			 jiffies +
			 (fb_data[par->devflag].iTimeoutTime *
			 HZ / 1000);
			add_timer(
			 &fb_data[par->devflag].emxx_fb_timer);
			fb_data[par->devflag].iTimerStatusFlag =
			 FB_TIMER_START;
		}
	}

	atomic_dec(&par->use_count);
	up(&par->sem_fb_image_info);		/* unlock fb_image_info */
	up(&par->sem_outputmode);
	return 0;
}


/******************************************************************************
* MODULE   : get_line_length
* FUNCTION : get line length
* RETURN   : length
* NOTE     : none
******************************************************************************/
static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return length;
}


/******************************************************************************
* MODULE   : emxx_fb_check_var
* FUNCTION : variable check
* RETURN   : 0       : success
*            -EINVAL : input value incorrect
*            -ENOMEM :
* NOTE     : none
******************************************************************************/
static int emxx_fb_check_var(struct fb_var_screeninfo *var,
 struct fb_info *info)
{
	u_long line_length;
	emxx_fb_par *par  = (emxx_fb_par *)info->par;

	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	/* Some very basic checks */
	if (!var->xres)
		var->xres = 1;
	if (!var->yres)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres_virtual = var->xres;
	if (var->yres > var->yres_virtual)
		var->yres_virtual = var->yres;
	if (var->bits_per_pixel <= 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel <= 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel <= 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel <= 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel <= 32)
		var->bits_per_pixel = 32;
	else
		return -EINVAL;

	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;

	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	/* Memory limit */
	line_length = get_line_length(var->xres_virtual, var->bits_per_pixel);
#ifdef CONFIG_FB_EMXX_PANDISP
	if (line_length * var->yres_virtual >
	 (fb_data[par->devflag].DispBufLength * 2))
#else	/* CONFIG_FB_EMXX_PANDISP */
	if (line_length * var->yres_virtual >
	 fb_data[par->devflag].DispBufLength)
#endif	/* CONFIG_FB_EMXX_PANDISP */
		return -ENOMEM;


	/* check var */
	switch (var->bits_per_pixel) {
	case 1:	/* FALL THROUGH */
	case 8:
		{
			var->red.offset    = 0;
			var->red.length    = 8;
			var->green.offset  = 0;
			var->green.length  = 8;
			var->blue.offset   = 0;
			var->blue.length   = 8;
			var->transp.offset = 0;
			var->transp.length = 0;
			break;
		}
	case 16:		/* RGBA 5551 */
		{
			if (var->transp.length) {
				var->red.offset    = 0;
				var->red.length    = 5;
				var->green.offset  = 5;
				var->green.length  = 5;
				var->blue.offset   = 10;
				var->blue.length   = 5;
				var->transp.offset = 15;
				var->transp.length = 1;
			} else {		/* RGB 565 */
				var->red.offset    = 11;
				var->red.length    = 5;
				var->green.offset  = 5;
				var->green.length  = 6;
				var->blue.offset   = 0;
				var->blue.length   = 5;
				var->transp.offset = 0;
				var->transp.length = 0;
			}
			break;
		}
	case 24:		/* RGB 888 */
		{
			var->red.offset    = 0;
			var->red.length    = 8;
			var->green.offset  = 8;
			var->green.length  = 8;
			var->blue.offset   = 16;
			var->blue.length   = 8;
			var->transp.offset = 0;
			var->transp.length = 0;
			break;
		}
#ifdef CONFIG_FB_EMXX_ABGR8888
	case 32:		/* RGBA 8888 */
 #ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
			var->red.offset    = 16;
			var->red.length    = 8;
			var->green.offset  = 8;
			var->green.length  = 8;
			var->blue.offset   = 0;
			var->blue.length   = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
		} else {
 #endif
			var->red.offset    = 0;
			var->red.length    = 8;
			var->green.offset  = 8;
			var->green.length  = 8;
			var->blue.offset   = 16;
			var->blue.length   = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
			break;
 #ifdef CONFIG_MACH_EMEV
		}
 #endif
#else
	case 32:		/* ARGB 8888 */
		{
			var->red.offset    = 16;
			var->red.length    = 8;
			var->green.offset  = 8;
			var->green.length  = 8;
			var->blue.offset   = 0;
			var->blue.length   = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
			break;
		}
#endif
	}
	var->red.msb_right    = 0;
	var->green.msb_right  = 0;
	var->blue.msb_right   = 0;
	var->transp.msb_right = 0;

	return 0;
}

/******************************************************************************
* MODULE   : emxx_fb_set_par
* FUNCTION : set qfb parameter
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int emxx_fb_set_par(struct fb_info *info)
{
	info->fix.line_length = get_line_length(info->var.xres_virtual,
	 info->var.bits_per_pixel);
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_setcolreg
* FUNCTION : set qfb color
* RETURN   : 0 : success
*            1 :
* NOTE     : none
******************************************************************************/
static int emxx_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
 u_int transp, struct fb_info *info)
{
	/* No. of hw registers */
	if (regno >= 256)
		return 1;

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:	/* FALL THROUGH */
	case FB_VISUAL_PSEUDOCOLOR:
		{
			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);
			break;
		}
	case FB_VISUAL_DIRECTCOLOR:
		{
			red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
			green = CNVT_TOHW(green, 8);
			blue = CNVT_TOHW(blue, 8);
			/* hey, there is bug in transp handling... */
			transp = CNVT_TOHW(transp, 8);
			break;
		}
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset)
		 | (green << info->var.green.offset)
		 | (blue << info->var.blue.offset)
		 | (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 16:	/* FALL THROUGH */
		case 24:	/* FALL THROUGH */
		case 32:
			{
				((u32 *) (info->pseudo_palette))[regno] = v;
				break;
			}
		case 8:		/* FALL THROUGH */
		default:
			{
				break;
			}
		}
	}
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_blank
* FUNCTION : FBIOBLANK
* RETURN   : 0       : success
*            -EINVAL : input value incorrect
* NOTE     : none
******************************************************************************/
static int emxx_fb_blank(int blank_mode, struct fb_info *info)
{
	int iRet = 0;
	iRet = emxx_lcd_blank(blank_mode);
	if (iRet)
		return iRet;
#ifdef CONFIG_EMXX_NTS
	iRet = emxx_nts_blank(blank_mode);
	if (iRet)
		return iRet;
#endif
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_pan_display
* FUNCTION :
* RETURN   : 0       : success
*            -EINVAL : input value incorrect
*            -EBUSY  : chk_scrn time out
* NOTE     : none
******************************************************************************/
static int emxx_fb_pan_display(struct fb_var_screeninfo *var,
 struct fb_info *info)
{
#ifndef CONFIG_FB_EMXX_PANDISP
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset < 0 || var->yoffset >= info->var.yres_virtual
		 || var->xoffset)
			return -EINVAL;
	else
		if (var->xoffset + var->xres > info->var.xres_virtual
		 || var->yoffset + var->yres > info->var.yres_virtual)
			return -EINVAL;

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

#else	/* CONFIG_FB_EMXX_PANDISP */
	emxx_fb_par *par = (emxx_fb_par *)info->par;
	struct scrn_modes   FBScrnModes;
#ifdef CONFIG_FB_EMXX_PANDISP_BLOCK
	struct scrn_mode    FBScrnMode;
#endif	/* CONFIG_FB_EMXX_PANDISP_BLOCK */
	int          page = FB_NO_A;
	int          ret;
	int          offset = DISPBUF_A_OFFSET;

	if (fb_data[par->devflag].is_first == 0) {

		/* Page Switch Check */
		if (par->old_offset != var->yoffset) {
			if (var->yoffset == offset) {
				page = FB_NO_A;
				down(&par->sem_fb_other_info_A);
				fb_data[par->devflag].fb_other_info_A.iPage =
				 page;
				up(&par->sem_fb_other_info_A);
			} else {
				page = FB_NO_B;
				down(&par->sem_fb_other_info_B);
				fb_data[par->devflag].fb_other_info_B.iPage =
				 page;
				up(&par->sem_fb_other_info_B);
			}

			set_image_info_to_2DAPI(page, &FBScrnModes, info);
			/* UPDATE_FLAG_OFF */
			ret = fb_update_scrn(&FBScrnModes, info,
			 EMXX_FB_UPDATE_SCRN);
			if (ret < 0)
				return ret;

			par->old_offset = var->yoffset;

#ifdef CONFIG_FB_EMXX_PANDISP_BLOCK
			/* BLOCK MODE */
			/* check frame buffer status */
			FBScrnMode.page0 =
			 chk_fb_status(fb_data[par->devflag].iFBStatus_A);
			FBScrnMode.page1 =
			 chk_fb_status(fb_data[par->devflag].iFBStatus_B);
			FBScrnMode.timeout  = FB_PAN_DISP_TIMEOUT;

			fb_chk_scrn(&FBScrnMode, info);

			if (FBScrnMode.page0 == WAITING_FOR_ON_DISPLAY &&
				FBScrnMode.page1 == WAITING_FOR_ON_DISPLAY) {
				return -EBUSY;
			}
#endif	/* CONFIG_FB_EMXX_PANDISP_BLOCK */
		}
	}
#endif	/* CONFIG_FB_EMXX_PANDISP */
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_callback
* FUNCTION : call back from LCD driver
* RETURN   : none
* NOTE     : public function
******************************************************************************/
void emxx_fb_callback(int iWrittenFBNo, int iMixModeStatus, int fb_num)
{
	printk_dbg((_DEBUG_FB & 0x01), "fb(%d) status(%d) from(%d)\n",
	 iWrittenFBNo, iMixModeStatus, fb_num);
	fb_data[fb_num].iMixModeFlg = iMixModeStatus;

	/* fb timer stop */
	del_timer(&fb_data[fb_num].emxx_fb_timer);
	fb_data[fb_num].iTimerStatusFlag = FB_TIMER_STOP;

	/* change FB (A/B) status */
	if (iWrittenFBNo == FB_NO_A) {
		fb_data[fb_num].iNowDispPage = FB_NO_A;
		fb_data[fb_num].iFBStatus_A  = ALREADY_DISPLAY;
		if (fb_data[fb_num].iNextDispPage != FB_NO_B) {
			fb_data[fb_num].iFBStatus_B = ENABLE_WRITE;
#if (TIMEOUT_MSLEEP == 0)
			wake_up_interruptible(
			 &fb_data[fb_num].emxx_fb_dev->wait_fb_chk_scrn);
#endif
		}
	} else {
		fb_data[fb_num].iNowDispPage = FB_NO_B;
		fb_data[fb_num].iFBStatus_B  = ALREADY_DISPLAY;
		if (fb_data[fb_num].iNextDispPage != FB_NO_A) {
			fb_data[fb_num].iFBStatus_A = ENABLE_WRITE;
#if (TIMEOUT_MSLEEP == 0)
			wake_up_interruptible(
			 &fb_data[fb_num].emxx_fb_dev->wait_fb_chk_scrn);
#endif
		}
	}
	printk_dbg((_DEBUG_FB & 0x08), "<] callback:    A(%d) B(%d)\n",
	 fb_data[fb_num].iFBStatus_A, fb_data[fb_num].iFBStatus_B);

	/* start timer */
	if ((fb_data[fb_num].iUpdateTimerFlag == TIMER_ON)
	 && (fb_data[fb_num].iMixModeFlg == ONLY_2D_MODE)
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	 && !fb_data[fb_num].iDpmSuspendFlag
#endif /* CONFIG_PM || CONFIG_DPM */
	) {
		printk_dbg((_DEBUG_FB & 0x10), "resume: callback: add_timer\n");
		fb_data[fb_num].emxx_fb_timer.expires = jiffies
		 + (fb_data[fb_num].iTimeoutTime * HZ / 1000);
		add_timer(&fb_data[fb_num].emxx_fb_timer);
		fb_data[fb_num].iTimerStatusFlag = FB_TIMER_START;
	}
}
EXPORT_SYMBOL(emxx_fb_callback);


/********************************************************
 *  Init Function Definitions                           *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_fb_init_module
* FUNCTION : frame buffer driver init function
* RETURN   : 0      : success
*            -ENXIO : fail
* NOTE     : none
******************************************************************************/
int __init emxx_fb_init_module(void)
{
	int   iRet   = 0;

	iRet = platform_driver_register(&emxx_fb_driver);
	if (!iRet)
		return iRet;

	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_probe
* FUNCTION : frame buffer driver probe function
* RETURN   : 0       : success
*            -ENOMEM : mem error
*            -EBUSY  : other driver init error
* NOTE     : none
******************************************************************************/
static int __init emxx_fb_probe(struct platform_device *dev)
{
	struct fb_info         *info = NULL;
	struct fb_info         *info_0 = NULL;
	struct fb_info         *info_1 = NULL;
	emxx_fb_par           *par  = NULL;
	int                     iRet = -ENOMEM;
	int                     fb_num = 0;

	/* reserve region Smem */
	if (!request_mem_region(Smem, SmemLength, "qfb")) {
		printk_wrn("cannot reserve smem\n");
		emxx_fb_probe_error(FB_INIT_ERROR__SMEM_NOT_RESERVE, info_0,
		 info_1, fb_num);
		return -ENOMEM;
	}

	/* remap SmemV */
	SmemV = ioremap_wc(Smem, SmemLength);
	if (!SmemV) {
		printk_wrn("cannot ioremap_wc smem\n");
		emxx_fb_probe_error(FB_INIT_ERROR__SMEM_NOT_REMAP, info_0,
		 info_1, fb_num);
		return -ENOMEM;
	}
	printk_dbg((_DEBUG_FB),
	 " Smem    (0x%lx)  SmemV    (0x%p)  SmemLength    (0x%lx)\n",
	 Smem, SmemV, SmemLength);

	for (fb_num = 0; fb_num < EMXX_FB_DEVICES; fb_num++) {
		/* alloc framebuffer */
		info = framebuffer_alloc(sizeof(emxx_fb_par), &dev->dev);
		if (!info) {
			printk_wrn("cannot framebuffer alloc\n");
			emxx_fb_probe_error(FB_INIT_ERROR__FB_NOT_ALLOC,
			 info_0, info_1, fb_num);
			return -ENOMEM;
		} else {
			if (fb_num == 0)
				info_0 = info;
			else
				info_1 = info;
		}

		/* flag initialize */
		set_flag_init(fb_num);

		/* set buffer address */
		set_buffer_address(fb_num);

		/* set infomation */
		set_info(info, &par, fb_num);

		/* device flag set */
		par->devflag = fb_num;

		/* alloc cmap */
		iRet = fb_alloc_cmap(&info->cmap, 256, 0);
		if (iRet < 0) {
			printk_wrn("cannot framebuffer alloc cmap\n");
			emxx_fb_probe_error(FB_INIT_ERROR__FB_CMAP_NOT_ALLOC,
			 info_0, info_1, fb_num);
			return -ENOMEM;
		}

		/* regist frame buffer */
		iRet = register_framebuffer(info);
		if (iRet < 0) {
			printk_wrn("cannot regist framebuffer\n");
			emxx_fb_probe_error(FB_INIT_ERROR__FB_NOT_REGIST,
			 info_0, info_1, fb_num);
			return -EBUSY;
		}

		/* spinlock initialize */
		spin_lock_init(&par->fb_lock);

		/* semaphore initialize */
		sema_init(&par->sem_fb_image_info, 1);
		sema_init(&par->sem_fb_other_info_A, 1);
		sema_init(&par->sem_fb_other_info_B, 1);
		sema_init(&par->sem_outputmode, 1);
		sema_init(&par->sem_copybit, 1);

#if (TIMEOUT_MSLEEP == 0)
		/* wait_event initialize */
		init_waitqueue_head(&par->wait_fb_chk_scrn);
#endif

		/* workqueue initialize */
		par->emxx_fb_workqueue =
		 create_singlethread_workqueue(DEV_NAME);
		INIT_WORK(&par->wk_timeout_update_bottom_half,
		 timeout_update_bottom_half_do);

		/* output mode initialize */
		par->output_mode = EMXX_FB_OUTPUT_MODE_LCD;

		/* variable initialize */
		set_val_init(info, fb_num);

		/* set drvdata */
		drvdata.info[fb_num] = info;

		/* for callback function */
		fb_data[fb_num].emxx_fb_dev = par;
	}
	fb_num--;

	/*initialize LCD driver */
	if (emxx_lcd_init_module() != 0) {
		printk_wrn("cannot initialize LCD\n");
		emxx_fb_probe_error(FB_INIT_ERROR__LCD_NOT_INITIALIZE,
		 info_0, info_1, fb_num);
		return -EBUSY;
	}

	/* set drvdate */
	dev_set_drvdata(&dev->dev, &drvdata);

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#ifdef CONFIG_HAS_EARLYSUSPEND
	drvdata.dev = dev;
	drvdata.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	drvdata.early_suspend.suspend = emxx_fb_early_suspend;
	drvdata.early_suspend.resume = emxx_fb_late_resume;
	register_early_suspend(&drvdata.early_suspend);
#endif
#endif /* CONFIG_PM || CONFIG_DPM */

#ifdef CONFIG_EMXX_ANDROID
	emxx_fb_init_blit();
#endif

	printk_info("registered device fb\n");
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_probe_error
* FUNCTION : When probe error, cleanup
* RETURN   : none
* NOTE     :
******************************************************************************/
static void __init emxx_fb_probe_error(int iErrorMode,
 struct fb_info *info_0, struct fb_info *info_1, int fb_num)
{
	if (fb_num > 0) {
		switch (iErrorMode) {
		case FB_INIT_ERROR__LCD_NOT_INITIALIZE:
			unregister_framebuffer(info_1);
		case FB_INIT_ERROR__FB_NOT_REGIST:
			fb_dealloc_cmap(&info_1->cmap);
		case FB_INIT_ERROR__FB_CMAP_NOT_ALLOC:
			framebuffer_release(info_1);
		case FB_INIT_ERROR__FB_NOT_ALLOC:
		default:
			break;
		}
		unregister_framebuffer(info_0);
		fb_dealloc_cmap(&info_0->cmap);
		framebuffer_release(info_0);
		iounmap(SmemV);
		release_mem_region(Smem, DispBufAllLength);
	} else {
		switch (iErrorMode) {
		case FB_INIT_ERROR__LCD_NOT_INITIALIZE:
			unregister_framebuffer(info_0);
		case FB_INIT_ERROR__FB_NOT_REGIST:
			fb_dealloc_cmap(&info_0->cmap);
		case FB_INIT_ERROR__FB_CMAP_NOT_ALLOC:
			framebuffer_release(info_0);
		case FB_INIT_ERROR__FB_NOT_ALLOC:
			iounmap(SmemV);
		case FB_INIT_ERROR__SMEM_NOT_REMAP:
			release_mem_region(Smem, DispBufAllLength);
		case FB_INIT_ERROR__SMEM_NOT_RESERVE:
		default:
			break;
		}
	}
}


/******************************************************************************
* MODULE   : set_buffer_address
* FUNCTION : set frame buffer (A/B) address
* RETURN   : none
* NOTE     :
******************************************************************************/
static void set_buffer_address(int fb_num)
{
	/* set & clean DispBufA(2D) address */
	fb_data[fb_num].DispBufAV = SmemV + DISPBUF_A_OFFSET;
	memset((void *)fb_data[fb_num].DispBufAV, 0,
	 fb_data[fb_num].DispBufLength);

	/* set & clean DispBufB(2D) address */
	fb_data[fb_num].DispBufBV = SmemV + DISPBUF_B_OFFSET;
	memset((void *)fb_data[fb_num].DispBufBV, 0,
	 fb_data[fb_num].DispBufLength);

#if _DEBUG_FB
	{
		unsigned short *p;
		int i;
		p = (unsigned short *)fb_data[fb_num].DispBufAV;
		for (i = 0; i < 800; i++) {
			*p = 0xF800;
			p++;
		}
		for (i = 0; i < 800; i++) {
			*p = 0x07E0;
			p++;
		}
		for (i = 0; i < 800; i++) {
			*p = 0x001F;
			p++;
		}
		for (i = 0; i < 800*(100-3) ; i++) {
			*p = 0xF800;
			p++;
		}
		for (i = 0; i < 800*100; i++) {
			*p = 0x07E0;
			p++;
		}
		for (i = 0; i < 800*100; i++) {
			*p = 0x001F;
			p++;
		}
		for (i = 0; i < 800*100; i++) {
			*p = 0x0000;
			p++;
		}
		for (i = 0; i < 800*80; i++) {
			*p = 0xFFFF;
			p++;
		}
		p = (unsigned short *)fb_data[fb_num].DispBufBV;
		for (i = 0; i < 800*160; i++) {
			*p = 0xF800;
			p++;
		}
		for (i = 0; i < 800*160; i++) {
			*p = 0x07E0;
			p++;
		}
		for (i = 0; i < 800*160; i++) {
			*p = 0x001F;
			p++;
		}
	}
#endif
}


/******************************************************************************
* MODULE   : set_info
* FUNCTION : set frame buffer driver infomation
* RETURN   : none
* NOTE     :
******************************************************************************/
static void set_info(struct fb_info *info, emxx_fb_par **par, int fb_num)
{
	*par                  = (emxx_fb_par *)info->par;
	(*par)->fb_inf        = info;
#ifdef CONFIG_FB_EMXX_PANDISP
	(*par)->old_offset    = -1;
#endif	/* CONFIG_FB_EMXX_PANDISP */

	info->screen_base     = (char __iomem *)fb_data[fb_num].DispBufAV;
#ifdef CONFIG_FB_EMXX_PANDISP
	info->screen_size     = fb_data[fb_num].DispBufLength * 2;
#else	/* CONFIG_FB_EMXX_PANDISP */
	info->screen_size     = fb_data[fb_num].DispBufLength;
#endif	/* CONFIG_FB_EMXX_PANDISP */
	info->fbops           = &emxx_fb_ops;

	info->var             = emxx_fb_default;
#ifdef CONFIG_FB_EMXX_ABGR8888
 #ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
		info->var.red.offset    = 16;
		info->var.green.offset  = 8;
		info->var.blue.offset   = 0;
		info->var.transp.offset = 24;
	}
 #endif
#endif

	info->fix             = emxx_fb_fix;
	info->fix.line_length = info->var.xres_virtual
	 * info->var.bits_per_pixel / 8;
	info->fix.smem_start  = fb_data[fb_num].DispBufA;
#ifdef CONFIG_FB_EMXX_PANDISP
	info->fix.smem_len    = DISPBUF_LENGTH * 2;
#else	/* CONFIG_FB_EMXX_PANDISP */
	info->fix.smem_len    = fb_data[fb_num].DispBufLength;
#endif	/* CONFIG_FB_EMXX_PANDISP */
	info->pseudo_palette  = (*par)->pseudo_palette;
	info->flags           = FBINFO_FLAG_DEFAULT;
}


/******************************************************************************
* MODULE   : set_flag_init
* FUNCTION : flag initialize
* RETURN   : none
* NOTE     :
******************************************************************************/
static void set_flag_init(int fb_num)
{
	fb_data[fb_num].iFBStatus_A      = IS_NODATA;
	fb_data[fb_num].iFBStatus_B      = IS_NODATA;
	printk_dbg((_DEBUG_FB & 0x08), "A(%d) B(%d)\n",
	 fb_data[fb_num].iFBStatus_A, fb_data[fb_num].iFBStatus_B);
	fb_data[fb_num].iNowDispPage     = DISP_BUFA;
	fb_data[fb_num].iNextDispPage    = DISP_BUFA;

	fb_data[fb_num].iUpdateTimerFlag = DEFALT_UPDATE_TIMER_LCD;
	fb_data[fb_num].DispBufA = SMEM_START + DISPBUF_A_OFFSET;
	fb_data[fb_num].DispBufB = SMEM_START + DISPBUF_B_OFFSET;
	fb_data[fb_num].DispBufLength = DISPBUF_LENGTH;

	fb_data[fb_num].iMixModeFlg      = ONLY_2D_MODE;
	fb_data[fb_num].iDpmSuspendFlag  = 0;

	fb_data[fb_num].is_first         = 1;

}


/******************************************************************************
* MODULE   : set_val_init
* FUNCTION : variable initialize
* RETURN   : none
* NOTE     :
******************************************************************************/
static void set_val_init(struct fb_info *info, int fb_num)
{
	/* fb A val init */
	fb_data[fb_num].fb_other_info_A.iMaskColFlg = TC_COLOR_DISABLE;
#if defined(CONFIG_FB_EMXX_ARGB8888) || defined(CONFIG_FB_EMXX_BGR888)
	fb_data[fb_num].fb_other_info_A.iMaskCol    = 0x000100;
#else
	fb_data[fb_num].fb_other_info_A.iMaskCol    = 0x0020;
#endif
	fb_data[fb_num].fb_other_info_A.iAlpha      = 0xFF;
	fb_data[fb_num].fb_other_info_A.iInvFlg     = NO_INVERSE;
	fb_data[fb_num].fb_other_info_A.iPage       = DISP_BUFA;
	fb_data[fb_num].fb_other_info_A.iUpDate     = UPDATE_FLAG_OFF;

	/* fb B val init */
	fb_data[fb_num].fb_other_info_B.iMaskColFlg = TC_COLOR_DISABLE;
#if defined(CONFIG_FB_EMXX_ARGB8888) || defined(CONFIG_FB_EMXX_BGR888)
	fb_data[fb_num].fb_other_info_B.iMaskCol    = 0x000100;
#else
	fb_data[fb_num].fb_other_info_B.iMaskCol    = 0x0020;
#endif
	fb_data[fb_num].fb_other_info_B.iAlpha      = 0xFF;
	fb_data[fb_num].fb_other_info_B.iInvFlg     = NO_INVERSE;
	fb_data[fb_num].fb_other_info_B.iPage       = DISP_BUFB;
	fb_data[fb_num].fb_other_info_B.iUpDate     = UPDATE_FLAG_OFF;

	/* fb data init (to LCD) */
	fb_data[fb_num].fb_image_info.image_data.hsize    = info->var.xres;
	fb_data[fb_num].fb_image_info.image_data.vsize    = info->var.yres;
	fb_data[fb_num].fb_image_info.image_data.size     =
		info->var.xres_virtual * info->var.bits_per_pixel / 8;
	fb_data[fb_num].fb_image_info.image_data.yrgbaddr =
		fb_data[fb_num].DispBufA;
	fb_data[fb_num].fb_image_info.image_data.uvaddr   = 0x00000000;
	fb_data[fb_num].fb_image_info.image_data.x        = 0;
	fb_data[fb_num].fb_image_info.image_data.y        = 0;

	/* fb info init (to LCD) */
	fb_data[fb_num].fb_image_info.maskcolrflg  = FB_MASK_COLOR_DISP_OFF;
#if defined(CONFIG_FB_EMXX_ARGB8888) || defined(CONFIG_FB_EMXX_BGR888)
	fb_data[fb_num].fb_image_info.maskcolr     = 0x000100;
#else
	fb_data[fb_num].fb_image_info.maskcolr     = 0x0020;
#endif
	fb_data[fb_num].fb_image_info.alpha        = 0xFF;
	fb_data[fb_num].fb_image_info.invflg       = NO_INVERSE;
	fb_data[fb_num].fb_image_info.mix_buf_page = DISP_BUFA;
	fb_data[fb_num].fb_image_info.update_flag  = FB_UPDATE_OFF;

	/* timer */
	fb_data[fb_num].iTimeoutTime               = TIMEOUT_TIME;
}


/********************************************************
 *  Timer Function Definitions                          *
 *******************************************************/
/******************************************************************************
* MODULE   : timer_init
* FUNCTION : call back from LCD driver
* RETURN   : none
* NOTE     : public function
******************************************************************************/
static void timer_init(struct fb_info *info)
{
	emxx_fb_par *par = (emxx_fb_par *)info->par;

	init_timer(&fb_data[par->devflag].emxx_fb_timer);
	fb_data[par->devflag].emxx_fb_timer.function = timeout_update;
	fb_data[par->devflag].emxx_fb_timer.entry.next = NULL;
	fb_data[par->devflag].emxx_fb_timer.data = (unsigned long)info;
	down(&par->sem_outputmode);
	if ((fb_data[par->devflag].iUpdateTimerFlag == TIMER_ON)
	 && (fb_data[par->devflag].iMixModeFlg == ONLY_2D_MODE)) {
		printk_dbg((_DEBUG_FB & 0x10), "add_timer\n");
		fb_data[par->devflag].emxx_fb_timer.expires = jiffies
		 + (TIMEOUT_TIME * HZ / 1000);
		add_timer(&fb_data[par->devflag].emxx_fb_timer);
		fb_data[par->devflag].iTimerStatusFlag = FB_TIMER_START;
		printk_dbg((_DEBUG_FB & 0x01), "%d <start>\n",
		 __LINE__);
	}
	up(&par->sem_outputmode);
}


/******************************************************************************
* MODULE   : timeout_update
* FUNCTION : when Timer timeout, update execution
* RETURN   : none
* NOTE     : public function
******************************************************************************/
static void timeout_update(unsigned long data)
{
	unsigned long        flags;
	struct fb_info      *info;
	emxx_fb_par *par;

	printk_dbg((_DEBUG_FB & 0x10), "\n");

	info = (struct fb_info *)data;
	par  = (emxx_fb_par *)info->par;

	/* fb timer stop */
	spin_lock_irqsave(&fb_data[par->devflag].emxx_fb_dev->fb_lock, flags);
	del_timer(&fb_data[par->devflag].emxx_fb_timer);
	fb_data[par->devflag].iTimerStatusFlag = FB_TIMER_STOP;
	spin_unlock_irqrestore(&fb_data[par->devflag].emxx_fb_dev->fb_lock,
			flags);

	printk_dbg((_DEBUG_FB & 0x10), "queue_work\n");
	queue_work(par->emxx_fb_workqueue,
	 &par->wk_timeout_update_bottom_half);

	/* start timer */
	if ((fb_data[par->devflag].iUpdateTimerFlag == TIMER_ON)
	 && (fb_data[par->devflag].iMixModeFlg == ONLY_2D_MODE)) {
		spin_lock_irqsave(&fb_data[par->devflag].emxx_fb_dev->fb_lock,
				flags);
		if (fb_data[par->devflag].iTimerStatusFlag == FB_TIMER_STOP) {
			printk_dbg((_DEBUG_FB & 0x10), "add_timer\n");
			fb_data[par->devflag].emxx_fb_timer.expires =
				jiffies + (fb_data[par->devflag].iTimeoutTime *
				HZ / 1000);
			add_timer(&fb_data[par->devflag].emxx_fb_timer);
			fb_data[par->devflag].iTimerStatusFlag = FB_TIMER_START;
		}
		spin_unlock_irqrestore(
			&fb_data[par->devflag].emxx_fb_dev->fb_lock, flags);
	}
}


/******************************************************************************
* MODULE   : timeout_update_bottom_half_do
* FUNCTION : when Timer timeout, update execution
* RETURN   : none
* NOTE     : private function
******************************************************************************/
static void timeout_update_bottom_half_do(struct work_struct *num)
{
	emxx_fb_par *par;
	struct fb_info *info;

	printk_dbg((_DEBUG_FB & 0x10), "\n");

	par = container_of(num, emxx_fb_par, wk_timeout_update_bottom_half);
	info = par->fb_inf;
	if (down_trylock(&par->sem_fb_image_info))	/* lock fb_image_info */
		return;

	/* set image infomation to LCD */
	set_image_info_to_LCD(fb_data[par->devflag].iNextDispPage, info);

	/* udpate on (normal EMXX_FB_UPDATE_SCRN exe) */
	fb_data[par->devflag].fb_image_info.update_flag = FB_UPDATE_ON;

	fb_data[par->devflag].fb_image_info.output_mode = par->output_mode;

	/* call fb function */
	printk_dbg((_DEBUG_FB & 0x10), "request LCD\n");
	emxx_lcd_set_fb_image(&fb_data[par->devflag].fb_image_info);

	up(&par->sem_fb_image_info);		/* unlock fb_image_info */
}


/********************************************************
 *  Exit Function Definitions                           *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_fb_remove
* FUNCTION : remove
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int emxx_fb_remove(struct platform_device *device)
{
	struct emxx_fb_drvdata *drvdata = platform_get_drvdata(device);
	struct fb_info *info;
	int i;

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&drvdata->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM || CONFIG_DPM */

	for (i = 0; i < EMXX_FB_DEVICES; i++) {
		/* remove frame buffer */
		info = drvdata->info[i];
		if (info) {
			unregister_framebuffer(info);
			fb_dealloc_cmap(&info->cmap);
			framebuffer_release(info);
		}
	}

	/* lcd remove */
	emxx_lcd_exit_module();

	/* remove Smem */
	iounmap(SmemV);
	release_mem_region(Smem, SmemLength);

	return 0;
}


#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/********************************************************
 *  Suspend/Resume Function Definitions                 *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_fb_suspend
* FUNCTION : suspend frame buffer driver
* RETURN   :  0 : success
*          : -1 : faile
* NOTE     : none
******************************************************************************/
static int emxx_fb_suspend(struct platform_device *dev, pm_message_t state)
{
	/*** this module suspend ***/
	if (!fb_data[EMXX_FB_DEVICE_LCD].is_first) {
		/*** this module suspend ***/
		/* call fb suspend function  */
		if (emxx_fb_suspend_sub(dev, state, EMXX_FB_DEVICE_LCD))
			goto fail_fb_lcd;

		/*** relational module suspend ***/
		/* call LCD suspend function */
		if (emxx_lcd_suspend(dev, state))
			goto fail_lcd;

#ifdef CONFIG_EMXX_IMC
		/* call IMC suspend function */
		if (emxx_imc_suspend(dev, state))
			goto fail_imc;
#endif

#ifdef CONFIG_EMXX_NTS
		/* call NTS suspend function */
		if (emxx_nts_suspend(dev, state))
			goto fail_nts;
#endif

	}
	return 0;

#ifdef CONFIG_EMXX_NTS
fail_nts:
	emxx_imc_resume(dev);
#endif
#ifdef CONFIG_EMXX_IMC
fail_imc:
	emxx_lcd_resume(dev);
#endif
fail_lcd:
	emxx_fb_resume_sub(dev, EMXX_FB_DEVICE_LCD);
fail_fb_lcd:
	return -EBUSY;
}


/******************************************************************************
* MODULE   : emxx_fb_suspend_sub
* FUNCTION : suspend frame buffer driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int emxx_fb_suspend_sub(struct platform_device *dev,
 pm_message_t state, int fb_num)
{
	int ret;

	/*** this module suspend ***/
	fb_data[fb_num].iDpmSuspendFlag = 1;
	/* stop timer interrupt */
	del_timer(&fb_data[fb_num].emxx_fb_timer);
	ret = 0;
	return ret;
}


/******************************************************************************
* MODULE   : emxx_fb_resume
* FUNCTION : resume frame buffer driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int emxx_fb_resume(struct platform_device *dev)
{
	if (!fb_data[EMXX_FB_DEVICE_LCD].is_first) {
		/*** relathional module resume ***/
#ifdef CONFIG_EMXX_IMC
		/* call IMC resume function */
		emxx_imc_resume(dev);
#endif
		/* call LCD resume function */
		emxx_lcd_resume(dev);
		/* call fb  resume function */
		emxx_fb_resume_sub(dev, EMXX_FB_DEVICE_LCD);
	}
#ifdef CONFIG_EMXX_NTS
	/* call NTS resume function */
	emxx_nts_resume(dev);
#endif
	return 0;
}


/******************************************************************************
* MODULE   : emxx_fb_resume_sub
* FUNCTION : resume frame buffer driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int emxx_fb_resume_sub(struct platform_device *dev, int fb_num)
{
	struct emxx_fb_drvdata *drvdata = platform_get_drvdata(dev);
	struct fb_info *info;
	emxx_fb_par *par;

	if (fb_data[fb_num].iDpmSuspendFlag) {
		/*** this module resume ***/
		fb_data[fb_num].iDpmSuspendFlag = 0;

		if (fb_data[fb_num].iMixModeFlg == ONLY_2D_MODE) {
			if (fb_data[fb_num].iUpdateTimerFlag ==
				TIMER_ON) {
				printk_dbg((_DEBUG_FB & 0x10),
				 "add_timer\n");
				/* start timer interrupt */
				add_timer(
				&fb_data[fb_num].emxx_fb_timer);
			} else {
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
				if (!direct_path) {
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
					info = drvdata->info[fb_num];
					par  = (
					emxx_fb_par *)info->par;

					printk_dbg((_DEBUG_FB & 0x10),
					 "queue_work\n");
					queue_work(
					par->emxx_fb_workqueue,
					&par->wk_timeout_update_bottom_half);
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
				}
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
			}
		}
	}
	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void emxx_fb_early_suspend(struct early_suspend *h)
{
	struct emxx_fb_drvdata *drvdata;
	pm_message_t state;
	drvdata = container_of(h, struct emxx_fb_drvdata, early_suspend);
	state.event = PM_EVENT_SUSPEND;
	emxx_fb_suspend(drvdata->dev, state);
}


static void emxx_fb_late_resume(struct early_suspend *h)
{
	struct emxx_fb_drvdata *drvdata;
	drvdata = container_of(h, struct emxx_fb_drvdata, early_suspend);
	emxx_fb_resume(drvdata->dev);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM || CONFIG_DPM */


MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("EMMA Mobile EV framebuffer driver");
MODULE_LICENSE("GPL");


/* function alias  */
module_init(emxx_fb_init_module);


