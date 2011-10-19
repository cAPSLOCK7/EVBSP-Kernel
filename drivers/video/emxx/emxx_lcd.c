/*
 * File Name       : drivers/video/emxx/emxx_lcd.c
 * Function        : LCD Driver
 * Release Version : Ver 1.25
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
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/system.h>

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <mach/pm.h>
#endif /* CONFIG_PM || CONFIG_DPM */

#include <linux/uaccess.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/pmu.h>
#include <mach/emxx_mem.h>

#include "emxx_common.h"
#include "emxx_fb.h"

#include "lcdc.h"
#include "emxx_lcd_common.h"
#include "emxx_lcdhw.h"
#include "emxx_lcd.h"

#include <mach/imc.h>
#include <mach/emxx_imc.h>
#include "../../imc/emxx_imc.h"


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define _DEBUG_LCD  0x00 /* 00008421(bit) */
			 /* 0x01: debug function in
			  * 0x02: debug function out
			  * 0x10: debug INTERLACE TB/BT
			  * 0x40: debug FBIOBLANK
			  * 0x80: debug semafore
			  */


#define DEV_NAME "emxx_lcd"
#define DPM_SUSPEND_FLG_INIT 0
#define INDEX_LCD_VAR_NEXT_INIT 0


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

#if _DEBUG_LCD
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
 *  Structure                                           *
 *******************************************************/
/* structure lcd device status */
struct emxx_lcd_dev {
	/* semafore v4l2/fb/img local data */
	struct semaphore      sem_image_data;
	/* semafore IMC refresh */
	struct semaphore      sem_image_refresh;
	/* spin lock LCD timer flag        */
	spinlock_t	      lcd_lock;
};


/********************************************************
 *  Variables                                           *
 *******************************************************/

/* input image data from fb driver */
/* (2D) name of mix frame buffer temp   */
static  unsigned int  uiMixFrameBufferPage_tmp;

/* other img infomation */
static  unsigned int  uiMaskColor_tmp;		/* mask color */
static  unsigned int  uiAlpha_tmp;		/* alpha      */

/* Flags */
static           int  iMixDSPFlg_tmp;		/* DSP mix ON/OFF            */
static           int  iDSPPauseFlg;		/* DSP pause status          */
static  unsigned int  uiMaskColorFlag_tmp;	/* mask color ON/OFF Flag    */
static  unsigned int  uiAbsolutelyUpFlag;	/* absolutely update flag    */
	unsigned int  uiInverseFlag_tmp;	/* inverse mode Flag         */

/*
 * Smem
 */
/* LCDC MMIO */
static  unsigned long LCDCMmio = EMXX_LCD_BASE;
	char         *LCDCMmioV;

/* LCD device status */
static struct emxx_lcd_dev lcd_dev;

/*
 * V4L2 dev data
 */
static void *pvDevData_tmp;		/* emxx_v4l2_device                */
static void *pvVideoBuf_tmp;		/* videobuf_buffer                 */

/* V4L2 field */
       unsigned int v4l2_field;

/*
 * timer
 */
static  struct timer_list lcd_timer;
static  int               iTimerFlag;
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
static  struct timer_list lcd_timer_framecache;
static  int               iTimerFlag_framecache;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

/* lcd_refresh_flg */
enum imc_refresh_status {
	IMC_R_IDLE         = 0,
	IMC_R_WAIT_REFRESH = 1,
	IMC_R_REFRESHED    = 2,
	IMC_R_DISPLAYED    = 3,
};

struct emxx_lcd_var {
	struct list_head      list;
	/* semafore v4l2/fb/img local flag */
	struct semaphore      sem_image_flag;

	/* input image data from fb driver */
	/* (2D) name of mix frame buffer        */
	unsigned int  uiMixFrameBufferPage;
	/* Flags */
	int  iMixDSPFlgToFB;		/* return FB DSP mix ON/OFF  */
	int  iCallbackV4L2Flg;		/* need to callback to V4L2  */
	/* V4L2 dev data */
	void *pvDevData;		/* emxx_v4l2_device                */
	void *pvVideoBuf;		/* videobuf_buffer                 */

	enum imc_refresh_status lcd_refresh_flg;

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	int          ctrl_func;
#endif /* CONFIG_PM || CONFIG_DPM */
};
#define EMXX_LCD_VAR struct emxx_lcd_var

static EMXX_LCD_VAR lcd_var[2];
static int index_lcd_var_next = INDEX_LCD_VAR_NEXT_INIT;

static struct list_head list_lcd_var;

	LCD_BLANK_STATE blank_state;

#ifdef CONFIG_EMXX_ANDROID
extern int v4l2_open_flag;
wait_queue_head_t v4l2_close_q;
#endif

/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
/*
 * LCD Driver
 */
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
static int          lcd_suspend_chk_lcdout(void);

       int          IMC_reset_flg   = 0;
#endif /* CONFIG_PM || CONFIG_DPM */
static int          DPM_suspend_flg = DPM_SUSPEND_FLG_INIT;
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
       unsigned long save_ckrqmode;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

/* Private function */
static void         mix_image(int iCallbackV4L2_tmp, int update, int type);
static void         lcd_irq_handler_callback(void);

static void         set_val_init(void);
static void         lcd_probe(void);
static void         lcd_timer_init(void);
static void         lcd_timeout(unsigned long data);
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
static void         lcd_timeout_framecache(unsigned long data);
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */


/*
 * work queue
 */
static struct workqueue_struct *emxx_lcd_workqueue;
static struct work_struct       wk_timeout_bottom_half;
static void         lcd_timeout_bottom_half_do(struct work_struct *num);


/******/
#include "emxx_lcd_perf.h"


/********************************************************
 *  Function Definitions                                *
 *******************************************************/

/******************************************************************************
* MODULE   : emxx_lcd_blank
* FUNCTION : FBIOBLANK
* RETURN   : 0       : success
*            -EINVAL : input value incorrect
* NOTE     : none
******************************************************************************/
int emxx_lcd_blank(int blank_mode)
{
	int backlight, output, clock;
	int lock = 0;
	unsigned long flags = 0;

	printk_dbg((_DEBUG_LCD & 0x40), "<start>\n");

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		/* Screen: On,  HSync: On,  VSync: On */
		printk_dbg((_DEBUG_LCD & 0x40), "<FB_BLANK_UNBLANK>\n");
		backlight = 1;
		output    = 1;
		clock     = 1;
		break;
	case FB_BLANK_NORMAL:
		/* Screen: Off, HSync: On,  VSync: On */
		printk_dbg((_DEBUG_LCD & 0x40), "<FB_BLANK_NORMAL>\n");
		backlight = 0;
		output    = 1;
		clock     = 1;
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		/* Screen: Off, HSync: On,  VSync: Off */
		printk_dbg((_DEBUG_LCD & 0x40), "<FB_BLANK_VSYNC_SUSPEND>\n");
		backlight = 0;
		output    = 0;
		clock     = 1;
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		/* Screen: Off, HSync: Off, VSync: On */
		printk_dbg((_DEBUG_LCD & 0x40), "<FB_BLANK_HSYNC_SUSPEND>\n");
		backlight = 0;
		output    = 0;
		clock     = 1;
		break;
	case FB_BLANK_POWERDOWN:
		/* Screen: Off, HSync: Off, VSync: Off */
		printk_dbg((_DEBUG_LCD & 0x40), "<FB_BLANK_POWERDOWN>\n");
		backlight = 0;
		output    = 0;
		clock     = 0;
		break;
	default:
		printk_dbg((_DEBUG_LCD & 0x40), "<failed(%d)>\n", -EINVAL);
		return -EINVAL;
	}

	if ((lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) &&
	    (backlight == 0) && (blank_state.lcd_backlight == 1)) {
		/* backlight On -> Off */
		printk_dbg((_DEBUG_LCD & 0x40), "<backlight On -> Off>\n");
		blank_state.lcd_backlight = 0;
		lcd_hw_backlight_off();
	}

	if (((output    == 0) && (blank_state.lcd_output    == 1)) ||
	    ((output    == 1) && (blank_state.lcd_output    == 0)) ||
	    ((lcdc_output_mode != EMXX_FB_OUTPUT_MODE_LCD) &&
	     (((backlight == 0) && (blank_state.lcd_backlight == 1)) ||
	      ((backlight == 1) && (blank_state.lcd_backlight == 0))))) {
		down(&lcd_dev.sem_image_data);
		down(&lcd_dev.sem_image_refresh);
		down(&lcd_var[0].sem_image_flag);
		down(&lcd_var[1].sem_image_flag);
		spin_lock_irqsave(&lcd_dev.lcd_lock, flags);
		lock = 1;
	}

	blank_state.current_mode = blank_mode;

	if ((lcdc_output_mode != EMXX_FB_OUTPUT_MODE_LCD) &&
	    (backlight == 0) && (blank_state.lcd_backlight == 1)) {
		/* backlight On -> Off */
		printk_dbg((_DEBUG_LCD & 0x40), "<backlight On -> Off>\n");
		blank_state.lcd_backlight = 0;
		writel(LCD_BUSSEL_BLACK, LCDCMmioV + LCD_BUSSEL);
		lcd_suspend_chk_lcdout();
	}

	if ((output    == 0) && (blank_state.lcd_output    == 1)) {
		/* output    On -> Off */
		printk_dbg((_DEBUG_LCD & 0x40), "<output    On -> Off>\n");
		blank_state.lcd_output    = 0;
		writel(LCD_BUSSEL_BLACK, LCDCMmioV + LCD_BUSSEL);
		lcd_suspend_chk_lcdout();
		lcd_module_hw_standby();
		lcd_hw_stop();
	}
	if ((clock     == 0) && (blank_state.lcd_clock     == 1)) {
		/* clock     On -> Off */
		printk_dbg((_DEBUG_LCD & 0x40), "<clock     On -> Off>\n");
		blank_state.lcd_clock     = 0;
		lcd_hw_save_reg();
		lcd_hw_reset();
	}
	if ((clock     == 1) && (blank_state.lcd_clock     == 0)) {
		/* clock     Off -> On */
		printk_dbg((_DEBUG_LCD & 0x40), "<clock     Off -> On>\n");
		blank_state.lcd_clock     = 1;
		lcd_hw_unreset();
		lcd_hw_restore_reg();	/* LCD_BUSSEL_BLACK is
					   set to LCD_BUSSEL*/
	}
	if ((output    == 1) && (blank_state.lcd_output    == 0)) {
		/* output    Off -> on */
		printk_dbg((_DEBUG_LCD & 0x40), "<output    Off -> On>\n");
		blank_state.lcd_output    = 1;
		writel(LCD_BUSSEL_LOCAL, LCDCMmioV + LCD_BUSSEL);
		lcd_hw_start();
		lcd_module_hw_wakeup();
	}

	if ((lcdc_output_mode != EMXX_FB_OUTPUT_MODE_LCD) &&
	    (backlight == 1) && (blank_state.lcd_backlight == 0)) {
		/* backlight Off -> On */
		printk_dbg((_DEBUG_LCD & 0x40), "<backlight Off -> On>\n");
		blank_state.lcd_backlight = 1;
		writel(LCD_BUSSEL_LOCAL, LCDCMmioV + LCD_BUSSEL);
	}

	if (lock == 1) {
		spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);
		up(&lcd_dev.sem_image_data);
		up(&lcd_dev.sem_image_refresh);
		up(&lcd_var[0].sem_image_flag);
		up(&lcd_var[1].sem_image_flag);
	}

	if ((lcdc_output_mode == EMXX_FB_OUTPUT_MODE_LCD) &&
	    (backlight == 1) && (blank_state.lcd_backlight == 0)) {
		/* backlight Off -> On */
		printk_dbg((_DEBUG_LCD & 0x40), "<backlight Off -> On>\n");
		blank_state.lcd_backlight = 1;
		lcd_hw_backlight_on();
	}

	return 0;
}
EXPORT_SYMBOL(emxx_lcd_blank);


/******************************************************************************
* MODULE   : emxx_lcd_set_v4l2_image
* FUNCTION : set image data from v4l2 to LCD local
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int emxx_lcd_set_v4l2_image(V4L2_IMAGE_INFO *v4l2_image_info)
{
	unsigned long flags;

	int bpp_y, bpp_uv, bpp_v; /* bit par pixel  */
	int ppl_y, ppl_uv, ppl_v; /* pixel par line */

#if LCD_PERF
if (!start_perf) {
	printk(KERN_INFO "log_start!\n");
	map_nowtime();
	cnt_time = 0;
	start_perf = 1;
}
set_nowtime(SEQ_V4L2);
#endif /* LCD_PERF */
	printk_dbg((_DEBUG_LCD & 0x01), "<start>\n");

	/* Stop pause timer */
	spin_lock_irqsave(&lcd_dev.lcd_lock, flags);
	del_timer(&lcd_timer);
	iTimerFlag = LCD_TIMER_STOP;
	spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);
	iDSPPauseFlg = PAUSE_DSP_OFF;

	if (v4l2_image_info->image_data.yrgbaddr == 0 &&
	    iMixDSPFlg_tmp == MIX_DSP_OFF)
		return 0;

	/* get v4l2 data semafore */
	down(&lcd_dev.sem_image_data);
	printk_dbg((_DEBUG_LCD & 0x80), "down(A) set_v4l2\n");

	if (init_is_first) {
		/* LCD not initialize */
		up(&lcd_dev.sem_image_data);
		printk_dbg((_DEBUG_LCD & 0x80), "up(A) set_v4l2\n");
		return -1;
	}

	/* DSP stop */
	if (v4l2_image_info->image_data.yrgbaddr == 0) {
		printk_dbg((_DEBUG_LCD & 0x01), "<stop DSP>\n");
		/* movie off */
		iMixDSPFlg_tmp = MIX_DSP_OFF;
		v4l2_field = V4L2_TOP_BOTTOM;
		/* update Layer data */
		mix_image(CALLBACK_V4L2_OFF, UPDATE_ON, CLR_LAYER_V4L2);
		return 0;
	}

	if (lcdc_output_mode != v4l2_image_info->output_mode) {
		printk_err("error! V4L2 output mode and LCD output mode "
			   "must be the same.\n\n");
		up(&lcd_dev.sem_image_data);
		printk_dbg((_DEBUG_LCD & 0x80), "up(A) set_v4l2\n");
		return -1;
	}

	/* select 2D(1) */
	memset(&v4l2_layer, 0, sizeof(struct l2_param));

	/* movie on */
	iMixDSPFlg_tmp   = MIX_DSP_ON;

	v4l2_layer.bufsel      = IMC_Lx_BUFSEL_P;
	v4l2_layer.offset      = v4l2_image_info->image_data.size;
	/* y, x */
	v4l2_layer.position    =
		(v4l2_image_info->screen_data.y << IMC_Lx_POSY_SFT |
		v4l2_image_info->screen_data.x << IMC_Lx_POSX_SFT);
	/* h, w */
	v4l2_layer.size        =
		(v4l2_image_info->screen_data.vsize << IMC_Lx_SIZEY_SFT |
		v4l2_image_info->screen_data.hsize << IMC_Lx_SIZEX_SFT);
	v4l2_layer.mposition   = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	v4l2_layer.msize       = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;
	v4l2_layer.control     = IMC_Lx_CONTROL_ENABLE;
	v4l2_layer.resize      = IMC_Lx_RESIZE_DISABLE;
	v4l2_layer.mirror      = IMC_Lx_MIRROR_NO_FLIP;

	switch (v4l2_image_info->yuvfmt) {
	case V4L2_FORMAT_YUV420Pl2: /* YUV420 Semi-Planar */
		printk_dbg((_DEBUG_LCD & 0x02), "format(YUV420Pl2)\n");
		bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
		ppl_y = 1; ppl_uv = 2; ppl_v = 0;
		v4l2_layer.format      = IMC_L2x_FORMAT_YUV420SP;
		v4l2_layer.frameadr_yp =
			v4l2_image_info->image_data.yrgbaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset /
			ppl_y +
			v4l2_image_info->image_data.x * bpp_y  / 8;
		v4l2_layer.frameadr_up =
			v4l2_image_info->image_data.uvaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset *
			bpp_uv / bpp_y / ppl_uv +
			v4l2_image_info->image_data.x * bpp_uv / 8;
		v4l2_layer.frameadr_vp = 0;
		break;
	case V4L2_FORMAT_YUV422Pl2: /* YUV422 Semi-Planar */
		printk_dbg((_DEBUG_LCD & 0x02), "format(YUV422Pl2)\n");
		bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
		ppl_y = 1; ppl_uv = 1; ppl_v = 0;
		v4l2_layer.format      = IMC_L2x_FORMAT_YUV422SP;
		v4l2_layer.frameadr_yp =
			v4l2_image_info->image_data.yrgbaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset /
			ppl_y +
			v4l2_image_info->image_data.x * bpp_y  / 8;
		v4l2_layer.frameadr_up =
			v4l2_image_info->image_data.uvaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset *
			bpp_uv / bpp_y / ppl_uv +
			v4l2_image_info->image_data.x * bpp_uv / 8;
		v4l2_layer.frameadr_vp = 0;
		break;
	case V4L2_FORMAT_YUV420Pl: /* YUV420 Planar      */
		printk_dbg((_DEBUG_LCD & 0x02), "format(YUV420Pl)\n");
		bpp_y = 8;  bpp_uv = 4; bpp_v = 4;
		ppl_y = 1; ppl_uv = 2; ppl_v = 2;
		v4l2_layer.format      = IMC_L2x_FORMAT_YUV420P;
		v4l2_layer.frameadr_yp =
			v4l2_image_info->image_data.yrgbaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset /
			ppl_y +
			v4l2_image_info->image_data.x * bpp_y  / 8;
		v4l2_layer.frameadr_up =
			v4l2_image_info->image_data.uvaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset *
			bpp_uv / bpp_y / ppl_uv +
			v4l2_image_info->image_data.x * bpp_uv / 8;
		v4l2_layer.frameadr_vp =
			v4l2_image_info->image_data.vaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset *
			bpp_v / bpp_y / ppl_v +
			v4l2_image_info->image_data.x * bpp_v  / 8;
		break;
	case V4L2_FORMAT_YUV422Pl: /* YUV422 Planar      */
		printk_dbg((_DEBUG_LCD & 0x02), "format(YUV422Pl)\n");
		bpp_y = 8;  bpp_uv = 4; bpp_v = 4;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		v4l2_layer.format      = IMC_L2x_FORMAT_YUV422P;
		v4l2_layer.frameadr_yp =
			v4l2_image_info->image_data.yrgbaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset /
			ppl_y +
			v4l2_image_info->image_data.x * bpp_y  / 8;
		v4l2_layer.frameadr_up =
			v4l2_image_info->image_data.uvaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset *
			bpp_uv / bpp_y / ppl_uv +
			v4l2_image_info->image_data.x * bpp_uv / 8;
		v4l2_layer.frameadr_vp =
			v4l2_image_info->image_data.vaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset *
			bpp_v / bpp_y / ppl_v +
			v4l2_image_info->image_data.x * bpp_v  / 8;
		break;
	case V4L2_FORMAT_YUV422Px: /* YUV422 Interleave  */
		printk_dbg((_DEBUG_LCD & 0x02), "format(YUV422Px)\n");
		bpp_y = 16; bpp_uv = 0; bpp_v = 0;
		ppl_y = 1; ppl_uv = 0; ppl_v = 0;
		v4l2_layer.format      = IMC_L2x_FORMAT_YUV422I;
		v4l2_layer.frameadr_yp =
			v4l2_image_info->image_data.yrgbaddr +
			v4l2_image_info->image_data.y * v4l2_layer.offset /
			ppl_y +
			v4l2_image_info->image_data.x * bpp_y  / 8;
		v4l2_layer.frameadr_up = 0;
		v4l2_layer.frameadr_vp = 0;
		break;
	default:
		printk_dbg((_DEBUG_LCD & 0x02), "format(%ld)\n",
		 v4l2_layer.format);
		up(&lcd_dev.sem_image_data); /* release data semafore */
		printk_dbg((_DEBUG_LCD & 0x80), "up(A) set_v4l2\n");
		return -1;
	}
	v4l2_layer.frameadr_yq = 0;
	v4l2_layer.frameadr_uq = 0;
	v4l2_layer.frameadr_vq = 0;

	/* set frame data from V4L2 */
	pvDevData_tmp  = v4l2_image_info->frame_data.dev;
	pvVideoBuf_tmp = v4l2_image_info->frame_data.buf;

	v4l2_field = v4l2_image_info->field;

	/* mix - image data from v4l2 and image data
	   from fb driver (DSP and 2D) */
	mix_image(CALLBACK_V4L2_ON, UPDATE_ON, SET_LAYER_V4L2);

	/* start PAUSE timer */
	spin_lock_irqsave(&lcd_dev.lcd_lock, flags);
	if (iTimerFlag == LCD_TIMER_STOP && !DPM_suspend_flg) {
		lcd_timer.expires = jiffies
		 + (MIX_DSP_TIMEOUT * 10 * HZ / 1000);
		add_timer(&lcd_timer);
		iTimerFlag = LCD_TIMER_START;
	}
	spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);
#if LCD_PERF
set_nowtime(SEQ_V4L2);
#endif /* LCD_PERF */

	return 0;
}
EXPORT_SYMBOL(emxx_lcd_set_v4l2_image);


/******************************************************************************
* MODULE   : emxx_lcd_set_fb_image
* FUNCTION : set image data from Frame Buffer to LCD local
* RETURN   :  0 : success
*            -1 : faile
* NOTE     : none
******************************************************************************/
int emxx_lcd_set_fb_image(FB_IMAGE_INFO *fb_image_info)
{
	printk_dbg((_DEBUG_LCD & 0x01), "<start>\n");

#if LCD_PERF
if (!start_perf) {
	printk(KERN_INFO "log_start!\n");
	map_nowtime();
	cnt_time = 0;
	start_perf = 1;
}
set_nowtime(SEQ_FB);
#endif /* LCD_PERF */
	/* get data semafore */
#if 1
	down(&lcd_dev.sem_image_data);
#else
	if (down_interruptible(&lcd_dev.sem_image_data)) {
		printk_dbg((_DEBUG_LCD & 0x80), "down(A) FAILED!! set_fb\n");
		return -1;
	}
#endif
	printk_dbg((_DEBUG_LCD & 0x80), "down(A) set_fb\n");

	if (lcdc_output_mode != fb_image_info->output_mode) {
		iMixDSPFlg_tmp = MIX_DSP_OFF;
#ifdef CONFIG_VIDEO_EMXX
		printk_dbg((_DEBUG_LCD & 0x02),
		 "notify lcd output mode to v4l2: %d\n",
		 (unsigned int)lcdc_output_mode);
		emxx_v4l2_notify_lcd_output((unsigned int)
		 fb_image_info->output_mode);
#endif
		change_output(lcdc_output_mode, fb_image_info->output_mode);
	}

	/* Set other infomation of input data from fb */
	uiAlpha_tmp              = fb_image_info->alpha;
	uiInverseFlag_tmp        = fb_image_info->invflg;
	uiMixFrameBufferPage_tmp = fb_image_info->mix_buf_page;
	printk_dbg((_DEBUG_LCD & 0x01), "MixFrameBuffer(%d)\n",
	 uiMixFrameBufferPage_tmp);

	uiMaskColorFlag_tmp      = fb_image_info->maskcolrflg;
	uiMaskColor_tmp          = fb_image_info->maskcolr;
#if defined(CONFIG_FB_EMXX_ARGB8888) || defined(CONFIG_FB_EMXX_BGR888)
	uiMaskColor_tmp          =
		(((uiMaskColor_tmp & 0xFF0000) >> 16) << IMC_Lx_KEYR_SFT) |
		(((uiMaskColor_tmp & 0x00FF00) >> 8) << IMC_Lx_KEYG_SFT) |
		(((uiMaskColor_tmp & 0x0000FF) >>  0) << IMC_Lx_KEYB_SFT);
#else
	uiMaskColor_tmp          =
		(((((uiMaskColor_tmp & 0xF800) >> 11) << 3) +
		((uiMaskColor_tmp & 0x8000) >> 15) +
		((uiMaskColor_tmp & 0x8000) >> 14) +
		((uiMaskColor_tmp & 0x8000) >> 13)) << IMC_Lx_KEYR_SFT) |
		(((((uiMaskColor_tmp & 0x07E0) >>  5) << 2) +
		((uiMaskColor_tmp & 0x0400) >> 10) +
		((uiMaskColor_tmp & 0x0400) >>  9)) << IMC_Lx_KEYG_SFT) |
		(((((uiMaskColor_tmp & 0x001F) >>  0) << 3) +
		((uiMaskColor_tmp & 0x0010) >>  4) +
		((uiMaskColor_tmp & 0x0010) >>  3) +
		((uiMaskColor_tmp & 0x0010) >>  2)) << IMC_Lx_KEYB_SFT);
#endif

	/* select 2D(1) */
	memset(&fb_layer, 0, sizeof(struct l01_param));

#ifdef CONFIG_FB_EMXX_ARGB8888
	fb_layer.format   = IMC_L01x_FORMAT_ARGB8888;
#elif defined(CONFIG_FB_EMXX_BGR888)
	fb_layer.format   = IMC_L01x_FORMAT_RGB888;
#else
	fb_layer.format   = IMC_L01x_FORMAT_RGB565;
#endif
	fb_layer.bufsel   = IMC_Lx_BUFSEL_P;
#if defined(CONFIG_FB_EMXX_ABGR8888) || defined(CONFIG_FB_EMXX_BGR888)
	fb_layer.bytelane = IMC_Lx_BYTELANE_ABGR;
#else
	fb_layer.bytelane = IMC_Lx_BYTELANE_ARGB;
#endif
	fb_layer.resize   = IMC_Lx_RESIZE_DISABLE;
	fb_layer.mirror   = IMC_Lx_MIRROR_NO_FLIP;
	fb_layer.offset   = fb_image_info->image_data.size;
	fb_layer.control  = IMC_Lx_CONTROL_ENABLE;
	if (fb_image_info->maskcolrflg == TC_COLOR_ENABLE)
		fb_layer.keyenable = IMC_Lx_KEYEN_ENABLE;
	else
		fb_layer.keyenable = IMC_Lx_KEYEN_DISABLE;
	fb_layer.keycolor = uiMaskColor_tmp;

	fb_layer.alpha    = uiAlpha_tmp;
	fb_layer.position =
		(fb_image_info->image_data.y << IMC_Lx_POSY_SFT |
		fb_image_info->image_data.x << IMC_Lx_POSX_SFT);  /* y, x */
	fb_layer.size     =
		(fb_image_info->image_data.vsize << IMC_Lx_SIZEY_SFT |
		fb_image_info->image_data.hsize << IMC_Lx_SIZEX_SFT); /* h, w */
	fb_layer.mposition = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	fb_layer.msize     = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;

	/* image area Y/RGB plane address */
	fb_layer.frameadr_p = fb_image_info->image_data.yrgbaddr;
	fb_layer.frameadr_q = 0;

	/* check update flag */
	switch (fb_image_info->update_flag) {
	case FB_UPDATE_ON:  /*    ioctl(EMXX_FB_UPDATE_SCRN) & update = 0 */
			    /* or interval update                          */
		printk_dbg((_DEBUG_LCD & 0x02), "FB_UPDATE_ON\n");

		/* movie on */
		if (iMixDSPFlg_tmp == MIX_DSP_ON) {
			/* pause status */
			if (iDSPPauseFlg == PAUSE_DSP_ON) {
				mix_image(CALLBACK_V4L2_OFF, UPDATE_ON,
				 SET_LAYER_2D);
			} else {
				/* not pause status */
				mix_image(CALLBACK_V4L2_OFF, UPDATE_OFF,
				 SET_LAYER_2D);
			}
		} else {
			/* movie off */
			mix_image(CALLBACK_V4L2_OFF, UPDATE_ON, SET_LAYER_2D);
		}
		break;
	/* ioctl(EMXX_FB_UPDATE_SCRN) & update = 1 */
	case FB_ABSOLUTERY_UPDATE:
		printk_dbg((_DEBUG_LCD & 0x02), "FB_ABSOLUTERY_UPDATE\n");
		/* movie on */
		mix_image(CALLBACK_V4L2_OFF, UPDATE_ON, SET_LAYER_2D);
		break;
	case FB_UPDATE_OFF:  /* ioctl(EMXX_FB_SET_MODES) */
	default:  /* other                     */
		printk_dbg((_DEBUG_LCD & 0x02), "FB_UPDATE_OFF\n");
		mix_image(CALLBACK_V4L2_OFF, UPDATE_OFF, SET_LAYER_2D);
		break;
	}

#if LCD_PERF
set_nowtime(SEQ_FB);
#endif /* LCD_PERF */
	printk_dbg((_DEBUG_LCD & 0x02), "<end>\n");
	return 0;
}
EXPORT_SYMBOL(emxx_lcd_set_fb_image);


/******************************************************************************
* MODULE   : mix_image
* FUNCTION : By using IMG, mix 2D (and DSP) image
* RETURN   :  0 : success
*          : -1 : failed
* NOTE     : none
******************************************************************************/
static void mix_image(int iCallbackV4L2_tmp, int update, int type)
{
	unsigned long flags;
	int i = 0;

	if (update == UPDATE_ON) {
		/* get refresh semafore */
		down(&lcd_dev.sem_image_refresh);
		printk_dbg((_DEBUG_LCD & 0x80), "down(C) mix_image\n");

		spin_lock_irqsave(&lcd_dev.lcd_lock, flags);
		i = index_lcd_var_next;
		index_lcd_var_next++;
		if (index_lcd_var_next == 2)
			index_lcd_var_next = 0;
		spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);

		/* get flag semafore */
		down(&lcd_var[i].sem_image_flag);
		printk_dbg((_DEBUG_LCD & 0x80), "down(B) mix_image\n");

		lcd_var[i].lcd_refresh_flg = IMC_R_IDLE;

		/* set flag and temp data */
		lcd_var[i].iCallbackV4L2Flg      = iCallbackV4L2_tmp;
		lcd_var[i].uiMixFrameBufferPage  = uiMixFrameBufferPage_tmp;
		lcd_var[i].iMixDSPFlgToFB        = iMixDSPFlg_tmp;

		if (lcd_var[i].iCallbackV4L2Flg == CALLBACK_V4L2_ON) {
			/* set frame data from V4L2 */
			lcd_var[i].pvDevData  = pvDevData_tmp;
			lcd_var[i].pvVideoBuf = pvVideoBuf_tmp;
		}
	}

	if (iMixDSPFlg_tmp == MIX_DSP_OFF) {
#ifdef CONFIG_FB_EMXX_ARGB8888
		fb_layer.alpha  = IMC_Lx_ALPHA_OPAQUE | IMC_Lx_ALPHASEL_BIT;
#else
		fb_layer.alpha  = IMC_Lx_ALPHA_OPAQUE;
#endif
	} else {
		fb_layer.alpha  = uiAlpha_tmp;

		if (v4l2_layer.format == IMC_L2x_FORMAT_YUV422I) {
			v4l2_layer.bytelane =
				IMC_L2x_BYTELANE_422I_YUYV;
		} else {
			v4l2_layer.bytelane = IMC_L2x_BYTELANE_INIT;
		}
	}

	spin_lock_irqsave(&lcd_dev.lcd_lock, flags);
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	if (update == UPDATE_ON)
		lcd_var[i].ctrl_func = 2;
#endif /* CONFIG_PM || CONFIG_DPM */

	if (update == UPDATE_ON) {
		lcd_var[i].lcd_refresh_flg = IMC_R_WAIT_REFRESH;
		list_add_tail(&lcd_var[i].list, &list_lcd_var);
	}

	/* update Layer data */
	if (imc_hw_set_update_reserve(type, iMixDSPFlg_tmp, update))
		printk_dbg((_DEBUG_LCD & 0x02), "update_reserve() failed.\n");

	if (update == UPDATE_ON) {
		/* set IMC all layer data */
		int chg_flg = 0;
		if ((uiInverseFlag_tmp == NO_INVERSE) &&
		    (*ImcNxtVsync.mirror != IMC_MIRROR_NO_FLIP)) {
			*ImcNxtVsync.mirror = IMC_MIRROR_NO_FLIP;
			chg_flg = 1;
		} else if ((uiInverseFlag_tmp != NO_INVERSE) &&
			 (*ImcNxtVsync.mirror == IMC_MIRROR_NO_FLIP)) {
			*ImcNxtVsync.mirror = IMC_MIRROR_HV_FLIP;
			chg_flg = 1;
		}
		if (chg_flg) {
			if (imc_hw_set_update_vsync(&ImcNxtVsync))
				printk_wrn("IMC update_vsync() failed.\n");
		}
	}

	if (update == UPDATE_ON) {
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
		if (iTimerFlag_framecache == LCD_TIMER_STOP) {
			lcd_timer_framecache.expires = jiffies
			 + (FRAMECACHE_TIMEOUT * 10 * HZ / 1000);
			add_timer(&lcd_timer_framecache);
			iTimerFlag_framecache = LCD_TIMER_START;
		} else {
			mod_timer(&lcd_timer_framecache, jiffies
			 + (FRAMECACHE_TIMEOUT * 10 * HZ / 1000));
		}
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
		change_frame();
	}

	/* release data semafore */
	up(&lcd_dev.sem_image_data);
	printk_dbg((_DEBUG_LCD & 0x80), "up(A) mix_image\n");

	spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);

	if (((update == UPDATE_ON) && (blank_state.lcd_output == 0)) ||
	    ((lcdc_output_mode != EMXX_FB_OUTPUT_MODE_LCD) &&
	     (blank_state.lcd_backlight == 0))) {
		printk_dbg((_DEBUG_LCD & 0x40),
		 "call lcd_callback_imc_refresh(): current blank mode = %d\n",
		 blank_state.current_mode);
		lcd_callback_imc_refresh();
	}
}


/*****************************************************************************
* MODULE   : lcd_irq_handler_callback
* FUNCTION : LCDC interrupt handler
* RETURN   :
* NOTE     : none
******************************************************************************/
static void lcd_irq_handler_callback(void)
{
	FRAME_DATA frame_data;
	EMXX_LCD_VAR *p_lcd_var;

#if LCD_PERF
set_nowtime(SEQ_IMC);
#endif /* LCD_PERF */

	printk_dbg((_DEBUG_LCD & 0x01), "<start>\n");

	if (list_empty(&list_lcd_var))
		return;

	p_lcd_var = list_entry(list_lcd_var.next, struct emxx_lcd_var, list);
	list_del(&p_lcd_var->list);

	p_lcd_var->lcd_refresh_flg = IMC_R_DISPLAYED;

	/* when kick mix_image by V4L2 */
	if (p_lcd_var->iCallbackV4L2Flg == CALLBACK_V4L2_ON) {
		p_lcd_var->iCallbackV4L2Flg = CALLBACK_V4L2_OFF;
		/* set frame date to V4L2 */
		frame_data.dev = p_lcd_var->pvDevData;
		frame_data.buf = p_lcd_var->pvVideoBuf;

#ifdef CONFIG_VIDEO_EMXX
		printk_dbg((_DEBUG_LCD & 0x02), "-> v4l2\n");
		/*call v4l2 callback function */
		emxx_v4l2_lcd_callback(frame_data);
#endif
	}

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	p_lcd_var->ctrl_func = 0;
#endif /* CONFIG_PM || CONFIG_DPM */

	/* release semafore */
	up(&p_lcd_var->sem_image_flag);
	printk_dbg((_DEBUG_LCD & 0x80), "up(B) lcd_irq\n");

	printk_dbg((_DEBUG_LCD & 0x02), "<end>\n");
}


/*****************************************************************************
* MODULE   : lcd_irq_handler
* FUNCTION : LCDC interrupt handler
* RETURN   : IRQ_HANDLED
* NOTE     : none
******************************************************************************/
irqreturn_t lcd_irq_handler(int irq, void *dev_id)
{
	unsigned long ulRegVal32;
	unsigned long flags;
	spin_lock_irqsave(&lcd_dev.lcd_lock, flags);

	/*  check Interuppt Status */
	ulRegVal32 = lcd_hw_chk_int_status();

	/* Interrupt Status Clear */
	lcd_hw_int_factor_clr();

	if (ulRegVal32 & LCD_UNDERRUN_BIT) {
		printk_err("LCDC UNDERRUN ERROR. Reset LCDC and IMC.\n");

		/* LCD display off */
		lcd_module_hw_standby();
		lcd_hw_stop();

		/* Reset LCD */
		lcd_hw_reset();
		/* UnReset LCD */
		lcd_hw_unreset();
		/* Restore LCD H/W register data */
		lcd_hw_restore_reg();

		imc_hw_reset(IMC_CH0); /* Reset IMC */
		imc_hw_unreset(IMC_CH0); /* UnReset IMC */

		IMC_reset_flg = 1;
		imc_hw_restore_reg();

		{
			struct list_head *list_tmp;
			EMXX_LCD_VAR *p_lcd_var;
			void (*callback_func[4])(void) =
				{NULL, NULL, NULL, NULL};
			int i = 0;

			list_for_each(list_tmp, &list_lcd_var) {
				p_lcd_var =
					list_entry(list_tmp,
						struct emxx_lcd_var, list);
				switch (p_lcd_var->ctrl_func) {
				case 2:
					callback_func[i] =
						lcd_callback_imc_refresh;
					i++;
					/* FALL THROUGH */
				case 0:
				default:
					break;
				}
			}

			for (i = 0; i < 4; i++) {
				if (callback_func[i] == NULL)
					break;
				else {
					spin_unlock_irqrestore(
					 &lcd_dev.lcd_lock, flags);
					callback_func[i]();
					spin_lock_irqsave(
					 &lcd_dev.lcd_lock, flags);
				}
			}
		}
		/* LCD display on */
		lcd_hw_start();
		lcd_module_hw_wakeup();
	}

	if ((ulRegVal32 & LCD_LCDVS_BIT) && (ulRegVal32 & LCD_FIELD_BIT)) {
		printk_dbg((_DEBUG_LCD & 0x10), "<FIELD_ODD>\n");
		lcd_field = FIELD_ODD;
		if (v4l2_field == V4L2_TOP_BOTTOM) {
			if (emxx_imc_cancel_refresh(imc_info.id)) {
				printk_dbg((_DEBUG_LCD & 0x10),
				 "Cancel & Reserve REFRESH\n");
				refresh_reserved = 1;
			}
		} else { /* v4l2_field == V4L2_BOTTOM_TOP */
			if (refresh_reserved) {
				printk_dbg((_DEBUG_LCD & 0x10),
				 "Set REFRESH\n");
				emxx_imc_set_refresh(imc_info.id);
				refresh_reserved = 0;
			}
		}
	} else if (ulRegVal32 & LCD_FIELD_BIT) {
		printk_dbg((_DEBUG_LCD & 0x10), "<FIELD_EVEN>\n");
		lcd_field = FIELD_EVEN;
		if (v4l2_field == V4L2_TOP_BOTTOM) {
			if (refresh_reserved) {
				printk_dbg((_DEBUG_LCD & 0x10),
				 "Set REFRESH\n");
				emxx_imc_set_refresh(imc_info.id);
				refresh_reserved = 0;
			}
		} else { /* v4l2_field == V4L2_BOTTOM_TOP */
			if (emxx_imc_cancel_refresh(imc_info.id)) {
				printk_dbg((_DEBUG_LCD & 0x10),
				 "Cancel & Reserve REFRESH\n");
				refresh_reserved = 1;
			}
		}
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
		if (direct_reserved) {
 #ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
				save_ckrqmode = readl(SMU_CKRQ_MODE);
				writel(0, SMU_CKRQ_MODE);
			} else {
 #endif
				writel(1, SMU_MEMCHSENA_AFRQ);
 #ifdef CONFIG_MACH_EMEV
				/* unmask LCD (bit0=0) */
				LCD_FIFO_REQ_UNMASK;
			}
 #endif
			writel(LCD_BUSSEL_WB_DIRECT, LCDCMmioV + LCD_BUSSEL);
			direct_reserved = 0;
		}
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
	}

	spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);
	return IRQ_HANDLED;
}

/******************************************************************************
* MODULE   : lcd_callback_imc_refresh
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_callback_imc_refresh(void)
{
	FRAME_DATA    frame_data;
	unsigned long flags;
	struct list_head *list_tmp;
	EMXX_LCD_VAR *p_lcd_var;
	int find = 0;

	spin_lock_irqsave(&lcd_dev.lcd_lock, flags);
	printk_dbg((_DEBUG_LCD & 0x01), "<start>\n");

#ifdef CONFIG_EMXX_LCD_FRAMECACHE
 #ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		writel(save_ckrqmode, SMU_CKRQ_MODE);
	else {
 #endif
		writel(0, SMU_MEMCHSENA_AFRQ);
 #ifdef CONFIG_MACH_EMEV
		/* mask LCD (bit0=1) */
		LCD_FIFO_REQ_MASK;
	}
 #endif
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

	list_for_each(list_tmp, &list_lcd_var) {
		p_lcd_var = list_entry(list_tmp, struct emxx_lcd_var, list);
		if (p_lcd_var->lcd_refresh_flg == IMC_R_WAIT_REFRESH) {
			find = 1;
			break;
		}
	}

	if (find) {
		/* release refresh semafore */
		up(&lcd_dev.sem_image_refresh);
		printk_dbg((_DEBUG_LCD & 0x80),
		 "up(C) lcd_callback_imc_refresh\n");

		/* call FB Driver function "emxx_fb_callback" */
		emxx_fb_callback((int)p_lcd_var->uiMixFrameBufferPage,
		 p_lcd_var->iMixDSPFlgToFB, EMXX_FB_DEVICE_LCD);

		/* when kick mix_image by V4L2 */
		if (p_lcd_var->iCallbackV4L2Flg == CALLBACK_V4L2_ON) {
			/* set frame date to V4L2 */
			frame_data.dev = p_lcd_var->pvDevData;
			frame_data.buf = p_lcd_var->pvVideoBuf;

#ifdef CONFIG_VIDEO_EMXX
			printk_dbg((_DEBUG_LCD & 0x02), "-> v4l2\n");
			/* call v4l2 callback function */
			emxx_v4l2_lcd_refresh_callback(frame_data);
#endif
		}

		p_lcd_var->lcd_refresh_flg = IMC_R_REFRESHED;

		lcd_irq_handler_callback();
	}

	printk_dbg((_DEBUG_LCD & 0x02), "<end>\n");
	spin_unlock_irqrestore(&lcd_dev.lcd_lock, flags);
}


/*****************************************************************************
* MODULE   : lcd_callback_imc_wb
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void lcd_callback_imc_wb(int status)
{
	;
}


/********************************************************
 *                                                      *
 *  Init Function Definitions                           *
 *                                                      *
 *******************************************************/

/******************************************************************************
* MODULE   : emxx_lcd_init_module
* FUNCTION : initialize LCD Driver
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
int __init emxx_lcd_init_module(void)
{
	/* lcd device memset */
	memset(&lcd_dev, 0, sizeof(struct emxx_lcd_dev));

	/* semafore initialize */
	sema_init(&lcd_dev.sem_image_data, 1);
	sema_init(&lcd_dev.sem_image_refresh, 1);

	/* list head initialize */
	INIT_LIST_HEAD(&list_lcd_var);

	/* spin lock initialize */
	spin_lock_init(&lcd_dev.lcd_lock);

	/* call lcd_probe */
	lcd_probe();

	/* variable initialized */
	set_val_init();

	/* timer initiarize */
	lcd_timer_init();

#ifdef CONFIG_EMXX_ANDROID
	init_waitqueue_head(&v4l2_close_q);
#endif
	return 0;
}

/******************************************************************************
* MODULE   : lcd_probe
* FUNCTION : LCD Driver probe
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static void lcd_probe()
{
	/* reserve LCDC Mmio region */
	LCDCMmioV = (char *)IO_ADDRESS(LCDCMmio);
	printk_dbg((_DEBUG_LCD),
	 "LCDCMmio(0x%08lx)  LCDCMmioV(0x%p)\n", LCDCMmio, LCDCMmioV);

	/* initiarize work queue */
	emxx_lcd_workqueue = create_singlethread_workqueue(DEV_NAME);
	INIT_WORK(&wk_timeout_bottom_half, lcd_timeout_bottom_half_do);
}


/******************************************************************************
* MODULE   : set_val_init
* FUNCTION : variable default set
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void set_val_init(void)
{
	int i;

	/* other img infomation */
#if defined(CONFIG_FB_EMXX_ARGB8888) || defined(CONFIG_FB_EMXX_BGR888)
	uiMaskColor_tmp      = 0x000100;
#else
	uiMaskColor_tmp      = 0x0020;
#endif
	uiAlpha_tmp          = IMC_Lx_ALPHA_OPAQUE;

	/* flags */
	iMixDSPFlg_tmp       = MIX_DSP_OFF;
	iDSPPauseFlg         = PAUSE_DSP_OFF;
	uiMaskColorFlag_tmp  = FB_MASK_COLOR_DISP_OFF;
	uiAbsolutelyUpFlag   = FB_UPDATE_OFF;

	for (i = 0; i < 2; i++) {
		sema_init(&lcd_var[i].sem_image_flag, 1);

		/* input image data from fb driver */
		lcd_var[i].uiMixFrameBufferPage = DISPLAY_FRAME_NO_A;

		/* flags */
		lcd_var[i].iMixDSPFlgToFB       = MIX_DSP_OFF;
		lcd_var[i].iCallbackV4L2Flg     = CALLBACK_V4L2_OFF;

		/* frame data */
		lcd_var[i].pvDevData            = 0x00;
		lcd_var[i].pvVideoBuf           = 0x00;

		lcd_var[i].lcd_refresh_flg = IMC_R_IDLE;
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
		lcd_var[i].ctrl_func       = 0;
#endif /* CONFIG_PM || CONFIG_DPM */
	}

	blank_state.current_mode  = FB_BLANK_UNBLANK;
	blank_state.lcd_backlight = 1;
	blank_state.lcd_output    = 1;
	blank_state.lcd_clock     = 1;
	return;
}


/********************************************************
 *  Timer Function Definitions                          *
 *******************************************************/
/******************************************************************************
* MODULE   : lcd_timer_init
* FUNCTION : timer function init
* RETURN   : none
* NOTE     : private function
******************************************************************************/
static void lcd_timer_init(void)
{
	init_timer(&lcd_timer);
	lcd_timer.function = lcd_timeout;
	lcd_timer.data = 0;
	iTimerFlag = LCD_TIMER_STOP;
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	init_timer(&lcd_timer_framecache);
	lcd_timer_framecache.function = lcd_timeout_framecache;
	lcd_timer_framecache.data = 0;
	iTimerFlag_framecache = LCD_TIMER_STOP;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

	return;
}


/******************************************************************************
* MODULE   : lcd_timeout
* FUNCTION : when call emxx_fb_call_timer
* RETURN   : none
* NOTE     : public function
******************************************************************************/
static void lcd_timeout(unsigned long data)
{
	/* change LCD status to PAUSE */
	iDSPPauseFlg = PAUSE_DSP_ON;

	queue_work(emxx_lcd_workqueue, &wk_timeout_bottom_half);
	return;
}


/******************************************************************************
* MODULE   : lcd_timeout_bottom_half_do
* FUNCTION : when call emxx_fb_call_timer
* RETURN   : none
* NOTE     : public function
******************************************************************************/
static void lcd_timeout_bottom_half_do(struct work_struct *num)
{
	/* get semafore */
	down(&lcd_dev.sem_image_data);
	printk_dbg((_DEBUG_LCD & 0x80), "down(A) lcd_timeout\n");

	/* mix image */
	mix_image(CALLBACK_V4L2_OFF, UPDATE_ON, SET_LAYER_2D);

	return;
}


#ifdef CONFIG_EMXX_LCD_FRAMECACHE
/******************************************************************************
* MODULE   : lcd_timeout_framecache
* FUNCTION : change to Direct Path
* RETURN   : none
* NOTE     : public function
******************************************************************************/
static void lcd_timeout_framecache(unsigned long data)
{
	printk_dbg((_DEBUG_LCD & 0x01), "<start>\n");

	if (!down_trylock(&lcd_dev.sem_image_data)) {
		if (!down_trylock(&lcd_var[0].sem_image_flag)) {
			if (!down_trylock(&lcd_var[1].sem_image_flag)) {
				if (lcdc_output_mode ==
				    EMXX_FB_OUTPUT_MODE_HDMI_1080I) {
					direct_reserved = 1;
				} else {
 #ifdef CONFIG_MACH_EMEV
					if ((system_rev & EMXX_REV_MASK) ==
					    EMXX_REV_ES1) {
						save_ckrqmode =
						 readl(SMU_CKRQ_MODE);
						writel(0, SMU_CKRQ_MODE);
					} else {
 #endif
						writel(1, SMU_MEMCHSENA_AFRQ);
 #ifdef CONFIG_MACH_EMEV
						/* unmask LCD (bit0=0) */
						LCD_FIFO_REQ_UNMASK;
					}
 #endif
					writel(LCD_BUSSEL_WB_DIRECT,
					 LCDCMmioV + LCD_BUSSEL);
				}
				up(&lcd_dev.sem_image_data);
				up(&lcd_var[0].sem_image_flag);
				up(&lcd_var[1].sem_image_flag);
			} else {
				up(&lcd_dev.sem_image_data);
				up(&lcd_var[0].sem_image_flag);
			}
		} else
			up(&lcd_dev.sem_image_data);
	}

	return;
}
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */


/********************************************************
 *  Exit Function Definitions                           *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_lcd_exit_module
* FUNCTION : cleanup LCD module
* RETURN   : none
* NOTE     : none
******************************************************************************/
void emxx_lcd_exit_module()
{
	exit_lcdhw();
}


#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/********************************************************
 *  Suspend/Resume Function Definitions                 *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_lcd_suspend
* FUNCTION : suspend LCD driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int emxx_lcd_suspend(struct platform_device *dev, pm_message_t state)
{
	unsigned long ulRegVal32, ulBUSSEL_save;

	if (blank_state.lcd_output == 1) {
#ifdef CONFIG_EMXX_ANDROID
		wait_event_interruptible(v4l2_close_q, (v4l2_open_flag == 0));
#endif
		ulRegVal32 = lcd_hw_chk_bussel();
		ulBUSSEL_save = ulRegVal32;
#ifndef CONFIG_EMXX_LCD_FRAMECACHE
		switch (ulRegVal32 & LCD_BUSSEL_BIT) {
		case LCD_BUSSEL_BACKCOLOR:	/* FALL THROUGH */
		case LCD_BUSSEL_BLACK:
			break;
		case LCD_BUSSEL_WB_LOCAL:	/* FALL THROUGH */
		case LCD_BUSSEL_LOCAL:		/* FALL THROUGH */
		default:
			writel(LCD_BUSSEL_BLACK, LCDCMmioV + LCD_BUSSEL);
			break;
		}
#else /* CONFIG_EMXX_LCD_FRAMECACHE */
		switch (ulRegVal32 & LCD_BUSSEL_BIT) {
		case LCD_BUSSEL_BACKCOLOR: /* FALL THROUGH */
		case LCD_BUSSEL_BLACK:	   /* FALL THROUGH */
		case LCD_BUSSEL_DIRECT:	   /* FALL THROUGH */
		case LCD_BUSSEL_WB_DIRECT:
			break;
		case LCD_BUSSEL_WB_LOCAL:  /* FALL THROUGH */
		case LCD_BUSSEL_LOCAL:	   /* FALL THROUGH */
		default:
			if (lcdc_output_mode ==
			    EMXX_FB_OUTPUT_MODE_HDMI_1080I) {
				writel(LCD_BUSSEL_BLACK,
				 LCDCMmioV + LCD_BUSSEL);
			} else {
 #ifdef CONFIG_MACH_EMEV
				if ((system_rev & EMXX_REV_MASK)
				    == EMXX_REV_ES1) {
					save_ckrqmode =
					 readl(SMU_CKRQ_MODE);
					writel(0, SMU_CKRQ_MODE);
				} else {
 #endif
					writel(1, SMU_MEMCHSENA_AFRQ);
 #ifdef CONFIG_MACH_EMEV
					/* unmask LCD (bit0=0) */
					LCD_FIFO_REQ_UNMASK;
				}
 #endif
				writel(LCD_BUSSEL_WB_DIRECT,
				 LCDCMmioV + LCD_BUSSEL);
			}
			break;
		}
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
	}

#ifdef CONFIG_EMXX_LCD_FRAMECACHE
	del_timer(&lcd_timer_framecache);
	iTimerFlag_framecache = LCD_TIMER_STOP;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */

	DPM_suspend_flg = 1;
	/* LCD timer stop */
	del_timer(&lcd_timer);

	if (blank_state.lcd_output == 1) {
		/* LCD status check */
		lcd_suspend_chk_lcdout();
		if (blank_state.lcd_backlight == 1) {
			/* BackLight OFF  */
			lcd_hw_backlight_off();
		}
		/* LCD display off */
		lcd_module_hw_standby();
		lcd_hw_stop();
	}
	if (blank_state.lcd_clock == 1) {
		/* Save LCD H/W register data */
		lcd_hw_save_reg();

		/* Reset LCD */
		lcd_hw_reset();
	}
	/* Restore IMC H/W register data after resume */
	IMC_reset_flg = 1;
	return 0;
}


/******************************************************************************
* MODULE   : lcd_suspend_chk_lcdout
* FUNCTION : enabled suspend
* RETURN   :  0 : success
*            -1 : failed
* NOTE     : none
******************************************************************************/
int lcd_suspend_chk_lcdout(void)
{
	int i = 0;
	int iRet = -1;
	int wait_max  = 32; /* ms */
	int wait_time = 2;  /* ms */
	int wait_cnt  = wait_max / wait_time;
	unsigned long ulRegVal32;

	do {
		ulRegVal32 = lcd_hw_chk_status();
		switch (ulRegVal32 & LCD_MODSTATUS_BIT) {
		case LCD_MODSTATUS_BACKCOLOR:	/* FALL THROUGH */
		case LCD_MODSTATUS_BLACK:	/* FALL THROUGH */
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
		case LCD_MODSTATUS_DIRECT:
			if ((ulRegVal32 & LCD_MODSTATUS_BIT)
			 == LCD_MODSTATUS_DIRECT)
				direct_path = 1;
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
			i    = wait_cnt;
			iRet = 0;
			break;
#ifdef CONFIG_EMXX_LCD_FRAMECACHE
		case LCD_MODSTATUS_WB_DIRECT:	/* FALL THROUGH */
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */
		case LCD_MODSTATUS_WB_LOCAL:	/* FALL THROUGH */
		case LCD_MODSTATUS_LOCAL:
		default:
			if (i == wait_cnt) {
				printk_dbg((_DEBUG_LCD & 0x01),
				 "failed(%08lx)\n", ulRegVal32);
			} else {
				mdelay(wait_time);
			}
			break;
		}
		i++;
	} while (i <= wait_cnt);

	return iRet;
}


/******************************************************************************
* MODULE   : emxx_lcd_resume
* FUNCTION : suspend LCD driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int emxx_lcd_resume(struct platform_device *dev)
{
	struct list_head *list_tmp;
	EMXX_LCD_VAR *p_lcd_var;
	void (*callback_func[4])(void) = {NULL, NULL, NULL, NULL};
	int i = 0;

	if (DPM_suspend_flg) {
		/* Restore IMC H/W register data */
		if (IMC_reset_flg)
			imc_hw_restore_reg();

		if (blank_state.lcd_clock == 1) {
			/* UnReset LCD */
			lcd_hw_unreset();

			/* Restore LCD H/W register data */
			lcd_hw_restore_reg();
		}
		if (blank_state.lcd_output == 1) {
			/* LCD display on */
			lcd_hw_start();
			lcd_module_hw_wakeup();
		}
		if (blank_state.lcd_backlight == 1) {
			/* BackLight ON */
			lcd_hw_backlight_on();
		}

		/* LCD timer start */
		if ((iTimerFlag == LCD_TIMER_START)
		 && (iMixDSPFlg_tmp == MIX_DSP_ON)) {
			lcd_timer.expires = jiffies
			 + (MIX_DSP_TIMEOUT * 10 * HZ / 1000);
			add_timer(&lcd_timer);
		}

		if (blank_state.lcd_output == 1) {
			list_for_each(list_tmp, &list_lcd_var) {
				p_lcd_var = list_entry(list_tmp,
				 struct emxx_lcd_var, list);
				switch (p_lcd_var->ctrl_func) {
				case 2:
					callback_func[i] =
						lcd_callback_imc_refresh;
					i++;
					/* FALL THROUGH */
				case 0:
				default:
					break;
				}
			}

			for (i = 0; i < 4; i++) {
				if (callback_func[i] == NULL)
					break;
				else
					callback_func[i]();
			}
		}
		DPM_suspend_flg = 0;
	}
	return 0;
}
#endif /* CONFIG_PM || CONFIG_DPM */


MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("EMMA Mobile EV LCD control driver");
MODULE_LICENSE("GPL");


/* module alias */
#ifdef MODULE
module_init(emxx_lcd_init_module);
module_exit(emxx_lcd_exit_module);
#else
#ifndef CONFIG_FB_EMXX
device_initcall(emxx_lcd_init_module);
#endif
#endif
