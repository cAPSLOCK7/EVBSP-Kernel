/*
 * File Name       : drivers/nts/emxx_nts.c
 * Function        : NTSC Driver
 * Release Version : Ver 1.01
 * Release Date    : 2011.02.17
 * Copyright (C) 2010-2011 Renesas Electronics Corporation
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
#include <linux/video_encoder.h>
#include <linux/uaccess.h>

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <mach/pm.h>
#endif /* CONFIG_PM || CONFIG_DPM */

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/pmu.h>
#include <mach/emxx_mem.h>

#include "../video/emxx/emxx_common.h"
#include "../video/emxx/emxx_lcd_common.h"
#include "ntsc.h"
#include "emxx_nts_common.h"
#include "emxx_ntshw.h"
#include "emxx_nts.h"
#include "emxx_nts_image.h"
#include "ntsc_encoder.h"


/********************************************************
 *  debug parameters                                    *
 *******************************************************/
#define _DEBUG_NTS  0x00 /* 00008421(bit) */
			 /* 0x01: debug function in
			  * 0x02: debug function out
			  * 0x40: debug FBIOBLANK
			  * 0x80: debug semafore
			  */


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

#if _DEBUG_NTS
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
#define DEV_NAME "emxx_nts"

/* V4L2 mix ON/OFF */
#define V4L2_MIX_OFF		0
#define V4L2_MIX_ON		1
/* callback to V4L2 status */
#define V4L2_CALLBACK_OFF	0
#define V4L2_CALLBACK_ON	1

/* nts_probe() */
#define NTS_SUCCESS		0
#define NTS_FAILED_REQUESTIRQ	-1

#define DPM_SUSPEND_FLG_INIT 0
#define CTRL_FUNC_INIT 0


/********************************************************
 *  Macros                                              *
 *******************************************************/


/********************************************************
 *  Structure                                           *
 *******************************************************/


/********************************************************
 *  Variables                                           *
 *******************************************************/
struct emxx_nts_dev *ntsc;


/********************************************************
 *  Prototype declarations of local function            *
 *******************************************************/
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
static int          DPM_suspend_flg = DPM_SUSPEND_FLG_INIT;
static int          ctrl_func       = CTRL_FUNC_INIT;
#endif /* CONFIG_PM || CONFIG_DPM */

/* Private function */
static int         nts_set_v4l2_image(V4L2_IMAGE_INFO *v4l2_info);
static void        nts_mix_image(int iMixMode, int iCallbackV4L2);
static void        nts_return_callback_refresh_lock(void);
       /* VSYNC interrupt handler */
static irqreturn_t nts_irq_handler(int irq, void *dev_id);

static void        nts_init_flags(struct emxx_nts_dev *ntsc);
static void        nts_init_val(struct emxx_nts_dev *ntsc);
static int         nts_probe(void);

static int  __init emxx_nts_init_module(void);
#ifdef MODULE
static void __exit emxx_nts_exit_module(void);
static void        emxx_nts_remove(void);
#endif /* MODULE */


/******************************************************************************
* MODULE   : emxx_nts_blank
* FUNCTION : FBIOBLANK
* RETURN   : 0       : success
*            -EINVAL : input value incorrect
* NOTE     : none
******************************************************************************/
int emxx_nts_blank(int blank_mode)
{
	int backlight, output, clock;
	unsigned long flags;

	printk_dbg((_DEBUG_NTS & 0x40), "<start>\n");

	switch (blank_mode) {
	/* Screen: On,  HSync: On,  VSync: On */
	case FB_BLANK_UNBLANK:
		printk_dbg((_DEBUG_NTS & 0x40), "<FB_BLANK_UNBLANK>\n");
		backlight = 1;
		output    = 1;
		clock     = 1;
		break;
	/* Screen: Off, HSync: On,  VSync: On */
	case FB_BLANK_NORMAL:
		printk_dbg((_DEBUG_NTS & 0x40), "<FB_BLANK_NORMAL>\n");
		backlight = 0;
		output    = 1;
		clock     = 1;
		break;
	/* Screen: Off, HSync: On,  VSync: Off */
	case FB_BLANK_VSYNC_SUSPEND:
		printk_dbg((_DEBUG_NTS & 0x40), "<FB_BLANK_VSYNC_SUSPEND>\n");
		backlight = 0;
		output    = 0;
		clock     = 1;
		break;
	/* Screen: Off, HSync: Off, VSync: On */
	case FB_BLANK_HSYNC_SUSPEND:
		printk_dbg((_DEBUG_NTS & 0x40), "<FB_BLANK_HSYNC_SUSPEND>\n");
		backlight = 0;
		output    = 0;
		clock     = 1;
		break;
	/* Screen: Off, HSync: Off, VSync: Off */
	case FB_BLANK_POWERDOWN:
		printk_dbg((_DEBUG_NTS & 0x40), "<FB_BLANK_POWERDOWN>\n");
		backlight = 0;
		output    = 0;
		clock     = 0;
		break;
	default:
		printk_dbg((_DEBUG_NTS & 0x40), "<failed(%d)>\n", -EINVAL);
		return -EINVAL;
	}

	if (ntsc->iOutmodeFlg == NTS_OUTPUT_DISABLE) {
		ntsc->blank_state.nts_backlight = backlight;
		ntsc->blank_state.nts_output    = output;
		ntsc->blank_state.nts_clock     = clock;
		ntsc->blank_state.current_mode  = blank_mode;
		return 0;
	}

	down(&ntsc->sem_image_data);
	down(&ntsc->sem_image_flag);
	spin_lock_irqsave(&ntsc->nts_lock, flags);

	/* output    On -> Off */
	if ((output == 0) && (ntsc->blank_state.nts_output == 1)) {
		/* backlight On -> Off */
		if ((backlight == 0)
		 && (ntsc->blank_state.nts_backlight == 1)) {
			printk_dbg((_DEBUG_NTS & 0x40),
			 "<backlight On -> Off>\n");
			ntsc->blank_state.nts_backlight = 0;
		}
		printk_dbg((_DEBUG_NTS & 0x40), "<output    On -> Off>\n");
		ntsc->blank_state.nts_output    = 0;
		if (ntsc->ntsout_flg != NTS_NTSOUT_DISABLE)
			ntshw_start(NTSHW_STOP);

	} else {
		/* backlight On -> Off */
		if ((backlight == 0)
		 && (ntsc->blank_state.nts_backlight == 1)) {
			printk_dbg((_DEBUG_NTS & 0x40),
			 "<backlight On -> Off>\n");
			ntsc->blank_state.nts_backlight = 0;
			if (ntsc->ntsout_flg == NTS_NTSOUT_ENABLE)
				ntshw_set_ntsout(NTS_NTSOUT_ENABLE_BLACK);
		}
	}

	/* clock     On -> Off */
	if ((clock == 0) && (ntsc->blank_state.nts_clock == 1)) {
		printk_dbg((_DEBUG_NTS & 0x40), "<clock     On -> Off>\n");
		ntsc->blank_state.nts_clock     = 0;
		ntshw_save_reg();
		ntshw_reset(NTSHW_RESET);
	}
	/* clock     Off -> On */
	if ((clock == 1) && (ntsc->blank_state.nts_clock == 0)) {
		printk_dbg((_DEBUG_NTS & 0x40), "<clock     Off -> On>\n");
		ntsc->blank_state.nts_clock     = 1;
		ntshw_reset(NTSHW_UNRESET);
		ntshw_restore_reg();
	}

	/* backlight Off -> On */
	if ((backlight == 1) && (ntsc->blank_state.nts_backlight == 0)) {
		/* output    Off -> on */
		if ((output == 1) && (ntsc->blank_state.nts_output == 0)) {
			printk_dbg((_DEBUG_NTS & 0x40),
			 "<output    Off -> on>\n");
			ntsc->blank_state.nts_output    = 1;
		}
		printk_dbg((_DEBUG_NTS & 0x40), "<backlight Off -> On>\n");
		ntsc->blank_state.nts_backlight = 1;
#if (_DEBUG_NTS & 0x40)
		if (ntsc->ntsout_flg == NTS_NTSOUT_ENABLE)
			printk_dbg((_DEBUG_NTS & 0x40),
			 "<set NTSOUT On (Normal)>\n");
#endif
		ntshw_set_ntsout(ntsc->ntsout_flg);
	} else {
		/* output    Off -> on */
		if ((output == 1) && (ntsc->blank_state.nts_output == 0)) {
			printk_dbg((_DEBUG_NTS & 0x40),
			 "<output    Off -> on>\n");
			ntsc->blank_state.nts_output    = 1;
			if (ntsc->ntsout_flg == NTS_NTSOUT_ENABLE)
				ntshw_set_ntsout(NTS_NTSOUT_ENABLE_BLACK);
		}
	}

	spin_unlock_irqrestore(&ntsc->nts_lock, flags);
	up(&ntsc->sem_image_data);
	up(&ntsc->sem_image_flag);

	ntsc->blank_state.current_mode = blank_mode;
	return 0;
}
EXPORT_SYMBOL(emxx_nts_blank);


/******************************************************************************
* MODULE   : emxx_nts_reserve
* FUNCTION :
* RETURN   :       0 : success
*          : -EINVAL : misstype argument
*          :    -EIO :
* NOTE     : none
******************************************************************************/
int emxx_nts_reserve(int iActiveDevice, int iSetOutmode)
{
	int iRet = 0;
	printk_dbg((_DEBUG_NTS & 0x01),
	 "%d: iActiveDevice(%x)  iSetOutmode(%x)\n", __LINE__, iActiveDevice,
	 iSetOutmode);

	/* check argument */
	if (iActiveDevice < NTS_ACTIVE_FB || iActiveDevice > NTS_ACTIVE_V4L2) {
		iRet = -EINVAL;
		goto failed;
	}
	if (iSetOutmode < NTS_OUTPUT_NTSC || iSetOutmode > NTS_OUTPUT_PAL) {
		iRet = -EINVAL;
		goto failed;
	}

	down(&ntsc->sem_image_data);
	printk_dbg((_DEBUG_NTS & 0x80), "down(A)\n");

	if (!ntsc->iNtsActive || ntsc->iOutmodeFlg == iSetOutmode) {
		if (!ntsc->iNtsActive) {
			/* NTSC initialize */
			iRet = ntshw_reserve(iSetOutmode);
			if (iRet || !ntsc->encoder->hw_init) {
				iRet = -EIO;
			} else {
				int dummy;
				/* ADV7179 initialize */
				iRet = ntsc->encoder->hw_init(&dummy);
				if (iRet) {
					ntshw_release();
					iRet = -EIO;
				} else {
					/* install NTSC irq handler */
					iRet = request_irq(INT_NTS,
						nts_irq_handler, 0,
						DEV_NAME, NULL);
					if (iRet) {
						if (ntsc->encoder->
							hw_shutdown) {
							int dummy;
							ntsc->encoder->
								hw_shutdown(
									&dummy);
						}
						ntshw_release();
						iRet = -EIO;
					} else {
						int iZero = 0;
						ntsc->encoder->hw_command(
							ENCODER_SET_NORM,
							 &iSetOutmode);
						ntsc->encoder->hw_command(
							ENCODER_SET_INPUT,
							&iZero);

						/* start NTSC output */
						ntsc->iOutmodeFlg = iSetOutmode;
						ntsc->ntsout_flg =
							NTS_NTSOUT_ENABLE_BLACK;
						if (ntsc->blank_state.nts_output
							== 1) {
							ntshw_start(
								NTSHW_START);
						} else {
							printk_dbg(
							(_DEBUG_NTS &
							0x40),
							"set NTSOUT disable: "
							"current blank mode = "
							"%d\n",
							ntsc->blank_state.
							current_mode);
							ntshw_set_ntsout(
							NTS_NTSOUT_DISABLE);
						}
					}
				}
			}
		}

		if (!iRet) {
			ntsc->iNtsActive |= iActiveDevice;
			printk_dbg((_DEBUG_NTS & 0x01),
			 "%d: iNtsActive(%04x)\n", __LINE__, ntsc->iNtsActive);
		}
	} else {
		iRet = -EIO;
	}

	up(&ntsc->sem_image_data);
	printk_dbg((_DEBUG_NTS & 0x80), "up(A)\n");

failed:
	return iRet;
}
EXPORT_SYMBOL(emxx_nts_reserve);


/******************************************************************************
* MODULE   : emxx_nts_release
* FUNCTION :
* RETURN   :       0 : success
*          : -EINVAL : misstype argument
* NOTE     : none
******************************************************************************/
int emxx_nts_release(int iActiveDevice)
{
	int iRet = 0;
	printk_dbg((_DEBUG_NTS & 0x01), "%d: iActiveDevice(%x)\n", __LINE__,
	 iActiveDevice);

	/* check argument */
	if (iActiveDevice < NTS_ACTIVE_FB || iActiveDevice > NTS_ACTIVE_V4L2) {
		iRet = -EINVAL;
		goto failed;
	}

	down(&ntsc->sem_image_data);
	printk_dbg((_DEBUG_NTS & 0x80), "down(A)\n");
	down(&ntsc->sem_image_flag);
	printk_dbg((_DEBUG_NTS & 0x80), "down(B)\n");

	if (ntsc->iNtsActive & iActiveDevice) {
		up(&ntsc->sem_image_flag);
		printk_dbg((_DEBUG_NTS & 0x80), "up(B)\n");

		ntsc->iNtsActive &= ~iActiveDevice;
		printk_dbg((_DEBUG_NTS & 0x01), "%d: iNtsActive(%04x)\n",
		 __LINE__, ntsc->iNtsActive);

		if (!ntsc->iNtsActive) {
			ntsc->iOutmodeFlg = NTS_OUTPUT_DISABLE;
			ntsc->ntsout_flg = NTS_NTSOUT_DISABLE;
			ntshw_start(NTSHW_STOP);
			free_irq(INT_NTS, NULL);
			ntshw_release();

			if (ntsc->encoder->hw_shutdown) {
				int dummy;
				ntsc->encoder->hw_shutdown(&dummy);
			}
			nts_init_flags(ntsc);
		}
	} else {
		up(&ntsc->sem_image_flag);
		printk_dbg((_DEBUG_NTS & 0x80), "up(B)\n");
	}

	up(&ntsc->sem_image_data);
	printk_dbg((_DEBUG_NTS & 0x80), "up(A)\n");

failed:
	return iRet;
}
EXPORT_SYMBOL(emxx_nts_release);


/******************************************************************************
* MODULE   : emxx_nts_getmode
* FUNCTION :
* RETURN   : 0 : non active
*          : 1 : active & NTSC
*          : 2 : active & PAL
* NOTE     : none
******************************************************************************/
int emxx_nts_getmode(void)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");
	return ntsc->iOutmodeFlg;
}
EXPORT_SYMBOL(emxx_nts_getmode);


/******************************************************************************
* MODULE   : emxx_nts_set_v4l2_image
* FUNCTION : set image data from v4l2 to NTSC local
* RETURN   :  0 : success
*            -1 : faile
* NOTE     : none
******************************************************************************/
int emxx_nts_set_v4l2_image(V4L2_IMAGE_INFO *v4l2_info)
{
	int ret = 0;

	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	down(&ntsc->sem_image_data);	/* get data semafore */
	printk_dbg((_DEBUG_NTS & 0x80), "down(A)\n");

	if (ntsc->iNtsActive & NTS_ACTIVE_V4L2) {
		/* V4L2 active */
		nts_set_v4l2_image(v4l2_info);
	} else {
		/* V4L2 not active */
		ret = -EACCES;
	}
	up(&ntsc->sem_image_data);	/* release data semafore */
	printk_dbg((_DEBUG_NTS & 0x80), "up(A)\n");
	return ret;
}
EXPORT_SYMBOL(emxx_nts_set_v4l2_image);


/******************************************************************************
* MODULE   : nts_set_v4l2_image
* FUNCTION : set image data from v4l2 to NTSC local
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
static int nts_set_v4l2_image(V4L2_IMAGE_INFO *v4l2_info)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* DSP stop */
	if (v4l2_info->image_data.yrgbaddr == 0) {
		printk_dbg((_DEBUG_NTS & 0x01), "<stop V4L2>\n");
		/* movie off */
		ntsc->iMixDSPFlg = V4L2_MIX_OFF;
		/* update Layer data */
		nts_mix_image(V4L2_OFF, V4L2_CALLBACK_OFF);
		return 0;
	}

	/* movie on */
	ntsc->iMixDSPFlg = V4L2_MIX_ON;

	memcpy(&ntsc->from_v4l2, v4l2_info, sizeof(V4L2_IMAGE_INFO));

	/* mix - image data from v4l2 */
	if (ntsc->from_v4l2.screen_data.hsize == 0) {
		printk_dbg((_DEBUG_NTS & 0x01), "<V4L2_OFF>\n");
		nts_mix_image(V4L2_OFF, V4L2_CALLBACK_ON);
	} else {
		printk_dbg((_DEBUG_NTS & 0x01), "<V4L2_ON>\n");
		nts_mix_image(V4L2_ON,  V4L2_CALLBACK_ON);
	}

	return 0;
}


/******************************************************************************
* MODULE   : nts_mix_image
* FUNCTION : By using SIZ/ROT, mix image
* RETURN   :  0 : success
*          : -1 : failed
* NOTE     : none
******************************************************************************/
static void nts_mix_image(int iMixMode, int iCallbackV4L2)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* get flag semafore */
	down(&ntsc->sem_image_flag);
	printk_dbg((_DEBUG_NTS & 0x80), "down(B)\n");

	/* set flag and temp data */
	ntsc->iV4L2CallbackFlg     = iCallbackV4L2;

	ntsc->iMixImageMode = (ntsc->iMixImageMode & ~V4L2_BIT)
	 | iMixMode;
#if 0
	ntsc->iMixImageMode = (ntsc->iMixImageMode & ~FB_BIT)
	 | FB_OFF;
#endif

	if (ntsc->iV4L2CallbackFlg == V4L2_CALLBACK_ON) {
		/* set frame data from V4L2 */
		memcpy(&ntsc->v4l2_callback_data,
		 &ntsc->from_v4l2.frame_data,
		 sizeof(struct _FRAME_DATA));
	}

	if (ntsc->iMixImageMode == FB_V4L2_OFF) {
		printk_dbg((_DEBUG_NTS & 0x02),
		 "iMixImageMode(%d) -> Black screen\n", ntsc->iMixImageMode);

		/* Black Screen */
		ntsc->ntsout_flg = NTS_NTSOUT_ENABLE_BLACK;
		if (ntsc->blank_state.nts_output == 1) {
			ntshw_set_ntsout(NTS_NTSOUT_ENABLE_BLACK);
		} else {
			printk_dbg((_DEBUG_NTS & 0x40),
			 "set NTSOUT disable: current blank mode = %d\n",
			 ntsc->blank_state.current_mode);
			ntshw_set_ntsout(NTS_NTSOUT_DISABLE);
		}
		nts_return_callback_refresh_lock();
		nts_return_callback();
	} else {
		printk_dbg((_DEBUG_NTS & 0x02),
		 "iMixImageMode(%d)\n", ntsc->iMixImageMode);
		printk_dbg((_DEBUG_NTS & 0x02),
		 "nts_image_compose_request() call\n");
		/* request to compose image */
		if (nts_image_compose_request()) {
			printk_dbg((_DEBUG_NTS & 0x02),
			 "nts_image_compose_request() failed\n");
			nts_return_callback_refresh_lock();
			nts_return_callback();
		}
	}
}


/*****************************************************************************
* MODULE   : nts_irq_handler
* FUNCTION : NTSC interrupt handler
* RETURN   : IRQ_HANDLED
* NOTE     : none
******************************************************************************/
static irqreturn_t nts_irq_handler(int irq, void *dev_id)
{
	unsigned long ulRegVal32, flags;

	printk_dbg((_DEBUG_NTS & 0x01), "\n");
	spin_lock_irqsave(&ntsc->nts_lock, flags);

	/*  check Interrupt Status */
	ulRegVal32 = ntshw_chk_intstatus();

	/* Interrupt Status Clear */
	ntshw_set_interruput(NTSHW_INTFFCLR);

	if (ulRegVal32 & NTS_NTSVS_BIT)
		nts_irq_handler_sub();

	spin_unlock_irqrestore(&ntsc->nts_lock, flags);
	printk_dbg((_DEBUG_NTS & 0x02), "<end>\n");

	return IRQ_HANDLED;
}


/*****************************************************************************
* MODULE   : nts_irq_handler_sub
* FUNCTION : NTSC interrupt handler
* RETURN   :
* NOTE     : none
******************************************************************************/
void nts_irq_handler_sub(void)
{
	unsigned long ulRegVal32;

	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	if (ntsc->blank_state.nts_output == 1) {
		/* check AREASTATUS */
		ulRegVal32 = ntshw_chk_framesel();

		if (((ntsc->iFrameNoNext == NTS_DISP_FRAME_A) &&
			((ulRegVal32 & NTS_AREASTATUS_BIT) ==
			NTS_AREASTATUS_BUFA)) || ((ntsc->iFrameNoNext ==
			NTS_DISP_FRAME_B) && ((ulRegVal32 &
			NTS_AREASTATUS_BIT) == NTS_AREASTATUS_BUFB))) {
			/* disable NTSC interrupts */
			ntshw_set_interruput(NTSHW_INTENCLR);

			ntsc->iFrameNoNow = ntsc->iFrameNoNext;

			nts_return_callback();
		}
	} else {
		/* disable NTSC interrupts */
		ntshw_set_interruput(NTSHW_INTENCLR);

		ntsc->iFrameNoNow = ntsc->iFrameNoNext;

		nts_return_callback();
	}
	printk_dbg((_DEBUG_NTS & 0x02), "<end>\n");
}


/*****************************************************************************
* MODULE   : nts_return_callback
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void nts_return_callback(void)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* when kick nts_mix_image by V4L2 */
	if (ntsc->iV4L2CallbackFlg == V4L2_CALLBACK_ON) {
		ntsc->iV4L2CallbackFlg = V4L2_CALLBACK_OFF;

		printk_dbg((_DEBUG_NTS & 0x02), "-> v4l2\n");
		/* call v4l2 callback function */
		emxx_v4l2_lcd_callback(ntsc->v4l2_callback_data);
		memset(&ntsc->v4l2_callback_data, 0,
		 sizeof(struct _FRAME_DATA));
	}

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	ctrl_func = 0;
#endif /* CONFIG_PM || CONFIG_DPM */

	ntsc->compose_complete = 1;
	wake_up_interruptible(&ntsc->wait_compose);

	/* release semafore */
	up(&ntsc->sem_image_flag);
	printk_dbg((_DEBUG_NTS & 0x80), "up(B)\n");
}


/*****************************************************************************
* MODULE   : nts_return_callback_refresh
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void nts_return_callback_refresh_lock(void)
{
	unsigned long flags;
	spin_lock_irqsave(&ntsc->lock_callback, flags);
	nts_return_callback_refresh();
	spin_unlock_irqrestore(&ntsc->lock_callback, flags);
}
void nts_return_callback_refresh(void)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* when kick nts_mix_image by V4L2 */
	if (ntsc->iV4L2CallbackFlg == CALLBACK_V4L2_ON) {
		printk_dbg((_DEBUG_NTS & 0x02), "-> v4l2\n");
		/* call v4l2 callback function */
		emxx_v4l2_lcd_refresh_callback(ntsc->v4l2_callback_data);
	}

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	ctrl_func = 1;
#endif /* CONFIG_PM || CONFIG_DPM */
}


/*****************************************************************************
* MODULE   : nts_return_callback_ready
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void nts_return_callback_ready(void)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* when kick nts_mix_image by V4L2 */
	if (ntsc->iV4L2CallbackFlg == CALLBACK_V4L2_ON) {
		printk_dbg((_DEBUG_NTS & 0x02), "-> v4l2\n");
		/* call v4l2 callback function */
		emxx_v4l2_ntsc_ready_callback(ntsc->v4l2_callback_data);
	}
}


#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/********************************************************
 *  Suspend/Resume Function Definitions                 *
 *******************************************************/
/******************************************************************************
* MODULE   : emxx_nts_suspend
* FUNCTION : suspend NTSC driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int emxx_nts_suspend(struct platform_device *dev, pm_message_t state)
{
	if (ntsc->iOutmodeFlg == NTS_OUTPUT_DISABLE) {
		return 0;
	} else {
		DPM_suspend_flg = 1;

		if (ntsc->blank_state.nts_output == 1) {
			/* NTSC POWER OFF */
			ntshw_start(NTSHW_STOP);
		}

		/* Save NTSC H/W register data */
		ntshw_save_reg();

		/* Reset NTSC */
		ntshw_reset(NTSHW_RESET);
		return 0;
	}
}


/******************************************************************************
* MODULE   : emxx_nts_resume
* FUNCTION : suspend NTSC driver
* RETURN   : 0 : success
* NOTE     : none
******************************************************************************/
int emxx_nts_resume(struct platform_device *dev)
{
	if (ntsc->iOutmodeFlg == NTS_OUTPUT_DISABLE) {
		return 0;
	} else {
		if (DPM_suspend_flg) {
			DPM_suspend_flg = 0;

			/* UnReset NTSC */
			ntshw_reset(NTSHW_UNRESET);

			/* Restore NTSC H/W register data */
			ntshw_restore_reg();

			if (ntsc->blank_state.nts_output == 1) {
				/* NTSC POWER ON */
				ntshw_start(NTSHW_START);
			}

			switch (ctrl_func) {
/*			case 2: */
/*				nts_return_callback_refresh(); */
				/* FALL THROUGH */
			case 1:
				nts_return_callback();
				break;
			case 0:
			default:
				break;
			}
		}
		return 0;
	}
}
#endif /* CONFIG_PM || CONFIG_DPM */


/********************************************************
 *  Init Function Definitions                           *
 *******************************************************/
/******************************************************************************
* MODULE   : nts_probe
* FUNCTION : NTSC Driver probe
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static int nts_probe(void)
{
	int iRet;
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* initiarize image compose request */
	iRet = nts_image_initialize(ntsc);
	if (iRet)
		goto err_ret;

	/* initialize semafore */
	sema_init(&ntsc->sem_image_data, 1);
	sema_init(&ntsc->sem_image_flag, 1);

	/* initialize spin lock */
	spin_lock_init(&ntsc->nts_lock);
	spin_lock_init(&ntsc->lock_callback);

	init_waitqueue_head(&ntsc->wait_compose);

	ntsc->blank_state.current_mode  = FB_BLANK_UNBLANK;
	ntsc->blank_state.nts_backlight = 1;
	ntsc->blank_state.nts_output    = 1;
	ntsc->blank_state.nts_clock     = 1;
	ntsc->ntsout_flg                = NTS_NTSOUT_DISABLE;

err_ret:
	return iRet;
}


/******************************************************************************
* MODULE   : nts_init_flags
* FUNCTION : variable default set
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void nts_init_flags(struct emxx_nts_dev *ntsc)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");
	ntsc->iNtsActive           = NTS_NONACTIVE;

	/*** flags ***/
	/* movie mix ON/OFF               */
	ntsc->iMixDSPFlg           = V4L2_MIX_OFF;

	/* need to callback to V4L2       */
	ntsc->iV4L2CallbackFlg     = V4L2_CALLBACK_OFF;
	memset(&ntsc->v4l2_callback_data, 0, sizeof(struct _FRAME_DATA));

	/* now display frame no           */
	ntsc->iFrameNoNow          = NTS_DISP_INIT;
	/* set change frame no            */
	ntsc->iFrameNoNext         = NTS_DISP_INIT;

	/* output i/f                     */
	ntsc->iOutmodeFlg          = NTS_OUTPUT_DISABLE;
	ntsc->iMixImageMode        = FB_OFF | V4L2_OFF;

	memset(&ntsc->from_v4l2, 0, sizeof(V4L2_IMAGE_INFO));
}


/******************************************************************************
* MODULE   : nts_init_val
* FUNCTION : variable default set
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void nts_init_val(struct emxx_nts_dev *ntsc)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");
	memset(ntsc, 0, sizeof(struct emxx_nts_dev));

	nts_init_flags(ntsc);
}


/******************************************************************************
* MODULE   : nts_alloc_dev
* FUNCTION : variable default set
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void *nts_alloc_dev(void)
{
	void *alloc_dev;

	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	alloc_dev = kmalloc(sizeof(struct emxx_nts_dev), GFP_KERNEL);
	if (!alloc_dev) {
		;
	} else {
		/* variable initialized */
		nts_init_val((struct emxx_nts_dev *)alloc_dev);
	}
	return alloc_dev;
}


/******************************************************************************
* MODULE   : nts_free_dev
* FUNCTION : variable default set
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void nts_free_dev(void)
{
	if (ntsc) {
		memset(ntsc, 0, sizeof(struct emxx_nts_dev));
		kfree(ntsc);
	}
}


/******************************************************************************
* MODULE   : emxx_nts_init_module
* FUNCTION : initialize NTSC Driver
* RETURN   : 0     : success
*            other : fail
* NOTE     : none
******************************************************************************/
static int __init emxx_nts_init_module(void)
{
	int iRet;

	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	/* nts_device memset */
	ntsc = (struct emxx_nts_dev *)nts_alloc_dev();
	if (!ntsc) {
		printk_err("memory allocate error\n");
		iRet = -ENOMEM;
		goto fail_nts_alloc_dev;
	}

	/* initialize ADV7179 */
	iRet = ntsc_encoder_init(ntsc);
	if (iRet) {
		printk_err("adv7179 initalize error\n");
		goto fail_ntsc_encoder_init;
	}

	/* initialize NTSC */
	iRet = ntshw_init(ntsc);
	if (iRet) {
		printk_err("ntshw initalize error\n");
		goto fail_ntshw_init;
	}

	/* call nts_probe */
	nts_probe();
	if (iRet) {
		printk_err("nts probe error\n");
		goto fail_nts_probe;
	}

	printk_info("registered device ntsc\n");
	iRet = 0;
	goto success;

fail_nts_probe:
	ntshw_exit();
fail_ntshw_init:
	ntsc_encoder_exit(ntsc);
fail_ntsc_encoder_init:
	nts_free_dev();
fail_nts_alloc_dev:
success:
	return iRet;
}


/********************************************************
 *  Exit Function Definitions                           *
 *******************************************************/
#ifdef MODULE
/******************************************************************************
* MODULE   : emxx_nts_exit_module
* FUNCTION : cleanup NTSC module
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void __exit emxx_nts_exit_module(void)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	emxx_nts_remove();

	/* disable NTSC */
	ntshw_exit();

	ntsc_encoder_exit(ntsc);
	nts_free_dev();
}


/******************************************************************************
* MODULE   : emxx_nts_remove
* FUNCTION : release NTSC resource
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void emxx_nts_remove(void)
{
	printk_dbg((_DEBUG_NTS & 0x01), "\n");

	nts_image_finalize();
}
#endif /* MODULE */


/* module alias */
#ifdef MODULE
module_init(emxx_nts_init_module);
module_exit(emxx_nts_exit_module);
#else
device_initcall(emxx_nts_init_module);
#endif


