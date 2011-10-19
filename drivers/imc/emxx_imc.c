/*
 * File Name       : drivers/imc/emxx_imc.c
 * Function        : IMC Driver
 * Release Version : Ver 1.14
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
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <mach/pm.h>
#endif /* CONFIG_PM || CONFIG_DPM */

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/pmu.h>
#include <mach/emxx_mem.h>

#include <mach/imc.h>
#include <mach/emxx_imc.h>
#include "emxx_imc.h"

#include <asm/system.h>


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define _DEBUG_IMC  0x00 /* 00008421(bit) */
			 /* 0x01: debug function in
			  * 0x02: debug function out
			  */

#define DEV_NAME "emxx_imc"
#define DPM_SUSPEND_FLG_INIT 0

#define ID_IMC_CH0_MIN  0x80000000
#define ID_IMC_CH0_MAX  0xBFFFFFFF
#define ID_IMC_CH1_MIN  0xC0000000
#define ID_IMC_CH1_MAX  0xFFFFFFFF
#define ID_CHANNEL_BIT  0x40000000
#define ID_CHANNEL_SHT  30

#define COEF_R0_BT601 0x12A
#define COEF_R1_BT601 0x000
#define COEF_R2_BT601 0x199
#define COEF_R3_BT601 0x000
#define COEF_G0_BT601 0x12A
#define COEF_G1_BT601 0x79C
#define COEF_G2_BT601 0x730
#define COEF_G3_BT601 0x000
#define COEF_B0_BT601 0x12A
#define COEF_B1_BT601 0x205
#define COEF_B2_BT601 0x000
#define COEF_B3_BT601 0x000
#define COEF_R0_BT709 0x12A
#define COEF_R1_BT709 0x000
#define COEF_R2_BT709 0x1CB
#define COEF_R3_BT709 0x000
#define COEF_G0_BT709 0x12A
#define COEF_G1_BT709 0x7C9
#define COEF_G2_BT709 0x777
#define COEF_G3_BT709 0x000
#define COEF_B0_BT709 0x12A
#define COEF_B1_BT709 0x21D
#define COEF_B2_BT709 0x000
#define COEF_B3_BT709 0x000


/********************************************************
 *  Macros                                              *
 *******************************************************/
#define printk_err(fmt, arg...) \
	do {                     \
		printk(KERN_ERR DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_info(fmt, arg...) \
	do {                      \
		printk(KERN_INFO DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#if _DEBUG_IMC
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
 *  Structures                                          *
 *******************************************************/
struct emxx_imc_device {
	unsigned long             IMCMmio;
	char                     *IMCMmioV;

	spinlock_t                imc_hw_lock;

	int                       chk_preset;
	int                       ctrl_func;

	unsigned long             sequence;
	unsigned long             id;
	unsigned int              device;
	imc_callback_func_refresh callback_func_refresh;
	imc_callback_func_wb      callback_func_wb;
};


/********************************************************
 *  Variables                                           *
 *******************************************************/
static struct emxx_imc_device *dev;
static wait_queue_head_t wait_que_resource;


/********************************************************
 *  Prototype declarations of local function            *
 *******************************************************/
/* ----- internal function --------------------------- */
static void        imc_hw_init(void);
static void        imc_hw_int_enable(int channel);
static void        imc_hw_int_disable(int channel);
static int         imc_hw_chk_startmode(int channel);
static int         imc_hw_chk_busy(int channel);
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
static int         imc_hw_chk_status(int channel);
#endif /* CONFIG_PM || CONFIG_DPM */

/* ----- IMC IRQ handler ----------------------------- */
static irqreturn_t imc_irq_handler(int irq, void *dev_id);
static irqreturn_t imcw_irq_handler(int irq, void *dev_id);
static void        imc_irq_handler_sub(int irq, void *dev_id, int channel);

/* ----- suspend/resume function --------------------- */
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
static void        imc_resume_kick(unsigned long num);

/* Tasklet */
static DECLARE_TASKLET(imc_nexttask, imc_resume_kick, 0);

/* Flags */
static int DPM_suspend_flg = DPM_SUSPEND_FLG_INIT;

/* Macro */
#define Call_StatusCtrlFunc_ON(channel)  \
	do { dev[channel].ctrl_func = 1; } while (0)
#define Call_StatusCtrlFunc_OFF(channel) \
	do { dev[channel].ctrl_func = 0; } while (0)
#else
#define Call_StatusCtrlFunc_ON(channel)
#define Call_StatusCtrlFunc_OFF(channel)
#endif /* CONFIG_PM || CONFIG_DPM */


/* ----- external function ------------------------------------------------- */
/*****************************************************************************
* MODULE   : emxx_request_imc
* FUNCTION : Acquire IMC channel
* RETURN   : 0            : success
*          : -EBUSY (-16) : IMC channel is busy
*          : -EINVAL(-22) : channel or arg is invalid
* NOTE     : none
******************************************************************************/
int emxx_request_imc(int channel, struct emxx_imc_info *arg)
{
	int ret = 0;
	unsigned long min = 0;
	unsigned long max = 0;
	unsigned long flags;
	struct timeval start_time;
	struct timeval now_time;
	int delta_msec = 0;
	int waittime = 0;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (channel == IMC_CH0) {
		min = ID_IMC_CH0_MIN;
		max = ID_IMC_CH0_MAX;
	} else if (channel == IMC_CH1) {
		min = ID_IMC_CH1_MIN;
		max = ID_IMC_CH1_MAX;
	} else {
		printk_err("error! channel is invalid\n");
		ret = -EINVAL;
		goto request_imc_ret;
	}

	if (arg == NULL) {
		printk_err("error! arg is NULL\n");
		ret = -EINVAL;
		goto request_imc_ret;
	}

	arg->id   = 0;
	arg->adr  = 0;
	arg->size = 0;

	switch (arg->device) {
	case DEV_FB:
	case DEV_LCD:
	case DEV_DSP:
	case DEV_CAM:
	case DEV_OTHER:
		break;
	default:
		printk_err("error! arg->device is invalid\n");
		ret = -EINVAL;
		goto request_imc_ret;
		break;
	}

	do_gettimeofday(&start_time);
	do {
		if (dev[channel].id != 0 && arg->timeout != 0) {
			waittime = (arg->timeout - delta_msec + 1000 / HZ - 1) /
				   (1000 / HZ);
			wait_event_interruptible_timeout(wait_que_resource,
			 dev[channel].id == 0, waittime);
			do_gettimeofday(&now_time);
			delta_msec =
			 (now_time.tv_sec - start_time.tv_sec) * 1000 +
			 (now_time.tv_usec - start_time.tv_usec) / 1000;
		}

		spin_lock_irqsave(&dev[channel].imc_hw_lock, flags);
		if (dev[channel].id == 0) {
			if (dev[channel].sequence == 0)
				dev[channel].sequence = min;
			else if (dev[channel].sequence == max)
				dev[channel].sequence = min;
			else
				dev[channel].sequence++;

			dev[channel].id = dev[channel].sequence;
			dev[channel].device = arg->device;
			spin_unlock_irqrestore(&dev[channel].imc_hw_lock,
			 flags);

			arg->id = dev[channel].sequence;
			if (channel == IMC_CH0) {
				arg->adr  = EMXX_IMC_DATA_BASE;
				arg->size = 0x08000000;
			} else { /* (channel == IMC_CH1) */
				arg->adr  = EMXX_IMCW_DATA_BASE;
				arg->size = 0x08000000;
			}

			break;
		} else {
			spin_unlock_irqrestore(&dev[channel].imc_hw_lock,
			 flags);
		}
	} while (arg->timeout != 0 && delta_msec < arg->timeout);

	if (arg->id == 0) {
		printk_err("IMC is busy\n");
		printk_err("  channel: %d\n", channel);
		printk_err("  dev_now: %d\n", dev[channel].device);
		printk_err("  dev_err: %d\n", arg->device);
		ret = -EBUSY;
	}

request_imc_ret:
	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return ret;
}
EXPORT_SYMBOL(emxx_request_imc);


/*****************************************************************************
* MODULE   : emxx_free_imc
* FUNCTION : Release IMC channel
* RETURN   : 0            : success
*          : -ENODEV(-19) : id is invalid
* NOTE     : none
******************************************************************************/
int emxx_free_imc(unsigned long id)
{
	int ret = 0;
	int channel = IMC_CH0;
	unsigned long flags;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		ret = -ENODEV;
	}

	if (ret == 0) {
		spin_lock_irqsave(&dev[channel].imc_hw_lock, flags);
		dev[channel].id = 0;
		wake_up_interruptible(&wait_que_resource);
		spin_unlock_irqrestore(&dev[channel].imc_hw_lock, flags);
	}

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return ret;
}
EXPORT_SYMBOL(emxx_free_imc);


/******************************************************************************
* MODULE   : emxx_imc_set_preset
* FUNCTION : Set IMC Immediately-reflected Registers.
* RETURN   : 0            : success
*          : -EBUSY (-16) : IMC Running
*          : -ENODEV(-19) : id is invalid
*          : -EINVAL(-22) : imc_preset is invalid
* NOTE     : none
******************************************************************************/
int emxx_imc_set_preset(unsigned long id, struct emxx_imc_preset *imc_preset)
{
#if 0 /* provisional transaction (no check IMC_STATUS) */
	int iLoop;
#endif
	char *IMCMmioV;
	int channel = IMC_CH0;
	unsigned long flags;
	int gamma_cnt = 0;
	unsigned long imc_control;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return -ENODEV;
	}

	if (imc_preset == NULL) {
		printk_err("error! imc_preset is NULL\n");
		return -EINVAL;
	}

#if 0 /* provisional transaction (no check IMC_STATUS) */
	for (iLoop = 0; iLoop < 16; iLoop++) {
		if (imc_hw_chk_busy(channel))
			schedule();
		else
			break;
	}
	if (iLoop == 16) {
		/* IMC running */
		printk_err("IMC WORKING NOW\n");
		return -EBUSY;
	}
#endif

	/*>>>>>> interuppt disable */
	spin_lock_irqsave(&dev[channel].imc_hw_lock, flags);

	IMCMmioV = dev[channel].IMCMmioV;

	if (imc_preset->imc_control) {
		imc_control = *imc_preset->imc_control;
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			imc_control &= ~IMC_CLKCNT_BIT;
		writel(imc_control, IMCMmioV + IMC_CONTROL);
	}

	if (imc_preset->imc_datareq)
		writel(*imc_preset->imc_datareq, IMCMmioV + IMC_DATAREQ);

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1 ||
	    channel == IMC_CH0) {
#else
	if (channel == IMC_CH0) {
#endif
		if (imc_preset->gamma) {
			unsigned long *data = imc_preset->gamma->data;
			if (imc_preset->gamma->en & IMC_FRAME_BIT) {
				unsigned long ulRegVal;
				ulRegVal = readl(IMCMmioV + IMC_CONTROL);
				writel(ulRegVal & ~IMC_CLKCNT_GAMMA,
				 IMCMmioV + IMC_CONTROL);
				writel(IMC_GAMMA_EN_OFF,
				 IMCMmioV + IMC_GAMMA_EN);
				writel(imc_preset->gamma->adr,
				 IMCMmioV + IMC_GAMMA_ADR);
				for (gamma_cnt = 0; gamma_cnt < 256;
					gamma_cnt++ , data++) {
					writel(*data,
					 IMCMmioV + IMC_GAMMA_DATA);
				}
				writel(ulRegVal, IMCMmioV + IMC_CONTROL);
			}
			writel(imc_preset->gamma->en, IMCMmioV + IMC_GAMMA_EN);
		}

		if (imc_preset->yuv) {
			writel(imc_preset->yuv->ygain,
			 IMCMmioV + IMC_YGAINOFFSET);
			writel(imc_preset->yuv->ugain,
			 IMCMmioV + IMC_UGAINOFFSET);
			writel(imc_preset->yuv->vgain,
			 IMCMmioV + IMC_VGAINOFFSET);
			writel(imc_preset->yuv->yuv2rgb,
			 IMCMmioV + IMC_YUV2RGB);

			if ((imc_preset->yuv->yuv2rgb & IMC_TRANSMODE_BIT) ==
				IMC_TRANSMODE_CUSTOM1 ||
			    (imc_preset->yuv->yuv2rgb & IMC_TRANSMODE_BIT) ==
				IMC_TRANSMODE_CUSTOM2) {
				writel(imc_preset->yuv->coef_r[0],
				 IMCMmioV + IMC_COEF_R0);
				writel(imc_preset->yuv->coef_r[1],
				 IMCMmioV + IMC_COEF_R1);
				writel(imc_preset->yuv->coef_r[2],
				 IMCMmioV + IMC_COEF_R2);
				writel(imc_preset->yuv->coef_r[3],
				 IMCMmioV + IMC_COEF_R3);

				writel(imc_preset->yuv->coef_g[0],
				 IMCMmioV + IMC_COEF_G0);
				writel(imc_preset->yuv->coef_g[1],
				 IMCMmioV + IMC_COEF_G1);
				writel(imc_preset->yuv->coef_g[2],
				 IMCMmioV + IMC_COEF_G2);
				writel(imc_preset->yuv->coef_g[3],
				 IMCMmioV + IMC_COEF_G3);

				writel(imc_preset->yuv->coef_b[0],
				 IMCMmioV + IMC_COEF_B0);
				writel(imc_preset->yuv->coef_b[1],
				 IMCMmioV + IMC_COEF_B1);
				writel(imc_preset->yuv->coef_b[2],
				 IMCMmioV + IMC_COEF_B2);
				writel(imc_preset->yuv->coef_b[3],
				 IMCMmioV + IMC_COEF_B3);
			}

#ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
				if ((imc_preset->yuv->yuv2rgb &
					IMC_TRANSMODE_BIT) ==
					IMC_TRANSMODE_BT601) {
					writel((IMC_TRANSMODE_CUSTOM1 |
					 (imc_preset->yuv->yuv2rgb &
					 IMC_DITHER_BIT)),
					 IMCMmioV + IMC_YUV2RGB);
					writel(COEF_R0_BT601,
					 IMCMmioV + IMC_COEF_R0);
					writel(COEF_R1_BT601,
					 IMCMmioV + IMC_COEF_R1);
					writel(COEF_R2_BT601,
					 IMCMmioV + IMC_COEF_R2);
					writel(COEF_R3_BT601,
					 IMCMmioV + IMC_COEF_R3);
					writel(COEF_G0_BT601,
					 IMCMmioV + IMC_COEF_G0);
					writel(COEF_G1_BT601,
					 IMCMmioV + IMC_COEF_G1);
					writel(COEF_G2_BT601,
					 IMCMmioV + IMC_COEF_G2);
					writel(COEF_G3_BT601,
					 IMCMmioV + IMC_COEF_G3);
					writel(COEF_B0_BT601,
					 IMCMmioV + IMC_COEF_B0);
					writel(COEF_B1_BT601,
					 IMCMmioV + IMC_COEF_B1);
					writel(COEF_B2_BT601,
					 IMCMmioV + IMC_COEF_B2);
					writel(COEF_B3_BT601,
					 IMCMmioV + IMC_COEF_B3);
				}
				if ((imc_preset->yuv->yuv2rgb &
					IMC_TRANSMODE_BIT) ==
					IMC_TRANSMODE_BT709) {
					writel((IMC_TRANSMODE_CUSTOM1 |
					 (imc_preset->yuv->yuv2rgb &
					 IMC_DITHER_BIT)),
					 IMCMmioV + IMC_YUV2RGB);
					writel(COEF_R0_BT709,
					 IMCMmioV + IMC_COEF_R0);
					writel(COEF_R1_BT709,
					 IMCMmioV + IMC_COEF_R1);
					writel(COEF_R2_BT709,
					 IMCMmioV + IMC_COEF_R2);
					writel(COEF_R3_BT709,
					 IMCMmioV + IMC_COEF_R3);
					writel(COEF_G0_BT709,
					 IMCMmioV + IMC_COEF_G0);
					writel(COEF_G1_BT709,
					 IMCMmioV + IMC_COEF_G1);
					writel(COEF_G2_BT709,
					 IMCMmioV + IMC_COEF_G2);
					writel(COEF_G3_BT709,
					 IMCMmioV + IMC_COEF_G3);
					writel(COEF_B0_BT709,
					 IMCMmioV + IMC_COEF_B0);
					writel(COEF_B1_BT709,
					 IMCMmioV + IMC_COEF_B1);
					writel(COEF_B2_BT709,
					 IMCMmioV + IMC_COEF_B2);
					writel(COEF_B3_BT709,
					 IMCMmioV + IMC_COEF_B3);
				}
			}
#endif
		}
	}

	if (imc_preset->burst) {
		writel(imc_preset->burst->burst_en, IMCMmioV + IMC_BURST_EN);
		writel(imc_preset->burst->threshold, IMCMmioV + IMC_THRESHOLD);
	}

	if (imc_preset->imc_round_en)
		writel(*imc_preset->imc_round_en, IMCMmioV + IMC_ROUND_EN);

	dev[channel].chk_preset = 1;

	/*<<<<<< interuppt enable */
	spin_unlock_irqrestore(&dev[channel].imc_hw_lock, flags);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return 0;
}
EXPORT_SYMBOL(emxx_imc_set_preset);


/******************************************************************************
* MODULE   : emxx_imc_set_update_vsync
* FUNCTION : Set IMC V-sync Registers.
* RETURN   : 0            : success
*          : -EACCES(-13) : need to emxx_imc_preset()
*          : -ENODEV(-19) : id is invalid
*          : -EINVAL(-22) : imc_vsync is invalid
* NOTE     : none
******************************************************************************/
int emxx_imc_set_update_vsync(unsigned long id,
 struct emxx_imc_update_vsync *imc_vsync)
{
	char *IMCMmioV;
	int channel = IMC_CH0;
	unsigned long flags;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return -ENODEV;
	}

	if (imc_vsync == NULL) {
		printk_err("error! imc_vsync is NULL\n");
		return -EINVAL;
	}

	if (!dev[channel].chk_preset) {
		printk_err("error! need to \"emxx_imc_preset()\"\n");
		return -EACCES;
	}

	/*>>>>>> interuppt disable */
	spin_lock_irqsave(&dev[channel].imc_hw_lock, flags);

	IMCMmioV = dev[channel].IMCMmioV;

	if (imc_vsync->cpubufsel) {
		writel(*imc_vsync->cpubufsel,
		 IMCMmioV + IMC_CPUBUFSEL);
	}

	if (imc_vsync->wb) {
		writel(imc_vsync->wb->areaadr_p,
		 IMCMmioV + IMC_WB_AREAADR_P);
		writel(imc_vsync->wb->hoffset,
		 IMCMmioV + IMC_WB_HOFFSET);
		writel(imc_vsync->wb->format,
		 IMCMmioV + IMC_WB_FORMAT);
		writel(imc_vsync->wb->size,
		 IMCMmioV + IMC_WB_SIZE);
		writel(imc_vsync->wb->areaadr_q,
		 IMCMmioV + IMC_WB_AREAADR_Q);
		writel(imc_vsync->wb->bufsel,
		 IMCMmioV + IMC_WB_BUFSEL);
		writel(imc_vsync->wb->mposition,
		 IMCMmioV + IMC_WB_MPOSITION);
		writel(imc_vsync->wb->msize,
		 IMCMmioV + IMC_WB_MSIZE);
		writel(imc_vsync->wb->color,
		 IMCMmioV + IMC_BACKCOLOR);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			writel(imc_vsync->wb->bytelane,
			 IMCMmioV + IMC_WB_BYTELANE);
		writel(imc_vsync->wb->scanmode,
		 IMCMmioV + IMC_WB_SCANMODE);
	}

	if (imc_vsync->mirror)
		writel(*imc_vsync->mirror, IMCMmioV + IMC_MIRROR);

	if (imc_vsync->alphasel) {
		writel(imc_vsync->alphasel->alphasel0,
		 IMCMmioV + IMC_ALPHASEL0);
		writel(imc_vsync->alphasel->alphasel1,
		 IMCMmioV + IMC_ALPHASEL1);
	}

	if (imc_vsync->l0_scanmode) {
		writel(*imc_vsync->l0_scanmode,
		 IMCMmioV + IMC_L0_SCANMODE);
	}
	if (imc_vsync->l1a_scanmode) {
		writel(*imc_vsync->l1a_scanmode,
		 IMCMmioV + IMC_L1A_SCANMODE);
	}
	if (imc_vsync->l1b_scanmode) {
		writel(*imc_vsync->l1b_scanmode,
		 IMCMmioV + IMC_L1B_SCANMODE);
	}
	if (imc_vsync->l1c_scanmode) {
		writel(*imc_vsync->l1c_scanmode,
		 IMCMmioV + IMC_L1C_SCANMODE);
	}
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1 ||
	    channel == IMC_CH0) {
#else
	if (channel == IMC_CH0) {
#endif
		if (imc_vsync->l2a_scanmode) {
			writel(*imc_vsync->l2a_scanmode,
			 IMCMmioV + IMC_L2A_SCANMODE);
		}
		if (imc_vsync->l2b_scanmode) {
			writel(*imc_vsync->l2b_scanmode,
			 IMCMmioV + IMC_L2B_SCANMODE);
		}
		if (imc_vsync->bg_scanmode) {
			writel(*imc_vsync->bg_scanmode,
			 IMCMmioV + IMC_BG_SCANMODE);
		}
	}

	/*<<<<<< interuppt enable */
	spin_unlock_irqrestore(&dev[channel].imc_hw_lock, flags);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return 0;
}
EXPORT_SYMBOL(emxx_imc_set_update_vsync);


/******************************************************************************
* MODULE   : emxx_imc_set_update_reserve
* FUNCTION : Set IMC Update Target Registers
* RETURN   : 0            : success
*          : -EACCES(-13) : need to emxx_imc_preset()
*          : -ENODEV(-19) : id is invalid
*          : -EINVAL(-22) : imc_reserve is invalid
* NOTE     : none
******************************************************************************/
int emxx_imc_set_update_reserve(unsigned long id,
 struct emxx_imc_update_reserve *imc_reserve,
 imc_callback_func_refresh callback_refresh)
{
	char *IMCMmioV;
	int channel = IMC_CH0;
	unsigned long flags;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return -ENODEV;
	}

	IMCMmioV = dev[channel].IMCMmioV;

	writel(IMC_UPDATE_OFF, IMCMmioV + IMC_REFRESH);

	if (imc_reserve == NULL) {
		printk_err("error! imc_reserve is NULL\n");
		return -EINVAL;
	}

	if (!dev[channel].chk_preset) {
		printk_err("error! need to \"emxx_imc_preset()\"\n");
		return -EACCES;
	}

	/*>>>>>> interuppt disable */
	spin_lock_irqsave(&dev[channel].imc_hw_lock, flags);

	if (imc_reserve->l0) {
		writel(imc_reserve->l0->control,
		 IMCMmioV + IMC_L0_CONTROL);
		writel(imc_reserve->l0->format,
		 IMCMmioV + IMC_L0_FORMAT);
		writel(imc_reserve->l0->bufsel,
		 IMCMmioV + IMC_L0_BUFSEL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			writel(imc_reserve->l0->bytelane,
			 IMCMmioV + IMC_L0_BYTELANE);
		writel(imc_reserve->l0->keyenable,
		 IMCMmioV + IMC_L0_KEYENABLE);
		writel(imc_reserve->l0->keycolor,
		 IMCMmioV + IMC_L0_KEYCOLOR);
		writel(imc_reserve->l0->alpha,
		 IMCMmioV + IMC_L0_ALPHA);
		writel(imc_reserve->l0->resize,
		 IMCMmioV + IMC_L0_RESIZE);
		writel(imc_reserve->l0->mirror,
		 IMCMmioV + IMC_L0_MIRROR);
		writel(imc_reserve->l0->offset,
		 IMCMmioV + IMC_L0_OFFSET);
		writel(imc_reserve->l0->frameadr_p,
		 IMCMmioV + IMC_L0_FRAMEADR_P);
		writel(imc_reserve->l0->frameadr_q,
		 IMCMmioV + IMC_L0_FRAMEADR_Q);
		writel(imc_reserve->l0->position,
		 IMCMmioV + IMC_L0_POSITION);
		writel(imc_reserve->l0->size,
		 IMCMmioV + IMC_L0_SIZE);
		writel(imc_reserve->l0->mposition,
		 IMCMmioV + IMC_L0_MPOSITION);
		writel(imc_reserve->l0->msize,
		 IMCMmioV + IMC_L0_MSIZE);
	}

	if (imc_reserve->l1a) {
		writel(imc_reserve->l1a->control,
		 IMCMmioV + IMC_L1A_CONTROL);
		writel(imc_reserve->l1a->format,
		 IMCMmioV + IMC_L1A_FORMAT);
		writel(imc_reserve->l1a->bufsel,
		 IMCMmioV + IMC_L1A_BUFSEL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			writel(imc_reserve->l1a->bytelane,
			 IMCMmioV + IMC_L1A_BYTELANE);
		writel(imc_reserve->l1a->keyenable,
		 IMCMmioV + IMC_L1A_KEYENABLE);
		writel(imc_reserve->l1a->keycolor,
		 IMCMmioV + IMC_L1A_KEYCOLOR);
		writel(imc_reserve->l1a->alpha,
		 IMCMmioV + IMC_L1A_ALPHA);
		writel(imc_reserve->l1a->resize,
		 IMCMmioV + IMC_L1A_RESIZE);
		writel(imc_reserve->l1a->mirror,
		 IMCMmioV + IMC_L1A_MIRROR);
		writel(imc_reserve->l1a->offset,
		 IMCMmioV + IMC_L1A_OFFSET);
		writel(imc_reserve->l1a->frameadr_p,
		 IMCMmioV + IMC_L1A_FRAMEADR_P);
		writel(imc_reserve->l1a->frameadr_q,
		 IMCMmioV + IMC_L1A_FRAMEADR_Q);
		writel(imc_reserve->l1a->position,
		 IMCMmioV + IMC_L1A_POSITION);
		writel(imc_reserve->l1a->size,
		 IMCMmioV + IMC_L1A_SIZE);
		writel(imc_reserve->l1a->mposition,
		 IMCMmioV + IMC_L1A_MPOSITION);
		writel(imc_reserve->l1a->msize,
		 IMCMmioV + IMC_L1A_MSIZE);
	}

	if (imc_reserve->l1b) {
		writel(imc_reserve->l1b->control,
		 IMCMmioV + IMC_L1B_CONTROL);
		writel(imc_reserve->l1b->format,
		 IMCMmioV + IMC_L1B_FORMAT);
		writel(imc_reserve->l1b->bufsel,
		 IMCMmioV + IMC_L1B_BUFSEL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			writel(imc_reserve->l1b->bytelane,
			 IMCMmioV + IMC_L1B_BYTELANE);
		writel(imc_reserve->l1b->keyenable,
		 IMCMmioV + IMC_L1B_KEYENABLE);
		writel(imc_reserve->l1b->keycolor,
		 IMCMmioV + IMC_L1B_KEYCOLOR);
		writel(imc_reserve->l1b->alpha,
		 IMCMmioV + IMC_L1B_ALPHA);
		writel(imc_reserve->l1b->resize,
		 IMCMmioV + IMC_L1B_RESIZE);
		writel(imc_reserve->l1b->mirror,
		 IMCMmioV + IMC_L1B_MIRROR);
		writel(imc_reserve->l1b->offset,
		 IMCMmioV + IMC_L1B_OFFSET);
		writel(imc_reserve->l1b->frameadr_p,
		 IMCMmioV + IMC_L1B_FRAMEADR_P);
		writel(imc_reserve->l1b->frameadr_q,
		 IMCMmioV + IMC_L1B_FRAMEADR_Q);
		writel(imc_reserve->l1b->position,
		 IMCMmioV + IMC_L1B_POSITION);
		writel(imc_reserve->l1b->size,
		 IMCMmioV + IMC_L1B_SIZE);
		writel(imc_reserve->l1b->mposition,
		 IMCMmioV + IMC_L1B_MPOSITION);
		writel(imc_reserve->l1b->msize,
		 IMCMmioV + IMC_L1B_MSIZE);
	}

	if (imc_reserve->l1c) {
		writel(imc_reserve->l1c->control,
		 IMCMmioV + IMC_L1C_CONTROL);
		writel(imc_reserve->l1c->format,
		 IMCMmioV + IMC_L1C_FORMAT);
		writel(imc_reserve->l1c->bufsel,
		 IMCMmioV + IMC_L1C_BUFSEL);
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			writel(imc_reserve->l1c->bytelane,
			 IMCMmioV + IMC_L1C_BYTELANE);
		writel(imc_reserve->l1c->keyenable,
		 IMCMmioV + IMC_L1C_KEYENABLE);
		writel(imc_reserve->l1c->keycolor,
		 IMCMmioV + IMC_L1C_KEYCOLOR);
		writel(imc_reserve->l1c->alpha,
		 IMCMmioV + IMC_L1C_ALPHA);
		writel(imc_reserve->l1c->resize,
		 IMCMmioV + IMC_L1C_RESIZE);
		writel(imc_reserve->l1c->mirror,
		 IMCMmioV + IMC_L1C_MIRROR);
		writel(imc_reserve->l1c->offset,
		 IMCMmioV + IMC_L1C_OFFSET);
		writel(imc_reserve->l1c->frameadr_p,
		 IMCMmioV + IMC_L1C_FRAMEADR_P);
		writel(imc_reserve->l1c->frameadr_q,
		 IMCMmioV + IMC_L1C_FRAMEADR_Q);
		writel(imc_reserve->l1c->position,
		 IMCMmioV + IMC_L1C_POSITION);
		writel(imc_reserve->l1c->size,
		 IMCMmioV + IMC_L1C_SIZE);
		writel(imc_reserve->l1c->mposition,
		 IMCMmioV + IMC_L1C_MPOSITION);
		writel(imc_reserve->l1c->msize,
		 IMCMmioV + IMC_L1C_MSIZE);
	}

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1 ||
	    channel == IMC_CH0) {
#else
	if (channel == IMC_CH0) {
#endif
		if (imc_reserve->l2a) {
			writel(imc_reserve->l2a->control,
			 IMCMmioV + IMC_L2A_CONTROL);
			writel(imc_reserve->l2a->format,
			 IMCMmioV + IMC_L2A_FORMAT);
			writel(imc_reserve->l2a->bufsel,
			 IMCMmioV + IMC_L2A_BUFSEL);
			writel(imc_reserve->l2a->bytelane,
			 IMCMmioV + IMC_L2A_BYTELANE);
			writel(imc_reserve->l2a->resize,
			 IMCMmioV + IMC_L2A_RESIZE);
			writel(imc_reserve->l2a->mirror,
			 IMCMmioV + IMC_L2A_MIRROR);
			writel(imc_reserve->l2a->offset,
			 IMCMmioV + IMC_L2A_OFFSET);
			writel(imc_reserve->l2a->frameadr_yp,
			 IMCMmioV + IMC_L2A_FRAMEADR_YP);
			writel(imc_reserve->l2a->frameadr_up,
			 IMCMmioV + IMC_L2A_FRAMEADR_UP);
			writel(imc_reserve->l2a->frameadr_vp,
			 IMCMmioV + IMC_L2A_FRAMEADR_VP);
			writel(imc_reserve->l2a->frameadr_yq,
			 IMCMmioV + IMC_L2A_FRAMEADR_YQ);
			writel(imc_reserve->l2a->frameadr_uq,
			 IMCMmioV + IMC_L2A_FRAMEADR_UQ);
			writel(imc_reserve->l2a->frameadr_vq,
			 IMCMmioV + IMC_L2A_FRAMEADR_VQ);
			writel(imc_reserve->l2a->position,
			 IMCMmioV + IMC_L2A_POSITION);
			writel(imc_reserve->l2a->size,
			 IMCMmioV + IMC_L2A_SIZE);
			writel(imc_reserve->l2a->mposition,
			 IMCMmioV + IMC_L2A_MPOSITION);
			writel(imc_reserve->l2a->msize,
			 IMCMmioV + IMC_L2A_MSIZE);
		}

		if (imc_reserve->l2b) {
			writel(imc_reserve->l2b->control,
			 IMCMmioV + IMC_L2B_CONTROL);
			writel(imc_reserve->l2b->format,
			 IMCMmioV + IMC_L2B_FORMAT);
			writel(imc_reserve->l2b->bufsel,
			 IMCMmioV + IMC_L2B_BUFSEL);
			writel(imc_reserve->l2b->bytelane,
			 IMCMmioV + IMC_L2B_BYTELANE);
			writel(imc_reserve->l2b->resize,
			 IMCMmioV + IMC_L2B_RESIZE);
			writel(imc_reserve->l2b->mirror,
			 IMCMmioV + IMC_L2B_MIRROR);
			writel(imc_reserve->l2b->offset,
			 IMCMmioV + IMC_L2B_OFFSET);
			writel(imc_reserve->l2b->frameadr_yp,
			 IMCMmioV + IMC_L2B_FRAMEADR_YP);
			writel(imc_reserve->l2b->frameadr_up,
			 IMCMmioV + IMC_L2B_FRAMEADR_UP);
			writel(imc_reserve->l2b->frameadr_vp,
			 IMCMmioV + IMC_L2B_FRAMEADR_VP);
			writel(imc_reserve->l2b->frameadr_yq,
			 IMCMmioV + IMC_L2B_FRAMEADR_YQ);
			writel(imc_reserve->l2b->frameadr_uq,
			 IMCMmioV + IMC_L2B_FRAMEADR_UQ);
			writel(imc_reserve->l2b->frameadr_vq,
			 IMCMmioV + IMC_L2B_FRAMEADR_VQ);
			writel(imc_reserve->l2b->position,
			 IMCMmioV + IMC_L2B_POSITION);
			writel(imc_reserve->l2b->size,
			 IMCMmioV + IMC_L2B_SIZE);
			writel(imc_reserve->l2b->mposition,
			 IMCMmioV + IMC_L2B_MPOSITION);
			writel(imc_reserve->l2b->msize,
			 IMCMmioV + IMC_L2B_MSIZE);
		}

		if (imc_reserve->bg) {
			/* LayerBG Setting Register */
			writel(imc_reserve->bg->format,
			 IMCMmioV + IMC_BG_FORMAT);
			writel(imc_reserve->bg->bufsel,
			 IMCMmioV + IMC_BG_BUFSEL);
#ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
				writel(imc_reserve->bg->bytelane,
				 IMCMmioV + IMC_BG_BYTELANE);
			writel(imc_reserve->bg->resize,
			 IMCMmioV + IMC_BG_RESIZE);
			writel(imc_reserve->bg->mirror,
			 IMCMmioV + IMC_BG_MIRROR);
			writel(imc_reserve->bg->offset,
			 IMCMmioV + IMC_BG_OFFSET);
			writel(imc_reserve->bg->frameadr_p,
			 IMCMmioV + IMC_BG_FRAMEADR_P);
			writel(imc_reserve->bg->frameadr_q,
			 IMCMmioV + IMC_BG_FRAMEADR_Q);
			writel(imc_reserve->bg->mposition,
			 IMCMmioV + IMC_BG_MPOSITION);
			writel(imc_reserve->bg->msize,
			 IMCMmioV + IMC_BG_MSIZE);
		}
	}

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) >= EMXX_REV_ES3) {
		if (imc_reserve->rr) {
			if (channel == IMC_CH1) {
				/* AMODE_MB(24bit) = 0 */
				writel(imc_reserve->rr->reserved
				       & ~IMC_AMODE_MB,
				       IMCMmioV + IMC_RESERVED);
			}
		}
	}
#endif
	dev[channel].callback_func_refresh = callback_refresh;

	imc_hw_int_enable(channel);

	if (channel == IMC_CH1) {
		writel(IMC_UPDATE_ON, IMCMmioV + IMC_REFRESH);
		if (readl(IMCMmioV + IMC_REFRESH) == IMC_UPDATE_OFF)
			writel(IMC_UPDATE_ON, IMCMmioV + IMC_REFRESH);
	}

	/*<<<<<< interuppt enable */
	spin_unlock_irqrestore(&dev[channel].imc_hw_lock, flags);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return 0;
}
EXPORT_SYMBOL(emxx_imc_set_update_reserve);


void emxx_imc_set_refresh(unsigned long id)
{
	char *IMCMmioV;
	int channel = IMC_CH0;

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return;
	}

	IMCMmioV = dev[channel].IMCMmioV;
	if (channel == IMC_CH0) {
		writel(IMC_UPDATE_ON, IMCMmioV + IMC_REFRESH);
		if (readl(IMCMmioV + IMC_REFRESH) == IMC_UPDATE_OFF)
			writel(IMC_UPDATE_ON, IMCMmioV + IMC_REFRESH);
	}
}
EXPORT_SYMBOL(emxx_imc_set_refresh);


int emxx_imc_cancel_refresh(unsigned long id)
{
	char *IMCMmioV;
	int channel = IMC_CH0;
	int ret = 0;

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return 0;
	}

	IMCMmioV = dev[channel].IMCMmioV;
	if (channel == IMC_CH0) {
		if (readl(IMCMmioV + IMC_REFRESH) == IMC_UPDATE_ON) {
			writel(IMC_UPDATE_OFF, IMCMmioV + IMC_REFRESH);
			ret = 1;
		}
	}

	return ret;
}
EXPORT_SYMBOL(emxx_imc_cancel_refresh);


/******************************************************************************
* MODULE   : emxx_imc_start
* FUNCTION : Start the IMC in the immediate startup mode (STARTMODE = 1).
* RETURN   : 0            : success
*          : -EACCES(-13) : IMC startup mode is not Immediate startup mode,
*                           or need to emxx_imc_preset()
*          : -EBUSY (-16) : IMC Running
*          : -ENODEV(-19) : id is invalid
* NOTE     : none
******************************************************************************/
int emxx_imc_start(unsigned long id)
{
	int channel = IMC_CH0;
	unsigned long flags;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return -ENODEV;
	}

	if (!imc_hw_chk_startmode(channel)) {
		printk_err("error! IMC startup mode (IMC_CONTROL) is"
				" LCD-synchronous mode\n");
		return -EACCES;
	}

	if (!dev[channel].chk_preset) {
		printk_err("error! need to \"emxx_imc_preset()\"\n");
		return -EACCES;
	}

	if (imc_hw_chk_busy(channel)) {
		/* IMC running */
		printk_err("IMC WORKING NOW\n");
		return -EBUSY;
	}

	/* IMCW auto frequency control on */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) < EMXX_REV_ES3) {
#endif
	if (channel == IMC_CH1)
		writel(readl(SMU_CKRQMODE_MASK0) & ~0x80, SMU_CKRQMODE_MASK0);
#ifdef CONFIG_MACH_EMEV
	}
#endif

	/*>>>>>> interuppt disable */
	spin_lock_irqsave(&dev[channel].imc_hw_lock, flags);

	imc_hw_int_enable(channel);

	writel(IMC_IMCSTART_ON, dev[channel].IMCMmioV + IMC_START);

	/*<<<<<< interuppt enable */
	spin_unlock_irqrestore(&dev[channel].imc_hw_lock, flags);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return 0;
}
EXPORT_SYMBOL(emxx_imc_start);


/******************************************************************************
* MODULE   : emxx_imc_stop
* FUNCTION : Terminate the IMC.
* RETURN   : 0            : success
*          : -ENODEV(-19) : id is invalid
* NOTE     : none
******************************************************************************/
int emxx_imc_stop(unsigned long id)
{
	int channel = IMC_CH0;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return -ENODEV;
	}

	writel(IMC_IMCSTOP_ON, dev[channel].IMCMmioV + IMC_STOP);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return 0;
}
EXPORT_SYMBOL(emxx_imc_stop);


/******************************************************************************
* MODULE   : emxx_imc_set_callback
* FUNCTION :
* RETURN   : 0            : success
*          : -ENODEV(-19) : id is invalid
* NOTE     : none
******************************************************************************/
int emxx_imc_set_callback(unsigned long id,
 imc_callback_func_refresh callback_refresh, imc_callback_func_wb callback_wb)
{
	int channel = IMC_CH0;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (id == dev[IMC_CH0].id) {
		channel = IMC_CH0;
	} else if (id == dev[IMC_CH1].id) {
		channel = IMC_CH1;
	} else {
		printk_err("error! id is invalid\n");
		return -ENODEV;
	}

	dev[channel].callback_func_refresh = callback_refresh;
	dev[channel].callback_func_wb      = callback_wb;
	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return 0;
}
EXPORT_SYMBOL(emxx_imc_set_callback);


/* ----- internal function ------------------------------------------------- */
/******************************************************************************
* MODULE   : imc_hw_init
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void imc_hw_init(void)
{
	unsigned long flags;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	/*>>>>>> interuppt disable */
	spin_lock_irqsave(&dev[IMC_CH0].imc_hw_lock, flags);

	imc_hw_unreset(IMC_CH0);

	imc_hw_int_disable(IMC_CH0);

	/*<<<<<< interuppt enable */
	spin_unlock_irqrestore(&dev[IMC_CH0].imc_hw_lock, flags);

	/*>>>>>> interuppt disable */
	spin_lock_irqsave(&dev[IMC_CH1].imc_hw_lock, flags);

	imc_hw_unreset(IMC_CH1);

	imc_hw_int_disable(IMC_CH1);

	/*<<<<<< interuppt enable */
	spin_unlock_irqrestore(&dev[IMC_CH1].imc_hw_lock, flags);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}


/******************************************************************************
* MODULE   : imc_hw_reset
* FUNCTION : reset IMC module
* RETURN   : none
* NOTE     : none
******************************************************************************/
void imc_hw_reset(int channel)
{
	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (channel == IMC_CH0) {
		/* Reset IMC */
		emxx_clkctrl_off(EMXX_CLKCTRL_IMC);
		emxx_clkctrl_off(EMXX_CLKCTRL_IMCPCLK);
		emxx_reset_device(EMXX_RST_IMC);
		emxx_close_clockgate(EMXX_CLK_IMC | EMXX_CLK_IMC_P);
	} else {
		emxx_clkctrl_off(EMXX_CLKCTRL_IMCW);
		emxx_clkctrl_off(EMXX_CLKCTRL_IMCWPCLK);
		emxx_reset_device(EMXX_RST_IMCW);
		emxx_close_clockgate(EMXX_CLK_IMCW | EMXX_CLK_IMCW_P);
	}

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}


/******************************************************************************
* MODULE   : imc_hw_unreset
* FUNCTION : un-reset IMC module
* RETURN   : none
* NOTE     : none
******************************************************************************/
void imc_hw_unreset(int channel)
{
	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	if (channel == IMC_CH0) {
		/* UnReset IMC */
		emxx_open_clockgate(EMXX_CLK_IMC | EMXX_CLK_IMC_P);
		emxx_clkctrl_off(EMXX_CLKCTRL_IMC);
		emxx_clkctrl_off(EMXX_CLKCTRL_IMCPCLK);
		emxx_unreset_device(EMXX_RST_IMC);
		emxx_clkctrl_on(EMXX_CLKCTRL_IMC);
		emxx_clkctrl_on(EMXX_CLKCTRL_IMCPCLK);
	} else {
		/* UnReset IMCW */
		emxx_open_clockgate(EMXX_CLK_IMCW | EMXX_CLK_IMCW_P);
		emxx_clkctrl_off(EMXX_CLKCTRL_IMCW);
		emxx_clkctrl_off(EMXX_CLKCTRL_IMCWPCLK);
		emxx_unreset_device(EMXX_RST_IMCW);
		emxx_clkctrl_on(EMXX_CLKCTRL_IMCW);
		emxx_clkctrl_on(EMXX_CLKCTRL_IMCWPCLK);
	}

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}


/******************************************************************************
* MODULE   : imc_hw_int_enable
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void imc_hw_int_enable(int channel)
{

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	/* Interuppt Control Register */
	writel(IMC_INT_ALL_BIT, dev[channel].IMCMmioV + IMC_INTENSET);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}


/******************************************************************************
* MODULE   : imc_hw_int_disable
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void imc_hw_int_disable(int channel)
{
	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	/* Interuppt Control Register */
	writel(IMC_INT_ALL_BIT, dev[channel].IMCMmioV + IMC_INTENCLR);
	writel(IMC_INT_ALL_BIT, dev[channel].IMCMmioV + IMC_INTFFCLR);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}


/******************************************************************************
* MODULE   : imc_hw_chk_startmode
* FUNCTION :
* RETURN   : 0: LCD-synchronous mode
*          : 1: Immediate startup mode
* NOTE     : none
******************************************************************************/
static int imc_hw_chk_startmode(int channel)
{
	int iRet;
	unsigned long ulRegVal32;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");
	ulRegVal32 = readl(dev[channel].IMCMmioV + IMC_CONTROL);
	iRet = (((ulRegVal32 & IMC_START_MODE_BIT) == IMC_START_MODE_IMMEDIATE)
			? 1 : 0);
	printk_dbg((_DEBUG_IMC & 0x02), "out\n");

	return iRet;
}


/******************************************************************************
* MODULE   : imc_hw_chk_busy
* FUNCTION :
* RETURN   :  0: IMC Stopped
*          : -1: IMC Running
* NOTE     : none
******************************************************************************/
static int imc_hw_chk_busy(int channel)
{
	int iRet;
	unsigned long ulRegVal32;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");
	ulRegVal32 = readl(dev[channel].IMCMmioV + IMC_STATUS);
	iRet = (((ulRegVal32 & IMC_STATUS_BIT) == IMC_STATUS_STOP) ? 0 : -1);
	printk_dbg((_DEBUG_IMC & 0x02), "out\n");

	return iRet;
}


/******************************************************************************
* MODULE   : imc_hw_wait_stop
* FUNCTION : wait for IMC stopped
* RETURN   : none
* NOTE     : none
******************************************************************************/
void imc_hw_wait_stop(int channel)
{
	int iRet, wait_cnt = 0, wait_max = 40;

	/* wait for IMC stopped (max 40ms) */
	do {
		iRet = imc_hw_chk_busy(channel);
		if (iRet == 0) {
			printk_dbg((_DEBUG_IMC & 0x01), "wait(%dms)\n",
			 wait_cnt);
			wait_cnt = wait_max;
		} else {
			if (wait_cnt < wait_max)
				mdelay(1);
		}
		wait_cnt++;
	} while (wait_cnt <= wait_max);
}


#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/******************************************************************************
* MODULE   : imc_hw_chk_status
* FUNCTION : enabled suspend
* RETURN   :  0 : success
*            -1 : failed
* NOTE     : none
******************************************************************************/
static int imc_hw_chk_status(int channel)
{
	int iRet, i = 0;

	do {
		iRet = imc_hw_chk_busy(channel);
		if (iRet == 0) {
			i = 5;
		} else if (i == 4) {
			printk_err("imc_hw_chk_status\n");
			Call_StatusCtrlFunc_OFF(channel);
			return -1;
		} else {
			Call_StatusCtrlFunc_ON(channel);
			mdelay(16);
			i++;
		}
	} while (i < 5);

	return 0;
}
#endif /* CONFIG_PM || CONFIG_DPM */


/* ----- IMC IRQ handler --------------------------------------------------- */
/******************************************************************************
* MODULE   : imc_irq_handler
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static irqreturn_t imc_irq_handler(int irq, void *dev_id)
{
	imc_irq_handler_sub(irq, dev_id, IMC_CH0);
	return IRQ_HANDLED;
}


/******************************************************************************
* MODULE   : imcw_irq_handler
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static irqreturn_t imcw_irq_handler(int irq, void *dev_id)
{
	imc_irq_handler_sub(irq, dev_id, IMC_CH1);
	return IRQ_HANDLED;
}


/******************************************************************************
* MODULE   : imc_irq_handler_sub
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void imc_irq_handler_sub(int irq, void *dev_id, int channel)
{
	char *IMCMmioV;
	unsigned long ulRegVal32;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");
	IMCMmioV = dev[channel].IMCMmioV;

	ulRegVal32 = readl(IMCMmioV + IMC_INTSTATUS);
	if (ulRegVal32 & (IMC_REFRESH_BIT | IMC_WBEND_BIT | IMC_PWBEND_BIT)) {
		if (ulRegVal32 & IMC_REFRESH_BIT) {
			printk_dbg((_DEBUG_IMC & 0x02), "IMC_REFRESH\n");
			writel(IMC_UPDATE_OFF, IMCMmioV + IMC_REFRESH);
			writel(IMC_REFRESH_BIT, IMCMmioV + IMC_INTFFCLR);
			if (dev[channel].callback_func_refresh)
				(dev[channel].callback_func_refresh)();
		}
		if (ulRegVal32 & (IMC_WBEND_BIT | IMC_PWBEND_BIT)) {
			printk_dbg((_DEBUG_IMC & 0x02), "IMC_PWBEND\n");
			writel(IMC_WBEND_BIT | IMC_PWBEND_BIT,
			 IMCMmioV + IMC_INTFFCLR);
			/* IMCW auto frequency control off */
#ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) < EMXX_REV_ES3) {
#endif
			if (channel == IMC_CH1)
				writel(readl(SMU_CKRQMODE_MASK0) | 0x80,
				 SMU_CKRQMODE_MASK0);
#ifdef CONFIG_MACH_EMEV
			}
#endif
			if (dev[channel].callback_func_wb)
				(dev[channel].callback_func_wb)(ulRegVal32);
		}
	}

	/* error interrupt */
	else if (ulRegVal32 & IMC_SAXIWERR_BIT) {
		printk_err("IMC AXI slave write side error. Reset IMC.\n");
		writel(IMC_SAXIWERR_BIT, IMCMmioV + IMC_INTFFCLR);
		imc_hw_reset(channel);
		imc_hw_unreset(channel);
		dev[channel].chk_preset = 0;
		if (dev[channel].callback_func_wb)
			(dev[channel].callback_func_wb)(ulRegVal32);
	} else if (ulRegVal32 & IMC_MAXIRERR_BIT) {
		printk_err("IMC AXI master read side error. Reset IMC.\n");
		writel(IMC_MAXIRERR_BIT, IMCMmioV + IMC_INTFFCLR);
		imc_hw_reset(channel);
		imc_hw_unreset(channel);
		dev[channel].chk_preset = 0;
		if (dev[channel].callback_func_wb)
			(dev[channel].callback_func_wb)(ulRegVal32);
	} else if (ulRegVal32 & IMC_MAXIWERR_BIT) {
		printk_err("IMC AXI master write side error. Reset IMC.\n");
		writel(IMC_MAXIWERR_BIT, IMCMmioV + IMC_INTFFCLR);
		imc_hw_reset(channel);
		imc_hw_unreset(channel);
		dev[channel].chk_preset = 0;
		if (dev[channel].callback_func_wb)
			(dev[channel].callback_func_wb)(ulRegVal32);
	} else if (ulRegVal32 & IMC_OVERRUN_BIT) {
		printk_err("IMC OVERRUN ERROR. Retry IMC.\n");
		writel(IMC_OVERRUN_BIT, IMCMmioV + IMC_INTFFCLR);
	}

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}




#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/* ----- suspend/resume function ------------------------------------------- */
/******************************************************************************
* MODULE   : emxx_imc_suspend
* FUNCTION : suspend IMC driver
* RETURN   : none
* NOTE     : none
******************************************************************************/
int emxx_imc_suspend(struct platform_device *dev, pm_message_t state)
{
	if (imc_hw_chk_status(IMC_CH0) ||
	    imc_hw_chk_status(IMC_CH1))
		return -1;

	DPM_suspend_flg = 1;
	imc_hw_reset(IMC_CH0); /* Reset IMC */
	imc_hw_reset(IMC_CH1); /* Reset IMCW */
	return 0;
}


/******************************************************************************
* MODULE   : emxx_imc_resume
* FUNCTION : suspend IMC driver
* RETURN   : none
* NOTE     : none
******************************************************************************/
int emxx_imc_resume(struct platform_device *device)
{
	if (DPM_suspend_flg) {
		DPM_suspend_flg = 0;
		imc_hw_unreset(IMC_CH0); /* UnReset IMC */
		imc_hw_unreset(IMC_CH1); /* UnReset IMCW */

		if (dev[IMC_CH0].ctrl_func || dev[IMC_CH1].ctrl_func)
			tasklet_schedule(&imc_nexttask);
	}
	return 0;
}


/******************************************************************************
* MODULE   : imc_resume_kick
* FUNCTION : suspend IMC driver. call TASKLET
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void imc_resume_kick(unsigned long num)
{
	if (dev[IMC_CH0].ctrl_func) {

		Call_StatusCtrlFunc_OFF(IMC_CH0);

		if (dev[IMC_CH0].callback_func_wb) {
			printk_dbg((_DEBUG_IMC & 0x02), "\n");
			(dev[IMC_CH0].callback_func_wb)(IMC_WBEND_BIT |
							IMC_PWBEND_BIT);
		}
	}
	if (dev[IMC_CH1].ctrl_func) {

		Call_StatusCtrlFunc_OFF(IMC_CH1);

		if (dev[IMC_CH1].callback_func_wb) {
			printk_dbg((_DEBUG_IMC & 0x02), "\n");
			(dev[IMC_CH1].callback_func_wb)(IMC_WBEND_BIT |
							IMC_PWBEND_BIT);
		}
	}
}
#endif /* CONFIG_PM || CONFIG_DPM */


/* ----- init function ----------------------------------------------------- */
/******************************************************************************
* MODULE   : emxx_imc_init_module
* FUNCTION :
* RETURN   : 0    : success
*            other: fail
* NOTE     : none
******************************************************************************/
int emxx_imc_init_module(void)
{
	int iRet = 0;

	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	/* allocate device infomation */
	dev = kmalloc(sizeof(struct emxx_imc_device) * 2, GFP_KERNEL);
	if (!dev) {
		printk_err("could not allocate memory\n");
		iRet = -ENOMEM;
		goto alloc_err;
	}
	memset(dev, 0, sizeof(struct emxx_imc_device) * 2);
	dev[IMC_CH0].IMCMmio       = EMXX_IMC_BASE;
	dev[IMC_CH1].IMCMmio       = EMXX_IMCW_BASE;

	/* ioremap IMCMmio */
	dev[IMC_CH0].IMCMmioV = (char *)IO_ADDRESS(dev[IMC_CH0].IMCMmio);
	printk_dbg((_DEBUG_IMC),
	 "IMCMmio (0x%lx)  IMCMmioV (0x%p)\n",
	  dev[IMC_CH0].IMCMmio, dev[IMC_CH0].IMCMmioV);
	dev[IMC_CH1].IMCMmioV = (char *)IO_ADDRESS(dev[IMC_CH1].IMCMmio);
	printk_dbg((_DEBUG_IMC),
	 "IMCMmio (0x%lx)  IMCMmioV (0x%p)\n",
	  dev[IMC_CH1].IMCMmio, dev[IMC_CH1].IMCMmioV);

	/* spinlock initialize */
	spin_lock_init(&dev[IMC_CH0].imc_hw_lock);
	spin_lock_init(&dev[IMC_CH1].imc_hw_lock);

	init_waitqueue_head(&wait_que_resource);

	/* IMC initialize */
	imc_hw_init();

	/* install IMC irq handler */
	iRet = request_irq(INT_IMC, imc_irq_handler, 0, DEV_NAME, NULL);
	if (iRet) {
		printk_err("fail in request_irq(INT_IMC)\n");
		goto imc_irq_err;
	}
	iRet = request_irq(INT_IMCW, imcw_irq_handler, 0, DEV_NAME, NULL);
	if (iRet) {
		printk_err("fail in request_irq(INT_IMCW)\n");
		goto imcw_irq_err;
	}

	goto init_success;

imcw_irq_err:
	free_irq(INT_IMC, NULL);
imc_irq_err:
	imc_hw_reset(IMC_CH0); /* Reset IMC */
	imc_hw_reset(IMC_CH1); /* Reset IMCW */
	kfree(dev);
alloc_err:
init_success:

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
	return iRet;
}


#ifdef MODULE
/******************************************************************************
* MODULE   : emxx_imc_exit_module
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void emxx_imc_exit_module(void)
{
	printk_dbg((_DEBUG_IMC & 0x01), "in\n");

	imc_hw_int_disable(IMC_CH0);
	free_irq(INT_IMC, NULL);
	imc_hw_reset(IMC_CH0);

	imc_hw_int_disable(IMC_CH1);
	free_irq(INT_IMCW, NULL);
	imc_hw_reset(IMC_CH1);

	kfree(dev);

	printk_dbg((_DEBUG_IMC & 0x02), "out\n");
}


module_init(emxx_imc_init_module);
module_exit(emxx_imc_exit_module);
#else
device_initcall(emxx_imc_init_module);
#endif


MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("EMMA Mobile EV IMC Driver");
MODULE_LICENSE("GPL");

