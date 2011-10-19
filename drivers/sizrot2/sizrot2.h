/*
 *  File Name       : drivers/sizrot2/sizrot2.h
 *  Function        : SIZ/ROT Driver
 *  Release Version : Ver 1.07
 *  Release Date    : 2010.09.01
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#ifndef _SIZROT2_H_
#define _SIZROT2_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#ifdef CONFIG_VIDEO_EMXX
#include <mach/emxx_v4l2.h>
#endif
#include <mach/siz.h>
#include <mach/siz_ioctl.h>
#include <mach/rot2.h>
#include <mach/rot2_ioctl.h>
#include <mach/pmu.h>
#include <mach/dma.h>
#include <mach/hardware.h>

#include <asm/system.h>

/*===========================================================================*/
/* SIZ/ROT Driver Internal Structures                                        */
/*===========================================================================*/

/* SIZ resize filter parameter */
struct filter_param {
	unsigned long		filtopt;
	unsigned long		filt0;
	unsigned long		filt1;
	unsigned long		filt2;
	unsigned long		filt3;
	unsigned long		filt4;
	unsigned long		filt5;
	unsigned long		filt6;
	unsigned long		filt7;
};

struct coef_param {
	unsigned long		coef_r0;
	unsigned long		coef_r1;
	unsigned long		coef_r2;
	unsigned long		coef_r3;
	unsigned long		coef_g0;
	unsigned long		coef_g1;
	unsigned long		coef_g2;
	unsigned long		coef_g3;
	unsigned long		coef_b0;
	unsigned long		coef_b1;
	unsigned long		coef_b2;
	unsigned long		coef_b3;
};


/* SIZ resource information */
struct siz_info {
	char			*reg_base;
	unsigned long		sequence;
	unsigned long		id;
	struct emxx_siz_param	siz_param;
	unsigned int		dma_channel_yrgb;
	unsigned int		dma_channel_uv;
	unsigned int		dma_channel_v;
	struct emxx_dma_param	dma_param;
	int			cnt_callback;
	dma_callback_func	callback;
	u32			aadd_yrgb;
	u32			aadd_uv;
	u32			aadd_v;
	u32			badd_yrgb;
	u32			badd_uv;
	u32			badd_v;
	u32			size_yrgb;
	u32			size_uv;
	u32			size_v;
	u32			leng_yrgb;
	u32			leng_uv;
	u32			leng_v;
	int			cnt_strip;
	u32			strip_leng_yrgb;
	u32			strip_leng_uv;
	u32			strip_leng_v;
	int			strip_cnt_callback;
	struct workqueue_struct	*siz_workqueue;
	struct work_struct	wk_strip_dma_to_siz;
	struct work_struct	wk_start_dma_to_siz;
	wait_queue_head_t	wait_que_ioctl;
	wait_queue_head_t	wait_que_resource;
	int			dma_running;
	int			dma_line_count;
	struct filter_param	filter_linear;
	struct filter_param	filter_sharpness;
	struct filter_param	filter_4x4_smoothing;
	struct filter_param	filter_2x2_smoothing;
	struct filter_param	filter;
	struct coef_param	coef_yuv_rgb;
	struct coef_param	coef_rgb_yuv;
	struct coef_param	coef;
	int			change_format;
	unsigned char	device;
};

/* ROT resource information */
struct rot_info {
	char			*reg_base;
	unsigned long		sequence;
	unsigned long		id;
	struct emxx_rot_param	rot_param;
	int			rot_boundary;
	unsigned int		dma_channel;
	struct emxx_dma_param	dma_param;
	int			cnt_callback;
	dma_callback_func	callback;
	u32			aadd;
	u32			badd;
	u32			aoff;
	u32			boff;
	u32			size;
	u32			leng;
	int			cnt_strip;
	u32			strip_leng;
	struct workqueue_struct	*rot_workqueue;
	struct work_struct	wk_rotate_rgb888_sw;
	wait_queue_head_t	wait_que_ioctl;
	wait_queue_head_t	wait_que_resource;
	int			dma_running;
	int			dma_line_count;
	unsigned char		device;
};

/*===========================================================================*/
/* SIZ/ROT Driver Internal Definitions                                       */
/*===========================================================================*/
/* SIZ/ROT input address */
#define SIZ_ADRYRGB		EMXX_SIZ_DATA_BASE
#define SIZ_ADRUV		(EMXX_SIZ_DATA_BASE + 0x04000000)
#define SIZ_ADRV		(EMXX_SIZ_DATA_BASE + 0x06000000)
#define ROT2_LCH0_ADRYRGB	EMXX_ROT_DATA_BASE
#define ROT2_LCH0_ADRUV		(EMXX_ROT_DATA_BASE + 0x08000000)
#define ROT2_LCH0_ADRV		(EMXX_ROT_DATA_BASE + 0x0C000000)

/* sequence */
#define ID_SIZ_MIN		0x40000000
#define ID_SIZ_MAX		0x7FFFFFFF
#define ID_ROT_CH0_MIN		0x80000000
#define ID_ROT_CH0_MAX		0xBFFFFFFF
/* id */
#define ID_FUNC_BIT		0xC0000000
#define ID_FUNC_SHT		30
/* dma_running */
#define DMA_NOT_START		0
#define DMA_RUNNING		1
#define DMA_DONE		0
#define DMA_CANCELED		-1

#define DEV_MAJOR		194
#define DEV_MINOR_SIZ		0
#define DEV_MINOR_ROT0		1

#define MAX_SIZE_SIZROT2IOCTL sizeof(struct emxx_set_siz_info)

#define SIZ_FILTOPT_4x4_SHARPNESS	SIZ_FILTOPT_MODE_4x4FILT_MINUS
#define SIZ_FILT0_4x4_SHARPNESS		0x11050A08
#define SIZ_FILT1_4x4_SHARPNESS		0x2D081A1B
#define SIZ_FILT2_4x4_SHARPNESS		0x45072A30
#define SIZ_FILT3_4x4_SHARPNESS		0x57013C48
#define SIZ_FILT4_4x4_SHARPNESS		0x64F74A60
#define SIZ_FILT5_4x4_SHARPNESS		0x6CE95879
#define SIZ_FILT6_4x4_SHARPNESS		0x70D66293
#define SIZ_FILT7_4x4_SHARPNESS		0x70C26AAB
/* sharpness filter */

#define SIZ_FILTOPT_4x4_SMOOTHING	SIZ_FILTOPT_MODE_4x4FILT_PLUS
#define SIZ_FILT0_4x4_SMOOTHING		0x88720249
#define SIZ_FILT1_4x4_SMOOTHING		0x7E700A4C
#define SIZ_FILT2_4x4_SMOOTHING		0x746D1250
#define SIZ_FILT3_4x4_SMOOTHING		0x6A6C1853
#define SIZ_FILT4_4x4_SMOOTHING		0x606A2056
#define SIZ_FILT5_4x4_SMOOTHING		0x56682859
#define SIZ_FILT6_4x4_SMOOTHING		0x4C65325C
#define SIZ_FILT7_4x4_SMOOTHING		0x44633A5E
/* smooth filter */

#define SIZ_FILTOPT_2x2_LINEAR		SIZ_FILTOPT_MODE_2x2FILT
#define SIZ_FILT0_2x2_LINEAR		0x00F80008
#define SIZ_FILT1_2x2_LINEAR		0x00E80018
#define SIZ_FILT2_2x2_LINEAR		0x00D80028
#define SIZ_FILT3_2x2_LINEAR		0x00C80038
#define SIZ_FILT4_2x2_LINEAR		0x00B80048
#define SIZ_FILT5_2x2_LINEAR		0x00A80058
#define SIZ_FILT6_2x2_LINEAR		0x00980068
#define SIZ_FILT7_2x2_LINEAR		0x00880078
/* linear filter */

#define SIZ_FILTOPT_2x2_SMOOTHING	SIZ_FILTOPT_MODE_2x2FILT
#define SIZ_FILT0_2x2_SMOOTHING		0x00FF0001
#define SIZ_FILT1_2x2_SMOOTHING		0x00FD0003
#define SIZ_FILT2_2x2_SMOOTHING		0x00F6000A
#define SIZ_FILT3_2x2_SMOOTHING		0x00EA0016
#define SIZ_FILT4_2x2_SMOOTHING		0x00D90027
#define SIZ_FILT5_2x2_SMOOTHING		0x00C4003C
#define SIZ_FILT6_2x2_SMOOTHING		0x00AA0056
#define SIZ_FILT7_2x2_SMOOTHING		0x008E0072
/* smooth filter */

#define SIZ_FILTOPT_YUV_RGB		(5 << 8)
#define SIZ_COEF_R0_YUV_RGB		0x0000012A
#define SIZ_COEF_R1_YUV_RGB		0x00000205
#define SIZ_COEF_R2_YUV_RGB		0x00000000
#define SIZ_COEF_R3_YUV_RGB		0x00000000
#define SIZ_COEF_G0_YUV_RGB		0x0000012A
#define SIZ_COEF_G1_YUV_RGB		0x0000079C
#define SIZ_COEF_G2_YUV_RGB		0x00000730
#define SIZ_COEF_G3_YUV_RGB		0x00000000
#define SIZ_COEF_B0_YUV_RGB		0x0000012A
#define SIZ_COEF_B1_YUV_RGB		0x00000000
#define SIZ_COEF_B2_YUV_RGB		0x00000199
#define SIZ_COEF_B3_YUV_RGB		0x00000000


#define SIZ_FILTOPT_RGB_YUV		(7 << 8)
#define SIZ_COEF_R0_RGB_YUV		0x000007DA
#define SIZ_COEF_R1_RGB_YUV		0x00000070
#define SIZ_COEF_R2_RGB_YUV		0x000007B6
#define SIZ_COEF_R3_RGB_YUV		0x00000000
#define SIZ_COEF_G0_RGB_YUV		0x00000070
#define SIZ_COEF_G1_RGB_YUV		0x000007EE
#define SIZ_COEF_G2_RGB_YUV		0x000007A2
#define SIZ_COEF_G3_RGB_YUV		0x00000000
#define SIZ_COEF_B0_RGB_YUV		0x00000042
#define SIZ_COEF_B1_RGB_YUV		0x00000019
#define SIZ_COEF_B2_RGB_YUV		0x00000081
#define SIZ_COEF_B3_RGB_YUV		0x00000000

#define CHG_NORMAL		0x0
#define CHG_YUV_RGB		0x1
#define CHG_RGB_YUV		0x2

#define MAX_TIME_TO_WAIT	1000	/* usec */

#define DMA_NOT_INITIALIZED	NULL
#define DMAC_LENG_MAX		16777215	/* ((2^24)-1) */

/*===========================================================================*/
/* SIZ/ROT Driver Internal Variables                                         */
/*===========================================================================*/
static const char *sizrot2_dev_name = "sizrot2";
static struct siz_info siz_info;
static struct rot_info rot_info;
static spinlock_t sizrot2_lock;

/*===========================================================================*/
/* performance measurement                                                   */
/*===========================================================================*/
#define _SIZROT2_PERF	0 /* 0: disable
			     1: enable M2M-DMA start - end
			     2: enable M2M-DMA start - SIZ end
			   */
#if _SIZROT2_PERF > 0
static struct timeval	tv_start, tv_stop;
static unsigned long	delta;

#define dbg_CalcTime(num)                                                \
	do {                                                             \
		delta = (tv_stop.tv_sec - tv_start.tv_sec) * 1000 * 1000 \
			+ (tv_stop.tv_usec - tv_start.tv_usec);          \
		printk(KERN_DEBUG " @sizrot2: " num                      \
		 ": ->cycle(%ld.%03ld)msec  ->start(%ld)  ->end(%ld)\n", \
		 delta / 1000, delta % 1000,                             \
		 tv_start.tv_usec, tv_stop.tv_usec);                     \
	} while (0);

#define dbg_GetStartTime(level)             \
	if (_SIZROT2_PERF == level) {       \
		do_gettimeofday(&tv_start); \
	}
#define dbg_GetStopTime(level, num)         \
	if (_SIZROT2_PERF == level) {       \
		do_gettimeofday(&tv_stop);  \
		dbg_CalcTime(num);          \
	}
#else
#define dbg_GetStartTime(level)		;
#define dbg_GetStopTime(level, num)	;
#endif /* _SIZROT2_PERF */

#endif /* _SIZROT2_H_ */
