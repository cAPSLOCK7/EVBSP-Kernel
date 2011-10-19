/*
 * File Name       : drivers/nts/emxx_nts_image.c
 * Function        : NTSC Driver (SIZ/ROT Control)
 * Release Version : Ver 1.00
 * Release Date    : 2010.09.27
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
#include <linux/uaccess.h>

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <mach/pm.h>
#endif /* CONFIG_PM || CONFIG_DPM */

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/pmu.h>
#include <mach/emxx_mem.h>

#include "../video/emxx/emxx_common.h"
#include "ntsc.h"
#include "emxx_nts_common.h"
#include "emxx_nts.h"
#include "emxx_ntshw.h"
#include "emxx_nts_image.h"


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define DEV_NAME "emxx_nts_image"


/********************************************************
 *  debug parameters                                    *
 *******************************************************/
#define _DEBUG_NTS_IMAGE  0x00 /* 00008421(bit) */
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

#if _DEBUG_NTS_IMAGE
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
enum old_frame_max {
	OLD_FRAME_A = 0,
	OLD_FRAME_B = 1,
	OLD_MAX     = 2,
};

/* request data. from fb/V4L2 driver */
enum image_max {
	IMAGE_SRC_B = 0,
	IMAGE_SRC_F = 1,
	IMAGE_DST   = 2,
	IMAGE_MAX   = 3,
};

enum fillrect_max {
	FILLRECT_1   = 0,
	FILLRECT_MAX = 1,
};

enum siz_transfer_max {
	SIZ_1   = 0,
	SIZ_2   = 1,
	SIZ_MAX = 2,
};

/* request data. for SIZ/ROT driver */
struct emxx_nts_image {
	struct emxx_nts_dev *ntsc;
	struct image_data    *old[OLD_MAX];
	struct image_data    *image[IMAGE_MAX];
	struct image_data    *fillrect[FILLRECT_MAX];

	struct emxx_siz_info  siz_info;
	struct emxx_rot_info  rot_info;
	struct emxx_siz_param siz_param;
	struct emxx_rot_param rot_param;
	struct emxx_dma_param dma_param;
};
static struct emxx_nts_image *nts_image;


/********************************************************
 *  Prototype declarations of local function                  *
 *******************************************************/
/* ------------------ private function < image compose > -------------------- */
static int  nts_image_prepare(void);
static int  nts_image_resize(void);
static void nts_image_callback_resize(int flag);
static void nts_image_change_frame(void);
/* ------------------ private function < fillrest > ------------------------- */
static int  nts_image_chk_fillrect(void);
static int  nts_image_fillrect_request(int index);


/* ------------------ public function -------------------------------------- */
/******************************************************************************
* MODULE   : nts_image_initialize
* FUNCTION :
* RETURN   :        0 : success
	   : -ENOEMEM : failed
* NOTE     : none
******************************************************************************/
int nts_image_initialize(struct emxx_nts_dev *ntsc)
{
	int i, iRet;

	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	/* allocate request data. from fb/V4L2 driver */
	nts_image = kmalloc(sizeof(struct emxx_nts_image), GFP_KERNEL);
	if (!nts_image)
		goto fail_alloc_mem;

	memset(nts_image, 0, sizeof(struct emxx_nts_image));
	nts_image->ntsc = ntsc;

	/* allocate old request data. from fb/V4L2 driver */
	for (i = 0; i < OLD_MAX; i++) {
		nts_image->old[i] =
		 kmalloc(sizeof(struct image_data), GFP_KERNEL);
		if (!nts_image->old[i])
			goto fail_alloc_mem;

		memset(nts_image->old[i], 0, sizeof(struct image_data));
	}
	/* allocate request data. from fb/V4L2 driver */
	for (i = IMAGE_SRC_B; i < IMAGE_MAX; i++) {
		nts_image->image[i] =
		 kmalloc(sizeof(struct image_data), GFP_KERNEL);
		if (!nts_image->image[i])
			goto fail_alloc_mem;

		memset(nts_image->image[i], 0, sizeof(struct image_data));
	}
	/* allocate fillrect data. */
	for (i = FILLRECT_1; i < FILLRECT_MAX; i++) {
		nts_image->fillrect[i] = kmalloc(sizeof(struct image_data),
		 GFP_KERNEL);
		if (!nts_image->fillrect[i])
			goto fail_alloc_mem;

		memset(nts_image->fillrect[i], 0, sizeof(struct image_data));
	}

	iRet = 0;
	goto success;

fail_alloc_mem:
	printk_wrn("memory allocate err");
	nts_image_finalize();
	iRet = -ENOMEM;
success:
	return iRet;
}


/******************************************************************************
* MODULE   : nts_image_finalize
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
void nts_image_finalize(void)
{
	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	if (nts_image) {
		int i;

		/* free fillrect data. */
		for (i = FILLRECT_1; i < FILLRECT_MAX; i++) {
			if (nts_image->fillrect[i]) {
				memset(nts_image->fillrect[i], 0,
				 sizeof(struct image_data));
				kfree(nts_image->fillrect[i]);
			}
		}
		/* free request data. from fb/V4L2 driver */
		for (i = IMAGE_SRC_B; i < IMAGE_MAX; i++) {
			if (nts_image->image[i]) {
				memset(nts_image->image[i], 0,
				 sizeof(struct image_data));
				kfree(nts_image->image[i]);
			}
		}
		/* free old request data. from fb/V4L2 driver */
		for (i = 0; i < OLD_MAX; i++) {
			if (nts_image->old[i]) {
				memset(nts_image->old[i], 0,
				 sizeof(struct image_data));
				kfree(nts_image->old[i]);
			}
		}
		memset(nts_image, 0, sizeof(struct emxx_nts_image));
		kfree(nts_image);
	}
}


/*****************************************************************************
* MODULE   : 	nts_image_clr_framebuf
* FUNCTION :
* RETURN   : -
* NOTE     :
******************************************************************************/
int nts_image_clr_framebuf(struct image_data *buff)
{
	int index = 0;

	memcpy(nts_image->fillrect[index], buff, sizeof(struct image_data));

	return nts_image_fillrect_request(index);
}


/******************************************************************************
* MODULE   : nts_image_compose_request
* FUNCTION : changing frame and callback to fb and v4l2
* RETURN   :       0 : success
	   : -EINVAL : failed
* NOTE     : none
******************************************************************************/
int nts_image_compose_request(void)
{
	int iRet;

	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	nts_image->ntsc->compose_complete = 0;

	iRet = nts_image_prepare();
	if (iRet) {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "(%d) error\n",
		 __LINE__);
		goto failed;
	}

	/* check framebuffer fillrect */
	nts_image_chk_fillrect();

	iRet = nts_image_resize();
	if (iRet) {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "(%d) error\n",
		 __LINE__);
		goto failed;
	}

failed:
	return iRet;
}


/* ------------------ private function < image compose > -------------------- */
/*****************************************************************************
* MODULE   : nts_image_prepare
* FUNCTION :
* RETURN   : -
* NOTE     :
******************************************************************************/
static int nts_image_prepare(void)
{
	int i;
	int iRet = 0;

	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	for (i = IMAGE_SRC_B; i < IMAGE_MAX; i++)
		memset(nts_image->image[i], 0, sizeof(struct image_data));

	switch (nts_image->ntsc->iMixImageMode) {
	case V4L2_ONLY:
		/* copy Src(Back) data */
		nts_image->image[IMAGE_SRC_B]->uiWidth        =
			nts_image->ntsc->from_v4l2.image_data.hsize;
		nts_image->image[IMAGE_SRC_B]->uiHeight       =
			nts_image->ntsc->from_v4l2.image_data.vsize;
		nts_image->image[IMAGE_SRC_B]->uiX            =
			nts_image->ntsc->from_v4l2.image_data.x;
		nts_image->image[IMAGE_SRC_B]->uiY            =
			nts_image->ntsc->from_v4l2.image_data.y;
		nts_image->image[IMAGE_SRC_B]->uiScreenWidth  =
			nts_image->ntsc->from_v4l2.image_data.size;
		switch (nts_image->ntsc->from_v4l2.yuvfmt) {
		case V4L2_FORMAT_YUV420Pl2: /* YUV420 Semi-Planar */
			nts_image->image[IMAGE_SRC_B]->uiFormat =
				SIZ_FORMAT_YUV420SP;
			break;
		case V4L2_FORMAT_YUV422Pl2: /* YUV422 Semi-Planar */
			nts_image->image[IMAGE_SRC_B]->uiFormat =
				SIZ_FORMAT_YUV422SP;
			break;
		case V4L2_FORMAT_YUV420Pl: /* YUV420 Planar      */
			nts_image->image[IMAGE_SRC_B]->uiFormat =
				SIZ_FORMAT_YUV420PL;
			break;
		case V4L2_FORMAT_YUV422Pl: /* YUV422 Planar      */
			nts_image->image[IMAGE_SRC_B]->uiFormat =
				SIZ_FORMAT_YUV422PL;
			break;
		case V4L2_FORMAT_YUV422Px: /* YUV422 Interleave  */
			nts_image->image[IMAGE_SRC_B]->uiFormat =
				SIZ_FORMAT_YUV422IL;
			break;
		default:
			iRet = -EINVAL;
			break;
		}

		nts_image->image[IMAGE_SRC_B]->ulPhysAddrYRGB =
			nts_image->ntsc->from_v4l2.image_data.yrgbaddr;
		nts_image->image[IMAGE_SRC_B]->ulPhysAddrUV   =
			nts_image->ntsc->from_v4l2.image_data.uvaddr;
		nts_image->image[IMAGE_SRC_B]->ulPhysAddrV    =
			nts_image->ntsc->from_v4l2.image_data.vaddr;

		/* copy Destination data */
		nts_image->image[IMAGE_DST]->uiWidth          =
			nts_image->ntsc->from_v4l2.screen_data.hsize;
		nts_image->image[IMAGE_DST]->uiHeight         =
			nts_image->ntsc->from_v4l2.screen_data.vsize;
		nts_image->image[IMAGE_DST]->uiX              =
			nts_image->ntsc->from_v4l2.screen_data.x;
		nts_image->image[IMAGE_DST]->uiY              =
			nts_image->ntsc->from_v4l2.screen_data.y;
		nts_image->image[IMAGE_DST]->uiScreenWidth    =
			NTS_WIDTH;
		/* dst_format is always SIZ_FORMAT_YUV422SP */
		nts_image->image[IMAGE_DST]->uiFormat         =
			SIZ_FORMAT_YUV422SP;
		if (nts_image->ntsc->iFrameNoNow == NTS_DISP_FRAME_A) {
			nts_image->image[IMAGE_DST]->ulPhysAddrYRGB =
			NTSC_SMEM_START + NTSCFRAME_B_OFFSET;
		} else {
			nts_image->image[IMAGE_DST]->ulPhysAddrYRGB =
			NTSC_SMEM_START + NTSCFRAME_A_OFFSET;
		}
		nts_image->image[IMAGE_DST]->ulPhysAddrUV     =
			nts_image->image[IMAGE_DST]->ulPhysAddrYRGB +
			NTS_WIDTH * NTS_HEIGHT;
		nts_image->image[IMAGE_DST]->ulPhysAddrV      = 0;

		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			"=== nts_image_prepare(V4L2_ONLY) "
			"===============================\n");

		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
		 " SRC_F | Y(%08lx)  U(%08lx)  V(%08lx)\n",
		 nts_image->image[IMAGE_SRC_F]->ulPhysAddrYRGB,
		 nts_image->image[IMAGE_SRC_F]->ulPhysAddrUV,
		 nts_image->image[IMAGE_SRC_F]->ulPhysAddrV);
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
		 "       | p(%3d)  w(%3d)  h(%3d)   x(%3d)  y(%3d)\n",
		 nts_image->image[IMAGE_SRC_F]->uiScreenWidth,
		 nts_image->image[IMAGE_SRC_F]->uiWidth,
		 nts_image->image[IMAGE_SRC_F]->uiHeight,
		 nts_image->image[IMAGE_SRC_F]->uiX,
		 nts_image->image[IMAGE_SRC_F]->uiY);
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			"-------+-----------------------"
			"-------------------------------\n");
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
		 " SRC_B | Y(%08lx)  U(%08lx)  V(%08lx)\n",
		 nts_image->image[IMAGE_SRC_B]->ulPhysAddrYRGB,
		 nts_image->image[IMAGE_SRC_B]->ulPhysAddrUV,
		 nts_image->image[IMAGE_SRC_B]->ulPhysAddrV);
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
		 "       | p(%3d)  w(%3d)  h(%3d)   x(%3d)  y(%3d)\n",
		 nts_image->image[IMAGE_SRC_B]->uiScreenWidth,
		 nts_image->image[IMAGE_SRC_B]->uiWidth,
		 nts_image->image[IMAGE_SRC_B]->uiHeight,
		 nts_image->image[IMAGE_SRC_B]->uiX,
		 nts_image->image[IMAGE_SRC_B]->uiY);
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			"-------+-----------------------"
			"-------------------------------\n");
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
		 " DST   | Y(%08lx)  U(%08lx)  V(%08lx)\n",
		 nts_image->image[IMAGE_DST]->ulPhysAddrYRGB,
		 nts_image->image[IMAGE_DST]->ulPhysAddrUV,
		 nts_image->image[IMAGE_DST]->ulPhysAddrV);
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
		 "       | p(%3d)  w(%3d)  h(%3d)   x(%3d)  y(%3d)\n",
		 nts_image->image[IMAGE_DST]->uiScreenWidth,
		 nts_image->image[IMAGE_DST]->uiWidth,
		 nts_image->image[IMAGE_DST]->uiHeight,
		 nts_image->image[IMAGE_DST]->uiX,
		 nts_image->image[IMAGE_DST]->uiY);
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			"==============================="
			"===============================\n\n");
		break;
	default:
		iRet = -EINVAL;
		break;
	}

	return iRet;
}


/******************************************************************************
* MODULE   : nts_image_resize
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static int nts_image_resize(void)
{
	int iRet;

	struct emxx_siz_info *siz_info = &nts_image->ntsc->from_v4l2.siz_info;
	struct emxx_rot_info *rot_info = &nts_image->ntsc->from_v4l2.rot_info;

	struct emxx_siz_param *siz_param = &nts_image->siz_param;
	struct emxx_rot_param *rot_param = &nts_image->rot_param;
	struct emxx_dma_param *dma_param = &nts_image->dma_param;

	unsigned int src_format   = nts_image->image[IMAGE_SRC_B]->uiFormat;
	unsigned int src_x        = nts_image->image[IMAGE_SRC_B]->uiX;
	unsigned int src_y        = nts_image->image[IMAGE_SRC_B]->uiY;
	unsigned int src_width    = nts_image->image[IMAGE_SRC_B]->uiWidth;
	unsigned int src_height   = nts_image->image[IMAGE_SRC_B]->uiHeight;
	unsigned int src_size   = nts_image->image[IMAGE_SRC_B]->uiScreenWidth;
	unsigned long src_adryrgb;
	unsigned long src_adruv;
	unsigned long src_adrv;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	unsigned int image_width  = nts_image->ntsc->from_v4l2.image_width;
	unsigned int image_height = nts_image->ntsc->from_v4l2.image_height;
#endif
	unsigned int dst_format   = nts_image->image[IMAGE_DST]->uiFormat;
	unsigned int dst_x        = nts_image->image[IMAGE_DST]->uiX;
	unsigned int dst_y        = nts_image->image[IMAGE_DST]->uiY;
	unsigned int dst_width    = nts_image->image[IMAGE_DST]->uiWidth;
	unsigned int dst_height   = nts_image->image[IMAGE_DST]->uiHeight;
	unsigned int dst_size     = nts_image->image[IMAGE_DST]->uiScreenWidth;
	unsigned long dst_adryrgb;
	unsigned long dst_adruv;
	unsigned long dst_adrv;
	unsigned int field        = nts_image->ntsc->from_v4l2.field;
	struct v4l2_filter_option *filter = &nts_image->ntsc->from_v4l2.filter;
	unsigned int angle        = nts_image->ntsc->from_v4l2.angle;

	unsigned int siz_dst_width;
	unsigned int siz_dst_height;
	int bpp_y, bpp_uv, bpp_v; /* bit par pixel  */
	int ppl_y, ppl_uv, ppl_v; /* pixel par line */

	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	memset(siz_param, 0, sizeof(struct emxx_siz_param));
	memset(rot_param, 0, sizeof(struct emxx_rot_param));
	memset(dma_param, 0, sizeof(struct emxx_dma_param));

	if (angle == ROT2_MODE_MODE_0 || angle == ROT2_MODE_MODE_180) {
		siz_dst_width  = dst_width;
		siz_dst_height = dst_height;
	} else {
		siz_dst_width  = dst_height;
		siz_dst_height = dst_width;
	}

	switch (src_format) {
	default:
	case SIZ_FORMAT_YUV420SP:
		bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
		ppl_y = 1; ppl_uv = 2; ppl_v = 1;
		break;
	case SIZ_FORMAT_YUV422SP:
		bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		break;
	case SIZ_FORMAT_YUV420PL:
		bpp_y = 8;  bpp_uv = 4; bpp_v = 4;
		ppl_y = 1; ppl_uv = 2; ppl_v = 2;
		break;
	case SIZ_FORMAT_YUV422PL:
		bpp_y = 8;  bpp_uv = 4; bpp_v = 4;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		break;
	case SIZ_FORMAT_YUV422IL:
		bpp_y = 16; bpp_uv = 0; bpp_v = 0;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		break;
	}

	src_adryrgb = nts_image->image[IMAGE_SRC_B]->ulPhysAddrYRGB
		      + (src_y / ppl_y) * src_size
		      + src_x * bpp_y / 8;
	src_adruv   = nts_image->image[IMAGE_SRC_B]->ulPhysAddrUV
		      + (src_y / ppl_uv) * (src_size * bpp_uv / bpp_y)
		      + src_x * bpp_uv / 8;
	src_adrv    = nts_image->image[IMAGE_SRC_B]->ulPhysAddrV
		      + (src_y / ppl_v) * (src_size * bpp_v / bpp_y)
		      + src_x * bpp_v / 8;

	/* dst_format is always SIZ_FORMAT_YUV422SP */
	dst_adryrgb = nts_image->image[IMAGE_DST]->ulPhysAddrYRGB
		      + dst_y * dst_size
		      + dst_x;
	dst_adruv   = nts_image->image[IMAGE_DST]->ulPhysAddrUV
		      + dst_y * dst_size
		      + dst_x;
	dst_adrv    = nts_image->image[IMAGE_DST]->ulPhysAddrV;


	/* set SIZ parameter */
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	if (image_width != siz_dst_width || image_height != siz_dst_height ||
#else
	if (src_width != siz_dst_width || src_height != siz_dst_height ||
#endif
	    field == V4L2_FIELD_INTERLACED ||
	    field == V4L2_FIELD_INTERLACED_TB ||
	    field == V4L2_FIELD_INTERLACED_BT ||
	    src_format != dst_format ||
	    angle == ROT2_MODE_MODE_0 ||
	    dst_width != dst_size) {
		siz_param->src_hsize	= src_width;
		siz_param->src_vsize	= src_height;
		siz_param->src_format	= src_format;
		siz_param->dst_hsize	= siz_dst_width;
		siz_param->dst_vsize	= siz_dst_height;
		if (angle != ROT2_MODE_MODE_0) {
			siz_param->dst_hskip	= 0x1000 - siz_dst_width;
			siz_param->dst_adryrgb	= rot_info->adryrgb;
			siz_param->dst_adruv	= rot_info->adruv;
			siz_param->dst_adrv	= rot_info->adrv;
			if (dst_width != dst_size) {
				switch (angle) {
				default:
				case ROT2_MODE_MODE_90:
					siz_param->dst_adryrgb	+=
					 0x1000 *
					 (dst_size - (dst_x + dst_width));
					siz_param->dst_adruv	+=
					 0x1000 *
					 (dst_size - (dst_x + dst_width));
					break;
				case ROT2_MODE_MODE_180:
					siz_param->dst_adryrgb	+=
					 (dst_size - (dst_x + dst_width));
					siz_param->dst_adruv	+=
					 (dst_size - (dst_x + dst_width));
					break;
				case ROT2_MODE_MODE_270:
					siz_param->dst_adryrgb	+=
					 0x1000 * dst_x;
					siz_param->dst_adruv	+=
					 0x1000 * dst_x;
					break;
				}
			}
		} else {
			siz_param->dst_hskip	= dst_size - siz_dst_width;
			siz_param->dst_adryrgb	= dst_adryrgb;
			siz_param->dst_adruv	= dst_adruv;
			siz_param->dst_adrv	= dst_adrv;
		}
		siz_param->dst_format	= dst_format;
		siz_param->dst_bytelane	= SIZ_DSTBL_RESET;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		siz_param->hstep = 256*image_width/siz_dst_width;
		siz_param->vstep = 256*image_height/siz_dst_height;
#else
		siz_param->hstep	= 256*src_width/siz_dst_width;
		siz_param->vstep	= 256*src_height/siz_dst_height;
#endif
		siz_param->dst_hcrop	= 0;
		siz_param->dst_vcrop	= 0;
		siz_param->rot_dst_format	= SIZ_ROTDSTFMT_OFF;
#ifdef CONFIG_VIDEO_EMXX_FILTER
		siz_param->filter_option	= filter->filter_option;

		if (field != V4L2_FIELD_INTERLACED &&
		    field != V4L2_FIELD_INTERLACED_TB &&
		    field != V4L2_FIELD_INTERLACED_BT &&
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		    image_width  == siz_dst_width &&
		    image_height == siz_dst_height)
#else
		    src_width  == siz_dst_width &&
		    src_height == siz_dst_height)
#endif
			siz_param->filter_option |=
			 (SIZ_FILTOPT_X_FILTER_THROUGH |
			  SIZ_FILTOPT_Y_FILTER_THROUGH);
		siz_param->filt0	= filter->filt0;
		siz_param->filt1	= filter->filt1;
		siz_param->filt2	= filter->filt2;
		siz_param->filt3	= filter->filt3;
		siz_param->filt4	= filter->filt4;
		siz_param->filt5	= filter->filt5;
		siz_param->filt6	= filter->filt6;
		siz_param->filt7	= filter->filt7;
#else
		siz_param->filter_option = SIZ_FILTER_DEFAULT;
#endif

		emxx_set_siz(siz_info->id, siz_param);
	}

	/* set ROT parameter */
	if (angle != ROT2_MODE_MODE_0) {
		switch (angle) {
		default:
		case ROT2_MODE_MODE_0: /*   0 deg */
			rot_param->mode =
			 ROT2_MODE_MODE_0   | ROT2_MODE_BOUNDARY_2_12;
			break;
		case ROT2_MODE_MODE_90: /*  90 deg */
			rot_param->mode =
			 ROT2_MODE_MODE_90  | ROT2_MODE_BOUNDARY_2_12;
			break;
		case ROT2_MODE_MODE_180: /* 180 deg */
			rot_param->mode =
			 ROT2_MODE_MODE_180 | ROT2_MODE_BOUNDARY_2_12;
			break;
		case ROT2_MODE_MODE_270: /* 270 deg */
			rot_param->mode =
			 ROT2_MODE_MODE_270 | ROT2_MODE_BOUNDARY_2_12;
			break;
		}
		rot_param->src_hsize	= siz_dst_width;
		rot_param->src_vsize	= siz_dst_height;
		if (dst_width != dst_size) {
			dst_adryrgb =
			 nts_image->image[IMAGE_DST]->ulPhysAddrYRGB
			 + dst_y * dst_size;
			dst_adruv   =
			 nts_image->image[IMAGE_DST]->ulPhysAddrUV
			 + dst_y * dst_size;
			switch (angle) {
			default:
			case ROT2_MODE_MODE_90:
				rot_param->src_vsize = dst_size;
				break;
			case ROT2_MODE_MODE_180:
				rot_param->src_hsize = dst_size;
				break;
			case ROT2_MODE_MODE_270:
				rot_param->src_vsize = dst_size;
				break;
			}
		}
		rot_param->src_format	= dst_format;
		rot_param->dst_adryrgb	= dst_adryrgb;
		rot_param->dst_adruv	= dst_adruv;
		rot_param->dst_adrv	= dst_adrv;
		rot_param->dst_bytelane	= ROT2_DSTBL_RESET;
		rot_param->input_mode   = RANDOM_MODE;

		emxx_set_rot(rot_info->id, rot_param);
	}

	/* set M2M parameter and start M2M */
	dma_param->src_hsize		= src_width;
	dma_param->src_vsize		= src_height;
	dma_param->src_hskipyrgb	=
	 (src_size * 8 / bpp_y - src_width) * bpp_y  / 8;
	dma_param->src_hskipuv		=
	 (src_size * 8 / bpp_y - src_width) * bpp_uv / 8;
	dma_param->src_hskipv		=
	 (src_size * 8 / bpp_y - src_width) * bpp_v  / 8;
	dma_param->src_adryrgb		= src_adryrgb;
	dma_param->src_adruv		= src_adruv;
	dma_param->src_adrv		= src_adrv;
	dma_param->src_format		= src_format;

#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	if (image_width != siz_dst_width || image_height != siz_dst_height ||
#else
	if (src_width != siz_dst_width || src_height != siz_dst_height ||
#endif
	    field == V4L2_FIELD_INTERLACED ||
	    field == V4L2_FIELD_INTERLACED_TB ||
	    field == V4L2_FIELD_INTERLACED_BT ||
	    src_format != dst_format ||
	    angle == ROT2_MODE_MODE_0 ||
	    dst_width != dst_size) {
		emxx_set_dma_to_siz(siz_info->id, dma_param);
		iRet = emxx_start_dma_to_siz(siz_info->id,
		 nts_image_callback_resize);
		if (iRet) {
			printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			 "start_dma_to_siz(%d)\n", iRet);
		}
	} else {
		emxx_set_dma_to_rot(rot_info->id, dma_param);
		iRet = emxx_start_dma_to_rot(rot_info->id,
		 nts_image_callback_resize);
		if (iRet) {
			printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			 "start_dma_to_rot(%d)\n", iRet);
		}
	}

	printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
	 "=== emxx_set_siz() =====================\n");
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_hsize     :%ld\n",
	 siz_param->src_hsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_vsize     :%ld\n",
	 siz_param->src_vsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_format    :%ld\n",
	 siz_param->src_format);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_hsize     :%ld\n",
	 siz_param->dst_hsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_vsize     :%ld\n",
	 siz_param->dst_vsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_hskip     :%ld\n",
	 siz_param->dst_hskip);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_adryrgb   :0x%08lx\n",
	 siz_param->dst_adryrgb);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_adruv     :0x%08lx\n",
	 siz_param->dst_adruv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_adrv      :0x%08lx\n",
	 siz_param->dst_adrv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_format    :%ld\n",
	 siz_param->dst_format);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_bytelane  :0x%08lx\n",
	 siz_param->dst_bytelane);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "hstep         :0x%08lx\n",
	 siz_param->hstep);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "vstep         :0x%08lx\n",
	 siz_param->vstep);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_hcrop     :%ld\n",
	 siz_param->dst_hcrop);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_vcrop     :%ld\n",
	 siz_param->dst_vcrop);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "rot_dst_format:%ld\n",
	 siz_param->rot_dst_format);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "filter_option :%ld\n",
	 siz_param->filter_option);

	printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
	 "=== emxx_set_rot() =====================\n");
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "mode          :0x%08lx\n",
	 rot_param->mode);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_hsize     :%ld\n",
	 rot_param->src_hsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_vsize     :%ld\n",
	 rot_param->src_vsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_format    :%ld\n",
	 rot_param->src_format);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_adryrgb   :0x%08lx\n",
	 rot_param->dst_adryrgb);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_adruv     :0x%08lx\n",
	 rot_param->dst_adruv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_adrv      :0x%08lx\n",
	 rot_param->dst_adrv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "dst_bytelane  :0x%08lx\n",
	 rot_param->dst_bytelane);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "input_mode    :%ld\n",
	 rot_param->input_mode);

	printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
	 "=== emxx_set_dma_to_xxx() ==============\n");
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_hsize     :%ld\n",
	 dma_param->src_hsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_vsize     :%ld\n",
	 dma_param->src_vsize);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_hskipyrgb :%ld\n",
	 dma_param->src_hskipyrgb);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_hskipuv   :%ld\n",
	 dma_param->src_hskipuv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_hskipv    :%ld\n",
	 dma_param->src_hskipv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_adryrgb   :0x%08lx\n",
	 dma_param->src_adryrgb);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_adruv     :0x%08lx\n",
	 dma_param->src_adruv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_adrv      :0x%08lx\n",
	 dma_param->src_adrv);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "src_format    :%ld\n",
	 dma_param->src_format);

	return iRet;
}


/******************************************************************************
* MODULE   : nts_image_callback_resize
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void nts_image_callback_resize(int flag)
{
	unsigned long flags;
	spin_lock_irqsave(&nts_image->ntsc->lock_callback, flags);
	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	if (flag != M2M_DMA_CALLBACK_SUCCESS)
		printk_err("fail to resize and rotate\n");

	nts_image_change_frame();

	spin_unlock_irqrestore(&nts_image->ntsc->lock_callback, flags);
}


/******************************************************************************
* MODULE   : nts_image_change_frame
* FUNCTION :
* RETURN   : none
* NOTE     : none
******************************************************************************/
static void nts_image_change_frame(void)
{
	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	/* frame change */
	if (nts_image->ntsc->iFrameNoNow == NTS_DISP_FRAME_A) {
		nts_image->ntsc->iFrameNoNext = NTS_DISP_FRAME_B;
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "now(A) -> next(B)\n");
	} else {
		nts_image->ntsc->iFrameNoNext = NTS_DISP_FRAME_A;
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "now(B) -> next(A)\n");
	}
	ntshw_set_framesel(nts_image->ntsc->iFrameNoNext);
	nts_image->ntsc->ntsout_flg = NTS_NTSOUT_ENABLE;
	if (nts_image->ntsc->blank_state.nts_backlight == 1) {
		ntshw_set_ntsout(NTS_NTSOUT_ENABLE);
	} else if (nts_image->ntsc->blank_state.nts_output == 1) {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x40),
		 "set NTSOUT black: current blank mode = %d\n",
		 nts_image->ntsc->blank_state.current_mode);
		ntshw_set_ntsout(NTS_NTSOUT_ENABLE_BLACK);
	} else if (nts_image->ntsc->blank_state.nts_clock == 1) {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x40),
		 "set NTSOUT disable: current blank mode = %d\n",
		 nts_image->ntsc->blank_state.current_mode);
		ntshw_set_ntsout(NTS_NTSOUT_DISABLE);
	} else {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x40),
		 "not set NTSOUT: current blank mode = %d\n",
		 nts_image->ntsc->blank_state.current_mode);
	}

	/* enable NTSC VSYNC interrupt */
	ntshw_set_interruput(NTSHW_INTENSET);

	/* ready callback (informed v4l2 of compose complete time) */
	nts_return_callback_ready();

	/* refresh callback (informed v4l2 of budder free) */
	nts_return_callback_refresh();

	if (nts_image->ntsc->blank_state.nts_output == 0) {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x40),
		 "call nts_irq_handler_sub(): current blank mode = %d\n",
		 nts_image->ntsc->blank_state.current_mode);
		nts_irq_handler_sub();
	}
}


/*****************************************************************************
* MODULE   : nts_image_chk_fillrect
* FUNCTION :
* RETURN   : -
* NOTE     :
******************************************************************************/
static int nts_image_chk_fillrect(void)
{
	struct image_data *old = NULL;
	int index = 0, iRet = 0;

	printk_dbg((_DEBUG_NTS_IMAGE & 0x01), "\n");

	if (nts_image->ntsc->iFrameNoNow == NTS_DISP_FRAME_A) {
		old = nts_image->old[OLD_FRAME_B];
		if (old->uiWidth == 0)
			goto failed;

	} else { /* NTS_DISP_FRAME_B */
		old = nts_image->old[OLD_FRAME_A];
		if (old->uiWidth == 0)
			goto failed;

	}

	if (nts_image->fillrect[index]->uiX == old->uiX
	 && nts_image->fillrect[index]->uiWidth == old->uiWidth
	 && nts_image->fillrect[index]->uiY == old->uiY
	 && nts_image->fillrect[index]->uiHeight == old->uiHeight) {
		goto failed;
	}

	if (nts_image->image[IMAGE_DST]->uiY > old->uiY) {
		/* need fillrect */
		iRet = 1;
	}
	if (nts_image->image[IMAGE_DST]->uiX > old->uiX) {
		/* need fillrect */
		iRet = 1;
	}
	if (nts_image->image[IMAGE_DST]->uiX +
	    nts_image->image[IMAGE_DST]->uiWidth
	    < old->uiX + old->uiWidth) {
		/* need fillrect */
		iRet = 1;
	}
	if (nts_image->image[IMAGE_DST]->uiY +
	    nts_image->image[IMAGE_DST]->uiHeight
	    < old->uiY + old->uiHeight) {
		/* need fillrect */
		iRet = 1;
	}

	if (iRet) {
		printk_dbg((_DEBUG_NTS_IMAGE & 0x02), "all fill\n");
		memcpy(nts_image->fillrect[index], nts_image->image[IMAGE_DST],
			sizeof(struct image_data));

		nts_image->fillrect[index]->uiX      = old->uiX;
		nts_image->fillrect[index]->uiY      = old->uiY;
		nts_image->fillrect[index]->uiWidth  = old->uiWidth;
		nts_image->fillrect[index]->uiHeight = old->uiHeight;

		/* request fillrect function */
		iRet = nts_image_fillrect_request(index);
		if (iRet) {
			printk_dbg((_DEBUG_NTS_IMAGE & 0x02),
			 "fillrect_request(%d)\n", iRet);
		}
	}

failed:
	memcpy(old, nts_image->image[IMAGE_DST], sizeof(struct image_data));

	return iRet;
}


/*****************************************************************************
* MODULE   : nts_image_fillrect_request
* FUNCTION :
* RETURN   : -
* NOTE     :
******************************************************************************/
static int nts_image_fillrect_request(int index)
{
	int iRet;
	unsigned int  uiScreenPixcelWidthY;
	unsigned int  uiScreenPixcelWidthUV;
	unsigned int  uiScreenPixcelWidthV;
	int bpp_y, bpp_uv, bpp_v; /* bit par pixel  */
	int ppl_y, ppl_uv, ppl_v; /* pixel par line */
	unsigned char *p_dst;
	int i;

	bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
	ppl_y = 1; ppl_uv = 1; ppl_v = 0;
	uiScreenPixcelWidthY  =
		nts_image->fillrect[index]->uiScreenWidth / bpp_y  * 8;
	uiScreenPixcelWidthUV =
		nts_image->fillrect[index]->uiScreenWidth / bpp_uv * 8;
	uiScreenPixcelWidthV  = 0;
	nts_image->fillrect[index]->ulPhysAddrYRGB =
		nts_image->fillrect[index]->ulPhysAddrYRGB +
		nts_image->fillrect[index]->uiY * uiScreenPixcelWidthY /
		ppl_y  + nts_image->fillrect[index]->uiX * bpp_y  / 8;
	nts_image->fillrect[index]->ulPhysAddrUV   =
		nts_image->fillrect[index]->ulPhysAddrUV +
		nts_image->fillrect[index]->uiY * uiScreenPixcelWidthUV /
		ppl_uv + nts_image->fillrect[index]->uiX * bpp_uv / 8;
	nts_image->fillrect[index]->ulPhysAddrYRGB &= ~0x03;
	nts_image->fillrect[index]->ulPhysAddrUV   &= ~0x03;
	nts_image->fillrect[index]->ulPhysAddrV    = 0;

	/* fill black */
	p_dst = nts_image->ntsc->SmemV
		+ nts_image->fillrect[index]->ulPhysAddrYRGB
		- nts_image->ntsc->Smem;
	for (i = 0; i < nts_image->fillrect[index]->uiHeight; i++) {
		memset(p_dst, 0x00, nts_image->fillrect[index]->uiWidth);
		p_dst += nts_image->fillrect[index]->uiScreenWidth;
	}
	p_dst = nts_image->ntsc->SmemV
		+ nts_image->fillrect[index]->ulPhysAddrUV
		- nts_image->ntsc->Smem;
	for (i = 0; i < nts_image->fillrect[index]->uiHeight; i++) {
		memset(p_dst, 0x80, nts_image->fillrect[index]->uiWidth);
		p_dst += nts_image->fillrect[index]->uiScreenWidth;
	}
	iRet = 0;

	memset(nts_image->fillrect[index],        0, sizeof(struct image_data));
	return iRet;
}


