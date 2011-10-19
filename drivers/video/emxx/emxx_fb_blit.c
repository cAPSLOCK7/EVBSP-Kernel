/*
 * File Name       : drivers/video/emxx/emxx_fb_blit.c
 * Function        : FraemBuffer Driver
 * Release Version : Ver 1.14
 * Release Date    : 2010.09.01
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


#ifdef CONFIG_EMXX_ANDROID


/********************************************************
 *  Include Files                                       *
 *******************************************************/
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/file.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#include <linux/major.h>
#include <linux/platform_device.h>

#include <asm/system.h>

#include <mach/emxx_mem.h>
#include <mach/fbcommon.h>
#ifdef CONFIG_EMXX_IMC
#include <mach/imc.h>
#include <mach/emxx_imc.h>
#endif
#ifdef CONFIG_EMXX_SIZROT2
#include <mach/siz.h>
#include <mach/rot2.h>
#endif
#include <mach/dma.h>

#include "emxx_fb.h"
#include "emxx_fb_blit.h"
#include "alphacomposite.h"


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define _DEBUG_FB_BLIT 0x00 /* 00008421(bit) */
			    /* 0x01: debug function in
			     * 0x02: debug function out
			     */

#define DEV_NAME "emxx_fb_blit"

#define EMXX_FB_BLIT_M2M 0
#define EMXX_FB_BLIT_SIZ 1
#define EMXX_FB_BLIT_ROT 2
#define EMXX_FB_BLIT_IMC 4

/* blit_running */
#define BLIT_NOT_START		0
#define BLIT_RUNNING		1
#define BLIT_DONE		0
#define BLIT_ERROR		-1

/* IMC register initialize */
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

#if _DEBUG_FB_BLIT
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

#define HAS_ALPHA(img) ((img == EMXX_FB_ARGB_8888) | \
			(img == EMXX_FB_ABGR_8888) | \
			(img == EMXX_FB_RGBA_8888) | \
			(img == EMXX_FB_BGRA_8888))


/********************************************************
 *  Variables                                           *
 *******************************************************/
static unsigned char    *buffer_vstart;
static int               blit_running;
static wait_queue_head_t wait_que_blit;

static const uint32_t bytes_per_pixel[] = {
	[EMXX_FB_RGB_565] = 2,
	[EMXX_FB_BGR_888] = 3,
	[EMXX_FB_ARGB_8888] = 4,
	[EMXX_FB_ABGR_8888] = 4,
	[EMXX_FB_RGBA_8888] = 4,
	[EMXX_FB_BGRA_8888] = 4,
};

static const unsigned long siz_format[] = {
	[EMXX_FB_RGB_565] = SIZ_FORMAT_RGB565,
	[EMXX_FB_BGR_888] = SIZ_FORMAT_RGB888,
};

static const unsigned long rot_format[] = {
	[EMXX_FB_RGB_565] = ROT2_FORMAT_RGB565,
	[EMXX_FB_BGR_888] = ROT2_FORMAT_RGB888_RASTER,
	[EMXX_FB_ARGB_8888] = ROT2_FORMAT_ARGB8888_RASTER,
	[EMXX_FB_ABGR_8888] = ROT2_FORMAT_ARGB8888_RASTER,
	[EMXX_FB_RGBA_8888] = ROT2_FORMAT_ARGB8888_RASTER,
	[EMXX_FB_BGRA_8888] = ROT2_FORMAT_ARGB8888_RASTER,
};

static const unsigned long imc_format[] = {
	[EMXX_FB_RGB_565] = IMC_L01x_FORMAT_RGB565,
	[EMXX_FB_BGR_888] = IMC_L01x_FORMAT_RGB888,
	[EMXX_FB_ARGB_8888] = IMC_L01x_FORMAT_ARGB8888,
	[EMXX_FB_ABGR_8888] = IMC_L01x_FORMAT_ARGB8888,
	[EMXX_FB_RGBA_8888] = IMC_L01x_FORMAT_ARGB8888,
	[EMXX_FB_BGRA_8888] = IMC_L01x_FORMAT_ARGB8888,
};

static const unsigned long imc_bytelane[] = {
	[EMXX_FB_RGB_565] = IMC_Lx_BYTELANE_ARGB,
	[EMXX_FB_BGR_888] = IMC_Lx_BYTELANE_ABGR,
	[EMXX_FB_ARGB_8888] = IMC_Lx_BYTELANE_ARGB,
	[EMXX_FB_ABGR_8888] = IMC_Lx_BYTELANE_ABGR,
	[EMXX_FB_RGBA_8888] = IMC_Lx_BYTELANE_RGBA,
	[EMXX_FB_BGRA_8888] = IMC_Lx_BYTELANE_BGRA,
};


/********************************************************
 *  Prototype declarations of local function            *
 *******************************************************/
static int get_img(struct emxx_fb_img *img, struct fb_info *info,
 unsigned long *start, unsigned long *vstart, unsigned long *len,
 struct file **filep);
static void put_img(struct file *src_file, struct file *dst_file);
static void flush_imgs(struct emxx_fb_blit_req *req, struct file *src_file,
 struct file *dst_file);
static int analyze_blit(struct emxx_fb_blit_req *req, struct fb_info *info,
 unsigned long src_start);
static int blit_m2m(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_start, unsigned long src_len,
 struct file *dst_file, unsigned long dst_start, unsigned long dst_len);
static int blit_siz_rot(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_start, unsigned long src_len,
 struct file *dst_file, unsigned long dst_start, unsigned long dst_len);
static int blit_imc(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_start, unsigned long src_len,
 struct file *dst_file, unsigned long dst_start, unsigned long dst_len,
 struct fb_info *info);
static int blit_cpu(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_vstart, unsigned long src_len,
 struct file *dst_file, unsigned long dst_vstart, unsigned long dst_len);
static void callback_m2m(void *data, int intsts, int intrawsts);
static void callback_sizrot2(int flag);
static void callback_imc(int status);


/******************************************************************************
* MODULE   : emxx_fb_init_blit
* FUNCTION : copybit initialize
* RETURN   :
* NOTE     : none
******************************************************************************/
void emxx_fb_init_blit(void)
{
	buffer_vstart = ioremap_wc(V4L2_ROT_BUFFER_ADDR,
					V4L2_ROT_BUFFER_SIZE);
	init_waitqueue_head(&wait_que_blit);
	blit_running = BLIT_NOT_START;

}


/******************************************************************************
* MODULE   : emxx_fb_blit
* FUNCTION : copybit
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_fb_blit(struct fb_info *fb, struct emxx_fb_blit_req *req)
{
	int ret;
	int blit = 0;
	unsigned long src_start = 0, src_vstart = 0, src_len = 0;
	unsigned long dst_start = 0, dst_vstart = 0, dst_len = 0;
	struct file *src_file = 0, *dst_file = 0;
	int buf_size = 0;

	printk_dbg((_DEBUG_FB_BLIT & 0x01), "<start>\n");

	if (unlikely(req->src_rect.h == 0 ||
		     req->src_rect.w == 0)) {
		printk_err("src img of zero size!\n");
		return -EINVAL;
	}
	if (unlikely(req->dst_rect.h == 0 ||
		     req->dst_rect.w == 0)) {
		printk_err("dst img of zero size!\n");
		return -EINVAL;
	}

	if (unlikely(req->src.format >= EMXX_FB_IMGTYPE_LIMIT ||
		     req->dst.format >= EMXX_FB_IMGTYPE_LIMIT)) {
		printk_err("img is of wrong format\n");
		return -EINVAL;
	}

	if (unlikely(req->src_rect.x > req->src.width ||
		     req->src_rect.y > req->src.height ||
		     req->dst_rect.x > req->dst.width ||
		     req->dst_rect.y > req->dst.height)) {
		printk("img rect is outside of img!\n");
		return -EINVAL;
	}

	if (unlikely(get_img(&req->src, fb, &src_start, &src_vstart, &src_len,
	    &src_file))) {
		printk_err("could not retrieve src image from memory\n");
		return -EINVAL;
	}

	if (unlikely(get_img(&req->dst, fb, &dst_start, &dst_vstart, &dst_len,
	    &dst_file))) {
		printk_err("could not retrieve dst image from memory\n");
#ifdef CONFIG_ANDROID_PMEM
		put_pmem_file(src_file);
#endif
		return -EINVAL;
	}

	printk_dbg((_DEBUG_FB_BLIT & 0x01),
	 "dst_start = 0x%08lx dst_vstart = 0x%08lx dst_len = 0x%08lx\n",
	 dst_start, dst_vstart, dst_len);
	printk_dbg((_DEBUG_FB_BLIT & 0x01),
	 "src_start = 0x%08lx src_vstart = 0x%08lx src_len = 0x%08lx\n",
	 src_start, src_vstart, src_len);

	printk_dbg((_DEBUG_FB_BLIT & 0x04),
	 "src format = %d\n", req->src.format);
	printk_dbg((_DEBUG_FB_BLIT & 0x04),
	 "dst format = %d\n", req->dst.format);

	/* analyze blit */
	blit = analyze_blit(req, fb, src_start);
	printk_dbg((_DEBUG_FB_BLIT & 0x05), "blit = %d\n", blit);

	if (blit == EMXX_FB_BLIT_M2M) {
		ret = blit_m2m(blit, req, src_file, src_start, src_len,
				dst_file, dst_start, dst_len);
		goto done;
	}

	if (blit & EMXX_FB_BLIT_SIZ || blit & EMXX_FB_BLIT_ROT) {
		if (blit & EMXX_FB_BLIT_IMC) {
			if (blit & EMXX_FB_BLIT_SIZ)
				buf_size = req->dst_rect.w * req->dst_rect.h *
					   bytes_per_pixel[req->dst.format];
			else
				buf_size = req->src_rect.w * req->src_rect.h *
					   bytes_per_pixel[req->src.format];
			if (buf_size > V4L2_ROT_BUFFER_SIZE) {
				printk_err("buffer size is insufficient!\n");
				ret = -EINVAL;
				goto err_ret;
			}
		}

		ret = blit_siz_rot(blit, req,
				src_file, src_start, src_len,
				dst_file, dst_start, dst_len);
		if (ret)
			goto err_ret;
	}

	if (blit & EMXX_FB_BLIT_IMC) {
		switch (req->src.format) {
		default:
		case EMXX_FB_RGB_565:
		case EMXX_FB_BGR_888:
#ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
				if (req->dst.format == EMXX_FB_RGB_565 ||
				    req->dst.format == EMXX_FB_BGR_888) {
					ret = blit_imc(blit, req,
					       src_file, src_start, src_len,
					       dst_file, dst_start, dst_len,
					       fb);
				} else {
					printk_err("cannot support format!"
					 " src:%d dst:%d\n", req->src.format,
					 req->dst.format);
					ret = -EINVAL;
					goto err_ret;
				}
			} else {
#endif
				ret = blit_imc(blit, req,
				       src_file, src_start, src_len,
				       dst_file, dst_start, dst_len, fb);
#ifdef CONFIG_MACH_EMEV
			}
#endif
			break;
		case EMXX_FB_ARGB_8888:
#ifndef CONFIG_FB_EMXX_COPYBIT_PREMULTI
			if (req->dst.format == EMXX_FB_RGB_565 ||
			    req->dst.format == EMXX_FB_BGR_888) {
				ret = blit_imc(blit, req,
				       src_file, src_start, src_len,
				       dst_file, dst_start, dst_len,
				       fb);
			} else
#else
#ifdef CONFIG_MACH_EMEV
			if ((req->dst.format == EMXX_FB_RGB_565 ||
			     req->dst.format == EMXX_FB_BGR_888)
			    && ((system_rev & EMXX_REV_MASK)
				>= EMXX_REV_ES3)) {
				ret = blit_imc(blit, req,
				       src_file, src_start, src_len,
				       dst_file, dst_start, dst_len,
				       fb);
			} else
#endif
#endif /* CONFIG_FB_EMXX_COPYBIT_PREMULTI */
			if (req->dst.format == EMXX_FB_ARGB_8888) {
				if (src_start != fb->fix.smem_start) {
#ifdef CONFIG_MACH_EMEV
					if ((system_rev & EMXX_REV_MASK)
					    >= EMXX_REV_ES3) {
						ret = blit_imc(blit, req,
							       src_file,
							       src_start,
							       src_len,
							       dst_file,
							       dst_start,
							       dst_len,
							       fb);
					} else
#endif
					  ret = blit_cpu(blit, req,
					       src_file, src_vstart, src_len,
					       dst_file, dst_vstart, dst_len);
				} else
					ret = blit_m2m(blit, req,
					       src_file, src_start, src_len,
					       dst_file, dst_start, dst_len);
			} else {
				printk_err("cannot support format!"
				 " src:%d dst:%d\n", req->src.format,
				 req->dst.format);
				ret = -EINVAL;
				goto err_ret;
			}
			break;
		case EMXX_FB_ABGR_8888:
			if (req->dst.format == EMXX_FB_RGB_565 ||
			    req->dst.format == EMXX_FB_BGR_888) {
#ifdef CONFIG_MACH_EMEV
				if ((system_rev & EMXX_REV_MASK) ==
				    EMXX_REV_ES1) {
 #ifdef CONFIG_FB_EMXX_COPYBIT_PREMULTI
					if (req->dst.format == EMXX_FB_RGB_565)
						ret = blit_cpu(blit, req,
							src_file, src_vstart,
							src_len,
							dst_file, dst_vstart,
							dst_len);
					else {
 #endif
						printk_err("cannot support "
						 "format! src:%d dst:%d\n",
							req->src.format,
							req->dst.format);
						ret = -EINVAL;
						goto err_ret;
 #ifdef CONFIG_FB_EMXX_COPYBIT_PREMULTI
					}
 #endif
				} else {
#endif
 #ifdef CONFIG_FB_EMXX_COPYBIT_PREMULTI
#ifdef CONFIG_MACH_EMEV
					if ((system_rev & EMXX_REV_MASK)
					    >= EMXX_REV_ES3) {
						ret = blit_imc(blit, req,
							       src_file,
							       src_start,
							       src_len,
							       dst_file,
							       dst_start,
							       dst_len,
							       fb);
					} else
#endif
					ret = blit_cpu(blit, req,
					       src_file, src_vstart, src_len,
					       dst_file, dst_vstart, dst_len);
 #else
					ret = blit_imc(blit, req,
					       src_file, src_start, src_len,
					       dst_file, dst_start, dst_len,
					       fb);
 #endif
#ifdef CONFIG_MACH_EMEV
				}
#endif
			} else if (req->dst.format ==
				   EMXX_FB_ABGR_8888) {
				if (src_start != fb->fix.smem_start) {
#ifdef CONFIG_MACH_EMEV
					if ((system_rev & EMXX_REV_MASK)
					    >= EMXX_REV_ES3) {
						ret = blit_imc(blit, req,
							       src_file,
							       src_start,
							       src_len,
							       dst_file,
							       dst_start,
							       dst_len,
							       fb);
					} else
#endif
					ret = blit_cpu(blit, req,
					       src_file, src_vstart, src_len,
					       dst_file, dst_vstart, dst_len);
				} else
					ret = blit_m2m(blit, req,
					       src_file, src_start, src_len,
					       dst_file, dst_start, dst_len);
			} else {
				printk_err("cannot support format!"
				 " src:%d dst:%d\n", req->src.format,
				 req->dst.format);
				ret = -EINVAL;
				goto err_ret;
			}
			break;
		case EMXX_FB_RGBA_8888:
		case EMXX_FB_BGRA_8888:
#ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
				printk_err("cannot support format!"
				 " src:%d dst:%d\n",
				 req->src.format, req->dst.format);
				ret = -EINVAL;
				goto err_ret;
			} else {
#endif
				if (req->dst.format == EMXX_FB_RGB_565 ||
				    req->dst.format == EMXX_FB_BGR_888) {
					ret = blit_imc(blit, req,
					      src_file, src_start, src_len,
					      dst_file, dst_start, dst_len, fb);
				} else {
					printk_err("cannot support format!"
					 " src:%d dst:%d\n",
					 req->src.format, req->dst.format);
					ret = -EINVAL;
					goto err_ret;
				}
#ifdef CONFIG_MACH_EMEV
			}
#endif
			break;
		}
	}

err_ret:
done:
	put_img(src_file, dst_file);

	printk_dbg((_DEBUG_FB_BLIT & 0x02), "< end >\n");

	return ret;
}


static int get_img(struct emxx_fb_img *img, struct fb_info *info,
 unsigned long *start, unsigned long *vstart, unsigned long *len,
 struct file **filep)
{
	int put_needed, ret = 0;
	struct file *file;

	printk_dbg((_DEBUG_FB_BLIT & 0x01), "<start>\n");

#ifdef CONFIG_ANDROID_PMEM
	if (!get_pmem_file(img->memory_id, start, vstart, len, filep))
		return 0;
#endif

	file = fget_light(img->memory_id, &put_needed);
	if (file == NULL)
		return -1;

	if (MAJOR(file->f_dentry->d_inode->i_rdev) == FB_MAJOR) {
		*start = info->fix.smem_start;
		*vstart = (unsigned long)SmemV;
		*len = info->fix.smem_len;
	} else
		ret = -1;
	fput_light(file, put_needed);

	return ret;
}


static void put_img(struct file *src_file, struct file *dst_file)
{
	printk_dbg((_DEBUG_FB_BLIT & 0x01), "<start>\n");

#ifdef CONFIG_ANDROID_PMEM
	if (src_file)
		put_pmem_file(src_file);
	if (dst_file)
		put_pmem_file(dst_file);
#endif
}


static void flush_imgs(struct emxx_fb_blit_req *req, struct file *src_file,
 struct file *dst_file)
{
#ifdef CONFIG_ANDROID_PMEM
	uint32_t src_len, dst_len;

	/* flush src images */
	src_len = req->src.width * req->src_rect.h *
		  bytes_per_pixel[req->src.format];
	flush_pmem_file(src_file, req->src.offset, src_len);

	/* flush dst images */
	dst_len = req->dst.width * req->dst_rect.h *
		  bytes_per_pixel[req->dst.format];
	flush_pmem_file(dst_file, req->dst.offset, dst_len);
#endif
}


static int analyze_blit(struct emxx_fb_blit_req *req, struct fb_info *info,
 unsigned long src_start)
{
	int blit = 0;

	printk_dbg((_DEBUG_FB_BLIT & 0x01), "<start>\n");

	if (req->flags & EMXX_FB_ROT_90) {
		blit |= EMXX_FB_BLIT_ROT;
		if (req->src_rect.w != req->dst_rect.h ||
		    req->src_rect.h != req->dst_rect.w)
			blit |= EMXX_FB_BLIT_SIZ;
		if (req->dst_rect.w != req->dst.width)
			blit |= EMXX_FB_BLIT_IMC;
	} else {
		if (req->src_rect.w != req->dst_rect.w ||
		    req->src_rect.h != req->dst_rect.h)
			blit |= EMXX_FB_BLIT_SIZ;
		if ((req->flags & EMXX_FB_ROT_180) == EMXX_FB_ROT_180) {
			blit |= EMXX_FB_BLIT_ROT;
			if (req->dst_rect.w != req->dst.width)
				blit |= EMXX_FB_BLIT_IMC;
		}
	}

	if (req->transp_mask != EMXX_FB_TRANSP_NOP ||
	    req->alpha != EMXX_FB_ALPHA_NOP ||
	    (src_start != info->fix.smem_start && HAS_ALPHA(req->src.format)) ||
	    req->src.format != req->dst.format)
		blit |= EMXX_FB_BLIT_IMC;

	if (((req->flags & EMXX_FB_ROT_180) != EMXX_FB_ROT_180) &&
	    (req->flags & EMXX_FB_FLIP_LR || req->flags & EMXX_FB_FLIP_UD))
		blit |= EMXX_FB_BLIT_IMC;

	printk_dbg((_DEBUG_FB_BLIT & 0x02), "< end > blit = %d\n", blit);
	return blit;
}


#define DMA_NOT_INITIALIZED	NULL
static int blit_m2m(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_start, unsigned long src_len,
 struct file *dst_file, unsigned long dst_start, unsigned long dst_len)
{
	int ret;
	static dma_regs_t *dmaregs = DMA_NOT_INITIALIZED;
	u32 src_add, dst_add, src_off, dst_off, size, leng;

	printk_dbg((_DEBUG_FB_BLIT & 0x05), "<start>\n");

	if (req->transp_mask != EMXX_FB_TRANSP_NOP)
		return -EINVAL;

	if (req->alpha != EMXX_FB_ALPHA_NOP)
		return -EINVAL;

	if (((req->flags & EMXX_FB_ROT_180) != EMXX_FB_ROT_180) &&
	    (req->flags & EMXX_FB_FLIP_LR || req->flags & EMXX_FB_FLIP_UD))
		return -EINVAL;

	if (dmaregs == DMA_NOT_INITIALIZED) {
		ret = emxx_request_dma(EMXX_DMAC_M2M_ACPU_LCH3, DEV_NAME,
			callback_m2m, (void *)NULL, &dmaregs);
		if (ret) {
			printk_err("M2M is busy!\n");
			dmaregs = DMA_NOT_INITIALIZED;
			return ret;
		}
	}

	if (blit & EMXX_FB_BLIT_SIZ || blit & EMXX_FB_BLIT_ROT) {
		src_add = V4L2_ROT_BUFFER_ADDR;
		src_off = 0;
	} else {
		src_add = src_start + req->src.offset +
			  (req->src_rect.x +
			   (req->src_rect.y * req->src.width)) *
			  bytes_per_pixel[req->src.format];
		src_off = (req->src.width - req->src_rect.w) *
			  bytes_per_pixel[req->src.format];
	}
	dst_add = dst_start + req->dst.offset +
		  (req->dst_rect.x + (req->dst_rect.y * req->dst.width)) *
		  bytes_per_pixel[req->dst.format];
	dst_off = (req->dst.width - req->dst_rect.w) *
		  bytes_per_pixel[req->dst.format];
	size    = req->dst_rect.w * bytes_per_pixel[req->dst.format];
	leng    = size * req->dst_rect.h;

	dmaregs->aadd = src_add;
	dmaregs->badd = dst_add;
	dmaregs->aoff = src_off;
	dmaregs->boff = dst_off;
	dmaregs->size = size;
	dmaregs->leng = leng;
	dmaregs->mode = EMXX_DMAC_DEFMODE_32BIT;

	flush_imgs(req, src_file, dst_file);

	blit_running = BLIT_RUNNING;

	emxx_start_m2m_dma(EMXX_M2M_DMA_LCH(3),
			   EMXX_DMAC_INT_LENG_EN);

	wait_event_interruptible(wait_que_blit, (blit_running != BLIT_RUNNING));

	return 0;
}


static int blit_siz_rot(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_start, unsigned long src_len,
 struct file *dst_file, unsigned long dst_start, unsigned long dst_len)
{
	int ret;
	struct emxx_siz_info  siz_info;
	struct emxx_rot_info  rot_info;
	struct emxx_siz_param siz_param;
	struct emxx_rot_param rot_param;
	struct emxx_dma_param dma_param;
	u32 src_add, dst_add, src_off, dst_off;

	printk_dbg((_DEBUG_FB_BLIT & 0x05), "<start>\n");

	memset(&siz_info,  0, sizeof(struct emxx_siz_info));
	memset(&rot_info,  0, sizeof(struct emxx_rot_info));
	memset(&siz_param, 0, sizeof(struct emxx_siz_param));
	memset(&rot_param, 0, sizeof(struct emxx_rot_param));
	memset(&dma_param, 0, sizeof(struct emxx_dma_param));

	if (blit & EMXX_FB_BLIT_SIZ) {
		switch (req->src.format) {
		case EMXX_FB_RGB_565:
		case EMXX_FB_BGR_888:
			break;
		case EMXX_FB_ARGB_8888:
		case EMXX_FB_ABGR_8888:
		case EMXX_FB_RGBA_8888:
		case EMXX_FB_BGRA_8888:
		default:
			printk_err("resizer cannot support src format(%d)!\n",
			 req->src.format);
			ret = -EINVAL;
			goto err_ret;
			break;
		}

		siz_info.device = DEV_FB;
		ret = emxx_request_siz(&siz_info);
		if (ret) {
			printk_err("SIZ is busy!\n");
			goto request_siz_err;
		}
	}

	if (blit & EMXX_FB_BLIT_ROT) {
		rot_info.device = DEV_FB;
		ret = emxx_request_rot(ROT_LCH0, &rot_info);
		if (ret) {
			printk_err("ROT is busy!\n");
			goto request_rot_err;
		}
	}

	src_add = src_start + req->src.offset +
		  (req->src_rect.x + (req->src_rect.y * req->src.width)) *
		  bytes_per_pixel[req->src.format];
	dst_add = dst_start + req->dst.offset +
		  (req->dst_rect.x + (req->dst_rect.y * req->dst.width)) *
		  bytes_per_pixel[req->dst.format];
	src_off = (req->src.width - req->src_rect.w) *
		  bytes_per_pixel[req->src.format];
	dst_off = (req->dst.width - req->dst_rect.w) *
		  bytes_per_pixel[req->dst.format];

	if (blit & EMXX_FB_BLIT_SIZ) {
		/* SIZ parameter */
		siz_param.src_hsize	= req->src_rect.w;
		siz_param.src_vsize	= req->src_rect.h;
		siz_param.src_format	= siz_format[req->src.format];
		if (blit & EMXX_FB_BLIT_ROT) {
			if (req->flags & EMXX_FB_ROT_90) {
				siz_param.dst_hsize	= req->dst_rect.h;
				siz_param.dst_vsize	= req->dst_rect.w;
			} else {
				siz_param.dst_hsize	= req->dst_rect.w;
				siz_param.dst_vsize	= req->dst_rect.h;
			}
			if (req->src.format == EMXX_FB_RGB_565)
				siz_param.dst_hskip	= (1 << 12) -
				 (siz_param.dst_hsize *
				  bytes_per_pixel[req->src.format]);
			else
				siz_param.dst_hskip	= 0;
			siz_param.dst_adryrgb	= rot_info.adryrgb;
		} else if (blit & EMXX_FB_BLIT_IMC) {
			siz_param.dst_hsize	= req->dst_rect.w;
			siz_param.dst_vsize	= req->dst_rect.h;
			siz_param.dst_hskip	= 0;
			siz_param.dst_adryrgb	= V4L2_ROT_BUFFER_ADDR;
		} else {
			siz_param.dst_hsize	= req->dst_rect.w;
			siz_param.dst_vsize	= req->dst_rect.h;
			siz_param.dst_hskip	= dst_off;
			siz_param.dst_adryrgb	= dst_add;
		}
		siz_param.dst_format	= siz_format[req->src.format];
		siz_param.dst_bytelane	= SIZ_DSTBL_RESET;
		siz_param.hstep	= 256*siz_param.src_hsize/siz_param.dst_hsize;
		siz_param.vstep	= 256*siz_param.src_vsize/siz_param.dst_vsize;
		siz_param.dst_hcrop	= 0;
		siz_param.dst_vcrop	= 0;
		siz_param.rot_dst_format	= SIZ_ROTDSTFMT_OFF;
		siz_param.filter_option = SIZ_FILTER_DEFAULT;

		ret = emxx_set_siz(siz_info.id, &siz_param);
		if (ret) {
			printk_err("SIZ parameter error!\n");
			goto param_err;
		}
	}

	if (blit & EMXX_FB_BLIT_ROT) {
		/* ROT parameter */
		if (req->src.format == EMXX_FB_RGB_565)
			rot_param.mode	= ROT2_MODE_BOUNDARY_2_12;
		else
			rot_param.mode	= 0;
		if (req->flags & EMXX_FB_ROT_90)
			rot_param.mode	|= ROT2_MODE_MODE_90;
		if ((req->flags & EMXX_FB_ROT_180) == EMXX_FB_ROT_180)
			rot_param.mode	|= ROT2_MODE_MODE_180;
		if (req->flags & EMXX_FB_ROT_90) {
			rot_param.src_hsize	= req->dst_rect.h;
			rot_param.src_vsize	= req->dst_rect.w;
		} else {
			rot_param.src_hsize	= req->dst_rect.w;
			rot_param.src_vsize	= req->dst_rect.h;
		}
		rot_param.src_format	= rot_format[req->src.format];
		if (blit & EMXX_FB_BLIT_IMC)
			rot_param.dst_adryrgb	= V4L2_ROT_BUFFER_ADDR;
		else
			rot_param.dst_adryrgb	= dst_add;
		rot_param.dst_bytelane	= ROT2_DSTBL_RESET;
		if (req->src.format == EMXX_FB_RGB_565)
			rot_param.input_mode	= RANDOM_MODE;
		else
			rot_param.input_mode	= RASTER_MODE;

		ret = emxx_set_rot(rot_info.id, &rot_param);
		if (ret) {
			printk_err("ROT parameter error!\n");
			goto param_err;
		}
	}

	/* M2M parameter */
	dma_param.src_hsize	= req->src_rect.w;
	dma_param.src_vsize	= req->src_rect.h;
	dma_param.src_hskipyrgb	= src_off;
	dma_param.src_adryrgb	= src_add;
	if (blit & EMXX_FB_BLIT_SIZ) {
		dma_param.src_format	= siz_format[req->src.format];
		ret = emxx_set_dma_to_siz(siz_info.id, &dma_param);
		if (ret) {
			printk_err("M2M parameter error!\n");
			goto param_err;
		}
	} else {
		dma_param.src_format	= rot_format[req->src.format];
		ret = emxx_set_dma_to_rot(rot_info.id, &dma_param);
		if (ret) {
			printk_err("M2M parameter error!\n");
			goto param_err;
		}
	}

	flush_imgs(req, src_file, dst_file);

	blit_running = BLIT_RUNNING;

	if (blit & EMXX_FB_BLIT_SIZ)
		emxx_start_dma_to_siz(siz_info.id, callback_sizrot2);
	else
		emxx_start_dma_to_rot(rot_info.id, callback_sizrot2);

	wait_event_interruptible(wait_que_blit, (blit_running != BLIT_RUNNING));

param_err:
	if (blit & EMXX_FB_BLIT_ROT)
		emxx_free_rot(rot_info.id);
request_rot_err:
	if (blit & EMXX_FB_BLIT_SIZ)
		emxx_free_siz(siz_info.id);
request_siz_err:
err_ret:
	return ret;
}


static int blit_imc(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_start, unsigned long src_len,
 struct file *dst_file, unsigned long dst_start, unsigned long dst_len,
 struct fb_info *info)
{
	int ret;

	struct emxx_imc_info  imc_info;

	struct emxx_imc_preset imc_preset;
	unsigned long imc_control;
	unsigned long imc_datareq;
	struct imc_gamma_param gamma_param;
	struct imc_yuv_param   yuv_param;
	struct imc_burst_param burst_param;
	unsigned long imc_round_en;

	unsigned long keycolor;

	struct emxx_imc_update_vsync imc_update_vsync;
	unsigned long cpubufsel;
	struct imc_wb_param wb_param;
	unsigned long mirror;
	struct imc_alphasel_param alphasel_param;
	unsigned long l0_scanmode;
	unsigned long l1a_scanmode;
	unsigned long l1b_scanmode;
	unsigned long l1c_scanmode;
	unsigned long l2a_scanmode;
	unsigned long l2b_scanmode;
	unsigned long bg_scanmode;

	struct emxx_imc_update_reserve imc_update_reserve;
	struct l01_param l0_param;
	struct l01_param l1a_param;
	struct l01_param l1b_param;
	struct l01_param l1c_param;
	struct l2_param  l2a_param;
	struct l2_param  l2b_param;
	struct bg_param  bg_param;
	struct rr_param  rr_param;

	struct l01_param *l01;

	u32 src_add, dst_add, src_off, dst_off;

	printk_dbg((_DEBUG_FB_BLIT & 0x05), "<start>\n");

	memset(&imc_info, 0, sizeof(struct emxx_imc_info));

	memset(&imc_preset, 0, sizeof(struct emxx_imc_preset));
	memset(&gamma_param, 0, sizeof(struct imc_gamma_param));
	memset(&yuv_param, 0, sizeof(struct imc_yuv_param));
	memset(&burst_param, 0, sizeof(struct imc_burst_param));
	imc_preset.imc_control = &imc_control;
	imc_preset.imc_datareq = &imc_datareq;
	imc_preset.gamma       = &gamma_param;
	imc_preset.yuv         = &yuv_param;
	imc_preset.burst       = &burst_param;
	imc_preset.imc_round_en = &imc_round_en;

	memset(&imc_update_vsync, 0, sizeof(struct emxx_imc_update_vsync));
	memset(&wb_param, 0, sizeof(struct imc_wb_param));
	memset(&alphasel_param, 0, sizeof(struct imc_alphasel_param));
	imc_update_vsync.cpubufsel    = &cpubufsel;
	imc_update_vsync.wb           = &wb_param;
	imc_update_vsync.mirror       = &mirror;
	imc_update_vsync.alphasel     = &alphasel_param;
	imc_update_vsync.l0_scanmode  = &l0_scanmode;
	imc_update_vsync.l1a_scanmode = &l1a_scanmode;
	imc_update_vsync.l1b_scanmode = &l1b_scanmode;
	imc_update_vsync.l1c_scanmode = &l1c_scanmode;
	imc_update_vsync.l2a_scanmode = &l2a_scanmode;
	imc_update_vsync.l2b_scanmode = &l2b_scanmode;
	imc_update_vsync.bg_scanmode  = &bg_scanmode;

	memset(&imc_update_reserve, 0, sizeof(struct emxx_imc_update_reserve));
	memset(&l0_param, 0, sizeof(struct l01_param));
	memset(&l1a_param, 0, sizeof(struct l01_param));
	memset(&l1b_param, 0, sizeof(struct l01_param));
	memset(&l1c_param, 0, sizeof(struct l01_param));
	memset(&l2a_param, 0, sizeof(struct l2_param));
	memset(&l2b_param, 0, sizeof(struct l2_param));
	memset(&bg_param, 0, sizeof(struct bg_param));
	memset(&rr_param, 0, sizeof(struct rr_param));
	imc_update_reserve.l0  = &l0_param;
	imc_update_reserve.l1a = &l1a_param;
	imc_update_reserve.l1b = &l1b_param;
	imc_update_reserve.l1c = &l1c_param;
	imc_update_reserve.l2a = &l2a_param;
	imc_update_reserve.l2b = &l2b_param;
	imc_update_reserve.bg  = &bg_param;
	imc_update_reserve.rr  = &rr_param;

	if (HAS_ALPHA(req->src.format) && req->alpha != EMXX_FB_ALPHA_NOP)
		return -EINVAL;

	src_add = src_start + req->src.offset +
		  (req->src_rect.x + (req->src_rect.y * req->src.width)) *
		  bytes_per_pixel[req->src.format];
	dst_add = dst_start + req->dst.offset +
		  (req->dst_rect.x + (req->dst_rect.y * req->dst.width)) *
		  bytes_per_pixel[req->dst.format];
	src_off = req->src.width * bytes_per_pixel[req->src.format];
	dst_off = req->dst.width * bytes_per_pixel[req->dst.format];

	imc_info.device = DEV_FB;
	ret = emxx_request_imc(IMC_CH1, &imc_info);
	if (ret) {
		printk_err("IMCW is busy!\n");
		goto request_imc_err;
	}

	/* IMC immediately-reflected registers */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
		*imc_preset.imc_control = imc_format[req->dst.format] |
					  IMC_START_MODE_IMMEDIATE;
#ifdef CONFIG_MACH_EMEV
	else
		*imc_preset.imc_control = IMC_CLKCNT_ALL |
					  imc_format[req->dst.format] |
					  IMC_START_MODE_IMMEDIATE;
#endif
	*imc_preset.imc_datareq = IMC_DATAREQ_INIT;
	imc_preset.gamma->en    = IMC_GAMMA_EN_OFF;
	imc_preset.yuv->ygain   = IMC_YGAINOFFSET_INIT;
	imc_preset.yuv->ugain   = IMC_UGAINOFFSET_INIT;
	imc_preset.yuv->vgain   = IMC_VGAINOFFSET_INIT;
	imc_preset.yuv->yuv2rgb = IMC_DITHER_ON | IMC_TRANSMODE_BT601;
	imc_preset.burst->burst_en  = IMC_BURST_EN_INIT;
	imc_preset.burst->threshold = IMC_THRESHOLD_INIT;
	*imc_preset.imc_round_en = IMC_ROUND_EN_OFF;
	ret = emxx_imc_set_preset(imc_info.id, &imc_preset);
	if (ret) {
		printk_err(
		 "Fail to set IMC immediately-reflected registers!\n");
		goto set_param_err;
	}

	/* IMC V-sync registers */
	*imc_update_vsync.cpubufsel    = IMC_CPUBUFSEL_P;
	wb_param.areaadr_p = dst_add;
	wb_param.hoffset   = dst_off;
	wb_param.format    = imc_format[req->dst.format];
	wb_param.size      = (req->dst_rect.h << IMC_WB_VSIZE_SFT) |
					 (req->dst_rect.w << IMC_WB_HSIZE_SFT);
	wb_param.areaadr_q = 0;
	wb_param.bufsel    = IMC_WB_BUFSEL_P;
	wb_param.mposition = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	wb_param.msize     = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;
	wb_param.color     = IMC_BACKCOLOR_INIT;
	wb_param.bytelane  = imc_bytelane[req->dst.format];
	wb_param.scanmode  = IMC_WB_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.mirror       = IMC_MIRROR_NO_FLIP;
	imc_update_vsync.alphasel->alphasel0 = IMC_ALPHASEL0_INIT;
	imc_update_vsync.alphasel->alphasel1 = IMC_ALPHASEL1_INIT;
	*imc_update_vsync.l0_scanmode  = IMC_Lx_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.l1a_scanmode = IMC_Lx_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.l1b_scanmode = IMC_Lx_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.l1c_scanmode = IMC_Lx_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.l2a_scanmode = IMC_Lx_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.l2b_scanmode = IMC_Lx_SCANMODE_PROGRESSIVE;
	*imc_update_vsync.bg_scanmode  = IMC_Lx_SCANMODE_PROGRESSIVE;
	ret = emxx_imc_set_update_vsync(imc_info.id, &imc_update_vsync);
	if (ret) {
		printk_err("Fail to set IMC V-sync registers!\n");
		goto set_param_err;
	}

	/* IMC update target registers */

	/* set src (start) */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
		l0_param.control     = IMC_Lx_CONTROL_DISABLE;
		l01 = &l1a_param;
	} else
#endif
		l01 = &l0_param;

	l01->control    = IMC_Lx_CONTROL_ENABLE;
	l01->format     = imc_format[req->src.format];
	l01->bytelane   = imc_bytelane[req->src.format];
	l01->bufsel     = IMC_Lx_BUFSEL_P;
	if (req->transp_mask != EMXX_FB_TRANSP_NOP) {
		if (req->src.format == EMXX_FB_RGB_565)
			keycolor = (((((req->transp_mask & 0xF800) >> 11) << 3)
				    + ((req->transp_mask & 0x8000) >> 15)
				    + ((req->transp_mask & 0x8000) >> 14)
				    + ((req->transp_mask & 0x8000) >> 13))
					<< IMC_Lx_KEYR_SFT) |
				   (((((req->transp_mask & 0x07E0) >> 5) << 2)
				    + ((req->transp_mask & 0x0400) >> 10)
				    + ((req->transp_mask & 0x0400) >> 9))
					<< IMC_Lx_KEYG_SFT) |
				   (((((req->transp_mask & 0x001F) >> 0) << 3)
				    + ((req->transp_mask & 0x0010) >>  4)
				    + ((req->transp_mask & 0x0010) >>  3)
				    + ((req->transp_mask & 0x0010) >>  2))
					<< IMC_Lx_KEYB_SFT);
		else /* EMXX_FB_BGR_888 or EMXX_FB_ARGB_8888 */
			keycolor = (((req->transp_mask & 0xFF0000) >> 16)
					<< IMC_Lx_KEYR_SFT) |
				   (((req->transp_mask & 0x00FF00) >> 8)
					<< IMC_Lx_KEYG_SFT) |
				   (((req->transp_mask & 0x0000FF) >>  0)
					<< IMC_Lx_KEYB_SFT);
		l01->keyenable  = IMC_Lx_KEYEN_ENABLE;
		l01->keycolor   = keycolor;
	} else {
		l01->keyenable  = IMC_Lx_KEYEN_DISABLE;
		l01->keycolor   = IMC_L1A_KEYCOLOR_INIT;
	}
	if (req->alpha != EMXX_FB_ALPHA_NOP)
		l01->alpha      = req->alpha;
	else
		l01->alpha      = IMC_Lx_ALPHA_OPAQUE;
	if (src_start == info->fix.smem_start)
		l01->alpha      |= IMC_Lx_ALPHASEL_BIT;
	l01->resize     = IMC_Lx_RESIZE_DISABLE;
	l01->mirror     = IMC_Lx_MIRROR_NO_FLIP;
	if (((req->flags & EMXX_FB_ROT_180) != EMXX_FB_ROT_180) &&
	    (req->flags & EMXX_FB_FLIP_LR))
		l01->mirror     |= IMC_Lx_MIRROR_H_FLIP;
	if (((req->flags & EMXX_FB_ROT_180) != EMXX_FB_ROT_180) &&
	    (req->flags & EMXX_FB_FLIP_UD))
		l01->mirror     |= IMC_Lx_MIRROR_V_FLIP;
	if (blit & EMXX_FB_BLIT_SIZ || blit & EMXX_FB_BLIT_ROT) {
		l01->offset     =
		 req->dst_rect.w * bytes_per_pixel[req->src.format];
		l01->frameadr_p = V4L2_ROT_BUFFER_ADDR;
	} else {
		l01->offset     =
		 req->src.width * bytes_per_pixel[req->src.format];
		l01->frameadr_p = src_add;
	}
	l01->frameadr_q = 0;
	l01->position   =
	 (0 << IMC_Lx_POSY_SFT) | (0 << IMC_Lx_POSX_SFT);
	l01->size       =
	 (req->dst_rect.h << IMC_Lx_SIZEY_SFT) |
	 (req->dst_rect.w << IMC_Lx_SIZEX_SFT);
	l01->mposition  =
	 IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
	l01->msize      =
	 IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;
	/* set src (end) */

	l1b_param.control = IMC_Lx_CONTROL_DISABLE;
	l1c_param.control = IMC_Lx_CONTROL_DISABLE;

	/* set dst (start) */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1) {
#endif
		l1a_param.control    = IMC_Lx_CONTROL_ENABLE;
		l1a_param.format     = imc_format[req->dst.format];
		l1a_param.bufsel     = IMC_Lx_BUFSEL_P;
		l1a_param.bytelane   = imc_bytelane[req->dst.format];
		l1a_param.keyenable  = IMC_Lx_KEYEN_DISABLE;
		l1a_param.keycolor   = IMC_L1A_KEYCOLOR_INIT;
		l1a_param.alpha      = IMC_Lx_ALPHA_OPAQUE;
		l1a_param.resize     = IMC_Lx_RESIZE_DISABLE;
		l1a_param.mirror     = IMC_Lx_MIRROR_NO_FLIP;
		l1a_param.offset     = dst_off;
		l1a_param.frameadr_p = dst_add;
		l1a_param.frameadr_q = 0;
		l1a_param.position   =
		 (0 << IMC_Lx_POSY_SFT) | (0 << IMC_Lx_POSX_SFT);
		l1a_param.size       =
		 (req->dst_rect.h << IMC_Lx_SIZEY_SFT) |
		 (req->dst_rect.w << IMC_Lx_SIZEX_SFT);
		l1a_param.mposition  = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
		l1a_param.msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;
#ifdef CONFIG_MACH_EMEV
	} else {
		l2a_param.control = IMC_Lx_CONTROL_DISABLE;
		l2b_param.control = IMC_Lx_CONTROL_DISABLE;

		bg_param.format     = imc_format[req->dst.format];
		bg_param.bufsel     = IMC_Lx_BUFSEL_P;
		bg_param.bytelane   = imc_bytelane[req->dst.format];
		bg_param.resize     = IMC_Lx_RESIZE_DISABLE;
		bg_param.mirror     = IMC_Lx_MIRROR_NO_FLIP;
		bg_param.offset     = dst_off;
		bg_param.frameadr_p = dst_add;
		bg_param.frameadr_q = 0;
		bg_param.mposition  = IMC_Lx_MPOSX_MIN | IMC_Lx_MPOSY_MIN;
		bg_param.msize      = IMC_Lx_MSIZEX_MAX | IMC_Lx_MSIZEY_MAX;
	}
#endif
	/* set dst (end) */

	/* PreMulti */
	if ((req->src.format == EMXX_FB_ABGR_8888)
	    || (req->src.format == EMXX_FB_ARGB_8888))
		rr_param.reserved = IMC_AMODE_FM;
	else
		rr_param.reserved = 0;


	ret = emxx_imc_set_update_reserve(imc_info.id, &imc_update_reserve,
					  NULL);
	if (ret) {
		printk_err("Fail to set IMC update target registers!\n");
		goto set_param_err;
	}

	flush_imgs(req, src_file, dst_file);

	blit_running = BLIT_RUNNING;

	ret = emxx_imc_set_callback(imc_info.id, NULL, callback_imc);
	if (ret) {
		printk_err("Fail to set callback!\n");
		goto set_callback_err;
	}

	ret = emxx_imc_start(imc_info.id);
	if (ret) {
		printk_err("Fail to start IMC!\n");
		goto start_imc_err;
	}

	wait_event_interruptible(wait_que_blit, (blit_running != BLIT_RUNNING));

start_imc_err:
set_callback_err:
set_param_err:
	emxx_free_imc(imc_info.id);
request_imc_err:
	return ret;
}


static int blit_cpu(int blit, struct emxx_fb_blit_req *req,
 struct file *src_file, unsigned long src_vstart, unsigned long src_len,
 struct file *dst_file, unsigned long dst_vstart, unsigned long dst_len)
{
	int ret;
	unsigned long src_vadd = 0, dst_vadd = 0;
	struct AlphaCompositeArgs_t arg;

	printk_dbg((_DEBUG_FB_BLIT & 0x05), "<start>\n");

	if (req->transp_mask != EMXX_FB_TRANSP_NOP)
		return -EINVAL;

	if (req->alpha != EMXX_FB_ALPHA_NOP)
		return -EINVAL;

	if (((req->flags & EMXX_FB_ROT_180) != EMXX_FB_ROT_180) &&
	    (req->flags & EMXX_FB_FLIP_LR || req->flags & EMXX_FB_FLIP_UD))
		return -EINVAL;

	src_vadd = src_vstart + req->src.offset +
		   (req->src_rect.x + (req->src_rect.y * req->src.width)) *
		   bytes_per_pixel[req->src.format];
	dst_vadd = dst_vstart + req->dst.offset +
		   (req->dst_rect.x + (req->dst_rect.y * req->dst.width)) *
		   bytes_per_pixel[req->dst.format];
	memset(&arg, 0, sizeof(struct AlphaCompositeArgs_t));
	arg.type	= ALPHACOMPOSITE_TYPE2;
	arg.srcfmt_f	= ALPHACOMPOSITE_FMT_ABGR8888;
	switch (req->dst.format) {
	case EMXX_FB_RGB_565:
		arg.srcfmt_b	= ALPHACOMPOSITE_FMT_RGB565;
		arg.dstfmt	= ALPHACOMPOSITE_FMT_RGB565;
		break;
	case EMXX_FB_BGR_888:
		arg.srcfmt_b	= ALPHACOMPOSITE_FMT_BGR888;
		arg.dstfmt	= ALPHACOMPOSITE_FMT_BGR888;
		break;
	default:
		arg.srcfmt_b	= ALPHACOMPOSITE_FMT_ABGR8888;
		arg.dstfmt	= ALPHACOMPOSITE_FMT_ABGR8888;
		break;
	}

	if (blit & EMXX_FB_BLIT_ROT) {
		arg.src_ptr	= buffer_vstart;
		arg.src_size	= req->dst_rect.w *
				  bytes_per_pixel[req->src.format];
	} else {
		arg.src_ptr	= (unsigned char *)src_vadd;
		arg.src_size	= req->src.width *
				  bytes_per_pixel[req->src.format];
	}
	arg.src_b_ptr	= (unsigned char *)dst_vadd;
	arg.dst_ptr	= (unsigned char *)dst_vadd;
	arg.src_b_size	= req->dst.width * bytes_per_pixel[req->dst.format];
	arg.dst_size	= req->dst.width * bytes_per_pixel[req->dst.format];
	arg.dst_hsize	= req->dst_rect.w;
	arg.dst_vsize	= req->dst_rect.h;
	ret = image_alpha_composite(&arg);
	if (ret == 1)
		ret = 0; /* Success */

	return ret;
}


static void callback_m2m(void *data, int intsts, int intrawsts)
{
	printk_dbg((_DEBUG_FB_BLIT & 0x01),
	 "<start> intrawsts = %d\n", intrawsts);
	blit_running = BLIT_DONE;
	wake_up_interruptible(&wait_que_blit);
}


static void callback_sizrot2(int flag)
{
	printk_dbg((_DEBUG_FB_BLIT & 0x01), "<start> flag = %d\n", flag);
	if (flag == M2M_DMA_CALLBACK_SUCCESS)
		blit_running = BLIT_DONE;
	else
		blit_running = BLIT_ERROR;

	wake_up_interruptible(&wait_que_blit);
}


static void callback_imc(int status)
{
	printk_dbg((_DEBUG_FB_BLIT & 0x01), "<start> status = %d\n", status);
	blit_running = BLIT_DONE;
	wake_up_interruptible(&wait_que_blit);
}


#endif /* CONFIG_EMXX_ANDROID */


