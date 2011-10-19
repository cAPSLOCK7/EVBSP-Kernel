/*
 * File Name       : arch/arm/mach-emxx/include/mach/fbcommon.h
 * Function        : Common parameters for Frame Buffer Driver
 * Release Version : Ver 1.17
 * Release Date    : 2011.01.18
 *
 * Copyright (C) 2011 Renesas Electronics Corporation
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
 *
 */

#ifndef _FBCOMMON_H_
#define _FBCOMMON_H_


/********************************************************
 * Include Files                                        *
 *******************************************************/
#include <linux/ioctl.h>
#include <linux/types.h>


/********************************************************
 * Structures                                           *
 *******************************************************/
/* Image infomation (use I/F fb <-> 2D API) */
struct scrn_modes {
	int   tc_enable; /* enable(1)/disable(0) transparent color         */
	int   t_color;	 /* transparent color (16bpp RGB565                */
			 /*                 or 24bpp RGB888)               */
	int   alpha;	 /* alpha ratio (0 - 255)                          */
	int   page; /* 2D framebuffer page(0/1)                            */
	int   rot; /* LCD image rotation(0/1=180deg.)                      */
	int   update; /* absolutely update(1) / update only 2D mix mode(0) */
		      /* when EMXX_FB_SET_MODES ,this value is no effect   */
		      /* only use EMXX_FB_UPDATE_SCRN			   */
};


/* frame buffer page infomation (use I/F fb <-> 2D API) */
struct scrn_mode{
	int   timeout; /* waiting time for any frame ready(0) (10mS)          */
	int   page0; /* page 0 is out of display(0)/waiting for on display(1) */
	int   page1; /* page 1 is out of display(0)/waiting for on display(1) */
};


/* frame buffer output infomation (use I/F fb <-> 2D API) */
enum EMXX_FB_OUTPUT_MODE{
	EMXX_FB_OUTPUT_MODE_LCD,	/* LCD(WVGA:800x480)     */
	EMXX_FB_OUTPUT_MODE_HDMI_1080I,	/* HDMI(1080i:1920x1080) */
	EMXX_FB_OUTPUT_MODE_HDMI_720P,	/* HDMI(720p:1280x720)   */
};


struct emxx_fb_rect {
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
};

struct emxx_fb_img {
	unsigned int width;
	unsigned int height;
	unsigned int format;
	unsigned int offset;
	int memory_id;		/* the file descriptor */
};

struct emxx_fb_blit_req {
	struct emxx_fb_img src;
	struct emxx_fb_img dst;
	struct emxx_fb_rect src_rect;
	struct emxx_fb_rect dst_rect;
	unsigned int alpha;
	unsigned int transp_mask;
	unsigned int flags;
};

struct emxx_fb_blit_req_list {
	unsigned int count;
	struct emxx_fb_blit_req req[];
};


/********************************************************
 *  Definitions                                         *
 *******************************************************/
/*
 * scrn modes
 */
/* enable/disable transparent color */
#define TC_COLOR_DISABLE	1
#define TC_COLOR_ENABLE		0

/* frame buffer page No. */
#define DISP_BUFA		0
#define DISP_BUFB		1
#define NOT_DISP_ON_YET		2

/* inverse mode */
#define NO_INVERSE		0
#define UDRL_INVERSE		1

/* absolutely update flag status */
#define UPDATE_FLAG_OFF		0
#define UPDATE_FLAG_ON		1


/*
 * scrn mode
 */
/* frame buffer page status */
#define OUT_OF_DISPLAY		0
#define WAITING_FOR_ON_DISPLAY	1

/* timeout */
#define CHKSCRN_TIMEOUT		-1


/*
 * emxx_fb_blit_req_list
 */
/* emxx_fb_img.format */
enum {
	EMXX_FB_RGB_565,      /* RGB 565  */
	EMXX_FB_BGR_888,      /* BGR 888  */
	EMXX_FB_ARGB_8888,    /* ARGB 888 */
	EMXX_FB_ABGR_8888,    /* ARGB 888 */
	EMXX_FB_RGBA_8888,    /* ARGB 888 */
	EMXX_FB_BGRA_8888,    /* ARGB 888 */
	EMXX_FB_IMGTYPE_LIMIT /* Non valid image type after this enum */
};

enum {
	PMEM_IMG,
	FB_IMG,
};

/* emxx_fb_blit_req.alpha */
#define EMXX_FB_ALPHA_NOP 0xff

/* emxx_fb_blit_req.transp_mask */
#define EMXX_FB_TRANSP_NOP 0xffffffff

/* emxx_fb_blit_req.flags */
#define EMXX_FB_ROT_NOP 0
#define EMXX_FB_FLIP_LR 0x1
#define EMXX_FB_FLIP_UD 0x2
#define EMXX_FB_ROT_90 0x4
#define EMXX_FB_ROT_180 (EMXX_FB_FLIP_UD|EMXX_FB_FLIP_LR)
#define EMXX_FB_ROT_270 (EMXX_FB_ROT_90|EMXX_FB_FLIP_UD|EMXX_FB_FLIP_LR)
#define EMXX_FB_DITHER 0x8


/*
 * IOCTLs to EMXX fb driver. 0x4E is 'N' for NEC.
 */
#define IOC_EMXX_FB_MAGIC	('N')

#define EMXX_FB_CHKSCRN \
	_IOR(IOC_EMXX_FB_MAGIC, 0x00, struct scrn_mode)
#define EMXX_FB_UPDATE_SCRN \
	_IOW(IOC_EMXX_FB_MAGIC, 0x01, struct scrn_modes)
#define EMXX_FB_SET_MODES \
	_IOW(IOC_EMXX_FB_MAGIC, 0x02, struct scrn_modes)
#define EMXX_FB_GET_MODES \
	_IOR(IOC_EMXX_FB_MAGIC, 0x03, struct scrn_modes)
#define EMXX_FB_SET_OUTPUT \
	_IOW(IOC_EMXX_FB_MAGIC, 0x05, enum EMXX_FB_OUTPUT_MODE)
#define EMXX_FB_GET_OUTPUT \
	_IOR(IOC_EMXX_FB_MAGIC, 0x06, enum EMXX_FB_OUTPUT_MODE)
#define EMXX_FB_BLIT \
	_IOW(IOC_EMXX_FB_MAGIC, 0x07, struct emxx_fb_blit_req_list)


/*
 * fb/LCD Memory Map
 */
#define FRONT_WIDTH_LCD		800
#define FRONT_HEIGHT_LCD	480
#define FRONT_WIDTH_720P	1280
#define FRONT_HEIGHT_720P	720
#define FRONT_WIDTH_1080I	1920
#define FRONT_HEIGHT_1080I	1080

#ifdef CONFIG_EMGR_1G_MEM
#define FRONT_WIDTH_MAX		FRONT_WIDTH_LCD
#define FRONT_HEIGHT_MAX	FRONT_HEIGHT_LCD
#else
#define FRONT_WIDTH_MAX		FRONT_WIDTH_1080I
#define FRONT_HEIGHT_MAX	FRONT_HEIGHT_1080I
#endif /* CONFIG_EMGR_1G_MEM */
#define FRONT_WIDTH_V_LCD	FRONT_WIDTH_LCD
#define FRONT_HEIGHT_V_LCD	\
	(FRONT_WIDTH_MAX * FRONT_HEIGHT_MAX / FRONT_WIDTH_V_LCD)
#define FRONT_WIDTH_V_720P	FRONT_WIDTH_720P
#define FRONT_HEIGHT_V_720P	\
	(FRONT_WIDTH_MAX * FRONT_HEIGHT_MAX / FRONT_WIDTH_V_720P)
#define FRONT_WIDTH_V_1080I	FRONT_WIDTH_1080I
#define FRONT_HEIGHT_V_1080I	\
	(FRONT_WIDTH_MAX * FRONT_HEIGHT_MAX / FRONT_WIDTH_V_1080I)
#ifdef CONFIG_FB_EMXX_ARGB8888
#define BYTES_PER_PIXEL		4	/* for DispBuf  (2D)          - 32bpp */
#elif defined(CONFIG_FB_EMXX_BGR888)
#define BYTES_PER_PIXEL		3	/* for DispBuf  (2D)          - 24bpp */
#else
#define BYTES_PER_PIXEL		2	/* for DispBuf  (2D)          - 16bpp */
#endif
#define BITS_PER_PIXEL		24	/* for FrameBuf (LCDC RGB888) - 24bpp */

#define SMEM_START		FB_FRAME_BUFFER_ADDR
#define SMEM_LENGTH		FB_FRAME_BUFFER_SIZE

#define DISPBUF_LENGTH \
	(FRONT_WIDTH_MAX * FRONT_HEIGHT_MAX * BYTES_PER_PIXEL)
#define DISPBUF_A_OFFSET	0x00000000
#define DISPBUF_B_OFFSET	(DISPBUF_A_OFFSET + DISPBUF_LENGTH)

#ifdef CONFIG_EMXX_LCD_FRAMECACHE
#define FRAMEBUF_START		LCD_FRAME_BUFFER_ADDR
#define FRAMEBUF_LENGTH \
	(FRONT_WIDTH_MAX * FRONT_HEIGHT_MAX * BITS_PER_PIXEL / 8)
#endif /* CONFIG_EMXX_LCD_FRAMECACHE */


#ifdef CONFIG_EMXX_NTS
#define NTSC_WIDTH		720
#define NTSC_HEIGHT		486
#define PAL_WIDTH		720
#define PAL_HEIGHT		576
#define NTS_WIDTH		\
	(NTSC_WIDTH  > PAL_WIDTH  ? NTSC_WIDTH  : PAL_WIDTH)
#define NTS_HEIGHT		\
	(NTSC_HEIGHT > PAL_HEIGHT ? NTSC_HEIGHT : PAL_HEIGHT)
#define NTS_WIDTH_V		\
	(NTSC_WIDTH  > PAL_WIDTH  ? NTSC_WIDTH  : PAL_WIDTH)
#define NTS_HEIGHT_V		\
	(NTSC_HEIGHT > PAL_HEIGHT ? NTSC_HEIGHT : PAL_HEIGHT)
#define NTSC_SMEM_START		NTSC_FRAME_BUFFER_ADDR
#define NTSC_SMEM_LENGTH	NTSC_FRAME_BUFFER_SIZE
#define NTSCFRAME_LENGTH	(NTS_WIDTH * NTS_HEIGHT * 2) /* YUV422 */
#define NTSCFRAME_A_OFFSET	0x00000000
#define NTSCFRAME_B_OFFSET	(NTSCFRAME_A_OFFSET + NTSCFRAME_LENGTH)
#endif /* CONFIG_EMXX_NTS */


#endif /* _FBCOMMON_H_ */
