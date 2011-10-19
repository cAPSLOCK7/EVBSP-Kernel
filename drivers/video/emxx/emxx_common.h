/*
 * File Name       : drivers/video/emxx/emxx_common.h
 * Function        : LCD Driver I/F definitions
 * Release Version : Ver 1.12
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

#ifndef _EMXX_COMMON_H_
#define _EMXX_COMMON_H_


/********************************************************
 *  Include Files                                       *
 *******************************************************/
#include <mach/fbcommon.h>
#ifdef CONFIG_EMXX_NTS
#include <mach/siz.h>
#include <mach/rot2.h>
#ifdef CONFIG_VIDEO_EMXX_FILTER
#include <linux/videodev2.h>
#endif
#endif /* CONFIG_EMXX_NTS */


/********************************************************
 *  Definitions                                         *
 *******************************************************/
/* image infomation from V4L2 */
/* YUV format */
#define V4L2_FORMAT_YUV420Pl	0
#define V4L2_FORMAT_YUV420Pl2	1
#define V4L2_FORMAT_YUV422Pl	2
#define V4L2_FORMAT_YUV422Pl2	3
#define V4L2_FORMAT_YUV422Px	4
/* endian */
#define V4L2_BIG_ENDIAN		0
#define V4L2_LITTLE_ENDIAN	1
/* field */
#define V4L2_TOP_BOTTOM		0
#define V4L2_BOTTOM_TOP		1


/* image infomation from fb driver */
#define FB_MASK_COLOR_DISP_OFF	TC_COLOR_DISABLE
#define FB_MASK_COLOR_DISP_ON	TC_COLOR_ENABLE


/* mix_image_mode */
#define ONLY_2D_MODE		0
#define DSP_AND_2D_MODE		1


/* fb data update ON/OFF */
#define FB_UPDATE_OFF		0
#define FB_UPDATE_ON		1
#define FB_ABSOLUTERY_UPDATE	2

#ifdef CONFIG_FB_EMXX_PANDISP_BLOCK
#define FB_PAN_DISP_TIMEOUT	200	/* CHK_TIMEOUT For FB_PAN_DISPLAY */
#endif	/* FB_EMXX_PANDISP_BLOCK */


#ifdef CONFIG_EMXX_NTS
/********************************************************
 *  NTS Definitions                                     *
 *******************************************************/
/*     emxx_nts_reserve(int iActiveDevice, int iSetOutmode); */
/*     ntshw_reserve(int iSetOutmode); */
/* for iActiveDevice */
#define NTS_NONACTIVE		0x00
#define NTS_ACTIVE_FB		0x01
#define NTS_ACTIVE_V4L2		0x02

/* for iSetOutmode */
#define NTS_OUTPUT_DISABLE	0
#define NTS_OUTPUT_NTSC		3
#define NTS_OUTPUT_PAL		4
#endif /* CONFIG_EMXX_NTS */


/********************************************************
 *  Structure                                           *
 *******************************************************/
/* Image data (use I/F V4L2 -> LCD  or  fb -> LCD) */
struct _IMAGE_DATA {
	unsigned int  size;		/* address addition               */
	unsigned long yrgbaddr;		/* Y  plane address               */
	unsigned long uvaddr;		/* UV plane address               */
	unsigned long vaddr;		/* V  plane address               */
	unsigned int  x;		/* x start pixcel                 */
	unsigned int  y;		/* y start pixcel                 */
	unsigned int  hsize;		/* image width                    */
	unsigned int  vsize;		/* image height                   */
};
#define IMAGE_DATA struct _IMAGE_DATA

struct _SCREEN_DATA {
	unsigned int  x;		/* x outputt pixcel               */
	unsigned int  y;		/* y outputt pixcel               */
	unsigned int  hsize;		/* output width                   */
	unsigned int  vsize;		/* output height                  */
};
#define SCREEN_DATA struct _SCREEN_DATA


struct _FRAME_DATA {
	void *dev;			/* emxx_v4l2_device               */
	void *buf;			/* videobuf_buffer                */
};
#define FRAME_DATA struct _FRAME_DATA


/* Image data from V4L2   (use I/F V4L2 -> LCD) */
struct _V4L2_IMAGE_INFO {
	IMAGE_DATA    image_data;	/* input  image data from v4l2     */
	SCREEN_DATA   screen_data;	/* output image data to LCD        */
	FRAME_DATA    frame_data;	/* v4l2 data delivery              */
	unsigned int  yuvfmt;		/* input image data format         */
/*	unsigned long offsetx;	*/	/* offset x                        */
/*	unsigned long offsety;	*/	/* offset y                        */
	unsigned int  endian;		/* endian type                     */
	unsigned int  output_mode;
	unsigned int  field;
#ifdef CONFIG_EMXX_NTS
	struct emxx_siz_info siz_info;
	struct emxx_rot_info rot_info;
 #ifdef CONFIG_VIDEO_EMXX_FILTER
	struct v4l2_filter_option	filter;	/* resize filter           */
 #endif
 #ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	unsigned int  image_width;	/* original image width            */
	unsigned int  image_height;	/* original image height           */
 #endif
	unsigned int  angle;		/* angle                           */
#endif /* CONFIG_EMXX_NTS */
};
#define V4L2_IMAGE_INFO struct _V4L2_IMAGE_INFO


/* Image data from fb driver  (use I/F fb -> LCD) */
struct _FB_IMAGE_INFO {
	IMAGE_DATA    image_data;	/* input image data from fb driver */
	unsigned int  maskcolr;		/* maskcolor                       */
	unsigned int  maskcolrflg;	/* maskcolor flag                  */
	unsigned int  alpha;		/* alpha                           */
	unsigned int  invflg;		/* inverse mode flag               */
	unsigned int  mix_buf_page;	/* mix 2D buffer frame page        */
	unsigned int  update_flag;	/* update flag ON/Off              */
	enum EMXX_FB_OUTPUT_MODE output_mode;
};
#define FB_IMAGE_INFO struct _FB_IMAGE_INFO


#ifdef CONFIG_VIDEO_EMXX
extern void emxx_v4l2_lcd_callback(FRAME_DATA frame_data);
extern void emxx_v4l2_lcd_refresh_callback(FRAME_DATA frame_data);
extern void emxx_v4l2_notify_lcd_output(unsigned int output);
extern void emxx_v4l2_ntsc_ready_callback(FRAME_DATA frame_data);
#endif


#endif /* _EMXX_COMMON_H_ */
