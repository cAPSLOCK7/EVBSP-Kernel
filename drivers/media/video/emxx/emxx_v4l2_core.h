/*
 * File Name       : drivers/media/video/emxx/emxx_v4l2_core.h
 * Function        : V4L2 driver for EM/EV
 * Release Version : Ver 1.14
 * Release Date    : 2010.06.23
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

#ifndef EMXX_V4L2_CORE__H
#define EMXX_V4L2_CORE__H


#include <media/videobuf-dma-sg.h>
#include <linux/scatterlist.h>
#include <mach/siz.h>
#include <mach/rot2.h>

#include "emxx_v4l2_time.h"


#define NUM_BUF_V4L2	3	/* number of ROT buffers */
#define NUM_V4L2_MMAP	32


struct emxx_v4l2_fh;
struct emxx_v4l2_device;


/*===============================================================*/
/* next_step_do() call function                                  */
/*===============================================================*/
enum call_func_num {
	VBQ_QUEUE,
	RESCHEDULE,
	ROT_REQUEST,
	ROT_CALLBACK,
	TMR_REQUEST,
	TMR_CALLBACK,
	LCD_REQUEST,
	LCD_CALLBACK,
	VBQ_COMPLETE,
	VBQ_DQUEUE,
};


/*===============================================================*/
/* main thread definition structure                              */
/*===============================================================*/
enum thread_control {
	TH_IDLE,
	TH_RUNNING,
};

/* ROT temporary buffer */
enum buffer_select {
	BUF_A,
	BUF_B,
#if (NUM_BUF_V4L2 == 3)
	BUF_C,
#endif
};

enum buffer_state {
	TMPBUF_IDLE,
	TMPBUF_ROT_WRITE,
	TMPBUF_LCD_READ,
	TMPBUF_LCD_WRITE,
};

/* temporary buffer infomation structure ------------------------*/
struct temp_buffer {
	enum buffer_state	state;
	unsigned long		paddr;

	/* temporary image data (from emxx_common.h) */
	unsigned long      pixelformat;
	struct _IMAGE_DATA dst_data;
};

/* temporary buffer control structure ---------------------------*/
struct buf_object {
	/** temporary buffer data **/
	/* temporary buffer infomation structure */
	struct temp_buffer	buf[NUM_BUF_V4L2];
	wait_queue_head_t	buf_busy;

	/** use emxx_v4l2_rot_callback_main() **/
	/* callback function (from ROT driver) */
	dma_callback_func	callback;
	int (*make_request)(struct emxx_v4l2_device *,
	 struct videobuf_buffer *, enum buffer_select);

	struct videobuf_buffer	*vb;
	/* request structure (to ROT driver) */
	struct emxx_siz_param	siz_param;
	struct emxx_rot_param	rot_param;
	struct emxx_dma_param	dma_param;
};


/* main structure -----------------------------------------------*/
struct th_object {
	struct task_struct		*th;
	char				*th_name;	/* thread name */
	int (*call_request)(struct emxx_v4l2_device *,
	 struct videobuf_buffer *, int);	/* main function to thread*/

	wait_queue_head_t		th_idle;
	/* peculiar idle state flag to thread */
	enum videobuf_state_queued	th_idle_fixed;
	enum videobuf_state_queued	th_idle_flag;
	wait_queue_head_t		th_busy;
	/* peculiar busy state flag to thread */
	enum videobuf_state_queued	th_busy_fixed;

	/* temporary buffer control structure */
	struct buf_object		*th_buf;
};


#if _V4L2_RT_THREAD /* RT thread */
#define V4L2_THREAD_PRIORITY	1	/* thread priority */
#else /* Normal thread */
#define V4L2_THREAD_NICE	-20	/* thread priority */
#endif


/*===============================================================*/
/* mixing flag (member of emxx_v4l2_device)                     */
/*===============================================================*/
#define ROT_MIXING	0x000F
#define TMR_MIXING	0x0F00
#define LCD_MIXING	0xF000
#define ROT_MIXING_SFT	0x00
#define TMR_MIXING_SFT	0x08
#define LCD_MIXING_SFT	0x0C

#define ADD_MIXING(BIT, SFT) \
	do { \
		dev->mixing = (dev->mixing & ~BIT) | ((1 << SFT) & BIT); \
	} while (0)

#define DEL_MIXING(BIT, SFT) \
	do { \
		dev->mixing = (dev->mixing & ~BIT) | ((0 << SFT) & BIT); \
	} while (0)


/*===============================================================*/
/* per-device data structure                                     */
/*===============================================================*/
struct v4l2_mmap {
	unsigned long	vm_start;
	unsigned long	vm_end;
	unsigned long	offset;
};


struct emxx_v4l2_device {
	struct device			dev;
	struct video_device		*vfd;

	struct semaphore		sem_lcdout; /* semafore for lcdout */

	/* spinlock for videobuf queues */
	spinlock_t			vbq_lock;
	/* videobuf queue operations    */
	struct videobuf_queue_ops	vbq_ops;
	/* field counter for videobuf_buffer */
	unsigned long			field_count;
	struct videobuf_buffer		*vb_old_refresh;
	struct videobuf_buffer		*vb_old;

	/* v4l2_workqueue defines the v4l2 driver to LCD driver or ROT driver
	 * working task.
	 */
	struct workqueue_struct		*v4l2_workqueue;
	struct work_struct		wk_tmr_callback_bottom;

	/* The img_lock is used to serialize access to the image parameters for
	 * overlay and capture.  Need to use spin_lock_irq when writing to the
	 * reading, streaming, and previewing parameters.  A regular spin_lock
	 * will suffice for all other cases.
	 */

	/* We allow streaming from at most one filehandle at a time.
	 * non-NULL means streaming is in progress.
	*/
	struct emxx_v4l2_fh		*streaming;
	struct emxx_v4l2_fh		*streamoff;
	unsigned long			mixing;

	/* revise time data */
	struct emxx_v4l2_time		timer;

	/* kernel thread data */
	struct th_object		th_rot;
	struct th_object		th_tmr;
	struct th_object		th_lcd;


	/* pix defines the size and pixel format of the image captured by the
	 * sensor.  This also defines the size of the framebuffers.  The
	 * same pool of framebuffers is used for video capture and video
	 * overlay.  These parameters are set/queried by the
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with a CAPTURE buffer type.
	 */
	struct v4l2_pix_format		pix;

	/* saving effet data */
	struct v4l2_effect		efct;

	/* saving waork buffer data */
	struct v4l2_workbuffer		workbuf;

	/* saving output device data (V4L2 status) */
	unsigned int			output;

	/* saving output device data (LCD status) */
	unsigned int			output_lcd;

	char				suspend;
	char				active;

	struct emxx_siz_info		siz_info;
	struct emxx_rot_info		rot_info;

	int				num_v4l2_mmap;
	struct v4l2_mmap		v4l2_mmap[NUM_V4L2_MMAP];
};


/*===============================================================*/
/* per-filehandle data structure                                 */
/*===============================================================*/
struct emxx_v4l2_fh {
	struct emxx_v4l2_device	*dev;
	enum   v4l2_buf_type	type;
	struct videobuf_queue	vbq;
};


/*===============================================================*/
/* V4L2 streaming state flag. (need to LCD request)              */
/*===============================================================*/
#define STREAM_STOP		0
#define STREAM_PLAYBACK		1
#define STREAM_PLAYBACK_ROT	2


/*===============================================================*/
/* use image sizes check.                                        */
/*===============================================================*/
#define FULL_HD_WIDTH		1920
#define FULL_HD_HEIGHT		1088
#define QXGA_WIDTH		2048
#define QXGA_HEIGHT		1536
#define INPUT_WIDTH		FULL_HD_WIDTH
#define INPUT_HEIGHT		FULL_HD_HEIGHT
#define INPUT_MAX (INPUT_WIDTH > INPUT_HEIGHT ? INPUT_WIDTH : INPUT_HEIGHT)
#define LCD_MAX   (FRONT_WIDTH_LCD > FRONT_HEIGHT_LCD ?\
		   FRONT_WIDTH_LCD : FRONT_HEIGHT_LCD)


#define IMAGE_WIDTH_MIN		8
#define IMAGE_WIDTH_MAX		(LCD_MAX > INPUT_MAX ? LCD_MAX : INPUT_MAX)
#define IMAGE_WIDTH_ALIGN_IL	0x1
#define IMAGE_WIDTH_ALIGN_SP	0x3
#define IMAGE_WIDTH_ALIGN_PL	0x7
#define IMAGE_WIDTH_ALIGN_SIZ_OUT	0x1

#define IMAGE_HEIGHT_MIN	8
#define IMAGE_HEIGHT_MAX	(LCD_MAX > INPUT_MAX ? LCD_MAX : INPUT_MAX)
#define IMAGE_HEIGHT_ALIGN_IL	0x1
#define IMAGE_HEIGHT_ALIGN_SP	0x1
#define IMAGE_HEIGHT_ALIGN_PL	0x1

#define IMAGE_AREA_MAX \
	((FRONT_WIDTH_LCD  > INPUT_WIDTH  ? FRONT_WIDTH_LCD  : INPUT_WIDTH) * \
	 (FRONT_HEIGHT_LCD > INPUT_HEIGHT ? FRONT_HEIGHT_LCD : INPUT_HEIGHT))

#define IMAGE_LEFT_MIN		0
#define IMAGE_LEFT_MAX		IMAGE_WIDTH_MAX
#define IMAGE_LEFT_ALIGN_IL	0x1
#define IMAGE_LEFT_ALIGN_SP	0x3
#define IMAGE_LEFT_ALIGN_PL	0x7
#define IMAGE_TOP_MIN		0
#define IMAGE_TOP_MAX		IMAGE_HEIGHT_MAX
#define IMAGE_TOP_ALIGN		0x1

#define IMAGE_PADDR_NULL	0
#define IMAGE_PADDR_ALIGN	0x3

#define DEST_WIDTH_MIN		8
#define DEST_HEIGHT_MIN		8

#define RESIZE_WIDTH_MIN	64	/* *(1/64) */
#define RESIZE_WIDTH_MAX	256	/* *256    */
#define RESIZE_HEIGHT_MIN	64	/* *(1/64) */
#define RESIZE_HEIGHT_MAX	256	/* *256    */


/*===============================================================*/
/* device name definetion                                        */
/*===============================================================*/
#define DEV_NAME "emxx_v4l2"


#endif /* EMXX_V4L2_CORE__H */
