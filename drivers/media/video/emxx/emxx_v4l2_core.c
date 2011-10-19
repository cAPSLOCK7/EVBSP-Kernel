/*
 * File Name       : drivers/media/video/emxx/emxx_v4l2_core.c
 * Function        : V4L2 driver for EM/EV
 * Release Version : Ver 1.24
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <asm/processor.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <media/v4l2-common.h>

#include <mach/hardware.h>
#include <mach/emxx_v4l2.h>
#include <mach/emxx_mem.h>
#include <mach/fbcommon.h>
#include <media/v4l2-ioctl.h>
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <linux/freezer.h>
#include <mach/pm.h>
#endif /* CONFIG_PM || CONFIG_DPM */
#include "../drivers/video/emxx/emxx_common.h"
#ifdef CONFIG_EMXX_NTS
#include "../drivers/nts/emxx_nts.h"
#endif /* CONFIG_EMXX_NTS */
#include "../drivers/video/emxx/emxx_lcd.h"


#define _V4L2_RT_THREAD 1 /* 1:enable  0:disable */
#if _V4L2_RT_THREAD
#include <linux/sched.h>
#endif
#include "emxx_v4l2_core.h"


/*===============================================================*/
/* v4l2 local configuration					 */
/*===============================================================*/
/* input yuv422planer */
#define VF_YUV422PL	1	/* 0: disable  1: enable */
/* input yuv422interleave */
#define VF_YUV422PX	0	/* 0: disable  1: enable */
/* resize scaling check */
#define RESIZE_CHECK	1	/* 0: disable  1: enable */

/*===============================================================*/
/* debug parameters						 */
/*===============================================================*/
#define _DPM_DBG   0 /* debug suspend/resume */   /* 0: disable  1: enable */
#define _LCK_DBG   0 /* debug spinlock */         /* 0: disable  1: enable */
#define _WKQ_DBG   0 /* debug workqueue */        /* 0: disable  1: enable */
#define _VBQ_DBG   0 /* debug videobuf_queue */   /* 0: disable  1: enable */
#define _ROT_DBG   0 /* debug ROT thread */       /* 0: disable  1: print
						     2: callback print */
#define _TMR_DBG   0 /* debug timer thread */     /* 0: disable  1: enable */
#define _LCD_DBG   0 /* debug LCD thread */       /* 0: disable  1: print
						     2: callback print */
#define _MMAP_DBG  0 /* debug mmap */             /* 0: disable  1: enable */
#define _PERF1_DBG 0 /* debug v4l2 performance */ /* 0: disable  1: enable */
#define _PERF2_DBG 0 /* debug v4l2 performance */ /* 0: disable  1: enable */
#define _TRACE_DBG 0 /* debug back trace */       /* 0: disable  1: enable */


#define printk_err(fmt, arg...) \
	do { \
		printk(KERN_ERR DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_wrn(fmt, arg...) \
	do { \
		printk(KERN_WARNING DEV_NAME ": %s: " fmt, __func__, ## arg); \
	} while (0)

#define printk_info(fmt, arg...) \
	do { \
		printk(KERN_INFO DEV_NAME ": " fmt, ## arg);	\
	} while (0)

#define printk_dbg(level, fmt, arg...) \
	do { \
		if (level > 0) \
			printk(KERN_INFO DEV_NAME ": %s: " fmt, \
				__func__, ## arg); \
	} while (0)

#define dbg_lock(lock, fmt, arg...) \
	do { \
		if (_LCK_DBG > 0) \
			if (spin_is_locked(lock)) \
				printk(KERN_DEBUG DEV_NAME ": %s: " fmt, \
				 __func__, ## arg); \
	} while (0)



/*===============================================================*/
/* kernel thread function					 */
/*===============================================================*/
static int   emxx_v4l2_thread_startup(struct emxx_v4l2_device *dev);
static void  emxx_v4l2_thread_cleanup(struct emxx_v4l2_device *dev);
static void  emxx_v4l2_thread_init(struct emxx_v4l2_device *dev);
static void  emxx_v4l2_thread_main(struct emxx_v4l2_device *dev,
				    struct th_object *th_obj);
static inline int   emxx_v4l2_thread_rot(void *dev);
static inline int   emxx_v4l2_thread_tmr(void *dev);
static inline int   emxx_v4l2_thread_lcd(void *dev);

static void *emxx_v4l2_reschedule(struct emxx_v4l2_device *dev,
				  struct th_object *th_obj);
static int   emxx_v4l2_next_proccess(struct emxx_v4l2_device *dev,
				     struct videobuf_buffer *vb,
				     enum buffer_select buf_sel,
				     enum call_func_num call_func);
static int   emxx_v4l2_chk_siz(struct emxx_v4l2_device *dev,
			       struct videobuf_buffer *vb);
#ifdef CONFIG_VIDEO_EMXX_FRAMESKIP
static int   emxx_v4l2_chk_frameskip(struct emxx_v4l2_device *dev,
				     struct videobuf_buffer *vb,
				     struct th_object *th_obj);
#endif /* CONFIG_VIDEO_EMXX_FRAMESKIP */


/*===============================================================*/
/* kernel timer function					 */
/*===============================================================*/
static inline long  emxx_v4l2_tmr_cmpare_time(struct timeval *start,
					      struct timeval *end);
static inline long  emxx_v4l2_tmr_calc_waittime(struct emxx_v4l2_device *dev,
						struct videobuf_buffer *vb);
static inline void  emxx_v4l2_tmr_chk_waittime(struct emxx_v4l2_device *dev,
					       struct videobuf_buffer *vb);
static inline long
	emxx_v4l2_tmr_calc_adjusttime(struct emxx_v4l2_device *dev);
static inline void  emxx_v4l2_tmr_init(struct emxx_v4l2_device *dev);
static	      int   emxx_v4l2_tmr_request(struct emxx_v4l2_device *dev,
					  struct videobuf_buffer *vb,
					  int dummy);
static inline void  emxx_v4l2_tmr_callback_top(unsigned long data);
#if ENABLE_DELAY
static	      void  emxx_v4l2_tmr_callback_bottom_do(struct work_struct *num);
#else
static inline void
	emxx_v4l2_tmr_callback_bottom_do(struct emxx_v4l2_device *dev);
#endif


/*===============================================================*/
/* ROT proccessing function prottyped				 */
/*===============================================================*/
static void  emxx_v4l2_rot_callback_main(struct emxx_v4l2_device *dev,
					 struct th_object *th_obj,
					 enum call_func_num call_func,
					 int flag);
static int   emxx_v4l2_rot_request_main(struct emxx_v4l2_device *dev,
					struct videobuf_buffer *vb,
					struct th_object *th_obj,
					enum call_func_num call_func);
static inline void  emxx_v4l2_rot_callback(int flag);
static inline int   emxx_v4l2_rot_request(struct emxx_v4l2_device *dev,
					  struct videobuf_buffer *vb,
					  int dummy);
static	      int   emxx_v4l2_rot_make_req(struct emxx_v4l2_device *dev,
					   struct videobuf_buffer *vb,
					   enum buffer_select buf_sel);


/*===============================================================*/
/* LCD proccessing function prottyped				 */
/*===============================================================*/
static inline void *emxx_v4l2_lcd_set_outfunc(int output);
static int emxx_v4l2_lcd_request(struct emxx_v4l2_device *dev,
				 struct videobuf_buffer *vb,
				  int call_flg);


/*===============================================================*/
/* videobuf queue operate function prottyped			 */
/*===============================================================*/
static inline void emxx_v4l2_core_vbq_release(struct videobuf_queue *q,
					      struct videobuf_buffer *vb);
static inline void emxx_v4l2_core_vbq_dqueue(struct videobuf_queue *q,
					     struct videobuf_buffer *vb);
static int emxx_v4l2_core_vbq_cancel(struct videobuf_queue *q,
				     struct videobuf_buffer *vb);
static void  emxx_v4l2_core_vbq_complete(void *arg1,
					 void *arg,
					 int err_flg);
static inline void
	emxx_v4l2_core_vbq_complete_skipped(struct list_head *vbq_stream);
static void  emxx_v4l2_core_vbq_queue(struct videobuf_queue *q,
				      struct videobuf_buffer *vb);
static int emxx_v4l2_core_vbq_prepare(struct videobuf_queue *q,
				      struct videobuf_buffer *vb,
				      enum v4l2_field field);
static int emxx_v4l2_core_vbq_sizecheck_pix(struct videobuf_buffer *vb);
static int emxx_v4l2_core_vbq_sizecheck_crop(struct videobuf_buffer *vb);
static int emxx_v4l2_core_vbq_sizecheck(struct videobuf_queue *q,
					struct videobuf_buffer *vb);
#ifdef CONFIG_VIDEO_EMXX_FILTER
static int emxx_v4l2_core_chkfilter(struct videobuf_buffer *vb);
#endif
static int emxx_v4l2_core_vbq_setup(struct videobuf_queue *q,
				    unsigned int *cnt,
				    unsigned int *size);


/*===============================================================*/
/* ROT workbuffer						 */
/*===============================================================*/
static inline void  emxx_v4l2_core_wkb_init(struct emxx_v4l2_device *dev);
static int emxx_v4l2_core_wkb_active(struct emxx_v4l2_device *dev,
				     unsigned int cmd,
				     void *arg);
static int emxx_v4l2_core_wkb_chkbuf(struct v4l2_workbuffer *wbuf);
static int emxx_v4l2_core_wkb_chksize(struct emxx_v4l2_device *dev,
				      struct videobuf_buffer *vb);


/*===============================================================*/
/* systemcall function prottyped				 */
/*===============================================================*/
static inline int emxx_v4l2_ioc_g_output(struct emxx_v4l2_fh *fh,
					 unsigned int *output);
static int emxx_v4l2_ioc_s_output(struct emxx_v4l2_fh *fh,
				  unsigned int *output);
static int emxx_v4l2_ioc_g_fmt(struct emxx_v4l2_device *dev,
			       struct v4l2_format *fmt);
static int emxx_v4l2_ioc_s_fmt(struct emxx_v4l2_device *dev,
			       struct v4l2_format *fmt);
static int emxx_v4l2_ioc_g_crop(struct emxx_v4l2_device *dev,
				struct v4l2_crop *vc);
static int emxx_v4l2_ioc_s_crop(struct emxx_v4l2_device *dev,
				struct v4l2_crop *vc);
static int emxx_v4l2_ioc_g_ctrl(struct emxx_v4l2_device *dev,
				struct v4l2_control *vc);
static int emxx_v4l2_ioc_s_ctrl(struct emxx_v4l2_device *dev,
				struct v4l2_control *vc);
static int emxx_v4l2_ioc_g_effect(struct emxx_v4l2_device *dev,
				  struct v4l2_effect *vef);
static int emxx_v4l2_ioc_s_effect(struct emxx_v4l2_device *dev,
				  struct v4l2_effect *vef);
static inline int emxx_v4l2_ioc_reqbufs(struct emxx_v4l2_device *dev,
					struct videobuf_queue *vbq,
					struct v4l2_requestbuffers *req);
static inline int emxx_v4l2_ioc_qbuf(struct emxx_v4l2_device *dev,
				     struct videobuf_queue *vbq,
				     struct v4l2_buffer *vb);
static inline int emxx_v4l2_ioc_dqbuf(struct emxx_v4l2_device *dev,
				      struct videobuf_queue *vbq,
				      struct v4l2_buffer *vb,
				      int f_flags);
static int emxx_v4l2_ioc_streamon(struct emxx_v4l2_fh *fh,
				  struct emxx_v4l2_device *dev);
static int emxx_v4l2_ioc_streamoff(struct emxx_v4l2_fh *fh,
				   struct emxx_v4l2_device *dev);
static long emxx_v4l2_core_do_ioctl(struct file *file,
				    unsigned int cmd,
				    void *arg);
static inline long emxx_v4l2_core_ioctl(struct file *file,
					unsigned int cmd,
					unsigned long arg);
static inline unsigned int emxx_v4l2_core_poll(struct file *file,
					       struct poll_table_struct *wait);
static int emxx_v4l2_core_release(struct file *file);
static int emxx_v4l2_core_open(struct file *file);
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
static int emxx_v4l2_core_suspend(struct platform_device *pdev,
				  pm_message_t state);
static int emxx_v4l2_core_resume(struct platform_device *pdev);
#endif /* CONFIG_PM || CONFIG_DPM */


/*===============================================================*/
/* mmap								 */
/*===============================================================*/
static unsigned long emxx_v4l2_virt_to_phys(unsigned long addr);


/*===============================================================*/
/* file_operations						 */
/*===============================================================*/
static struct v4l2_file_operations emxx_v4l2_core_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= emxx_v4l2_core_ioctl,
	.poll		= emxx_v4l2_core_poll,
	.open		= emxx_v4l2_core_open,
	.release	= emxx_v4l2_core_release,
};


/*===============================================================*/
/* device_driver						 */
/*===============================================================*/
static struct platform_driver emxx_v4l2_core_driver = {
	.probe		= NULL,
	.remove		= NULL,
	.shutdown	= NULL,

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	.suspend	= emxx_v4l2_core_suspend,
	.resume		= emxx_v4l2_core_resume,
#else
	.suspend	= NULL,
	.resume		= NULL,
#endif /* CONFIG_PM || CONFIG_DPM */

	.driver.name	= DEV_NAME,
	.driver.bus	= &platform_bus_type,
};


/*===============================================================*/
/* platform_device						 */
/*===============================================================*/
static struct platform_device emxx_v4l2_core_device = {
	.name	= DEV_NAME,
	.dev	= {
			.release	= NULL,
		  },
	.id	= 0,
};


/*===============================================================*/
/* module parameters						 */
/*===============================================================*/
struct emxx_v4l2_device *emxx_dev;

#ifdef CONFIG_EMXX_ANDROID
int v4l2_open_flag;
extern wait_queue_head_t v4l2_close_q;
#endif

static int video_nr = -1;	/* video device minor (-1 ==> auto assign) */




#include "emxx_v4l2_perf.h"
#include "emxx_v4l2_debug.h"


/*===============================================================*/
/* kernel thread function					 */
/*===============================================================*/
/*****************************************************************************
* MODULE   : emxx_v4l2_thread_startup
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_thread_startup(struct emxx_v4l2_device *dev)
{
	emxx_v4l2_thread_init(dev);

	init_waitqueue_head(&dev->th_rot.th_idle);
	init_waitqueue_head(&dev->th_rot.th_busy);
	init_waitqueue_head(&dev->th_tmr.th_idle);
	init_waitqueue_head(&dev->th_tmr.th_busy);
	init_waitqueue_head(&dev->th_lcd.th_idle);
	init_waitqueue_head(&dev->th_lcd.th_busy);

	/*----------------------*/
	/* create kernel thread */
	/*----------------------*/
	dev->th_rot.th = kthread_run(emxx_v4l2_thread_rot,
				     dev,
				     dev->th_rot.th_name);
	if (IS_ERR(dev->th_rot.th))
		return -1;

	dev->th_tmr.th = kthread_run(emxx_v4l2_thread_tmr,
				     dev,
				     dev->th_tmr.th_name);
	if (IS_ERR(dev->th_tmr.th))
		return -1;

	dev->th_lcd.th = kthread_run(emxx_v4l2_thread_lcd,
				     dev,
				     dev->th_lcd.th_name);
	if (IS_ERR(dev->th_lcd.th))
		return -1;

	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_thread_cleanup
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static void emxx_v4l2_thread_cleanup(struct emxx_v4l2_device *dev)
{
	emxx_v4l2_thread_init(dev);

	/*-------------------------*/
	/* discreate kernel thread */
	/*-------------------------*/
	wake_up_interruptible(&dev->th_rot.th_idle);
	wake_up_interruptible(&dev->th_rot.th_buf->buf_busy);
	wake_up_interruptible(&dev->th_rot.th_busy);
	wake_up_interruptible(&dev->th_tmr.th_idle);
	wake_up_interruptible(&dev->th_tmr.th_busy);
	wake_up_interruptible(&dev->th_lcd.th_idle);
	wake_up_interruptible(&dev->th_lcd.th_busy);

	/*-------------------------*/
	/* ROT thread uninitialize */
	/*-------------------------*/
	if (dev->th_rot.th)
		kthread_stop(dev->th_rot.th);
	kfree(dev->th_rot.th_buf);
	memset(&dev->th_rot, 0, sizeof(struct th_object));

	/*-------------------------*/
	/* TMR thread uninitialize */
	/*-------------------------*/
	if (dev->th_tmr.th)
		kthread_stop(dev->th_tmr.th);
	memset(&dev->th_tmr, 0, sizeof(struct th_object));

	/*-------------------------*/
	/* LCD thread uninitialize */
	/*-------------------------*/
	if (dev->th_lcd.th)
		kthread_stop(dev->th_lcd.th);
	memset(&dev->th_lcd, 0, sizeof(struct th_object));
}


/*****************************************************************************
* MODULE   : emxx_v4l2_thread_init
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static void emxx_v4l2_thread_init(struct emxx_v4l2_device *dev)
{
	int i;

	/*-----------------------*/
	/* ROT thread initialize */
	/*-----------------------*/
	dev->th_rot.th_name		      = "kv4l2_rot";
	dev->th_rot.th_idle_flag	      = STATE_QUEUED_IDLE;
	dev->th_rot.th_idle_fixed	      = STATE_QUEUED_ROT_PREPARED;
	dev->th_rot.th_busy_fixed	      = STATE_QUEUED_ROT_QUEUED;
	dev->th_rot.call_request	      = (void *)emxx_v4l2_rot_request;
	/* ROT temp buffer init */
	if (dev->th_rot.th_buf == NULL) {
		dev->th_rot.th_buf = kmalloc(sizeof(struct buf_object),
					      GFP_KERNEL);
	}
	memset(dev->th_rot.th_buf, 0, sizeof(struct buf_object));
	init_waitqueue_head(&dev->th_rot.th_buf->buf_busy);

	for (i = 0; i < NUM_BUF_V4L2; i++) {
		dev->th_rot.th_buf->buf[i].state  = TMPBUF_IDLE;
		dev->th_rot.th_buf->buf[i].paddr  = dev->workbuf.rot_addr
		 + (((dev->workbuf.rot_size / NUM_BUF_V4L2) & ~0x3) * i);
	}
	dev->th_rot.th_buf->make_request      = (void *)emxx_v4l2_rot_make_req;
	dev->th_rot.th_buf->callback	      = (void *)emxx_v4l2_rot_callback;

	/*-----------------------*/
	/* TMR thread initialize */
	/*-----------------------*/
	dev->th_tmr.th_name		      = "kv4l2_tmr";
	dev->th_tmr.th_idle_flag	      = STATE_QUEUED_IDLE;
	dev->th_tmr.th_idle_fixed	      = STATE_QUEUED_TMR_PREPARED;
	dev->th_tmr.th_busy_fixed	      = STATE_QUEUED_TMR_QUEUED;
	dev->th_tmr.call_request	      = (void *)emxx_v4l2_tmr_request;
	/* temp buffer not used */
	dev->th_tmr.th_buf		      = NULL;

	/*-----------------------*/
	/* LCD thread initialize */
	/*-----------------------*/
	dev->th_lcd.th_name		      = "kv4l2_lcd";
	dev->th_lcd.th_idle_flag	      = STATE_QUEUED_IDLE;
	dev->th_lcd.th_idle_fixed	      = STATE_QUEUED_LCD_PREPARED;
	dev->th_lcd.th_busy_fixed	      = STATE_QUEUED_LCD_QUEUED;
	dev->th_lcd.call_request	      = (void *)emxx_v4l2_lcd_request;
	/* temp buffer not used */
	dev->th_lcd.th_buf		      = NULL;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_thread_main
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static void emxx_v4l2_thread_main(struct emxx_v4l2_device *dev,
 struct th_object *th_obj)
{
	struct videobuf_buffer *vb;

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
	try_to_freeze();
#endif /* CONFIG_PM || CONFIG_DPM */

	vb = emxx_v4l2_reschedule(dev, th_obj);
	if (vb == NULL) {
		/* Queue is free */
		trace_add_th(0, "thread_main: wait_event(", "th_idle) <--");
		wait_event_interruptible(th_obj->th_idle,
		 (th_obj->th_idle_flag == th_obj->th_idle_fixed));
		trace_add_th(0, "thread_main: wait_event(", "th_idle) -->");
	} else {
#ifdef CONFIG_VIDEO_EMXX_FRAMESKIP
		if (emxx_v4l2_chk_frameskip(dev, vb, th_obj) !=
		    STATE_FRAME_SKIPPED) {
#endif /* CONFIG_VIDEO_EMXX_FRAMESKIP */
			if (th_obj->th_buf) {
				int i;
				for (i = 0; i < NUM_BUF_V4L2; i++) {
					if (th_obj->th_buf->buf[i].state ==
					    TMPBUF_IDLE)
						break;
				}
				if (i >= NUM_BUF_V4L2) {
					trace_add_th(vb, "thread_main: ",
					 "wait_event(buf_busy) <--");
#if (NUM_BUF_V4L2 == 3)
					wait_event_interruptible(
					 th_obj->th_buf->buf_busy,
					 (th_obj->th_buf->buf[BUF_A].state ==
					 TMPBUF_IDLE ||
					 th_obj->th_buf->buf[BUF_B].state ==
					 TMPBUF_IDLE ||
					 th_obj->th_buf->buf[BUF_C].state ==
					 TMPBUF_IDLE));
#else /* (NUM_BUF_V4L2 == 2) */
					wait_event_interruptible(
					 th_obj->th_buf->buf_busy,
					 (th_obj->th_buf->buf[BUF_A].state ==
					 TMPBUF_IDLE ||
					 th_obj->th_buf->buf[BUF_B].state ==
					 TMPBUF_IDLE));
#endif
					trace_add_th(vb, "thread_main: ",
					 "wait_event(buf_busy) -->");
				}
			}

			/* call request function */
			if (!th_obj->call_request(dev, vb, STREAM_PLAYBACK)) {
				/* processing request */
				trace_add_th(vb, "thread_main: wait_event(",
				 "th_busy) <--");
				wait_event_interruptible(th_obj->th_busy,
				 (vb->state_queued != th_obj->th_busy_fixed));
				trace_add_th(vb, "thread_main: wait_event(",
				 "th_busy) -->");
			}
#ifdef CONFIG_VIDEO_EMXX_FRAMESKIP
		} else {
			trace_add(vb, "thread_main: frame skipped!");
		}
#endif /* CONFIG_VIDEO_EMXX_FRAMESKIP */
	}
}

/*****************************************************************************
* MODULE   : emxx_v4l2_thread_rot
* FUNCTION : start to run request in ROT Request's queue.(ROT thread)
* RETURN   : 0
* NOTE	   :
******************************************************************************/
static inline int emxx_v4l2_thread_rot(void *dev)
{
	/* set thread priority */
#if _V4L2_RT_THREAD /* RT thread */
	struct sched_param param = {.sched_priority = V4L2_THREAD_PRIORITY};
	sched_setscheduler(current, SCHED_FIFO, &param);
#else /* Normal thread */
	set_user_nice(current, V4L2_THREAD_NICE);
#endif

	while (!kthread_should_stop()) {
		emxx_v4l2_thread_main(dev,
		 &((struct emxx_v4l2_device *)dev)->th_rot);
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_thread_tmr
* FUNCTION : start to run request in TIMER Request's queue.(TMR thread)
* RETURN   : 0
* NOTE	   :
******************************************************************************/
static inline int emxx_v4l2_thread_tmr(void *dev)
{
	/* set thread priority */
#if _V4L2_RT_THREAD /* RT thread */
	struct sched_param param = {.sched_priority = V4L2_THREAD_PRIORITY};
	sched_setscheduler(current, SCHED_FIFO, &param);
#else /* Normal thread */
	set_user_nice(current, V4L2_THREAD_NICE);
#endif

	while (!kthread_should_stop()) {
		emxx_v4l2_thread_main(dev,
		 &((struct emxx_v4l2_device *)dev)->th_tmr);
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_thread_lcd
* FUNCTION : start to run request in LCD Request's queue.(LCD thread)
* RETURN   : 0
* NOTE	   :
******************************************************************************/
static inline int emxx_v4l2_thread_lcd(void *dev)
{
	/* set thread priority */
#if _V4L2_RT_THREAD /* RT thread */
	struct sched_param param = {.sched_priority = V4L2_THREAD_PRIORITY};
	sched_setscheduler(current, SCHED_FIFO, &param);
#else /* Normal thread */
	set_user_nice(current, V4L2_THREAD_NICE);
#endif

	while (!kthread_should_stop()) {
		emxx_v4l2_thread_main(dev,
		 &((struct emxx_v4l2_device *)dev)->th_lcd);
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_reschedule
* FUNCTION : call next queue.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static void *emxx_v4l2_reschedule(struct emxx_v4l2_device *dev,
	struct th_object *th_obj)
{
	struct videobuf_queue  *q  = NULL;
	struct videobuf_buffer *vb = NULL;
	struct list_head       *list;
	unsigned long flags;

	printk_dbg((_ROT_DBG | _TMR_DBG | _LCD_DBG), "call in.\n");
	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	if (dev->streaming) {
		q = &dev->streaming->vbq;
		list_for_each(list, &q->stream)
		{
			vb = list_entry(list, struct videobuf_buffer, stream);
			switch (vb->state) {
			case VIDEOBUF_PREPARED:
				vb->state = VIDEOBUF_QUEUED;
				emxx_v4l2_next_proccess(dev, vb, 0,
				 RESCHEDULE);
				/* FALL THROUGH */
			case VIDEOBUF_QUEUED:
				printk_dbg(_VBQ_DBG,
				 "(%p) ->prev(%p) ->next(%p) ->state(%d) "
				 "->sequence(%d)\n", &vb->stream,
				 vb->stream.prev, vb->stream.next, vb->state,
				 vb->sequence);
				if (vb->state_queued < th_obj->th_idle_fixed) {
					/* don't process it for status_queued
					  that is smaller than call_function. */
					trace_add_th(vb, "reschedule:" ,
					 "busy...");
					th_obj->th_idle_flag =
						STATE_QUEUED_IDLE;
					spin_unlock_irqrestore(&dev->vbq_lock,
					 flags);
					return NULL;
				} else if (vb->state_queued ==
					th_obj->th_idle_fixed) {
					trace_add_th(vb, "reschedule:" ,
					 "next find.");
					spin_unlock_irqrestore(&dev->vbq_lock,
					 flags);
					return vb;
				} else {
					/* search next queue */
					break;
				}
			case VIDEOBUF_CANCELED:
				if (vb->state_queued != STATE_QUEUED_DONE) {
					if (vb->state_queued ==
						th_obj->th_idle_fixed) {
						trace_add_th(vb, "reschedule:",
						 "CANCELED.");
						spin_unlock_irqrestore(
							&dev->vbq_lock, flags);
						return vb;
					}
				}
				break;
			default:
				/* search next queue */
				break;
			}
		}
	}

	trace_add(0, "reschedule: not find.");
	th_obj->th_idle_flag = STATE_QUEUED_IDLE;
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return NULL;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_next_proccess
* FUNCTION : Allocate & Set next Queue status.
* RETURN   :
* NOTE	   : request() & callback() allways call this function.
******************************************************************************/
static int emxx_v4l2_next_proccess(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb, enum buffer_select buf_sel,
 enum call_func_num call_func)
{
	if (call_func == VBQ_DQUEUE) {
		vb->state_queued    = STATE_QUEUED_IDLE;
		vb->state_proccess |= STATE_PROCCESS_IDLE;
		return -1;
	}

	/* phase1 **************/
	/* check CANCELED task */
	if ((vb->state == VIDEOBUF_CANCELED || vb->state == VIDEOBUF_ERROR) ||
	   (vb->state == VIDEOBUF_DQBUF_PERMIT
	    && dev->vb_old_refresh == NULL)) {
		trace_add_func(vb, "cancel_proccess", call_func);

		/* initialize buffer */
		switch (vb->state_queued) {
		case STATE_QUEUED_LCD_PREPARED:	/* FALL THROUGH */
		case STATE_QUEUED_TMR_QUEUED:	/* FALL THROUGH */
		case STATE_QUEUED_TMR_PREPARED:	/* FALL THROUGH */
		case STATE_QUEUED_ROT_QUEUED:	/* FALL THROUGH */
		case STATE_QUEUED_DONE:
			/* free buffer for ROT */
			if (vb->rot_buf) {
				vb->rot_buf->state = TMPBUF_IDLE;
				vb->rot_buf	   = NULL;
				trace_add(vb, "ROT: buffer -->");
			}
			break;
		default:
			break;
		}

		/* to next proccess */
		switch (call_func) {
		case ROT_REQUEST: /* FALL THROUGH */
		case ROT_CALLBACK:
			DEL_MIXING(ROT_MIXING, ROT_MIXING_SFT);
			if (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE
			 || vb->state_proccess & STATE_PROCCESS_ROT_QUEUED) {
				trace_add(vb,
				 "cancel_proccess: wake_up(ROT: buf_busy)");
				wake_up_interruptible(
					&dev->th_rot.th_buf->buf_busy);
			}
			trace_add(vb, "cancel_proccess: wake_up(ROT: th_busy)");
			wake_up_interruptible(&dev->th_rot.th_busy);
			break;
		case TMR_REQUEST: /* FALL THROUGH */
		case TMR_CALLBACK:
			DEL_MIXING(TMR_MIXING, TMR_MIXING_SFT);
			if (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE
			 || vb->state_proccess & STATE_PROCCESS_ROT_QUEUED) {
				trace_add(vb,
				 "cancel_proccess: wake_up(ROT: buf_busy)");
				wake_up_interruptible(
					&dev->th_rot.th_buf->buf_busy);
			}
			trace_add(vb, "cancel_proccess: wake_up(TMR: th_busy)");
			wake_up_interruptible(&dev->th_tmr.th_busy);
			break;
		case LCD_REQUEST: /* FALL THROUGH */
		case LCD_CALLBACK:
			DEL_MIXING(LCD_MIXING, LCD_MIXING_SFT);
			if (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE
			 || vb->state_proccess & STATE_PROCCESS_ROT_QUEUED) {
				trace_add(vb,
				 "cancel_proccess: wake_up(ROT: buf_busy)");
				wake_up_interruptible(
					&dev->th_rot.th_buf->buf_busy);
			}
			trace_add(vb, "cancel_proccess: wake_up(LCD: th_busy)");
			wake_up_interruptible(&dev->th_lcd.th_busy);
			break;
		default:
			break;
		}

		vb->state_queued    = STATE_QUEUED_DONE;
		vb->state_proccess |= STATE_PROCCESS_LCD_COMPLETE;
		return -1;
	} else if (vb->state != VIDEOBUF_PREPARED
		 && vb->state != VIDEOBUF_QUEUED
		 && vb->state != VIDEOBUF_ACTIVE
		 && vb->state != VIDEOBUF_DONE) {
		switch (call_func) {
		case ROT_REQUEST: /* FALL THROUGH */
		case ROT_CALLBACK:
			DEL_MIXING(ROT_MIXING, ROT_MIXING_SFT);
			break;
		case TMR_REQUEST: /* FALL THROUGH */
		case TMR_CALLBACK:
			DEL_MIXING(TMR_MIXING, TMR_MIXING_SFT);
			break;
		case LCD_REQUEST: /* FALL THROUGH */
		case LCD_CALLBACK:
			DEL_MIXING(LCD_MIXING, LCD_MIXING_SFT);
			break;
		default:
			break;
		}
		return 0;
	}

	trace_add_func(vb, "next_proccess", call_func);

	/* phase2 ********************/
	/* set flag to next proccess */
	switch (call_func) {
	case VBQ_DQUEUE:
		break;

	case LCD_CALLBACK:
		DEL_MIXING(LCD_MIXING, LCD_MIXING_SFT);
		if (dev->vb_old_refresh) {
			if (dev->vb_old_refresh->rot_buf
			 && (dev->vb_old_refresh->state_proccess
			     & STATE_PROCCESS_ROT_COMPLETE)) {
				dev->vb_old_refresh->rot_buf->state =
					TMPBUF_IDLE;
				dev->vb_old_refresh->rot_buf	    = NULL;
				trace_add(dev->vb_old_refresh,
				 "ROT: buffer -->");
			}
		}
		dev->vb_old_refresh = vb;
		if (vb->rot_buf
		 && (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE)) {
			vb->rot_buf->state = TMPBUF_LCD_WRITE;
			trace_add(vb, "ROT: buffer -->");
		}
		vb->state_queued    = STATE_QUEUED_DONE;
		vb->state_proccess |= STATE_PROCCESS_LCD_COMPLETE;
		break;

	case VBQ_COMPLETE:
		break;

	case ROT_REQUEST:
		ADD_MIXING(ROT_MIXING, ROT_MIXING_SFT);
		vb->state_queued    = STATE_QUEUED_ROT_QUEUED;
		vb->state_proccess |= STATE_PROCCESS_ROT_QUEUED;

		switch (buf_sel) {
		case 0:
			trace_add(vb, "ROT: buffer0 <--");
			break;
		case 1:
			trace_add(vb, "ROT: buffer1 <--");
			break;
		case 2:
			trace_add(vb, "ROT: buffer2 <--");
			break;
		}
		vb->rot_buf	    = &dev->th_rot.th_buf->buf[buf_sel];
		vb->rot_buf->state  = TMPBUF_ROT_WRITE;
		break;
	case TMR_REQUEST:
		ADD_MIXING(TMR_MIXING, TMR_MIXING_SFT);
		vb->state_queued    = STATE_QUEUED_TMR_QUEUED;
		vb->state_proccess |= STATE_PROCCESS_TMR_QUEUED;
		break;
	case LCD_REQUEST:
		ADD_MIXING(LCD_MIXING, LCD_MIXING_SFT);
		vb->state_queued    = STATE_QUEUED_LCD_QUEUED;
		vb->state_proccess |= STATE_PROCCESS_LCD_QUEUED;
		break;

	case VBQ_QUEUE:
		/* FALL THROUGH */
	case RESCHEDULE:
		vb->state_proccess = STATE_PROCCESS_IDLE;

		if (
#ifdef CONFIG_EMXX_NTS
		    (vb->output == V4L2_OUTPUT_LCD ||
		     vb->output == V4L2_OUTPUT_HDMI_1080I ||
		     vb->output == V4L2_OUTPUT_HDMI_720P) &&
#endif /* CONFIG_EMXX_NTS */
		    (vb->save_effect.movie_angle ||
		     emxx_v4l2_chk_siz(dev, vb))) {
			/* next process is ROT */
			vb->state_queued	 = STATE_QUEUED_ROT_PREPARED;
			dev->th_rot.th_idle_flag = dev->th_rot.th_idle_fixed;
			break;
		}
		/* FALL THROUGH */
	case ROT_CALLBACK:
		if (call_func == ROT_CALLBACK) {
			DEL_MIXING(ROT_MIXING, ROT_MIXING_SFT);
			vb->state_proccess |= STATE_PROCCESS_ROT_COMPLETE;
		}
		if (vb->state_frame != STATE_FRAME_IMMEDIATE) {
			/* next process is TIMER */
			vb->state_queued	 = STATE_QUEUED_TMR_PREPARED;
			dev->th_tmr.th_idle_flag = dev->th_tmr.th_idle_fixed;
			break;
		}
		/* FALL THROUGH */
	case TMR_CALLBACK:
		if (call_func == TMR_CALLBACK) {
			DEL_MIXING(TMR_MIXING, TMR_MIXING_SFT);
			vb->state_proccess |= STATE_PROCCESS_TMR_COMPLETE;
		}
		/* next process is LCD */
		if (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE)
			vb->rot_buf->state = TMPBUF_LCD_READ;

		vb->state_queued	 = STATE_QUEUED_LCD_PREPARED;
		dev->th_lcd.th_idle_flag = dev->th_lcd.th_idle_fixed;
		break;
	}

	/* phase3 ******************/
	/* wakeup to next proccess */
	switch (call_func) {
	case VBQ_DQUEUE:
		break;

	case LCD_CALLBACK:
#ifdef CONFIG_EMXX_NTS
		if ((vb->output == V4L2_OUTPUT_LCD) ||
		    (vb->output == V4L2_OUTPUT_HDMI_1080I) ||
		    (vb->output == V4L2_OUTPUT_HDMI_720P)) {
#endif /* CONFIG_EMXX_NTS */
			trace_add(vb, "next_proccess: wake_up(LCD: th_busy)");
			wake_up_interruptible(&dev->th_lcd.th_busy);
#ifdef CONFIG_EMXX_NTS
		}
#endif /* CONFIG_EMXX_NTS */
		if (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE) {
			trace_add(vb, "next_proccess: wake_up(ROT: buf_busy)");
			wake_up_interruptible(&dev->th_rot.th_buf->buf_busy);
		}
		break;

	case VBQ_COMPLETE:
#ifdef CONFIG_EMXX_NTS
		if ((vb->output != V4L2_OUTPUT_LCD) &&
		    (vb->output != V4L2_OUTPUT_HDMI_1080I) &&
		    (vb->output != V4L2_OUTPUT_HDMI_720P)) {
			trace_add(vb, "next_proccess: wake_up(LCD: th_busy)");
			wake_up_interruptible(&dev->th_lcd.th_busy);
		}
#endif /* CONFIG_EMXX_NTS */
		break;

	case ROT_REQUEST:
		break;
	case TMR_REQUEST:
		break;
	case LCD_REQUEST:
		break;

	case VBQ_QUEUE:
		/* FALL THROUGH */
	case RESCHEDULE:
		if (
#ifdef CONFIG_EMXX_NTS
		    (vb->output == V4L2_OUTPUT_LCD ||
		     vb->output == V4L2_OUTPUT_HDMI_1080I ||
		     vb->output == V4L2_OUTPUT_HDMI_720P) &&
#endif /* CONFIG_EMXX_NTS */
		    (vb->save_effect.movie_angle ||
		     emxx_v4l2_chk_siz(dev, vb))) {
			/* next process is ROT */
			trace_add(vb, "next_proccess: wake_up(ROT: th_idle)");
			wake_up_interruptible(&dev->th_rot.th_idle);
			break;
		}
		/* FALL THROUGH */
	case ROT_CALLBACK:
		if (call_func == ROT_CALLBACK) {
			trace_add(vb, "next_proccess: wake_up(ROT: th_busy)");
			wake_up_interruptible(&dev->th_rot.th_busy);
		}
		if (vb->state_frame != STATE_FRAME_IMMEDIATE) {
			/* next process is TIMER */
			trace_add(vb, "next_proccess: wake_up(TMR: th_idle)");
			wake_up_interruptible(&dev->th_tmr.th_idle);
			break;
		}
		/* FALL THROUGH */
	case TMR_CALLBACK:
		if (call_func == TMR_CALLBACK) {
			trace_add(vb, "next_proccess: wake_up(TMR: th_busy)");
			wake_up_interruptible(&dev->th_tmr.th_busy);
		}
		/* next process is LCD */
		trace_add(vb, "next_proccess: wake_up(LCD: th_idle)");
		wake_up_interruptible(&dev->th_lcd.th_idle);
		break;
	}

	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_chk_siz
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_chk_siz(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb)
{
	unsigned long src_width, src_height, dst_width, dst_height;
	unsigned int image_align_rot;

#if CONFIG_VIDEO_EMXX_IMAGESIZE
	src_width  = vb->image_width;
	src_height = vb->image_height;
#else
	src_width  = vb->save_effect.source_crop.width;
	src_height = vb->save_effect.source_crop.height;
#endif

	if ((vb->save_effect.screen.width != 0)
	 && (vb->save_effect.screen.height != 0)) {
#ifdef CONFIG_EMXX_NTS
		if ((vb->output == V4L2_OUTPUT_LCD) ||
		    (vb->output == V4L2_OUTPUT_HDMI_1080I) ||
		    (vb->output == V4L2_OUTPUT_HDMI_720P)) {
			/* output LCD */
			trace_add(vb, "chk_siz: output LCD");
#endif
			if (vb->output != V4L2_OUTPUT_HDMI_1080I &&
			    (vb->field == V4L2_FIELD_INTERLACED ||
			     vb->field == V4L2_FIELD_INTERLACED_TB ||
			     vb->field == V4L2_FIELD_INTERLACED_BT)) {
				trace_add(vb, "chk_siz: ->SIZ(filter)");
				return 1;
			}

			if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
			 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
				dst_width  = vb->save_effect.screen.width;
				dst_height = vb->save_effect.screen.height;
			} else {
				dst_width  = vb->save_effect.screen.height;
				dst_height = vb->save_effect.screen.width;
				switch (vb->pixelformat) {
				default:
				case V4L2_PIX_FMT_NV12: /* YUV420 Semi-Planar */
				case V4L2_PIX_FMT_NV422:/* YUV422 Semi-Planar */
					image_align_rot = IMAGE_WIDTH_ALIGN_SP;
					break;
				case V4L2_PIX_FMT_YUV420: /* YUV420 Planar    */
#if VF_YUV422PL
				case V4L2_PIX_FMT_YUV422P:/* YUV422 Planar    */
#endif
					image_align_rot = IMAGE_WIDTH_ALIGN_PL;
					break;
#if VF_YUV422PX
				case V4L2_PIX_FMT_YUYV: /* YUV422 Interleave  */
					image_align_rot = IMAGE_WIDTH_ALIGN_IL;
					break;
#endif
				}
				if ((image_align_rot != IMAGE_WIDTH_ALIGN_IL) &&
				    (dst_height & image_align_rot)) {
					trace_add(vb,
					 "chk_siz: ->SIZ(convert format)");
					return 1;
				}
			}
			if ((dst_width != src_width)
			 || (dst_height != src_height)) {
				trace_add(vb, "chk_siz: ->SIZ(resize)");
				return 1;
			}
#ifdef CONFIG_EMXX_NTS
		} else {
			/* output NTSC */
			trace_add(vb, "chk_siz: output NTSC");
			return 1;
		}
#endif /* CONFIG_EMXX_NTS */
	}

	trace_add(vb, "chk_siz: ->not SIZ");
	return 0;
}


#ifdef CONFIG_VIDEO_EMXX_FRAMESKIP
#define SKIP_COUNT_MAX 1
#define SKIP_COUNT_INIT 0
/*****************************************************************************
* MODULE   : emxx_v4l2_chk_frameskip
* FUNCTION : check condition of frameskip function.
* RETURN   : STATE_FRAME_DONE    : not skipped
*          : STATE_FRAME_SKIPPED : skipped
* NOTE     :
******************************************************************************/
static int emxx_v4l2_chk_frameskip(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb, struct th_object *th_obj)
{
#if (SKIP_COUNT_MAX > 0)
	static int skip_count = SKIP_COUNT_INIT;
#endif
	unsigned long flags;
	long adjust_time;
	int  adjust;
	int  ret = STATE_FRAME_DONE;
	enum call_func_num call_func = VBQ_QUEUE;

	trace_add(vb, "chk_frameskip");

	spin_lock_irqsave(&dev->vbq_lock, flags);

	/* this process */
	if (th_obj->call_request == emxx_v4l2_rot_request) {
		/* rot thread */
		adjust = 0;
		call_func = ROT_REQUEST;
	} else if (th_obj->call_request == emxx_v4l2_tmr_request) {
		/* tmr thread */
		adjust = 1; /* IMC */
		call_func = TMR_REQUEST;
	} else{
		/* lcd thread */
		if (vb->state_proccess & STATE_PROCCESS_TMR_COMPLETE)
			adjust = 0;
		else
			adjust = 1;

		call_func = LCD_REQUEST;
	}

	if (adjust > 0) {
		if (vb->state_frame == STATE_FRAME_IMMEDIATE) {
#if (SKIP_COUNT_MAX > 0)
			skip_count = 0;
#endif
		} else {
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			down(&dev->sem_lcdout);
			trace_add(vb, "chk_frameskip:down sem_lcdout");
			spin_lock_irqsave(&dev->vbq_lock, flags);

#if (SKIP_COUNT_MAX > 0)
			if (skip_count >= SKIP_COUNT_MAX) {
				skip_count = 0;
			} else {
#endif
				adjust_time  =
				 emxx_v4l2_tmr_calc_waittime(dev, vb);
				adjust_time *= adjust;

#if _TRACE_DBG
				memset(cp_name, 0, sizeof(cp_name));
				sprintf(cp_name,
				 "chk_frameskip: waittime(%ld) = "
				 "wait(%ld) - adjust(%ld)",
				 dev->timer.wait_time - adjust_time,
				 dev->timer.wait_time, adjust_time);
				trace_add(vb, cp_name);
#endif /* _TRACE_DBG */

				if (dev->timer.wait_time < adjust_time) {
#if (SKIP_COUNT_MAX > 0)
					skip_count++;
#endif
					vb->state       = VIDEOBUF_CANCELED;
					vb->state_frame = STATE_FRAME_SKIPPED;
					ret             = STATE_FRAME_SKIPPED;

					up(&dev->sem_lcdout);
					trace_add(vb,
					 "chk_frameskip:up sem_lcdout");

					emxx_v4l2_next_proccess(dev, vb, 0,
								call_func);
				}
#if (SKIP_COUNT_MAX > 0)
				else
					skip_count = 0;

			}
#endif
		}
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return ret;
}
#endif /* CONFIG_VIDEO_EMXX_FRAMESKIP */


/*===============================================================*/
/* kernel timer function					 */
/*===============================================================*/
/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_cmpare_time
* FUNCTION : wait to call LCD driver function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static inline long emxx_v4l2_tmr_cmpare_time(struct timeval *start,
 struct timeval *end)
{
	long delta = 0;

	if (end->tv_sec > start->tv_sec) {
		delta = (end->tv_sec - start->tv_sec) * 1000;
		if (end->tv_usec > start->tv_usec)
			delta += (end->tv_usec - start->tv_usec) / 1000;
		else
			delta -= (start->tv_usec - end->tv_usec) / 1000;

	} else if (end->tv_sec == start->tv_sec) {
		if (end->tv_usec > start->tv_usec)
			delta = (end->tv_usec - start->tv_usec) / 1000;
		else
			delta = STATE_FRAME_IMMEDIATE;

	} else
		delta = STATE_FRAME_IMMEDIATE;

	return delta;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_calc_waittime
* FUNCTION : wait to call LCD driver function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static inline long emxx_v4l2_tmr_calc_waittime(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb)
{
	long adjust_time;

	do_gettimeofday(&dev->timer.now_time);
	dev->timer.wait_time =
		emxx_v4l2_tmr_cmpare_time(&dev->timer.now_time, &vb->ts);
	printk_dbg(_TMR_DBG, "waittime(%ld)\n", dev->timer.wait_time);

#if _TRACE_DBG
	memset(cp_name, 0, sizeof(cp_name));
	sprintf(cp_name, "calc_waittime: vb->ts.tv_usec     (%ld)",
	 vb->ts.tv_usec);
	trace_add(vb, cp_name);
	memset(cp_name, 0, sizeof(cp_name));
	sprintf(cp_name, "calc_waittime: dev->timer.now_time(%ld)",
	 dev->timer.now_time.tv_usec);
	trace_add(vb, cp_name);
#endif /* _TRACE_DBG */

#ifdef CONFIG_EMXX_NTS
	if ((vb->output == V4L2_OUTPUT_LCD) ||
	    (vb->output == V4L2_OUTPUT_HDMI_1080I) ||
	    (vb->output == V4L2_OUTPUT_HDMI_720P)) {
#endif /* CONFIG_EMXX_NTS */
		adjust_time = dev->timer.adjust_time_lcd;
#ifdef CONFIG_EMXX_NTS
	} else {
		adjust_time = dev->timer.adjust_time_ntsc;
	}
#endif /* CONFIG_EMXX_NTS */
	return adjust_time;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_chk_waittime
* FUNCTION : wait to call LCD driver function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static inline void emxx_v4l2_tmr_chk_waittime(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb)
{
	long adjust_time = emxx_v4l2_tmr_calc_waittime(dev, vb);
	long roundup;

#ifdef CONFIG_EMXX_NTS
	if ((vb->output == V4L2_OUTPUT_LCD) ||
	    (vb->output == V4L2_OUTPUT_HDMI_1080I) ||
	    (vb->output == V4L2_OUTPUT_HDMI_720P))
#endif /* CONFIG_EMXX_NTS */
		roundup = ROUNDUP_ADJUST_LCD;
#ifdef CONFIG_EMXX_NTS
	else if (vb->output == V4L2_OUTPUT_NTSC)
		roundup = ROUNDUP_ADJUST_NTSC;
	else
		roundup = ROUNDUP_ADJUST_PAL;
#endif /* CONFIG_EMXX_NTS */

	adjust_time += roundup;

#if _TRACE_DBG
	memset(cp_name, 0, sizeof(cp_name));
	sprintf(cp_name,
	 "chk_waittime: waittime(%ld) = wait(%ld) - adjust(%ld)",
	 dev->timer.wait_time - adjust_time, dev->timer.wait_time, adjust_time);
	trace_add(vb, cp_name);
#endif /* _TRACE_DBG */

	if (dev->timer.wait_time >= adjust_time)
		dev->timer.wait_time -= adjust_time;
	else
		dev->timer.wait_time = STATE_FRAME_IMMEDIATE;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_calc_adjusttime
* FUNCTION : wait to call LCD driver function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static inline long emxx_v4l2_tmr_calc_adjusttime(struct emxx_v4l2_device *dev)
{
	long adjust_time;

	do_gettimeofday(&dev->timer.comp_time);
	adjust_time = emxx_v4l2_tmr_cmpare_time(&dev->timer.do_time,
	 &dev->timer.comp_time);
	printk_dbg(_TMR_DBG, "adjusttime(%ld)\n", adjust_time);

	return adjust_time;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_init
* FUNCTION : wait to call LCD driver init function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static inline void emxx_v4l2_tmr_init(struct emxx_v4l2_device *dev)
{
	dev->timer.adjust_time_lcd  = DEFAULT_ADJUST_LCD;
#ifdef CONFIG_EMXX_NTS
	dev->timer.adjust_time_ntsc = DEFAULT_ADJUST_NTSC;
#endif /* CONFIG_EMXX_NTS */

	init_timer(&dev->timer.wait_timer);
	dev->timer.wait_timer.function = emxx_v4l2_tmr_callback_top;
	dev->timer.wait_timer.data = (unsigned long)dev;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_request
* FUNCTION : wait to call LCD driver function.
* RETURN   : 0: success
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_tmr_request(struct emxx_v4l2_device *dev,
	struct videobuf_buffer *vb, int dummy)
{
	int ret = 0;
	unsigned long flags;

	printk_dbg(_TMR_DBG, "in.\n");
	trace_add(vb, "tmr_request");

#ifndef CONFIG_VIDEO_EMXX_FRAMESKIP
	down(&dev->sem_lcdout);
	trace_add(vb, "tmr_request:down sem_lcdout");
#endif

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	/* check next queue status */
	if (emxx_v4l2_next_proccess(dev, vb, 0, TMR_REQUEST)) {
		printk_dbg(_TMR_DBG, "VIDEOBUF_CANCELED.\n");
		ret = -1;
		up(&dev->sem_lcdout); /* release semafore */
		trace_add(vb, "tmr_request:up sem_lcdout");
	} else {
		emxx_v4l2_tmr_chk_waittime(dev, vb);
		dev->timer.vb	= vb;
		vb->state_frame = STATE_FRAME_DONE;

		if (dev->timer.wait_time >= 10) {
			trace_add(vb, "tmr_request: add_timer()");
			perf_add(vb->sequence, PHS_TMR);
			dev->timer.wait_timer.expires = jiffies
			 + (dev->timer.wait_time * HZ / 1000);
			add_timer(&dev->timer.wait_timer);
		} else {
			trace_add(vb, "tmr_request: queue_work()");
#if ENABLE_DELAY
			queue_work(dev->v4l2_workqueue,
			 &dev->wk_tmr_callback_bottom);
#else
			emxx_v4l2_tmr_callback_bottom_do(dev);
#endif
		}
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return ret;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_callback_top
* FUNCTION : wait to call LCD driver function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
static inline void emxx_v4l2_tmr_callback_top(unsigned long data)
{
	struct emxx_v4l2_device *dev = (struct emxx_v4l2_device *)data;
#if ENABLE_DELAY
#else
	unsigned long flags;
#endif

	trace_add(((struct videobuf_buffer *)(dev->timer.vb)),
	 "tmr_callback_top_half");
	perf_add(((struct videobuf_buffer *)(dev->timer.vb))->sequence,
	 PHS_TMR);
#if ENABLE_DELAY
	queue_work(dev->v4l2_workqueue, &dev->wk_tmr_callback_bottom);
#else
	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	emxx_v4l2_tmr_callback_bottom_do(dev);
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
#endif
}


/*****************************************************************************
* MODULE   : emxx_v4l2_tmr_callback_bottom_do
* FUNCTION : wait to call LCD driver function.
* RETURN   : -
* NOTE	   :
******************************************************************************/
#if ENABLE_DELAY
static void emxx_v4l2_tmr_callback_bottom_do(struct work_struct *num)
{
	struct emxx_v4l2_device *dev =
		container_of(num, struct emxx_v4l2_device,
			wk_tmr_callback_bottom);
	struct videobuf_buffer	 *vb  = dev->timer.vb;
	unsigned long flags;
	long wait_time;
#else
static inline void
emxx_v4l2_tmr_callback_bottom_do(struct emxx_v4l2_device *dev)
{
	struct videobuf_buffer *vb = dev->timer.vb;
#endif
	int ret = 0;
	dev->timer.vb = NULL;

	trace_add(vb, "tmr_callback_bottom_half");
	printk_dbg(_WKQ_DBG, "in\n");
	printk_dbg(_VBQ_DBG, "(%p) ->prev(%p) ->next(%p) ->state(%d)\n",
	 &vb->stream, vb->stream.prev, vb->stream.next, vb->state);

#if ENABLE_DELAY
	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	emxx_v4l2_tmr_chk_waittime(dev, vb);
	wait_time = dev->timer.wait_time;
	spin_unlock_irqrestore(&dev->vbq_lock, flags);

	if (wait_time > 10) {
		trace_add(vb, "tmr_callback_bottom_half: msleep(TMR) <--");
		msleep((unsigned int)wait_time);
		trace_add(vb, "tmr_callback_bottom_half: msleep(TMR) -->");
	}
#endif

	/* check next queue status */
#if ENABLE_DELAY
	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
#endif
	ret = emxx_v4l2_next_proccess(dev, vb, 0, TMR_CALLBACK);
#if ENABLE_DELAY
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
#endif
	if (ret) {
		printk_dbg(_TMR_DBG, "VIDEOBUF_CANCELED.\n");
		if (down_trylock(&dev->sem_lcdout))
			;

		up(&dev->sem_lcdout); /* release semafore */
		trace_add(vb, "tmr_callback:up sem_lcdout");
	}
}


/*===============================================================*/
/* ROT proccessing function prottyped				 */
/*===============================================================*/
/*****************************************************************************
* MODULE   : emxx_v4l2_rot_callback_main
* FUNCTION : ROT driver callback function.
* RETURN   : -
* NOTE	   : When the animation rotation is completed/error,
*	   : the ROT driver calls this function.
******************************************************************************/
static void emxx_v4l2_rot_callback_main(struct emxx_v4l2_device *dev,
 struct th_object *th_obj, enum call_func_num call_func, int flag)
{
	unsigned long flags;
	struct videobuf_buffer *vb = th_obj->th_buf->vb;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	th_obj->th_buf->vb = NULL;

	trace_add_th(vb, "rot_callback(", ")");
	perf_add_th(vb->sequence);

	/* check callback flag */
	if (flag != M2M_DMA_CALLBACK_SUCCESS)
		vb->state = VIDEOBUF_CANCELED;

	/* check next queue status */
	if (emxx_v4l2_next_proccess(dev, vb, 0, call_func))
		printk_dbg((_ROT_DBG), "VIDEOBUF_CANCELED.\n");

	spin_unlock_irqrestore(&dev->vbq_lock, flags);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_rot_request_main
* FUNCTION : request to ROT driver function.
* RETURN   :  0: success
*	   : -1: buffer busy
* NOTE	   : This function the movie rotation demand to the ROT driver.
******************************************************************************/
static int emxx_v4l2_rot_request_main(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb, struct th_object *th_obj,
 enum call_func_num call_func)
{
	enum buffer_select  buf_sel;
	unsigned long flags;
	int i;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	trace_add(vb, "rot_call_func_main");

	/* check use buffer */
	for (i = 0; i < NUM_BUF_V4L2; i++) {
		if (th_obj->th_buf->buf[i].state == TMPBUF_IDLE) {
			buf_sel = i;
			break;
		}
	}
	if (i >= NUM_BUF_V4L2) {
		dbg_lock(&dev->vbq_lock, "buffer busy\n");
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		return -1;
	}

	/* check next queue status */
	if (emxx_v4l2_next_proccess(dev, vb, buf_sel, call_func)) {
		dbg_lock(&dev->vbq_lock, "VIDEOBUF_CANCELED\n");
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		return -1;
	}

	/* make ROT request data */
	perf_add_th(vb->sequence);
	if (th_obj->th_buf->make_request(dev, vb, buf_sel)) {
		/* check next queue status */
		if (emxx_v4l2_next_proccess(dev, vb, 0, VBQ_COMPLETE)) {
			/* CANCELED */
			trace_add(vb,
			 "rot_request_main: next_proccess canceled.\n");
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			return -1;
		}
		emxx_v4l2_core_vbq_complete(dev, vb, 1);
		printk_info("ROT request data making error.\n\n");
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		return -1;
	}

	th_obj->th_buf->vb			= (void *)vb;

	/* call ROT driver */
	if (th_obj->th_buf->siz_param.src_hsize > 0) {
		emxx_set_dma_to_siz(dev->siz_info.id,
		 &th_obj->th_buf->dma_param);
		emxx_start_dma_to_siz(dev->siz_info.id,
		 th_obj->th_buf->callback);
	} else {
		emxx_set_dma_to_rot(dev->rot_info.id,
		 &th_obj->th_buf->dma_param);
		emxx_start_dma_to_rot(dev->rot_info.id,
		 th_obj->th_buf->callback);
	}

	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_rot_callback
* FUNCTION : ROT driver callback function.
* RETURN   : -
* NOTE	   : When the animation rotation is completed/error,
*	   : the ROT driver calls this function.
******************************************************************************/
static inline void emxx_v4l2_rot_callback(int flag)
{
	struct emxx_v4l2_device *dev = emxx_dev;

	trace_add(dev->th_rot.th_buf->vb, "rot_callback");
	printk_dbg((_ROT_DBG & 0x2), "call in.\n");

	/* ROT callback main function */
	emxx_v4l2_rot_callback_main(dev, &dev->th_rot, ROT_CALLBACK, flag);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_rot_request
* FUNCTION : request to ROT driver function.
* RETURN   :  0: success
*	   : -1: buffer busy
* NOTE	   : This function the movie rotation demand to the ROT driver.
******************************************************************************/
static inline int emxx_v4l2_rot_request(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb, int dummy)
{
	trace_add(vb, "rot_call_func");
	return emxx_v4l2_rot_request_main(dev, vb, &dev->th_rot, ROT_REQUEST);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_rot_make_req
* FUNCTION : make rotate request data to ROT driver.
* RETURN   :  0: success
*	   : -1: error
* NOTE	   : The register information table make, requested to the ROT driver.
******************************************************************************/
static int emxx_v4l2_rot_make_req(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb, enum buffer_select buf_sel)
{
	struct buf_object  *rot_buf  = dev->th_rot.th_buf;
	struct _IMAGE_DATA *rot_data = &vb->rot_buf->dst_data;
	unsigned long pixelformat;
	unsigned long src_width, src_height;
	unsigned long siz_dst_width, siz_dst_height;
	unsigned long rot_mode, src_format, dst_format;
	unsigned long base_addr;
	unsigned long src_yrgbaddr = 0, src_uvaddr = 0, src_vaddr = 0;
	int bpp_y, bpp_uv, bpp_v, bpp_y_out; /* bit par pixel  */
	int ppl_y, ppl_uv, ppl_v; /* pixel par line */

	trace_add(vb, "make_rot_req");

	/* init ROT request data */
	memset(&rot_buf->siz_param, 0, sizeof(struct emxx_siz_param));
	memset(&rot_buf->rot_param, 0, sizeof(struct emxx_rot_param));
	memset(&rot_buf->dma_param, 0, sizeof(struct emxx_dma_param));
	memset(rot_data, 0, sizeof(struct _IMAGE_DATA));

	/* ROT MODE */
	switch (vb->save_effect.movie_angle) {
	default:
	case ROT2_MODE_MODE_0: /*   0 deg */
		rot_mode = ROT2_MODE_MODE_0   | ROT2_MODE_BOUNDARY_2_12;
		break;
	case ROT2_MODE_MODE_90: /*  90 deg */
		rot_mode = ROT2_MODE_MODE_90  | ROT2_MODE_BOUNDARY_2_12;
		break;
	case ROT2_MODE_MODE_180: /* 180 deg */
		rot_mode = ROT2_MODE_MODE_180 | ROT2_MODE_BOUNDARY_2_12;
		break;
	case ROT2_MODE_MODE_270: /* 270 deg */
		rot_mode = ROT2_MODE_MODE_270 | ROT2_MODE_BOUNDARY_2_12;
		break;
	}

	/* FORMAT */
	pixelformat = vb->pixelformat;
	switch (vb->pixelformat) {
	case V4L2_PIX_FMT_NV12:	   /* YUV420 Semi-Planar */
		src_format = ROT2_FORMAT_YUV420SP;
		if (emxx_v4l2_chk_siz(dev, vb)) {
			/* SP -> IL */
			dst_format = ROT2_FORMAT_YUV422IL;
			pixelformat = V4L2_PIX_FMT_YUYV;
			bpp_y_out = 16;
		} else {
			dst_format = ROT2_FORMAT_YUV420SP;
			bpp_y_out = 8;
		}
		bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
		ppl_y = 1; ppl_uv = 2; ppl_v = 1;
		break;
	case V4L2_PIX_FMT_NV422:   /* YUV422 Semi-Planar */
		src_format = ROT2_FORMAT_YUV422SP;
		if (emxx_v4l2_chk_siz(dev, vb)) {
			/* SP -> IL */
			dst_format = ROT2_FORMAT_YUV422IL;
			pixelformat = V4L2_PIX_FMT_YUYV;
			bpp_y_out = 16;
		} else {
			dst_format = ROT2_FORMAT_YUV422SP;
			bpp_y_out = 8;
		}
		bpp_y = 8;  bpp_uv = 8; bpp_v = 0;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		break;
	case V4L2_PIX_FMT_YUV420:  /* YUV420 Planar      */
		src_format = ROT2_FORMAT_YUV420PL;
		if (emxx_v4l2_chk_siz(dev, vb)) {
			/* PL -> IL */
			dst_format = ROT2_FORMAT_YUV422IL;
			pixelformat = V4L2_PIX_FMT_YUYV;
			bpp_y_out = 16;
		} else {
			dst_format = ROT2_FORMAT_YUV420PL;
			bpp_y_out = 8;
		}
		bpp_y = 8;  bpp_uv = 4; bpp_v = 4;
		ppl_y = 1; ppl_uv = 2; ppl_v = 2;
		break;
#if VF_YUV422PL
	case V4L2_PIX_FMT_YUV422P: /* YUV422 Planar      */
		src_format = ROT2_FORMAT_YUV422PL;
		if (emxx_v4l2_chk_siz(dev, vb)) {
			/* PL -> IL */
			dst_format = ROT2_FORMAT_YUV422IL;
			pixelformat = V4L2_PIX_FMT_YUYV;
			bpp_y_out = 16;
		} else {
			dst_format = ROT2_FORMAT_YUV422PL;
			bpp_y_out = 8;
		}
		bpp_y = 8;  bpp_uv = 4; bpp_v = 4;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		break;
#endif
#if VF_YUV422PX
	case V4L2_PIX_FMT_YUYV:   /* YUV422 Interleave   */
		src_format = ROT2_FORMAT_YUV422IL;
		dst_format = ROT2_FORMAT_YUV422IL;
		bpp_y_out = 16;
		bpp_y = 16; bpp_uv = 0; bpp_v = 0;
		ppl_y = 1; ppl_uv = 1; ppl_v = 1;
		break;
#endif
	default:
		return -1;
	}

	/* SIZE */
	src_width  = vb->save_effect.source_crop.width;
	src_height = vb->save_effect.source_crop.height;
	if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
	 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
		siz_dst_width  = vb->save_effect.screen.width;
		siz_dst_height = vb->save_effect.screen.height;
		rot_data->hsize = siz_dst_width;
		rot_data->vsize = siz_dst_height;
		rot_data->size	= siz_dst_width * bpp_y_out / 8;
	} else {
		siz_dst_width  = vb->save_effect.screen.height;
		siz_dst_height = vb->save_effect.screen.width;
		rot_data->hsize = siz_dst_height;
		rot_data->vsize = siz_dst_width;
		rot_data->size	= siz_dst_height * bpp_y_out / 8;
	}

	vb->rot_buf->pixelformat = pixelformat;

	base_addr = vb->rot_buf->paddr;

	/* ADDR */
	/* src */
	src_yrgbaddr	= vb->base_addr.PhysAddr_Y
	  + vb->save_effect.source_crop.top
	   * vb->bytesperline / ppl_y
	    + vb->save_effect.source_crop.left * bpp_y  / 8;
	src_uvaddr	   = vb->base_addr.PhysAddr_UV
	  + vb->save_effect.source_crop.top
	   * vb->bytesperline * bpp_uv / bpp_y / ppl_uv
	    + vb->save_effect.source_crop.left * bpp_uv / 8;
	src_vaddr	= vb->base_addr.PhysAddr_V
	  + vb->save_effect.source_crop.top
	   * vb->bytesperline * bpp_v  / bpp_y / ppl_v
	    + vb->save_effect.source_crop.left * bpp_v  / 8;
	/* dst */
	rot_data->yrgbaddr	= base_addr;
	rot_data->uvaddr	= rot_data->yrgbaddr
	 + rot_data->vsize * rot_data->hsize * bpp_y  / 8 / ppl_y;
	rot_data->vaddr		= rot_data->uvaddr
	 + rot_data->vsize * rot_data->hsize * bpp_uv / 8 / ppl_uv;

	switch (vb->pixelformat) {
#if VF_YUV422PX
	case V4L2_PIX_FMT_YUYV:		/* YUV422 Interleave  */
		src_uvaddr		= 0;
		src_vaddr		= 0;
		rot_data->uvaddr	= 0;
		rot_data->vaddr		= 0;
		break;
#endif
	case V4L2_PIX_FMT_NV422:	/* YUV422 Semi-Planar */
	case V4L2_PIX_FMT_NV12:		/* YUV420 Semi-Planar */
		src_vaddr		= 0;
		rot_data->vaddr		= 0;
		if (dst_format == ROT2_FORMAT_YUV422IL)
			rot_data->uvaddr	= 0;	/* SP -> IL */
		break;
#if VF_YUV422PL
	case V4L2_PIX_FMT_YUV422P:	/* YUV422 Planar      */
#endif
	case V4L2_PIX_FMT_YUV420:	/* YUV420 Planar      */
		if (dst_format == ROT2_FORMAT_YUV422IL) {
			rot_data->uvaddr	= 0;	/* PL -> IL */
			rot_data->vaddr		= 0;	/* PL -> IL */
		}
		break;
	}

	/* SIZ parameter */
	if (emxx_v4l2_chk_siz(dev, vb)) {
		rot_buf->siz_param.src_hsize	= src_width;
		rot_buf->siz_param.src_vsize	= src_height;
		rot_buf->siz_param.src_format	= src_format;
		rot_buf->siz_param.dst_hsize	= siz_dst_width;
		rot_buf->siz_param.dst_vsize	= siz_dst_height;
		if (vb->save_effect.movie_angle) {
			rot_buf->siz_param.dst_hskip	=
			 0x1000 - siz_dst_width * bpp_y_out / 8;
			rot_buf->siz_param.dst_adryrgb	=
			 dev->rot_info.adryrgb;
			rot_buf->siz_param.dst_adruv	=
			 dev->rot_info.adruv;
			rot_buf->siz_param.dst_adrv	=
			 dev->rot_info.adrv;
		} else {
			rot_buf->siz_param.dst_hskip	= 0;
			rot_buf->siz_param.dst_adryrgb	= rot_data->yrgbaddr;
			rot_buf->siz_param.dst_adruv	= rot_data->uvaddr;
			rot_buf->siz_param.dst_adrv	= rot_data->vaddr;
		}
		rot_buf->siz_param.dst_format	= dst_format;
		rot_buf->siz_param.dst_bytelane	= SIZ_DSTBL_RESET;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		rot_buf->siz_param.hstep = 256*vb->image_width/siz_dst_width;
		rot_buf->siz_param.vstep = 256*vb->image_height/siz_dst_height;
#else
		rot_buf->siz_param.hstep	= 256*src_width/siz_dst_width;
		rot_buf->siz_param.vstep	= 256*src_height/siz_dst_height;
#endif
		rot_buf->siz_param.dst_hcrop	= 0;
		rot_buf->siz_param.dst_vcrop	= 0;
		rot_buf->siz_param.rot_dst_format	= SIZ_ROTDSTFMT_OFF;
#ifdef CONFIG_VIDEO_EMXX_FILTER
		rot_buf->siz_param.filter_option = vb->filter.filter_option;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		if (!(vb->output != V4L2_OUTPUT_HDMI_1080I &&
		      (vb->field == V4L2_FIELD_INTERLACED ||
		       vb->field == V4L2_FIELD_INTERLACED_TB ||
		       vb->field == V4L2_FIELD_INTERLACED_BT)) &&
		    vb->image_width  == siz_dst_width &&
		    vb->image_height == siz_dst_height)
#else
		if (!(vb->output != V4L2_OUTPUT_HDMI_1080I &&
		      (vb->field == V4L2_FIELD_INTERLACED ||
		       vb->field == V4L2_FIELD_INTERLACED_TB ||
		       vb->field == V4L2_FIELD_INTERLACED_BT)) &&
		    src_width  == siz_dst_width &&
		    src_height == siz_dst_height)
#endif
			rot_buf->siz_param.filter_option |=
			 (SIZ_FILTOPT_X_FILTER_THROUGH |
			  SIZ_FILTOPT_Y_FILTER_THROUGH);
		rot_buf->siz_param.filt0	 = vb->filter.filt0;
		rot_buf->siz_param.filt1	 = vb->filter.filt1;
		rot_buf->siz_param.filt2	 = vb->filter.filt2;
		rot_buf->siz_param.filt3	 = vb->filter.filt3;
		rot_buf->siz_param.filt4	 = vb->filter.filt4;
		rot_buf->siz_param.filt5	 = vb->filter.filt5;
		rot_buf->siz_param.filt6	 = vb->filter.filt6;
		rot_buf->siz_param.filt7	 = vb->filter.filt7;
#else
		rot_buf->siz_param.filter_option = SIZ_FILTER_DEFAULT;
#endif

		emxx_set_siz(dev->siz_info.id, &rot_buf->siz_param);
	}

	/* ROT parameter */
	if (vb->save_effect.movie_angle) {
		rot_buf->rot_param.mode		= rot_mode;
		rot_buf->rot_param.src_hsize	= siz_dst_width;
		rot_buf->rot_param.src_vsize	= siz_dst_height;
		rot_buf->rot_param.src_format	= dst_format;
		rot_buf->rot_param.dst_adryrgb	= rot_data->yrgbaddr;
		rot_buf->rot_param.dst_adruv	= rot_data->uvaddr;
		rot_buf->rot_param.dst_adrv	= rot_data->vaddr;
		rot_buf->rot_param.dst_bytelane	= ROT2_DSTBL_RESET;
		rot_buf->rot_param.input_mode   = RANDOM_MODE;

		emxx_set_rot(dev->rot_info.id, &rot_buf->rot_param);
	}

	/* M2M parameter */
	rot_buf->dma_param.src_hsize		= src_width;
	rot_buf->dma_param.src_vsize		= src_height;
	rot_buf->dma_param.src_hskipyrgb	=
	 (vb->bytesperline * 8 / bpp_y - src_width) * bpp_y  / 8;
	rot_buf->dma_param.src_hskipuv		=
	 (vb->bytesperline * 8 / bpp_y - src_width) * bpp_uv / 8;
	rot_buf->dma_param.src_hskipv		=
	 (vb->bytesperline * 8 / bpp_y - src_width) * bpp_v  / 8;
	rot_buf->dma_param.src_adryrgb		= src_yrgbaddr;
	rot_buf->dma_param.src_adruv		= src_uvaddr;
	rot_buf->dma_param.src_adrv		= src_vaddr;
	rot_buf->dma_param.src_format		= src_format;

	printk_dbg(_ROT_DBG, "=== emxx_set_siz() =====================\n");
	printk_dbg(_ROT_DBG, "src_hsize     :%ld\n",
	 rot_buf->siz_param.src_hsize);
	printk_dbg(_ROT_DBG, "src_vsize     :%ld\n",
	 rot_buf->siz_param.src_vsize);
	printk_dbg(_ROT_DBG, "src_format    :%ld\n",
	 rot_buf->siz_param.src_format);
	printk_dbg(_ROT_DBG, "dst_hsize     :%ld\n",
	 rot_buf->siz_param.dst_hsize);
	printk_dbg(_ROT_DBG, "dst_vsize     :%ld\n",
	 rot_buf->siz_param.dst_vsize);
	printk_dbg(_ROT_DBG, "dst_hskip     :%ld\n",
	 rot_buf->siz_param.dst_hskip);
	printk_dbg(_ROT_DBG, "dst_adryrgb   :0x%08lx\n",
	 rot_buf->siz_param.dst_adryrgb);
	printk_dbg(_ROT_DBG, "dst_adruv     :0x%08lx\n",
	 rot_buf->siz_param.dst_adruv);
	printk_dbg(_ROT_DBG, "dst_adrv      :0x%08lx\n",
	 rot_buf->siz_param.dst_adrv);
	printk_dbg(_ROT_DBG, "dst_format    :%ld\n",
	 rot_buf->siz_param.dst_format);
	printk_dbg(_ROT_DBG, "dst_bytelane  :0x%08lx\n",
	 rot_buf->siz_param.dst_bytelane);
	printk_dbg(_ROT_DBG, "hstep         :0x%08lx\n",
	 rot_buf->siz_param.hstep);
	printk_dbg(_ROT_DBG, "vstep         :0x%08lx\n",
	 rot_buf->siz_param.vstep);
	printk_dbg(_ROT_DBG, "dst_hcrop     :%ld\n",
	 rot_buf->siz_param.dst_hcrop);
	printk_dbg(_ROT_DBG, "dst_vcrop     :%ld\n",
	 rot_buf->siz_param.dst_vcrop);
	printk_dbg(_ROT_DBG, "rot_dst_format:%ld\n",
	 rot_buf->siz_param.rot_dst_format);
	printk_dbg(_ROT_DBG, "filter_option :%ld\n",
	 rot_buf->siz_param.filter_option);

	printk_dbg(_ROT_DBG, "=== emxx_set_rot() =====================\n");
	printk_dbg(_ROT_DBG, "mode          :0x%08lx\n",
	 rot_buf->rot_param.mode);
	printk_dbg(_ROT_DBG, "src_hsize     :%ld\n",
	 rot_buf->rot_param.src_hsize);
	printk_dbg(_ROT_DBG, "src_vsize     :%ld\n",
	 rot_buf->rot_param.src_vsize);
	printk_dbg(_ROT_DBG, "src_format    :%ld\n",
	 rot_buf->rot_param.src_format);
	printk_dbg(_ROT_DBG, "dst_adryrgb   :0x%08lx\n",
	 rot_buf->rot_param.dst_adryrgb);
	printk_dbg(_ROT_DBG, "dst_adruv     :0x%08lx\n",
	 rot_buf->rot_param.dst_adruv);
	printk_dbg(_ROT_DBG, "dst_adrv      :0x%08lx\n",
	 rot_buf->rot_param.dst_adrv);
	printk_dbg(_ROT_DBG, "dst_bytelane  :0x%08lx\n",
	 rot_buf->rot_param.dst_bytelane);
	printk_dbg(_ROT_DBG, "input_mode    :%ld\n",
	 rot_buf->rot_param.input_mode);

	printk_dbg(_ROT_DBG, "=== emxx_set_dma_to_xxx() ==============\n");
	printk_dbg(_ROT_DBG, "src_hsize     :%ld\n",
	 rot_buf->dma_param.src_hsize);
	printk_dbg(_ROT_DBG, "src_vsize     :%ld\n",
	 rot_buf->dma_param.src_vsize);
	printk_dbg(_ROT_DBG, "src_hskipyrgb :%ld\n",
	 rot_buf->dma_param.src_hskipyrgb);
	printk_dbg(_ROT_DBG, "src_hskipuv   :%ld\n",
	 rot_buf->dma_param.src_hskipuv);
	printk_dbg(_ROT_DBG, "src_hskipv    :%ld\n",
	 rot_buf->dma_param.src_hskipv);
	printk_dbg(_ROT_DBG, "src_adryrgb   :0x%08lx\n",
	 rot_buf->dma_param.src_adryrgb);
	printk_dbg(_ROT_DBG, "src_adruv     :0x%08lx\n",
	 rot_buf->dma_param.src_adruv);
	printk_dbg(_ROT_DBG, "src_adrv      :0x%08lx\n",
	 rot_buf->dma_param.src_adrv);
	printk_dbg(_ROT_DBG, "src_format    :%ld\n",
	 rot_buf->dma_param.src_format);


	return 0;
}


/*===============================================================*/
/* LCD proccessing function prottyped				 */
/*===============================================================*/
/*****************************************************************************
* MODULE   : emxx_v4l2_notify_lcd_output
* FUNCTION : Notify LCD driver output mode.
* RETURN   : -
* NOTE	   : The LCD driver calls this function.
******************************************************************************/
void emxx_v4l2_notify_lcd_output(unsigned int output)
{
	struct emxx_v4l2_device *dev = emxx_dev;
	struct emxx_v4l2_fh *fh = NULL;

	printk_dbg((_LCD_DBG & 0x2), "call in.\n");

	dev->output_lcd = output;

	if (dev->output != dev->output_lcd) {
		if (dev->streaming) {
			fh = dev->streaming;
			emxx_v4l2_ioc_streamoff(fh, dev);
			emxx_v4l2_ioc_streamon(fh, dev);
		}
	}
}


/*****************************************************************************
* MODULE   : emxx_v4l2_lcd_callback
* FUNCTION : LCD driver callback function.
* RETURN   : -
* NOTE	   : The LCD driver calls this function.
******************************************************************************/
void emxx_v4l2_lcd_callback(FRAME_DATA frame_data)
{
	struct emxx_v4l2_device *dev;
	struct videobuf_buffer	 *vb;
	unsigned long flags;

	dev = (struct emxx_v4l2_device *)frame_data.dev;
	vb  = (struct videobuf_buffer *)frame_data.buf;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	printk_dbg((_LCD_DBG & 0x2), "call in.\n");
	perf_add(vb->sequence, PHS_LCD);

#ifdef CONFIG_EMXX_NTS
	if (((vb->output != V4L2_OUTPUT_LCD) &&
	     (vb->output != V4L2_OUTPUT_HDMI_1080I) &&
	     (vb->output != V4L2_OUTPUT_HDMI_720P))
	    && vb->state_frame != STATE_FRAME_IMMEDIATE) {
		if (down_trylock(&dev->sem_lcdout))
			;

		up(&dev->sem_lcdout); /* release semafore */
		trace_add(vb, "lcd_callback:up sem_lcdout");
	}
#endif /* CONFIG_EMXX_NTS */

#if CALC_ADJUST
 #ifdef CONFIG_EMXX_NTS
	if ((vb->output == V4L2_OUTPUT_LCD) ||
	    (vb->output == V4L2_OUTPUT_HDMI_1080I) ||
	    (vb->output == V4L2_OUTPUT_HDMI_720P)) {
 #endif /* CONFIG_EMXX_NTS */
		dev->timer.adjust_time_lcd =
			emxx_v4l2_tmr_calc_adjusttime(dev);
 #if _TRACE_DBG
		memset(cp_name, 0, sizeof(cp_name));
		sprintf(cp_name, "lcd_callback: adjust(%ld)",
		 dev->timer.adjust_time_lcd);
		trace_add(vb, cp_name);
 #endif /* _TRACE_DBG */
 #ifdef CONFIG_EMXX_NTS
	}
 #endif /* CONFIG_EMXX_NTS */
#endif

#if _PERF2_DBG
	Perf2Calc(dev, vb);
#endif
	emxx_v4l2_core_vbq_complete(dev, vb, 0);
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_lcd_refresh_callback
* FUNCTION : LCD driver callback function.
* RETURN   : -
* NOTE	   : The LCD driver calls this function.
******************************************************************************/
void emxx_v4l2_lcd_refresh_callback(FRAME_DATA frame_data)
{
	struct emxx_v4l2_device *dev;
	struct videobuf_buffer	 *vb;
	unsigned long flags;

	dev = (struct emxx_v4l2_device *)frame_data.dev;
	vb  = (struct videobuf_buffer *)frame_data.buf;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	printk_dbg((_LCD_DBG & 0x2), "call in.\n");
	trace_add(vb, "lcd_refresh_callback");
	perf_add(vb->sequence, PHS_LCD);

	if (
#ifdef CONFIG_EMXX_NTS
	    ((vb->output == V4L2_OUTPUT_LCD) ||
	     (vb->output == V4L2_OUTPUT_HDMI_1080I) ||
	     (vb->output == V4L2_OUTPUT_HDMI_720P)) &&
#endif /* CONFIG_EMXX_NTS */
	    vb->state_frame != STATE_FRAME_IMMEDIATE) {
		if (down_trylock(&dev->sem_lcdout))
			;

		up(&dev->sem_lcdout); /* release semafore */
		trace_add(vb, "lcd_refresh_callback:up sem_lcdout");
	}

	/* check next queue status */
	if (emxx_v4l2_next_proccess(dev, vb, 0, LCD_CALLBACK)) {
		/* CANCELED */
		trace_add(vb,
		 "lcd_refresh_callback: next_proccess canceled.\n");
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
}


#ifdef CONFIG_EMXX_NTS
/*****************************************************************************
* MODULE   : emxx_v4l2_ntsc_ready_callback
* FUNCTION : NTSC driver callback function. when finished SIZ.
* RETURN   : -
* NOTE	   :
******************************************************************************/
void emxx_v4l2_ntsc_ready_callback(FRAME_DATA frame_data)
{
	struct emxx_v4l2_device *dev;
	struct videobuf_buffer   *vb;
	unsigned long flags;

	dev = (struct emxx_v4l2_device *)frame_data.dev;
	vb  = (struct videobuf_buffer *)frame_data.buf;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	printk_dbg((_LCD_DBG & 0x2), "call in.\n");
	trace_add(vb, "ntsc_ready_callback");

	if (vb->output == V4L2_OUTPUT_NTSC || vb->output == V4L2_OUTPUT_PAL) {
		dev->timer.adjust_time_ntsc =
			emxx_v4l2_tmr_calc_adjusttime(dev);
#if _TRACE_DBG
		memset(cp_name, 0, sizeof(cp_name));
		sprintf(cp_name, "ntsc_ready_callback: adjust(%ld)",
		 dev->timer.adjust_time_ntsc);
		trace_add(vb, cp_name);
#endif /* _TRACE_DBG */
	}

	spin_unlock_irqrestore(&dev->vbq_lock, flags);
}
#endif /* CONFIG_EMXX_NTS */


/*****************************************************************************
* MODULE   : emxx_v4l2_lcd_set_outfunc
* FUNCTION : request to LCD driver function.
* RETURN   :  0: success
* NOTE	   : This function the drawing demand to the LCD driver.
******************************************************************************/
static inline void *emxx_v4l2_lcd_set_outfunc(int output)
{
	printk_dbg((_LCD_DBG || _ROT_DBG), "\n");

#ifdef CONFIG_EMXX_NTS
	if ((output == V4L2_OUTPUT_LCD) ||
	    (output == V4L2_OUTPUT_HDMI_1080I) ||
	    (output == V4L2_OUTPUT_HDMI_720P)) {
#endif /* CONFIG_EMXX_NTS */
		/* LCD driver call */
		return (void *)emxx_lcd_set_v4l2_image;
#ifdef CONFIG_EMXX_NTS
	} else {
		/* NTS driver call */
		return (void *)emxx_nts_set_v4l2_image;
	}
#endif /* CONFIG_EMXX_NTS */
}


/*****************************************************************************
* MODULE   : emxx_v4l2_lcd_request
* FUNCTION : request to LCD driver function.
* RETURN   :  0: success
* NOTE	   : This function the drawing demand to the LCD driver.
******************************************************************************/
static int emxx_v4l2_lcd_request(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb, int call_flg)
{
	int			 ret = 0;
	unsigned long		 flags;
	unsigned long		 pixelformat = 0;

	struct _V4L2_IMAGE_INFO	 image_info;
	int (*call_outfunc)(V4L2_IMAGE_INFO *);

	switch (call_flg) {
	case STREAM_PLAYBACK:
		trace_add(vb, "lcd_request(PLAYBACK)");
		printk_dbg(_VBQ_DBG,
		 "(%p) ->prev(%p)  ->next(%p) ->state(%d)\n",
		 &vb->stream, vb->stream.prev, vb->stream.next, vb->state);
#if CALC_ADJUST
		do_gettimeofday(&dev->timer.do_time);
#else
#ifdef CONFIG_EMXX_NTS
		if ((vb->output == V4L2_OUTPUT_NTSC)
		 || (vb->output == V4L2_OUTPUT_PAL)) {
			do_gettimeofday(&dev->timer.do_time);
		}
#endif /* CONFIG_EMXX_NTS */
#endif

		/* LCD driver's request information making */
		dbg_lock(&dev->vbq_lock, "vbq_lock\n");
		spin_lock_irqsave(&dev->vbq_lock, flags);

		/* check next queue status */
		if (emxx_v4l2_next_proccess(dev, vb, 0, LCD_REQUEST)) {
			printk_dbg((_LCD_DBG || _ROT_DBG),
			 "VIDEOBUF_CANCELED.\n");
			if (vb->state_frame != STATE_FRAME_IMMEDIATE) {
				if (down_trylock(&dev->sem_lcdout))
					;

				up(&dev->sem_lcdout); /* release semafore */
				trace_add(vb, "lcd_request:up sem_lcdout");
			}
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			return -1;
		}

		if (vb->state == VIDEOBUF_QUEUED)
			vb->state = VIDEOBUF_ACTIVE;

		/* movie reproduction processing */
		if (vb->state_proccess & STATE_PROCCESS_ROT_COMPLETE) {
			printk_dbg((_LCD_DBG || _ROT_DBG), "ROT processing.\n");
			memcpy(&image_info.image_data, &vb->rot_buf->dst_data,
			 sizeof(struct _IMAGE_DATA));
			pixelformat = vb->rot_buf->pixelformat;
		} else {
			image_info.image_data.size = vb->bytesperline;
			image_info.image_data.yrgbaddr =
				vb->base_addr.PhysAddr_Y;
			image_info.image_data.uvaddr =
				vb->base_addr.PhysAddr_UV;
			image_info.image_data.vaddr =
				vb->base_addr.PhysAddr_V;
			image_info.image_data.x =
				vb->save_effect.source_crop.left;
			image_info.image_data.y =
				vb->save_effect.source_crop.top;
			image_info.image_data.hsize =
				vb->save_effect.source_crop.width;
			image_info.image_data.vsize =
				vb->save_effect.source_crop.height;
			pixelformat = vb->pixelformat;
		}

		image_info.screen_data.x     = vb->save_effect.screen.left;
		image_info.screen_data.y     = vb->save_effect.screen.top;
		image_info.screen_data.hsize = vb->save_effect.screen.width;
		image_info.screen_data.vsize = vb->save_effect.screen.height;

		switch (pixelformat) {
		case V4L2_PIX_FMT_NV12:	   /* YUV420 Semi-Planar */
			image_info.yuvfmt = V4L2_FORMAT_YUV420Pl2;
			break;
		case V4L2_PIX_FMT_NV422:   /* YUV422 Semi-Planar */
			image_info.yuvfmt = V4L2_FORMAT_YUV422Pl2;
			break;
		case V4L2_PIX_FMT_YUV420:  /* YUV420 Planar      */
			image_info.yuvfmt = V4L2_FORMAT_YUV420Pl;
			break;
#if VF_YUV422PL
		case V4L2_PIX_FMT_YUV422P: /* YUV422 Planar      */
			image_info.yuvfmt = V4L2_FORMAT_YUV422Pl;
			break;
#endif
		case V4L2_PIX_FMT_YUYV:	   /* YUV422 Interleave  */
			image_info.yuvfmt = V4L2_FORMAT_YUV422Px;
			break;
		default:
			trace_add(vb, "lcd_request1");
			printk_dbg((_LCD_DBG || _ROT_DBG),
			 "not support format.\n");
			if (vb->state_frame != STATE_FRAME_IMMEDIATE) {
				if (down_trylock(&dev->sem_lcdout))
					;

				up(&dev->sem_lcdout); /* release semafore */
				trace_add(vb, "lcd_request:up sem_lcdout");
			}
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			return -1;
		}

		image_info.endian	  = V4L2_LITTLE_ENDIAN;
		image_info.frame_data.dev = (void *)dev;
		image_info.frame_data.buf = (void *)vb;

#ifdef CONFIG_EMXX_NTS
		if (vb->output == V4L2_OUTPUT_NTSC ||
		    vb->output == V4L2_OUTPUT_PAL)
			image_info.field = vb->field;
		else {
#endif /* CONFIG_EMXX_NTS */
			if (vb->output == V4L2_OUTPUT_HDMI_1080I) {
				if (vb->field == V4L2_FIELD_INTERLACED_BT)
					image_info.field = V4L2_BOTTOM_TOP;
				else
					image_info.field = V4L2_TOP_BOTTOM;
			} else {
				image_info.field = V4L2_TOP_BOTTOM;
			}
#ifdef CONFIG_EMXX_NTS
		}
#endif /* CONFIG_EMXX_NTS */

#ifdef CONFIG_EMXX_NTS
		memcpy(&image_info.siz_info, &dev->siz_info,
		 sizeof(struct emxx_siz_info));
		memcpy(&image_info.rot_info, &dev->rot_info,
		 sizeof(struct emxx_rot_info));

 #ifdef CONFIG_VIDEO_EMXX_FILTER
		memcpy(&image_info.filter, &vb->filter,
		 sizeof(struct v4l2_filter_option));
 #endif

 #ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		image_info.image_width  = vb->image_width;
		image_info.image_height = vb->image_height;
 #endif

		image_info.angle = vb->save_effect.movie_angle;
#endif /* CONFIG_EMXX_NTS */

		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "lcd_request: LCD driver call. ->sequence(%d)\n",
		 vb->sequence);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "=== emxx_lcd_set_v4l2_image() ==============================="
		 " sequence(%d) ===\n", vb->sequence);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 " src | Y(0x%08lx)  UV(0x%08lx)  V(0x%08lx)\n",
		 image_info.image_data.yrgbaddr, image_info.image_data.uvaddr,
		 image_info.image_data.vaddr);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "     | w(%4d)  h(%4d)  x(%4d)  y(%4d)\n",
		 image_info.image_data.hsize, image_info.image_data.vsize,
		 image_info.image_data.x, image_info.image_data.y);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "     | p(%d)\n", image_info.image_data.size);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "-----+-------------------------------------------------------"
		 "-------------------\n");
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 " dst | w(%4d)  h(%4d)  x(%4d)  y(%4d)\n",
		 image_info.screen_data.hsize, image_info.screen_data.vsize,
		 image_info.screen_data.x, image_info.screen_data.y);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "     | endi(%d), fmt(%d)\n",
		 image_info.endian, image_info.yuvfmt);
		printk_dbg((_LCD_DBG || _ROT_DBG),
		 "============================================================="
		 "===================\n\n");

		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		break;

	default:
	case STREAM_STOP:
		trace_add(0, "lcd_request(STOP)");
		printk_dbg((_LCD_DBG || _ROT_DBG), "STOP processing.\n");

		/* movie stop processing */
		memset(&image_info, 0, sizeof(struct _IMAGE_DATA));
		printk_dbg((_LCD_DBG || _ROT_DBG), "LCD driver call.\n");
		break;
	}

#ifdef CONFIG_FB_EMXX
	if (call_flg != STREAM_STOP) {
		perf_add(vb->sequence, PHS_LCD);
		call_outfunc = emxx_v4l2_lcd_set_outfunc(vb->output);
		image_info.output_mode    = vb->output;

	} else {
		call_outfunc = emxx_v4l2_lcd_set_outfunc(dev->output);
		image_info.output_mode    = dev->output;

	}

	if (call_outfunc(&image_info)) {
		if (call_flg != STREAM_STOP) {
			/* check next queue status */
			dbg_lock(&dev->vbq_lock, "vbq_lock\n");
			spin_lock_irqsave(&dev->vbq_lock, flags);
			if (emxx_v4l2_next_proccess(dev,
				vb, 0, VBQ_COMPLETE)) {
				/* CANCELED */
				trace_add(vb,
				 "lcd_request: next_proccess canceled.\n");
			} else {
				emxx_v4l2_core_vbq_complete(dev, vb, 1);
			}
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			ret = -1;
		}
	}
#endif
	return ret;
}


/* ------------------ videobuf_queue_ops ----------------------------------- */
/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_release
* FUNCTION :
* RETURN   : -
* NOTE	   : This function is called a videobuf_buffer release.
******************************************************************************/
static inline void emxx_v4l2_core_vbq_release(struct videobuf_queue *q,
 struct videobuf_buffer *vb)
{
	videobuf_waiton(vb, 0, 0);
	emxx_v4l2_core_vbq_dqueue(q, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_dqueue
* FUNCTION :
* RETURN   : -
* NOTE	   : This function executes video buffer information.
******************************************************************************/
static inline void emxx_v4l2_core_vbq_dqueue(struct videobuf_queue *q,
 struct videobuf_buffer *vb)
{
	struct emxx_v4l2_device *dev = q->priv_data;
	unsigned long flags;
	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	emxx_v4l2_next_proccess(dev, vb, 0, VBQ_DQUEUE);
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_cancel
* FUNCTION :
* RETURN   : 0: success
*	   : 1: stop timer
* NOTE	   : This function registers information to video buffer.
******************************************************************************/
static int emxx_v4l2_core_vbq_cancel(struct videobuf_queue *q,
 struct videobuf_buffer *vb)
{
	struct emxx_v4l2_device *dev = q->priv_data;
	unsigned int  retval = 0;

	if (vb->state_queued == STATE_QUEUED_TMR_QUEUED) {
		del_timer_sync(&dev->timer.wait_timer);
		vb->state_frame	 = STATE_FRAME_CANCELED;
		vb->state	 = VIDEOBUF_CANCELED;
		vb->state_queued = STATE_QUEUED_DONE;
		emxx_v4l2_next_proccess(dev, vb, 0, TMR_CALLBACK);

		if (down_trylock(&dev->sem_lcdout))
			;

		up(&dev->sem_lcdout); /* release semafore */
		trace_add(vb, "vbq_cancel:up sem_lcdout");
	} else {
		if (vb->state == VIDEOBUF_QUEUED)
			retval = 1;

		if (vb->state_queued == STATE_QUEUED_IDLE) {
			vb->state_queued = STATE_QUEUED_DONE;
			retval = 0;
		}
		if (dev->vb_old_refresh)
			retval = 1;
	}

	if (retval)
		trace_add(vb, "vbq_cancel(1)");
	else
		trace_add(vb, "vbq_cancel(0)");
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_complete
* FUNCTION :
* RETURN   : -
* NOTE	   : This function is called from interrupt context
*	   : when a transfer of a videobuf_buffer completes.
******************************************************************************/
static void emxx_v4l2_core_vbq_complete(void *arg1, void *arg, int err_flg)
{
	struct emxx_v4l2_device *dev = (struct emxx_v4l2_device *)arg1;
	struct videobuf_buffer	 *vb  = (struct videobuf_buffer *)arg;

	vb->field_count	  = dev->field_count;
	dev->field_count += 2;

	/* check next queue status */
	if (emxx_v4l2_next_proccess(dev, vb, 0, VBQ_COMPLETE)) {
		/* CANCELED */
		trace_add(vb, "vbq_complete: next_proccess canceled.\n");
		return;
	}

	if (err_flg) {
		trace_add(vb, "vbq_complete: callback error.\n");
		if (dev->vb_old_refresh) {
			vb->state = VIDEOBUF_CANCELED;
		} else {
			vb->state = VIDEOBUF_DQBUF_PERMIT;
			wake_up(&vb->done);
			wake_up(&vb->clear_done);
		}
	} else {
		trace_add(vb, "vbq_complete: callback success.\n");
		vb->state = VIDEOBUF_DONE;

		if (dev->vb_old) {
			if (dev->vb_old->state == VIDEOBUF_DONE
			 || dev->vb_old->state == VIDEOBUF_CANCELED)
				dev->vb_old->state = VIDEOBUF_DQBUF_PERMIT;

			printk_dbg(_VBQ_DBG, "(%p) ->prev(%p) ->next(%p) "
			 "->state(%d) ->sequence(%d) :old (%p) ->state(%d) "
			 "->done(%p)\n", &vb->stream, vb->stream.prev,
			 vb->stream.next, vb->state, vb->sequence,
			 &dev->vb_old->stream, dev->vb_old->state,
			 &dev->vb_old->done);

			emxx_v4l2_core_vbq_complete_skipped(
				&dev->vb_old->stream);
			wake_up(&dev->vb_old->done);
		} else {
			emxx_v4l2_core_vbq_complete_skipped(
				&((struct videobuf_queue *)
				(&dev->streaming->vbq))->stream);
		}
		dev->vb_old = vb;
		wake_up(&vb->clear_done);
	}
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_complete_skipped
* FUNCTION :
* RETURN   : -
* NOTE	   : This function is called from interrupt context
*	   : when a transfer of a videobuf_buffer completes.
******************************************************************************/
static inline void
emxx_v4l2_core_vbq_complete_skipped(struct list_head *vbq_stream)
{
	struct list_head       *list;
	struct videobuf_buffer *buf;

	list_for_each(list, vbq_stream) {
		buf = list_entry(list, struct videobuf_buffer, stream);
		if (buf->state == VIDEOBUF_DONE) {
			break;
		} else if (buf->state == VIDEOBUF_CANCELED) {
			if (buf->state_frame == STATE_FRAME_DONE
			 || buf->state_frame == STATE_FRAME_IMMEDIATE) {
				buf->state_frame = STATE_FRAME_CANCELED;
			}
			buf->state = VIDEOBUF_DQBUF_PERMIT;
			wake_up(&buf->done);
		}
		printk_dbg(_VBQ_DBG,
		 "(%p) ->prev(%p) ->next(%p) ->state(%d) ->sequence(%d)\n",
		 &buf->stream, buf->stream.prev, buf->stream.next, buf->state,
		 buf->sequence);
	}
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_queue
* FUNCTION :
* RETURN   : -
* NOTE	   : This function executes video buffer information.
******************************************************************************/
static void emxx_v4l2_core_vbq_queue(struct videobuf_queue *q,
 struct videobuf_buffer *vb)
{
	struct emxx_v4l2_device *dev = q->priv_data;

	if (dev->suspend == 0 && dev->mixing == 0 && dev->streamoff == NULL) {
		trace_add(vb, "vbq_queue");
		printk_dbg(_VBQ_DBG, "(%p) ->prev(%p) ->next(%p) ->state(%d)\n",
		 &vb->stream, vb->stream.prev, vb->stream.next, vb->state);

		vb->state = VIDEOBUF_QUEUED;
		emxx_v4l2_next_proccess(dev, vb, 0, VBQ_QUEUE);
	}
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_prepare
* FUNCTION :
* RETURN   :	   0: success
*	   : -EINVAL: error
* NOTE	   : This function registers information to video buffer.
******************************************************************************/
static int emxx_v4l2_core_vbq_prepare(struct videobuf_queue *q,
 struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct emxx_v4l2_device *dev = q->priv_data;
	unsigned long flags;
	unsigned int  retval = 0;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	if (((dev->output == V4L2_OUTPUT_LCD) ||
	     (dev->output == V4L2_OUTPUT_HDMI_1080I) ||
	     (dev->output == V4L2_OUTPUT_HDMI_720P))
	    && (dev->output != dev->output_lcd)) {
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		printk_info("error! V4L2 output mode and LCD output mode "
			    "must be the same.\n\n");
		return -EINVAL;
	}

	if (dev->pix.sizeimage > vb->bsize) {
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		return -EINVAL;
	}
	vb->field  = field;

	if (
	   /* YUV420 Semi-Planar */
	      (dev->pix.pixelformat != V4L2_PIX_FMT_NV12)
	   /* YUV422 Semi-Planar */
	   && (dev->pix.pixelformat != V4L2_PIX_FMT_NV422)
	   /* YUV420 Planar      */
	   && (dev->pix.pixelformat != V4L2_PIX_FMT_YUV420)
#if VF_YUV422PL
	   /* YUV422 Planar      */
	   && (dev->pix.pixelformat != V4L2_PIX_FMT_YUV422P)
#endif
#if VF_YUV422PX
	   /* YUV422 Interleave  */
	   && (dev->pix.pixelformat != V4L2_PIX_FMT_YUYV)
#endif
	) {
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		printk_info("error! pixelformat is incorrect.\n\n");
		return -EINVAL;
	}

	memcpy(&vb->save_effect, &dev->efct, sizeof(struct v4l2_effect));
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	vb->image_width  = dev->pix.width;
	vb->image_height = dev->pix.height;
#endif
	vb->pixelformat = dev->pix.pixelformat;
	vb->output	= dev->output;

	/* call size check function */
	if (emxx_v4l2_core_vbq_sizecheck(q, vb)) {
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		return -EINVAL;
	}

#ifdef CONFIG_VIDEO_EMXX_FILTER
	if (emxx_v4l2_chk_siz(dev, vb)) {
		/* call SIZ filter check function */
		if (emxx_v4l2_core_chkfilter(vb)) {
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			return -EINVAL;
		}
	}
#endif

	if (emxx_v4l2_core_wkb_chksize(dev, vb)) {
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		return -EINVAL;
	}

	printk_dbg(_VBQ_DBG,
	 "=== vbq_queue() ==============================================\n");
	printk_dbg(_VBQ_DBG,
	 " timestamp: sec(%ld), usec(%ld)\n", vb->ts.tv_sec, vb->ts.tv_usec);
	printk_dbg(_VBQ_DBG,
	 " src :      address Y(0x%08lx),  UV(0x%08lx),  V(0x%08lx)\n",
	 vb->base_addr.PhysAddr_Y, vb->base_addr.PhysAddr_UV,
	  vb->base_addr.PhysAddr_V);
	printk_dbg(_VBQ_DBG,
	 "                                 width(%4d), height(%4d), "
	 "pitch(%4d)\n", vb->width, vb->height, vb->bytesperline);
	printk_dbg(_VBQ_DBG,
	 " crop:      left(%4d), top(%4d), width(%4d), height(%4d)\n",
	 vb->save_effect.source_crop.top, vb->save_effect.source_crop.left,
	 vb->save_effect.source_crop.width, vb->save_effect.source_crop.height);
	printk_dbg(_VBQ_DBG,
	 "            angle(%ddeg)\n", vb->save_effect.movie_angle * 90);
	printk_dbg(_VBQ_DBG,
	 " screen:    left(%4d), top(%4d), width(%4d), height(%4d)\n",
	 vb->save_effect.screen.top, vb->save_effect.screen.left,
	 vb->save_effect.screen.width, vb->save_effect.screen.height);
	printk_dbg(_VBQ_DBG,
	 "==============================================================\n");

	q->sequence++;
	if (q->sequence == 0)
		q->sequence++;

	dev->efct.sequence = q->sequence;

	vb->sequence	= q->sequence;
	vb->state	= VIDEOBUF_PREPARED;

	if (vb->ts.tv_sec == STATE_FRAME_IMMEDIATE
	 || vb->ts.tv_usec == STATE_FRAME_IMMEDIATE) {
		vb->state_frame = STATE_FRAME_IMMEDIATE;
	} else {
		vb->state_frame = STATE_FRAME_DONE;
	}

	trace_add(vb, "vbq_prepare");

	spin_unlock_irqrestore(&dev->vbq_lock, flags);

	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_sizecheck_pix
* FUNCTION :
* RETURN   :  0: success
*	   : -1: error
* NOTE	   : This function registers information to video buffer.
******************************************************************************/
static int emxx_v4l2_core_vbq_sizecheck_pix(struct videobuf_buffer *vb)
{
	unsigned int image_width_align, image_height_align;
	int bpp_y;

	switch (vb->pixelformat) {
	default:
	case V4L2_PIX_FMT_NV12:	   /* YUV420 Semi-Planar */
	case V4L2_PIX_FMT_NV422:   /* YUV422 Semi-Planar */
		image_width_align  = IMAGE_WIDTH_ALIGN_SP;
		image_height_align = IMAGE_HEIGHT_ALIGN_SP;
		bpp_y = 8;
		break;
	case V4L2_PIX_FMT_YUV420:  /* YUV420 Planar      */
#if VF_YUV422PL
	case V4L2_PIX_FMT_YUV422P: /* YUV422 Planar      */
#endif
		image_width_align  = IMAGE_WIDTH_ALIGN_PL;
		image_height_align = IMAGE_HEIGHT_ALIGN_PL;
		bpp_y = 8;
		break;
#if VF_YUV422PX
	case V4L2_PIX_FMT_YUYV:	   /* YUV422 Interleave  */
		image_width_align  = IMAGE_WIDTH_ALIGN_IL;
		image_height_align = IMAGE_HEIGHT_ALIGN_IL;
		bpp_y = 16;
		break;
#endif
	}

	if ((vb->width < IMAGE_WIDTH_MIN) ||
	   (vb->width > IMAGE_WIDTH_MAX) ||
	   (vb->width & image_width_align) != 0) {
		printk_info("error! width(%4d) is incorrect.\n\n", vb->width);
		return -1;
	}
	if ((vb->height < IMAGE_HEIGHT_MIN) ||
	   (vb->height > IMAGE_HEIGHT_MAX) ||
	   (vb->height & image_height_align) != 0) {
		printk_info("error! height(%4d) is incorrect.\n\n", vb->height);
		return -1;
	}
	if ((vb->width * vb->height) > IMAGE_AREA_MAX) {
		printk_info("error! width*height(%4d)*(%4d) is incorrect.\n\n",
		 vb->width, vb->height);
		return -1;
	}
	if (((vb->bytesperline * 8 / bpp_y) < vb->width)     ||
	   ((vb->bytesperline  * 8 / bpp_y) > IMAGE_WIDTH_MAX) ||
	   ((vb->bytesperline * 8 / bpp_y) & image_width_align) != 0) {
		printk_info("error! bytesperline(%4d) is incorrect.\n\n",
		 vb->bytesperline);
		return -1;
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_sizecheck_crop
* FUNCTION :
* RETURN   :  0: success
*	   : -1: error
* NOTE	   : This function registers information to video buffer.
******************************************************************************/
static int emxx_v4l2_core_vbq_sizecheck_crop(struct videobuf_buffer *vb)
{
	unsigned int image_width_align, image_height_align, image_left_align;

	switch (vb->pixelformat) {
	default:
	case V4L2_PIX_FMT_NV12:	   /* YUV420 Semi-Planar */
	case V4L2_PIX_FMT_NV422:   /* YUV422 Semi-Planar */
		image_width_align  = IMAGE_WIDTH_ALIGN_SP;
		image_height_align = IMAGE_HEIGHT_ALIGN_SP;
		image_left_align   = IMAGE_LEFT_ALIGN_SP;
		break;
	case V4L2_PIX_FMT_YUV420:  /* YUV420 Planar      */
#if VF_YUV422PL
	case V4L2_PIX_FMT_YUV422P: /* YUV422 Planar      */
#endif
		image_width_align  = IMAGE_WIDTH_ALIGN_PL;
		image_height_align = IMAGE_HEIGHT_ALIGN_PL;
		image_left_align   = IMAGE_LEFT_ALIGN_PL;
		break;
#if VF_YUV422PX
	case V4L2_PIX_FMT_YUYV:	   /* YUV422 Interleave  */
		image_width_align  = IMAGE_WIDTH_ALIGN_IL;
		image_height_align = IMAGE_HEIGHT_ALIGN_IL;
		image_left_align   = IMAGE_LEFT_ALIGN_IL;
		break;
#endif
	}

	if ((vb->save_effect.source_crop.left < IMAGE_LEFT_MIN) ||
	   (vb->save_effect.source_crop.left > vb->width)      ||
	   (vb->save_effect.source_crop.left & image_left_align) != 0) {
		printk_info("error! source_crop.left(%4d) is incorrect.\n\n",
		 vb->save_effect.source_crop.left);
		return -1;
	}
	if ((vb->save_effect.source_crop.top < IMAGE_TOP_MIN) ||
	   (vb->save_effect.source_crop.top > vb->height)    ||
	   (vb->save_effect.source_crop.top & IMAGE_TOP_ALIGN) != 0) {
		printk_info("error! source_crop.top(%4d) is incorrect.\n\n",
		 vb->save_effect.source_crop.top);
		return -1;
	}

	if ((vb->save_effect.source_crop.width < IMAGE_WIDTH_MIN) ||
	   (vb->save_effect.source_crop.width > vb->width)	 ||
	   (vb->save_effect.source_crop.width & image_width_align) != 0) {
		printk_info("error! source_crop.width(%4d) is incorrect.\n\n",
		 vb->save_effect.source_crop.width);
		return -1;
	}

	if ((vb->save_effect.source_crop.height < IMAGE_HEIGHT_MIN) ||
	   (vb->save_effect.source_crop.height > vb->height)	   ||
	   (vb->save_effect.source_crop.height & image_height_align) != 0) {
		printk_info("error! source_crop.height(%4d) is incorrect.\n\n",
		 vb->save_effect.source_crop.height);
		return -1;
	}

	if ((vb->save_effect.source_crop.left +
		vb->save_effect.source_crop.width) > vb->width) {
		printk_info(
			"error! source_crop.left+width(%4d) is incorrect.\n\n",
			vb->save_effect.source_crop.left +
			vb->save_effect.source_crop.width);
		return -1;
	}
	if ((vb->save_effect.source_crop.top +
		vb->save_effect.source_crop.height) > vb->height) {
		printk_info(
			"error! source_crop.top+height(%4d) is incorrect.\n\n",
			vb->save_effect.source_crop.top +
			vb->save_effect.source_crop.height);
		return -1;
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_sizecheck
* FUNCTION :
* RETURN   :  0: success
*	   : -1: error
* NOTE	   : This function registers information to video buffer.
******************************************************************************/
static int emxx_v4l2_core_vbq_sizecheck(struct videobuf_queue *q,
 struct videobuf_buffer *vb)
{
	struct emxx_v4l2_device *dev = q->priv_data;
	unsigned int src_size, dst_size, dst_pos;
	unsigned int src_org_size_w, src_org_size_h;
	unsigned int image_width_align, image_height_align;
	unsigned int image_width_align_out, image_height_align_out;
	unsigned int front_width, front_height;
	unsigned long PhysAddr_Y, PhysAddr_UV, PhysAddr_V;
	unsigned int src_width, src_height;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	unsigned int src_add;
#endif

	switch (vb->pixelformat) {
	default:
	case V4L2_PIX_FMT_NV12:	   /* YUV420 Semi-Planar */
	case V4L2_PIX_FMT_NV422:   /* YUV422 Semi-Planar */
		image_width_align      = IMAGE_WIDTH_ALIGN_SP;
		image_height_align     = IMAGE_HEIGHT_ALIGN_SP;
		if (emxx_v4l2_chk_siz(dev, vb)) {
#ifdef CONFIG_EMXX_NTS
			if (vb->output == V4L2_OUTPUT_LCD ||
			    vb->output == V4L2_OUTPUT_HDMI_1080I ||
			    vb->output == V4L2_OUTPUT_HDMI_720P) {
#endif /* CONFIG_EMXX_NTS */
				/* SP -> IL */
				image_width_align_out  = IMAGE_WIDTH_ALIGN_IL;
				image_height_align_out = IMAGE_HEIGHT_ALIGN_IL;
#ifdef CONFIG_EMXX_NTS
			} else {
				/* SP -> SP */
				image_width_align_out  = IMAGE_WIDTH_ALIGN_SP;
				image_height_align_out = IMAGE_HEIGHT_ALIGN_SP;
			}
#endif /* CONFIG_EMXX_NTS */
		} else {
			image_width_align_out  = IMAGE_WIDTH_ALIGN_SP;
			image_height_align_out = IMAGE_HEIGHT_ALIGN_SP;
		}
		break;
	case V4L2_PIX_FMT_YUV420:  /* YUV420 Planar      */
#if VF_YUV422PL
	case V4L2_PIX_FMT_YUV422P: /* YUV422 Planar      */
#endif
		image_width_align      = IMAGE_WIDTH_ALIGN_PL;
		image_height_align     = IMAGE_HEIGHT_ALIGN_PL;
		if (emxx_v4l2_chk_siz(dev, vb)) {
#ifdef CONFIG_EMXX_NTS
			if (vb->output == V4L2_OUTPUT_LCD ||
			    vb->output == V4L2_OUTPUT_HDMI_1080I ||
			    vb->output == V4L2_OUTPUT_HDMI_720P) {
#endif /* CONFIG_EMXX_NTS */
				/* PL -> IL */
				image_width_align_out  = IMAGE_WIDTH_ALIGN_IL;
				image_height_align_out = IMAGE_HEIGHT_ALIGN_IL;
#ifdef CONFIG_EMXX_NTS
			} else {
				/* PL -> SP */
				image_width_align_out  = IMAGE_WIDTH_ALIGN_SP;
				image_height_align_out = IMAGE_HEIGHT_ALIGN_SP;
			}
#endif /* CONFIG_EMXX_NTS */
		} else {
			image_width_align_out  = IMAGE_WIDTH_ALIGN_PL;
			image_height_align_out = IMAGE_HEIGHT_ALIGN_PL;
		}
		break;
#if VF_YUV422PX
	case V4L2_PIX_FMT_YUYV:	   /* YUV422 Interleave  */
		image_width_align      = IMAGE_WIDTH_ALIGN_IL;
		image_height_align     = IMAGE_HEIGHT_ALIGN_IL;
		if (emxx_v4l2_chk_siz(dev, vb)) {
#ifdef CONFIG_EMXX_NTS
			if (vb->output == V4L2_OUTPUT_LCD ||
			    vb->output == V4L2_OUTPUT_HDMI_1080I ||
			    vb->output == V4L2_OUTPUT_HDMI_720P) {
#endif /* CONFIG_EMXX_NTS */
				image_width_align_out  = IMAGE_WIDTH_ALIGN_IL;
				image_height_align_out = IMAGE_HEIGHT_ALIGN_IL;
#ifdef CONFIG_EMXX_NTS
			} else {
				/* IL -> SP */
				image_width_align_out  = IMAGE_WIDTH_ALIGN_SP;
				image_height_align_out = IMAGE_HEIGHT_ALIGN_SP;
			}
#endif /* CONFIG_EMXX_NTS */
		} else {
			image_width_align_out  = IMAGE_WIDTH_ALIGN_IL;
			image_height_align_out = IMAGE_HEIGHT_ALIGN_IL;
		}
		break;
#endif
	}
	if (vb->save_effect.movie_angle == ROT2_MODE_MODE_90 ||
	    vb->save_effect.movie_angle == ROT2_MODE_MODE_270)
		image_height_align_out = image_width_align_out;

	switch (vb->output) {
	default:
	case V4L2_OUTPUT_LCD:
		front_width  = FRONT_WIDTH_LCD;
		front_height = FRONT_HEIGHT_LCD;
		break;
	case V4L2_OUTPUT_HDMI_1080I:
		front_width  = FRONT_WIDTH_1080I;
		front_height = FRONT_HEIGHT_1080I;
		break;
	case V4L2_OUTPUT_HDMI_720P:
		front_width  = FRONT_WIDTH_720P;
		front_height = FRONT_HEIGHT_720P;
		break;
#ifdef CONFIG_EMXX_NTS
	case V4L2_OUTPUT_NTSC:
		front_width  = NTSC_WIDTH;
		front_height = NTSC_HEIGHT;
		break;
	case V4L2_OUTPUT_PAL:
		front_width  = PAL_WIDTH;
		front_height = PAL_HEIGHT;
		break;
#endif
	}

#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	/* original image size check */
	if (vb->image_width > vb->save_effect.source_crop.width) {
		printk_info("error! pix.width (set by VIDIOC_S_FMT) (%d)"
		 " must be less or equal to crop.width (%d).\n\n",
		 vb->image_width, vb->save_effect.source_crop.width);
		return -1;
	}
	if (vb->image_height > vb->save_effect.source_crop.height) {
		printk_info("error! pix.height (set by VIDIOC_S_FMT) (%d)"
		 " must be less or equal to crop.height (%d).\n\n",
		 vb->image_height, vb->save_effect.source_crop.height);
		return -1;
	}
#endif

	/* pix size check */
	if (emxx_v4l2_core_vbq_sizecheck_pix(vb))
		return -1;

	/* crop size check */
	if (emxx_v4l2_core_vbq_sizecheck_crop(vb))
		return -1;

	/* angle check */
	if (vb->save_effect.movie_angle < ROT2_MODE_MODE_0
	 || vb->save_effect.movie_angle > ROT2_MODE_MODE_270) {
		printk_info("error! movie_angle(%d) is incorrect.\n\n",
		 vb->save_effect.movie_angle);
		return -1;
	}

	/* address check */

	if (vb->memory == V4L2_MEMORY_USERPTR) {
		printk_dbg(_MMAP_DBG, "vb->base_addr.PhysAddr_Y  = 0x%08lx\n",
		 vb->base_addr.PhysAddr_Y);
		printk_dbg(_MMAP_DBG, "vb->base_addr.PhysAddr_UV = 0x%08lx\n",
		 vb->base_addr.PhysAddr_UV);
		printk_dbg(_MMAP_DBG, "vb->base_addr.PhysAddr_V  = 0x%08lx\n",
		 vb->base_addr.PhysAddr_V);
		PhysAddr_Y  =
		 emxx_v4l2_virt_to_phys(vb->base_addr.PhysAddr_Y);
		PhysAddr_UV =
		 emxx_v4l2_virt_to_phys(vb->base_addr.PhysAddr_UV);
		PhysAddr_V  =
		 emxx_v4l2_virt_to_phys(vb->base_addr.PhysAddr_V);
	} else {
		PhysAddr_Y  = vb->base_addr.PhysAddr_Y;
		PhysAddr_UV = vb->base_addr.PhysAddr_UV;
		PhysAddr_V  = vb->base_addr.PhysAddr_V;
	}

	if ((PhysAddr_Y & IMAGE_PADDR_ALIGN) != 0
	 || PhysAddr_Y == IMAGE_PADDR_NULL) {
		printk_info("error! PhysAddrY(0x%lx) is incorrect.\n\n",
		 vb->base_addr.PhysAddr_Y);
		return -1;
	}
	if (((PhysAddr_UV & IMAGE_PADDR_ALIGN) != 0
	 || PhysAddr_UV == IMAGE_PADDR_NULL)
#if VF_YUV422PX
	   && (vb->pixelformat != V4L2_PIX_FMT_YUYV)	/* YUV422 Interleave */
#endif
	) {
		printk_info("error! PhysAddrUV(0x%lx) is incorrect.\n\n",
		 vb->base_addr.PhysAddr_UV);
		return -1;
	}
	if (((vb->pixelformat == V4L2_PIX_FMT_YUV420) /* YUV420 Planar */
#if VF_YUV422PL
	  || (vb->pixelformat == V4L2_PIX_FMT_YUV422P) /* YUV422 Planar */
#endif
	    ) && ((PhysAddr_V & IMAGE_PADDR_ALIGN) != 0
	       || PhysAddr_V == IMAGE_PADDR_NULL)) {
		printk_info("error! PhysAddrV(0x%lx) is incorrect.\n\n",
		 vb->base_addr.PhysAddr_V);
		return -1;
	}

	vb->base_addr.PhysAddr_Y  = PhysAddr_Y ;
	vb->base_addr.PhysAddr_UV = PhysAddr_UV;
	vb->base_addr.PhysAddr_V  = PhysAddr_V ;
	printk_dbg(_MMAP_DBG, "vb->base_addr.PhysAddr_Y  = 0x%08lx\n",
	 vb->base_addr.PhysAddr_Y);
	printk_dbg(_MMAP_DBG, "vb->base_addr.PhysAddr_UV = 0x%08lx\n",
	 vb->base_addr.PhysAddr_UV);
	printk_dbg(_MMAP_DBG, "vb->base_addr.PhysAddr_V  = 0x%08lx\n",
	 vb->base_addr.PhysAddr_V);

	/* dest size check */
	vb->save_effect.screen.width  -=
		vb->save_effect.screen.width  & image_width_align_out;
	vb->save_effect.screen.height -=
		vb->save_effect.screen.height & image_height_align_out;

	/* revise LCD output size */
	if (vb->save_effect.screen.left > front_width
	 || vb->save_effect.screen.top > front_height) {
		vb->save_effect.screen.width  = 0;
		vb->save_effect.screen.height = 0;
		return 0;
	}

	src_org_size_w = vb->save_effect.source_crop.left
	 + vb->save_effect.source_crop.width;
	src_org_size_h = vb->save_effect.source_crop.top
	 + vb->save_effect.source_crop.height;

	if (vb->save_effect.screen.left + vb->save_effect.screen.width >
		front_width) {
		dst_pos	 = vb->save_effect.screen.left;
		dst_size = vb->save_effect.screen.width;
		vb->save_effect.screen.width = (front_width - dst_pos)
		 & ~image_width_align_out;
		vb->save_effect.screen.left  =
			front_width - vb->save_effect.screen.width;
		if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
		 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
			src_add  = vb->save_effect.source_crop.width -
				   vb->image_width;
			src_size = vb->image_width;
			vb->image_width =
				vb->save_effect.screen.width *
				src_size / dst_size;
			vb->save_effect.source_crop.width =
				(vb->image_width + image_width_align) &
				~image_width_align;
			if (src_add >
				vb->save_effect.source_crop.width -
				vb->image_width)
				vb->save_effect.source_crop.width +=
					image_width_align + 1;
#else
			src_size = vb->save_effect.source_crop.width;
			vb->save_effect.source_crop.width  =
				vb->save_effect.screen.width *
				src_size / dst_size;
			vb->save_effect.source_crop.width -=
				(vb->save_effect.source_crop.width &
				image_width_align);
#endif
			if (vb->save_effect.movie_angle == ROT2_MODE_MODE_180)
				vb->save_effect.source_crop.left =
					src_org_size_w -
					vb->save_effect.source_crop.width;
		} else {
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
			src_add  = vb->save_effect.source_crop.height -
				   vb->image_height;
			src_size = vb->image_height;
			vb->image_height =
				vb->save_effect.screen.width *
				src_size / dst_size;
			vb->save_effect.source_crop.height =
				(vb->image_height + image_height_align) &
				~image_height_align;
			if (src_add >
				vb->save_effect.source_crop.height -
				vb->image_height)
				vb->save_effect.source_crop.height +=
					image_height_align + 1;
#else
			src_size = vb->save_effect.source_crop.height;
			vb->save_effect.source_crop.height  =
				vb->save_effect.screen.width *
				src_size / dst_size;
			vb->save_effect.source_crop.height -=
				(vb->save_effect.source_crop.height &
				image_height_align);
#endif
			if (vb->save_effect.movie_angle == ROT2_MODE_MODE_90)
				vb->save_effect.source_crop.top =
					src_org_size_h -
					vb->save_effect.source_crop.height;
		}
	}
	if (vb->save_effect.screen.top + vb->save_effect.screen.height >
		 front_height) {
		dst_pos	 = vb->save_effect.screen.top;
		dst_size = vb->save_effect.screen.height;
		vb->save_effect.screen.height = (front_height - dst_pos)
		 & ~image_height_align_out;
		vb->save_effect.screen.top = front_height
		 - vb->save_effect.screen.height;
		if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
		 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
			src_add  = vb->save_effect.source_crop.height -
				   vb->image_height;
			src_size = vb->image_height;
			vb->image_height =
				vb->save_effect.screen.height *
				src_size / dst_size;
			vb->save_effect.source_crop.height =
				(vb->image_height + image_height_align) &
				~image_height_align;
			if (src_add >
				vb->save_effect.source_crop.height -
				vb->image_height)
				vb->save_effect.source_crop.height +=
					image_height_align + 1;
#else
			src_size = vb->save_effect.source_crop.height;
			vb->save_effect.source_crop.height  =
				vb->save_effect.screen.height *
				src_size / dst_size;
			vb->save_effect.source_crop.height -=
				(vb->save_effect.source_crop.height &
				image_height_align);
#endif
			if (vb->save_effect.movie_angle == ROT2_MODE_MODE_180)
				vb->save_effect.source_crop.top =
					src_org_size_h -
					vb->save_effect.source_crop.height;
		} else {
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
			src_add  = vb->save_effect.source_crop.width -
				   vb->image_width;
			src_size = vb->image_width;
			vb->image_width =
				vb->save_effect.screen.height *
				src_size / dst_size;
			vb->save_effect.source_crop.width =
				(vb->image_width + image_width_align) &
				~image_width_align;
			if (src_add >
				vb->save_effect.source_crop.width -
				vb->image_width)
				vb->save_effect.source_crop.width +=
					image_width_align + 1;
#else
			src_size = vb->save_effect.source_crop.width;
			vb->save_effect.source_crop.width  =
				vb->save_effect.screen.height *
				src_size / dst_size;
			vb->save_effect.source_crop.width -=
				(vb->save_effect.source_crop.width &
				image_width_align);
#endif
			if (vb->save_effect.movie_angle == ROT2_MODE_MODE_270)
				vb->save_effect.source_crop.left =
					src_org_size_w -
					vb->save_effect.source_crop.width;
		}
	}

	/* IMG not support dest size check */
	if (vb->save_effect.screen.width < DEST_WIDTH_MIN
	 || vb->save_effect.screen.height < DEST_HEIGHT_MIN) {
		vb->save_effect.screen.width  = 0;
		vb->save_effect.screen.height = 0;
	} else if (vb->save_effect.source_crop.height < IMAGE_HEIGHT_MIN
	 || vb->save_effect.source_crop.width < IMAGE_WIDTH_MIN) {
		printk_info(
			"error! source_crop.width(%4d) or "
			"source_crop.height(%4d) is incorrect.\n\n",
			vb->save_effect.source_crop.width,
			vb->save_effect.source_crop.height);
		return -1;
	}

#if RESIZE_CHECK
	/* resize scaling check */

#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	src_width  = vb->image_width;
	src_height = vb->image_height;
#else
	src_width  = vb->save_effect.source_crop.width;
	src_height = vb->save_effect.source_crop.height;
#endif

	if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
	 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
		if (((RESIZE_WIDTH_MIN * vb->save_effect.screen.width) <
			src_width) ||
			(vb->save_effect.screen.width >
			(RESIZE_WIDTH_MAX *
			src_width))) {
			printk_info(
				"error! source_crop.width(%4d) or "
				"screen.width(%4d) is incorrect.\n",
				vb->save_effect.source_crop.width,
				vb->save_effect.screen.width);
			return -1;
		}
		if (((RESIZE_HEIGHT_MIN * vb->save_effect.screen.height) <
			src_height) ||
			(vb->save_effect.screen.height >
			(RESIZE_HEIGHT_MAX *
			src_height))) {
			printk_info(
				"error! source_crop.height(%4d) or "
				"screen.height(%4d) is incorrect.\n",
				vb->save_effect.source_crop.height,
				vb->save_effect.screen.height);
			return -1;
		}
	} else {
		if (((RESIZE_WIDTH_MIN * vb->save_effect.screen.width) <
			src_height) ||
			(vb->save_effect.screen.width >
			(RESIZE_WIDTH_MAX *
			src_height))) {
			printk_info(
				"error! source_crop.height(%4d) or "
				"screen.width(%4d) is incorrect.\n",
				vb->save_effect.source_crop.height,
				vb->save_effect.screen.width);
			return -1;
		}
		if (((RESIZE_HEIGHT_MIN * vb->save_effect.screen.height) <
			src_width) ||
		    (vb->save_effect.screen.height >
		    (RESIZE_HEIGHT_MAX *
		    src_width))) {
			printk_info(
				"error! source_crop.width(%4d) or "
				"screen.height(%4d) is incorrect.\n",
				vb->save_effect.source_crop.width,
				vb->save_effect.screen.height);
			return -1;
		}
	}
#endif

	return 0;
}


#ifdef CONFIG_VIDEO_EMXX_FILTER
static int emxx_v4l2_core_chkfilter(struct videobuf_buffer *vb)
{
	struct v4l2_filter_option *filter = &vb->filter;
	int hstep, src_width, dst_width;
	int max_4x4 = 8190;
	int max_2x2 = 8190;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	int image_width;
#endif

	switch (filter->filter_option) {
	case SIZ_FILTOPT_MODE_4x4FILT_MINUS:
	case SIZ_FILTOPT_MODE_4x4FILT_PLUS:
	case SIZ_FILTOPT_MODE_2x2FILT_WITH_BUFFER:
	case SIZ_FILTOPT_MODE_2x2FILT:
	case SIZ_FILTER_DEFAULT:
	case SIZ_FILTER_SMOOTHING:
		break;
	default:
		printk_info("error! filter_option(%ld) is incorrect.\n\n",
		 filter->filter_option);
		return -EINVAL;
		break;
	}

	if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
	 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
		src_width = vb->save_effect.source_crop.width;
		dst_width = vb->save_effect.screen.width;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		image_width = vb->image_width;
#endif
	} else {
		src_width = vb->save_effect.source_crop.width;
		dst_width = vb->save_effect.screen.height;
#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
		image_width = vb->image_width;
#endif
	}

#ifdef CONFIG_VIDEO_EMXX_IMAGESIZE
	hstep = 256*image_width/dst_width;
#else
	hstep = 256*src_width/dst_width;
#endif

	/* size: MIN. MAX. */
	if (hstep < 256) {
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
			max_4x4 = 1024; max_2x2 = 2048;
		} else {
#endif
			max_4x4 = 2048; max_2x2 = 4096;
#ifdef CONFIG_MACH_EMEV
		}
#endif
	} else if (hstep < 256 * 4) {
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
			max_4x4 = 1024; max_2x2 = 2048;
		} else {
#endif
			max_4x4 = 2048; max_2x2 = 4096;
#ifdef CONFIG_MACH_EMEV
		}
#endif
	} else if (hstep < 256 * 8) {
		max_4x4 = 4096; max_2x2 = 8190;
	} else if (hstep < 256 * 16) {
		max_4x4 = 8190; max_2x2 = 8190;
	} else {
		max_4x4 = 8190; max_2x2 = 8190;
	}

	if (src_width > max_2x2) {
		if (vb->save_effect.movie_angle == ROT2_MODE_MODE_0
		 || vb->save_effect.movie_angle == ROT2_MODE_MODE_180) {
			printk_info("error! source_crop.width(%4d) is too big."
			 "\n\n", src_width);
		} else {
			printk_info("error! source_crop.width(%4d) is too big."
			 "\n\n", src_width);
		}
		return -EINVAL;
	}

	if ((src_width > max_4x4) &&
	    ((filter->filter_option == SIZ_FILTOPT_MODE_4x4FILT_MINUS) ||
	     (filter->filter_option == SIZ_FILTOPT_MODE_4x4FILT_PLUS))) {
		printk_info("error! filter_option(%ld) is incorrect.\n",
		 filter->filter_option);
		printk_info("filter_option must be 2x2 filter.\n\n");
		return -EINVAL;
	}

	return 0;
}
#endif


/*****************************************************************************
* MODULE   : emxx_v4l2_core_vbq_setup
* FUNCTION :
* RETURN   : 0: success
* NOTE	   : Limit the number of available kernel image capture buffers based
*	   : on the number requested, the currently selected image size,
*	   : and the maximum amount of memory permitted for kernel buffers.
******************************************************************************/
static int emxx_v4l2_core_vbq_setup(struct videobuf_queue *q,
 unsigned int *cnt, unsigned int *size)
{
	int retval = 0;
	struct emxx_v4l2_device *dev = q->priv_data;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME; /* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	*size = dev->pix.sizeimage;

	if (dev->pix.sizeimage <= 0)
		retval = -1;

	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/* ------------------ work_buffer_ops -------------------------------------- */
/*****************************************************************************
* MODULE   : emxx_v4l2_core_wkb_init
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static inline void emxx_v4l2_core_wkb_init(struct emxx_v4l2_device *dev)
{
	if (dev->workbuf.active == WORKBUF_NO_INIT) { /* not init */
		dev->workbuf.rot_addr = V4L2_ROT_BUFFER_ADDR;
		dev->workbuf.rot_size = V4L2_ROT_BUFFER_SIZE;
		dev->workbuf.active   = WORKBUF_INIT;
	}
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_wkb_active
* FUNCTION :
* RETURN   :	   0: success
*	   : -EINVAL: misstype argument
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_core_wkb_active(struct emxx_v4l2_device *dev,
 unsigned int cmd, void *arg)
{
	emxx_v4l2_core_wkb_init(dev);

	if (cmd == VIDIOC_G_FMT || cmd == VIDIOC_G_CROP
	 || cmd == VIDIOC_G_CTRL || cmd == VIDIOC_G_EFFECT) {
		/* ioctl(VIDIOC_G_FMT) || ioctl(VIDIOC_G_CROP)
		 || ioctl(VIDIOC_G_CTRL) || ioctl(VIDIOC_G_EFFECT) */
		;
	} else if (cmd == VIDIOC_S_FMT) {
		/* ioctl(VIDIOC_S_FMT) */
		if (((struct v4l2_format *)arg)->type
		 == V4L2_BUF_TYPE_PRIVATE) {
			if (dev->workbuf.active == WORKBUF_ACTIVE) {
				printk_info(
					"error! this ioctl(VIDIOC_S_FMT) "
					"is invalid.\n");
				return -EPERM;
			}
	} else{
			/* other type */
			dev->workbuf.active = WORKBUF_ACTIVE;
		}
	} else{
		/* other ioctl */
		dev->workbuf.active = WORKBUF_ACTIVE;
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_wkb_chkbuf
* FUNCTION :
* RETURN   :	   0: success
*	   : -EINVAL: misstype argument
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_core_wkb_chkbuf(struct v4l2_workbuffer *wbuf)
{
	/* check workbuffer size */
	if (wbuf->rot_size < 0)
		return -EINVAL;

	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_wkb_chksize
* FUNCTION :
* RETURN   :	   0: success
*	   : -EINVAL: misstype argument
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_QBUF
******************************************************************************/
static int emxx_v4l2_core_wkb_chksize(struct emxx_v4l2_device *dev,
 struct videobuf_buffer *vb)
{
	int bpp = 0;

	if (vb->pixelformat == V4L2_PIX_FMT_NV12
	 || vb->pixelformat == V4L2_PIX_FMT_YUV420) {
		bpp = 12;	/* YUV420 Semi-Planar || YUV420 Planar */
	} else if (vb->pixelformat == V4L2_PIX_FMT_NV422
#if VF_YUV422PL
		|| vb->pixelformat == V4L2_PIX_FMT_YUV422P
#endif
#if VF_YUV422PX
		|| vb->pixelformat == V4L2_PIX_FMT_YUYV
#endif
	) {
		bpp = 16;	/* YUV422 Semi-Planar || YUV422 Planar */
	} else {
		return -EINVAL;
	}

	/* check use ROT */
	if (
#ifdef CONFIG_EMXX_NTS
	    (vb->output == V4L2_OUTPUT_LCD ||
	     vb->output == V4L2_OUTPUT_HDMI_1080I ||
	     vb->output == V4L2_OUTPUT_HDMI_720P) &&
#endif /* CONFIG_EMXX_NTS */
	    (vb->save_effect.movie_angle || emxx_v4l2_chk_siz(dev, vb))) {
		if ((vb->save_effect.screen.width *
			vb->save_effect.screen.height * bpp / 8) >
			((dev->workbuf.rot_size / NUM_BUF_V4L2) & ~0x3)) {
			printk_info(
				"error! WorkBuffer(ROT) is insufficient.  "
				">>> screen(%d*%d =0x%x) > workbuf(0x%x)\n\n",
				vb->save_effect.screen.width,
				vb->save_effect.screen.height,
				(vb->save_effect.screen.width *
				vb->save_effect.screen.height * bpp / 8),
				((dev->workbuf.rot_size / NUM_BUF_V4L2) &
				~0x3));
			return -EINVAL;
		}
	}

	return 0;
}


/* ------------------ system_call_ops -------------------------------------- */
/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_g_output
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_G_OUTPUT
******************************************************************************/
static inline int emxx_v4l2_ioc_g_output(struct emxx_v4l2_fh *fh,
 unsigned int *output)
{
	*output = ((struct emxx_v4l2_device *)fh->dev)->output;
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_s_output
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype *output
*	   :	-EIO: error
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_S_OUTPUT
******************************************************************************/
static int emxx_v4l2_ioc_s_output(struct emxx_v4l2_fh *fh,
 unsigned int *output)
{
	struct emxx_v4l2_device *dev = fh->dev;
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	switch (*output) {
	case V4L2_OUTPUT_LCD:
	case V4L2_OUTPUT_HDMI_1080I:
	case V4L2_OUTPUT_HDMI_720P:
		if (*output != dev->output_lcd) {
			retval = -EIO;
		} else {
			if (dev->output != *output) {
				spin_unlock_irqrestore(&dev->vbq_lock, flags);
				if (dev->streaming) {
					emxx_v4l2_ioc_streamoff(fh, dev);
#ifdef CONFIG_EMXX_NTS
					if ((dev->output == V4L2_OUTPUT_NTSC) ||
					    (dev->output == V4L2_OUTPUT_PAL))
						emxx_nts_release(
						 NTS_ACTIVE_V4L2);
#endif /* CONFIG_EMXX_NTS */
					dev->output = *output;
					emxx_v4l2_ioc_streamon(fh, dev);
#ifdef CONFIG_EMXX_NTS
				} else {
					if ((dev->output == V4L2_OUTPUT_NTSC) ||
					    (dev->output == V4L2_OUTPUT_PAL))
						emxx_nts_release(
						 NTS_ACTIVE_V4L2);
#endif /* CONFIG_EMXX_NTS */
				}
				spin_lock_irqsave(&dev->vbq_lock, flags);
			}
			dev->output = *output;
		}
		break;
#ifdef CONFIG_EMXX_NTS
	case V4L2_OUTPUT_NTSC:
	case V4L2_OUTPUT_PAL:
		retval = emxx_nts_getmode();
		if (retval != V4L2_OUTPUT_LCD && retval != *output) {
			/* NTS reserved in other mode. */
			retval = -EIO;
		} else {
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			if (dev->streaming) {
				emxx_v4l2_ioc_streamoff(fh, dev);
				retval = emxx_nts_reserve(NTS_ACTIVE_V4L2,
				 *output);
				emxx_v4l2_ioc_streamon(fh, dev);
			} else {
				retval = emxx_nts_reserve(NTS_ACTIVE_V4L2,
				 *output);
			}
			spin_lock_irqsave(&dev->vbq_lock, flags);
			if (retval) {
				retval = -EIO;
			} else {
				/* success NTS reserve */
				dev->output = *output;
			}
		}
		break;
#endif /* CONFIG_EMXX_NTS */
	default:
		retval = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_g_fmt
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype fmt->type
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_G_FMT
******************************************************************************/
static int emxx_v4l2_ioc_g_fmt(struct emxx_v4l2_device *dev,
 struct v4l2_format *fmt)
{
	struct v4l2_workbuffer *wbuf;
	int retval = 0;
	unsigned long flags;

	wbuf = (struct v4l2_workbuffer *)fmt->fmt.raw_data;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		/* get the current format */
		fmt->fmt.pix = dev->pix;
	} else if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		/* get overlay info */
		fmt->fmt.win.w = dev->efct.screen;
	} else if (fmt->type == V4L2_BUF_TYPE_PRIVATE) {
		/* get workbuffer info */
		memcpy(wbuf, &dev->workbuf, sizeof(struct v4l2_workbuffer));
	} else {
		retval = -EINVAL;
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_s_fmt
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype fmt->type
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_S_FMT
******************************************************************************/
static int emxx_v4l2_ioc_s_fmt(struct emxx_v4l2_device *dev,
 struct v4l2_format *fmt)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		/* set image info */
		dev->pix.sizeimage    = sizeof(struct videobuf_buffer);
		dev->pix.width	      = fmt->fmt.pix.width;
		dev->pix.height	      = fmt->fmt.pix.height;
		dev->pix.pixelformat  = fmt->fmt.pix.pixelformat;
		dev->pix.bytesperline = fmt->fmt.pix.bytesperline;
		dev->pix.field	      = fmt->fmt.pix.field;
	} else if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		if (dev->efct.source_crop.width &&
			dev->efct.source_crop.height) {
			if ((dev->efct.source_crop.width !=
				fmt->fmt.win.w.width ||
				dev->efct.source_crop.height !=
				fmt->fmt.win.w.height) &&
				(!dev->workbuf.rot_addr ||
				!dev->workbuf.rot_size)) {
				retval = -ENOMEM;
				goto err_ret;
			}
		}
		/* set overlay info */
		dev->efct.screen.left	= fmt->fmt.win.w.left;
		dev->efct.screen.top	= fmt->fmt.win.w.top;
		dev->efct.screen.width	= fmt->fmt.win.w.width;
		dev->efct.screen.height = fmt->fmt.win.w.height;
	} else if (fmt->type == V4L2_BUF_TYPE_PRIVATE) {
		struct v4l2_workbuffer *wbuf;

		wbuf = (struct v4l2_workbuffer *)fmt->fmt.raw_data;
		if (emxx_v4l2_core_wkb_chkbuf(wbuf)) {
			retval = -EINVAL;
			goto err_ret;
		}
		/* set workbuffer info */
		dev->workbuf.rot_addr = wbuf->rot_addr;
		dev->workbuf.rot_size = wbuf->rot_size;
	} else {
		retval = -EINVAL;
	}
err_ret:
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_g_crop
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype vc->type
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_G_CROP
******************************************************************************/
static int emxx_v4l2_ioc_g_crop(struct emxx_v4l2_device *dev,
 struct v4l2_crop *vc)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (vc->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		/* get crop data of original movie */
		vc->c.left   = dev->efct.source_crop.left;
		vc->c.top    = dev->efct.source_crop.top;
		vc->c.width  = dev->efct.source_crop.width;
		vc->c.height = dev->efct.source_crop.height;
	} else {
		retval = -EINVAL;
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_s_crop
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype vc->type
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_S_CROP
******************************************************************************/
static int emxx_v4l2_ioc_s_crop(struct emxx_v4l2_device *dev,
 struct v4l2_crop *vc)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (vc->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		if (dev->efct.screen.width && dev->efct.screen.height) {
			if ((dev->efct.screen.width != vc->c.width
			 || dev->efct.screen.height != vc->c.height)
			  && (!dev->workbuf.rot_addr
			   || !dev->workbuf.rot_size)) {
				retval = -ENOMEM;
				goto err_ret;
			}
		}
		/* set crop data of original movie */
		dev->efct.source_crop.left   = vc->c.left;
		dev->efct.source_crop.top    = vc->c.top;
		dev->efct.source_crop.width  = vc->c.width;
		dev->efct.source_crop.height = vc->c.height;
	} else {
		retval = -EINVAL;
	}
err_ret:
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_g_ctrl
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype vc->id
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_G_CTRL
******************************************************************************/
static int emxx_v4l2_ioc_g_ctrl(struct emxx_v4l2_device *dev,
 struct v4l2_control *vc)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (vc->id == V4L2_MOVIE_DISPLAY_ANGLE) {
		/* get current rotation angle */
		vc->value = dev->efct.movie_angle;
	} else {
		retval = -EINVAL;
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_s_ctrl
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   : -EINVAL: misstype vc->id
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_S_CTRL
******************************************************************************/
static int emxx_v4l2_ioc_s_ctrl(struct emxx_v4l2_device *dev,
 struct v4l2_control *vc)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (vc->id == V4L2_MOVIE_DISPLAY_ANGLE) {
		if (vc->value != ROT2_MODE_MODE_0
		 && (!dev->workbuf.rot_addr || !dev->workbuf.rot_size)) {
			retval = -ENOMEM;
		} else {
			/* set rotation angle */
			dev->efct.movie_angle = vc->value;
		}
	} else {
		retval = -EINVAL;
	}
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_g_effect
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_G_EFFECT
******************************************************************************/
static int emxx_v4l2_ioc_g_effect(struct emxx_v4l2_device *dev,
 struct v4l2_effect *vef)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	/* get current effect infomation */
	vef->sequence		= dev->efct.sequence;
	vef->source_crop.left	= dev->efct.source_crop.left;
	vef->source_crop.top	= dev->efct.source_crop.top;
	vef->source_crop.width	= dev->efct.source_crop.width;
	vef->source_crop.height = dev->efct.source_crop.height;
	vef->movie_angle	= dev->efct.movie_angle;
	vef->screen.left	= dev->efct.screen.left;
	vef->screen.top		= dev->efct.screen.top;
	vef->screen.width	= dev->efct.screen.width;
	vef->screen.height	= dev->efct.screen.height;
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_s_effect
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_S_EFFECT
******************************************************************************/
static int emxx_v4l2_ioc_s_effect(struct emxx_v4l2_device *dev,
 struct v4l2_effect *vef)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

	if ((vef->screen.width != vef->source_crop.width
	 || vef->screen.height != vef->source_crop.height)
	  && (!dev->workbuf.rot_addr || !dev->workbuf.rot_size)) {
		retval = -ENOMEM;
		goto err_ret;
	}
	if (vef->movie_angle != ROT2_MODE_MODE_0
	 && (!dev->workbuf.rot_addr || !dev->workbuf.rot_size)) {
		retval = -ENOMEM;
		goto err_ret;
	}

	/* set effect infomation */
	dev->efct.sequence	     = 0;
	dev->efct.source_crop.left   = vef->source_crop.left;
	dev->efct.source_crop.top    = vef->source_crop.top;
	dev->efct.source_crop.width  = vef->source_crop.width;
	dev->efct.source_crop.height = vef->source_crop.height;
	dev->efct.movie_angle	     = vef->movie_angle;
	dev->efct.screen.left	     = vef->screen.left;
	dev->efct.screen.top	     = vef->screen.top;
	dev->efct.screen.width	     = vef->screen.width;
	dev->efct.screen.height	     = vef->screen.height;

err_ret:
	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_reqbufs
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_REQBUFS
******************************************************************************/
static inline int emxx_v4l2_ioc_reqbufs(struct emxx_v4l2_device *dev,
 struct videobuf_queue *vbq, struct v4l2_requestbuffers *req)
{
	/* request memory for queue */
	return videobuf_reqbufs(vbq, req);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_qbuf
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_QBUF
******************************************************************************/
static inline int emxx_v4l2_ioc_qbuf(struct emxx_v4l2_device *dev,
 struct videobuf_queue *vbq, struct v4l2_buffer *vb)
{
	/* enqueue videoframe-buffer */
	return videobuf_qbuf(vbq, vb);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_dqbuf
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_DQBUF
******************************************************************************/
static inline int emxx_v4l2_ioc_dqbuf(struct emxx_v4l2_device *dev,
 struct videobuf_queue *vbq, struct v4l2_buffer *vb, int f_flags)
{
	/* dequeue videoframe-buffer */
	return videobuf_dqbuf(vbq, vb, f_flags & O_NONBLOCK);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_streamon
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   :  -EBUSY:
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_STREAMON
******************************************************************************/
static int emxx_v4l2_ioc_streamon(struct emxx_v4l2_fh *fh,
 struct emxx_v4l2_device *dev)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (dev->streaming) {
		retval = -EBUSY;
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
	} else {
		dev->streaming = fh;
		dev->vb_old_refresh = NULL;
		dev->vb_old = NULL;
		dev->mixing = 0;

		/* init thread info */
		emxx_v4l2_thread_init(dev);

		/* init timer */
		emxx_v4l2_tmr_init(dev);
		spin_unlock_irqrestore(&dev->vbq_lock, flags);

		retval = videobuf_streamon(&fh->vbq);
	}
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_ioc_streamoff
* FUNCTION : sub function of emxx_v4l2_core_do_ioctl()
* RETURN   :	   0: success
*	   :	 err:
* NOTE	   : this function is substance of the ioctl system call.
*	   :	VIDIOC_STREAMOFF
******************************************************************************/
static int emxx_v4l2_ioc_streamoff(struct emxx_v4l2_fh *fh,
 struct emxx_v4l2_device *dev)
{
	int retval = 0;
	unsigned long flags;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
#if _PERF2_DBG
	Perf2Deinit(dev);
#endif

	dev->streamoff = fh;
	spin_unlock_irqrestore(&dev->vbq_lock, flags);

	/* streamoff */
	retval = videobuf_streamoff(&fh->vbq);
	if (retval < 0) {
		dbg_lock(&dev->vbq_lock, "vbq_lock\n");
		spin_lock_irqsave(&dev->vbq_lock, flags);
		dev->streamoff = NULL;
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
	} else {
		dbg_lock(&dev->vbq_lock, "vbq_lock\n");
		spin_lock_irqsave(&dev->vbq_lock, flags);

		/* init thread info */
		emxx_v4l2_thread_init(dev);
		wake_up_interruptible(&dev->th_rot.th_busy);
		wake_up_interruptible(&dev->th_rot.th_buf->buf_busy);
		wake_up_interruptible(&dev->th_tmr.th_busy);
		wake_up_interruptible(&dev->th_lcd.th_busy);

		if (dev->streaming == fh) {
			/* stop timer */
			del_timer_sync(&dev->timer.wait_timer);

			/* call lcd driver function. */
			spin_unlock_irqrestore(&dev->vbq_lock, flags);
			emxx_v4l2_lcd_request(dev, NULL, STREAM_STOP);
			spin_lock_irqsave(&dev->vbq_lock, flags);

			dev->streaming = NULL;
			dev->vb_old_refresh = NULL;
			dev->vb_old    = NULL;
			dev->mixing    = 0;
		}
		/* video-buffer-queue sequence number initialize */
		fh->vbq.sequence = 0;
		dev->streamoff	 = NULL;

		spin_unlock_irqrestore(&dev->vbq_lock, flags);
	}
	return retval;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_do_ioctl
* FUNCTION :
* RETURN   :	   0: success
*	   : -EINVAL: Argument error
* NOTE	   : this function is substance of the ioctl system call.
*	   : @@ supported ioctl command
*	   :	VIDIOC_S_FMT,	    VIDIOC_G_FMT,	VIDIOC_S_CROP,
*	   :	VIDIOC_G_CROP,	    VIDIOC_S_CTRL,	VIDIOC_G_CTRL,
*	   :	VIDIOC_REQBUFS,	    VIDIOC_S_EFFECT,	VIDIOC_G_EFFECT,
*	   :	VIDIOC_STREAMON,    VIDIOC_STREAMOFF,	VIDIOC_QBUF,
*	   :	VIDIOC_DQBUF
*	   : @@ not supported ioctl command
*	   :	VIDIOC_ENUMINPUT,   VIDIOC_G_INPUT,	VIDIOC_S_INPUT,
*	   :	VIDIOC_ENUM_FMT,    VIDIOC_TRY_FMT,	VIDIOC_QUERYCTRL,
*	   :	VIDIOC_G_FBUF,	    VIDIOC_S_FBUF,	VIDIOC_OVERLAY,
*	   :	VIDIOC_ENUMSTD,	    VIDIOC_G_STD,	VIDIOC_S_STD,
*	   :	VIDIOC_QUERYSTD,    VIDIOC_G_AUDIO,	VIDIOC_S_AUDIO
*	   :	VIDIOC_G_AUDOUT,    VIDIOC_S_AUDOUT,	VIDIOC_G_JPEGCOMP,
*	   :	VIDIOC_S_JPEGCOMP,  VIDIOC_G_TUNER,	VIDIOC_S_TUNER,
*	   :	VIDIOC_G_MODULATOR, VIDIOC_S_MODULATOR, VIDIOC_G_FREQUENCY,
*	   :	VIDIOC_S_FREQUENCY, VIDIOC_QUERYCAP,	VIDIOC_ENUMOUTPUT,
*	   :	VIDIOC_G_OUTPUT,    VIDIOC_S_OUTPUT,	VIDIOC_QUERYBUF
******************************************************************************/
static long emxx_v4l2_core_do_ioctl(struct file *file, unsigned int cmd,
 void *arg)
{
	struct emxx_v4l2_fh	 *fh  = file->private_data;
	struct emxx_v4l2_device *dev = fh->dev;


	/* check ROT workbuffer */
	if (emxx_v4l2_core_wkb_active(dev, cmd, arg))
		return -EPERM;

	switch (cmd) {
	case VIDIOC_G_OUTPUT:
		return emxx_v4l2_ioc_g_output(fh, arg);

	case VIDIOC_S_OUTPUT:
		return emxx_v4l2_ioc_s_output(fh, arg);

	case VIDIOC_G_FMT:
		return emxx_v4l2_ioc_g_fmt(dev, arg);

	case VIDIOC_S_FMT:
		return emxx_v4l2_ioc_s_fmt(dev, arg);

	case VIDIOC_G_CROP:
		return emxx_v4l2_ioc_g_crop(dev, arg);

	case VIDIOC_S_CROP:
		return emxx_v4l2_ioc_s_crop(dev, arg);

	case VIDIOC_G_CTRL:
		return emxx_v4l2_ioc_g_ctrl(dev, arg);

	case VIDIOC_S_CTRL:
		return emxx_v4l2_ioc_s_ctrl(dev, arg);

	case VIDIOC_G_EFFECT:
		return emxx_v4l2_ioc_g_effect(dev, arg);

	case VIDIOC_S_EFFECT:
		return emxx_v4l2_ioc_s_effect(dev, arg);

	case VIDIOC_REQBUFS:
		return emxx_v4l2_ioc_reqbufs(dev, &fh->vbq, arg);

	case VIDIOC_QBUF:
		return emxx_v4l2_ioc_qbuf(dev, &fh->vbq, arg);

	case VIDIOC_DQBUF:
		return emxx_v4l2_ioc_dqbuf(dev, &fh->vbq, arg, file->f_flags);

	case VIDIOC_STREAMON:
		return emxx_v4l2_ioc_streamon(fh, dev);

	case VIDIOC_STREAMOFF:
		return emxx_v4l2_ioc_streamoff(fh, dev);

	/*--- not supported ---*/
	case VIDIOC_QUERYBUF:
		/* not supported */
		return -EINVAL;

	case VIDIOC_ENUMINPUT:	/* FALL THROUGH */
	case VIDIOC_G_INPUT:	/* FALL THROUGH */
	case VIDIOC_S_INPUT:	/* FALL THROUGH */
	case VIDIOC_ENUM_FMT:	/* FALL THROUGH */
	case VIDIOC_TRY_FMT:	/* FALL THROUGH */
	case VIDIOC_QUERYCTRL:
		/* not supported */
		return -EINVAL;

	case VIDIOC_QUERYCAP:	/* FALL THROUGH */
	/* FALL THROUGH Get the frame buffer parameters */
	case VIDIOC_G_FBUF:
	/* FALL THROUGH set the frame buffer parameters */
	case VIDIOC_S_FBUF:
	case VIDIOC_OVERLAY:
		/* not supported */
		return -EINVAL;

	case VIDIOC_ENUMSTD:	/* FALL THROUGH */
	case VIDIOC_G_STD:	/* FALL THROUGH */
	case VIDIOC_S_STD:	/* FALL THROUGH */
	case VIDIOC_QUERYSTD:
		/* we don't have an analog video standard,
		 * so we don't need to implement these ioctls.
		 */
		 return -EINVAL;

	case VIDIOC_G_AUDIO:	/* FALL THROUGH */
	case VIDIOC_S_AUDIO:	/* FALL THROUGH */
	case VIDIOC_G_AUDOUT:	/* FALL THROUGH */
	case VIDIOC_S_AUDOUT:
		/* we don't have any audio inputs or outputs */
		return -EINVAL;

	case VIDIOC_G_JPEGCOMP:	/* FALL THROUGH */
	case VIDIOC_S_JPEGCOMP:
		/* JPEG compression is not supported */
		return -EINVAL;

	case VIDIOC_G_TUNER:		/* FALL THROUGH */
	case VIDIOC_S_TUNER:		/* FALL THROUGH */
	case VIDIOC_G_MODULATOR:	/* FALL THROUGH */
	case VIDIOC_S_MODULATOR:	/* FALL THROUGH */
	case VIDIOC_G_FREQUENCY:	/* FALL THROUGH */
	case VIDIOC_S_FREQUENCY:
		/* we don't have a tuner or modulator */
		return -EINVAL;

	case VIDIOC_ENUMOUTPUT:
		/* not supported */
		return -EINVAL;

	default:
		/* unrecognized ioctl */
		return -ENOIOCTLCMD;
	}
	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_ioctl
* FUNCTION : a stub function of ioctl
* RETURN   :	   0: success
*	   : -EINVAL: Argument error
* NOTE	   :
******************************************************************************/
static long  emxx_v4l2_core_ioctl(struct file *file, unsigned int cmd,
 unsigned long arg)
{
	return (int)(video_usercopy(file, cmd, arg, emxx_v4l2_core_do_ioctl));
}

/*****************************************************************************
* MODULE   : emxx_v4l2_core_poll
* FUNCTION : a system call function of poll & select
* RETURN   :	   0: success
*	   : -EINVAL: Argument error
* NOTE	   :
******************************************************************************/
static inline unsigned int emxx_v4l2_core_poll(struct file *file,
 struct poll_table_struct *wait)
{
	struct emxx_v4l2_fh *fh = file->private_data;

	return videobuf_poll_stream(file, &fh->vbq, wait);
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_release
* FUNCTION : v4l2 driver's close processing function.
* RETURN   : 0: success
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_core_release(struct file *file)
{
	struct emxx_v4l2_fh     *fh  = file->private_data;
	struct emxx_v4l2_device *dev = fh->dev;
	unsigned long flags;
	int err;

	emxx_v4l2_ioc_streamoff(fh, dev);

	if (fh->vbq.read_buf) {
		emxx_v4l2_core_vbq_release(&fh->vbq, fh->vbq.read_buf);
		kfree(fh->vbq.read_buf);
	}

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);

#ifdef CONFIG_EMXX_NTS
	if ((dev->output == V4L2_OUTPUT_NTSC) ||
	    (dev->output == V4L2_OUTPUT_PAL)) {
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		err = emxx_nts_release(NTS_ACTIVE_V4L2);
		spin_lock_irqsave(&dev->vbq_lock, flags);
		if (err)
			printk_err("NTS out disable failed.\n\n");
	}
#endif
	dev->output = V4L2_OUTPUT_LCD;

	spin_unlock_irqrestore(&dev->vbq_lock, flags);
	err = videobuf_mmap_free(&fh->vbq);
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (err)
		printk_err("videobuf_free failed.\n\n");

	file->private_data = NULL;
	kfree(fh->vbq.int_ops);
	kfree(fh);
	spin_unlock_irqrestore(&dev->vbq_lock, flags);

	/* workqueue uninitialize */
	destroy_workqueue(dev->v4l2_workqueue);

	dev->vb_old_refresh = NULL;
	dev->vb_old = NULL;
	memset(&dev->pix,     0, sizeof(dev->pix));
	memset(&dev->efct,    0, sizeof(dev->efct));
	memset(&dev->workbuf, 0, sizeof(dev->workbuf));

	emxx_free_siz(dev->siz_info.id);
	emxx_free_rot(dev->rot_info.id);

	/* v4l2 unactive */
	dev->active = 0;

#ifdef CONFIG_EMXX_ANDROID
	v4l2_open_flag = 0;
	wake_up(&v4l2_close_q);
#endif

	return 0;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_open
* FUNCTION : v4l2 driver's open processing function.
* RETURN   :	   0: success
*	   : -ENODEV: corresponding device doesn't exist.
*	   : -ENOMEM: memory that can be used for the kernel is insufficient.
*	   : -EPERM : no privilege in the call origin.
* NOTE	   :
******************************************************************************/
static int emxx_v4l2_core_open(struct file *file)
{
	int minor = video_devdata(file)->minor;
	struct emxx_v4l2_device *dev = emxx_dev;
	struct emxx_v4l2_fh	 *fh;
	struct videobuf_qtype_ops  *int_ops;
	unsigned long flags;

	if (!dev || !dev->vfd || (dev->vfd->minor != minor))
		return -ENODEV;

#if (_PERF1_DBG | _TRACE_DBG)
	if (dev->active == 1) {
#if _PERF1_DBG
		perf_out();
#endif /* _PERF1_DBG */
#if _TRACE_DBG
		fh = NULL;
		if (dev->streaming)
			fh = dev->streaming;
		else if (dev->streamoff)
			fh = dev->streamoff;

		if (fh) {
			dbg_vbqtrace(dev, &fh->vbq);
			dbg_backtrace(dev);
		}
		return -EPERM;
#endif /* _TRACE_DBG */
	}
#endif /* _PERF1_DBG | _TRACE_DBG */

	/* allocate per-filehandle data */
	fh = kmalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		emxx_free_siz(dev->siz_info.id);
		emxx_free_rot(dev->rot_info.id);
		return -ENOMEM;
	}

	int_ops = kmalloc(sizeof(*int_ops), GFP_KERNEL);
	if (NULL == int_ops) {
		emxx_free_siz(dev->siz_info.id);
		emxx_free_rot(dev->rot_info.id);
		kfree(fh);
		return -ENOMEM;
	}

	file->private_data = fh;
	fh->dev = dev;
	fh->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	int_ops->magic = MAGIC_QTYPE_OPS;
	fh->vbq.int_ops = int_ops;

	dbg_lock(&dev->vbq_lock, "vbq_lock\n");
	spin_lock_irqsave(&dev->vbq_lock, flags);
	if (dev->active == 1) {
		printk_err("v4l2 device Active\n");
		spin_unlock_irqrestore(&dev->vbq_lock, flags);
		kfree(fh);
		kfree(int_ops);
		return -EPERM;
	}
	dev->active = 1;
	spin_unlock_irqrestore(&dev->vbq_lock, flags);

	dev->rot_info.device  = DEV_LCD;
	dev->rot_info.timeout = 1000;
	if (emxx_request_rot(ROT_LCH0, &dev->rot_info)) {
		printk_err("could not request ROT\n");
		dev->active = 0;
		kfree(fh);
		kfree(int_ops);
		return -EBUSY;
	}
	dev->siz_info.device  = DEV_LCD;
	dev->siz_info.timeout = 1000;
	if (emxx_request_siz(&dev->siz_info)) {
		printk_err("could not request SIZ\n");
		emxx_free_rot(dev->rot_info.id);
		dev->active = 0;
		kfree(fh);
		kfree(int_ops);
		return -EBUSY;
	}

	videobuf_queue_core_init(&fh->vbq, &dev->vbq_ops, NULL, &dev->vbq_lock,
	 fh->type, V4L2_FIELD_NONE, sizeof(struct videobuf_buffer), fh->dev,
	 fh->vbq.int_ops);

	/* workqueue initialize */
	dev->v4l2_workqueue = create_singlethread_workqueue("v4l2");
#if ENABLE_DELAY
	INIT_WORK(&dev->wk_tmr_callback_bottom,
	 emxx_v4l2_tmr_callback_bottom_do);
#endif

	/* video-buffer-queue index & sequence number initialize */
	fh->vbq.index_max = 0;
	fh->vbq.sequence  = 0;

	trace_clr();
	perf_clr();

#ifdef CONFIG_EMXX_ANDROID
	v4l2_open_flag = 1;
#endif

	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
/*****************************************************************************
* MODULE   : emxx_v4l2_core_suspend
* FUNCTION : callback function when to suspend.
* RETURN   :
* NOTE	   : unsupport
******************************************************************************/
static int  emxx_v4l2_core_suspend(struct platform_device *pdev,
 pm_message_t state)
{
	int ret;
	struct emxx_v4l2_device *dev = dev_get_drvdata(&pdev->dev);

	if (dev->streaming)
		ret = -EBUSY;
	else {
		dev->suspend = 1;
		ret = 0;
	}
	return ret;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_resume
* FUNCTION : callback function when to resume.
* RETURN   :
* NOTE	   : unsupport
******************************************************************************/
static int  emxx_v4l2_core_resume(struct platform_device *pdev)
{
	struct emxx_v4l2_device *dev = dev_get_drvdata(&pdev->dev);

	if (dev->suspend)
		dev->suspend = 0;
	return 0;
}
#endif /* CONFIG_PM || CONFIG_DPM */


/*****************************************************************************
* MODULE   : emxx_v4l2_core_cleanup
* FUNCTION : cleanup module
* RETURN   :	   0: success
*	   : -ENODEV: corresponding device doesn't exist.
* NOTE	   : v4l2 driver cleanup function, when to driver ends.
******************************************************************************/
void emxx_v4l2_core_cleanup(void)
{
	struct emxx_v4l2_device *dev = emxx_dev;
	struct video_device	 *vfd;

	if (!dev)
		return;

	vfd = dev->vfd;
	if (vfd) {
		if (vfd->minor == -1) {
			/* The device never got registered, so release the
			** video_device struct directly
			*/
			video_device_release(vfd);
		} else {
			/* The unregister function will release the video_device
			** struct as well as unregistering it.
			*/
			video_unregister_device(vfd);
			platform_driver_unregister(&emxx_v4l2_core_driver);
			platform_device_unregister(&emxx_v4l2_core_device);
		}
		dev->vfd = NULL;
	}

	/* uninitialize kernel thread */
	emxx_v4l2_thread_cleanup(dev);

	kfree(dev);
	emxx_dev = NULL;

	return;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_core_init
* FUNCTION : initialize module
* RETURN   :	   0: success
*	   : -ENODEV: corresponding device doesn't exist.
* NOTE	   : v4l2 driver initialize function, when to driver starts.
******************************************************************************/
int __init emxx_v4l2_core_init(void)
{
	struct emxx_v4l2_device *dev;
	struct video_device	 *vfd;

	dev = kmalloc(sizeof(struct emxx_v4l2_device), GFP_KERNEL);
	if (!dev) {
		printk_err("could not allocate memory\n");
		goto init_error;
	}
	memset(dev, 0, sizeof(struct emxx_v4l2_device));

	/* Save the pointer to device in a global variable */
	emxx_dev = dev;

	/* initialize the video_device struct */
	vfd = dev->vfd = video_device_alloc();
	if (!vfd) {
		printk_err("could not allocate video device struct\n");
		goto init_error;
	}

	vfd->release = video_device_release;

	strlcpy(vfd->name, DEV_NAME, sizeof(vfd->name));

	vfd->fops     = &emxx_v4l2_core_fops;
	video_set_drvdata(vfd, dev);
	vfd->minor    = -1;

	/* initialize the semafore */
	sema_init(&dev->sem_lcdout, 1);

	/* initialize the spinlock used to image parameters */
	spin_lock_init(&dev->vbq_lock);

	emxx_v4l2_core_wkb_init(dev);

	/* initialize the kernel thread */
	if (emxx_v4l2_thread_startup(dev)) {
		printk_err("failed create thread\n");
		goto init_error;
	}

	/* initialize the kernel timer */
	emxx_v4l2_tmr_init(dev);

	/* initialize the videobuf queue ops */
	dev->vbq_ops.buf_setup	 = emxx_v4l2_core_vbq_setup;
	dev->vbq_ops.buf_prepare = emxx_v4l2_core_vbq_prepare;
	dev->vbq_ops.buf_queue	 = emxx_v4l2_core_vbq_queue;
	dev->vbq_ops.buf_release = emxx_v4l2_core_vbq_release;
	dev->vbq_ops.buf_cancel	 = emxx_v4l2_core_vbq_cancel;
	dev->vbq_ops.buf_dqueue	 = emxx_v4l2_core_vbq_dqueue;

	dev_set_drvdata(&emxx_v4l2_core_device.dev, (void *)dev);
	if (platform_device_register(&emxx_v4l2_core_device) < 0) {
		printk_err("could not register platform_device\n");
		goto init_error;
	}

	if (platform_driver_register(&emxx_v4l2_core_driver) < 0) {
		printk_err("could not register driver\n");
		platform_device_unregister(&emxx_v4l2_core_device);
		goto init_error;
	}
	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		printk_err("could not register Video for Linux device\n");
		platform_device_unregister(&emxx_v4l2_core_device);
		platform_driver_unregister(&emxx_v4l2_core_driver);
		goto init_error;
	}

	printk(KERN_INFO DEV_NAME ": registered device video%d [v4l2]\n",
	 vfd->minor);
	return 0;

init_error:
	emxx_v4l2_core_cleanup();
	return -ENODEV;
}


/*****************************************************************************
* MODULE   : emxx_v4l2_vma_open
* FUNCTION : open VMA
* RETURN   : none
* NOTE     : none
******************************************************************************/
void emxx_v4l2_vma_open(struct vm_area_struct *vma)
{
	int i;
	unsigned long offset;

	printk_dbg(_MMAP_DBG, "call in.\n");
	printk_dbg(_MMAP_DBG, "vma->vm_start: 0x%lx\n", vma->vm_start);
	printk_dbg(_MMAP_DBG, "vma->vm_end  : 0x%lx\n", vma->vm_end);
	printk_dbg(_MMAP_DBG, "vma->vm_pgoff: 0x%lx\n", vma->vm_pgoff);

	offset = vma->vm_pgoff << PAGE_SHIFT;

	emxx_dev->num_v4l2_mmap++;

	for (i = 0; i < NUM_V4L2_MMAP; i++) {
		if (emxx_dev->v4l2_mmap[i].vm_start == 0)
			break;
	}
	if (i < NUM_V4L2_MMAP) {
		emxx_dev->v4l2_mmap[i].vm_start = vma->vm_start;
		emxx_dev->v4l2_mmap[i].vm_end   = vma->vm_end;
		emxx_dev->v4l2_mmap[i].offset   = offset;
	}
}


/*****************************************************************************
* MODULE   : emxx_v4l2_vma_close
* FUNCTION : close VMA
* RETURN   : none
* NOTE     : none
******************************************************************************/
void emxx_v4l2_vma_close(struct vm_area_struct *vma)
{
	int i;

	printk_dbg(_MMAP_DBG, "call in.\n");
	printk_dbg(_MMAP_DBG, "vma->vm_start: 0x%lx\n", vma->vm_start);
	printk_dbg(_MMAP_DBG, "vma->vm_end  : 0x%lx\n", vma->vm_end);
	printk_dbg(_MMAP_DBG, "vma->vm_pgoff: 0x%lx\n", vma->vm_pgoff);

	emxx_dev->num_v4l2_mmap--;

	for (i = 0; i < NUM_V4L2_MMAP; i++) {
		if (emxx_dev->v4l2_mmap[i].vm_start == vma->vm_start)
			break;
	}
	if (i < NUM_V4L2_MMAP) {
		emxx_dev->v4l2_mmap[i].vm_start = 0;
		emxx_dev->v4l2_mmap[i].vm_end   = 0;
		emxx_dev->v4l2_mmap[i].offset   = 0;
	}
}


struct vm_operations_struct emxx_v4l2_vm_ops = {
	.open	= emxx_v4l2_vma_open,
	.close	= emxx_v4l2_vma_close,
};


/*****************************************************************************
* MODULE   : emxx_v4l2_mmap
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
int emxx_v4l2_mmap(struct vm_area_struct *vma)
{
	printk_dbg(_MMAP_DBG, "call in.\n");
	printk_dbg(_MMAP_DBG, "vma->vm_start: 0x%lx\n", vma->vm_start);
	printk_dbg(_MMAP_DBG, "vma->vm_end  : 0x%lx\n", vma->vm_end);
	printk_dbg(_MMAP_DBG, "vma->vm_pgoff: 0x%lx\n", vma->vm_pgoff);

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	if (emxx_dev->num_v4l2_mmap >= NUM_V4L2_MMAP)
		return -EAGAIN;

	/* Accessing memory will be done non-cached. */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	/* To stop the swapper from even considering these pages */
	vma->vm_flags |= (VM_IO | VM_RESERVED);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	vma->vm_ops = &emxx_v4l2_vm_ops;
	emxx_v4l2_vma_open(vma);

	printk_dbg(_MMAP_DBG, "success.\n");
	return 0;
}
EXPORT_SYMBOL(emxx_v4l2_mmap);


/*****************************************************************************
* MODULE   : emxx_v4l2_virt_to_phys
* FUNCTION :
* RETURN   :
* NOTE	   :
******************************************************************************/
static unsigned long emxx_v4l2_virt_to_phys(unsigned long addr)
{
	int i;
	unsigned long offset;

	printk_dbg(_MMAP_DBG, "call in.\n");

	for (i = 0; i < NUM_V4L2_MMAP; i++) {
		if ((addr >= emxx_dev->v4l2_mmap[i].vm_start) &&
		    (addr <  emxx_dev->v4l2_mmap[i].vm_end))
			break;
	}
	if (i < NUM_V4L2_MMAP) {
		offset = emxx_dev->v4l2_mmap[i].offset;
		offset += addr - emxx_dev->v4l2_mmap[i].vm_start;
		return offset;
	} else {
		return 0;
	}
}


MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("EMMA Mobile EV Video for Linux2 driver");
MODULE_LICENSE("GPL");
module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr,
 "Minor number for video device (-1 ==> auto assign)");

module_init(emxx_v4l2_core_init);
module_exit(emxx_v4l2_core_cleanup);


