/*
 *  File Name       : emxx_cam.c
 *  Function        : CAMERA I/F Driver
 *  Release Version : Ver 1.02
 *  Release Date    : 2011/01/25
 *
 *  Copyright (C) Renesas Electronics Corporation 2011
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY;
 *  without even the implied warrnty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program;
 *  If not, write to the Free Software Foundation, Inc., 59 Temple Place -
 *  Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#ifdef CONFIG_EMXX_ANDROID
#include <linux/wakelock.h>
#endif /* CONFIG_EMXX_ANDROID */
#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/emxx_mem.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <linux/freezer.h>
#include <mach/pm.h>
#include <mach/pwc.h>
#endif /* CONFIG_PM || CONFIG_DPM */
#ifdef CONFIG_VIDEO_EMXX
#include <mach/emxx_v4l2.h>
#endif /* CONFIG_VIDEO_EMXX */

#define EMXX_CAM_MAKING_DEBUG

#include "emxx_cam.h"

#define CAM_NAME "emxx_camera"
#if 1
#define EMXX_CAM_MAX_BUFNBRS 4
#else
#define EMXX_CAM_MAX_BUFNBRS VIDEO_MAX_FRAME
#endif

/* #define CAM_FPS_DEBUG */
#define EMXX_CAM_USE_MMAP    1

#define DEV_NAME "emxx_cam"

#ifdef CAM_FPS_DEBUG
#define ZERO_VALUE 0
unsigned int transmit_end_cnt = ZERO_VALUE;
unsigned int vsync_detect_cnt = ZERO_VALUE;
unsigned int deq_done_push_cnt = ZERO_VALUE;
unsigned int deq_done_pull_cnt = ZERO_VALUE;
unsigned int grab_push_cnt = ZERO_VALUE;
unsigned int grab_pull_cnt = ZERO_VALUE;

unsigned int dma_run_cnt = ZERO_VALUE;
#endif

/*** DEBUG code by the making ->*/
#ifdef EMXX_CAM_MAKING_DEBUG

int debug = 4;

#include <linux/moduleparam.h>

#define FNC_ENTRY	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "entry:%s\n", __func__); \
	}

#define FNC_EXIT_N	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "exit: %s:%d\n", __func__, __LINE__); \
	}

#define FNC_EXIT(r)	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "exit:%d :%s:%d\n", r, __func__, __LINE__); \
	}

#define d0b(fmt, args...)	\
	{ \
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d1b(fmt, args...)	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d2b(fmt, args...)	\
	if (debug == 2 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d3b(fmt, args...)	\
	if (debug == 3 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d4b(fmt, args...)	\
	if (debug == 4 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d5b(fmt, args...)	\
	if (debug == 5 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d6b(fmt, args...)	\
	if (debug == 6 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}

#else
#define FNC_ENTRY do { } while (0);
#define FNC_EXIT_N  do { } while (0);
#define FNC_EXIT(r) do { } while (0);
#define d0b(fmt, args...) do { } while (0);
#define d1b(fmt, args...) do { } while (0);
#define d2b(fmt, args...) do { } while (0);
#define d3b(fmt, args...) do { } while (0);
#define d4b(fmt, args...) do { } while (0);
#define d5b(fmt, args...) do { } while (0);
#define d6b(fmt, args...) do { } while (0);
#endif

enum {
	CAM_MMAP = 0,
	CAM_READ,
};

enum {
	CAM_OFF = 0,
	CAM_ON,
};

#define CAM_IPU_OFF  CAM_OFF
#define CAM_IPU_ON   CAM_ON

enum {
	BUF_FIXED = 0,
	BUF_IPU,
};

enum {
	CAM_BUF_IDLE = 0,
	CAM_BUF_QUEUED,
	CAM_BUF_GRABBING,
	CAM_BUF_DONE,
	CAM_BUF_BREAK,
};

struct emxx_cam_mapping {
	unsigned int count;
	unsigned long start;
	unsigned long end;
#ifdef CONFIG_VIDEO_EMXX
	struct vm_operations_struct *vm_ops;
#endif
};

struct emxx_cam_buffer {
	__u32 index;
	__u32 state;
	__u32 bytesused;
	struct timeval timestamp;
	unsigned long sequence;
	char *vadr;             /* points to actual buffer */
	unsigned long padr;     /* physical buffer address */
	union {
		__u32 offset;
		unsigned long userptr;
	} m;
	struct emxx_cam_mapping *map;
};

struct emxx_cam_fmt {
	__u8 description[32];      /* Description string */
	__u32 pixelformat;         /* Format fourcc      */
	int depth;                 /* bit/pixel          */
	int flags;
	int boundary;
};

struct emxx_cam_frames {
	struct kfifo *enq;
	spinlock_t enq_lock;
	struct emxx_cam_buffer *buff;
	wait_queue_head_t proc_list;
	unsigned int cnt;
	unsigned int max;
	int (*update)(void *);
	__u32 blocksize;
	enum v4l2_memory memory;
	const struct emxx_cam_fmt *fmt;
	int buf_type;
};

/* CAMIF status */
#define B_MAINOR (0x01 << 3)
#define B_MAINTC (0x01 << 2)
#define B_DMAERR (0x01 << 1)
#define B_CAMVS  (0x01 << 0)

struct emxx_cam {
	__u32 status;

	__u32 setup:1;
	__u32 reset:1;
	__u32 stop:1;
	__u32 action:1; /* 0:mmap 1:read */
	__u32 reading:1;
	__u32 streaming:1;
	__u32 frames_active:1;
	__u32 userptr:1;
	unsigned int mapping;

	__u32 ipu:1;

	struct mutex lock;

	struct kfifo *deq_done;
	spinlock_t deq_done_lock;
	unsigned long sequence;

#ifdef CONFIG_EMXX_ANDROID
	struct wake_lock idle_lock; /* suspend control */
#endif /* CONFIG_EMXX_ANDROID */

	spinlock_t cam_lock;

	struct task_struct *th;

	__u32 width;              /* Image width in pixels. */
	__u32 height;             /* Image height in pixels. */
	struct v4l2_rect c;       /* Cropping rectangle */
	struct v4l2_rect bounds;  /* Defines the window within capturing */
	const struct emxx_cam_fmt *fmt;
	struct mutex frames_lock;
	struct emxx_cam_frames *grab;
	struct emxx_cam_prepare pre;

	struct emxx_cam_hw_operations hw;
	struct video_device *vdev;
	struct proc_dir_entry *proc_entry;
	int open_count;
	__u32 active_number;
	__u32 used_number;
};

struct emxx_cam *em_cam;
#if 1 /* XXX */
static int warming_up = 1;
#endif


struct emxx_cam_private {
	int number;
};

/*
 * Camera I/F Functions
 */

struct emxx_camif {
	__u32 status;
	struct mutex lock;

	__u32 update:1;

	__u8 mirror;
	__u8 bngr;
	__u8 cbgr;
	__u8 crgr;
	__s8 bnzr;
	__s8 cbzr;
	__s8 crzr;
};

struct emxx_camif *camif;

#define CA_STATUS        IO_ADDRESS(EMXX_CAM_BASE + 0x0000)
#define CA_RAWSTATUS     IO_ADDRESS(EMXX_CAM_BASE + 0x0004)
#define CA_ENSET         IO_ADDRESS(EMXX_CAM_BASE + 0x0008)
#define CA_ENCLR         IO_ADDRESS(EMXX_CAM_BASE + 0x000C)
#define CA_FFCLR         IO_ADDRESS(EMXX_CAM_BASE + 0x0010)
#define CA_ERRORADR      IO_ADDRESS(EMXX_CAM_BASE + 0x0014)
#define CA_CSR           IO_ADDRESS(EMXX_CAM_BASE + 0x0020)
#define CA_X1R           IO_ADDRESS(EMXX_CAM_BASE + 0x0030)
#define CA_X2R           IO_ADDRESS(EMXX_CAM_BASE + 0x0034)
#define CA_Y1R           IO_ADDRESS(EMXX_CAM_BASE + 0x0038)
#define CA_Y2R           IO_ADDRESS(EMXX_CAM_BASE + 0x003C)
#define CA_BNZR          IO_ADDRESS(EMXX_CAM_BASE + 0x0040)
#define CA_BNGR          IO_ADDRESS(EMXX_CAM_BASE + 0x0044)
#define CA_CBZR          IO_ADDRESS(EMXX_CAM_BASE + 0x0048)
#define CA_CBGR          IO_ADDRESS(EMXX_CAM_BASE + 0x004C)
#define CA_CRZR          IO_ADDRESS(EMXX_CAM_BASE + 0x0050)
#define CA_CRGR          IO_ADDRESS(EMXX_CAM_BASE + 0x0054)
#define CA_DMACNT        IO_ADDRESS(EMXX_CAM_BASE + 0x0080)
#define CA_FRAME         IO_ADDRESS(EMXX_CAM_BASE + 0x0084)
#define CA_DMAREQ        IO_ADDRESS(EMXX_CAM_BASE + 0x0088)
#define CA_DMASTOP       IO_ADDRESS(EMXX_CAM_BASE + 0x008C)
#define CA_LINESIZE_MAIN IO_ADDRESS(EMXX_CAM_BASE + 0x0100)
#define CA_XRATIO_MAIN   IO_ADDRESS(EMXX_CAM_BASE + 0x0104)
#define CA_YRATIO_MAIN   IO_ADDRESS(EMXX_CAM_BASE + 0x0108)
#define CA_DMAX_MAIN     IO_ADDRESS(EMXX_CAM_BASE + 0x010C)
#define CA_DMAY_MAIN     IO_ADDRESS(EMXX_CAM_BASE + 0x0110)
#define CA_YPLANE_A      IO_ADDRESS(EMXX_CAM_BASE + 0x0114)
#define CA_UVPLANE_A     IO_ADDRESS(EMXX_CAM_BASE + 0x0118)
#define CA_VPLANE_A      IO_ADDRESS(EMXX_CAM_BASE + 0x0244)
#define CA_YPLANE_B      IO_ADDRESS(EMXX_CAM_BASE + 0x011C)
#define CA_UVPLANE_B     IO_ADDRESS(EMXX_CAM_BASE + 0x0120)
#define CA_VPLANE_B      IO_ADDRESS(EMXX_CAM_BASE + 0x0248)
#define CA_MODULECONT    IO_ADDRESS(EMXX_CAM_BASE + 0x022C)
#define CA_UPDATE        IO_ADDRESS(EMXX_CAM_BASE + 0x0230)
#define CA_MIRROR        IO_ADDRESS(EMXX_CAM_BASE + 0x0234)
#define CA_OD_BYTELANE   IO_ADDRESS(EMXX_CAM_BASE + 0x0238)
#define CA_X3R           IO_ADDRESS(EMXX_CAM_BASE + 0x0240)
#define CA_OD_BYTELANE2  IO_ADDRESS(EMXX_CAM_BASE + 0x0254)
#define CA_QOS           IO_ADDRESS(EMXX_CAM_BASE + 0x0258)

/* #define M_CA_ENSET (B_MAINOR | B_MAINTC | B_DMAERR | B_CAMVS) */
#define M_CA_ENSET (B_MAINOR | B_MAINTC)

#define S_656MODE       14
#define S_PIXEL_YUV     13
#define S_SYNCTYPE      12
#define S_PIXELMODE     11
#define S_DATA_OD       10
#define S_DATA_ID       9
#define S_LD_TMG        8
#define S_VS_DET        7
#define S_HS_DET        6
#define S_LIMITSEL      5
#define S_SYNCMODE      4
#define S_CLK_EDGE      3
#define S_DATA_DET      2
#define S_VS_POL        1
#define S_HS_POL        0

#define S_SUBYUV        13
#define S_MAINYUV       12
#define S_SUBREC        10
#define S_MAINREC       8
#define S_SUBMODE       6
#define S_MAINMODE      4
#define S_SBRESIZE      3
#define S_MNRESIZE      2
#define S_PCULLR        0

#define S_SUBFRM        2
#define S_MAINFRM       0

struct camif_reg {
	__u32 csr;
	__u32 x1r;
	__u32 x2r;
	__u32 x3r;
	__u32 y1r;
	__u32 y2r;
	__u32 dmacnt;
	__u32 od_bytelane;
	__u32 od_bytelane2;
	__u32 yplane_a;
	__u32 uvplane_a;
	__u32 vplane_a;
	__u32 frame;
	__u32 dmax_main;
	__u32 dmay_main;
	__u32 linesize_main;
	__u32 xratio_main;
	__u32 yratio_main;
};



/*!
 * start CAM module power supply
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */

#define P2_POWER_WAIT 20000000

#define P2_SEQ_BUSY 0x80

#define CAM_POWERDOWN 1
#define CAM_RETENTION	0

#define P2_SWON		0x1
#define P2_PDON		0x100

static int camif_power_on(int mode)
{

	int pv_seq;
	int count = P2_POWER_WAIT;
	FNC_ENTRY;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & P2_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		err("power busy flag\n");
		return -1;
	}
	if (mode == CAM_POWERDOWN) {
		writel(readl(SMU_P2_SWON) | P2_SWON | P2_PDON,
		       SMU_P2_SWON);
	} else {
		writel((readl(SMU_P2_SWON) | P2_SWON) & ~P2_PDON,
		       SMU_P2_SWON);
	}
	count = P2_POWER_WAIT;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & P2_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		err("power busy flag\n");
		return -1;
	}

	mdelay(100);
	FNC_EXIT_N;
	return 0;
}

/*!
 * stop CAM module power supply
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int camif_power_off(int mode)
{

	int pv_seq;
	int count = P2_POWER_WAIT;
	FNC_ENTRY;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & P2_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		err("power busy flag\n");
		return -1;
	}

	if (mode == CAM_POWERDOWN) {
		writel((readl(SMU_P2_SWON) | P2_PDON) & ~P2_SWON,
			SMU_P2_SWON);
	} else {
		writel(readl(SMU_P2_SWON) & ~(P2_SWON | P2_PDON),
		       SMU_P2_SWON);
	}
	count = P2_POWER_WAIT;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & P2_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		err("power busy flag\n");
		return -1;
	}

	FNC_EXIT_N;
	return 0;

}


static inline int camif_chg_pinsel(void)
{

	/* Pin select
	 * GIO_P131 : CAM_CLKO
	 * GIO_P132 : CAM_CLKI
	 * GIO_P133 : CAM_VS
	 * GIO_P134 : CAM_HS
	 * GIO_P135~142 : CAM_YUV0~YUV7
	 */
	FNC_ENTRY;
	/* unreset CAM module*/
	camif_power_on(CAM_POWERDOWN);

	emxx_unreset_device(EMXX_RST_CAM);
	emxx_unreset_device(EMXX_RST_CAM_SAFE);

	/* GIO_P131~P142 switch to extend function pin(not GPIO)*/
	writel((readl(CHG_PINSEL_G128) & ~0x00007FF8), CHG_PINSEL_G128);
	/* chose mode0->CAM function */
	writel(0x00000000, CHG_PINSEL_CAM);

	/*input enable, PU/PD setting*/
	/*all input enable and PU/PD disable, except for CLKO*/
	writel((readl(CHG_PULL18) & ~0xffff0000) | 0x44400000, CHG_PULL18);
	writel(0x44444444, CHG_PULL19);

	FNC_EXIT_N;
	return 0;
}

static inline int camif_start_clock(void)
{
	FNC_ENTRY;
	/* start clock */
	emxx_clkctrl_off(EMXX_CLKCTRL_CAMPCLK);
	emxx_open_clockgate(EMXX_CLK_CAM | EMXX_CLK_CAM_P|EMXX_CLK_CAM_S);

	/* 229.376MHz/20=11.468M
	   writel(0x00000013, SMU_CAMSCLKDIV);*/
	/* 229.376MHz/5=44.8M
	writel(0x00000004, SMU_CAMSCLKDIV); */
	/* 229.376MHz/19=12.072MH
	writel(0x00000012, SMU_CAMSCLKDIV); */
	/* 229.376MHz/10=22.9376MH*/
	writel(0x0000000A, SMU_CAMSCLKDIV);

	FNC_EXIT_N;
	return 0;
}

static inline int camif_stop_clock(void)
{
	/*stop clock*/
	FNC_ENTRY;
	emxx_close_clockgate(EMXX_CLK_CAM | EMXX_CLK_CAM_P|EMXX_CLK_CAM_S);

	FNC_EXIT_N;
	return 0;
}

static inline int camif_irq_enclr(void)
{
	FNC_ENTRY;

	outl(M_CA_ENSET, CA_ENCLR);
	outl(M_CA_ENSET, CA_FFCLR);

	FNC_EXIT_N;
	return 0;
}

static inline __u32 camif_fmt_boundary(const struct emxx_cam_fmt *fmt,
				       __u32 width)
{
	FNC_ENTRY;
	width = width >> fmt->boundary;
	width = width << fmt->boundary;

	FNC_EXIT_N;
	return width;
}


static inline __u32 camif_width_ratio_limit(const struct emxx_cam_fmt *fmt,
					    __u32 size)
{
	u32 limit = 64 * size / 1023;
	FNC_ENTRY;

	size = camif_fmt_boundary(fmt, limit);

	if (limit >= size)
		size += (1 << fmt->boundary);

	FNC_EXIT_N;
	return size;
}

static inline __u32 camif_height_ratio_limit(const struct emxx_cam_fmt *fmt,
					     __u32 size)
{
	u32 limit = 64 * size / 1023;
	FNC_ENTRY;

	if (64 * size % 1023)
		limit += 1;
	size = limit;

	FNC_EXIT_N;
	return size;
}

static inline __u32 camif_width_limit(const struct emxx_cam_fmt *fmt,
				      __u32 width)
{
	u32 limit = 1;
	FNC_ENTRY;

	width = camif_width_ratio_limit(fmt, width);

	limit = limit << fmt->boundary;

	if (limit > width)
		width = limit;

	FNC_EXIT_N;
	return width;
}

static inline __u32 camif_height_limit(const struct emxx_cam_fmt *fmt,
				       __u32 height)
{
	u32 limit = 1;

	FNC_ENTRY;

	height = camif_height_ratio_limit(fmt, height);
	if (limit > height)
		height = limit;

	FNC_EXIT_N;
	return height;
}

static inline int cam_reset_update(void)
{
	int ret = 0;
	FNC_ENTRY;

	em_cam->c = em_cam->pre.bounds;
	em_cam->bounds = em_cam->pre.bounds;
	em_cam->width  = camif_fmt_boundary(em_cam->fmt, em_cam->c.width);
	em_cam->height = em_cam->c.height;

	FNC_EXIT_N;
	return ret;
}

static inline int camif_set_frame_setting(struct camif_reg *ca, __u32 index)
{
	int ret = 0;

	FNC_ENTRY;
	switch (em_cam->fmt->pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV422:
		ca->yplane_a = em_cam->grab->buff[index].padr;
		ca->uvplane_a = ca->yplane_a + (em_cam->width * em_cam->height);
		ca->vplane_a = 0;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		ca->yplane_a = em_cam->grab->buff[index].padr;
		ca->uvplane_a = 0;
		ca->vplane_a = 0;
		break;
	case V4L2_PIX_FMT_YUV422P:
		ca->yplane_a = em_cam->grab->buff[index].padr;
		ca->uvplane_a = ca->yplane_a +
				(em_cam->width * em_cam->height);
		ca->vplane_a = ca->uvplane_a +
				(em_cam->width * em_cam->height / 2);
		break;
	case V4L2_PIX_FMT_YUV420:
		ca->yplane_a = em_cam->grab->buff[index].padr;
		ca->uvplane_a = ca->yplane_a +
				(em_cam->width * em_cam->height);
		ca->vplane_a = ca->uvplane_a +
				(em_cam->width * em_cam->height / 4);
		break;
	case V4L2_PIX_FMT_YVU420:
		ca->yplane_a = em_cam->grab->buff[index].padr;
		ca->vplane_a = ca->yplane_a +
				(em_cam->width * em_cam->height);
		ca->uvplane_a = ca->vplane_a +
				(em_cam->width * em_cam->height / 4);
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT_N;
	return ret;
}

static inline int camif_set_format_setting(struct camif_reg *ca)
{
	int ret = 0;
	FNC_ENTRY;

	switch (em_cam->fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		ca->csr |= (0x01 << S_PIXELMODE);
		ca->od_bytelane  = 0xd8;
		ca->od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_UYVY:
		ca->csr |= (0x01 << S_PIXELMODE);
		ca->od_bytelane  = 0x72;
		ca->od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_YUV422P:
		ca->csr |= (0x01 << S_PIXEL_YUV);
		ca->od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		ca->csr |= (0x01 << S_PIXEL_YUV);
		/* add for tp_value */
		ca->csr |= (0x01 << S_DATA_OD);
		ca->dmacnt |= (0x01 << S_MAINYUV);
		ca->od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_NV12:
		ca->csr |= (0x01 << S_DATA_OD);
		ca->dmacnt |= (0x01 << S_MAINYUV);
		ca->od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_NV21:
		ca->csr |= (0x01 << S_DATA_OD);
		ca->dmacnt |= (0x01 << S_MAINYUV);
		ca->od_bytelane2 = 0xe4b1;
		break;
	case V4L2_PIX_FMT_NV422:
		ca->csr |= (0x01 << S_DATA_OD);
		ca->od_bytelane2 = 0xe4e4;
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT_N;
	return ret;
}

static inline int camif_set_output_setting(struct camif_reg *ca)
{
	int ret = 0;

	FNC_ENTRY;
	/* output setting : transfer */
#if 1
	ca->dmacnt |= (0x01 << S_MNRESIZE); /* resize */
#endif
	ca->od_bytelane = 0xe4;

	/* output setting : address */
	ca->frame |= (0x01 << S_MAINFRM);

	if (em_cam->width && em_cam->height) {
		/* output setting : size */
		ca->dmax_main = em_cam->width;
		ca->dmay_main = em_cam->height;

		switch (em_cam->fmt->pixelformat) {
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		/*case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB24:*/
			ca->linesize_main = em_cam->width * 2;
			break;
		default:
			ca->linesize_main = em_cam->width;
		}

		ca->xratio_main = ((em_cam->c.width - em_cam->width) * 64)
				  / em_cam->width;

		ca->yratio_main = ((em_cam->c.height - em_cam->height) * 64)
				  / em_cam->height;
	} else {
		ret = -EINVAL;
	}

	FNC_EXIT_N;
	return ret;
}

static inline int camif_set_input_setting(struct camif_reg *ca)
{
	int ret = 0;
	int cpe = ((em_cam->pre.clk_edge) ? 1 : 2);
	FNC_ENTRY;

	/* input setting */
	ca->csr |= (em_cam->pre.syncmode << S_SYNCMODE);
	ca->csr |= (em_cam->pre.synctype << S_SYNCTYPE);
	ca->csr |= (em_cam->pre.data_id << S_DATA_ID);
	ca->csr |= (em_cam->pre.vs_det << S_VS_DET);
	ca->csr |= (em_cam->pre.hs_det << S_HS_DET);
	ca->csr |= (em_cam->pre.clk_edge << S_CLK_EDGE);
	ca->csr |= (em_cam->pre.data_det << S_DATA_DET);
	ca->csr |= (em_cam->pre.vs_pol << S_VS_POL);
	ca->csr |= (em_cam->pre.hs_pol << S_HS_POL);
#if 0
	ca->csr |= (0x01 << S_LIMITSEL);  /* @@/ * 8bit full       * / */
#else
	ca->csr |= (0x00 << S_LIMITSEL); /* ITU-R BT656(601) */
#endif

	/* input setting : range */
	ca->x1r = em_cam->pre.c.left * cpe;
	ca->x2r = (em_cam->pre.c.left + em_cam->pre.c.width) * cpe;
	ca->x3r = (em_cam->pre.bounds.left + em_cam->pre.bounds.width) * cpe;
	ca->y1r = em_cam->pre.c.top;
	ca->y2r = em_cam->pre.c.top + em_cam->pre.c.height;

	FNC_EXIT_N;
	return ret;
}

static inline int camif_set_setting(void)
{
	int ret = 0;
	struct camif_reg ca;

	FNC_ENTRY;
	memset(&ca, 0, sizeof(ca));

	ret = camif_set_input_setting(&ca);
	if (ret)
		return ret;

	ret = camif_set_output_setting(&ca);
	if (ret)
		return ret;

	ret = camif_set_format_setting(&ca);
	if (ret)
		return ret;

	assert(!(ca.csr & ~0xffff));
	assert(!(ca.x1r & ~0x1fff));
	assert(!(ca.x2r & ~0x1fff));
	assert(!(ca.x3r & ~0x1fff));
	assert(!(ca.y1r & ~0xfff));
	assert(!(ca.y2r & ~0xfff));
	assert(!(ca.dmacnt & ~0x3fff));
	assert(!(ca.od_bytelane & ~0xff));
	assert(!(ca.od_bytelane2 & ~0xffff));
	assert(!(ca.frame & ~0xf));
	assert(!(ca.dmax_main & ~0xffe));
	assert(!(ca.dmay_main & ~0xfff));
	assert(!(ca.linesize_main & ~0x1ffc));
	assert(!(ca.xratio_main & ~0x3ff));
	assert(!(ca.yratio_main & ~0x3ff));

	outl(ca.csr, CA_CSR);
	outl(ca.x1r, CA_X1R);
	outl(ca.x2r, CA_X2R);
	outl(ca.x3r, CA_X3R);
	outl(ca.y1r, CA_Y1R);
	outl(ca.y2r, CA_Y2R);
	outl(ca.dmacnt, CA_DMACNT);
	outl(ca.od_bytelane, CA_OD_BYTELANE);
	outl(ca.od_bytelane2, CA_OD_BYTELANE2);
	outl(ca.frame, CA_FRAME);
	outl(ca.dmax_main, CA_DMAX_MAIN);
	outl(ca.dmay_main, CA_DMAY_MAIN);
	outl(ca.linesize_main, CA_LINESIZE_MAIN);
	outl(ca.xratio_main, CA_XRATIO_MAIN);
	outl(ca.yratio_main, CA_YRATIO_MAIN);

	FNC_EXIT_N;
	return ret;
}

static inline int camif_set_frame(__u32 index)
{
	int ret = 0;
	struct camif_reg ca;
	FNC_ENTRY;

	memset(&ca, 0, sizeof(ca));

	ret = camif_set_frame_setting(&ca, index);

	if (ret)
		return ret;

	assert(!(ca.yplane_a & ~0xfffffffc));
	assert(!(ca.uvplane_a & ~0xfffffffc));
	assert(!(ca.vplane_a & ~0xfffffffc));

	outl(ca.yplane_a, CA_YPLANE_A);
	outl(ca.uvplane_a, CA_UVPLANE_A);
	outl(ca.vplane_a, CA_VPLANE_A);

	FNC_EXIT_N;
	return ret;
}

struct control_menu_info {
	int value;
	char name[32];
};

static const struct control_menu_info camif_ca_mirror_menus[] =
{
	{ 0x00, "Normal" },
	{ 0x04, "Horizontally" },
	{ 0x08, "Vertically" },
	{ 0x0C, "Horizontally & Vertically" }

};
#define NUM_CA_MIRROR_MENUS ARRAY_SIZE(camif_ca_mirror_menus)

static const struct v4l2_queryctrl no_ctrl = {
	.name  = "camif",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

static const struct v4l2_queryctrl camif_ctrls[] = {
	{
		.id            = EMXX_CID_CA_MIRROR,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Mirror Horizontally & Vertically",
		.minimum       = 0,
		.maximum       = (NUM_CA_MIRROR_MENUS - 1),
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	}, {
		.id            = EMXX_CID_CA_BNGR,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "CA_BNGR is brightness gain",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 1,
		.default_value = 128,
		.flags         = 0,
	}, {
		.id            = EMXX_CID_CA_CBGR,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "CA_CBGR is U color gain",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 1,
		.default_value = 128,
		.flags         = 0,
	}, {
		.id            = EMXX_CID_CA_CRGR,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "CA_CRGR is V color gain",
		.minimum       = 0,
		.maximum       = 255,
		.step          = 1,
		.default_value = 128,
		.flags         = 0,
	}, {
		.id            = EMXX_CID_CA_BNZR,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "CA_BNZR is brightness offset",
		.minimum       = -128,
		.maximum       = 127,
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	}, {
		.id            = EMXX_CID_CA_CBZR,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "CA_CBZR is U color offset",
		.minimum       = -128,
		.maximum       = 127,
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	}, {
		.id            = EMXX_CID_CA_CRZR,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "CA_CRZR is V color offset",
		.minimum       = -128,
		.maximum       = 127,
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	}
};
#define NUM_CAMIF_CTRLS ARRAY_SIZE(camif_ctrls)


/* EMXX_CID_CA_MIRROR */
static inline int camif_set_ca_mirror(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;
#if 0
/* @@	outl(val, CA_MIRROR); */
#else
	outl(camif_ca_mirror_menus[val].value, CA_MIRROR);
#endif
	camif->mirror = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}

/* EMXX_CID_CA_BNGR */
static inline int camif_set_ca_bngr(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;

	outl(val, CA_BNGR);
	camif->bngr = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}

/* EMXX_CID_CA_CBGR */
static inline int camif_set_ca_cbgr(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;

	outl(val, CA_CBGR);
	camif->cbgr = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}

/* EMXX_CID_CA_CRGR */
static inline int camif_set_ca_crgr(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;

	outl(val, CA_CRGR);
	camif->crgr = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}

/* EMXX_CID_CA_BNZR */
static inline int camif_set_ca_bnzr(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;

	outl(val, CA_BNZR);
	camif->bnzr = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}

/* EMXX_CID_CA_CBZR */
static inline int camif_set_ca_cbzr(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;

	outl(val, CA_CBZR);
	camif->cbzr = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}

/* EMXX_CID_CA_CRZR */
static inline int camif_set_ca_crzr(__u8 val)
{
	int ret = 0;
	FNC_ENTRY;

	outl(val, CA_CRZR);
	camif->crzr = val;
	camif->update = 1;

	FNC_EXIT_N;
	return ret;
}



static inline const struct v4l2_queryctrl *camif_ctrl_by_id(unsigned int id)
{
	unsigned int i;
	FNC_ENTRY;

	for (i = 0; i < NUM_CAMIF_CTRLS; i++)
		if (camif_ctrls[i].id == id)
			return camif_ctrls + i;

	FNC_EXIT_N;
	return NULL;
}

static int emxx_camif_vidioc_queryctrl(struct file *file, void *fh,
					struct v4l2_queryctrl *a)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	if (a->id <  EMXX_CID_CA_MIRROR ||
	    a->id > EMXX_CID_CA_CRZR) {
		if (em_cam->hw.vidioc_queryctrl)
			ret = em_cam->hw.vidioc_queryctrl(file, fh, a);
		else
			ret = -EINVAL;
	} else {
		ctrl = camif_ctrl_by_id(a->id);
		*a = (NULL != ctrl) ? *ctrl : no_ctrl;
	}

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_vidioc_querymenu(struct file *file, void *fh,
					struct v4l2_querymenu *m)
{
	int ret = 0;
	FNC_ENTRY;

	/* memset(m->name, 0, sizeof(m->name)); */
	/* m->reserved = 0; */

	switch (m->id) {
	case EMXX_CID_CA_MIRROR:
		if (m->index < 0 || m->index >= NUM_CA_MIRROR_MENUS) {
			ret = -EINVAL;
			break;
		}
		strcpy(m->name, camif_ca_mirror_menus[m->index].name);
		break;
	default:
		if (em_cam->hw.vidioc_querymenu)
			ret = em_cam->hw.vidioc_querymenu(file, fh, m);
		else
			ret = -EINVAL;
		break;
	}

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_vidioc_g_ctrl(struct file *file, void *fh,
				     struct v4l2_control *c)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = camif_ctrl_by_id(c->id);
	if (NULL == ctrl) {
		if (em_cam->hw.vidioc_g_ctrl)
			ret = em_cam->hw.vidioc_g_ctrl(file, fh, c);
		else
			ret = -EINVAL;
		FNC_EXIT(ret)
		return ret;
	}
	switch (c->id) {
	case EMXX_CID_CA_MIRROR:
		c->value = camif->mirror;
		break;
	case EMXX_CID_CA_BNGR:
		c->value = camif->bngr;
		break;
	case EMXX_CID_CA_CBGR:
		c->value = camif->cbgr;
		break;
	case EMXX_CID_CA_CRGR:
		c->value = camif->crgr;
		break;
	case EMXX_CID_CA_BNZR:
		c->value = camif->bnzr;
		break;
	case EMXX_CID_CA_CBZR:
		c->value = camif->cbzr;
		break;
	case EMXX_CID_CA_CRZR:
		c->value = camif->crzr;
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_vidioc_s_ctrl(struct file *file, void *fh,
				     struct v4l2_control *c)
{
	int ret = 0;
	int moving;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = camif_ctrl_by_id(c->id);
	if (NULL == ctrl) {
		if (em_cam->hw.vidioc_s_ctrl) {
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
			moving = ((em_cam->mapping
				   | em_cam->reading
				   | em_cam->streaming) ? 1 : 0);
#else
			moving = (em_cam->streaming ? 1 : 0);
#endif
			ret = em_cam->hw.vidioc_s_ctrl(file, (void *)moving, c);
			if (!ret) {
				if (em_cam->hw.prepare) {
					em_cam->pre.actions = 0;
					ret = em_cam->hw.prepare(&em_cam->pre);
					if (ret) {
						d1b("stop\n");
						em_cam->stop = 1;
						FNC_EXIT(ret)
						return ret;
					}
					if (em_cam->pre.reset) {
						cam_reset_update();
						em_cam->reset = 1;
					}
				}
			}
			FNC_EXIT(ret)
			return ret;
		} else {
			FNC_EXIT(ret)
			return -EINVAL;
		}
	}
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	}
	mutex_lock(&camif->lock);
	switch (c->id) {
	case EMXX_CID_CA_MIRROR:
		ret = camif_set_ca_mirror(c->value);
		break;
	case EMXX_CID_CA_BNGR:
		ret = camif_set_ca_bngr(c->value);
		break;
	case EMXX_CID_CA_CBGR:
		ret = camif_set_ca_cbgr(c->value);
		break;
	case EMXX_CID_CA_CRGR:
		ret = camif_set_ca_crgr(c->value);
		break;
	case EMXX_CID_CA_BNZR:
		ret = camif_set_ca_bnzr(c->value);
		break;
	case EMXX_CID_CA_CBZR:
		ret = camif_set_ca_cbzr(c->value);
		break;
	case EMXX_CID_CA_CRZR:
		ret = camif_set_ca_crzr(c->value);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&camif->lock);

	FNC_EXIT(ret)
	return ret;
}

static void emxx_camif_reg_debug(void);

static irqreturn_t emxx_camif_handler(int irq, void *dev_id)
{
	int ret = 0;

	FNC_ENTRY;

	camif->status = inl(CA_STATUS);
	/* info(" status is 0x%x\n", camif->status); */


	assert(!(~M_CA_ENSET & camif->status));

	em_cam->status |= camif->status;

	if (B_MAINOR & camif->status) {
		err("%s: MAINOR fault.\n", CAM_NAME);
		ret = IRQ_HANDLED;
	}
	if (B_DMAERR & camif->status) {
		err("%s: DMAERR fault.\n", CAM_NAME);
		ret = IRQ_HANDLED;
	}

	if (ret) {
		emxx_camif_reg_debug();

		outl(M_CA_ENSET, CA_ENCLR);
		outl(M_CA_ENSET, CA_FFCLR);
		d1b("stop\n");
		em_cam->stop = 1;
		if (em_cam->grab->update)
			em_cam->grab->update(em_cam);
		return ret;
	}

	ret = IRQ_HANDLED;

	if (B_CAMVS & camif->status) {
		d1b(" B_CAMVS !\n");
#ifdef CAM_FPS_DEBUG
		vsync_detect_cnt++;
#endif
		if (camif->update) {
			d1b(" update the UPDATE register!!!!!!\n");
			outl(camif->update, CA_UPDATE);
			camif->update = 0;
		}
	}

	if (B_MAINTC & camif->status) {
		d1b(" B_MAINTC !\n");
#ifdef CAM_FPS_DEBUG
		transmit_end_cnt++;
#endif
		/* emxx_camif_reg_debug();*/
		if (em_cam->grab->update)
			em_cam->grab->update(em_cam);
	}

	outl(camif->status, CA_FFCLR);

	FNC_EXIT_N;
	return ret;
}

static int emxx_camif_running(int flag)
{
	FNC_ENTRY;
	return inl(CA_DMAREQ) ? 1 : 0;
}

static int emxx_camif_restart(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	/* module reset */
	outl(0, CA_MODULECONT);

	schedule_timeout_uninterruptible(1);

	/* initial */
	ret = camif_set_setting();

	/* module unreset */
	outl(1, CA_MODULECONT);

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_capture(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	outl(0x01, CA_DMAREQ);

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_stop_capture(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	/*outl(0x01, CA_DMASTOP);*/

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_prepare(__u32 index)
{
	int ret = 0;
	FNC_ENTRY;

	if (em_cam->setup) {
		em_cam->pre.width = em_cam->width;
		em_cam->pre.height = em_cam->height;
		em_cam->pre.c = em_cam->c;
#if 1 /* XXX */
		ret = emxx_camif_restart(0);
#else
		ret = camif_set_setting();
#endif

		if (ret) {
			FNC_EXIT(ret)
			return ret;
		}

		/* ret = camif_set_frame(0); */
		outl(1, CA_UPDATE);
		em_cam->setup = 0;
	}

	ret = camif_set_frame(index);

	FNC_EXIT(ret)
	return ret;
}

#ifdef CAM_FPS_DEBUG
static void emxx_camif_buffer_debug(unsigned int index)
{
	unsigned int i = 0;
	unsigned int j;
	unsigned int length = em_cam->width*em_cam->width;
	char *addr_y = em_cam->grab->buff[index].vadr;
	FNC_ENTRY;

	for (i = 0; i < length; i++) {
		if (addr_y[i] == 0) {
			for (j = i + 1; j < length; j++) {
				if (addr_y[j] != 0)
					break;
			}
			if (j < length) {
				i = j;
				continue;
			}
			d1b("buffer y: %d\n", i);
			break;
		}
	}

	FNC_EXIT_N;
	return;
}
#endif

static void emxx_camif_reg_debug(void)
{
	FNC_ENTRY;

	d1b("  CA_STATUS        is 0x%08x\n", inl(CA_STATUS));
	d1b("  CA_RAWSTATUS     is 0x%08x\n", inl(CA_RAWSTATUS));
	d1b("  CA_ENSET         is 0x%08x\n", inl(CA_ENSET));
	d1b("  CA_ENCLR         is 0x%08x\n", inl(CA_ENCLR));
	d1b("  CA_FFCLR         is 0x%08x\n", inl(CA_FFCLR));
	d1b("  CA_ERRORADR      is 0x%08x\n", inl(CA_ERRORADR));
	d1b("  CA_CSR           is 0x%08x\n", inl(CA_CSR));
	d1b("  CA_X1R           is 0x%08x\n", inl(CA_X1R));
	d1b("  CA_X2R           is 0x%08x\n", inl(CA_X2R));
	d1b("  CA_Y1R           is 0x%08x\n", inl(CA_Y1R));
	d1b("  CA_Y2R           is 0x%08x\n", inl(CA_Y2R));
	d1b("  CA_BNZR          is 0x%08x\n", inl(CA_BNZR));
	d1b("  CA_BNGR          is 0x%08x\n", inl(CA_BNGR));
	d1b("  CA_CBZR          is 0x%08x\n", inl(CA_CBZR));
	d1b("  CA_CBGR          is 0x%08x\n", inl(CA_CBGR));
	d1b("  CA_CRZR          is 0x%08x\n", inl(CA_CRZR));
	d1b("  CA_CRGR          is 0x%08x\n", inl(CA_CRGR));
	d1b("  CA_DMACNT        is 0x%08x\n", inl(CA_DMACNT));
	d1b("  CA_FRAME         is 0x%08x\n", inl(CA_FRAME));
	d1b("  CA_DMAREQ        is 0x%08x\n", inl(CA_DMAREQ));
	d1b("  CA_DMASTOP       is 0x%08x\n", inl(CA_DMASTOP));
	d1b("  CA_LINESIZE_MAIN is 0x%08x\n", inl(CA_LINESIZE_MAIN));
	d1b("  CA_XRATIO_MAIN   is 0x%08x\n", inl(CA_XRATIO_MAIN));
	d1b("  CA_YRATIO_MAIN   is 0x%08x\n", inl(CA_YRATIO_MAIN));
	d1b("  CA_DMAX_MAIN     is 0x%08x\n", inl(CA_DMAX_MAIN));
	d1b("  CA_DMAY_MAIN     is 0x%08x\n", inl(CA_DMAY_MAIN));
	d1b("  CA_YPLANE_A      is 0x%08x\n", inl(CA_YPLANE_A));
	d1b("  CA_UVPLANE_A     is 0x%08x\n", inl(CA_UVPLANE_A));
	d1b("  CA_VPLANE_A      is 0x%08x\n", inl(CA_VPLANE_A));
	d1b("  CA_YPLANE_B      is 0x%08x\n", inl(CA_YPLANE_B));
	d1b("  CA_UVPLANE_B     is 0x%08x\n", inl(CA_UVPLANE_B));
	d1b("  CA_VPLANE_B      is 0x%08x\n", inl(CA_VPLANE_B));
	d1b("  CA_MODULECONT    is 0x%08x\n", inl(CA_MODULECONT));
	d1b("  CA_UPDATE        is 0x%08x\n", inl(CA_UPDATE));
	d1b("  CA_MIRROR        is 0x%08x\n", inl(CA_MIRROR));
	d1b("  CA_OD_BYTELANE   is 0x%08x\n", inl(CA_OD_BYTELANE));
	d1b("  CA_X3R           is 0x%08x\n", inl(CA_X3R));
	d1b("  CA_OD_BYTELANE2  is 0x%08x\n", inl(CA_OD_BYTELANE2));
	d1b("  CA_QOS           is 0x%08x\n", inl(CA_QOS));

	FNC_EXIT_N;
}

void emxx_camif_change_io_level(void)
{
	/* Please set the best voltage for your camera. */

	pwc_write(DA9052_SUPPLY_REG, 0x10, 0x10);
}

static void emxx_camif_unreset(void)
{
	FNC_ENTRY;
	/* module unreset */
	outl(1, CA_MODULECONT);
	/* update the reservation */
	outl(1, CA_UPDATE);

	FNC_EXIT_N;
}

static int emxx_camif_startup(void)
{
	int ret = 0;
	FNC_ENTRY;

	/* start clock */
	camif_start_clock();

	/* change pin select */
	camif_chg_pinsel();

	/* irq */
	camif_irq_enclr();

	/* request irq handler*/
	ret = request_irq(INT_CAM, emxx_camif_handler,
			  IRQF_DISABLED, "CAMIF", (void *)em_cam);

	if (ret) {
		FNC_EXIT(-ENODEV)
		return -ENODEV;
	}

	outl(M_CA_ENSET, CA_ENSET);

	/* initial */
	/*first time use this function,
	it will set the initial values into the CAM register, maybe not right
	but it will update before the capture or preview*/
	ret = camif_set_setting();
#if 0
	/* module unreset */
/* @@	outl(1, CA_MODULECONT); */
/* @@	outl(1, CA_UPDATE); */
#else
/*	emxx_camif_unreset(); */
#endif

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_shutdown(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	/* module reset */
	outl(0, CA_MODULECONT);

	/* irq */
	camif_irq_enclr();

	free_irq(INT_CAM, (void *)em_cam);

	/* stop clock */
	camif_stop_clock();

	/* irq */
	outl(M_CA_ENSET, CA_ENSET);

	/* initial */
	ret = camif_set_setting();

	/* module unreset */
	outl(1, CA_MODULECONT);
	outl(1, CA_UPDATE);

	/* module power off*/
	camif_power_off(CAM_POWERDOWN);
	/* emxx_cam_free_buff(); */

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_unregister(void)
{
	int ret = 0;
	FNC_ENTRY;

	kfree(camif);
	camif = NULL;

	FNC_EXIT(ret)
	return ret;
}

static int emxx_camif_register(void)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	camif = kzalloc(sizeof(*camif), GFP_KERNEL);

	if (NULL == camif)
		return -ENOMEM;

	/* initial camif */
	mutex_init(&camif->lock);

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_MIRROR);
	camif->mirror  = ctrl->default_value;

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_BNGR);
	camif->bngr  = ctrl->default_value;

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_CBGR);
	camif->cbgr  = ctrl->default_value;

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_CRGR);
	camif->crgr  = ctrl->default_value;

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_BNZR);
	camif->bnzr  = ctrl->default_value;

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_CBZR);
	camif->cbzr  = ctrl->default_value;

	ctrl = camif_ctrl_by_id(EMXX_CID_CA_CRZR);
	camif->crzr  = ctrl->default_value;

	FNC_EXIT_N;
	return ret;
}

/*
 * Tools
 */
static inline int no_active(__u32 number)
{
	return (em_cam->active_number == number) ? 0 : 1;
}

static const struct v4l2_queryctrl cam_ctrls[] = {
#if 0 /* not supported */
/* @@	{ */
/* @@		.id            = EMXX_CID_IPU, */
/* @@		.type          = V4L2_CTRL_TYPE_BOOLEAN, */
/* @@		.name          = "Use a image processor", */
/* @@		.minimum       = 0, */
/* @@		.maximum       = 1, */
/* @@		.step          = 1, */
/* @@		.default_value = 1, */
/* @@		.flags         = 0, */
/* @@	} */
#endif

};
#define NUM_CAM_CTRLS ARRAY_SIZE(cam_ctrls)

static inline const struct v4l2_queryctrl *cam_ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < NUM_CAM_CTRLS; i++)
		if (cam_ctrls[i].id == id)
			return cam_ctrls + i;
	return NULL;
}

#define FORMAT_FLAGS_PACKED       0x00
#define FORMAT_FLAGS_PLANAR       0x01

static const struct emxx_cam_fmt emxx_cam_formats[] = {
	{
		.description    = "YUYV : YUV422 Interleave",
		/* Y U Y V Y U Y V Y U Y V Y U Y V Y U Y V */
		.pixelformat    = V4L2_PIX_FMT_YUYV,
		/* Y U Y V Y U Y V Y U Y V Y U Y V Y U Y V */
		.depth          = 16,
		.flags          = FORMAT_FLAGS_PACKED,
		.boundary       = 1,
	}, {
#if 1  /*need to reconfirm whether support this format*/
		.description	= "UYVY : YUV 422 Interleave",
		 /* U Y V Y U Y V Y U Y V Y U Y V Y U Y V Y */
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		 /* U Y V Y U Y V Y U Y V Y U Y V Y U Y V Y */
		.depth		= 16,
		.flags		= FORMAT_FLAGS_PACKED,
		.boundary	= 1,
#endif
	}, {
		.description    = "YUV422P : YUV 422 Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_YUV422P,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 16,
		/* U U U U U U U U */
		.flags          = FORMAT_FLAGS_PLANAR,
		/* V V V V V V V V */
		.boundary       = 3,
	}, {
		.description    = "NV422 : YUV 422 Semi-Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_NV422,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 16,
		/* U V U V U V U V */
		.flags          = FORMAT_FLAGS_PLANAR,
		/* U V U V U V U V */
		.boundary       = 2,
	}, {
		.description    = "YUV420 : YUV 420 Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 12,
		/* U U U U         */
		.flags          = FORMAT_FLAGS_PLANAR,
		/* V V V V         */
		.boundary       = 3,
#if 0 /* not supported */
/* @@	}, { */
/* @@		.description	= "YVU420 : YUV 420 Planar",
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.pixelformat	= V4L2_PIX_FMT_YVU420,
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.depth		= 12,
/ * V V V V         * / */
/* @@		.flags		= FORMAT_FLAGS_PLANAR,
/ * U U U U         * / */
/* @@		.boundary	= 3, */
#endif
	}, {
		.description    = "NV12 : YUV 420 Semi-Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_NV12,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 12,
		/* U V U V U V U V */
		.flags          = FORMAT_FLAGS_PLANAR,
		.boundary       = 2,
#if 0 /* not supported */
/* @@	}, { */
/* @@		.description	= "NV21 : YUV 420 Semi-Planar",
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.pixelformat	= V4L2_PIX_FMT_NV21,
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.depth		= 12,
/ * V U V U V U V U * / */
/* @@		.flags		= FORMAT_FLAGS_PLANAR, */
/* @@		.boundary	= 2, */
#endif
	}
};
#define NUM_CAM_FORMATS ARRAY_SIZE(emxx_cam_formats)

static inline const struct emxx_cam_fmt *format_by_pixelformat(
	__u32 pixelformat)
{
	unsigned int i;
	FNC_ENTRY;

	for (i = 0; i < NUM_CAM_FORMATS; i++)
		if (emxx_cam_formats[i].pixelformat == pixelformat)
			return emxx_cam_formats + i;

	FNC_EXIT_N;
	return NULL;
}

static inline __u32 get_sizeimage(const struct emxx_cam_fmt *fmt,
				  __u32 width, __u32 height,
				  __u32 *bytesperline)
{
	__u32 sizeimage;
	FNC_ENTRY;

	if (fmt->flags & FORMAT_FLAGS_PLANAR) {
		*bytesperline = width; /* Y plane */
		sizeimage = (width * height * fmt->depth) >> 3;
	} else {
		*bytesperline = (width * fmt->depth) >> 3;
		sizeimage = height * *bytesperline;
	}

	FNC_EXIT(sizeimage)
	return sizeimage;
}

static inline void pix_format_set_size(struct v4l2_pix_format *f,
				       const struct emxx_cam_fmt *fmt,
				       __u32 width, __u32 height)
{
	FNC_ENTRY;
	f->width = width;
	f->height = height;

	f->sizeimage = get_sizeimage(fmt, width, height, &f->bytesperline);
	FNC_EXIT_N;
}

static inline int check_reset_fmt(const struct emxx_cam_fmt *src,
				  const struct emxx_cam_fmt *dir)
{
	int reset = 0;
	FNC_ENTRY;

	if (src->depth != dir->depth)
		reset = 1;
	if (V4L2_PIX_FMT_RGB24 == src->pixelformat
	    && V4L2_PIX_FMT_RGB24 != dir->pixelformat) {
		reset = 1;
	}
	if (V4L2_PIX_FMT_RGB565 == src->pixelformat
	    && V4L2_PIX_FMT_RGB565 != dir->pixelformat) {
		reset = 1;
	}

	FNC_EXIT(reset)
	return reset;
}

static inline unsigned int __kfifo_try_get(struct kfifo *fifo,
					   unsigned char *buffer,
					   unsigned int len)
{
	unsigned int l;
	FNC_ENTRY;

	len = min(len, fifo->in - fifo->out);

	/*
	 * Ensure that we sample the fifo->in index -before- we
	 * start removing bytes from the kfifo.
	 */

	smp_rmb();

	/* first get the data from fifo->out until the end of the buffer */
	l = min(len, fifo->size - (fifo->out & (fifo->size - 1)));
	memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), l);

	/* then get the rest (if any) from the beginning of the buffer */
	memcpy(buffer + l, fifo->buffer, len - l);

	/*
	 * Ensure that we remove the bytes from the kfifo -before-
	 * we update the fifo->out index.
	 */

	smp_mb();

	/* fifo->out += len; */

	FNC_EXIT(len)
	return len;
}

static inline unsigned int kfifo_try_get(struct kfifo *fifo,
					 unsigned char *buffer,
					 unsigned int len)
{
	unsigned long flags;
	unsigned int i = len;
	unsigned int ret;
	FNC_ENTRY;

	spin_lock_irqsave(fifo->lock, flags);

	ret = __kfifo_try_get(fifo, buffer, len);
	len = fifo->in - fifo->out;

	spin_unlock_irqrestore(fifo->lock, flags);

	assert((0 == ret || 1 == (ret / i)));

	ret = len / i;

	FNC_EXIT(ret)
	return ret;
}

static inline void deq_done_pushq(unsigned int q)
{
	unsigned int ret;
	FNC_ENTRY;

	ret = kfifo_put(em_cam->deq_done, (unsigned char *)&q, sizeof(int));
#ifdef CAM_FPS_DEBUG
	deq_done_push_cnt++;
#endif
	assert(1 == (ret / sizeof(int)));
	d2b("deq_done_pushq: %d\n", q);

	FNC_EXIT_N;
	return;
}


static inline unsigned int deq_done_pullq(void)
{
	unsigned int buf;
	unsigned int ret;
	FNC_ENTRY;

	ret = kfifo_get(em_cam->deq_done, (unsigned char *)&buf, sizeof(int));

	assert(1 == (ret / sizeof(int)));
#ifdef CAM_FPS_DEBUG
	deq_done_pull_cnt++;
#endif
	d2b("deq_done_pullq: %d\n", buf);

	FNC_EXIT_N;
	return buf;
}

static inline unsigned int deq_done_peepq(unsigned int *q)
{
	FNC_ENTRY;
	return kfifo_try_get(em_cam->deq_done, (unsigned char *)q, sizeof(int));
}


static inline void grab_pushq(unsigned int q)
{
	unsigned int ret;
	FNC_ENTRY;

	ret = kfifo_put(em_cam->grab->enq, (unsigned char *)&q, sizeof(int));

	assert(1 == (ret / sizeof(int)));

	em_cam->grab->cnt++;
	if (em_cam->grab->cnt >= em_cam->grab->max)
		em_cam->grab->cnt = 0;

#ifdef CAM_FPS_DEBUG
	grab_push_cnt++;
#endif
	em_cam->grab->buff[q].state = CAM_BUF_QUEUED;
	d2b("grab_pushq: %d\n", q);
	FNC_EXIT_N;
	return;
}

static inline unsigned int grab_pullq(void)
{
	unsigned int buf;
	unsigned int ret;
	FNC_ENTRY;

	ret = kfifo_get(em_cam->grab->enq, (unsigned char *)&buf,
			sizeof(int));

	assert(1 == (ret / sizeof(int)));

	em_cam->grab->buff[buf].state = CAM_BUF_DONE;

#ifdef CAM_FPS_DEBUG
	grab_pull_cnt++;
#endif
	d2b("grab_pullq: %d\n", buf);

	FNC_EXIT_N;
	return buf;
}

static inline unsigned int grab_peepq(unsigned int *q)
{
	FNC_ENTRY;
	return kfifo_try_get(em_cam->grab->enq, (unsigned char *)q,
			     sizeof(int));
}

static inline unsigned int deq_peepq(unsigned int *i, struct kfifo *q)
{
	FNC_ENTRY;
	return kfifo_try_get(q, (unsigned char *)i, sizeof(int));
}

static inline void deq_pushq(unsigned int grab)
{
	FNC_ENTRY;

	deq_done_pushq(grab);
	grab_pushq(grab);

	FNC_EXIT_N;
	return;
}


static int emxx_cam_grabs_handler(void *p);


static void release_userptr(struct emxx_cam_buffer *buff)
{
	FNC_ENTRY;
	if (buff->m.userptr) {
		iounmap((void *)buff->vadr);
		buff->vadr = NULL;
		buff->padr = 0;
		buff->m.userptr = 0;
	}

	FNC_EXIT_N;
	return;
}

static int request_userptr(struct emxx_cam_buffer *buff,
			   unsigned long userptr, __u32 length)
{
	int ret = 0;
	FNC_ENTRY;

	d1b("(%p, %lu, %d)\n", buff, userptr, length);

	buff->vadr = ioremap(userptr, length);

	if (buff->vadr) {
		buff->padr = userptr;
		buff->m.userptr = userptr;
	} else {
		err("%s: userptr can't set up page mapping.\n", CAM_NAME);
		ret = -EINVAL;
		buff->vadr = NULL;
		buff->padr = 0;
		buff->m.userptr = 0;
	}

	FNC_EXIT(ret)
	return ret;
}

static void release_frm_userptr(struct emxx_cam_frames *frms)
{
	int i;
	__u32 count = frms->max;
	FNC_ENTRY;

	d1b("(%p)\n", frms);

	for (i = 0; i < count; i++)
		release_userptr(&frms->buff[i]);

	FNC_EXIT_N;
	return;
}

static unsigned long camera_frame_offset;

static void free_fixed_buffer(struct emxx_cam_frames *frms)
{
	int i;
	__u32 count = frms->max;
	FNC_ENTRY;

	iounmap((void *)frms->buff[0].vadr);

	for (i = 0; i < count; i++) {
		frms->buff[i].vadr = NULL;
		frms->buff[i].padr = 0;
		frms->buff[i].m.offset = 0;
	}

	FNC_EXIT_N;
	return;
}

static int alloc_fixed_buffer(__u32 count, size_t blocksize,
			      struct emxx_cam_buffer *buff)
{
	int i = 0;
	char *vadr = NULL;
	unsigned long camera_frame = CAMERA_FRAME_BASE + camera_frame_offset;
	unsigned long frame_size = CAMERA_FRAME_SIZE - camera_frame_offset;
	FNC_ENTRY;

	d1b("(%d, %d, %p)\n", count, blocksize, buff);

	for (i = count; i > 0; i--) {
		if (frame_size > blocksize * i)
			break;
	}

	if (!i) {
		err("%s: no buffer.\n", CAM_NAME);
		return 0;
	}

	count = i;

	vadr = ioremap(camera_frame, blocksize * count);

	for (i = 0; i < count; i++) {
		buff[i].index = i;
		buff[i].m.offset = blocksize * i;
		buff[i].vadr = vadr + (blocksize * i);
		buff[i].padr = camera_frame + (blocksize * i);
	}

	camera_frame_offset += blocksize * count;

	FNC_EXIT(i)
	return i;
}

static void release_frame_buff(struct emxx_cam_frames *frm)
{
	FNC_ENTRY;

	if (V4L2_MEMORY_MMAP == frm->memory) {
		free_fixed_buffer(frm);
	} else {
		release_frm_userptr(frm);
		em_cam->userptr   = 0;
	}

	frm->max = 0;
	frm->blocksize = 0;
	frm->memory = 0;
	kfree(frm->buff);
	frm->buff = NULL;

	FNC_EXIT_N;
	return;
}

static int request_frame_buff(struct emxx_cam_frames *frm,
			      __u32 count, const struct emxx_cam_fmt *fmt,
			      enum v4l2_memory memory, int buf_type)
{
	__u32 blocksize, bpl;
	FNC_ENTRY;

	frm->buff = kzalloc(count * sizeof(*em_cam->grab->buff), GFP_KERNEL);

	if (NULL == frm->buff) {
		err("%s: frame info allocation failed\n", CAM_NAME);
		return 0;
	}

	blocksize = get_sizeimage(fmt, em_cam->bounds.width,
				  em_cam->bounds.height, &bpl);

	blocksize = PAGE_ALIGN(blocksize);

	if (V4L2_MEMORY_MMAP == memory)
		count = alloc_fixed_buffer(count, blocksize, frm->buff);
	else
		em_cam->userptr   = 1;

	if (count) {
		frm->max = count;
		frm->blocksize = blocksize;
		frm->memory = memory;
		frm->fmt = fmt;
		frm->buf_type = buf_type;
	} else {
		frm->max = 0;
		frm->blocksize = 0;
		frm->memory = 0;
		kfree(frm->buff);
		frm->buff = NULL;
	}

	frm->cnt = 0;

	FNC_EXIT(count)
	return count;
}

static void cam_release_buffers(int flag)
{
	FNC_ENTRY;

	if (!em_cam->grab)
		goto out_release_buffers;

	release_frame_buff(em_cam->grab);
	camera_frame_offset = 0;

out_release_buffers:

	FNC_EXIT_N;
	return;
}

static int cam_request_buffers(int action, __u32 count,
			       enum v4l2_memory memory)
{
	/* action 0:mmap, 1:read */
	int ret = 0;
	int i = 0;
	FNC_ENTRY;

	i = request_frame_buff(em_cam->grab, count, em_cam->fmt,
				       memory, BUF_FIXED);

	if (i) {
		d1b("request %d buffers\n", em_cam->grab->max);
		em_cam->reset = 0;
	} else
		ret = -ENOMEM;

	FNC_EXIT(i)
	return ret;
}



static int cam_choice_function(int action)
{
	/* action 0 : mmap, 1 : read */
	int ret = 0;
	FNC_ENTRY;

	assert(em_cam->grab);

	switch (em_cam->fmt->pixelformat) {
	default:
		em_cam->grab->update = emxx_cam_grabs_handler;
	}

	FNC_EXIT_N;
	return ret;
}

static inline void cam_buffer_status(struct v4l2_buffer *b,
				     __u32 index, enum v4l2_buf_type type)
{
	struct emxx_cam_buffer *cdb = &em_cam->grab->buff[index];
	FNC_ENTRY;

	b->index    = index;
	b->type     = type;
	b->memory   = em_cam->grab->memory;

	switch (b->memory) {
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	case V4L2_MEMORY_MMAP:
		b->m.offset  = cdb->m.offset;
		b->length    = em_cam->grab->blocksize;
		break;
#endif
	case V4L2_MEMORY_USERPTR:
		b->m.userptr = cdb->m.userptr;
		b->length    = em_cam->grab->blocksize;
		break;
	default:
		/* nothing */;
	}

	b->flags    = 0;
	if (cdb->map)
		b->flags |= V4L2_BUF_FLAG_MAPPED;

	switch (cdb->state) {
	case CAM_BUF_QUEUED:
	case CAM_BUF_GRABBING:
		b->flags |= V4L2_BUF_FLAG_QUEUED;
		break;
	case CAM_BUF_DONE:
		b->flags |= V4L2_BUF_FLAG_DONE;
		break;
	case CAM_BUF_IDLE:
	case CAM_BUF_BREAK:
		/* nothing */
		break;
	}

	b->field = V4L2_FIELD_NONE;

	b->timestamp = cdb->timestamp;
	b->bytesused = cdb->bytesused;
	b->sequence  = cdb->sequence;

	FNC_EXIT_N;
	return;
}

static int emxx_cam_thread(void *p);

#ifdef EMXX_CAM_MAKING_DEBUG
#define LIMIT_TIME 2
#else
#define LIMIT_TIME 1
#endif


static inline int cam_grab_waiton(struct emxx_cam_frames *frm,
			     struct kfifo *q, int non_blocking,
			     int intr, int th, int timeout)
{
	int ret = 0;
	__u32 state;
	unsigned int len, index, limit;
	unsigned long flags;
	DECLARE_WAITQUEUE(wait, current);
	FNC_ENTRY;

	if (timeout)
		limit = timeout / HZ;
	else
		limit = 1;

	local_irq_save(flags);

	len = deq_peepq(&index, q);
	if (len)
		state = frm->buff[index].state;
	else {
		ret = -EAGAIN ;
		return ret;
	}

	switch (state) {
	case CAM_BUF_BREAK:
		err("%s: buffer break.\n", CAM_NAME);
		ret = -EIO;
		break;

	case CAM_BUF_GRABBING:
		if (non_blocking) {
			ret = -EAGAIN;
			break;
		}
		set_current_state(intr  ? TASK_INTERRUPTIBLE
				  : TASK_UNINTERRUPTIBLE);

		add_wait_queue(&frm->proc_list, &wait);
		do {
			local_irq_restore(flags);

			if (timeout) {
				schedule_timeout(HZ);
				limit--;
				d1b("chk limit %d\n", limit);
			} else {
				schedule();
			}

			/* assert(limit != 0); */

			local_irq_save(flags);

			set_current_state(intr  ? TASK_INTERRUPTIBLE
					  : TASK_UNINTERRUPTIBLE);

			if (intr && signal_pending(current)) {
				warn("%s: buffer waiton: -EINTR.\n", CAM_NAME);
				ret = -EINTR;
				break;
			}

			if (th && kthread_should_stop()) {
				ret = -EINTR;
				break;
			}

			deq_peepq(&index, q);

			len = deq_peepq(&index, q);
			if (!len)
				break;

			if (CAM_BUF_BREAK == frm->buff[index].state) {
				err("%s: buffer break.\n", CAM_NAME);
				ret = -EIO;
				break;
			}

			if (CAM_BUF_DONE == frm->buff[index].state
			    || CAM_BUF_IDLE == frm->buff[index].state
			    || CAM_BUF_QUEUED == frm->buff[index].state) {
				break;
			}
		} while (limit);
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&frm->proc_list, &wait);

		if (ret)
			break;
	case CAM_BUF_DONE:
	case CAM_BUF_QUEUED:

		break;
	}

	local_irq_restore(flags);

	if (!ret && !limit)
		ret = -ETIMEDOUT;

	FNC_EXIT(ret)
	return ret;
}

static inline int cam_waiton(struct emxx_cam_frames *frm,
			     struct kfifo *q, int non_blocking,
			     int intr, int th, int timeout)
{
	int ret = 0;
	__u32 state;
	unsigned int len, index, limit;
	unsigned long flags;
	DECLARE_WAITQUEUE(wait, current);
	FNC_ENTRY;

	if (timeout)
		limit = timeout / HZ;
	else
		limit = 1;

	local_irq_save(flags);

	/* d1b("cam_waiton chk SIGINT %d\n", sigismember(
					&(current->blocked), SIGINT)); */

	len = deq_peepq(&index, q);
	if (len)
		state = frm->buff[index].state;
	else
		state = CAM_BUF_GRABBING;

	switch (state) {
	case CAM_BUF_BREAK:
		err("%s: buffer break.\n", CAM_NAME);
		ret = -EIO;
		break;
	case CAM_BUF_IDLE:
	case CAM_BUF_QUEUED:
	case CAM_BUF_GRABBING:
		if (non_blocking) {
			ret = -EAGAIN;
			break;
		}
		set_current_state(intr  ? TASK_INTERRUPTIBLE
				  : TASK_UNINTERRUPTIBLE);

		add_wait_queue(&frm->proc_list, &wait);
		do {
			local_irq_restore(flags);

			if (timeout) {
				schedule_timeout(HZ);
				limit--;
				d1b("chk limit %d\n", limit);
			} else {
				schedule();
			}

			/* assert(limit != 0); */

			local_irq_save(flags);

			set_current_state(intr  ? TASK_INTERRUPTIBLE
					  : TASK_UNINTERRUPTIBLE);

			if (intr && signal_pending(current)) {
				warn("%s: buffer waiton: -EINTR.\n", CAM_NAME);
				ret = -EINTR;
				break;
			}

			if (th && kthread_should_stop()) {
				ret = -EINTR;
				break;
			}

			deq_peepq(&index, q);

			len = deq_peepq(&index, q);
			if (!len)
				continue;

			if (CAM_BUF_BREAK == frm->buff[index].state) {
				err("%s: buffer break.\n", CAM_NAME);
				ret = -EIO;
				break;
			}

			if (CAM_BUF_DONE == frm->buff[index].state
			    || CAM_BUF_IDLE == frm->buff[index].state) {
				break;
			}
		} while (limit);
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&frm->proc_list, &wait);

		if (ret)
			break;
	case CAM_BUF_DONE:
		break;
	}

	local_irq_restore(flags);

	if (!ret && !limit)
		ret = -ETIMEDOUT;

	FNC_EXIT(ret)
	return ret;
}

static inline int cam_stop(int flag)
{
	int ret = 0;
	unsigned int i, len, index;
	FNC_ENTRY;

	em_cam->stop = 1;

	emxx_camif_stop_capture(0);

	len = deq_done_peepq(&index);

	for (i = 0; i < len; i++) {
		if (CAM_BUF_GRABBING == em_cam->grab->buff[index].state) {
			cam_waiton(em_cam->grab, em_cam->deq_done,
				   0, 1, 0, LIMIT_TIME * HZ);
			break;
		}
		deq_done_pullq();
		deq_done_peepq(&index);
	}


	if (emxx_camif_running(0)) {
		err("%s: don't stream stop !\n", CAM_NAME);
		ret = -EFAULT;
	}

	if (em_cam->hw.stream_off)
		ret = em_cam->hw.stream_off(0);

	if (ret == 0)
		em_cam->stop = 0;

	FNC_EXIT(ret)
	return ret;
}

static inline int cam_stream(int flag)
{
	/* flag 0 : OFF, 1 : ON */
	int ret = 0;
	FNC_ENTRY;

	/* if stream on */
	if (flag) {
		struct task_struct *th;

		if (em_cam->hw.stream_on) {
			ret = em_cam->hw.stream_on(0);
			if (ret) {
				d1b("stop\n");
				em_cam->stop = 1;
			}
		}

		mdelay(10);

	/* create and wake a thread */
		th = kthread_run(emxx_cam_thread, &em_cam, "emxx_cam_thread");
		if (IS_ERR(th)) {
			err("%s:  kernel_thread() failed\n", CAM_NAME);
			return PTR_ERR(th);
		}
		em_cam->th = th;


#if 1 /* XXX */
		if (warming_up) {
			schedule_timeout_uninterruptible(HZ);
			warming_up = 0;
		}
#endif

	} else {
		if (em_cam->th) {
			ret = kthread_stop(em_cam->th);
			em_cam->th = NULL;
		}
		cam_stop(0);
	}

	FNC_EXIT(ret)
	return ret;
}

static inline int cam_capture(int flag)
{
	int ret = 0;
	unsigned int index;
	unsigned long flags;
	FNC_ENTRY;

	if (em_cam->stop) {
		d3b(" stop\n");
		return -EIO;
	}

	if (emxx_camif_running(0)) {
#ifdef CAM_FPS_DEBUG
		dma_run_cnt++;
		if (dma_run_cnt > 200) {
			/* emxx_camif_reg_debug(); */
			dma_run_cnt = 0;
		}
#endif
		return ret;
	}

	if (!grab_peepq(&index))
		return ret;

	if (CAM_BUF_QUEUED != em_cam->grab->buff[index].state) {
		d3b("C3-%d-%d", index, em_cam->grab->buff[index].state);
		return ret;
	}

	if (em_cam->frames_active) {
		d3b("C4");
		return ret;
	}

	if (em_cam->hw.prepare) {
		em_cam->pre.actions = 1;
		ret = em_cam->hw.prepare(&em_cam->pre);
		if (ret) {
			d1b("stop\n");
			em_cam->stop = 1;
			d3b("C5");
			return ret;
		}
	}

	if (em_cam->pre.reset) {
		d1b("stop\n");
		em_cam->stop = 1;
		d3b("C6");
		return ret;
	}

#ifdef CAM_FPS_DEBUG
	d1b("C------:  %d\n", index);
	dma_run_cnt = 0;
#endif

	ret = emxx_camif_prepare(index);
	if (ret) {
		d1b("stop\n");
		em_cam->stop = 1;
		return ret;
	}

	if (em_cam->hw.trigger) {
		ret = em_cam->hw.trigger(0);
		if (ret) {
			d1b("stop\n");
			em_cam->stop = 1;
			return ret;
		}
	}

	spin_lock_irqsave(&em_cam->cam_lock, flags);
	em_cam->grab->buff[index].state = CAM_BUF_GRABBING;
	em_cam->frames_active = 1;
	ret = emxx_camif_capture(0);
	if (ret) {
		d1b("stop\n");
		em_cam->stop = 1;
		return ret;
	}

	spin_unlock_irqrestore(&em_cam->cam_lock, flags);

	FNC_EXIT(ret)
	return ret;
}

static inline int cam_sync(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	if (!em_cam->frames_active)
		return ret;

	/* assert(0 == emxx_camif_running(0)); */

	{
		int i = 0x1000;

		do {
			if (!emxx_camif_running(0))
				break;
			/* udelay(1); */
		} while (--i);
		/* module reset */

		/* assert(0 != i); */
	}

	if (em_cam->hw.sync) {
		em_cam->pre.actions = 1;
		ret = em_cam->hw.sync(&em_cam->pre);
		if (ret) {
			d1b("stop\n");
			em_cam->stop = 1;
			d3b("S-1\n");
			return ret;
		}
	}

	if (em_cam->pre.actions) {
		ret = emxx_camif_restart(0);
		{
			if (ret)
				d3b("S-2\n");
		}
		em_cam->setup = 1;
	}

	em_cam->frames_active = 0;

	FNC_EXIT(ret)
	return ret;
}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
static inline int cam_read_start(int flag)
{
	int i, ret = 0;
	ret = -EINVAL;
	FNC_ENTRY;

	if ((!em_cam->action && em_cam->grab) || em_cam->reset)
		cam_release_buffers(0);

	if (!em_cam->grab) {
		ret = cam_request_buffers(CAM_READ, EMXX_CAM_MAX_BUFNBRS,
					  V4L2_MEMORY_MMAP);
		if (ret)
			goto done_cam_read_start;

		ret = cam_choice_function(CAM_READ);

		if (ret) {
			cam_release_buffers(0);
			ret = -ENOMEM;
			goto done_cam_read_start;
		}

		em_cam->action = CAM_READ;
	}

	d1b(" re push all the buffer!\n");
	for (i = 0; em_cam->grab->max > i; i++)
		deq_pushq(em_cam->grab->cnt);

	ret = cam_stream(CAM_ON);

	if (ret)
		goto done_cam_read_start;

	em_cam->reading = 1;

	mutex_lock(&em_cam->frames_lock);
	ret = cam_capture(0);
	mutex_unlock(&em_cam->frames_lock);
	FNC_EXIT_N;

done_cam_read_start:
	return ret;
}
#endif

/*
 * Convert Functions
 */
static int emxx_cam_thread(void *p)
{
	int ret = 0;
	FNC_ENTRY;

	while (!kthread_should_stop()) {

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
		try_to_freeze();
#endif /* CONFIG_PM || CONFIG_DPM */

		ret = cam_grab_waiton(em_cam->grab, em_cam->deq_done,
				 0, 1, 1, LIMIT_TIME * HZ);

		if (-EINTR == ret) {
			mutex_lock(&em_cam->frames_lock);
			cam_sync(0);
			mutex_unlock(&em_cam->frames_lock);

			wake_up_interruptible(&em_cam->grab->proc_list);
			/*continue;*/
		} else if (-EIO == ret)   {
			mutex_lock(&em_cam->frames_lock);
			cam_sync(0);
			mutex_unlock(&em_cam->frames_lock);

			wake_up_interruptible(&em_cam->grab->proc_list);
			/*deq_grab_pullq();
			  continue;*/
		} else if (-ETIMEDOUT == ret)   {
			d1b("cam_waiton timeout.\n");
			/*continue;*/
		} else {
			/*deq_grab_pullq();*/
		}

		mutex_lock(&em_cam->frames_lock);
		ret = cam_sync(0);
		mutex_unlock(&em_cam->frames_lock);

		if (ret) {
			wake_up_interruptible(&em_cam->grab->proc_list);
			d1b("continue\n");
			continue;
		}

		mutex_lock(&em_cam->frames_lock);
		ret = cam_capture(0);
		mutex_unlock(&em_cam->frames_lock);

	}

	FNC_EXIT_N;
	return 0;
}


#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *emxx_cam_proc_root;

static int proc_emxx_cam_read(char *page, char **start, off_t off,
			       int count, int *eof, void *data)
{
	char *out = page;
	int len;
	FNC_ENTRY;

	/* IMPORTANT: This output MUST be kept under PAGE_SIZE
	 *            or we need to get more sophisticated. */
	out += sprintf(out, "%s : read-only\n-----------------------\n",
		       CAM_NAME);
	out += sprintf(out, "V4L Driver version:       %d.%d.%d\n",
			EMXX_CAM_MAJ_VER, EMXX_CAM_MIN_VER,
			EMXX_CAM_PATCH_VER);

	len = out - page;
	len -= off;

	if (len < count) {
		*eof = 1;
		if (len <= 0)
			return 0;
	} else
		len = count;

	*start = page + off;

	FNC_EXIT(len)
	return len;
}


static int proc_emxx_cam_write(struct file *file,
				const char __user *buf,
				unsigned long count, void *data)
{
	int retval = 0;
	FNC_ENTRY;

	if (mutex_lock_interruptible(&em_cam->lock))
		return -ERESTARTSYS;

	mutex_unlock(&em_cam->lock);

	FNC_EXIT_N;
	return retval;
}

static void create_proc_emxx_cam(struct emxx_cam *cam)
{
	char name[5 + 1 + 10 + 1];
	struct proc_dir_entry *ent;
	FNC_ENTRY;

	if (!emxx_cam_proc_root || !cam)
		return;

	snprintf(name, sizeof(name), "video%d", cam->vdev->minor);

	ent = create_proc_entry(name, S_IFREG | S_IRUGO | S_IWUSR,
				emxx_cam_proc_root);
	if (!ent)
		return;

	ent->data = cam;
	ent->read_proc = proc_emxx_cam_read;
	ent->write_proc = proc_emxx_cam_write;
	cam->proc_entry = ent;
	FNC_EXIT_N;
}

static void destroy_proc_emxx_cam(struct emxx_cam *cam)
{
	char name[5 + 1 + 10 + 1];
	FNC_ENTRY;

	if (!cam || !cam->proc_entry)
		return;

	snprintf(name, sizeof(name), "video%d", cam->vdev->minor);
	remove_proc_entry(name, emxx_cam_proc_root);
	cam->proc_entry = NULL;
	FNC_EXIT_N;
}

static void proc_emxx_cam_create(void)
{
	FNC_ENTRY;

	emxx_cam_proc_root = proc_mkdir("driver/cam", NULL);

	if (emxx_cam_proc_root)
		emxx_cam_proc_root->owner = THIS_MODULE;
	else
		err("Unable to initialise /proc/driver/cam\n");
	FNC_EXIT_N;
}

#if 1
static void proc_emxx_cam_destroy(void)
#else
static void __exit proc_emxx_cam_destroy(void)
#endif
{
	FNC_ENTRY;
	remove_proc_entry("driver/cam", NULL);
	FNC_EXIT_N;
}
#endif

/*
 * V4L2 Core Functions
 */
static int emxx_cam_grabs_handler(void *p)
{
	int ret = 0;
	__u32 bytesused, bpl;
	unsigned int index;
	FNC_ENTRY;

	index = grab_pullq();

	if (em_cam->status & (B_MAINOR | B_DMAERR)) {
		em_cam->grab->buff[index].state = CAM_BUF_BREAK;
		em_cam->grab->buff[index].bytesused = 0;
		ret = -EIO;
	} else {
		bytesused = get_sizeimage(em_cam->grab->fmt,
					  em_cam->width, em_cam->height, &bpl);
		em_cam->grab->buff[index].bytesused = bytesused;
	}
	em_cam->grab->buff[index].sequence = em_cam->sequence;
	do_gettimeofday(&em_cam->grab->buff[index].timestamp);
	em_cam->sequence++;

	wake_up_interruptible_all(&em_cam->grab->proc_list);

	FNC_EXIT(ret)
	return ret;
}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
static void emxx_cam_vm_open(struct vm_area_struct *vma)
{
	struct emxx_cam_mapping *map = vma->vm_private_data;
#ifdef CONFIG_VIDEO_EMXX
	void (*open)(struct vm_area_struct *area) = map->vm_ops->open;
#endif
	FNC_ENTRY;

#ifdef CONFIG_VIDEO_EMXX
	open(vma);
#endif
	d1b("vm_open %p [count=%d,vma=%08lx-%08lx]\n", map,
	    map->count, vma->vm_start, vma->vm_end);
	map->count++;
	em_cam->mapping++;
	FNC_EXIT_N;
}

static void emxx_cam_vm_close(struct vm_area_struct *vma)
{
	struct emxx_cam_mapping *map = vma->vm_private_data;
	struct emxx_cam_frames *frm = em_cam->grab;
	int i;
	FNC_ENTRY;

	d1b("vm_close %p [count=%d,vma=%08lx-%08lx]\n", map,
	    map->count, vma->vm_start, vma->vm_end);

	map->count--;
	if (0 == map->count) {
		mutex_lock(&em_cam->lock);
		for (i = 0; i < frm->max; i++) {
			if (frm->buff[i].map != map)
				continue;
			frm->buff[i].map   = NULL;
		}
		mutex_unlock(&em_cam->lock);
		kfree(map);
	}
	em_cam->mapping--;

#ifdef CONFIG_VIDEO_EMXX
	map->vm_ops->close(vma);
#endif
	FNC_EXIT_N;
	return;
}
#endif

/*
 * V4L2 Functions
 */
#define CAM_VERSION KERNEL_VERSION(EMXX_CAM_MAJ_VER, EMXX_CAM_MIN_VER, \
				   EMXX_CAM_PATCH_VER)

/* VIDIOC_QUERYCAP handler */
static int emxx_cam_vidioc_querycap(struct file *file, void *fh,
				     struct v4l2_capability *cap)
{
	int ret = 0;
	FNC_ENTRY;

	memset(cap, 0, sizeof(*cap));
	strlcpy(cap->driver, "cam", sizeof(cap->driver));
	strlcpy(cap->card, "emxx", sizeof(cap->card));
	strlcpy(cap->bus_info, "own", sizeof(cap->bus_info));
	cap->version = CAM_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	FNC_EXIT(ret)
	return ret;
}

/* Input handling */
static int emxx_cam_vidioc_enum_input(struct file *file, void *fh,
				       struct v4l2_input *inp)
{
	int ret = 0;
	FNC_ENTRY;

	if (inp->index != 0) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	memset(inp, 0, sizeof(*inp));
	strlcpy(inp->name, "cam", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_g_input(struct file *file, void *fh,
				    unsigned int *i)
{
	int ret = 0;
	FNC_ENTRY;

	*i = 0;

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_s_input(struct file *file, void *fh,
				    unsigned int i)
{
	int ret = 0;
	FNC_ENTRY;

	if (i > 0)
		ret = -EINVAL;

	FNC_EXIT(ret)
	return ret;
}

/* Control handling */
static int emxx_cam_vidioc_queryctrl(struct file *file, void *fh,
				      struct v4l2_queryctrl *a)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

#if 0
/* @@#if EMXX_CAM_NON_PMMU */
/* @@	if (a->id < EMXX_CID_IPU || */
/* @@	    a->id > EMXX_CID_IPU) */
/* @@#else */
/* @@	if (a->id < EMXX_CID_IPU || */
/* @@	    a->id > EMXX_CID_PMMU) */
/* @@#endif */
#else
	if (a->id < EMXX_CID_IPU  || a->id > EMXX_CID_PMMU)
#endif
	{
		ret = emxx_camif_vidioc_queryctrl(file, fh, a);
	} else {
		ctrl = cam_ctrl_by_id(a->id);
		*a = (NULL != ctrl) ? *ctrl : no_ctrl;
	}

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_querymenu(struct file *file, void *fh,
				      struct v4l2_querymenu *m)
{
	int ret = 0;
	FNC_ENTRY;

	memset(m->name, 0, sizeof(m->name));
	m->reserved = 0;

	ret = emxx_camif_vidioc_querymenu(file, fh, m);

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_g_ctrl(struct file *file, void *fh,
				   struct v4l2_control *c)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = cam_ctrl_by_id(c->id);
	if (NULL == ctrl) {
		ret = emxx_camif_vidioc_g_ctrl(file, fh, c);
		FNC_EXIT(ret)
		return ret;
	}
	switch (c->id) {
	case EMXX_CID_IPU:
		c->value = em_cam->ipu;
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_s_ctrl(struct file *file, void *fh,
				   struct v4l2_control *c)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = cam_ctrl_by_id(c->id);
	if (NULL == ctrl) {
		ret = emxx_camif_vidioc_s_ctrl(file, fh, c);
		FNC_EXIT(ret)
		return ret;
	}
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
		break;
	default:
		/* nothing */;
	}

	mutex_lock(&em_cam->lock);
	switch (c->id) {
	case EMXX_CID_IPU:
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
		if (em_cam->mapping || em_cam->reading || em_cam->streaming) {
#else
		if (em_cam->streaming) {
#endif
			warn("%s: s_ctrl IPU: streaming already exists.\n",
			     CAM_NAME);
			ret = -EBUSY;
			break;
		}
		if (em_cam->ipu != c->value) {
			if (V4L2_PIX_FMT_RGB24 == em_cam->fmt->pixelformat ||
			    V4L2_PIX_FMT_RGB565 == em_cam->fmt->pixelformat) {
				em_cam->reset = 1;
			}
			em_cam->ipu = (c->value) ? 1 : 0;
		}
		break;

	default:
		ret = -EINVAL;
	}
	mutex_unlock(&em_cam->lock);

	FNC_EXIT(ret)
	return ret;
}

/* VIDIOC_ENUM_FMT handlers */
static int emxx_cam_vidioc_enum_fmt_cap(struct file *file, void *fh,
					 struct v4l2_fmtdesc *f)
{
	int ret = 0;
	FNC_ENTRY;

#if 0
/* @@	if (f->index > NUM_CAM_FORMATS) */
#else
	if (f->index >= NUM_CAM_FORMATS)
#endif
	{
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	strlcpy(f->description, emxx_cam_formats[f->index].description,
	sizeof(f->description));
	f->pixelformat = emxx_cam_formats[f->index].pixelformat;

	FNC_EXIT(ret)
	return ret;
}

/* VIDIOC_TRY_FMT handlers */
static int emxx_cam_vidioc_try_fmt_cap(struct file *file, void *fh,
					struct v4l2_format *f)
{
	int ret = 0;
	const struct emxx_cam_fmt *fmt;
	FNC_ENTRY;

	if (f->fmt.pix.field != V4L2_FIELD_ANY &&
	    f->fmt.pix.field != V4L2_FIELD_NONE) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;

	fmt = format_by_pixelformat(f->fmt.pix.pixelformat);
	if (NULL == fmt) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	{
		u32 max_width  = camif_fmt_boundary(fmt, em_cam->c.width);
		u32 max_height = em_cam->c.height;
		u32 min_width  = camif_width_limit(fmt, max_width);
		u32 min_height = camif_height_limit(fmt, max_height);

		if (min_width > f->fmt.pix.width)
			f->fmt.pix.width = min_width;
		if (max_width < f->fmt.pix.width)
			f->fmt.pix.width = max_width;
		if (min_height > f->fmt.pix.height)
			f->fmt.pix.height = min_height;
		if (max_height < f->fmt.pix.height)
			f->fmt.pix.height = max_height;

		f->fmt.pix.width = camif_fmt_boundary(fmt, f->fmt.pix.width);
	}

	pix_format_set_size(&f->fmt.pix, fmt,
			    f->fmt.pix.width, f->fmt.pix.height);

	f->fmt.pix.colorspace = 0;
	f->fmt.pix.priv = 0;

	FNC_EXIT(ret)
	return ret;
}

/* VIDIOC_G_FMT handlers */
static int emxx_cam_vidioc_g_fmt_cap(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	int ret = 0;
	FNC_ENTRY;

	f->fmt.pix.width        = em_cam->width;
	f->fmt.pix.height       = em_cam->height;
	f->fmt.pix.field        = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat  = em_cam->fmt->pixelformat;

	pix_format_set_size(&f->fmt.pix, em_cam->fmt,
				em_cam->width, em_cam->height);

	FNC_EXIT(ret)
	return ret;
}

/* VIDIOC_S_FMT handlers */
static int emxx_cam_vidioc_s_fmt_cap(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	int ret = 0;
	FNC_ENTRY;

	ret = emxx_cam_vidioc_try_fmt_cap(file, fh, f);
	if (ret < 0) {
		FNC_EXIT(ret)
		return ret;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	if (em_cam->mapping || em_cam->reading || em_cam->streaming) {
#else
	if (em_cam->streaming) {
#endif
		warn("%s: s_fmt: streaming already exists.\n", CAM_NAME);
		ret = -EBUSY;
	} else {
		const struct emxx_cam_fmt *fmt;

		mutex_lock(&em_cam->lock);
		fmt = format_by_pixelformat(f->fmt.pix.pixelformat);
		if (em_cam->fmt->pixelformat != fmt->pixelformat) {
			em_cam->reset = check_reset_fmt(em_cam->fmt, fmt);
			em_cam->setup = 1;
		}
		if (em_cam->width != f->fmt.pix.width)
			em_cam->setup = 1;
		if (em_cam->height != f->fmt.pix.height)
			em_cam->setup = 1;
		em_cam->fmt    = fmt;
		em_cam->width  = f->fmt.pix.width;
		em_cam->height = f->fmt.pix.height;
		mutex_unlock(&em_cam->lock);
	}

	FNC_EXIT(ret)
	return ret;
}

/* Crop ioctls */
static int emxx_cam_vidioc_cropcap(struct file *file, void *fh,
				    struct v4l2_cropcap *c)
{
	int ret = 0;
	FNC_ENTRY;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != c->type) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	mutex_lock(&em_cam->lock);
	c->bounds  = em_cam->bounds;
	c->defrect = em_cam->bounds;
	c->pixelaspect.numerator   = 1;
	c->pixelaspect.denominator = 1;
	mutex_unlock(&em_cam->lock);

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_g_crop(struct file *file, void *fh,
				   struct v4l2_crop *crop)
{
	int ret = 0;
	FNC_ENTRY;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != crop->type) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	mutex_lock(&em_cam->lock);
	crop->c = em_cam->c;
	mutex_unlock(&em_cam->lock);

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_s_crop(struct file *file, void *fh,
				   struct v4l2_crop *crop)
{
	int ret = 0;
	__s32 width, height;
	FNC_ENTRY;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != crop->type) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	if (em_cam->mapping || em_cam->reading || em_cam->streaming) {
#else
	if (em_cam->streaming) {
#endif
		warn("%s: s_crop: streaming already exists.\n", CAM_NAME);
		return -EBUSY;
	}

	crop->c.left = crop->c.left >> 1;
	crop->c.left = crop->c.left << 1;

	width = (em_cam->bounds.left + em_cam->bounds.width)
		- abs(em_cam->bounds.left - crop->c.left);
	if (width < crop->c.width)
		crop->c.width = width;
	crop->c.width = camif_fmt_boundary(em_cam->fmt, crop->c.width);

	height = (em_cam->bounds.top + em_cam->bounds.height)
		- abs(em_cam->bounds.top - crop->c.top);
	if (height < crop->c.height)
		crop->c.height = height;

	if (em_cam->bounds.left > crop->c.left)
		crop->c.left = em_cam->bounds.left;

	if (em_cam->bounds.top > crop->c.top)
		crop->c.top = em_cam->bounds.top;

	if ((crop->c.left >= em_cam->bounds.left + em_cam->bounds.width)  ||
	    (crop->c.top  >= em_cam->bounds.top  + em_cam->bounds.height) ||
	    (crop->c.left + crop->c.width  <= em_cam->bounds.left)     ||
	    (crop->c.top  + crop->c.height <= em_cam->bounds.top)      ||
	    crop->c.width  <= 0                                    ||
	    crop->c.height <= 0
	) {
		warn("%s: s_crop: crop out of range.\n", CAM_NAME);
		return -EINVAL;
	}

	mutex_lock(&em_cam->lock);
	if (crop->c.width < em_cam->width) {
		em_cam->width = crop->c.width;
		em_cam->setup = 1;
	}

#if 1
	else if (camif_width_ratio_limit(em_cam->fmt, crop->c.width)
		 > em_cam->width) {
		crop->c.width = 1023 * em_cam->width / 64;
		crop->c.width = camif_fmt_boundary(em_cam->fmt, crop->c.width);
	}
#endif

	if (crop->c.height < em_cam->height) {
		em_cam->height = crop->c.height;
		em_cam->setup = 1;
	}
#if 1
	else if (camif_height_ratio_limit(em_cam->fmt, crop->c.height)
		 > em_cam->height)
		crop->c.height = 1023 * em_cam->height / 64;
#endif

	if (em_cam->c.top    != crop->c.top
	    || em_cam->c.left   != crop->c.left
	    || em_cam->c.width  != crop->c.width
	    || em_cam->c.height != crop->c.height) {
		em_cam->c = crop->c;
		em_cam->setup = 1;
	}

	mutex_unlock(&em_cam->lock);

	FNC_EXIT(ret)
	return ret;
}

/* Buffer handlers */
static int emxx_cam_vidioc_reqbufs(struct file *file, void *fh,
				    struct v4l2_requestbuffers *req)
{
	struct emxx_cam_private *priv = fh;
	int ret = 0;
	unsigned int count;
	FNC_ENTRY;

	if (no_active(priv->number)) {
		warn("%s: reqbufs: no active.\n", CAM_NAME);
		return -EINVAL;
	}

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != req->type) {
		warn("%s: reqbufs: queue type invalid.\n", CAM_NAME);
		return -EINVAL;
	}

	if (1 > req->count) {
		warn("%s: reqbufs: count invalid (%d).\n",
			CAM_NAME, req->count);
		return -EINVAL;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	if (V4L2_MEMORY_MMAP != req->memory
	    && V4L2_MEMORY_USERPTR != req->memory) {
#else
	if (V4L2_MEMORY_USERPTR != req->memory) {
#endif
		warn("%s: reqbufs: memory type invalid.\n", CAM_NAME);
		return -EINVAL;
	}

	if (em_cam->streaming) {
		warn("%s: reqbufs: streaming already exists.\n", CAM_NAME);
		return -EBUSY;
	}
	em_cam->reading = 0;
#if EMXX_CAM_USE_MMAP/* need support V4L2_MEMORY_MMAP */
	if (em_cam->mapping) {
		warn("%s: reqbufs: buffers already mapping.\n", CAM_NAME);
		return -EBUSY;
	}
#endif

	mutex_lock(&em_cam->lock);

	if (em_cam->action) {
		cam_stream(CAM_OFF);
		kfifo_reset(em_cam->grab->enq);
		kfifo_reset(em_cam->deq_done);
	}

	count = req->count;

	if (EMXX_CAM_MAX_BUFNBRS < count)
		count = EMXX_CAM_MAX_BUFNBRS;

	ret = cam_request_buffers(CAM_MMAP, count, req->memory);

	if (ret) {
		warn("%s: mmap setup returned %d.\n", CAM_NAME, ret);
		goto done_vidioc_reqbufs;
	}

	ret = cam_choice_function(CAM_MMAP);
	if (ret) {
		cam_release_buffers(0);
		ret = -ENOMEM;
		goto done_vidioc_reqbufs;
	}

	req->count = em_cam->grab->max;

done_vidioc_reqbufs:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_querybuf(struct file *file, void *fh,
				     struct v4l2_buffer *b)
{
	int ret = 0;
	FNC_ENTRY;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != b->type) {
		warn("%s: querybuf: wrong type.\n", CAM_NAME);
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	if (!em_cam->grab || em_cam->action || em_cam->reset) {
		warn("%s: querybuf: buffer is null.\n", CAM_NAME);
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	if (0 > b->index || em_cam->grab->max <= b->index) {
		warn("%s: querybuf: index out of range.\n", CAM_NAME);
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	cam_buffer_status(b, b->index, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_qbuf(struct file *file, void *fh,
				 struct v4l2_buffer *b)
{
	struct emxx_cam_private *priv = fh;
	int ret = 0;
	__u32 bytesused, bpl;
	struct emxx_cam_buffer *cdb;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);

	if (no_active(priv->number)) {
		warn("%s: qbuf: no active.\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	if (em_cam->reading) {
		warn("%s: qbuf: Reading running...\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}
#endif

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != b->type) {
		warn("%s: qbuf: Wrong type.\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}

	if (!em_cam->grab || em_cam->action || em_cam->reset) {
		warn("%s: qbuf: buffer is null.\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}

	if (0 > b->index || em_cam->grab->max <= b->index) {
		warn("%s: qbuf: index out of range.\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}

	cdb = &em_cam->grab->buff[b->index];

	if (em_cam->grab->memory != b->memory) {
		warn("%s: qbuf: memory type is wrong.\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}

	if (cdb->state == CAM_BUF_QUEUED || cdb->state == CAM_BUF_GRABBING) {
		warn("%s: qbuf: buffer is already queued or active.\n",
			CAM_NAME);
		goto done_vidioc_qbuf;
	}

	/* XXX if (b->flags & V4L2_BUF_FLAG_INPUT) XXX */

	bytesused = get_sizeimage(em_cam->grab->fmt,
				  em_cam->width, em_cam->height, &bpl);

	switch (b->memory) {
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	case V4L2_MEMORY_MMAP:
		if (0 == cdb->vadr) {
			err("%s: qbuf: mmap requested"
			    "but buffer addr is zero!\n", CAM_NAME);
			goto done_vidioc_qbuf;
		}
		break;
#endif
	case V4L2_MEMORY_USERPTR:
		if (b->length < bytesused) {
			warn("%s: qbuf: buffer length is not enough.\n",
				CAM_NAME);
			goto done_vidioc_qbuf;
		}

		if (!b->m.userptr)
			goto done_vidioc_qbuf;

		if (cdb->m.userptr && cdb->m.userptr != b->m.userptr)
			release_userptr(cdb);

		if (!cdb->m.userptr) {
			if (request_userptr(cdb, b->m.userptr, b->length))
				goto done_vidioc_qbuf;
		}
		/* debug_dump(snprintf_buffs, cdb, 1, V4L2_MEMORY_USERPTR); */
		break;
	default:
		warn("%s: qbuf: wrong memory type.\n", CAM_NAME);
		goto done_vidioc_qbuf;
	}

	b->field = V4L2_FIELD_NONE;

	b->flags |= V4L2_BUF_FLAG_QUEUED;
	b->flags &= ~V4L2_BUF_FLAG_DONE;

	deq_pushq(b->index);

done_vidioc_qbuf:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_dqbuf(struct file *file, void *fh,
				  struct v4l2_buffer *b)
{
	struct emxx_cam_private *priv = fh;
	unsigned int len, index;
	int ret = 0;
	unsigned int index_pull;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);
	ret = -EINVAL;

	if (no_active(priv->number)) {
		warn("%s: dqbuf: no active.\n", CAM_NAME);
		goto done_vidioc_dqbuf;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	ret = -EBUSY;

	if (em_cam->reading) {
		warn("%s: dqbuf: Reading running...\n", CAM_NAME);
		goto done_vidioc_dqbuf;
	}
#endif

	ret = -EINVAL;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != b->type) {
		warn("%s: dqbuf: Wrong type.\n", CAM_NAME);
		goto done_vidioc_dqbuf;
	}

	if (!em_cam->grab || em_cam->action || em_cam->reset) {
		warn("%s: dqbuf: buffer is null.\n", CAM_NAME);
		goto done_vidioc_dqbuf;
	}

	len = deq_done_peepq(&index);

	if (!len) {
		warn("%s: dqbuf: no qbuf.\n", CAM_NAME);
		goto done_vidioc_dqbuf;
	}

	/* debug_snprintf("%s(len : %d, index : %d)\n",
				 __FUNCTION__, len, index); */

	ret = cam_waiton(em_cam->grab, em_cam->deq_done,
			 file->f_flags & O_NONBLOCK, 1, 0, LIMIT_TIME * HZ);


	if (ret)
		goto done_vidioc_dqbuf;

	memset(b, 0, sizeof(*b));
	cam_buffer_status(b, index, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	b->m.phys_add.PhysAddr_Y = em_cam->grab->buff[index].padr;
	b->m.phys_add.PhysAddr_UV = b->m.phys_add.PhysAddr_Y +
					(em_cam->width * em_cam->height);
	b->m.phys_add.PhysAddr_V = b->m.phys_add.PhysAddr_UV +
					(em_cam->width * em_cam->height / 4);


	index_pull = deq_done_pullq();
	em_cam->grab->buff[index_pull].state = CAM_BUF_IDLE;


done_vidioc_dqbuf:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

/* Stream on/off */
static int emxx_cam_vidioc_streamon(struct file *file, void *fh,
				     enum v4l2_buf_type i)
{
	struct emxx_cam_private *priv = fh;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);

	ret = -EINVAL;

	if (no_active(priv->number)) {
		warn("%s: streamon: no active.\n", CAM_NAME);
		goto done_vidioc_streamon;
	}

	if (!em_cam->grab || em_cam->action || em_cam->reset) {
		warn("%s: streamon: buffer is null.\n", CAM_NAME);
		goto done_vidioc_streamon;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	ret = -EBUSY;

	if (em_cam->reading) {
		warn("%s: streamon: Reading running...\n", CAM_NAME);
		goto done_vidioc_streamon;
	}
#endif

	ret = 0;

	if (em_cam->streaming)
		goto done_vidioc_streamon;

	ret = cam_stream(CAM_ON);

	if (ret)
		goto done_vidioc_streamon;

	em_cam->streaming = 1;

	mutex_lock(&em_cam->frames_lock);
	ret = cam_capture(0);
	mutex_unlock(&em_cam->frames_lock);
	/*d4b("\n\nstream on\n");
	emxx_camif_reg_debug();*/

done_vidioc_streamon:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_vidioc_streamoff(struct file *file, void *fh,
				      enum v4l2_buf_type i)
{
	struct emxx_cam_private *priv = fh;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);

	ret = -EINVAL;

	if (no_active(priv->number)) {
		warn("%s: streamoff: no active.\n", CAM_NAME);
		goto done_vidioc_streamoff;
	}

	if (!em_cam->streaming) {
		warn("%s: streamoff: no streamon.\n", CAM_NAME);
		goto done_vidioc_streamoff;
	}

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	ret = -EBUSY;

	if (em_cam->reading) {
		warn("%s: streamoff: Reading running...\n", CAM_NAME);
		goto done_vidioc_streamoff;
	}
#endif

	ret = 0;

	cam_stream(CAM_OFF);
	kfifo_reset(em_cam->grab->enq);
	kfifo_reset(em_cam->deq_done);
	/*release buffers?*/
	if (em_cam->grab->buff)
		cam_release_buffers(0);

	em_cam->streaming = 0;
	/*d4b("\n\nstream off\n");
	emxx_camif_reg_debug();*/

done_vidioc_streamoff:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

static unsigned int emxx_cam_poll(struct file *file, poll_table *wait)
{
	struct emxx_cam_private *priv = file->private_data;
	unsigned int len, index;
	unsigned int rc = 0;
	int i = 0;
	FNC_ENTRY;

	if (no_active(priv->number)) {
		warn("%s: poll: no active.\n", CAM_NAME);
		FNC_EXIT(POLLERR)
		return POLLERR;
	}

	mutex_lock(&em_cam->lock);

	len = 0;

#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	if (em_cam->mapping ||  em_cam->userptr) {
		if (!em_cam->streaming) {
			warn("%s: poll: no stream.\n", CAM_NAME);
			rc = POLLERR;
		}
	} else {
		if (!em_cam->reading) {
			int ret;
			ret = cam_read_start(0);
			if (ret) {
				warn("%s: read setup returned %d.\n",
				     CAM_NAME, ret);
				rc = POLLERR;
			}
		}
	}
#else
	if (em_cam->userptr && !em_cam->streaming) {
		warn("%s: poll: no stream.\n", CAM_NAME);
		rc = POLLERR;
	}
#endif

	len = deq_done_peepq(&index);

	if (!len)
		rc = POLLERR;

	if (0 == rc) {
		struct emxx_cam_buffer *cdb = &em_cam->grab->buff[index];
		d1b("chk poll %d\n", cdb->state);

		switch (cdb->state) {
		case CAM_BUF_BREAK:
			rc = POLLERR;
			break;
		case CAM_BUF_IDLE:
		case CAM_BUF_QUEUED:
		case CAM_BUF_GRABBING:
			poll_wait(file, &em_cam->grab->proc_list, wait);

			if (CAM_BUF_BREAK == cdb->state) {
				rc = POLLERR;
				break;
			}
			if (CAM_BUF_DONE == cdb->state) {
				rc = POLLIN | POLLRDNORM;
				break;
			}
			for (i = 0; i < em_cam->grab->max; i++)
				d2b("%d, %d\n", i, em_cam->grab->buff[i].state);
			d2b("index: %d\n", index);
			d2b("grab--in: %d, out: %d, max: %d\n",
				em_cam->grab->enq->in,
				em_cam->grab->enq->out,
				em_cam->grab->max);
			d2b("deq_done--in: %d, out: %d\n",
				em_cam->deq_done->in,
				em_cam->deq_done->out);
#ifdef CAM_FPS_DEBUG
			d2b("deq_done_push_cnt: %d, deq_done_pull_cnt:%d\n",
				deq_done_push_cnt, deq_done_pull_cnt);
			d2b("grab_push_cnt: %d, grab_pull_cnt:%d\n",
				grab_push_cnt, grab_pull_cnt);
			d2b("vsync_detect_cnt: %d, transmit_end_cnt:%d\n",
				vsync_detect_cnt, transmit_end_cnt);
			emxx_camif_buffer_debug(index);
		    /*emxx_camif_reg_debug();*/
#endif
			break;
		case CAM_BUF_DONE:
			d1b("chk poll 3 %d\n", CAM_BUF_QUEUED);
			rc = POLLIN | POLLRDNORM;
			break;
		}

	}

	mutex_unlock(&em_cam->lock);
	FNC_EXIT(rc)
	return rc;
}

#if EMXX_CAM_USE_MMAP/* need support V4L2_MEMORY_MMAP */
static ssize_t emxx_cam_read(struct file *file, char *buf,
			      size_t count, loff_t *ppos)
{
	struct emxx_cam_private *priv = file->private_data;
	int ret = 0;
	unsigned int len, index, next;
	__u32 cnt, bpl;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);

	if (!count)
		goto done_cam_read;

	ret = -EINVAL;

	if (no_active(priv->number)) {
		warn("%s: read: no active.\n", CAM_NAME);
		goto done_cam_read;
	}

	if (em_cam->streaming) {
		warn("%s: read: streaming already exists.\n", CAM_NAME);
		goto done_cam_read;
	}

	if (em_cam->mapping) {
		warn("%s: read: mapping done.\n", CAM_NAME);
		goto done_cam_read;
	}

	if (em_cam->userptr) {
		warn("%s: read: userptr done.\n", CAM_NAME);
		goto done_cam_read;
	}

	if (!em_cam->reading) {
		ret = cam_read_start(0);
		if (ret) {
			warn("%s: read setup returned %d.\n", CAM_NAME, ret);
			goto done_cam_read;
		}
	}

	cnt = get_sizeimage(em_cam->grab->fmt,
			em_cam->width, em_cam->height, &bpl);

	if (cnt > count) {
		warn("%s: read: count < %d.\n", CAM_NAME, cnt);
		goto done_cam_read;
	}

	ret = -EFAULT;

	len = deq_done_peepq(&index);

	if (!len) {
		err("%s: read: no deq_done.\n", CAM_NAME);
		goto done_cam_read;
	}

	ret = cam_waiton(em_cam->grab, em_cam->deq_done,
			 file->f_flags & O_NONBLOCK, 1, 0, LIMIT_TIME * HZ);

	if (ret)
		goto done_cam_read;

	ret = -EFAULT;

	if (copy_to_user(buf, em_cam->grab->buff[index].vadr, cnt)) {
		err("%s: read: copy_to_user fault.\n", CAM_NAME);
		goto done_cam_read;
	}


	em_cam->grab->buff[index].state = CAM_BUF_IDLE;

	next = deq_done_pullq();

	assert(index == next);

	deq_pushq(next);

	mutex_lock(&em_cam->frames_lock);
	ret = cam_capture(0);
	mutex_unlock(&em_cam->frames_lock);

	if (!ret)
		ret = cnt;

done_cam_read:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

static struct vm_operations_struct cam_vm_ops = {
	.open           = emxx_cam_vm_open,
	.close          = emxx_cam_vm_close,
};

static int emxx_cam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct emxx_cam_private *priv = file->private_data;
	struct emxx_cam_frames *frm = em_cam->grab;
	struct emxx_cam_mapping *map;
	unsigned int first, last, size, i;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);
	ret = -EINVAL;
	if (no_active(priv->number)) {
		warn("%s: mmap: no active.\n", CAM_NAME);
		goto done_cam_mmap;
	}

	if (!(vma->vm_flags & VM_WRITE)) {
		warn("%s: mmap app bug: PROT_WRITE please.\n", CAM_NAME);
		goto done_cam_mmap;
	}

	if (!(vma->vm_flags & VM_SHARED)) {
		warn("%s: mmap app bug: MAP_SHARED please.\n", CAM_NAME);
		goto done_cam_mmap;
	}

	if (!frm) {
		warn("%s: mmap: buffer is null.\n", CAM_NAME);
		goto done_cam_mmap;
	}

	if (V4L2_MEMORY_MMAP != frm->memory) {
		warn("%s: mmap: memory type invalid.\n", CAM_NAME);
		goto done_cam_mmap;
	}

	/* look for first buffer to map */
	for (first = 0; first < frm->max; first++) {
		if (frm->buff[first].m.offset == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}
	if (frm->max == first) {
		warn("%s: mmap app bug: offset invalid [offset=0x%lx]\n",
		     CAM_NAME, (vma->vm_pgoff << PAGE_SHIFT));
		goto done_cam_mmap;
	}

	/* look for last buffer to map */
	for (size = 0, last = first; last < frm->max; last++) {
		if (frm->buff[last].map) {
			ret = -EBUSY;
			goto done_cam_mmap;
		}
		size += frm->blocksize;
		if (size == (vma->vm_end - vma->vm_start))
			break;
	}
	if (frm->max == last) {
		warn("%s: mmap app bug: size invalid [size=0x%lx]\n",
		     CAM_NAME, (vma->vm_end - vma->vm_start));
		goto done_cam_mmap;
	}

	/* create mapping */
	ret = -ENOMEM;
	map = kzalloc(sizeof(*map), GFP_KERNEL);

	if (NULL == map)
		goto done_cam_mmap;

	vma->vm_pgoff = vma->vm_pgoff + (CAMERA_FRAME_BASE >> PAGE_SHIFT);

#ifdef CONFIG_VIDEO_EMXX
	if (emxx_v4l2_mmap(vma)) {
		ret = -EAGAIN;
		goto  done_cam_mmap;
	}
#else
	/* Accessing memory will be done non-cached. */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
		vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		ret = -EAGAIN;
		goto  done_cam_mmap;
	}
#endif
	for (i = first; i <= last; i++)
		frm->buff[i].map = map;

	map->start    = vma->vm_start;
	map->end      = vma->vm_end;
#ifdef CONFIG_VIDEO_EMXX
	map->vm_ops   = vma->vm_ops;
#endif
	vma->vm_ops   = &cam_vm_ops;
#ifndef CONFIG_VIDEO_EMXX
	vma->vm_flags |= VM_DONTEXPAND | VM_RESERVED;
	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */
#endif
	vma->vm_private_data = map;

#ifdef CONFIG_VIDEO_EMXX
	map->count++;
	em_cam->mapping++;
#else
	emxx_cam_vm_open(vma);
#endif

	d1b("mmap %p: %08lx-%08lx pgoff %08lx bufs %d-%d\n",
	    map, vma->vm_start, vma->vm_end, vma->vm_pgoff, first, last);
	ret = 0;

done_cam_mmap:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}
#endif

#define CAM_OPEN_MAX 32
/*
 open function
*/

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
static int emxx_cam_open(struct inode *inode, struct file *file)
#else
static int emxx_cam_open(struct file *file)
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)) */
{
	int ret = 0, i;
	struct emxx_cam_private *priv;
	__u32 nouse;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);

	/*check whether the open number > 32*/
	if (CAM_OPEN_MAX <= em_cam->open_count) {
		ret = -EBUSY;
		goto done_cam_open;
	}

	/*alloc memory space*/
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto done_cam_open;
	}

	/*check the used_number
	  make the nouse as new number which not been used.
	  used_number + (1 << x)*/
	for (i = 0; i < CAM_OPEN_MAX; i++) {
		nouse = 0x01 << i;
		if (!(em_cam->used_number & nouse))
			break;
	}

	/* if current no active number*/
	if (0 == em_cam->active_number) {
		/*if em_cam->action return value is not 0, error*/
		assert(!em_cam->action)
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
		assert(!em_cam->reading)
#endif
		assert(!em_cam->streaming)

		em_cam->status = 0;
		em_cam->setup  = 1;
		em_cam->reset  = 1;
		em_cam->stop   = 0;
		em_cam->frames_active = 0;

		em_cam->th     = NULL;

		kfifo_reset(em_cam->deq_done);
		kfifo_reset(em_cam->grab->enq);

		/* make the active_number as the current open number */
		em_cam->active_number = nouse;
	}

	/*
	if it is the first time open operation
	need to startup the camera IF and sensor hw
	*/
	if (0 == em_cam->open_count) {
		/*startup cam if in EMEV*/
		ret = emxx_camif_startup();
		if (ret) {
			kfree(priv);
			priv = NULL;
			err(" emxx_camif_startup failed\n");
			goto done_cam_open;
		}

#ifdef CAM_FPS_DEBUG
		transmit_end_cnt = 0;
		vsync_detect_cnt = 0;
		deq_done_push_cnt = 0;
		deq_done_pull_cnt = 0;
		grab_push_cnt = 0;
		grab_pull_cnt = 0;
#endif
		/*sensor hw startup*/
		if (em_cam->hw.startup) {
			ret = em_cam->hw.startup(0);
			if (ret) {
				kfree(priv);
				priv = NULL;
				goto done_cam_open;
			}
		}

		if (em_cam->hw.sync) {
			em_cam->pre.actions = 0;
			ret = em_cam->hw.sync(&em_cam->pre);
			if (ret) {
				if (em_cam->hw.shutdown)
					em_cam->hw.shutdown(0);
				kfree(priv);
				priv = NULL;
				goto done_cam_open;
			}
		}

		emxx_camif_unreset();

#ifdef CONFIG_EMXX_ANDROID
		wake_lock(&em_cam->idle_lock); /* lock suspend */
#endif /* CONFIG_EMXX_ANDROID */
	}

	/* take the current open number */
	priv->number = nouse;

	/* add the current open number to used number*/
	em_cam->used_number |= priv->number;

	/* take the priv as the open file's private data */
	file->private_data = priv;

	/* open number add 1*/
	em_cam->open_count++;

done_cam_open:
	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}


/*
 close function
*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
static int emxx_cam_close(struct inode *inode, struct file *file)
#else
static int emxx_cam_close(struct file *file)
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)) */
{
	struct emxx_cam_private *priv = file->private_data;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&em_cam->lock);
	em_cam->open_count--;/* open count del 1*/

	/* if the file private number is the current active number,
	do following operation*/
	if (em_cam->active_number == priv->number) {

		cam_stream(CAM_OFF);

		em_cam->action    = 0;
		em_cam->reading   = 0;
		em_cam->streaming = 0;
		em_cam->userptr   = 0;

		/*release buffers?*/
		if (em_cam->grab->buff)
			cam_release_buffers(0);

		em_cam->active_number = 0;
	}

	/* if no camera opened, then shutdown the CAM module IF and sensor hw*/
	if (0 == em_cam->open_count) {
		emxx_camif_shutdown(0);

		if (em_cam->hw.shutdown)
			em_cam->hw.shutdown(0);

#ifdef CONFIG_EMXX_ANDROID
		wake_unlock(&em_cam->idle_lock); /* unlock suspend */
#endif /* CONFIG_EMXX_ANDROID */
	}

	/* d1b("chk used_number : 0x%08x number : 0x%08x \n",
					cam->used_number, ~priv->number); */
	em_cam->used_number &= ~priv->number;

	kfree(priv);
	priv = NULL;

	mutex_unlock(&em_cam->lock);
	FNC_EXIT(ret)
	return ret;
}

/*
 * register V4L2 device
 */

static int video_nr = -1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
static const struct file_operations emxx_cam_fops = {
	.llseek         = no_llseek,
#else
static const struct v4l2_file_operations emxx_cam_fops = {
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)) */
	.owner          = THIS_MODULE,
	.open           = emxx_cam_open,
	.release        = emxx_cam_close,
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	.read           = emxx_cam_read,
#endif
	.poll           = emxx_cam_poll,
	.ioctl          = video_ioctl2,
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	.mmap           = emxx_cam_mmap,
#endif
};

static const struct v4l2_ioctl_ops emxx_cam_ioctl_ops = {
	.vidioc_querycap        = emxx_cam_vidioc_querycap,
	.vidioc_enum_input      = emxx_cam_vidioc_enum_input,
	.vidioc_g_input         = emxx_cam_vidioc_g_input,
	.vidioc_s_input         = emxx_cam_vidioc_s_input,
	.vidioc_queryctrl       = emxx_cam_vidioc_queryctrl,
	.vidioc_querymenu       = emxx_cam_vidioc_querymenu,
	.vidioc_g_ctrl          = emxx_cam_vidioc_g_ctrl,
	.vidioc_s_ctrl          = emxx_cam_vidioc_s_ctrl,
	.vidioc_enum_fmt_vid_cap        = emxx_cam_vidioc_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap   = emxx_cam_vidioc_g_fmt_cap,
	.vidioc_s_fmt_vid_cap   = emxx_cam_vidioc_s_fmt_cap,
	.vidioc_try_fmt_vid_cap         = emxx_cam_vidioc_try_fmt_cap,
	.vidioc_cropcap         = emxx_cam_vidioc_cropcap,
	.vidioc_g_crop          = emxx_cam_vidioc_g_crop,
	.vidioc_s_crop          = emxx_cam_vidioc_s_crop,
	.vidioc_reqbufs         = emxx_cam_vidioc_reqbufs,
	.vidioc_querybuf        = emxx_cam_vidioc_querybuf,
	.vidioc_qbuf            = emxx_cam_vidioc_qbuf,
	.vidioc_dqbuf           = emxx_cam_vidioc_dqbuf,
	.vidioc_streamon        = emxx_cam_vidioc_streamon,
	.vidioc_streamoff       = emxx_cam_vidioc_streamoff,
};

static struct video_device emxx_cam_template = {
	.ioctl_ops              = &emxx_cam_ioctl_ops,
	.fops                   = &emxx_cam_fops,
	.name                   = CAM_NAME,
	.minor                  = -1,
	.release                = video_device_release,
};

/*
 init cam members, alloc memory space, register the cam to vedio system/layer
 create proc access node
*/
static int emxx_cam_register(int flag)
{
	int ret = -ENOMEM;
	FNC_ENTRY;

	/*allocate memory*/
	em_cam = kzalloc(sizeof(*em_cam), GFP_KERNEL);

	if (NULL == em_cam) {
		err("%s: only one device allowed!\n", CAM_NAME);
		goto out_emxx_cam_register;
	}

	/*init lock member for emxx_cam structure*/
	mutex_init(&em_cam->lock);
	mutex_init(&em_cam->frames_lock);

#ifdef CONFIG_EMXX_ANDROID
	/* Android power management wake lock init */
	/* suspend control*/
	wake_lock_init(&em_cam->idle_lock, WAKE_LOCK_IDLE, CAM_NAME);
#endif /* CONFIG_EMXX_ANDROID */

	/*allocate memory space*/
	em_cam->deq_done = kfifo_alloc(sizeof(int) * EMXX_CAM_MAX_BUFNBRS,
				    GFP_KERNEL, &em_cam->deq_done_lock);

	if (IS_ERR(em_cam->deq_done)) {
		err("%s: deq_done fifo allocation failed\n", CAM_NAME);
		goto out_kfree_cam;
	}

	/*allocate memory space*/
	em_cam->grab = kzalloc(sizeof(*em_cam->grab), GFP_KERNEL);

	if (NULL == em_cam->grab) {
		err("%s: grab buffer allocation failed\n", CAM_NAME);
		goto out_kfifo_free_deq_done;
	}

	init_waitqueue_head(&em_cam->grab->proc_list);

	/*allocate memory space*/
	em_cam->grab->enq = kfifo_alloc(sizeof(int) * EMXX_CAM_MAX_BUFNBRS,
					GFP_KERNEL, &em_cam->grab->enq_lock);

	if (IS_ERR(em_cam->grab->enq)) {
		err("%s: grab fifo allocation failed\n", CAM_NAME);
		goto out_kfree_grab;
	}

	/*allocate memory space for the video device member in 'cam'*/
	em_cam->vdev = video_device_alloc();

	if (NULL == em_cam->vdev) {
		err("%s: video_device_alloc() failed!\n", CAM_NAME);
		goto out_kfifo_free_grab;
	}

	/*copy the inforamtion to em_cam->vdev,construct this member structure
	 why not setting in directly? */
	memcpy(em_cam->vdev, &emxx_cam_template, sizeof(emxx_cam_template));
	/* connect cam with the cam->video_dev->driver_data, *
	 * make the driver and device as one gather          */
	video_set_drvdata(em_cam->vdev, em_cam);

	/* register v4l device */
	ret = video_register_device(em_cam->vdev, VFL_TYPE_GRABBER, video_nr);

	if (ret) {
		err("%s: video_register_device() failed!\n", CAM_NAME);
		goto out_video_device_release;
	}

	/* initialize the spinlock */
	spin_lock_init(&em_cam->cam_lock);

#ifdef CONFIG_PROC_FS
	/* filesystem proc node create, make read and *
	 * write function for R/W available           */
	create_proc_emxx_cam(em_cam);
#endif

	FNC_EXIT(ret)
	return 0;

out_video_device_release:
	video_device_release(em_cam->vdev);
out_kfifo_free_grab:
	kfifo_free(em_cam->grab->enq);
out_kfree_grab:
	kfree(em_cam->grab);
	em_cam->grab = NULL;
out_kfifo_free_deq_done:
	kfifo_free(em_cam->deq_done);
out_kfree_cam:
	kfree(em_cam);
	em_cam = NULL;
out_emxx_cam_register:

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_unregister(void)
{
	int ret = 0;
	FNC_ENTRY;

	video_unregister_device(em_cam->vdev);

#ifdef CONFIG_PROC_FS
	destroy_proc_emxx_cam(em_cam);
#endif

	kfifo_free(em_cam->grab->enq);
	kfree(em_cam->grab);
	em_cam->grab = NULL;
	kfifo_free(em_cam->deq_done);
	kfree(em_cam);
	em_cam = NULL;

	FNC_EXIT(ret)
	return ret;
}

/*
 * Driver init and exit Functions
 */

#ifdef CONFIG_PM
static int emxx_cam_suspend(struct platform_device *dev, pm_message_t state)
{
	int ret = 0;
	FNC_ENTRY;

	switch (state.event) {
	case DEV_SUSPEND_IDLE_1:
	case PM_EVENT_SUSPEND:
		if (em_cam->open_count)
			ret = -EBUSY;
		break;
	default:
		break;
	}

	FNC_EXIT(ret)
	return ret;
}
static int emxx_cam_resume(struct platform_device *dev)
{
	FNC_ENTRY;
	FNC_EXIT(0)
	return 0;
}
#endif

static int emxx_cam_probe(struct platform_device *devptr)
{
	int ret = 0;

	FNC_ENTRY;

	/* change the io level */
	emxx_camif_change_io_level();

	/*register cam variable*/
	ret = emxx_cam_register(0);
	if (ret)
		goto out_emxx_cam_probe;
	/*connect the cam to platform device,
	make cam as the device private member*/
	else
		platform_set_drvdata(devptr, em_cam);

	/*register cam IF for EMEV module*/
	ret = emxx_camif_register();
	if (ret)
		goto out_emxx_cam_unregister;

	/*register cam hw*/
	ret = emxx_cam_hw_register(&em_cam->hw);
	if (ret)
		goto out_emxx_camif_unregister;

	/* initial cam sensor*/
	if (em_cam->hw.prepare) {
		em_cam->pre.actions = 0;
		/* get the prepare information */
		ret = em_cam->hw.prepare(&em_cam->pre);
		if (ret) {
			FNC_EXIT(ret)
			return ret;
		}
	} else {
		err("%s: hw.prepare() nothing!\n", CAM_NAME);
	}

	em_cam->ipu  = CAM_IPU_OFF;/* this flag is not clear enough */

	em_cam->sequence = 0;

	/* em_cam->fmt = format_by_pixelformat(V4L2_PIX_FMT_NV12);
	em_cam->fmt = format_by_pixelformat(V4L2_PIX_FMT_NV422);*/
	em_cam->fmt = format_by_pixelformat(V4L2_PIX_FMT_YUV420);

	/* get the em_cam->c= em_cam->pre.bounds */
	cam_reset_update();

	info("%s : \"%s\" registered device video%d [V4L2]\n",
	     CAM_NAME, em_cam->hw.name, em_cam->vdev->minor);

	FNC_EXIT(0)
	return 0;

out_emxx_camif_unregister:
	emxx_camif_unregister();
out_emxx_cam_unregister:
	emxx_cam_unregister();
out_emxx_cam_probe:

	FNC_EXIT(ret)
	return ret;
}

static int emxx_cam_remove(struct platform_device *devptr)
{
	int ret = 0;
	FNC_ENTRY;

	em_cam->hw.unregister(0);
	emxx_camif_unregister();
	emxx_cam_unregister();
	platform_set_drvdata(devptr, NULL);

	FNC_EXIT(ret)
	return ret;
}

static struct platform_device *emxx_cam_device;
static struct platform_driver emxx_cam_driver = {
	.probe          = emxx_cam_probe,
	.remove         = __devexit_p(emxx_cam_remove),
#ifdef CONFIG_PM
	.suspend        = emxx_cam_suspend,
	.resume         = emxx_cam_resume,
#endif
	.driver.name    = CAM_NAME,
};

static int __init emxx_cam_init(void)
{
	int ret = 0;
	FNC_ENTRY;

#ifdef CONFIG_PROC_FS
	proc_emxx_cam_create();
#endif
#if 1 /* XXX */
	warming_up = 1;
#endif
	ret = platform_driver_register(&emxx_cam_driver);

	if (0 > ret) {
		err("%s: platform_driver_register() failed!\n", CAM_NAME);
		FNC_EXIT(ret)
		return ret;
	}

	emxx_cam_device =
		platform_device_register_simple(CAM_NAME, -1, NULL, 0);

	if (!IS_ERR(emxx_cam_device)) {
		if (platform_get_drvdata(emxx_cam_device)) {
			FNC_EXIT(ret)
			return 0;
		}
		err("%s: platform_get_drvdata() failed!\n", CAM_NAME);
		platform_device_unregister(emxx_cam_device);
		ret = -ENODEV;
	} else {
		err("%s: platform_device_register_simple() failed!\n",
		    CAM_NAME);
		ret = PTR_ERR(emxx_cam_device);
	}

	platform_driver_unregister(&emxx_cam_driver);

#ifdef CONFIG_PROC_FS
	proc_emxx_cam_destroy();
#endif

	FNC_EXIT(ret)
	return ret;
}

static void __exit emxx_cam_exit(void)
{
	FNC_ENTRY;

	platform_device_unregister(emxx_cam_device);
	platform_driver_unregister(&emxx_cam_driver);

#ifdef CONFIG_PROC_FS
	proc_emxx_cam_destroy();
#endif

	FNC_EXIT_N;
	return;
}

module_init(emxx_cam_init);
module_exit(emxx_cam_exit);


module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr, "video device to register (0=/dev/video0, etc)");

MODULE_DESCRIPTION("Mega Camera driver for emxx chip");
MODULE_SUPPORTED_DEVICE("video");
MODULE_LICENSE("GPL");

