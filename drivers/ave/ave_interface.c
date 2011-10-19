/*
 *  File Name       : ave_interface.c
 *  Function        : AVE interface
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/07/20
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/init.h> /* __init */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>      /* remap_page_range */
#include <linux/slab.h> /* kmalloc */
#include <linux/poll.h> /* POLLIN */
#include <linux/interrupt.h> /* tasklet */
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/io.h>  /* ioremap */
#include <mach/irqs.h> /* request_irq */
#include <mach/hardware.h> /* AVE base address */
#include <mach/pmu.h> /* clock */
#include <mach/smu.h>

#include "ave_info.h"
#include "ave_debug.h"
#include "ave_code.h"

static struct ave_info_t s_ave_info;

#define AVE_MODNAME     "em_ave"
#define AVE_DEVNAME     "ave"

static char *devname = AVE_DEVNAME; /* !< default device file name */
static int devmajor = 125;    /* !< default device major number */
struct ave_info_t *ave_info = &s_ave_info; /* !< this driver information */
/*
static void ave_tasklet_handler(unsigned long value);
static DECLARE_TASKLET(ave_tasklet, ave_tasklet_handler, 0);
*/
static DECLARE_MUTEX(ave_mutex);
static DECLARE_WAIT_QUEUE_HEAD(readq);
static DEFINE_SPINLOCK(ave_lock);

static void initialize_common_info(struct ave_common_info_t *common_info);


#ifdef CONFIG_PM
static int ave_pf_suspend(struct platform_device *dev, pm_message_t state);
static int ave_pf_resume(struct platform_device *dev);
#endif
static int ave_pf_probe(struct platform_device *dev);
static int ave_pf_remove(struct platform_device *dev);

static void ave_pf_release(struct device *dev);

static struct class *ave_class;
static struct device *ave_class_device;

static struct platform_device ave_pf_device = {
	.name = AVE_MODNAME,
	.id = -1,
	.dev = {
		.release = ave_pf_release,
	},
};

static struct platform_driver ave_pf_driver = {
	.probe = ave_pf_probe,
	.remove = ave_pf_remove,
#ifdef CONFIG_PM
	.suspend = ave_pf_suspend,
	.resume = ave_pf_resume,
#endif
	.driver = {
		.name  = AVE_MODNAME,
		.owner = THIS_MODULE,
	},
};

static int ave_open_state;


#define RETRY_COUNT(x)  ((loops_per_jiffy * x)/(5000/HZ))

static int ave_seq_init(void)
{
	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;

	struct ave_seq_init_set_param_t *initp =
		&ave_info->common_info->ave_seq_init_set_param;

	unsigned int bitstreamctl, framememctrl, seqoption, busy_flag;
	unsigned long flags;


	if ((hostp->work_buff_addr == 0) || (hostp->para_buf_addr == 0)
	   || (hostp->stream_wr_ptr == 0) || (initp->bitbuf_addr == 0))
		return -EINVAL;

	if ((initp->bitbuf_size == 0) || (initp->bitbuf_size > 16383))
		return -EINVAL;

	bitstreamctl = hostp->bit_sel_bigendian
		| (hostp->bit_sel64bits_endian << 1)
		| (hostp->bit_bufsts_checkdis << 2);

	framememctrl = hostp->frame_sel_bigendian
		| (hostp->frame_sel64bits_endian << 1)
		| (hostp->frame_selcinterleave << 2);

	seqoption = initp->mpqpreport_en
		| (initp->reordr_en << 1)
		| (initp->fileplay_en << 2)
		| (initp->dynalloc_en << 3);

	spin_lock_irqsave(&ave_lock, flags);

	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	writel(hostp->work_buff_addr, AVE_WORK_ADDR);

	writel(hostp->para_buf_addr, AVE_PARA_ADDR);

	writel(bitstreamctl, AVE_BITSTR_CTRL);

	if (hostp->seq_init_escape == AVE_INITESCAPE_ENA) {
		writel((readl(AVE_DECFUN_CTRL) | AVE_INITESCAPE_ENA)
		       & ~((0x1 << 2) | 0xFFFFFFF8), AVE_DECFUN_CTRL);
	} else {
		writel(readl(AVE_DECFUN_CTRL)
		       & ~(0x1 << 2 | AVE_INITESCAPE_ENA | 0xFFFFFFF8)
		       , AVE_DECFUN_CTRL);
	}

	writel(framememctrl, AVE_FRAME_CTRL);

	writel(0x0, AVE_BITF_DISF0);

	writel(hostp->stream_rd_ptr, AVE_BITSTR_RP0);

	writel(hostp->stream_wr_ptr, AVE_BITSTR_WP0);

	writel(initp->bitbuf_addr, AVE_SEQC_BB_START);

	writel(initp->bitbuf_size, AVE_SEQC_BB_SIZE);

	writel(seqoption, AVE_SEQC_OPTION);

	writel(initp->cmd_picheight | (initp->cmd_picwidth << 16),
	       AVE_SEQC_SRC_SIZE);

	writel(initp->streamstartoffset, AVE_SEQC_START_BYTE);

	writel(initp->psbufaddr, AVE_SEQC_PS_START);

	writel(initp->psbufsize, AVE_SEQC_PS_SIZE);

	if (hostp->run_code_std == AVE_CODE_MP4)
		writel(initp->mp4class, AVE_SEQC_MP4_CLASS);

	if (hostp->run_code_std == AVE_CODE_VC_1) {
		writel((initp->vc1_streamdetect << 2)
		       | initp->vc1_streamfmt,
		       AVE_SEQC_VC1_FMT);
	}

	writel(0x0, AVE_RUN_INDEX);

	writel((hostp->run_code_std & 0x3), AVE_RUN_CODSTD);

	if (hostp->run_code_std == AVE_CODE_DIV3)
		writel(0x1, AVE_RUNAUXSTD);
	else if (hostp->run_code_std == AVE_CODE_MP4)
		writel(0x0, AVE_RUNAUXSTD);


	writel(readl(AVE_INT_ENABLE) | INT_SEQ_INIT | INT_BUF_EMP
	       , AVE_INT_ENABLE);

#if 0

	debug1("AVE_WORK_ADDR  %x", readl(AVE_WORK_ADDR));
	debug1("AVE_PARA_ADDR  %x", readl(AVE_PARA_ADDR));
	debug1("AVE_BITSTR_CTRL  %x", readl(AVE_BITSTR_CTRL));
	debug1("AVE_DECFUN_CTRL  %x", readl(AVE_DECFUN_CTRL));
	debug1("AVE_FRAME_CTRL  %x", readl(AVE_FRAME_CTRL));
	debug1("AVE_BITF_DISF0  %x", readl(AVE_BITF_DISF0));
	debug1("AVE_BITSTR_RP0  %x", readl(AVE_BITSTR_RP0));
	debug1("AVE_BITSTR_WP0  %x", readl(AVE_BITSTR_WP0));
	debug1("AVE_SEQC_BB_START  %x", readl(AVE_SEQC_BB_START));
	debug1("AVE_SEQC_BB_SIZE  %x", readl(AVE_SEQC_BB_SIZE));
	debug1("AVE_SEQC_OPTION  %x", readl(AVE_SEQC_OPTION));
	debug1("AVE_SEQC_SRC_SIZE  %x", readl(AVE_SEQC_SRC_SIZE));
	debug1("AVE_SEQC_START_BYTE  %x", readl(AVE_SEQC_START_BYTE));
	debug1("AVE_SEQC_PS_START  %x", readl(AVE_SEQC_PS_START));
	debug1("AVE_SEQC_PS_SIZE  %x", readl(AVE_SEQC_PS_SIZE));
	debug1("AVE_SEQC_VC1_FMT  %x", readl(AVE_SEQC_VC1_FMT));
	debug1("AVE_RUN_INDEX  %x", readl(AVE_RUN_INDEX));
	debug1("AVE_RUN_CODSTD  %x", readl(AVE_RUN_CODSTD));
	debug1("AVE_INT_ENABLE  %x", readl(AVE_INT_ENABLE));

#endif

	writel(SEQ_INIT, AVE_RUNCMD);

	spin_unlock_irqrestore(&ave_lock, flags);

	return 0;
}

static int ave_seq_end(void)
{
	struct ave_base_info_t *basep =
		&ave_info->common_info->ave_base_info;

	int count, busy_flag;
	unsigned long flags;

	spin_lock_irqsave(&ave_lock, flags);

	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	writel(SEQ_END, AVE_RUNCMD);

	spin_unlock_irqrestore(&ave_lock, flags);

	count = RETRY_COUNT(basep->wait_count);

	do {
		busy_flag = readl(AVE_BUSY_FLAG) & 0x1;
	} while ((busy_flag != 0) && count--);

	if (busy_flag != 0)
		return -EBUSY;

	emxx_clkctrl_on(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_on(EMXX_CLKCTRL_AVEA);

	return 0;
}

static int ave_set_frame_buf(void)
{
	struct ave_base_info_t *basep =
		&ave_info->common_info->ave_base_info;
	struct ave_frame_buf_set_param_t *framep =
		&ave_info->common_info->ave_frame_buf_set_param;
	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;
	int count, busy_flag, i;
	void *buf_virt;
	unsigned long flags;

	/*if ((framep->slice_bufaddr == 0) || (framep->slice_bufsize == 0 ))*/
	/*	return -EINVAL;*/

	buf_virt = ioremap(hostp->para_buf_addr, PARAM_BUFSIZE);
	if (buf_virt == NULL)
		return -EINVAL;

	for (i = 0; i < framep->framebufnum; i++) {
		if (i & 0x1) {
			iowrite32(framep->frame_addr.yaddr[i],
				  buf_virt + i * 3 * 4 - 4);
			iowrite32(framep->frame_addr.cbaddr[i],
				  buf_virt + i * 3 * 4 + 8);
			iowrite32(framep->frame_addr.craddr[i],
				  buf_virt + i * 3 * 4 + 4);
			iowrite32(framep->frame_addr.mvcoladdr[i],
				  buf_virt + (i + 96) * 4 - 4);
		} else {
			iowrite32(framep->frame_addr.yaddr[i],
				  buf_virt + i * 3 * 4 + 4);
			iowrite32(framep->frame_addr.cbaddr[i],
				  buf_virt + i * 3 * 4);
			iowrite32(framep->frame_addr.craddr[i],
				  buf_virt + i * 3 * 4 + 12);
			iowrite32(framep->frame_addr.mvcoladdr[i],
				  buf_virt + (i + 96) * 4 + 4);
		}
	}

	iounmap(buf_virt);

	spin_lock_irqsave(&ave_lock, flags);

	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	writel(framep->slice_bufaddr, AVE_FRMC_SLC_START);

	writel(framep->slice_bufsize, AVE_FRMC_SLC_SIZE);

	writel(framep->framebufnum, AVE_FRMC_BUF_NUM);

	writel(framep->linestride, AVE_FRMC_BUF_STRIDE);

	writel(EMXX_SRAM_BASE + 0x10000, AVE_FRMC_AXI_DY_ADDR);
	writel(EMXX_SRAM_BASE + 0x13C00, AVE_FRMC_AXI_DC_ADDR);
	writel(EMXX_SRAM_BASE + 0x17800, AVE_FRMC_AXI_B_ADDR);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	writel(SET_FRAME_BUF, AVE_RUNCMD);

	spin_unlock_irqrestore(&ave_lock, flags);

	count = RETRY_COUNT(basep->wait_count);

	do {
		busy_flag = readl(AVE_BUSY_FLAG) & 0x1;
	} while ((busy_flag != 0) && count--);

	if (busy_flag != 0)
		return -EBUSY;

	return 0;
}

static int ave_dec_para_set(void)
{
	struct ave_base_info_t *basep =
		&ave_info->common_info->ave_base_info;
	struct ave_dec_para_set_param_t *parap =
		&ave_info->common_info->ave_dec_para_set_param;

	int count, busy_flag;
	unsigned long flags;

	spin_lock_irqsave(&ave_lock, flags);

	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	writel(parap->parasettype, AVE_PARC_TYPE);
	writel(parap->parasetsize, AVE_PARC_SIZE);

	writel(DEC_PARA_SET, AVE_RUNCMD);

	spin_unlock_irqrestore(&ave_lock, flags);

	count = RETRY_COUNT(basep->wait_count);

	do {
		busy_flag = readl(AVE_BUSY_FLAG) & 0x1;
	} while ((busy_flag != 0) && count--);

	if (busy_flag != 0)
		return -EBUSY;

	return 0;
}

static int ave_dec_buf_flush(void)
{
	struct ave_base_info_t *basep =
		&ave_info->common_info->ave_base_info;
	struct ave_flash_set_param_t *flushp =
		&ave_info->common_info->ave_flash_set_param;

	int count, busy_flag;
	unsigned long flags;

	spin_lock_irqsave(&ave_lock, flags);

	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	writel(flushp->flash_type, AVE_FLASHC_TYPE);
	writel(flushp->flash_rdptr, AVE_FLASHC_RDPTR);

#if 0

	debug1("AVE_FLASHC_TYPE  %x", readl(AVE_FLASHC_TYPE));
	debug1("AVE_FLASHC_RDPTR  %x", readl(AVE_FLASHC_RDPTR));

#endif


	writel(DEC_BUF_FLUSH, AVE_RUNCMD);

	spin_unlock_irqrestore(&ave_lock, flags);

	count = RETRY_COUNT(basep->wait_count);

	do {
		busy_flag = readl(AVE_BUSY_FLAG) & 0x1;
	} while ((busy_flag != 0) && count--);

	if (busy_flag != 0)
		return -EBUSY;

	return 0;
}


static int ave_pic_run(void)
{
	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;

	struct ave_pic_run_set_param_t *picp =
		&ave_info->common_info->ave_pic_run_set_param;

	unsigned int pic_option;
	unsigned long flags;
	int busy_flag;

	pic_option = picp->prescan_en | (picp->prescan_mode << 1)
		| (picp->iframesearch_en << 2) | (picp->skipframe_mode << 3);


	spin_lock_irqsave(&ave_lock, flags);

	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	if (hostp->seq_init_escape == AVE_INITESCAPE_ENA) {
		writel(readl(AVE_DECFUN_CTRL) | AVE_INITESCAPE_ENA
		       | (hostp->stream_end << 2), AVE_DECFUN_CTRL);
	} else {
		writel((readl(AVE_DECFUN_CTRL) | (hostp->stream_end << 2))
		       & ~(AVE_INITESCAPE_ENA) , AVE_DECFUN_CTRL);
	}
	writel(hostp->stream_wr_ptr, AVE_BITSTR_WP0);

	writel(hostp->stream_rd_ptr, AVE_BITSTR_RP0);

	writel(hostp->bitframe_disflag, AVE_BITF_DISF0);

	writel(0x0000, AVE_PICC_ROT_MODE);

	writel(pic_option, AVE_PICC_OPTION);

	writel(picp->skipframenum, AVE_PICC_FSKIP_NUM);

	writel(picp->framechunksize, AVE_PICC_FILE_SIZE);

	writel(picp->picbitbufstart, AVE_PICC_BB_START);

	writel(picp->picstartbyteoffset, AVE_PICC_START_BYTE);

	if (hostp->run_code_std == AVE_CODE_H264)
		writel(0x285, AVE_AXI_SDR_USE);
	else
		writel(0x81, AVE_AXI_SDR_USE);

	writel(0x0, AVE_RUN_INDEX);

	writel((hostp->run_code_std & 0x3), AVE_RUN_CODSTD);

	if (hostp->run_code_std == AVE_CODE_DIV3)
		writel(0x1, AVE_RUNAUXSTD);
	else if (hostp->run_code_std == AVE_CODE_MP4)
		writel(0x0, AVE_RUNAUXSTD);


	writel(readl(AVE_INT_ENABLE) | INT_PIC_RUN | INT_BUF_EMP,
	       AVE_INT_ENABLE);
#if 0

	debug1("AVE_DECFUN_CTRL  %x", readl(AVE_DECFUN_CTRL));
	debug1("AVE_BITSTR_WP0  %x", readl(AVE_BITSTR_WP0));
	debug1("AVE_BITSTR_RP0  %x", readl(AVE_BITSTR_RP0));
	debug1("AVE_BITF_DISF0  %x", readl(AVE_BITF_DISF0));
	debug1("AVE_PICC_ROT_MODE  %x", readl(AVE_PICC_ROT_MODE));
	debug1("AVE_PICC_OPTION  %x", readl(AVE_PICC_OPTION));
	debug1("AVE_PICC_FSKIP_NUM  %x", readl(AVE_PICC_FSKIP_NUM));
	debug1("AVE_PICC_FILE_SIZE  %x", readl(AVE_PICC_FILE_SIZE));
	debug1("AVE_PICC_BB_START  %x", readl(AVE_PICC_BB_START));
	debug1("AVE_PICC_START_BYTE  %x", readl(AVE_PICC_START_BYTE));
	debug1("AVE_RUN_INDEX  %x", readl(AVE_RUN_INDEX));
	debug1("AVE_RUN_CODSTD  %x", readl(AVE_RUN_CODSTD));
	debug1("AVE_RUNAUXSTD  %x", readl(AVE_RUNAUXSTD));
	debug1("AVE_INT_ENABLE  %x", readl(AVE_INT_ENABLE));

#endif

	writel(PICTURE_RUN, AVE_RUNCMD);

	spin_unlock_irqrestore(&ave_lock, flags);

	return 0;

}

static int ave_add_stream(void)
{

	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;

	int busy_flag;
	unsigned long flags;


	spin_lock_irqsave(&ave_lock, flags);

	busy_flag = readl(AVE_BUSY_FLAG);

	if (!busy_flag) {
		spin_unlock_irqrestore(&ave_lock, flags);
		return -EBUSY;
	}

	if (hostp->seq_init_escape == AVE_INITESCAPE_ENA) {
		writel(readl(AVE_DECFUN_CTRL) | AVE_INITESCAPE_ENA
		       | (hostp->stream_end << 2), AVE_DECFUN_CTRL);
	} else {
		writel((readl(AVE_DECFUN_CTRL) | (hostp->stream_end << 2))
		       & ~(AVE_INITESCAPE_ENA) , AVE_DECFUN_CTRL);
	}

	writel(hostp->stream_wr_ptr, AVE_BITSTR_WP0);

	writel(readl(AVE_INT_REASON) & ~INT_BUF_EMP, AVE_INT_REASON);

	spin_unlock_irqrestore(&ave_lock, flags);

	return 0;
}

static void get_seq_init_data(void)
{

	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;
	struct ave_seq_init_get_param_t *initp =
		&ave_info->common_info->ave_seq_init_get_param;
	struct ave_interrupt_t *interruptp =
		&ave_info->interrupt;

	unsigned int picsize, seq_info, crop_lr, crop_tb, head_rep;

	hostp->stream_rd_ptr = readl(AVE_BITSTR_RP0);
	hostp->bitframe_disflag = readl(AVE_BITF_DISF0);

	initp->aspect_ratio = readl(AVE_SEQR_ASPECT);
	picsize = readl(AVE_SEQR_SRC_SIZE);

	initp->ret_picheight = picsize & 0xFFFF;
	initp->ret_picwidth = (picsize >> 16) & 0xFFFF;

	initp->framerateinfo = readl(AVE_SEQR_SRC_FRATE);
	initp->framebufneed = readl(AVE_SEQR_FRAME_NEED);
	initp->framebufdelay = readl(AVE_SEQR_FRAME_DLY);

	seq_info = readl(AVE_SEQR_INFO);
	initp->datapart_en = seq_info & 0x1;
	initp->revvlc_en = (seq_info >> 1) & 0x1;
	initp->shortvideohead_en = (seq_info >> 2) & 0x1;
	initp->h263_annexj_en = (seq_info >> 3) & 0x1;

	crop_lr = readl(AVE_SEQR_CROP_LR);
	initp->croprightoffset = crop_lr & 0x3FF;
	initp->cropleftoffset = (crop_lr >> 10) & 0x3FF;

	crop_tb = readl(AVE_SEQR_CROP_TB);
	initp->cropbottomoffset = crop_tb & 0x3FF;
	initp->croptopoffset = (crop_tb >> 10) & 0x3FF;

	head_rep = readl(AVE_SEQR_HDR_RPRT);

	initp->profile = (head_rep & 0xFF) | ((head_rep >> 16) & 0xFF00);
	initp->level = (head_rep >> 8) & 0xFF;
	initp->interlace = (head_rep >> 16) & 0x1;
	initp->direct_8x8flag = (head_rep >> 17) & 0x1;
	initp->vc1_psf = (head_rep >> 18) & 0x1;
	initp->constraint_setflag[0] = (head_rep >> 19) & 0x1;
	initp->constraint_setflag[1] = (head_rep >> 20) & 0x1;
	initp->constraint_setflag[2] = (head_rep >> 21) & 0x1;
	initp->constraint_setflag[3] = (head_rep >> 22) & 0x1;

	if (readl(AVE_RUN_CODSTD) == AVE_CODE_MP4) {
		picsize = readl(AVE_SEQR_MP4_PAR);

		initp->mpeg4_height = picsize & 0xFF;
		initp->mpeg4_width = (picsize >> 8) & 0xFF;
	} else {
		initp->timescale = readl(AVE_SEQR_TIME_SCALE);
	}
	initp->numunits_in_tick = readl(AVE_SEQR_NUM_UNIT);

	interruptp->ret_status = readl(AVE_SEQR_SUCCESS);

	return;
}

static void get_pic_run_data(void)
{

	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;
	struct ave_pic_run_get_param_t *picp =
		&ave_info->common_info->ave_pic_run_get_param;
	struct ave_interrupt_t *interruptp =
		&ave_info->interrupt;


	unsigned int picsize, pic_type, pic_post, picsuccess;

	hostp->stream_rd_ptr = readl(AVE_BITSTR_RP0);
	hostp->bitframe_disflag = readl(AVE_BITF_DISF0);

	picsize = readl(AVE_PICR_SIZE);
	picp->decpicheight = picsize & 0xFFFF;
	picp->decpicwidth = (picsize >> 16) & 0xFFFF;

	picp->decframenum = readl(AVE_PICR_FRAME_NUM);

	picp->decpicldx = readl(AVE_PICR_IDX);

	picp->errmbnum = readl(AVE_PICR_ERR_MB_NUM);

	pic_type = readl(AVE_PICR_TYPE);
	picp->pictype = pic_type & 0xFF;
	picp->npf_flag = (pic_type >> 16) & 0x3;
	picp->interlacedframe = (pic_type >> 18) & 0x1;
	picp->picstructure = (pic_type >> 19) & 0x3;
	picp->topfieldfirst = (pic_type >> 21) & 0x1;
	picp->repeatfirstfield = (pic_type >> 22) & 0x1;
	picp->progressiveframe = (pic_type >> 23) & 0x3;
	picp->fieldsequence = (pic_type >> 25) & 0x3;

	if (hostp->run_code_std == AVE_CODE_VC_1) {
		pic_post = readl(AVE_PICR_POST);
		picp->rangered = pic_post & 0x1;
		picp->hscaleflag = (pic_post >> 1) & 0x1;
		picp->vscaleflag = (pic_post >> 2) & 0x1;

#ifdef CONFIG_EMXX_AVE_VC1
		picp->postsrcbufidx = (pic_post >> 3) & 0x1F;
#endif
	} else {
		picp->rangered = 0;
		picp->hscaleflag = 0;
		picp->vscaleflag = 0;
#ifdef CONFIG_EMXX_AVE_VC1
		picp->postsrcbufidx = 0;
#endif
	}
	picp->prescanres = readl(AVE_PICR_OPTION);

	picp->decordedpicldx = readl(AVE_PICR_CUR_IDX);

	picp->consumedbit = readl(AVE_PICR_CONSUMED);

	picsuccess = readl(AVE_PICR_SUCCESS);

	picp->mp4packedpbframe = (picsuccess >> 16) & 0x1;

	interruptp->ret_status = picsuccess & 0x1;
	interruptp->lackofslicebuf = (picsuccess >> 2) & 0x1;
	interruptp->lackofpsbuf = (picsuccess >> 3) & 0x1;

	return;
}

static void get_buf_emp_data(void)
{
	struct ave_host_param_t *hostp =
		&ave_info->common_info->ave_host_param;

	hostp->stream_rd_ptr = readl(AVE_BITSTR_RP0);

	return;
}

/* file operations ----------------------------------------------------------*/

/*!
 * read file operation
 * @param[in] filp
 * @param[out] buf to copy struct ave_interrupt_t
 * @param[in] count buf size
 * @param[in] offp
 * @retval size size of struct ave_interrupt_t
 * @retval -EINVAL count error
 */
static ssize_t ave_read(struct file *filp, char *buf,
			size_t count, loff_t *offp)
{
	int minor = MINOR(filp->f_dentry->d_inode->i_rdev);
	unsigned long flags;
	struct ave_interrupt_t tmp_int;
	int ret;
	debug1("minor %d, count %d", minor, count);

	if (count < sizeof(ave_info->interrupt)) {
		debug1("count < sizeof(ave_info->interrupt)");
		return -EINVAL;
	}

	if (down_interruptible(&ave_mutex)) {
		debug0("down_interruptible failed");
		return -ERESTARTSYS;
	}

	if (wait_event_interruptible(readq,
				     ave_info->interrupt.interrupt.l != 0)) {
		debug0("wait_event_interruptible failed");
		ret = -ERESTARTSYS;
		goto up_exit;
	}

	spin_lock_irqsave(&ave_lock, flags);
	tmp_int = ave_info->interrupt;
	memset(&ave_info->interrupt, 0, sizeof(ave_info->interrupt));
	spin_unlock_irqrestore(&ave_lock, flags);

	if (copy_to_user(buf, &tmp_int, sizeof(tmp_int))) {
		debug0("copy_to_user failed");
		ret = -EFAULT;
		goto up_exit;
	}

	ret = sizeof(ave_info->interrupt);

up_exit:
	up(&ave_mutex);
	return ret;
}

/*!
 * select file operation
 * @param[in] filp
 * @param[in] wait
 * @retval mask POLLIN | POLLRDNORM
 */
static unsigned int ave_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	debug2("minor %d", MINOR(filp->f_dentry->d_inode->i_rdev));

	poll_wait(filp, &readq, wait);
	if (ave_info->interrupt.interrupt.l)
		mask |= POLLIN | POLLRDNORM;

	debug2("mask 0x%X", mask);
	return mask;
}

/*!
 * ioctl file operation
 * @param[in] inode
 * @param[in] filp
 * @param[in] cmd AVE_IOCQBUFSIZE or AVE_IOCSREGCMD
 * @param[in] arg struct ave_command_t with AVE_IOCSREGCMD
 * @retval size for AVE_IOCQBUFSIZE
 * @retval 0 command successful
 * @retval -EINVAL command error
 */
static int ave_ioctl(struct inode *inode, struct file *filp,
		     unsigned int cmd, unsigned long arg)
{
	struct ave_register_access_t ra;
	unsigned int busy_reg;
	unsigned long flags;
	int ret = 0;

	switch (cmd) {

	case AVE_IOCQBUFSIZE:
		ret = PAGE_ALIGN(sizeof(struct ave_common_info_t));
		debug1("AVE_IOCQBUFSIZE %d", ret);
		break;

	case AVE_IOCRUNCMD:
		debug1("AVE_IOCRUNCMD 0x%08lX", arg);
		switch (arg) {
		case seq_init:
			ret = ave_seq_init();
			break;
		case seq_end:
			ret = ave_seq_end();
			break;
		case pic_run:
			ret = ave_pic_run();
			break;
		case set_frame_buf:
			ret = ave_set_frame_buf();
			break;
		case dec_para_set:
			ret = ave_dec_para_set();
			break;
		case dec_buf_flush:
			ret = ave_dec_buf_flush();
			break;
		case add_stream:
			ret = ave_add_stream();
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	case AVE_IOCGBUSYFLAG:
		debug1("AVE_IOCGBUSYFLAG");
		spin_lock_irqsave(&ave_lock, flags);

		busy_reg = readl(AVE_BUSY_FLAG);

		spin_unlock_irqrestore(&ave_lock, flags);

		if (copy_to_user((void *)arg, &busy_reg, sizeof(busy_reg))) {
			debug0("copy_to_user failed");
			ret = -EFAULT;
			break;
		}

		break;

	case AVE_IOCSREGISTER:
	case AVE_IOCGREGISTER:
		debug1("AVE_IOCxREGISTER");
		if (arg == 0) {
			ret = -EINVAL;
			break;
		}
		if (copy_from_user(&ra, (void *)arg, sizeof(ra))) {
			debug0("copy_from_user failed");
			ret = -EFAULT;
			break;
		}
		{
			unsigned int addr = AVE_CODE_RUN + ra.address;

			spin_lock_irqsave(&ave_lock, flags);

			emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
			emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

			if (cmd == AVE_IOCSREGISTER)
				writel(ra.value, addr);
			else
				ra.value = readl(addr);

			spin_unlock_irqrestore(&ave_lock, flags);

			if (cmd == AVE_IOCGREGISTER &&
			    copy_to_user((void *)arg, &ra, sizeof(ra))) {
				debug0("copy_to_user failed");
				ret = -EFAULT;
				break;
			}
			debug1(" address 0x%X", ra.address);
			debug1(" value   0x%X", ra.value);
		}
		break;

	case AVE_IOCSDEBUGLEVEL:
		debug1("AVC_IOCSDEBUGLEVEL %lu", arg);
		debug_level = (int)arg;
		break;

	default:
		debug1("unknown cmd 0x%08X, arg 0x%08lX", cmd, arg);
		ret = -EINVAL; /* ENOTTY */
		break;
	}

	debug1("ret %d", ret);
	return ret;
}

/*!
 * mmap file operation
 * @param[in] filp
 * @param[in] vma
 * @retval 0 successful
 * @retval -EINVAL size error
 */
static int ave_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long info_size = PAGE_ALIGN(sizeof(struct ave_common_info_t));
	unsigned long info_pa = (unsigned long)ave_info->pa_common_info;

	debug1("vm_start 0x%08lX", vma->vm_start);
	debug1("vm_end   0x%08lX", vma->vm_end);
	debug1("size     %lu", size);
	debug1("vm_pgoff 0x%08lX", vma->vm_pgoff);
	debug1("vm_page_prot 0x%08lX", pgprot_val(vma->vm_page_prot));

	if (size > info_size) {
		debug1("error request size %lu > allocated size %lu",
		       size, info_size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, info_pa >> PAGE_SHIFT, size,
			     vma->vm_page_prot)) {
		debug1("remap_pfn_range failed");
		return -EAGAIN;
	}

	initialize_common_info(ave_info->common_info);
	debug1("mapped to 0x%08lX", info_pa);
	return 0;
}


/*!
 * stop AVE module clock supply
 * @param void
 * @return void
 */
static void ave_clock_off(void)
{

	emxx_reset_device(EMXX_RST_AVE_C);
	emxx_reset_device(EMXX_RST_AVE_A);
	emxx_reset_device(EMXX_RST_AVE_P);

	emxx_close_clockgate(EMXX_CLK_AVE_A);
	emxx_close_clockgate(EMXX_CLK_AVE_C);
	emxx_close_clockgate(EMXX_CLK_AVE_P);

}


/*!
 * start AVE module clock supply
 * @param void
 * @return void
 */
static void ave_clock_on(void)
{
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
	emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);

	emxx_open_clockgate(EMXX_CLK_AVE_A);
	emxx_open_clockgate(EMXX_CLK_AVE_C);
	emxx_open_clockgate(EMXX_CLK_AVE_P);
}

static void ave_unreset(void)
{

	emxx_unreset_device(EMXX_RST_AVE_C);
	emxx_unreset_device(EMXX_RST_AVE_A);
	emxx_unreset_device(EMXX_RST_AVE_P);

}

/*!
 * start AVE module power supply
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int ave_power_on(int mode)
{

	int pv_seq;
	int count = PV_POWER_WAIT;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & PV_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		debug1("power busy flag");
		return -1;
	}
	if (mode == AVE_POWERDOWN) {
		writel(readl(SMU_PV_SWON) | PV_SWON | PV_PDON,
		       SMU_PV_SWON);
	} else {
		writel((readl(SMU_PV_SWON) | PV_SWON) & ~PV_PDON ,
		       SMU_PV_SWON);
	}
	count = PV_POWER_WAIT;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & PV_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		debug1("power busy flag");
		return -1;
	}

	udelay(100);

	if (mode == AVE_RETENTION)
		writel(1, AVE_CODE_RUN);

	udelay(30);

	return 0;

}

/*!
 * stop AVE module power supply
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int ave_power_off(int mode)
{

	int pv_seq;
	int count = PV_POWER_WAIT;

	if (mode == AVE_RETENTION)
		writel(0, AVE_CODE_RUN);

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & PV_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		debug1("power busy flag");
		return -1;
	}

	if (mode == AVE_POWERDOWN) {
		writel((readl(SMU_PV_SWON) | PV_PDON) & ~PV_SWON,
			SMU_PV_SWON);
	} else {
		writel(readl(SMU_PV_SWON) & ~(PV_SWON | PV_PDON),
		       SMU_PV_SWON);
	}
	count = PV_POWER_WAIT;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & PV_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		debug1("power busy flag");
		return -1;
	}

	return 0;

}

/*!
 * open file operation
 * @param[in] inode
 * @param[in] filp
 * @retval 0 successful
 * @retval -EBUSY double request
 * @retval -FAULT power setting failure etc
 */
static int ave_open(struct inode *inode, struct file *filp)
{

	unsigned int i, count, busy_flag, data;


	if (ave_info->state != state_initial) {
		debug1("already opened");
		return -EBUSY;
	}

	ave_clock_on();

	/* power on */
	if (ave_power_on(AVE_POWERDOWN) < 0) {
		debug0("ave_power_on failed");
		return -EFAULT;
	}

	ave_unreset();

	writel(0x0, AVE_CODE_RUN);

	for (i = 0; i < 512; i++) {
		data = bit_code[i];
		writel((i << 16) | data, AVE_CODE_DL);
	}

	writel(ave_info->pa_code_addr, AVE_CODE_ADDR);

	writel(0x1, AVE_BIT_INT_CLR);

	writel(0x0, AVE_AXI_SDR_USE);

	writel(0x0, AVE_INT_ENABLE);

	writel(0x1, AVE_CODE_RUN);

	udelay(30);

	writel(CMD_GETVER, AVE_RUNCMD);

	count = RETRY_COUNT(100);

	do {
		busy_flag = readl(AVE_BUSY_FLAG) & 0x1;
	} while ((busy_flag != 0) && count--);

	if (busy_flag != 0)
		return -EBUSY;

	ave_info->fw_code_ver = readl(AVE_VERC_NUM);

	ave_info->state = state_idle;

	ave_open_state = 1;

	return 0;
}

/*!
 * close file operation
 * @param[in] inode
 * @param[in] filp
 * @retval 0 successful
 */
static int ave_release(struct inode *inode, struct file *filp)
{
	int minor = MINOR(inode->i_rdev);

	debug1("minor %d", minor);

	ave_info->state = state_initial;

	ave_clock_off();

	if (ave_power_off(AVE_POWERDOWN) < 0)
		debug0("avc_power_off failed");

	ave_open_state = 0;

	return 0;
}

/* ! file operations for this driver */
static const struct file_operations ave_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.open       = ave_open,
	.release    = ave_release,
	.read       = ave_read,
	.poll       = ave_poll,
	.ioctl      = ave_ioctl,
	.mmap       = ave_mmap,
};


/* interruption -------------------------------------------------------------*/

/*!
 * AVE interruption handler about stream buffer full
 * @param irq
 * @param dev_id
 * @param regs
 */
static irqreturn_t ave_interrupt(int irq, void *dev_id)
{

	unsigned short status, factor;

	status = readl(AVE_BIT_INT_STS);

	if (status) {
		factor = readl(AVE_INT_REASON);
		ave_info->interrupt.busy_flag =  readl(AVE_BUSY_FLAG);
		if (factor & INT_SEQ_INIT) {
			writel(0x1, AVE_BIT_INT_CLR); /* clear interruption */
			get_seq_init_data();
			writel(readl(AVE_INT_REASON) & ~INT_SEQ_INIT,
			       AVE_INT_REASON);
		} else if (factor & INT_PIC_RUN) {
			writel(0x1, AVE_BIT_INT_CLR); /* clear interruption */
			get_pic_run_data();
			writel(readl(AVE_INT_REASON) & ~INT_PIC_RUN,
			       AVE_INT_REASON);
			emxx_clkctrl_on(EMXX_CLKCTRL_AVEC);
			emxx_clkctrl_on(EMXX_CLKCTRL_AVEA);
		} else if (factor & INT_BUF_EMP) {
			writel(0x1, AVE_BIT_INT_CLR); /* clear interruption */
			get_buf_emp_data();
		}
		debug1("AVE interrupt = %x ", factor);
		ave_info->interrupt.interrupt.l = factor;
		wake_up_interruptible(&readq);

		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

#if 0
/*!
 * tasklet for interruption
 * @param[in] value unused
 * @return void
 */
void ave_tasklet_handler(unsigned long value)
{
	unsigned int factor;
	unsigned int int_a, int_c;


	wake_up_interruptible(&readq);
}
#endif

/* module functions ---------------------------------------------------------*/

static void initialize_common_info(struct ave_common_info_t *common_info)
{

	struct ave_base_info_t *basep =
		&common_info->ave_base_info;
	struct ave_host_param_t *hostp =
		&common_info->ave_host_param;
	struct ave_seq_init_set_param_t *initp =
		&common_info->ave_seq_init_set_param;
	struct ave_pic_run_set_param_t *runp =
		&common_info->ave_pic_run_set_param;
	struct ave_frame_buf_set_param_t *fbufp =
		&common_info->ave_frame_buf_set_param;
	struct ave_dec_para_set_param_t *parap =
		&common_info->ave_dec_para_set_param;

	memset(common_info, 0, sizeof(*common_info));

	basep->wait_count = 100;
	basep->code_ver = ave_info->fw_code_ver;

	hostp->bit_sel_bigendian = AVE_BIGENDIAN;
	hostp->bit_sel64bits_endian = AVE_64BITSFORMAT;
	hostp->bit_bufsts_checkdis = AVE_CHK_DIS_ON;
	hostp->frame_sel_bigendian = AVE_BIGENDIAN;
	hostp->frame_sel64bits_endian = AVE_64BITSFORMAT;
	hostp->frame_selcinterleave = AVE_CBCR_SEP;

	initp->mpqpreport_en = AVE_MPQP_DIS;
	initp->reordr_en = AVE_REODER_DIS;
	initp->fileplay_en = AVE_FILEPLAY_DIS;
	initp->dynalloc_en = AVE_DYNALLOC_DIS;
	initp->mp4class = AVE_MP4C_MP4;
	initp->vc1_streamdetect = AVE_VC1_AUTO;

	runp->prescan_en = AVE_PRESCAN_DIS;
	runp->prescan_mode = AVE_PRESCAN_DEC;
	runp->iframesearch_en = AVE_IFRAMES_DIS;
	runp->skipframe_mode = AVE_SKIPF_NON;

	fbufp->linestride = 0;

	parap->parasettype = AVE_PARA_TYPE_SEQ;

}
#ifdef CONFIG_PM
static int ave_pf_suspend(struct platform_device *dev, pm_message_t state)
{
	if (ave_open_state) {
		emxx_clkctrl_off(EMXX_CLKCTRL_AVEC);
		emxx_clkctrl_off(EMXX_CLKCTRL_AVEA);
		ave_power_off(AVE_RETENTION);
	}
	return 0;
}

static int ave_pf_resume(struct platform_device *dev)
{
	if (ave_open_state)
		ave_power_on(AVE_RETENTION);

	return 0;
}
#endif

static int ave_pf_probe(struct platform_device *dev)
{
	struct ave_common_info_t *common_info;
	dma_addr_t cmn_paddr, code_paddr;
	int ret, i;
	void *virt;
	unsigned short data;
	unsigned short *code_addr;

	if (ave_power_off(AVE_POWERDOWN) < 0) {
		debug0("avc_power_off failed");
		ret = -EFAULT;
		goto error_return;
	}

	common_info = dma_alloc_coherent(0, sizeof *common_info,
					 &cmn_paddr, GFP_KERNEL | GFP_DMA);
	if (common_info) {
		initialize_common_info(common_info);
		ave_info->common_info = common_info;
		ave_info->pa_common_info = cmn_paddr;
	} else {
		debug0("dma_alloc_coherent ave_common_info failed");
		ret = -ENOMEM;
		goto error_return;
	}

	virt = dma_alloc_coherent(0, sizeof(bit_code),
					 &code_paddr, GFP_KERNEL | GFP_DMA);
	if (virt) {
		ave_info->v_code_addr = virt;
		ave_info->pa_code_addr = code_paddr;
		debug1("code_paddr 0x%08lX", ave_info->pa_code_addr);
		debug1("v_code_addr 0x%08lX", ave_info->v_code_addr);
	} else {
		debug0("dma_alloc_coherent ave_code failed");
		ret = -ENOMEM;
		goto free_common_info;
	}

	code_addr = (unsigned short *)ave_info->v_code_addr;

	for (i = 0; i < sizeof(bit_code) / sizeof(bit_code[0]); i += 4) {
		data = bit_code[i + 3];
		*(code_addr + i) = data;
		data = bit_code[i + 2];
		*(code_addr + i + 1) = data;
		data = bit_code[i + 1];
		*(code_addr + i + 2) = data;
		data = bit_code[i];
		*(code_addr + i + 3) = data;
	}

	/* request_irq returns ENOMEM/EINVAL */
	ret = request_irq(INT_AVE, ave_interrupt,
			  IRQF_SHARED, AVE_MODNAME, ave_info);
	if (ret < 0) {
		debug0("request_irq(%d) failed %d", INT_AVE, ret);
		goto free_ave_code;
	}

	/* register chrdev */
	debug1("register_chrdev %d, %s", devmajor, AVE_MODNAME);
	ret = register_chrdev(devmajor, AVE_MODNAME, &ave_fops);
	if (ret < 0) {
		debug0("register_chrdev %d, %s failed %d",
				devmajor, AVE_MODNAME, ret);
		ret = -EFAULT;
		goto free_irq_int;
	}

	ave_class = class_create(THIS_MODULE, AVE_MODNAME);
	if (IS_ERR(ave_class)) {
		debug0("class_create failed %d", ret);
		ret = PTR_ERR(ave_class);
		goto free_chrdev;
	}

	ave_class_device = device_create(ave_class, &dev->dev,
					 MKDEV(devmajor, 0),  NULL,
					 "%s", AVE_DEVNAME);
	if (IS_ERR(ave_class_device)) {
		debug0("class_device_create failed %s %d", AVE_DEVNAME, ret);
		class_destroy(ave_class);
		ret = PTR_ERR(ave_class_device);
		goto free_chrdev;
	}

	if (readl(SMU_PLL1CTRL1) != 0xFF)
		writel(0x00000201, SMU_AVECCLKDIV);
	else
		writel(0x00000301, SMU_AVECCLKDIV);

	ave_info->state = state_initial;

	ave_open_state = 0;
	debug1("success");
	return 0;

free_chrdev:
	unregister_chrdev(devmajor, AVE_MODNAME);

free_irq_int:
	free_irq(INT_AVE, ave_info);

free_ave_code:
	dma_free_coherent(0, sizeof(bit_code), virt, code_paddr);

free_common_info:
	dma_free_coherent(0, sizeof *common_info, common_info, cmn_paddr);

error_return:
	return ret;
}

static int ave_pf_remove(struct platform_device *dev)
{
	device_destroy(ave_class, MKDEV(devmajor, 0));
	class_destroy(ave_class);
	debug1("unregister_chrdev %d, %s", devmajor, AVE_MODNAME);
	unregister_chrdev(devmajor, AVE_MODNAME);
	free_irq(INT_AVE, ave_info);
	dma_free_coherent(0, sizeof *ave_info->common_info,
			  ave_info->common_info, ave_info->pa_common_info);
	dma_free_coherent(0, sizeof(bit_code),
			  ave_info->v_code_addr, ave_info->pa_code_addr);

	return 0;
}

static void ave_pf_release(struct device *dev)
{
	/* none */
}

/*!
 * initialize for insmod
 * @param void
 * @return void
 */
static int __init ave_init(void)
{
	int ret;

	ret = platform_device_register(&ave_pf_device);
	if (ret)
		return ret;
	return platform_driver_register(&ave_pf_driver);
}

/*!
 * finalize for rmmod
 * @param void
 * @return void
 */
static void __exit ave_exit(void)
{
	(void)platform_driver_unregister(&ave_pf_driver);
	(void)platform_device_unregister(&ave_pf_device);
}

module_init(ave_init);
module_exit(ave_exit);
module_param(devname, charp, 0444);
module_param(devmajor, int, 0444);
module_param(debug_level, int, 0444);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AVE Driver for EMMA Mobile series");
MODULE_AUTHOR("Renesas Electronics Corporation");
