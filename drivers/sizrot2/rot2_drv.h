/*
 *  File Name       : drivers/sizrot2/rot2_drv.h
 *  Function        : ROT Driver
 *  Release Version : Ver 1.06
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


/* ------------------------------------------- */
/*   Private functions                         */
/* ------------------------------------------- */
static int  init_rot(void);
static void unreset_rot_hw(void);
#if 0 /* no more use */
static void reset_rot_hw(void);
#endif /* no more use */
static void init_rot_info(void);
static int  request_rot(int channel, struct emxx_rot_info *arg);
static int  set_rot(unsigned long id, struct emxx_rot_param *arg);
static int  check_rot_param(struct emxx_rot_param *arg);
static int  set_dma_to_rot(unsigned long id, struct emxx_dma_param *arg);
static int  start_dma_to_rot(unsigned long id, dma_callback_func callback);
static void do_start_dma_to_rot(struct rot_info *info);
static void do_strip_dma_to_rot(struct rot_info *info);
static void callback_dma_to_rot(void *data, int intsts, int intrawsts);
static int  free_rot(unsigned long id);
static int  rot2_ioctl(struct inode *inode, struct file *file,
 unsigned int request, unsigned long arg);


/* ------------------------------------------- */
/*   Public functions                          */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : emxx_request_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_request_rot(int channel, struct emxx_rot_info *arg)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_request_rot() <start>\n");

	ret = request_rot(channel, arg);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_request_rot() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_request_rot);


/*****************************************************************************
* MODULE   : emxx_set_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_set_rot(unsigned long id, struct emxx_rot_param *arg)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_set_rot() <start>\n");

	ret = set_rot(id, arg);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_set_rot() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_set_rot);


/*****************************************************************************
* MODULE   : emxx_free_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_free_rot(unsigned long id)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_free_rot() <start>\n");

	ret = free_rot(id);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_free_rot() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_free_rot);


/*****************************************************************************
* MODULE   : emxx_set_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_set_dma_to_rot(unsigned long id, struct emxx_dma_param *arg)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_set_dma_to_rot() <start>\n");

	ret = set_dma_to_rot(id, arg);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_set_dma_to_rot() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_set_dma_to_rot);


/*****************************************************************************
* MODULE   : emxx_start_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_start_dma_to_rot(unsigned long id, dma_callback_func callback)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "emxx_start_dma_to_rot() <start>\n");

	if (callback == NULL) {
		printk(KERN_INFO
		 " @sizrot2: emxx_start_dma_to_rot: callback is NULL\n");
		ret = -EINVAL;
	} else {
		ret = start_dma_to_rot(id, callback);
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_start_dma_to_rot() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_start_dma_to_rot);


/* ------------------------------------------- */
/*   Private functions                         */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : init_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int init_rot(void)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "init_rot() <start>\n");

	unreset_rot_hw();
	init_rot_info();

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "init_rot() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : unreset_rot_hw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void unreset_rot_hw(void)
{
	/* Unreset ROT2 */
	emxx_clkctrl_off(EMXX_CLKCTRL_ROT);
	emxx_clkctrl_off(EMXX_CLKCTRL_ROTPCLK);
	emxx_open_clockgate(EMXX_CLK_ROT | EMXX_CLK_ROT_P);
	emxx_unreset_device(EMXX_RST_ROT);
	emxx_clkctrl_on(EMXX_CLKCTRL_ROT);
	emxx_clkctrl_on(EMXX_CLKCTRL_ROTPCLK);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "unreset_rot_hw() <Unreset ROT2>\n");

	/* reset HISTCTRL register */
	writel(0, IO_ADDRESS(EMXX_ROT_BASE) + ROT2_HISTCTRL);
}


#if 0 /* no more use */
/*****************************************************************************
* MODULE   : reset_rot_hw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void reset_rot_hw(void)
{
	/* Reset ROT2 */
	emxx_clkctrl_off(EMXX_CLKCTRL_ROT);
	emxx_clkctrl_off(EMXX_CLKCTRL_ROTPCLK);
	emxx_reset_device(EMXX_RST_ROT);
	emxx_close_clockgate(EMXX_CLK_ROT | EMXX_CLK_ROT_P);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "reset_rot_hw() <Reset ROT2>\n");
}
#endif /* no more use */


/*****************************************************************************
* MODULE   : init_rot_info
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void init_rot_info(void)
{
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "init_rot_info() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "&rot_info = 0x%p\n", &rot_info);

	memset(&rot_info, 0, sizeof(struct rot_info));
	rot_info.reg_base    = (char *)IO_ADDRESS(EMXX_ROT_BASE);
	rot_info.dma_channel = EMXX_DMAC_M2M_ACPU_LCH4;
	init_waitqueue_head(&rot_info.wait_que_ioctl);
	init_waitqueue_head(&rot_info.wait_que_resource);

#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
		rot_info.rot_workqueue =
		 create_singlethread_workqueue("sizrot2");
		INIT_WORK(&rot_info.wk_rotate_rgb888_sw, rotate_rgb888_sw);
	}
#endif

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "IO_ADDRESS(EMXX_ROT_BASE) = 0x%p\n", rot_info.reg_base);

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "init_rot_info() <end>\n");
}


/*****************************************************************************
* MODULE   : request_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int request_rot(int channel, struct emxx_rot_info *arg)
{
	int ret = 0;
	struct rot_info *info = NULL;
	unsigned char func = 0;
	unsigned long min = 0;
	unsigned long max = 0;
	unsigned long flags;
	struct timeval start_time;
	struct timeval now_time;
	int delta_msec = 0;
	int waittime = 0;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "request_rot() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  channel = %d\n", channel);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  arg     = 0x%p\n" , arg);

	if (channel == ROT_LCH0) {
		info = &rot_info;
		func = FUNC_ROT_CH0;
		min  = ID_ROT_CH0_MIN;
		max  = ID_ROT_CH0_MAX;
	} else {
		printk(KERN_INFO
		 " @sizrot2: request_rot: channel is incorrect\n");
		ret = -EINVAL;
		goto request_rot_ret;
	}

	if (arg == NULL) {
		printk(KERN_INFO " @sizrot2: request_rot: arg is NULL\n");
		ret = -EINVAL;
		goto request_rot_ret;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->device = %d\n", arg->device);

	arg->id      = 0;
	arg->adryrgb = 0;
	arg->adruv   = 0;
	arg->adrv    = 0;

	switch (arg->device) {
	case DEV_FB:
	case DEV_LCD:
	case DEV_DSP:
	case DEV_CAM:
	case DEV_OTHER:
		break;
	default:
		printk(KERN_INFO
		 " @sizrot2: request_rot: arg->device is incorrect\n");
		ret = -EINVAL;
		goto request_rot_ret;
		break;
	}

	do_gettimeofday(&start_time);
	do {
		if (info->id != 0 && arg->timeout != 0) {
			waittime = (arg->timeout - delta_msec + 1000 / HZ - 1) /
				   (1000 / HZ);
			wait_event_interruptible_timeout(
			 info->wait_que_resource, info->id == 0, waittime);
			do_gettimeofday(&now_time);
			delta_msec =
			 (now_time.tv_sec - start_time.tv_sec) * 1000 +
			 (now_time.tv_usec - start_time.tv_usec) / 1000;
		}

		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		if (info->id == 0) {
			if (info->sequence == 0)
				info->sequence = min;
			else if (info->sequence == max)
				info->sequence = min;
			else
				info->sequence++;

			info->id = info->sequence;
			info->device = arg->device;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "call status_ctrl_func(DRV_ROT, STAT_ON)\n");
			status_ctrl_func(DRV_ROT, STAT_ON);
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);

			arg->id = info->sequence;
			arg->adryrgb = ROT2_LCH0_ADRYRGB;
			arg->adruv   = ROT2_LCH0_ADRUV;
			arg->adrv    = ROT2_LCH0_ADRV;

			/* reset HISTCTRL register */
			writel(0, IO_ADDRESS(EMXX_ROT_BASE) + ROT2_HISTCTRL);

			break;
		} else {
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);
		}
	} while (arg->timeout != 0 && delta_msec < arg->timeout);

	if (arg->id == 0) {
		printk(KERN_INFO " @sizrot2: request_rot: ROT is busy\n");
		printk(KERN_INFO "               func   : %d\n", func);
		printk(KERN_INFO "               dev_now: %d\n", info->device);
		printk(KERN_INFO "               dev_err: %d\n", arg->device);
		ret = -EBUSY;
	}

request_rot_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "request_rot() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : set_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int set_rot(unsigned long id, struct emxx_rot_param *arg)
{
	int ret = 0;
	struct rot_info *info = NULL;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "set_rot() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  id                = 0x%08lx\n", id);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg               = 0x%p\n"   , arg);

	if (id == rot_info.id) {
		info = &rot_info;
	} else {
		printk(KERN_INFO " @sizrot2: set_rot: id is incorrect\n");
		ret = -ENODEV;
		goto set_rot_ret;
	}

	if (arg == NULL) {
		printk(KERN_INFO " @sizrot2: set_rot: arg is NULL\n");
		ret = -EINVAL;
		goto set_rot_ret;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->mode         = 0x%08lx\n", arg->mode);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_hsize    = %ld\n"    , arg->src_hsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_vsize    = %ld\n"    , arg->src_vsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_format   = %ld\n"    , arg->src_format);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_adryrgb  = 0x%08lx\n", arg->dst_adryrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_adruv    = 0x%08lx\n", arg->dst_adruv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_adrv     = 0x%08lx\n", arg->dst_adrv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_bytelane = 0x%08lx\n", arg->dst_bytelane);

	ret = check_rot_param(arg);
	if (ret != 0)
		goto set_rot_ret;

	memcpy(&info->rot_param, arg, sizeof(struct emxx_rot_param));

	if (info->rot_param.input_mode == RANDOM_MODE) {
		/* Random Mode */
		if ((info->rot_param.src_format != ROT2_FORMAT_YUV422PL) &&
		    (info->rot_param.src_format != ROT2_FORMAT_YUV420PL))
			info->rot_param.dst_adrv = ROT2_DSTADRV_RANDOM_MODE;

		writel(info->rot_param.src_hsize,
		 info->reg_base + ROT2_LCHx_SRCHSIZE);
		writel(info->rot_param.src_vsize,
		 info->reg_base + ROT2_LCHx_SRCVSIZE);
		writel(info->rot_param.dst_adryrgb,
		 info->reg_base + ROT2_LCHx_DSTADRYRGB);
	} else {
		/* Raster Order Mode */
		unsigned long dst_adryrgb = info->rot_param.dst_adryrgb;

		info->rot_param.dst_adrv = ROT2_DSTADRV_RASTER_ORDER_MODE;

		switch (info->rot_param.mode & ROT2_MODE_MODE_BIT) {
		default:
		case ROT2_MODE_MODE_0:
			dst_adryrgb += 0;
			break;
		case ROT2_MODE_MODE_90:
			dst_adryrgb += (info->rot_param.src_vsize - 1) *
			 info->rot_param.src_format;
			break;
		case ROT2_MODE_MODE_180:
			dst_adryrgb += (info->rot_param.src_hsize *
			 info->rot_param.src_vsize - 1) *
			 info->rot_param.src_format;
			break;
		case ROT2_MODE_MODE_270:
			dst_adryrgb += (info->rot_param.src_hsize *
			 info->rot_param.src_vsize -
			 info->rot_param.src_vsize) *
			 info->rot_param.src_format;
			break;
		}

		writel(info->rot_param.src_hsize - 1,
		 info->reg_base + ROT2_LCHx_SRCHSIZE);
		writel(info->rot_param.src_vsize - 1,
		 info->reg_base + ROT2_LCHx_SRCVSIZE);
		writel(dst_adryrgb,
		 info->reg_base + ROT2_LCHx_DSTADRYRGB);
	}

	writel(info->rot_param.mode,
	 info->reg_base + ROT2_LCHx_MODE);
	writel(info->rot_param.src_format,
	 info->reg_base + ROT2_LCHx_SRCFMT);
	writel(info->rot_param.dst_adruv,
	 info->reg_base + ROT2_LCHx_DSTADRUV);
	writel(info->rot_param.dst_adrv,
	 info->reg_base + ROT2_LCHx_DSTADRV);
	writel(info->rot_param.dst_bytelane,
	 info->reg_base + ROT2_LCHx_DSTBL);

	writel(1, info->reg_base + ROT2_HISTCLR);

	info->dma_line_count = 0;

set_rot_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "set_rot() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : check_rot_param
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int check_rot_param(struct emxx_rot_param *arg)
{
	int ret = 0;
	int boundary_h = 1;
	int boundary_v = 1;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "check_rot_param() <start>\n");

	/* input_mode */
	if ((arg->input_mode != RANDOM_MODE) &&
	    (arg->input_mode != RASTER_MODE)) {
		ERR_PRINT_X("check_rot_param", "input_mode", arg->input_mode);
		ret = -EINVAL;
		goto check_rot_param_ret;
	}

	if (arg->input_mode == RANDOM_MODE) {	/* Random Mode */
		/* mode */
		if (arg->mode & ~(ROT2_MODE_MODE_BIT |
				  ROT2_MODE_XMIRROR_BIT |
				  ROT2_MODE_YMIRROR_BIT |
				  ROT2_MODE_BOUNDARY_BIT)) {
			ERR_PRINT_X("check_rot_param[Random Mode]",
			 "mode", arg->mode);
			ret = -EINVAL;
			goto check_rot_param_ret;
		}

		/* format */
		switch (arg->src_format) {
		case ROT2_FORMAT_RGB565:
			boundary_h = 2;
			boundary_v = 1;
			break;
		case ROT2_FORMAT_YUV422IL:
			boundary_h = 2;
			boundary_v = 2;
			break;
		case ROT2_FORMAT_YUV422SP:
		case ROT2_FORMAT_YUV420SP:
			boundary_h = 4;
			boundary_v = 2;
			break;
		case ROT2_FORMAT_YUV422PL:
		case ROT2_FORMAT_YUV420PL:
			boundary_h = 8;
			boundary_v = 2;
			break;
		default:
			ERR_PRINT_D("check_rot_param[Random Mode]",
			 "src_format", arg->src_format);
			ret = -EINVAL;
			goto check_rot_param_ret;
			break;
		}
	} else {				/* Raster Order Mode */
		/* mode */
		if (arg->mode & ~(ROT2_MODE_MODE_BIT)) {
			ERR_PRINT_X("check_rot_param[Raster Order Mode]",
			 "mode", arg->mode);
			ret = -EINVAL;
			goto check_rot_param_ret;
		}

		/* format */
		switch (arg->src_format) {
		case ROT2_FORMAT_RGB565_RASTER:
			boundary_h = 2;
			boundary_v = 1;
			break;
		case ROT2_FORMAT_RGB888_RASTER:
		/* case ROT2_FORMAT_YUV444_RASTER: (is the same value) */
			boundary_h = 4;
			boundary_v = 1;
			break;
		case ROT2_FORMAT_ARGB8888_RASTER:
			boundary_h = 1;
			boundary_v = 1;
			break;
		default:
			ERR_PRINT_D("check_rot_param[Raster Order Mode]",
			 "src_format", arg->src_format);
			ret = -EINVAL;
			goto check_rot_param_ret;
			break;
		}
	}

	/* size: MIN. MAX. */
	if ((arg->src_hsize < 2) || (arg->src_hsize > 8190)) {
		ERR_PRINT_D("check_rot_param", "src_hsize", arg->src_hsize);
		ret = -EINVAL;
		goto check_rot_param_ret;
	}
	if ((arg->src_vsize < 2) || (arg->src_vsize > 8190)) {
		ERR_PRINT_D("check_rot_param", "src_vsize", arg->src_vsize);
		ret = -EINVAL;
		goto check_rot_param_ret;
	}

	/* size: boundary */
	if (arg->src_hsize % boundary_h) {
		ERR_PRINT_D("check_rot_param", "src_hsize", arg->src_hsize);
		ret = -EINVAL;
		goto check_rot_param_ret;
	}
	if (arg->src_vsize % boundary_v) {
		ERR_PRINT_D("check_rot_param", "src_vsize", arg->src_vsize);
		ret = -EINVAL;
		goto check_rot_param_ret;
	}

	if (((arg->mode & ROT2_MODE_MODE_BIT) == ROT2_MODE_MODE_90) ||
	    ((arg->mode & ROT2_MODE_MODE_BIT) == ROT2_MODE_MODE_270)) {
		/* dest. size: boundary */
		if (arg->src_hsize % boundary_v) {
			ERR_PRINT_D("check_rot_param",
			 "src_hsize", arg->src_hsize);
			ret = -EINVAL;
			goto check_rot_param_ret;
		}
		if (arg->src_vsize % boundary_h) {
			ERR_PRINT_D("check_rot_param",
			 "src_vsize", arg->src_vsize);
			ret = -EINVAL;
			goto check_rot_param_ret;
		}
	}

	/* bytelane */
	if (((arg->dst_bytelane & ROT2_DSTBL_DATA0_BIT) >> ROT2_DSTBL_DATA0_SFT)
	     == ((arg->dst_bytelane & ROT2_DSTBL_DATA1_BIT)
	     >> ROT2_DSTBL_DATA1_SFT) ||
	    ((arg->dst_bytelane & ROT2_DSTBL_DATA0_BIT) >> ROT2_DSTBL_DATA0_SFT)
	     == ((arg->dst_bytelane & ROT2_DSTBL_DATA2_BIT)
	     >> ROT2_DSTBL_DATA2_SFT) ||
	    ((arg->dst_bytelane & ROT2_DSTBL_DATA0_BIT) >> ROT2_DSTBL_DATA0_SFT)
	     == ((arg->dst_bytelane & ROT2_DSTBL_DATA3_BIT)
	     >> ROT2_DSTBL_DATA3_SFT) ||
	    ((arg->dst_bytelane & ROT2_DSTBL_DATA1_BIT) >> ROT2_DSTBL_DATA1_SFT)
	     == ((arg->dst_bytelane & ROT2_DSTBL_DATA2_BIT)
	     >> ROT2_DSTBL_DATA2_SFT) ||
	    ((arg->dst_bytelane & ROT2_DSTBL_DATA1_BIT) >> ROT2_DSTBL_DATA1_SFT)
	     == ((arg->dst_bytelane & ROT2_DSTBL_DATA3_BIT)
	     >> ROT2_DSTBL_DATA3_SFT) ||
	    ((arg->dst_bytelane & ROT2_DSTBL_DATA2_BIT) >> ROT2_DSTBL_DATA2_SFT)
	     == ((arg->dst_bytelane & ROT2_DSTBL_DATA3_BIT)
	     >> ROT2_DSTBL_DATA3_SFT)) {
		ERR_PRINT_X("check_rot_param",
		 "dst_bytelane", arg->dst_bytelane);
		ret = -EINVAL;
		goto check_rot_param_ret;
	}

	/* dest. address */
	if (arg->input_mode == RANDOM_MODE) {	/* Random Mode */
		switch (arg->src_format) {
		case ROT2_FORMAT_YUV422PL:
		case ROT2_FORMAT_YUV420PL:
			if (arg->dst_adrv % 4) {
				ERR_PRINT_X("check_rot_param", "dst_adrv",
				 arg->dst_adrv);
				ret = -EINVAL;
				goto check_rot_param_ret;
			}
			/* FALL THROUGH */
		case ROT2_FORMAT_YUV422SP:
		case ROT2_FORMAT_YUV420SP:
			if (arg->dst_adruv % 4) {
				ERR_PRINT_X("check_rot_param", "dst_adruv",
				 arg->dst_adruv);
				ret = -EINVAL;
				goto check_rot_param_ret;
			}
			/* FALL THROUGH */
		case ROT2_FORMAT_RGB565:
		case ROT2_FORMAT_YUV422IL:
		default:
			if (arg->dst_adryrgb % 4) {
				ERR_PRINT_X("check_rot_param", "dst_adryrgb",
				 arg->dst_adryrgb);
				ret = -EINVAL;
				goto check_rot_param_ret;
			}
			break;
		}
	} else {				/* Raster Order Mode */
		if (arg->dst_adryrgb % 4) {
			ERR_PRINT_X("check_rot_param", "dst_adryrgb",
			 arg->dst_adryrgb);
			ret = -EINVAL;
			goto check_rot_param_ret;
		}
	}

check_rot_param_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "check_rot_param() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : set_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int set_dma_to_rot(unsigned long id, struct emxx_dma_param *arg)
{
	int ret = 0;
	struct rot_info *info = NULL;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "set_dma_to_rot() <start>\n");

	if (id == rot_info.id) {
		info = &rot_info;
	} else {
		printk(KERN_INFO
		 " @sizrot2: set_dma_to_rot: id is incorrect\n");
		ret = -ENODEV;
		goto set_dma_to_rot_ret;
	}

	if (arg == NULL) {
		printk(KERN_INFO " @sizrot2: set_dma_to_rot: arg is NULL\n");
		ret = -EINVAL;
		goto set_dma_to_rot_ret;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_hsize   = %ld\n"    , arg->src_hsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_vsize   = %ld\n"    , arg->src_vsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_hskipyrgb = %ld\n"    , arg->src_hskipyrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_hskipuv   = %ld\n"    , arg->src_hskipuv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_hskipv    = %ld\n"    , arg->src_hskipv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_adryrgb = 0x%08lx\n", arg->src_adryrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_adruv   = 0x%08lx\n", arg->src_adruv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_adrv    = 0x%08lx\n", arg->src_adrv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_format  = %ld\n"    , arg->src_format);

	if (arg->src_hsize != info->rot_param.src_hsize) {
		ERR_PRINT_D("set_dma_to_rot", "src_hsize", arg->src_hsize);
		printk(KERN_INFO " @sizrot2: "
		 "DMA src_hsize and ROT src_hsize must be the same.\n");
		ret = -EINVAL;
		goto set_dma_to_rot_ret;
	}
#if 0
	if (arg->src_vsize != info->rot_param.src_vsize) {
		ERR_PRINT_D("set_dma_to_rot", "src_vsize", arg->src_vsize);
		printk(KERN_INFO " @sizrot2: "
		 "DMA src_vsize and ROT src_vsize must be the same.\n");
		ret = -EINVAL;
		goto set_dma_to_rot_ret;
	}
#else
	if ((info->dma_line_count + arg->src_vsize) >
	    info->rot_param.src_vsize) {
		ERR_PRINT_D("set_dma_to_rot", "src_vsize", arg->src_vsize);
		printk(KERN_INFO " @sizrot2: DMA src_vsize must be "
		 "less than, or equal to ROT src_vsize.\n");
		ret = -EINVAL;
		goto set_dma_to_rot_ret;
	}
#endif
	if (arg->src_format != info->rot_param.src_format) {
		ERR_PRINT_D("set_dma_to_rot", "src_format", arg->src_format);
		printk(KERN_INFO " @sizrot2: "
		 "DMA src_format and ROT src_format must be the same.\n");
		ret = -EINVAL;
		goto set_dma_to_rot_ret;
	}

	memcpy(&info->dma_param, arg, sizeof(struct emxx_dma_param));

set_dma_to_rot_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "set_dma_to_rot() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : start_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int start_dma_to_rot(unsigned long id, dma_callback_func callback)
{
	int ret = 0;
	struct rot_info *info = NULL;
	unsigned long flags;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "start_dma_to_rot() <start>\n");

	if (id == rot_info.id) {
		info = &rot_info;
	} else {
		printk(KERN_INFO
		 " @sizrot2: start_dma_to_rot: id is incorrect\n");
		ret = -ENODEV;
	}

	if (info) {
		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		if (info->dma_running == DMA_RUNNING) {
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);
			printk(KERN_INFO
			 " @sizrot2: start_dma_to_rot: DMA is busy\n");
			ret = -EBUSY;
		} else {
			info->dma_running = DMA_RUNNING;
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);

			info->callback = callback;

			if (info->rot_param.input_mode == RANDOM_MODE) {
				/* Random Mode */
				switch (info->dma_param.src_format) {
				default:
				case M2M_DMA_FORMAT_RGB565:
				case M2M_DMA_FORMAT_YUV422IL:
					info->cnt_callback = 1;
					break;
				case M2M_DMA_FORMAT_YUV422SP:
				case M2M_DMA_FORMAT_YUV420SP:
					info->cnt_callback = 2;
					break;
				case M2M_DMA_FORMAT_YUV422PL:
				case M2M_DMA_FORMAT_YUV420PL:
					info->cnt_callback = 3;
					break;
				}
				switch (info->rot_param.mode &
					ROT2_MODE_BOUNDARY_BIT) {
				default:
				case ROT2_MODE_BOUNDARY_2_12:
					info->rot_boundary = 1 << 12; /* 2^12 */
					break;
				case ROT2_MODE_BOUNDARY_2_13:
					info->rot_boundary = 1 << 13; /* 2^13 */
					break;
				case ROT2_MODE_BOUNDARY_2_14:
					info->rot_boundary = 1 << 14; /* 2^14 */
					break;
				case ROT2_MODE_BOUNDARY_2_15:
					info->rot_boundary = 1 << 15; /* 2^15 */
					break;
				}
			} else {
				/* Raster Order Mode */
				info->cnt_callback = 1;
				info->rot_boundary = 0;
			}

#ifdef CONFIG_MACH_EMEV
			if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1 &&
			    info->rot_param.input_mode == RASTER_MODE &&
			    info->dma_param.src_format ==
			     M2M_DMA_FORMAT_RGB888_RASTER &&
			    (info->rot_param.mode & ROT2_MODE_MODE_BIT) !=
			     ROT2_MODE_MODE_0)
				queue_work(info->rot_workqueue,
				 &info->wk_rotate_rgb888_sw);
			else
#endif
				do_start_dma_to_rot(info);
		}
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "start_dma_to_rot() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : do_start_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void do_start_dma_to_rot(struct rot_info *info)
{
	u32  bpp_yrgb, bpp_uv, strip_lines;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "do_start_dma_to_rot() <start>\n");

	if (info->rot_param.input_mode == RANDOM_MODE) {
		/* Random Mode */
		switch (info->dma_param.src_format) {
		default:
		case M2M_DMA_FORMAT_RGB565:
		case M2M_DMA_FORMAT_YUV422IL:
			bpp_yrgb = 16;
			bpp_uv   = 0;
			break;
		case M2M_DMA_FORMAT_YUV422SP:
		case M2M_DMA_FORMAT_YUV420SP:
			bpp_yrgb = 8;
			bpp_uv   = 8;
			break;
		case M2M_DMA_FORMAT_YUV422PL:
		case M2M_DMA_FORMAT_YUV420PL:
			bpp_yrgb = 8;
			bpp_uv   = 4;
			break;
		}
	} else {
		/* Raster Order Mode */
		switch (info->dma_param.src_format) {
		default:
		case M2M_DMA_FORMAT_RGB565_RASTER:
			bpp_yrgb = 16;
			bpp_uv   = 0;
			break;
		case M2M_DMA_FORMAT_RGB888_RASTER:
		/* case M2M_DMA_FORMAT_YUV444_RASTER: (is the same value) */
			bpp_yrgb = 24;
			bpp_uv   = 0;
			break;
		case M2M_DMA_FORMAT_ARGB8888_RASTER:
			bpp_yrgb = 32;
			bpp_uv   = 0;
			break;
		}
	}

	switch (info->cnt_callback) {
	case 3:
		/* transfer V plane */
		info->aadd = info->dma_param.src_adrv;
		info->aoff = info->dma_param.src_hskipv;
		info->size = info->dma_param.src_hsize * bpp_uv   / 8;
		info->boff = info->rot_boundary * bpp_uv / bpp_yrgb -
			     info->size;
		if (info->dma_param.src_format == M2M_DMA_FORMAT_YUV420PL) {
			info->badd = ROT2_LCH0_ADRV + (info->size + info->boff)
				     * info->dma_line_count / 2;
			info->leng = info->dma_param.src_hsize *
				     info->dma_param.src_vsize * bpp_uv / 8 / 2;
		} else {
			info->badd = ROT2_LCH0_ADRV + (info->size + info->boff)
				     * info->dma_line_count;
			info->leng = info->dma_param.src_hsize *
				     info->dma_param.src_vsize * bpp_uv / 8;
		}
		break;
	case 2:
		/* transfer UV plane */
		info->aadd = info->dma_param.src_adruv;
		info->aoff = info->dma_param.src_hskipuv;
		info->size = info->dma_param.src_hsize * bpp_uv   / 8;
		info->boff = info->rot_boundary * bpp_uv / bpp_yrgb -
			     info->size;
		if ((info->dma_param.src_format == M2M_DMA_FORMAT_YUV420SP) ||
		    (info->dma_param.src_format == M2M_DMA_FORMAT_YUV420PL)) {
			info->badd = ROT2_LCH0_ADRUV + (info->size + info->boff)
				     * info->dma_line_count / 2;
			info->leng = info->dma_param.src_hsize *
				     info->dma_param.src_vsize * bpp_uv / 8 / 2;
		} else {
			info->badd = ROT2_LCH0_ADRUV + (info->size + info->boff)
				     * info->dma_line_count;
			info->leng = info->dma_param.src_hsize *
				     info->dma_param.src_vsize * bpp_uv / 8;
		}
		break;
	case 1:
		/* transfer Y or RGB plane */
		info->aadd = info->dma_param.src_adryrgb;
		info->aoff = info->dma_param.src_hskipyrgb;
		info->size = info->dma_param.src_hsize * bpp_yrgb / 8;
		if (info->rot_param.input_mode == RANDOM_MODE)
			info->boff = info->rot_boundary - info->size;
		else
			info->boff = 0;
		info->badd = ROT2_LCH0_ADRYRGB + (info->size + info->boff)
			     * info->dma_line_count;
		info->leng = info->dma_param.src_hsize *
			     info->dma_param.src_vsize * bpp_yrgb / 8;
	default:
		break;
	}
	strip_lines      = DMAC_LENG_MAX / info->size;
	strip_lines      = strip_lines & ~1;
	info->strip_leng = info->size * strip_lines;
	info->cnt_strip  =
	 (info->leng + info->strip_leng - 1) / info->strip_leng;

	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->size       = %d\n", info->size);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "strip_lines      = %d\n", strip_lines);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->strip_leng = %d\n", info->strip_leng);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->cnt_strip  = %d\n", info->cnt_strip);
	dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");

	do_strip_dma_to_rot(info);

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "do_start_dma_to_rot() <end>\n");
}


/*****************************************************************************
* MODULE   : do_strip_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void do_strip_dma_to_rot(struct rot_info *info)
{
	static dma_regs_t   *dmaregs = DMA_NOT_INITIALIZED;

	if (dmaregs == DMA_NOT_INITIALIZED) {
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "do_strip_dma_to_rot: call emxx_request_dma(ch4)\n");
		emxx_request_dma(info->dma_channel, sizrot2_dev_name,
		 callback_dma_to_rot, (void *)&rot_info, &dmaregs);
	}
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "do_strip_dma_to_rot: dmaregs = %p\n", dmaregs);

	dmaregs->aadd = info->aadd;
	dmaregs->badd = info->badd;
	dmaregs->aoff = info->aoff;
	dmaregs->boff = info->boff;
	dmaregs->size = info->size;
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "dmaregs->aoff = %d\n", dmaregs->aoff);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "dmaregs->boff = %d\n", dmaregs->boff);
	if (info->cnt_strip > 1) {
		dmaregs->leng = info->strip_leng;
		info->leng -= info->strip_leng;
		info->aadd += (dmaregs->leng / dmaregs->size) *
			      (dmaregs->size + dmaregs->aoff);
		info->badd += (dmaregs->leng / dmaregs->size) *
			      (dmaregs->size + dmaregs->boff);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs->aadd = %x\n", dmaregs->aadd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs->badd = %x\n", dmaregs->badd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs->leng = %d\n", dmaregs->leng);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "info->leng    = %d\n", info->leng);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "info->aadd    = %x\n", info->aadd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "info->badd    = %x\n", info->badd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
	} else {
		dmaregs->leng = info->leng;
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs->aadd = %x\n", dmaregs->aadd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs->badd = %x\n", dmaregs->badd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs->leng = %d\n", dmaregs->leng);
		dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
	}
	dmaregs->mode = EMXX_DMAC_DEFMODE_32BIT;

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "do_strip_dma_to_rot: call emxx_start_m2m_dma(%d)\n",
	 EMXX_M2M_DMA_LCH(4));
	emxx_start_m2m_dma(EMXX_M2M_DMA_LCH(4),
			   EMXX_DMAC_INT_LENG_EN);

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "do_strip_dma_to_rot() <end>\n");
}


/*****************************************************************************
* MODULE   : callback_dma_to_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void callback_dma_to_rot(void *data, int intsts, int intrawsts)
{
	struct rot_info *info = (struct rot_info *)data;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "callback_dma_to_rot() <start>\n");

	info->cnt_strip--;
	if (info->cnt_strip == 0) {
		info->cnt_callback--;
		if (info->cnt_callback > 0) {
			do_start_dma_to_rot(info);
		} else {
			info->dma_line_count += info->dma_param.src_vsize;

			if (info->dma_line_count >= info->rot_param.src_vsize) {
				info->dma_line_count = 0;
				udelay(1);
			}
			info->dma_running = DMA_DONE;
			if (info->callback)
				(info->callback)(M2M_DMA_CALLBACK_SUCCESS);
			else
				wake_up_interruptible(&info->wait_que_ioctl);
		}
	} else {
		do_strip_dma_to_rot(info);
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "callback_dma_to_rot() <end>\n");
}


/*****************************************************************************
* MODULE   : free_rot
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int free_rot(unsigned long id)
{
	int ret = 0;
	unsigned long flags;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "free_rot() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id = 0x%08lx\n", id);

	if (id == rot_info.id) {
		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		rot_info.id = 0;
		wake_up_interruptible(&rot_info.wait_que_resource);
		memset(&rot_info.rot_param, 0, sizeof(struct emxx_rot_param));
		memset(&rot_info.dma_param, 0, sizeof(struct emxx_dma_param));

		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "call status_ctrl_func(DRV_ROT, STAT_OFF)\n");
		status_ctrl_func(DRV_ROT, STAT_OFF);

		/* permit interrupts */
		spin_unlock_irqrestore(&sizrot2_lock, flags);
	} else {
		printk(KERN_INFO " @sizrot2: free_rot: id is incorrect\n");
		ret = -ENODEV;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "free_rot() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : rot2_ioctl
* FUNCTION : ROT IOCTL main function
* RETURN   :       0  : success
*          : negative : fail
* NOTE     : none
******************************************************************************/
int rot2_ioctl(struct inode *inode, struct file *file, unsigned int request,
 unsigned long arg)
{
	int ret = 0;
	char buf_data[MAX_SIZE_SIZROT2IOCTL];
	void *alloc_obj = &buf_data[0];

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "rot2_ioctl() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  request = 0x%08x\n", request);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  arg     = 0x%08lx\n", arg);

	switch (request) {
	case EMXX_REQUEST_ROT:
		{
			struct emxx_rot_info *p;
			int channel;

			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_REQUEST_ROT\n");

			channel = ROT_LCH0;

			if (copy_from_user(alloc_obj, (void *)arg,
			    sizeof(struct emxx_rot_info))) {
				ret = -EFAULT;
				break;
			} else {
				p = (struct emxx_rot_info *)alloc_obj;
				ret = request_rot(channel, p);
				if (ret == 0) {
					if (copy_to_user((void *)arg, alloc_obj,
					    sizeof(struct emxx_rot_info))) {
						ret = -EFAULT;
						break;
					}
				}
				break;
			}
		}
	case EMXX_SET_ROT:
		{
			struct emxx_set_rot_info *p;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_SET_ROT\n");
			if (copy_from_user(alloc_obj, (void *)arg,
			    sizeof(struct emxx_set_rot_info))) {
				ret = -EFAULT;
				break;
			} else {
				p = (struct emxx_set_rot_info *)alloc_obj;
				ret = set_rot(p->id, &p->param);
				break;
			}
		}
	case EMXX_FREE_ROT:
		{
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_FREE_ROT\n");
			ret = free_rot(arg);
			break;
		}
	case EMXX_SET_DMA_TO_ROT:
		{
			struct emxx_set_dma_to_rot_info *p;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_SET_DMA_TO_ROT\n");
			if (copy_from_user(alloc_obj, (void *)arg,
			    sizeof(struct emxx_set_dma_to_rot_info))) {
				ret = -EFAULT;
				break;
			} else {
				p = (struct emxx_set_dma_to_rot_info *)
				    alloc_obj;
				ret = set_dma_to_rot(p->id, &p->param);
				break;
			}
		}
	case EMXX_START_DMA_TO_ROT:
		{
			struct rot_info *info = &rot_info;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_START_DMA_TO_ROT\n");
			ret = start_dma_to_rot(arg, NULL);
			if (ret == 0) {
				wait_event_interruptible(info->wait_que_ioctl,
				 (info->dma_running != DMA_RUNNING));
				if (info->dma_running == DMA_DONE)
					ret = 0;
				else /* info->dma_running == DMA_CANCELED */
					ret = -ECANCELED;
			}
			break;
		}
	default:
		{
			ret = -EINVAL;
			break;
		}
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "rot2_ioctl() <end> return (%d)\n", ret);
	return ret;
}


