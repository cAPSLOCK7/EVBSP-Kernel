/*
 *  File Name       : drivers/sizrot2/siz_drv.h
 *  Function        : SIZ Driver
 *  Release Version : Ver 1.10
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
static int  init_siz(void);
static void unreset_siz_hw(void);
static void reset_siz_hw(void);
static void clkctrl_off_siz_hw(void);
static void clkctrl_on_siz_hw(void);
static void init_siz_info(void);
static void update_filter(void);
static int  request_siz(struct emxx_siz_info *arg);
static int  set_siz(unsigned long id, struct emxx_siz_param *arg);
static int  check_siz_param(struct emxx_siz_param *arg);
static void set_filter(void);
static int  set_dma_to_siz(unsigned long id, struct emxx_dma_param *arg);
static int  start_dma_to_siz(unsigned long id, dma_callback_func callback);
static void do_start_dma_to_siz(struct siz_info *info);
static void do_strip_dma_to_siz(struct siz_info *info);
static void callback_dma_to_siz(void *data, int intsts, int intrawsts);
static inline void do_callback(struct siz_info *info);
static int  wait_siz(unsigned long id);
static int  reset_siz(unsigned long id);
static int  free_siz(unsigned long id);
static int  siz_ioctl(struct inode *inode, struct file *file,
 unsigned int request, unsigned long arg);


static int               dummy_dma_running = DMA_DONE;


/* ------------------------------------------- */
/*   Public functions                          */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : emxx_request_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_request_siz(struct emxx_siz_info *arg)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_request_siz() <start>\n");

	ret = request_siz(arg);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_request_siz() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_request_siz);


/*****************************************************************************
* MODULE   : emxx_set_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_set_siz(unsigned long id, struct emxx_siz_param *arg)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_set_siz() <start>\n");

	ret = set_siz(id, arg);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_set_siz() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_set_siz);


/*****************************************************************************
* MODULE   : emxx_free_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_free_siz(unsigned long id)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_free_siz() <start>\n");

	ret = free_siz(id);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_free_siz() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_free_siz);


/*****************************************************************************
* MODULE   : emxx_reset_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_reset_siz(unsigned long id)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_reset_siz() <start>\n");

	ret = reset_siz(id);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_reset_siz() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_reset_siz);


/*****************************************************************************
* MODULE   : emxx_set_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_set_dma_to_siz(unsigned long id, struct emxx_dma_param *arg)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "emxx_set_dma_to_siz() <start>\n");

	ret = set_dma_to_siz(id, arg);

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_set_dma_to_siz() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_set_dma_to_siz);


/*****************************************************************************
* MODULE   : emxx_start_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
int emxx_start_dma_to_siz(unsigned long id, dma_callback_func callback)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "emxx_start_dma_to_siz() <start>\n");

	if (callback == NULL) {
		printk(KERN_INFO
		 " @sizrot2: emxx_start_dma_to_siz: callback is NULL\n");
		ret = -EINVAL;
	} else {
		ret = start_dma_to_siz(id, callback);
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "emxx_start_dma_to_siz() <end> return (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL(emxx_start_dma_to_siz);


/* ------------------------------------------- */
/*   Private functions                         */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : init_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  init_siz(void)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "init_siz() <start>\n");

	unreset_siz_hw();
	init_siz_info();

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "init_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : unreset_siz_hw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void unreset_siz_hw(void)
{
	/* Unreset SIZ */
	emxx_clkctrl_off(EMXX_CLKCTRL_SIZ);
	emxx_clkctrl_off(EMXX_CLKCTRL_SIZPCLK);
	emxx_open_clockgate(EMXX_CLK_SIZ | EMXX_CLK_SIZ_P);
	emxx_unreset_device(EMXX_RST_SIZ);
	emxx_clkctrl_on(EMXX_CLKCTRL_SIZ);
	emxx_clkctrl_on(EMXX_CLKCTRL_SIZPCLK);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "unreset_siz_hw() <Unreset SIZ>\n");
}


/*****************************************************************************
* MODULE   : reset_siz_hw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void reset_siz_hw(void)
{
	/* Reset SIZ */
	emxx_clkctrl_off(EMXX_CLKCTRL_SIZ);
	emxx_clkctrl_off(EMXX_CLKCTRL_SIZPCLK);
	emxx_reset_device(EMXX_RST_SIZ);
	emxx_close_clockgate(EMXX_CLK_SIZ | EMXX_CLK_SIZ_P);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "reset_siz_hw() <Reset SIZ>\n");
}


/*****************************************************************************
* MODULE   : clkctrl_off_siz_hw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void clkctrl_off_siz_hw(void)
{
	emxx_clkctrl_off(EMXX_CLKCTRL_SIZ);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "clkctrl_off_siz_hw() <clock control off SIZ_CLK>\n");
}


/*****************************************************************************
* MODULE   : clkctrl_on_siz_hw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void clkctrl_on_siz_hw(void)
{
	emxx_clkctrl_on(EMXX_CLKCTRL_SIZ);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "clkctrl_on_siz_hw() <clock control on SIZ_CLK>\n");
}


/*****************************************************************************
* MODULE   : init_siz_info
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void init_siz_info(void)
{
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "init_siz_info() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "&siz_info = 0x%p\n", &siz_info);

	memset(&siz_info, 0, sizeof(struct siz_info));
	siz_info.reg_base         = (char *)IO_ADDRESS(EMXX_SIZ_BASE);
	siz_info.dma_channel_yrgb = EMXX_DMAC_M2M_ACPU_LCH5;
	siz_info.dma_channel_uv   = EMXX_DMAC_M2M_ACPU_LCH6;
	siz_info.dma_channel_v    = EMXX_DMAC_M2M_ACPU_LCH7;
	init_waitqueue_head(&siz_info.wait_que_ioctl);
	init_waitqueue_head(&siz_info.wait_que_resource);

	update_filter();

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "IO_ADDRESS(EMXX_SIZ_BASE) = 0x%p\n", siz_info.reg_base);

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "init_siz_info() <end>\n");
}


/*****************************************************************************
* MODULE   : update_filter
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void update_filter(void)
{
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "update_filter() <start>\n");

	siz_info.filter_linear.filtopt = SIZ_FILTOPT_2x2_LINEAR;
	siz_info.filter_linear.filt0 = SIZ_FILT0_2x2_LINEAR;
	siz_info.filter_linear.filt1 = SIZ_FILT1_2x2_LINEAR;
	siz_info.filter_linear.filt2 = SIZ_FILT2_2x2_LINEAR;
	siz_info.filter_linear.filt3 = SIZ_FILT3_2x2_LINEAR;
	siz_info.filter_linear.filt4 = SIZ_FILT4_2x2_LINEAR;
	siz_info.filter_linear.filt5 = SIZ_FILT5_2x2_LINEAR;
	siz_info.filter_linear.filt6 = SIZ_FILT6_2x2_LINEAR;
	siz_info.filter_linear.filt7 = SIZ_FILT7_2x2_LINEAR;

	siz_info.filter_sharpness.filtopt = SIZ_FILTOPT_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt0 = SIZ_FILT0_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt1 = SIZ_FILT1_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt2 = SIZ_FILT2_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt3 = SIZ_FILT3_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt4 = SIZ_FILT4_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt5 = SIZ_FILT5_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt6 = SIZ_FILT6_4x4_SHARPNESS;
	siz_info.filter_sharpness.filt7 = SIZ_FILT7_4x4_SHARPNESS;

	siz_info.filter_4x4_smoothing.filtopt = SIZ_FILTOPT_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt0 = SIZ_FILT0_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt1 = SIZ_FILT1_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt2 = SIZ_FILT2_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt3 = SIZ_FILT3_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt4 = SIZ_FILT4_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt5 = SIZ_FILT5_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt6 = SIZ_FILT6_4x4_SMOOTHING;
	siz_info.filter_4x4_smoothing.filt7 = SIZ_FILT7_4x4_SMOOTHING;

	siz_info.filter_2x2_smoothing.filtopt = SIZ_FILTOPT_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt0 = SIZ_FILT0_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt1 = SIZ_FILT1_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt2 = SIZ_FILT2_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt3 = SIZ_FILT3_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt4 = SIZ_FILT4_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt5 = SIZ_FILT5_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt6 = SIZ_FILT6_2x2_SMOOTHING;
	siz_info.filter_2x2_smoothing.filt7 = SIZ_FILT7_2x2_SMOOTHING;

	siz_info.coef_yuv_rgb.coef_r0 = SIZ_COEF_R0_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_r1 = SIZ_COEF_R1_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_r2 = SIZ_COEF_R2_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_r3 = SIZ_COEF_R3_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_g0 = SIZ_COEF_G0_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_g1 = SIZ_COEF_G1_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_g2 = SIZ_COEF_G2_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_g3 = SIZ_COEF_G3_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_b0 = SIZ_COEF_B0_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_b1 = SIZ_COEF_B1_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_b2 = SIZ_COEF_B2_YUV_RGB;
	siz_info.coef_yuv_rgb.coef_b3 = SIZ_COEF_B3_YUV_RGB;

	siz_info.coef_rgb_yuv.coef_r0 = SIZ_COEF_R0_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_r1 = SIZ_COEF_R1_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_r2 = SIZ_COEF_R2_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_r3 = SIZ_COEF_R3_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_g0 = SIZ_COEF_G0_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_g1 = SIZ_COEF_G1_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_g2 = SIZ_COEF_G2_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_g3 = SIZ_COEF_G3_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_b0 = SIZ_COEF_B0_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_b1 = SIZ_COEF_B1_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_b2 = SIZ_COEF_B2_RGB_YUV;
	siz_info.coef_rgb_yuv.coef_b3 = SIZ_COEF_B3_RGB_YUV;

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filtopt ="
	 " 0x%08lx\n", siz_info.filter_linear.filtopt);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt0   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt0);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt1   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt1);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt2   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt2);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt3   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt3);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt4   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt4);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt5   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt5);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt6   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt6);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_linear.filt7   ="
	 " 0x%08lx\n", siz_info.filter_linear.filt7);

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filtopt ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filtopt);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt0   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt0);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt1   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt1);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt2   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt2);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt3   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt3);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt4   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt4);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt5   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt5);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt6   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt6);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_sharpness.filt7   ="
	 " 0x%08lx\n", siz_info.filter_sharpness.filt7);

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filtopt ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filtopt);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt0   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt0);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt1   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt1);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt2   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt2);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt3   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt3);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt4   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt4);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt5   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt5);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt6   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt6);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_4x4_smoothing.filt7   ="
	 " 0x%08lx\n", siz_info.filter_4x4_smoothing.filt7);

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filtopt ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filtopt);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt0   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt0);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt1   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt1);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt2   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt2);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt3   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt3);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt4   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt4);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt5   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt5);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt6   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt6);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  siz_info.filter_2x2_smoothing.filt7   ="
	 " 0x%08lx\n", siz_info.filter_2x2_smoothing.filt7);

    dbg_printk((_DEBUG_SIZROT2 & 0x02), "update_filter() <end>\n");
}


/*****************************************************************************
* MODULE   : request_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  request_siz(struct emxx_siz_info *arg)
{
	int ret = 0;
	struct siz_info *info = &siz_info;
	unsigned char func = FUNC_SIZ;
	unsigned long min = ID_SIZ_MIN;
	unsigned long max = ID_SIZ_MAX;
	unsigned long flags;
	struct timeval start_time;
	struct timeval now_time;
	int delta_msec = 0;
	int waittime = 0;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "request_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  arg         = 0x%p\n", arg);

	if (arg == NULL) {
		printk(KERN_INFO " @sizrot2: request_siz: arg is NULL\n");
		ret = -EINVAL;
		goto request_siz_ret;
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
		 " @sizrot2: request_siz: arg->device is incorrect\n");
		ret = -EINVAL;
		goto request_siz_ret;
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
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);

			arg->id = info->sequence;
			arg->adryrgb = SIZ_ADRYRGB;
			arg->adruv   = SIZ_ADRUV;
			arg->adrv    = SIZ_ADRV;

			break;
		} else {
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);
		}
	} while (arg->timeout != 0 && delta_msec < arg->timeout);

	if (arg->id == 0) {
		printk(KERN_INFO " @sizrot2: request_siz: SIZ is busy\n");
		printk(KERN_INFO "               func   : %d\n", func);
		printk(KERN_INFO "               dev_now: %d\n", info->device);
		printk(KERN_INFO "               dev_err: %d\n", arg->device);
		ret = -EBUSY;
	}

request_siz_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "request_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : set_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  set_siz(unsigned long id, struct emxx_siz_param *arg)
{
	int ret = 0;
	struct siz_info *info = &siz_info;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "set_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id  = 0x%08lx\n", id);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  arg = 0x%p\n"   , arg);

	if (id != info->id) {
		printk(KERN_INFO " @sizrot2: set_siz: id is incorrect\n");
		ret = -ENODEV;
		goto set_siz_ret;
	}

	if (arg == NULL) {
		printk(KERN_INFO " @sizrot2: set_siz: arg is NULL\n");
		ret = -EINVAL;
		goto set_siz_ret;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_hsize       = %ld\n"    , arg->src_hsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_vsize       = %ld\n"    , arg->src_vsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->src_format      = %ld\n"    , arg->src_format);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_hsize       = %ld\n"    , arg->dst_hsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_vsize       = %ld\n"    , arg->dst_vsize);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_hskip       = %ld\n"    , arg->dst_hskip);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_adryrgb     = 0x%08lx\n", arg->dst_adryrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_adruv       = 0x%08lx\n", arg->dst_adruv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_adrv        = 0x%08lx\n", arg->dst_adrv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_format      = %ld\n"    , arg->dst_format);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_bytelane    = 0x%08lx\n", arg->dst_bytelane);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->hstep           = %ld\n"    , arg->hstep);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->vstep           = %ld\n"    , arg->vstep);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_hcrop       = %ld\n"    , arg->dst_hcrop);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->dst_vcrop       = %ld\n"    , arg->dst_vcrop);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->rot_dst_adryrgb = 0x%08lx\n", arg->rot_dst_adryrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->rot_dst_adruv   = 0x%08lx\n", arg->rot_dst_adruv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->rot_mode        = 0x%08lx\n", arg->rot_mode);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->rot_dst_format  = %ld\n"    , arg->rot_dst_format);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  arg->filter_option   = %ld\n"    , arg->filter_option);

	ret = check_siz_param(arg);
	if (ret != 0)
		goto set_siz_ret;

	memcpy(&info->siz_param, arg, sizeof(struct emxx_siz_param));

	set_filter();

	info->dma_line_count = 0;

set_siz_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "set_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : check_siz_param
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  check_siz_param(struct emxx_siz_param *arg)
{
	int ret = 0;
	int boundary_s_h = 1;
	int boundary_s_v = 1;
	int boundary_d_h = 1;
	int boundary_d_v = 1;
	int max_4x4 = 8190;
	int max_2x2 = 8190;
	int max = 8190;
	int yuv_flag = 0;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "check_siz_param() <start>\n");

	/*  filtopt */
#ifdef CONFIG_MACH_EMEV
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
		switch (arg->filter_option) {
		case SIZ_FILTOPT_MODE_4x4FILT_MINUS:
		case SIZ_FILTOPT_MODE_4x4FILT_PLUS:
		case SIZ_FILTOPT_MODE_2x2FILT_WITH_BUFFER:
		case SIZ_FILTOPT_MODE_2x2FILT:
		case SIZ_FILTER_DEFAULT:
		case SIZ_FILTER_SMOOTHING:
			break;
		default:
			ERR_PRINT_D("check_siz_param",
			 "filter_option", arg->filter_option);
			ret = -EINVAL;
			goto check_siz_param_ret;
			break;
		}
	} else {
#endif
		switch (arg->filter_option & 0xF) {
		case SIZ_FILTOPT_MODE_4x4FILT_MINUS:
		case SIZ_FILTOPT_MODE_4x4FILT_PLUS:
		case SIZ_FILTOPT_MODE_2x2FILT_WITH_BUFFER:
		case SIZ_FILTOPT_MODE_2x2FILT:
		case SIZ_FILTER_DEFAULT:
		case SIZ_FILTER_SMOOTHING:
			break;
		default:
			ERR_PRINT_D("check_siz_param",
			 "filter_option", arg->filter_option);
			ret = -EINVAL;
			goto check_siz_param_ret;
			break;
		}
		if (arg->filter_option & ~0x3F) {
			ERR_PRINT_D("check_siz_param",
			 "filter_option", arg->filter_option);
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
#ifdef CONFIG_MACH_EMEV
	}
#endif

	/* format */
	switch (arg->src_format) { /* source format for resizer channel */
	case SIZ_FORMAT_RGB888:
		boundary_s_h = 4;
		boundary_s_v = 1;
		yuv_flag = 0;
		break;
	case SIZ_FORMAT_RGB565:
		boundary_s_h = 2;
		boundary_s_v = 1;
		yuv_flag = 0;
		break;
	case SIZ_FORMAT_YUV422IL:
		boundary_s_h = 2;
		boundary_s_v = 1;
		yuv_flag = 1;
		break;
	case SIZ_FORMAT_YUV422SP:
	case SIZ_FORMAT_YUV420SP:
		boundary_s_h = 4;
		boundary_s_v = 1;
		yuv_flag = 1;
		break;
	case SIZ_FORMAT_YUV422PL:
	case SIZ_FORMAT_YUV420PL:
		boundary_s_h = 8;
		boundary_s_v = 1;
		yuv_flag = 1;
		break;
	default:
		ERR_PRINT_D("check_siz_param", "src_format", arg->src_format);
		ret = -EINVAL;
		goto check_siz_param_ret;
		break;
	}
	siz_info.change_format = CHG_NORMAL;
	switch (arg->dst_format) { /* dest. format for resizer channel */
	case SIZ_FORMAT_RGB888:
	case SIZ_FORMAT_RGB565:
		if (yuv_flag == 1)
			siz_info.change_format = CHG_YUV_RGB;
		boundary_d_h = 1;
		boundary_d_v = 1;
		break;
	case SIZ_FORMAT_YUV422IL:
	case SIZ_FORMAT_YUV422SP:
	case SIZ_FORMAT_YUV420SP:
	case SIZ_FORMAT_YUV422PL:
	case SIZ_FORMAT_YUV420PL:
		if (yuv_flag == 0)
			siz_info.change_format = CHG_RGB_YUV;
		boundary_d_h = 2;
		boundary_d_v = 1;
		break;
	default:
		ERR_PRINT_D("check_siz_param", "dst_format", arg->dst_format);
		ret = -EINVAL;
		goto check_siz_param_ret;
		break;
	}
	switch (arg->rot_dst_format) { /* dest. format for rotation channel */
	case SIZ_ROTDSTFMT_OFF:
		break;
	case SIZ_ROTDSTFMT_YUV422IL:
	case SIZ_ROTDSTFMT_YUV422SP:
	case SIZ_ROTDSTFMT_YUV420SP:
		if (arg->src_format != SIZ_FORMAT_YUV422IL) {
			ERR_PRINT_D("check_siz_param",
			 "src_format", arg->src_format);
			printk(KERN_INFO
			 " @sizrot2: src_format must be YUV422IL "
			 "when using Rotation channel.\n");
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		boundary_s_h = 4;
		boundary_s_v = 4;
		break;
	default:
		ERR_PRINT_D("check_siz_param",
		 "rot_dst_format", arg->rot_dst_format);
		break;
	}

	/* step */
	if ((arg->hstep < 256/256) || (arg->hstep >= 256*64)) {
		ERR_PRINT_D("check_siz_param", "hstep", arg->hstep);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if ((arg->vstep < 256/256) || (arg->vstep >= 256*64)) {
		ERR_PRINT_D("check_siz_param", "vstep", arg->vstep);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if (arg->rot_dst_format != SIZ_ROTDSTFMT_OFF) {
		/* when using Rotation channel */
		if (arg->hstep >= 256*4) {
			ERR_PRINT_D("check_siz_param", "hstep", arg->hstep);
			printk(KERN_INFO
			 " @sizrot2: hstep must be less than 256*4 "
			 "when using Rotation channel.\n");
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		if (arg->vstep >= 256*4) {
			ERR_PRINT_D("check_siz_param", "vstep", arg->vstep);
			printk(KERN_INFO
			 " @sizrot2: vstep must be less than 256*4 "
			 "when using Rotation channel.\n");
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		if ((arg->filter_option & 0xF) ==
		    SIZ_FILTOPT_MODE_2x2FILT_WITH_BUFFER ||
		    (arg->filter_option & 0xF) == SIZ_FILTOPT_MODE_2x2FILT) {
			ERR_PRINT_D("check_siz_param", "filter_option",
			 arg->filter_option);
			printk(KERN_INFO
			 " @sizrot2: filter_option cannot be 2x2 filter "
			 "when using Rotation channel.\n");
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
	}

	/* size: MIN. MAX. */
	if (arg->hstep < 256) {
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
			max_4x4 = 1024; max_2x2 = 2048;
		} else {
#endif
			max_4x4 = 2048; max_2x2 = 4096;
#ifdef CONFIG_MACH_EMEV
		}
#endif
	} else if (arg->hstep < 256 * 4) {
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1) {
			max_4x4 = 1024; max_2x2 = 2048;
		} else {
#endif
			max_4x4 = 2048; max_2x2 = 4096;
#ifdef CONFIG_MACH_EMEV
		}
#endif
	} else if (arg->hstep < 256 * 8) {
		max_4x4 = 4096; max_2x2 = 8190;
	} else if (arg->hstep < 256 * 16) {
		max_4x4 = 8190; max_2x2 = 8190;
	} else {
		max_4x4 = 8190; max_2x2 = 8190;
	}

	if (arg->rot_dst_format != SIZ_ROTDSTFMT_OFF) {
		/* when using Rotation channel */
		max = max_4x4;
	} else {
		max = max_2x2;
	}

	if ((arg->src_hsize < 2) || (arg->src_hsize > max)) {
		ERR_PRINT_D("check_siz_param", "src_hsize", arg->src_hsize);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if ((arg->src_vsize < 2) || (arg->src_vsize > 8190)) {
		ERR_PRINT_D("check_siz_param", "src_vsize", arg->src_vsize);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if ((arg->dst_hsize < 2) || (arg->dst_hsize > 8190)) {
		ERR_PRINT_D("check_siz_param", "dst_hsize", arg->dst_hsize);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if ((arg->dst_vsize < 2) || (arg->dst_vsize > 8190)) {
		ERR_PRINT_D("check_siz_param", "dst_vsize", arg->dst_vsize);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}

	if (arg->src_hsize > max_4x4) {
		if ((arg->filter_option & 0xF) ==
		    SIZ_FILTOPT_MODE_4x4FILT_MINUS ||
		    (arg->filter_option & 0xF) ==
		    SIZ_FILTOPT_MODE_4x4FILT_PLUS) {
			ERR_PRINT_D("check_siz_param",
			 "filter_option", arg->filter_option);
			printk(KERN_INFO
			 " @sizrot2: filter_option must be 2x2 filter.\n");
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		siz_info.filter.filtopt = SIZ_FILTOPT_MODE_2x2FILT;
	} else {
		siz_info.filter.filtopt = SIZ_FILTOPT_MODE_4x4FILT_PLUS;
	}

	/* size: boundary */
	if (arg->src_hsize % boundary_s_h) {
		ERR_PRINT_D("check_siz_param", "src_hsize", arg->src_hsize);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if (arg->src_vsize % boundary_s_v) {
		ERR_PRINT_D("check_siz_param", "src_vsize", arg->src_vsize);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if (arg->dst_hcrop > arg->dst_hsize) {
		printk(KERN_INFO
		 " @sizrot2: dst_hcrop must be less than dst_hsize\n");
		ERR_PRINT_D("check_siz_param", "dst_hsize", arg->dst_hsize);
		ERR_PRINT_D("check_siz_param", "dst_hcrop", arg->dst_hcrop);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if (arg->dst_vcrop > arg->dst_vsize) {
		printk(KERN_INFO
		 " @sizrot2: dst_vcrop must be less than dst_vsize\n");
		ERR_PRINT_D("check_siz_param", "dst_vsize", arg->dst_vsize);
		ERR_PRINT_D("check_siz_param", "dst_vcrop", arg->dst_vcrop);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if ((arg->dst_hsize - arg->dst_hcrop) % boundary_d_h) {
		printk(KERN_INFO " @sizrot2: (dst_hsize - dst_hcrop) "
		 "must keep boundary(%d)\n", boundary_d_h);
		ERR_PRINT_D("check_siz_param", "dst_hsize", arg->dst_hsize);
		ERR_PRINT_D("check_siz_param", "dst_hcrop", arg->dst_hcrop);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}
	if ((arg->dst_vsize - arg->dst_vcrop) % boundary_d_v) {
		printk(KERN_INFO " @sizrot2: (dst_vsize - dst_vcrop) "
		 "must keep boundary(%d)\n", boundary_d_v);
		ERR_PRINT_D("check_siz_param", "dst_vsize", arg->dst_vsize);
		ERR_PRINT_D("check_siz_param", "dst_vcrop", arg->dst_vcrop);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}

	/* bytelane */
	if (((arg->dst_bytelane & SIZ_DSTBL_DATA0_BIT) >> SIZ_DSTBL_DATA0_SFT)
	     == ((arg->dst_bytelane & SIZ_DSTBL_DATA1_BIT)
	     >> SIZ_DSTBL_DATA1_SFT) ||
	    ((arg->dst_bytelane & SIZ_DSTBL_DATA0_BIT) >> SIZ_DSTBL_DATA0_SFT)
	     == ((arg->dst_bytelane & SIZ_DSTBL_DATA2_BIT)
	     >> SIZ_DSTBL_DATA2_SFT) ||
	    ((arg->dst_bytelane & SIZ_DSTBL_DATA0_BIT) >> SIZ_DSTBL_DATA0_SFT)
	     == ((arg->dst_bytelane & SIZ_DSTBL_DATA3_BIT)
	     >> SIZ_DSTBL_DATA3_SFT) ||
	    ((arg->dst_bytelane & SIZ_DSTBL_DATA1_BIT) >> SIZ_DSTBL_DATA1_SFT)
	     == ((arg->dst_bytelane & SIZ_DSTBL_DATA2_BIT)
	     >> SIZ_DSTBL_DATA2_SFT) ||
	    ((arg->dst_bytelane & SIZ_DSTBL_DATA1_BIT) >> SIZ_DSTBL_DATA1_SFT)
	     == ((arg->dst_bytelane & SIZ_DSTBL_DATA3_BIT)
	     >> SIZ_DSTBL_DATA3_SFT) ||
	    ((arg->dst_bytelane & SIZ_DSTBL_DATA2_BIT) >> SIZ_DSTBL_DATA2_SFT)
	     == ((arg->dst_bytelane & SIZ_DSTBL_DATA3_BIT)
	     >> SIZ_DSTBL_DATA3_SFT)) {
		ERR_PRINT_X("check_siz_param",
		 "dst_bytelane", arg->dst_bytelane);
		ret = -EINVAL;
		goto check_siz_param_ret;
	}

	switch (arg->rot_dst_format) {
	default:
	case SIZ_ROTDSTFMT_OFF:
		/* DO NOTHING */
		break;
	case SIZ_ROTDSTFMT_YUV422SP:
	case SIZ_ROTDSTFMT_YUV420SP:
		/* dest. address(UV) for rotation channel */
		if (arg->rot_dst_adruv % 8) {
			ERR_PRINT_X("check_siz_param",
			 "rot_dst_adruv", arg->rot_dst_adruv);
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		/* FALL THROUGH */
	case SIZ_ROTDSTFMT_YUV422IL:
		/* dest. address(Y) for rotation channel */
		if (arg->rot_dst_adryrgb % 8) {
			ERR_PRINT_X("check_siz_param",
			 "rot_dst_adryrgb", arg->rot_dst_adryrgb);
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		/* mode for rotation channel */
		if (arg->rot_mode & ~(SIZ_ROTMODE_MODE_BIT |
				      SIZ_ROTMODE_XMIRROR_BIT |
				      SIZ_ROTMODE_YMIRROR_BIT)) {
			ERR_PRINT_X("check_siz_param",
			 "rot_mode", arg->rot_mode);
			ret = -EINVAL;
			goto check_siz_param_ret;
		}
		break;
	}

check_siz_param_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "check_siz_param() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : set_filter
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void set_filter(void)
{
	struct filter_param *filter;
	struct coef_param *coef;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "set_filter() <start>\n");

	if ((siz_info.siz_param.filter_option & 0xF) == SIZ_FILTER_DEFAULT) {
		/* SIZ Driver Default */
		if (siz_info.siz_param.rot_dst_format == SIZ_ROTDSTFMT_OFF)
			filter = &siz_info.filter_linear;
		else
			filter = &siz_info.filter_sharpness;
		memcpy(&siz_info.filter, filter, sizeof(struct filter_param));
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			siz_info.filter.filtopt |=
			 (siz_info.siz_param.filter_option &
			  SIZ_FILTOPT_FILTER_THROUGH_BIT);
	} else if ((siz_info.siz_param.filter_option & 0xF) ==
		   SIZ_FILTER_SMOOTHING) {
		/* SIZ Driver Default <Smoothing Filter> */
		if (siz_info.filter.filtopt == SIZ_FILTOPT_MODE_2x2FILT)
			filter = &siz_info.filter_2x2_smoothing;
		else
			filter = &siz_info.filter_4x4_smoothing;
		memcpy(&siz_info.filter, filter, sizeof(struct filter_param));
#ifdef CONFIG_MACH_EMEV
		if ((system_rev & EMXX_REV_MASK) != EMXX_REV_ES1)
#endif
			siz_info.filter.filtopt |=
			 (siz_info.siz_param.filter_option &
			  SIZ_FILTOPT_FILTER_THROUGH_BIT);
	} else {
		/* user settings */
		siz_info.filter.filtopt = siz_info.siz_param.filter_option;
		siz_info.filter.filt0   = siz_info.siz_param.filt0;
		siz_info.filter.filt1   = siz_info.siz_param.filt1;
		siz_info.filter.filt2   = siz_info.siz_param.filt2;
		siz_info.filter.filt3   = siz_info.siz_param.filt3;
		siz_info.filter.filt4   = siz_info.siz_param.filt4;
		siz_info.filter.filt5   = siz_info.siz_param.filt5;
		siz_info.filter.filt6   = siz_info.siz_param.filt6;
		siz_info.filter.filt7   = siz_info.siz_param.filt7;
	}

	if (siz_info.change_format == CHG_YUV_RGB) {
		siz_info.filter.filtopt |= SIZ_FILTOPT_YUV_RGB;
		coef = &siz_info.coef_yuv_rgb;
		memcpy(&siz_info.coef, coef, sizeof(struct coef_param));
	} else if (siz_info.change_format == CHG_RGB_YUV) {
		siz_info.filter.filtopt |= SIZ_FILTOPT_RGB_YUV;
		coef = &siz_info.coef_rgb_yuv;
		memcpy(&siz_info.coef, coef, sizeof(struct coef_param));
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "set_filter() <end>\n");
}


/*****************************************************************************
* MODULE   : set_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  set_dma_to_siz(unsigned long id, struct emxx_dma_param *arg)
{
	int ret = 0;
	struct siz_info *info = &siz_info;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "set_dma_to_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id  = 0x%08lx\n", id);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  arg = 0x%p\n"   , arg);

	if (id != info->id) {
		printk(KERN_INFO
		 " @sizrot2: set_dma_to_siz: id is incorrect\n");
		ret = -ENODEV;
		goto set_dma_to_siz_ret;
	}

	if (arg == NULL) {
		printk(KERN_INFO " @sizrot2: set_dma_to_siz: arg is NULL\n");
		ret = -EINVAL;
		goto set_dma_to_siz_ret;
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

	if ((arg->src_format == M2M_DMA_FORMAT_YUV420SP ||
	     arg->src_format == M2M_DMA_FORMAT_YUV420PL) &&
	    (arg->src_vsize % 2)) {
		printk(KERN_INFO " @sizrot2: set_dma_to_siz: "
		 "virtical size of YUV420 can not be odd number\n");
		ret = -EINVAL;
		goto set_dma_to_siz_ret;
	}

	if (arg->src_hsize != info->siz_param.src_hsize) {
		ERR_PRINT_D("set_dma_to_siz", "src_hsize", arg->src_hsize);
		printk(KERN_INFO " @sizrot2: "
		 "DMA src_hsize and SIZ src_hsize must be the same.\n");
		ret = -EINVAL;
		goto set_dma_to_siz_ret;
	}
#if 0
	if ((arg->src_format == M2M_DMA_FORMAT_YUV420SP ||
	     arg->src_format == M2M_DMA_FORMAT_YUV420PL) &&
	    (info->siz_param.src_vsize % 2)) {
		if (arg->src_vsize != (info->siz_param.src_vsize + 1)) {
			ERR_PRINT_D("set_dma_to_siz",
			 "src_vsize", arg->src_vsize);
			printk(KERN_INFO " @sizrot2: DMA src_vsize and "
			 "SIZ src_vsize(+1) must be the same.\n");
			ret = -EINVAL;
			goto set_dma_to_siz_ret;
		}
	} else {
		if (arg->src_vsize != info->siz_param.src_vsize) {
			ERR_PRINT_D("set_dma_to_siz",
			 "src_vsize", arg->src_vsize);
			printk(KERN_INFO " @sizrot2: DMA src_vsize and "
			 "SIZ src_vsize must be the same.\n");
			ret = -EINVAL;
			goto set_dma_to_siz_ret;
		}
	}
#else
	if ((arg->src_format == M2M_DMA_FORMAT_YUV420SP ||
	     arg->src_format == M2M_DMA_FORMAT_YUV420PL) &&
	    (info->siz_param.src_vsize % 2)) {
		if ((info->dma_line_count + arg->src_vsize) >
		    (info->siz_param.src_vsize + 1)) {
			ERR_PRINT_D("set_dma_to_siz",
			 "src_vsize", arg->src_vsize);
			printk(KERN_INFO " @sizrot2: DMA src_vsize must be "
			 "less than, or equal to SIZ src_vsize(+1).\n");
			ret = -EINVAL;
			goto set_dma_to_siz_ret;
		}
	} else {
		if ((info->dma_line_count + arg->src_vsize) >
		    info->siz_param.src_vsize) {
			ERR_PRINT_D("set_dma_to_siz",
			 "src_vsize", arg->src_vsize);
			printk(KERN_INFO " @sizrot2: DMA src_vsize must be "
			 "less than, or equal to SIZ src_vsize.\n");
			ret = -EINVAL;
			goto set_dma_to_siz_ret;
		}
	}
#endif
	if (arg->src_format != info->siz_param.src_format) {
		ERR_PRINT_D("set_dma_to_siz", "src_format", arg->src_format);
		printk(KERN_INFO " @sizrot2: DMA src_format and "
		 "SIZ src_format must be the same.\n");
		ret = -EINVAL;
		goto set_dma_to_siz_ret;
	}

	memcpy(&info->dma_param, arg, sizeof(struct emxx_dma_param));

set_dma_to_siz_ret:
	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "set_dma_to_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : start_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  start_dma_to_siz(unsigned long id, dma_callback_func callback)
{
	int ret = 0;
	struct siz_info *info = &siz_info;
	unsigned long flags;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "start_dma_to_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id       = 0x%08lx\n", id);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  callback = 0x%p\n"   , callback);

	if (id != info->id) {
		printk(KERN_INFO
		 " @sizrot2: start_dma_to_siz: id is incorrect\n");
		ret = -ENODEV;
	} else {
		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		if (info->dma_running == DMA_RUNNING) {
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);
			printk(KERN_INFO
			 " @sizrot2: start_dma_to_siz: DMA is busy\n");
			ret = -EBUSY;
		} else {
			info->dma_running = DMA_RUNNING;
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);

			info->callback = callback;

			switch (info->dma_param.src_format) {
			default:
			case M2M_DMA_FORMAT_RGB888:
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

			do_start_dma_to_siz(info);
		}
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "start_dma_to_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : do_start_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void do_start_dma_to_siz(struct siz_info *info)
{
	u32  bpp_yrgb, bpp_uv, strip_lines;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "do_start_dma_to_siz() <start>\n");

	/* status_ctrl_func(STAT_ON -> STAT_OFF)
	   while M2M-DMA and SIZ are running */
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "call status_ctrl_func(DRV_SIZ, STAT_ON)\n");
	status_ctrl_func(DRV_SIZ, STAT_ON);

	writel(info->siz_param.src_hsize,
	 info->reg_base + SIZ_SRCHSIZE);
	writel(info->siz_param.src_vsize,
	 info->reg_base + SIZ_SRCVSIZE);
	writel(info->siz_param.src_format,
	 info->reg_base + SIZ_SRCFMT);
	writel(info->siz_param.dst_hsize,
	 info->reg_base + SIZ_DSTHSIZE);
	writel(info->siz_param.dst_vsize,
	 info->reg_base + SIZ_DSTVSIZE);
	writel(info->siz_param.dst_hskip,
	 info->reg_base + SIZ_DSTHSKIP);
	writel(info->siz_param.dst_adryrgb,
	 info->reg_base + SIZ_DSTADRYRGB);
	writel(info->siz_param.dst_adruv,
	 info->reg_base + SIZ_DSTADRUV);
	writel(info->siz_param.dst_adrv,
	 info->reg_base + SIZ_DSTADRV);
	writel(info->siz_param.dst_format,
	 info->reg_base + SIZ_DSTFMT);
	writel(info->siz_param.dst_bytelane,
	 info->reg_base + SIZ_DSTBL);
	writel(info->siz_param.hstep,
	 info->reg_base + SIZ_HSTEP);
	writel(info->siz_param.vstep,
	 info->reg_base + SIZ_VSTEP);
	writel(info->siz_param.dst_hcrop,
	 info->reg_base + SIZ_DSTHCROP);
	writel(info->siz_param.dst_vcrop,
	 info->reg_base + SIZ_DSTVCROP);
	writel(info->siz_param.rot_dst_adryrgb,
	 info->reg_base + SIZ_ROTDSTADRYRGB);
	writel(info->siz_param.rot_dst_adruv,
	 info->reg_base + SIZ_ROTDSTADRUV);
	writel(info->siz_param.rot_mode,
	 info->reg_base + SIZ_ROTMODE);
	writel(info->siz_param.rot_dst_format,
	 info->reg_base + SIZ_ROTDSTFMT);

	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "  writel(0x%08lx, SIZ_FILTOPT)\n", info->filter.filtopt);
	writel(info->filter.filtopt, info->reg_base + SIZ_FILTOPT);
	writel(info->filter.filt0,   info->reg_base + SIZ_FILT0);
	writel(info->filter.filt1,   info->reg_base + SIZ_FILT1);
	writel(info->filter.filt2,   info->reg_base + SIZ_FILT2);
	writel(info->filter.filt3,   info->reg_base + SIZ_FILT3);
	writel(info->filter.filt4,   info->reg_base + SIZ_FILT4);
	writel(info->filter.filt5,   info->reg_base + SIZ_FILT5);
	writel(info->filter.filt6,   info->reg_base + SIZ_FILT6);
	writel(info->filter.filt7,   info->reg_base + SIZ_FILT7);

	if (info->change_format != CHG_NORMAL) {
		writel(info->coef.coef_r0, info->reg_base + SIZ_COEF_R0);
		writel(info->coef.coef_r1, info->reg_base + SIZ_COEF_R1);
		writel(info->coef.coef_r2, info->reg_base + SIZ_COEF_R2);
		writel(info->coef.coef_r3, info->reg_base + SIZ_COEF_R3);
		writel(info->coef.coef_g0, info->reg_base + SIZ_COEF_G0);
		writel(info->coef.coef_g1, info->reg_base + SIZ_COEF_G1);
		writel(info->coef.coef_g2, info->reg_base + SIZ_COEF_G2);
		writel(info->coef.coef_g3, info->reg_base + SIZ_COEF_G3);
		writel(info->coef.coef_b0, info->reg_base + SIZ_COEF_B0);
		writel(info->coef.coef_b1, info->reg_base + SIZ_COEF_B1);
		writel(info->coef.coef_b2, info->reg_base + SIZ_COEF_B2);
		writel(info->coef.coef_b3, info->reg_base + SIZ_COEF_B3);
	}


	switch (info->dma_param.src_format) {
	default:
	case M2M_DMA_FORMAT_RGB888:
		bpp_yrgb = 24;
		bpp_uv   = 0;
		break;
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

	info->aadd_yrgb = info->dma_param.src_adryrgb;
	info->aadd_uv   = info->dma_param.src_adruv;
	info->aadd_v    = info->dma_param.src_adrv;
	info->size_yrgb = info->dma_param.src_hsize * bpp_yrgb / 8;
	info->size_uv   = info->dma_param.src_hsize * bpp_uv   / 8;
	info->size_v    = info->dma_param.src_hsize * bpp_uv   / 8;
	info->badd_yrgb = SIZ_ADRYRGB
			  + (info->size_yrgb * info->dma_line_count);
	if ((info->dma_param.src_format == M2M_DMA_FORMAT_YUV420SP) ||
	    (info->dma_param.src_format == M2M_DMA_FORMAT_YUV420PL)) {
		info->badd_uv   = SIZ_ADRUV
				  + (info->size_uv * info->dma_line_count / 2);
		info->badd_v    = SIZ_ADRV
				  + (info->size_v  * info->dma_line_count / 2);
	} else {
		info->badd_uv   = SIZ_ADRUV
				  + (info->size_uv * info->dma_line_count);
		info->badd_v    = SIZ_ADRV
				  + (info->size_v  * info->dma_line_count);
	}

#if 0
	info->leng_yrgb =
	 info->dma_param.src_hsize * info->siz_param.src_vsize * bpp_yrgb / 8;
#else
	info->leng_yrgb =
	 info->dma_param.src_hsize * info->dma_param.src_vsize * bpp_yrgb / 8;
#endif
	strip_lines           = DMAC_LENG_MAX / info->size_yrgb;
	strip_lines           = strip_lines & ~1;
	info->strip_leng_yrgb = info->size_yrgb * strip_lines;
	info->cnt_strip       =
	 (info->leng_yrgb + info->strip_leng_yrgb - 1) / info->strip_leng_yrgb;
	if ((info->dma_param.src_format == M2M_DMA_FORMAT_YUV420SP) ||
	    (info->dma_param.src_format == M2M_DMA_FORMAT_YUV420PL)) {
		info->leng_uv = info->dma_param.src_hsize *
				info->dma_param.src_vsize * bpp_uv / 8 / 2;
		info->strip_leng_uv = info->size_uv   * strip_lines / 2;
		info->leng_v        = info->leng_uv;
		info->strip_leng_v  = info->size_v    * strip_lines / 2;
	} else {
		info->leng_uv = info->dma_param.src_hsize *
				info->dma_param.src_vsize * bpp_uv / 8;
		info->strip_leng_uv = info->size_uv   * strip_lines;
		info->leng_v        = info->leng_uv;
		info->strip_leng_v  = info->size_v    * strip_lines;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->size_yrgb       = %d\n", info->size_yrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->size_uv         = %d\n", info->size_uv);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->size_v          = %d\n", info->size_v);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "strip_lines           = %d\n", strip_lines);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->strip_leng_yrgb = %d\n", info->strip_leng_yrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->strip_leng_uv   = %d\n", info->strip_leng_uv);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->strip_leng_v    = %d\n", info->strip_leng_v);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "info->cnt_strip       = %d\n", info->cnt_strip);
	dbg_printk((_DEBUG_SIZROT2 & 0x08),
	 "\n");

	/* clock control off SIZ_CLK */
	clkctrl_off_siz_hw();

	do_strip_dma_to_siz(info);

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "do_start_dma_to_siz() <end>\n");
}


/*****************************************************************************
* MODULE   : do_strip_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void do_strip_dma_to_siz(struct siz_info *info)
{
	static dma_regs_t   *dmaregs_yrgb = DMA_NOT_INITIALIZED;
	static dma_regs_t   *dmaregs_uv   = DMA_NOT_INITIALIZED;
	static dma_regs_t   *dmaregs_v    = DMA_NOT_INITIALIZED;
	       unsigned int  dma_channel;
	       int           i;

	if (dmaregs_yrgb == DMA_NOT_INITIALIZED) {
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "do_strip_dma_to_siz: call emxx_request_dma(ch5)\n");
		emxx_request_dma(info->dma_channel_yrgb, sizrot2_dev_name,
		 callback_dma_to_siz, (void *)&siz_info, &dmaregs_yrgb);
	}

	if ((dmaregs_uv == DMA_NOT_INITIALIZED) && (info->cnt_callback >= 2)) {
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "do_strip_dma_to_siz: call emxx_request_dma(ch6)\n");
		emxx_request_dma(info->dma_channel_uv,   sizrot2_dev_name,
		 callback_dma_to_siz, (void *)&siz_info, &dmaregs_uv);
	}

	if ((dmaregs_v == DMA_NOT_INITIALIZED) && (info->cnt_callback >= 3)) {
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "do_strip_dma_to_siz: call emxx_request_dma(ch7)\n");
		emxx_request_dma(info->dma_channel_v,    sizrot2_dev_name,
		 callback_dma_to_siz, (void *)&siz_info, &dmaregs_v);
	}
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "do_strip_dma_to_siz: dmaregs_yrgb = %p\n", dmaregs_yrgb);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "do_strip_dma_to_siz: dmaregs_uv   = %p\n", dmaregs_uv);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "do_strip_dma_to_siz: dmaregs_v    = %p\n", dmaregs_v);

	info->strip_cnt_callback = info->cnt_callback;

	if (info->cnt_callback >= 2) {
		/* dummy transfer with LCH#0 */
		dummy_dma_running = DMA_RUNNING;
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "do_strip_dma_to_siz: call emxx_start_dma(dummy)\n");
		dmaregs_yrgb->aadd = info->aadd_yrgb;
		dmaregs_yrgb->badd = info->aadd_yrgb;
		dmaregs_yrgb->aoff = 0;
		dmaregs_yrgb->boff = 0;
		dmaregs_yrgb->size = 1;
		dmaregs_yrgb->leng = 1;
		dmaregs_yrgb->mode = EMXX_DMAC_DEFMODE_32BIT;
		emxx_start_m2m_dma(EMXX_M2M_DMA_LCH(5), 0);
		for (i = 0; i < 1000000; i++) {
			if (emxx_dma_status(EMXX_DMAC_M2M_ACPU_LCH5) == 0)
				break;
			udelay(1);
		}
		if (dummy_dma_running == DMA_CANCELED)
			return;
		dummy_dma_running = DMA_DONE;
	}

	dmaregs_yrgb->aadd = info->aadd_yrgb;
	dmaregs_yrgb->badd = info->badd_yrgb;
	dmaregs_yrgb->aoff = info->dma_param.src_hskipyrgb;
	dmaregs_yrgb->boff = 0;
	dmaregs_yrgb->size = info->size_yrgb;
	if (info->cnt_strip > 1) {
		dmaregs_yrgb->leng = info->strip_leng_yrgb;
		info->leng_yrgb -= info->strip_leng_yrgb;
		info->aadd_yrgb += (dmaregs_yrgb->leng / dmaregs_yrgb->size) *
				   (dmaregs_yrgb->size + dmaregs_yrgb->aoff);
		info->badd_yrgb += (dmaregs_yrgb->leng / dmaregs_yrgb->size) *
				   (dmaregs_yrgb->size + dmaregs_yrgb->boff);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs_yrgb->aadd = %x\n", dmaregs_yrgb->aadd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs_yrgb->badd = %x\n", dmaregs_yrgb->badd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs_yrgb->leng = %d\n", dmaregs_yrgb->leng);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "info->leng_yrgb    = %d\n", info->leng_yrgb);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "info->aadd_yrgb    = %x\n", info->aadd_yrgb);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "info->badd_yrgb    = %x\n", info->badd_yrgb);
		dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
	} else {
		dmaregs_yrgb->leng = info->leng_yrgb;
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs_yrgb->aadd = %x\n", dmaregs_yrgb->aadd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs_yrgb->badd = %x\n", dmaregs_yrgb->badd);
		dbg_printk((_DEBUG_SIZROT2 & 0x08),
		 "dmaregs_yrgb->leng = %d\n", dmaregs_yrgb->leng);
		dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
	}
	dmaregs_yrgb->mode = EMXX_DMAC_DEFMODE_32BIT;

	if (info->cnt_callback >= 2) {
		dmaregs_uv->aadd   = info->aadd_uv;
		dmaregs_uv->badd   = info->badd_uv;
		dmaregs_uv->aoff   = info->dma_param.src_hskipuv;
		dmaregs_uv->boff   = 0;
		dmaregs_uv->size   = info->size_uv;
		if (info->cnt_strip > 1) {
			dmaregs_uv->leng = info->strip_leng_uv;
			info->leng_uv -= info->strip_leng_uv;
			info->aadd_uv += (dmaregs_uv->leng / dmaregs_uv->size) *
					 (dmaregs_uv->size + dmaregs_uv->aoff);
			info->badd_uv += (dmaregs_uv->leng / dmaregs_uv->size) *
					 (dmaregs_uv->size + dmaregs_uv->boff);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_uv->aadd = %x\n", dmaregs_uv->aadd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_uv->badd = %x\n", dmaregs_uv->badd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_uv->leng = %d\n", dmaregs_uv->leng);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "info->leng_uv    = %d\n", info->leng_uv);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "info->aadd_uv    = %x\n", info->aadd_uv);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "info->badd_uv    = %x\n", info->badd_uv);
			dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
		} else {
			dmaregs_uv->leng   = info->leng_uv;
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_uv->aadd = %x\n", dmaregs_uv->aadd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_uv->badd = %x\n", dmaregs_uv->badd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_uv->leng = %d\n", dmaregs_uv->leng);
			dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
		}
		if ((info->dma_param.src_format == M2M_DMA_FORMAT_YUV420SP) ||
		    (info->dma_param.src_format == M2M_DMA_FORMAT_YUV420PL)) {
			dmaregs_uv->mode =
			 EMXX_DMAC_DEFMODE_32BIT | EMXX_DMAC_MODE_ORDER_UV;
		} else {
			dmaregs_uv->mode = EMXX_DMAC_DEFMODE_32BIT;
		}
	}

	if (info->cnt_callback >= 3) {
		dmaregs_v->aadd   = info->aadd_v;
		dmaregs_v->badd   = info->badd_v;
		dmaregs_v->aoff   = info->dma_param.src_hskipv;
		dmaregs_v->boff   = 0;
		dmaregs_v->size   = info->size_v;
		if (info->cnt_strip > 1) {
			dmaregs_v->leng = info->strip_leng_v;
			info->leng_v -= info->strip_leng_v;
			info->aadd_v += (dmaregs_v->leng / dmaregs_v->size) *
					 (dmaregs_v->size + dmaregs_v->aoff);
			info->badd_v += (dmaregs_v->leng / dmaregs_v->size) *
					 (dmaregs_v->size + dmaregs_v->boff);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_v->aadd = %x\n", dmaregs_v->aadd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_v->badd = %x\n", dmaregs_v->badd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_v->leng = %d\n", dmaregs_v->leng);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "info->leng_v    = %d\n", info->leng_v);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "info->aadd_v    = %x\n", info->aadd_v);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "info->badd_v    = %x\n", info->badd_v);
			dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
		} else {
			dmaregs_v->leng   = info->leng_v;
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_v->aadd = %x\n", dmaregs_v->aadd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_v->badd = %x\n", dmaregs_v->badd);
			dbg_printk((_DEBUG_SIZROT2 & 0x08),
			 "dmaregs_v->leng = %d\n", dmaregs_v->leng);
			dbg_printk((_DEBUG_SIZROT2 & 0x08), "\n");
		}
		if ((info->dma_param.src_format == M2M_DMA_FORMAT_YUV420SP) ||
		    (info->dma_param.src_format == M2M_DMA_FORMAT_YUV420PL)) {
			dmaregs_v->mode =
			 EMXX_DMAC_DEFMODE_32BIT | EMXX_DMAC_MODE_ORDER_UV;
		} else {
			dmaregs_v->mode = EMXX_DMAC_DEFMODE_32BIT;
		}
	}

	switch (info->cnt_callback) {
	default:
	case 1:
		dma_channel = EMXX_M2M_DMA_LCH(5);
		break;
	case 2:
		dma_channel = EMXX_M2M_DMA_LCH(5) | EMXX_M2M_DMA_LCH(6);
		break;
	case 3:
		dma_channel = EMXX_M2M_DMA_LCH(5) | EMXX_M2M_DMA_LCH(6) |
			      EMXX_M2M_DMA_LCH(7);
		break;
	}
	dbg_GetStartTime(1);
	dbg_GetStartTime(2);
	dbg_printk((_DEBUG_SIZROT2 & 0x01),
	 "do_strip_dma_to_siz: call emxx_start_m2m_dma(%d)\n", dma_channel);
	emxx_start_m2m_dma(dma_channel,
			   EMXX_DMAC_INT_LENG_EN);

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "do_strip_dma_to_siz() <end>\n");
}


/*****************************************************************************
* MODULE   : callback_dma_to_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void callback_dma_to_siz(void *data, int intsts, int intrawsts)
{
	struct siz_info *info = (struct siz_info *)data;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "callback_dma_to_siz() <start>\n");

	if (siz_info.dma_running == DMA_RUNNING) {
		info->strip_cnt_callback--;
		if (info->strip_cnt_callback == 0) {
			info->cnt_strip--;
			if (info->cnt_strip == 0)
				do_callback(info);
			else
				do_strip_dma_to_siz(info);
		}
	} else {
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "do nothing because everything has been done in reset_siz\n");
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02), "callback_dma_to_siz() <end>\n");
}


/*****************************************************************************
* MODULE   : do_callback
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static inline void do_callback(struct siz_info *info)
{
	int input_complete = 0;
	int ret;
	int result;
	unsigned long flags;

	info->dma_line_count += info->dma_param.src_vsize;

	if ((info->siz_param.src_format == M2M_DMA_FORMAT_YUV420SP ||
	     info->siz_param.src_format == M2M_DMA_FORMAT_YUV420PL) &&
	    (info->siz_param.src_vsize % 2)) {
		if (info->dma_line_count >= (info->siz_param.src_vsize + 1))
			input_complete = 1;
	} else {
		if (info->dma_line_count >= info->siz_param.src_vsize)
			input_complete = 1;
	}

	if (input_complete == 1) {
		info->dma_line_count = 0;

		dbg_GetStopTime(1, "M2M-DMA start - end")
		ret = wait_siz(info->id);
		dbg_GetStopTime(2, "M2M-DMA start - SIZ end")
		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		if (ret == 0) {
			info->dma_running = DMA_DONE;
			result = M2M_DMA_CALLBACK_SUCCESS;
		} else {
			printk(KERN_INFO "DMA->SIZ can not completed!\n");
			printk(KERN_INFO "reset SIZ!\n");
			reset_siz_hw();
			unreset_siz_hw();
			info->dma_running = DMA_CANCELED;
			result = M2M_DMA_CALLBACK_ERROR;
		}

		/* clock control on SIZ_CLK */
		clkctrl_on_siz_hw();

		/* status_ctrl_func(STAT_ON -> STAT_OFF)
		   while M2M-DMA and SIZ are running */
		dbg_printk((_DEBUG_SIZROT2 & 0x01),
		 "call status_ctrl_func(DRV_SIZ, STAT_OFF)\n");
		status_ctrl_func(DRV_SIZ, STAT_OFF);

		/* permit interrupts */
		spin_unlock_irqrestore(&sizrot2_lock, flags);
	} else {
		info->dma_running = DMA_DONE;
		result = M2M_DMA_CALLBACK_SUCCESS;
	}

	if (info->callback)
		(info->callback)(result);
	else
		wake_up_interruptible(&info->wait_que_ioctl);
}


/*****************************************************************************
* MODULE   : wait_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  wait_siz(unsigned long id)
{
	int ret = 0;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "wait_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id = 0x%08lx\n", id);

	if (id == siz_info.id) {
		int i;
		unsigned long state;
		for (i = 0; i < (MAX_TIME_TO_WAIT / 10); i++) {
			state = readl(siz_info.reg_base + SIZ_STAT);
			if ((state & SIZ_STAT_BIT) == SIZ_STAT_INACTIVE)
				break;
			udelay(10);
		}
		if (i == (MAX_TIME_TO_WAIT / 10)) {
			printk(KERN_INFO "Error: wait %d usec, but SIZ STAT "
			 "cannot be inactive!\n", MAX_TIME_TO_WAIT);
			ret = -EBUSY;
		} else {
#if (_DEBUG_SIZROT2 & 0x04)
			printk(KERN_INFO "Success: wait %d usec, and SIZ STAT "
			 "become inactive!\n", i*10);
#endif
			ret = 0;
		}
	} else {
		printk(KERN_INFO " @sizrot2: wait_siz: id is incorrect\n");
		ret = -ENODEV;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "wait_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : reset_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  reset_siz(unsigned long id)
{
	int ret = 0;
	unsigned long flags;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "reset_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id = 0x%08lx\n", id);

	if (id == siz_info.id) {
		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		if (siz_info.dma_running == DMA_RUNNING) {
			emxx_reset_dma(siz_info.dma_channel_yrgb);
			emxx_reset_dma(siz_info.dma_channel_uv);
			emxx_reset_dma(siz_info.dma_channel_v);

			reset_siz_hw();
			unreset_siz_hw();
			memset(&siz_info.siz_param, 0,
			 sizeof(struct emxx_siz_param));

			status_ctrl_func(DRV_SIZ, STAT_OFF);

			if (dummy_dma_running == DMA_RUNNING)
				dummy_dma_running = DMA_CANCELED;

			siz_info.dma_running = DMA_CANCELED;
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);

			if (siz_info.callback)
				(siz_info.callback)(M2M_DMA_CALLBACK_ERROR);
			else
				wake_up_interruptible(&siz_info.wait_que_ioctl);
		} else {
			reset_siz_hw();
			unreset_siz_hw();
			memset(&siz_info.siz_param, 0,
			 sizeof(struct emxx_siz_param));
			/* permit interrupts */
			spin_unlock_irqrestore(&sizrot2_lock, flags);
		}
	} else {
		printk(KERN_INFO " @sizrot2: reset_siz: id is incorrect\n");
		ret = -ENODEV;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "reset_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : free_siz
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static int  free_siz(unsigned long id)
{
	int ret = 0;
	unsigned long flags;
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "free_siz() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  id = 0x%08lx\n", id);

	if (id == siz_info.id) {
		if (wait_siz(id) == -EBUSY) {
			reset_siz(id);
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "free_siz: reset SIZ\n");
		}
		/* prohibit interrupts */
		spin_lock_irqsave(&sizrot2_lock, flags);
		siz_info.id = 0;
		wake_up_interruptible(&siz_info.wait_que_resource);
		memset(&siz_info.siz_param, 0, sizeof(struct emxx_siz_param));
		memset(&siz_info.dma_param, 0, sizeof(struct emxx_dma_param));
		/* permit interrupts */
		spin_unlock_irqrestore(&sizrot2_lock, flags);
	} else {
		printk(KERN_INFO " @sizrot2: free_siz: id is incorrect\n");
		ret = -ENODEV;
	}

	dbg_printk((_DEBUG_SIZROT2 & 0x02),
	 "free_siz() <end> return (%d)\n", ret);
	return ret;
}


/*****************************************************************************
* MODULE   : siz_ioctl
* FUNCTION : SIZ IOCTL main function
* RETURN   :       0  : success
*          : negative : fail
* NOTE     : none
******************************************************************************/
int  siz_ioctl(struct inode *inode, struct file *file, unsigned int request,
 unsigned long arg)
{
	int ret = 0;
	char buf_data[MAX_SIZE_SIZROT2IOCTL];
	void *alloc_obj = &buf_data[0];

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "siz_ioctl() <start>\n");
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  request = 0x%08x\n", request);
	dbg_printk((_DEBUG_SIZROT2 & 0x01), "  arg     = 0x%08lx\n", arg);

	switch (request) {
	case EMXX_REQUEST_SIZ:
		{
			struct emxx_siz_info *p;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_REQUEST_SIZ\n");
			if (copy_from_user(alloc_obj, (void *)arg,
			    sizeof(struct emxx_siz_info))) {
				ret = -EFAULT;
				break;
			} else {
				p = (struct emxx_siz_info *)alloc_obj;
				ret = request_siz(p);
				if (ret == 0) {
					if (copy_to_user((void *)arg, alloc_obj,
					    sizeof(struct emxx_siz_info))) {
						ret = -EFAULT;
						break;
					}
				}
				break;
			}
		}
	case EMXX_SET_SIZ:
		{
			struct emxx_set_siz_info *p;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_SET_SIZ\n");
			if (copy_from_user(alloc_obj, (void *)arg,
			    sizeof(struct emxx_set_siz_info))) {
				ret = -EFAULT;
				break;
			} else {
				p = (struct emxx_set_siz_info *)alloc_obj;
				ret = set_siz(p->id, &p->param);
				break;
			}
		}
	case EMXX_RESET_SIZ:
		{
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_RESET_SIZ\n");
			ret = reset_siz(arg);
			break;
		}
	case EMXX_FREE_SIZ:
		{
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_FREE_SIZ\n");
			ret = free_siz(arg);
			break;
		}
	case EMXX_SET_DMA_TO_SIZ:
		{
			struct emxx_set_dma_to_siz_info *p;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_SET_DMA_TO_SIZ\n");
			if (copy_from_user(alloc_obj, (void *)arg,
			    sizeof(struct emxx_set_dma_to_siz_info))) {
				ret = -EFAULT;
				break;
			} else {
				p = (struct emxx_set_dma_to_siz_info *)
				    alloc_obj;
				ret = set_dma_to_siz(p->id, &p->param);
				break;
			}
		}
	case EMXX_START_DMA_TO_SIZ:
		{
			struct siz_info *info = &siz_info;
			dbg_printk((_DEBUG_SIZROT2 & 0x01),
			 "  request: EMXX_START_DMA_TO_SIZ\n");
			ret = start_dma_to_siz(arg, NULL);
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
	 "siz_ioctl() <end> return (%d)\n", ret);
	return ret;
}


