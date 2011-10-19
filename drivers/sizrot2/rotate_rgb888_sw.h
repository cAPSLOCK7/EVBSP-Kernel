/*
 *  File Name       : drivers/sizrot2/rotate_rgb888_sw.h
 *  Function        : ROT Driver
 *  Release Version : Ver 1.00
 *  Release Date    : 2010.04.19
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
static void rotate_rgb888_sw(struct work_struct *num);


/* ------------------------------------------- */
/*   Private functions                         */
/* ------------------------------------------- */
/*****************************************************************************
* MODULE   : rotate_rgb888_sw
* FUNCTION :
* RETURN   :
* NOTE     : none
******************************************************************************/
static void rotate_rgb888_sw(struct work_struct *num)
{
	struct rot_info *info =
	 container_of(num, struct rot_info, wk_rotate_rgb888_sw);

	unsigned long src_hsize = info->dma_param.src_hsize;
	unsigned long src_vsize = info->dma_param.src_vsize;
	unsigned long src_skip  = info->dma_param.src_hskipyrgb;
	unsigned long src_start = info->dma_param.src_adryrgb;
	unsigned long dst_start = readl(info->reg_base + ROT2_LCHx_DSTADRYRGB);

	unsigned long src_image_vsize = info->rot_param.src_vsize;
	unsigned long dst_image_start = info->rot_param.dst_adryrgb;
	unsigned long dst_image_leng  = info->rot_param.src_hsize *
					info->rot_param.src_vsize * 3;

	char *src_start_v, *dst_start_v, *dst_image_start_v;
	char *p_src, *p_dst;
	int i, j;

	dbg_printk((_DEBUG_SIZROT2 & 0x01), "rotate_rgb888_sw() <start>\n");

	src_start_v = ioremap_nocache(src_start,
				      (src_hsize * 3 + src_skip) * src_vsize);
	if (!src_start_v) {
		printk(KERN_INFO
		 "rotate_rgb888_sw: cannot ioremap_nocache src image buffer\n");
		goto err_return;
	}

	dst_image_start_v = ioremap_nocache(dst_image_start, dst_image_leng);
	if (!dst_image_start_v) {
		printk(KERN_INFO
		 "rotate_rgb888_sw: cannot ioremap_nocache dst image buffer\n");
		iounmap(src_start_v);
		goto err_return;
	}
	dst_start_v = dst_image_start_v + dst_start - dst_image_start;

	p_src = src_start_v;

	switch (info->rot_param.mode & ROT2_MODE_MODE_BIT) {
	default:
	case ROT2_MODE_MODE_0:
		dst_start_v += src_hsize * 3 * info->dma_line_count;
		p_dst = dst_start_v;
		for (i = 0; i < src_vsize; i++) {
			memcpy(p_dst, p_src, src_hsize * 3);
			p_src += src_hsize * 3 + src_skip;
			p_dst += src_hsize * 3;
		}
		break;
	case ROT2_MODE_MODE_90:
		dst_start_v -= info->dma_line_count * 3;
		for (i = 0; i < src_vsize; i++) {
			p_dst = dst_start_v - i * 3;
			for (j = 0; j < src_hsize; j++) {
				*p_dst       = *p_src;
				*(p_dst + 1) = *(p_src + 1);
				*(p_dst + 2) = *(p_src + 2);
				p_src += 3;
				p_dst += src_image_vsize * 3;
			}
			p_src += src_skip;
		}
		break;
	case ROT2_MODE_MODE_180:
		dst_start_v -= src_hsize * 3 * info->dma_line_count;
		p_dst = dst_start_v;
		for (i = 0; i < src_vsize; i++) {
			for (j = 0; j < src_hsize; j++) {
				*p_dst       = *p_src;
				*(p_dst + 1) = *(p_src + 1);
				*(p_dst + 2) = *(p_src + 2);
				p_src += 3;
				p_dst -= 3;
			}
			p_src += src_skip;
		}
		break;
	case ROT2_MODE_MODE_270:
		dst_start_v += info->dma_line_count * 3;
		for (i = 0; i < src_vsize; i++) {
			p_dst = dst_start_v + i * 3;
			for (j = 0; j < src_hsize; j++) {
				*p_dst       = *p_src;
				*(p_dst + 1) = *(p_src + 1);
				*(p_dst + 2) = *(p_src + 2);
				p_src += 3;
				p_dst -= src_image_vsize * 3;
			}
			p_src += src_skip;
		}
		break;
	}

	iounmap(src_start_v);
	iounmap(dst_image_start_v);

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
	goto done;

err_return:
	info->dma_running = DMA_CANCELED;
	if (info->callback)
		(info->callback)(M2M_DMA_CALLBACK_ERROR);
	else
		wake_up_interruptible(&info->wait_que_ioctl);

done:
	dbg_printk((_DEBUG_SIZROT2 & 0x02), "rotate_rgb888_sw() <end>\n");
	return;
}


