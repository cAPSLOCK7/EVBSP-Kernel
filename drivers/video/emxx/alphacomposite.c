/*
 * File Name       : drivers/video/emxx/alphacomposite.c
 * Function        : It's the function of the entrance
 *                   the user carries out to do alpha blending.
 * Release Version : Ver 1.01
 * Release Date    : 2010.06.16
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

#include "alphacomposite.h"
#include "neon_alphablend.h"
#include <linux/stddef.h>

typedef void (*COMPOSITE_FUNC)(struct AlphaCompositeArgs_t *arg);

/* #define DP(fmt,arg...) printk(fmt,##arg); */
#define DP(fmt, arg...)

static int common_check(struct AlphaCompositeArgs_t  *arg)
{
	/* pointer */
	if (NULL == arg ||
		NULL == arg->src_ptr   ||
		NULL == arg->src_b_ptr ||
		NULL == arg->dst_ptr)
		/* pointer invalid. */
		return -1;

	/* pixel size */
	if (8192 < arg->dst_vsize ||
		8192 < arg->dst_hsize ||
		0   >= arg->dst_vsize ||
		0   >= arg->dst_hsize)
		/* size not supported or restricted by specification. */
		return -2;

	return 0;
}

static int type2_check(struct AlphaCompositeArgs_t *arg)
{
	int err;
	unsigned int srcb_min_length, src_min_length;

	err = common_check(arg);
	if (err) {
		/* return if found error */
		return err;
	}

	switch (arg->srcfmt_f) {
	case ALPHACOMPOSITE_FMT_ABGR8888:
		src_min_length = arg->dst_hsize*4;
		break;
	default:
		return -4;
	}

	switch (arg->srcfmt_b) {
	case ALPHACOMPOSITE_FMT_ABGR8888:
		srcb_min_length = arg->dst_hsize*4;
		break;
	case ALPHACOMPOSITE_FMT_BGR888:
		srcb_min_length = arg->dst_hsize*3;
		break;
	case ALPHACOMPOSITE_FMT_RGB565:
		srcb_min_length = arg->dst_hsize*2;
		break;
	default:
		return -4;
	}

	/* minimum size */
	if (src_min_length > arg->src_size ||
		srcb_min_length > arg->src_b_size ||
		srcb_min_length > arg->dst_size)
		/* line size not valid. */
		return -3;

	if (src_min_length  == arg->src_size &&
		srcb_min_length == arg->src_b_size &&
		srcb_min_length == arg->dst_size)
		/* execute special mode */
		return 1;

	return 0;
}

static int type2(struct AlphaCompositeArgs_t *arg)
{
	int err = 0;
	COMPOSITE_FUNC func;

	err = type2_check(arg);
	if (err < 0)
		return err;

	/* determine function. */
	switch (arg->srcfmt_b) {
	case ALPHACOMPOSITE_FMT_ABGR8888:
		func = AlphaComposit_neon_type2_0_0_0;
		break;
	case ALPHACOMPOSITE_FMT_BGR888:
		func = AlphaComposit_neon_type2_0_1_1;
		break;
	case ALPHACOMPOSITE_FMT_RGB565:
	default:
		func = AlphaComposit_neon_type2_0_2_2;
		break;
	}

	/* execute program */
	if (err > 0) {
		/* special mode. */
		struct AlphaCompositeArgs_t tmp = *arg;
		tmp.dst_hsize *= tmp.dst_vsize;
		tmp.dst_vsize  = 1;
		(*func)(&tmp);
	} else
		(*func)(arg);

	return 1;
}


int image_alpha_composite(struct AlphaCompositeArgs_t *arg)
{
	DP("type     : %d\n", arg->type);
	DP("srcfmt_f : %d\n", arg->srcfmt_f);
	DP("srcfmt_b : %d\n", arg->srcfmt_b);
	DP("dstfmt   : %d\n", arg->dstfmt);
	DP("src_ptr  : %p\n", arg->src_ptr);
	DP("src_b_ptr: %p\n", arg->src_b_ptr);
	DP("dst_ptr  : %p\n", arg->dst_ptr);
	DP("src_size : %d\n", arg->src_size);
	DP("src_b_size:%d\n", arg->src_b_size);
	DP("dst_size : %d\n", arg->dst_size);
	DP("dst_hsize: %d\n", arg->dst_hsize);
	DP("dst_vsize: %d\n", arg->dst_vsize);

	if (2 == arg->type)
		return type2(arg);
	else
		return -4;
}

