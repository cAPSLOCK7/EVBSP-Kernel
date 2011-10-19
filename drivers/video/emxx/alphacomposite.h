/*
 * File Name       : drivers/video/emxx/alphacomposite.h
 * Function        : This file defines the structure used by alpha blending.
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

#ifndef ALPHACOMPOSITE_H
#define ALPHACOMPOSITE_H

struct AlphaCompositeArgs_t {
/* specify the type of alpha-blending  */
  int             type;
/* specify the front image data format */
  int             srcfmt_f;
/* specify the back image data format   (ignored except type2) */
  int             srcfmt_b;
/* specify the output image data format (ignored.)             */
  int             dstfmt;
/* specify the virtual memory address in which
   an input picture of front image is placed. */
  unsigned char  *src_ptr;
/* specify the virtual memory address in which
   an input picture of back image is placed. */
  unsigned char  *src_b_ptr;
/* specify the virtual memory address in which an output picture is placed. */
  unsigned char  *dst_ptr;
/* specify the length of one-row line of the input picture of front image. */
  unsigned int    src_size;
/* specify the length of one-row line of the input picture of back image. */
  unsigned int    src_b_size;
/* specify the length of one-row line of the output picture. */
  unsigned int    dst_size;
/* specify the width in pixel. */
  unsigned int    dst_hsize;
/* specify the heigh in pixel. */
  unsigned int    dst_vsize;
};

extern int  image_alpha_composite(struct AlphaCompositeArgs_t *arg);

/**************************************
type2:
   Cd = Cs'    + Cb*(1-As)
   Ad =     As + Ab*(1-As)

   Cd, Cs', Cb: color of component (Cs' is pre-alpha)
   As, Ad,  Ab : alpha of colors    (from 0 to 1.0)
**************************************/
#define ALPHACOMPOSITE_TYPE2   2

/*************************************
 Format
*************************************/
#define ALPHACOMPOSITE_FMT_ABGR8888  0
#define ALPHACOMPOSITE_FMT_BGR888    1
#define ALPHACOMPOSITE_FMT_RGB565    2

#endif

