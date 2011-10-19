/*
 * File Name       : drivers/video/emxx/neon_alphablend.h
 * Function        : Prototype declaration of the function described by
 *                   an assembler is enumerated.
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

#ifndef NEON_ALPHABLEND_H
#define NEON_ALPHABLEND_H

/***********************************************************************
 The following definition is given as a naming convention of the function.

 AlphaComposit_neon_<type>_<src1 fmt>_<src2_fmt>_<dst fmt>

 <type>
   the numerical value which corresponds to the kind of alpha blending
 <src1 Fmt>
   the numerical value which corresponds to a data format of a
   source picture in the front.
 <src2 Fmt>
   the numerical value which corresponds to a data format of
   a source picture in the back
 <dst fmt>
   The numerical value which corresponds to a data format of
   an output picture.
***********************************************************************/

extern void AlphaComposit_neon_type2_0_0_0(struct AlphaCompositeArgs_t *arg);
extern void AlphaComposit_neon_type2_0_1_1(struct AlphaCompositeArgs_t *arg);
extern void AlphaComposit_neon_type2_0_2_2(struct AlphaCompositeArgs_t *arg);

#endif

