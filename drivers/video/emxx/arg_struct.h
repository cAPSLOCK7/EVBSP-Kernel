/*
 * File Name       : drivers/video/emxx/arg_struct.h
 * Function        : the structure used by alpha blending for assembler.
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

#include "struct.h"

__struct_top(_AlphaARG)
__struct_begin
	__struct_define(type          , (4)*(1))
	__struct_define(srcfmt_f      , (4)*(1))
	__struct_define(srcfmt_b      , (4)*(1))
	__struct_define(dstfmt        , (4)*(1))
	__struct_define(src_ptr       , (4)*(1))
	__struct_define(src_b_ptr     , (4)*(1))
	__struct_define(dst_ptr       , (4)*(1))
	__struct_define(src_size      , (4)*(1))
	__struct_define(src_b_size    , (4)*(1))
	__struct_define(dst_size      , (4)*(1))
	__struct_define(dst_hsize     , (4)*(1))
	__struct_define(dst_vsize     , (4)*(1))
__struct_btm

