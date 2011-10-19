/*
 * File Name       : /drivers/video/emxx/struct.h
 * Function        : This file defines used macro to define a structure.
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

#ifndef _MACRO_DEFINE_STRUCT
#define _MACRO_DEFINE_STRUCT

/*----------------------------------
 *define for structure reference
 ---------------------------------*/
#define __l__struct_size(x)       $_sizeof_##x
#define __l__struct(x, y)         $_##x##_$##y

#define __struct_size(x)       __l__struct_size(##x##)
#define __struct(x, y)         __l__struct(##x##, ##y##)

/*----------------------------------
 *define for declare structure
 ---------------------------------*/
#define __l__struct_define(y, z) \
	$_\struct_name\()_$\postname\()##y## : .struct . + z
#define __l__struct_struct(y, x) \
	$_\struct_name\()_$\postname\()##y## : struct_##x ., \postname\()##y##., \struct_name
#define __l__struct_top(x)       .macro struct_##x adr = 0, postname = , struct_name = ##x

#define __struct_define(y, z) __l__struct_define(y, ##z##)
#define __struct_struct(y, x) __l__struct_struct(##y##, ##x##)
#define __struct_top(x)       __l__struct_top(##x##)

#define __struct_btm         .endm
#define __struct_begin       .struct \adr

.macro usestruct structname
  struct_\structname
  $_sizeof_\structname :
.endm

/*----------------------------------
 *define for initialize structure
 ---------------------------------*/
.macro initstruct       structname
	.ifndef $_sizeof_\structname
		.err
	.endif
	.struct 0
.endm

.macro initstruct_byte  structname, member, data
	.ifndef $_\structname\()_$\member
		.err
	.endif
	.skip $_\structname\()_$\member - (.struct .)
	.byte \data
.endm
.macro initstruct_short  structname, member, data
	.ifndef $_\structname\()_$\member
		.err
	.endif
	.skip $_\structname\()_$\member - .
	.short \data
.endm
.macro initstruct_long  structname, member, data
	.ifndef $_\structname\()_$\member
		.err
	.endif
	.skip $_\structname\()_$\member - .
	.long \data
.endm
.macro initstruct_end structname
	.ifndef $_sizeof_\structname
		.err
	.endif
	.skip $_sizeof_\structname - .
.endm

/*----------------------------------
 * examples
 *
 * __struct_top(X)
 * __struct_begin
 *   __struct_define(A1,4)  // X.A1 has zero and size 4 byte
 *   __struct_define(A2,4)  // X.A2 has 4    and size 4 byte
 *	 __struct_struct(A3,OX) // X.A3 has 8    and size was structur OX
 * __struct_end
 *
 * __struct(X,A1)     // refer X.A1
 * __struct(X,A2)     // refer X.A2
 * __struct_size(X)   // size of struct X
 ---------------------------------*/


#endif


