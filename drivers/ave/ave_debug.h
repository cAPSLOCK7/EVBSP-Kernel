/*
 *  File Name       : ave_debug.h
 *  Function        : AVE debug
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

#ifndef __ave_debug_h__
#define __ave_debug_h__

/* debug functions ----------------------------------------------------------*/

extern int debug_level;

#define debug2_np(...) __debug_np(2, __VA_ARGS__)
#define debug0(...) __debug(0, __func__, __VA_ARGS__)
#define debug1(...) __debug(1, __func__, __VA_ARGS__)
#define debug2(...) __debug(2, __func__, __VA_ARGS__)
#define dump1(...)   __dump(1, __func__, __VA_ARGS__)
#define dump2(...)   __dump(2, __func__, __VA_ARGS__)
extern void __debug_np(int level, const char *format, ...);
extern void __debug(int level, const char *function, const char *format, ...);
extern void __dump(int level, const char *function, void *ptr, int len);

#endif /* __ave_debug_h__ */
