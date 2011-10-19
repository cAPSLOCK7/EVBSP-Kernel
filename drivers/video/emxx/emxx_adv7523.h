/*
 * File Name       : drivers/video/emxx/emxx_adv7523.h
 * Function        : HDMI Driver (H/W Control)
 * Release Version : Ver 1.10
 * Release Date    : 2010.04.01
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


#ifndef _EMXX_ADV7523_H_
#define _EMXX_ADV7523_H_

/* debug functions ----------------------------------------------------------*/

#ifdef	CONFIG_EMXX_HDMI_ADV7523_DEBUG

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

#else /* !EMXX_HDMI_ADV7523_DEBUG */

#define debug2_np(...) /**/
#define debug0(...)  /**/
#define debug1(...)  /**/
#define debug2(...)  /**/
#define dump1(...)   /**/
#define dump2(...)   /**/

#endif /* !EMXX_HDMI_ADV7523_DEBUG */

/* state          ----------------------------------------------------------*/

#include <mach/emxx_hdmi_adv7523.h>

struct hdmi_adv7523_info_t {
	int open_state;
	int poweron_state;
	int intr;
	enum EMXX_HDMI_OUTPUT_MODE resolution;
};

#endif /* _EMXX_ADV7523_H_ */

