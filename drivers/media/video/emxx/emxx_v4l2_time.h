/*
 * File Name       : drivers/media/video/emxx/emxx_v4l2_time.h
 * Function        : Video for Linux driver for EM/EV driver core
 * Release Version : Ver 1.00
 * Release Date    : 2010.03.05
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

#ifndef EMXX_V4L2_TIME__H
#define EMXX_V4L2_TIME__H


struct emxx_v4l2_time;


/*===============================================================*/
/* make option definetion					 */
/*===============================================================*/
#define CALC_ADJUST	     0	/* 1: adjust(LCD-IMC process) enable
				   0: adjust(LCD-IMC process) disable */
#define ENABLE_DELAY	     0	/* 1: delay enable
				 * 0: delay disable */

#define ROUNDUP_ADJUST_LCD  16
#define ROUNDUP_ADJUST_NTSC 32
#define ROUNDUP_ADJUST_PAL  39
#define DEFAULT_ADJUST_LCD   0 /* adjust(LCD-IMC process)  time [msec] */
#define DEFAULT_ADJUST_NTSC 17 /* adjust(NTSC-IMG process) time [msec] */


/*===============================================================*/
/* rotate data structure                                         */
/*===============================================================*/
struct emxx_v4l2_time {
	struct timer_list       wait_timer;
	struct timeval		now_time;
	struct timeval		do_time;
	struct timeval		comp_time;
	long			wait_time;
	long			adjust_time_lcd;
	long			adjust_time_ntsc;
	void			*vb;
};


#endif /* EMXX_V4L2_TIME__H */
