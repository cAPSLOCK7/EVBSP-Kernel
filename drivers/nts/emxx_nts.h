/*
 * File Name       : drivers/nts/emxx_nts.h
 * Function        : NTSC Driver
 * Release Version : Ver 1.00
 * Release Date    : 2010.09.24
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


#ifndef _EMXX_NTS_H_
#define _EMXX_NTS_H_



/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
extern int          emxx_nts_blank(int blank_mode);
extern int          emxx_nts_reserve(int iActiveDevice, int iSetOutmode);
extern int          emxx_nts_release(int iActiveDevice);
extern int          emxx_nts_getmode(void);
extern int          emxx_nts_set_v4l2_image(V4L2_IMAGE_INFO *v4l2_info);
extern int          emxx_nts_set_fb_image(FB_IMAGE_INFO *fb_info);
extern void         nts_irq_handler_sub(void);
extern void         nts_return_callback(void);
extern void         nts_return_callback_refresh(void);
extern void         nts_return_callback_ready(void);

#if defined(CONFIG_PM) || defined(CONFIG_DPM)
extern int          emxx_nts_suspend(struct platform_device *dev,
 pm_message_t state);
extern int          emxx_nts_resume(struct platform_device *dev);
#endif /* CONFIG_PM || CONFIG_DPM */

extern void         emxx_v4l2_ntsc_ready_callback(FRAME_DATA frame_data);


#endif /* _EMXX_NTS_H_ */
