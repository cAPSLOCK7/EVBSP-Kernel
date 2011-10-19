/*
 * File Name       : drivers/video/emxx/emxx_fb_blit.h
 * Function        : FraemBuffer Driver
 * Release Version : Ver 1.00
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

#ifndef _EMXX_FB_BLIT_H_
#define _EMXX_FB_BLIT_H_


/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/

extern void emxx_fb_init_blit(void);
extern int emxx_fb_blit(struct fb_info *fb, struct emxx_fb_blit_req *req);


#endif /* _EMXX_FB_BLIT_H_ */
