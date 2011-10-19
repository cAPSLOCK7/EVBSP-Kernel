/*
 * File Name       : drivers/nts/emxx_nts_image.h
 * Function        : NTSC Driver (SIZ/ROT Control)
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


#ifndef _EMXX_NTS_IMAGE_H_
#define _EMXX_NTS_IMAGE_H_



/********************************************************
 *  Structure                                           *
 *******************************************************/
struct image_data {
	unsigned long ulPhysAddrYRGB;
	unsigned long ulPhysAddrUV;
	unsigned long ulPhysAddrV;
	unsigned int  uiWidth;
	unsigned int  uiHeight;
	unsigned int  uiX;
	unsigned int  uiY;
	unsigned int  uiScreenWidth;
	unsigned int  uiFormat;
};


/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
extern int  nts_image_initialize(struct emxx_nts_dev *ntsc);
extern void nts_image_finalize(void);
extern int  nts_image_clr_framebuf(struct image_data *buff);
extern int  nts_image_compose_request(void);



#endif /* _EMXX_NTS_IMAGE_H_ */
