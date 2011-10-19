/*
 *  File Name       : drivers/nts/ntsc_encoder.h
 *  Function        : NTSC video encoder for NTSC I/F Driver
 * Release Version : Ver 1.00
 * Release Date    : 2011.02.17
 *
 * Copyright (C) 2011 Renesas Electronics Corporation
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


#ifndef _NTSC_ENCODER_H_
#define _NTSC_ENCODER_H_

/********************************************************
 *  Prototype declarations of function                  *
 *******************************************************/
extern int  ntsc_encoder_init(struct emxx_nts_dev *ntsc);
extern void ntsc_encoder_exit(struct emxx_nts_dev *ntsc);

#endif /* _NTSC_ENCODER_H_ */

