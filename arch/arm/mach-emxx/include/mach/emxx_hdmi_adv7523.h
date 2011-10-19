/*
 * File Name       : arch/arm/mach-emxx/include/mach/fbcommon.h
 * Function        : Common parameters for Frame Buffer Driver
 * Release Version : Ver 1.10
 * Release Date    : 2010.03.10
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
 *
 */

#ifndef _EMXX_HDMI_ADV7523_H_
#define _EMXX_HDMI_ADV7523_H_


/********************************************************
 * Include Files                                        *
 *******************************************************/
#include <linux/ioctl.h>
#include <linux/types.h>


/********************************************************
 * Structures                                           *
 *******************************************************/
/* HDMI output infomation */
enum EMXX_HDMI_OUTPUT_MODE{
	EMXX_HDMI_OUTPUT_MODE_LCD,		/* NOT_VALID     */
	/* default framerate: 60fps */
	EMXX_HDMI_OUTPUT_MODE_HDMI_1080I,	/* HDMI(1080i:1920x1080) */
	EMXX_HDMI_OUTPUT_MODE_HDMI_720P,	/* HDMI(720p:1280x720)   */
	/* specific framerate */
	EMXX_HDMI_OUTPUT_MODE_HDMI_1080I_50fps,	/* HDMI(1080i:1920x1080) */
	EMXX_HDMI_OUTPUT_MODE_HDMI_1080I_60fps,	/* HDMI(1080i:1920x1080) */
	EMXX_HDMI_OUTPUT_MODE_HDMI_720P_50fps,	/* HDMI(720p:1280x720)   */
	EMXX_HDMI_OUTPUT_MODE_HDMI_720P_60fps,	/* HDMI(720p:1280x720)   */
};


/********************************************************
 *  Definitions                                         *
 *******************************************************/
/*
 * IOCTLs to EMXX HDMI ADV7523 driver. 0x4E is 'N' for NEC.
 */
#define IOC_EMXX_HDMI_ADV7523_MAGIC	('N')

#define EMXX_HDMI_SET_OUTPUT \
	_IOW(IOC_EMXX_HDMI_ADV7523_MAGIC, 0x05, enum EMXX_HDMI_OUTPUT_MODE)
#define EMXX_HDMI_GET_OUTPUT \
	_IOR(IOC_EMXX_HDMI_ADV7523_MAGIC, 0x06, enum EMXX_HDMI_OUTPUT_MODE)


#endif /* _EMXX_HDMI_ADV7523_H_ */
