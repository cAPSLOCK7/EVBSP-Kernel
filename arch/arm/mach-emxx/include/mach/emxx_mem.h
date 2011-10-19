/*
 *  File Name       : arch/arm/mach-emxx/include/mach/emxx_mem.h
 *  Function        : memory map difinition
 *  Release Version : Ver 1.14
 *  Release Date    : 2011/01/18
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
 *
 */

#ifndef __EMEV_MEM_H
#define __EMEV_MEM_H

/******* SDRAM ********/

#ifdef CONFIG_MACH_EMEV

/* PMEM(40MB) */
#define EMXX_PMEM_BASE		0x48100000
#define EMXX_PMEM_SIZE		0x02800000

/* Camera frame buffer(2MB) */
#define CAMERA_FRAME_BASE	0x4A900000
#define CAMERA_FRAME_SIZE	0x00200000

/* FB frame buffer(12MB) */
#define FB_FRAME_BUFFER_ADDR	0x4AB00000
#define FB_FRAME_BUFFER_SIZE	0x01000000

/* Direct Path frame buffer(6MB) */
#define LCD_FRAME_BUFFER_ADDR	0x4BB00000
#define LCD_FRAME_BUFFER_SIZE	0x00600000

/* V4L2 ROT buffer(12MB) */
#define V4L2_ROT_BUFFER_ADDR	0x4C100000
#define V4L2_ROT_BUFFER_SIZE	0x00C00000

/* NTSC frame buffer(2MB) */
#define NTSC_FRAME_BUFFER_ADDR	0x4CD00000
#define NTSC_FRAME_BUFFER_SIZE	0x00200000

/* InterDSP */
#define INTERDSP_DL_TOP_ADDR	0x4CF00000 /* DSP-FW downlaod range (top) */
#define INTERDSP_DL_BOTTOM_ADDR	0x4FFDFFFF /*                       (end) */

/* CPU<->DSP command buffer(128K) */
#define INTERDSP_SHARED_ADDR	0x4FFE0000
#define INTERDSP_SHARED_SIZE	0x00020000

#else /* CONFIG_MACH_EMEV */

#ifdef CONFIG_EMGR_1G_MEM

/* kernel memory 71MB */

/* PMEM(16.2MB) */
#define EMXX_PMEM_BASE		0x44700000
#define EMXX_PMEM_SIZE		0x01038000

/* FB frame buffer(2.9MB) (800 * 480 * 4(ARGB8888) * 2frame) */
#define FB_FRAME_BUFFER_ADDR	0x45738000
#define FB_FRAME_BUFFER_SIZE	0x002EE000

/* Direct Path frame buffer(1.1MB) (800 * 480 * 3(RGB888) * 1frame) */
#define LCD_FRAME_BUFFER_ADDR	0x45A26000
#define LCD_FRAME_BUFFER_SIZE	0x0011A000

/* V4L2 ROT buffer(2.2MB) (800 * 480 * 2(YUV420) * 3frame) */
#define V4L2_ROT_BUFFER_ADDR	0x45B40000
#define V4L2_ROT_BUFFER_SIZE	0x00233000

/* NTSC frame buffer(1.6MB) (720 * 576 * 2(YUV420) * 2frame) */
#define NTSC_FRAME_BUFFER_ADDR	0x45D73000
#define NTSC_FRAME_BUFFER_SIZE	0x00195000

/* InterDSP */
#define INTERDSP_DL_TOP_ADDR	0x45F08000 /* DSP-FW downlaod range (top) */
#define INTERDSP_DL_BOTTOM_ADDR	0x47FDFFFF /*                       (end) */

/* CPU<->DSP command buffer(128K) */
#define INTERDSP_SHARED_ADDR	0x47FE0000
#define INTERDSP_SHARED_SIZE	0x00020000

#else /* CONFIG_EMGR_1G_MEM */

/* PMEM(40MB) */
#define EMXX_PMEM_BASE		0x4D800000
#define EMXX_PMEM_SIZE		0x02800000

/* FB frame buffer(12MB) */
#define FB_FRAME_BUFFER_ADDR	0x42000000
#define FB_FRAME_BUFFER_SIZE	0x00C00000

/* Direct Path frame buffer(6MB) */
#define LCD_FRAME_BUFFER_ADDR	0x42C00000
#define LCD_FRAME_BUFFER_SIZE	0x00600000

/* V4L2 ROT buffer(12MB) */
#define V4L2_ROT_BUFFER_ADDR	0x43200000
#define V4L2_ROT_BUFFER_SIZE	0x00C00000

/* NTSC frame buffer(2MB) */
#define NTSC_FRAME_BUFFER_ADDR	0x43E00000
#define NTSC_FRAME_BUFFER_SIZE	0x00200000

/* InterDSP */
#define INTERDSP_DL_TOP_ADDR	0x44E00000 /* DSP-FW downlaod range (top) */
#define INTERDSP_DL_BOTTOM_ADDR	0x47FDFFFF /*                       (end) */

/* CPU<->DSP command buffer(128K) */
#define INTERDSP_SHARED_ADDR	0x47FE0000
#define INTERDSP_SHARED_SIZE	0x00020000

#endif /* CONFIG_EMGR_1G_MEM */

#endif /* CONFIG_MACH_EMEV */

#define SHARED_MEM_ADDRESS 	INTERDSP_SHARED_ADDR
#define SHARED_MEM_SIZE 	INTERDSP_SHARED_SIZE
#define DOWNLOAD_RANGE_START 	INTERDSP_DL_TOP_ADDR
#define DOWNLOAD_RANGE_END 	INTERDSP_DL_BOTTOM_ADDR

#endif /* __EMEV_MEM_H */
