/*
 *  File Name	    : arch-emxx/emxx_cam.h
 *  Function	    : CAMERA I/F Driver
 *  Release Version : Ver 0.01
 *  Release Date    : 2010/11/05
 *
 *  Copyright (C) Renesas Electronics Corporation 2010
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY;
 *  without even the implied warrnty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program;
 *  If not, write to the Free Software Foundation, Inc., 59 Temple Place -
 *  Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#ifndef EMXX_CAM_INC_H
#define EMXX_CAM_INC_H

#include <linux/videodev2.h>

/* V4L2 private controls */
#define EMXX_CID_IPU				   V4L2_CID_PRIVATE_BASE
	/* NOT SUPPORT */
#define EMXX_CID_PMMU                (V4L2_CID_PRIVATE_BASE + 1)
	/* NOT SUPPORT */
#define EMXX_CID_CA_MIRROR		   (V4L2_CID_PRIVATE_BASE + 2)
#define EMXX_CID_CA_BNGR			   (V4L2_CID_PRIVATE_BASE + 3)
#define EMXX_CID_CA_CBGR			   (V4L2_CID_PRIVATE_BASE + 4)
#define EMXX_CID_CA_CRGR			   (V4L2_CID_PRIVATE_BASE + 5)
#define EMXX_CID_CA_BNZR			   (V4L2_CID_PRIVATE_BASE + 6)
#define EMXX_CID_CA_CBZR			   (V4L2_CID_PRIVATE_BASE + 7)
#define EMXX_CID_CA_CRZR			   (V4L2_CID_PRIVATE_BASE + 8)
#define EMXX_RJ6ABA100_CID_FW			   (V4L2_CID_PRIVATE_BASE + 9)
#define EMXX_RJ6ABA100_CID_OUTIMG_SIZE		   (V4L2_CID_PRIVATE_BASE + 10)
#define EMXX_RJ6ABA100_CID_OUTIMG_FORMAT	   (V4L2_CID_PRIVATE_BASE + 11)
#define EMXX_RJ6ABA100_CID_OUTDATA_FORMAT	   (V4L2_CID_PRIVATE_BASE + 12)
#define EMXX_RJ6ABA100_CID_FLICKER		   (V4L2_CID_PRIVATE_BASE + 13)
#define EMXX_RJ6ABA100_CID_FLICKER_AUTO_DETECTION (V4L2_CID_PRIVATE_BASE + 14)
	/* NOT SUPPORT */
#define EMXX_RJ6ABA100_CID_AE			   (V4L2_CID_PRIVATE_BASE + 15)
	/* NOT SUPPORT */
#define EMXX_RJ6ABA100_CID_AE_MANUAL		   (V4L2_CID_PRIVATE_BASE + 16)
	/* NOT SUPPORT */
#define EMXX_RJ6ABA100_CID_WB			   (V4L2_CID_PRIVATE_BASE + 17)
#define EMXX_RJ6ABA100_CID_WB_MANUAL_GAIN	   (V4L2_CID_PRIVATE_BASE + 18)
#define EMXX_RJ6ABA100_CID_BRIGHTNESS		   (V4L2_CID_PRIVATE_BASE + 19)
#define EMXX_RJ6ABA100_CID_CONTRAST		   (V4L2_CID_PRIVATE_BASE + 20)
#define EMXX_RJ6ABA100_CID_SHARPNESS		   (V4L2_CID_PRIVATE_BASE + 21)
#define EMXX_RJ6ABA100_CID_MIRROR		   (V4L2_CID_PRIVATE_BASE + 22)
#define EMXX_RJ6ABA100_CID_EFFECT_COLOR	   (V4L2_CID_PRIVATE_BASE + 23)
#define EMXX_RJ6ABA100_CID_EFFECT_COLOR_MANUAL	   (V4L2_CID_PRIVATE_BASE + 24)
#define EMXX_RJ6ABA100_CID_EFFECT_NEGATIVE	   (V4L2_CID_PRIVATE_BASE + 25)
#define EMXX_RJ6ABA100_CID_EFFECT_EMBOSS	   (V4L2_CID_PRIVATE_BASE + 26)
#define EMXX_RJ6ABA100_CID_EFFECT_SKETCH	   (V4L2_CID_PRIVATE_BASE + 27)
#define EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL   (V4L2_CID_PRIVATE_BASE + 28)

#endif

