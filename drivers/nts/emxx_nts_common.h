/*
 * File Name       : drivers/nts/emxx_nts_common.h
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


#ifndef _EMXX_NTS_COMMON_H_
#define _EMXX_NTS_COMMON_H_


/********************************************************
 *  Definitions                                         *
 *******************************************************/
/* for iOutmodeFlg */
#define FB_OFF			0x01
#define FB_ON			0x02
#define V4L2_OFF		0x04
#define V4L2_ON			0x08

#define FB_V4L2_OFF		0x05
#define FB_ONLY			0x06
#define V4L2_ONLY		0x09
#define FB_V4L2_ON		0x0A

#define FB_BIT			0x03
#define V4L2_BIT		0x0C

/* display frame no. for iFrameNoNow/iFrameNoNext */
#define NTS_DISP_INIT		NTS_AREASEL_BUFDISABLE
#define NTS_DISP_FRAME_A	NTS_AREASEL_BUFA
#define NTS_DISP_FRAME_B	NTS_AREASEL_BUFB
#define NTS_DISP_FRAME_C	NTS_AREASEL_BUFC


/********************************************************
 *  Structure                                           *
 *******************************************************/
/* structure video endcoder */
struct encoder_reg {
	char   name[32];
	int    (*hw_init)(void *);
	void   (*hw_shutdown)(void *);
	int    (*hw_command)(unsigned int cmd, void *arg);
	void  *private_data;
};
#define ENCODER_REG struct encoder_reg

/* structure FBIOBLANK status */
struct nts_blank_state {
	int current_mode;
	int nts_backlight;
	int nts_output;
	int nts_clock;
};
#define NTS_BLANK_STATE struct nts_blank_state

/* structure nts device status */
struct emxx_nts_dev {
	/* semafore */
	 /* semafore v4l2/fb local data */
	struct semaphore         sem_image_data;
	 /* semafore v4l2/fb local flag */
	struct semaphore         sem_image_flag;
	/* spinlock */
	 /* spin lock for NTS               */
	spinlock_t               nts_lock;
	 /* spin lock for callback          */
	spinlock_t               lock_callback;

	/* wait queue */
	wait_queue_head_t        wait_compose;
	int                      compose_complete;

	/* flags */
	int                      iNtsActive;

	/* movie mix ON/OFF                */
	int                      iMixDSPFlg;

	/* need to callback to V4L2        */
	int                      iV4L2CallbackFlg;
	struct _FRAME_DATA       v4l2_callback_data;

	int                      iFrameNoNow;	/* now display frame no  */
	int                      iFrameNoNext;	/* set change frame no   */

	int                      iOutmodeFlg;	/* output i/f            */
	int                      iMixImageMode;	/*  */

	/* absolutely update flag    */
	unsigned int             uiAbsolutelyUpFlag;

	V4L2_IMAGE_INFO          from_v4l2;

	/* NTSC Encoder */
	ENCODER_REG             *encoder;

	/* FBIOBLANK */
	NTS_BLANK_STATE          blank_state;
	int                      ntsout_flg;

	/* Smem (common memory to NTS) */
	unsigned long Smem;
	unsigned long SmemLength;
	char         *SmemV;
};


#endif /* _EMXX_NTS_COMMON_H_ */
