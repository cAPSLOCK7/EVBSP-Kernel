/*
 * File Name       : arch/arm/mach-emxx/include/mach/emxx_imc.h
 * Function        : IMC Driver I/F definitions
 * Release Version : Ver 1.03
 * Release Date    : 2010.06.16
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

#ifndef _EMXX_IMC_H_
#define _EMXX_IMC_H_


/********************************************************
 *  Definitions                                         *
 *******************************************************/
#define IMC_CH0		0	/* IMC */
#define IMC_CH1		1	/* IMCW */

#define DEV_FB		1
#define DEV_LCD		2
#define DEV_DSP		3
#define DEV_CAM		4
#define DEV_OTHER	5


/********************************************************
 *  Macros                                              *
 *******************************************************/


/********************************************************
 *  Sub Structures                                      *
 *******************************************************/

/* ----- IMC Resource Information Structure ----------- */
struct emxx_imc_info {
	unsigned long id;
	unsigned long adr;
	unsigned long size;
	unsigned int  device;
	unsigned int  timeout;
};

/* ----- IMC GAMMA Setting Structure ------------------ */
struct imc_gamma_param {
	unsigned long en;
	unsigned long adr;
	unsigned long *data;
};

/* ----- IMC YUV Setting Structure -------------------- */
struct imc_yuv_param {
	unsigned long ygain;
	unsigned long ugain;
	unsigned long vgain;
	unsigned long yuv2rgb;
	unsigned long coef_r[4];
	unsigned long coef_g[4];
	unsigned long coef_b[4];
};

/* ----- IMC Burst Setting Structure ------------------ */
struct imc_burst_param {
	unsigned long burst_en;
	unsigned long threshold;
};

/* ----- IMC WB Setting Structure --------------------- */
struct imc_wb_param {
	unsigned long areaadr_p;
	unsigned long hoffset;
	unsigned long format;
	unsigned long size;
	unsigned long areaadr_q;
	unsigned long bufsel;
	unsigned long mposition;
	unsigned long msize;
	unsigned long color;
	unsigned long bytelane;
	unsigned long scanmode;
};

/* ----- IMC Alpha Setting Structure ------------------ */
struct imc_alphasel_param {
	unsigned long alphasel0;
	unsigned long alphasel1;
};

/* ----- IMC Layer0/1 Setting Structure --------------- */
struct l01_param {
	unsigned long control;
	unsigned long format;
	unsigned long bufsel;
	unsigned long bytelane;
	unsigned long keyenable;
	unsigned long keycolor;
	unsigned long alpha;
	unsigned long resize;
	unsigned long mirror;
	unsigned long offset;
	unsigned long frameadr_p;
	unsigned long frameadr_q;
	unsigned long position;
	unsigned long size;
	unsigned long mposition;
	unsigned long msize;
};

/* ----- IMC Layer2 Setting Structure ----------------- */
struct l2_param {
	unsigned long control;
	unsigned long format;
	unsigned long bufsel;
	unsigned long bytelane;
	unsigned long resize;
	unsigned long mirror;
	unsigned long offset;
	unsigned long frameadr_yp;
	unsigned long frameadr_up;
	unsigned long frameadr_vp;
	unsigned long frameadr_yq;
	unsigned long frameadr_uq;
	unsigned long frameadr_vq;
	unsigned long position;
	unsigned long size;
	unsigned long mposition;
	unsigned long msize;
};

/* ----- IMC LayerBG Setting Structure ---------------- */
struct bg_param {
	unsigned long format;
	unsigned long bufsel;
	unsigned long bytelane;
	unsigned long resize;
	unsigned long mirror;
	unsigned long offset;
	unsigned long frameadr_p;
	unsigned long frameadr_q;
	unsigned long mposition;
	unsigned long msize;
};

/* ----- IMC Reserved Register Setting Structure ---------------- */
struct rr_param {
	unsigned long reserved;
};


/********************************************************
 *  Main Structures                                     *
 *******************************************************/
/* ----- IMC Immediately-reflected Register Setting Structure ----- */
struct emxx_imc_preset {
	unsigned long          *imc_control;
	unsigned long          *imc_datareq;
	struct imc_gamma_param *gamma;
	struct imc_yuv_param   *yuv;
	struct imc_burst_param *burst;
	unsigned long          *imc_round_en;
};

/* ----- IMC V-sync Register Setting Structure -------------------- */
struct emxx_imc_update_vsync {
	unsigned long             *cpubufsel;
	struct imc_wb_param       *wb;
	unsigned long             *mirror;
	struct imc_alphasel_param *alphasel;
	unsigned long             *l0_scanmode;
	unsigned long             *l1a_scanmode;
	unsigned long             *l1b_scanmode;
	unsigned long             *l1c_scanmode;
	unsigned long             *l2a_scanmode;
	unsigned long             *l2b_scanmode;
	unsigned long             *bg_scanmode;
};

/* ----- IMC Update Target Register Setting Structure ------------- */
struct emxx_imc_update_reserve {
	struct l01_param *l0;
	struct l01_param *l1a;
	struct l01_param *l1b;
	struct l01_param *l1c;
	struct l2_param  *l2a;
	struct l2_param  *l2b;
	struct bg_param  *bg;
	struct rr_param  *rr;
};


/********************************************************
 *  Function Type Definitions                           *
 *******************************************************/
typedef void (*imc_callback_func_refresh)(void);
typedef void (*imc_callback_func_wb)(int status);


/********************************************************
 *  external function                                   *
 *******************************************************/
extern int emxx_request_imc(int channel, struct emxx_imc_info *arg);
extern int emxx_free_imc(unsigned long id);
extern int emxx_imc_set_preset(unsigned long id,
	struct emxx_imc_preset *imc_preset);
extern int emxx_imc_set_update_vsync(unsigned long id,
	struct emxx_imc_update_vsync *imc_vsync);
extern int emxx_imc_set_update_reserve(unsigned long id,
	struct emxx_imc_update_reserve *imc_reserve,
	imc_callback_func_refresh callback_refresh);
extern void emxx_imc_set_refresh(unsigned long id);
extern int emxx_imc_cancel_refresh(unsigned long id);
extern int emxx_imc_set_callback(unsigned long id,
	imc_callback_func_refresh callback_refresh,
	imc_callback_func_wb callback_wb);
extern int emxx_imc_start(unsigned long id);
extern int emxx_imc_stop(unsigned long id);

#endif /* _EMXX_FB_H_ */
