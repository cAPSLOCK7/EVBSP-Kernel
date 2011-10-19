/*
 *  File Name	    : include/sound/emxx_pcm_ioctl.h
 *  Function	    : audio interface for the EMMA Mobile
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/04/01
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

#ifndef __SOUND_EMXX_PCM_IOCTL_H
#define __SOUND_EMXX_PCM_IOCTL_H

#include <linux/ioctl.h>

/* mode_sel : Operation mode */
#define PCM_MODE_0              0       /* mode 0 */
#define PCM_MODE_1              1       /* mode 1 */
#define PCM_MODE_2              2       /* mode 2 (I2S Format) */
#define PCM_MODE_3              3       /* mode 3 (MSB First)  */
#define PCM_MODE_4              4       /* mode 4 (LSB First)  */
#define PCM_MODE_5              5       /* mode 5 (Multi Channel Mode) */
#define PCM_MODE_6              6       /* mode 6 (Multi Channel Mode) */
#define PCM_MODE_7              7       /* mode 7 (VCSI Mode) */

/* m_s : Master/Slave */
#define PCM_MASTER_MODE         0       /* Master */
#define PCM_SLAVE_MODE          1       /* Slave  */

/* tx_tim : Transmission timing */
#define PCM_TX_30_WORD          0       /* 30word */
#define PCM_TX_24_WORD          1       /* 24word */
#define PCM_TX_20_WORD          2       /* 20word */
#define PCM_TX_16_WORD          3       /* 16word */
#define PCM_TX_12_WORD          4       /* 12word */
#define PCM_TX_08_WORD          5       /*  8word */
#define PCM_TX_04_WORD          6       /*  4word */
#define PCM_TX_01_WORD          7       /*  1word */

/* rx_pd : Data padding (RX) */
/* tx_pd : Data padding (TX) */
#define PCM_PADDING_OFF         0       /* Data padding non */
#define PCM_PADDING_ON          1       /* Data padding */


struct pcm_ctrl {
	struct func_sel {
		unsigned char mode_sel; /* Operation mode */
		unsigned char m_s;      /* Master/Slave */
		unsigned char tx_tim;   /* Transmission timing */
	} func;
	struct cycle {
		unsigned char cyc_val;  /* Frame length */
		unsigned char sib;      /* Data bit length (PMx_SI) */
		unsigned char rx_pd;    /* Data padding (RX) */
		unsigned char sob;      /* Data bit length (PMx_SO) */
		unsigned char tx_pd;    /* Data padding (TX) */
	} cyc;
	struct cycle2 {
		unsigned char cyc_val2; /* Frame length */
		unsigned char sib2;     /* Data bit length (PMx_SI) */
		unsigned char sob2;     /* Data bit length (PMx_SO) */
	} cyc2;
};
#define pcm_ctrl_t	struct pcm_ctrl

#define SNDRV_EMXX_IOCTL_PCM_SET_CTRL  _IOW('H', 0x11, pcm_ctrl_t)
#define SNDRV_EMXX_IOCTL_PCM_GET_CTRL  _IOR('H', 0x12, pcm_ctrl_t)

#endif  /* __SOUND_EMXX_PCM_IOCTL_H */

