/*
* arch/arm/mach-emxx/include/mach/buslim.h
* This file is bus width limit
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
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#ifndef __BUS_LIMIT_H__
#define __BUS_LIMIT_H__


/* ioctl command definition */
#define BUSLIMIT_SET_CPU0	_IOW('P', 1, unsigned long)
#define BUSLIMIT_GET_CPU0	_IOR('P', 2, unsigned long)

#define BUSLIMIT_SET_CPU1	_IOW('P', 3, unsigned long)
#define BUSLIMIT_GET_CPU1	_IOR('P', 4, unsigned long)

#define BUSLIMIT_SET_DSPD	_IOW('P', 5, unsigned long)
#define BUSLIMIT_GET_DSPD	_IOR('P', 6, unsigned long)

#define BUSLIMIT_SET_DSPI	_IOW('P', 7, unsigned long)
#define BUSLIMIT_GET_DSPI	_IOR('P', 8, unsigned long)

#define BUSLIMIT_SET_AHB	_IOW('P', 9, unsigned long)
#define BUSLIMIT_GET_AHB	_IOR('P', 10, unsigned long)

#define BUSLIMIT_SET_AVE0	_IOW('P', 11, unsigned long)
#define BUSLIMIT_GET_AVE0	_IOR('P', 12, unsigned long)

#define BUSLIMIT_SET_AVE1	_IOW('P', 13, unsigned long)
#define BUSLIMIT_GET_AVE1	_IOR('P', 14, unsigned long)

#define BUSLIMIT_SET_M2M	_IOW('P', 15, unsigned long)
#define BUSLIMIT_GET_M2M	_IOR('P', 16, unsigned long)

#define BUSLIMIT_SET_A3D	_IOW('P', 17, unsigned long)
#define BUSLIMIT_GET_A3D	_IOR('P', 18, unsigned long)

#define BUSLIMIT_SET_IMC	_IOW('P', 19, unsigned long)
#define BUSLIMIT_GET_IMC	_IOR('P', 20, unsigned long)

#define BUSLIMIT_SET_IMCW	_IOW('P', 21, unsigned long)
#define BUSLIMIT_GET_IMCW	_IOR('P', 22, unsigned long)

#define BUSLIMIT_SET_M2P	_IOW('P', 23, unsigned long)
#define BUSLIMIT_GET_M2P	_IOR('P', 24, unsigned long)

#define BUSLIMIT_SET_P2M	_IOW('P', 29, unsigned long)
#define BUSLIMIT_GET_P2M	_IOR('P', 30, unsigned long)

#define BUSLIMIT_SET_SIZ	_IOW('P', 31, unsigned long)
#define BUSLIMIT_GET_SIZ	_IOR('P', 32, unsigned long)

#define BUSLIMIT_SET_ROT	_IOW('P', 33, unsigned long)
#define BUSLIMIT_GET_ROT	_IOR('P', 34, unsigned long)

#define REFRESH_SET_CONFIGR1	_IOW('P', 25, unsigned long)
#define REFRESH_GET_CONFIGR1	_IOR('P', 26, unsigned long)

#define REFRESH_SET_CONFIGR2	_IOW('P', 27, unsigned long)
#define REFRESH_GET_CONFIGR2	_IOR('P', 28, unsigned long)

/* bus limit val shift */
#define WRITE_LIMIT_SHIFT 8
#define READ_LIMIT_SHIFT 0

/* bus limit val mask */
#define LIMIT_VAL_MASK 0x3F

#endif /* __BUS_LIMIT_H__ */




