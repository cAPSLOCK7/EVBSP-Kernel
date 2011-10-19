/*
 *  File Name	    : arch/arm/mach-emxx/include/mach/extio.h
 *  Function        : External GPIO
 *  Release Version : Ver 1.00
 *  Release Date    : 2010/02/05
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

#ifndef _ASM_ARCH_EXTIO_H_
#define _ASM_ARCH_EXTIO_H_

/* EXTIO address */
#define EXTIO_INPUT0	0x00
#define EXTIO_INPUT1	0x01
#define EXTIO_OUTPUT0	0x02
#define EXTIO_OUTPUT1	0x03
#define EXTIO_POLRITY0	0x04
#define EXTIO_POLRITY1	0x05
#define EXTIO_CONF0	0x06
#define EXTIO_CONF1	0x07
#define EXTIO_INPUT2	0x10
#define EXTIO_INPUT3	0x11
#define EXTIO_OUTPUT2	0x12
#define EXTIO_OUTPUT3	0x13
#define EXTIO_POLRITY2	0x14
#define EXTIO_POLRITY3	0x15
#define EXTIO_CONF2	0x16
#define EXTIO_CONF3	0x17

extern int emxx_extio_initialized;

extern int extio_reg_read(int addr, unsigned char *data);
extern int extio_reg_write(int addr, unsigned char data);
extern int extio_read(int addr, unsigned char *data);
extern int extio_write(int addr, unsigned char data, unsigned char mask);
extern int extio_set_direction(unsigned gpio, int is_input);
extern int extio_get_value(unsigned int gpio);
extern void extio_set_value(unsigned int gpio, int value);

#endif /* _ASM_ARCH_EXTIO_H_ */
