/*
 *  File Name       : arch/arm/mach-emxx/include/mach/charger.h
 *  Function        : charger
 *  Release Version : Ver 1.10
 *  Release Date    : 2010/09/10
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

#ifndef __ASM_ARCH_CHARGER_H
#define __ASM_ARCH_CHARGER_H

#define EMXX_USB_OFF		0
#define EMXX_USB_DEVICE		1
#define EMXX_USB_CHARGER	2
#define EMXX_USB_DEVICE_100	3

extern int emxx_battery_usb_state(int state);

#endif	/* __ASM_ARCH_CHARGER_H */
