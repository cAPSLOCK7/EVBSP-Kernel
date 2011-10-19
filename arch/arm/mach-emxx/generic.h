/*
 *  File Name       : arch/arm/mach-emxx/generic.h
 *  Function        : generic
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#ifndef __ARCH_ARM_MACH_EMXX_GENERIC_H
#define __ARCH_ARM_MACH_EMXX_GENERIC_H

extern void __init emxx_map_io(void);
extern void __init emxx_serial_init(int ports[]);

#endif	/* __ARCH_ARM_MACH_EMXX_GENERIC_H */
