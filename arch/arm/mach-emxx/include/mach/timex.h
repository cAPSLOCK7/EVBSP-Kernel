/*
 *  File Name       : arch/arm/mach-emxx/include/mach/timex.h
 *  Function        : timex
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

#ifndef __ASM_ARCH_TIMEX_H
#define __ASM_ARCH_TIMEX_H

#define CLOCK_TICK_RATE		32000

#ifdef CONFIG_EMXX_PLL3_238MHZ
#define CLOCK_TICK_RATE_PLL3    5939200
#else
#define CLOCK_TICK_RATE_PLL3    5734400
#endif

#define CLOCK_TICK_RATE_32K     32000


#endif	/* __ASM_ARCH_TIMEX_H */
