/*
 *  File Name       : arch/arm/mach-emxx/include/mach/uncompress.h
 *  Function	    : Serial port stubs for kernel decompress status messages
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

#include <linux/types.h>
#include <linux/serial_reg.h>
#include <mach/hardware.h>

static void putc(int c)
{
	volatile u8 *uart = (volatile u8 *)(EMXX_UART0_BASE);

	while (!(uart[UART_LSR] & UART_LSR_THRE))
		barrier();
	uart[UART_TX] = c;
}

static inline void flush(void)
{
	volatile u8 *uart = (volatile u8 *)(EMXX_UART0_BASE);

	while (!(uart[UART_LSR] & UART_LSR_THRE))
		barrier();
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
