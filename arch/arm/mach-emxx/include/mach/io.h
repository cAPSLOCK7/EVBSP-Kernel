/*
 *  File Name       : arch/arm/mach-emxx/include/mach/io.h
 *  Function        : io
 *  Release Version : Ver 1.01
 *  Release Date    : 2010/09/01
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

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT 0xffffffff

/*
 * We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */
#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)

/*
 * ----------------------------------------------------------------------------
 * I/O mapping
 * ----------------------------------------------------------------------------
 */
#define IO_BASE		0xf0000000
#define IO_SIZE		0x05000000

#define IO_ADDRESS(x)	(((x) & 0x0fffffff) | IO_BASE)

#define __io_address(n)	__io(IO_ADDRESS(n))

#endif	/* __ASM_ARCH_IO_H */
