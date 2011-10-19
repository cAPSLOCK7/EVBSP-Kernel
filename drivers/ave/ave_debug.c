/*
 *  File Name       : ave_debug.c
 *  Function        : AVE DEBUG
 *  Release Version : Ver 1.00
 *  Release Date    : 2009/12/04
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


#include <linux/kernel.h> /* printk */
#include <stdarg.h>

/* debug functions --------------------------------------------------------- */

int debug_level;
static const char debug_prefix[] = "";
#define debug_printf printk
/* #define debug_printf __debug_printf */

static int debug_nowstring(char *buf, int len)
{

	buf[0] = '\0';
	return 0;
}


void __debug_np(int level, const char *format, ...)
{
	char sbuf[128];
	va_list args;
	if (debug_level < level)
		return;

	va_start(args, format);
	vsnprintf(sbuf, sizeof(sbuf), format, args);
	va_end(args);

	debug_printf("%s%s\n", debug_prefix, sbuf);
}

void __debug(int level, const char *function, const char *format, ...)
{
	char sbuf[128];
	char times[20];
	va_list args;
	if (debug_level < level)
		return;

	va_start(args, format);
	vsnprintf(sbuf, sizeof(sbuf), format, args);
	va_end(args);

	debug_nowstring(times, sizeof(times));
	debug_printf("%s%s%-16s: %s\n", debug_prefix, times, function, sbuf);
}

void __dump(int level, const char *function, void *ptr, int len)
{
	unsigned char *p = (unsigned char *)ptr;
	char sbuf[128];
	int i;
	if (debug_level < level)
		return;

	__debug(level, function, "length=%d", len);
	if (len > 32)
		len = 32;

	for (i = 0; i < len;) {
		int nl = 16;
		char *sbufp;

		sbufp = sbuf + sprintf(sbuf, " %04X: ", (unsigned int)(i));
		for (; i < len && nl > 0; i++, nl--)
			sbufp += sprintf(sbufp, "%02X ", p[i]);
		debug_printf("%s\n", sbuf);
	}
}
