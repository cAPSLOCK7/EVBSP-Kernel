/*
 *  File Name	    : linux/drivers/usb/gadget/usb_debug_message.h
 *  Function	    : USB Function Class Driver Interface
 *  Release Version : Ver 1.01
 *  Release Date    : 2009/04/16
 *
 *  Copyright (C) 2009-2010 Renesas Electronics Corporation
 *
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
 *
 */

#ifndef __USB_DEBUG_MESSAGE_H
#define __USB_DEBUG_MESSAGE_H


/*===========================================================================*/

/* #define USE_DEBUG_MESSAGE */
/* #define USE_INFOMATION_MESSAGE */

/*===========================================================================*/
/*
 * debug message ON/OFF
 * priority is higher than "USE_DEBUG_MESSAGE, USE_INFOMATION_MESSAGE" defines.
 */
#define USE_DEBUG_MESSAGE_ALL
/* #undef  USE_DEBUG_MESSAGE_ALL */

static inline char *usb_debug_get_file_name(char *p_filename)	/* __in__ */
{
	int		pos, len, i;
	char *p;

	pos = 0;
	p = p_filename;
	len = strlen(p);

	for (i = 0; i < len; i++)
		if (p[i] == '/')
			pos = i + 1;

	p = p + pos;

	return p;
}


#ifdef USE_DEBUG_MESSAGE_ALL
	#ifdef USE_DEBUG_MESSAGE
		#define DBG_MSG(format, arg...) \
			printk(KERN_DEBUG "%s : " format "\n", \
				usb_debug_get_file_name(__FILE__), ## arg)
	#else
		#define DBG_MSG(format, arg...) do {} while (0)
	#endif

	#define DBG_MSGG(format, arg...) do {} while (0)

	#ifdef USE_INFOMATION_MESSAGE
		#define INFO_MSG(format, arg...) \
			printk(KERN_INFO "%s : " format "\n", \
				usb_debug_get_file_name(__FILE__), ## arg)
	#else
		#define INFO_MSG(format, arg...) do {} while (0)
	#endif
	#define INFO_MSGG(format, arg...) do {} while (0)

	#ifdef USE_REQUEST_INFO_MESSAGE
		#define REQUEST_INFO_MSG(format, arg...) \
			printk(KERN_DEBUG "%s : " format "\n", \
				usb_debug_get_file_name(__FILE__), ## arg)
	#else
		#define REQUEST_INFO_MSG(format, arg...) do {} while (0)
	#endif
	#define REQUEST_INFO_MSGG(format, arg...) do {} while (0)
#else	/* !USE_DEBUG_MESSAGE_ALL */

	#define DBG_MSG(format, arg...) do {} while (0)
	#define DBG_MSGG(format, arg...) do {} while (0)
	#define INFO_MSG(format, arg...) do {} while (0)
	#define INFO_MSGG(format, arg...) do {} while (0)
	#define REQUEST_INFO_MSG(format, arg...) do {} while (0)
	#define REQUEST_INFO_MSGG(format, arg...) do {} while (0)

#endif	/* USE_DEBUG_MESSAGE_ALL */

#define ERR_MSG(format, arg...) \
	printk(KERN_ERR "%s : *** Error *** " format "\n", \
		usb_debug_get_file_name(__FILE__), ## arg)
#define WARN_MSG(format, arg...) \
	printk(KERN_WARNING "%s : === Warning === " format "\n", \
		usb_debug_get_file_name(__FILE__), ## arg)


#endif	/* __USB_DEBUG_MESSAGE_H */
