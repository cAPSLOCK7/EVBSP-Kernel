/*
 *  File Name	    : linux/drivers/usb/gadget/usbclass_common.h
 *  Function	    : USB Function Class Driver Interface
 *  Release Version : Ver 1.00
 *  Release Date    : 2007/01/04
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
 *
 */

#ifndef _USBCLASS_COMMON_H_
#define _USBCLASS_COMMON_H_

/*===========================================================================*/
/* include */

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#define BOOL long

#ifndef NULL
 #define NULL 0
#endif

#ifndef TRUE
 #define TRUE 1
#endif
#ifndef FALSE
 #define FALSE 0
#endif


/*===========================================================================*/
/* Class type */
/*   NWM driver registration specify */
enum USB_CLASS_TYPE {
	USB_START_TYPE = 0,
	USB_MSC_TYPE,
	USB_MTP_TYPE,
	USB_END_TYPE
};


/*===========================================================================*/
/* Structure */
typedef struct usb_class_driver {

	struct usb_gadget_driver *pDriver;   /* gadget driver pointer */
	struct usb_gadget	*gadget;     /* gadget pointer */
	struct usb_gadget	*org_gadget; /* original gadget pointer */
	__u32	ClassType;                   /* USB_CLASS_TYPE */
	__u32	running;

	struct task_struct	*thread_task;

	/* main loop function pointer */
	void	(*pMainLoop)(struct usb_class_driver *);

	/* main loop  initialize function pointer */
	void	(*pMainLoopInit)(struct usb_class_driver *);

	/* gadget context */
	void	*pContext;

	struct usb_ep		*ep0;		/* Handy copy of gadget->ep0 */
	struct usb_request	*ep0req;	/* For control responses */
	void	(*ep0_complete)(struct usb_ep *, struct usb_request *);
	void	(*ep0_send_status_stage)(void);

	struct usb_ep		*bulk_in;
	struct usb_ep		*bulk_out;
	struct usb_ep		*intr_in;

	struct usb_request	*bulk_in_req;
	struct usb_request	*bulk_out_req;
	struct usb_request	*intr_in_req;

	struct usb_endpoint_descriptor *bulk_in_desc;
	struct usb_endpoint_descriptor *bulk_out_desc;
	struct usb_endpoint_descriptor *intr_in_desc;

} T_USB_CLASS_DRIVER, *PT_USB_CLASS_DRIVER;


extern int fc_nwm_register_driver(struct usb_class_driver *driver);
extern int fc_nwm_unregister_driver(struct usb_class_driver *driver);


#endif /* _USBCLASS_COMMON_H_ */

