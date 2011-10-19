/*
 *  drivers/usb/gadget/f_nwm.h
 *    NEC Windows Media Driver for Android
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */

#ifndef __F_NWM_MAIN_H__
#define __F_NWM_MAIN_H__

/*---------------------------------------------------------------------------*/
/* Include */
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/autoconf.h>

#include "usbclass_common.h"


/*---------------------------------------------------------------------------*/
extern  int fsg_init(void);


/*---------------------------------------------------------------------------
 * Compile Option
*/
/*------------------------------ default define */

/*------------------------------ default undef */
#undef USE_CLASS_CALLBACK

/* #define USE_USB_IAD */		/* Interface Association Descriptor */


/*---------------------------------------------------------------------------*/
enum FC_NWM_MODE {
	FC_NWM_MSC_MTP_MODE = 0,	/* MSC/MTP Mode */
	FC_NWM_MSC_ONLY_MODE,		/* MSC Only Mode */
	FC_NWM_MTP_ONLY_MODE,		/* MTP Only Mode */
	FC_NWM_MSC_ONLY_SD_MODE,	/* MSC Only Shutdown Mode */
	FC_NWM_MTP_ONLY_SD_MODE,	/* MTP Only Shutdown Mode */
	FC_NWM_MODE_END			/* End */
};

/*---------------------------------------------------------------------------*/
/* NWM STATE */
enum FC_NWM_STATE {
	FC_NWM_STATE_UNKNOWN = 0,
	FC_NWM_STATE_INIT,

	FC_NWM_STATE_WAIT_REGIST,
	FC_NWM_STATE_REGIST_END,
	FC_NWM_STATE_WAIT_CONFIG,
	FC_NWM_STATE_SET_CONFIG,
	FC_NWM_STATE_WAIT_BULK,
	FC_NWM_STATE_RECIVE_BULK,
	FC_NWM_STATE_CLASS_LOOP,

	FC_NWM_STATE_RESET_CONFIG,
	FC_NWM_STATE_RESET_INTERFACE,

	FC_NWM_STATE_RESET,
	FC_NWM_STATE_DISCONNECT,

	FC_NWM_STATE_UNREGIST,
	FC_NWM_STATE_EXIT,
};

/*---------------------------------------------------------------------------*/
/* Request */
#define FC_NWM_RSV_REQ_NONE          0
#define FC_NWM_RSV_REQ_SET_CONFIG    1
#define FC_NWM_RSV_REQ_SET_INTERFACE 2

/*---------------------------------------------------------------------------*/
/* NWM Event */
#define FC_NWM_EVENT_VBUS_OFF			(1 << 0)
#define FC_NWM_EVENT_DISCONNECT			(1 << 1)
#define FC_NWM_EVENT_RESET			(1 << 2)
#define FC_NWM_EVENT_SUSPEND			(1 << 3)
#define FC_NWM_EVENT_RESUME			(1 << 4)
#define FC_NWM_EVENT_EP0_COMPLETE		(1 << 5)
#define FC_NWM_EVENT_SET_CONFIGURATION		(1 << 6)
#define FC_NWM_EVENT_BULK_OUT_COMPLETE		(1 << 7)

#define FC_NWM_EVENT_REGIST			(1 << 16)

#define FC_NWM_DISCONNECT_EVENT		   \
		(FC_NWM_EVENT_VBUS_OFF |   \
		 FC_NWM_EVENT_DISCONNECT | \
		 FC_NWM_EVENT_RESET)

/*---------------------------------------------------------------------------*/
/* USB Descriptor */
/*------------------------------ Device Descriptor */
#define DRIVER_VENDOR_ID		0x0409	/* NEC */
#define DRIVER_PRODUCT_ID		0xFFFF	/* NEC Test Device */

#define FC_NWM_DEVICE_VERSION		0x0100

#define FC_NWM_CONF_NUM				1


/*------------------------------ Configuration Descriptor */
#define FC_NWM_IF_NUM			1
#define CONFIG_VALUE			1
#define FC_NWM_MAX_POWER		(500/2)	/* 500mA */

/*------------------------------ Interface Descriptor */
#define FC_NWM_IF_VALUE			0
#define FC_NWM_ALT_VALUE		0

/* USB protocol value = the transport method */
#define USB_PR_CBI	0x00		/* Control/Bulk/Interrupt */
#define USB_PR_CB	0x01		/* Control/Bulk w/o interrupt */
#define USB_PR_BULK	0x50		/* Bulk-only */

/* USB subclass value = the protocol encapsulation */
#define USB_SC_RBC	0x01		/* Reduced Block Commands (flash) */
#define USB_SC_8020	0x02		/* SFF-8020i, MMC-2, ATAPI (CD-ROM) */
#define USB_SC_QIC	0x03		/* QIC-157 (tape) */
#define USB_SC_UFI	0x04		/* UFI (floppy) */
#define USB_SC_8070	0x05		/* SFF-8070i (removable) */
#define USB_SC_SCSI	0x06		/* Transparent SCSI */

/*------------------------------ String Descriptor Number */
#define STRING_MANUFACTURER		1
#define STRING_PRODUCT			2
#define STRING_SERIAL			3
#define STRING_CONFIG			4
#ifdef USE_USB_IAD
	#define STRING_IAD		5
	#define STRING_INTERFACE_0	6
	#define STRING_INTERFACE_1	7
#else	/* !USE_USB_IAD */
	#define STRING_INTERFACE	5
#endif	/* USE_USB_IAD */
#define FC_NWM_STRING_OS_DESCRIPTOR	0xEE


/*------------------------------ OS Descriptor */
#define	FC_NWM_OSD_VENDOR_CODE		0x30	/* OS Descriptor */
#define	FC_NWM_MS_DESC_LENGTH		0x40

#define FC_NWM_MS_DESC_HEADER_SIZE	16	/* Header Size */
#define FC_NWM_MS_DESC_FUNCTION_SIZE	24	/* Function Size */

/*------------------------------ Endpoint */
#define FC_NWM_MSC_EP_NUM		2
#define FC_NWM_MTP_EP_NUM		3
#define FC_NWM_CONTROL_MAXPACKETSIZE	64
#define FC_NWM_INTIN_MAXPACKETSIZE	64
#define FC_NWM_INTIN_INTERVAL		10
#define FC_NWM_BULKOUT_MAXPACKETSIZE	512
#define FC_NWM_BULKIN_MAXPACKETSIZE	512

#define FC_NWM_EP0_BUF_SIZE		512
#define DELAYED_STATUS			(FC_NWM_EP0_BUF_SIZE+1)

/*------------------------------ Request Mask */
#define USB_GET_STANDARD_REQ		\
		(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE)

#define USB_GET_STANDARD_REQ_IF		\
		(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE)

#define USB_SET_STANDARD_REQ		\
		(USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE)

#define USB_SET_STANDARD_REQ_IF		\
		(USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE)

#define USB_GET_VENDOR_REQ		\
		(USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE)

#define USB_GET_VENDOR_REQ_IF		\
		(USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE)

/*------------------------------ Command Header */
#define MSC_CBW_HEADER			0x43425355
#define MTP_COMMAND_HEADER		0x00000020
#define MTP_COMMAND_ID			0x0001


/*---------------------------------------------------------------------------*/
/* Structure */
typedef struct _T_NWM_OS_DESC {
	__u32	length;
	__u16	bcdVersion;
	__u16	wIndex;
	__u8	bCount;
	__u8	reserved[7];
} T_NWM_OS_DESC, *PT_NWM_OS_DESC;

typedef struct _T_NWM_WORK {
	__u8	config_num;			/* Configuration Number */
	__u8	interface_num;			/* Interface Number */

	__u32 	act_gadget;			/* MSC/MTP Select Flag */
	__u32	nwm_state;			/* NWM State */
	__u32	nwm_event;			/* NWM Event */

	struct completion thread_comp;		/* Task Init/Exit */
	struct completion thread_wait_comp;	/* Task Wait */
	struct semaphore  work_sem;		/* for Work Area */
	BOOL	bRegist_flag;			/* Class Driver registration */
	BOOL	bRunning_flag;			/* Run Flag */

	int	thread_pid;			/* Thread ID */

	__u32	rsv_req;			/* Reservation request */
	__u16	rsv_req_val;			/* Reservation request value */
	PT_USB_CLASS_DRIVER	pMSC_Context;	/* MSC Context */
	PT_USB_CLASS_DRIVER	pMTP_Context;	/* MTP Context */

	spinlock_t	lock;			/* Lock */

	struct usb_gadget	*gadget;	/* Gadget */

	struct usb_ep		*ep0;		/* Endpoint 0 */
	struct usb_request	*ep0req;	/* Endpoint 0 Request */
	unsigned int		ep0_req_tag;	/* Endpoint 0 Request Tag */
	const char		*ep0req_name;	/* Endpoint 0 Request Name */
	BOOL	bStatusStageFlag;		/* Status Stage Flag */

	/* Endpoint Structure */
	struct usb_ep		*bulk_in;	/* Bulk IN */
	struct usb_ep		*bulk_out;	/* Bulk OUT */
	struct usb_ep		*intr_in;	/* Interrupt IN */

	/* Endpoint Request */
	struct usb_request	*bulk_in_req;	/* Bulk IN */
	struct usb_request	*bulk_out_req;	/* Bulk OUT */
	struct usb_request	*intr_in_req;	/* Interrupt IN */

	/* Endpoint Enable Flag */
	BOOL	bBulk_in_enable;		/* Bulk IN */
	BOOL	bBulk_out_enable;		/* Bulk OUT */
	BOOL	bIntr_in_enable;		/* Interrupt IN */

	__u8	*p_bulk_buf;			/* Data Buffer Pointer */

	BOOL	bShutdownFlag;			/* Shutdown Flag */
	BOOL	bVBUS_OFF_Flag;			/* VBUS OFF Flag */

	struct usb_function function;		/* Android */
	struct usb_configuration *c;		/* */
	int		nluns;			/* LUN (MSC) */

} T_NWM_WORK, *PT_NWM_WORK;


int nwm_function_add(struct usb_configuration *c, int nluns);
void nwm_function_del(struct usb_configuration *c);


#endif /* __F_NWM_MAIN_H__ */

