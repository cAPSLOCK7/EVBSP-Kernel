/*
 *  drivers/usb/gadget/nwm_mtp.c
 *    USB MTP Transport Driver
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


#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pagemap.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/utsname.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "usbclass_common.h"

/*---------------------------------------------------------------------------*/
/* Debug Message */
/*#define USE_DEBUG_MESSAGE*/
/*#define USE_INFOMATION_MESSAGE*/
#include "usb_debug_message.h"


/*---------------------------------------------------------------------------*/
/* Macro */
#define	NWM_DEVICE_TYPE_MTP		1
#define USB_MTP_ERROR			-1
#define USB_DEVICE_NOT_CONNECTED	-2
#define USB_HOST_CANCELING		-3

#define	READ_BUFFER_SIZE		(1024*16)
#define	WRITE_BUFFER_SIZE		(1024*16)
#define	EVENT_BUFFER_SIZE		(1024*16)

#define USB_HOST_CANCEL_TIME		(200)		/* 200ms */
#define USB_HOST_CANCEL_COUNT		(1000/USB_HOST_CANCEL_TIME)
#define USB_HOST_CANCEL_TIMEOUT		(HZ/USB_HOST_CANCEL_COUNT)

/*---------------------------------------------------------------------------*/
#define USB_MTP_MODULE_NAME	"g_mtp_transport"

#define DRIVER_DESC		"Media Transport Protcol Transport Gadget"
#define DRIVER_NAME		"g_mtp_transport"
#define DRIVER_VERSION		"0.1"

/*---------------------------------------------------------------------------*/
/* I/O Control */
#define MTP_IOCTL_BASE 'M'

#define MTP_IOCTL_INITIALIZE		_IOR(MTP_IOCTL_BASE, 0, int)
#define MTP_IOCTL_RESET			_IOR(MTP_IOCTL_BASE, 1, int)
/*#define MTP_IOCTL_SENDDATA		_IOR(MTP_IOCTL_BASE, 2, int)*/
/*#define MTP_IOCTL_RECEIVEDATA		_IOR(MTP_IOCTL_BASE, 3, int)*/
#define MTP_IOCTL_STALL			_IOR(MTP_IOCTL_BASE, 4, int)
#define MTP_IOCTL_UNSTALL		_IOR(MTP_IOCTL_BASE, 5, int)
#define MTP_IOCTL_DISCONNECT		_IOR(MTP_IOCTL_BASE, 6, int)
#define MTP_IOCTL_CONNECT		_IOR(MTP_IOCTL_BASE, 7, int)
#define MTP_IOCTL_IS_CONNECTED		_IOR(MTP_IOCTL_BASE, 8, int)
#define MTP_IOCTL_INITWAIT		_IOR(MTP_IOCTL_BASE, 9, int)
#define MTP_IOCTL_SENDEVENT		_IOR(MTP_IOCTL_BASE, 10, int)
#define MTP_IOCTL_SET_TX_FLAG		_IOR(MTP_IOCTL_BASE, 11, int)
#define MTP_IOCTL_XCHG_STATUS		_IOR(MTP_IOCTL_BASE, 12, int)
#define MTP_IOCTL_GET_VBUS_LEVEL	_IOR(MTP_IOCTL_BASE, 13, int)
#define MTP_IOCTL_ABORT_TRANSFER	_IOR(MTP_IOCTL_BASE, 14, int)

#define MTP_BULK_OUT_STALL		(0)
#define MTP_BULK_IN_STALL		(1)

/* Class Request */
#define USB_PTPREQUEST_TYPE_OUT		0x21	/* Host to Device */
#define USB_PTPREQUEST_TYPE_IN		0xA1	/* Device to Host */
#define USB_PTPREQUEST_CANCELIO		0x64	/* Cancel request */
#define USB_PTPREQUEST_GETEVENT		0x65	/* Get extened event data */
#define USB_PTPREQUEST_RESET		0x66	/* Reset Device */
#define USB_PTPREQUEST_GETSTATUS	0x67	/* Get Device Status */

#define USB_PTP_CANCELL_CODE		0x4001	/* PTP Cancellation Code */


/*---------------------------------------------------------------------------*/
/* for Event */
typedef struct mtp_event_type {
	char		*pBuffer;
	unsigned int	size;
	int		count;
} T_MTP_EVENT;

/*---------------------------------------------------------------------------*/
/* for Cancel (USB_PTPREQUEST_CANCELIO) */
typedef struct tagCancelReqData {
	__u16	wCancelIOCode;
	__u32	TransactionId;
} __attribute__ ((packed)) USB_PTP_CANCELREQ_DATA, *PUSB_PTP_CANCELREQ_DATA;

/*---------------------------------------------------------------------------*/
/* for Status Data (USB_PTPREQUEST_GETSTATUS) */
#define	STATUS_DEVICE_OK			0x2001
#define	STATUS_DEVICE_BUSY			0x2019
#define	STATUS_TRANSACTION_CANCELED		0x201F

#define GETSTATUS_HEADER_SIZE			(4)
#define DELAYED_STATUS	(512 + 999)	/* An impossibly large value */

typedef struct tagStatusReqData {
	__u16	wLength;
	__u16	Code;
	__u32	Params[4];
} __attribute__	((packed)) USB_PTP_STATUSREQ_DATA, *PUSB_PTP_STATUSREQ_DATA;

/*---------------------------------------------------------------------------*/
/* MTP Status (for Disconnect) */
typedef enum {
	MTP_STATUS_OK = 0,
	MTP_STATUS_BULK_IN_WAIT,
	MTP_STATUS_BULK_OUT_WAIT,
	MTP_STATUS_INTR_WAIT
} eMTP_STATUS;

/*---------------------------------------------------------------------------*/
/**
 * \enum eDevicePhase from MTP App.
 *
 * Defines which phase the device is currently in.
 */
typedef enum DevicePhase {
	DEVICE_PHASE_NOTREADY = 0,	/**< Device is in a busy state. */
	DEVICE_PHASE_IDLE = 1,		/**< Device is in an idle state. */
	DEVICE_PHASE_COMMAND = 2,	/**< Device is in the command phase. */
	DEVICE_PHASE_DATAIN = 3,	/**< Device is in the data in phase. */
	DEVICE_PHASE_DATAOUT = 4,	/**< Device is in the data out phase. */
	DEVICE_PHASE_RESP0NSE = 5	/**< Device is in the response phase. */
} eDevicePhase;

/**
 * \enum eDeviceStatus from MTP App.
 *
 * Defines the status a PTP camera could be in.
 */
typedef enum DeviceStatus {
	DEVICE_STATUSOK = 0,	  /**< Device OK. */
	DEVICE_LOWBATTERY = 1,	  /**< Fatal error, can't continue operation. */
	DEVICE_OUTOFMEMORY = 2,   /**< Out of memory, Can't continue. */
	DEVICE_STOREREMOVED = 3,  /**< Fatal error, especially if during a data transfer. */
	DEVICE_DEVICEERROR = 4	  /**< Unidentified fatal device error, can't continue. */
} eDeviceStatus;

typedef	struct _T_USB_MTP_XCHG_STATUS {
	/* Application Side */
	__u32			TransactionID;
	eDevicePhase	Phase;
	__u8			CancelClear;
	__u8			SuspendClear;
	__u8			ResetClear;
	eDeviceStatus	Status;

	/* Device Side */
	BOOL			b_Cancelling;
	BOOL			bBulkInCanceled;
	BOOL			bBulkOutCanceled;
	BOOL			bSuspend;
	BOOL			bReset;
	BOOL			bBusy;
} USB_MTP_XCHG_STATUS, *PUSB_MTP_XCHG_STATUS;


/*---------------------------------------------------------------------------*/
/* MTP Device Context */
struct mtp_device {
	/* list of usb_mtptrans devices */
	dev_t	devcode;
	struct cdev cdev;

	struct class *usb_mtp_class;

	spinlock_t	lock;
	int		open_count;
	struct usb_gadget	*gadget;	/* for Class */
	struct usb_gadget	*org_gadget;	/* Original */

	struct completion	mtpthread_comp;	/* Disconnect or Stop */

	struct semaphore	interrupt_comp;	/* Interrupt */
	struct semaphore	bulk_comp;	/* Bulk OUT */
	struct semaphore	mtpinit_comp;	/* MTP Driver Start */
	struct semaphore	drv_reg_comp;	/* MTP Gadget Regist */

	BOOL		b_Vbus;			/* VBUS Level */
	__u8		config;			/* Config Status */
	BOOL		b_BulkInSetStall;	/* Bulk IN Stall */
	BOOL		b_BulkOutSetStall;	/* Bulk OUT Stall */
	BOOL		b_InitSuccess;		/* Driver Initialization */

	USB_MTP_XCHG_STATUS	devStatus;	/* Status Change */

	BOOL		b_Firstflg;		/* Data first time reception */
	BOOL		b_DataZeroRequired;	/* Send 0 Length */
	BOOL		b_BulkInWorking;	/* Using Bulk IN */
	BOOL		b_BulkOutWorking;	/* Using Bulk OUT */
	BOOL		b_LastTransferIsIn;	/* Last Transfer Type */

	struct usb_ep		*ep0;		/* Endpoint 0 */
	struct usb_request	*ep0req;	/* Endpoint 0 Request */

	/* Endpoint Structure */
	struct usb_ep		*bulk_in;	/* Bulk IN */
	struct usb_ep		*bulk_out;	/* Bulk OUT */
	struct usb_ep		*intr_in;	/* Interrupt IN */

	/* Endpoint Request */
	struct usb_request	*inreq;		/* Bulk IN */
	struct usb_request	*outreq;	/* Bulk OUT */
	struct usb_request	*intreq;	/* Interrupt IN */

	/* Endpoint Descriptor */
	struct usb_endpoint_descriptor *bulk_in_desc;	/* Bulk IN */
	struct usb_endpoint_descriptor *bulk_out_desc;	/* Bulk OUT */
	struct usb_endpoint_descriptor *intr_in_desc;	/* Interrupt IN */

	/* Endpoint Semaphore */
	struct semaphore	bulk_in_use_sem;	/* Bulk IN */
	struct semaphore	bulk_out_use_sem;	/* Bulk OUT */
	struct semaphore	intr_in_use_sem;	/* Interrupt IN */

	eMTP_STATUS		tx_status;		/* Transfer Status */

	__u8		*p_send_buf;		/* Send Buffer */
	__u8		*p_recv_buf;		/* Receive Buffer */
	__u8		*p_event_buf;		/* Event Buffer */

	BOOL		b_RegistFlag;		/* Regist Flag */
	__u32		class_req_type;		/* Class Type */

	/* Cancel */
	struct timer_list	cancel_timer;	/* Cancel Timer */
	__u32		host_cancel_count;	/* Cancel Count */
	BOOL		b_timer_flag;		/* Timer Flag */

	__u32		status_count;		/* Status Count */
};

T_USB_CLASS_DRIVER g_mtp_regist_driver;




static const char longname[] = DRIVER_DESC;
static const char shortname[] = DRIVER_NAME;

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("Dual BSD/GPL");

/*******************************************************************************
*  Switch for Debug
*******************************************************************************/


/*******************************************************************************
*  Function
*******************************************************************************/

struct	mtp_device the_mtp;
struct	mtp_device *g_mtp = &the_mtp;

/*---------------------------------------------------------------------------*/
/* Prototype */
static int _mtp_ioctl_waitinit(void);
static int _mtp_ioctl_initialize(void);
static int _mtp_ioctl_reset(void);
static int _mtp_ioctl_sendevent(char *pBuffer, unsigned int size, int *count);
static int _mtp_ioctl_disconnect(void);
static int _mtp_ioctl_connect(void);
static int _mtp_ioctl_isconnected(void);
static int _mtp_ioctl_stall(long arg);
static int _mtp_ioctl_unstall(long arg);
static int _mtp_ioctl_abort_transfer(void);

static int _mtp_ioctl_get_vbus_level(void);
static int _mtp_ioctl_xchg_status(PUSB_MTP_XCHG_STATUS devStatus);

static int usb_mtptrans_start(void);
static void usb_mtptrans_stop(void);

static void ep0_complete(struct usb_ep *ep, struct usb_request *req);
static void bulk_in_complete(struct usb_ep *ep, struct usb_request *req);
static void bulk_out_complete(struct usb_ep *ep, struct usb_request *req);
static void intr_in_complete(struct usb_ep *ep, struct usb_request *req);

static int alloc_endpoint(void);
static int free_endpoint(void);

static void mtp_main_thread(struct usb_class_driver *p_class);
static	int do_set_config(u8 new_config);
static	int do_set_interface(int altsetting);

static	int mtp_bind(struct usb_gadget *gadget);
static void mtp_unbind(struct usb_gadget *gadget);
static void mtp_disconnect(struct usb_gadget *gadget);
static	int mtp_setup(struct usb_gadget *, const struct usb_ctrlrequest *);
static void mtp_suspend(struct usb_gadget *gadget);
static void mtp_resume(struct usb_gadget *gadget);

static void mtp_free(void);

static void exe_cancel_io_req(struct usb_request *req);
static void exe_reset_req(struct usb_request *req);

static	int start_transfer(struct usb_ep *ep, struct usb_request *req);
static	int class_setup_req(const struct usb_ctrlrequest *ctrl);
static	int standard_setup_req(const struct usb_ctrlrequest *ctrl);

static void _nwm_mtp_cancel_timer_start(void);
static void _nwm_mtp_cancel_timer_restart(void);
static void _nwm_mtp_cancel_timer_stop(void);
static void _nwm_mtp_cancel_timer_fn(unsigned long arg);

static int _usb_mtp_read(char *buffer, size_t count);
static int _usb_mtp_write(const char *buffer, size_t count);


static struct usb_gadget_driver mtp_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed		= USB_SPEED_HIGH,
#else
	.speed		= USB_SPEED_FULL,
#endif
	.function	= (char *) longname,
	.bind		= mtp_bind,
	.unbind		= mtp_unbind,
	.disconnect	= mtp_disconnect,
	.setup		= mtp_setup,
	.suspend	= mtp_suspend,
	.resume		= mtp_resume,

	.driver		= {
		.name		= (char *) shortname,
		/* .release = ... */
		/* .suspend = ... */
		/* .resume = ... */
	},
};

/*---------------------------------------------------------------------------*/

/**
 * IO Control
 *   Wait Initialization
 */
static int _mtp_ioctl_waitinit(void)
{
	int retcode;

	DBG_MSG("----- %s wait", __func__);

	/* MTP Driver initialization completion waiting */
	retcode = down_interruptible(&g_mtp->mtpinit_comp);
	if (retcode != 0) {
		DBG_MSG("=== WAIT INIT Failed(INTERRUPT SIGNAL)");
		return -EINTR;
	}

	DBG_MSG("----- %s wakeup", __func__);

	return 0;
}

/**
 * IO Control
 *   Initialization
 */
static int _mtp_ioctl_initialize(void)
{
	DBG_MSG("%s() Call", __func__);

	return 0;
}

/**
 * IO Control
 *   Reset
 */
static int _mtp_ioctl_reset(void)
{
	int return_value;

	DBG_MSG("%s() Call", __func__);

	return_value = _mtp_ioctl_disconnect();
	if (return_value != 0)
		return 1;

	return_value = _mtp_ioctl_initialize();
	if (return_value != 0)
		return 2;
	else
		return 0;
}

/**
 * IO Control
 *   Report a MTP event block through MTP Interrupt Endpoint
 */
static int _mtp_ioctl_sendevent(char *pBuffer, unsigned int size, int *count)
{
	int retcode;

	DBG_MSG("%s() Call", __func__);

	/* Do sanity checks */
	if (count)
		*count = 0;

	if (!pBuffer)
		return 1;

	down(&g_mtp->intr_in_use_sem);

	if (!g_mtp->config || g_mtp->devStatus.bReset
		 || g_mtp->devStatus.bSuspend) {
		DBG_MSG("*** MTP Device Disconnected ***");
		up(&g_mtp->intr_in_use_sem);
		return USB_DEVICE_NOT_CONNECTED;
	}

	if (size > EVENT_BUFFER_SIZE) {
		DBG_MSG("%s too large write size", __func__);
		up(&g_mtp->intr_in_use_sem);
		return USB_MTP_ERROR;
	}

	/* User Memory -> DMA Buffer */
	retcode = copy_from_user(g_mtp->intreq->buf, pBuffer, size);
	if (retcode != 0) {
		DBG_MSG("=== %s copy_from_user() Failed", __func__);
		up(&g_mtp->intr_in_use_sem);
		return USB_MTP_ERROR;
	}
	g_mtp->intreq->length = size;
	g_mtp->intreq->zero = 1;

	g_mtp->tx_status = MTP_STATUS_INTR_WAIT;
	retcode = start_transfer(g_mtp->intr_in, g_mtp->intreq);
	if (retcode != 0) {
		ERR_MSG("_mtp_ioctl_sendevent()");
		ERR_MSG("  start_transfer retcode = %d", (int)retcode);
		g_mtp->tx_status = MTP_STATUS_OK;
		up(&g_mtp->intr_in_use_sem);
		return USB_MTP_ERROR;
	}

	retcode = down_interruptible(&g_mtp->interrupt_comp);
	if (retcode != 0) {
		DBG_MSG("=== IOCTL Failed(INTERRUPT SIGNAL)");
		_mtp_ioctl_abort_transfer();
		retcode = -EINTR;
	} else {
		if (g_mtp->intreq->status != 0) {
			INFO_MSGG("---- Interuupt IN status = %d",
				 (int)g_mtp->intreq->status);
			retcode = USB_MTP_ERROR;
		} else {
			if (g_mtp->config && !g_mtp->devStatus.bReset)
				*count = g_mtp->intreq->actual;
			else
				retcode = USB_DEVICE_NOT_CONNECTED;
		}
	}
	up(&g_mtp->intr_in_use_sem);

	return retcode;
}

/**
 * IO Control
 *   Request of Disconnect
 */
static int _mtp_ioctl_disconnect(void)
{
	int		nret = 0;

	DBG_MSG("%s() Enter", __func__);

	if (g_mtp->org_gadget != NULL)
		nret = usb_gadget_disconnect(g_mtp->org_gadget);

	return nret;
}

/**
 * IO Control
 *   Request of Connect
 */
static int _mtp_ioctl_connect(void)
{
	int		nret = 0;

	DBG_MSG("%s() Enter", __func__);

	if (g_mtp->org_gadget != NULL)
		nret = usb_gadget_connect(g_mtp->org_gadget);

	return nret;
}

/**
 * IO Control
 *   Connected check
 */
static int _mtp_ioctl_isconnected(void)
{
	DBG_MSGG("%s() Enter", __func__);
	if (g_mtp->config != 0)
		return 0;	/* Connect */
	else
		return 1;	/* Disconnect */
}

/**
 * IO Control
 *   VBUS state check
 */
static int _mtp_ioctl_get_vbus_level(void)
{
	if (g_mtp->b_Vbus == FALSE)
		return 1;	/* VBUS OFF */
	else
		return 0;	/* VBUS ON */
}

/**
 * IO Control
 *   Status exchange
 */
static int _mtp_ioctl_xchg_status(PUSB_MTP_XCHG_STATUS devStatus)
{
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	DBG_MSGG("---  _mtp_ioctl_xchg_status");

	/* Status from application */
	ds->Phase = devStatus->Phase;

	if (ds->bReset && devStatus->ResetClear) {
		DBG_MSGG("!!! Reset Clear !!!");
		ds->bReset = FALSE;
	}

	if (devStatus->CancelClear != FALSE) {
		if ((ds->bBulkInCanceled == FALSE)
			 && (ds->bBulkOutCanceled == FALSE))
			ds->b_Cancelling = FALSE;
	}

	ds->Status = devStatus->Status;
	ds->bBusy  = devStatus->bBusy;

	if (ds->bBusy != FALSE)
		ds->TransactionID = devStatus->TransactionID;

	/* Return to application */
	devStatus->b_Cancelling		= ds->b_Cancelling;
	devStatus->bBulkInCanceled	= ds->bBulkInCanceled;
	devStatus->bBulkOutCanceled	= ds->bBulkOutCanceled;
	devStatus->bSuspend		= ds->bSuspend;
	devStatus->bReset		= ds->bReset;

	return	0;
}

/**
 * IO Control
 *   Stall the BULKIN/OUT Endpoints
 */
static int _mtp_ioctl_stall(long arg)
{
	int		nret = 0;

	if (g_mtp->b_Vbus == FALSE)
		return nret;

	if (arg) {
		DBG_MSG("--- Bulk IN Set Halt");
		g_mtp->b_BulkInSetStall = TRUE;
		nret = usb_ep_set_halt(g_mtp->bulk_in);
	} else {
		DBG_MSG("--- Bulk OUT Set Halt");
		g_mtp->b_BulkOutSetStall = TRUE;
		nret = usb_ep_set_halt(g_mtp->bulk_out);
	}

	return nret;
}

/**
 * IO Control
 *   Unstall the BULKIN/OUT Endpoints
 */
static int _mtp_ioctl_unstall(long arg)
{
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;
	int		nret = 0;
	struct usb_ep *ep;

	if (g_mtp->b_Vbus == FALSE)
		return nret;

	if (arg) {
		if (g_mtp->b_BulkInSetStall) {
			ep = g_mtp->bulk_in;
			usb_ep_fifo_flush(ep);
			DBG_MSG("--- Bulk IN Clear Stall");
			nret = usb_ep_clear_halt(ep);
			g_mtp->b_BulkInSetStall = FALSE;
		}
		ds->bBulkInCanceled = FALSE;
	} else {
		if (g_mtp->b_BulkOutSetStall) {
			ep = g_mtp->bulk_out;
			usb_ep_fifo_flush(ep);
			DBG_MSG("--- Bulk OUT Clear Stall");
			nret = usb_ep_clear_halt(ep);
			g_mtp->b_BulkOutSetStall = FALSE;
		}
		ds->bBulkOutCanceled = FALSE;
	}

	return nret;
}

/**
 * IO Control
 *   Abort Transfer
 */
static int _mtp_ioctl_abort_transfer()
{
	int		nret = 0;

	DBG_MSGG("%s() Enter", __func__);

	if (g_mtp->b_Vbus == FALSE)
		return nret;

	_mtp_ioctl_stall(MTP_BULK_IN_STALL);
	if (g_mtp->b_BulkInWorking)
		nret = usb_ep_dequeue(g_mtp->bulk_in, g_mtp->inreq);

	_mtp_ioctl_stall(MTP_BULK_OUT_STALL);
	if (g_mtp->b_BulkOutWorking)
		nret = usb_ep_dequeue(g_mtp->bulk_out, g_mtp->outreq);

	return 0;
}

/**
 * Transferring completion callback of Endpoint 0
 */
static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	switch (g_mtp->class_req_type) {
	case USB_PTPREQUEST_CANCELIO:
		exe_cancel_io_req(req);
		break;

	case USB_PTPREQUEST_RESET:
		exe_reset_req(req);
		break;

	default:
		break;
	}

	DBG_MSGG("----- %s()", __func__);

	g_mtp->class_req_type = 0;
}

/**
 * Transfer Start
 */
static int start_transfer(struct usb_ep *ep, struct usb_request *req)
{
	int		rc;

	rc = usb_ep_queue(ep, req, GFP_KERNEL);

	return rc;
}

/**
 * Transferring completion callback of BULK_IN
 */
static void bulk_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	DBG_MSGG("----- %s(%d)", __func__, req->actual);

	if (req->status == -ECONNRESET) {
		usb_ep_fifo_flush(ep);

		if (req->actual > ep->maxpacket)
			req->actual -= ep->maxpacket;
		else
			req->actual = 0;
	}

	if (g_mtp->tx_status == MTP_STATUS_BULK_IN_WAIT) {
		g_mtp->tx_status = MTP_STATUS_OK;
		up(&g_mtp->bulk_comp);
	} else {
		DBG_MSG("%s() req->status = %d", __func__, req->status);
		DBG_MSG("%s() tx_status = %d", __func__, g_mtp->tx_status);
	}
}

/**
 * Transferring completion callback of BULK_OUT
 */
static void bulk_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	DBG_MSGG("----- %s(%d)", __func__, req->actual);

	if (req->status == -ECONNRESET) {
		INFO_MSGG("----- %s(%d)", __func__, req->actual);
		usb_ep_fifo_flush(ep);
	}

	if (g_mtp->tx_status == MTP_STATUS_BULK_OUT_WAIT) {
		g_mtp->tx_status = MTP_STATUS_OK;
		if (g_mtp->host_cancel_count != 0) {
			g_mtp->host_cancel_count = 0;
			req->status = -ETIMEDOUT;
		}
		up(&g_mtp->bulk_comp);
	} else {
		DBG_MSGG("%s() req->status = %d", __func__, req->status);
		DBG_MSGG("%s() tx_status = %d", __func__, g_mtp->tx_status);
	}
}

/**
 * Transferring completion callback of INTERRUPT_IN
 */
static void intr_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status == -ECONNRESET)
		usb_ep_fifo_flush(ep);

	if (g_mtp->tx_status == MTP_STATUS_INTR_WAIT) {
		g_mtp->tx_status = MTP_STATUS_OK;
		up(&g_mtp->interrupt_comp);
	}
}

/**
 * Memory securing for Endpoints
 */
static int alloc_endpoint()
{
	sema_init(&g_mtp->interrupt_comp, 0);
	sema_init(&g_mtp->bulk_comp, 0);

	/* Bulk IN */
	g_mtp->inreq = usb_ep_alloc_request(g_mtp->bulk_in, GFP_ATOMIC);
	if (g_mtp->inreq == NULL)
		return -ENOMEM;

	g_mtp->p_send_buf = kmalloc(
			WRITE_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
	if (g_mtp->p_send_buf == NULL)
		return -ENOMEM;

	g_mtp->inreq->complete = bulk_in_complete;
	g_mtp->inreq->buf = g_mtp->p_send_buf;

	/* Bulk OUT */
	g_mtp->outreq = usb_ep_alloc_request(g_mtp->bulk_out, GFP_ATOMIC);
	if (g_mtp->outreq == NULL)
		return -ENOMEM;

	g_mtp->p_recv_buf = kmalloc(
			READ_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
	if (g_mtp->p_recv_buf == NULL)
		return -ENOMEM;

	g_mtp->outreq->complete = bulk_out_complete;
	g_mtp->outreq->buf = g_mtp->p_recv_buf;

	/* Interrupt IN */
	g_mtp->intreq = usb_ep_alloc_request(g_mtp->intr_in, GFP_ATOMIC);
	if (g_mtp->intreq == NULL)
		return -ENOMEM;

	g_mtp->p_event_buf = kmalloc(
			EVENT_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
	if (g_mtp->p_event_buf == NULL)
		return -ENOMEM;

	g_mtp->intreq->complete = intr_in_complete;
	g_mtp->intreq->buf = g_mtp->p_event_buf;

	return 0;
}

/**
 * Memory opening for Endpoints
 */
static int free_endpoint()
{
	/* Interrupt IN */
	if (g_mtp->intreq) {
		down(&g_mtp->intr_in_use_sem);

		if (g_mtp->p_event_buf != NULL) {
			kfree(g_mtp->intreq->buf);
			g_mtp->p_event_buf = NULL;
		}

		usb_ep_free_request(g_mtp->intr_in, g_mtp->intreq);
		g_mtp->intreq = NULL;
		up(&g_mtp->intr_in_use_sem);
	}

	/* Bulk OUT */
	if (g_mtp->outreq) {
		down(&g_mtp->bulk_out_use_sem);

		if (g_mtp->p_recv_buf != NULL) {
			kfree(g_mtp->p_recv_buf);
			g_mtp->p_recv_buf = NULL;
		}

		usb_ep_free_request(g_mtp->bulk_out, g_mtp->outreq);
		g_mtp->outreq = NULL;
		up(&g_mtp->bulk_out_use_sem);
	}

	/* Bulk IN */
	if (g_mtp->inreq) {
		down(&g_mtp->bulk_in_use_sem);

		if (g_mtp->p_send_buf != NULL) {
			kfree(g_mtp->p_send_buf);
			g_mtp->p_send_buf = NULL;
		}

		usb_ep_free_request(g_mtp->bulk_in, g_mtp->inreq);
		g_mtp->inreq = NULL;
		up(&g_mtp->bulk_in_use_sem);
	}

	return 0;
}

/**
 * Main Loop
 */
static void mtp_main_thread(struct usb_class_driver *p_class)
{
	INFO_MSGG("=================================");
	INFO_MSGG("---  mtp_main_thread START");

	g_mtp->config = 1;
	alloc_endpoint();

	g_mtp->bulk_in_desc = p_class->bulk_in_desc;
	g_mtp->bulk_out_desc = p_class->bulk_out_desc;
	g_mtp->intr_in_desc = p_class->intr_in_desc;

	g_mtp->devStatus.bBulkInCanceled	= FALSE;
	g_mtp->devStatus.bBulkOutCanceled	= FALSE;
	g_mtp->devStatus.b_Cancelling		= FALSE;
	g_mtp->devStatus.bReset			= FALSE;
	g_mtp->devStatus.bBusy			= FALSE;
	g_mtp->b_BulkInWorking			= FALSE;
	g_mtp->b_BulkOutWorking			= FALSE;

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
	g_mtp->b_Firstflg = TRUE;
	g_mtp->b_Vbus = TRUE;
	g_mtp->devStatus.bSuspend = FALSE;
#endif	/* CONFIG_USB_ANDROID_NWM */

	p_class->running = 1;

	/* Initialization completion */
	if (g_mtp->b_InitSuccess != FALSE)
		up(&g_mtp->mtpinit_comp);

	/* Because NWM receives it, data first time analyzes this */
	g_mtp->outreq->actual = p_class->bulk_out_req->actual;
	g_mtp->outreq->status = p_class->bulk_out_req->status;
	memcpy(g_mtp->outreq->buf, p_class->bulk_out_req->buf,
		 g_mtp->outreq->actual);
	INFO_MSGG("---  Got first bulk data, actual=%d", g_mtp->outreq->actual);

	/* Disconnect and stop request waiting */
	if (p_class->running != 0)
		wait_for_completion(&g_mtp->mtpthread_comp);

	free_endpoint();
	p_class->running = 0;

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)
	g_mtp->b_Firstflg = FALSE;
	/*g_mtp->b_Vbus = FALSE;*/
#endif	/* CONFIG_USB_ANDROID_NWM */

	INFO_MSGG("---  mtp_main_thread END");
	INFO_MSGG("=================================");
}

/**
 * SET_CONFIGURATION
 */
static int do_set_config(u8 new_config)
{
	int		rc = 0;

	INFO_MSGG("----- %s() Start", __func__);
	INFO_MSGG("----- %s() new_config = %d, config = %d",
		 __func__, new_config, g_mtp->config);

	if ((new_config == 0) && (g_mtp->config == 0))
		return rc;

	rc = do_set_interface(-1);

	INFO_MSGG("----- %s(%d) End", __func__, new_config);

	return rc;
}

/**
 * SET_INTERFACE
 */
static int do_set_interface(int altsetting)
{
	PT_USB_CLASS_DRIVER p_class;

	INFO_MSGG("----- %s(%d) Start", __func__, altsetting);
	INFO_MSGG("----- %s(%d) Enter", __func__, altsetting);

	g_mtp->b_Firstflg = TRUE;
	g_mtp->config = 0;

	if (g_mtp->tx_status == MTP_STATUS_INTR_WAIT) {
		g_mtp->tx_status = MTP_STATUS_OK;

		/* The intrrupt waiting is Wakeup */
		up(&g_mtp->interrupt_comp);
	}

	if ((g_mtp->tx_status == MTP_STATUS_BULK_IN_WAIT)
		 || (g_mtp->tx_status == MTP_STATUS_BULK_OUT_WAIT)) {
		g_mtp->tx_status = MTP_STATUS_OK;

		/* The read/write waiting is Wakeup */
		up(&g_mtp->bulk_comp);
	}

	p_class = &g_mtp_regist_driver;
	if (p_class->running) {
		/* The Main Loop is Wakeup */
		complete(&g_mtp->mtpthread_comp);
	}

	/*g_mtp->devStatus.b_Cancelling = 0;*/
	/*g_mtp->devStatus.bBulkInCanceled = 0;*/
	/*g_mtp->devStatus.bBulkOutCanceled = 0;*/
	/*g_mtp->devStatus.bReset = 0;*/
	/*g_mtp->devStatus.bBusy = 0;*/

	if (altsetting < 0)
		return 0;

	INFO_MSGG("----- %s() End", __func__);

	return 0;
}

/**
 * Preprocessing of Class Cancel
 */
static int do_cancel_io_req(struct usb_request *req)
{
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	if (g_mtp->b_LastTransferIsIn)
		ds->bBulkInCanceled = TRUE;
	else
		ds->bBulkOutCanceled = TRUE;

	return sizeof(USB_PTP_CANCELREQ_DATA);
}

/**
 * Preprocessing of Class Reset
 */
static int do_reset_req(struct usb_request *req)
{
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	ds->bReset = TRUE;

	return 0;
}

/**
 * Class Status processing
 */
static int do_get_status_req(struct usb_request *req)
{
	USB_PTP_STATUSREQ_DATA	s_Status;
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	s_Status.wLength = GETSTATUS_HEADER_SIZE;

	if (ds->b_Cancelling != FALSE)
		s_Status.Code = STATUS_DEVICE_BUSY;

	else
		s_Status.Code = STATUS_DEVICE_OK;

	if ((ds->bBulkOutCanceled == FALSE)
		 && (ds->bBulkInCanceled == FALSE)) {
		ds->b_Cancelling = FALSE;

		if (g_mtp->b_BulkInSetStall)
			_mtp_ioctl_unstall(MTP_BULK_IN_STALL);

		if (g_mtp->b_BulkOutSetStall)
			_mtp_ioctl_unstall(MTP_BULK_OUT_STALL);
	}

	memcpy(req->buf, &s_Status, s_Status.wLength);

	return s_Status.wLength;
}

/**
 * Class Cancel processing
 */
static void exe_cancel_io_req(struct usb_request *req)
{
	USB_PTP_CANCELREQ_DATA	s_Cancel;
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	DBG_MSG("----- %s() Start", __func__);

	memcpy(&s_Cancel, req->buf, sizeof(USB_PTP_CANCELREQ_DATA));
	INFO_MSGG("wCancelIOCode = 0x%x, TransactionId = 0x%x",
		 s_Cancel.wCancelIOCode, s_Cancel.TransactionId);
	INFO_MSGG("                      TransactionID = 0x%x",
		 ds->TransactionID);

	if ((s_Cancel.wCancelIOCode == USB_PTP_CANCELL_CODE)
			&& (s_Cancel.TransactionId == ds->TransactionID)) {

		g_mtp->host_cancel_count = 0;

		if (ds->bBulkInCanceled != FALSE) {
			ds->b_Cancelling = TRUE;
			if (g_mtp->b_BulkInWorking != FALSE) {
				usb_ep_dequeue(g_mtp->bulk_in, g_mtp->inreq);
				g_mtp->b_BulkInWorking = FALSE;
			}
		}

		if (ds->bBulkOutCanceled != FALSE) {
			ds->b_Cancelling = TRUE;
			if (g_mtp->b_BulkOutWorking != FALSE) {
				usb_ep_dequeue(g_mtp->bulk_out, g_mtp->outreq);
				g_mtp->b_BulkOutWorking = FALSE;
			}
		}
	} else {
		INFO_MSG(" TransactionID Failed !!");
		ds->bBulkInCanceled = FALSE;
		ds->bBulkOutCanceled = FALSE;
	}

	DBG_MSGG("----- %s() End", __func__);
}

/**
 * Class Reset processing
 */
static void exe_reset_req(struct usb_request *req)
{
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	if (ds->bReset != FALSE) {
		if (g_mtp->b_BulkInWorking)
			usb_ep_dequeue(g_mtp->bulk_in, g_mtp->inreq);

		if (g_mtp->b_BulkOutWorking)
			usb_ep_dequeue(g_mtp->bulk_out, g_mtp->outreq);
	}

	ds->bBulkInCanceled = FALSE;
	ds->bBulkOutCanceled = FALSE;
	ds->b_Cancelling = FALSE;
}

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)

#define	FC_NWM_OSD_VENDOR_CODE			0x30
#define FC_NWM_ALT_VALUE			0

/**
 * Vendor Request processing
 */
static int vendor_setup_req(const struct usb_ctrlrequest *ctrl)
{
	int		value = -EOPNOTSUPP;
	u16		w_value = le16_to_cpu(ctrl->wValue);

	if (w_value != FC_NWM_ALT_VALUE)
		return value;

	if (ctrl->bRequest == FC_NWM_OSD_VENDOR_CODE) {
		if (g_mtp->open_count)
			value = g_mtp->open_count;
		else
			INFO_MSG("Not Open MTP Driver");
	}

	return value;
}
#endif	/* CONFIG_USB_ANDROID_NWM */

/**
 * Class Request processing
 */
static int class_setup_req(const struct usb_ctrlrequest *ctrl)
{
	struct usb_request *req = g_mtp->ep0req;
	int		value = -EOPNOTSUPP;
	u16		w_index = le16_to_cpu(ctrl->wIndex);
	u16		w_length = le16_to_cpu(ctrl->wLength);
	u16		w_value = le16_to_cpu(ctrl->wValue);

	DBG_MSG("----- %s() Start", __func__);

	g_mtp->class_req_type = ctrl->bRequest;

	/* Handle Bulk-only class-specific requests */
	switch (ctrl->bRequest) {
	case USB_PTPREQUEST_CANCELIO:		/* 0x64 */
		INFO_MSG("USB_PTPREQUEST_CANCELIO");

		if ((w_value == 0x00) && (w_index == 0x00)
			 && (w_length == 0x06)) {
			value = do_cancel_io_req(req);
		}
		break;

	case USB_PTPREQUEST_GETEVENT:		/* 0x65 */
		INFO_MSG("USB_PTPREQUEST_GETEVENT");
		break;

	case USB_PTPREQUEST_RESET:		/* 0x66 */
		INFO_MSG("USB_PTPREQUEST_RESET");

		if ((w_value == 0x00) && (w_index == 0x00)
			 && (w_length == 0x00)) {
			value = do_reset_req(req);
		}
		break;

	case USB_PTPREQUEST_GETSTATUS:		/* 0x67 */
		INFO_MSG("USB_PTPREQUEST_GETSTATUS");
		if ((w_value == 0x00) && (w_index == 0x00))
			value = do_get_status_req(req);
		break;

	default:
		DBG_MSGG("Unknown Request");
		break;
	}

	DBG_MSGG("----- %s() End", __func__);

	return value;
}

/**
 * Standard Request processing
 */
static int standard_setup_req(const struct usb_ctrlrequest *ctrl)
{
	int		value = -EOPNOTSUPP;
	u16		w_index = le16_to_cpu(ctrl->wIndex);
	u16		w_value = le16_to_cpu(ctrl->wValue);

	DBG_MSGG("----- %s() Start", __func__);

	switch (ctrl->bRequest) {
	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != (USB_DIR_IN
			 | USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;

		switch (w_value >> 8) {

		case USB_DT_DEVICE:
			value = 0;
			break;
#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			value = 0;
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			value = 0;
			break;
#endif
		case USB_DT_CONFIG:
			value = 0;
			break;

		case USB_DT_STRING:
			value = 0;
			break;
		}
		break;

	case USB_REQ_SET_CONFIGURATION:
		if (ctrl->bRequestType != (USB_DIR_OUT
			 | USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;

		if ((w_value == 1) || (w_value == 0)) {
			do_set_config(w_value);
			value = DELAYED_STATUS;
		}
		break;

	case USB_REQ_GET_CONFIGURATION:
		if (ctrl->bRequestType != (USB_DIR_IN
			 | USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;
		value = 0;
		break;

	case USB_REQ_SET_INTERFACE:
		if (ctrl->bRequestType != (USB_DIR_OUT |
			 USB_TYPE_STANDARD | USB_RECIP_INTERFACE))
			break;

		if (g_mtp->config && (w_index == 0)) {
			do_set_interface(-1);
			value = DELAYED_STATUS;
		}
		break;

	case USB_REQ_GET_INTERFACE:
		value = 0;
		break;

	default:
		break;
	}

	DBG_MSGG("----- %s() End", __func__);

	return value;
}

/**
 * bind
 */
static int mtp_bind(struct usb_gadget *gadget)
{
	PT_USB_CLASS_DRIVER p_class;

	DBG_MSG("----- %s() Start", __func__);

	g_mtp->gadget = gadget;
	set_gadget_data(gadget, g_mtp);
	g_mtp->ep0 = gadget->ep0;

	p_class = &g_mtp_regist_driver;
	g_mtp->org_gadget = p_class->org_gadget;
	g_mtp->ep0req = p_class->ep0req;
	g_mtp->bulk_in = p_class->bulk_in;
	g_mtp->bulk_out = p_class->bulk_out;
	g_mtp->intr_in = p_class->intr_in;

	up(&g_mtp->drv_reg_comp);

	usb_mtptrans_start();

	DBG_MSG("----- %s() End", __func__);

	return 0;
}

/**
 * unbind
 */
static void mtp_unbind(struct usb_gadget *gadget)
{
	DBG_MSG("----- %s() Start", __func__);

	set_gadget_data(gadget, NULL);
	g_mtp->gadget = NULL;
	g_mtp->org_gadget = NULL;

	DBG_MSG("----- %s() End", __func__);
}

/**
 * Disconnect
 */
static void mtp_disconnect(struct usb_gadget *gadget)
{
	unsigned long	flags;

	INFO_MSG("----- %s() Start", __func__);

	spin_lock_irqsave(&g_mtp->lock, flags);
	do_set_config(0);
	g_mtp->b_Vbus = FALSE;
	spin_unlock_irqrestore(&g_mtp->lock, flags);

	INFO_MSG("----- %s() End", __func__);
}

/**
 * SETUP
 */
static int mtp_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl)
{
	int			rc;

	DBG_MSG("----- %s() Start", __func__);
	INFO_MSG("----- %s() Start", __func__);

	g_mtp->b_Vbus = TRUE;
	g_mtp->devStatus.bSuspend = FALSE;

#if defined(CONFIG_USB_ANDROID_NWM) || defined(CONFIG_USB_ANDROID_NWM_MODULE)

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
		rc = standard_setup_req(ctrl);
	else if (ctrl->bRequestType & USB_TYPE_CLASS)
		rc = class_setup_req(ctrl);
	else
		rc = vendor_setup_req(ctrl);

#else

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS)
		rc = class_setup_req(ctrl);
	else
		rc = standard_setup_req(ctrl);

#endif	/* CONFIG_USB_ANDROID_NWM */

	DBG_MSG("----- %s(%d) End", __func__, rc);

	return rc;
}

/**
 * USB Suspned
 */
static void mtp_suspend(struct usb_gadget *gadget)
{
	unsigned long	flags;
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	INFO_MSGG("----- %s() Start", __func__);

	spin_lock_irqsave(&g_mtp->lock, flags);
	ds->bSuspend = TRUE;
	do_set_config(0);
	spin_unlock_irqrestore(&g_mtp->lock, flags);

	INFO_MSGG("----- %s() End", __func__);
}

/**
 * USB Resume
 */
static void mtp_resume(struct usb_gadget *gadget)
{
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	ds->bSuspend = FALSE;
}

/**
 * Timer Start
 */
static void _nwm_mtp_cancel_timer_restart()
{
	if (g_mtp->b_timer_flag != FALSE) {
		mod_timer(&g_mtp->cancel_timer,
			 jiffies + USB_HOST_CANCEL_TIMEOUT);
	}
}

/**
 * Timer Init & Start
 */
static void _nwm_mtp_cancel_timer_start()
{
	init_timer(&g_mtp->cancel_timer);

	g_mtp->host_cancel_count = 0;
	g_mtp->b_timer_flag = TRUE;

	g_mtp->cancel_timer.data = (unsigned long)g_mtp;
	g_mtp->cancel_timer.function = _nwm_mtp_cancel_timer_fn;
	g_mtp->cancel_timer.expires  = jiffies + USB_HOST_CANCEL_TIMEOUT;
	add_timer(&g_mtp->cancel_timer);
}

/**
 * Timer Stop
 */
static void _nwm_mtp_cancel_timer_stop()
{
	if (g_mtp->b_timer_flag != FALSE) {
		del_timer(&g_mtp->cancel_timer);
		g_mtp->b_timer_flag = FALSE;
	}
}

/**
 * Timer processing
 */
static void _nwm_mtp_cancel_timer_fn(unsigned long arg)
{
	g_mtp->host_cancel_count++;
	if (g_mtp->b_BulkOutWorking)
		usb_ep_dequeue(g_mtp->bulk_out, g_mtp->outreq);
	_nwm_mtp_cancel_timer_stop();
}

/**
 * Cancel processing
 */
static int _usb_mtp_host_cancel(size_t count)
{
	int		nret = 0;

	DBG_MSGG("----- %s() Start", __func__);

	g_mtp->b_BulkOutWorking = TRUE;
	g_mtp->outreq->length = g_mtp->bulk_out->maxpacket << 2;

	_nwm_mtp_cancel_timer_start();

	while (1) {
		if (g_mtp->b_BulkOutWorking == FALSE)
			break;

		g_mtp->tx_status = MTP_STATUS_BULK_OUT_WAIT;
		nret = start_transfer(g_mtp->bulk_out, g_mtp->outreq);
		if (nret != 0) {
			ERR_MSG("%s() start_transfer nret=%d", __func__, nret);
			g_mtp->b_BulkOutWorking = FALSE;
			nret = USB_MTP_ERROR;
		}

		_nwm_mtp_cancel_timer_restart();
		nret = down_interruptible(&g_mtp->bulk_comp);
		if (nret != 0) {
			INFO_MSGG("=== Bulk OUT Failed(INTERRUPT SIGNAL)");
			_mtp_ioctl_abort_transfer();
			g_mtp->b_BulkOutWorking = FALSE;
			nret = -EINTR;
		}

		if (g_mtp->outreq->status == -ETIMEDOUT) {
			INFO_MSGG("----- Host Cancel Timeout");
			g_mtp->b_BulkOutWorking = FALSE;
			nret = 0;
		}
	}

	DBG_MSGG("----- %s() End", __func__);

	return nret;
}

/**
 * Read
 */
static int _usb_mtp_read(char *buffer, size_t count)
{
	int		retcode;
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	DBG_MSGG("----- %s() Enter", __func__);

	if (!g_mtp->config || ds->bReset || ds->bSuspend) {
		INFO_MSG("=== MTP Device Disconnected");
		ds->bReset = FALSE;
		return USB_DEVICE_NOT_CONNECTED;
	}

	if (count > READ_BUFFER_SIZE) {
		ERR_MSG("%s too large read size", __func__);
		return USB_MTP_ERROR;
	}

	if (ds->b_Cancelling != FALSE) {
		INFO_MSGG("--- _usb_mtp_host_cancel()");
		retcode = _usb_mtp_host_cancel(count);
		if (retcode == 0) {
			/* Host Cancel */
			ds->bBulkOutCanceled = FALSE;
			ds->bBulkInCanceled = FALSE;
			INFO_MSGG("----- Host Cancel End");
			return USB_HOST_CANCELING;
		}
		return 0;
	}

	g_mtp->b_LastTransferIsIn = FALSE;

	if (g_mtp->b_DataZeroRequired)
		g_mtp->outreq->zero = 1;
	else
		g_mtp->outreq->zero = 0;

	/* Data transfer */
	if (g_mtp->b_Firstflg == FALSE) {
		g_mtp->b_BulkOutWorking = TRUE;
		g_mtp->outreq->length = count;
		g_mtp->tx_status = MTP_STATUS_BULK_OUT_WAIT;
		retcode = start_transfer(g_mtp->bulk_out, g_mtp->outreq);
		if (retcode != 0) {
			ERR_MSG("%s() start_transfer retcode=%d",
				 __func__, retcode);
			g_mtp->b_BulkOutWorking = FALSE;
			g_mtp->tx_status = MTP_STATUS_OK;
			return USB_MTP_ERROR;
		}

		/* Transferring completion waiting */
		retcode = down_interruptible(&g_mtp->bulk_comp);
		if (retcode != 0) {
			INFO_MSG("=== Bulk OUT Failed(INTERRUPT SIGNAL)");
			if (g_mtp->outreq->status != 0) {
				if (g_mtp->outreq->status == -EINPROGRESS) {
					usb_ep_dequeue(
						g_mtp->bulk_out, g_mtp->outreq);
					retcode = down_interruptible(
						&g_mtp->bulk_comp);
				} else {
					g_mtp->b_BulkOutWorking = FALSE;
					return -EINTR;
				}
			}
		}
	} else {
		/* Because the first Read(Bulk OUT) is done by the NWM driver,
		 * start_transfer() is not called
		*/
		g_mtp->b_Firstflg = FALSE;
	}

	g_mtp->b_BulkOutWorking = FALSE;

	/* Data transfer status check */
	if (g_mtp->outreq->status != 0)
		return USB_MTP_ERROR;

	/* Data transfer judgment */
	if (g_mtp->config && !ds->bReset) {
		DBG_MSGG("---- bulkout actual_len = %d",
			 (int)g_mtp->outreq->actual);
		DBG_MSGG("---- bulkout status = %d",
			 (int)g_mtp->outreq->status);

		/* When the data transmission is completed ahead of that,
		 * it doesn't treat as Error though the cancellation was
		 * generated.
		*/
		if (ds->bBulkOutCanceled != FALSE) {
			if (g_mtp->outreq->actual != count)
				return USB_MTP_ERROR;
		}

		retcode = copy_to_user(buffer, g_mtp->outreq->buf,
			 g_mtp->outreq->actual);
		if (retcode != 0)
			return USB_MTP_ERROR;
		else
			return g_mtp->outreq->actual;
	} else {
		INFO_MSGG("=== Bulk OUT Failed(Disconnect/Aborted)");
		return USB_MTP_ERROR;
	}
}

/**
 * Write
 */
static int _usb_mtp_write(const char *buffer, size_t count)
{
	int		retcode;
	PUSB_MTP_XCHG_STATUS	ds = &g_mtp->devStatus;

	DBG_MSGG("----- %s(%d)", __func__, count);

	if (!g_mtp->config || ds->bReset || ds->bSuspend) {
		DBG_MSG("=== MTP Device Disconnected");
		return USB_DEVICE_NOT_CONNECTED;
	}

	if (count > WRITE_BUFFER_SIZE) {
		DBG_MSG("%s too large write size", __func__);
		return USB_MTP_ERROR;
	}

	if ((ds->b_Cancelling) || (ds->bBulkInCanceled)) {
		DBG_MSG("=== MTP device write cancelling");
		return USB_MTP_ERROR;
	}

	g_mtp->b_LastTransferIsIn = TRUE;

	/* User Memory -> Kernel Buffer */
	retcode = copy_from_user(g_mtp->inreq->buf, buffer, count);
	if (retcode != 0) {
		DBG_MSG("=== %s copy_from_user() Failed", __func__);
		return USB_MTP_ERROR;
	}

	g_mtp->inreq->length = count;
	if (g_mtp->b_DataZeroRequired)
		g_mtp->inreq->zero = 1;
	else
		g_mtp->inreq->zero = 0;

	/* Data transfer */
	g_mtp->b_BulkInWorking = TRUE;
	g_mtp->tx_status = MTP_STATUS_BULK_IN_WAIT;
	retcode = start_transfer(g_mtp->bulk_in, g_mtp->inreq);
	if (retcode != 0) {
		ERR_MSG("usb_mtptrans_write write_transfer retcode=%d",
			 (int)retcode);
		g_mtp->b_BulkInWorking = FALSE;
		g_mtp->tx_status = MTP_STATUS_OK;
		return USB_MTP_ERROR;
	} else {
		retcode = down_interruptible(&g_mtp->bulk_comp);
		if (retcode != 0) {
			DBG_MSG("=== Bulk IN Failed(INTERRUPT SIGNAL)");
			_mtp_ioctl_abort_transfer();
			g_mtp->b_BulkInWorking = FALSE;
			return -EINTR;
		}
	}

	g_mtp->b_BulkInWorking = FALSE;

	/* Data transfer status check */
	if (g_mtp->inreq->status != 0)
		return USB_MTP_ERROR;

	/* Data transfer judgment */
	if (g_mtp->config && !ds->bReset && !ds->bBulkInCanceled)
		return g_mtp->inreq->actual;
	else {
		DBG_MSG("=== Bulk IN Failed(Disconnect/Aborted)");
		return USB_MTP_ERROR;
	}
}

/**
 * Read (Bulk OUT)
 */
static ssize_t usb_mtptrans_read(
	struct file *file,
	char *buffer,
	size_t count,
	loff_t *ppos
)
{
	int		nret;

	DBG_MSGG("");
	DBG_MSGG("----- %s() Enter", __func__);
	INFO_MSGG("----- read size (%d)", count);

	down(&g_mtp->bulk_out_use_sem);
	nret = _usb_mtp_read(buffer, count);
	up(&g_mtp->bulk_out_use_sem);

	return nret;
}

/**
 * Write (Bulk IN)
 */
static ssize_t usb_mtptrans_write(
	struct file *file,
	const char *buffer,
	size_t count,
	loff_t *ppos
)
{
	int		nret;

	DBG_MSGG("");
	DBG_MSGG("----- %s(%d)", __func__, count);
	DBG_MSGG("----- write size (%d)", count);

	down(&g_mtp->bulk_in_use_sem);
	nret = _usb_mtp_write(buffer, count);
	up(&g_mtp->bulk_in_use_sem);

	return nret;
}

/**
 * I/O Control
 */
static int usb_mtptrans_ioctl(
	struct inode *inode,
	struct file *file,
	unsigned int cmd,
	unsigned long arg
)
{
	int		ret = 0;
	long		dir;
	T_MTP_EVENT	*event;
	PUSB_MTP_XCHG_STATUS	devStatus;

	DBG_MSGG("--- %s called.", __func__);

	switch (cmd) {
	case MTP_IOCTL_INITIALIZE:
		ret = _mtp_ioctl_initialize();
		break;

	case MTP_IOCTL_RESET:
		ret = _mtp_ioctl_reset();
		break;

	case MTP_IOCTL_STALL:
		dir = *(long *)arg;
		ret = _mtp_ioctl_stall(dir);
		break;

	case MTP_IOCTL_UNSTALL:
		dir = *(long *)arg;
		ret = _mtp_ioctl_unstall(dir);
		break;

	case MTP_IOCTL_DISCONNECT:
		ret = _mtp_ioctl_disconnect();
		break;

	case MTP_IOCTL_CONNECT:
		ret = _mtp_ioctl_connect();
		break;

	case MTP_IOCTL_IS_CONNECTED:
		ret = _mtp_ioctl_isconnected();
		break;

	case MTP_IOCTL_INITWAIT:
		ret = _mtp_ioctl_waitinit();
		break;

	case MTP_IOCTL_SENDEVENT:
		event = (T_MTP_EVENT *)arg;
		ret = _mtp_ioctl_sendevent(
			event->pBuffer, event->size, &event->count);
		break;

	case MTP_IOCTL_SET_TX_FLAG:
		g_mtp->b_DataZeroRequired = *(u32 *)arg;
		break;

	case MTP_IOCTL_XCHG_STATUS:
		devStatus = (PUSB_MTP_XCHG_STATUS) arg;
		ret = _mtp_ioctl_xchg_status(devStatus);
		break;

	case MTP_IOCTL_GET_VBUS_LEVEL:
		ret = _mtp_ioctl_get_vbus_level();
		break;

	case MTP_IOCTL_ABORT_TRANSFER:
		ret = _mtp_ioctl_abort_transfer();
		break;

	default:
		ERR_MSG("MTP driver, Illegal IOCTL");
		break;
	}

	return ret;
}

/**
 * MTP Driver Open
 */
static int usb_mtptrans_open(struct inode *inode, struct file *file)
{
	DBG_MSG("%s() Enter", __func__);

	spin_lock(&g_mtp->lock);
	if (g_mtp->open_count) {
		spin_unlock(&g_mtp->lock);
		ERR_MSG("Already MTP driver open !!");
		return -EBUSY;
	}
	g_mtp->open_count++;
	spin_unlock(&g_mtp->lock);

	file->private_data = g_mtp;

	return 0;
}

/**
 * MTP Driver Close
 */
static int usb_mtptrans_release(struct inode *inode, struct file *file)
{
	DBG_MSG("%s() Enter", __func__);

	spin_lock(&g_mtp->lock);
	g_mtp->open_count--;
	spin_unlock(&g_mtp->lock);

	return 0;
}

const struct file_operations usb_mtptrans_fops = {
	.owner = THIS_MODULE,
	.read = usb_mtptrans_read,
	.write = usb_mtptrans_write,
	.ioctl = usb_mtptrans_ioctl,
	.open = usb_mtptrans_open,
	.release = usb_mtptrans_release,
};

static struct miscdevice usb_mtp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = USB_MTP_MODULE_NAME,
	.fops = &usb_mtptrans_fops,
};

/**
 * MTP Driver Start
 */
static int usb_mtptrans_start(void)
{
	int ret = 0;

	DBG_MSG("%s() Enter", __func__);

	if (g_mtp->open_count) {
		ret = -1;
		ERR_MSG("Already MTP driver open = %d", ret);
		goto err_class_create;
	}

	ret = down_interruptible(&g_mtp->drv_reg_comp);
	if (ret != 0) {
		DBG_MSG("=== START Failed(INTERRUPT SIGNAL)");
		ret = -EINTR;
		goto err_class_create;
	}
	g_mtp->b_InitSuccess = FALSE;

	misc_register(&usb_mtp_device);
	g_mtp->b_InitSuccess = TRUE;
	g_mtp->open_count = 0;

	DBG_MSG("Register MTP Device Success, ret = %d", ret);

	return 0;

err_class_create:

	return ret;
}

/**
 * MTP Driver Stop
 */
static void usb_mtptrans_stop(void)
{
	DBG_MSG("%s() Enter", __func__);

	misc_deregister(&usb_mtp_device);
	g_mtp->b_InitSuccess = FALSE;
}

/**
 * Liberating of memory of MTP Driver
 */
static void mtp_free(void)
{
}

/**
 * MTP Driver initialization
 */
static int __init mtp_init(void)
{
	int		rc = 0;
	PT_USB_CLASS_DRIVER p_class;

	DBG_MSG("%s() Start", __func__);

	memset(g_mtp, 0, sizeof(struct mtp_device));
	spin_lock_init(&g_mtp->lock);

	init_completion(&g_mtp->mtpthread_comp);

	sema_init(&g_mtp->interrupt_comp, 0);
	sema_init(&g_mtp->bulk_comp, 0);
	sema_init(&g_mtp->mtpinit_comp, 0);
	sema_init(&g_mtp->drv_reg_comp, 0);
	sema_init(&g_mtp->bulk_in_use_sem, 1);
	sema_init(&g_mtp->bulk_out_use_sem, 1);
	sema_init(&g_mtp->intr_in_use_sem, 1);
	g_mtp->tx_status = MTP_STATUS_OK;

	/* Registration to NWM Driver */
	p_class = &g_mtp_regist_driver;
	memset(p_class, 0, sizeof(T_USB_CLASS_DRIVER));
	p_class->pDriver = &mtp_driver;
	p_class->ClassType = USB_MTP_TYPE;
	p_class->pMainLoop = mtp_main_thread;
	p_class->pContext = g_mtp;
	p_class->ep0_complete = ep0_complete;

	rc = fc_nwm_register_driver(p_class);
	if (rc != 0) {
		g_mtp->b_RegistFlag = FALSE;
		mtp_free();
		ERR_MSG("Regist Error for NWM");
		return rc;
	}
	g_mtp->b_RegistFlag = TRUE;

	DBG_MSG("%s() End", __func__);

	return rc;
}
module_init(mtp_init);


/**
 * MTP Driver termination
 */
static void __exit mtp_cleanup(void)
{
	PT_USB_CLASS_DRIVER p_class;

	DBG_MSG("%s() Enter", __func__);

	if (g_mtp->b_RegistFlag != FALSE) {
		p_class = &g_mtp_regist_driver;
		fc_nwm_unregister_driver(p_class);
	}

	usb_mtptrans_stop();
	complete(&g_mtp->mtpthread_comp);
	mtp_free();
}
module_exit(mtp_cleanup);

