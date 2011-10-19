/*
 *  drivers/usb/gadget/fc_nwm.c
 *    NEC Windows Media Driver
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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/kthread.h>


#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>


#include "gadget_chips.h"
#include "fc_nwm.h"


/*---------------------------------------------------------------------------*/
/* Debug Message */
/*#define USE_DEBUG_MESSAGE */
/*#define USE_INFOMATION_MESSAGE */
#include "usb_debug_message.h"
/*---------------------------------------------------------------------------*/

#include "usbstring.c"
#include "config.c"

#define DRIVER_DESC		"NEC Windows Media Gadget"
#define DRIVER_NAME		"g_nwm_gadget"
#define DRIVER_VERSION		"12 August 2010"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_LICENSE("Dual BSD/GPL");

/*---------------------------------------------------------------------------*/
/* Prototype */
static int  __init fc_nwm_init(void);
static void __exit fc_nwm_cleanup(void);

static int  __init _fc_nwm_alloc(void);
static void _fc_nwm_free(void);

static int  _fc_nwm_select_class_driver(u8 *p_buf, u32 len);
static int  _fc_nwm_populate_config_buf(
		struct usb_gadget *gadget,
		u8 *buf, u8 type, unsigned index);
static void _fc_nwm_bulk_out_complete(
		struct usb_ep *ep, struct usb_request *req);
static void _fc_nwm_ep0_complete(struct usb_ep *ep, struct usb_request *req);
static int	_fc_nwm_ep0_queue(PT_NWM_WORK p_work);
static void _fc_nwm_ep0_send_status_stage(void);
static void _fc_nwm_ep0_stall(PT_NWM_WORK p_work);

static int _fc_nwm_get_os_descriptor(PT_NWM_WORK p_work);
static int _fc_nwm_set_configuration(PT_NWM_WORK p_work, u8 config_value);
static int _fc_nwm_set_interface(PT_NWM_WORK p_work, int alt_value);
static int _fc_nwm_enable_endpoint(PT_NWM_WORK p_work);
static int _fc_nwm_disable_endpoint(PT_NWM_WORK p_work);
static int _fc_nwm_alloc_request(PT_NWM_WORK p_work);
static int _fc_nwm_free_request(PT_NWM_WORK p_work);

static void _fc_nwm_callback_setup(
		PT_NWM_WORK, const struct usb_ctrlrequest *);
static int  _fc_nwm_standard_setup(
		PT_NWM_WORK, const struct usb_ctrlrequest *);
static int  _fc_nwm_class_setup(
		PT_NWM_WORK, const struct usb_ctrlrequest *);
static int  _fc_nwm_vendor_setup(
		PT_NWM_WORK, const struct usb_ctrlrequest *);

static int  _fc_nwm_regist_state(PT_NWM_WORK p_work);
static int  _fc_nwm_wait_bulk_state(PT_NWM_WORK p_work);
static void _fc_nwm_recive_bulk_state(PT_NWM_WORK p_work);
static void _fc_nwm_class_loop_state(PT_NWM_WORK p_work);
static void _fc_nwm_reset_state(PT_NWM_WORK p_work);
static void _fc_nwm_disconnect_state(PT_NWM_WORK p_work);
static void _fc_nwm_unregist_state(PT_NWM_WORK p_work);

static void _fc_nwm_check_event(PT_NWM_WORK p_work);
static void _fc_nwm_wait_thread(PT_NWM_WORK p_work);
static void _fc_nwm_wakeup_thread(PT_NWM_WORK p_work);
static int  _fc_nwm_thread(void *pdata);

static int _fc_nwm_callback_bind(
	PT_NWM_WORK p_work, PT_USB_CLASS_DRIVER p_class,
	struct usb_gadget *gadget);
static void _fc_nwm_callback_unbind(PT_USB_CLASS_DRIVER p_class);

static int  _fc_nwm_bind(struct usb_gadget *gadget);
static void _fc_nwm_unbind(struct usb_gadget *gadget);
static int  _fc_nwm_setup(
		struct usb_gadget *, const struct usb_ctrlrequest *);
static void _fc_nwm_disconnect(struct usb_gadget *gadget);
static void _fc_nwm_suspend(struct usb_gadget *gadget);
static void _fc_nwm_resume(struct usb_gadget *gadget);



/*---------------------------------------------------------------------------*/
/* Global */
static const char longname[] = DRIVER_DESC;
static const char shortname[] = DRIVER_NAME;

static struct {
	unsigned short	vendor;
	unsigned short	product;
	unsigned short	release;
	unsigned short	mode;
}  mod_data = {
	.vendor		= DRIVER_VENDOR_ID,
	.product	= DRIVER_PRODUCT_ID,
	.release	= 0xffff,	/* Use controller chip type */
	.mode		= FC_NWM_MSC_MTP_MODE,
};


static PT_NWM_WORK gp_nwm_work;		/* Driver Work Area */

module_param_named(vendor, mod_data.vendor, ushort, S_IRUGO);
MODULE_PARM_DESC(vendor, "USB Vendor ID");

module_param_named(product, mod_data.product, ushort, S_IRUGO);
MODULE_PARM_DESC(product, "USB Product ID");

module_param_named(release, mod_data.release, ushort, S_IRUGO);
MODULE_PARM_DESC(release, "USB release number");

module_param_named(mode, mod_data.mode, ushort, S_IRUGO);
MODULE_PARM_DESC(mode, "Mode");


static struct usb_device_descriptor
device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_PER_INTERFACE,

	/* The next three values can be overridden by module parameters */
	.idVendor =		__constant_cpu_to_le16(DRIVER_VENDOR_ID),
	.idProduct =		__constant_cpu_to_le16(DRIVER_PRODUCT_ID),
	.bcdDevice =		__constant_cpu_to_le16(0xffff),

	.iManufacturer =	STRING_MANUFACTURER,
	.iProduct =		STRING_PRODUCT,
	.iSerialNumber =	STRING_SERIAL,
	.bNumConfigurations =	1,
};

static struct usb_config_descriptor
config_desc = {
	.bLength =		sizeof config_desc,
	.bDescriptorType =	USB_DT_CONFIG,

	/* wTotalLength computed by usb_gadget_config_buf() */
	.bNumInterfaces =	1,
	.bConfigurationValue =	CONFIG_VALUE,
	.iConfiguration =	STRING_CONFIG,
	.bmAttributes =		USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower =		500/2,	/* self-powered */
};

static struct usb_otg_descriptor
otg_desc = {
	.bLength =		sizeof(otg_desc),
	.bDescriptorType =	USB_DT_OTG,

	.bmAttributes =		USB_OTG_SRP,
};

/* There is only one interface. */

#ifdef USE_ALT_INTERFACE
static struct usb_interface_descriptor
intf_desc_0 = {
	.bLength = sizeof(struct usb_interface_descriptor),
	.bDescriptorType =	USB_DT_INTERFACE,

	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,

	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass =	USB_SC_SCSI,
	.bInterfaceProtocol =	USB_PR_BULK,
	.iInterface =		STRING_INTERFACE,
};

static struct usb_interface_descriptor
intf_desc_1 = {
	.bLength = sizeof(struct usb_interface_descriptor),
	.bDescriptorType =	USB_DT_INTERFACE,

	.bInterfaceNumber = 0,
	.bAlternateSetting = 1,

	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass =	USB_SC_SCSI,
	.bInterfaceProtocol =	USB_PR_BULK,
	.iInterface =		STRING_INTERFACE,
};
#else	/* !USE_ALT_INTERFACE */
static struct usb_interface_descriptor
intf_desc = {
	.bLength =		sizeof intf_desc,
	.bDescriptorType =	USB_DT_INTERFACE,

	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_MASS_STORAGE,
	.bInterfaceSubClass =	USB_SC_SCSI,
	.bInterfaceProtocol =	USB_PR_BULK,
	.iInterface =		STRING_INTERFACE,
};
#endif	/* USE_ALT_INTERFACE */

/* Three full-speed endpoint descriptors: bulk-in, bulk-out,
 * and interrupt-in. */

static struct usb_endpoint_descriptor
fs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor
fs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	/* wMaxPacketSize set by autoconfiguration */
};

static struct usb_endpoint_descriptor
fs_intr_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(2),
	.bInterval =		32,	/* frames -> 32 ms */
};

#ifdef USE_ALT_INTERFACE
static const struct usb_descriptor_header *fs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc_0,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_intr_in_desc,
	(struct usb_descriptor_header *) &intf_desc_1,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_intr_in_desc,
	NULL,
};
#else	/* !USE_ALT_INTERFACE */
static const struct usb_descriptor_header *fs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_intr_in_desc,
	NULL,
};
#endif	/* USE_ALT_INTERFACE */



#define FS_FUNCTION_PRE_EP_ENTRIES	2


#ifdef	CONFIG_USB_GADGET_DUALSPEED

/*
 * USB 2.0 devices need to expose both high speed and full speed
 * descriptors, unless they only run at full speed.
 *
 * That means alternate endpoint descriptors (bigger packets)
 * and a "device qualifier" ... plus more construction options
 * for the config descriptor.
 */
static struct usb_qualifier_descriptor
dev_qualifier = {
	.bLength =		sizeof dev_qualifier,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,

	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	.bNumConfigurations =	1,
};

static struct usb_endpoint_descriptor
hs_bulk_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_in_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor
hs_bulk_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_bulk_out_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	.bInterval =		1,	/* NAK every 1 uframe */
};

static struct usb_endpoint_descriptor
hs_intr_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	/* bEndpointAddress copied from fs_intr_in_desc during fsg_bind() */
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(2),
	.bInterval =		9,	/* 2**(9-1) = 256 uframes -> 32 ms */
};

#ifdef USE_ALT_INTERFACE
static const struct usb_descriptor_header *hs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc_0,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_intr_in_desc,
	(struct usb_descriptor_header *) &intf_desc_1,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_intr_in_desc,
	NULL,
};
#else	/* !USE_ALT_INTERFACE */
static const struct usb_descriptor_header *hs_function[] = {
	(struct usb_descriptor_header *) &otg_desc,
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_intr_in_desc,
	NULL,
};
#endif	/* USE_ALT_INTERFACE */

#define HS_FUNCTION_PRE_EP_ENTRIES	2

/* Maxpacket and other transfer characteristics vary by speed. */
#define ep_desc(g, fs, hs)	(((g)->speed == USB_SPEED_HIGH) ? (hs) : (fs))

#else

/* If there's no high speed support, always use the full-speed descriptor. */
#define ep_desc(g, fs, hs)	fs

#endif	/* !CONFIG_USB_GADGET_DUALSPEED */


static char				serial[13];


static struct usb_string strings[] = {
	{STRING_MANUFACTURER,		"Windows MTP Device"},
	{STRING_PRODUCT,		"NEC Windows Media Device"},
	{STRING_SERIAL,			serial},
	{STRING_CONFIG,			"Self-powered"},
	{STRING_INTERFACE,		"Mass Storage"},
	{FC_NWM_STRING_OS_DESCRIPTOR,	"MSFT1000"},
	{}
};

static struct usb_gadget_strings stringtab = {
	.language	= 0x0409,		/* en-us */
	.strings	= strings,
};


unsigned char os_feature_desc[] = {
	/* Header */
	FC_NWM_MS_DESC_LENGTH, 0x00, 0x00, 0x00,	/* dwLength */
	0x00, 0x01,					/* bcdVersion (1.00) */
	0x04, 0x00,					/* wIndex (0x0004) */
	0x01, 						/* bCount (0x0001) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* reserved */

	/* Function */
	0x00,					/* bFirstInterfaceNumber */
	0x01,					/* bInterfaceCount */
	'M', 'T', 'P', 0x00, 0x00, 0x00, 0x00, 0x00,	/* CompatibleID */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* SubcompatibleID */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00		/* reserved */
};


static struct usb_gadget_driver g_nwm_driver = {
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed		= USB_SPEED_HIGH,
#else
	.speed		= USB_SPEED_FULL,
#endif

	.function	= (char *) longname,
	.bind		= _fc_nwm_bind,
	.unbind		= _fc_nwm_unbind,
	.setup		= _fc_nwm_setup,
	.disconnect	= _fc_nwm_disconnect,
	.suspend	= _fc_nwm_suspend,
	.resume		= _fc_nwm_resume,

	.driver		= {
		.name		= (char *) shortname,
		/* .release = ... */
		/* .suspend = ... */
		/* .resume = ... */
	},
};


/*---------------------------------------------------------------------------*/
/* we must assign addresses for configurable endpoints (like net2280) */
static unsigned epnum;

/* #define MANY_ENDPOINTS */
#ifdef MANY_ENDPOINTS
/* more than 15 configurable endpoints */
static unsigned in_epnum;
#endif


/*
 * This should work with endpoints from controller drivers sharing the
 * same endpoint naming convention.  By example:
 *
 *	- ep1, ep2, ... address is fixed, not direction or type
 *	- ep1in, ep2out, ... address and direction are fixed, not type
 *	- ep1-bulk, ep2-bulk, ... address and type are fixed, not direction
 *	- ep1in-bulk, ep2out-iso, ... all three are fixed
 *	- ep-* ... no functionality restrictions
 *
 * Type suffixes are "-bulk", "-iso", or "-int".  Numbers are decimal.
 * Less common restrictions are implied by gadget_is_*().
 *
 * NOTE:  each endpoint is unidirectional, as specified by its USB
 * descriptor; and isn't specific to a configuration or altsetting.
 */
static int _fc_nwm_ep_matches(
	struct usb_gadget		*gadget,
	struct usb_ep			*ep,
	struct usb_endpoint_descriptor	*desc
)
{
	u8		type;
	const char	*tmp;
	u16		max;

	/* endpoint already claimed? */
	if (0 != ep->driver_data)
		return 0;

	/* only support ep0 for portable CONTROL traffic */
	type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	if (USB_ENDPOINT_XFER_CONTROL == type)
		return 0;

	/* some other naming convention */
	if ('e' != ep->name[0])
		return 0;

	/* type-restriction:  "-iso", "-bulk", or "-int".
	 * direction-restriction:  "in", "out".
	 */
	if ('-' != ep->name[2]) {
		tmp = strrchr(ep->name, '-');
		if (tmp) {
			switch (type) {
			case USB_ENDPOINT_XFER_INT:
				/* bulk endpoints handle interrupt transfers,
				 * except the toggle-quirky iso-synch kind
				 */
				if ('s' == tmp[2])	/* == "-iso" */
					return 0;
				/* for now, avoid PXA "interrupt-in";
				 * it's documented as never using DATA1.
				 */
				if (gadget_is_pxa(gadget)
						&& 'i' == tmp[1])
					return 0;
				break;
			case USB_ENDPOINT_XFER_BULK:
				if ('b' != tmp[1])	/*!= "-bulk" */
					return 0;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				if ('s' != tmp[2])	/* != "-iso" */
					return 0;
			}
		} else {
			tmp = ep->name + strlen(ep->name);
		}

		/* direction-restriction:  "..in-..", "out-.." */
		tmp--;
		if (!isdigit(*tmp)) {
			if (desc->bEndpointAddress & USB_DIR_IN) {
				if ('n' != *tmp)
					return 0;
			} else {
				if ('t' != *tmp)
					return 0;
			}
		}
	}

	/* endpoint maxpacket size is an input parameter, except for bulk
	 * where it's an output parameter representing the full speed limit.
	 * the usb spec fixes high speed bulk maxpacket at 512 bytes.
	 */
	max = 0x7ff & le16_to_cpup(&desc->wMaxPacketSize);
	switch (type) {
	case USB_ENDPOINT_XFER_INT:
		/* INT:  limit 64 bytes full speed, 1024 high speed */
		if (!gadget->is_dualspeed && max > 64)
			return 0;
		/* FALLTHROUGH */

	case USB_ENDPOINT_XFER_ISOC:
		/* ISO:  limit 1023 bytes full speed, 1024 high speed */
		if (ep->maxpacket < max)
			return 0;
		if (!gadget->is_dualspeed && max > 1023)
			return 0;

		/* BOTH:  "high bandwidth" works only at high speed */
		if ((desc->wMaxPacketSize & __constant_cpu_to_le16(3<<11))) {
			if (!gadget->is_dualspeed)
				return 0;
			/* configure your hardware with enough buffering!! */
		}
		break;
	}

	/* MATCH!! */

	/* report address */
	if (isdigit(ep->name[2])) {
		int	res;
		unsigned long	num;
		char	tempbuf[2];

		tempbuf[0] = ep->name[2];
		tempbuf[1] = '\0';
		res = strict_strtoul(tempbuf, 16, &num);
		/* printk("num = %ld(%d)\n", num, res); */
		desc->bEndpointAddress |= (u8)(num & 0xFF);
#ifdef	MANY_ENDPOINTS
	} else if (desc->bEndpointAddress & USB_DIR_IN) {
		if (++in_epnum > 15)
			return 0;
		desc->bEndpointAddress = USB_DIR_IN | in_epnum;
#endif
	} else {
		if (++epnum > 15)
			return 0;
		desc->bEndpointAddress |= epnum;
	}

	/* report (variable) full speed bulk maxpacket */
	if (USB_ENDPOINT_XFER_BULK == type) {
		int size = ep->maxpacket;

		/* min() doesn't work on bitfields with gcc-3.5 */
		if (size > 64)
			size = 64;
		desc->wMaxPacketSize = cpu_to_le16(size);
	}
	return 1;
}

/**
 * Find of Endpoint number
 */
static struct usb_ep *_fc_nwm_find_ep(
	struct usb_gadget *gadget,
	const char *name
)
{
	struct usb_ep	*ep;

	list_for_each_entry(ep, &gadget->ep_list, ep_list) {
		if (0 == strcmp(ep->name, name))
			return ep;
	}
	return NULL;
}

/**
 * Allocation of Endpoint number
 */
static struct usb_ep *_fc_nwm_usb_ep_autoconfig(
	struct usb_gadget		*gadget,
	struct usb_endpoint_descriptor	*desc
)
{
	struct usb_ep	*ep = NULL;
	u8		type;

	type = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	/* Scheme to fix Endpoint number */
		/* EP1:Bulk IN */
		/* EP2:Bulk OUT */
		/* EP3:Interrupt IN */
	if (gadget_is_emxx(gadget)) {
		if (USB_ENDPOINT_XFER_BULK == type) {
			if (USB_DIR_IN & desc->bEndpointAddress)
				ep = _fc_nwm_find_ep(gadget, "ep1in");
			else
				ep = _fc_nwm_find_ep(gadget, "ep2out");
		} else if (USB_ENDPOINT_XFER_INT == type) {
			ep = _fc_nwm_find_ep(gadget, "ep3in");
		}

		if (ep != NULL) {
			if (ep && _fc_nwm_ep_matches(gadget, ep, desc))
				return ep;
		}
	}

	/* Second, look at endpoints until an unclaimed one looks usable */
	list_for_each_entry(ep, &gadget->ep_list, ep_list) {
		if (_fc_nwm_ep_matches(gadget, ep, desc))
			return ep;
	}

	return NULL;	/* Fail */
}

/**
 * usb_ep_autoconfig_reset - reset endpoint autoconfig state
 * @gadget: device for which autoconfig state will be reset
 *
 * Use this for devices where one configuration may need to assign
 * endpoint resources very differently from the next one.  It clears
 * state such as ep->driver_data and the record of assigned endpoints
 * used by usb_ep_autoconfig().
 */
static void _fc_nwm_usb_ep_autoconfig_reset(struct usb_gadget *gadget)
{
	struct usb_ep	*ep;

	list_for_each_entry(ep, &gadget->ep_list, ep_list) {
		ep->driver_data = NULL;
	}
#ifdef	MANY_ENDPOINTS
	in_epnum = 0;
#endif
	epnum = 0;
}
/*---------------------------------------------------------------------------*/



/**
 * MTP/MSC Select
 */
static int _fc_nwm_select_class_driver(u8 *p_buf, u32 len)
{
	u32		*pbuff;
	u16		cmd_id;
	int		nret = USB_MSC_TYPE;

	if (len == 0) {
		WARN_MSG(" %s() Length Warning", __func__);
		return -1;
	}

	switch (mod_data.mode) {
	case FC_NWM_MSC_MTP_MODE:
		pbuff = (u32 *)p_buf;
		cmd_id = *(u16 *)(pbuff + 1);

		if ((*pbuff <= MTP_COMMAND_HEADER)
			&& (cmd_id == MTP_COMMAND_ID)) {
			nret = USB_MTP_TYPE;
		} else {
			nret = USB_MSC_TYPE;
		}
		break;

	case FC_NWM_MSC_ONLY_MODE:
		nret = USB_MSC_TYPE;
		break;

	case FC_NWM_MTP_ONLY_MODE:
		nret = USB_MTP_TYPE;
		break;

	default:
		ERR_MSG(" %s() mode error", __func__);
		nret = -1;
		break;
	}

	return nret;
}

/**
 * Configuration Descriptor
 */
static int _fc_nwm_populate_config_buf(struct usb_gadget *gadget,
		u8 *buf, u8 type, unsigned index)
{
#ifdef CONFIG_USB_GADGET_DUALSPEED
	enum usb_device_speed speed = gadget->speed;
#endif
	int		len;
	const struct usb_descriptor_header **function;

	if ((index != 0) && (index != CONFIG_VALUE))
		return -EINVAL;

#ifdef CONFIG_USB_GADGET_DUALSPEED
	if (type == USB_DT_OTHER_SPEED_CONFIG)
		speed = (USB_SPEED_FULL + USB_SPEED_HIGH) - speed;
	if (speed == USB_SPEED_HIGH)
		function = hs_function;
	else
#endif
		function = fs_function;

	/* for now, don't advertise srp-only devices */
	if (!gadget->is_otg)
		function++;

	len = usb_gadget_config_buf(
		&config_desc, buf, FC_NWM_EP0_BUF_SIZE, function);
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
}

/**
 * Callback of Bulk OUT
 */
static void _fc_nwm_bulk_out_complete(
	struct usb_ep *ep,
	struct usb_request *req
)
{
	PT_NWM_WORK p_work = (PT_NWM_WORK)ep->driver_data;

	DBG_MSGG("----- %s() Start", __func__);
	DBG_MSGG("----- %s() length = %d", __func__, req->actual);

	p_work->bBulk_out_wait = FALSE;

	if (req->status == -ECONNRESET)
		usb_ep_fifo_flush(ep);

	INFO_MSG("----- %s() status=%d, length=%d",
		 __func__, req->status, req->actual);
	if ((req->status != 0) || (req->actual == 0)) {
		DBG_MSGG("----- %s() status = %d, length = %d",
			__func__, req->status, req->actual);
	} else {
		spin_lock(&p_work->lock);
		p_work->nwm_event |= FC_NWM_EVENT_BULK_OUT_COMPLETE;
		spin_unlock(&p_work->lock);
	}

	/* If Endpoint is effective, the thread is caused */
	if (p_work->bBulk_out_enable)
		_fc_nwm_wakeup_thread(p_work);

	DBG_MSGG("----- %s() End", __func__);
}

/**
 * Callback of Control Transfer
 */
static void _fc_nwm_ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	PT_USB_CLASS_DRIVER p_class = NULL;

	switch (p_work->act_gadget) {
	case USB_MSC_TYPE:
		p_class = p_work->pMSC_Context;
		if (p_class && p_class->ep0_complete)
				p_class->ep0_complete(ep, req);
		break;

	case USB_MTP_TYPE:
		p_class = p_work->pMTP_Context;
		if (p_class && p_class->ep0_complete)
				p_class->ep0_complete(ep, req);
		break;

	default:
		p_class = p_work->pMSC_Context;
		if (p_class && p_class->ep0_complete)
				p_class->ep0_complete(ep, req);

		p_class = p_work->pMTP_Context;
		if (p_class && p_class->ep0_complete)
				p_class->ep0_complete(ep, req);
		break;
	}
}

/**
 * Control transferring queuing
 */
static int _fc_nwm_ep0_queue(PT_NWM_WORK p_work)
{
	int		nret;

	nret = usb_ep_queue(p_work->ep0, p_work->ep0req, GFP_ATOMIC);

	return nret;
}

/**
 * Status Stage processing of Endpoint 0
 *   Callback function called from Class Driver
 */
static void _fc_nwm_ep0_send_status_stage()
{
	PT_NWM_WORK p_work = gp_nwm_work;

	DBG_MSGG("----- %s() Start", __func__);

	if (p_work->bStatusStageFlag != FALSE) {
		INFO_MSG("----- Call EP0 Status Stage");
		p_work->ep0req->length = 0;
		p_work->ep0req->zero = 0;

		_fc_nwm_ep0_queue(p_work);
		p_work->bStatusStageFlag = FALSE;
	}

	DBG_MSGG("----- %s() End", __func__);
}

/**
 * Endpoint 0 STALL
 */
static void _fc_nwm_ep0_stall(PT_NWM_WORK p_work)
{
	usb_ep_set_halt(p_work->ep0);
}

/**
 * OS Descriptor processing
 */
static int _fc_nwm_get_os_descriptor(PT_NWM_WORK p_work)
{
	int		nret = -1;
	struct usb_request *req;

	switch (mod_data.mode) {
	case FC_NWM_MSC_MTP_MODE:
	case FC_NWM_MTP_ONLY_MODE:
		req = p_work->ep0req;
		memcpy(req->buf, os_feature_desc,
			FC_NWM_MS_DESC_LENGTH);
		nret = FC_NWM_MS_DESC_LENGTH;
		break;

	default:
		break;
	}

	DBG_MSGG("     %s() length = %d", __func__, nret);

	return nret;
}

/**
 * SET_CONFIGURATION
 */
static int _fc_nwm_set_configuration(PT_NWM_WORK p_work, u8 config_value)
{
	int		nret = -EOPNOTSUPP;

	INFO_MSG("----- %s(%d) Start", __func__, config_value);
	INFO_MSGG("----- %s(%d) Enter", __func__, config_value);

	switch (config_value) {
	case 0:
		DBG_MSGG("Reset Configuration");
		p_work->config_num = config_value;
		nret = _fc_nwm_set_interface(p_work, -1);
		p_work->nwm_state = FC_NWM_STATE_WAIT_CONFIG;
		break;

	case CONFIG_VALUE:
		p_work->config_num = config_value;

		nret = _fc_nwm_set_interface(p_work, FC_NWM_ALT_VALUE);
		if (nret != 0) {
			p_work->config_num = 0;
			p_work->nwm_state = FC_NWM_STATE_WAIT_CONFIG;
		} else {
			p_work->nwm_state = FC_NWM_STATE_WAIT_BULK;
		}

	default:
		break;
	}

	INFO_MSG("----- %s(%d) End", __func__, config_value);

	return nret;
}

/**
 * SET_INTERFACE
 */
static int _fc_nwm_set_interface(PT_NWM_WORK p_work, int alt_value)
{
	int		nret = 0;

	DBG_MSGG("----- %s() Start", __func__);

	INFO_MSG("_fc_nwm_free_request()");

	/* Free Request */
	_fc_nwm_free_request(p_work);

	INFO_MSG("_fc_nwm_disable_endpoint()");

	/* Disable Endpoint */
	_fc_nwm_disable_endpoint(p_work);

	p_work->bRunning_flag = FALSE;

	if (alt_value < 0)
		return nret;

	INFO_MSG("_fc_nwm_enable_endpoint()");

	/* Enable Endpoint */
	nret = _fc_nwm_enable_endpoint(p_work);
	if (nret != 0) {
		DBG_MSG("_fc_nwm_enable_endpoint() Failed");
		_fc_nwm_disable_endpoint(p_work);
		return nret;
	}

	INFO_MSG("_fc_nwm_alloc_request()");

	/* Alloc Request */
	nret = _fc_nwm_alloc_request(p_work);
	if (nret != 0) {
		DBG_MSG("_fc_nwm_alloc_request() Failed");
		_fc_nwm_disable_endpoint(p_work);
		return nret;
	}

	p_work->bRunning_flag = TRUE;

	DBG_MSGG("----- %s() End", __func__);

	return nret;
}

/**
 * Enable Endpoint
 */
static int _fc_nwm_enable_endpoint(PT_NWM_WORK p_work)
{
	struct usb_ep *ep;
	const struct usb_endpoint_descriptor *p_desc;
	int		nret = 0;

	p_desc = ep_desc(p_work->gadget, &fs_bulk_in_desc, &hs_bulk_in_desc);
	ep = p_work->bulk_in;
	ep->driver_data = p_work;
	nret = usb_ep_enable(ep, p_desc);
	if (nret != 0) {
		ERR_MSG("usb_ep_enable(bulk_in) return %d", nret);
		return nret;
	}
	p_work->bBulk_in_enable = TRUE;

	p_desc = ep_desc(p_work->gadget, &fs_bulk_out_desc, &hs_bulk_out_desc);
	ep = p_work->bulk_out;
	ep->driver_data = p_work;
	nret = usb_ep_enable(ep, p_desc);
	if (nret != 0) {
		ERR_MSG("usb_ep_enable(bulk_out) return %d", nret);
		return nret;
	}
	p_work->bBulk_out_enable = TRUE;

	p_desc = ep_desc(p_work->gadget, &fs_intr_in_desc, &hs_intr_in_desc);
	ep = p_work->intr_in;
	ep->driver_data = p_work;
	nret = usb_ep_enable(ep, p_desc);
	if (nret != 0) {
		ERR_MSG("usb_ep_enable(intr_in) return %d", nret);
		return nret;
	}
	p_work->bIntr_in_enable = TRUE;

	return nret;
}

/**
 * Disable Endpoint
 */
static int _fc_nwm_disable_endpoint(PT_NWM_WORK p_work)
{
	int		nret = 0;

	INFO_MSG("----- %s() Start", __func__);

	if (p_work->bBulk_in_enable != FALSE) {
		p_work->bBulk_in_enable = FALSE;
		usb_ep_disable(p_work->bulk_in);
	}

	if (p_work->bBulk_out_enable != FALSE) {
		p_work->bBulk_out_enable = FALSE;
		usb_ep_disable(p_work->bulk_out);
	}

	if (p_work->bIntr_in_enable != FALSE) {
		p_work->bIntr_in_enable = FALSE;
		usb_ep_disable(p_work->intr_in);
	}

	INFO_MSG("----- %s() End", __func__);

	return nret;
}

/**
 * Alloc Request
 */
static int _fc_nwm_alloc_request(PT_NWM_WORK p_work)
{
	int		nret = 0;
	struct usb_ep *ep;
	struct usb_request *p_req;

	INFO_MSGG("----- %s() Enter", __func__);

	ep = p_work->bulk_in;
	p_req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (p_req == NULL) {
		ERR_MSG("usb_ep_alloc_request()");
		return -ENOMEM;
	}
	p_work->bulk_in_req = p_req;

	ep = p_work->bulk_out;
	p_req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (p_req == NULL) {
		ERR_MSG("usb_ep_alloc_request()");
		return -ENOMEM;
	}
	p_req->complete = _fc_nwm_bulk_out_complete;
	p_work->bulk_out_req = p_req;

	if (p_work->p_bulk_buf == NULL) {
		p_work->p_bulk_buf = kmalloc(
				FC_NWM_BULKOUT_MAXPACKETSIZE,
				GFP_KERNEL | GFP_DMA);

		if (p_work->p_bulk_buf == NULL) {
			ERR_MSG("usb_ep_alloc_buffer()");
			return -ENOMEM;
		}
	}

	ep = p_work->intr_in;
	p_req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (p_req == NULL) {
		ERR_MSG("usb_ep_alloc_request()");
		return -ENOMEM;
	}
	p_work->intr_in_req = p_req;

	return nret;
}

/**
 * Free Request
 */
static int _fc_nwm_free_request(PT_NWM_WORK p_work)
{
	int		nret = 0;
	struct usb_ep *ep;
	struct usb_request *p_req;

	INFO_MSGG("----- %s() Enter", __func__);

	p_req = p_work->intr_in_req;
	if (p_req != NULL) {
		ep = p_work->intr_in;
		usb_ep_free_request(ep, p_req);
		p_work->intr_in_req = NULL;
	}

	p_req = p_work->bulk_out_req;
	if (p_req != NULL) {
		ep = p_work->bulk_out;

		if (p_work->p_bulk_buf != NULL) {
			kfree(p_work->p_bulk_buf);
			p_work->p_bulk_buf = NULL;
		}

		usb_ep_free_request(ep, p_req);
		p_work->bulk_out_req = NULL;
	}

	p_req = p_work->bulk_in_req;
	if (p_req != NULL) {
		ep = p_work->bulk_in;
		usb_ep_free_request(ep, p_req);
		p_work->bulk_in_req = NULL;
	}

	return nret;
}

/**
 * Callback function of SETUP (USB Request)
 */
static void _fc_nwm_callback_setup(
	PT_NWM_WORK p_work,
	const struct usb_ctrlrequest *ctrl
)
{
	PT_USB_CLASS_DRIVER p_class = NULL;

	p_class = p_work->pMSC_Context;
	if (p_class) {
		p_class->gadget->speed = p_work->gadget->speed;
		p_class->pDriver->setup(p_class->gadget, ctrl);
	}

	p_class = p_work->pMTP_Context;
	if (p_class) {
		p_class->gadget->speed = p_work->gadget->speed;
		p_class->pDriver->setup(p_class->gadget, ctrl);
	}
}

/**
 * Standard request
 */
static int _fc_nwm_standard_setup(
	PT_NWM_WORK p_work,
	const struct usb_ctrlrequest *ctrl
)
{
	struct usb_request *req = p_work->ep0req;
	int		value = -EOPNOTSUPP;
	int		nret;
	u16		w_value = le16_to_cpu(ctrl->wValue);
	u16		w_index = le16_to_cpu(ctrl->wIndex);

	switch (ctrl->bRequest) {
	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_GET_STANDARD_REQ)
			break;

		switch (w_value >> 8) {
		case USB_DT_DEVICE:
			DBG_MSGG("GET_DESCRIPTOR:Device");
			value = sizeof device_desc;
			memcpy(req->buf, &device_desc, value);
			_fc_nwm_callback_setup(p_work, ctrl);
			break;

#ifdef CONFIG_USB_GADGET_DUALSPEED
		case USB_DT_DEVICE_QUALIFIER:
			DBG_MSGG("GET_DESCRIPTOR:Device Qualifier");
			if (!p_work->gadget->is_dualspeed)
				break;
			value = sizeof dev_qualifier;
			memcpy(req->buf, &dev_qualifier, value);
			break;

		case USB_DT_OTHER_SPEED_CONFIG:
			DBG_MSGG("GET_DESCRIPTOR:Other Speed Configuration");
			if (!p_work->gadget->is_dualspeed)
				break;
			goto get_config;

#endif
		case USB_DT_CONFIG:
			DBG_MSGG("GET_DESCRIPTOR:Configuration");
#ifdef CONFIG_USB_GADGET_DUALSPEED
get_config:
#endif
			value = _fc_nwm_populate_config_buf(
				p_work->gadget,
				req->buf, w_value >> 8,
				w_value & 0xff);
			break;

		case USB_DT_STRING:
			DBG_MSGG("GET_DESCRIPTOR:String");
			value = usb_gadget_get_string(
				&stringtab,
				w_value & 0xff,
				req->buf);
			break;
		}
		break;

	case USB_REQ_SET_CONFIGURATION:
		DBG_MSGG("SET_CONFIGURATION(%d)", w_value);
		if (ctrl->bRequestType != USB_SET_STANDARD_REQ)
			break;

		if ((p_work->nwm_state == FC_NWM_STATE_CLASS_LOOP) &&
				(p_work->act_gadget != USB_START_TYPE)) {
			PT_USB_CLASS_DRIVER p_class = NULL;

			if (p_work->act_gadget == USB_MSC_TYPE)
					p_class = p_work->pMSC_Context;
			else if (p_work->act_gadget == USB_MTP_TYPE)
					p_class = p_work->pMTP_Context;

			if (p_class != NULL) {
				p_class->gadget->speed = p_work->gadget->speed;
				value = p_class->pDriver->setup(
					p_class->gadget, ctrl);
				if (value == 0) {
					value = _fc_nwm_set_configuration(
						p_work, w_value);
				} else if (value > 0) {
					p_work->rsv_req =
						FC_NWM_RSV_REQ_SET_CONFIG;
					p_work->rsv_req_val = w_value;
				}
			}
		} else {
			if (p_work->nwm_state == FC_NWM_STATE_WAIT_BULK) {
				nret = usb_ep_dequeue(
					p_work->bulk_out,
					p_work->bulk_out_req);
				if (nret == 0)
					wait_for_completion(&p_work->thread_wait_comp);
			}

			value = _fc_nwm_set_configuration(p_work, w_value);
			if (value >= 0)
				value = DELAYED_STATUS;

			_fc_nwm_callback_setup(p_work, ctrl);
		}

		if (p_work->act_gadget == USB_START_TYPE)
			_fc_nwm_wakeup_thread(p_work);

		break;

	case USB_REQ_GET_CONFIGURATION:
		DBG_MSGG("GET_CONFIGURATION");
		if (ctrl->bRequestType != USB_GET_STANDARD_REQ)
			break;

		*(u8 *) req->buf = p_work->config_num;
		value = 1;
		break;

	case USB_REQ_SET_INTERFACE:
		DBG_MSG("SET_INTERFACE(%d)", w_index);
		if (ctrl->bRequestType != USB_SET_STANDARD_REQ_IF)
			break;

		if (p_work->config_num == 0)
			break;

		if (w_index != 0) {
			value = -EDOM;
			break;
		}

		if ((p_work->nwm_state  == FC_NWM_STATE_CLASS_LOOP) &&
			(p_work->act_gadget != USB_START_TYPE)) {

			PT_USB_CLASS_DRIVER p_class = NULL;

			if (p_work->act_gadget == USB_MSC_TYPE)
					p_class = p_work->pMSC_Context;
			else if (p_work->act_gadget == USB_MTP_TYPE)
					p_class = p_work->pMTP_Context;

			if (p_class != NULL) {
				value = p_class->pDriver->setup(
					p_class->gadget, ctrl);
				if (value == 0) {
					value = _fc_nwm_set_interface(
						p_work, w_index);
				} else if (value > 0) {
					p_work->rsv_req =
						FC_NWM_RSV_REQ_SET_INTERFACE;
					p_work->rsv_req_val = w_index;
				}
			}
		} else {
			if (p_work->nwm_state == FC_NWM_STATE_WAIT_BULK) {
				nret = usb_ep_dequeue(
					p_work->bulk_out,
					p_work->bulk_out_req);
				if (nret == 0)
					wait_for_completion(
						&p_work->thread_wait_comp);
			}

			value = _fc_nwm_set_interface(p_work, w_index);
			if (value >= 0)
				value = DELAYED_STATUS;

			_fc_nwm_callback_setup(p_work, ctrl);
		}

		if (p_work->act_gadget == USB_START_TYPE)
			_fc_nwm_wakeup_thread(p_work);

		break;

	case USB_REQ_GET_INTERFACE:
		DBG_MSGG("GET_INTERFACE");
		if (ctrl->bRequestType != USB_GET_STANDARD_REQ_IF)
			break;

		if (p_work->config_num == 0)
			break;

		if (w_index != 0) {
			value = -EDOM;
			break;
		}

		*(u8 *) req->buf = FC_NWM_ALT_VALUE;
		value = 1;
		break;

	default:
		break;
	}

	return value;
}

/**
 * Class request
 */
static int _fc_nwm_class_setup(
	PT_NWM_WORK p_work,
	const struct usb_ctrlrequest *ctrl
)
{
	int		value = -EOPNOTSUPP;
	PT_USB_CLASS_DRIVER p_class = NULL;

	switch (p_work->act_gadget) {
	case USB_MSC_TYPE:
		p_class = p_work->pMSC_Context;
		if (p_class) {
			value = p_class->pDriver->setup(
				p_class->gadget, ctrl);
		}
		break;

	case USB_MTP_TYPE:
		p_class = p_work->pMTP_Context;
		if (p_class) {
			value = p_class->pDriver->setup(
				p_class->gadget, ctrl);
		}
		break;

	default:
		p_class = p_work->pMSC_Context;
		if (p_class) {
			value = p_class->pDriver->setup(
				p_class->gadget, ctrl);
		}

		if (value == -EOPNOTSUPP) {
			p_class = p_work->pMTP_Context;
			if (p_class) {
				value = p_class->pDriver->setup(
					p_class->gadget, ctrl);
			}
		}
		break;
	}

	return value;
}

/**
 * Vendor request
 */
static int _fc_nwm_vendor_setup(
	PT_NWM_WORK p_work,
	const struct usb_ctrlrequest *ctrl
)
{
	int		value = -EOPNOTSUPP;
	u16		w_value = le16_to_cpu(ctrl->wValue);
	PT_USB_CLASS_DRIVER p_class = NULL;

	switch (ctrl->bRequest) {
	case FC_NWM_OSD_VENDOR_CODE:
		DBG_MSGG("GET_OS_DESCRIPTOR");
		if (ctrl->bRequestType != USB_GET_VENDOR_REQ)
			break;

		if (w_value != FC_NWM_ALT_VALUE)
			break;

		value = _fc_nwm_get_os_descriptor(p_work);
		break;

	default:
		break;
	}

	if (value != -EOPNOTSUPP)
		return value;

	switch (p_work->act_gadget) {
	case USB_MSC_TYPE:
		p_class = p_work->pMSC_Context;
		if (p_class) {
			value = p_class->pDriver->setup(
				p_class->gadget, ctrl);
		}
		break;

	case USB_MTP_TYPE:
		p_class = p_work->pMTP_Context;
		if (p_class) {
			value = p_class->pDriver->setup(
				p_class->gadget, ctrl);
		}
		break;

	default:
		p_class = p_work->pMSC_Context;
		if (p_class) {
			value = p_class->pDriver->setup(
				p_class->gadget, ctrl);
		}

		if (value == -EOPNOTSUPP) {
			p_class = p_work->pMTP_Context;
			if (p_class) {
				value = p_class->pDriver->setup(
					p_class->gadget, ctrl);
			}
		}
		break;
	}

	return value;
}

/**
 * Class bind()
 */
static int _fc_nwm_callback_bind(
	PT_NWM_WORK p_work,
	PT_USB_CLASS_DRIVER p_class,
	struct usb_gadget *gadget
)
{
	int		nret = -1;
	struct usb_gadget *pgadget;

	if (p_class == NULL)
		return nret;

	/* Acquisition of Gadget memory */
	pgadget = kmalloc(sizeof(struct usb_gadget), GFP_KERNEL);
	if (pgadget == NULL)
		return nret;

	memcpy(pgadget, gadget, sizeof(struct usb_gadget));
	p_class->gadget = pgadget;
	p_class->org_gadget = gadget;
	p_class->bulk_in = p_work->bulk_in;
	p_class->bulk_out = p_work->bulk_out;
	p_class->intr_in = p_work->intr_in;
	p_class->ep0req = p_work->ep0req;
	p_class->ep0_send_status_stage = _fc_nwm_ep0_send_status_stage;

	nret = p_class->pDriver->bind(pgadget);

	return nret;
}

/**
 * Class unbind()
 */
static void _fc_nwm_callback_unbind(PT_USB_CLASS_DRIVER p_class)
{
	struct usb_gadget *pgadget;

	pgadget = p_class->gadget;
	p_class->pDriver->unbind(pgadget);
	kfree(pgadget);
}

/**
 * bind()
 */
static int _fc_nwm_bind(struct usb_gadget *gadget)
{
	struct usb_ep	*ep;
	struct usb_request	*req;
	struct usb_string	*p_str;
	PT_NWM_WORK p_work = gp_nwm_work;
	int		nret;
	int		i;

	DBG_MSGG("----- %s() Start", __func__);

	p_work->gadget = gadget;
	set_gadget_data(gadget, p_work);
	p_work->ep0 = gadget->ep0;
	p_work->ep0->driver_data = p_work;

	/* Endpoint reset */
	_fc_nwm_usb_ep_autoconfig_reset(gadget);

	ep = _fc_nwm_usb_ep_autoconfig(gadget, &fs_bulk_in_desc);
	if (ep == NULL)
		goto autoconf_fail;

	ep->driver_data = p_work;
	p_work->bulk_in = ep;

	ep = _fc_nwm_usb_ep_autoconfig(gadget, &fs_bulk_out_desc);
	if (ep == NULL)
		goto autoconf_fail;

	ep->driver_data = p_work;
	p_work->bulk_out = ep;

	ep = _fc_nwm_usb_ep_autoconfig(gadget, &fs_intr_in_desc);
	if (ep == NULL)
		goto autoconf_fail;

	ep->driver_data = p_work;
	p_work->intr_in = ep;

	device_desc.bMaxPacketSize0 = p_work->ep0->maxpacket;

#ifdef CONFIG_USB_GADGET_DUALSPEED

	dev_qualifier.bMaxPacketSize0 = p_work->ep0->maxpacket;

	hs_bulk_in_desc.bEndpointAddress = fs_bulk_in_desc.bEndpointAddress;
	hs_bulk_out_desc.bEndpointAddress = fs_bulk_out_desc.bEndpointAddress;
	hs_intr_in_desc.bEndpointAddress = fs_intr_in_desc.bEndpointAddress;

	DBG_MSGG("Bulk IN = 0x%02x", fs_bulk_in_desc.bEndpointAddress);
	DBG_MSGG("Bulk OUT = 0x%02x", fs_bulk_out_desc.bEndpointAddress);
	DBG_MSGG("Interrupt IN = 0x%02x", fs_intr_in_desc.bEndpointAddress);

#endif	/* CONFIG_USB_GADGET_DUALSPEED */

	nret = -ENOMEM;

	/* Acquisition of memory for Endpoint 0 */
	p_work->ep0req = req = usb_ep_alloc_request(p_work->ep0, GFP_KERNEL);
	if (req == NULL)
		goto out;

	req->buf = kmalloc(FC_NWM_EP0_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (req->buf == NULL)
		goto out;

	req->complete = _fc_nwm_ep0_complete;

	usb_gadget_set_selfpowered(gadget);

	device_desc.idVendor = cpu_to_le16(mod_data.vendor);
	device_desc.idProduct = cpu_to_le16(mod_data.product);
	device_desc.bcdDevice = cpu_to_le16(mod_data.release);

	switch (mod_data.mode) {
	case FC_NWM_MSC_ONLY_MODE:
		p_str = strings;

		for (i = 0; (1); i++) {
			if (p_str->id == 0)
				break;

			if (p_str->id == FC_NWM_STRING_OS_DESCRIPTOR) {
				p_str->id = 0;
				break;
			}

			p_str++;
		}
		break;

	case FC_NWM_MTP_ONLY_MODE:

#ifdef USE_ALT_INTERFACE
		intf_desc_0.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
		intf_desc_1.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
#else	/* !USE_ALT_INTERFACE */
		intf_desc.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
#endif	/* USE_ALT_INTERFACE */
			break;

	default:
		break;
	}

	for (i = 0; i < sizeof(serial) - 2; i += 2) {
		unsigned char	c = DRIVER_VERSION[i / 2];

		if (!c)
			break;
		sprintf(&serial[i], "%02X", c);
	}

	switch (mod_data.mode) {
	case FC_NWM_MSC_MTP_MODE:
		nret = _fc_nwm_callback_bind(
			p_work, p_work->pMSC_Context, gadget);
		if (nret != 0)
			break;

		nret = _fc_nwm_callback_bind(
			p_work, p_work->pMTP_Context, gadget);
		break;

	case FC_NWM_MSC_ONLY_MODE:
		nret = _fc_nwm_callback_bind(
			p_work, p_work->pMSC_Context, gadget);
		break;

	case FC_NWM_MTP_ONLY_MODE:
		nret = _fc_nwm_callback_bind(
			p_work, p_work->pMTP_Context, gadget);
		break;

	default:
		nret = 0;
		break;
	}

	if (nret != 0)
		goto out;

	DBG_MSGG("----- %s() End", __func__);

	return 0;

autoconf_fail:
	nret = -ENOTSUPP;

out:
	_fc_nwm_unbind(gadget);

	return nret;
}

/**
 * unbind()
 */
static void _fc_nwm_unbind(struct usb_gadget *gadget)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	struct usb_request	*req = p_work->ep0req;

	switch (mod_data.mode) {
	case FC_NWM_MSC_MTP_MODE:
		_fc_nwm_callback_unbind(p_work->pMTP_Context);
		_fc_nwm_callback_unbind(p_work->pMSC_Context);
		break;

	case FC_NWM_MSC_ONLY_MODE:
		_fc_nwm_callback_unbind(p_work->pMSC_Context);
		break;

	case FC_NWM_MTP_ONLY_MODE:
		_fc_nwm_callback_unbind(p_work->pMTP_Context);
		break;

	default:
		break;
	}

	/* Opening of memory for Endpoint 0 */
	if (req) {
		if (req->buf)
			kfree(req->buf);

		usb_ep_free_request(p_work->ep0, req);
	}

	set_gadget_data(gadget, NULL);
}

/**
 * Request Decode
 */
static int _fc_nwm_setup(
	struct usb_gadget *gadget,
	const struct usb_ctrlrequest *ctrl
)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	int		nret;
	int		w_length = le16_to_cpu(ctrl->wLength);

	DBG_MSGG("----- %s() Start", __func__);

	p_work->ep0_req_tag++;
	p_work->ep0req->context = NULL;
	p_work->ep0req->length = 0;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
		nret = _fc_nwm_standard_setup(p_work, ctrl);
	else if (ctrl->bRequestType & USB_TYPE_CLASS)
		nret = _fc_nwm_class_setup(p_work, ctrl);
	else
		nret = _fc_nwm_vendor_setup(p_work, ctrl);

	if ((nret >= 0) && (nret <= FC_NWM_EP0_BUF_SIZE)) {
		nret = min(nret, w_length);
		p_work->ep0req->length = nret;
		p_work->ep0req->zero = nret < w_length;
		nret = _fc_nwm_ep0_queue(p_work);
	} else {
		p_work->bStatusStageFlag = TRUE;
	}

	DBG_MSGG("----- %s() End", __func__);

	return nret;
}

/**
 * Disconnect
 */
static void _fc_nwm_disconnect(struct usb_gadget *gadget)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	PT_USB_CLASS_DRIVER p_class = NULL;

	INFO_MSG("----- %s() Start", __func__);

	/* NWM Driver event setting */
	spin_lock(&p_work->lock);
	p_work->nwm_event |= FC_NWM_EVENT_DISCONNECT;
	spin_unlock(&p_work->lock);

	INFO_MSG(" %s spin_unlock", __func__);

	if (p_work->nwm_state != FC_NWM_STATE_CLASS_LOOP) {
		_fc_nwm_wakeup_thread(p_work);

		p_class = p_work->pMSC_Context;
		if (p_class != NULL)
			p_class->pDriver->disconnect(p_class->gadget);

		p_class = p_work->pMTP_Context;
		if (p_class != NULL)
			p_class->pDriver->disconnect(p_class->gadget);
	} else {
		if (p_work->act_gadget == USB_MSC_TYPE)
			p_class = p_work->pMSC_Context;
		else if (p_work->act_gadget == USB_MTP_TYPE)
			p_class = p_work->pMTP_Context;

		if (p_class != NULL)
			p_class->pDriver->disconnect(p_class->gadget);
	}

	INFO_MSG("----- %s() End", __func__);
}

/**
 * USB Suspend
 */
static void _fc_nwm_suspend(struct usb_gadget *gadget)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	PT_USB_CLASS_DRIVER p_class = NULL;

	INFO_MSG("----- %s() Start", __func__);

	/* NWM Driver event setting */
	spin_lock(&p_work->lock);
	if (p_work->nwm_state == FC_NWM_STATE_CLASS_LOOP)
		p_work->nwm_event |= FC_NWM_EVENT_SUSPEND;
	spin_unlock(&p_work->lock);

#ifdef USE_CLASS_CALLBACK
	p_class = p_work->pMSC_Context;
	if (p_class)
		p_class->pDriver->suspend(p_class->gadget);

	p_class = p_work->pMTP_Context;
	if (p_class)
		p_class->pDriver->suspend(p_class->gadget);
#else	/* !USE_CLASS_CALLBACK */
	if (p_work->nwm_state == FC_NWM_STATE_CLASS_LOOP) {
		if (p_work->act_gadget == USB_MSC_TYPE)
			p_class = p_work->pMSC_Context;
		else if (p_work->act_gadget == USB_MTP_TYPE)
			p_class = p_work->pMTP_Context;
	}

	if (p_class != NULL)
		p_class->pDriver->suspend(p_class->gadget);
#endif	/* USE_CLASS_CALLBACK */

	INFO_MSG("----- %s() End", __func__);
}

/**
 * USB Resume
 */
static void _fc_nwm_resume(struct usb_gadget *gadget)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	PT_USB_CLASS_DRIVER p_class = NULL;

	DBG_MSGG("----- %s() Start", __func__);
	DBG_MSGG("----- %s() Enter", __func__);

	/* NWM Driver event setting */
	spin_lock(&p_work->lock);
	p_work->nwm_event |= FC_NWM_EVENT_RESUME;
	spin_unlock(&p_work->lock);

#ifdef USE_CLASS_CALLBACK
	p_class = p_work->pMSC_Context;
	if (p_class)
		p_class->pDriver->resume(p_class->gadget);

	p_class = p_work->pMTP_Context;
	if (p_class)
		p_class->pDriver->resume(p_class->gadget);
#else	/* !USE_CLASS_CALLBACK */
	if (p_work->nwm_state == FC_NWM_STATE_CLASS_LOOP) {
		if (p_work->act_gadget == USB_MSC_TYPE)
			p_class = p_work->pMSC_Context;
		else if (p_work->act_gadget == USB_MTP_TYPE)
			p_class = p_work->pMTP_Context;
	}

	if (p_class != NULL)
		p_class->pDriver->resume(p_class->gadget);
#endif	/* USE_CLASS_CALLBACK */

	DBG_MSGG("----- %s() End", __func__);
}

/**
 * Acquisition of work area
 */
static int __init _fc_nwm_alloc(void)
{
	PT_NWM_WORK p_work;

	DBG_MSG("----- %s() Start", __func__);

	p_work = kmalloc(sizeof(T_NWM_WORK), GFP_KERNEL);
	if (p_work == NULL)
		return -ENOMEM;

	DBG_MSG("  alloc size = %d", sizeof(T_NWM_WORK));

	memset(p_work, 0, sizeof(T_NWM_WORK));
	p_work->nwm_state = FC_NWM_STATE_INIT;
	spin_lock_init(&p_work->lock);
	init_completion(&p_work->thread_comp);
	init_completion(&p_work->thread_wait_comp);
	init_MUTEX(&p_work->work_sem);

	gp_nwm_work = p_work;

	DBG_MSG("----- %s() End", __func__);

	return 0;
}

/**
 * Opening of work area
 */
static void _fc_nwm_free()
{
	PT_NWM_WORK p_work = gp_nwm_work;

	if (p_work != NULL)
		kfree(p_work);
}

/**
 * USB Driver is registered in Gadget
 */
static int _fc_nwm_regist_state(PT_NWM_WORK p_work)
{
	int		nret;

	DBG_MSG("----- %s() Start", __func__);

	nret = usb_gadget_register_driver(&g_nwm_driver);
	if (nret != 0) {
		ERR_MSG("usb_gadget_register_driver()");
		p_work->nwm_state = FC_NWM_STATE_WAIT_REGIST;
	} else {
		p_work->bRegist_flag = TRUE;
		p_work->nwm_state = FC_NWM_STATE_WAIT_CONFIG;
	}

	DBG_MSG("----- %s() End", __func__);

	return nret;
}

/**
 * Preparation for Bulk OUT
 */
static int _fc_nwm_wait_bulk_state(PT_NWM_WORK p_work)
{
	int		nret = 0;
	struct usb_ep *ep;
	struct usb_request *p_req;

	DBG_MSGG("----- %s() Start", __func__);

	if (p_work->bBulk_out_wait)
		return nret;

	ep = p_work->bulk_out;
	ep->driver_data = p_work;

	p_req = p_work->bulk_out_req;
	p_req->complete = _fc_nwm_bulk_out_complete;
	p_req->buf = p_work->p_bulk_buf;
	p_req->length = FC_NWM_BULKOUT_MAXPACKETSIZE;

	if (p_work->nwm_state != FC_NWM_STATE_WAIT_BULK)
		return -1;

	nret = usb_ep_queue(ep, p_req, GFP_KERNEL);

	DBG_MSGG("----- %s(%d) End", __func__, nret);

	return nret;
}

/**
 * Analysis of Bulk OUT
 */
static void _fc_nwm_recive_bulk_state(PT_NWM_WORK p_work)
{
	struct usb_request *p_req = p_work->bulk_out_req;
	int		nret;

	DBG_MSGG("----- %s() Start", __func__);

	/* Check on receive data */
	nret = _fc_nwm_select_class_driver(p_req->buf, p_req->actual);
	if (nret < 0) {
		p_work->nwm_state = FC_NWM_STATE_WAIT_BULK;
	} else {
		spin_lock_irq(&p_work->lock);
		p_work->act_gadget = nret;
		p_work->nwm_state  = FC_NWM_STATE_CLASS_LOOP;
		p_work->rsv_req    = FC_NWM_RSV_REQ_NONE;
		if (p_work->act_gadget == USB_MSC_TYPE) {
			if (p_work->pMSC_Context->pMainLoopInit) {
				p_work->pMSC_Context->pMainLoopInit(
					p_work->pMSC_Context);
			}
		}
		spin_unlock_irq(&p_work->lock);
	}

	DBG_MSGG("----- %s() End", __func__);
}

/**
 * Class Loop
 */
static void _fc_nwm_class_loop_state(PT_NWM_WORK p_work)
{
	PT_USB_CLASS_DRIVER p_class = NULL;

	INFO_MSG("----- %s() Start", __func__);

	/* Confirmation of type (MSC or MTP) */
	if (p_work->act_gadget == USB_MSC_TYPE)
		p_class = p_work->pMSC_Context;
	else if (p_work->act_gadget == USB_MTP_TYPE)
		p_class = p_work->pMTP_Context;

	if (p_class == NULL) {
		/* NWM Driver Only */
		p_work->nwm_state = FC_NWM_STATE_WAIT_BULK;
		return;
	}

	p_class->bulk_in = p_work->bulk_in;
	p_class->bulk_out = p_work->bulk_out;
	p_class->intr_in = p_work->intr_in;

	p_class->bulk_in_req = p_work->bulk_in_req;
	p_class->bulk_out_req = p_work->bulk_out_req;
	p_class->intr_in_req = p_work->intr_in_req;

	p_class->bulk_in_desc = &fs_bulk_in_desc;
	p_class->bulk_out_desc = &fs_bulk_out_desc;
	p_class->intr_in_desc = &fs_intr_in_desc;

	/* The main loop function of Class Driver is called */
	p_class->pMainLoop(p_class);
	spin_lock_irq(&p_work->lock);
	p_work->act_gadget = USB_START_TYPE;
	_fc_nwm_check_event(p_work);
	spin_unlock_irq(&p_work->lock);

	if ((p_work->nwm_state == FC_NWM_STATE_CLASS_LOOP) &&
			(p_work->rsv_req != FC_NWM_RSV_REQ_NONE)) {
		int value = -1;

		if (p_work->rsv_req == FC_NWM_RSV_REQ_SET_CONFIG) {
			value = _fc_nwm_set_configuration(
				p_work, p_work->rsv_req_val);
		} else if (p_work->rsv_req == FC_NWM_RSV_REQ_SET_INTERFACE) {
			value = _fc_nwm_set_interface(
				p_work, p_work->rsv_req_val);
		}

		if (value == 0)
			_fc_nwm_ep0_send_status_stage();
		else
			_fc_nwm_ep0_stall(p_work);
	}

	INFO_MSG("----- %s() End", __func__);
}

/**
 * Reset State
 */
static void _fc_nwm_reset_state(PT_NWM_WORK p_work)
{
	p_work->nwm_state = FC_NWM_STATE_WAIT_CONFIG;
}

/**
 * Disconnect State
 */
static void _fc_nwm_disconnect_state(PT_NWM_WORK p_work)
{
	if (p_work->config_num != 0)
		_fc_nwm_set_configuration(p_work, 0);

	p_work->bBulk_out_wait = FALSE;
	p_work->nwm_state = FC_NWM_STATE_WAIT_CONFIG;
}

/**
 * Unregist
 */
static void _fc_nwm_unregist_state(PT_NWM_WORK p_work)
{
	p_work->nwm_state = FC_NWM_STATE_WAIT_REGIST;
}

/**
 * Event check
 */
static void _fc_nwm_check_event(PT_NWM_WORK p_work)
{
	INFO_MSG("----- %s(%x) Enter", __func__, p_work->nwm_event);

	if (p_work->nwm_event & FC_NWM_EVENT_VBUS_OFF) {
		p_work->nwm_state = FC_NWM_STATE_DISCONNECT;
		p_work->nwm_event &=
			~(FC_NWM_DISCONNECT_EVENT | FC_NWM_EVENT_SUSPEND);
		return;
	}

	if (p_work->nwm_event & FC_NWM_EVENT_DISCONNECT) {
		p_work->nwm_state = FC_NWM_STATE_DISCONNECT;
		p_work->nwm_event &=
			~(FC_NWM_DISCONNECT_EVENT | FC_NWM_EVENT_SUSPEND);
		return;
	}

	if (p_work->nwm_event & FC_NWM_EVENT_RESET) {
		p_work->nwm_state = FC_NWM_STATE_DISCONNECT;
		p_work->nwm_event &=
			~(FC_NWM_DISCONNECT_EVENT | FC_NWM_EVENT_SUSPEND);
		return;
	}

	if (p_work->nwm_event & FC_NWM_EVENT_SUSPEND) {
		if (p_work->nwm_state == FC_NWM_STATE_CLASS_LOOP)
			p_work->nwm_state = FC_NWM_STATE_CLASS_LOOP;

		p_work->nwm_event &= ~FC_NWM_EVENT_SUSPEND;
		return;
	}

	if (p_work->nwm_event & FC_NWM_EVENT_RESUME) {
		p_work->nwm_event &= ~FC_NWM_EVENT_RESUME;
		return;
	}

	if (p_work->nwm_event & FC_NWM_EVENT_REGIST) {
		p_work->nwm_state = FC_NWM_STATE_REGIST_END;
		p_work->nwm_event &= ~FC_NWM_EVENT_REGIST;
		return;
	}

	if (p_work->nwm_event & FC_NWM_EVENT_BULK_OUT_COMPLETE) {
		if (p_work->nwm_state == FC_NWM_STATE_WAIT_BULK)
			p_work->nwm_state = FC_NWM_STATE_RECIVE_BULK;

		p_work->nwm_event &= ~FC_NWM_EVENT_BULK_OUT_COMPLETE;
	}
}

/**
 * Sleep Thread
 */
static void _fc_nwm_wait_thread(PT_NWM_WORK p_work)
{
	INFO_MSG("----- %s() Start", __func__);

	spin_lock(&p_work->lock);
	if (p_work->nwm_event == 0) {
		INFO_MSGG("nwm_event = %d", nwm_event);
		spin_unlock(&p_work->lock);
		_fc_nwm_ep0_send_status_stage();

		if (p_work->bShutdownFlag != FALSE) {
			if (p_work->bVBUS_OFF_Flag != FALSE)
				p_work->bVBUS_OFF_Flag = FALSE;
		}

		wait_for_completion(&p_work->thread_wait_comp);
		spin_lock(&p_work->lock);
	}

	_fc_nwm_check_event(p_work);
	spin_unlock(&p_work->lock);

	INFO_MSG("----- %s() End", __func__);
}

/**
 * Wakeup Thread
 */
static void _fc_nwm_wakeup_thread(PT_NWM_WORK p_work)
{
	complete(&p_work->thread_wait_comp);
}

/**
 * NWM Thread
 */
static int _fc_nwm_thread(void *pdata)
{
	int		nret;
	PT_NWM_WORK p_work = gp_nwm_work;

	DBG_MSG("----- %s() Start", __func__);

	daemonize("knwwm_thread");
	current->flags |= PF_NOFREEZE;
	strcpy(current->comm, "knwwm_thread");

	complete(&p_work->thread_comp);

	p_work->nwm_state = FC_NWM_STATE_WAIT_REGIST;

	while (p_work->nwm_state != FC_NWM_STATE_EXIT) {

		switch (p_work->nwm_state) {
		case FC_NWM_STATE_WAIT_REGIST:
			INFO_MSG("FC_NWM_STATE_WAIT_REGIST");
			_fc_nwm_wait_thread(p_work);
			break;

		case FC_NWM_STATE_REGIST_END:
			INFO_MSG("FC_NWM_STATE_REGIST_END");
			nret = _fc_nwm_regist_state(p_work);
			break;

		case FC_NWM_STATE_WAIT_CONFIG:
			INFO_MSG("FC_NWM_STATE_WAIT_CONFIG");
			_fc_nwm_wait_thread(p_work);
			break;

		case FC_NWM_STATE_WAIT_BULK:
			INFO_MSG("FC_NWM_STATE_WAIT_BULK");
			nret = _fc_nwm_wait_bulk_state(p_work);
			if (nret == 0) {
				p_work->bBulk_out_wait = TRUE;
				_fc_nwm_wait_thread(p_work);
			} else {
				INFO_MSG("Make Bulk OUT Packet");
				p_work->nwm_state =
					FC_NWM_STATE_WAIT_CONFIG;
			}
			break;

		case FC_NWM_STATE_RECIVE_BULK:
			INFO_MSG("FC_NWM_STATE_RECIVE_BULK");
			_fc_nwm_recive_bulk_state(p_work);
			break;

		case FC_NWM_STATE_CLASS_LOOP:
			INFO_MSG("FC_NWM_STATE_CLASS_LOOP");
			_fc_nwm_class_loop_state(p_work);
			break;

		case FC_NWM_STATE_RESET:
			INFO_MSG("FC_NWM_STATE_RESET");
			_fc_nwm_reset_state(p_work);
			break;

		case FC_NWM_STATE_DISCONNECT:
			INFO_MSG("FC_NWM_STATE_DISCONNECT");
			_fc_nwm_disconnect_state(p_work);
			break;

		case FC_NWM_STATE_UNREGIST:
			INFO_MSG("FC_NWM_STATE_UNREGIST");
			_fc_nwm_unregist_state(p_work);
			break;

		case FC_NWM_STATE_EXIT:
			INFO_MSG("FC_NWM_STATE_EXIT");
			break;

		default:
			_fc_nwm_wait_thread(p_work);
			break;
		}
	}

	complete(&p_work->thread_comp);	/* exit */
	return 0;
}
EXPORT_SYMBOL(fc_nwm_register_driver);

/**
 * Registration of Class Driver
 */
int fc_nwm_register_driver(struct usb_class_driver *driver)
{
	BOOL	bRegistFlag = FALSE;
	PT_NWM_WORK p_work = gp_nwm_work;
	int		nret = 0;

	DBG_MSG("----- %s() Start", __func__);

	if (p_work->bRunning_flag != FALSE)
		return -EBUSY;

	switch (mod_data.mode) {
	case FC_NWM_MSC_MTP_MODE:
		switch (driver->ClassType) {
		case USB_MSC_TYPE:
			p_work->pMSC_Context = driver;
			DBG_MSG("Regist MSC Driver");
			break;

		case USB_MTP_TYPE:
			p_work->pMTP_Context = driver;
			DBG_MSG("Regist MTP Driver");
			break;

		default:
			nret = -1;
			break;
		}
		break;

	case FC_NWM_MSC_ONLY_MODE:
		if (driver->ClassType == USB_MSC_TYPE) {
			p_work->pMSC_Context = driver;
			DBG_MSG("Regist MSC Driver");
		} else {
			nret = -1;
		}
		break;

	case FC_NWM_MTP_ONLY_MODE:
		if (driver->ClassType == USB_MTP_TYPE) {
			p_work->pMTP_Context = driver;
			DBG_MSG("Regist MTP Driver");
		} else {
			nret = -1;
		}
		break;

	default:
		nret = -1;
		break;
	}

	if (nret == 0) {
		switch (mod_data.mode) {
		case FC_NWM_MSC_MTP_MODE:
			if ((p_work->pMSC_Context != NULL)
					&& (p_work->pMTP_Context != NULL))
				bRegistFlag = TRUE;
			break;

		case FC_NWM_MSC_ONLY_MODE:
			if (p_work->pMSC_Context != NULL)
				bRegistFlag = TRUE;
			break;

		case FC_NWM_MTP_ONLY_MODE:
			if (p_work->pMTP_Context != NULL)
				bRegistFlag = TRUE;
			break;

		default:
			break;
		}
	}

	if (bRegistFlag != FALSE) {
		nret = usb_gadget_register_driver(&g_nwm_driver);
		if (nret != 0) {
			ERR_MSG(" %s()", __func__);
		} else {
			p_work->bRegist_flag = TRUE;
			p_work->nwm_state = FC_NWM_STATE_WAIT_CONFIG;
			_fc_nwm_wakeup_thread(p_work);
		}
	}

	DBG_MSG("----- %s(%d) End", __func__, nret);

	return nret;
}

/**
 * Unregister Class Driver
 */
int fc_nwm_unregister_driver(struct usb_class_driver *driver)
{
	PT_NWM_WORK p_work = gp_nwm_work;
	int		nret = 0;

	DBG_MSG("----- %s() Start", __func__);

	if ((p_work == NULL) || (driver->ClassType > USB_END_TYPE)) {
		ERR_MSG("Not Regist Driver");
		return nret;
	}

	p_work->nwm_state = FC_NWM_STATE_UNREGIST;

	if (p_work->bRegist_flag != FALSE) {
		usb_gadget_unregister_driver(&g_nwm_driver);
		p_work->bRegist_flag = FALSE;
	}

	switch (driver->ClassType) {
	case USB_MSC_TYPE:
		p_work->pMSC_Context = NULL;
		DBG_MSG("Unregist MSC Driver");
		break;

	case USB_MTP_TYPE:
		p_work->pMTP_Context = NULL;
		DBG_MSG("Unregist MTP Driver");
		break;

	default:
		nret = -1;
		break;
	}

	if (nret == 0)
		_fc_nwm_wakeup_thread(p_work);

	DBG_MSG("----- %s(%d) End", __func__, nret);

	return nret;
}
EXPORT_SYMBOL(fc_nwm_unregister_driver);

/**
 * Init
 */
static int __init fc_nwm_init(void)
{
	int		nret;

	DBG_MSG("----- %s() Start", __func__);

	nret = _fc_nwm_alloc();
	if (nret != 0)
		goto nwm_alloc_error;

	nret = kernel_thread(_fc_nwm_thread, NULL,
			(CLONE_VM | CLONE_FS | CLONE_FILES));
	if (nret < 0) {
		ERR_MSG("kernel_thread() return %d", nret);
		goto alloc_thread_error;
	}

	/* Thread start */
	wait_for_completion(&gp_nwm_work->thread_comp);

	/* Mode check */
	switch (mod_data.mode) {
	case FC_NWM_MSC_MTP_MODE:
	case FC_NWM_MSC_ONLY_MODE:
	case FC_NWM_MTP_ONLY_MODE:
		gp_nwm_work->bShutdownFlag = FALSE;
		break;

	case FC_NWM_MSC_ONLY_SD_MODE:
		mod_data.mode = FC_NWM_MSC_ONLY_MODE;
		gp_nwm_work->bShutdownFlag = TRUE;
		break;

	case FC_NWM_MTP_ONLY_SD_MODE:
		mod_data.mode = FC_NWM_MTP_ONLY_MODE;
		gp_nwm_work->bShutdownFlag = TRUE;
		break;

	default:
		mod_data.mode = FC_NWM_MSC_MTP_MODE;
		gp_nwm_work->bShutdownFlag = TRUE;
		break;
	}

	DBG_MSG("----- %s(%d) End", __func__, nret);

	return 0;

alloc_thread_error:
	_fc_nwm_free();

nwm_alloc_error:
	return nret;
}
module_init(fc_nwm_init);

/**
 * Exit
 */
static void __exit fc_nwm_cleanup(void)
{
	PT_NWM_WORK p_work = gp_nwm_work;

	if (p_work == NULL)
		return;

	DBG_MSG("----- %s() Start", __func__);

	if (p_work->bRegist_flag != FALSE)
		usb_gadget_unregister_driver(&g_nwm_driver);

	_fc_nwm_set_configuration(p_work, 0);
	p_work->nwm_state = FC_NWM_STATE_EXIT;
	_fc_nwm_wakeup_thread(p_work);
	wait_for_completion(&p_work->thread_comp);

	_fc_nwm_free();

	DBG_MSG("----- %s() End", __func__);
}
module_exit(fc_nwm_cleanup);



